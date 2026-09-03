# Infrared interface, IRTIM (STM32G0)

> **PROVISIONAL.** Chapter 27 is two pages, three register bits and one
> pad, and all three bits are implemented and bench-measured. What keeps
> the banner is in "Not covered yet" and is all ELECTRICAL or
> BOARD-SHAPED: the high-sink driver's current (the bit is proven to
> stick, its effect needs an LED and a meter), the second IR_OUT pad
> (PA13 is SWDIO on every Nucleo and this project never claims it), and
> the second USART envelope, which is selected and read back but not
> counted on the pad.

Documents of record: RM0444 Rev 6 - IRTIM ch. 27 (figure 278, the
envelope multiplexer's per-part note, the high-sink paragraph) and
SYSCFG 6.1.3 (IR_MOD, IR_POL, I2C_PB9_FMP); DS13560 Rev 5 tables 13 and
15 for the IR_OUT pads. Errata ES0548 Rev 3: NO ITEM TOUCHES IRTIM - a
statement about the document, not a claim about the silicon. Driver:
`stm32g0/irtim.hpp` (`Irtim` monostate + `IrtimPad<PinSel>`); the
per-part envelope instance comes from `stm32g0/device_tables.hpp`
(`irtim_second_usart()`). Bench suite: `test_stm32_serial` letter o
(7 verdicts, wireless). Family fixture `test/family_stm32g0/irtim.cpp`
plus one negative under `tools/check_stm32g0.sh`.

## What the silicon does

**It is an AND gate with a multiplexer in front of it, and it has no
registers of its own.** TIM17 channel 1 is ALWAYS the carrier;
SYSCFG_CFGR1.IR_MOD chooses the envelope among TIM16 channel 1, USART1
and one more USART; IR_POL inverts the product; and that product leaves
the chip on the IR_OUT alternate function. That is the whole chapter,
and it is the whole physical layer of an infrared remote with no CPU in
it at all.

**Neither timer needs a pad.** Figure 278's connections are internal, so
an infrared output costs EXACTLY ONE PIN - and the same is true of the
USART envelope, which is the transmit line inside the chip and not the
TX pad. Measured: TIM17 and TIM16 drove the product onto PB9 with
neither timer's own channel bonded anywhere.

**The three bits live in SYSCFG**, so RCC_APBENR2.SYSCFGEN is what makes
them writable - the same gate the comparators (whose CSRs are inside the
block) and the voltage reference sit behind. `Irtim::init()` opens it
and `release()` leaves it open: it is shared.

**Which USART code 10 means is a PER-PART fact** and the manual states it
twice (ch. 27's own footnote and 6.1.3's IR_MOD description): USART4 on
the STM32G071/G081/G0B1/G0C1, USART2 on the STM32G031/G041/G051/G061.
No header symbol carries it, so it is a stated table in the reserve with
its citation, and the driver publishes it as
`Irtim::second_usart_index` rather than restating it. Code 11 is
Reserved and is refused.

**The high-sink LED driver is PB9's alone and wears an I2C name.** Ch.
27's last paragraph: it "can be activated through the I2C_PB9_FMP bit in
the SYSCFG_CFGR1 register and used to sink the high current needed to
directly control an infrared LED". One bit, two jobs - that is the
silicon's spelling and not brio's, and a program that wants PB9 for I2C
fast-mode-plus and for an infrared LED cannot have it both ways.

## Types and verbs

- `Irtim` - `init()` (the SYSCFG bus clock; idempotent), `bus_clock()`,
  `envelope(IrtimEnvelope)` / `envelope()` (IR_MOD; a Reserved code is
  refused with nothing written), `polarity(bool)` / `polarity()`
  (IR_POL), `pb9_high_sink(bool)` / `pb9_high_sink()` (I2C_PB9_FMP),
  `second_usart_index` (the per-part constant), `release()` (the three
  bits back to reset; the SYSCFG clock stays on because it is shared).
- `IrtimEnvelope { tim16, usart1, second_usart }` +
  `irtim_envelope_valid()`.
- `IrtimPad<PinSel>` - `claim(speed, open_drain)` / `release()`. The AF
  is the DATASHEET's: PB9 is AF0 and PA13 is AF1, and no device header
  has a table to check either against.

## How to use it

```cpp
using Carrier  = brio::TimPwm<brio::Tim<17>, 0>;   // no pad: internal
using Envelope = brio::TimPwm<brio::Tim<16>, 0>;   // no pad: internal
constexpr brio::PinSel ir_out{'B', 9, brio::PinFunction::af0};

brio::Tim<17>::init();
brio::Tim<16>::init();
// ... 38 kHz 50 % on TIM17 CH1, the data envelope on TIM16 CH1 ...

brio::Irtim::init();
(void)brio::Irtim::envelope(brio::IrtimEnvelope::tim16);
brio::Irtim::polarity(false);          // the pad rests LOW: no light at rest
brio::Irtim::pb9_high_sink(true);      // PB9 only
brio::IrtimPad<ir_out>::claim();
```

A USART as the envelope needs no pad on the USART either:

```cpp
(void)brio::Irtim::envelope(brio::IrtimEnvelope::usart1);
brio::Usart<1>::configure({}, brio::usart_brr(SysClock::pclk_hz, 1200).value());
brio::Usart<1>::enable(true);
brio::Usart<1>::write_data(0x00);      // nine bit times of carrier
```

## Bench findings

Wireless. PB9 is the pad (proven free by its own pull first), and the
counter is a DMAMUX request generator on EXTI line 9 driving a DMA
channel - so the carrier's edges are counted with no CPU and no
interrupt.

**The product is exactly the product.** A 38 kHz carrier at 50 % under a
1 kHz 50 % envelope gave **1901 rising edges on PB9 in 100 ms**, where
half of 38000 for a tenth of a second is 1900.

**IR_POL inverts the resting level**, measured with both timers stopped:
the pad rests LOW normally and HIGH inverted - which is what an LED
wired to the supply wants.

**THE FINDING CHAPTER 27 DOES NOT CARRY: the USART envelope is ACTIVE
LOW where TIM16's is active high.** With IR_MOD = 01 an IDLE transmit
line - which is HIGH - passes **0 carrier edges a millisecond**, i.e. it
shuts the gate completely; twenty 0x00 characters at 1200 baud, which
hold the line low for nine of every ten bit times, pass **34 edges a
millisecond** against a free-running carrier's 38, which is nine tenths
of it. That is the right way round for infrared (no light at rest, light
for a zero) and the chapter draws one AND gate and says nothing about
the polarity of its second input.

**The envelope multiplexer** takes all three implemented codes; code 11
is Reserved and `envelope()` refuses it with nothing written.
`second_usart_index` reads 4 on this part.

**The high-sink bit sticks and clears**, which is the whole of what a
board with no infrared LED can say about it.

## Not covered yet

Driver gaps: none - the chapter's three bits and its one pad are all
here.

Implemented, not bench-verified:
- The high-sink driver's EFFECT (I2C_PB9_FMP is proven to stick; what it
  is worth in milliamps needs an LED and a meter).
- IR_MOD = 10, the second USART: selected and read back, never counted
  on the pad. It is USART4 here, and USART4 is a BASIC instance whose
  transmit line is the same signal USART1's was, so nothing new is
  expected - which is why this is a gap and not a doubt.
- The PA13 IR_OUT pad. `IrtimPad<{'A',13,af1}>` compiles and would claim
  it; on this board PA13 is SWDIO and claiming it costs the debug port,
  so it never runs. NEVER DO THIS ON A NUCLEO.
