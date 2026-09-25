# Clock (CH32X035)

The main clock of the CH32X035 series, where brio's clock model
([../design/clock.md](../design/clock.md)) meets its simplest silicon:
ONE root, the 48 MHz internal RC, and ONE divider between it and every
block on the chip. A static `Clock<internal, hz>` names a rate the root
divides into exactly and is the one truth every driver derives from; a
`DynamicClock` moves the divider at run time with its users rebased
first. `Rcc` beside them is the block itself - the root and its trim,
the divider as it stands, the clock output, the peripheral gates and
resets, and the reset flags.

Documents of record: the CH32X035 reference manual V1.8 (3.2 for the
reset sources, 3.3 and figure 3-2 for the tree, 3.3.2 for the HSI,
3.3.3.4 for the clock output, 3.4.1 to 3.4.9 for the registers, 20.3.1
for the flash's wait states) and the CH32X035/X033 datasheet V1.7
(table 3-9 for the HSI's accuracy and start-up, table 2-1 for the pad
the clock output needs). The header is
[brio/ch32x035/clock.hpp](../../brio/ch32x035/clock.hpp); the reference
suite is `test_x035_clock`.

## What the silicon does

- **ONE ROOT.** The HSI, a 48 MHz RC oscillator, is on out of reset and
  ready (HSION and HSIRDY both 1), and the manual recommends not turning
  it off (3.3.2); SYSCLK IS the HSI - there is no PLL, no crystal
  oscillator, no low-speed RC and no clock switch (3.3.1, 3.3.3.1). Its
  accuracy after the factory calibration is -1.7 to +1.6 per cent over 0
  to 70 C and -2.6 to +2.2 over -40 to 85 C, and it starts in 1.5 to 3.5
  us (datasheet table 3-9).
- **THE CALIBRATION AND THE TRIM.** HSICAL[7:0] is the factory's value,
  loaded at reset and read-only; HSITRIM[4:0] is the user's, SUPERIMPOSED
  on it, 16 at reset - "48MHz +-1%" - at "about 110KHz per step"
  (3.4.1). HSICAL reads 0x53 on the CH32X035F8U6, and the trim takes a
  step each way (measured).
- **ONE DIVIDER.** HCLK is SYSCLK through HPRE[3:0]: codes 0 to 7 divide
  by 1 to 8, codes 8 to 15 by 2, 4, 8, 16, 32, 64, 128 and 256 - the two
  halves overlapping on 2, 4 and 8 - and the reset value is 0101, /6, so
  the chip wakes at **8 MHz** (3.4.2). Every rate the chip can run at is
  48 MHz divided by one of those. The dividers /1 to /6, /8 and /16 are
  measured, each through the low code where two divide alike.
- **NO BUS PRESCALER.** Everything below HB runs at HCLK - the USARTs'
  divisors count HCLK (14.3), the timers and the rest the same - so a
  peripheral's rate is the clock's and nothing else, and `pclk_hz` is
  `hz` - a USART's divisor measured as HCLK / baud at every rung of the
  ladder below. What does not run at HCLK is the tree's other branches
  (figure 3-2): SYSCLK/1024, some 47 kHz, for the auto-wakeup and the
  independent watchdog - whose start forces the HSI on for good
  (3.3.3.3) - the ADC's own divider, the USB clock, and SYSCLK/6 for the
  flash's programming interface.
- **THE FLASH'S WAIT STATES FOLLOW HCLK** (20.3.1): 0 up to 12 MHz, 1
  up to 24 MHz, 2 up to 48 MHz, the fourth code invalid - FLASH_ACTLR
  holds that field and nothing else. The three codes read back as the
  ladder below moves (measured).
- **A NOTE WITH NO BIT.** 3.4.2 closes HPRE's description with "when the
  prescaler factor of the HB clock source is greater than 1, the
  prefetch buffer must be turned on", and no register of the manual has
  a prefetch-buffer bit; WCH's own clock setup divides HCLK and touches
  nothing else. The driver does the same, and on the CH32X035F8U6 every
  divided rung, /2 to /16, ran that way (measured).
- **THE CLOCK OUTPUT** carries SYSCLK or the HSI (CFGR0.MCO = 100 or
  101; every other code, nothing) on **PB9** as an alternate push-pull
  output (3.3.3.4; the datasheet's pin table). PB9 is bonded on the
  LQFP64M, the LQFP48 and the two 28-pin packages, and on neither
  20-pin one.
- **THE GATES.** Three enable registers, one bit a block (3.4.5 to
  3.4.7): RCC_AHBPCENR for the DMA, SRAMEN (the SRAM's clock IN SLEEP,
  not the SRAM's clock), the USB host/device controller and the USB PD
  controller - its reset value 0x00021004, those last three ON
  (measured); the PB2 register for AFIO, the three ports, the ADC, TIM1,
  SPI1 and USART1; the PB1 register for TIM2, TIM3, the window watchdog,
  USART2 to USART4, I2C1 and PWR. The reset registers mirror them
  (3.4.3, 3.4.4), and the HB one holds three lines alone - the USB
  host/device controller's, the PIOC's and the USB PD's (3.4.9): the DMA
  and the SRAM have none.
- **THE RESET FLAGS ACCUMULATE** in RCC_RSTSCKR until RMVF is written
  (3.4.8): LPWRRSTF, WWDGRSTF, IWDGRSTF, SFTRSTF, PORRSTF, PINRSTF and
  OPARSTF. On this silicon RMVF is a LEVEL, not a pulse: written 1 it
  reads back 1 (measured on a CH32X035F8U6, where the CH32V203's clears
  itself), so `clear_reset_flags()` writes it and then clears it,
  leaving nothing standing. The system reset has more sources than flags
  (3.2.2): a core deadlock, an OPA output going high, a USB PD hard reset
  (whose flag, the manual says, is the software reset's) and an ADC
  watchdog reset among them.
  THE MANUAL GIVES TWO RESET VALUES for the register: its register table
  (table 3-1) says 0x0C000000, PORRSTF and PINRSTF, and 3.4.8's own bit
  table gives PINRSTF a reset value of 0. On the QFN20 there is no reset
  pin at all, and there 3.4.8's value is the silicon's (measured).

## Types and verbs

Vocabulary: `ClockSource` (`internal`, the one root), `sysclk_max_hz`
(48 MHz), `hpre_divider(code)` and `hpre_for(src_hz, hz)` (the HPRE
code that divides exactly, the low code where two do, 0xFF where none
does), `Bus` (`hb`, `pb2`, `pb1` - where a gate and a reset line sit),
`McoSource` (`none`, `sysclk`, `hsi`), `clock_timeout_turns` (every
wait for HSIRDY is bounded).

`Rcc`, the block, monostate:

| verbs | what they do |
|---|---|
| `enable(bus, mask)`, `disable(bus, mask)`, `enabled(bus, mask)` | a peripheral's gate, by the `rcc_*` masks of device.hpp |
| `reset(bus, mask)` | a peripheral's reset line pulsed, its gate left alone, true when pulsed; on `hb` the mask is RCC_AHBRSTR's; the power controller's line REFUSED, false and nothing written (bench findings) |
| `hsi_on()`, `hsi_ready()`, `hsi_start()` | the root read, and asked for with a bounded wait (it is on out of reset; a program that stopped it gets it back) |
| `hsi_trim(t)`, `hsi_trim()`, `hsi_calibration()` | the user trim written and read, the factory value read |
| `hpre_code()`, `hpre(code)`, `hclk_hz()` | the divider as it stands, a raw store, and the rate the registers say |
| `mco(src)`, `mco()` | the output multiplexer alone - the pad is `Mco`'s |
| `reset_flags()`, `clear_reset_flags()` | RCC_RSTSCKR's flags read without disturbing them, and RMVF written and cleared again |

`Mco`, the clock output as a task: `has_pad` (PB9 bonded or not, from
the part table), `init(src)` - the pad claimed and the multiplexer set
in one verb, FALSE and nothing done on a package without the pad -
`off()` and `source()`.

`Clock<ClockSource::internal, hz>`, the static clock: `hz` (HCLK),
`pclk_hz` (the same), `sysclk_hz` (48 MHz), `hpre_code`, `is_static`.
A rate that is not 48 MHz divided exactly by an HPRE divider is a
COMPILE error, never a rounded one. `init()` asks for the HSI, raises
the wait states to what the faster of the current and the new rate
needs, moves HPRE, then sets the new rate's own wait states - false,
the tree untouched, if the HSI never reports ready; `restore()` is
`init()` again, for a program coming back from a sleep that stopped
the tree.

`DynamicClock<Boot, Users...>`, the runtime regime - the CH32V00x's
shape: `Boot` names the root undivided (`Clock<internal, 48'000'000>`,
enforced), `set<hz>()` and `set(hz)` name the NEW RATE - a rate no
divider reaches is a compile error or a false - and fan it out to the
`Users` (each a `ClockUser`, checked where the list is written) in list
order, synchronously, BEFORE the divider moves, so a user can drain
what it has in flight at the old rate; the wait states are raised
before a rise and lowered after a fall. `hz()`, `pclk_hz()`,
`can_run_at(hz)`, `rebases<U>`, and the discrete-rate surface a delay
table is indexed by: `rate_count` (16), `rate_hz(i)`, `rate_index()`.
`restore()` puts the CURRENT rate back after a sleep that stopped the
tree, telling the users nothing - their rate never changed.

## How to use it

The static clock, which is most programs:

```cpp
using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

int main() {
    SysClock::init();                     // first: 48 MHz, two wait states
    Serial::init(clock, 115200);          // drivers ask the tag
    brio::Ticker::init(clock);
}
```

`Clock<ClockSource::internal, 8'000'000>` is the reset clock named out
loud; 16 MHz, 9.6 MHz or 187.5 kHz are rates too, and 20 MHz is a
compile error.

A clock that moves, with the ticker and a console as its users:

```cpp
using Boot = brio::Clock<brio::ClockSource::internal, 48'000'000>;
using SysClock = brio::DynamicClock<Boot, brio::Ticker, Serial>;
constexpr SysClock clock;

SysClock::init();                 // 48 MHz
SysClock::set<6'000'000>();       // the ticker and the port rebased, then HPRE /8
```

The clock output, where the package has PB9:

```cpp
if (!brio::Mco::init(brio::McoSource::hsi)) {
    // a 20-pin package: no pad for it
}
```

The reset causes at boot, before anything clears them:

```cpp
const uint32_t flags = brio::Rcc::reset_flags();
brio::Rcc::clear_reset_flags();
const bool watchdog = (flags & (brio::rcc_iwdgrstf | brio::rcc_wwdgrstf)) != 0u;
```

## Bench findings

The reference suite is `test_x035_clock` (21 verdicts in `z`, nothing
wired) on a CH32X035F8U6 - WCH's evaluation board in its QFN20 edition,
over a WCH-LinkE - with the reset flags from `test_x035_platform`'s
letter `a`. What they measured:

- **The tree after `init()`** (letter `a`): RCC_CTLR reads 0x5383 - the
  HSI on and ready, the trim at its centre of 16 and the factory
  calibration HSICAL at 0x53 - CFGR0 reads zero, HPRE at /1 and the
  clock output carrying nothing, and FLASH_ACTLR 0x2, the two wait
  states 48 MHz wants; the console's two gates, USART2's and port A's,
  are open. RCC_AHBPCENR read 0x00021004 at the top of main(), before any
  driver had touched it: SRAMEN and the two USB controllers' gates on out
  of reset, as 3.4.5 gives it.
- **The whole ladder, down and back up** (letter `b`), through a
  `DynamicClock` whose users are the ticker and the console's own port,
  so every line below left the chip through the divisor computed for the
  rate it names:

  | HCLK | HPRE code | wait states | the console's BRR | the rate it gives |
  |---|---|---|---|---|
  | 48 MHz | 0 (/1) | 2 | 417 | 115107 |
  | 24 MHz | 1 (/2) | 1 | 208 | 115384 |
  | 16 MHz | 2 (/3) | 1 | 139 | 115107 |
  | 12 MHz | 3 (/4) | 0 | 104 | 115384 |
  | 9.6 MHz | 4 (/5) | 0 | 83 | 115662 |
  | 8 MHz | 5 (/6) | 0 | 69 | 115942 |
  | 6 MHz | 7 (/8) | 0 | 52 | 115384 |
  | 3 MHz | 11 (/16) | 0 | 26 | 115384 |

  At every rung, on the way down and on the way up, two hundred ticks
  counted exactly two hundred, a 500 us wait was served and the rung's
  line read clean - at 8 MHz with the divisor's own rate 0.64 per cent
  fast. The switch writes nothing of the tree but HPRE and FLASH_ACTLR,
  so every divided rung ran with nothing turned on for the prefetch
  buffer 3.4.2 asks for and no register has. A rate no divider reaches,
  7 MHz, is refused with nothing changed.
- **The trim takes a step each way** (letter `c`): 16 to 17 and then to
  15, each read back and the centre restored after, with a console line
  read clean at each of the two trims.
- **The clock output's multiplexer** (letter `e`): the source field takes
  SYSCLK, the HSI and nothing, and reads each back; `Mco::init()`
  answers false on this package, which does not bond PB9, and `off()`
  leaves the field carrying nothing.
- **Fifteen gates open and close** (letter `f`), each written and read
  back one at a time and left as it was found: DMA1's on HB; AFIO's,
  port B's and C's, ADC1's, TIM1's, SPI1's and USART1's on PB2; TIM2's,
  TIM3's, the window watchdog's, USART3's, USART4's, I2C1's and PWR's on
  PB1. USART1's is among them on a package that offers no USART1 column
  ([usart.md](usart.md)): the gate is the die's, not the package's. The
  five the letter leaves alone are the console's two and the three HB
  gates on out of reset. A reset pulse on I2C1's line leaves its gate
  open, and `reset()` answers false for the power controller's line,
  writing nothing - the next finding is why.
- **A reset pulse on the power controller's line cut the part off its
  debug port.** The gates letter pulsed RCC_APB1PRSTR.PWRRST after every
  gate had opened and closed; the console fell silent before the next
  verdict drained, and the debug port answered "failed to connect" until
  the supply was cycled - on a package with no reset pin, where a hand on
  the supply is the only way back. Observed once and not repeated on
  purpose: the driver refuses that line, and the suite pulses I2C1's
  instead.
- **RMVF is a level** (`test_x035_platform` letter `a`). After a clear
  that wrote RMVF alone, RCC_RSTSCKR read 0x01000000 - the flags
  cleared, the bit standing - where the CH32V203's bit clears itself.
  `clear_reset_flags()` writes it and then clears it, and after that the
  register reads zero.
- **A power-on raises PORRSTF without PINRSTF** (the same letter): the
  flags at the boot after the probe's programming read 0x18000000 -
  SFTRSTF and PORRSTF, a power-on and the probe's resets behind them -
  and no PINRSTF, in both runs. Of the manual's two reset values for
  this register, 3.4.8's bit table is the silicon's on this package,
  which has no reset pin.

## Not covered yet

Driver gaps, each with its reason:

- **Stopping the HSI**: `hsi_on()` reads it and no verb clears HSION -
  it is the one root, the manual advises against it, and a program that
  wants the core stopped wants a sleep, whose chapter is not written
  here ([README.md](README.md)).
- **The branches this file does not set**: the ADC's divider (CLK_DIV,
  the ADC chapter's), the auto-wakeup's and the watchdog's SYSCLK/1024
  (their chapters'), the USB clock (the USB chapters').
- **Closing the USB blocks' gates at boot**: RCC_AHBPCENR has them open
  out of reset and nothing here closes them - born with the USB chapters,
  or with a current measurement that says what closing them saves.
- **A trim loop** that steers HSITRIM against an outside reference (a
  host-timed bracket, the USB's start of frame): born with its first
  user.
- **A reset pulse on the power controller's line**: `reset()` refuses
  it - declined because the one pulse given cut the part off its debug
  port until its supply was cycled (the bench findings). What that pulse
  does to the part, whether to the regulator's setting, to the debug
  module or to both, is a measurement for a supply the bench can cycle
  without a hand.

Implemented but not bench-verified, each with what would measure it:

- **The trim's step in hertz**: letter `c` moves the trim a step each
  way with the console still reading; how many hertz a step is - the
  manual's "about 110KHz" - wants a counter on the clock output, which
  the QFN20 does not bond.
- **The HSI's accuracy against table 3-9**: letter `d` brackets 3000
  ticks at 48 MHz and at the reset clock's 8 MHz and runs to its count
  at both, the tree back at 48 MHz after; what is missing is a host that
  time-stamps the two lines of each bracket as they arrive.
- **The clock output's waveform**: the multiplexer is measured (letter
  `e`); what leaves PB9 wants a package that bonds it and a counter on
  the pad.
- **The five gates letter `f` leaves alone**: port A's and USART2's,
  which the console holds open (letter `a` reads them open), and
  SRAMEN's and the two USB controllers', open out of reset; the three HB
  gates are a letter away, the console's two want a letter that runs
  without the console.
- **The reset flags one event at a time**: the boots measured read
  SFTRSTF and PORRSTF together, a power-on and the probe's resets behind
  them; a boot after a clear, across a re-flash, would say whether the
  probe's reset alone reads as SFTRSTF, as it does on the CH32V203
  ([../ch32vx03/platform.md](../ch32vx03/platform.md)). The watchdogs',
  the low-power and the OPA's flags wait for their chapters, and PINRSTF
  for a package that bonds the reset pin.
