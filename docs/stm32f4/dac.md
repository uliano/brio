# DAC (STM32F4)

Documents of record: RM0090 Rev 22 ch. 14 and RM0390 Rev 6 ch. 14 - the
same converter under one chapter number, and RM0383 Rev 4 has no chapter
for it at all because the F411 has no DAC. The electrical numbers (the
output swing, the settling time, the buffer's headroom) are the
datasheets': DS10693 Rev 11 and the F429 datasheet, DocID024030 Rev 10.
Errata: ES0206 Rev 24 2.6.1 and 2.6.2, ES0298 Rev 8 2.7.1 and 2.7.2, on
every silicon revision - two items about the DMA, both quoted below;
ES0287 files the same two though the part it describes has no DAC.
Driver: `stm32f4/dac.hpp`, with the per-part facts in
`stm32f4/device_tables.hpp`. Bench suite: `test_stm32f4_analog` (one
suite with the ADC: the pad they share is the only route between them).
Family fixture `test/family_stm32f4/analog.cpp` plus a negative under
`brio check stm32f4`.

## What the silicon does

**Two 12-bit channels, and their outputs go to a PAD and nowhere else.**
DAC_OUT1 is PA4 and DAC_OUT2 is PA5 on every part of the family. This
DAC has no MCR and no internal route - 14.3's note connects the pin to
the converter as soon as the channel is enabled, and there is no code
that keeps it off. So the only path from this converter to the ADC is the
bond pad the two share (PA4 is ADC12_IN4, PA5 is ADC12_IN5), which is
what makes a wireless DAC-into-ADC experiment possible on this family at
all - and what makes "configure the pad to analog FIRST" a real
obligation rather than hygiene: a pad still in its reset state (input
floating, its input buffer live) sits across a driven analog node.

**Which parts have one.** The F401, F411 and F412 have no DAC and their
device header declares no `DAC_BASE`, so the resource does not exist
there rather than answering false. How many channels the parts that do
have it really have is the MANUAL's number and not the header's - ST
declares both channels' bits everywhere, the F410's header included,
where RM0401 gives that part one output - so the count is keyed on the
part class like the RTC's tamper inputs, and a class whose manual is not
on the desk gets one channel and the second refused.

**The data register is not the output register.** A write lands in a
holding register and reaches DAC_DORx one APB1 cycle later with no
trigger, three cycles after a hardware trigger, and one with the software
trigger (14.3.4, 14.3.6's note). DOR is read-only. So a caller that
confuses the two measures its own write.

**The buffer is disabled by a ONE.** BOFFx is an output buffer DISABLE
(14.5.1), the opposite polarity of everything around it. A buffered
output drives a load and cannot reach either rail; an unbuffered one
reaches them and cannot drive anything - including, measurably, an LED's
resistor.

**A trigger is an edge, and the wave generators need one.** WAVEx is
"only used if TENx = 1" (14.5.1); TSELx cannot be written with ENx set
and MAMPx cannot be changed once the channel is enabled (14.3.9's note),
which is why configuring drops the enable first and does not put it back.
Table 94's eight triggers are TIM6, TIM8, TIM7, TIM5, TIM2 and TIM4's
TRGO, EXTI line 9, and the software bit - and which of the six timer
codes carry anything is a per-part question the TIMERS answer (the F410
has four timers out of eight).

**The DMA request comes from a hardware trigger and from nothing else** -
14.3.7's own parenthesis, "an external trigger (but not a software
trigger)". And it is NOT QUEUED: a second trigger arriving before the
first request was acknowledged is lost, the channel keeps converting the
old datum, and DMAUDRx says so. Its cell in the fabric is DAC1 on DMA1's
stream 5 and DAC2 on stream 6, channel 7 both.

**What this family's DAC has not**, and the STM32G0's has: an offset
calibration (no DAC_CCR, no CEN, no trim), sample-and-hold (no SHSR /
SHHR / SHRR), and any mode bits at all (no DAC_MCR). Three registers
fewer and one decision fewer. Its reference is the ADC's: 14.3.5's
transfer function is VREF+ x DOR / 4096, and `Ref` lives in
[adc.md](adc.md)'s driver because the pad is shared.

### The errata, and how they are answered

- **DMA request not automatically cleared by clearing DMAEN** (ES0206
  2.6.1, ES0298 2.7.1, ES0321 2.7.1): a request already asserted is not withdrawn by
  clearing DMAENx or by closing the DAC's clock, and it will be served
  the moment the converter is enabled again. The workaround is a sequence
  over TWO peripherals; `stop_dma()` is the DAC's half of it (the flag
  cleared, DMAENx and the channel enable dropped) and says in its own
  comment that the stream's half - disabled and re-initialized before
  anything is enabled again - is still the caller's.
- **DMA underrun flag not set when an internal trigger is detected on
  the clock cycle of the DMA request acknowledge** (ES0206 2.6.2, ES0298
  2.7.2, ES0321 2.7.2): no workaround, and it bites only where software and hardware
  triggers are used together. Stated on `underrun()`.

## Types and verbs

### The reserve's facts (`stm32f4/device_tables.hpp`)

| Fact | Where it comes from |
|------|---------------------|
| whether there is a DAC | `DAC_BASE` |
| its clock and reset bits, and its vector | `RCC_APB1ENR_DACEN`, `RCC_APB1RSTR_DACRST`, `TIM6_DAC_IRQn` - the vector the pack spells that way on exactly the parts that have a DAC |
| how many channels it really has | the REFERENCE MANUAL, keyed on the part class; a class whose manual was not read gets one and the second refused |
| which timers exist, and therefore which trigger codes carry a signal | the timers' base-address macros |
| where each channel's DMA request sits | RM0090 table 43 / RM0390 table 28, keyed on the part class in the same `DmaPlacement` shape the serial instances use |

### Vocabulary

- **`DacTrigger`** - table 94's eight codes by name, `software` among
  them; `dac_trigger_valid()` refuses a code whose timer is absent, and
  `dac_exti_line` names line 9 so no application spells the number.
- **`DacWave`** - `none`, `noise`, `triangle`;
  `dac_wave_amplitude(mamp)` is 2^(mamp+1) - 1 with everything from 11 up
  meaning 4095, and `dac_lfsr_preload` is 14.3.8's 0xAAA.
- **`dac_pad_port()` / `dac_pad_pin()`** - PA4 and PA5, so an application
  need not name them.
- **`DacChannelConfig`** - `buffered` (BOFFx inverted once, here),
  `triggered` + `trigger`, `wave` + `amplitude`, `dma`,
  `underrun_interrupt`. `dac_channel_config_valid()` refuses a wave
  without a trigger, an amplitude past four bits, a DMA without a trigger
  and a DMA on the SOFTWARE trigger.

### The resource: `Dac`

A monostate - a part has one DAC block or none.

- **The block**: `bus_clock()`, `reset()`, `init()` (clock, reset, clock),
  `release()`, `claim_pad<Pin>()` / `release_pad<Pin>()` (analog mode,
  which is not this family's reset state), `channels` and
  `channels_known`, `steps` (4096 always - the 8-bit format is a
  placement, not a resolution), `irq()`.
- **Per channel**: `configure(ch, cfg)` (which leaves the channel
  DISABLED), `config(ch)`, `enable(ch, on)`, `enabled(ch)`, and
  `wave(ch, w)` - the one part of a configured channel 14.3.8 lets a
  program change without disabling anything, and the write that reloads
  the LFSR.
- **Data**: `write()` (12-bit right), `write_left()`, `write8()`,
  `write_dual()`, `write_dual_left()` (whose halves are NOT sixteen
  apart), `write_dual8()`, `code(ch)` (what was asked for), `output(ch)`
  (DAC_DORx - what is being produced, holding register plus the
  generator's own value where one runs), `trigger(ch)`, `trigger_both()`.
- **The DMA**: `data_address_12r/12l/8r(ch)` and
  `data_address_dual_12r()` - the format is the caller's choice and it
  decides the beat width - `engine_placed<E>(ch)` and `dma_placements(ch)`
  for the cell check, and `stop_dma(ch)` for the errata's own sequence.
- **Flags**: `flags()`, `clear_flags()` (write-1-to-clear, the ADC's
  register three away being the opposite), `underrun(ch)`,
  `clear_underrun(ch)`, `interrupts(ch, on)`, and the ISR body `isr()`,
  which answers 0 when the DAC did not speak because the vector is TIM6's
  too.

## How to use it

A DC level on a pad:

```cpp
using Pa4 = brio::Pin<'A', 4>;                   // DAC_OUT1 on every part
brio::Dac::init();
brio::Dac::claim_pad<Pa4>();
brio::Dac::configure(0, brio::DacChannelConfig{});   // buffered, untriggered
brio::Dac::enable(0, true);
brio::delay_us(clock, 20);                       // tWAKEUP
brio::Dac::write(0, brio::dac_code(1650, brio::Dac::steps, 3300));
```

A value held until a trigger says so:

```cpp
brio::DacChannelConfig cfg{};
cfg.triggered = true;
cfg.trigger = brio::DacTrigger::software;
brio::Dac::configure(0, cfg);                    // the enable goes down for the write
brio::Dac::enable(0, true);
brio::Dac::write(0, 0x777);                      // DOR does not move
brio::Dac::trigger(0);                           // now it does
```

Both channels in one store:

```cpp
brio::Dac::write_dual(0x111, 0x222);
brio::Dac::trigger_both();                       // 14.4.6's simultaneous start
```

A triangle on top of a DC level:

```cpp
brio::DacChannelConfig cfg{};
cfg.triggered = true;
cfg.trigger = brio::DacTrigger::software;
cfg.wave = brio::DacWave::triangle;
cfg.amplitude = 9;                               // 2^10 - 1 = 1023 counts of peak
brio::Dac::configure(0, cfg);
brio::Dac::write(0, 1000);                       // the base the triangle rides on
brio::Dac::enable(0, true);
brio::Dac::trigger(0);                           // one step per trigger
```

A table played by hardware, with no CPU in the loop:

```cpp
using DacStream = brio::DmaTxEngine<1, 5, 7, uint32_t>;   // DAC1's own cell
static_assert(brio::Dac::engine_placed<DacStream>(0));
static const uint32_t table[8] = { /* ... */ };

brio::DacChannelConfig cfg{};
cfg.triggered = true;
cfg.trigger = brio::DacTrigger::exti9;           // or a timer's TRGO
cfg.dma = true;
brio::Dac::configure(0, cfg);
brio::Dac::write(0, table[0]);                   // 14.3.7: the FIRST datum is the caller's
brio::Dma<1>::init();
DacStream::arm(brio::Dac::data_address_12r(0));
DacStream::start(&table[1], 7);                  // the tail; the DMA refills behind each trigger
brio::Dac::enable(0, true);
```

## Bench findings

`test_stm32f4_analog`, on an STM32F446 at 180 MHz (the numbers below
unless a part is named), with the DAC's own outputs read back through the
pads they share with the ADC - one pad free on the board and one carrying
a LOAD, which is the board's: on the Nucleo-64 PA4 is free and PA5 carries
the LED; on the STM32F429I-DISC1 PA5 is free and PA4 is the display's
VSYNC, pulled HIGH by the board; on the 32F469IDISCOVERY both pads reach
a header and nothing else, so the buffer's verdict is declined there by
name and every other letter runs.

**The holding register reaches the output by itself, and it is there
before the first read of DOR** - 43 HCLK cycles for a write and a
read-and-compare together, with an APB1 cycle worth 4 of them at 180/45.
With TEN set the same write does not move DOR at all until SWTRIG, and
SWTRIG clears itself.

**The three data formats are one holding register.** 0x80 written 8-bit
reads back 0x800; 0xABC0 written left-aligned reads back 0xABC; a
right-aligned datum is itself. One 32-bit store into the dual register
carries both channels, and one `trigger_both()` moves both outputs.

**The transfer curve, read back through the pad it drives**: monotonic
over the whole range, and within 38 to 47 LSB of the identity between
codes 256 and 3840 - a little over one per cent of full scale for the
DAC's error, the ADC's and their shared reference together. Code 0 reads
34..40 and code 4095 reads 4072..4073 with the buffer on: the buffer
cannot reach either rail, and those two numbers are its own output swing.

**Buffered against unbuffered, measured twice - on a free pad and on the
LED's.** On PA4, with nothing but the ADC's sampling network on it,
buffered reads 37 / 2062 / 4074 at codes 0 / 2048 / 4095 and unbuffered
reads 13 / 2054 / 4095: the unbuffered output reaches both rails and
stays linear in between. On PA5, which carries LD2 and its resistor,
buffered reads 36 / 2053 / 3926 and unbuffered reads 2 / 1994 / 2278 - so
at full scale THE LOAD PULLS THE UNBUFFERED OUTPUT DOWN BY 1648 COUNTS,
1.3 V, while the buffered one holds within 170. At mid-scale, where the
LED barely conducts, the two agree to 59 counts. That is what the buffer is
for, in one pair of numbers. The STM32F429I-DISC1 says the same from the
other side: on its free pad PA5 unbuffered reads 2 / 2049 / 4093 and
buffered 36 / 2051 / 4069, and on PA4 - pulled high by the display's
circuit - unbuffered reads 2906 / 3500 / 4095 while buffered holds
80 / 2005 / 4069: the load lifts the unbuffered output by 2900 counts at
code 0 and the buffer holds it within 45 of the free pad's. The
transfer curve on that board's free pad is within 1.5 per cent too, at a
2.93 V rail.

**The settling of a step.** After a 0 -> 4000 code step, a 15-cycle
conversion started with no delay reads 432..475, at +1 us about 2200, at
+2 us about 3440, at +5 us 4031, and 4040..4041 from +10 us against a
settled 4038..4041. So the pad is within one per cent between 2 and 5 us
after the write.

**The LFSR is deterministic where the chapter says it is.** With the
holding register at zero and MAMP unmasking all twelve bits, the FIRST
trigger puts 0xAAA on the output - 14.3.8's preloaded value - and the
second puts 0xD55, which is figure 98's own number. Clearing WAVEx and
setting it again reloads the register: the next trigger gives 0xAAA once
more. Over 400 steps the pad, read by the ADC, covers about 30 to 4075.

**The triangle's amplitude is exactly what MAMP promises.** At MAMP 7 the
output register sweeps 0..255 above the holding register and comes back;
at MAMP 9, 0..1023.

**A table played by hardware, with no CPU in the loop.** Eight rising
edges driven onto EXTI line 9's pad, a DMA stream on DAC1's own cell
(DMA1 stream 5, channel 7) refilling the holding register behind each
trigger: the pad, read by the ADC after each edge, plays 180, 692, 1198,
1701, 2212, 2720, 3228, 3739 for a table of 200 to 3700 by 500. The
stream reported its own completion after its last beat, and no underrun
happened while it was serving - nor on the ONE trigger past that last
beat, because 14.3.7 wants a SECOND unserved request before it says so.

**ES0206 2.6.1 / ES0298 2.7.1 SEEN ONCE.** A request left pending when a
DMA-to-DAC round is stopped survives the DMAEN clear, the block's reset
and its clock being closed, and is served the moment a stream is armed
again - so a table can start playing one entry ahead of itself with no
trigger having happened. It appeared once, with this letter following the
ADC's DMA letter, and it does not reproduce on demand; the letter now
COUNTS the beats taken before its first edge and compares the table from
wherever the stream really is, which is both the measurement and the only
defence a caller has.

**A SPENT STREAM IS AN UNDERRUN.** Three more edges after the stream's
last beat: DMAUDR is up, and the pad still sits at 3738 - the channel
keeps converting the old datum, exactly as 14.3.7 says. `stop_dma()`
clears the flag and drops both DMAENx and the channel enable, which is
the DAC's half of the errata's stop sequence.

**Every refusal the driver makes was exercised**: a wave generator
without a trigger, a DMA request off the software trigger, an amplitude
past four bits, and a channel the part has not got.

## Not covered yet

Driver gaps:

- **A timer's TRGO as the trigger.** Six of the eight codes are a
  timer's, refused where the timer is absent and otherwise untried:
  they need the timer chapter. What is measured here is EXTI line 9 and
  the software bit.
- **A stream that never stops.** The table above plays once, because
  these engines run a block and end; a waveform that repeats wants a
  CIRCULAR stream (and the double-buffer mode beside it), which the DMA
  chapter's engines do not offer. The DAC's side of that is one bit
  (DMAENx) and is already here.
- **Dual-channel DMA.** `write_dual()` and `data_address_dual_12r()` are
  here and one stream on one channel's request can drive both outputs
  (14.3.7's own paragraph); the suite plays one channel, because the
  second pad on this board carries the LED and the point of the dual
  register is bus bandwidth, which one letter cannot show.
- **The second channel's DMA cell** (DMA1 stream 6): stated by the
  reserve, checked by `engine_placed()`, and not run.
- **The channel count on the F410 and the F413/F423.** RM0401 and RM0430
  are not on the desk, so those parts get one channel and the second is
  refused rather than guessed - and the same two manuals would add their
  DMA rows.

Implemented, not bench-verified:

- **The dual left-aligned and dual 8-bit registers**: written and
  compiled; the right-aligned dual one is what the suite measures.
- **The underrun INTERRUPT.** The flag is raised, read and cleared here;
  the vector is TIM6's too and the suite binds no handler for it.
- **ES0206 2.6.2 / ES0298 2.7.2** (the underrun flag lost when an
  internal trigger lands on the acknowledge cycle): it needs software and
  hardware triggers used together on one channel, which no letter here
  does - and it has no workaround to verify anyway.
