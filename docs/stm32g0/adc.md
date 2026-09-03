# ADC (STM32G0)

> **PROVISIONAL.** The whole of chapter 15 is implemented - the
> regulator and the self calibration, both clock schemes, all four
> resolutions and both alignments, the two sampling times with their
> per-channel selector, both faces of the sequencer with their
> handshake, continuous and discontinuous conversion, the eight hardware
> triggers, the DMA request in both its modes, the three analog
> watchdogs, the oversampler and the three internal channels - and all
> of it is bench-verified. What is NOT here is what needs another
> chapter or another board: the low-power modes (WAIT and AUTOFF are
> written and never measured, because there is no PWR driver in this
> stratum), VREF+ as anything but the board's own supply, and the
> external analog inputs, of which this desk drives exactly one. The
> list is in "Not covered yet".

Documents of record: RM0444 Rev 6 ch. 15, with 5.4.13 (RCC_CCIPR.ADCSEL)
and the vector table 12.3 (table 61); DS13560 Rev 5 tables 5, 6, 12, 27,
62, 63, 65 and 66 (the factory calibration values and their addresses,
the pad-to-channel map, fADC and the sampling-time minima); errata
ES0548 Rev 3 items 2.6.1..2.6.5, read on the bench chip's revision Z
column. Driver: `stm32g0/adc.hpp`; the reference vocabulary is
`stm32g0/vref.hpp`'s (see [vref.md](vref.md)), the per-part facts come
from `stm32g0/device_tables.hpp`. Bench suite: `test_stm32_analog`
(12 letters, 94 verdicts, wireless), shared with the DAC, the reference
buffer and the comparators. Family fixture
`test/family_stm32g0/adc.cpp` plus four negatives under
`tools/check_stm32g0.sh`.

## What the silicon does

**One converter, nineteen channels, and no instance question at all.**
Every STM32G0 carries exactly one ADC (15.1), so `Adc` is a MONOSTATE
and not an `Adc<n>` - the samc `Dac`/`Sdadc`/`Tsens` precedent. Sixteen
of the nineteen channels are pads and three are internal: the
temperature sensor on 12, VREFINT on 13 and VBAT/3 on 14 (15.3.8), each
woken by its own bit of ADC_CCR. That register is NOT part of
`ADC_TypeDef`: the device header puts it in its own `ADC_Common_TypeDef`
at its own base, which is why the driver has a `common()` beside
`regs()`.

**It has a regulator and a self calibration, and both are procedures.**
ADVREGEN must be raised and tADCVREG_STUP spent (15.3.2; DS13560 table
62 gives 20 us max) before anything else; ADCAL then measures and
applies an offset correction that varies part to part (15.3.3). Neither
is a factory value copied into a register, which is where this converter
differs from both earlier brio targets - and it is why `init()` takes
the CLOCK: it has real microseconds to spend. Measured on this die:
`CALFACT` comes out **55**, and the whole bring-up (regulator, 25 us,
calibration, configure, enable) fits in one call.

**The factory values are elsewhere, and they are MEASUREMENTS, not
trims.** VREFINT_CAL at 0x1FFF75AA and TS_CAL1/TS_CAL2 at 0x1FFF75A8 and
0x1FFF75CA (DS13560 tables 5 and 6) are ADC RESULTS taken at
VDDA = VREF+ = 3.0 V - the first at 30 C, the others at 30 C and 130 C.
Nothing is written back anywhere: they are the arithmetic's input, which
is why `AdcFactory` is a read-only view and `vdda_mv()` /
`temperature_centi_c()` live in this driver. On the bench chip they read
**VREFINT_CAL 1667, TS_CAL1 1045, TS_CAL2 1394**.

**THE SEQUENCER HAS TWO FACES AND A HANDSHAKE.** With CHSELRMOD clear,
CHSELR is a bitmap of the nineteen channels scanned in numeric order,
forward or backward as CFGR1.SCANDIR says; with it set, CHSELR is eight
4-bit slots scanned in the order written, channels 0..14 only, a 0xF
terminating a short list (15.3.8). Either way **the write is not in
force until ISR.CCRDY rises**, and 15.12.5's own note says an ADSTART
written before that is IGNORED. So every channel verb here clears
CCRDY, writes, and waits - `select()` bounded and silent, because
`util/analog_sampler.hpp`'s converter concept wants a void, and
`select_sync()` with an answer.

**There is no current-channel register.** Nothing in this converter
reports which channel a result was taken on: the sequencer walks a list
software wrote, so `selected()` is the driver's own memory of the last
selection. With a single-channel selection - which is exactly what the
util sampler does - that is exact; inside a multi-channel sequence it is
the sequence's first channel, and a caller walking a sequence labels its
results by position.

**The clock is two choices, not one** (15.3.5). CKMODE picks between the
APB clock divided by 1, 2 or 4 - table 74's deterministic trigger
latency - and an ASYNCHRONOUS root chosen in RCC_CCIPR.ADCSEL (SYSCLK,
PLLPCLK or HSI16) and then divided by ADC_CCR.PRESC, which reaches the
top rate whatever the bus is doing and adds jitter to a hardware
trigger. `init()` refuses PLLPCLK when the PLL's P output is not enabled
(this stratum's `Clock<>` drives R only), refuses PCLK/1 unless the bus
prescalers are in bypass (15.3.5's duty-cycle caution, asked of the
silicon and not of the `Clock<>` type), and refuses any combination
above DS13560 table 62's 35 MHz ceiling.

**The oversampler changes the FULL SCALE, and the full scale is
`util/analog.hpp`'s `steps`.** Up to 256 conversions are accumulated,
shifted right by up to 8 and truncated to the sixteen bits ADC_DR holds
(15.8, table 79). `result_steps()` computes that from the config, which
is what the util arithmetic must be handed - and alignment is IGNORED
while oversampling (15.8.1), so the driver refuses the pair rather than
letting a caller believe in it.

**ADC_ISR is write-1-to-clear**, unlike `TIMx_SR`, which is this
stratum's one inverted reflex (see [tim.md](tim.md)). Reading ADC_DR
also clears EOC (15.4.3), so a caller who wants the flag left standing
reads `flags()` and not `result()`.

## The types and the verbs

`stm32g0/adc.hpp` is the reference. In outline:

- `AdcConfig` + `adc_config_valid()` - the whole configuration as one
  value, with the chapter's refusals as a predicate: a Reserved
  prescaler, continuous AND discontinuous together (15.4.1), an
  oversampling shift past 8, left alignment with oversampling, circular
  DMA without DMA, an SMPSEL bit past the channel count.
- `Adc` - `init(clock, cfg, async_hz)` does the whole bring-up in the
  chapter's order; `regulator_on`/`calibrate`/`enable`/`disable`/
  `configure` are the steps for a caller who wants them apart.
  `configure()` REFUSES while the converter is enabled, which is
  15.3.7's rule and the structural answer to ES0548 2.6.2.
- Channels: `select`/`select_sync`/`select_channel`, `sequence(mask)`,
  `sequence_ordered(order, count)`, `selection()`, `selected()`.
- Conversions: `start`/`stop`/`ready`/`result`/`read`/`read_settled`,
  `converting()`, `overrun()`.
- Arithmetic: `result_steps()`, `sample_steps()`,
  `conversion_half_cycles(ch)`, `vdda_mv(vrefint_data)`,
  `temperature_centi_c(ts_data, vdda_mv)`, and the free
  `adc_clock_hz()`/`adc_conversion_half_cycles()` for a caller sizing a
  pace before anything is running.
- Watchdogs: `watchdog1(low, high, single, channel)`,
  `watchdog2(mask, low, high)`, `watchdog3(...)` - all three
  DISABLED-STATE verbs, see the finding below - and
  `watchdog_thresholds(n, low, high)`, which is live.
- `AnalogIn<Pin, channel>` - a pad on a channel. **The channel number is
  the DATASHEET's and nothing checks it**: no device header of this pack
  carries an analog pin table (`stm32g0/pin.hpp` says that once for the
  stratum), so the caller writes the number exactly as `TimPad` takes an
  AF number, and the bench is the only check there is.
- The DMA request id (table 55 row 5) is published as
  `Adc::dma_request`, and `data_address()` is where a channel reads.

One example, the whole of it:

```cpp
using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
constexpr SysClock clock;
using Sense = brio::AnalogIn<brio::Pin<'A', 4>, 4>;   // DS13560: PA4 is ADC_IN4

constexpr brio::AdcConfig cfg{
    .clock_mode = brio::AdcClockMode::pclk_div2,       // 32 MHz from a 64 MHz bus
    .sample1 = brio::AdcSampleTime::cycles160_5,       // slow source, long window
};

brio::Adc::init(clock, cfg);
brio::Adc::vrefint(true);
Sense::claim();

const uint16_t vref = (brio::Adc::select(brio::AdcInput::vrefint), brio::Adc::read());
const uint16_t vdda = brio::Adc::vdda_mv(vref);        // millivolts, with no meter
const uint16_t mv   = brio::adc_mv((brio::Adc::select(Sense{}), brio::Adc::read()),
                                   brio::Adc::result_steps(), vdda);
```

## Bench findings

Measured by `test_stm32_analog` on an STM32G0B1RE at 3.3 V, silicon
revision Z. The numbers below are one run's; the suite scores 94/94
three times, once cold from a fresh flash.

**VDDA MEASURED WITH NO METER, and it is 3310 mV.** VREFINT converts to
1511 counts against a factory value of 1667 taken at 3.0 V, so 15.9's
ratio puts this board's analog supply at **3310 mV** - and the reading
falls on the right side of the calibration by construction, a supply
above 3.0 V giving fewer counts for the same bandgap. The same reading
in volts is **1221 mV at 12 bits and 1219 mV at 10**, two counts of the
coarse scale apart, which is the full-scale arithmetic checked at two
resolutions; DS13560 table 27 allows 1182..1232 mV.

**The junction temperature is 32.6 C**, from 15.9's two-point formula
with the raw reading first RESCALED from the 3310 mV it was taken at to
the 3.0 V ST's calibration points assume - the step the chapter's
formula leaves out and every application gets wrong once.

**VBAT reads a third of the supply** (1369 counts, 3318 mV after the
divider), which is what the divider in front of that channel does and
what a board tying VBAT to VDD looks like. Its sampling-time minimum is
12 us (DS13560 table 66), which is why the internal-channel letters run
the asynchronous clock at 8 MHz: 160.5 cycles there is 20 us.

**CONVERSION TIME IS EXACT TO THE CPU CYCLE.** With CKMODE = PCLK/2 the
ADC clock is exactly half the CPU clock, so 15.3.9's tSMPL + tSAR is an
integer number of CPU cycles and a DMA channel counting 256 conversions
measures it with no reader in the loop:

| sampling + tSAR | predicted | measured | CPU cycles / 256 conversions |
|---|---|---|---|
| 1.5 + 12.5 (12-bit) | 14.0 | **14.0** | 7278 |
| 3.5 + 12.5 | 16.0 | **16.0** | 8298 |
| 7.5 + 12.5 | 20.0 | **20.0** | 10348 |
| 39.5 + 12.5 | 52.0 | **52.0** | 26728 |
| 7.5 + 6.5 (6-bit) | 14.0 | **14.0** | 7272 |
| 160.5 + 12.5 | 173.0 | **173.0** | 88676 |

Table 76's four tSAR rows and the eight sampling times are therefore
right to the cycle, and so is the resolution's own saving.

**ES0548 2.6.4 measured, and the answer is one cycle.** The erratum says
a sampling time of 1.5 or 3.5 cycles takes one extra cycle on a SINGLE
conversion or the first of a sequence - invisible in the continuous runs
above, where it is amortized over 256. Timed single-shot and
differenced against a 7.5-cycle conversion, where the erratum does not
apply, the step is **13 CPU cycles where 12 is predicted**: the short
conversion pays half a cycle more than the chapter says, which at this
resolution is the erratum's one ADC cycle seen through a clock running
at twice fADC.

**A FORBIDDEN WRITE IS NOT ONE THING ON THIS CONVERTER.** 15.3.7 says
CFGR1 is writable only with ADEN clear and 15.12.13 says the same of
AWD2CR, in the same words - and the silicon does two different things.
Staged with both halves in one letter: **CFGR1 takes an AWD1 bit with
the converter enabled** (which is exactly the door ES0548 2.6.2 comes
through, since the same write resets RES to 12 bits) while **AWD2CR
ignores the identical forbidden write in complete silence**. The first
version of the suite configured every watchdog with the converter
running: AWD1 worked and AWD2/AWD3 did nothing at all, which is how this
was found. Every watchdog verb in the driver now refuses while enabled.

**The watchdogs themselves, all three.** AWD1 guards a window and both
sides of it (a reading inside raises nothing, one above the high
threshold and one below the low one both flag); AWD1SGL narrows it to
one channel and an unguarded conversion then raises nothing; AWD2 and
AWD3 are the same watchdog again with **the channel mask as their own
enable** (15.7.2: a mask of zero turns one off and there is no separate
bit to forget). The interrupt arrives on the vector shared with the
comparators, once per guarded conversion, 4 for 4.

**But the THRESHOLDS are live** (15.7.4): moving the window around a
standing reading while the converter runs stops the flag, with no enable
cycle anywhere. So on this converter a control loop moves its limits
freely and only the CHANNEL selection costs a stop.

**ES0548 2.6.3 staged with a control.** With a two-channel sequence and
AWD1 in single-channel mode on the SECOND channel, the out-of-window
reading raised **nothing**; the same watchdog on the FIRST channel of
the same sequence, equally far outside its own window, flagged. The
erratum reproduces, and the control is what makes that a measurement.

**The oversampler, against a noise floor measured first.** Over 64
conversions VREFINT spans 4 counts and the DAC-driven pad 7 - both a
converter's own noise. With x16 oversampling and a 4-bit shift the pad's
spread falls to **2 counts** with the mean unmoved (2038 both ways), and
with NO shift the accumulator itself lands in ADC_DR: **32616 of a
65536 full scale**, sixteen times the value on sixteen times the scale,
which `util/analog.hpp` converts to the same voltage because its
arithmetic takes the full scale as an argument.

**A DAC, a bond pad and an ADC in series are straight to 2 counts of
4096.** See [dac.md](dac.md) - the transfer curve is measured through
this converter and the nonlinearity is reported as the pair's, not
apportioned.

**The sequencer, both faces.** The bitmap scans in numeric order and
SCANDIR reverses it exactly (1011 / 3062 / 1510 forward, the same three
values back to front); the ordered face walks 13, 5, 4 in that order
with CHSELR reading 0xFFFFF45D, the 0xF terminating a three-entry list;
EOS rises at the end of the sequence and not before; and a channel
selection is refused while a conversion runs.

**`AnalogSampler` runs unchanged on the third architecture.** Letter j
puts `util/analog_sampler.hpp`'s AO inside a real kernel walking
VREFINT, the temperature sensor and a DAC-driven pad: 60 samples,
20 on each input, zero mislabelled, zero queue overflows, and the values
right for their labels. The file's own comment doubted this shape would
survive a converter with a hardware sequencer and DMA - both of which
this one has - and the answer is that it does, because the sampler uses
neither. One method note the letter pays for: `Kernel::step()` serves a
queued event and nothing else, so a pump that is not `Kernel::run()`
must call `TimeEvents<P>::process()` itself or a software pace never
matures.

**One trigger, both converters, no CPU.** TIM6's TRGO is `dac_ch1_trg5`
(table 85) AND the ADC's EXTSEL 101 (table 73), so one basic timer paces
both converters from the same edge while a `DmaLoopEngine` plays a table
into the DAC and a `DmaPingPongEngine` drains ADC_DR. Over six blocks of
24 samples at 5 kHz the captured sequence follows the 16-entry table
with **zero samples off it** and **5 of 5 seams stepping by exactly 8**
(24 mod 16), with no engine overrun and no DAC underrun. Two things had
to be got right and both are findings: the FIRST block is spent, not
judged, because 16.4.8 makes the caller write the first datum before the
first trigger and the launch block is therefore one entry out of phase
(measured exactly that way - entries 0, 7, 15, 7, 15, 7); and **the
sampling time is the delay**, because both converters start on the same
edge and the ADC's sample-and-hold closes at the END of tSMPL, so a
window longer than the DAC's settling time (5 us against 1.7 us typical)
holds the value the DAC has just reached instead of one caught mid-slew.
A 1.2 us window read nine transitional samples in six blocks; a 5 us one
reads none.

## Errata (ES0548 Rev 3, revision Z)

| Item | Applies | Answer |
|---|---|---|
| 2.6.1 overrun flag may stay low | yes | A TIMING obligation on the reader, stated on `overrun()` and on `result()`; no driver can enforce it. The overrun path itself is exercised (a continuous conversion nobody reads raises OVR inside a millisecond) |
| 2.6.2 CFGR1 write with ADEN set resets RES | yes | ANSWERED STRUCTURALLY - `configure()` refuses while enabled, so the combination cannot be spelled. Staged through a named escape and it REPRODUCES: RES goes 10-bit -> 12-bit, and the control (the same word with ADEN clear) keeps it |
| 2.6.3 AWD1 single mode misses a later channel | yes | REPRODUCED with a control (above). Stated on `watchdog1()`, which does not own the sequence and cannot enforce it |
| 2.6.4 sampling time one cycle longer | yes | MEASURED (above). `adc_conversion_half_cycles()` deliberately reports the CHAPTER's number, with the extra cycle named where it is measured |
| 2.6.5 ADC offset out of specification | **no** | Revision A only; this die is revision Z |

## Not covered yet

**Driver gaps** (the register is there, the verb is not):

- The low-power features. WAIT and AUTOFF are in `AdcConfig` and are
  written, and neither is measured: AUTOFF's whole point is the
  power-down between sequences, which wants a PWR driver and a current
  meter, and this stratum has neither.
- Nothing sets ADC_CCR's prescaler independently of `configure()`, so a
  caller cannot re-divide the asynchronous clock without an enable
  cycle. No user has wanted to.

**Implemented but not bench-verified:**

- The external analog inputs, of which this desk drives ONE (PA4, from
  the DAC). Fifteen other channels exist and nothing on this board puts
  a known voltage on any of them.
- The asynchronous clock from SYSCLK or PLLPCLK. HSI16 is what every
  letter uses; the other two roots are written, refused where the PLL's
  P output is dark, and never run.
- Discontinuous mode, triggered oversampling (TOVS) and the
  low-frequency trigger bit: all three are configured and refused
  correctly, none is staged on silicon.
- Seven of the eight hardware triggers. TIM6_TRGO carries the no-CPU
  chain; the other seven are codes in an enum.
- VREF+ at anything but the board's supply - see [vref.md](vref.md),
  where the reason is that the buffer must not be enabled on a board
  whose VREF+ wiring is not known.
