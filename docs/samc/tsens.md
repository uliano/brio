# TSENS - Temperature Sensor (SAM C21)

> **PROVISIONAL.** The whole register description is built and bench-run,
> but the chapter's sleep half (table 43-1) has never been entered, the
> hysteresis window modes have never been made to cross a threshold and
> come back, and no absolute accuracy is claimed - this bench has no
> thermometer. The list is in "Not covered yet".

Documents of record: SAM C20/C21 data sheet DS60001479M ch. 43, table 9-6
(the NVM Temperature Calibration Area), tables 45-37 / 46-9 (the accuracy)
- and errata DS80000740S item **1.19.1, live on every silicon revision
including this one**, which is reproduced below. Driver: `samc/tsens.hpp`.
Family fixture `test/family_samc/tsens.cpp` plus five negatives under
`tools/check_samc.sh`; the bench suite is `test_samc_tsens`.

The chapter is C21-only, and table 1-1's own note adds that TSENS is
absent from the AEC-Q100 qualified part numbers - a marking difference the
device headers do not express, so `tsens_count()` answers only whether
this header declares the block.

## What the silicon does

**This is not an ADC channel, and everything else follows.** On the AVR
the die temperature is one more analog input: a voltage into the SAR, a
SIGROW correction, a reading in kelvin. Here it is a **clock ratio**. A
temperature-dependent oscillator (TOSC) is run twice - once in a "min"
configuration and once in a "max" - and the difference of the two periods,
amplified over GAIN periods of GCLK_TSENS, is counted by a counter clocked
by GCLK_TSENS itself: up during the first phase, down during the second
(43.6.1). What lands in VALUE is

    VALUE = OFFSET + GAIN x (f_TOSCMIN - f_TOSCMAX) / f_GCLK

**So the generic clock is the measurement's RULER, not merely its pace**,
and three things follow that no other peripheral in this stratum has to
worry about:

1. **The factory GAIN and OFFSET belong to one clock** - "the undivided
   internal 48MHz oscillator (OSC48M)" (43.6.1's note). Under that clock,
   and only under it, VALUE is a temperature in **hundredths of a degree
   Celsius**: 43.8.10's own worked example gives 2500 for 25 C and -2500
   for -25 C. That is the unit the whole driver speaks, and it is why
   nothing in it uses a float - a signed 24-bit register already carries
   +-83886 C at a centi-degree a step.
2. **Any other GCLK_TSENS rate rescales the answer**, by exactly
   f_actual / 48 MHz on the GAIN term. Two escapes, both offered as
   constexpr arithmetic taking the rate as a caller argument (the
   `samc/freqm.hpp` `reference_hz` pattern - a ratio meter cannot know
   what its own reference is worth).
3. **And the reference's own error is in the answer.** OSC48M on this die
   measures about 47.76 MHz against the board's crystal - 5000 ppm slow,
   comfortably inside table 45-57 and not a fault - so a reading taken on
   the factory clock is scaled by 48/47.76 in its GAIN term. See "Bench
   findings": the same die read on two references differs by exactly that,
   which is a measurement no thermometer could have made.

**Calibration is the chapter's heart** (43.5.9). Four production values
live in the NVM Temperature Calibration Area at 0x00806030 (table 9-6) and
must be copied in by software: GAIN and OFFSET into their own registers,
TCAL and FCAL into CAL. `samc/nvm.hpp`'s `NvmTemperatureCalibration` has
typed all four since the NVMCTRL pass; `TsensCalibration::factory()` is
the promise that file's comment made.

**And the reset value of GAIN is a trap, not a benign nothing.** GAIN
reads zero out of reset, the field is 24 bits, and a zero GAIN behaves as
**2^24**: measured twice over, the conversion takes 699 ms instead of
3.7 ms and the result is the gain term amplified about two hundredfold,
arriving as a plausible-looking -16000 C. `tsens_config_valid()` refuses a
zero GAIN outright.

**The register disciplines**, spelled per register:

1. **Enable-protected** (43.6.2.1): CTRLC, EVCTRL, WINLT, WINUT, GAIN,
   OFFSET, CAL - and CTRLA.RUNSTDBY, which is a bit and not a register.
   Bench-confirmed by writing each one raw under a running block.
2. **Write-synchronized** (43.6.7): CTRLA.SWRST and CTRLA.ENABLE, and
   nothing else - SYNCBUSY has exactly two bits. 43.6.7 promises that an
   operation needing synchronization issued while its own busy bit stands
   is "discarded and a bus error is generated", and a bus error on a
   Cortex-M0+ is a HardFault, so **every such write in the driver waits on
   the near side of the store and returns bool** (the `samc/sdadc.hpp`
   position, taken for the identical sentence in 39.6.8).
3. **Not PAC-protected** (43.5.8): CTRLB and INTFLAG - which is exactly
   the sentence the silicon does not honour, see "Bench findings".
4. **Not reset by a software reset** (43.8.1): GAIN, OFFSET, CAL and
   DBGCTRL. So a `reset()` keeps the die's calibration.
5. **Write-only**: CTRLB is `__O` in the device header and W in 43.8.2.
   `start()` is a plain store, and this block has **no BUSY bit at all** -
   INTFLAG.RESRDY is the only evidence a measurement finished.

**How wide is VALUE? The chapter says both.** 43.6.4 introduces
INTFLAG.OVF as "the result required more than 16 bits and overflowed the
VALUE register"; 43.8.7's own bit description says twenty-four, and the
register summary, the bit table and `TSENS_VALUE_Msk` all draw VALUE[23:0].
The bench settles it - the rail is at -2^23, located to a few hundred
counts in eight million.

**The window monitor** (43.6.2.4) compares VALUE against WINLT and WINUT,
both signed 24-bit. Six modes plus a Reserved seventh, and **one of them
is described twice and differently**: 43.8.3's table prints OUTSIDE as
"WINUT < VALUE < WINLT" while the device header's enumerator comment says
"VALUE < WINLT or VALUE > WINUT" - not the same condition, and not even
the same threshold order. The bench says which is the silicon's.

**Events and DMA.** One output event (WINMON, generator 30), one input
event action (START, **user 0** - the first row of table 29-3, and the
only START user on this family granted all three propagation paths, where
the DAC's and the SDADC's are asynchronous-only). One DMA request
(RESRDY, trigger 1), set when a result is available and cleared when VALUE
is read. One interrupt vector for all four flags.

## Types and verbs

**The datum.** `tsens_signed(raw)` / `tsens_field(value)` convert between
the 24-bit two's-complement register field and a signed value;
`tsens_value_fits()` bounds it; `tsens_value_min` / `tsens_value_max` are
the rails; `tsens_milli_celsius(centi)` is there for callers whose other
numbers are in thousandths. The unit everywhere is **centi-degrees
Celsius**.

**The clock arithmetic.** `tsens_calibration_gclk_hz` is the 48 MHz the
factory assumed. `tsens_rescale(value, offset, gclk_hz)` scales a READING
taken at another rate back to that one, pivoting on the OFFSET because
only the GAIN term carries the clock. `tsens_gain_for(factory_gain,
gclk_hz)` does the same job on the way in, giving the GAIN to WRITE so
that VALUE keeps arriving in centi-degrees. Both are constexpr, both take
the rate as an argument, and both work in 64 bits on the way through.

**`TsensCalibration`** - `gain`, `offset` (signed), `tcal`, `fcal`;
`factory()` reads them through `samc/nvm.hpp`; `programmed()` is a weak
sanity check (neither zero nor an erased row's all-ones); `cal_word()`
packs CAL.

**`TsensConfig`** - the `calibration`, `free_running` (CTRLC.FREERUN),
`window` + `window_lower` / `window_upper`, `run_standby`, `debug_run` and
`events` (`TsensEventControl`: `start_in`, `invert_start`, `window_out`).

**`tsens_config_valid()`** refuses: the Reserved WINMODE code; a zero GAIN
(which is 2^24); an OFFSET or a threshold outside the 24-bit signed field;
a crossed threshold pair in INSIDE or either hysteresis mode - **and not
in OUTSIDE**, whose two descriptions want opposite orders; and an inverted
event input nothing listens to.

**`Tsens`** - the block, monostate (one instance on every C21 variant, so
there is no index to carry - the `Rtc` / `Dac` / `Sdadc` precedent).

- *Constants*: `gclk_id` (5), `pac_id` (12, published for erratum 1.19.1
  and for the PAC pass that does not exist yet), `window_generator` (30),
  `start_event_user` (0), `dma_trigger_resrdy` (1), the four `flag_*`
  masks, `irq()`.
- *Claim and release*: `init(generator, cfg)` and its compile-time twin
  `init<cfg>(generator)`; `release()`. `init()` writes every
  enable-protected register between the reset and the enable, in that
  order and no other.
- *Plumbing*: `bus_clock`, `clock(generator)`, `sync_busy`, `sync_wait`,
  `reset`, `enable`, `enabled`, `config`.
- *Calibration*: `calibration(c)` (refused while enabled),
  `calibration()`, `offset()`, `gain()`.
- *Measuring*: `start()`, `value()` / `value_raw()`, `read()`,
  `measure()`, `measure_average(n)` - the chapter's own recommendation
  (43.6.2.3: "an average on 10 measurements is recommended", which is also
  the condition table 45-37's accuracy is stated under) - and
  `rescaled(value, gclk_hz)`.
- *Flags*: `flags`, `arm`, `disarm`, `armed`, `clear_flags`,
  `result_ready`, `overrun`, `window_hit`, `overflow_flag`, `overflowed`
  (STATUS.OVF, the level beside the latch), and `isr()`.
- *Window*: `window(mode, lower, upper)` and its readbacks,
  `free_running(bool)`.
- *Events*: `event_config()`, `start_on(channel, cfg, invert)` -
  **refusing no path**, which is where it differs from the identical-
  looking verbs in `dac.hpp` and `sdadc.hpp` - and `stop_events()`.

`read()` and `measure()` return nothing on an overflow as well as on a
timeout: 43.8.8 says an overflowed VALUE "is not valid", and the register
**wraps** rather than saturating, so the number would be plausible and of
the wrong sign.

## How to use it

**A temperature, calibrated, on the clock the factory assumed:**

```cpp
Tsens::init(0, TsensConfig{.calibration = TsensCalibration::factory()});
if (auto centi = Tsens::measure_average(10)) {   // 43.6.2.3
    // *centi is hundredths of a degree Celsius
}
```

**On any other GCLK_TSENS rate**, either scale the GAIN on the way in:

```cpp
TsensConfig cfg{.calibration = TsensCalibration::factory()};
cfg.calibration.gain = tsens_gain_for(cfg.calibration.gain, 24'000'000);
Tsens::init(gen_on_the_crystal, cfg);            // VALUE is still centi-C
```

or leave the factory number alone and scale the reading on the way out:

```cpp
Tsens::init(gen_on_the_crystal,
            TsensConfig{.calibration = TsensCalibration::factory()});
const int32_t centi = Tsens::rescaled(*Tsens::measure(), 24'000'000);
```

**A threshold that costs no CPU** - free-running measurement, the window
monitor's match leaving the block as an event:

```cpp
TsensConfig cfg{.calibration = TsensCalibration::factory()};
cfg.free_running = true;
cfg.window = TsensWindow::above;
cfg.window_lower = 6000;                          // 60.00 C
cfg.events.window_out = true;
Tsens::init(0, cfg);
Evsys::connect(some_user, channel,
               EventChannelConfig{.generator = Tsens::window_generator,
                                  .path = EventPath::asynchronous});
```

**Paced by an event, harvested by the DMAC**, with the CPU in neither
path:

```cpp
Tsens::enable(false);
Tsens::start_on(channel, EventChannelConfig{.generator = pacer_overflow,
                                            .path = EventPath::asynchronous});
Tsens::enable(true);
// DMA channel: trigger = Tsens::dma_trigger_resrdy, source = TSENS_VALUE
```

## Bench findings

From `test_samc_tsens` (10 letters in `z` plus letter `p` by name, 168
verdicts in `z`, 168/168 - three warm runs and two cold from a fresh
flash, about two minutes). **Nothing to wire**, and this chapter could not
need wires in principle: 43.5.1 is "Not applicable".

**No absolute accuracy is claimed anywhere in that suite**, and that is a
design decision rather than an omission: this bench has no independent
thermometer, and table 45-37 allows the sensor -11.3 .. +6.2 C over
[0,60] C to begin with. Every band is preceded by its own noise
measurement, the only statement made about the temperature itself is a
plausibility one, and what cannot be answered is printed and declined.

### This die, and what the factory wrote on it

- The NVM Temperature Calibration Area reads **GAIN 87513, OFFSET 9969,
  TCAL 31, FCAL 44**.
- **The gain term is NEGATIVE at room temperature**: the OFFSET sits
  ABOVE the reading (2580 against 9969, a span of about -7390), so of
  43.6.1's two frequencies f_TOSCMIN is the smaller and the counter's
  second phase outruns its first. The chapter does not say which way it
  points, and every other measurement here is arithmetic on that span.
- Fully calibrated, the die reads about **2580..2640 centi-C (25.8..26.4 C)**
  in this room - **plausible and nothing more**.

### What each part of the calibration is worth

- **CAL.TCAL and CAL.FCAL are worth ten degrees.** With them cleared and
  GAIN/OFFSET loaded, the same die reads **1572 against 2580 centi-C - a
  shift of -10.08 C**, against a combined reading spread of 60. 43.8.15's
  "the value must be copied only, and must not be changed" is not
  decoration.
- **OFFSET is added one for one**: +1000 in the register moves the reading
  by 1009..1022 centi-C.
- **GAIN'S RESET VALUE IS 2^24, NOT ZERO**, and two independent witnesses
  say so. With GAIN, OFFSET and CAL all zero the block still finishes a
  measurement, and (1) the result is **-1606509**, where the CAL-cleared
  gain term (-8397) multiplied by 2^24 / 87513 predicts -1609798 - agreement
  to two parts in a thousand; (2) the conversion takes **699 ms**, where
  2 x 2^24 periods at 48 MHz predicts 699 ms **to the millisecond**. An
  uncalibrated TSENS therefore does not report a benign zero: it waits
  most of a second and hands back what looks like -16000 C.
- A refused `init()` leaves the block exactly as it was - it returns
  before touching a register.

### THE SCALE LEVER: one die, two references, one prediction

The letter this chapter exists for on a bench with no thermometer. FREQM
weighs OSC48M against the board's crystal in the same letter, so the
prediction comes from a measured ppm and not from a memory; a DPLL locked
to the same crystal provides a true 48 MHz; the comparison is
**interleaved A-B-B-A and repeated four times**, because a linear drift of
the die's own temperature cancels exactly out of four equally spaced
batches, and the **median** of the four is reported (the technique
`test_samc_rtc`'s FREQCORR letter had to invent for the same reason).

- **OSC48M measures 47759811 Hz, 5003 ppm slow**; the crystal-locked DPLL
  measures 48000000 Hz, 0 ppm - the crystal's own error cancels out of the
  comparison because both were weighed against the same reference.
- The span from the OFFSET is -7354 centi-C, so 5003 ppm of it **predicts
  a difference of -37 centi-C**.
- **Measured: a median of -35 centi-C** (per-cycle -40, -38, -33, -33),
  with a cycle-to-cycle spread of 7 against a single reading's own spread
  of 42. The interleaving is what makes a 37-centi-degree effect readable
  off a 42-centi-degree instrument.
- **THE ERROR RIDES ON THE SPAN, NOT ON THE TEMPERATURE.** A 5000 ppm
  reference error on a 26 C reading would be 0.13 centi-C if the scale
  applied to the temperature; it is 35, because it applies to the whole
  distance from the OFFSET. Anyone reading this sensor near zero degrees
  is off by far more than the oscillator's ppm suggests.
- **The 1/f law is the silicon's, not an inference.** The same die on the
  bare crystal at 24 MHz with the factory GAIN unchanged reads **-4667
  centi-C** where doubling the span from the OFFSET predicts -4673 - and
  a raw number nowhere near a temperature, which is the trap 43.6.1's note
  warns about in one sentence.
- **Both escapes work.** `tsens_gain_for(87513, 24 MHz) = 43757` puts the
  24 MHz reading back at 2653 against the true-48 MHz 2648 - **5 centi-C**;
  `tsens_rescale(-4667, 24 MHz) = 2651` - **3 centi-C**.

### Time, ruled by the crystal

Timed on a TC0+TC1 32-bit stopwatch running on the board's 24 MHz crystal,
with GCLK_TSENS's own rate measured by FREQM so that crystal ticks become
GCLK periods without the internal RC's error in the way.

- Free-running periods: **88967 / 44993 / 23006 crystal ticks** at GAIN
  87513 / 43756 / 21878, i.e. **3706 / 1874 / 958 us**. The cost is
  proportional to GAIN.
- In GCLK_TSENS periods that is 177046 / 89537 / 45782 against 2 x GAIN =
  175026 / 87513 / 43756, so **a measurement is 2 x GAIN periods plus an
  overhead of 2020 / 2025 / 2026** - a constant to six parts in two
  thousand across a fourfold range. **Chapter 43 gives no conversion time
  at all**, not even a table row.
- A CPU-started single measurement costs 89061 ticks against the
  free-running 88967: the start is on top of the measurement, not instead
  of it. `measure_average(10)` costs 37 ms.

### The datapath's real width

- **A result well past +-32767 stands with no overflow at all**: at GAIN
  x 8 the reading is **-48820** with INTFLAG.OVF and STATUS.OVF both
  clear, and it is eight times the gain term, so GAIN is a linear
  multiplier and not a mode. That alone convicts 43.6.4's "more than 16
  bits".
- **The rail is at -2^23.** Walking OFFSET down in steps of 250 (which
  moves the whole result without touching the measurement), the last
  result that fits is **-8388602** and the first overflow puts the rail at
  **-8388852 +- 250** against -8388608. The positive rail is out of reach
  on this die: the gain term is negative and OFFSET's own field stops at
  +2^23 - 1, and this page says so rather than pretending otherwise.
- **43.8.7 is exact: RESRDY does not set when the conversion overflowed.**
- **An overflowed VALUE WRAPS rather than saturating.** The register held
  0x7FFFE4 = +8388580 where a wrap of the true result predicts +8388565
  and a saturation would give -8388608. So a caller who ignores STATUS.OVF
  gets a plausible number **of the wrong sign** - and `samc/sdadc.hpp`'s
  converter saturates instead, so the two blocks on this die do opposite
  things.
- INTFLAG.OVERRUN sets when a free-running block's results go unread, and
  clears only by writing a one.

### The window monitor

Thresholds placed around the current reading, each mode asked twice - once
arranged to catch it and once to miss it, so "it fired" and "it stayed
silent" are both verdicts.

- ABOVE, BELOW and INSIDE behave as 43.8.3 prints them on a signed result.
- **OUTSIDE is the complement of INSIDE**: with WINLT and WINUT both above
  the reading it fires, with the reading between them it does not, and
  with the thresholds reversed around it it fires. **The device header's
  enumerator comment ("VALUE < WINLT or VALUE > WINUT") is right and
  43.8.3's printed "WINUT < VALUE < WINLT" is not.** The driver refuses a
  crossed threshold pair in every mode where the two documents agree, and
  refuses nothing in this one.
- Both hysteresis modes fire and stay silent as their plain counterparts
  do from a cold start. **The hysteresis itself is not exercised** - it
  needs the die to cross a threshold and come back.
- **Reading VALUE clears WINMON**, exactly as it clears RESRDY (43.8.7).

### The no-CPU chain, and what a "path" costs

TC2 pacing at about 1 kHz through EVSYS into the START user, the DMAC
lifting each result out of VALUE, TC3 counting the window monitor's own
events - and every value the DMAC brings back compared against a reading
the CPU took **at the same GAIN**.

- **The DMAC fills the buffer from VALUE, one beat per measurement**, 16
  of 16, with the CPU in a wait loop; with the pacer stopped, zero beats.
- **Table 29-3 is exact and this user is the exception**: the START user
  takes the asynchronous, synchronous AND resynchronized paths, all three
  at 16 of 16, where the DAC's and the SDADC's take only the first.
- **BUT A SAMPLED PATH SAMPLES, and neither 29.6.2.6 nor table 29-3 says
  what that costs.** A TC's overflow event is **one GCLK_TC period wide** -
  21 ns off a 48 MHz generator, 10.4 us off the crystal-derived 96 kHz
  one - and with the pacer's rate held at 1 kHz and only that width and
  the channel's clock changed:

      pulse     path             channel clock        beats of 16
      21 ns     asynchronous     OSCULP32K                 16
      21 ns     synchronous      48 MHz (the pacer's)      16
      21 ns     synchronous      OSCULP32K                  1
      21 ns     resynchronized   24 MHz crystal             1
      10.4 us   resynchronized   48 MHz                    16
      10.4 us   synchronous      96 kHz (the pacer's)       1

  The asynchronous path is indifferent to both - it has no clock to sample
  with. A sampled path on a channel slower than the generator loses almost
  everything, and **24 MHz against 48 is no better than 32 kHz**. And
  widening the pulse is not the whole answer either: a synchronous channel
  on a 96 kHz generator loses a pulse as wide as its own period, which is
  what one expects when the channel clock is ON-DEMAND - and erratum
  1.12.1 leaves no alternative to that.
- **The window monitor is a generator too**: 50 ms of free-running
  measurement produced 52 WINMON events on TC3, and a window placed out of
  reach produced 0.

### The interrupts, through the one vector

The DAC's and the SDADC's suites both read their flags and never bound
their vectors, so this closes the gap their pages carry; the ADC's did
bind one, through `util/analog_sampler.hpp`.

- One started measurement produces exactly **one** interrupt, with RESRDY
  in the mask `isr()` returns; reading VALUE in the handler clears it, so
  it does not re-enter.
- Ten milliseconds of unread free-running results produce **10 OVERRUN
  interrupts**, and `isr()` acknowledges them - nothing else would.
- A flag with nothing armed sets without interrupting.

### Drift under load - printed and declined

- Baseline 2635 centi-C (spread 45 over 64 readings); then a minute of the
  CPU spinning, the DMAC copying (154335 blocks) and the TSENS measuring
  without pause, sampled every ten seconds: **2624, 2615, 2615, 2624, 2619,
  2640 centi-C**; then 2636 (spread 59).
- First-to-last 16 centi-C, whole-minute range 25, against the baseline
  batch's own spread of 45. **DECLINED**: the trend is inside the
  reading's own spread, so this bench cannot tell self-heating from noise
  and claims nothing. What is claimed is only that a minute of continuous
  conversion under load does not walk the reading away.
- 64 single measurements spread **60 centi-C**; averaging ten narrows that
  to **9**, which is why 43.6.2.3 asks for it.

### Erratum 1.19.1, reproduced - and worse than it says

Letter `p`, **outside `z`** because 11.5.2.4's "the peripheral returns an
access error" might have been a bus error, and a HardFault mid-suite would
have reset the board. It is not: the letter leaves a `.noinit` breadcrumb
before it starts, and the breadcrumb has never been needed.

- **PAC write protection for the TSENS can be set** (PERID 12, STATUSA bit
  12 on bridge A).
- **A PAC-protected write does not fault this core.** The CPU walks
  straight past it.
- **The erratum is reproduced**: with the protection standing, a write to
  CTRLB starts nothing. So 43.5.8's list of the two registers protection
  "does not apply to" is **wrong about CTRLB on this silicon**.
- **And it is worse than the erratum says: no PAC interrupt flag is raised
  either.** INTFLAGA and INTFLAGB both read zero, so 11.5.2.4's "the
  corresponding interrupt flag bit in the INTFLAGn register will be set"
  does not happen - the write is dropped in complete silence, and nothing
  a program can read tells it the measurement never started.
- The errata's second workaround is real: **free-running mode needs no
  CTRLB write at all**.

There is no PAC driver in this stratum, and PAC write protection is off
out of reset (11.5.2.2), so the item is inapplicable by construction today
and becomes a real constraint the day a PAC pass arrives. `Tsens::pac_id`
is the number that pass will need.

### Two things this suite learned the hard way, for other suites

- **TC2 and TC3 share generic clock channel 31** (TC0/TC1 share 30, TC4
  has 32 to itself), so `Tc<2>::release()` silently stops TC3. The first
  version of the no-CPU letter lost the whole window-monitor half of its
  chain that way, with every verdict before it still passing.
- **`tools/bench.py --expect="->"` can truncate a capture.** The judge
  returns as soon as the marker has been seen AND the text ends with the
  prompt `"> "` - and `"  -> "` ends with `"> "`. When the tally lands in
  a later read chunk the run looks like it stopped mid-line. Judging a
  single letter with `--expect="fail"` avoids it; `z` is judged on `ALL:`
  and is unaffected.

**Table 43-1, in standby, and a witness this chapter forces.** Free-
running, the block measured 4 times in a 30 ms window awake, 4 in the
same window spent in STANDBY with CTRLA.RUNSTDBY set, and 0 with it
clear - the table's rows 4 and 2. THE WITNESS HAD TO BE THE WINDOW
MONITOR: unlike every other converter in this stratum the TSENS
publishes NO result-ready event generator (43.6.5 lists WINMON alone),
so a window whose lower limit sits below the datum's rail matches on
every measurement and its event is the count. See
[platform.md](platform.md), "Sleep, peripheral by peripheral".

## Not covered yet

Driver gaps (not built):

- **The block as a WAKE source.** RESRDY, WINMON, OVERRUN and OVF have
  never driven the NVIC out of a sleep; table 43-1's rows are measured
  (see "Bench findings") through an event witness and not an interrupt.
- **A `MeterSource` or sampler adapter.** `util/analog_sampler.hpp`'s
  converter concept wants an unsigned reading and a `void select()`; this
  block has neither a channel to select nor an unsigned datum. Whether the
  util contract should grow to cover it is the same design question
  `samc/sdadc.hpp` left open, and it is not attempted here.

Implemented but not bench-verified:

- **Absolute accuracy, in any form.** There is no thermometer on this
  bench. Everything above is a ratio, a difference, a repeatability figure
  or a plausibility band, and table 45-37's -11.3 .. +6.2 C is untested
  and untestable here.
- **The hysteresis window modes' hysteresis**, which needs the die to
  cross a threshold and come back.
- **The positive overflow rail**, unreachable on this die (the gain term
  is negative and OFFSET's field stops at +2^23 - 1).
- **EVCTRL.STARTINV**, the inverted start event: implemented, and no
  chain here needed a falling edge.
- **43.6.7's bus error.** Every synchronized write in the driver waits
  before storing on the strength of that sentence, and the sentence has
  never been provoked - deliberately, since proving it costs a HardFault.
  If the silicon in fact discards silently, the API is stricter than it
  needs to be.
- **The E and G variants**: compile-checked only. Neither the block nor
  any of its numbers varies by package, and the chapter has no pads.
