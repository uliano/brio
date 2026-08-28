# ADC - Analog-to-Digital Converter (SAM C21)

> **PROVISIONAL.** Both converters are implemented over the whole
> chapter and bench-verified, and `util/analog_sampler.hpp` runs on top
> of them unchanged. The reference selections and `AdcInput::dac` are
> exercised against the DAC on the same die ([dac.md](dac.md)). What is
> NOT here: VREFA needs a pin nothing drives, and the host/client pair,
> sleep, differential mode and the automatic sequencer are written and
> never exercised. The list is in "Not covered yet".

Documents of record: SAM C20/C21 data sheet DS60001479M ch. 38, the ADC
characteristics of table 45-22, the NVM software calibration area of
table 9-5, and the event tables of ch. 29 - and errata DS80000740S
section 1.4, read on the **E/G/J row at revision F**: items 1.4.4,
1.4.5, 1.4.6, 1.4.9 and 1.4.10 are live and named in the code, while
1.4.1, 1.4.2 and 1.4.3 are revision B only and 1.4.7 and 1.4.8 are
revisions B..E. Driver: `samc/adc.hpp`, over the reserve's ADC entries
in `samc/device_tables.hpp`. The family fixture is
`test/family_samc/adc.cpp` plus nine negatives under
`tools/check_samc.sh`; the bench suite is `test_samc_adc`.

## What the silicon does

**Two converters, and they are not a pair of copies.** ADC0 and ADC1
are the same peripheral at two addresses, but the device header gives
each a role - `ADC0_MASTER_SLAVE_MODE` = 1, `ADC1_MASTER_SLAVE_MODE` =
2 - and the roles are asymmetric in the registers: only ADC1 has a
working `CTRLA.SLAVEEN`, only ADC0's `CTRLC.DUALSEL` means anything.
Each has its own generic clock channel, its own DMA trigger and its own
interrupt vector.

**The two pad maps OVERLAP but are different maps.** PA08 is ADC0/AIN8
*and* ADC1/AIN10; PB08 is ADC0/AIN2 *and* ADC1/AIN4. That is why the
reserve's ADC lookup takes the INSTANCE as a key, the way the TCC's
takes the peripheral function, and why `AnalogIn<Pin>` carries no code:
the code is `Adc<n>::input_code(in)`, resolved by the converter. Package
variation is severe here - **the E bonds no PORT B pad to either
converter at all**, which leaves ADC1 there with AIN10 and AIN11 and
nothing else, and takes AIN2/AIN3 off ADC0.

**Four register disciplines in one chapter, and the mixture is the
trap.** Enable-protected: CTRLB (the prescaler), REFCTRL, EVCTRL,
CALIB. Write-synchronized: CTRLA.SWRST/ENABLE, INPUTCTRL, CTRLC,
AVGCTRL, SAMPCTRL, WINLT, WINUT, GAINCORR, OFFSETCORR, SWTRIG.
**Double-buffered as well as synchronized** (38.6.3.3): all of the
second list but the two CTRLA bits and SWTRIG - so a write made during a
conversion is held until the next RESULT and its SYNCBUSY bit stands for
the whole conversion. A caller spinning on SYNCBUSY after changing the
input mid-stream is waiting out a conversion, not a bus. That is why
`select()` is a plain void store with no wait and `select_sync()` is a
separate verb. Neither discipline: DBGCTRL (which SWRST does not reset
either) and SEQCTRL.

**Accumulation requires the 16BIT resolution code.** 38.6.2.9,
38.6.2.10 and 38.8.11 each say it in a Note: `AVGCTRL.SAMPLENUM` above
one sample needs `CTRLC.RESSEL` = 16BIT, which is not a sixteen-bit
conversion but the statement that RESULT carries an accumulated value up
to sixteen bits wide. `adc_config_valid()` refuses the pairing, at
compile time in the `init<cfg>()` form.

**The two AVGCTRL fields are one arithmetic.** SAMPLENUM sums N
samples; above sixteen the hardware right-shifts automatically to keep
the sum inside the 16-bit register (table 38-1); ADJRES is an
*additional* shift the software asks for, and it is what turns an
accumulation into an average (table 38-2) or into an oversampled
higher-resolution reading (table 38-3). The full scale is therefore
`(base << log2 N) >> (auto + ADJRES)`, which is exactly the `steps`
argument `util/analog.hpp`'s `adc_mv()` wants -
`adc_result_steps()` computes it at compile time and `result_steps()`
at run time. `adc_adjres_for_average()` and
`adc_adjres_for_oversampling()` fill ADJRES from the chapter's own two
tables so nobody copies them by hand.

**The calibration is mandatory and nothing loads it for you.** 38.5.10:
the BIAS and LINEARITY values from production test "must be loaded from
the NVM Software Calibration Area into the ADC Calibration register
(CALIB) by software to achieve specified accuracy... The value must be
copied only, and must not be changed." `samc/nvm.hpp` has typed the
four fields (two per converter) since its own campaign with a comment
promising this; `init()` copies them, always. CALIB is enable-protected,
so `load_calibration()` refuses under a running converter.

**The negative multiplexer is six pads wide where the positive one is
twelve** - MUXNEG reaches AIN0..AIN5 and internal ground and nothing
else (38.8.9).

**The clock has a floor as well as a ceiling.** CLK_ADC = GCLK_ADC /
2^(1 + PRESCALER) with a minimum divider of two, and table 45-22 bounds
f_adc at 160 kHz .. 16 MHz - so **at a 48 MHz generator DIV2 is illegal**
(24 MHz) and DIV4 is the fastest legal setting. `init()` refuses a
prescaler that leaves the range when told the generator's rate. There is
no `rebase()` and no `ClockUser` here: on this family the converter has
its own generic clock channel, so a main-clock change does not move
CLK_ADC and the AVR's fan-out has nothing to fan out to.

**The INTREF channel wants a long sample.** 38.8.9 says MUXPOS = INTREF
requires SAMPCTRL.SAMPLEN "written with a corresponding value" and table
45-22 puts that value at 10 us minimum. `adc_samplen_for_ns()` converts
a wanted sampling time into a SAMPLEN at a given CLK_ADC and
`adc_intref_sampling_ns` is the 10 us; nothing enforces it, because the
driver does not know CLK_ADC at the moment `select()` is called.

**Reading RESULT clears both RESRDY and WINMON** (38.8.7), so the window
verdict has to be captured before the value - `result()` does, and
`window_hit()` reports it for the last value read.

**One interrupt vector for all three sources** (RESRDY, OVERRUN,
WINMON), so `isr()` returns the pending mask and the application
dispatches. It clears OVERRUN and deliberately does not clear the other
two: reading RESULT is what clears them and the value is the point.

## Types and verbs

`Ref` and `ref_mv()` are **this target's** implementation of the
vocabulary `util/analog.hpp` asks every target for. They live in
`adc.hpp` rather than in a separate reference header, because on this
family there is no shared reference block: the ADC's REFSEL, the DAC's
and the SDADC's are three different registers with three different
vocabularies. `Ref::intref` defers to `samc/supc.hpp`'s `VrefLevel` for
the bandgap's actual level; the three supply-derived codes take VDDANA
in millivolts from the caller, because the driver cannot know it.

`Adc<n>` is the resource: `init(generator, cfg, gclk_hz)` and its
compile-time twin `init<cfg>(generator, gclk_hz)`, `enable`, `reset`,
`release`; `select` / `select_sync` / `select_negative` / `selected`;
`start`, `flush`, `ready`, `result`, `read`, `discard`; `window`,
`window_signed`, `window_off`, `window_hit`; `gain_correction`,
`offset_correction`, `correction_enable`; `free_running`; `sequence`,
`sequence_busy`, `sequence_state`; `arm` / `disarm` / `flags` /
`clear_flags` / `isr` / `resrdy`; `event_config`, `start_on`,
`flush_on`, `stop_events`; `load_calibration`, `calibration`;
`sample_steps`, `result_steps`, `result_shift`, `to_mv`. The pad
helpers `claim_pad<P>()` / `release_pad<P>()` apply 38.5.1's recommended
analog pad configuration (function B, digital input off) - **not needed
to read a pad**, whose analog connection is direct, and deliberately not
done by `select()`.

`AnalogIn<Pin>` is the input tag; `AdcInput` names the four non-pad
MUXPOS codes; `AdcNegative` the seven MUXNEG ones. `AdcConfig` carries
the whole configuration and `adc_config_valid(instance, cfg)` is the one
place every rule of the chapter is written down.

The EVSYS and DMAC vocabularies this peripheral publishes -
`resrdy_generator`, `winmon_generator`, `start_event_user`,
`flush_event_user`, `dma_trigger_resrdy` - live here and not in
`evsys.hpp` or `dmac.hpp`, per the ruling in
[evsys.md](evsys.md): those files own the fabric and the channels, a
peripheral owns its own codes.

### One example per use

A single reading in millivolts:

```
using Meter = brio::Adc<0>;
Meter::init(0, brio::AdcConfig{.reference = brio::Ref::vddana,
                               .prescaler = brio::AdcPresc::div32,
                               .sample_length = 5},
            48'000'000);
Meter::select(brio::AnalogIn<brio::Pin<'A', 8>>{});
const uint16_t mv = Meter::to_mv(Meter::read(), 5100);
```

An oversampled 16-bit reading (table 38-3's own row):

```
constexpr brio::AdcConfig cfg{
    .reference = brio::Ref::vddana,
    .prescaler = brio::AdcPresc::div32,
    .resolution = brio::AdcRes::bits16,
    .average = brio::adc_oversampling_average(4),   // 256 samples
    .adjust = brio::adc_adjres_for_oversampling(4), // ADJRES 0
    .sample_length = 5};
static_assert(brio::adc_result_steps(cfg) == 65536);
```

A conversion started by an event and taken away by the DMAC, with no
CPU in the path (the asynchronous channel is not a preference - see the
errata below):

```
Meter::init(0, cfg, 48'000'000);
Meter::enable(false);
Meter::start_on(channel, brio::EventChannelConfig{
                             .generator = brio::Tc<2>::overflow_generator,
                             .path = brio::EventPath::asynchronous});
Meter::enable(true);
// DMA channel triggered by Meter::dma_trigger_resrdy, one beat per
// conversion, source &Meter::regs().ADC_RESULT with no increment.
```

The sampler AO (`util/analog_sampler.hpp`, unchanged from the AVR):

```
using Sampler = brio::AnalogSampler<Meter, brio::SamPlatform, Subs,
                                    brio::AdcInput::scaled_supply,
                                    brio::AnalogIn<brio::Pin<'A', 8>>{}>;
extern "C" void ADC0_Handler() {
    if ((Meter::isr() & Meter::flag_resrdy) != 0) {
        const uint8_t in = Meter::selected();
        brio::post<Sampler>(brio::Sampled{Meter::resrdy(), in});
    }
}
```

## Errata

Live on this silicon (E/G/J at revision F), each with what the driver
does about it:

- **1.4.4 Synchronized Event** - a synchronized event arriving during a
  conversion is never acknowledged and **stalls the event channel**. The
  workaround is to use only the asynchronous path, which table 29-3
  independently requires of both ADC users. **This is code**:
  `start_on()` and `flush_on()` refuse a channel configuration whose
  path is not asynchronous.
- **1.4.5 Software Trigger Sync Busy** - SYNCBUSY.SWTRIG sticks at one
  after a wake from standby, and the item's own instruction is to ignore
  it. So `start()` and `flush()` are plain stores that wait for nothing,
  and RESRDY is the only progress this driver believes.
- **1.4.6 Reference Buffer Offset Compensation** - with REFCOMP set and
  any reference but VDDANA, the first five conversions are out of
  specification. `init()` spends them exactly when that combination is
  configured; `warm_up_conversions()` says how many.
- **1.4.9 Sequence State** - SEQSTATUS is not updated for the first
  conversion of a sequence exiting standby. Stated on
  `sequence_state()`; there is nothing to write.
- **1.4.10 Syncbusy Enable** - enabling ADC1 while ADC0 is disabled can
  leave ADC0.SYNCBUSY.ENABLE stuck at one, and the workaround is
  "enable ADC0 before ADC1 or disregard the bit". **It DOES reproduce
  on this die, and it is worse than the item's sentence** - see the
  findings. `enable()` waits on that bit, so `Adc<0>::init()` FAILS and
  leaves the converter disabled once the state is entered. The
  enumerator's comment is not enough here; the sequencing obligation
  falls on the application.

Not this silicon, and deliberately not coded around: **1.4.1** (START
never clearing), **1.4.2** (the LSB stuck at zero at 8 and 10 bits) and
**1.4.3** (the window monitor keeping GCLK alive) are revision B only;
**1.4.7** (differential and single-ended electrical characteristics) and
**1.4.8** (power consumption) are revisions B..E. The device-level
**1.8.2**, which says GCLK_AC is dead and the AC must borrow GCLK_ADC1's
channel, is revision B only - which is why `ac.hpp` uses AC_GCLK_ID.
**1.8.9** (the DAC output as MUXPOS making both the DAC and the reading
noisy) is live at every revision and is now measured: see
[dac.md](dac.md), where the output half is large and the reading half is
declined, and where the workaround's "external wire" turns out to have
zero length because PA02 is DAC/VOUT and ADC0/AIN0 at once.

## Bench findings

From `test_samc_adc`, 9 letters / 97 verdicts, 97/97 four times
(including one from a fresh flash), wireless, on the C21J at ~5.2 V.

- **THE BANDGAP CHANNEL IS DEAD WITHOUT SUPC.VREF.VREFOE.** MUXPOS =
  INTREF reads a **flat zero** with that bit clear and **795 counts of
  4096** with it set. 22.8.7 words VREFOE as routing the reference "to an
  ADC input channel" and chapter 38 never mentions it at all, so the
  connection between the two chapters is a bench fact rather than a
  documented one. **The REFERENCE path is a different matter and does
  NOT need the bit**: a conversion against REFSEL = INTREF reads 2991
  counts with VREFOE clear and 2990 with it set ([dac.md](dac.md)).
- **VDD LOCATED FROM THE ADC'S SIDE, and it agrees with the AC's.**
  Against VDDANA, INTREF at 1.024 / 2.048 / 4.096 V reads 795 / 1603 /
  3226 counts, which puts VDDANA at **5276 / 5233 / 5201 mV**. The SUPC
  campaign, through the comparator's own 64-step scaler against the same
  three levels, got 5251 / 5141 / 5090 mV ([supc.md](supc.md)). Two
  peripherals sharing no mechanism, agreeing to under 2 %, and both
  sloping the same way with the reference level - which says the slope
  belongs to the bandgap's own level accuracy and not to either
  instrument.
- **The scaled analog supply is a quarter of full scale to four parts in
  a thousand**: 1019 counts of 4096 against VDDANA, with no external
  voltage in the measurement at all. The scaled core supply reads 230
  counts, putting VDDCORE at about **1156 mV**.
- **TWO CONVERTERS, ONE TRUTH.** On the same pad (PA08 = ADC0/AIN8 =
  ADC1/AIN10) at both rails, ADC0 and ADC1 agree **exactly** - 4095 and
  4095, 0 and 0. On the internal quarter-supply divider they differ by
  **5..6 counts of 4096**, one part in a thousand of full scale, about
  8 mV. That is the whole disagreement between the two converters this
  board can produce.
- **CONVERSION TIME IS EXACT TO THE TICK.** Ruled by the board's 24 MHz
  crystal (a TC0+TC1 pair at 3 MHz, GCLK_ADC0 from the same crystal at
  375 kHz so one CLK_ADC cycle is exactly eight stopwatch ticks),
  free-running, measured over 200 results:

  | configuration | predicted | measured |
  |---|---|---|
  | 12-bit, SAMPLEN 0 | 13 cycles / 104 ticks | 104 (34.7 us) |
  | 12-bit, SAMPLEN 20 | 33 / 264 | 264 (88.0 us) |
  | 8-bit, SAMPLEN 0 | 10 / 80 | 80 (26.7 us) |
  | 12-bit, OFFCOMP | 16 / 128 | 128 (42.7 us) |
  | 12-bit, CORREN | 13 / 104 | 104 (34.7 us) |
  | 16x average | 208 / 1664 | 1664 (554.7 us) |

  Zero per mille off in every row. Table 45-22's four cycle-count rows
  are right, the offset compensation's fixed four cycles are real, and
  the accumulation multiplies exactly.
- **THE DIGITAL CORRECTION'S 13 CYCLES ARE NOT WHERE THE CHAPTER PUTS
  THEM.** 38.6.2.14 says the latency is paid once in free-running mode
  and "for each conversion" in single mode. The free-running half is
  confirmed above. The single half is **not observed**: over 200 single
  conversions, CORREN off costs 117 ticks and CORREN on costs **119** -
  two ticks, a quarter of a CLK_ADC cycle, where thirteen cycles would
  be 104. The same at OFFSETCORR 100, so the sentence's odd qualifier
  ("increased by 13 cycles *according to the value in* the Offset
  Correction Value bit group") does not explain it either. The
  correction was proved to be genuinely in the path first: **OFFSETCORR
  100 takes exactly 100 counts off the reading** (1017 -> 917), which is
  38.6.2.14's own subtraction. `adc_conversion_cycles()` keeps charging
  the 13 cycles in single mode deliberately - a pacing prediction that
  is too generous is safe and one that is too tight is not.
- **MODE4 IS THE COMPLEMENT OF MODE3, and the two documents disagreed.**
  38.8.10's table prints MODE4 as "WINUT < RESULT < WINLT" while the
  device header's comment on the same value reads "!(WINLT < RESULT <
  WINUT)". With WINLT = 1000 below WINUT = 3000 the first reading is an
  empty band that can never fire and the second must fire at both rails.
  Measured: **it fires at both rails.** The header is right and the
  register description's table is not. The other three modes behave
  exactly as printed, and the thresholds follow the resolution - at
  8 bits, full scale is 256 and a MODE1 threshold of 128 is the
  mid-point.
- **THE NO-CPU CHAIN RUNS IN BOTH DIRECTIONS AT ONCE.** A TC2 overflow
  crosses an asynchronous event channel into ADC0's START user; each
  conversion's RESRDY pulls one DMA beat out of RESULT; the same RESRDY
  crosses a second channel into TC3, which counts them. 16 of 16 results
  land, every one at the rail the pad holds, at both rails, with 58
  result-ready events counted in the same window. With the pacer stopped
  **nothing moves at all** - the event really is the only thing starting
  a conversion.
- **THE DMA REQUEST IS THE RESRDY FLAG, and clearing the flag is not the
  same as reading RESULT.** 38.6.4 says the request is "cleared when the
  RESULT register is read". A result left standing from a previous run
  moves one stale beat the instant the channel is enabled - caught as a
  suite bug, and the fix is to read RESULT away, not to write the flag.
- **AVERAGING WORKS, AND THE BOARD IS ALMOST TOO QUIET TO SHOW IT.** The
  three internal sources span 1 count (1/4 VDDANA), 4 counts (1/4
  VDDCORE) and 5 counts (INTREF) over 64 single 12-bit readings. On the
  noisiest of them a 64x average spans **0**, on the same mean. The
  suite measures the noise before choosing the measurand and declines
  the reduction verdict in print if even the noisiest source spans fewer
  than four counts - a comparison between spreads of one and zero would
  be a coin toss dressed as a measurement.
- **THE TWO SCALES AGREE THROUGH `util/analog.hpp`.** A 256-sample
  16-bit oversampled reading of the same source is 12714 of 65536, which
  is 794 of 4096 - the 12-bit reading exactly - and both convert to the
  same millivolts through `result_steps()`.
- **`util/analog_sampler.hpp` RUNS UNCHANGED ON THIS SILICON**, which
  was the campaign's point. One second at a 20 ms software pace: 49
  conversion interrupts, 49 `AnalogSample` events received through the
  kernel, 25 on the scaled supply and 24 on the pad, **zero** results
  with an input code outside the list, each value attributed to the
  right input. The file's own comment doubted the shape would survive on
  a target with a hardware sequencer and DMA; the answer is that it
  does, because the sequencer and the DMA are not what the sampler
  uses.
- **ERRATUM 1.4.10 REPRODUCES, AND IT IS WORSE THAN ITS OWN SENTENCE.**
  A narrow probe sees nothing: with ADC1 enabled and ADC0 disabled,
  ADC0.SYNCBUSY reads 0x0000. Running the two converters in earnest is
  what shows it ([dac.md](dac.md)). Once ADC1 has been enabled in a
  power cycle, ADC0.SYNCBUSY.ENABLE is stuck at one and STAYS stuck,
  ADC0 will not enable at all - `Adc<0>::init()` returns false and the
  converter reads zero - and a software reset does not clear it, SWRST's
  own busy bit joining the stuck one. The errata's order is the way out:
  bring ADC1 up FIRST and ADC0 second, after which ADC0 converts and
  keeps converting even when ADC1 is released again.
- **The INTREF sampling rule is real but small here.** At 1.5 MHz,
  INTREF read with SAMPLEN 14 (10.0 us, the table's minimum) gives 795
  counts and with SAMPLEN 0 (0.67 us) gives 791..792 - four or five per
  mille apart. The long sample is the one that agrees with the other two
  witnesses to VDD, so it is the one to use; how far the rule bites at a
  faster CLK_ADC or a higher source impedance is not measured.

## Not covered yet

Driver gaps:
- **The host/client pair** (38.6.3.1): `AdcConfig::client_enable` and
  `AdcConfig::dual` are written, refused on the wrong instance and read
  back, but no letter runs the two converters as a pair. Simultaneous
  and interleaved dual-mode triggering, and the flush/disable/reset
  restart options, are untouched silicon from here.
- **The automatic sequence** (38.6.2.12): `sequence()` writes SEQCTRL
  and `sequence_state()` reads SEQSTATUS, and nothing has ever run a
  sequence. It needs several bonded pads to be worth testing.
- **Sleep**: `run_standby` and `on_demand` are config fields with
  table 38-4 behind them and no owner - the ADC has never been a wake
  source or a sleepwalking peripheral here.
- **VREFA** (`Ref::vrefa`) is encodable and unreachable: the pin is PA03
  and nothing on this board drives it inside table 45-30's range. Needs
  a wire.
- **No temperature reading.** On this family the sensor is the separate
  TSENS peripheral (ch. 43), not an ADC channel; `samc/nvm.hpp` already
  reads its calibration and its driver is a future pass.
- **No rebase / ClockUser**, deliberately: the converter has its own
  generic clock channel and a main-clock change does not move CLK_ADC.

Implemented but not bench-verified:
- **Differential mode.** `DIFFMODE`, the negative multiplexer and the
  signed window thresholds are all written and refused correctly, and
  nothing has converted a differential pair - two pads at two different
  driven levels would only ever give a full-scale or a zero difference,
  which is not a test of anything.
- **Rail-to-rail** (`CTRLC.R2R`) and its offset-compensation
  requirement: the pairing is enforced, the mode never measured.
- **`CTRLC.LEFTADJ`**, written and never read back.
- **The gain correction.** `GAINCORR` is written, range-checked and
  read back; only the OFFSET half was measured against a reading.
- **The FLUSH event input** (`flush_on()`) and `flush()` itself: both
  are exercised for their refusals, neither for its effect.
- **The WINMON and OVERRUN interrupts.** Both flags are read and
  cleared; only RESRDY has ever driven the NVIC.
- **`AdcInput::intref` as a MUXPOS at a fast CLK_ADC**, where table
  45-22's 10 us sampling rule should bite harder than the four per mille
  measured at 1.5 MHz.
- **INL and DNL as this converter's own numbers.** What is measured is
  the transfer of this converter and the DAC IN SERIES - monotonic, with
  a worst residual of about 2.5 counts from a best-fit line - and that
  residual is deliberately not apportioned between the two
  ([dac.md](dac.md)). Separating them needs a source more accurate than
  either, which this board has not got; so does any use of the
  correction registers to cancel the offset and gain errors of table
  45-24.
