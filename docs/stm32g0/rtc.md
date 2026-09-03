# RTC and the backup registers - the clock that outlives everything else (STM32G0)

> **PROVISIONAL.** The RTC domain, the calendar, both alarms, the
> periodic wake-up timer, the smooth calibrator, the reference-clock
> detection, the sub-second shift, the five backup registers and the
> TAMPER DETECTION half of chapter 31 are built and bench-verified. What
> is left out is the RTC output pads, which want the one pin an erratum
> makes hostile to the crystal, HSE/32 as a source, and a tamper on a
> controlled EDGE - which is not a driver gap but a measured
> impossibility on this board, arming a tamper input taking the pad away
> from its port. The list is in "Not covered yet".

Documents of record: RM0444 Rev 6 ch. 30 (RTC), ch. 31 (TAMP), 5.2.5 /
5.2.6 / 5.2.12 / 5.4.23 (the RCC's side of the domain) and 4.1.2 (the
PWR's), plus errata ES0548 Rev 3 - **2.9.1 is live on this silicon and
is coded**, 2.2.1 and 2.2.11 are live and are reasons to prefer the
crystal, 2.2.6 is live and unfixable. Driver: `stm32g0/rtc.hpp`
(`RtcDomain`, `Rtc`, `Tamp`). Family fixture:
`test/family_stm32g0/rtc.cpp` + eleven negatives under
`tools/check_stm32g0.sh`. Bench suite: `test_stm32_rtc`.

## What the silicon does

**The RTC lives in a power domain of its own, and that is the whole
point of it.** VBAT supplies it, the LSE crystal and the PC13..PC15 pads
when VDD is gone (4.1.2); a system reset does not reach it (30.3.10);
Stop, Standby and Shutdown do not stop it (table 155). Everything below
follows from that: two locks on the way in, one-way choices once made,
and a counter that is still counting when the program that set it has
been reset three times.

**There are TWO locks, not one, and they have different scopes.**
`PWR_CR1.DBP` gates the whole domain - RCC_BDCR, every RTC register and
every backup register - and is CLEAR after every system reset. On top of
it most RTC registers carry a second lock that only the key sequence
0xCA then 0x53 into `RTC_WPR` opens (30.3.8), and "writing a wrong key
reactivates the write protection". The two are independent: DBP is the
domain's and is cleared by a reset; the WPR lock is the peripheral's and
is "not affected by system reset". Measured both ways, and measured to
have different reach - the backup registers are behind DBP and NOT
behind the key.

**RTCSEL is one-way until the domain is reset.** 5.4.23: "Once the RTC
clock source is selected, it cannot be changed anymore unless the RTC
domain is reset". `RtcDomain::select()` therefore returns a bool and
answers false rather than writing a field the silicon ignores, and
`RtcDomain::reset()` - RCC_BDCR.BDRST - is a separately spelled verb
because of what it costs: the calendar, the prescalers, both alarms, the
wake-up timer, the calibration and THE FIVE BACKUP REGISTERS. Its
sequence is ES0548 2.2.11's own (write 0x0001_0000, read it back so the
pulse is long enough, write 0).

**Initialization mode is a stopped calendar, and entering it twice in a
row is an erratum.** ES0548 2.9.1, live on both silicon revisions: INIT
set between one and two RTCCLK cycles after being cleared sets INITF
immediately instead of waiting for the synchronization, and "a write
occurring during this critical period might result in the corruption of
one or more calendar registers". The workaround is applied
UNCONDITIONALLY on the way OUT - `exit_init()` clears INIT, clears
BYPSHAD if it was set, waits for RSF, and puts BYPSHAD back - so a
second `enter_init()` is safe by construction and no caller has to
remember. It costs one RTCCLK period per exit.

**The calendar is read through shadows, or not at all.** With
BYPSHAD = 0 the three readable registers are copies refreshed every two
RTCCLK cycles, RSF says a copy has landed, and reading SSR or TR LOCKS
the higher-order ones until DR is read - so a coherent reading is always
three reads ending in DR. With BYPSHAD = 1 there is no copy and no
waiting, at the price the chapter states: the values may disagree if an
RTCCLK edge lands between two reads, so "the software must read all the
registers twice". `Rtc::read()` does exactly that, per mode, and is the
only calendar reader this file offers. **The shadow is not updated in
Stop or Standby** (30.3.5), which is why the sleep site sets BYPSHAD.

**Every armed thing has a write window, and the window is a flag.** The
wake-up timer's reload and clock select may be written only with WUTE
clear and WUTWF set; alarm A's registers only with ALRAE clear and
ALRAWF set; both flags take about two RTCCLK cycles to appear. Every
configuring verb disables, waits for its own flag with a bound, writes,
re-enables - and answers false if the flag never came, which on a
stopped RTCCLK is what happens.

**The interrupt is one vector and the EXTI is already open.** Table 61
gives RTC and TAMP one line; table 65 makes EXTI 19 (RTC) and 21 (TAMP)
DIRECT lines - no trigger selection, no pending bit, nothing to clear -
and EXTI_IMR1 comes out of reset with both already unmasked. So 4.3.10's
instruction to "configure the EXTI Line 19 to be sensitive to rising
edge" describes a register bit this device has not got; what a wake
really needs is the RTC's own interrupt enable, the NVIC line, and the
IMR bit that is already there.

**Why RCC_BDCR lives in this file and not in `clock.hpp`.** One
register, one owner, as everywhere in brio - and here the owner is the
RTC domain rather than the clock tree, for three reasons: BDCR is
unreachable without PWR_CR1.DBP, so its access discipline is this
domain's; RTCSEL is one-way, and a clock verb whose consequence can only
be undone by wiping a calendar belongs beside the calendar; and the same
register carries RTCEN, LSE and BDRST, which are this chapter's subject
matter. The day `ClockSource::lse` is built, the clock task will ASK
this type for a running LSE rather than start one behind its back.

## Types and verbs

- `RtcDomain` - the gate and the clock. `pwr_bus_clock`, `apb_clock`
  (RTCAPBEN, the register bank's own enable, distinct from RTCEN),
  `unlock`/`unlocked` (DBP), `bdcr()`, `reset()` (BDRST, and what it
  costs), `lse_enable`/`lse_ready`/`lse_wait_ready`, `lse_drive` (a
  setter that REFUSES a raise under a running crystal, 5.2.5),
  `lse_bypass` (refused unless the oscillator is stopped), `lse_css`
  (one-way, with the chapter's own preconditions enforced),
  `lse_css_failed`, `select`/`selected` (one-way), `enable`/`enabled`
  (RTCEN), `lsco`, and `open(source, wipe)` - 4.1.2's whole sequence in
  one call. Publishes `css_exti_line` (31) and `lse_tim16_ti1_code` (2).
- `RtcClockSource` {none, lse, lsi, hse_div32}, `LseDrive` {low,
  medium_low, medium_high, high}.
- `RtcPrescalers` + `rtc_prescalers_for(hz)` (the CALENDAR's split -
  the asynchronous factor as high as 30.3.4 recommends) and
  `rtc_prescalers_for_resolution(hz)` (the TIMEBASE's - the synchronous
  factor as high as possible, because PREDIV_S is the resolution of
  every sub-second reading); `rtc_ck_spre_hz`, `rtc_ck_apre_hz`,
  `rtc_subsecond_ms`. An impossible rate comes back with `async` 0xFF,
  the `hsidiv_for` convention.
- `RtcDateTime` in ordinary numbers with `rtc_datetime_valid`,
  `rtc_days_in_month` (the one-century leap rule), `rtc_to_bcd` /
  `rtc_from_bcd`, `rtc_time_register` / `rtc_date_register` /
  `rtc_decode`; `RtcReading` = time + sub-second.
- `RtcAlarm` (every field masked by default, so the default alarm is
  "every second") with `rtc_alarm_valid` and the two register makers;
  `RtcAlarmId` {a, b}.
- `RtcWakeupClock` {div16, div8, div4, div2, ck_spre, ck_spre_high} with
  `rtc_wakeup_divider`, `rtc_wakeup_valid` (30.6.6's forbidden
  combination refused) and `rtc_wakeup_clock_hz`.
- `RtcCalibration` + `RtcCalibrationWindow` with `rtc_calibration_valid`
  (30.6.9's two stuck-bit notes as refusals) and
  `rtc_calibration_ppb`.
- `RtcFlag` - one bit per event, the shape RTC_SR, RTC_MISR and RTC_SCR
  all share.
- `Rtc` - `unlock`/`lock`, the raw readbacks, `calendar_set` (INITS),
  `in_init`, `synchronized`, `bypass_shadow` both ways, `enter_init`,
  `exit_init` (with the erratum's workaround) and `exit_init_raw`
  (without it - so the erratum can be STAGED, and so a caller doing its
  own RSF wait need not pay twice), `wait_sync`, `set_prescalers`,
  `set_calendar`, `init(prescalers, time)`, `read`, `time_of_hour_ms`
  and `elapsed_ms`, `shift_hour` / `daylight_flag`, `set_alarm` /
  `clear_alarm` / `alarm_enabled`, `set_wakeup` / `clear_wakeup` /
  `wakeup_write_allowed`, `calibrate` / `calibration` /
  `calr_unprotected` (the one verb that deliberately does not bracket
  itself, so a bench can PROVE the lock), `subsecond()` (RTC_SSR alone -
  the PHASE in one load where `read()` is three plus a retry, which is
  what 30.3.11's synchronization needs and the only reading fine enough
  to catch a calendar update as it happens), `reference_clock` both ways
  (REFCKON, refused outside initialization mode and under any prescaler
  pair but the default one 30.3.12 demands), `shift(add1s, subfs)` (with
  30.3.11's four refusals: a field overflow, a shift already pending,
  REFCKON standing, and SS[15] set), `timestamp_enable` / `timestamp` /
  `timestamp_on_tamper` (TAMPTS) / `timestamp_internal` (ITSE), `flag` /
  `clear_flags`, `wake_line_open`, `isr()`, `debug_freeze`. Publishes
  `exti_line` (19), `irq()` and `wakeup_tim16_ti1_code` (3).
- `TamperFilter` {edge, samples2, samples4, samples8} - the BLOCK's
  detection mode, one FLTCR for every input; `TamperSampling` (RTCCLK
  divided by 32768 down to 256) with `tamper_sampling_divider` and
  `tamper_sampling_hz(s, rtcclk_hz)`, the rate a CALLER'S argument the
  way `rtc_ck_spre_hz` takes one; `TamperPrecharge` {cycles1..cycles8};
  and `TamperTrigger`, whose two values are spelled
  `low_level_or_rising_edge` and `high_level_or_falling_edge` because
  31.6.2 gives the one bit OPPOSITE senses in the two modes.
- `TamperConfig` (the block: filter, sampling, precharge, pullup) and
  `TamperInput` (one TAMP_INx in the manual's 1-based numbering:
  trigger, `erase_backups` DEFAULTING TRUE because that is the
  register's own reset state, `masked`, `interrupt`) with
  `tamper_input_valid(t, filter)` - which refuses the two combinations
  ch. 31 forbids, an interrupt over a masked input and a masked input
  without a filter.
- `TampFlag` with `tamper_flag(index)` and `internal_tamper_flag(y)`,
  the internal masks zero where a part has none.
- `Tamp` - `backup(n)` / `backup(n, v)` over the five words,
  `backup_count` read off the header's own struct, the register
  readbacks (`config1`, `config2`, `filter`, `interrupts`, `status`,
  `masked_status`), `any_armed` (the EXTERNAL half),
  `any_internal_armed` and `erase_source_armed` (the question a program
  keeping something in those five words actually needs answered - the
  reset value arms four internal sources), `clear_flags`,
  `filter_config` (refused while any input is armed, 31.6.1's own
  footnote), `arm` / `disarm` / `armed`, `filter_mode`,
  `internal_tamper(y, on, interrupt)` for ITAMP3..ITAMP6, `flag`,
  `wake_line_open` and `isr()`. Publishes `exti_line` (21), `irq()`,
  `input_count` and `has_internal_tampers`.

## How to use it

```cpp
#include "stm32g0/rtc.hpp"

// The domain is locked at every reset, and RTCSEL is one-way: a board
// whose domain came up on another source must be wiped first, which
// costs the calendar and the backup registers.
if (brio::RtcDomain::selected() != brio::RtcClockSource::lse) {
    brio::RtcDomain::reset();
}
brio::RtcDomain::lse_enable(true);
if (!brio::RtcDomain::lse_wait_ready()) { /* no crystal on this board */ }
brio::RtcDomain::open(brio::RtcClockSource::lse);

brio::Rtc::bypass_shadow(true);        // no RSF dance after a Stop
brio::Rtc::init(brio::rtc_prescalers_for(32768),
                {.hour = 12, .day = 1, .month = 6, .year = 24, .weekday = 6});

// One coherent look at the calendar.
brio::RtcReading now{};
if (brio::Rtc::read(now)) { /* now.time, now.subsecond */ }

// A periodic wake-up every 250 ms, and the vector that answers it.
brio::Rtc::set_wakeup(brio::RtcWakeupClock::div16, 512);
brio::Nvic::enable(brio::Rtc::irq());

extern "C" void RTC_TAMP_IRQHandler() {
    (void)brio::Rtc::isr();
    (void)brio::Tamp::isr();   // one vector, two blocks
}
```

Guarding the backup registers, and arming a tamper over them:

```cpp
// THE QUESTION, and it is not the obvious one: the domain's reset value
// arms four INTERNAL tampers, every one of which erases these words.
if (brio::Tamp::erase_source_armed()) { /* something will wipe them */ }

// A filtered detector on TAMP_IN2 that keeps the backup registers and
// raises the shared vector. The MODE goes first - 31.6.1's footnote -
// and the driver refuses it while anything is armed.
brio::Tamp::filter_config({.filter = brio::TamperFilter::samples4,
                           .sampling = brio::TamperSampling::div256});
brio::Tamp::arm({.index = 2,
                 .trigger = brio::TamperTrigger::high_level_or_falling_edge,
                 .erase_backups = false,
                 .interrupt = true});
(void)brio::Tamp::wake_line_open();
brio::Nvic::enable(brio::Tamp::irq());
```

Correcting the calendar from an outside clock:

```cpp
// REFCKON is bit 4 of RTC_CR, so it needs initialization mode, and
// 30.3.12 demands the default prescaler pair.
if (brio::Rtc::enter_init()) {
    (void)brio::Rtc::reference_clock(true);
    brio::Rtc::exit_init();
}

// ...or a one-off shift of a fraction of a second. This one ADVANCES
// the clock by 1 - 64/256 s in one atomic operation.
(void)brio::Rtc::shift(true, 64);
while (brio::Rtc::shift_pending()) { }
```

## Bench findings

The reference suite is `test_stm32_rtc` (fourteen letters in `z`, 125
verdicts, **125/125 cold and warm**; letter `v` outside it reboots the
board once, and letter `w` outside it spends the five backup registers
and the calendar's value on the erase and the calendar overflow).
NOTHING IS WIRED: TIM16's input multiplexer
reaches LSI, LSE and the RTC's own wake-up signal (25.6.18), so a 64 MHz
capture channel weighs all three against the core clock with no pad.

- **THE NUCLEO'S X2 CRYSTAL IS FITTED AND IT RUNS**, which
  [README.md](README.md) had listed as unverified since the bring-up:
  LSE starts, LSERDY rises, and the period measures **32703 Hz against
  the core** - 1983 ppm from 32768, which is the CORE's own error (HSI16
  is trimmed to 1 %) and not the crystal's. Sixty-four consecutive
  periods span 1952..1968 timer ticks, so the instrument resolves a
  32 kHz period to a handful of core cycles.
- **LSI measures 32586 Hz on this die**, and that CONFIRMS the figure
  `test_stm32_platform` derived from an IWDG time-out (32536 Hz) by a
  completely different route - a period capture against a watchdog's
  coarse reset. The two agree to 1.5 per mille.
- **AN UNFILTERED CAPTURE OF AN INTERNAL CLOCK LINE IS NOT A
  MEASUREMENT, AND ITS ERROR IS TWO-SIDED.** With ICyF = 0 the intervals
  scatter in BOTH directions against a 1965-tick period; with ICyF = 8
  they sit at 1968 with 63 of 64 samples in a narrow band. The two
  settings agree on the PERIOD to 1524..2032 ppm over five runs, so what
  the filter buys is robustness and not a different number - and it is
  needed on the CRYSTAL too, which is why this is a fact about the
  capture path and NOT about ES0548 2.2.1.

  **The two tails are different mechanisms and the second one is not
  obvious.** A missed edge - the capture flag standing while a polling
  loop that also serves a console is elsewhere, the next capture
  overwriting CCR - hands back the distance between edges that are not
  neighbours, and LENGTHENS an interval: seen at up to 62188 ticks on
  the filtered leg. But the BARE leg also OVER-captures, and an extra
  edge SHORTENS one: across five z runs the bare leg's shortest interval
  sat 4, 5, 49, 353 and 754 ticks below the filtered median. So the
  shortest sample is NOT "the period seen from below", and neither a
  mean (dragged up by the long tail) nor a minimum (dragged down by the
  short one) estimates this period. **The MEDIAN does** - it is the one
  statistic both tails have to outnumber - and it is stable to a tick
  run to run where the extremes move by tens of thousands. This is what
  the suite quotes and what its band is anchored on.
- **WHAT TISEL CALLS "RTC WAKE-UP" IS THE MASKED INTERRUPT LINE, NOT A
  PULSE.** It rises with WUTF and stays up until the flag is
  acknowledged, so a capture channel sees exactly one edge per
  acknowledgement - and an acknowledgement that arrives late does not
  shorten an interval, it DELETES the next one. 25.6.18's own footnote
  ("requires to enable the RTC interrupt") is the same fact from the
  other side. Measured through the RTC's own vector, the four divided
  clocks come out exact: RTCCLK/16, /8, /4 and /2 all give 31316..31320
  us for 1024 RTCCLK against 31250 predicted, i.e. 2100 ppm, which is
  again the core's error against the crystal.
- **THE FIRST ASSERTION OF WUTF IS SHORT, exactly as 30.6.6 warns**: a
  ck_spre wake-up of two counts measures 1032 ms on its first period and
  2005 ms on every one after it.
- **THE SMOOTH CALIBRATION DOES NOT REACH THE DIVIDED RTCCLK WAKE-UP
  CLOCKS.** On ck_spre the full swing between CALP and CALM = 511
  measures **1022 ppm** where 30.3.13's range is 975 (-487.1 .. +488.5),
  with CALM lengthening the second and CALP shortening it. The same two
  settings measured on a wake-up period built from RTCCLK/16 give 62657
  ticks BOTH TIMES - a swing of exactly zero. The calibrator masks
  pulses into the PRESCALERS, so the calendar moves and the divided
  wake-up clocks do not; no chapter says so, and an application pacing
  something off RTCCLK/16 should know its period is uncalibrated.
- **An alarm lands AT its match**, not a whole period after it - the
  opposite of the SAM's counter compare: alarm A on a seconds match
  fired with the calendar reading second 5, 0 ms into it, and a
  sub-second alarm at half a second fired 500 ms into its second.
- **The calendar's boundaries are all exact in one second each**:
  midnight, the end of a 30-day month, 28 February into 29 February in a
  leap year and into 1 March in a common one, and the end of a year -
  with the weekday advancing correctly through every one. ADD1H and
  SUB1H move the hour and come back, and **they take effect in the NEXT
  SECOND** (30.6.7's last sentence): a suite that reads straight back
  reports a working register as broken.
- **A shadow re-synchronization is one RTCCLK period** (under a
  millisecond of kernel tick), and 2000 coherent readings cost 10 ms
  through the shadows against 11 ms bypassing them - so on this family
  the bypass mode's double-read is about as cheap as the shadow's single
  one, and what it really buys is the deleted RSF dance after a Stop.
- **ES0548 2.9.1 did NOT reproduce** in sixteen unguarded back-to-back
  initialization entries, against sixteen guarded ones that also did
  not. Recorded as unreproduced, not disproved: it is a race, sixteen
  tries is not a proof, and the workaround costs one RTCCLK period.
- **The two locks have different reach**, measured: a write to RCC_BDCR
  with DBP clear is dropped; a write to a protected RTC register with
  DBP set and no key is dropped; and a write to a BACKUP register lands
  with the RTC's key never written and is dropped with DBP clear. So the
  RTC's WPR does not cover chapter 31's registers.
- **The five backup registers and the calendar survive a real reset**
  (letter `v`, 4/4): all five words come back bit for bit after a
  software reset, RCC_BDCR still names its source, INITS still stands,
  and the calendar has kept counting across the reset AND across the
  reflash that preceded it.
- **This board's domain comes up on LSI with RTCEN set** (RCC_BDCR
  0x8200 on a board that has never been told otherwise), which is what
  [reset.md](reset.md) reported from the other side and what makes
  ES0548 2.2.1's precondition - LSI clocking the RTC - true here by
  default. Moving to the crystal takes a BDRST first, and the suite
  spends one.

- **RTC_REFIN really drags the calendar onto an outside reference**
  (letter `k`). The reference is a GPIO-driven PB15 paced by TIM2, so it
  is the CORE's second, and the calendar's is the crystal's: with
  REFCKON clear a calendar second measures 64131086 TIM2 ticks, and with
  a 50 Hz reference on the pad it measures 64006056 against the
  64000000 that fifty reference periods are by construction. A 60 Hz
  reference gives 64004301 against 63999960 - one detector, both of the
  chapter's mains rates, the same corrected second. And the control is
  the strongest part: **a HALTED reference gives the crystal's second
  back with REFCKON still set** (64126458 ticks), so the correction is
  an EDGE and not a mode, exactly as 30.3.12's "the calendar is updated
  continuously based solely on the LSE clock" says.
- **The correction is quantized at one ck_apre period**, which nothing
  states but the mechanism implies: 30.3.12 reloads the ASYNCHRONOUS
  prescaler, so a corrected second's spread across four measurements is
  about 250000 TIM2 ticks - 3.9 ms, one ck_apre at the default pair -
  where the uncorrected second's spread is 12000.
- **The reference correction can make the sub-second counter SKIP ITS
  ZERO.** A calendar update is normally 0 followed by PREDIV_S; under
  REFCKON the prescaler reload produces its ck_apre edge early, and a
  loop polling RTC_SSR every microsecond steps straight over the zero.
  The symptom is a corrected second measured as exactly twice its
  length. The reliable criterion is an ARRIVAL at PREDIV_S from anywhere
  else, which is the only way the counter can reach its top.
- **RTC_SSR read directly (BYPSHAD set) can be caught mid-transition**,
  and one load is not enough: two consecutive loads that agree are the
  cheapest coherent reading, and the letters that time a second use it.
- **The sub-second shift, timed as a length** (letter `l`): with the
  default prescalers one SUBFS unit is 1/256 s, and a shift of 64 makes
  the one second it lands in exactly a quarter longer (80170757 ticks
  against a base of 64128144, where a quarter is 16032036). ADD1S with
  SUBFS 0 adds a whole second to the calendar and moves no sub-second at
  all; ADD1S with SUBFS 64 does both at once, which is the register's
  only atomic ADVANCE. **SHPF stands for about 47 ms** - 46970 us over
  44161 polls, and 46963 us in another run - which is a great deal
  longer than the "as soon as the shift operation has been executed" of
  30.6.10 suggests to a reader. Writing SUBFS clears RSF, as the same
  paragraph's note says.
- **A shift must not be issued at a calendar boundary.** SUBFS is ADDED
  to a counter whose top is PREDIV_S, so a shift issued just after a
  reload leaves SS above the top and the counter then walks DOWN through
  PREDIV_S on its way - which any "arrived at the top" detector reads as
  an update that has not happened. Issued at half a second it stays
  inside the range. 30.3.11's caution about SS[15] is the same fact one
  register wider.
- **TAMP_CR1 comes out of a domain reset with FOUR INTERNAL TAMPERS
  ARMED** (letter `m`). 31.6.1 gives the register the reset value
  0xFFFF0000 and bits 18..21 of that are ITAMP3E..ITAMP6E - LSE
  monitoring, HSE monitoring, the calendar overflow and the ST
  manufacturer readout - every one of which erases the backup registers
  and none of which has a NOERASE bit. Measured on this board, which had
  never been told anything about tampering. `Tamp::any_armed()` reads
  the EXTERNAL half only and would have answered "none";
  `erase_source_armed()` is the question that matters and it exists
  because of this measurement.
- **ARMING A TAMPER INPUT TAKES THE PAD, pulls and all** - the finding
  that decided how the whole letter is built, and one ch. 31 does not
  carry. A PA0 driven high by its own port reads 1 in IDR before TAMP2E
  is set and 0 the instant it is, with MODER untouched; the internal
  pulls go the same way. So **no program on this board can put an EDGE
  on a tamper input**, and the "the EXTI sees a pad its owner drives"
  technique has no twin here. What is left is a pad the OUTSIDE world
  holds: TAMP_IN1 is PC13, which carries the user button and its
  external pull-up, and over a pad at a standing active level the
  filtered detector starts its sample train when it is ARMED - so the
  latency from the arming to the flag IS the filter's own N/f.
- **The filter and the sampling rate, measured that way**: 2, 4 and 8
  samples at 128 Hz cost 17465 / 35211 / 66523 us against the 15625 /
  31250 / 62500 the counts and the rate predict, each inside one sample
  period; and two samples at 128 / 32 / 8 Hz cost 19464 / 43032 /
  136944 us. The residue in both is the sampler's own phase at the
  moment of arming, which nothing here controls.
- **31.3.4's caution about the edge detector DOES NOT REPRODUCE.** An
  edge detector armed over a pad already at the active level fires in
  NEITHER polarity (half a second each way), so on this silicon
  TAMPFLT = 00 is an edge detector and not a level one - and since no
  edge is reachable, the edge mode's own latency is not measured here.
- **TAMPPUDIS is real, and it is what makes an unheld pad readable
  either way**: a free PA0 with an active-low detector and the precharge
  ON never fires in a second, and with the precharge OFF the floating
  node drifts down and fires in 349215 us. The pull-up is the block's
  and it wins against nothing at all.
- **TAMPxMSK outranks the erase**: a masked detector's flag stays clear
  in TAMP_SR over a detection that fires in milliseconds unmasked, and
  the five backup registers survive it although the same configuration
  ASKED to erase them.
- **A tamper fills the timestamp registers with no RTC_TS pad anywhere
  in it** (TAMPTS), to the second of the running calendar. And **TAMPTS
  costs a FILTERED detector nothing**: 19507 us without it against
  19557 with, because 31.3.4's three latency rows are ALTERNATIVES and
  not a sum - a detector already paying the filter's 3 ck_apre pays no
  second helping.
- **A DETECTED TAMPER ERASES ALL FIVE BACKUP REGISTERS** (letter `w`,
  outside `z`), which is the chapter's default and the reason every
  other letter arms with NOERASE.
- **The calendar's own overflow IS a tamper**: 99-12-31 23:59:59 raises
  ITAMP5F, erases the backup registers like any external one, and
  freezes the calendar where it stopped. **A frozen calendar is not a
  dead one** - writing it in initialization mode starts it again, which
  31.6.1's "the calendar is then frozen and cannot overflow" does not
  say. ITAMP3 (LSE monitoring) over a healthy crystal and ITAMP6 stay
  quiet; ITAMP4 watches an HSE this board does not run and stayed quiet
  too, which is a fact about this board and not about the source.
- **The date and the weekday alarm masks both match** (letter `n`): an
  alarm with every mask clear matches the date as well as the time, the
  same four bits read as a WEEKDAY under WDSEL match too, and a weekday
  that is not today's stays silent for five seconds.
- **LSCOEN does not take a pad an alternate function owns.** 5.2.15 puts
  the LSCO additional function on PA2, which on this board is the
  console's own transmit line; setting LSCOEN with LSE selected, holding
  it for 50 ms and clearing it again leaves the console talking - and
  the console printing the verdict is the witness.
- **PB15 is UCPD1_CC2 and comes out of a power-on holding a Type-C
  dead-battery pull-down.** 7.3.16 and SYSCFG_CFGR1's UCPD1_STROBE say
  so ("upon power on, internal pull-down resistors on UCPD1 CC1 and CC2
  pins are enabled"), and the bench agrees: PB15 does NOT follow its own
  internal pull-up until the strobe releases it, and does afterwards.
  The same is true of PA8, which is CC1 - see [port.md](port.md).

## Not covered yet

Driver gaps (this chapter's option space the stratum does not touch):
- **The RTC_OUT1/OUT2 pads, the calibration output and TAMPALRM**
  (30.3.15, 30.3.16): they want PC13, which ES0548 2.2.6 makes hostile
  to the LSE this stratum prefers, and RTC_CR's OSEL/COE/TAMPOE are the
  three fields left unwritten because of it.
- **RTCSEL = HSE/32**: there is no HSE root in `clock.hpp` yet.
- **The ACTIVE tampers** of ch. 31 - this part declares none (the
  register set is the passive one), so there is nothing to build.

Implemented, not bench-verified:
- **A tamper on a controlled EDGE**, and the reason is measured rather
  than assumed (above): arming a tamper input takes the pad from its
  port, pulls included, so nothing inside this chip can produce one.
  Everything the edge detector is asked here is asked over a STANDING
  level, and 31.3.4's "no latency when TAMPFLT = 0" is therefore
  unmeasured.
- **TAMP_IN3**, which is PE6 on this device and a port this package does
  not bond ([port.md](port.md)'s standing per-package gap), and the
  third input's absence on the G071 and the G031 - compile-checked by
  the family fixture, never run.
- **`RtcDomain::lse_css` ARMED**, and this is a decline with a reason:
  5.4.23 reads the enable as one-way ("cannot be disabled, except after
  a LSE failure detection") while 5.x's own bit description offers a
  0, and the only way back from a wrong guess would be the domain reset
  this bench must not take. The refusal path IS exercised (disarming one
  that never fired is refused), and the arming is not.
- **The internal timestamp** (ITSE), whose one event is the switch to
  the VBAT supply.
- **`Rtc::timestamp_enable`'s own pad**: the RTC_TS route is PC13 and
  the timestamp is reached here through TAMPTS instead.

Declined with the reason, and printed as such by the suite: nothing in
this chapter. Every letter of `z` judges what it measures.
