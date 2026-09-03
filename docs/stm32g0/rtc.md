# RTC and the backup registers - the clock that outlives everything else (STM32G0)

> **PROVISIONAL.** The RTC domain, the calendar, both alarms, the
> periodic wake-up timer, the smooth calibrator and the five backup
> registers are built and bench-verified. What is deliberately left out
> is the TAMPER DETECTION half of chapter 31 - decoded read-only, never
> armed - the reference-clock detection this board has no input for, and
> the RTC output pads, which want the one pin an erratum makes hostile to
> the crystal. The list is in "Not covered yet".

Documents of record: RM0444 Rev 6 ch. 30 (RTC), ch. 31 (TAMP), 5.2.5 /
5.2.6 / 5.2.12 / 5.4.23 (the RCC's side of the domain) and 4.1.2 (the
PWR's), plus errata ES0548 Rev 3 - **2.9.1 is live on this silicon and
is coded**, 2.2.1 and 2.2.11 are live and are reasons to prefer the
crystal, 2.2.6 is live and unfixable. Driver: `stm32g0/rtc.hpp`
(`RtcDomain`, `Rtc`, `Tamp`). Family fixture:
`test/family_stm32g0/rtc.cpp` + eight negatives under
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
  itself, so a bench can PROVE the lock), `timestamp_enable` /
  `timestamp`, `flag` / `clear_flags`, `wake_line_open`, `isr()`,
  `debug_freeze`. Publishes `exti_line` (19), `irq()` and
  `wakeup_tim16_ti1_code` (3).
- `Tamp` - `backup(n)` / `backup(n, v)` over the five words,
  `backup_count` read off the header's own struct, the tamper
  configuration decoded READ-ONLY (`config1`, `config2`, `filter`,
  `interrupts`, `status`, `masked_status`, `any_armed`) and
  `clear_flags` (a standing tamper flag blocks a Standby entry).

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

extern "C" void RTC_TAMP_IRQHandler() { (void)brio::Rtc::isr(); }
```

## Bench findings

The reference suite is `test_stm32_rtc` (ten letters in `z`, 77
verdicts, **77/77 cold and warm, five runs**; letter `v` outside it
reboots the board once). NOTHING IS WIRED: TIM16's input multiplexer
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

## Not covered yet

Driver gaps (this chapter's option space the stratum does not touch):
- **Tamper detection**, the whole of chapter 31 bar the backup
  registers: the pads, the filters, the active tampers and the
  erase-on-tamper. Decoded read-only on purpose - arming a tamper wipes
  the very registers this stratum uses as a breadcrumb, and nothing on
  this desk can drive a TAMP_IN pad.
- **RTC_REFIN reference-clock detection** (30.3.12): a mains input this
  board has not got.
- **The RTC_OUT1/OUT2 pads and the calibration output** (30.3.15,
  30.3.16): they want PC13, which ES0548 2.2.6 makes hostile to the LSE
  this stratum prefers.
- **RTCSEL = HSE/32**: there is no HSE root in `clock.hpp` yet.
- **RTC_SHIFTR**, the sub-second shift of 30.3.11: offered by no verb,
  because its only user is a remote-clock synchronization this stratum
  has no source for.

Implemented, not bench-verified: the timestamp (its event source is a
pad or a tamper and this desk drives neither), `RtcDomain::lse_css` and
its EXTI line 31, `lsco()`, `Tamp::clear_flags`, and the ten-bit and
weekday alarm masks.

Declined with the reason, and printed as such by the suite: nothing in
this chapter. Every letter of `z` judges what it measures.
