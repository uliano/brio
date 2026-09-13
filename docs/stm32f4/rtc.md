# RTC and the backup domain (STM32F4)

Documents of record: RM0383 Rev 4 ch. 17 (the RTC), 5.1.2 and 5.4.1
(the backup domain's access and PWR_CR.DBP), 6.2.8 and 6.3.17 (RTCCLK,
RCC_BDCR), 6.3.3 (RCC_CFGR's RTCPRE), 6.3.18 (the LSI in RCC_CSR),
8.3.15 and its table 25 (what RTC_AF1 does), 10.2.5 and 17.5 (the three
EXTI lines and the three vectors); RM0090 Rev 22 ch. 26 and 8.3.15 for
the F405 class and the F42x/F43x, RM0390 Rev 6 ch. 22 and 7.3.15 for the
F446 - the same peripheral with a second tamper input and a second pad.
Errata: ES0287 Rev 6 items 2.8.1 to 2.8.5, 2.2.10 and 2.2.12 (with the
ES0206 2.9.x / 2.2.x and ES0298 2.9.x / 2.2.x twins). Driver:
`brio/stm32f4/rtc.hpp` (`RtcDomain`, `Rtc`, and the constexpr arithmetic
around them); the reserve's RTC facts are at the end of
`brio/stm32f4/device_tables.hpp`. The family fixture is
`test/family_stm32f4/rtc.cpp` with six negatives that refuse a
sub-second mask past MASKSS's four bits, a wake-up reload past its
sixteen, RTCCLK/2 with a reload of zero, a prescaler past its field, a
calibration value the window's stuck bits would round, and a
twenty-first backup register. The reference suite is
`test_stm32f4_rtc`.

## What the silicon does

**A calendar in a power domain of its own.** Seconds, minutes, hours,
weekday, date, month and year in BCD, plus a binary sub-second counter;
two alarms; a periodic wake-up timer; two calibrators; a timestamp; one
or two tamper inputs; and twenty 32-bit backup registers. All of it
runs from RTCCLK - LSE, LSI or the HSE divided by RTCPRE - and NONE of
it is in a system reset's scope (17.3.7): a reset clears the three
shadow registers and three bits of RTC_ISR, and nothing else. That is
the whole point of the block, and it is why INITS exists: the one flag
that tells a fresh boot whether the calendar it inherited means
anything.

**Two locks, and they cover different things.** PWR_CR.DBP gates the
domain - RCC_BDCR, every RTC register and every backup register - and is
clear after every system reset; on top of it most RTC registers carry a
second lock that only 0xCA then 0x53 into RTC_WPR opens, and that one is
"not affected by system reset". Three things sit outside the key and
inside DBP alone: RTC_ISR[13:8] (the six event flags), RTC_TAFCR and
RTC_BKPxR - measured, letter `a`. Writing DBP wants a read back before
its effect can be relied on (5.4.1's own note; ES0206 2.2.9 says the
delay grows with the APB1 prescaler), which `unlock()` does.

**RTCSEL is one-way and BDRST is the way back.** 6.2.8: "once the RTCCLK
clock source has been selected, the only possible way of modifying the
selection is to reset the power domain". A domain reset stops the RTC
and wipes the calendar, the prescalers, both alarms, the wake-up timer,
both calibrations, the timestamp, TAFCR, the twenty backup registers and
the clock choice. It is also ES0287 2.2.12's workaround: a supply dip
can leave the domain without the power-on reset it was owed, and the way
out is a domain reset from a power-on reset flag.

**The three roots.** LSE is a 32768 Hz crystal in the domain, so it
alone keeps the calendar through a loss of VDD; LSI is an RC oscillator
of 32 kHz nominal and 17..47 kHz over the range, outside the domain;
HSE/RTCPRE divides the board's high-speed crystal by 2..31, which 6.2.8
wants set to 1 MHz. THE VALUE OF RTCPRE IS THE DIVIDER, with 0 and 1
both meaning "no clock", so a 25 MHz crystal reaches 1 MHz exactly at
/25 and an 8 MHz one at /8; a crystal above 31 MHz, or one that is not a
whole multiple of the wanted rate, reaches it at none, and
`rtc_hse_divider_for()` answers 0 there. Because LSE is 32768 Hz by
construction, LSI is uncalibrated and the HSE branch is whatever a board
carries, EVERY RATE THIS DRIVER COMPUTES TAKES RTCCLK AS AN ARGUMENT: no
verb pretends to know which root is running.

**The prescalers make the second.** RTCCLK / (PREDIV_A + 1) is ck_apre,
what the sub-second counter counts down at; that over (PREDIV_S + 1) is
ck_spre, the calendar's own tick. 127 and 255 is the domain reset value
and the 32768 Hz pair. The chapter recommends a high asynchronous factor
for current, and the opposite - PREDIV_A small, PREDIV_S large - for
resolution, since 1 / (PREDIV_S + 1) second is what every sub-second
reading and every sub-second alarm resolve to.

**The calendar is read through shadows, or not at all.** With BYPSHAD = 0
the three readable registers are copies refreshed every two RTCCLK
cycles, RSF says a copy has landed, and reading SSR or TR is supposed to
lock the higher-order ones until DR is read. ES0287 2.8.2 says that lock
can be missed, and its workaround is either BYPSHAD = 1 or a second SSR
read to confirm. With BYPSHAD = 1 there is no copy and no waiting - what
a low-power program wants, and what a program measuring the 1 Hz edge
needs - at the price the chapter states: an RTCCLK edge between two
reads makes them disagree, so everything must be read twice. THAT PRICE
IS REAL AND NOT RARE: a loop polling RTC_SSR continuously catches about
nine torn values a second (measured), each of which reads as a reload to
anyone watching for one. `read()` applies the right discipline per mode
and `subsecond()` confirms its own load, so no caller has to remember.
One arithmetic precondition governs the shadow path: fPCLK1 must be at
least seven times fRTCCLK (17.3.6), which `rtc_shadow_read_allowed()`
prices.

**Initialization mode is a stopped calendar**, and it is the only state
in which TR, DR, PRER, RTC_CALIBR and CR's DCE, FMT and REFCKON may be
written. Entering it twice in a row is ES0287 2.8.4: INIT set one to two
RTCCLK cycles after being cleared sets INITF immediately instead of
waiting for the synchronization, and a calendar write in that window may
be dropped. The workaround - clear BYPSHAD if set, wait for RSF, restore
it - is applied unconditionally on the way OUT, so a second entry is
safe by construction; `exit_init_raw()` is the bare sequence, kept so
the erratum can be staged.

**Everything armed has a write window.** The wake-up timer's reload and
clock select may be written only with WUTE clear and WUTWF set; an
alarm's registers only with its enable clear and its own write flag set,
or in initialization mode; and both flags take one to two RTCCLK cycles
to appear. Every configuring verb disables, waits for its flag with a
bound, writes and re-enables, and answers false if the flag never came -
which on a stopped RTCCLK is what happens.

**Two calibrators, not to be used together.** Smooth calibration
(RTC_CALR) masks or inserts individual RTCCLK pulses over a 32-second
window: 0.954 ppm a step, -487.1 to +488.5 ppm, writable while the
calendar runs, with CALM's low bits stuck at zero in the shorter
windows. Coarse calibration (RTC_CALIBR, kept "for compatibility
reasons") adds two ck_apre cycles a minute or removes one, for the first
2 x DC minutes of a 64-minute cycle: about +4 / -2 ppm a step, -63 to
+126 ppm, initialization mode only, and refused below PREDIV_A = 6.
17.3.10 says the application must choose one, so each verb here refuses
while the other is armed. A CALR write raises RECALPF and blocks the
register until the setting takes - AND RECALPF DOES NOT FALL WHILE THE
CALENDAR IS STOPPED (measured, letter `j`): it is cleared on ck_apre,
which initialization mode stops, so a second smooth setting inside that
mode is refused until the calendar runs again.

**The sub-second shift** adds SUBFS to the synchronous prescaler's
counter, which counts down - so it DELAYS the clock by
SUBFS / (PREDIV_S + 1) seconds; with ADD1S the whole second is added at
the same time, which ADVANCES it by the complement. The asymmetry is the
register's: there is no "subtract a second" bit. The chapter's four
refusals - a field overflow, a shift already pending, REFCKON set, SS[15]
standing - are all coded.

**Three interrupts, three vectors, and the EXTI in between.** Alarms A
and B share EXTI line 17 and RTC_Alarm; the wake-up timer has line 22
and RTC_WKUP; tamper and timestamp share line 21 and TAMP_STAMP. These
are CONFIGURABLE EXTI lines and not the STM32G0's direct ones: the
rising edge must be selected, the mask opened, and the pending bit
cleared in the handler or the vector re-enters for ever. And a shared
line can swallow an event - ES0287 2.8.3: one source's flag rising after
the handler has checked it but before the EXTI's pending bit is
effectively clear never raises the line again. The three ISR bodies
therefore clear the EXTI first and then loop over the standing masked
flags until none stands.

**One pad carries the whole chapter's outside.** RTC_AF1 is PC13 on
every part whose manual was read: the tamper input, the timestamp input,
the calibration output and the alarm output are all that one pin, and
table 25 gives their priority - RTC_ALARM over RTC_CALIB over the
inputs. TWO CONSEQUENCES, both measured (letters `m` and `n`): with an
input function enabled the RTC takes the pad and the core cannot drive
it any more, and RTC_CR is not in a system reset's scope, so an output
left on the pad is still there at the next boot. ES0287 2.2.10 adds that
PC13 transitions disturb the LSE crystal on the LQFP and UFQFPN
packages, which no software can fix.

**Twenty backup registers**, ordinary 32-bit words in the domain: not
reset by a system reset, kept through Standby, powered from VBAT when
VDD goes away - and lost to a domain reset or a tamper detection. They
are this target's second kind of surviving storage, beside the .noinit
breadcrumb, which only a system reset spares. A TAMPER DETECTION ERASES
ALL TWENTY: this block has no per-input NOERASE bit and no mask, so
there is no way to arm a tamper input and keep a breadcrumb.

## Types and verbs

- `RtcDomain` (monostate) - `pwr_bus_clock(on)` (`Pwr`'s APB1 gate),
  `unlock(on)`/`unlocked()` (PWR_CR.DBP with the manual's read back),
  `bdcr()`, `reset()` (BDRST, the whole domain); LSE:
  `lse_enable`/`lse_enabled`/`lse_ready`/`lse_wait_ready(spins)`,
  `lse_bypass(on)` and `lse_mode(LseMode)`, each refused while the
  oscillator runs, `has_lse_mode()` (the parts whose RCC_BDCR carries
  LSEMOD); the LSI's verbs are `Rcc`'s ([reset.md](reset.md) says why);
  the HSE branch: `hse_divider(div)` in 2..31, refused once a source is
  selected, and its readback; `selected()`, `select(RtcClockSource)`
  (false when a different source stands), `enable(on)`/`enabled()`, and
  `open(source, wipe)` - the whole of 5.1.2's sequence. `ready_spins`
  bounds the register waits, `lse_ready_spins` the crystal's.
  `RtcClockSource::none | lse | lsi | hse_divided`, `LseMode::low_power |
  high_drive`, `rtc_hse_divider_for(hse_hz, want_hz)`.
- `RtcPrescalers{async, sync}` with `rtc_prescalers_valid`,
  `rtc_ck_spre_hz`, `rtc_ck_apre_hz`, `rtc_cycles_per_second`,
  `rtc_prescalers_for(rtcclk)` (the low-current pair),
  `rtc_prescalers_for_resolution(rtcclk)` (the stopwatch pair),
  `rtc_subsecond_ms(ss, prediv_s)`, `rtc_shadow_read_allowed(pclk1,
  rtcclk)`.
- `RtcDateTime` in ordinary numbers with `rtc_to_bcd`/`rtc_from_bcd`,
  `rtc_days_in_month`, `rtc_datetime_valid`, `rtc_time_register`,
  `rtc_date_register`, `rtc_decode`; `RtcReading{time, subsecond}`.
- `RtcAlarm` (the four masks, the weekday select, MASKSS and SS) with
  `rtc_alarm_valid`, `rtc_alarm_register`,
  `rtc_alarm_subsecond_register`; `RtcAlarmId::a | b`.
- `RtcWakeupClock::div16 | div8 | div4 | div2 | ck_spre | ck_spre_high`
  with `rtc_wakeup_divider`, `rtc_wakeup_valid`, `rtc_wakeup_clock_hz`.
- `RtcCalibration{plus, minus, window}` and `RtcCalibrationWindow`, with
  `rtc_calibration_valid` and `rtc_calibration_ppb`;
  `RtcCoarseCalibration{negative, steps}` with
  `rtc_coarse_calibration_valid` and `rtc_coarse_calibration_ppb`.
- `RtcOutput::off | alarm_a | alarm_b | wakeup` (OSEL) and
  `RtcCalibOutput::hz512 | hz1` (COSEL).
- `TamperFilter`, `TamperSampling` (with `tamper_sampling_divider` and
  `tamper_sampling_hz`), `TamperPrecharge`, `TamperTrigger` - whose two
  values are named for BOTH readings, because the bit means a level in
  the filtered detector and an edge in the other - `TamperConfig` (the
  block's half of TAFCR), `TamperInput{index, trigger}` and
  `tamper_input_valid`.
- `RtcFlag` - one mask per event, `all` for the set, `tamper2` zero on a
  part with one input.
- `Rtc` (monostate) - the keys `unlock()`/`lock()`; the readbacks `cr`,
  `isr`, `prer`, `wutr`, `calr`, `calibr`, `tafcr`, `status`,
  `calendar_set` (INITS), `in_init`, `synchronized`, `shift_pending`,
  `recalibration_pending`, `bypass_shadow`; the mode `enter_init`,
  `exit_init` (with ES0287 2.8.4's workaround), `exit_init_raw`,
  `wait_sync`; the configuration `set_prescalers` (and `set_prescalers<p>`
  checked at compile time), `prescalers`, `bypass_shadow(on)`,
  `set_calendar`, `init(prescalers, time)`; the readers `read(out)` and
  `subsecond()`; daylight saving `shift_hour(add)` and
  `daylight_flag`; `shift(add1s, subfs)`; `reference_clock()` and
  `reference_clock(on)` with its three refusals; the alarms
  `alarm_enabled`, `set_alarm(id, alarm, interrupt)` and `set_alarm<a>`,
  `clear_alarm`, and the bit helpers `alarm_enable_bit`,
  `alarm_interrupt_bit`, `alarm_write_flag`, `alarm_flag`; the wake-up
  timer `wakeup_enabled`, `wakeup_write_allowed`, `set_wakeup(clock,
  reload, interrupt)` and `set_wakeup<c, reload>`, `clear_wakeup`;
  calibration `calibrate` and `calibrate<c>`, `calibration`,
  `calr_unprotected` (the bench's back door, the one verb that does not
  bracket itself with the keys), `coarse_calibrate(on, c)`,
  `coarse_calibration_enabled`, `coarse_calibration`; the timestamp
  `timestamp_enable(on, falling)`, `timestamp_enabled`,
  `timestamp_interrupt`, `timestamp()`; the pad `alarm_output(sel,
  active_low, push_pull)` and its readback, `calibration_output(on,
  which)` - refused when the running prescalers cannot make the
  frequency asked for - `timestamp_pad(second)`, `tamper_pad(second)`;
  tamper `tamper_config`, `tamper_filter`, `tamper_arm`, `tamper_armed`,
  `any_tamper_armed`, `tamper_disarm`, `tamper_interrupt`,
  `timestamp_on_tamper`, `tamper_flag(index)`; the backup registers
  `backup(n)`, `backup(n, value)` and the compile-time-checked
  `backup<n>()` / `backup<n>(value)`; the flags `flag`, `clear_flags`,
  `masked_status`, `enabled_mask`; the EXTI `wake_line_open`,
  `wake_line_is_open` (the rising edge and the mask through
  [exti.md](exti.md)'s `Exti`, whose `pending(line)` / `clear(line)` are
  the flag's); the three
  ISR bodies `alarm_isr`, `wakeup_isr`, `tamper_stamp_isr`; the vectors
  `alarm_irq`, `wakeup_irq`, `tamper_stamp_irq` and the lines
  `alarm_exti_line`, `wakeup_exti_line`, `tamper_stamp_exti_line`; the
  constants `backup_count`, `tamper_inputs`, `has_second_pad`; and
  `debug_freeze` for DBGMCU's own bit.
- The reserve (`stm32f4/device_tables.hpp`): `rtc_backup_registers()`
  derived from `RTC_TypeDef` itself, `rtc_has_lse_mode()`,
  `rtc_pad_facts()` (how many tamper inputs and where RTC_AF2 is - a
  MANUAL fact, because ST's header declares TAMP2E on every part
  including the ones whose manual has one input), `rtc_af1_port` /
  `rtc_af1_pin`, the three EXTI line numbers and the three vectors.

## How to use it

Bring the domain up on the crystal, then set a calendar if none
survived:

```cpp
#include "stm32f4/rtc.hpp"

brio::RtcDomain::pwr_bus_clock(true);
brio::RtcDomain::unlock(true);
if (brio::RtcDomain::selected() != brio::RtcClockSource::lse) {
    brio::RtcDomain::reset();          // RTCSEL is one-way; this is the way back
}
brio::RtcDomain::lse_enable(true);
if (!brio::RtcDomain::lse_wait_ready()) { /* no crystal on this board */ }
brio::RtcDomain::open(brio::RtcClockSource::lse);

if (!brio::Rtc::calendar_set()) {      // INITS: nothing has ever been set
    brio::Rtc::init(brio::RtcPrescalers{}, brio::RtcDateTime{12, 0, 0, 1, 6, 24, 6});
}
brio::Rtc::wait_sync();
```

Read it, in one coherent look:

```cpp
brio::RtcReading r{};
if (brio::Rtc::read(r)) {
    brio::print(sink, r.time.hour, ":", r.time.minute, ":", r.time.second, " ss ", r.subsecond);
}
```

An alarm on a stated second, and the handler an app binds:

```cpp
brio::RtcAlarm a{};
a.second = 30;
a.mask_seconds = false;                       // the other three stay masked
brio::Rtc::set_alarm(brio::RtcAlarmId::a, a); // the EXTI line opens with it
brio::Nvic::enable(brio::Rtc::alarm_irq());

extern "C" void RTC_Alarm_IRQHandler() {
    const uint32_t served = brio::Rtc::alarm_isr();   // EXTI first, then the flags
    if ((served & brio::RtcFlag::alarm_a) != 0u) { /* ... */ }
}
```

A periodic wake-up every second, and one every 122 microseconds:

```cpp
brio::Rtc::set_wakeup(brio::RtcWakeupClock::ck_spre, 0);   // WUTF every (0 + 1) second
brio::Rtc::set_wakeup(brio::RtcWakeupClock::div4, 0);      // every 4 RTCCLK cycles
brio::Nvic::enable(brio::Rtc::wakeup_irq());

extern "C" void RTC_WKUP_IRQHandler() { (void)brio::Rtc::wakeup_isr(); }
```

Trim the crystal, in parts per million the arithmetic prices:

```cpp
brio::Rtc::calibrate(brio::RtcCalibration{.minus = 38});   // -36 ppm
static_assert(brio::rtc_calibration_ppb(brio::RtcCalibration{.minus = 38}) == -36'239);
```

Leave a note that outlives the program:

```cpp
brio::Rtc::backup<0>(0x5F4B4231u);              // refused at compile time past the end
if (brio::Rtc::backup<0>() == 0x5F4B4231u) { /* the last run left this */ }
```

Take a timestamp, and keep it: THE REGISTERS ARE FROZEN ONLY WHILE TSF
STANDS, so a handler reads them before the ISR body clears the flag.

```cpp
brio::Rtc::timestamp_enable(true, false);   // rising edge on RTC_AF1
brio::Rtc::timestamp_interrupt(true);
brio::Nvic::enable(brio::Rtc::tamper_stamp_irq());

extern "C" void TAMP_STAMP_IRQHandler() {
    const brio::RtcReading when = brio::Rtc::timestamp();   // FIRST
    const uint32_t served = brio::Rtc::tamper_stamp_isr();
    if ((served & brio::RtcFlag::timestamp) != 0u) { /* `when` is the event's time */ }
}
```

A one-second clock on the pad, for an instrument to count:

```cpp
brio::Rtc::calibration_output(true, brio::RtcCalibOutput::hz1);   // false: the prescalers cannot
```

## Bench findings

`test_stm32f4_rtc` on an STM32F411CE black pill with a 32.768 kHz
crystal, the core at 100 MHz from the board's 25 MHz HSE crystal through
the PLL: **131 pass, 0 fail** over letters `a` to `n`, plus `v`, `w` and
`x` by name. Every frequency below is a ratio against that core clock,
so a crystal weighs a crystal; the instrument is RTC_SSR's reload, which
a polling loop locates to a few hundred nanoseconds.

- **The crystal is 32769.18 Hz, +36 ppm** against the core clock over a
  two-second window (letter `b`), and the same +36 ppm comes back three
  more ways: the wake-up timer on RTCCLK/16, on RTCCLK/2 and on ck_spre,
  each weighed over two of its periods (letter `g`). The four agree to
  one part per million, which is the measurement's own floor.
- **The crystal starts in 125 ms** when restarted immediately after a
  stop, and 364 ms was measured at a boot that found it stopped - both
  far inside the two seconds the datasheet gives as typical, and the
  reason `lse_ready_spins` is sized where it is.
- **The LSI measures 32123.96 Hz**, 19654 ppm below 32768 and well
  inside RM0383's own 17..47 kHz window (letter `c`). Weighing it costs
  two domain resets, RTCSEL being one-way.
- **An alarm lands 31 microseconds after the second it matches**: alarm
  A on a seconds match raised its handler 3190 core cycles - 31 us -
  after the calendar's own second edge (letter `f`). A sub-second alarm at
  MASKSS = 15, SS = 128 landed 496 ms into the second against the 500 ms
  its arithmetic asks for, the difference being one ck_apre period and
  the reading loop.
- **Smooth calibration lands on its arithmetic** (letter `i`): against
  an uncalibrated +36 ppm, CALP gave +524 (+488 measured, +488 priced),
  CALM 256 gave -207 (-243 measured, -244 priced) and CALM 511 gave -467
  (-503 measured, -487 priced). The residual on the last is the
  two-second window against the 32-second calibration cycle: the masked
  pulses are spread over the whole cycle, not over the window.
- **A shift is visible as the length of one second** (letter `k`):
  SUBFS = 128 with PREDIV_S = 255 made the second it landed in 1499 ms
  long, and ADD1S with the same SUBFS put two calendar seconds inside
  1499 ms. The asymmetry is the register's - ADD1S bumps the calendar the
  instant the shift lands and then pushes the next update back.
- **RTC_CALIB counts out on the pad** (letter `m`): 511 rising edges in
  0.99938 s at COSEL = 0, and exactly 4 in 4.2 s at COSEL = 1, read
  through PC13's own input register with no instrument. The alarm
  output follows the flag it is pointed at - low with WUTF clear, high
  with it set, POL = 0.
- **ES0287 2.2.10 is not measurable here**: the crystal weighed 36 ppm
  before RTC_AF1 was driven and 36 ppm after a second of 512 Hz on it
  (letter `m`). The erratum stands - the measurement's floor is about a
  part per million and the package is an affected one - but nothing this
  suite can see moved.
- **The RTC takes the pad, and the board's own pull-up is a stimulus**
  (letter `n`). With no RTC input enabled the core drives PC13 both ways
  and reads it back; with the timestamp enabled the pad reads high
  whatever the core writes, which is table 25's "input floating" on
  silicon. So an edge is made by driving the pad low with the timestamp
  off and enabling it: the release floats up through the board's LED in
  18 to 29 ms and TSF stands. The frozen registers then carry the
  calendar's own second - and reading them AFTER the flag is cleared
  gives all zeros, which is why the suite's handler reads them first.
- **A tamper needs no wire either, and it costs the twenty registers**
  (letter `w`). The filtered detector precharges the pad through its own
  internal pull-up, so "staying high" with two samples at 128 Hz fires
  18 ms after arming against the 16 ms two samples price. All twenty
  backup registers read zero afterwards, and TAMPTS had filled the
  timestamp registers with the second it happened in.
- **The backup registers cross a system reset** (letter `v`): a token in
  BKP0 and nineteen words behind it, written in one run, re-flashed over
  and read back intact in the next - the flash ending in the reset that
  the domain does not see. A loss of VDD is not measured: this board has
  no VBAT cell.
- **RECALPF does not fall while the calendar is stopped** (letter `j`).
  A write to RTC_CALR raises it, and it is cleared on ck_apre - which
  initialization mode stops - so a second smooth setting inside that
  mode is refused until the calendar runs again. Neither manual says so.
- **A torn RTC_SSR read is common, not rare.** A loop polling the
  bypassed sub-second register continuously reads about nine values a
  second that no counter ever held; every one of them looks like a
  reload to a program watching for the 1 Hz edge, which is what turned a
  two-second measurement into 0.23 s before `subsecond()` was made to
  confirm its own load.
- **A coherent calendar read costs 144 core cycles through the shadows
  and 182 with them bypassed** (letter `e`) - four register loads
  against six, the bypass paying for the chapter's read-everything-twice
  and the shadow path for ES0287 2.8.2's extra SSR.
- **ES0287 2.8.4 did not show itself** in 200 consecutive
  initialization-mode pairs with the bare exit, and none with the
  workaround (letter `h`). The erratum is described as depending on
  where INIT's clear and set fall against the RTCCLK edges; 200 pairs at
  this core rate did not hit the window. The workaround stays
  unconditional.
- **Both locks behave as the chapter says** (letter `a`): a backup
  register write with DBP closed does not land and the same write with
  it open does; a store into RTC_CALR with the WPR key locked is dropped
  and the same store after 0xCA/0x53 lands; RTC_TAFCR and RTC_ISR's
  flags take a write with the key locked. A raw store into CALR leaves
  RECALPF standing, so the next one would be lost - which is why
  `calibrate()` waits it out and `calr_unprotected()`, the bench's back
  door, does not.
- The calendar's BCD conversion, INITS, the refusal of a write outside
  initialization mode and four boundaries crossed one second at a time -
  midnight, 29 February in a leap year, its absence in an ordinary one,
  and the year's turn - all hold (letter `d`).
- The family fixture compiles on all twenty-three device headers of the
  pack and the six negatives are refused for their own reasons
  (`brio check stm32f4`).

## Not covered yet

Driver gaps:
- **The 12-hour format.** `set_calendar()` pins FMT to 24 hours and
  `RtcDateTime` has no AM/PM. Declined: the struct is binary and one
  clock should not have two truths about what "one" means; an
  application that wants AM/PM converts.
- **The second tamper input and RTC_AF2.** `tamper_pad()` and
  `timestamp_pad()` refuse the second pad on a part whose manual gives
  it none, and the F411 is such a part - so the F42x/F43x's PI8 and the
  F446's PA0 are compiled and never armed. Needs one of those boards
  with the suite on it.
- **A part class whose manual is not on the desk gets one tamper
  input**, the one every manual read documents, and the second is
  refused there rather than guessed (the reserve says why). Reading
  RM0401, RM0402, RM0430 or RM0386 would close it.
- **Waking from Stop or Standby on an alarm, a wake-up or a tamper.**
  The interrupts are built and the EXTI lines are open; what enters a
  low-power mode is the power chapter's, and so is the RSF discipline
  after such a sleep.
- **A kernel timebase on this block.** The STM32G0 has an LPTIM ticker
  that counts through Stop; the RTC could be the F4's equivalent
  witness, and the sleep site that would use it is not built.
- **The other tamper combinations.** Only the filtered detector at two
  samples and 128 Hz is exercised, because every detection spends the
  twenty backup registers: the edge detector, four and eight samples,
  the other seven sampling rates and TAMPPUDIS (the precharge off, which
  needs an outside level to sample) are compiled and unarmed.

Implemented, not bench-verified:
- **RTC_REFIN (`reference_clock`)** - the three refusals are coded and
  the enable path is compiled; what would measure it is a 50 or 60 Hz
  square wave on PB15 and the calendar's second weighed while it locks
  to it, which needs a generator or a wire this bench does not have.
- **Coarse calibration's effect** - the register, the three refusals and
  the interlock with the smooth calibrator are all measured, but the
  correction itself is not: it modifies the first 2 x DC minutes of a
  64-minute cycle, so a measurement is a 64-minute window against the
  ruler.
- **The HSE branch of RTCSEL** (`hse_divided` with `hse_divider`) -
  compiled, and its refusals proven; never selected on a board, which
  would cost a domain reset and give a calendar as good as the HSE
  crystal and no better. Weighing it is the same two-second window.
- **`lse_bypass` with a real external clock** and **`lse_mode`'s high
  drive** - both register rules are proven (each is refused while the
  oscillator runs), neither state is entered: the board has a crystal
  that starts at low power in 125 ms.
- **The 17-bit wake-up range** (`ck_spre_high`) - the code is compiled
  and the encoding checked; the shortest interval it can be asked for is
  18 hours.
- **The alarm masks other than the two measured.** The seconds-only
  match and the sub-second match are weighed; the date, weekday and hour
  masks are compiled, and each would cost a day, a week or an hour to
  see fire.
- **The alarm output's open-drain type and POL = 1** - `alarm_output`
  writes both and only push-pull with POL = 0 was read back on the pad.
- **The 8- and 16-second calibration windows** - their stuck-bit
  refusals are proven; their effect on the correction's distribution is
  not (it wants a window as long as the cycle).
- **VBAT.** The backup registers and the calendar are measured across a
  system reset; surviving a loss of VDD is the manual's statement and
  needs a cell on the board's VBAT pin.
- **The chapter on STM32F42x/F43x silicon.** The driver compiles on
  every header of the pack and the suite ran on the STM32F446 and the
  STM32F411; the STM32F429 board on the desk has no LSE crystal, so the
  suite is not built for it. What would measure it is that board with an
  X2 fitted, or a part of the class on a board that has one.
