# RTC - Real-Time Counter (SAM C21)

> **PROVISIONAL.** The whole of chapter 24 is implemented and
> bench-verified in all three modes. What is NOT here is any TASK over
> it (an alarm clock, a slow periodic source, a power-pass timebase -
> each is a policy and is born with its first user), anything about
> sleep and wake, and one measurement the board cannot make: the
> frequency correction's per-step linearity, because the trim's whole
> range is smaller than the short-term wander of every clock this board
> can give the RTC. The list is in "Not covered yet".

Documents of record: SAM C20/C21 data sheet DS60001479M ch. 24, plus
21.6.7 for the clock selection that lives in another chapter - and
errata DS80000740S items 1.16.1, 1.16.2, 1.16.3 and 1.8.7, of which
**one is live on this silicon**. Driver: `samc/rtc.hpp`. The family
fixture is `test/family_samc/rtc.cpp` plus seven negatives under
`tools/check_samc.sh`; the bench suite is `test_samc_rtc`.

## What the silicon does

**One counter wearing three faces.** A 10-bit prescaler feeds a 32-bit
counter, and CTRLA.MODE decides what that counter IS: a 32-bit counter
with one 32-bit compare (mode 0), a 16-bit counter whose top is the PER
register with two 16-bit compares (mode 1), or a clock/calendar with
one masked alarm (mode 2). The register map is three OVERLAID VIEWS -
CTRLA, EVCTRL, the interrupt registers, DBGCTRL, SYNCBUSY and FREQCORR
sit at the same offsets in all three and only what follows differs - so
the driver writes the control surface once and gives the width-carrying
verbs explicit flavours, exactly as [tc.md](tc.md) does for
COUNT8/16/32.

**THE CLOCK IS NOT THIS DRIVER'S TO CHOOSE.** The RTC has no generic
clock channel at all; its source is picked by OSC32KCTRL.RTCCTRL.RTCSEL
- one register in chapter 21, owned by
[osc32kctrl.md](osc32kctrl.md)'s driver. `samc/rtc.hpp` never writes
it. What it does is state 21.6.7's obligation ("it is highly
recommended to disable the RTC module first, before the RTC clock
source selection is changed") and take the chosen RATE as an argument
where arithmetic needs one. The corollary is sharper than the rule:
SWRST and ENABLE synchronize INTO that clock's domain, so resetting the
RTC with no source running would leave SYNCBUSY standing forever. The
reset default (RTCSEL = ULP1K, off the always-running OSCULP32K) is
what makes the naive order work anyway, and every wait in the driver is
bounded and reports.

**A 32.768 kHz source CANNOT REACH 1 Hz, so the calendar needs a
1.024 kHz one.** Mode 2 requires a 1 Hz counter clock (24.6.2.5) and
the prescaler stops at /1024. Chapter 24 never draws the conclusion;
`rtc_prescaler_for_hz()` is where it lives here, returning nothing for
a source it cannot divide. That is also why RTCCTRL offers a 1 kHz
output on every one of the three oscillators.

**PRESCALER = OFF and PRESCALER = DIV1 divide the same and are not the
same thing.** Both pass the source straight through; OFF additionally
SILENCES all eight periodic events and interrupts (24.8.1, 24.6.8.1).
The driver names both codes and refuses a configuration that asks for a
periodic event with the prescaler off - which the silicon would accept
and then never honour.

**The periodic taps are the prescaler's own bits.** PEREO[7:0] and
PER[7:0] tap prescaler bits 2..9, giving f_source / 2^(n+3): PER0 every
eight source cycles, PER7 every 1024. They are independent of the
divisor the COUNTER uses - the same prescaler serves both.

**Enable-protection and write-synchronization are different things, and
CTRLA carries both.** MODE, PRESCALER, MATCHCLR and CLKREP are
enable-protected and not synchronized; ENABLE and SWRST are
synchronized and not protected; EVCTRL is enable-protected as a whole.
COUNTSYNC/CLOCKSYNC is the exception the chapter states explicitly
(24.8.1, "this bit is not enable-protected"): it is write-synchronized
and can be moved under a running counter, which is what makes it
measurable and is why the driver gives it a live verb beside the
configuration field.

**READING THE COUNTER NEEDS A BIT THAT IS OFF AT RESET.** COUNT (and
CLOCK) are read-synchronized, gated by CTRLA.COUNTSYNC / CLOCKSYNC, and
24.8.1 says plainly that "disabling the synchronization will prevent
reading valid values". `RtcConfig::read_sync` therefore defaults TRUE.
What the bit actually gates is measured below.

**MATCHCLR raises two flags at once.** With the counter cleared on a
COMP0 (or ALARM0) match, "INTFLAG.CMP0 and INTFLAG.OVF will both be set
simultaneously" (24.6.2.3) - a counter that never reaches its top still
reports overflows. MATCHCLR is valid in modes 0 and 2 only (24.12.1),
and mode 1 asking for it is refused.

**The calendar's leap rule is not the Gregorian one.** "The year is
considered a leap year if YEAR[1:0] is zero" (24.12.9), where YEAR is
a 0..63 OFFSET from a reference year the software picks and the silicon
never sees - and 24.6.2.5 requires that reference to BE a leap year
(2016, 2020, ...). There is no century exception, which is harmless
over the register's 64-year span as long as it does not cross a
non-leap century.

**CLKREP changes what HOUR MEANS, not how it is displayed.** In the
24-hour representation HOUR is five bits holding 0..23 and bit 4 is
part of the number; in the 12-hour one HOUR[3:0] holds 1..12 and
HOUR[4] is AM/PM. The same bits, two readings. `RtcClockValue::
from_register()` is therefore told which, and `Rtc::clock_value()` asks
CTRLA rather than the caller.

**The chapter names an event it does not implement.** 24.6.5 lists
"Periodic Daily (PERD): generated when the COUNT/CLOCK has incremented
at a fixed period of time" among the output events. No EVCTRL bit for
it appears in any of the three register summaries, the register
descriptions do not mention it, and the device header declares nothing
of the kind. The driver does not offer it.

### Errata, read on the E/G/J ROW at revision F

- **1.16.3 Write Corruption - LIVE ON EVERY REVISION, this one
  included.** "An 8-bit or 16-bit write access for a 32-bit register,
  or an 8-bit write access for a 16-bit register can fail", for COUNT
  in both counter modes and for CLOCK. The workaround is to write each
  register at its full width, and the workaround here is STRUCTURAL:
  the driver has no verb that writes a byte or a half of any of them,
  so no caller is offered a way to get it wrong.
- **1.16.1 Read Synchronization** (COUNTSYNC/CLOCKSYNC has no effect,
  read synchronization always on): **E/G/J revision B ONLY.**
- **1.16.2 COUNTSYNC** (the first COUNT value after enabling COUNTSYNC
  is wrong): **E/G/J revisions B..E.** The marks under F and H on that
  item belong to the N-FAMILY ROW - the read-the-row trap
  [dmac.md](dmac.md) already records. The bench looks for the symptom
  anyway and does not find it (below).
- **1.8.7 DMA Write Access** (a DMA write during standby may not land;
  RTC.COUNT is on the list) is live on every revision, and is a
  caller obligation for the power pass rather than something a driver
  can wrap: use Idle rather than Standby when SleepWalking writes
  COUNT.

## Types and verbs

- **`Rtc`** - the block and the whole chapter, a monostate struct
  rather than a template because this family has exactly one instance.
  `init()` / `release()` (APB clock, interrupt off, software reset),
  `reset()` (disabling first, as 24.6.2.2 requires), `enable(bool)`,
  `configure(RtcConfig)` and its compile-time twin `configure<cfg>()`,
  `event_config(RtcConfig, RtcEventConfig)`, `read_sync(bool)`,
  `sync_flags()` / `sync_wait()`, `flags` / `arm` / `disarm` /
  `clear_flags` / `isr()`, `debug_run(bool)`.
- Per-mode accessors: `count32` / `set_count32` / `comp32` /
  `set_comp32`; `count16` / `set_count16` / `period16` /
  `set_period16` / `comp16` / `set_comp16`; `clock_value` /
  `set_clock` / `alarm` / `set_alarm` / `alarm_mask` /
  `set_alarm_mask`. Each synchronized read waits its SYNCBUSY bit;
  `*_raw()` is the unsynchronized load and says so.
- `set_frequency_correction(negative, value)` - refuses a value past
  the seven-bit field AND a prescaler of OFF or DIV1, which 24.6.8.2
  requires; `correction_value()` / `correction_negative()` read back.
- **`RtcConfig`** (mode, prescaler, match_clear, twelve_hour,
  read_sync) and **`RtcEventConfig`** (periodic_out, compare_out,
  overflow_out), with `rtc_config_valid()` and
  `rtc_event_config_valid()` as constexpr predicates - which is what
  the negatives bite on.
- **`RtcMode`**, **`RtcPrescaler`** + `rtc_prescaler_divisor()` +
  `rtc_prescaler_for_hz()`, **`RtcAlarmMask`** +
  `rtc_alarm_mask_valid()`, **`RtcFlag`** (the periodic, compare,
  alarm and overflow masks, with `compare0` and `alarm0` static-
  asserted equal because they are one bit under two names).
- **`RtcClockValue`** - the calendar word unpacked, with
  `to_register()`, `from_register(word, twelve_hour)`, `valid()`, and
  the free functions `rtc_is_leap()` / `rtc_days_in_month()` carrying
  the chapter's own rule.
- `rtc_periodic_mhz(source_hz, n)` and `rtc_correction_ppb(value)` -
  the two pieces of arithmetic the chapter states and nobody should
  re-derive.
- The EVSYS vocabulary this peripheral publishes, per
  [evsys.md](evsys.md)'s division of labour: `compare_generator(n)`,
  `alarm_generator`, `overflow_generator`, `periodic_generator(n)`,
  all from the device header's own `EVENT_ID_GEN_RTC_*`. **The RTC is
  a generator only** - it consumes no events, and the header declares
  no `EVENT_ID_USER_RTC_*` at all.

## How to use it

The order is the point: the clock is selected through the other
driver, with the RTC disabled, and only then does this one start.

```cpp
// A 32-bit tick counter on the always-available root.
brio::Rtc::init();                                   // APB clock + reset
brio::Osc32kctrl::rtc_clock(brio::RtcClock::ulp_32k);   // 21.6.7: RTC disabled
brio::Rtc::init();                                   // reset on the chosen clock
brio::Rtc::configure({.mode = brio::RtcMode::count32,
                      .prescaler = brio::RtcPrescaler::div1});
brio::Rtc::enable(true);
const uint32_t ticks = brio::Rtc::count32();
```

A calendar needs a 1.024 kHz source, because the prescaler cannot bring
32.768 kHz down to 1 Hz:

```cpp
brio::Osc32kctrl::rtc_clock(brio::RtcClock::ulp_1k);
brio::Rtc::init();
brio::Rtc::configure({.mode = brio::RtcMode::clock,
                      .prescaler = brio::RtcPrescaler::div1024});   // 1 Hz
brio::Rtc::enable(true);
brio::Rtc::set_clock({.second = 0, .minute = 30, .hour = 9,
                      .day = 4, .month = 7, .year = 8});   // year+8 of a leap reference
brio::Rtc::set_alarm({.second = 0, .minute = 0, .hour = 7,
                      .day = 5, .month = 7, .year = 8});
brio::Rtc::set_alarm_mask(brio::RtcAlarmMask::day_and_below);
brio::Rtc::arm(brio::RtcFlag::alarm0);
brio::Nvic::enable(brio::Rtc::irq());
```

A periodic event out of the prescaler, with no CPU in the path - and
note that the prescaler must not be OFF or none of these exist:

```cpp
constexpr brio::RtcConfig cfg{.mode = brio::RtcMode::count32,
                              .prescaler = brio::RtcPrescaler::div1};
brio::Rtc::configure(cfg);
brio::Rtc::event_config(cfg, {.periodic_out = 0x08});   // PER3 = f / 64
brio::Rtc::enable(true);
brio::Evsys::connect(user, channel,
                     {.generator = brio::Rtc::periodic_generator(3),
                      .path = brio::EventPath::asynchronous});
```

## Bench findings

`test_samc_rtc`, 8 letters / 125 verdicts, **125/125** on the C21J at
revision F, wireless. The instruments: a TC0+TC1 pair as a 32-bit
stopwatch clocked FROM THE BOARD'S 24 MHz CRYSTAL through generator 2
(not from GCLK0, which is OSC48M - an RC 5100 ppm slow with a wander of
its own, [clock.md](clock.md)), `samc/freqm.hpp` measuring the RTC's
source against the same crystal, and a DMA channel armed with no
hardware trigger as the event witness.

**THE COUNTER COUNTS ITS SOURCE, TICK FOR TICK**, on all four clock
selects this board can reach. FREQM weighs the oscillator and the
stopwatch counts the RTC, both against the crystal, and the two agree
to **48 to 740 ppm** - which is not the counter's error but the RC's
own wander between the two instruments' windows (FREQM averages over
eight milliseconds, the counter over a second). The spread of eight
consecutive FREQM readings, printed by the letter, is 350 to 1100 ppm
on the same oscillators.

**And because the ruler is the crystal, those are ABSOLUTE numbers**:
OSCULP32K measures **33002 Hz** and a factory-trimmed OSC32K **33152
to 33174 Hz**, both about 7 per mille above the nominal 32768 - which
is the first time this stratum has weighed the slow oscillators
without an RC in the reference. Note that it does not agree to the
hertz with the rescaled figure in [clock.md](clock.md) (32907 Hz for
OSCULP32K, derived hours earlier from an OSC48M-scaled reading): the
two differ by about 3 per mille, which is the oscillator's own drift
between sessions and not a disagreement between the instruments. An
RC read to five figures is a reading of one afternoon.

**The prescaler is exact.** DIV2, DIV32 and DIV1024 imply the same
source rate as DIV1 to within **70 to 300 ppm** - and the DIV1024
measurement is only possible because the rate windows are EDGE-ALIGNED
at both ends: at 32 Hz a plain count would carry 15000 ppm of its own
quantization.

**PRESCALER = OFF divides by one and silences the periodic
intervals**, which is 24.8.1's sentence and the whole reason both codes
are named. Over the same 50 ms window PER0 is raised at DIV1 and is
silent at OFF, while the counter runs at the same rate in both.

**Events, both kinds, with no CPU in the path.** A COMP0 event and a
PER3 event each moved a 16-byte DMA block through an ASYNCHRONOUS EVSYS
channel on a DMA channel armed with `dma_trigger_none` - the transfer
is the witness, as in [evsys.md](evsys.md). The single interrupt vector
carried the compare (INTFLAG 0x0100 acknowledged by the driver's ISR
body).

**MATCHCLR observed doing both things at once**: with COMP0 at 1024 the
counter sits at a few hundred and never approaches its own top, and
INTFLAG reads 0x81FF - the compare, the overflow AND all eight periodic
flags together. 24.6.2.3's note is exact.

### What the read synchronization costs, and what it hides

- **A synchronized COUNT read costs 2.2 us**, against 0.19 us for the
  raw load, where one source period is 30.5 us. So COUNTSYNC is NOT a
  per-read handshake into the 32 kHz domain: the register is kept
  synchronized in the background and the cost is the SYNCBUSY check.
- **THE READ TRAILS THE COUNTER BY A CONSTANT FOUR TICKS.** At the
  instant INTFLAG.CMP0 appears, the readable COUNT is the compare value
  MINUS 4 - and it is exactly minus 4 on all eight repetitions, never
  minus 3 or minus 5. Since the flag is itself set one tick after the
  match, the readable value trails the counter by five source periods,
  about 150 us at 32 kHz. The number that matters is not the lag but
  its CONSTANCY: a jittery synchronizer could not be corrected for, and
  this one could.
- **With COUNTSYNC clear the readable COUNT is FROZEN**, not garbage
  and not wrong-by-a-little: the same value comes back fifty
  milliseconds later, and it moves again the moment the bit is set.
  That is what 24.8.1's "will prevent reading valid values" means on
  this silicon.
- **Toggling COUNTSYNC off and on costs about 5 ms** - two
  write-synchronizations into the 32 kHz domain, at some 80 source
  periods each. It is not a knob to flick in a loop.
- **After a COUNT WRITE the synchronized read shows the OLD value for
  about 190 us** (six source periods) before it catches up. A caller
  that writes COUNT and reads it back immediately reads the past.
- **ERRATUM 1.16.2 IS NOT ON THIS DIE, by behaviour as well as by the
  matrix.** The first COUNT read after enabling COUNTSYNC is judged
  against the wall clock rather than by eye: the toggle takes 5 ms of
  real time, and the counter advanced by 180 ticks where the measured
  source rate accounts for 181.

### Mode 1

- **The period is PER + 1 source ticks, not PER.** 33 overflows in one
  second against a source measured at 33041 Hz gives **1001 ticks a
  period for PER = 999**, and the counter is never seen above 998. The
  same one-off that the AVR TCD's printed dual-slope formula got wrong
  ([avrdx/tcd.md](../avrdx/tcd.md)) is right here, and 24.6.2.4's
  wording ("increments until it reaches the PER value, and then wraps")
  is exact.
- **A MODE CHANGE DOES NOT CLEAR COUNT**, and a 16-bit counter that
  starts ABOVE PER never meets it: it runs to 0xFFFF instead. The first
  version of the suite watched exactly that happen, with the value
  mode 0 had left behind. Nothing in the chapter warns of it.
- **AND A DRIVER TRAP THE BENCH CAUGHT.** The device header's group
  mask for the compare EVENT outputs is ONE bit in the mode 0 view
  (`RTC_MODE0_EVCTRL_CMPEO_Msk`) and TWO in the mode 1 view, at the
  same position - so a driver writing the shared control surface
  through the mode 0 macro, which is the natural thing to do when
  everything else is identical, silently drops CMPEO1. This one did,
  until a verdict asked for both bits to be readable back. The driver
  now writes that field through the mode 1 macro, which is a superset,
  and a family static_assert pins both widths.

### Mode 2, the calendar

- **One second carries every field at once.** 23:59:59 on 31 December
  of year+0 becomes 00:00:00 on 1 January of year+1.
- **The leap rule is the chapter's, and the bench confirms both
  branches**: 28 February of year+0 (a leap year by YEAR[1:0] == 0) is
  followed by **29 February**; 28 February of year+1 goes straight to
  **1 March**.
- **The top of the range wraps as 24.6.2.5 says**: 23:59:59 on
  31 December of year+63 becomes 00:00:00 on 1 January of year+0, with
  INTFLAG.OVF raised.
- **THE ALARM IS A WHOLE COUNTER PERIOD LATE**, measured at **989 ms**
  after the match on a 1 Hz counter. 24.6.2.5 says so in a sentence
  easy to read past ("the Alarm 0 Interrupt flag is set with a delay of
  1s after the occurrence of alarm match"), and the first version of
  this suite read the flags at the instant of the match and reported a
  working alarm as broken. The overflow flag, by contrast, is already
  there.
- **MASK.SEL = OFF is a real disarm**: the same match two seconds later
  raises nothing, while the overflow still does.
- **CLKREP is a reading, not a format.** The same CLOCK word that reads
  as 11 PM in the 12-hour representation reads as **hour 27** in the
  24-hour one, because HOUR[4] is part of the number there.

### The frequency correction, and what this board cannot say about it

- **The sign is confirmed, under the register's double negative.**
  SIGN = 0 is called a POSITIVE correction and DECREASES the frequency
  (24.8.8): the +127 windows measure longer than the -127 ones, in
  every block of every session.
- **The magnitude measures LARGER than 24.6.8.2's formula.** The full
  swing between SIGN positive and SIGN negative at VALUE 127 comes out
  at **415 to 620 ppm** (median of seven ABBA blocks, five separate
  sessions) where `2 x VALUE / 983040` predicts **258 ppm** - a factor
  of 1.6 to 2.4. The driver's `rtc_correction_ppb()` still states the
  chapter's formula: one board's RC is no basis for replacing it, and
  the discrepancy is recorded here instead.
- **THE TRIM'S WHOLE RANGE IS SMALLER THAN THE SOURCE'S OWN WANDER.**
  Both 32 kHz roots on this board are internal RCs, and a repeated
  untrimmed measurement of either moves by a hundred to three hundred
  ppm from one window to the next - against a trim whose extreme is
  129 ppm. That is why the measurement is built as a lock-in (the trim
  switched + - - + so that any drift linear across four windows cancels
  exactly, seven such blocks, and the MEDIAN rather than the mean,
  because one block in seven lands on a stretch where the RC moved).
  FREQCORR is a crystal's instrument, and this board cannot put a
  crystal on the RTC's clock select.
- **The per-step linearity is therefore NOT judged.** A sweep over
  VALUE 32, 64, 96, 127 was written first and thrown away: its readings
  were the wander with the trim underneath, non-monotonic from run to
  run, and asserting a shape on them would have made a coin toss into a
  verdict.
- **What one "count in the prescaler" is worth is OPEN.** The same
  measurement repeated at DIV16 was meant to settle it - an unchanged
  answer would mean one SOURCE cycle, an eightfold one would mean one
  COUNTER tick. It settles nothing: the DIV16 answer moves with the
  WINDOW LENGTH as well (around 500 ppm over a 32768-cycle window,
  under 100 ppm over a 16384-cycle one), which neither reading
  explains and which points at the adjustments being distributed
  unevenly inside the 240-period correction cycle. The number is
  printed by the suite and not verdicted.


- **As a wake source** (the transversal sleep pass -
  [platform.md](platform.md)): all three of this chapter's interrupts
  wake the device from STANDBY and none needs a RUNSTDBY bit, this
  block having none and riding OSC32KCTRL rather than a generator. A
  COMP0 compare 1000 ticks ahead woke it in 30177 us against 30312 us
  asked, four of four; a periodic interrupt woke it inside one of its
  own periods (7679 us against 7744 us measured awake); and a mode-2
  ALARM woke it where the calendar said, to the resolution the calendar
  can be read at. Its periodic outputs are also what paces every
  sleepwalking chain in this stratum, over an asynchronous EVSYS
  channel whose CHANNELn.RUNSTDBY is set.

## Not covered yet

Driver gaps:
- **No tasks at all**, deliberately, following `avrdx/rtc.hpp`: an
  alarm clock, a slow periodic source and a power-manager timebase are
  each a policy, and each is born with its first user.
- **The RTC is not the kernel timebase and this pass did not make it
  one.** `samc/ticker.hpp` stays on SysTick and says why; the RTC is
  what the power pass will want when SysTick stops in standby.
- **Erratum 1.8.7's caveat** - that a DMA write to RTC.COUNT during
  standby SleepWalking may not land - is stated and unexercised: it
  needs the DMAC across a sleep, which is [dmac.md](dmac.md)'s own gap.
- **PERD** (24.6.5's "Periodic Daily" event) is not offered, because no
  register in the chapter and no symbol in the device header
  implements it.

Implemented but not bench-verified:
- **The overflow EVENT** (EVCTRL.OVFEO) is configured and its flag is
  observed, but no letter has taken an overflow event through EVSYS to
  a user - only the compare and periodic ones.
- **`RtcAlarmMask` levels other than `year_and_below` and `off`.** The
  ladder is written and refused past SEL 6; only the two ends have been
  exercised on silicon, because the intermediate ones need waits of up
  to a minute.
- **The 12-hour representation only round-trips through the register**;
  no letter has watched a 12-hour clock roll from 11:59 PM to
  12:00 AM, which is another minute of waiting for a bit whose meaning
  is already proven by the two readings of the same word.
- **`Rtc::release()`** and `bus_clock(false)`, which would make the
  block unreachable, are never called by the suite.
- **The XOSC1K / XOSC32K clock selects**, because this board carries no
  32 kHz crystal ([osc32kctrl.md](osc32kctrl.md) says the same of the
  oscillator itself).
- **`debug_run(true)` under an actual halted debugger.** The bit is set
  and proven to survive a software reset; nobody has halted the core to
  watch the counter keep going.
