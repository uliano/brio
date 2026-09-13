# RTC (RP2040)

Documents of record: the RP2040 datasheet (build 3184e62), 4.8 (the
real-time clock: 4.8.1 the storage format and the day of the week,
4.8.2 the leap year, 4.8.3 the interrupt, 4.8.4 the reference clock
and the clock-domain crossing, 4.8.5 the programmer's model - the
divider, the set, the read, the alarm, the dormant state -, 4.8.6
the registers), 2.15 (clk_rtc). The driver: `brio/rp2040/rtc.hpp`
(`Rtc` the resource, `RtcDateTime`, `RtcAlarm` and the calendar
arithmetic) over `clock.hpp` (`Clocks::rtc_select`) and
`resets.hpp`. The reference suite: `test_rp2040_rtc`, wireless.

## What the silicon does

A calendar of seven binary fields - a 12-bit year, the month, the
day, the day of the week (Sunday 0), the hour, the minute, the second
- counting seconds from a reference the block makes by dividing
clk_rtc by an integer (CLKDIV_M1 + 1): clk_rtc is a generator of its
own with an aux mux and a 24.8 divider, and any whole rate from 1 to
65536 Hz of it will do, the chapter's example being the crystal over
256, 46875 Hz. A leap year is "divisible by four" and nothing more,
with a bit (FORCE_NOTLEAPYEAR) to switch it off for a century year.
The block checks no field's range and computes no day of the week -
it increments the one given. ONE alarm: a match on any subset of the
seven fields, a field left out a wildcard, so fewer fields make a
repeating alarm; the match is the block's interrupt and the wake of
the chip's dormant state. Every register lives in clk_sys and is
synchronized into the clk_rtc domain: a write takes two clk_rtc
periods to land, and a coherent reading is RTC_0 before RTC_1, the
first read latching the second.

Three facts measured that the chapter does not state: the block's
reset completes without clk_rtc (the converter's does not); A
CALENDAR ENABLED FROM A STOP TICKS AT ONCE - one second some 130 us
after RTC_ACTIVE, whatever the divider's phase was, the next a whole
second later, where a LOAD into a running calendar ticks nothing -
so a set value reads one second on within microseconds unless undone;
and THE ALARM IS A LEVEL - the interrupt stands for the whole matching
second, and the disarm itself lands two clk_rtc periods later, during
which the handler is re-entered (124 entries in 43 us measured), so a
handler masks the line first.

## Types and verbs

- `RtcDateTime` (year, month, day, weekday, hour, minute, second, the
  chapter's encoding, ordinary numbers), `rtc_silicon_leap` /
  `rtc_gregorian_leap` / `rtc_needs_force_not_leap` (the century years
  the silicon gets wrong), `rtc_days_in_month`, `rtc_weekday_of`
  (Sakamoto's method, Sunday 0), `rtc_datetime_valid` (table 551's
  ranges, the day against its month), `RtcAlarm` (a date-time with a
  match flag per field) with `rtc_alarm_repeats` and
  `rtc_alarm_valid`, the register words `rtc_word0_of` /
  `rtc_word1_of` / `rtc_datetime_of`, `RtcClock` (crystal_div256,
  gpin0_1hz).
- `Rtc`: `init(clock, source)` (clk_rtc from the crystal over 256 or
  the one-pulse-per-second input, the block out of reset, CLKDIV_M1
  for a one-second reference; the calendar not started), `clock_hz`,
  `reference_divider`, `release`; `running` (RTC_ACTIVE), `enabled`,
  `set(datetime)` (the chapter's sequence, then the value waited for
  in the read path, the enable's tick waited for and undone by a
  second LOAD, the value waited for again: the first second whole,
  the value the one asked for, about 200 us in all), `adjust(datetime)`
  (a LOAD into the running calendar), `enable(on)`, `read()` (RTC_0
  then RTC_1; nothing while stopped), `force_not_leap_year(on)`;
  `alarm(RtcAlarm)` (written disarmed, then armed), `arm_alarm` /
  `disarm_alarm` / `alarm_armed`, `alarm_matching` (MATCH_ACTIVE),
  `alarm()` read back; `interrupt(on)`, `raw_pending` / `pending`,
  `force`, `isr()` (true on the alarm, WITH THE LINE MASKED AND THE
  MATCH DISARMED), `rearm_after()` (the match re-armed once the
  calendar has left the matching value, the line unmasked; false with
  both left off), `irq()`.
- `clock.hpp` grew `Clocks::rtc_select(aux, div_int, div_frac)`,
  `rtc_stop`, `rtc_enabled`, `rtc_source`, `rtc_divider256` with
  `RtcAux`.

## How to use it

```cpp
brio::Rtc::init(clock);                                   // clk_rtc = crystal / 256, CLKDIV_M1 = 46874
brio::Rtc::set({.year = 2026, .month = 9, .day = 13, .weekday = brio::rtc_weekday_of(2026, 9, 13),
                .hour = 12, .minute = 30, .second = 0});
if (const auto now = brio::Rtc::read()) { show(*now); }

brio::Rtc::alarm({.at = {.second = 0}, .match_second = true});   // every minute at :00
brio::Rtc::interrupt(true);
extern "C" void isr_rtc() {
    if (brio::Rtc::isr()) {            // the line masked, the match disarmed
        minute_passed = true;          // the application re-arms with rearm_after() once the second is over
    }
}
```

A one-shot alarm matches every field; the century year 2100 wants
`force_not_leap_year(true)` between 1 March 2096 and 28 February
2100.

## Bench findings

The reference suite is `test_rp2040_rtc`, green on the Pico and the
WeAct board,
every letter wireless: clk_rtc counted by the frequency counter, the
calendar's second and the alarm's latency timed on the system timer,
the boundaries crossed one second at a time.

- THE CLOCK: init() puts the crystal over 256 on clk_rtc, counted at
  46875 Hz, and 46874 in CLKDIV_M1; the block's reset completes with
  clk_rtc stopped. The arithmetic: both leap rules, the days of a
  month, the weekday of four known dates, the refusals (a 4096th year,
  a 13th month, a 30 February, a 7th weekday, a 24th hour).
- SET AND READ: a date set reads back the same, set() returning in
  about 200 us; the calendar's second is 1.000 s on the timer (999998
  us between two rollovers); the first second after a set is a whole
  one at three phases of the set (the enable's tick waited for and
  undone); three seconds of the timer are three of the calendar;
  read() declines while stopped.
- THE BOUNDARIES, one second each: the year rolls with the weekday,
  29 February in 2024 and not in 2023, 1 March after 29 February 2024,
  April ends, Saturday rolls to Sunday, and 2100 gets a 29 February
  the Gregorian calendar has not - unless FORCE_NOTLEAPYEAR, which
  gives 1 March.
- THE ALARM: a second-matched alarm fires 37 us after the read that
  saw the rollover, once, the ISR body masking the line and disarming
  the match; left armed, it re-enters the handler for the whole
  matching second (1155 entries in one second); re-armed after its
  second it fires at the next minute's :03, 4999 ms after a jump to
  :58; a fully matched alarm fires once and reads as non-repeating; an
  alarm on the wrong weekday never fires.
- Adjusted while running the calendar takes the new value; stopped it
  reports not running and restarted it counts on; the forced
  interrupt appears in the status with no raw flag.

## Not covered yet

Driver gaps, each with its reason:

- The one-pulse-per-second reference on GPIO 20 (`RtcClock::gpin0_1hz`):
  selected and divided by one; a GPS or a second board's 1 Hz on the
  wire would measure it.
- The dormant wake by the alarm (4.8.5.5) is [sleep.md](sleep.md)'s
  measurement: the ring oscillator dormant, the crystal keeping
  clk_rtc, the alarm the wake.
- The calendar across a chip reset: the block is not in the core's
  reset scope, and whether the watchdog's PSM selection spares it is a
  reset chapter's measurement across a reboot.

Implemented but not bench-verified, each with what would measure it:

- `rtc_needs_force_not_leap` across the years 2200 and 2400 and the
  divide-by-400 rule: pinned at compile time; the 2100 crossing is
  the one measured.
- A coarser repeating alarm (a minute or an hour matched): the
  minute's repeat would take an hour of the desk; the second's is the
  one measured.
