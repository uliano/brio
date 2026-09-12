// test_rp2040_rtc - the reference bench suite for the RP2040's RTC
// (rp2040/rtc.hpp over datasheet 4.8), measured with NO WIRE: the
// frequency counter on clk_rtc, the system timer on the calendar's
// second and on the alarm's latency, the calendar's own boundaries
// crossed one second at a time.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// NOTHING TO WIRE. GP0/GP1 are the console's, GP25 the LED.
//
// THE RULERS: the clock chapter's frequency counter for clk_rtc, the
// system timer for everything under a second.
//
// What is exercised, letter by letter:
//   a  the block, WIRELESS: the calendar arithmetic (both leap rules,
//      the days of a month, the weekday of three known dates, the
//      refusals), clk_rtc COUNTED at 46875 Hz, the reset needing the
//      clock, the reference divider
//   b  SET AND READ: a date set and read back, the time from the
//      write to RTC_ACTIVE, the second's period against the timer,
//      three seconds counted
//   c  THE BOUNDARIES, one second each: the year, 29 February in a
//      leap year and its absence in another, the end of April, the
//      week's wrap, and 2100 - the silicon's leap year that is not,
//      with FORCE_NOTLEAPYEAR and without
//   d  THE ALARM: a second matched, the latency from the calendar's
//      rollover to the interrupt; the match as a LEVEL (the interrupt
//      re-entered for the whole matching second when left armed); the
//      repeat re-armed after the second; a full date matched once; a
//      wrong weekday never
//   e  the calendar adjusted while running, stopped and restarted, the
//      forced interrupt
//
// build: boards = pico,weact2040
// build: monitor_speed = 115200

#include <stdint.h>

#include "rp2040/clock.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/rtc.hpp"
#include "rp2040/ticker.hpp"
#include "rp2040/timer.hpp"
#include "rp2040/uart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using SysClock = Clock<ClockSource::pll, 125'000'000>;
constexpr SysClock clock;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;
using Led = Pin<25>;

TestBench<Serial, 16> bench;

volatile uint32_t rtc_isr_entries = 0;
volatile uint32_t rtc_fired_at = 0;
volatile bool rtc_leave_armed = false;   // the storm probe: the handler does not disarm

uint32_t us_now() { return Timer::now_low(); }
void spin_us(uint32_t us) {
    const uint32_t t0 = us_now();
    while (us_now() - t0 < us) {
    }
}
bool within(uint32_t got, uint32_t want, uint32_t per_mille) {
    const uint32_t tol = static_cast<uint32_t>(static_cast<uint64_t>(want) * per_mille / 1000u);
    return got + tol >= want && got <= want + tol;
}

void print_dt(const RtcDateTime& d) {
    print(serial, d.year, "-", d.month, "-", d.day, " (", d.weekday, ") ", d.hour, ":", d.minute, ":", d.second);
}

/// Wait for the calendar's second to change; the timer's stamp of the
/// change and the new value. 0 when it never did within `us`.
uint32_t next_second(RtcDateTime& out, uint32_t us = 1'500'000u) {
    const auto first = Rtc::read();
    if (!first) {
        return 0;
    }
    const uint32_t t0 = us_now();
    for (;;) {
        const auto now = Rtc::read();
        if (now && now->second != first->second) {
            out = *now;
            return us_now();
        }
        if (us_now() - t0 > us) {
            return 0;
        }
    }
}

bool rtc_up() {
    rtc_leave_armed = false;
    Rtc::disarm_alarm();
    Rtc::interrupt(false);
    return Rtc::init(clock);
}

// =============================================================================
// a - the block, wireless
// =============================================================================
void ta_block() {
    bench.verdict("the two leap rules: 2024 is a leap year to both, 2100 to the silicon alone (the year "
                  "FORCE_NOTLEAPYEAR is for), 2000 to both; February has 29 days in 2024 and 28 in 2100",
                  rtc_silicon_leap(2024) && rtc_gregorian_leap(2024) && rtc_needs_force_not_leap(2100) &&
                      rtc_gregorian_leap(2000) && !rtc_needs_force_not_leap(2000) && rtc_days_in_month(2, 2024) == 29u &&
                      rtc_days_in_month(2, 2100) == 28u);
    bench.verdict("the weekday of a date, Sunday 0: 13 September 2026 a Sunday, 1 January 2000 a Saturday, 29 February "
                  "2024 a Thursday, 1 January 1970 a Thursday",
                  rtc_weekday_of(2026, 9, 13) == 0u && rtc_weekday_of(2000, 1, 1) == 6u && rtc_weekday_of(2024, 2, 29) == 4u &&
                      rtc_weekday_of(1970, 1, 1) == 4u);
    bench.verdict("the refusals: a 4096th year, a 13th month, a 30 February, a 7th weekday, a 24th hour are no date; an "
                  "alarm's unmatched field is not checked",
                  !rtc_datetime_valid({.year = 4096}) && !rtc_datetime_valid({.month = 13}) &&
                      !rtc_datetime_valid({.month = 2, .day = 30}) && !rtc_datetime_valid({.weekday = 7}) &&
                      !rtc_datetime_valid({.hour = 24}) && rtc_alarm_valid({.at = {.month = 13}}) &&
                      !rtc_alarm_valid({.at = {.month = 13}, .match_month = true}));
    Clocks::rtc_stop();
    const bool unclocked = Resets::cycle(ResetBlock::rtc);
    const bool up = rtc_up();
    const auto rtc_hz = SysClock::count_hz(CountSource::clk_rtc);
    print(serial, "  the reset with clk_rtc stopped: ", unclocked ? "completed" : "never completes", "; init: ", up ? "ok" : "FAILED",
          ", clk_rtc counted ", rtc_hz ? *rtc_hz : 0u, " Hz (", Rtc::clock_hz(), " stated), CLKDIV_M1 + 1 = ", Rtc::reference_divider(),
          ", CTRL=", hex(Rtc::regs().CTRL), crlf);
    bench.verdict("the block's reset completes WITHOUT clk_rtc (the converter's does not); init() puts the crystal over "
                  "256 on clk_rtc - counted at 46875 Hz within a tenth of a per cent - and 46874 in CLKDIV_M1; the "
                  "calendar not started",
                  unclocked && up && rtc_hz && within(*rtc_hz, 46'875, 1) && Rtc::reference_divider() == 46'875u && !Rtc::running() &&
                      !Rtc::enabled());
}

// =============================================================================
// b - set and read
// =============================================================================
void tb_set_read() {
    (void)rtc_up();
    const RtcDateTime d{.year = 2026, .month = 9, .day = 13, .weekday = rtc_weekday_of(2026, 9, 13), .hour = 12, .minute = 30, .second = 0};
    const uint32_t t0 = us_now();
    const bool set = Rtc::set(d);
    const uint32_t took = us_now() - t0;
    const auto back = Rtc::read();
    print(serial, "  set ");
    print_dt(d);
    print(serial, ": ", set ? "running" : "NOT running", " after ", took, " us; read back ");
    if (back) {
        print_dt(*back);
    }
    print(serial, crlf);
    bench.verdict("a date set is read back the same, set() returning once the read path shows it: within 500 us "
                  "(RTC_ACTIVE after two clk_rtc periods, the enable's own tick some 130 us later, the value loaded "
                  "again after it)",
                  set && back && *back == d && took < 500u);
    RtcDateTime s1{};
    RtcDateTime s2{};
    const uint32_t e1 = next_second(s1);
    const uint32_t e2 = next_second(s2);
    const uint32_t period = e2 - e1;
    print(serial, "  the first rollover ", (e1 - t0) / 1000u, " ms after the set, two rollovers ", period, " us apart (", s1.second, " then ",
          s2.second, ")", crlf);
    bench.verdict("the calendar's second is 1.000 s on the timer within a tenth of a per cent, the seconds consecutive",
                  e1 != 0u && e2 != 0u && within(period, 1'000'000, 1) && s2.second == static_cast<uint8_t>(s1.second + 1u));
    // The first second after a set is a whole one whatever the phase
    // the set lands at: three sets at three delays after a rollover.
    const uint32_t delays_ms[] = {100, 400, 700};
    uint8_t whole = 0;
    for (uint32_t dly : delays_ms) {
        RtcDateTime r{};
        (void)next_second(r);
        spin_us(dly * 1000u);
        const uint32_t t_set = us_now();
        (void)Rtc::set(d);
        RtcDateTime f{};
        const uint32_t e = next_second(f);
        const uint32_t first_us = e - t_set;
        const bool good = e != 0u && within(first_us, 1'000'000, 2) && f.second == 1u;
        print(serial, "  set ", dly, " ms after a rollover: the first rollover ", first_us, " us after the set, into :", f.second,
              good ? "" : "  OUT", crlf);
        if (good) {
            ++whole;
        }
    }
    bench.verdict("the first second after a set is a whole one, into the value's next second, at three phases of the "
                  "set (the enable's tick waited for and undone)",
                  whole == 3u);
    const auto a = Rtc::read();
    spin_us(3'000'000);
    const auto b = Rtc::read();
    const int32_t elapsed = a && b ? static_cast<int32_t>(b->second) - static_cast<int32_t>(a->second) : -1;
    bench.verdict("three seconds of the timer are three seconds of the calendar", elapsed == 3);
    bench.verdict("read() answers nothing while the calendar is stopped", (Rtc::enable(false), spin_us(100), !Rtc::read().has_value()));
    Rtc::enable(true);
}

// =============================================================================
// c - the boundaries
// =============================================================================
void tc_boundaries() {
    (void)rtc_up();
    struct Case {
        const char* name;
        RtcDateTime from;
        RtcDateTime to;
        bool force_not_leap;
    };
    const Case cases[] = {
        {"the year", {2025, 12, 31, 3, 23, 59, 59}, {2026, 1, 1, 4, 0, 0, 0}, false},
        {"29 February 2024", {2024, 2, 28, 3, 23, 59, 59}, {2024, 2, 29, 4, 0, 0, 0}, false},
        {"1 March 2023", {2023, 2, 28, 2, 23, 59, 59}, {2023, 3, 1, 3, 0, 0, 0}, false},
        {"1 March 2024", {2024, 2, 29, 4, 23, 59, 59}, {2024, 3, 1, 5, 0, 0, 0}, false},
        {"the end of April", {2026, 4, 30, 4, 23, 59, 59}, {2026, 5, 1, 5, 0, 0, 0}, false},
        {"the week's wrap", {2026, 9, 12, 6, 23, 59, 59}, {2026, 9, 13, 0, 0, 0, 0}, false},
        {"2100, the silicon's rule", {2100, 2, 28, 0, 23, 59, 59}, {2100, 2, 29, 1, 0, 0, 0}, false},
        {"2100 under FORCE_NOTLEAPYEAR", {2100, 2, 28, 0, 23, 59, 59}, {2100, 3, 1, 1, 0, 0, 0}, true},
    };
    uint8_t ok = 0;
    for (const Case& c : cases) {
        Rtc::force_not_leap_year(c.force_not_leap);
        // 2100's 29 February is not a date to set(): the silicon's rule
        // is what crosses into it, and set() checks the Gregorian one.
        const bool set = Rtc::set(c.from);
        RtcDateTime next{};
        const uint32_t at = next_second(next);
        const bool good = set && at != 0u && next == c.to;
        print(serial, "  ", c.name, ": ");
        print_dt(c.from);
        print(serial, " -> ");
        print_dt(next);
        print(serial, good ? "" : "  WRONG", crlf);
        if (good) {
            ++ok;
        }
    }
    Rtc::force_not_leap_year(false);
    bench.verdict("eight boundaries crossed one second each as the chapter says: the year and the weekday roll, 29 "
                  "February in 2024 and not in 2023, April ends, and 2100 gets a 29 February the Gregorian calendar "
                  "has not - unless FORCE_NOTLEAPYEAR",
                  ok == 8u);
}

// =============================================================================
// d - the alarm
// =============================================================================
void td_alarm() {
    (void)rtc_up();
    Nvic::enable(Rtc::irq());
    (void)Rtc::set({2026, 9, 13, 0, 12, 0, 0});
    rtc_isr_entries = 0;
    rtc_fired_at = 0;
    const bool armed = Rtc::alarm({.at = {.second = 3}, .match_second = true});
    Rtc::interrupt(true);
    // The rollover into :03 on the timer, then the interrupt's stamp.
    RtcDateTime seen{};
    uint32_t rolled = 0;
    for (uint8_t i = 0; i < 4u && !(seen.second == 3u); ++i) {
        rolled = next_second(seen);
    }
    spin_us(2000);
    const uint32_t fired = rtc_fired_at;
    const int32_t latency = static_cast<int32_t>(fired - rolled);
    print(serial, "  alarm on second 3: the calendar rolled into :", seen.second, ", the interrupt ", fired != 0u ? "fired " : "NEVER FIRED",
          fired != 0u ? latency : 0, " us after the read that saw it, ", rtc_isr_entries, " entries, the match ",
          Rtc::alarm_matching() ? "active" : "over", ", armed=", Rtc::alarm_armed(), crlf);
    bench.verdict("a second-matched alarm fires at the rollover into that second, within 200 us of the read that saw "
                  "it, ONCE - the ISR body masks the line and disarms the match",
                  armed && seen.second == 3u && fired != 0u && latency > -200 && latency < 200 && rtc_isr_entries == 1u &&
                      !Rtc::alarm_armed());
    // The match as a level: the handler leaves it armed through the
    // matching second (bounded: the handler disarms at the thousandth).
    (void)Rtc::set({2026, 9, 13, 0, 12, 0, 0});
    rtc_isr_entries = 0;
    rtc_leave_armed = true;
    Rtc::arm_alarm();
    Rtc::interrupt(true);   // the ISR body masked it
    spin_us(4'500'000);
    rtc_leave_armed = false;
    Rtc::disarm_alarm();
    const uint32_t storm = rtc_isr_entries;
    print(serial, "  the match left armed through second 3: ", storm, " interrupt entries", crlf);
    bench.verdict("the alarm is a LEVEL: a handler that leaves the match armed is re-entered for the whole matching second "
                  "(a thousand entries, the bound, in one second)",
                  storm >= 1000u);
    // The repeat: re-armed after the second, the next :03 five seconds
    // after a jump to :58.
    (void)Rtc::set({2026, 9, 13, 0, 12, 0, 58});
    rtc_isr_entries = 0;
    rtc_fired_at = 0;
    const bool rearmed = Rtc::rearm_after();
    const uint32_t t0 = us_now();
    while (rtc_fired_at == 0u && us_now() - t0 < 7'000'000u) {
    }
    const uint32_t waited = rtc_fired_at - t0;
    const auto when = Rtc::read();
    print(serial, "  re-armed from :58: fired after ", waited / 1000u, " ms at ");
    if (when) {
        print_dt(*when);
    }
    print(serial, crlf);
    bench.verdict("a repeating alarm re-armed after its second fires at the next minute's :03, five seconds after a jump "
                  "to :58",
                  rearmed && rtc_fired_at != 0u && within(waited, 5'000'000, 20) && when && when->second == 3u && when->minute == 1u);
    // A full date matched: once, then never again.
    (void)Rtc::set({2026, 9, 13, 0, 12, 0, 0});
    rtc_isr_entries = 0;
    rtc_fired_at = 0;
    (void)Rtc::alarm({.at = {2026, 9, 13, 0, 12, 0, 2}, .match_year = true, .match_month = true, .match_day = true,
                      .match_weekday = true, .match_hour = true, .match_minute = true, .match_second = true});
    Rtc::interrupt(true);
    spin_us(3'500'000);
    const uint32_t once = rtc_isr_entries;
    const bool repeats = rtc_alarm_repeats(Rtc::alarm());
    bench.verdict("a fully matched alarm is a one-shot: it fires once at 12:00:02 and reads as non-repeating",
                  once == 1u && rtc_fired_at != 0u && !repeats);
    // A wrong weekday: never.
    (void)Rtc::set({2026, 9, 13, 0, 12, 0, 0});
    rtc_isr_entries = 0;
    (void)Rtc::alarm({.at = {.weekday = 3, .second = 2}, .match_weekday = true, .match_second = true});
    Rtc::interrupt(true);
    spin_us(3'500'000);
    Rtc::disarm_alarm();
    bench.verdict("an alarm on the wrong weekday and second 2 never fires on a Sunday", rtc_isr_entries == 0u);
    Rtc::interrupt(false);
    Nvic::disable(Rtc::irq());
}

// =============================================================================
// e - adjust, stop, force
// =============================================================================
void te_adjust() {
    (void)rtc_up();
    (void)Rtc::set({2026, 9, 13, 0, 10, 0, 0});
    spin_us(200'000);
    const bool adjusted = Rtc::adjust({2026, 9, 13, 0, 11, 0, 0});
    spin_us(200);
    const auto after = Rtc::read();
    print(serial, "  adjusted while running from 10:00:00 to 11:00:00: ");
    if (after) {
        print_dt(*after);
    }
    print(serial, crlf);
    bench.verdict("a calendar adjusted while running takes the new value (LOAD, two clk_rtc periods later)",
                  adjusted && after && after->hour == 11u && after->minute == 0u && after->second == 0u);
    Rtc::enable(false);
    spin_us(100);
    const bool stopped = !Rtc::running();
    Rtc::enable(true);
    spin_us(100);
    const auto r1 = Rtc::read();
    spin_us(1'200'000);
    const auto r2 = Rtc::read();
    bench.verdict("stopped, the block reports not running and read() declines; restarted, it counts on",
                  stopped && r1 && r2 && r2->second != r1->second);
    Nvic::disable(Rtc::irq());
    Rtc::interrupt(false);
    Rtc::force(true);
    const bool forced = Rtc::pending() && !Rtc::raw_pending();
    Rtc::force(false);
    bench.verdict("the forced interrupt appears in the status with no raw flag and leaves with the force",
                  forced && !Rtc::pending());
}

void banner() {
    print(serial, crlf, "test_rp2040_rtc - the RP2040 RTC (datasheet 4.8): clk_rtc = crystal / 256, nothing to wire; clk_sys=",
          SysClock::hz, " Hz", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_uart0() { (void)Serial::isr(); }
extern "C" void isr_systick() { brio::Ticker::tick(); }
extern "C" void isr_rtc() {
    rtc_isr_entries = rtc_isr_entries + 1u;
    if (rtc_leave_armed) {
        if (rtc_isr_entries >= 1000u) {
            brio::Rtc::disarm_alarm();
        }
        return;
    }
    if (brio::Rtc::isr()) {
        if (rtc_fired_at == 0u) {
            rtc_fired_at = us_now();
        }
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool timer_ok = brio::Timer::init(clock);
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the block, wireless: the arithmetic, clk_rtc counted", ta_block);
    bench.letter('b', "set and read, the second on the timer", tb_set_read);
    bench.letter('c', "the boundaries, one second each", tc_boundaries);
    bench.letter('d', "the alarm: latency, the level, the repeat, the one-shot", td_alarm);
    bench.letter('e', "adjust, stop, the forced interrupt", te_adjust);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL125" : "FAILED", " timer=", timer_ok ? "1us" : "FAILED",
                    " tick=", tick_ok ? "SysTick" : "FAILED", brio::crlf);
        banner();
        bench.prompt();
    }

    for (;;) {
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        brio::print(serial, static_cast<char>(c), brio::crlf);
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        bench.prompt();
    }
}
