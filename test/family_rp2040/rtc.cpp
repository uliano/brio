// RTC family smoke TU: the calendar arithmetic, the alarm, the
// resource's verbs, the clk_rtc generator.
#include "rp2040/clock.hpp"
#include "rp2040/rtc.hpp"

using namespace brio;

static_assert(rtc_weekday_of(1970, 1, 1) == 4u);   // a Thursday
static_assert(rtc_days_in_month(2, 2000) == 29u);
static_assert(!rtc_datetime_valid({.year = 2026, .month = 2, .day = 29}));
static_assert(rtc_alarm_repeats({.match_hour = true, .match_minute = true, .match_second = true}));
static_assert(Rtc::crystal_divider == 256u);

using SysClock = Clock<ClockSource::pll, 125'000'000>;

void rtc_verbs() {
    constexpr SysClock clock;
    (void)Rtc::init(clock);
    (void)Rtc::init(clock, RtcClock::gpin0_1hz);
    (void)Rtc::clock_hz();
    (void)Rtc::reference_divider();
    (void)Rtc::running();
    (void)Rtc::enabled();
    (void)Rtc::set({.year = 2026, .month = 9, .day = 13, .weekday = rtc_weekday_of(2026, 9, 13), .hour = 12});
    (void)Rtc::adjust({.year = 2026, .month = 9, .day = 13, .hour = 13});
    Rtc::enable(false);
    Rtc::enable(true);
    (void)Rtc::read();
    Rtc::force_not_leap_year(true);
    (void)Rtc::force_not_leap_year();
    (void)Rtc::alarm({.at = {.second = 0}, .match_second = true});
    Rtc::arm_alarm();
    Rtc::disarm_alarm();
    (void)Rtc::alarm_armed();
    (void)Rtc::alarm_matching();
    (void)Rtc::alarm();
    Rtc::interrupt(true);
    (void)Rtc::raw_pending();
    (void)Rtc::pending();
    Rtc::force(false);
    (void)Rtc::isr();
    (void)Rtc::rearm_after();
    Rtc::release();
    Clocks::rtc_select(RtcAux::xosc, 256);
    (void)Clocks::rtc_enabled();
    (void)Clocks::rtc_source();
    (void)Clocks::rtc_divider256();
    Clocks::rtc_stop();
}
