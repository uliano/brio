// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// The timed site's FAST alarm clock must be one of the divided-RTCCLK
// codes: ck_spre is the LONG one, chosen automatically for deadlines the
// fast clock's sixteen bits cannot hold, and naming it here would leave
// the site with no fine resolution at all.
#include "stm32g0/platform_stm32.hpp"
#include "stm32g0/sleep.hpp"
using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
brio::Stm32TimedSleepSite<
    brio::Stm32Platform, SysClock,
    brio::TimedSleepConfig{.fast_clock = brio::RtcWakeupClock::ck_spre}> site;
