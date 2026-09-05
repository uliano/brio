// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// A timed site repairs a timebase that STOPS in Stop: it pauses the
// SysTick ticker and hands it the frozen span through advance(). A
// platform on a Tickless timebase keeps counting and places its own
// wake from idle_until(), so there is nothing to repair and no second
// alarm to own - the plain Stm32g0SleepSite is its only site.
#include "stm32g0/platform.hpp"
#include "stm32g0/sleep.hpp"
using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
struct FakeTickless {
    static uint32_t ticks() { return 0; }
    static constexpr uint32_t ticks_per_second = 1024;
    static bool arm_wake(uint32_t, uint32_t) { return true; }
};
brio::Stm32g0TimedSleepSite<brio::Stm32g0Platform<FakeTickless>, SysClock> site;
