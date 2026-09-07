// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// The LPTIM timed site on a tickless platform: the same refusal as the
// RTC one, and doubly so - a tickless program's LPTIM IS this site's
// counter, promoted to the timebase itself (stm32g0/lptim_ticker.hpp).
#include "stm32g0/platform.hpp"
#include "stm32g0/sleep.hpp"
using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
struct FakeTickless {
    static uint32_t ticks() { return 0; }
    static constexpr uint32_t ticks_per_second = 1024;
    static bool arm_wake(uint32_t, uint32_t) { return true; }
    static bool park() { return true; }
};
brio::Stm32g0LptimTimedSleepSite<brio::Stm32g0Platform<FakeTickless>, SysClock> site;
