// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// A timed sleep site needs a counter that survives the Stop it arms:
// 26.5 names LSE and LSI, and PCLK stops with the whole VCORE domain.
#include "stm32g0/platform.hpp"
#include "stm32g0/sleep.hpp"
using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
brio::Stm32g0LptimTimedSleepSite<
    brio::Stm32g0Platform<>, SysClock,
    brio::LptimTimedSleepConfig{.source = brio::LptimClock::pclk}> site;
