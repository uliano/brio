// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// Below 1 kHz the timed site's resync granularity is coarser than the
// kernel tick it is meant to repair.
#include "stm32g0/platform_stm32.hpp"
#include "stm32g0/sleep.hpp"
using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
brio::Stm32TimedSleepSite<brio::Stm32Platform, SysClock,
                          brio::TimedSleepConfig{.rtcclk_hz = 512}> site;
