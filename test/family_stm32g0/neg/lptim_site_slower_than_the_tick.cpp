// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// A resync quantized more coarsely than the kernel tick it repairs can
// advance a tick too many and mature a time event EARLY: 32768 / 128 is
// 256 counts a second against a 1000 Hz tick.
#include "stm32g0/platform_stm32.hpp"
#include "stm32g0/sleep.hpp"
using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
brio::Stm32LptimTimedSleepSite<
    brio::Stm32Platform, SysClock,
    brio::LptimTimedSleepConfig{.prescaler = brio::LptimPrescaler::div128}> site;
