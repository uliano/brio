// mcu: stm32g030xx stm32g070xx stm32g0b0xx
// The LPTIM timed sleep site rides an LPTIM, and the value line has none:
// sleep.hpp gates the site on the same symbol lptim.hpp gates its
// resource on, so the site's name is unknown there while the RTC-backed
// sites above it compile unchanged.
#include "stm32g0/clock.hpp"
#include "stm32g0/platform.hpp"
#include "stm32g0/sleep.hpp"
using Sys = brio::Clock<brio::ClockSource::pll, 64'000'000>;
using Site = brio::Stm32g0LptimTimedSleepSite<brio::Stm32g0Platform<>, Sys>;
void f() { (void)Site::init(); }
