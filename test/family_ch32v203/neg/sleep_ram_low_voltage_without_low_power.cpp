// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// A STOP CONFIGURATION THE SILICON HAS NOT. RAMLV is "valid when the
// LPDS bit of PWR_CTLR register is 1" (RM 2.4.1's own note on the bit),
// so the RAM's low-voltage mode over the MAIN regulator is not a rung
// of this ladder - and a site is where a program names one, so a site
// is where it is refused.
#include "ch32v203/clock.hpp"
#include "ch32v203/sleep.hpp"

using Clk = brio::Clock<brio::ClockSource::internal, 8'000'000>;
using Site = brio::Ch32v203SleepSite<
    Clk, brio::SleepSiteConfig{brio::StopConfig{brio::StopRegulator::main, true},
                               brio::StopConfig{brio::StopRegulator::low_power, false}}>;

void f() {
    (void)Site::arm(brio::SleepDepth::standby);
}
