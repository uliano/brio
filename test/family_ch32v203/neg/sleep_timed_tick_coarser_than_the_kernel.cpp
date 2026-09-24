// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// A RULER COARSER THAN WHAT IT MEASURES. The RTC's tick is what the
// timed site places deadlines on and measures slept spans with, so a
// tick slower than the kernel's own (1000 Hz here) can do neither: the
// alarm would land up to one of ITS periods late while the model's
// deadline guard counts in kernel ticks. 256 Hz out of the crystal is
// a legal prescaler and an illegal ruler.
#include "ch32v203/clock.hpp"
#include "ch32v203/platform.hpp"
#include "ch32v203/sleep.hpp"

using Clk = brio::Clock<brio::ClockSource::internal, 8'000'000>;
using P = brio::Ch32v203Platform<>;
using Timed = brio::Ch32v203TimedSleepSite<
    P, Clk, brio::TimedSleepConfig{.rtcclk_hz = 32'768, .tick_hz = 256}>;

void f() {
    (void)Timed::init();
}
