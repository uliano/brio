// A timed site whose RTCCLK cannot divide a second a thousand ways would
// resync more coarsely than the kernel tick it repairs, and could mature
// an event EARLY.
// mcu: stm32f411xe stm32f446xx
#include "stm32f4/sleep.hpp"
using Boot = brio::Clock<brio::ClockSource::hsi, 16'000'000>;
using Site = brio::Stm32f4TimedSleepSite<brio::Stm32f4Platform<>, Boot,
                                         brio::TimedSleepConfig{.rtcclk_hz = 512}>;
void f() { (void)Site::init(); }
