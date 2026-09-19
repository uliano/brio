// The timed site is CORE 0's: a dormant is chip-wide and the clock tree
// is core 0's to restore.
#include "rp2350/sleep.hpp"
using SysClock = brio::Clock<brio::ClockSource::pll, 150'000'000>;
using Bad = brio::Rp2350TimedSleepSite<brio::Rp2350Platform<1>, SysClock>;
void f() { (void)Bad::arm(brio::SleepDepth::standby); }
