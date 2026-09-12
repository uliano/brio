// Must FAIL: a dormant is chip-wide and the tree is core 0's to
// restore - the timed site is core 0's platform's.
#include "rp2040/sleep.hpp"

using P1 = brio::Rp2040Platform<1>;
using SysClock = brio::Clock<brio::ClockSource::pll, 125'000'000>;
using Site = brio::Rp2040TimedSleepSite<P1, SysClock>;
bool f() { return Site::init(); }
