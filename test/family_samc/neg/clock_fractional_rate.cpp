// mcu: samc21e18a samc21g18a samc21j18a
// 48 MHz / 7 = 6.857142... MHz: a real OSC48M divider ratio, but not a
// whole number of hertz - Clock::hz would be a lie, so the rate is
// refused. (48/5 = 9.6 MHz, by contrast, IS exact and is accepted.)
#include "samc/clock.hpp"
using namespace brio;
void f() { (void)Clock<ClockSource::internal, 6'857'142>::init(); }
