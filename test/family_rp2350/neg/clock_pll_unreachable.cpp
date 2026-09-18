// No exact ratio from a 12 MHz crystal reaches this rate: the search
// refuses rather than rounding.
#include "rp2350/clock.hpp"
using Bad = brio::Clock<brio::ClockSource::pll, 123'456'789>;
void f() { (void)Bad::init(); }
