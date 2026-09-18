// A microsecond tick divides clk_ref by a whole number of cycles, so a
// reference that is not a whole number of megahertz has no setting.
#include "rp2350/timer.hpp"
using Odd = brio::Clock<brio::ClockSource::crystal, 12'500'000, 12'500'000>;
void f() { (void)brio::Timer<0>::init(Odd{}); }
