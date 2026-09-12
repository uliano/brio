// On the crystal source clk_sys IS the crystal's rate.
#include "rp2040/clock.hpp"
using Bad = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
void f() { (void)Bad::init(); }
