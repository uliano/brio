// On the crystal, clk_sys IS the crystal's rate.
#include "rp2350/clock.hpp"
using Bad = brio::Clock<brio::ClockSource::crystal, 48'000'000, 12'000'000>;
void f() { (void)Bad::init(); }
