// clk_sys may not exceed 150 MHz on this chip (datasheet 8.6.1).
#include "rp2350/clock.hpp"
using Bad = brio::Clock<brio::ClockSource::pll, 200'000'000>;
void f() { (void)Bad::init(); }
