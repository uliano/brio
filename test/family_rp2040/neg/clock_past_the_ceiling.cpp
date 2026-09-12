// clk_sys tops out at 133 MHz.
#include "rp2040/clock.hpp"
using Bad = brio::Clock<brio::ClockSource::pll, 150'000'000>;
void f() { (void)Bad::init(); }
