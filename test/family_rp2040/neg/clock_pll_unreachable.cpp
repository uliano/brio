// 100.1 MHz has no exact ratio from 12 MHz within 2.18.2's constraints.
#include "rp2040/clock.hpp"
using Bad = brio::Clock<brio::ClockSource::pll, 100'100'000>;
void f() { (void)Bad::init(); }
