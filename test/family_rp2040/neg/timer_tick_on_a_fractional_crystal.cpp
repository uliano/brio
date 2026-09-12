// The 1 us tick divides clk_ref by a whole number: a 14.7456 MHz crystal
// cannot give it.
#include "rp2040/clock.hpp"
#include "rp2040/timer.hpp"
using Odd = brio::Clock<brio::ClockSource::crystal, 14'745'600, 14'745'600>;
void f() { constexpr Odd clock; (void)brio::Timer::init(clock); }
