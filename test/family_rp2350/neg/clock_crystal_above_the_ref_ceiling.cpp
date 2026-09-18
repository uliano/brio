// clk_ref may not exceed 25 MHz, and this task puts the crystal on it
// undivided: a 48 MHz crystal is refused rather than divided in silence.
#include "rp2350/clock.hpp"
using Bad = brio::Clock<brio::ClockSource::crystal, 48'000'000UL, 48'000'000UL>;
void f() { (void)Bad::init(); }
