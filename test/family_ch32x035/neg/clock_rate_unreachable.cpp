// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// The one root is the 48 MHz HSI and HPRE divides it by 1..8 or a power
// of two to 256: 7 MHz is not a whole division, and is refused.
#include "ch32x035/clock.hpp"

using Odd = brio::Clock<brio::ClockSource::internal, 7'000'000>;
void f() { (void)Odd::init(); }
