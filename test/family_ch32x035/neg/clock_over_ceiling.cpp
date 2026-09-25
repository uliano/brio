// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// No PLL and no crystal: nothing on this series runs above the HSI's 48
// MHz, and 96 MHz is refused.
#include "ch32x035/clock.hpp"

using Fast = brio::Clock<brio::ClockSource::internal, 96'000'000>;
void f() { (void)Fast::init(); }
