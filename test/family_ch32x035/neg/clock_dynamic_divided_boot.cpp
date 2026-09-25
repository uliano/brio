// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// A dynamic clock's Boot names the root at its undivided rate - the
// divider is what set() changes - so a Boot at 24 MHz is refused.
#include "ch32x035/clock.hpp"
#include "ch32x035/ticker.hpp"

using Boot = brio::Clock<brio::ClockSource::internal, 24'000'000>;
using SysClock = brio::DynamicClock<Boot, brio::Ticker>;
void f() { (void)SysClock::init(); }
