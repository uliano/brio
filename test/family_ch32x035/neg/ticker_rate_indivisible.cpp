// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// The tick rate must divide 1000, so millis() stays exact: 300 Hz is
// refused.
#include "ch32x035/clock.hpp"
#include "ch32x035/ticker.hpp"

using Odd = brio::BasicTicker<300>;
void f() {
    constexpr brio::Clock<brio::ClockSource::internal, 48'000'000> clock;
    (void)Odd::init(clock);
}
