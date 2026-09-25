// mcu: ch32x035r8 ch32x035c8 ch32x035g8u ch32x035g8r ch32x035f8 ch32x035f7 ch32x033f8
// The microsecond wait REFUSES a request of one tick period or more and
// spends no time on it, so its answer is the only thing that says the time
// was served: dropping it must be REFUSED.
#include "ch32x035/clock.hpp"
#include "ch32x035/delay.hpp"

constexpr brio::Clock<brio::ClockSource::internal, 48'000'000> clock;

void f() { brio::delay_us(clock, 100); }
