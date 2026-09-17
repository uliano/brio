// mcu: ch32v203c8
// The microsecond wait REFUSES a request of one tick period or more and
// spends no time on it, so its answer is the only thing that says the
// time was served: dropping it must be REFUSED.
#include "ch32v203/clock.hpp"
#include "ch32v203/delay.hpp"

constexpr brio::Clock<brio::ClockSource::pll, 144'000'000> clock;

void f() { brio::delay_us(clock, 100); }
