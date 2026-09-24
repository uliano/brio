// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// The microsecond wait REFUSES a request of one tick period or more and
// spends no time on it, so its answer is the only thing that says the
// time was served: dropping it must be REFUSED.
#include "ch32vx03/clock.hpp"
#include "ch32vx03/delay.hpp"

constexpr brio::Clock<brio::ClockSource::pll, 144'000'000> clock;

void f() { brio::delay_us(clock, 100); }
