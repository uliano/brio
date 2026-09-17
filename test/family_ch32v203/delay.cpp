// delay_us family smoke TU: the cycles-per-microsecond factor, which is
// pure arithmetic and so checkable here, and the wait itself over a
// static clock.
#include "ch32v203/clock.hpp"
#include "ch32v203/delay.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::pll, 48'000'000>;
using FastClock = Clock<ClockSource::pll, 144'000'000>;

// Ceil, never floor: a wait of one microsecond at 48 MHz counts 48
// cycles, and a rate that is not a whole number of megahertz rounds UP,
// so the wait is late and never early.
static_assert(delay_rate(48'000'000UL).cycles_per_us == 48u);
static_assert(delay_rate(144'000'000UL).cycles_per_us == 144u);
static_assert(delay_rate(8'000'000UL).cycles_per_us == 8u);
static_assert(delay_rate(24'500'000UL).cycles_per_us == 25u);
static_assert(delay_rate(0u).cycles_per_us == 0u);

void delay_verbs() {
    constexpr SysClock slow;
    constexpr FastClock fast;
    (void)delay_us(slow, 1);
    (void)delay_us(fast, 100);
    (void)delay_us(delay_rate(clock_hz(fast)), 250);
    // Refused, and no time spent: a factor of zero, and a wait of a tick
    // period or more.
    (void)delay_us(DelayRate{0}, 10);
    (void)delay_us(fast, 65'536u);
}
