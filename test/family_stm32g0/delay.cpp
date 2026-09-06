// Delay family smoke TU: the SysTick microsecond busy-wait
// (stm32g0/delay.hpp). Nothing here is variant-dependent - SysTick is
// the core's - so what this proves is the arithmetic and that the two
// spellings (a compile-time Clock, a precomputed DelayRate) both
// compile against every device header.
#include "stm32g0/clock.hpp"
#include "stm32g0/delay.hpp"
#include "stm32g0/ticker.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::pll, 64'000'000>;
using SlowClock = Clock<ClockSource::internal, 2'000'000>;

// Ceil, always: the at-least direction.
static_assert(delay_rate(64'000'000UL).cycles_per_us == 64);
static_assert(delay_rate(16'000'000UL).cycles_per_us == 16);
static_assert(delay_rate(2'000'000UL).cycles_per_us == 2);
static_assert(delay_rate(125'000UL).cycles_per_us == 1);
static_assert(delay_rate(64'000'001UL).cycles_per_us == 65);
static_assert(delay_rate(0).cycles_per_us == 0);

// A dynamic clock's factors are a compile-time table over its pack,
// selected by the rate index at wait time (no division).
using Dyn = DynamicClock<Rates<SysClock, Clock<ClockSource::internal, 16'000'000>, SlowClock>,
                         SysTickCounter>;
static_assert(delay_rates<Dyn>.size() == 3);
static_assert(delay_rates<Dyn>[0].cycles_per_us == 64);
static_assert(delay_rates<Dyn>[1].cycles_per_us == 16);
static_assert(delay_rates<Dyn>[2].cycles_per_us == 2);

void delay_verbs() {
    constexpr SysClock fast;
    constexpr SlowClock slow;
    constexpr Dyn dyn;
    (void)delay_us(fast, 100);
    (void)delay_us(slow, 5);
    (void)delay_us(dyn, 50);
    (void)SysTickCounter::start(dyn);
    SysTickCounter::rebase(16'000'000);
    Ticker::rebase(2'000'000);
    (void)delay_us(fast, 65'535);          // refused at run time, not here
    (void)delay_us(delay_rate(SysClock::hz), 12);

    // The rate a driver rebased at run time would carry.
    DelayRate rate = delay_rate(clock_hz(fast));
    (void)delay_us(rate, 3);
}
