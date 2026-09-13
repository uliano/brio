// Delay family smoke TU: the core stratum's delay_us over a Cortex-M4
// header - the include-order contract (the device header first, then
// armv6m/delay.hpp) and the static-clock fold, on every header.
#include "stm32f4/clock.hpp"
#include "stm32f4/delay.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::hsi, 16'000'000>;

static_assert(delay_rate(16'000'000u).cycles_per_us == 16);

void delay_verbs() {
    constexpr SysClock clock;
    (void)delay_us(clock, 10u);
    (void)delay_us(delay_rate(16'000'000u), 5u);
}
