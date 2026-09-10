// Delay family smoke TU: the cycles-per-microsecond factor at compile
// time, and the two wait verbs.
#include "ch32v00x/clock.hpp"
#include "ch32v00x/delay.hpp"

using namespace brio;

using Fast = Clock<ClockSource::pll, 48'000'000>;
using Reset = Clock<ClockSource::internal, 8'000'000>;

static_assert(delay_rate(48'000'000).cycles_per_us == 48);
static_assert(delay_rate(8'000'000).cycles_per_us == 8);
static_assert(delay_rate(48'000'001).cycles_per_us == 49);   // ceil: late, never early
static_assert(delay_rate(0).cycles_per_us == 0);

void delay_verbs() {
    constexpr Fast fast;
    constexpr Reset slow;
    (void)delay_us(fast, 100);
    (void)delay_us(slow, 100);
    (void)delay_us(delay_rate(24'000'000), 10);
}
