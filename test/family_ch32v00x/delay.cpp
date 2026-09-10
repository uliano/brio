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

struct Follower {
    static void rebase(uint32_t) {}
};
using Dyn = DynamicClock<Fast, Follower>;
static_assert(delay_rates<Dyn>.size() == 16);
static_assert(delay_rates<Dyn>[0].cycles_per_us == 48);
static_assert(delay_rates<Dyn>[11].cycles_per_us == 3);
static_assert(delay_rates<Dyn>[15].cycles_per_us == 1);   // 187.5 kHz rounds up

void delay_verbs() {
    constexpr Fast fast;
    constexpr Reset slow;
    constexpr Dyn dyn;
    (void)delay_us(fast, 100);
    (void)delay_us(slow, 100);
    (void)delay_us(dyn, 100);
    (void)delay_us(delay_rate(24'000'000), 10);
}
