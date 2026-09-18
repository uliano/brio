// Delay family smoke TU: the microsecond busy-wait in both of its
// faces, and the two shapes the IP strata ask a family for - a rate
// type with a maker, and a verb over it found by argument-dependent
// lookup (brio/pl022/spi.hpp's Pl022ChipDelay, brio/dw_apb_i2c/i2c.hpp's
// ruler members).
#include "rp2350/clock.hpp"
#include "rp2350/delay.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::pll, 150'000'000>;

static_assert(delay_us_cap == 1000u);
static_assert(delay_rate(150'000'000u).usable);
static_assert(!delay_rate(0u).usable);

// What the two IP strata require of a family's traits: the type, the
// maker, and a `delay_us` the type's own namespace supplies.
static_assert(requires(DelayRate rate, uint32_t us) {
    { delay_us(rate, us) } -> std::convertible_to<bool>;
});

void delay_verbs() {
    (void)delay_us(SysClock{}, 10u);
    (void)delay_us(delay_rate(SysClock::hz), 999u);
    (void)delay_us(delay_rate(SysClock::pclk_hz), 0u);
}
