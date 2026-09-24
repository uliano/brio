// mcu: ch32v203f6 ch32v203g6 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// The HSE half of the clock chapter, on the seven parts whose package
// brings out OSC_IN and OSC_OUT. The crystal's rate is read from the
// part's own table rather than written here, because it is not the same
// number on all of them: 3..25 MHz on the CH32V20x_D6 parts and exactly
// 32 MHz on the CH32V203RB, whose oscillator has its load capacitors
// built in. The other two parts of the family have no oscillator pad at
// all and a neg TU proves the refusal.
#include "ch32vx03/clock.hpp"

using namespace brio;

static_assert(device::has_hse_pins);

/// The crystal this part's oscillator takes at the top of its range, and
/// the tree that runs SYSCLK straight off it with no PLL.
inline constexpr uint32_t xtal_hz = device::hse_max_hz;

using Crystal = Clock<ClockSource::crystal, xtal_hz, xtal_hz>;
using External = Clock<ClockSource::external, xtal_hz, xtal_hz>;
using Divided = Clock<ClockSource::crystal, xtal_hz / 8u, xtal_hz>;

static_assert(Crystal::uses_hse && Crystal::crystal_hz == xtal_hz);
static_assert(Crystal::sysclk_hz == xtal_hz && Crystal::hpre_code == 0u);
static_assert(Crystal::sysclk_source == SysclkSource::hse);
static_assert(!Crystal::hse_bypass && External::hse_bypass);
static_assert(hpre_divider(Divided::hpre_code) == 8u);
// No PLL in these trees, so no 48 MHz for the USB blocks.
static_assert(Crystal::usb_divider == 0u);

/// A crystal the PLL can reach the family ceiling from, which is not the
/// same part number on both classes: 8 MHz where PLLXTPRE divides by one
/// or two, 32 MHz where it divides by four or eight.
inline constexpr uint32_t pll_xtal_hz = 8'000'000UL * device::pll_hse_div[0];

using PllXtal = Clock<ClockSource::pll, 144'000'000, pll_xtal_hz>;
/// A rate only the SECOND divider reaches: 4 MHz into the PLL times
/// thirteen, whichever class the part belongs to.
using PllXtalDivided = Clock<ClockSource::pll, 52'000'000, pll_xtal_hz>;

// The PLL's input is 8 MHz on both classes - the D6 takes the crystal
// whole, the D8 divides its own by four - and the multiplier is the
// same eighteen.
static_assert(PllXtal::uses_hse && PllXtal::pll_in_hz == 8'000'000UL);
static_assert(!PllXtal::pll_input_divided && PllXtal::pll_mul == 18u);
static_assert(PllXtal::pll_in_div == device::pll_hse_div[0]);
static_assert(PllXtal::sysclk_hz == 144'000'000UL && PllXtal::usb_hz == 48'000'000UL);
static_assert(PllXtal::sysclk_source == SysclkSource::pll);
// And the divided path, which is /2 on one class and /8 on the other.
static_assert(PllXtalDivided::pll_input_divided && PllXtalDivided::pll_mul == 13u);
static_assert(PllXtalDivided::pll_in_div == device::pll_hse_div[1]);
static_assert(PllXtalDivided::pll_in_hz == 4'000'000UL);

void clock_hse_verbs() {
    (void)Rcc::hse_start(false);
    Rcc::hse_enable(false);
    (void)Rcc::hse_ready();
    Rcc::hse_stop();
    (void)Rcc::hse_start(true);
    Rcc::clock_monitor(true);
    (void)Rcc::clock_failed();
    Rcc::clock_monitor(false);
    Rcc::hse_stop();

    (void)Crystal::init();
    (void)External::init();
    (void)Divided::init();
    (void)PllXtal::init();
    (void)PllXtalDivided::init();
}
