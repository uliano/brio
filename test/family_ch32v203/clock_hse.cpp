// mcu: ch32v203f6 ch32v203g6 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// The HSE half of the clock chapter, on the seven parts whose package
// brings out OSC_IN and OSC_OUT. The crystal's rate is read from the
// part's own table rather than written here, because it is not the same
// number on all of them: 3..25 MHz on the CH32V20x_D6 parts and exactly
// 32 MHz on the CH32V203RB, whose oscillator has its load capacitors
// built in. The other two parts of the family have no oscillator pad at
// all and a neg TU proves the refusal.
#include "ch32v203/clock.hpp"

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
static_assert(!Crystal::hse_bypass && External::hse_bypass);
static_assert(hpre_divider(Divided::hpre_code) == 8u);
// No PLL in these trees, so no 48 MHz for the USB blocks.
static_assert(Crystal::usb_divider == 0u);

void clock_hse_verbs() {
    (void)Rcc::hse_start(false);
    Rcc::hse_stop();
    (void)Rcc::hse_start(true);
    Rcc::clock_monitor(true);
    Rcc::clock_monitor(false);
    Rcc::hse_stop();

    (void)Crystal::init();
    (void)External::init();
    (void)Divided::init();
}
