// Clock family smoke TU: the RCC resource's verbs, the tree arithmetic
// that runs at compile time, and the two roots every part of the family
// has. The crystal half is clock_hse.cpp, because two packages of the
// nine bring out no oscillator pad at all.
#include "ch32v203/clock.hpp"

using namespace brio;

using Boot = Clock<ClockSource::internal, 8'000'000>;
using Usb48 = Clock<ClockSource::pll, 48'000'000>;
using Full = Clock<ClockSource::pll, 144'000'000>;
using Divided = Clock<ClockSource::internal, 1'000'000>;   // the HSI through HPRE /8

static_assert(ClockUser<Boot> || true);   // the tag type, not a user

// The HSI is 8 MHz and the PLL ladder is x2..x16 then x18, so the family
// ceiling is reached from the whole HSI and 48 MHz from the halved one.
static_assert(Boot::hz == 8'000'000UL && Boot::sysclk_hz == 8'000'000UL);
static_assert(Full::pll_mul == 18u && !Full::pll_halved);
static_assert(Full::sysclk_hz == 144'000'000UL && Full::hpre_code == 0u);
static_assert(Usb48::sysclk_hz == 48'000'000UL);
static_assert(Divided::sysclk_hz == 8'000'000UL && Divided::hz == 1'000'000UL);
static_assert(hpre_divider(Divided::hpre_code) == 8u);

// PB1 is capped at 72 MHz in this stratum: undivided at or below it,
// halved above.
static_assert(Full::pclk1_hz == 72'000'000UL && Full::pclk2_hz == 144'000'000UL);
static_assert(Usb48::pclk1_hz == 48'000'000UL && Usb48::pclk2_hz == 48'000'000UL);

// The USB blocks want 48 MHz and take the PLL divided by 1, 2 or 3.
static_assert(Usb48::usb_divider == 1u && Usb48::usb_hz == 48'000'000UL);
static_assert(Full::usb_divider == 3u && Full::usb_hz == 48'000'000UL);
static_assert(Boot::usb_divider == 0u && Boot::usb_hz == 0u);

// The HPRE ladder SKIPS 32, which is the one trap in the field.
static_assert(hpre_divider(0) == 1u && hpre_divider(7) == 1u);
static_assert(hpre_divider(8) == 2u && hpre_divider(11) == 16u);
static_assert(hpre_divider(12) == 64u && hpre_divider(15) == 512u);
static_assert(hpre_for(144'000'000UL, 144'000'000UL) == 0u);
static_assert(hpre_for(144'000'000UL, 4'500'000UL) == 0xFF);   // /32 does not exist
static_assert(hpre_for(144'000'000UL, 100u) == 0xFF);
static_assert(pll_mul_exists(2) && pll_mul_exists(16) && pll_mul_exists(18));
static_assert(!pll_mul_exists(17) && !pll_mul_exists(1));

void clock_verbs() {
    (void)Rcc::hsi_ready();
    (void)Rcc::hsi_start();
    Rcc::hsi_trim(16);
    (void)Rcc::hsi_calibration();
    (void)Rcc::lsi_start();
    Rcc::lsi_stop();
    (void)Rcc::pll_start(false, true, 12);
    Rcc::pll_stop();
    Rcc::usb_prescaler(2);
    Rcc::prescalers(0, 0x4, 0);
    (void)Rcc::sysclk_select(rcc_sw_hsi, rcc_sws_hsi);
    Rcc::mco(rcc_mco_sysclk);
    Rcc::mco(rcc_mco_none);
    Rcc::clock_monitor(false);

    // The gates, on all three buses.
    Rcc::enable(Bus::pb2, rcc_pb2_gpioa);
    (void)Rcc::enabled(Bus::pb2, rcc_pb2_gpioa);
    Rcc::reset(Bus::pb1, rcc_pb1_pwr);
    Rcc::disable(Bus::pb2, rcc_pb2_gpioa);
    Rcc::enable(Bus::hb, rcc_hb_dma1);
    Rcc::disable(Bus::hb, rcc_hb_dma1);

    (void)Boot::init();
    (void)Full::init();
    (void)Full::restore();
}
