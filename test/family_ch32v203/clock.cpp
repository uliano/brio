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
static_assert(Full::pll_mul == 18u && !Full::pll_input_divided && Full::pll_in_hz == 8'000'000UL);
static_assert(Full::sysclk_hz == 144'000'000UL && Full::hpre_code == 0u);
static_assert(Usb48::sysclk_hz == 48'000'000UL && Usb48::pll_mul == 6u);
static_assert(Divided::sysclk_hz == 8'000'000UL && Divided::hz == 1'000'000UL);
static_assert(hpre_divider(Divided::hpre_code) == 8u);
// Which root each tree switches to, which is what a restore() compares
// against SWS.
static_assert(Full::sysclk_source == SysclkSource::pll);
static_assert(Boot::sysclk_source == SysclkSource::hsi);

// PB1 is capped at 72 MHz in this stratum: undivided at or below it,
// halved above. A timer on a DIVIDED bus counts twice the bus rate.
static_assert(Full::pclk1_hz == 72'000'000UL && Full::pclk2_hz == 144'000'000UL);
static_assert(Full::timclk1_hz == 144'000'000UL && Full::timclk2_hz == 144'000'000UL);
static_assert(Usb48::pclk1_hz == 48'000'000UL && Usb48::pclk2_hz == 48'000'000UL);
static_assert(Usb48::timclk1_hz == 48'000'000UL);
// The same arithmetic a rebased driver does from the HCLK it is handed.
static_assert(pclk1_hz_at(144'000'000UL) == 72'000'000UL);
static_assert(pclk1_hz_at(72'000'000UL) == 72'000'000UL);
static_assert(pclk1_hz_at(96'000'000UL) == 48'000'000UL);
static_assert(pclk2_hz_at(96'000'000UL) == 96'000'000UL);

// The USB blocks want 48 MHz and take the PLL divided by 1, 2 or 3 -
// and by 5 on the CH32V20x_D8 alone.
static_assert(Usb48::usb_divider == 1u && Usb48::usb_hz == 48'000'000UL);
static_assert(Full::usb_divider == 3u && Full::usb_hz == 48'000'000UL);
static_assert(Boot::usb_divider == 0u && Boot::usb_hz == 0u);
static_assert(usbpre_exists(1) && usbpre_exists(2) && usbpre_exists(3));
static_assert(usbpre_exists(5) == device::has_usb_pre_div5);
static_assert(!usbpre_exists(4));
static_assert(usbpre_code_for(1) == 0u && usbpre_code_for(3) == 2u && usbpre_code_for(5) == 3u);
static_assert(usbpre_divider(0) == 1u && usbpre_divider(2) == 3u && usbpre_divider(3) == 5u);

// The ADC's divider is part of the tree, and the family's ceiling is
// the one rate where no code keeps the converter in range: PCLK2 / 8 is
// 18 MHz against a 14 MHz rating.
static_assert(adcpre_divider(0) == 2u && adcpre_divider(3) == 8u);
static_assert(adcpre_for(48'000'000UL, adc_max_hz) == 1u);   // /4 = 12 MHz
static_assert(Usb48::adc_hz == 12'000'000UL && Usb48::adc_in_spec);
static_assert(Full::adc_code == 3u && Full::adc_hz == 18'000'000UL && !Full::adc_in_spec);
// And the rate above which a flash operation asks for HCLK halved.
static_assert(Full::flash_needs_halving && !Usb48::flash_needs_halving);

// The HPRE ladder SKIPS 32, which is the one trap in the field.
static_assert(hpre_divider(0) == 1u && hpre_divider(7) == 1u);
static_assert(hpre_divider(8) == 2u && hpre_divider(11) == 16u);
static_assert(hpre_divider(12) == 64u && hpre_divider(15) == 512u);
static_assert(hpre_for(144'000'000UL, 144'000'000UL) == 0u);
static_assert(hpre_for(144'000'000UL, 4'500'000UL) == 0xFF);   // /32 does not exist
static_assert(hpre_for(144'000'000UL, 100u) == 0xFF);
static_assert(pll_mul_exists(2) && pll_mul_exists(16) && pll_mul_exists(18));
static_assert(!pll_mul_exists(17) && !pll_mul_exists(1));

// The output pad is a part fact: two packages of the series do not
// bring PA8 out, and there the task answers false instead of refusing
// to compile.
static_assert(Mco::has_pad == ((device::port_pins('A') & (1u << 8)) != 0u));

// One encoding for both peripheral prescalers: 0xx undivided, then /2 up
// to /16.
static_assert(ppre_divider(0) == 1u && ppre_divider(3) == 1u);
static_assert(ppre_divider(4) == 2u && ppre_divider(7) == 16u);
static_assert(ppre_for(72'000'000UL, pb1_max_hz) == 0u);
static_assert(ppre_for(144'000'000UL, pb1_max_hz) == 4u);
static_assert(ppre_for(144'000'000UL, 8'000'000UL) == 7u);
static_assert(timclk_hz_at(72'000'000UL, 4) == 144'000'000UL);
static_assert(timclk_hz_at(72'000'000UL, 0) == 72'000'000UL);

void clock_verbs() {
    (void)Rcc::hsi_ready();
    (void)Rcc::hsi_start();
    Rcc::hsi_enable(true);
    Rcc::hsi_stop();
    Rcc::hsi_trim(16);
    (void)Rcc::hsi_trim();
    (void)Rcc::hsi_calibration();
    (void)Rcc::lsi_start();
    Rcc::lsi_enable(true);
    (void)Rcc::lsi_ready();
    Rcc::lsi_stop();
    (void)Rcc::pll_start(false, true, 12);
    (void)Rcc::pll_ready();
    (void)Rcc::pll_from_hse();
    (void)Rcc::pll_input_divided();
    (void)Rcc::pll_multiplier();
    Rcc::pll_stop();

    // The prescalers, all four of them, and what they read back.
    Rcc::prescalers(0, 0x4, 0);
    (void)Rcc::hpre_code();
    (void)Rcc::ppre1_code();
    (void)Rcc::ppre2_code();
    Rcc::adc_prescaler(3);
    (void)Rcc::adc_prescaler();
    Rcc::adc_duty_extended(true);
    (void)Rcc::adc_duty_extended();
    Rcc::eth_prescaler(true);
    (void)Rcc::usb_prescaler(2);
    (void)Rcc::usb_prescaler();

    (void)Rcc::sysclk_select(SysclkSource::hsi);
    (void)Rcc::sysclk_status();

    // The clock output, through the resource and through the task that
    // claims the pad with it.
    Rcc::mco(McoSource::sysclk);
    (void)Rcc::mco();
    Rcc::mco(McoSource::none);
    (void)Mco::init(McoSource::hsi);
    (void)Mco::init(McoSource::pll_div2, PinSpeed::slow);
    (void)Mco::source();
    Mco::off();

    // The security system and the ready interrupts.
    Rcc::clock_monitor(false);
    (void)Rcc::clock_monitor();
    (void)Rcc::clock_failed();
    (void)Rcc::css_isr();
    Rcc::ready_interrupts(rcc_hserdyie | rcc_pllrdyie);
    (void)Rcc::ready_interrupts();
    (void)Rcc::interrupt_flags();
    Rcc::clear_interrupt_flags(rcc_hserdyf);
    (void)Rcc::ready_isr();
    Rcc::hse_in_low_power(true);
    (void)Rcc::hse_in_low_power();

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

// ---- the dynamic regime ----------------------------------------------------
// A pack of rate tuples with a user that follows them. The user is a
// stand-in for the ticker and the port a real program lists: what is
// checked here is that the fan-out compiles, that the pack's surface is
// constexpr and that a driver can ask whether it is in the list.
struct RebasedUser {
    static inline uint32_t seen = 0;
    static void rebase(uint32_t hz) { seen = hz; }
};
struct StrangerUser {
    static void rebase(uint32_t) {}
};

using SysClock = DynamicClock<Rates<Full, Usb48, Boot>, RebasedUser>;

static_assert(!SysClock::is_static && SysClock::rate_count == 3);
static_assert(SysClock::rate_hz(0) == 144'000'000UL && SysClock::rate_hz(2) == 8'000'000UL);
static_assert(SysClock::rate_pclk1_hz(0) == 72'000'000UL);
static_assert(SysClock::rate_pclk2_hz(0) == 144'000'000UL);
static_assert(SysClock::rate_usb_hz(1) == 48'000'000UL && SysClock::rate_usb_hz(2) == 0u);
static_assert(SysClock::rate_source(2) == SysclkSource::hsi);
static_assert(SysClock::can_run_at(48'000'000UL) && !SysClock::can_run_at(72'000'000UL));
static_assert(SysClock::index_of(8'000'000UL) == 2u);
static_assert(SysClock::rebases<RebasedUser> && !SysClock::rebases<StrangerUser>);
// The contract every clocked driver asserts in its own init().
static_assert(clock_follows<SysClock, RebasedUser>());
static_assert(!clock_follows<SysClock, StrangerUser>());
static_assert(clock_follows<Full, StrangerUser>());   // a static clock follows nothing

void dynamic_clock_verbs() {
    (void)SysClock::init();
    (void)SysClock::set<48'000'000UL>();
    (void)SysClock::set(8'000'000UL);
    (void)SysClock::set_index<0>();
    (void)SysClock::set_index(1);
    (void)SysClock::restore();
    (void)SysClock::switching();
    (void)SysClock::hz();
    (void)SysClock::pclk1_hz();
    (void)SysClock::pclk2_hz();
    (void)SysClock::usb_hz();
    (void)SysClock::adc_hz();
    (void)SysClock::adc_in_spec();
    (void)SysClock::rate_index();
    (void)clock_hz(SysClock{});
}
