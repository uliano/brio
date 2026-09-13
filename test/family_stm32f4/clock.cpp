// Clock family smoke TU: the RCC resource, the PWR regulator verbs, the
// flash latency and the Clock task, on every header of the pack. The
// 16 MHz reset rate compiles everywhere; the ladder-dependent rates are
// instantiated only where the reserve knows the part's ladder - through
// a template parameter, because a discarded branch of a non-template
// would still be checked - and the two PLL constructions the bench boards
// use are asserted to the ratio.
#include "stm32f4/clock.hpp"
#include "stm32f4/flash.hpp"
#include "stm32f4/pwr.hpp"

using namespace brio;

// ---- the arithmetic, header-independent --------------------------------------------
static_assert(pll_config_for(8'000'000u, 180'000'000u, true).m == 4);
static_assert(pll_config_for(8'000'000u, 180'000'000u, true).n == 180);
static_assert(pll_config_for(8'000'000u, 180'000'000u, true).p == 2);
static_assert(pll_config_for(8'000'000u, 180'000'000u, true).q == 8);      // 360 / 8 = 45 MHz
static_assert(pll_config_for(8'000'000u, 168'000'000u, true).q == 7);      // 336 / 7 = 48 MHz
static_assert(pll_config_for(16'000'000u, 84'000'000u, false).m == 8);
static_assert(pll_config_for(25'000'000u, 100'000'000u, true).m == 16);   // 25 / 16 = 1.5625 MHz
static_assert(pll_config_for(25'000'000u, 100'000'000u, true).n == 128);
static_assert(pll_config_for(16'000'000u, 1'000'000u, false).m == 0);     // below the VCO's floor
static_assert(apb_divider_for(180'000'000u, 45'000'000u) == 4);
static_assert(apb_divider_for(180'000'000u, 90'000'000u) == 2);
static_assert(apb_divider_for(16'000'000u, 45'000'000u) == 1);
static_assert(ppre_code(1) == 0 && ppre_code(2) == 4 && ppre_code(16) == 7);

// ---- the reset rate, on every header --------------------------------------------------
using Boot = Clock<ClockSource::hsi, 16'000'000>;
static_assert(Boot::is_static && Boot::hz == 16'000'000 && Boot::pclk1_hz == 16'000'000 &&
              Boot::pclk2_hz == 16'000'000 && Boot::usb_hz == 0 && Boot::wait_states == 0);
static_assert(clock_hz(Boot{}) == 16'000'000);
static_assert(apb_hz(Boot{}, true) == 16'000'000);

// The HSE undivided: no ladder needed either.
using Ext = Clock<ClockSource::hse, 8'000'000, 8'000'000, HseMode::bypass>;
static_assert(Ext::hz == 8'000'000 && Ext::uses_hse && !Ext::uses_pll);

// ---- the ladder-dependent rates, where the ladder is known ------------------------------
template <bool known = sysclk_ladder().known>
void ladder_rates() {
    if constexpr (known) {
        // A rate every known ladder reaches: 84 MHz from HSI.
        using Mid = Clock<ClockSource::pll_hsi, known ? 84'000'000 : 16'000'000>;
        static_assert(Mid::pll.m == 8 && Mid::regime.known && !Mid::regime.over_drive);
        static_assert(Mid::wait_states == 2 || Mid::wait_states == 3);   // 84 MHz: 2 WS on the 30/60/90 ladders, 3 on the F411's
        (void)Mid::init();
        // The part's ceiling, whatever it is, from the 8 MHz crystal.
        constexpr uint32_t top = sysclk_max_hz();
        using Top = Clock<ClockSource::pll_hse, known ? top : 16'000'000, 8'000'000>;
        static_assert(Top::regime.known);
        static_assert(Top::pclk1_hz <= sysclk_ladder().apb1_max_hz);
        static_assert(Top::pclk2_hz <= sysclk_ladder().apb2_max_hz);
        (void)Top::init();
    }
}

// The over-drive fact follows the header's ODEN.
static_assert(Pwr::has_over_drive() == pwr_has_over_drive());
static_assert(Pwr::scale_exists(VoltageScale::scale1));
static_assert(Pwr::scale_exists(VoltageScale::scale3) == pwr_vos_two_bits());

void resource_verbs() {
    (void)Boot::init();
    (void)Ext::init();
    ladder_rates();

    Rcc::hsi_enable(true);
    (void)Rcc::hsi_ready();
    (void)Rcc::hsi_wait_ready();
    Rcc::hse_enable(true, false);
    (void)Rcc::hse_ready();
    (void)Rcc::hse_wait_ready();
    (void)Rcc::hse_bypassed();
    Rcc::css(false);
    (void)Rcc::pll_configure(PllConfig{8, 180, 2, 8, true});
    Rcc::pll_enable(true);
    (void)Rcc::pll_ready();
    (void)Rcc::pll_wait(true);
    Rcc::sysclk_select(SysclkSource::hsi);
    (void)Rcc::sysclk_status();
    (void)Rcc::sysclk_wait(SysclkSource::hsi);
    Rcc::bus_prescalers(4, 2);
    (void)Rcc::apb1_divider();
    (void)Rcc::apb2_divider();
    (void)Rcc::ahb_undivided();
    (void)Rcc::mco1(Rcc::mco1_hse_code, 2);
#if defined(RCC_CFGR_MCO2)
    (void)Rcc::mco2(Rcc::mco2_sysclk_code, 4);
#endif
    Rcc::io_clock('A', true);
    (void)Rcc::io_clock('A');
    Rcc::ahb1_clock(RCC_AHB1ENR_DMA1EN, true);
    (void)Rcc::ahb1_clock(RCC_AHB1ENR_DMA1EN);
    Rcc::ahb1_reset(RCC_AHB1ENR_DMA1EN);
    Rcc::apb1_clock(RCC_APB1ENR_USART2EN, true);
    (void)Rcc::apb1_clock(RCC_APB1ENR_USART2EN);
    Rcc::apb1_reset(RCC_APB1ENR_USART2EN);
    Rcc::apb2_clock(RCC_APB2ENR_USART1EN, true);
    (void)Rcc::apb2_clock(RCC_APB2ENR_USART1EN);
    Rcc::apb2_reset(RCC_APB2ENR_USART1EN);

    Pwr::bus_clock(true);
    (void)Pwr::bus_clock();
    (void)Pwr::scale(VoltageScale::scale1);
    (void)Pwr::scale();
    (void)Pwr::scale_ready();
    (void)Pwr::over_drive_enter();
    Pwr::over_drive_exit();
    (void)Pwr::over_drive_active();

    (void)FlashWaitStates::get();
    (void)FlashWaitStates::set(0);
    (void)FlashWaitStates::needed_for(16'000'000u);
    FlashAccel::prefetch(true);
    (void)FlashAccel::prefetch();
    FlashAccel::icache(true);
    (void)FlashAccel::icache();
    FlashAccel::dcache(true);
    (void)FlashAccel::dcache();
    (void)FlashAccel::icache_reset();
    (void)FlashAccel::dcache_reset();
    FlashAccel::enable_all();
    (void)DeviceUid::read();
    (void)flash_size_kbytes();
    (void)DeviceIdcode::read();
}
