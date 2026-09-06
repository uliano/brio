// Clock family smoke TU: the RCC/PWR resources, the flash latency verb
// and the two built roots of the Clock task, with the PLL arithmetic
// pinned by the chapter's own numbers (RM0444 5.4.4).
#include "stm32g0/clock.hpp"
#include "stm32g0/flash.hpp"

using namespace brio;

// The 64 MHz road: M 1, N 8, R 2 - VCO 128 MHz.
static_assert(pll_config_for(64'000'000).m == 1);
static_assert(pll_config_for(64'000'000).n == 8);
static_assert(pll_config_for(64'000'000).r == 2);
static_assert(pll_output_hz(pll_config_for(64'000'000)) == 64'000'000);
// 48 MHz: M 1, N 12, R 4 (VCO 192) is the first exact hit at R 2? No:
// R 2 wants VCO 96 = N 6, below the floor; R 3 wants 144 = N 9 - exact.
static_assert(pll_config_for(48'000'000).r == 3);
static_assert(pll_config_for(48'000'000).n == 9);
// Unreachable: 65 MHz exceeds the ceiling, 1 MHz has no VCO in range.
static_assert(pll_config_for(65'000'000).m == 0);
static_assert(pll_config_for(1'000'000).m == 0);
// The chapter's limits, one by one.
static_assert(!pll_config_valid({.m = 1, .n = 5, .r = 2}));    // VCO 80 < 96
static_assert(!pll_config_valid({.m = 1, .n = 22, .r = 2}));   // VCO 352 > 344
static_assert(!pll_config_valid({.m = 8, .n = 60, .r = 2}));   // input 2 MHz < 2.66
static_assert(pll_config_valid({.m = 2, .n = 16, .r = 2}));    // 8 MHz in, 128 VCO, 64 out
static_assert(!pll_config_valid({.m = 1, .n = 8, .r = 1}));    // R 1 is Reserved

// HSISYS: 16 MHz over powers of two only.
static_assert(hsidiv_for(16'000'000) == 0);
static_assert(hsidiv_for(2'000'000) == 3);
static_assert(hsidiv_for(125'000) == 7);
static_assert(hsidiv_for(12'000'000) == 0xFF);

// Table 13, Range 1.
static_assert(FlashWaitStates::for_hz(24'000'000) == 0);
static_assert(FlashWaitStates::for_hz(24'000'001) == 1);
static_assert(FlashWaitStates::for_hz(48'000'000) == 1);
static_assert(FlashWaitStates::for_hz(64'000'000) == 2);
static_assert(FlashWaitStates::for_hz_range2(16'000'000) == 1);

using Fast = Clock<ClockSource::pll, 64'000'000>;
using Boot = Clock<ClockSource::internal, 16'000'000>;
using Slow = Clock<ClockSource::internal, 2'000'000>;
static_assert(Fast::is_static && Fast::hz == 64'000'000 && Fast::pclk_hz == 64'000'000);
static_assert(clock_hz(Boot{}) == 16'000'000);
static_assert(Slow::hsidiv == 3);
static_assert(Fast::power_regime == PowerRegime::range1 && Fast::wait_states == 2);
static_assert(Fast::sysclk_source == SysclkSource::pllrclk);
static_assert(Boot::sysclk_source == SysclkSource::hsisys);

// The voltage side of a rate: the Range 2 column of table 13, the two
// ceilings (16 MHz for Range 2, 2 MHz for low-power run - the negatives
// hold the refusals), and the regime a rate carries.
using Mid = Clock<ClockSource::internal, 16'000'000, PowerRegime::range2>;
using Lpr = Clock<ClockSource::internal, 2'000'000, PowerRegime::low_power_run>;
using Lpr8 = Clock<ClockSource::internal, 8'000'000, PowerRegime::range2>;
static_assert(Mid::wait_states == 1 && Lpr::wait_states == 0 && Lpr8::wait_states == 0);
static_assert(Mid::power_regime == PowerRegime::range2);
static_assert(Lpr::power_regime == PowerRegime::low_power_run);
static_assert(flash_wait_states_for(PowerRegime::range1, 16'000'000) == 0);
static_assert(flash_wait_states_for(PowerRegime::range2, 16'000'000) == 1);
static_assert(flash_wait_states_for(PowerRegime::low_power_run, 2'000'000) == 0);

// The dynamic clock over a pack of three tuples: the discrete-rate
// surface, the users list, the first-match rule of set<hz>() and the
// by-position verbs.
struct UserA { static void rebase(uint32_t) {} };
struct UserB { static void rebase(uint32_t) {} };
struct NotAUser {};
using Dyn = DynamicClock<Rates<Fast, Mid, Lpr>, UserA, UserB>;
static_assert(!Dyn::is_static);
static_assert(Dyn::rate_count == 3);
static_assert(Dyn::rate_hz(0) == 64'000'000 && Dyn::rate_hz(1) == 16'000'000 &&
              Dyn::rate_hz(2) == 2'000'000);
static_assert(Dyn::rate_regime(2) == PowerRegime::low_power_run);
static_assert(Dyn::rate_source(0) == SysclkSource::pllrclk &&
              Dyn::rate_source(1) == SysclkSource::hsisys);
static_assert(Dyn::can_run_at(16'000'000) && !Dyn::can_run_at(8'000'000));
static_assert(Dyn::index_of(2'000'000) == 2 && Dyn::index_of(1) == Dyn::rate_count);
static_assert(Dyn::rebases<UserA> && Dyn::rebases<UserB> && !Dyn::rebases<NotAUser>);
static_assert(clock_follows<Dyn, UserA>() && !clock_follows<Dyn, NotAUser>());
static_assert(clock_follows<Fast, NotAUser>());   // a static clock follows nothing
// Two rates sharing an hz: set<hz>() takes the first, set_index() names one.
using Twins = DynamicClock<Rates<Boot, Mid>>;
static_assert(Twins::index_of(16'000'000) == 0 && Twins::rate_regime(1) == PowerRegime::range2);
// The boot rate alone is a legal pack.
using Lone = DynamicClock<Rates<Fast>, UserA>;
static_assert(Lone::rate_count == 1);

void dynamic_verbs() {
    constexpr Dyn clock;
    (void)Dyn::init();
    (void)clock_hz(clock);
    (void)Dyn::hz();
    (void)Dyn::pclk_hz();
    (void)Dyn::rate_index();
    (void)Dyn::power_regime();
    (void)Dyn::set<2'000'000>();
    (void)Dyn::set(64'000'000);
    (void)Dyn::set_index<1>();
    (void)Dyn::set_index(2);
    (void)Dyn::restore();
    (void)Twins::set_index<1>();
    (void)Lone::init();
}

void clock_verbs() {
    (void)Fast::init();
    (void)Boot::init();
    (void)Slow::init();
    (void)Mid::init();
    (void)Lpr::init();

    Rcc::hsi_enable(true);
    (void)Rcc::hsi_ready();
    (void)Rcc::hsi_wait_ready();
    Rcc::hsi_div(1);
    (void)Rcc::hsi_div();
    Rcc::pll_enable(false);
    (void)Rcc::pll_wait(false);
    (void)Rcc::pll_configure({.m = 1, .n = 8, .r = 2});
    Rcc::sysclk_select(SysclkSource::hsisys);
    (void)Rcc::sysclk_status();
    (void)Rcc::sysclk_wait(SysclkSource::hsisys);
    Rcc::bus_prescalers_unity();
    (void)Rcc::bus_prescalers_are_unity();
    Rcc::io_clock('A', true);
    (void)Rcc::io_clock('A');
    Rcc::ahb_clock(RCC_AHBENR_DMA1EN, true);
    Rcc::apb1_clock(RCC_APBENR1_USART2EN, true);
    Rcc::apb2_clock(RCC_APBENR2_USART1EN, true);
    Rcc::kernel_clock(RCC_CCIPR_USART1SEL_Pos, 0);
    (void)Rcc::kernel_clock(RCC_CCIPR_USART1SEL_Pos);
    (void)Rcc::mco(Rcc::mco_hsi16_code, 6);
    Rcc::mco_off();
    (void)Pwr::range();
    (void)Pwr::range(2);
    (void)Pwr::range_changing();
    (void)Pwr::low_power_run();
    (void)Pwr::low_power_run(false);
    (void)Pwr::on_low_power_regulator();

    (void)FlashWaitStates::get();
    (void)FlashWaitStates::set(2);
    (void)FlashAccel::prefetch();
    FlashAccel::prefetch(false);
    (void)FlashAccel::instruction_cache();
    (void)flash_size_kb();
}
