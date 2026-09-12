// Clock family smoke TU: the crystal and PLL resources, the generators,
// both built sources of the task, and the PLL arithmetic of 2.18.2.
#include "rp2040/clock.hpp"

using namespace brio;

// 2.16.3's own example: 12 MHz, 1 ms -> 47.
static_assert(xosc_startup_delay(12'000'000) == 47u);
static_assert(xosc_startup_delay(15'000'000) == 59u);

// 2.18.2's examples: 125 MHz = 1500 MHz VCO / 6 / 2 (the highest VCO),
// 100 MHz from FBDIV 100 with 6 x 2, 48 MHz with the 1440 MHz VCO.
static_assert(pll_config_for(12'000'000, 125'000'000).fbdiv == 125);
static_assert(pll_config_for(12'000'000, 125'000'000).postdiv1 == 6);
static_assert(pll_config_for(12'000'000, 125'000'000).postdiv2 == 2);
static_assert(pll_config_for(12'000'000, 48'000'000).fbdiv == 120);
static_assert(pll_config_for(12'000'000, 48'000'000).postdiv1 == 6);
static_assert(pll_config_for(12'000'000, 48'000'000).postdiv2 == 5);
static_assert(pll_config_for(12'000'000, 133'000'000).valid());
static_assert(pll_config_for(12'000'000, 127'000'000).valid());     // 1524 / 6 / 2: any whole MHz above 15
static_assert(!pll_config_for(12'000'000, 12'000'000).valid());     // 750 MHz / 49 is the floor: the crystal source's rate
static_assert(!pll_config_for(12'000'000, 134'000'000).valid());    // past the ceiling
static_assert(!pll_config_for(12'000'000, 100'100'000).valid());    // no exact ratio
static_assert(!pll_config_for(4'000'000, 48'000'000).valid());      // reference below 5 MHz

using Fast = Clock<ClockSource::pll, 125'000'000>;
using Slow = Clock<ClockSource::crystal, 12'000'000>;
static_assert(Fast::hz == 125'000'000u && Fast::pclk_hz == Fast::hz && Fast::is_static);
static_assert(Slow::hz == 12'000'000u);
static_assert(Fast::startup_delay == 47u);
static_assert(clock_hz(Fast{}) == 125'000'000u);

void clock_verbs() {
    (void)Fast::init();
    (void)Slow::init();
    (void)Xosc::init(47);
    (void)Xosc::stable();
    (void)Xosc::enabled();
    Xosc::stop();
    (void)PllSys::init(Fast::pll);
    (void)PllSys::locked();
    PllSys::stop();
    (void)Clocks::ref_select(RefSource::xosc);
    (void)Clocks::sys_from_ref();
    (void)Clocks::sys_from_aux(SysAux::pll_sys);
    (void)Clocks::sys_source();
    Clocks::ref_divider(1);
    Clocks::sys_divider(1);
    Clocks::peri_select(PeriAux::xosc);
    (void)Clocks::peri_enabled();
}

// The ring oscillator, the counter, the clock pins and the crystal-fed
// clk_peri.
using PeriOnCrystal = Clock<ClockSource::pll, 125'000'000, 12'000'000, PeriSource::crystal>;
static_assert(PeriOnCrystal::pclk_hz == 12'000'000u && PeriOnCrystal::hz == 125'000'000u);
static_assert(Fast::peri_source == PeriSource::sys);
static_assert(gpout_pin(0) == 21u && gpout_pin(3) == 25u && gpin_pin(1) == 22u);
static_assert(ClockOut<2>::pin == 24u && ClockIn<0>::count_source == CountSource::gpin0);

void clock_measures() {
    (void)Rosc::running();
    (void)Rosc::stable();
    (void)Rosc::start();
    Rosc::stop();
    (void)Xosc::startup_delay();
    (void)(PllSys::config() == Fast::pll);
    (void)Clocks::peri_source();
    (void)Clocks::sys_divider();
    (void)FreqCounter::count_hz(CountSource::clk_sys, 12'000'000u);
    (void)Fast::count_hz(CountSource::rosc, 10);
    (void)PeriOnCrystal::init();
    (void)ClockOut<0>::init(GpoutSource::xosc, 12);
    (void)ClockOut<0>::enabled();
    ClockOut<0>::stop();
    ClockIn<1>::init();
    ClockIn<1>::release();
}
