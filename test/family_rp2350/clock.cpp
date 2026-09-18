// Clock family smoke TU: the oscillators, both PLLs, the generators,
// the tick generators, the frequency counter, the GPIO clock pins and
// the task - every verb instantiated once, and the compile-time ratio
// search checked against the datasheet's own example.
#include "rp2350/clock.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::pll, 150'000'000>;
using XtalClock = Clock<ClockSource::crystal, 12'000'000>;
using PeriOnXtal = Clock<ClockSource::pll, 150'000'000, 12'000'000, PeriSource::crystal>;

static_assert(SysClock::hz == 150'000'000u);
static_assert(SysClock::pclk_hz == 150'000'000u);
static_assert(SysClock::ref_hz == 12'000'000u);
static_assert(SysClock::is_static);
static_assert(PeriOnXtal::pclk_hz == 12'000'000u);
static_assert(XtalClock::hz == 12'000'000u);

// 8.6.1's own worked ratio: 150 MHz from 12 MHz is FBDIV 125 (a VCO of
// 1500 MHz) over 5 x 2, the larger post divider first.
static_assert(SysClock::pll.fbdiv == 125u);
static_assert(SysClock::pll.postdiv1 == 5u);
static_assert(SysClock::pll.postdiv2 == 2u);
static_assert(SysClock::pll.refdiv == 1u);
static_assert(pll_config_for(12'000'000u, 48'000'000u).valid());
static_assert(!pll_config_for(12'000'000u, 123'456'789u).valid());
static_assert(!pll_config_for(12'000'000u, 200'000'000u).valid());
static_assert(xosc_startup_delay(12'000'000u) == 47u);
static_assert(xosc_range_code(12'000'000u) == XOSC_CTRL_FREQ_RANGE_VALUE_1_15MHZ);
static_assert(xosc_range_code(25'000'000u) == XOSC_CTRL_FREQ_RANGE_VALUE_10_30MHZ);
static_assert(xosc_range_code(50'000'000u) == XOSC_CTRL_FREQ_RANGE_VALUE_25_60MHZ);
static_assert(gpout_pin(0) == 21u && gpout_pin(3) == 25u);
static_assert(gpin_pin(0) == 20u && gpin_pin(1) == 22u);
static_assert(ClockIn<0>::count_source == CountSource::gpin0);

void clock_verbs() {
    (void)SysClock::init();
    (void)SysClock::count_hz(CountSource::clk_sys);
    (void)XtalClock::init();

    (void)Xosc::init(SysClock::startup_delay, SysClock::xtal_hz);
    (void)Xosc::stable();
    (void)Xosc::enabled();
    (void)Xosc::startup_delay();
    Xosc::stop();

    (void)Rosc::start();
    (void)Rosc::running();
    (void)Rosc::stable();
    Rosc::stop();

    (void)PllSys::init(SysClock::pll);
    (void)PllSys::locked();
    (void)(PllSys::config() == SysClock::pll);
    PllSys::stop();
    (void)PllUsb::init(pll_config_for(12'000'000u, 48'000'000u));
    PllUsb::stop();

    (void)Clocks::ref_select(RefSource::xosc);
    (void)Clocks::ref_source();
    (void)Clocks::sys_from_ref();
    (void)Clocks::sys_from_aux(SysAux::pll_sys);
    (void)Clocks::sys_source();
    Clocks::ref_divider(1u);
    Clocks::sys_divider(1u);
    (void)Clocks::sys_divider();
    Clocks::peri_select(PeriAux::clk_sys);
    (void)Clocks::peri_enabled();
    (void)Clocks::peri_source();

    (void)TickGenerator<TickConsumer::riscv>::start(12u);
    (void)TickGenerator<TickConsumer::riscv>::running();
    (void)TickGenerator<TickConsumer::riscv>::cycles();
    (void)TickGenerator<TickConsumer::riscv>::count();
    TickGenerator<TickConsumer::riscv>::stop();
    (void)TickGenerator<TickConsumer::proc0>::start(12u);
    (void)TickGenerator<TickConsumer::proc1>::start(12u);
    (void)TickGenerator<TickConsumer::timer0>::start(12u);
    (void)TickGenerator<TickConsumer::timer1>::start(12u);
    (void)TickGenerator<TickConsumer::watchdog>::start(12u);

    (void)FreqCounter::count_hz(CountSource::xosc, 12'000'000u);
    (void)FreqCounter::count_hz(CountSource::clk_peri, 12'000'000u, 10u);
    (void)FreqCounter::count_hz(CountSource::lposc, 12'000'000u);
    (void)FreqCounter::count_hz(CountSource::clk_hstx, 12'000'000u);

    (void)ClockOut<0>::init(GpoutSource::clk_sys, 100u);
    (void)ClockOut<0>::enabled();
    ClockOut<0>::stop();
    (void)ClockOut<3>::init(GpoutSource::xosc, 1u, 0u);
    (void)ClockIn<0>::init();
    (void)ClockIn<1>::release();
}
