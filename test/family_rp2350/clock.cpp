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
static_assert(ClockIn<1>::pin == 22u);
static_assert(tick_cycles_per_us(12'000'000u) == 12u);
static_assert(tick_cycles_per_us(48'000'000u) == 48u);
static_assert(tick_cycles_per_us(32'768u) == 0u);   // not a whole megahertz
// 8.1.4's table: 2^interval microseconds up to code 6, then 125 us
// doubling to 32 ms at code 15.
static_assert(fc_interval_us(0) == 1u);
static_assert(fc_interval_us(6) == 64u);
static_assert(fc_interval_us(7) == 125u);
static_assert(fc_interval_us(15) == 32'000u);

void clock_verbs() {
    (void)SysClock::init();
    (void)SysClock::count_hz(CountSource::clk_sys);
    (void)XtalClock::init();

    (void)Xosc::init(SysClock::startup_delay, SysClock::xtal_hz);
    (void)Xosc::stable();
    (void)Xosc::enabled();
    (void)Xosc::startup_delay();
    Xosc::stop();

    (void)Xosc::startup_x4();
    Xosc::startup_x4(false);
    (void)Xosc::range();
    (void)Xosc::wait_periods(16u);
    (void)Xosc::count();
    (void)Xosc::badwrite();
    Xosc::clear_badwrite();

    (void)Rosc::start();
    (void)Rosc::running();
    (void)Rosc::stable();
    (void)Rosc::div_running();
    (void)Rosc::range_code();
    Rosc::range(RoscRange::medium);
    Rosc::range(RoscRange::low);
    Rosc::drive(0u, 1u);
    (void)Rosc::drive(7u);
    Rosc::randomise(true, true);
    (void)Rosc::randomised();
    Rosc::seed(0x12345678u);
    Rosc::divider(8u);
    (void)Rosc::divider();
    (void)Rosc::random_bit();
    (void)Rosc::wait_periods(16u);
    (void)Rosc::count();
    (void)Rosc::badwrite();
    Rosc::clear_badwrite();
    Rosc::stop();

    (void)Lposc::trim();
    Lposc::trim(32u);
    (void)Lposc::mode();
    (void)Lposc::freq_khz_int();
    (void)Lposc::freq_khz_frac();
    Lposc::freq_khz(32u, 0xc49cu);
    (void)Lposc::declared_hz();
    (void)Lposc::bad_password();
    Lposc::clear_bad_password();

    (void)PllSys::init(SysClock::pll);
    (void)PllSys::locked();
    (void)PllSys::unlocked();
    (void)(PllSys::config() == SysClock::pll);
    (void)PllSys::lock_lost();
    PllSys::clear_lock_lost();
    PllSys::lock_lost_interrupt(true);
    (void)PllSys::lock_lost_interrupt();
    (void)PllSys::interrupt_pending();
    PllSys::force_interrupt(false);
    (void)PllSys::isr();
    (void)PllSys::irq();
    PllSys::bypass(false);
    (void)PllSys::bypassed();
    (void)PllSys::stopped();
    PllSys::stop();
    (void)PllUsb::init(pll_config_for(12'000'000u, 48'000'000u));
    (void)PllUsb::irq();
    PllUsb::stop();

    (void)Clocks::ref_select(RefSource::xosc);
    (void)Clocks::ref_select(RefSource::lposc);
    (void)Clocks::ref_from_aux(RefAux::gpin0);
    (void)Clocks::ref_source();
    (void)Clocks::ref_aux_source();
    (void)Clocks::sys_from_ref();
    (void)Clocks::sys_from_aux(SysAux::pll_sys);
    (void)Clocks::sys_source();
    (void)Clocks::sys_aux_source();
    Clocks::ref_divider(1u);
    (void)Clocks::ref_divider();
    Clocks::sys_divider(1u, 0u);
    (void)Clocks::sys_divider();
    (void)Clocks::sys_divider_frac();
    (void)Clocks::peri_select(PeriAux::clk_sys);
    Clocks::peri_stop();
    (void)Clocks::peri_enabled();
    (void)Clocks::peri_source();
    (void)Clocks::peri_divider();
    (void)Clocks::usb_select(UsbAux::pll_usb);
    Clocks::usb_stop();
    (void)Clocks::usb_enabled();
    (void)Clocks::usb_source();
    (void)Clocks::usb_divider();
    (void)Clocks::adc_select(AdcAux::pll_usb);
    Clocks::adc_stop();
    (void)Clocks::adc_enabled();
    (void)Clocks::adc_source();
    (void)Clocks::adc_divider();
    (void)Clocks::hstx_select(HstxAux::clk_sys);
    Clocks::hstx_stop();
    (void)Clocks::hstx_enabled();
    (void)Clocks::hstx_source();
    (void)Clocks::hstx_divider();

    Clocks::wake_enables(sleep_clocks_all);
    (void)Clocks::wake_enables();
    Clocks::sleep_enables(sleep_clocks_all & ~sleep_clocks_none);
    (void)Clocks::sleep_enables();
    (void)(Clocks::enabled() == sleep_clocks_all);
    (void)(sleep_clocks_none | sleep_clocks_all);

    Resus::enable(255u);
    (void)Resus::enabled();
    (void)Resus::timeout();
    (void)Resus::resussed();
    Resus::force();
    Resus::unforce();
    Resus::clear();
    Resus::interrupt(false);
    (void)Resus::interrupt();
    (void)Resus::raw();
    (void)Resus::pending();
    Resus::force_interrupt(false);
    (void)Resus::isr();
    (void)Resus::irq();
    Resus::disable();

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
    (void)FreqCounter::count_hz(CountSource::clk_usb, 12'000'000u);
    (void)FreqCounter::count_hz(CountSource::clk_adc, 12'000'000u);
    (void)FreqCounter::count_hz(CountSource::rosc_ph, 12'000'000u);
    (void)FreqCounter::count_hz(CountSource::otp, 12'000'000u);
    (void)FreqCounter::measure(CountSource::clk_sys, 12'000'000u, 15u, 149'000u, 151'000u).pass;
    FreqCounter::start_delay(1u);
    (void)FreqCounter::start_delay();
    (void)FreqCounter::running();
    FreqCounter::stop();

    (void)ClockOut<0>::init(GpoutSource::clk_sys, 100u);
    (void)ClockOut<0>::enabled();
    (void)ClockOut<0>::running();
    (void)ClockOut<0>::source();
    (void)ClockOut<0>::divider();
    (void)ClockOut<0>::divider_frac();
    ClockOut<0>::duty_correction(true);
    (void)ClockOut<0>::duty_correction();
    ClockOut<0>::phase(1u);
    (void)ClockOut<0>::phase();
    ClockOut<0>::nudge();
    ClockOut<0>::stop();
    (void)ClockOut<1>::init(GpoutSource::lposc, 1u);
    (void)ClockOut<2>::init(GpoutSource::clk_hstx, 2u);
    (void)ClockOut<3>::init(GpoutSource::xosc, 1u, 0u);
    (void)ClockIn<0>::init();
    (void)ClockIn<1>::release();
    static_assert(ClockIn<0>::max_hz == 50'000'000u);
}
