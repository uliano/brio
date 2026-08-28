// CLKCTRL-equivalent family smoke TU: the OSCCTRL / GCLK / MCLK
// resources and the static Clock task. All three blocks are identical
// across the E/G/J variants, so what this TU really proves is that the
// task's compile-time arithmetic and every resource verb instantiate
// from the device header alone.
#include "samc/clock.hpp"
#include "util/clock.hpp"   // clock_hz: the contract the task satisfies

using namespace brio;

using SysClock = Clock<ClockSource::internal, 48'000'000>;
using SlowClock = Clock<ClockSource::internal, 3'000'000>;

static_assert(SysClock::is_static);
static_assert(SysClock::hz == 48'000'000);
static_assert(SysClock::divider == 1);
static_assert(SlowClock::divider == 16);
static_assert(clock_hz(SysClock{}) == 48'000'000);

// The exact-ratio rule and the wait-state table, at compile time.
static_assert(osc48m_div_for(48'000'000) == 0);
static_assert(osc48m_div_for(24'000'000) == 1);
static_assert(osc48m_div_for(4'000'000) == 11);   // the reset divider
static_assert(osc48m_div_for(9'600'000) == 4);    // 48/5 IS a whole number of Hz
static_assert(osc48m_div_for(6'857'142) == 0xFF); // 48/7 is not
static_assert(osc48m_div_for(10'000'000) == 0xFF);// no ratio produces it
static_assert(FlashWaitStates::for_hz(19'000'000) == 0);
static_assert(FlashWaitStates::for_hz(24'000'000) == 1);
static_assert(FlashWaitStates::for_hz(48'000'000) == 2);

void clock_tasks() {
    (void)SysClock::init();
    (void)SlowClock::init();
}

void oscillator_verbs() {
    Osc48m::enable(true);
    Osc48m::on_demand(false);
    Osc48m::run_standby(true);
    Osc48m::startup(OSCCTRL_OSC48MSTUP_STARTUP_CYCLE32_Val);
    (void)Osc48m::enabled();
    (void)Osc48m::on_demand();
    (void)Osc48m::ready();
    (void)Osc48m::wait_ready();
    (void)Osc48m::divider(2);
    (void)Osc48m::divider(17);   // refused at run time: the field is 1..16
    (void)Osc48m::divider();
    (void)Osc48m::sync_busy();
    (void)Osc48m::wait_sync();
    (void)FlashWaitStates::get();
    FlashWaitStates::set(2);
}

// ---- XOSC: the external multipurpose crystal oscillator --------------------

// The gain chooser is a table with edges, so the edges are what is
// asserted: each code covers UP TO its own frequency.
static_assert(xosc_gain_for(400'000) == XoscGain::up_to_2mhz);
static_assert(xosc_gain_for(2'000'000) == XoscGain::up_to_2mhz);
static_assert(xosc_gain_for(2'000'001) == XoscGain::up_to_4mhz);
static_assert(xosc_gain_for(16'000'000) == XoscGain::up_to_16mhz);
static_assert(xosc_gain_for(24'000'000) == XoscGain::up_to_32mhz);

static_assert(xosc_startup_us(0) == 31);
static_assert(xosc_startup_us(4) == 496);   // 31 x 2^4, the table's 488

// The safe clock must be no faster than the crystal it stands in for.
static_assert(cfd_prescaler_for(24'000'000, 48'000'000) == 1);
static_assert(cfd_prescaler_for(48'000'000, 48'000'000) == 0);
static_assert(cfd_prescaler_for(400'000, 48'000'000) == 7);

static_assert(Xosc::config_valid(XoscConfig{.hz = 24'000'000}));
static_assert(!Xosc::config_valid(XoscConfig{.hz = 0}));
static_assert(!Xosc::config_valid(XoscConfig{.hz = 40'000'000}));   // past 32 MHz
static_assert(!Xosc::config_valid(XoscConfig{.hz = 100'000}));      // below 0.4 MHz
static_assert(!Xosc::config_valid(XoscConfig{.hz = 24'000'000, .startup = 16}));

// A crystal configuration carries XTALEN and the derived gain; an
// external clock carries neither.
static_assert((Xosc::ctrl_word(XoscConfig{.hz = 24'000'000}) &
               OSCCTRL_XOSCCTRL_XTALEN_Msk) != 0u);
static_assert((Xosc::ctrl_word(XoscConfig{.hz = 24'000'000}) &
               OSCCTRL_XOSCCTRL_GAIN_Msk) ==
              OSCCTRL_XOSCCTRL_GAIN(static_cast<uint32_t>(XoscGain::up_to_32mhz)));
static_assert((Xosc::ctrl_word(XoscConfig{.hz = 24'000'000, .crystal = false}) &
               (OSCCTRL_XOSCCTRL_XTALEN_Msk | OSCCTRL_XOSCCTRL_GAIN_Msk)) == 0u);

void xosc_verbs() {
    (void)Xosc::init(XoscConfig{.hz = 24'000'000, .startup = 4}, 1000);
    (void)Xosc::init(XoscConfig{.hz = 8'000'000,
                                .crystal = false,
                                .on_demand = true,
                                .run_standby = true},
                     1000);
    (void)Xosc::init(XoscConfig{.hz = 24'000'000,
                                .gain_from_hz = false,
                                .gain = XoscGain::up_to_32mhz,
                                .automatic_gain = true,
                                .failure_detector = true,
                                .cfd_prescaler = 1},
                     1000);
    (void)Xosc::enabled();
    (void)Xosc::ready();
    (void)Xosc::failing();
    (void)Xosc::switched_to_safe_clock();
    Xosc::failure_detector(true);
    (void)Xosc::failure_detector();
    Xosc::cfd_prescaler(3);
    (void)Xosc::cfd_prescaler();
    Xosc::switch_back();
    (void)Xosc::switch_back_pending();
    (void)Xosc::reg();
    Xosc::stop();
}

void oscctrl_block_verbs() {
    (void)Oscctrl::irq();
    (void)Oscctrl::status();
    (void)Oscctrl::flags();
    (void)Oscctrl::armed();
    Oscctrl::arm(OscctrlFlag::xosc_ready | OscctrlFlag::dpll_lock_rise);
    Oscctrl::disarm(OscctrlFlag::all);
    Oscctrl::clear_flags();
    (void)Oscctrl::isr();
    Oscctrl::failure_event(true);
    (void)Oscctrl::failure_event();
    static_assert(Oscctrl::failure_generator == EVENT_ID_GEN_OSCCTRL_XOSC_FAIL);
}

// ---- FDPLL96M --------------------------------------------------------------

// The ratio chooser, in sixteenths of the reference.
static_assert(dpll_ratio_for(2'000'000, 48'000'000).ldr == 23);
static_assert(dpll_ratio_for(2'000'000, 48'000'000).frac == 0);
static_assert(dpll_ratio_for(2'000'000, 48'000'000).actual_hz == 48'000'000);
static_assert(dpll_ratio_for(2'000'000, 48'000'000).exact);
static_assert(dpll_ratio_for(2'000'000, 49'000'000).ldr == 23);
static_assert(dpll_ratio_for(2'000'000, 49'000'000).frac == 8);   // 24.5 x 2 MHz
static_assert(dpll_ratio_for(2'000'000, 49'000'000).exact);
// A rate off the sixteenth grid is reported as INEXACT with the nearest
// achievable value, never rounded silently.
static_assert(!dpll_ratio_for(2'000'000, 48'050'000).exact);
static_assert(dpll_ratio_for(2'000'000, 48'050'000).actual_hz == 48'000'000);
// 96 MHz from 32.768 kHz is 2929.6875 references: LDR + 1 = 2929 and
// eleven sixteenths, which is what the fractional part is FOR.
static_assert(dpll_ratio_for(32'768, 96'000'000).ldr == 2928);
static_assert(dpll_ratio_for(32'768, 96'000'000).frac == 11);
static_assert(dpll_ratio_for(32'768, 96'000'000).actual_hz == 96'000'000);

constexpr FdpllConfig dpll_48_from_xosc{
    .reference = DpllReference::xosc,
    .reference_hz = 24'000'000,
    .xosc_div = 5,   // 24 MHz / (2 x 6) = 2 MHz
    .ldr = 23,
};
static_assert(Fdpll::divided_reference_hz(dpll_48_from_xosc) == 2'000'000);
static_assert(Fdpll::dco_hz(dpll_48_from_xosc) == 48'000'000);
static_assert(Fdpll::output_hz(dpll_48_from_xosc) == 48'000'000);
static_assert(Fdpll::config_valid(dpll_48_from_xosc));

// The prescaler divides what the DCO made - it is not a way past the
// bottom of the DCO's own range.
constexpr FdpllConfig dpll_24_out{
    .reference = DpllReference::xosc,
    .reference_hz = 24'000'000,
    .xosc_div = 5,
    .ldr = 23,
    .prescaler = DpllPrescaler::div2,
};
static_assert(Fdpll::dco_hz(dpll_24_out) == 48'000'000);
static_assert(Fdpll::output_hz(dpll_24_out) == 24'000'000);
static_assert(Fdpll::config_valid(dpll_24_out));

// The refusals: a DCO under 48 MHz, a reference past 2 MHz, and the XOSC
// divider asked for on a reference that does not have one.
static_assert(!Fdpll::config_valid(FdpllConfig{.reference = DpllReference::gclk,
                                               .reference_hz = 2'000'000,
                                               .ldr = 11}));   // 24 MHz DCO
static_assert(!Fdpll::config_valid(FdpllConfig{.reference = DpllReference::gclk,
                                               .reference_hz = 4'000'000,
                                               .ldr = 11}));   // reference too fast
static_assert(!Fdpll::config_valid(FdpllConfig{.reference = DpllReference::gclk,
                                               .reference_hz = 2'000'000,
                                               .xosc_div = 1,
                                               .ldr = 23}));
static_assert(Fdpll::config_valid(FdpllConfig{.reference = DpllReference::xosc32k,
                                              .reference_hz = 32'768,
                                              .ldr = 1464}));   // 48.005 MHz

// LBYPASS defaults SET - erratum 1.25.1 - and clearing it is deliberate.
static_assert((Fdpll::ctrlb_word(FdpllConfig{}) & OSCCTRL_DPLLCTRLB_LBYPASS_Msk) != 0u);
static_assert((Fdpll::ctrlb_word(FdpllConfig{.lock_bypass = false}) &
               OSCCTRL_DPLLCTRLB_LBYPASS_Msk) == 0u);

void dpll_verbs() {
    static_assert(Fdpll::gclk_reference == OSCCTRL_GCLK_ID_FDPLL);
    static_assert(Fdpll::gclk_lock_timer == OSCCTRL_GCLK_ID_FDPLL32K);
    (void)Fdpll::reference_clock(3);
    (void)Fdpll::lock_timer_clock(4);
    (void)Fdpll::init(dpll_48_from_xosc, 1000);
    (void)Fdpll::init(FdpllConfig{.reference = DpllReference::gclk,
                                  .reference_hz = 2'000'000,
                                  .ldr = 23,
                                  .ldr_frac = 8,
                                  .prescaler = DpllPrescaler::div4,
                                  .filter = DpllFilter::high_damping,
                                  .lock_time = DpllLockTime::ms11,
                                  .wake_up_fast = true,
                                  .low_power = true,
                                  .lock_bypass = false},
                      1000);
    (void)Fdpll::enabled();
    (void)Fdpll::locked();
    (void)Fdpll::clock_ready();
    (void)Fdpll::wait_locked(1000);
    (void)Fdpll::timed_out();
    (void)Fdpll::ratio_updated();
    (void)Fdpll::sync_busy();
    (void)Fdpll::wait_sync();
    (void)Fdpll::set_ratio(47, 0, 1000);
    (void)Fdpll::prescaler(DpllPrescaler::div2);
    (void)Fdpll::prescaler();
    (void)Fdpll::stop();
}

void generator_verbs() {
    (void)Gclk<0>::configure({.source = GclkSource::osc48m});
    (void)Gclk<1>::configure({.source = GclkSource::osculp32k,
                              .div = 4,
                              .div_pow2 = true,
                              .improve_duty = true,
                              .output_enable = true,
                              .run_standby = true});
    static_assert(Gclk<8>::index == 8);
    static_assert(Gclk<1>::sync_mask == (GCLK_SYNCBUSY_GENCTRL0_Msk << 1));
    (void)Gclk<0>::enabled();
    (void)Gclk<0>::source();
    (void)Gclk<0>::sync_busy();
    (void)Gclk<2>::enable(false);
    (void)Gclk<2>::wait_sync();

    (void)GclkChannel::connect(0, 0);
    GclkChannel::disconnect(0);
    (void)GclkChannel::connected(0);
    (void)GclkChannel::generator(0);
    (void)GclkChannel::locked(0);
}

void main_clock_verbs() {
    (void)Mclk::cpu_div(1);
    (void)Mclk::cpu_div(3);      // refused: CPUDIV is one-hot
    (void)Mclk::cpu_div();
    Mclk::ahb(MCLK_AHBMASK_DSU_Msk, true);
    Mclk::apb_a(MCLK_APBAMASK_RTC_Msk, true);
    Mclk::apb_b(MCLK_APBBMASK_PORT_Msk, true);
    Mclk::apb_c(MCLK_APBCMASK_EVSYS_Msk, false);
}
