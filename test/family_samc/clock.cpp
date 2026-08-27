// CLKCTRL-equivalent family smoke TU: the OSCCTRL / GCLK / MCLK
// resources and the static Clock task. All three blocks are identical
// across the E/G/J variants, so what this TU really proves is that the
// task's compile-time arithmetic and every resource verb instantiate
// from the device header alone.
#include "samc/clock.hpp"

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
