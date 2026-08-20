// RTC/PIT family smoke TU: every package must compile this
// (instantiation only). The RTC is one instance with the same register
// set on every DA/DB package - no tiers to gate here; what the TU
// proves is that the whole option space instantiates, that the counter
// and the PIT stay independent, and that the Ticker still builds on
// top of the resources for every supported tick rate.
#include "avrdx/clock.hpp"
#include "avrdx/evsys.hpp"
#include "avrdx/rtc.hpp"
#include "avrdx/ticker.hpp"

using namespace brio;

// The four clock sources exist on every package (the device header's
// own RTC_CLKSEL_t is identical across DA and DB).
static_assert(rtc_source_hz(RtcSource::osc32k) == 32768u);
static_assert(rtc_source_hz(RtcSource::osc1k) == 1024u);
static_assert(rtc_source_hz(RtcSource::xosc32k) == 32768u);
static_assert(rtc_source_hz(RtcSource::extclk) == 0u);

// The prescaler codes ARE the divisors' exponents.
static_assert(rtc_prescaler_div(RtcPrescaler::div1) == 1);
static_assert(rtc_prescaler_div(RtcPrescaler::div32) == 32);
static_assert(rtc_prescaler_div(RtcPrescaler::div32768) == 32768);
static_assert(pit_cycles(PitPeriod::off) == 0);
static_assert(pit_cycles(PitPeriod::cyc4) == 4);
static_assert(pit_cycles(PitPeriod::cyc32) == 32);
static_assert(pit_cycles(PitPeriod::cyc32768) == 32768);

// The correction rule of 26.6, as the config and calibrate() enforce it.
static_assert(rtc_correction_valid(100, RtcPrescaler::div1));
static_assert(!rtc_correction_valid(-100, RtcPrescaler::div1));
static_assert(rtc_correction_valid(-100, RtcPrescaler::div2));
static_assert(!rtc_correction_valid(-128, RtcPrescaler::div32768));

void rtc_clock() {
    RtcClock::select(RtcSource::osc32k);
    RtcClock::select(RtcSource::osc1k);
    RtcClock::select(RtcSource::xosc32k);
    RtcClock::select(RtcSource::extclk);
    (void)RtcClock::selected();
    (void)RtcClock::hz();
    (void)RtcClock::preferred();
}

void rtc_counter() {
    Rtc::init<RtcConfig{.prescaler = RtcPrescaler::div1, .period = 32767, .compare = 1000}>();
    Rtc::init<RtcConfig{.prescaler = RtcPrescaler::div2, .period = 16383,
                        .correction_ppm = -100, .run_standby = true, .debug_run = true}>();
    (void)Rtc::init({.prescaler = RtcPrescaler::div32768, .period = 3, .compare = 1});
    Rtc::count(0);
    Rtc::period(1234);
    Rtc::compare(567);
    (void)Rtc::count();
    (void)Rtc::period();
    (void)Rtc::compare();
    Rtc::prescaler(RtcPrescaler::div1024);
    (void)Rtc::prescaler();
    (void)Rtc::tick_hz();
    (void)Rtc::calibrate(127);
    (void)Rtc::calibrate(0);
    (void)Rtc::calibration_ppm();
    (void)Rtc::correcting();
    Rtc::enable_ovf_interrupt(true);
    Rtc::enable_cmp_interrupt(true);
    (void)Rtc::take_flags();
    Rtc::clear_ovf();
    Rtc::clear_cmp();
    (void)Rtc::ovf_flag();
    (void)Rtc::cmp_flag();
    (void)Rtc::ctrla_busy();
    (void)Rtc::count_busy();
    (void)Rtc::period_busy();
    (void)Rtc::compare_busy();
    (void)Rtc::sync();
    Rtc::run_standby(true);
    Rtc::debug_run(false);
    Rtc::disable();
    Rtc::enable();
    (void)Rtc::enabled();
}

void rtc_pit() {
    Pit::init(PitPeriod::cyc32);
    Pit::init(PitPeriod::cyc32768, false);
    Pit::period(PitPeriod::cyc4);
    (void)Pit::period();
    Pit::enable_interrupt(true);
    (void)Pit::interrupt_enabled();
    (void)Pit::flag();
    Pit::clear_flag();
    (void)Pit::take_flag();
    (void)Pit::ctrl_busy();
    (void)Pit::tick_hz();
    Pit::debug_run(true);
    Pit::disable();
    Pit::enable();
    (void)Pit::enabled();
}

void rtc_events() {
    // The counter's two generators reach every channel; the PIT's
    // divided clocks split by channel parity (evsys.hpp).
    EventChannel<0>::source(Rtc::OvfEvent{});
    EventChannel<1>::source(Rtc::CmpEvent{});
    EventChannel<0>::source(EvPitDiv<8192>{});
    EventChannel<1>::source(EvPitDiv<64>{});
}

void rtc_ticker() {
    Ticker::init();
    Ticker::init(RtcSource::osc1k);
    Ticker::pause();
    Ticker::resume();
    (void)Ticker::ticks();
    (void)Ticker::millis();
    (void)Ticker::secs();
    TimeStamp ts;
    Ticker::now(ts);
    BasicTicker<16>::init();
    BasicTicker<512>::init(RtcSource::xosc32k);
}
