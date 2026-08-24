// TCD family smoke TU: every package must compile this (instantiation
// only). TCD0 exists on every DA/DB package; what changes is the route
// table (ALT1 needs PORTB, ALT3 needs PORTG) and the pin-level bonding
// inside an existing port (48-pin ALT1 and 28-pin ALT2 stop at WOB).
#include "avrdx/clock.hpp"
#include "avrdx/evsys.hpp"
#include "avrdx/tcd.hpp"

using namespace brio;

// The clock contract: TcdPwm is a ClockUser (a TCD clocked from CLK_PER
// changes frequency with the main prescaler), so a DynamicClock can list
// it and its init(clock) asserts clock_follows.
using Boot = Clock<ClockSource::internal, 24'000'000>;
using Dyn = DynamicClock<Boot, TcdPwm<TcdRoute::def>>;

void tcd_resource() {
    // The compile-time form, every knob of the chapter at once.
    (void)Tcd<0>::init<TcdConfig{
        .route = TcdRoute::def,
        .clock = TcdClock::oschf,
        .sync_prescaler = TcdSyncPrescaler::div2,
        .count_prescaler = TcdCountPrescaler::div4,
        .waveform = TcdWaveform::two_ramp,
        .compare_a_set = 100,
        .compare_a_clear = 400,
        .compare_b_set = 700,
        .compare_b_clear = 1199,
        .compare_override = true,
        .auto_update = true,
        .fifty_percent = false,
        .wo_c = TcdWaveformSelect::pwm_a,
        .wo_d = TcdWaveformSelect::pwm_b,
        .compare_a_value = 0x5,
        .compare_b_value = 0xA,
        .input_a = {.enable = true, .action = TcdEventAction::capture, .rising = true,
                    .config = TcdEventConfig::filter, .mode = TcdInputMode::exec_wait},
        .input_b = {.enable = true, .action = TcdEventAction::fault, .rising = false,
                    .config = TcdEventConfig::async, .mode = TcdInputMode::level_trig_freq},
        .enable_woa = true,
        .enable_wob = true,
        .fault_woa = true,
        .delay = TcdDelaySelect::input_blanking,
        .delay_trigger = TcdDelayTrigger::cmpaset,
        .delay_prescaler = TcdDelayPrescaler::div8,
        .delay_value = 40,
        .dither_select = TcdDitherSelect::dead_time_ab,
        .dither = 8,
        .debug_run = true,
        .fault_on_debug = true}>();

    // The run-time form, and the whole verb surface.
    (void)Tcd<0>::init({.route = TcdRoute::alt2, .waveform = TcdWaveform::dual_slope,
                        .compare_a_set = 500, .compare_b_set = 1000,
                        .compare_b_clear = 2047, .enable_woa = true, .enable_wob = true});
    (void)Tcd<0>::enable();
    (void)Tcd<0>::disable();
    (void)Tcd<0>::enabled();
    (void)Tcd<0>::enable_ready();
    (void)Tcd<0>::wait_enable_ready(16);
    (void)Tcd<0>::command_ready();
    (void)Tcd<0>::wait_command_ready(16);
    (void)Tcd<0>::sync();
    (void)Tcd<0>::sync_at_end();
    (void)Tcd<0>::restart();
    (void)Tcd<0>::software_capture_a();
    (void)Tcd<0>::software_capture_b();
    (void)Tcd<0>::disable_at_end();
    (void)Tcd<0>::compare_a_set(1);
    (void)Tcd<0>::compare_a_clear(2);
    (void)Tcd<0>::compare_b_set(3);
    (void)Tcd<0>::compare_b_clear(4);
    (void)Tcd<0>::compare_a_set();
    (void)Tcd<0>::compare_a_clear();
    (void)Tcd<0>::compare_b_set();
    (void)Tcd<0>::compare_b_clear();
    (void)Tcd<0>::dither(3);
    (void)Tcd<0>::dither();
    (void)Tcd<0>::capture_a();
    (void)Tcd<0>::capture_b();
    (void)Tcd<0>::waveform(TcdWaveform::four_ramp);
    (void)Tcd<0>::waveform();
    (void)Tcd<0>::clock(TcdClock::clkper, TcdSyncPrescaler::div8, TcdCountPrescaler::div32);
    (void)Tcd<0>::clock();
    (void)Tcd<0>::input_mode_a(TcdInputMode::freq);
    (void)Tcd<0>::input_mode_b(TcdInputMode::edge_trig);
    (void)Tcd<0>::event_input_a({.enable = true, .mode = TcdInputMode::none});
    (void)Tcd<0>::event_input_b({.enable = true, .mode = TcdInputMode::none});
    (void)Tcd<0>::output_control(true, true, true, TcdWaveformSelect::pwm_b,
                                 TcdWaveformSelect::pwm_a);
    (void)Tcd<0>::output_values(0x3, 0xC);
    Tcd<0>::fault_control(true, true, false, false, true, false, false, false);
    (void)Tcd<0>::fault_control();
    (void)Tcd<0>::delay(TcdDelaySelect::output_event, TcdDelayTrigger::cmpbclr,
                        TcdDelayPrescaler::div4, 200);
    (void)Tcd<0>::input_blanking_enabled();
    (void)Tcd<0>::output_event_enabled();
    (void)Tcd<0>::delay_cycles();
    (void)Tcd<0>::take_pwm_activity();
    Tcd<0>::clear_pwm_activity();
    (void)Tcd<0>::ovf_flag();
    (void)Tcd<0>::trig_a_flag();
    (void)Tcd<0>::trig_b_flag();
    Tcd<0>::clear_ovf();
    Tcd<0>::clear_trig_a();
    Tcd<0>::clear_trig_b();
    Tcd<0>::clear_flags();
    Tcd<0>::enable_ovf_interrupt(true);
    Tcd<0>::enable_trig_a_interrupt(true);
    Tcd<0>::enable_trig_b_interrupt(true);
    Tcd<0>::ovf();
    (void)Tcd<0>::take_triggers();
    Tcd<0>::input_a_on(EventChannel<2>{});
    Tcd<0>::input_b_on(EventChannel<3>{});
    Tcd<0>::input_a_off();
    Tcd<0>::input_b_off();
    (void)Tcd<0>::route();
    (void)Tcd<0>::claimed_outputs();
    (void)Tcd<0>::release();
}

void tcd_events() {
    EventChannel<4>::source(EvTcdCmpBClr{});
    EventChannel<5>::source(EvTcdCmpASet{});
    EventChannel<4>::source(EvTcdCmpBSet{});
    EventChannel<5>::source(EvTcdProgEv{});
    EvTcdInputA::listen(EventChannel<4>{});
    EvTcdInputB::listen(EventChannel<5>{});
    EvTcdInputA::unlisten();
    EvTcdInputB::unlisten();
}

// The routes this package offers, each with the outputs it bonds. WOC
// and WOD are asked for only where tcd_pin() says they exist, which is
// the pinless pattern: the route stays usable, the missing position is
// refused.
template <TcdRoute r>
void tcd_route() {
    if constexpr (tcd_route_exists(r)) {
        (void)Tcd<0>::init({.route = r,
                            .compare_b_clear = 999,
                            .enable_woa = true,
                            .enable_wob = true,
                            .enable_woc = tcd_pin(r, TcdOutput::woc).bonded,
                            .enable_wod = tcd_pin(r, TcdOutput::wod).bonded});
        (void)Tcd<0>::release();
    }
}

void tcd_routes() {
    tcd_route<TcdRoute::def>();
    tcd_route<TcdRoute::alt1>();
    tcd_route<TcdRoute::alt2>();
    tcd_route<TcdRoute::alt3>();
}

void tcd_task() {
    using Pwm = TcdPwm<TcdRoute::def>;
    (void)Pwm::init(Dyn{}, {.clock = TcdClock::clkper, .hz = 50'000, .dead_time_ticks = 8,
                            .woc = true, .wod = true, .auto_update = true});
    Pwm::rebase(12'000'000);
    (void)Pwm::duty(Pwm::max() / 4);
    (void)Pwm::duty();
    (void)Pwm::max();
    (void)Pwm::period_ticks();
    (void)Pwm::dead_ticks();
    (void)Pwm::counter_hz();
    (void)Pwm::cycle_hz();
    (void)Pwm::sync();
    (void)Pwm::sync_at_end();
    (void)Pwm::stop_at_end();
    (void)Pwm::stop();
    (void)Pwm::start();
    (void)Pwm::release();

    // A PLL-clocked pair: the PLL's only consumer on this silicon.
    (void)TcdPwm<TcdRoute::def>::init(Boot{},
        {.clock = TcdClock::pll, .source_hz = 48'000'000, .hz = 100'000,
         .dead_time_ticks = 12});
}

// The arithmetic is constexpr and the same on every package.
static_assert(tcd_counter_hz(24'000'000, TcdSyncPrescaler::div2, TcdCountPrescaler::div4) ==
              3'000'000);
static_assert(tcd_cycle_ticks(TcdWaveform::one_ramp, 1, 2, 3, 999) == 1000);
static_assert(tcd_cycle_ticks(TcdWaveform::two_ramp, 0, 99, 0, 199) == 300);
static_assert(tcd_cycle_ticks(TcdWaveform::four_ramp, 9, 19, 29, 39) == 100);
static_assert(tcd_cycle_ticks(TcdWaveform::dual_slope, 0, 0, 0, 500) == 1002);
static_assert(!tcd_input_mode_valid(TcdWaveform::one_ramp, TcdInputMode::exec_wait));
static_assert(tcd_input_mode_valid(TcdWaveform::two_ramp, TcdInputMode::exec_wait));
static_assert(!tcd_input_mode_valid(TcdWaveform::dual_slope, TcdInputMode::wait_sw));
static_assert(tcd_input_mode_valid(TcdWaveform::dual_slope, TcdInputMode::freq));
static_assert(tcd_dither_cycle_cost(TcdWaveform::one_ramp, TcdDitherSelect::on_time_b) == 1);
static_assert(tcd_dither_cycle_cost(TcdWaveform::one_ramp, TcdDitherSelect::dead_time_b) == 0);
static_assert(tcd_dither_cycle_cost(TcdWaveform::four_ramp, TcdDitherSelect::dead_time_ab) == 2);

// DEFAULT is PA4..PA7 on every package of both families.
static_assert(tcd_pin(TcdRoute::def, TcdOutput::woa).port == 'A' &&
              tcd_pin(TcdRoute::def, TcdOutput::woa).pin == 4 &&
              tcd_pin(TcdRoute::def, TcdOutput::wod).bonded);
