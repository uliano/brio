// Family smoke TU for samc/tc.hpp: every verb must COMPILE on the E, G
// and J 18A headers (tools/check_samc.sh sweeps all three).
//
// The TC block is identical across this family - five instances, two
// channels each, the same registers. What VARIES is which pads carry a
// waveform output (26 on the J, 18 on the G, 8 on the E), and that is
// what this fixture asserts per variant, out of the device header's own
// PIN_P<pad>E_TC<n>_WO<k> symbols. The two other geometry facts pinned
// here - the SHARED generic clock channels and which instances can be
// PAIRED into a 32-bit counter - are the header's too, and they are the
// ones a reader would otherwise have to take on trust.

#include <stdint.h>

#include "samc/clock.hpp"
#include "samc/pin.hpp"
#include "samc/tc.hpp"
#include "util/pwm_channel.hpp"

using namespace brio;

static_assert(tc_count() == 5, "this family implements TC0..TC4");
static_assert(Tc<0>::cc_count == 2);

// 35.5.3's warning made checkable: TC0/TC1 share generic clock channel
// 30, TC2/TC3 share 31, TC4 has 32 to itself - so a pair that shares one
// "cannot be set to different clock frequencies".
static_assert(Tc<0>::gclk_id == TC0_GCLK_ID && Tc<1>::gclk_id == TC0_GCLK_ID);
static_assert(Tc<2>::gclk_id == TC2_GCLK_ID && Tc<3>::gclk_id == TC2_GCLK_ID);
static_assert(Tc<4>::gclk_id != Tc<0>::gclk_id);
static_assert(Tc<0>::gclk_id != Tc<2>::gclk_id);

// 35.6.2.4: TC0 pairs with TC1 and TC2 with TC3; TC4 cannot pair at all.
// TCn_MASTER_SLAVE_MODE says so and this driver reads it rather than
// restating the sentence.
static_assert(tc_can_pair(0) && tc_can_pair(2));
static_assert(!tc_can_pair(1) && !tc_can_pair(3) && !tc_can_pair(4));
static_assert(tc_pair_role(1) == 2 && tc_pair_role(4) == 0);
static_assert(Tc<0>::pair_index == 1 && Tc<2>::pair_index == 3);

// The EVSYS vocabulary the TC publishes (evsys.hpp owns only the fabric).
static_assert(Tc<0>::overflow_generator == 0x34);
static_assert(Tc<0>::match_generator(0) == 0x35 && Tc<0>::match_generator(1) == 0x36);
static_assert(Tc<4>::overflow_generator == 0x40);
static_assert(Tc<0>::event_user == 23 && Tc<4>::event_user == 27);

// And the DMAC trigger ids, from the header's own constants.
static_assert(Tc<0>::dma_trigger_overflow == TC0_DMAC_ID_OVF);
static_assert(Tc<0>::dma_trigger_match(1) == TC0_DMAC_ID_MC1);

static_assert(Tc<0>::irq() == TC0_IRQn && Tc<4>::irq() == TC4_IRQn);

// ---- the refusals ----------------------------------------------------------

static_assert(tc_config_valid(0, TcConfig{.mode = TcMode::count32}));
static_assert(!tc_config_valid(1, TcConfig{.mode = TcMode::count32}),
              "35.6.2.4: only an even, pairable instance can be a 32-bit master");
static_assert(!tc_config_valid(4, TcConfig{.mode = TcMode::count32}));
// COPEN without CAPTEN captures nothing.
static_assert(!tc_config_valid(0, TcConfig{.capture_on_pin = 0x1}));
static_assert(tc_config_valid(
    0, TcConfig{.capture_enable = 0x1, .capture_on_pin = 0x1}));
// A channel bit past the two this family implements.
static_assert(!tc_config_valid(0, TcConfig{.capture_enable = 0x4}));

// The two rules that live BETWEEN the structs.
static_assert(!tc_event_config_valid(
                  TcConfig{.waveform = TcWaveform::normal_pwm},
                  TcEventConfig{.action = TcEventAction::count}),
              "35.6.2.5.3: counting events does not support PWM generation");
static_assert(tc_event_config_valid(
    TcConfig{.waveform = TcWaveform::normal_frequency},
    TcEventConfig{.action = TcEventAction::count}));
static_assert(!tc_event_config_valid(TcConfig{},
                                     TcEventConfig{.action = TcEventAction::stamp}),
              "a capture action with no capture channel captures nothing");
static_assert(!tc_event_config_valid(
                  TcConfig{.capture_enable = 0x1},
                  TcEventConfig{.action = TcEventAction::stamp}),
              "a capture channel needs a source: its pad or the event input");
static_assert(tc_event_config_valid(
    TcConfig{.capture_enable = 0x1},
    TcEventConfig{.action = TcEventAction::stamp, .input_enable = true}));
static_assert(tc_event_config_valid(
    TcConfig{.capture_enable = 0x1, .capture_on_pin = 0x1},
    TcEventConfig{.action = TcEventAction::stamp}));

static_assert(tc_prescaler_divisor(TcPrescaler::div1024) == 1024);
static_assert(tc_action_is_capture(TcEventAction::pulse_width));
static_assert(!tc_action_is_capture(TcEventAction::start));

// ---- the pad table: irregular, per-package, and the header's ---------------

// Bonded on every variant of the family.
static_assert(tc_wo_exists<'A', 22> && TcWo<Pin<'A', 22>>::timer == 0);
static_assert(TcWo<Pin<'A', 22>>::channel == 0);
static_assert(TcWo<Pin<'A', 25>>::timer == 1 && TcWo<Pin<'A', 25>>::channel == 1);
// A pad with no waveform output anywhere.
static_assert(!tc_wo_exists<'A', 16>);

#if defined(__SAMC21E18A__)
// The E bonds only eight WO pads: TC4 on PA14/PA15/PA18/PA19, TC0 on
// PA22/PA23 and TC1 on PA24/PA25. No TC2 or TC3 output at all.
static_assert(!tc_wo_exists<'B', 23> && !tc_wo_exists<'B', 2>);
static_assert(!tc_wo_exists<'A', 20>);
#elif defined(__SAMC21G18A__)
static_assert(tc_wo_exists<'B', 23> && TcWo<Pin<'B', 23>>::timer == 3);
static_assert(tc_wo_exists<'B', 2> && TcWo<Pin<'B', 2>>::timer == 2);
static_assert(!tc_wo_exists<'B', 16>);
#else
// The bench board's LED is PB23 = TC3/WO1, which is how this stratum
// gets a PWM output with nothing to wire.
static_assert(tc_wo_exists<'B', 23> && TcWo<Pin<'B', 23>>::timer == 3 &&
              TcWo<Pin<'B', 23>>::channel == 1);
static_assert(tc_wo_exists<'B', 16> && TcWo<Pin<'B', 16>>::timer == 2);
// One instance, several pads: TC0/WO0 is on PA22, PB08 AND PB12.
static_assert(TcWo<Pin<'B', 8>>::timer == 0 && TcWo<Pin<'B', 8>>::channel == 0);
static_assert(TcWo<Pin<'B', 12>>::timer == 0 && TcWo<Pin<'B', 12>>::channel == 0);
#endif

// ---- the tasks satisfy the util contracts ----------------------------------

static_assert(PwmChannel<TcPwm<Tc<0>, 0>>);
static_assert(PwmChannel<TcPwm8<Tc<0>, 1, 199>>);
static_assert(TcPwm<Tc<0>, 0>::max == 0xFFFF);
static_assert(TcPwm8<Tc<0>, 1, 199>::max == 199);

void resource_verbs() {
    using T = Tc<0>;
    constexpr TcConfig cfg{.mode = TcMode::count16,
                           .prescaler = TcPrescaler::div64,
                           .waveform = TcWaveform::normal_pwm};

    (void)T::init(0);
    T::bus_clock(true);
    T::pair_bus_clock(true);
    (void)T::clock(0);
    (void)T::reset();
    (void)T::configure(cfg);
    (void)T::event_config(cfg, TcEventConfig{.action = TcEventAction::start,
                                             .input_enable = true,
                                             .overflow_out = true,
                                             .match_out = 0x1});
    (void)T::ctrla();
    (void)T::evctrl();
    (void)T::mode();
    (void)T::enable(true);
    (void)T::enabled();

    (void)T::command(TcCommand::update);
    (void)T::retrigger();
    (void)T::stop();
    (void)T::count_down(true);
    (void)T::counting_down();

    (void)T::read_sync();
    (void)T::count8();
    (void)T::count16();
    (void)T::count32();
    (void)T::count8_raw();
    (void)T::count16_raw();
    (void)T::count32_raw();
    (void)T::set_count8(1);
    (void)T::set_count16(1);
    (void)T::set_count32(1);

    (void)T::period8();
    (void)T::set_period8(199);
    (void)T::set_period_buffer8(100);

    (void)T::cc8(0);
    (void)T::cc16(0);
    (void)T::cc32(0);
    (void)T::set_cc8(0, 1);
    (void)T::set_cc16(0, 1);
    (void)T::set_cc32(0, 1);
    (void)T::set_cc_buffer8(0, 1);
    (void)T::set_cc_buffer16(0, 1);

    (void)T::status();
    (void)T::stopped();
    (void)T::is_client();
    (void)T::period_buffer_valid();
    (void)T::cc_buffer_valid(0);
    (void)T::clear_buffer_valid(TC_STATUS_CCBUFV0_Msk);

    (void)T::flags();
    T::clear_flags(T::overflow_flag | T::match_flag(0) | T::error_flag);
    T::arm(T::overflow_flag);
    T::disarm(T::overflow_flag);
    (void)T::armed();
    (void)T::isr();
    T::debug_run(true);
    (void)T::irq();
    T::release();
}

void task_verbs() {
    using Wo = TcWo<Pin<'A', 22>>;
    Wo::claim();
    Wo::release();

    (void)TcPwm<Tc<0>, 0>::setup(TcPrescaler::div8);
    TcPwm<Tc<0>, 0>::duty(0x8000);
    (void)TcPwm8<Tc<0>, 1, 199>::setup();
    TcPwm8<Tc<0>, 1, 199>::duty(100);

    (void)TcPeriodMeter<Tc<2>>::setup(TcPrescaler::div64, true);
    (void)TcPeriodMeter<Tc<2>>::period_ticks();
    (void)TcPeriodMeter<Tc<2>>::width_ticks();
    (void)TcPulseWidthMeter<Tc<3>>::setup();
    (void)TcPulseWidthMeter<Tc<3>>::width_ticks();
}
