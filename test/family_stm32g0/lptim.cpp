// Family smoke TU: stm32g0/lptim.hpp instantiated for BOTH low-power
// timers on each of the three headers the desk's boards span.
// Instantiation only - no main(), no hardware.
//
// WHAT THIS FIXTURE IS REALLY FOR. The two instances share one
// LPTIM_TypeDef and the device header declares CFGR.ENC, ISR.UP/DOWN and
// CFGR2.IN2SEL once, for both - so nothing in it refuses a driver that
// writes a bit table 135 and figure 271's footnote say LPTIM2 does not
// have. Those facts live in stm32g0/device_tables.hpp as the MANUAL'S,
// and this file is where they are re-stated as static_asserts and where
// every verb is instantiated against them. The per-header differences it
// covers: the VECTOR (each LPTIM shares one with a basic timer on the
// G0B1 and G071 classes and has its own on the G031, where there are no
// basic timers), and the COMPARATORS the trigger and input multiplexers
// name - three on the G0B1, two on the G071, NONE on the G031, where
// every comparator row of tables 138..142 is therefore refused.

#include <stdint.h>

#include "stm32g0/lptim.hpp"
#include "stm32g0/platform_stm32.hpp"
#include "util/pwm_channel.hpp"

using namespace brio;

// ---- presence, as the three headers state it -------------------------------
static_assert(lptim_present(1) && lptim_present(2), "every STM32G0x1 has both");
static_assert(!lptim_present(0) && !lptim_present(3), "and no part has a third");

// ---- the geometry the MANUAL states and the header cannot ------------------
static_assert(lptim_has_encoder(1) && !lptim_has_encoder(2),
              "table 135: encoder mode is LPTIM1's alone");
static_assert(lptim_has_input2(1) && !lptim_has_input2(2),
              "figure 271's footnote: LPTIM2 has only input channel 1");
static_assert(lptim_exti_line(1) == 29 && lptim_exti_line(2) == 30,
              "table 65: both DIRECT lines");
static_assert(lptim_dmamux_trigger(1) == 20 && lptim_dmamux_trigger(2) == 21,
              "table 56's trigger inputs");
static_assert(lptim_clock_select_pos(1) != 0xFF && lptim_clock_select_pos(2) != 0xFF,
              "5.4.21 gives BOTH instances a kernel-clock multiplexer");
static_assert(lptim_bus_clock_mask(1) != 0 && lptim_reset_mask(1) != 0,
              "the RCC reset bit is not a convenience here - it is the only "
              "disable ES0548 2.8.1 allows");

// ---- the vector, which is the per-header difference this file exists for ---
#if defined(STM32G031xx)
static_assert(lptim_irq(1) == LPTIM1_IRQn && lptim_irq(2) == LPTIM2_IRQn,
              "the G031 class gives each LPTIM a line of its own");
#else
static_assert(lptim_irq(1) == TIM6_DAC_LPTIM1_IRQn,
              "LPTIM1 shares TIM6's and the DAC's vector");
static_assert(lptim_irq(2) == TIM7_LPTIM2_IRQn, "LPTIM2 shares TIM7's");
#endif

// ---- the arithmetic --------------------------------------------------------
static_assert(lptim_prescaler_divider(LptimPrescaler::div1) == 1);
static_assert(lptim_prescaler_divider(LptimPrescaler::div128) == 128);
static_assert(lptim_counter_hz(32768, LptimPrescaler::div32) == 1024);
static_assert(lptim_clock_runs_in_stop(LptimClock::lse));
static_assert(lptim_clock_runs_in_stop(LptimClock::lsi));
static_assert(!lptim_clock_runs_in_stop(LptimClock::pclk));
static_assert(!lptim_clock_runs_in_stop(LptimClock::hsi16));

// ---- the trigger multiplexer, both tables ----------------------------------
static_assert(lptim_trigger_code(LptimTrigger::tamp_trg3) == 5 &&
                  lptim_trigger_code(LptimTrigger::comp3_out) == 5,
              "row 5 is one TRIGSEL code and two different signals");
static_assert(lptim_trigger_valid(1, LptimTrigger::etr_pad));
static_assert(lptim_trigger_valid(1, LptimTrigger::rtc_alarm_a));
static_assert(lptim_trigger_valid(2, LptimTrigger::rtc_alarm_b));
static_assert(lptim_trigger_valid(1, LptimTrigger::tamp2));
static_assert(!lptim_trigger_valid(2, LptimTrigger::comp3_out),
              "TRIGSEL 5 is TAMP_TRG3 on LPTIM2, not COMP3");
static_assert(!lptim_trigger_valid(1, LptimTrigger::tamp_trg3),
              "...and COMP3 on LPTIM1, not TAMP_TRG3");
static_assert(lptim_trigger_valid(2, LptimTrigger::tamp_trg3));

#if defined(STM32G0B1xx)
static_assert(lptim_trigger_valid(1, LptimTrigger::comp3_out),
              "the third comparator is the G0B1/G0C1's alone");
#else
static_assert(!lptim_trigger_valid(1, LptimTrigger::comp3_out));
#endif
#if defined(STM32G031xx)
static_assert(!lptim_trigger_valid(1, LptimTrigger::comp1_out) &&
                  !lptim_trigger_valid(2, LptimTrigger::comp2_out),
              "the G031 class has no comparator at all, so those rows are dead");
static_assert(!lptim_input1_valid(1, LptimInput1::comp1_out));
#else
static_assert(lptim_trigger_valid(1, LptimTrigger::comp1_out) &&
                  lptim_trigger_valid(2, LptimTrigger::comp2_out));
static_assert(lptim_input1_valid(1, LptimInput1::comp1_out));
static_assert(lptim_input2_valid(1, LptimInput2::comp2_out));
static_assert(lptim_input1_valid(2, LptimInput1::comp2_out),
              "table 142 gives LPTIM2's IN1 mux a COMP2 row that LPTIM1's has not");
#endif
static_assert(!lptim_input1_valid(1, LptimInput1::comp2_out),
              "table 140 marks LPTIM1's mux2 Not connected");
static_assert(!lptim_input1_valid(1, LptimInput1::comp1_or_comp2));
static_assert(lptim_input2_valid(2, LptimInput2::pad),
              "the only legal IN2SEL on an instance with no second input is "
              "leaving the Reserved field alone");
static_assert(!lptim_input2_valid(2, LptimInput2::comp2_out));

// ---- the configuration checker, rule by rule -------------------------------
static_assert(lptim_config_valid(1, LptimConfig{}));
static_assert(lptim_config_valid(2, LptimConfig{}));
static_assert(!lptim_config_valid(3, LptimConfig{}));
static_assert(!lptim_config_valid(
                  1, LptimConfig{.clock = LptimClockSource::external_input1,
                                 .clock_polarity = LptimClockPolarity::both}),
              "26.4.12: with an external clock, one edge or the other, never both");
static_assert(lptim_config_valid(
                  1, LptimConfig{.count_external = true,
                                 .clock_polarity = LptimClockPolarity::both}),
              "...but with the internal clock SAMPLING the input, both edges are legal");
static_assert(!lptim_config_valid(1, LptimConfig{.count_external = true,
                                                 .prescaler = LptimPrescaler::div2}),
              "26.4.12: COUNTMODE = 1 wants PRESC = /1");
static_assert(!lptim_config_valid(2, LptimConfig{.encoder = true}),
              "table 135: LPTIM2 has no encoder mode");
static_assert(lptim_config_valid(1, LptimConfig{.encoder = true}));
static_assert(!lptim_config_valid(1, LptimConfig{.prescaler = LptimPrescaler::div4,
                                                 .encoder = true}),
              "26.4.15's Caution: encoder mode wants PRESC = /1");
static_assert(!lptim_config_valid(1, LptimConfig{.clock = LptimClockSource::external_input1,
                                                 .encoder = true}),
              "26.4.15's Caution: encoder mode wants the internal clock");
static_assert(!lptim_config_valid(
                  1, LptimConfig{.clock_polarity = static_cast<LptimClockPolarity>(3)}),
              "26.7.4: CKPOL 11 is not allowed");
static_assert(!lptim_config_valid(2, LptimConfig{.interrupts = LptimFlag::up}),
              "26.7.3: the UP/DOWN enables are Reserved without encoder mode");
static_assert(lptim_config_valid(1, LptimConfig{.interrupts = LptimFlag::down}));
static_assert(!lptim_config_valid(1, LptimConfig{.interrupts = 0x80u}),
              "no bit outside the seven implemented flags");
static_assert(!lptim_config_valid(1, LptimConfig{.trigger = LptimTrigger::tamp_trg3,
                                                 .trigger_edge = LptimTriggerEdge::rising}),
              "an instance-wrong trigger row, with the trigger actually enabled");
static_assert(lptim_config_valid(1, LptimConfig{.trigger = LptimTrigger::tamp_trg3}),
              "...and the same row ignored while TRIGEN is software");

// ---- every verb of the resource, on both instances -------------------------
template <uint8_t n>
void exercise_resource() {
    using L = Lptim<n>;
    static_assert(L::instance == n);
    static_assert(L::exti_line == lptim_exti_line(n));
    static_assert(L::dmamux_generator_input == lptim_dmamux_trigger(n));
    static_assert(L::has_encoder == lptim_has_encoder(n));
    static_assert(L::has_input2 == lptim_has_input2(n));
    (void)L::irq();
    (void)&L::regs();

    L::bus_clock(true);
    (void)L::bus_clock();
    L::kernel_clock(LptimClock::lse);
    (void)L::kernel_clock();
    (void)L::kernel_clock_running();
    L::reset();
    L::disable();
    L::init();

    (void)L::configure(LptimConfig{});
    (void)L::template configure<LptimConfig{.prescaler = LptimPrescaler::div16}>();
    (void)L::interrupts(LptimFlag::arrm, true);
    (void)L::interrupts();
    (void)L::config_valid(LptimConfig{});

    L::enable();
    (void)L::enabled();
    (void)L::start_continuous();
    (void)L::start_single();

    (void)L::set_cmp(100);
    (void)L::cmp();
    (void)L::set_arr(200);
    (void)L::arr();
    (void)L::cmp_ok();
    (void)L::arr_ok();
    (void)L::wait_cmp_ok(4);
    (void)L::wait_arr_ok(4);

    (void)L::count();
    (void)L::count_raw();
    (void)L::reset_count();
    (void)L::count_reset_pending();
    (void)L::reset_on_read(true);
    (void)L::reset_on_read();

    (void)L::status();
    (void)L::clear_flags(LptimFlag::cmpm);
    L::clear_flags_raw(LptimFlag::arrm);
    (void)L::isr();

    (void)L::wake_line(true);
    (void)L::wake_line();
    (void)L::pending_wake();
    L::debug_freeze(true);
    (void)L::debug_freeze();
    L::release();
}

// ---- the tasks -------------------------------------------------------------
template <uint8_t n>
void exercise_tasks() {
    using L = Lptim<n>;

    using Pwm = LptimPwm<L, 999>;
    static_assert(PwmChannel<Pwm>, "the fourth PwmChannel implementation");
    static_assert(Pwm::max == 999);
    (void)Pwm::setup(LptimPrescaler::div8, true);
    Pwm::duty(250);
    (void)Pwm::duty();

    using Tick = LptimPeriodicTick<L>;
    static_assert(Tick::flag == LptimFlag::arrm);
    (void)Tick::setup(LptimPrescaler::div32, 1023);
    (void)Tick::set_period(511);
    Tick::stop();

    using Counter = LptimCounter<L>;
    (void)Counter::setup(LptimPrescaler::div1);
    (void)Counter::count32();
    (void)Counter::count();
    (void)Counter::laps();
    (void)Counter::isr();

    using Pulses = LptimPulseCounter<L>;
    static_assert(Pulses::documented_lost_edges == 5, "26.4.12's own number");
    (void)Pulses::setup_external(LptimClockPolarity::falling);
    (void)Pulses::setup_sampled(LptimClockPolarity::both);
    (void)Pulses::count32();
    (void)Pulses::count();
    (void)Pulses::isr();

    using Timeout = LptimTimeout<L>;
    static_assert(Timeout::flag == LptimFlag::cmpm);
    (void)Timeout::setup(LptimTrigger::rtc_alarm_a, LptimTriggerEdge::rising, 1000);
    (void)Timeout::expired();
    (void)Timeout::isr();
    Timeout::stop();
}

// Encoder mode exists on LPTIM1 alone, so it is instantiated there alone
// (the negative test/family_stm32g0/neg/lptim_encoder_on_lptim2.cpp is
// the other half of that claim).
void exercise_encoder() {
    using Enc = LptimEncoder<Lptim<1>>;
    (void)Enc::setup(4095, LptimClockPolarity::both, LptimInput1::pad,
                     LptimInput2::pad, true);
    (void)Enc::position();
    (void)Enc::went_up();
    (void)Enc::went_down();
    (void)Enc::isr();
    Enc::stop();
}

// ---- the pads (DS13560 tables 15 and 17; nothing in the header checks them)
constexpr PinSel lptim1_out{'B', 0, PinFunction::af5};
constexpr PinSel lptim1_in1{'B', 5, PinFunction::af5};
constexpr PinSel lptim1_in2{'B', 7, PinFunction::af5};
constexpr PinSel lptim1_etr{'B', 6, PinFunction::af5};
constexpr PinSel lptim2_out{'A', 8, PinFunction::af5};
constexpr PinSel lptim2_in1{'B', 1, PinFunction::af5};

void exercise_pads() {
    LptimPad<lptim1_out>::claim(PinSpeed::medium);
    LptimPad<lptim1_in1>::claim_input(PinPull::up);
    LptimPad<lptim1_in2>::claim_input(PinPull::down);
    LptimPad<lptim1_etr>::claim_input();
    LptimPad<lptim2_out>::claim();
    LptimPad<lptim2_in1>::claim_input(PinPull::up);
    LptimPad<lptim1_out>::release();
    LptimPad<lptim2_out>::release();
}

void instantiate_everything() {
    exercise_resource<1>();
    exercise_resource<2>();
    exercise_tasks<1>();
    exercise_tasks<2>();
    exercise_encoder();
    exercise_pads();
}
