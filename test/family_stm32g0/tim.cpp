// Family smoke TU: stm32g0/tim.hpp instantiated for EVERY timer this
// device header declares, on every header the pack ships. Instantiation
// only - no main(), no hardware.
//
// WHAT THIS FIXTURE IS REALLY FOR. The device header carries ONE
// TIM_TypeDef for ten different timers, so nothing in it refuses a
// driver that writes a register an instance does not implement: the
// geometry lives in stm32g0/device_tables.hpp as the DOCUMENTS' facts
// (DS13560 table 7, RM0444 21.2 / 22.2 / 23.2 / 24.2 / 25.2), and this
// file is where those facts are re-stated as static_asserts and where
// every verb is instantiated against them. The per-header differences it
// covers: TIM4 exists on the G0Bx/G0C1 class alone, TIM6/TIM7 and TIM15
// are absent from the G03x/G04x, TIM2 - the one 32-bit counter - from
// every x0 value-line part, and the vector sharing follows what shares
// the line (TIM3_TIM4_IRQn where there is a TIM4; TIM16_FDCAN_IT0_IRQn
// where there is an FDCAN; TIM6_DAC_LPTIM1_IRQn where there is a DAC or
// an LPTIM, TIM6_IRQn where there is neither) - each keyed here on the
// header's own presence symbol, never on a device name.

#include <stdint.h>

#include "stm32g0/platform.hpp"
#include "stm32g0/tim.hpp"
#include "util/meter_sampler.hpp"
#include "util/pwm_channel.hpp"

using namespace brio;

// ---- presence, as the header states it, and the vector each presence names
static_assert(tim_present(1) && tim_present(3));
static_assert(tim_present(14) && tim_present(16) && tim_present(17));
static_assert(tim_irq(1) == TIM1_BRK_UP_TRG_COM_IRQn && tim_cc_irq(1) == TIM1_CC_IRQn);
static_assert(tim_irq(14) == TIM14_IRQn);
#if defined(TIM2_BASE)
static_assert(tim_present(2) && tim_irq(2) == TIM2_IRQn);
#else
static_assert(!tim_present(2), "no TIM2 on the x0 value line");
#endif
#if defined(TIM4_BASE)
static_assert(tim_present(4), "this part has TIM4");
static_assert(tim_irq(3) == TIM3_TIM4_IRQn && tim_irq(4) == TIM3_TIM4_IRQn,
              "and TIM3 shares its line with it");
#else
static_assert(!tim_present(4) && tim_irq(3) == TIM3_IRQn);
#endif
#if defined(TIM6_BASE)
static_assert(tim_present(6) && tim_present(7) && tim_present(15));
static_assert(tim_irq(15) == TIM15_IRQn);
#if defined(DAC1_BASE) || defined(LPTIM1_BASE)
static_assert(tim_irq(6) == TIM6_DAC_LPTIM1_IRQn && tim_irq(7) == TIM7_LPTIM2_IRQn,
              "the basic timers share their lines with the DAC and the LPTIMs");
#else
static_assert(tim_irq(6) == TIM6_IRQn && tim_irq(7) == TIM7_IRQn,
              "no DAC and no LPTIM to share with (the x0 value line)");
#endif
#else
static_assert(!tim_present(6) && !tim_present(7) && !tim_present(15),
              "neither the basic timers nor TIM15 on the G03x/G04x");
#endif
#if defined(FDCAN1_BASE)
static_assert(tim_irq(16) == TIM16_FDCAN_IT0_IRQn && tim_irq(17) == TIM17_FDCAN_IT1_IRQn);
#else
static_assert(tim_irq(16) == TIM16_IRQn && tim_irq(17) == TIM17_IRQn);
#endif

// ---- the geometry ----------------------------------------------------------
static_assert(tim_counter_bits(1) == 16 && tim_counter_bits(3) == 16);
static_assert(tim_max_period(3) == 0xFFFFUL);
static_assert(tim_channels(1) == 4 && tim_channels(3) == 4);
static_assert(tim_channels(14) == 1 && tim_channels(16) == 1 && tim_channels(17) == 1);
static_assert(tim_complementary_channels(1) == 3);
static_assert(tim_complementary_channels(16) == 1 && tim_complementary_channels(3) == 0);
static_assert(tim_has_break(1) && tim_has_break(16) && !tim_has_break(3) && !tim_has_break(14));
static_assert(tim_has_slave_mode(3) && !tim_has_slave_mode(14) && !tim_has_slave_mode(16));
static_assert(tim_has_center_aligned(3) && !tim_has_center_aligned(14));
static_assert(tim_has_external_trigger(3) && !tim_has_external_trigger(16));
static_assert(tim_has_split_vector(1) && !tim_has_split_vector(3));
#if defined(TIM2_BASE)
static_assert(tim_counter_bits(2) == 32, "TIM2 is the family's one 32-bit counter");
static_assert(tim_max_period(2) == 0xFFFFFFFFUL);
static_assert(tim_channels(2) == 4 && tim_complementary_channels(2) == 0);
static_assert(!tim_has_break(2) && tim_has_slave_mode(2) && tim_has_external_trigger(2));
static_assert(!tim_has_split_vector(2));
#endif

// The manual's interconnect tables, both directions.
static_assert(tim_internal_trigger(3, 1) == 2, "TIM3's ITR1 is TIM2 (table 123)");
static_assert(tim_trigger_index_for(3, 2) == 1);
static_assert(tim_trigger_index_for(16, 2) == 0xFF, "TIM16 has no slave controller");
#if defined(TIM2_BASE)
static_assert(tim_trigger_index_for(2, 1) == 0, "TIM2's ITR0 is TIM1");
static_assert(tim_internal_trigger_is_oc1(2, 3), "TIM2's ITR3 is TIM14's OC1, not a TRGO");
#endif

// The dead-time generator's four ranges (RM0444 21.4.18).
static_assert(tim_dead_time_ticks(0x00) == 0 && tim_dead_time_ticks(0x7F) == 127);
static_assert(tim_dead_time_ticks(0x80) == 128 && tim_dead_time_ticks(0xBF) == 254);
static_assert(tim_dead_time_ticks(0xC0) == 256 && tim_dead_time_ticks(0xDF) == 504);
static_assert(tim_dead_time_ticks(0xE0) == 512 && tim_dead_time_ticks(0xFF) == 1008);
static_assert(tim_dead_time_ticks(tim_dead_time_code(200)) >= 200,
              "a dead-time request always rounds UP");

// ---- every present instance, every verb ------------------------------------
template <uint8_t n>
void exercise_base() {
    using T = Tim<n>;
    T::init();
    (void)T::configure({.prescaler = 63, .period = 999});
    T::enable(true);
    (void)T::count();
    T::set_count(0);
    (void)T::count_update_flag();
    (void)T::prescaler();
    T::set_prescaler(1);
    (void)T::period();
    (void)T::set_period(100);
    (void)T::auto_reload_preload();
    (void)T::repetition();
    (void)T::set_repetition(1);
    T::update();
    (void)T::trigger_event();
    (void)T::break_event();
    (void)T::flags();
    (void)T::flag(T::update_flag);
    T::clear_flags(T::update_flag);
    T::interrupts(T::update_interrupt, true);
    (void)T::interrupts();
    (void)T::isr();
    (void)T::master(TimMasterMode::update);
    (void)T::master();
    (void)T::slave({.mode = TimSlaveMode::gated, .trigger = TimTrigger::itr0});
    (void)T::slave_mode();
    (void)T::slave_trigger();
    (void)T::break_dead_time({.dead_time = 32});
    (void)T::main_output(true);
    (void)T::main_output();
    (void)T::clock_ok();
    (void)T::irq();
    (void)T::cc_irq();
    (void)T::bus_clock();
    T::release();
}

template <uint8_t n>
void exercise_channels() {
    using T = Tim<n>;
    for (uint8_t ch = 0; ch < T::channels; ++ch) {
        (void)T::output_channel(ch, {.mode = TimOutputMode::pwm1, .compare = 10});
        (void)T::capture_channel(ch, {.select = TimChannelSelect::direct});
        (void)T::channel_enable(ch, true);
        (void)T::channel_enabled(ch);
        (void)T::complementary_enable(ch, false);
        (void)T::compare(ch);
        (void)T::set_compare(ch, 1);
        (void)T::capture_compare_event(ch);
        (void)T::input_select(ch, 1);
        (void)T::input_select(ch);
    }
    // The ETR half of SMCR and its source multiplexer: real on the four
    // timers with an external trigger, a false everywhere else.
    (void)T::external_trigger({.prescaler = 2, .filter = 3, .clock_mode2 = true});
    (void)T::external_trigger_select(4);
    (void)T::external_trigger_select();
}
static_assert(tim_etr_config_valid({.prescaler = 3, .filter = 15}));
static_assert(!tim_etr_config_valid({.prescaler = 4}));
static_assert(!tim_etr_config_valid({.filter = 16}));
static_assert(tim_has_external_trigger(1) && !tim_has_external_trigger(15) &&
              !tim_has_external_trigger(16));

// A channel index past the instance's count is refused at RUN time by the
// resource (the negatives cover the compile-time half, in the tasks).
static_assert(Tim<3>::channels == 4 && Tim<16>::channels == 1);

void family_stm32g0_tim() {
    exercise_base<1>();
    exercise_base<3>();
    exercise_channels<1>();
    exercise_channels<3>();
#if defined(TIM2_BASE)
    exercise_base<2>();
    exercise_channels<2>();
#endif
    exercise_base<14>();
    exercise_base<16>();
    exercise_base<17>();
    exercise_channels<14>();
    exercise_channels<16>();
    exercise_channels<17>();
#if defined(TIM4_BASE)
    exercise_base<4>();
    exercise_channels<4>();
#endif
#if defined(TIM6_BASE)
    exercise_base<6>();
    exercise_base<7>();
    exercise_base<15>();
    exercise_channels<15>();
#endif
}

// ---- the tasks, and the util contracts they must satisfy -------------------
using Led = TimPad<PinSel{'A', 5, PinFunction::af2}>;   // TIM2_CH1, DS13560 table 13

/// The 32-bit counter's tasks, where the part has one; the 16-bit twins
/// on TIM3 stand in for the value line's instantiation.
#if defined(TIM2_BASE)
using Pwm32 = TimPwm<Tim<2>, 0, 1000>;
using Tick2 = TimPeriodicTick<Tim<2>>;
#else
using Pwm32 = TimPwm<Tim<3>, 0, 1000>;
using Tick2 = TimPeriodicTick<Tim<3>>;
#endif
using Pwm16 = TimPwm<Tim<3>, 2, 255>;
using PwmAdv = TimPwm<Tim<1>, 3, 4095>;
using Pair1 = TimPairPwm<Tim<1>, 0, 999>;
using Pair16 = TimPairPwm<Tim<16>, 0, 99>;

static_assert(PwmChannel<Pwm32>, "a general-purpose timer's channel is a PwmChannel");
static_assert(PwmChannel<Pwm16>);
static_assert(PwmChannel<PwmAdv>);
static_assert(PwmChannel<Pair1>, "so is a complementary PAIR - one actuator, one duty");
static_assert(PwmChannel<Pair16>);
static_assert(Pwm32::max == 1000 && Pair1::max == 999);

using Period3 = TimPeriodMeter<Tim<3>>;
using Interval16 = TimIntervalMeter<Tim<16>, 0>;
using Interval1 = TimIntervalMeter<Tim<1>, 2>;
using Counter3 = TimEventCounter<Tim<3>>;
using Gate3 = TimGatedCounter<Tim<3>>;
using Pulse3 = TimOnePulse<Tim<3>, 1>;

// What a capture handler feeds: the interval meter's reading is what a
// MeterLatch stores, and the latch is what MeterSampler consumes. The
// contract validated here is util/meter_sampler.hpp's, unchanged.
using CaptureLatch = MeterLatch<uint32_t, Stm32g0Platform<>, 0>;
static_assert(MeterSource<CaptureLatch>);

void family_stm32g0_tim_tasks() {
    Led::claim();
    Led::claim_input(PinPull::up);
    Led::release();

    (void)Pwm32::setup(63);
    Pwm32::duty(250);
    (void)Pwm32::duty();
    (void)Pwm16::setup(0);
    (void)PwmAdv::setup(0);
    (void)Pair1::setup(0, tim_dead_time_code(64));
    (void)Pair1::dead_time_ticks();
    Pair1::duty(10);
    (void)Pair16::setup(0, 4);

    (void)Period3::setup(63, 3);
    (void)Period3::period_ticks();
    (void)Period3::width_ticks();

    (void)Interval16::setup(0, 0, TimCapturePolarity::rising);
    (void)Interval16::interval();
    Interval16::restart();
    (void)Interval1::setup(0);

    (void)Counter3::setup(TimTrigger::itr1);
    (void)Counter3::count();
    Counter3::restart();

    (void)Gate3::setup(TimTrigger::itr1);
    (void)Gate3::count();
    Gate3::restart();

    (void)Tick2::setup(63, 999);
    Tick2::stop();

    (void)Pulse3::setup(63, 100, 200);
    (void)Pulse3::arm(TimTrigger::itr1);
    Pulse3::fire();
    (void)Pulse3::busy();

    // The DMA burst engine (TIMx_DCR/TIMx_DMAR): every verb on a timer
    // that has it and on one that has not, so both branches compile on
    // every header of the family.
    static_assert(brio::Tim<3>::has_dma_burst);
    static_assert(!brio::Tim<14>::has_dma_burst);
    (void)brio::Tim<3>::dma_burst(brio::TimBurstBase::arr, 4);
    (void)brio::Tim<3>::dma_burst(brio::TimBurstBase::ccr1, 2);
    (void)brio::Tim<3>::burst_base();
    (void)brio::Tim<3>::burst_length();
    (void)brio::Tim<3>::dmar_address();
    brio::Tim<3>::dma_burst_off();
    (void)brio::Tim<14>::dma_burst(brio::TimBurstBase::arr, 4);
    (void)brio::Tim<14>::dmar_address();
}
