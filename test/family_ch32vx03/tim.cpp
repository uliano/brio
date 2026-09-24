// Timer family smoke TU: the advanced-control block and the three
// general-purpose ones, every verb of the resource, every task, and the
// arithmetic that decides a dead time, an internal trigger and a
// counter's clock.
//
// WHICH TIMERS A PART HAS is the datasheet's table 2-1, folded by
// tim_present() from device:: - one advanced and three general-purpose
// on every part of this series, plus the 32-bit TIM5 on the 128 KB one,
// which is tim5.cpp's subject because eight of the nine parts have not
// got it.
//
// NO PAD IS SPELLED AS A LITERAL. A remap column's pads are the
// chapter's, the same on every part; WHICH of them a package brings out
// is the part's, so the pad-facing half is instantiated on the first
// channel pad each package actually bonds (the smallest ones bond
// neither PA8 nor PA9, so TIM1's first reachable channel there is not
// channel 1).
#include "ch32vx03/platform.hpp"
#include "ch32vx03/tim.hpp"
#include "util/meter_sampler.hpp"

using namespace brio;

// ---- what a timer IS -------------------------------------------------------
static_assert(tim_present(1) && tim_present(2) && tim_present(3) && tim_present(4));
static_assert(tim_present(5) == device::has_tim5);
static_assert(!tim_present(0) && !tim_present(6) && !tim_present(7) && !tim_present(8));

static_assert(tim_channels(1) == 4 && tim_channels(4) == 4 && tim_channels(6) == 0);
static_assert(tim_complementary_channels(1) == 3 && tim_complementary_channels(2) == 0);
static_assert(tim_has_break(1) && !tim_has_break(3));
static_assert(tim_has_repetition(1) && !tim_has_repetition(2));
static_assert(tim_counter_bits(1) == 16 && tim_max_period(2) == 0xFFFFu);
static_assert(tim_has_slave_mode(2) && tim_has_encoder(4) && tim_has_master_mode(3));
static_assert(tim_has_external_trigger(1) && tim_has_ti1_xor(2) && tim_has_dma(4));
static_assert(tim_bus(1) == Bus::pb2 && tim_bus(2) == Bus::pb1);
static_assert(tim_gate(1) == rcc_pb2_tim1 && tim_gate(4) == rcc_pb1_tim4);

// ---- the internal trigger table (14-2, 15-2) -------------------------------
// TIM1's ITR0 and TIM3's ITR2 are TIM5's, TIM2's ITR1 and TIM4's ITR3
// are TIM8's: the first pair exists on the 128 KB part alone and the
// second nowhere in this series, so the fold answers zero for them.
static_assert(tim_internal_trigger(1, 1) == 2 && tim_internal_trigger(1, 2) == 3);
static_assert(tim_internal_trigger(1, 0) == (device::has_tim5 ? 5 : 0));
static_assert(tim_internal_trigger(2, 0) == 1 && tim_internal_trigger(2, 1) == 0);
static_assert(tim_internal_trigger(3, 2) == (device::has_tim5 ? 5 : 0));
static_assert(tim_internal_trigger(4, 3) == 0);
static_assert(tim_internal_trigger(2, 4) == 0);
static_assert(tim_trigger_index_for(3, 1) == 0 && tim_trigger_index_for(3, 2) == 1);
static_assert(tim_trigger_index_for(2, 4) == 3);
static_assert(tim_trigger_index_for(1, 1) == 0xFF);

// ---- the dead-time generator (14.4.18) -------------------------------------
static_assert(tim_dead_time_ticks(0) == 0 && tim_dead_time_ticks(127) == 127);
static_assert(tim_dead_time_ticks(0x80) == 128 && tim_dead_time_ticks(0xBF) == 254);
static_assert(tim_dead_time_ticks(0xC0) == 256 && tim_dead_time_ticks(0xDF) == 504);
static_assert(tim_dead_time_ticks(0xE0) == 512 && tim_dead_time_ticks(0xFF) == 1008);
static_assert(tim_dead_time_code(0) == 0 && tim_dead_time_code(127) == 127);
// Never early: a request the ladder cannot hit exactly is rounded UP.
// Above 127 ticks the steps are 2, then 8, then 16 - so 200 is exact,
// 201 costs a tick and 500 costs four.
static_assert(tim_dead_time_ticks(tim_dead_time_code(200)) == 200);
static_assert(tim_dead_time_ticks(tim_dead_time_code(201)) == 202);
static_assert(tim_dead_time_ticks(tim_dead_time_code(500)) == 504);
static_assert(tim_dead_time_code(2000) == 0xFF);

// ---- the counter's clock ---------------------------------------------------
// A timer counts its bus clock undivided, or twice it where the bus is
// divided (RM 3.3.1) - and PB1 is the only divided bus this stratum
// makes, at exactly two. So TIMxCLK IS HCLK on both buses, at the
// ceiling and below it, which is what these two rates say.
using Ceiling = Clock<ClockSource::pll, 144'000'000>;
using Half = Clock<ClockSource::pll, 72'000'000>;
static_assert(Tim<1>::clock_hz(Ceiling{}) == Ceiling::timclk2_hz);
static_assert(Tim<2>::clock_hz(Ceiling{}) == Ceiling::timclk1_hz);
static_assert(Ceiling::pclk1_hz == 72'000'000u);
static_assert(Tim<2>::clock_hz(Ceiling{}) == 144'000'000u);
static_assert(Tim<2>::clock_hz(Half{}) == 72'000'000u);
static_assert(Tim<1>::clock_hz(Half{}) == 72'000'000u);

// ---- the vectors -----------------------------------------------------------
static_assert(Tim<1>::irq() == Irq::tim1_up && Tim<1>::cc_irq() == Irq::tim1_cc);
static_assert(Tim<1>::break_irq() == Irq::tim1_brk);
static_assert(Tim<1>::trigger_irq() == Irq::tim1_trg_com);
static_assert(Tim<1>::has_split_vectors);
static_assert(Tim<2>::irq() == Irq::tim2 && Tim<2>::cc_irq() == Irq::tim2);
static_assert(Tim<3>::irq() == Irq::tim3 && Tim<4>::irq() == Irq::tim4);
static_assert(!Tim<3>::has_split_vectors);
static_assert(Tim<1>::vector_flags(Irq::tim1_up) == tim_uif);
static_assert(Tim<1>::vector_flags(Irq::tim1_brk) == tim_bif);
static_assert(Tim<1>::vector_flags(Irq::tim2) == 0u);
static_assert(Tim<3>::vector_flags(Irq::tim3) == Tim<3>::interrupt_flags);
static_assert(Tim<3>::vector_flags(Irq::tim1_up) == 0u);
// The general-purpose timers have no break and no commutation, so
// neither flag is one their vector answers for.
static_assert((Tim<3>::interrupt_flags & (tim_bif | tim_comif)) == 0u);
static_assert((Tim<1>::interrupt_flags & (tim_bif | tim_comif)) == (tim_bif | tim_comif));
static_assert((Tim<1>::all_flags & tim_cc1of) != 0u);
static_assert((Tim<1>::interrupt_flags & tim_cc1of) == 0u);

// ---- the flag and enable bits share their positions -------------------------
static_assert(Tim<1>::compare_flag(0) == tim_cc1if && Tim<1>::compare_flag(3) == (tim_cc1if << 3));
static_assert(Tim<1>::compare_interrupt(2) == (tim_cc1ie << 2));
static_assert(Tim<1>::overcapture_flag(1) == (tim_cc1of << 1));
static_assert(Tim<1>::compare_dma(0) == tim_cc1de);

// ---- the pads --------------------------------------------------------------
// Column 0 of TIM3 is PA6/PA7/PB0/PB1, TIM2's is PA0..PA3 and TIM1's
// carries the three complementary outputs on PB13..PB15.
static_assert(tim_channel_pad(3, 0, 0) == Pad{'A', 6});
static_assert(tim_channel_pad(2, 0, 1) == Pad{'A', 1});
static_assert(tim_channel_pad(1, 0, 0) == Pad{'A', 8});
static_assert(tim_complementary_pad(0, 0) == Pad{'B', 13});
static_assert(tim_complementary_pad(0, 3) == Pad{});
static_assert(tim_etr_pad(3, 0) == Pad{'D', 2});
static_assert(tim_break_pad(0) == Pad{'B', 12});
static_assert(tim_channel_pad(2, 9, 0) == tim_channel_pad(2, 0, 0));   // a code past the table
static_assert(tim_channel_pad(6, 0, 0) == Pad{});

/// The first channel pad of `n`'s column `code` that THIS package
/// brings out - the smallest ones bond neither PA8 nor PA9, so the
/// advanced timer's first reachable channel there is not channel 1.
constexpr Pad first_bonded_channel(uint8_t n, uint8_t code) {
    for (uint8_t ch = 0; ch < 4u; ++ch) {
        const Pad p = tim_channel_pad(n, code, ch);
        if (pad_bonded(p)) {
            return p;
        }
    }
    return Pad{'A', 0};
}

using Tim1Pad = TimPad<first_bonded_channel(1, 0)>;
using Tim3Pad = TimPad<first_bonded_channel(3, 0)>;
static_assert(pad_bonded(Tim1Pad::selection) && pad_bonded(Tim3Pad::selection));

// ---- the tasks -------------------------------------------------------------
using Wave = TimPwm<Tim<3>, 0, 1000>;
using Pair = TimPairPwm<Tim<1>, 0, 2000>;
using Period = TimPeriodMeter<Tim<2>>;
using Interval = TimIntervalMeter<Tim<4>, 1>;
using Events = TimEventCounter<Tim<3>>;
using Gate = TimGatedCounter<Tim<4>>;
using Tick = TimPeriodicTick<Tim<2>>;
using Pulse = TimOnePulse<Tim<3>, 1>;
using Knob = TimEncoder<Tim<4>>;

static_assert(Wave::max == 1000 && Pair::max == 2000);
static_assert(Period::period_flag == tim_cc1if && Period::width_flag == (tim_cc1if << 1));
static_assert(Interval::capture_flag == (tim_cc1if << 1));
static_assert(Tick::flag == tim_uif);

void exercise_resource() {
    using T = Tim<3>;
    T::init();
    T::bus_clock(true);
    (void)T::bus_clock();
    T::reset();
    (void)T::remap(2);
    (void)T::remap();
    (void)T::clock_hz_now(48'000'000);

    (void)T::configure({.prescaler = 71,
                        .period = 999,
                        .direction = TimDirection::down,
                        .alignment = TimAlignment::center_both,
                        .clock_division = TimClockDivision::div4,
                        .auto_reload_preload = true,
                        .one_pulse = false,
                        .update_disable = false,
                        .update_on_overflow_only = true});
    T::enable(true);
    (void)T::enabled();
    (void)T::count();
    T::set_count(4);
    (void)T::direction();
    (void)T::prescaler();
    T::set_prescaler(7);
    (void)T::period();
    (void)T::set_period(100);
    (void)T::auto_reload_preload();
    (void)T::repetition();
    (void)T::set_repetition(2);

    T::update();
    (void)T::capture_compare_event(1);
    (void)T::trigger_event();
    (void)T::commutation_event();
    (void)T::break_event();

    (void)T::flags();
    (void)T::flag(T::update_flag);
    T::clear_flags(T::all_flags);
    T::interrupts(T::update_interrupt | T::compare_interrupt(0), true);
    (void)T::interrupts();
    (void)T::isr();
    (void)T::isr(T::vector_flags(Irq::tim3));

    (void)T::compare(0);
    (void)T::set_compare(0, 50);
    (void)T::output_channel(0, {.mode = TimOutputMode::pwm2,
                                .compare = 25,
                                .preload = false,
                                .fast = true,
                                .clear_on_etrf = true,
                                .active_low = true});
    (void)T::capture_channel(1, {.select = TimChannelSelect::indirect,
                                 .polarity = TimCapturePolarity::falling,
                                 .prescaler = TimCapturePrescaler::every4,
                                 .filter = 3});
    (void)T::output_mode(0, TimOutputMode::force_active);
    (void)T::output_mode(0);
    (void)T::channel_enable(0, false);
    (void)T::channel_enabled(0);
    (void)T::complementary_enable(0, true);
    (void)T::complementary_enabled(0);

    (void)T::slave({.mode = TimSlaveMode::external_clock1,
                    .trigger = TimTrigger::itr0,
                    .master_slave = true});
    (void)T::slave_mode();
    (void)T::slave_trigger();
    (void)T::master(TimMasterMode::update);
    (void)T::master();
    (void)T::ti1_xor(true);
    (void)T::ti1_xor();
    (void)T::preload_channels(true, true);
    (void)T::compare_dma_on_update(true);
    (void)T::break_dead_time({.dead_time = 32, .break_enable = true});
    (void)T::main_output(true);
    (void)T::main_output();
    (void)T::dead_time_ticks();
    (void)T::external_trigger({.inverted = true, .prescaler = 2, .filter = 5, .clock_mode2 = true});
    (void)T::external_clock_mode2();
    (void)T::dma_burst(TimBurstBase::ch1cvr, 3);
    (void)T::burst_base();
    (void)T::burst_length();
    T::dma_burst_off();
    (void)T::dmaadr_address();
    (void)T::chcvr_address(2);
    (void)T::cnt_address();
    (void)T::channel_pad(0, 0);
    (void)T::complementary_pad(0, 0);
    (void)T::etr_pad(0);
    (void)T::break_pad(0);
    T::release();
}

void exercise_advanced() {
    using T = Tim<1>;
    T::init();
    (void)T::configure({.prescaler = 0, .period = 1000, .repetition = 3});
    (void)T::set_repetition(1);
    (void)T::output_channel(0, {.mode = TimOutputMode::pwm1,
                                .compare = 500,
                                .complementary_enable = true,
                                .idle_high = true,
                                .complementary_idle_high = true});
    (void)T::break_dead_time({.dead_time = tim_dead_time_code(40),
                              .main_output_enable = true,
                              .automatic_output_enable = true,
                              .break_enable = true,
                              .break_active_high = true,
                              .off_state_run = true,
                              .off_state_idle = true,
                              .lock = 0});
    (void)T::isr(T::vector_flags(Irq::tim1_cc));
    T::release();
}

void exercise_tasks() {
    (void)Wave::setup(71);
    Wave::duty(250);
    (void)Wave::duty();

    (void)Pair::setup(0, tim_dead_time_code(20), TimClockDivision::div2);
    Pair::duty(1000);
    (void)Pair::dead_time_ticks();

    (void)Period::setup(71, 2, false);
    (void)Period::period_ticks();
    (void)Period::width_ticks();

    (void)Interval::setup(71, 1, TimCapturePolarity::falling, TimCapturePrescaler::every2);
    (void)Interval::interval();
    Interval::restart();

    (void)Events::setup(TimTrigger::itr1, 0xFFFF);
    (void)Events::count();
    Events::restart();

    (void)Gate::setup(TimTrigger::itr0, 0);
    (void)Gate::count();
    Gate::restart();

    (void)Tick::setup(71, 999, true);
    Tick::stop();

    (void)Pulse::setup(71, 100, 400);
    (void)Pulse::arm(TimTrigger::itr1);
    Pulse::fire();
    (void)Pulse::busy();

    (void)Knob::setup({.mode = TimSlaveMode::encoder3, .filter = 2, .invert_a = true}, 4000);
    (void)Knob::count();
    Knob::set_count(0);
    (void)Knob::reversing();

    Tim1Pad::claim();
    Tim1Pad::claim_input(PinPull::up);
    (void)Tim1Pad::read();
    Tim1Pad::release();
    Tim3Pad::claim(PinDrive::open_drain, PinSpeed::medium);
    Tim3Pad::release();
}

// ---- the meters as util/meter_sampler.hpp sees them -------------------------
// THE MeterSource IS THE LATCH, not the task: the two meter tasks are
// what a capture body READS to fill one (design/meters.md), which is
// what the bodies below do.
using Latch = MeterLatch<uint32_t, Ch32vx03Platform<>, 0>;
using WidthLatch = MeterLatch<uint32_t, Ch32vx03Platform<>, 1>;
static_assert(MeterSource<Latch> && MeterSource<WidthLatch>);

extern "C" BRIO_CH32_INTERRUPT void tim2_handler() {
    const uint16_t hit = Tim<2>::isr();
    if ((hit & Period::period_flag) != 0u) {
        Latch::store(Period::period_ticks());
    }
    if ((hit & Period::width_flag) != 0u) {
        WidthLatch::store(Period::width_ticks());
    }
}

extern "C" BRIO_CH32_INTERRUPT void tim4_handler() {
    if ((Tim<4>::isr() & Interval::capture_flag) != 0u) {
        if (const std::optional<uint32_t> d = Interval::interval()) {
            Latch::store(*d);
        }
    }
}

/// The vector bodies an application binds, in both their shapes: one
/// line answering for everything, and the advanced timer's four.
extern "C" BRIO_CH32_INTERRUPT void tim3_handler() { (void)Tim<3>::isr(); }
extern "C" BRIO_CH32_INTERRUPT void tim1_cc_handler() {
    (void)Tim<1>::isr(Tim<1>::vector_flags(Irq::tim1_cc));
}
