// mcu: ch32v203rb
// The 32-BIT TIMER, which one part of the series has: chapter 15's own
// opening note gives TIM5 thirty-two bits on the CH32V20x_D8 device
// class, and the datasheet's table 2-1 gives the block itself to the
// 128 KB part alone. So this TU is the one place Tim<5> is
// instantiated, and what it is about is the width: a period, a compare
// and a capture that do not fit sixteen bits, and a counter that reads
// back whole.
//
// TIM5 has no remap COLUMN either - the AFIO field that names it moves
// its channel 4 to the LSI instead of a pad (afio.hpp's
// Remap::tim5_ch4), which is not a column of pads - so remap() answers
// false here and the pads are the datasheet's fixed ones.
#include "ch32v203/tim.hpp"

using namespace brio;

using T5 = Tim<5>;

static_assert(tim_present(5) && device::has_tim5 && device::is_d8_class);
static_assert(T5::counter_bits == 32 && T5::max_period == 0xFFFFFFFFu);
static_assert(T5::channels == 4 && T5::complementary_channels == 0);
static_assert(!T5::has_break && !T5::has_repetition);
static_assert(T5::has_slave_mode && T5::has_encoder && T5::has_master_mode);
static_assert(!T5::on_pb2 && tim_gate(5) == rcc_pb1_tim5);

// The sixteen-bit timers beside it, for contrast.
static_assert(Tim<2>::max_period == 0xFFFFu && tim_counter_bits(2) == 16);

// Its own trigger table (15-2): TIM2, TIM3, TIM4 and a TIM8 this family
// has not got.
static_assert(tim_internal_trigger(5, 0) == 2 && tim_internal_trigger(5, 2) == 4);
static_assert(tim_internal_trigger(5, 3) == 0);
// And what reaches IT: the 32-bit timer is TIM1's ITR0 and TIM3's ITR2.
static_assert(tim_internal_trigger(1, 0) == 5 && tim_internal_trigger(3, 2) == 5);

void exercise_wide() {
    T5::init();
    (void)T5::remap(0);                       // no column: false
    (void)T5::configure({.prescaler = 0, .period = 0x0001FFFFu});
    (void)T5::set_period(0xFFFFFFFFu);
    (void)T5::set_compare(0, 0x0001FFFFu);
    (void)T5::compare(0);
    T5::set_count(0x00020000u);
    (void)T5::count();
    (void)T5::capture_channel(1, {.select = TimChannelSelect::direct});
    (void)T5::master(TimMasterMode::update);
    (void)T5::slave({.mode = TimSlaveMode::external_clock1, .trigger = TimTrigger::itr0});
    (void)T5::isr();
    T5::release();
}

void exercise_wide_tasks() {
    using Interval = TimIntervalMeter<T5, 3>;
    (void)Interval::setup(0, 0, TimCapturePolarity::rising);
    (void)Interval::interval();
    using Knob = TimEncoder<T5>;
    (void)Knob::setup({}, 0x00100000u);
    (void)Knob::count();
}

extern "C" BRIO_CH32_INTERRUPT void tim5_handler() { (void)Tim<5>::isr(); }
