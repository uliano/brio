// mcu: ch32v303rc ch32v303vc
// The timers the 256 KB CH32V303 adds that have CHANNELS: three more
// advanced-control blocks (TIM8, TIM9, TIM10) with TIM1's shape - the
// break, the dead time, the repetition counter, the complementary
// outputs and FOUR unshared vectors each - and a general-purpose TIM5
// that is sixteen bits wide on this class (chapter 15's note gives
// thirty-two to the CH32V20x_D8's alone). The datasheet's table 2-1-1
// gives all four to the RC and the VC and none to the 128 KB CB and RB;
// the internal triggers of table 14-2, the remap columns of tables
// 10-20..10-22 and the vector table's tail (59..62, 90..93, 94..97, 66)
// are the rest of what this TU pins.
#include "ch32vx03/platform.hpp"
#include "ch32vx03/tim.hpp"

using namespace brio;

using T5 = Tim<5>;
using T8 = Tim<8>;
using T9 = Tim<9>;
using T10 = Tim<10>;

// ---- what they ARE ----------------------------------------------------------
static_assert(tim_advanced(8) && tim_advanced(9) && tim_advanced(10) && tim_general(5));
static_assert(T8::channels == 4 && T8::complementary_channels == 3);
static_assert(T9::has_break && T10::has_repetition && T8::has_split_vectors);
static_assert(T8::on_pb2 && T9::on_pb2 && T10::on_pb2 && !T5::on_pb2);
static_assert(tim_gate(8) == rcc_pb2_tim8 && tim_gate(9) == rcc_pb2_tim9 &&
              tim_gate(10) == rcc_pb2_tim10 && tim_gate(5) == rcc_pb1_tim5);
static_assert(T5::counter_bits == 16 && T5::max_period == 0xFFFFu);
static_assert(!T5::has_break && T5::complementary_channels == 0 && !T5::has_remap);
static_assert(T8::has_remap && T8::remap_field == Remap::tim8 && T10::remap_field == Remap::tim10);
static_assert(T8::has_dual_edge_capture && T5::has_dual_edge_capture);

// ---- their vectors: four lines each, and TIM5's one --------------------------
static_assert(T8::irq() == Irq::tim8_up && T8::cc_irq() == Irq::tim8_cc &&
              T8::break_irq() == Irq::tim8_brk && T8::trigger_irq() == Irq::tim8_trg_com);
static_assert(T9::irq() == Irq::tim9_up && T9::cc_irq() == Irq::tim9_cc);
static_assert(T10::break_irq() == Irq::tim10_brk && T10::trigger_irq() == Irq::tim10_trg_com);
static_assert(static_cast<uint8_t>(Irq::tim8_brk) == 59 && static_cast<uint8_t>(Irq::tim9_brk) == 90 &&
              static_cast<uint8_t>(Irq::tim10_cc) == 97 && static_cast<uint8_t>(Irq::tim5) == 66);
static_assert(T5::irq() == Irq::tim5 && T5::cc_irq() == Irq::tim5 && !T5::has_split_vectors);
static_assert(T8::vector_flags(Irq::tim8_up) == tim_uif && T8::vector_flags(Irq::tim8_brk) == tim_bif);
static_assert(T8::vector_flags(Irq::tim1_up) == 0u && T9::vector_flags(Irq::tim8_cc) == 0u);
static_assert(T10::vector_flags(Irq::tim10_trg_com) == (tim_tif | tim_comif));

// ---- table 14-2 --------------------------------------------------------------
static_assert(tim_internal_trigger(8, 0) == 1 && tim_internal_trigger(8, 1) == 2 &&
              tim_internal_trigger(8, 2) == 4 && tim_internal_trigger(8, 3) == 5);
static_assert(tim_internal_trigger(9, 0) == 10 && tim_internal_trigger(9, 1) == 5 &&
              tim_internal_trigger(9, 2) == 6 && tim_internal_trigger(9, 3) == 7);
static_assert(tim_internal_trigger(10, 0) == 9 && tim_internal_trigger(10, 3) == 5);
// And where TIM8 is the MASTER: TIM2's ITR1, TIM4's ITR3, TIM5's ITR3.
static_assert(tim_trigger_index_for(2, 8) == 1 && tim_trigger_index_for(4, 8) == 3 &&
              tim_trigger_index_for(5, 8) == 3);
static_assert(tim_trigger_index_for(9, 6) == 2 && tim_trigger_index_for(9, 7) == 3);

// ---- their pads ----------------------------------------------------------------
static_assert(tim_channel_pad(8, 0, 0) == Pad{'C', 6} && tim_complementary_pad(8, 0, 0) == Pad{'A', 7});
static_assert(tim_break_pad(8, 0) == Pad{'A', 6} && tim_etr_pad(8, 1) == Pad{'A', 0});
static_assert(tim_channel_pad(9, 0, 0) == Pad{'A', 2} && tim_break_pad(9, 0) == Pad{'C', 5});
static_assert(tim_channel_pad(10, 0, 0) == Pad{'B', 8} && tim_etr_pad(10, 0) == Pad{'C', 10});
static_assert(tim_channel_pad(5, 0, 3) == Pad{'A', 3} && tim_channel_pad(5, 1, 0) == Pad{});
static_assert(!tim_etr_pad(5, 0).valid() && !tim_break_pad(5, 0).valid());
// The LQFP100's columns: port D and port E.
static_assert(afio_remap_has_code(Remap::tim9, 2) == device::has_port('E'));
static_assert(afio_remap_has_code(Remap::tim10, 3) == device::has_port('E'));

using T8Out = TimPad<tim_channel_pad(8, 0, 0)>;
using T8OutN = TimPad<tim_complementary_pad(8, 0, 0)>;
using T8Break = TimPad<tim_break_pad(8, 0)>;

// ---- the tasks on them ----------------------------------------------------------
using Pair8 = TimPairPwm<T8, 0, 1000>;
using Pair9 = TimPairPwm<T9, 2, 1000>;
using Pwm10 = TimPwm<T10, 3, 500>;
using Period5 = TimPeriodMeter<T5>;
using Tick9 = TimPeriodicTick<T9>;
using Pulse8 = TimOnePulse<T8, 1>;
using Knob10 = TimEncoder<T10>;
using Count8 = TimEventCounter<T8>;
using Gate9 = TimGatedCounter<T9>;

void exercise_advanced_trio() {
    T8::init();
    (void)T8::remap(1);
    (void)T8::remap();
    (void)T8::configure({.prescaler = 0, .period = 1000, .repetition = 2});
    (void)T8::output_channel(0, {.mode = TimOutputMode::pwm1,
                                 .compare = 500,
                                 .complementary_enable = true,
                                 .idle_high = true});
    (void)T8::break_dead_time({.dead_time = tim_dead_time_code(100),
                               .main_output_enable = true,
                               .break_enable = true});
    (void)T8::commutation_event();
    (void)T8::preload_channels(true, false);
    (void)T8::isr(T8::vector_flags(Irq::tim8_cc));
    (void)T8::dual_edge_capture(1);
    (void)T8::dual_edge(1);
    (void)T8::dual_edge_capture_off(1);
    T8::release();

    T9::init();
    (void)T9::remap(0);
    (void)T9::slave({.mode = TimSlaveMode::external_clock1, .trigger = TimTrigger::itr2});
    (void)T9::master(TimMasterMode::oc1ref);
    T9::release();

    T10::init();
    (void)T10::external_trigger({.prescaler = 1});
    (void)T10::dma_burst(TimBurstBase::ch1cvr, 4);
    (void)T10::dmaadr_address();
    T10::release();

    T5::init();
    (void)T5::remap(0);
    (void)T5::configure({.prescaler = 0, .period = 0xFFFF});
    (void)T5::capture_channel(3, {.select = TimChannelSelect::direct});
    T5::release();

    (void)Pair8::setup(0, tim_dead_time_code(40), TimClockDivision::div2);
    Pair8::duty(300);
    (void)Pair9::setup(1, 10);
    (void)Pwm10::setup(0);
    Pwm10::duty(100);
    (void)Period5::setup(143);
    (void)Tick9::setup(143, 999);
    Tick9::stop();
    (void)Pulse8::setup(143, 10, 20);
    Pulse8::fire();
    (void)Knob10::setup();
    (void)Count8::setup(TimTrigger::itr0);
    (void)Gate9::setup(TimTrigger::itr1);

    T8Out::claim();
    T8OutN::claim();
    T8Break::claim_input(PinPull::down);
}

// Their vector bodies, the four lines of an advanced timer each bound.
extern "C" BRIO_CH32_INTERRUPT void tim8_up_handler() { (void)T8::isr(T8::vector_flags(Irq::tim8_up)); }
extern "C" BRIO_CH32_INTERRUPT void tim8_cc_handler() { (void)T8::isr(T8::vector_flags(Irq::tim8_cc)); }
extern "C" BRIO_CH32_INTERRUPT void tim8_brk_handler() { (void)T8::isr(T8::vector_flags(Irq::tim8_brk)); }
extern "C" BRIO_CH32_INTERRUPT void tim8_trg_com_handler() {
    (void)T8::isr(T8::vector_flags(Irq::tim8_trg_com));
}
extern "C" BRIO_CH32_INTERRUPT void tim9_up_handler() { (void)T9::isr(T9::vector_flags(Irq::tim9_up)); }
extern "C" BRIO_CH32_INTERRUPT void tim10_cc_handler() { (void)T10::isr(T10::vector_flags(Irq::tim10_cc)); }
extern "C" BRIO_CH32_INTERRUPT void tim5_handler() { (void)T5::isr(); }
