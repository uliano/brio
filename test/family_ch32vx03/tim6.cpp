// mcu: ch32v303rc ch32v303vc
// The BASIC TIMERS (RM ch. 16), which of this family's thirteen parts
// only the 256 KB CH32V303 carry: a time base and nothing else - an up
// counter behind a prescaler, the update event as an interrupt, a DMA
// request and a TRGO of three codes, no channel, no slave controller, no
// burst engine. What this TU proves is the shape: every verb a basic
// timer HAS compiles and every feature it has not answers false, while
// its channel verbs do not compile at all (neg/tim_basic_channel.cpp).
// Its TRGO reaches TIM9's internal triggers 2 and 3 (table 14-2), which
// is how a program - and the reference suite - watches it with no pad.
#include "ch32vx03/platform.hpp"
#include "ch32vx03/tim.hpp"

using namespace brio;

using T6 = Tim<6>;
using T7 = Tim<7>;

static_assert(tim_basic(6) && tim_basic(7) && T6::is_basic);
static_assert(T6::channels == 0 && T7::channels == 0 && T6::complementary_channels == 0);
static_assert(!T6::has_break && !T6::has_repetition && !T6::has_slave_mode && !T6::has_encoder);
static_assert(T6::has_master_mode && T6::has_dma && !T6::has_dma_burst && !T6::has_up_down);
static_assert(!T6::has_external_trigger && !T6::has_ti1_xor && !T6::has_dual_edge_capture);
static_assert(!T6::has_remap && !T6::on_pb2 && tim_gate(6) == rcc_pb1_tim6 && tim_gate(7) == rcc_pb1_tim7);
static_assert(T6::counter_bits == 16);
static_assert(T6::irq() == Irq::tim6 && T7::irq() == Irq::tim7 && !T6::has_split_vectors);
static_assert(static_cast<uint8_t>(Irq::tim6) == 70 && static_cast<uint8_t>(Irq::tim7) == 71);
static_assert(T6::interrupt_flags == tim_uif && T6::all_flags == tim_uif);
static_assert(T6::vector_flags(Irq::tim6) == tim_uif && T6::vector_flags(Irq::tim7) == 0u);
// A basic timer listens to nobody and is heard by TIM9 alone.
static_assert(tim_internal_trigger(6, 0) == 0 && tim_trigger_index_for(9, 6) == 2);
static_assert(!tim_channel_pad(6, 0, 0).valid() && !tim_etr_pad(7, 0).valid());
// Its time base is up-counting and edge-aligned with no CKD.
static_assert(T6::config_valid({.prescaler = 143, .period = 999}));
static_assert(!T6::config_valid({.period = 999, .direction = TimDirection::down}));
static_assert(!T6::config_valid({.period = 999, .alignment = TimAlignment::center_up}));
static_assert(!T6::config_valid({.period = 999, .clock_division = TimClockDivision::div2}));
static_assert(!T6::config_valid({.period = 999, .repetition = 1}));

using Tick6 = TimPeriodicTick<T6>;
using Tick7 = TimPeriodicTick<T7>;

void exercise_basic() {
    T6::init();
    (void)T6::bus_clock();
    (void)T6::configure({.prescaler = 143, .period = 999, .auto_reload_preload = true,
                         .one_pulse = true, .update_on_overflow_only = true});
    (void)T6::master(TimMasterMode::update);
    (void)T6::master(TimMasterMode::enable);
    (void)T6::master(TimMasterMode::oc1ref);            // false: no channel to publish
    (void)T6::master(TimMasterMode::compare_pulse);     // false as well
    (void)T6::master();
    T6::enable(true);
    (void)T6::enabled();
    (void)T6::count();
    T6::set_count(0);
    (void)T6::period();
    (void)T6::set_period(10);
    (void)T6::prescaler();
    T6::set_prescaler(7);
    (void)T6::auto_reload_preload();
    (void)T6::direction();
    T6::update();
    (void)T6::trigger_event();                           // false: no slave controller
    (void)T6::break_event();                             // false: no break unit
    (void)T6::slave({.mode = TimSlaveMode::gated});      // false
    (void)T6::ti1_xor(true);                             // false
    (void)T6::external_trigger({});                      // false
    (void)T6::compare_dma_on_update(true);               // false: no channel
    (void)T6::dma_burst(TimBurstBase::ctlr1, 1);         // false: no burst engine
    (void)T6::burst_base();
    (void)T6::burst_length();
    T6::dma_burst_off();
    (void)T6::dmaadr_address();                          // nullptr
    (void)T6::cnt_address();
    T6::interrupts(T6::update_interrupt | T6::update_dma, true);
    (void)T6::flags();
    T6::clear_flags(T6::all_flags);
    (void)T6::isr();
    (void)T6::remap(0);                                  // false: no pad at all
    (void)T6::remap();
    T6::release();

    (void)Tick6::setup(143, 999);
    Tick6::stop();
    (void)Tick7::setup(0, 0xFFFF, false);
    Tick7::stop();
}

extern "C" BRIO_CH32_INTERRUPT void tim6_handler() { (void)T6::isr(); }
extern "C" BRIO_CH32_INTERRUPT void tim7_handler() { (void)T7::isr(); }
