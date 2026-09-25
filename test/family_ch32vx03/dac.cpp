// mcu: ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// DAC family smoke TU: the converter of RM ch. 17 - every verb of `Dac`,
// both pads, the three formats and the dual ones, the triggers folded
// through each part's timers, the two generators, and the DMA streams on
// DMA2's channels 3 and 4 with the block player the loop runs on.
//
// EVERY CH32V303 HAS IT, NOT EVERY TRIGGER. The 128 KB CB and RB have
// TIM2 and TIM4 among table 17-1's six timers and none of TIM5..TIM8; the
// RC and the VC have all six. So this TU names TIM2, TIM4, EXTI9 and the
// software trigger as literals and asks `dac_trigger_valid()` for the
// rest; neg/dac_trigger_timer_absent.cpp is the refusal.
#include "ch32vx03/dac.hpp"
#include "ch32vx03/dma.hpp"
#include "util/block_stream.hpp"

using namespace brio;

// ---- the triggers, by part --------------------------------------------------
static_assert(dac_trigger_valid(DacTrigger::tim2_trgo) && dac_trigger_valid(DacTrigger::tim4_trgo));
static_assert(dac_trigger_valid(DacTrigger::tim5_trgo) == device::has_tim5);
static_assert(dac_trigger_valid(DacTrigger::tim6_trgo) ==
              ((device::basic_timer_instances & (1U << 6)) != 0u));
static_assert(dac_trigger_valid(DacTrigger::tim7_trgo) ==
              ((device::basic_timer_instances & (1U << 7)) != 0u));
static_assert(dac_trigger_valid(DacTrigger::tim8_trgo) ==
              ((device::advanced_timer_instances & (1U << 8)) != 0u));
static_assert(dac_exti_line == 9);

// ---- the DMA slots (table 11-3) ---------------------------------------------
static_assert(Dac::dma_slot(1) == DmaSlot{2, 3} && Dac::dma_slot(2) == DmaSlot{2, 4});
static_assert(dac_dma_slot(3) == DmaSlot{});
static_assert(Dac::steps == 4096 && Dac::max_code == 4095 && Dac::channels == 2);
static_assert(Dac::shift_of(1) == 0 && Dac::shift_of(2) == 16);
static_assert(Dac::channel_valid(1) && Dac::channel_valid(2) && !Dac::channel_valid(0) &&
              !Dac::channel_valid(3));

// ---- the arithmetic (util/analog.hpp) ---------------------------------------
static_assert(Dac::code_for_mv(1650, 3300) == 2048);
static_assert(Dac::mv_for_code(4095, 3300) == 3299);

// ---- the pads -----------------------------------------------------------------
using Out1 = DacOut<1>;
using Out2 = DacOut<2>;
static_assert(Out1::pad == Pad{'A', 4} && Out1::adc_channel == 4);
static_assert(Out2::pad == Pad{'A', 5} && Out2::adc_channel == 5);

// ---- the streams --------------------------------------------------------------
using Dac1Row = DmaRequestOf<DmaRequest::dac1>;
using Dac2Row = DmaRequestOf<DmaRequest::dac2>;
using Wave1 = DmaLoopEngine<Dac1Row::controller, Dac1Row::channel, uint16_t>;
using Wave2 = DmaLoopEngine<Dac2Row::controller, Dac2Row::channel, uint8_t>;
using Both = DmaLoopEngine<Dac1Row::controller, Dac1Row::channel, uint32_t>;
static_assert(BlockPlayer<Wave1> && BlockPlayer<Both>);

constexpr uint16_t table[4] = {0, 1024, 2048, 4095};
constexpr uint32_t dual_table[2] = {0x0FFF'0000UL, 0x0000'0FFFUL};
constexpr uint8_t byte_table[2] = {0x00, 0xFF};

void block_verbs() {
    Dac::init();
    (void)Dac::bus_clock();
    Dac::bus_clock(true);
    (void)Dac::regs();
    Out1::claim();
    Out2::claim();
    Out1::release();
}

void configuration_verbs() {
    (void)Dac::configure(1, DacChannelConfig{});
    (void)Dac::configure(2, DacChannelConfig{.buffered = false, .triggered = true,
                                             .trigger = DacTrigger::tim4_trgo,
                                             .wave = DacWave::noise, .amplitude = 11});
    (void)Dac::configure(3, DacChannelConfig{});   // refused: no such channel
    Dac::configure<1, DacChannelConfig{.triggered = true, .trigger = DacTrigger::tim2_trgo,
                                       .dma = true}>();
    Dac::configure<2, DacChannelConfig{.triggered = true, .trigger = DacTrigger::exti9,
                                       .wave = DacWave::triangle, .amplitude = 7}>();
    Dac::configure<1, DacChannelConfig{.triggered = true, .trigger = DacTrigger::software}>();
    (void)Dac::configuration(1);
    (void)Dac::configuration(2);
    (void)Dac::enable(1, true);
    (void)Dac::enabled(1);
    (void)Dac::enable(2, false);
    (void)Dac::wave(1, DacWave::triangle);
    (void)Dac::wave(1);
    (void)Dac::dma(1, true);
    (void)Dac::dma(1);
}

/// A trigger of a timer only some CH32V303 have, configured where the part
/// has it: a template, so a part without it never instantiates the line.
template <DacTrigger t>
void configure_if_present() {
    if constexpr (dac_trigger_valid(t)) {
        Dac::configure<1, DacChannelConfig{.triggered = true, .trigger = t, .dma = true}>();
    }
}

void trigger_verbs() {
    configure_if_present<DacTrigger::tim5_trgo>();
    configure_if_present<DacTrigger::tim6_trgo>();
    configure_if_present<DacTrigger::tim7_trgo>();
    configure_if_present<DacTrigger::tim8_trgo>();
    (void)Dac::software_trigger(1);
    (void)Dac::software_trigger(2);
    Dac::software_trigger_both();
}

void data_verbs() {
    (void)Dac::write(1, 2048);
    (void)Dac::write_left(2, 0x8000);
    (void)Dac::write8(1, 0x80);
    Dac::write_dual(1024, 3072);
    Dac::write_dual_left(0x4000, 0xC000);
    Dac::write_dual8(0x40, 0xC0);
    (void)Dac::code(1);
    (void)Dac::output(1);
    (void)Dac::output(2);
    (void)Dac::data_address(1, DacFormat::right12);
    (void)Dac::data_address(2, DacFormat::left12);
    (void)Dac::data_address(2, DacFormat::right8);
    (void)Dac::dual_data_address(DacFormat::right12);
}

void stream_verbs() {
    Dac::claim_stream<1, Wave1>();
    (void)Wave1::start(table, 4);
    Dac::claim_stream<2, Wave2, DacFormat::right8>();
    (void)Wave2::start(byte_table, 2);
    Dac::claim_dual_stream<Both>();
    (void)Both::start(dual_table, 2);
    Wave1::stop();
    Wave2::stop();
    Both::stop();
    Dac::release();
}

// The player's lap is counted in DMA2 channel 3's handler, as an
// application binds it.
extern "C" BRIO_CH32_INTERRUPT void dma2_channel3_handler() {
    const uint8_t f = Wave1::service();
    if ((f & Wave1::flag_complete) != 0u) {
        Wave1::lap();
    }
    if ((f & Wave1::flag_error) != 0u) {
        Wave1::fail();
    }
}
