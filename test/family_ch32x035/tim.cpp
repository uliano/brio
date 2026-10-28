// TIM family smoke TU (RM ch. 12), every part: the map against table
// 12-3, the two advanced-control timers' facts, their DMA rows against
// figure 9-1, the columns against what each package bonds, and every verb
// of the subset on both instances - instantiation only, no main(), no
// hardware. A pad a package does not bond is formed only inside an
// `if constexpr` that asks first, so this TU compiles on all seven parts.
#include <stddef.h>

#include "ch32x035/clock.hpp"
#include "ch32x035/dma.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/tim.hpp"

using namespace brio;

using P = Ch32x035Platform<>;
using SysClock = Clock<ClockSource::internal, 48'000'000>;

// ---- the map (table 12-3) ---------------------------------------------------
static_assert(tim_base_for(1) == 0x40012C00 && tim_base_for(2) == 0x40000000 && tim_base_for(3) == 0);
static_assert(offsetof(TimRegs, CTLR2) == 0x04 && offsetof(TimRegs, SMCFGR) == 0x08 &&
              offsetof(TimRegs, DMAINTENR) == 0x0c && offsetof(TimRegs, INTFR) == 0x10 &&
              offsetof(TimRegs, SWEVGR) == 0x14 && offsetof(TimRegs, CHCTLR1) == 0x18 &&
              offsetof(TimRegs, CHCTLR2) == 0x1c && offsetof(TimRegs, CCER) == 0x20 &&
              offsetof(TimRegs, CNT) == 0x24 && offsetof(TimRegs, PSC) == 0x28 &&
              offsetof(TimRegs, ATRLR) == 0x2c && offsetof(TimRegs, RPTCR) == 0x30 &&
              offsetof(TimRegs, CHCVR) == 0x34 && offsetof(TimRegs, BDTR) == 0x44 &&
              offsetof(TimRegs, DMACFGR) == 0x48 && offsetof(TimRegs, DMAADR) == 0x4c &&
              offsetof(TimRegs, SPEC) == 0x50 && sizeof(TimRegs) == 0x54);
static_assert(tim_clock_hz(SysClock{}) == 48'000'000UL);

// ---- the facts and the vectors (table 7-1) ----------------------------------
static_assert(Tim<1>::channels == 4 && Tim<1>::complementary_channels == 3 && Tim<2>::complementary_channels == 3);
static_assert(static_cast<uint8_t>(Tim<1>::update_irq()) == 35 && static_cast<uint8_t>(Tim<1>::cc_irq()) == 37 &&
              static_cast<uint8_t>(Tim<2>::update_irq()) == 38 && static_cast<uint8_t>(Tim<2>::cc_irq()) == 51);

// ---- the DMA requests (figure 9-1) ------------------------------------------
static_assert(Tim<1>::compare_dma_channel(0) == 2 && Tim<1>::compare_dma_channel(1) == 3 &&
              Tim<1>::compare_dma_channel(2) == 6 && Tim<1>::compare_dma_channel(3) == 4 &&
              Tim<1>::compare_dma_channel(4) == 0 && Tim<1>::update_dma_channel == 5);
static_assert(Tim<2>::compare_dma_channel(0) == 5 && Tim<2>::compare_dma_channel(1) == 7 &&
              Tim<2>::compare_dma_channel(2) == 1 && Tim<2>::compare_dma_channel(3) == 7 &&
              Tim<2>::update_dma_channel == 2);

// ---- the columns (8.3.2.1) ---------------------------------------------------
static_assert(afio_tim1_pads(1).ch2n == Pad{'B', 0} && afio_tim1_pads(1).ch3n == Pad{'B', 1} &&
              afio_tim1_pads(0).ch3 == Pad{'B', 11} && !afio_tim1_pads(5).ch1.valid() &&
              afio_tim1_pads(6).ch3 == Pad{'C', 14});
static_assert(afio_tim2_pads(5).ch1 == Pad{'C', 19} && tim_pad_on_debug_port(afio_tim2_pads(5).ch1n) &&
              afio_tim2_pads(7) == afio_tim2_pads(6) && !afio_tim2_pads(8).ch1.valid());
// A code is a column of this package only where it bonds a pad of it.
static_assert(afio_remap_has_code(Remap::tim1, 1) == afio_tim1_pads(1).any_bonded());
static_assert(afio_remap_has_code(Remap::tim2, 4) == afio_tim2_pads(4).any_bonded());
static_assert(!afio_remap_has_code(Remap::tim1, 8) && !afio_remap_has_code(Remap::tim2, 8));

template <uint8_t n>
void resource_verbs() {
    using T = Tim<n>;
    T::init();
    (void)T::bus_clock();
    (void)T::remap(1);
    (void)T::remap();
    static_assert(T::column(0).ch1.valid());
    (void)T::configure({.prescaler = 47, .period = 999, .direction = TimDirection::down,
                        .alignment = TimAlignment::center_both, .clock_division = TimClockDivision::div4,
                        .auto_reload_preload = true, .one_pulse = true, .update_disable = false,
                        .update_on_overflow_only = true, .repetition = 3});
    T::enable(true); (void)T::enabled();
    (void)T::count(); T::set_count(5);
    (void)T::prescaler(); T::set_prescaler(1);
    (void)T::period(); (void)T::set_period(100);
    (void)T::repetition(); T::set_repetition(2); (void)T::one_pulse();
    T::update();
    (void)T::flags(); (void)T::flag(T::update_flag); T::clear_flags(T::compare_flag(2));
    T::interrupts(T::update_interrupt | T::compare_interrupt(1) | T::update_dma | T::compare_dma(0), true);
    (void)T::interrupts();
    (void)T::isr();
    (void)T::ccr_address(0);
    (void)T::compare(0); (void)T::set_compare(3, 10);
    (void)T::output_channel(0, {.mode = TimOutputMode::pwm1, .compare = 100, .preload = true, .active_low = true,
                                .complementary_active_low = true, .enable = true, .complementary_enable = true});
    (void)T::output_channel(3, {.mode = TimOutputMode::toggle, .preload = false});
    (void)T::output_mode(1, TimOutputMode::force_active); (void)T::output_mode(1);
    (void)T::channel_enable(2, true); (void)T::channel_enabled(2);
    (void)T::complementary_enable(1, true); (void)T::complementary_enabled(1);
    T::main_output(true); (void)T::main_output();
    T::release();
}

template void resource_verbs<1>();
template void resource_verbs<2>();

// ---- a pad from its column, where the package bonds it alone on its pin ------
template <Pad pad>
bool claim_where_bonded() {
    if constexpr (pad_can_drive(pad)) {
        return TimPad<pad>::claim();
    } else {
        return false;
    }
}
bool pads() {
    return claim_where_bonded<afio_tim1_pads(1).ch2n>() && claim_where_bonded<afio_tim1_pads(1).ch3n>() &&
           claim_where_bonded<afio_tim2_pads(6).ch1>() && claim_where_bonded<afio_tim1_pads(0).ch3>();
}

// The compare request feeds a transmit engine on the table's channel.
using Edges = DmaTxEngine<Tim<1>::compare_dma_channel(1), uint32_t>;
void engine() {
    Edges::arm(Tim<1>::ccr_address(1), 0u, DmaPriority::very_high);
    static const uint32_t list[2] = {10, 20};
    (void)Edges::start(std::span<const uint32_t>(list));
}

// The ISR body bound to a vector, both prologues.
BRIO_CH32_VECTOR(tim1_cc_handler) { (void)Tim<1>::isr(); }
