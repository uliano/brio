// Family smoke TU: stm32g0/dma.hpp instantiated for every DMA controller
// and every channel each of the three device headers declares, plus the
// whole DMAMUX surface and the four engines at every element width.
// Instantiation only - no main(), no hardware.
//
// WHAT THIS FIXTURE IS REALLY FOR. Three things this stratum cannot check
// any other way:
//
//  1. THE PER-PART GEOMETRY. DMA2 exists on the G0B1 class alone; DMA1
//     has seven channels there and on the G071 class, five on the G031
//     class; the DMAMUX has twelve, seven and five multiplexer channels
//     to match. All of it is counted off the device header's own
//     DMAn_ChannelK_BASE / DMAMUX1_ChannelK_BASE symbols in
//     stm32g0/device_tables.hpp, and this file is where the counts are
//     re-stated as static_asserts.
//
//  2. THE VECTOR MAP. Three lines serve twelve channels and the third
//     one's NAME differs on all three headers
//     (DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR / DMA1_Ch4_7_DMAMUX1_OVR /
//     DMA1_Ch4_5_DMAMUX1_OVR). A wrong line is a silent Default_Handler
//     spin - the samc stratum's NMI lesson - so every present channel's
//     irq() is evaluated here on every header.
//
//  3. THE util CONTRACTS. util/block_stream.hpp was written BEFORE its
//     second implementation exactly so that friction would show up as a
//     concept that does not fit. These static_asserts are that
//     measurement: DmaLoopEngine is a BlockPlayer and DmaPingPongEngine a
//     BlockSource at every width, unchanged, on the third architecture -
//     and DmaTxEngine is NEITHER, which is what makes the two concepts
//     say something (test/family_stm32g0/neg/dma_tx_engine_is_no_source.cpp
//     is the refusal side of the same claim).

#include <stdint.h>

#include "stm32g0/dma.hpp"
#include "stm32g0/tim.hpp"
#include "stm32g0/usart.hpp"
#include "util/block_stream.hpp"

using namespace brio;

// ---- presence and geometry, as the three headers state it ------------------
static_assert(dma_present(1), "every part of this family has DMA1");
static_assert(!dma_present(3), "there is no third controller anywhere");

#if defined(STM32G0B1xx)
static_assert(dma_present(2), "the G0B1 class has a second controller");
static_assert(dma_channels(1) == 7 && dma_channels(2) == 5);
static_assert(dmamux_channels() == 12, "table 54: twelve on the G0B1/G0C1");
#elif defined(STM32G071xx)
static_assert(!dma_present(2), "the G071 class has one controller");
static_assert(dma_channels(1) == 7);
static_assert(dmamux_channels() == 7, "table 54: seven on the G071/G081");
#else
static_assert(!dma_present(2), "the G031 class has one controller");
static_assert(dma_channels(1) == 5);
static_assert(dmamux_channels() == 5, "table 54: five on the G031/G041");
#endif

static_assert(dmamux_generators() == 4, "table 54: four request generators everywhere");

// 11.3.2's hardwired map, at both ends of it.
static_assert(dmamux_channel_of(1, 1) == 0);
static_assert(dmamux_channel_of(1, dma_channels(1)) == dma_channels(1) - 1u);
#if defined(STM32G0B1xx)
static_assert(dmamux_channel_of(2, 1) == 7);
static_assert(dmamux_channel_of(2, 5) == 11);
#endif
static_assert(dmamux_channel_of(1, 8) == 0xFF, "no controller of this family has eight");

// ---- the widths ------------------------------------------------------------
static_assert(dma_width_of<uint8_t>() == DmaWidth::byte);
static_assert(dma_width_of<uint16_t>() == DmaWidth::half);
static_assert(dma_width_of<uint32_t>() == DmaWidth::word);
static_assert(dma_width_bytes(DmaWidth::byte) == 1);
static_assert(dma_width_bytes(DmaWidth::half) == 2);
static_assert(dma_width_bytes(DmaWidth::word) == 4);

// 10.4.5's own rule, said three times in the chapter and enforced once here.
static_assert(!dma_channel_config_valid({.circular = true, .memory_to_memory = true}));
static_assert(dma_channel_config_valid({.circular = true}));
static_assert(dma_channel_config_valid({.memory_to_memory = true}));

// ES0548 2.4.1 made structural: the global flag is not in the clearable set.
static_assert((DmaFlag::all & DmaFlag::global) == 0u,
              "DmaFlag::all must not contain GIFx - clearing it is the erratum");

// ---- every channel of every present controller ------------------------------
namespace {

volatile uint32_t sink;
uint8_t bytes[8];
volatile uint8_t rx_bytes[2][8];
uint16_t halves[8];
volatile uint16_t rx_halves[2][8];
uint32_t words[8];
volatile uint32_t rx_words[2][8];

template <uint8_t n, uint8_t ch>
void one_channel() {
    using C = DmaChannel<n, ch>;
    static_assert(C::index == ch);
    static_assert(C::controller == n);
    static_assert(C::mux_channel < dmamux_channels());
    (void)C::regs();
    (void)C::irq();
    (void)C::enabled();
    (void)C::enable(false);
    (void)C::configure({});
    (void)C::control();
    (void)C::circular();
    (void)C::memory_to_memory();
    (void)C::set_count(4);
    (void)C::count();
    C::set_peripheral(&sink);
    C::set_memory(bytes);
    (void)C::load(DmaTransfer{.peripheral = &sink, .memory = bytes, .count = 4});
    (void)C::prepare(DmaTransfer{.peripheral = &sink, .memory = bytes, .count = 4});
    (void)C::flags();
    (void)C::flag(DmaFlag::complete);
    C::clear(DmaFlag::all);
    C::arm(DmaFlag::all, true);
    (void)C::armed();
    (void)C::isr();
    (void)C::progress(4);
    C::stop();
}

template <uint8_t n, uint8_t... chs>
void channels_of(std::integer_sequence<uint8_t, chs...>) {
    (one_channel<n, static_cast<uint8_t>(chs + 1u)>(), ...);
}

template <uint8_t n>
void one_controller() {
    using D = Dma<n>;
    static_assert(D::channels == dma_channels(n));
    (void)D::regs();
    D::bus_clock(true);
    (void)D::bus_clock();
    D::reset();
    (void)D::flags();
    (void)D::irq(1);
    channels_of<n>(std::make_integer_sequence<uint8_t, dma_channels(n)>{});
}

template <uint8_t x>
void one_generator() {
    using G = DmaMuxGenerator<x>;
    static_assert(G::request_id == x + 1u);
    (void)static_cast<uint32_t>(G::rgcr());
    (void)G::configure(dmamux_trigger_exti(0), DmaMuxEdge::both, 2);
    G::enable(true);
    (void)G::enabled();
    G::overrun_interrupt(true);
    (void)G::overrun();
    G::clear_overrun();
    G::release();
}

void the_multiplexer() {
    static_assert(DmaMux::channels == dmamux_channels());
    static_assert(DmaMux::generators == 4);
    (void)DmaMux::request(0, 5);
    (void)DmaMux::request(0);
    (void)DmaMux::request_synchronized(
        0, 5, DmaMuxSync{.input = dmamux_trigger_exti(3), .edge = DmaMuxEdge::rising});
    (void)DmaMux::request_counted(0, 5, 4);
    (void)DmaMux::release(0);
    (void)DmaMux::overruns();
    (void)DmaMux::overrun(0);
    DmaMux::clear_overrun(1u);
    one_generator<0>();
    one_generator<1>();
    one_generator<2>();
    one_generator<3>();
    // Table 56, both halves of it.
    static_assert(dmamux_trigger_exti(15) == 15);
    static_assert(dmamux_trigger_event(0) == 16);
    static_assert(dmamux_trigger_event(3) == 19);
}

/// The four engines at one width, on one channel. Every verb touched, so
/// a template that does not compile is caught here and not at a bench.
template <uint8_t n, uint8_t ch, typename Elem, typename Buf>
void engines_at(Elem* table, Buf* a, Buf* b) {
    using Tx = DmaTxEngine<n, ch, Elem>;
    using Rx = DmaRxEngine<n, ch, Elem>;
    using Loop = DmaLoopEngine<n, ch, Elem>;
    using Pong = DmaPingPongEngine<n, ch, Elem>;

    static_assert(Tx::width == dma_width_of<Elem>());
    static_assert(BlockPlayer<Loop>, "DmaLoopEngine must satisfy util's BlockPlayer");
    static_assert(BlockSource<Pong>, "DmaPingPongEngine must satisfy util's BlockSource");
    static_assert(!BlockSource<Loop>, "a player is not a source");
    static_assert(!BlockPlayer<Tx>, "a byte-transport engine is neither");
    static_assert(!BlockSource<Tx>);
    static_assert(!BlockSource<Rx>, "an RX engine hands over no BLOCK - it is asked");

    Tx::arm(&sink, 51);
    (void)Tx::start(table, 4);
    (void)Tx::service();
    (void)Tx::complete();
    (void)Tx::busy();
    (void)Tx::in_flight();
    (void)Tx::progress();
    (void)Tx::abandon();
    (void)Tx::faults();
    Tx::clear_faults();
    Tx::stop();

    Rx::arm(&sink, 50);
    (void)Rx::start(table, 4);
    (void)Rx::idle();
    (void)Rx::take();
    (void)Rx::full();
    (void)Rx::capacity();
    (void)Rx::taken();
    (void)Rx::service();
    (void)Rx::abandon();
    (void)Rx::faults();
    Rx::stop();

    Loop::arm(&sink, 31);
    (void)Loop::start(table, 4);
    (void)Loop::complete();
    Loop::fail();
    (void)Loop::laps();
    (void)Loop::faults();
    (void)Loop::running();
    (void)Loop::length();
    (void)Loop::progress();
    (void)Loop::service();
    (void)Loop::abandon();
    Loop::clear_faults();
    Loop::stop();

    Pong::arm(&sink, 37);
    (void)Pong::start(a, b, 4);
    (void)Pong::complete();
    Pong::fail();
    (void)Pong::ready();
    (void)Pong::ready_length();
    (void)Pong::release();
    (void)Pong::laps();
    (void)Pong::overruns();
    (void)Pong::stalled();
    (void)Pong::running();
    (void)Pong::pending();
    (void)Pong::progress();
    (void)Pong::service();
    (void)Pong::abandon();
    (void)Pong::faults();
    Pong::clear_faults();
    Pong::stop();
}

}   // namespace

void family_stm32g0_dma();
void family_stm32g0_dma() {
    one_controller<1>();
#if defined(STM32G0B1xx)
    one_controller<2>();
#endif
    the_multiplexer();

    engines_at<1, 1, uint8_t>(bytes, rx_bytes[0], rx_bytes[1]);
    engines_at<1, 2, uint16_t>(halves, rx_halves[0], rx_halves[1]);
    engines_at<1, 3, uint32_t>(words, rx_words[0], rx_words[1]);
#if defined(STM32G0B1xx)
    engines_at<2, 5, uint32_t>(words, rx_words[0], rx_words[1]);
#endif
}

// ---- what the peripherals publish ------------------------------------------
// The request ids are RM0444 table 55's and no header of this pack declares
// one, so the fixture is where the two publishing drivers are held to the
// SAME table: a request line belongs to exactly one peripheral function,
// and a duplicate here would be two channels fighting over one row
// (11.4.4's caution).
static_assert(Usart<1>::dma_rx_request() == 50 && Usart<1>::dma_tx_request() == 51);
static_assert(Usart<2>::dma_rx_request() == 52 && Usart<2>::dma_tx_request() == 53);
static_assert(Tim<1>::dma_update_request() == 25);
static_assert(Tim<2>::dma_update_request() == 31 && Tim<2>::dma_compare_request(0) == 26);
static_assert(Tim<3>::dma_update_request() == 37 && Tim<3>::dma_trigger_request() == 36);
static_assert(Tim<16>::dma_update_request() == 46 && Tim<16>::dma_compare_request(0) == 44);
static_assert(Tim<14>::dma_update_request() == dma_request_none,
              "DS13560 table 7: TIM14 generates no DMA request at all");
static_assert(!Tim<14>::has_dma_request);
static_assert(Tim<2>::dma_compare_request(4) == dma_request_none,
              "a channel past the count has no request line");
