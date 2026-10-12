// DMA family smoke TU (RM ch. 9): the controller's map against table 9-2,
// the request table against figure 9-1 on every part, the channel's
// one-shot verbs on all eight channels, the three engines with every beat
// they take, and the serial transport with its two engines on every
// instance the part offers - each instance's branch asked of device:: (a
// template, so a part without the instance never forms it).
#include <stddef.h>

#include "ch32x035/clock.hpp"
#include "ch32x035/dma.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/usart.hpp"
#include "util/ring.hpp"
#include "util/stream.hpp"

using namespace brio;

using P = Ch32x035Platform<>;
using SysClock = Clock<ClockSource::internal, 48'000'000>;

// ---- the map (table 9-2) ------------------------------------------------------
static_assert(offsetof(DmaRegs, INTFR) == 0x00 && offsetof(DmaRegs, INTFCR) == 0x04);
static_assert(offsetof(DmaRegs, channel) == 0x08 && sizeof(DmaChannelRegs) == 0x14);
static_assert(offsetof(DmaRegs, channel) + 7 * sizeof(DmaChannelRegs) == 0x94);   // CFGR8
static_assert(offsetof(DmaChannelRegs, CNTR) == 0x04 && offsetof(DmaChannelRegs, PADDR) == 0x08 &&
              offsetof(DmaChannelRegs, MADDR) == 0x0c);
static_assert(dma_channel_count == 8);
// The vectors (table 7-1): 22..28, and 44 for the eighth.
static_assert(dma_channel_irq(1) == Irq::dma1_channel1 && dma_channel_irq(7) == Irq::dma1_channel7);
static_assert(dma_channel_irq(8) == Irq::dma1_channel8 && static_cast<uint8_t>(Irq::dma1_channel8) == 44);
static_assert(static_cast<uint8_t>(dma_channel_irq(4)) == 25);
// CFGR's fields (9.3.3).
static_assert(dma_cfgr_of(DmaChannelConfig{.direction = DmaDirection::memory_to_peripheral,
                                           .circular = true,
                                           .memory_increment = true,
                                           .peripheral_width = DmaWidth::half,
                                           .memory_width = DmaWidth::word,
                                           .priority = DmaPriority::very_high}) ==
              ((1u << 4) | (1u << 5) | (1u << 7) | (1u << 8) | (2u << 10) | (3u << 12)));
static_assert(!dma_channel_config_valid(DmaChannelConfig{.circular = true, .memory_to_memory = true}));
static_assert(dma_cfgr_beat<uint16_t>() == ((1u << 8) | (1u << 10)));
static_assert(dma_aligned(0x20000002u, DmaWidth::half) && !dma_aligned(0x20000002u, DmaWidth::word));

// ---- the request table (figure 9-1) ------------------------------------------
static_assert(dma_request_channel(DmaRequest::adc1) == 1);
static_assert(dma_request_channel(DmaRequest::spi1_rx) == 2 && dma_request_channel(DmaRequest::spi1_tx) == 3);
static_assert(dma_request_channel(DmaRequest::i2c1_tx) == 6 && dma_request_channel(DmaRequest::i2c1_rx) == 7);
static_assert(dma_request_channel(DmaRequest::tim1_ch1) == 2 && dma_request_channel(DmaRequest::tim1_ch2) == 3 &&
              dma_request_channel(DmaRequest::tim1_trig) == 4 && dma_request_channel(DmaRequest::tim1_up) == 5 &&
              dma_request_channel(DmaRequest::tim1_ch3) == 6);
static_assert(dma_request_channel(DmaRequest::tim2_ch3) == 1 && dma_request_channel(DmaRequest::tim2_up) == 2 &&
              dma_request_channel(DmaRequest::tim2_ch1) == 5 && dma_request_channel(DmaRequest::tim2_ch4) == 7 &&
              dma_request_channel(DmaRequest::tim2_com) == 8);
// USART2 and USART3 are every part's; USART1 and USART4 the part table's.
static_assert(usart_dma_tx_channel(2) == 7 && usart_dma_rx_channel(2) == 6);
static_assert(usart_dma_tx_channel(3) == 2 && usart_dma_rx_channel(3) == 3);
static_assert(device::has_usart(1) == (usart_dma_tx_channel(1) == 4u));
static_assert(device::has_usart(1) == (usart_dma_rx_channel(1) == 5u));
static_assert(device::has_usart(4) == (usart_dma_tx_channel(4) == 1u));
static_assert(device::has_usart(4) == (usart_dma_rx_channel(4) == 8u));
static_assert(DmaRequestOf<DmaRequest::usart2_tx>::channel == 7);

// ---- the channel's one-shot verbs, every channel ---------------------------------
template <uint8_t ch>
void channel_verbs() {
    using C = DmaChannel<ch>;
    static uint32_t a[4];
    static uint32_t b[4];
    C::stop();
    (void)C::configure(DmaChannelConfig{.memory_to_memory = true, .peripheral_increment = true});
    (void)C::set_count(4);
    (void)C::count();
    C::set_peripheral(a);
    C::set_memory(b);
    (void)C::prepare(DmaTransfer{.peripheral = a, .memory = b, .count = 4});
    (void)C::load(DmaTransfer{.peripheral = a, .memory = b, .count = 4});
    (void)C::flags();
    (void)C::flag(DmaFlag::complete);
    C::clear(DmaFlag::all);
    C::arm(DmaFlag::complete, true);
    (void)C::isr();
    (void)C::progress(4);
    C::enable(false);
    (void)C::enabled();
    static_assert(C::irq() == dma_channel_irq(ch));
}

template <uint8_t... chs>
void every_channel() {
    (channel_verbs<chs>(), ...);
}

// ---- the engines, every beat ----------------------------------------------------
using Tx = DmaTxEngine<3, uint16_t>;
using Rx = DmaRxEngine<2, uint16_t>;
using Copy = DmaCopyEngine<5>;
using Copy8 = DmaCopyEngine<8, uint8_t>;
static_assert(Tx::present && Tx::channel == 3 && Tx::width == DmaWidth::half);
static_assert(Rx::present && Rx::channel == 2 && dma_engines_distinct<Tx, Rx>());
static_assert(!dma_engines_distinct<DmaTxEngine<4>, DmaRxEngine<4>>());
static_assert(dma_engine_channel<NoDmaEngine>() == 0 && dma_engine_channel<Rx>() == 2);

volatile uint16_t data_register;
uint8_t ring8[64];

void engine_verbs() {
    static const uint8_t b8[4] = {1, 2, 3, 4};
    static const uint16_t b16[4] = {1, 2, 3, 4};
    static uint16_t in16[4];
    static uint8_t in8[4];
    Tx::arm(&data_register);
    Tx::arm(&data_register, DmaFlag::error, DmaPriority::medium);
    Tx::arm(&data_register, DmaPriority::very_high);
    (void)Tx::start(std::span<const uint8_t>(b8));
    (void)Tx::start(std::span<const uint16_t>(b16));
    (void)Tx::claim();
    (void)Tx::launch(std::span<const uint16_t>(b16));
    Tx::unclaim();
    (void)Tx::start_fixed(&b16[0], 8);
    (void)Tx::service();
    (void)Tx::service(Tx::block_flags());
    (void)Tx::complete();
    (void)Tx::busy();
    (void)Tx::in_flight();
    (void)Tx::progress();
    (void)Tx::abandon();
    (void)Tx::faults();
    Tx::clear_faults();
    Tx::stop();

    Rx::arm(&data_register);
    Rx::arm(&data_register, DmaPriority::high);
    (void)Rx::start(std::span<uint16_t>(in16));
    (void)Rx::start(std::span<uint8_t>(in8));
    (void)Rx::start_discard(&in16[0], 4);
    (void)Rx::take();
    (void)Rx::full();
    (void)Rx::capacity();
    (void)Rx::taken();
    (void)Rx::idle();
    (void)Rx::service();
    (void)Rx::abandon();
    (void)Rx::faults();
    Rx::stop();
    (void)Rx::arm_ring(&data_register, ring8, true, DmaPriority::very_high);
    (void)Rx::remaining();
    (void)Rx::laps();
    Rx::lap();

    static uint32_t w[8];
    static const uint32_t cell = 7;
    Copy::arm();
    Copy::arm(DmaPriority::high, false);
    (void)Copy::copy(w, w + 4, 4);
    (void)Copy::copy(reinterpret_cast<uint8_t*>(w), reinterpret_cast<const uint8_t*>(w + 4), 16);
    (void)Copy::fill(w, &cell, 8);
    (void)Copy::busy();
    (void)Copy::abandon();
    (void)Copy::isr();
    (void)Copy::faults();
    Copy8::arm(DmaPriority::low);
    (void)Copy8::copy(in8, b8, 4);
}

// The ring the receive engine feeds, read as util/ring.hpp's consumer.
static_assert(RingCounter<Rx>);
using View = HardwareRing<ring8, Rx>;

// ---- the transport on its engines -------------------------------------------------
// USART2, every part's: transmits on 7, receives on 6.
using Console = Uart<2, P, 64, 128, UartFormat{}, DmaTxEngine<7>, DmaRxEngine<6>>;
static_assert(ByteTransport<Console> && BulkSink<Console> && SpanSource<Console>);
static_assert(Console::has_tx_engine && Console::has_rx_engine);
// One engine at a time, and the ring ranked low by the knob.
using TxOnly = Uart<2, P, 64, 64, UartFormat{}, DmaTxEngine<7>>;
using RxOnly = Uart<2, P, 128, 64, UartFormat{}, NoDmaEngine, DmaRxEngine<6>, 0,
                    UartOptions{.rx_priority = DmaPriority::low}>;
// The transmit engine's block restart in SRAM (util/placement.hpp's InRam).
using RamConsole = Uart<2, P, 64, 128, UartFormat{}, DmaTxEngine<7>, DmaRxEngine<6>, 0, UartOptions{}, InRam>;

template <typename S>
void transport_verbs() {
    constexpr SysClock clock;
    (void)S::init(clock, 115200);
    (void)S::isr();
    (void)S::dma_isr();
    (void)S::harvest();
    (void)S::dma_faults();
    (void)S::write_byte('x');
    uint8_t b = 0;
    (void)S::read_byte(b);
    static const uint8_t bytes[40] = {};
    (void)S::write_bulk(bytes);
    (void)S::read_span();
    (void)S::consume(1);
    (void)S::tx_idle();
    (void)S::rx_pending();
    (void)S::rx_skips();
    (void)S::rx_overruns();
    S::clear_errors();
    S::rebase(24'000'000);
    (void)S::set_baud(48'000'000, 57600);
    S::release();
}

// USART4 and USART1 where the part offers their default column: USART4
// transmits on 1 and receives on 8, USART1 on 4 and 5.
template <uint8_t n>
void default_column_engines() {
    if constexpr (device::has_usart(n) && usart_remap_valid(n, 0)) {
        using Link = Uart<n, P, 256, 256, UartFormat{}, DmaTxEngine<usart_dma_tx_channel(n)>,
                          DmaRxEngine<usart_dma_rx_channel(n)>>;
        transport_verbs<Link>();
    }
}

void everything() {
    every_channel<1, 2, 3, 4, 5, 6, 7, 8>();
    engine_verbs();
    transport_verbs<Console>();
    transport_verbs<TxOnly>();
    transport_verbs<RxOnly>();
    transport_verbs<RamConsole>();
    default_column_engines<4>();
    default_column_engines<1>();
    (void)Dma::any_working();
    (void)Dma::is_open();
}

// Vector bindings, as an app writes them - the console's placed in SRAM as
// a program places one of its own plain functions (`.fast_text`), beside
// the InRam port's placed completion (`.fast_text.brio`) in this unit.
extern "C" BRIO_CH32_INTERRUPT [[gnu::section(".fast_text")]] void usart2_handler() { (void)Console::isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel6_handler() {
    (void)Console::dma_isr();
    (void)RamConsole::dma_isr();
}
extern "C" BRIO_CH32_INTERRUPT void dma1_channel7_handler() { (void)Console::dma_isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel8_handler() { (void)Copy8::isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel5_handler() { (void)Copy::isr(); }
