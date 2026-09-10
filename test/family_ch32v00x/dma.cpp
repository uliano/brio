// DMA family smoke TU: ch32v00x/dma.hpp instantiated for every one of
// the seven channels, the two engines at the three element widths, and
// the Uart's two engine slots in every combination - instantiation
// only, no main(), no hardware.
//
// The vocabulary's compile-time arithmetic is re-stated here (the
// vector per channel, the flag nibble, the width of an element, the
// one config the chapter forbids) because a wrong vector is a silent
// default_handler spin and a wrong nibble a flag read off the
// neighbour channel.
#include "ch32v00x/clock.hpp"
#include "ch32v00x/dma.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/usart.hpp"
#include "util/print.hpp"
#include "util/stream.hpp"

using namespace brio;

using P = Ch32v00xPlatform<>;
using SysClock = Clock<ClockSource::pll, 48'000'000>;

// ---- the vocabulary --------------------------------------------------------
static_assert(dma_channel_count == 7);
static_assert(dma_channel_irq(1) == Irq::dma1_channel1 && dma_channel_irq(7) == Irq::dma1_channel7);
static_assert(static_cast<uint8_t>(Irq::dma1_channel7) == static_cast<uint8_t>(Irq::dma1_channel1) + 6u,
              "the seven vectors are consecutive (table 3-2)");
static_assert(DmaChannel<1>::flag_shift == 0 && DmaChannel<7>::flag_shift == 24,
              "four flag bits per channel, channel 1 at the bottom");
static_assert(DmaFlag::all == 0xF);
static_assert(dma_width_of<uint8_t>() == DmaWidth::byte && dma_width_of<uint16_t>() == DmaWidth::half &&
              dma_width_of<uint32_t>() == DmaWidth::word);
static_assert(dma_channel_config_valid(DmaChannelConfig{}));
static_assert(!dma_channel_config_valid(DmaChannelConfig{.circular = true, .memory_to_memory = true}),
              "RM 8.2.1: circular mode is not for memory-to-memory");
static_assert(dma_channel_config_valid(DmaChannelConfig{.circular = true}));
static_assert(!dma_transfer_valid(DmaTransfer{}), "a transfer needs both ends and a count");
static_assert(dma_engines_distinct<NoDmaEngine, NoDmaEngine>() &&
              dma_engines_distinct<DmaTxEngine<4>, NoDmaEngine>() &&
              dma_engines_distinct<DmaTxEngine<4>, DmaRxEngine<5>>() &&
              !dma_engines_distinct<DmaTxEngine<4>, DmaRxEngine<4>>());

// ---- every channel's verbs -------------------------------------------------
template <uint8_t ch>
void channel_verbs(uint8_t* buf) {
    using C = DmaChannel<ch>;
    static_assert(C::number == ch);
    static_assert(C::irq() == dma_channel_irq(ch));
    Dma::open();
    (void)Dma::flags();
    (void)C::regs();
    (void)C::enabled();
    C::enable(false);
    (void)C::configure({.direction = DmaDirection::memory_to_peripheral,
                        .circular = true,
                        .memory_to_memory = false,
                        .peripheral_increment = false,
                        .memory_increment = true,
                        .peripheral_width = DmaWidth::half,
                        .memory_width = DmaWidth::word,
                        .priority = DmaPriority::very_high});
    (void)C::set_count(16);
    (void)C::count();
    C::set_peripheral(buf);
    C::set_memory(buf);
    (void)C::prepare(DmaTransfer{.peripheral = buf, .memory = buf + 8, .count = 8,
                                 .config = {.memory_to_memory = true, .peripheral_increment = true}});
    (void)C::load(DmaTransfer{.peripheral = buf, .memory = buf + 8, .count = 8,
                              .config = {.memory_to_memory = true, .peripheral_increment = true}});
    (void)C::flags();
    (void)C::flag(DmaFlag::complete | DmaFlag::error);
    C::clear(DmaFlag::all);
    C::arm(DmaFlag::complete | DmaFlag::half | DmaFlag::error, true);
    C::arm(DmaFlag::half, false);
    (void)C::isr();
    (void)C::progress(8);
    C::stop();
}

void all_channels() {
    static uint8_t buf[16];
    channel_verbs<1>(buf);
    channel_verbs<2>(buf);
    channel_verbs<3>(buf);
    channel_verbs<4>(buf);
    channel_verbs<5>(buf);
    channel_verbs<6>(buf);
    channel_verbs<7>(buf);
}

// ---- the engines at the three widths -------------------------------------
template <uint8_t ch, typename Elem>
void engines(volatile uint32_t* reg, Elem* run) {
    using Tx = DmaTxEngine<ch, Elem>;
    using Rx = DmaRxEngine<ch, Elem>;
    static_assert(Tx::present && Rx::present);
    static_assert(Tx::channel == ch && Rx::channel == ch);
    static_assert(Tx::width == dma_width_of<Elem>() && Rx::width == dma_width_of<Elem>());
    static_assert(std::same_as<typename Tx::element, Elem>);
    static_assert(Tx::flag_complete == DmaFlag::complete && Tx::flag_error == DmaFlag::error);

    Tx::arm(reg, DmaPriority::high);
    (void)Tx::start(run, 8);
    (void)Tx::start_fixed(run, 8);
    (void)Tx::busy();
    (void)Tx::in_flight();
    (void)Tx::progress();
    (void)Tx::service();
    (void)Tx::complete();
    (void)Tx::abandon();
    (void)Tx::faults();
    Tx::clear_faults();
    Tx::stop();

    Rx::arm(reg);
    (void)Rx::idle();
    (void)Rx::start(run, 8);
    (void)Rx::start_discard(run, 8);
    (void)Rx::take();
    (void)Rx::full();
    (void)Rx::capacity();
    (void)Rx::taken();
    (void)Rx::service();
    (void)Rx::abandon();
    (void)Rx::faults();
    Rx::clear_faults();
    Rx::stop();
}

void all_engines() {
    static uint8_t r8[8];
    static uint16_t r16[8];
    static uint32_t r32[8];
    engines<1, uint8_t>(&dma()->channel[0].CNTR, r8);
    engines<2, uint16_t>(&dma()->channel[0].CNTR, r16);
    engines<7, uint32_t>(&dma()->channel[0].CNTR, r32);
}

// ---- the Uart's slots, every combination ----------------------------------
using Plain = Uart<1, P>;
using TxOnly = Uart<1, P, 64, 128, DmaTxEngine<4>>;
using RxOnly = Uart<1, P, 64, 64, NoDmaEngine, DmaRxEngine<5>>;
using Both = Uart<1, P, 64, 128, DmaTxEngine<4>, DmaRxEngine<5>>;

static_assert(!Plain::has_tx_engine && !Plain::has_rx_engine);
static_assert(TxOnly::has_tx_engine && !TxOnly::has_rx_engine);
static_assert(!RxOnly::has_tx_engine && RxOnly::has_rx_engine);
static_assert(Both::has_tx_engine && Both::has_rx_engine);
static_assert(ByteTransport<Plain> && ByteTransport<TxOnly> && ByteTransport<RxOnly> && ByteTransport<Both>);

template <typename U>
void uart_verbs() {
    constexpr SysClock clock;
    constexpr U serial;
    (void)U::init(clock, 115200);
    (void)U::isr();
    (void)U::dma_isr();
    (void)U::harvest();
    (void)U::dma_faults();
    (void)U::write_byte(0x55);
    uint8_t b;
    (void)U::read_byte(b);
    (void)U::tx_idle();
    U::rebase(24'000'000);
    print(serial, "x=", hex(0x1234u), crlf);
}

void all_uarts() {
    uart_verbs<Plain>();
    uart_verbs<TxOnly>();
    uart_verbs<RxOnly>();
    uart_verbs<Both>();
}
