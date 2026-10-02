// DMA family smoke TU: ch32v00x/dma.hpp instantiated for every one of
// the seven channels, the two transfer engines at the three widest
// beats with every narrower beat a start() takes, the copy engine, and
// the Uart's two engine slots in every combination - instantiation
// only, no main(), no hardware.
//
// The vocabulary's compile-time arithmetic is re-stated here (the
// vector per channel, the flag nibble, the width of an element, the
// one config the chapter forbids, the configuration word and the
// alignment rule) because a wrong vector is a silent default_handler
// spin, a wrong nibble a flag read off the neighbour channel and a
// wrong word a block that moves the wrong way.
#include <span>

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
static_assert(dma_cfgr_of(DmaChannelConfig{}) == dma_cfgr_minc, "the default: read the peripheral, MINC");
static_assert(dma_cfgr_of({.direction = DmaDirection::memory_to_peripheral, .memory_increment = false,
                           .peripheral_width = DmaWidth::half, .memory_width = DmaWidth::word,
                           .priority = DmaPriority::very_high}) == (dma_cfgr_dir | (1u << 8) | (2u << 10) | (3u << 12)));
static_assert(dma_cfgr_beat<uint8_t>() == 0u && dma_cfgr_beat<uint16_t>() == 0x500u &&
              dma_cfgr_beat<uint32_t>() == 0xA00u, "PSIZE and MSIZE both follow the beat");
static_assert(dma_aligned(0x20000001u, DmaWidth::byte) && !dma_aligned(0x20000001u, DmaWidth::half) &&
              dma_aligned(0x20000002u, DmaWidth::half) && !dma_aligned(0x20000002u, DmaWidth::word),
              "8.3.6: a 16- or 32-bit access ignores the address's low bits");
static_assert(dma_beat_fits<uint8_t, uint16_t> && dma_beat_fits<uint16_t, uint16_t> &&
              !dma_beat_fits<uint32_t, uint16_t>);
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

// ---- the engines at the three widest beats --------------------------------
/// Every beat up to the engine's widest, through both spans.
template <typename Tx, typename Rx, typename T>
void beats(T* run) {
    if constexpr (sizeof(T) <= sizeof(typename Tx::element)) {
        (void)Tx::start(std::span<const T>(run, 8));
        (void)Tx::start(std::span<T>(run, 8));
        (void)Tx::start_fixed(run, 8);
        (void)Rx::start(std::span<T>(run, 8));
        (void)Rx::start_discard(run, 8);
    }
}

template <uint8_t ch, typename Elem>
void engines(volatile uint32_t* reg, uint8_t* r8, uint16_t* r16, uint32_t* r32) {
    using Tx = DmaTxEngine<ch, Elem>;
    using Rx = DmaRxEngine<ch, Elem>;
    static_assert(Tx::present && Rx::present);
    static_assert(Tx::channel == ch && Rx::channel == ch);
    static_assert(Tx::width == dma_width_of<Elem>() && Rx::width == dma_width_of<Elem>());
    static_assert(std::same_as<typename Tx::element, Elem>);
    static_assert(Tx::flag_complete == DmaFlag::complete && Tx::flag_error == DmaFlag::error);

    Tx::arm(reg, Tx::flag_error, DmaPriority::high);
    Tx::arm(reg, DmaPriority::medium);
    Tx::arm(reg);
    if (Tx::claim()) {
        (void)Tx::launch(std::span<const uint8_t>(r8, 8));
    }
    (void)Tx::claim();
    Tx::unclaim();
    beats<Tx, Rx, uint8_t>(r8);
    beats<Tx, Rx, uint16_t>(r16);
    beats<Tx, Rx, uint32_t>(r32);
    (void)Tx::service(Tx::block_flags());
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
    Rx::arm(reg, DmaPriority::low);
    Rx::arm(reg, Rx::flag_complete | Rx::flag_error, DmaPriority::very_high);
    (void)Rx::idle();
    (void)Rx::service(Rx::block_flags());
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
    engines<1, uint8_t>(&dma()->channel[0].CNTR, r8, r16, r32);
    engines<2, uint16_t>(&dma()->channel[0].CNTR, r8, r16, r32);
    engines<7, uint32_t>(&dma()->channel[0].CNTR, r8, r16, r32);
}

// ---- the copy engine, every beat --------------------------------------------
template <uint8_t ch, typename Elem>
void copier() {
    using C = DmaCopyEngine<ch, Elem>;
    static_assert(C::present && C::channel == ch && C::width == dma_width_of<Elem>());
    static uint8_t b8[16];
    static uint16_t b16[16];
    static uint32_t b32[16];
    static const uint32_t cell = 0x5A5A5A5Au;
    C::arm();
    C::arm(DmaPriority::high, false);
    (void)C::copy(b8, b8 + 8, 8u);
    (void)C::fill(b8, reinterpret_cast<const uint8_t*>(&cell), 16u);
    if constexpr (sizeof(Elem) >= 2u) {
        (void)C::copy(b16, b16 + 8, 8u);
        (void)C::fill(b16, reinterpret_cast<const uint16_t*>(&cell), 16u);
    }
    if constexpr (sizeof(Elem) >= 4u) {
        (void)C::copy(b32, b32 + 8, 8u);
        (void)C::fill(b32, &cell, 16u);
    }
    (void)C::busy();
    (void)C::isr();
    (void)C::faults();
    (void)C::abandon();
}

void all_copiers() {
    copier<1, uint32_t>();
    copier<4, uint16_t>();
    copier<7, uint8_t>();
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
static_assert(BulkSink<Both> && SpanSource<Both>);

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
