// DMA family smoke TU: the sixteen channels, the four interrupt lines,
// the count modes in TRANS_COUNT's top nibble, the four address steps,
// the pacing timers, the sniffer, the read-only security and MPU views,
// and the two engines - every verb instantiated, and the CTRL and
// TRANS_COUNT words computed at COMPILE TIME against the fields the
// device header names, which is where this chip differs from its
// predecessor by two bits' worth of shifting.
//
// Compiled for both packages and both architectures: nothing of this
// chapter is bonded to a pad, so the package changes nothing here, and
// what the sweep proves is that the controller knows no instruction set
// (the interrupt line goes to `Irq`, which is an NVIC under one
// architecture and Hazard3's own under the other).
#include "rp2350/dma.hpp"
#include "rp2350/uart.hpp"

using namespace brio;

// ---- the counts ------------------------------------------------------------

static_assert(dma_channel_count == 16u);
static_assert(dma_line_count == 4u);
static_assert(dma_timer_count == 4u);
static_assert(dma_count_max == 0x0FFFFFFFu);
static_assert(DmaMpu::regions == 8u);

// ---- the widths and the steps ----------------------------------------------

static_assert(dma_size_of<uint8_t>() == DmaSize::byte &&
              dma_size_of<uint16_t>() == DmaSize::half &&
              dma_size_of<uint32_t>() == DmaSize::word);
static_assert(dma_size_bytes(DmaSize::byte) == 1u && dma_size_bytes(DmaSize::half) == 2u &&
              dma_size_bytes(DmaSize::word) == 4u);

// The INCR/REV pair as four behaviours: fixed, forward, backward, and
// forward by twice the transfer size.
static_assert(!dma_step_increments(DmaStep::fixed) && !dma_step_reversed(DmaStep::fixed));
static_assert(dma_step_increments(DmaStep::forward) && !dma_step_reversed(DmaStep::forward));
static_assert(dma_step_increments(DmaStep::backward) && dma_step_reversed(DmaStep::backward));
static_assert(!dma_step_increments(DmaStep::forward_by_two) &&
              dma_step_reversed(DmaStep::forward_by_two));

// ---- the count modes -------------------------------------------------------

static_assert(static_cast<uint8_t>(DmaCountMode::normal) == 0u &&
              static_cast<uint8_t>(DmaCountMode::trigger_self) == 1u &&
              static_cast<uint8_t>(DmaCountMode::endless) == 0xFu);
static_assert(dma_count_mode_valid(DmaCountMode::normal) &&
              dma_count_mode_valid(DmaCountMode::trigger_self) &&
              dma_count_mode_valid(DmaCountMode::endless));
static_assert(!dma_count_mode_valid(static_cast<DmaCountMode>(2)));
static_assert(!dma_count_mode_valid(static_cast<DmaCountMode>(0xE)));

// The mode lives in the top nibble and the count in the 28 bits below.
static_assert(dma_count_word(1024, DmaCountMode::normal) == 1024u);
static_assert(dma_count_word(1024, DmaCountMode::trigger_self) == (0x10000000u | 1024u));
static_assert(dma_count_word(1, DmaCountMode::endless) == (0xF0000000u | 1u));

// ---- the CTRL word ---------------------------------------------------------

// A plain memory-to-memory word transfer with no chain: the default
// config's own word, with EN and the channel's index as CHAIN_TO.
constexpr uint32_t m2m = dma_ctrl_word({.size = DmaSize::word}, 4, true);
static_assert((m2m & DMA_CH0_CTRL_TRIG_EN_BITS) != 0u);
static_assert(((m2m & DMA_CH0_CTRL_TRIG_DATA_SIZE_BITS) >> DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) == 2u);
static_assert(((m2m & DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS) >> DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) == 4u);
static_assert(((m2m & DMA_CH0_CTRL_TRIG_TREQ_SEL_BITS) >> DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) == 63u);
static_assert((m2m & DMA_CH0_CTRL_TRIG_INCR_READ_BITS) != 0u &&
              (m2m & DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS) != 0u);
static_assert((m2m & DMA_CH0_CTRL_TRIG_INCR_READ_REV_BITS) == 0u &&
              (m2m & DMA_CH0_CTRL_TRIG_INCR_WRITE_REV_BITS) == 0u);

// Every step in the word, both sides.
constexpr uint32_t stepped = dma_ctrl_word(
    {.read_step = DmaStep::backward, .write_step = DmaStep::forward_by_two}, 0, false);
static_assert((stepped & DMA_CH0_CTRL_TRIG_INCR_READ_BITS) != 0u &&
              (stepped & DMA_CH0_CTRL_TRIG_INCR_READ_REV_BITS) != 0u);
static_assert((stepped & DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS) == 0u &&
              (stepped & DMA_CH0_CTRL_TRIG_INCR_WRITE_REV_BITS) != 0u);
static_assert((stepped & DMA_CH0_CTRL_TRIG_EN_BITS) == 0u);

// A peripheral transmit: a fixed write address, a peripheral request, the
// ring on the read side, the sniffer watching, quiet interrupts.
constexpr uint32_t to_uart = dma_ctrl_word({.size = DmaSize::byte,
                                            .read_step = DmaStep::forward,
                                            .write_step = DmaStep::fixed,
                                            .ring_bits = 6,
                                            .treq = Dreq::uart1_tx,
                                            .chain_to = 9,
                                            .high_priority = true,
                                            .byte_swap = true,
                                            .sniff = true,
                                            .irq_quiet = true},
                                           3, true);
static_assert((to_uart & DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS) == 0u);
static_assert(((to_uart & DMA_CH0_CTRL_TRIG_RING_SIZE_BITS) >> DMA_CH0_CTRL_TRIG_RING_SIZE_LSB) == 6u);
static_assert((to_uart & DMA_CH0_CTRL_TRIG_RING_SEL_BITS) == 0u);
static_assert(((to_uart & DMA_CH0_CTRL_TRIG_TREQ_SEL_BITS) >> DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) == 30u);
static_assert(((to_uart & DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS) >> DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) == 9u);
static_assert((to_uart & (DMA_CH0_CTRL_TRIG_HIGH_PRIORITY_BITS | DMA_CH0_CTRL_TRIG_BSWAP_BITS |
                          DMA_CH0_CTRL_TRIG_SNIFF_EN_BITS | DMA_CH0_CTRL_TRIG_IRQ_QUIET_BITS)) ==
              (DMA_CH0_CTRL_TRIG_HIGH_PRIORITY_BITS | DMA_CH0_CTRL_TRIG_BSWAP_BITS |
               DMA_CH0_CTRL_TRIG_SNIFF_EN_BITS | DMA_CH0_CTRL_TRIG_IRQ_QUIET_BITS));

// The ring on the write side instead.
static_assert((dma_ctrl_word({.ring_bits = 4, .ring_on_write = true}, 0, false) &
               DMA_CH0_CTRL_TRIG_RING_SEL_BITS) != 0u);

// CHAIN_TO is FOUR bits here, and reaches the sixteenth channel.
static_assert(((dma_ctrl_word({.chain_to = 15}, 0, false) & DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS) >>
               DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) == 15u);

// ---- what a config and a transfer refuse -----------------------------------

static_assert(dma_channel_config_valid({}, 0));
static_assert(dma_channel_config_valid({.ring_bits = 15}, 0));
static_assert(!dma_channel_config_valid({.ring_bits = 16}, 0));
static_assert(!dma_channel_config_valid({.chain_to = 4}, 4));    // a channel cannot chain to itself
static_assert(!dma_channel_config_valid({.chain_to = 16}, 0));   // there is no seventeenth
static_assert(dma_channel_config_valid({.chain_to = 15}, 0));

alignas(4) inline uint8_t bytes[64];
inline volatile uint32_t sink;

/// A register named by reference is READ through this: `(void)` on a
/// volatile lvalue accesses nothing and -Werror says so.
uint32_t peek(volatile uint32_t& reg) { return reg; }

void dma_transfer_rules() {
    // Both ends present, a non-zero count within the 28 bits, a legal
    // mode and config, and each address aligned to its own width.
    const DmaTransfer good{.read = bytes, .write = &sink, .count = 16,
                           .config = {.write_step = DmaStep::fixed}};
    (void)dma_transfer_valid(good, 0);
    (void)dma_transfer_valid({.read = bytes, .write = &sink, .count = 0}, 0);
    (void)dma_transfer_valid({.read = bytes, .write = &sink, .count = dma_count_max + 1u}, 0);
    (void)dma_transfer_valid({.read = bytes, .write = &sink, .count = 4,
                              .mode = static_cast<DmaCountMode>(7)}, 0);
    (void)dma_transfer_valid({.read = bytes + 1, .write = &sink, .count = 4,
                              .config = {.size = DmaSize::word}}, 0);
    (void)dma_aligned(bytes, DmaSize::word);
}

// ---- the block -------------------------------------------------------------

void dma_block_verbs() {
    (void)Dma::init();
    (void)Dma::released();
    (void)Dma::channels();
    (void)Dma::raw();
    Dma::clear_raw(0xFFFFu);
    Dma::trigger(1u << 5);
    Dma::abort_raw(1u << 5);
    (void)Dma::abort_pending();
    (void)Dma::abort((1u << 5) | (1u << 6));
    const DmaFifoLevels levels = Dma::fifo_levels();
    (void)levels.read_address;
    (void)levels.write_address;
    (void)levels.transfer_data;
    (void)Dma::channel_security(0);
    (void)Dma::channel_security_locked(15);
    (void)Dma::line_security(3);
    (void)Dma::sniffer_security();
    (void)Dma::timer_security(2);
    (void)peek(Dma::channel_ctrl(7));
    (void)peek(Dma::enables(1));
    (void)peek(Dma::force_bits(2));
    (void)peek(Dma::status(3));
}

// ---- the four lines --------------------------------------------------------

static_assert(DmaLine<0>::irq() == DMA_IRQ_0_IRQn && DmaLine<1>::irq() == DMA_IRQ_1_IRQn &&
              DmaLine<2>::irq() == DMA_IRQ_2_IRQn && DmaLine<3>::irq() == DMA_IRQ_3_IRQn);

template <uint8_t line>
void dma_line_verbs() {
    (void)DmaLine<line>::irq();
    (void)DmaLine<line>::routed();
    (void)DmaLine<line>::pending();
    DmaLine<line>::clear(0xFFFFu);
    DmaLine<line>::force(1u << 2, true);
    DmaLine<line>::force(1u << 2, false);
    DmaLine<line>::enable();
    DmaLine<line>::disable();
}
void dma_all_lines() {
    dma_line_verbs<0>();
    dma_line_verbs<1>();
    dma_line_verbs<2>();
    dma_line_verbs<3>();
}

// ---- a channel -------------------------------------------------------------

static_assert(DmaChannel<0>::number == 0u && DmaChannel<0>::bit == 1u);
static_assert(DmaChannel<15>::number == 15u && DmaChannel<15>::bit == 0x8000u);

template <uint8_t ch>
void dma_channel_verbs() {
    using C = DmaChannel<ch>;
    (void)peek(C::read_addr());
    (void)peek(C::write_addr());
    (void)peek(C::trans_count());
    (void)peek(C::ctrl_trig());
    (void)peek(C::ctrl());
    (void)peek(C::trans_count_trig());
    (void)peek(C::write_addr_trig());
    (void)peek(C::read_addr_trig());
    (void)C::enabled();
    (void)C::busy();
    C::enable(false);
    (void)C::configure({.size = DmaSize::half, .read_step = DmaStep::backward});
    (void)C::set_read(bytes);
    (void)C::set_write(&sink);
    (void)C::set_count(8);
    (void)C::set_count(8, DmaCountMode::trigger_self);
    (void)C::set_count(1, DmaCountMode::endless);
    (void)C::prepare({.read = bytes, .write = &sink, .count = 8,
                      .config = {.write_step = DmaStep::fixed}});
    (void)C::load({.read = bytes, .write = &sink, .count = 8, .mode = DmaCountMode::trigger_self,
                   .config = {.write_step = DmaStep::fixed}});
    C::trigger();
    C::null_trigger();
    (void)C::count();
    (void)C::mode();
    const DmaProgress p = C::progress(8);
    (void)p.remaining;
    (void)p.done;
    (void)C::errors();
    C::clear_errors();
    (void)C::raised();
    C::clear_raised();
    C::route(0, true);
    C::route(3, false);
    (void)C::routed(1);
    (void)C::pending(2);
    C::clear_pending(2);
    (void)C::abort();
    C::stop();
    (void)C::debug_credits();
    C::clear_credits();
    (void)C::debug_reload();
    (void)C::security();
    (void)C::security_locked();
}
void dma_channels() {
    dma_channel_verbs<0>();
    dma_channel_verbs<7>();
    dma_channel_verbs<15>();   // the four channels the RP2040 has not
}

// ---- the pacing timers, the sniffer, the MPU -------------------------------

static_assert(DmaTimer<0>::dreq() == Dreq::timer0 && DmaTimer<1>::dreq() == Dreq::timer1 &&
              DmaTimer<2>::dreq() == Dreq::timer2 && DmaTimer<3>::dreq() == Dreq::timer3);
static_assert(static_cast<uint8_t>(Dreq::timer0) == 59u &&
              static_cast<uint8_t>(Dreq::permanent) == 63u);

void dma_timer_verbs() {
    DmaTimer<0>::set(1, 150);
    (void)DmaTimer<0>::numerator();
    (void)DmaTimer<0>::denominator();
    (void)DmaTimer<0>::security();
    DmaTimer<0>::stop();
    DmaTimer<3>::set(0, 1);
    DmaTimer<3>::stop();
}

void dma_sniffer_verbs() {
    DmaSniffer::start(4, {.calc = DmaSniffCalc::crc32}, 0xFFFFFFFFu);
    DmaSniffer::start(4, {.calc = DmaSniffCalc::crc32_reversed});
    DmaSniffer::start(4, {.calc = DmaSniffCalc::crc16}, 0xFFFFu);
    DmaSniffer::start(4, {.calc = DmaSniffCalc::crc16_reversed});
    DmaSniffer::start(4, {.calc = DmaSniffCalc::even_parity});
    DmaSniffer::start(4, {.calc = DmaSniffCalc::sum, .byte_swap = true, .out_reverse = true,
                          .out_invert = true});
    (void)DmaSniffer::result();
    (void)DmaSniffer::security();
    DmaSniffer::stop();
}

void dma_mpu_verbs() {
    (void)DmaMpu::global_level();
    (void)DmaMpu::hides_addresses();
    for (uint8_t i = 0; i < DmaMpu::regions; ++i) {
        const DmaMpuRegion r = DmaMpu::region(i);
        (void)r.base;
        (void)r.limit;
        (void)r.level;
        (void)r.enabled;
    }
}

// The four levels, ordered as 12.6.6 orders them.
static_assert(static_cast<uint8_t>(DmaSecurity::nsu) < static_cast<uint8_t>(DmaSecurity::nsp) &&
              static_cast<uint8_t>(DmaSecurity::nsp) < static_cast<uint8_t>(DmaSecurity::su) &&
              static_cast<uint8_t>(DmaSecurity::su) < static_cast<uint8_t>(DmaSecurity::sp));

// ---- the engines -----------------------------------------------------------

using Tx = DmaTxEngine<0>;
using Rx = DmaRxEngine<1>;
using WideTx = DmaTxEngine<2, uint16_t, 1>;
using WordRx = DmaRxEngine<3, uint32_t, 3>;   // a line the RP2040 has not

static_assert(Tx::present && Rx::present && !NoDmaEngine::present);
static_assert(Tx::channel == 0u && Rx::channel == 1u);
static_assert(Tx::irq_line == 0u && WideTx::irq_line == 1u && WordRx::irq_line == 3u);
static_assert(Tx::size == DmaSize::byte && WideTx::size == DmaSize::half &&
              WordRx::size == DmaSize::word);
static_assert(Tx::flag_complete != Tx::flag_error && Tx::flag_half == 0u);

void dma_engine_verbs() {
    Tx::arm(&sink, Dreq::uart0_tx);
    (void)Tx::service();
    (void)Tx::start(bytes, 16);
    (void)Tx::start_fixed(bytes, 16);
    (void)Tx::complete();
    (void)Tx::busy();
    (void)Tx::in_flight();
    (void)Tx::progress();
    (void)Tx::abandon();
    (void)Tx::faults();
    Tx::clear_faults();
    Tx::stop();

    Rx::arm(&sink, Dreq::uart0_rx, true);
    (void)Rx::service();
    (void)Rx::idle();
    (void)Rx::start(bytes, 16);
    (void)Rx::start_discard(bytes, 16);
    (void)Rx::take();
    (void)Rx::full();
    (void)Rx::capacity();
    (void)Rx::taken();
    (void)Rx::abandon();
    (void)Rx::faults();
    Rx::clear_faults();
    Rx::stop();

    uint16_t halves[4] = {};
    WideTx::arm(&sink, Dreq::spi0_tx);
    (void)WideTx::start(halves, 4);
    WideTx::stop();
    alignas(4) uint32_t words[4] = {};
    WordRx::arm(&sink, Dreq::spi0_rx);
    (void)WordRx::start(words, 4);
    WordRx::stop();
}

// ---- the engines in a transport's slots ------------------------------------
//
// The UART's two slots filled, which is the chapter's one user today: the
// transport asks the engine only for `present`, `channel` and the verbs
// above, and the chip traits check the two channels differ.

constexpr UartPins instrument_pins{.tx = {4, PinFunction::uart}, .rx = {5, PinFunction::uart}};
using Link = Uart<1, instrument_pins, 1024, 1024, DmaTxEngine<2>, DmaRxEngine<3>>;

static_assert(Link::has_tx_engine && Link::has_rx_engine);
static_assert(uart_engines_distinct<DmaTxEngine<2>, DmaRxEngine<3>>());
static_assert(!uart_engines_distinct<DmaTxEngine<2>, DmaRxEngine<2>>());
static_assert(uart_engines_distinct<NoDmaEngine, DmaRxEngine<3>>());

void dma_transport_slots() {
    constexpr Clock<ClockSource::pll, 150'000'000UL> clock;
    (void)Link::init(clock, 3'000'000);
    (void)Link::dma_isr();
    (void)Link::harvest();
    (void)Link::dma_faults();
    Link::release();
}
