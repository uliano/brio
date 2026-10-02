// DMA family smoke TU: the block, a channel's every verb, the lines, a
// pacing timer, the sniffer, the two engines at every beat their Elem
// allows with the claim and the errors-only binding, the copy engine,
// and a UART with both slots filled.
#include <span>
#include <type_traits>

#include "rp2040/clock.hpp"
#include "rp2040/dma.hpp"
#include "rp2040/uart.hpp"

using namespace brio;

static_assert(dma_size_of<uint8_t>() == DmaSize::byte && dma_size_of<uint32_t>() == DmaSize::word);
static_assert(dma_size_bytes(DmaSize::half) == 2u);
static_assert(static_cast<uint8_t>(Dreq::uart1_rx) == 23u && static_cast<uint8_t>(Dreq::permanent) == 63u);
static_assert(dma_channel_config_valid({}, 0));
static_assert(!dma_channel_config_valid({.ring_bits = 16}, 0));
static_assert(!dma_channel_config_valid({.chain_to = 12}, 0));
static_assert(!dma_channel_config_valid({.chain_to = 3}, 3));   // a channel cannot chain to itself
static_assert(dma_channel_config_valid({.chain_to = 4}, 3));
static_assert((dma_ctrl_word({}, 5, true) & DMA_CH0_CTRL_TRIG_EN_BITS) != 0u);
static_assert(((dma_ctrl_word({}, 5, false) & DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS) >> DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) == 5u);
static_assert(DmaTimer<3>::dreq() == Dreq::timer3);
static_assert(DmaLine<1>::irq() == DMA_IRQ_1_IRQn);
// The binding word: EN, the request, CHAIN_TO at the channel itself (no
// chain), IRQ_QUIET for a binding that reports errors alone.
static_assert((dma_binding_word(5, Dreq::spi0_tx, false, DmaReport::blocks) & DMA_CH0_CTRL_TRIG_EN_BITS) != 0u);
static_assert(((dma_binding_word(5, Dreq::spi0_tx, false, DmaReport::blocks) & DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS) >>
               DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) == 5u);
static_assert(((dma_binding_word(5, Dreq::spi0_tx, false, DmaReport::blocks) & DMA_CH0_CTRL_TRIG_TREQ_SEL_BITS) >>
               DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) == 16u);
static_assert((dma_binding_word(5, Dreq::spi0_tx, false, DmaReport::blocks) & DMA_CH0_CTRL_TRIG_IRQ_QUIET_BITS) == 0u);
static_assert((dma_binding_word(5, Dreq::spi0_tx, true, DmaReport::errors) &
               (DMA_CH0_CTRL_TRIG_IRQ_QUIET_BITS | DMA_CH0_CTRL_TRIG_HIGH_PRIORITY_BITS)) ==
              (DMA_CH0_CTRL_TRIG_IRQ_QUIET_BITS | DMA_CH0_CTRL_TRIG_HIGH_PRIORITY_BITS));
static_assert(dma_beat_bits<uint16_t>() == (1u << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB));
static_assert(std::is_same_v<DmaTxEngine<4, uint16_t>::Report, DmaReport>);

using SysClock = Clock<ClockSource::pll, 125'000'000>;
constexpr UartPins u1{.tx = {4, PinFunction::uart}, .rx = {5, PinFunction::uart}};
using Streamed = Uart<1, u1, 512, 512, DmaTxEngine<2>, DmaRxEngine<3>>;
static_assert(Streamed::has_tx_engine && Streamed::has_rx_engine);
using HalfStreamed = Uart<1, u1, 64, 256, NoDmaEngine, DmaRxEngine<4, uint8_t, 1>>;
static_assert(!HalfStreamed::has_tx_engine && HalfStreamed::has_rx_engine);

uint8_t src[64];
uint8_t dst[64];
uint32_t words[16];

void dma_verbs() {
    (void)Dma::init();
    (void)Dma::released();
    (void)Dma::channels();
    (void)Dma::raw();
    Dma::clear_raw(0xFFFu);
    Dma::trigger(1u << 5);
    Dma::abort_raw(1u << 5);

    using C = DmaChannel<5>;
    (void)C::enabled();
    (void)C::busy();
    C::enable(false);
    (void)C::configure({.size = DmaSize::word, .treq = Dreq::spi0_tx, .chain_to = 6});
    (void)C::set_read(src);
    (void)C::set_write(dst);
    (void)C::set_count(64);
    (void)C::prepare(DmaTransfer{.read = src, .write = dst, .count = 64});
    C::trigger();
    C::null_trigger();
    (void)C::load(DmaTransfer{.read = words, .write = words, .count = 16, .config = {.size = DmaSize::word}});
    (void)C::count();
    (void)C::progress(64);
    (void)C::errors();
    C::clear_errors();
    (void)C::raised();
    C::clear_raised();
    C::route(0, true);
    (void)C::routed(0);
    (void)C::pending(1);
    C::clear_pending(1);
    (void)C::abort();
    C::stop();
    (void)C::debug_credits();
    (void)C::debug_reload();

    (void)DmaLine<0>::routed();
    (void)DmaLine<0>::pending();
    DmaLine<0>::clear(1u);
    DmaLine<1>::force(1u, true);
    DmaLine<1>::enable();
    DmaLine<1>::disable();

    DmaTimer<0>::set(1, 125);
    DmaTimer<0>::stop();

    DmaSniffer::start(5, {.calc = DmaSniffCalc::crc16}, 0xFFFFu);
    (void)DmaSniffer::result();
    DmaSniffer::stop();

    using Tx = DmaTxEngine<6, uint16_t>;
    Tx::arm(&SPI0->SSPDR, Dreq::spi0_tx);
    Tx::arm(&SPI0->SSPDR, Dreq::spi0_tx, true, Tx::Report::errors);
    (void)Tx::service();
    uint16_t halves[8] = {};
    (void)Tx::start(halves, 8);
    (void)Tx::start(src, 8);   // a byte run on a half-word engine
    (void)Tx::start(std::span<const uint16_t>(halves));
    (void)Tx::start(std::span<const uint8_t>(src));
    (void)Tx::start_fixed(halves, 8);
    (void)Tx::start_fixed(src, 8);
    if (Tx::claim()) {
        (void)Tx::launch(std::span<const uint16_t>(halves));
    }
    if (Tx::claim()) {
        (void)Tx::launch_fixed(src, 4);
    }
    if (Tx::claim()) {
        Tx::unclaim();
    }
    (void)Tx::busy();
    (void)Tx::in_flight();
    (void)Tx::progress();
    (void)Tx::complete();
    (void)Tx::abandon();
    (void)Tx::faults();
    Tx::clear_faults();
    Tx::stop();

    using Rx = DmaRxEngine<7, uint16_t, 1>;
    Rx::arm(&SPI0->SSPDR, Dreq::spi0_rx);
    (void)Rx::service();
    (void)Rx::idle();
    (void)Rx::start(halves, 8);
    (void)Rx::start(dst, 8);
    (void)Rx::start(std::span<uint16_t>(halves));
    (void)Rx::start(std::span<uint8_t>(dst));
    (void)Rx::start_discard(halves, 8);
    (void)Rx::start_discard(dst, 8);
    (void)Rx::take();
    (void)Rx::full();
    (void)Rx::capacity();
    (void)Rx::taken();
    (void)Rx::abandon();
    (void)Rx::faults();
    Rx::stop();

    using Word = DmaTxEngine<9, uint32_t>;   // a paced table into a word register
    Word::arm(&words[0], DmaTimer<1>::dreq());
    (void)Word::start(std::span<const uint32_t>(words));
    (void)Word::start(halves, 8);
    Word::stop();

    using Copier = DmaCopyEngine<8>;
    Copier::arm();
    Copier::arm(DmaReport::errors, true);
    (void)Copier::copy(words, words + 8, 8u);
    (void)Copier::copy(halves, halves + 4, 4u);
    (void)Copier::copy(dst, src, 64u);
    (void)Copier::fill(words, &words[15], 15u);
    (void)Copier::fill(dst, &src[0], 64u);
    (void)Copier::busy();
    (void)Copier::errors();
    Copier::clear_errors();
    (void)Copier::remaining();
    (void)Copier::service();
    Copier::stop();
    using CopierOn1 = DmaCopyEngine<11, 1>;
    CopierOn1::arm();
    (void)CopierOn1::service();

    constexpr SysClock clock;
    (void)Streamed::init(clock, 3'000'000);
    (void)Streamed::dma_isr();
    (void)Streamed::harvest();
    (void)Streamed::dma_faults();
    (void)Streamed::write_byte(1);
    (void)Streamed::write_bulk(src);
    (void)Streamed::read_bulk(dst);
    (void)Streamed::tx_idle();
    (void)Streamed::isr();
    Streamed::release();
    (void)HalfStreamed::init(clock, 115200);
    (void)HalfStreamed::harvest();
}
