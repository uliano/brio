// DMA family smoke TU: the block, a channel's every verb, the lines, a
// pacing timer, the sniffer, the two engines, and a UART with both
// slots filled.
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
    (void)Tx::service();
    uint16_t halves[8] = {};
    (void)Tx::start(halves, 8);
    (void)Tx::start_fixed(halves, 8);
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
    (void)Rx::start_discard(halves, 8);
    (void)Rx::take();
    (void)Rx::full();
    (void)Rx::capacity();
    (void)Rx::taken();
    (void)Rx::abandon();
    (void)Rx::faults();
    Rx::stop();

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
