// mcu: ch32v303cb ch32v303rb ch32v303rc ch32v303vc
// DMA2 family smoke TU: the second controller of the CH32V303 (RM 11.2.3,
// tables 11-3 and 11-4 for its request map, table 11-8 and 11.3.7..11.3.12
// for its registers), every one of its eleven channels, its extended flag
// pair, its vectors, its rows of the request table folded through each
// part, and the four engines on it.
//
// THE CONTROLLER IS EVERY CH32V303'S, THE ROWS ARE NOT. The CB and the RB
// have DMA2 and nothing that raises most of its requests - no TIM5..TIM10,
// no UART4..8, no SPI3, no SDIO host - but they do have the two DACs and,
// by the table's note, ADC2's row; the RC and the VC have all of it. So
// this TU asks the table where the part decides, and names a literal row
// only where every CH32V303 raises it.
#include "ch32vx03/dma.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/usart.hpp"
#include "util/block_stream.hpp"

using namespace brio;

using P = Ch32vx03Platform<>;
using SysClock = Clock<ClockSource::pll, 48'000'000>;

// ---- the controller and its channels --------------------------------------
static_assert(device::dma_controller_count == 2u);
static_assert(Dma<2>::channel_count == 11 && dma_channels_of(2) == 11);
static_assert(Dma<1>::channel_count == 7);
static_assert(Dma<2>::has_extended_flags && !Dma<1>::has_extended_flags);
static_assert(Dma<2>::gate == rcc_hb_dma2 && Dma<1>::gate == rcc_hb_dma1);
static_assert(dma_base_for(2) == 0x4002'0400UL);
// Channels 1..7 at twenty bytes from 0x08, 8..11 at sixteen from 0x90
// (table 11-8, 11.3.7..11.3.10).
static_assert(dma_channel_address(2, 1) == 0x4002'0408UL);
static_assert(dma_channel_address(2, 5) == 0x4002'0458UL);
static_assert(dma_channel_address(2, 7) == 0x4002'0480UL);
static_assert(dma_channel_address(2, 8) == 0x4002'0490UL);
static_assert(dma_channel_address(2, 9) == 0x4002'04A0UL);
static_assert(dma_channel_address(2, 10) == 0x4002'04B0UL);
static_assert(dma_channel_address(2, 11) == 0x4002'04C0UL);
// The flags: channels 1..7 in INTFR at 4 x (ch - 1), 8..11 in the
// extended pair at 4 x (ch - 8).
static_assert(!DmaChannel<2, 7>::extended && DmaChannel<2, 7>::flag_shift == 24);
static_assert(DmaChannel<2, 8>::extended && DmaChannel<2, 8>::flag_shift == 0);
static_assert(DmaChannel<2, 11>::extended && DmaChannel<2, 11>::flag_shift == 12);
static_assert(DmaChannel<2, 4>::slot == DmaSlot{2, 4});
// DMA2 has "no restriction" (11.2.3's note (3)); DMA1 of this class does.
static_assert(!DmaChannel<2, 1>::bounded_to_64k && DmaChannel<1, 1>::bounded_to_64k);

// ---- the vectors: 72..76 and 98..103 (the crt's table) --------------------
static_assert(dma_channel_irq(2, 1) == Irq::dma2_channel1);
static_assert(static_cast<uint8_t>(dma_channel_irq(2, 1)) == 72);
static_assert(static_cast<uint8_t>(dma_channel_irq(2, 5)) == 76);
static_assert(static_cast<uint8_t>(dma_channel_irq(2, 6)) == 98);
static_assert(static_cast<uint8_t>(dma_channel_irq(2, 11)) == 103);
static_assert(DmaChannel<2, 9>::irq() == Irq::dma2_channel9);

// ---- the request table: DMA2's rows (tables 11-3 and 11-4) ----------------
constexpr DmaSlot on2(bool present, uint8_t ch) { return present ? DmaSlot{2, ch} : DmaSlot{}; }
constexpr bool tim(uint8_t n) { return dma_timer_present(n); }

// Every CH32V303 has the two DACs and a second converter.
static_assert(dma_request_channel(DmaRequest::dac1) == DmaSlot{2, 3});
static_assert(dma_request_channel(DmaRequest::dac2) == DmaSlot{2, 4});
static_assert(dma_request_channel(DmaRequest::adc2) == DmaSlot{2, 5});
static_assert(DmaRequestOf<DmaRequest::dac1>::controller == 2);

// TIM5 on DMA2 where the part has it (the RC and the VC).
static_assert(tim5_on_dma2() == device::has_tim5 && !tim5_on_dma1());
static_assert(dma_request_channel(DmaRequest::tim5_ch4) == on2(device::has_tim5, 1));
static_assert(dma_request_channel(DmaRequest::tim5_trig) == on2(device::has_tim5, 1));
static_assert(dma_request_channel(DmaRequest::tim5_ch3) == on2(device::has_tim5, 2));
static_assert(dma_request_channel(DmaRequest::tim5_up) == on2(device::has_tim5, 2));
static_assert(dma_request_channel(DmaRequest::tim5_ch2) == on2(device::has_tim5, 4));
static_assert(dma_request_channel(DmaRequest::tim5_ch1) == on2(device::has_tim5, 5));

static_assert(dma_request_channel(DmaRequest::tim6_up) == on2(tim(6), 3));
static_assert(dma_request_channel(DmaRequest::tim7_up) == on2(tim(7), 4));

static_assert(dma_request_channel(DmaRequest::tim8_ch3) == on2(tim(8), 1));
static_assert(dma_request_channel(DmaRequest::tim8_up) == on2(tim(8), 1));
static_assert(dma_request_channel(DmaRequest::tim8_ch4) == on2(tim(8), 2));
static_assert(dma_request_channel(DmaRequest::tim8_trig) == on2(tim(8), 2));
static_assert(dma_request_channel(DmaRequest::tim8_com) == on2(tim(8), 2));
static_assert(dma_request_channel(DmaRequest::tim8_ch1) == on2(tim(8), 3));
static_assert(dma_request_channel(DmaRequest::tim8_ch2) == on2(tim(8), 5));

static_assert(dma_request_channel(DmaRequest::tim9_up) == on2(tim(9), 6));
static_assert(dma_request_channel(DmaRequest::tim9_ch1) == on2(tim(9), 7));
static_assert(dma_request_channel(DmaRequest::tim9_ch4) == on2(tim(9), 8));
static_assert(dma_request_channel(DmaRequest::tim9_ch2) == on2(tim(9), 9));
static_assert(dma_request_channel(DmaRequest::tim9_trig) == on2(tim(9), 10));
static_assert(dma_request_channel(DmaRequest::tim9_com) == on2(tim(9), 10));
static_assert(dma_request_channel(DmaRequest::tim9_ch3) == on2(tim(9), 11));

static_assert(dma_request_channel(DmaRequest::tim10_ch4) == on2(tim(10), 6));
static_assert(dma_request_channel(DmaRequest::tim10_trig) == on2(tim(10), 7));
static_assert(dma_request_channel(DmaRequest::tim10_com) == on2(tim(10), 7));
static_assert(dma_request_channel(DmaRequest::tim10_ch1) == on2(tim(10), 8));
static_assert(dma_request_channel(DmaRequest::tim10_ch3) == on2(tim(10), 9));
static_assert(dma_request_channel(DmaRequest::tim10_ch2) == on2(tim(10), 10));
static_assert(dma_request_channel(DmaRequest::tim10_up) == on2(tim(10), 11));

static_assert(dma_request_channel(DmaRequest::uart4_rx) == on2(device::has_usart(4), 3));
static_assert(dma_request_channel(DmaRequest::uart4_tx) == on2(device::has_usart(4), 5));
static_assert(dma_request_channel(DmaRequest::uart5_rx) == on2(device::has_usart(5), 2));
static_assert(dma_request_channel(DmaRequest::uart5_tx) == on2(device::has_usart(5), 4));
static_assert(dma_request_channel(DmaRequest::uart6_tx) == on2(device::has_usart(6), 6));
static_assert(dma_request_channel(DmaRequest::uart6_rx) == on2(device::has_usart(6), 7));
static_assert(dma_request_channel(DmaRequest::uart7_tx) == on2(device::has_usart(7), 8));
static_assert(dma_request_channel(DmaRequest::uart7_rx) == on2(device::has_usart(7), 9));
static_assert(dma_request_channel(DmaRequest::uart8_tx) == on2(device::has_usart(8), 10));
static_assert(dma_request_channel(DmaRequest::uart8_rx) == on2(device::has_usart(8), 11));

inline constexpr bool spi3 = (device::spi_instances & (1U << 3)) != 0u;
static_assert(dma_request_channel(DmaRequest::spi3_rx) == on2(spi3, 1));
static_assert(dma_request_channel(DmaRequest::spi3_tx) == on2(spi3, 2));
static_assert(dma_request_channel(DmaRequest::sdio) == on2(device::has_sdio, 4));

// And DMA1 on this class carries no UART4 row and no eighth channel
// (table 11-2).
static_assert(dma_request_channel(DmaRequest::uart4_tx).controller != 1u);
static_assert(!dma_channel_exists(1, 8));

// ---- the four engines on the second controller ----------------------------
using DacRow = DmaRequestOf<DmaRequest::dac1>;
using Player = DmaLoopEngine<DacRow::controller, DacRow::channel, uint16_t>;
using Source = DmaPingPongEngine<2, 5, uint16_t>;
using Tx = DmaTxEngine<2, 4, uint16_t>;
using Rx = DmaRxEngine<2, 9>;
static_assert(BlockPlayer<Player> && BlockSource<Source>);
static_assert(Player::slot == DmaSlot{2, 3} && Player::controller == 2);
static_assert(dma_engine_slot<Rx>() == DmaSlot{2, 9});
static_assert(dma_engines_distinct<DmaTxEngine<1, 5>, DmaRxEngine<2, 5>>());
static_assert(!dma_engines_distinct<DmaTxEngine<2, 5>, DmaRxEngine<2, 5>>());

alignas(4) uint16_t table[8];
alignas(4) volatile uint16_t half_a[8];
alignas(4) volatile uint16_t half_b[8];
alignas(4) uint8_t src[16];
alignas(4) uint8_t dst[16];
uint32_t peripheral_register;

template <uint8_t ch>
void channel_verbs() {
    using C = DmaChannel<2, ch>;
    C::stop();
    (void)C::controller;
    (void)C::number;
    (void)C::extended;
    (void)C::irq();
    (void)C::regs();
    (void)C::enabled();
    (void)C::configure(DmaChannelConfig{.memory_to_memory = true,
                                        .peripheral_increment = true,
                                        .peripheral_width = DmaWidth::half,
                                        .memory_width = DmaWidth::half});
    (void)C::configuration();
    (void)C::set_count(4);
    (void)C::remaining();
    (void)C::set_peripheral(src);
    (void)C::set_memory(dst);
    (void)C::accepts(DmaTransfer{.peripheral = src, .memory = dst, .count = 16,
                                 .config = {.memory_to_memory = true}});
    (void)C::prepare(DmaTransfer{.peripheral = src, .memory = dst, .count = 16,
                                 .config = {.memory_to_memory = true,
                                            .peripheral_increment = true}});
    (void)C::trigger();
    (void)C::load(DmaTransfer{.peripheral = src, .memory = dst, .count = 16,
                              .config = {.memory_to_memory = true,
                                         .peripheral_increment = true}});
    (void)C::flags();
    (void)C::flag(DmaFlag::complete);
    C::clear(DmaFlag::all);
    C::arm(DmaFlag::complete | DmaFlag::half | DmaFlag::error, true);
    (void)C::armed();
    (void)C::isr();
    (void)C::progress(16);
    C::enable(true);
    C::enable(false);
    C::stop();
}

void dma2_verbs() {
    Dma<2>::open();
    (void)Dma<2>::opened();
    (void)Dma<2>::regs();
    (void)Dma<2>::flags();
    Dma<2>::clear(0xFFFFFFFFUL);
    (void)Dma<2>::extended_flags();
    Dma<2>::clear_extended(0xFFFFUL);
    (void)Dma<2>::enabled_channels();
    (void)Dma<2>::any_enabled();
    Dma<2>::stop_all();

    channel_verbs<1>();
    channel_verbs<2>();
    channel_verbs<3>();
    channel_verbs<4>();
    channel_verbs<5>();
    channel_verbs<6>();
    channel_verbs<7>();
    channel_verbs<8>();
    channel_verbs<9>();
    channel_verbs<10>();
    channel_verbs<11>();
}

void engine_verbs() {
    Player::arm(&peripheral_register, DmaPriority::very_high);
    (void)Player::start(table, 8);
    Player::lap();
    Player::fail();
    (void)Player::laps();
    (void)Player::kick();
    (void)Player::service();
    Player::stop();

    Source::arm(&peripheral_register);
    (void)Source::start(half_a, half_b, 8);
    (void)Source::complete();
    (void)Source::release();
    (void)Source::service();
    Source::stop();

    Tx::arm(&peripheral_register);
    (void)Tx::start(table, 8);
    (void)Tx::complete();
    (void)Tx::service();
    Tx::stop();

    Rx::arm(&peripheral_register);
    (void)Rx::start(dst, 8);
    (void)Rx::take();
    (void)Rx::service();
    Rx::stop();
}

// ---- UART4 on its DMA2 slots, where the part has UART4 ----------------------
template <uint8_t n, DmaRequest tx, DmaRequest rx>
void uart_on_dma2() {
    if constexpr (device::has_usart(n)) {
        using TxRow = DmaRequestOf<tx>;
        using RxRow = DmaRequestOf<rx>;
        using Fed = Uart<n, P, 64, 64, UartFormat{}, DmaTxEngine<TxRow::controller, TxRow::channel>,
                         DmaRxEngine<RxRow::controller, RxRow::channel>>;
        static_assert(Usart<n>::dma_tx_slot == DmaSlot{2, 5} &&
                      Usart<n>::dma_rx_slot == DmaSlot{2, 3});
        constexpr SysClock clock;
        (void)Fed::init(clock, 115200);
        (void)Fed::write_byte('x');
        (void)Fed::harvest();
    }
}

void uart4_verbs() { uart_on_dma2<4, DmaRequest::uart4_tx, DmaRequest::uart4_rx>(); }

// The eleven vectors, bound the way an application binds them.
extern "C" BRIO_CH32_INTERRUPT void dma2_channel1_handler() { (void)DmaChannel<2, 1>::isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma2_channel2_handler() { (void)DmaChannel<2, 2>::isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma2_channel3_handler() {
    const uint8_t f = Player::service();
    if ((f & Player::flag_complete) != 0u) { Player::lap(); }
    if ((f & Player::flag_error) != 0u) { Player::fail(); }
}
extern "C" BRIO_CH32_INTERRUPT void dma2_channel4_handler() { (void)DmaChannel<2, 4>::isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma2_channel5_handler() { (void)DmaChannel<2, 5>::isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma2_channel6_handler() { (void)DmaChannel<2, 6>::isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma2_channel7_handler() { (void)DmaChannel<2, 7>::isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma2_channel8_handler() { (void)DmaChannel<2, 8>::isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma2_channel9_handler() { (void)DmaChannel<2, 9>::isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma2_channel10_handler() { (void)DmaChannel<2, 10>::isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma2_channel11_handler() { (void)DmaChannel<2, 11>::isr(); }
