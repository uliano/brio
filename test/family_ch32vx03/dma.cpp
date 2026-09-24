// DMA family smoke TU: the block, all of DMA1's channels, the request
// table of RM tables 11-5 and 11-6 (11-2 on the CH32V303), the two
// engines, and the transport slots they fill.
//
// EIGHT CHANNELS ON EVERY CH32V203 - the register notes of 11.3.1..11.3.6
// name both of its device classes among the five that have channel 8 -
// and SEVEN on the CH32V303, whose class they do not name, beside a
// second controller this driver does not reach. So the eighth channel is
// exercised where the part has it. WHAT IS per-part besides is which
// peripheral can raise a request at all: the smallest package offers one
// usart (and it is USART2), one SPI and no I2C, and the 128 KB CH32V203
// is the only one whose TIM5 and UART4 put their requests on this
// controller. So this TU names no instance as a literal where the part
// decides: it asks the table, and a neg TU proves that a request the
// part cannot raise is refused on the line that asked.
#include "ch32vx03/dma.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/usart.hpp"

using namespace brio;

using P = Ch32vx03Platform<>;
using SysClock = Clock<ClockSource::pll, 48'000'000>;

// ---- the widths and their alignment ---------------------------------------
static_assert(dma_width_of<uint8_t>() == DmaWidth::byte);
static_assert(dma_width_of<uint16_t>() == DmaWidth::half);
static_assert(dma_width_of<uint32_t>() == DmaWidth::word);
static_assert(dma_width_bytes(DmaWidth::byte) == 1 && dma_width_bytes(DmaWidth::word) == 4);
static_assert(dma_width_mask(DmaWidth::byte) == 0u && dma_width_mask(DmaWidth::half) == 1u);
static_assert(dma_width_mask(DmaWidth::word) == 3u);

// ---- the one configuration the chapter forbids ----------------------------
static_assert(dma_channel_config_valid(DmaChannelConfig{}));
static_assert(dma_channel_config_valid(DmaChannelConfig{.circular = true}));
static_assert(dma_channel_config_valid(DmaChannelConfig{.memory_to_memory = true}));
static_assert(!dma_channel_config_valid(
    DmaChannelConfig{.circular = true, .memory_to_memory = true}));

// ---- a transfer's shape, at compile time ----------------------------------
static_assert(!dma_transfer_valid(DmaTransfer{}));
static_assert(!dma_transfer_valid(DmaTransfer{.peripheral = nullptr, .memory = nullptr,
                                              .count = 4, .config = {}}));

// ---- the vectors ----------------------------------------------------------
// Seven consecutive from the table's entry 27, and the eighth, where
// there is one, on the tail this DEVICE CLASS has.
static_assert(dma_channel_irq(1) == Irq::dma1_channel1);
static_assert(dma_channel_irq(7) == Irq::dma1_channel7);
static_assert(dma_channel_irq(8) == Irq::dma1_channel8);
static_assert(static_cast<uint8_t>(Irq::dma1_channel8) ==
              irq_by_class(62, 67, irq_none));
static_assert(DmaChannel<1>::flag_shift == 0 && DmaChannel<7>::flag_shift == 24);
static_assert(dma_channel_count == device::dma1_channel_count);
static_assert(dma_channel_count == (device::dma_controller_count == 2u ? 7 : 8));

// ---- the request table (11-5 for our class, 11-6 for the other) -----------
// The rows a part cannot change: one converter, four timers of the F1
// set, and the channel each of their events answers on.
static_assert(dma_request_channel(DmaRequest::adc1) == 1);
static_assert(dma_request_channel(DmaRequest::tim2_ch3) == 1);
static_assert(dma_request_channel(DmaRequest::tim4_ch1) == 1);
static_assert(dma_request_channel(DmaRequest::tim1_ch1) == 2);
static_assert(dma_request_channel(DmaRequest::tim2_up) == 2);
static_assert(dma_request_channel(DmaRequest::tim3_ch3) == 2);
static_assert(dma_request_channel(DmaRequest::tim1_ch2) == 3);
static_assert(dma_request_channel(DmaRequest::tim3_ch4) == 3);
static_assert(dma_request_channel(DmaRequest::tim3_up) == 3);
// TIM1's fourth channel, its trigger and its commutation are one row.
static_assert(dma_request_channel(DmaRequest::tim1_ch4) == 4);
static_assert(dma_request_channel(DmaRequest::tim1_trig) == 4);
static_assert(dma_request_channel(DmaRequest::tim1_com) == 4);
static_assert(dma_request_channel(DmaRequest::tim4_ch2) == 4);
static_assert(dma_request_channel(DmaRequest::tim1_up) == 5);
static_assert(dma_request_channel(DmaRequest::tim2_ch1) == 5);
static_assert(dma_request_channel(DmaRequest::tim4_ch3) == 5);
static_assert(dma_request_channel(DmaRequest::tim1_ch3) == 6);
static_assert(dma_request_channel(DmaRequest::tim3_ch1) == 6);
static_assert(dma_request_channel(DmaRequest::tim3_trig) == 6);
static_assert(dma_request_channel(DmaRequest::tim2_ch2) == 7);
static_assert(dma_request_channel(DmaRequest::tim2_ch4) == 7);
static_assert(dma_request_channel(DmaRequest::tim4_up) == 7);

// USART2 is the one serial instance every part of the series offers.
static_assert(dma_request_channel(DmaRequest::usart2_rx) == 6);
static_assert(dma_request_channel(DmaRequest::usart2_tx) == 7);
static_assert(dma_request_present(DmaRequest::usart2_tx));

// The rows a PART decides. A count is not a list, so each of these is
// asked of the table and not of a package this TU happens to know.
static_assert(dma_request_channel(DmaRequest::usart1_tx) == (device::has_usart(1) ? 4 : 0));
static_assert(dma_request_channel(DmaRequest::usart1_rx) == (device::has_usart(1) ? 5 : 0));
static_assert(dma_request_channel(DmaRequest::usart3_tx) == (device::has_usart(3) ? 2 : 0));
static_assert(dma_request_channel(DmaRequest::usart3_rx) == (device::has_usart(3) ? 3 : 0));
// UART4 is DMA1's where there is no DMA2: on the CH32V303 its requests
// are the second controller's (table 11-3).
inline constexpr bool uart4_on_dma1 = device::has_usart(4) && device::dma_controller_count == 1u;
static_assert(dma_request_channel(DmaRequest::uart4_tx) == (uart4_on_dma1 ? 1 : 0));
static_assert(dma_request_channel(DmaRequest::uart4_rx) == (uart4_on_dma1 ? 8 : 0));
static_assert(dma_request_channel(DmaRequest::spi1_rx) == (device::spi_count >= 1u ? 2 : 0));
static_assert(dma_request_channel(DmaRequest::spi1_tx) == (device::spi_count >= 1u ? 3 : 0));
static_assert(dma_request_channel(DmaRequest::spi2_rx) == (device::spi_count >= 2u ? 4 : 0));
static_assert(dma_request_channel(DmaRequest::spi2_tx) == (device::spi_count >= 2u ? 5 : 0));
static_assert(dma_request_channel(DmaRequest::i2c1_tx) == (device::i2c_count >= 1u ? 6 : 0));
static_assert(dma_request_channel(DmaRequest::i2c1_rx) == (device::i2c_count >= 1u ? 7 : 0));
static_assert(dma_request_channel(DmaRequest::i2c2_tx) == (device::i2c_count >= 2u ? 4 : 0));
static_assert(dma_request_channel(DmaRequest::i2c2_rx) == (device::i2c_count >= 2u ? 5 : 0));
// Table 11-6's own six rows - the CH32V203RB's; the CH32V303's TIM5 is
// DMA2's.
static_assert(tim5_on_dma1() == (device::has_tim5 && device::dma_controller_count == 1u));
static_assert(dma_request_channel(DmaRequest::tim5_ch2) == (tim5_on_dma1() ? 1 : 0));
static_assert(dma_request_channel(DmaRequest::tim5_ch3) == (tim5_on_dma1() ? 3 : 0));
static_assert(dma_request_channel(DmaRequest::tim5_ch4) == (tim5_on_dma1() ? 6 : 0));
static_assert(dma_request_channel(DmaRequest::tim5_ch1) == (tim5_on_dma1() ? 7 : 0));
static_assert(dma_request_channel(DmaRequest::tim5_trig) == (tim5_on_dma1() ? 7 : 0));
static_assert(dma_request_channel(DmaRequest::tim5_up) == (tim5_on_dma1() ? 8 : 0));
static_assert(dma_request_present(DmaRequest::tim5_up) == tim5_on_dma1());

// The compile-time form: the channel of a request, refused where the
// part has not got the peripheral (neg/dma_request_absent_*.cpp).
static_assert(DmaRequestOf<DmaRequest::adc1>::channel == 1);
static_assert(DmaRequestOf<DmaRequest::usart2_tx>::channel == 7);

// ---- the tag and the two engine rules -------------------------------------
static_assert(!NoDmaEngine::present);
static_assert(dma_engine_channel<NoDmaEngine>() == 0);
static_assert(dma_engine_channel<DmaTxEngine<7>>() == 7);
static_assert(dma_engines_distinct<NoDmaEngine, NoDmaEngine>());
static_assert(dma_engines_distinct<DmaTxEngine<7>, DmaRxEngine<6>>());
static_assert(!dma_engines_distinct<DmaTxEngine<6>, DmaRxEngine<6>>());
static_assert(DmaTxEngine<7>::width == DmaWidth::byte);
static_assert(DmaRxEngine<6, uint16_t>::width == DmaWidth::half);
static_assert(DmaTxEngine<7, uint32_t>::width == DmaWidth::word);

// ---- the transport with both slots ----------------------------------------
// USART2, the instance every part has, on its own two channels.
using Fed = Uart<2, P, 64, 64, UartFormat{},
                 DmaTxEngine<DmaRequestOf<DmaRequest::usart2_tx>::channel>,
                 DmaRxEngine<DmaRequestOf<DmaRequest::usart2_rx>::channel>>;
using Plain = Uart<2, P>;
static_assert(Fed::has_tx_engine && Fed::has_rx_engine);
static_assert(!Plain::has_tx_engine && !Plain::has_rx_engine);
static_assert(Usart<2>::dma_tx_channel == 7 && Usart<2>::dma_rx_channel == 6);
static_assert(usart_dma_tx_channel(2) == 7 && usart_dma_rx_channel(2) == 6);

// ---- every verb of the block and of a channel -----------------------------
alignas(4) uint8_t src[16];
alignas(4) uint8_t dst[16];

template <uint8_t ch>
void channel_verbs() {
    using C = DmaChannel<ch>;
    C::stop();
    (void)C::number;
    (void)C::irq();
    (void)C::regs();
    (void)C::enabled();
    (void)C::configure(DmaChannelConfig{.memory_to_memory = true,
                                        .peripheral_increment = true,
                                        .peripheral_width = DmaWidth::word,
                                        .memory_width = DmaWidth::word,
                                        .priority = DmaPriority::very_high});
    (void)C::configuration();
    (void)C::set_count(4);
    (void)C::remaining();
    (void)C::set_peripheral(src);
    (void)C::set_memory(dst);
    (void)C::prepare(DmaTransfer{.peripheral = src,
                                 .memory = dst,
                                 .count = 16,
                                 .config = {.memory_to_memory = true,
                                            .peripheral_increment = true}});
    (void)C::trigger();
    C::enable(false);
    (void)C::load(DmaTransfer{.peripheral = src,
                              .memory = dst,
                              .count = 16,
                              .config = {.memory_to_memory = true,
                                         .peripheral_increment = true}});
    (void)C::flags();
    (void)C::flag(DmaFlag::complete | DmaFlag::half | DmaFlag::error | DmaFlag::global);
    C::clear(DmaFlag::all);
    C::arm(DmaFlag::complete | DmaFlag::error, true);
    (void)C::armed();
    (void)C::isr();
    (void)C::progress(16);
    C::enable(true);
    C::stop();
}

/// The eighth channel, exercised on the parts that have one: a template,
/// so the branch a part without it discards is never instantiated.
template <uint8_t ch>
void channel_verbs_if_present() {
    if constexpr (ch <= dma_channel_count) {
        channel_verbs<ch>();
    }
}

void dma_verbs() {
    Dma::open();
    (void)Dma::opened();
    (void)Dma::flags();
    Dma::clear(0xFFFFFFFFUL);
    (void)Dma::any_enabled();
    Dma::stop_all();

    channel_verbs<1>();
    channel_verbs<2>();
    channel_verbs<3>();
    channel_verbs<4>();
    channel_verbs<5>();
    channel_verbs<6>();
    channel_verbs<7>();
    channel_verbs_if_present<8>();

    // The alignment rule: the same buffer refused at its odd byte.
    (void)dma_transfer_aligned(DmaTransfer{.peripheral = src,
                                           .memory = dst + 1,
                                           .count = 2,
                                           .config = {.memory_to_memory = true,
                                                      .peripheral_width = DmaWidth::half,
                                                      .memory_width = DmaWidth::half}});
}

void engine_verbs() {
    using Tx = DmaTxEngine<DmaRequestOf<DmaRequest::usart2_tx>::channel>;
    using Rx = DmaRxEngine<DmaRequestOf<DmaRequest::usart2_rx>::channel>;

    Tx::arm(Usart<2>::data_address(), DmaPriority::high);
    (void)Tx::present;
    (void)Tx::channel;
    (void)Tx::start(src, 8);
    (void)Tx::start_fixed(src, 8);
    (void)Tx::kick(src, 8);
    (void)Tx::busy();
    (void)Tx::in_flight();
    (void)Tx::progress();
    (void)Tx::complete();
    (void)Tx::abandon();
    (void)Tx::faults();
    Tx::clear_faults();
    (void)Tx::service();
    Tx::stop();

    Rx::arm(Usart<2>::data_address());
    (void)Rx::idle();
    (void)Rx::start(dst, 8);
    (void)Rx::start_discard(dst, 8);
    (void)Rx::kick(dst, 8);
    (void)Rx::take();
    (void)Rx::harvest();
    (void)Rx::full();
    (void)Rx::capacity();
    (void)Rx::taken();
    (void)Rx::abandon();
    (void)Rx::faults();
    Rx::clear_faults();
    (void)Rx::service();
    Rx::stop();
}

void transport_verbs() {
    constexpr SysClock clock;
    (void)Fed::init(clock, 115200);
    (void)Fed::write_byte('x');
    (void)Fed::harvest();
    (void)Fed::dma_faults();
    (void)Fed::tx_idle();
    Fed::rebase(24'000'000UL);
    Usart<2>::dma_transmit(true);
    Usart<2>::dma_receive(true);
    Usart<2>::dma_transmit(false);
    Usart<2>::dma_receive(false);
}

// The engines' vectors: one handler a channel, each reading only its own
// flags. The two the transport owns, bound the way an application does.
extern "C" BRIO_CH32_INTERRUPT void dma1_channel7_handler() { (void)Fed::dma_isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel6_handler() { (void)Fed::dma_isr(); }
// The eighth channel's vector is bound where the part has the channel; on
// the CH32V303 the crt carries no such entry and the body is empty.
template <uint8_t ch>
void channel_isr_if_present() {
    if constexpr (ch <= dma_channel_count) {
        (void)DmaChannel<ch>::isr();
    }
}
extern "C" BRIO_CH32_INTERRUPT void dma1_channel8_handler() { channel_isr_if_present<8>(); }
