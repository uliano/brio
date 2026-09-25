// DMA family smoke TU: the first controller, all of its channels, the
// request table of RM 11.2.3 (tables 11-5 and 11-6, and 11-2 to 11-4 on
// the CH32V303), the two engines, and the transport slots they fill.
// The second controller, the CH32V303's, is dma2.cpp's.
//
// EIGHT CHANNELS ON DMA1 OF EVERY CH32V203 - the register notes of
// 11.3.1..11.3.6 name both of its device classes among the five that have
// channel 8 - and SEVEN on the CH32V303, whose class they do not name,
// beside a second controller of eleven. So the eighth channel is
// exercised where the part has it. WHAT IS per-part besides is which
// peripheral can raise a request at all, and on which controller: the
// smallest package offers one usart (and it is USART2), one SPI and no
// I2C, the 128 KB CH32V203 puts its TIM5 and UART4 on this controller,
// and the CH32V303 puts both on its second. So this TU names no instance
// as a literal where the part decides: it asks the table, and a neg TU
// proves that a request the part cannot raise is refused on the line
// that asked.
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

// ---- the 64 KB rule of the CH32V303's DMA1 (11.2.3's note) ----------------
static_assert(!dma_span_crosses_64k(0x2000'0000UL, 16384, DmaWidth::word, true));
static_assert(dma_span_crosses_64k(0x2000'FFF0UL, 8, DmaWidth::word, true));
static_assert(!dma_span_crosses_64k(0x2000'FFF0UL, 8, DmaWidth::word, false));
static_assert(!dma_span_crosses_64k(0x0801'0000UL, 0, DmaWidth::byte, true));
static_assert(dma1_bounded_to_64k == (device::device_class == DeviceClass::v30x_d8));
static_assert(DmaChannel<1, 1>::bounded_to_64k == dma1_bounded_to_64k);

// ---- the block, the addresses and the vectors -----------------------------
// DMA1's channels twenty bytes apart from 0x08, the eighth included where
// there is one; seven vectors consecutive from the table's entry 27 and
// the eighth, where there is one, on the tail this DEVICE CLASS has.
static_assert(dma_base_for(1) == 0x4002'0000UL);
static_assert(dma_channel_address(1, 1) == 0x4002'0008UL);
static_assert(dma_channel_address(1, 7) == 0x4002'0080UL);
static_assert(dma_channel_address(1, 8) == 0x4002'0094UL);
static_assert(dma_channel_irq(1, 1) == Irq::dma1_channel1);
static_assert(dma_channel_irq(1, 7) == Irq::dma1_channel7);
static_assert(dma_channel_irq(1, 8) == Irq::dma1_channel8);
static_assert(static_cast<uint8_t>(Irq::dma1_channel8) == irq_by_class(62, 67, irq_none));
static_assert(DmaChannel<1, 1>::flag_shift == 0 && DmaChannel<1, 7>::flag_shift == 24);
static_assert(DmaChannel<1, 3>::slot == DmaSlot{1, 3});
static_assert(!DmaChannel<1, 7>::extended);
static_assert(Dma<1>::channel_count == device::dma1_channel_count);
static_assert(Dma<1>::channel_count == (device::dma_controller_count == 2u ? 7 : 8));
static_assert(dma_channels_of(1) == Dma<1>::channel_count);
static_assert(dma_channels_of(2) == (device::dma_controller_count == 2u ? 11 : 0));
static_assert(dma_channels_of(3) == 0);
static_assert(dma_channel_exists(1, 7) && !dma_channel_exists(1, 9) && !dma_channel_exists(1, 0));
static_assert(!Dma<1>::has_extended_flags);

// ---- the slot -------------------------------------------------------------
static_assert(!DmaSlot{}.present());
static_assert(DmaSlot{1, 1}.present() && !DmaSlot{1, 0}.present());
static_assert(DmaSlot{1, 5} != DmaSlot{2, 5});

// ---- the request table: DMA1's rows ---------------------------------------
// The rows a part cannot change: one converter, four timers of the F1
// set, and the channel each of their events answers on - DMA1 on every
// part of the family.
static_assert(dma_request_channel(DmaRequest::adc1) == DmaSlot{1, 1});
static_assert(dma_request_channel(DmaRequest::tim2_ch3) == DmaSlot{1, 1});
static_assert(dma_request_channel(DmaRequest::tim4_ch1) == DmaSlot{1, 1});
static_assert(dma_request_channel(DmaRequest::tim1_ch1) == DmaSlot{1, 2});
static_assert(dma_request_channel(DmaRequest::tim2_up) == DmaSlot{1, 2});
static_assert(dma_request_channel(DmaRequest::tim3_ch3) == DmaSlot{1, 2});
static_assert(dma_request_channel(DmaRequest::tim1_ch2) == DmaSlot{1, 3});
static_assert(dma_request_channel(DmaRequest::tim3_ch4) == DmaSlot{1, 3});
static_assert(dma_request_channel(DmaRequest::tim3_up) == DmaSlot{1, 3});
// TIM1's fourth channel, its trigger and its commutation are one row.
static_assert(dma_request_channel(DmaRequest::tim1_ch4) == DmaSlot{1, 4});
static_assert(dma_request_channel(DmaRequest::tim1_trig) == DmaSlot{1, 4});
static_assert(dma_request_channel(DmaRequest::tim1_com) == DmaSlot{1, 4});
static_assert(dma_request_channel(DmaRequest::tim4_ch2) == DmaSlot{1, 4});
static_assert(dma_request_channel(DmaRequest::tim1_up) == DmaSlot{1, 5});
static_assert(dma_request_channel(DmaRequest::tim2_ch1) == DmaSlot{1, 5});
static_assert(dma_request_channel(DmaRequest::tim4_ch3) == DmaSlot{1, 5});
static_assert(dma_request_channel(DmaRequest::tim1_ch3) == DmaSlot{1, 6});
static_assert(dma_request_channel(DmaRequest::tim3_ch1) == DmaSlot{1, 6});
static_assert(dma_request_channel(DmaRequest::tim3_trig) == DmaSlot{1, 6});
static_assert(dma_request_channel(DmaRequest::tim2_ch2) == DmaSlot{1, 7});
static_assert(dma_request_channel(DmaRequest::tim2_ch4) == DmaSlot{1, 7});
static_assert(dma_request_channel(DmaRequest::tim4_up) == DmaSlot{1, 7});

// USART2 is the one serial instance every part of the family offers.
static_assert(dma_request_channel(DmaRequest::usart2_rx) == DmaSlot{1, 6});
static_assert(dma_request_channel(DmaRequest::usart2_tx) == DmaSlot{1, 7});
static_assert(dma_request_present(DmaRequest::usart2_tx));

// The rows a PART decides. A count is not a list, so each of these is
// asked of the table and not of a package this TU happens to know.
constexpr DmaSlot on1(bool present, uint8_t ch) { return present ? DmaSlot{1, ch} : DmaSlot{}; }
static_assert(dma_request_channel(DmaRequest::usart1_tx) == on1(device::has_usart(1), 4));
static_assert(dma_request_channel(DmaRequest::usart1_rx) == on1(device::has_usart(1), 5));
static_assert(dma_request_channel(DmaRequest::usart3_tx) == on1(device::has_usart(3), 2));
static_assert(dma_request_channel(DmaRequest::usart3_rx) == on1(device::has_usart(3), 3));
// UART4 is DMA1's where there is no DMA2, and DMA2's where there is
// (table 11-3: transmit on 5, receive on 3).
inline constexpr bool uart4_on_dma1 = device::has_usart(4) && device::dma_controller_count == 1u;
inline constexpr bool uart4_on_dma2 = device::has_usart(4) && device::dma_controller_count == 2u;
static_assert(dma_request_channel(DmaRequest::uart4_tx) ==
              (uart4_on_dma1 ? DmaSlot{1, 1} : uart4_on_dma2 ? DmaSlot{2, 5} : DmaSlot{}));
static_assert(dma_request_channel(DmaRequest::uart4_rx) ==
              (uart4_on_dma1 ? DmaSlot{1, 8} : uart4_on_dma2 ? DmaSlot{2, 3} : DmaSlot{}));
static_assert(dma_request_channel(DmaRequest::spi1_rx) == on1(device::spi_count >= 1u, 2));
static_assert(dma_request_channel(DmaRequest::spi1_tx) == on1(device::spi_count >= 1u, 3));
static_assert(dma_request_channel(DmaRequest::spi2_rx) == on1(device::spi_count >= 2u, 4));
static_assert(dma_request_channel(DmaRequest::spi2_tx) == on1(device::spi_count >= 2u, 5));
static_assert(dma_request_channel(DmaRequest::i2c1_tx) == on1(device::i2c_count >= 1u, 6));
static_assert(dma_request_channel(DmaRequest::i2c1_rx) == on1(device::i2c_count >= 1u, 7));
static_assert(dma_request_channel(DmaRequest::i2c2_tx) == on1(device::i2c_count >= 2u, 4));
static_assert(dma_request_channel(DmaRequest::i2c2_rx) == on1(device::i2c_count >= 2u, 5));
// Table 11-6's own six rows - the CH32V203RB's; the CH32V303's TIM5 is
// DMA2's (dma2.cpp).
static_assert(tim5_on_dma1() == (device::has_tim5 && device::dma_controller_count == 1u));
static_assert(!(tim5_on_dma1() && tim5_on_dma2()));
static_assert(dma_request_channel(DmaRequest::tim5_ch2) == on1(tim5_on_dma1(), 1) ||
              tim5_on_dma2());
static_assert(dma_request_channel(DmaRequest::tim5_ch3) == on1(tim5_on_dma1(), 3) ||
              tim5_on_dma2());
static_assert(dma_request_channel(DmaRequest::tim5_ch4) == on1(tim5_on_dma1(), 6) ||
              tim5_on_dma2());
static_assert(dma_request_channel(DmaRequest::tim5_ch1) == on1(tim5_on_dma1(), 7) ||
              tim5_on_dma2());
static_assert(dma_request_channel(DmaRequest::tim5_trig) == on1(tim5_on_dma1(), 7) ||
              tim5_on_dma2());
static_assert(dma_request_channel(DmaRequest::tim5_up) == on1(tim5_on_dma1(), 8) ||
              tim5_on_dma2());
static_assert(dma_request_present(DmaRequest::tim5_up) == device::has_tim5);
// Every DMA2 row answers the empty slot where there is no DMA2.
static_assert(device::dma_controller_count == 2u ||
              (!dma_request_present(DmaRequest::dac1) && !dma_request_present(DmaRequest::tim6_up) &&
               !dma_request_present(DmaRequest::tim8_up) && !dma_request_present(DmaRequest::adc2) &&
               !dma_request_present(DmaRequest::uart5_tx) && !dma_request_present(DmaRequest::sdio)));

// The compile-time form: the slot of a request, refused where the part
// has not got the peripheral (neg/dma_request_absent_*.cpp).
static_assert(DmaRequestOf<DmaRequest::adc1>::slot == DmaSlot{1, 1});
static_assert(DmaRequestOf<DmaRequest::adc1>::controller == 1 &&
              DmaRequestOf<DmaRequest::adc1>::channel == 1);
static_assert(DmaRequestOf<DmaRequest::usart2_tx>::channel == 7);

// ---- the tag and the two engine rules -------------------------------------
static_assert(!NoDmaEngine::present);
static_assert(dma_engine_slot<NoDmaEngine>() == DmaSlot{});
static_assert(dma_engine_slot<DmaTxEngine<1, 7>>() == DmaSlot{1, 7});
static_assert(dma_engines_distinct<NoDmaEngine, NoDmaEngine>());
static_assert(dma_engines_distinct<DmaTxEngine<1, 7>, DmaRxEngine<1, 6>>());
static_assert(!dma_engines_distinct<DmaTxEngine<1, 6>, DmaRxEngine<1, 6>>());
static_assert(DmaTxEngine<1, 7>::width == DmaWidth::byte);
static_assert(DmaRxEngine<1, 6, uint16_t>::width == DmaWidth::half);
static_assert(DmaTxEngine<1, 7, uint32_t>::width == DmaWidth::word);
static_assert(DmaTxEngine<1, 7>::controller == 1 && DmaTxEngine<1, 7>::slot == DmaSlot{1, 7});

// ---- the transport with both slots ----------------------------------------
// USART2, the instance every part has, on its own two channels.
using Usart2Tx = DmaRequestOf<DmaRequest::usart2_tx>;
using Usart2Rx = DmaRequestOf<DmaRequest::usart2_rx>;
using Fed = Uart<2, P, 64, 64, UartFormat{}, DmaTxEngine<Usart2Tx::controller, Usart2Tx::channel>,
                 DmaRxEngine<Usart2Rx::controller, Usart2Rx::channel>>;
using Plain = Uart<2, P>;
static_assert(Fed::has_tx_engine && Fed::has_rx_engine);
static_assert(!Plain::has_tx_engine && !Plain::has_rx_engine);
static_assert(Usart<2>::dma_tx_channel == 7 && Usart<2>::dma_rx_channel == 6);
static_assert(Usart<2>::dma_tx_slot == DmaSlot{1, 7} && Usart<2>::dma_rx_slot == DmaSlot{1, 6});
static_assert(usart_dma_tx_channel(2) == 7 && usart_dma_rx_channel(2) == 6);
static_assert(usart_dma_tx_slot(2) == DmaSlot{1, 7} && usart_dma_rx_slot(2) == DmaSlot{1, 6});

// ---- every verb of the block and of a channel -----------------------------
alignas(4) uint8_t src[16];
alignas(4) uint8_t dst[16];

template <uint8_t ch>
void channel_verbs() {
    using C = DmaChannel<1, ch>;
    C::stop();
    (void)C::controller;
    (void)C::number;
    (void)C::slot;
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
    (void)C::accepts(DmaTransfer{.peripheral = src, .memory = dst, .count = 16,
                                 .config = {.memory_to_memory = true}});
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
    if constexpr (dma_channel_exists(1, ch)) {
        channel_verbs<ch>();
    }
}

void dma_verbs() {
    Dma<1>::open();
    (void)Dma<1>::opened();
    (void)Dma<1>::regs();
    (void)Dma<1>::flags();
    Dma<1>::clear(0xFFFFFFFFUL);
    (void)Dma<1>::enabled_channels();
    (void)Dma<1>::any_enabled();
    (void)dma_enabled_channels(1);
    (void)dma_enabled_channels(2);   // the empty mask where there is no DMA2
    Dma<1>::stop_all();

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
    (void)dma_transfer_crosses_64k(DmaTransfer{.peripheral = src, .memory = dst, .count = 2,
                                               .config = {.memory_to_memory = true}});
}

void engine_verbs() {
    using Tx = DmaTxEngine<Usart2Tx::controller, Usart2Tx::channel>;
    using Rx = DmaRxEngine<Usart2Rx::controller, Usart2Rx::channel>;

    Tx::arm(Usart<2>::data_address(), DmaPriority::high);
    (void)Tx::present;
    (void)Tx::controller;
    (void)Tx::channel;
    (void)Tx::slot;
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
    if constexpr (dma_channel_exists(1, ch)) {
        (void)DmaChannel<1, ch>::isr();
    }
}
extern "C" BRIO_CH32_INTERRUPT void dma1_channel8_handler() { channel_isr_if_present<8>(); }
