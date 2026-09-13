// USART family smoke TU: the resource over the whole of the chapter and
// the Uart task, on every header. USART1 and USART2 exist on every F4;
// the others are asked of the header (USART6 is missing from the 36-pin
// F410Tx alone). Every verb of the resource is called once, the
// FULL-only ones on USART1 (full everywhere) and, where the part has it,
// refused on UART4.
#include "stm32f4/clock.hpp"
#include "stm32f4/usart.hpp"

using namespace brio;

static_assert(usart_present(1) && usart_present(2));
static_assert(!usart_present(0) && !usart_present(11));
static_assert(usart_on_apb2(1) && usart_on_apb2(6) && !usart_on_apb2(2) && !usart_on_apb2(4));
static_assert(usart_is_full(1) && usart_is_full(2) && usart_is_full(6) && !usart_is_full(4) && !usart_is_full(7) &&
              !usart_is_full(9));
static_assert(usart_irq(1) == USART1_IRQn && usart_irq(2) == USART2_IRQn);
static_assert(usart_clock_mask(1) == RCC_APB2ENR_USART1EN && usart_clock_mask(2) == RCC_APB1ENR_USART2EN);
#if defined(USART6_BASE)
static_assert(usart_present(6) && usart_irq(6) == USART6_IRQn && usart_clock_mask(6) == RCC_APB2ENR_USART6EN);
#else
static_assert(!usart_present(6));
#endif
#if defined(USART3_BASE)
static_assert(usart_present(3) && usart_irq(3) == USART3_IRQn);
#else
static_assert(!usart_present(3));
#endif
#if defined(UART4_BASE)
static_assert(usart_present(4) && usart_present(5) && usart_irq(4) == UART4_IRQn && !usart_on_apb2(5));
#else
static_assert(!usart_present(4) && !usart_present(5));
#endif
#if defined(UART7_BASE)
static_assert(usart_present(7) && usart_present(8) && usart_irq(8) == UART8_IRQn);
#else
static_assert(!usart_present(7) && !usart_present(8));
#endif
#if defined(UART9_BASE)
static_assert(usart_present(9) && usart_present(10) && usart_on_apb2(9) && usart_on_apb2(10) &&
              usart_irq(10) == UART10_IRQn && usart_clock_mask(9) == RCC_APB2ENR_UART9EN);
#else
static_assert(!usart_present(9) && !usart_present(10));
#endif

// The divisor arithmetic (30.3.4).
static_assert(*usart_brr(45'000'000u, 115'200u) == 391);
static_assert(*usart_brr(90'000'000u, 115'200u) == 781);
static_assert(!usart_brr(1'000'000u, 115'200u).has_value());   // below 16
static_assert(*usart_brr_over8(45'000'000u, 115'200u) == ((781u >> 3) << 4 | (781u & 7u)));
static_assert(usart_actual_baud(45'000'000u, 391) == 115'089);
static_assert(usart_min_hz(115'200u) == 1'843'200u && usart_min_hz_over8(115'200u) == 921'600u);
static_assert(uart_format_valid(UartFormat{}) && !uart_format_valid(UartFormat{UartBits::seven, UartParity::none}) &&
              !uart_format_valid(UartFormat{UartBits::nine, UartParity::even}));
static_assert(usart_cr1_format(UartFormat{UartBits::eight, UartParity::odd}) == (USART_CR1_M | USART_CR1_PCE | USART_CR1_PS));
static_assert(uart_data_mask(UartFormat{UartBits::seven, UartParity::even}) == 0x7F);

using SysClock = Clock<ClockSource::hsi, 16'000'000>;

constexpr UartPins pins1{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
constexpr UartPins pins2{.tx = {'A', 2, PinFunction::af7}, .rx = {'A', 3, PinFunction::af7}};
constexpr UartPins pins6{.tx = {'C', 6, PinFunction::af8}, .rx = {'C', 7, PinFunction::af8}};

using Serial1 = Uart<1, pins1>;
using Serial2 = Uart<2, pins2, 128, 512>;
constexpr UartOptions fast{.over8 = true, .one_bit = true, .tx_speed = PinSpeed::high};
// The over8 option on the instance every part but the F410Tx has.
#if defined(USART6_BASE)
using Serial6 = Uart<6, pins6, 64, 256, NoDmaEngine, NoDmaEngine, fast>;
#else
using Serial6 = Uart<2, pins2, 64, 256, NoDmaEngine, NoDmaEngine, fast>;
#endif
constexpr UartOptions single{.half_duplex = true};
using OneWire = Uart<1, pins1, 64, 64, NoDmaEngine, NoDmaEngine, single>;
constexpr UartOptions flow{.rts = true, .cts = true,
                           .rts_pin = {'A', 12, PinFunction::af7}, .cts_pin = {'A', 11, PinFunction::af7}};
using Modem = Uart<1, pins1, 64, 256, NoDmaEngine, NoDmaEngine, flow>;

static_assert(ByteTransport<Serial1> && ByteTransport<Serial6> && ByteTransport<OneWire>);
static_assert(Serial1::kernel_hz<SysClock>() == 16'000'000);
static_assert(Serial1::can_baud(16'000'000u, 115'200u) && !Serial1::can_baud(1'000'000u, 115'200u));
static_assert(Serial6::divisor_for(16'000'000u, 921'600u).has_value());   // over8 reaches it
static_assert(!Serial1::divisor_for(16'000'000u, 1'843'201u).has_value());

void task_verbs() {
    constexpr SysClock clock;
    (void)Serial1::init(clock, 115'200);
    (void)Serial2::init(clock, 9'600, UartFormat{UartBits::eight, UartParity::even, UartStop::two});
    (void)Serial6::init(clock, 921'600);
    (void)OneWire::init(clock, 19'200);
    (void)Modem::init(clock, 115'200);
    (void)Serial1::isr();
    (void)Serial1::write_byte('x');
    uint8_t b = 0;
    (void)Serial1::read_byte(b);
    const uint8_t buf[4] = {1, 2, 3, 4};
    (void)Serial1::write(buf, 4);
    (void)Serial1::write_bulk(std::span<const uint8_t>{buf});
    uint8_t dst[8];
    (void)Serial1::read_bulk(std::span<uint8_t>{dst});
    (void)Serial1::rx_pending();
    (void)Serial1::tx_idle();
    (void)Serial1::rx_overruns();
    (void)Serial1::frame_errors();
    (void)Serial1::parity_errors();
    (void)Serial1::noise_errors();
    (void)Serial1::hw_overruns();
    Serial1::clear_errors();
    Serial1::rebase(16'000'000u);
    (void)Serial1::set_baud(16'000'000u, 57'600u);
    (void)Serial1::actual_baud(16'000'000u);
    (void)Serial1::min_hz_for(115'200u);
    Serial1::release();
}

void resource_verbs() {
    using U = Usart<1>;
    U::bus_clock(true);
    (void)U::bus_clock();
    U::reset();
    (void)U::enabled();
    U::enable(false);
    U::transmitter(true);
    U::receiver(true);
    (void)U::configure(UartFormat{}, 391);
    (void)U::stop_bits(UartStop::one_and_half);
    (void)U::set_brr(391);
    (void)U::brr();
    (void)U::oversampling8(true);
    (void)U::oversampling8();
    (void)U::actual_baud(45'000'000u);
    U::one_bit_sampling(true);
    (void)U::mute_mode(MuteConfig{MuteWake::address_mark, 5});
    (void)U::mute();
    U::unmute();
    (void)U::muted();
    (void)U::lin(LinConfig{true, true});
    U::lin_off();
    (void)U::lin_enabled();
    U::send_break();
    (void)U::break_pending();
    (void)U::half_duplex(true);
    (void)U::half_duplex();
    (void)U::irda(IrdaConfig{true, 4});
    U::irda_off();
    (void)U::irda_enabled();
    (void)U::smartcard(SmartcardConfig{true, 16, 5});
    U::smartcard_off();
    (void)U::smartcard_enabled();
    (void)U::synchronous(UsartSyncConfig{true, true, true});
    (void)U::synchronous_off();
    (void)U::synchronous_enabled();
    (void)U::flow_control(true, true);
    (void)U::rts_enabled();
    (void)U::cts_enabled();
    U::dma_transmit(true);
    U::dma_receive(true);
    U::interrupts(USART_CR1_IDLEIE | USART_CR1_TCIE, true);
    U::rxne_interrupt(true);
    U::txe_interrupt(true);
    (void)U::txe_interrupt();
    U::tc_interrupt(false);
    U::idle_interrupt(false);
    U::parity_interrupt(false);
    U::break_interrupt(false);
    U::cts_interrupt(false);
    U::error_interrupt(false);
    (void)U::status();
    (void)U::flag(UsartFlag::rxne);
    U::clear_flags(UsartFlag::tc | UsartFlag::lbd);
    U::clear_by_read();
    (void)U::tx_empty();
    (void)U::tx_complete();
    (void)U::rx_ready();
    U::write_data(0x55);
    U::write_word(0x1AA);
    (void)U::read_data();
    (void)U::read_word();
    (void)U::data_address();
}

// A basic instance where the header has one: the FULL-only verbs answer
// false and write nothing, and the half stop bits are refused.
#if defined(UART4_BASE)
using U4 = Usart<4>;
static_assert(!U4::is_full && !U4::on_apb2);
void basic_instance() {
    U4::bus_clock(true);
    (void)U4::configure(UartFormat{}, 391);
    (void)U4::stop_bits(UartStop::half);        // false
    (void)U4::smartcard(SmartcardConfig{});     // false
    (void)U4::synchronous(UsartSyncConfig{});   // false
    (void)U4::flow_control(true, true);         // false
    (void)U4::lin(LinConfig{});                 // LIN is every instance's
    (void)U4::irda(IrdaConfig{});
    (void)U4::half_duplex(true);
}
#endif
