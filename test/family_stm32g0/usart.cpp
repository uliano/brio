// USART family smoke TU: the Usart<n> resource, the Uart task above it
// and the baud arithmetic between them - on USART1 and USART2, which
// every G0 has; the higher instances are the negatives' business. The
// pads are PA9/PA10 (USART1 AF1) and PA2/PA3 (USART2 AF1), DS13560
// table 13 - bonded on every package that carries these ports.
#include "stm32g0/clock.hpp"
#include "stm32g0/usart.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::pll, 64'000'000>;

static_assert(usart_present(1) && usart_present(2));
static_assert(!usart_present(7) && !usart_present(0));
static_assert(usart_bus_clock(1).apb2 && usart_bus_clock(1).mask == RCC_APBENR2_USART1EN);
static_assert(!usart_bus_clock(2).apb2 && usart_bus_clock(2).mask == RCC_APBENR1_USART2EN);
static_assert(usart_has_clock_select(1));
static_assert(!usart_has_clock_select(6));

// 33.5.7, OVER8 = 0: BRR = USARTDIV = f / baud, nearest, >= 16.
static_assert(usart_brr(64'000'000, 115200).value() == 556);
static_assert(usart_brr(16'000'000, 115200).value() == 139);
static_assert(usart_brr(8'000'000, 9600).value() == 833);      // the chapter's own example
static_assert(usart_brr(48'000'000, 921'600).value() == 52);   // and its second
static_assert(usart_brr(64'000'000, 4'000'000).value() == 16); // USARTDIV floor: legal
static_assert(!usart_brr(64'000'000, 5'000'000).has_value());  // below 16: refused
static_assert(!usart_brr(0, 115200).has_value());
static_assert(!usart_brr(64'000'000, 0).has_value());
static_assert(usart_actual_baud(64'000'000, 556) == 115107);
static_assert(usart_actual_baud(16'000'000, 139) == 115107);
static_assert(usart_min_hz(115200) == 1'843'200);

constexpr UartPins u2{.tx = {'A', 2, PinFunction::af1}, .rx = {'A', 3, PinFunction::af1}};
constexpr UartPins u1{.tx = {'A', 9, PinFunction::af1}, .rx = {'A', 10, PinFunction::af1}};
static_assert(uart_pins_valid(u2));
static_assert(!uart_pins_valid({.tx = {'A', 2, PinFunction::af1}, .rx = {'A', 2, PinFunction::af1}}));
static_assert(!uart_pins_valid({.tx = {'G', 2, PinFunction::af1}, .rx = {'A', 3, PinFunction::af1}}));

using Console = Uart<2, u2>;
using Aux = Uart<1, u1, 128, 512>;

void usart_verbs() {
    constexpr SysClock clock;
    (void)Console::init(clock, 115200);
    (void)Aux::init(clock, 9600, {.bits = UartBits::eight, .parity = UartParity::even, .stop_bits = 2});
    (void)Console::isr();
    (void)Aux::isr();
    (void)Console::write_byte(0x55);
    uint8_t b;
    (void)Console::read_byte(b);
    const uint8_t buf[4] = {1, 2, 3, 4};
    (void)Console::write(buf, 4);
    (void)Console::write_bulk(buf);
    uint8_t out[8];
    (void)Console::read_bulk(out);
    (void)Console::rx_pending();
    (void)Console::tx_idle();
    (void)Console::rx_overruns();
    (void)Console::frame_errors();
    (void)Console::parity_errors();
    (void)Console::noise_errors();
    (void)Console::hw_overruns();
    Console::clear_errors();
    (void)Console::actual_baud(SysClock::pclk_hz);
    (void)Console::can_baud(SysClock::pclk_hz, 3'000'000);
    Console::rebase(16'000'000);
    Console::release();

    Usart<2>::bus_clock(true);
    (void)Usart<2>::kernel_clock(UsartClock::pclk);
    (void)Usart<2>::configure({}, 556);
    Usart<2>::enable(true);
    (void)Usart<2>::enabled();
    (void)Usart<2>::status();
    Usart<2>::clear_flags(USART_ICR_ORECF);
    (void)Usart<2>::read_data();
    Usart<2>::write_data(0);
    Usart<2>::rxne_interrupt(false);
    Usart<2>::txe_interrupt(false);
    (void)Usart<2>::txe_interrupt();
    Usart<2>::reset();
    (void)Usart<2>::irq();
}
