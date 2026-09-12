// UART family smoke TU: the pin table of 2.19.2, the baud arithmetic of
// 4.2.7.1, the resource and the task on both instances.
#include "rp2040/clock.hpp"
#include "rp2040/uart.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::pll, 125'000'000>;

// Table 279: UART0 transmits on 0, 12, 16, 28 and receives on 1, 13,
// 17, 29; UART1 on 4, 8, 20, 24 and 5, 9, 21, 25.
static_assert(uart_tx_pin(0, 0) && uart_tx_pin(0, 12) && uart_tx_pin(0, 16) && uart_tx_pin(0, 28));
static_assert(uart_rx_pin(0, 1) && uart_rx_pin(0, 13) && uart_rx_pin(0, 17) && uart_rx_pin(0, 29));
static_assert(uart_tx_pin(1, 4) && uart_tx_pin(1, 8) && uart_tx_pin(1, 20) && uart_tx_pin(1, 24));
static_assert(uart_rx_pin(1, 5) && uart_rx_pin(1, 9) && uart_rx_pin(1, 21) && uart_rx_pin(1, 25));
static_assert(!uart_tx_pin(0, 4) && !uart_tx_pin(1, 0) && !uart_tx_pin(0, 2) && !uart_rx_pin(0, 0));
static_assert(!uart_tx_pin(0, 32));

// 4.2.7.1's own example: 115200 at 125 MHz -> 67 + 52/64, 115207 baud.
static_assert(uart_divisor(125'000'000, 115200).value().integer == 67);
static_assert(uart_divisor(125'000'000, 115200).value().fraction == 52);
static_assert(uart_actual_baud(125'000'000, {67, 52}) == 115207);
static_assert(uart_divisor(125'000'000, 7'812'500).value().integer == 1);   // UARTCLK / 16
static_assert(!uart_divisor(125'000'000, 8'000'000).has_value());           // past it
static_assert(!uart_divisor(125'000'000, 100).has_value());                 // too slow: > 65535
static_assert(!uart_divisor(0, 115200).has_value());
static_assert(uart_min_hz(115200) == 1'843'200u);
static_assert(uart_format_valid({}) && !uart_format_valid({.stop_bits = 3}));
static_assert(!uart_format_valid({.bits = static_cast<UartBits>(9)}));

constexpr UartPins u0{.tx = {0, PinFunction::uart}, .rx = {1, PinFunction::uart}};
constexpr UartPins u1{.tx = {8, PinFunction::uart}, .rx = {9, PinFunction::uart}};
static_assert(uart_pins_valid(0, u0) && uart_pins_valid(1, u1));
static_assert(!uart_pins_valid(1, u0) && !uart_pins_valid(0, u1));
static_assert(!uart_pins_valid(0, {.tx = {0, PinFunction::sio}, .rx = {1, PinFunction::uart}}));

using Console = Uart<0, u0>;
using Aux = Uart<1, u1, 128, 512>;
static_assert(Console::Resource::fifo_depth == 32u);
static_assert(!Console::has_tx_engine && !Console::has_rx_engine);

void uart_verbs() {
    constexpr SysClock clock;
    (void)Console::init(clock, 115200);
    (void)Aux::init(clock, 9600, {.bits = UartBits::seven, .parity = UartParity::even, .stop_bits = 2});
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
    (void)Console::break_errors();
    (void)Console::hw_overruns();
    Console::clear_errors();
    (void)Console::actual_baud(SysClock::pclk_hz);
    (void)Console::can_baud(SysClock::pclk_hz, 3'000'000);
    (void)Console::set_baud(SysClock::pclk_hz, 9600);
    Console::rebase(12'000'000);
    Console::release();

    (void)Pl011<1>::reset();
    (void)Pl011<1>::released();
    Pl011<1>::hold();
    (void)Pl011<1>::enabled();
    Pl011<1>::enable(false);
    (void)Pl011<1>::line_control({}, true);
    (void)Pl011<1>::divisor(UartDivisor{67, 52});
    (void)Pl011<1>::divisor();
    Pl011<1>::fifo_levels(UartFifoLevel::quarter, UartFifoLevel::three_quarters);
    (void)Pl011<1>::rx_fifo_level();
    (void)Pl011<1>::tx_fifo_level();
    (void)Pl011<1>::loopback(true);
    (void)Pl011<1>::loopback();
    Pl011<1>::break_send(false);
    (void)Pl011<1>::fifos_enabled();
    (void)Aux::set_format({.bits = UartBits::five});
    (void)Aux::loopback(true);
    (void)Pl011<1>::flags();
    (void)Pl011<1>::tx_full();
    (void)Pl011<1>::rx_empty();
    (void)Pl011<1>::busy();
    (void)Pl011<1>::read_data();
    Pl011<1>::write_data(0);
    (void)Pl011<1>::receive_status();
    Pl011<1>::clear_receive_status();
    Pl011<1>::interrupts(UartInterrupt::errors, true);
    (void)Pl011<1>::interrupts();
    (void)Pl011<1>::pending();
    (void)Pl011<1>::raw_pending();
    Pl011<1>::clear_pending(UartInterrupt::all);
    Pl011<1>::dma_requests(true, true);
    (void)Pl011<1>::irq();
}
