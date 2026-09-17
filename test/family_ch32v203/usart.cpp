// USART family smoke TU: the resource's verbs, the frame and divisor
// arithmetic at compile time, and the interrupt-driven transport with
// its vector binding.
//
// WHICH INSTANCE a part offers is the table's answer and it is not
// always the first n: the smallest package offers exactly one usart and
// it is USART2, because it bonds neither of USART1's pin pairs. So this
// TU names no instance number as a literal - it asks - and a neg TU
// proves that USART1 is refused there.
#include "ch32v203/clock.hpp"
#include "ch32v203/platform.hpp"
#include "ch32v203/usart.hpp"
#include "util/stream.hpp"

using namespace brio;

using P = Ch32v203Platform<>;
using SysClock = Clock<ClockSource::pll, 48'000'000>;

/// The console's instance on this part: USART1 where the package bonds
/// its pads, USART2 otherwise - the only one every part of the family
/// offers.
inline constexpr uint8_t console_instance = device::has_usart(1) ? 1 : 2;
/// The HIGHEST instance this part offers, which is UART4 on the two
/// parts that have four and the console's own everywhere else: a second
/// resource to exercise, and the one whose vector number moves with the
/// device class.
inline constexpr uint8_t last_instance = device::has_usart(4) ? 4 : console_instance;

using Console = Usart<console_instance>;
using Last = Usart<last_instance>;
using Serial = Uart<console_instance, P>;
using Nine = Uart<console_instance, P, 32, 32, UartFormat{UartBits::seven, UartParity::even}>;

// USART1 is the PB2 instance and the other three are PB1's, which is
// what decides the gate and the clock a divisor counts.
static_assert(usart_bus_for(1) == Bus::pb2);
static_assert(usart_bus_for(2) == Bus::pb1 && usart_bus_for(4) == Bus::pb1);
static_assert(usart_gate_for(1) == rcc_pb2_usart1 && usart_gate_for(4) == rcc_pb1_uart4);
static_assert(usart_irq_for(1) == Irq::usart1 && usart_irq_for(4) == Irq::uart4);

// UART4's default pads on this family are PB0/PB1, not the PC10/PC11 of
// the manual's other table (the file header of usart.hpp).
static_assert(usart_pads_for(1).tx == Pad{'A', 9} && usart_pads_for(1).rx == Pad{'A', 10});
static_assert(usart_pads_for(2).tx == Pad{'A', 2} && usart_pads_for(2).rx == Pad{'A', 3});
static_assert(usart_pads_for(3).tx == Pad{'B', 10});
static_assert(usart_pads_for(4).tx == Pad{'B', 0} && usart_pads_for(4).rx == Pad{'B', 1});

// The frame: seven data bits exist only with a parity bit and nine only
// without, because the register counts the WORD.
static_assert(uart_format_valid(UartFormat{}));
static_assert(uart_format_valid(UartFormat{UartBits::seven, UartParity::odd}));
static_assert(!uart_format_valid(UartFormat{UartBits::seven, UartParity::none}));
static_assert(!uart_format_valid(UartFormat{UartBits::nine, UartParity::even}));
static_assert(usart_ctlr1_format(UartFormat{}) == 0u);
static_assert(usart_ctlr1_format(UartFormat{UartBits::nine, UartParity::none}) == usart_m);
static_assert(usart_ctlr1_format(UartFormat{UartBits::eight, UartParity::odd}) ==
              (usart_m | usart_pce | usart_ps));
static_assert(usart_ctlr2_stop(UartStop::two) == (2u << usart_stop_shift));
static_assert(uart_data_mask(UartFormat{}) == 0xFFu);
static_assert(uart_data_mask(UartFormat{UartBits::nine, UartParity::none}) == 0x1FFu);
static_assert(uart_data_mask(UartFormat{UartBits::seven, UartParity::even}) == 0x7Fu);

// The divisor is pclk/baud in sixteenths, rounded to nearest, and below
// sixteen the generator has nothing to divide.
static_assert(usart_divisor(48'000'000UL, 115200) == 417u);
static_assert(usart_divisor(0u, 115200) == 0u);
static_assert(usart_divisor(115200, 0) == 0u);
static_assert(usart_divisor_valid(16u) && !usart_divisor_valid(15u));
static_assert(!usart_divisor_valid(0x10000u));

// The transport is a ByteTransport for util's services.
static_assert(ByteSink<Serial> && ByteSource<Serial>);

void usart_verbs() {
    constexpr SysClock clock;

    Console::bus_clock(true);
    Console::reset();
    (void)Console::configure(UartFormat{}, 417);
    Console::enable(true);
    Console::transmitter(true);
    Console::receiver(true);
    Console::interrupt(usart_rxneie, true);
    (void)Console::status();
    (void)Console::read_word();
    Console::write_word(0x55);
    (void)Console::take_errors();
    (void)Console::actual_baud(48'000'000UL);
    Console::enable(false);
    (void)Console::number;
    (void)Console::regs();

    Last::bus_clock(true);
    Last::bus_clock(false);
    Pfic::enable(usart_irq_for(last_instance));
    Pfic::disable(usart_irq_for(last_instance));

    (void)Serial::init(clock, 115200);
    (void)Nine::init(clock, 9600);
    (void)Serial::write_byte('x');
    static const uint8_t greeting[2] = {'h', 'i'};
    Serial::write(greeting, 2);
    uint8_t got = 0;
    (void)Serial::read_byte(got);
    (void)Serial::rx_available();
    (void)Serial::tx_idle();
    (void)Serial::baud();
    (void)Serial::actual_baud(48'000'000UL);
    Serial::rebase(24'000'000UL);
    (void)Serial::rx_overruns();
    (void)Serial::hw_overruns();
    (void)Serial::frame_errors();
    (void)Serial::noise_errors();
    (void)Serial::parity_errors();
    (void)Serial::isr();
}

extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }
