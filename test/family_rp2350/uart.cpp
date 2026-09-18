// UART family smoke TU: this chip's half of the PL011 contract - the pin
// table with BOTH of its function columns, the chip traits against the IP
// stratum's concept, and every verb of the resource and of the transport.
// Compiled for both packages and both architectures, so the same source
// proves that the QFN-60's table stops at GP29, that the traits satisfy
// `Pl011Chip` whichever interrupt controller `Irq` names, and that the
// public aliases instantiate over each.
#include "rp2350/uart.hpp"

using namespace brio;

// ---- the table of 9.4, both columns ---------------------------------------

// The groups of four cycle UART0, UART1, UART1, UART0 over the bank.
static_assert(uart_pad_instance(0) == 0u && uart_pad_instance(4) == 1u &&
              uart_pad_instance(8) == 1u && uart_pad_instance(12) == 0u &&
              uart_pad_instance(16) == 0u && uart_pad_instance(20) == 1u &&
              uart_pad_instance(44) == 0u);

// Function 2 on the first two pads of a group, function 11 on the other
// two - and that is the whole rule.
static_assert(uart_pad_function(0) == PinFunction::uart &&
              uart_pad_function(1) == PinFunction::uart &&
              uart_pad_function(2) == PinFunction::uart_alt &&
              uart_pad_function(3) == PinFunction::uart_alt &&
              uart_pad_function(46) == PinFunction::uart_alt);

// UART0's transmit pads: 0, 2, 12, 14, 16, 18, 28, 30, 32, 34, 44, 46.
static_assert(uart_tx_pin(0, 0) && uart_tx_pin(0, 2) && uart_tx_pin(0, 12) &&
              uart_tx_pin(0, 14) && uart_tx_pin(0, 16) && uart_tx_pin(0, 18) &&
              uart_tx_pin(0, 28) && uart_tx_pin(0, 30) && uart_tx_pin(0, 32) &&
              uart_tx_pin(0, 34) && uart_tx_pin(0, 44) && uart_tx_pin(0, 46));
// and its receive pads are each of those plus one.
static_assert(uart_rx_pin(0, 1) && uart_rx_pin(0, 3) && uart_rx_pin(0, 13) &&
              uart_rx_pin(0, 15) && uart_rx_pin(0, 47));
// UART1 has the groups between them.
static_assert(uart_tx_pin(1, 4) && uart_tx_pin(1, 6) && uart_tx_pin(1, 8) &&
              uart_tx_pin(1, 10) && uart_tx_pin(1, 20) && uart_tx_pin(1, 40) &&
              uart_rx_pin(1, 5) && uart_rx_pin(1, 7) && uart_rx_pin(1, 41));
// No pad is both instances', and no transmit pad is a receive pad.
static_assert(!uart_tx_pin(1, 0) && !uart_tx_pin(0, 4) && !uart_rx_pin(0, 5) &&
              !uart_tx_pin(0, 1) && !uart_rx_pin(0, 0));
// Nothing above the bank is a UART pad on any package.
static_assert(!uart_tx_pin(0, 48) && !uart_rx_pin(1, 200));

// A pin set is legal only when each pad is named under ITS OWN column:
// GP2 under function 2 is UART0's CTS, not its transmitter.
constexpr UartPins console_pins{.tx = {0, PinFunction::uart}, .rx = {1, PinFunction::uart}};
constexpr UartPins instrument_pins{.tx = {4, PinFunction::uart}, .rx = {5, PinFunction::uart}};
constexpr UartPins alt_pins{.tx = {2, PinFunction::uart_alt}, .rx = {3, PinFunction::uart_alt}};
static_assert(uart_pins_valid(0, console_pins) && uart_pins_valid(1, instrument_pins) &&
              uart_pins_valid(0, alt_pins));
static_assert(!uart_pins_valid(1, console_pins));
static_assert(!uart_pins_valid(0, {.tx = {2, PinFunction::uart}, .rx = {3, PinFunction::uart}}));
static_assert(!uart_pins_valid(0, {.tx = {0, PinFunction::uart_alt}, .rx = {1, PinFunction::uart}}));
static_assert(!uart_pins_valid(0, {.tx = {0, PinFunction::sio}, .rx = {1, PinFunction::uart}}));

// ---- the traits ------------------------------------------------------------

static_assert(Pl011Chip<Rp2350Pl011>);
static_assert(Rp2350Pl011::instances == 2u);
static_assert(Rp2350Pl011::fifo_depth == 32u);
static_assert(Rp2350Pl011::irq<0>() == UART0_IRQ_IRQn && Rp2350Pl011::irq<1>() == UART1_IRQ_IRQn);
static_assert(Rp2350Pl011::reset_bit<0>() == ResetBlock::uart0 &&
              Rp2350Pl011::reset_bit<1>() == ResetBlock::uart1);
// 12.6.4.1's rows, which are NOT the RP2040's 20..23.
static_assert(Rp2350Pl011::tx_request<0>() == Dreq::uart0_tx &&
              Rp2350Pl011::rx_request<0>() == Dreq::uart0_rx &&
              Rp2350Pl011::tx_request<1>() == Dreq::uart1_tx &&
              Rp2350Pl011::rx_request<1>() == Dreq::uart1_rx);
static_assert(static_cast<uint8_t>(Dreq::uart0_tx) == 28u &&
              static_cast<uint8_t>(Dreq::uart1_rx) == 31u &&
              static_cast<uint8_t>(Dreq::permanent) == 63u);
static_assert(Rp2350Pl011::tx_pad(instrument_pins) == 4u &&
              Rp2350Pl011::rx_pad(instrument_pins) == 5u);
// The receive pad comes up pulled UP: this chip's pads reset pulled DOWN,
// and a receiver enabled over a low line takes a break.
static_assert(Rp2350Pl011::rx_pad_config().pull == PinPull::up);
static_assert(Rp2350Pl011::tx_pad_config().pull == PinPull::none);
static_assert(Rp2350Pl011::engines_distinct<NoDmaEngine, NoDmaEngine>());

using SysClock = Clock<ClockSource::pll, 150'000'000UL>;
using PeriClock = Clock<ClockSource::pll, 150'000'000UL, 12'000'000UL, PeriSource::crystal>;
static_assert(Pl011ChipClock<Rp2350Pl011, SysClock>);
// UARTCLK is clk_peri, which follows the peri source and not clk_sys.
static_assert(Rp2350Pl011::uartclk_hz(SysClock{}) == 150'000'000UL);
static_assert(Rp2350Pl011::uartclk_hz(PeriClock{}) == 12'000'000UL);

// ---- the public names ------------------------------------------------------

using U0 = Pl011<0>;
using U1 = Pl011<1>;
using Serial = Uart<0, console_pins>;
using Instrument = Uart<1, instrument_pins, 512, 512>;
using AltPort = Uart<0, alt_pins, 32, 32>;

static_assert(U0::index == 0u && U1::index == 1u);
static_assert(U0::fifo_depth == 32u);
static_assert(!Serial::has_tx_engine && !Serial::has_rx_engine);
static_assert(Serial::min_hz_for(115200) == 115200u * 16u);

void uart_resource_verbs() {
    (void)U1::regs().UARTDR;
    (void)U1::irq();
    (void)U1::reset();
    (void)U1::released();
    U1::hold();
    (void)U1::enabled();
    U1::enable(false);
    (void)U1::loopback(true);
    (void)U1::loopback();
    U1::break_send(false);
    (void)U1::fifos_enabled();
    (void)U1::line_control({.bits = UartBits::seven, .parity = UartParity::even, .stop_bits = 2}, true);
    (void)U1::divisor(UartDivisor{.integer = 81, .fraction = 24});
    (void)U1::divisor();
    U1::fifo_levels(UartFifoLevel::half, UartFifoLevel::eighth);
    (void)U1::rx_fifo_level();
    (void)U1::tx_fifo_level();
    (void)U1::flags();
    (void)U1::tx_full();
    (void)U1::rx_empty();
    (void)U1::busy();
    (void)U1::read_data();
    U1::write_data(0x5A);
    (void)U1::receive_status();
    U1::clear_receive_status();
    U1::interrupts(UartInterrupt::rx, true);
    (void)U1::interrupts();
    (void)U1::pending();
    (void)U1::raw_pending();
    U1::clear_pending(UartInterrupt::all);
    U1::dma_requests(false, false);
}

void uart_transport_verbs() {
    constexpr SysClock clock;
    (void)Instrument::init(clock, 115200);
    (void)Instrument::init(clock, 9600, {.bits = UartBits::five, .parity = UartParity::odd});
    Instrument::rebase(SysClock::pclk_hz);
    (void)Instrument::set_baud(SysClock::pclk_hz, 460800);
    (void)Instrument::set_format({});
    (void)Instrument::loopback(true);
    (void)Instrument::can_baud(SysClock::pclk_hz, 115200);
    (void)Instrument::actual_baud(SysClock::pclk_hz);
    (void)Instrument::isr();
    (void)Instrument::dma_isr();
    (void)Instrument::harvest();
    (void)Instrument::dma_faults();
    (void)Instrument::write_byte('x');
    uint8_t b = 0;
    (void)Instrument::read_byte(b);
    const uint8_t out[2] = {1, 2};
    (void)Instrument::write(out, sizeof out);
    (void)Instrument::write_bulk(out);
    uint8_t in[4] = {};
    (void)Instrument::read_bulk(in);
    (void)Instrument::rx_pending();
    (void)Instrument::tx_idle();
    (void)Instrument::rx_overruns();
    (void)Instrument::frame_errors();
    (void)Instrument::parity_errors();
    (void)Instrument::break_errors();
    (void)Instrument::hw_overruns();
    Instrument::clear_errors();
    Instrument::release();

    // The second column drives exactly as the first, with no verb of its
    // own: the pad number is what chose the function.
    (void)AltPort::init(clock, 115200);
    (void)AltPort::isr();
    AltPort::release();
}

// The console instance, which is what every app of this target names.
void uart_console() {
    constexpr SysClock clock;
    constexpr Serial serial;
    (void)serial;
    (void)Serial::init(clock, 115200);
    (void)Serial::isr();
}
