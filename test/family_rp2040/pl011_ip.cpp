// THE PROOF THAT THE PL011 DRIVER KNOWS NO CHIP: brio/pl011/uart.hpp
// instantiated over a SECOND chip that is not a chip at all - two
// register blocks in RAM, a reset that memsets them, an interrupt
// controller that counts and pads that remember (brio/host/sim_pl011.hpp)
// - with every verb of the resource and of the transport named, both
// engine slots empty and both filled. Compiled here by the family's own
// compiler and by the host's (test/test_pl011/main.cpp): a file that
// needed a silicon would fail in one of the two.
#include "host/sim_pl011.hpp"
#include "pl011/uart.hpp"

using namespace brio;

using Clock = SimPl011Clock<125'000'000>;

constexpr SimPl011Pins pins0{.tx = 0, .rx = 1};
constexpr SimPl011Pins pins1{.tx = 2, .rx = 3};
static_assert(SimPl011::pins_valid(0, pins0) && SimPl011::pins_valid(1, pins1));
static_assert(!SimPl011::pins_valid(0, {.tx = 4, .rx = 4}));

using Block = Pl011Uart<SimPl011, 1>;
using Plain = Pl011Transport<SimPl011, 0, pins0, 64, 256, SimNoEngine, SimNoEngine>;
using Engined = Pl011Transport<SimPl011, 1, pins1, 64, 64, SimPl011Engine<0>, SimPl011Engine<1>>;

static_assert(Block::fifo_depth == 32u);
static_assert(Block::index == 1u);
static_assert(!Plain::has_tx_engine && !Plain::has_rx_engine);
static_assert(Engined::has_tx_engine && Engined::has_rx_engine);
static_assert(Pl011ChipClock<SimPl011, Clock>);

void pl011_resource_verbs() {
    (void)Block::regs().UARTDR;
    (void)Block::irq();
    (void)Block::reset();
    (void)Block::released();
    Block::hold();
    (void)Block::enabled();
    Block::enable(false);
    (void)Block::loopback(true);
    (void)Block::loopback();
    Block::break_send(false);
    (void)Block::fifos_enabled();
    (void)Block::line_control({}, true);
    (void)Block::divisor(UartDivisor{67, 52});
    (void)Block::divisor();
    Block::fifo_levels(UartFifoLevel::quarter, UartFifoLevel::seven_eighths);
    (void)Block::rx_fifo_level();
    (void)Block::tx_fifo_level();
    (void)Block::flags();
    (void)Block::tx_full();
    (void)Block::rx_empty();
    (void)Block::busy();
    (void)Block::read_data();
    Block::write_data(0x55);
    (void)Block::receive_status();
    Block::clear_receive_status();
    Block::interrupts(UartInterrupt::errors, true);
    (void)Block::interrupts();
    (void)Block::pending();
    (void)Block::raw_pending();
    Block::clear_pending(UartInterrupt::all);
    Block::dma_requests(true, true);
}

void pl011_transport_verbs() {
    constexpr Clock clock;
    (void)Plain::init(clock, 115200);
    (void)Plain::init(clock, 9600,
                      {.bits = UartBits::seven, .parity = UartParity::even, .stop_bits = 2});
    (void)Plain::isr();
    (void)Plain::write_byte(0x55);
    uint8_t b = 0;
    (void)Plain::read_byte(b);
    const uint8_t buf[4] = {1, 2, 3, 4};
    (void)Plain::write(buf, 4);
    (void)Plain::write_bulk(buf);
    uint8_t out[8];
    (void)Plain::read_bulk(out);
    (void)Plain::rx_pending();
    (void)Plain::tx_idle();
    (void)Plain::rx_overruns();
    (void)Plain::frame_errors();
    (void)Plain::parity_errors();
    (void)Plain::break_errors();
    (void)Plain::hw_overruns();
    Plain::clear_errors();
    (void)Plain::min_hz_for(115200);
    (void)Plain::can_baud(Clock::hz, 3'000'000);
    (void)Plain::actual_baud(Clock::hz);
    (void)Plain::set_baud(Clock::hz, 9600);
    (void)Plain::set_format({.bits = UartBits::five});
    (void)Plain::loopback(true);
    Plain::rebase(12'000'000);
    (void)Plain::harvest();
    (void)Plain::dma_faults();
    Plain::release();
}

void pl011_engine_verbs() {
    constexpr Clock clock;
    (void)Engined::init(clock, 3'000'000);
    (void)Engined::isr();
    (void)Engined::dma_isr();
    (void)Engined::harvest();
    (void)Engined::dma_faults();
    (void)Engined::write_byte(0x5A);
    const uint8_t buf[4] = {1, 2, 3, 4};
    (void)Engined::write_bulk(buf);
    (void)Engined::tx_idle();
    Engined::release();
}

// The IP's own arithmetic, judged where no chip can colour it.
static_assert(uart_divisor(125'000'000, 115200).value().integer == 67);
static_assert(uart_divisor(125'000'000, 115200).value().fraction == 52);
static_assert(uart_actual_baud(125'000'000, {67, 52}) == 115207);
static_assert(!uart_divisor(125'000'000, 8'000'000).has_value());
static_assert(!uart_divisor(0, 115200).has_value());
static_assert(uart_min_hz(115200) == 1'843'200u);
static_assert(uart_format_valid({}) && !uart_format_valid({.stop_bits = 3}));
static_assert(UartInterrupt::all == 0x7FFu);
static_assert(UartInterrupt::errors == 0x780u);
static_assert(UartDataError::dropped == 0x700u);
