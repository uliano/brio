// Host tests for the ARM PrimeCell UART driver (brio/pl011/uart.hpp)
// over a chip that is not a chip: two register blocks in RAM
// (brio/host/sim_pl011.hpp). What is judged here is what a bench cannot
// see cheaply - the exact WORDS a bring-up leaves in the block, and the
// ORDER of the acts that make it - and what a bench cannot see at all:
// that the driver compiles and runs with no silicon under it.
// Run with: ctest --preset host (or ctest --preset host -R test_pl011)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <stdint.h>

#include "host/sim_pl011.hpp"
#include "pl011/uart.hpp"

using namespace brio;

namespace {

using Clock = SimPl011Clock<125'000'000>;

constexpr SimPl011Pins pins{.tx = 0, .rx = 1};
using Port = Pl011Transport<SimPl011, 0, pins, 64, 256, SimNoEngine, SimNoEngine>;
using Block = Pl011Uart<SimPl011, 0>;

SimPl011Regs& regs() { return SimPl011::regs<0>(); }

void fresh() {
    SimPl011::reset_all();
    HostPlatform::reset();
}

}   // namespace

TEST_CASE("the baud arithmetic is the fractional divider's own") {
    // The worked example every PL011 data sheet carries: 115200 baud at
    // 125 MHz is 67 + 52/64, which really produces 115207.
    const auto d = uart_divisor(125'000'000, 115200);
    REQUIRE(d.has_value());
    CHECK(d->integer == 67);
    CHECK(d->fraction == 52);
    CHECK(uart_actual_baud(125'000'000, *d) == 115207u);

    // The reach: UARTCLK/16 at the top, an integer part of 65535 at the
    // bottom, and nothing outside it.
    CHECK(uart_divisor(125'000'000, 7'812'500).value().integer == 1);
    CHECK(uart_divisor(125'000'000, 7'812'500).value().fraction == 0);
    // A rate within half a sixty-fourth of a divisor rounds onto it.
    CHECK(uart_divisor(125'000'000, 7'812'501).value().integer == 1);
    CHECK(uart_divisor(125'000'000, 7'812'501).value().fraction == 0);
    CHECK_FALSE(uart_divisor(125'000'000, 8'000'000).has_value());
    CHECK_FALSE(uart_divisor(125'000'000, 100).has_value());
    CHECK_FALSE(uart_divisor(0, 115200).has_value());
    CHECK_FALSE(uart_divisor(125'000'000, 0).has_value());
    CHECK(uart_min_hz(115200) == 1'843'200u);
    CHECK(uart_actual_baud(125'000'000, {0, 0}) == 0u);
}

TEST_CASE("the frame word is what LCR_H holds") {
    fresh();
    REQUIRE(Block::reset());

    // 8N1 with the FIFOs: WLEN = 3 (bits - 5) in bits 6:5, FEN set, no
    // parity, one stop bit.
    REQUIRE(Block::line_control({}, true));
    CHECK(regs().UARTLCR_H == ((3u << 5) | UartLineControl::fifos));

    // 7E2 with the FIFOs off: WLEN = 2, PEN and EPS, STP2.
    REQUIRE(Block::line_control(
        {.bits = UartBits::seven, .parity = UartParity::even, .stop_bits = 2}, false));
    CHECK(regs().UARTLCR_H == ((2u << 5) | UartLineControl::parity_enable |
                               UartLineControl::even_parity | UartLineControl::stop2));

    // Odd parity is PEN without EPS; five bits is WLEN 0.
    REQUIRE(Block::line_control({.bits = UartBits::five, .parity = UartParity::odd}, false));
    CHECK(regs().UARTLCR_H == UartLineControl::parity_enable);

    // A format the block has not got is refused and writes nothing.
    const uint32_t before = regs().UARTLCR_H;
    CHECK_FALSE(Block::line_control({.stop_bits = 3}, true));
    CHECK(regs().UARTLCR_H == before);
}

TEST_CASE("the divisor is latched by the line control that follows it") {
    fresh();
    REQUIRE(Block::reset());
    REQUIRE(Block::line_control({}, true));
    const uint32_t frame = regs().UARTLCR_H;

    REQUIRE(Block::divisor({.integer = 67, .fraction = 52}));
    CHECK(regs().UARTIBRD == 67u);
    CHECK(regs().UARTFBRD == 52u);
    // The dummy write that makes them take leaves the frame as it was.
    CHECK(regs().UARTLCR_H == frame);
    const UartDivisor back = Block::divisor();
    CHECK(back.integer == 67);
    CHECK(back.fraction == 52);

    // Every configuring verb is refused while the UART is enabled.
    Block::enable(true);
    CHECK(Block::enabled());
    CHECK_FALSE(Block::divisor({.integer = 1, .fraction = 0}));
    CHECK_FALSE(Block::line_control({}, true));
    CHECK_FALSE(Block::loopback(true));
    CHECK(regs().UARTIBRD == 67u);
}

TEST_CASE("the trigger levels, the masks and the enable touch their own bits") {
    fresh();
    REQUIRE(Block::reset());

    Block::fifo_levels(UartFifoLevel::half, UartFifoLevel::eighth);
    CHECK(regs().UARTIFLS == (2u << UartTriggerField::rx_lsb));
    CHECK(Block::rx_fifo_level() == UartFifoLevel::half);
    CHECK(Block::tx_fifo_level() == UartFifoLevel::eighth);
    Block::fifo_levels(UartFifoLevel::quarter, UartFifoLevel::seven_eighths);
    CHECK(Block::rx_fifo_level() == UartFifoLevel::quarter);
    CHECK(Block::tx_fifo_level() == UartFifoLevel::seven_eighths);

    Block::interrupts(UartInterrupt::rx | UartInterrupt::rx_timeout, true);
    CHECK(regs().UARTIMSC == (UartInterrupt::rx | UartInterrupt::rx_timeout));
    Block::interrupts(UartInterrupt::rx, false);
    CHECK(regs().UARTIMSC == UartInterrupt::rx_timeout);
    CHECK(Block::interrupts() == UartInterrupt::rx_timeout);

    // The loop-back is set with the UART down and SURVIVES the enable
    // going off: enable(false) clears the three enables and nothing else.
    REQUIRE(Block::loopback(true));
    CHECK(Block::loopback());
    Block::enable(true);
    CHECK(regs().UARTCR == (UartControl::enable | UartControl::tx_enable |
                            UartControl::rx_enable | UartControl::loopback));
    Block::enable(false);
    CHECK(regs().UARTCR == UartControl::loopback);
    CHECK(Block::loopback());

    Block::dma_requests(true, false);
    CHECK(regs().UARTDMACR == UartDmaControl::tx);
    Block::dma_requests(false, true);
    CHECK(regs().UARTDMACR == UartDmaControl::rx);
}

TEST_CASE("a bring-up leaves the block set up, in the order it promises") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Port::init(clock, 115200));

    // The divisor from the clock the chip named UARTCLK, and the frame.
    CHECK(regs().UARTIBRD == 67u);
    CHECK(regs().UARTFBRD == 52u);
    CHECK(regs().UARTLCR_H == ((3u << 5) | UartLineControl::fifos));
    CHECK(regs().UARTIFLS == (2u << UartTriggerField::rx_lsb));

    // The receive interrupts armed, nothing else, and the line enabled.
    CHECK(regs().UARTIMSC == (UartInterrupt::rx | UartInterrupt::rx_timeout));
    CHECK(SimPl011Interrupts::line_enabled[0]);
    CHECK(regs().UARTCR == (UartControl::enable | UartControl::tx_enable |
                            UartControl::rx_enable));
    CHECK(regs().UARTDMACR == 0u);

    // THE ORDER init() promises: the receive pad, with the setup the
    // chip states for it, BEFORE the enable; the transmit pad after it.
    CHECK(SimPl011Bench::pad_function[1] == SimPl011::pad_function);
    CHECK(SimPl011Bench::pad_pulled_up[1]);
    CHECK(SimPl011Bench::pad_function[0] == SimPl011::pad_function);
    CHECK_FALSE(SimPl011Bench::pad_pulled_up[0]);
    CHECK(SimPl011Bench::pad_claimed_at[1] < SimPl011Bench::enabled_at);
    CHECK(SimPl011Bench::enabled_at < SimPl011Bench::pad_claimed_at[0]);

    // A rate the generator cannot express is refused, and says so.
    CHECK_FALSE(Port::init(clock, 8'000'000));
    CHECK(Port::can_baud(Clock::hz, 3'000'000));
    CHECK_FALSE(Port::can_baud(Clock::hz, 8'000'000));
    CHECK(Port::min_hz_for(115200) == 1'843'200u);
}

TEST_CASE("release puts the pads back and the block into reset") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Port::init(clock, 115200));
    Port::release();

    CHECK_FALSE(SimPl011Interrupts::line_enabled[0]);
    CHECK(regs().UARTIMSC == 0u);
    CHECK(regs().UARTDMACR == 0u);
    CHECK(SimPl011Bench::pad_function[0] == 0u);
    CHECK(SimPl011Bench::pad_function[1] == 0u);
    CHECK(SimPl011Bench::pad_released_at[0] != 0u);
    CHECK(SimPl011Bench::pad_released_at[1] != 0u);
    CHECK_FALSE(Block::released());
}

TEST_CASE("a queued byte pends the line, and the handler is the FIFO's one feeder") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Port::init(clock, 115200));
    const uint32_t pends_after_init = SimPl011Interrupts::pends[0];

    // write_byte() queues and PENDS; it never writes the data register.
    regs().UARTDR = 0;
    CHECK(Port::write_byte('A'));
    CHECK(SimPl011Interrupts::pends[0] == pends_after_init + 1u);
    CHECK(regs().UARTDR == 0u);
    CHECK_FALSE(Port::tx_idle());

    // The handler moves the ring into the FIFO and, with the ring empty,
    // disarms the transmit interrupt instead of arming it.
    CHECK_FALSE(Port::isr());
    CHECK(regs().UARTDR == static_cast<uint32_t>('A'));
    CHECK((regs().UARTIMSC & UartInterrupt::tx) == 0u);
    CHECK(Port::tx_idle());

    // A bulk run is one nudge for the whole run.
    const uint8_t run[5] = {'b', 'r', 'i', 'o', '!'};
    const uint32_t before = SimPl011Interrupts::pends[0];
    CHECK(Port::write_bulk(run) == 5u);
    CHECK(SimPl011Interrupts::pends[0] == before + 1u);
    CHECK_FALSE(Port::isr());
    CHECK(regs().UARTDR == static_cast<uint32_t>('!'));   // the last one written
}

TEST_CASE("the transmit ring refuses when full, and counts nothing for it") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Port::init(clock, 115200));

    // The ring holds size - 1 (util/ring.hpp's one sacrificed slot), and
    // nothing drains it while the handler is not called.
    uint32_t queued = 0;
    while (Port::write_byte(static_cast<uint8_t>(queued & 0xFFu))) {
        ++queued;
        REQUIRE(queued < 1000u);
    }
    CHECK(queued == 255u);
    CHECK(Port::rx_overruns() == 0u);
    CHECK(Port::frame_errors() == 0u);
    CHECK(Port::hw_overruns() == 0u);
    CHECK_FALSE(Port::rx_pending());

    // The handler empties it in one pass: this fake's transmit FIFO is
    // never full.
    CHECK_FALSE(Port::isr());
    CHECK(Port::tx_idle());
}

TEST_CASE("a rate change drains, reprograms and comes back up") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Port::init(clock, 115200));

    REQUIRE(Port::set_baud(Clock::hz, 9600));
    CHECK(regs().UARTIBRD == 813u);
    CHECK(regs().UARTFBRD == 51u);
    CHECK(Port::actual_baud(Clock::hz) == 9600u);
    CHECK(regs().UARTCR == (UartControl::enable | UartControl::tx_enable |
                            UartControl::rx_enable));

    // A rate the new clock cannot carry changes nothing.
    CHECK_FALSE(Port::set_baud(1'000'000, 8'000'000));
    CHECK(regs().UARTIBRD == 813u);

    // rebase() keeps the BIT RATE across a clock change.
    Port::rebase(48'000'000);
    CHECK(Port::actual_baud(48'000'000) == 9600u);

    // And the format changes under the running port.
    REQUIRE(Port::set_format({.bits = UartBits::seven, .parity = UartParity::odd}));
    CHECK(regs().UARTLCR_H == ((2u << 5) | UartLineControl::parity_enable |
                               UartLineControl::fifos));
    CHECK_FALSE(Port::set_format({.stop_bits = 3}));
}

TEST_CASE("the engine slots carry the requests and the blocks") {
    using TxEngine = SimPl011Engine<0>;
    using RxEngine = SimPl011Engine<1>;
    using Streamed = Pl011Transport<SimPl011, 1, SimPl011Pins{.tx = 2, .rx = 3}, 64, 64,
                                    TxEngine, RxEngine>;
    fresh();
    TxEngine::reset();
    RxEngine::reset();
    constexpr Clock clock;
    REQUIRE(Streamed::init(clock, 3'000'000));

    // Both engines armed on the data register, the requests enabled, and
    // the receive INTERRUPTS left off: the engine has the FIFO.
    CHECK(TxEngine::armed == 1u);
    CHECK(RxEngine::armed == 1u);
    CHECK(SimPl011::regs<1>().UARTDMACR == (UartDmaControl::tx | UartDmaControl::rx));
    CHECK(SimPl011::regs<1>().UARTIMSC == 0u);
    CHECK(RxEngine::blocks == 1u);   // the first free run armed by init()

    // A queued byte starts a block instead of pending the line.
    const uint32_t started = TxEngine::blocks;
    CHECK(Streamed::write_byte('z'));
    CHECK(TxEngine::blocks == started + 1u);
    CHECK_FALSE(Streamed::tx_idle());

    // The completion releases exactly that run; nothing else is queued,
    // so no next block starts.
    TxEngine::next_flags = TxEngine::flag_complete;
    CHECK(Streamed::dma_isr());
    CHECK(Streamed::tx_idle());

    // A bus error throws the block away and is counted.
    CHECK(Streamed::dma_faults() == 0u);
    TxEngine::next_flags = TxEngine::flag_error;
    CHECK(Streamed::dma_isr());
    CHECK(Streamed::dma_faults() == 1u);

    // harvest() reads the sticky errors once per run and re-arms.
    SimPl011::regs<1>().UARTRSR = UartReceiveStatus::overrun | UartReceiveStatus::frame;
    (void)Streamed::harvest();
    CHECK(SimPl011::regs<1>().UARTRSR == 0u);
    CHECK(Streamed::hw_overruns() == 1u);
    CHECK(Streamed::frame_errors() == 1u);

    Streamed::release();
    CHECK(SimPl011::regs<1>().UARTDMACR == 0u);
}
