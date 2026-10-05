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

// THE CORE, played by the test: the handler runs while the line is
// raised - UARTINTR is the OR of UARTMIS, so a handler that leaves a
// source standing runs again at once. A storm is a failure here, not a
// count.
uint32_t entries = 0;

void serve() {
    uint32_t back_to_back = 0;
    while (SimPl011::line_raised<0>()) {
        (void)Port::isr();
        entries = entries + 1u;
        back_to_back = back_to_back + 1u;
        REQUIRE(back_to_back < 4u);
    }
}

constexpr uint8_t pattern(uint32_t i) { return static_cast<uint8_t>((i * 7u + 3u) & 0xFFu); }

constexpr uint8_t tx_level = SimPl011::fifo_depth / 8u;   // the transport's 1/8

// The wire takes the oldest entry; the core serves whatever that raised.
// Returns false when the byte is not the next one of the pattern.
bool wire_takes_one(uint32_t& sent) {
    const auto b = SimPl011::shift<0>();
    serve();
    if (!b) {
        return true;
    }
    const bool in_order = *b == pattern(sent);
    sent = sent + 1u;
    return in_order;
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

TEST_CASE("an idle transmitter takes a FIFO's depth with no interrupt, the rest queues behind it") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Port::init(clock, 115200));
    entries = 0;

    // Straight into UARTDR while nothing is queued and the FIFO has room:
    // no mask armed, nothing raised.
    for (uint32_t i = 0; i < SimPl011::fifo_depth; ++i) {
        REQUIRE(Port::write_byte(pattern(i)));
        serve();
    }
    CHECK(SimPl011TxFifo::count[0] == SimPl011::fifo_depth);
    CHECK((regs().UARTIMSC & UartInterrupt::tx) == 0u);
    CHECK(entries == 0u);
    CHECK_FALSE(Port::tx_idle());   // BUSY: the FIFO holds them

    // The next byte finds the FIFO full: queued, and TXIM armed. The FIFO
    // is above its level, so nothing stands yet.
    REQUIRE(Port::write_byte(pattern(SimPl011::fifo_depth)));
    CHECK(SimPl011TxFifo::count[0] == SimPl011::fifo_depth);
    CHECK((regs().UARTIMSC & UartInterrupt::tx) != 0u);
    CHECK_FALSE(SimPl011::line_raised<0>());

    // The wire drains the FIFO; the fall through the level is the one
    // entry, which moves the queued byte and, the ring dry, disarms.
    uint32_t sent = 0;
    while (SimPl011TxFifo::count[0] > tx_level + 1u) {
        CHECK(wire_takes_one(sent));
    }
    CHECK(entries == 0u);
    CHECK(wire_takes_one(sent));
    CHECK(entries == 1u);
    CHECK((regs().UARTIMSC & UartInterrupt::tx) == 0u);
    CHECK(SimPl011TxFifo::count[0] == tx_level + 1u);
    while (!Port::tx_idle()) {
        CHECK(wire_takes_one(sent));
    }
    CHECK(sent == SimPl011::fifo_depth + 1u);
    CHECK(SimPl011TxFifo::lost[0] == 0u);
    CHECK(entries == 1u);
}

TEST_CASE("a refused byte writes nothing, and a full ring waits on its edge") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Port::init(clock, 115200));
    entries = 0;

    // The FIFO's depth goes to the FIFO, then the ring holds size - 1
    // (util/ring.hpp's one sacrificed slot); nothing drains either while
    // the wire stands still.
    uint32_t taken = 0;
    while (Port::write_byte(pattern(taken))) {
        ++taken;
        serve();
        REQUIRE(taken < 1000u);
    }
    CHECK(taken == SimPl011::fifo_depth + 255u);
    CHECK((regs().UARTIMSC & UartInterrupt::tx) != 0u);

    // A caller spinning on the full ring: no register written, nothing
    // raised, nothing counted.
    const uint32_t written = SimPl011TxFifo::written[0];
    for (uint32_t spin = 0; spin < 1000u; ++spin) {
        CHECK_FALSE(Port::write_byte(0xEE));
        serve();
    }
    CHECK(entries == 0u);
    CHECK(SimPl011TxFifo::written[0] == written);
    CHECK(Port::rx_overruns() == 0u);
    CHECK(Port::frame_errors() == 0u);
    CHECK(Port::hw_overruns() == 0u);
    CHECK_FALSE(Port::rx_pending());

    // And the edge is what drains it, in order.
    uint32_t sent = 0;
    while (!Port::tx_idle()) {
        REQUIRE(wire_takes_one(sent));
    }
    CHECK(sent == taken);
    CHECK(SimPl011TxFifo::lost[0] == 0u);
}

TEST_CASE("a long print takes one interrupt per FIFO level, however fast the caller spins") {
    // THE STORM this replaced: a refused write pended the line, so a
    // print spinning on a full ring took one entry per spin. Here the
    // wire takes one byte every `spins_per_frame` calls of write_byte,
    // refused or not - a caller exactly as fast as the wire, twice as
    // fast, fifty and a thousand times - and the entries are the refills
    // alone: none at all while the wire keeps up, one per FIFO level once
    // the caller outruns it.
    constexpr uint32_t n = 4096;
    constexpr uint32_t refill = SimPl011::fifo_depth - tx_level;
    constexpr uint32_t bound = (n - SimPl011::fifo_depth + refill - 1u) / refill + 1u;
    for (const uint32_t spins_per_frame : {1u, 2u, 50u, 1000u}) {
        CAPTURE(spins_per_frame);
        fresh();
        constexpr Clock clock;
        REQUIRE(Port::init(clock, 115200));
        entries = 0;
        uint32_t sent = 0;
        uint32_t calls = 0;
        bool in_order = true;
        for (uint32_t i = 0; i < n; ++i) {
            for (;;) {
                const bool taken = Port::write_byte(pattern(i));
                serve();
                calls = calls + 1u;
                if (calls % spins_per_frame == 0u) {
                    in_order = wire_takes_one(sent) && in_order;
                }
                if (taken) {
                    break;
                }
            }
        }
        while (!Port::tx_idle()) {
            in_order = wire_takes_one(sent) && in_order;
        }
        CHECK(in_order);
        CHECK(sent == n);
        CHECK(SimPl011TxFifo::lost[0] == 0u);
        CHECK(entries <= bound);
        if (spins_per_frame == 1u) {
            CHECK(entries == 0u);
        }
        if (spins_per_frame >= 50u) {
            CHECK(entries >= bound - 2u);
        }
        CHECK((regs().UARTIMSC & UartInterrupt::tx) == 0u);
    }
}

TEST_CASE("a bulk run from idle goes straight into the FIFO, its tail queued behind it") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Port::init(clock, 115200));
    entries = 0;

    // Shorter than the FIFO: all of it written, nothing armed.
    const uint8_t run[5] = {'b', 'r', 'i', 'o', '!'};
    CHECK(Port::write_bulk(run) == 5u);
    CHECK(SimPl011TxFifo::count[0] == 5u);
    CHECK((regs().UARTIMSC & UartInterrupt::tx) == 0u);
    CHECK(regs().UARTDR == static_cast<uint32_t>('!'));   // the last one written

    // Longer than the FIFO's room: the room written, the rest queued with
    // TXIM armed, and the edge sends it in order.
    uint32_t sent = 0;
    while (!Port::tx_idle()) {
        (void)SimPl011::shift<0>();
    }
    uint8_t long_run[40];
    for (uint32_t i = 0; i < sizeof long_run; ++i) {
        long_run[i] = pattern(i);
    }
    CHECK(Port::write_bulk(long_run) == sizeof long_run);
    CHECK(SimPl011TxFifo::count[0] == SimPl011::fifo_depth);
    CHECK((regs().UARTIMSC & UartInterrupt::tx) != 0u);
    while (!Port::tx_idle()) {
        REQUIRE(wire_takes_one(sent));
    }
    CHECK(sent == sizeof long_run);
    CHECK(entries == 1u);
}

TEST_CASE("an entry for the receiver alone leaves the transmit side to its edge") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Port::init(clock, 115200));
    entries = 0;

    for (uint32_t i = 0; i < SimPl011::fifo_depth + 5u; ++i) {
        REQUIRE(Port::write_byte(pattern(i)));
    }
    for (uint32_t i = 0; i < 10u; ++i) {
        (void)SimPl011::shift<0>();
    }
    const uint8_t fifo_before = SimPl011TxFifo::count[0];

    // The receive timeout stands, the transmit edge does not: the handler
    // serves the receiver and writes nothing into the transmit FIFO.
    SimPl011::raise<0>(UartInterrupt::rx_timeout);
    REQUIRE(SimPl011::line_raised<0>());
    serve();
    CHECK(entries == 1u);
    CHECK(SimPl011TxFifo::count[0] == fifo_before);
    CHECK((regs().UARTIMSC & UartInterrupt::tx) != 0u);
}

TEST_CASE("a ring running dry disarms TXIM and leaves TXRIS as it stands") {
    fresh();
    constexpr Clock clock;
    REQUIRE(Port::init(clock, 115200));
    entries = 0;

    // One byte queued behind a full FIFO; the FIFO falls through its
    // level and on to one entry before the core gets round to it.
    for (uint32_t i = 0; i < SimPl011::fifo_depth + 1u; ++i) {
        REQUIRE(Port::write_byte(pattern(i)));
    }
    while (SimPl011TxFifo::count[0] > 1u) {
        (void)SimPl011::shift<0>();
    }
    REQUIRE((regs().UARTRIS & UartInterrupt::tx) != 0u);
    serve();

    // The handler wrote the one byte - two entries, still at or below the
    // level, so the write did not clear TXRIS - and disarmed TXIM over
    // the dry ring WITHOUT clearing TXRIS: an arming that follows finds
    // the edge latched and fires.
    CHECK(entries == 1u);
    CHECK(SimPl011TxFifo::count[0] == 2u);
    CHECK((regs().UARTIMSC & UartInterrupt::tx) == 0u);
    CHECK((regs().UARTRIS & UartInterrupt::tx) != 0u);
    CHECK_FALSE(SimPl011::line_raised<0>());

    // A byte now has nothing queued ahead of it and room: straight in,
    // nothing armed.
    CHECK(Port::write_byte(0x42));
    CHECK(SimPl011TxFifo::count[0] == 3u);
    CHECK((regs().UARTIMSC & UartInterrupt::tx) == 0u);
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

    // Both engines armed on the data register, the requests enabled, the
    // receive INTERRUPTS left off - the engine has the FIFO - and the four
    // error interrupts on: under an engine they are where errors count.
    CHECK(TxEngine::armed == 1u);
    CHECK(RxEngine::armed == 1u);
    CHECK(SimPl011::regs<1>().UARTDMACR == (UartDmaControl::tx | UartDmaControl::rx));
    CHECK(SimPl011::regs<1>().UARTIMSC == UartInterrupt::errors);
    CHECK(RxEngine::blocks == 1u);   // the first free run armed by init()

    // A queued byte starts a block instead of pending the line.
    const uint32_t started = TxEngine::blocks;
    CHECK(Streamed::write_byte('z'));
    CHECK(TxEngine::blocks == started + 1u);
    CHECK_FALSE(Streamed::tx_idle());

    // A byte queued behind a running block cannot CLAIM the engine: it
    // waits in the ring, and the completion starts it as the next block.
    // dma_isr() answers the RECEIVE edge, and a transmit completion is
    // none.
    CHECK(Streamed::write_byte('y'));
    CHECK(TxEngine::blocks == started + 1u);
    TxEngine::next_flags = TxEngine::flag_complete;
    CHECK_FALSE(Streamed::dma_isr());
    CHECK(TxEngine::blocks == started + 2u);
    CHECK(TxEngine::length == 1u);

    // The completion releases exactly that run; nothing else is queued,
    // so the claim the pump took is given back and no next block starts.
    TxEngine::next_flags = TxEngine::flag_complete;
    CHECK_FALSE(Streamed::dma_isr());
    CHECK(TxEngine::blocks == started + 2u);
    CHECK_FALSE(TxEngine::busy());
    CHECK(Streamed::tx_idle());

    // A bus error throws the block away and is counted.
    CHECK(Streamed::dma_faults() == 0u);
    TxEngine::next_flags = TxEngine::flag_error;
    CHECK_FALSE(Streamed::dma_isr());
    CHECK(Streamed::dma_faults() == 1u);


    // Each received error enters the line and is counted there, its
    // status cleared through UARTICR; nothing reads UARTDR, the
    // channel's.
    SimPl011::raise<1>(UartInterrupt::overrun | UartInterrupt::frame);
    REQUIRE(SimPl011::line_raised<1>());
    CHECK_FALSE(Streamed::isr());
    CHECK_FALSE(SimPl011::line_raised<1>());
    CHECK(Streamed::hw_overruns() == 1u);
    CHECK(Streamed::frame_errors() == 1u);
    SimPl011::raise<1>(UartInterrupt::brk | UartInterrupt::frame);
    CHECK_FALSE(Streamed::isr());
    CHECK(Streamed::break_errors() == 1u);
    CHECK(Streamed::frame_errors() == 2u);
    (void)Streamed::harvest();

    // A COMPLETION IS ACTED ON ONCE. A run fills while harvest() holds
    // the guard: harvest() serves the completion the masked line owes
    // and publishes the run - so the handler that runs when the guard
    // opens finds nothing of this transport's, and begins no second run
    // over the first (on silicon the second begin aborts a busy channel
    // and loses the byte it had already fetched).
    uint8_t sink = 0;
    while (Streamed::read_byte(sink)) {
    }
    RxEngine::next_flags = RxEngine::flag_complete;
    (void)Streamed::harvest();
    while (Streamed::read_byte(sink)) {
    }
    const uint32_t runs = RxEngine::blocks;
    CHECK_FALSE(Streamed::dma_isr());
    CHECK(RxEngine::blocks == runs);

    // And a bus error met on an ENDED run is thrown away and counted
    // there; on a run still going the flag is the handler's.
    const uint16_t faults = Streamed::dma_faults();
    RxEngine::stop();
    RxEngine::next_flags = RxEngine::flag_complete | RxEngine::flag_error;
    (void)Streamed::harvest();
    CHECK(Streamed::dma_faults() == faults + 1u);
    CHECK_FALSE(Streamed::dma_isr());

    // THE END OF A RUN IS ASKED BEFORE ITS COUNT. The last element lands
    // right after the count was read: asked afterwards, the run would be
    // seen ended one element short and re-armed OVER that element - one
    // byte of the stream gone, which is what the silicon showed. Asked
    // before, the run is simply not ended yet and is left alone.
    while (Streamed::read_byte(sink)) {
    }
    (void)Streamed::harvest();
    REQUIRE(RxEngine::busy());
    const uint32_t before_the_race = RxEngine::blocks;
    RxEngine::ends_under_take = true;
    (void)Streamed::harvest();
    CHECK(RxEngine::blocks == before_the_race);

    // A RECEIVE RUN THAT FILLS IS AN EDGE: its completion publishes the
    // run onto a ring the consumer had drained, and dma_isr() answers
    // true - the glue's RxActivity, with no poll; the next completion,
    // over a ring the consumer has not drained, answers false.
    while (Streamed::read_byte(sink)) {
    }
    (void)Streamed::harvest();   // what the run holds published, a run re-armed
    while (Streamed::read_byte(sink)) {
    }
    REQUIRE(RxEngine::busy());
    REQUIRE_FALSE(Streamed::rx_pending());
    RxEngine::next_flags = RxEngine::flag_complete;
    CHECK(Streamed::dma_isr());
    CHECK(Streamed::rx_pending());
    RxEngine::next_flags = RxEngine::flag_complete;
    CHECK_FALSE(Streamed::dma_isr());

    Streamed::release();
    CHECK(SimPl011::regs<1>().UARTDMACR == 0u);
}
