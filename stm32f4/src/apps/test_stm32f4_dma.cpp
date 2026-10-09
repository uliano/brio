// test_stm32f4_dma - the reference bench suite for the STM32F4's two DMA
// controllers: the stream, its FIFO, its eight channels, the double
// buffer, the flow controller, the five flags and the two engines a byte
// transport plugs into - stm32f4/dma.hpp.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NOTHING TO WIRE. This chapter is the one that needs no pad at all,
// because a DMA controller's own memory-to-memory mode is an instrument:
// DMA2 moves a block of RAM into another block of RAM with no peripheral
// in the path, so every width, every burst, every FIFO threshold and
// every error can be measured against a byte-exact expectation and a
// cycle count. What memory-to-memory cannot show - circular mode, the
// double buffer, a stream paced by a peripheral's request - is measured
// on THE CONSOLE'S OWN TRANSMITTER, whose bit rate is the ruler and whose
// output the host sees.
//
// What is exercised, letter by letter:
//   a  the two controllers: the gates closed at reset, the reset values,
//      the sixteen vectors, and which controller reaches the bus matrix
//   b  a memory-to-memory block, byte-exact, with its flags in order
//   c  what the driver refuses, and that a refusal writes nothing
//   d  the FIFO: table 49 accepted and refused cell by cell, and every
//      accepted one moving its block exactly
//   e  the throughput ladder: bytes per core cycle for each width and
//      each memory burst
//   f  the counting rules: table 48, the alignments, a packed transfer
//   g  circular mode on the console's transmitter: laps, the reload, and
//      where the half-transfer flag falls
//   h  the double buffer: the two halves in turn, CT read back, and the
//      idle half replaced under a running stream
//   i  the abort and 10.3.14's resume: how long EN takes to come down
//      and what SxNDTR holds when it does
//   j  a transfer error: an address no DMA master reaches
//   k  priority arbitration between two streams of one controller
//   l  the console through the DMA engines: the Uart task with both
//      slots filled, measured against its own bit rate
//   m  the receive engine in its circular shape, on the transmitter's
//      own echo (single-wire half duplex): every byte, the laps counted,
//      the stream never re-armed, and what one harvest() costs
//   n  the circular receive's two edges: a lap the consumer slept
//      through, counted and skipped, and the vectors' edge re-opened by a
//      drain
//   r  a released requester: USART6's transmit, SPI1's transmit, TIM1's
//      update and ADC1's conversion, each with its request standing and
//      released five ways, then a stream on its own cell as the next
//      owner - what the release contract's reset buys on this silicon
//   u  (outside z) brio stress: the host's stream into the receive
//      engine at 115200, 460800 and 921600, the consumer looking at most
//      once a millisecond and only after an edge from the vectors - every
//      byte accounted: brio stress --letters u --board <board>
//   p  (outside z) brio stress: letter u's sink at 115200 and 921600 with
//      four copies kept busy on DMA2 at low and at very_high, the
//      console's ring at its default level - every byte accounted:
//      brio stress --letters p --board <board>
//
// build: boards = f429zi,f446re,f411ce,f469ni
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>

#include "stm32f4/adc.hpp"
#include "stm32f4/clock.hpp"
#include "stm32f4/dma.hpp"
#include "stm32f4/dwt.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/spi.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/tim.hpp"
#include "stm32f4/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

#if defined(STM32F411xE)
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 100'000'000, 25'000'000>;
#elif defined(STM32F429xx) || defined(STM32F469xx)
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000>;
#else
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000, brio::HseMode::bypass>;
#endif
constexpr SysClock clock;

namespace {

using namespace brio;

// ---- the console, and the cells its requests sit on --------------------------
//
// The board's console instance decides which (controller, stream,
// channel) the two engines take: USART1 is DMA2's stream 7 channel 4 out
// and stream 2 channel 4 in; USART2 is DMA1's stream 6 and stream 5, both
// channel 4 (RM0090 tables 43 and 44, and their RM0390 / RM0383 twins);
// USART3 is DMA1's stream 3 channel 4 out and stream 1 channel 4 in
// (RM0386 table 29). The Uart checks the cells at compile time - these
// lines only have to name the right ones.
#if defined(STM32F446xx)
constexpr UartPins console_pins{.tx = {'A', 2, PinFunction::af7}, .rx = {'A', 3, PinFunction::af7}};
constexpr uint8_t console_instance = 2;
using ConsoleTxStream = DmaStream<1, 6>;
using ConsoleRxStream = DmaStream<1, 5>;
using ConsoleTxEngine = DmaTxEngine<1, 6, 4>;
using ConsoleRxEngine = DmaRxEngine<1, 5, 4>;
#elif defined(STM32F469xx)
constexpr UartPins console_pins{.tx = {'B', 10, PinFunction::af7}, .rx = {'B', 11, PinFunction::af7}};
constexpr uint8_t console_instance = 3;
using ConsoleTxStream = DmaStream<1, 3>;
using ConsoleRxStream = DmaStream<1, 1>;
using ConsoleTxEngine = DmaTxEngine<1, 3, 4>;
using ConsoleRxEngine = DmaRxEngine<1, 1, 4>;
#else
constexpr UartPins console_pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
constexpr uint8_t console_instance = 1;
using ConsoleTxStream = DmaStream<2, 7>;
using ConsoleRxStream = DmaStream<2, 2>;
using ConsoleTxEngine = DmaTxEngine<2, 7, 4>;
using ConsoleRxEngine = DmaRxEngine<2, 2, 4>;
#endif
constexpr uint8_t console_tx_channel = 4;

using Console = Usart<console_instance>;
using Serial = Uart<console_instance, console_pins>;
constexpr Serial serial;

/// The same port with both engine slots filled - a SECOND type on the
/// same instance, so letter l can hand the peripheral over and take it
/// back without either transport knowing about the other.
constexpr uint32_t rx_ring_bytes = 64;
using DmaSerial = Uart<console_instance, console_pins, rx_ring_bytes, 256, ConsoleTxEngine,
                       ConsoleRxEngine>;

/// And the same port again in SINGLE-WIRE HALF DUPLEX, which is what
/// gives the receive engine a stimulus with no wire at all: with HDSEL
/// the receiver listens on the transmit pad, so everything the port
/// sends it also hears (30.3.10). Both engines, both directions, one
/// pad.
constexpr UartOptions loop_options{.half_duplex = true};
using LoopSerial = Uart<console_instance, console_pins, rx_ring_bytes, 256, ConsoleTxEngine,
                        ConsoleRxEngine, loop_options>;

/// And once more with a ring sized for letter u's pace: a consumer that
/// looks once a millisecond must hold a millisecond of the fastest rung,
/// 92 bytes at 921600, and a lap of 256 holds 2.7 ms of it.
constexpr uint32_t stress_ring_bytes = 256;
using StressSerial = Uart<console_instance, console_pins, stress_ring_bytes, 256,
                          ConsoleTxEngine, ConsoleRxEngine>;

TestBench<Serial> bench;

// ---- the wireless instrument: memory to memory on DMA2 ------------------------

using Block = Dma<2>;
using Work = DmaStream<2, 0>;    ///< the worker of most letters
using Rival = DmaStream<2, 3>;   ///< the second stream of the arbitration letter

constexpr uint16_t buffer_bytes = 2048;
alignas(4) volatile uint8_t src[buffer_bytes];
alignas(4) volatile uint8_t dst[buffer_bytes];

constexpr uint16_t rival_bytes = 512;
alignas(4) volatile uint8_t rival_src[rival_bytes];
alignas(4) volatile uint8_t rival_dst[rival_bytes];

/// The core-coupled memory's address. On the parts that HAVE a CCM it is
/// the one RAM the bus matrix does not let a DMA master reach (RM0090
/// 2.3: the CCM hangs off the CPU's own D-bus); on the parts that have
/// not, nothing is mapped there. Either way a stream pointed at it takes
/// a bus error, which is letter j's whole point.
constexpr uint32_t unreachable_address = 0x1000'0000u;

void fill_source() {
    for (uint16_t i = 0; i < buffer_bytes; ++i) {
        src[i] = static_cast<uint8_t>(i * 7u + 1u);
        dst[i] = 0;
    }
}

bool same(uint16_t bytes) {
    for (uint16_t i = 0; i < bytes; ++i) {
        if (dst[i] != src[i]) {
            return false;
        }
    }
    return true;
}

bool untouched_after(uint16_t bytes) {
    for (uint16_t i = bytes; i < buffer_bytes; ++i) {
        if (dst[i] != 0u) {
            return false;
        }
    }
    return true;
}

/// A memory-to-memory transfer of `count` items of `width`, source and
/// destination both walking. Direct mode is forbidden here (10.3.6), so
/// the FIFO is always on and the threshold is an argument.
DmaTransfer mem_to_mem(uint16_t count, DmaWidth width, DmaBurst memory_burst,
                       DmaFifoThreshold threshold, DmaPriority priority = DmaPriority::low) {
    DmaTransfer t{};
    t.peripheral = src;
    t.memory = dst;
    t.count = count;
    t.config.direction = DmaDirection::memory_to_memory;
    t.config.peripheral_increment = true;
    t.config.memory_increment = true;
    t.config.peripheral_width = width;
    t.config.memory_width = width;
    t.config.memory_burst = memory_burst;
    t.config.priority = priority;
    t.config.use_fifo = true;
    t.config.fifo_threshold = threshold;
    return t;
}

/// Wait for a stream to raise its completion flag. Bounded: a wrong
/// configuration must show as a failed letter, never as a dead board.
template <typename Stream>
bool wait_complete(uint32_t spins = 4'000'000u) {
    while (!Stream::flag(DmaFlag::complete)) {
        if (spins-- == 0u) {
            return false;
        }
    }
    return true;
}

// ---- the rulers ---------------------------------------------------------------

constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000u;

uint32_t systick_period() { return SysTick->LOAD + 1u; }

/// SysTick counts DOWN, so a later sample is a smaller number - unless
/// the period wrapped between the two, which at 1000 Hz means the
/// measurement was longer than a millisecond and is not to be trusted.
uint32_t val_delta(uint32_t first, uint32_t second) {
    return first >= second ? first - second : first + systick_period() - second;
}

/// Run one memory-to-memory transfer and report the core cycles it took,
/// or 0 when it did not complete.
uint32_t timed_block(const DmaTransfer& t) {
    if (!Work::prepare(t)) {
        return 0;
    }
    const uint32_t v0 = SysTick->VAL;
    if (!Work::trigger()) {
        return 0;
    }
    if (!wait_complete<Work>()) {
        return 0;
    }
    const uint32_t took = val_delta(v0, SysTick->VAL);
    return took;
}

// ---- what the registers held before this program touched them ----------------

struct BootState {
    bool gate1 = false, gate2 = false;
    uint32_t lisr1 = 0, hisr1 = 0, lisr2 = 0, hisr2 = 0;
    uint32_t cr = 0, fcr = 0, ndtr = 0;
};
BootState boot;

// ---- the console, quiesced -----------------------------------------------------
//
// Letters g, h and i drive the console's TRANSMITTER from a DMA stream,
// which means taking the data register from the interrupt-driven
// transport for the length of the letter. The handover is safe only with
// the ring drained and the shifter empty, and 10.3.17's warning says the
// stream goes off before the peripheral's request bit does.

void quiesce_console() {
    uint32_t spins = 4'000'000u;
    while (!Serial::tx_idle() && spins-- != 0u) {
    }
    spins = 400'000u;
    while (!Console::tx_complete() && spins-- != 0u) {
    }
}

void release_console_stream() {
    ConsoleTxStream::stop();
    Console::dma_transmit(false);
}

// ---- a  the two controllers ----------------------------------------------------

void ta_block() {
    bench.verdict("both gates are closed at reset", !boot.gate1 && !boot.gate2);
    print(serial, "  reset: LISR1=", hex(boot.lisr1), " HISR1=", hex(boot.hisr1),
          " LISR2=", hex(boot.lisr2), " HISR2=", hex(boot.hisr2), crlf);
    bench.verdict("no flag stands at reset",
                  boot.lisr1 == 0u && boot.hisr1 == 0u && boot.lisr2 == 0u && boot.hisr2 == 0u);
    print(serial, "  reset: S0CR=", hex(boot.cr), " S0FCR=", hex(boot.fcr), " S0NDTR=",
          boot.ndtr, crlf);
    bench.verdict("a stream comes up idle, its FIFO register at 0x21",
                  boot.cr == 0u && boot.fcr == 0x21u && boot.ndtr == 0u);

    Block::init();
    bench.verdict("init opens the gate", Block::bus_clock());

    // The reset line puts a configured stream back where it started.
    (void)Work::configure(DmaStreamConfig{.channel = 5, .priority = DmaPriority::very_high});
    const bool written = Work::channel() == 5u;
    Block::reset();
    bench.verdict("a configured stream is written and the block reset clears it",
                  written && Work::control() == 0u && Work::fifo_control() == 0x21u);

    print(serial, "  streams=", static_cast<uint32_t>(Block::streams),
          " channels=", static_cast<uint32_t>(dma_channels),
          " fifo=", static_cast<uint32_t>(dma_fifo_words), " words", crlf);
    bench.verdict("eight streams, eight channels, a four-word FIFO",
                  Block::streams == 8u && dma_channels == 8u && dma_fifo_words == 4u);

    // Sixteen vectors and no two the same: checked here over the whole
    // pair, because a shared line would make an ISR body answer for
    // somebody else's stream.
    bool distinct = true;
    for (uint8_t a = 0; a < 16u; ++a) {
        for (uint8_t b = static_cast<uint8_t>(a + 1u); b < 16u; ++b) {
            const IRQn_Type ia = dma_stream_irq(static_cast<uint8_t>(1u + a / 8u), a % 8u);
            const IRQn_Type ib = dma_stream_irq(static_cast<uint8_t>(1u + b / 8u), b % 8u);
            if (ia == ib) {
                distinct = false;
            }
        }
    }
    bench.verdict("one vector per stream, sixteen of them, all distinct", distinct);

    bench.verdict("memory to memory is DMA2's alone",
                  Dma<2>::memory_to_memory && !Dma<1>::memory_to_memory);
}

// ---- b  a memory-to-memory block -----------------------------------------------

void tb_memory_to_memory() {
    Block::init();
    fill_source();
    constexpr uint16_t bytes = 256;
    const DmaTransfer t = mem_to_mem(bytes, DmaWidth::byte, DmaBurst::single,
                                     DmaFifoThreshold::full);
    bench.verdict("the transfer is accepted", Work::prepare(t));
    bench.verdict("prepared and not running", !Work::enabled() && Work::count() == bytes);
    const bool up = Work::trigger();
    const bool done = wait_complete<Work>();
    bench.verdict("it starts on the enable alone - no peripheral asks", up && done);

    print(serial, "  ", bytes, " bytes, NDTR=", Work::count(), " flags=", hex(Work::flags()),
          " EN=", Work::enabled() ? 1u : 0u, crlf);
    bench.verdict("the block is byte-exact", same(bytes));
    bench.verdict("and nothing past it was written", untouched_after(bytes));
    bench.verdict("NDTR reached zero", Work::count() == 0u);
    bench.verdict("the hardware cleared EN at the end", !Work::enabled());
    bench.verdict("the completion flag stands, alone",
                  Work::flag(DmaFlag::complete) && !Work::flag(DmaFlag::errors));
    Work::clear(DmaFlag::all);
    bench.verdict("and the clear register takes it away", Work::flags() == 0u);

    // THE READ-BACK THAT WEDGES: on the STM32F446 a load of SxCR in the
    // cycles after the EN store wedges the stream every other time -
    // sixteen bytes into the FIFO, nothing drains, EN standing, no flag,
    // until the controller is reset. Four blocks enabled with a read of
    // EN right after the store, reported and not judged (the F429 takes
    // them all, the F411 wedges like the F446); then four with the store
    // left alone, which is
    // what enable() does, and which every part completes.
    uint8_t wedged = 0;
    for (uint8_t k = 0; k < 4u; ++k) {
        Work::stop();
        if (Work::enabled()) {
            Block::init();
        }
        (void)Work::prepare(t);
        Work::regs().CR = Work::regs().CR | DMA_SxCR_EN;
        (void)Work::enabled();   // the load right after the store
        if (!wait_complete<Work>(200'000u)) {
            ++wedged;
        }
    }
    print(serial, "  four blocks enabled with SxCR read back right after the store: ", wedged,
          " wedged (NDTR ", Work::count(), " of ", bytes, " on the last)", crlf);
    if (Work::enabled()) {
        Block::init();
    }
    uint8_t alone = 0;
    for (uint8_t k = 0; k < 4u; ++k) {
        Work::stop();
        if (Work::prepare(t) && Work::trigger() && wait_complete<Work>(200'000u) && same(bytes)) {
            ++alone;
        }
    }
    bench.verdict("four blocks back to back with the enable's store left alone all complete",
                  alone == 4u);

    // AND THE ENABLE IS THE ONE STORE THAT DOES NOT READ BACK AT ONCE.
    // A plain register - SxNDTR here, which has no side effect on a
    // stopped stream - is visible to the very next load; EN is not, and
    // the verdict above is what stands guard over that: enable() polls
    // its read-back, and a version that trusted one read reported this
    // running stream as a refused one.
    Work::stop();
    (void)Work::set_count(0x5A5Au);
    uint32_t slack = 0;
    while (Work::count() != 0x5A5Au && slack < 8u) {
        ++slack;
    }
    print(serial, "  SxNDTR reads back after ", slack, " stale read(s); EN takes one more",
          crlf);
    bench.verdict("a plain stream register reads back at once", slack == 0u);
    Work::stop();
}

// ---- c  what the driver refuses ------------------------------------------------

void tc_refusals() {
    Block::init();
    Dma<1>::init();
    const DmaTransfer good = mem_to_mem(64, DmaWidth::byte, DmaBurst::single,
                                        DmaFifoThreshold::full);

    bench.verdict("memory to memory on DMA1 is refused - its peripheral port "
                  "does not reach the bus matrix",
                  !DmaStream<1, 0>::configure(good.config));
    bench.verdict("and accepted on DMA2", Work::configure(good.config));

    DmaStreamConfig c = good.config;
    c.circular = true;
    bench.verdict("memory to memory with circular mode is refused",
                  !Work::configure(c));
    c = good.config;
    c.use_fifo = false;
    bench.verdict("memory to memory in direct mode is refused", !Work::configure(c));
    c = good.config;
    c.double_buffer = true;
    bench.verdict("memory to memory with the double buffer is refused",
                  !Work::configure(c));
    c = good.config;
    c.peripheral_flow_control = true;
    c.circular = true;
    c.direction = DmaDirection::peripheral_to_memory;
    bench.verdict("a circular stream under the peripheral flow controller is refused",
                  !Work::configure(c));
    c = good.config;
    c.direction = DmaDirection::peripheral_to_memory;
    c.use_fifo = false;
    c.memory_width = DmaWidth::word;
    bench.verdict("direct mode with two different widths is refused", !Work::configure(c));
    c.memory_width = DmaWidth::byte;
    c.memory_burst = DmaBurst::incr4;
    bench.verdict("direct mode with a burst is refused", !Work::configure(c));
    c = good.config;
    c.channel = 8;
    bench.verdict("a ninth channel is refused", !Work::configure(c));

    // A refusal writes NOTHING: the register still holds what the last
    // accepted configuration put there.
    (void)Work::configure(good.config);
    const uint32_t before = Work::control();
    c = good.config;
    c.channel = 8;
    (void)Work::configure(c);
    bench.verdict("a refused configuration leaves the register alone",
                  Work::control() == before);

    // Transfers, not configurations.
    DmaTransfer t = good;
    t.memory = nullptr;
    bench.verdict("a transfer with no destination is refused", !Work::prepare(t));
    t = good;
    t.count = 0;
    bench.verdict("a zero count is refused - it would serve nothing", !Work::prepare(t));
    t = good;
    t.config.peripheral_width = DmaWidth::word;
    t.config.memory_width = DmaWidth::word;
    t.peripheral = &src[1];
    bench.verdict("a word transfer from an unaligned address is refused",
                  !Work::prepare(t));

    // And SxCR is read-only while EN is set, so configure() refuses then
    // instead of storing into a register the silicon ignores.
    const DmaTransfer big = mem_to_mem(buffer_bytes, DmaWidth::byte, DmaBurst::single,
                                       DmaFifoThreshold::full);
    (void)Work::load(big);
    const bool refused_running = !Work::configure(good.config);
    (void)wait_complete<Work>();
    Work::stop();
    bench.verdict("a running stream refuses to be reconfigured", refused_running);
}

// ---- d  the FIFO, table 49 ------------------------------------------------------

void td_fifo() {
    Block::init();
    fill_source();

    constexpr DmaFifoThreshold thresholds[4] = {
        DmaFifoThreshold::quarter, DmaFifoThreshold::half, DmaFifoThreshold::three_quarters,
        DmaFifoThreshold::full};
    constexpr DmaBurst bursts[4] = {DmaBurst::single, DmaBurst::incr4, DmaBurst::incr8,
                                    DmaBurst::incr16};
    constexpr DmaWidth widths[3] = {DmaWidth::byte, DmaWidth::half, DmaWidth::word};

    uint8_t accepted = 0, refused = 0, exact = 0, moved = 0;
    for (uint8_t w = 0; w < 3u; ++w) {
        for (uint8_t th = 0; th < 4u; ++th) {
            for (uint8_t b = 0; b < 4u; ++b) {
                const uint16_t items = static_cast<uint16_t>(256u / dma_width_bytes(widths[w]));
                const DmaTransfer t = mem_to_mem(items, widths[w], bursts[b], thresholds[th]);
                const bool legal = dma_fifo_burst_valid(thresholds[th], bursts[b], widths[w],
                                                        DmaBurst::single, widths[w]);
                for (uint16_t i = 0; i < buffer_bytes; ++i) {
                    dst[i] = 0;
                }
                const bool taken = Work::prepare(t);
                if (taken != legal) {
                    continue;
                }
                if (!legal) {
                    ++refused;
                    continue;
                }
                ++accepted;
                if (!Work::trigger() || !wait_complete<Work>()) {
                    continue;
                }
                ++moved;
                if (same(256) && untouched_after(256) && !Work::flag(DmaFlag::fifo_error)) {
                    ++exact;
                }
                Work::stop();
            }
        }
    }
    print(serial, "  table 49 over 48 cells: ", accepted, " accepted, ", refused,
          " refused, ", moved, " run, ", exact, " byte-exact with no FIFO error", crlf);
    // Twelve single-burst cells are always legal; table 49 adds seven for
    // the byte width, three for the half-word and one for the word.
    bench.verdict("every cell of table 49 answers as the table says",
                  accepted + refused == 48u);
    bench.verdict("twenty-three of the forty-eight are legal", accepted == 23u);
    bench.verdict("and every legal one moves its block exactly", exact == accepted);
}

// ---- e  the throughput ladder ---------------------------------------------------

void te_throughput() {
    Block::init();
    fill_source();
    print(serial, "  ", buffer_bytes, " bytes at ", SysClock::hz / 1'000'000u,
          " MHz, memory to memory, source and destination in SRAM:", crlf);

    constexpr DmaBurst bursts[3] = {DmaBurst::single, DmaBurst::incr4, DmaBurst::incr8};
    constexpr DmaWidth widths[3] = {DmaWidth::byte, DmaWidth::half, DmaWidth::word};
    uint8_t ran = 0, exact = 0;
    uint32_t single_of[3] = {0, 0, 0};   ///< the single-beat cycles per width
    uint32_t burst_best[3] = {0, 0, 0};  ///< the best BURST cycles per width
    for (uint8_t w = 0; w < 3u; ++w) {
        for (uint8_t b = 0; b < 3u; ++b) {
            const uint8_t bytes_per = dma_width_bytes(widths[w]);
            const uint16_t items = static_cast<uint16_t>(buffer_bytes / bytes_per);
            // BOTH PORTS BURST TOGETHER, which is the configuration a
            // real block copy would use; table 49's arithmetic is what
            // decides whether the pair exists at all.
            if (!dma_fifo_burst_valid(DmaFifoThreshold::full, bursts[b], widths[w], bursts[b],
                                      widths[w])) {
                continue;
            }
            for (uint16_t i = 0; i < buffer_bytes; ++i) {
                dst[i] = 0;
            }
            DmaTransfer t = mem_to_mem(items, widths[w], bursts[b], DmaFifoThreshold::full);
            t.config.peripheral_burst = bursts[b];
            const uint32_t cycles = timed_block(t);
            Work::stop();
            ++ran;
            const bool ok = cycles != 0u && same(buffer_bytes);
            if (ok) {
                ++exact;
            }
            if (b == 0u) {
                single_of[w] = cycles;
            } else if (cycles != 0u && (burst_best[w] == 0u || cycles < burst_best[w])) {
                burst_best[w] = cycles;
            }
            const uint32_t per100 = cycles == 0u ? 0u : (buffer_bytes * 100u) / cycles;
            print(serial, "    ", static_cast<uint32_t>(bytes_per), "-byte beats, burst ",
                  static_cast<uint32_t>(dma_burst_beats(bursts[b])), ": ", cycles, " cycles (",
                  per100, " bytes per 100 cycles, ", cycles / cycles_per_us, " us)",
                  ok ? "" : "  MISMATCH", crlf);
        }
    }
    bench.verdict("every width and burst moved the block exactly", ran != 0u && exact == ran);
    // The WIDTH is what buys the throughput - a word beat moves four
    // bytes for the same two bus accesses a byte beat pays for.
    bench.verdict("the wider the beat, the fewer the cycles",
                  single_of[2] != 0u && single_of[2] < single_of[1] &&
                      single_of[1] < single_of[0]);
    print(serial, "    bursting both ports against single beats: ", single_of[0], " -> ",
          burst_best[0], " (byte), ", single_of[1], " -> ", burst_best[1], " (half), ",
          single_of[2], " -> ", burst_best[2], " (word) cycles", crlf);
}

// ---- f  the counting rules ------------------------------------------------------

void tf_counting() {
    Block::init();
    fill_source();

    // Table 48: a peripheral port narrower than the memory port packs,
    // and the last access would be incomplete unless the count divides.
    DmaTransfer t = mem_to_mem(64, DmaWidth::byte, DmaBurst::single, DmaFifoThreshold::full);
    t.config.memory_width = DmaWidth::word;
    t.count = 63;
    bench.verdict("table 48: a byte source into word writes refuses a count of 63",
                  !Work::prepare(t));
    t.count = 64;
    const bool taken = Work::prepare(t);
    const bool ran = taken && Work::trigger() && wait_complete<Work>();
    print(serial, "  packed 64 bytes into 16 words, NDTR=", Work::count(), crlf);
    bench.verdict("and accepts 64", taken);
    bench.verdict("the packed block is byte-exact, little-endian", ran && same(64));
    bench.verdict("NDTR counts PERIPHERAL-side items, so it reached zero from 64",
                  Work::count() == 0u);
    Work::stop();

    // Alignment: the address is aligned on the width of the accesses
    // made through it, both ends.
    t = mem_to_mem(32, DmaWidth::half, DmaBurst::single, DmaFifoThreshold::full);
    t.memory = &dst[1];
    bench.verdict("a half-word destination at an odd address is refused",
                  !Work::prepare(t));
    t.memory = &dst[2];
    bench.verdict("and accepted two bytes along", Work::prepare(t));
    Work::stop();

    // The circular note of 10.3.8: under CIRC a memory burst must divide
    // the block. Checked as a refusal, since a circular memory-to-memory
    // stream does not exist to run.
    DmaStreamConfig c{};
    c.direction = DmaDirection::peripheral_to_memory;
    c.circular = true;
    c.memory_increment = true;
    c.memory_burst = DmaBurst::incr8;
    c.use_fifo = true;
    c.fifo_threshold = DmaFifoThreshold::full;
    DmaTransfer ct{};
    ct.peripheral = Console::data_address();
    ct.memory = dst;
    ct.config = c;
    ct.count = 12;
    bench.verdict("10.3.8: a circular block that is not a whole number of memory "
                  "bursts is refused",
                  !Rival::prepare(ct));
    ct.count = 16;
    bench.verdict("and one that is, accepted", Rival::prepare(ct));
    Rival::stop();
}

// ---- g  circular mode, on the console's transmitter ------------------------------

void tg_circular() {
    Block::init();
    Dma<1>::init();
    constexpr uint8_t lap_bytes = 16;
    static const char table[lap_bytes] = {'[', 'c', 'i', 'r', 'c', 'u', 'l', 'a',
                                          'r', ' ', 'l', 'a', 'p', ']', '\r', '\n'};

    quiesce_console();
    Console::dma_transmit(true);

    DmaTransfer t{};
    t.peripheral = Console::data_address();
    t.memory = const_cast<char*>(table);
    t.count = lap_bytes;
    t.config.channel = console_tx_channel;
    t.config.direction = DmaDirection::memory_to_peripheral;
    t.config.circular = true;
    t.config.memory_increment = true;
    const bool taken = ConsoleTxStream::load(t);

    // Five laps at the console's own bit rate: every lap is the same
    // sixteen bytes and the host sees them all. Nothing re-arms anything
    // - the controller reloads NDTR and the address itself.
    uint8_t laps = 0;
    uint16_t half_seen = 0;
    uint32_t spins = 8'000'000u;
    while (laps < 5u && spins-- != 0u) {
        if (ConsoleTxStream::flag(DmaFlag::half) && half_seen == 0u) {
            half_seen = ConsoleTxStream::count();
        }
        if (ConsoleTxStream::flag(DmaFlag::complete)) {
            ConsoleTxStream::clear(DmaFlag::complete | DmaFlag::half);
            ++laps;
        }
    }
    const uint16_t reloaded = ConsoleTxStream::count();
    const bool still_running = ConsoleTxStream::enabled();
    release_console_stream();
    quiesce_console();

    bench.verdict("the circular transfer is accepted", taken);
    print(serial, "  laps=", laps, " NDTR after the wrap=", reloaded,
          " half-transfer flag at NDTR=", half_seen, crlf);
    bench.verdict("five laps ran with nothing re-arming them", laps == 5u);
    bench.verdict("the stream is still enabled after the wrap", still_running);
    bench.verdict("NDTR reloaded itself", reloaded != 0u && reloaded <= lap_bytes);
    bench.verdict("the half-transfer flag falls at the half", half_seen != 0u &&
                                                                  half_seen <= lap_bytes / 2u);
}

// ---- h  the double buffer ---------------------------------------------------------

void th_double_buffer() {
    Block::init();
    Dma<1>::init();
    constexpr uint8_t lap_bytes = 12;
    static const char first[lap_bytes] = {'[', 'b', 'u', 'f', 'f', 'e',
                                          'r', ' ', 'A', ']', '\r', '\n'};
    static const char second[lap_bytes] = {'[', 'b', 'u', 'f', 'f', 'e',
                                           'r', ' ', 'B', ']', '\r', '\n'};
    static const char third[lap_bytes] = {'[', 'b', 'u', 'f', 'f', 'e',
                                          'r', ' ', 'C', ']', '\r', '\n'};

    quiesce_console();
    Console::dma_transmit(true);

    DmaTransfer t{};
    t.peripheral = Console::data_address();
    t.memory = const_cast<char*>(first);
    t.memory1 = const_cast<char*>(second);
    t.count = lap_bytes;
    t.config.channel = console_tx_channel;
    t.config.direction = DmaDirection::memory_to_peripheral;
    t.config.double_buffer = true;
    t.config.memory_increment = true;
    const bool taken = ConsoleTxStream::load(t);
    const bool circular_forced = ConsoleTxStream::circular();
    const bool starts_on_m0 = !ConsoleTxStream::current_target_is_m1();

    // Four laps, reading CT at each: the hardware swaps the target at
    // every end of transaction, which is what makes the mode worth its
    // name.
    uint8_t laps = 0, flips = 0;
    bool target = ConsoleTxStream::current_target_is_m1();
    bool swapped_live = false;
    uint32_t spins = 8'000'000u;
    while (laps < 4u && spins-- != 0u) {
        if (ConsoleTxStream::flag(DmaFlag::complete)) {
            ConsoleTxStream::clear(DmaFlag::complete | DmaFlag::half);
            ++laps;
            const bool now = ConsoleTxStream::current_target_is_m1();
            if (now != target) {
                ++flips;
            }
            target = now;
            if (laps == 2u) {
                // 10.3.9's window: at the completion the target has just
                // changed, so the OTHER half is the one that may be
                // written - and the driver reads CT to know which.
                swapped_live = ConsoleTxStream::set_idle_buffer(const_cast<char*>(third));
            }
        }
    }
    const bool no_error = !ConsoleTxStream::flag(DmaFlag::transfer_error);
    release_console_stream();
    quiesce_console();

    bench.verdict("the double-buffer transfer is accepted", taken);
    bench.verdict("the double buffer forces circular mode", circular_forced);
    bench.verdict("the first lap runs from memory 0", starts_on_m0);
    print(serial, "  laps=", laps, " target flips=", flips, crlf);
    bench.verdict("four laps ran", laps == 4u);
    bench.verdict("the target flipped at every one", flips == 4u);
    bench.verdict("the idle half was replaced under a running stream", swapped_live);
    bench.verdict("and writing the idle half raised no transfer error", no_error);
}

// ---- i  the abort, and 10.3.14's resume --------------------------------------------

void ti_abort() {
    Block::init();
    Dma<1>::init();
    fill_source();

    // First: how long EN takes to come down on a stream that is not
    // running at all, and on one in the middle of a memory-to-memory
    // block. The chapter's claim is that the disable waits for the
    // current transfer AND the FIFO flush.
    const uint32_t v0 = SysTick->VAL;
    const bool idle_down = Work::abort();
    const uint32_t idle_cycles = val_delta(v0, SysTick->VAL);

    (void)Work::load(mem_to_mem(buffer_bytes, DmaWidth::byte, DmaBurst::single,
                                DmaFifoThreshold::full));
    const uint32_t v1 = SysTick->VAL;
    const bool mid_down = Work::abort();
    const uint32_t mid_cycles = val_delta(v1, SysTick->VAL);
    const uint16_t left = Work::count();
    const bool flushed_complete = Work::flag(DmaFlag::complete);
    Work::stop();

    print(serial, "  abort: idle ", idle_cycles, " cycles, mid-block ", mid_cycles,
          " cycles, NDTR left ", left, " of ", buffer_bytes, crlf);
    bench.verdict("EN comes down on an idle stream at once", idle_down);
    bench.verdict("and on a running one, after the flush", mid_down);
    bench.verdict("the disable costs more than the idle one", mid_cycles >= idle_cycles);
    bench.verdict("SxNDTR says how much of the block was left", left != 0u &&
                                                                    left < buffer_bytes);
    bench.verdict("10.3.14: the flush raises the completion flag as well",
                  flushed_complete);

    // Then the resume of 10.3.14, on the console's transmitter because it
    // is slow enough to be interrupted in the middle: send 120 bytes,
    // stop after a millisecond, and finish the rest from where SxNDTR
    // says the block got to.
    constexpr uint16_t run = 120;
    static char line[run];
    for (uint16_t i = 0; i < run; ++i) {
        line[i] = static_cast<char>('0' + (i % 10u));
    }
    line[run - 2] = '\r';
    line[run - 1] = '\n';

    quiesce_console();
    Console::dma_transmit(true);
    DmaTransfer t{};
    t.peripheral = Console::data_address();
    t.memory = line;
    t.count = run;
    t.config.channel = console_tx_channel;
    t.config.direction = DmaDirection::memory_to_peripheral;
    t.config.memory_increment = true;
    const bool started = ConsoleTxStream::load(t);
    const uint32_t mark = Ticker::millis();
    while (Ticker::millis() - mark < 2u) {
    }
    const bool stopped = ConsoleTxStream::abort();
    const uint16_t remaining = ConsoleTxStream::count();
    ConsoleTxStream::clear(DmaFlag::all);

    // The three steps the chapter names: the address forward by what was
    // sent, the count to what is left, the enable again.
    t.memory = &line[run - remaining];
    t.count = remaining;
    const bool resumed = remaining != 0u && ConsoleTxStream::load(t);
    const bool finished = resumed && wait_complete<ConsoleTxStream>();
    release_console_stream();
    quiesce_console();

    print(serial, "  suspended after ", run - remaining, " of ", run,
          " bytes, resumed and finished", crlf);
    bench.verdict("a peripheral-paced block starts", started);
    bench.verdict("and stops in the middle", stopped && remaining != 0u &&
                                                  remaining < run);
    bench.verdict("10.3.14's resume finishes the rest", finished);
}

// ---- j  a transfer error ------------------------------------------------------------

void tj_transfer_error() {
    Block::init();
    fill_source();

    DmaTransfer t = mem_to_mem(64, DmaWidth::byte, DmaBurst::single, DmaFifoThreshold::full);
    t.memory = reinterpret_cast<volatile void*>(unreachable_address);
    const bool taken = Work::prepare(t);
    const bool up = Work::trigger();

    uint32_t spins = 2'000'000u;
    while (!Work::flag(DmaFlag::transfer_error) && spins-- != 0u) {
    }
    const uint32_t flags = Work::flags();
    const bool still_enabled = Work::enabled();
    const uint16_t left = Work::count();

    // With the error standing the stream is dead where it is: its
    // destination has not changed, so an enable faults again on the
    // first beat. What proves the stream RECOVERS is a good block after
    // the flags are cleared.
    (void)Work::abort();
    Work::clear(DmaFlag::all);
    fill_source();
    const bool recovered = Work::load(mem_to_mem(64, DmaWidth::byte, DmaBurst::single,
                                                 DmaFifoThreshold::full)) &&
                           wait_complete<Work>() && same(64);
    Work::stop();

    print(serial, "  writing to ", hex(unreachable_address), ": flags=", hex(flags),
          " EN=", still_enabled ? 1u : 0u, " NDTR=", left, " of 64", crlf);
    bench.verdict("the transfer is accepted - nothing in the driver knows the address "
                  "is unreachable",
                  taken);
    print(serial, "  the enable answered ", up ? "true" : "false",
          " - whether the stream is already dead when the read-back sees it is a race "
          "the core clock decides", crlf);
    bench.verdict("an address no DMA master reaches raises TEIF",
                  (flags & DmaFlag::transfer_error) != 0u);
    bench.verdict("and the hardware clears EN with it", !still_enabled);
    bench.verdict("no FIFO or direct-mode error came with it",
                  (flags & (DmaFlag::fifo_error | DmaFlag::direct_mode_error)) == 0u);
    bench.verdict("SxNDTR says where the block died", left != 0u && left < 64u);
    bench.verdict("a cleared stream takes the next block", recovered);
}

// ---- k  priority arbitration ----------------------------------------------------------

void tk_priority() {
    Block::init();
    for (uint16_t i = 0; i < rival_bytes; ++i) {
        rival_src[i] = static_cast<uint8_t>(i);
        rival_dst[i] = 0;
    }
    fill_source();

    // The same block on two streams of one controller, competing for the
    // same two AHB ports. Only the software half of the arbitration is
    // configurable (10.3.4); the hardware half is the stream index, and
    // stream 0 outranks stream 3 on a tie - which is why the letter runs
    // the pair BOTH WAYS and compares.
    // `gap` puts a few cycles between the two enables. Back to back, the
    // second stream can STALL after its first FIFO fill on one part of the
    // family (measured below, reported and not judged); the timed races
    // keep the gap so that what they measure is the arbitration.
    auto race = [](DmaPriority work, DmaPriority rival, bool gap) {
        DmaTransfer a = mem_to_mem(rival_bytes, DmaWidth::byte, DmaBurst::single,
                                   DmaFifoThreshold::full, work);
        DmaTransfer b = a;
        b.peripheral = rival_src;
        b.memory = rival_dst;
        b.config.priority = rival;
        (void)Work::prepare(a);
        (void)Rival::prepare(b);
        const uint32_t v0 = SysTick->VAL;
        (void)Work::trigger();
        if (gap) {
            for (uint8_t k = 0; k < 8u; ++k) {
                __NOP();
            }
        }
        (void)Rival::trigger();
        uint32_t first = 0, second = 0;
        uint32_t spins = 400'000u;
        while ((first == 0u || second == 0u) && spins-- != 0u) {
            if (first == 0u && Work::flag(DmaFlag::complete)) {
                first = val_delta(v0, SysTick->VAL);
            }
            if (second == 0u && Rival::flag(DmaFlag::complete)) {
                second = val_delta(v0, SysTick->VAL);
            }
        }
        if (second == 0u) {
            print(serial, "    the second stream STALLED: EN=", Rival::enabled() ? 1u : 0u,
                  " NDTR=", Rival::regs().NDTR, " of ", rival_bytes, " flags=",
                  hex(Rival::flags()), crlf);
        }
        Work::stop();
        Rival::stop();
        return static_cast<uint64_t>(first) << 32 | second;
    };

    // What the same block costs with nobody to share the ports with -
    // the yardstick both racers are measured against.
    const uint32_t alone =
        timed_block(mem_to_mem(rival_bytes, DmaWidth::byte, DmaBurst::single,
                               DmaFifoThreshold::full));
    Work::stop();

    // First the back-to-back start, both ways, as a probe: does the second
    // enable's stream run at all?
    print(serial, "  two enables back to back, the low stream second:", crlf);
    const uint64_t bb = race(DmaPriority::very_high, DmaPriority::low, false);
    const bool stalled = static_cast<uint32_t>(bb) == 0u;
    print(serial, "    ", stalled ? "the low stream stalled and never finished"
                                  : "both streams finished", crlf);
    const uint64_t low_high = race(DmaPriority::low, DmaPriority::very_high, true);
    const uint32_t s0_low = static_cast<uint32_t>(low_high >> 32);
    const uint32_t s3_high = static_cast<uint32_t>(low_high);
    const uint64_t high_low = race(DmaPriority::very_high, DmaPriority::low, true);
    const uint32_t s0_high = static_cast<uint32_t>(high_low >> 32);
    const uint32_t s3_low = static_cast<uint32_t>(high_low);

    print(serial, "  ", rival_bytes, " bytes alone: ", alone, " cycles", crlf);
    print(serial, "  with a few cycles between the two enables:", crlf);
    print(serial, "  ", rival_bytes, " bytes each, both streams started together:", crlf);
    print(serial, "    stream 0 low + stream 3 very high: ", s0_low, " / ", s3_high,
          " cycles", crlf);
    print(serial, "    stream 0 very high + stream 3 low: ", s0_high, " / ", s3_low,
          " cycles", crlf);
    bench.verdict("both races finished", s0_low != 0u && s3_high != 0u && s0_high != 0u &&
                                             s3_low != 0u);
    bench.verdict("the very high stream finishes first, whichever index it has",
                  s3_high < s0_low && s0_high < s3_low);
    bench.verdict("and the loser pays for it", s0_low > s0_high && s3_low > s3_high);
    print(serial, "  what the winner pays for the company: ", s3_high, " and ", s0_high,
          " against ", alone, " alone - within the measurement", crlf);
    bench.verdict("both blocks arrived whole", same(rival_bytes));
    bench.verdict("and a stalled stream, if there was one, was recovered by stop()",
                  !Rival::enabled());
}

// ---- l  the console through the DMA engines ---------------------------------------

/// What letters l and m leave behind for the verdicts, which can only be
/// printed once the plain console is back.
struct EngineRun {
    bool up = false;
    uint32_t queued = 0;
    uint32_t millis = 0;
    uint8_t faults = 0;
    bool drained = false;
    uint16_t harvested = 0;
    bool rx_running = false;
    uint8_t hw_overruns = 0;    ///< ORE: a byte the receiver lost in silicon
    uint8_t rx_overruns = 0;    ///< laps the consumer did not keep up with
    uint8_t line_errors = 0;    ///< NE, FE, PE: each costs the byte in DR
    uint16_t first_gap = 0;     ///< where the echo first left the pattern
    uint32_t laps = 0;          ///< the receive engine's completions, at the end
    uint16_t rx_remaining = 0;  ///< the receive stream's SxNDTR, at the end
    bool circular = false;      ///< the receive stream still circular, at the end
    uint32_t ruler = 0;         ///< an empty pair of SysTick reads, core cycles
    uint32_t harvest_idle = 0;  ///< one harvest() with nothing new, the ruler included
    uint32_t harvest_busy = 0;  ///< one harvest() with bytes waiting, the ruler included
};

/// Which transport owns the port right now, for the USART's and the two
/// stream vectors to dispatch on: 0 the plain interrupt-driven one, 1 the
/// engined console, 2 the engined half-duplex loop, 3 letter u's.
volatile uint8_t engined_console = 0;
/// The edges the engined transports' vectors reported: what a program
/// would post RxActivity on.
volatile uint32_t engined_edges = 0;

/// Wait for `Port`'s transmit ring to drain and its last frame to leave
/// the shifter, then `ms` more for the line to settle.
template <typename Port>
void wait_line(uint32_t ms) {
    uint32_t spins = 4'000'000u;
    while (!Port::tx_idle() && spins-- != 0u) {
    }
    spins = 400'000u;
    while (!Console::tx_complete() && spins-- != 0u) {
    }
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 <= ms) {
    }
}

/// What one harvest() costs on the port `Port` holds - the least of three
/// tries, once with nothing new and once with eight bytes waiting for the
/// consumer - with the ruler's own pair beside it. Run on the half-duplex
/// loop, whose transmitter is the stimulus; the port must be idle.
template <typename Port>
void time_harvest(EngineRun& r) {
    static const uint8_t poke[8] = {'0', '1', '2', '3', '4', '5', '6', '7'};
    uint8_t byte = 0;
    r.ruler = 0xFFFF'FFFFu;
    r.harvest_idle = 0xFFFF'FFFFu;
    r.harvest_busy = 0xFFFF'FFFFu;
    for (uint8_t k = 0; k < 3u; ++k) {
        while (Port::read_byte(byte)) {
        }
        (void)Port::harvest();
        uint32_t v0 = SysTick->VAL;
        uint32_t d = val_delta(v0, SysTick->VAL);
        r.ruler = d < r.ruler ? d : r.ruler;
        v0 = SysTick->VAL;
        (void)Port::harvest();
        d = val_delta(v0, SysTick->VAL);
        r.harvest_idle = d < r.harvest_idle ? d : r.harvest_idle;
        (void)Port::write_bulk(std::span<const uint8_t>(poke, sizeof(poke)));
        wait_line<Port>(2);
        v0 = SysTick->VAL;
        (void)Port::harvest();
        d = val_delta(v0, SysTick->VAL);
        r.harvest_busy = d < r.harvest_busy ? d : r.harvest_busy;
    }
    while (Port::read_byte(byte)) {
    }
}

/// Hand the port to `Port` (an engined Uart type), send `block` bytes
/// through the transmit stream, harvest for `rx_window_ms`, and give it
/// back. Nothing may print between the two swaps: the verdicts wait for
/// the plain console to come home. With `timed`, what one harvest()
/// costs is measured before the port goes back.
template <typename Port>
EngineRun run_engined(uint8_t which, uint16_t block, uint32_t rx_window_ms, bool timed = false) {
    EngineRun r{};
    static uint8_t line[64];
    for (uint16_t i = 0; i < sizeof(line); ++i) {
        line[i] = static_cast<uint8_t>('a' + (i % 26u));
    }
    line[sizeof(line) - 2] = '\r';
    line[sizeof(line) - 1] = '\n';

    quiesce_console();
    Serial::release();
    engined_console = which;
    r.up = Port::init(clock, 115200);
    if (!r.up) {
        engined_console = 0;
        (void)Serial::init(clock, 115200);
        return r;
    }
    r.rx_running = !ConsoleRxEngine::idle();
    Port::clear_errors();   // what the handover itself left behind is not the run's

    // THE CONSUMER DRAINS AS IT SENDS, the edge being the vectors' to
    // report. On the half-duplex loop everything sent comes straight
    // back, so the two directions are busy at once.
    const uint32_t t0 = Ticker::millis();
    for (;;) {
        if (r.queued < block) {
            r.queued += Port::write_bulk(std::span<const uint8_t>(line, sizeof(line)));
        }
        uint8_t taken = 0;
        while (Port::read_byte(taken)) {
            if (r.first_gap == 0u && taken != line[r.harvested % sizeof(line)]) {
                r.first_gap = static_cast<uint16_t>(r.harvested + 1u);
            }
            ++r.harvested;
        }
        if (r.queued >= block && Port::tx_idle()) {
            break;
        }
        if (Ticker::millis() - t0 > 3000u) {
            break;
        }
    }
    uint32_t spins = 400'000u;
    while (!Console::tx_complete() && spins-- != 0u) {
    }
    r.millis = Ticker::millis() - t0;
    r.drained = Port::tx_idle();

    // The tail: whatever is still in flight after the last byte left.
    const uint32_t r0 = Ticker::millis();
    while (Ticker::millis() - r0 <= rx_window_ms) {
        uint8_t byte = 0;
        while (Port::read_byte(byte)) {
            if (r.first_gap == 0u && byte != line[r.harvested % sizeof(line)]) {
                r.first_gap = static_cast<uint16_t>(r.harvested + 1u);
            }
            ++r.harvested;
        }
    }
    r.faults = Port::dma_faults();
    r.hw_overruns = Port::hw_overruns();
    r.rx_overruns = Port::rx_overruns();
    r.line_errors = static_cast<uint8_t>(Port::noise_errors() + Port::frame_errors() +
                                         Port::parity_errors());
    r.laps = ConsoleRxEngine::laps();
    r.rx_remaining = ConsoleRxStream::count();
    r.circular = ConsoleRxStream::circular();
    if (timed) {
        time_harvest<Port>(r);
    }

    Port::release();
    engined_console = 0;
    (void)Serial::init(clock, 115200);
    return r;
}

void tl_engines() {
    Block::init();
    Dma<1>::init();
    const EngineRun r = run_engined<DmaSerial>(1, 512, 0);

    bench.verdict("the port comes up with both engine slots filled", r.up);
    bench.verdict("the receive engine is running from the first byte", r.rx_running);
    print(serial, "  ", r.queued, " bytes through the transmit stream in ", r.millis,
          " ms (", r.millis == 0u ? 0u : r.queued * 1000u / r.millis,
          " bytes/s at 115200 baud), dma faults ", r.faults, crlf);
    bench.verdict("the whole block was queued", r.queued >= 512u);
    bench.verdict("the ring drained through the stream", r.drained);
    bench.verdict("no block was thrown away", r.faults == 0u);
    // A byte is ten bits on the wire at 115200, so the block's own
    // length says how long it must have taken: the time measures the
    // LINE and not the ring, which is the whole claim of a DMA console.
    const uint32_t expected = r.queued * 10'000u / 115200u;
    print(serial, "  the line's own time for ", r.queued, " bytes is ", expected, " ms", crlf);
    bench.verdict("the time is the line's, not the CPU's",
                  r.millis + 5u >= expected && r.millis <= expected + 5u);
}

// ---- m  the circular receive, on the transmitter's own echo ---------------------------

void tm_receive() {
    Block::init();
    Dma<1>::init();
    // SINGLE-WIRE HALF DUPLEX IS THE WIRE THIS BOARD HAS NOT GOT: with
    // HDSEL the receiver listens on the transmit pad, so a block sent by
    // the transmit stream arrives back at the receive stream. The host
    // sees the block too - the pad is still the console's. 128 bytes
    // through a ring of 64 is two whole laps.
    const EngineRun r = run_engined<LoopSerial>(2, 128, 20, true);

    print(serial, "  sent ", r.queued, " byte(s) in ", r.millis, " ms and read ", r.harvested,
          " back through the receive stream; ", r.laps, " lap(s), SxNDTR ", r.rx_remaining,
          "; ", r.rx_overruns, " lap(s) the consumer missed, ", r.hw_overruns,
          " overrun(s) in silicon, ", r.line_errors, " line error(s), ", r.faults,
          " fault(s)", crlf);
    bench.verdict("the port comes up in half duplex with both slots filled", r.up);
    bench.verdict("the receive engine is running from the first byte", r.rx_running);
    bench.verdict("the transmit stream drained the ring", r.drained);
    // THE ECHO ARRIVES WHOLE: the stream writes the ring lap after lap and
    // is never re-armed, so there is no moment between two runs at which
    // a byte can arrive with nobody serving the receiver.
    bench.verdict("every byte of the echo arrives, in order", r.harvested == r.queued &&
                                                                  r.first_gap == 0u);
    bench.verdict("the laps are the stream's own: two of 64 bytes for 128",
                  r.laps == r.queued / rx_ring_bytes);
    bench.verdict("SxNDTR was reloaded by the hardware at the last wrap",
                  r.rx_remaining == rx_ring_bytes - (r.queued % rx_ring_bytes));
    bench.verdict("the stream is still circular: nothing re-armed it", r.circular);
    bench.verdict("no lap missed, no overrun, no line error",
                  r.rx_overruns == 0u && r.hw_overruns == 0u && r.line_errors == 0u);
    bench.verdict("no block was thrown away", r.faults == 0u);
    print(serial, "  one harvest() costs ", r.harvest_idle - r.ruler, " cycles with nothing new and ",
          r.harvest_busy - r.ruler, " with bytes waiting (the least of three; the ruler's own ",
          r.ruler, " taken off)", crlf);
}

// ---- n  the circular receive's two edges ----------------------------------------------

/// The stream letter n sends: position `k` of it is a printable
/// character, so what the host sees on the shared pad is legible.
constexpr uint8_t loop_byte(uint16_t k) { return static_cast<uint8_t>(0x20u + k % 95u); }

/// Positions `from` .. `from + count - 1` of that stream through the
/// half-duplex loop, and the line settled after them.
void loop_send(uint16_t from, uint16_t count) {
    if (count == 0u) {
        wait_line<LoopSerial>(2);
        return;
    }
    static uint8_t block[192];
    for (uint16_t i = 0; i < count && i < sizeof(block); ++i) {
        block[i] = loop_byte(static_cast<uint16_t>(from + i));
    }
    uint16_t sent = 0;
    const uint32_t t0 = Ticker::millis();
    while (sent < count && Ticker::millis() - t0 < 1000u) {
        sent = static_cast<uint16_t>(
            sent + LoopSerial::write_bulk(std::span<const uint8_t>(block + sent, count - sent)));
    }
    wait_line<LoopSerial>(2);
}

/// Every byte the loop holds, judged against the stream from position
/// `from` on; the count read, and in `in_order` whether each was the one
/// expected.
uint16_t loop_drain(uint16_t from, bool& in_order) {
    uint16_t got = 0;
    in_order = true;
    uint8_t byte = 0;
    while (LoopSerial::read_byte(byte)) {
        if (byte != loop_byte(static_cast<uint16_t>(from + got))) {
            in_order = false;
        }
        ++got;
    }
    return got;
}

void tn_receive_edges() {
    Block::init();
    Dma<1>::init();
    quiesce_console();
    Serial::release();
    engined_console = 2;
    const bool up = LoopSerial::init(clock, 115200);
    LoopSerial::clear_errors();

    // THE EDGE, FROM THE VECTORS. The USART's idle line and a burst's
    // first frame, and the lap's marks, report bytes once per idle-to-busy
    // transition of the consumer: once, not again to a consumer that has
    // not drained, and again once it has. harvest() asks the same gate
    // from the consumer's side, and finds it closed once a vector told.
    const uint32_t k0 = engined_edges;
    loop_send(0, 16);
    const bool e_first = engined_edges != k0;
    const bool e_harvest = LoopSerial::harvest();
    const uint32_t k1 = engined_edges;
    loop_send(16, 0);   // the line settled again: nothing new
    const bool e_again = engined_edges != k1;
    bool order1 = false;
    const uint16_t d1 = loop_drain(0, order1);
    const bool e_empty = LoopSerial::harvest();
    const uint32_t k2 = engined_edges;
    loop_send(16, 16);
    const bool e_next = engined_edges != k2;
    bool order2 = false;
    const uint16_t d2 = loop_drain(16, order2);

    // THE LAP SLEPT THROUGH. 160 bytes go by with nobody looking: the
    // ring of 64 is written over two and a half times, and only the lap
    // count can say so. The first look counts it once and SKIPS to the
    // stream's head - skip rather than tear - and what arrives after it
    // reads whole.
    const uint32_t laps0 = ConsoleRxEngine::laps();
    const uint32_t k_lap = engined_edges;
    loop_send(32, 160);
    const bool e_lap = engined_edges != k_lap;
    const uint32_t laps1 = ConsoleRxEngine::laps();
    const uint32_t pending = LoopSerial::rx_pending();
    const uint8_t missed = LoopSerial::rx_overruns();
    loop_send(192, 32);
    bool order3 = false;
    const uint16_t d3 = loop_drain(192, order3);
    const uint8_t missed_after = LoopSerial::rx_overruns();
    const uint8_t faults = LoopSerial::dma_faults();
    const uint8_t hw = LoopSerial::hw_overruns();

    LoopSerial::release();
    engined_console = 0;
    (void)Serial::init(clock, 115200);

    print(serial, "  the edge: ", e_first ? 1u : 0u, " at the first bytes, ", e_harvest ? 1u : 0u,
          " from harvest() after it, ", e_again ? 1u : 0u, " again undrained, ", e_empty ? 1u : 0u,
          " from harvest() drained and empty, ", e_next ? 1u : 0u, " at the next bytes; drained ",
          d1, " and ", d2, crlf);
    bench.verdict("the port comes up in half duplex", up);
    bench.verdict("the vectors report the first bytes", e_first);
    bench.verdict("and harvest() asked after them answers false: the gate is one", !e_harvest);
    bench.verdict("no second edge to a consumer that has not drained", !e_again);
    bench.verdict("harvest() reports no empty ring", !e_empty);
    bench.verdict("the vectors report the next bytes once the consumer has drained", e_next);
    bench.verdict("both drains read their 16 bytes in order",
                  d1 == 16u && d2 == 16u && order1 && order2);
    print(serial, "  a consumer asleep for ", laps1 - laps0, " lap(s): ", pending,
          " byte(s) offered at its first look, ", missed, " lap(s) counted missed, then ",
          d3, " byte(s) read after the skip", crlf);
    bench.verdict("160 bytes through a ring of 64 from byte 32 on: three laps went by",
                  laps1 - laps0 == 3u);
    bench.verdict("the first look counts the lap once and skips to the stream's head",
                  missed == 1u && pending == 0u);
    bench.verdict("what arrives after the skip is read whole and in order",
                  d3 == 32u && order3 && missed_after == 1u);
    bench.verdict("the sleeping consumer was told once, at the stream's first bytes", e_lap);
    bench.verdict("no overrun in silicon, no fault", hw == 0u && faults == 0u);
}

// ---- u  brio stress: the host's stream into the receive engine (OUTSIDE z) ------------
//
// brio stress (cli/bench/stress.py) is the host end: the board prints one
// "HOST sink mode baud format window count" line, the script moves its own
// port to that rate after a settle of its own and pumps its xorshift for
// the window less half a second, then goes quiet. The board runs the
// circular receive at the announced rate and judges every byte against
// the same generator. THE CONSUMER IS PACED BY THE TICK - one look a
// millisecond, a TimeEvent's pace - because that is what the receive
// engine has to survive in a program: a consumer that is not always
// looking.

uint32_t lfsr_state = 0x12345678u;

/// brio stress's generator: three shifts, the low byte a step.
constexpr uint32_t lfsr_step(uint32_t s) {
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
}

struct SinkLeg {
    uint32_t baud = 0;
    bool up = false;
    uint32_t in = 0;         ///< bytes received
    uint32_t gaps = 0;       ///< places the stream skipped positions
    uint32_t lost = 0;       ///< positions skipped there, in all
    uint32_t bad = 0;        ///< bytes matching no position near the expected one
    uint32_t first_gap = 0;  ///< which received byte first left the stream, 0 for none
    uint32_t looks = 0;      ///< consumer turns: at most one a millisecond, after an edge
    uint32_t seen = 0;       ///< the vectors' edge count at the last turn
    uint8_t rx_overruns = 0;
    uint8_t hw_overruns = 0;
    uint8_t line_errors = 0;
    uint8_t faults = 0;
};

/// A run of received bytes against the host's stream. A byte that is not
/// the next position's is looked for among the next 255, confirmed by the
/// byte after it where the run has one: the positions stepped over were
/// LOST, one GAP, and the judge carries on from where the pair fits. A
/// byte that fits nowhere near is BAD and stands in for the position.
void judge(SinkLeg& leg, const uint8_t* run, uint32_t n) {
    constexpr uint32_t horizon = 255;
    for (uint32_t i = 0; i < n; ++i) {
        ++leg.in;
        const uint32_t s = lfsr_step(lfsr_state);
        if (static_cast<uint8_t>(s) == run[i]) {
            lfsr_state = s;
            continue;
        }
        if (leg.first_gap == 0u) {
            leg.first_gap = leg.in;
        }
        uint32_t t = s;
        bool found = false;
        for (uint32_t k = 1; k <= horizon; ++k) {
            t = lfsr_step(t);
            if (static_cast<uint8_t>(t) == run[i] &&
                (i + 1u == n || static_cast<uint8_t>(lfsr_step(t)) == run[i + 1u])) {
                leg.lost += k;
                ++leg.gaps;
                lfsr_state = t;
                found = true;
                break;
            }
        }
        if (!found) {
            ++leg.bad;
            lfsr_state = s;
        }
    }
}

// ---- p's load: copies kept busy on DMA2 beside the console's ring ----------------
//
// Four copy engines on DMA2's streams 3 to 6 - above stream 2, the ring's
// on the parts whose console is USART1, so a tie of levels is the ring's
// - each restarted the moment its last block ends, 4 KB of words a block.

alignas(16) uint32_t load_src[4][1024];
alignas(16) uint32_t load_dst[4][1024];
using LoadA = DmaCopyEngine<2, 3>;
using LoadB = DmaCopyEngine<2, 4>;
using LoadC = DmaCopyEngine<2, 5>;
using LoadD = DmaCopyEngine<2, 6>;
uint8_t load_copies = 0;      ///< how many of the four a sink leg keeps busy
uint32_t load_blocks = 0;     ///< blocks they started in the leg

template <typename E>
void keep_loaded(uint8_t i) {
    if (!E::busy() && E::copy(load_dst[i], load_src[i], 1024u)) {
        ++load_blocks;
    }
}

void arm_load(DmaPriority level) {
    LoadA::arm(level);
    LoadB::arm(level);
    LoadC::arm(level);
    LoadD::arm(level);
    load_blocks = 0;
}

void run_load() {
    if (load_copies >= 1u) { keep_loaded<LoadA>(0); }
    if (load_copies >= 2u) { keep_loaded<LoadB>(1); }
    if (load_copies >= 3u) { keep_loaded<LoadC>(2); }
    if (load_copies >= 4u) { keep_loaded<LoadD>(3); }
}

void end_load() {
    while (LoadA::busy() || LoadB::busy() || LoadC::busy() || LoadD::busy()) {
    }
    load_copies = 0;
}

template <typename Port>
SinkLeg sink_leg(uint32_t baud, uint32_t window_ms) {
    SinkLeg leg{};
    leg.baud = baud;
    print(serial, "HOST sink 3 ", baud, " 8N1 ", window_ms, " 0", crlf);
    const uint32_t t_line = Ticker::millis();
    quiesce_console();
    Serial::release();
    engined_console = 3;
    leg.up = Port::init(clock, baud);
    if (leg.up) {
        // The host settles 140 ms before it pumps: what comes before that
        // is the switch's, not the stream's.
        while (Ticker::millis() - t_line < 100u) {
        }
        uint8_t junk = 0;
        while (Port::read_byte(junk)) {
        }
        Port::clear_errors();
        leg.seen = engined_edges - 1u;   // the first turn looks
        lfsr_state = 0x12345678u;
        uint8_t chunk[64];
        uint32_t last = Ticker::millis();
        while (Ticker::millis() - t_line < window_ms) {
            run_load();
            const uint32_t now = Ticker::millis();
            if (now == last) {
                continue;
            }
            last = now;
            if (engined_edges == leg.seen) {
                continue;   // no edge since the last look: nothing to read
            }
            leg.seen = engined_edges;
            ++leg.looks;
            for (;;) {
                const uint32_t n = Port::read_bulk(std::span<uint8_t>(chunk, sizeof(chunk)));
                if (n == 0u) {
                    break;
                }
                judge(leg, chunk, n);
            }
        }
        leg.rx_overruns = Port::rx_overruns();
        leg.hw_overruns = Port::hw_overruns();
        leg.line_errors = static_cast<uint8_t>(Port::noise_errors() + Port::frame_errors() +
                                               Port::parity_errors());
        leg.faults = Port::dma_faults();
        end_load();
        Port::release();
    }
    engined_console = 0;
    (void)Serial::init(clock, 115200);
    // A SILENCE BEFORE THE REPORT, the script's contract: it reads until
    // the wire has been quiet, and whatever it still had in flight lands
    // in the plain console's ring - thrown away here, it is no letter.
    const uint32_t q0 = Ticker::millis();
    while (Ticker::millis() - q0 < 500u) {
    }
    uint8_t junk = 0;
    while (Serial::read_byte(junk)) {
    }
    Serial::clear_errors();
    return leg;
}

void tu_stress() {
    print(serial, "  this letter needs brio stress on the other end of the console:", crlf,
          "  brio stress --letters u --board <board>", crlf);
    Block::init();
    Dma<1>::init();
    constexpr uint32_t rungs[] = {115200, 460800, 921600};
    constexpr uint32_t window_ms = 1500;
    bool all_whole = true;
    bool all_fed = true;
    for (const uint32_t baud : rungs) {
        const SinkLeg leg = sink_leg<StressSerial>(baud, window_ms);
        print(serial, "  sink at ", baud, ": ", leg.in, " byte(s) in, ", leg.gaps, " gap(s) of ",
              leg.lost, " lost in all, ", leg.bad, " bad, the first gap at byte ", leg.first_gap,
              "; ", leg.looks, " look(s), laps missed ", leg.rx_overruns, ", ORE ",
              leg.hw_overruns, ", line errors ", leg.line_errors, ", faults ", leg.faults, crlf);
        // The bridge carries what it carries: a quarter of the line's own
        // rate over the pump's second is enough to call the leg fed.
        if (!leg.up || leg.in < baud / 10u / 4u) {
            all_fed = false;
        }
        if (leg.gaps != 0u || leg.bad != 0u || leg.first_gap != 0u || leg.rx_overruns != 0u ||
            leg.hw_overruns != 0u || leg.line_errors != 0u || leg.faults != 0u) {
            all_whole = false;
        }
    }
    bench.verdict("the host fed every rung (brio stress on the other end)", all_fed);
    bench.verdict("and every byte it sent arrived, in order, at every rung: none lost "
                  "between laps, no lap missed, no overrun in silicon",
                  all_whole);
}

// ---- p  brio stress: the console's ring beside four copies on DMA2 (OUTSIDE z) -----
//
// Letter u's sink, at 115200 and at the bridge's 921600, with four copies
// kept busy on DMA2 the whole window: at their own level (low, the copy
// engine's) and at very_high. The console's ring arms at its default
// level. Where the console is USART1 (the F429 and the F411) the ring is
// DMA2's stream 2 and shares the arbiter with the copies; where it is
// USART2 or USART3 (the F446, the F469) it is DMA1's, and the copies reach
// it only through the bus matrix. Every byte the host sent is judged.
//
//   brio stress --letters p --board <board>

void tp_ring_beside_copies() {
    print(serial, "  this letter needs brio stress on the other end of the console:", crlf,
          "  brio stress --letters p --board <board>", crlf);
    for (uint32_t k = 0; k < 4u; ++k) {
        for (uint32_t i = 0; i < 1024u; ++i) {
            load_src[k][i] = (i ^ k) * 2246822519u;
        }
    }
    Block::init();
    Dma<1>::init();
    constexpr uint32_t rungs[] = {115200, 921600};
    constexpr uint32_t window_ms = 1500;
    static constexpr DmaPriority levels[] = {DmaPriority::low, DmaPriority::very_high};
    bool all_whole = true;
    bool all_fed = true;
    bool all_loaded = true;
    for (const uint32_t baud : rungs) {
        for (const DmaPriority level : levels) {
            arm_load(level);
            load_copies = 4;
            const SinkLeg leg = sink_leg<StressSerial>(baud, window_ms);
            print(serial, "  sink at ", baud, " beside four copies at ",
                  level == DmaPriority::low ? "low" : "very_high", " (", load_blocks,
                  " blocks of 4 KB): ", leg.in, " byte(s) in, ", leg.gaps, " gap(s) of ",
                  leg.lost, " lost, ", leg.bad, " bad; laps missed ", leg.rx_overruns,
                  ", ORE ", leg.hw_overruns, ", line errors ", leg.line_errors, ", faults ",
                  leg.faults, crlf);
            if (!leg.up || leg.in < baud / 10u / 4u) {
                all_fed = false;
            }
            if (load_blocks < 100u) {
                all_loaded = false;
            }
            if (leg.gaps != 0u || leg.bad != 0u || leg.first_gap != 0u ||
                leg.rx_overruns != 0u || leg.hw_overruns != 0u || leg.line_errors != 0u ||
                leg.faults != 0u) {
                all_whole = false;
            }
        }
    }
    bench.verdict("the host fed every leg (brio stress on the other end)", all_fed);
    bench.verdict("the four copies ran the whole window, block after block", all_loaded);
    bench.verdict("THE RING AT ITS DEFAULT LEVEL LOSES NOTHING beside four copies on DMA2, at "
                  "their own level and at very_high: every byte in order, no overrun",
                  all_whole);
}

// ---- r  a released requester (the release contract, no wire) -----------------------
//
// docs/design/dma.md's release contract asks whether a peripheral's DMA
// request, once raised, outlives the peripheral's release. On this
// controller a request line reaches a stream only through the CHSEL of
// the one or two cells the request mapping gives it, so a stray can only
// reach the NEXT OWNER OF THE SAME CELL. The letter measures it with no
// wire: a requester brought up with its request standing and no stream
// listening, released one of five ways, then the INCOMING owner - a
// stream on the requester's own cell, four items between memory and the
// requester's own register, the direction its owner would use - enabled
// for 2000 cycles. Every item it moves is a request that outlived the
// release. Where the requester's clock was gated it is opened again
// afterwards, no reset, and the count read once more.
//
//   A  the contract: block disabled, RESET pulsed, clock gated
//   B  the DMA enable cleared, block disabled, clock gated (no reset)
//   C  the DMA enable left set, block disabled, clock gated (no reset)
//   D  the DMA enable cleared, block disabled, clock left on (no reset)
//   E  the control: nothing released - the request is live
//
// The requesters, none of them with a pad: USART6's transmit (TXE behind
// DMAT), SPI1's transmit (TXE behind TXDMAEN, a host under software
// select), TIM1's update (one UG behind UDE, the counter never started)
// and ADC1's end of conversion (one software-started conversion behind
// CR2.DMA). The ADC is the one whose reset line is shared - one line
// resets every converter and the common block (RM0090 6.3.9, "common to
// all ADCs") - so its driver's release() clears CR2 with the clock on and
// gates it, and that release is measured as a sixth way, F; for the
// other three the driver's release is the contract, A.
//
// MEASURED on the STM32F446RE: a cleared DMA enable withdraws a raised
// request, the clock on or gated, but a gated clock does NOT - with the
// enable left set the stream on the requester's cell takes an item while
// the requester's clock is still gated, and more when it returns.

enum class Release : uint8_t { contract, enable_cleared_gated, enable_set_gated,
                               enable_cleared_clocked, live, driver };

struct Handover {
    uint16_t moved = 0;           ///< items the incoming owner moved
    uint16_t moved_ungated = 0;   ///< and after the clock was opened again (B, C, F)
};

constexpr DmaPlacement u6_cell = usart_dma_placements(6, true).at[0];
constexpr DmaPlacement spi1_cell = spi_dma_placements(1, true).at[0];
constexpr DmaPlacement adc1_cell = adc_dma_placements(1).at[0];
/// TIM1_UP's cell, the same in the three tables (RM0090 table 44, RM0390
/// table 29, RM0383 table 28): the timers' rows are not in the reserve.
constexpr DmaPlacement tim1_up_cell{2, 5, 6};
static_assert(u6_cell.controller == 2u && spi1_cell.controller == 2u &&
                  adc1_cell.controller == 2u,
              "the incoming owners below are DMA2's streams");
static_assert(u6_cell.stream != spi1_cell.stream && u6_cell.stream != adc1_cell.stream &&
                  u6_cell.stream != tim1_up_cell.stream &&
                  spi1_cell.stream != adc1_cell.stream &&
                  spi1_cell.stream != tim1_up_cell.stream &&
                  adc1_cell.stream != tim1_up_cell.stream,
              "four requesters, four streams");

alignas(4) volatile uint16_t handover_items[4];

void spin_cycles(uint32_t n) {
    const uint32_t t0 = CycleCounter::now();
    while (CycleCounter::now() - t0 < n) {
    }
}

/// The incoming owner: a stream on `cell`, four items of `width` between
/// memory and the requester's register `data`, given 2000 cycles.
template <uint8_t s>
uint16_t incoming(uint8_t channel, volatile void* data, DmaDirection direction, DmaWidth width) {
    using In = DmaStream<2, s>;
    for (auto& v : handover_items) {
        v = 0;
    }
    In::stop();
    In::clear(DmaFlag::all);
    DmaTransfer t{};
    t.peripheral = data;
    t.memory = handover_items;
    t.count = 4;
    t.config.channel = channel;
    t.config.direction = direction;
    t.config.memory_increment = true;
    t.config.peripheral_width = width;
    t.config.memory_width = width;
    (void)In::load(t);
    spin_cycles(2000u);
    return static_cast<uint16_t>(4u - In::count());
}
template <uint8_t s>
uint16_t incoming_moved() {
    return static_cast<uint16_t>(4u - DmaStream<2, s>::count());
}
template <uint8_t s>
void incoming_stop() {
    DmaStream<2, s>::stop();
    DmaStream<2, s>::clear(DmaFlag::all);
}

bool gated(Release how) {
    return how == Release::enable_cleared_gated || how == Release::enable_set_gated ||
           how == Release::driver;
}

Handover usart6_handover(Release how) {
    using U = Usart<6>;
    U::bus_clock(true);
    U::reset();
    (void)U::configure(UartFormat{}, 781);
    U::dma_transmit(true);
    U::transmitter(true);
    U::enable(true);
    spin_cycles(SysClock::hz / 10000u);
    switch (how) {
        case Release::contract:
        case Release::driver:
            U::enable(false);
            U::reset();
            U::bus_clock(false);
            break;
        case Release::enable_cleared_gated:
            U::enable(false);
            U::dma_transmit(false);
            U::bus_clock(false);
            break;
        case Release::enable_set_gated:
            U::enable(false);
            U::bus_clock(false);
            break;
        case Release::enable_cleared_clocked:
            U::enable(false);
            U::dma_transmit(false);
            break;
        case Release::live:
            break;
    }
    Handover h{};
    h.moved = incoming<u6_cell.stream>(u6_cell.channel, U::data_address(),
                                       DmaDirection::memory_to_peripheral, DmaWidth::byte);
    if (gated(how)) {
        U::bus_clock(true);
        spin_cycles(2000u);
    }
    h.moved_ungated = incoming_moved<u6_cell.stream>();
    incoming_stop<u6_cell.stream>();
    U::bus_clock(true);
    U::enable(false);
    U::reset();
    U::bus_clock(false);
    return h;
}

Handover spi1_handover(Release how) {
    using S1 = Spi<1>;
    S1::bus_clock(true);
    S1::reset();
    (void)S1::configure(SpiConfig{.dma_transmit = true});
    S1::enable();
    spin_cycles(SysClock::hz / 10000u);
    switch (how) {
        case Release::contract:
        case Release::driver:
            (void)S1::disable();
            S1::reset();
            S1::bus_clock(false);
            break;
        case Release::enable_cleared_gated:
            (void)S1::disable();
            S1::dma_transmit(false);
            S1::bus_clock(false);
            break;
        case Release::enable_set_gated:
            (void)S1::disable();
            S1::bus_clock(false);
            break;
        case Release::enable_cleared_clocked:
            (void)S1::disable();
            S1::dma_transmit(false);
            break;
        case Release::live:
            break;
    }
    Handover h{};
    h.moved = incoming<spi1_cell.stream>(spi1_cell.channel, S1::data_address(),
                                         DmaDirection::memory_to_peripheral, DmaWidth::byte);
    if (gated(how)) {
        S1::bus_clock(true);
        spin_cycles(2000u);
    }
    h.moved_ungated = incoming_moved<spi1_cell.stream>();
    incoming_stop<spi1_cell.stream>();
    S1::bus_clock(true);
    (void)S1::disable();
    S1::reset();
    S1::bus_clock(false);
    return h;
}

/// TIM1's update request: UDE and one software update, the counter never
/// started - one request raised, none after it. The incoming owner writes
/// zeros through TIMx_DMAR, which with DCR at its reset value lands in CR1:
/// the counter stays stopped.
Handover tim1_handover(Release how) {
    using T1 = Tim<1>;
    T1::init();
    (void)T1::configure(TimConfig{.prescaler = 0, .period = 0xFFFFu});
    T1::interrupts(T1::update_dma, true);
    T1::update();
    spin_cycles(64u);
    switch (how) {
        case Release::contract:
        case Release::driver:
            T1::release();
            break;
        case Release::enable_cleared_gated:
            T1::interrupts(T1::update_dma, false);
            T1::bus_clock(false);
            break;
        case Release::enable_set_gated:
            T1::bus_clock(false);
            break;
        case Release::enable_cleared_clocked:
            T1::interrupts(T1::update_dma, false);
            break;
        case Release::live:
            break;
    }
    Handover h{};
    h.moved = incoming<tim1_up_cell.stream>(tim1_up_cell.channel, T1::dmar_address(),
                                            DmaDirection::memory_to_peripheral, DmaWidth::half);
    if (gated(how)) {
        T1::bus_clock(true);
        spin_cycles(2000u);
    }
    h.moved_ungated = incoming_moved<tim1_up_cell.stream>();
    incoming_stop<tim1_up_cell.stream>();
    T1::bus_clock(true);
    T1::release();
    return h;
}

/// ADC1's request: one software-started conversion behind CR2.DMA (DDS
/// clear: no request after the stream's last item, and none was served).
/// The incoming owner reads DR into memory, a half-word an item.
Handover adc1_handover(Release how) {
    using A1 = Adc<1>;
    AdcCommon::reset();
    (void)A1::init(clock, AdcConfig{.dma = true});
    A1::start();
    spin_cycles(SysClock::hz / 100000u);   // ten microseconds: one conversion and then some
    switch (how) {
        case Release::contract:
            A1::release();
            AdcCommon::reset();
            break;
        case Release::driver:
            A1::release();
            break;
        case Release::enable_cleared_gated:
            (void)A1::configure(AdcConfig{});
            A1::bus_clock(false);
            break;
        case Release::enable_set_gated:
            A1::bus_clock(false);
            break;
        case Release::enable_cleared_clocked:
            (void)A1::configure(AdcConfig{});
            break;
        case Release::live:
            break;
    }
    Handover h{};
    h.moved = incoming<adc1_cell.stream>(adc1_cell.channel, A1::data_address(),
                                         DmaDirection::peripheral_to_memory, DmaWidth::half);
    if (gated(how)) {
        A1::bus_clock(true);
        spin_cycles(2000u);
    }
    h.moved_ungated = incoming_moved<adc1_cell.stream>();
    incoming_stop<adc1_cell.stream>();
    A1::bus_clock(true);
    A1::release();
    AdcCommon::reset();
    return h;
}

void tr_released_requester() {
    (void)CycleCounter::init();
    Block::init();
    const char* names[6] = {"A contract (reset)       ", "B enable cleared, gated  ",
                            "C enable set, gated      ", "D enable cleared, clocked",
                            "E live (control)         ", "F the driver's release() "};
    Handover u[6];
    Handover sp[6];
    Handover t[6];
    Handover ad[6];
    for (uint8_t i = 0; i < 6u; ++i) {
        u[i] = usart6_handover(static_cast<Release>(i));
        sp[i] = spi1_handover(static_cast<Release>(i));
        t[i] = tim1_handover(static_cast<Release>(i));
        ad[i] = adc1_handover(static_cast<Release>(i));
    }
    for (uint8_t i = 0; i < 6u; ++i) {
        print(serial, "  ", names[i], ": USART6_TX ", u[i].moved, "/", u[i].moved_ungated,
              "  SPI1_TX ", sp[i].moved, "/", sp[i].moved_ungated, "  TIM1_UP ", t[i].moved,
              "/", t[i].moved_ungated, "  ADC1 ", ad[i].moved, "/", ad[i].moved_ungated,
              "  (items moved of 4 / after the clock reopened)", crlf);
    }
    bench.verdict("THE CONTROL: a live request moves the incoming owner's items on all four "
                  "requesters - the detector sees a request",
                  u[4].moved >= 1u && sp[4].moved >= 1u && t[4].moved >= 1u &&
                      ad[4].moved >= 1u);
    bench.verdict("A GATED CLOCK DOES NOT WITHDRAW A RAISED REQUEST: with the DMA enable left "
                  "set, the next owner of the cell moved an item WHILE the requester's clock "
                  "was gated, on all four (C)",
                  u[2].moved >= 1u && sp[2].moved >= 1u && t[2].moved >= 1u &&
                      ad[2].moved >= 1u);
    bench.verdict("A CLEARED DMA ENABLE DOES: gated or clocked, nothing reached the next "
                  "owner, nor when the clock returned (B, D)",
                  u[1].moved == 0u && sp[1].moved == 0u && t[1].moved == 0u &&
                      ad[1].moved == 0u && u[1].moved_ungated == 0u &&
                      sp[1].moved_ungated == 0u && t[1].moved_ungated == 0u &&
                      ad[1].moved_ungated == 0u && u[3].moved == 0u && sp[3].moved == 0u &&
                      t[3].moved == 0u && ad[3].moved == 0u);
    bench.verdict("THE CONTRACT HANDS OVER CLEAN: a requester released with its reset pulsed "
                  "before the gate leaves the incoming owner of its cell nothing (A)",
                  u[0].moved == 0u && sp[0].moved == 0u && t[0].moved == 0u &&
                      ad[0].moved == 0u && u[0].moved_ungated == 0u &&
                      sp[0].moved_ungated == 0u && t[0].moved_ungated == 0u &&
                      ad[0].moved_ungated == 0u);
    bench.verdict("THE ADC's OWN RELEASE HANDS OVER CLEAN without the shared reset: CR2 "
                  "cleared with the clock on, then the gate - nothing moved, nothing when the "
                  "clock returns (F)",
                  ad[5].moved == 0u && ad[5].moved_ungated == 0u);
}

// ---- the menu -----------------------------------------------------------------------

void banner() {
    print(serial, crlf, "test_stm32f4_dma - the DMA controllers", crlf);
    bench.menu();
}

}   // namespace

// The console's own two streams are the only ones this suite ever arms an
// interrupt on: the engines do it in their arm(), and the vector is one
// stream's alone on this family.
namespace {
/// One body for both stream vectors: whichever engined transport owns
/// the port answers for its own streams, and nobody answers while the
/// plain interrupt-driven console has it.
[[gnu::always_inline]] inline void serve_engined() {
    bool edge = false;
    if (engined_console == 1u) {
        edge = DmaSerial::dma_isr();
    } else if (engined_console == 2u) {
        edge = LoopSerial::dma_isr();
    } else if (engined_console == 3u) {
        edge = StressSerial::dma_isr();
    }
    if (edge) {
        engined_edges = engined_edges + 1u;
    }
}
/// The USART's vector: the plain console's, or the engined transport's -
/// whose idle-line and first-frame edges live here.
[[gnu::always_inline]] inline void serve_usart() {
    bool edge = false;
    if (engined_console == 0u) {
        (void)Serial::isr();
    } else if (engined_console == 1u) {
        edge = DmaSerial::isr();
    } else if (engined_console == 2u) {
        edge = LoopSerial::isr();
    } else if (engined_console == 3u) {
        edge = StressSerial::isr();
    }
    if (edge) {
        engined_edges = engined_edges + 1u;
    }
}
}   // namespace

#if defined(STM32F446xx)
extern "C" void DMA1_Stream6_IRQHandler() { serve_engined(); }
extern "C" void DMA1_Stream5_IRQHandler() { serve_engined(); }
extern "C" void USART2_IRQHandler() { serve_usart(); }
#elif defined(STM32F469xx)
extern "C" void DMA1_Stream3_IRQHandler() { serve_engined(); }
extern "C" void DMA1_Stream1_IRQHandler() { serve_engined(); }
extern "C" void USART3_IRQHandler() { serve_usart(); }
#else
extern "C" void DMA2_Stream7_IRQHandler() { serve_engined(); }
extern "C" void DMA2_Stream2_IRQHandler() { serve_engined(); }
extern "C" void USART1_IRQHandler() { serve_usart(); }
#endif

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

int main() {
    // What the silicon held before a line of this program ran. The two
    // gates are read out of the RCC, so reading them does not open them;
    // a stream's own registers need the clock, so DMA2's is opened for
    // the reading and the reset values are what it finds.
    boot.gate1 = brio::Dma<1>::bus_clock();
    boot.gate2 = brio::Dma<2>::bus_clock();
    brio::Dma<1>::bus_clock(true);
    brio::Dma<2>::bus_clock(true);
    boot.lisr1 = brio::Dma<1>::low_flags();
    boot.hisr1 = brio::Dma<1>::high_flags();
    boot.lisr2 = brio::Dma<2>::low_flags();
    boot.hisr2 = brio::Dma<2>::high_flags();
    boot.cr = Work::control();
    boot.fcr = Work::fifo_control();
    boot.ndtr = Work::count();

    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    bench.letter('a', "the two controllers and their sixteen vectors", ta_block);
    bench.letter('b', "a memory-to-memory block, byte-exact", tb_memory_to_memory);
    bench.letter('c', "what the driver refuses", tc_refusals);
    bench.letter('d', "the FIFO, table 49 cell by cell", td_fifo);
    bench.letter('e', "the throughput ladder", te_throughput);
    bench.letter('f', "the counting and alignment rules", tf_counting);
    bench.letter('g', "circular mode on the console's transmitter", tg_circular);
    bench.letter('h', "the double buffer", th_double_buffer);
    bench.letter('i', "the abort and the resume", ti_abort);
    bench.letter('j', "a transfer error", tj_transfer_error);
    bench.letter('k', "priority arbitration between two streams", tk_priority);
    bench.letter('l', "the console through the DMA engines", tl_engines);
    bench.letter('m', "the circular receive, on the transmitter's own echo", tm_receive);
    bench.letter('n', "the circular receive's two edges: a lap missed, the drain",
                 tn_receive_edges);
    bench.letter('r', "a released requester: what the next owner of its cell sees",
                 tr_released_requester);
    bench.letter('u', "brio stress: the host's stream into the receive engine", tu_stress,
                 false);
    bench.letter('p', "brio stress: the console's ring beside four copies on DMA2",
                 tp_ring_beside_copies, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL" : "FAILED", " tick=",
                    tick_ok ? "SysTick" : "FAILED", brio::crlf);
        banner();
        bench.prompt();
    }

    for (;;) {
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        brio::print(serial, static_cast<char>(c), brio::crlf);
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        bench.prompt();
    }
}
