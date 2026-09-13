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
//   r  (not in z) the receive engine, harvesting a line the host sends:
//      brio run <board> "rhello dma"
//
// build: boards = f429zi,f446re,f411ce
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>

#include "stm32f4/clock.hpp"
#include "stm32f4/dma.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

#if defined(STM32F411xE)
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 100'000'000, 25'000'000>;
#elif defined(STM32F429xx)
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
// channel 4 (RM0090 tables 43 and 44, and their RM0390 / RM0383 twins).
// The Uart checks the cells at compile time - these lines only have to
// name the right ones.
#if defined(STM32F446xx)
constexpr UartPins console_pins{.tx = {'A', 2, PinFunction::af7}, .rx = {'A', 3, PinFunction::af7}};
constexpr uint8_t console_instance = 2;
using ConsoleTxStream = DmaStream<1, 6>;
using ConsoleTxEngine = DmaTxEngine<1, 6, 4>;
using ConsoleRxEngine = DmaRxEngine<1, 5, 4>;
#else
constexpr UartPins console_pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
constexpr uint8_t console_instance = 1;
using ConsoleTxStream = DmaStream<2, 7>;
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

/// What letter l and letter r leave behind for the verdicts, which can
/// only be printed once the plain console is back.
struct EngineRun {
    bool up = false;
    uint32_t queued = 0;
    uint32_t millis = 0;
    uint8_t faults = 0;
    bool drained = false;
    uint16_t harvested = 0;
    bool rx_running = false;
    uint8_t hw_overruns = 0;   ///< ORE: a byte the receiver lost in silicon
    uint8_t rx_overruns = 0;   ///< the ring had no room for a new run
    uint8_t line_errors = 0;   ///< NE, FE, PE: each costs the byte in DR
    uint16_t first_gap = 0;    ///< where the echo first left the pattern
};

/// Which transport owns the port right now, for the two stream vectors
/// to dispatch on: 0 the plain interrupt-driven one, 1 the engined
/// console, 2 the engined half-duplex loop.
volatile uint8_t engined_console = 0;

/// Hand the port to `Port` (an engined Uart type), send `block` bytes
/// through the transmit stream, harvest for `rx_window_ms`, and give it
/// back. Nothing may print between the two swaps: the verdicts wait for
/// the plain console to come home.
template <typename Port>
EngineRun run_engined(uint8_t which, uint16_t block, uint32_t rx_window_ms) {
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

    // THE HARVEST RIDES ALONG WITH THE SENDING, because that is how a
    // program uses this port: the receive run is the ring's free span,
    // and an owner that stops asking while the line is busy lets that
    // run fill and loses what arrives after it. On the half-duplex loop
    // everything sent comes straight back, so the two directions are
    // busy at once and the pacing is the whole test.
    const uint32_t t0 = Ticker::millis();
    for (;;) {
        if (r.queued < block) {
            r.queued += Port::write_bulk(std::span<const uint8_t>(line, sizeof(line)));
        }
        (void)Port::harvest();
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
        (void)Port::harvest();
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

// ---- m  the receive engine, on the transmitter's own echo -----------------------------

void tm_receive() {
    Block::init();
    Dma<1>::init();
    // SINGLE-WIRE HALF DUPLEX IS THE WIRE THIS BOARD HAS NOT GOT: with
    // HDSEL the receiver listens on the transmit pad, so a block sent by
    // the transmit stream arrives back at the receive stream. The host
    // sees the block too - the pad is still the console's.
    const EngineRun r = run_engined<LoopSerial>(2, 128, 20);

    print(serial, "  sent ", r.queued, " byte(s) in ", r.millis, " ms and harvested ",
          r.harvested, " through the receive stream; ", r.hw_overruns, " overrun(s) in "
          "silicon, ", r.line_errors, " line error(s), ", r.rx_overruns,
          " ring(s) with no room, ", r.faults, " fault(s)", crlf);
    bench.verdict("the port comes up in half duplex with both slots filled", r.up);
    bench.verdict("the receive engine is armed from the first byte", r.rx_running);
    bench.verdict("the transmit stream drained the ring", r.drained);
    // WHAT IS NOT HARVESTED IS ACCOUNTED FOR - or it is THE RE-ARM GAP,
    // which is this engine's one real cost and is measured here rather
    // than assumed away. A receive run that fills stops the stream, and
    // the byte that arrives before harvest() has started the next run
    // can be lost - SILENTLY, with no overrun flag to show for it,
    // because nothing overran: the receiver was simply not being served.
    // Measured: one byte, exactly at the boundary where the first full
    // run is swapped for the next or a partial tail is read - WHERE it
    // falls depends on the harvest's timing against the line, so the
    // position is reported and the count is judged.
    const uint32_t accounted =
        static_cast<uint32_t>(r.harvested) + static_cast<uint32_t>(r.hw_overruns) +
        static_cast<uint32_t>(r.rx_overruns) + static_cast<uint32_t>(r.line_errors);
    print(serial, "  accounted for: ", accounted, " of ", r.queued, ", first gap at byte ",
          r.first_gap, crlf);
    bench.verdict("the echo arrives whole but for the re-arm gap",
                  r.harvested + 1u >= r.queued && r.harvested <= r.queued);
    print(serial, "  the gap, when there is one, is at a harvest: byte ", r.first_gap,
          " this time (", rx_ring_bytes, " is the first run's boundary)", crlf);
    bench.verdict("no block was thrown away", r.faults == 0u);
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
    if (engined_console == 1u) {
        (void)DmaSerial::dma_isr();
    } else if (engined_console == 2u) {
        (void)LoopSerial::dma_isr();
    }
}
}   // namespace

#if defined(STM32F446xx)
extern "C" void DMA1_Stream6_IRQHandler() { serve_engined(); }
extern "C" void DMA1_Stream5_IRQHandler() { serve_engined(); }
extern "C" void USART2_IRQHandler() {
    if (engined_console == 0u) {
        (void)Serial::isr();
    }
}
#else
extern "C" void DMA2_Stream7_IRQHandler() { serve_engined(); }
extern "C" void DMA2_Stream2_IRQHandler() { serve_engined(); }
extern "C" void USART1_IRQHandler() {
    if (engined_console == 0u) {
        (void)Serial::isr();
    }
}
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
    bench.letter('m', "the receive engine, on the transmitter's own echo", tm_receive);

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
