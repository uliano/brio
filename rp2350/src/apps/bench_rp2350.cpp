// bench_rp2350 - the benchmark skeleton on an RP2350 (docs/design/benchmark.md),
// FROM ONE SOURCE ON BOTH ARCHITECTURES: the Cortex-M33 pair and the
// Hazard3 pair each build it from their own preset, and nothing below asks
// which one runs - every difference between the two cores is a name from
// rp2350/core.hpp, and the one word printed differently for each comes from
// `core_kind`, a value.
//
// NOT A TEST. Every letter prints its numbers as bench lines in the grammar
// of util/bench.hpp,
//
//   bench <op> n=<bytes> wall=<cycles> busy=<cycles> irq=<count> isr=<cycles> rate=<B/s> wire=<B/s> x=<wall/wire>
//
// and closes with one verdict, "ran", so that bin/brio reads its ALL: line.
// Letter r alone judges something, because a ruler that is wrong makes every
// other line wrong.
//
// THE CLOCK is test_rp2350_platform's: the 12 MHz crystal through the
// system PLL to clk_sys = 150 MHz, clk_peri = clk_sys (the UART's clock),
// clk_ref = the crystal undivided.
//
// THE RULER is TIMER1 with its SOURCE on clk_sys (rp2350/timer.hpp, datasheet
// 12.8.1.1): the system timer taken off the microsecond tick and onto the
// system clock, so its counter advances once a clk_sys cycle on both
// architectures - one ruler for two instruction sets, which no core counter
// of this chip is (SysTick exists on the Arm half alone). `Ruler::now()` is
// the counter's raw low word, ONE LOAD THROUGH THE APB BRIDGE (datasheet
// 2.2.4: TIMER1 sits in the APB segment, whose reads take three cycles at
// least); `Ruler::hz()` is clk_sys.
// It is hardware, so it counts through a masked window, and 2^32 of its
// cycles are 28.6 s - every operation here is far under that. TIMER0 is left
// alone, and the RISC-V platform timer stays what it is on this target: the
// ruler of delay_us on both halves and the kernel timebase on Hazard3.
//
// THE CONSOLE is UART0 on GP0 (TX) / GP1 (RX) under function 2 at 115200
// 8N1, the Debug Probe's bridge, bound as rp2350/src/apps/console.cpp binds
// it: `Uart<0, console_pins>` with its 64/256 rings, `isr_uart0` and
// `isr_systick` bound BY NAME - a vector-table slot on the M33, a dispatch
// entry of the crt on Hazard3, one source either way.
//
// NO KERNEL, NO ACTIVE OBJECT. The prompt loop and letters r and t idle
// through `Idle::idle()` directly - `BenchIdle<Rp2350Platform<>, Ruler>`,
// called with interrupts masked after the check, the kernel loop's own
// shape - so every idle turn of the program is a measured window.
//
// THE METERS: one `IsrMeter<Ruler, Idle>` per bound vector, `uart_meter` on
// isr_uart0 and `tick_meter` on isr_systick, enter() the handler's first
// statement and leave() its last, the driver's ISR body between them. WHAT
// THE STAMPS DO NOT SEE is the way into and out of the handler and the
// handler's own prologue and epilogue: on the M33 the exception entry and
// return, which are the hardware's (the basic frame stacked, the vector
// fetched); on Hazard3 the crt's dispatch, which is software
// (rp2350/src/glue/startup_rp2350_riscv.S: the trap table's jump, seventeen
// instructions to save the caller-saved registers, the meinext loop and the
// call, seventeen to restore, mret - forty-five instructions an external
// interrupt, thirty-seven a timer one). That path is counted as idle when
// the interrupt ended a window and as busy-but-not-isr when it preempted the
// thread, so `isr` is the bodies' cycles and not the interrupt's whole
// price, and the difference is the larger on Hazard3.
//
// What each letter prints:
//   r  THE RULER'S SELF-CHECK and the instrument's cost.
//      `delay_us(clock, 999)` timed on the ruler, judged AT LEAST 999 us of
//      clk_sys cycles and under 5 per cent more. 999 and not 1000: this
//      target's delay REFUSES a request of one kernel tick or more and spends
//      no time on it (rp2350/delay.hpp, delay_us_cap). The wait counts the
//      platform timer's microsecond, which is clk_ref - the crystal - and the
//      ruler counts clk_sys off the PLL, so the verdicts compare two clocks;
//      the delay's documented price is up to one microsecond late, far inside
//      the 5 per cent. Then three bench lines with n=0 and wire=0:
//        ruler   one Ruler::now(): 1000 PAIRS of back-to-back reads under the
//                mask, the mean difference of a pair - the time one read
//                takes, with no loop in it;
//        stamp   one enter()+leave() of a meter that is no vector's, with a
//                compiler fence after each pair (so the meter's fields are
//                loaded and stored as a handler does): a loop of 1000 turns
//                of one pair and the same loop with two pairs a turn, under
//                the mask, whose difference is 1000 pairs with no loop in it
//                - what a stamp pair adds to a handler's run. Not an empty
//                loop subtracted: on Hazard3 a loop whose body is one 16-bit
//                instruction runs a cycle slower a turn when that body lands
//                word-aligned (datasheet 3.8.7.10.1), and both loops here
//                have bodies the branch predictor serves alike;
//        window  100 ms of idle turns with nothing pending but the tick,
//                every field divided by the turns: one turn's wall, busy, irq
//                and isr - IT HOLDS THE TICK HANDLER, whose cycles are its
//                isr and part of its busy.
//      A plain line beside each carries its raw totals; the stamp's also
//      carries the meter's own reading of an empty body (the cost, below).
//   m  memcpy and memset of 1, 16, 256 and 4096 bytes between two static
//      word-aligned buffers (rt.cpp's word path), through VOLATILE FUNCTION
//      POINTERS so that the call is the runtime's and not a builtin the
//      compiler expanded in place (brio/rt/selftest.hpp's reason); each the
//      best of 8 runs by wall on a Stopwatch, interrupts on: busy = wall,
//      irq and isr whatever the tick took in the run kept.
//      THE WIRE is the core's load/store floor. Each core reaches the bus
//      through one 32-bit port for loads and stores, which performs one
//      access a cycle (datasheet chapter 3's opening, "Processor subsystem":
//      "each core can perform one instruction fetch and one load or store
//      access per cycle"), and the SRAM answers with no wait state (2.1.4:
//      the AHB5 fabric "offers zero-wait-state accesses everywhere"). On
//      Hazard3 the core's own table agrees: lw and sw are one cycle each
//      (3.8.7.1). The Cortex-M33 Technical Reference Manual publishes no
//      instruction timing, so the port is the floor stated for both cores. A
//      copy moves a word with one read and one write: 2 bytes a cycle, 300
//      MB/s at 150 MHz. A fill moves it with one write: 4 bytes a cycle,
//      600 MB/s.
//   p  print(serial, ...) of 1, 16, 256 and 4096 bytes - prefixes of one
//      static 4096-byte string, 62 characters and a CRLF in every 64 so the
//      capture stays legible - then a spin on Serial::tx_idle(), the
//      transport's own verb for "the ring is empty and the shifter has sent
//      the last stop bit" (UARTFR.BUSY clear). Counters before the print and
//      after the drain. THE WIRE is 115200 baud over the ten bits of an 8N1
//      frame: 11520 B/s. busy is wall (print spins on a full ring, the drain
//      spins on the flag); irq and isr are the transport's SHAPE
//      (pl011/uart.hpp: from an idle transmitter the first FIFO-depth bytes
//      go straight into the FIFO, then one handler per refill as the FIFO
//      falls through its level; a byte the full ring refuses writes
//      nothing) plus the ticks that landed in the print, so a plain line
//      under each splits them: the console's vector, and the tick's.
//   t  the tick's floor: one second of Idle::idle() turns with nothing to
//      do, counters before and after: wall = the second, irq = the ticks,
//      isr = the tick handler's cycles, busy = the floor.
//   d  THE DMA (rp2350/dma.hpp, datasheet 12.6), four operations:
//        copy   DmaCopyEngine moving 16, 256 and 4096 bytes as WORDS between
//               the two static buffers, polled: wall = the start to BUSY
//               falling, busy = wall, each the best of 8. THE WIRE is the
//               controller's own rate, one read and one write of up to 32
//               bits every clock (12.6's opening): 4 bytes a cycle, 600 MB/s.
//               A plain line beside each: the fixed cost, wall less one
//               cycle a word.
//        fill   the same sizes written from one word cell: the same wire.
//        paced  256 words from a table into ONE CELL, one a microsecond, the
//               request a DMA pacing timer's (DmaTimer<0>, X 1 / Y 150 of
//               clk_sys), the block's completion on line 0 - the core idles
//               through it, so busy is what the block cost the CPU and irq
//               the interrupts it took; the second of two runs, the first
//               running the handler and the idle path cold. THE WIRE is the pace, 4 MB/s. Then
//               the same block again with the core STAMPING THE RULER at
//               every step of the channel's count - a polled loop of a few
//               cycles a turn, which bounds the resolution - and the plain
//               line gives the 255 intervals against the 150-cycle pace:
//               the mean, how many lie within one polling turn of it, and
//               the extremes.
//        spi.dma  SPI0 as a host on its two engines (DmaTxEngine<4,
//               uint16_t>, DmaRxEngine<5, uint16_t>), 16 and 256 frames of
//               8 and of 16 bits at clk_peri / 2 (75 MHz) and clk_peri / 8
//               (18.75 MHz), on GP18/GP19/GP16 with GP17 as the select and
//               NOTHING ON THE WIRE: MISO floats and the time is the wire's
//               and the engines', the data unjudged (the suite's loop-back
//               judges it, test_rp2350_spi letter d). Wall = start() to the
//               completion on line 0, the core idling between, the best of
//               five after one thrown away (which runs cold out of the flash
//               and applies the new rate to the block). THE WIRE is
//               the SCK rate over eight bits a byte. The plain line beside
//               each gives the transaction's fixed cost - wall less the
//               wire's own cycles - which is the number this round is
//               judged by; another, per rate and width, the cycles a byte
//               by the difference of the two sizes. Then the same
//               transactions on the PL022's loop-back (LBM), plain lines
//               only, the cycles a byte beside the wire's: the instrument the
//               SPI suite's letter d judges the data on. And 256 bytes at
//               each rate with the core AWAKE while the engines run -
//               spinning on the RAM flag the handler sets, then spinning on
//               the ruler, a register behind the APB bridge the engines'
//               accesses to SSPDR cross too - beside the same block with the
//               core asleep.
//
// THE INSTRUMENT'S COST, and how a reader takes it out. Every number is RAW.
//  - A Stopwatch frames an operation between two ruler reads, inline, so
//    `wall` holds one read's latency beside the operation: subtract the
//    `ruler` figure.
//  - A stamp pair stretches the handler it sits in by the `stamp` figure:
//    `busy` holds that once per `irq`.
//  - The meter's own reading of an empty body (the plain line under `stamp`)
//    is what `isr` holds per `irq` beyond the bodies: isr - irq x that
//    figure is the bodies alone.
//  - `window`'s busy minus its isr is one idle turn's own cost - the loop's
//    check (a Stopwatch read here), the mask, the window's two stamps, the
//    sleep's entry and exit and the way into and out of the tick's handler -
//    which the tick's floor in `t` pays once a tick.
//
// The bench lines of a run carry the board's numbers and nothing else: the
// core, the clock, the ruler and the console's rate are one line printed by
// the banner and again at the head of letter r, so a capture of `z` has it.
//
// build: boards = weact2350b,weact2350b-rv
// build: monitor_speed = 115200

#include <stdint.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstring>
#include <string_view>

#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/delay.hpp"
#include "rp2350/dma.hpp"
#include "rp2350/mtime.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/spi.hpp"
#include "rp2350/ticker.hpp"
#include "rp2350/timer.hpp"
#include "rp2350/uart.hpp"
#include "util/bench.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 150'000'000UL>;
constexpr SysClock clock;
using P = brio::Rp2350Platform<>;

/// THE RULER (the file header): TIMER1 counting clk_sys. always_inline
/// because it is the stamps' one read: a call here would be inside every
/// measurement it takes.
///
/// AND A COMPILER BARRIER ON EITHER SIDE OF THE READ. The read is volatile,
/// which orders it against other volatile accesses and nothing else, so the
/// compiler is free to schedule a handler body's plain loads and stores
/// across a stamp - and does: without the fences the release image computes
/// the tick handler's counter increment ABOVE enter()'s read. The fences
/// emit no instruction; they make a read of the ruler a point the
/// program's memory accesses cannot cross.
struct Ruler {
    using Counter = brio::Timer<1>;
    [[gnu::always_inline]] static uint32_t now() {
        std::atomic_signal_fence(std::memory_order_seq_cst);
        const uint32_t t = Counter::now_low();
        std::atomic_signal_fence(std::memory_order_seq_cst);
        return t;
    }
    static uint32_t hz() { return SysClock::hz; }
};
static_assert(brio::CycleRuler<Ruler>);

using Idle = brio::BenchIdle<P, Ruler>;
static_assert(brio::Platform<Idle>);

namespace {

using namespace brio;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;   // rings 64/256 (defaults)
constexpr Serial serial;

constexpr uint32_t console_baud = 115200;
constexpr uint32_t frame_bits = 10;     // 8N1: start, eight data, stop

TestBench<Serial, 8> bench;

// One meter per bound vector (the file header).
IsrMeter<Ruler, Idle> uart_meter;
IsrMeter<Ruler, Idle> tick_meter;
IsrMeter<Ruler, Idle> dma_meter;
IsrMeter<Ruler, Idle> spi_meter;

// A meter no vector carries: letter r's stamp pair is measured on it, so
// the two above count interrupts and nothing else.
IsrMeter<Ruler, Idle> stamp_probe;

/// What every number of a run is relative to: the core, the clock, the
/// ruler as it stands (SOURCE read back) and the console's rate. Printed by
/// the banner and again at the head of letter r, so that a capture of `z`
/// carries it.
void identity() {
    print(serial, "  on ", core_kind == CoreKind::hazard3 ? "Hazard3" : "Cortex-M33",
          ": clk_sys ", SysClock::hz, " Hz, ruler TIMER1 counting ",
          Ruler::Counter::source() == TimerSource::sysclk ? "clk_sys" : "THE TICK (SOURCE lost)",
          " at ", Ruler::hz(), " Hz, console UART0 ", console_baud, " 8N1", crlf);
}

/// The counters at one instant. Each is a word this core moves in one
/// access, but the tick runs on through every letter, so the set is read
/// under the mask: a handler that lands between two of the reads would
/// leave a snapshot whose count and cycles belong to different instants.
/// Outside every measured span.
BenchCounters counters() {
    P::CriticalSection masked;
    return bench_counters<Idle>(uart_meter, tick_meter, dma_meter, spi_meter);
}

/// Let the console fall silent before a measurement, so its interrupts are
/// not inside what is measured. Bounded at one second of ruler.
void console_drain() {
    Stopwatch<Ruler> sw;
    sw.start();
    while (!Serial::tx_idle() && sw.elapsed() < Ruler::hz()) {
    }
}

/// One idle turn the way the kernel's loop takes it: masked, then idle(),
/// which sleeps and unmasks.
void idle_turn() {
    disable_interrupts();
    Idle::idle();
}

/// A sample with every field divided by `turns`, rounded to the nearest.
BenchSample per_turn(const BenchSample& s, uint32_t turns) {
    if (turns == 0u) {
        return s;
    }
    const auto div = [turns](uint32_t v) { return (v + turns / 2u) / turns; };
    return {div(s.wall), div(s.busy), div(s.irq), div(s.isr)};
}

// =============================================================================
// r - the ruler's self-check and the instrument's cost
// =============================================================================
constexpr uint32_t delay_request_us = 999;   // delay_us_cap - 1 (the file header)
constexpr uint32_t instrument_reps = 1000;

void tr_ruler() {
    identity();

    console_drain();
    Stopwatch<Ruler> sw;
    sw.start();
    const bool served = delay_us(clock, delay_request_us);
    const uint32_t took = sw.elapsed();
    const uint32_t floor_cycles =
        static_cast<uint32_t>(static_cast<uint64_t>(Ruler::hz()) * delay_request_us / 1'000'000u);
    print(serial, "  delay_us(clock, ", delay_request_us, ") = ", took, " ruler cycles; ",
          delay_request_us, " us of clk_sys is ", floor_cycles, crlf);
    bench.verdict("delay_us(clock, 999) is served and lasts AT LEAST 999 us of the ruler's "
                  "clk_sys cycles - the platform timer on the crystal against TIMER1 on the PLL",
                  served && took >= floor_cycles);
    bench.verdict("and under 5 per cent more: the delay's documented price is one microsecond "
                  "late (rp2350/delay.hpp)",
                  took < floor_cycles + floor_cycles / 20u);

    // ruler: pairs of back-to-back reads, the mean difference of a pair.
    uint32_t pair_sum = 0;
    {
        P::CriticalSection masked;
        for (uint32_t i = 0; i < instrument_reps; ++i) {
            const uint32_t a = Ruler::now();
            const uint32_t b = Ruler::now();
            pair_sum += b - a;
        }
    }
    print(serial, "  ruler: ", instrument_reps, " pairs of reads, ", pair_sum,
          " cycles between the two reads in all", crlf);
    const uint32_t read_cycles = (pair_sum + instrument_reps / 2u) / instrument_reps;
    bench_line(serial, "ruler", 0, {read_cycles, read_cycles, 0, 0}, Ruler::hz(), 0);

    // stamp: pairs on a meter no vector carries - a loop with one pair a
    // turn and the same loop with two, whose difference is 1000 pairs and
    // no loop. The fence makes the compiler load and store the meter's
    // fields every pair, as a handler that runs once does.
    uint32_t one_a_turn = 0;
    uint32_t two_a_turn = 0;
    const uint32_t probe_before = stamp_probe.cycles();
    {
        P::CriticalSection masked;
        Stopwatch<Ruler> loop;
        loop.start();
        for (uint32_t i = 0; i < instrument_reps; ++i) {
            stamp_probe.enter();
            stamp_probe.leave();
            std::atomic_signal_fence(std::memory_order_seq_cst);
        }
        one_a_turn = loop.elapsed();
        loop.start();
        for (uint32_t i = 0; i < instrument_reps; ++i) {
            stamp_probe.enter();
            stamp_probe.leave();
            std::atomic_signal_fence(std::memory_order_seq_cst);
            stamp_probe.enter();
            stamp_probe.leave();
            std::atomic_signal_fence(std::memory_order_seq_cst);
        }
        two_a_turn = loop.elapsed();
    }
    constexpr uint32_t probe_pairs = 3u * instrument_reps;
    const uint32_t probe_reading = stamp_probe.cycles() - probe_before;
    const uint32_t pair_cycles =
        two_a_turn > one_a_turn
            ? (two_a_turn - one_a_turn + instrument_reps / 2u) / instrument_reps
            : 0u;
    print(serial, "  stamp: ", instrument_reps, " turns of one pair in ", one_a_turn,
          " cycles, of two pairs in ", two_a_turn, "; the meter's own reading of an empty body ",
          probe_reading, " over ", probe_pairs, " pairs = ",
          (probe_reading + probe_pairs / 2u) / probe_pairs, " a pair", crlf);
    bench_line(serial, "stamp", 0, {pair_cycles, pair_cycles, 0, 0}, Ruler::hz(), 0);

    // window: 100 ms of idle turns, per turn. It holds the tick handler.
    console_drain();
    const BenchCounters before = counters();
    const uint32_t turns0 = Idle::idle_turns();
    Stopwatch<Ruler> span;
    span.start();
    while (span.elapsed() < Ruler::hz() / 10u) {
        idle_turn();
    }
    const uint32_t wall = span.elapsed();
    const BenchCounters after = counters();
    const uint32_t turns = Idle::idle_turns() - turns0;
    const BenchSample total = bench_sample(wall, before, after);
    print(serial, "  window: 100 ms = ", turns, " turns, wall ", total.wall, " busy ", total.busy,
          " irq ", total.irq, " isr ", total.isr, " (the tick handler is inside every turn)",
          crlf);
    bench_line(serial, "window", 0, per_turn(total, turns), Ruler::hz(), 0);
    bench.verdict("ran", true);
}

// =============================================================================
// m - the runtime's memcpy and memset against the load/store floor
// =============================================================================
constexpr uint32_t sizes[] = {1, 16, 256, 4096};
constexpr uint32_t copy_bytes_per_cycle = 2;   // a word read and a word write per word
constexpr uint32_t fill_bytes_per_cycle = 4;   // a word write per word

alignas(4) uint8_t source_buffer[4096];
alignas(4) uint8_t destination_buffer[4096];

// Called through volatile copies, so that the runtime's own symbols run
// (the file header).
void* (*volatile copy_fn)(void*, const void*, std::size_t) = &::memcpy;
void* (*volatile fill_fn)(void*, int, std::size_t) = &::memset;

template <typename Op>
BenchSample best_of_8(Op op) {
    BenchSample best{};
    for (uint8_t run = 0; run < 8u; ++run) {
        const BenchCounters before = counters();
        Stopwatch<Ruler> sw;
        sw.start();
        op();
        const uint32_t wall = sw.elapsed();
        const BenchCounters after = counters();
        const BenchSample s = bench_sample(wall, before, after);
        if (run == 0u || s.wall < best.wall) {
            best = s;
        }
    }
    return best;
}

void tm_memory() {
    for (uint32_t i = 0; i < sizeof(source_buffer); ++i) {
        source_buffer[i] = static_cast<uint8_t>(i * 7u + 1u);
    }
    console_drain();
    for (const uint32_t n : sizes) {
        const BenchSample s = best_of_8([n] { (void)copy_fn(destination_buffer, source_buffer, n); });
        bench_line(serial, "memcpy", n, s, Ruler::hz(), copy_bytes_per_cycle * Ruler::hz());
        console_drain();
    }
    for (const uint32_t n : sizes) {
        const BenchSample s = best_of_8([n] { (void)fill_fn(destination_buffer, 0x5A, n); });
        bench_line(serial, "memset", n, s, Ruler::hz(), fill_bytes_per_cycle * Ruler::hz());
        console_drain();
    }
    bench.verdict("ran", true);
}

// =============================================================================
// p - the console's per-byte path at its rate
// =============================================================================
constexpr std::array<char, 4096> make_payload() {
    constexpr char glyphs[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
    std::array<char, 4096> text{};
    for (uint32_t i = 0; i < text.size(); ++i) {
        const uint32_t column = i % 64u;
        text[i] = column == 62u ? '\r' : column == 63u ? '\n' : glyphs[column];
    }
    return text;
}
constexpr std::array<char, 4096> payload = make_payload();

/// The counters with the console's own meter beside them, read at one
/// instant: a print lasts long enough for the tick to land in it, and the
/// line's irq and isr sum both vectors.
struct PrintCounters {
    BenchCounters all;
    uint32_t uart_irq;
    uint32_t uart_isr;
};
PrintCounters print_counters() {
    P::CriticalSection masked;
    return {bench_counters<Idle>(uart_meter, tick_meter, dma_meter, spi_meter), uart_meter.count(),
            uart_meter.cycles()};
}

void tp_print() {
    for (const uint32_t n : sizes) {
        console_drain();
        const PrintCounters before = print_counters();
        Stopwatch<Ruler> sw;
        sw.start();
        print(serial, std::string_view{payload.data(), n});
        while (!Serial::tx_idle()) {
        }
        const uint32_t wall = sw.elapsed();
        const PrintCounters after = print_counters();
        const BenchSample s = bench_sample(wall, before.all, after.all);
        const uint32_t uart_irq = after.uart_irq - before.uart_irq;
        const uint32_t uart_isr = after.uart_isr - before.uart_isr;
        print(serial, crlf);
        bench_line(serial, "print", n, s, Ruler::hz(), console_baud / frame_bits);
        print(serial, "  of which the console's vector irq ", uart_irq, " isr ", uart_isr,
              ", the tick's irq ", s.irq - uart_irq, " isr ", s.isr - uart_isr, crlf);
    }
    bench.verdict("ran", true);
}

// =============================================================================
// t - the tick's floor
// =============================================================================
void tt_tick() {
    console_drain();
    const BenchCounters before = counters();
    Stopwatch<Ruler> sw;
    sw.start();
    while (sw.elapsed() < Ruler::hz()) {
        idle_turn();
    }
    const uint32_t wall = sw.elapsed();
    const BenchCounters after = counters();
    bench_line(serial, "tick", 0, bench_sample(wall, before, after), Ruler::hz(), 0);
    bench.verdict("ran", true);
}

// =============================================================================
// d - the DMA: copy and fill, a paced block, the SPI engines
// =============================================================================
using Copy = DmaCopyEngine<8>;
using Paced = DmaTxEngine<9, uint32_t>;
using Pace = DmaTimer<0>;
constexpr uint32_t dma_bytes_per_cycle = 4;   // one 32-bit read and write a clock (12.6)
constexpr uint32_t paced_words = 256;
constexpr uint16_t pace_cycles = 150;         // Y / X: one request a microsecond at 150 MHz

constexpr SpiPins spi_pins{.sck = 18, .tx = 19, .rx = 16};
using Spi = SpiHost<0, spi_pins, DmaTxEngine<4, uint16_t>, DmaRxEngine<5, uint16_t>>;
using SpiSelect = Pin<17>;

alignas(4) uint32_t paced_table[paced_words];
volatile uint32_t paced_cell = 0;
uint32_t pace_stamps[paced_words];
volatile bool paced_done = false;
volatile bool spi_done = false;
volatile bool spi_live = false;
volatile uint32_t spi_irq_at = 0;   // the ruler at the DMA line's entry, a transaction's completion

/// A copy or a fill and its wait on BUSY, timed: the best of 8.
template <typename Op>
BenchSample best_dma_of_8(Op op) {
    return best_of_8([&op] {
        (void)op();
        while (Copy::busy()) {
        }
    });
}

void dma_memory() {
    Copy::arm(DmaReport::errors);   // polled: the channel on no line
    auto* const to = reinterpret_cast<uint32_t*>(destination_buffer);
    const auto* const from = reinterpret_cast<const uint32_t*>(source_buffer);
    static uint32_t cell = 0x5A5A5A5Au;   // in SRAM: a const one would be read through the XIP cache
    for (uint32_t i = 0; i < sizeof(source_buffer); ++i) {
        source_buffer[i] = static_cast<uint8_t>(i * 7u + 1u);
    }
    console_drain();
    for (const uint32_t n : {16u, 256u, 4096u}) {
        const BenchSample s = best_dma_of_8([=] { return Copy::copy(to, from, n / 4u); });
        const bool exact = std::memcmp(destination_buffer, source_buffer, n) == 0;
        bench_line(serial, "copy", n, s, Ruler::hz(), dma_bytes_per_cycle * Ruler::hz());
        print(serial, "  copy ", n, ": fixed cost ", s.wall - n / 4u, " cycles over one a word, ",
              exact ? "exact" : "WRONG", crlf);
        console_drain();
    }
    for (const uint32_t n : {16u, 256u, 4096u}) {
        const BenchSample s = best_dma_of_8([=] { return Copy::fill(to, &cell, n / 4u); });
        bool exact = true;
        for (uint32_t i = 0; i < n / 4u; ++i) {
            exact = exact && to[i] == cell;
        }
        bench_line(serial, "fill", n, s, Ruler::hz(), dma_bytes_per_cycle * Ruler::hz());
        print(serial, "  fill ", n, ": fixed cost ", s.wall - n / 4u, " cycles over one a word, ",
              exact ? "exact" : "WRONG", crlf);
        console_drain();
    }
    Copy::stop();
}

void dma_paced() {
    for (uint32_t i = 0; i < paced_words; ++i) {
        paced_table[i] = i * 0x01010101u;
    }
    Pace::set(1, pace_cycles);
    Paced::arm(&paced_cell, Pace::dreq());   // a completion on line 0
    // Twice, the first thrown away: it runs the handler and the idle path
    // cold out of the flash.
    BenchSample paced{};
    uint32_t banked = 0;
    for (uint8_t run = 0; run < 2u; ++run) {
        console_drain();
        paced_done = false;
        banked = DmaChannel<Paced::channel>::debug_credits();
        const BenchCounters before = counters();
        Stopwatch<Ruler> sw;
        sw.start();
        (void)Paced::start(paced_table, paced_words);
        while (!paced_done && sw.elapsed() < Ruler::hz() / 10u) {
            idle_turn();
        }
        const uint32_t wall = sw.elapsed();
        const BenchCounters after = counters();
        paced = bench_sample(wall, before, after);
    }
    bench_line(serial, "paced", paced_words * 4u, paced, Ruler::hz(),
               4u * (Ruler::hz() / pace_cycles));
    print(serial, "  paced: the cell holds ", hex(paced_cell), " (the table's last word ",
          hex(paced_table[paced_words - 1u]), "); the channel held ", banked,
          " request credits from the timer when the block started", crlf);

    // The jitter: the same block, the core stamping the ruler each time
    // the channel's count steps.
    console_drain();
    Paced::arm(&paced_cell, Pace::dreq(), false, DmaReport::errors);
    uint32_t turns = 0;
    uint32_t start_at = 0;
    uint32_t banked_now = 0;
    {
        P::CriticalSection masked;
        uint32_t seen = paced_words;
        uint32_t k = 0;
        banked_now = DmaChannel<Paced::channel>::debug_credits();
        const uint32_t t0 = Ruler::now();
        start_at = t0;
        (void)Paced::start(paced_table, paced_words);
        while (k < paced_words && Ruler::now() - t0 < Ruler::hz() / 10u) {
            const uint32_t left = DmaChannel<Paced::channel>::count();
            const uint32_t now = Ruler::now();
            ++turns;
            while (seen > left && k < paced_words) {
                pace_stamps[k++] = now;
                --seen;
            }
        }
        (void)Paced::complete();
    }
    const uint32_t span = pace_stamps[paced_words - 1u] - pace_stamps[0];
    const uint32_t turn = span / (turns != 0u ? turns : 1u);   // the instrument's resolution
    uint32_t lo = 0xFFFFFFFFu;
    uint32_t hi = 0;
    uint32_t inside = 0;
    for (uint32_t i = 1; i < paced_words; ++i) {
        const uint32_t d = pace_stamps[i] - pace_stamps[i - 1u];
        lo = d < lo ? d : lo;
        hi = d > hi ? d : hi;
        if (d + turn >= pace_cycles && d <= pace_cycles + turn) {
            ++inside;
        }
    }
    print(serial, "  paced, the pace against the ruler: ", span, " cycles over 255 steps = ",
          (span + 127u) / 255u, " a step (", pace_cycles, " asked); ", inside,
          " of the 255 intervals within one polling turn (", turn, " cycles) of it, the "
          "extremes ", lo, " and ", hi, "; ", banked_now, " credits banked at the start, the "
          "first step seen ", pace_stamps[0] - start_at, " cycles after it and the last ",
          pace_stamps[paced_words - 1u] - start_at, crlf);
    Pace::stop();
    Paced::stop();
}

/// A transaction's numbers, and where its wall went: start() returning,
/// and the completion's handler entered, both from the first stamp.
struct SpiRun {
    BenchSample sample;
    uint32_t launch;
    uint32_t to_irq;
};

/// How the core waits for a transaction's completion: asleep (the idle
/// turn, the default), spinning on the RAM flag the handler sets, or
/// spinning on a read of the ruler - a register behind the APB bridge, as
/// the SPI's own data register is.
enum class SpiWait : uint8_t { sleep, spin_ram, spin_apb };

/// One engined transaction of `frames` frames, timed start() to the
/// completion on line 0 with the core idling between.
SpiRun spi_once(uint16_t frames, SpiClock rate, SpiDataSize bits, SpiWait wait = SpiWait::sleep) {
    Spi::Request r{};
    r.cs = SpiSelect::ref();
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
    r.cmd_len = 0;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(source_buffer));
    r.rx = lend<Lease::reply>(destination_buffer);
    r.len = frames;
    r.clock = rate;
    r.mode = SpiMode::mode0;
    r.bits = bits;
    console_drain();
    spi_done = false;
    spi_irq_at = 0;
    const BenchCounters before = counters();
    const uint32_t t0 = Ruler::now();
    if (Spi::start(r)) {
        spi_done = true;
    }
    const uint32_t launched = Ruler::now();
    if (wait == SpiWait::spin_ram) {
        for (uint32_t spins = 1'000'000u; !spi_done && spins != 0u; --spins) {
        }
    } else if (wait == SpiWait::spin_apb) {
        while (!spi_done && Ruler::now() - t0 < Ruler::hz() / 10u) {
        }
    }
    while (!spi_done && Ruler::now() - t0 < Ruler::hz() / 10u) {
        idle_turn();
    }
    const uint32_t wall = Ruler::now() - t0;
    const BenchCounters after = counters();
    return {bench_sample(wall, before, after), launched - t0, spi_irq_at - t0};
}

/// The best of five by wall, after one run thrown away: the first of a
/// configuration runs its code cold out of the flash and applies the new
/// rate or width to the block, which are the cache's cost and the
/// reconfiguration's, not the engines'.
uint32_t spi_cold_wall = 0;   // the very first transaction's wall: the code cold out of the flash

SpiRun spi_transaction(uint16_t frames, SpiClock rate, SpiDataSize bits) {
    const SpiRun thrown = spi_once(frames, rate, bits);
    if (spi_cold_wall == 0u) {
        spi_cold_wall = thrown.sample.wall;
    }
    SpiRun best{};
    for (uint8_t run = 0; run < 5u; ++run) {
        const SpiRun s = spi_once(frames, rate, bits);
        if (run == 0u || s.sample.wall < best.sample.wall) {
            best = s;
        }
    }
    return best;
}

void dma_spi() {
    (void)SpiSelect::output(true);
    (void)Spi::init(clock);
    spi_live = true;
    spi_cold_wall = 0;
    struct Rate { SpiClock clock; const char* name; };
    constexpr Rate rates[] = {{SpiClocks::div2, "75 MHz"}, {SpiClocks::div8, "18.75 MHz"}};
    for (const Rate& rate : rates) {
        const uint32_t sck = Spi::sck_hz(rate.clock);
        for (const SpiDataSize bits : {SpiDataSize::bits8, SpiDataSize::bits16}) {
            const uint32_t frame_bytes = bits == SpiDataSize::bits16 ? 2u : 1u;
            uint32_t walls[2] = {0, 0};
            uint8_t k = 0;
            for (const uint16_t frames : {static_cast<uint16_t>(16), static_cast<uint16_t>(256)}) {
                const SpiRun run = spi_transaction(frames, rate.clock, bits);
                const BenchSample& s = run.sample;
                const uint32_t n = frames * frame_bytes;
                const uint32_t wire_cycles =
                    static_cast<uint32_t>(static_cast<uint64_t>(n) * 8u * Ruler::hz() / sck);
                walls[k++] = s.wall;
                bench_line(serial, "spi.dma", n, s, Ruler::hz(), sck / 8u);
                const uint32_t fixed = s.wall > wire_cycles ? s.wall - wire_cycles : 0u;
                print(serial, "  spi.dma ", frames, " frames of ", spi_data_bits(bits), " bits at ",
                      rate.name, ": the wire ", wire_cycles, " cycles, the transaction's fixed "
                      "cost ", fixed, " cycles = ", fixed * 1000u / (Ruler::hz() / 1'000'000u),
                      " ns; start() returned at ", run.launch, ", the completion's handler "
                      "entered at ", run.to_irq, crlf);
            }
            const uint32_t per_byte_x100 =
                walls[1] > walls[0] ? (walls[1] - walls[0]) * 100u / (240u * frame_bytes) : 0u;
            print(serial, "  spi.dma at ", rate.name, ", ", spi_data_bits(bits),
                  "-bit frames: ", per_byte_x100 / 100u, '.', per_byte_x100 % 100u / 10u,
                  per_byte_x100 % 10u, " cycles a byte by the difference of the two sizes, the "
                  "wire's ", 8u * Ruler::hz() / sck, crlf);
            if (k == 2u && rate.clock == SpiClocks::div2 && bits == SpiDataSize::bits8) {
                print(serial, "  spi.dma, the very first transaction (16 frames, its code cold "
                      "out of the flash and the rate applied to the block): ", spi_cold_wall,
                      " cycles", crlf);
            }
        }
    }
    // THE SAME TRANSACTIONS ON THE LOOP-BACK (LBM), plain lines and not
    // bench lines: the cycles a byte by the difference of two sizes, the
    // instrument test_rp2350_spi's letter d uses - for the reader who
    // compares the two.
    Spi::loopback(true);
    for (const Rate& rate : rates) {
        const uint32_t sck = Spi::sck_hz(rate.clock);
        for (const SpiDataSize bits : {SpiDataSize::bits8, SpiDataSize::bits16}) {
            const uint32_t frame_bytes = bits == SpiDataSize::bits16 ? 2u : 1u;
            const SpiRun small = spi_transaction(16, rate.clock, bits);
            const SpiRun large = spi_transaction(256, rate.clock, bits);
            const uint32_t per_byte_x100 =
                large.sample.wall > small.sample.wall
                    ? (large.sample.wall - small.sample.wall) * 100u / (240u * frame_bytes)
                    : 0u;
            print(serial, "  spi.dma ON THE LOOP-BACK at ", rate.name, ", ", spi_data_bits(bits),
                  "-bit frames: 16 frames in ", small.sample.wall, " cycles, 256 in ",
                  large.sample.wall, ", irq ", large.sample.irq, "; ", per_byte_x100 / 100u, '.',
                  per_byte_x100 % 100u / 10u, per_byte_x100 % 10u,
                  " cycles a byte by the difference, the wire's ", 8u * Ruler::hz() / sck, crlf);
        }
    }
    Spi::loopback(false);

    // AND THE SAME 256 BYTES WITH THE CORE AWAKE: spinning on the flag in
    // RAM, then spinning on the ruler, whose every read crosses the APB
    // bridge the DMA's accesses to SSPDR cross too.
    for (const Rate& rate : rates) {
        const uint32_t sck = Spi::sck_hz(rate.clock);
        uint32_t walls[3] = {0, 0, 0};
        const SpiWait waits[3] = {SpiWait::sleep, SpiWait::spin_ram, SpiWait::spin_apb};
        for (uint8_t w = 0; w < 3u; ++w) {
            (void)spi_once(256, rate.clock, SpiDataSize::bits8, waits[w]);
            uint32_t best = 0;
            for (uint8_t run = 0; run < 5u; ++run) {
                const uint32_t wall = spi_once(256, rate.clock, SpiDataSize::bits8, waits[w]).sample.wall;
                best = run == 0u || wall < best ? wall : best;
            }
            walls[w] = best;
        }
        print(serial, "  spi.dma 256 bytes at ", rate.name, " (the wire ",
              256u * 8u * (Ruler::hz() / sck), " cycles): the core asleep ", walls[0],
              ", spinning on a RAM flag ", walls[1], ", spinning on the APB ruler ", walls[2],
              " cycles", crlf);
    }
    spi_live = false;
    Spi::release();
}

void td_dma() {
    dma_memory();
    dma_paced();
    dma_spi();
    bench.verdict("ran", true);
}

void banner() {
    print(serial, crlf, "bench_rp2350", crlf);
    identity();
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
//
// ONE NAME, BOTH ARCHITECTURES (the file header), each vector with its
// meter's stamps around the driver's ISR body.
extern "C" void isr_uart0() {
    uart_meter.enter();
    (void)Serial::isr();
    uart_meter.leave();
}
extern "C" void isr_systick() {
    tick_meter.enter();
    brio::Ticker::tick();
    tick_meter.leave();
}
extern "C" void isr_dma_0() {
    dma_meter.enter();
    spi_irq_at = Ruler::now();
    if (spi_live && Spi::dma_isr()) {
        spi_done = true;
    }
    if ((Paced::service() & Paced::flag_complete) != 0u) {
        (void)Paced::complete();
        paced_done = true;
    }
    dma_meter.leave();
}
extern "C" void isr_spi0() {
    spi_meter.enter();
    if (spi_live && Spi::isr()) {
        spi_done = true;
    }
    spi_meter.leave();
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool mtime_ok = brio::Mtime::start(clock);   // delay_us's ruler, on both halves
    const bool ruler_ok = Ruler::Counter::init(clock);
    Ruler::Counter::source(brio::TimerSource::sysclk);
    const bool serial_ok = Serial::init(clock, console_baud);
    const bool tick_ok = brio::Ticker::init(clock);
    const bool dma_ok = brio::Dma::init();
    brio::enable_interrupts();

    bench.letter('r', "the ruler's self-check and the instrument's cost", tr_ruler);
    bench.letter('m', "memcpy and memset against the load/store floor", tm_memory);
    bench.letter('p', "a print through the console at its rate", tp_print);
    bench.letter('t', "the tick's floor: one second of idle", tt_tick);
    bench.letter('d', "the DMA: copy, fill, a paced block, the SPI engines", td_dma);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL150" : "FAILED",
                    " mtime=", mtime_ok ? "1us" : "FAILED", " ruler=",
                    ruler_ok ? "clk_sys" : "FAILED", " tick=", tick_ok ? "1kHz" : "FAILED",
                    " dma=", dma_ok ? "released" : "FAILED", brio::crlf);
        banner();
        bench.prompt();
    }

    for (;;) {
        brio::disable_interrupts();
        if (!Serial::rx_pending()) {
            Idle::idle();
            continue;
        }
        brio::enable_interrupts();
        uint8_t c = 0;
        if (!Serial::read_byte(c) || c == '\r' || c == '\n') {
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
