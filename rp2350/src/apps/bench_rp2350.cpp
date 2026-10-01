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
#include "rp2350/mtime.hpp"
#include "rp2350/platform.hpp"
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
    return bench_counters<Idle>(uart_meter, tick_meter);
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
    return {bench_counters<Idle>(uart_meter, tick_meter), uart_meter.count(), uart_meter.cycles()};
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

int main() {
    const bool clock_ok = SysClock::init();
    const bool mtime_ok = brio::Mtime::start(clock);   // delay_us's ruler, on both halves
    const bool ruler_ok = Ruler::Counter::init(clock);
    Ruler::Counter::source(brio::TimerSource::sysclk);
    const bool serial_ok = Serial::init(clock, console_baud);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    bench.letter('r', "the ruler's self-check and the instrument's cost", tr_ruler);
    bench.letter('m', "memcpy and memset against the load/store floor", tm_memory);
    bench.letter('p', "a print through the console at its rate", tp_print);
    bench.letter('t', "the tick's floor: one second of idle", tt_tick);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL150" : "FAILED",
                    " mtime=", mtime_ok ? "1us" : "FAILED", " ruler=",
                    ruler_ok ? "clk_sys" : "FAILED", " tick=", tick_ok ? "1kHz" : "FAILED",
                    brio::crlf);
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
