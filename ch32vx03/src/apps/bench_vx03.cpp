// bench_vx03 - the benchmark skeleton on the CH32V203 and the CH32V303
// (QingKe V4B and V4F, RV32IMAC and RV32IMAFC): the instrument's
// self-check and cost, the runtime's copy and fill against the core's
// floor, a print through the console transport against the wire, and the
// tick's floor - one line per operation and size, in util/bench.hpp's
// grammar (docs/design/benchmark.md).
//
// NOT A TEST. TestBench frames it (the menu, the ALL: line bin/brio
// waits for); every letter ends with one verdict "ran", and letter r's
// two verdicts on the ruler are the only ones that judge anything.
//
// THE BOARD AND THE RATES. The WeAct CH32V203C8T6 core board at 144 MHz
// (the HSI times eighteen through the PLL), the rate test_vx03_platform
// runs at. The console is USART1 on PA9/PA10 through the WCH-Link at
// 115200 8N1 (BRR = 1250 on the 144 MHz of PB2: the rate is exact), the
// interrupt-driven transport with no DMA engine, bound as that suite
// binds it. The same source builds for the CH32V203C6 (the 32 KB tier's
// link guard) and for the CH32V303VC, whose V4F runs it under the ilp32f
// ABI with the console wired the same way on WCH's evaluation board.
//
// NO KERNEL. The suite's shape: a prompt loop over the console, which
// here sleeps through BenchIdle<P, Ruler>::idle() when no byte is
// pending, as the kernel's loop would. That adapter is the platform every
// type below names, the transport's rings included. No bus master but
// the core works here, so the platform's idle() takes its sleep on every
// turn (ch32vx03/bus_activity.hpp).
//
// THE RULER. Ticker::cycles() on the STK (ch32vx03/ticker.hpp): the tick
// count and the counter's position in its period composed by
// util/cycle_count.hpp, in HCLK cycles (STCLK = 1) - so hz() is the
// clock's 144 MHz. A read is a CMPLR load, two loads each of the tick
// count and of SR, a CNTL load, a multiply and an add, retried when the
// pieces straddle the handler. Ruler::now() is always_inline and
// flatten, so the whole read is spelled inline wherever a stamp or a
// Stopwatch takes one, the vectors included: no call on the hot path.
//
// THE METERS. Two vectors, one IsrMeter each, enter() the handler's first
// statement and leave() its last, the body between them as the suite
// binds it:
//   systick_handler  the tick. Its meter reads the STK's POSITION alone
//                    (TickRuler: CNTL, one load). Ticker::cycles() would
//                    be right here too - CNTIF stands from the restart
//                    until tick() clears it, so the composition counts
//                    the period the handler has not counted yet - and the
//                    position is merely the cheaper read, taken because
//                    letter t measures the floor this vector makes: a
//                    pair of full reads would be most of what it
//                    measures. Two reads of one period are exact, and the
//                    handler runs right after the restart, a whole
//                    period away from the next.
//   usart1_handler   USART1's ISR body, on the full Ruler: this handler
//                    may straddle a restart, where the position alone
//                    would go back by a period.
// The hardware prologue's entry and exit (16 and 25 HCLK cycles with HPE,
// docs/ch32vx03/platform.md) lie outside the stamps.
//
// What each letter prints:
//
//   r  THE RULER'S SELF-CHECK. delay_us(clock, 999) - the longest wait
//      this family serves, one tick period and above being refused
//      (ch32vx03/delay.hpp) - read on the ruler from a fresh tick: at
//      least the 143856 cycles due, and under them + 5 per cent (the
//      wait is documented late by some 40 cycles of polling,
//      docs/ch32vx03/platform.md).
//      Then the instrument's cost, five lines with n=0 and wire=0; in
//      the four averaged ones EVERY FIELD IS THE RUN'S TOTAL DIVIDED BY
//      ITS UNITS, and the note line before each states the units and the
//      run's interrupts:
//        ruler       one Ruler::now(): 1000 reads in ten batches of 100,
//                    each batch started on a fresh tick so that no
//                    interrupt lands in it; a batch's Stopwatch adds one
//                    read per hundred
//        stopwatch   an empty Stopwatch - start() then elapsed(), nothing
//                    between - the best of 8: not averaged, because it is
//                    exactly what every Stopwatch line of m, p and t holds
//                    of the instrument
//        stamp       one enter()+leave() pair with nothing between, on a
//                    Ruler meter bound to no vector: 1000 pairs, batched
//                    the same - what the USART vector's stamps add
//        stamp.tick  the same on a TickRuler meter: what the tick
//                    vector's stamps add
//        window      one BenchIdle::idle() turn with nothing pending but
//                    the tick, over the turns of 100 ms. wall is the turn
//                    and busy HOLDS THE TICK HANDLER, added back because it
//                    ran inside the window, beside the loop's own
//                    turnaround; isr is the handler alone, so busy - isr
//                    is what a turn costs besides it. A tick makes TWO
//                    turns here (measured on the CH32V203C8T6): after
//                    every wake the platform's WFE form finds an event
//                    standing, and the next wfi returns at once on it -
//                    so wall is half a tick period, and the note line's
//                    two counts say so
//   m  memcpy and memset of the runtime (rt/rt.cpp), each the best of 8
//      runs on a Stopwatch, at 1, 16, 256 and 4096 bytes between two
//      static word-aligned buffers (the runtime's word path). Nothing
//      idles and nothing is expected to interrupt: busy is wall, and irq
//      and isr are whatever the tick took in the best run. At 1 and 16
//      bytes a line is mostly the call and the stopwatch, not the copy:
//      x is large there by construction.
//   p  print(serial, s) of a static string of 1, 16, 256 and 4096 bytes
//      (62 letters and a CR LF to a line of 64), then the DRAIN: the
//      transport's own tx_idle(), which on this family is the ring empty
//      AND the resource's TC flag (the last frame out of the shifter) in
//      one verb (ch32vx03/usart.hpp). The counters are read before the
//      print and after the drain. The print spins on a full ring, so busy
//      is close to wall; irq and isr are the transport's per-byte shape -
//      one TXE interrupt per byte, one more that disarms TXEIE - and the
//      ticks. The transmitter starts a frame on the edge of its own
//      free-running bit clock (measured on the CH32V203C8T6: up to one
//      bit between the first DATAR store and the shifter's load, then
//      exactly ten bits to TC), and a print begun just after a drain,
//      which ends on such an edge, waits nearly the whole bit: every line
//      carries about one bit time above the wire, whatever its length.
//   t  THE TICK'S FLOOR: one second of BenchIdle::idle() turns with
//      nothing to do (this loop, not the kernel's), counters before and
//      after: wall = the second in cycles, irq = the ticks, isr = the tick
//      handler's cycles, busy = the floor.
//
// THE WIRE, and why it is the limit:
//   memcpy, memset  the core's load/store floor. The QingKe V4 manual
//                   (V1.1, the Features table and table 1-1) states a
//                   three-stage pipeline and no instruction timing; the
//                   reference manual (RM 1.1, figure 1-4) puts the SRAM
//                   behind the system bus and the bus matrix with no wait
//                   state named, and the instruction fetch on the I-Code
//                   bus from the flash's zero-wait area (datasheet 2.5.2,
//                   table 2-1's note 1) - not on the data's bus. The floor
//                   taken is that bus's: one 32-bit SRAM access per HCLK
//                   cycle - memset 4 bytes a cycle (a store a word),
//                   memcpy 2 (a load and a store a word) - 576 and 288
//                   MB/s at 144 MHz. It is the bus's figure and not a
//                   documented instruction timing: x says what the
//                   pipeline and the loop cost above it.
//   print           the console's ACTUAL baud, read back from BRR (144 MHz
//                   over 1250, 115200 baud), over the ten bits of an 8N1
//                   frame: 11520 B/s.
//
// THE INSTRUMENT'S COST, and how the reader subtracts it. Every line
// carries RAW numbers. A Stopwatch measurement holds the stopwatch line's
// wall - start()'s read's tail and elapsed()'s read's head, both spelled
// inline - so subtract it from every line of m, p and t. The USART
// handler's metered cycles hold about one ruler read as well (enter()'s
// tail and leave()'s head): subtract its interrupts times the ruler line
// from isr. The tick handler's hold the store of its start stamp and
// little else. The two stamp lines are what each vector's pair adds to
// the program's real cost per interrupt. And util/bench.hpp's two stated
// seams: the latency of entering and leaving a handler is counted as
// idle, and an interrupt landing between P::idle()'s return and the
// window's close is counted in both.
//
// build: boards = v203c6,v203c8,v303vc
// build: monitor_speed = 115200

#include <stdint.h>
#include <string.h>

#include <cstddef>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/delay.hpp"
#include "ch32vx03/device.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/ticker.hpp"
#include "ch32vx03/usart.hpp"
#include "util/bench.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 144'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

/// The STK's cycle count as the bench's ruler. flatten pulls the ticker's
/// read into this function and always_inline pulls this function into
/// every stamp: a vector carries the read itself, not a call to it.
struct Ruler {
    [[gnu::always_inline, gnu::flatten]] static uint32_t now() { return Ticker::cycles(); }
    static constexpr uint32_t hz() { return SysClock::hz; }
};
static_assert(CycleRuler<Ruler>);

/// The STK's position in its period: the tick vector's ruler (the header
/// says why). Exact between two reads of one period.
struct TickRuler {
    [[gnu::always_inline, gnu::flatten]] static uint32_t now() { return stk()->CNTL; }
    static constexpr uint32_t hz() { return SysClock::hz; }
};
static_assert(CycleRuler<TickRuler>);

using Plat = BenchIdle<Ch32vx03Platform<>, Ruler>;
using Meter = IsrMeter<Ruler, Plat>;
using TickMeter = IsrMeter<TickRuler, Plat>;

using Serial = Uart<1, Plat>;   // USART1 on PA9/PA10, rings 64/64
constexpr Serial serial;
constexpr uint32_t console_baud = 115200;

TestBench<Serial> bench;

TickMeter tick_meter;          // the STK's vector
Meter usart_meter;             // USART1's vector
Meter probe_meter;             // bound to nothing: letter r's stamp line
TickMeter probe_tick_meter;    // bound to nothing: letter r's stamp.tick line

/// The counters of the bound meters and the idle adapter. Read
/// quiescent, before an operation starts and after it has completed;
/// this core moves an aligned word in one access (atomic_width 4), so no
/// guard is wanted.
BenchCounters counters() { return bench_counters<Plat>(tick_meter, usart_meter); }

/// The console silent: the transport's own verb, the ring empty and the
/// last frame out of the shifter (TC) in one.
void console_drain() {
    while (!Serial::tx_idle()) {
    }
}

/// Spin to the start of a tick period: a whole millisecond of quiet
/// follows, in which no interrupt is pending.
void fresh_tick() {
    const uint32_t t = Ticker::ticks();
    while (Ticker::ticks() == t) {
    }
}

/// Every field over `units`: letter r's averaged lines.
BenchSample per_unit(const BenchSample& s, uint32_t units) {
    return {s.wall / units, s.busy / units, s.irq / units, s.isr / units};
}

BenchSample add(const BenchSample& a, const BenchSample& b) {
    return {a.wall + b.wall, a.busy + b.busy, a.irq + b.irq, a.isr + b.isr};
}

// ---------------------------------------------------------------------------
// The strings letter p prints: 62 letters and a CR LF to a line of 64,
// NUL-terminated, word-aligned.
// ---------------------------------------------------------------------------
template <uint16_t N>
struct Filler {
    alignas(4) char text[N + 1u];
};

template <uint16_t N>
constexpr Filler<N> make_filler() {
    Filler<N> f{};
    for (uint16_t i = 0; i < N; ++i) {
        const uint16_t col = static_cast<uint16_t>(i % 64u);
        f.text[i] = col == 62u ? '\r' : col == 63u ? '\n' : static_cast<char>('a' + col % 26u);
    }
    f.text[N] = '\0';
    return f;
}

template <uint16_t N>
constexpr Filler<N> filler = make_filler<N>();

// ---------------------------------------------------------------------------
// r - the ruler's self-check and the instrument's cost
// ---------------------------------------------------------------------------
volatile uint32_t ruler_sink = 0;

/// `n` ruler reads spelled out, each stored so that none is dropped.
template <uint8_t n>
[[gnu::always_inline]] inline void ruler_reads() {
    if constexpr (n > 0u) {
        ruler_sink = Ruler::now();
        ruler_reads<n - 1u>();
    }
}

/// `n` stamp pairs with nothing between, as a vector would take them:
/// the barrier keeps the compiler from folding one pair's stores into
/// the next.
template <uint8_t n, typename M>
[[gnu::always_inline]] inline void stamp_pairs(M& meter) {
    if constexpr (n > 0u) {
        meter.enter();
        meter.leave();
        __asm__ volatile("" ::: "memory");
        stamp_pairs<n - 1u>(meter);
    }
}

/// Ten batches of 100 units, each from a fresh tick: the run's total.
template <typename Unit>
BenchSample quiet_batches(Unit ten_units) {
    BenchSample total{0, 0, 0, 0};
    for (uint8_t batch = 0; batch < 10u; ++batch) {
        fresh_tick();
        const BenchCounters before = counters();
        Stopwatch<Ruler> sw;
        sw.start();
        for (uint8_t i = 0; i < 10u; ++i) {
            ten_units();
        }
        const uint32_t wall = sw.elapsed();
        total = add(total, bench_sample(wall, before, counters()));
    }
    return total;
}

void tr_ruler() {
    constexpr uint32_t asked_us = 999;
    constexpr uint32_t due = asked_us * (Ruler::hz() / 1'000'000u);

    console_drain();
    fresh_tick();
    const BenchCounters b = counters();
    Stopwatch<Ruler> sw;
    sw.start();
    const bool served = delay_us(clock, asked_us);
    const uint32_t took = sw.elapsed();
    const BenchSample d = bench_sample(took, b, counters());
    print(serial, "  delay_us(clock, ", asked_us, ") ", served ? "served" : "REFUSED", ": ", took,
          " cycles on the ruler, ", due, " due, ", d.irq, " interrupts in it", crlf);
    bench.verdict("999 us on the ruler is at least the due cycles (at least, never early)",
                  served && took >= due);
    bench.verdict("and under the due cycles + 5 per cent", took < due + due / 20u);

    console_drain();
    const BenchSample reads = quiet_batches([] { ruler_reads<10>(); });
    print(serial, "  ruler: 1000 reads, ten batches of 100 each from a fresh tick, ", reads.irq,
          " interrupts in the run", crlf);
    bench_line(serial, "ruler", 0, per_unit(reads, 1000u), Ruler::hz(), 0);

    console_drain();
    BenchSample empty{0xFFFF'FFFFu, 0, 0, 0};
    for (uint8_t run = 0; run < 8u; ++run) {
        fresh_tick();
        const BenchCounters before = counters();
        sw.start();
        const uint32_t w = sw.elapsed();
        const BenchSample s = bench_sample(w, before, counters());
        if (s.wall < empty.wall) {
            empty = s;
        }
    }
    print(serial, "  stopwatch: start() then elapsed() with nothing between, the best of 8", crlf);
    bench_line(serial, "stopwatch", 0, empty, Ruler::hz(), 0);

    console_drain();
    const BenchSample pairs = quiet_batches([] { stamp_pairs<10>(probe_meter); });
    print(serial, "  stamp: 1000 enter()+leave() pairs on the Ruler, batched the same, ",
          pairs.irq, " interrupts in the run", crlf);
    bench_line(serial, "stamp", 0, per_unit(pairs, 1000u), Ruler::hz(), 0);

    console_drain();
    const BenchSample tick_pairs = quiet_batches([] { stamp_pairs<10>(probe_tick_meter); });
    print(serial, "  stamp.tick: 1000 pairs on the TickRuler (the tick vector's), batched the same, ",
          tick_pairs.irq, " interrupts in the run", crlf);
    bench_line(serial, "stamp.tick", 0, per_unit(tick_pairs, 1000u), Ruler::hz(), 0);

    console_drain();
    fresh_tick();
    const BenchCounters before = counters();
    sw.start();
    uint32_t turns = 0;
    uint32_t wall = 0;
    do {
        {
            Plat::CriticalSection cs;
            Plat::idle();
        }
        ++turns;
        wall = sw.elapsed();
    } while (wall < Ruler::hz() / 10u);
    const BenchSample window = bench_sample(wall, before, counters());
    print(serial, "  window: ", turns, " idle turns in 100 ms, ", window.irq,
          " interrupts; busy and isr hold the tick handler", crlf);
    bench_line(serial, "window", 0, per_unit(window, turns), Ruler::hz(), 0);

    bench.verdict("ran", true);
}

// ---------------------------------------------------------------------------
// m - the runtime's copy and fill
// ---------------------------------------------------------------------------
alignas(4) uint8_t buffer_a[4096];
alignas(4) uint8_t buffer_b[4096];

enum class MemOp : uint8_t { copy, fill };

/// The best of 8 runs of one operation. The length and the pointers pass
/// through an empty asm so that the compiler sees neither as a constant:
/// a small constant memcpy would be expanded inline, and this letter
/// measures the runtime's function.
template <MemOp op>
BenchSample mem_best(uint32_t n) {
    BenchSample best{0xFFFF'FFFFu, 0, 0, 0};
    for (uint8_t run = 0; run < 8u; ++run) {
        uint8_t* dst = buffer_b;
        const uint8_t* src = buffer_a;
        size_t len = n;
        __asm__ volatile("" : "+r"(len), "+r"(dst), "+r"(src));
        const BenchCounters before = counters();
        Stopwatch<Ruler> sw;
        sw.start();
        if constexpr (op == MemOp::fill) {
            memset(dst, 0x5A, len);
        } else {
            memcpy(dst, src, len);
        }
        const uint32_t wall = sw.elapsed();
        __asm__ volatile("" ::: "memory");   // the bytes are read: nothing above is dead
        const BenchSample s = bench_sample(wall, before, counters());
        if (s.wall < best.wall) {
            best = s;
        }
    }
    return best;
}

constexpr uint32_t mem_sizes[4] = {1, 16, 256, 4096};

// The floors of the header: bytes per HCLK cycle times the rate.
constexpr uint32_t wire_copy = 2u * Ruler::hz();
constexpr uint32_t wire_fill = 4u * Ruler::hz();

void tm_memory() {
    console_drain();
    for (const uint32_t n : mem_sizes) {
        bench_line(serial, "memcpy", n, mem_best<MemOp::copy>(n), Ruler::hz(), wire_copy);
    }
    for (const uint32_t n : mem_sizes) {
        bench_line(serial, "memset", n, mem_best<MemOp::fill>(n), Ruler::hz(), wire_fill);
    }
    bench.verdict("ran", true);
}

// ---------------------------------------------------------------------------
// p - a print through the console
// ---------------------------------------------------------------------------
template <uint16_t N>
void print_one_size(uint32_t wire_bps) {
    console_drain();
    const BenchCounters before = counters();
    Stopwatch<Ruler> sw;
    sw.start();
    print(serial, filler<N>.text);
    console_drain();
    const uint32_t wall = sw.elapsed();
    const BenchSample s = bench_sample(wall, before, counters());
    print(serial, crlf);
    bench_line(serial, "print", N, s, Ruler::hz(), wire_bps);
}

void tp_print() {
    // The rate the line runs at, from the divisor in the register, over
    // the ten bits of an 8N1 frame.
    const uint32_t baud = Serial::actual_baud(SysClock::pclk2_hz);
    const uint32_t wire_bps = baud / 10u;
    print(serial, "  console at ", baud, " baud, 8N1: ", wire_bps, " B/s", crlf);
    print_one_size<1>(wire_bps);
    print_one_size<16>(wire_bps);
    print_one_size<256>(wire_bps);
    print_one_size<4096>(wire_bps);
    bench.verdict("ran", true);
}

// ---------------------------------------------------------------------------
// t - the tick's floor
// ---------------------------------------------------------------------------
void tt_tick() {
    console_drain();
    fresh_tick();
    const BenchCounters before = counters();
    Stopwatch<Ruler> sw;
    sw.start();
    uint32_t wall = 0;
    do {
        {
            Plat::CriticalSection cs;
            Plat::idle();
        }
        wall = sw.elapsed();
    } while (wall < Ruler::hz());
    bench_line(serial, "tick", 0, bench_sample(wall, before, counters()), Ruler::hz(), 0);
    bench.verdict("ran", true);
}

void banner() {
    print(serial, crlf, "bench_vx03 - ", device::part_name,
          " (clk=144 MHz PLL, ruler=STK cycles at 144 MHz, tick=STK 1000 Hz, console=USART1 ",
          console_baud, " 8N1)", crlf);
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------
// Each vector carries its meter's stamps around the ISR body the suites
// bind: enter() first, leave() last.
extern "C" BRIO_CH32_INTERRUPT void systick_handler() {
    tick_meter.enter();
    brio::Ticker::tick();
    tick_meter.leave();
}

extern "C" BRIO_CH32_INTERRUPT void usart1_handler() {
    usart_meter.enter();
    (void)Serial::isr();
    usart_meter.leave();
}

int main() {
    const bool clock_ok = SysClock::init();            // HSI x18 -> 144 MHz
    const bool serial_ok = Serial::init(clock, console_baud);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    bench.letter('r', "the ruler's self-check and the instrument's cost", tr_ruler);
    bench.letter('m', "memcpy and memset of the runtime against the core's floor", tm_memory);
    bench.letter('p', "a print through the console against the wire", tp_print);
    bench.letter('t', "the tick's floor: one second of idle turns", tt_tick);

    // Guarded: print() BLOCKS until the transport accepts each byte, so
    // printing into a port that failed to come up would never return.
    if (!serial_ok) {
        for (;;) {
        }
    }
    brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL144" : "FAILED", " tick=",
                tick_ok ? "STK" : "FAILED", brio::crlf);
    banner();
    bench.prompt();

    for (;;) {
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            // Nothing typed: sleep as the kernel's loop does, re-checked
            // under the mask.
            Plat::CriticalSection cs;
            if (Serial::rx_pending() == 0u) {
                Plat::idle();
            }
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
        brio::print(serial, "  stack: ", brio::stack_untouched(), " B never touched", brio::crlf);
        bench.prompt();
    }
}
