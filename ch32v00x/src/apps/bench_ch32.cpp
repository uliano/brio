// bench_ch32 - the benchmark skeleton on the CH32V00x (QingKe V2C,
// RV32EmC): the instrument's self-check and cost, the runtime's copy and
// fill against the core's floor, a print through the console transport
// against the wire, and the tick's floor - one line per operation and
// size, in util/bench.hpp's grammar (docs/design/benchmark.md).
//
// NOT A TEST. TestBench frames it (the menu, the ALL: line bin/brio
// waits for); every letter ends with one verdict "ran", and letter r's
// two verdicts on the ruler are the only ones that judge anything.
//
// THE BOARD AND THE RATES. The CH32V006K8U6 module at 48 MHz (the HSI's
// 24 MHz doubled by the PLL), the rate test_ch32_platform runs at. The
// console is USART1 on PD5/PD6 through the WCH-Link at 115200 8N1, the
// interrupt-driven transport with no DMA engine, bound as that suite
// binds it. The CH32V003 is outside this round: the app is built for the
// v006k8 alone, and the smallest-chip rule (design/overview.md) is noted
// and not applied - the 4 KB copy buffer below is twice that part's RAM.
//
// NO KERNEL. The suite's shape: a prompt loop over the console, which
// here sleeps through BenchIdle<P, Ruler>::idle() when no byte is
// pending, as the kernel's loop would. That adapter is the platform every
// type below names, the transport's rings included.
//
// THE RULER. Ticker::cycles() on the STK (ch32v00x/ticker.hpp): the tick
// count and the counter's position in its period composed by
// util/cycle_count.hpp, in HCLK cycles - so hz() is the clock's 48 MHz.
// A read is a CMP load, two loads each of the tick count and of SR, a
// CNT load, a multiply and an add. Ruler::now() is always_inline and
// flatten, so the whole read is spelled inline wherever a stamp or a
// Stopwatch takes one, the vectors included: no call on the hot path.
//
// THE METERS. Two vectors, one IsrMeter each: the STK's (the tick) and
// USART1's (the transport's ISR body). enter() is each handler's first
// statement and leave() its last, the body between them as the suite
// binds it. The hardware prologue's entry and exit, measured in
// docs/ch32v00x/platform.md, lie outside the stamps.
//
// What each letter prints:
//
//   r  THE RULER'S SELF-CHECK. delay_us(clock, 1000) is REFUSED on this
//      family, as every wait of one tick period or more is
//      (ch32v00x/delay.hpp), so the check reads TWO delay_us(clock, 500)
//      back to back on the ruler: at least hz/1000 cycles, and under
//      hz/1000 + 5 per cent.
//      Then the instrument's cost, four lines with n=0 and wire=0; in
//      the three averaged ones EVERY FIELD IS THE RUN'S TOTAL DIVIDED BY
//      ITS UNITS, and the note line before each states the units and the
//      run's interrupts:
//        ruler   one Ruler::now(): 1000 reads in ten batches of 100, each
//                batch started on a fresh tick so that no interrupt lands
//                in it; a batch's Stopwatch adds one read per hundred
//        stopwatch  an empty Stopwatch - start() then elapsed(), nothing
//                between - the best of 8: not averaged, because it is
//                exactly what every Stopwatch line of m, p and t holds
//                of the instrument
//        stamp   one enter()+leave() pair with nothing between, on a
//                meter bound to no vector: 1000 pairs, batched the same
//        window  one BenchIdle::idle() turn with nothing pending but the
//                tick, over the turns of 100 ms. wall is the turn (a tick
//                period) and busy HOLDS THE TICK HANDLER, added back
//                because it ran inside the window, beside the loop's own
//                turnaround; isr is the handler alone, so busy - isr is
//                what a turn costs besides it
//   m  memcpy and memset of the runtime (rt/rt.cpp), each the best of 8
//      runs on a Stopwatch, between static word-aligned buffers (the
//      runtime's word path). THE RAM BOUNDS THE SIZES: two 4096-byte
//      buffers would be the part's whole 8 KB, so
//        memcpy        RAM to RAM, 1, 16, 256 and 2048 bytes, between the
//                      two halves of one 4096-byte buffer
//        memcpy.flash  the 4096-byte string of letter p, in flash, into
//                      the buffer: 1, 16, 256 and 4096 bytes
//        memset        the buffer: 1, 16, 256 and 4096 bytes
//      Nothing idles and nothing is expected to interrupt: busy is wall,
//      and irq and isr are whatever the tick took in the best run. At 1
//      and 16 bytes a line is mostly the call and the stopwatch, not the
//      copy: x is large there by construction.
//   p  print(serial, s) of a static string of 1, 16, 256 and 4096 bytes
//      (62 letters and a CR LF to a line of 64), then the DRAIN: the
//      transport's tx_idle() (its ring empty) and then the resource's TC
//      flag (Usart<1>::tx_complete(): the last frame has left the
//      shifter) - the transport has no verb for the second half. The
//      counters are read before the print and after the drain. The print
//      spins on a full ring, so busy is close to wall; irq and isr are the
//      transport's per-byte shape - one TXE interrupt per byte, one more
//      that disarms TXEIE - and the ticks.
//   t  THE TICK'S FLOOR: one second of BenchIdle::idle() turns with
//      nothing to do (this loop, not the kernel's), counters before and
//      after: wall = the second in cycles, irq = the ticks, isr = the tick
//      handler's cycles, busy = the floor.
//
// THE WIRE, and why it is the limit:
//   memcpy, memset  the core's load/store floor. The QingKe V2 manual
//                   gives no instruction timing (table 1-1 states a
//                   two-stage pipeline and nothing per instruction), and
//                   the reference manual (RM 1.1) a system bus from the
//                   core through the bus matrix to the SRAM with no wait
//                   state named; the floor taken is that bus's own, one
//                   32-bit SRAM access per HCLK cycle: memset 4 bytes a
//                   cycle (a store a word), memcpy 2 (a load and a store a
//                   word) - 192 and 96 MB/s at 48 MHz. It is the bus's
//                   figure and not a documented instruction timing: x
//                   says what the pipeline, the loop and the instruction
//                   fetch from flash at two wait states cost above it.
//   memcpy.flash    a word read through the D-Code bus at one cycle and
//                   two wait states (RM 18.3.1: LATENCY = 2 above 24 MHz)
//                   and a word store: 4 bytes in 4 cycles, 48 MB/s.
//   print           the console's ACTUAL baud, read back from BRR (48 MHz
//                   over 417, 115107 baud), over the ten bits of an 8N1
//                   frame: 11510 B/s.
//
// THE INSTRUMENT'S COST, and how the reader subtracts it. Every line
// carries RAW numbers. A Stopwatch measurement holds the stopwatch line's
// wall - start()'s read's tail and elapsed()'s read's head, both spelled
// inline (util/bench.hpp forces them, as it forces the stamps): subtract
// it from every line of m, p and t. A handler's metered cycles
// hold about one ruler read as well (enter()'s tail and leave()'s head):
// subtract irq times the ruler line from isr. The stamp line is what the
// pair adds to the program's real cost per interrupt. And util/bench.hpp's
// two stated seams: the latency of entering and leaving a handler is
// counted as idle, and an interrupt landing between P::idle()'s return
// and the window's close is counted in both.
//
// build: boards = v006k8
// build: monitor_speed = 115200

#include <stdint.h>
#include <string.h>

#include <cstddef>

#include "ch32v00x/clock.hpp"
#include "ch32v00x/delay.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/ticker.hpp"
#include "ch32v00x/usart.hpp"
#include "util/bench.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 48'000'000>;
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

using Plat = BenchIdle<Ch32v00xPlatform<>, Ruler>;
using Meter = IsrMeter<Ruler, Plat>;

using Serial = Uart<1, Plat>;   // USART1 on PD5/PD6, rings 64/64
constexpr Serial serial;
constexpr uint32_t console_baud = 115200;

TestBench<Serial> bench;

Meter tick_meter;    // the STK's vector
Meter usart_meter;   // USART1's vector
Meter probe_meter;   // bound to nothing: letter r's stamp line

/// The counters of the bound meters and the idle adapter. Read
/// quiescent, before an operation starts and after it has completed;
/// this core moves an aligned word in one access (atomic_width 4), so no
/// guard is wanted.
BenchCounters counters() { return bench_counters<Plat>(tick_meter, usart_meter); }

/// The console silent: the ring empty, then the last frame out of the
/// shifter (the resource's TC; the transport has no verb for that half).
void console_drain() {
    while (!Serial::tx_idle()) {
    }
    while (!Serial::Resource::tx_complete()) {
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
// The strings letter p prints and letter m copies from flash: 62 letters
// and a CR LF to a line of 64, NUL-terminated, word-aligned so that the
// runtime's copy takes its word path from them.
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
template <uint8_t n>
[[gnu::always_inline]] inline void stamp_pairs() {
    if constexpr (n > 0u) {
        probe_meter.enter();
        probe_meter.leave();
        __asm__ volatile("" ::: "memory");
        stamp_pairs<n - 1u>();
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
    constexpr uint32_t ms = Ruler::hz() / 1000u;

    console_drain();
    const bool refused = !delay_us(clock, 1000);
    Stopwatch<Ruler> sw;
    sw.start();
    const bool first = delay_us(clock, 500);
    const bool second = delay_us(clock, 500);
    const uint32_t took = sw.elapsed();
    print(serial, "  delay_us(clock, 1000) ", refused ? "refused" : "SERVED",
          " (a wait of one tick period is, ch32v00x/delay.hpp): 2 x delay_us(500) = ", took,
          " cycles on the ruler, ", ms, " asked", crlf);
    bench.verdict("2 x 500 us on the ruler is at least hz/1000 cycles (at least, never early)",
                  first && second && took >= ms);
    bench.verdict("and under hz/1000 + 5 per cent", took < ms + ms / 20u);

    console_drain();
    const BenchSample reads = quiet_batches([] { ruler_reads<10>(); });
    print(serial, "  ruler: 1000 reads, ten batches of 100 each from a fresh tick, ", reads.irq,
          " interrupts in the run", crlf);
    bench_line(serial, "ruler", 0, per_unit(reads, 1000u), Ruler::hz(), 0);

    console_drain();
    BenchSample empty{0xFFFF'FFFFu, 0, 0, 0};
    for (uint8_t run = 0; run < 8u; ++run) {
        fresh_tick();
        const BenchCounters b = counters();
        sw.start();
        const uint32_t w = sw.elapsed();
        const BenchSample s = bench_sample(w, b, counters());
        if (s.wall < empty.wall) {
            empty = s;
        }
    }
    print(serial, "  stopwatch: start() then elapsed() with nothing between, the best of 8", crlf);
    bench_line(serial, "stopwatch", 0, empty, Ruler::hz(), 0);

    console_drain();
    const BenchSample pairs = quiet_batches([] { stamp_pairs<10>(); });
    print(serial, "  stamp: 1000 enter()+leave() pairs, batched the same, ", pairs.irq,
          " interrupts in the run", crlf);
    bench_line(serial, "stamp", 0, per_unit(pairs, 1000u), Ruler::hz(), 0);

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
alignas(4) uint8_t ram[4096];

enum class MemOp : uint8_t { copy_ram, copy_flash, fill };

/// The best of 8 runs of one operation. The length and the pointers pass
/// through an empty asm so that the compiler sees neither as a constant:
/// a small constant memcpy would be expanded inline, and this letter
/// measures the runtime's function.
template <MemOp op>
BenchSample mem_best(uint32_t n) {
    BenchSample best{0xFFFF'FFFFu, 0, 0, 0};
    for (uint8_t run = 0; run < 8u; ++run) {
        uint8_t* dst = op == MemOp::copy_ram ? ram + 2048 : ram;
        const uint8_t* src = op == MemOp::copy_ram
                                 ? ram
                                 : reinterpret_cast<const uint8_t*>(filler<4096>.text);
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
constexpr uint32_t wire_copy_ram = 2u * Ruler::hz();
constexpr uint32_t wire_copy_flash = 1u * Ruler::hz();
constexpr uint32_t wire_fill = 4u * Ruler::hz();

void tm_memory() {
    console_drain();
    for (const uint32_t n : mem_sizes) {
        const uint32_t half = n > 2048u ? 2048u : n;   // the RAM's bound: two halves
        bench_line(serial, "memcpy", half, mem_best<MemOp::copy_ram>(half), Ruler::hz(),
                   wire_copy_ram);
    }
    for (const uint32_t n : mem_sizes) {
        bench_line(serial, "memcpy.flash", n, mem_best<MemOp::copy_flash>(n), Ruler::hz(),
                   wire_copy_flash);
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
    const uint32_t wire_bps = Serial::actual_baud(SysClock::pclk_hz) / 10u;
    print(serial, "  console at ", Serial::actual_baud(SysClock::pclk_hz), " baud, 8N1: ", wire_bps,
          " B/s", crlf);
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
    print(serial, crlf, "bench_ch32 - ", device::part_name,
          " (clk=48 MHz PLL, ruler=STK cycles at 48 MHz, tick=STK 1000 Hz, console=USART1 ",
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
    const bool clock_ok = SysClock::init();            // HSI x2 -> 48 MHz
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
    brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL48" : "FAILED", " tick=",
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
