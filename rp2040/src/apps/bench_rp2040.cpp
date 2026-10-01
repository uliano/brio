// bench_rp2040 - the benchmark skeleton on the RP2040 (docs/design/
// benchmark.md, util/bench.hpp): four letters that print NUMBERS, one
// `bench` line per operation and size, in the grammar every family
// prints. NOT A TEST: a letter's one verdict is "ran", except letter r,
// whose verdicts judge the ruler every other line is read with.
//
// NOTHING TO WIRE. The console is the Debug Probe's UART bridge on GP0
// (TX) / GP1 (RX) = UART0 under function 2, 115200 8N1 - the binding of
// console.cpp and test_rp2040_platform.cpp, and the TestBench frame and
// polled prompt loop of the latter. No kernel and no AO: the letters
// are plain functions and the idle path is called by hand, so what is
// measured is the transport, the runtime and the idle path, never a
// dispatch.
//
// THE CLOCK: the 12 MHz crystal through the PLL to 125 MHz, clk_peri =
// clk_sys (rp2040/clock.hpp), the rate test_rp2040_platform runs at.
//
// THE RULER is `Ruler` below: core 0's SysTick ticker read as cycles
// (cortexm/ticker.hpp's BasicTicker::cycles(), the tick count and
// SysTick's position in its period composed by util/cycle_count.hpp),
// so it counts clk_sys cycles and hz() is SysClock::hz. Its now() is
// always_inline AND flatten, so the whole read - six register and
// memory loads (LOAD, the count, ICSR, VAL, ICSR, the count) and the
// compose - lands inline in every vector and in every loop that stamps,
// with no call. (flatten is the release build's: the debug preset's
// -fno-inline switches it off and the vectors there call cycles().)
//
// THE PLATFORM the idle path runs on is brio::BenchIdle<Rp2040Platform<>,
// Ruler> (`Idle`): every idle turn is a masked call of Idle::idle(),
// which stamps the window around the platform's WFI.
//
// THE METERS: one IsrMeter per bound vector - UART0's (`uart_meter`, on
// Ruler) and SysTick's (`tick_meter`). The SysTick vector's meter runs on
// `TickRuler`, SysTick's position in its period, and NOT on Ruler: inside
// the SysTick handler, before Ticker::tick() has counted the tick, the
// exception is ACTIVE and no longer pending (ICSR.PENDSTSET reads 0) while
// the count still holds the previous tick - so cycles() reads one whole
// period behind there and right again after the increment, and a meter
// on it would charge every tick a period (125 000 cycles). Both stamps of
// the tick vector fall inside one period (the handler runs right after
// the reload that pended it and lasts some hundred cycles; no masked
// section of this app approaches a period), so the position alone gives
// their difference exactly - with one load where Ruler takes six.
//
// What each letter prints:
//
//   r  THE RULER'S SELF-CHECK. delay_us(clock, 999) read on the ruler:
//      verdicts "at least hz x 999 us" and "under that + 5 per cent", and
//      the same wait read on the system timer (rp2040/timer.hpp, 1 us
//      ticks from the crystal, independent of clk_sys) as a third
//      verdict, because the ruler and delay_us count the SAME SysTick
//      and so cannot catch a wrong clk_sys between them. 999 and not
//      1000: cortexm/delay.hpp refuses a wait of one SysTick period or
//      more, and a period is one millisecond here (the line printed
//      shows the refusal). Then the instrument's cost, four bench lines
//      with n=0 and wire=0. Three are the AVERAGE of the run named,
//      every field divided by the units and rounded to the nearest (the
//      raw totals printed on the line above):
//        ruler     one Ruler::now(), over 1000 reads in a loop (the
//                  loop's increment, test and branch and a volatile store
//                  of the value are in it - about six cycles);
//        stamp     one enter()+leave() pair of a namespace-scope IsrMeter
//                  on Ruler with nothing between, over 1000 pairs, a
//                  compiler barrier between pairs so each pair loads and
//                  stores the meter's sums as a handler does: irq=1, isr =
//                  what the meter charges an EMPTY body, the floor under
//                  UART0's isr (the tick meter's floor is one VAL load and
//                  a subtract, a few cycles);
//        window    one idle turn (the masked call, the window's two
//                  stamps, the WFI, the tick that ends it, the loop's
//                  test on the Stopwatch) over the turns of 100 ms: wall
//                  is one tick period, irq=1, isr is the TICK HANDLER's
//                  cycles - it ran inside the window, so busy holds it -
//                  and busy - isr is the turn's own cost;
//      and one is the BEST of 8, as letter m's lines are:
//        stopwatch an empty Stopwatch interval, start() then elapsed()
//                  with nothing between: the floor every Stopwatch wall
//                  of letters m, p and t holds beyond its operation, the
//                  tail of one ruler read and the head of the next.
//   m  memcpy and memset (brio/rt/rt.cpp, the word path) of 1, 16, 256
//      and 4096 bytes between two word-aligned static buffers in the
//      striped SRAM, each the BEST of 8 runs on a Stopwatch, the length
//      read from a volatile so the call is the runtime's and not an
//      inlined copy. No idle and no interrupt expected: busy = wall,
//      irq and isr whatever tick landed in the best run.
//   p  a print of 1, 16, 256 and 4096 bytes - print(serial, s) with s a
//      static string in flash of that length (rows of 62 digits and a
//      CRLF; the 1-byte one is the final LF) - then a spin on
//      Serial::tx_idle(), the transport's own "ring empty and the
//      shifter finished" (UARTFR.BUSY clear, which stands until the last
//      stop bit has left the shift register). Counters before the print
//      and after the drain. busy = wall (the print and the drain spin);
//      irq and isr are the transport's shape: from an idle transmitter
//      the first FIFO-depth bytes go straight into the FIFO, then one
//      handler per refill as the FIFO falls through its level; a byte the
//      full 256-byte ring refuses writes nothing (pl011/uart.hpp).
//   t  the tick's floor: one second (hz cycles on the ruler) of masked
//      Idle::idle() turns with the console drained - not a kernel loop:
//      no kernel runs here. wall = the second, irq = the ticks, isr = the
//      tick handler's cycles, busy = the floor. n=0, wire=0.
//
// THE WIRES (the `wire` field, bytes per second, and why it is the
// limit):
//   print   115200 baud over the ten bits of an 8N1 frame (a start bit,
//           eight data bits, a stop bit): 11 520 B/s. The divisor gives
//           115 207 baud at 125 MHz (IBRD 67, FBRD 52, 4.2.7.1); the
//           configured rate is the figure.
//   memcpy  2 bytes a cycle x hz = 250 000 000 B/s. The Cortex-M0+ has
//           one AHB-Lite master port for every access below 0xd0000000
//           (datasheet 2.4.3.4), and table 81 of 2.4.3.3 gives LDM and
//           STM 1+N cycles for N words: a word copied is at least one
//           beat in and one beat out, the +1 per instruction and the
//           loop being the implementation's. The SRAM answers each beat
//           in one cycle (2.1.1.1, 2.6.2: zero wait states, word-striped
//           banks).
//   memset  4 bytes a cycle x hz = 500 000 000 B/s: one STM beat per
//           word (the same table), nothing loaded.
//   r and t wire=0: an instrument's cost and an idle second carry no
//           bytes.
//
// THE INSTRUMENT'S COST, and how a reader subtracts it: every line is
// RAW. A Stopwatch wall holds the `stopwatch` line's wall beyond the
// operation; UART0's isr holds the `stamp` line's isr per interrupt (what
// an empty body is charged) and busy the whole `stamp` wall per
// interrupt; an idle turn's busy holds the `window` line's busy - isr.
// Two seams util/bench.hpp states are left open: entry and
// exit latency (15 cycles worst case for the entry, 2.4.3.6.1) count as
// idle, and a handler landing between the WFI's return and the window's
// close counts in both.
//
// build: boards = pico,picow,weact2040
// build: monitor_speed = 115200

#include <stdint.h>

#include <array>
#include <cstring>

#include "rp2040/clock.hpp"
#include "rp2040/delay.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/ticker.hpp"
#include "rp2040/timer.hpp"
#include "rp2040/uart.hpp"
#include "util/bench.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 125'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;  // rings 64/256 (defaults)
constexpr Serial serial;

constexpr uint32_t console_baud = 115200;

// ---- the ruler ---------------------------------------------------------------

/// Core 0's SysTick ticker read as clk_sys cycles (the file header).
struct Ruler {
    [[gnu::always_inline, gnu::flatten]] static uint32_t now() { return CoreTicker<0>::cycles(); }
    static constexpr uint32_t hz() { return SysClock::hz; }
};
static_assert(CycleRuler<Ruler>);

/// The SysTick vector's ruler: SysTick's position in its period, exact
/// for a difference of two reads inside one period (the file header says
/// why the tick vector cannot use Ruler).
struct TickRuler {
    static constexpr uint32_t last = SysClock::hz / Ticker::ticks_per_second - 1u;
    [[gnu::always_inline]] static uint32_t now() { return last - SysTick->VAL; }
    static constexpr uint32_t hz() { return SysClock::hz; }
};
static_assert(CycleRuler<TickRuler>);

using Idle = BenchIdle<Rp2040Platform<>, Ruler>;
static_assert(Platform<Idle>);

// One meter per bound vector, and the bare one letter r times.
IsrMeter<Ruler, Idle> uart_meter;
IsrMeter<TickRuler, Idle> tick_meter;
IsrMeter<Ruler, Idle> empty_meter;

BenchCounters counters() { return bench_counters<Idle>(uart_meter, tick_meter); }

TestBench<Serial> bench;

// ---- the wires (the file header) ---------------------------------------------

constexpr uint32_t print_wire_bps = console_baud / 10u;
constexpr uint32_t memcpy_wire_bps = 2u * SysClock::hz;
constexpr uint32_t memset_wire_bps = 4u * SysClock::hz;

constexpr std::array<uint32_t, 4> sizes{1u, 16u, 256u, 4096u};
constexpr uint32_t max_size = 4096u;

/// Let the console fall silent: its interrupt would otherwise land in
/// whatever is measured next.
void drain() {
    while (!Serial::tx_idle()) {
    }
}

/// A run's numbers divided by its units, rounded to the nearest.
BenchSample per_unit(const BenchSample& s, uint32_t units) {
    const auto avg = [units](uint32_t v) { return (v + units / 2u) / units; };
    return {avg(s.wall), avg(s.busy), avg(s.irq), avg(s.isr)};
}

void print_totals(uint32_t units, const char* what, const BenchSample& s) {
    print(serial, "  over ", units, ' ', what, ": wall=", s.wall, " busy=", s.busy, " irq=", s.irq,
          " isr=", s.isr, crlf);
}

/// The best (shortest wall) of 8 runs of `op` on a Stopwatch; the barrier
/// after each run keeps what op wrote alive.
template <typename Op>
BenchSample best_of_8(Op op) {
    BenchSample best{};
    Stopwatch<Ruler> sw;
    for (uint8_t run = 0; run < 8u; ++run) {
        const BenchCounters c0 = counters();
        sw.start();
        op();
        const uint32_t wall = sw.elapsed();
        const BenchSample s = bench_sample(wall, c0, counters());
        asm volatile("" ::: "memory");
        if (run == 0u || s.wall < best.wall) {
            best = s;
        }
    }
    return best;
}

// =============================================================================
// r - the ruler's self-check and the instrument's cost
// =============================================================================
constexpr uint32_t probe_us = 999u;
constexpr uint32_t probe_floor =
    static_cast<uint32_t>(static_cast<uint64_t>(SysClock::hz) * probe_us / 1'000'000u);
constexpr uint32_t reps = 1000u;

void tr_ruler() {
    drain();
    (void)delay_us(clock, probe_us);   // the wait's code into the XIP cache
    const bool whole_ms_refused = !delay_us(clock, 1000u);

    Stopwatch<Ruler> sw;
    const uint32_t us0 = Timer::now_low();
    sw.start();
    const bool served = delay_us(clock, probe_us);
    const uint32_t took = sw.elapsed();
    const uint32_t us = Timer::now_low() - us0;
    print(serial, "  delay_us(clock, ", probe_us, ") served=", served, ": ", took,
          " cycles on the ruler (floor ", probe_floor, "), ", us,
          " us on the system timer; delay_us(clock, 1000) refused=", whole_ms_refused, crlf);
    bench.verdict("the ruler reads delay_us(999) at least hz x 999 us", served && took >= probe_floor);
    bench.verdict("and under that + 5 per cent", took <= probe_floor + probe_floor / 20u);
    bench.verdict("the system timer (the crystal, not clk_sys) reads the same wait within 5 per cent",
                  us + 1u >= probe_us && us <= probe_us + probe_us / 20u + 1u);

    // ruler: one read
    {
        volatile uint32_t sink = 0;
        const BenchCounters c0 = counters();
        sw.start();
        for (uint32_t i = 0; i < reps; ++i) {
            sink = Ruler::now();
        }
        const uint32_t wall = sw.elapsed();
        const BenchSample s = bench_sample(wall, c0, counters());
        (void)sink;
        print_totals(reps, "reads", s);
        bench_line(serial, "ruler", 0u, per_unit(s, reps), Ruler::hz(), 0u);
    }

    // stamp: one enter()+leave() pair, nothing between
    {
        const BenchCounters c0 = bench_counters<Idle>(uart_meter, tick_meter, empty_meter);
        sw.start();
        for (uint32_t i = 0; i < reps; ++i) {
            empty_meter.enter();
            empty_meter.leave();
            asm volatile("" ::: "memory");
        }
        const uint32_t wall = sw.elapsed();
        const BenchSample s =
            bench_sample(wall, c0, bench_counters<Idle>(uart_meter, tick_meter, empty_meter));
        print_totals(reps, "pairs", s);
        bench_line(serial, "stamp", 0u, per_unit(s, reps), Ruler::hz(), 0u);
    }

    // stopwatch: an empty interval, best of 8
    bench_line(serial, "stopwatch", 0u, best_of_8([] {}), Ruler::hz(), 0u);

    // window: one idle turn with nothing pending but the tick
    {
        drain();
        const uint32_t turns0 = Idle::idle_turns();
        const BenchCounters c0 = counters();
        sw.start();
        while (sw.elapsed() < Ruler::hz() / 10u) {
            disable_interrupts();
            Idle::idle();
        }
        const uint32_t wall = sw.elapsed();
        const BenchSample s = bench_sample(wall, c0, counters());
        const uint32_t turns = Idle::idle_turns() - turns0;
        print_totals(turns, "turns", s);
        bench_line(serial, "window", 0u, per_unit(s, turns), Ruler::hz(), 0u);
    }
    bench.verdict("ran", true);
}

// =============================================================================
// m - memcpy and memset
// =============================================================================
alignas(4) uint8_t mem_src[max_size];
alignas(4) uint8_t mem_dst[max_size];
volatile uint32_t mem_len = 0;

void tm_memory() {
    for (uint32_t i = 0; i < max_size; ++i) {
        mem_src[i] = static_cast<uint8_t>(i);
    }
    for (const uint32_t n : sizes) {
        mem_len = n;
        const uint32_t len = mem_len;
        const BenchSample s = best_of_8([len] { std::memcpy(mem_dst, mem_src, len); });
        bench_line(serial, "memcpy", n, s, Ruler::hz(), memcpy_wire_bps);
    }
    for (const uint32_t n : sizes) {
        mem_len = n;
        const uint32_t len = mem_len;
        const BenchSample s = best_of_8([len] { std::memset(mem_dst, 0x5A, len); });
        bench_line(serial, "memset", n, s, Ruler::hz(), memset_wire_bps);
    }
    bench.verdict("ran", true);
}

// =============================================================================
// p - a print through the console
// =============================================================================
/// The payload: rows of 62 digits and a CRLF, NUL-terminated; the string
/// of length n is its last n bytes.
constexpr std::array<char, max_size + 1u> payload = [] {
    std::array<char, max_size + 1u> t{};
    for (uint32_t i = 0; i < max_size; ++i) {
        const uint32_t col = i % 64u;
        t[i] = col == 62u ? '\r' : col == 63u ? '\n' : static_cast<char>('0' + (i / 64u + col) % 10u);
    }
    t[max_size] = '\0';
    return t;
}();

void tp_print() {
    Stopwatch<Ruler> sw;
    for (const uint32_t n : sizes) {
        drain();
        const char* text = payload.data() + (max_size - n);
        const BenchCounters c0 = counters();
        sw.start();
        print(serial, text);
        while (!Serial::tx_idle()) {
        }
        const uint32_t wall = sw.elapsed();
        const BenchSample s = bench_sample(wall, c0, counters());
        print(serial, crlf);
        bench_line(serial, "print", n, s, Ruler::hz(), print_wire_bps);
    }
    bench.verdict("ran", true);
}

// =============================================================================
// t - the tick's floor
// =============================================================================
void tt_tick() {
    drain();
    Stopwatch<Ruler> sw;
    const BenchCounters c0 = counters();
    sw.start();
    while (sw.elapsed() < Ruler::hz()) {
        disable_interrupts();
        Idle::idle();
    }
    const uint32_t wall = sw.elapsed();
    const BenchSample s = bench_sample(wall, c0, counters());
    bench_line(serial, "tick", 0u, s, Ruler::hz(), 0u);
    bench.verdict("ran", true);
}

void banner() {
    print(serial, crlf, "bench_rp2040 - the benchmark skeleton (util/bench.hpp), clk=", SysClock::hz,
          " Hz, console UART0 ", console_baud, " 8N1, ruler SysTick cycles", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
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
    const bool timer_ok = brio::Timer::init(clock);
    const bool serial_ok = Serial::init(clock, console_baud);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    bench.letter('r', "the ruler's self-check and the instrument's cost", tr_ruler);
    bench.letter('m', "memcpy and memset, 1/16/256/4096 bytes", tm_memory);
    bench.letter('p', "a print of 1/16/256/4096 bytes through the console", tp_print);
    bench.letter('t', "the tick's floor: one second of idle", tt_tick);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL125" : "FAILED",
                    " timer=", timer_ok ? "1us" : "FAILED", " tick=",
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
