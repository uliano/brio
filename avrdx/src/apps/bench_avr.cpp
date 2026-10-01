// bench_avr - the benchmark skeleton on the AVR DA/DB (docs/design/
// benchmark.md, util/bench.hpp): four letters that print NUMBERS, one
// `bench` line per operation and size, in the grammar every family
// prints. NOT A TEST: a letter's one verdict is "ran", except letter r,
// whose verdicts judge the ruler every other line is read with.
//
// NOTHING TO WIRE. The console is USART2 on its ALT1 pins PF4 (TX) / PF5
// (RX) through the board's USB bridge - the binding of console.cpp and
// of test_avr_platform.cpp, the same Uart<2, Route::alt1> with its 64/256
// rings and its three vectors - at 115200 8N1, the rate every other
// family's bench console runs at (the console app and the platform suite
// run this port at 460800; a print at 460800 is this file's console_baud
// changed and nothing else). The TestBench frame and the polled prompt
// loop are test_avr_meter's. No kernel and no AO: the letters are plain
// functions and the idle path is called by hand, so what is measured is
// the transport, the runtime and the idle path, never a dispatch.
//
// THE CLOCK: CLK_PER at 24 MHz, the rate test_avr_platform runs at - the
// DB's crystal on PA0/PA1, or on a DA (no crystal oscillator) an external
// 24 MHz clock on PA0, with OSCHF at the same rate as the fallback either
// way (avrdx/clock.hpp); the banner says which one runs.
//
// THE RULER is `Ruler` below: TCB1 and TCB2 cascaded into one 32-bit
// counter of CLK_PER cycles (avrdx/tcb.hpp's CascadedCounter, the
// instrument test_avr_platform takes its timings with), wrapping at 2^32
// - 178 s at 24 MHz. TCB1 counts CLK_PER; its overflow event on channel
// 4 clocks TCB2; a software event on channel 5 latches both halves into
// their CCMP registers (TCB2 through CASCADE, one CLK_PER behind), and
// read() waits for both CAPT flags, bounded, then reads the two
// captures. The console takes USART2 and the timebase the RTC's PIT at
// 1024 Hz (avrdx/ticker.hpp), nothing else, so the two TCBs and the two
// channels are free. now() is always_inline AND
// flatten, so the whole read lands inline in every vector and in every
// loop that stamps, with no call; and it takes the platform's
// CriticalSection around read(), because the read is a sequence and not
// a load - a 16-bit CCMP is read through the TCB's one TEMP register,
// and a handler that latches a new snapshot between the thread's two
// half reads would hand it the high half of another instant. Inside a
// vector the mask is already down (no interrupt nests on this core), and
// the guard costs its IN, CLI and OUT. hz() is SysClock::hz.
//
// The tick vector's meter takes the same ruler. The SysTick families need
// a second one there, because their ruler IS the ticker's count composed
// with its counter and reads one period low inside the tick handler;
// this ruler is two timers the PIT never touches, so a stamp in the PIT
// vector reads what a stamp anywhere else reads.
//
// THE PLATFORM the idle path runs on is brio::BenchIdle<AvrPlatform,
// Ruler> (`Idle`): every idle turn is cli() then Idle::idle(), which
// stamps the window around AvrPlatform::idle()'s SLEEP in IDLE mode.
//
// THE METERS: one IsrMeter per bound vector - USART2_RXC (`rxc_meter`),
// USART2_DRE (`dre_meter`), RTC_PIT (`tick_meter`) - all on Ruler, the
// enter() stamp the handler's first statement and leave() its last, the
// driver's always_inline body between them. The counters are read under
// AvrPlatform::CriticalSection: on an 8-bit core a 32-bit sum is four
// loads, and a handler may write it between two of them.
//
// What each letter prints:
//
//   r  THE RULER'S SELF-CHECK. delay_us(clock, 999) read on the ruler,
//      interrupts on: verdicts "at least the due cycles" (999 x 24) and
//      "under the due cycles + 5 per cent". The PIT's nominal period (32
//      cycles of a 32.768 kHz clock, 976.6 us) is shorter than the wait,
//      so a tick lands inside it unless the internal 32 kHz oscillator
//      runs more than 2.3 per cent slow, and the line printed says what
//      the handlers took; this delay refuses nothing, 999 is the figure
//      every family's letter r reads. Then the instrument's cost, four
//      bench lines with n=0 and wire=0. Three are the AVERAGE of the run
//      named,
//      every field divided by the units and rounded to the nearest (the
//      raw totals printed on the line above):
//        ruler     one Ruler::now(), over 1000 reads in a loop (the
//                  loop's 16-bit count and branch and a volatile 32-bit
//                  store of the value - four STS - are in it);
//        stamp     one enter()+leave() pair of a namespace-scope IsrMeter
//                  with nothing between, over 1000 pairs, a compiler
//                  barrier between pairs so each pair loads and stores the
//                  meter's sums as a handler does: irq=1, isr = what the
//                  meter charges an EMPTY body, the floor under every
//                  vector's isr (both runs are functions of their own,
//                  so the loop holds nothing else live and the code is
//                  the code a vector carries);
//        window    one idle turn (cli, the window's two stamps, the
//                  SLEEP, the tick that ends it, the loop's test on the
//                  Stopwatch) over the turns of 100 ms: wall is one PIT
//                  period, irq=1, isr is the TICK HANDLER's cycles - it
//                  ran inside the window, so busy holds it - and busy -
//                  isr is the turn's own cost;
//      and one is the BEST of 8, as letter m's lines are:
//        stopwatch an empty Stopwatch interval, start() then elapsed()
//                  with nothing between: the floor every Stopwatch wall
//                  of letters m, p and t holds beyond its operation, the
//                  tail of one ruler read and the head of the next.
//   m  memcpy and memset of 1, 16, 256 and 4096 bytes between two static
//      buffers in SRAM, each the BEST of 8 runs on a Stopwatch, the length
//      read from a volatile so the call is the library's and not an
//      inlined copy. The routines are avr-libc's: the 32-bit runtime
//      (design/runtime.md) is not linked into an AVR image, and on this
//      core there is no word path to take - every load and store moves
//      one byte, so the buffers' alignment changes nothing. No idle and
//      no interrupt expected but the tick: busy = wall, irq and isr
//      whatever tick landed in the best run (the 4096-byte memcpy outlasts
//      a PIT period, so every one of its runs holds one).
//   p  a print of 1, 16, 256 and 4096 bytes - print(serial, s) with s a
//      static string in flash of that length (rows of 62 digits and a
//      CRLF; the 1-byte one is the final LF), then the DRAIN: the
//      transport has no verb for "the ring is empty and the shifter has
//      finished" (Uart::tx_idle() is the ring alone), so the drain is
//      tx_idle() and then the resource's TXCIF (27.5.5: the frame in the
//      shift register has gone and TXDATA holds nothing new), cleared
//      while the transmitter was idle just before the print and waited
//      for with a bounded spin. Counters before the print and after the
//      drain. busy = wall (the print and the drain spin); irq and isr are
//      the transport's shape - one DRE interrupt per byte, the silicon's
//      (TXDATA is one byte deep beside the shifter) - plus the ticks that
//      landed, the split printed on the line above.
//   t  the tick's floor: one second (hz cycles on the ruler) of cli()
//      then Idle::idle() turns with the console drained - not a kernel
//      loop: no kernel runs here. wall = the second, irq = the ticks
//      (1024 on the 32 kHz oscillator's nominal rate), isr = the tick
//      handler's cycles, busy = the floor. n=0, wire=0.
//
// THE WIRES (the `wire` field, bytes per second, and why it is the
// limit):
//   print   115200 baud over the ten bits of an 8N1 frame (a start bit,
//           eight data bits, a stop bit): 11 520 B/s. The fractional
//           generator makes the rate exact to the tick at 24 MHz
//           (BAUD = 64 x 24e6 / (16 x 115200) = 833.33, 833 written:
//           115 246 baud, 0.04 per cent over); the configured rate is the
//           figure.
//   memcpy  one byte per three cycles x hz = 8 000 000 B/s: a byte copied
//           is one LD from SRAM and one ST to it, 2 and 1 cycles on this
//           core (AVR Instruction Set Manual DS40002198B, table 5-4, the
//           AVRxt column, note 2: internal SRAM). The loop's count and
//           branch are the implementation's.
//   memset  one byte a cycle x hz = 24 000 000 B/s: one ST per byte
//           (the same table), nothing loaded.
//   r and t wire=0: an instrument's cost and an idle second carry no
//           bytes.
//
// THE INSTRUMENT'S COST, and how a reader subtracts it: every line is
// RAW. A Stopwatch wall holds the `stopwatch` line's wall beyond the
// operation; a vector's isr holds the `stamp` line's isr per interrupt
// (what an empty body is charged); an idle turn's busy holds the
// `window` line's busy - isr. What a metered vector costs OUTSIDE its
// two snapshots is not in isr at all, and on this core it is the larger
// part: the hardware entry, a prologue that pushes the registers the
// ruler's read and the meter's 32-bit sums occupy, the head of enter()'s
// read before its snapshot, the tail of leave()'s read after its
// snapshot with the three sums, the epilogue and RETI - counted as thread
// time when the handler preempted the thread and as IDLE when it ran in
// a window. Letter r's delay line measures that whole for the tick
// vector: when it names one handler, the delay's excess over the due
// count, less the `stopwatch` floor, is one tick handler all told. Two seams util/bench.hpp states
// are left open: the entry and exit latency (six cycles in, 15.3.2.3 and
// table 15-1 of DS40002247B, five more from sleep, four for RETI) count
// as idle, and a handler landing between the SLEEP's wake and the
// window's close counts in both.
//
// build: boards = db48,da48
// build: monitor_speed = 115200

#include <avr/interrupt.h>
#include <stdint.h>
#include <string.h>

#include <array>
#include <string_view>

#include "avrdx/clock.hpp"
#include "avrdx/delay.hpp"
#include "avrdx/evsys.hpp"
#include "avrdx/platform.hpp"
#include "avrdx/tcb.hpp"
#include "avrdx/ticker.hpp"
#include "avrdx/usart.hpp"
#include "avrdx/userrow.hpp"
#include "util/bench.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

// The DB's 24 MHz crystal, or the DA's external 24 MHz clock on PA0.
using SysClock = brio::Clock<brio::has_xoschf ? brio::ClockSource::crystal
                                              : brio::ClockSource::external,
                             24'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = AvrPlatform;
using Serial = Uart<2, Route::alt1>;  // rings 64/256 (defaults)
constexpr Serial serial;
using Usart2 = Serial::Resource;

constexpr uint32_t console_baud = 115200;
constexpr uint32_t frame_bits = 10;  // 8N1

// ---- the ruler ---------------------------------------------------------------

using WatchLo = Tcb<1>;
using WatchHi = Tcb<2>;
using Watch = CascadedCounter<WatchLo, WatchHi>;
using ChCarry = EventChannel<4>;
using ChSnap = EventChannel<5>;

/// TCB1 + TCB2 cascaded: CLK_PER cycles, 32 bits (the file header).
struct Ruler {
    [[gnu::always_inline, gnu::flatten]] static uint32_t now() {
        P::CriticalSection cs;
        return Watch::read();
    }
    static constexpr uint32_t hz() { return SysClock::hz; }
};
static_assert(CycleRuler<Ruler>);

using Idle = BenchIdle<P, Ruler>;
static_assert(Platform<Idle>);

// One meter per bound vector, and the bare one letter r times.
IsrMeter<Ruler, Idle> rxc_meter;
IsrMeter<Ruler, Idle> dre_meter;
IsrMeter<Ruler, Idle> tick_meter;
IsrMeter<Ruler, Idle> empty_meter;

/// The counters, read quiescent and under the mask (the file header).
BenchCounters counters() {
    P::CriticalSection cs;
    return bench_counters<Idle>(rxc_meter, dre_meter, tick_meter);
}

TestBench<Serial> bench;

// ---- the wires (the file header) ---------------------------------------------

constexpr uint32_t print_wire_bps = console_baud / frame_bits;
constexpr uint32_t memcpy_wire_bps = SysClock::hz / 3u;
constexpr uint32_t memset_wire_bps = SysClock::hz;

constexpr std::array<uint16_t, 4> sizes{1u, 16u, 256u, 4096u};
constexpr uint16_t max_size = 4096u;

// ---- the console's drain ------------------------------------------------------

/// CLK_PER cycles of one frame on the wire.
constexpr uint32_t frame_cycles = SysClock::hz / console_baud * frame_bits;
/// A spin count that outlasts several frames whatever a turn costs: every
/// turn of the loops below is at least four cycles.
constexpr uint16_t spin_bound = static_cast<uint16_t>(2u * frame_cycles);

/// TXCIF, bounded: false when the flag never rose.
bool wait_shifted() {
    for (uint16_t i = 0; i < spin_bound; ++i) {
        if (Usart2::txc_flag()) {
            return true;
        }
    }
    return false;
}

/// Let the console fall silent - the ring empty, the last byte through
/// the shifter - so its interrupt lands in nothing measured next. TXCIF
/// may stand from any earlier idle moment, so it is cleared once DREIF
/// says the last byte has entered the shifter (TXDATA empty: a frame
/// still to run) and then waited for. With nothing left to send the wait
/// runs out its bound, a few frames.
void drain() {
    while (!Serial::tx_idle()) {
    }
    for (uint16_t i = 0; i < spin_bound && !Usart2::dre_flag(); ++i) {
    }
    Usart2::clear_txc();
    (void)wait_shifted();
}

// ---- helpers --------------------------------------------------------------------

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
constexpr uint32_t probe_due = SysClock::hz / 1'000'000u * probe_us;
constexpr uint16_t reps = 1000u;

/// What the meters and the ruler print beside letter r's runs.
BenchCounters counters_with_empty() {
    P::CriticalSection cs;
    return bench_counters<Idle>(rxc_meter, dre_meter, tick_meter, empty_meter);
}

volatile uint32_t ruler_sink = 0;

// The two runs below are functions of their own, out of line: inside the
// letter's body the loop would share the registers of everything else
// live there, and the compiler would compose each read through the stack
// frame - a cost of that body, not of the read a vector carries.

/// 1000 reads of the ruler, each stored to a volatile word.
[[gnu::noinline]] BenchSample run_reads() {
    const BenchCounters c0 = counters();
    Stopwatch<Ruler> sw;
    sw.start();
    for (uint16_t i = 0; i < reps; ++i) {
        ruler_sink = Ruler::now();
    }
    const uint32_t wall = sw.elapsed();
    return bench_sample(wall, c0, counters());
}

/// 1000 stamp pairs of the bare meter with nothing between, a compiler
/// barrier between pairs so each pair loads and stores the meter's sums
/// as a handler does.
[[gnu::noinline]] BenchSample run_pairs() {
    const BenchCounters c0 = counters_with_empty();
    Stopwatch<Ruler> sw;
    sw.start();
    for (uint16_t i = 0; i < reps; ++i) {
        empty_meter.enter();
        empty_meter.leave();
        asm volatile("" ::: "memory");
    }
    const uint32_t wall = sw.elapsed();
    return bench_sample(wall, c0, counters_with_empty());
}

void tr_ruler() {
    drain();
    Stopwatch<Ruler> sw;
    {
        const BenchCounters c0 = counters();
        sw.start();
        delay_us(clock, probe_us);
        const uint32_t took = sw.elapsed();
        const BenchSample s = bench_sample(took, c0, counters());
        print(serial, "  delay_us(clock, ", probe_us, "): ", took, " cycles on the ruler (due ",
              probe_due, "), ", s.irq, " handler(s) inside it, ", s.isr, " cycles between their stamps",
              crlf);
        bench.verdict("the ruler reads delay_us(999) at least the due cycles", took >= probe_due);
        bench.verdict("and under the due cycles + 5 per cent", took <= probe_due + probe_due / 20u);
    }

    // ruler: one read
    drain();
    {
        const BenchSample s = run_reads();
        print_totals(reps, "reads", s);
        bench_line(serial, "ruler", 0u, per_unit(s, reps), Ruler::hz(), 0u);
    }

    // stamp: one enter()+leave() pair, nothing between
    drain();
    {
        const BenchSample s = run_pairs();
        print_totals(reps, "pairs", s);
        bench_line(serial, "stamp", 0u, per_unit(s, reps), Ruler::hz(), 0u);
    }

    // stopwatch: an empty interval, best of 8
    drain();
    bench_line(serial, "stopwatch", 0u, best_of_8([] {}), Ruler::hz(), 0u);

    // window: one idle turn with nothing pending but the tick
    {
        drain();
        const uint32_t turns0 = Idle::idle_turns();
        const BenchCounters c0 = counters();
        sw.start();
        while (sw.elapsed() < Ruler::hz() / 10u) {
            cli();
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
uint8_t mem_src[max_size];
uint8_t mem_dst[max_size];
volatile uint16_t mem_len = 0;

void tm_memory() {
    for (uint16_t i = 0; i < max_size; ++i) {
        mem_src[i] = static_cast<uint8_t>(i);
    }
    for (const uint16_t n : sizes) {
        mem_len = n;
        const uint16_t len = mem_len;
        drain();
        const BenchSample s = best_of_8([len] { memcpy(mem_dst, mem_src, len); });
        bench_line(serial, "memcpy", n, s, Ruler::hz(), memcpy_wire_bps);
    }
    for (const uint16_t n : sizes) {
        mem_len = n;
        const uint16_t len = mem_len;
        drain();
        const BenchSample s = best_of_8([len] { memset(mem_dst, 0x5A, len); });
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
    for (uint16_t i = 0; i < max_size; ++i) {
        const uint16_t col = i % 64u;
        t[i] = col == 62u ? '\r' : col == 63u ? '\n' : static_cast<char>('0' + (i / 64u + col) % 10u);
    }
    t[max_size] = '\0';
    return t;
}();

/// One meter's count and cycles at an instant, for the per-vector split.
struct Share {
    uint32_t irq;
    uint32_t cycles;
};

template <typename M>
Share share(const M& m) {
    P::CriticalSection cs;
    return {m.count(), m.cycles()};
}

void tp_print() {
    Stopwatch<Ruler> sw;
    for (const uint16_t n : sizes) {
        drain();
        Usart2::clear_txc();  // the transmitter is idle: the flag starts clean
        const char* text = payload.data() + (max_size - n);
        const Share dre0 = share(dre_meter);
        const Share tick0 = share(tick_meter);
        const BenchCounters c0 = counters();
        sw.start();
        print(serial, text);
        while (!Serial::tx_idle()) {
        }
        const bool shifted = wait_shifted();
        const uint32_t wall = sw.elapsed();
        const BenchSample s = bench_sample(wall, c0, counters());
        const Share dre1 = share(dre_meter);
        const Share tick1 = share(tick_meter);
        print(serial, crlf, "  dre: irq=", dre1.irq - dre0.irq, " isr=", dre1.cycles - dre0.cycles,
              "  tick: irq=", tick1.irq - tick0.irq, " isr=", tick1.cycles - tick0.cycles,
              shifted ? "" : "  (TXCIF never rose: the wall ends at the bound)", crlf);
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
        cli();
        Idle::idle();
    }
    const uint32_t wall = sw.elapsed();
    const BenchSample s = bench_sample(wall, c0, counters());
    bench_line(serial, "tick", 0u, s, Ruler::hz(), 0u);
    bench.verdict("ran", true);
}

bool xtal = false;

void banner() {
    std::string_view board = board_id();
    if (board.empty()) {
        board = "?";
    }
    print(serial, crlf, "bench_avr - the benchmark skeleton (util/bench.hpp), board ", board,
          ", clk=", xtal ? (has_xoschf ? "XTAL " : "EXTCLK ") : "OSCHF ", SysClock::hz,
          " Hz, console USART2 ALT1 ", console_baud,
          " 8N1, ruler TCB1+TCB2 cascade at CLK_PER", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
ISR(USART2_RXC_vect) {
    rxc_meter.enter();
    (void)Serial::rxc();
    rxc_meter.leave();
}
ISR(USART2_DRE_vect) {
    dre_meter.enter();
    Serial::dre();
    dre_meter.leave();
}
ISR(RTC_PIT_vect) {
    tick_meter.enter();
    brio::Ticker::pit();
    tick_meter.leave();
}

int main() {
    xtal = SysClock::init();
    Serial::init(clock, console_baud);
    brio::Ticker::init();
    Watch::init(brio::TcbClock::div1, ChCarry{}, ChSnap{});
    Watch::reset();
    sei();

    bench.letter('r', "the ruler's self-check and the instrument's cost", tr_ruler);
    bench.letter('m', "memcpy and memset, 1/16/256/4096 bytes", tm_memory);
    bench.letter('p', "a print of 1/16/256/4096 bytes through the console", tp_print);
    bench.letter('t', "the tick's floor: one second of idle", tt_tick);

    banner();
    bench.prompt();

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
