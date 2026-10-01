// bench_samc - the benchmark skeleton on the SAM C21 (docs/design/
// benchmark.md, util/bench.hpp): four letters that print NUMBERS, one
// `bench` line per operation and size, in the grammar every family
// prints. NOT A TEST: a letter's one verdict is "ran", except letter r,
// whose verdicts judge the ruler every other line is read with.
//
// NOTHING TO WIRE. The console is the board's CH340 bridge on PB30 (TX)
// / PB31 (RX) = SERCOM5 PAD[0]/PAD[1] under function D, 115200 8N1 - the
// binding of console.cpp and test_samc_platform.cpp, with the latter's
// TestBench frame and polled prompt loop. NO KERNEL AND NO AO: the
// letters are plain functions and the idle path is called by hand,
// masked, as the kernel's loop calls it - what is measured is the
// transport, the runtime and the idle path, never a dispatch.
//
// THE CLOCK: OSC48M undivided, 48 MHz on GCLK0 (samc21/clock.hpp), the
// flash at 2 wait states (DS60001479M table 45-41) behind NVMCTRL's
// 64-byte cache in its reset mode, NO_MISS_PENALTY (27.6.7, 27.8.2) - the
// rate test_samc_platform runs at.
//
// WHERE THE VECTORS RUN. SERCOM5_Handler and SysTick_Handler are bound
// through BENCH_PLACEMENT. It is empty in this file, so the handlers -
// and the ISR bodies they inline - execute from flash. bench_samc_ram.cpp
// is this same file with BENCH_RAM_TEXT defined, which binds both
// handlers through this family's documented option of the binding
// pattern: [[gnu::section(".ram_text")]] on the handler the app binds,
// the input section samc21/ld/samc21j18a.ld puts first in .data and the
// crt copies to SRAM (single-cycle at full speed, 9.1) - what the option
// is and what it buys are docs/samc21/platform.md's, "A handler in
// SRAM". Letter r's `stamp` loop carries the same placement, so in each
// image it is the cost of a stamp pair where the vectors' stamps run.
// Letters p and t of the two images are the comparison; nothing else
// moves between them.
//
// THE RULER is `Ruler`: the kernel ticker read as cycles (cortexm/
// ticker.hpp's BasicTicker::cycles() - SysTick's position in its 1 ms
// period composed with the tick count and the pending flag by util/
// cycle_count.hpp), so it counts CLK_CPU cycles and hz() is SysClock::hz,
// 48 000 000. now() is always_inline AND flatten: the whole read - SysTick
// LOAD, the tick count, ICSR, VAL, ICSR, the tick count again, and the
// compose - lands inline in every vector and every loop that stamps, with
// no call (the debug preset's -fno-inline switches flatten off: only the
// release image is the instrument).
//
// THE PLATFORM the idle path runs on is brio::BenchIdle<SamPlatform,
// Ruler> (`Idle`): every idle turn is a masked call of Idle::idle(),
// which stamps the window around the platform's DSB and WFI.
//
// THE METERS: one IsrMeter per bound vector. SERCOM5's (`sercom_meter`)
// runs on Ruler. SysTick's (`tick_meter`) runs on `TickRuler`, SysTick's
// position in its period (period - 1 - VAL) and NOT on Ruler: inside the
// SysTick handler, before Ticker::tick() has counted, the exception is
// active and no longer pending, so cycles() reads one whole period low
// there (cortexm/ticker.hpp's comment on cycles()) and a meter on it
// would charge every tick 48 000 cycles. Both stamps of the tick vector
// fall inside one period (the handler starts some tens of cycles after
// the reload that pended it and lasts under a hundred; no masked section
// of this app approaches a millisecond), so the position alone gives
// their difference exactly, in one load.
//
// What each letter prints:
//
//   r  THE RULER'S SELF-CHECK. delay_us(clock, 999) read on the ruler:
//      verdicts "at least the due cycles" (hz x 999 us = 47 952) and
//      "under the due cycles + 5 per cent". 999 and not 1000:
//      cortexm/delay.hpp refuses a wait of one SysTick period or more,
//      and a period is one millisecond here (the line printed shows the
//      refusal of 1000). Then the instrument's cost, bench lines with n=0
//      and wire=0, each run on a console drained first (letter p's
//      drain), three of them the AVERAGE of the run named, every field
//      divided by the units and rounded to the nearest (the raw totals on
//      the line above each):
//        ruler     one Ruler::now(), over 1000 reads in a loop (the
//                  loop's increment, test, branch and the volatile store
//                  of the value are in it);
//        stamp     one enter()+leave() pair of a bare IsrMeter on Ruler
//                  with nothing between, over 1000 pairs, a compiler
//                  barrier between pairs so each loads and stores the
//                  meter's sums as a handler does, the loop placed where
//                  the vectors are: irq=1, and isr = what the meter
//                  charges an EMPTY body - the floor under SERCOM5's isr
//                  (the tick meter's own floor is one VAL load and a
//                  subtract);
//        window    one idle turn with nothing pending but the tick (the
//                  masked call, the window's two stamps, the WFI, the
//                  tick handler that ends it, the loop's test on the
//                  ruler) over the turns of 100 ms: wall is one tick
//                  period, irq=1, and isr is the TICK HANDLER's cycles -
//                  it ran inside the window, so busy holds it, and busy -
//                  isr is the turn's own cost;
//      and one the BEST of 8, as letter m's lines are:
//        interval  an empty measured interval, start() then stop() with
//                  nothing between (the counters' snapshot and the
//                  ruler's read under the guard at each end, below): the
//                  floor every wall of letters m, p and t holds beyond
//                  its operation.
//   m  memcpy and memset (brio/rt/rt.cpp, the word path) of 1, 16, 256
//      and 4096 bytes between two word-aligned static buffers in SRAM,
//      each the BEST of 8 measured intervals, the length read from a
//      volatile so the call is the runtime's and not an inlined copy, the
//      console drained before each size (a line still leaving would put a
//      SERCOM5 interrupt in the run). No idle and no interrupt expected:
//      busy = wall, irq and isr whatever tick landed in the best run.
//   p  a print of 1, 16, 256 and 4096 bytes - print(serial, s) with s a
//      static string in flash of that length (rows of 62 digits and a
//      CRLF; the 1-byte one is the last LF) - then the DRAIN: this
//      transport has no single verb for "ring empty and shifter
//      finished", so it is the ring's Serial::tx_idle() and then the
//      resource's INTFLAG.TXC (Serial::Resource::txc_flag()), which
//      writing DATA clears and only the last stop bit's departure with
//      nothing new in DATA sets (31.6.2.5, 31.8.8) - both spins bounded,
//      a bound that runs out printed. Counters read before the print and
//      after the drain. busy = wall (the print and the drain spin); irq
//      and isr are the transport's shape: print hands the transport runs
//      (write_bulk arms DRE once a run), and SERCOM5's handler feeds one
//      byte an entry.
//   t  the tick's floor: one second (hz cycles on the ruler) of masked
//      Idle::idle() turns with the console drained - no kernel loop, none
//      runs here. wall = the second, irq = the ticks, isr = the tick
//      handler's cycles, busy = the floor. n=0, wire=0.
//
// THE WIRES (the `wire` field, bytes per second, and why it is the
// limit):
//   print   115200 baud over the ten bits of an 8N1 frame (a start bit,
//           eight data bits, a stop bit): 11 520 B/s. The arithmetic
//           baud generator makes a rate within a few hundredths of a per
//           cent of it at 48 MHz (31.6.2.3; the banner prints the rate
//           BAUD actually gives); the configured rate is the figure.
//   memcpy  2 bytes a cycle x hz = 96 000 000 B/s. The Cortex-M0+ has ONE
//           AHB-Lite master port for flash, SRAM and peripherals alike
//           (10.1.1), the SRAM answers it in a single cycle at full speed
//           (9.1) and the CPU's QoS out of reset is HIGH, so it pays no
//           extra cycle (10.4.3); LDM and STM take 1+N cycles for N words
//           (the Cortex-M0+ TRM, ARM DDI 0484C 3.3 table 3-1 - the same
//           core's table is also the RP2040 datasheet's 2.4.3.3, table
//           81). A word copied is at least one beat in and one beat
//           out; the +1 per instruction and the loop are the
//           implementation's.
//   memset  4 bytes a cycle x hz = 192 000 000 B/s: one STM beat a word,
//           nothing loaded.
//   r and t wire=0: an instrument's cost and an idle second carry no
//           bytes.
//
// THE COUNTERS are read before and after each run as one instant, and
// together with the ruler's read that starts or ends the wall, under
// SamPlatform's guard (snapshot() and Interval below say why).
//
// THE INSTRUMENT'S COST, and how a reader subtracts it: every line is
// RAW, and the figures to subtract are letter r's of the same run. A
// wall holds the `interval` line's wall beyond the operation;
// SERCOM5's isr holds the `stamp` line's isr per interrupt (what an empty
// body is charged) and busy its whole wall per interrupt; an idle turn's
// busy holds the `window` line's busy - isr. Two seams util/bench.hpp
// states are left open: the hardware's entry and exit (the core's
// worst-case entry latency is 15 cycles at zero wait states - the figure
// the RP2040 datasheet's 2.4.3.6.1 gives for the same core) count as idle
// or thread, and a handler landing between the WFI's return and the
// window's close counts in both.
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include <array>
#include <cstring>

#include "samc21/clock.hpp"
#include "samc21/delay.hpp"
#include "samc21/nvic.hpp"
#include "samc21/platform.hpp"
#include "samc21/sercom.hpp"
#include "samc21/ticker.hpp"
#include "util/bench.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

// The placement of the bound vectors and of letter r's stamp loop: flash
// here; in bench_samc_ram.cpp, SRAM through the attribute this family's
// binding pattern documents (docs/samc21/platform.md, "A handler in
// SRAM"), spelled as an app spells it on the handler it binds.
#if defined(BENCH_RAM_TEXT)
#define BENCH_PLACEMENT [[gnu::section(".ram_text")]]
constexpr const char* bench_image = "bench_samc_ram";
constexpr const char* vectors_in = "SRAM (.ram_text)";
#else
#define BENCH_PLACEMENT
constexpr const char* bench_image = "bench_samc";
constexpr const char* vectors_in = "flash";
#endif

using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

constexpr UartPads console_pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'B', 30, PinFunction::d},
    .rx_pin = {'B', 31, PinFunction::d},
};
using Serial = Uart<5, console_pads>;  // rings 64/256 (defaults)
constexpr Serial serial;

constexpr uint32_t console_baud = 115200;

// ---- the rulers ----------------------------------------------------------------

/// The SysTick ticker read as CLK_CPU cycles (the file header).
struct Ruler {
    [[gnu::always_inline, gnu::flatten]] static uint32_t now() { return Ticker::cycles(); }
    static constexpr uint32_t hz() { return SysClock::hz; }
};
static_assert(CycleRuler<Ruler>);

/// The SysTick vector's ruler: SysTick's position in its period, exact for
/// a difference of two reads inside one period (the file header says why
/// the tick vector cannot use Ruler). The period is fixed: no dynamic
/// clock on this family, so LOAD is what Ticker::init() wrote.
struct TickRuler {
    static constexpr uint32_t last = SysClock::hz / Ticker::ticks_per_second - 1u;
    [[gnu::always_inline]] static uint32_t now() { return last - SysTick->VAL; }
    static constexpr uint32_t hz() { return SysClock::hz; }
};
static_assert(CycleRuler<TickRuler>);

using Idle = BenchIdle<SamPlatform, Ruler>;
static_assert(Platform<Idle>);

// One meter per bound vector, and the bare one letter r times.
IsrMeter<Ruler, Idle> sercom_meter;
IsrMeter<TickRuler, Idle> tick_meter;
IsrMeter<Ruler, Idle> empty_meter;

/// The counters as ONE instant: read under the platform's guard. The tick
/// is never quiescent, and an interrupt landing between bench_counters()'
/// reads of the cycles and of the count is counted in irq and not in isr
/// (or the other way round) - a line with irq=1 isr=0 is what that looks
/// like. The guard holds a pending tick off for the few tens of cycles
/// the four loads take.
template <typename... Meters>
BenchCounters snapshot(const Meters&... m) {
    SamPlatform::CriticalSection guard;
    return bench_counters<Idle>(m...);
}

/// One measured interval of the two bound vectors' meters: the counters
/// and the Stopwatch read TOGETHER under the guard at each end, so that
/// an interrupt held off by the guard is taken inside the wall and the
/// counters both, or outside both - with the snapshot alone, one pended
/// during it is taken in the gap before start() and counted in irq and
/// isr but not in wall.
class Interval {
public:
    [[gnu::always_inline]] void start() {
        SamPlatform::CriticalSection guard;
        c0_ = bench_counters<Idle>(sercom_meter, tick_meter);
        sw_.start();
    }
    [[gnu::always_inline]] uint32_t elapsed() const { return sw_.elapsed(); }
    [[gnu::always_inline]] BenchSample stop() {
        SamPlatform::CriticalSection guard;
        const uint32_t wall = sw_.elapsed();
        return bench_sample(wall, c0_, bench_counters<Idle>(sercom_meter, tick_meter));
    }

private:
    BenchCounters c0_{};
    Stopwatch<Ruler> sw_;
};

TestBench<Serial> bench;

// ---- the wires (the file header) ---------------------------------------------

constexpr uint32_t print_wire_bps = console_baud / 10u;
constexpr uint32_t memcpy_wire_bps = 2u * SysClock::hz;
constexpr uint32_t memset_wire_bps = 4u * SysClock::hz;

constexpr std::array<uint32_t, 4> sizes{1u, 16u, 256u, 4096u};
constexpr uint32_t max_size = 4096u;

// ---- the drain ------------------------------------------------------------------

/// Spin bounds, the driver's own (Uart::rebase()): the ring holds at most
/// 256 bytes, 22 ms at 115200, and a frame is 87 us - both far inside.
constexpr uint32_t ring_spins = 8'000'000u;
constexpr uint32_t frame_spins = 200'000u;

/// Let the console fall silent: the transport's ring empty, then the
/// resource's TXC (the file header, letter p). False when a bound ran out.
bool drain() {
    uint32_t spins = 0;
    while (!Serial::tx_idle()) {
        if (++spins == ring_spins) {
            return false;
        }
    }
    spins = 0;
    while (!Serial::Resource::txc_flag()) {
        if (++spins == frame_spins) {
            return false;
        }
    }
    return true;
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

/// The best (shortest wall) of 8 measured intervals of `op`; the barrier
/// after each run keeps what op wrote alive.
template <typename Op>
BenchSample best_of_8(Op op) {
    BenchSample best{};
    Interval iv;
    for (uint8_t run = 0; run < 8u; ++run) {
        iv.start();
        op();
        const BenchSample s = iv.stop();
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
constexpr uint32_t probe_due = SysClock::hz / 1'000'000u * probe_us;   // 47 952
constexpr uint32_t reps = 1000u;

/// The stamp pairs, placed where the vectors are (the file header).
/// noinline, so the placement is this loop's and not its caller's.
BENCH_PLACEMENT [[gnu::noinline]] uint32_t run_stamps() {
    Stopwatch<Ruler> sw;
    sw.start();
    for (uint32_t i = 0; i < reps; ++i) {
        empty_meter.enter();
        empty_meter.leave();
        asm volatile("" ::: "memory");
    }
    return sw.elapsed();
}

void tr_ruler() {
    (void)drain();
    (void)delay_us(clock, probe_us);   // the wait's code into the flash cache
    const bool whole_ms_refused = !delay_us(clock, 1000u);

    Stopwatch<Ruler> sw;
    sw.start();
    const bool served = delay_us(clock, probe_us);
    const uint32_t took = sw.elapsed();
    print(serial, "  delay_us(clock, ", probe_us, ") served=", served, ": ", took,
          " cycles on the ruler (due ", probe_due, "); delay_us(clock, 1000) refused=",
          whole_ms_refused, crlf);
    bench.verdict("the ruler reads delay_us(999) at least the due cycles",
                  served && took >= probe_due);
    bench.verdict("and under the due cycles + 5 per cent", took <= probe_due + probe_due / 20u);

    // ruler: one read
    {
        (void)drain();
        volatile uint32_t sink = 0;
        Interval iv;
        iv.start();
        for (uint32_t i = 0; i < reps; ++i) {
            sink = Ruler::now();
        }
        const BenchSample s = iv.stop();
        (void)sink;
        print_totals(reps, "reads", s);
        bench_line(serial, "ruler", 0u, per_unit(s, reps), Ruler::hz(), 0u);
    }

    // stamp: one enter()+leave() pair, nothing between
    {
        (void)drain();
        const BenchCounters c0 = snapshot(sercom_meter, tick_meter, empty_meter);
        const uint32_t wall = run_stamps();
        const BenchSample s =
            bench_sample(wall, c0, snapshot(sercom_meter, tick_meter, empty_meter));
        print_totals(reps, "pairs", s);
        bench_line(serial, "stamp", 0u, per_unit(s, reps), Ruler::hz(), 0u);
    }

    // interval: an empty measured interval, best of 8
    (void)drain();
    bench_line(serial, "interval", 0u, best_of_8([] {}), Ruler::hz(), 0u);

    // window: one idle turn with nothing pending but the tick
    {
        (void)drain();
        const uint32_t turns0 = Idle::idle_turns();
        Interval iv;
        iv.start();
        while (iv.elapsed() < Ruler::hz() / 10u) {
            disable_interrupts();
            Idle::idle();
        }
        const BenchSample s = iv.stop();
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
        (void)drain();
        const BenchSample s = best_of_8([len] { std::memcpy(mem_dst, mem_src, len); });
        bench_line(serial, "memcpy", n, s, Ruler::hz(), memcpy_wire_bps);
    }
    for (const uint32_t n : sizes) {
        mem_len = n;
        const uint32_t len = mem_len;
        (void)drain();
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
    Interval iv;
    for (const uint32_t n : sizes) {
        (void)drain();
        const char* text = payload.data() + (max_size - n);
        iv.start();
        print(serial, text);
        const bool drained = drain();
        const BenchSample s = iv.stop();
        print(serial, crlf);
        if (!drained) {
            print(serial, "  the drain's bound ran out: the line below is not a drained print", crlf);
        }
        bench_line(serial, "print", n, s, Ruler::hz(), print_wire_bps);
    }
    bench.verdict("ran", true);
}

// =============================================================================
// t - the tick's floor
// =============================================================================
void tt_tick() {
    (void)drain();
    Interval iv;
    iv.start();
    while (iv.elapsed() < Ruler::hz()) {
        disable_interrupts();
        Idle::idle();
    }
    const BenchSample s = iv.stop();
    bench_line(serial, "tick", 0u, s, Ruler::hz(), 0u);
    bench.verdict("ran", true);
}

void banner() {
    print(serial, crlf, bench_image, " - the benchmark skeleton (util/bench.hpp), clk=", SysClock::hz,
          " Hz, console SERCOM5 ", console_baud, " 8N1 (BAUD gives ",
          Serial::actual_baud(SysClock::hz), "), ruler SysTick cycles, vectors in ", vectors_in,
          crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
// One meter per vector; the placement is the file header's.
extern "C" BENCH_PLACEMENT void SERCOM5_Handler() {
    sercom_meter.enter();
    (void)Serial::isr();
    sercom_meter.leave();
}
extern "C" BENCH_PLACEMENT void SysTick_Handler() {
    tick_meter.enter();
    brio::Ticker::tick();
    tick_meter.leave();
}

int main() {
    const bool clock_ok = SysClock::init();       // OSC48M -> GCLK0 -> 48 MHz
    const bool serial_ok = Serial::init(clock, console_baud);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    bench.letter('r', "the ruler's self-check and the instrument's cost", tr_ruler);
    bench.letter('m', "memcpy and memset, 1/16/256/4096 bytes", tm_memory);
    bench.letter('p', "a print of 1/16/256/4096 bytes through the console", tp_print);
    bench.letter('t', "the tick's floor: one second of idle", tt_tick);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "OSC48M" : "FAILED", " tick=",
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
