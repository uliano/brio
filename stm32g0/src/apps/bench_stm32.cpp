// bench_stm32 - the benchmark skeleton on the STM32G0 (design/benchmark.md,
// util/bench.hpp): letters that print NUMBERS, one `bench` line per
// operation and size, in the grammar every family prints, plus the flash
// prefetch measured as a column of its own. NOT A TEST: a letter's one
// verdict is "ran", except letter r, whose two verdicts judge the ruler
// every other line is read with.
//
// NOTHING TO WIRE. The console is the Nucleo's own ST-LINK virtual COM
// port, USART2 on PA2 (TX) / PA3 (RX) at AF1, 115200 8N1 - the binding of
// console.cpp and test_stm32_platform.cpp (the handler name the reserve
// spells, BRIO_STM32G0_USART2_HANDLER), and the TestBench frame and the
// polled prompt loop of the latter. NO KERNEL AND NO ACTIVE OBJECT: the
// letters are plain functions and the idle path is called by hand, so
// what is measured is the transport, the runtime and the idle path,
// never a dispatch.
//
// THE CLOCK: HSI16 through the PLL to 64 MHz, PCLK = HCLK
// (stm32g0/clock.hpp), the rate test_stm32_platform runs at. The flash
// then runs at TWO WAIT STATES (RM0444 3.3.4, table 13) behind the
// instruction cache that is on at reset and the prefetch that is off at
// reset (3.7.1), which is what letter f is for.
//
// THE RULER is `Ruler` below: the SysTick ticker read as cycles
// (cortexm/ticker.hpp's BasicTicker::cycles(), the tick count and
// SysTick's position in its period composed by util/cycle_count.hpp), so
// it counts HCLK cycles and hz() is SysClock::hz, 64 000 000. Its now()
// is always_inline AND flatten, so the whole read - LOAD, the count,
// ICSR, VAL, ICSR, the count, and the compose - lands inline in every
// vector and in every loop that stamps, with no call. (flatten is the
// release build's: the debug preset's -fno-inline switches it off and the
// vectors there call cycles().)
//
// THE PLATFORM the idle path runs on is brio::BenchIdle<Stm32g0Platform<>,
// Ruler> (`Idle`): every idle turn is a masked call of Idle::idle(),
// which stamps the window around the platform's DSB + WFI + unmask.
//
// THE METERS: one IsrMeter per bound vector - USART2's (`usart_meter`, on
// Ruler) and SysTick's (`tick_meter`, on `TickRuler`). The SysTick
// vector's meter CANNOT run on Ruler: inside the SysTick handler, before
// Ticker::tick() has counted, the exception is active and no longer
// pending (ICSR.PENDSTSET reads 0) while the count still holds the
// previous tick, so cycles() reads one whole period low there and right
// again after the increment - a meter on it would charge every tick a
// period, 64 000 cycles. TickRuler is the counter's POSITION alone,
// period - 1 - VAL: both stamps of the tick vector fall inside one period
// (the handler runs right after the reload that pended it and lasts some
// hundred cycles; no masked section of this app approaches a period), so
// their difference is exact - with one load where Ruler takes six.
//
// What each letter prints:
//
//   r  THE RULER'S SELF-CHECK. delay_us(clock, 999) read on the ruler:
//      verdicts "at least the due cycles" (999 x 64 = 63 936) and "under
//      the due cycles + 5 per cent". 999 and not 1000: cortexm/delay.hpp
//      refuses a wait of one SysTick period or more, and the period is one
//      millisecond here (the line printed shows the refusal of 1000). The
//      ruler and delay_us count the SAME SysTick, so what the check proves
//      is the ruler's COMPOSITION across the tick the wait straddles, not
//      the rate (a wrong SYSCLK would move both alike). Then the
//      instrument's cost, three bench lines with n=0 and wire=0, each the
//      AVERAGE of its run, every field divided by the units and rounded to
//      the nearest (the raw totals on the line above it):
//        ruler   one Ruler::now(), over 1000 reads in a loop (the loop's
//                increment, test and branch and a volatile store of the
//                value are in it);
//        stamp   one enter()+leave() pair of an IsrMeter on Ruler with
//                nothing between, over 1000 pairs, a compiler barrier
//                between pairs so each pair loads and stores the meter's
//                sums as a handler does: irq=1, and isr = what the meter
//                charges an EMPTY body - the floor under USART2's isr
//                (the tick meter's floor is one VAL load and a subtract);
//        window  one idle turn (the masked call, the window's two stamps,
//                the WFI, the tick that ends it, the loop's test on the
//                Stopwatch) over the turns of 100 ms: wall is one tick
//                period, irq=1, and isr is the TICK HANDLER's cycles - it
//                ran inside the window, so busy holds it - and busy - isr
//                is the turn's own cost.
//   m  memcpy and memset (rt/rt.cpp, the word path) of 1, 16, 256 and
//      4096 bytes between two word-aligned static buffers in SRAM, each
//      the BEST of 8 runs on a Stopwatch (the shortest wall, then the
//      fewest interrupts), the length read from a volatile so the call is
//      the runtime's and not an inlined copy. No idle and no interrupt
//      expected: busy = wall, irq and isr whatever tick landed in the best
//      run. ON A PART WITH 8 KBYTES OF SRAM (the STM32G031K8, SRAM_SIZE_MAX
//      in its header) the largest size is 2048: two 4096-byte buffers
//      would be the whole of its RAM.
//   p  a print of 1, 16, 256 and 4096 bytes - print(serial, s) with s a
//      static string in flash of that length (rows of 62 digits and a
//      CRLF; the 1-byte one is the final LF) - then the drain. The
//      transport has NO verb for "the ring is empty AND the shifter has
//      finished": Uart::tx_idle() is the ring alone, so the drain is
//      tx_idle() followed by the resource's TC flag (RM0444 33.8.10: set
//      when the last frame written to TDR has left the shift register,
//      cleared by the TDR write that started it), both bounded on the
//      ruler. Counters before the print and after the drain. busy = wall
//      (the print spins in write_blocking, the drain spins on the flags);
//      irq and isr are the transport's shape: one USART2 interrupt per
//      byte on TXE, plus two that find the ring dry and disarm TXEIE -
//      one right after the first byte (the idle shift register takes TDR
//      at once, so TXE fires again before the thread has pushed the
//      second byte, and the next push re-arms) and one after the last -
//      plus the ticks of the run; the line after each bench line gives the
//      two vectors apart.
//   t  the tick's floor: one second (hz cycles on the ruler) of masked
//      Idle::idle() turns with the console drained - not a kernel loop:
//      no kernel runs here. wall = the second, irq = the ticks, isr = the
//      tick handler's cycles, busy = the floor. n=0, wire=0.
//   f  THE PREFETCH (FLASH_ACR.PRFTEN, RM0444 3.3.5) as a column of its
//      own: the instrument's three cost lines and letters m, p and t run
//      TWICE through stm32g0/flash.hpp's FlashAccel::prefetch() - off,
//      then on - every op suffixed `.pf0` / `.pf1`, and the reset state
//      (PRFTEN clear, 3.7.1) restored after. The instruction cache (ICEN,
//      on at reset) stays on in both columns. The erratum that keeps the
//      prefetch off by default, ES0548 2.2.10 (a prefetch may fail on a
//      branch ACROSS BANKS), cannot bite this image: ld/stm32g0b1re.ld
//      gives the linker bank 1 alone, and the G071 and G031 have one bank.
//
// THE WIRES (the `wire` field, bytes per second, and why it is the
// limit):
//   print   115200 baud over the ten bits of an 8N1 frame (a start bit,
//           eight data bits, a stop bit): 11 520 B/s. The divisor gives
//           115 107 baud at 64 MHz (BRR 556, 33.5.7), 0.08 per cent slow,
//           so x can never fall below 1.0008 on this line; the configured
//           rate is the figure.
//   memcpy  2 bytes a cycle x hz = 128 000 000 B/s. The Cortex-M0+ reaches
//           SRAM through ONE system bus into the bus matrix (RM0444 2.1,
//           figure 1), the SRAM answers at SYSCLK with no wait state
//           (RM0444 2.3), and LDM and STM cost 1+N cycles for N words -
//           the core's instruction summary, ARM's table as the RP2040
//           datasheet reproduces it for this core (2.4.3.3, table 81),
//           the only copy of the Cortex-M0+'s figures on the desk, and the
//           same 1+N as the Cortex-M0's own (DDI0432C 3.3, table 3-1): a
//           word copied is at least one beat in and one beat out, the +1
//           per instruction, the loop and the instruction fetch (over the
//           same bus, from a flash at two wait states) being the
//           implementation's.
//   memset  4 bytes a cycle x hz = 256 000 000 B/s: one STM beat per word
//           (the same table), nothing loaded.
//   r, t    wire=0: an instrument's cost and an idle second carry no
//           bytes.
//
// THE INSTRUMENT'S COST, and how a reader subtracts it: every line is
// RAW. A Stopwatch wall holds about one `ruler` line's wall beyond the
// operation (the tail of the first read and the head of the second);
// USART2's isr holds the `stamp` line's isr per interrupt (what an empty
// body is charged), and busy the whole `stamp` wall per interrupt; an
// idle turn's busy holds the `window` line's busy - isr. A `.pf1` line
// is read against the `.pf1` cost lines, since the prefetch speeds the
// instrument too. Every run starts with the console drained, so that the
// line printed before it is not still going out under it, and the
// counters are read under the platform's critical section (`counters_of`
// says why). Two seams util/bench.hpp states are left open: the
// exception entry and return (the Cortex-M0+'s worst-case entry is 15
// cycles at zero wait states, RP2040 datasheet 2.4.3.6 reproducing ARM's
// figure; the vector and the handler's first lines come from a flash at
// two wait states here) count as idle or as thread, and a handler landing
// between the WFI's return and the window's close counts in both. And one
// of this app's: the counters bracket the Stopwatch from outside, so a
// tick landing between a counters read and the Stopwatch's read is in irq
// and isr and not in wall - one tick, the `window` line's isr, at most at
// either end of a run.
//
// build: boards = g0b1re,g071rb,g031k8
// build: monitor_speed = 115200

#include <stdint.h>

#include <array>
#include <cstring>

#include "stm32g0/clock.hpp"
#include "stm32g0/delay.hpp"
#include "stm32g0/flash.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/platform.hpp"
#include "stm32g0/ticker.hpp"
#include "stm32g0/usart.hpp"
#include "util/bench.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

// The console pads: USART2_TX on PA2, USART2_RX on PA3, both AF1
// (DS13560 table 13), which is the ST-LINK's virtual COM port.
constexpr UartPins console_pins{
    .tx = {'A', 2, PinFunction::af1},
    .rx = {'A', 3, PinFunction::af1},
};
using Serial = Uart<2, console_pins>;  // rings 64/256 (defaults)
constexpr Serial serial;

constexpr uint32_t console_baud = 115200;

// ---- the rulers ----------------------------------------------------------------

/// The SysTick ticker read as HCLK cycles (the file header).
struct Ruler {
    [[gnu::always_inline, gnu::flatten]] static uint32_t now() { return Ticker::cycles(); }
    static constexpr uint32_t hz() { return SysClock::hz; }
};
static_assert(CycleRuler<Ruler>);

/// The SysTick vector's ruler: the counter's position in its period, exact
/// for a difference of two reads inside one period (the file header says
/// why the tick vector cannot use Ruler). The period's last step is the
/// reload Ticker::init() writes, clock_hz / tps - 1.
struct TickRuler {
    static constexpr uint32_t last = SysClock::hz / Ticker::ticks_per_second - 1u;
    [[gnu::always_inline]] static uint32_t now() { return last - SysTick->VAL; }
    static constexpr uint32_t hz() { return SysClock::hz; }
};
static_assert(CycleRuler<TickRuler>);

using Idle = BenchIdle<Stm32g0Platform<>, Ruler>;
static_assert(Platform<Idle>);

// One meter per bound vector, and the bare one letter r times.
IsrMeter<Ruler, Idle> usart_meter;
IsrMeter<TickRuler, Idle> tick_meter;
IsrMeter<Ruler, Idle> empty_meter;

/// The counters at one instant, read UNDER THE MASK although every word
/// of them is atomic on this core: the tick keeps running between two
/// letters' operations, and a tick landing between the read of the
/// meters' cycles and the read of their counts would hand one reading
/// the handler's cycles without its count (measured: a line with irq=0
/// and isr=133 before this guard).
template <typename... Meters>
BenchCounters counters_of(const Meters&... m) {
    const Idle::CriticalSection cs;
    return bench_counters<Idle>(m...);
}
BenchCounters counters() { return counters_of(usart_meter, tick_meter); }

/// The same instant with the two vectors kept apart, for letter p.
struct Snapshot {
    BenchCounters all;
    uint32_t usart_irq;
    uint32_t usart_isr;
    uint32_t tick_irq;
    uint32_t tick_isr;
};
Snapshot snapshot() {
    const Idle::CriticalSection cs;
    return {bench_counters<Idle>(usart_meter, tick_meter), usart_meter.count(), usart_meter.cycles(),
            tick_meter.count(), tick_meter.cycles()};
}

TestBench<Serial> bench;

// ---- the wires (the file header) -----------------------------------------------

constexpr uint32_t print_wire_bps = console_baud / 10u;
constexpr uint32_t memcpy_wire_bps = 2u * SysClock::hz;
constexpr uint32_t memset_wire_bps = 4u * SysClock::hz;

constexpr std::array<uint32_t, 4> print_sizes{1u, 16u, 256u, 4096u};
constexpr uint32_t print_max = 4096u;

/// Two buffers of this size fit every part the app builds for: 2048 where
/// the SRAM is 8 Kbytes (the file header).
constexpr uint32_t mem_max = SRAM_SIZE_MAX >= 0x4000UL ? 4096u : 2048u;
constexpr std::array<uint32_t, 4> mem_sizes{1u, 16u, 256u, mem_max};

/// The op names of one column: the plain run, and letter f's two.
struct OpNames {
    const char* ruler;
    const char* stamp;
    const char* window;
    const char* memcpy;
    const char* memset;
    const char* print;
    const char* tick;
};
constexpr OpNames plain_ops{"ruler", "stamp", "window", "memcpy", "memset", "print", "tick"};
constexpr OpNames pf0_ops{"ruler.pf0",  "stamp.pf0", "window.pf0", "memcpy.pf0",
                          "memset.pf0", "print.pf0", "tick.pf0"};
constexpr OpNames pf1_ops{"ruler.pf1",  "stamp.pf1", "window.pf1", "memcpy.pf1",
                          "memset.pf1", "print.pf1", "tick.pf1"};

// ---- helpers -------------------------------------------------------------------

/// Let the console fall silent: the ring empty (tx_idle()), then the last
/// frame out of the shift register (TC, RM0444 33.8.10) - the transport
/// has no verb for both. Bounded on the ruler at a tenth of a second, the
/// full 256-byte ring at the wire being 22 ms; false = it gave up.
bool drain() {
    Stopwatch<Ruler> sw;
    sw.start();
    while (!Serial::tx_idle()) {
        if (sw.elapsed() > Ruler::hz() / 10u) {
            return false;
        }
    }
    while ((Serial::Resource::status() & UsartFlag::tc) == 0u) {
        if (sw.elapsed() > Ruler::hz() / 10u) {
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

/// The two vectors apart, between two snapshots.
void print_vectors(const Snapshot& a, const Snapshot& b) {
    print(serial, "  usart2: irq=", b.usart_irq - a.usart_irq, " isr=", b.usart_isr - a.usart_isr,
          "  systick: irq=", b.tick_irq - a.tick_irq, " isr=", b.tick_isr - a.tick_isr, crlf);
}

/// The best (shortest wall, then fewest interrupts) of 8 runs of `op` on
/// a Stopwatch, the console drained first so that the bench line printed
/// before it is not still going out under the runs; the barrier after
/// each run keeps what op wrote alive.
template <typename Op>
BenchSample best_of_8(Op op) {
    (void)drain();
    BenchSample best{};
    Stopwatch<Ruler> sw;
    for (uint8_t run = 0; run < 8u; ++run) {
        const BenchCounters c0 = counters();
        sw.start();
        op();
        const uint32_t wall = sw.elapsed();
        const BenchSample s = bench_sample(wall, c0, counters());
        asm volatile("" ::: "memory");
        if (run == 0u || s.wall < best.wall || (s.wall == best.wall && s.irq < best.irq)) {
            best = s;
        }
    }
    return best;
}

// =============================================================================
// the instrument's cost (letter r, and both columns of letter f)
// =============================================================================
constexpr uint32_t reps = 1000u;

void run_costs(const OpNames& ops) {
    Stopwatch<Ruler> sw;

    // ruler: one read
    {
        (void)drain();
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
        bench_line(serial, ops.ruler, 0u, per_unit(s, reps), Ruler::hz(), 0u);
    }

    // stamp: one enter()+leave() pair, nothing between
    {
        (void)drain();
        const BenchCounters c0 = counters_of(usart_meter, tick_meter, empty_meter);
        sw.start();
        for (uint32_t i = 0; i < reps; ++i) {
            empty_meter.enter();
            empty_meter.leave();
            asm volatile("" ::: "memory");
        }
        const uint32_t wall = sw.elapsed();
        const BenchSample s =
            bench_sample(wall, c0, counters_of(usart_meter, tick_meter, empty_meter));
        print_totals(reps, "pairs", s);
        bench_line(serial, ops.stamp, 0u, per_unit(s, reps), Ruler::hz(), 0u);
    }

    // window: one idle turn with nothing pending but the tick
    {
        (void)drain();
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
        bench_line(serial, ops.window, 0u, per_unit(s, turns), Ruler::hz(), 0u);
    }
}

// =============================================================================
// r - the ruler's self-check and the instrument's cost
// =============================================================================
constexpr uint32_t probe_us = 999u;
constexpr uint32_t probe_floor = SysClock::hz / 1'000'000u * probe_us;
static_assert(SysClock::hz % 1'000'000u == 0u, "the floor is exact in whole cycles per microsecond");

void tr_ruler() {
    (void)drain();
    (void)delay_us(clock, probe_us);   // the wait's code into the instruction cache
    const bool whole_tick_refused = !delay_us(clock, 1000u);

    Stopwatch<Ruler> sw;
    sw.start();
    const bool served = delay_us(clock, probe_us);
    const uint32_t took = sw.elapsed();
    print(serial, "  delay_us(clock, ", probe_us, ") served=", served, ": ", took,
          " cycles on the ruler (floor ", probe_floor, ", +5% ", probe_floor + probe_floor / 20u,
          "); delay_us(clock, 1000) refused=", whole_tick_refused, crlf);
    bench.verdict("the ruler reads delay_us(999) at least the due cycles",
                  served && took >= probe_floor);
    bench.verdict("and under the due cycles + 5 per cent", took <= probe_floor + probe_floor / 20u);

    run_costs(plain_ops);
    bench.verdict("ran", true);
}

// =============================================================================
// m - memcpy and memset
// =============================================================================
alignas(4) uint8_t mem_src[mem_max];
alignas(4) uint8_t mem_dst[mem_max];
volatile uint32_t mem_len = 0;

void run_memory(const OpNames& ops) {
    for (uint32_t i = 0; i < mem_max; ++i) {
        mem_src[i] = static_cast<uint8_t>(i);
    }
    for (const uint32_t n : mem_sizes) {
        mem_len = n;
        const uint32_t len = mem_len;
        const BenchSample s = best_of_8([len] { std::memcpy(mem_dst, mem_src, len); });
        bench_line(serial, ops.memcpy, n, s, Ruler::hz(), memcpy_wire_bps);
    }
    for (const uint32_t n : mem_sizes) {
        mem_len = n;
        const uint32_t len = mem_len;
        const BenchSample s = best_of_8([len] { std::memset(mem_dst, 0x5A, len); });
        bench_line(serial, ops.memset, n, s, Ruler::hz(), memset_wire_bps);
    }
}

void tm_memory() {
    run_memory(plain_ops);
    bench.verdict("ran", true);
}

// =============================================================================
// p - a print through the console
// =============================================================================
/// The payload: rows of 62 digits and a CRLF, NUL-terminated; the string
/// of length n is its last n bytes.
constexpr std::array<char, print_max + 1u> payload = [] {
    std::array<char, print_max + 1u> t{};
    for (uint32_t i = 0; i < print_max; ++i) {
        const uint32_t col = i % 64u;
        t[i] = col == 62u ? '\r' : col == 63u ? '\n' : static_cast<char>('0' + (i / 64u + col) % 10u);
    }
    t[print_max] = '\0';
    return t;
}();

void run_print(const OpNames& ops) {
    Stopwatch<Ruler> sw;
    for (const uint32_t n : print_sizes) {
        (void)drain();
        const char* text = payload.data() + (print_max - n);
        const Snapshot v0 = snapshot();
        sw.start();
        print(serial, text);
        const bool drained = drain();
        const uint32_t wall = sw.elapsed();
        const Snapshot v1 = snapshot();
        const BenchSample s = bench_sample(wall, v0.all, v1.all);
        print(serial, crlf);
        bench_line(serial, ops.print, n, s, Ruler::hz(), print_wire_bps);
        print_vectors(v0, v1);
        if (!drained) {
            print(serial, "  (the drain gave up: the line above is not a measurement)", crlf);
        }
    }
}

void tp_print() {
    run_print(plain_ops);
    bench.verdict("ran", true);
}

// =============================================================================
// t - the tick's floor
// =============================================================================
void run_tick(const OpNames& ops) {
    (void)drain();
    Stopwatch<Ruler> sw;
    const BenchCounters c0 = counters();
    sw.start();
    while (sw.elapsed() < Ruler::hz()) {
        disable_interrupts();
        Idle::idle();
    }
    const uint32_t wall = sw.elapsed();
    const BenchSample s = bench_sample(wall, c0, counters());
    bench_line(serial, ops.tick, 0u, s, Ruler::hz(), 0u);
}

void tt_tick() {
    run_tick(plain_ops);
    bench.verdict("ran", true);
}

// =============================================================================
// f - the prefetch, off and on
// =============================================================================
void print_acr() {
    print(serial, "  FLASH_ACR: LATENCY=", FlashWaitStates::get(),
          " ICEN=", FlashAccel::instruction_cache(), " PRFTEN=", FlashAccel::prefetch(), crlf);
}

void tf_prefetch() {
    FlashAccel::prefetch(false);
    print_acr();
    run_costs(pf0_ops);
    run_memory(pf0_ops);
    run_print(pf0_ops);
    run_tick(pf0_ops);

    FlashAccel::prefetch(true);
    print_acr();
    run_costs(pf1_ops);
    run_memory(pf1_ops);
    run_print(pf1_ops);
    run_tick(pf1_ops);

    FlashAccel::prefetch(false);   // the reset state (RM0444 3.7.1)
    print_acr();
    bench.verdict("ran", true);
}

void banner() {
    print(serial, crlf, "bench_stm32 - the benchmark skeleton (util/bench.hpp), clk=", SysClock::hz,
          " Hz, console USART2 ", console_baud, " 8N1 (", Serial::actual_baud(SysClock::pclk_hz),
          " baud), ruler SysTick cycles", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
// USART2's line is USART2_LPUART2_IRQHandler on the G0B1 and
// USART2_IRQHandler on the G071/G031: the name the reserve derives from
// the header's presence macros is what an app on three boards binds.
extern "C" void BRIO_STM32G0_USART2_HANDLER() {
    usart_meter.enter();
    (void)Serial::isr();
    usart_meter.leave();
}
extern "C" void SysTick_Handler() {
    tick_meter.enter();
    brio::Ticker::tick();
    tick_meter.leave();
}

int main() {
    const bool clock_ok = SysClock::init();   // HSI16 -> PLL -> 64 MHz, two wait states
    const bool serial_ok = Serial::init(clock, console_baud);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    bench.letter('r', "the ruler's self-check and the instrument's cost", tr_ruler);
    bench.letter('m', "memcpy and memset, 1/16/256 bytes and the largest the SRAM holds twice",
                 tm_memory);
    bench.letter('p', "a print of 1/16/256/4096 bytes through the console", tp_print);
    bench.letter('t', "the tick's floor: one second of idle", tt_tick);
    bench.letter('f', "the prefetch: the costs, m, p and t with PRFTEN off, then on", tf_prefetch);

    if (serial_ok) {
        const auto idcode = brio::DeviceIdcode::read();
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL64" : "FAILED", " tick=",
                    tick_ok ? "SysTick" : "FAILED", " DEV_ID ", brio::hex(idcode.dev_id),
                    " REV_ID ", brio::hex(idcode.rev_id), brio::crlf);
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
