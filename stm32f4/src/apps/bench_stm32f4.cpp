// bench_stm32f4 - the benchmark skeleton on the STM32F4 (design/benchmark.md):
// the ruler checked, the instrument's own cost, the runtime's memory
// operations against the core's load/store floor, a print through the
// console against the wire, and the tick's floor. NOT A TEST: every line
// it prints is a number, `bench <op> n= wall= busy= irq= isr= rate= wire=
// x=` (util/bench.hpp), and a letter's verdicts say only that it ran -
// letter r's excepted, because a wrong ruler makes every other line wrong.
//
// THE RULER is the DWT's cycle counter (stm32f4/dwt.hpp over
// cortexm/dwt.hpp): HCLK cycles, one load from the private peripheral
// bus, enabled by this app at boot and again at the start of every letter
// (init() is idempotent, and a probe may have touched the unit since).
// Ruler::hz() is the app's Clock: 180 MHz on the F429ZI, the F446RE and
// the F469NI, 100 MHz on the F411CE - the rates test_stm32f4_platform runs
// at, the same Clock lines.
//
// THE SHAPE. No kernel: the prompt loop of test_stm32f4_platform, polling
// the console's receive ring. The platform the app names is
// BenchIdle<Stm32f4Platform<>, Ruler>, and letters r and t call its idle()
// directly, under its critical section, exactly as the kernel's
// idle_if_empty() would with nothing posted. Two vectors are bound - the
// console's USART and SysTick - each with ONE IsrMeter of its own:
// enter() its first statement, leave() its last, the driver's ISR body
// between them. irq and isr in every line are the two meters' sums.
//
// THE CONSOLE is the board's, as console.cpp binds it: USART2 on PA2/PA3
// (the Nucleo-F446RE's ST-LINK VCP), USART1 on PA9/PA10 (the
// STM32F429I-DISC1's VCP, the black pill's probe bridge), USART3 on
// PB10/PB11 (the 32F469IDISCOVERY's VCP); 115200 8N1, the plain transport
// (no DMA engine), rings of 64 and 256 bytes.
//
// What each letter prints:
//
//   r  THE RULER'S SELF-CHECK AND THE INSTRUMENT'S COST.
//      The counter is enabled (verdict), and DBGMCU_CR.DBG_SLEEP is printed
//      as found: a probe's connection sets it, and with it HCLK runs through
//      a Sleep (stm32f4/dwt.hpp). delay_us(clock, 999) is read on the ruler:
//      verdicts "at least 999 us" and "under 999 us + 5 per cent" - 999 and
//      not 1000, because cortexm/delay.hpp REFUSES a wait of one SysTick
//      period or more and returns at once, so 1000 would measure the
//      refusal. Then three lines, n=0 and wire=0, each the average of its
//      run, measured right after a tick edge so that no tick lands in it:
//        ruler   the cycles between two consecutive Ruler::now() reads (an
//                empty Stopwatch), over 1000 pairs: what every wall below
//                carries beyond its operation.
//        stamp   one enter() + leave() pair with nothing between, timed on
//                a Stopwatch over 1000 pairs of a meter of its own, each
//                pair fenced by compiler barriers so the meter's fields go
//                through memory as a handler's do: what one pair costs a
//                handler. The plain line after it says how many cycles an
//                empty pair RECORDS into its meter - subtract irq times that
//                from a line's isr to get the handlers' bodies.
//        window  one BenchIdle::idle() turn with nothing pending but the
//                tick, averaged over the turns of 100 ticks: wall is the
//                turn (the sleep included), irq and isr are the tick
//                handler the window HOLDS, and busy - isr is what an idle
//                turn costs of its own (the window's stamps, the critical
//                section, the loop's test).
//      And a verdict on the window run: its wall, 100 ticks of idle on the
//      ruler, is 100 ms within 1 per cent - the counter counts through the
//      WFI under the DBG_SLEEP printed above, or every busy below is wrong.
//      Then the CONTROL, window.dbg0 (or window.dbg1 on a board that came
//      up without a probe's bit): the same run with DBG_SLEEP the other
//      way, the bit put back after. Its wall says whether the count runs
//      through a Sleep that does not feed HCLK - stm32f4/dwt.hpp's open
//      question - and it is a number, not a verdict.
//
//   m  memcpy and memset of 1, 16, 256 and 4096 bytes between two static
//      word-aligned buffers in SRAM1, each the best of 8 runs on a
//      Stopwatch: the runtime's word path (rt/rt.cpp), the call included;
//      no idle and no interrupt expected, so busy = wall and irq is a tick
//      that landed. THE WIRE is the core's load/store floor: an LDM or STM
//      of N registers takes 1 + N cycles (DDI 0439B 3.3.1, table 3-1;
//      3.3.2: everything after the first element pipelines), so the core
//      moves at most a word a cycle - a copy, a load and a store a word, is
//      2 bytes a cycle, 2 x hz; a fill, a store a word, is 4 bytes a cycle,
//      4 x hz. The buffers are SRAM1, read and written at CPU speed with no
//      wait state (RM0390 2.2.3) over the core's S-bus (2.1.3); the loop's
//      instructions come from flash through the ART accelerator's cache.
//
//   p  print(console, s) of a static string of 1, 16, 256 and 4096 bytes,
//      then the wait for the transport to DRAIN: the transmit ring empty
//      (Uart::tx_idle(), which is the ring and not the wire) and then the
//      resource's transmission-complete flag (Usart<n>::tx_complete(), SR.TC
//      - RM0390 25.6.1: the last frame's stop bit is out). The counters
//      are read before the print and after the drain. THE WIRE is 115200
//      baud over ten bits a byte (8N1: a start bit, eight, a stop bit),
//      11520 bytes a second. busy ~ wall: print() spins on a full ring.
//      irq counts the USART's handlers (one a byte, the transport's shape:
//      no FIFO on this block, and one more that finds the ring dry and
//      disarms TXE) and the ticks that fell in the run.
//
//   t  the tick's floor: one second - 1000 kernel ticks - of
//      BenchIdle::idle() turns with nothing else to do, the console
//      drained first. wall is the second on the ruler, irq the ticks, isr
//      the tick handler's cycles, busy the floor every program on this
//      timebase pays. n=0, wire=0.
//
// THE COST OF THE INSTRUMENT, and how the reader subtracts it: every
// line is RAW. A wall carries one ruler read (letter r's `ruler`); an isr
// carries what each stamp pair records (the plain line after `stamp`) once
// per irq; a busy carries the stamp pairs' whole cost (`stamp`) once per
// irq and the window's own cost (`window`'s busy - isr) once per idle turn.
// Two seams stay open (design/benchmark.md): the hardware's entry and exit
// of a handler is outside the stamps, and a handler that lands between
// P::idle()'s return and the window's close counts in both.
//
// NOTHING TO WIRE. Connect at 115200 8N1; `z` runs the four letters.
//
// build: boards = f429zi,f446re,f411ce,f469ni
// build: monitor_speed = 115200

#include <stdint.h>
#include <string.h>

#include <array>

#include "stm32f4/clock.hpp"
#include "stm32f4/delay.hpp"
#include "stm32f4/dwt.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/pwr.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/usart.hpp"
#include "util/bench.hpp"
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

#if defined(STM32F429xx)
using Led = Pin<'G', 13>;
constexpr UartPins console_pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
constexpr uint8_t console_instance = 1;
#elif defined(STM32F469xx)
using Led = Pin<'G', 6>;   // LD1, lit when low
constexpr UartPins console_pins{.tx = {'B', 10, PinFunction::af7}, .rx = {'B', 11, PinFunction::af7}};
constexpr uint8_t console_instance = 3;
#elif defined(STM32F411xE)
using Led = Pin<'C', 13>;
constexpr UartPins console_pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
constexpr uint8_t console_instance = 1;
#else
using Led = Pin<'A', 5>;
constexpr UartPins console_pins{.tx = {'A', 2, PinFunction::af7}, .rx = {'A', 3, PinFunction::af7}};
constexpr uint8_t console_instance = 2;
#endif

using Serial = Uart<console_instance, console_pins>;
constexpr Serial serial;

constexpr uint32_t console_baud = 115200;
/// 8N1: a start bit, eight data bits, a stop bit.
constexpr uint32_t console_wire_bps = console_baud / 10u;

/// The ruler: the DWT's cycle counter at the app's clock.
struct Ruler {
    [[gnu::always_inline]] static uint32_t now() { return CycleCounter::now(); }
    static uint32_t hz() { return SysClock::hz; }
};
static_assert(CycleRuler<Ruler>);

using P = Stm32f4Platform<>;
using Idle = BenchIdle<P, Ruler>;
using Meter = IsrMeter<Ruler, Idle>;

Meter usart_meter;   ///< the console's vector
Meter tick_meter;    ///< SysTick
Meter probe_meter;   ///< letter r's stamp run: no vector touches it

TestBench<Serial> bench;

constexpr uint32_t sizes[] = {1, 16, 256, 4096};

/// The memory operations' buffers: static, in SRAM1, word-aligned so the
/// runtime takes its word path.
alignas(4) uint8_t src_buffer[4096];
alignas(4) uint8_t dst_buffer[4096];

/// letter p's text: 4096 printable characters and no line break, so a
/// run of n bytes is the last n of it.
constexpr auto print_pattern = [] {
    std::array<char, 4097> a{};
    for (uint32_t i = 0; i < 4096u; ++i) {
        a[i] = static_cast<char>('a' + i % 26u);
    }
    a[4096] = '\0';
    return a;
}();

BenchCounters counters() { return bench_counters<Idle>(usart_meter, tick_meter); }

/// A compiler barrier: memory is what the code says it is on both sides.
[[gnu::always_inline]] inline void fence() { asm volatile("" ::: "memory"); }

/// Wait for the console to fall silent: the ring empty, then the last
/// frame's stop bit out (SR.TC). Bounded at a second, some forty-five
/// times what a full 256-byte ring takes at 115200; false if it ran out.
bool console_drain() {
    const uint32_t t0 = Ticker::ticks();
    while (!Serial::tx_idle()) {
        if (Ticker::ticks() - t0 > 1000u) {
            return false;
        }
    }
    while (!Serial::Resource::tx_complete()) {
        if (Ticker::ticks() - t0 > 1000u) {
            return false;
        }
    }
    return true;
}

/// Return right after a tick: a short run started here sees none.
void tick_edge() {
    const uint32_t t = Ticker::ticks();
    while (Ticker::ticks() == t) {
    }
}

/// Idle turns, as the kernel takes them with nothing posted, until `ticks`
/// kernel ticks have passed.
void idle_for(uint32_t ticks) {
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < ticks) {
        Idle::CriticalSection cs;
        Idle::idle();
    }
}

/// x / n rounded to nearest; n is never zero here.
constexpr uint32_t average(uint32_t x, uint32_t n) { return (x + n / 2u) / n; }

/// The start of every letter: the ruler on (a probe may have been here).
void ruler_on() { (void)CycleCounter::init(); }

/// The idle turns of 100 kernel ticks, the console drained and the run
/// started on a tick edge: the totals, and how many turns they cover.
struct WindowRun {
    BenchSample all;
    uint32_t turns;
};

WindowRun window_run() {
    (void)console_drain();
    tick_edge();
    const uint32_t turns0 = Idle::idle_turns();
    const BenchCounters c0 = counters();
    Stopwatch<Ruler> sw;
    sw.start();
    idle_for(100u);
    const uint32_t wall = sw.elapsed();
    const BenchCounters c1 = counters();
    return {bench_sample(wall, c0, c1), Idle::idle_turns() - turns0};
}

/// One turn's share of a window run, every field rounded to nearest.
BenchSample per_turn(const WindowRun& w) {
    return {average(w.all.wall, w.turns), average(w.all.busy, w.turns), average(w.all.irq, w.turns),
            average(w.all.isr, w.turns)};
}

// =============================================================================
// r - the ruler's self-check and the instrument's cost
// =============================================================================
void tr_ruler() {
    const bool counting = CycleCounter::init();
    print(serial, "  DWT CYCCNT ", counting ? "counting" : "NOT COUNTING", ", hz=", Ruler::hz(),
          ", DBGMCU_CR.DBG_SLEEP=", Pwr::debug_in_sleep() ? 1 : 0, crlf);
    bench.verdict("the DWT's cycle counter is implemented and enabled", counting);

    // delay_us against the ruler: 999 us, the longest wait the core
    // stratum's delay serves (a whole SysTick period is refused).
    (void)console_drain();
    const uint32_t due = static_cast<uint32_t>(static_cast<uint64_t>(Ruler::hz()) * 999u / 1'000'000u);
    Stopwatch<Ruler> sw;
    sw.start();
    const bool served = delay_us(clock, 999u);
    const uint32_t took = sw.elapsed();
    print(serial, "  delay_us(999) ", served ? "served" : "REFUSED", " in ", took, " cycles, ", due,
          " due", crlf);
    bench.verdict("delay_us(999) reads at least 999 us on the ruler", took >= due);
    bench.verdict("and under 999 us + 5 per cent", took <= due + due / 20u);

    // ruler: an empty Stopwatch, a thousand times.
    (void)console_drain();
    tick_edge();
    BenchCounters c0 = counters();
    uint32_t sum = 0;
    for (uint16_t i = 0; i < 1000u; ++i) {
        const uint32_t a = Ruler::now();
        const uint32_t b = Ruler::now();
        sum += b - a;
    }
    BenchCounters c1 = counters();
    bench_line(serial, "ruler", 0, bench_sample(average(sum, 1000u), c0, c1), Ruler::hz(), 0);

    // stamp: an empty enter() + leave() pair, a thousand times.
    (void)console_drain();
    tick_edge();
    const uint32_t recorded0 = probe_meter.cycles();
    c0 = counters();
    sw.start();
    for (uint16_t i = 0; i < 1000u; ++i) {
        fence();
        probe_meter.enter();
        fence();
        probe_meter.leave();
    }
    const uint32_t stamps = sw.elapsed();
    c1 = counters();
    const uint32_t recorded = probe_meter.cycles() - recorded0;
    bench_line(serial, "stamp", 0, bench_sample(average(stamps, 1000u), c0, c1), Ruler::hz(), 0);
    print(serial, "  an empty pair records ", average(recorded, 1000u),
          " cycles into its meter: subtract irq x that from isr", crlf);

    // window: the idle turns of 100 ticks, DBG_SLEEP as found.
    const bool dbg_sleep = Pwr::debug_in_sleep();
    const WindowRun found = window_run();
    bench_line(serial, "window", 0, per_turn(found), Ruler::hz(), 0);
    print(serial, "  ", found.turns, " turns in 100 ticks: wall ", found.all.wall, " busy ",
          found.all.busy, " irq ", found.all.irq, " isr ", found.all.isr, crlf);
    const uint32_t tenth = Ruler::hz() / 10u;
    bench.verdict("the ruler counts through idle(): 100 ticks read 100 ms within 1 per cent",
                  found.all.wall >= tenth - tenth / 100u && found.all.wall <= tenth + tenth / 100u);

    // The control: the same run with DBG_SLEEP the other way, then the bit
    // put back - whether the count runs through a Sleep without the probe's
    // bit is this family's open question (stm32f4/dwt.hpp).
    Pwr::debug_in_sleep(!dbg_sleep);
    const WindowRun other = window_run();
    Pwr::debug_in_sleep(dbg_sleep);
    bench_line(serial, dbg_sleep ? "window.dbg0" : "window.dbg1", 0, per_turn(other), Ruler::hz(), 0);
    print(serial, "  with DBG_SLEEP=", dbg_sleep ? 0 : 1, ", 100 ticks read ", other.all.wall,
          " cycles on the ruler, ", tenth, " in 100 ms", crlf);

    bench.verdict("ran", true);
}

// =============================================================================
// m - the runtime's memcpy and memset against the core's load/store floor
// =============================================================================
enum class MemOp : uint8_t { copy, fill };

/// The best of 8 runs of one operation over `n` bytes; the operation is a
/// template argument, so no branch sits inside the measured span.
template <MemOp op>
BenchSample memory_best_of_8(uint32_t n) {
    BenchSample best{UINT32_MAX, 0, 0, 0};
    for (uint8_t run = 0; run < 8u; ++run) {
        uint32_t len = n;
        asm volatile("" : "+r"(len));   // a run-time length: the runtime's call, never a builtin's expansion
        const BenchCounters c0 = counters();
        Stopwatch<Ruler> sw;
        sw.start();
        fence();
        if constexpr (op == MemOp::copy) {
            memcpy(dst_buffer, src_buffer, len);
        } else {
            memset(dst_buffer, 0x5A, len);
        }
        fence();
        const uint32_t w = sw.elapsed();
        const BenchCounters c1 = counters();
        if (w < best.wall) {
            best = bench_sample(w, c0, c1);
        }
    }
    return best;
}

void tm_memory() {
    ruler_on();
    for (uint32_t i = 0; i < sizeof(src_buffer); ++i) {
        src_buffer[i] = static_cast<uint8_t>(i);
    }
    (void)console_drain();
    const uint32_t copy_wire = 2u * Ruler::hz();   // a load and a store a word: 2 bytes a cycle
    const uint32_t fill_wire = 4u * Ruler::hz();   // a store a word: 4 bytes a cycle
    for (const uint32_t n : sizes) {
        const BenchSample s = memory_best_of_8<MemOp::copy>(n);
        bench_line(serial, "memcpy", n, s, Ruler::hz(), copy_wire);
        (void)console_drain();
    }
    for (const uint32_t n : sizes) {
        const BenchSample s = memory_best_of_8<MemOp::fill>(n);
        bench_line(serial, "memset", n, s, Ruler::hz(), fill_wire);
        (void)console_drain();
    }
    bench.verdict("ran", true);
}

// =============================================================================
// p - a print through the console, to the wire's end
// =============================================================================
void tp_print() {
    ruler_on();
    for (const uint32_t n : sizes) {
        const char* text = print_pattern.data() + (4096u - n);
        (void)console_drain();
        const BenchCounters c0 = counters();
        Stopwatch<Ruler> sw;
        sw.start();
        print(serial, text);
        const bool drained = console_drain();
        const uint32_t w = sw.elapsed();
        const BenchCounters c1 = counters();
        print(serial, crlf);
        if (!drained) {
            print(serial, "  the console did not drain within a second", crlf);
        }
        bench_line(serial, "print", n, bench_sample(w, c0, c1), Ruler::hz(), console_wire_bps);
    }
    bench.verdict("ran", true);
}

// =============================================================================
// t - the tick's floor
// =============================================================================
void tt_tick() {
    ruler_on();
    (void)console_drain();
    tick_edge();
    const BenchCounters c0 = counters();
    Stopwatch<Ruler> sw;
    sw.start();
    idle_for(1000u);
    const uint32_t w = sw.elapsed();
    const BenchCounters c1 = counters();
    bench_line(serial, "tick", 0, bench_sample(w, c0, c1), Ruler::hz(), 0);
    bench.verdict("ran", true);
}

void banner() {
    print(serial, crlf, "bench_stm32f4 - the benchmark skeleton: the ruler, memory, the console, "
          "the tick's floor", crlf);
    bench.menu();
}

/// The console's ISR body between its meter's stamps.
[[gnu::always_inline]] inline void console_vector() {
    usart_meter.enter();
    (void)Serial::isr();
    usart_meter.leave();
}

} // namespace

// ---- target glue ------------------------------------------------------------
#if defined(STM32F446xx)
extern "C" void USART2_IRQHandler() { console_vector(); }
#elif defined(STM32F469xx)
extern "C" void USART3_IRQHandler() { console_vector(); }
#else
extern "C" void USART1_IRQHandler() { console_vector(); }
#endif
extern "C" void SysTick_Handler() {
    tick_meter.enter();
    brio::Ticker::tick();
    tick_meter.leave();
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, console_baud);
    const bool tick_ok = brio::Ticker::init(clock);
    const bool ruler_ok = brio::CycleCounter::init();
    Led::output();
    brio::enable_interrupts();

    bench.letter('r', "the ruler's self-check and the instrument's cost", tr_ruler);
    bench.letter('m', "memcpy and memset against the core's load/store floor", tm_memory);
    bench.letter('p', "a print through the console, to the wire's end", tp_print);
    bench.letter('t', "the tick's floor: a second of idle", tt_tick);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL" : "FAILED", " tick=",
                    tick_ok ? "SysTick" : "FAILED", " ruler=", ruler_ok ? "DWT" : "FAILED",
                    brio::crlf);
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
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        bench.prompt();
    }
}
