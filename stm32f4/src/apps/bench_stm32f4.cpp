// bench_stm32f4 - the benchmark skeleton on the STM32F4 (design/benchmark.md):
// the ruler checked, the instrument's own cost, the runtime's memory
// operations against the core's load/store floor, a print through the
// console against the wire, the tick's floor, and the DMA's engines. NOT
// A TEST: every line
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
// idle_if_empty() would with nothing posted. The vectors bound - the
// console's USART, SysTick, letter d's three DMA streams and SPI1 - carry
// an IsrMeter (one for the console, one for the tick, one for the
// streams, one for SPI1): enter() its first statement, leave() its last,
// the driver's ISR body between them. irq and isr in every line are the
// meters' sums; letter d waits in the same idle turns.
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
//   d  THE DMA (stm32f4/dma.hpp), four operations:
//        copy   DmaCopyEngine on DMA2 stream 4: 16, 256 and 4096 bytes as
//               words between two 16-byte-aligned buffers in SRAM, polled
//               to the end with busy(), the best of 8. THE WIRE is one
//               beat a cycle of the DMA's clock, HCLK: a word a cycle, 4 x
//               hz. A line after the three sizes says the cycles per 100
//               bytes at the margin (the difference of the two larger
//               sizes) and the fixed cost of a block (the smallest, less
//               its bytes at that margin).
//        fill   the same, the source the engine reads at every beat being
//               a word in SRAM (fill_cell).
//        paced  256 words moved by TIM1's update request at 1 MHz (DMA2
//               stream 5, channel 6 - a cell of every manual's table), the
//               SOURCE the pacing timer's own counter read through
//               TIMx_DMAR with the burst base on CNT: each word is the
//               counter at its beat, so the spread of the 256 is the
//               jitter of the DMA's service against the pace, in ticks of
//               the timer's clock (HCLK on these boards). The thread
//               idles until the stream's completion; irq is that one
//               interrupt and the ticks, busy what the core spent. THE
//               WIRE is the pace: 4 bytes a microsecond.
//        spi.dma, spi.dma16
//               (the Nucleo-F446RE alone) an engined SpiHost request on
//               SPI1 - SCK PA5 (the LED, given back after), MISO PA6
//               floating on its pull-up, MOSI PA7 - DMA2 stream 3 out and
//               stream 2 in, channel 3: 16 and 256 frames of 8 and of 16
//               bits at SCK 22.5 and 5.625 MHz, ISR-style, the thread
//               idling until the completion. THE WIRE is SCK / 8 bytes a
//               second; the line after each says wall minus the wire's
//               time - the request's fixed cost. Before them, one
//               DmaTxEngine block start alone on an idle stream. The data
//               of the frames is judged nowhere: no wire joins MOSI to
//               MISO here, and the time is the wire's either way.
//
//   e  THE SPI HOST ABOVE THE WIRE (the Nucleo-F446RE alone): the same
//      SPI1 pads as letter d, MISO floating, but the ENGINELESS host -
//      every frame through the pump - and the bus's own loop against the
//      wire. Three operations, each the best of its runs:
//        spi.poll, spi.poll16
//               a polled WRITE of 16 and of 256 frames, 8 and 16 bits (tx
//               set, rx null: the host's transmit-only shape), at SCK 45,
//               22.5, 11.25 and 5.625 MHz (/2, /4, /8 and /16): wall
//               against the wire's time, the loop's own cost per frame
//               being what is above 1.00. THE WIRE is SCK / 8 bytes a
//               second. The line after each says the worst run's wall,
//               the worst status and the host's overrun count: a frame
//               lost to a handler shows there and never in the best.
//        spi.poll.rx, spi.poll16.rx
//               the same requests with rx set too (the receive shape:
//               one frame in flight below the host's write-ahead
//               threshold, two above it), the same line after.
//        spi.pump, spi.pump16
//               the same requests ISR-style, the thread idling until the
//               completion: the line after says how many times the SPI1
//               vector ran (interrupts per frame), what one run of it
//               cost, whether the host keeps one or two frames in flight
//               at that rate and width (its write-ahead threshold), and
//               whether an overrun was seen - the flag after the run, and
//               the host's own count - with the console's and the tick's
//               interrupts live throughout.
//        spi.req
//               THE FIXED COST: polled requests of 1, 3 and 16 bytes at
//               /16 - a command byte with D/C low and 0, 2 or 15 data
//               bytes with D/C high, rx null, the select on PC0 and D/C
//               on PC1 (two spare pads of this board, on no recorded
//               wire) - and the line after each says wall minus the
//               wire's cycles: the price of a DCS command.
//        spi.req.e, spi.poll.e, spi.poll.rx.e
//               the same short requests, and 256 frames written and read
//               polled at /2 and /4, over letter d's ENGINED host: where
//               the host's thresholds send a polled phase by its shape
//               (irq 1 = the engines' completion, 0 = the loop).
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
// NOTHING TO WIRE. Connect at 115200 8N1; `z` runs the six letters.
//
// build: boards = f429zi,f446re,f411ce,f469ni
// build: monitor_speed = 115200

#include <stdint.h>
#include <string.h>

#include <array>
#include <span>

#include "stm32f4/clock.hpp"
#include "stm32f4/delay.hpp"
#include "stm32f4/dma.hpp"
#include "stm32f4/dwt.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/pwr.hpp"
#include "stm32f4/spi.hpp"
#include "stm32f4/tim.hpp"
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
Meter dma_meter;     ///< letter d's DMA streams
Meter spi_meter;     ///< letter d's SPI1 vector (a pumped request)

/// letter d's engines.
using Copier = DmaCopyEngine<2, 4>;
using PacedRx = DmaRxEngine<2, 5, 6, uint32_t>;
#if defined(STM32F446xx)
constexpr SpiPins spi1_pins{.sck = {'A', 5, PinFunction::af5},
                            .miso = {'A', 6, PinFunction::af5},
                            .mosi = {'A', 7, PinFunction::af5}};
using SpiTx = DmaTxEngine<2, 3, 3, uint16_t>;
using SpiRx = DmaRxEngine<2, 2, 3, uint16_t>;
using FastSpi = SpiHost<1, spi1_pins, SpiTx, SpiRx>;
/// letter e's host: the same instance and pads with no engine, so every
/// frame goes through the pump. The two hosts share SPI1's vector, and
/// `pump_host` says which one the vector serves (one predictable branch).
using PumpSpi = SpiHost<1, spi1_pins>;
using ReqCs = Pin<'C', 0>;   ///< letter e's select
using ReqDc = Pin<'C', 1>;   ///< letter e's D/C
volatile bool pump_host = false;
#endif
volatile bool paced_done = false;
volatile bool spi_done = false;

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

BenchCounters counters() { return bench_counters<Idle>(usart_meter, tick_meter, dma_meter, spi_meter); }

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

// =============================================================================
// d - the DMA: copy and fill, a paced transfer, an engined SPI request
// =============================================================================

/// The source and destination of the memory-to-memory runs: their own,
/// sixteen-byte aligned so both ports may burst (RM0390 9.3.12: a burst
/// must not cross a 1 KB boundary, which a 16-byte burst from a 16-byte
/// boundary never does).
alignas(16) uint8_t dma_src[4096];
alignas(16) uint8_t dma_dst[4096];

/// The fill's cell: a word in SRAM the stream reads at every beat.
alignas(4) uint32_t fill_cell = 0x5A5A5A5Au;

/// The paced run's 256 words: the pacing timer's own counter, one read a
/// request - so each word IS the latency of its beat.
alignas(4) volatile uint32_t stamps[256];

#if defined(STM32F446xx)
alignas(2) uint8_t spi_out[512];
alignas(2) volatile uint8_t spi_in[512];
#endif

/// The best of 8 runs of a copy or a fill of `n` bytes, polled to its end.
template <bool fill>
BenchSample dma_best_of_8(uint32_t n, bool& ok) {
    BenchSample best{UINT32_MAX, 0, 0, 0};
    ok = true;
    for (uint8_t run = 0; run < 8u; ++run) {
        const BenchCounters c0 = counters();
        Stopwatch<Ruler> sw;
        sw.start();
        fence();
        bool started = false;
        if constexpr (fill) {
            started = Copier::fill(reinterpret_cast<uint32_t*>(dma_dst), &fill_cell, n / 4u);
        } else {
            started = Copier::copy(reinterpret_cast<uint32_t*>(dma_dst),
                                   reinterpret_cast<const uint32_t*>(dma_src), n / 4u);
        }
        while (Copier::busy()) {
        }
        fence();
        const uint32_t w = sw.elapsed();
        const BenchCounters c1 = counters();
        if (!started) {
            ok = false;
        }
        if (w < best.wall) {
            best = bench_sample(w, c0, c1);
        }
    }
    for (uint32_t i = 0; i < n; ++i) {
        const uint8_t want = fill ? static_cast<uint8_t>(0x5A) : dma_src[i];
        if (dma_dst[i] != want) {
            ok = false;
            break;
        }
    }
    return best;
}

/// Wait in the idle path, as the kernel would with nothing posted, until
/// `done` rises; bounded at a second of ticks.
void idle_until_set(volatile bool& done) {
    const uint32_t t0 = Ticker::ticks();
    while (!done && Ticker::ticks() - t0 < 1000u) {
        Idle::CriticalSection cs;
        if (!done) {
            Idle::idle();
        }
    }
}

void td_dma() {
    ruler_on();
    for (uint32_t i = 0; i < sizeof(dma_src); ++i) {
        dma_src[i] = static_cast<uint8_t>(i * 7u + 3u);
    }
    Dma<2>::init();
    Copier::arm(DmaPriority::very_high);
    (void)console_drain();

    // copy and fill: the wire is one beat a cycle of the DMA's clock (HCLK),
    // a word beat on these aligned buffers - four bytes a cycle.
    const uint32_t dma_wire = 4u * Ruler::hz();
    constexpr uint32_t dma_sizes[3] = {16u, 256u, 4096u};
    uint32_t walls[2][3] = {};
    for (uint8_t op = 0; op < 2u; ++op) {
        for (uint8_t k = 0; k < 3u; ++k) {
            bool ok = false;
            const BenchSample s = op == 0u ? dma_best_of_8<false>(dma_sizes[k], ok)
                                           : dma_best_of_8<true>(dma_sizes[k], ok);
            walls[op][k] = s.wall;
            bench_line(serial, op == 0u ? "copy" : "fill", dma_sizes[k], s, Ruler::hz(), dma_wire);
            if (!ok) {
                print(serial, "  the block did NOT land byte-exact", crlf);
            }
            (void)console_drain();
        }
    }
    for (uint8_t op = 0; op < 2u; ++op) {
        // The margin from the difference of two sizes, the fixed cost from
        // the smallest: cycles per hundred bytes, to keep the arithmetic whole.
        const uint32_t per100 = (walls[op][2] - walls[op][1]) * 100u / (4096u - 256u);
        const uint32_t fixed = walls[op][0] - per100 * 16u / 100u;
        print(serial, "  ", op == 0u ? "copy" : "fill", ": ", per100, " cycles per 100 bytes at the margin, a fixed ",
              fixed, " cycles a block (start to completion, the ruler's read included)", crlf);
    }

    // paced: TIM1's update request at 1 MHz moves the timer's own counter
    // (through TIMx_DMAR with the burst base on CNT) into 256 words.
    using Pacer = Tim<1>;
    Pacer::init();
    Pacer::set_prescaler(0);
    const uint32_t tick_hz = Pacer::clock_hz(clock);
    (void)Pacer::set_period(tick_hz / 1'000'000u - 1u);
    Pacer::update();
    (void)Pacer::dma_burst(TimBurstBase::cnt, 1);
    for (uint16_t i = 0; i < 256u; ++i) {
        stamps[i] = 0xFFFFFFFFu;
    }
    PacedRx::arm(Pacer::dmar_address(), DmaPriority::very_high);
    Pacer::interrupts(Pacer::update_dma, true);
    paced_done = false;
    const bool armed = PacedRx::start(std::span<uint32_t>(const_cast<uint32_t*>(stamps), 256));
    (void)console_drain();
    tick_edge();
    const BenchCounters p0 = counters();
    Stopwatch<Ruler> psw;
    psw.start();
    Pacer::enable(true);
    idle_until_set(paced_done);
    const uint32_t pwall = psw.elapsed();
    const BenchCounters p1 = counters();
    Pacer::enable(false);
    Pacer::interrupts(Pacer::update_dma, false);
    Pacer::dma_burst_off();
    PacedRx::stop();
    Pacer::release();
    uint32_t lo = UINT32_MAX;
    uint32_t hi = 0;
    for (uint16_t i = 0; i < 256u; ++i) {
        const uint32_t v = stamps[i];
        lo = v < lo ? v : lo;
        hi = v > hi ? v : hi;
    }
    bench_line(serial, "paced", 1024u, bench_sample(pwall, p0, p1), Ruler::hz(), 4'000'000u);
    print(serial, "  ", armed ? "armed" : "REFUSED", ", ", paced_done ? "completed" : "NOT COMPLETED",
          "; the 256 beats read the counter ", lo, "..", hi, " ticks of ", tick_hz,
          " Hz after their update: a jitter of ", hi - lo, crlf);

#if defined(STM32F446xx)
    // spi.dma: SPI1 on PA5/PA6/PA7 with MISO floating (its pull-up), the
    // data phase on DMA2 streams 3 (out) and 2 (in), channel 3, ISR style.
    for (uint16_t i = 0; i < sizeof(spi_out); ++i) {
        spi_out[i] = static_cast<uint8_t>(i);
    }
    Dma<2>::init();
    // One block start on an idle stream, the engine's verb alone: SPI1's
    // gate is closed, so no request reaches the stream and it waits.
    SpiTx::arm(Spi<1>::data_address());
    uint32_t start_best = UINT32_MAX;
    for (uint8_t run = 0; run < 8u; ++run) {
        Stopwatch<Ruler> sw;
        sw.start();
        const bool started = SpiTx::start(std::span<const uint8_t>(spi_out, 16));
        const uint32_t w = sw.elapsed();
        SpiTx::stop();
        if (started && w < start_best) {
            start_best = w;
        }
    }
    print(serial, "  one block start (DmaTxEngine::start, 16 bytes, an idle stream): ", start_best,
          " cycles, the ruler's read included", crlf);
    (void)FastSpi::init(clock);
    const SpiClock rates[2] = {SpiClock::div4, SpiClock::div16};
    const SpiDataSize widths[2] = {SpiDataSize::bits8, SpiDataSize::bits16};
    const uint16_t counts[2] = {16u, 256u};
    for (const SpiDataSize bits : widths) {
        for (const SpiClock rate : rates) {
            const uint32_t wire = FastSpi::sck_hz(rate) / 8u;
            for (const uint16_t frames : counts) {
                FastSpi::Request r{};
                r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(spi_out));
                r.rx = lend<Lease::reply>(const_cast<uint8_t*>(spi_in));
                r.len = frames;
                r.clock = rate;
                r.mode = SpiMode::mode0;
                r.bits = bits;
                r.polled = false;
                BenchSample best{UINT32_MAX, 0, 0, 0};
                bool all_done = true;
                for (uint8_t run = 0; run < 5u; ++run) {   // the first one applies the rate
                    (void)console_drain();
                    tick_edge();
                    spi_done = false;
                    const BenchCounters c0 = counters();
                    Stopwatch<Ruler> sw;
                    sw.start();
                    const bool sync = FastSpi::start(r);
                    if (!sync) {
                        idle_until_set(spi_done);
                    }
                    const uint32_t w = sw.elapsed();
                    const BenchCounters c1 = counters();
                    if (!sync && !spi_done) {
                        all_done = false;
                    }
                    if (run != 0u && w < best.wall) {
                        best = bench_sample(w, c0, c1);
                    }
                }
                const uint32_t bytes = bits == SpiDataSize::bits16 ? 2u * frames : frames;
                bench_line(serial, bits == SpiDataSize::bits16 ? "spi.dma16" : "spi.dma", bytes, best,
                           Ruler::hz(), wire);
                const uint32_t wire_cycles =
                    static_cast<uint32_t>(static_cast<uint64_t>(bytes) * Ruler::hz() / wire);
                print(serial, "  SCK ", FastSpi::sck_hz(rate), " Hz, ", frames, " frames: ",
                      all_done ? "completed" : "NOT COMPLETED", ", wall - wire = ",
                      static_cast<int32_t>(best.wall - wire_cycles), " cycles", crlf);
            }
        }
    }
    FastSpi::release();
    Led::output();
#else
    print(serial, "  spi.dma declined on this board: the free pads are surveyed on the "
                  "Nucleo-F446RE alone", crlf);
#endif
    bench.verdict("ran", true);
}

// =============================================================================
// e - the SPI host above the wire: the polled loop, the pump, the fixed cost
// =============================================================================
#if defined(STM32F446xx)
/// One request timed `runs` times, the first run discarded (it applies
/// the rate); `polled` false idles until the vector reports the
/// completion. `spi_irq` and `spi_isr` are the SPI1 vector's own count
/// and cycles in the best run, `ovr` whether OVR stood after any run.
struct SpiRun {
    BenchSample best{UINT32_MAX, 0, 0, 0};
    uint32_t spi_irq = 0;
    uint32_t spi_isr = 0;
    uint32_t worst_wall = 0;
    uint8_t worst_status = spi_ok;
    bool ovr = false;
    bool all_done = true;
};

template <typename Host>
SpiRun spi_best_of(const typename Host::Request& r, uint8_t runs) {
    SpiRun out;
    for (uint8_t run = 0; run < runs; ++run) {
        (void)console_drain();
        tick_edge();
        spi_done = false;
        const uint32_t n0 = spi_meter.count();
        const uint32_t i0 = spi_meter.cycles();
        const BenchCounters c0 = counters();
        Stopwatch<Ruler> sw;
        sw.start();
        const bool sync = Host::start(r);
        if (!sync) {
            idle_until_set(spi_done);
        }
        const uint32_t w = sw.elapsed();
        const BenchCounters c1 = counters();
        if (Spi<1>::overrun()) {
            out.ovr = true;
            Spi<1>::clear_overrun();
        }
        if (!sync && !spi_done) {
            out.all_done = false;
        }
        if (Host::status() > out.worst_status) {
            out.worst_status = Host::status();
        }
        if (w > out.worst_wall) {
            out.worst_wall = w;
        }
        if (run != 0u && w < out.best.wall) {
            out.best = bench_sample(w, c0, c1);
            out.spi_irq = spi_meter.count() - n0;
            out.spi_isr = spi_meter.cycles() - i0;
        }
    }
    return out;
}
#endif

void te_spi() {
    ruler_on();
#if defined(STM32F446xx)
    for (uint16_t i = 0; i < sizeof(spi_out); ++i) {
        spi_out[i] = static_cast<uint8_t>(i);
    }
    pump_host = true;
    (void)PumpSpi::init(clock);
    const SpiClock rates[4] = {SpiClock::div2, SpiClock::div4, SpiClock::div8, SpiClock::div16};
    const SpiDataSize widths[2] = {SpiDataSize::bits8, SpiDataSize::bits16};
    const uint16_t counts[2] = {16u, 256u};
    for (const SpiDataSize bits : widths) {
        for (const SpiClock rate : rates) {
            const uint32_t wire = PumpSpi::sck_hz(rate) / 8u;
            for (const uint16_t frames : counts) {
                const uint32_t bytes = bits == SpiDataSize::bits16 ? 2u * frames : frames;
                const uint32_t wire_cycles =
                    static_cast<uint32_t>(static_cast<uint64_t>(bytes) * Ruler::hz() / wire);
                PumpSpi::Request r{};
                r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(spi_out));
                r.len = frames;
                r.clock = rate;
                r.mode = SpiMode::mode0;
                r.bits = bits;
                r.polled = true;
                // spi.poll: the WRITE shape, rx null.
                const SpiRun written = spi_best_of<PumpSpi>(r, 6);
                bench_line(serial, bits == SpiDataSize::bits16 ? "spi.poll16" : "spi.poll", bytes,
                           written.best, Ruler::hz(), wire);
                print(serial, "  SCK ", PumpSpi::sck_hz(rate), " Hz, ", frames, " frames written: wall - wire = ",
                      static_cast<int32_t>(written.best.wall - wire_cycles), " cycles, worst wall ",
                      written.worst_wall, ", status ", written.worst_status, written.ovr ? ", OVR SEEN" : "",
                      ", the host counted ", PumpSpi::overruns(), crlf);
                // spi.poll.rx: the RECEIVE shape, rx set.
                r.rx = lend<Lease::reply>(const_cast<uint8_t*>(spi_in));
                const SpiRun polled = spi_best_of<PumpSpi>(r, 6);
                bench_line(serial, bits == SpiDataSize::bits16 ? "spi.poll16.rx" : "spi.poll.rx", bytes,
                           polled.best, Ruler::hz(), wire);
                print(serial, "  SCK ", PumpSpi::sck_hz(rate), " Hz, ", frames, " frames read: wall - wire = ",
                      static_cast<int32_t>(polled.best.wall - wire_cycles), " cycles, ",
                      PumpSpi::two_in_flight(rate, bits) ? "two" : "one", " in flight, worst wall ",
                      polled.worst_wall, ", status ", polled.worst_status, polled.ovr ? ", OVR SEEN" : "",
                      ", the host counted ", PumpSpi::overruns(), crlf);
                r.polled = false;
                const SpiRun pumped = spi_best_of<PumpSpi>(r, 6);
                bench_line(serial, bits == SpiDataSize::bits16 ? "spi.pump16" : "spi.pump", bytes,
                           pumped.best, Ruler::hz(), wire);
                print(serial, "  SCK ", PumpSpi::sck_hz(rate), " Hz, ", frames, " frames pumped: ",
                      pumped.all_done ? "completed" : "NOT COMPLETED", ", SPI1 vector ", pumped.spi_irq,
                      " runs of ", pumped.spi_irq == 0u ? 0u : average(pumped.spi_isr, pumped.spi_irq),
                      " cycles, wall - wire = ", static_cast<int32_t>(pumped.best.wall - wire_cycles),
                      " cycles, ", PumpSpi::two_in_flight(rate, bits) ? "two" : "one", " in flight, worst wall ",
                      pumped.worst_wall, ", status ", pumped.worst_status, ", OVR after a run: ",
                      pumped.ovr ? "YES" : "no", ", the host counted ", PumpSpi::overruns(), crlf);
            }
        }
    }

    // spi.req: the fixed cost of a short polled request at /16 - a command
    // byte with D/C low, then 0, 2 or 15 data bytes with D/C high, rx
    // null, the select on PC0 and D/C on PC1.
    ReqCs::output(true);
    ReqDc::output(true);
    static const uint8_t command = 0x2C;
    const uint16_t lens[3] = {0u, 2u, 15u};
    const uint32_t req_wire = PumpSpi::sck_hz(SpiClock::div16) / 8u;
    for (const uint16_t len : lens) {
        PumpSpi::Request r{};
        r.cs = ReqCs::ref();
        r.dc = ReqDc::ref();
        r.cmd = lend<Lease::reply>(&command);
        r.cmd_len = 1;
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(spi_out));
        r.len = len;
        r.clock = SpiClock::div16;
        r.mode = SpiMode::mode0;
        r.bits = SpiDataSize::bits8;
        r.polled = true;
        const SpiRun req = spi_best_of<PumpSpi>(r, 9);
        const uint32_t bytes = 1u + len;
        const uint32_t wire_cycles =
            static_cast<uint32_t>(static_cast<uint64_t>(bytes) * Ruler::hz() / req_wire);
        bench_line(serial, "spi.req", bytes, req.best, Ruler::hz(), req_wire);
        print(serial, "  a command and ", len, " data bytes at SCK ", PumpSpi::sck_hz(SpiClock::div16),
              " Hz: wall - wire = ", static_cast<int32_t>(req.best.wall - wire_cycles),
              " cycles, the fixed cost", crlf);
    }
    PumpSpi::release();
    pump_host = false;

    // The same two shapes over the ENGINED host, where the thresholds
    // decide: spi.req.e, the three short polled requests at /16 (the pump,
    // by dma_min_frames and by the polled rule alike), and spi.poll.e, 256
    // frames polled at /2 (the engines: the loop cannot keep a 32-cycle
    // frame) and at /4 (the pump: the loop is wire-bound there).
    Dma<2>::init();
    (void)FastSpi::init(clock);
    for (const uint16_t len : lens) {
        FastSpi::Request r{};
        r.cs = ReqCs::ref();
        r.dc = ReqDc::ref();
        r.cmd = lend<Lease::reply>(&command);
        r.cmd_len = 1;
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(spi_out));
        r.len = len;
        r.clock = SpiClock::div16;
        r.mode = SpiMode::mode0;
        r.bits = SpiDataSize::bits8;
        r.polled = true;
        const SpiRun req = spi_best_of<FastSpi>(r, 9);
        const uint32_t bytes = 1u + len;
        const uint32_t wire_cycles =
            static_cast<uint32_t>(static_cast<uint64_t>(bytes) * Ruler::hz() / req_wire);
        bench_line(serial, "spi.req.e", bytes, req.best, Ruler::hz(), req_wire);
        print(serial, "  over the engined host, a command and ", len, " data bytes at SCK ",
              FastSpi::sck_hz(SpiClock::div16), " Hz: wall - wire = ",
              static_cast<int32_t>(req.best.wall - wire_cycles), " cycles, irq ", req.best.irq, crlf);
    }
    const SpiClock fast_rates[2] = {SpiClock::div2, SpiClock::div4};
    for (const bool with_rx : {false, true}) {
        for (const SpiClock rate : fast_rates) {
            FastSpi::Request r{};
            r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(spi_out));
            if (with_rx) {
                r.rx = lend<Lease::reply>(const_cast<uint8_t*>(spi_in));
            }
            r.len = 256;
            r.clock = rate;
            r.mode = SpiMode::mode0;
            r.bits = SpiDataSize::bits8;
            r.polled = true;
            const SpiRun polled = spi_best_of<FastSpi>(r, 6);
            const uint32_t wire = FastSpi::sck_hz(rate) / 8u;
            const uint32_t wire_cycles =
                static_cast<uint32_t>(static_cast<uint64_t>(256u) * Ruler::hz() / wire);
            bench_line(serial, with_rx ? "spi.poll.rx.e" : "spi.poll.e", 256u, polled.best, Ruler::hz(), wire);
            print(serial, "  over the engined host, 256 frames ", with_rx ? "read" : "written", " polled at SCK ",
                  FastSpi::sck_hz(rate), " Hz: wall - wire = ", static_cast<int32_t>(polled.best.wall - wire_cycles),
                  " cycles, irq ", polled.best.irq, " (one = the engines' completion, none = the loop)", crlf);
        }
    }
    FastSpi::release();
    ReqCs::release();
    ReqDc::release();
    Led::output();
#else
    print(serial, "  letter e declined on this board: the free pads are surveyed on the "
                  "Nucleo-F446RE alone", crlf);
#endif
    bench.verdict("ran", true);
}

void banner() {
    print(serial, crlf, "bench_stm32f4 - the benchmark skeleton: the ruler, memory, the console, "
          "the tick's floor, the DMA, the SPI host", crlf);
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
extern "C" void DMA2_Stream5_IRQHandler() {
    dma_meter.enter();
    if ((PacedRx::service() & PacedRx::flag_complete) != 0u) {
        paced_done = true;
    }
    dma_meter.leave();
}
#if defined(STM32F446xx)
extern "C" void DMA2_Stream2_IRQHandler() {
    dma_meter.enter();
    if (FastSpi::dma_isr()) {
        spi_done = true;
    }
    dma_meter.leave();
}
extern "C" void DMA2_Stream3_IRQHandler() {
    dma_meter.enter();
    if (FastSpi::dma_isr()) {
        spi_done = true;
    }
    dma_meter.leave();
}
extern "C" void SPI1_IRQHandler() {
    spi_meter.enter();
    const bool done = pump_host ? PumpSpi::isr() : FastSpi::isr();
    if (done) {
        spi_done = true;
    }
    spi_meter.leave();
}
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
    bench.letter('d', "the DMA: copy and fill, a paced transfer, an engined SPI request", td_dma);
    bench.letter('e', "the SPI host above the wire: polled, pumped, and a short request's fixed cost",
                 te_spi);

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
