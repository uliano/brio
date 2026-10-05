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
//   i  THE I2C HOST ABOVE THE WIRE (the STM32F429I-DISC1 alone): I2C3 on
//      PA8/PC9 against the board's STMPE811 at 0x41 - the one device the
//      board puts on that bus - through the engineless host (the pump) and
//      the host with DMA1 streams 4 and 2 on channel 3 (the engines),
//      ISR-style, the thread idling until the completion, each line the
//      best of 8 at 100 and 400 kHz (DUTY 2; this block has no Fm+):
//        i2c.write      1, 2, 16 and 255 bytes (and 3, last): SYS_CTRL2's
//                       index, its reset value, then zeros into the
//                       registers after it (the device's soft reset puts
//                       every one back at the letter's end, and the letter
//                       checks CHIP_ID)
//        i2c.wr         the register read: the index, a repeated START, 1,
//                       2, 16 and 255 bytes - the STMPE811 answers a read
//                       no write opened with a NACK on its address
//                       (measured, printed once), so a plain read is
//                       measured through this shape alone
//        i2c.probe      the address alone, 0x41 (ACK) and 0x42 (NACK)
//      irq and isr are the I2C vectors' alone (events, errors, the two
//      streams); busy is every meter's. THE WIRE is the tenure's own time
//      at the SCL period MEASURED on the pad: the clock pad's input buffer
//      polled through a 255-byte register read on the engines, its rising
//      edges counted over 240 data bytes (every clock of them, the
//      acknowledges included). A START and each repeated START count a
//      period, the address and each byte nine; the STOP is not counted,
//      because the engine completes when it REQUESTS the STOP. The line
//      after each says wall minus that wire - THE FIXED COST - and what
//      start() took on a quiet bus. Then, per speed:
//        per byte       the pump's handler cycles and entries per byte,
//                       the margin between the 16- and the 255-byte rows
//        i2c.stop       start() called the moment a write, a read and a
//                       register read completed, as an arbiter's dispatch
//                       calls the next tenure: what it spends waiting for
//                       the last STOP to leave, against a quiet bus
//      and, last, the longest single entry of each vector, watched in a
//      pass of its own (its two extra ruler reads would sit inside every
//      other line's isr).
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
// NOTHING TO WIRE. Connect at 115200 8N1; `z` runs every letter.
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
#include "stm32f4/i2c.hpp"
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
Meter loop_meter;    ///< letter u's USART6 vector
Meter loop_dma_meter;   ///< letter u's two DMA streams
Meter i2c_meter;        ///< letter i's two I2C3 vectors (events, errors)
Meter i2c_dma_meter;    ///< letter i's two DMA1 streams

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

BenchCounters counters() {
    return bench_counters<Idle>(usart_meter, tick_meter, dma_meter, spi_meter, loop_meter, loop_dma_meter,
                                i2c_meter, i2c_dma_meter);
}

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

// =============================================================================
// u - the serial transport on the single-wire loop
// =============================================================================
#if defined(STM32F446xx)
/// USART6 in single-wire half duplex on PC6 (AF8, DS10693 table 11), a
/// pad on no recorded wire: the receiver hears the transmitter's own
/// frames on the pad. APB2's instance, so the loop's top rate is 90 MHz /
/// 16. Its two DMA cells, stream 6 out and stream 1 in on channel 5
/// (RM0390 table 29), are free of letter d's.
constexpr UartPins loop_pins{.tx = {'C', 6, PinFunction::af8}, .rx = {'C', 7, PinFunction::af8}};
constexpr UartOptions loop_opts{.half_duplex = true, .tx_speed = PinSpeed::very_high,
                                 .single_wire_push_pull = true};
using LoopTxEngine = DmaTxEngine<2, 6, 5>;
using LoopRxEngine = DmaRxEngine<2, 1, 5>;
constexpr uint32_t loop_ring = 512;
/// The three shapes: both directions on interrupts; the transmit engine
/// with the interrupt receiver; both engines.
using LoopIrq = Uart<6, loop_pins, loop_ring, loop_ring, NoDmaEngine, NoDmaEngine, loop_opts>;
using LoopTxe = Uart<6, loop_pins, loop_ring, loop_ring, LoopTxEngine, NoDmaEngine, loop_opts>;
using LoopDma = Uart<6, loop_pins, loop_ring, loop_ring, LoopTxEngine, LoopRxEngine, loop_opts>;
using LoopRes = Usart<6>;
/// Which shape owns USART6 now: 0 none, 1 LoopIrq, 2 LoopTxe, 3 LoopDma.
volatile uint8_t loop_owner = 0;
/// The edge as the app's glue sees it: the vector returned true.
volatile bool loop_edge = false;
volatile uint32_t loop_edge_at = 0;

/// The rates: 115200, 1 Mbaud and the top, BRR 781, 90 and 16 at 90 MHz.
constexpr uint32_t loop_rates[] = {115200, 1'000'000, 5'625'000};

/// Give USART6 back. The owner is forgotten AFTER the release: stopping
/// a stream raises its completion flag (RM0390 9.3.17), and the vector
/// that serves it must still know whose it is.
void loop_down() {
    const uint8_t was = loop_owner;
    if (was == 1u) {
        LoopIrq::release();
    } else if (was == 2u) {
        LoopTxe::release();
    } else if (was == 3u) {
        LoopDma::release();
    }
    loop_owner = 0;
}

template <typename Port>
bool loop_up(uint8_t owner, uint32_t baud) {
    loop_down();
    loop_owner = owner;
    const bool up = Port::init(clock, baud);
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < 2u) {
    }
    loop_edge = false;
    return up;
}

/// The wire of the rate the port runs at: the divisor in the register over
/// ten bits a byte.
uint32_t loop_wire() { return LoopIrq::actual_baud(LoopIrq::kernel_hz<SysClock>()) / 10u; }

/// Drain what the loop holds, judging it against the pattern from `at`.
template <typename Port>
uint32_t loop_drain(uint32_t at, bool& in_order) {
    uint32_t got = 0;
    const uint8_t* want = reinterpret_cast<const uint8_t*>(print_pattern.data());
    for (;;) {
        const auto run = Port::read_span();
        if (run.empty()) {
            return got;
        }
        for (uint32_t i = 0; i < run.size(); ++i) {
            if (run[i] != want[(at + got + i) % 4096u]) {
                in_order = false;
            }
        }
        got += static_cast<uint32_t>(run.size());
        (void)Port::consume(static_cast<uint32_t>(run.size()));
    }
}

/// uart.tx: n bytes through the transport, the receiver off, the thread
/// idling while the ring is full and until the last stop bit.
template <typename Port>
BenchSample loop_tx(uint32_t n) {
    (void)console_drain();
    LoopRes::receiver(false);
    LoopRes::clear_flags(UsartFlag::tc);
    const uint8_t* src = reinterpret_cast<const uint8_t*>(print_pattern.data());
    const BenchCounters c0 = counters();
    Stopwatch<Ruler> sw;
    sw.start();
    uint32_t queued = 0;
    for (;;) {
        Idle::CriticalSection cs;
        queued += Port::write_bulk(std::span<const uint8_t>(src + queued, n - queued));
        if (queued >= n) {
            break;
        }
        Idle::idle();
    }
    // Idle while an interrupt is coming - the transmitter's TXE armed or
    // the engine's block in flight - and spin through the last frame,
    // whose stop bit raises no interrupt: tx_idle() is the wire's.
    for (;;) {
        Idle::CriticalSection cs;
        if (Port::tx_idle()) {
            break;
        }
        if (LoopRes::txe_interrupt() || LoopTxEngine::busy()) {
            Idle::idle();
        }
    }
    const uint32_t w = sw.elapsed();
    const BenchCounters c1 = counters();
    LoopRes::receiver(true);
    return bench_sample(w, c0, c1);
}

/// uart.rx: a burst of n bytes sent by the transmit engine and received
/// on the loop, the consumer idling until the edge and draining on it -
/// or, with `poll`, asking harvest() once a tick (the owner's poll).
template <typename Port, bool poll>
BenchSample loop_rx(uint32_t n, uint32_t& got, bool& in_order) {
    (void)console_drain();
    bool scrap = true;
    (void)loop_drain<Port>(0, scrap);
    Port::clear_errors();
    loop_edge = false;
    in_order = true;
    got = 0;
    const uint8_t* src = reinterpret_cast<const uint8_t*>(print_pattern.data());
    const BenchCounters c0 = counters();
    Stopwatch<Ruler> sw;
    sw.start();
    (void)Port::write_bulk(std::span<const uint8_t>(src, n));
    uint32_t tick = Ticker::ticks();
    const uint32_t t0 = tick;
    while (got < n && Ticker::ticks() - t0 < 1000u) {
        // The platform's idle() returns with interrupts ENABLED, so the
        // flag is taken under a guard of its own after it: a vector between
        // the read and the clear would be lost.
        {
            Idle::CriticalSection cs;
            if (!loop_edge) {
                Idle::idle();
            }
        }
        bool edge = false;
        {
            Idle::CriticalSection cs;
            edge = loop_edge;
            loop_edge = false;
        }
        if constexpr (poll) {
            if (Ticker::ticks() != tick) {
                tick = Ticker::ticks();
                if (Port::harvest()) {
                    edge = true;
                }
            }
        }
        if (edge) {
            got += loop_drain<Port>(got, in_order);
        }
    }
    const uint32_t w = sw.elapsed();
    const BenchCounters c1 = counters();
    return bench_sample(w, c0, c1);
}

/// uart.edge: a burst of 16, the sender's TC spun on, then the cycles to
/// the edge that follows it - the vector's true, or with `poll` the first
/// harvest() a tick that answers true.
template <typename Port, bool poll>
BenchSample loop_edge_after(uint32_t& late) {
    (void)console_drain();
    bool scrap = true;
    (void)loop_drain<Port>(0, scrap);
    LoopRes::clear_flags(UsartFlag::tc);
    loop_edge = false;
    const uint8_t* src = reinterpret_cast<const uint8_t*>(print_pattern.data());
    (void)Port::write_bulk(std::span<const uint8_t>(src, 16));
    uint32_t got = 0;
    while (!LoopRes::tx_complete()) {
        if constexpr (!poll) {
            if (loop_edge) {
                loop_edge = false;
                got += loop_drain<Port>(got, scrap);
            }
        }
    }
    const uint32_t t_tc = Ruler::now();
    const BenchCounters c0 = counters();
    uint32_t tick = Ticker::ticks();
    uint32_t at = 0;
    late = 0;
    for (;;) {
        if (loop_edge) {
            at = loop_edge_at;
            break;
        }
        if constexpr (poll) {
            if (Ticker::ticks() != tick) {
                tick = Ticker::ticks();
                if (Port::harvest()) {
                    at = Ruler::now();
                    break;
                }
            }
        }
        if (Ruler::now() - t_tc > Ruler::hz() / 100u) {
            late = 1;
            at = Ruler::now();
            break;
        }
    }
    const BenchCounters c1 = counters();
    (void)loop_drain<Port>(got, scrap);
    return bench_sample(at - t_tc, c0, c1);
}
#endif

void tu_uart() {
    ruler_on();
#if defined(STM32F446xx)
    bool all_up = true;
    for (const uint32_t baud : loop_rates) {
        all_up = loop_up<LoopIrq>(1, baud) && all_up;
        const uint32_t wire = loop_wire();
        print(serial, "  the loop at ", LoopIrq::actual_baud(LoopIrq::kernel_hz<SysClock>()), " baud", crlf);
        for (const uint32_t n : {256u, 4096u}) {
            bench_line(serial, "uart.tx", n, loop_tx<LoopIrq>(n), Ruler::hz(), wire);
        }
        all_up = loop_up<LoopTxe>(2, baud) && all_up;
        for (const uint32_t n : {256u, 4096u}) {
            bench_line(serial, "uart.tx.dma", n, loop_tx<LoopTxe>(n), Ruler::hz(), wire);
        }
        for (const uint32_t n : {16u, 256u}) {
            uint32_t got = 0;
            bool in_order = false;
            const BenchSample s = loop_rx<LoopTxe, false>(n, got, in_order);
            bench_line(serial, "uart.rx", n, s, Ruler::hz(), wire);
            if (got != n || !in_order) {
                print(serial, "  received ", got, in_order ? " in order" : " OUT OF ORDER", "; ORE ",
                      LoopTxe::hw_overruns(), " FE ", LoopTxe::frame_errors(), " ring ", LoopTxe::rx_overruns(), crlf);
            }
        }
        all_up = loop_up<LoopDma>(3, baud) && all_up;
        for (const uint32_t n : {16u, 256u}) {
            uint32_t got = 0;
            bool in_order = false;
            const BenchSample s = loop_rx<LoopDma, false>(n, got, in_order);
            bench_line(serial, "uart.rx.dma", n, s, Ruler::hz(), wire);
            if (got != n || !in_order) {
                print(serial, "  received ", got, in_order ? " in order" : " OUT OF ORDER", "; ORE ",
                      LoopDma::hw_overruns(), " FE ", LoopDma::frame_errors(), " ring ", LoopDma::rx_overruns(), crlf);
            }
        }
        {
            uint32_t late = 0;
            const BenchSample e = loop_edge_after<LoopDma, false>(late);
            bench_line(serial, "uart.edge", 16, e, Ruler::hz(), 0);
            print(serial, "  the edge ", e.wall, " cycles after the last stop bit = ",
                  e.wall * 10u / (Ruler::hz() / wire), " tenths of a frame", late != 0u ? " (TIMED OUT)" : "", crlf);
        }
        loop_down();
    }
    bench.verdict("the loop came up at every rate", all_up);
#else
    print(serial, "  letter u declined on this board: the loop's pad is surveyed on the "
                  "Nucleo-F446RE alone", crlf);
#endif
    bench.verdict("ran", true);
}

// =============================================================================
// i - the I2C host above the wire, on the board's touch controller
// =============================================================================
#if defined(STM32F429xx)
/// I2C3 on the STM32F429I-DISC1: PA8 SCL, PC9 SDA (AF4), the board's own
/// pull-ups, the STMPE811 at 0x41 the one device on the wire.
constexpr I2cPins i2c_pins{.scl = {'A', 8, PinFunction::af4}, .sda = {'C', 9, PinFunction::af4}};
using I2cTxE = DmaTxEngine<1, 4, 3>;
using I2cRxE = DmaRxEngine<1, 2, 3>;
using I2cPump = I2cHost<3, i2c_pins>;
using I2cDma = I2cHost<3, i2c_pins, I2cTxE, I2cRxE>;
using I2cRes = I2c<3>;
using I2cScl = Pin<'A', 8>;
constexpr uint8_t i2c_peer = 0x41;     ///< the STMPE811
constexpr uint8_t i2c_nobody = 0x42;   ///< nobody on this wire (test_stm32f4_i2c's scan)
constexpr uint8_t stmpe_sys_ctrl1 = 0x03;
constexpr uint8_t stmpe_sys_ctrl2 = 0x04;
/// Which host the two vectors serve: 1 the pump, 2 the engines.
volatile uint8_t i2c_owner = 0;
/// The longest single entry of each vector over the letter, in cycles of
/// the body between two ruler reads of its own (R5's figure).
uint32_t i2c_ev_max = 0;
uint32_t i2c_er_max = 0;
uint32_t i2c_dma_max = 0;
/// Watched in a pass of its own: its two extra ruler reads are inside
/// the meters' stamps, and every other line reads the bodies without them.
volatile bool i2c_watching = false;
[[gnu::always_inline]] inline void i2c_longest(uint32_t& max, uint32_t t0) {
    const uint32_t d = Ruler::now() - t0;
    if (d > max) {
        max = d;
    }
}
volatile bool i2c_done = false;
alignas(4) uint8_t i2c_out[256];
alignas(4) uint8_t i2c_in[256];

/// The operations, each a Request shape.
struct I2cOp {
    const char* name;
    uint8_t addr;
    uint8_t tx;
    uint8_t rx;
};
constexpr I2cOp i2c_ops[] = {
    {"i2c.write", i2c_peer, 1, 0},   {"i2c.write", i2c_peer, 2, 0},
    {"i2c.write", i2c_peer, 16, 0},  {"i2c.write", i2c_peer, 255, 0},
    {"i2c.wr", i2c_peer, 1, 1},      {"i2c.wr", i2c_peer, 1, 2},
    {"i2c.wr", i2c_peer, 1, 16},     {"i2c.wr", i2c_peer, 1, 255},
    {"i2c.probe", i2c_peer, 0, 0},   {"i2c.probe", i2c_nobody, 0, 0},
    {"i2c.write", i2c_peer, 3, 0},   // the third short write the polled-verb question asks for
};
/// Where the per-byte figures come from: the rows of 16 and 255 data
/// bytes, written and read.
constexpr uint8_t i2c_row_w16 = 2;
constexpr uint8_t i2c_row_w255 = 3;
constexpr uint8_t i2c_row_r16 = 6;
constexpr uint8_t i2c_row_r255 = 7;

template <typename Host>
typename Host::Request i2c_request(uint8_t addr, uint8_t tx, uint8_t rx, I2cSpeed speed) {
    typename Host::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(i2c_out));
    r.tx_len = tx;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(i2c_in));
    r.rx_len = rx;
    r.speed = speed;
    return r;
}

/// The last tenure's STOP off the wire and BUSY down, outside every
/// measurement; bounded at ten ticks.
void i2c_quiet() {
    const uint32_t t0 = Ticker::ticks();
    while ((I2cRes::stopping() || I2cRes::busy()) && Ticker::ticks() - t0 < 10u) {
    }
}

template <typename Host>
void i2c_take(bool engines) {
    i2c_owner = 0;
    if (engines) {
        Dma<1>::init();
    }
    (void)Host::init(clock);
    i2c_owner = engines ? 2u : 1u;
}

/// One tenure, ISR-style, the thread idling until the completion: the
/// sample (busy from every meter, irq and isr the I2C vectors' alone),
/// the cycles start() itself took, and the status (0xFF: never answered).
struct I2cRun {
    BenchSample s;
    uint32_t start;
    uint8_t status;
};

template <typename Host>
I2cRun i2c_once(const typename Host::Request& r) {
    i2c_quiet();
    const BenchCounters c0 = counters();
    const uint32_t q0 = i2c_meter.count() + i2c_dma_meter.count();
    const uint32_t k0 = i2c_meter.cycles() + i2c_dma_meter.cycles();
    i2c_done = false;
    Stopwatch<Ruler> sw;
    sw.start();
    const bool sync = Host::start(r);
    const uint32_t st = sw.elapsed();
    if (!sync) {
        idle_until_set(i2c_done);
    }
    const uint32_t w = sw.elapsed();
    const BenchCounters c1 = counters();
    BenchSample s = bench_sample(w, c0, c1);
    s.irq = i2c_meter.count() + i2c_dma_meter.count() - q0;
    s.isr = i2c_meter.cycles() + i2c_dma_meter.cycles() - k0;
    return {s, st, (sync || i2c_done) ? Host::status() : static_cast<uint8_t>(0xFF)};
}

/// The SCL period in core cycles, on the wire: a register read of 255
/// bytes through the engines while the thread watches the clock pad's own input buffer
/// (an open-drain alternate function: its IDR is the wire), the rising
/// edges counted from the end of the second data byte over 240 bytes -
/// every bit of them, the acknowledges included, with no software
/// sequence inside the span to stretch it. 0 if the span was not seen.
uint32_t i2c_scl_period(I2cSpeed speed) {
    i2c_take<I2cDma>(true);
    i2c_out[0] = 0x00;
    const auto r = i2c_request<I2cDma>(i2c_peer, 1, 255, speed);
    i2c_quiet();
    i2c_done = false;
    // The address, the index, the repeated START's own rising edge, the
    // address again and two data bytes.
    constexpr uint32_t first = 9u + 9u + 1u + 9u + 9u * 2u;
    constexpr uint32_t span = 9u * 240u;
    uint32_t edges = 0;
    uint32_t t1 = 0;
    uint32_t t2 = 0;
    bool level = I2cScl::read();
    const uint32_t t0 = Ruler::now();
    (void)I2cDma::start(r);
    while (!i2c_done && Ruler::now() - t0 < Ruler::hz() / 10u) {
        const bool now = I2cScl::read();
        if (now && !level) {
            ++edges;
            if (edges == first) {
                t1 = Ruler::now();
            } else if (edges == first + span) {
                t2 = Ruler::now();
            }
        }
        level = now;
    }
    idle_until_set(i2c_done);
    if (t2 == 0u) {
        print(serial, "  the SCL span was not seen: ", edges, " rising edges, status ",
              i2c_done ? I2cDma::status() : 0xFFu, crlf);
    }
    return t2 != 0u ? (t2 - t1 + span / 2u) / span : 0u;
}

/// The bus's own time for a tenure at an SCL period of `t` cycles: the
/// START and each repeated START a period, the address and every byte
/// nine (eight bits and the acknowledge). The STOP is not counted: every
/// engine of this lineage completes when it REQUESTS the STOP, and what
/// the STOP's drain costs the next tenure is its own line (i2c.stop).
uint32_t i2c_wire_cycles(uint32_t t, uint8_t tx, uint8_t rx) {
    uint32_t periods = 1u + 9u;
    periods += 9u * tx;
    if (rx != 0u) {
        if (tx != 0u) {
            periods += 1u + 9u;
        }
        periods += 9u * rx;
    }
    return t * periods;
}

const char* i2c_status_name(uint8_t st) {
    switch (st) {
        case i2c_ok: return "ok";
        case i2c_nack_addr: return "nack_addr";
        case i2c_nack_data: return "nack_data";
        case i2c_arb_lost: return "arb_lost";
        case i2c_bus_error: return "bus_error";
        case 0xFF: return "NEVER ANSWERED";
        default: return "other";
    }
}

/// Every operation through one host at one speed, each the best of 8 by
/// wall, the worst status of the 8 printed after it.
template <typename Host>
void i2c_ops_on(const char* suffix, I2cSpeed speed, uint32_t t, bool engines) {
    i2c_take<Host>(engines);
    BenchSample rows[sizeof(i2c_ops) / sizeof(i2c_ops[0])] = {};
    uint8_t row = 0;
    for (const I2cOp& op : i2c_ops) {
        const auto r = i2c_request<Host>(op.addr, op.tx, op.rx, speed);
        I2cRun best{{UINT32_MAX, 0, 0, 0}, 0, 0};
        uint8_t worst = i2c_ok;
        const uint8_t want = op.addr == i2c_nobody ? i2c_nack_addr : i2c_ok;
        for (uint8_t run = 0; run < 8u; ++run) {
            const I2cRun one = i2c_once<Host>(r);
            if (one.status != want) {
                worst = one.status;
            }
            if (one.s.wall < best.s.wall) {
                best = one;
            }
        }
        const uint32_t wire = i2c_wire_cycles(t, op.tx, op.rx);
        const uint32_t n = static_cast<uint32_t>(op.tx) + op.rx;
        char name[24] = {};
        uint8_t k = 0;
        for (const char* c = op.name; *c != '\0' && k < 15u; ++c) {
            name[k++] = *c;
        }
        for (const char* c = suffix; *c != '\0' && k < 23u; ++c) {
            name[k++] = *c;
        }
        bench_line(serial, name, n, best.s, Ruler::hz(),
                   n == 0u ? 0u : static_cast<uint32_t>(static_cast<uint64_t>(n) * Ruler::hz() / wire));
        print(serial, "  ", op.addr == i2c_nobody ? "absent address, " : "",
              "fixed=", static_cast<int32_t>(best.s.wall - wire), " start=", best.start,
              " status=", i2c_status_name(worst == i2c_ok || worst == want ? want : worst), crlf);
        rows[row++] = best.s;
        (void)console_drain();
    }
    // The margin between 16 and 255 bytes: what a byte costs the pump's
    // handlers (the engines take none).
    if (!engines) {
        print(serial, "  per byte: written ", (rows[i2c_row_w255].isr - rows[i2c_row_w16].isr) / 239u,
              " cycles of handler and ", rows[i2c_row_w255].irq - rows[i2c_row_w16].irq,
              " interrupts in 239; read ", (rows[i2c_row_r255].isr - rows[i2c_row_r16].isr) / 239u,
              " cycles and ", rows[i2c_row_r255].irq - rows[i2c_row_r16].irq, " interrupts in 239", crlf);
    }
}

/// Every operation once through `Host` with the vectors' longest entry
/// watched (R5): a pass of its own, so no other line carries the watch.
template <typename Host>
void i2c_watch(I2cSpeed speed, bool engines) {
    i2c_take<Host>(engines);
    i2c_watching = true;
    for (const I2cOp& op : i2c_ops) {
        (void)i2c_once<Host>(i2c_request<Host>(op.addr, op.tx, op.rx, speed));
    }
    i2c_watching = false;
}

/// start() called the moment a tenure completed, as an arbiter's dispatch
/// calls the next one: what it spends waiting for the STOP to leave and
/// BUSY to fall, against the same start() on a quiet bus.
void i2c_stop_drain(I2cSpeed speed) {
    i2c_take<I2cPump>(false);
    const auto next = i2c_request<I2cPump>(i2c_peer, 0, 0, speed);
    const I2cOp firsts[] = {{"write", i2c_peer, 2, 0}, {"read", i2c_peer, 0, 2}, {"wr", i2c_peer, 1, 1}};
    for (const I2cOp& op : firsts) {
        const auto r = i2c_request<I2cPump>(op.addr, op.tx, op.rx, speed);
        uint32_t best = UINT32_MAX;
        uint32_t quiet = UINT32_MAX;
        for (uint8_t run = 0; run < 8u; ++run) {
            i2c_quiet();
            i2c_done = false;
            (void)I2cPump::start(r);
            idle_until_set(i2c_done);
            i2c_done = false;
            Stopwatch<Ruler> sw;
            sw.start();
            (void)I2cPump::start(next);
            const uint32_t behind = sw.elapsed();
            idle_until_set(i2c_done);
            const uint32_t q = i2c_once<I2cPump>(next).start;
            best = behind < best ? behind : best;
            quiet = q < quiet ? q : quiet;
        }
        print(serial, "bench i2c.stop after a ", op.name, " n=", static_cast<uint32_t>(op.tx) + op.rx,
              ": start() ", best, " cycles right behind its completion, ", quiet,
              " on a quiet bus - the drain ", best - quiet, " cycles", crlf);
        (void)console_drain();
    }
}
#endif

void ti_i2c() {
    ruler_on();
#if defined(STM32F429xx)
    i2c_out[0] = stmpe_sys_ctrl2;   // a register index: SYS_CTRL2, its reset value after it,
    i2c_out[1] = 0x0F;              // then zeros into the registers that follow
    for (uint16_t i = 2; i < sizeof(i2c_out); ++i) {
        i2c_out[i] = 0;
    }
    i2c_take<I2cPump>(false);
    i2c_ev_max = 0;
    i2c_er_max = 0;
    i2c_dma_max = 0;
    for (const I2cSpeed speed : {I2cSpeed::standard_100k, I2cSpeed::fast_400k}) {
        uint32_t t = i2c_scl_period(speed);
        print(serial, "  SCL at ", speed == I2cSpeed::fast_400k ? "400" : "100", " kHz: CCR ",
              hex(I2cPump::timing_of(speed).ccr), " states ", I2cPump::scl_hz(speed), " Hz, the wire ",
              t, " cycles a period = ", t != 0u ? Ruler::hz() / t : 0u, " Hz", crlf);
        if (t == 0u) {
            t = Ruler::hz() / I2cPump::scl_hz(speed);
        }
        (void)console_drain();
        if (speed == I2cSpeed::standard_100k) {
            // A read no write opened: the STMPE811's answer, measured.
            i2c_take<I2cPump>(false);
            const uint8_t plain = i2c_once<I2cPump>(i2c_request<I2cPump>(i2c_peer, 0, 1, speed)).status;
            print(serial, "  a plain read of the STMPE811 (no index written first) answers ",
                  i2c_status_name(plain), ": the read procedures are measured through i2c.wr", crlf);
        }
        i2c_out[0] = stmpe_sys_ctrl2;
        i2c_ops_on<I2cPump>("", speed, t, false);
        i2c_ops_on<I2cDma>(".dma", speed, t, true);
        i2c_stop_drain(speed);
        i2c_watch<I2cPump>(speed, false);
        i2c_watch<I2cDma>(speed, true);
    }
    // The device back where its reset leaves it: SYS_CTRL1's soft reset.
    i2c_take<I2cPump>(false);
    i2c_out[0] = stmpe_sys_ctrl1;
    i2c_out[1] = 0x02;
    const uint8_t rst = i2c_once<I2cPump>(i2c_request<I2cPump>(i2c_peer, 2, 0, I2cSpeed::standard_100k)).status;
    idle_for(10u);
    i2c_out[0] = 0x00;
    const uint8_t id = i2c_once<I2cPump>(i2c_request<I2cPump>(i2c_peer, 1, 2, I2cSpeed::standard_100k)).status;
    const bool chip = id == i2c_ok && i2c_in[0] == 0x08 && i2c_in[1] == 0x11;
    i2c_out[0] = stmpe_sys_ctrl2;
    (void)i2c_once<I2cPump>(i2c_request<I2cPump>(i2c_peer, 1, 1, I2cSpeed::standard_100k));
    print(serial, "  the STMPE811 soft-reset (", i2c_status_name(rst), "): CHIP_ID ", chip ? "0x0811" : "WRONG",
          ", SYS_CTRL2 ", hex(i2c_in[0]), crlf);
    print(serial, "  the longest entry of each vector: events ", i2c_ev_max, " cycles, errors ", i2c_er_max,
          ", the streams ", i2c_dma_max, crlf);
    bench.verdict("the device answers its identity after the letter", chip);
#else
    print(serial, "  letter i declined on this board: the I2C host is measured on the "
                  "STM32F429I-DISC1's STMPE811 alone", crlf);
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
#if defined(STM32F446xx)
extern "C" void USART6_IRQHandler() {
    loop_meter.enter();
    bool edge = false;
    if (loop_owner == 1u) {
        edge = LoopIrq::isr();
    } else if (loop_owner == 2u) {
        edge = LoopTxe::isr();
    } else if (loop_owner == 3u) {
        edge = LoopDma::isr();
    }
    if (edge) {
        loop_edge_at = Ruler::now();
        loop_edge = true;
    }
    loop_meter.leave();
}
/// The loop's two streams: the transmit engine's completions, and the
/// receive engine's lap marks - whose true is the edge, as the USART's.
[[gnu::always_inline]] inline void loop_dma_vector() {
    loop_dma_meter.enter();
    bool edge = false;
    if (loop_owner == 2u) {
        edge = LoopTxe::dma_isr();
    } else if (loop_owner == 3u) {
        edge = LoopDma::dma_isr();
    }
    if (edge) {
        loop_edge_at = Ruler::now();
        loop_edge = true;
    }
    loop_dma_meter.leave();
}
extern "C" void DMA2_Stream1_IRQHandler() { loop_dma_vector(); }
extern "C" void DMA2_Stream6_IRQHandler() { loop_dma_vector(); }
#endif
#if defined(STM32F429xx)
extern "C" void I2C3_EV_IRQHandler() {
    i2c_meter.enter();
    const uint32_t t0 = i2c_watching ? Ruler::now() : 0u;
    if (i2c_owner == 2u ? I2cDma::isr() : I2cPump::isr()) {
        i2c_done = true;
    }
    if (i2c_watching) {
        i2c_longest(i2c_ev_max, t0);
    }
    i2c_meter.leave();
}
extern "C" void I2C3_ER_IRQHandler() {
    i2c_meter.enter();
    const uint32_t t0 = i2c_watching ? Ruler::now() : 0u;
    if (i2c_owner == 2u ? I2cDma::error_isr() : I2cPump::error_isr()) {
        i2c_done = true;
    }
    if (i2c_watching) {
        i2c_longest(i2c_er_max, t0);
    }
    i2c_meter.leave();
}
/// The engines' two streams, DMA1 4 (transmit) and 2 (receive).
[[gnu::always_inline]] inline void i2c_dma_vector() {
    i2c_dma_meter.enter();
    const uint32_t t0 = i2c_watching ? Ruler::now() : 0u;
    if (I2cDma::dma_isr()) {
        i2c_done = true;
    }
    if (i2c_watching) {
        i2c_longest(i2c_dma_max, t0);
    }
    i2c_dma_meter.leave();
}
extern "C" void DMA1_Stream4_IRQHandler() { i2c_dma_vector(); }
extern "C" void DMA1_Stream2_IRQHandler() { i2c_dma_vector(); }
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
    bench.letter('u', "the serial transport on the single-wire loop: transmit, receive, the edge", tu_uart);
    bench.letter('i', "the I2C host above the wire: tenures, their fixed cost, the STOP's drain", ti_i2c);

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
