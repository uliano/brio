// bench_avr - the benchmark skeleton on the AVR DA/DB (docs/design/
// benchmark.md, util/bench.hpp): four letters that print NUMBERS, one
// `bench` line per operation and size, in the grammar every family
// prints. NOT A TEST: a letter's one verdict is "ran", except letter r,
// whose verdicts judge the ruler every other line is read with, and
// letter e, whose one verdict judges the pump it times.
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
// 2 clocks TCB2; a software event on channel 5 latches both halves into
// their CCMP registers (TCB2 through CASCADE, one CLK_PER behind), and
// read() waits for both CAPT flags, bounded, then reads the two
// captures. The console takes USART2 and the timebase the RTC's PIT at
// 1024 Hz (avrdx/ticker.hpp), nothing else, so the two TCBs and the two
// channels are free. The carry is on channel 2 and not 4 because a PORTE
// pin's event reaches channels 4 and 5 alone (16.5) and letter u takes
// one of them. now() is always_inline AND
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
// USART2_DRE (`dre_meter`), RTC_PIT (`tick_meter`), and letter u's two
// USART4 vectors in their metered binding (`u4_meter`) - all on Ruler, the
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
//      CRLF; the 1-byte one is the final LF), then the DRAIN: tx_idle(),
//      which is the wire's - the ring empty and the last stop bit out,
//      TXCIF cleared by the transport behind the run's last byte (27.5.5).
//      Counters before the print and after the
//      drain. busy = wall (the print and the drain spin); irq and isr are
//      the transport's shape - one DRE interrupt per byte, the silicon's
//      (TXDATA is one byte deep beside the shifter) - plus the ticks that
//      landed, the split printed on the line above.
//   t  the tick's floor: one second (hz cycles on the ruler) of cli()
//      then Idle::idle() turns with the console drained - not a kernel
//      loop: no kernel runs here. wall = the second, irq = the ticks
//      (1024 on the 32 kHz oscillator's nominal rate), isr = the tick
//      handler's cycles, busy = the floor. n=0, wire=0.
//   e  THE SPI HOST (avrdx/spi.hpp, docs/avrdx/spi.md): SpiHost<0> on
//      ALT1 - PE0 (MOSI), PE2 (SCK) and PE1 (MISO, FLOATING: the bytes
//      are not judged, the time is the wire's) - the pins test_avr_spi
//      runs on, its own vector SPI0_INT metered (`spi_meter`). Every
//      line is the BEST of 8 by wall. No kernel: a pumped request is
//      launched by start() and the thread IDLES until the vector's
//      completion edge, so busy is the launch, the handlers and the
//      loop's turns. n counts BYTES (the frame is a byte here).
//        spi.poll    a polled full-duplex data phase (tx and rx both
//                    set) of 16 and 256 bytes at CLK_PER/4 and /16: no
//                    command, no select, no D/C;
//        spi.poll.tx the same, write-only (rx null) - the display's
//                    shape, the one A1 of the retrospective counted;
//        spi.pump    spi.poll's requests on the interrupt pump (polled
//                    false): irq and isr are the pump's shape, the line
//                    after prints both PER BYTE and the BUFOVF flag read
//                    after the run (a one means the two-entry receive
//                    FIFO overflowed: the pump lost a byte). They run
//                    AFTER the polled lines, on the receiver those left:
//                    the letter's verdict is that every one of their
//                    requests completed and left BUFOVF clear;
//        spi.req     THE PRICE OF A DCS COMMAND: a polled request with a
//                    one-byte command phase and a data phase of 0, 2 and
//                    15 bytes (n = 1, 3, 16) at CLK_PER/16, the select on
//                    PA2 and the D/C on PA3 - two real pads, driven by
//                    the engine as a panel's would be (free on both bench
//                    boards: TWI0's default pins, a GPIO while no TWI
//                    runs; the DB's PF0/PF1 carry the DA's 32 kHz
//                    crystal) - and the line after prints wall minus the
//                    wire's cycles, the fixed cost of the request.
//      The vendor's column (`spi.vendor` in benchmark.md's protocol) is
//      not in this image: the AVR has no vendor library, and the data
//      sheet's own sequence (28.3.2.1.1 and 28.3.2.1.2) runs as a bare
//      loop in a scratch program beside this one, the number in
//      docs/avrdx/spi.md.
//   u  THE UART TRANSPORT (avrdx/usart.hpp's Uart, docs/avrdx/usart.md):
//      `Loop` = Uart<4, Route::def, 256, 256> on USART4's default route
//      in internal loop-back (LBME: the receiver listens to the TXD pad
//      PE0, RXD PE1 is not read) - test_avr_serial's single-board loop,
//      nothing to wire. Four rates: 115200 and 1 000 000 in normal mode
//      (BAUD 833 and 96), 2 000 000 and 3 000 000 in double speed (RXMODE
//      CLK2X, BAUD 96 and 64 - CLK_PER / 8, the register's floor and the
//      loop's top rate). The
//      console, USART2, is never touched. The two USART4 vectors are bound
//      FOUR WAYS at once, chosen per run by GPR0 (the general purpose
//      register at I/O 0x1C, which SBIS/SBIC read without touching SREG):
//      the vector is naked, a chain of skips and JMPs, and each binding is
//      a handler gcc compiles whole with its own prologue -
//        plain    (GPR0 = 0) the driver's body alone, the binding an app
//                 writes, behind the dispatch's first exit (SBIS and JMP,
//                 four cycles): the line the wire sees, isr=0;
//        metered  (0x81) the IsrMeter around the body, as every other
//                 vector here: irq and isr exact, but the stamps cost
//                 about 220 cycles an entry - nearly a frame at 1 Mbaud
//                 (240) and nearly three at 3 Mbaud (80);
//        counted  (0x84) the body and a 16-bit entry count, twelve cycles:
//                 how many entries the bytes took, printed as a note;
//        stamped  (0x82, RXC only) the body, then TCB0's count stored as
//                 the moment the entry left the body (uart.edge).
//      Every op runs metered, counted and plain, ONE run each (the loop is
//      deterministic); n counts bytes.
//        uart.tx       256 and 4096 bytes queued by write_bulk() as the
//                      ring takes them, the thread spinning on a full ring
//                      (print's policy: busy = wall), the wall closed by
//                      tx_idle(), the wire's (the last stop bit out) - the
//                      receiver OFF, so the line is the transmitter's
//                      alone; the plain run is `uart.tx.bare`;
//        uart.rx       bursts of 16 and 256 bytes through the interrupt
//                      receiver, SENT BY NO CPU: the transmitter off, PE0
//                      is PORT's and TCA0's WO0 drives it (single slope,
//                      PER one frame, the start bit from the compare match
//                      to TOP), a stream of 0xFF frames back to back at
//                      the generator's rate, so the receiver alone is
//                      measured - on a loop the same core fed by the thread
//                      at 3 Mbaud would measure the thread; the thread
//                      drains the ring by read_span()/consume() as it goes
//                      and sums the bytes; the wall runs from the timer's
//                      enable to the n-th byte consumed; the line after
//                      says what arrived and the three counters that can
//                      move; the plain run is `uart.rx.bare`;
//        uart.edge     the receive edge's latency, the stamped binding:
//                      TCB0 free-running at CLK_PER in capture mode, its
//                      capture fed by the TXD pad's rising edges (EVSYS
//                      channel 4), so the last frame's stop bit begins at
//                      the capture and ends one bit time later (S x BAUD /
//                      64 CLK_PER, exact); the burst is sent by the
//                      resource's polled send() and its last byte is 0x00,
//                      whose stop bit is the only rising edge in it. n=1:
//                      wall = TCB0 where rxc() returned true (the edge the
//                      glue posts on) minus the stop bit's end - negative
//                      when the edge comes first, RXCIF rising in the
//                      middle of the stop bit (usart.md); n=16: the same
//                      for the burst's LAST byte (the edge itself came with
//                      the first). The pin event and the capture's
//                      synchronizer add two or three cycles.
//      The vendor's column (`uart.vendor`): no vendor library on this
//      family; the data sheet's sequence (27.3.2.4: RXDATAH, then RXDATAL)
//      as a bare RXC handler into a ring of its own runs in a scratch
//      program beside this one, the number in docs/avrdx/usart.md.
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
//   spi.*   SCK / 8 bytes a second: one frame of eight bits a byte, the
//           division exact at every rate (docs/avrdx/spi.md).
//   uart.*  the rate the generator really produces (the BAUD register
//           read back, usart_actual_baud) over the ten bits of an 8N1
//           frame: 11 524, 100 000, 200 000 and 300 000 B/s.
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
#include "avrdx/pin.hpp"
#include "avrdx/platform.hpp"
#include "avrdx/spi.hpp"
#include "avrdx/tca.hpp"
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
using ChCarry = EventChannel<2>;
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
IsrMeter<Ruler, Idle> spi_meter;
IsrMeter<Ruler, Idle> u4_meter;
IsrMeter<Ruler, Idle> empty_meter;

/// The counters, read quiescent and under the mask (the file header).
BenchCounters counters() {
    P::CriticalSection cs;
    return bench_counters<Idle>(rxc_meter, dre_meter, tick_meter, spi_meter, u4_meter);
}

TestBench<Serial> bench;

// ---- the wires (the file header) ---------------------------------------------

constexpr uint32_t print_wire_bps = console_baud / frame_bits;
constexpr uint32_t memcpy_wire_bps = SysClock::hz / 3u;
constexpr uint32_t memset_wire_bps = SysClock::hz;

constexpr std::array<uint16_t, 4> sizes{1u, 16u, 256u, 4096u};
constexpr uint16_t max_size = 4096u;

// ---- the console's drain ------------------------------------------------------

/// Let the console fall silent - the ring empty, the last stop bit out -
/// so its interrupt lands in nothing measured next: tx_idle() is the
/// wire's (avrdx/usart.hpp).
void drain() {
    while (!Serial::tx_idle()) {
    }
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
    return bench_counters<Idle>(rxc_meter, dre_meter, tick_meter, spi_meter, u4_meter,
                                empty_meter);
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
        const char* text = payload.data() + (max_size - n);
        const Share dre0 = share(dre_meter);
        const Share tick0 = share(tick_meter);
        const BenchCounters c0 = counters();
        sw.start();
        print(serial, text);
        while (!Serial::tx_idle()) {
        }
        const uint32_t wall = sw.elapsed();
        const BenchSample s = bench_sample(wall, c0, counters());
        const Share dre1 = share(dre_meter);
        const Share tick1 = share(tick_meter);
        print(serial, crlf, "  dre: irq=", dre1.irq - dre0.irq, " isr=", dre1.cycles - dre0.cycles,
              "  tick: irq=", tick1.irq - tick0.irq, " isr=", tick1.cycles - tick0.cycles, crlf);
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

// =============================================================================
// e - the SPI host: spi.poll, spi.poll.tx, spi.pump, spi.req
// =============================================================================
namespace de {

using SpiHw = SpiHost<0, SpiRoute::alt1>;   // PE0 MOSI, PE1 MISO, PE2 SCK
using Cs = Pin<'A', 2>;                     // the select, a real pad
using Dc = Pin<'A', 3>;                     // the D/C, a real pad
using S0 = SpiHw::Resource;

/// The pump's completion edge, written by the vector.
volatile bool spi_done = false;

/// The two rates letter e runs at: the family's second fastest and the
/// one the DCS link reads at.
constexpr SpiClock rates[] = {SpiClock::div4, SpiClock::div16};
constexpr uint16_t lengths[] = {16u, 256u};

constexpr uint32_t sck_hz(SpiClock c) { return spi_sck_hz(SysClock::hz, c); }
constexpr uint32_t wire_bps(SpiClock c) { return sck_hz(c) / 8u; }
/// CLK_PER cycles the wire takes for n bytes at a rate.
constexpr uint32_t wire_cycles(SpiClock c, uint32_t n) {
    return n * 8u * static_cast<uint32_t>(spi_division(c));
}

/// A data-phase request over the two static buffers: tx always lent,
/// rx lent or null, no command, no select, no D/C.
SpiHw::Request data_request(SpiClock c, uint16_t n, bool duplex, bool polled) {
    SpiHw::Request r{};
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(mem_src));
    if (duplex) {
        r.rx = lend<Lease::reply>(static_cast<uint8_t*>(mem_dst));
    }
    r.len = n;
    r.clock = c;
    r.mode = SpiMode::mode0;
    r.polled = polled;
    return r;
}

/// A pumped request: start() launches it, the thread idles until the
/// vector's edge; bounded at 20 ms on the ruler so a lost completion
/// ends the run instead of the bench. `ok` is false if any run timed
/// out. The best of 8 by wall.
template <typename Launch>
BenchSample best_pump(Launch launch, bool& ok) {
    BenchSample best{};
    Stopwatch<Ruler> sw;
    ok = true;
    constexpr uint32_t bound = Ruler::hz() / 50u;
    for (uint8_t run = 0; run < 8u; ++run) {
        spi_done = false;
        const BenchCounters c0 = counters();
        sw.start();
        const bool sync = launch();
        bool done = sync;
        while (!done) {
            cli();
            if (spi_done) {
                sei();
                done = true;
                break;
            }
            if (sw.elapsed() >= bound) {
                sei();
                break;
            }
            Idle::idle();
        }
        const uint32_t wall = sw.elapsed();
        const BenchSample s = bench_sample(wall, c0, counters());
        ok = ok && done;
        if (run == 0u || s.wall < best.wall) {
            best = s;
        }
    }
    return best;
}

/// spi.poll and spi.poll.tx: the polled loop, both shapes.
void poll() {
    for (const bool duplex : {true, false}) {
        for (const SpiClock c : rates) {
            for (const uint16_t n : lengths) {
                drain();
                const SpiHw::Request r = data_request(c, n, duplex, true);
                const BenchSample s = best_of_8([&r] { (void)SpiHw::start(r); });
                bench_line(serial, duplex ? "spi.poll" : "spi.poll.tx", n, s, Ruler::hz(),
                           wire_bps(c));
                print(serial, "  CLK_PER/", spi_division(c), ", wire ", wire_cycles(c, n),
                      " cycles: above the wire ", s.wall - wire_cycles(c, n), crlf);
            }
        }
    }
}

/// spi.pump: the same full-duplex requests on the interrupt pump.
bool pump() {
    bool all = true;
    for (const SpiClock c : rates) {
        for (const uint16_t n : lengths) {
            drain();
            S0::clear_overflow();
            const SpiHw::Request r = data_request(c, n, true, false);
            bool ok = false;
            const BenchSample s = best_pump([&r] { return SpiHw::start(r); }, ok);
            const bool overflow = S0::overflow_flag();
            bench_line(serial, "spi.pump", n, s, Ruler::hz(), wire_bps(c));
            print(serial, "  CLK_PER/", spi_division(c), ": per byte irq=",
                  (s.irq + n / 2u) / n, " isr=", (s.isr + n / 2u) / n, ", completions ",
                  ok ? "all" : "MISSING", ", BUFOVF after the run ", overflow ? "1" : "0", crlf);
            all = all && ok && !overflow;
        }
    }
    return all;
}

/// spi.req: the price of a DCS command - one command byte, a data
/// phase of 0, 2 and 15 bytes, the select and the D/C on real pads.
void req() {
    static const uint8_t command = 0x2C;   // RAMWR, the panel's memory write
    constexpr uint16_t data_lengths[] = {0u, 2u, 15u};
    for (const uint16_t len : data_lengths) {
        drain();
        SpiHw::Request r{};
        r.cs = Cs::ref();
        r.dc = Dc::ref();
        r.cmd = lend<Lease::reply>(&command);
        r.cmd_len = 1;
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(mem_src));
        r.len = len;
        r.clock = SpiClock::div16;
        r.mode = SpiMode::mode0;
        r.polled = true;
        const uint16_t n = static_cast<uint16_t>(1u + len);
        const BenchSample s = best_of_8([&r] { (void)SpiHw::start(r); });
        bench_line(serial, "spi.req", n, s, Ruler::hz(), wire_bps(SpiClock::div16));
        print(serial, "  wire ", wire_cycles(SpiClock::div16, n), " cycles: fixed cost ",
              s.wall - wire_cycles(SpiClock::div16, n), " cycles", crlf);
    }
}

}  // namespace de

void te_spi() {
    using namespace de;
    for (uint16_t i = 0; i < 256u; ++i) {
        mem_src[i] = static_cast<uint8_t>(i * 7u + 1u);
    }
    Cs::set();
    Cs::output();
    Dc::output();
    if (!SpiHw::init(clock)) {
        print(serial, "  the SPI host did not come up", crlf);
        bench.verdict("ran", false);
        return;
    }
    de::poll();
    const bool pumped = de::pump();
    de::req();
    SpiHw::release();
    Cs::input();
    Dc::input();
    bench.verdict("every pumped request completed and left BUFOVF clear", pumped);
}

// =============================================================================
// u - the UART transport: uart.tx, uart.rx, uart.edge
// =============================================================================
namespace du {

using Loop = Uart<4, Route::def, 256u, 256u>;   // TXD PE0, RXD PE1 (unused: LBME)
using U4 = Loop::Resource;
using Stamp = Tcb<0>;                           // CLK_PER, capture on the pad's rising edge
using ChPad = EventChannel<4>;
using Pad = Pin<'E', 0>;
using Gen = Tca<0>;                             // uart.rx's sender: WO0 on PE0

/// The binding GPR0 selects (the file header): bit 7 marks an
/// instrumented binding, so the plain one is the dispatch's first exit.
constexpr uint8_t plain = 0x00;
constexpr uint8_t metered = 0x81;
constexpr uint8_t stamped = 0x82;
constexpr uint8_t counted = 0x84;

volatile uint16_t rxc_entries = 0;
volatile uint16_t dre_entries = 0;
volatile uint16_t edge_at = 0;       ///< TCB0 where rxc() returned true
volatile uint16_t last_at = 0;       ///< TCB0 at the last stamped entry
volatile bool edge_seen = false;

struct LoopRate {
    uint32_t baud;
    bool clk2x;
};
constexpr LoopRate loop_rates[] = {
    {115'200u, false}, {1'000'000u, false}, {2'000'000u, true}, {3'000'000u, true}};

/// The loop at a rate: the task's init (normal speed), loop-back, and in
/// double speed RXMODE and the divisor rewritten under it.
void loop_at(const LoopRate& r) {
    Loop::init(clock, r.clk2x ? 115'200u : r.baud);
    U4::loop_back(true);
    if (r.clk2x) {
        U4::rx_mode(UsartRxMode::clk2x);
        U4::baud_reg(usart_baud_reg(SysClock::hz, r.baud, 8u));
    }
}

uint32_t actual_baud() { return U4::actual_baud(SysClock::hz); }
uint32_t wire_bps() { return actual_baud() / frame_bits; }
/// One bit in CLK_PER cycles, in 1/64ths: S x BAUD (27.3.2.2.1).
uint32_t bit64() { return static_cast<uint32_t>(U4::samples()) * U4::baud_reg(); }

/// The receive ring emptied (nothing pending is this run's).
void drain_rx() {
    for (;;) {
        const auto run = Loop::read_span();
        if (run.empty()) {
            return;
        }
        Loop::consume(static_cast<uint32_t>(run.size()));
    }
}

/// TXCIF, bounded by a few frames at the slowest rate.
bool wait_txc() {
    for (uint16_t i = 0; i < 4000u; ++i) {
        if (U4::txc_flag()) {
            return true;
        }
    }
    return false;
}

constexpr uint16_t tx_sizes[] = {256u, 4096u};
constexpr uint16_t rx_sizes[] = {16u, 256u};
constexpr uint8_t modes[] = {metered, counted, plain};

/// The line, or the counted run's note: the binding decides.
/// `moved` is the bytes the run carried (a receive run may take a few
/// more than n before the thread sees n): the per-byte figures divide by it.
void print_run(const char* op, const char* bare_op, uint8_t mode, uint16_t n, uint16_t moved,
               const BenchSample& s, const Share& m0, const Share& m1, uint16_t entries) {
    if (mode == metered) {
        bench_line(serial, op, n, s, Ruler::hz(), wire_bps());
        print(serial, "  ", actual_baud(), " baud, metered: per byte irq=",
              (m1.irq - m0.irq + moved / 2u) / moved, " isr=",
              (m1.cycles - m0.cycles + moved / 2u) / moved);
    } else if (mode == counted) {
        print(serial, "  ", actual_baud(), " baud, counted: ", entries, " entries for ", moved,
              " bytes, wall ", s.wall);
    } else {
        bench_line(serial, bare_op, n, s, Ruler::hz(), wire_bps());
        print(serial, "  ", actual_baud(), " baud, plain");
    }
}

void tx_lines() {
    U4::enable_rx(false);   // the line is the transmitter's alone
    Stopwatch<Ruler> sw;
    for (const uint16_t n : tx_sizes) {
        for (const uint8_t mode : modes) {
            drain();
            GPR.GPR0 = mode;
            const BenchCounters c0 = counters();
            const uint16_t e0 = dre_entries;
            const Share m0 = share(u4_meter);
            sw.start();
            uint16_t sent = static_cast<uint16_t>(Loop::write_bulk({mem_src, n}));
            while (sent < n) {
                sent = static_cast<uint16_t>(
                    sent + Loop::write_bulk({mem_src + sent, static_cast<size_t>(n - sent)}));
            }
            while (!Loop::tx_idle()) {
            }
            const uint32_t wall = sw.elapsed();
            const BenchSample s = bench_sample(wall, c0, counters());
            const Share m1 = share(u4_meter);
            const uint16_t e = static_cast<uint16_t>(dre_entries - e0);
            GPR.GPR0 = plain;
            print_run("uart.tx", "uart.tx.bare", mode, n, n, s, m0, m1, e);
            print(serial, crlf);
        }
    }
    U4::enable_rx(true);
}

/// uart.rx: TCA0 sends, the interrupt receiver takes. The transmitter is
/// off, so PORT owns PE0 and TCA0's WO0 drives it: single slope, PER one
/// frame, CMP0 at nine bits - high from BOTTOM to the match (eight data
/// ones and the stop bit), low from the match to TOP (the start bit):
/// a stream of 0xFF frames back to back at the generator's own rate,
/// with no CPU in it. The count starts one cycle short of the match, so
/// the first start bit begins as the timer is enabled. At 115200 the
/// frame is 2082.5 CLK_PER and PER takes 2082: 0.02 per cent fast.
void rx_lines() {
    U4::enable_tx(false);
    Pad::set();
    Pad::output();
    const uint16_t bit = static_cast<uint16_t>(bit64() / 64u);
    const uint16_t frame = static_cast<uint16_t>(bit64() * frame_bits / 64u);
    for (const uint16_t n : rx_sizes) {
        for (const uint8_t mode : modes) {
            drain();
            drain_rx();
            Loop::clear_errors();
            (void)Gen::init({.mode = TcaMode::single_slope, .clock = TcaClock::div1,
                             .period = static_cast<uint16_t>(frame - 1u),
                             .compare0 = static_cast<uint16_t>(9u * bit), .route = 'E'});
            Gen::disable();
            Gen::output_value<0>(true);
            Gen::count(static_cast<uint16_t>(9u * bit - 1u));
            Gen::output<0>(true);
            GPR.GPR0 = mode;
            const BenchCounters c0 = counters();
            const uint16_t e0 = rxc_entries;
            const Share m0 = share(u4_meter);
            Stopwatch<Ruler> sw;
            uint16_t got = 0;
            uint16_t sum = 0;
            uint32_t turns = 0;
            sw.start();
            Gen::enable();
            while (got < n && turns < 100'000u) {
                ++turns;
                const auto run = Loop::read_span();
                if (!run.empty()) {
                    for (const uint8_t b : run) {
                        sum = static_cast<uint16_t>(sum + b);
                    }
                    got = static_cast<uint16_t>(got + run.size());
                    Loop::consume(static_cast<uint32_t>(run.size()));
                }
            }
            const uint32_t wall = sw.elapsed();
            Gen::disable();   // the stream stops here: nothing after the wall is counted
            const uint16_t e = static_cast<uint16_t>(rxc_entries - e0);
            const uint8_t fe = Loop::frame_errors();
            const uint8_t ring = Loop::rx_overruns();
            const uint8_t hw = Loop::hw_overruns();
            const BenchSample s = bench_sample(wall, c0, counters());
            const Share m1 = share(u4_meter);
            Gen::output_value<0>(true);   // the line parked high; a cut frame is drained below
            GPR.GPR0 = plain;
            print_run("uart.rx", "uart.rx.bare", mode, n, got, s, m0, m1, e);
            print(serial, ", delivered ", got, '/', n,
                  sum == static_cast<uint16_t>(0xFFu * got) ? " all 0xFF" : " NOT 0xFF",
                  ", errors fe=", fe, " ring=", ring, " hw=", hw, crlf);
            delay_us(clock, 200);
        }
    }
    Gen::output<0>(false);
    Gen::reset();
    Gen::route('A');
    drain_rx();
    U4::enable_tx(true);
}

/// uart.edge: the stamped binding, the pad's capture, a burst ending in 0x00.
void edge_lines() {
    for (const uint16_t n : {static_cast<uint16_t>(1u), static_cast<uint16_t>(16u)}) {
        drain();
        drain_rx();
        edge_seen = false;
        const uint16_t e0 = rxc_entries;
        GPR.GPR0 = stamped;
        for (uint16_t i = 0; i + 1u < n; ++i) {
            (void)U4::send(0x55u);
        }
        (void)U4::send(0x00u);
        for (uint32_t i = 0; i < 100'000u && static_cast<uint16_t>(rxc_entries - e0) < n; ++i) {
        }
        (void)wait_txc();
        GPR.GPR0 = plain;
        const uint16_t cap = Stamp::capture();
        const uint16_t entries = static_cast<uint16_t>(rxc_entries - e0);
        const uint16_t bit = static_cast<uint16_t>((bit64() + 32u) / 64u);
        const uint16_t at = n == 1u ? edge_at : last_at;
        const int32_t latency = static_cast<int32_t>(static_cast<int16_t>(at - cap)) - bit;
        print(serial, "bench uart.edge n=", n, " wall=", latency, " busy=0 irq=", entries,
              " isr=0 rate=- wire=", wire_bps(), " x=-", crlf);
        print(serial, "  ", actual_baud(), " baud: bit ", bit, " cycles, the stop bit began at ",
              cap, ", ", n == 1u ? "the edge" : "the last byte", " left the handler at ", at,
              edge_seen ? "" : " (NO EDGE)", crlf);
        drain_rx();
    }
}

}  // namespace du

void tu_uart() {
    using namespace du;
    for (uint16_t i = 0; i < max_size; ++i) {
        mem_src[i] = static_cast<uint8_t>(i * 13u + 7u);
    }
    GPR.GPR0 = plain;
    Stamp::init({.mode = TcbMode::capture, .clock = TcbClock::div1, .event_input = true});
    ChPad::source(EvPin<Pad>{});
    Stamp::capture_on(ChPad{});
    for (const LoopRate& r : loop_rates) {
        loop_at(r);
        tx_lines();
        rx_lines();
        edge_lines();
    }
    Loop::release();
    Stamp::disable();
    ChPad::off();
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
// Letter u's USART4 bindings (the file header): three handlers compiled
// whole, the vector a naked jump on GPR0 - SBIC tests an I/O bit without
// touching SREG, so the dispatch needs no prologue of its own.
#pragma GCC diagnostic push
#if !defined(__clang__)   // the language server's parser knows no such group
#pragma GCC diagnostic ignored "-Wmisspelled-isr"
#endif
extern "C" {
[[gnu::signal, gnu::used]] void bench_u4_rxc_metered() {
    u4_meter.enter();
    (void)du::Loop::rxc();
    u4_meter.leave();
}
[[gnu::signal, gnu::used]] void bench_u4_rxc_stamped() {
    const bool edge = du::Loop::rxc();
    const uint16_t t = du::Stamp::count();
    if (edge) {
        du::edge_at = t;
        du::edge_seen = true;
    }
    du::last_at = t;
    du::rxc_entries = static_cast<uint16_t>(du::rxc_entries + 1u);
}
[[gnu::signal, gnu::used]] void bench_u4_rxc_counted() {
    du::rxc_entries = static_cast<uint16_t>(du::rxc_entries + 1u);
    (void)du::Loop::rxc();
}
[[gnu::signal, gnu::used]] void bench_u4_rxc_plain() { (void)du::Loop::rxc(); }
[[gnu::signal, gnu::used]] void bench_u4_dre_plain() { du::Loop::dre(); }
[[gnu::signal, gnu::used]] void bench_u4_dre_metered() {
    u4_meter.enter();
    du::Loop::dre();
    u4_meter.leave();
}
[[gnu::signal, gnu::used]] void bench_u4_dre_counted() {
    du::dre_entries = static_cast<uint16_t>(du::dre_entries + 1u);
    du::Loop::dre();
}
}
#pragma GCC diagnostic pop
ISR(USART4_RXC_vect, ISR_NAKED) {
    asm volatile("sbis %[gpr], 7\n\t"
                 "jmp bench_u4_rxc_plain\n\t"
                 "sbic %[gpr], 0\n\t"
                 "jmp bench_u4_rxc_metered\n\t"
                 "sbic %[gpr], 1\n\t"
                 "jmp bench_u4_rxc_stamped\n\t"
                 "jmp bench_u4_rxc_counted\n\t" ::[gpr] "I"(_SFR_IO_ADDR(GPR_GPR0)));
}
ISR(USART4_DRE_vect, ISR_NAKED) {
    asm volatile("sbis %[gpr], 7\n\t"
                 "jmp bench_u4_dre_plain\n\t"
                 "sbic %[gpr], 0\n\t"
                 "jmp bench_u4_dre_metered\n\t"
                 "jmp bench_u4_dre_counted\n\t" ::[gpr] "I"(_SFR_IO_ADDR(GPR_GPR0)));
}
ISR(SPI0_INT_vect) {
    spi_meter.enter();
    if (de::SpiHw::isr()) {
        de::spi_done = true;
    }
    spi_meter.leave();
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
    bench.letter('e', "the SPI host: spi.poll, spi.poll.tx, spi.pump, spi.req", te_spi);
    bench.letter('u', "the UART transport on USART4's loop: uart.tx, uart.rx, uart.edge", tu_uart);

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
