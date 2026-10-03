// bench_vx03 - the benchmark skeleton on the CH32V203 and the CH32V303
// (QingKe V4B and V4F, RV32IMAC and RV32IMAFC): the instrument's
// self-check and cost, the runtime's copy and fill against the core's
// floor, a print through the console transport against the wire, the
// tick's floor, and the DMA engines - one line per operation and size, in
// util/bench.hpp's grammar (docs/design/benchmark.md).
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
// ON THE 32 KB TIER the app is two images (the groups line below): the
// skeleton's four letters in one, the engines and the SPI host in the
// other - whole everywhere else.
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
// THE METERS. One IsrMeter a vector, enter() the handler's first
// statement and leave() its last, the body between them as the suite
// binds it - two for letters r, m, p and t:
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
// and letter d's five, each on the full Ruler: dma1_channel1_handler (the
// copy engine), dma1_channel7_handler (TIM4's update, the paced block),
// dma1_channel2_handler and dma1_channel3_handler (SPI1's receive and
// transmit engines - the second armed for a transfer error alone), and
// spi1_handler (the pump, which an engined write never enters). SPI1's
// pads are NSS PA4, SCK PA5, MISO PA6 and MOSI PA7, nothing wired to them.
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
//   d  THE DMA ENGINES (ch32vx03/dma.hpp), four operations, each the best
//      of 4 from a fresh tick, the counters those of the four DMA vectors
//      and SPI1's (not the tick's). NOTHING IDLES IN THIS LETTER: a running
//      channel keeps the core awake on this family (bus_activity.hpp), so
//      the core spins on the completion and busy is wall by construction;
//      what the operation costs the CPU is the launch and the isr, printed
//      on the note line before each bench line.
//        copy, fill  DmaCopyEngine on DMA1's channel 1, 16, 256 and 4096
//                    bytes handed over as words, from start() to the
//                    completion's interrupt; the note line gives the
//                    launch alone (copy() called to copy() returned - at
//                    16 bytes the block completes inside it, so the
//                    completion's handler is in it too) and the fixed
//                    cost: wall less the DMA's own time, measured as the
//                    difference of the two larger sizes over their words.
//                    Then the data judged, and a word copy, a byte copy and
//                    a half-word fill back to back judged byte for byte -
//                    each block starting with a width the last had not.
//        paced       256 words from a table into ONE cell on TIM4's update
//                    request (DMA1's channel 7), one every 1440 cycles
//                    (100 kHz), the completion's interrupt the edge; then
//                    the same run WATCHED under the mask, the core stamping
//                    each fall of the count on the STK's position (one
//                    load) - the spread of the intervals is the jitter as
//                    the core sees it, the watching loop's own turn
//                    included.
//        spi.dma     SPI1's host on its two engines, a write of 16 and 256
//                    frames of 8 and of 16 bits at SCK /4 (36 MHz) and /16
//                    (9 MHz), MISO floating - no strap on this desk, so the
//                    time is the wire's and the received data is not
//                    judged here (test_vx03_spi's letters c and d judge it
//                    on the strap and against the peer; the transmitted is,
//                    below). Each point warmed by a ONE-frame
//                    request first. The note line: the launch (start()
//                    called to start() returned) and the fixed cost, wall
//                    less the frames' own wire time.
//      Then the TRANSMIT DATA JUDGED WITH NO WIRE: 256 frames of 8 and of
//      16 bits sent with SPI1's CRC unit on, its TCRCR - the CRC of every
//      frame the shift register sent - against a bitwise loop over the
//      same buffer: the beat and the byte order inside a 16-bit frame are
//      what that judges. The last line: the bus masters still counted
//      after the SPI requests - zero when every engine stopped its channel.
//   e  THE SPI HOST ABOVE THE WIRE (ch32vx03/spi.hpp's SpiHost with NO
//      engines, on SPI1's same pads, MISO floating; the counters are
//      SPI1's vector's alone, each run from a fresh tick), in two SHAPES
//      the host tells apart - a WRITE (no in buffer: the display's pixel
//      path, every command phase) and a RECEIVE (an in buffer) - and two
//      widths:
//        spi.poll       a POLLED WRITE of 16 and 256 8-bit frames at SCK /4
//                       (36 MHz) and /16 (9 MHz): wall against the frames'
//                       wire time, the note line giving the fixed cost
//                       (wall less the wire) and the cycles a frame; the
//                       overrun flag read after every run. Nothing
//                       interrupts but the tick: busy is wall.
//        spi.poll16     the same in 16-bit frames
//        spi.poll.rx    the polled RECEIVE, 8-bit, the frames landing in
//        spi.poll16.rx  a buffer; and in 16-bit frames
//        spi.pump,      the same four on the INTERRUPT pump: irq and isr
//        spi.pump16,    are the vector's, per frame on the note line, the
//        spi.pump.rx,   core spinning on the completion (busy is wall
//        spi.pump16.rx  again); the overrun flag read after every run
//        spi.req        THE PRICE OF A DCS COMMAND: a polled request of 1, 3
//                       and 16 bytes (cmd_len 1, len 0, 2 and 15 - the D/C
//                       scripted low then high, no in buffer) at /16, the
//                       select on PB0 and the D/C on PB1, two pads nothing
//                       on this board uses; the note line: wall less the
//                       wire's cycles = the fixed cost of the round, the
//                       best of 4.
//        spi.ahead      THE OVERRUN ORACLE for the write-ahead's threshold:
//                       the vendor's own two-frames-in-flight loop (the
//                       EVT's 2Lines_FullDuplex shape - the transmit
//                       buffer kept full on TXE, the receive buffer drained
//                       on RXNE) run on the RESOURCE at every BR code and
//                       both widths, 1024 frames a run, eight runs a point,
//                       each run under a console print in flight (the
//                       USART's interrupts, some 200 cycles a trip in this
//                       image with the stamps) and the tick: the runs whose
//                       OVR stood afterwards, or that came up a frame
//                       short, are counted. A frame shorter than the
//                       longest handler is where a second frame in flight
//                       loses one; this measures where that is on this
//                       image.
//        spi.live       the HOST under the same load: both shapes, the
//                       pump and the polled loop, every code and width,
//                       1024 frames a run, eight runs: the overruns and the
//                       statuses other than spi_ok counted - what the
//                       host's own threshold makes of it (a write tolerates
//                       the overrun and finishes; a receive must never see
//                       one).
//
// build: boards = v203c6,v203c8,v303vc
// build: groups = rmpt,de
// build: monitor_speed = 115200

#include <stdint.h>
#include <string.h>

#include <cstddef>
#include <span>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/delay.hpp"
#include "ch32vx03/device.hpp"
#include "ch32vx03/dma.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/spi.hpp"
#include "ch32vx03/ticker.hpp"
#include "ch32vx03/tim.hpp"
#include "ch32vx03/usart.hpp"
#include "kernel/borrowed.hpp"
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

// ---------------------------------------------------------------------------
// d - the DMA engines
// ---------------------------------------------------------------------------
Meter copy_meter;    // dma1_channel1_handler: the copy engine
Meter paced_meter;   // dma1_channel7_handler: TIM4's update
Meter spi_rx_meter;  // dma1_channel2_handler: SPI1's receive engine
Meter spi_tx_meter;  // dma1_channel3_handler: SPI1's transmit engine (errors alone)
Meter spi_meter;     // spi1_handler: the pump

/// Letter d's counters: the DMA vectors and SPI1's, not the tick (each
/// short operation starts on a fresh tick, and the paced one says how many
/// ticks its wall holds). Nothing idles in this letter: a running channel
/// keeps the core awake on this family (ch32vx03/bus_activity.hpp), so the
/// core spins on the completion and busy is wall by construction.
BenchCounters dma_counters() {
    return bench_counters<Plat>(copy_meter, paced_meter, spi_rx_meter, spi_tx_meter, spi_meter);
}

// ---- copy and fill: DmaCopyEngine on DMA1's first channel -----------------
using Copier = DmaCopyEngine<1, 1>;
volatile bool copy_done = false;
uint32_t fill_cell = 0;

void copy_arm() { Copier::arm(DmaPriority::low, true); }

/// One block of `bytes`, handed over as words: the beat is the element
/// type, and the buffers and the sizes here are all whole words.
bool copy_start(void* dst, const void* src, uint32_t bytes, bool fill) {
    copy_done = false;
    if (fill) {
        return Copier::fill(static_cast<uint32_t*>(dst), &fill_cell, bytes / 4u);
    }
    return Copier::copy(static_cast<uint32_t*>(dst), static_cast<const uint32_t*>(src),
                        bytes / 4u);
}

[[gnu::always_inline]] inline void copy_vector() {
    if (Copier::service() != 0u) {
        copy_done = true;
    }
}

// ---- paced: TIM4's update request on DMA1's channel 7 ----------------------
using PacedTx = DmaTxEngine<1, DmaRequestOf<DmaRequest::tim4_up>::channel, uint32_t>;
using PacedCh = DmaChannel<1, DmaRequestOf<DmaRequest::tim4_up>::channel>;
using Pacer = Tim<4>;
volatile bool paced_done = false;
volatile uint32_t paced_cell = 0;

void paced_arm() { (void)PacedTx::arm(&paced_cell, DmaPriority::high); }

bool paced_start(const uint32_t* table, uint16_t n) {
    paced_done = false;
    return PacedTx::start(std::span<const uint32_t>(table, n));
}

[[gnu::always_inline]] inline void paced_vector() {
    const uint8_t f = PacedTx::service();
    if ((f & PacedTx::flag_error) != 0u) {
        (void)PacedTx::abandon();
        paced_done = true;
    } else if ((f & PacedTx::flag_complete) != 0u) {
        (void)PacedTx::complete();
        paced_done = true;
    }
}

// ---- SPI1's host on its two engines, MISO floating -------------------------
constexpr SpiPins spi_pins = spi_pins_for(1, 0);   // NSS PA4, SCK PA5, MISO PA6, MOSI PA7
using SpiDma = SpiHost<1, spi_pins, DmaTxEngine<1, Spi<1>::dma_tx_channel>,
                       DmaRxEngine<1, Spi<1>::dma_rx_channel>>;
volatile bool spi_done = false;
bool spi_up = false;

[[gnu::always_inline]] inline void spi_rx_vector() {
    if (SpiDma::dma_rx_isr()) {
        spi_done = true;
    }
}
[[gnu::always_inline]] inline void spi_tx_vector() {
    if (SpiDma::dma_tx_isr()) {
        spi_done = true;
    }
}

// ---- what letter d prints ---------------------------------------------------
// The DMA's clock is HCLK, and its wire here is the brief's: ONE ITEM A
// CYCLE of that clock, four bytes a cycle on the word beat a copy and a
// fill take. The controller does not come near it - the chapter states no
// item time, and the difference of two sizes below is the measurement.
constexpr uint32_t wire_dma_word = 4u * Ruler::hz();

constexpr uint32_t copy_sizes[3] = {16, 256, 4096};

/// One copy or fill, the best of 4 from a fresh tick: the wall from the
/// start to the completion's flag, and the launch alone (start() called
/// to start() returned) in `launch`.
BenchSample copy_once(uint32_t n, bool fill, uint32_t& launch) {
    BenchSample best{0xFFFF'FFFFu, 0, 0, 0};
    for (uint8_t run = 0; run < 4u; ++run) {
        void* dst = buffer_b;
        const void* src = fill ? static_cast<const void*>(&fill_cell) : buffer_a;
        fresh_tick();
        const BenchCounters before = dma_counters();
        Stopwatch<Ruler> sw;
        sw.start();
        const bool ok = copy_start(dst, src, n, fill);
        const uint32_t l = sw.elapsed();
        uint32_t spins = 1'000'000UL;
        while (ok && !copy_done && spins-- != 0u) {
        }
        const uint32_t wall = sw.elapsed();
        const BenchSample s = bench_sample(wall, before, dma_counters());
        if (ok && copy_done && s.wall < best.wall) {
            best = s;
            launch = l;
        }
    }
    return best;
}

void copy_report(const char* op, bool fill) {
    BenchSample s[3]{};
    uint32_t launch[3]{};
    for (uint8_t i = 0; i < 3u; ++i) {
        s[i] = copy_once(copy_sizes[i], fill, launch[i]);
    }
    // The DMA's own time an item, from the difference of the two larger
    // sizes (word items: 64 and 1024), in hundredths of a cycle.
    const uint32_t per_item_x100 = (s[2].wall - s[1].wall) * 100u / (1024u - 64u);
    for (uint8_t i = 0; i < 3u; ++i) {
        const uint32_t items = copy_sizes[i] / 4u;
        const uint32_t moving = items * per_item_x100 / 100u;
        print(serial, "  ", op, " ", copy_sizes[i], " B = ", items, " words: launch ", launch[i],
              ", fixed ", s[i].wall - moving, " (wall less ", per_item_x100 / 100u, '.',
              static_cast<char>('0' + per_item_x100 / 10u % 10u),
              static_cast<char>('0' + per_item_x100 % 10u), " cycles a word)", crlf);
        bench_line(serial, op, copy_sizes[i], s[i], Ruler::hz(), wire_dma_word);
    }
    // The data, judged once: the last copy or fill is what buffer_b holds.
    bool same = true;
    for (uint32_t k = 0; k < 4096u; ++k) {
        const uint8_t want = fill ? static_cast<uint8_t>(fill_cell >> (8u * (k & 3u))) : buffer_a[k];
        if (buffer_b[k] != want) {
            same = false;
            break;
        }
    }
    print(serial, "  ", op, " 4096 B: ", same ? "every byte as expected" : "A BYTE DIFFERS", crlf);
}

// ---- paced -----------------------------------------------------------------
constexpr uint32_t paced_period = 1440;   // TIM4 at HCLK, an update every 10 us
constexpr uint16_t paced_items = 256;
/// The table is the first kilobyte of buffer_a, written after the copies
/// have judged it: the 32 KB parts have 10 KB of SRAM, and this app
/// builds for them.
uint32_t* const paced_table = reinterpret_cast<uint32_t*>(buffer_a);

void pacer_start() {
    Pacer::init();
    (void)Pacer::configure(TimConfig{.prescaler = 0, .period = paced_period - 1u});
    Pacer::interrupts(tim_ude, true);
    Pacer::enable(true);
}
void pacer_stop() {
    Pacer::enable(false);
    Pacer::interrupts(tim_ude, false);
}

void paced_report() {
    for (uint16_t i = 0; i < paced_items; ++i) {
        paced_table[i] = 0x01010101UL * i;
    }
    paced_arm();

    // The line: 256 words into one cell, the completion's interrupt the
    // edge, the core spinning on it (it cannot sleep under a channel here).
    fresh_tick();
    const BenchCounters before = dma_counters();
    const uint32_t ticks0 = Ticker::ticks();
    Stopwatch<Ruler> sw;
    sw.start();
    const bool ok = paced_start(paced_table, paced_items);
    const uint32_t launch = sw.elapsed();
    pacer_start();
    uint32_t spins = 10'000'000UL;
    while (ok && !paced_done && spins-- != 0u) {
    }
    const uint32_t wall = sw.elapsed();
    const BenchSample s = bench_sample(wall, before, dma_counters());
    pacer_stop();
    const uint32_t ticks = Ticker::ticks() - ticks0;
    const uint32_t due = paced_period * paced_items;
    // Items moved at 4 bytes: the wire is the pace's own, 4 bytes a period.
    const uint32_t wire_pace = 4u * (Ruler::hz() / paced_period);
    print(serial, "  paced: ", paced_items, " words at one a ", paced_period,
          " cycles (TIM4's update, 100 kHz) into one cell: ", ok ? "" : "REFUSED ", "launch ",
          launch, ", wall ", wall, " for ", due, " due, cell=", hex(paced_cell), ", ", ticks,
          " ticks in the wall", crlf);
    bench_line(serial, "paced", 4u * paced_items, s, Ruler::hz(), wire_pace);

    // The pace against the ruler: the core watches the count fall and
    // stamps each step on the STK's position (one load), the intervals'
    // spread being the jitter as the core sees it. Under the mask, so that
    // no handler stretches one interval and shortens the next: the STK
    // counts on regardless, and the completion and the ticks are served
    // when the mask lifts.
    uint32_t* const stamp = reinterpret_cast<uint32_t*>(buffer_b);
    (void)paced_start(paced_table, paced_items);
    fresh_tick();
    uint16_t seen = 0;
    {
        InterruptGuard masked;
        pacer_start();
        uint16_t last = PacedCh::remaining();
        spins = 40'000'000UL;
        while (seen < paced_items && spins-- != 0u) {
            const uint16_t now = PacedCh::remaining();
            if (now != last) {
                stamp[seen++] = TickRuler::now();
                last = now;
            }
        }
    }
    uint32_t spins2 = 1'000'000UL;
    while (!paced_done && spins2-- != 0u) {
    }
    pacer_stop();
    constexpr uint32_t tick_period = Ruler::hz() / 1000u;
    uint32_t lo = 0xFFFF'FFFFu;
    uint32_t hi = 0;
    uint32_t off = 0;
    for (uint16_t k = 1; k < seen; ++k) {
        const uint32_t a = stamp[k - 1u];
        const uint32_t b = stamp[k];
        const uint32_t d = b >= a ? b - a : b + tick_period - a;
        if (d < lo) { lo = d; }
        if (d > hi) { hi = d; }
        if (d > paced_period + 32u || d + 32u < paced_period) { ++off; }
    }
    print(serial, "  paced, watched: ", seen, " steps, interval min ", lo, " max ", hi,
          " cycles (", paced_period, " due), ", off, " beyond +-32",
          crlf);
}

// ---- spi.dma ---------------------------------------------------------------
/// One write of `frames` frames on the engined host, MISO floating: the
/// wall from start() to the completion edge, the launch alone in `launch`.
BenchSample spi_once(uint16_t frames, SpiClock rate, SpiDataSize bits, uint32_t& launch) {
    BenchSample best{0xFFFF'FFFFu, 0, 0, 0};
    for (uint8_t run = 0; run < 4u; ++run) {
        SpiDma::Request r{};
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(buffer_a));
        r.len = frames;
        r.mode = SpiMode::mode0;
        r.clock = rate;
        r.bits = bits;
        r.polled = false;
        spi_done = false;
        fresh_tick();
        const BenchCounters before = dma_counters();
        Stopwatch<Ruler> sw;
        sw.start();
        const bool sync = SpiDma::start(r);
        const uint32_t l = sw.elapsed();
        uint32_t spins = 1'000'000UL;
        while (!sync && !spi_done && spins-- != 0u) {
        }
        const uint32_t wall = sw.elapsed();
        const BenchSample s = bench_sample(wall, before, dma_counters());
        if ((sync || spi_done) && s.wall < best.wall) {
            best = s;
            launch = l;
        }
    }
    return best;
}

void spi_report() {
    if (!spi_up) {
        spi_up = SpiDma::init(clock);
    }
    constexpr SpiClock rates[2] = {SpiClock::div4, SpiClock::div16};
    for (const SpiClock rate : rates) {
        const uint32_t sck = spi_sck_hz(Spi<1>::bus_hz(clock), rate);
        for (const SpiDataSize bits : {SpiDataSize::bits8, SpiDataSize::bits16}) {
            const uint32_t frame_bits = bits == SpiDataSize::bits16 ? 16u : 8u;
            const uint32_t frame_cycles = frame_bits * (Ruler::hz() / sck);
            for (const uint16_t frames : {uint16_t{16}, uint16_t{256}}) {
                uint32_t launch = 0;
                // Warm the configuration (a rate or width change costs a
                // disable and an enable once) with ONE frame - the request
                // that left both DMA requests standing before the engines'
                // rework, and stalled the next - then measure.
                (void)spi_once(1, rate, bits, launch);
                const BenchSample s = spi_once(frames, rate, bits, launch);
                const uint32_t bytes = frames * (frame_bits / 8u);
                const uint32_t wire_cycles = frames * frame_cycles;
                print(serial, "  spi.dma SCK ", sck, " Hz, ", frame_bits, "-bit frames x ", frames,
                      ": launch ", launch, ", fixed ", s.wall - wire_cycles, " (wall less ",
                      wire_cycles, " of wire), ", s.irq, " interrupts", crlf);
                bench_line(serial, "spi.dma", bytes, s, Ruler::hz(), sck / 8u);
            }
        }
    }
}

/// The CRC of `frames` frames of `width` bits, MSB first, no reflection,
/// from zero - the SPI's own CRC unit's arithmetic (RM 20.2.5); a 16-bit
/// frame is two bytes low first, as the Request stores it.
uint16_t crc_frames(uint16_t poly, uint8_t width, const uint8_t* data, uint16_t frames) {
    uint32_t crc = 0;
    const uint32_t top = 1UL << (width - 1u);
    const uint32_t mask = (1UL << width) - 1UL;
    for (uint16_t k = 0; k < frames; ++k) {
        const uint32_t v = width == 16u ? (data[2u * k] | (static_cast<uint32_t>(data[2u * k + 1u]) << 8))
                                        : data[k];
        for (uint8_t b = width; b-- > 0;) {
            const bool in = ((v >> b) & 1u) != 0u;
            const bool out = (crc & top) != 0u;
            crc = (crc << 1) & mask;
            if (in != out) {
                crc ^= poly;
            }
        }
    }
    return static_cast<uint16_t>(crc);
}

/// THE TRANSMIT DATA JUDGED WITH NO WIRE: SPI1's CRC unit accumulates every
/// frame the shift register sends (TCRCR), so a write of `frames` frames on
/// the engines - CRCEN raised with SPE down, the configuration the request
/// wants already applied by a request before it - must leave there the
/// CRC a bitwise loop computes over the same buffer. For 16-bit frames the
/// half-word beat and the byte order inside it are what that judges.
/// WITH CRCEN SET A DMA-FED BLOCK IS FOLLOWED BY THE CRC FRAME ON THE WIRE
/// (measured: RXNE stands after the receive block's completion and BSY is
/// still up at its interrupt - the frame after the last datum), so TCRCR is
/// read only once BSY is down (20.4.7's rule; read earlier it holds a value
/// of the frame in flight) and the received CRC frame is drained after.
bool spi_crc_judge(SpiDataSize bits, uint16_t frames) {
    const bool wide = bits == SpiDataSize::bits16;
    const uint16_t poly = wide ? 0x1021u : 0x0007u;
    SpiDma::Request r{};
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(buffer_a));
    r.len = 1;
    r.clock = SpiClock::div16;
    r.bits = bits;
    spi_done = false;
    if (!SpiDma::start(r)) {
        while (!spi_done) {
        }
    }
    (void)Spi<1>::disable();
    Spi<1>::crc_polynomial(poly);
    Spi<1>::regs().CTLR1 = static_cast<uint16_t>(Spi<1>::regs().CTLR1 | spi_crcen);
    Spi<1>::enable();
    r.len = frames;
    spi_done = false;
    if (!SpiDma::start(r)) {
        uint32_t spins = 1'000'000UL;
        while (!spi_done && spins-- != 0u) {
        }
    }
    for (uint32_t spins = 100'000UL; Spi<1>::busy() && spins != 0u; --spins) {
    }
    const uint16_t got = Spi<1>::tx_crc();
    const bool crc_frame = Spi<1>::rxne();   // the CRC frame the hardware appended
    Spi<1>::flush_rx();
    (void)Spi<1>::disable();
    Spi<1>::regs().CTLR1 = static_cast<uint16_t>(Spi<1>::regs().CTLR1 & ~spi_crcen);
    Spi<1>::enable();
    const uint16_t want = crc_frames(poly, wide ? 16u : 8u, buffer_a, frames);
    print(serial, "  spi.dma ", wide ? 16 : 8, "-bit frames x ", frames, ", the data judged by ",
          "the transmit CRC: TCRCR ", hex(got), ", software ", hex(want), want == got ? "" : " - DIFFER",
          crc_frame ? "; the CRC frame followed the block on the wire" : "; no CRC frame followed", crlf);
    return want == got;
}

/// The width and EN in ONE store (DmaBinding): three blocks back to back,
/// each starting with a width the last did not have - a word copy, a byte
/// copy whose two ends disagree modulo four, a half-word fill at a
/// half-word-aligned destination - and every byte judged.
bool beats_check() {
    for (uint32_t k = 0; k < 64u; ++k) {
        buffer_b[k] = 0;
    }
    static constexpr uint16_t half_cell = 0xBEEF;
    bool ok = Copier::copy(reinterpret_cast<uint32_t*>(buffer_b),
                           reinterpret_cast<const uint32_t*>(buffer_a), 4u);   // words
    while (Copier::busy()) {
    }
    ok = ok && Copier::copy(buffer_b + 17, buffer_a + 2, 15u);   // bytes
    while (Copier::busy()) {
    }
    ok = ok && Copier::fill(reinterpret_cast<uint16_t*>(buffer_b + 34), &half_cell, 7u);
    while (Copier::busy()) {
    }
    for (uint32_t k = 0; k < 16u; ++k) {
        ok = ok && buffer_b[k] == buffer_a[k];
    }
    ok = ok && buffer_b[16] == 0u;
    for (uint32_t k = 0; k < 15u; ++k) {
        ok = ok && buffer_b[17u + k] == buffer_a[2u + k];
    }
    ok = ok && buffer_b[32] == 0u && buffer_b[33] == 0u;
    for (uint32_t k = 0; k < 7u; ++k) {
        ok = ok && buffer_b[34u + 2u * k] == 0xEFu && buffer_b[35u + 2u * k] == 0xBEu;
    }
    ok = ok && buffer_b[48] == 0u;
    return ok;
}

void td_dma() {
    console_drain();
    for (uint32_t k = 0; k < 4096u; ++k) {
        buffer_a[k] = static_cast<uint8_t>(k * 7u + 3u);
    }
    fill_cell = 0xA5A5A5A5UL;
    copy_arm();
    copy_report("copy", false);
    console_drain();
    copy_report("fill", true);
    console_drain();
    print(serial, "  beats: a word copy, a byte copy, a half-word fill back to back: ",
          beats_check() ? "every byte as expected" : "A BYTE DIFFERS", crlf);
    console_drain();
    paced_report();
    console_drain();
    spi_report();
    (void)spi_crc_judge(SpiDataSize::bits8, 256);
    (void)spi_crc_judge(SpiDataSize::bits16, 256);
    print(serial, "  bus masters standing after the SPI requests: ", BusActivity::active(), crlf);
    bench.verdict("ran", true);
}

// ---------------------------------------------------------------------------
// e - the SPI host above the wire
// ---------------------------------------------------------------------------
using SpiPlain = SpiHost<1, spi_pins>;   // no engines: the polled loop and the pump
volatile bool spi_plain_done = false;
/// Which host spi1_handler serves: letter d's engined one or letter e's
/// plain one. A load and a branch before the body, the bench's own.
bool spi_plain_active = false;

/// Letter e's counters: SPI1's vector alone (each run starts on a fresh
/// tick and ends inside it; spi.ahead and spi.live run under the console
/// on purpose and count overruns, not cycles).
BenchCounters spi_counters() { return bench_counters<Plat>(spi_meter); }

constexpr uint32_t frame_bits_of(SpiDataSize bits) {
    return bits == SpiDataSize::bits16 ? 16u : 8u;
}

/// One request on the plain host, MISO floating: the wall from start()
/// to the completion - inside start() when polled, the vector's edge
/// otherwise - the best of 4 from a fresh tick. `ovr` says whether the
/// overrun flag stood after ANY of the runs (cleared each time), `status`
/// is the host's last.
BenchSample spi_plain_once(uint16_t frames, SpiClock rate, SpiDataSize bits, bool polled,
                           bool receive, bool& ovr, uint8_t& status) {
    BenchSample best{0xFFFF'FFFFu, 0, 0, 0};
    for (uint8_t run = 0; run < 4u; ++run) {
        SpiPlain::Request r{};
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(buffer_a));
        if (receive) {
            r.rx = lend<Lease::reply>(static_cast<uint8_t*>(buffer_b));
        }
        r.len = frames;
        r.mode = SpiMode::mode0;
        r.clock = rate;
        r.bits = bits;
        r.polled = polled;
        spi_plain_done = false;
        fresh_tick();
        const BenchCounters before = spi_counters();
        Stopwatch<Ruler> sw;
        sw.start();
        const bool sync = SpiPlain::start(r);
        uint32_t spins = 4'000'000UL;
        while (!sync && !spi_plain_done && spins-- != 0u) {
        }
        const uint32_t wall = sw.elapsed();
        const BenchSample s = bench_sample(wall, before, spi_counters());
        if (Spi<1>::overrun()) {
            ovr = true;
            Spi<1>::clear_overrun();
        }
        status = SpiPlain::status();
        if ((sync || spi_plain_done) && s.wall < best.wall) {
            best = s;
        }
    }
    return best;
}

/// The eight op names: the completion style, the width and the shape.
constexpr const char* spi_op_name(bool polled, bool wide, bool receive) {
    if (polled) {
        return wide ? (receive ? "spi.poll16.rx" : "spi.poll16") : (receive ? "spi.poll.rx" : "spi.poll");
    }
    return wide ? (receive ? "spi.pump16.rx" : "spi.pump16") : (receive ? "spi.pump.rx" : "spi.pump");
}

/// The polled and the pumped lines: both shapes, both widths, 16 and 256
/// frames at /4 and /16, each point warmed by a one-frame request (a
/// rate or width change costs a disable and an enable once, outside the
/// measurement).
void spi_plain_report(bool polled) {
    constexpr SpiClock rates[2] = {SpiClock::div4, SpiClock::div16};
    for (const bool receive : {false, true}) {
        for (const SpiClock rate : rates) {
            const uint32_t sck = spi_sck_hz(Spi<1>::bus_hz(clock), rate);
            for (const SpiDataSize bits : {SpiDataSize::bits8, SpiDataSize::bits16}) {
                const bool wide = bits == SpiDataSize::bits16;
                const char* const op = spi_op_name(polled, wide, receive);
                const uint32_t frame_bits = frame_bits_of(bits);
                const uint32_t frame_cycles = frame_bits * (Ruler::hz() / sck);
                for (const uint16_t frames : {uint16_t{16}, uint16_t{256}}) {
                    bool ovr = false;
                    uint8_t status = 0;
                    (void)spi_plain_once(1, rate, bits, polled, receive, ovr, status);
                    ovr = false;
                    const BenchSample s =
                        spi_plain_once(frames, rate, bits, polled, receive, ovr, status);
                    const uint32_t bytes = frames * (frame_bits / 8u);
                    const uint32_t wire_cycles = frames * frame_cycles;
                    print(serial, "  ", op, " SCK ", sck, " Hz, ", frame_bits, "-bit frames x ", frames,
                          ": fixed ", s.wall - wire_cycles, " (wall less ", wire_cycles, " of wire), ",
                          s.wall / frames, " cycles a frame against the wire's ", frame_cycles, ", ",
                          s.irq, " interrupts");
                    if (!polled && s.irq != 0u) {
                        print(serial, " = ", s.isr / s.irq, " isr each");
                    }
                    print(serial, ", status ", status, ovr ? ", OVERRUN" : ", no overrun", crlf);
                    bench_line(serial, op, bytes, s, Ruler::hz(), sck / 8u);
                }
            }
        }
    }
}

/// spi.req: the price of a DCS command - a polled request of 1, 3 and 16
/// bytes with the D/C scripted (cmd_len 1, then len 0, 2 and 15), the
/// select on PB0 and the D/C on PB1, at /16 in 8-bit frames, the best of
/// 4 from a fresh tick; the fixed cost is the wall less the frames' wire
/// time.
void spi_req_report() {
    using Cs = Pin<'B', 0>;
    using Dc = Pin<'B', 1>;
    Cs::output(true);
    Dc::output(false);
    constexpr SpiClock rate = SpiClock::div16;
    const uint32_t sck = spi_sck_hz(Spi<1>::bus_hz(clock), rate);
    const uint32_t frame_cycles = 8u * (Ruler::hz() / sck);
    static constexpr uint8_t command = 0x2C;
    for (const uint16_t n : {uint16_t{1}, uint16_t{3}, uint16_t{16}}) {
        BenchSample best{0xFFFF'FFFFu, 0, 0, 0};
        bool ovr = false;
        for (uint8_t run = 0; run < 4u; ++run) {
            SpiPlain::Request r{};
            r.cs = Cs::ref();
            r.dc = Dc::ref();
            r.cmd = lend<Lease::reply>(&command);
            r.cmd_len = 1;
            r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(buffer_a));
            r.len = static_cast<uint16_t>(n - 1u);
            r.mode = SpiMode::mode0;
            r.clock = rate;
            r.bits = SpiDataSize::bits8;
            r.polled = true;
            fresh_tick();
            const BenchCounters before = spi_counters();
            Stopwatch<Ruler> sw;
            sw.start();
            const bool sync = SpiPlain::start(r);
            const uint32_t wall = sw.elapsed();
            const BenchSample s = bench_sample(wall, before, spi_counters());
            if (Spi<1>::overrun()) {
                ovr = true;
                Spi<1>::clear_overrun();
            }
            if (sync && s.wall < best.wall) {
                best = s;
            }
        }
        const uint32_t wire_cycles = n * frame_cycles;
        print(serial, "  spi.req ", n, " bytes (cmd 1 + data ", n - 1u, ") at SCK ", sck,
              " Hz: fixed ", best.wall - wire_cycles, " (wall less ", wire_cycles,
              " of wire), status ", SpiPlain::status(), ovr ? ", OVERRUN" : "", crlf);
        bench_line(serial, "spi.req", n, best, Ruler::hz(), sck / 8u);
    }
    Cs::release();
    Dc::release();
}

/// A line the console carries while a run is on the wire: it fits the
/// transport's ring whole, so print() returns at once and the USART's
/// interrupts land in the run that follows.
constexpr Filler<60> live_line = make_filler<60>();

/// The vendor's two-frames-in-flight loop, transcribed from the EVT's
/// 2Lines_FullDuplex example (its main.c: `if TXE: send; if RXNE:
/// receive`, in one loop until both counts are done) on the resource as
/// letter e has it configured; bounded, and the overrun flag read and
/// cleared afterwards. True when the flag stood.
bool ahead_loop_overran(uint16_t frames) {
    auto& regs = Spi<1>::regs();
    uint16_t i = 0;
    uint16_t j = 0;
    uint32_t budget = 8'000'000UL;
    while ((i < frames || j < frames) && budget-- != 0u) {
        if (i < frames && (regs.STATR & spi_txe) != 0u) {
            regs.DATAR = buffer_a[i];
            ++i;
        }
        if (j < frames && (regs.STATR & spi_rxne) != 0u) {
            (void)regs.DATAR;
            ++j;
        }
    }
    const bool ovr = Spi<1>::overrun();
    if (ovr || j < frames) {
        Spi<1>::clear_overrun();
    }
    return ovr || j < frames;
}

/// spi.ahead: the overrun oracle. Every BR code, both widths, 1024 frames
/// a run, eight runs under a console print in flight: how many runs lost
/// a frame to a handler longer than one.
void spi_ahead_report() {
    constexpr uint16_t frames = 1024;
    for (const SpiDataSize bits : {SpiDataSize::bits8, SpiDataSize::bits16}) {
        for (uint8_t code = 0; code < 8u; ++code) {
            const SpiClock rate = static_cast<SpiClock>(code);
            const uint32_t sck = spi_sck_hz(Spi<1>::bus_hz(clock), rate);
            const uint32_t frame_cycles = frame_bits_of(bits) * (Ruler::hz() / sck);
            SpiPlain::prime(SpiMode::mode0, rate, bits);
            uint8_t overran = 0;
            for (uint8_t run = 0; run < 8u; ++run) {
                console_drain();
                print(serial, live_line.text);
                if (ahead_loop_overran(frames)) {
                    ++overran;
                }
            }
            console_drain();
            print(serial, "  spi.ahead /", spi_division(rate), " ", frame_bits_of(bits),
                  "-bit: ", frame_cycles, " cycles a frame, ", overran,
                  " of 8 runs overran under the console's interrupts", crlf);
        }
    }
}

/// spi.live: the host at every code and width under the same load, the
/// pump and the polled loop, 1024 frames a run, eight runs: the overruns
/// and the statuses other than spi_ok counted.
void spi_live_report() {
    constexpr uint16_t frames = 1024;
    for (const bool receive : {false, true}) {
    for (const bool polled : {true, false}) {
        for (const SpiDataSize bits : {SpiDataSize::bits8, SpiDataSize::bits16}) {
            for (uint8_t code = 0; code < 8u; ++code) {
                const SpiClock rate = static_cast<SpiClock>(code);
                const uint32_t sck = spi_sck_hz(Spi<1>::bus_hz(clock), rate);
                const uint32_t frame_cycles = frame_bits_of(bits) * (Ruler::hz() / sck);
                uint8_t overran = 0;
                uint8_t faulted = 0;
                uint8_t hung = 0;
                for (uint8_t run = 0; run < 8u; ++run) {
                    SpiPlain::Request r{};
                    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(buffer_a));
                    if (receive) {
                        r.rx = lend<Lease::reply>(static_cast<uint8_t*>(buffer_b));
                    }
                    r.len = frames;
                    r.mode = SpiMode::mode0;
                    r.clock = rate;
                    r.bits = bits;
                    r.polled = polled;
                    spi_plain_done = false;
                    console_drain();
                    print(serial, live_line.text);
                    const bool sync = SpiPlain::start(r);
                    uint32_t spins = 8'000'000UL;
                    while (!sync && !spi_plain_done && spins-- != 0u) {
                    }
                    if (!sync && !spi_plain_done) {
                        ++hung;
                        (void)SpiPlain::recover();
                    }
                    if (Spi<1>::overrun()) {
                        ++overran;
                        Spi<1>::clear_overrun();
                    }
                    if (SpiPlain::status() != spi_ok) {
                        ++faulted;
                    }
                }
                console_drain();
                print(serial, "  spi.live ", polled ? "polled" : "pump", receive ? " receive" : " write",
                      " /", spi_division(rate), " ", frame_bits_of(bits), "-bit: ", frame_cycles,
                      " cycles a frame, ", overran, " overruns, ", faulted, " statuses not spi_ok, ",
                      hung, " never completed, in 8 runs under the console's interrupts", crlf);
            }
        }
    }
    }
}

void te_spi() {
    console_drain();
    for (uint32_t k = 0; k < 4096u; ++k) {
        buffer_a[k] = static_cast<uint8_t>(k * 7u + 3u);
    }
    spi_plain_active = true;
    const bool up = SpiPlain::init(clock);
    const auto ahead8 = SpiPlain::write_ahead_from(SpiDataSize::bits8);
    const auto ahead16 = SpiPlain::write_ahead_from(SpiDataSize::bits16);
    print(serial, "  SpiHost<1> with no engines: ", up ? "up" : "REFUSED",
          "; a receive runs two frames ahead from /", ahead8 ? spi_division(*ahead8) : 0u,
          " in 8-bit frames and from /", ahead16 ? spi_division(*ahead16) : 0u, " in 16-bit ones", crlf);
    spi_plain_report(true);
    console_drain();
    spi_plain_report(false);
    console_drain();
    spi_req_report();
    console_drain();
    spi_ahead_report();
    spi_live_report();
    spi_plain_active = false;
    spi_up = false;   // letter d brings its own host up again
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

extern "C" BRIO_CH32_INTERRUPT void dma1_channel1_handler() {
    copy_meter.enter();
    copy_vector();
    copy_meter.leave();
}

extern "C" BRIO_CH32_INTERRUPT void dma1_channel7_handler() {
    paced_meter.enter();
    paced_vector();
    paced_meter.leave();
}

extern "C" BRIO_CH32_INTERRUPT void dma1_channel2_handler() {
    spi_rx_meter.enter();
    spi_rx_vector();
    spi_rx_meter.leave();
}

extern "C" BRIO_CH32_INTERRUPT void dma1_channel3_handler() {
    spi_tx_meter.enter();
    spi_tx_vector();
    spi_tx_meter.leave();
}

extern "C" BRIO_CH32_INTERRUPT void spi1_handler() {
    spi_meter.enter();
    if (spi_plain_active) {
        if (SpiPlain::isr()) {
            spi_plain_done = true;
        }
    } else if (SpiDma::isr()) {
        spi_done = true;
    }
    spi_meter.leave();
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
    bench.letter('d', "the DMA engines: copy, fill, a paced block, SPI1's engined write", td_dma);
    bench.letter('e', "the SPI host above the wire: polled, pumped, a request's price, the overrun oracle",
                 te_spi);

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
