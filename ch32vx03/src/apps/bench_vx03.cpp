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
// ON THE 32 KB TIER the app is three images (the groups line below): the
// skeleton's four letters in one, the engines in the second and the SPI
// host in the third. ON THE CH32V203C8 the whole is within a few hundred bytes of the
// part's 60 KB, so it is two images there too (the groups.v203c8 line):
// the skeleton with the engines and the SPI host, then the serial
// transport and the I2C host. Whole on the CH32V303.
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
//                    is what a turn costs besides it. A tick makes ONE
//                    turn: the platform's idle() consumes the event its
//                    wake left standing (ch32vx03/platform.hpp), so wall
//                    is a tick period, and the note line's two counts say
//                    so
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
//        spi.held       THE HOLD-OFF AN IMAGE DECLARES (SpiHost's last
//                       parameter) against the one it has: the default
//                       host (200 cycles, two frames of a receive in
//                       flight from /32) and one declaring 100 (from /16),
//                       a receive of 1024 8-bit frames at /16 - a frame of
//                       128 cycles, shorter than the console's handler -
//                       pumped and polled, eight runs each under the same
//                       load: the overruns and the statuses counted. The
//                       declared-short host is the one that must lose
//                       frames, the honest one none.
//        spi.window     THE PUMP'S OVR WINDOW, swept: a pumped write of 2
//                       and of 3 16-bit frames at /4 and /8 on the plain
//                       host, two in flight, SPI1's line masked at the
//                       PFIC around start() and unmasked after a number of
//                       empty turns that grows by one a run, 0 to 127,
//                       four sweeps - so the handler for frame 0 enters
//                       before frame 1 completes, after it, and ON it,
//                       where the completion falls between the handler's
//                       status read and its DATAR read. The line counts
//                       the runs that completed, those that took fewer
//                       interrupts than frames (frame 1 lost and counted:
//                       with two frames the LAST frame's loss), those that
//                       never completed (recovered after a bound) and
//                       those after which OVR stood; every run must
//                       complete.
//
//   i  THE I2C HOST ABOVE THE WIRE on the self-link (the parts with I2C2;
//      a part without it declines the letter by name, and the 32 KB
//      tier's images leave it out, as they leave u): I2C1 the host on
//      PB6/PB7, I2C2 the target at 0x3A on PB10/PB11 - wired to them on
//      the bench board, a pull-up on each line - SERVED FROM ITS OWN TWO
//      VECTORS (I2cClient<2>: each byte asked for comes from a pattern
//      that restarts at every address match, and the byte its shifter
//      wanted beyond the host's NACK is dropped by flush()). AT 120 MHz,
//      NOT 144: CTLR2.FREQ cannot state the 72 MHz PB1 reaches at 144
//      (RM 19.12.2), and 120 is the fastest HCLK whose PB1 it takes - the
//      letter moves the core there, the console and the tick with it, and
//      back at its end, every cycle it prints a 120 MHz one. Through the
//      engineless host (the pump) and the host with DMA1 channels 6 and 7
//      (the engines), ISR-style, the thread idling until the completion,
//      each line the best of 8 at 100 and 400 kHz (DUTY 2):
//        i2c.write      1, 2, 16 and 255 bytes (and 3, last)
//        i2c.read       1, 2, 16 and 255 bytes, checked against the pattern
//        i2c.wr         one byte written, a repeated START, 1, 2 and 16 read
//        i2c.probe      the address alone, 0x3A (ACK) and 0x42 (NACK)
//      irq and isr are the host's vectors' alone (I2C1's two, the two
//      channels); busy is every meter's, the target's included; the line
//      after each says what the TARGET's vectors took a tenure - the
//      instrument's own share of the wall, the same core serving both
//      ends. With an engine running the core may not sleep (this family's
//      bus matrix serves the core alone in a sleep): busy then reads the
//      idle loop's spin. THE WIRE is the tenure's own time at the SCL
//      period MEASURED on the pad: TIM4's channel 1 is SCL's pad PB6, and
//      in external clock mode 1 its count is SCL's rising edges, read
//      twice inside a 255-byte read over 240 bytes. A START and each
//      repeated START count a period, the address and each byte nine; the
//      STOP is not counted (the engine completes when it REQUESTS it). The
//      line after each says wall minus that wire - THE FIXED COST - and
//      what start() took on a quiet bus. Then, per speed:
//        per byte       the pump's handler cycles and entries per byte
//        i2c.stop       start() called the moment a write, a read and a
//                       register read completed, as an arbiter's dispatch
//                       calls the next tenure: the last STOP's drain
//        the timeline   a write of one byte and a register read of two,
//                       every entry of the host's (H, E) and the target's
//                       (T) vectors stamped from the start() call with
//                       STAR1 as it found it - where the fixed cost goes
//      and the longest single entry of each host vector, watched in a
//      pass of its own.
//
// build: boards = v203c6,v203c8,v303vc
// On the CH32V203C8 letters u and i sit in different images: letter i run
// after letter u in one session stalls at its first tenure (each passes
// alone), not yet explained - benchmark.md lists it.
// build: groups = rmpt,d,e
// build: groups.v203c8 = rmptdeu,i
// build: monitor_speed = 115200

#include <stdint.h>
#include <string.h>

#include <cstddef>
#include <span>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/delay.hpp"
#include "ch32vx03/device.hpp"
#include "ch32vx03/dma.hpp"
#include "ch32vx03/i2c.hpp"
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
/// The pacer RELEASED, not only stopped: an update request it raised is
/// held through CEN and UDE cleared until the channel acknowledges it or
/// the timer is reset (docs/ch32vx03/dma.md, "A released requester"), and
/// channel 7 is I2C1's receive channel and USART2's transmit one too.
/// Tim::release() pulses the reset before the gate; pacer_start()'s init()
/// opens it again.
void pacer_stop() { Pacer::release(); }

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
/// spi.held's second host: the same, DECLARING a hold-off of 100 cycles -
/// shorter than this image's console handler with its stamps, about 200.
using SpiShort = SpiHost<1, spi_pins, NoDmaEngine, NoDmaEngine, 100>;
volatile bool spi_plain_done = false;
/// Which host spi1_handler serves: letter e's plain one (zero, so the
/// pump's path is a load and one branch before the body, the bench's
/// own), letter d's engined one, or spi.held's short one. VOLATILE: the
/// vector is the only reader, and gcc, seeing none between two stores of
/// the main path, drops the first as dead - spi.held's store of the
/// short host, read in the listing - and that host's interrupts would go
/// to the plain one.
constexpr uint8_t spi1_plain = 0;
constexpr uint8_t spi1_dma = 1;
constexpr uint8_t spi1_short = 2;
volatile uint8_t spi1_serves = spi1_dma;

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

/// spi.held: THE HOLD-OFF AN IMAGE DECLARES against the one it has. A
/// receive of 1024 8-bit frames at /16 - a frame of 128 cycles, between
/// the two hosts' thresholds - eight runs under the console's interrupts
/// on each host, the pump and the polled loop: the host declaring 100
/// keeps two frames in flight there and the honest default one.
template <typename H>
void spi_held_line(const char* name, bool polled) {
    constexpr uint16_t frames = 1024;
    uint8_t overran = 0;
    uint8_t faulted = 0;
    uint8_t lost = 0;   // of those, spi_overrun: a frame lost to the receive buffer
    uint8_t hung = 0;
    for (uint8_t run = 0; run < 8u; ++run) {
        typename H::Request r{};
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(buffer_a));
        r.rx = lend<Lease::reply>(static_cast<uint8_t*>(buffer_b));
        r.len = frames;
        r.mode = SpiMode::mode0;
        r.clock = SpiClock::div16;
        r.bits = SpiDataSize::bits8;
        r.polled = polled;
        spi_plain_done = false;
        console_drain();
        print(serial, live_line.text);
        const bool sync = H::start(r);
        uint32_t spins = 8'000'000UL;
        while (!sync && !spi_plain_done && spins-- != 0u) {
        }
        if (!sync && !spi_plain_done) {
            ++hung;
            (void)H::recover();
        }
        if (Spi<1>::overrun()) {
            ++overran;
            Spi<1>::clear_overrun();
        }
        if (H::status() != spi_ok) {
            ++faulted;
            lost = static_cast<uint8_t>(lost + (H::status() == spi_overrun ? 1u : 0u));
        }
    }
    console_drain();
    const auto ahead = H::write_ahead_from(SpiDataSize::bits8);
    print(serial, "  spi.held ", name, " (two in flight from /", ahead ? spi_division(*ahead) : 0u, ") ",
          polled ? "polled" : "pump", " receive /16 8-bit: ", overran, " overruns, ", faulted,
          " statuses not spi_ok (", lost, " spi_overrun), ", hung,
          " never completed, in 8 runs under the console's interrupts", crlf);
}

void spi_held_report() {
    for (const bool polled : {false, true}) {
        spi1_serves = spi1_plain;
        (void)SpiPlain::init(clock);
        spi_held_line<SpiPlain>("hold-off 200 (the default)", polled);
        spi1_serves = spi1_short;
        (void)SpiShort::init(clock);
        spi_held_line<SpiShort>("hold-off 100 (declared short)", polled);
    }
    spi1_serves = spi1_plain;
}

/// spi.window: THE PUMP'S OVR WINDOW, swept. A pumped WRITE of `frames`
/// 16-bit frames at /4 or /8 (64 or 128 cycles a frame), two in flight: SPI1's line
/// masked at the PFIC around start(), then `d` empty turns, then unmasked,
/// so the handler for frame 0 enters at a delay that moves one turn a run
/// across the moment frame 1 completes - before it (two interrupts), after
/// it (one: the overrun counted as the frame it is), and on it, where the
/// completion falls between the handler's status read and its DATAR read.
/// Every delay from 0 to 127, four sweeps: the runs that completed, those
/// that completed in fewer interrupts than frames (a frame lost and
/// counted), those that never completed (recovered after a bound), and
/// those after which OVR stood; the delays at which a frame was first lost
/// and at which a run first and last hung. With two frames the loss is the
/// LAST frame's; with three, the first pair's inside the run.
void spi_window_line(uint16_t frames, SpiClock rate) {
    constexpr uint16_t delays = 128;
    uint32_t done = 0;
    uint32_t lost = 0;
    uint32_t hung = 0;
    uint32_t ovr = 0;
    uint16_t first_lost = 0xFFFF;
    uint16_t first_hung = 0xFFFF;
    uint16_t last_hung = 0;
    for (uint8_t sweep = 0; sweep < 4u; ++sweep) {
        for (uint16_t d = 0; d < delays; ++d) {
            SpiPlain::Request r{};
            r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(buffer_a));
            r.len = frames;
            r.mode = SpiMode::mode0;
            r.clock = rate;
            r.bits = SpiDataSize::bits16;
            spi_plain_done = false;
            const uint32_t n0 = spi_meter.count();
            Pfic::disable(Spi<1>::irq);
            const bool sync = SpiPlain::start(r);
            for (uint32_t k = d; k != 0u; --k) {
                __asm__ volatile("");
            }
            Pfic::enable(Spi<1>::irq);
            uint32_t spins = 20'000u;
            while (!sync && !spi_plain_done && spins-- != 0u) {
            }
            // The meter's count is a plain word the vector writes: the
            // barrier makes the read below a fresh one.
            asm volatile("" ::: "memory");
            if (sync || spi_plain_done) {
                ++done;
                if (spi_meter.count() - n0 < frames) {
                    ++lost;
                    if (d < first_lost) {
                        first_lost = d;
                    }
                }
            } else {
                ++hung;
                if (d < first_hung) {
                    first_hung = d;
                }
                last_hung = d;
                (void)SpiPlain::recover();
            }
            if (Spi<1>::overrun()) {
                ++ovr;
                Spi<1>::clear_overrun();
            }
        }
    }
    console_drain();
    print(serial, "  spi.window write of ", frames, " 16-bit frames at /", spi_division(rate), ", two in flight, the entry swept over ",
          delays, " delays x 4: ", done, " completed (", lost, " with a frame lost and counted, the first at delay ",
          first_lost, "), ", hung, " never completed");
    if (hung != 0u) {
        print(serial, " (delays ", first_hung, " to ", last_hung, ")");
    }
    print(serial, ", ", ovr, " with OVR standing after", crlf);
}

void spi_window_report() {
    spi1_serves = spi1_plain;
    (void)SpiPlain::init(clock);
    for (const SpiClock rate : {SpiClock::div4, SpiClock::div8}) {
        spi_window_line(2, rate);
        spi_window_line(3, rate);
    }
}

void te_spi() {
    console_drain();
    for (uint32_t k = 0; k < 4096u; ++k) {
        buffer_a[k] = static_cast<uint8_t>(k * 7u + 3u);
    }
    spi1_serves = spi1_plain;
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
    spi_held_report();
    spi_window_report();
    spi1_serves = spi1_dma;
    spi_up = false;   // letter d brings its own host up again
    bench.verdict("ran", true);
}

// =============================================================================
// u - the serial transport on the crossed pair
// =============================================================================
/// USART2 on PA2/PA3 is the transport measured; the fourth port on PB0/PB1
/// is its peer through the board's crossed pair (PA2-PB1, PB0-PA3) - the
/// stimulus a receive is measured on, sent through its own transmit
/// engine (DMA1 channel 1) so that it adds one interrupt a burst. Both
/// ports count PCLK1, 72 MHz: the top rate is 72 MHz / 16, 4.5 Mbaud.
/// USART2's engines are DMA1's channels 7 out and 6 in (RM 11.2.3).
constexpr uint16_t loop_ring = 512;
using LoopTxEngine = DmaTxEngine<1, 7>;
using LoopRxEngine = DmaRxEngine<1, 6>;
using LoopIrq = Uart<2, Plat, loop_ring, loop_ring>;
using LoopTxe = Uart<2, Plat, loop_ring, loop_ring, UartFormat{}, LoopTxEngine, NoDmaEngine>;
using LoopDma = Uart<2, Plat, loop_ring, loop_ring, UartFormat{}, LoopTxEngine, LoopRxEngine>;
using LoopRes = Usart<2>;

/// Whether the part has the fourth port the pair needs: UART4 is the
/// CH32V203C8's, RB's and the CH32V303's, and not the 32 KB tier's
/// (parts/<part>.hpp). A part without it declines the letter by name,
/// as letter i does without I2C2.
constexpr bool uart_pair_part = device::has_usart(4u);

/// The stimulus side of the pair, formed only where the fourth port
/// exists: every name depends on `on`, so a part without UART4 never
/// instantiates the port's types (whose static_asserts refuse the
/// instance there). Its transmit slot is DMA1's channel 1 on the
/// CH32V203, DMA2's channel 5 on the CH32V303 (RM 11.2.3).
template <bool on>
struct StimSide {
    using Row = DmaRequestOf<on ? DmaRequest::uart4_tx : DmaRequest::uart4_tx>;
    using Port = Uart<on ? 4 : 4, Plat, 64, loop_ring, UartFormat{}, DmaTxEngine<Row::controller, Row::channel>,
                      NoDmaEngine>;
    using Res = Usart<on ? 4 : 4>;
};

Meter loop_meter;       // usart2_handler
Meter loop_dma_meter;   // dma1_channel6_handler and dma1_channel7_handler under letter u

/// Which shape owns USART2: 0 none, 1 LoopIrq, 2 LoopTxe, 3 LoopDma; and
/// whether the fourth port is up as the stimulus.
volatile uint8_t loop_owner = 0;
volatile bool stim_up = false;
volatile bool loop_edge = false;
volatile uint32_t loop_edge_at = 0;

constexpr uint32_t loop_rates[] = {115200, 1'000'000, 4'500'000};

BenchCounters loop_counters() {
    return bench_counters<Plat>(tick_meter, usart_meter, loop_meter, loop_dma_meter);
}

/// Give USART2 back; the owner forgotten after the release, whose stopped
/// streams may still raise a flag their vector serves.
template <bool on = uart_pair_part>
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
    if constexpr (on) {
        if (stim_up) {
            StimSide<on>::Port::release();
            stim_up = false;
        }
    }
}

void loop_wait_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 <= ms) {
    }
}

template <typename Port, bool on = uart_pair_part>
bool loop_up(uint8_t owner, uint32_t baud, bool with_stim) {
    using Stim = typename StimSide<on>::Port;
    loop_down();
    loop_owner = owner;
    bool up = Port::init(clock, baud);
    if (with_stim) {
        up = Stim::init(clock, baud) && up;
        stim_up = true;
    }
    loop_wait_ms(2);
    loop_edge = false;
    return up;
}

/// The wire of the rate USART2 runs at: its divisor over ten bits a byte.
uint32_t loop_wire() { return LoopIrq::actual_baud(72'000'000u) / 10u; }

const uint8_t* loop_pattern() { return reinterpret_cast<const uint8_t*>(filler<4096>.text); }

template <typename Port>
uint32_t loop_drain(uint32_t at, bool& in_order) {
    uint32_t got = 0;
    const uint8_t* want = loop_pattern();
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

/// uart.tx: n bytes through the transport, idling while the ring is full
/// and while an interrupt is coming, spinning through the last frame.
template <typename Port>
BenchSample loop_tx(uint32_t n) {
    console_drain();
    const BenchCounters c0 = loop_counters();
    Stopwatch<Ruler> sw;
    sw.start();
    uint32_t queued = 0;
    for (;;) {
        Plat::CriticalSection cs;
        queued += Port::write_bulk(std::span<const uint8_t>(loop_pattern() + queued, n - queued));
        if (queued >= n) {
            break;
        }
        Plat::idle();
    }
    for (;;) {
        Plat::CriticalSection cs;
        if (Port::tx_idle()) {
            break;
        }
        if (LoopRes::txe_interrupt() || LoopTxEngine::busy()) {
            Plat::idle();
        }
    }
    const uint32_t w = sw.elapsed();
    const BenchCounters c1 = loop_counters();
    return bench_sample(w, c0, c1);
}

/// uart.rx: a burst of n sent by the fourth port's engine, received on
/// USART2, the consumer idling until the edge and draining on it - or,
/// with `poll`, asking harvest() once a tick (the owner's poll).
template <typename Port, bool poll, bool on = uart_pair_part>
BenchSample loop_rx(uint32_t n, uint32_t& got, bool& in_order) {
    using Stim = typename StimSide<on>::Port;
    console_drain();
    bool scrap = true;
    (void)loop_drain<Port>(0, scrap);
    Port::clear_errors();
    loop_edge = false;
    in_order = true;
    got = 0;
    const BenchCounters c0 = loop_counters();
    Stopwatch<Ruler> sw;
    sw.start();
    (void)Stim::write_bulk(std::span<const uint8_t>(loop_pattern(), n));
    uint32_t tick = Ticker::ticks();
    const uint32_t t0 = tick;
    while (got < n && Ticker::ticks() - t0 < 1000u) {
        // The platform's idle() returns with interrupts ENABLED, so the
        // flag is taken under a guard of its own after it: a vector between
        // the read and the clear would be lost.
        {
            Plat::CriticalSection cs;
            if (!loop_edge) {
                Plat::idle();
            }
        }
        bool edge = false;
        {
            Plat::CriticalSection cs;
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
    const BenchCounters c1 = loop_counters();
    return bench_sample(w, c0, c1);
}

/// uart.edge: a burst of 16 from the fourth port, its TC spun on, then
/// the cycles to the edge that follows.
template <typename Port, bool poll, bool on = uart_pair_part>
BenchSample loop_edge_after(uint32_t& late) {
    using Stim = typename StimSide<on>::Port;
    using StimRes = typename StimSide<on>::Res;
    console_drain();
    bool scrap = true;
    (void)loop_drain<Port>(0, scrap);
    StimRes::clear_flags(usart_tc);
    loop_edge = false;
    (void)Stim::write_bulk(std::span<const uint8_t>(loop_pattern(), 16));
    uint32_t got = 0;
    while (!StimRes::tx_complete()) {
        if constexpr (!poll) {
            if (loop_edge) {
                loop_edge = false;
                got += loop_drain<Port>(got, scrap);
            }
        }
    }
    const uint32_t t_tc = Ruler::now();
    const BenchCounters c0 = loop_counters();
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
    const BenchCounters c1 = loop_counters();
    (void)loop_drain<Port>(got, scrap);
    return bench_sample(at - t_tc, c0, c1);
}

template <bool on = uart_pair_part>
void tu_uart() {
    if constexpr (on) {
        using Stim = typename StimSide<on>::Port;
        bool all_up = true;
        for (const uint32_t baud : loop_rates) {
            all_up = loop_up<LoopIrq>(1, baud, false) && all_up;
            const uint32_t wire = loop_wire();
            print(serial, "  USART2 at ", LoopIrq::actual_baud(72'000'000u), " baud", crlf);
            for (const uint32_t n : {256u, 4096u}) {
                bench_line(serial, "uart.tx", n, loop_tx<LoopIrq>(n), Ruler::hz(), wire);
            }
            all_up = loop_up<LoopTxe>(2, baud, false) && all_up;
            for (const uint32_t n : {256u, 4096u}) {
                bench_line(serial, "uart.tx.dma", n, loop_tx<LoopTxe>(n), Ruler::hz(), wire);
            }
            all_up = loop_up<LoopTxe>(2, baud, true) && all_up;
            for (const uint32_t n : {16u, 256u}) {
                uint32_t got = 0;
                bool in_order = false;
                const BenchSample s = loop_rx<LoopTxe, false>(n, got, in_order);
                bench_line(serial, "uart.rx", n, s, Ruler::hz(), wire);
                if (got != n || !in_order) {
                    print(serial, "  received ", got, in_order ? " in order" : " OUT OF ORDER", "; ORE ",
                          LoopTxe::hw_overruns(), " FE ", LoopTxe::frame_errors(), " stim idle ",
                          Stim::tx_idle() ? 1 : 0, " pending ", LoopTxe::rx_pending(), crlf);
                }
            }
            all_up = loop_up<LoopDma>(3, baud, true) && all_up;
            for (const uint32_t n : {16u, 256u}) {
                uint32_t got = 0;
                bool in_order = false;
                const BenchSample s = loop_rx<LoopDma, false>(n, got, in_order);
                bench_line(serial, "uart.rx.dma", n, s, Ruler::hz(), wire);
                if (got != n || !in_order) {
                    print(serial, "  received ", got, in_order ? " in order" : " OUT OF ORDER", "; ORE ",
                          LoopDma::hw_overruns(), " FE ", LoopDma::frame_errors(), " ring ",
                          LoopDma::rx_overruns(), crlf);
                }
            }
            {
                uint32_t late = 0;
                const BenchSample e = loop_edge_after<LoopDma, false>(late);
                bench_line(serial, "uart.edge", 16, e, Ruler::hz(), 0);
                print(serial, "  the edge ", e.wall, " cycles after the last stop bit = ",
                      e.wall * 10u / (Ruler::hz() / wire), " tenths of a frame",
                      late != 0u ? " (TIMED OUT)" : "", crlf);
            }
            loop_down();
        }
        bench.verdict("the pair came up at every rate", all_up);
    } else {
        print(serial, "  letter u declined on this part: no UART4, so no crossed pair to measure the "
                      "transport against",
              crlf);
    }
    bench.verdict("ran", true);
}

// =============================================================================
// i - the I2C host above the wire, on the self-link
// =============================================================================

/// THE RATE LETTER i RUNS AT. CTLR2.FREQ states PB1 in whole megahertz
/// and RM 19.12.2 stops it at 60, and this app's 144 MHz puts PB1 at 72:
/// no I2C timing exists there at all. 120 MHz is the fastest HCLK whose
/// PB1 (60 MHz through the /2) the chapter takes - both speeds exact
/// (CCR 300 and 50) - so the letter takes the core there, the console and
/// the tick re-initialized at it, and back to 144 at its end. The ruler
/// (the STK's composed count) reads the reload it runs at, so its cycles
/// are 120 MHz ones inside the letter and every line says so.
using I2cClock = Clock<ClockSource::pll, 120'000'000>;
constexpr I2cClock i2c_clock;

Meter i2c_meter;        ///< I2C1's two vectors: the host
Meter i2c_dma_meter;    ///< DMA1 channels 6 and 7 under letter i: the host's engines
Meter i2c_peer_meter;   ///< I2C2's two vectors: the target, the instrument
/// Which host I2C1's vectors serve: 1 the pump, 2 the engines, 0 letter i
/// is not running (the two channels are letter u's then).
volatile uint8_t i2c_owner = 0;
volatile bool i2c_done = false;
/// The longest single entry of each of the host's vectors over the
/// letter (R5's figure), cycles of the body between two reads of its own.
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
/// THE TIMELINE of one tenure: every entry of the host's and the
/// target's vectors stamped against the start() call, with STAR1 as the
/// entry found it - where a fixed cost goes, entry by entry.
constexpr uint8_t i2c_trace_depth = 24;
volatile bool i2c_tracing = false;
uint32_t i2c_trace_base = 0;
volatile uint8_t i2c_trace_n = 0;
uint32_t i2c_trace_at[i2c_trace_depth];
uint16_t i2c_trace_s1[i2c_trace_depth];
char i2c_trace_who[i2c_trace_depth];
[[gnu::always_inline]] inline void i2c_trace(char who, uint32_t t0, uint16_t s1) {
    if (i2c_tracing && i2c_trace_n < i2c_trace_depth) {
        i2c_trace_at[i2c_trace_n] = t0 - i2c_trace_base;
        i2c_trace_s1[i2c_trace_n] = s1;
        i2c_trace_who[i2c_trace_n] = who;
        i2c_trace_n = static_cast<uint8_t>(i2c_trace_n + 1u);
    }
}
alignas(4) uint8_t i2c_out[256];
alignas(4) uint8_t i2c_in[256];
alignas(4) uint8_t i2c_src[256];    ///< what the target transmits
alignas(4) uint8_t i2c_sink[256];   ///< what the target received

BenchCounters i2c_counters() {
    return bench_counters<Plat>(tick_meter, usart_meter, i2c_meter, i2c_dma_meter, i2c_peer_meter);
}

constexpr uint8_t i2c_target = 0x3A;   ///< I2C2's own address on the self-link
constexpr uint8_t i2c_nobody = 0x42;

struct I2cOp {
    const char* name;
    uint8_t addr;
    uint8_t tx;
    uint8_t rx;
};
constexpr I2cOp i2c_ops[] = {
    {"i2c.write", i2c_target, 1, 0},   {"i2c.write", i2c_target, 2, 0},
    {"i2c.write", i2c_target, 16, 0},  {"i2c.write", i2c_target, 255, 0},
    {"i2c.read", i2c_target, 0, 1},    {"i2c.read", i2c_target, 0, 2},
    {"i2c.read", i2c_target, 0, 16},   {"i2c.read", i2c_target, 0, 255},
    {"i2c.wr", i2c_target, 1, 1},      {"i2c.wr", i2c_target, 1, 2},
    {"i2c.wr", i2c_target, 1, 16},     {"i2c.probe", i2c_target, 0, 0},
    {"i2c.probe", i2c_nobody, 0, 0},
    {"i2c.write", i2c_target, 3, 0},   // the third short write the polled-verb question asks for
};
constexpr uint8_t i2c_row_w16 = 2;
constexpr uint8_t i2c_row_w255 = 3;
constexpr uint8_t i2c_row_r16 = 6;
constexpr uint8_t i2c_row_r255 = 7;

struct I2cRun {
    BenchSample s;
    uint32_t start;
    uint8_t status;
};

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

/// The bus's own time for a tenure at an SCL period of `t` cycles: the
/// START and each repeated START a period, the address and every byte
/// nine. The STOP is not counted: the engine completes when it REQUESTS
/// it, and what its drain costs the next tenure is its own line.
uint32_t i2c_wire_cycles(uint32_t t, uint8_t tx, uint8_t rx) {
    uint32_t periods = 1u + 9u + 9u * tx;
    if (rx != 0u) {
        if (tx != 0u) {
            periods += 1u + 9u;
        }
        periods += 9u * rx;
    }
    return t * periods;
}

void i2c_idle_until_done() {
    const uint32_t t0 = Ticker::ticks();
    while (!i2c_done && Ticker::ticks() - t0 < 1000u) {
        Plat::CriticalSection cs;
        if (!i2c_done) {
            Plat::idle();
        }
    }
    // What the handlers wrote is read from memory after this: the meters
    // and the timeline are plain words a handler stores.
    asm volatile("" ::: "memory");
}

/// Everything that names I2C2 hangs on `on`, so a part without it forms
/// none of it (the CH32V203C6's image declines the letter by name).
template <bool on>
struct I2cLetter {
    using Res = I2c<on ? 1 : 1>;
    using Pump = I2cHost<on ? 1 : 1>;
    using Engined = I2cHost<on ? 1 : 1, i2c_default_pins<1>, DmaTxEngine<1, 6>, DmaRxEngine<1, 7>>;
    using Target = I2cClient<on ? 2 : 2>;
    using Scl = Pin<'B', on ? 6 : 6>;
    using EdgeTim = Tim<on ? 4 : 4>;

    static inline uint8_t pos = 0;

    /// The target, served from its own two vectors: every byte it is
    /// asked for comes from i2c_src, restarting at every address match,
    /// and the byte its shifter wanted beyond the host's NACK is dropped
    /// by flush() (docs/ch32vx03/i2c.md).
    [[gnu::always_inline]] static void target_event() {
        switch (Target::service()) {
            case I2cClientEvent::addressed: pos = 0; break;
            case I2cClientEvent::byte_received: i2c_sink[pos++] = Target::take(); break;
            case I2cClientEvent::byte_wanted: Target::give(i2c_src[pos++]); break;
            default: break;
        }
    }
    [[gnu::always_inline]] static void target_error() {
        if (Target::error_service() == I2cClientEvent::nacked) {
            (void)Target::flush();
        }
    }

    template <typename Host>
    static typename Host::Request request(uint8_t addr, uint8_t tx, uint8_t rx, I2cSpeed speed) {
        typename Host::Request r{};
        r.addr = addr;
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(i2c_out));
        r.tx_len = tx;
        r.rx = lend<Lease::reply>(static_cast<uint8_t*>(i2c_in));
        r.rx_len = rx;
        r.speed = speed;
        return r;
    }

    static void quiet() {
        const uint32_t t0 = Ticker::ticks();
        while ((Res::stopping() || Res::busy()) && Ticker::ticks() - t0 < 10u) {
        }
    }

    template <typename Host>
    static void take(bool engines) {
        i2c_owner = 0;
        (void)Host::init(i2c_clock);
        i2c_owner = engines ? 2u : 1u;
    }

    template <typename Host>
    static I2cRun once(const typename Host::Request& r) {
        quiet();
        const BenchCounters c0 = i2c_counters();
        const uint32_t q0 = i2c_meter.count() + i2c_dma_meter.count();
        const uint32_t k0 = i2c_meter.cycles() + i2c_dma_meter.cycles();
        i2c_done = false;
        Stopwatch<Ruler> sw;
        sw.start();
        const bool sync = Host::start(r);
        const uint32_t st = sw.elapsed();
        if (!sync) {
            i2c_idle_until_done();
        }
        const uint32_t w = sw.elapsed();
        const BenchCounters c1 = i2c_counters();
        BenchSample s = bench_sample(w, c0, c1);
        s.irq = i2c_meter.count() + i2c_dma_meter.count() - q0;
        s.isr = i2c_meter.cycles() + i2c_dma_meter.cycles() - k0;
        return {s, st, (sync || i2c_done) ? Host::status() : static_cast<uint8_t>(0xFF)};
    }

    /// The SCL period on the wire: TIM4's channel 1 is PB6, SCL's own pad,
    /// and in external clock mode 1 the counter IS the number of SCL
    /// rising edges (test_vx03_i2c's instrument). Read twice inside a
    /// 255-byte read through the engines, after the address and two data
    /// bytes and 240 bytes later: every clock of those bytes, the
    /// acknowledges and whatever the target stretched included.
    static uint32_t scl_period(I2cSpeed speed) {
        take<Engined>(true);
        EdgeTim::init();
        if (!EdgeTim::remap(0) || !EdgeTim::configure({.prescaler = 0, .period = 0xFFFF})) {
            return 0;
        }
        if (!EdgeTim::capture_channel(0, {.select = TimChannelSelect::direct,
                                          .polarity = TimCapturePolarity::rising,
                                          .prescaler = TimCapturePrescaler::every,
                                          .filter = 0,
                                          .enable = true}) ||
            !EdgeTim::slave({.mode = TimSlaveMode::external_clock1, .trigger = TimTrigger::ti1})) {
            return 0;
        }
        EdgeTim::set_count(0);
        EdgeTim::enable(true);
        const auto r = request<Engined>(i2c_target, 0, 255, speed);
        quiet();
        i2c_done = false;
        constexpr uint32_t first = 9u * 3u;
        constexpr uint32_t last = first + 9u * 240u;
        uint32_t t1 = 0, c1 = 0, t2 = 0, c2 = 0;
        const uint32_t t0 = Ruler::now();
        (void)Engined::start(r);
        while (!i2c_done && Ruler::now() - t0 < I2cClock::hz / 10u) {
            const uint32_t c = EdgeTim::count();
            const uint32_t t = Ruler::now();
            if (c1 == 0u && c >= first) {
                c1 = c;
                t1 = t;
            } else if (c1 != 0u && c >= last) {
                c2 = c;
                t2 = t;
                break;
            }
        }
        i2c_idle_until_done();
        EdgeTim::enable(false);
        EdgeTim::release();
        return c2 > c1 ? (t2 - t1 + (c2 - c1) / 2u) / (c2 - c1) : 0u;
    }

    template <typename Host>
    static void ops(const char* suffix, I2cSpeed speed, uint32_t t, bool engines) {
        take<Host>(engines);
        BenchSample rows[sizeof(i2c_ops) / sizeof(i2c_ops[0])] = {};
        uint8_t row = 0;
        uint8_t mismatched = 0;
        for (const I2cOp& op : i2c_ops) {
            const auto r = request<Host>(op.addr, op.tx, op.rx, speed);
            I2cRun best{{UINT32_MAX, 0, 0, 0}, 0, 0};
            const uint8_t want = op.addr == i2c_nobody ? i2c_nack_addr : i2c_ok;
            uint8_t worst = want;
            asm volatile("" ::: "memory");
            const uint32_t p0 = i2c_peer_meter.count();
            const uint32_t pk0 = i2c_peer_meter.cycles();
            for (uint8_t run = 0; run < 8u; ++run) {
                for (uint16_t i = 0; i < op.rx; ++i) {
                    i2c_in[i] = 0;
                }
                const I2cRun one = once<Host>(r);
                if (one.status != want) {
                    worst = one.status;
                }
                for (uint16_t i = 0; i < op.rx; ++i) {
                    if (i2c_in[i] != i2c_src[i]) {
                        ++mismatched;
                        break;
                    }
                }
                if (one.s.wall < best.s.wall) {
                    best = one;
                }
            }
            const uint32_t wire = i2c_wire_cycles(t, op.tx, op.rx);
            const uint32_t n = static_cast<uint32_t>(op.tx) + op.rx;
            asm volatile("" ::: "memory");
            const uint32_t peer_entries = (i2c_peer_meter.count() - p0 + 4u) / 8u;
            const uint32_t peer_cycles = (i2c_peer_meter.cycles() - pk0 + 4u) / 8u;
            char name[24] = {};
            uint8_t k = 0;
            for (const char* c = op.name; *c != '\0' && k < 15u; ++c) {
                name[k++] = *c;
            }
            for (const char* c = suffix; *c != '\0' && k < 23u; ++c) {
                name[k++] = *c;
            }
            bench_line(serial, name, n, best.s, I2cClock::hz,
                       n == 0u ? 0u : static_cast<uint32_t>(static_cast<uint64_t>(n) * I2cClock::hz / wire));
            print(serial, "  ", op.addr == i2c_nobody ? "absent address, " : "", "fixed=",
                  static_cast<int32_t>(best.s.wall - wire), " start=", best.start, " status=",
                  i2c_status_name(worst), "; the target ", peer_entries, " entries and ", peer_cycles,
                  " cycles a tenure", crlf);
            rows[row++] = best.s;
            console_drain();
        }
        if (!engines) {
            print(serial, "  per byte: written ",
                  (rows[i2c_row_w255].isr - rows[i2c_row_w16].isr) / 239u, " cycles of handler and ",
                  rows[i2c_row_w255].irq - rows[i2c_row_w16].irq, " interrupts in 239; read ",
                  (rows[i2c_row_r255].isr - rows[i2c_row_r16].isr) / 239u, " cycles and ",
                  rows[i2c_row_r255].irq - rows[i2c_row_r16].irq, " interrupts in 239", crlf);
        }
        if (mismatched != 0u) {
            print(serial, "  READ DATA MISMATCHED in ", mismatched, " runs", crlf);
        }
    }

    /// One tenure traced: the host's and the target's entries in order,
    /// each with its time from the start() call and STAR1 as found.
    static void trace(const char* what, uint8_t tx, uint8_t rx, I2cSpeed speed) {
        take<Pump>(false);
        const auto r = request<Pump>(i2c_target, tx, rx, speed);
        quiet();
        i2c_trace_n = 0;
        i2c_done = false;
        i2c_tracing = true;
        i2c_trace_base = Ruler::now();
        (void)Pump::start(r);
        const uint32_t ret = Ruler::now() - i2c_trace_base;
        i2c_idle_until_done();
        const uint32_t end = Ruler::now() - i2c_trace_base;
        i2c_tracing = false;
        print(serial, "  the timeline of a ", what, " at ", speed == I2cSpeed::fast_400k ? "400" : "100",
              " kHz: start() returned at ", ret, ", the thread saw the end at ", end, crlf, "   ");
        for (uint8_t i = 0; i < i2c_trace_n; ++i) {
            print(serial, " ", i2c_trace_who[i], "@", i2c_trace_at[i], "(", hex(i2c_trace_s1[i]), ")");
        }
        print(serial, crlf);
        console_drain();
    }

    /// Every operation once with the host vectors' longest entry watched
    /// (R5): a pass of its own, so no other line carries the watch.
    template <typename Host>
    static void watch(I2cSpeed speed, bool engines) {
        take<Host>(engines);
        i2c_watching = true;
        for (const I2cOp& op : i2c_ops) {
            (void)once<Host>(request<Host>(op.addr, op.tx, op.rx, speed));
        }
        i2c_watching = false;
    }

    static void stop_drain(I2cSpeed speed) {
        take<Pump>(false);
        const auto next = request<Pump>(i2c_target, 0, 0, speed);
        const I2cOp firsts[] = {{"write", i2c_target, 2, 0}, {"read", i2c_target, 0, 2},
                                {"wr", i2c_target, 1, 1}};
        for (const I2cOp& op : firsts) {
            const auto r = request<Pump>(op.addr, op.tx, op.rx, speed);
            uint32_t best = UINT32_MAX;
            uint32_t calm = UINT32_MAX;
            for (uint8_t run = 0; run < 8u; ++run) {
                quiet();
                i2c_done = false;
                (void)Pump::start(r);
                i2c_idle_until_done();
                i2c_done = false;
                Stopwatch<Ruler> sw;
                sw.start();
                (void)Pump::start(next);
                const uint32_t behind = sw.elapsed();
                i2c_idle_until_done();
                const uint32_t q = once<Pump>(next).start;
                best = behind < best ? behind : best;
                calm = q < calm ? q : calm;
            }
            print(serial, "bench i2c.stop after a ", op.name, " n=", static_cast<uint32_t>(op.tx) + op.rx,
                  ": start() ", best, " cycles right behind its completion, ", calm,
                  " on a quiet bus - the drain ", best - calm, " cycles", crlf);
            console_drain();
        }
    }

    static void run() {
        for (uint16_t i = 0; i < 256u; ++i) {
            i2c_out[i] = static_cast<uint8_t>(0xA0u + i);
            i2c_src[i] = static_cast<uint8_t>(0x5Bu * i + 7u);
        }
        // The core to 120 MHz: the console drained first, then re-made at
        // the new rate, the tick likewise.
        console_drain();
        const bool up = I2cClock::init();
        (void)Serial::init(i2c_clock, console_baud);
        (void)Ticker::init(i2c_clock);
        print(serial, "  the core at 120 MHz (", up ? "PLL" : "FAILED",
              "): PB1 60 MHz, the most CTLR2.FREQ states; every cycle below is a 120 MHz one", crlf);
        const bool target_up = Target::init(i2c_clock, {.own = i2c_target}, {.interrupts = true});
        print(serial, "  I2C1 the host on PB6/PB7, I2C2 the target at ", hex(i2c_target),
              " on PB10/PB11 (", target_up ? "up" : "FAILED", "), served from its own vectors", crlf);
        i2c_ev_max = 0;
        i2c_er_max = 0;
        i2c_dma_max = 0;
        for (const I2cSpeed speed : {I2cSpeed::standard_100k, I2cSpeed::fast_400k}) {
            const uint32_t t = scl_period(speed);
            print(serial, "  SCL at ", speed == I2cSpeed::fast_400k ? "400" : "100", " kHz: CKCFGR ",
                  hex(Engined::timing_of(speed).ckcfgr), " states ", Engined::scl_hz(speed),
                  " Hz, the wire ", t, " cycles a period = ", t != 0u ? I2cClock::hz / t : 0u, " Hz",
                  crlf);
            if (t == 0u) {
                continue;
            }
            console_drain();
            ops<Pump>("", speed, t, false);
            ops<Engined>(".dma", speed, t, true);
            stop_drain(speed);
            watch<Pump>(speed, false);
            watch<Engined>(speed, true);
            trace("write of 1", 1, 0, speed);
            trace("register read of 1+2", 1, 2, speed);
        }
        print(serial, "  the longest entry of each host vector: events ", i2c_ev_max, " cycles, errors ",
              i2c_er_max, ", the channels ", i2c_dma_max, crlf);
        console_drain();
        i2c_owner = 0;
        Pump::release();
        Target::release();
        // Back to the app's 144 MHz.
        const bool back = SysClock::init();
        (void)Serial::init(clock, console_baud);
        (void)Ticker::init(clock);
        print(serial, "  the core back at 144 MHz (", back ? "PLL" : "FAILED", ")", crlf);
    }
};

constexpr bool i2c_self_link_part = device::i2c_count >= 2u;

template <bool on = i2c_self_link_part>
void ti_i2c() {
    if constexpr (on) {
        I2cLetter<on>::run();
    } else {
        print(serial, "  letter i declined on this part: no I2C2, so no self-link to measure the "
                      "host against",
              crlf);
    }
    bench.verdict("ran", true);
}

/// The vectors' bodies, empty on a part without I2C2.
template <bool on = i2c_self_link_part>
[[gnu::always_inline]] inline void i2c_host_event() {
    if constexpr (on) {
        using L = I2cLetter<on>;
        if (i2c_owner == 2u ? L::Engined::isr() : L::Pump::isr()) {
            i2c_done = true;
        }
    }
}
template <bool on = i2c_self_link_part>
[[gnu::always_inline]] inline void i2c_host_error() {
    if constexpr (on) {
        using L = I2cLetter<on>;
        if (i2c_owner == 2u ? L::Engined::error_isr() : L::Pump::error_isr()) {
            i2c_done = true;
        }
    }
}
template <bool on = i2c_self_link_part>
[[gnu::always_inline]] inline void i2c_host_dma() {
    if constexpr (on) {
        if (I2cLetter<on>::Engined::dma_isr()) {
            i2c_done = true;
        }
    }
}
template <bool on = i2c_self_link_part>
[[gnu::always_inline]] inline void i2c_target_event() {
    if constexpr (on) {
        I2cLetter<on>::target_event();
    }
}
template <bool on = i2c_self_link_part>
[[gnu::always_inline]] inline uint16_t i2c_target_status() {
    if constexpr (on) {
        return I2c<on ? 2 : 2>::status1();
    } else {
        return 0;
    }
}
template <bool on = i2c_self_link_part>
[[gnu::always_inline]] inline void i2c_target_error() {
    if constexpr (on) {
        I2cLetter<on>::target_error();
    }
}

/// Letter u's USART2 vectors, formed only in an image that carries the
/// letter on a part with the pair: elsewhere no loop shape is referenced
/// and its rings - three kilobytes and a half - are not in the image.
constexpr bool loop_carried = uart_pair_part && test_letter_carried('u');
template <bool on = loop_carried>
[[gnu::always_inline]] inline void loop_usart_vector() {
    if constexpr (on) {
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
}
/// DMA1's channel 7 under letter u: whether the loop's transmit engine
/// owned it and was served.
template <bool on = loop_carried>
[[gnu::always_inline]] inline bool loop_tx_dma_vector() {
    if constexpr (on) {
        if (loop_owner >= 2u) {
            loop_dma_meter.enter();
            if (loop_owner == 2u) {
                (void)LoopTxe::dma_isr();
            } else {
                (void)LoopDma::dma_isr();
            }
            loop_dma_meter.leave();
            return true;
        }
    }
    return false;
}
/// DMA1's channel 6 under letter u: the circular ring's laps and their
/// half and full marks, whose true is the edge as the USART's.
template <bool on = loop_carried>
[[gnu::always_inline]] inline void loop_rx_dma_vector() {
    if constexpr (on) {
        loop_dma_meter.enter();
        if (loop_owner == 3u && LoopDma::dma_isr()) {
            loop_edge_at = Ruler::now();
            loop_edge = true;
        }
        loop_dma_meter.leave();
    }
}

/// Letter u's stimulus vectors, empty on a part without the fourth port:
/// whether controller `c`'s channel `ch` is the stimulus's transmit slot
/// and served it, and the port's own vector.
template <uint8_t c, uint8_t ch, bool on = uart_pair_part>
[[gnu::always_inline]] inline bool stim_dma_vector() {
    if constexpr (on) {
        using Side = StimSide<on>;
        if constexpr (Side::Row::controller == c && Side::Row::channel == ch) {
            if (stim_up) {
                (void)Side::Port::dma_isr();
                return true;
            }
        }
    }
    return false;
}
template <bool on = uart_pair_part>
[[gnu::always_inline]] inline void stim_isr() {
    if constexpr (on) {
        if (stim_up) {
            (void)StimSide<on>::Port::isr();
        }
    }
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
BRIO_CH32_LEAF_VECTOR(systick_handler) {
    tick_meter.enter();
    brio::Ticker::tick();
    tick_meter.leave();
}

BRIO_CH32_LEAF_VECTOR(usart1_handler) {
    usart_meter.enter();
    (void)Serial::isr();
    usart_meter.leave();
}

BRIO_CH32_VECTOR(dma1_channel1_handler) {
    if (stim_dma_vector<1, 1>()) {   // letter u's stimulus: not metered, one a burst
        return;
    }
    copy_meter.enter();
    copy_vector();
    copy_meter.leave();
}

BRIO_CH32_VECTOR(dma1_channel7_handler) {
    if (i2c_owner == 2u) {
        i2c_dma_meter.enter();
        const uint32_t t0 = i2c_watching ? Ruler::now() : 0u;
        i2c_host_dma();
        if (i2c_watching) {
            i2c_longest(i2c_dma_max, t0);
        }
        i2c_dma_meter.leave();
        return;
    }
    if (loop_tx_dma_vector()) {
        return;
    }
    paced_meter.enter();
    paced_vector();
    paced_meter.leave();
}

/// USART2's receive channel under letter u: the circular ring's laps and
/// their half and full marks, whose true is the edge as the USART's.
BRIO_CH32_VECTOR(dma1_channel6_handler) {
    if (i2c_owner == 2u) {
        i2c_dma_meter.enter();
        const uint32_t t0 = i2c_watching ? Ruler::now() : 0u;
        i2c_host_dma();
        if (i2c_watching) {
            i2c_longest(i2c_dma_max, t0);
        }
        i2c_dma_meter.leave();
        return;
    }
    loop_rx_dma_vector();
}

BRIO_CH32_LEAF_VECTOR(usart2_handler) { loop_usart_vector(); }

/// The stimulus's slot on the CH32V303; nothing in the CH32V203's vector
/// table names this body.
BRIO_CH32_VECTOR(dma2_channel5_handler) { (void)stim_dma_vector<2, 5>(); }

BRIO_CH32_LEAF_VECTOR(uart4_handler) { stim_isr(); }

BRIO_CH32_VECTOR(dma1_channel2_handler) {
    spi_rx_meter.enter();
    spi_rx_vector();
    spi_rx_meter.leave();
}

BRIO_CH32_VECTOR(dma1_channel3_handler) {
    spi_tx_meter.enter();
    spi_tx_vector();
    spi_tx_meter.leave();
}

BRIO_CH32_VECTOR(spi1_handler) {
    spi_meter.enter();
    if (spi1_serves == spi1_plain) {
        if (SpiPlain::isr()) {
            spi_plain_done = true;
        }
    } else if (spi1_serves == spi1_short) {
        if (SpiShort::isr()) {
            spi_plain_done = true;
        }
    } else if (SpiDma::isr()) {
        spi_done = true;
    }
    spi_meter.leave();
}

/// Letter i: I2C1 the host, I2C2 the target on the self-link.
BRIO_CH32_VECTOR(i2c1_ev_handler) {
    i2c_meter.enter();
    const uint32_t t0 = (i2c_watching || i2c_tracing) ? Ruler::now() : 0u;
    if (i2c_tracing) {
        i2c_trace('H', t0, I2c<1>::status1());
    }
    i2c_host_event();
    if (i2c_watching) {
        i2c_longest(i2c_ev_max, t0);
    }
    i2c_meter.leave();
}
BRIO_CH32_VECTOR(i2c1_er_handler) {
    i2c_meter.enter();
    const uint32_t t0 = (i2c_watching || i2c_tracing) ? Ruler::now() : 0u;
    if (i2c_tracing) {
        i2c_trace('E', t0, I2c<1>::status1());
    }
    i2c_host_error();
    if (i2c_watching) {
        i2c_longest(i2c_er_max, t0);
    }
    i2c_meter.leave();
}
BRIO_CH32_LEAF_VECTOR(i2c2_ev_handler) {
    i2c_peer_meter.enter();
    if (i2c_tracing) {
        i2c_trace('T', Ruler::now(), i2c_target_status());
    }
    i2c_target_event();
    i2c_peer_meter.leave();
}
BRIO_CH32_LEAF_VECTOR(i2c2_er_handler) {
    i2c_peer_meter.enter();
    i2c_target_error();
    i2c_peer_meter.leave();
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
    bench.letter('u', "the serial transport on the crossed pair: transmit, receive, the edge", tu_uart<>);
    bench.letter('i', "the I2C host above the wire on the self-link: tenures, their fixed cost, the STOP's drain",
                 ti_i2c<>);

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
