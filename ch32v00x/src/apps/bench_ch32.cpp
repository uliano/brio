// bench_ch32 - the benchmark skeleton on the CH32V00x (QingKe V2C on the
// CH32V006, V2A on the CH32V003, RV32EC): the instrument's self-check and
// cost, the runtime's copy and fill against the core's floor, a print
// through the console transport against the wire, the tick's floor, the
// DMA - copy, fill, a timer-paced block and the SPI's engines - and the
// SPI host above the wire (its polled loops, its pump, a request's fixed
// cost), one line per operation and size, in util/bench.hpp's grammar
// (docs/design/benchmark.md).
//
// NOT A TEST. TestBench frames it (the menu, the ALL: line bin/brio
// waits for); every letter ends with one verdict "ran", and letter r's
// two verdicts on the ruler are the only ones that judge anything.
//
// THE BOARD AND THE RATES. The CH32V006K8U6 module or the CH32V003F4P6
// board at 48 MHz (the HSI's 24 MHz doubled by the PLL), the rate
// test_ch32_platform runs at. The console is USART1 on PD5/PD6 through
// the WCH-Link at 115200 8N1, the interrupt-driven transport with no DMA
// engine, bound as that suite binds it. THE RAM BOUNDS THE SIZES: the one
// work buffer is half the part's SRAM (4096 bytes on the CH32V006, 1024
// on the CH32V003), and every size below that names "the buffer" is that
// part's; on the CH32V003 the app builds as group images (design/
// overview.md, "A suite's image fits the family's smallest chip").
//
// NO KERNEL. The suite's shape: a prompt loop over the console, which
// here sleeps through BenchIdle<P, Ruler>::idle() when no byte is
// pending, as the kernel's loop would. That adapter is the platform every
// type below names, the transport's rings included.
//
// THE RULER. Ticker::cycles() on the STK (ch32v00x/ticker.hpp): the tick
// count and the counter's position in its period composed by
// util/cycle_count.hpp, in HCLK cycles - so hz() is the clock's 48 MHz.
// A read is a CMP load, two loads each of the tick count and of SR, a
// CNT load, a multiply and an add. Ruler::now() is always_inline and
// flatten, so the whole read is spelled inline wherever a stamp or a
// Stopwatch takes one, the vectors included: no call on the hot path.
//
// THE METERS. One IsrMeter for each vector or group of vectors: the
// STK's (the tick), USART1's (the transport's ISR body), and letter d's
// three - DMA channel 1 (the copy engine), channels 2 and 3 with SPI1's
// own (the SPI host's engines; letter e's pump shares that one), channel
// 5 (the paced block). enter() is
// each handler's first statement and leave() its last, the body between
// them as a suite binds it. The hardware prologue's entry and exit,
// measured in docs/ch32v00x/platform.md, lie outside the stamps.
//
// What each letter prints:
//
//   r  THE RULER'S SELF-CHECK. delay_us(clock, 1000) is REFUSED on this
//      family, as every wait of one tick period or more is
//      (ch32v00x/delay.hpp), so the check reads TWO delay_us(clock, 500)
//      back to back on the ruler: at least hz/1000 cycles, and under
//      hz/1000 + 5 per cent.
//      Then the instrument's cost, four lines with n=0 and wire=0; in
//      the three averaged ones EVERY FIELD IS THE RUN'S TOTAL DIVIDED BY
//      ITS UNITS, and the note line before each states the units and the
//      run's interrupts:
//        ruler   one Ruler::now(): 1000 reads in ten batches of 100, each
//                batch started on a fresh tick so that no interrupt lands
//                in it; a batch's Stopwatch adds one read per hundred
//        stopwatch  an empty Stopwatch - start() then elapsed(), nothing
//                between - the best of 8: not averaged, because it is
//                exactly what every Stopwatch line of m, p and t holds
//                of the instrument
//        stamp   one enter()+leave() pair with nothing between, on a
//                meter bound to no vector: 1000 pairs, batched the same
//        window  one BenchIdle::idle() turn with nothing pending but the
//                tick, over the turns of 100 ms. wall is the turn (a tick
//                period) and busy HOLDS THE TICK HANDLER, added back
//                because it ran inside the window, beside the loop's own
//                turnaround; isr is the handler alone, so busy - isr is
//                what a turn costs besides it
//   m  memcpy and memset of the runtime (rt/rt.cpp), each the best of 8
//      runs on a Stopwatch, between static word-aligned buffers (the
//      runtime's word path). THE RAM BOUNDS THE SIZES: two buffers of the
//      larger size would be the part's whole SRAM, so
//        memcpy        RAM to RAM, 1, 16, 256 and half the buffer, between
//                      its two halves
//        memcpy.flash  the 4096-byte string of letter p, in flash, into
//                      the buffer: 1, 16, 256 and the buffer
//        memset        the buffer: 1, 16, 256 and the buffer
//      Nothing idles and nothing is expected to interrupt: busy is wall,
//      and irq and isr are whatever the tick took in the best run. At 1
//      and 16 bytes a line is mostly the call and the stopwatch, not the
//      copy: x is large there by construction.
//   p  print(serial, s) of a static string of 1, 16, 256 and 4096 bytes
//      (62 letters and a CR LF to a line of 64), then the DRAIN: the
//      transport's tx_idle() (its ring empty) and then the resource's TC
//      flag (Usart<1>::tx_complete(): the last frame has left the
//      shifter) - the transport has no verb for the second half. The
//      counters are read before the print and after the drain. The print
//      spins on a full ring, so busy is close to wall; irq and isr are the
//      transport's per-byte shape - one TXE interrupt per byte, one more
//      that disarms TXEIE - and the ticks.
//   t  THE TICK'S FLOOR: one second of BenchIdle::idle() turns with
//      nothing to do (this loop, not the kernel's), counters before and
//      after: wall = the second in cycles, irq = the ticks, isr = the tick
//      handler's cycles, busy = the floor.
//   d  THE DMA (ch32v00x/dma.hpp, docs/ch32v00x/dma.md). Every line runs
//      from the block's launch to its COMPLETION INTERRUPT - the loop
//      idles through BenchIdle until the handler has said so - so wall
//      holds the launch, the controller's own time, the vector and the
//      idle turn's wake, and busy what the core spent of it.
//        copy        DmaCopyEngine on channel 1, word beats, RAM to RAM
//                    between the buffer's two halves: 16, 256 and half
//                    the buffer - the best of 8
//        copy.flash  a string as long as the buffer, in flash, into it:
//                    16, 256 and the buffer (on the CH32V006 letter p's
//                    own 4096 bytes)
//        fill        the buffer from one word cell: 16, 256 and the buffer
//                    After each op a note line: the controller's cycles
//                    an item, from the difference of the two larger sizes,
//                    and the FIXED COST a block (wall at 16 bytes less
//                    that slope's share), which is the engine's number.
//        paced       DmaTxEngine<5, uint32_t> pouring 256 words of the
//                    buffer into ONE word cell at TIM1's update request
//                    (table 8-2: channel 5), TIM1 at 10 kHz: irq, isr and
//                    busy of a block the CPU only launches and is told
//                    the end of. Then the same block with the core polling
//                    CNTR and stamping every decrement on the ruler: the
//                    intervals' least and greatest against the period, a
//                    note line - the pace's jitter as a polling core sees
//                    it, its resolution the poll turn's.
//        spi.dma     SpiHost<1> on DmaTxEngine<3, uint16_t> and
//                    DmaRxEngine<2, uint16_t>, a WRITE (the receive side
//                    discarding), no chip select, MISO left floating: the
//                    time is the wire's and nothing needs to be judged.
//                    16 and 256 frames of 8 bits at SCK = HCLK/4 (12 MHz)
//                    and HCLK/16 (3 MHz), the best of 4; spi.dma16 the
//                    same frame counts of 16 bits, the half-word beat.
//                    A note line per rate: the cycles a byte above the
//                    wire's, and THE FIXED COST A TRANSACTION (wall less
//                    the wire's time), the round's number.
//
// THE WIRE, and why it is the limit:
//   memcpy, memset  the core's load/store floor. The QingKe V2 manual
//                   gives no instruction timing (table 1-1 states a
//                   two-stage pipeline and nothing per instruction), and
//                   the reference manual (RM 1.1) a system bus from the
//                   core through the bus matrix to the SRAM with no wait
//                   state named; the floor taken is that bus's own, one
//                   32-bit SRAM access per HCLK cycle: memset 4 bytes a
//                   cycle (a store a word), memcpy 2 (a load and a store a
//                   word) - 192 and 96 MB/s at 48 MHz. It is the bus's
//                   figure and not a documented instruction timing: x
//                   says what the pipeline, the loop and the instruction
//                   fetch from flash at two wait states cost above it.
//   memcpy.flash    a word read through the D-Code bus at one cycle and
//                   two wait states (RM 18.3.1: LATENCY = 2 above 24 MHz)
//                   and a word store: 4 bytes in 4 cycles, 48 MB/s.
//   print           the console's ACTUAL baud, read back from BRR (48 MHz
//                   over 417, 115107 baud), over the ten bits of an 8N1
//                   frame: 11510 B/s.
//   copy, fill      ONE ITEM PER HCLK CYCLE, the bus the controller sits on
//                   (RM 1.1: the DMA a master of the same matrix as the
//                   core, at HCLK): 4 bytes a cycle for word beats, 192
//                   MB/s. The chapter states no cycles per item; the
//                   suite measured about six an item (docs/ch32v00x/
//                   dma.md: a read, a write and the arbitration), so x is
//                   about six at the large sizes by the controller's own
//                   cost, and the note line separates it from the block's.
//   paced           the pace: 4 bytes a request at 10 kHz, 40000 B/s.
//   spi.dma         SCK over eight bits a byte: 1.5 MB/s at HCLK/4, 375
//                   kB/s at HCLK/16 - for both frame sizes, a 16-bit frame
//                   being two bytes in sixteen clocks.
//   spi.poll, spi.pump, spi.req  the same wire, at the line's own rate.
//
//   e  THE SPI HOST ABOVE THE WIRE (ch32v00x/spi.hpp, docs/ch32v00x/
//      spi.md): SpiHost<1> with NO engines, on the default pads, MISO
//      floating, the select on PC3 and the D/C line on PC4 - two pads
//      of the board that no other letter and no jumper uses. Every
//      request is built by hand and handed to start(): no arbiter, so
//      what the lines hold is the host's own cost and nothing of the
//      kernel's.
//        spi.poll      a POLLED write of 16 and 256 frames at HCLK/4 and
//                      HCLK/16, the out buffer the work buffer and no in
//                      buffer (the frames read back are discarded, as
//                      letter d's are); spi.poll16 the same in 16-bit
//                      frames. The best of 4 on a Stopwatch around
//                      start(): wall against the wire's time, the gap
//                      above it the loop's own.
//        spi.poll.rx   the same with an IN buffer (the upper half of the
//                      work buffer): the receive shape of the loop, which
//                      must read every frame back and is the one the
//                      arithmetic of the host's threshold bears on;
//                      spi.poll16.rx in 16-bit frames.
//        spi.pump      the same sizes and widths ON THE INTERRUPT, an in
//                      buffer present, the loop idling through BenchIdle
//                      until the vector reports the end: irq is the
//                      handlers taken (one per frame on this silicon: no
//                      FIFO), isr their cycles, wall against the wire's.
//                      At HCLK/4 and HCLK/16, and at HCLK/64 - the rate
//                      from which the host keeps TWO frames in flight on
//                      8-bit frames (HCLK/32 on 16-bit ones), by the
//                      threshold its header computes - so that both
//                      regimes are on the page. After every run a note
//                      line prints the host's status and the overrun
//                      flag read off STATR: an overrun is the one thing
//                      the write-ahead can cost, and the recovery
//                      session reads it here.
//        spi.req       THE FIXED COST OF A REQUEST, the number a DCS
//                      command pays: a polled request of 1, 3 and 16
//                      bytes - a one-byte command (D/C low) and 0, 2 and
//                      15 data bytes (D/C high) - at HCLK/16 with the
//                      select and the D/C line scripted on PC3 and PC4,
//                      the best of 8; the note line gives wall less the
//                      wire's cycles.
//      The vendor's own loop (the EVT's 2Lines_FullDuplex example, the
//      Host side) is NOT in this app: it is a scratch program of the
//      round, counted in its listing beside this host's.
//
// THE INSTRUMENT'S COST, and how the reader subtracts it. Every line
// carries RAW numbers. A Stopwatch measurement holds the stopwatch line's
// wall - start()'s read's tail and elapsed()'s read's head, both spelled
// inline (util/bench.hpp forces them, as it forces the stamps): subtract
// it from every line of m, p and t. A handler's metered cycles
// hold about one ruler read as well (enter()'s tail and leave()'s head):
// subtract irq times the ruler line from isr. The stamp line is what the
// pair adds to the program's real cost per interrupt. And util/bench.hpp's
// two stated seams: the latency of entering and leaving a handler is
// counted as idle, and an interrupt landing between P::idle()'s return
// and the window's close is counted in both.
//
// build: boards = v006k8,v003f4
// build: groups = rt,m,p,d,e
// build: monitor_speed = 115200

#include <stdint.h>
#include <string.h>

#include <cstddef>

#include <span>

#include "ch32v00x/clock.hpp"
#include "ch32v00x/delay.hpp"
#include "ch32v00x/dma.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/spi.hpp"
#include "ch32v00x/ticker.hpp"
#include "ch32v00x/tim.hpp"
#include "ch32v00x/usart.hpp"
#include "util/bench.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 48'000'000>;
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

using Plat = BenchIdle<Ch32v00xPlatform<>, Ruler>;
using Meter = IsrMeter<Ruler, Plat>;

using Serial = Uart<1, Plat>;   // USART1 on PD5/PD6, rings 64/64
constexpr Serial serial;
constexpr uint32_t console_baud = 115200;

TestBench<Serial> bench;

Meter tick_meter;    // the STK's vector
Meter usart_meter;   // USART1's vector
Meter copy_meter;    // DMA channel 1: the copy engine
Meter spi_meter;     // DMA channels 2 and 3 and SPI1: the SPI host
Meter paced_meter;   // DMA channel 5: the paced block
Meter probe_meter;   // bound to nothing: letter r's stamp line

/// The counters of the bound meters and the idle adapter. Read
/// quiescent, before an operation starts and after it has completed;
/// this core moves an aligned word in one access (atomic_width 4), so no
/// guard is wanted.
BenchCounters counters() {
    return bench_counters<Plat>(tick_meter, usart_meter, copy_meter, spi_meter, paced_meter);
}

/// The console silent: the ring empty, then the last frame out of the
/// shifter (the resource's TC; the transport has no verb for that half).
void console_drain() {
    while (!Serial::tx_idle()) {
    }
    while (!Serial::Resource::tx_complete()) {
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
// The strings letter p prints and letter m copies from flash: 62 letters
// and a CR LF to a line of 64, NUL-terminated, word-aligned so that the
// runtime's copy takes its word path from them.
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
template <uint8_t n>
[[gnu::always_inline]] inline void stamp_pairs() {
    if constexpr (n > 0u) {
        probe_meter.enter();
        probe_meter.leave();
        __asm__ volatile("" ::: "memory");
        stamp_pairs<n - 1u>();
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
    constexpr uint32_t ms = Ruler::hz() / 1000u;

    console_drain();
    const bool refused = !delay_us(clock, 1000);
    Stopwatch<Ruler> sw;
    sw.start();
    const bool first = delay_us(clock, 500);
    const bool second = delay_us(clock, 500);
    const uint32_t took = sw.elapsed();
    print(serial, "  delay_us(clock, 1000) ", refused ? "refused" : "SERVED",
          " (a wait of one tick period is, ch32v00x/delay.hpp): 2 x delay_us(500) = ", took,
          " cycles on the ruler, ", ms, " asked", crlf);
    bench.verdict("2 x 500 us on the ruler is at least hz/1000 cycles (at least, never early)",
                  first && second && took >= ms);
    bench.verdict("and under hz/1000 + 5 per cent", took < ms + ms / 20u);

    console_drain();
    const BenchSample reads = quiet_batches([] { ruler_reads<10>(); });
    print(serial, "  ruler: 1000 reads, ten batches of 100 each from a fresh tick, ", reads.irq,
          " interrupts in the run", crlf);
    bench_line(serial, "ruler", 0, per_unit(reads, 1000u), Ruler::hz(), 0);

    console_drain();
    BenchSample empty{0xFFFF'FFFFu, 0, 0, 0};
    for (uint8_t run = 0; run < 8u; ++run) {
        fresh_tick();
        const BenchCounters b = counters();
        sw.start();
        const uint32_t w = sw.elapsed();
        const BenchSample s = bench_sample(w, b, counters());
        if (s.wall < empty.wall) {
            empty = s;
        }
    }
    print(serial, "  stopwatch: start() then elapsed() with nothing between, the best of 8", crlf);
    bench_line(serial, "stopwatch", 0, empty, Ruler::hz(), 0);

    console_drain();
    const BenchSample pairs = quiet_batches([] { stamp_pairs<10>(); });
    print(serial, "  stamp: 1000 enter()+leave() pairs, batched the same, ", pairs.irq,
          " interrupts in the run", crlf);
    bench_line(serial, "stamp", 0, per_unit(pairs, 1000u), Ruler::hz(), 0);

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
/// Half the part's SRAM: 4096 bytes on the CH32V006, 1024 on the CH32V003.
constexpr uint32_t ram_bytes = device::sram_bytes / 2u;
alignas(4) uint8_t ram[ram_bytes];

enum class MemOp : uint8_t { copy_ram, copy_flash, fill };

/// The best of 8 runs of one operation. The length and the pointers pass
/// through an empty asm so that the compiler sees neither as a constant:
/// a small constant memcpy would be expanded inline, and this letter
/// measures the runtime's function.
template <MemOp op>
BenchSample mem_best(uint32_t n) {
    BenchSample best{0xFFFF'FFFFu, 0, 0, 0};
    for (uint8_t run = 0; run < 8u; ++run) {
        uint8_t* dst = op == MemOp::copy_ram ? ram + ram_bytes / 2u : ram;
        const uint8_t* src = op == MemOp::copy_ram
                                 ? ram
                                 : reinterpret_cast<const uint8_t*>(filler<4096>.text);
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

constexpr uint32_t mem_sizes[4] = {1, 16, 256, ram_bytes};

// The floors of the header: bytes per HCLK cycle times the rate.
constexpr uint32_t wire_copy_ram = 2u * Ruler::hz();
constexpr uint32_t wire_copy_flash = 1u * Ruler::hz();
constexpr uint32_t wire_fill = 4u * Ruler::hz();

void tm_memory() {
    console_drain();
    for (const uint32_t n : mem_sizes) {
        const uint32_t half = n > ram_bytes / 2u ? ram_bytes / 2u : n;   // the RAM's bound: two halves
        bench_line(serial, "memcpy", half, mem_best<MemOp::copy_ram>(half), Ruler::hz(),
                   wire_copy_ram);
    }
    for (const uint32_t n : mem_sizes) {
        bench_line(serial, "memcpy.flash", n, mem_best<MemOp::copy_flash>(n), Ruler::hz(),
                   wire_copy_flash);
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
    const uint32_t wire_bps = Serial::actual_baud(SysClock::pclk_hz) / 10u;
    print(serial, "  console at ", Serial::actual_baud(SysClock::pclk_hz), " baud, 8N1: ", wire_bps,
          " B/s", crlf);
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
// d - the DMA: copy and fill, a paced block, the SPI's engines
// ---------------------------------------------------------------------------
using Copier = DmaCopyEngine<1>;               // word beats at most
using Paced = DmaTxEngine<5, uint32_t>;        // TIM1's update request (table 8-2)
using PaceTimer = Tim<1>;
using SpiDma = SpiHost<1, spi1_default_pins, DmaTxEngine<3, uint16_t>, DmaRxEngine<2, uint16_t>>;

constexpr uint32_t pace_hz = 10'000;
constexpr uint32_t pace_period = SysClock::hz / pace_hz;   // 4800 HCLK cycles

volatile bool copy_done = false;
volatile bool paced_done = false;
volatile bool spi_done = false;
bool dma_ready = false;   // letter d's peripherals brought up once

/// The one word cell the paced block pours into, and the one the fill
/// reads from.
volatile uint32_t pace_cell = 0;
const uint32_t fill_cell = 0xA5A5'5A5Au;

/// Launch, then idle until the completion handler sets `done`: the
/// sample a block makes from its start to its interrupt.
template <typename Launch>
BenchSample dma_sample(volatile bool& done, Launch launch) {
    done = false;
    const BenchCounters before = counters();
    Stopwatch<Ruler> sw;
    sw.start();
    launch();
    for (;;) {
        Plat::CriticalSection cs;
        if (done) {
            break;
        }
        Plat::idle();
    }
    const uint32_t wall = sw.elapsed();
    return bench_sample(wall, before, counters());
}

/// The best (least wall) of `runs` samples.
template <typename Launch>
BenchSample dma_best(uint8_t runs, volatile bool& done, Launch launch) {
    BenchSample best{0xFFFF'FFFFu, 0, 0, 0};
    for (uint8_t run = 0; run < runs; ++run) {
        const BenchSample s = dma_sample(done, launch);
        if (s.wall < best.wall) {
            best = s;
        }
    }
    return best;
}

/// The two numbers a block line leaves to the reader: the controller's
/// cycles an item (tenths), from the difference of two sizes, and the
/// fixed cost a block at the smallest - wall less that slope's share.
void block_note(const char* op, uint32_t n_small, uint32_t wall_small, uint32_t n_mid, uint32_t wall_mid,
                uint32_t n_big, uint32_t wall_big) {
    const uint32_t items_small = n_small / 4u;
    const uint32_t items_mid = n_mid / 4u;
    const uint32_t items_big = n_big / 4u;
    const uint32_t tenths = (wall_big - wall_mid) * 10u / (items_big - items_mid);
    const uint32_t share = tenths * items_small / 10u;
    const uint32_t fixed = wall_small > share ? wall_small - share : 0u;
    print(serial, "  ", op, ": ", tenths / 10u, '.', tenths % 10u, " cycles an item of 4 bytes (", n_mid,
          " and ", n_big, " bytes), fixed ", fixed, " cycles a block", crlf);
}

/// copy / copy.flash / fill at 16, 256 and `big` bytes, word beats.
enum class BlockOp : uint8_t { copy, copy_flash, fill };

template <BlockOp op>
BenchSample block_run(uint32_t n) {
    const uint32_t words = n / 4u;
    return dma_best(8, copy_done, [words] {
        uint32_t* const base = reinterpret_cast<uint32_t*>(ram);
        if constexpr (op == BlockOp::copy) {
            (void)Copier::copy(base + ram_bytes / 8u, base, words);
        } else if constexpr (op == BlockOp::copy_flash) {
            (void)Copier::copy(base, reinterpret_cast<const uint32_t*>(filler<ram_bytes>.text), words);
        } else {
            (void)Copier::fill(base, &fill_cell, words);
        }
    });
}

template <BlockOp op>
void block_lines(const char* name, uint32_t big) {
    constexpr uint32_t wire = 4u * Ruler::hz();   // one word item a cycle
    const BenchSample s16 = block_run<op>(16);
    const BenchSample s256 = block_run<op>(256);
    const BenchSample sbig = block_run<op>(big);
    bench_line(serial, name, 16, s16, Ruler::hz(), wire);
    bench_line(serial, name, 256, s256, Ruler::hz(), wire);
    bench_line(serial, name, big, sbig, Ruler::hz(), wire);
    block_note(name, 16, s16.wall, 256, s256.wall, big, sbig.wall);
}

/// TIM1 at the pace, its update request armed: the paced block's clock.
void pace_start() {
    PaceTimer::init();
    (void)PaceTimer::configure({.prescaler = 0, .period = static_cast<uint16_t>(pace_period - 1u)});
    PaceTimer::interrupts(PaceTimer::update_dma, true);
    PaceTimer::enable(true);
}

void pace_stop() {
    PaceTimer::enable(false);
    PaceTimer::interrupts(PaceTimer::update_dma, false);
}

constexpr uint16_t paced_words = 256;

void paced_lines() {
    const BenchSample s = dma_sample(paced_done, [] {
        (void)Paced::start(std::span<const uint32_t>(reinterpret_cast<const uint32_t*>(ram), paced_words));
        pace_start();
    });
    pace_stop();
    bench_line(serial, "paced", 4u * paced_words, s, Ruler::hz(), 4u * pace_hz);

    // The jitter: the same block with the core polling CNTR, every
    // decrement stamped. Masked, so that no tick or completion handler
    // lands in the poll - and so the stamp is the STK's own counter, one
    // period of the tick (CMP + 1 cycles) long, the interval taken modulo
    // it: an interval is a fifth of that period, and the ruler's
    // composition with a tick count would go wrong with the tick held off.
    console_drain();
    uint32_t least = 0xFFFF'FFFFu;
    uint32_t most = 0;
    uint32_t turns = 0;
    {
        (void)Paced::start(std::span<const uint32_t>(reinterpret_cast<const uint32_t*>(ram), paced_words));
        Plat::CriticalSection cs;   // the poll alone: no tick, no completion handler
        const uint32_t tick_period = stk()->CMP + 1u;
        pace_start();
        uint16_t last = paced_words;
        uint32_t at = 0;
        while (last != 0u) {
            const uint16_t now = DmaChannel<5>::count();
            ++turns;
            if (now != last) {
                const uint32_t t = stk()->CNT;
                if (last != paced_words && now == last - 1u) {
                    const uint32_t d = t >= at ? t - at : t + tick_period - at;
                    least = d < least ? d : least;
                    most = d > most ? d : most;
                }
                at = t;
                last = now;
            }
        }
        pace_stop();
        (void)Paced::complete();
        DmaChannel<5>::clear(DmaFlag::all);   // the completion the masked handler never took
    }
    print(serial, "  paced, polled: intervals ", least, "..", most, " cycles against a period of ", pace_period,
          ", jitter ", most - least, " cycles; ", turns, " poll turns over ", paced_words,
          " requests", crlf);
}

/// One SPI write of `frames` frames of `bits`, MISO floating: the line
/// the round is judged by.
BenchSample spi_run(uint16_t frames, SpiClock clock, SpiDataSize bits) {
    return dma_best(4, spi_done, [frames, clock, bits] {
        SpiDma::Request r{};
        r.tx = Borrowed<const uint8_t, Lease::reply>{ram};
        r.len = frames;
        r.clock = clock;
        r.bits = bits;
        (void)SpiDma::start(r);
    });
}

void spi_lines(SpiClock clock, uint8_t div) {
    const uint32_t wire = SysClock::hz / div / 8u;   // bytes a second
    for (const SpiDataSize bits : {SpiDataSize::bits8, SpiDataSize::bits16}) {
        const uint32_t bytes = bits == SpiDataSize::bits16 ? 2u : 1u;
        const char* const op = bits == SpiDataSize::bits16 ? "spi.dma16" : "spi.dma";
        (void)spi_run(16, clock, bits);   // a warm-up: apply() reconfigures on a change
        const BenchSample s16 = spi_run(16, clock, bits);
        const BenchSample s256 = spi_run(256, clock, bits);
        bench_line(serial, op, 16u * bytes, s16, Ruler::hz(), wire);
        bench_line(serial, op, 256u * bytes, s256, Ruler::hz(), wire);
        const uint32_t wire16 = 16u * bytes * 8u * div;   // cycles on the wire
        const uint32_t wire256 = 256u * bytes * 8u * div;
        const uint32_t per_byte = (s256.wall - s16.wall) * 10u / (240u * bytes);
        print(serial, "  ", op, " at HCLK/", div, ": ", per_byte / 10u, '.', per_byte % 10u,
              " cycles a byte against the wire's ", 8u * div, "; fixed ",
              s16.wall > wire16 ? s16.wall - wire16 : 0u, " cycles a transaction at 16 frames, ",
              s256.wall > wire256 ? s256.wall - wire256 : 0u, " at 256", crlf);
    }
}

void td_dma() {
    if (!dma_ready) {
        Copier::arm(DmaPriority::high);
        Paced::arm(&pace_cell);
        (void)SpiDma::init(clock);
        dma_ready = true;
    }
    for (uint32_t i = 0; i < ram_bytes; ++i) {
        ram[i] = static_cast<uint8_t>(i * 7u + 1u);
    }
    console_drain();
    print(serial, "  the buffer: ", ram_bytes, " bytes; every line to the completion interrupt", crlf);
    console_drain();
    block_lines<BlockOp::copy>("copy", ram_bytes / 2u);
    console_drain();
    block_lines<BlockOp::copy_flash>("copy.flash", ram_bytes);
    console_drain();
    block_lines<BlockOp::fill>("fill", ram_bytes);
    print(serial, "  copy faults: ", Copier::faults(), crlf);
    console_drain();
    paced_lines();
    console_drain();
    spi_lines(SpiClock::div4, 4);
    console_drain();
    spi_lines(SpiClock::div16, 16);
    print(serial, "  spi faults: tx ", DmaTxEngine<3, uint16_t>::faults(), " rx ", DmaRxEngine<2, uint16_t>::faults(),
          ", status ", SpiDma::status(), crlf);
    bench.verdict("ran", true);
}

// ---------------------------------------------------------------------------
// e - the SPI host above the wire: the polled loops, the pump, a request
// ---------------------------------------------------------------------------
using SpiPoll = SpiHost<1>;   // no engines: the polled loops and the pump alone
using CsPin = Pin<'C', 3>;    // the select, as test_ch32_spi scripts it
using DcPin = Pin<'C', 4>;    // the D/C line: a pad no jumper and no other letter uses

volatile bool pump_done = false;
/// Which host spi1_handler serves: letter e's or letter d's. Volatile:
/// the vector reads it, and a plain bool's `true` at the letter's start
/// is a dead store to a compiler that sees the `false` at its end.
volatile bool poll_host_live = false;

/// A request on the work buffer: `frames` frames of `bits`, the out
/// buffer the buffer's lower half, the in buffer its upper half or none.
SpiPoll::Request poll_request(uint16_t frames, SpiClock clock, SpiDataSize bits, bool with_rx, bool polled) {
    SpiPoll::Request r{};
    r.tx = Borrowed<const uint8_t, Lease::reply>{ram};
    r.rx = with_rx ? Borrowed<uint8_t, Lease::reply>{ram + ram_bytes / 2u} : Borrowed<uint8_t, Lease::reply>{};
    r.len = frames;
    r.clock = clock;
    r.bits = bits;
    r.polled = polled;
    return r;
}

/// A polled transaction on a Stopwatch around start(), the best of `runs`.
BenchSample poll_best(uint8_t runs, uint16_t frames, SpiClock clock, SpiDataSize bits, bool with_rx) {
    BenchSample best{0xFFFF'FFFFu, 0, 0, 0};
    for (uint8_t run = 0; run < runs; ++run) {
        const SpiPoll::Request r = poll_request(frames, clock, bits, with_rx, true);
        const BenchCounters before = counters();
        Stopwatch<Ruler> sw;
        sw.start();
        (void)SpiPoll::start(r);
        const uint32_t wall = sw.elapsed();
        const BenchSample s = bench_sample(wall, before, counters());
        if (s.wall < best.wall) {
            best = s;
        }
    }
    return best;
}

/// The polled lines of one rate: both widths, both shapes, two sizes.
void poll_lines(SpiClock clock, uint8_t div) {
    const uint32_t wire = SysClock::hz / div / 8u;   // bytes a second
    for (const bool with_rx : {false, true}) {
        for (const SpiDataSize bits : {SpiDataSize::bits8, SpiDataSize::bits16}) {
            const uint32_t bytes = bits == SpiDataSize::bits16 ? 2u : 1u;
            const char* const op = with_rx ? (bits == SpiDataSize::bits16 ? "spi.poll16.rx" : "spi.poll.rx")
                                           : (bits == SpiDataSize::bits16 ? "spi.poll16" : "spi.poll");
            (void)poll_best(1, 16, clock, bits, with_rx);   // a warm-up: apply() reconfigures on a change
            const BenchSample s16 = poll_best(4, 16, clock, bits, with_rx);
            const BenchSample s256 = poll_best(4, 256, clock, bits, with_rx);
            bench_line(serial, op, 16u * bytes, s16, Ruler::hz(), wire);
            bench_line(serial, op, 256u * bytes, s256, Ruler::hz(), wire);
            const uint32_t per_frame = (s256.wall - s16.wall) * 10u / 240u;
            print(serial, "  ", op, " at HCLK/", div, ": ", per_frame / 10u, '.', per_frame % 10u,
                  " cycles a frame against the wire's ", 8u * bytes * div, crlf);
        }
    }
}

/// One pumped transaction, the loop idling until the vector reports the
/// end; the overrun flag and the host's status read after it.
BenchSample pump_run(uint16_t frames, SpiClock clock, SpiDataSize bits) {
    const SpiPoll::Request r = poll_request(frames, clock, bits, true, false);
    return dma_sample(pump_done, [&r] { (void)SpiPoll::start(r); });
}

void pump_lines(SpiClock clock, uint8_t div) {
    const uint32_t wire = SysClock::hz / div / 8u;
    for (const SpiDataSize bits : {SpiDataSize::bits8, SpiDataSize::bits16}) {
        const uint32_t bytes = bits == SpiDataSize::bits16 ? 2u : 1u;
        const char* const op = bits == SpiDataSize::bits16 ? "spi.pump16" : "spi.pump";
        (void)pump_run(16, clock, bits);   // a warm-up
        const BenchSample s16 = pump_run(16, clock, bits);
        const bool ovr16 = Spi<1>::overrun();
        const uint8_t st16 = SpiPoll::status();
        const BenchSample s256 = pump_run(256, clock, bits);
        const bool ovr256 = Spi<1>::overrun();
        const uint8_t st256 = SpiPoll::status();
        bench_line(serial, op, 16u * bytes, s16, Ruler::hz(), wire);
        bench_line(serial, op, 256u * bytes, s256, Ruler::hz(), wire);
        const uint32_t per_frame = (s256.wall - s16.wall) * 10u / 240u;
        print(serial, "  ", op, " at HCLK/", div, ": ", per_frame / 10u, '.', per_frame % 10u,
              " cycles a frame against the wire's ", 8u * bytes * div, "; status ", st16, '/', st256,
              ", overrun ", ovr16 ? '1' : '0', '/', ovr256 ? '1' : '0', crlf);
    }
}

/// A polled request of one command byte and `len` data bytes at HCLK/16,
/// the select and the D/C line scripted: the best of 8.
void request_line(uint16_t len) {
    static const uint8_t command = 0x2C;
    BenchSample best{0xFFFF'FFFFu, 0, 0, 0};
    for (uint8_t run = 0; run < 8u; ++run) {
        SpiPoll::Request r{};
        r.cs = CsPin::ref();
        r.dc = DcPin::ref();
        r.cmd = Borrowed<const uint8_t, Lease::reply>{&command};
        r.cmd_len = 1;
        r.tx = Borrowed<const uint8_t, Lease::reply>{ram};
        r.len = len;
        r.clock = SpiClock::div16;
        r.polled = true;
        const BenchCounters before = counters();
        Stopwatch<Ruler> sw;
        sw.start();
        (void)SpiPoll::start(r);
        const uint32_t wall = sw.elapsed();
        const BenchSample s = bench_sample(wall, before, counters());
        if (s.wall < best.wall) {
            best = s;
        }
    }
    const uint32_t bytes = 1u + len;
    const uint32_t wire_cycles = bytes * 8u * 16u;
    bench_line(serial, "spi.req", bytes, best, Ruler::hz(), SysClock::hz / 16u / 8u);
    print(serial, "  spi.req ", bytes, " bytes: fixed ", best.wall > wire_cycles ? best.wall - wire_cycles : 0u,
          " cycles above the wire's ", wire_cycles, crlf);
}

void te_spi_host() {
    (void)SpiPoll::init(clock);
    CsPin::output(true);
    DcPin::output(true);
    poll_host_live = true;
    dma_ready = false;   // letter d brings its own host up again after this one
    for (uint32_t i = 0; i < ram_bytes; ++i) {
        ram[i] = static_cast<uint8_t>(i * 7u + 1u);
    }
    console_drain();
    print(serial, "  SpiHost<1> with no engines, MISO floating, the select on PC3, D/C on PC4", crlf);
    console_drain();
    poll_lines(SpiClock::div4, 4);
    console_drain();
    poll_lines(SpiClock::div16, 16);
    console_drain();
    pump_lines(SpiClock::div4, 4);
    console_drain();
    pump_lines(SpiClock::div16, 16);
    console_drain();
    pump_lines(SpiClock::div64, 64);
    console_drain();
    request_line(0);
    request_line(2);
    request_line(15);
    poll_host_live = false;
    bench.verdict("ran", true);
}

void banner() {
    print(serial, crlf, "bench_ch32 - ", device::part_name,
          " (clk=48 MHz PLL, ruler=STK cycles at 48 MHz, tick=STK 1000 Hz, console=USART1 ",
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

// Letter d's vectors, empty in an image that does not carry the letter
// (nothing enables their lines there).
extern "C" BRIO_CH32_INTERRUPT void dma1_channel1_handler() {
    if constexpr (brio::test_letter_carried('d')) {
        copy_meter.enter();
        if (Copier::isr() != 0u) {
            copy_done = true;
        }
        copy_meter.leave();
    }
}

// The SPI host's two channels share one body (dma_isr() serves both),
// spelled once and called from both vectors: two inline copies of it
// would cost the CH32V003's image a quarter of a kilobyte.
namespace {
[[gnu::noinline]] void spi_dma_vector() {
    spi_meter.enter();
    if (SpiDma::dma_isr()) {
        spi_done = true;
    }
    spi_meter.leave();
}
} // namespace

extern "C" BRIO_CH32_INTERRUPT void dma1_channel2_handler() {
    if constexpr (brio::test_letter_carried('d')) {
        spi_dma_vector();
    }
}

extern "C" BRIO_CH32_INTERRUPT void dma1_channel3_handler() {
    if constexpr (brio::test_letter_carried('d')) {
        spi_dma_vector();
    }
}

// SPI1's vector serves letter e's engineless host while that letter runs
// and letter d's engined one otherwise; an image carrying neither leaves
// it empty.
extern "C" BRIO_CH32_INTERRUPT void spi1_handler() {
    spi_meter.enter();
    if constexpr (brio::test_letter_carried('e')) {
        if (poll_host_live) {
            if (SpiPoll::isr()) {
                pump_done = true;
            }
            spi_meter.leave();
            return;
        }
    }
    if constexpr (brio::test_letter_carried('d')) {
        if (SpiDma::isr()) {
            spi_done = true;
        }
    }
    spi_meter.leave();
}

extern "C" BRIO_CH32_INTERRUPT void dma1_channel5_handler() {
    if constexpr (brio::test_letter_carried('d')) {
        paced_meter.enter();
        const uint8_t f = Paced::service();
        if (f != 0u) {
            (void)Paced::complete();
            paced_done = true;
        }
        paced_meter.leave();
    }
}

int main() {
    const bool clock_ok = SysClock::init();            // HSI x2 -> 48 MHz
    const bool serial_ok = Serial::init(clock, console_baud);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    bench.letter('r', "the ruler's self-check and the instrument's cost", tr_ruler);
    bench.letter('m', "memcpy and memset of the runtime against the core's floor", tm_memory);
    bench.letter('p', "a print through the console against the wire", tp_print);
    bench.letter('t', "the tick's floor: one second of idle turns", tt_tick);
    bench.letter('d', "the DMA: copy, fill, a paced block, the SPI's engines", td_dma);
    bench.letter('e', "the SPI host above the wire: polled, pumped, a request's fixed cost", te_spi_host);

    // Guarded: print() BLOCKS until the transport accepts each byte, so
    // printing into a port that failed to come up would never return.
    if (!serial_ok) {
        for (;;) {
        }
    }
    brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL48" : "FAILED", " tick=",
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
