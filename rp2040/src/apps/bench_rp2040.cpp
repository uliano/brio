// bench_rp2040 - the benchmark skeleton on the RP2040 (docs/design/
// benchmark.md, util/bench.hpp): four letters that print NUMBERS, one
// `bench` line per operation and size, in the grammar every family
// prints. NOT A TEST: a letter's one verdict is "ran", except letter r,
// whose verdicts judge the ruler every other line is read with.
//
// NOTHING TO WIRE. The console is the Debug Probe's UART bridge on GP0
// (TX) / GP1 (RX) = UART0 under function 2, 115200 8N1 - the binding of
// console.cpp and test_rp2040_platform.cpp, and the TestBench frame and
// polled prompt loop of the latter. Letter d drives SPI0's pads (GP16,
// GP18, GP19) on its internal loop-back: whatever hangs on them is
// clocked at but never read. No kernel and no AO: the letters
// are plain functions and the idle path is called by hand, so what is
// measured is the transport, the runtime and the idle path, never a
// dispatch.
//
// THE CLOCK: the 12 MHz crystal through the PLL to 125 MHz, clk_peri =
// clk_sys (rp2040/clock.hpp), the rate test_rp2040_platform runs at.
//
// THE RULER is `Ruler` below: core 0's SysTick ticker read as cycles
// (cortexm/ticker.hpp's BasicTicker::cycles(), the tick count and
// SysTick's position in its period composed by util/cycle_count.hpp),
// so it counts clk_sys cycles and hz() is SysClock::hz. Its now() is
// always_inline AND flatten, so the whole read - six register and
// memory loads (LOAD, the count, ICSR, VAL, ICSR, the count) and the
// compose - lands inline in every vector and in every loop that stamps,
// with no call. (flatten is the release build's: the debug preset's
// -fno-inline switches it off and the vectors there call cycles().)
//
// THE PLATFORM the idle path runs on is brio::BenchIdle<Rp2040Platform<>,
// Ruler> (`Idle`): every idle turn is a masked call of Idle::idle(),
// which stamps the window around the platform's WFI.
//
// THE METERS: one IsrMeter per bound vector - UART0's (`uart_meter`, on
// Ruler), DMA_IRQ_0's (`dma_meter`) and SPI0's (`spi_meter`, which a
// request the engines leave to the pump would run), and SysTick's
// (`tick_meter`). The SysTick vector's meter runs on
// `TickRuler`, SysTick's position in its period, and NOT on Ruler: inside
// the SysTick handler, before Ticker::tick() has counted the tick, the
// exception is ACTIVE and no longer pending (ICSR.PENDSTSET reads 0) while
// the count still holds the previous tick - so cycles() reads one whole
// period behind there and right again after the increment, and a meter
// on it would charge every tick a period (125 000 cycles). Both stamps of
// the tick vector fall inside one period (the handler runs right after
// the reload that pended it and lasts some hundred cycles; no masked
// section of this app approaches a period), so the position alone gives
// their difference exactly - with one load where Ruler takes six.
//
// What each letter prints:
//
//   r  THE RULER'S SELF-CHECK. delay_us(clock, 999) read on the ruler:
//      verdicts "at least hz x 999 us" and "under that + 5 per cent", and
//      the same wait read on the system timer (rp2040/timer.hpp, 1 us
//      ticks from the crystal, independent of clk_sys) as a third
//      verdict, because the ruler and delay_us count the SAME SysTick
//      and so cannot catch a wrong clk_sys between them. 999 and not
//      1000: cortexm/delay.hpp refuses a wait of one SysTick period or
//      more, and a period is one millisecond here (the line printed
//      shows the refusal). Then the instrument's cost, four bench lines
//      with n=0 and wire=0. Three are the AVERAGE of the run named,
//      every field divided by the units and rounded to the nearest (the
//      raw totals printed on the line above):
//        ruler     one Ruler::now(), over 1000 reads in a loop (the
//                  loop's increment, test and branch and a volatile store
//                  of the value are in it - about six cycles);
//        stamp     one enter()+leave() pair of a namespace-scope IsrMeter
//                  on Ruler with nothing between, over 1000 pairs, a
//                  compiler barrier between pairs so each pair loads and
//                  stores the meter's sums as a handler does: irq=1, isr =
//                  what the meter charges an EMPTY body, the floor under
//                  UART0's isr (the tick meter's floor is one VAL load and
//                  a subtract, a few cycles);
//        window    one idle turn (the masked call, the window's two
//                  stamps, the WFI, the tick that ends it, the loop's
//                  test on the Stopwatch) over the turns of 100 ms: wall
//                  is one tick period, irq=1, isr is the TICK HANDLER's
//                  cycles - it ran inside the window, so busy holds it -
//                  and busy - isr is the turn's own cost;
//      and one is the BEST of 8, as letter m's lines are:
//        stopwatch an empty Stopwatch interval, start() then elapsed()
//                  with nothing between: the floor every Stopwatch wall
//                  of letters m, p and t holds beyond its operation, the
//                  tail of one ruler read and the head of the next.
//   m  memcpy and memset (brio/rt/rt.cpp, the word path) of 1, 16, 256
//      and 4096 bytes between two word-aligned static buffers in the
//      striped SRAM, each the BEST of 8 runs on a Stopwatch, the length
//      read from a volatile so the call is the runtime's and not an
//      inlined copy. No idle and no interrupt expected: busy = wall,
//      irq and isr whatever tick landed in the best run.
//   p  a print of 1, 16, 256 and 4096 bytes - print(serial, s) with s a
//      static string in flash of that length (rows of 62 digits and a
//      CRLF; the 1-byte one is the final LF) - then a spin on
//      Serial::tx_idle(), the transport's own "ring empty and the
//      shifter finished" (UARTFR.BUSY clear, which stands until the last
//      stop bit has left the shift register). Counters before the print
//      and after the drain. busy = wall (the print and the drain spin);
//      irq and isr are the transport's shape: from an idle transmitter
//      the first FIFO-depth bytes go straight into the FIFO, then one
//      handler per refill as the FIFO falls through its level; a byte the
//      full 256-byte ring refuses writes nothing (pl011/uart.hpp).
//   t  the tick's floor: one second (hz cycles on the ruler) of masked
//      Idle::idle() turns with the console drained - not a kernel loop:
//      no kernel runs here. wall = the second, irq = the ticks, isr = the
//      tick handler's cycles, busy = the floor. n=0, wire=0.
//   d  THE DMA ENGINES (rp2040/dma.hpp), nothing on a wire. Every block
//      completes on DMA_IRQ_0 (`dma_meter`) while the core idles in
//      masked Idle::idle() turns, so busy is the launch, the handler and
//      the idle turns, and wall runs from the launch to the loop seeing
//      the handler's flag:
//        copy     16, 256 and 4096 bytes between the two buffers of
//                 letter m by DmaCopyEngine<8>, as words (n/4 beats),
//                 the BEST of 8. The fixed cost is wall - n/4: the
//                 controller moves one beat a clk_sys cycle (below).
//        fill     the same sizes from ONE word cell, the read address
//                 held.
//        paced    256 words from a table into ONE word cell by
//                 DmaTxEngine<9, uint32_t> on DmaTimer<0> at X/Y = 1/400
//                 (a request every 400 clk_sys cycles, 312 500 a second),
//                 started on a fresh SysTick period so the block's
//                 102 400 cycles hold no tick: irq is the one completion.
//                 Then THE JITTER: the same block with the core polling
//                 TRANS_COUNT and stamping every decrement on SysTick's
//                 VAL (one load, exact inside its period): the shortest
//                 and the longest interval between two transfers against
//                 the 400 the timer is set to, the stamp's own resolution
//                 being one turn of the poll (printed).
//        spi.dma  SPI0 (GP18 SCK, GP19 TX, GP16 RX, no select) on its
//                 LOOP-BACK - the PL022's LBM, test_rp2040_spi's wireless
//                 instrument: nothing is clocked onto a wire that needs a
//                 peer, and the frames come back so each run is judged
//                 exact - with DmaTxEngine<4, uint16_t> / DmaRxEngine<5,
//                 uint16_t>, SSPDR's width, so both frame sizes ride them:
//                 16 and 256 8-bit frames at div4 (31.25 MHz) and div32
//                 (3.906 MHz), then 16, 256 and 4096 16-bit frames at
//                 div4, each the best of 4. After each group, THE NUMBER
//                 OF THE ROUND: the fixed cost of a transaction (the
//                 16-frame wall less its wire time) and the time a byte
//                 (the 16- and 256-frame walls' difference over the bytes
//                 of the 240 frames between them) against the wire's.
//   e  THE SPI HOST ABOVE THE WIRE (brio/pl022/spi.hpp over SPI0 on the
//      same pads, GP17 the select and GP22 the D/C line, nothing on the
//      wire), on a host WITHOUT engines so the polled loop and the pump
//      are what runs - bench_rp2350's letter e on this chip, the same ops
//      under the same names: spi.poll (a polled WRITE, no in buffer) and
//      spi.poll.rx (the receive) of 16 and 256 frames at clk_peri / 2, / 4
//      and / 16 in mode 0 and in mode 3, spi.poll16 and spi.poll16.rx the
//      same in 16-bit frames, spi.pump and spi.pump16 the receives on the
//      interrupt, the receive shape's proof under a tick handler stretched
//      past eight frame times, and spi.req - a command byte and 0, 2 and
//      15 data bytes with D/C scripted at clk_peri / 16, wall less the
//      wire's cycles the host's fixed cost per request, and spi.cold - the
//      three-byte request with its code COLD: the host's very first
//      request after init(), then the same request with every line of the
//      XIP cache invalidated first (Xip::flush(): the repeatable reading),
//      each with its warm repeat and the cache's misses beside it, run
//      first in the letter so that the first request is the first. The
//      block's receive-overrun flag is read after every run. Each the best
//      of five after one thrown away, spi.cold apart. Its readings are
//      docs/rp2040/spi.md's.
//
//   u  THE SERIAL TRANSPORT ON A LOOP OF ITS OWN (pl011/uart.hpp's
//      Pl011Transport): UART1 on GP4/GP5 under UARTCR.LBE, its vector
//      metered apart (`loop_meter`), DMA channels 2 and 3 for its
//      engines. uart.tx: 256 and 4096 bytes through write_bulk() at
//      115200, 1 Mbaud and UARTCLK / 16, the receiver's two interrupts
//      masked through the resource, through the interrupt transmitter
//      and the transmit engine, to the last stop bit (BUSY falling);
//      busy is the transport's share, the thread's wait being a spin.
//      uart.rx: bursts of 16 and 256, the transmitter on its engine so
//      UART1's vector is the receiver's, through the interrupt receiver
//      and the receive engine (harvest() publishing the engine's run as
//      the ring is read). uart.edge: a burst of 17 with the consumer
//      draining on every edge, the last edge against BUSY falling,
//      signed; the engine's edge is harvest() asked from the tick, its
//      owner's poll. Then the receive time-out probed under the engine
//      (UARTRIS read raw after ten frames of silence), and the copy into
//      the ring: write_bulk's byte loop against the runtime's memcpy.
//   i  THE I2C HOST ABOVE THE WIRE (dw_apb_i2c/i2c.hpp over rp2040/i2c.hpp)
//      on the SELF-LINK the I2C suite uses: GP12 (I2C0 SDA) to GP14 (I2C1
//      SDA), GP13 (I2C0 SCL) to GP15 (I2C1 SCL), 4.7 kOhm to 3V3 on each.
//      I2C0 is the host; I2C1 answers 0x42 from its own vector - its
//      RX_TL raised to a level of eight and a read answered with as many
//      bytes as its FIFO takes per read request - and is an INSTRUMENT:
//      its interrupts (`peer_meter`) are printed beside each line and kept
//      out of busy, irq and isr, which are the host's vector and the DMA
//      line's. 0x43 is nobody. At 100 kHz, 400 kHz and 1 MHz: i2c.write of
//      1, 2, 16 and 255 bytes, i2c.read of the same, i2c.wr (one byte
//      written, a repeated START, 1, 2 and 16 read), i2c.probe (an ACK -
//      a one-byte read on this block) and i2c.probe.nack, i2c.write.readdr
//      (a one-byte write right after a probe of 0x43, so its start() finds
//      another address in the block), then on the engined host
//      (DmaTxEngine<6, uint16_t>, DmaRxEngine<7>) i2c.read.dma of 16 and
//      255 and i2c.wr.dma of 1 + 16. Each the best of five after one
//      thrown away, every run judged byte-exact against what the client
//      heard or gave. Under each line: fixed = wall less the wire's time,
//      start = the thread inside start(), the client's interrupts and
//      cycles, the XIP cache's misses over the tenure and the longest
//      host entry. BOTH I2C VECTORS RUN FROM SRAM, flattened into
//      .ram_text, so that a line reads the code and not the XIP cache's
//      state at that instant: from flash the one-STOP handler measured
//      188 to 365 cycles on the RP2350's Cortex-M33 across builds, with up
//      to six cache misses of about seventy cycles each a tenure, against
//      84 from SRAM.
//
// THE WIRES (the `wire` field, bytes per second, and why it is the
// limit):
//   print   115200 baud over the ten bits of an 8N1 frame (a start bit,
//           eight data bits, a stop bit): 11 520 B/s. The divisor gives
//           115 207 baud at 125 MHz (IBRD 67, FBRD 52, 4.2.7.1); the
//           configured rate is the figure.
//   memcpy  2 bytes a cycle x hz = 250 000 000 B/s. The Cortex-M0+ has
//           one AHB-Lite master port for every access below 0xd0000000
//           (datasheet 2.4.3.4), and table 81 of 2.4.3.3 gives LDM and
//           STM 1+N cycles for N words: a word copied is at least one
//           beat in and one beat out, the +1 per instruction and the
//           loop being the implementation's. The SRAM answers each beat
//           in one cycle (2.1.1.1, 2.6.2: zero wait states, word-striped
//           banks).
//   memset  4 bytes a cycle x hz = 500 000 000 B/s: one STM beat per
//           word (the same table), nothing loaded.
//   r and t wire=0: an instrument's cost and an idle second carry no
//           bytes.
//   copy, fill  one beat a clk_sys cycle, four bytes a beat for a word:
//           4 x hz = 500 000 000 B/s. Datasheet 2.5: the DMA "can perform
//           one read access and one write access, up to 32 bits in size,
//           every clock cycle"; the two buffers sit in the striped SRAM
//           (2.6.2), so the read and the write of one beat fall in
//           different banks and the fabric serves both in the same cycle.
//   paced   the timer's pace: 4 bytes x hz / 400 = 1 250 000 B/s.
//   spi.dma the clock over the bits of a byte: 31.25 MHz / 8 = 3 906 250
//           B/s at div4, 3.906 MHz / 8 = 488 281 at div32 - a 16-bit frame
//           is two bytes of the same wire.
//   spi.poll, spi.pump, spi.req  the same: 7 812 500 B/s at div2,
//           3 906 250 at div4 and 976 562 at div16.
//   i2c.*   the tenure's own time on the bus at the MEASURED SCL period
//           (letter i measures it first: the 255- and the 16-byte
//           writes' walls apart, over the 9 x 239 periods between them),
//           in tenths of a period: nine a byte with its acknowledge, the
//           address one of them; 1.4 for the START's hold and the STOP
//           together and 1.4 for a repeated START (4.3.14.1's counts:
//           tHD;STA and tSU;STO are high counts, a STOP or a repeated
//           START also takes a low); 9.4 for a refused address, whose
//           abort is raised at the NACK. The sum was measured: a START_DET
//           and a STOP_DET read on the ruler at the three speeds put a
//           one-byte write at 19.4 periods, the rest a constant. `wire` is
//           n bytes over that time, so x is wall over it and wall less it
//           is the tenure's fixed cost.
//
// THE INSTRUMENT'S COST, and how a reader subtracts it: every line is
// RAW. A Stopwatch wall holds the `stopwatch` line's wall beyond the
// operation; UART0's isr holds the `stamp` line's isr per interrupt (what
// an empty body is charged) and busy the whole `stamp` wall per
// interrupt; an idle turn's busy holds the `window` line's busy - isr.
// Two seams util/bench.hpp states are left open: entry and
// exit latency (15 cycles worst case for the entry, 2.4.3.6.1) count as
// idle, and a handler landing between the WFI's return and the window's
// close counts in both.
//
// build: boards = pico,picow,weact2040
// build: monitor_speed = 115200

#include <stdint.h>

#include <array>
#include <atomic>
#include <cstring>

#include "rp2040/clock.hpp"
#include "rp2040/delay.hpp"
#include "rp2040/dma.hpp"
#include "rp2040/flash.hpp"
#include "rp2040/i2c.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/spi.hpp"
#include "rp2040/ticker.hpp"
#include "rp2040/timer.hpp"
#include "rp2040/uart.hpp"
#include "util/bench.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 125'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;  // rings 64/256 (defaults)
constexpr Serial serial;

constexpr uint32_t console_baud = 115200;

// ---- the ruler ---------------------------------------------------------------

/// Core 0's SysTick ticker read as clk_sys cycles (the file header).
struct Ruler {
    [[gnu::always_inline, gnu::flatten]] static uint32_t now() { return CoreTicker<0>::cycles(); }
    static constexpr uint32_t hz() { return SysClock::hz; }
};
static_assert(CycleRuler<Ruler>);

/// The SysTick vector's ruler: SysTick's position in its period, exact
/// for a difference of two reads inside one period (the file header says
/// why the tick vector cannot use Ruler).
struct TickRuler {
    static constexpr uint32_t last = SysClock::hz / Ticker::ticks_per_second - 1u;
    [[gnu::always_inline]] static uint32_t now() { return last - SysTick->VAL; }
    static constexpr uint32_t hz() { return SysClock::hz; }
};
static_assert(CycleRuler<TickRuler>);

using Idle = BenchIdle<Rp2040Platform<>, Ruler>;
static_assert(Platform<Idle>);

// One meter per bound vector, and the bare one letter r times.
IsrMeter<Ruler, Idle> uart_meter;
IsrMeter<TickRuler, Idle> tick_meter;
IsrMeter<Ruler, Idle> dma_meter;
IsrMeter<Ruler, Idle> spi_meter;
IsrMeter<Ruler, Idle> empty_meter;

IsrMeter<Ruler, Idle> loop_meter;   // letter u: UART1's vector
BenchCounters counters() {
    return bench_counters<Idle>(uart_meter, tick_meter, dma_meter, spi_meter, loop_meter);
}

TestBench<Serial> bench;

// ---- the wires (the file header) ---------------------------------------------

constexpr uint32_t print_wire_bps = console_baud / 10u;
constexpr uint32_t memcpy_wire_bps = 2u * SysClock::hz;
constexpr uint32_t memset_wire_bps = 4u * SysClock::hz;

constexpr std::array<uint32_t, 4> sizes{1u, 16u, 256u, 4096u};
constexpr uint32_t max_size = 4096u;

/// Let the console fall silent: its interrupt would otherwise land in
/// whatever is measured next.
void drain() {
    while (!Serial::tx_idle()) {
    }
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
constexpr uint32_t probe_floor =
    static_cast<uint32_t>(static_cast<uint64_t>(SysClock::hz) * probe_us / 1'000'000u);
constexpr uint32_t reps = 1000u;

void tr_ruler() {
    drain();
    (void)delay_us(clock, probe_us);   // the wait's code into the XIP cache
    const bool whole_ms_refused = !delay_us(clock, 1000u);

    Stopwatch<Ruler> sw;
    const uint32_t us0 = Timer::now_low();
    sw.start();
    const bool served = delay_us(clock, probe_us);
    const uint32_t took = sw.elapsed();
    const uint32_t us = Timer::now_low() - us0;
    print(serial, "  delay_us(clock, ", probe_us, ") served=", served, ": ", took,
          " cycles on the ruler (floor ", probe_floor, "), ", us,
          " us on the system timer; delay_us(clock, 1000) refused=", whole_ms_refused, crlf);
    bench.verdict("the ruler reads delay_us(999) at least hz x 999 us", served && took >= probe_floor);
    bench.verdict("and under that + 5 per cent", took <= probe_floor + probe_floor / 20u);
    bench.verdict("the system timer (the crystal, not clk_sys) reads the same wait within 5 per cent",
                  us + 1u >= probe_us && us <= probe_us + probe_us / 20u + 1u);

    // ruler: one read
    {
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
        bench_line(serial, "ruler", 0u, per_unit(s, reps), Ruler::hz(), 0u);
    }

    // stamp: one enter()+leave() pair, nothing between
    {
        const BenchCounters c0 = bench_counters<Idle>(uart_meter, tick_meter, dma_meter, spi_meter, empty_meter);
        sw.start();
        for (uint32_t i = 0; i < reps; ++i) {
            empty_meter.enter();
            empty_meter.leave();
            asm volatile("" ::: "memory");
        }
        const uint32_t wall = sw.elapsed();
        const BenchSample s =
            bench_sample(wall, c0, bench_counters<Idle>(uart_meter, tick_meter, dma_meter, spi_meter, empty_meter));
        print_totals(reps, "pairs", s);
        bench_line(serial, "stamp", 0u, per_unit(s, reps), Ruler::hz(), 0u);
    }

    // stopwatch: an empty interval, best of 8
    bench_line(serial, "stopwatch", 0u, best_of_8([] {}), Ruler::hz(), 0u);

    // window: one idle turn with nothing pending but the tick
    {
        drain();
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
        const BenchSample s = best_of_8([len] { std::memcpy(mem_dst, mem_src, len); });
        bench_line(serial, "memcpy", n, s, Ruler::hz(), memcpy_wire_bps);
    }
    for (const uint32_t n : sizes) {
        mem_len = n;
        const uint32_t len = mem_len;
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
    Stopwatch<Ruler> sw;
    for (const uint32_t n : sizes) {
        drain();
        const char* text = payload.data() + (max_size - n);
        const BenchCounters c0 = counters();
        sw.start();
        print(serial, text);
        while (!Serial::tx_idle()) {
        }
        const uint32_t wall = sw.elapsed();
        const BenchSample s = bench_sample(wall, c0, counters());
        print(serial, crlf);
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
        disable_interrupts();
        Idle::idle();
    }
    const uint32_t wall = sw.elapsed();
    const BenchSample s = bench_sample(wall, c0, counters());
    bench_line(serial, "tick", 0u, s, Ruler::hz(), 0u);
    bench.verdict("ran", true);
}

// =============================================================================
// d - the DMA engines
// =============================================================================
using Copier = DmaCopyEngine<8>;
using Paced = DmaTxEngine<9, uint32_t>;

constexpr SpiPins spi_pins{.sck = 18, .tx = 19, .rx = 16};
/// Half-word engines: SSPDR's width, so 16-bit frames ride them too.
using DmaHost = SpiHost<0, spi_pins, DmaTxEngine<4, uint16_t>, DmaRxEngine<5, uint16_t>>;

/// Set by the handler that ends the block under measure.
volatile bool dma_done = false;
/// DmaHost owns SPI0 from its init() to its release() in letter d, and
/// only then are its two bodies called: letter e's Plain is the same
/// instance, and an engined host's isr() run on Plain's frames would take
/// them into its own stale tenure and leave Plain's pump waiting.
volatile bool dma_host_live = false;

constexpr uint32_t copy_wire_bps = 4u * SysClock::hz;
constexpr uint16_t pace_y = 400u;
constexpr uint32_t paced_words = 256u;
constexpr uint32_t paced_wire_bps = 4u * (SysClock::hz / pace_y);

alignas(4) uint32_t fill_cell = 0x5A5A5A5Au;
alignas(4) uint32_t paced_table[paced_words];
volatile uint32_t paced_cell = 0;

alignas(4) uint8_t spi_tx[8192];
alignas(4) uint8_t spi_rx[8192];

/// Idle until the handler has flagged the block: masked turns, as the
/// kernel's loop takes them.
void idle_until_done() {
    for (;;) {
        disable_interrupts();
        if (dma_done) {
            enable_interrupts();
            return;
        }
        Idle::idle();
    }
}

/// One block launched by `launch` and idled out, on a Stopwatch.
template <typename Launch>
BenchSample timed_block(Launch launch) {
    Stopwatch<Ruler> sw;
    dma_done = false;
    const BenchCounters c0 = counters();
    sw.start();
    const bool started = launch();
    if (started) {
        idle_until_done();
    }
    const uint32_t wall = sw.elapsed();
    return bench_sample(wall, c0, counters());
}

/// The best (shortest wall) of `runs` blocks.
template <typename Launch>
BenchSample best_block(uint8_t runs, Launch launch) {
    BenchSample best{};
    for (uint8_t run = 0; run < runs; ++run) {
        drain();
        const BenchSample s = timed_block(launch);
        asm volatile("" ::: "memory");
        if (run == 0u || s.wall < best.wall) {
            best = s;
        }
    }
    return best;
}

/// Wait for SysTick's next reload: what follows starts a fresh period.
void await_tick() {
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() == t0) {
    }
}

void td_dma() {
    drain();
    for (uint32_t i = 0; i < max_size; ++i) {
        mem_src[i] = static_cast<uint8_t>(i * 7u);
    }
    Copier::arm();

    // copy and fill, words
    bool exact = true;
    for (const uint32_t n : {16u, 256u, 4096u}) {
        std::memset(mem_dst, 0, n);
        const BenchSample s = best_block(8u, [n] {
            return Copier::copy(reinterpret_cast<uint32_t*>(mem_dst), reinterpret_cast<const uint32_t*>(mem_src), n / 4u);
        });
        exact = exact && std::memcmp(mem_dst, mem_src, n) == 0;
        bench_line(serial, "copy", n, s, Ruler::hz(), copy_wire_bps);
    }
    print(serial, "  copy ", exact ? "exact" : "MISMATCH", ", the fixed cost = wall - n/4 cycles", crlf);
    exact = true;
    for (const uint32_t n : {16u, 256u, 4096u}) {
        std::memset(mem_dst, 0, n);
        const BenchSample s = best_block(8u, [n] {
            return Copier::fill(reinterpret_cast<uint32_t*>(mem_dst), &fill_cell, n / 4u);
        });
        for (uint32_t i = 0; i < n; ++i) {
            exact = exact && mem_dst[i] == 0x5Au;
        }
        bench_line(serial, "fill", n, s, Ruler::hz(), copy_wire_bps);
    }
    print(serial, "  fill ", exact ? "exact" : "MISMATCH", crlf);

    // paced: a table into one cell at the timer's rate
    for (uint32_t i = 0; i < paced_words; ++i) {
        paced_table[i] = i + 1u;
    }
    Paced::arm(&paced_cell, DmaTimer<0>::dreq());
    DmaTimer<0>::set(1u, pace_y);
    drain();
    await_tick();
    const BenchSample ps = timed_block([] { return Paced::start(paced_table, paced_words); });
    bench_line(serial, "paced", 4u * paced_words, ps, Ruler::hz(), paced_wire_bps);
    const bool landed = paced_cell == paced_words;

    // the jitter: the same block, the core stamping every decrement
    using Ch = DmaChannel<Paced::channel>;
    constexpr uint32_t period = SysClock::hz / Ticker::ticks_per_second;
    drain();
    await_tick();
    dma_done = false;
    uint32_t shortest = 0xFFFFFFFFu;
    uint32_t longest = 0;
    uint32_t steps = 0;
    uint32_t skipped = 0;
    uint32_t turns = 0;
    const uint32_t t_poll0 = SysTick->VAL;
    (void)Paced::start(paced_table, paced_words);
    uint32_t last_count = paced_words;
    uint32_t last_val = SysTick->VAL;
    bool first = true;
    while (last_count != 0u && turns < 4'000'000u) {
        ++turns;
        const uint32_t c = Ch::count();
        if (c == last_count) {
            continue;
        }
        const uint32_t v = SysTick->VAL;
        const uint32_t d = last_val >= v ? last_val - v : last_val + period - v;   // VAL counts down
        if (last_count - c != 1u) {
            ++skipped;
        } else if (!first) {
            ++steps;
            shortest = d < shortest ? d : shortest;
            longest = d > longest ? d : longest;
        }
        first = false;
        last_count = c;
        last_val = v;
    }
    const uint32_t poll_cycles = t_poll0 >= last_val ? t_poll0 - last_val : t_poll0 + period - last_val;
    idle_until_done();
    DmaTimer<0>::stop();
    print(serial, "  paced: the cell holds ", paced_cell, landed ? " (the last word)" : " (NOT the last word)",
          "; jitter over ", steps, " intervals: shortest ", shortest, ", longest ", longest, " cycles against ",
          pace_y, ", ", skipped, " skipped; one poll turn ~", turns == 0u ? 0u : poll_cycles / turns, " cycles",
          crlf);
    Paced::stop();

    // spi.dma on the loop-back
    (void)DmaHost::init(clock);
    DmaHost::loopback(true);
    dma_host_live = true;
    for (uint32_t i = 0; i < sizeof spi_tx; ++i) {
        spi_tx[i] = static_cast<uint8_t>(0x31u + 3u * i);
    }
    struct Rung {
        SpiClock clock;
        SpiDataSize bits;
        uint16_t frames[3];
        uint8_t count;
    };
    const Rung rungs[] = {
        {SpiClocks::div4, SpiDataSize::bits8, {16u, 256u, 0u}, 2u},
        {SpiClocks::div32, SpiDataSize::bits8, {16u, 256u, 0u}, 2u},
        {SpiClocks::div4, SpiDataSize::bits16, {16u, 256u, 4096u}, 3u},
    };
    for (const Rung& g : rungs) {
        const uint32_t sck = SysClock::pclk_hz / g.clock.divisor();
        const uint32_t wire_bps = sck / 8u;
        const uint8_t width = g.bits == SpiDataSize::bits16 ? 2u : 1u;
        print(serial, "  spi.dma at ", sck, " Hz, ", width * 8u, "-bit frames:", crlf);
        uint32_t walls[3] = {};
        for (uint8_t k = 0; k < g.count; ++k) {
            const uint16_t frames = g.frames[k];
            const uint32_t bytes = static_cast<uint32_t>(frames) * width;
            std::memset(spi_rx, 0xEE, bytes);
            uint8_t status = 0xFEu;
            const BenchSample s = best_block(4u, [&] {
                DmaHost::Request r{};
                r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(spi_tx));
                r.rx = lend<Lease::reply>(static_cast<uint8_t*>(spi_rx));
                r.len = frames;
                r.clock = g.clock;
                r.bits = g.bits;
                if (DmaHost::start(r)) {
                    status = DmaHost::status();
                    dma_done = true;
                }
                return true;
            });
            walls[k] = s.wall;
            bench_line(serial, "spi.dma", bytes, s, Ruler::hz(), wire_bps);
            print(serial, "    status ", status == 0xFEu ? DmaHost::status() : status, ", ",
                  std::memcmp(spi_tx, spi_rx, bytes) == 0 ? "exact" : "MISMATCH", crlf);
        }
        const uint32_t wire16 = static_cast<uint32_t>(static_cast<uint64_t>(16u * width) * SysClock::hz / wire_bps);
        const uint32_t per_byte_x100 = (walls[1] - walls[0]) * 100u / (240u * width);
        print(serial, "    fixed cost of a transaction: ", walls[0] > wire16 ? walls[0] - wire16 : 0u,
              " cycles (16 frames' wall less their wire); a byte: ", per_byte_x100 / 100u, '.',
              per_byte_x100 % 100u / 10u, per_byte_x100 % 10u, " cycles against the wire's ",
              SysClock::hz / wire_bps, crlf);
    }
    dma_host_live = false;
    DmaHost::release();
    bench.verdict("ran", true);
}

// =============================================================================
// e - the SPI host above the wire: the polled loop, the pump, a request's price
// =============================================================================
using Plain = SpiHost<0, spi_pins>;   // no engines: the polled loop and the pump
using SpiSelect = Pin<17>;
using SpiDc = Pin<22>;
volatile bool plain_live = false;
volatile bool plain_done = false;
uint8_t dcs_command = 0x2C;   // in SRAM, as a client's command byte would be
volatile uint32_t tick_stretch = 0;
constexpr uint32_t rx_proof_runs = 100;

struct SpiRate { SpiClock clock; const char* name; };
constexpr SpiRate e_rates[] = {{SpiClocks::div2, "62.5 MHz"}, {SpiClocks::div4, "31.25 MHz"}, {SpiClocks::div16, "7.8125 MHz"}};

struct SpiReqRun {
    BenchSample sample;
    uint32_t spi_irq;
    uint32_t spi_isr;
    bool overrun;
};

/// One request on host H, polled or pumped, the core idling to a pumped
/// one's completion with the flag read under the mask (the kernel loop's
/// shape, so a completion between the read and the sleep is not lost).
template <typename H>
SpiReqRun e_once(uint8_t cmd_len, uint16_t frames, SpiClock rate, SpiMode mode, SpiDataSize bits,
                 bool polled, bool with_rx) {
    typename H::Request r{};
    r.cs = SpiSelect::ref();
    if (cmd_len != 0u) {
        r.dc = SpiDc::ref();
        r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(&dcs_command));
    }
    r.cmd_len = cmd_len;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(spi_tx));
    r.rx = lend<Lease::reply>(with_rx ? spi_rx : nullptr);
    r.len = frames;
    r.clock = rate;
    r.mode = mode;
    r.bits = bits;
    r.polled = polled;
    drain();
    H::Resource::clear_pending(SpiInterrupt::overrun);
    plain_done = false;
    const uint32_t irq0 = spi_meter.count();
    const uint32_t isr0 = spi_meter.cycles();
    const BenchCounters before = counters();
    const uint32_t t0 = Ruler::now();
    bool done = H::start(r);
    while (!done) {
        disable_interrupts();
        done = plain_done;
        if (done || Ruler::now() - t0 >= Ruler::hz() / 10u) {
            enable_interrupts();
            break;
        }
        Idle::idle();
    }
    const uint32_t wall = Ruler::now() - t0;
    const BenchCounters after = counters();
    const bool overrun = (H::Resource::raw_pending() & SpiInterrupt::overrun) != 0u;
    if (!done) {
        print(serial, "  STALLED: SSPSR ", hex(H::Resource::flags()), " SSPRIS ",
              hex(H::Resource::raw_pending()), " SSPIMSC ", hex(H::Resource::interrupts()), crlf);
        (void)H::recover();
    }
    return {bench_sample(wall, before, after), spi_meter.count() - irq0,
            spi_meter.cycles() - isr0, overrun};
}

template <typename H>
SpiReqRun e_best(uint8_t cmd_len, uint16_t frames, SpiClock rate, SpiMode mode, SpiDataSize bits,
                 bool polled, bool with_rx) {
    bool overrun = e_once<H>(cmd_len, frames, rate, mode, bits, polled, with_rx).overrun;
    SpiReqRun best{};
    for (uint8_t run = 0; run < 5u; ++run) {
        const SpiReqRun s = e_once<H>(cmd_len, frames, rate, mode, bits, polled, with_rx);
        overrun = overrun || s.overrun;
        if (run == 0u || s.sample.wall < best.sample.wall) {
            best = s;
        }
    }
    best.overrun = overrun;
    return best;
}

const char* e_op(bool polled, SpiDataSize bits, bool with_rx) {
    const bool wide = bits == SpiDataSize::bits16;
    if (!polled) {
        return wide ? "spi.pump16" : "spi.pump";
    }
    if (with_rx) {
        return wide ? "spi.poll16.rx" : "spi.poll.rx";
    }
    return wide ? "spi.poll16" : "spi.poll";
}

void e_rx_proof() {
    Plain::loopback(true);
    for (const SpiRate& rate : e_rates) {
        uint32_t exact = 0;
        uint32_t overruns = 0;
        drain();
        // The meter is written by the handler and read here: a fence on
        // either side of the loop, so the compiler reloads what it cannot
        // see change.
        std::atomic_signal_fence(std::memory_order_seq_cst);
        const uint32_t ticks0 = tick_meter.count();
        tick_stretch = 1500;
        for (uint32_t run = 0; run < rx_proof_runs; ++run) {
            for (uint32_t i = 0; i < 256u; ++i) {
                spi_tx[i] = static_cast<uint8_t>(i * 7u + run);
                spi_rx[i] = 0xEE;
            }
            Plain::Resource::clear_pending(SpiInterrupt::overrun);
            Plain::Request r{};
            r.cs = SpiSelect::ref();
            r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(spi_tx));
            r.rx = lend<Lease::reply>(static_cast<uint8_t*>(spi_rx));
            r.len = 256;
            r.clock = rate.clock;
            r.mode = SpiMode::mode3;
            r.polled = true;
            (void)Plain::start(r);
            if (std::memcmp(spi_tx, spi_rx, 256) == 0) {
                ++exact;
            }
            if ((Plain::Resource::raw_pending() & SpiInterrupt::overrun) != 0u) {
                ++overruns;
            }
        }
        tick_stretch = 0;
        std::atomic_signal_fence(std::memory_order_seq_cst);
        const uint32_t ticks = tick_meter.count() - ticks0;
        print(serial, "  spi.poll.rx under a tick handler of 1500 cycles (eight frames at ", rate.name,
              " are ", 8u * (8u * Ruler::hz() / Plain::sck_hz(rate.clock)), "): ", rx_proof_runs,
              " requests of 256 frames on the loop-back, ", exact, " exact, ", overruns, " overruns, ",
              ticks, " ticks landed", crlf);
    }
    Plain::loopback(false);
}

/// THE COLD REQUEST (spi.cold): the price of a DCS command when the XIP
/// cache has not got the host's code - bench_rp2350's op on this chip.
/// spi.req's three-byte request run four times from here, the first of
/// them the host's VERY FIRST transaction after init(): (1) cold as a
/// program meets it, (2) its repeat, warm; (3) with every line of the
/// cache invalidated first (Xip::flush(), 2.6.3.2: the repeatable
/// reading, nothing of the image warm) and (4) its repeat. Each timed as
/// e_once times a request, with the cache's misses beside it
/// (Xip::accesses() less Xip::hits(), the counters cleared right before
/// the start and read right after the wall). The bench line is (3).
struct ColdRun {
    BenchSample sample;
    uint32_t misses;
};

ColdRun e_cold_once(bool invalidate) {
    Plain::Request r{};
    r.cs = SpiSelect::ref();
    r.dc = SpiDc::ref();
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(&dcs_command));
    r.cmd_len = 1;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(spi_tx));
    r.len = 2;
    r.clock = e_rates[2].clock;
    r.mode = SpiMode::mode0;
    r.bits = SpiDataSize::bits8;
    r.polled = true;
    drain();
    Plain::Resource::clear_pending(SpiInterrupt::overrun);
    const BenchCounters before = counters();
    if (invalidate) {
        Xip::flush();
    }
    // UNDER THE MASK: a polled request needs no interrupt, and a tick
    // landing inside this one run - one shot, no best of five - would
    // add its handler's wall and, after the flush, its handler's own
    // cold lines, which are not the host's.
    disable_interrupts();
    Xip::reset_counters();
    const uint32_t t0 = Ruler::now();
    (void)Plain::start(r);
    const uint32_t wall = Ruler::now() - t0;
    // Exact to one line: the reset is two stores a fetch apart, so one
    // hit can land between them and leave the hit count a line ahead.
    const uint32_t hits = Xip::hits();
    const uint32_t accesses = Xip::accesses();
    enable_interrupts();
    const uint32_t misses = accesses > hits ? accesses - hits : 0u;
    const BenchCounters after = counters();
    return {bench_sample(wall, before, after), misses};
}

void e_cold() {
    const SpiRate& rate = e_rates[2];
    const uint32_t sck = Plain::sck_hz(rate.clock);
    const uint32_t wire = static_cast<uint32_t>(static_cast<uint64_t>(3) * 8u * Ruler::hz() / sck);
    const uint32_t gap = static_cast<uint32_t>(static_cast<uint64_t>(3) * 3u * Ruler::hz() / (2u * sck));
    const ColdRun first = e_cold_once(false);
    const ColdRun repeat = e_cold_once(false);
    const ColdRun cold = e_cold_once(true);
    const ColdRun warm = e_cold_once(false);
    bench_line(serial, "spi.cold", 3, cold.sample, Ruler::hz(), sck / 8u);
    print(serial, "  spi.cold: a command byte and 2 data bytes at ", rate.name,
          " with every line of the XIP cache invalidated first: wall ", cold.sample.wall,
          " cycles (the wire ", wire, ", ", wire + gap, " with the gap), ", cold.misses,
          " XIP misses; the same request right after, warm: ", warm.sample.wall, " cycles, ",
          warm.misses, " misses", crlf);
    print(serial, "  spi.cold: the host's first request after init(): ", first.sample.wall,
          " cycles, ", first.misses, " XIP misses; its repeat ", repeat.sample.wall, " cycles, ",
          repeat.misses, " misses", crlf);
}

void te_host() {
    (void)SpiSelect::output(true);
    (void)SpiDc::output(true);
    (void)Plain::init(clock);
    plain_live = true;
    for (uint32_t i = 0; i < 512u; ++i) {
        spi_tx[i] = static_cast<uint8_t>(i * 7u + 1u);
    }
    e_cold();   // FIRST: its first request must be the host's first
    struct Shape { bool polled; bool with_rx; };
    constexpr Shape shapes[] = {{true, false}, {true, true}, {false, true}};
    for (const Shape& shape : shapes) {
        for (const SpiRate& rate : e_rates) {
            const uint32_t sck = Plain::sck_hz(rate.clock);
            for (const SpiDataSize bits : {SpiDataSize::bits8, SpiDataSize::bits16}) {
                const char* const op = e_op(shape.polled, bits, shape.with_rx);
                const uint32_t frame_bytes = bits == SpiDataSize::bits16 ? 2u : 1u;
                for (const SpiMode mode : {SpiMode::mode0, SpiMode::mode3}) {
                    uint32_t walls[2] = {0, 0};
                    uint8_t k = 0;
                    for (const uint16_t frames : {static_cast<uint16_t>(16), static_cast<uint16_t>(256)}) {
                        const SpiReqRun run =
                            e_best<Plain>(0, frames, rate.clock, mode, bits, shape.polled, shape.with_rx);
                        walls[k++] = run.sample.wall;
                        bench_line(serial, op, frames * frame_bytes, run.sample, Ruler::hz(), sck / 8u);
                        print(serial, "  ", op, " ", frames, " frames of ", spi_data_bits(bits), " bits at ",
                              rate.name, ", mode ", static_cast<uint8_t>(mode), ": the SPI vector irq ",
                              run.spi_irq, " isr ", run.spi_isr, "; overrun flag ",
                              run.overrun ? "RAISED" : "clear", crlf);
                    }
                    const uint32_t per_frame_x100 =
                        walls[1] > walls[0] ? (walls[1] - walls[0]) * 100u / 240u : 0u;
                    const uint32_t wire_frame = 8u * frame_bytes * Ruler::hz() / sck;
                    print(serial, "  ", op, " at ", rate.name, ", ", spi_data_bits(bits), "-bit frames, mode ",
                          static_cast<uint8_t>(mode), ": ", per_frame_x100 / 100u, '.',
                          per_frame_x100 % 100u / 10u, per_frame_x100 % 10u,
                          " cycles a frame by the difference of the two sizes, the wire's ", wire_frame,
                          mode == SpiMode::mode0 ? " and with the block's gap " : " (no gap in mode 3) ",
                          mode == SpiMode::mode0 ? wire_frame + 3u * Ruler::hz() / (2u * sck) : 0u, crlf);
                }
            }
        }
    }
    e_rx_proof();
    const SpiRate& req_rate = e_rates[2];
    const uint32_t req_sck = Plain::sck_hz(req_rate.clock);
    for (const uint16_t len : {static_cast<uint16_t>(0), static_cast<uint16_t>(2), static_cast<uint16_t>(15)}) {
        const SpiReqRun run =
            e_best<Plain>(1, len, req_rate.clock, SpiMode::mode0, SpiDataSize::bits8, true, false);
        const uint32_t n = 1u + len;
        const uint32_t wire = static_cast<uint32_t>(static_cast<uint64_t>(n) * 8u * Ruler::hz() / req_sck);
        bench_line(serial, "spi.req", n, run.sample, Ruler::hz(), req_sck / 8u);
        print(serial, "  spi.req: a command byte and ", len, " data bytes at ", req_rate.name, ": the wire ",
              wire, " cycles, the fixed cost ", run.sample.wall > wire ? run.sample.wall - wire : 0u,
              " cycles; overrun flag ", run.overrun ? "RAISED" : "clear", crlf);
    }
    plain_live = false;
    Plain::release();
    bench.verdict("ran", true);
}

// =============================================================================
// u - the serial transport on a loop of its own: UART1 under LBE
//     (pl011/uart.hpp's Pl011Transport over design/serial.md)
// =============================================================================
/// UART1 on its first pin pair under function 2, its transmitter fed into
/// its receiver by UARTCR.LBE (the RX pad ignored while on). The TRANSMIT
/// ops mask the receiver's two interrupts through the resource, so the
/// line serves the transmitter alone (the receive FIFO fills and overruns,
/// unread); the RECEIVE ops put the transmitter on its engine, so the
/// line serves the receiver alone.
constexpr UartPins loop_pins{
    .tx = {4, PinFunction::uart},
    .rx = {5, PinFunction::uart},
};
using LoopTx = DmaTxEngine<2>;
using LoopRx = DmaRxEngine<3>;
using TxPlain = Uart<1, loop_pins, 16, 256>;
using TxEngined = Uart<1, loop_pins, 16, 256, LoopTx>;
using RxPlain = Uart<1, loop_pins, 512, 256, LoopTx>;
using RxEngined = Uart<1, loop_pins, 512, 256, LoopTx, LoopRx>;

enum class LoopMode : uint8_t { none, tx_plain, tx_engined, rx_plain, rx_engined };
volatile LoopMode loop_mode = LoopMode::none;   // read by the vectors
volatile uint32_t edge_at = 0;                  // the ruler at the last edge a vector reported
volatile uint32_t edge_count = 0;

/// The rates: the console's, about 1 Mbaud, and the loop's top rate,
/// UARTCLK / 16 (the divisor's integer floor of one).
constexpr std::array<uint32_t, 3> loop_rates{115200u, 1'000'000u, SysClock::hz / 16u};

struct LoopSnap {
    uint32_t uart_irq, uart_isr, dma_irq, dma_isr;
};
LoopSnap loop_snap() {
    const Idle::CriticalSection cs;
    return {loop_meter.count(), loop_meter.cycles(), dma_meter.count(), dma_meter.cycles()};
}
void print_loop(const LoopSnap& a, const LoopSnap& b) {
    print(serial, "  uart1: irq=", b.uart_irq - a.uart_irq, " isr=", b.uart_isr - a.uart_isr,
          "  dma: irq=", b.dma_irq - a.dma_irq, " isr=", b.dma_isr - a.dma_isr, crlf);
}

std::span<const uint8_t> payload_bytes(uint32_t from, uint32_t n) {
    return {reinterpret_cast<const uint8_t*>(payload.data()) + from, n};
}

template <typename T>
bool loop_up(LoopMode mode, uint32_t baud, bool receiver) {
    loop_mode = mode;   // the vector serves T from its init on
    const bool ok = T::init(clock, baud) && T::loopback(true);
    if (!receiver) {
        T::Resource::interrupts(UartInterrupt::rx | UartInterrupt::rx_timeout, false);
    }
    const uint32_t frame = SysClock::hz / (baud / 10u);
    Stopwatch<Ruler> sw;
    sw.start();
    while (sw.elapsed() < 4u * frame) {
    }
    // Whatever the switch to the loop-back put in the FIFO is junk - at
    // UARTCLK / 16 the change of the receiver's input reads as a frame of
    // 0xFF (measured on the RP2350) - and under an engine it sits in the
    // run until harvest() publishes it.
    (void)T::harvest();
    uint8_t junk = 0;
    while (T::read_byte(junk)) {
    }
    T::clear_errors();
    return ok;
}

template <typename T>
void loop_down() {
    T::release();
    loop_mode = LoopMode::none;
}

/// uart.tx: n bytes of the payload through write_bulk(), refused runs
/// retried, then the wire to its last stop bit (BUSY falling). busy is the
/// TRANSPORT'S share - every write_bulk() call that took bytes and the
/// handlers - because the thread here waits by spinning, which a program
/// with a kernel would spend idle.
template <typename T>
void run_uart_tx(const char* op, LoopMode mode, uint32_t baud, uint32_t n) {
    drain();
    if (!loop_up<T>(mode, baud, false)) {
        print(serial, "  ", op, ": init refused at ", baud, crlf);
        return;
    }
    Stopwatch<Ruler> sw;
    const BenchCounters c0 = counters();
    const LoopSnap v0 = loop_snap();
    uint32_t thread = 0;
    sw.start();
    uint32_t sent = 0;
    const uint32_t budget = SysClock::hz / (baud / 10u) * n * 2u + SysClock::hz / 100u;
    while (sent < n && sw.elapsed() < budget) {
        const uint32_t a = Ruler::now();
        const uint32_t k = T::write_bulk(payload_bytes(max_size - n + sent, n - sent));
        if (k != 0u) {
            thread += Ruler::now() - a;
            sent += k;
        }
    }
    while (!T::tx_idle() && sw.elapsed() < budget) {
    }
    const uint32_t wall = sw.elapsed();
    const LoopSnap v1 = loop_snap();
    BenchSample s = bench_sample(wall, c0, counters());
    s.busy = thread + (v1.uart_isr - v0.uart_isr) + (v1.dma_isr - v0.dma_isr);
    const uint32_t actual = T::actual_baud(SysClock::hz);
    loop_down<T>();
    print(serial, "  ", op, " at ", baud, " baud (", actual, "), sent ", sent,
          ", the thread's write_bulk ", thread, crlf);
    bench_line(serial, op, n, s, Ruler::hz(), baud / 10u);
    print_loop(v0, v1);
}

/// uart.rx: a burst of n bytes round the loop, the transmitter on its
/// engine; wall from the first byte queued to the last one in the receive
/// ring, the data checked after.
template <typename T>
void run_uart_rx(const char* op, LoopMode mode, uint32_t baud, uint32_t n) {
    drain();
    if (!loop_up<T>(mode, baud, true)) {
        print(serial, "  ", op, ": init refused at ", baud, crlf);
        return;
    }
    Stopwatch<Ruler> sw;
    const BenchCounters c0 = counters();
    const LoopSnap v0 = loop_snap();
    sw.start();
    uint32_t sent = 0;
    const uint32_t budget = SysClock::hz / (baud / 10u) * n * 2u + SysClock::hz / 100u;
    while (sent < n && sw.elapsed() < budget) {
        sent += T::write_bulk(payload_bytes(sent, n - sent));
    }
    // The ring read as it fills; harvest() publishes an engine's run (and
    // is free without one).
    uint8_t got[256];
    uint32_t read = 0;
    while (read < n && sw.elapsed() < budget) {
        (void)T::harvest();
        read += T::read_bulk({got + read, n - read});
    }
    const uint32_t wall = sw.elapsed();
    const LoopSnap v1 = loop_snap();
    const BenchSample s = bench_sample(wall, c0, counters());
    uint32_t wrong = 0;
    for (uint32_t i = 0; i < read; ++i) {
        if (got[i] != static_cast<uint8_t>(payload[i])) {
            ++wrong;
        }
    }
    print(serial, "  ", op, " at ", baud, " baud: ", read, " of ", n, " back, ", wrong,
          " wrong, OE ", T::hw_overruns(), " FE ", T::frame_errors(), " ring overruns ",
          T::rx_overruns(), crlf);
    if (wrong != 0u) {
        print(serial, "    got ", hex(got[0]), " ", hex(got[1]), " ", hex(got[2]), " ", hex(got[3]),
              " ... ", hex(got[read - 1u]), ", sent ", hex(static_cast<uint8_t>(payload[0])), " ",
              hex(static_cast<uint8_t>(payload[1])), " ", hex(static_cast<uint8_t>(payload[2])),
              " ", hex(static_cast<uint8_t>(payload[3])), " ... ",
              hex(static_cast<uint8_t>(payload[n - 1u])), crlf);
    }
    loop_down<T>();
    bench_line(serial, op, n, s, Ruler::hz(), baud / 10u);
    print_loop(v0, v1);
}

/// uart.edge: a burst of 17 bytes - one past a receive level of sixteen,
/// so the tail is the edge's to deliver - with the consumer draining on
/// every edge a vector reports; wall = the ruler at the LAST edge minus
/// the ruler at the burst's last stop bit (tx_idle() turning true: BUSY
/// falling), signed.
template <typename T>
void run_uart_edge(const char* op, LoopMode mode, uint32_t baud) {
    constexpr uint32_t n = 17u;
    drain();
    if (!loop_up<T>(mode, baud, true)) {
        print(serial, "  ", op, ": init refused at ", baud, crlf);
        return;
    }
    const uint32_t frame = SysClock::hz / (baud / 10u);
    edge_count = 0;
    uint32_t seen = 0;
    uint32_t got = 0;
    uint32_t wrong = 0;
    bool idle_seen = false;
    uint32_t idle_at = 0;
    (void)T::write_bulk(payload_bytes(0, n));
    const uint32_t t0 = Ruler::now();
    const uint32_t settle = 8u * frame + SysClock::hz / 250u;   // 4 ms past the tail
    for (;;) {
        if (!idle_seen && T::tx_idle()) {
            idle_at = Ruler::now();
            idle_seen = true;
        }
        if (edge_count != seen) {
            seen = edge_count;
            for (;;) {
                const std::span<const uint8_t> run = T::read_span();
                if (run.empty()) {
                    break;
                }
                for (const uint8_t b : run) {
                    if (b != static_cast<uint8_t>(payload[got])) {
                        ++wrong;
                    }
                    ++got;
                }
                (void)T::consume(static_cast<uint32_t>(run.size()));
            }
        }
        const uint32_t now = Ruler::now();
        if ((idle_seen && now - idle_at > settle) || now - t0 > SysClock::hz / 10u) {
            break;
        }
    }
    loop_down<T>();
    const int32_t latency = static_cast<int32_t>(edge_at - idle_at);
    print(serial, "  ", op, " at ", baud, " baud: ", got, " of ", n, " bytes (", wrong,
          " wrong) on ", edge_count, " edges; the last edge ", latency, " cycles after the "
          "last stop bit = ", latency * 100 / static_cast<int32_t>(frame), "/100 of a frame",
          idle_seen ? "" : " (BUSY NEVER FELL)", crlf);
    print(serial, "bench ", op, " n=", n, " wall=", latency, " busy=- irq=", edge_count,
          " isr=- rate=- wire=- x=-", crlf);
}

/// THE QUESTION THE ENGINE'S EDGE STANDS ON: does the receive time-out
/// rise while a channel keeps the FIFO empty? 17 bytes under the receive
/// engine, then ten frames of silence: UARTRIS read raw (RTRIS, RXRIS),
/// the FIFO's emptiness and what the channel took.
void probe_rt_under_engine(uint32_t baud) {
    drain();
    (void)loop_up<RxEngined>(LoopMode::rx_engined, baud, true);
    const uint32_t frame = SysClock::hz / (baud / 10u);
    (void)RxEngined::write_bulk(payload_bytes(0, 17));
    const uint32_t t0 = Ruler::now();
    while (!RxEngined::tx_idle() && Ruler::now() - t0 < 40u * frame) {
    }
    const uint32_t t1 = Ruler::now();
    while (Ruler::now() - t1 < 10u * frame) {
    }
    const uint32_t ris = Pl011<1>::raw_pending();
    const bool empty = Pl011<1>::rx_empty();
    (void)RxEngined::harvest();
    uint8_t got[32];
    const uint32_t landed = RxEngined::read_bulk(got);
    loop_down<RxEngined>();
    print(serial, "  the receive time-out under the engine at ", baud, ": UARTRIS ", hex(ris),
          " (RTRIS ", (ris & UartInterrupt::rx_timeout) != 0u, ", RXRIS ",
          (ris & UartInterrupt::rx) != 0u, "), the FIFO ", empty ? "empty" : "NOT empty", ", ",
          landed, " of 17 bytes in the ring", crlf);
}

/// write_bulk()'s copy: its byte loop against the runtime's memcpy, at
/// the lengths a run takes, with the two ends sharing their word
/// alignment and not: the cycles of one copy, over 64 copies, the empty
/// loop's own taken out.
alignas(4) uint8_t copy_src[136];
alignas(4) uint8_t copy_dst[136];
volatile uint32_t copy_len = 0;

[[gnu::noinline, gnu::optimize("no-tree-loop-distribute-patterns")]]
uint32_t time_byte_loop(uint32_t off_dst, uint32_t k) {
    const uint32_t t0 = Ruler::now();
    for (uint32_t rep = 0; rep < 64u; ++rep) {
        const uint8_t* from = copy_src;
        uint8_t* to = copy_dst + off_dst;
        uint8_t* const end = to + k;
        do {
            *to++ = *from++;
        } while (to != end);
        asm volatile("" ::: "memory");
    }
    return Ruler::now() - t0;
}
[[gnu::noinline]] uint32_t time_memcpy(uint32_t off_dst) {
    const uint32_t t0 = Ruler::now();
    for (uint32_t rep = 0; rep < 64u; ++rep) {
        std::memcpy(copy_dst + off_dst, copy_src, copy_len);
        asm volatile("" ::: "memory");
    }
    return Ruler::now() - t0;
}
[[gnu::noinline]] uint32_t time_empty() {
    const uint32_t t0 = Ruler::now();
    for (uint32_t rep = 0; rep < 64u; ++rep) {
        asm volatile("" ::: "memory");
    }
    return Ruler::now() - t0;
}

void run_copy_crossover() {
    drain();
    uint32_t empty = 0xFFFFFFFFu;
    for (uint8_t i = 0; i < 8u; ++i) {
        const uint32_t e = time_empty();
        empty = e < empty ? e : empty;
    }
    print(serial, "  write_bulk's copy, cycles a copy (byte loop / memcpy), aligned and "
          "dst+1:", crlf);
    for (const uint32_t k : {1u, 2u, 4u, 8u, 12u, 16u, 24u, 32u, 48u, 64u, 128u}) {
        copy_len = k;
        uint32_t best[4] = {0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu};
        for (uint8_t i = 0; i < 8u; ++i) {
            const uint32_t r[4] = {time_byte_loop(0, k), time_memcpy(0), time_byte_loop(1, k),
                                   time_memcpy(1)};
            for (uint8_t j = 0; j < 4u; ++j) {
                best[j] = r[j] < best[j] ? r[j] : best[j];
            }
        }
        print(serial, "  copy n=", k, ": aligned ", (best[0] - empty) / 64u, " / ",
              (best[1] - empty) / 64u, "   dst+1 ", (best[2] - empty) / 64u, " / ",
              (best[3] - empty) / 64u, crlf);
    }
}

void tu_uart() {
    run_copy_crossover();
    print(serial, "  UART1 under LBE (512/256 rings): the transmit ops with the receiver's "
          "interrupts masked, the receive ops with the transmitter on DmaTxEngine<2>, the "
          "receive engine DmaRxEngine<3>", crlf);
    for (const uint32_t baud : loop_rates) {
        for (const uint32_t n : {256u, 4096u}) {
            run_uart_tx<TxPlain>("uart.tx", LoopMode::tx_plain, baud, n);
            run_uart_tx<TxEngined>("uart.tx.dma", LoopMode::tx_engined, baud, n);
        }
    }
    for (const uint32_t baud : loop_rates) {
        for (const uint32_t n : {16u, 256u}) {
            run_uart_rx<RxPlain>("uart.rx", LoopMode::rx_plain, baud, n);
            run_uart_rx<RxEngined>("uart.rx.dma", LoopMode::rx_engined, baud, n);
        }
    }
    for (const uint32_t baud : {115200u, 1'000'000u}) {
        run_uart_edge<RxPlain>("uart.edge", LoopMode::rx_plain, baud);
        run_uart_edge<RxEngined>("uart.edge.dma", LoopMode::rx_engined, baud);
    }
    probe_rt_under_engine(115200u);
    probe_rt_under_engine(1'000'000u);
    bench.verdict("ran", true);
}

// =============================================================================
// i - the I2C host above the wire (dw_apb_i2c/i2c.hpp over rp2040/i2c.hpp),
//     on the self-link: I2C0 the host, I2C1 the client
// =============================================================================
constexpr I2cPins i2c_host_pins{.scl = 13, .sda = 12};
constexpr I2cPins i2c_peer_pins{.scl = 15, .sda = 14};
using I2cPump = I2cHost<0, i2c_host_pins>;
using I2cEngined = I2cHost<0, i2c_host_pins, DmaTxEngine<6, uint16_t>, DmaRxEngine<7>>;
using I2cPeer = I2cClient<1, i2c_peer_pins>;
constexpr uint8_t i2c_peer_addr = 0x42;
constexpr uint8_t i2c_nobody = 0x43;

/// Which host type owns I2C0's bodies (both are instance 0, each with its
/// own statics: one at a time, each brought up by its own init()).
enum class I2cMode : uint8_t { none, pump, engined };
volatile I2cMode i2c_mode = I2cMode::none;
volatile bool i2c_peer_live = false;
volatile bool i2c_done = false;
volatile uint8_t i2c_status = 0;
IsrMeter<Ruler, Idle> i2c_meter;    // I2C0's vector: the host
IsrMeter<Ruler, Idle> peer_meter;   // I2C1's vector: the client, an instrument and not the host's cost
uint32_t i2c_isr_sum = 0;            // the meter's sum after the last entry, for the longest one
volatile uint32_t i2c_isr_max = 0;   // the longest I2C0 entry since the op began (stamps included)

uint8_t i2c_out[256];
uint8_t i2c_in[256];
uint8_t peer_heard[256];
volatile uint16_t peer_count = 0;
volatile uint16_t peer_given = 0;

constexpr uint8_t peer_answer(uint16_t i) { return static_cast<uint8_t>(0xA0u + 5u * i); }

/// The client's service, from its own vector: a written run taken eight at
/// a time (its RX_TL raised by the letter) and whatever stands at any
/// other event - the STOP, the read request of a write-then-read, before
/// the abort that flushes its unread answers flushes the receive FIFO
/// with them -, a read answered with as many bytes as its FIFO takes per
/// read request.
void peer_drain() {
    uint16_t k = peer_count;
    while (I2cPeer::data_ready()) {
        const uint8_t b = I2cPeer::take();
        if (k < 256u) {
            peer_heard[k] = b;
        }
        ++k;
    }
    peer_count = k;
}
void peer_serve() {
    const I2cClientEvent ev = I2cPeer::service();
    peer_drain();
    if (ev == I2cClientEvent::byte_wanted) {
        uint16_t k = peer_given;
        while (I2cPeer::writable()) {
            I2cPeer::give(peer_answer(k));
            ++k;
        }
        peer_given = k;
        I2cPeer::clear_read_request();
    }
}

/// The host's counters: its own vector and the DMA line, nothing else -
/// the client's interrupts and the tick land in the idle windows.
BenchCounters i2c_counters() {
    const Idle::CriticalSection cs;
    return bench_counters<Idle>(i2c_meter, dma_meter);
}

struct I2cOp {
    BenchSample s;
    uint32_t start_cycles;   ///< the thread inside start() alone
    uint32_t peer_irq;       ///< the client's interrupts, on the same core
    uint32_t peer_isr;       ///< and their cycles
    uint32_t xip_miss;       ///< the XIP cache's misses over the tenure, every context (two
                             ///< counters read apart: a count of -1 or -2 is none)
    uint32_t isr_max;        ///< the longest host entry, stamps included
    uint8_t status;
    bool exact;
};

/// One tenure through host H, the core idling to its completion with the
/// flag read under the mask (the kernel loop's shape); then judged
/// against what the client heard or gave.
template <typename H>
I2cOp i2c_once(uint8_t addr, uint8_t tx_len, uint8_t rx_len, I2cSpeed speed) {
    typename H::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(i2c_out));
    r.tx_len = tx_len;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(i2c_in));
    r.rx_len = rx_len;
    r.speed = speed;
    for (uint16_t i = 0; i < rx_len; ++i) {
        i2c_in[i] = 0;
    }
    peer_count = 0;
    peer_given = 0;
    drain();
    i2c_done = false;
    std::atomic_signal_fence(std::memory_order_seq_cst);
    const uint32_t p0 = peer_meter.count();
    const uint32_t q0 = peer_meter.cycles();
    const BenchCounters c0 = i2c_counters();
    i2c_isr_sum = i2c_meter.cycles();
    i2c_isr_max = 0;
    Xip::reset_counters();
    const uint32_t t0 = Ruler::now();
    const bool sync = H::start(r);
    const uint32_t t1 = Ruler::now();
    bool done = sync;
    while (!done) {
        disable_interrupts();
        done = i2c_done;
        if (done || Ruler::now() - t0 >= Ruler::hz() / 10u) {
            enable_interrupts();
            break;
        }
        Idle::idle();
    }
    const uint32_t wall = Ruler::now() - t0;
    const uint32_t xip_hits = Xip::hits();
    const uint32_t xip_miss = Xip::accesses() - xip_hits;
    const BenchCounters c1 = i2c_counters();
    std::atomic_signal_fence(std::memory_order_seq_cst);
    I2cOp op{bench_sample(wall, c0, c1), t1 - t0, 0, 0, xip_miss, i2c_isr_max, sync ? H::status() : i2c_status, false};
    if (!done) {
        using B = typename H::Resource;
        print(serial, "    STALLED: raw ", hex(B::raw_pending()), " mask ", hex(B::interrupts()), " status ",
              hex(B::flags()), " txflr ", B::tx_count(), " rxflr ", B::rx_count(), " abort ", hex(B::abort_source()),
              " con ", hex(B::con()), " tl ", B::regs().IC_RX_TL, '/', B::regs().IC_TX_TL, " i2c0 irq ",
              i2c_meter.count(), " peer given ", peer_given, crlf);
        op.status = 0xFEu;
        (void)H::recover();
    }
    // The client's last service (its STOP) may follow the host's
    // completion: let it land before judging.
    const uint32_t t2 = Ruler::now();
    while (Ruler::now() - t2 < Ruler::hz() / 5000u) {
    }
    op.peer_irq = peer_meter.count() - p0;
    op.peer_isr = peer_meter.cycles() - q0;
    bool exact = op.status == (addr == i2c_peer_addr ? i2c_ok : i2c_nack_addr);
    if (exact && addr == i2c_peer_addr) {
        exact = peer_count == tx_len;
        for (uint16_t i = 0; exact && i < tx_len; ++i) {
            exact = peer_heard[i] == i2c_out[i];
        }
        for (uint16_t i = 0; exact && i < rx_len; ++i) {
            exact = i2c_in[i] == peer_answer(i);
        }
    }
    op.exact = exact;
    if (!exact && addr == i2c_peer_addr) {
        print(serial, "    MISMATCH: status ", op.status, " heard ", peer_count, " of ", tx_len, " [", hex(peer_heard[0]),
              ' ', hex(peer_heard[1]), "] sent [", hex(i2c_out[0]), ' ', hex(i2c_out[1]), "] read [", hex(i2c_in[0]), ' ',
              hex(i2c_in[1]), "] given ", peer_given, crlf);
    }
    return op;
}

/// One warm run thrown away, then the best (shortest wall) of five; exact
/// only if every run was. `readdress` puts a probe of an address nobody
/// answers before every run, so the run's start() finds another address
/// standing in the block.
template <typename H>
I2cOp i2c_best(uint8_t addr, uint8_t tx_len, uint8_t rx_len, I2cSpeed speed, bool readdress = false) {
    if (readdress) {
        (void)i2c_once<H>(i2c_nobody, 0, 0, speed);
    }
    bool exact = i2c_once<H>(addr, tx_len, rx_len, speed).exact;
    I2cOp best{};
    for (uint8_t run = 0; run < 5u; ++run) {
        if (readdress) {
            (void)i2c_once<H>(i2c_nobody, 0, 0, speed);
        }
        const I2cOp o = i2c_once<H>(addr, tx_len, rx_len, speed);
        exact = exact && o.exact;
        if (run == 0u || o.s.wall < best.s.wall) {
            best = o;
        }
    }
    best.exact = exact;
    return best;
}

/// The SCL periods a tenure holds on the wire, in TENTHS: nine for every
/// byte with its acknowledge (the address one of them); the START's hold
/// (tHD;STA, a high count: 0.4 of a period) and the STOP (a low count,
/// then tSU;STO: 1.0) together 1.4; a repeated START another 1.4 (the
/// low, tSU;STA and tHD;STA). A refused address ends at its NACK - the
/// abort is raised there, before the STOP: 9.4. The probe is a one-byte
/// read on this block. The split of START and STOP is the counts' own
/// (4.3.14.1); the sum was measured on the ruler as the START_DET of a
/// tenure against its STOP_DET at the three speeds - 19.4 periods for a
/// one-byte write, the constant between them the software's.
constexpr uint32_t i2c_periods_x10(uint8_t tx_len, uint8_t rx_len, bool refused) {
    if (refused) {
        return 94u;
    }
    if (tx_len == 0u && rx_len == 0u) {
        rx_len = 1;
    }
    uint32_t p = 14u;
    if (tx_len != 0u) {
        p += 90u * (1u + tx_len);
    }
    if (rx_len != 0u) {
        p += 90u * (1u + rx_len) + (tx_len != 0u ? 14u : 0u);
    }
    return p;
}

/// The line, then what it means: the fixed cost (wall less the wire's
/// time at the measured SCL period), the thread's start(), the client's
/// interrupts and the verdict of the bytes.
void i2c_print(const char* op, uint32_t n, const I2cOp& o, uint32_t periods_x10, uint32_t period_x1000) {
    const uint32_t wire_cycles = static_cast<uint32_t>(static_cast<uint64_t>(periods_x10) * period_x1000 / 10000u);
    const uint32_t wire_bps =
        n == 0u || wire_cycles == 0u ? 0u : static_cast<uint32_t>(static_cast<uint64_t>(n) * Ruler::hz() / wire_cycles);
    bench_line(serial, op, n, o.s, Ruler::hz(), wire_bps);
    print(serial, "    fixed=", static_cast<int32_t>(o.s.wall - wire_cycles), " (wire ", wire_cycles, " = ", periods_x10 / 10u, '.', periods_x10 % 10u,
          " SCL) start=", o.start_cycles, " peer_irq=", o.peer_irq, " peer_isr=", o.peer_isr, " xip_miss=", static_cast<int32_t>(o.xip_miss), " isr_max=", o.isr_max, " status=", o.status,
          o.exact ? " exact" : " MISMATCH", crlf);
}

/// One speed on host H: the SCL period measured first (the 255- and the
/// 16-byte writes' walls, the difference over the 9 x 239 periods between
/// them - the bus's own rate, rise time and all), then every op.
template <typename H>
uint32_t i2c_group(I2cSpeed speed) {
    const I2cOp w16 = i2c_best<H>(i2c_peer_addr, 16, 0, speed);
    const I2cOp w255 = i2c_best<H>(i2c_peer_addr, 255, 0, speed);
    const uint32_t period_x1000 =
        static_cast<uint32_t>(static_cast<uint64_t>(w255.s.wall - w16.s.wall) * 1000u / (9u * 239u));
    const I2cTiming t = H::timing_of(speed);
    const uint32_t programmed = (static_cast<uint32_t>(t.hcnt) + t.spklen + 7u) + (static_cast<uint32_t>(t.lcnt) + 1u);
    const uint32_t stretch_x1000 = period_x1000 > programmed * 1000u ? period_x1000 - programmed * 1000u : 0u;
    print(serial, "  ", i2c_speed_hz(speed) / 1000u, " kHz asked: SCL period measured ", period_x1000 / 1000u, '.',
          (period_x1000 % 1000u) / 100u, " cycles = ",
          static_cast<uint32_t>(static_cast<uint64_t>(Ruler::hz()) * 1000u / period_x1000), " Hz; the counts give ",
          programmed, " cycles on an ideal wire, the wire adds ",
          static_cast<uint32_t>(static_cast<uint64_t>(stretch_x1000) * 1'000'000u / Ruler::hz()), " ns a period (SCL's rise to VIH and the synchronizer)",
          crlf);
    for (const uint8_t n : {uint8_t{1}, uint8_t{2}}) {
        i2c_print("i2c.write", n, i2c_best<H>(i2c_peer_addr, n, 0, speed), i2c_periods_x10(n, 0, false), period_x1000);
    }
    i2c_print("i2c.write", 16, w16, i2c_periods_x10(16, 0, false), period_x1000);
    i2c_print("i2c.write", 255, w255, i2c_periods_x10(255, 0, false), period_x1000);
    for (const uint8_t n : {uint8_t{1}, uint8_t{2}, uint8_t{16}, uint8_t{255}}) {
        i2c_print("i2c.read", n, i2c_best<H>(i2c_peer_addr, 0, n, speed), i2c_periods_x10(0, n, false), period_x1000);
    }
    for (const uint8_t n : {uint8_t{1}, uint8_t{2}, uint8_t{16}}) {
        i2c_print("i2c.wr", n, i2c_best<H>(i2c_peer_addr, 1, n, speed), i2c_periods_x10(1, n, false), period_x1000);
    }
    i2c_print("i2c.probe", 1, i2c_best<H>(i2c_peer_addr, 0, 0, speed), i2c_periods_x10(0, 0, false), period_x1000);
    i2c_print("i2c.probe.nack", 0, i2c_best<H>(i2c_nobody, 0, 0, speed), i2c_periods_x10(0, 0, true), period_x1000);
    i2c_print("i2c.write.readdr", 1, i2c_best<H>(i2c_peer_addr, 1, 0, speed, true), i2c_periods_x10(1, 0, false),
              period_x1000);
    return period_x1000;
}

/// The engines' ops at one speed, on the period the pump's group measured.
void i2c_engined_group(I2cSpeed speed, uint32_t period_x1000) {
    for (const uint8_t n : {uint8_t{16}, uint8_t{255}}) {
        i2c_print("i2c.read.dma", n, i2c_best<I2cEngined>(i2c_peer_addr, 0, n, speed), i2c_periods_x10(0, n, false),
                  period_x1000);
    }
    i2c_print("i2c.wr.dma", 16, i2c_best<I2cEngined>(i2c_peer_addr, 1, 16, speed), i2c_periods_x10(1, 16, false),
              period_x1000);
}

void ti_i2c() {
    drain();
    i2c_mode = I2cMode::none;
    i2c_peer_live = true;   // before init(), as the host's mode below
    const bool peer_ok =
        I2cPeer::init(clock, {.address = i2c_peer_addr, .speed = I2cSpeed::fast_plus_1m});
    DwApbI2c<1>::rx_threshold(7);   // the instrument's own: a written run taken eight at a time
    I2cPeer::interrupts(I2cPeer::events, true);
    i2c_mode = I2cMode::pump;   // before init(): a stale pending line lands in the idle engine
    const bool host_ok = I2cPump::init(clock);
    for (uint16_t i = 0; i < 256u; ++i) {
        i2c_out[i] = static_cast<uint8_t>(0x11u + 3u * i);
    }
    print(serial, "  I2C0 hosts on GP13/GP12, I2C1 answers ", hex(i2c_peer_addr),
          " on GP15/GP14 from its own vector (not metered into busy, irq or isr); ", hex(i2c_nobody),
          " is nobody. init: host=", host_ok, " client=", peer_ok, crlf);
    const I2cOp p = i2c_once<I2cPump>(i2c_peer_addr, 0, 0, I2cSpeed::fast_400k);
    if (!peer_ok || !host_ok || p.status != i2c_ok) {
        print(serial, "  NO SELF-LINK: the probe of 0x42 answered ", p.status,
              " - wire GP12-GP14 and GP13-GP15 with 4.7 kOhm to 3V3", crlf);
        bench.verdict("ran", false);
    } else {
        for (const I2cSpeed speed : {I2cSpeed::standard_100k, I2cSpeed::fast_400k, I2cSpeed::fast_plus_1m}) {
            const uint32_t period = i2c_group<I2cPump>(speed);
            i2c_mode = I2cMode::none;
            I2cPump::release();
            i2c_mode = I2cMode::engined;
            (void)I2cEngined::init(clock);
            i2c_engined_group(speed, period);
            i2c_mode = I2cMode::none;
            I2cEngined::release();
            i2c_mode = I2cMode::pump;
            (void)I2cPump::init(clock);
        }
        bench.verdict("ran", true);
    }
    i2c_mode = I2cMode::none;
    I2cPump::release();
    i2c_peer_live = false;
    I2cPeer::release();
}

void banner() {
    print(serial, crlf, "bench_rp2040 - the benchmark skeleton (util/bench.hpp), clk=", SysClock::hz,
          " Hz, console UART0 ", console_baud, " 8N1, ruler SysTick cycles", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_uart0() {
    uart_meter.enter();
    (void)Serial::isr();
    uart_meter.leave();
}
extern "C" void isr_uart1() {
    loop_meter.enter();
    bool edge = false;
    switch (loop_mode) {
        case LoopMode::tx_plain: edge = TxPlain::isr(); break;
        case LoopMode::tx_engined: edge = TxEngined::isr(); break;
        case LoopMode::rx_plain: edge = RxPlain::isr(); break;
        case LoopMode::rx_engined: edge = RxEngined::isr(); break;
        default:
            // Nothing of ours is up: silence the line rather than re-enter
            // for ever on a condition nobody clears.
            Nvic::disable(Pl011<1>::irq());
            break;
    }
    loop_meter.leave();
    if (edge) {
        edge_at = Ruler::now();
        edge_count = edge_count + 1u;
    }
}
extern "C" void isr_dma_0() {
    dma_meter.enter();
    bool edge = false;
    switch (loop_mode) {
        case LoopMode::tx_engined: (void)TxEngined::dma_isr(); break;
        case LoopMode::rx_plain: (void)RxPlain::dma_isr(); break;
        case LoopMode::rx_engined: edge = RxEngined::dma_isr(); break;   // a run's completion
        default: break;
    }
    if (edge) {
        edge_at = Ruler::now();
        edge_count = edge_count + 1u;
    }
    if ((Copier::service() & Copier::flag_complete) != 0u) {
        dma_done = true;
    }
    if ((Paced::service() & Paced::flag_complete) != 0u) {
        (void)Paced::complete();
        dma_done = true;
    }
    if (dma_host_live && DmaHost::dma_isr()) {
        dma_done = true;
    }
    if (i2c_mode == I2cMode::engined && I2cEngined::dma_isr()) {
        i2c_status = I2cEngined::status();
        i2c_done = true;
    }
    dma_meter.leave();
}
// Letter i's two I2C vectors run from SRAM, flattened (the file header).
extern "C" [[gnu::section(".ram_text"), gnu::flatten]] void isr_i2c0() {
    i2c_meter.enter();
    bool edge = false;
    switch (i2c_mode) {
        case I2cMode::pump:
            edge = I2cPump::isr();
            if (edge) { i2c_status = I2cPump::status(); }
            break;
        case I2cMode::engined:
            edge = I2cEngined::isr();
            if (edge) { i2c_status = I2cEngined::status(); }
            break;
        default:
            // Nothing of ours is up: silence the block, not the line, so
            // that the next init() finds it as it left it.
            DwApbI2c<0>::interrupts_only(0);
            break;
    }
    if (edge) {
        i2c_done = true;
    }
    i2c_meter.leave();
    const uint32_t sum = i2c_meter.cycles();
    if (sum - i2c_isr_sum > i2c_isr_max) {
        i2c_isr_max = sum - i2c_isr_sum;
    }
    i2c_isr_sum = sum;
}
extern "C" [[gnu::section(".ram_text"), gnu::flatten]] void isr_i2c1() {
    peer_meter.enter();
    if (i2c_peer_live) {
        peer_serve();
    } else {
        DwApbI2c<1>::interrupts_only(0);
    }
    peer_meter.leave();
}
extern "C" void isr_spi0() {
    spi_meter.enter();
    if (dma_host_live && DmaHost::isr()) {
        dma_done = true;
    }
    if (plain_live && Plain::isr()) {
        plain_done = true;
    }
    spi_meter.leave();
}
extern "C" void isr_systick() {
    tick_meter.enter();
    brio::Ticker::tick();
    if (tick_stretch != 0u) {
        // Letter e's proof: a handler longer than eight frame times.
        const uint32_t t0 = Ruler::now();
        while (Ruler::now() - t0 < tick_stretch) {
        }
    }
    tick_meter.leave();
    // THE OWNER'S POLL, a TimeEvent's worth: under the receive engine no
    // vector ends a burst on this block (pl011/uart.hpp's dma_isr()), so
    // the burst edge of letter u's engined op is harvest() asked once a
    // tick.
    if (loop_mode == LoopMode::rx_engined && RxEngined::harvest()) {
        edge_at = Ruler::now();
        edge_count = edge_count + 1u;
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool timer_ok = brio::Timer::init(clock);
    const bool dma_ok = brio::Dma::init();
    const bool serial_ok = Serial::init(clock, console_baud);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    bench.letter('r', "the ruler's self-check and the instrument's cost", tr_ruler);
    bench.letter('m', "memcpy and memset, 1/16/256/4096 bytes", tm_memory);
    bench.letter('p', "a print of 1/16/256/4096 bytes through the console", tp_print);
    bench.letter('t', "the tick's floor: one second of idle", tt_tick);
    bench.letter('d', "the DMA engines: copy, fill, paced, spi.dma", td_dma);
    bench.letter('e', "the SPI host above the wire: the polled loop, the pump, a request's price", te_host);
    bench.letter('u', "the serial transport on UART1's own loop: tx, rx, the edge", tu_uart);
    bench.letter('i', "the I2C host above the wire on the self-link: a tenure's price", ti_i2c);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL125" : "FAILED",
                    " timer=", timer_ok ? "1us" : "FAILED", " dma=", dma_ok ? "up" : "FAILED", " tick=",
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
