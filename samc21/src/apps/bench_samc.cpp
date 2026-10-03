// bench_samc - the benchmark skeleton on the SAM C21 (docs/design/
// benchmark.md, util/bench.hpp): five letters that print NUMBERS, one
// `bench` line per operation and size, in the grammar every family
// prints. NOT A TEST: a letter's one verdict is "ran", except letter r,
// whose verdicts judge the ruler every other line is read with.
//
// NOTHING TO WIRE. The console is the board's CH340 bridge on PB30 (TX)
// / PB31 (RX) = SERCOM5 PAD[0]/PAD[1] under function D, 115200 8N1 - the
// binding of console.cpp and test_samc_platform.cpp, with the latter's
// TestBench frame and polled prompt loop. Letter d drives PA16 and PA17
// (SERCOM1's MOSI and SCK) and leaves PA19 (MISO) floating: nothing is
// wired there and nothing should be. NO KERNEL AND NO AO: the letters are
// plain functions and the idle path is called by hand, masked, as the
// kernel's loop calls it - what is measured is the transport, the
// runtime, the DMA and the idle path, never a dispatch.
//
// THE CLOCK: OSC48M undivided, 48 MHz on GCLK0 (samc21/clock.hpp), the
// flash at 2 wait states (DS60001479M table 45-41) behind NVMCTRL's
// 64-byte cache in its reset mode, NO_MISS_PENALTY (27.6.7, 27.8.2) - the
// rate test_samc_platform runs at.
//
// WHERE THE VECTORS RUN. SERCOM5_Handler and SysTick_Handler (and
// letter d's DMAC_Handler and SERCOM1_Handler) are bound through
// BENCH_PLACEMENT. It is empty in this file, so the handlers -
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
//   d  THE DMA (samc21/dmac.hpp, docs/samc21/dmac.md), on its own vector
//      DMAC_Handler and SERCOM1's, each with a meter. Every operation
//      waits by IDLING (a masked test, then Idle::idle(), bounded at
//      10 ms), so busy is the launch, the completion's handler and the
//      loop's turns; copy, fill and spi.dma are the BEST of 8 by wall,
//      and the line after each prints `launch` - the fewest cycles from
//      the interval's start to the start verb's return - and whether every
//      run completed:
//        copy      16/256/4096 bytes as word beats between two word arrays
//                  in SRAM by DmaCopyEngine<0>, the copy judged word for
//                  word after;
//        fill      the same sizes from one cell;
//        paced     DmaLoopEngine<1, uint32_t> playing a 256-word table into
//                  one cell, TC0's overflow the request at 100 kHz (MFRQ,
//                  CC0 = 479 on GCLK0), eight laps re-armed from the
//                  completion, the lap's end stamped on the ruler in the
//                  handler: the line, then the lap-to-lap deviation from
//                  256 x 480 cycles;
//        spi.dma   SpiHost<1> on PA16 (MOSI), PA17 (SCK) and PA19 (MISO,
//                  FLOATING: the bytes are not judged, the time is the
//                  wire's) with DmaTxEngine<2> and DmaRxEngine<3>, a
//                  full-duplex data phase of 16 and 256 frames at 12 and
//                  3 MHz, no command phase, no chip select, ISR-style;
//        spi.dma.tx  the same, write-only (null rx): ONE channel, TXC the
//                  edge.
//   e  THE SPI HOST ABOVE THE WIRE (samc21/spi.hpp, docs/samc21/spi.md):
//      the ENGINELESS host on letter d's pads (the same SERCOM1, PA16
//      MOSI, PA17 SCK, PA19 MISO floating - the time is the wire's, the
//      bytes are not judged), so what is measured is the polled loop and
//      the byte pump and nothing of the DMAC. Every line the BEST of 8 by
//      wall; the two rates are letter d's, BAUD 1 (12 MHz, f_ref/4) and
//      BAUD 7 (3 MHz, f_ref/16); a frame is eight bits on this family:
//        spi.poll  a POLLED WRITE data phase (tx set, rx null - the
//                  display's bulk shape) of 16 and 256 frames, no command
//                  phase, no select: wall against the wire's cycles is the
//                  loop's shape; STATUS.BUFOVF is cleared before and read
//                  after every run and the line after counts the runs it
//                  stood - the overrun the two-level receive buffer raises
//                  if a loop ever leaves more unread than it holds;
//        spi.poll.rx  the same, the RECEIVE shape (tx and rx both set);
//        spi.pump  the same two sizes and rates ISR-style (tx and rx), the
//                  thread idling as letter d does: irq and isr are the
//                  pump's shape (one interrupt a frame), busy its CPU cost;
//                  BUFOVF read after every run as above;
//        spi.req   THE FIXED COST OF A REQUEST, the price of a DCS command:
//                  a polled request of 1, 3 and 16 bytes (cmd_len 1, len 0,
//                  2 and 15, the D/C scripted) at 3 MHz, the select on PA18
//                  (SERCOM1's SS pad, which a software-select host leaves
//                  an ordinary GPIO) and the D/C on PB23 (the board's LED):
//                  two real pads, two real edges each. The line after prints
//                  wall minus the wire's cycles (128 a byte at 3 MHz) - the
//                  instrument's `interval` of letter r is inside it;
//        spi.req.eng  the same three requests on letter d's ENGINED host:
//                  what a short request costs when the engines are named.
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
//   copy, fill  2 bytes a cycle x hz = 96 000 000 B/s: the DMAC's data
//           bus moves a word beat as one read and one write (25.6.2.5), an
//           access a cycle at best. The controller's own pace is slower -
//           five cycles a beat, measured by the difference of 256 and 4096
//           bytes - so x is about 2.5 there and the fixed cost is wall minus
//           five cycles a beat.
//   paced   4 bytes a period of TC0's overflow: 400 000 B/s.
//   spi.dma, spi.poll, spi.poll.rx, spi.pump, spi.req  SCK / 8 bytes a
//           second: one frame of eight bits a byte.
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
#include "samc21/dmac.hpp"
#include "samc21/nvic.hpp"
#include "samc21/platform.hpp"
#include "samc21/sercom.hpp"
#include "samc21/spi.hpp"
#include "samc21/tc.hpp"
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
IsrMeter<Ruler, Idle> dmac_meter;
IsrMeter<Ruler, Idle> spi_meter;

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
        c0_ = bench_counters<Idle>(sercom_meter, tick_meter, dmac_meter, spi_meter);
        sw_.start();
    }
    [[gnu::always_inline]] uint32_t elapsed() const { return sw_.elapsed(); }
    [[gnu::always_inline]] BenchSample stop() {
        SamPlatform::CriticalSection guard;
        const uint32_t wall = sw_.elapsed();
        return bench_sample(wall, c0_, bench_counters<Idle>(sercom_meter, tick_meter, dmac_meter, spi_meter));
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

// =============================================================================
// d - the DMA: copy, fill, paced, spi.dma
// =============================================================================
namespace dd {

constexpr uint8_t ch_copy = 0;
constexpr uint8_t ch_paced = 1;
constexpr uint8_t ch_spi_tx = 2;
constexpr uint8_t ch_spi_rx = 3;

using Copy = DmaCopyEngine<ch_copy>;
using Paced = DmaLoopEngine<ch_paced, uint32_t>;
using PaceTc = Tc<0>;

/// SERCOM1 as an SPI host on PA16 (MOSI), PA17 (SCK), PA19 (MISO, left
/// floating: the bytes are not judged here) - test_samc_spi's host pads.
constexpr SpiPads spi_pads{
    .data_out = SercomPad::pad0,
    .sck = SercomPad::pad1,
    .ss = SercomPad::pad2,
    .data_in = SercomPad::pad3,
    .data_out_pin = {'A', 16, PinFunction::c},
    .sck_pin = {'A', 17, PinFunction::c},
    .ss_pin = {'A', 18, PinFunction::c},
    .data_in_pin = {'A', 19, PinFunction::c},
};
using SpiHw = SpiHost<1, spi_pads, 0, DmaTxEngine<ch_spi_tx>, DmaRxEngine<ch_spi_rx>>;

alignas(4) volatile uint32_t src[1024];
alignas(4) volatile uint32_t dst[1024];
volatile uint32_t cell = 0;

constexpr uint32_t pace_period = 480u;        // TC0 on GCLK0 at 48 MHz: 100 kHz
constexpr uint16_t pace_words = 256u;
constexpr uint8_t pace_laps = 8u;
alignas(4) uint32_t pace_table[pace_words];
volatile uint8_t pace_seen = 0;
uint32_t pace_stamp[pace_laps];

uint8_t spi_tx[256];
uint8_t spi_rx[256];
volatile bool spi_done = false;

/// The completions, called from DMAC_Handler for each take_pending() result.
/// The copy channel's TCMPL needs nothing: take_pending() acknowledged it,
/// and its interrupt only woke the idling core.
[[gnu::always_inline]] inline void dmac_dispatch(const DmaInterrupt& irq) {
    if (irq.channel == ch_copy) {
        return;
    }
    if (irq.channel == ch_paced) {
        pace_stamp[pace_seen] = Ruler::now();
        pace_seen = static_cast<uint8_t>(pace_seen + 1u);
        if (pace_seen >= pace_laps) {
            Paced::stop();
        } else {
            (void)Paced::complete();
        }
    } else if (SpiHw::dma_isr(irq.channel, irq.flags)) {
        spi_done = true;
    }
}

bool start_copy(volatile uint32_t* to, const volatile uint32_t* from, uint16_t words,
                bool fixed_source) {
    return fixed_source ? Copy::fill(to, from, words) : Copy::copy(to, from, words);
}
bool copy_finished() { return !Copy::busy(); }

bool copy_setup() {
    Copy::arm();
    return true;
}

}  // namespace dd

namespace dd {

/// The wait for a completion: the core IDLES - a masked test, then the
/// platform's sleep, as the kernel's loop does - bounded at 10 ms. Busy is
/// then the launch, the completion's handler and the loop's own turns.
/// (A spin on the flag instead moves the DMAC's beats no slower: measured,
/// the same five cycles a word beat either way.)
template <typename Flag>
bool wait_for(Flag done) {
    const uint32_t t0 = Ruler::now();
    for (;;) {
        disable_interrupts();
        if (done()) {
            enable_interrupts();
            return true;
        }
        Idle::idle();
        if (Ruler::now() - t0 > Ruler::hz() / 100u) {
            return done();
        }
    }
}

/// One operation, the best of 8 by wall, and the launch's own share: the
/// fewest cycles from the interval's start to start()'s return over the 8.
template <typename Start, typename Done>
BenchSample best_dma(Start start, Done done, uint32_t& launch, bool& ok) {
    BenchSample best{};
    Interval iv;
    launch = 0;
    ok = true;
    for (uint8_t run = 0; run < 8u; ++run) {
        iv.start();
        const bool started = start();
        const uint32_t l = iv.elapsed();
        const bool finished = started && wait_for(done);
        const BenchSample s = iv.stop();
        asm volatile("" ::: "memory");
        ok = ok && finished;
        if (run == 0u || s.wall < best.wall) {
            best = s;
        }
        if (run == 0u || l < launch) {
            launch = l;
        }
    }
    return best;
}

/// The wires (the file header): the DMAC's data bus moves a word beat as
/// one read and one write, an access a cycle at best - 2 bytes a cycle.
constexpr uint32_t dma_wire_bps = 2u * SysClock::hz;

void copy_and_fill() {
    for (uint32_t i = 0; i < 1024u; ++i) {
        src[i] = 0x01020304u * (i + 1u);
    }
    for (const uint32_t n : sizes) {
        if (n < 16u) {
            continue;
        }
        const uint16_t words = static_cast<uint16_t>(n / 4u);
        for (uint32_t i = 0; i < 1024u; ++i) {
            dst[i] = 0;
        }
        (void)drain();
        uint32_t launch = 0;
        bool ok = false;
        const BenchSample s = best_dma([words] { return start_copy(dst, src, words, false); },
                                       [] { return copy_finished(); }, launch, ok);
        uint32_t mism = 0;
        for (uint32_t i = 0; i < words; ++i) {
            mism += dst[i] != src[i] ? 1u : 0u;
        }
        bench_line(serial, "copy", n, s, Ruler::hz(), dma_wire_bps);
        print(serial, "  launch ", launch, " cycles, completions ", ok ? "all" : "MISSING",
              ", mismatched words ", mism, crlf);
    }
    for (const uint32_t n : sizes) {
        if (n < 16u) {
            continue;
        }
        const uint16_t words = static_cast<uint16_t>(n / 4u);
        cell = 0xA5C3E1F0u;
        (void)drain();
        uint32_t launch = 0;
        bool ok = false;
        const BenchSample s = best_dma([words] { return start_copy(dst, &cell, words, true); },
                                       [] { return copy_finished(); }, launch, ok);
        uint32_t mism = 0;
        for (uint32_t i = 0; i < words; ++i) {
            mism += dst[i] != 0xA5C3E1F0u ? 1u : 0u;
        }
        bench_line(serial, "fill", n, s, Ruler::hz(), dma_wire_bps);
        print(serial, "  launch ", launch, " cycles, completions ", ok ? "all" : "MISSING",
              ", mismatched words ", mism, crlf);
    }
}

/// paced: 8 laps of 256 words into one cell, TC0's overflow the request
/// at 100 kHz, the loop re-armed from the completion; the thread idles.
void paced() {
    for (uint32_t i = 0; i < pace_words; ++i) {
        pace_table[i] = i;
    }
    if (!PaceTc::init(0) ||
        !PaceTc::configure({.mode = TcMode::count16, .waveform = TcWaveform::match_frequency}) ||
        !PaceTc::set_cc16(0, static_cast<uint16_t>(pace_period - 1u))) {
        print(serial, "  TC0 did not come up", crlf);
        return;
    }
    Paced::arm(&cell, PaceTc::dma_trigger_overflow);
    (void)PaceTc::enable(true);
    (void)drain();
    pace_seen = 0;
    Interval iv;
    iv.start();
    (void)Paced::start(pace_table, pace_words);
    while (pace_seen < pace_laps && iv.elapsed() < Ruler::hz() / 10u) {
        disable_interrupts();
        if (pace_seen >= pace_laps) {
            enable_interrupts();
            break;
        }
        Idle::idle();
    }
    const BenchSample s = iv.stop();
    (void)PaceTc::enable(false);
    PaceTc::release();
    const uint32_t nominal = pace_period * pace_words;
    int32_t lo = 0;
    int32_t hi = 0;
    for (uint8_t k = 1; k < pace_seen; ++k) {
        const int32_t dev = static_cast<int32_t>(pace_stamp[k] - pace_stamp[k - 1u] - nominal);
        if (k == 1u || dev < lo) {
            lo = dev;
        }
        if (k == 1u || dev > hi) {
            hi = dev;
        }
    }
    constexpr uint32_t paced_wire_bps = 4u * (SysClock::hz / pace_period);
    bench_line(serial, "paced", static_cast<uint32_t>(pace_seen) * pace_words * 4u, s,
               Ruler::hz(), paced_wire_bps);
    print(serial, "  laps ", pace_seen, " of ", pace_laps, ", lap-to-lap against ", nominal,
          " cycles: ", lo, " .. ", hi, ", the cell holds ", cell, crlf);
}

/// spi.dma: a full-duplex data phase (two channels) and a write-only one,
/// 16 and 256 frames at two rates, MISO floating on PA19: the time is
/// the wire's and the engines', the bytes are not judged.
void spi_dma() {
    if (!SpiHw::init(clock)) {
        print(serial, "  the SPI host did not come up", crlf);
        return;
    }
    for (uint32_t i = 0; i < 256u; ++i) {
        spi_tx[i] = static_cast<uint8_t>(i * 7u + 1u);
    }
    static constexpr uint8_t bauds[] = {1, 7};   // 12 MHz and 3 MHz SCK
    for (const bool duplex : {true, false}) {
        for (const uint8_t baud : bauds) {
            const uint32_t sck = spi_sck_hz(SysClock::hz, baud);
            for (const uint16_t frames : {static_cast<uint16_t>(16), static_cast<uint16_t>(256)}) {
                (void)drain();
                uint32_t launch = 0;
                bool ok = false;
                const BenchSample s = best_dma(
                    [frames, baud, duplex] {
                        spi_done = false;
                        SpiHw::Request r{
                            .cs = {}, .dc = {}, .cmd = {},
                            .tx = lend<Lease::reply>(static_cast<const uint8_t*>(spi_tx)),
                            .rx = duplex ? lend<Lease::reply>(static_cast<uint8_t*>(spi_rx))
                                         : Borrowed<uint8_t,
Lease::reply>{},
                            .len = frames, .cmd_len = 0, .polled = false, .baud = baud,
                            .mode = SpiMode::mode0, .reply = {},
                        };
                        return !SpiHw::start(r);
                    },
                    [] { return spi_done; }, launch, ok);
                bench_line(serial, duplex ? "spi.dma" : "spi.dma.tx", frames, s, Ruler::hz(),
                           sck / 8u);
                print(serial, "  ", duplex ? "full duplex" : "write-only", ", SCK ", sck,
                      " Hz: launch ", launch, " cycles, completions ", ok ? "all" : "MISSING",
                      ", status ", SpiHw::status(), ", wire ",
                      static_cast<uint32_t>(frames) * 8u * (SysClock::hz / sck), " cycles", crlf);
            }
        }
    }
    SpiHw::release();
}

}  // namespace dd

void td_dma() {
    if (!Dmac::init() || !dd::copy_setup()) {
        print(serial, "  the DMAC did not come up", crlf);
        bench.verdict("ran", false);
        return;
    }
    dd::copy_and_fill();
    dd::paced();
    dd::spi_dma();
    Dmac::release();
    bench.verdict("ran", true);
}

// =============================================================================
// e - the SPI host above the wire: spi.poll, spi.pump, spi.req
// =============================================================================
namespace ee {

/// The ENGINELESS host on letter d's pads: the polled loop and the byte
/// pump, nothing of the DMAC. The two hosts share SERCOM1 and are never up
/// at once; `live` routes SERCOM1_Handler to this one.
using SpiPoll = SpiHost<1, dd::spi_pads>;
volatile bool live = false;
volatile bool done = false;

/// The two real pads of spi.req: the select on SERCOM1's SS pad, which a
/// software-select host never claims, and the D/C on the board's LED.
using CsPin = Pin<'A', 18>;
using DcPin = Pin<'B', 23>;

constexpr uint8_t bauds[] = {1, 7};   // 12 MHz (f_ref/4) and 3 MHz (f_ref/16)
constexpr uint16_t sizes_e[] = {16, 256};
constexpr uint8_t req_baud = 7;       // spi.req at 3 MHz: 128 cycles a byte
constexpr uint8_t cmd_byte = 0x2C;    // a DCS memory write, as a command byte

/// A request over the shared buffers, the fields assigned by NAME (the
/// descriptor's order is the host's own business).
template <typename Host>
typename Host::Request request(uint16_t len, bool duplex, uint8_t baud, bool polled) {
    typename Host::Request r{};
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(dd::spi_tx));
    r.rx = duplex ? lend<Lease::reply>(static_cast<uint8_t*>(dd::spi_rx))
                  : Borrowed<uint8_t, Lease::reply>{};
    r.len = len;
    r.baud = baud;
    r.mode = SpiMode::mode0;
    r.polled = polled;
    return r;
}

/// The wire's cycles for n frames at this BAUD.
uint32_t wire_cycles(uint32_t n, uint8_t baud) {
    return n * 8u * (SysClock::hz / spi_sck_hz(SysClock::hz, baud));
}

/// spi.poll (the WRITE shape: tx set, rx null) and spi.poll.rx (the
/// RECEIVE shape: tx and rx set), two sizes, two rates, the best of 8;
/// STATUS.BUFOVF cleared before and read after every run.
void poll() {
    for (const bool receive : {false, true}) {
        for (const uint8_t baud : bauds) {
            const uint32_t sck = spi_sck_hz(SysClock::hz, baud);
            for (const uint16_t n : sizes_e) {
                (void)drain();
                BenchSample best{};
                uint8_t overruns = 0;
                Interval iv;
                for (uint8_t run = 0; run < 8u; ++run) {
                    SpiPoll::Resource::clear_status(SpiStatus::overflow);
                    const auto r = request<SpiPoll>(n, receive, baud, true);
                    iv.start();
                    (void)SpiPoll::start(r);
                    const BenchSample s = iv.stop();
                    asm volatile("" ::: "memory");
                    if (SpiPoll::Resource::overflow_flag()) {
                        ++overruns;
                    }
                    if (run == 0u || s.wall < best.wall) {
                        best = s;
                    }
                }
                bench_line(serial, receive ? "spi.poll.rx" : "spi.poll", n, best, Ruler::hz(),
                           sck / 8u);
                print(serial, "  ", receive ? "receive (tx and rx)" : "write (rx null)", ", SCK ",
                      sck, " Hz, wire ", wire_cycles(n, baud), " cycles, above it ",
                      best.wall - wire_cycles(n, baud), ", BUFOVF after ", overruns, " of 8 runs",
                      crlf);
            }
        }
    }
}

/// spi.pump: the byte pump ISR-style, the thread idling; BUFOVF read after
/// every run.
void pump() {
    for (const uint8_t baud : bauds) {
        const uint32_t sck = spi_sck_hz(SysClock::hz, baud);
        for (const uint16_t n : sizes_e) {
            (void)drain();
            uint32_t launch = 0;
            bool ok = false;
            uint8_t overruns = 0;
            // best_dma's 8 runs; the overflow flag is cleared before each
            // and read after it by the completion wait's done lambda.
            const BenchSample s = dd::best_dma(
                [n, baud] {
                    done = false;
                    SpiPoll::Resource::clear_status(SpiStatus::overflow);
                    const auto r = request<SpiPoll>(n, true, baud, false);
                    return !SpiPoll::start(r);
                },
                [&overruns] {
                    if (!done) {
                        return false;
                    }
                    if (SpiPoll::Resource::overflow_flag()) {
                        ++overruns;
                        SpiPoll::Resource::clear_status(SpiStatus::overflow);
                    }
                    return true;
                },
                launch, ok);
            bench_line(serial, "spi.pump", n, s, Ruler::hz(), sck / 8u);
            print(serial, "  full duplex, SCK ", sck, " Hz: launch ", launch, " cycles, completions ",
                  ok ? "all" : "MISSING", ", wire ", wire_cycles(n, baud), " cycles, BUFOVF after ",
                  overruns, " of 8 runs", crlf);
        }
    }
}

/// spi.req: the fixed cost of a polled request with a scripted D/C and a
/// real select, on the engineless host and on the engined one.
template <typename Host>
void req(const char* op) {
    static constexpr uint16_t lens[] = {0, 2, 15};
    static const uint8_t cmd[1] = {cmd_byte};
    for (const uint16_t len : lens) {
        (void)drain();
        const uint32_t bytes = 1u + len;
        const BenchSample s = best_of_8([len] {
            auto r = request<Host>(len, false, req_baud, true);
            r.cs = CsPin::ref();
            r.dc = DcPin::ref();
            r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(cmd));
            r.cmd_len = 1;
            (void)Host::start(r);
        });
        bench_line(serial, op, bytes, s, Ruler::hz(), spi_sck_hz(SysClock::hz, req_baud) / 8u);
        print(serial, "  cmd 1 + data ", len, ", status ", Host::status(), ": fixed = wall - wire = ",
              s.wall - wire_cycles(bytes, req_baud), " cycles", crlf);
    }
}

}  // namespace ee

void te_spi_host() {
    for (uint32_t i = 0; i < 256u; ++i) {
        dd::spi_tx[i] = static_cast<uint8_t>(i * 7u + 1u);
    }
    ee::CsPin::output();
    ee::CsPin::set();
    ee::DcPin::output();
    ee::DcPin::clear();
    ee::live = true;
    if (!ee::SpiPoll::init(clock)) {
        print(serial, "  the engineless SPI host did not come up", crlf);
        bench.verdict("ran", false);
        return;
    }
    ee::poll();
    ee::pump();
    ee::req<ee::SpiPoll>("spi.req");
    ee::SpiPoll::release();
    ee::live = false;

    if (!Dmac::init() || !dd::SpiHw::init(clock)) {
        print(serial, "  the engined SPI host did not come up", crlf);
        bench.verdict("ran", false);
        return;
    }
    ee::req<dd::SpiHw>("spi.req.eng");
    dd::SpiHw::release();
    Dmac::release();
    ee::CsPin::release();
    ee::DcPin::release();
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
extern "C" BENCH_PLACEMENT void DMAC_Handler() {
    dmac_meter.enter();
    while (const auto irq = brio::Dmac::take_pending()) {
        dd::dmac_dispatch(*irq);
    }
    dmac_meter.leave();
}
extern "C" BENCH_PLACEMENT void SERCOM1_Handler() {
    spi_meter.enter();
    if (ee::live) {
        if (ee::SpiPoll::isr()) {
            ee::done = true;
        }
    } else if (dd::SpiHw::isr()) {
        dd::spi_done = true;
    }
    spi_meter.leave();
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
    bench.letter('d', "the DMA: copy, fill, paced, spi.dma", td_dma);
    bench.letter('e', "the SPI host above the wire: spi.poll, spi.pump, spi.req", te_spi_host);

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
