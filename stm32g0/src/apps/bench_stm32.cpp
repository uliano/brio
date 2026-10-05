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
// instruction cache that is on at reset and the prefetch that
// Clock::init() turns on - this family's default - so letters r, m, p
// and t measure the default, and letter f is the prefetch as a column.
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
//      irq and isr are the transport's shape. USART2 is a FULL instance,
//      so the default Uart runs it in FIFO mode (stm32g0/usart.hpp's
//      UartTask::fifo_mode) and paces the transmitter on TXFT at "TXFIFO
//      becomes empty": one USART2 interrupt per eight bytes, each refill
//      taking the ring as a run, plus two at the print's start - the
//      first byte goes straight through the idle FIFO into the shift
//      register and the vector is entered once more to find the
//      transmitter disarmed, and the second, pushed before the first has
//      left, goes out alone (2, 5, 35 and 515 entries for the four
//      sizes) - plus the ticks of the run; the line after each bench line
//      gives the two vectors apart.
//      TC means the TXFIFO AND the shift register empty in this mode
//      (RM0444 33.5.5), so the drain still ends on the last frame out.
//   t  the tick's floor: one second (hz cycles on the ruler) of masked
//      Idle::idle() turns with the console drained - not a kernel loop:
//      no kernel runs here. wall = the second, irq = the ticks, isr = the
//      tick handler's cycles, busy = the floor. n=0, wire=0.
//   d  THE DMA ENGINES (stm32g0/dma.hpp), each op the best of 8 runs, the
//      completion an interrupt awaited on the masked idle path (one meter,
//      `dma_meter`, on the three DMA vectors and SPI1's), and after each
//      line the start call alone - "the launch call" / "start()", timed
//      with interrupts masked, one ruler read in it:
//        copy, fill  DmaCopyEngine on DMA1 channel 1, word beats, 16, 256
//                    and letter m's largest size; the data checked after;
//                    the cycles a word by the difference of the two larger
//                    walls and each wall's fixed cost beside it;
//        paced       256 words from a table into one RAM cell by
//                    DmaTxEngine<1, 4, uint32_t> on TIM3's update at 100
//                    kHz, the counter restarted from zero every run so the
//                    first item lands one period after the start: the wall
//                    against the 256 periods, the jitter the spread of the
//                    eight walls;
//        spi.dma     SpiHost<1> on its two engines (uint16_t, DMA1 channels
//                    2 and 3), SPI1 on PB3/PB4/PB5 with nothing wired - MISO
//                    floats: 16 and 256 frames of 8 bits (`spi.dma`) and of
//                    16 (`spi.dma.w16`) at PCLK/2 and PCLK/8, the wall
//                    against the wire.
//   e  THE SPI HOST ABOVE THE WIRE (stm32g0/spi.hpp): SpiHost<1> WITHOUT
//      engines on letter d's pads, MISO floating, each op the best of 8
//      runs of start() alone (the Request built outside the clock, as a
//      client builds it; on the pump the wait for the completion on the
//      masked idle path is inside the clock too), the frames read back
//      into a buffer where the shape reads, every line followed by the
//      wire's cycles, "the rest" above them, and SPI1's FTLVL, FRLVL and
//      OVR read right after the run (both levels zero and the flag clear
//      after every shape: a receive shape that moved every frame both
//      ways, a transmit-only one that cleared the overrun its unread
//      echoes raise; a frame lost to an overrun never completes the pump,
//      which the 20 ms budget and recover() would then report):
//        spi.poll    16 and 256 frames of 8 bits (`spi.poll`) and of 16
//                    (`spi.poll16`) at PCLK/4 and PCLK/16, POLLED - the
//                    whole transaction inside start(): wall against the
//                    wire - in the WRITE shape (rx null: the transmit-only
//                    loop, nothing read back) and, as `.rx`, in the receive
//                    shape (the frames read back into a buffer, the
//                    answers paced by count);
//        spi.pump    the same four on the frame pump, SPI1's own vector
//                    counted apart (the last run's): its interrupts per
//                    frame (one per RXNE where the handler keeps up with
//                    the wire, fewer where it does not), its cycles per
//                    entry and per frame;
//        spi.req     THE FIXED COST OF A REQUEST, a DCS command's shapes: a
//                    polled command of one byte and 0, 2 and 15 bytes of
//                    data, write-only, the D/C scripted on PB6 and the
//                    select on PA15 (two spare pads nothing is wired to),
//                    at PCLK/16; "the rest" above the wire is the price of
//                    a command.
//   f  THE PREFETCH (FLASH_ACR.PRFTEN, RM0444 3.3.5) as a column of its
//      own: the instrument's three cost lines and letters m, p and t run
//      TWICE through stm32g0/flash.hpp's FlashAccel::prefetch() - off,
//      then on - every op suffixed `.pf0` / `.pf1`, and the state the
//      boot left (on, Clock::init()'s default) restored after. The
//      instruction cache (ICEN, on at reset) stays on in both columns.
//      The erratum that would forbid the prefetch, ES0548 2.2.10 (a
//      prefetch may fail on a branch ACROSS BANKS), cannot bite this
//      image: stm32g0/ld/stm32g0b1re.ld gives the linker bank 1 alone,
//      and the G071 and G031 have one bank.
//
//   u  THE SERIAL TRANSPORT ON A LOOP OF ITS OWN (stm32g0/usart.hpp's
//      UartTask): USART1 on PA9, a vector of its own (`loop_meter`) and
//      DMA1 channels 2 and 3 for its engines. uart.tx: 256 and 4096
//      bytes through write_bulk() at 115200, 1 M and 2 Mbaud, full duplex
//      with the receiver on PA10's pull-up, through the interrupt
//      transmitter and the transmit engine, to the last stop bit (TC);
//      busy is the transport's share (the write_bulk() calls that took
//      bytes and the handlers), the thread's wait being a spin.
//      uart.rx: bursts of 16 and 256 bytes round the single wire, the
//      transmitter on its engine so USART1's vector is the receiver's,
//      through the paced receiver and the receive engine. uart.edge: a
//      burst of 17 with the consumer draining on every edge a vector
//      reports, the last edge against TC rising, signed. And the copy
//      into the ring: write_bulk's byte loop against the runtime's
//      memcpy at the lengths a run takes, ends aligned alike and not.
//      2 Mbaud is the loop's top rate: the single wire's rise on the
//      pad's own pull-up (docs/stm32g0/usart.md).
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
//   copy    a word in two cycles x hz = 128 000 000 B/s (and fill): RM0444
//           10.4.3, a single DMA transfer is a read and a write over the
//           controller's one AHB master - the same floor as memcpy's.
//   paced   4 bytes x 100 kHz = 400 000 B/s, the request's own rate.
//   spi.dma SCK / 8 bytes a second: PCLK/2 = 32 MHz -> 4 000 000 B/s,
//           PCLK/8 -> 1 000 000; a 16-bit frame is two bytes in sixteen
//           bit times, the same rate.
//   spi.poll, spi.pump, spi.req  the same wire: PCLK/4 = 16 MHz ->
//           2 000 000 B/s, PCLK/16 -> 500 000.
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
#include <span>

#include "stm32g0/clock.hpp"
#include "stm32g0/delay.hpp"
#include "stm32g0/dma.hpp"
#include "stm32g0/flash.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/platform.hpp"
#include "stm32g0/spi.hpp"
#include "stm32g0/ticker.hpp"
#include "stm32g0/tim.hpp"
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
IsrMeter<Ruler, Idle> dma_meter;   // letter d: the three DMA vectors and SPI1's
IsrMeter<Ruler, Idle> loop_meter;  // letter u: USART1's vector

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
BenchCounters counters() { return counters_of(usart_meter, tick_meter, dma_meter, loop_meter); }

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
    const bool booted = FlashAccel::prefetch();   // on: Clock::init()'s default
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

    FlashAccel::prefetch(booted);   // the state the boot left
    print_acr();
    bench.verdict("ran", true);
}

// =============================================================================
// d - the DMA: the engines' cost (stm32g0/dma.hpp)
// =============================================================================
/// A word moved in two cycles: 10.4.3's single transfer is a read and a
/// write over the controller's one AHB master.
constexpr uint32_t dma_wire_bps = 2u * SysClock::hz;

// -- copy and fill: DmaCopyEngine on DMA1 channel 1, word beats
using Copy = DmaCopyEngine<1, 1>;

bool copy_words(uint32_t* dst, const uint32_t* src, uint16_t words, bool fill) {
    return fill ? Copy::fill(dst, src, words) : Copy::copy(dst, src, words);
}

void copy_vector() {
    const uint8_t f = Copy::service();
    if ((f & Copy::flag_error) != 0u) {
        (void)Copy::abandon();
    } else if ((f & Copy::flag_complete) != 0u) {
        (void)Copy::complete();
    }
}

// -- paced: 256 words from a table into one cell on TIM3's update
using T3 = Tim<3>;
using Paced = DmaTxEngine<1, 4, uint32_t>;
constexpr uint32_t paced_hz = 100'000u;
constexpr uint32_t paced_period = SysClock::hz / paced_hz;   // 640 cycles
constexpr uint16_t paced_items = 256u;
static_assert(SysClock::hz % paced_hz == 0u);
uint32_t paced_table[paced_items];
volatile uint32_t paced_cell = 0;

bool paced_start(const uint32_t* table, uint16_t n) {
    return Paced::start(std::span<const uint32_t>(table, n));
}

void paced_vector() {
    const uint8_t f = Paced::service();
    if ((f & Paced::flag_error) != 0u) {
        (void)Paced::abandon();
    } else if ((f & Paced::flag_complete) != 0u) {
        (void)Paced::complete();
    }
}

// -- the engined SPI host: test_stm32_spi's SPI1 pads, NOTHING WIRED to
// them - MISO floats, and the time is the wire's whatever comes back.
using SpiTx = DmaTxEngine<1, 2, uint16_t>;
using SpiRx = DmaRxEngine<1, 3, uint16_t>;
constexpr SpiPins spi_pins{
    .sck = {'B', 3, PinFunction::af0},
    .miso = {'B', 4, PinFunction::af0},
    .mosi = {'B', 5, PinFunction::af0},
    .nss = {},
};
using SpiHw = SpiHost<1, spi_pins, SpiTx, SpiRx>;
volatile bool spi_done = false;
volatile bool spi_live = false;   // read by the vectors: a plain store would be dead to the compiler
alignas(4) uint8_t spi_tx[512];
alignas(4) uint8_t spi_rx[512];

/// The THREAD'S own cost of starting a block: the call alone, timed with
/// interrupts masked (so a completion that lands at once is served after
/// the second read, not inside the window), the completion waited for
/// outside the clock; the shortest of 8 runs, one ruler read in it.
template <typename Prep, typename Launch, typename Wait>
uint32_t launch_cost(Prep prep, Launch launch, Wait wait) {
    uint32_t best = 0;
    for (uint8_t run = 0; run < 8u; ++run) {
        prep();
        disable_interrupts();
        const uint32_t t0 = Ruler::now();
        asm volatile("" ::: "memory");
        const bool started = launch();
        asm volatile("" ::: "memory");
        const uint32_t d = Ruler::now() - t0;
        enable_interrupts();
        (void)wait(started);
        if (run == 0u || d < best) {
            best = d;
        }
    }
    return best;
}

/// Idle (the masked call of Idle::idle()) until `pending()` falls; false
/// when `budget_ms` ran out first.
template <typename Pending>
bool idle_while(Pending pending, uint32_t budget_ms) {
    const uint32_t t0 = Ticker::millis();
    for (;;) {
        disable_interrupts();
        if (!pending()) {
            enable_interrupts();
            return true;
        }
        if (Ticker::millis() - t0 > budget_ms) {
            enable_interrupts();
            return false;
        }
        Idle::idle();
    }
}

/// best_of_8 with the wait inside the op, a preparation outside the
/// clock, every run's success and the worst wall kept.
template <typename Prep, typename Op>
BenchSample best_of_8_dma(Prep prep, Op op, bool& ok, uint32_t& worst) {
    (void)drain();
    BenchSample best{};
    worst = 0;
    ok = true;
    Stopwatch<Ruler> sw;
    for (uint8_t run = 0; run < 8u; ++run) {
        prep();
        const BenchCounters c0 = counters();
        sw.start();
        const bool done = op();
        const uint32_t wall = sw.elapsed();
        const BenchSample s = bench_sample(wall, c0, counters());
        asm volatile("" ::: "memory");
        ok = ok && done;
        if (wall > worst) {
            worst = wall;
        }
        if (run == 0u || s.wall < best.wall || (s.wall == best.wall && s.irq < best.irq)) {
            best = s;
        }
    }
    return best;
}

void run_copy(bool fill) {
    uint32_t* const dst = reinterpret_cast<uint32_t*>(mem_dst);
    const uint32_t* const src = reinterpret_cast<const uint32_t*>(mem_src);
    static uint32_t cell = 0;
    for (uint32_t i = 0; i < mem_max; ++i) {
        mem_src[i] = static_cast<uint8_t>(i * 7u + 1u);
    }
    uint32_t walls[3] = {};
    uint8_t k = 0;
    for (const uint32_t n : std::array<uint32_t, 3>{16u, 256u, mem_max}) {
        const uint16_t words = static_cast<uint16_t>(n / 4u);
        cell = 0xA5000000u | n;
        bool ok = false;
        uint32_t worst = 0;
        const BenchSample s = best_of_8_dma(
            [n] { std::memset(mem_dst, 0, n); },
            [&] {
                if (!copy_words(dst, fill ? &cell : src, words, fill)) {
                    return false;
                }
                return idle_while([] { return Copy::busy(); }, 10u);
            },
            ok, worst);
        bool data = ok;
        for (uint32_t i = 0; i < words && data; ++i) {
            data = fill ? dst[i] == cell : dst[i] == src[i];
        }
        bench_line(serial, fill ? "fill" : "copy", n, s, Ruler::hz(), dma_wire_bps);
        const uint32_t launch = launch_cost(
            [] {}, [&] { return copy_words(dst, fill ? &cell : src, words, fill); },
            [](bool started) { return started && idle_while([] { return Copy::busy(); }, 10u); });
        print(serial, "  data ", data ? "exact" : "WRONG", ", worst wall ", worst,
              ", the launch call ", launch, crlf);
        walls[k++] = s.wall;
    }
    // The DMA's own time per word, by the difference of the two larger
    // sizes, and what is left of each wall: the fixed cost of a block.
    const uint32_t big_words = mem_max / 4u;
    const uint32_t per_word_x100 = (walls[2] - walls[1]) * 100u / (big_words - 64u);
    print(serial, "  ", fill ? "fill" : "copy", ": ", per_word_x100 / 100u, '.',
          per_word_x100 % 100u < 10u ? "0" : "", per_word_x100 % 100u,
          " cycles a word (", mem_max, " - 256 bytes); fixed cost");
    const uint32_t sizes[3] = {16u, 256u, mem_max};
    for (uint8_t i = 0; i < 3u; ++i) {
        const uint32_t dma = (sizes[i] / 4u) * per_word_x100 / 100u;
        print(serial, " n=", sizes[i], ':', walls[i] > dma ? walls[i] - dma : 0u);
    }
    print(serial, crlf);
}

void run_paced() {
    for (uint16_t i = 0; i < paced_items; ++i) {
        paced_table[i] = 0x5A000000u | i;
    }
    T3::init();
    (void)T3::configure({.prescaler = 0, .period = paced_period - 1u});
    Paced::arm(&paced_cell, T3::dma_update_request());
    bool ok = false;
    uint32_t worst = 0;
    const BenchSample s = best_of_8_dma(
        [] {
            // The counter from zero and the request line down, so the
            // first item lands one whole period after the start.
            T3::enable(false);
            T3::interrupts(T3::update_dma, false);
            T3::set_count(0);
            T3::clear_flags(TIM_SR_UIF);
            T3::interrupts(T3::update_dma, true);
        },
        [] {
            if (!paced_start(paced_table, paced_items)) {
                return false;
            }
            T3::enable(true);
            const bool done = idle_while([] { return Paced::busy(); }, 20u);
            T3::enable(false);
            return done;
        },
        ok, worst);
    // The launch alone, the timer stopped so nothing is served under it.
    const uint32_t launch = launch_cost(
        [] {}, [] { return paced_start(paced_table, paced_items); },
        [](bool started) {
            if (started) {
                Paced::stop();
            }
            return true;
        });
    T3::interrupts(T3::update_dma, false);
    T3::release();
    Paced::stop();
    constexpr uint32_t nominal = paced_items * paced_period;
    bench_line(serial, "paced", paced_items * 4u, s, Ruler::hz(), paced_hz * 4u);
    print(serial, "  ", paced_items, " words at ", paced_hz, " Hz into one cell: nominal ",
          nominal, " cycles, best wall ", s.wall, " (+", s.wall - nominal, "), worst ", worst,
          " (+", worst - nominal, "), jitter ", worst - s.wall, ", the launch call ", launch,
          ", cell ", hex(paced_cell),
          ok ? "" : " - A RUN TIMED OUT", crlf);
}

void run_spi(SpiDataSize bits, SpiClock rate, uint16_t div) {
    const bool wide = bits == SpiDataSize::bits16;
    for (const uint16_t frames : {uint16_t{16}, uint16_t{256}}) {
        const uint32_t bytes = wide ? 2u * frames : frames;
        bool ok = false;
        uint32_t worst = 0;
        const BenchSample s = best_of_8_dma(
            [] {},
            [=] {
                SpiHw::Request r{};
                r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(spi_tx));
                r.rx = lend<Lease::reply>(spi_rx);
                r.len = frames;
                r.clock = rate;
                r.mode = SpiMode::mode0;
                r.bits = bits;
                r.polled = false;
                spi_done = false;
                if (SpiHw::start(r)) {
                    return true;
                }
                if (!idle_while([] { return !spi_done; }, 20u)) {
                    (void)SpiHw::recover();
                    return false;
                }
                return true;
            },
            ok, worst);
        // start() alone: the Request built outside the clock.
        SpiHw::Request lr{};
        lr.tx = lend<Lease::reply>(static_cast<const uint8_t*>(spi_tx));
        lr.rx = lend<Lease::reply>(spi_rx);
        lr.len = frames;
        lr.clock = rate;
        lr.mode = SpiMode::mode0;
        lr.bits = bits;
        const uint32_t launch = launch_cost(
            [] { spi_done = false; }, [&] { return SpiHw::start(lr); },
            [](bool done) {
                if (done || idle_while([] { return !spi_done; }, 20u)) {
                    return true;
                }
                (void)SpiHw::recover();
                return false;
            });
        const uint32_t wire_bps = SysClock::hz / div / 8u;
        bench_line(serial, wide ? "spi.dma.w16" : "spi.dma", bytes, s, Ruler::hz(), wire_bps);
        const uint32_t wire_cycles = bytes * 8u * div;
        print(serial, "  ", frames, wide ? " 16-bit" : " 8-bit", " frames at PCLK/", div,
              ": the wire ", wire_cycles, " cycles, the rest ",
              s.wall > wire_cycles ? s.wall - wire_cycles : 0u, ", worst wall ", worst,
              ", start() ", launch,
              ok ? "" : " - A TRANSACTION TIMED OUT", crlf);
    }
}

void td_dma() {
    print(serial, "  copy/fill: DMA1 channel 1, word beats, memory to memory; paced: "
                  "DMA1 channel 4 on TIM3's update; spi: DmaTxEngine<1, 2> + "
                  "DmaRxEngine<1, 3> (uint16_t)",
          crlf);
    Copy::arm();
    run_copy(false);
    run_copy(true);
    run_paced();
    for (uint16_t i = 0; i < 512u; ++i) {
        spi_tx[i] = static_cast<uint8_t>(0x30u + i);
    }
    (void)SpiHw::init(clock);
    spi_live = true;
    run_spi(SpiDataSize::bits8, SpiClock::div2, 2u);
    run_spi(SpiDataSize::bits8, SpiClock::div8, 8u);
    run_spi(SpiDataSize::bits16, SpiClock::div2, 2u);
    run_spi(SpiDataSize::bits16, SpiClock::div8, 8u);
    spi_live = false;
    SpiHw::release();
    bench.verdict("ran", true);
}

// =============================================================================
// e - the SPI host above the wire: the polled loop, the pump, a request's
//     fixed cost (stm32g0/spi.hpp over design/spi-bus.md)
// =============================================================================
/// The ENGINELESS host on letter d's pads: its polled loop and its frame
/// pump are what an engined host hands the command phase to, and what a
/// host with no engines runs for everything.
using SpiSw = SpiHost<1, spi_pins>;
volatile bool sw_live = false;   // the SPI1 vector's switch, as spi_live is
/// The select and the D/C of `spi.req`, two spare pads nothing is wired
/// to: PA15 (the SPI suite's own chip select on this board) and PB6.
using ReqCs = Pin<'A', 15>;
using ReqDc = Pin<'B', 6>;
static const uint8_t req_cmd[1] = {0x2C};   // a DCS memory write's opcode

/// SPI1's vector alone, between two instants: its count and its cycles,
/// read under the mask as counters() are.
struct VectorSample {
    uint32_t irq;
    uint32_t isr;
};
VectorSample spi_vector() {
    const Idle::CriticalSection cs;
    return {dma_meter.count(), dma_meter.cycles()};
}

/// One operation of letter e through `Host`: `frames` frames of `bits`
/// at `rate`, polled or on the pump, after an optional command phase of
/// `cmd_len` frames with the D/C scripted; the best of 8 runs on the
/// Stopwatch, every run's completion required. The bench line, then a
/// line with the wire's cycles, "the rest" above them, the FIFO levels
/// and the overrun flag read after the run, and - on the pump - SPI1's
/// own interrupts and cycles per frame.
template <typename Host>
void run_host(const char* op, SpiDataSize bits, SpiClock rate, uint16_t div, bool polled,
              uint16_t frames, uint8_t cmd_len, PinRef cs, PinRef dc, bool read_back) {
    const bool wide = bits == SpiDataSize::bits16;
    const uint32_t bytes = (wide ? 2u : 1u) * (static_cast<uint32_t>(frames) + cmd_len);
    // The Request, built OUTSIDE the clock: what is timed is start() and,
    // on the pump, the wait for its completion on the idle path.
    typename Host::Request r{};
    r.cs = cs;
    r.dc = dc;
    if (cmd_len != 0u) {
        r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(req_cmd));
        r.cmd_len = cmd_len;
    }
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(spi_tx));
    if (read_back) {
        r.rx = lend<Lease::reply>(spi_rx);
    }
    r.len = frames;
    r.clock = rate;
    r.mode = SpiMode::mode0;
    r.bits = bits;
    r.polled = polled;

    (void)drain();
    BenchSample best{};
    uint32_t worst = 0;
    bool ok = true;
    uint8_t ftlvl = 0;
    uint8_t frlvl = 0;
    bool ovr = false;
    VectorSample v0{};
    VectorSample v1{};
    Stopwatch<Ruler> sw;
    for (uint8_t run = 0; run < 8u; ++run) {
        spi_done = false;
        v0 = spi_vector();
        const BenchCounters c0 = counters();
        sw.start();
        bool done = Host::start(r);
        if (!done) {
            done = idle_while([] { return !spi_done; }, 20u);
        }
        const uint32_t wall = sw.elapsed();
        const BenchSample s = bench_sample(wall, c0, counters());
        asm volatile("" ::: "memory");
        // The flags right after the run; the levels must both be zero on
        // a transaction that moved every frame both ways.
        ftlvl = Host::Resource::tx_level();
        frlvl = Host::Resource::rx_level();
        ovr = Host::Resource::overrun();
        v1 = spi_vector();
        if (!done) {
            (void)Host::recover();
        }
        ok = ok && done;
        if (wall > worst) {
            worst = wall;
        }
        if (run == 0u || s.wall < best.wall || (s.wall == best.wall && s.irq < best.irq)) {
            best = s;
        }
    }
    const BenchSample& s = best;
    const uint32_t wire_bps = SysClock::hz / div / 8u;
    bench_line(serial, op, bytes, s, Ruler::hz(), wire_bps);
    const uint32_t wire_cycles = bytes * 8u * div;
    print(serial, "  ", cmd_len != 0u ? "cmd 1 + " : "", frames, wide ? " 16-bit" : " 8-bit",
          " frames at PCLK/", div, ": the wire ", wire_cycles, " cycles, the rest ",
          s.wall > wire_cycles ? s.wall - wire_cycles : 0u, ", worst wall ", worst,
          "; after the run FTLVL ", ftlvl, " FRLVL ", frlvl, " OVR ", ovr ? 1u : 0u);
    if (!polled) {
        const uint32_t irq = v1.irq - v0.irq;
        const uint32_t isr = v1.isr - v0.isr;
        const uint32_t all = static_cast<uint32_t>(frames) + cmd_len;
        print(serial, "; SPI1 vector (the last run): irq ", irq, " = ",
              (irq * 100u + all / 2u) / all, "/100 a frame, isr ", isr, " = ",
              (isr + irq / 2u) / (irq == 0u ? 1u : irq), " an entry, ",
              (isr + all / 2u) / all, " a frame");
    }
    print(serial, ok ? "" : " - A TRANSACTION TIMED OUT", crlf);
}

void te_host() {
    print(serial, "  SpiHost<1> without engines on SPI1 PB3/PB4/PB5, MISO floating; "
                  "spi.req's select PA15, D/C PB6",
          crlf);
    for (uint16_t i = 0; i < 512u; ++i) {
        spi_tx[i] = static_cast<uint8_t>(0x30u + i);
    }
    ReqCs::output(true);
    ReqDc::output(true);
    (void)SpiSw::init(clock);
    sw_live = true;
    // The polled loop and the pump in both shapes - the WRITE (rx null:
    // nothing read back) and the receive (`.rx`: the frames read back
    // into a buffer) - 16 and 256 frames, both widths, at PCLK/4 and
    // PCLK/16, the select on PA15 and no D/C.
    struct Op {
        const char* name;
        bool wide;
        bool polled;
        bool read_back;
    };
    constexpr Op ops[8] = {
        {"spi.poll", false, true, false},     {"spi.poll.rx", false, true, true},
        {"spi.pump", false, false, false},    {"spi.pump.rx", false, false, true},
        {"spi.poll16", true, true, false},    {"spi.poll16.rx", true, true, true},
        {"spi.pump16", true, false, false},   {"spi.pump16.rx", true, false, true},
    };
    for (const Op& op : ops) {
        const SpiDataSize bits = op.wide ? SpiDataSize::bits16 : SpiDataSize::bits8;
        for (const uint16_t div : {uint16_t{4}, uint16_t{16}}) {
            const SpiClock rate = div == 4u ? SpiClock::div4 : SpiClock::div16;
            for (const uint16_t frames : {uint16_t{16}, uint16_t{256}}) {
                run_host<SpiSw>(op.name, bits, rate, div, op.polled, frames, 0, ReqCs::ref(), {},
                                op.read_back);
            }
        }
    }
    // The fixed cost of a request: a polled command of one byte and 0, 2
    // and 15 bytes of data (a DCS command's shapes), write-only, the D/C
    // scripted, at PCLK/16.
    for (const uint16_t data : {uint16_t{0}, uint16_t{2}, uint16_t{15}}) {
        run_host<SpiSw>("spi.req", SpiDataSize::bits8, SpiClock::div16, 16u, true, data, 1,
                        ReqCs::ref(), ReqDc::ref(), false);
    }
    sw_live = false;
    SpiSw::release();
    bench.verdict("ran", true);
}


// =============================================================================
// u - the serial transport on a loop of its own: USART1 on PA9
//     (stm32g0/usart.hpp's UartTask over design/serial.md)
// =============================================================================
/// USART1_TX on PA9 and USART1_RX on PA10, AF1 (DS13560 table 13): two pads
/// nothing on the Nucleo-64s drives. The TRANSMIT ops run full duplex with
/// the receiver on PA10's pull-up, so USART1's vector serves the
/// transmitter alone; the RECEIVE ops run the single wire (33.5.15), the
/// instance hearing its own frames, with the transmitter on its engine so
/// that USART1's vector serves the receiver alone.
constexpr UartPins loop_pins{
    .tx = {'A', 9, PinFunction::af1},
    .rx = {'A', 10, PinFunction::af1},
};
constexpr UartOptions single_wire = uart_half_duplex();

/// The rings: a 256-byte burst needs a 512-byte receive ring; the 8 KB part
/// takes a quarter of everything (and its long burst is 64 bytes).
constexpr bool loop_big = SRAM_SIZE_MAX >= 0x4000UL;
constexpr uint32_t loop_rx_ring = loop_big ? 512u : 128u;
constexpr uint32_t loop_tx_ring = loop_big ? 256u : 64u;
constexpr uint32_t loop_long = loop_big ? 256u : 64u;

using LoopTx = DmaTxEngine<1, 2>;
using LoopRx = DmaRxEngine<1, 3>;
using TxPlain = Uart<1, loop_pins, 16, loop_tx_ring>;
using TxEngined = Uart<1, loop_pins, 16, loop_tx_ring, LoopTx>;
using RxPlain = Uart<1, loop_pins, loop_rx_ring, loop_tx_ring, LoopTx, NoDmaEngine, single_wire>;
using RxEngined = Uart<1, loop_pins, loop_rx_ring, loop_tx_ring, LoopTx, LoopRx, single_wire>;

enum class LoopMode : uint8_t { none, tx_plain, tx_engined, rx_plain, rx_engined };
volatile LoopMode loop_mode = LoopMode::none;   // read by the vectors
volatile uint32_t edge_at = 0;                  // the ruler at the last edge a vector reported
volatile uint32_t edge_count = 0;

/// The rates: the console's, about 1 Mbaud, and the loop's top rate - 2
/// Mbaud, where the single wire's rise on the pad's own 40 k pull-up still
/// carries a frame (docs/stm32g0/usart.md, "The baud generator"); the
/// generator itself reaches 4 Mbaud (USARTDIV 16 at 64 MHz).
constexpr std::array<uint32_t, 3> loop_rates{115200u, 1'000'000u, 2'000'000u};

/// The vector pair apart, between two instants.
struct LoopSnap {
    uint32_t usart_irq, usart_isr, dma_irq, dma_isr;
};
LoopSnap loop_snap() {
    const Idle::CriticalSection cs;
    return {loop_meter.count(), loop_meter.cycles(), dma_meter.count(), dma_meter.cycles()};
}
void print_loop(const LoopSnap& a, const LoopSnap& b) {
    print(serial, "  usart1: irq=", b.usart_irq - a.usart_irq, " isr=", b.usart_isr - a.usart_isr,
          "  dma: irq=", b.dma_irq - a.dma_irq, " isr=", b.dma_isr - a.dma_isr, crlf);
}

std::span<const uint8_t> payload_bytes(uint32_t from, uint32_t n) {
    return {reinterpret_cast<const uint8_t*>(payload.data()) + from, n};
}

template <typename T>
bool loop_up(LoopMode mode, uint32_t baud) {
    // The vector serves T from the init on: the receiver time-out rises
    // as the receiver is enabled (33.8.10's note), before a byte moves.
    loop_mode = mode;
    const bool ok = T::init(clock, baud);
    T::clear_errors();
    // TE's idle frame (33.5.5) and anything the pad saw on the way.
    const uint32_t frame = SysClock::hz / (baud / 10u);
    Stopwatch<Ruler> sw;
    sw.start();
    while (sw.elapsed() < 4u * frame) {
    }
    uint8_t junk = 0;
    while (T::read_byte(junk)) {
    }
    return ok;
}

bool loop_tc() { return (Usart<1>::status() & UsartFlag::tc) != 0u; }

/// uart.tx: n bytes of the payload through write_bulk(), refused runs
/// retried, then the wire to its last stop bit (TC). busy is the
/// TRANSPORT'S share - every write_bulk() call that took bytes and the
/// handlers - because the thread here waits by spinning, which a program
/// with a kernel would spend idle.
template <typename T>
void run_uart_tx(const char* op, LoopMode mode, uint32_t baud, uint32_t n) {
    (void)drain();
    if (!loop_up<T>(mode, baud)) {
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
        const uint32_t k = T::write_bulk(payload_bytes(print_max - n + sent, n - sent));
        if (k != 0u) {
            thread += Ruler::now() - a;
            sent += k;
        }
    }
    while (!(T::tx_idle() && loop_tc()) && sw.elapsed() < budget) {
    }
    const uint32_t wall = sw.elapsed();
    const LoopSnap v1 = loop_snap();
    BenchSample s = bench_sample(wall, c0, counters());
    s.busy = thread + (v1.usart_isr - v0.usart_isr) + (v1.dma_isr - v0.dma_isr);
    const uint32_t actual = T::actual_baud(SysClock::pclk_hz);
    T::release();
    loop_mode = LoopMode::none;
    print(serial, "  ", op, " at ", baud, " baud (", actual, "), sent ", sent,
          ", the thread's write_bulk ", thread, crlf);
    bench_line(serial, op, n, s, Ruler::hz(), baud / 10u);
    print_loop(v0, v1);
}

/// uart.rx: a burst of n bytes round the single wire, the transmitter on
/// its engine; wall from the first byte queued to the last one in the
/// receive ring, the data checked after.
template <typename T>
void run_uart_rx(const char* op, LoopMode mode, uint32_t baud, uint32_t n) {
    (void)drain();
    if (!loop_up<T>(mode, baud)) {
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
    while (static_cast<uint32_t>(T::rx_pending()) < n && sw.elapsed() < budget) {
    }
    const uint32_t wall = sw.elapsed();
    const LoopSnap v1 = loop_snap();
    const BenchSample s = bench_sample(wall, c0, counters());
    uint8_t got[loop_long];
    const uint32_t read = T::read_bulk({got, n});
    uint32_t wrong = 0;
    for (uint32_t i = 0; i < read; ++i) {
        if (got[i] != static_cast<uint8_t>(payload[i])) {
            ++wrong;
        }
    }
    print(serial, "  ", op, " at ", baud, " baud: ", read, " of ", n, " back, ", wrong,
          " wrong, ORE ", T::hw_overruns(), " FE ", T::frame_errors(), " ring overruns ",
          T::rx_overruns(), crlf);
    T::release();
    loop_mode = LoopMode::none;
    bench_line(serial, op, n, s, Ruler::hz(), baud / 10u);
    print_loop(v0, v1);
}

/// uart.edge: a burst of 17 bytes - one past a multiple of any FIFO level,
/// so the tail is the edge's to deliver - with the consumer draining on
/// every edge a vector reports; wall = the ruler at the LAST edge minus the
/// ruler at the burst's last stop bit (TC rising), signed: an edge that
/// came before the stop bit ended prints negative.
template <typename T>
void run_uart_edge(const char* op, LoopMode mode, uint32_t baud) {
    constexpr uint32_t n = 17u;
    (void)drain();
    if (!loop_up<T>(mode, baud)) {
        print(serial, "  ", op, ": init refused at ", baud, crlf);
        return;
    }
    const uint32_t frame = SysClock::hz / (baud / 10u);
    edge_count = 0;
    uint32_t seen = 0;
    uint32_t got = 0;
    uint32_t wrong = 0;
    bool tc_seen = false;
    uint32_t tc_at = 0;
    Usart<1>::clear_flags(UsartClear::tc);
    (void)T::write_bulk(payload_bytes(0, n));
    const uint32_t t0 = Ruler::now();
    const uint32_t settle = 4u * frame + SysClock::hz / 250u;   // 4 ms past the tail
    for (;;) {
        if (!tc_seen && loop_tc()) {
            tc_at = Ruler::now();
            tc_seen = true;
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
        if ((tc_seen && now - tc_at > settle) || now - t0 > SysClock::hz / 10u) {
            break;
        }
    }
    T::release();
    loop_mode = LoopMode::none;
    const int32_t latency = static_cast<int32_t>(edge_at - tc_at);
    print(serial, "  ", op, " at ", baud, " baud: ", got, " of ", n, " bytes (", wrong,
          " wrong) on ", edge_count, " edges; the last edge ", latency, " cycles after the "
          "last stop bit = ", latency * 100 / static_cast<int32_t>(frame), "/100 of a frame",
          tc_seen ? "" : " (TC NEVER SEEN)", crlf);
    // The grammar's line with a SIGNED wall: the edge may precede the stop
    // bit's end (a receiver flag rises at the stop bit's middle).
    print(serial, "bench ", op, " n=", n, " wall=", latency, " busy=- irq=", edge_count,
          " isr=- rate=- wire=- x=-", crlf);
}

/// write_bulk()'s copy: its byte loop (the shape in stm32g0/usart.hpp's
/// write_bulk, two pointers and the test at the bottom) against the
/// runtime's memcpy, at the lengths a run takes, with the source and the
/// destination sharing their word alignment and not: the cycles of one
/// copy, over 64 copies, the ruler's own read taken out.
alignas(4) uint8_t copy_src[136];
alignas(4) uint8_t copy_dst[136];
volatile uint32_t copy_len = 0;

// The loop is write_bulk's own; the attribute keeps GCC from replacing
// it with a call to memcpy, which it does here (a global destination) and
// does not do inside write_bulk (the listing).
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
    (void)drain();
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
    print(serial, "  USART1 on PA9 (", loop_rx_ring, "/", loop_tx_ring, " rings): the transmit ops "
          "full duplex, RX on PA10's pull-up; the receive ops on the single wire, the "
          "transmitter on DmaTxEngine<1, 2>, the receive engine DmaRxEngine<1, 3>", crlf);
    for (const uint32_t baud : loop_rates) {
        for (const uint32_t n : {256u, 4096u}) {
            run_uart_tx<TxPlain>("uart.tx", LoopMode::tx_plain, baud, n);
            run_uart_tx<TxEngined>("uart.tx.dma", LoopMode::tx_engined, baud, n);
        }
    }
    for (const uint32_t baud : loop_rates) {
        for (const uint32_t n : {uint32_t{16}, loop_long}) {
            run_uart_rx<RxPlain>("uart.rx", LoopMode::rx_plain, baud, n);
            run_uart_rx<RxEngined>("uart.rx.dma", LoopMode::rx_engined, baud, n);
        }
    }
    for (const uint32_t baud : {115200u, 1'000'000u}) {
        run_uart_edge<RxPlain>("uart.edge", LoopMode::rx_plain, baud);
        run_uart_edge<RxEngined>("uart.edge.dma", LoopMode::rx_engined, baud);
    }
    bench.verdict("ran", true);
}

void banner() {
    print(serial, crlf, "bench_stm32 - the benchmark skeleton (util/bench.hpp), clk=", SysClock::hz,
          " Hz, console USART2 ", console_baud, " 8N1 (", Serial::actual_baud(SysClock::pclk_hz),
          " baud), ruler SysTick cycles", crlf);
    print_acr();   // the condition the plain letters run under
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
extern "C" void DMA1_Channel1_IRQHandler() {
    dma_meter.enter();
    copy_vector();
    dma_meter.leave();
}
extern "C" void DMA1_Channel2_3_IRQHandler() {
    dma_meter.enter();
    bool edge = false;
    switch (loop_mode) {
        case LoopMode::tx_engined:
            (void)TxEngined::dma_isr();
            break;
        case LoopMode::rx_plain:
            (void)RxPlain::dma_isr();
            break;
        case LoopMode::rx_engined:
            edge = RxEngined::dma_isr();
            break;
        default:
            if (spi_live && SpiHw::dma_isr()) {
                spi_done = true;
            }
            break;
    }
    dma_meter.leave();
    if (edge) {
        edge_at = Ruler::now();
        edge_count = edge_count + 1u;
    }
}
/// Letter u's loop. The switch picks the transport the letter brought up;
/// the edge's stamp is taken after the meter's, outside the metered body.
extern "C" void USART1_IRQHandler() {
    loop_meter.enter();
    bool edge = false;
    switch (loop_mode) {
        case LoopMode::tx_plain:
            edge = TxPlain::isr();
            break;
        case LoopMode::tx_engined:
            edge = TxEngined::isr();
            break;
        case LoopMode::rx_plain:
            edge = RxPlain::isr();
            break;
        case LoopMode::rx_engined:
            edge = RxEngined::isr();
            break;
        default:
            // Nothing of ours is up: silence the line rather than re-enter
            // for ever on a condition nobody clears.
            Nvic::disable(Usart<1>::irq());
            break;
    }
    loop_meter.leave();
    if (edge) {
        edge_at = Ruler::now();
        edge_count = edge_count + 1u;
    }
}
extern "C" void BRIO_STM32G0_DMA1_CH4_UP_HANDLER() {
    dma_meter.enter();
    paced_vector();
    dma_meter.leave();
}
extern "C" void SPI1_IRQHandler() {
    dma_meter.enter();
    if (spi_live && SpiHw::isr()) {
        spi_done = true;
    } else if (sw_live && SpiSw::isr()) {
        spi_done = true;
    }
    dma_meter.leave();
}
extern "C" void SysTick_Handler() {
    tick_meter.enter();
    brio::Ticker::tick();
    tick_meter.leave();
}

int main() {
    const bool clock_ok = SysClock::init();   // HSI16 -> PLL -> 64 MHz, two wait states, prefetch on
    const bool serial_ok = Serial::init(clock, console_baud);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    bench.letter('r', "the ruler's self-check and the instrument's cost", tr_ruler);
    bench.letter('m', "memcpy and memset, 1/16/256 bytes and the largest the SRAM holds twice",
                 tm_memory);
    bench.letter('p', "a print of 1/16/256/4096 bytes through the console", tp_print);
    bench.letter('t', "the tick's floor: one second of idle", tt_tick);
    bench.letter('d', "the DMA: copy and fill, a paced block, the engined SPI host", td_dma);
    bench.letter('e', "the SPI host above the wire: polled, pumped, a request's fixed cost",
                 te_host);
    bench.letter('f', "the prefetch: the costs, m, p and t with PRFTEN off, then on", tf_prefetch);
    bench.letter('u', "the serial transport on USART1's own loop: tx, rx, the edge", tu_uart);

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
