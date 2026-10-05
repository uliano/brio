// bench_rp2350 - the benchmark skeleton on an RP2350 (docs/design/benchmark.md),
// FROM ONE SOURCE ON BOTH ARCHITECTURES: the Cortex-M33 pair and the
// Hazard3 pair each build it from their own preset, and nothing below asks
// which one runs - every difference between the two cores is a name from
// rp2350/core.hpp, and the one word printed differently for each comes from
// `core_kind`, a value.
//
// NOT A TEST. Every letter prints its numbers as bench lines in the grammar
// of util/bench.hpp,
//
//   bench <op> n=<bytes> wall=<cycles> busy=<cycles> irq=<count> isr=<cycles> rate=<B/s> wire=<B/s> x=<wall/wire>
//
// and closes with one verdict, "ran", so that bin/brio reads its ALL: line.
// Letter r alone judges something, because a ruler that is wrong makes every
// other line wrong.
//
// THE CLOCK is test_rp2350_platform's: the 12 MHz crystal through the
// system PLL to clk_sys = 150 MHz, clk_peri = clk_sys (the UART's clock),
// clk_ref = the crystal undivided.
//
// THE RULER is TIMER1 with its SOURCE on clk_sys (rp2350/timer.hpp, datasheet
// 12.8.1.1): the system timer taken off the microsecond tick and onto the
// system clock, so its counter advances once a clk_sys cycle on both
// architectures - one ruler for two instruction sets, which no core counter
// of this chip is (SysTick exists on the Arm half alone). `Ruler::now()` is
// the counter's raw low word, ONE LOAD THROUGH THE APB BRIDGE (datasheet
// 2.2.4: TIMER1 sits in the APB segment, whose reads take three cycles at
// least); `Ruler::hz()` is clk_sys.
// It is hardware, so it counts through a masked window, and 2^32 of its
// cycles are 28.6 s - every operation here is far under that. TIMER0 is left
// alone, and the RISC-V platform timer stays what it is on this target: the
// ruler of delay_us on both halves and the kernel timebase on Hazard3.
//
// THE CONSOLE is UART0 on GP0 (TX) / GP1 (RX) under function 2 at 115200
// 8N1, the Debug Probe's bridge, bound as rp2350/src/apps/console.cpp binds
// it: `Uart<0, console_pins>` with its 64/256 rings, `isr_uart0` and
// `isr_systick` bound BY NAME - a vector-table slot on the M33, a dispatch
// entry of the crt on Hazard3, one source either way.
//
// NO KERNEL, NO ACTIVE OBJECT. The prompt loop and letters r and t idle
// through `Idle::idle()` directly - `BenchIdle<Rp2350Platform<>, Ruler>`,
// called with interrupts masked after the check, the kernel loop's own
// shape - so every idle turn of the program is a measured window.
//
// THE METERS: one `IsrMeter<Ruler, Idle>` per bound vector, `uart_meter` on
// isr_uart0 and `tick_meter` on isr_systick, enter() the handler's first
// statement and leave() its last, the driver's ISR body between them. WHAT
// THE STAMPS DO NOT SEE is the way into and out of the handler and the
// handler's own prologue and epilogue: on the M33 the exception entry and
// return, which are the hardware's (the basic frame stacked, the vector
// fetched); on Hazard3 the crt's dispatch, which is software
// (rp2350/src/glue/startup_rp2350_riscv.S: the trap table's jump, seventeen
// instructions to save the caller-saved registers, the meinext loop and the
// call, seventeen to restore, mret - forty-five instructions an external
// interrupt, thirty-seven a timer one). That path is counted as idle when
// the interrupt ended a window and as busy-but-not-isr when it preempted the
// thread, so `isr` is the bodies' cycles and not the interrupt's whole
// price, and the difference is the larger on Hazard3.
//
// What each letter prints:
//   r  THE RULER'S SELF-CHECK and the instrument's cost.
//      `delay_us(clock, 999)` timed on the ruler, judged AT LEAST 999 us of
//      clk_sys cycles and under 5 per cent more. 999 and not 1000: this
//      target's delay REFUSES a request of one kernel tick or more and spends
//      no time on it (rp2350/delay.hpp, delay_us_cap). The wait counts the
//      platform timer's microsecond, which is clk_ref - the crystal - and the
//      ruler counts clk_sys off the PLL, so the verdicts compare two clocks;
//      the delay's documented price is up to one microsecond late, far inside
//      the 5 per cent. Then three bench lines with n=0 and wire=0:
//        ruler   one Ruler::now(): 1000 PAIRS of back-to-back reads under the
//                mask, the mean difference of a pair - the time one read
//                takes, with no loop in it;
//        stamp   one enter()+leave() of a meter that is no vector's, with a
//                compiler fence after each pair (so the meter's fields are
//                loaded and stored as a handler does): a loop of 1000 turns
//                of one pair and the same loop with two pairs a turn, under
//                the mask, whose difference is 1000 pairs with no loop in it
//                - what a stamp pair adds to a handler's run. Not an empty
//                loop subtracted: on Hazard3 a loop whose body is one 16-bit
//                instruction runs a cycle slower a turn when that body lands
//                word-aligned (datasheet 3.8.7.10.1), and both loops here
//                have bodies the branch predictor serves alike;
//        window  100 ms of idle turns with nothing pending but the tick,
//                every field divided by the turns: one turn's wall, busy, irq
//                and isr - IT HOLDS THE TICK HANDLER, whose cycles are its
//                isr and part of its busy.
//      A plain line beside each carries its raw totals; the stamp's also
//      carries the meter's own reading of an empty body (the cost, below).
//   m  memcpy and memset of 1, 16, 256 and 4096 bytes between two static
//      word-aligned buffers (rt.cpp's word path), through VOLATILE FUNCTION
//      POINTERS so that the call is the runtime's and not a builtin the
//      compiler expanded in place (brio/rt/selftest.hpp's reason); each the
//      best of 8 runs by wall on a Stopwatch, interrupts on: busy = wall,
//      irq and isr whatever the tick took in the run kept.
//      THE WIRE is the core's load/store floor. Each core reaches the bus
//      through one 32-bit port for loads and stores, which performs one
//      access a cycle (datasheet chapter 3's opening, "Processor subsystem":
//      "each core can perform one instruction fetch and one load or store
//      access per cycle"), and the SRAM answers with no wait state (2.1.4:
//      the AHB5 fabric "offers zero-wait-state accesses everywhere"). On
//      Hazard3 the core's own table agrees: lw and sw are one cycle each
//      (3.8.7.1). The Cortex-M33 Technical Reference Manual publishes no
//      instruction timing, so the port is the floor stated for both cores. A
//      copy moves a word with one read and one write: 2 bytes a cycle, 300
//      MB/s at 150 MHz. A fill moves it with one write: 4 bytes a cycle,
//      600 MB/s.
//   p  print(serial, ...) of 1, 16, 256 and 4096 bytes - prefixes of one
//      static 4096-byte string, 62 characters and a CRLF in every 64 so the
//      capture stays legible - then a spin on Serial::tx_idle(), the
//      transport's own verb for "the ring is empty and the shifter has sent
//      the last stop bit" (UARTFR.BUSY clear). Counters before the print and
//      after the drain. THE WIRE is 115200 baud over the ten bits of an 8N1
//      frame: 11520 B/s. busy is wall (print spins on a full ring, the drain
//      spins on the flag); irq and isr are the transport's SHAPE
//      (pl011/uart.hpp: from an idle transmitter the first FIFO-depth bytes
//      go straight into the FIFO, then one handler per refill as the FIFO
//      falls through its level; a byte the full ring refuses writes
//      nothing) plus the ticks that landed in the print, so a plain line
//      under each splits them: the console's vector, and the tick's.
//   t  the tick's floor: one second of Idle::idle() turns with nothing to
//      do, counters before and after: wall = the second, irq = the ticks,
//      isr = the tick handler's cycles, busy = the floor.
//   d  THE DMA (rp2350/dma.hpp, datasheet 12.6), four operations:
//        copy   DmaCopyEngine moving 16, 256 and 4096 bytes as WORDS between
//               the two static buffers, polled: wall = the start to BUSY
//               falling, busy = wall, each the best of 8. THE WIRE is the
//               controller's own rate, one read and one write of up to 32
//               bits every clock (12.6's opening): 4 bytes a cycle, 600 MB/s.
//               A plain line beside each: the fixed cost, wall less one
//               cycle a word.
//        fill   the same sizes written from one word cell: the same wire.
//        paced  256 words from a table into ONE CELL, one a microsecond, the
//               request a DMA pacing timer's (DmaTimer<0>, X 1 / Y 150 of
//               clk_sys), the block's completion on line 0 - the core idles
//               through it, so busy is what the block cost the CPU and irq
//               the interrupts it took; the second of two runs, the first
//               running the handler and the idle path cold. THE WIRE is the pace, 4 MB/s. Then
//               the same block again with the core STAMPING THE RULER at
//               every step of the channel's count - a polled loop of a few
//               cycles a turn, which bounds the resolution - and the plain
//               line gives the 255 intervals against the 150-cycle pace:
//               the mean, how many lie within one polling turn of it, and
//               the extremes.
//        spi.dma  SPI0 as a host on its two engines (DmaTxEngine<4,
//               uint16_t>, DmaRxEngine<5, uint16_t>), 16 and 256 frames of
//               8 and of 16 bits at clk_peri / 2 (75 MHz) and clk_peri / 8
//               (18.75 MHz), on GP18/GP19/GP16 with GP17 as the select and
//               NOTHING ON THE WIRE: MISO floats and the time is the wire's
//               and the engines', the data unjudged (the suite's loop-back
//               judges it, test_rp2350_spi letter d). Wall = start() to the
//               completion on line 0, the core idling between, the best of
//               five after one thrown away (which runs cold out of the flash
//               and applies the new rate to the block). THE WIRE is
//               the SCK rate over eight bits a byte. The plain line beside
//               each gives the transaction's fixed cost - wall less the
//               wire's own cycles - which is the number this round is
//               judged by; another, per rate and width, the cycles a byte
//               by the difference of the two sizes. Then the same
//               transactions on the PL022's loop-back (LBM), plain lines
//               only, the cycles a byte beside the wire's: the instrument the
//               SPI suite's letter d judges the data on. And 256 bytes at
//               each rate with the core AWAKE while the engines run -
//               spinning on the RAM flag the handler sets, then spinning on
//               the ruler, a register behind the APB bridge the engines'
//               accesses to SSPDR cross too - beside the same block with the
//               core asleep.
//   e  THE SPI HOST ABOVE THE WIRE (brio/pl022/spi.hpp over SPI0 on the same
//      pads, GP17 the select and GP22 the D/C line, MISO floating as in d):
//      what the host's own code costs around and between the frames, on a
//      host WITHOUT engines so the polled loop and the pump are what runs.
//        spi.poll  a polled WRITE (no in buffer: a display's pixel path) of
//               16 and 256 frames at clk_peri / 2 (75 MHz), / 4 (37.5 MHz)
//               and / 16 (9.375 MHz), IN MODE 0 AND IN MODE 3 - the mode is an axis because
//               the block itself keeps 1.5 SCK periods between the frames of
//               a continuous transfer with SPH = 0 and none with SPH = 1
//               (12.3.4.10, and letter d's measurement), so mode 3 is where
//               the loop's own pace shows against the nominal wire. Wall =
//               start() to its return, busy = wall. The plain line beside
//               each pair: the cycles a frame by the difference of the two
//               sizes, against the wire's and the wire's with the gap.
//        spi.poll.rx  the same requests as a RECEIVE (the in buffer set),
//               the host's other polled shape.
//        spi.poll16, spi.poll16.rx  both shapes in 16-bit frames.
//               Then THE RECEIVE SHAPE'S PROOF: 100 receive requests of 256
//               frames on the loop-back at each rate with the tick's handler
//               stretched to 1500 cycles - past eight frame times at both
//               rates - each judged exact and the overrun flag read after
//               each: the frames in flight never exceed the receive FIFO's
//               depth, so a handler that holds the loop off cannot overrun
//               it.
//        spi.pump, spi.pump16  the receive requests on the interrupt, the
//               core idling to the completion: irq and isr are the SPI
//               vector's own (the tick's split out on the plain line).
//      The block's RECEIVE OVERRUN flag (SSPRIS.RORRIS) is read after every
//      run of every op - an overrun would be more frames in flight than the
//      receive FIFO holds; the write shape raises it by design and clears
//      it at its tail, so a RAISED here is a defect of the host.
//        spi.req  THE PRICE OF A DCS COMMAND: a polled request of a command
//               byte and 0, 2 and 15 data bytes (1, 3 and 16 bytes on the
//               wire), D/C low for the command and high for the data,
//               nothing read back, at 9.375 MHz: wall less the wire's
//               cycles is the host's fixed cost per request. Then the same three requests on letter d's
//               ENGINED host, plain lines: what a short polled request costs
//               on a host whose engines exist for the long ones.
//        spi.cold  THE SAME THREE-BYTE REQUEST WITH ITS CODE COLD: the
//               host's very first request after init() - as a program
//               meets it - and then the request with EVERY LINE of the XIP
//               cache invalidated first (Xip::invalidate_all(): the
//               repeatable reading, and the worst a program meets after a
//               phase that walked the flash), each with its warm repeat
//               right behind it and the cache's misses beside each (Xip's
//               two counters around the start). The bench line is the
//               invalidated one; the plain lines carry all four. Run FIRST
//               in the letter, before any other transaction of the host, so
//               that the first request is the first.
//      Each the best of five after one thrown away (cold out of the flash),
//      spi.cold apart - there the one run IS the measurement.
//      THE WIRE is the SCK rate over eight bits a byte, as in d.
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
//
// THE INSTRUMENT'S COST, and how a reader takes it out. Every number is RAW.
//  - A Stopwatch frames an operation between two ruler reads, inline, so
//    `wall` holds one read's latency beside the operation: subtract the
//    `ruler` figure.
//  - A stamp pair stretches the handler it sits in by the `stamp` figure:
//    `busy` holds that once per `irq`.
//  - The meter's own reading of an empty body (the plain line under `stamp`)
//    is what `isr` holds per `irq` beyond the bodies: isr - irq x that
//    figure is the bodies alone.
//  - `window`'s busy minus its isr is one idle turn's own cost - the loop's
//    check (a Stopwatch read here), the mask, the window's two stamps, the
//    sleep's entry and exit and the way into and out of the tick's handler -
//    which the tick's floor in `t` pays once a tick.
//
// The bench lines of a run carry the board's numbers and nothing else: the
// core, the clock, the ruler and the console's rate are one line printed by
// the banner and again at the head of letter r, so a capture of `z` has it.
//
// build: boards = weact2350b,weact2350b-rv
// build: monitor_speed = 115200

#include <stdint.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstring>
#include <string_view>

#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/delay.hpp"
#include "rp2350/dma.hpp"
#include "rp2350/flash.hpp"
#include "rp2350/mtime.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/spi.hpp"
#include "rp2350/ticker.hpp"
#include "rp2350/timer.hpp"
#include "rp2350/uart.hpp"
#include "util/bench.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 150'000'000UL>;
constexpr SysClock clock;
using P = brio::Rp2350Platform<>;

/// THE RULER (the file header): TIMER1 counting clk_sys. always_inline
/// because it is the stamps' one read: a call here would be inside every
/// measurement it takes.
///
/// AND A COMPILER BARRIER ON EITHER SIDE OF THE READ. The read is volatile,
/// which orders it against other volatile accesses and nothing else, so the
/// compiler is free to schedule a handler body's plain loads and stores
/// across a stamp - and does: without the fences the release image computes
/// the tick handler's counter increment ABOVE enter()'s read. The fences
/// emit no instruction; they make a read of the ruler a point the
/// program's memory accesses cannot cross.
struct Ruler {
    using Counter = brio::Timer<1>;
    [[gnu::always_inline]] static uint32_t now() {
        std::atomic_signal_fence(std::memory_order_seq_cst);
        const uint32_t t = Counter::now_low();
        std::atomic_signal_fence(std::memory_order_seq_cst);
        return t;
    }
    static uint32_t hz() { return SysClock::hz; }
};
static_assert(brio::CycleRuler<Ruler>);

using Idle = brio::BenchIdle<P, Ruler>;
static_assert(brio::Platform<Idle>);

namespace {

using namespace brio;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;   // rings 64/256 (defaults)
constexpr Serial serial;

constexpr uint32_t console_baud = 115200;
constexpr uint32_t frame_bits = 10;     // 8N1: start, eight data, stop

TestBench<Serial, 8> bench;

// One meter per bound vector (the file header).
IsrMeter<Ruler, Idle> uart_meter;
IsrMeter<Ruler, Idle> tick_meter;
IsrMeter<Ruler, Idle> dma_meter;
IsrMeter<Ruler, Idle> spi_meter;
IsrMeter<Ruler, Idle> loop_meter;   // letter u: UART1's vector

// A meter no vector carries: letter r's stamp pair is measured on it, so
// the two above count interrupts and nothing else.
IsrMeter<Ruler, Idle> stamp_probe;

/// What every number of a run is relative to: the core, the clock, the
/// ruler as it stands (SOURCE read back) and the console's rate. Printed by
/// the banner and again at the head of letter r, so that a capture of `z`
/// carries it.
void identity() {
    print(serial, "  on ", core_kind == CoreKind::hazard3 ? "Hazard3" : "Cortex-M33",
          ": clk_sys ", SysClock::hz, " Hz, ruler TIMER1 counting ",
          Ruler::Counter::source() == TimerSource::sysclk ? "clk_sys" : "THE TICK (SOURCE lost)",
          " at ", Ruler::hz(), " Hz, console UART0 ", console_baud, " 8N1", crlf);
}

/// The counters at one instant. Each is a word this core moves in one
/// access, but the tick runs on through every letter, so the set is read
/// under the mask: a handler that lands between two of the reads would
/// leave a snapshot whose count and cycles belong to different instants.
/// Outside every measured span.
BenchCounters counters() {
    P::CriticalSection masked;
    return bench_counters<Idle>(uart_meter, tick_meter, dma_meter, spi_meter, loop_meter);
}

/// Let the console fall silent before a measurement, so its interrupts are
/// not inside what is measured. Bounded at one second of ruler.
void console_drain() {
    Stopwatch<Ruler> sw;
    sw.start();
    while (!Serial::tx_idle() && sw.elapsed() < Ruler::hz()) {
    }
}

/// One idle turn the way the kernel's loop takes it: masked, then idle(),
/// which sleeps and unmasks.
void idle_turn() {
    disable_interrupts();
    Idle::idle();
}

/// A sample with every field divided by `turns`, rounded to the nearest.
BenchSample per_turn(const BenchSample& s, uint32_t turns) {
    if (turns == 0u) {
        return s;
    }
    const auto div = [turns](uint32_t v) { return (v + turns / 2u) / turns; };
    return {div(s.wall), div(s.busy), div(s.irq), div(s.isr)};
}

// =============================================================================
// r - the ruler's self-check and the instrument's cost
// =============================================================================
constexpr uint32_t delay_request_us = 999;   // delay_us_cap - 1 (the file header)
constexpr uint32_t instrument_reps = 1000;

void tr_ruler() {
    identity();

    console_drain();
    Stopwatch<Ruler> sw;
    sw.start();
    const bool served = delay_us(clock, delay_request_us);
    const uint32_t took = sw.elapsed();
    const uint32_t floor_cycles =
        static_cast<uint32_t>(static_cast<uint64_t>(Ruler::hz()) * delay_request_us / 1'000'000u);
    print(serial, "  delay_us(clock, ", delay_request_us, ") = ", took, " ruler cycles; ",
          delay_request_us, " us of clk_sys is ", floor_cycles, crlf);
    bench.verdict("delay_us(clock, 999) is served and lasts AT LEAST 999 us of the ruler's "
                  "clk_sys cycles - the platform timer on the crystal against TIMER1 on the PLL",
                  served && took >= floor_cycles);
    bench.verdict("and under 5 per cent more: the delay's documented price is one microsecond "
                  "late (rp2350/delay.hpp)",
                  took < floor_cycles + floor_cycles / 20u);

    // ruler: pairs of back-to-back reads, the mean difference of a pair.
    uint32_t pair_sum = 0;
    {
        P::CriticalSection masked;
        for (uint32_t i = 0; i < instrument_reps; ++i) {
            const uint32_t a = Ruler::now();
            const uint32_t b = Ruler::now();
            pair_sum += b - a;
        }
    }
    print(serial, "  ruler: ", instrument_reps, " pairs of reads, ", pair_sum,
          " cycles between the two reads in all", crlf);
    const uint32_t read_cycles = (pair_sum + instrument_reps / 2u) / instrument_reps;
    bench_line(serial, "ruler", 0, {read_cycles, read_cycles, 0, 0}, Ruler::hz(), 0);

    // stamp: pairs on a meter no vector carries - a loop with one pair a
    // turn and the same loop with two, whose difference is 1000 pairs and
    // no loop. The fence makes the compiler load and store the meter's
    // fields every pair, as a handler that runs once does.
    uint32_t one_a_turn = 0;
    uint32_t two_a_turn = 0;
    const uint32_t probe_before = stamp_probe.cycles();
    {
        P::CriticalSection masked;
        Stopwatch<Ruler> loop;
        loop.start();
        for (uint32_t i = 0; i < instrument_reps; ++i) {
            stamp_probe.enter();
            stamp_probe.leave();
            std::atomic_signal_fence(std::memory_order_seq_cst);
        }
        one_a_turn = loop.elapsed();
        loop.start();
        for (uint32_t i = 0; i < instrument_reps; ++i) {
            stamp_probe.enter();
            stamp_probe.leave();
            std::atomic_signal_fence(std::memory_order_seq_cst);
            stamp_probe.enter();
            stamp_probe.leave();
            std::atomic_signal_fence(std::memory_order_seq_cst);
        }
        two_a_turn = loop.elapsed();
    }
    constexpr uint32_t probe_pairs = 3u * instrument_reps;
    const uint32_t probe_reading = stamp_probe.cycles() - probe_before;
    const uint32_t pair_cycles =
        two_a_turn > one_a_turn
            ? (two_a_turn - one_a_turn + instrument_reps / 2u) / instrument_reps
            : 0u;
    print(serial, "  stamp: ", instrument_reps, " turns of one pair in ", one_a_turn,
          " cycles, of two pairs in ", two_a_turn, "; the meter's own reading of an empty body ",
          probe_reading, " over ", probe_pairs, " pairs = ",
          (probe_reading + probe_pairs / 2u) / probe_pairs, " a pair", crlf);
    bench_line(serial, "stamp", 0, {pair_cycles, pair_cycles, 0, 0}, Ruler::hz(), 0);

    // window: 100 ms of idle turns, per turn. It holds the tick handler.
    console_drain();
    const BenchCounters before = counters();
    const uint32_t turns0 = Idle::idle_turns();
    Stopwatch<Ruler> span;
    span.start();
    while (span.elapsed() < Ruler::hz() / 10u) {
        idle_turn();
    }
    const uint32_t wall = span.elapsed();
    const BenchCounters after = counters();
    const uint32_t turns = Idle::idle_turns() - turns0;
    const BenchSample total = bench_sample(wall, before, after);
    print(serial, "  window: 100 ms = ", turns, " turns, wall ", total.wall, " busy ", total.busy,
          " irq ", total.irq, " isr ", total.isr, " (the tick handler is inside every turn)",
          crlf);
    bench_line(serial, "window", 0, per_turn(total, turns), Ruler::hz(), 0);
    bench.verdict("ran", true);
}

// =============================================================================
// m - the runtime's memcpy and memset against the load/store floor
// =============================================================================
constexpr uint32_t sizes[] = {1, 16, 256, 4096};
constexpr uint32_t copy_bytes_per_cycle = 2;   // a word read and a word write per word
constexpr uint32_t fill_bytes_per_cycle = 4;   // a word write per word

alignas(4) uint8_t source_buffer[4096];
alignas(4) uint8_t destination_buffer[4096];

// Called through volatile copies, so that the runtime's own symbols run
// (the file header).
void* (*volatile copy_fn)(void*, const void*, std::size_t) = &::memcpy;
void* (*volatile fill_fn)(void*, int, std::size_t) = &::memset;

template <typename Op>
BenchSample best_of_8(Op op) {
    BenchSample best{};
    for (uint8_t run = 0; run < 8u; ++run) {
        const BenchCounters before = counters();
        Stopwatch<Ruler> sw;
        sw.start();
        op();
        const uint32_t wall = sw.elapsed();
        const BenchCounters after = counters();
        const BenchSample s = bench_sample(wall, before, after);
        if (run == 0u || s.wall < best.wall) {
            best = s;
        }
    }
    return best;
}

void tm_memory() {
    for (uint32_t i = 0; i < sizeof(source_buffer); ++i) {
        source_buffer[i] = static_cast<uint8_t>(i * 7u + 1u);
    }
    console_drain();
    for (const uint32_t n : sizes) {
        const BenchSample s = best_of_8([n] { (void)copy_fn(destination_buffer, source_buffer, n); });
        bench_line(serial, "memcpy", n, s, Ruler::hz(), copy_bytes_per_cycle * Ruler::hz());
        console_drain();
    }
    for (const uint32_t n : sizes) {
        const BenchSample s = best_of_8([n] { (void)fill_fn(destination_buffer, 0x5A, n); });
        bench_line(serial, "memset", n, s, Ruler::hz(), fill_bytes_per_cycle * Ruler::hz());
        console_drain();
    }
    bench.verdict("ran", true);
}

// =============================================================================
// p - the console's per-byte path at its rate
// =============================================================================
constexpr std::array<char, 4096> make_payload() {
    constexpr char glyphs[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
    std::array<char, 4096> text{};
    for (uint32_t i = 0; i < text.size(); ++i) {
        const uint32_t column = i % 64u;
        text[i] = column == 62u ? '\r' : column == 63u ? '\n' : glyphs[column];
    }
    return text;
}
constexpr std::array<char, 4096> payload = make_payload();

/// The counters with the console's own meter beside them, read at one
/// instant: a print lasts long enough for the tick to land in it, and the
/// line's irq and isr sum both vectors.
struct PrintCounters {
    BenchCounters all;
    uint32_t uart_irq;
    uint32_t uart_isr;
};
PrintCounters print_counters() {
    P::CriticalSection masked;
    return {bench_counters<Idle>(uart_meter, tick_meter, dma_meter, spi_meter), uart_meter.count(),
            uart_meter.cycles()};
}

void tp_print() {
    for (const uint32_t n : sizes) {
        console_drain();
        const PrintCounters before = print_counters();
        Stopwatch<Ruler> sw;
        sw.start();
        print(serial, std::string_view{payload.data(), n});
        while (!Serial::tx_idle()) {
        }
        const uint32_t wall = sw.elapsed();
        const PrintCounters after = print_counters();
        const BenchSample s = bench_sample(wall, before.all, after.all);
        const uint32_t uart_irq = after.uart_irq - before.uart_irq;
        const uint32_t uart_isr = after.uart_isr - before.uart_isr;
        print(serial, crlf);
        bench_line(serial, "print", n, s, Ruler::hz(), console_baud / frame_bits);
        print(serial, "  of which the console's vector irq ", uart_irq, " isr ", uart_isr,
              ", the tick's irq ", s.irq - uart_irq, " isr ", s.isr - uart_isr, crlf);
    }
    bench.verdict("ran", true);
}

// =============================================================================
// t - the tick's floor
// =============================================================================
void tt_tick() {
    console_drain();
    const BenchCounters before = counters();
    Stopwatch<Ruler> sw;
    sw.start();
    while (sw.elapsed() < Ruler::hz()) {
        idle_turn();
    }
    const uint32_t wall = sw.elapsed();
    const BenchCounters after = counters();
    bench_line(serial, "tick", 0, bench_sample(wall, before, after), Ruler::hz(), 0);
    bench.verdict("ran", true);
}

// =============================================================================
// d - the DMA: copy and fill, a paced block, the SPI engines
// =============================================================================
using Copy = DmaCopyEngine<8>;
using Paced = DmaTxEngine<9, uint32_t>;
using Pace = DmaTimer<0>;
constexpr uint32_t dma_bytes_per_cycle = 4;   // one 32-bit read and write a clock (12.6)
constexpr uint32_t paced_words = 256;
constexpr uint16_t pace_cycles = 150;         // Y / X: one request a microsecond at 150 MHz

constexpr SpiPins spi_pins{.sck = 18, .tx = 19, .rx = 16};
using Spi = SpiHost<0, spi_pins, DmaTxEngine<4, uint16_t>, DmaRxEngine<5, uint16_t>>;
using SpiSelect = Pin<17>;

alignas(4) uint32_t paced_table[paced_words];
volatile uint32_t paced_cell = 0;
uint32_t pace_stamps[paced_words];
volatile bool paced_done = false;
volatile bool spi_done = false;
volatile bool spi_live = false;
volatile uint32_t spi_irq_at = 0;   // the ruler at the DMA line's entry, a transaction's completion

/// A copy or a fill and its wait on BUSY, timed: the best of 8.
template <typename Op>
BenchSample best_dma_of_8(Op op) {
    return best_of_8([&op] {
        (void)op();
        while (Copy::busy()) {
        }
    });
}

void dma_memory() {
    Copy::arm(DmaReport::errors);   // polled: the channel on no line
    auto* const to = reinterpret_cast<uint32_t*>(destination_buffer);
    const auto* const from = reinterpret_cast<const uint32_t*>(source_buffer);
    static uint32_t cell = 0x5A5A5A5Au;   // in SRAM: a const one would be read through the XIP cache
    for (uint32_t i = 0; i < sizeof(source_buffer); ++i) {
        source_buffer[i] = static_cast<uint8_t>(i * 7u + 1u);
    }
    console_drain();
    for (const uint32_t n : {16u, 256u, 4096u}) {
        const BenchSample s = best_dma_of_8([=] { return Copy::copy(to, from, n / 4u); });
        const bool exact = std::memcmp(destination_buffer, source_buffer, n) == 0;
        bench_line(serial, "copy", n, s, Ruler::hz(), dma_bytes_per_cycle * Ruler::hz());
        print(serial, "  copy ", n, ": fixed cost ", s.wall - n / 4u, " cycles over one a word, ",
              exact ? "exact" : "WRONG", crlf);
        console_drain();
    }
    for (const uint32_t n : {16u, 256u, 4096u}) {
        const BenchSample s = best_dma_of_8([=] { return Copy::fill(to, &cell, n / 4u); });
        bool exact = true;
        for (uint32_t i = 0; i < n / 4u; ++i) {
            exact = exact && to[i] == cell;
        }
        bench_line(serial, "fill", n, s, Ruler::hz(), dma_bytes_per_cycle * Ruler::hz());
        print(serial, "  fill ", n, ": fixed cost ", s.wall - n / 4u, " cycles over one a word, ",
              exact ? "exact" : "WRONG", crlf);
        console_drain();
    }
    Copy::stop();
}

void dma_paced() {
    for (uint32_t i = 0; i < paced_words; ++i) {
        paced_table[i] = i * 0x01010101u;
    }
    Pace::set(1, pace_cycles);
    Paced::arm(&paced_cell, Pace::dreq());   // a completion on line 0
    // Twice, the first thrown away: it runs the handler and the idle path
    // cold out of the flash.
    BenchSample paced{};
    uint32_t banked = 0;
    for (uint8_t run = 0; run < 2u; ++run) {
        console_drain();
        paced_done = false;
        banked = DmaChannel<Paced::channel>::debug_credits();
        const BenchCounters before = counters();
        Stopwatch<Ruler> sw;
        sw.start();
        (void)Paced::start(paced_table, paced_words);
        while (!paced_done && sw.elapsed() < Ruler::hz() / 10u) {
            idle_turn();
        }
        const uint32_t wall = sw.elapsed();
        const BenchCounters after = counters();
        paced = bench_sample(wall, before, after);
    }
    bench_line(serial, "paced", paced_words * 4u, paced, Ruler::hz(),
               4u * (Ruler::hz() / pace_cycles));
    print(serial, "  paced: the cell holds ", hex(paced_cell), " (the table's last word ",
          hex(paced_table[paced_words - 1u]), "); the channel held ", banked,
          " request credits from the timer when the block started", crlf);

    // The jitter: the same block, the core stamping the ruler each time
    // the channel's count steps.
    console_drain();
    Paced::arm(&paced_cell, Pace::dreq(), false, DmaReport::errors);
    uint32_t turns = 0;
    uint32_t start_at = 0;
    uint32_t banked_now = 0;
    {
        P::CriticalSection masked;
        uint32_t seen = paced_words;
        uint32_t k = 0;
        banked_now = DmaChannel<Paced::channel>::debug_credits();
        const uint32_t t0 = Ruler::now();
        start_at = t0;
        (void)Paced::start(paced_table, paced_words);
        while (k < paced_words && Ruler::now() - t0 < Ruler::hz() / 10u) {
            const uint32_t left = DmaChannel<Paced::channel>::count();
            const uint32_t now = Ruler::now();
            ++turns;
            while (seen > left && k < paced_words) {
                pace_stamps[k++] = now;
                --seen;
            }
        }
        (void)Paced::complete();
    }
    const uint32_t span = pace_stamps[paced_words - 1u] - pace_stamps[0];
    const uint32_t turn = span / (turns != 0u ? turns : 1u);   // the instrument's resolution
    uint32_t lo = 0xFFFFFFFFu;
    uint32_t hi = 0;
    uint32_t inside = 0;
    for (uint32_t i = 1; i < paced_words; ++i) {
        const uint32_t d = pace_stamps[i] - pace_stamps[i - 1u];
        lo = d < lo ? d : lo;
        hi = d > hi ? d : hi;
        if (d + turn >= pace_cycles && d <= pace_cycles + turn) {
            ++inside;
        }
    }
    print(serial, "  paced, the pace against the ruler: ", span, " cycles over 255 steps = ",
          (span + 127u) / 255u, " a step (", pace_cycles, " asked); ", inside,
          " of the 255 intervals within one polling turn (", turn, " cycles) of it, the "
          "extremes ", lo, " and ", hi, "; ", banked_now, " credits banked at the start, the "
          "first step seen ", pace_stamps[0] - start_at, " cycles after it and the last ",
          pace_stamps[paced_words - 1u] - start_at, crlf);
    Pace::stop();
    Paced::stop();
}

/// A transaction's numbers, and where its wall went: start() returning,
/// and the completion's handler entered, both from the first stamp.
struct SpiRun {
    BenchSample sample;
    uint32_t launch;
    uint32_t to_irq;
};

/// How the core waits for a transaction's completion: asleep (the idle
/// turn, the default), spinning on the RAM flag the handler sets, or
/// spinning on a read of the ruler - a register behind the APB bridge, as
/// the SPI's own data register is.
enum class SpiWait : uint8_t { sleep, spin_ram, spin_apb };

/// One engined transaction of `frames` frames, timed start() to the
/// completion on line 0 with the core idling between.
SpiRun spi_once(uint16_t frames, SpiClock rate, SpiDataSize bits, SpiWait wait = SpiWait::sleep) {
    Spi::Request r{};
    r.cs = SpiSelect::ref();
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
    r.cmd_len = 0;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(source_buffer));
    r.rx = lend<Lease::reply>(destination_buffer);
    r.len = frames;
    r.clock = rate;
    r.mode = SpiMode::mode0;
    r.bits = bits;
    console_drain();
    spi_done = false;
    spi_irq_at = 0;
    const BenchCounters before = counters();
    const uint32_t t0 = Ruler::now();
    if (Spi::start(r)) {
        spi_done = true;
    }
    const uint32_t launched = Ruler::now();
    if (wait == SpiWait::spin_ram) {
        for (uint32_t spins = 1'000'000u; !spi_done && spins != 0u; --spins) {
        }
    } else if (wait == SpiWait::spin_apb) {
        while (!spi_done && Ruler::now() - t0 < Ruler::hz() / 10u) {
        }
    }
    while (!spi_done && Ruler::now() - t0 < Ruler::hz() / 10u) {
        idle_turn();
    }
    const uint32_t wall = Ruler::now() - t0;
    const BenchCounters after = counters();
    return {bench_sample(wall, before, after), launched - t0, spi_irq_at - t0};
}

/// The best of five by wall, after one run thrown away: the first of a
/// configuration runs its code cold out of the flash and applies the new
/// rate or width to the block, which are the cache's cost and the
/// reconfiguration's, not the engines'.
uint32_t spi_cold_wall = 0;   // the very first transaction's wall: the code cold out of the flash

SpiRun spi_transaction(uint16_t frames, SpiClock rate, SpiDataSize bits) {
    const SpiRun thrown = spi_once(frames, rate, bits);
    if (spi_cold_wall == 0u) {
        spi_cold_wall = thrown.sample.wall;
    }
    SpiRun best{};
    for (uint8_t run = 0; run < 5u; ++run) {
        const SpiRun s = spi_once(frames, rate, bits);
        if (run == 0u || s.sample.wall < best.sample.wall) {
            best = s;
        }
    }
    return best;
}

void dma_spi() {
    (void)SpiSelect::output(true);
    (void)Spi::init(clock);
    spi_live = true;
    spi_cold_wall = 0;
    struct Rate { SpiClock clock; const char* name; };
    constexpr Rate rates[] = {{SpiClocks::div2, "75 MHz"}, {SpiClocks::div8, "18.75 MHz"}};
    for (const Rate& rate : rates) {
        const uint32_t sck = Spi::sck_hz(rate.clock);
        for (const SpiDataSize bits : {SpiDataSize::bits8, SpiDataSize::bits16}) {
            const uint32_t frame_bytes = bits == SpiDataSize::bits16 ? 2u : 1u;
            uint32_t walls[2] = {0, 0};
            uint8_t k = 0;
            for (const uint16_t frames : {static_cast<uint16_t>(16), static_cast<uint16_t>(256)}) {
                const SpiRun run = spi_transaction(frames, rate.clock, bits);
                const BenchSample& s = run.sample;
                const uint32_t n = frames * frame_bytes;
                const uint32_t wire_cycles =
                    static_cast<uint32_t>(static_cast<uint64_t>(n) * 8u * Ruler::hz() / sck);
                walls[k++] = s.wall;
                bench_line(serial, "spi.dma", n, s, Ruler::hz(), sck / 8u);
                const uint32_t fixed = s.wall > wire_cycles ? s.wall - wire_cycles : 0u;
                print(serial, "  spi.dma ", frames, " frames of ", spi_data_bits(bits), " bits at ",
                      rate.name, ": the wire ", wire_cycles, " cycles, the transaction's fixed "
                      "cost ", fixed, " cycles = ", fixed * 1000u / (Ruler::hz() / 1'000'000u),
                      " ns; start() returned at ", run.launch, ", the completion's handler "
                      "entered at ", run.to_irq, crlf);
            }
            const uint32_t per_byte_x100 =
                walls[1] > walls[0] ? (walls[1] - walls[0]) * 100u / (240u * frame_bytes) : 0u;
            print(serial, "  spi.dma at ", rate.name, ", ", spi_data_bits(bits),
                  "-bit frames: ", per_byte_x100 / 100u, '.', per_byte_x100 % 100u / 10u,
                  per_byte_x100 % 10u, " cycles a byte by the difference of the two sizes, the "
                  "wire's ", 8u * Ruler::hz() / sck, crlf);
            if (k == 2u && rate.clock == SpiClocks::div2 && bits == SpiDataSize::bits8) {
                print(serial, "  spi.dma, the very first transaction (16 frames, its code cold "
                      "out of the flash and the rate applied to the block): ", spi_cold_wall,
                      " cycles", crlf);
            }
        }
    }
    // THE SAME TRANSACTIONS ON THE LOOP-BACK (LBM), plain lines and not
    // bench lines: the cycles a byte by the difference of two sizes, the
    // instrument test_rp2350_spi's letter d uses - for the reader who
    // compares the two.
    Spi::loopback(true);
    for (const Rate& rate : rates) {
        const uint32_t sck = Spi::sck_hz(rate.clock);
        for (const SpiDataSize bits : {SpiDataSize::bits8, SpiDataSize::bits16}) {
            const uint32_t frame_bytes = bits == SpiDataSize::bits16 ? 2u : 1u;
            const SpiRun small = spi_transaction(16, rate.clock, bits);
            const SpiRun large = spi_transaction(256, rate.clock, bits);
            const uint32_t per_byte_x100 =
                large.sample.wall > small.sample.wall
                    ? (large.sample.wall - small.sample.wall) * 100u / (240u * frame_bytes)
                    : 0u;
            print(serial, "  spi.dma ON THE LOOP-BACK at ", rate.name, ", ", spi_data_bits(bits),
                  "-bit frames: 16 frames in ", small.sample.wall, " cycles, 256 in ",
                  large.sample.wall, ", irq ", large.sample.irq, "; ", per_byte_x100 / 100u, '.',
                  per_byte_x100 % 100u / 10u, per_byte_x100 % 10u,
                  " cycles a byte by the difference, the wire's ", 8u * Ruler::hz() / sck, crlf);
        }
    }
    Spi::loopback(false);

    // AND THE SAME 256 BYTES WITH THE CORE AWAKE: spinning on the flag in
    // RAM, then spinning on the ruler, whose every read crosses the APB
    // bridge the DMA's accesses to SSPDR cross too.
    for (const Rate& rate : rates) {
        const uint32_t sck = Spi::sck_hz(rate.clock);
        uint32_t walls[3] = {0, 0, 0};
        const SpiWait waits[3] = {SpiWait::sleep, SpiWait::spin_ram, SpiWait::spin_apb};
        for (uint8_t w = 0; w < 3u; ++w) {
            (void)spi_once(256, rate.clock, SpiDataSize::bits8, waits[w]);
            uint32_t best = 0;
            for (uint8_t run = 0; run < 5u; ++run) {
                const uint32_t wall = spi_once(256, rate.clock, SpiDataSize::bits8, waits[w]).sample.wall;
                best = run == 0u || wall < best ? wall : best;
            }
            walls[w] = best;
        }
        print(serial, "  spi.dma 256 bytes at ", rate.name, " (the wire ",
              256u * 8u * (Ruler::hz() / sck), " cycles): the core asleep ", walls[0],
              ", spinning on a RAM flag ", walls[1], ", spinning on the APB ruler ", walls[2],
              " cycles", crlf);
    }
    spi_live = false;
    Spi::release();
}

void td_dma() {
    dma_memory();
    dma_paced();
    dma_spi();
    bench.verdict("ran", true);
}

// =============================================================================
// e - the SPI host above the wire: the polled loop, the pump, a request's price
// =============================================================================
using Plain = SpiHost<0, spi_pins>;   // no engines: the polled loop and the pump
using SpiDc = Pin<22>;
volatile bool plain_live = false;
volatile bool plain_done = false;
uint8_t dcs_command = 0x2C;   // in SRAM, as a client's command byte would be

struct SpiRate { SpiClock clock; const char* name; };
constexpr SpiRate e_rates[] = {{SpiClocks::div2, "75 MHz"}, {SpiClocks::div4, "37.5 MHz"}, {SpiClocks::div16, "9.375 MHz"}};

/// One request's numbers: the bench sample, the SPI vector's own entries and
/// cycles inside it, and the block's overrun flag read after the run.
struct SpiReqRun {
    BenchSample sample;
    uint32_t spi_irq;
    uint32_t spi_isr;
    bool overrun;
};

/// One request on host H: a command phase of `cmd_len` frames when asked
/// (D/C scripted), a data phase of `frames`, polled or pumped. Wall from the
/// first stamp to the completion, the core idling through a pumped one.
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
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(source_buffer));
    r.rx = lend<Lease::reply>(with_rx ? destination_buffer : nullptr);
    r.len = frames;
    r.clock = rate;
    r.mode = mode;
    r.bits = bits;
    r.polled = polled;
    console_drain();
    H::Resource::clear_pending(SpiInterrupt::overrun);
    plain_done = false;
    spi_done = false;
    const uint32_t irq0 = spi_meter.count();
    const uint32_t isr0 = spi_meter.cycles();
    const BenchCounters before = counters();
    const uint32_t t0 = Ruler::now();
    bool done = H::start(r);
    // The wait is the kernel loop's shape: the flag read UNDER THE MASK and
    // the sleep entered from there, so a completion that lands between the
    // read and the sleep cannot be lost - read outside it, a handler that
    // fires in that window leaves the core asleep until the tick, which the
    // first version of this letter measured as a stall of exactly one tick.
    while (!done) {
        disable_interrupts();
        done = plain_done || spi_done;
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
        // A transaction that never completed: the block's state, for the reader.
        print(serial, "  STALLED: SSPSR ", hex(H::Resource::flags()), " SSPRIS ",
              hex(H::Resource::raw_pending()), " SSPIMSC ", hex(H::Resource::interrupts()),
              " spi irq ", spi_meter.count() - irq0, crlf);
        (void)H::recover();
    }
    return {bench_sample(wall, before, after), spi_meter.count() - irq0,
            spi_meter.cycles() - isr0, overrun};
}

/// The best of five by wall after one thrown away, the overrun flag OR-ed
/// over every run.
template <typename H>
SpiReqRun e_best(uint8_t cmd_len, uint16_t frames, SpiClock rate, SpiMode mode, SpiDataSize bits,
                 bool polled, bool with_rx = true) {
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

/// The wire's cycles for `bytes` at `sck`, and the same with mode 0's gap of
/// 1.5 SCK periods a frame (letter d's measurement, 12.3.4.10).
uint32_t e_wire_cycles(uint32_t bytes, uint32_t sck) {
    return static_cast<uint32_t>(static_cast<uint64_t>(bytes) * 8u * Ruler::hz() / sck);
}
uint32_t e_gap_cycles(uint32_t frames, uint32_t sck) {
    return static_cast<uint32_t>(static_cast<uint64_t>(frames) * 3u * Ruler::hz() / (2u * sck));
}

void e_print_per_frame(const char* op, const SpiRate& rate, SpiDataSize bits, SpiMode mode,
                       uint32_t sck, const uint32_t walls[2]) {
    const uint32_t frame_bytes = bits == SpiDataSize::bits16 ? 2u : 1u;
    const uint32_t per_frame_x100 = walls[1] > walls[0] ? (walls[1] - walls[0]) * 100u / 240u : 0u;
    const uint32_t wire_frame = 8u * frame_bytes * Ruler::hz() / sck;
    print(serial, "  ", op, " at ", rate.name, ", ", spi_data_bits(bits), "-bit frames, mode ",
          static_cast<uint8_t>(mode), ": ", per_frame_x100 / 100u, '.', per_frame_x100 % 100u / 10u,
          per_frame_x100 % 10u, " cycles a frame by the difference of the two sizes, the wire's ",
          wire_frame, mode == SpiMode::mode0 ? " and with the block's gap " : " (no gap in mode 3)",
          mode == SpiMode::mode0 ? wire_frame + 3u * Ruler::hz() / (2u * sck) : 0u, crlf);
}

/// The op's name: the completion style, the width, and for a polled
/// request its SHAPE - the write with no in buffer, or the receive.
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

/// THE RECEIVE SHAPE UNDER A LONG HANDLER: the frames in flight are never
/// more than the receive FIFO holds, so no handler that holds the loop
/// off can overrun it - the claim, and this is its measurement. The tick
/// handler is stretched to `tick_stretch` cycles (past eight frame times
/// at both rates), 100 receive requests of 256 frames run on the
/// loop-back, each judged exact, the overrun flag read after each.
volatile uint32_t tick_stretch = 0;
constexpr uint32_t rx_proof_runs = 100;

void e_rx_proof() {
    Plain::loopback(true);
    for (const SpiRate& rate : e_rates) {
        uint32_t exact = 0;
        uint32_t overruns = 0;
        console_drain();
        // The meter is written by the handler and read here: a fence on
        // either side of the loop, so the compiler reloads what it cannot
        // see change.
        std::atomic_signal_fence(std::memory_order_seq_cst);
        const uint32_t ticks0 = tick_meter.count();
        tick_stretch = 1500;
        for (uint32_t run = 0; run < rx_proof_runs; ++run) {
            for (uint32_t i = 0; i < 256u; ++i) {
                source_buffer[i] = static_cast<uint8_t>(i * 7u + run);
                destination_buffer[i] = 0xEE;
            }
            Plain::Resource::clear_pending(SpiInterrupt::overrun);
            Plain::Request r{};
            r.cs = SpiSelect::ref();
            r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(source_buffer));
            r.rx = lend<Lease::reply>(destination_buffer);
            r.len = 256;
            r.clock = rate.clock;
            r.mode = SpiMode::mode3;
            r.polled = true;
            (void)Plain::start(r);
            if (std::memcmp(source_buffer, destination_buffer, 256) == 0) {
                ++exact;
            }
            if ((Plain::Resource::raw_pending() & SpiInterrupt::overrun) != 0u) {
                ++overruns;
            }
        }
        tick_stretch = 0;
        std::atomic_signal_fence(std::memory_order_seq_cst);
        const uint32_t ticks = tick_meter.count() - ticks0;
        print(serial, "  spi.poll.rx under a tick handler of ", 1500, " cycles (eight frames at ",
              rate.name, " are ", 8u * (8u * Ruler::hz() / Plain::sck_hz(rate.clock)), "): ",
              rx_proof_runs, " requests of 256 frames on the loop-back, ", exact, " exact, ",
              overruns, " overruns, ", ticks, " ticks landed", crlf);
    }
    Plain::loopback(false);
}

/// THE COLD REQUEST (spi.cold): the price of a DCS command when the XIP
/// cache has not got the host's code. spi.req's three-byte request - a
/// command byte with D/C low and two data bytes, polled, nothing read
/// back, at 9.375 MHz - run four times from here, the first of them the
/// host's VERY FIRST transaction after init(): (1) cold as a program meets
/// it, (2) its repeat, warm; (3) with every line of the cache invalidated
/// first (Xip::invalidate_all(), the maintenance by set and way of
/// 4.4.1.1: the repeatable reading, nothing of the image warm, which is
/// also the worst a program meets after a phase that walked the flash)
/// and (4) its repeat. Each timed as e_once times a request, with the
/// cache's misses beside it - Xip::accesses() less Xip::hits(), the
/// counters cleared right before the start and read right after the wall
/// - so a warm repeat's misses are the lines the request's own code and
/// the glue around it still fetch from the flash. The bench line is (3).
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
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(source_buffer));
    r.len = 2;
    r.clock = e_rates[2].clock;
    r.mode = SpiMode::mode0;
    r.bits = SpiDataSize::bits8;
    r.polled = true;
    console_drain();
    Plain::Resource::clear_pending(SpiInterrupt::overrun);
    const BenchCounters before = counters();
    if (invalidate) {
        Xip::invalidate_all();
    }
    // UNDER THE MASK: a polled request needs no interrupt, and a tick
    // landing inside this one run - one shot, no best of five - would
    // add its handler's wall and, after the invalidate, its handler's
    // own cold lines, which are not the host's.
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
    const uint32_t wire = e_wire_cycles(3, sck);
    const uint32_t gap = e_gap_cycles(3, sck);
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
    for (uint32_t i = 0; i < sizeof(source_buffer); ++i) {
        source_buffer[i] = static_cast<uint8_t>(i * 7u + 1u);
    }
    e_cold();   // FIRST: its first request must be the host's first

    // spi.poll (the write), spi.poll.rx (the receive) and spi.pump:
    // rates x widths x modes x sizes.
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
                        const uint32_t n = frames * frame_bytes;
                        walls[k++] = run.sample.wall;
                        bench_line(serial, op, n, run.sample, Ruler::hz(), sck / 8u);
                        if (!shape.polled) {
                            print(serial, "  ", op, " ", frames, " frames of ", spi_data_bits(bits),
                                  " bits at ", rate.name, ", mode ", static_cast<uint8_t>(mode),
                                  ": the SPI vector irq ", run.spi_irq, " isr ", run.spi_isr, " = ",
                                  run.spi_irq == 0u ? 0u : (run.spi_isr + run.spi_irq / 2u) / run.spi_irq,
                                  " a handler, ", (run.spi_isr + frames / 2u) / frames,
                                  " a frame; the tick's irq ", run.sample.irq - run.spi_irq,
                                  "; overrun flag ", run.overrun ? "RAISED" : "clear", crlf);
                        } else {
                            print(serial, "  ", op, " ", frames, " frames of ", spi_data_bits(bits),
                                  " bits at ", rate.name, ", mode ", static_cast<uint8_t>(mode),
                                  ": overrun flag ", run.overrun ? "RAISED" : "clear", crlf);
                        }
                    }
                    e_print_per_frame(op, rate, bits, mode, sck, walls);
                }
            }
        }
    }
    e_rx_proof();

    // spi.req: the price of a DCS command on the plain host - a command
    // byte with D/C low, then data bytes with it high, nothing read back
    // (the display's shape).
    const SpiRate& req_rate = e_rates[2];
    const uint32_t req_sck = Plain::sck_hz(req_rate.clock);
    for (const uint16_t len : {static_cast<uint16_t>(0), static_cast<uint16_t>(2), static_cast<uint16_t>(15)}) {
        const SpiReqRun run =
            e_best<Plain>(1, len, req_rate.clock, SpiMode::mode0, SpiDataSize::bits8, true, false);
        const uint32_t n = 1u + len;
        const uint32_t wire = e_wire_cycles(n, req_sck);
        const uint32_t gap = e_gap_cycles(n, req_sck);
        bench_line(serial, "spi.req", n, run.sample, Ruler::hz(), req_sck / 8u);
        print(serial, "  spi.req: a command byte and ", len, " data bytes at ", req_rate.name,
              ": the wire ", wire, " cycles (", wire + gap, " with the gap), the fixed cost ",
              run.sample.wall > wire ? run.sample.wall - wire : 0u, " cycles = ",
              (run.sample.wall > wire ? run.sample.wall - wire : 0u) * 1000u / (Ruler::hz() / 1'000'000u),
              " ns (", run.sample.wall > wire + gap ? run.sample.wall - wire - gap : 0u,
              " over the wire with the gap); overrun flag ", run.overrun ? "RAISED" : "clear", crlf);
    }
    plain_live = false;
    Plain::release();

    // The same three requests on the ENGINED host of letter d.
    (void)Spi::init(clock);
    spi_live = true;
    for (const uint16_t len : {static_cast<uint16_t>(0), static_cast<uint16_t>(2), static_cast<uint16_t>(15)}) {
        const SpiReqRun run =
            e_best<Spi>(1, len, req_rate.clock, SpiMode::mode0, SpiDataSize::bits8, true, false);
        const uint32_t n = 1u + len;
        const uint32_t wire = e_wire_cycles(n, req_sck);
        print(serial, "  spi.req on the engined host: a command byte and ", len, " data bytes at ",
              req_rate.name, ": wall ", run.sample.wall, ", busy ", run.sample.busy, ", irq ",
              run.sample.irq, ", the fixed cost ", run.sample.wall > wire ? run.sample.wall - wire : 0u,
              " cycles", crlf);
    }
    spi_live = false;
    Spi::release();
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
constexpr std::array<uint32_t, 3> loop_rates{115200u, 1'000'000u, static_cast<uint32_t>(SysClock::hz / 16u)};

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
    console_drain();
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
        const uint32_t k = T::write_bulk(payload_bytes(4096u - n + sent, n - sent));
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
    console_drain();
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
    console_drain();
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
    console_drain();
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
    console_drain();
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

void banner() {
    print(serial, crlf, "bench_rp2350", crlf);
    identity();
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
//
// ONE NAME, BOTH ARCHITECTURES (the file header), each vector with its
// meter's stamps around the driver's ISR body.
extern "C" void isr_uart0() {
    uart_meter.enter();
    (void)Serial::isr();
    uart_meter.leave();
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
            brio::Irq::disable(Pl011<1>::irq());
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
    spi_irq_at = Ruler::now();
    if (spi_live && Spi::dma_isr()) {
        spi_done = true;
    }
    if ((Paced::service() & Paced::flag_complete) != 0u) {
        (void)Paced::complete();
        paced_done = true;
    }
    dma_meter.leave();
}
extern "C" void isr_spi0() {
    spi_meter.enter();
    if (spi_live && Spi::isr()) {
        spi_done = true;
    }
    if (plain_live && Plain::isr()) {
        plain_done = true;
    }
    spi_meter.leave();
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool mtime_ok = brio::Mtime::start(clock);   // delay_us's ruler, on both halves
    const bool ruler_ok = Ruler::Counter::init(clock);
    Ruler::Counter::source(brio::TimerSource::sysclk);
    const bool serial_ok = Serial::init(clock, console_baud);
    const bool tick_ok = brio::Ticker::init(clock);
    const bool dma_ok = brio::Dma::init();
    brio::enable_interrupts();

    bench.letter('r', "the ruler's self-check and the instrument's cost", tr_ruler);
    bench.letter('m', "memcpy and memset against the load/store floor", tm_memory);
    bench.letter('p', "a print through the console at its rate", tp_print);
    bench.letter('t', "the tick's floor: one second of idle", tt_tick);
    bench.letter('d', "the DMA: copy, fill, a paced block, the SPI engines", td_dma);
    bench.letter('e', "the SPI host above the wire: the polled loop, the pump, a request's price", te_host);
    bench.letter('u', "the serial transport on UART1's own loop: tx, rx, the edge", tu_uart);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL150" : "FAILED",
                    " mtime=", mtime_ok ? "1us" : "FAILED", " ruler=",
                    ruler_ok ? "clk_sys" : "FAILED", " tick=", tick_ok ? "1kHz" : "FAILED",
                    " dma=", dma_ok ? "released" : "FAILED", brio::crlf);
        banner();
        bench.prompt();
    }

    for (;;) {
        brio::disable_interrupts();
        if (!Serial::rx_pending()) {
            Idle::idle();
            continue;
        }
        brio::enable_interrupts();
        uint8_t c = 0;
        if (!Serial::read_byte(c) || c == '\r' || c == '\n') {
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
