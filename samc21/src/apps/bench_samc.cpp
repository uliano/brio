// bench_samc - the benchmark skeleton on the SAM C21 (docs/design/
// benchmark.md, util/bench.hpp): five letters that print NUMBERS, one
// `bench` line per operation and size, in the grammar every family
// prints. NOT A TEST: a letter's one verdict is "ran", except letter r,
// whose verdicts judge the ruler every other line is read with.
//
// NOTHING TO WIRE BUT LETTER I's BUS (PA22/PA23 to the peer board's
// node, its file header below). The console is the board's CH340 bridge on PB30 (TX)
// / PB31 (RX) = SERCOM5 PAD[0]/PAD[1] under function D, 115200 8N1 - the
// binding of console.cpp and test_samc_platform.cpp, with the latter's
// TestBench frame and polled prompt loop. Letter d drives PA16 and PA17
// (SERCOM1's MOSI and SCK) and leaves PA19 (MISO) floating, and letter u
// makes PA16 a UART's loop (its TxD and its RxD): nothing is wired there
// and nothing should be. NO KERNEL AND NO AO: the letters are
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
// BENCH_PLACEMENT (and letter i's SERCOM3_Handler). It is empty in this file, so the handlers -
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
//   u  THE UART ON A LOOP (samc21/sercom.hpp, docs/samc21/sercom.md):
//      SERCOM1 with TxD and RxD on one pad, PAD[0] = PA16 under function
//      C - 31.6.3.8's loop-back through the pad, nothing wired - at
//      115200, 1 Mbaud and 3 Mbaud (f_ref/16, the generator's top):
//        uart.tx     256 and 4096 bytes through the plain transport (the
//                    thread spins: busy = wall; irq and isr SERCOM1's)
//                    and through the transmit engine (uart.tx.dma: the
//                    thread idles between completions, busy is the CPU's
//                    share; irq and isr the DMAC's); the loop's receive
//                    interrupt disarmed;
//        uart.rx     bursts of 16 and 256 into the interrupt receiver,
//                    sent by the transmit engine (irq and isr SERCOM1's:
//                    the receiver's alone), and into the receive engine
//                    (uart.rx.dma), sent by the plain transmitter (irq and
//                    isr the DMAC's), the transport brought up anew so a
//                    256 burst is one block, the owner asking once a tick
//                    for a tail; every burst checked byte for byte;
//        uart.edge   the cycles (in `wall`) from the sender's TXC - the
//                    thread masks once the transmitter holds the last
//                    byte and stamps TXC's rise - to the receive ring
//                    holding the burst, stamped in whichever context
//                    published it: the interrupt receiver (a burst of
//                    four, 115200) and the engine (16 - a tail, the ask -
//                    and 256 - a block, the vector - at 115200 and 1 Mbaud);
//        uart.copy   write_bulk() of 1 to 1024 bytes on the engined
//                    transport, masked, the source at the four offsets from
//                    a word: the copy into the ring and its crossover.
//   i  THE I2C HOST (samc21/i2c.hpp, docs/samc21/i2c.md): SERCOM3 on PA22
//      (SDA) / PA23 (SCL), function C - test_samc_i2c's pads - on the
//      desk's 5 V node (1.5k pull-ups) against a SECOND BOARD running
//      `twi_peer`, commanded in band over the same wires (twi_link.hpp,
//      included by its relative path): per speed it is told to `serve` at
//      0x2C for 15 s - a client that takes any write and answers a read
//      with one counting stream - and the letter waits the deadline out
//      before the next command. THE PEER IS A POLLED CLIENT: it stretches
//      every byte by its own turnaround, so wall minus wire holds the
//      peer's share beside the host's (the engined lines, whose bytes the
//      DMA moves within a few cycles of the request, read the peer's share
//      almost alone). Two hosts on the one SERCOM, brought up in turn:
//      the engineless host (the byte pump, `i2c.*`) and the engined one
//      with DmaTxEngine<6> and DmaRxEngine<7> (`i2c.*.dma`, a phase under
//      its dma_min_bytes on its own pump) - in THIS image; the SRAM twin
//      carries the pump alone, its RAM having no room for the engines'
//      handler code beside the others' and a stack. SERCOM3_Handler (and,
//      for the engined host, DMAC_Handler) bound metered or plain per run
//      (a flag): every op is the BEST of 8 by wall, metered (the thread
//      idles masked between the vector's edges, as letter d's waits) and
//      plain (`.bare`: the thread spins on the completion edge and no
//      stamp runs in either vector); n counts data bytes:
//        i2c.write       n = 1, 2, 16, 255 bytes after the address;
//        i2c.read        n = 1, 2, 16, 255, every read's bytes checked to
//                        be the peer's consecutive stream;
//        i2c.wr          one byte written, a repeated START, n - 1 = 1, 2,
//                        16 read;
//        i2c.probe       the address alone, ACKed (n=0);
//        i2c.probe.nack  to 0x77, nobody's: i2c_nack_addr;
//        i2c.read.nack   a one-byte READ to 0x77: i2c_nack_addr too;
//        i2c.write.dma n=254  the engined write's longest block (LEN
//                        counts to 255 and the engine writes LEN = w + 1;
//                        its 255 is the pump's);
//        i2c.write.nack.dma, i2c.read.nack.dma  16 bytes to 0x77 on the
//                        engines: i2c_nack_addr from LENERR;
//        i2c.write.nackdata(.dma)  after the speeds, the peer told to
//                        NACK the 3rd byte it receives: a 16-byte write at
//                        100 kHz on each host is i2c_nack_data.
//      The line after each prints the wire's cycles and wall minus them,
//      and after a metered line the fewest cycles start() took (one
//      ruler read in them). Each speed closes with the engined host's
//      count of start() calls that found the last engined read's SB
//      standing and waited, and with eight pairs of engined 16-byte
//      reads BACK TO BACK - the second started the moment the first's
//      edge is seen, as an arbiter with a request queued does - judged
//      both, with the second start()'s cycles.
//      The letter's verdict: every tenure completed with its status, every
//      read exact. No vendor library is on this desk: the data sheet's
//      own sequence, polled, is the vendor's column, in a scratch program
//      beside this one (the numbers in docs/samc21/i2c.md).
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
//   uart.*  the loop's rate over the ten bits of an 8N1 frame; uart.edge
//           wire=0 (a latency, not a rate).
//   i2c.*   the tenure's own time on the bus: its SCL rising edges - nine
//           a frame (the address and every data byte with its
//           acknowledge), one for a repeated START, one for the STOP -
//           times the SCL period the register pair in force gives on this
//           node, 10 + BAUD + BAUDLOW + f x T_RISE cycles (33.6.2.4.1),
//           T_RISE the node's 166 ns as the AVR128DB48's TCB meters measure
//           it (test_avr_twi letter b): PA23 cannot be both SERCOM3's pad
//           and an EIC line, so this chip cannot time its own SCL. As n
//           data bytes over that time; the probes wire=0.
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
#include "samc21/i2c.hpp"
#include "samc21/nvic.hpp"
#include "samc21/platform.hpp"
#include "samc21/sercom.hpp"
#include "samc21/spi.hpp"
#include "samc21/tc.hpp"
#include "samc21/ticker.hpp"
#include "util/bench.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

// The twi_link wire format the peer speaks: one file, included by its
// relative path from every board that speaks it.
#include "../../../avrdx/src/apps/twi_link.hpp"

// The placement of the bound vectors and of letter r's stamp loop: flash
// here; in bench_samc_ram.cpp, SRAM through the attribute this family's
// binding pattern documents (docs/samc21/platform.md, "A handler in
// SRAM"), spelled as an app spells it on the handler it binds.
#if defined(BENCH_RAM_TEXT)
#define BENCH_PLACEMENT [[gnu::section(".ram_text")]]
constexpr const char* bench_image = "bench_samc_ram";
constexpr const char* vectors_in = "SRAM (.ram_text)";
/// Letter i's DMA engines are this image's only: the SRAM twin has no
/// room for their handler code beside the others' and a stack (the file
/// header, letter i).
constexpr bool i2c_engines = false;
#else
constexpr bool i2c_engines = true;
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
IsrMeter<Ruler, Idle> sercom1_meter;
IsrMeter<Ruler, Idle> sercom3_meter;

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
        c0_ = bench_counters<Idle>(sercom_meter, tick_meter, dmac_meter, sercom1_meter, sercom3_meter);
        sw_.start();
    }
    [[gnu::always_inline]] uint32_t elapsed() const { return sw_.elapsed(); }
    [[gnu::always_inline]] BenchSample stop() {
        SamPlatform::CriticalSection guard;
        const uint32_t wall = sw_.elapsed();
        return bench_sample(wall, c0_, bench_counters<Idle>(sercom_meter, tick_meter, dmac_meter, sercom1_meter, sercom3_meter));
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

// =============================================================================
// u - the UART transport on a loop: uart.tx, uart.rx, uart.edge
// =============================================================================
namespace uu {

/// SERCOM1 as a USART whose receiver listens on its own transmitter's pad:
/// TxD and RxD both on PAD[0], PA16 under function C - the loop-back
/// "through the pad" of 31.6.3.8, no wire. PA16 is letter d's MOSI, with
/// nothing wired to it; the three transports below and letter d's and e's
/// SPI hosts share SERCOM1 and are never up at once (`live`).
constexpr UartPads loop_pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad0,
    .tx_pin = {'A', 16, PinFunction::c},
    .rx_pin = {'A', 16, PinFunction::c},
};
constexpr uint8_t ch_tx = 4;
constexpr uint8_t ch_rx = 5;

/// The three shapes: the plain transport (a 4096 ring, so a 4096-byte run
/// is queued in one call but its last byte), the transmit engine beside
/// the interrupt receiver, the receive engine beside the interrupt
/// transmitter (the pair is refused, erratum 1.10.4).
using LoopIrq = Uart<1, loop_pads, 512, 4096>;
using LoopTxE = Uart<1, loop_pads, 512, 2048, DmaTxEngine<ch_tx>, NoDmaEngine>;
using LoopRxE = Uart<1, loop_pads, 512, 512, NoDmaEngine, DmaRxEngine<ch_rx>>;

enum class Live : uint8_t { none, irq, txe, rxe };
volatile Live live = Live::none;

/// The edge's stamp: when the receive ring first holds `want` bytes, in
/// whichever context published the last of them.
volatile bool stamping = false;
volatile uint32_t want = 0;
volatile uint32_t edge_at = 0;
volatile bool edge_seen = false;
volatile bool edge_by_vector = false;

/// `pending` is asked only while a stamp is wanted: outside letter u's
/// edge lines the vectors pay one flag test for it.
template <typename Pending>
[[gnu::always_inline]] inline void check_edge(Pending pending, bool vector) {
    if (stamping && !edge_seen && pending() >= want) {
        edge_at = Ruler::now();
        edge_seen = true;
        edge_by_vector = vector;
    }
}

constexpr uint32_t rates[] = {115'200u, 1'000'000u, 3'000'000u};

/// A run's counters at one instant: every meter (for busy), and SERCOM1's
/// and the DMAC's apart (the irq and isr a line names), with the ruler.
struct Mark {
    BenchCounters all, s1, dm, s5;
    uint32_t t;
};
[[gnu::always_inline]] inline Mark mark() {
    SamPlatform::CriticalSection guard;
    return {bench_counters<Idle>(sercom_meter, tick_meter, dmac_meter, sercom1_meter),
            bench_counters<Idle>(sercom1_meter), bench_counters<Idle>(dmac_meter),
            bench_counters<Idle>(sercom_meter), Ruler::now()};
}
struct Run {
    BenchSample all, s1, dm, s5;
};
Run between(const Mark& a, const Mark& b) {
    const uint32_t wall = b.t - a.t;
    return {bench_sample(wall, a.all, b.all), bench_sample(wall, a.s1, b.s1),
            bench_sample(wall, a.dm, b.dm), bench_sample(wall, a.s5, b.s5)};
}
/// The line: the wall and busy of the run, the irq and isr of the vector
/// named.
BenchSample line_of(const BenchSample& all, const BenchSample& vec) {
    return {all.wall, all.busy, vec.irq, vec.isr};
}

const uint8_t* bytes_of(uint32_t n) {
    return reinterpret_cast<const uint8_t*>(payload.data()) + (max_size - n);
}

uint32_t char_cycles(uint32_t baud) { return 10u * SysClock::hz / baud; }

/// The console falls silent (letter p's drain: a line still leaving puts
/// SERCOM5's interrupts in the run) and the loop's line idles a few frames.
void settle(uint32_t baud) {
    (void)drain();
    const uint32_t t0 = Ruler::now();
    const uint32_t c = 4u * char_cycles(baud);
    while (Ruler::now() - t0 < c) {
    }
}

/// The per-unit figures under a line: interrupts and handler cycles per
/// byte, in hundredths.
void per_byte(const char* what, const BenchSample& v, uint32_t n) {
    const uint32_t irq100 = v.irq * 100u / n;
    const uint32_t isr100 = v.isr * 100u / n;
    print(serial, "  ", what, ": ", irq100 / 100u, '.', static_cast<char>('0' + irq100 % 100u / 10u),
          static_cast<char>('0' + irq100 % 10u), " interrupts a byte, ", isr100 / 100u, '.',
          static_cast<char>('0' + isr100 % 100u / 10u), static_cast<char>('0' + isr100 % 10u),
          " handler cycles a byte", crlf);
}

template <typename U>
void counters_line() {
    print(serial, "  counters: frame ", U::frame_errors(), " parity ", U::parity_errors(),
          " hw_overrun ", U::hw_overruns(), " rx_overrun ", U::rx_overruns(), crlf);
}

/// uart.tx: n bytes through U, then the wait until the last stop bit
/// (TXC): the plain transport's thread SPINS (busy = wall, the shape is
/// irq and isr), the engined one IDLES between completions (busy is the
/// CPU's share). `handed` says the transport has given the hardware its
/// last byte: DRE disarmed on the plain one, the engine free on the other.
template <typename U, bool engined, typename Handed>
Run tx_run(uint32_t n, Handed handed, bool& ok) {
    const uint8_t* p = bytes_of(n);
    const Mark a = mark();
    uint32_t done = U::write_bulk(std::span<const uint8_t>(p, n));
    uint32_t spins = 0;
    while (done < n && spins++ < 40'000'000u) {
        if constexpr (engined) {
            disable_interrupts();
            Idle::idle();
        }
        done += U::write_bulk(std::span<const uint8_t>(p + done, n - done));
    }
    spins = 0;
    for (;;) {
        if constexpr (engined) {
            disable_interrupts();
            if (handed()) {
                enable_interrupts();
                break;
            }
            Idle::idle();
        } else if (handed()) {
            break;
        }
        if (++spins > 40'000'000u) {
            break;
        }
    }
    spins = 0;
    while (!U::Resource::txc_flag() && spins++ < 400'000u) {
    }
    const Mark b = mark();
    ok = done == n && U::Resource::txc_flag();
    return between(a, b);
}

void tx_lines() {
    for (const uint32_t baud : rates) {
        for (const uint32_t n : {256u, 4096u}) {
            live = Live::irq;
            bool ok = false;
            (void)LoopIrq::init(clock, baud);
            LoopIrq::Resource::enable_rxc_interrupt(false);   // the loop's receiver is not measured here
            settle(baud);
            const Run r = tx_run<LoopIrq, false>(
                n, [] { return (LoopIrq::Resource::armed() & SercomFlag::dre) == 0u; }, ok);
            LoopIrq::release();
            live = Live::none;
            bench_line(serial, "uart.tx", n, line_of(r.all, r.s1), Ruler::hz(), baud / 10u);
            per_byte("SERCOM1", r.s1, n);
            if (!ok) {
                print(serial, "  the run did not complete", crlf);
            }
        }
    }
    for (const uint32_t baud : rates) {
        for (const uint32_t n : {256u, 4096u}) {
            live = Live::txe;
            bool ok = false;
            (void)LoopTxE::init(clock, baud);
            LoopTxE::Resource::enable_rxc_interrupt(false);
            settle(baud);
            const uint32_t turns0 = Idle::idle_turns();
            const Run r =
                tx_run<LoopTxE, true>(n, [] { return !DmaTxEngine<ch_tx>::busy(); }, ok);
            const uint32_t idle_turns = Idle::idle_turns() - turns0;
            LoopTxE::release();
            live = Live::none;
            bench_line(serial, "uart.tx.dma", n, line_of(r.all, r.dm), Ruler::hz(), baud / 10u);
            print(serial, "  DMAC ", r.dm.irq, " completions, SERCOM1 ", r.s1.irq,
                  " interrupts, ", idle_turns, " idle turns, console ", r.s5.irq,
                  " interrupts (rx ", Serial::rx_pending(), ", frame ", Serial::frame_errors(),
                  "); CPU share ",
                  r.all.busy * 100u / r.all.wall, " per cent", crlf);
            if (!ok) {
                print(serial, "  the run did not complete", crlf);
            }
        }
    }
}

/// Compare what the ring holds with what was sent, and release it.
template <typename U>
uint32_t check_and_consume(const uint8_t* sent, uint32_t n) {
    uint32_t bad = 0;
    uint32_t seen = 0;
    while (seen < n) {
        const auto run = U::read_span();
        if (run.empty()) {
            break;
        }
        for (uint32_t i = 0; i < run.size() && seen < n; ++i, ++seen) {
            bad += run[i] != sent[seen] ? 1u : 0u;
        }
        U::consume(static_cast<uint32_t>(run.size()));
    }
    return bad + (n - seen);
}

/// uart.rx through the interrupt receiver: the transmit engine sends the
/// burst (its completions are the DMAC's, apart), the thread spins until
/// the ring holds it; irq and isr are SERCOM1's - the receiver's shape,
/// one entry for every byte or for every level the handler takes.
void rx_lines() {
    for (const uint32_t baud : rates) {
        for (const uint32_t n : {16u, 256u}) {
            live = Live::txe;
            (void)LoopTxE::init(clock, baud);
            settle(baud);
            const uint8_t* p = bytes_of(n);
            const Mark a = mark();
            (void)LoopTxE::write_bulk(std::span<const uint8_t>(p, n));
            uint32_t spins = 0;
            while (LoopTxE::rx_pending() < n && spins++ < 4'000'000u) {
            }
            const Mark b = mark();
            const Run r = between(a, b);
            const uint32_t bad = check_and_consume<LoopTxE>(p, n);
            bench_line(serial, "uart.rx", n, line_of(r.all, r.s1), Ruler::hz(), baud / 10u);
            per_byte("SERCOM1 (the receiver)", r.s1, n);
            print(serial, "  wrong or missing bytes ", bad, crlf);
            counters_line<LoopTxE>();
            LoopTxE::release();
            live = Live::none;
        }
    }
    // Through the receive engine: the interrupt transmitter sends (SERCOM1's
    // entries are the TRANSMIT side here), the DMAC's interrupts are the
    // receiver's; the transport is brought up anew before each burst so
    // the burst starts the engine's first block. What the owner does with
    // no edge is ask: harvest() once a tick, the thread spinning between.
    for (const uint32_t baud : rates) {
        for (const uint32_t n : {16u, 256u}) {
            live = Live::rxe;
            (void)LoopRxE::init(clock, baud);
            settle(baud);
            const uint8_t* p = bytes_of(n);
            const Mark a = mark();
            (void)LoopRxE::write_bulk(std::span<const uint8_t>(p, n));
            uint32_t tick = Ticker::ticks();
            uint32_t asks = 0;
            uint32_t spins = 0;
            while (LoopRxE::rx_pending() < n && spins++ < 4'000'000u) {
                if (Ticker::ticks() != tick) {
                    tick = Ticker::ticks();
                    ++asks;
                    (void)LoopRxE::harvest();
                }
            }
            const Mark b = mark();
            const Run r = between(a, b);
            const uint32_t bad = check_and_consume<LoopRxE>(p, n);
            bench_line(serial, "uart.rx.dma", n, line_of(r.all, r.dm), Ruler::hz(), baud / 10u);
            per_byte("DMAC (the receiver)", r.dm, n);
            print(serial, "  SERCOM1 (the transmitter) ", r.s1.irq, " interrupts; the owner asked ",
                  asks, " times; wrong or missing bytes ", bad, crlf);
            counters_line<LoopRxE>();
            LoopRxE::release();
            live = Live::none;
        }
    }
}

/// uart.edge: the cycles from the burst's last stop bit to the receive
/// ring holding the burst's last byte, in the context that published it.
/// The thread spins until the transmitter holds the burst's last byte
/// (`handed`), then MASKS and spins on its TXC: TXC's rise is stamped
/// exactly, and the receive side's interrupt, pended under the mask,
/// runs the moment it lifts - so the number is the handler's own latency
/// past the last stop bit. The receiver had the byte BEFORE that: it
/// takes the frame at its stop bit's middle sample (31.6.2.6), seven
/// sixteenths of a bit before TXC. The mask lasts the last two frames at
/// most, which the receiver's two levels hold.
template <typename U, typename Handed>
void edge_once(const char* op, uint32_t baud, uint32_t n, Handed handed) {
    settle(baud);
    const uint8_t* p = bytes_of(n);
    want = n;
    edge_seen = false;
    stamping = true;
    (void)U::write_bulk(std::span<const uint8_t>(p, n));
    uint32_t lead = 0;
    while (!handed() && lead++ < 4'000'000u) {
        asm volatile("" ::: "memory");   // the engine's busy flag is a plain bool
    }
    disable_interrupts();
    uint32_t spins = 0;
    while (!U::Resource::txc_flag() && spins++ < 400'000u) {
    }
    const uint32_t t_txc = Ruler::now();
    enable_interrupts();
    uint32_t tick = Ticker::ticks();
    uint32_t asks = 0;
    spins = 0;
    while (!edge_seen && spins++ < 4'000'000u) {
        if (Ticker::ticks() != tick) {
            tick = Ticker::ticks();
            ++asks;
            (void)U::harvest();
            check_edge([] { return U::rx_pending(); }, false);
        }
    }
    stamping = false;
    const uint32_t bad = check_and_consume<U>(p, n);
    const BenchSample s{edge_seen ? edge_at - t_txc : 0u, 0u, 0u, 0u};
    bench_line(serial, op, n, s, Ruler::hz(), 0u);
    print(serial, "  at ", baud, " baud: the edge ", edge_seen ? "seen" : "NOT SEEN", " by the ",
          edge_by_vector ? "vector" : "owner's ask", " (", asks, " asks), ",
          static_cast<int32_t>(edge_at - t_txc), " cycles past TXC; a frame is ",
          char_cycles(baud), " cycles; wrong or missing bytes ", bad, crlf);
}

void edge_lines() {
    // The interrupt receiver's edge: a burst of four from the plain
    // transmitter on the same vector, so no other handler is pended with
    // the receiver's when the mask lifts - DRE disarms as the last byte
    // enters DATA, two frames before its stop bit. ONE RATE: the handler
    // serves a pended RXC at its own latency whatever the rate, and from
    // 1 Mbaud the transmitter's and the receiver's entries leave the thread
    // too little of a frame to mask before the last RXC (the write itself
    // outlasts the four frames), so the stamps cannot be taken apart there.
    for (const uint32_t baud : {115'200u}) {
        live = Live::irq;
        (void)LoopIrq::init(clock, baud);
        edge_once<LoopIrq>("uart.edge", baud, 4u, [] {
            return (LoopIrq::Resource::armed() & SercomFlag::dre) == 0u;
        });
        LoopIrq::release();
        live = Live::none;
    }
    for (const uint32_t baud : {115'200u, 1'000'000u}) {
        for (const uint32_t n : {16u, 256u}) {
            live = Live::rxe;
            (void)LoopRxE::init(clock, baud);
            edge_once<LoopRxE>("uart.edge.dma", baud, n, [] {
                return (LoopRxE::Resource::armed() & SercomFlag::dre) == 0u;
            });
            LoopRxE::release();
            live = Live::none;
        }
    }
}

/// The copy into the ring: write_bulk() of n bytes on the engined
/// transport, freshly brought up (the ring's head at its first slot), the
/// source at each of the four offsets from a word - one of them shares the
/// ring storage's alignment. The call is timed masked, from its entry to
/// its return: the copy and the one pump that launches the block, so the
/// difference of two sizes is the copy alone.
void copy_lines() {
    constexpr uint32_t baud = 3'000'000u;
    for (const uint32_t n : {1u, 4u, 8u, 16u, 32u, 64u, 128u, 256u, 1024u}) {
        uint32_t lo = 0xFFFFFFFFu;
        uint32_t hi = 0;
        uint32_t walls[4] = {};
        for (uint32_t off = 0; off < 4u; ++off) {
            live = Live::txe;
            (void)LoopTxE::init(clock, baud);
            LoopTxE::Resource::enable_rxc_interrupt(false);
            settle(baud);
            const uint8_t* p = bytes_of(max_size) + off;
            disable_interrupts();
            const uint32_t t0 = Ruler::now();
            (void)LoopTxE::write_bulk(std::span<const uint8_t>(p, n));
            const uint32_t w = Ruler::now() - t0;
            enable_interrupts();
            uint32_t spins = 0;
            while (DmaTxEngine<ch_tx>::busy() && spins++ < 4'000'000u) {
            }
            LoopTxE::release();
            live = Live::none;
            walls[off] = w;
            lo = w < lo ? w : lo;
            hi = w > hi ? w : hi;
        }
        print(serial, "  uart.copy write_bulk n=", n, ": ", walls[0], ' ', walls[1], ' ', walls[2],
              ' ', walls[3], " cycles at source offsets 0..3 (low ", lo, ", high ", hi, ')',
              crlf);
    }
}

/// The completions this letter's engines raise, from DMAC_Handler.
[[gnu::always_inline]] inline bool dmac_dispatch(const DmaInterrupt& irq) {
    if (live == Live::txe && irq.channel == ch_tx) {
        (void)LoopTxE::dma_isr(irq.channel);
        return true;
    }
    if (live == Live::rxe && irq.channel == ch_rx) {
        (void)LoopRxE::dma_isr(irq.channel);
        check_edge([] { return LoopRxE::rx_pending(); }, true);
        return true;
    }
    return false;
}

}  // namespace uu

void tu_uart() {
    if (!Dmac::init()) {
        print(serial, "  the DMAC did not come up", crlf);
        bench.verdict("ran", false);
        return;
    }
    (void)drain();
    uu::tx_lines();
    uu::rx_lines();
    uu::edge_lines();
    uu::copy_lines();
    Dmac::release();
    bench.verdict("ran", true);
}

// =============================================================================
// i - the I2C host: i2c.write, i2c.read, i2c.wr, i2c.probe
// =============================================================================
namespace di {

/// SERCOM3 on PA22 (SDA, PAD[0]) / PA23 (SCL, PAD[1]), function C - the
/// pads test_samc_i2c runs on - on the desk's 5 V node with the peer.
constexpr I2cPads bus_pads{
    .sda_pin = {'A', 22, PinFunction::c},
    .scl_pin = {'A', 23, PinFunction::c},
};
using Pump = I2cHost<3, bus_pads>;
constexpr uint8_t ch_i2c_tx = 6;   ///< free in every other letter
constexpr uint8_t ch_i2c_rx = 7;
using Eng = I2cHost<3, bus_pads, 0, DmaTxEngine<ch_i2c_tx>, DmaRxEngine<ch_i2c_rx>>;

/// The bus's rise time, measured on this node by the AVR128DB48's
/// TCB meters (test_avr_twi letter b: the SCL period over the register's
/// floor): 166 ns. The chapter's formula (33.6.2.4.1) with it is the SCL
/// period the wire field charges, and init() is told the same figure.
constexpr uint32_t rise_ns = 166u;
constexpr uint32_t rise_cycles = (SysClock::hz / 1000u * rise_ns + 500'000u) / 1'000'000u;

constexpr uint8_t peer_addr = 0x2C;       ///< the address the peer serves at
constexpr uint8_t absent_addr = 0x77;     ///< nobody's

/// Which host the SERCOM3 vector serves, and whether it is metered.
enum class Live : uint8_t { none, pump, engine };
volatile Live live = Live::none;
volatile bool metered = false;
volatile bool done = false;

/// Letter d's SPI buffers, reused: the letters run one at a time, and the
/// SRAM-placed twin of this image has no room for two more.
uint8_t (&i2c_tx)[256] = dd::spi_tx;
uint8_t (&i2c_rx)[256] = dd::spi_rx;

template <typename H>
[[gnu::always_inline]] inline void serve_vector() {
    if (H::isr()) {
        done = true;
    }
}

/// One tenure on the pump, spun to its completion, bounded: the command
/// channel's own tenures.
uint8_t link(uint8_t addr, const uint8_t* tx, uint8_t txn, uint8_t* rx, uint8_t rxn) {
    Pump::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = txn;
    r.rx = lend<Lease::reply>(rx);
    r.rx_len = rxn;
    r.speed = I2cSpeed::standard_100k;
    done = false;
    live = Live::pump;
    if (Pump::start(r)) {
        return Pump::status();
    }
    const uint32_t t0 = Ticker::millis();
    while (!done) {
        if (Ticker::millis() - t0 > 300u) {
            (void)Pump::recover();
            return 0xEE;
        }
    }
    return Pump::status();
}

/// The command channel's frame and answer, in letter m's buffers (free
/// while this letter runs: the SRAM twin's room is tight).
uint8_t (&frame_buf)[twilink::max_payload + 4] =
    reinterpret_cast<uint8_t (&)[twilink::max_payload + 4]>(mem_src);
uint8_t (&resp_buf)[twilink::response_bytes] =
    reinterpret_cast<uint8_t (&)[twilink::response_bytes]>(mem_dst);

void settle_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

/// One command frame to the peer and its ack collected (twi_link.hpp).
bool command(twilink::Op op, const uint8_t* p, uint8_t len) {
    for (uint8_t k = 0; k < 3u; ++k) {
        uint8_t n = 0;
        twilink::write_frame(
            [&](uint8_t b) {
                if (n < sizeof frame_buf) frame_buf[n++] = b;
            },
            op, p, len);
        if (link(twilink::command_addr, frame_buf, n, nullptr, 0) == i2c_ok) {
            settle_ms(2);
            if (link(twilink::command_addr, nullptr, 0, resp_buf, twilink::response_bytes) == i2c_ok) {
                twilink::Decoder dec;
                for (const uint8_t b : resp_buf) {
                    if (dec.feed(b) == twilink::Decoder::Result::frame) {
                        const twilink::Frame& f = dec.frame();
                        if (f.op == twilink::Op::ack && f.len == 2 && f.data[0] == twilink::byte_of(op)) {
                            settle_ms(twilink::arm_ms);
                            return true;
                        }
                    }
                }
            }
        }
        settle_ms(400);
    }
    return false;
}

/// The peer becomes a client at peer_addr for `ms` milliseconds, serving
/// 0, 1, 2, ... to a read and taking any write.
bool serve(uint16_t ms) {
    twilink::Params a{};
    a.addr = peer_addr;
    a.count = 0;
    a.ms = ms;
    uint8_t p[twilink::params_size];
    twilink::put_params(p, a);
    return command(twilink::Op::serve, p, twilink::params_size);
}

struct Speed {
    I2cSpeed speed;
    const char* name;
};
constexpr Speed speeds[] = {{I2cSpeed::standard_100k, "100 kHz"},
                            {I2cSpeed::fast_400k, "400 kHz"},
                            {I2cSpeed::fast_plus_1m, "1 MHz (Fm+)"}};
constexpr uint8_t lengths[] = {1u, 2u, 16u, 255u};
constexpr uint8_t wr_lengths[] = {1u, 2u, 16u};

/// SCL rising edges in a tenure: nine a frame, one for a repeated START,
/// one for the STOP.
constexpr uint32_t rises(uint8_t tx, uint8_t rx, bool probe) {
    if (probe) {
        return 10u;
    }
    uint32_t r = 1u;
    if (tx != 0u) {
        r += 9u * (1u + tx);
    }
    if (rx != 0u) {
        r += 9u * (1u + rx) + (tx != 0u ? 1u : 0u);
    }
    return r;
}

/// The SCL period the register pair in force produces on this node, in
/// CLK_CPU cycles: 10 + BAUD + BAUDLOW + f x T_RISE (33.6.2.4.1).
template <typename H>
uint32_t period(I2cSpeed s) {
    const I2cBaud b = H::baud_of(s);
    return 10u + b.baud + (b.baudlow != 0u ? b.baudlow : b.baud) + rise_cycles;
}

/// One tenure, the best of 8 by wall: metered, the thread idles masked
/// between the vector's edges as the kernel's loop does; plain, it spins
/// on the edge and no stamp runs. Bounded at 100 ms.
/// The fewest cycles a metered run's start() took, from just before the
/// call to its return - one ruler read included (letter r's `ruler`).
uint32_t launch_min = 0;

template <typename H>
BenchSample best_tenure(const typename H::Request& r, uint8_t expect, bool spin, bool& ok) {
    BenchSample best{};
    Interval iv;
    ok = true;
    launch_min = 0xFFFFFFFFu;
    for (uint8_t run = 0; run < 8u; ++run) {
        done = false;
        iv.start();
        bool finished = false;
        if (spin) {
            finished = H::start(r);
        } else {
            const uint32_t t0 = Ruler::now();
            finished = H::start(r);
            const uint32_t launch = Ruler::now() - t0;
            launch_min = launch < launch_min ? launch : launch_min;
        }
        while (!finished) {
            if (spin) {
                if (done) {
                    finished = true;
                } else if (iv.elapsed() >= SysClock::hz / 10u) {
                    break;
                }
                continue;
            }
            disable_interrupts();
            if (done) {
                enable_interrupts();
                finished = true;
                break;
            }
            if (iv.elapsed() >= SysClock::hz / 10u) {
                enable_interrupts();
                break;
            }
            Idle::idle();
        }
        const BenchSample smp = iv.stop();
        ok = ok && finished && H::status() == expect;
        if (run == 0u || smp.wall < best.wall) {
            best = smp;
        }
        (void)delay_us(clock, 100);   // the STOP on the wire, outside the next wall
    }
    return best;
}

template <typename H>
bool op(const char* name, const char* bare, I2cSpeed s, uint8_t addr, uint8_t tx, uint8_t rx,
        bool probe, uint8_t expect) {
    typename H::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(i2c_tx));
    r.tx_len = tx;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(i2c_rx));
    r.rx_len = rx;
    r.speed = s;
    const uint32_t wire = rises(tx, rx, probe) * period<H>(s);
    const uint32_t n = probe ? 0u : static_cast<uint32_t>(tx) + rx;
    const uint32_t wire_bps =
        n == 0u ? 0u : static_cast<uint32_t>((static_cast<uint64_t>(n) * SysClock::hz + wire / 2u) / wire);
    bool all = true;
    for (const bool m : {true, false}) {
        (void)drain();
        metered = m;
        bool ok = false;
        for (uint8_t& b : i2c_rx) {
            b = 0xEE;
        }
        const BenchSample smp = best_tenure<H>(r, expect, !m, ok);
        metered = false;
        // The peer serves one counting stream for its whole action (its
        // index is not reset at an address): a read is exact when its bytes
        // are consecutive.
        bool exact = true;
        if (expect == i2c_ok) {
            for (uint8_t k = 1; k < rx; ++k) {
                exact = exact && i2c_rx[k] == static_cast<uint8_t>(i2c_rx[0] + k);
            }
        }
        bench_line(serial, m ? name : bare, n, smp, SysClock::hz, wire_bps);
        print(serial, "  wire ", wire, " cycles (", rises(tx, rx, probe), " SCL rises x ",
              period<H>(s), "): above the wire ", static_cast<int32_t>(smp.wall - wire));
        if (m) {
            print(serial, ", start() ", launch_min, " cycles");
        }
        print(serial, ok ? "" : " - STATUS OR COMPLETION WRONG", exact ? "" : " - READ NOT EXACT", crlf);
        all = all && ok && exact;
    }
    return all;
}

/// The op names of one host: the pump's plain names, the engines' with
/// ".dma", each with its ".bare" twin.
struct Names {
    const char* write[2];
    const char* read[2];
    const char* wr[2];
    const char* probe[2];
    const char* probe_nack[2];
    const char* read_nack[2];
};
constexpr Names pump_names{{"i2c.write", "i2c.write.bare"},
                           {"i2c.read", "i2c.read.bare"},
                           {"i2c.wr", "i2c.wr.bare"},
                           {"i2c.probe", "i2c.probe.bare"},
                           {"i2c.probe.nack", "i2c.probe.nack.bare"},
                           {"i2c.read.nack", "i2c.read.nack.bare"}};
constexpr Names dma_names{{"i2c.write.dma", "i2c.write.dma.bare"},
                          {"i2c.read.dma", "i2c.read.dma.bare"},
                          {"i2c.wr.dma", "i2c.wr.dma.bare"},
                          {"i2c.probe.dma", "i2c.probe.dma.bare"},
                          {"i2c.probe.nack.dma", "i2c.probe.nack.dma.bare"},
                          {"i2c.read.nack.dma", "i2c.read.nack.dma.bare"}};

/// Two engined 16-byte reads BACK TO BACK, the second started the moment
/// the first's completion edge is seen (the thread spinning on it), as an
/// arbiter with a request queued would: the case the engined read's SB
/// tail exists for (samc21/i2c.hpp). Eight pairs; prints how many second
/// starts found SB standing, and judges both reads.
bool back_to_back(I2cSpeed s) {
    Eng::Request r[2]{};
    for (uint8_t leg = 0; leg < 2u; ++leg) {
        r[leg].addr = peer_addr;
        r[leg].rx = lend<Lease::reply>(static_cast<uint8_t*>(i2c_rx + 16u * leg));
        r[leg].rx_len = 16;
        r[leg].speed = s;
    }
    const uint32_t w0 = Eng::tail_waits();
    bool ok = true;
    uint32_t lo = 0xFFFFFFFFu;
    uint32_t hi = 0;
    for (uint8_t k = 0; k < 8u; ++k) {
        done = false;
        bool sync = Eng::start(r[0]);
        uint32_t spins = 0;
        while (!sync && !done && ++spins < 4'000'000u) {
        }
        ok = ok && !sync && done && Eng::status() == i2c_ok;
        done = false;
        const uint32_t t0 = Ruler::now();
        sync = Eng::start(r[1]);
        const uint32_t gap = Ruler::now() - t0;
        lo = gap < lo ? gap : lo;
        hi = gap > hi ? gap : hi;
        spins = 0;
        while (!sync && !done && ++spins < 4'000'000u) {
        }
        ok = ok && !sync && done && Eng::status() == i2c_ok;
        for (uint8_t i = 1; i < 32u; ++i) {
            ok = ok && (i == 16u || i2c_rx[i] == static_cast<uint8_t>(i2c_rx[i - 1u] + 1u));
        }
        (void)delay_us(clock, 200);
    }
    print(serial, "  i2c.read.dma back to back, 8 pairs of 16: ", ok ? "all i2c_ok and exact" : "FAILED",
          "; the second start() found SB standing ", Eng::tail_waits() - w0,
          " times; that start() took ", lo, "..", hi, " cycles (a ruler read in it)", crlf);
    return ok;
}

/// Every op at one speed, on host H.
template <typename H>
bool ops(I2cSpeed s, const Names& nm) {
    bool all = true;
    for (const uint8_t n : lengths) {
        all = op<H>(nm.write[0], nm.write[1], s, peer_addr, n, 0, false, i2c_ok) && all;
    }
    for (const uint8_t n : lengths) {
        all = op<H>(nm.read[0], nm.read[1], s, peer_addr, 0, n, false, i2c_ok) && all;
    }
    for (const uint8_t n : wr_lengths) {
        all = op<H>(nm.wr[0], nm.wr[1], s, peer_addr, 1, n, false, i2c_ok) && all;
    }
    if constexpr (H::tx_engined) {
        // LEN counts to 255 and the engined write needs LEN = w + 1: its
        // longest block is 254 bytes, a 255-byte write the pump's.
        all = op<H>(nm.write[0], nm.write[1], s, peer_addr, 254, 0, false, i2c_ok) && all;
        // The engined phases' address NACK: LENERR + ERROR under LEN, the
        // STOP the silicon's - i2c_nack_addr, for a write and a read.
        all = op<H>("i2c.write.nack.dma", "i2c.write.nack.dma.bare", s, absent_addr, 16, 0, false,
                    i2c_nack_addr) && all;
        all = op<H>("i2c.read.nack.dma", "i2c.read.nack.dma.bare", s, absent_addr, 0, 16, false,
                    i2c_nack_addr) && all;
    }
    all = op<H>(nm.probe[0], nm.probe[1], s, peer_addr, 0, 0, true, i2c_ok) && all;
    all = op<H>(nm.probe_nack[0], nm.probe_nack[1], s, absent_addr, 0, 0, true, i2c_nack_addr) && all;
    all = op<H>(nm.read_nack[0], nm.read_nack[1], s, absent_addr, 0, 1, true, i2c_nack_addr) && all;
    return all;
}

}  // namespace di

void ti_i2c() {
    using namespace di;
    for (uint16_t i = 0; i < 256u; ++i) {
        i2c_tx[i] = static_cast<uint8_t>(i * 7u + 3u);
    }
    (void)GclkChannel::connect(Sercom<3>::gclk_slow_id(), 0);
    live = Live::pump;
    if (!Pump::init(clock, rise_ns)) {
        print(serial, "  the I2C host did not come up", crlf);
        bench.verdict("ran", false);
        return;
    }
    if (!Dmac::init()) {
        print(serial, "  the DMAC did not come up", crlf);
        bench.verdict("ran", false);
        return;
    }
    bool all = true;
    for (const Speed& sp : speeds) {
        live = Live::pump;
        (void)Pump::init(clock, rise_ns);
        if (!serve(15000u)) {
            print(serial, "  THE PEER DID NOT ANSWER on 0x6B: board B must run twi_peer", crlf);
            all = false;
            break;
        }
        const uint32_t t0 = Ticker::millis();
        print(serial, "-- ", sp.name, ": BAUD ", Pump::baud_of(sp.speed).baud, '/',
              Pump::baud_of(sp.speed).baudlow, ", SCL period ", period<Pump>(sp.speed),
              " cycles (", SysClock::hz / period<Pump>(sp.speed), " Hz) with the node's ", rise_ns,
              " ns rise; the peer is twi_peer serving 0x2C", crlf);
        all = ops<Pump>(sp.speed, pump_names) && all;
        if constexpr (i2c_engines) {
            live = Live::engine;
            if (!Eng::init(clock, rise_ns)) {
                print(serial, "  the engined host did not come up", crlf);
                all = false;
            } else {
                const uint32_t w0 = Eng::tail_waits();
                all = ops<Eng>(sp.speed, dma_names) && all;
                print(serial, "  the engined reads' tail: ", Eng::tail_waits() - w0,
                      " start() calls found SB standing and waited", crlf);
                all = back_to_back(sp.speed) && all;
                Eng::release();
            }
        }
        live = Live::pump;
        (void)Pump::init(clock, rise_ns);
        print(serial, "  (", Ticker::millis() - t0, " ms with the peer serving)", crlf);
        while (Ticker::millis() - t0 < 15100u) {
        }
    }
    // A DATA NACK: the peer refuses the 3rd byte it receives, and a
    // 16-byte write on each host is i2c_nack_data - the pump's on its MB
    // with RXNACK, the engine's on LENERR with the block under way.
    {
        live = Live::pump;
        (void)Pump::init(clock, rise_ns);
        twilink::Params a{};
        a.addr = peer_addr;
        a.ms = 3000;
        a.nack_at = 3;
        uint8_t p[twilink::params_size];
        twilink::put_params(p, a);
        if (!command(twilink::Op::serve, p, twilink::params_size)) {
            print(serial, "  THE PEER DID NOT TAKE THE NACKING SERVE", crlf);
            all = false;
        } else {
            all = op<Pump>("i2c.write.nackdata", "i2c.write.nackdata.bare", I2cSpeed::standard_100k,
                           peer_addr, 16, 0, false, i2c_nack_data) && all;
            if constexpr (i2c_engines) {
                live = Live::engine;
                if (Eng::init(clock, rise_ns)) {
                    all = op<Eng>("i2c.write.nackdata.dma", "i2c.write.nackdata.dma.bare",
                                  I2cSpeed::standard_100k, peer_addr, 16, 0, false, i2c_nack_data) &&
                          all;
                    Eng::release();
                }
                live = Live::pump;
            }
            settle_ms(3100);
            (void)Pump::init(clock, rise_ns);
        }
    }
    Pump::release();
    live = Live::none;
    Dmac::release();
    bench.verdict("every tenure completed with its status, every read byte-exact", all);
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
    // Letter i's plain runs leave this vector unmetered too (its stamps
    // would sit inside the engined read's completion edge).
    const bool m = di::live != di::Live::engine || di::metered;
    if (m) {
        dmac_meter.enter();
    }
    while (const auto irq = brio::Dmac::take_pending()) {
        if (i2c_engines && di::live == di::Live::engine) {
            if constexpr (i2c_engines) {
                if (di::Eng::dma_isr(irq->channel, irq->flags)) {
                    di::done = true;
                }
            }
        } else if (!uu::dmac_dispatch(*irq)) {
            dd::dmac_dispatch(*irq);
        }
    }
    if (m) {
        dmac_meter.leave();
    }
}
extern "C" BENCH_PLACEMENT void SERCOM1_Handler() {
    sercom1_meter.enter();
    switch (uu::live) {
        case uu::Live::irq:
            (void)uu::LoopIrq::isr();
            uu::check_edge([] { return uu::LoopIrq::rx_pending(); }, true);
            break;
        case uu::Live::txe:
            (void)uu::LoopTxE::isr();
            uu::check_edge([] { return uu::LoopTxE::rx_pending(); }, true);
            break;
        case uu::Live::rxe: (void)uu::LoopRxE::isr(); break;
        default:
            if (ee::live) {
                if (ee::SpiPoll::isr()) {
                    ee::done = true;
                }
            } else if (dd::SpiHw::isr()) {
                dd::spi_done = true;
            }
            break;
    }
    sercom1_meter.leave();
}
extern "C" BENCH_PLACEMENT void SERCOM3_Handler() {
    if (di::metered) {
        sercom3_meter.enter();
    }
    if constexpr (i2c_engines) {
        if (di::live == di::Live::engine) {
            di::serve_vector<di::Eng>();
        } else {
            di::serve_vector<di::Pump>();
        }
    } else {
        di::serve_vector<di::Pump>();
    }
    if (di::metered) {
        sercom3_meter.leave();
    }
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
    bench.letter('u', "the UART on a loop: uart.tx, uart.rx, uart.edge", tu_uart);
    bench.letter('i', "the I2C host against the peer: i2c.write, i2c.read, i2c.wr, i2c.probe",
                 ti_i2c);

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
