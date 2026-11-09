// test_vx03_dma_serial - the reference bench suite for the USARTs'
// requests on the DMA controllers of the CH32V203 and the CH32V303
// (ch32vx03/dma.hpp over RM ch. 11) and the two engine slots
// ch32vx03/usart.hpp grew for them: the transmit engine and the receive
// ring a transport takes, the ring over a channel that does not serve,
// the priority rule beside an unpaced copy, an overrun at a message's
// tail, and where UART4 is the pair's link UART4's engines on the second
// controller.
//
// One of three suites over the chapter: test_vx03_dma walks every channel
// memory to memory, test_vx03_dma_paced takes the timers' requests. A
// channel has one owner for the whole image (docs/design/dma.md, "The
// ownership rule"), and here DMA1's channels 6 and 7 are USART2's, channel
// 3 the copy engine's and, on the CH32V203, channel 1 the fourth port's
// transmit; on the CH32V303 DMA2's channels 5 and 3 are UART4's and
// channel 1 the copy engine's.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// WHAT IT MEASURES WITH. The core's own STK counter is the ruler every
// number is weighed against, and the SOURCE of every run is a
// four-kilobyte pattern in the image itself. A SERIAL LINE NEEDS NO WIRE: the
// receive pad PA3 handed to USART2 is an input whose level follows its own
// pull, which the output data register moves, so software banging the
// pull puts 8N1 frames on the receive line at 115200 baud
// (test_vx03_serial's first fact) - and the receive ring of letter g takes
// bursts with nobody reading. ONE OPTIONAL JUMPER, PA2 to PA3 - USART2's
// transmitter to its own receiver - turns that half of letter g into a
// round trip: the letter says whether the strap is there (a plain output
// on one pad read on the other, both levels), bangs the line when it is
// not and goes round it when it is; the transmit half needs no listener
// and runs either way; neither desk board carries it.
//
// THE CH32V303RC'S AND VC'S PAIR LETTERS TAKE THE FAR CHIP. Their crossed
// pair is retired - UART4's default pads PC10/PC11 are the desk pair's SPI
// link (vx03_pair.hpp) - so where UART4 is the pair's link letters n, q and
// r are the SECOND KEY `y`'s (util/testbench.hpp's TwoKeyTestBench): UART4
// on its second column (PB0/PB1, DMA2's channels 5 and 3), the far chip's
// usart_peer the sender and the echo, driven in band over the link
// (vx03_link_dut.hpp); they fail loudly with nobody there, so `z` never
// runs them. Where the part has a fourth port and keeps the crossed pair
// (the CH32V203C8) letters q and r stay in `z`, on the core board's
// crossed pair. A part with neither - the CH32V303CB, whose link is USART3
// on DMA1's channels 2 and 3 beside the copy engine's 3 - registers none
// of the three.
//
// THE PADS. PA2 and PA3 in letters g and p (PA3 the banged line when the
// strap is absent); on the CH32V203, in letters q and r, PA3 and the
// fourth port's TX pad PB0, which the board's crossed pair wires to it
// (each letter looks for that wire first, and says so when it is not
// there); where UART4 is the pair's link, in letters n, q and r, PB0 and
// PB1 - that link. NEVER
// TOUCHED: PA9/PA10 (the console), PA13/PA14 (the debug port), PA11/PA12
// (the USB pads), PC14/PC15 and PD0/PD1 (the crystals), PA0 (the CH32V203
// board's KEY) - and PB2, the CH32V203 board's LED, left undriven, as in
// every suite of this target.
//
// What is exercised, letter by letter:
//   g  THE USART ENGINES: a run drained from the ring by the transmit
//      engine, which needs no listener; the RECEIVE RING standing from
//      init() - the channel circular over the whole ring's storage - and
//      started again by the consumer's next look after its channel
//      stopped; then, with the board bare, bursts banged into the receive
//      pad while the core reads nothing: one into an empty ring and its
//      edge from the vectors, one across the
//      storage's end delivered whole, a lap the consumer did not keep up
//      with counted once and skipped, and a run written over while it was
//      held refused at its release, and a REAL transfer error under the
//      ring at the SRAM region's top, whose write raises one
//      - or, WITH THE JUMPER, the transmit
//      run coming back through the receive ring; and either way the
//      transmit engine's run timed on the core's counter against the wire
//      at the rate asked, which a loop dividing one bus at both ends
//      cannot tell
//   p  THE RECEIVE RING OVER A CHANNEL THAT NEVER TAKES THE FRAME: TIM3's
//      channel 1 request held on channel 6 by a gate closed without its
//      reset, USART2's ring armed over it, frames into the receiver (banged,
//      or round the jumper): the vector's wait for a frame giving the
//      channel up within its bound - one DMA fault, no storm, the thread
//      running - and, the request let go by TIM3's reset line, the ring
//      started again by the consumer's next look and the next burst whole,
//      the port's transmit engine timed against the wire after it
//   q  THE PRIORITY RULE (docs/design/dma.md): the Uart's levels as armed -
//      the ring at very_high, its transmit engine at high, and the ring
//      UartOptions::rx_priority moves; both ends' rates timed against the
//      wire at 1 and 4.5 Mbaud - then the fourth port pouring the
//      image's pattern into USART2's ring of 512 at 1 Mbaud and at 4.5
//      Mbaud while a memory-to-memory copy of 2 KB runs back to back on
//      DMA1's channel 3, a lower number than the ring's 6, at the copy
//      engine's default level: every byte in order, no overrun, no channel
//      given up; the copy's rate against its rate alone; and the CONTROL,
//      the same load against a ring told rx_priority = low, which overruns.
//      WHERE UART4 IS THE LINK (the second key): the far chip's counting
//      blast into UART4's ring of 512 on DMA2's channel 3, the copy on
//      DMA2's channel 1, the far chip stating its rate and UART4's transmit
//      engine timed into its sink
//   r  AN OVERRUN AT A MESSAGE'S TAIL, both ends timed against the wire
//      at 1 Mbaud first: a copy at the ring's own level on a
//      lower channel starving the ring under a message's last frames, the
//      vector held off until the channel has taken the frame DATAR held -
//      ORE up with RXNE down and no frame after it: the vector entered a
//      handful of times, no channel given up, the overrun and the gap each
//      counted once, the bytes before it delivered; the next message whole;
//      and a frame the CPU reads from under a starved channel - what the
//      overrun's clear would do to a frame landing between its two loads -
//      followed to see what the channel does with the request it raised.
//      WHERE UART4 IS THE LINK (the second key): the far chip's messages
//      into UART4's ring on DMA2, the copy on DMA2's channel 1 started when
//      28 have landed, and the staged frame's copy started on its start
//      bit's fall, seen on the pad
//   s  THE SAME OWNER OVER THE REQUEST ITS RELEASE LEFT STANDING
//      (docs/design/dma.md's release contract): USART2's receive request
//      raised over a banged frame with the ring's channel stopped, the
//      port released and initialized again, a burst after it landing whole
//      with nothing before it; and TIM1's update request raised with
//      channel 5 disabled, the timer released, its own paced block moving
//      nothing before the pace
// and where UART4 is the link, the second controller:
//   n  UART4'S ENGINES ON DMA2 (the second key): a run out of the transmit
//      engine on channel 5 round the far chip's echo and back through the
//      receive engine on channel 3, byte for byte - table 11-3's slots
//      answering real requests - and the transmit engine timed against the
//      wire into the far chip's sink, the port released at the end, since a
//      receiver left running keeps its request on its channel
//
// build: boards = v203c6,v203c8,v303cb,v303vc
// build: groups = gp,qrs
// build: monitor_speed = 115200

#include <stdint.h>
#include <span>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/dma.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/ticker.hpp"
#include "ch32vx03/tim.hpp"
#include "ch32vx03/usart.hpp"
#include "util/print.hpp"
#include "util/test_wait.hpp"
#include "util/testbench.hpp"

// The DUT's half of the desk pair's serial link: the CH32V303's letters n,
// q and r take the far chip through it.
#include "vx03_link_dut.hpp"

namespace {

using namespace brio;

using P = Ch32vx03Platform<>;
using Serial = Uart<1, P>;
constexpr Serial serial;

using SysClock = Clock<ClockSource::pll, 144'000'000>;
constexpr SysClock clock;

TwoKeyTestBench<Serial, 'y'> bench;

/// The transport letter g drives: USART2 with both engines, each on the
/// channel table 11-5 gives that direction (7 out, 6 in). The console
/// above has none, which is what makes an empty slot cost nothing.
using Loop = Uart<2, P, 128, 128, UartFormat{},
                  DmaTxEngine<1, DmaRequestOf<DmaRequest::usart2_tx>::channel>,
                  DmaRxEngine<1, DmaRequestOf<DmaRequest::usart2_rx>::channel>>;
using LoopRx = DmaRxEngine<1, DmaRequestOf<DmaRequest::usart2_rx>::channel>;
/// The receive edges USART2's vector and its receive channel's reported:
/// what a program would post RxActivity on.
volatile uint32_t loop_edges = 0;
/// The channel under the receive engine, asked directly by letter g.
using LoopRxChannel = DmaChannel<1, DmaRequestOf<DmaRequest::usart2_rx>::channel>;
/// USART2's receive pad, the bit-banged line of letter g.
using RxPad = Pin<'A', 3>;

using Holder = Tim<3>;   ///< letter p's holder: its first compare's request, on channel 6

constexpr uint32_t tim_hz = Holder::clock_hz(clock);
constexpr uint32_t ticks_per_us = tim_hz / 1'000'000u;
static_assert(tim_hz == SysClock::hz);

// ---------------------------------------------------------------------------
// The buffers
// ---------------------------------------------------------------------------

/// FOUR KILOBYTES OF SOURCE IN THE IMAGE. 11.1 lists flash among the
/// memories a channel may read, so the pattern lives there: the chip
/// carries one copy, the RAM carries none, and a verdict compares what
/// the controller wrote against what the core reads at the same place.
struct Pattern {
    uint32_t word[1024];
};

constexpr Pattern make_pattern() {
    Pattern p{};
    uint32_t x = 0x1234'5678UL;
    for (uint32_t i = 0; i < 1024u; ++i) {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        p.word[i] = x;
    }
    return p;
}

/// Aligned to its own size where DMA1 is held to the 64 KB rule (the
/// CH32V303's): a 4 KB block on a 4 KB boundary cannot straddle a 64 KB
/// page, wherever the image grows to put it.
alignas(DmaChannel<1, 1>::bounded_to_64k ? 4096 : 4) constexpr Pattern source = make_pattern();
constexpr uint16_t source_bytes = 4096;

alignas(4) uint8_t destination[source_bytes];

const uint8_t* source_first_bytes() { return reinterpret_cast<const uint8_t*>(&source.word[0]); }

// ---------------------------------------------------------------------------
// The ruler
// ---------------------------------------------------------------------------

/**
 * THE RULER OF THIS SUITE: the composed cycle count (util/test_wait.hpp's
 * ruler) - the kernel tick count and the STK counter's position, so a span
 * of any length reads whole, polled or not, while the tick handler runs:
 * nothing in this suite masks interrupts or pauses the tick.
 */
struct Ruler {
    static uint32_t now() { return Ticker::cycles(); }
    static constexpr uint32_t hz() { return SysClock::hz; }
};

/// A stopwatch's reading in whole microseconds.
uint32_t us_of(const Stopwatch<Ruler>& w) { return w.elapsed() / ticks_per_us; }

/**
 * THE MASKED STOPWATCH, for the spans letter r times INSIDE a critical
 * section, and for those alone: the composed cycle count above is right
 * only while the tick handler runs, and under the mask it does not - past
 * one tick period the count folds back, and a bound of a few milliseconds
 * read on it would never come. This one reads the STK counter alone and
 * folds one period in at each wrap it sees, which holds however long the
 * mask lasts so long as it is polled at least once a period (every loop
 * that polls it turns in well under a microsecond).
 */
class MaskedStopwatch {
public:
    MaskedStopwatch() : last_(stk()->CNTL) {}

    /// Microseconds since construction; one call a turn of the poll.
    uint32_t us() {
        const uint32_t period = stk()->CMPLR + 1u;
        const uint32_t now = stk()->CNTL;
        acc_ += (now >= last_) ? (now - last_) : (now + period - last_);
        last_ = now;
        return acc_ / ticks_per_us;
    }

private:
    uint32_t last_;
    uint32_t acc_ = 0;
};

/**
 * THE RATE ON THE WIRE, which a loop alone cannot tell: a transmitter and
 * the receiver it reaches here both divide PB1 by one arithmetic, so a
 * wrong bus rate is a slower or a faster stream and nothing else. The STK
 * counts HCLK instead: `frames` 8N1 frames of the image's pattern queued
 * into a transport `T` with write_bulk() as fast as its ring takes them,
 * timed from the first queue to tx_idle() - the last stop bit's end -
 * against their time at the rate ASKED; the answer in thousandths. What
 * the transport receives meanwhile is read and thrown away, and its
 * counters are cleared after.
 */
template <typename T>
uint32_t wire_permille(uint32_t baud, uint32_t frames) {
    const uint8_t* src = source_first_bytes();
    const uint64_t wire = 10ULL * frames * SysClock::hz / baud;
    const uint64_t budget = 2u * wire + SysClock::hz / 100u;
    uint8_t b = 0;
    uint32_t queued = 0;
    Stopwatch<Ruler> w;
    w.start();
    while (queued < frames && w.elapsed() < budget) {
        const uint32_t want = frames - queued;
        queued += T::write_bulk(std::span<const uint8_t>(
            src + (queued & (source_bytes - 1u)),
            want < source_bytes - (queued & (source_bytes - 1u))
                ? want
                : source_bytes - (queued & (source_bytes - 1u))));
        while (T::read_byte(b)) {
        }
    }
    while (!T::tx_idle() && w.elapsed() < budget) {
        while (T::read_byte(b)) {
        }
    }
    const uint32_t took = w.elapsed();
    spin_us<Ruler>(64u * 10u * 1'000'000u / baud + 1000u);
    while (T::read_byte(b)) {
    }
    T::clear_errors();
    return wire == 0u ? 0u : static_cast<uint32_t>(1000ULL * took / wire);
}

/**
 * The verdict on a reading at `baud`, on a port that counts PCLK1 (USART2,
 * the fourth port, the link's).
 *
 * THE FAST SIDE IS THE DIVISOR'S, derived: the STK counts HCLK and the
 * port divides PCLK1, both out of one oscillator, so the oscillator's error
 * cancels and a bit lasts BRR periods of PCLK1 whatever the crystal or the
 * HSI does. The stopwatch opens before the first queue and closes after
 * the flag that ends the run, so the reading is never under 1000 x BRR x
 * baud / PCLK1, rounded down - 1000 at every rate this suite asks, each
 * dividing PCLK1 whole (625, 72 and 16 at 115200, 1 and 4.5 Mbaud).
 *
 * THE SLOW SIDE IS A THRESHOLD, not a model of the path. What it is there
 * to catch is a wrong bus divisor - a bus prescaler, or BRR computed from
 * the other bus - and each of those is a factor of two: a thousand per
 * mille. The path adds the frame start's latency (up to a bit, a tenth of
 * a per mille of the shortest run timed here, 256 frames) and a poll turn,
 * and a transmit engine leaves gaps between its blocks, which is not this
 * verdict's subject. Thirty per mille sits between the path's own figure
 * and the next wrong rate.
 */
bool wire_ok(uint32_t permille, uint32_t baud) {
    constexpr uint32_t pclk1 = SysClock::pclk1_hz;
    const uint32_t floor_pm = static_cast<uint32_t>(1000ULL * usart_divisor(pclk1, baud) * baud / pclk1);
    return permille >= floor_pm && permille <= 1030u;
}

// ---------------------------------------------------------------------------
// The handlers' counters, and which face channels 6 and 7 wear
// ---------------------------------------------------------------------------

volatile uint16_t channel_calls[8] = {};
volatile uint32_t channel_mask[8] = {};
/// Channels 6 and 7 are USART2's two, so their vectors serve the
/// transport in letter g and the bare channel everywhere else.
volatile bool uart_mode = false;

void note(uint8_t index, uint32_t flags) {
    if (flags != 0u) {
        channel_calls[index] = static_cast<uint16_t>(channel_calls[index] + 1u);
        channel_mask[index] = channel_mask[index] | flags;
    }
}

// ---------------------------------------------------------------------------
// Transfers, named once
// ---------------------------------------------------------------------------

DmaTransfer copy(const volatile void* from, volatile void* to, uint16_t items, DmaWidth width,
                 bool increment_source = true, DmaPriority priority = DmaPriority::low) {
    return DmaTransfer{
        .peripheral = const_cast<volatile void*>(from),
        .memory = to,
        .count = items,
        .config = {.direction = DmaDirection::peripheral_to_memory,
                   .circular = false,
                   .memory_to_memory = true,
                   .peripheral_increment = increment_source,
                   .memory_increment = true,
                   .peripheral_width = width,
                   .memory_width = width,
                   .priority = priority},
    };
}

/// Every channel, every line and the timer back to reset, at the top of
/// each letter, so no letter inherits another's state.
void all_off() {
    Dma<1>::open();
    Dma<1>::stop_all();
    for (uint8_t ch = 1; ch <= Dma<1>::channel_count; ++ch) {
        Pfic::disable(dma_channel_irq(1, ch));
        Pfic::clear_pending(dma_channel_irq(1, ch));
    }
    Holder::release();
    uart_mode = false;
    for (uint8_t i = 0; i < 8u; ++i) {
        channel_calls[i] = 0;
        channel_mask[i] = 0;
    }
}

// ---------------------------------------------------------------------------
// The transfer error letter g stages under its ring
// ---------------------------------------------------------------------------

/// An address that refuses a channel's write at every width: the SRAM
/// region's top, far past either part's SRAM (test_vx03_dma's letter d
/// measures it, read and written).
constexpr uint32_t refused_address = 0x2FFF'F000UL;

/// Whether a four-word write to `target` from RAM ends in a transfer error
/// on DMA1's channel 3 - the copy engine's channel, programmed by hand
/// under the same copy tag - its line silent.
bool word_write_faults(uint32_t target) {
    using Ch = DmaChannel<1, 3>;
    static_assert(dma_claim<DmaSlotId<1, 3>, DmaCopyOwner<1, 3>>);
    Ch::stop();
    (void)Ch::load(copy(destination, reinterpret_cast<volatile void*>(target), 4, DmaWidth::word));
    // Four words end in well under a microsecond, an error sooner; 1 ms
    // allowed, and a channel that never ends reads as no fault.
    uint32_t f = 0;
    (void)wait_until<Ruler>(
        [&f] {
            f = Ch::flags();
            return (f & (DmaFlag::error | DmaFlag::complete)) != 0u;
        },
        1'000);
    Ch::stop();
    return (f & DmaFlag::error) != 0u;
}

// ---------------------------------------------------------------------------
// The jumper
// ---------------------------------------------------------------------------

bool jumper = false;
bool jumper_known = false;

/// Is PA2 strapped to PA3? Drive one as a plain output and read the
/// other, both ways round: a strap shows both levels, an open pad holds
/// whatever its pull says.
bool measure_jumper() {
    using Src = Pin<'A', 2>;
    using Dst = Pin<'A', 3>;
    Src::output(false);
    Dst::input(PinPull::up);
    spin_us<Ruler>(20);
    const bool low_seen = !Dst::read();
    Src::set();
    spin_us<Ruler>(20);
    const bool high_seen = Dst::read();
    Dst::input(PinPull::down);
    Src::clear();
    spin_us<Ruler>(20);
    const bool low_again = !Dst::read();
    Src::release();
    Dst::release();
    return low_seen && high_seen && low_again;
}

void need_jumper() {
    if (!jumper_known) {
        jumper = measure_jumper();
        jumper_known = true;
    }
}

// ===========================================================================
// g - the USART's two engines: the ring on a banged line, and the jumper
// ===========================================================================

/// THE RECEIVE PAD IS A BIT-BANGED TRANSMITTER (test_vx03_serial's first
/// fact): a pad handed to the USART as RX is an input whose level follows
/// its own pull, which the output data register moves. `n` bytes of
/// `bytes` go out as 8N1 frames back to back, every edge at an ABSOLUTE
/// offset from the run's start so a handler's delay never accumulates, and
/// the line then idles for two bit times more: the last frame is received
/// and stored by the time this returns. THE MARGIN, derived: an edge is
/// written up to one turn of the stopwatch's poll after its offset - one
/// composed cycle count read and its compare, about twenty instructions -
/// plus any handler served in that turn (the tick's, a console character's,
/// the ring's own vector: the bang masks nothing). The receiver samples a
/// bit no earlier than 37 % of the cell after its start edge (the census of
/// test_vx03_serial's letter 4, on both parts; chapter 18 does not place
/// the samples), so an edge up to 462 of the 1250 cycles late is never
/// sampled on its wrong side. A later one - a handler past some 400 cycles
/// in the turn - misreads a bit, which the letters see as a wrong byte or a
/// counted error: a failure, never a pass.
constexpr uint32_t bang_baud = 115200;

void bang(const uint8_t* bytes, uint16_t n) {
    constexpr uint32_t bit = SysClock::hz / bang_baud;
    Stopwatch<Ruler> w;
    w.start();
    uint32_t edge = 0;
    for (uint16_t i = 0; i < n; ++i) {
        const uint16_t frame = static_cast<uint16_t>((bytes[i] << 1) | 0x200u);   // start, 8 data, stop
        for (uint8_t b = 0; b < 10u; ++b) {
            while (w.elapsed() < edge) {
            }
            if (((frame >> b) & 1u) != 0u) {
                RxPad::set();
            } else {
                RxPad::clear();
            }
            edge += bit;
        }
    }
    edge += 2u * bit;
    while (w.elapsed() < edge) {
    }
}

/// Everything the transport holds, read away: how many bytes, and whether
/// they were `want`'s first ones in order.
uint16_t read_all(const uint8_t* want, uint16_t most, bool& in_order) {
    uint16_t got = 0;
    in_order = true;
    uint8_t b = 0;
    while (Loop::read_byte(b)) {
        if (got < most && b != want[got]) {
            in_order = false;
        }
        ++got;
    }
    if (got > most) {
        in_order = false;
    }
    return got;
}

/// The ring with NO WIRE: bursts banged into PA3 while the core does
/// nothing else, so no read runs between their bytes.
void ring_on_a_banged_line() {
    const uint8_t* src = source_first_bytes();
    bool junk_in_order = true;
    (void)read_all(src, 0, junk_in_order);   // whatever the pad framed before the pull settled
    Loop::clear_errors();

    // A burst into an empty ring, then the edge from the vectors - USART2's
    // idle line, a burst's first frame, the lap's marks: reported once,
    // and harvest() asked after it answers false, the gate being one;
    // nor does harvest() report the ring the consumer has drained.
    const uint32_t e0 = loop_edges;
    bang(src, 100);
    const bool edge = loop_edges != e0;
    const uint32_t e1 = loop_edges;
    const bool edge_again = Loop::harvest() || loop_edges != e1;
    bool first_in_order = false;
    const uint16_t first = read_all(src, 100, first_in_order);
    const bool edge_drained = Loop::harvest();
    print(serial, "  100 bytes banged at ", bang_baud, " baud: ", first,
          first_in_order ? " read back in order" : " read back WRONG", "; edges ", edge ? 1 : 0,
          edge_again ? 1 : 0, edge_drained ? 1 : 0, crlf);
    bench.verdict("a burst banged into the receive pad lands in the ring with no step of the "
                  "core's, every byte in order - and the vectors report the edge once",
                  first == 100u && first_in_order && edge && !edge_again && !edge_drained);

    // THE BURST ACROSS THE STORAGE'S END, with no read between its bytes:
    // the ring is 128 long and 100 of it are behind the consumer, so these
    // sixty wrap. A run engine stopped at its run's end and lost what came
    // before the next re-arm. The consumer drained: the vectors tell it.
    const uint32_t e2 = loop_edges;
    bang(src + 100, 60);
    const bool edge_b = loop_edges != e2;
    bool second_in_order = false;
    const uint16_t second = read_all(src + 100, 60, second_in_order);
    print(serial, "  60 more across the storage's end: ", second,
          second_in_order ? " in order" : " NOT in order", ", the channel's count ",
          LoopRxChannel::remaining(), ", overruns ", Loop::rx_overruns(), "/",
          Loop::hw_overruns(), crlf);
    bench.verdict("a burst across the end of the ring's storage arrives whole and in order with "
                  "no read between its bytes - nothing is lost between runs, there being "
                  "none - and no overrun of either kind is counted",
                  edge_b && second == 60u && second_in_order && Loop::rx_overruns() == 0u &&
                      Loop::hw_overruns() == 0u);

    // A LAP THE CONSUMER DID NOT KEEP UP WITH: two hundred bytes into a
    // ring of 128 with nobody reading. Counted once and skipped; the next
    // burst is read whole.
    bang(src + 160, 200);
    bool skipped_in_order = false;
    const uint16_t after_lap = read_all(src + 160, 0, skipped_in_order);
    const uint16_t lap_overruns = Loop::rx_overruns();
    bang(src + 360, 10);
    bool resumed_in_order = false;
    const uint16_t resumed = read_all(src + 360, 10, resumed_in_order);
    print(serial, "  200 bytes unread into a ring of 128: ", after_lap,
          " delivered, overruns ", lap_overruns, "; ten more after it: ", resumed,
          resumed_in_order ? " in order" : " NOT in order", crlf);
    bench.verdict("a lap the consumer did not keep up with is counted ONCE and skipped - no "
                  "byte of the overwritten ring delivered - and the stream resumes with the "
                  "next burst, whole",
                  after_lap == 0u && lap_overruns == 1u && resumed == 10u && resumed_in_order);

    // A RUN WRITTEN OVER WHILE IT IS HELD: consume() is the one witness.
    bang(src + 370, 20);
    const std::span<const uint8_t> held = Loop::read_span();
    const uint16_t held_size = static_cast<uint16_t>(held.size());
    bang(src + 390, 140);
    const bool intact = Loop::consume(held_size);
    print(serial, "  a run of ", held_size, " held while 140 more landed: consume() says ",
          intact ? "intact" : "torn", ", overruns ", Loop::rx_overruns(), crlf);
    bench.verdict("a run the channel wrote over while it was held is refused at its release - "
                  "consume() answers false and the overrun is counted",
                  held_size != 0u && !intact && Loop::rx_overruns() == 2u);

    // A REAL TRANSFER ERROR UNDER THE RING, at the address whose write
    // raises one when the probe finds it does. The running ring's channel is stopped BY
    // REGISTER, its memory address pointed there and EN set again - the ledger and the transport left as they stand - and one
    // frame banged into the receive pad: the channel's write of it is
    // refused, the hardware drops EN, and the transport's vector abandons
    // the ring and counts the fault. The consumer's next look starts the
    // ring again from its RAM storage's first byte, and a burst after it
    // lands whole.
    const uint32_t te_target = word_write_faults(refused_address) ? refused_address : 0u;
    if (te_target == 0u) {
        bench.skip("the SRAM region's top raised no transfer error on a write: the ring's real error "
                   "is not staged");
    } else {
        bool junk = true;
        (void)read_all(src, 0, junk);
        Loop::clear_errors();
        const uint32_t into = te_target;
        const uint32_t kept = source.word[128];
        const uint16_t faults0 = Loop::dma_faults();
        const uint32_t rx_faults0 = LoopRx::faults();
        auto& r = LoopRxChannel::regs();
        {
            P::CriticalSection cs;
            r.CFGR = r.CFGR & ~dma_cfgr_en;   // EN down by hand: the ledger keeps its count
            r.MADDR = into;
            r.CFGR = r.CFGR | dma_cfgr_en;
        }
        bang(src + 600, 1);
        spin_us<Ruler>(200);
        const bool dropped = LoopRx::idle();
        const uint16_t faults1 = Loop::dma_faults();
        const uint32_t rx_faults1 = LoopRx::faults();
        const uint8_t masters_dropped = BusActivity::active();
        (void)Loop::harvest();   // the consumer's next look
        const bool running = !LoopRx::idle() && LoopRxChannel::remaining() == 128u &&
                             r.MADDR != into && BusActivity::active() == 1u;
        bang(src + 610, 24);
        bool again_in_order = false;
        const uint16_t again = read_all(src + 610, 24, again_in_order);
        print(serial, "  the ring's channel pointed at ", hex(into), ", a frame banged: the channel ",
              dropped ? "stopped" : "STILL RUNNING", ", the transport's DMA faults ", faults0, " -> ", faults1,
              ", the engine's ", rx_faults0, " -> ", rx_faults1, ", bus masters ", masters_dropped,
              "; the next look: ", running ? "the ring running again from its first byte" : "NOT RESTARTED",
              "; 24 more banged: ", again, again_in_order ? " in order" : " NOT in order", "; the flash ",
              source.word[128] == kept ? "untouched" : "CHANGED", crlf);
        bench.verdict("A REAL TRANSFER ERROR STOPS THE RING: the hardware drops EN on the refused "
                      "write and the transport's vector counts the fault once",
                      dropped && faults1 == faults0 + 1u && rx_faults1 == rx_faults0 + 1u &&
                          source.word[128] == kept);
        bench.verdict("... and the consumer's next look starts the ring again from its storage's first "
                      "byte, one bus master counted, the next burst whole and in order",
                      running && again == 24u && again_in_order);
    }
}

/// Letter g's last step, either way round: the transmit engine's run at
/// the rate asked - the jumper's loop, where there is one, divides PB1 at
/// both ends, and the core's counter does not.
void tg_rate() {
    const uint32_t pm = wire_permille<Loop>(115200, 256);
    print(serial, "  256 frames through the transmit engine in ", pm,
          " thousandths of their wire time at 115200", crlf);
    bench.verdict("the transmit engine runs the line at the rate asked: 256 frames in their wire "
                  "time at 115200 on the core's counter, no faster than the divisor and at most 3 % slower",
                  wire_ok(pm, 115200));
}

void tg_engines() {
    all_off();
    need_jumper();
    print(serial, "  the jumper PA2-PA3 is ", jumper ? "in place" : "ABSENT", crlf);

    uart_mode = true;
    const bool opened = Loop::init(clock, 115200);
    // The receive pad is pulled UP after init: an idle serial line is
    // high, so with no strap the receiver sees no frame instead of
    // whatever a floating pad invents. A transmitter driving the pad
    // overrides it.
    RxPad::input(PinPull::up);
    LoopRx::clear_faults();

    // THE TRANSMIT ENGINE NEEDS NO LISTENER. TXE raises its request
    // whether or not anything is on the other end of the pad, so the
    // half of this letter that drains a ring by blocks is measured
    // with the board bare.
    static const uint8_t message[] = "the channel is the request";
    constexpr uint16_t length = sizeof(message) - 1u;
    (void)Loop::write_bulk(std::span<const uint8_t>(message, length));
    Stopwatch<Ruler> w;
    w.start();
    while (!Loop::tx_idle() && us_of(w) < 20'000UL) {
    }
    const uint32_t drained_us = us_of(w);
    const bool drained = Loop::tx_idle();
    // 26 bytes of ten bits at 115200 baud are some 2260 us, and the
    // run left the ring in ONE block: the engine's own completion is
    // what released it.
    print(serial, "  ", length, " bytes drained by the transmit engine in ", drained_us,
          " us, faults ", Loop::dma_faults(), crlf);
    bench.verdict("the transmit engine drains the ring by whole blocks with no listener at "
                  "all - the ring is empty and the shift register idle, no fault counted",
                  opened && drained && Loop::dma_faults() == 0u);

    // THE RING STANDS FROM init(): the receive engine runs circular over
    // the whole receive ring's storage, its count plus what has arrived
    // equal to the storage's length, and a running channel is a bus master
    // counted - the one this program holds while the transmit engine rests.
    const DmaChannelConfig standing = LoopRxChannel::configuration();
    const bool running = LoopRxChannel::enabled();
    const uint16_t count = LoopRxChannel::remaining();
    const uint32_t pending = Loop::rx_pending();
    const uint8_t masters = BusActivity::active();
    print(serial, "  the receive channel: ", running ? "running" : "STOPPED", ", ",
          standing.circular ? "circular" : "one-shot", ", count ", count, " + ", pending,
          " arrived of the ring's 128, bus masters counted ", masters, crlf);
    bench.verdict("the receive ring stands from init(): the channel running in circular mode "
                  "over the whole 128-byte storage, its count and what has arrived adding up "
                  "to that length, and one bus master counted for it",
                  opened && running && standing.circular && count + pending == 128u &&
                      masters == 1u);

    // A RING WHOSE CHANNEL STOPPED is started again by the consumer's next
    // look - harvest() here, any read verb alike.
    // A transfer error is the one way a ring stops on its own (11.2.1: the
    // silicon drops EN), and abandon() - what the transport's handler calls
    // on one - stands for it here.
    const bool abandoned = LoopRx::abandon();
    const bool idle_after = LoopRx::idle();
    (void)Loop::harvest();
    const bool restarted = !LoopRx::idle() && LoopRxChannel::remaining() == 128u;
    print(serial, "  abandon(): the channel ", idle_after ? "stopped" : "STILL RUNNING",
          "; the next harvest(): ", restarted ? "the ring running again from its first byte"
                                              : "NOT restarted",
          ", faults ", LoopRx::faults(), crlf);
    bench.verdict("a ring whose channel stopped is started again by the next harvest() from "
                  "its storage's first byte, and the fault is counted",
                  abandoned && idle_after && restarted && LoopRx::faults() == 1u);

    if (!jumper) {
        ring_on_a_banged_line();
        bench.skip("the round trip: without the strap between USART2's two pads a transmit engine has "
                   "no receive engine to reach");
        tg_rate();
        Loop::release();
        uart_mode = false;
        all_off();
        return;
    }

    // With the strap, the same run comes back: the bytes land where the
    // consumer reads them, nothing published and nothing polled.
    (void)Loop::write_bulk(std::span<const uint8_t>(message, length));
    uint16_t got = 0;
    uint8_t seen[64] = {};
    Stopwatch<Ruler> r;
    r.start();
    while (got < length && us_of(r) < 50'000UL) {
        uint8_t b = 0;
        while (got < length && Loop::read_byte(b)) {
            seen[got++] = b;
        }
    }
    bool same = got == length;
    for (uint16_t i = 0; i < got && same; ++i) {
        same = seen[i] == message[i];
    }
    print(serial, "  ", got, " of ", length, " bytes round the jumper, faults ",
          Loop::dma_faults(), ", overruns ", Loop::rx_overruns(), crlf);
    bench.verdict("a run poured out by the transmit engine comes back through the receive "
                  "ring one byte for byte",
                  same && Loop::dma_faults() == 0u);
    tg_rate();

    Loop::release();
    uart_mode = false;
    all_off();
}

// ===========================================================================
// p - the receive ring over a channel that never takes the frame
// ===========================================================================

/// USART2's vector counted while letter p watches it, and silenced past a
/// budget: a vector that storms reports the fact instead of starving the
/// suite.
volatile bool ring_watch = false;
volatile uint32_t ring_entries = 0;
volatile bool ring_stormed = false;
constexpr uint32_t ring_entry_budget = 100'000;
/// The budget the binding silences the vector at; letter q's cells lift it.
volatile uint32_t ring_budget = ring_entry_budget;

/// A REQUEST HELD ON CHANNEL 6 BY A PERIPHERAL WHOSE CLOCK IS GATED - the
/// state a release that does not pulse its block's reset leaves
/// (docs/ch32vx03/dma.md, "A released requester") - made at the register
/// level, on purpose: TIM3's channel 1 compare with CC1DE (table 11-5's
/// TIM3_CH1 row, USART2's receive channel), one match, the counter
/// stopped and the gate closed. No pad. TIM3's reset line, which the
/// letter pulses after its measurement, lets it go.
void hold_channel6() {
    static_assert(DmaRequestOf<DmaRequest::tim3_ch1>::channel ==
                      DmaRequestOf<DmaRequest::usart2_rx>::channel,
                  "table 11-5: TIM3's channel 1 request and USART2's receive request share a "
                  "channel");
    Holder::init();
    (void)Holder::configure(TimConfig{.prescaler = 0, .period = 0xFFFF});
    (void)Holder::set_compare(0, 16);
    Holder::interrupts(Holder::compare_dma(0), true);
    Holder::enable(true);
    spin_us<Ruler>(20);
    Holder::enable(false);
    Holder::bus_clock(false);
}

void tp_dead_ring() {
    all_off();
    need_jumper();
    print(serial, "  the jumper PA2-PA3 is ", jumper ? "in place" : "ABSENT", crlf);

    // The receive ring's channel enabled over the held request: it moves
    // ONE item on the enable - USART2's empty data register - and waits
    // for that request to fall, which a gated timer never does.
    hold_channel6();
    uart_mode = true;
    const bool opened = Loop::init(clock, 115200);
    RxPad::input(PinPull::up);
    LoopRx::clear_faults();
    const uint16_t armed_count = LoopRxChannel::remaining();

    // Frames into the receiver: the first stands in the data register
    // with nothing to take it, the next overrun it; the error and the line
    // gone idle turn the vector to its wait for a frame, RXNE standing
    // under RXNEIE.
    const uint8_t* src = source_first_bytes();
    ring_entries = 0;
    ring_stormed = false;
    ring_watch = true;
    if (jumper) {
        (void)Loop::write_bulk(std::span<const uint8_t>(src, 4));
        Stopwatch<Ruler> w;
        w.start();
        while (!Loop::tx_idle() && us_of(w) < 5'000UL) {
        }
    } else {
        bang(src, 4);
    }
    spin_us<Ruler>(2'000);
    ring_watch = false;
    const uint32_t entries = ring_entries;
    const bool stormed = ring_stormed;
    const uint16_t faults = Loop::dma_faults();
    const bool given_up = (Loop::Resource::regs().CTLR1 & usart_rxneie) == 0u;
    print(serial, "  the ring armed over channel 6 held by TIM3 (count ", armed_count,
          " of 128 after the enable), four frames sent: USART2's vector ", entries, " entries",
          stormed ? " - STORMED, silenced by the suite" : "", ", dma_faults ", faults,
          ", the wait for a frame ", given_up ? "given up (RXNEIE down)" : "STILL ARMED", crlf);
    // THE COUNT, derived from the vector's two states (ch32vx03/usart.hpp):
    // the ring armed waits for the end with RXNEIE down, so the first frame
    // standing in DATAR raises nothing; the error or the idle line that
    // follows raises ONE entry, which turns the vector to its wait for a
    // frame with RXNEIE up; RXNE standing then re-enters it back to back,
    // each entry finding the channel's count unmoved, and the 255th of
    // those (rx_stall_entries) gives the channel up, RXNEIE down and
    // nothing left armed - 1 + 255 entries exactly.
    bench.verdict("A RECEIVE CHANNEL THAT NEVER TAKES THE FRAME leaves the thread running: the "
                  "vector's wait for a frame gives the channel up at its bound of 255 unmoved "
                  "entries, 256 in all - RXNEIE down, one DMA fault counted - and no storm",
                  opened && !stormed && entries == 1u + 255u && faults == 1u && given_up);

    // The held request let go by TIM3's reset line; the consumer's next
    // look starts the ring again and arms the wait for the end, and the
    // next burst arrives whole.
    Holder::reset();
    bool junk_in_order = true;
    (void)read_all(src, 0, junk_in_order);   // the look that restarts it, and what stood
    const uint16_t ctlr1 = Loop::Resource::regs().CTLR1;
    const bool restarted = !LoopRx::idle() && (ctlr1 & usart_idleie) != 0u &&
                           (ctlr1 & usart_rxneie) == 0u;
    (void)read_all(src, 0, junk_in_order);
    if (jumper) {
        (void)Loop::write_bulk(std::span<const uint8_t>(src + 8, 10));
        Stopwatch<Ruler> w;
        w.start();
        while (!Loop::tx_idle() && us_of(w) < 5'000UL) {
        }
        spin_us<Ruler>(500);
    } else {
        bang(src + 8, 10);
    }
    bool in_order = false;
    const uint16_t got = read_all(src + 8, 10, in_order);
    print(serial, "  the request let go: the ring ", restarted ? "running again, the wait for the end armed" : "NOT RESTARTED",
          ", ten frames after it: ", got, in_order ? " in order" : " NOT in order", crlf);
    bench.verdict("... and with the held request let go by TIM3's reset line, the consumer's "
                  "next look starts the ring again and the next burst arrives whole",
                  restarted && got == 10u && in_order);
    const uint32_t pm = wire_permille<Loop>(115200, 256);
    print(serial, "  then 256 frames through the transmit engine in ", pm,
          " thousandths of their wire time at 115200", crlf);
    bench.verdict("and the port runs at the rate asked after it: 256 frames in their wire time "
                  "at 115200, no faster than the divisor and at most 3 % slower",
                  wire_ok(pm, 115200));

    Loop::release();
    uart_mode = false;
    all_off();
}

// ===========================================================================
// s - the same owner over the request its release left standing
// ===========================================================================
//
// docs/design/dma.md's release contract on this family: release() gates the
// block and pulses no reset, so a request the block raised and no channel
// took stays frozen on the channel's OR behind the closed gate; the channel
// being that request's alone for the whole image, the one that can meet it
// is the same owner initialized again, and its init() starts from the
// block's reset, before an engine is armed.

/// TIM1's update request, the second owner measured: DMA1's channel 5 for
/// the whole image (table 11-5; table 11-2 on the CH32V303).
using HeldPacer = Tim<1>;
using HeldRow = DmaRequestOf<DmaRequest::tim1_up>;
static_assert(dma_claim<DmaSlotId<HeldRow::controller, HeldRow::channel>,
                        DmaOwner<DmaPeriph::tim, 1, DmaSignal::update>>);

/// The pad's rest around a burst: two frames at bang_baud are 174 us, so a
/// frame the receiver started on the pad's way high is over by then.
constexpr uint32_t pull_settle_us = 200;

/// The paced block's landing ground: sixteen half-word samples.
alignas(4) uint16_t work[16];
constexpr uint16_t sample_count = 16;

/// What a paced block made of its channel.
struct Paced {
    bool armed = false;
    uint16_t early = 0;   ///< items moved before the pacer ran: a held request's stray
    uint16_t left = 0;    ///< CNTR after sixteen periods and two more
    uint16_t steps = 0;   ///< consecutive samples one period apart
};

/// SIXTEEN SAMPLES OF TIM3'S COUNT (Holder, at 1 MHz) on DMA `c`'s channel `ch`,
/// paced by timer `tn` - its update (cc < 0), or its channel cc's compare
/// - once every 500 us. A request held on the channel's OR moves one item
/// at the enable: `early` is read 100 us after it, the pacer still
/// stopped. A held level its gated owner never drops stalls the channel
/// after that item: `left` is read after eighteen periods. A clean channel
/// moves nothing early, ends at zero and takes fifteen steps of 500 +- 5
/// (the pacer and TIM3 divide one timer clock: a step is exact but
/// for the sample's own bus wait).
template <uint8_t c, uint8_t ch, uint8_t tn, int8_t cc>
Paced paced_block() {
    using Ch = DmaChannel<c, ch>;
    using T = Tim<tn>;
    Paced p{};
    Holder::init();
    (void)Holder::configure(TimConfig{.prescaler = static_cast<uint16_t>(ticks_per_us - 1u), .period = 0xFFFF});
    Holder::enable(true);
    T::init();
    (void)T::configure(TimConfig{.prescaler = static_cast<uint16_t>(T::clock_hz(clock) / 1'000'000u - 1u),
                                 .period = 499});
    if constexpr (cc >= 0) {
        (void)T::output_channel(static_cast<uint8_t>(cc),
                                TimChannelConfig{.mode = TimOutputMode::frozen, .compare = 250, .preload = false,
                                                 .enable = false});
        T::interrupts(T::compare_dma(static_cast<uint8_t>(cc)), true);
    } else {
        T::interrupts(tim_ude, true);
    }
    for (uint16_t i = 0; i < sample_count; ++i) {
        work[i] = 0;
    }
    Ch::stop();
    p.armed = Ch::load(DmaTransfer{
        .peripheral = Holder::cnt_address(),
        .memory = work,
        .count = sample_count,
        .config = {.direction = DmaDirection::peripheral_to_memory,
                   .circular = false,
                   .memory_to_memory = false,
                   .peripheral_increment = false,
                   .memory_increment = true,
                   .peripheral_width = DmaWidth::half,
                   .memory_width = DmaWidth::half,
                   .priority = DmaPriority::high},
    });
    spin_us<Ruler>(100);
    p.early = static_cast<uint16_t>(sample_count - Ch::remaining());
    T::enable(true);
    spin_us<Ruler>(18u * 500u);
    p.left = Ch::remaining();
    T::enable(false);
    Ch::stop();
    for (uint16_t i = 1; i < sample_count; ++i) {
        const uint16_t step = static_cast<uint16_t>(work[i] - work[i - 1u]);
        if (step >= 495u && step <= 505u) {
            ++p.steps;
        }
    }
    T::release();
    Holder::release();
    return p;
}

/// The paced block's reading as one line, and whether it was clean.
bool paced_line(const char* what, const Paced& p) {
    const bool clean = p.armed && p.early == 0u && p.left == 0u && p.steps == sample_count - 1u;
    print(serial, "  ", what, ": ", p.early, " item(s) before the pacer, ", p.left, " left after it, ", p.steps,
          " steps of a period", clean ? " - clean" : (p.early != 0u ? " - A HELD REQUEST" : " - NOT CLEAN"), crlf);
    return clean;
}

void ts_same_owner() {
    all_off();
    need_jumper();
    if (jumper) {
        bench.skip("the jumper PA2-PA3 is in place: USART2's transmitter drives the pad this letter "
                   "bangs, so its half is not staged");
    } else {
        // USART2'S RECEIVE REQUEST LEFT STANDING: the ring's channel stopped
        // and the port's vector masked, one frame banged into the receive
        // pad - RXNE up under DMAR, the request raised and nothing taking it
        // - then release(), DMAR left set and the gate closed over it. The
        // pad is pulled high before every init() and rests past two frames
        // before every burst, so the receiver frames no edge but the bangs'.
        uart_mode = true;
        RxPad::input(PinPull::up);
        spin_us<Ruler>(pull_settle_us);
        const bool up = Loop::init(clock, bang_baud);
        RxPad::input(PinPull::up);
        spin_us<Ruler>(pull_settle_us);
        bool junk = true;
        (void)read_all(source_first_bytes(), 0, junk);
        LoopRx::stop();
        Pfic::disable(Irq::usart2);
        static const uint8_t stale[1] = {0xA5};
        bang(stale, 1);
        const uint16_t statr = Usart<2>::regs().STATR;
        const uint16_t ctlr3 = Usart<2>::regs().CTLR3;
        const bool raised = (statr & usart_rxne) != 0u && (ctlr3 & usart_dmar) != 0u;
        Loop::release();
        RxPad::input(PinPull::up);
        spin_us<Ruler>(pull_settle_us);
        // THE SAME OWNER AGAIN: init() pulses the reset first, then arms the
        // ring; a burst of sixteen banged after it is all the ring may hold,
        // nothing drained between.
        const bool again = Loop::init(clock, bang_baud);
        RxPad::input(PinPull::up);
        spin_us<Ruler>(pull_settle_us);
        const uint32_t before = Loop::rx_pending();
        const uint8_t* src = source_first_bytes() + 300;
        bang(src, 16);
        uint8_t first = 0;
        const bool any = Loop::read_span().size() != 0u;
        if (any) {
            first = Loop::read_span()[0];
        }
        bool in_order = false;
        const uint16_t got = read_all(src, 16, in_order);
        const uint16_t faults = Loop::dma_faults();
        print(serial, "  USART2's ring: a frame left under DMAR with the channel stopped (", raised ? "raised" : "NOT RAISED",
              ", STATR ", hex(statr), ", CTLR3 ", hex(ctlr3), "), the port released and initialized again: ", before,
              " byte(s) in the ring; then 16 banged, ", got, " byte(s), the first ", hex(first), " of the burst's ",
              hex(src[0]), in_order ? ", the burst's own in order" : ", NOT the burst", ", faults ", faults, crlf);
        bench.verdict("THE SAME OWNER OVER ITS OWN HELD REQUEST, USART2's receive: released with RXNE standing "
                      "under DMAR and the gate closed, initialized again - its reset first - the ring holds the "
                      "sixteen bytes banged after it and nothing before them",
                      up && raised && again && before == 0u && got == 16u && in_order && faults == 0u);
        Loop::release();
        uart_mode = false;
    }

    // TIM1'S UPDATE REQUEST LEFT STANDING: UDE set and the counter run
    // through a few updates with channel 5 disabled, then release(), UDE
    // left set and the gate closed over the request; then the same owner's
    // paced block, whose init() starts from the timer's reset.
    HeldPacer::init();
    (void)HeldPacer::configure(TimConfig{
        .prescaler = static_cast<uint16_t>(HeldPacer::clock_hz(clock) / 1'000'000u - 1u), .period = 99});
    HeldPacer::interrupts(tim_ude, true);
    HeldPacer::enable(true);
    spin_us<Ruler>(500);
    HeldPacer::release();
    const Paced p = paced_block<HeldRow::controller, HeldRow::channel, 1, -1>();
    const bool clean = paced_line("TIM1's update left standing under UDE, released, then its own paced block", p);
    bench.verdict("THE SAME OWNER OVER ITS OWN HELD REQUEST, TIM1's update: released with the request raised and "
                  "UDE set, its next paced block - init()'s reset first - moves nothing before the pace and "
                  "sixteen samples a period apart",
                  clean);
    all_off();
}

// ===========================================================================
// The second controller - the CH32V303's alone (letter n, and q and r there)
// ===========================================================================
//
// Every letter below is a template on the controller's number, 2 by
// default, and does its work inside `if constexpr` on the part's count of
// controllers: on a part with one the body is a discarded branch the
// compiler never instantiates, and the image carries none of it - its
// state included, which lives in a template for the same reason.

/// What DMA2's handlers count: the bare channels' completions, beside
/// UART4's transport and the far letters' copy engine.
template <uint8_t C = 2>
struct Dma2State {
    static inline volatile uint16_t calls[11] = {};
    static inline volatile uint32_t mask[11] = {};

    static void note(uint8_t index, uint32_t flags) {
        if (flags != 0u) {
            calls[index] = static_cast<uint16_t>(calls[index] + 1u);
            mask[index] = mask[index] | flags;
        }
    }

    static void reset() {
        for (uint8_t i = 0; i < 11u; ++i) {
            calls[i] = 0;
            mask[i] = 0;
        }
    }
};

template <uint8_t C>
void all_off2() {
    if constexpr (C <= device::dma_controller_count) {
        Dma<C>::open();
        Dma<C>::stop_all();
        for (uint8_t ch = 1; ch <= Dma<C>::channel_count; ++ch) {
            Pfic::disable(dma_channel_irq(C, ch));
            Pfic::clear_pending(dma_channel_irq(C, ch));
        }
        Dma2State<C>::reset();
    }
}

/// Is `Src` wired to `Dst`? The jumper test of letter g for any pair.
template <typename Src, typename Dst>
bool strapped() {
    Src::output(false);
    Dst::input(PinPull::up);
    spin_us<Ruler>(20);
    const bool low_seen = !Dst::read();
    Src::set();
    spin_us<Ruler>(20);
    const bool high_seen = Dst::read();
    Dst::input(PinPull::down);
    Src::clear();
    spin_us<Ruler>(20);
    const bool low_again = !Dst::read();
    Src::release();
    Dst::release();
    return low_seen && high_seen && low_again;
}

/// Everything a transport's receive engine has collected, published and
/// read away: how many bytes that was.
template <typename Port>
uint16_t drain() {
    uint16_t n = 0;
    uint8_t b = 0;
    while (Port::read_byte(b)) {
        ++n;
    }
    return n;
}

// ===========================================================================
// q - the priority rule: a receive ring outranks the copies
// ===========================================================================

/// The receive transport of letters q and r: USART2 on both its engines,
/// as letter g's, with a ring of 512 - room for a consumer behind a
/// stream at 4.5 Mbaud - at the level the transport arms it at by
/// default; and the same port told to arm its ring at low, the
/// arrangement the priority rule replaces (UartOptions::rx_priority).
using FatTx = DmaTxEngine<1, DmaRequestOf<DmaRequest::usart2_tx>::channel>;
using FatRx = DmaRxEngine<1, DmaRequestOf<DmaRequest::usart2_rx>::channel>;
using Fat = Uart<2, P, 512, 64, UartFormat{}, FatTx, FatRx>;
using FatLow = Uart<2, P, 512, 64, UartFormat{}, FatTx, FatRx, 0,
                    UartOptions{.rx_priority = DmaPriority::low}>;
using FatTxChannel = DmaChannel<1, DmaRequestOf<DmaRequest::usart2_tx>::channel>;
constexpr uint16_t fat_ring = 512;
/// Which of the two owns USART2's vectors while letters q and r run: 1
/// Fat, 2 FatLow, 0 neither.
volatile uint8_t fat_mode = 0;

/// THE LOAD: a memory-to-memory copy on DMA1's channel 3 - a lower number
/// than the ring's 6 - of two kilobytes as bytes (about 12700 cycles, nine
/// character times at 1 Mbaud), launched again from its own completion
/// for as long as `copy_loop` stands: back to back.
using Copier = DmaCopyEngine<1, 3>;
constexpr uint16_t copy_bytes = 2048;
volatile bool copy_loop = false;
volatile uint32_t copies_done = 0;
/// The copy engine's completion is channel 3's vector's while this stands.
volatile bool copy_mode = false;

void copy_once() { (void)Copier::copy(destination + copy_bytes, destination, copy_bytes); }

/// THE SENDER: the fourth port's transmitter, fed by a bare transmit
/// engine on its own slot with the image's pattern and no interrupt -
/// UART4 on DMA2's channel 5 on the CH32V303, USART4 on DMA1's channel 1
/// on the CH32V203 (table 11-5 and 11-3) - at the transmit level, high.
/// Its TX pad is wired to USART2's RX, PA3, on both boards (PC10 on the
/// evaluation board, PB0 on the core board). A template, so that nothing
/// of it is formed on a part without the instance.
template <uint8_t n = 4>
struct Feeder {
    using Port = Usart<n>;
    static constexpr DmaSlot slot = Port::dma_tx_slot;
    using Engine = DmaTxEngine<slot.controller, slot.channel, uint8_t>;
    using TxPad = Pin<Port::pads.tx.port, Port::pads.tx.pin>;
    // The slot is the port's transmit request's for the whole image
    // (docs/design/dma.md, "The ownership rule").
    static_assert(dma_claim<DmaSlotId<slot.controller, slot.channel>,
                            DmaOwner<DmaPeriph::usart, n, DmaSignal::tx>>);

    static bool open(uint32_t baud) {
        Port::bus_clock(true);
        Port::reset();
        if (!Port::configure(UartFormat{}, usart_divisor(usart_bus_hz<n>(clock), baud))) {
            return false;
        }
        TxPad::function();
        Port::dma_transmit(true);
        Port::transmitter(true);
        Port::enable(true);
        return Engine::arm(Port::data_address(), uint8_t{0}, DmaPriority::high);
    }

    /// A block finished: the channel stopped, true. Polled - the engine
    /// takes no interrupt.
    static bool finished() {
        if (Engine::busy() && Engine::progress().remaining == 0u) {
            (void)Engine::complete();
        }
        return !Engine::busy();
    }

    static void close() {
        Engine::stop();
        Port::reset();
        Port::bus_clock(false);
        TxPad::release();
    }
};

/// What the consumer read, judged against the pattern the sender pours.
struct Sink {
    uint32_t got = 0;
    uint32_t bad = 0;
};

template <typename Port>
void take(Sink& s) {
    const uint8_t* src = source_first_bytes();
    for (;;) {
        const std::span<const uint8_t> run = Port::read_span();
        if (run.empty()) {
            return;
        }
        for (const uint8_t b : run) {
            if (b != src[s.got & (source_bytes - 1u)]) {
                s.bad = s.bad + 1u;
            }
            s.got = s.got + 1u;
        }
        (void)Port::consume(static_cast<uint32_t>(run.size()));
    }
}

/// One cell: `blocks` runs of the 4 KB pattern from the fourth port into
/// USART2's ring at `baud`, the copies back to back beside it when `load`.
struct Cell {
    uint32_t sent = 0;
    uint32_t got = 0;
    uint32_t bad = 0;
    uint32_t entries = 0;
    uint32_t copies = 0;
    uint32_t us = 0;
    uint16_t hw = 0;
    uint16_t ring = 0;
    uint16_t faults = 0;
    uint8_t level = 0;
};

template <typename Port, uint8_t mode, typename F>
Cell stream_cell(uint32_t baud, uint16_t blocks, bool load) {
    Cell c{};
    const uint8_t* src = source_first_bytes();
    fat_mode = mode;
    const bool opened = Port::init(clock, baud) && F::open(baud);
    c.level = static_cast<uint8_t>(LoopRxChannel::configuration().priority);
    spin_us<Ruler>(500);
    Sink junk{};
    take<Port>(junk);   // what the pad framed before the sender drove it
    Port::clear_errors();
    const uint16_t f0 = Port::dma_faults();
    copies_done = 0;
    if (load) {
        Copier::stop();
        Copier::arm(DmaPriority::low, true);   // the copy engine's default, the rule's
        copy_mode = true;
        copy_loop = true;
        copy_once();
    }
    ring_entries = 0;
    ring_budget = 0xFFFF'FFFFUL;   // counted, never silenced: the control's starvation is the point
    ring_watch = true;
    Sink s{};
    uint16_t started = 0;
    Stopwatch<Ruler> w;
    w.start();
    Stopwatch<Ruler> tail;
    tail.start();
    bool tail_running = false;
    if (opened) {
        (void)F::Engine::start(std::span<const uint8_t>(src, source_bytes));
        started = 1;
    }
    const uint32_t budget_us = 40'000UL + (blocks * 4096UL * 10UL * 1000UL) / (baud / 1000UL);
    while (opened && us_of(w) < budget_us) {
        take<Port>(s);
        if (F::finished()) {
            if (started < blocks) {
                (void)F::Engine::start(std::span<const uint8_t>(src, source_bytes));
                started = static_cast<uint16_t>(started + 1u);
            } else if (!tail_running) {
                tail.start();
                tail_running = true;
            } else if (s.got >= static_cast<uint32_t>(blocks) * source_bytes || us_of(tail) > 3'000UL) {
                break;
            }
        }
    }
    copy_loop = false;
    {
        Stopwatch<Ruler> q;
        q.start();
        while (Copier::busy() && us_of(q) < 5'000UL) {
        }
    }
    ring_watch = false;
    ring_budget = ring_entry_budget;
    take<Port>(s);
    c.us = us_of(w);
    c.sent = static_cast<uint32_t>(started) * source_bytes;
    c.got = s.got;
    c.bad = s.bad;
    c.entries = ring_entries;
    c.copies = copies_done;
    c.hw = Port::hw_overruns();
    c.ring = Port::rx_overruns();
    c.faults = static_cast<uint16_t>(Port::dma_faults() - f0);
    Port::release();
    F::close();
    Copier::stop();
    Pfic::disable(dma_channel_irq(1, 3));
    copy_mode = false;
    fat_mode = 0;
    return c;
}

void print_cell(const char* what, const Cell& c) {
    print(serial, "  ", what, ": ", c.got, " of ", c.sent, " bytes, ", c.bad, " wrong, ",
          c.hw, " hardware overruns, ", c.ring, " ring overruns, ", c.faults,
          " channels given up; ", c.copies, " copies in ", c.us, " us; USART2's vector ",
          c.entries, " entries", crlf);
}

/// THE TWO ENDS' RATES, which the stream alone cannot tell - the sender
/// and the ring divide PB1 by one arithmetic: the fourth port's transmit
/// engine pouring one 4 KB block with nothing listening, timed on the
/// core's counter from the block's start to TC after its last frame, and
/// USART2's own transmit engine through the transport of letters q and r
/// (wire_permille), each in thousandths of the wire at the rate asked.
struct EndRates {
    uint32_t sender;
    uint32_t port;
};

template <typename F>
EndRates end_rates(uint32_t baud) {
    EndRates r{0, 0};
    // TE sends an idle frame after each open (18.2): out before the clock.
    const uint32_t frames_us = 20u * 1'000'000u / baud + 10u;
    if (F::open(baud)) {
        spin_us<Ruler>(frames_us);
        const uint64_t wire = 10ULL * source_bytes * SysClock::hz / baud;
        const uint64_t budget = 2u * wire + SysClock::hz / 100u;
        Stopwatch<Ruler> w;
        w.start();
        (void)F::Engine::start(std::span<const uint8_t>(source_first_bytes(), source_bytes));
        F::Port::clear_flags(usart_tc);   // TC stood for the idle line before the block
        while (!F::finished() && w.elapsed() < budget) {
        }
        while (!F::Port::tx_complete() && w.elapsed() < budget) {
        }
        r.sender = static_cast<uint32_t>(1000ULL * w.elapsed() / wire);
    }
    F::close();
    fat_mode = 1;
    (void)Fat::init(clock, baud);
    spin_us<Ruler>(frames_us);
    r.port = wire_permille<Fat>(baud, 1024);
    Fat::release();
    fat_mode = 0;
    return r;
}

template <uint8_t n = 4>
bool sender_wired() {
    if constexpr (device::has_usart(n)) {
        using F = Feeder<n>;
        return strapped<typename F::TxPad, Pin<'A', 3>>();
    } else {
        return false;
    }
}

template <uint8_t n = 4>
void tq_priority() {
    all_off();
    all_off2<2>();
    if constexpr (!device::has_usart(n)) {
        bench.skip("the priority letter: this part has no fourth serial port to send with");
    } else {
        using F = Feeder<n>;
        const bool wired = sender_wired<n>();
        print(serial, "  the wire: the fourth port's TX to PA3 ", wired ? "in place" : "ABSENT",
              crlf);
        if (!wired) {
            bench.skip("the priority letter: it needs the fourth port's TX wired to USART2's RX");
            return;
        }

        // The defaults the transport arms: the ring at very_high, its
        // transmit channel at high (DmaPriority's rule).
        // The transmit engine writes its word into CFGR with its first
        // block, and a block's end leaves it there without EN.
        fat_mode = 1;
        (void)Fat::init(clock, 115200);
        const DmaPriority ring_level = LoopRxChannel::configuration().priority;
        (void)Fat::write_byte('\n');
        {
            Stopwatch<Ruler> w;
            w.start();
            while (!Fat::tx_idle() && us_of(w) < 2'000UL) {
            }
        }
        const DmaPriority tx_level = FatTxChannel::configuration().priority;
        Fat::release();
        fat_mode = 2;
        (void)FatLow::init(clock, 115200);
        const DmaPriority low_level = LoopRxChannel::configuration().priority;
        FatLow::release();
        fat_mode = 0;
        print(serial, "  CFGR.PL: the ring ", static_cast<uint8_t>(ring_level), ", its transmit ",
              static_cast<uint8_t>(tx_level), ", the ring told rx_priority = low ",
              static_cast<uint8_t>(low_level), crlf);
        bench.verdict("a Uart arms its receive ring at very_high and its transmit engine at high "
                      "by default, and UartOptions::rx_priority moves the ring's level",
                      ring_level == DmaPriority::very_high && tx_level == DmaPriority::high &&
                          low_level == DmaPriority::low);

        // The rates the cells below stream at, on the wire.
        const EndRates r1 = end_rates<F>(1'000'000UL);
        const EndRates r4 = end_rates<F>(4'500'000UL);
        print(serial, "  in thousandths of the wire: the sender's 4 KB block ", r1.sender,
              " at 1 Mbaud and ", r4.sender, " at 4.5 Mbaud; USART2's transmit engine's 1024 "
              "frames ", r1.port, " and ", r4.port, crlf);
        bench.verdict("BOTH ENDS RUN AT THE RATE ASKED, which the stream alone cannot tell: the "
                      "sender's block and USART2's own transmit engine in their wire time at 1 "
                      "and 4.5 Mbaud on the core's counter, no faster than the divisor and at most 3 % slower",
                      wire_ok(r1.sender, 1'000'000) && wire_ok(r4.sender, 4'500'000) &&
                          wire_ok(r1.port, 1'000'000) && wire_ok(r4.port, 4'500'000));

        // The copy alone: its own rate, the reference for its cost.
        Copier::stop();
        Copier::arm(DmaPriority::low, true);
        copy_mode = true;
        copies_done = 0;
        copy_loop = true;
        Stopwatch<Ruler> w;
        w.start();
        copy_once();
        while (us_of(w) < 50'000UL) {
        }
        copy_loop = false;
        // The last copy's end: 2 KB in some tens of microseconds; 5 ms, and
        // the count read either way.
        (void)wait_until<Ruler>([] { return !Copier::busy(); }, 5'000);
        const uint32_t alone = copies_done;
        const uint32_t alone_us = us_of(w);
        Copier::stop();
        Pfic::disable(dma_channel_irq(1, 3));
        copy_mode = false;
        print(serial, "  the copy alone: ", alone, " copies of ", copy_bytes, " bytes in ",
              alone_us, " us", crlf);

        const Cell m1 = stream_cell<Fat, 1, F>(1'000'000UL, 4, true);
        print_cell("1 Mbaud, the copies back to back", m1);
        bench.verdict("AN UNPACED COPY ON A LOWER CHANNEL, BACK TO BACK, DOES NOT STARVE THE RING "
                      "AT 1 MBAUD: every byte of 16 KB in order, no overrun of either kind, no "
                      "channel given up",
                      m1.got == m1.sent && m1.sent == 4u * source_bytes && m1.bad == 0u &&
                          m1.hw == 0u && m1.ring == 0u && m1.faults == 0u && m1.copies > 0u);

        const Cell m4 = stream_cell<Fat, 1, F>(4'500'000UL, 16, true);
        print_cell("4.5 Mbaud, the copies back to back", m4);
        bench.verdict("... nor at 4.5 Mbaud, the link's top rate: every byte of 64 KB in order, "
                      "no overrun, no channel given up",
                      m4.got == m4.sent && m4.sent == 16u * source_bytes && m4.bad == 0u &&
                          m4.hw == 0u && m4.ring == 0u && m4.faults == 0u && m4.copies > 0u);

        // What the rule costs the copy: its rate beside each stream against
        // its rate alone, in thousandths.
        const auto per_mille = [&](const Cell& c) {
            return static_cast<uint32_t>((static_cast<uint64_t>(c.copies) * alone_us * 1000ULL) /
                                         (static_cast<uint64_t>(c.us) * (alone != 0u ? alone : 1u)));
        };
        print(serial, "  the copy's rate beside the ring, against alone: ", per_mille(m1),
              "/1000 at 1 Mbaud, ", per_mille(m4), "/1000 at 4.5 Mbaud", crlf);

        // THE CONTROL: the same load against the ring a program armed at
        // low - the arrangement the rule replaces, the copy on channel 3
        // winning every tie against channel 6.
        const Cell l1 = stream_cell<FatLow, 2, F>(1'000'000UL, 2, true);
        print_cell("1 Mbaud, the ring at low (the control)", l1);
        bench.verdict("and the load is real: with the ring told rx_priority = low the same copies "
                      "overrun it",
                      l1.level == static_cast<uint8_t>(DmaPriority::low) && l1.hw > 0u);
        all_off2<2>();
        all_off();
    }
}

// ===========================================================================
// r - an overrun at a message's tail: cleared, counted once, no storm
// ===========================================================================

/// The bytes the ring's channel has written since `at`, a count it read.
uint16_t arrived_since(uint16_t at) {
    return static_cast<uint16_t>((at - LoopRxChannel::remaining()) & (fat_ring - 1u));
}

template <uint8_t n = 4>
void tr_overrun_tail() {
    all_off();
    all_off2<2>();
    if constexpr (!device::has_usart(n)) {
        bench.skip("the overrun letter: this part has no fourth serial port to send with");
    } else {
        using F = Feeder<n>;
        const bool wired = sender_wired<n>();
        print(serial, "  the wire: the fourth port's TX to PA3 ", wired ? "in place" : "ABSENT",
              crlf);
        if (!wired) {
            bench.skip("the overrun letter: it needs the fourth port's TX wired to USART2's RX");
            return;
        }
        const uint8_t* src = source_first_bytes();
        constexpr uint16_t msg = 32;
        constexpr uint16_t lead = 28;
        const EndRates rates = end_rates<F>(1'000'000UL);
        print(serial, "  at 1 Mbaud: the sender's 4 KB block in ", rates.sender,
              " and USART2's transmit engine's 1024 frames in ", rates.port,
              " thousandths of their wire time", crlf);
        bench.verdict("both ends run at the rate asked: in their wire time at 1 Mbaud, no faster than the "
                      "divisor and at most 3 % slower",
                      wire_ok(rates.sender, 1'000'000) && wire_ok(rates.port, 1'000'000));
        fat_mode = 1;
        const bool opened = Fat::init(clock, 1'000'000UL) && F::open(1'000'000UL);
        spin_us<Ruler>(500);
        Sink junk{};
        take<Fat>(junk);
        Fat::clear_errors();
        // The staging's own block, at the ring's level on a lower channel:
        // it wins the tie, and the ring is starved for as long as it runs.
        Copier::stop();
        Copier::arm(DmaPriority::very_high, false);

        // 1. THE OVERRUN AT A MESSAGE'S TAIL. A message of 32 starts; when
        // 28 have landed the copy starves the ring for nine frame times,
        // the tail's frames overrun it, and the copy's end lets the channel
        // take the frame DATAR held - ORE left up with RXNE down and no
        // frame after it. All of it under the mask, so the vector first
        // looks AFTER the channel took that frame, as a vector held off by
        // another one does: the state the storm grew from.
        const uint16_t hw0 = Fat::hw_overruns();
        const uint16_t f0 = Fat::dma_faults();
        const uint32_t k0 = Fat::rx_skips();
        const uint16_t at0 = LoopRxChannel::remaining();
        uint16_t before_copy = 0;
        bool copied = false;
        {
            P::CriticalSection cs;
            (void)F::Engine::start(std::span<const uint8_t>(src + 64, msg));
            MaskedStopwatch w;
            while (arrived_since(at0) < lead && w.us() < 2'000UL) {
            }
            before_copy = arrived_since(at0);
            copied = Copier::copy(destination + copy_bytes, destination, copy_bytes);
            // 2 KB a byte a beat ends in some tens of microseconds; 1 ms.
            MaskedStopwatch c;
            while (Copier::busy() && c.us() < 1'000UL) {
            }
            MaskedStopwatch t;
            while (t.us() < 300UL) {   // the tail sent and the line idle
            }
            ring_entries = 0;
            ring_watch = true;
        }
        spin_us<Ruler>(3'000);
        ring_watch = false;
        (void)F::finished();
        const uint32_t entries = ring_entries;
        const uint16_t hw = static_cast<uint16_t>(Fat::hw_overruns() - hw0);
        const uint16_t faults = static_cast<uint16_t>(Fat::dma_faults() - f0);
        const uint32_t skips = Fat::rx_skips() - k0;
        Sink s1{};
        uint8_t first[msg] = {};
        {
            uint8_t b = 0;
            while (Fat::read_byte(b)) {
                if (s1.got < msg) {
                    first[s1.got] = b;
                }
                s1.got = s1.got + 1u;
            }
        }
        bool lead_in_order = s1.got >= lead && s1.got < msg;
        for (uint16_t i = 0; i < lead && lead_in_order; ++i) {
            lead_in_order = first[i] == src[64u + i];
        }
        const uint16_t ctlr1 = Fat::Resource::regs().CTLR1;
        const bool waiting_for_the_end = (ctlr1 & usart_idleie) != 0u && (ctlr1 & usart_rxneie) == 0u;
        print(serial, "  a message of ", msg, ", the copy at its tail (", before_copy,
              " landed, the copy ", copied ? "run" : "REFUSED", "): ", s1.got,
              " bytes delivered, USART2's vector ", entries, " entries, ", hw,
              " overruns counted, ", skips, " gaps, ", faults, " channels given up; the vector ",
              waiting_for_the_end ? "waiting for the end" : "NOT waiting for the end", crlf);
        // THE COUNT, derived from the vector's two states (ch32vx03/usart.hpp):
        // unmasked, the vector waiting for the end takes ONE entry for the
        // overrun and the idle line together and turns to its wait for a
        // frame, RXNEIE up; ORE alone re-enters it (18.10.4) with the
        // channel's count unmoved, and the second such entry
        // (rx_overrun_probe) reads STATR and DATAR, clearing ORE, and goes
        // back to the wait for the end - 1 + 2 entries exactly.
        bench.verdict("AN OVERRUN AT A MESSAGE'S TAIL, ORE up with nothing in DATAR: no storm - "
                      "three entries, the overrun probed on the second unmoved one, no channel "
                      "given up - the overrun counted once and the gap once, the bytes before it "
                      "delivered in order",
                      opened && copied && entries == 1u + 2u && faults == 0u && hw == 1u &&
                          skips == 1u && lead_in_order && waiting_for_the_end);

        // 2. The next message, whole: the vector left waiting for the end
        // with nothing standing.
        const uint32_t e2 = static_cast<uint32_t>(Fat::frame_errors()) + Fat::noise_errors() +
                            Fat::parity_errors();
        (void)F::Engine::start(std::span<const uint8_t>(src + 128, msg));
        {
            Stopwatch<Ruler> w;
            w.start();
            while (!F::finished() && us_of(w) < 2'000UL) {
            }
        }
        spin_us<Ruler>(500);
        bool second_in_order = true;
        uint16_t second = 0;
        {
            uint8_t b = 0;
            while (Fat::read_byte(b)) {
                if (second < msg && b != src[128u + second]) {
                    second_in_order = false;
                }
                ++second;
            }
        }
        const uint32_t e2_after = static_cast<uint32_t>(Fat::frame_errors()) +
                                  Fat::noise_errors() + Fat::parity_errors();
        print(serial, "  the next message: ", second, " of ", msg,
              second_in_order ? " in order" : " NOT in order", ", errors counted ",
              e2_after - e2, ", overruns ",
              static_cast<uint16_t>(Fat::hw_overruns() - hw0), crlf);
        bench.verdict("... and the next message arrives whole, nothing else counted",
                      second == msg && second_in_order && e2_after == e2 &&
                          Fat::hw_overruns() - hw0 == 1u && Fat::dma_faults() == f0);

        // 3. THE WINDOW, STAGED WIDE: a frame lands while the ring is
        // starved, and the CPU reads STATR and then DATAR under it - what
        // the overrun's clear would do to a frame landing between its two
        // loads. What the channel does with the request that frame raised,
        // once the copy lets it go, is the measurement.
        uint16_t c0 = 0;
        uint16_t c1 = 0;
        uint16_t c2 = 0;
        uint16_t st = 0;
        uint8_t cpu = 0;
        {
            P::CriticalSection cs;
            c0 = LoopRxChannel::remaining();
            (void)Copier::copy(destination + copy_bytes, destination, copy_bytes);
            F::Port::write_data(0xA5);
            MaskedStopwatch w;
            while (!Fat::Resource::rx_ready() && w.us() < 200UL) {
            }
            st = Fat::Resource::status();
            cpu = Fat::Resource::read_data();
            c1 = LoopRxChannel::remaining();
            MaskedStopwatch c;
            while (Copier::busy() && c.us() < 1'000UL) {
            }
            MaskedStopwatch t;
            while (t.us() < 50UL) {
            }
            c2 = LoopRxChannel::remaining();
        }
        spin_us<Ruler>(500);
        uint16_t ring_after = 0;
        uint8_t ring_byte = 0;
        {
            uint8_t b = 0;
            while (Fat::read_byte(b)) {
                ring_byte = b;
                ++ring_after;
            }
        }
        print(serial, "  a frame read by the CPU under a starved channel (STATR ", hex(st),
              ", the CPU read ", hex(cpu), "): the channel moved ",
              static_cast<uint16_t>((c0 - c1) & (fat_ring - 1u)), " before the copy ended and ",
              static_cast<uint16_t>((c1 - c2) & (fat_ring - 1u)), " after; the ring then held ",
              ring_after, ring_after != 0u ? " byte(s), the last " : " bytes", crlf);
        if (ring_after != 0u) {
            print(serial, "    ", hex(ring_byte), crlf);
        }

        // 4. And the port is whole after it.
        (void)F::Engine::start(std::span<const uint8_t>(src + 192, msg));
        {
            Stopwatch<Ruler> w;
            w.start();
            while (!F::finished() && us_of(w) < 2'000UL) {
            }
        }
        spin_us<Ruler>(500);
        bool third_in_order = true;
        uint16_t third = 0;
        {
            uint8_t b = 0;
            while (Fat::read_byte(b)) {
                if (third < msg && b != src[192u + third]) {
                    third_in_order = false;
                }
                ++third;
            }
        }
        bench.verdict("... and after a frame read from under the channel, the next message whole "
                      "too",
                      third == msg && third_in_order && Fat::dma_faults() == f0);

        Fat::release();
        F::close();
        Copier::stop();
        fat_mode = 0;
        all_off2<2>();
        all_off();
    }
}

// ===========================================================================
// THE PAIR'S LETTERS WHERE UART4 IS THE LINK (the second key `y`): n, q and
// r with the far chip's usart_peer as the sender
// ===========================================================================
//
// The CH32V303RC's and VC's crossed pair is retired - UART4's default pads
// PC10/PC11 are the desk pair's SPI link - so there these three letters
// take the pair's serial link instead (vx03_link_dut.hpp): UART4 on
// its SECOND column (PB0/PB1), whose requests are DMA2's channels 5 out
// and 3 in (table 11-3), and the far chip's usart_peer the sender - its
// blast a counting stream, gapless, on its crystal. They FAIL loudly when
// nobody answers, so they run under `y` and never under `z`.

/// Whether the far letters register: where the pair's link is UART4, whose
/// two requests are DMA2's (vx03_link_dut.hpp).
constexpr bool far_letters = vx03link::on_fourth_port;
/// Whether the single-board q and r register: where the part has a fourth
/// port and keeps the crossed pair (vx03_pair.hpp).
constexpr bool crossed_pair_part = device::has_usart(4) && !vx03pair::fourth_default_on_spi_link;

template <bool on>
struct FarSide {
    using D = vx03link::Dut<Serial, SysClock>;
    using TxRow = DmaRequestOf<on ? DmaRequest::uart4_tx : DmaRequest::uart4_tx>;
    using RxRow = DmaRequestOf<on ? DmaRequest::uart4_rx : DmaRequest::uart4_rx>;
    static_assert(!on || (TxRow::controller == 2u && RxRow::controller == 2u),
                  "table 11-3: UART4's two requests are DMA2's on this class");
    using TxEngine = DmaTxEngine<TxRow::controller, TxRow::channel>;
    using RxEngine = DmaRxEngine<RxRow::controller, RxRow::channel>;
    /// The ring of letters q and r, 512 as USART2's is on the CH32V203,
    /// at the transport's default level; and the same port told low.
    using Port = Uart<4, P, 512, 64, UartFormat{}, TxEngine, RxEngine, vx03link::column>;
    using PortLow = Uart<4, P, 512, 64, UartFormat{}, TxEngine, RxEngine, vx03link::column,
                         UartOptions{.rx_priority = DmaPriority::low}>;
    using RxChannel = DmaChannel<RxRow::controller, RxRow::channel>;
    /// THE LOAD: a memory-to-memory copy on DMA2's channel 1, a lower number
    /// than the ring's 3 - the CH32V203 letters' arrangement on the other
    /// controller.
    using Copier = DmaCopyEngine<2, 1>;
};

/// Which port the link's vectors serve while a far letter runs: 1 Port,
/// 2 PortLow, 0 neither; and whether DMA2's channel 1 is the far copier's.
volatile uint8_t far_mode = 0;
volatile bool far_copy_mode = false;
volatile bool far_copy_loop = false;
volatile uint32_t far_copies_done = 0;

template <bool on>
void far_copy_once() {
    (void)FarSide<on>::Copier::copy(destination + copy_bytes, destination, copy_bytes);
}

/// The far letters' prologue: the board's DMA2 and the link found, the far
/// chip's peer on the far side.
template <bool on>
bool far_ready() {
    using D = typename FarSide<on>::D;
    all_off();
    all_off2<2>();
    if (!D::ensure()) {
        bench.verdict("the peer is reachable on the pair's serial link", false);
        return false;
    }
    link::Ident d{};
    if (!D::ident(d) || !D::far_peer(d)) {
        bench.verdict("the far chip's usart_peer answers", false);
        return false;
    }
    return true;
}

template <bool on>
void far_done() {
    using D = typename FarSide<on>::D;
    far_mode = 0;
    far_copy_mode = false;
    (void)D::command_mode();
    D::release();
    all_off2<2>();
    all_off();
}

/// What the consumer read of a counting stream, judged byte by byte.
struct SeqSink {
    uint32_t got = 0;
    uint32_t bad = 0;
    uint8_t expect = 0;
};

template <typename T>
void take_seq(SeqSink& s) {
    for (;;) {
        const std::span<const uint8_t> run = T::read_span();
        if (run.empty()) {
            return;
        }
        for (const uint8_t b : run) {
            if (b != s.expect) {
                s.bad = s.bad + 1u;
            }
            s.expect = static_cast<uint8_t>(b + 1u);
            s.got = s.got + 1u;
        }
        (void)T::consume(static_cast<uint32_t>(run.size()));
    }
}

/// The far chip's blast of `blocks` x 4096 counting frames at `baud`,
/// gapless, into the ring of `T` - the copies back to back on DMA2's
/// channel 1 beside it when `load`.
template <bool on, typename T, uint8_t mode>
Cell far_cell(uint32_t baud, uint16_t blocks, bool load) {
    using F = FarSide<on>;
    using D = typename F::D;
    Cell c{};
    link::Params a{};
    a.cfg.rate = baud;
    a.count = 4096;
    a.aux16 = blocks;
    a.ms = 4000;
    a.value = 0;
    a.pattern = link::pattern_counting;
    const bool acted = D::act(link::Op::blast, a);
    F::D::W::bus_clock(true);
    F::D::W::reset();
    far_mode = mode;
    const bool opened = acted && T::init(clock, baud);
    F::D::WRx::input(PinPull::up);
    c.level = static_cast<uint8_t>(F::RxChannel::configuration().priority);
    const uint16_t f0 = T::dma_faults();
    far_copies_done = 0;
    if (load) {
        F::Copier::stop();
        F::Copier::arm(DmaPriority::low, true);   // the copy engine's default, the rule's
        far_copy_mode = true;
        far_copy_loop = true;
        far_copy_once<on>();
    }
    ring_entries = 0;
    ring_budget = 0xFFFF'FFFFUL;   // counted, never silenced: the control's starvation is the point
    ring_watch = true;
    SeqSink s{};
    const uint32_t total = static_cast<uint32_t>(blocks) * 4096u;
    const uint32_t budget_us = 40'000UL + (total * 10UL * 1000UL) / (baud / 1000UL);
    Stopwatch<Ruler> w;
    w.start();
    Stopwatch<Ruler> quiet;
    quiet.start();
    uint32_t last_got = 0;
    while (opened && us_of(w) < budget_us && s.got < total) {
        take_seq<T>(s);
        if (s.got != last_got) {
            last_got = s.got;
            quiet.start();
        } else if (s.got != 0u && us_of(quiet) > 3'000UL) {
            break;
        }
    }
    far_copy_loop = false;
    {
        Stopwatch<Ruler> q;
        q.start();
        while (F::Copier::busy() && us_of(q) < 5'000UL) {
        }
    }
    ring_watch = false;
    ring_budget = ring_entry_budget;
    take_seq<T>(s);
    c.us = us_of(w);
    c.sent = total;
    c.got = s.got;
    c.bad = s.bad;
    c.entries = ring_entries;
    c.copies = far_copies_done;
    c.hw = T::hw_overruns();
    c.ring = T::rx_overruns();
    c.faults = static_cast<uint16_t>(T::dma_faults() - f0);
    T::release();
    F::Copier::stop();
    Pfic::disable(dma_channel_irq(2, 1));
    far_copy_mode = false;
    far_mode = 0;
    link::Report r{};
    (void)D::report(r, 5);
    return c;
}

void print_far_cell(const char* what, const Cell& c) {
    print(serial, "  ", what, ": ", c.got, " of ", c.sent, " bytes, ", c.bad, " out of order, ", c.hw,
          " hardware overruns, ", c.ring, " ring overruns, ", c.faults, " channels given up; ", c.copies,
          " copies in ", c.us, " us; UART4's vector ", c.entries, " entries", crlf);
}

// ---------------------------------------------------------------------------
// n - UART4's engines on DMA2 against the far chip
// ---------------------------------------------------------------------------

template <bool on = far_letters>
void tn_far_uart4() {
    if constexpr (on) {
        using F = FarSide<on>;
        using D = typename F::D;
        if (!far_ready<on>()) {
            far_done<on>();
            return;
        }
        static_assert(F::Port::pads.tx == Pad{'B', 0} && F::Port::pads.rx == Pad{'B', 1},
                      "table 10-26: UART4's second column is PB0/PB1");
        // A run poured out by the transmit engine on DMA2's channel 5 into
        // the far chip's echo, and back in through the receive engine on
        // channel 3.
        static const uint8_t msg[] = "DMA2 out on channel 5, round the far echo, in on channel 3";
        constexpr uint16_t len = sizeof(msg) - 1u;
        link::Params a{};
        a.cfg.rate = 115'200;
        a.count = len;
        a.ms = D::bound_ms(115'200, 2u * len);
        const bool acted = D::act(link::Op::echo, a);
        D::W::bus_clock(true);
        D::W::reset();
        far_mode = 1;
        const bool opened = acted && F::Port::init(clock, 115'200);
        D::WRx::input(PinPull::up);
        D::wait_for_window();
        uint8_t b = 0;
        while (F::Port::read_byte(b)) {
        }
        (void)F::Port::write_bulk(std::span<const uint8_t>(msg, len));
        uint8_t seen[64] = {};
        uint16_t got = 0;
        Stopwatch<Ruler> w;
        w.start();
        while (got < len && us_of(w) < 50'000UL) {
            while (got < len && F::Port::read_byte(b)) {
                seen[got++] = b;
            }
        }
        bool same = got == len;
        for (uint16_t i = 0; i < got && same; ++i) {
            same = seen[i] == msg[i];
        }
        const uint16_t faults = F::Port::dma_faults();
        F::Port::release();
        far_mode = 0;
        link::Report r{};
        const bool have = D::report(r, 5);
        print(serial, "  ", got, " of ", len, " bytes back round the far echo, ", same ? "exact" : "WRONG",
              "; the far chip counted ", r.count, ", faults ", faults, crlf);
        bench.verdict("a run poured out by UART4's transmit engine on DMA2 comes back through its receive engine "
                      "on DMA2 after a second chip echoed it, byte for byte - the slots of table 11-3 answering "
                      "real requests",
                      opened && same && have && r.count == len && r.ferr == 0u && faults == 0u);
        // The engine's run timed on the core's counter, the far chip
        // sinking it.
        a = link::Params{};
        a.cfg.rate = 115'200;
        a.count = 256;
        a.ms = D::bound_ms(115'200, 512);
        const bool sunk = D::act(link::Op::sink, a);
        D::W::bus_clock(true);
        D::W::reset();
        far_mode = 1;
        const bool up = sunk && F::Port::init(clock, 115'200);
        D::WRx::input(PinPull::up);
        D::wait_for_window();
        const uint32_t pm = up ? wire_permille<typename F::Port>(115'200, 256) : 0u;
        F::Port::release();
        far_mode = 0;
        link::Report s{};
        (void)D::report(s, 5);
        print(serial, "  256 frames in ", pm, " thousandths of their wire time at 115200; the far chip counted ",
              s.count, crlf);
        bench.verdict("and the transmit engine runs at the rate asked: 256 frames in their wire time at 115200, "
                      "no faster than the divisor and at most 3 % slower, every one counted at the far end",
                      wire_ok(pm, 115'200) && s.count == 256u && s.ferr == 0u);
        far_done<on>();
    }
}

// ---------------------------------------------------------------------------
// q - the priority rule, the far chip pouring into UART4's ring
// ---------------------------------------------------------------------------

template <bool on = far_letters>
void tq_far_priority() {
    if constexpr (on) {
        using F = FarSide<on>;
        using D = typename F::D;
        if (!far_ready<on>()) {
            far_done<on>();
            return;
        }
        // The defaults the transport arms: the ring at very_high, its
        // transmit channel at high (DmaPriority's rule), and the knob.
        far_mode = 1;
        (void)F::Port::init(clock, 115200);
        const DmaPriority ring_level = F::RxChannel::configuration().priority;
        F::Port::release();
        far_mode = 2;
        (void)F::PortLow::init(clock, 115200);
        const DmaPriority low_level = F::RxChannel::configuration().priority;
        F::PortLow::release();
        far_mode = 0;
        print(serial, "  CFGR.PL of UART4's ring on DMA2: ", static_cast<uint8_t>(ring_level),
              ", told rx_priority = low ", static_cast<uint8_t>(low_level), crlf);
        bench.verdict("a Uart arms its receive ring at very_high by default on DMA2 too, and rx_priority moves it",
                      ring_level == DmaPriority::very_high && low_level == DmaPriority::low);

        // THE TWO ENDS' RATES, which the stream alone cannot tell: the far
        // chip STATES its blast's rate - its PCLK1 over its divisor, on its
        // crystal - and UART4's own transmit engine is timed on this core's
        // counter, the far chip sinking and counting its frames.
        bool rates_ok = true;
        for (const uint32_t baud : {1'000'000UL, 4'500'000UL}) {
            link::Params a{};
            a.cfg.rate = baud;
            a.count = 64;
            a.ms = 200;
            const bool acted = D::act(link::Op::blast, a);
            D::W::bus_clock(true);
            D::W::reset();
            far_mode = 1;
            const bool up = acted && F::Port::init(clock, baud);
            D::WRx::input(PinPull::up);
            // The far chip starts settle_ms + 2 ms after its ack in its
            // ticks: the 64 frames awaited, bounded.
            SeqSink junk{};
            {
                Stopwatch<Ruler> w;
                w.start();
                while (junk.got < 64u && us_of(w) < 20'000UL) {
                    take_seq<typename F::Port>(junk);
                }
            }
            F::Port::release();
            far_mode = 0;
            link::Report r{};
            const bool have = D::report(r, 2);
            const int32_t stated = static_cast<int16_t>(r.aux16);
            a = link::Params{};
            a.cfg.rate = baud;
            a.count = 1024;
            a.ms = D::bound_ms(baud, 2048);
            const bool sunk = D::act(link::Op::sink, a);
            D::W::bus_clock(true);
            D::W::reset();
            far_mode = 1;
            const bool up2 = sunk && F::Port::init(clock, baud);
            D::WRx::input(PinPull::up);
            D::wait_for_window();
            const uint32_t pm = up2 ? wire_permille<typename F::Port>(baud, 1024) : 0u;
            F::Port::release();
            far_mode = 0;
            link::Report k{};
            (void)D::report(k, 5);
            print(serial, "  at ", baud, " baud: the far blast's rate stated ",
                  (r.flags & link::report_rate_stated) != 0u ? "" : "NOT ", stated, " ppm (", junk.got,
                  " frames here); UART4's transmit engine 1024 frames in ", pm,
                  " thousandths of their wire time, the far chip counting ", k.count, crlf);
            rates_ok = rates_ok && up && have && (r.flags & link::report_rate_stated) != 0u &&
                       stated == D::divisor_ppm(baud) && junk.got == 64u && wire_ok(pm, baud) && k.count == 1024u &&
                       k.ferr == 0u;
        }
        bench.verdict("BOTH ENDS RUN AT THE RATE ASKED, which the stream alone cannot tell: the far chip states its "
                      "rate exact at 1 and 4.5 Mbaud, and UART4's transmit engine runs in its wire time on this "
                      "core's counter, no faster than the divisor and at most 3 % slower, every frame counted at "
                      "the far end",
                      rates_ok);

        // The copy alone: its own rate, the reference for its cost.
        F::Copier::stop();
        F::Copier::arm(DmaPriority::low, true);
        far_copy_mode = true;
        far_copies_done = 0;
        far_copy_loop = true;
        Stopwatch<Ruler> w;
        w.start();
        far_copy_once<on>();
        while (us_of(w) < 50'000UL) {
        }
        far_copy_loop = false;
        // The last copy's end, as letter q's own: 5 ms, the count read either way.
        (void)wait_until<Ruler>([] { return !F::Copier::busy(); }, 5'000);
        const uint32_t alone = far_copies_done;
        const uint32_t alone_us = us_of(w);
        F::Copier::stop();
        Pfic::disable(dma_channel_irq(2, 1));
        far_copy_mode = false;
        print(serial, "  the copy alone: ", alone, " copies of ", copy_bytes, " bytes in ", alone_us, " us", crlf);

        const Cell m1 = far_cell<on, typename F::Port, 1>(1'000'000UL, 4, true);
        print_far_cell("1 Mbaud, the copies back to back", m1);
        bench.verdict("AN UNPACED COPY ON A LOWER DMA2 CHANNEL, BACK TO BACK, DOES NOT STARVE UART4'S RING AT 1 "
                      "MBAUD: every byte of the far chip's 16 KB in order, no overrun, no channel given up",
                      m1.got == m1.sent && m1.bad == 0u && m1.hw == 0u && m1.ring == 0u && m1.faults == 0u &&
                          m1.copies > 0u);
        const Cell m4 = far_cell<on, typename F::Port, 1>(4'500'000UL, 16, true);
        print_far_cell("4.5 Mbaud, the copies back to back", m4);
        bench.verdict("... nor at 4.5 Mbaud, the link's top rate: every byte of 64 KB in order, no overrun, no "
                      "channel given up",
                      m4.got == m4.sent && m4.bad == 0u && m4.hw == 0u && m4.ring == 0u && m4.faults == 0u &&
                          m4.copies > 0u);
        const auto per_mille = [&](const Cell& c) {
            return static_cast<uint32_t>((static_cast<uint64_t>(c.copies) * alone_us * 1000ULL) /
                                         (static_cast<uint64_t>(c.us) * (alone != 0u ? alone : 1u)));
        };
        print(serial, "  the copy's rate beside the ring, against alone: ", per_mille(m1), "/1000 at 1 Mbaud, ",
              per_mille(m4), "/1000 at 4.5 Mbaud", crlf);
        // THE CONTROL: the same load against the ring told low - the copy
        // on channel 1 wins every tie against channel 3.
        const Cell l1 = far_cell<on, typename F::PortLow, 2>(1'000'000UL, 2, true);
        print_far_cell("1 Mbaud, the ring at low (the control)", l1);
        bench.verdict("and the load is real: with the ring told rx_priority = low the same copies overrun it",
                      l1.level == static_cast<uint8_t>(DmaPriority::low) && l1.hw > 0u);
        far_done<on>();
    }
}

// ---------------------------------------------------------------------------
// r - an overrun at a message's tail, the far chip the sender
// ---------------------------------------------------------------------------

/// The bytes the ring's channel has written since `at`.
template <bool on>
uint16_t far_arrived_since(uint16_t at) {
    return static_cast<uint16_t>((at - FarSide<on>::RxChannel::remaining()) & (fat_ring - 1u));
}

template <bool on = far_letters>
void tr_far_overrun_tail() {
    if constexpr (on) {
        using F = FarSide<on>;
        using D = typename F::D;
        using Port = typename F::Port;
        if (!far_ready<on>()) {
            far_done<on>();
            return;
        }
        constexpr uint16_t msg = 32;
        constexpr uint16_t lead = 28;
        // One message from the far chip: `act` it, the ring brought up, and
        // the first frame awaited with the mask on - the far chip starts
        // settle_ms + 2 ms after its ack, this end's init in between.
        const auto message = [&](uint8_t seed, uint16_t count) {
            link::Params a{};
            a.cfg.rate = 1'000'000;
            a.count = count;
            a.ms = 200;
            a.value = seed;
            a.pattern = link::pattern_counting;
            return D::act(link::Op::blast, a);
        };
        // The ring on the link, opened once for the whole letter; the
        // command path below borrows the port between messages and gives it
        // back to the ring through init().
        const auto ring_up = [&]() {
            D::W::bus_clock(true);
            D::W::reset();
            far_mode = 1;
            const bool ok = Port::init(clock, 1'000'000UL);
            D::WRx::input(PinPull::up);
            return ok;
        };
        F::Copier::stop();
        F::Copier::arm(DmaPriority::very_high, false);   // the ring's level, on a lower channel: it wins the tie

        // 1. THE OVERRUN AT A MESSAGE'S TAIL: when 28 of 32 have landed the
        // copy starves the ring for nine frame times, the tail overruns it,
        // and the copy's end lets the channel take the frame DATAR held -
        // ORE left up with RXNE down and no frame after it - all under the
        // mask, so the vector first looks after that.
        const bool sent1 = message(0x40, msg);
        const bool opened = sent1 && ring_up();
        const uint16_t hw0 = Port::hw_overruns();
        const uint16_t f0 = Port::dma_faults();
        const uint32_t k0 = Port::rx_skips();
        const uint16_t at0 = F::RxChannel::remaining();
        uint16_t before_copy = 0;
        bool copied = false;
        {
            P::CriticalSection cs;
            MaskedStopwatch w;
            while (far_arrived_since<on>(at0) < lead && w.us() < 20'000UL) {
            }
            before_copy = far_arrived_since<on>(at0);
            copied = F::Copier::copy(destination + copy_bytes, destination, copy_bytes);
            MaskedStopwatch c;
            while (F::Copier::busy() && c.us() < 1'000UL) {
            }
            MaskedStopwatch t;
            while (t.us() < 300UL) {   // the tail sent and the line idle
            }
            ring_entries = 0;
            ring_watch = true;
        }
        spin_us<Ruler>(3'000);
        ring_watch = false;
        const uint32_t entries = ring_entries;
        const uint16_t hw = static_cast<uint16_t>(Port::hw_overruns() - hw0);
        const uint16_t faults = static_cast<uint16_t>(Port::dma_faults() - f0);
        const uint32_t skips = Port::rx_skips() - k0;
        uint16_t got1 = 0;
        bool lead_in_order = true;
        {
            uint8_t b = 0;
            while (Port::read_byte(b)) {
                if (got1 < lead && b != static_cast<uint8_t>(0x40u + got1)) {
                    lead_in_order = false;
                }
                ++got1;
            }
        }
        lead_in_order = lead_in_order && got1 >= lead && got1 < msg;
        const uint16_t ctlr1 = Port::Resource::regs().CTLR1;
        const bool waiting_for_the_end = (ctlr1 & usart_idleie) != 0u && (ctlr1 & usart_rxneie) == 0u;
        print(serial, "  a message of ", msg, " from the far chip, the copy at its tail (", before_copy,
              " landed, the copy ", copied ? "run" : "REFUSED", "): ", got1, " bytes delivered, UART4's vector ",
              entries, " entries, ", hw, " overruns counted, ", skips, " gaps, ", faults,
              " channels given up; the vector ", waiting_for_the_end ? "waiting for the end" : "NOT waiting for the end",
              crlf);
        // THE COUNT: 1 + 2 entries exactly, letter r's derivation on the
        // same vector body.
        bench.verdict("AN OVERRUN AT A MESSAGE'S TAIL, ORE up with nothing in DATAR, on DMA2: no storm - three "
                      "entries, the overrun probed on the second unmoved one, no channel given up - the overrun "
                      "counted once and the gap once, the bytes before it delivered in order",
                      opened && copied && entries == 1u + 2u && faults == 0u && hw == 1u && skips == 1u &&
                          lead_in_order &&
                          waiting_for_the_end);
        Port::release();
        far_mode = 0;
        link::Report r{};
        (void)D::report(r, 2);

        // 2. The next message, whole, on a ring opened again.
        const bool sent2 = message(0x80, msg) && ring_up();
        spin_us<Ruler>(4'000);
        uint16_t second = 0;
        bool second_in_order = true;
        {
            uint8_t b = 0;
            while (Port::read_byte(b)) {
                if (second < msg && b != static_cast<uint8_t>(0x80u + second)) {
                    second_in_order = false;
                }
                ++second;
            }
        }
        const uint16_t hw2 = Port::hw_overruns();
        const uint16_t faults2 = Port::dma_faults();
        print(serial, "  the next message: ", second, " of ", msg, second_in_order ? " in order" : " NOT in order",
              ", overruns ", hw2, ", faults ", faults2, crlf);
        bench.verdict("... and the next message arrives whole, nothing counted",
                      sent2 && second == msg && second_in_order && hw2 == 0u && faults2 == 0u);
        Port::release();
        far_mode = 0;
        (void)D::report(r, 2);

        // 3. THE WINDOW, STAGED WIDE: one frame from the far chip lands while
        // the ring is starved - the copy started on the start bit's fall,
        // seen on the pad - and the CPU reads STATR and then DATAR under
        // it, what the overrun's clear would do to a frame landing between
        // its two loads. What the channel does with the request that frame
        // raised, once the copy lets it go, is the measurement.
        const bool sent3 = message(0xA5, 1) && ring_up();
        uint16_t c0 = 0, c1 = 0, c2 = 0, st = 0;
        uint8_t cpu = 0;
        bool fell = false;
        {
            P::CriticalSection cs;
            c0 = F::RxChannel::remaining();
            MaskedStopwatch w;
            while (D::WRx::read() && w.us() < 20'000UL) {
            }
            fell = !D::WRx::read();
            (void)F::Copier::copy(destination + copy_bytes, destination, copy_bytes);
            MaskedStopwatch f;
            while (!Port::Resource::rx_ready() && f.us() < 200UL) {
            }
            st = Port::Resource::status();
            cpu = Port::Resource::read_data();
            c1 = F::RxChannel::remaining();
            MaskedStopwatch c;
            while (F::Copier::busy() && c.us() < 1'000UL) {
            }
            MaskedStopwatch t;
            while (t.us() < 50UL) {
            }
            c2 = F::RxChannel::remaining();
        }
        spin_us<Ruler>(500);
        uint16_t ring_after = 0;
        uint8_t ring_byte = 0;
        {
            uint8_t b = 0;
            while (Port::read_byte(b)) {
                ring_byte = b;
                ++ring_after;
            }
        }
        print(serial, "  a far frame read by the CPU under a starved channel (the start bit ",
              fell ? "seen" : "NOT SEEN", ", STATR ", hex(st), ", the CPU read ", hex(cpu), "): the channel moved ",
              static_cast<uint16_t>((c0 - c1) & (fat_ring - 1u)), " before the copy ended and ",
              static_cast<uint16_t>((c1 - c2) & (fat_ring - 1u)), " after; the ring then held ", ring_after,
              ring_after != 0u ? " byte(s), the last " : " bytes", crlf);
        if (ring_after != 0u) {
            print(serial, "    ", hex(ring_byte), crlf);
        }
        Port::release();
        far_mode = 0;
        (void)D::report(r, 2);
        bench.verdict("the staging ran: the far frame's start bit seen and read from under the starved channel",
                      sent3 && fell && cpu == 0xA5u);

        // 4. And the port is whole after it.
        const bool sent4 = message(0xC0, msg) && ring_up();
        spin_us<Ruler>(4'000);
        uint16_t third = 0;
        bool third_in_order = true;
        {
            uint8_t b = 0;
            while (Port::read_byte(b)) {
                if (third < msg && b != static_cast<uint8_t>(0xC0u + third)) {
                    third_in_order = false;
                }
                ++third;
            }
        }
        bench.verdict("... and after a frame read from under the channel, the next message whole too",
                      sent4 && third == msg && third_in_order && Port::dma_faults() == 0u);
        Port::release();
        F::Copier::stop();
        far_done<on>();
    }
}

/// The link's vector body while a far letter owns UART4; true when it was
/// theirs.
template <bool on = far_letters>
bool far_isr() {
    if constexpr (on) {
        const uint8_t m = far_mode;
        if (m == 1u) {
            if (FarSide<on>::Port::isr()) {
                loop_edges = loop_edges + 1u;
            }
            return true;
        }
        if (m == 2u) {
            if (FarSide<on>::PortLow::isr()) {
                loop_edges = loop_edges + 1u;
            }
            return true;
        }
    }
    return false;
}

/// UART4's two DMA2 channels' body while a far letter owns the port.
template <bool on = far_letters>
bool far_dma_isr() {
    if constexpr (on) {
        const uint8_t m = far_mode;
        if (m == 1u) {
            if (FarSide<on>::Port::dma_isr()) {
                loop_edges = loop_edges + 1u;
            }
            return true;
        }
        if (m == 2u) {
            if (FarSide<on>::PortLow::dma_isr()) {
                loop_edges = loop_edges + 1u;
            }
            return true;
        }
    }
    return false;
}

/// DMA2's channel 1 while letter q's copies run on it.
template <bool on = far_letters>
bool far_copy_vector() {
    if constexpr (on) {
        if (far_copy_mode) {
            if (FarSide<on>::Copier::service() != 0u) {
                far_copies_done = far_copies_done + 1u;
                if (far_copy_loop) {
                    far_copy_once<on>();
                }
            }
            return true;
        }
    }
    return false;
}

/// The far letters, in the second set, on the CH32V303.
template <bool on = far_letters>
void register_far_letters() {
    if constexpr (on) {
        bench.second_letter('n', "UART4'S ENGINES ON DMA2 against the far chip: a round trip, the rate", tn_far_uart4<>);
        bench.second_letter('q', "THE PRIORITY RULE: the far chip pouring into UART4's ring beside an unpaced copy",
                            tq_far_priority<>);
        bench.second_letter('r', "AN OVERRUN AT A MESSAGE'S TAIL on DMA2, the far chip the sender",
                            tr_far_overrun_tail<>);
    }
}

/// USART2's vector body while letter q or r owns the port; true when it
/// was theirs. Compiled only into an image that carries one of them.
[[gnu::always_inline]] inline bool fat_isr() {
    if constexpr (test_letter_carried('q') || test_letter_carried('r')) {
        const uint8_t m = fat_mode;
        if (m == 1u) {
            if (Fat::isr()) {
                loop_edges = loop_edges + 1u;
            }
            return true;
        }
        if (m == 2u) {
            if (FatLow::isr()) {
                loop_edges = loop_edges + 1u;
            }
            return true;
        }
    }
    return false;
}

/// USART2's two channels' body while letter q or r owns the port.
inline bool fat_dma_isr() {
    if constexpr (test_letter_carried('q') || test_letter_carried('r')) {
        const uint8_t m = fat_mode;
        if (m == 1u) {
            if (Fat::dma_isr()) {
                loop_edges = loop_edges + 1u;
            }
            return true;
        }
        if (m == 2u) {
            if (FatLow::dma_isr()) {
                loop_edges = loop_edges + 1u;
            }
            return true;
        }
    }
    return false;
}

/// Channel 3's body while letter q's copies run on it: the block ended,
/// counted, and the next one launched while the loop stands.
inline bool copy_vector() {
    if constexpr (test_letter_carried('q')) {
        if (copy_mode) {
            if (Copier::service() != 0u) {
                copies_done = copies_done + 1u;
                if (copy_loop) {
                    copy_once();
                }
            }
            return true;
        }
    }
    return false;
}

/// A DMA2 channel's handler body, where the part has the controller: the
/// bare channel's counters, or UART4's transport and the far letters' copy
/// engine while a far letter runs.
template <uint8_t ch, uint8_t C = 2>
void dma2_vector() {
    if constexpr (C <= device::dma_controller_count) {
        using S = Dma2State<C>;
        if constexpr (ch == 3u || ch == 5u) {   // UART4's two (table 11-3)
            if (far_dma_isr<>()) {
                return;
            }
        }
        if constexpr (ch == 1u) {   // the far letters' copier
            if (far_copy_vector<>()) {
                return;
            }
        }
        S::note(static_cast<uint8_t>(ch - 1u), DmaChannel<C, ch>::isr());
    }
}

/// UART4's own vector while a far letter's transport owns the port: the
/// receive engine's two states live there (ch32vx03/usart.hpp); its entries
/// counted, and silenced past the budget, as USART2's are.
template <bool on = device::has_usart(4)>
void uart4_vector() {
    if constexpr (on) {
        if (ring_watch) {
            ring_entries = ring_entries + 1u;
            if (ring_entries > ring_budget) {
                ring_stormed = true;
                brio::Usart<on ? 4 : 4>::rxne_interrupt(false);
                return;
            }
        }
        (void)far_isr<>();
    }
}

/// DMA1's eighth channel, the CH32V203's alone: the CH32V303's DMA1 has
/// seven, and its vector table has no entry for an eighth.
template <uint8_t ch = 8>
void dma1_eighth_vector() {
    if constexpr (dma_channel_exists(1, ch)) {
        note(static_cast<uint8_t>(ch - 1u), DmaChannel<1, ch>::isr());
    }
}

// ===========================================================================
// The menu
// ===========================================================================
void banner() {
    print(serial, crlf,
          device::dma_controller_count == 2u
              ? "test_vx03_dma_serial - the USARTs' requests on the two DMA controllers of RM ch. 11"
              : "test_vx03_dma_serial - the USARTs' requests on the DMA controller of RM ch. 11",
          crlf,
          "  one optional jumper: PA2 to PA3 (USART2's own two pads) turns letter g's banged "
          "line into a round trip",
          crlf);
    bench.menu();
}

}  // namespace

// DMA1's vectors, eight on the CH32V203 and seven on the CH32V303.
// Channels 6 and 7 are USART2's, so their bodies serve the transport
// while letters g, p, q and r run and the bare channel the rest of the
// time; channel 3 is the copy engine's while letter q's copies run.
BRIO_CH32_VECTOR(dma1_channel1_handler) { note(0, brio::DmaChannel<1, 1>::isr()); }
BRIO_CH32_VECTOR(dma1_channel2_handler) { note(1, brio::DmaChannel<1, 2>::isr()); }
BRIO_CH32_VECTOR(dma1_channel3_handler) {
    if (!copy_vector()) {
        note(2, brio::DmaChannel<1, 3>::isr());
    }
}
BRIO_CH32_VECTOR(dma1_channel4_handler) { note(3, brio::DmaChannel<1, 4>::isr()); }
BRIO_CH32_VECTOR(dma1_channel5_handler) { note(4, brio::DmaChannel<1, 5>::isr()); }
BRIO_CH32_VECTOR(dma1_channel6_handler) {
    if (fat_dma_isr()) {
        return;
    }
    if (uart_mode) {
        if (Loop::dma_isr()) {
            loop_edges = loop_edges + 1u;
        }
    } else {
        note(5, brio::DmaChannel<1, 6>::isr());
    }
}

BRIO_CH32_VECTOR(dma1_channel7_handler) {
    if (fat_dma_isr()) {
        return;
    }
    if (uart_mode) {
        (void)Loop::dma_isr();
    } else {
        note(6, brio::DmaChannel<1, 7>::isr());
    }
}

BRIO_CH32_VECTOR(dma1_channel8_handler) { dma1_eighth_vector(); }

// DMA2's eleven, the CH32V303's alone: on the other series these bodies
// are empty and nothing in the vector table names them.
BRIO_CH32_VECTOR(dma2_channel1_handler) { dma2_vector<1>(); }
BRIO_CH32_VECTOR(dma2_channel2_handler) { dma2_vector<2>(); }
BRIO_CH32_VECTOR(dma2_channel3_handler) { dma2_vector<3>(); }
BRIO_CH32_VECTOR(dma2_channel4_handler) { dma2_vector<4>(); }
BRIO_CH32_VECTOR(dma2_channel5_handler) { dma2_vector<5>(); }
BRIO_CH32_VECTOR(dma2_channel6_handler) { dma2_vector<6>(); }
BRIO_CH32_VECTOR(dma2_channel7_handler) { dma2_vector<7>(); }
BRIO_CH32_VECTOR(dma2_channel8_handler) { dma2_vector<8>(); }
BRIO_CH32_VECTOR(dma2_channel9_handler) { dma2_vector<9>(); }
BRIO_CH32_VECTOR(dma2_channel10_handler) { dma2_vector<10>(); }
BRIO_CH32_VECTOR(dma2_channel11_handler) { dma2_vector<11>(); }

BRIO_CH32_LEAF_VECTOR(systick_handler) { brio::Ticker::tick(); }
BRIO_CH32_LEAF_VECTOR(usart1_handler) { (void)Serial::isr(); }
BRIO_CH32_LEAF_VECTOR(usart2_handler) {
    if (ring_watch) {
        ring_entries = ring_entries + 1u;
        if (ring_entries > ring_budget) {
            ring_stormed = true;
            brio::Usart<2>::rxne_interrupt(false);
            return;
        }
    }
    if (fat_isr()) {
        return;
    }
    if (Loop::isr()) {
        loop_edges = loop_edges + 1u;
    }
}
BRIO_CH32_LEAF_VECTOR(uart4_handler) { uart4_vector<>(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    bench.letter('g', "the USART's two engines: the receive ring on a banged line, the jumper",
                 tg_engines);
    bench.letter('p', "the receive ring over a channel that never takes the frame", tp_dead_ring);
    // Letters q and r on the CH32V203 board's crossed pair; where UART4 is
    // the pair's link the crossed pair is retired and they take the far chip
    // in the second set.
    if constexpr (crossed_pair_part) {
        bench.letter('q', "the priority rule: an unpaced copy beside the receive ring", tq_priority<>);
        bench.letter('r', "an overrun at a message's tail: no storm, counted once", tr_overrun_tail<>);
    }
    bench.letter('s', "the same owner over the request its release left standing: USART2's, TIM1's",
                 ts_same_owner);
    register_far_letters();

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "PLL144" : "FAILED",
              " tick=", tick_ok ? "STK" : "FAILED", crlf);
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
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        print(serial, "  stack: ", brio::stack_untouched(), " B never touched", crlf);
        bench.prompt();
    }
}
