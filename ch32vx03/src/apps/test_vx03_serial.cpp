// test_vx03_serial - the reference bench suite for the USART chapter (RM
// ch. 18) of the CH32V203 and the CH32V303, beyond the console
// personality: the resource ch32vx03/usart.hpp's `Usart<n>` in every mode
// the chapter has, and the transport `Uart`'s options and engines, on the
// instances the console does not own. Letters a..g and j..l and r are
// both series'; h and i the full USART4's, registered where the part has
// one; m, q and s the CH32V203C8 board's crossed pair's; p the CH32V303RC's
// and VC's upper ports, registered on the part that has them and compiled
// out of every other image; and the TWO-BOARD KEY `y` (below), with n
// (where the link's engines are DMA2's) and o (on the CH32V303 class) in it.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// THE INSTRUMENTS ARE USART2 AND THE FOURTH PORT, and three facts of this
// silicon make almost every letter need NO WIRE AT ALL:
//
//  1. THE RX PAD IS A BIT-BANGED TRANSMITTER. A pad handed to the USART
//     as RX is an input, and an input's level follows its own pull -
//     which the output data register moves in a few cycles (pin.hpp:
//     the pull direction IS the output data bit). At 9600 baud a bit is
//     15000 core cycles, so software paced on the STK puts an ARBITRARY
//     frame on the receive line: every format, a parity that does not
//     add up, a stop bit that is low, a break of ten or eleven bits, an
//     address mark, an RZI frame for the infrared decoder.
//  2. A PAD DRIVEN BY A PERIPHERAL STILL REACHES A CAPTURE INPUT (the
//     timer chapter's finding). USART2's TX pad PA2 is TIM2's channel 3
//     input, so the transmitter's own line is measured by the timer
//     with nothing strapped: a start bit is the divisor to the cycle, a
//     break is its ten or thirteen bits, an IrDA pulse is three
//     sixteenths of a bit, and the stop length is the gap between two
//     frames. USART4's remapped CK pad PA6 is TIM3's channel 1 input
//     the same way, which is how the synchronous clock and the
//     smartcard's card clock are counted.
//  3. THE FOURTH SERIAL PORT IS A USART ON THE CH32V203C8. Chapter 18's
//     opening names that part as the exception whose UART4 is a USART4,
//     with a CK, a CTS and an RTS pad - so the synchronous mode and the
//     smartcard have an instance to run on that is not the console's. On
//     the CH32V303RC and VC the fourth port is a UART4 with TX and RX
//     alone, and the CH32V303CB has none: letters h and i register only
//     where the part has the full USART4.
//
// WIRES ADD LETTERS and each is detected rather than assumed: PA2 to PA3
// (USART2's own loopback, letter l) and the crossed pair of USART2's
// pads with UART4's default ones, PA2-PB1 with PB0-PA3 on the CH32V203C8
// board (letters m, q and s, registered where the part has a fourth port).
// Without them the letters say so and pass nothing. On the CH32V303RC and
// VC the pair is retired - UART4's default pads PC10/PC11 are the desk
// pair's SPI link there (vx03_pair.hpp) - and with it those parts' m, q
// and s, whose subjects the two-board key takes up against the far chip.
//
// THE RATE ON THE WIRE IS TIMED ON THE CORE'S COUNTER. A loop and the
// crossed pair share one bus and one divisor arithmetic at both ends, and
// TIM2 counts a clock of that same bus, so neither can tell a wrong bus
// rate: letter b, the generator's, and every letter that loops or crosses
// a wire (l, m, n, o, q, r, s) or drives an upper port (p) also sends a
// run of frames back to back - through the bare port or the transport -
// timed on the STK, which counts HCLK, from the first store to the last
// stop bit's end, against their time at the rate ASKED, in thousandths:
// never faster than the divisor gives, at most 3 % slower (wire_ok()).
// The letters between them receive a line banged on the STK
// itself or measure one frame's shape. HCLK is the board's crystal x18
// (the HSI's PLL the fallback at the same rates); what no counter on the
// chip can tell - its own clock's error, which moves the wire and the
// ruler together - the two-board key has the far chip time.
//
// THE CH32V303'S OWN LETTERS. UART5..UART8 carry no wire on the evaluation
// board and need none for what letter p asks: the register face, a frame
// read back off the TRANSMIT pad by software (a pad handed to the
// peripheral still reads its level in the input register), a frame
// bit-banged into the RECEIVE pad through its pull, and the vector of
// this class's tail. The pads of those four columns are ports C's and
// D's, and nothing else of this suite touches them.
//
// What is exercised, letter by letter:
//   a  the baud generator: every standard rate from 1200 to 3 Mbaud on
//      both peripheral buses, the divisor, the rate it really gives and
//      the error in per mille, the rates the generator cannot serve
//   b  the frame on the wire, measured by the timer: the start bit at
//      four rates and a run of frames at each timed against the wire,
//      the low run of every word length and parity, all four stop
//      lengths, and TE's idle frame
//   c  the bit-banged line into the receiver: every format received, a
//      parity error, a framing error, an overrun staged and cleared
//   d  mute mode: the receiver asleep through frames until the line
//      idles, or until a nine-bit frame carries its own address
//   e  LIN: the break SBK sends timed on the pad, and the detector's
//      flag and interrupt at ten and eleven bits from the banged line
//   f  half duplex on one wire, and a board with no pull-up on it: the
//      mode's own bits, and the frame and the echo printed rather than
//      judged, the wire being held by nothing between frames
//   g  IrDA: the encoder's pulse in normal and low-power mode measured
//      on the pad, and the decoder fed a banged RZI frame
//   h  THE SYNCHRONOUS MODE on USART4's remapped column: the clock
//      pulses per frame counted on the CK pad, LBCL both ways, CPOL's
//      idle level read with the port driving the pad against the CK
//      node's keeper (PA8, joined to PA6 by the board's SPI1 straps)
//      pulled the other way, CPHA, and the refusals
//   i  the smartcard: SCEN with its NACK, guard time and 1.5 stop bits,
//      and the CARD CLOCK measured on the same CK pad - still, with TE
//      clear, against the keeper
//   j  the flags and the vectors: every source reaching its body on
//      USART2, and UART4's own vector
//   k  hardware flow control: CTS held high stalls the transmitter, RTS
//      rises while a received byte waits and drops when it is read
//   l  the loopback PA2-PA3 when it is strapped: a round trip at every
//      format, the two DMA engines, and four kilobytes at the highest
//      rate the loop takes clean, the transmit engine timed at it
//   m  the crossed pair when it is strapped (the CH32V203C8 board):
//      USART2 against the fourth port at two formats and two rates, with
//      a stress pattern each way and each transmitter timed against the
//      wire
//   q  ERRORS UNDER THE ENGINE, on the crossed pair: breaks the fourth
//      port puts into a continuous stream into USART2's receive ring -
//      every data byte delivered intact and in order, no byte taken by a
//      clear, the frame errors counted one a break; back-to-back breaks
//      counting every other one (ch32vx03/usart.hpp's header); and the
//      interrupt receiver's breaks, dropped, counted and skipped - a
//      reader that looks between them handed every data byte, one that
//      does not handed nothing the ring held with a break behind it;
//      both ends timed against the wire at the stream's rate first
//   r  tx_idle() IS THE WIRE'S: the moment it turns true against the
//      last stop bit's start on PA2, captured by TIM2's channel 3 - never
//      before the stop bit's end, and at the first poll after it - for
//      the interrupt transmitter and the transmit engine, each timed
//      against the wire at 115200 and 1 Mbaud
//   s  THE BURST EDGE FROM THE VECTOR, nothing polled: a burst of one
//      frame told, a burst of 16 told within two frames of its last stop
//      bit with two interrupts a burst and none a byte, four laps of the
//      ring with no silence read whole on the lap's marks; both ends
//      timed against the wire at 1 Mbaud
// and on the CH32V303RC and VC alone:
//   p  UART5..UART8 WITH NO WIRE: the gate, the divisor and the frame
//      read back, a run of frames out of each timed against the wire, a
//      byte read off each TRANSMIT pad by software, a frame
//      banged into each RECEIVE pad through its pull, and each port's
//      own vector reached by a transmission complete - and SW_CFG read
//      with the probe attached, the two columns on the debug port's pads
//      refused by init()
//
// THE TWO-BOARD KEY `y`, letters 1 to 9, outside z: the desk pair's serial
// link - USART3 on PB10/PB11, or UART4 on its second column (PB0/PB1) on
// the CH32V303RC and VC, vx03_link_dut.hpp - as the DUT, the
// far chip running usart_peer, driven in band over the link itself
// (usart_link.hpp), every leg judged from what the far chip reports or from
// what this end received, FAILING loudly when nobody answers; a leg the
// peer REFUSES is SKIPPED with the reason. Both ends on their crystals:
//   1  the rendezvous - ping, IDENT (kind 0x05, the far part's number), a
//      bad checksum, a desync, the refusals - and THE TWO CRYSTALS from
//      both sides: this end's gapless runs timed by the far receiver, the
//      far blast timed here, opposite to the stamps' bound
//   2  the echo ladder 9600 to 4.5 Mbaud, at each rate a gapless run timed
//      by the far receiver against a 1 Mbaud run back to back - PCLK1 over
//      BRR; the peer's blast stating its own rate, and timed here
//   3  the frame matrix echoed, the four stop codes as the far receiver
//      times them (the halves rounded up), the refusals neither chip's
//      vocabulary has
//   4  the receiver's errors from the far engine's drawn trains: PE, FE, a
//      ten-bit break, the sampling census and its three windows, the rate
//      tolerance, the start filter from one pulse, ORE, LIN's detector
//   5  the far engine's shapes, one stretched cell against this end's own
//      pull-banger on the same wire, a LIN break ahead of a frame
//   6  the link's receive engine under a second of the far blast at 1, 2,
//      3 and 4.5 Mbaud, every byte in sequence
//   7  half duplex on the TX pad hearing the far chip's drive_rxd, and the
//      wired AND of an open-drain wire under its collide
//   8  mute by address mark and by idle line, IrDA between two WCH codecs
//   9  the link's receive ring: rx_pending() on the ring and on the
//      interrupt receiver, the error counts under the ring from trains of
//      errored frames the far engine draws (two and twelve bits apart),
//      and the burst edge from the vector - a burst of one, a burst of
//      sixteen at 1 Mbaud told within two frames on one interrupt
// and in the same key, where the link's port is UART4, whose engines are
// DMA2's (the CH32V303RC and VC):
//   n  UART4'S ENGINES ON DMA2, on its second column: a message round the
//      far echo, four kilobytes each way at 921600 with every counter
//      read, the transmit engine timed on this counter and by the far one
// and on every part of the CH32V303 class, whose notes give it the lot's
// registers (ch32vx03/usart.hpp):
//   o  THE LOT'S REGISTERS: CTLR4 probed on USART2 and the link's port and every
//      other register compared across the probe; then - where the die has
//      it - the MARK parity judged by the far receiver; where it has not,
//      CTLR4 and M_EXT stored RAW and both USART2's own pad and the far
//      receiver asked whether the register is absent or only write-only;
//      RX_BUSY polled while the far frame arrives at 1200
//
// THE 32 KB TIER does not hold the suite whole (some 29 KB against the
// CH32V203C6's 28): its four parts build it as two images, letters a
// to j - the generator, the frame and the modes of one port - in one,
// k to s - the flow control, the loops and the pair - in the other (the
// groups line below; design/overview.md, "A suite's image fits the
// family's smallest chip"); none of them is an end of the pair, so no
// digit is in either. The CH32V203C8 builds two images, the single-board
// letters and the two-board key; the CH32V303CB and VC one each. Letter p
// is the CH32V303RC's and VC's and in no group.
//
// build: boards = v203c6,v203c8,v303cb,v303vc
// build: groups = abcdefghij,klmqrs
// build: groups.v203c8 = abcdefghijklmqrs,123456789
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>
#include <span>

#include "ch32vx03/afio.hpp"
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

// The DUT's half of the desk pair's serial link (and usart_link.hpp under
// it): shared with test_vx03_dma_serial and test_vx03_rubato_link.
#include "vx03_link_dut.hpp"

using P = brio::Ch32vx03Platform<>;
/// THE BOARD'S 8 MHz CRYSTAL, x18: the two-board key's reference (each end
/// of the pair on its own crystal), with the HSI's PLL the fallback at the
/// same rates - every divisor and every wait the same either way. No letter
/// of this suite touches the oscillator's pads.
using SysClock = brio::Clock<brio::ClockSource::pll, 144'000'000, 8'000'000>;
using HsiClock = brio::Clock<brio::ClockSource::pll, 144'000'000>;
static_assert(SysClock::hz == HsiClock::hz && SysClock::pclk1_hz == HsiClock::pclk1_hz &&
                  SysClock::pclk2_hz == HsiClock::pclk2_hz,
              "the fallback must run every divisor at the same rate");
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<1, P, 64, 128>;
constexpr Serial serial;

TwoKeyTestBench<Serial, 'y', 32> bench;
/// Whether HCLK runs on the crystal (main()); the two-board key needs it.
bool crystal_ok = false;

// ---- the instruments -------------------------------------------------------

using U2 = Usart<2>;                 ///< TX PA2, RX PA3, CK PA4, CTS PA0, RTS PA1
using TxPad = Pin<'A', 2>;
using RxPad = Pin<'A', 3>;
using CtsPad = Pin<'A', 0>;
using RtsPad = Pin<'A', 1>;
using T2 = Tim<2>;                   ///< the ruler: channel 3 is PA2's input

/// THE FOURTH SERIAL PORT IS NOT ON EVERY PART of this family: the
/// packages below the CH32V203C8 count two usarts and the datasheet's
/// table says which. Where it is there it is a full USART4 - the
/// chapter's own exception - and it is the instrument of the clocked
/// letters; where it is not, the alias falls back to the console's
/// neighbour so that the image still LINKS for every part this suite
/// builds for, and the letters that want the fourth port decline by
/// name instead of measuring something else.
constexpr bool has_fourth = device::has_usart(4);
constexpr uint8_t fourth_instance = has_fourth ? 4 : 2;
using U4 = Usart<fourth_instance>;   ///< the fourth port, where the part has one
/// Whether that fourth port is a FULL USART - the CH32V203C8's alone - and
/// the instance the clocked letters (h, i) run on: the fourth port where
/// it is one, and the console's neighbour otherwise so the image still
/// links for every part; the letters decline by name there.
constexpr bool has_full_fourth = has_fourth && device::usart_full(4);
constexpr uint8_t clocked_instance = has_full_fourth ? 4 : 2;
using Uc = Usart<clocked_instance>;
constexpr uint8_t u4_column = 1;     ///< TX PA5, RX PB5, CK PA6, CTS PA7, RTS PA15
using CkPad = Pin<'A', 6>;           ///< USART4_CK on that column - and TIM3's channel 1
using U4TxPad = Pin<'A', 5>;
using U4RxPad = Pin<'B', 5>;
using T3 = Tim<3>;                   ///< counts the CK pad's transitions
/// UART4's DEFAULT column, the crossed pair's: PB0/PB1 on the CH32V203C8
/// (table 10-27), PC10/PC11 on the CH32V303 (table 10-26). A part with no
/// fourth port keeps the CH32V203C8's two pads, so the image links.
constexpr UsartPads cross_pads =
    has_fourth ? usart_pads_for(4, 0) : UsartPads{Pad{'B', 0}, Pad{'B', 1}};
using CrossTxPad = Pin<cross_pads.tx.port, cross_pads.tx.pin>;
using CrossRxPad = Pin<cross_pads.rx.port, cross_pads.rx.pin>;
/// Whether the crossed pair is retired: UART4's default column is the desk
/// pair's SPI link on this part (vx03_pair.hpp), so m, q and s do not
/// register; and whether they register at all - where the part has a
/// fourth port and keeps the pair.
constexpr bool pair_retired = vx03pair::fourth_default_on_spi_link;
constexpr bool crossed_pair_part = has_fourth && !pair_retired;
static_assert(!crossed_pair_part || (cross_pads.tx == Pad{'B', 0} && cross_pads.rx == Pad{'B', 1}),
              "letter m names the straps PA2-PB1 and PB0-PA3: UART4's default column must be PB0/PB1");
static_assert(!pair_retired || (cross_pads.tx == Pad{'C', 10} && cross_pads.rx == Pad{'C', 11}),
              "a retired pair's wires are named PA2-PC11 and PC10-PA3: UART4's default column PC10/PC11");
/// The fourth port's vector entry follows the DEVICE CLASS's table (the
/// tail differs: device.hpp's Irq table) - a class fact.
constexpr bool v30x_vectors = device::device_class == DeviceClass::v30x_d8;

/// The transport with both engines, for the loopback letter: USART2
/// transmits on DMA channel 7 and receives on 6 (table 11-5).
using Loop = Uart<2, P, 256, 256, UartFormat{}, DmaTxEngine<1, 7>, DmaRxEngine<1, 6>>;
/// The same port with no engine: the interrupt receiver and transmitter.
using Plain = Uart<2, P, 256, 256>;

constexpr uint32_t pclk1 = SysClock::pclk1_hz;    ///< USART2's, USART4's
constexpr uint32_t pclk2 = SysClock::pclk2_hz;    ///< USART1's, the console's
constexpr uint32_t timclk1 = SysClock::timclk1_hz;   ///< what the ruler counts
constexpr uint32_t ticks_per_us = SysClock::hz / 1'000'000UL;

/// Whether letter l's and letter m's straps are on the board. Probed
/// once, by the letters that want them.
bool loop_wire = false;
bool loop_known = false;
bool cross_wires = false;
bool cross_known = false;

/// True while the USART2 vector belongs to the Loop transport.
volatile bool u2_transport = false;
/// Which transport the USART2 vector serves while u2_transport: Loop, or
/// with this set the engineless Plain; and the edges they reported.
volatile bool u2_plain = false;
volatile uint32_t u2_edges = 0;
volatile uint32_t u2_edge_entries = 0;

/// The console vector's entries: what a timing letter reads around its
/// span beside the tick (the lengtheners rule).
volatile uint32_t console_entries = 0;

/// What the two vectors counted (letter j).
volatile uint32_t u2_interrupts = 0;
volatile uint32_t u2_rxne = 0;
volatile uint32_t u2_txe = 0;
volatile uint32_t u2_tc = 0;
volatile uint32_t u2_idle = 0;
volatile uint32_t u2_pe = 0;
volatile uint32_t u2_lbd = 0;
volatile uint32_t u2_cts = 0;
volatile uint16_t u2_last_word = 0;
volatile uint32_t u4_interrupts = 0;

// ---- time ------------------------------------------------------------------

/**
 * THE RULER FOR SOFTWARE: the composed cycle count (util/test_wait.hpp's
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

void console_drain() {
    (void)brio::console_drain<Ruler, Serial>();   // a wedged console shows as missing lines
}

/// Wait for a flag of the instrument, bounded. False on the time-out.
bool wait_flag(uint16_t mask, uint32_t us = 200'000) {
    return wait_until<Ruler>([mask] { return U2::flag(mask); }, us);
}

/// Both of the transmitter's idle flags.
bool tx_settled(uint32_t us = 200'000) {
    return wait_flag(usart_txe, us) && wait_flag(usart_tc, us);
}

// ---- the rate on the wire --------------------------------------------------

/**
 * THE RATE ON THE WIRE, which a loop alone cannot tell: both ends of a
 * loop or of the crossed pair divide one bus, PB1, by divisors from one
 * arithmetic, so a wrong bus rate is a slower or a faster loop and nothing
 * else - and TIM2, the frame ruler, counts a clock of the same bus. The
 * core's STK counts HCLK instead, so `frames` frames of `bits` bit periods
 * written back to back into the BARE port `U` - a store whenever TXE
 * stands - are timed on it from the first store to TC, the last stop bit's
 * end, against their time at the rate ASKED; the answer in thousandths.
 * What the far receiver takes meanwhile is the caller's to drain.
 */
template <typename U>
uint32_t usart_permille(uint32_t baud, uint32_t bits, uint32_t frames) {
    const uint64_t wire = static_cast<uint64_t>(frames) * bits * SysClock::hz / baud;
    const uint64_t budget = 2u * wire + SysClock::hz / 100u;
    // A frame still in flight ends first, waited out by time: TC cannot
    // say so, a write that followed no STATR read leaving it standing from
    // the frame before (measured: a whole frame inside the reading).
    const uint32_t frame = bits * SysClock::hz / baud;
    Stopwatch<Ruler> w;
    w.start();
    while (!U::tx_empty() && w.elapsed() < budget) {
    }
    w.start();
    while (w.elapsed() < 2u * frame) {
    }
    w.start();
    for (uint32_t i = 0; i < frames && w.elapsed() < budget; ++i) {
        while (!U::tx_empty() && w.elapsed() < budget) {
        }
        U::write_data(static_cast<uint8_t>(i * 37u + 11u));
        if (i == 0u) {
            U::clear_flags(usart_tc);   // TC stood for the idle line before the run
        }
    }
    while (!U::tx_complete() && w.elapsed() < budget) {
    }
    const uint32_t took = w.elapsed();
    return wire == 0u ? 0u : static_cast<uint32_t>(1000ULL * took / wire);
}

/// The same through a TRANSPORT `T`: 8N1 frames queued with write_bulk()
/// as fast as its transmit ring takes them, timed from the first queue to
/// tx_idle() - the last stop bit's end (letter r) - against the rate
/// asked. What the transport receives meanwhile is read and thrown away,
/// and its counters are cleared after.
template <typename T>
uint32_t wire_permille(uint32_t baud, uint32_t frames) {
    static uint8_t out[256];
    for (uint32_t i = 0; i < sizeof out; ++i) {
        out[i] = static_cast<uint8_t>(i * 37u + 11u);
    }
    const uint64_t wire = 10ULL * frames * SysClock::hz / baud;
    const uint64_t budget = 2u * wire + SysClock::hz / 100u;
    uint8_t b = 0;
    uint32_t queued = 0;
    Stopwatch<Ruler> w;
    w.start();
    while (queued < frames && w.elapsed() < budget) {
        const uint32_t want = frames - queued;
        queued += T::write_bulk(
            std::span<const uint8_t>(out, want < sizeof out ? want : sizeof out));
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
 * the fourth port, the link's port and UART5..UART8 all do).
 *
 * THE FAST SIDE IS THE DIVISOR'S, derived: the STK counts HCLK and the
 * port divides PCLK1, both out of one oscillator, so the oscillator's error
 * cancels and a bit lasts BRR periods of PCLK1 - HCLK / PCLK1 x BRR core
 * cycles - whatever the crystal or the HSI does. The stopwatch opens before
 * the first store and closes after the flag that ends the run, so the
 * reading is never under 1000 x BRR x baud / PCLK1, rounded down: 998 at
 * 921600 and 460800 (BRR 78 for 78.125, 156 for 156.25), 1001 at 230400
 * (313 for 312.5), 1000 where BRR divides PCLK1 whole.
 *
 * THE SLOW SIDE IS A THRESHOLD, not a model of the path. What it is there
 * to catch is a wrong bus divisor - a bus prescaler, or BRR computed from
 * the other bus - and each of those is a factor of two (a prescaler is a
 * power of two, PCLK2 twice PCLK1): a thousand per mille slow or fast. The
 * path itself adds the frame start's latency (a write to an idle
 * transmitter waits up to a bit for its start, letter b measures it: 6.25
 * per mille of the shortest run timed, 16 frames at 9600) and a poll turn,
 * and a feeder may leave gaps between frames - a transport's block
 * restart, an interrupt entry longer than a frame - which is not this
 * verdict's subject (the transports' own letters judge their bytes and
 * their end, r). Thirty per mille sits between the path's own figure and
 * the next wrong rate, and leaves a feeder's gaps out of a verdict about
 * the divisor.
 */
bool wire_ok(uint32_t permille, uint32_t baud) {
    const uint32_t floor_pm = static_cast<uint32_t>(1000ULL * usart_divisor(pclk1, baud) * baud / pclk1);
    return permille >= floor_pm && permille <= 1030u;
}

// ---- the ruler on the transmit pad -----------------------------------------

/**
 * TIM2 as the width meter of whatever PA2 carries: channel 3 is that
 * pad's input DIRECTLY and channel 4 is mapped INDIRECTLY onto the same
 * input (CC4S = 10, the cross-mapping this family's timers have), so
 * one pad is captured twice with opposite polarities and the difference
 * between the two captures is the width of one level. There is no
 * both-edge capture on this silicon, which is why it takes two
 * channels.
 *
 * `low_first` asks for the width of a LOW run (a start bit, a break, a
 * frame's low head); false measures a HIGH one (a stop gap, an IrDA
 * pulse, whose line idles low).
 */
void ruler_start(uint16_t prescaler, bool low_first) {
    T2::init();
    (void)T2::configure({.prescaler = prescaler, .period = 0xFFFF});
    (void)T2::capture_channel(2, {.select = TimChannelSelect::direct,
                                  .polarity = low_first ? TimCapturePolarity::falling
                                                        : TimCapturePolarity::rising});
    (void)T2::capture_channel(3, {.select = TimChannelSelect::indirect,
                                  .polarity = low_first ? TimCapturePolarity::rising
                                                        : TimCapturePolarity::falling});
    T2::clear_flags(T2::all_flags);
    T2::enable(true);
}

void ruler_rearm() { T2::clear_flags(T2::all_flags); }

/// The width the two captures bracket, in counts; 0 when one of them
/// never came. The counter is sixteen bits, so the caller's prescaler
/// is what keeps a long run inside one lap.
uint32_t ruler_width() {
    if (!T2::flag(T2::compare_flag(2)) || !T2::flag(T2::compare_flag(3))) {
        return 0;
    }
    const uint32_t first = T2::compare(2);
    const uint32_t second = T2::compare(3);
    return (second - first) & 0xFFFFu;
}

/// Wait until both captures have landed, bounded.
bool ruler_wait(uint32_t us = 100'000) {
    return wait_until<Ruler>([] { return T2::flag(T2::compare_flag(2)) && T2::flag(T2::compare_flag(3)); }, us);
}

void ruler_stop() {
    T2::enable(false);
    T2::release();
}

/// Wait for one of the ruler's flags, bounded.
bool ruler_flag(uint16_t mask, uint32_t us = 100'000) {
    return wait_until<Ruler>([mask] { return T2::flag(mask); }, us);
}

/**
 * The FIRST low run after the rearm, read edge by edge instead of off
 * the two latched captures: a capture register holds the LAST event of
 * its polarity, so on a line that is released between frames - a
 * single-wire bus with nothing to pull it up - the pair at the end
 * brackets noise. Here the fall is consumed as it arrives and the rise
 * right after it, so what comes back is one frame's own low run.
 * Zero when either edge never came.
 */
uint32_t ruler_low_run(uint16_t psc) {
    if (!ruler_flag(T2::compare_flag(2))) {
        return 0;
    }
    const uint32_t fall = T2::compare(2);
    if (!ruler_flag(T2::compare_flag(3))) {
        return 0;
    }
    const uint32_t rise = T2::compare(3);
    return ((rise - fall) & 0xFFFFu) * (static_cast<uint32_t>(psc) + 1u);
}

/**
 * The HIGH gap between two frames sent back to back - the stop length,
 * and the one measurement that cannot be read off two latched captures
 * at the end: an idle-high line falls FIRST, so the pair would bracket
 * a frame and not the space between two. The events are taken in their
 * own order instead - the first frame's fall consumed, its rise the
 * gap's start, the next fall its end - each read clearing its own flag.
 * Zero when one of them never came.
 */
uint32_t ruler_gap(uint16_t psc) {
    if (!ruler_flag(T2::compare_flag(2))) {
        return 0;
    }
    (void)T2::compare(2);   // the first frame's start, consumed
    if (!ruler_flag(T2::compare_flag(3))) {
        return 0;
    }
    const uint32_t rise = T2::compare(3);
    if (!ruler_flag(T2::compare_flag(2))) {
        return 0;
    }
    const uint32_t fall = T2::compare(2);
    return ((fall - rise) & 0xFFFFu) * (static_cast<uint32_t>(psc) + 1u);
}

/// A prescaler that keeps `cycles` of the timer's own clock inside one
/// sixteen-bit lap, with room to spare.
uint16_t ruler_prescaler_for(uint32_t cycles) {
    uint32_t psc = 0;
    while (cycles / (psc + 1u) > 50'000UL) {
        ++psc;
    }
    return static_cast<uint16_t>(psc);
}

/// Counts in timer clocks, from a width and the prescaler that measured
/// it - the one unit every number in this suite is printed in.
uint32_t ruler_cycles(uint32_t width, uint16_t prescaler) {
    return width * (static_cast<uint32_t>(prescaler) + 1u);
}

/**
 * Is a width the ruler read, `got` timer clocks, inside `lo`..`hi` to the
 * capture's quantization? THE BOUND, derived: the two edges are latched
 * off ONE input stage - the same pad, the same filter, two channels' edge
 * detectors - each on a count of the prescaled counter, so each lands up
 * to one count after its edge and the width reads within one count of the
 * true one: a count is psc + 1 timer clocks. The transmitter's widths are
 * whole periods of PCLK1 (a bit is BRR of them, the 16x clock's ticks
 * dividing it), and the timer counts twice PCLK1, so the true width is an
 * exact number of timer clocks. What the bound assumes: the pad's own rise
 * and fall differ by less than one timer clock (6.9 ns at 144 MHz) - no
 * output-timing table is cited here - which keeps the reading inside the
 * same one count. Zero (an edge that never came) is never inside.
 */
bool within_count(uint32_t got, uint32_t lo, uint32_t hi, uint16_t psc) {
    const uint32_t count = static_cast<uint32_t>(psc) + 1u;
    return got != 0u && got + count >= lo && got <= hi + count;
}

/// The width of `n` ticks of the 16x clock at divisor `brr`, in timer
/// clocks, as the range a transmitter can put on its pad: sixteen ticks are
/// BRR periods of PCLK1 exactly, so a whole number of bits is one figure,
/// and a part of a bit is `n` ticks of BRR / 16 periods each, rounded down
/// or up - how BRR's four fractional bits spread the remainder over a bit's
/// ticks chapter 18 does not say, and this range assumes nothing about it.
struct TickWidth {
    uint32_t lo;
    uint32_t hi;
};
constexpr TickWidth ticks_width(uint32_t n, uint32_t brr) {
    constexpr uint32_t per = timclk1 / pclk1;
    if (n % 16u == 0u) {
        return {per * brr * (n / 16u), per * brr * (n / 16u)};
    }
    return {per * n * (brr / 16u), per * n * ((brr + 15u) / 16u)};
}

// ---- the bit-banged receive line -------------------------------------------

/// The receive line's level, through the pull of an input pad.
void line(bool high) {
    if (high) {
        RxPad::set();
    } else {
        RxPad::clear();
    }
}

struct Frame {
    uint16_t data = 0;
    uint8_t bits = 8;
    bool parity = false;        ///< send a parity bit at all
    bool parity_value = false;
    uint8_t stop_bits = 1;
    bool bad_stop = false;      ///< a LOW stop bit: the framing error
    uint32_t baud = 9600;
};

/// One frame on the receive line, every edge at an ABSOLUTE offset from
/// the start so that a late store cannot accumulate.
void bang_frame(const Frame& fr) {
    const uint32_t bit = SysClock::hz / fr.baud;
    Stopwatch<Ruler> w;
    w.start();
    uint32_t n = 0;
    line(false);   // the start bit
    ++n;
    for (uint8_t i = 0; i < fr.bits; ++i) {
        while (w.elapsed() < n * bit) {
        }
        line(((fr.data >> i) & 1u) != 0u);
        ++n;
    }
    if (fr.parity) {
        while (w.elapsed() < n * bit) {
        }
        line(fr.parity_value);
        ++n;
    }
    while (w.elapsed() < n * bit) {
    }
    line(!fr.bad_stop);
    while (w.elapsed() < (n + fr.stop_bits) * bit) {
    }
    line(true);
}

bool even_parity(uint16_t v, uint8_t bits) {
    bool p = false;
    for (uint8_t i = 0; i < bits; ++i) {
        p ^= ((v >> i) & 1u) != 0u;
    }
    return p;
}

/// A frame of `f`'s shape carrying `v`, the parity computed: what a peer
/// speaking that format would put on the line.
void bang_word(uint16_t v, const UartFormat& f, uint32_t baud = 9600) {
    const uint8_t bits = static_cast<uint8_t>(f.bits);
    Frame fr{.data = v, .bits = bits, .baud = baud};
    if (f.parity != UartParity::none) {
        fr.parity = true;
        const bool p = even_parity(v, bits);
        fr.parity_value = f.parity == UartParity::even ? p : !p;
    }
    fr.stop_bits = f.stop == UartStop::two ? 2 : 1;
    bang_frame(fr);
}

/// A run of low bits on the receive line: a break of `bits` bit times
/// followed by one stop bit.
void bang_break(uint8_t bits, uint32_t baud = 9600) {
    const uint32_t bit = SysClock::hz / baud;
    Stopwatch<Ruler> w;
    w.start();
    line(false);
    while (w.elapsed() < static_cast<uint32_t>(bits) * bit) {
    }
    line(true);
    while (w.elapsed() < (static_cast<uint32_t>(bits) + 2u) * bit) {
    }
}

/**
 * ONE RZI FRAME on the receive line, for the infrared decoder: a zero
 * bit is a PULSE of three sixteenths of a bit against the idle level
 * and a one bit is the idle level itself. Which way round the pulse
 * goes is the question letter g asks the silicon - `pulse_high` idles
 * the line low and pulses it high, the encoder's own shape, and false
 * is its mirror.
 */
void bang_rzi(uint8_t word, bool pulse_high, uint32_t baud = 9600) {
    const uint32_t bit = SysClock::hz / baud;
    const uint32_t pulse = 3u * bit / 16u;
    const bool idle = !pulse_high;
    line(idle);
    spin_us<Ruler>(2000);
    Stopwatch<Ruler> w;
    w.start();
    uint32_t n = 0;
    line(pulse_high);            // the start bit is a zero: a pulse
    while (w.elapsed() < pulse) {
    }
    line(idle);
    ++n;
    for (uint8_t i = 0; i < 8; ++i) {
        while (w.elapsed() < n * bit) {
        }
        if (((word >> i) & 1u) == 0u) {
            line(pulse_high);
            while (w.elapsed() < n * bit + pulse) {
            }
            line(idle);
        }
        ++n;
    }
    while (w.elapsed() < (n + 1u) * bit) {
    }
}

/// The line idle (high) for `bits` bit times: what wakes a muted
/// receiver under the idle-line rule.
void bang_idle(uint8_t bits, uint32_t baud = 9600) {
    const uint32_t bit = SysClock::hz / baud;
    Stopwatch<Ruler> w;
    w.start();
    line(true);
    while (w.elapsed() < static_cast<uint32_t>(bits) * bit) {
    }
}

// ---- the instrument's life cycle -------------------------------------------

/// The extras put back (defined with letters n..p) - DMA2's two UART4
/// channels, UART5..UART8: nothing on a part without them.
template <bool on = (device::dma_controller_count >= 2u || device::has_usart(5))>
void upper_off();

/// Everything back: both vectors released, both instruments reset and
/// gated off, every pad floating, the two rulers stopped, the remap
/// column back at its reset value.
void all_off() {
    Pfic::disable(Irq::usart2);
    Pfic::disable(Irq::uart4);
    Pfic::disable(dma_channel_irq(1, 6));
    Pfic::disable(dma_channel_irq(1, 7));
    u2_transport = false;
    U2::bus_clock(true);
    U2::reset();
    U2::bus_clock(false);
    if (has_fourth) {
        U4::bus_clock(true);
        U4::reset();
        (void)U4::remap(0);
        U4::bus_clock(false);
    }
    TxPad::release();
    RxPad::release();
    CtsPad::release();
    RtsPad::release();
    CkPad::release();
    U4TxPad::release();
    U4RxPad::release();
    CrossTxPad::release();
    CrossRxPad::release();
    T2::init();
    T2::release();
    T3::init();
    T3::release();
    u2_interrupts = 0;
    u2_rxne = 0;
    u2_txe = 0;
    u2_tc = 0;
    u2_idle = 0;
    u2_pe = 0;
    u2_lbd = 0;
    u2_cts = 0;
    u2_last_word = 0;
    u4_interrupts = 0;
    upper_off();
}

/// USART2 brought up bare on its default column: gate, pads, the frame
/// and the rate, TE and RE, no interrupt. The RX pad takes a pull-up so
/// an idle line reads idle and the bit-banger has a level to move.
bool u2_up(const UartFormat& f, uint32_t baud) {
    U2::bus_clock(true);
    U2::reset();
    TxPad::function();
    RxPad::input(PinPull::up);
    if (!U2::configure(f, usart_divisor(pclk1, baud))) {
        return false;
    }
    U2::enable(true);
    U2::transmitter(true);
    U2::receiver(true);
    spin_us<Ruler>(3000);   // TE's idle frame out before anything is timed
    return true;
}

/// USART4 on its remapped column, with the CK pad handed over: the
/// instrument of the clocked letters.
bool u4_up(const UartFormat& f, uint32_t baud) {
    if (!has_full_fourth) {
        return false;
    }
    Afio::clock_on();
    Uc::bus_clock(true);
    Uc::reset();
    if (!Uc::remap(u4_column)) {
        return false;
    }
    U4TxPad::function();
    U4RxPad::input(PinPull::up);
    return Uc::configure(f, usart_divisor(pclk1, baud));
}

/// Drive one pad and read the other: the strap test every wired letter
/// opens with. Both pads are left floating afterwards.
template <typename Driver, typename Reader>
bool pads_linked() {
    Driver::output();
    Reader::input(PinPull::down);
    Driver::set();
    spin_us<Ruler>(200);
    const bool high = Reader::read();
    Reader::input(PinPull::up);
    Driver::clear();
    spin_us<Ruler>(200);
    const bool low = Reader::read();
    Driver::release();
    Reader::release();
    return high && !low;
}

bool need_loop_wire() {
    if (!loop_known) {
        loop_wire = pads_linked<TxPad, RxPad>();
        loop_known = true;
    }
    return loop_wire;
}

bool need_cross_wires() {
    if (!cross_known) {
        cross_wires = pads_linked<TxPad, CrossRxPad>() && pads_linked<CrossTxPad, RxPad>();
        cross_known = true;
    }
    return cross_wires;
}

// ===========================================================================
// a - the baud generator
// ===========================================================================

constexpr uint32_t standard_bauds[] = {1200,    2400,    4800,    9600,    19200,
                                       38400,   57600,   115200,  230400,  460800,
                                       921600,  1'000'000, 2'000'000, 3'000'000};

/// One row of the table: the divisor, the rate it really gives, and the
/// error in per mille. Returns the error, and 0xFFFF for a rate the
/// generator cannot serve at all.
uint32_t baud_row(uint32_t pclk, uint32_t baud) {
    const uint32_t brr = usart_divisor(pclk, baud);
    if (!usart_divisor_valid(brr)) {
        print(serial, "  ", baud, " at ", pclk / 1'000'000UL, " MHz: divisor ", brr,
              " - REFUSED", crlf);
        return 0xFFFFu;
    }
    const uint32_t got = usart_actual_baud(pclk, brr);
    const uint32_t diff = got > baud ? got - baud : baud - got;
    const uint32_t ppt = diff * 1000UL / baud;
    print(serial, "  ", baud, " at ", pclk / 1'000'000UL, " MHz: divisor ", brr, ", ", got,
          " baud, ", ppt, " per mille", crlf);
    return ppt;
}

void ta_divisor() {
    all_off();
    print(serial, "  PCLK2 = ", pclk2 / 1'000'000UL, " MHz (USART1's), PCLK1 = ",
          pclk1 / 1'000'000UL, " MHz (the other three's)", crlf);

    uint32_t worst2 = 0;
    uint32_t worst1 = 0;
    uint32_t refused2 = 0;
    uint32_t refused1 = 0;
    for (uint32_t baud : standard_bauds) {
        const uint32_t e2 = baud_row(pclk2, baud);
        if (e2 == 0xFFFFu) {
            ++refused2;
        } else if (e2 > worst2) {
            worst2 = e2;
        }
        const uint32_t e1 = baud_row(pclk1, baud);
        if (e1 == 0xFFFFu) {
            ++refused1;
        } else if (e1 > worst1) {
            worst1 = e1;
        }
        console_drain();
    }
    print(serial, "  worst error ", worst2, " per mille on PCLK2 and ", worst1, " on PCLK1; ",
          refused2 + refused1, " rates refused", crlf);
    // 18.3 gives the receiver a tolerance that "shall not be less than
    // 3 per cent": every rate the divisor can reach is well inside it.
    bench.verdict("every standard rate from 1200 to 3 Mbaud that the divisor can reach is "
                  "within 3 per mille of what was asked, on both peripheral buses",
                  worst2 <= 3u && worst1 <= 3u);
    // The register's sixteen bits are the SLOW end's limit and they bite
    // where the fast bus is: 1200 baud from 144 MHz asks for a divisor of
    // 120000. The floor of sixteen clocks is the fast end's, and no
    // standard rate up to 3 Mbaud comes near it.
    bench.verdict("the divisor's own range is what refuses a rate, and it refuses at the SLOW "
                  "end: 1200 baud is out of reach from a 144 MHz bus and within it from the 72 "
                  "MHz one, while every rate up to 3 Mbaud is above the sixteen-clock floor",
                  refused2 == 1u && refused1 == 0u);

    // The floor and the ceiling, as the vocabulary states them.
    const uint32_t too_fast = usart_divisor(pclk1, 8'000'000UL);
    const uint32_t too_slow = usart_divisor(pclk1, 1000);
    bench.verdict("8 Mbaud from a 72 MHz bus asks for a divisor of 9 and is refused, 1000 "
                  "baud one of 72000 and is refused too - sixteen is the floor and 65535 "
                  "the register",
                  !usart_divisor_valid(too_fast) && too_fast == 9u && !usart_divisor_valid(too_slow) &&
                      too_slow == 72000u);

    // And the silicon takes what the arithmetic says.
    const bool up = u2_up(UartFormat{}, 115200);
    const uint32_t brr = U2::brr();
    const uint32_t back = U2::actual_baud(pclk1);
    print(serial, "  USART2 at 115200: BRR reads ", brr, ", the rate it gives is ", back,
          " baud", crlf);
    const bool moved = U2::set_brr(usart_divisor(pclk1, 9600)) && U2::brr() == 7500u;
    bench.verdict("the divisor the arithmetic gives is what the register holds, and "
                  "actual_baud() inverts it",
                  up && brr == 625u && back == 115200u && moved);
    all_off();
}

// ===========================================================================
// b - the frame on the wire
// ===========================================================================

/// Send one byte and measure the width the ruler brackets.
uint32_t measure_frame(uint8_t byte, uint16_t psc) {
    ruler_rearm();
    U2::write_data(byte);
    if (!ruler_wait()) {
        return 0;
    }
    return ruler_cycles(ruler_width(), psc);
}

void tb_frame() {
    all_off();

    // The start bit IS the divisor: BRR peripheral clocks, and the ruler
    // counts the timer's own clock, which is twice PCLK1 here.
    const uint32_t bauds[4] = {9600, 115200, 921600, 3'000'000};
    // A hundredth of a second of frames at least, and 16 at the slowest.
    const uint32_t runs[4] = {16, 256, 1024, 1024};
    uint32_t good = 0;
    uint32_t on_time = 0;
    for (uint8_t k = 0; k < 4u; ++k) {
        const uint32_t baud = bauds[k];
        if (!u2_up(UartFormat{}, baud)) {
            continue;
        }
        const uint32_t brr = U2::brr();
        const uint32_t want = brr * (timclk1 / pclk1);
        const uint16_t psc = ruler_prescaler_for(want);
        ruler_start(psc, true);
        // 0xFF: the start bit alone is low, every data bit is high.
        const uint32_t got = measure_frame(0xFF, psc);
        const uint32_t pm = usart_permille<U2>(baud, 10, runs[k]);
        print(serial, "  ", baud, " baud: the start bit is ", got, " timer clocks for ", want,
              " asked (divisor ", brr, "); ", runs[k], " frames in ", pm,
              " thousandths of their wire time on the core's counter", crlf);
        if (wire_ok(pm, baud)) {
            ++on_time;
        }
        // The start bit is BRR periods of PCLK1, read to the ruler's one
        // count (within_count()).
        if (within_count(got, want, want, psc)) {
            ++good;
        }
        ruler_stop();
        U2::bus_clock(false);
        console_drain();
    }
    bench.verdict("a start bit lasts exactly the divisor: four rates from 9600 to 3 Mbaud "
                  "measured on the transmit pad by the timer, every one to the ruler's one count",
                  good == 4u);
    // TIM2 counts a clock of PB1, the bus the divisor divides, so the
    // start bit above cannot tell a wrong bus rate; the STK counts HCLK.
    bench.verdict("THE LINE RUNS AT THE RATE ASKED: at each of the four rates a run of frames "
                  "back to back takes its wire time at that rate on the core's own counter, "
                  "no faster than its divisor and at most 3 % slower - the timer above counts PB1's clock and "
                  "cannot tell a wrong bus",
                  on_time == 4u);

    // The word length: the low run of a zero byte is the start bit plus
    // every data bit plus the parity bit, all of them zero.
    struct FrameCase {
        UartFormat format;
        uint8_t low_bits;
        const char* name;
    };
    const FrameCase cases[4] = {
        {UartFormat{}, 9, "8N1"},
        {UartFormat{UartBits::eight, UartParity::even}, 10, "8E1"},
        {UartFormat{UartBits::seven, UartParity::even}, 9, "7E1"},
        {UartFormat{UartBits::nine, UartParity::none}, 10, "9N1"},
    };
    uint32_t shapes = 0;
    constexpr uint32_t baud = 9600;
    const uint32_t bit_cycles = (timclk1 / pclk1) * usart_divisor(pclk1, baud);
    const uint16_t psc = ruler_prescaler_for(bit_cycles * 14u);
    for (const FrameCase& c : cases) {
        if (!u2_up(c.format, baud)) {
            continue;
        }
        ruler_start(psc, true);
        ruler_rearm();
        U2::write_word(0);
        uint32_t got = 0;
        if (ruler_wait()) {
            got = ruler_cycles(ruler_width(), psc);
        }
        const uint32_t want = c.low_bits * bit_cycles;
        print(serial, "  ", c.name, ": a zero word is ", got, " timer clocks low for ", want,
              " asked (", c.low_bits, " bits)", crlf);
        if (within_count(got, want, want, psc)) {
            ++shapes;
        }
        ruler_stop();
        U2::bus_clock(false);
        console_drain();
    }
    bench.verdict("the word length is the frame's low run: 8N1 nine bits, 8E1 ten, 7E1 nine "
                  "and a nine-bit word ten - the parity bit counted with the data, as M says",
                  shapes == 4u);

    // The stop length: the HIGH gap between two frames back to back.
    const UartStop stops[4] = {UartStop::one, UartStop::half, UartStop::two,
                               UartStop::one_and_half};
    const char* stop_names[4] = {"1", "0.5", "2", "1.5"};
    const uint32_t stop_halves[4] = {2, 1, 4, 3};   // in half bits
    uint32_t gaps = 0;
    uint32_t whole = 0;
    uint32_t rounded = 0;
    for (uint8_t i = 0; i < 4; ++i) {
        UartFormat f{};
        f.stop = stops[i];
        if (!u2_up(f, baud)) {
            continue;
        }
        ruler_start(psc, true);
        ruler_rearm();
        // Two zero words back to back: the high run between them is the
        // stop bits and nothing else.
        U2::write_data(0);
        // The first frame in the shifter within a frame time (1.04 ms at
        // 9600; 5 ms allowed): a port that never frees DATAR measures no gap.
        const bool second = wait_until<Ruler>([] { return U2::tx_empty(); }, 5'000);
        if (second) {
            U2::write_data(0);
        }
        const uint32_t got = second ? ruler_gap(psc) : 0u;
        const uint32_t want = stop_halves[i] * bit_cycles / 2u;
        print(serial, "  the ", stop_names[i], " stop code: the gap is ", got,
              " timer clocks, ", bit_cycles == 0u ? 0u : (2u * got + bit_cycles) / (2u * bit_cycles),
              " whole bits (", want, " asked)", crlf);
        // A whole bit is BRR periods of PCLK1, a half eight ticks of the 16x
        // clock (ticks_width()); each read to the ruler's count.
        const TickWidth asked = ticks_width(8u * stop_halves[i], usart_divisor(pclk1, baud));
        if (within_count(got, asked.lo, asked.hi, psc)) {
            ++gaps;
        }
        if (i == 0u && within_count(got, bit_cycles, bit_cycles, psc)) {
            ++whole;
        }
        if (i == 2u && within_count(got, 2u * bit_cycles, 2u * bit_cycles, psc)) {
            ++whole;
        }
        if (i == 1u && within_count(got, bit_cycles, bit_cycles, psc)) {
            ++rounded;
        }
        if (i == 3u && within_count(got, 2u * bit_cycles, 2u * bit_cycles, psc)) {
            ++rounded;
        }
        ruler_stop();
        U2::bus_clock(false);
        console_drain();
    }
    bench.verdict("the two WHOLE stop codes are on the wire: the gap between two zero words is "
                  "one bit time under the 1 code and two under the 2 code",
                  whole == 2u);
    // What the two HALF codes put on the wire is a FINDING, not a
    // verdict: 18.10.5 offers the four codes with no restriction, the
    // CH32V203C8T6 and the CH32V303VCT6 send them rounded up to whole
    // bits, the CH32V303CBT6 sends the halves as asked, and nothing in
    // brio computes a frame time from the stop code.
    print(serial, "  FINDING: the half stop codes are ",
          gaps == 4u ? "THE HALVES on the wire" : rounded == 2u ? "ROUNDED UP to whole bits" : "neither halves nor whole",
          crlf);

    // TE's idle frame (18.2): the first byte waits a whole frame.
    U2::bus_clock(true);
    U2::reset();
    TxPad::function();
    (void)U2::configure(UartFormat{}, usart_divisor(pclk1, baud));
    U2::enable(true);
    U2::transmitter(true);
    Stopwatch<Ruler> w;
    w.start();
    U2::write_data(0x55);
    while (!U2::tx_empty() && us_of(w) < 20'000UL) {
    }
    const uint32_t first_us = us_of(w);
    // A second byte written while the first is still in the shift
    // register: TXE cannot come back before the wire is free.
    w.start();
    U2::write_data(0x55);
    while (!U2::tx_empty() && us_of(w) < 20'000UL) {
    }
    const uint32_t busy_us = us_of(w);
    // And one written with the transmitter idle, TC already up.
    (void)tx_settled();
    w.start();
    U2::write_data(0x55);
    while (!U2::tx_empty() && us_of(w) < 20'000UL) {
    }
    const uint32_t idle_us = us_of(w);
    const uint32_t tc_start = us_of(w);
    while (!U2::tx_complete() && us_of(w) < 20'000UL) {
    }
    const uint32_t tc_us = us_of(w) - tc_start;
    print(serial, "  TXE comes back ", first_us, " us after the FIRST write, ", busy_us,
          " us after one to a busy transmitter and ", idle_us,
          " us after one to an idle one; TC then ", tc_us,
          " us later (a frame at 9600 is 1042 us)", crlf);
    // THE BOUNDS, derived: a frame is ten bits of BRR periods of PCLK1,
    // 1041.7 us at 9600, read here as the difference of whole-microsecond
    // stopwatch readings (each rounded down) whose ends are each up to a
    // poll turn - well under a microsecond - from the edge they mark: never
    // under the frame less two microseconds. A byte written to an idle
    // transmitter waits for the generator's next bit tick (the run figures
    // of the rate verdict above show that wait), up to a bit, 104.2 us, and
    // the readings' microsecond.
    const uint32_t frame_us = 10u * usart_divisor(pclk1, baud) / (pclk1 / 1'000'000u);
    const uint32_t bit_us = (usart_divisor(pclk1, baud) + pclk1 / 1'000'000u - 1u) / (pclk1 / 1'000'000u);
    bench.verdict("TE sends an idle frame before the first byte: the first write waits a whole "
                  "frame for TXE, and so does one written while the shift register is busy",
                  first_us + 2u >= frame_us && busy_us + 2u >= frame_us);
    bench.verdict("TXE IS THE SHIFT REGISTER TAKING THE BYTE, not the frame leaving: on an "
                  "idle transmitter it comes back within a bit and TC a whole frame later",
                  idle_us <= bit_us + 1u && tc_us + 2u >= frame_us);
    all_off();
}

// ===========================================================================
// c - the bit-banged line into the receiver
// ===========================================================================

void tc_receive() {
    all_off();
    const UartFormat formats[5] = {
        UartFormat{},
        UartFormat{UartBits::eight, UartParity::even},
        UartFormat{UartBits::eight, UartParity::odd},
        UartFormat{UartBits::seven, UartParity::even},
        UartFormat{UartBits::nine, UartParity::none},
    };
    const char* names[5] = {"8N1", "8E1", "8O1", "7E1", "9N1"};
    const uint16_t words[5] = {0xA5, 0xA5, 0xA5, 0x55, 0x155};
    uint32_t taken = 0;
    for (uint8_t i = 0; i < 5; ++i) {
        if (!u2_up(formats[i], 9600)) {
            continue;
        }
        U2::clear_by_read();
        bang_word(words[i], formats[i]);
        const bool ready = wait_flag(usart_rxne, 5000);
        // The flags BEFORE the data read: STATR then DATAR is the pair
        // that clears them (18.10.1), so a status read after it would say
        // "no flag" whatever the frame raised.
        const uint16_t errors = U2::status() & (usart_pe | usart_fe | usart_ne | usart_ore);
        const uint16_t got = static_cast<uint16_t>(U2::read_word() & uart_data_mask(formats[i]));
        print(serial, "  ", names[i], ": sent ", words[i], ", read ", got, ", flags ", errors,
              crlf);
        if (ready && got == words[i] && errors == 0u) {
            ++taken;
        }
        U2::bus_clock(false);
        console_drain();
    }
    bench.verdict("every frame shape the register can make is received byte for byte from a "
                  "bit-banged line: 8N1, 8E1, 8O1, 7E1 and a nine-bit word, no flag raised",
                  taken == 5u);

    // A parity that does not add up.
    const UartFormat even{UartBits::eight, UartParity::even};
    bool pe_seen = false;
    bool pe_byte = false;
    if (u2_up(even, 9600)) {
        U2::clear_by_read();
        Frame fr{.data = 0xA5, .bits = 8, .parity = true,
                 .parity_value = !even_parity(0xA5, 8), .baud = 9600};
        bang_frame(fr);
        pe_seen = wait_flag(usart_pe, 5000);
        pe_byte = wait_flag(usart_rxne, 5000) && (U2::read_word() & 0xFFu) == 0xA5u;
        U2::bus_clock(false);
    }
    bench.verdict("a frame whose parity bit does not add up raises PE, and the byte is "
                  "delivered anyway - the receiver reports, it does not drop",
                  pe_seen && pe_byte);

    // A stop bit that is low.
    bool fe_seen = false;
    if (u2_up(UartFormat{}, 9600)) {
        U2::clear_by_read();
        Frame fr{.data = 0x5A, .bits = 8, .bad_stop = true, .baud = 9600};
        bang_frame(fr);
        fe_seen = wait_flag(usart_fe, 5000);
        U2::clear_by_read();
        U2::bus_clock(false);
    }
    bench.verdict("a stop bit that is low raises FE, the framing error", fe_seen);

    // An overrun, staged: two frames and no read between them.
    bool ore_seen = false;
    bool ore_cleared = false;
    bool kept = false;
    if (u2_up(UartFormat{}, 9600)) {
        U2::clear_by_read();
        bang_word(0x11, UartFormat{});
        (void)wait_flag(usart_rxne, 5000);
        bang_word(0x22, UartFormat{});
        ore_seen = wait_flag(usart_ore, 5000);
        // 18.10.1: the DATA REGISTER's value is not lost on an overrun,
        // the shift register's is - so the FIRST byte is what comes out.
        kept = (U2::read_word() & 0xFFu) == 0x11u;
        ore_cleared = (U2::status() & usart_ore) == 0u;
        U2::bus_clock(false);
    }
    bench.verdict("a second frame arriving on an unread first raises ORE, the STATR-then-DATAR "
                  "read clears it, and the byte that survives is the one already in the data "
                  "register",
                  ore_seen && kept && ore_cleared);
    all_off();
}

// ===========================================================================
// d - mute mode
// ===========================================================================

void td_mute() {
    all_off();

    // The idle-line wake. 18.10.4's note 1: a byte must have been
    // received before RWU is set, or the wake never comes.
    bool slept = false;
    bool woke = false;
    if (u2_up(UartFormat{}, 9600)) {
        (void)U2::mute_mode({.wake = MuteWake::idle_line});
        U2::clear_by_read();
        bang_word(0x01, UartFormat{});
        (void)wait_flag(usart_rxne, 5000);
        (void)U2::read_word();
        const bool muted = U2::mute() && U2::muted();
        bang_word(0x02, UartFormat{});   // asleep: not received
        slept = muted && !U2::flag(usart_rxne);
        bang_idle(12);                   // the line idle: the wake
        bang_word(0x03, UartFormat{});
        woke = wait_flag(usart_rxne, 5000) && (U2::read_word() & 0xFFu) == 0x03u &&
               !U2::muted();
        U2::bus_clock(false);
    }
    print(serial, "  idle-line wake: the muted receiver ", slept ? "slept through a frame" : "TOOK ONE",
          " and ", woke ? "took the one after the idle" : "STAYED ASLEEP", crlf);
    bench.verdict("under WAKE 0 a muted receiver takes no frame until the line goes idle, and "
                  "the idle clears RWU by itself",
                  slept && woke);

    // The address mark: a nine-bit frame whose MSB is set and whose low
    // four bits are this node's address.
    const UartFormat nine{UartBits::nine, UartParity::none};
    bool ignored_other = false;
    bool took_own = false;
    bool refused_while_ready = false;
    if (u2_up(nine, 9600)) {
        (void)U2::mute_mode({.wake = MuteWake::address_mark, .address = 5});
        U2::clear_by_read();
        (void)U2::mute();
        bang_word(0x107, nine);   // another node's address
        ignored_other = !U2::flag(usart_rxne) && U2::muted();
        bang_word(0x105, nine);   // ours
        took_own = wait_flag(usart_rxne, 5000) && !U2::muted();
        // 18.10.4's note 2: RWU cannot be set under an address mark
        // while RXNE stands.
        refused_while_ready = !U2::mute();
        (void)U2::read_word();
        U2::bus_clock(false);
    }
    print(serial, "  address mark: another node's frame ",
          ignored_other ? "slept through" : "WOKE IT", ", its own ",
          took_own ? "woke it" : "DID NOT", crlf);
    bench.verdict("under WAKE 1 a nine-bit frame whose MSB is set wakes the receiver only when "
                  "its low four bits are this node's address, and RWU is refused while RXNE "
                  "stands - 18.10.4's second note is real",
                  ignored_other && took_own && refused_while_ready);
    all_off();
}

// ===========================================================================
// e - LIN and the break
// ===========================================================================

void te_lin() {
    all_off();
    constexpr uint32_t baud = 9600;
    const uint32_t bit_cycles = (timclk1 / pclk1) * usart_divisor(pclk1, baud);
    const uint16_t psc = ruler_prescaler_for(bit_cycles * 16u);

    // The break SBK sends, outside LIN and inside it.
    uint32_t plain = 0;
    uint32_t in_lin = 0;
    bool in_lin_mode = false;
    if (u2_up(UartFormat{}, baud)) {
        ruler_start(psc, true);
        ruler_rearm();
        U2::clear_flags(usart_tc);
        U2::send_break();
        if (ruler_wait()) {
            plain = ruler_cycles(ruler_width(), psc);
        }
        // The break OUT before the next one is asked for: SBK clears
        // itself at the break's stop bit, and TC says the wire is free.
        (void)wait_flag(usart_tc, 50'000);
        ruler_stop();

        in_lin_mode = U2::lin({}) && U2::lin_enabled();
        ruler_start(psc, true);
        ruler_rearm();
        U2::clear_flags(usart_tc);
        U2::send_break();
        if (ruler_wait()) {
            in_lin = ruler_cycles(ruler_width(), psc);
        }
        (void)wait_flag(usart_tc, 50'000);
        ruler_stop();
        U2::bus_clock(false);
    }
    const uint32_t plain_bits = bit_cycles == 0u ? 0u : (plain + bit_cycles / 2u) / bit_cycles;
    const uint32_t lin_bits = bit_cycles == 0u ? 0u : (in_lin + bit_cycles / 2u) / bit_cycles;
    print(serial, "  the break is ", plain, " timer clocks outside LIN (", plain_bits,
          " bits) and ", in_lin, " inside it (", lin_bits, " bits)", crlf);
    bench.verdict("SBK sends ten bits of low outside LIN mode and thirteen inside it - the "
                  "chapter states the first and the second is this silicon's, measured",
                  in_lin_mode && plain_bits == 10u && lin_bits == 13u);

    // The detector, from the banged line: ten low bits under LBDL 0 and
    // eleven under LBDL 1, with the flag and its interrupt.
    //
    // THE FLAG IS JUDGED BY THE COUNTER AND NOT BY A POLL: LBDIE is
    // armed here, and the handler clears LBD - so what a poll would find
    // is whatever the vector left, while the counter says what happened.
    uint32_t ten = 0;
    uint32_t nine = 0;
    uint32_t eleven = 0;
    uint32_t ten_under_eleven = 0;
    if (u2_up(UartFormat{}, baud)) {
        Pfic::enable(Irq::usart2);
        (void)U2::lin({.break_11bit = false, .break_interrupt = true});
        U2::clear_flags(usart_lbd);
        U2::clear_by_read();

        u2_lbd = 0;
        bang_break(10);
        spin_us<Ruler>(3000);
        ten = u2_lbd;
        U2::clear_by_read();

        u2_lbd = 0;
        bang_break(9);
        spin_us<Ruler>(3000);
        nine = u2_lbd;
        U2::clear_by_read();

        (void)U2::lin({.break_11bit = true, .break_interrupt = true});
        u2_lbd = 0;
        bang_break(11);
        spin_us<Ruler>(3000);
        eleven = u2_lbd;
        U2::clear_by_read();

        u2_lbd = 0;
        bang_break(10);
        spin_us<Ruler>(3000);
        ten_under_eleven = u2_lbd;
        Pfic::disable(Irq::usart2);
        U2::bus_clock(false);
    }
    print(serial, "  the detector raised its vector ", ten, " times for ten low bits, ", nine,
          " for nine, ", eleven, " for eleven under LBDL and ", ten_under_eleven,
          " for ten under LBDL", crlf);
    bench.verdict("the break detector raises LBD once at ten low bits and never at nine, and "
                  "LBDIE carries it to the vector",
                  ten == 1u && nine == 0u);
    bench.verdict("LBDL moves the detector's own length: eleven low bits are a break under it "
                  "and ten are a frame",
                  eleven == 1u && ten_under_eleven == 0u);
    all_off();
}

// ===========================================================================
// f - half duplex with no wire
// ===========================================================================

void tf_half_duplex() {
    all_off();
    constexpr uint32_t baud = 9600;
    const uint32_t bit_cycles = (timclk1 / pclk1) * usart_divisor(pclk1, baud);
    const uint16_t psc = ruler_prescaler_for(bit_cycles * 12u);

    U2::bus_clock(true);
    U2::reset();
    // The one wire is the TX pad, and 18.5 asks for it as an OPEN-DRAIN
    // output against a pull-up on the line - which is what a bus with
    // more than one talker needs. THIS BOARD HAS NO SUCH RESISTOR and
    // this family's pads cannot supply one: 10.2.7 puts the pull-up out
    // of reach in every output mode. Both drives are measured for that
    // reason, and what they show is the state of the wire.
    TxPad::function();
    (void)U2::configure(UartFormat{}, usart_divisor(pclk1, baud));
    const bool armed = U2::half_duplex(true) && U2::half_duplex();
    U2::enable(true);
    U2::transmitter(true);
    U2::receiver(true);
    spin_us<Ruler>(3000);

    // IS THE IDLE LINE HELD? The ruler's two channels are both on this
    // pad, so an edge on a line nobody is driving raises one of their
    // flags. Twenty milliseconds of an idle bus, with nothing written.
    ruler_start(psc, true);
    ruler_rearm();
    spin_us<Ruler>(20'000);
    const bool idle_edges =
        T2::flag(T2::compare_flag(2)) || T2::flag(T2::compare_flag(3));

    // The frame itself, in both drives, and what the instance's own
    // receiver makes of what it sends.
    ruler_rearm();
    U2::clear_by_read();
    U2::write_data(0x00);
    const uint32_t low = ruler_low_run(psc);
    (void)tx_settled();
    const bool heard = wait_flag(usart_rxne, 5000);
    const uint16_t word = heard ? static_cast<uint16_t>(U2::read_word() & 0xFFu) : 0xFFFFu;

    TxPad::function(PinDrive::open_drain);
    spin_us<Ruler>(1000);
    ruler_rearm();
    U2::clear_by_read();
    U2::write_data(0x00);
    const uint32_t open_low = ruler_low_run(psc);
    (void)tx_settled();
    const bool open_heard = wait_flag(usart_rxne, 5000);
    const uint16_t open_word =
        open_heard ? static_cast<uint16_t>(U2::read_word() & 0xFFu) : 0xFFFFu;
    ruler_stop();

    print(serial, "  the idle bus showed ", idle_edges ? "EDGES" : "no edge",
          " in 20 ms with nothing sent", crlf);
    print(serial, "  push-pull: the zero frame measured ", low, " timer clocks low for ",
          9u * bit_cycles, " asked, and the receiver read ", word, crlf);
    print(serial, "  open drain: the same frame measured ", open_low,
          " timer clocks low, and the receiver read ", open_word, crlf);

    bench.verdict("HDSEL is armed and read back, and the mode refuses the company of LIN and "
                  "of the synchronous clock as 18.4 and 18.5 say",
                  armed && !U2::lin({}) && !U2::synchronous({}));
    // WHAT THIS BOARD CANNOT JUDGE. A half-duplex port drives the line
    // only while it transmits, and this family's pads carry no pull in
    // an output mode (10.2.7), so between frames the wire is held by
    // nothing: the idle line shows edges of its own, the frame's low run
    // measures nine bit times in one run and a wandering number in the
    // next, and the byte the instance's own receiver reads back changes
    // with it. The numbers above are this run's and the letter says so
    // rather than judging them.
    (void)idle_edges;
    bench.skip("the single wire wants a pull-up this board has not got: the frame's timing "
               "on the pad and what the instance's own receiver hears of it are printed "
               "above and not judged - a resistor to the supply on PA2 is what would "
               "decide them");
    all_off();
}

// ===========================================================================
// g - IrDA
// ===========================================================================

void tg_irda() {
    all_off();
    constexpr uint32_t baud = 9600;
    const uint32_t bit_cycles = (timclk1 / pclk1) * usart_divisor(pclk1, baud);

    // Normal mode: the pulse is three sixteenths of a bit, and the line
    // idles LOW - so the ruler measures a HIGH run.
    uint32_t normal = 0;
    bool normal_on = false;
    const uint16_t normal_psc = ruler_prescaler_for(bit_cycles);
    if (u2_up(UartFormat{}, baud)) {
        normal_on = U2::irda({.low_power = false, .prescaler = 1}) && U2::irda_enabled();
        const uint16_t psc = normal_psc;
        ruler_start(psc, false);
        ruler_rearm();
        U2::write_data(0xFF);   // every data bit a one: the start bit's pulse alone
        if (ruler_wait()) {
            normal = ruler_cycles(ruler_width(), psc);
        }
        (void)tx_settled();
        ruler_stop();
        U2::irda_off();
        U2::bus_clock(false);
    }
    const uint32_t want_normal = 3u * bit_cycles / 16u;
    print(serial, "  normal mode: the pulse is ", normal, " timer clocks for ", want_normal,
          " asked (three sixteenths of a bit)", crlf);
    // Three ticks of the 16x clock (ticks_width(): 2808 to 2814 timer
    // clocks at 9600, BRR 7500), read to the ruler's count.
    const TickWidth three = ticks_width(3, usart_divisor(pclk1, baud));
    bench.verdict("the SIR encoder's pulse is three sixteenths of a bit in normal mode, "
                  "measured on the transmit pad",
                  normal_on && within_count(normal, three.lo, three.hi, normal_psc));

    // Low-power mode: the pulse is three periods of the PRESCALED clock.
    uint32_t low_power = 0;
    bool lp_on = false;
    constexpr uint8_t lp_prescaler = 200;
    const uint16_t lp_psc = ruler_prescaler_for(3u * lp_prescaler * (timclk1 / pclk1) * 4u);
    if (u2_up(UartFormat{}, baud)) {
        lp_on = U2::irda({.low_power = true, .prescaler = lp_prescaler}) &&
                U2::prescaler() == lp_prescaler;
        const uint16_t psc = lp_psc;
        ruler_start(psc, false);
        ruler_rearm();
        U2::write_data(0xFF);
        if (ruler_wait()) {
            low_power = ruler_cycles(ruler_width(), psc);
        }
        (void)tx_settled();
        ruler_stop();
        U2::irda_off();
        U2::bus_clock(false);
    }
    const uint32_t want_lp = 3u * lp_prescaler * (timclk1 / pclk1);
    print(serial, "  low-power mode with a prescaler of ", lp_prescaler, ": the pulse is ",
          low_power, " timer clocks for ", want_lp, " asked", crlf);
    // Three periods of PCLK1 / PSC, each PSC periods of PCLK1: an exact
    // width, read to the ruler's count.
    bench.verdict("in low-power mode the pulse is three periods of the prescaled clock and no "
                  "longer a fraction of the bit",
                  lp_on && within_count(low_power, want_lp, want_lp, lp_psc));

    // THE DECODER, and the question of which way its pulse goes: 18.7
    // says the level logic between the USART and the SIR block is not
    // the same in the two directions, so the frame is banged both ways
    // and the silicon says which one it reads.
    uint16_t got_high = 0xFFFF;
    uint16_t got_low = 0xFFFF;
    constexpr uint8_t rzi_word = 0x96;
    if (u2_up(UartFormat{}, baud)) {
        (void)U2::irda({.low_power = false, .prescaler = 1});
        U2::clear_by_read();
        bang_rzi(rzi_word, true);
        if (wait_flag(usart_rxne, 5000)) {
            got_high = static_cast<uint16_t>(U2::read_word() & 0xFFu);
        }
        U2::clear_by_read();
        bang_rzi(rzi_word, false);
        if (wait_flag(usart_rxne, 5000)) {
            got_low = static_cast<uint16_t>(U2::read_word() & 0xFFu);
        }
        U2::irda_off();
        U2::bus_clock(false);
    }
    print(serial, "  the decoder read ", got_high, " from a frame whose zeros are HIGH pulses "
          "and ", got_low, " from one whose zeros are LOW pulses (the word was ", rzi_word,
          ")", crlf);
    bench.verdict("THE DECODER'S PULSE IS THE ENCODER'S MIRROR: the receive side takes an "
                  "idle-HIGH line whose zero bits are low pulses, where the transmit side "
                  "puts out high pulses on an idle-low one - 18.7's note that the two "
                  "directions do not share a level logic, measured",
                  got_low == rzi_word && got_high != rzi_word);

    // What IrDA refuses: a prescaler of zero, a stop length other than
    // one, and the company of the other modes (18.7).
    bool refusals = false;
    if (u2_up(UartFormat{}, baud)) {
        const bool zero = !U2::irda({.prescaler = 0});
        UartFormat two_stops{};
        two_stops.stop = UartStop::two;
        (void)U2::configure(two_stops, usart_divisor(pclk1, baud));
        const bool stops = !U2::irda({.prescaler = 1});
        (void)U2::configure(UartFormat{}, usart_divisor(pclk1, baud));
        (void)U2::lin({});
        const bool with_lin = !U2::irda({.prescaler = 1});
        U2::lin_off();
        const bool normal_two = !U2::irda({.low_power = false, .prescaler = 2});
        refusals = zero && stops && with_lin && normal_two;
        U2::bus_clock(false);
    }
    bench.verdict("IrDA refuses a prescaler of zero, a prescaler other than one in normal "
                  "mode, a stop length that is not one, and the company of LIN",
                  refusals);
    all_off();
}

// ===========================================================================
// h - the synchronous mode, on the fourth port's own clock pad
// ===========================================================================

/// TIM3 counting the TRANSITIONS of its channel-1 pad: external clock
/// mode 1 on TI1F_ED, which is a pulse per EDGE - so a clock pulse is
/// two counts.
void ck_counter_arm() {
    T3::init();
    (void)T3::configure({.prescaler = 0, .period = 0xFFFF});
    (void)T3::capture_channel(0, {.select = TimChannelSelect::direct});
    (void)T3::slave({.mode = TimSlaveMode::external_clock1, .trigger = TimTrigger::ti1_edge});
    T3::set_count(0);
}

void ck_counter_start() {
    ck_counter_arm();
    T3::enable(true);
}

uint32_t ck_transitions() { return T3::count(); }

/// THE CK NODE'S KEEPER. On the CH32V203C8 board the CK pad PA6 is one
/// node with PA7 and PA8 (SPI1's loop-back straps), and a pad the port
/// does not drive floats with that node: its level is whatever charge it
/// holds, and a window that counts its transitions counts the node's
/// noise. PA8 is no signal of the fourth port's column, so where the strap
/// joins it to PA6 it is the node's KEEPER - an input pulled one way, a
/// load of some 40 kOhm a push-pull CK drives through unmoved - and a
/// level read on PA6 is then the port's when it differs from the pull's.
using CkKeeper = Pin<'A', 8>;
/// Whether the keeper reaches the CK pad: PA8 driven both ways, PA6 read.
bool ck_keeper_strapped() { return pads_linked<CkKeeper, CkPad>(); }

void ck_counter_stop() {
    T3::enable(false);
    T3::release();
}

/// Eight bytes of 0x5A out of the clocked port and the last one off the
/// wire, every wait bounded: a frame at 9600 is 1.04 ms, each wait allowed
/// 5 ms. False on a bound - a port that stopped shifting.
bool uc_eight() {
    for (uint8_t i = 0; i < 8; ++i) {
        if (!wait_until<Ruler>([] { return Uc::flag(usart_txe); }, 5'000)) {
            return false;
        }
        Uc::write_data(0x5A);
    }
    return wait_until<Ruler>([] { return Uc::flag(usart_tc); }, 5'000);
}

// The instance's own fact, where the letter registers: on this part the
// fourth serial port is a USART4 and not a UART4 - the chapter's own
// exception, and what makes a clock pad exist at all.
static_assert(!has_full_fourth || (Uc::is_full && device::usart_full(4)),
              "letters h and i run on a full USART4");

void th_synchronous() {
    all_off();
    if (!has_fourth) {
        bench.skip("this part has no fourth serial port: the synchronous mode has no instrument here");
        return;
    }
    if constexpr (!has_full_fourth) {
        bench.skip("the fourth port of this part is a UART4 - TX and RX, no CK pad - so the synchronous "
                   "mode has no instrument here");
        return;
    }
    constexpr uint32_t baud = 9600;

    uint32_t without_lbcl = 0;
    uint32_t with_lbcl = 0;
    uint32_t with_cpha = 0;
    bool sent = true;
    bool idle_low = false;
    bool idle_high = false;
    bool armed = false;
    // THE IDLE LEVEL IS READ ONCE THE PORT DRIVES THE PAD - UE and TE set,
    // TE's idle frame out (1.04 ms at 9600) - and against the keeper pulled
    // the OTHER way where the strap joins it, so a pad the port did not
    // drive would read the pull and fail the verdict.
    const bool keeper = ck_keeper_strapped();
    print(serial, "  the CK node's keeper PA8 ", keeper ? "reaches PA6 (the board's strap)" : "is not strapped to PA6",
          crlf);
    if (u4_up(UartFormat{}, baud)) {
        armed = Uc::synchronous({.clock_idle_high = false, .capture_second_edge = false,
                                 .last_bit_clock = false}) &&
                Uc::synchronous_enabled();
        CkPad::function();
        Uc::enable(true);
        Uc::transmitter(true);
        if (keeper) {
            CkKeeper::input(PinPull::up);
        }
        spin_us<Ruler>(2000);
        idle_low = !CkPad::read();
        CkKeeper::release();
        ck_counter_start();
        sent = uc_eight() && sent;
        without_lbcl = ck_transitions();
        ck_counter_stop();

        // LBCL the other way, with the transmitter paused as 18.10.5
        // requires.
        Uc::transmitter(false);
        Uc::receiver(false);
        const bool moved = Uc::synchronous({.clock_idle_high = true, .last_bit_clock = true});
        Uc::transmitter(true);
        if (keeper) {
            CkKeeper::input(PinPull::down);
        }
        spin_us<Ruler>(2000);
        idle_high = CkPad::read();
        CkKeeper::release();
        ck_counter_start();
        sent = uc_eight() && sent;
        with_lbcl = ck_transitions();
        ck_counter_stop();

        // CPHA is the RECEIVER'S capture edge: what the pad puts out
        // must not move with it.
        Uc::transmitter(false);
        Uc::receiver(false);
        const bool phased = Uc::synchronous({.capture_second_edge = true});
        Uc::transmitter(true);
        ck_counter_start();
        sent = uc_eight() && sent;
        with_cpha = ck_transitions();
        ck_counter_stop();
        armed = armed && moved && phased && sent;
    }
    print(serial, "  eight bytes give ", without_lbcl, " transitions with LBCL clear and ",
          with_lbcl, " with it set (", without_lbcl / 16u, " and ", with_lbcl / 16u,
          " pulses a byte)", crlf);
    print(serial, "  the clock pad idles ", idle_low ? "low" : "NOT LOW", " under CPOL 0 and ",
          idle_high ? "high" : "NOT HIGH", " under CPOL 1", crlf);
    // EXACT COUNTS, derived: TIM3 counts every edge of the pad (TI1_ED, both
    // edges), started with the pad at its idle level and stopped after TC,
    // so a pulse is two transitions - eight bytes of seven pulses are 112,
    // of eight 128, and a single count more or less is a pulse or an edge
    // that should not be there.
    bench.verdict("the CK pad carries one pulse per data bit while the transmitter shifts: "
                  "eight bytes of eight bits give eight pulses each with the last bit's clock "
                  "and seven without it - 128 and 112 transitions exactly",
                  armed && without_lbcl == 8u * 7u * 2u && with_lbcl == 8u * 8u * 2u);
    bench.verdict("LBCL reads as the F1 family's and NOT as 18.10.5's words: the bit SET is "
                  "what adds the last data bit's pulse",
                  with_lbcl > without_lbcl);
    bench.verdict("CPOL is the clock's idle level on the pad, high or low with nothing being "
                  "sent - DRIVEN there by the port, the node's keeper pulling the other way",
                  idle_low && idle_high);
    print(serial, "  the same eight bytes under CPHA give ", with_cpha, " transitions", crlf);
    bench.verdict("CPHA is the RECEIVER's capture edge and nothing else: the same eight bytes "
                  "put the same seven pulses each on the pad with it set",
                  with_cpha == without_lbcl);

    // The refusals: the mode wants TE and RE clear, and no other mode on.
    bool live_refused = false;
    bool mode_refused = false;
    if (Uc::enabled()) {
        Uc::transmitter(true);
        live_refused = !Uc::synchronous({}) && !Uc::synchronous_off();
        Uc::transmitter(false);
        Uc::receiver(false);
        (void)Uc::synchronous_off();
        (void)Uc::half_duplex(true);
        mode_refused = !Uc::synchronous({});
        (void)Uc::half_duplex(false);
    }
    bench.verdict("the clock's three bits are refused while the transmitter or the receiver is "
                  "live, and while half duplex is on - 18.4's own conditions",
                  live_refused && mode_refused);

    // AND THE RECEIVER IS NOT FILLED BY THE CLOCK. The chapter says the
    // receiver "will only sample when outputting the clock"; what it
    // still wants is a start bit of its own, so a clocked frame with
    // nothing answering on RX leaves it empty - the sister family's
    // silicon collects a frame of the idle line instead.
    bool empty = false;
    uint16_t status = 0;
    if (Uc::enabled()) {
        (void)Uc::synchronous({});
        Uc::transmitter(true);
        Uc::receiver(true);
        Uc::clear_by_read();
        Uc::write_data(0x33);
        // A frame at 9600 is 1.04 ms; the verdict reads RXNE itself, so
        // the time-out is the outcome it judges.
        (void)wait_until<Ruler>([] { return Uc::flag(usart_rxne); }, 5'000);
        status = Uc::status();
        empty = (status & usart_rxne) == 0u && U4RxPad::read();
        print(serial, "  a clocked byte with nothing on RX left the receiver ",
              (status & usart_rxne) != 0u ? "FULL" : "empty", " (status ", status,
              ", the receive pad reads ", U4RxPad::read() ? 1 : 0, ")", crlf);
    }
    bench.verdict("the clock alone does not fill the receiver: a clocked frame with an idle "
                  "receive line raises no RXNE, the receiver wanting a start bit of its own "
                  "whatever the CK pad is doing",
                  empty);
    all_off();
}

// ===========================================================================
// i - the smartcard
// ===========================================================================

void ti_smartcard() {
    all_off();
    if (!has_fourth) {
        bench.skip("this part has two usarts and no fourth: the smartcard has no instrument here");
        return;
    }
    if constexpr (!has_full_fourth) {
        bench.skip("the fourth port of this part is a UART4 - no CK pad for a card's clock - so the "
                   "smartcard has no instrument here");
        return;
    }
    constexpr uint32_t baud = 9600;
    constexpr uint8_t card_prescaler = 6;   // the clock is PCLK1 / (2 x PSC)
    constexpr uint8_t guard = 16;
    // The timed window: under the kernel tick's period, and a whole
    // divisor of half a second, so the rate is the count times a constant.
    constexpr uint32_t card_window_us = 800;
    static_assert(card_window_us < 1000u && 500'000u % card_window_us == 0u,
                  "the card-clock window fits between two ticks and divides half a second");
    const bool keeper = ck_keeper_strapped();
    print(serial, "  the CK node's keeper PA8 ", keeper ? "reaches PA6 (the board's strap)" : "is not strapped to PA6",
          crlf);

    bool armed = false;
    bool stops = false;
    bool guarded = false;
    uint32_t transitions = 0;
    uint32_t armed_transitions = 0;
    uint32_t sending_transitions = 0;
    uint32_t bracket = 0;
    bool window_quiet = false;
    if (u4_up(UartFormat{UartBits::eight, UartParity::even}, baud)) {
        armed = Uc::smartcard({.nack = true, .guard_time = guard,
                               .clock_prescaler = card_prescaler}) &&
                Uc::smartcard_enabled();
        // The verb sets 1.5 stop bits, which is what 18.6 recommends.
        stops = (Uc::regs().CTLR2 & usart_stop_mask) == usart_ctlr2_stop(UartStop::one_and_half);
        guarded = Uc::guard_time() == guard && Uc::prescaler() == card_prescaler;

        // THE CARD CLOCK, which the mode's own verb turns on: 18.6 says
        // its waveform "has nothing to do with the communication" and
        // only provides the clock for the card. The pad is the caller's,
        // and WHEN that clock actually runs is the question - counted
        // three times, with the transmitter off, with it on, and with a
        // frame going out.
        Uc::transmitter(false);
        Uc::receiver(false);
        CkPad::function();
        Uc::enable(true);
        // The keeper holds the node while the port does not drive it: a
        // still pad counts no transition then, and a running clock still
        // counts every one (the keeper's 40 kOhm is no load to it).
        if (keeper) {
            CkKeeper::input(PinPull::down);
        }
        ck_counter_start();
        spin_us<Ruler>(2000);
        transitions = ck_transitions();
        ck_counter_stop();
        CkKeeper::release();

        Uc::transmitter(true);
        spin_us<Ruler>(1000);
        // THE TIMED WINDOW, the rate's verdict: the console drained first,
        // then the window opened right after a kernel tick and closed
        // before the next - card_window_us under the tick's 1000 us, so a
        // quiet window is the rule and not luck (a window of 2 ms always
        // holds a tick). The tick and the console's vector are still
        // counted around it; a window either moved is printed and taken
        // again (up to four times), for the Hz printed - the verdict below
        // brackets the window itself and holds either way.
        for (uint8_t attempt = 0; attempt < 4u; ++attempt) {
            if (!brio::console_drain<Ruler, Serial>()) {
                break;
            }
            const uint32_t edge = P::now();
            if (!wait_ticks<P>([edge] { return P::now() != edge; }, 2u)) {
                break;
            }
            const uint32_t console0 = console_entries;
            const TickWitness<P> tick;
            ck_counter_arm();
            const uint32_t opened = Ruler::now();
            T3::enable(true);
            spin_us<Ruler>(card_window_us);
            armed_transitions = ck_transitions();
            bracket = Ruler::now() - opened;
            ck_counter_stop();
            window_quiet = tick.quiet() && console_entries == console0;
            if (window_quiet) {
                break;
            }
            print(serial, "  a card-clock window of ", armed_transitions,
                  " transitions lengthened by an interrupt entry - not judged, taken again", crlf);
        }

        ck_counter_start();
        Uc::write_data(0x5A);
        (void)wait_flag(usart_tc, 50'000);
        sending_transitions = ck_transitions();
        ck_counter_stop();
    }
    // Two transitions a period, over the timed window's card_window_us.
    const uint32_t want_hz = pclk1 / (2u * card_prescaler);
    const uint32_t got_hz = armed_transitions * (500'000u / card_window_us);
    print(serial, "  the card clock counted ", transitions,
          " transitions in 2 ms with the transmitter off, ", armed_transitions,
          " in ", card_window_us, " us with it on (", got_hz, " Hz for ", want_hz, " asked) and ",
          sending_transitions, " while a frame went out", crlf);
    bench.verdict("the smartcard's own verb writes SCEN with its NACK, the guard time and the "
                  "prescaler, and sets the 1.5 stop bits 18.6 asks for",
                  armed && stops && guarded);
    // THE BOUND, derived both ways from the window itself. The counter is
    // enabled after `opened` is sampled and read before the bracket's
    // closing read, and spin_us() returns only once its own ruler, read
    // first after the enable, has seen 800 us pass: the counter runs at
    // least 115200 core cycles and at most `bracket`. The card clock is
    // PCLK1 / (2 x PSC), both edges counted: a transition every 12 core
    // cycles exactly, the timer and the card clock out of one oscillator,
    // so a window of W cycles holds W / 12 transitions rounded down or up.
    // Low side: 9600, less one for the enable's store reaching the timer a
    // few bus cycles after it issues (under one transition's 12). High
    // side: bracket / 12, plus one. An interrupt inside the window
    // lengthens both the count and the bracket alike, so the verdict holds
    // for a lengthened window too; the quiet one is sought only for the Hz
    // printed above. Measured 9606 and 9607 (72 to 95 cycles of overrun) on
    // the CH32V203C8T6.
    constexpr uint32_t cycles_per_transition = SysClock::hz / (pclk1 / card_prescaler);
    static_assert(SysClock::hz % (pclk1 / card_prescaler) == 0u, "a transition is a whole number of cycles");
    const uint32_t floor_transitions = card_window_us * ticks_per_us / cycles_per_transition - 1u;
    print(serial, "  the window bracketed at ", bracket, " core cycles: ", floor_transitions, " to ",
          bracket / cycles_per_transition + 1u, " transitions", crlf);
    bench.verdict("the card clock is the peripheral clock divided by TWICE the prescaler and it "
                  "runs free, whether or not anything is being sent - its transitions in the "
                  "window between 800 us and the bracket's span, a twelfth of a cycle count each",
                  armed_transitions >= floor_transitions &&
                      armed_transitions * cycles_per_transition <= bracket + cycles_per_transition);
    if (keeper) {
        bench.verdict("but only with the TRANSMITTER enabled: with TE clear the pad stays still, the "
                      "node's keeper holding it - no transition in 2 ms",
                      transitions == 0u);
    } else {
        bench.skip("the TE-clear window: nothing holds the CK node on this board, and a floating pad "
                   "counts its own noise");
    }

    // The refusals. The clock has to come off first: LIN and the clock
    // exclude each other, so a program leaving the smartcard hands the
    // pad back before it asks for anything else.
    bool refused = false;
    if (Uc::enabled()) {
        Uc::transmitter(false);
        Uc::receiver(false);
        const bool zero = !Uc::smartcard({.clock_prescaler = 0});
        const bool over = !Uc::smartcard({.clock_prescaler = 32});
        Uc::smartcard_off();
        const bool clock_off = Uc::synchronous_off();
        const bool lin_on = Uc::lin({});
        const bool with_lin = !Uc::smartcard({.clock_prescaler = 4});
        Uc::lin_off();
        refused = zero && over && clock_off && lin_on && with_lin;
    }
    bench.verdict("the smartcard refuses a prescaler outside one to thirty-one and the company "
                  "of LIN, and its clock comes off with the synchronous half's own verb",
                  refused);
    all_off();
}

// ===========================================================================
// j - the flags and the vectors
// ===========================================================================

void tj_flags() {
    all_off();
    constexpr uint32_t baud = 9600;
    if (!u2_up(UartFormat{}, baud)) {
        bench.verdict("USART2 comes up", false);
        return;
    }
    Pfic::enable(Irq::usart2);

    // TXE and TC, in that order.
    U2::clear_flags(usart_tc);
    U2::interrupts(usart_txeie | usart_tcie, true);
    u2_txe = 0;
    u2_tc = 0;
    U2::write_data(0x55);
    (void)tx_settled();
    spin_us<Ruler>(2000);
    const uint32_t txe = u2_txe;
    const uint32_t tc = u2_tc;
    U2::interrupts(usart_txeie | usart_tcie, false);
    print(serial, "  the vector ran ", txe, " times for TXE and ", tc, " for TC", crlf);
    bench.verdict("TXE and TC both reach the vector, TXE when the shift register takes the "
                  "byte and TC when the frame is out",
                  txe >= 1u && tc >= 1u);

    // RXNE and IDLE: a byte, then a line that goes quiet.
    U2::rxne_interrupt(true);
    U2::idle_interrupt(true);
    u2_rxne = 0;
    u2_idle = 0;
    U2::clear_by_read();
    bang_word(0x5A, UartFormat{});
    spin_us<Ruler>(4000);
    bang_idle(12);
    spin_us<Ruler>(2000);
    const uint32_t rxne = u2_rxne;
    const uint32_t idle = u2_idle;
    const uint16_t word = u2_last_word;
    print(serial, "  RXNE ", rxne, " times (last word ", word, "), IDLE ", idle, crlf);
    bench.verdict("RXNE carries the byte to the vector and IDLE rises once when the line goes "
                  "quiet after it - 18.10.1's note, IDLE not set again until RXNE is",
                  rxne == 1u && word == 0x5Au && idle == 1u);

    // PE with its own enable.
    U2::rxne_interrupt(false);
    U2::idle_interrupt(false);
    U2::bus_clock(false);
    bool parity_vector = false;
    if (u2_up(UartFormat{UartBits::eight, UartParity::even}, baud)) {
        Pfic::enable(Irq::usart2);
        U2::parity_interrupt(true);
        U2::rxne_interrupt(true);
        u2_pe = 0;
        U2::clear_by_read();
        Frame fr{.data = 0x3C, .bits = 8, .parity = true,
                 .parity_value = !even_parity(0x3C, 8), .baud = baud};
        bang_frame(fr);
        spin_us<Ruler>(4000);
        parity_vector = u2_pe >= 1u;
        U2::parity_interrupt(false);
        U2::rxne_interrupt(false);
    }
    bench.verdict("PEIE carries a parity error to the vector", parity_vector);

    // The fourth port's own vector, which is the device class's tail.
    if (!has_fourth) {
        bench.skip("the fourth serial port's vector is not this part's to reach: it has no fourth serial port");
        all_off();
        return;
    }
    U4::bus_clock(true);
    U4::reset();
    (void)U4::configure(UartFormat{}, usart_divisor(pclk1, baud));
    U4::enable(true);
    U4::transmitter(true);
    U4::interrupts(usart_tcie, true);
    Pfic::enable(Irq::uart4);
    u4_interrupts = 0;
    U4::clear_flags(usart_tc);
    U4::write_data(0x11);
    spin_us<Ruler>(4000);
    const uint32_t fourth = u4_interrupts;
    print(serial, "  the fourth port's vector ran ", fourth, " times", crlf);
    if constexpr (v30x_vectors) {
        bench.verdict("the fourth serial port's vector is its own - entry 68 of this device "
                      "class's table, and a transmission complete reaches it",
                      fourth >= 1u);
    } else {
        bench.verdict("the fourth serial port's vector is its own - entry 61 of this device "
                      "class's table, and a transmission complete reaches it",
                      fourth >= 1u);
    }
    all_off();
}

// ===========================================================================
// k - hardware flow control
// ===========================================================================

void tk_flow() {
    all_off();
    constexpr uint32_t baud = 9600;
    U2::bus_clock(true);
    U2::reset();
    TxPad::function();
    RxPad::input(PinPull::up);
    // CTS is an input the CPU drives through its pull; RTS is an output
    // the CPU reads on the pad.
    CtsPad::input(PinPull::up);          // high: NOT clear to send
    RtsPad::function();
    (void)U2::configure(UartFormat{}, usart_divisor(pclk1, baud));
    const bool pair = U2::flow_control(true, true) && U2::rts_enabled() && U2::cts_enabled();
    U2::enable(true);
    U2::transmitter(true);
    U2::receiver(true);
    spin_us<Ruler>(3000);

    // CTS high holds the frame back: TC does not come.
    U2::clear_flags(usart_tc);
    U2::write_data(0xA5);
    spin_us<Ruler>(4000);
    const bool held = !U2::flag(usart_tc);
    CtsPad::clear();                     // low: clear to send
    const bool released = wait_flag(usart_tc, 20'000);
    print(serial, "  CTS high ", held ? "held the frame back" : "DID NOT HOLD IT",
          "; low let it out ", released ? "at once" : "NOT AT ALL", crlf);
    bench.verdict("CTS sampled high holds the next frame in the data register - TC never "
                  "comes - and a low level lets it out",
                  pair && held && released);

    // RTS: low while the receiver can take a frame, high while a byte
    // waits unread.
    U2::clear_by_read();
    spin_us<Ruler>(1000);
    const bool ready_low = !RtsPad::read();
    bang_word(0x7E, UartFormat{});
    (void)wait_flag(usart_rxne, 5000);
    spin_us<Ruler>(500);
    const bool busy_high = RtsPad::read();
    (void)U2::read_word();
    spin_us<Ruler>(500);
    const bool free_again = !RtsPad::read();
    print(serial, "  RTS reads ", ready_low ? "low" : "HIGH", " with the receiver free, ",
          busy_high ? "high" : "LOW", " with a byte waiting, ", free_again ? "low" : "HIGH",
          " once it is read", crlf);
    bench.verdict("RTS is the receiver's own hand: low while it can take a frame, high while "
                  "a received byte waits unread, low again when it is taken",
                  ready_low && busy_high && free_again);

    // The CTS flag and its interrupt: a CHANGE of the line raises it,
    // and the body disarms the enable so a standing source cannot spin
    // the vector - which is why each change is armed on its own.
    Pfic::enable(Irq::usart2);
    U2::clear_flags(usart_cts);
    u2_cts = 0;
    U2::cts_interrupt(true);
    CtsPad::set();
    spin_us<Ruler>(2000);
    const uint32_t on_high = u2_cts;
    U2::clear_flags(usart_cts);
    U2::cts_interrupt(true);
    CtsPad::clear();
    spin_us<Ruler>(2000);
    const uint32_t on_low = u2_cts;
    U2::cts_interrupt(false);
    Pfic::disable(Irq::usart2);
    print(serial, "  the CTS flag reached the vector ", on_high, " time(s) on the rise and ",
          on_low, " in all after the fall", crlf);
    bench.verdict("a change on the CTS line raises its flag and CTSIE carries it to the "
                  "vector, in both directions",
                  on_high >= 1u && on_low > on_high);
    all_off();
}

// ===========================================================================
// l - the loopback PA2 to PA3
// ===========================================================================

/// The xorshift `brio stress` and every other suite of this project
/// generate, so a pattern is comparable across targets.
uint32_t xorshift(uint32_t& state) {
    uint32_t x = state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    state = x;
    return x;
}

void tl_loopback() {
    all_off();
    if (!need_loop_wire()) {
        bench.skip("the strap PA2 to PA3 is ABSENT: the loopback letter measures nothing");
        return;
    }
    print(serial, "  the strap PA2 to PA3 is in place", crlf);

    // A round trip at every format the resource can make.
    const UartFormat formats[4] = {
        UartFormat{},
        UartFormat{UartBits::eight, UartParity::odd},
        UartFormat{UartBits::seven, UartParity::even},
        UartFormat{UartBits::nine, UartParity::none},
    };
    const uint16_t words[4] = {0xC3, 0xC3, 0x43, 0x1C3};
    uint32_t round_trips = 0;
    for (uint8_t i = 0; i < 4; ++i) {
        if (!u2_up(formats[i], 115200)) {
            continue;
        }
        U2::clear_by_read();
        U2::write_word(words[i]);
        const bool got = wait_flag(usart_rxne, 20'000);
        const uint16_t back =
            static_cast<uint16_t>(U2::read_word() & uart_data_mask(formats[i]));
        if (got && back == words[i]) {
            ++round_trips;
        }
        U2::bus_clock(false);
    }
    bench.verdict("every frame shape goes out of the transmitter and comes back into the "
                  "receiver over the strap, byte for byte",
                  round_trips == 4u);

    // The two DMA engines, on the channels table 11-5 gives USART2.
    u2_transport = true;
    const bool opened = Loop::init(clock, 115200);
    static const uint8_t message[] = "the channel is the request";
    constexpr uint8_t length = sizeof(message) - 1u;
    Loop::clear_errors();
    (void)Loop::write_bulk(std::span<const uint8_t>(message, length));
    Stopwatch<Ruler> w;
    w.start();
    uint8_t got = 0;
    uint8_t back[length] = {};
    while (got < length && us_of(w) < 50'000UL) {
        uint8_t b = 0;
        while (got < length && Loop::read_byte(b)) {
            back[got++] = b;
        }
    }
    bool same = got == length;
    for (uint8_t i = 0; i < length && same; ++i) {
        same = back[i] == message[i];
    }
    print(serial, "  the engines carried ", got, " of ", length, " bytes in ", us_of(w),
          " us, faults ", Loop::dma_faults(), ", overruns ", Loop::rx_overruns(), crlf);
    bench.verdict("the transmit engine drains the ring by whole blocks and the receive engine "
                  "fills it, read as it lands and identical byte for byte",
                  opened && same && Loop::dma_faults() == 0u);

    // Four kilobytes at the highest rate the loop takes clean.
    constexpr uint32_t run_bytes = 4096;
    constexpr uint32_t fast_baud = 921600;
    (void)Loop::set_baud(SysClock::hz, fast_baud);
    Loop::clear_errors();
    uint32_t tx_state = 0x1234'5678UL;
    uint32_t rx_state = 0x1234'5678UL;
    uint32_t sent = 0;
    uint32_t received = 0;
    uint32_t wrong = 0;
    Stopwatch<Ruler> run;
    run.start();
    while (received < run_bytes && us_of(run) < 2'000'000UL) {
        while (sent < run_bytes && sent - received < 128u) {
            if (!Loop::write_byte(static_cast<uint8_t>(xorshift(tx_state) & 0xFFu))) {
                break;
            }
            ++sent;
        }
        uint8_t b = 0;
        while (Loop::read_byte(b)) {
            if (b != static_cast<uint8_t>(xorshift(rx_state) & 0xFFu)) {
                ++wrong;
            }
            ++received;
        }
    }
    const uint32_t run_us = us_of(run);
    print(serial, "  ", received, " of ", run_bytes, " bytes at ", fast_baud, " baud in ",
          run_us, " us, ", wrong, " wrong, overruns ", Loop::rx_overruns(), "/",
          Loop::hw_overruns(), ", frame errors ", Loop::frame_errors(), crlf);
    bench.verdict("four kilobytes travel the loop at 921600 baud with every byte of the "
                  "xorshift pattern back in order and no error counted",
                  received == run_bytes && wrong == 0u && Loop::frame_errors() == 0u &&
                      Loop::hw_overruns() == 0u);
    // A loop shares one divisor at both ends and cannot tell a wrong one:
    // the transmit engine's run timed against the wire at the rate asked.
    const uint32_t pm = wire_permille<Loop>(fast_baud, 1024);
    print(serial, "  1024 frames through the transmit engine in ", pm,
          " thousandths of their wire time at ", fast_baud, " baud (the divisor 78 gives 923077)",
          crlf);
    bench.verdict("and the loop runs at the rate asked: 1024 frames through the transmit engine "
                  "in their wire time at 921600 baud, no faster than its divisor and at most 3 % slower",
                  wire_ok(pm, fast_baud));
    Loop::release();
    all_off();
}

// ===========================================================================
// m - the crossed pair
// ===========================================================================

/// The receiver's error flags a polled read can see.
constexpr uint16_t rx_error_flags = usart_pe | usart_fe | usart_ne | usart_ore;

/// One port's half of a crossed exchange: `byte` written when its
/// transmitter takes it (20 ms allowed - a frame at 19200 is 0.6 ms) and
/// the frame awaited at the other port (20 ms), the receiver's flags taken
/// BEFORE the data read - STATR then DATAR is the pair that clears them
/// (18.10.1) - and OR-ed into `flags`. The word read, or nothing on a
/// bound.
template <typename From, typename To>
std::optional<uint16_t> cross_one(uint8_t byte, uint16_t& flags) {
    if (!wait_until<Ruler>([] { return From::tx_empty(); }, 20'000)) {
        return std::nullopt;
    }
    From::write_data(byte);
    if (!wait_until<Ruler>([] { return To::rx_ready(); }, 20'000)) {
        return std::nullopt;
    }
    flags = static_cast<uint16_t>(flags | (To::status() & rx_error_flags));
    return To::read_word();
}

/// One direction of the crossed pair, polled: `count` xorshift bytes out
/// of `from` and into `to`, the receiving end's flags gathered into
/// `flags` frame by frame. The wrong bytes, or 0xFFFF when the run stopped
/// short.
uint32_t cross_run(uint32_t count, bool from_u2, uint16_t& flags) {
    uint32_t state = 0x2468'1357UL;
    uint32_t check = 0x2468'1357UL;
    uint32_t wrong = 0;
    uint32_t got = 0;
    Stopwatch<Ruler> w;
    w.start();
    for (uint32_t i = 0; i < count && us_of(w) < 2'000'000UL; ++i) {
        const uint8_t byte = static_cast<uint8_t>(xorshift(state) & 0xFFu);
        const std::optional<uint16_t> word = from_u2 ? cross_one<U2, U4>(byte, flags)
                                                     : cross_one<U4, U2>(byte, flags);
        if (!word) {
            break;
        }
        if ((*word & 0xFFu) != (xorshift(check) & 0xFFu)) {
            ++wrong;
        }
        ++got;
    }
    return got == count ? wrong : 0xFFFFu;
}

void tm_cross() {
    all_off();
    if (!has_fourth) {
        bench.skip("this part has no fourth serial port: the crossed pair has no peer here");
        return;
    }
    if (!need_cross_wires()) {
        if constexpr (pair_retired) {
            bench.skip("the wires PA2-PC11 and PC10-PA3 are ABSENT: the crossed pair measures nothing");
        } else {
            bench.skip("the straps PA2-PB1 and PB0-PA3 are ABSENT: the crossed pair measures nothing");
        }
        return;
    }
    if constexpr (pair_retired) {
        print(serial, "  the wires PA2-PC11 and PC10-PA3 are in place", crlf);
    } else {
        print(serial, "  the straps PA2-PB1 and PB0-PA3 are in place", crlf);
    }

    const UartFormat rounds[2] = {UartFormat{},
                                  UartFormat{UartBits::eight, UartParity::even, UartStop::two}};
    const uint32_t bauds[2] = {115200, 19200};
    const char* names[2] = {"8N1 at 115200", "8E2 at 19200"};
    uint32_t clean = 0;
    uint32_t timed = 0;
    for (uint8_t r = 0; r < 2; ++r) {
        if (!u2_up(rounds[r], bauds[r])) {
            continue;
        }
        // UART4 on its DEFAULT column, which is the crossed pair's.
        U4::bus_clock(true);
        U4::reset();
        CrossTxPad::function();
        CrossRxPad::input(PinPull::up);
        (void)U4::configure(rounds[r], usart_divisor(pclk1, bauds[r]));
        U4::enable(true);
        U4::transmitter(true);
        U4::receiver(true);
        spin_us<Ruler>(3000);
        U2::clear_by_read();
        U4::clear_by_read();

        // Each receiver's flags gathered frame by frame, before the read
        // that clears them (cross_one()).
        uint16_t u4_errors = 0;
        uint16_t u2_errors = 0;
        const uint32_t out = cross_run(64, true, u4_errors);
        const uint32_t back = cross_run(64, false, u2_errors);
        // Both ends divide PB1 by one arithmetic, so the pair cannot tell a
        // wrong bus rate: each transmitter's run timed on the core's counter
        // at the rate asked - a start bit, the data, the parity, the stops.
        const uint32_t bits = r == 0u ? 10u : 12u;
        const uint32_t frames = r == 0u ? 256u : 32u;
        const uint32_t pm2 = usart_permille<U2>(bauds[r], bits, frames);
        const uint32_t pm4 = usart_permille<U4>(bauds[r], bits, frames);
        spin_us<Ruler>(2000);
        U2::clear_by_read();
        U4::clear_by_read();
        print(serial, "  ", names[r], ": 64 bytes out with ", out, " wrong, 64 back with ",
              back, " wrong; flags ", u2_errors, " and ", u4_errors, "; ", frames,
              " frames in ", pm2, " and ", pm4, " thousandths of their wire time", crlf);
        if (out == 0u && back == 0u && u2_errors == 0u && u4_errors == 0u) {
            ++clean;
        }
        if (wire_ok(pm2, bauds[r]) && wire_ok(pm4, bauds[r])) {
            ++timed;
        }
        U4::bus_clock(false);
        U2::bus_clock(false);
        console_drain();
    }
    bench.verdict("USART2 and the fourth port talk to each other over the crossed pair at two "
                  "formats and two rates, every xorshift byte arriving in order with no error "
                  "flag on either side",
                  clean == 2u);
    bench.verdict("and both run at the rate asked: each transmitter's frames back to back in "
                  "their wire time at 115200 and at 19200, no faster than the divisor and at most 3 % slower",
                  timed == 2u);
    all_off();
}

// ===========================================================================
// The CH32V303's own letters (n..p)
// ===========================================================================
//
// Every name below hangs on a template parameter and every body is
// `if constexpr` on the part's facts, so a part without UART4 on DMA2, the
// lot's registers or UART5..UART8 forms none of it and carries none of
// it - its state included, which lives in templates for the same reason.

/// What the DMA2 vectors serve while letter n runs, and what the four
/// upper vectors counted (letter p). Named as V303State<on> by templates
/// whose `on` is true only where the part has what they ask - the one
/// specialization the CH32V303 forms.
template <bool on>
struct V303State {
    static inline volatile bool engines = false;
    static inline volatile uint32_t upper_interrupts[4] = {};
    /// Letter p's readings, one a port, in thousandths of the wire.
    static inline uint32_t upper_permille[4] = {};
};

// ===========================================================================
// q, r, s - the transport's promises on the crossed pair
// ===========================================================================

/// The rate cross_break() times its low run at.
uint32_t cross_baud = 115200;
/// Raised when a wait of the stimulus on its own transmitter ran out - a
/// sender that stopped shifting; cleared by cross_sender_up(), and folded
/// into the verdicts of the letters that send through it.
bool cross_stalled = false;

/// The sender's last frame off the wire (TC), bounded: 20 ms against a
/// frame of 87 us at the slowest rate the letters use.
void cross_drain() {
    if (!wait_until<Ruler>([] { return U4::tx_complete(); }, 20'000)) {
        cross_stalled = true;
    }
}

/// The fourth port bare on its default column - the crossed pair's - as
/// the stimulus: TE alone, written by the CPU. Its own STATR is polled,
/// which is a different instance's and arms no clear of USART2's.
bool cross_sender_up(uint32_t baud) {
    cross_baud = baud;
    cross_stalled = false;
    U4::bus_clock(true);
    U4::reset();
    CrossTxPad::function();
    if (!U4::configure(UartFormat{}, usart_divisor(pclk1, baud))) {
        return false;
    }
    U4::enable(true);
    U4::transmitter(true);
    spin_us<Ruler>(3000);
    return true;
}

/// A byte into the stream: written as soon as the data register is free,
/// so bytes follow back to back with no idle frame between them. The wait
/// bounded at 20 ms (a frame is 87 us at 115200); on the bound the byte is
/// not written and the stall is raised.
void cross_byte(uint8_t b) {
    if (!wait_until<Ruler>([] { return U4::tx_empty(); }, 20'000)) {
        cross_stalled = true;
        return;
    }
    U4::write_data(b);
}

/// A break into the stream: after the frame in flight (TC), the fourth
/// port's TX pad taken as a plain output and held low for twelve bit
/// times, then high for one, then handed back. NOT SBK: measured on the
/// CH32V203C8T6, the transmitter sends the byte it sent last AGAIN after
/// an SBK break, so every SBK put a repeated byte into the stream - a
/// fact of the sender, not of the receiver this letter judges.
void cross_break() {
    cross_drain();
    const uint32_t bit = SysClock::hz / cross_baud;
    CrossTxPad::output();
    CrossTxPad::clear();
    Stopwatch<Ruler> w;
    w.start();
    while (w.elapsed() < 12u * bit) {
    }
    CrossTxPad::set();
    while (w.elapsed() < 13u * bit) {
    }
    CrossTxPad::function();
}

template <typename T>
bool transport_up(bool plain, uint32_t baud) {
    // The block through its reset line first: a transport's init() writes
    // CTLR3 only where it has something to put there, so the engines'
    // DMAR, DMAT and EIE would outlive the transport that set them.
    Pfic::disable(Irq::usart2);
    U2::bus_clock(true);
    U2::reset();
    u2_plain = plain;
    u2_transport = true;
    const bool ok = T::init(clock, baud);
    spin_us<Ruler>(2000);
    uint8_t b = 0;
    while (T::read_byte(b)) {
    }
    T::clear_errors();
    u2_edges = 0;
    u2_edge_entries = 0;
    return ok;
}

constexpr uint8_t stream_byte(uint32_t i) { return static_cast<uint8_t>(i * 151u + 7u); }

void tq_errors() {
    all_off();
    if (!has_fourth || !need_cross_wires()) {
        bench.skip("the crossed pair is absent: the errors under the engine measure nothing");
        return;
    }
    constexpr uint32_t n = 120;
    constexpr uint32_t every = 8;
    const bool up = transport_up<Loop>(false, 115200) && cross_sender_up(115200);
    // The two ends' rates first, which the stream alone cannot tell: the
    // sender's frames and the transport's own transmit engine timed.
    const uint32_t pm4 = usart_permille<U4>(115200, 10, 256);
    const uint32_t pm2 = wire_permille<Loop>(115200, 256);
    print(serial, "  at 115200: the fourth port's 256 frames in ", pm4, " and USART2's engine's in ",
          pm2, " thousandths of their wire time", crlf);
    bench.verdict("both ends run at the rate asked: the sender's frames and the transport's "
                  "transmit engine's in their wire time at 115200, no faster than the divisor and at most 3 % "
                  "slower",
                  wire_ok(pm4, 115200) && wire_ok(pm2, 115200));
    (void)transport_up<Loop>(false, 115200);
    uint32_t breaks = 0;
    for (uint32_t i = 0; i < n; ++i) {
        cross_byte(stream_byte(i));
        if (i % every == every - 1u && i + 1u < n) {
            cross_break();
            ++breaks;
        }
    }
    cross_drain();
    spin_us<Ruler>(2000);
    uint32_t data = 0;
    uint32_t zeros = 0;
    uint32_t got = 0;
    bool in_order = true;
    for (;;) {
        const auto run = Loop::read_span();
        if (run.empty()) {
            break;
        }
        for (const uint8_t b : run) {
            ++got;
            if (b == stream_byte(data)) {
                ++data;
            } else if (b == 0u) {
                ++zeros;
            } else {
                in_order = false;
            }
        }
        (void)Loop::consume(static_cast<uint32_t>(run.size()));
    }
    const uint16_t fe = Loop::frame_errors();
    print(serial, "  ", n, " data bytes and ", breaks, " breaks through the receive engine: ", data,
          " data bytes in order, ", zeros, " break frames, ", got, " stored; FE ", fe, ", ORE ",
          Loop::hw_overruns(), ", laps missed ", Loop::rx_overruns(), cross_stalled ? "; THE SENDER STALLED" : "",
          crlf);
    bench.verdict("every data byte delivered, intact and in order: no byte taken by a clear",
                  up && !cross_stalled && data == n && in_order);
    bench.verdict("each break stored as the frame it is (0x00)", zeros == breaks);
    bench.verdict("the frame errors counted one a break", fe == breaks);

    // Back to back, in the middle of a stream.
    uint16_t counts[2] = {0, 0};
    for (uint8_t k = 0; k < 2u; ++k) {
        (void)transport_up<Loop>(false, 115200);
        cross_byte(0x11);
        cross_byte(0x12);
        for (uint8_t b = 0; b < k + 2u; ++b) {
            cross_break();
        }
        cross_byte(0x22);
        cross_byte(0x23);
        spin_us<Ruler>(2000);
        counts[k] = Loop::frame_errors();
    }
    print(serial, "  back to back: two breaks count ", counts[0], ", three count ", counts[1], crlf);
    bench.verdict("back-to-back breaks count every other one (two count 1, three count 2)",
                  counts[0] == 1u && counts[1] == 2u);
    Loop::release();

    // The interrupt receiver drops each break's frame, counts it and tells
    // its ring, whose next look skips what it holds: read between the
    // breaks, every data byte is handed out ...
    (void)transport_up<Plain>(true, 115200);
    breaks = 0;
    uint32_t back = 0;
    bool ok = true;
    uint8_t b = 0;
    auto read_all = [&] {
        cross_drain();
        spin_us<Ruler>(100);   // the last frame's stop bit and its entry
        while (Plain::read_byte(b)) {
            if (b != stream_byte(back)) {
                ok = false;
            }
            ++back;
        }
    };
    const uint32_t skips0 = Plain::rx_skips();
    for (uint32_t i = 0; i < 40u; ++i) {
        cross_byte(stream_byte(i));
        if (i % 4u == 3u) {
            read_all();   // the four before the break
            cross_break();
            ++breaks;
            read_all();   // the skip, with nothing queued
        }
    }
    const uint32_t skips = (Plain::rx_skips() - skips0) & 0xFFu;
    print(serial, "  the interrupt receiver, read between: 40 bytes and ", breaks, " breaks, ",
          back, " delivered, FE ", Plain::frame_errors(), ", skips ", skips, crlf);
    bench.verdict("the interrupt receiver drops each break's frame, counts it and skips it: "
                  "a reader that looks between the breaks is handed every data byte",
                  !cross_stalled && back == 40u && ok && Plain::frame_errors() == breaks && skips == breaks);
    // ... and read after, nothing the ring held with a break behind it:
    // no run joins the two sides.
    const uint32_t skips1 = Plain::rx_skips();
    for (uint32_t i = 0; i < 8u; ++i) {
        cross_byte(stream_byte(40u + i));
        if (i == 3u) {
            cross_break();
        }
    }
    spin_us<Ruler>(2000);
    uint32_t joined = 0;
    while (Plain::read_byte(b)) {
        ++joined;
    }
    const uint32_t skips_after = (Plain::rx_skips() - skips1) & 0xFFu;
    print(serial, "  read after: 8 bytes around a break, ", joined, " delivered, FE ",
          Plain::frame_errors(), ", skips ", skips_after, crlf);
    bench.verdict("bytes queued with a break among them are skipped whole, in one skip",
                  joined == 0u && skips_after == 1u && Plain::frame_errors() == breaks + 1u);
    Plain::release();
    all_off();
}

/// One reading of letter r, every figure in timer clocks after the last
/// stop bit's rising edge on PA2: `clocks` where tx_idle() was first seen
/// true, `before` where the turn before it - the last that saw it false -
/// read the timer; and whether tx_idle() came at all.
struct IdleReading {
    int32_t clocks = 0;
    int32_t before = 0;
    bool seen = false;
};

/**
 * Four 0x55 frames (bit 7 low: the stop bit starts with a rising edge)
 * through `T`, TIM2's channel 3 latching PA2's last rising edge, and the
 * timer's count read at EVERY turn of the wait, before its tx_idle(): the
 * turn that sees it true reads the count again after it.
 *
 * THE VERDICT IT FEEDS, derived without a model of the turn: the stop bit
 * ends `bit` timer clocks after its rising edge (TIM2 counts twice PCLK1,
 * a bit is BRR periods of PCLK1), and TC rises at that end, at most one
 * tick of the 16x clock later - the transmitter's state machine steps on
 * that clock, a sixteenth of a bit (chapter 18 states no closer figure).
 * So the count read after the true answer is never under `bit` (the flag
 * stood when STATR was read, the count after it), and the count read at
 * the start of the turn before - whose own STATR, read after that count,
 * still said no - is under `bit` plus that sixteenth: tx_idle() answered
 * on the first turn whose STATR load came after the stop bit's end. An
 * interrupt served in either turn lengthens the turn and moves neither
 * statement, so the reading is judged whether or not it was quiet; how
 * long the last turn took is printed (`clocks` - `before`).
 */
template <typename T>
IdleReading idle_after_stop(bool plain, uint32_t baud) {
    static const uint8_t frames[4] = {0x55, 0x55, 0x55, 0x55};
    (void)transport_up<T>(plain, baud);
    T2::init();
    (void)T2::configure({.prescaler = 0, .period = 0xFFFF});
    (void)T2::capture_channel(2, {.select = TimChannelSelect::direct,
                                  .polarity = TimCapturePolarity::rising});
    T2::clear_flags(T2::all_flags);
    T2::enable(true);
    IdleReading out{};
    if (brio::console_drain<Ruler, Serial>()) {
        uint32_t t_before = T2::count();
        (void)T::write_bulk(std::span<const uint8_t>(frames, sizeof(frames)));
        uint32_t t_idle = 0;
        // Four frames at 115200 are 0.35 ms; 20 ms allowed.
        out.seen = wait_until<Ruler>(
            [&t_idle, &t_before] {
                const uint32_t stamp = T2::count();
                if (T::tx_idle()) {
                    t_idle = T2::count();
                    return true;
                }
                t_before = stamp;
                return false;
            },
            20'000);
        const uint32_t edge = T2::compare(2);
        // Both within a few hundred microseconds of the edge, on a counter
        // that laps in 455: a signed sixteen-bit difference.
        out.clocks = static_cast<int16_t>(static_cast<uint16_t>(t_idle - edge));
        out.before = static_cast<int16_t>(static_cast<uint16_t>(t_before - edge));
    }
    ruler_stop();
    T::release();
    return out;
}

/// A transport's transmit run at `baud`, brought up and released around
/// the reading.
template <typename T>
uint32_t transport_permille(bool plain, uint32_t baud, uint32_t frames) {
    (void)transport_up<T>(plain, baud);
    const uint32_t pm = wire_permille<T>(baud, frames);
    T::release();
    return pm;
}

void tr_tx_idle() {
    all_off();
    static constexpr uint32_t rates[] = {115200, 1'000'000};
    for (const uint32_t baud : rates) {
        const uint32_t frames = baud == 115200u ? 256u : 1024u;
        const uint32_t pm_plain = transport_permille<Plain>(true, baud, frames);
        const uint32_t pm_dma = transport_permille<Loop>(false, baud, frames);
        print(serial, "  ", baud, " baud: ", frames, " frames in ", pm_plain,
              " thousandths of their wire time through the interrupt transmitter, ", pm_dma,
              " through the transmit engine", crlf);
        bench.verdict("both transmitters run at the rate asked, no faster than the divisor and at most 3 % slower: ",
                      baud == 115200 ? "115200" : "1 Mbaud", wire_ok(pm_plain, baud) && wire_ok(pm_dma, baud));
        const uint32_t bit = timclk1 / baud;
        const IdleReading r_plain = idle_after_stop<Plain>(true, baud);
        const IdleReading r_dma = idle_after_stop<Loop>(false, baud);
        print(serial, "  ", baud, " baud, a bit ", bit, " timer clocks: tx_idle() true ", r_plain.clocks,
              " clocks after the stop bit's rising edge, the turn before at ", r_plain.before,
              " (interrupt transmitter); ", r_dma.clocks, ", the turn before at ", r_dma.before,
              " (transmit engine)", crlf);
        // idle_after_stop()'s derivation: the true answer's count never
        // under the stop bit, the false turn's count under the stop bit and
        // a sixteenth of a bit - so tx_idle() turned true at the first poll
        // after the wire went idle, however long that poll's turn was.
        const int32_t lo = static_cast<int32_t>(bit);
        const int32_t hi_before = static_cast<int32_t>(bit + bit / 16u);
        bench.verdict("tx_idle() never before the last stop bit is out, and true at the first poll after it: ",
                      baud == 115200 ? "115200" : "1 Mbaud",
                      r_plain.seen && r_dma.seen && r_plain.clocks >= lo && r_plain.before <= hi_before &&
                          r_dma.clocks >= lo && r_dma.before <= hi_before);
    }
    all_off();
}

/// Everything Loop holds, read away: the count, and whether each was the
/// stream's byte at its place from `from` on.
uint32_t loop_read(uint32_t from, bool& in_order) {
    uint32_t got = 0;
    for (;;) {
        const auto run = Loop::read_span();
        if (run.empty()) {
            return got;
        }
        for (uint32_t i = 0; i < run.size(); ++i) {
            if (run[i] != stream_byte(from + got + i)) {
                in_order = false;
            }
        }
        got += static_cast<uint32_t>(run.size());
        (void)Loop::consume(static_cast<uint32_t>(run.size()));
    }
}

void ts_edge() {
    all_off();
    if (!has_fourth || !need_cross_wires()) {
        bench.skip("the crossed pair is absent: the edge measures nothing");
        return;
    }
    const bool up = transport_up<Loop>(false, 1'000'000) && cross_sender_up(1'000'000);
    const uint32_t pm4 = usart_permille<U4>(1'000'000, 10, 1024);
    const uint32_t pm2 = wire_permille<Loop>(1'000'000, 1024);
    print(serial, "  at 1 Mbaud: the fourth port's 1024 frames in ", pm4,
          " and USART2's engine's in ", pm2, " thousandths of their wire time", crlf);
    bench.verdict("both ends run at the rate asked: the sender's frames and the transport's "
                  "transmit engine's in their wire time at 1 Mbaud, no faster than the divisor and at most 3 % "
                  "slower",
                  wire_ok(pm4, 1'000'000) && wire_ok(pm2, 1'000'000));
    (void)transport_up<Loop>(false, 1'000'000);
    const uint32_t frame = SysClock::hz / 100'000u;   // ten bits at 1 Mbaud, in core cycles
    bool all_told = true;
    for (uint8_t k = 0; k < 3u; ++k) {
        bool ok = true;
        (void)loop_read(0, ok);
        const uint32_t e0 = u2_edges;
        cross_byte(0x42);
        spin_us<Ruler>(200);
        const uint32_t e1 = u2_edges;
        const uint32_t got = loop_read(0, ok);
        print(serial, "  a burst of one frame: ", e1 - e0, " edge(s), ", got, " byte read", crlf);
        if (e1 == e0 || got != 1u) {
            all_told = false;
        }
    }
    bench.verdict("a burst of one frame is told, every time", up && all_told);

    // Sixteen frames back to back: the edges, the entries, the last edge
    // after the last stop bit on the cycle counter (U4's TC is the sender's
    // own, a different instance's STATR).
    bool ok = true;
    (void)loop_read(0, ok);
    spin_us<Ruler>(200);
    u2_edge_entries = 0;
    const uint32_t e0 = u2_edges;
    uint32_t got = 0;
    for (uint32_t i = 0; i < 16u; ++i) {
        if (i == 15u) {
            // TC CLEARED BEFORE THE LAST WRITE, with the frame before it
            // still in the shifter (TXE just seen): it cannot rise for that
            // frame, DATAR being full when it ends, and rises at the last
            // frame's stop bit. Cleared after the write instead, it would
            // stay down for good whenever the last frame had already ended.
            if (!wait_until<Ruler>([] { return U4::tx_empty(); }, 20'000)) {
                cross_stalled = true;
            }
            U4::clear_flags(usart_tc);
        }
        cross_byte(stream_byte(i));
        if (u2_edges != e0) {
            got += loop_read(got, ok);   // a first frame's edge: the consumer drains, as it would
        }
    }
    // The last stop bit, two frames at most away (20 us at 1 Mbaud; 1 ms
    // allowed).
    const bool last_out = wait_until<Ruler>([] { return U4::tx_complete(); }, 1'000);
    const uint32_t before = u2_edges;
    const std::optional<uint32_t> told = cycles_until<Ruler>([before] { return u2_edges != before; }, 2'000);
    const uint32_t late = told.value_or(us_cycles<Ruler>(2'000));
    spin_us<Ruler>(200);
    got += loop_read(got, ok);
    print(serial, "  16 frames: ", u2_edges - e0, " edge(s), the last about ", late,
          " cycles after the last stop bit (", late * 10u / frame, " tenths of a frame); ",
          u2_edge_entries, " USART interrupt(s); ", got, " read",
          last_out && !cross_stalled ? "" : "; THE SENDER STALLED", crlf);
    bench.verdict("the burst read whole, nothing polled", got == 16u && ok);
    bench.verdict("its edge within two frame times of the last stop bit",
                  last_out && !cross_stalled && told.has_value() && late <= 2u * frame);
    bench.verdict("at most two USART interrupts a burst, none a byte",
                  u2_edge_entries >= 1u && u2_edge_entries <= 2u);

    // Four laps of the 256-byte ring with no silence: told twice a lap.
    (void)loop_read(0, ok);
    Loop::clear_errors();
    uint32_t read = 0;
    ok = true;
    uint32_t seen = u2_edges;
    for (uint32_t i = 0; i < 1024u; ++i) {
        cross_byte(stream_byte(i));
        if (u2_edges != seen) {
            seen = u2_edges;
            read += loop_read(read, ok);
        }
    }
    Stopwatch<Ruler> tail;
    tail.start();
    while (read < 1024u && us_of(tail) < 5000u) {
        if (u2_edges != seen) {
            seen = u2_edges;
            read += loop_read(read, ok);
        }
    }
    print(serial, "  four laps without silence: ", read, " of 1024 read on the edges, laps missed ",
          Loop::rx_overruns(), crlf);
    bench.verdict("a stream with no silence is read whole on the lap's marks",
                  read == 1024u && ok && Loop::rx_overruns() == 0u);
    Loop::release();
    all_off();
}

template <typename U>
void port_on() {
    U::enable(true);
    U::transmitter(true);
    U::receiver(true);
}

// ===========================================================================
// THE TWO-BOARD KEY `y`: the pair's link to the far chip's usart_peer
// ===========================================================================
//
// The desk's link (docs/boards/weact-ch32v203c8.md, docs/boards/ch32v303-
// evt.md): USART3 on its default column (TX PB10, RX PB11) on the
// CH32V203C8, crossed to UART4 on its SECOND column (TX PB0, RX PB1) on the
// CH32V303 - each the other's DUT and peer. This end drives the far chip
// IN BAND over the link itself (usart_link.hpp); every leg is judged from
// what the far chip reports or from what this end received, FAILING loudly
// when nobody answers; a leg the peer REFUSES is SKIPPED with the reason.
// BOTH ENDS RUN ON THEIR CRYSTALS (8 MHz x18 = 144 MHz, PCLK1 72 MHz), so
// both divide one bus rate by one arithmetic and every rate rung is the
// same divisor at both ends - and what neither divisor can tell, the two
// crystals' disagreement, letter 1 measures from both sides.
//
// THE STAMP BOUND, every span's, is vx03_link_dut.hpp's span_bound_ppm().

/// Whether this part is one end of the pair (vx03_link_dut.hpp): a part
/// without the link's port registers none of the digit letters.
constexpr bool link_present = vx03link::present;
constexpr uint8_t link_n = vx03link::port_n;
constexpr uint8_t link_column = vx03link::column;

namespace two {

/// The DUT's half of the link: the port, its pads and the reviewed command
/// path, shared with test_vx03_dma_serial (vx03_link_dut.hpp).
using D = vx03link::Dut<Serial, SysClock>;
using W = D::W;
using WTx = D::WTx;
using WRx = D::WRx;
using Drain = D::Drain;
using Timed = D::Timed;
constexpr uint32_t wclk = D::wclk;
constexpr uint32_t hclk = D::hclk;
constexpr uint32_t calib_rate = D::calib_rate;
constexpr uint16_t calib_frames = D::calib_frames;

/// A verdict, unless the peer refused the leg: then a SKIP (report() has
/// said which op it refused), counted apart from passes and fails.
void judge(const char* name, bool ok) {
    if (D::leg_refused) {
        bench.skip(name, " (the peer refused the leg)");
        return;
    }
    bench.verdict(name, ok);
}
void judge(const char* name, const char* suffix, bool ok) {
    if (D::leg_refused) {
        bench.skip(name, suffix);
        return;
    }
    bench.verdict(name, suffix, ok);
}

/// ppm printed with its sign.
void print_ppm(int32_t ppm) {
    const uint32_t m = static_cast<uint32_t>(ppm < 0 ? -ppm : ppm);
    print(serial, ppm < 0 ? "-" : "+", m, " ppm");
}

/// Decimal for a verdict label (one call per expression).
char num_buf[12];
const char* num(uint32_t v) {
    uint8_t i = 11;
    num_buf[i] = 0;
    if (v == 0u) num_buf[--i] = '0';
    while (v != 0u && i != 0u) {
        num_buf[--i] = static_cast<char>('0' + (v % 10u));
        v /= 10u;
    }
    return &num_buf[i];
}

/// The start of every letter: this end on its crystal, the peer found and
/// the far chip's, on its own. A crystal that did not start, here or there,
/// is the key's PREMISE absent - a skip with its reason, not a verdict; a
/// peer that does not answer, or answers as something else, is a failure.
bool ready() {
    all_off();
    if (!crystal_ok) {
        bench.skip("this board's crystal did not start: the two-board key needs HCLK on it as the reference");
        return false;
    }
    if (!D::ensure()) {
        bench.verdict("the peer is reachable", false);
        return false;
    }
    link::Ident d{};
    if (!D::ident(d) || !D::far_peer(d)) {
        bench.verdict("the far chip's usart_peer answers", false);
        return false;
    }
    if (d.xtal != 1u) {
        bench.skip("the far chip runs on its HSI, not its crystal: the two-board key needs both as references");
        return false;
    }
    return true;
}

void done() {
    (void)D::command_mode();
    D::release();
}

// ---------------------------------------------------------------------------
// 1 - the rendezvous, and the two crystals from both sides
// ---------------------------------------------------------------------------

void t1_link() {
    all_off();
    bench.verdict("HCLK runs on this board's 8 MHz crystal, x18 (the reference of this key)", crystal_ok);
    const bool found = D::ensure();
    bench.verdict("command mode on the pair's link (8N1, 115200) and the peer answers a ping", found);
    if (!found) {
        D::release();
        return;
    }
    link::Ident d{};
    const bool got = D::ident(d);
    char label[9] = {};
    for (uint8_t i = 0; i < 8u; ++i) label[i] = d.label[i];
    if (got) {
        print(serial, "  peer '", label, "' kind ", hex(link::peer_kind(d.version)), " fw ", hex(d.version),
              " xtal=", d.xtal, " sanity=", hex(d.sanity), crlf);
    }
    bench.verdict("IDENT: the far chip's usart_peer (kind 0x05, its part number for a label), on its crystal",
                  got && D::far_peer(d) && d.xtal == 1u);

    D::flush();
    D::put(link::magic);
    D::put(link::byte_of(link::Op::ping));
    D::put(0);
    D::put(0x00);   // not the checksum
    const bool bad_out = D::wait_tc();
    link::Frame bad;
    bench.verdict("a bad checksum comes back as a NAK",
                  bad_out && D::recv_frame(bad, 120) && bad.op == link::Op::nak);
    for (uint8_t i = 0; i < 12u; ++i) D::put(static_cast<uint8_t>(0x11u * i));
    // The desync's bytes are noise by design; a port that did not shift
    // them fails the recovery verdict below all the same.
    (void)D::wait_tc();
    spin_us<Ruler>(120'000);
    (void)D::command_mode();
    bench.verdict("the link recovers after a desync", D::command(link::Op::ping));

    // THE REFUSAL PATH: onewire (no swap on this block - the peer's
    // transmitter reaches no other wire), a synchronous client (no CK on
    // the link), a clock pad held, a five-bit word: each acked, reported
    // refused, a ping answered after.
    bool refused_all = true;
    for (uint8_t k = 0; k < 4u; ++k) {
        link::Params a{};
        a.cfg.rate = 115'200;
        a.ms = 50;
        link::Op op = link::Op::onewire;
        if (k == 1u) {
            op = link::Op::set_link;
            a.cfg.mode = link::Mode::sync_client;
        } else if (k == 2u) {
            op = link::Op::hold_pad;
            a.value = link::pad_xck;
        } else if (k == 3u) {
            op = link::Op::echo;
            a.cfg.bits = 5;
            a.count = 4;
        }
        link::Report r{};
        refused_all = refused_all && D::act(op, a) && D::report(r, 0) && D::leg_refused &&
                      (r.flags & link::report_ran) == 0u;
    }
    D::leg_refused = false;
    bench.verdict("the peer refuses onewire, a synchronous client, a clock pad and a five-bit word, and "
                  "answers at once after",
                  refused_all && D::command(link::Op::ping));

    // THE TWO CRYSTALS, FROM BOTH SIDES. calibrate: this end's gapless run
    // at 1 Mbaud (a divisor of 72, exact at both ends) stamped by the far
    // receiver against ITS crystal - three runs; then the peer's blast
    // stamped HERE against this one. The same two crystals seen from the
    // two sides: the readings must be opposite, to the stamps' bound.
    int32_t here[3] = {};
    bool runs_ok = true;
    for (int32_t& p : here) {
        const Timed t = D::calibrate();
        runs_ok = runs_ok && t.ok;
        p = t.ppm;
    }
    const Timed back = D::timed_blast(calib_rate, calib_frames);
    const uint32_t b1 = D::span_bound_ppm(calib_rate, calib_frames);   // one span's
    print(serial, "  this crystal against the peer's, three runs: ");
    print_ppm(here[0]);
    print(serial, ", ");
    print_ppm(here[1]);
    print(serial, ", ");
    print_ppm(here[2]);
    print(serial, "; the peer's against this one, timed here: ");
    print_ppm(back.ppm);
    print(serial, " (one span good to ", b1, " ppm); this end's poll turn ", back.turn, " cycles", crlf);
    bench.verdict("the peer times three gapless runs of 4000 frames at 1 Mbaud, and this end one of the peer's",
                  runs_ok && back.ok);
    // THE STAMP BOUND'S PREMISE (vx03_link_dut.hpp's THE TURN): this end's
    // loop measured within half the bound's turn, the peer's longer one
    // then within the whole.
    bench.verdict("the stamps' loop turns as the stamp bound assumes: this end's measured within half its turn",
                  back.ok && back.turn != 0u && back.turn <= D::turn_ceiling);
    // TWO CRYSTALS, an ASSUMPTION: a quartz crystal, 100 ppm with its
    // temperature stability (the boards name no part) - 200 ppm the pair -
    // and the reading good to one span's stamp bound.
    bench.verdict("THE TWO CRYSTALS AGREE within two 100 ppm crystals' tolerance and one span's stamp bound",
                  runs_ok && D::within(here[0], 0, 200u + b1));
    bench.verdict("run to run the reading holds to two spans' stamp bound (the crystals do not move in a second)",
                  runs_ok && D::within(here[0], here[1], 2u * b1) && D::within(here[1], here[2], 2u * b1) &&
                      D::within(here[0], here[2], 2u * b1));
    bench.verdict("AND FROM THE OTHER SIDE THE SAME PAIR: the peer's rate timed here is minus this end's timed "
                  "there, to two spans' stamp bound",
                  runs_ok && back.ok && D::within(here[0], -back.ppm, 2u * b1 + 2u));
    done();
}

// ---------------------------------------------------------------------------
// 2 - the echo ladder, and the rate on the wire timed by the far chip
// ---------------------------------------------------------------------------

uint16_t echo_mismatches(uint16_t n, uint16_t mask) {
    uint16_t bad = 0;
    D::wait_for_window();
    for (uint16_t i = 0; i < n; ++i) {
        const uint16_t v = static_cast<uint16_t>((0x5Au + i * 37u) & mask);
        // A frame at the slowest rung, 9600, is 1 ms; 20 ms allowed, and a
        // byte that could not be sent is a mismatch.
        if (!wait_until<Ruler>([] { return W::tx_empty(); }, 20'000)) {
            ++bad;
            continue;
        }
        W::write_word(v);
        bool ok = false;
        const uint32_t t0 = Ticker::millis();
        while (Ticker::millis() - t0 < 20u) {
            const uint16_t st = W::status();
            if ((st & usart_rxne) != 0u) {
                ok = (W::read_word() & mask) == v && (st & (usart_fe | usart_pe | usart_ne)) == 0u;
                break;
            }
        }
        if (!ok) ++bad;
    }
    return bad;
}

/// Frames for a timed run at `rate`: a fiftieth of a second of wire, 192
/// at least and 9000 at most.
uint16_t span_frames(uint32_t rate) {
    const uint32_t n = rate / 500u;
    return static_cast<uint16_t>(n < 192u ? 192u : (n > 9000u ? 9000u : n));
}

void t2_rates() {
    if (!ready()) {
        done();
        return;
    }
    // Every rung a divisor both ends share; the top is 18.3's floor of 16.
    static constexpr uint32_t rates[] = {9600,      115'200,   921'600,   1'000'000,
                                         2'000'000, 3'000'000, 3'600'000, 4'500'000};
    link::Cfg ref{};
    ref.rate = calib_rate;
    for (const uint32_t rate : rates) {
        link::Cfg c{};
        c.rate = rate;
        link::Params a{};
        a.cfg = c;
        a.count = 16;
        a.ms = D::bound_ms(rate, 32);
        const bool acted = D::act(link::Op::echo, a);
        const bool up = acted && D::configure(c);
        const uint16_t bad = up ? echo_mismatches(16, 0xFFu) : 16u;
        link::Report r{};
        const bool have = D::report(r, 5);
        judge(num(rate), " baud: 16 bytes echoed by the far chip, every one equal, 16 clean frames counted there",
              up && bad == 0u && have && r.count == 16u && r.ferr == 0u && r.perr == 0u && r.ovf == 0u);
        // THE FOREIGN SPAN: a gapless run timed by the far receiver against
        // a 1 Mbaud run back to back, the two crystals' disagreement
        // cancelled between them; what is left is this end's divisor.
        const Timed r0 = D::timed_run(link::Op::sink, ref, 2000);
        const uint16_t n = span_frames(rate);
        const Timed k = D::timed_run(link::Op::sink, c, n);
        const int32_t expect = D::divisor_ppm(rate);
        const uint32_t bound = D::span_bound_ppm(rate, n) + D::span_bound_ppm(calib_rate, 2000);
        print(serial, "  ", rate, ": BRR ", usart_divisor(wclk, rate), ", ", n, " frames, the far chip times ");
        print_ppm(k.ppm - r0.ppm);
        print(serial, " against the divisor's ", expect, " ppm (bound ", bound, ")", crlf);
        judge(num(rate), " baud: the rate on the wire is PCLK1 / BRR to the stamps' bound, timed by the far chip",
              r0.ok && k.ok && D::within(k.ppm - r0.ppm, expect, bound));
    }
    // The peer's own rate: stated in its blast's report from its divisor,
    // and timed here against this crystal - its stated figure plus the
    // two crystals' disagreement (the 1 Mbaud blast's reading).
    {
        constexpr uint32_t rate = 921'600;
        link::Params a{};
        a.cfg.rate = rate;
        a.count = 64;
        a.ms = D::bound_ms(rate, 128);
        const bool acted = D::act(link::Op::blast, a);
        link::Cfg c{};
        c.rate = rate;
        const bool up = acted && D::configure(c);
        const Drain d = up ? D::collect(10) : Drain{};
        link::Report r{};
        const bool have = D::report(r, 5);
        const int32_t stated = static_cast<int16_t>(r.aux16);
        print(serial, "  the peer's blast at 921600: ", d.frames, " frames here, its rate stated ",
              (r.flags & link::report_rate_stated) != 0u ? "" : "NOT ", stated, " ppm, this end's divisor ",
              D::divisor_ppm(rate), crlf);
        judge("the peer STATES its blast's rate (report_rate_stated): PCLK1 / BRR on its crystal, the same "
              "divisor as this end's to the ppm",
              have && d.frames == 64u && (r.flags & link::report_rate_stated) != 0u &&
                  D::within(stated, D::divisor_ppm(rate), 1));
        const Timed b0 = D::timed_blast(calib_rate, 2000);
        const uint16_t n = span_frames(rate);
        const Timed b = D::timed_blast(rate, n);
        const uint32_t bound = D::span_bound_ppm(rate, n) + D::span_bound_ppm(calib_rate, 2000);
        print(serial, "  the peer's blast timed here: ");
        print_ppm(b.ppm - b0.ppm);
        print(serial, " against its stated ", stated, " ppm (bound ", bound, ")", crlf);
        judge("and timed HERE, against a 1 Mbaud blast back to back, the peer's 921600 is the rate it states",
              b0.ok && b.ok && D::within(b.ppm - b0.ppm, stated, bound));
    }
    done();
}

// ---------------------------------------------------------------------------
// 3 - the frame matrix across the wire
// ---------------------------------------------------------------------------

struct FmtCase {
    uint8_t bits;
    uint8_t parity;
    uint8_t stop;
    const char* name;
};

void t3_frames() {
    if (!ready()) {
        done();
        return;
    }
    static constexpr FmtCase cases[] = {
        {8, 0, 1, "8N1"},
        {8, 1, 1, "8E1"},
        {8, 2, 1, "8O1"},
        {7, 1, 1, "7E1"},
        {7, 2, 1, "7O1"},
        {link::bits9_low, 0, 1, "9N1"},
        {8, 0, 2, "8N2"},
        {8, 0, link::stop_half, "8N0.5"},
        {8, 0, link::stop_one_half, "8N1.5"},
    };
    for (const FmtCase& fc : cases) {
        link::Cfg c{};
        c.rate = 115'200;
        c.bits = fc.bits;
        c.parity = fc.parity;
        c.stop = fc.stop;
        link::Params a{};
        a.cfg = c;
        a.count = 8;
        a.ms = D::bound_ms(115'200, 24);
        const bool acted = D::act(link::Op::echo, a);
        const bool up = acted && D::configure(c);
        const uint16_t bad = up ? echo_mismatches(8, D::mask_of(c)) : 8u;
        link::Report r{};
        const bool have = D::report(r, 5);
        if (bad != 0u) {
            print(serial, "    ", fc.name, ": wrong=", bad, " peer count=", r.count, " ferr=", r.ferr, " perr=", r.perr,
                  crlf);
        }
        judge(fc.name, " across the wire, echoed", up && bad == 0u && have && r.count == 8u && r.ferr == 0u &&
                                                       r.perr == 0u);
    }
    // THE STOP LENGTHS AS A FOREIGN RECEIVER TIMES THEM: gapless runs of
    // 2000 frames at 1 Mbaud under the four codes, each span against the
    // 1-stop run's. A frame of 1 stop is 20 half bits and of 2 is 22, so
    // the 2 code reads 1.1 of the 1 code; what the half codes are on the
    // wire - rounded up or the halves - is letter b's finding on this
    // chip's own pad, timed again here by the far chip.
    uint32_t spans[4] = {};
    bool timed_all = true;
    static constexpr uint8_t stops[4] = {1, 2, link::stop_half, link::stop_one_half};
    for (uint8_t i = 0; i < 4u; ++i) {
        link::Cfg c{};
        c.rate = calib_rate;
        c.stop = stops[i];
        const Timed k = D::timed_run(link::Op::sink, c, 2000);
        timed_all = timed_all && k.ok;
        spans[i] = k.span;
    }
    // ppm of a span against another; a span's bound is the stamps'.
    const auto ratio_ppm = [](uint32_t a, uint32_t b) {
        return b == 0u ? 0 : static_cast<int32_t>((static_cast<int64_t>(a) - b) * 1'000'000 / b);
    };
    const uint32_t bound = 2u * D::span_bound_ppm(calib_rate, 2000);
    const int32_t two = ratio_ppm(spans[1], spans[0]);
    const int32_t half = ratio_ppm(spans[2], spans[0]);
    const int32_t one_half = ratio_ppm(spans[3], spans[1]);
    print(serial, "  spans at 1 Mbaud, ticks: 1 stop ", spans[0], ", 2 stops ", spans[1], ", the 0.5 code ", spans[2],
          ", the 1.5 code ", spans[3], crlf);
    print(serial, "  the 2 code against the 1 code: ", two, " ppm (22/20 = +100000); the 0.5 code against the 1: ",
          half, "; the 1.5 code against the 2: ", one_half, " (bound ", bound, ")", crlf);
    bench.verdict("the far receiver times the stop lengths: two stop bits make a frame of 22 half bits against 20",
                  timed_all && D::within(two, 100'000, bound));
    // The half codes as the far receiver times them: a FINDING, as in
    // letter b - rounded up (0 ppm against the whole code's frame) on the
    // CH32V203C8T6 and the CH32V303VCT6, the halves (19 half bits against
    // 20, -50000 ppm; 21 against 22, -45455 ppm) on the CH32V303CBT6.
    const bool far_rounded = D::within(half, 0, bound) && D::within(one_half, 0, bound);
    const bool far_halves = D::within(half, -50'000, bound) && D::within(one_half, -45'455, bound);
    print(serial, "  FINDING: the far receiver times the half codes as ",
          far_halves ? "THE HALVES" : far_rounded ? "ROUNDED UP to whole bits" : "neither halves nor whole", crlf);
    bench.verdict("the far receiver timed every run of the four codes (what the halves are is the finding above)",
                  timed_all);
    // What neither chip has: 5- and 6-bit words, nine data bits with a
    // parity bit - the peer refuses each, and this end's vocabulary too.
    bool refused = true;
    for (const uint8_t bits : {uint8_t{5}, uint8_t{6}, link::bits9_low}) {
        link::Params a{};
        a.cfg.rate = 115'200;
        a.cfg.bits = bits;
        a.cfg.parity = bits == link::bits9_low ? 1u : 0u;
        a.count = 4;
        a.ms = 50;
        link::Report r{};
        refused = refused && D::act(link::Op::echo, a) && D::report(r, 0) && D::leg_refused;
    }
    D::leg_refused = false;
    bench.verdict("the peer refuses 5- and 6-bit words and nine bits with parity, as this end's format does",
                  refused && !uart_format_valid(UartFormat{UartBits::nine, UartParity::even, UartStop::one}));
    done();
}

// ---------------------------------------------------------------------------
// 4 - the receiver's errors, from the far engine's drawn trains
// ---------------------------------------------------------------------------
//
// THE SAMPLING, MEASURED (chapter 18 does not describe it): a census of
// one low pulse 1 % or 3 % of a bit wide walked along a data cell of 0xFF
// frames at 9600 finds THREE sensitive instants a sixteenth apart near the
// cell's middle, the three moving together by up to a sixteenth from one
// leg to the next (the 16x clock runs free of the line, the start edge
// found on its tick) - the first between 37 % and 44 % of the cell, the
// last 12.5 % later. A pulse over one or two of them raises NE; which way
// the bit then goes the census does not settle (usart.md: single probes
// flip it at the outer instants on both parts, and a pulse over the first
// two kept it on the CH32V303 in one leg), so no verdict reads the word
// there. The deterministic windows are those the instants' spread decides
// for every leg: 33 % to 47 % covers the first always (margins 4 and 3 %)
// and the last never (2 %) - NE; 33 % to 61 % covers all three (4 % each
// side) - the bit flipped, no NE; 5 % to 30 % none - each margin above the
// two crystals' drift by the fifth cell (200 ppm x 5 bits = 0.1 % of a
// bit, two quartz crystals of 100 ppm - an assumption: the boards name no
// part).

/// One stage leg: `count` frames from the far engine at `rate`, read here
/// polled at `dc`; the far report's `drawn`.
struct Staged {
    bool up = false;
    bool drawn = false;
    Drain d{};
};

Staged stage_leg(const link::Params& a, const link::Cfg& dc, uint16_t ms, int32_t expect = -1) {
    Staged s{};
    const bool acted = D::act(link::Op::stage, a);
    s.up = acted && D::configure(dc);
    s.d = s.up ? D::collect(ms, expect) : Drain{};
    link::Report r{};
    s.drawn = D::report(r, 0) && (r.flags & link::report_ran) != 0u && (r.flags & link::report_mismatch) == 0u &&
              r.count == a.count;
    return s;
}

/// A drawn 8N1 train of `count` frames of `value` at `rate` with one
/// pulse, two bits of idle between frames.
link::Params staged(uint32_t rate, uint8_t value, uint16_t count, int8_t cell, int8_t pct, uint8_t width) {
    link::Params a{};
    a.cfg.rate = rate;
    a.count = count;
    a.value = value;
    a.pattern = link::pattern_fixed;
    a.cell = cell;
    a.pct = pct;
    a.aux8 = width;
    a.aux16 = 32;   // two bits of idle, in sixteenths
    a.ms = 400;
    return a;
}

void t4_errors() {
    if (!ready()) {
        done();
        return;
    }
    link::Cfg dc{};
    dc.rate = 9600;
    // 1. Odd parity into an even receiver, from the peer's own port.
    {
        link::Cfg pc{};
        pc.rate = 115'200;
        pc.parity = 2;
        link::Cfg ec = pc;
        ec.parity = 1;
        link::Params a{};
        a.cfg = pc;
        a.count = 6;
        a.ms = D::bound_ms(115'200, 12);
        a.value = 0x5A;
        a.pattern = link::pattern_fixed;
        const bool acted = D::act(link::Op::send, a);
        const bool up = acted && D::configure(ec);
        const Drain d = D::collect(D::bound_ms(115'200, 12));
        link::Report r{};
        (void)D::report(r, 0);
        print(serial, "  odd into even: frames=", d.frames, " pe=", d.pe, " fe=", d.fe, crlf);
        judge("every frame of the wrong parity raises PE, the byte still delivered",
              up && d.frames == 6u && d.pe == 6u && d.fe == 0u && (d.last & 0xFFu) == 0x5Au);
    }
    // 2. The DRAWN errors at 9600: the parity cell inverted (8E1 0x5A, a
    //    whole-cell pulse on the tenth cell), the stop cell held low (FE),
    //    and a break drawn as a zero frame with its stop low.
    {
        link::Params a = staged(9600, 0x5A, 6, 9, 0, 100);
        a.cfg.parity = 1;
        link::Cfg ec = dc;
        ec.parity = 1;
        const Staged p = stage_leg(a, ec, 60);
        print(serial, "  the parity cell drawn inverted: frames=", p.d.frames, " pe=", p.d.pe, " fe=", p.d.fe, " last ",
              hex(p.d.last), crlf);
        judge("a parity cell drawn the other way raises PE on every frame, the data intact",
              p.up && p.drawn && p.d.frames == 6u && p.d.pe == 6u && p.d.fe == 0u && (p.d.last & 0xFFu) == 0x5Au);
        const Staged f = stage_leg(staged(9600, 0x5A, 6, 9, 0, 100), dc, 60);
        print(serial, "  the stop cell drawn low: frames=", f.d.frames, " fe=", f.d.fe, " last ", hex(f.d.last), crlf);
        judge("a stop cell drawn low raises FE on every frame, the data intact",
              f.up && f.drawn && f.d.frames == 6u && f.d.fe == 6u && f.d.last == 0x5Au);
        const bool acted = D::act(link::Op::stage, staged(9600, 0x00, 4, 9, 0, 100));
        bool up = acted && D::configure(dc);
        W::enable(false);
        up = up && W::lin(LinConfig{false, false});
        W::enable(true);
        W::clear_flags(usart_lbd);
        const Drain b = D::collect(60);
        const bool lbd = W::flag(usart_lbd);
        W::lin_off();
        link::Report r{};
        (void)D::report(r, 0);
        print(serial, "  a zero frame with its stop drawn low, LIN at ten bits: LBD ", lbd, ", frames=", b.frames,
              " fe=", b.fe, crlf);
        judge("a zero frame with a low stop is a ten-bit break: LBD, and the 0x00 frame with FE",
              up && lbd && b.frames == 4u && b.fe == 4u && b.last == 0x00u);
    }
    // 3. THE NOISE CENSUS, printed: a 3 % pulse walked across data bit 3
    //    of 0xFF, two frames a step - '.' clean, 'n' NE with the bit kept,
    //    'N' NE with the bit flipped, 'f' flipped with no NE - then the
    //    three windows the spread decides (the section's header).
    {
        print(serial, "  a 3 % pulse across data bit 3 of 0xFF at 9600, its start 24 % to 64 %, 2 % a step: ");
        for (int8_t p = 24; p <= 64; p = static_cast<int8_t>(p + 2)) {
            const Staged s = stage_leg(staged(9600, 0xFF, 2, 4, p, 3), dc, 30, 0xF7);
            const uint16_t flips = s.d.frames >= s.d.wrong ? static_cast<uint16_t>(s.d.frames - s.d.wrong) : 0u;
            print(serial, s.d.frames != 2u ? "?" : (s.d.ne != 0u ? (flips != 0u ? "N" : "n") : (flips != 0u ? "f" : ".")));
        }
        print(serial, crlf);
        const Staged first = stage_leg(staged(9600, 0xFF, 6, 4, 33, 14), dc, 70);
        const Staged all = stage_leg(staged(9600, 0xFF, 6, 4, 33, 28), dc, 70, 0xF7);
        const Staged none = stage_leg(staged(9600, 0xFF, 6, 4, 5, 25), dc, 70, 0xFF);
        print(serial, "  33..47 %: ne=", first.d.ne, " wrong=", first.d.wrong, "; 33..61 %: ne=", all.d.ne,
              " wrong=", all.d.wrong, "; 5..30 %: ne=", none.d.ne, " wrong=", none.d.wrong, crlf);
        print(serial, "  33..47 %: the word ", hex(first.d.last), " (0xF7 the bit flipped, 0xFF kept)", crlf);
        judge("a pulse over a bit's first sample and never its last raises NE on every frame",
              first.up && first.drawn && first.d.frames == 6u && first.d.ne == 6u && first.d.fe == 0u);
        judge("a pulse over all three flips the bit with no NE",
              all.up && all.drawn && all.d.frames == 6u && all.d.ne == 0u && all.d.wrong == 0u);
        judge("a pulse that misses all three changes nothing",
              none.up && none.drawn && none.d.frames == 6u && none.d.ne == 0u && none.d.wrong == 0u);
    }
    // 4. A uniform rate error, drawn: 0x00 frames at 9600 x (1 + e) into
    //    this receiver at 9600. 18.3: the tolerance "shall not be less than
    //    3 %"; the two crystals take up to 0.02 % of it (two quartz
    //    crystals of 100 ppm, an assumption: the boards name no part), so
    //    every rate within 2.9 % must be clean.
    {
        static constexpr int16_t errs[] = {0, 20, -20, 29, -29, 40, -40, 50, -50, 60, -60, 80, -80};   // per mille
        int16_t onset_pos = 0, onset_neg = 0;
        bool inside_clean = true;
        for (const int16_t e : errs) {
            const uint32_t rate = static_cast<uint32_t>((9600 * (1000 + e)) / 1000);
            const Staged s = stage_leg(staged(rate, 0x00, 6, -1, 0, 0), dc, 70, 0x00);
            print(serial, "  ", e, " per mille: frames=", s.d.frames, " fe=", s.d.fe, " ne=", s.d.ne, " wrong=",
                  s.d.wrong, crlf);
            const bool broken = s.d.fe != 0u || s.d.frames != 6u || s.d.wrong != 0u;
            if (s.up && broken) {
                if (e > 0 && onset_pos == 0) onset_pos = e;
                if (e < 0 && onset_neg == 0) onset_neg = e;
            }
            if (e >= -29 && e <= 29) {
                inside_clean = inside_clean && s.up && s.drawn && !broken;
            }
        }
        print(serial, "  the frames break from +", onset_pos, " per mille and from ", onset_neg, crlf);
        judge("every rate within 2.9 % of this receiver's is clean (18.3's 3 %, less the crystals')", inside_clean);
        judge("and a rate far enough off breaks the frames both ways", onset_pos != 0 && onset_neg != 0);
    }
    // 5. Single low pulses on the idle line, ONE PULSE from the far timer:
    //    where a start bit starts being believed. The detector wants the
    //    line low at its 3rd, 5th and 7th sixteenths (2 of 3 with the
    //    noise rule): a pulse under 4/16 of a bit can cover the 3rd alone,
    //    so every pulse under 3/16 - a sixteenth of margin - is never a
    //    start bit; a whole bit always is.
    {
        static constexpr uint16_t widths[] = {6, 60, 160, 300, 625, 1250, 1875, 2500};   // ticks; a bit is 2500
        bool short_ignored = true, whole_taken = false;
        uint16_t first_taken = 0;
        print(serial, "  one low pulse at 9600 (a bit is 2500 ticks), frames it made:");
        for (const uint16_t w : widths) {
            link::Params a{};
            a.cfg.rate = 9600;
            a.count = 1;
            a.ms = 300;
            a.aux16 = w;
            a.aux8 = 1;
            const bool acted = D::act(link::Op::glitch, a);
            const bool up = acted && D::configure(dc);
            const Drain d = D::collect(20);
            link::Report r{};
            (void)D::report(r, 0);
            print(serial, " ", w, ":", d.frames, d.ne != 0u ? "n" : "", d.fe != 0u ? "f" : "");
            if (up && d.frames != 0u && first_taken == 0u) first_taken = w;
            if (w <= 300u) short_ignored = short_ignored && up && d.frames == 0u;
            if (w == 2500u) whole_taken = up && d.frames == 1u;
        }
        print(serial, crlf, "  the narrowest pulse taken as a start bit: ", first_taken, " ticks", crlf);
        judge("no pulse under 3/16 of a bit is a start bit (6 to 300 ticks)", short_ignored);
        judge("a whole bit low is one", whole_taken);
    }
    // 6. A run into a receiver that reads late: ORE.
    {
        link::Cfg pc{};
        pc.rate = 115'200;
        link::Params a{};
        a.cfg = pc;
        a.count = 8;
        a.ms = D::bound_ms(115'200, 16);
        a.value = 0xB0;
        a.pattern = link::pattern_fixed;
        const bool acted = D::act(link::Op::blast, a);
        const bool up = acted && D::configure(pc);
        spin_us<Ruler>(8'000);   // the window, and the run, unread
        const uint16_t st = W::status();
        const uint16_t kept = W::read_word();
        const Drain d = D::collect(5);
        link::Report r{};
        (void)D::report(r, 0);
        print(serial, "  a run of 8 read late: STATR ", hex(st), ", DATAR ", hex(kept), ", then ", d.frames, " more",
              crlf);
        judge("a run read late raises ORE", up && (st & usart_ore) != 0u && (st & usart_rxne) != 0u);
        judge("and DATAR keeps the first frame of the run, the rest lost", up && kept == 0xB0u && d.frames == 0u);
    }
    // 7. LIN's break detection at both lengths, the breaks from the far
    //    engine.
    {
        struct BreakCase {
            uint16_t bits;
            bool eleven;
            bool detect;
        };
        static constexpr BreakCase cases[] = {{9, false, false}, {13, false, true}, {10, true, false}, {13, true, true}};
        bool all = true;
        for (const BreakCase& bc : cases) {
            link::Params a{};
            a.cfg.rate = 9600;
            a.count = bc.bits;
            a.ms = 200;
            const bool acted = D::act(link::Op::brk, a);
            bool up = acted && D::configure(dc);
            W::enable(false);
            up = up && W::lin(LinConfig{bc.eleven, false});
            W::enable(true);
            W::clear_flags(usart_lbd);
            const Drain d = D::collect(30);
            const bool lbd = W::flag(usart_lbd);
            W::lin_off();
            link::Report r{};
            (void)D::report(r, 0);
            print(serial, "  a ", bc.bits, "-bit break, ", bc.eleven ? "11" : "10", "-bit detection: LBD ", lbd,
                  ", frames ", d.frames, " fe ", d.fe, crlf);
            all = all && up && lbd == bc.detect;
        }
        judge("LIN detects a 13-bit break at both lengths, a 9-bit low at neither, 10 bits not at 11", all);
    }
    done();
}

// ---------------------------------------------------------------------------
// 5 - the peer's engine shapes, and this end's own pull-banger on the wire
// ---------------------------------------------------------------------------

Drain bitbang_run(uint8_t value, uint16_t frames, int8_t cell, int8_t pct, uint16_t break_bits, bool lin) {
    link::Params a{};
    a.cfg.rate = 9600;
    a.count = frames;
    a.ms = 600;
    a.value = value;
    a.cell = cell;
    a.pct = pct;
    a.aux16 = break_bits;
    link::Cfg dc{};
    dc.rate = 9600;
    if (!D::act(link::Op::bitbang, a) || !D::configure(dc)) return Drain{};
    if (lin) {
        W::enable(false);
        (void)W::lin(LinConfig{});
        W::enable(true);
        W::clear_flags(usart_lbd);
    }
    Drain d = D::collect(static_cast<uint16_t>(30u + 15u * frames + break_bits / 4u));
    d.lbd = W::flag(usart_lbd);
    W::lin_off();
    link::Report r{};
    (void)D::report(r, 0);
    return d;
}

/// THIS END'S PULL-BANGER on its own receive pad: the pad's pull moved by
/// the output data bit (pin.md) at absolute offsets on the cycle count,
/// the far pad floating while the peer stands off. One 8N1 frame of
/// `value` at 9600, cell `cell` stretched by `pct` per cent.
void pull_bang(uint8_t value, int8_t cell, int8_t pct) {
    const uint32_t bit = hclk / 9600u;
    uint32_t at = 0;
    const uint32_t t0 = Ticker::cycles();
    for (int8_t k = 0; k < 10; ++k) {
        const bool high = k == 0 ? false : (k == 9 ? true : ((value >> (k - 1)) & 1u) != 0u);
        if (high) {
            WRx::set();
        } else {
            WRx::clear();
        }
        uint32_t w = bit;
        if (k == cell) {
            w = static_cast<uint32_t>(static_cast<int32_t>(bit) + (static_cast<int32_t>(bit) * pct) / 100);
        }
        at += w;
        while (Ticker::cycles() - t0 < at) {
        }
    }
    WRx::set();
    while (Ticker::cycles() - t0 < at + 4u * bit) {
    }
}

void t5_shapes() {
    if (!ready()) {
        done();
        return;
    }
    const Drain clean = bitbang_run(0x55, 6, -1, 0, 0, false);
    print(serial, "  six banged 0x55 at 9600: frames=", clean.frames, " fe=", clean.fe, " last ", hex(clean.last), crlf);
    judge("a frame from the peer's edge list is a clean 0x55, six times",
          clean.frames == 6u && clean.fe == 0u && clean.ne == 0u && clean.last == 0x55u);
    (void)D::command_mode();

    // One stretched cell, the peer's engine then this end's pull-banger on
    // the same wire (the peer standing off, its pads floating).
    static constexpr int8_t pcts[] = {10, 20, 30, 40, 50, 60, 70};
    int8_t onset_engine = 0;
    for (const int8_t p : pcts) {
        const Drain d = bitbang_run(0x00, 4, 5, p, 0, false);
        if ((d.fe != 0u || d.frames != 4u) && onset_engine == 0) onset_engine = p;
    }
    int8_t onset_pull = 0;
    link::Params so{};
    so.ms = 2000;
    uint8_t sp[link::params_size] = {};
    link::put_params(sp, so);
    const bool off = D::command(link::Op::standby, sp, link::params_size);
    for (const int8_t p : pcts) {
        if (!off) break;
        (void)D::open(UartFormat{}, 9600);
        WRx::input(PinPull::up);
        spin_us<Ruler>(1'000);
        D::flush();
        // DATAR holds one frame: each is read before the next is banged.
        uint8_t frames = 0, fe = 0;
        for (uint8_t i = 0; i < 4u; ++i) {
            pull_bang(0x00, 5, p);
            const Drain one = D::collect(1);
            frames = static_cast<uint8_t>(frames + one.frames);
            fe = static_cast<uint8_t>(fe + one.fe);
        }
        if ((fe != 0u || frames != 4u) && onset_pull == 0) onset_pull = p;
    }
    WRx::release();
    spin_us<Ruler>(2'000'000);   // the stand-off runs out; the peer's search resumes
    print(serial, "  one stretched cell breaks the frame from +", onset_engine, " % (the peer's engine), +", onset_pull,
          " % (this end's pull-banger)", crlf);
    judge("one stretched cell survives far past the rate tolerance (30 % and more) from the peer's engine",
          onset_engine == 0 || onset_engine >= 30);
    judge("and this end's own pull-banger on the same wire breaks the frame within one step of it",
          off && D::within(onset_engine, onset_pull, 10));
    (void)D::ensure();

    const Drain lin = bitbang_run(0x55, 1, -1, 0, 13, true);
    print(serial, "  a 13-bit break then 0x55: LBD ", lin.lbd, ", frames=", lin.frames, " last ", hex(lin.last), crlf);
    judge("a LIN break ahead of a frame: the break detected and the frame after it received",
          lin.lbd && lin.last == 0x55u);
    done();
}

// ---------------------------------------------------------------------------
// 6 - the link's receive engine under the peer's repeated blast
// ---------------------------------------------------------------------------

constexpr DmaSlot link_rx_slot = usart_dma_rx_slot(link_n);
constexpr DmaSlot link_tx_slot = usart_dma_tx_slot(link_n);
using LinkRxEngine = DmaRxEngine<link_rx_slot.controller, link_rx_slot.channel>;
using LinkTxEngine = DmaTxEngine<link_tx_slot.controller, link_tx_slot.channel>;
using LinkRing = Uart<link_n, P, 512, 64, UartFormat{}, NoDmaEngine, LinkRxEngine, link_column>;
/// The link with both engines: letter n's transport, where they are DMA2's.
using LinkDma = Uart<link_n, P, 256, 256, UartFormat{}, LinkTxEngine, LinkRxEngine, link_column>;
/// Whether the link's two engines are DMA2's (UART4's slots, table 11-3):
/// letter n's subject, registered where they are.
constexpr bool link_on_dma2 = link_present && link_tx_slot.controller == 2u && link_rx_slot.controller == 2u;
/// Which transport owns the link's two vectors: 1 the ring, 2 both engines.
volatile uint8_t link_owner = 0;
volatile uint32_t link_edges = 0;

void t6_engine() {
    if (!ready()) {
        done();
        return;
    }
    Dma<link_rx_slot.controller>::open();
    static constexpr uint32_t rates[] = {1'000'000, 2'000'000, 3'000'000, 4'500'000};
    for (const uint32_t rate : rates) {
        // About a second of gapless frames: blocks of 10000, repeated with
        // no gap, the counting pattern running on across them.
        const uint16_t block = 10'000;
        const uint16_t repeats = static_cast<uint16_t>(rate / 10u / block);
        const uint32_t total = static_cast<uint32_t>(block) * repeats;
        link::Params a{};
        a.cfg.rate = rate;
        a.count = block;
        a.aux16 = repeats;
        a.aux8 = 0;
        a.ms = 3000;
        a.value = 0x30;
        a.pattern = link::pattern_counting;
        const bool acted = D::act(link::Op::blast, a);
        W::bus_clock(true);
        W::reset();
        link_owner = 1;
        link_edges = 0;
        const bool up = acted && LinkRing::init(clock, rate);
        WRx::input(PinPull::up);
        uint32_t got = 0;
        uint8_t expect = 0x30;
        bool broken = false;
        uint8_t wrong = 0;
        const uint32_t t0 = Ticker::millis();
        while (up && got < total && !broken && Ticker::millis() - t0 < 2500u) {
            const std::span<const uint8_t> run = LinkRing::read_span();
            uint32_t n = 0;
            for (const uint8_t b : run) {
                if (b != expect) {
                    wrong = b;
                    broken = true;
                    break;
                }
                ++expect;
                ++n;
            }
            got += n;
            (void)LinkRing::consume(n);
        }
        const uint16_t ore = LinkRing::hw_overruns();
        const uint16_t fe = LinkRing::frame_errors();
        const uint16_t lost = LinkRing::rx_overruns();
        const uint16_t faults = LinkRing::dma_faults();
        const uint32_t edges = link_edges;
        LinkRing::release();
        link_owner = 0;
        link::Report r{};
        const bool have = D::report(r, 5);
        print(serial, "  ", rate, ": ", got, " of ", total, " bytes in sequence");
        if (broken) {
            print(serial, " - BROKEN, ", hex(wrong), " read");
        }
        print(serial, ", ORE ", ore, ", FE ", fe, ", ring overruns ", lost, ", faults ", faults, ", ", edges,
              " edges; the peer sent ", r.count, " (mod 65536)", crlf);
        judge(num(rate), " baud: a second of the peer's blast through the receive engine, every byte in sequence",
              up && have && got == total && !broken && ore == 0u && fe == 0u && lost == 0u && faults == 0u &&
                  r.count == static_cast<uint16_t>(total));
    }
    done();
}

// ---------------------------------------------------------------------------
// 7 - half duplex on the link's TX pad: a second chip's frames, and the
//     wired AND
// ---------------------------------------------------------------------------

void t7_half_duplex() {
    if (!ready()) {
        done();
        return;
    }
    // drive_rxd: the peer drives the wire this end's TX pad sits on, from
    // its receive pad; HDSEL listens there (18.5), the pad an open drain
    // that never pulls (TE off).
    bool heard_all = true;
    for (const uint32_t rate : {9600u, 115'200u, 1'000'000u}) {
        link::Params a{};
        a.cfg.rate = rate;
        a.count = 4;
        a.value = 0x5A;
        a.ms = 400;
        const bool acted = D::act(link::Op::drive_rxd, a);
        bool up = acted && D::open(UartFormat{}, rate);
        W::transmitter(false);
        up = up && W::half_duplex(true);
        WTx::function(PinDrive::open_drain);
        D::flush();
        const Drain d = D::collect(static_cast<uint16_t>(rate == 9600u ? 80u : 40u));
        (void)W::half_duplex(false);
        link::Report r{};
        (void)D::report(r, 0);
        print(serial, "  HDSEL on the link's TX pad at ", rate, ": ", d.frames, " frames, last ", hex(d.last), ", fe ",
              d.fe, "; the peer drove ", r.count, crlf);
        heard_all = heard_all && up && d.frames == 4u && d.fe == 0u && d.last == 0x5Au;
    }
    judge("HDSEL on the TX pad hears the four frames a second chip drives onto that wire, at 9600, 115200 and 1 Mbaud",
          heard_all);

    // collide: this end sends 0xA5 then 0xF0 back to back, its pad an open
    // drain; the peer, once 0xA5 is in, pulls 0x3C into the next frame as
    // an emulated open drain on its pull-up - the wire's only pull - and
    // its own receiver reads the wire: the AND.
    {
        // THE PEER'S DELAY, derived: it starts its frame aux16 us after its
        // receiver's RXNE for 0xA5, and this end's 0xF0 starts at 0xA5's
        // stop bit's end. RXNE comes once the stop bit is sampled - from its
        // middle to its end (chapter 18 places it no closer; the sampling
        // census of letter 4 puts the instants from 37 % of a cell on) - so
        // the peer's start lies between aux16 - 52 and aux16 us of this
        // end's at 9600 (a bit 104.2 us), plus its poll's microsecond. The
        // wired AND reads right while the two starts are under the receiver's
        // first sampling instant apart, 37 % of a bit, 38 us: aux16 = 26, a
        // quarter bit, centres the window and leaves 12 us each side (the two
        // crystals move it by 0.1 us over the frame).
        link::Params a{};
        a.cfg.rate = 9600;
        a.value = 0x3C;
        a.aux16 = 26;
        a.ms = 400;
        const bool acted = D::act(link::Op::collide, a);
        bool up = acted && D::open(UartFormat{}, 9600);
        up = up && W::half_duplex(true);
        WTx::function(PinDrive::open_drain);
        D::wait_for_window();
        D::flush();
        if (up) {
            D::put(0xA5);
            D::put(0xF0);
            up = D::wait_tc();
        }
        (void)W::half_duplex(false);
        link::Report r{};
        (void)D::report(r, 5);
        print(serial, "  collide at 9600: the peer heard ", r.count, " frames, the second ", hex(r.aux16),
              " (0xF0 AND 0x3C = 0x30), ferr ", r.ferr, crlf);
        judge("THE WIRED AND: the peer's frame pulled into this end's 0xF0 on an open-drain wire reads 0x30",
              up && r.count == 2u && r.aux16 == 0x30u);
    }
    done();
}

// ---------------------------------------------------------------------------
// 8 - mute and IrDA on the wire
// ---------------------------------------------------------------------------

void t8_modes() {
    if (!ready()) {
        done();
        return;
    }
    // 1. Mute woken by an address mark: 9-bit words, this node 5.
    {
        link::Cfg c{};
        c.rate = 115'200;
        c.bits = link::bits9_low;
        link::Params a{};
        a.cfg = c;
        a.value = 0x10;
        a.groups = 3;
        const uint8_t pairs[6] = {3, 2, 5, 3, 3, 2};
        const bool acted = D::act(link::Op::send_mpcm, a, pairs, 6);
        bool up = acted && D::configure(c);
        up = up && W::mute_mode(MuteConfig{MuteWake::address_mark, 5}) && W::mute();
        const Drain d = D::collect(40);
        print(serial, "  address mark: frames=", d.frames, " first ", hex(d.first), " last ", hex(d.last), crlf);
        link::Report r{};
        (void)D::report(r, 0);
        judge("a muted receiver wakes on its address alone: the address frame and its three data",
              up && d.frames == 4u && d.first == 0x105u && d.last == 0x14u);
        W::unmute();
    }
    // 2. Mute woken by an idle line: three runs of four, 3 ms apart; the
    //    receiver muted after the first frame wakes on the idle after the
    //    first run (18.10.4: a byte must have come before an idle wakes).
    {
        link::Cfg c{};
        c.rate = 9600;
        link::Params a{};
        a.cfg = c;
        a.count = 4;
        a.aux16 = 3;
        a.aux8 = 30;
        a.ms = 400;
        a.pattern = link::pattern_counting;
        a.value = 0;
        const bool acted = D::act(link::Op::blast, a);
        bool up = acted && D::configure(c) && W::mute_mode(MuteConfig{MuteWake::idle_line, 0});
        uint16_t seen[12] = {};
        uint8_t n = 0;
        const uint32_t t0 = Ticker::millis();
        while (Ticker::millis() - t0 < 60u) {
            if (!W::rx_ready()) continue;
            const uint16_t w = W::read_word();
            if (n < 12u) seen[n++] = w;
            if (n == 1u) up = up && W::mute();
        }
        print(serial, "  idle line: ", n, " frames:");
        for (uint8_t i = 0; i < n; ++i) print(serial, " ", seen[i]);
        print(serial, crlf);
        bool expected = n == 9u && seen[0] == 0u;
        for (uint8_t i = 1; expected && i < 9u; ++i) expected = seen[i] == static_cast<uint16_t>(i + 3u);
        link::Report r{};
        (void)D::report(r, 0);
        judge("muted after the first frame, the receiver drops the rest of the run and wakes on the idle line",
              up && expected);
        W::unmute();
    }
    // 3. IrDA, normal mode, between two WCH SIR codecs, both ways: this
    //    end's encoder into the far decoder, the far encoder back.
    {
        link::Cfg c{};
        c.rate = 115'200;
        c.mode = link::Mode::ircom;
        link::Params a{};
        a.cfg = c;
        a.count = 8;
        a.ms = D::bound_ms(115'200, 24);
        const bool acted = D::act(link::Op::echo, a);
        const bool up = acted && D::configure(c);
        const uint16_t bad = up ? echo_mismatches(8, 0xFFu) : 8u;
        link::Report r{};
        const bool have = D::report(r, 5);
        W::irda_off();
        print(serial, "  IrDA at 115200: wrong=", bad, " peer count=", r.count, " ferr=", r.ferr, crlf);
        judge("IrDA's SIR frames echoed by a second WCH codec, every one equal",
              up && bad == 0u && have && r.count == 8u && r.ferr == 0u);
    }
    done();
}

// ---------------------------------------------------------------------------
// 9 - the link's receive ring: rx_pending(), a broken stream's counts, and
//     the burst edge from the vector
// ---------------------------------------------------------------------------

/// The link with no engine at all: the interrupt receiver whose
/// rx_pending() letter 9 asks beside the ring's.
using LinkPlain = Uart<link_n, P, 64, 64, UartFormat{}, NoDmaEngine, NoDmaEngine, link_column>;
/// The link's USART vector entries while letter 9's ring owns it.
volatile uint32_t link_entries = 0;

/// Transport `T` up on the link at `rate` after an action was commanded:
/// the port through its reset line, the vectors given to `owner`, the RX
/// pad pulled up as letter 6 does.
template <typename T>
bool link_up(uint8_t owner, uint32_t rate) {
    W::bus_clock(true);
    W::reset();
    link_owner = owner;
    link_edges = 0;
    link_entries = 0;
    const bool up = T::init(clock, rate);
    WRx::input(PinPull::up);
    return up;
}

/// A train drawn by the far engine at 115200: `count` frames of the
/// counting pattern from `first`, the stop cell held low for the whole bit
/// (each frame a framing error carrying its byte), `idle16` sixteenths of
/// a bit between frames.
link::Params broken_train(uint8_t first, uint16_t count, uint16_t idle16) {
    link::Params a{};
    a.cfg.rate = 115'200;
    a.count = count;
    a.value = first;
    a.pattern = link::pattern_counting;
    a.cell = 9;
    a.pct = 0;
    a.aux8 = 100;
    a.aux16 = idle16;
    a.ms = 400;
    return a;
}

/// What the ring held after a train: the bytes in order from `first`, and
/// how many.
uint16_t ring_take(uint8_t first, bool& in_order) {
    uint16_t got = 0;
    uint8_t want = first;
    in_order = true;
    uint8_t b = 0;
    while (LinkRing::read_byte(b)) {
        if (b != want) {
            in_order = false;
        }
        ++want;
        ++got;
    }
    return got;
}

void t9_ring() {
    if (!ready()) {
        done();
        return;
    }
    Dma<link_rx_slot.controller>::open();

    // ---- rx_pending(), on the ring and on the interrupt receiver ----
    // 37 frames of the far chip's counting pattern at 115200 (3.2 ms of
    // wire), waited out with a margin; then ten read, then the rest.
    constexpr uint16_t n = 37;
    bool pending_ok = true;
    for (uint8_t shape = 0; shape < 2u; ++shape) {
        link::Params a{};
        a.cfg.rate = 115'200;
        a.count = n;
        a.value = 0x40;
        a.pattern = link::pattern_counting;
        a.ms = D::bound_ms(115'200, n);
        const bool acted = D::act(link::Op::send, a);
        const bool up = acted && (shape == 0u ? link_up<LinkRing>(1, 115'200) : link_up<LinkPlain>(3, 115'200));
        spin_us<Ruler>(20'000);
        uint32_t p0 = 0;
        uint32_t p1 = 0;
        uint32_t p2 = 0;
        bool in_order = true;
        uint8_t b = 0;
        uint8_t want = 0x40;
        if (shape == 0u) {
            p0 = LinkRing::rx_pending();
            for (uint8_t i = 0; i < 10u && LinkRing::read_byte(b); ++i) {
                in_order = in_order && b == want++;
            }
            p1 = LinkRing::rx_pending();
            while (LinkRing::read_byte(b)) {
                in_order = in_order && b == want++;
            }
            p2 = LinkRing::rx_pending();
            LinkRing::release();
        } else {
            p0 = LinkPlain::rx_pending();
            for (uint8_t i = 0; i < 10u && LinkPlain::read_byte(b); ++i) {
                in_order = in_order && b == want++;
            }
            p1 = LinkPlain::rx_pending();
            while (LinkPlain::read_byte(b)) {
                in_order = in_order && b == want++;
            }
            p2 = LinkPlain::rx_pending();
            LinkPlain::release();
        }
        link_owner = 0;
        link::Report r{};
        const bool have = D::report(r, 0);
        const bool ok = up && have && r.count == n && p0 == n && p1 == n - 10u && p2 == 0u && in_order &&
                        want == static_cast<uint8_t>(0x40u + n);
        pending_ok = pending_ok && ok;
        print(serial, "  ", shape == 0u ? "the ring" : "the interrupt receiver", ": rx_pending() ", p0, " with ", n,
              " arrived, ", p1, " after ten read, ", p2, " after the rest", in_order ? "" : " (NOT IN ORDER)", crlf);
    }
    judge("RX_PENDING() COUNTS WHAT ARRIVED AND WAS NOT READ, on the receive ring and on the interrupt receiver: "
          "37, then 27 after ten read, then none",
          pending_ok);

    // ---- a broken stream under the ring: the counts the channel's read bounds ----
    // Eight frames each with its stop cell drawn low: every one a framing
    // error carrying its byte. Two bits apart (no idle line between them)
    // the header's rule is that a run of errored frames counts every other
    // one - four of eight - each byte stored. Twelve bits apart there is an
    // idle line after each frame, but the vector never sees one: after a
    // counted error it waits for a frame with IDLEIE down, that frame's
    // read clears the IDLE standing, and the idle the clearing frame armed
    // is forgotten (the header) - so the train counts as a run, four of
    // eight again.
    constexpr uint16_t train = 8;
    for (uint8_t k = 0; k < 2u; ++k) {
        const uint16_t idle16 = k == 0u ? 32u : 192u;
        const uint8_t first = static_cast<uint8_t>(0x60u + 0x10u * k);
        const bool acted = D::act(link::Op::stage, broken_train(first, train, idle16));
        const bool up = acted && link_up<LinkRing>(1, 115'200);
        spin_us<Ruler>(20'000);
        bool in_order = false;
        const uint16_t got = ring_take(first, in_order);
        const uint16_t fe = LinkRing::frame_errors();
        const uint16_t ore = LinkRing::hw_overruns();
        const uint16_t lost = LinkRing::rx_overruns();
        LinkRing::release();
        link_owner = 0;
        link::Report r{};
        const bool drawn = D::report(r, 0) && (r.flags & link::report_ran) != 0u && r.count == train;
        print(serial, "  eight frames, each stop drawn low, ", k == 0u ? "two bits" : "twelve bits", " apart: ", got,
              " stored", in_order ? " in order" : " NOT IN ORDER", ", FE counted ", fe, ", ORE ", ore,
              ", laps missed ", lost, ", ", link_entries, " USART entries, ", link_edges, " edges", crlf);
        if (k == 0u) {
            judge("A RUN OF ERRORED FRAMES under the ring: every byte stored in order, the framing errors counted "
                  "every other one (4 of 8), nothing else counted",
                  up && drawn && got == train && in_order && fe == train / 2u && ore == 0u && lost == 0u);
        } else {
            judge("... and with an idle line after every frame the same: the vector never sees one, eight stored "
                  "in order, four counted",
                  up && drawn && got == train && in_order && fe == train / 2u && ore == 0u && lost == 0u);
        }
    }

    // ---- the burst edge from the vector, nothing polled ----
    // At 1 Mbaud a frame is 1440 core cycles. A burst of one: told, its
    // byte there. A burst of sixteen: the last byte's landing stamped on
    // the channel's count, the edge on the vector's counter - the idle line
    // a frame after the last stop bit, so within two frames - and at most
    // two USART entries for the burst, none a byte. THE EDGE TIMED IS THE
    // ONE AFTER THE LANDING: the burst's first frame is told on its own
    // (its RXNE entry), so the edge count is read at the landing and the
    // told stamp waits for it to move from there; the console drained
    // before the window.
    constexpr uint32_t frame = 10u * (SysClock::hz / 1'000'000u);
    bool one_ok = true;
    for (uint8_t k = 0; k < 3u; ++k) {
        link::Params a{};
        a.cfg.rate = 1'000'000;
        a.count = 1;
        a.value = static_cast<uint8_t>(0x21u + k);
        a.pattern = link::pattern_fixed;
        a.ms = 50;
        const bool acted = D::act(link::Op::blast, a);
        const bool up = acted && link_up<LinkRing>(1, 1'000'000);
        spin_us<Ruler>(10'000);
        bool in_order = false;
        const uint32_t edges = link_edges;
        const uint16_t got = ring_take(a.value, in_order);
        LinkRing::release();
        link_owner = 0;
        print(serial, "  a burst of one frame: ", edges, " edge(s), ", got, " byte read", crlf);
        one_ok = one_ok && up && edges >= 1u && got == 1u && in_order;
        link::Report r{};
        (void)D::report(r, 0);
    }
    judge("a burst of one frame into the ring is told, every time, its byte there", one_ok);
    {
        link::Params a{};
        a.cfg.rate = 1'000'000;
        a.count = 16;
        a.value = 0x30;
        a.pattern = link::pattern_counting;
        a.ms = 50;
        const bool drained = brio::console_drain<Ruler, Serial>();
        const bool acted = D::act(link::Op::blast, a);
        const bool up = acted && link_up<LinkRing>(1, 1'000'000);
        const uint32_t start = LinkRxEngine::remaining();
        const uint32_t t0 = Ticker::millis();
        uint32_t landed = 0;
        uint32_t told = 0;
        uint32_t at_landing = 0;
        bool edge_seen = false;
        while (up && Ticker::millis() - t0 < 30u) {
            if (landed == 0u && start - LinkRxEngine::remaining() >= 16u) {
                landed = Ticker::cycles();
                at_landing = link_edges;
                edge_seen = at_landing != 0u;
            }
            if (landed != 0u && link_edges != at_landing) {
                told = Ticker::cycles();
                break;
            }
        }
        const uint32_t late = told - landed;
        spin_us<Ruler>(2'000);
        bool in_order = false;
        const uint16_t got = ring_take(0x30, in_order);
        const uint32_t entries = link_entries;
        const uint32_t edges = link_edges;
        LinkRing::release();
        link_owner = 0;
        link::Report r{};
        (void)D::report(r, 0);
        print(serial, "  16 frames at 1 Mbaud: ", edges, " edge(s), the edge ", late, " cycles after the last byte "
              "landed (", late * 10u / frame, " tenths of a frame), ", entries, " USART interrupt(s), ", got, " read",
              in_order ? " in order" : " NOT IN ORDER", edge_seen ? " (an edge was told before the last byte)" : "",
              drained ? "" : "; the console did not drain", crlf);
        judge("THE BURST EDGE ON THE LINK'S RING: sixteen frames read whole, told within two frames of the last "
              "byte's landing, with at most two USART interrupts and none a byte",
              drained && up && got == 16u && in_order && told != 0u && late <= 2u * frame && entries >= 1u &&
                  entries <= 2u);
    }
    done();
}

}  // namespace two

// ===========================================================================
// n and o - the link's engines on DMA2, and the class's lot registers
// ===========================================================================

namespace two {

/// The byte `i` of letter n's outgoing run, and the 16-bit sum of the
/// first `n` - what the far sink's sum must be.
constexpr uint8_t run_byte(uint32_t i) { return static_cast<uint8_t>(i * 37u + 11u); }
uint16_t run_sum(uint32_t n) {
    uint32_t s = 0;
    for (uint32_t i = 0; i < n; ++i) {
        s += run_byte(i);
    }
    return static_cast<uint16_t>(s);
}

/// `n` bytes of run_byte() through transport `T`'s write_bulk() as fast as
/// its ring takes them, timed from the first queue to tx_idle() - the last
/// stop bit's end - against the run's wire time at `rate`, in thousandths.
template <typename T>
uint32_t engine_out(uint32_t rate, uint32_t n) {
    static uint8_t chunk[64];
    const uint64_t wire = 10ULL * n * SysClock::hz / rate;
    const uint64_t budget = 2u * wire + SysClock::hz / 100u;
    uint32_t queued = 0;
    Stopwatch<Ruler> w;
    w.start();
    while (queued < n && w.elapsed() < budget) {
        const uint32_t want = n - queued < sizeof chunk ? n - queued : sizeof chunk;
        for (uint32_t i = 0; i < want; ++i) {
            chunk[i] = run_byte(queued + i);
        }
        uint32_t took = 0;
        while (took < want && w.elapsed() < budget) {
            took += T::write_bulk(std::span<const uint8_t>(chunk + took, want - took));
        }
        queued += took;
    }
    while (!T::tx_idle() && w.elapsed() < budget) {
    }
    const uint32_t cycles = w.elapsed();
    return wire == 0u ? 0u : static_cast<uint32_t>(1000ULL * cycles / wire);
}

// ---------------------------------------------------------------------------
// n - UART4's engines on DMA2, against the far chip
// ---------------------------------------------------------------------------

template <bool on = link_on_dma2>
void tn_pair_engines() {
    if constexpr (on) {
        if (!ready()) {
            done();
            return;
        }
        static_assert(!on || (link_tx_slot.controller == 2u && link_rx_slot.controller == 2u),
                      "table 11-3: UART4's two requests are DMA2's on this class");
        Dma<(on ? 2 : 1)>::open();
        // 1. A message out through the transmit engine on DMA2's channel 5,
        //    echoed by the far chip, and back through the receive engine on
        //    channel 3, byte for byte.
        static const uint8_t msg[] = "UART4's engines on DMA2, round the far chip's echo";
        constexpr uint16_t len = sizeof(msg) - 1u;
        {
            link::Params a{};
            a.cfg.rate = 115'200;
            a.count = len;
            a.ms = D::bound_ms(115'200, 2u * len);
            const bool acted = D::act(link::Op::echo, a);
            W::bus_clock(true);
            W::reset();
            link_owner = 2;
            const bool up = acted && LinkDma::init(clock, 115'200);
            WRx::input(PinPull::up);
            D::wait_for_window();
            uint8_t b = 0;
            while (LinkDma::read_byte(b)) {
            }
            uint8_t seen[len] = {};
            uint16_t got = 0;
            if (up) {
                (void)LinkDma::write_bulk(std::span<const uint8_t>(msg, len));
                const uint32_t t0 = Ticker::millis();
                while (got < len && Ticker::millis() - t0 < 50u) {
                    while (got < len && LinkDma::read_byte(b)) {
                        seen[got++] = b;
                    }
                }
            }
            bool same = got == len;
            for (uint16_t i = 0; i < got && same; ++i) {
                same = seen[i] == msg[i];
            }
            LinkDma::release();
            link_owner = 0;
            link::Report r{};
            const bool have = D::report(r, 5);
            print(serial, "  a message of ", len, " bytes round the far echo: ", same ? "exact" : "WRONG", " (", got,
                  " back; the far chip counted ", r.count, ")", crlf);
            judge("UART4's transmit engine on DMA2 pours a message into the far chip's echo, and its receive engine "
                  "takes it back, byte for byte",
                  up && same && have && r.count == len && r.ferr == 0u);
        }
        // 2. Four kilobytes OUT at 921600: the transmit engine's run timed
        //    on this core's counter AND by the far receiver, which counts,
        //    sums and stamps it - against a 1 Mbaud run back to back, the
        //    crystals' disagreement cancelled.
        constexpr uint32_t fast = 921'600;
        constexpr uint16_t kb4 = 4096;
        {
            link::Cfg ref{};
            ref.rate = calib_rate;
            const Timed r0 = D::timed_run(link::Op::sink, ref, 2000);
            bool up = false;
            bool have = false;
            uint32_t pm = 0;
            uint16_t faults = 0;
            link::Report r{};
            // A run counted whole whose span the peer reports not measured
            // had a far stamp an interrupt entry lengthened
            // (vx03_link_dut.hpp's header): printed and run again, up to
            // the Dut's attempts.
            for (uint8_t attempt = 0; attempt < D::timed_attempts; ++attempt) {
                link::Params a{};
                a.cfg.rate = fast;
                a.count = kb4;
                a.ms = D::bound_ms(fast, kb4);
                const bool acted = D::quiet_console() && D::act(link::Op::sink, a);
                W::bus_clock(true);
                W::reset();
                link_owner = 2;
                up = acted && LinkDma::init(clock, fast);
                WRx::input(PinPull::up);
                D::wait_for_window();
                pm = up ? engine_out<LinkDma>(fast, kb4) : 0u;
                faults = LinkDma::dma_faults();
                LinkDma::release();
                link_owner = 0;
                r = link::Report{};
                have = D::report(r, 5);
                if (!(have && r.count == kb4 && r.span == 0u)) {
                    break;
                }
                print(serial, "    a 4 KB run whose far stamp an interrupt lengthened: not judged, run again", crlf);
            }
            const int32_t far =
                link::calibrate_ppm(link::nominal_span(fast, 20, kb4), r.span) - (r0.ok ? r0.ppm : 0);
            const uint32_t bound = D::span_bound_ppm(fast, kb4) + D::span_bound_ppm(calib_rate, 2000);
            print(serial, "  4 KB out at 921600: ", pm, " thousandths of the wire on this counter; the far chip "
                  "counted ", r.count, " (sum ", hex(r.sum), " for ", hex(run_sum(kb4)), "), ferr ", r.ferr,
                  " ovf ", r.ovf, ", timed ");
            print_ppm(far);
            print(serial, " against the divisor's ", D::divisor_ppm(fast), " (bound ", bound, ")", crlf);
            judge("four kilobytes out of the transmit engine at 921600, every frame counted clean by the far chip and "
                  "its sum the pattern's",
                  up && have && r.count == kb4 && r.ferr == 0u && r.perr == 0u && r.ovf == 0u &&
                      r.sum == run_sum(kb4) && faults == 0u);
            judge("the engine's run in its wire time on this core's counter (no faster than the divisor, at most 3 % "
                  "slower) and, timed by the far "
                  "chip, PCLK1 / BRR to the stamps' bound",
                  wire_ok(pm, fast) && r0.ok && r.span != 0u && D::within(far, D::divisor_ppm(fast), bound));
        }
        // 3. Four kilobytes IN at 921600: the far blast into the receive
        //    engine's ring, every byte in order and every counter read.
        {
            link::Params a{};
            a.cfg.rate = fast;
            a.count = kb4;
            a.ms = D::bound_ms(fast, kb4);
            a.value = 0x21;
            a.pattern = link::pattern_counting;
            const bool acted = D::act(link::Op::blast, a);
            W::bus_clock(true);
            W::reset();
            link_owner = 2;
            const bool up = acted && LinkDma::init(clock, fast);
            WRx::input(PinPull::up);
            uint32_t got = 0;
            uint32_t wrong = 0;
            uint8_t expect = 0x21;
            const uint32_t t0 = Ticker::millis();
            while (up && got < kb4 && Ticker::millis() - t0 < 200u) {
                uint8_t b = 0;
                while (LinkDma::read_byte(b)) {
                    if (b != expect) ++wrong;
                    expect = static_cast<uint8_t>(b + 1u);
                    ++got;
                }
            }
            const uint16_t ore = LinkDma::hw_overruns();
            const uint16_t lost = LinkDma::rx_overruns();
            const uint16_t fe = LinkDma::frame_errors();
            const uint16_t faults = LinkDma::dma_faults();
            LinkDma::release();
            link_owner = 0;
            link::Report r{};
            const bool have = D::report(r, 5);
            print(serial, "  4 KB in at 921600: ", got, " bytes, ", wrong, " out of order; overruns ", lost, "/", ore,
                  ", frame errors ", fe, ", faults ", faults, "; the far chip sent ", r.count, crlf);
            judge("four kilobytes in from the far chip's blast at 921600 through the receive engine's ring, every "
                  "byte in order and no error, overrun or fault counted",
                  up && have && got == kb4 && wrong == 0u && ore == 0u && lost == 0u && fe == 0u && faults == 0u &&
                      r.count == kb4);
        }
        done();
    }
}

// ---------------------------------------------------------------------------
// o - the lot's registers, asked of the die and of the far receiver
// ---------------------------------------------------------------------------

/// The five registers the CTLR4 probe must leave as it found them.
struct UsartWords {
    uint16_t brr, ctlr1, ctlr2, ctlr3, gpr;
    bool operator==(const UsartWords&) const = default;
};

template <typename U>
UsartWords usart_words() {
    return {U::regs().BRR, U::regs().CTLR1, U::regs().CTLR2, U::regs().CTLR3, U::regs().GPR};
}

/// The link's port CONFIGURED BUT DISABLED, its pads the port's - the state
/// the CTLR4 probe wants.
bool link_configured(const UartFormat& f, uint32_t baud) {
    WTx::output(true);
    W::bus_clock(true);
    W::reset();
    if (!W::remap(link_column) || !W::configure(f, usart_divisor(wclk, baud))) {
        return false;
    }
    WTx::function();
    WRx::input(PinPull::up);
    return true;
}

/// USART2 the same way, disabled, on its default pads (no wire on them).
bool near_configured(const UartFormat& f, uint32_t baud) {
    U2::bus_clock(true);
    U2::reset();
    TxPad::function();
    RxPad::input(PinPull::up);
    return U2::configure(f, usart_divisor(pclk1, baud));
}

/// A ZERO WORD'S LOW RUN on USART2's transmit pad, in bit times, with
/// `ctlr4` and `m_ext` stored RAW - past the verbs, which refuse on a die
/// whose read-back says no - so a register that TAKES a write and reads
/// back zero is caught by what the wire does. TIM2's ruler on PA2, no
/// wire; the port disabled for the stores, both put back to zero after.
template <bool on>
uint32_t raw_low_run(const UartFormat& f, uint16_t ctlr4, uint16_t m_ext) {
    constexpr uint32_t baud = 9600;
    U2::enable(false);
    (void)U2::configure(f, usart_divisor(pclk1, baud));
    U2::regs().CTLR4 = ctlr4;
    U2::regs().CTLR1 = static_cast<uint16_t>(U2::regs().CTLR1 | m_ext);
    port_on<U2>();
    spin_us<Ruler>(3000);
    const uint32_t bit_cycles = (timclk1 / pclk1) * usart_divisor(pclk1, baud);
    const uint16_t psc = ruler_prescaler_for(bit_cycles * 14u);
    ruler_start(psc, true);
    ruler_rearm();
    U2::write_word(0);
    uint32_t run = 0;
    if (ruler_wait()) {
        run = ruler_cycles(ruler_width(), psc);
    }
    (void)tx_settled();
    ruler_stop();
    U2::enable(false);
    U2::regs().CTLR4 = 0;
    U2::regs().CTLR1 = static_cast<uint16_t>(U2::regs().CTLR1 & ~usart_m_ext_mask);
    return (run + bit_cycles / 2u) / bit_cycles;
}

/// THE FAR RECEIVER'S JUDGEMENT of `count` words the link's port sends
/// with `ctlr4` and `m_ext` stored raw: the peer sinks them at `far` and
/// reports its count, its parity errors and its sum. `spaced` leaves the
/// line idle for one whole far frame after each word: a short word's frame
/// (seven bits for a five-bit word) is shorter than the far receiver's ten,
/// and a word sent back to back puts its start bit inside the far frame's
/// data bits - one far frame of idle lets the far stop sample fall on idle
/// whatever the word's length. The sink's deadline, two frames a word,
/// covers the gap.
link::Report far_judges(const link::Cfg& far, const UartFormat& f, uint16_t word, uint16_t count, uint16_t ctlr4,
                        uint16_t m_ext, bool spaced = false) {
    link::Report r{};
    link::Params a{};
    a.cfg = far;
    a.count = count;
    a.ms = D::bound_ms(far.rate, 2u * count);
    if (!D::act(link::Op::sink, a) || !link_configured(f, far.rate)) {
        return r;
    }
    W::regs().CTLR4 = ctlr4;
    W::regs().CTLR1 = static_cast<uint16_t>(W::regs().CTLR1 | m_ext);
    port_on<W>();
    D::wait_for_window();
    for (uint16_t i = 0; i < count; ++i) {
        D::put(static_cast<uint8_t>(word));
        if (!D::wait_tc()) {
            break;   // a port that stopped shifting: the far count says so
        }
        if (spaced) {
            spin_us<Ruler>(10u * 1'000'000u / far.rate + 1u);   // one far 8N1 frame: ten bits
        }
    }
    W::enable(false);
    W::regs().CTLR4 = 0;
    W::regs().CTLR1 = static_cast<uint16_t>(W::regs().CTLR1 & ~usart_m_ext_mask);
    (void)D::report(r, 5);
    return r;
}

template <bool on = (usart_class_has_lot_registers && link_present)>
void to_lot_registers() {
    if constexpr (on) {
        if (!ready()) {
            done();
            return;
        }
        constexpr uint32_t baud = 9600;
        // Eight data bits and a parity bit: the frame whose parity bit
        // CHECK_SEL would hold at one or at zero.
        constexpr UartFormat parity_frame{UartBits::eight, UartParity::even};

        // ---- the probe, and every other register compared across it ----
        const bool up = near_configured(parity_frame, baud) && link_configured(parity_frame, baud);
        const UsartWords before2 = usart_words<U2>();
        const UsartWords before4 = usart_words<W>();
        const uint16_t word2 = U2::regs().CTLR4;
        const uint16_t word4 = W::regs().CTLR4;
        const bool present2 = U2::ctlr4_present();
        const bool present4 = W::ctlr4_present();
        const UsartWords after2 = usart_words<U2>();
        const UsartWords after4 = usart_words<W>();
        print(serial, "  CTLR4's address read ", hex(word2), " on USART2 and ", hex(word4), " on ",
              vx03link::port_kind, link_n, " before the probe; the probe says ", present2 ? "PRESENT" : "absent", " and ",
              present4 ? "PRESENT" : "absent", crlf);
        bench.verdict("the CTLR4 probe answers on both ports and leaves BRR, CTLR1..3 and GPR of each block "
                      "exactly as it found them, whatever the address turned out to be",
                      up && before2 == after2 && before4 == after4);

        // The two ends' rates on this core's counter, which the frames
        // themselves cannot tell: USART2's run on its free pad, the link
        // port's into the far sink, eleven bits a frame at 9600.
        port_on<U2>();
        spin_us<Ruler>(3000);
        const uint32_t pm2 = usart_permille<U2>(baud, 11, 16);
        uint32_t pm4 = 0;
        {
            link::Cfg far{};
            far.rate = baud;
            far.parity = 1;
            link::Params a{};
            a.cfg = far;
            a.count = 16;
            a.ms = D::bound_ms(baud, 32);
            if (D::act(link::Op::sink, a) && link_configured(parity_frame, baud)) {
                port_on<W>();
                D::wait_for_window();
                pm4 = usart_permille<W>(baud, 11, 16);
            }
            link::Report r{};
            (void)D::report(r, 5);
        }
        print(serial, "  16 frames of 8E1 at 9600 in ", pm2, " (USART2) and ", pm4, " (", vx03link::port_kind, link_n,
              ") thousandths of their wire time", crlf);
        bench.verdict("both ports run at the rate asked: 16 frames each in their wire time at 9600, no faster than "
                      "the divisor and at most 3 % slower",
                      wire_ok(pm2, baud) && wire_ok(pm4, baud));

        if (present2 && present4) {
            // MARK and the TRANSMITTED parity bit: a FINDING, not a verdict.
            // 18.10.8 calls CHECK_SEL a "checksum function select" and
            // MS_ERR a receive-mode flag, and says nothing of the
            // transmitter; on the CH32V303CBT6, the one die measured with
            // the register, the transmitted bit stays COMPUTED under MARK
            // (a zero byte under even parity reaches a far even receiver
            // with no error) while the RECEIVER flags a parity bit that is
            // not the selected level with MS_ERR. The leg below counts the
            // far receiver's parity errors and prints what they say.
            const link::Report zero =
                far_judges(link::Cfg{.parity = 1, .rate = baud}, parity_frame, 0x00, 8, usart_check_mark, 0);
            const link::Report one =
                far_judges(link::Cfg{.parity = 1, .rate = baud}, parity_frame, 0x01, 8, usart_check_mark, 0);
            print(serial, "  MARK into a far even receiver: 0x00 perr ", zero.perr, " of ", zero.count, ", 0x01 perr ",
                  one.perr, " of ", one.count, crlf);
            const bool held = zero.count == 8u && zero.perr == 8u && one.count == 8u && one.perr == 0u;
            const bool computed = zero.count == 8u && zero.perr == 0u && one.count == 8u && one.perr == 0u;
            print(serial, "  FINDING: under MARK the transmitted parity bit is ",
                  held ? "HELD HIGH" : computed ? "COMPUTED, as without the register" : "neither held nor computed",
                  crlf);
            judge("under MARK every frame reaches the far receiver (what the transmitted parity bit is under it is "
                  "the finding above)",
                  zero.count == 8u && one.count == 8u);
        } else {
            print(serial, "  this die's lot has no CTLR4 by the probe: the MARK and SPACE parity is asked of the "
                  "WIRE instead", crlf);
            bench.verdict("with the register absent every MARK and SPACE verb answers false and writes nothing",
                          !U2::mark_space(UsartMarkSpace::mark) && !W::mark_space(UsartMarkSpace::space) &&
                              !U2::mark_space_interrupt(true) && U2::mark_space() == UsartMarkSpace::off);
            // A register that took the write and reads back zero would
            // still hold the parity bit: MARK makes a zero byte's low run
            // nine bits under even parity (ten computed), SPACE ten under
            // odd parity (nine computed) - on USART2's own pad.
            const uint32_t even_plain = raw_low_run<on>(UartFormat{UartBits::eight, UartParity::even}, 0, 0);
            const uint32_t even_mark =
                raw_low_run<on>(UartFormat{UartBits::eight, UartParity::even}, usart_check_mark, 0);
            const uint32_t odd_plain = raw_low_run<on>(UartFormat{UartBits::eight, UartParity::odd}, 0, 0);
            const uint32_t odd_space =
                raw_low_run<on>(UartFormat{UartBits::eight, UartParity::odd}, usart_check_space, 0);
            print(serial, "  a zero byte's low run with CTLR4 stored raw: even parity ", even_plain, " bits plain and ",
                  even_mark, " under MARK; odd parity ", odd_plain, " plain and ", odd_space, " under SPACE", crlf);
            bench.verdict("THE PAD AGREES WITH THE PROBE: CTLR4 stored raw with MARK or SPACE leaves the parity bit "
                          "computed - ten low bits under even parity, nine under odd, as without it",
                          even_plain == 10u && even_mark == 10u && odd_plain == 9u && odd_space == 9u);
            // AND THE FAR RECEIVER: the link port with MARK stored raw sends
            // zero bytes into a far EVEN receiver - a held one would be a parity
            // error each - and with SPACE into a far ODD one.
            const link::Report mark =
                far_judges(link::Cfg{.parity = 1, .rate = baud}, parity_frame, 0x00, 8, usart_check_mark, 0);
            const link::Report space = far_judges(link::Cfg{.parity = 2, .rate = baud},
                                                  UartFormat{UartBits::eight, UartParity::odd}, 0x00, 8,
                                                  usart_check_space, 0);
            print(serial, "  ", vx03link::port_kind, link_n,
                  " with CTLR4 stored raw, zero bytes: MARK into a far even receiver perr ", mark.perr,
                  " of ", mark.count, ", SPACE into a far odd one perr ", space.perr, " of ", space.count, crlf);
            judge("THE FAR RECEIVER AGREES: the link port's parity bit stays computed under a raw MARK and a raw "
                  "SPACE - the register absent, not write-only",
                  mark.count == 8u && mark.perr == 0u && space.count == 8u && space.perr == 0u);
        }

        // ---- M_EXT: the short words ----
        U2::enable(false);
        W::enable(false);
        (void)U2::configure(UartFormat{}, usart_divisor(pclk1, baud));
        (void)W::configure(UartFormat{}, usart_divisor(wclk, baud));
        const bool short2 = U2::short_word(UsartShortWord::seven);
        const bool short4 = W::short_word(UsartShortWord::seven);
        (void)U2::short_word(UsartShortWord::off);
        (void)W::short_word(UsartShortWord::off);
        print(serial, "  M_EXT kept a seven-bit word: ", short2 ? "yes" : "no", " on USART2, ", short4 ? "yes" : "no",
              " on ", vx03link::port_kind, link_n, crlf);
        // A short word into a far 8N1 receiver: its missing top bits are
        // the stop bit's and the idle line's ones - 0x55 as seven bits reads
        // 0xD5, as five (0x15) reads 0xF5; a word left at eight bits reads
        // what was sent.
        const link::Cfg far8{};
        if (short2 && short4) {
            const link::Report seven = far_judges(far8, UartFormat{}, 0x55, 4, 0,
                                                  usart_ctlr1_short_word(UsartShortWord::seven), true);
            const link::Report five =
                far_judges(far8, UartFormat{}, 0x15, 4, 0, usart_ctlr1_short_word(UsartShortWord::five), true);
            print(serial, "  short words into a far 8N1 receiver, one far frame apart: 0x55 as seven bits summed ",
                  seven.sum, " over ", seven.count, ", 0x15 as five bits ", five.sum, " over ", five.count,
                  " (", 4u * 0xD5u, " and ", 4u * 0xF5u, " if each reads with the stop's ones on top)", crlf);
            judge("seven- and five-bit words reach a far 8N1 receiver with their top bits the stop's ones",
                  seven.count == 4u && seven.sum == 4u * 0xD5u && five.count == 4u && five.sum == 4u * 0xF5u);
        } else {
            bench.verdict("with M_EXT absent the field reads back zero and the verb answers false, the word left to M",
                          !short2 && !short4 && U2::short_word() == UsartShortWord::off);
            const uint32_t eight = raw_low_run<on>(UartFormat{}, 0, 0);
            const uint32_t seven = raw_low_run<on>(UartFormat{}, 0, usart_ctlr1_short_word(UsartShortWord::seven));
            const uint32_t five = raw_low_run<on>(UartFormat{}, 0, usart_ctlr1_short_word(UsartShortWord::five));
            print(serial, "  a zero word's low run with M_EXT stored raw: ", eight, " bits plain, ", seven,
                  " under the seven-bit code, ", five, " under the five-bit one", crlf);
            bench.verdict("THE PAD AGREES WITH THE READ-BACK: M_EXT stored raw leaves the word at M's eight bits - "
                          "nine low bits for a zero word under every code",
                          eight == 9u && seven == 9u && five == 9u);
            const link::Report fk = far_judges(far8, UartFormat{}, 0x00, 4, 0,
                                               usart_ctlr1_short_word(UsartShortWord::five));
            print(serial, "  ", vx03link::port_kind, link_n,
                  " with the five-bit code stored raw, zero words: the far chip counted ", fk.count,
                  ", sum ", fk.sum, ", ferr ", fk.ferr, crlf);
            judge("AND THE FAR RECEIVER: the link port's zero words under a raw five-bit code arrive as zero bytes, "
                  "the word eight bits long",
                  fk.count == 4u && fk.sum == 0u && fk.ferr == 0u);
        }

        // ---- RX_BUSY, caught while the far chip's frame arrives at 1200 ----
        {
            link::Params a{};
            a.cfg.rate = 1200;
            a.count = 1;
            a.value = 0x00;
            a.pattern = link::pattern_fixed;
            a.ms = 200;
            link::Cfg c{};
            c.rate = 1200;
            const bool acted = D::act(link::Op::send, a);
            const bool up4 = acted && D::configure(c);
            uint32_t busy_seen = 0, polls = 0;
            const uint32_t t0 = Ticker::millis();
            while (up4 && !W::rx_ready() && Ticker::millis() - t0 < 40u) {
                ++polls;
                if (W::receiving()) {
                    ++busy_seen;
                }
            }
            const bool came = W::rx_ready();
            spin_us<Ruler>(2'000);
            const bool busy_after = W::receiving();
            (void)W::read_word();
            link::Report r{};
            (void)D::report(r, 0);
            print(serial, "  RX_BUSY read set in ", busy_seen, " of ", polls, " polls while the far frame arrived, ",
                  busy_after ? "SET" : "clear", " after it", crlf);
            // WHERE THE DIE HAS IT is the driver's own probe's answer, not
            // the lot's: 18.10.1 and 18.10.8 give RX_BUSY to the dies that
            // have CTLR4 and to no other (ch32vx03/usart.hpp, "The lot's
            // registers"), so a die whose CTLR4 the probe found must show
            // the bit set while the 8.3 ms frame arrives, and one whose
            // CTLR4 it did not find must read it zero throughout.
            judge("RX_BUSY, where the die has it, stands while a frame arrives and falls after it; a die without it "
                  "reads zero throughout (the line above says which)",
                  up4 && came && !busy_after && (present4 ? busy_seen != 0u : busy_seen == 0u));
        }
        done();
    }
}

}  // namespace two

// ---------------------------------------------------------------------------
// p - UART5..UART8 with no wire
// ---------------------------------------------------------------------------

/// A byte read off a TRANSMIT pad by software: the start bit's fall found
/// by polling the pad's own input register, then each bit sampled at its
/// middle. Bit 8 set when the stop bit came back low; 0xFFFF when no
/// start bit came at all.
template <typename Pad>
uint16_t sample_tx(uint32_t baud, uint32_t timeout_us) {
    const uint32_t bit = SysClock::hz / baud;
    if (!wait_until<Ruler>([] { return !Pad::read(); }, timeout_us)) {
        return 0xFFFFu;
    }
    Stopwatch<Ruler> f;
    f.start();
    uint16_t v = 0;
    for (uint8_t i = 0; i < 8u; ++i) {
        const uint32_t at = bit + bit / 2u + static_cast<uint32_t>(i) * bit;
        while (f.elapsed() < at) {
        }
        if (Pad::read()) {
            v = static_cast<uint16_t>(v | (1u << i));
        }
    }
    while (f.elapsed() < 9u * bit + bit / 2u) {
    }
    return Pad::read() ? v : static_cast<uint16_t>(v | 0x100u);
}

/// An 8N1 frame banged into a RECEIVE pad through its own pull - letter
/// c's bit-banger on any pad.
template <typename Pad>
void bang_on(uint8_t byte, uint32_t baud) {
    const uint32_t bit = SysClock::hz / baud;
    Stopwatch<Ruler> w;
    w.start();
    uint32_t n = 0;
    Pad::clear();
    ++n;
    for (uint8_t i = 0; i < 8u; ++i) {
        while (w.elapsed() < n * bit) {
        }
        if (((byte >> i) & 1u) != 0u) {
            Pad::set();
        } else {
            Pad::clear();
        }
        ++n;
    }
    while (w.elapsed() < n * bit) {
    }
    Pad::set();
    while (w.elapsed() < (n + 1u) * bit) {
    }
}

template <uint8_t n>
struct UpperRig {
    using U = Usart<n>;
    using Tx = Pin<U::pads.tx.port, U::pads.tx.pin>;
    using Rx = Pin<U::pads.rx.port, U::pads.rx.pin>;
};

template <uint8_t n>
void upper_port_off() {
    using R = UpperRig<n>;
    Pfic::disable(R::U::irq);
    R::U::bus_clock(true);
    R::U::reset();
    R::U::bus_clock(false);
    R::Tx::release();
    R::Rx::release();
}

/// The rate every upper port is measured at.
constexpr uint32_t upper_baud = 9600;

/// One upper port measured with no wire; true when every step held.
template <uint8_t n>
bool upper_port() {
    using R = UpperRig<n>;
    using U = typename R::U;
    using S = V303State<device::has_usart(n)>;
    constexpr uint32_t baud = upper_baud;
    U::bus_clock(true);
    U::reset();
    const bool gate = Rcc::enabled(Bus::pb1, usart_gate_for(n));
    R::Tx::function();
    R::Rx::input(PinPull::up);
    const uint32_t brr = usart_divisor(pclk1, baud);
    const bool framed = U::configure(UartFormat{}, brr) && U::brr() == brr && !U::is_full;
    port_on<U>();
    spin_us<Ruler>(3000);   // TE's idle frame out
    // The rate on the pad, on the core's counter: no far end to share it.
    const uint32_t pm = usart_permille<U>(baud, 10, 16);
    S::upper_permille[n - 5u] = pm;

    // Out: a byte read back off the transmit pad by software.
    U::write_data(0xA5);
    const uint16_t out = sample_tx<typename R::Tx>(baud, 5000);
    // The frame's end before the next step; a frame that never ends shows
    // in `out` and in the vector's count below, which are judged.
    (void)wait_until<Ruler>([] { return U::tx_complete(); }, 5'000);

    // In: a frame banged into the receive pad through its pull.
    U::clear_by_read();
    bang_on<typename R::Rx>(0x3C, baud);
    (void)wait_until<Ruler>([] { return U::rx_ready(); }, 5'000);   // `in` asks RXNE itself
    const uint16_t status = U::status();
    const uint16_t in = U::rx_ready() ? static_cast<uint16_t>(U::read_word() & 0xFFu) : 0xFFFFu;

    // The vector: a transmission complete reaching this port's own entry.
    S::upper_interrupts[n - 5u] = 0;
    U::clear_flags(usart_tc);
    U::interrupts(usart_tcie, true);
    Pfic::enable(U::irq);
    U::write_data(0x11);
    spin_us<Ruler>(4000);
    const uint32_t vec = S::upper_interrupts[n - 5u];
    U::interrupts(usart_tcie, false);
    Pfic::disable(U::irq);

    print(serial, "  UART", n, " (TX P", U::pads.tx.port, U::pads.tx.pin, ", RX P",
          U::pads.rx.port, U::pads.rx.pin, ", vector ", static_cast<uint8_t>(U::irq),
          "): gate ", gate ? "open" : "SHUT", ", BRR ", U::brr(), ", out ", hex(out), ", in ",
          hex(in), " (status ", hex(status), "), vector ran ", vec, "; 16 frames in ", pm,
          " thousandths of their wire time", crlf);
    upper_port_off<n>();
    return gate && framed && out == 0xA5u && in == 0x3Cu &&
           (status & (usart_fe | usart_ne | usart_pe)) == 0u && vec >= 1u;
}

/// The transports whose columns land on the two-wire debug port's own
/// pads: UART8 on code 1 (PA14/PA15) and USART3 on code 2 (PA13/PA14).
/// Named through `on`, so no other part forms them.
template <bool on>
struct DebugPadPorts {
    using Uart8OnDebug = Uart<on ? 8 : 8, P, 16, 16, UartFormat{}, NoDmaEngine, NoDmaEngine, 1>;
    using Usart3OnDebug = Uart<on ? 3 : 3, P, 16, 16, UartFormat{}, NoDmaEngine, NoDmaEngine, 2>;
    static_assert(usart_column_on_debug_port(8, 1) && usart_column_on_debug_port(3, 2),
                  "test_vx03_serial: UART8's code 1 and USART3's code 2 are the columns on the "
                  "debug port's pads (tables 10-25 and 10-31)");
};

template <bool on = (device::has_usart(5) && device::has_usart(6) && device::has_usart(7) &&
                     device::has_usart(8))>
void tp_upper_ports() {
    if constexpr (on) {
        all_off();
        const bool five = upper_port<5>();
        const bool six = upper_port<6>();
        const bool seven = upper_port<7>();
        const bool eight = upper_port<8>();
        bench.verdict("UART5: its gate, its divisor, a byte off its transmit pad, a frame into its "
                      "receive pad and its own vector (69)",
                      five);
        bench.verdict("UART6, UART7 and UART8 the same, each on its own vector at 87, 88 and 89 "
                      "of this class's tail - with no wire, the receive side fed through the "
                      "pad's own pull because nothing else reaches it",
                      six && seven && eight);
        using S = V303State<on>;
        bench.verdict("THE FOUR RUN AT THE RATE ASKED: 16 frames out of each at 9600 in their wire "
                      "time on the core's counter, no faster than the divisor and at most 3 % slower - the pad read "
                      "by software above "
                      "samples a bit at its middle and would not see a few per cent",
                      wire_ok(S::upper_permille[0], upper_baud) && wire_ok(S::upper_permille[1], upper_baud) &&
                          wire_ok(S::upper_permille[2], upper_baud) && wire_ok(S::upper_permille[3], upper_baud));

        // THE COLUMNS ON THE DEBUG PORT'S PADS: with the probe attached,
        // SW_CFG reads as the two-wire port alive and init() refuses both
        // transports before it touches a register. Asked only where the
        // field says so - a refusal that failed would take the probe's
        // pads from it until the next reset.
        const uint8_t sw_cfg = Afio::debug_config();
        const bool alive = Afio::debug_port_enabled();
        bool refused8 = false;
        bool refused3 = false;
        if (alive) {
            refused8 = !DebugPadPorts<on>::Uart8OnDebug::init(clock, 9600);
            refused3 = !DebugPadPorts<on>::Usart3OnDebug::init(clock, 9600);
        }
        const bool still = Afio::debug_port_enabled() && Afio::debug_config() == sw_cfg;
        print(serial, "  SW_CFG reads ", sw_cfg, " with the probe attached (", alive ? "the "
              "two-wire port alive" : "the port GONE", "); UART8 on PA14/PA15 ",
              refused8 ? "refused" : "NOT refused", ", USART3 on PA13/PA14 ",
              refused3 ? "refused" : "NOT refused", crlf);
        bench.verdict("SW_CFG reads the two-wire debug port alive while the probe is attached, "
                      "and a transport whose column lands on its pads - UART8's code 1, USART3's "
                      "code 2 - is refused by init() and leaves the field as it found it",
                      alive && refused8 && refused3 && still);
        all_off();
    }
}

template <bool on>
void upper_off() {
    if constexpr (on) {
        if constexpr (device::has_usart(5) && device::has_usart(8)) {
            upper_port_off<5>();
            upper_port_off<6>();
            upper_port_off<7>();
            upper_port_off<8>();
        }
        if constexpr (device::dma_controller_count >= 2u) {
            Pfic::disable(dma_channel_irq(2, 3));
            Pfic::disable(dma_channel_irq(2, 5));
        }
        V303State<on>::engines = false;
    }
}

/// The link's own vector while one of the two-board key's transports owns
/// the port - letter 6's ring, letter n's engines: the receive engine's two
/// states live there (ch32vx03/usart.hpp), and a body that read DATAR
/// would take a byte from under the channel. False when nobody does.
template <bool on = link_present>
bool link_usart_vector() {
    if constexpr (on) {
        if (two::link_owner == 1u) {
            two::link_entries = two::link_entries + 1u;
            if (two::LinkRing::isr()) {
                two::link_edges = two::link_edges + 1u;
            }
            return true;
        }
        if (two::link_owner == 3u) {
            (void)two::LinkPlain::isr();
            return true;
        }
        if constexpr (two::link_on_dma2) {
            if (two::link_owner == 2u) {
                (void)two::LinkDma::isr();
                return true;
            }
        }
    }
    return false;
}

/// The link's DMA channels' vectors, for the same two transports.
template <bool on = link_present>
void link_dma_vector() {
    if constexpr (on) {
        if (two::link_owner == 1u) {
            if (two::LinkRing::dma_isr()) {
                two::link_edges = two::link_edges + 1u;
            }
        }
        if constexpr (two::link_on_dma2) {
            if (two::link_owner == 2u) {
                (void)two::LinkDma::dma_isr();
            }
        }
    }
}

/// An upper port's vector body: counted, and its flags put down.
template <uint8_t n>
void upper_vector() {
    if constexpr (device::has_usart(n)) {
        using U = Usart<n>;
        using S = V303State<device::has_usart(n)>;
        S::upper_interrupts[n - 5u] = S::upper_interrupts[n - 5u] + 1u;
        const uint16_t st = U::status();
        if ((st & usart_tc) != 0u) {
            U::clear_flags(usart_tc);
            U::interrupts(usart_tcie, false);
        }
        if ((st & usart_rxne) != 0u) {
            (void)U::read_word();
        }
    }
}

/// The upper ports' letter, registered where the part has UART5..UART8.
template <bool on = device::has_usart(5)>
void register_upper_letters() {
    if constexpr (on) {
        bench.letter('p', "UART5..UART8 with no wire: the pads, the frame, the vectors",
                     tp_upper_ports<>);
    }
}

/// The two-board key's letters, where the part is one end of the pair; n
/// where the link's engines are DMA2's, o on the CH32V303 class.
template <bool on = link_present>
void register_two_letters() {
    if constexpr (on) {
        bench.second_letter('1', "THE RENDEZVOUS: ident, NAK, desync, refusals, the two crystals both ways",
                            two::t1_link);
        bench.second_letter('2', "THE LADDER: echo to 4.5 Mbaud, each rate timed by the far chip, the peer's own",
                            two::t2_rates);
        bench.second_letter('3', "THE FRAMES: the matrix echoed, the stop lengths timed far", two::t3_frames);
        bench.second_letter('4', "THE ERRORS: PE, FE, NE, the tolerance, the start filter, ORE, LIN, drawn far",
                            two::t4_errors);
        bench.second_letter('5', "THE SHAPES: the far engine's frames, a stretched cell, this end's pull-banger",
                            two::t5_shapes);
        bench.second_letter('6', "THE RING: the receive engine under a second of the far blast to 4.5 Mbaud",
                            two::t6_engine);
        bench.second_letter('7', "HALF DUPLEX: HDSEL hearing a second chip, the wired AND", two::t7_half_duplex);
        bench.second_letter('8', "MUTE AND IRDA across the wire", two::t8_modes);
        bench.second_letter('9', "THE RING: rx_pending(), a broken stream's counts, the burst edge", two::t9_ring);
        if constexpr (two::link_on_dma2) {
            bench.second_letter('n', "UART4'S ENGINES ON DMA2 against the far chip: 4 KB each way",
                                two::tn_pair_engines<>);
        }
        if constexpr (usart_class_has_lot_registers) {
            bench.second_letter('o', "THE LOT'S REGISTERS: CTLR4, M_EXT, RX_BUSY, the far receiver's verdict",
                                two::to_lot_registers<>);
        }
    }
}

// ---- the menu ---------------------------------------------------------------

void banner() {
    print(serial, crlf, "test_vx03_serial - the USART chapter on ", device::part_name, crlf);
    print(serial, "  the console is USART1 at 115200; the instruments are USART2 (PA2/PA3) "
                  "and the fourth port", crlf);
    bench.menu();
}

}  // namespace

BRIO_CH32_LEAF_VECTOR(systick_handler) { brio::Ticker::tick(); }
BRIO_CH32_LEAF_VECTOR(usart1_handler) {
    (void)Serial::isr();
    console_entries = console_entries + 1u;
}

/// USART2's vector: the transport's body while letter l runs, and the
/// flag counter of every other letter.
BRIO_CH32_VECTOR(usart2_handler) {
    if (u2_transport) {
        u2_edge_entries = u2_edge_entries + 1u;
        if (u2_plain ? Plain::isr() : Loop::isr()) {
            u2_edges = u2_edges + 1u;
        }
        return;
    }
    const uint16_t st = U2::status();
    u2_interrupts = u2_interrupts + 1u;
    if ((st & brio::usart_txe) != 0u && (U2::regs().CTLR1 & brio::usart_txeie) != 0u) {
        u2_txe = u2_txe + 1u;
        U2::interrupts(brio::usart_txeie, false);
    }
    if ((st & brio::usart_tc) != 0u) {
        u2_tc = u2_tc + 1u;
        U2::clear_flags(brio::usart_tc);
    }
    if ((st & brio::usart_lbd) != 0u) {
        u2_lbd = u2_lbd + 1u;
        U2::clear_flags(brio::usart_lbd);
    }
    if ((st & brio::usart_cts) != 0u) {
        u2_cts = u2_cts + 1u;
        U2::clear_flags(brio::usart_cts);
        // DISARMED THE WAY TXE IS: a flag whose source stands would
        // re-enter this body without end, and the letter that wants the
        // next change arms it again itself.
        U2::cts_interrupt(false);
    }
    if ((st & brio::usart_pe) != 0u) {
        u2_pe = u2_pe + 1u;
    }
    if ((st & brio::usart_idle) != 0u) {
        u2_idle = u2_idle + 1u;
    }
    if ((st & brio::usart_rxne) != 0u) {
        u2_rxne = u2_rxne + 1u;
        u2_last_word = U2::read_word();
    } else if ((st & (brio::usart_pe | brio::usart_idle)) != 0u) {
        // The read sequence the chapter prescribes: the status register
        // is already read, so one data read clears what stands.
        (void)U2::read_word();
    }
}

/// The fourth port's vector, the one at this device class's own tail.
BRIO_CH32_LEAF_VECTOR(uart4_handler) {
    if (!has_fourth) {
        return;
    }
    if constexpr (vx03link::on_fourth_port) {
        if (link_usart_vector<>()) {
            return;   // a two-board transport owns the vector: its edge states
        }
    }
    u4_interrupts = u4_interrupts + 1u;
    const uint16_t st = U4::status();
    if ((st & brio::usart_tc) != 0u) {
        U4::clear_flags(brio::usart_tc);
    }
    if ((st & brio::usart_rxne) != 0u) {
        (void)U4::read_word();
    }
}

/// USART2's two DMA1 channels: the Loop transport's engines while letter l
/// or letter n runs. The engines switch their channels' lines on, so the
/// vectors are bound whether or not a letter uses them.
BRIO_CH32_VECTOR(dma1_channel6_handler) {
    if (u2_transport && !u2_plain && Loop::dma_isr()) {
        u2_edges = u2_edges + 1u;
    }
}
BRIO_CH32_VECTOR(dma1_channel7_handler) {
    if (u2_transport && !u2_plain) {
        (void)Loop::dma_isr();
    }
}

// UART4's two DMA2 channels, where UART4 is the link (the CH32V303RC and
// VC), and the four upper ports' vectors. Elsewhere these bodies are empty;
// on the CH32V203 nothing in the vector table names them.
BRIO_CH32_VECTOR(dma2_channel3_handler) {
    if constexpr (vx03link::on_fourth_port) {
        link_dma_vector<>();
    }
}
BRIO_CH32_VECTOR(dma2_channel5_handler) {
    if constexpr (vx03link::on_fourth_port) {
        link_dma_vector<>();
    }
}

/// USART3's link and its receive channel, DMA1's third, for the two-board
/// key's ring, where USART3 is the link (the CH32V203C8, the CH32V303CB).
/// Where UART4 is, nothing here is the link's and the bodies are empty.
/// USART3's body is a leaf wherever it is not empty (the vector guard
/// holds it to that on the CH32V303); the channel's calls out on the
/// CH32V303CB, so it takes the trampoline there and everywhere.
BRIO_CH32_LEAF_VECTOR(usart3_handler) {
    if constexpr (!vx03link::on_fourth_port) {
        (void)link_usart_vector<>();
    }
}
BRIO_CH32_VECTOR(dma1_channel3_handler) {
    if constexpr (!vx03link::on_fourth_port) {
        link_dma_vector<>();
    }
}
BRIO_CH32_LEAF_VECTOR(uart5_handler) { upper_vector<5>(); }
BRIO_CH32_LEAF_VECTOR(uart6_handler) { upper_vector<6>(); }
BRIO_CH32_LEAF_VECTOR(uart7_handler) { upper_vector<7>(); }
BRIO_CH32_LEAF_VECTOR(uart8_handler) { upper_vector<8>(); }

int main() {
    crystal_ok = SysClock::init();
    const bool clock_ok = crystal_ok || HsiClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    bench.letter('a', "the baud generator: every standard rate on both buses", ta_divisor);
    bench.letter('b', "the frame on the wire, measured by the timer", tb_frame);
    bench.letter('c', "the bit-banged line into the receiver", tc_receive);
    bench.letter('d', "mute mode: both wakes", td_mute);
    bench.letter('e', "LIN: the break sent and the break detected", te_lin);
    bench.letter('f', "half duplex on one wire, and no pull-up on it", tf_half_duplex);
    bench.letter('g', "IrDA: the encoder's pulse and the decoder", tg_irda);
    if constexpr (has_full_fourth) {
        bench.letter('h', "the synchronous mode on the fourth port's clock pad", th_synchronous);
        bench.letter('i', "the smartcard: its register face and its card clock", ti_smartcard);
    }
    bench.letter('j', "the flags and the two vectors", tj_flags);
    bench.letter('k', "hardware flow control: the CTS hold and the RTS hand", tk_flow);
    bench.letter('l', "the loopback PA2-PA3, when it is strapped", tl_loopback);
    // The crossed pair's letters where the part has a fourth port and the
    // board keeps the pair: the CH32V203C8 board's PA2-PB1 and PB0-PA3. On
    // the CH32V303RC and VC the pair is retired - UART4's PC10/PC11 are the
    // SPI link's - and UART4's engines and the lot's registers are letters
    // n and o against the far chip instead.
    if constexpr (crossed_pair_part) {
        bench.letter('m', "the crossed pair, when it is strapped", tm_cross);
        bench.letter('q', "errors under the engine on the pair: breaks in a stream, nothing stolen",
                     tq_errors);
    }
    bench.letter('r', "tx_idle() against the last stop bit on the pad", tr_tx_idle);
    if constexpr (crossed_pair_part) {
        bench.letter('s', "the burst edge from the vector on the pair, nothing polled", ts_edge);
    }
    register_upper_letters();
    register_two_letters();

    if (serial_ok) {
        print(serial, crlf, "boot: clk=",
              crystal_ok ? "HSE 8 MHz x18 = 144 MHz" : (clock_ok ? "HSI x18 (NO CRYSTAL)" : "FAILED"),
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
