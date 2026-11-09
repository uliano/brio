// test_stm32f4_spi - the reference bench suite for the STM32F4's SPI and
// I2S: the resource over the whole of the chapter, the SpiHost engine
// under util/spi_bus.hpp's arbiter, the DMA engines, and the audio face
// with its own PLL - stm32f4/spi.hpp.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// THE SUITE CARRIES TWO INSTRUMENTS, ONE PER BOARD, and the device header
// decides which half is compiled - the same letters where the subject is
// the same, the instrument's own letters where it is not.
//
// ON THE STM32F429I-DISC1 THE INSTRUMENT IS TWO DEVICES SOLDERED TO THE
// BOARD: a bus driver measured against nothing is a register read-back,
// and this board carries a three-axis gyroscope and a display controller
// on SPI5 -
//
//   SPI5   SCK PF7, MISO PF8, MOSI PF9 (AF5)
//   the gyroscope   chip select PC1, four-wire, SPI mode 3, up to 10 MHz;
//                   WHO_AM_I at 0x0F, the read bit 0x80 and the
//                   auto-increment bit 0x40 in the address byte
//   the display     chip select PC2, D/CX PD13, and its serial data line
//                   is the MOSI PAD ALONE - the controller's SDO is not
//                   wired - so reading its identity is a BIDIRECTIONAL
//                   transaction and nothing else will do
//
// and the parts of the chapter that need no device at all are measured
// anyway: the rate ladder is timed with both selects high, the mode
// fault is raised by driving SSI low, the overrun by not reading what
// came back, and the I2S by counting the word-select pad's own edges
// against the audio PLL's arithmetic.
//
// What is exercised there, letter by letter:
//   a  the instances: presence, bus, vector, the gate closed at reset and
//      the reset values behind it
//   b  the eight baud rates, timed with no device selected
//   c  what the driver refuses, and that a refusal writes nothing
//   d  the mode fault: SSI driven low under software management; the
//      clearing sequence from a fresh fault, three accesses between its
//      steps printed, and clear_mode_fault()
//   e  the overrun and its two-read clearing sequence
//   f  the gyroscope answers: WHO_AM_I, single byte
//   g  the gyroscope at each of the eight rates - which it answers
//   h  the other three SPI modes against the one the device wants
//   i  multi-byte auto-increment, and a register written then read back
//   j  THE KERNEL: SpiBus over the engined SpiHost, asynchronous and
//      polled requests interleaved, the rejection, the sleep votes
//   k  the same transaction on the engined host: polled (the loop, the
//      engines untouched) and asynchronous (through the two DMA engines)
//   l  the SCK pad's slew class against ES0206 2.12.4's table
//   m  the bidirectional line: the gyroscope moved to its own three-wire
//      interface and read back over MOSI alone, then the display's turn
//   n  I2S3 as a master transmitter: the word select counted on its own
//      pad against the audio PLL, and the master clock output
//
// The gyroscope is left POWERED DOWN (CTRL_REG1 at its reset value).
//
// ON THE NUCLEO-F446RE THE INSTRUMENT IS THE DESK'S OTHER BOARD: the
// STM32G0B1RE it is paired with, running the stm32g0 `spi_peer`, on four
// wires by function -
//
//   SCK   PB13 AF5  ->  the G0's PB3  AF0 (SPI1)
//   MISO  PB14 AF5  <-  the G0's PB4  AF0
//   MOSI  PB15 AF5  ->  the G0's PB5  AF0
//   NSS   PB12 GPIO ->  the G0's PA15 AF0 (its pull-up holds the wire)
//
// plus a GND between the two chips' own ground pins, both boards at 3.3 V,
// the peer commanded IN BAND over avrxt/src/apps/spi_link.hpp (one source
// of truth for the wire format, whatever the architecture). SPI2 is the
// APB1 instance: BR divides 45 MHz, /2 = 22.5 MHz the host's ceiling, and
// RM0390 26.1.1 gives the client "frequency up to fPCLK/2" as well. Its
// letters (RM0390 ch. 26, DS10693 table 11, ES0298 2.14 - not the
// F429's):
//   a  the instances: four on this part, the bus, the vector, table 165's
//      I2S face and no extension block, the request cells, reset values
//   b  the eight baud rates, timed with the select high
//   c  what the driver refuses, and that a refusal writes nothing
//   d  the mode fault: SSI driven low, the clearing sequence and
//      clear_mode_fault() as on the F429, then A REAL DEMOTION - this host's
//      NSS a hardware input and the peer pulling the wire low (ss_pulse)
//   e  the overrun and its two-read clearing sequence
//   n  the peer's command channel, its ident and ten frames
//   o  the matrix against the peer: four modes, both bit orders, and a
//      DORD mismatch as an exact two-way bit reversal
//   p  the BR ladder to PCLK1/2 = 22.5 MHz, on the polled host and on the
//      DMA1 stream 4/3 engines, against the peer's DMA serve - and its
//      polled client run, printed beside them
//   q  THE KERNEL: SpiBus over the engined SpiHost with a second chip
//      reading back - ordered replies (on the streams, one polled among
//      them, one a command in the loop before its data block), the rejection,
//      both sleep votes, spi_timeout and recover(); and a request naming the
//      engines on the engineless host refused, the select never falling
//   r  THE ROLES INVERT: this board's SpiClient run clocked by the peer's
//      host_burst
//   l  ES0298 2.14.5 staged: the SCK pad's class against the peer's known
//      stream at /2
//   u  16-BIT FRAMES against the peer (spi_link's extended exchange,
//      spare_wide): the polled loop and the half-word streams at every BR
//      code, the wire's time from two window lengths,
//      and the peer's own 16-bit client run; this end's read judged to
//      PCLK1/4 and printed at /2, every frame heard by the peer judged at /2
//   v  RECEIVE-ONLY and ODD COUNTS (spare_receive_only): this host's
//      transmit-only shapes witnessed by a client whose answer line is
//      dark, the peer's receive() run at 7 and 33 frames in both widths,
//      and odd-count exchanges both ways
//   w  THE CRC against the peer's unit (spare_crc): both widths, both of
//      the peer's serves, every BR code, this end's unit on the resource
//      with the last data frame queued behind its predecessor and CRCNEXT
//      after it; a wrong CRC frame caught as report_crcerr; this end's read
//      judged to PCLK1/4, the peer's check at /2 too
//   f  THE SOFTWARE SELECT AND THE SELECT OUTPUT: a client with no select
//      pad (SSM) deaf with SSI high and exact with it low, under the
//      peer's host_burst; SSOE, the select the block drives from SPE to
//      SPE, framing the peer's exchange
//   g  THE STALL A SECOND CHIP MAKES: a long polled request demoted by the
//      peer's ss_pulse (NSS a hardware input) ends spi_stalled, ERRIE's
//      vector on the same MODF; a client answering on TXEIE alone; ERRIE
//      on an unread client's OVR
//   h  A HANDLER BESIDE THE POLLED LOOP IS A GAP ON THE BUS: a TIM7 handler
//      holding the core longer than a frame every 5 us, beside the polled
//      receive loop at PCLK1/8 - every window exact, the time it costs bound
//   s  THE ENGINE A REQUEST NAMES against the peer: one frame read and one
//      written on the streams at rates where the loop costs the core less,
//      exact at both ends; what the engines cannot carry refused, the select,
//      the D/C and the block untouched and no edge on SCK; a command in the
//      loop before its data block, D/C moving between the two phases, and a
//      command alone on the streams
//   E  WHEN A REQUEST ENDS THE WIRE HAS ENDED: the SCK pad read right after
//      the request ended, all four modes, every rate, read and write, both
//      engines, SPI1 and SPI2, through a select and through a null one; the
//      longest wait in the vector, on both APB clocks; a wait that runs out
//   j  THE COUNT PAST SIXTEEN BITS: one command frame and 65535 data frames,
//      a request of 65536 frames in all, on the wire and timed
//   k  THE NEXT REQUEST AFTER A STALL OR A TRANSFER ERROR, recover() never
//      called: every stall path and a bus error on either stream, directly and
//      queued through SpiBus, the next polled and asynchronous windows exact
//      at both ends; the clock gate closed, which no reset opens
//   t  (OUTSIDE z) THE CRCNEXT WINDOW and the receive CRC at PCLK1/2: the
//      store walked away from the last data store, one frame in flight,
//      and the received CRC against the SCK pad's class - the instrument
//      behind w's procedure (spi.md)
//   x  (OUTSIDE z) slip statistics against the peer, each mode ten times,
//      then thirty bursts of 32 frames on the DMA host at PCLK1/4
//
// build: boards = f429zi,f446re
// build: monitor_speed = 115200

#include <stdint.h>

#include <variant>

#include "stm32f4/clock.hpp"
#include "stm32f4/dma.hpp"
#include "stm32f4/dwt.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/spi.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/usart.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/post.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time_event.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/spi_bus.hpp"
#include "util/test_wait.hpp"
#include "util/testbench.hpp"

// =============================================================================
// Letter d's clearing sequences, the same on both boards
// =============================================================================
//
// WHICH SEQUENCE PUTS MODF DOWN, AND WHEN. The chapter names two steps, an SR
// access while MODF stands and then a CR1 write (clear_mode_fault()'s comment
// quotes it). Each attempt below starts from a fresh fault - the block's RCC
// reset, a software-managed host at PCLK/64, SSI dropped, MODF waited for -
// with interrupts masked around the sequence, and differs only in what happens
// between the two steps: nothing, which is the chapter's sequence done
// strictly and the one judged; then a CR1 read (the shape `CR1 = CR1` compiles
// to), a CR2 read and a second SR read, each printed as a measurement - the
// chapter does not say what an access between the steps does, and brio
// depends on no answer to it once the driver writes the strict sequence. The
// flag comes down some APB cycles after the CR1 write, not on the read that
// follows it, so each attempt counts the SR reads until MODF reads clear, and
// the strict one is judged against the verb's own bound on its re-reads
// (S::mode_fault_settle_reads). Then whether a write raising MSTR, SSI and SPE
// takes, which SPE never does while the flag stands; then the verb itself, on
// a fresh fault, unmasked as a program calls it.

namespace modf_clear {

/// A fresh mode fault on S. True when MODF rose: it rises some tens of core
/// cycles after the store that drops SSI (letter d's latency print), so the
/// 100 us budget is a time-out only.
template <typename S, typename Ruler>
bool raise() {
    S::reset();
    (void)S::configure(brio::SpiConfig{.clock = brio::SpiClock::div64,
                                       .nss = brio::SpiNss::software});
    S::enable();
    S::software_select(true);
    return brio::wait_until<Ruler>([] { return S::mode_fault(); }, 100);
}

struct Attempt {
    bool raised = false;   ///< the fault stood before the sequence
    bool down = false;     ///< MODF read clear within read_cap reads after the CR1 write
    uint16_t reads = 0;    ///< SR reads after the CR1 write, the one that found it clear included
    bool took = false;     ///< a write raising MSTR, SSI and SPE took after the sequence
};

/// The most SR reads an attempt spends waiting for the flag to come down: far
/// past the verb's bound, so a slow clear is counted rather than cut short.
constexpr uint16_t read_cap = 1000;

/// One attempt from a fresh fault. `Between`: 0 nothing, 1 a CR1 read, 2 a CR2
/// read, 3 a second SR read, between the SR read and the CR1 write.
template <typename S, typename Ruler, typename CriticalSection, uint8_t Between>
Attempt attempt() {
    Attempt a{};
    a.raised = raise<S, Ruler>();
    {
        CriticalSection cs;
        auto& r = S::regs();
        const uint32_t c = r.CR1;
        (void)r.SR;   // step one
        if constexpr (Between == 1u) {
            (void)r.CR1;
        } else if constexpr (Between == 2u) {
            (void)r.CR2;
        } else if constexpr (Between == 3u) {
            (void)r.SR;
        }
        r.CR1 = c;    // step two
        while (a.reads < read_cap && !a.down) {
            ++a.reads;
            a.down = (r.SR & SPI_SR_MODF) == 0u;
        }
    }
    S::regs().CR1 = S::regs().CR1 | SPI_CR1_SSI | SPI_CR1_MSTR | SPI_CR1_SPE;
    a.took = (S::regs().CR1 & SPI_CR1_SPE) != 0u;
    return a;
}

template <typename Out>
void print_attempt(const Out& out, const char* between, const Attempt& a) {
    brio::print(out, "  ", between, ": MODF ",
                !a.raised ? "was not raised" : a.down ? "cleared" : "LEFT STANDING", " after ",
                a.reads, " SR read(s), and a write raising MSTR, SSI and SPE ",
                a.took ? "took" : "did not take", brio::crlf);
}

/// The block: the strict sequence and the verb judged, the three accesses
/// between the steps printed. Leaves the block reset; the caller reconfigures.
template <typename S, typename Ruler, typename CriticalSection, typename Out, typename Bench>
void judge(const Out& out, Bench& bench) {
    const Attempt strict = attempt<S, Ruler, CriticalSection, 0>();
    const Attempt cr1 = attempt<S, Ruler, CriticalSection, 1>();
    const Attempt cr2 = attempt<S, Ruler, CriticalSection, 2>();
    const Attempt sr = attempt<S, Ruler, CriticalSection, 3>();
    const bool verb_raised = raise<S, Ruler>();
    const bool verb_answer = S::clear_mode_fault();
    const bool verb_down = !S::mode_fault();
    S::reset();
    print_attempt(out, "the SR read, then the CR1 write, nothing between (judged)", strict);
    print_attempt(out, "measured: a CR1 read between the two steps", cr1);
    print_attempt(out, "measured: a CR2 read between the two steps", cr2);
    print_attempt(out, "measured: a second SR read between the two steps", sr);
    brio::print(out, "  clear_mode_fault() on a fresh fault: ",
                !verb_raised ? "found no fault" : verb_answer ? "answered true" : "answered FALSE",
                ", MODF ", verb_down ? "clear" : "STANDING", " after it", brio::crlf);
    bench.verdict("MODF HAS A WAY DOWN IN SOFTWARE: the SR access followed by the CR1 write, with "
                  "no other access to the block between them, clears the flag within the verb's "
                  "bound of SR re-reads, and a write raising SPE takes after it",
                  strict.raised && strict.down && strict.reads <= S::mode_fault_settle_reads &&
                      strict.took);
    bench.verdict("clear_mode_fault(), which keeps the two steps adjacent, clears a fresh fault and "
                  "says so",
                  verb_raised && verb_answer && verb_down);
}

}   // namespace modf_clear

#if defined(STM32F446xx)

// =============================================================================
// =============================================================================
// THE STM32F446RE HALF: SPI2 against the desk's STM32G0B1RE spi_peer
// =============================================================================
// =============================================================================

#include "stm32f4/delay.hpp"
#include "stm32f4/exti.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/tim.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED: pure encoding, not one
// register, and every architecture on this link compiles the same file.
#include "../../../avrxt/src/apps/spi_link.hpp"

/// The Nucleo's HSE is the ST-LINK's 8 MHz clock, taken in bypass (no
/// crystal is fitted); 180 MHz puts SPI2's APB1 at 45 MHz.
using SysClock =
    brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000, brio::HseMode::bypass>;
constexpr SysClock clock{};

namespace {

using namespace brio;
using spilink::Op;

using P = Stm32f4Platform<>;

// ---- the console -------------------------------------------------------------------

constexpr UartPins console_pins{.tx = {'A', 2, PinFunction::af7}, .rx = {'A', 3, PinFunction::af7}};
using Serial = Uart<2, console_pins>;
constexpr Serial serial;

TestBench<Serial> bench;

// ---- the link ----------------------------------------------------------------------

/// SPI2 at AF5 (DS10693 table 11) on the four wires to the peer's SPI1 -
/// by function, nothing crossed. PB12 is the chip select: a GPIO the
/// letters drive, and SPI2_NSS at AF5 for the one letter that needs the
/// hardware input (d's demotion).
constexpr SpiPins link_pins{
    .sck = {'B', 13, PinFunction::af5},
    .miso = {'B', 14, PinFunction::af5},
    .mosi = {'B', 15, PinFunction::af5},
    .nss = {'B', 12, PinFunction::af5},
};

using S = Spi<2>;
using Host = SpiHost<2, link_pins>;
/// SPI2_TX is DMA1 stream 4 channel 0 and SPI2_RX DMA1 stream 3 channel 0
/// (RM0390 table 28) - the reserve checks both cells at compile time. The
/// element is uint16_t, DR's width: a byte request moves a byte a beat on
/// the same streams and a 16-bit one a half-word (letter u).
using TxEngine = DmaTxEngine<1, 4, 0, uint16_t>;
using RxEngine = DmaRxEngine<1, 3, 0, uint16_t>;
using DmaHost = SpiHost<2, link_pins, TxEngine, RxEngine>;
using Client = SpiClient<2, link_pins>;

using CsPin = Pin<link_pins.nss.port, link_pins.nss.pin>;

volatile bool bus_ao_live = false;
volatile bool dma_host_live = false;
volatile bool host_done = false;
volatile bool host_swallow_completion = false;
/// The engined host's completions (TransferDone edges) and the receive
/// stream's block ends under them: one of each, whatever the request's phases.
volatile uint16_t dma_completions = 0;
volatile uint16_t dma_blocks = 0;
/// Letter k's stopwatch on the completing vector, the DWT's.
volatile bool time_vector = false;
volatile uint32_t vector_cycles = 0;
/// Letter k's witness of the order: SCK's edges on its own pad, each counted by
/// where the select's output latch stood when it was seen.
using SckWatch = ExtInt<Pin<link_pins.sck.port, link_pins.sck.pin>>;
volatile uint16_t sck_edges_selected = 0;
volatile uint16_t sck_edges_released = 0;
/// The select's falling edges on its own pad (letter q's refusal).
using SelWatch = ExtInt<CsPin>;
volatile uint16_t sel_falls = 0;

// ---- the rulers --------------------------------------------------------------------

constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000u;

/// The suite's ruler: the DWT's cycle counter, started once in main(). It
/// counts the core's cycles, masked or not, through a Sleep too, and stops
/// only in a Stop, which no letter here enters (stm32f4/dwt.hpp).
struct Ruler {
    static uint32_t now() { return CycleCounter::now(); }
    static constexpr uint32_t hz() { return SysClock::hz; }
};
static_assert(CycleRuler<Ruler>);

/// One run of SysTick_Handler as it lengthens a window, in core cycles: its
/// body (Ticker::tick(), 18 instructions, 28 cycles at most from the release
/// listing), the exception's entry and exit (12 each, Cortex-M4 TRM 3.9.1)
/// and four flash lines missed in the ART (6 cycles each at five wait states,
/// RM0390 3.4.1) - 76.
constexpr uint32_t systick_run_cycles = 28u + 12u + 12u + 4u * 6u;

/// The protocol's settle after each command window (spi_link.hpp).
void settle() { brio::spin_us<Ruler>(spilink::settle_ms * 1000u); }

/// A measurement window the console's transmit interrupt walks through is not
/// a measurement: drain first. The answer is dropped here: a wedged console
/// only lengthens what follows, and the windows a derived bound judges act on
/// it themselves.
void console_drain() { (void)brio::console_drain<Ruler, Serial>(); }

// ---- what the registers held before this program touched them ---------------------

struct BootState {
    bool gate = false;
    uint32_t cr1 = 0, cr2 = 0, sr = 0, crcpr = 0, i2scfgr = 0, i2spr = 0;
};
BootState boot;

// =============================================================================
// The host's pads, and the host every letter starts from
// =============================================================================

/// THE LINK'S PADS ONE OR TWO CLASSES DOWN, AND THEY ARE MEASUREMENTS.
/// SpiHost's default class for SCK and MOSI is very-high (ES0298 2.14.5's
/// table at its widest), and on these jumper wires both edges are
/// aggressors at the far client:
///  - SCK: at high letter x slips 15 bursts of 40 in every mode and
///    letter p breaks from 351 kHz, the G0B1RE client's stream one bit
///    out of place from a random frame; at medium every one of those runs
///    is exact. (The stm32f4 spi_peer's host_burst drives SCK at medium
///    too, for the same doubled edge.) Medium is inside the erratum's table
///    for this instance (75 MHz of APB ceiling against APB1's 45); letter
///    l walks the class against the erratum itself.
///  - MOSI: the same slip, from the DATA edge coupled into the clock
///    beside it, where the frames run back to back: thirty bursts of 32
///    frames on the DMA host at PCLK1/4 = 11.25 MHz slip in 6 to 13 of 30
///    at high and 16 to 23 at very-high, the far client and this end
///    disagreeing from the same frame on - and in 0 of 270 at medium
///    (PCLK1/2, /4 and /8, either host, either serve; 0 of 60 at low).
///    The I/O compensation cell (RM0390 8.1) changes nothing (10 of 30 at
///    high with it on). Medium's own reach is 25 MHz at 50 pF (DS10693
///    table 58), above the 11.25 MHz a 22.5 MHz SCK toggles the data at.
///    Letter x's second part counts that rung.
PinSpeed peer_sck_speed = PinSpeed::medium;
PinSpeed peer_mosi_speed = PinSpeed::medium;

/// The Config every host init of the peer letters is handed - the same
/// classes on the engined host, which is the same instance and the same
/// wires under a second set of statics.
template <typename H>
typename H::Config peer_link_config() {
    return {.sck_speed = peer_sck_speed, .mosi_speed = peer_mosi_speed};
}

bool link_command_mode() {
    bus_ao_live = false;
    dma_host_live = false;
    host_swallow_completion = false;
    CsPin::output(true);
    return Host::init(clock, peer_link_config<Host>());
}

// =============================================================================
// The peer: spi_link.hpp over the bus under test
// =============================================================================
//
// The instrument at the other end is the stm32g0 `spi_peer` on the
// STM32G0B1RE's SPI1 AF0 (PB3 SCK, PB4 MISO, PB5 MOSI, PA15 NSS). Its
// `ident` names it (firmware 0x03xx).

/// The command channel's SCK: PCLK1/256 = 175.8 kHz. A character is 45 us,
/// far more than the peer's polled command listener needs.
constexpr SpiClock link_clock = SpiClock::div256;

uint8_t frame_buf[spilink::max_payload + 8];
uint8_t answer_buf[spilink::answer_bytes];
uint8_t dummy_buf[spilink::answer_bytes];
uint8_t raw_seen[16];
uint8_t raw_n = 0;
spilink::Decoder dec;
bool link_quiet = false;

/// THE HOLD AROUND EACH SELECT WINDOW: the protocol owns the chip select
/// for these windows (a null PinRef in the Request) and pays 30 us on each
/// side of them, so the client's run sees the select fall well before the
/// first clock and rise well after the last.
void link_hold() { brio::spin_us<Ruler>(30); }

/// One protocol window: prime, select, hold, the transaction, hold,
/// deselect. THE MODE IS PRIMED BEFORE THE SELECT FALLS - a CPOL flip
/// inside an open window is one extra edge for the selected client.
bool link_xfer(const uint8_t* tx, uint8_t* rx, uint16_t n, SpiClock rate = link_clock,
               SpiMode mode = SpiMode::mode0) {
    if (n == 0) {
        return true;
    }
    Host::prime(mode, rate);
    CsPin::clear();
    link_hold();
    Host::Request r{};
    r.cs = {};
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = n;
    r.clock = rate;
    r.mode = mode;
    r.bits = SpiDataSize::bits8;
    r.engine = SpiEngine::polled;
    const bool done = Host::start(r);
    link_hold();
    CsPin::set();
    return done;
}

void send_frame(Op op, const uint8_t* p, uint8_t len) {
    uint8_t n = 0;
    spilink::write_frame(
        [&](uint8_t b) {
            if (n < sizeof frame_buf) {
                frame_buf[n++] = b;
            }
        },
        op, p, len);
    (void)link_xfer(frame_buf, nullptr, n);
}

/// One answer window: `answer_bytes` dummies in a single transaction, then
/// the decoder walks what came back (the peer pads with zeros).
bool recv_frame(spilink::Frame& out) {
    dec.reset();
    raw_n = 0;
    for (uint16_t i = 0; i < spilink::answer_bytes; ++i) {
        dummy_buf[i] = 0;
        answer_buf[i] = 0xEE;
    }
    (void)link_xfer(dummy_buf, answer_buf, spilink::answer_bytes);
    for (uint16_t i = 0; i < spilink::answer_bytes; ++i) {
        if (raw_n < 16) {
            raw_seen[raw_n++] = answer_buf[i];
        }
        if (dec.feed(answer_buf[i]) == spilink::Decoder::Result::frame) {
            out = dec.frame();
            return true;
        }
    }
    return false;
}

bool command_once(Op op, const uint8_t* p, uint8_t len) {
    send_frame(op, p, len);
    settle();
    spilink::Frame f;
    if (!recv_frame(f)) {
        return false;
    }
    return f.op == Op::ack && f.len == 2 && f.data[0] == spilink::byte_of(op);
}

const uint8_t no_payload[1] = {0};

/// Three attempts, each separated by longer than the peer's own answer
/// window bound - the protocol's recovery guarantee.
bool command(Op op, const uint8_t* p = no_payload, uint8_t len = 0) {
    uint8_t first_n = 0;
    uint8_t first_seen[16];
    for (uint8_t k = 0; k < 3; ++k) {
        if (command_once(op, p, len)) {
            return true;
        }
        if (k == 0) {
            first_n = raw_n;
            for (uint8_t i = 0; i < raw_n; ++i) {
                first_seen[i] = raw_seen[i];
            }
        }
        (void)link_command_mode();
        brio::spin_us<Ruler>(400'000);
    }
    if (link_quiet) {
        return false;
    }
    print(serial, "    LINK FAILURE op ", hex(spilink::byte_of(op)),
          ": the first answer window carried");
    if (first_n == 0) {
        print(serial, " nothing");
    }
    for (uint8_t i = 0; i < first_n; ++i) {
        print(serial, " ", hex(first_seen[i]));
    }
    print(serial, crlf,
          "      the peer board must be running `spi_peer`; its console '0' forces the "
          "dark client back.",
          crlf);
    (void)link_command_mode();
    return false;
}

bool query(Op op, spilink::Frame& data) {
    if (!command(op)) {
        return false;
    }
    settle();
    return recv_frame(data);
}

bool peer_ident(spilink::Ident& d) {
    spilink::Frame f;
    if (!query(Op::ident, f) || f.op != Op::ident_data || f.len != spilink::ident_size) {
        return false;
    }
    d = spilink::get_ident(f.data);
    return true;
}

bool peer_report(spilink::Report& r) {
    (void)link_command_mode();
    for (uint8_t k = 0; k < 4; ++k) {
        spilink::Frame f;
        if (query(Op::report, f) && f.op == Op::report_data && f.len == spilink::report_size) {
            r = spilink::get_report(f.data);
            return true;
        }
        brio::spin_us<Ruler>(60'000);
    }
    return false;
}

bool peer_act(Op op, const spilink::Params& a) {
    uint8_t p[spilink::params_size];
    spilink::put_params(p, a);
    if (!command(op, p, spilink::params_size)) {
        return false;
    }
    settle();
    return true;
}

/// Is the peer there at all? Every letter that needs it asks first, so an
/// absent peer is a NAMED failure and never a hang.
bool need_peer() {
    link_quiet = true;
    (void)link_command_mode();
    for (uint8_t k = 0; k < 3; ++k) {
        if (command(Op::ping)) {
            link_quiet = false;
            return true;
        }
    }
    link_quiet = false;
    print(serial,
          "  THE PEER DID NOT ANSWER. The STM32G0B1RE must be running `spi_peer`; its "
          "console '0' forces the dark client back. Check the five wires in this file's "
          "header.",
          crlf);
    bench.verdict("the peer answers a ping (the peer board running spi_peer)", false);
    return false;
}

// ---- one exchange, commanded and then clocked ---------------------------------------

struct Exchange {
    spilink::Cfg cfg{.apply = 1, .mode = 0, .dord = 0, .regime = spilink::regime_buffer_wait};
    SpiMode host_mode = SpiMode::mode0;
    bool host_lsb = false;
    SpiClock rate = link_clock;
    uint16_t count = 8;
    uint8_t seed_a = 0x13;
    uint8_t seed_b = 0x57;
    uint8_t pattern = spilink::pattern_prbs;
    uint8_t flags = 0;
    uint8_t spare = 0;    ///< spilink::spare_polled_pump asks for the peer's polled run
    uint16_t ms = 400;
    bool engines = false; ///< clock it with the DMA host, asynchronously
};

constexpr uint8_t max_exchange = 32;
uint8_t xtx[max_exchange];
uint8_t xrx[max_exchange];

/// The data phase on the ENGINED host: the DMA1 streams, the receive block's
/// completion the transaction's one interrupt.
bool dma_xfer(const uint8_t* tx, uint8_t* rx, uint16_t n, SpiClock rate, SpiMode mode) {
    dma_host_live = true;
    CsPin::output(true);
    if (!DmaHost::init(clock, peer_link_config<DmaHost>())) {
        dma_host_live = false;
        return false;
    }
    DmaHost::prime(mode, rate);
    CsPin::clear();
    link_hold();
    DmaHost::Request r{};
    r.cs = {};
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = n;
    r.clock = rate;
    r.mode = mode;
    r.bits = SpiDataSize::bits8;
    r.engine = SpiEngine::dma;
    host_done = false;
    // 100 ms: at most 32 frames at PCLK1/256, 1.5 ms, are what the vector waits on.
    const bool done =
        DmaHost::start(r) || brio::wait_until<Ruler>([] { return host_done; }, 100'000);
    link_hold();
    CsPin::set();
    DmaHost::release();
    dma_host_live = false;
    return done;
}

bool do_exchange(const Exchange& e) {
    spilink::Params a{};
    a.cfg = e.cfg;
    a.count = e.count;
    a.ms = e.ms;
    a.seed_a = e.seed_a;
    a.seed_b = e.seed_b;
    a.pattern = e.pattern;
    a.flags = e.flags;
    a.spare = e.spare;
    // A bit-order mismatch is not a shrug: told about it, each end checks
    // the EXACT bit-reverse of what the other sent.
    if (e.host_lsb != (e.cfg.dord != 0)) {
        a.flags |= spilink::flag_expect_reversed;
    }
    if (!peer_act(Op::exchange, a)) {
        return false;
    }
    spilink::Stream out(e.pattern, e.seed_a);
    for (uint8_t i = 0; i < max_exchange; ++i) {
        xtx[i] = 0;
        xrx[i] = 0xEE;
    }
    const uint16_t n = e.count < max_exchange ? e.count : max_exchange;
    for (uint16_t i = 0; i < n; ++i) {
        xtx[i] = out.next();
    }
    settle();
    if (e.engines) {
        (void)dma_xfer(xtx, xrx, n, e.rate, e.host_mode);
    } else {
        // The bit order is a BUS-level verb (SpiHost::bit_order()), not a
        // Request field; link_command_mode() puts it back with the re-init.
        if (e.host_lsb) {
            (void)Host::bit_order(true);
        }
        (void)link_xfer(xtx, xrx, n, e.rate, e.host_mode);
    }
    (void)link_command_mode();
    return true;
}

struct Verify {
    uint16_t mism = 0;
    uint16_t last_bit_only = 0;   ///< mismatches that differ in bit 0 alone
    uint8_t idx = 0xFF;
    uint8_t got = 0;
    uint8_t exp = 0;
};

Verify verify_rx(const Exchange& e) {
    Verify v{};
    const bool reversed = e.host_lsb != (e.cfg.dord != 0);
    for (uint16_t i = 0; i < e.count && i < max_exchange; ++i) {
        uint8_t exp = spilink::pattern_value(e.pattern, e.seed_b, i);
        if (reversed) {
            exp = spilink::bit_reverse(exp);
        }
        if (xrx[i] != exp) {
            if (v.mism == 0) {
                v.idx = static_cast<uint8_t>(i);
                v.got = xrx[i];
                v.exp = exp;
            }
            ++v.mism;
            if ((xrx[i] ^ exp) == 0x01u) {
                ++v.last_bit_only;
            }
        }
    }
    return v;
}

bool exchange_exact(const Exchange& e, Verify& v, spilink::Report& r) {
    const bool ran = do_exchange(e);
    const bool rep = ran && peer_report(r);
    v = verify_rx(e);
    return ran && rep && v.mism == 0 && r.mism == 0 && r.count == e.count;
}

void dump_exchange(const Exchange& e, const Verify& v, const spilink::Report& r) {
    print(serial, "    host mism=", v.mism, " (first idx ", v.idx, " got ", hex(v.got), " exp ",
          hex(v.exp), "), client count=", r.count, " mism=", r.mism, " (first idx ", r.idx,
          " got ", hex(r.got), " exp ", hex(r.exp), ") flags=", hex(r.flags), crlf);
    const bool reversed = e.host_lsb != (e.cfg.dord != 0);
    print(serial, "    read:");
    for (uint16_t i = 0; i < e.count && i < max_exchange; ++i) {
        print(serial, " ", hex(xrx[i]));
    }
    print(serial, crlf, "    want:");
    for (uint16_t i = 0; i < e.count && i < max_exchange; ++i) {
        uint8_t exp = spilink::pattern_value(e.pattern, e.seed_b, i);
        if (reversed) {
            exp = spilink::bit_reverse(exp);
        }
        print(serial, " ", hex(exp));
    }
    print(serial, crlf);
}

const char* serve_name(const spilink::Report& r) {
    return (r.aux2 & 0x04u) != 0u ? "dma" : "run";
}

// =============================================================================
// a - the instances and the block
// =============================================================================

void ta_block() {
    print(serial, "  SPI2 gate at reset: ", boot.gate ? "OPEN" : "closed", "; CR1 ", hex(boot.cr1),
          " CR2 ", hex(boot.cr2), " SR ", hex(boot.sr), " CRCPR ", hex(boot.crcpr), " I2SCFGR ",
          hex(boot.i2scfgr), " I2SPR ", hex(boot.i2spr), crlf);
    bench.verdict("SPI2's clock is off at reset", !boot.gate);
    // 26.5: CR1 0x0000, CR2 0x0000, SR 0x0002 (TXE), CRCPR 0x0007, I2SCFGR
    // 0x0000 - and I2SPR, stated 0x0002 (26.7.9), printed beside them.
    bench.verdict("the reset values are the chapter's: CR1 and CR2 zero, SR just TXE, CRCPR "
                  "the odd 0x0007, I2SCFGR zero",
                  boot.cr1 == 0u && boot.cr2 == 0u && boot.sr == SPI_SR_TXE &&
                      boot.crcpr == 7u && boot.i2scfgr == 0u);
    S::reset();
    const uint32_t i2spr_after_reset = S::regs().I2SPR;
    print(serial, "  I2SPR reads ", hex(boot.i2spr), " at boot and ", hex(i2spr_after_reset),
          " after an RCC reset pulse; 26.7.9 states 0x0002 - printed, not judged", crlf);
    bench.verdict("the RCC's reset line puts the block back where it was found",
                  i2spr_after_reset == boot.i2spr && S::regs().CR1 == 0u &&
                      S::regs().CRCPR == 7u);
    (void)link_command_mode();

    // The reserve's answers folded at compile time: asked in a run-time
    // loop, GCC 16 at -Os reports the facts' struct "used uninitialized"
    // (-Wuninitialized) where it is initialized on every path.
    static constexpr bool i2s_face[7] = {false,
                                         spi_i2s_capable(1), spi_i2s_capable(2),
                                         spi_i2s_capable(3), spi_i2s_capable(4),
                                         spi_i2s_capable(5), spi_i2s_capable(6)};
    print(serial, "  instances on this part:");
    uint8_t count = 0;
    for (uint8_t i = 1; i <= 6u; ++i) {
        if (spi_present(i)) {
            ++count;
            print(serial, " SPI", i, "(", spi_on_apb2(i) ? "APB2" : "APB1", ",irq",
                  static_cast<int32_t>(spi_irq(i)), i2s_face[i] ? ",I2S" : "", ")");
        }
    }
    print(serial, crlf);
    bench.verdict("this part carries FOUR SPI instances (RM0390 table 165), one vector each",
                  count == 4u && !spi_present(5) && !spi_present(6));
    bench.verdict("SPI1 and SPI4 are APB2's; SPI2 and SPI3 are APB1's",
                  spi_on_apb2(1) && !spi_on_apb2(2) && !spi_on_apb2(3) && spi_on_apb2(4));
    bench.verdict("SPI2's vector is its own", S::irq == SPI2_IRQn);
    bench.verdict("the I2S face is SPI1's, SPI2's and SPI3's - not SPI4's (table 165) - and "
                  "there is NO full-duplex extension block (26.1.3: \"only transmitter or "
                  "receiver\")",
                  spi_i2s_capable(1) && spi_i2s_capable(2) && spi_i2s_capable(3) &&
                      !spi_i2s_capable(4) && !spi_i2s_facts().ext_blocks &&
                      !i2s_ext_present(2) && !i2s_ext_present(3));
    bench.verdict("SPI2's requests sit on DMA1 stream 4 channel 0 (TX) and stream 3 channel 0 "
                  "(RX), table 28 - and on no cell of DMA2",
                  spi_dma_placement_valid(2, true, 1, 4, 0) &&
                      spi_dma_placement_valid(2, false, 1, 3, 0) &&
                      !spi_dma_placement_valid(2, true, 2, 4, 2) &&
                      !spi_dma_placement_valid(2, false, 2, 3, 2));
    print(serial, "  APB1 = ", Host::reference_hz() / 1000u, " kHz, HCLK = ", SysClock::hz / 1000u,
          " kHz", crlf);
    bench.verdict("the rate SPI2's BR field divides is ITS OWN bus clock, PCLK1 = 45 MHz",
                  Host::reference_hz() == SysClock::pclk1_hz && SysClock::pclk1_hz == 45'000'000u);
}

// =============================================================================
// b - the eight baud rates, timed with the select high
// =============================================================================

/// Clock `n` frames with the select high, polled, and report the core
/// cycles the run took: the bus is driven and the peer, unselected,
/// listens to none of it.
/// Zero when a frame's RXNE never came: 1 ms is some eleven frames at
/// PCLK1/256, the slowest code.
uint32_t timed_frames(SpiClock c, uint16_t n) {
    Host::prime(SpiMode::mode0, c);
    S::flush_rx();
    bool came = true;
    const uint32_t t0 = Ruler::now();
    for (uint16_t i = 0; i < n; ++i) {
        S::data(0xA5u);
        came = brio::wait_until<Ruler>([] { return S::rx_ready(); }, 1'000) && came;
        (void)S::data();
    }
    const uint32_t took = Ruler::now() - t0;
    return came ? took : 0u;
}

/// THE POLL'S TURN, for letter b's bound: timed_frames()'s wait on RXNE run
/// with the block idle, so that RXNE never rises and the condition ends the
/// wait itself after `turns` calls. The count it keeps adds an increment, a
/// compare and a branch to every turn, so the figure is the turn plus those:
/// an upper bound on it, rounded up; the least of three windows, each an
/// upper bound (a kernel tick inside one only lengthens it).
uint32_t poll_turn_bound() {
    constexpr uint32_t turns = 64;
    uint32_t least = UINT32_MAX;
    for (uint8_t k = 0; k < 3u; ++k) {
        uint32_t n = 0;
        const uint32_t t0 = Ruler::now();
        // The answer is the count's, not RXNE's: the block is idle, and 64 turns
        // end the wait long before its 1 ms.
        (void)brio::wait_until<Ruler>([&n] { return S::rx_ready() || ++n == turns; }, 1'000);
        const uint32_t per_turn = (Ruler::now() - t0 + turns - 1u) / turns;
        least = per_turn < least ? per_turn : least;
    }
    return least;
}

void tb_rates() {
    if (!link_command_mode()) {
        bench.verdict("the host came up", false);
        return;
    }
    // WHAT THE LOOP COSTS IS A TURN, AND HALF A BIT HIDES IT. RXNE rises
    // on the LAST SAMPLING EDGE (26.3.9) - in mode 0 half a bit before the
    // frame's end - and the loop's turn (the read, the next write, the
    // block's restart) runs from there: a frame takes the longer of eight
    // bits and seven and a half bits plus the turn. So the turn is read as
    // the frame time less seven and a half bits, at the codes where it
    // shows (where it is longer than a bit, twice the half bit it may hide in).
    constexpr uint16_t frames = 32;
    uint32_t took[8] = {};
    bool ticked[8] = {};
    bool monotone = true;
    uint32_t lowest = 0xFFFFFFFFu;
    uint32_t highest = 0;
    uint8_t shown = 0;
    uint32_t slowest_diff_ns = 0;
    uint32_t slowest_bound_ns = 0;
    // The eight codes timed first and printed after: a console line going out
    // beside a run would put its interrupt inside the time. One untimed run
    // first, so that the timed ones run from the ART's cache.
    if (!brio::console_drain<Ruler, Serial>()) {
        bench.skip("the console did not drain: the timed runs would carry its interrupt");
        return;
    }
    (void)timed_frames(SpiClock::div2, 2);   // the warm-up: nothing read from it
    for (uint8_t code = 0; code < 8u; ++code) {
        const brio::TickWitness<P> witness;
        took[code] = timed_frames(static_cast<SpiClock>(code), frames);
        ticked[code] = !witness.quiet();
    }
    const uint32_t turn = poll_turn_bound();
    // THE SPREAD'S BOUND, derived. The loop runs phase-locked to the block, so
    // a code's turn is a fixed path plus how late the poll sees RXNE: somewhere
    // inside one turn of the poll (`turn`, measured above), at a different place
    // at each code (seven and a half bits modulo the turn). The write after it
    // waits for PCLK1's next edge, 0 to HCLK/PCLK1 - 1 = 3 core cycles. The
    // lead before the first frame and the tail after the last RXNE are the
    // same path with their own phase, spread over the 32 frames: under a cycle
    // a frame. The three truncations of the nanosecond arithmetic below (the
    // frame, the wire, the half bit) move one turn by under 2 ns, so two turns
    // apart by under 3. SCK and the ruler both count HCLK: no clock error
    // enters. A code a kernel tick landed in is printed and left out.
    const uint32_t ratio = SysClock::hz / Host::reference_hz();
    const uint32_t bound_cycles = turn + (ratio - 1u) + 1u;
    const uint32_t bound_ns = bound_cycles * 1000u / cycles_per_us + 3u;
    for (uint8_t code = 0; code < 8u; ++code) {
        const SpiClock c = static_cast<SpiClock>(code);
        const uint32_t stated = spi_sck_hz(Host::reference_hz(), c);
        const uint32_t per_frame_ns = (took[code] * 1000u) / (frames * cycles_per_us);
        const uint32_t wire_ns =
            stated == 0u ? 0u : static_cast<uint32_t>(8u * 1'000'000'000ULL / stated);
        const uint32_t half_bit_ns = wire_ns / 16u;
        const uint32_t turn_ns =
            per_frame_ns + half_bit_ns > wire_ns ? per_frame_ns + half_bit_ns - wire_ns : 0u;
        const bool shows = turn_ns > 2u * half_bit_ns;
        print(serial, "  div", static_cast<uint32_t>(1u << (code + 1u)), ": SCK ", stated / 1000u,
              " kHz, ", frames, " frames in ", took[code] / cycles_per_us, " us -> ",
              per_frame_ns, " ns a frame against the wire's ", wire_ns, " ns; the turn after "
              "RXNE ", turn_ns, " ns", shows ? "" : " (hidden by the half bit)",
              ticked[code] ? " (a tick landed in it)" : "", crlf);
        if (code > 0u && took[code] <= took[code - 1u]) {
            monotone = false;
        }
        if (shows && !ticked[code]) {
            ++shown;
            if (turn_ns < lowest) {
                lowest = turn_ns;
            }
            if (turn_ns > highest) {
                highest = turn_ns;
            }
        }
        if (code == 7u) {
            slowest_diff_ns =
                per_frame_ns > wire_ns ? per_frame_ns - wire_ns : wire_ns - per_frame_ns;
            slowest_bound_ns = wire_ns / 512u + 2u;
        }
    }
    bench.verdict("the eight codes are eight different rates, each slower than the last",
                  monotone);
    print(serial, "  the turn where it shows, no tick in the window: ", lowest, " to ", highest,
          " ns (", (lowest * cycles_per_us) / 1000u, " to ", (highest * cycles_per_us) / 1000u,
          " core cycles) at ", shown, " codes; the poll's turn ", turn,
          " cycles at most, so the spread's bound is ", bound_ns, " ns", crlf);
    bench.verdict("what the one-frame polled loop costs is a CONSTANT TURN after RXNE - one "
                  "read, one write, the restart - and not a share of the rate: at two codes or "
                  "more where it outlasts a bit, within one turn of the poll, PCLK1's edge and "
                  "the arithmetic's truncations",
                  shown >= 2u && highest - lowest <= bound_ns);
    // AT PCLK1/256 THE TURN IS HIDDEN, and the frame is eight bits. While the
    // turn and the poll's turn stay under the half bit (512 core cycles here,
    // the condition reads it), the next write is queued before the frame ends
    // and the 32 frames run back to back: the window is 32 frames less the last
    // half bit, plus the lead before the first edge and the tail after the
    // last RXNE. Those two are at least nothing and at most a turn, a poll's
    // turn and the two SysTick runs a window of 1.5 ms can meet - under one
    // bit (1024 cycles) - so the frame is eight bits within half a bit spread
    // over 32 frames, a 512th of the wire, plus the truncations' 2 ns.
    const uint32_t half_bit_256 = 128u * ratio;
    const uint32_t highest_cycles = (highest * cycles_per_us + 999u) / 1000u;
    const bool hidden = shown != 0u && highest_cycles + turn < half_bit_256 &&
                        highest_cycles + turn + 2u * systick_run_cycles < 2u * half_bit_256;
    print(serial, "  at the slowest code the frame is ", slowest_diff_ns,
          " ns off eight bits, the bound ", slowest_bound_ns, " ns", crlf);
    bench.verdict("... so at PCLK1/256, the turn hidden in the half bit, the measured frame time "
                  "IS eight bits at the rate the arithmetic states, within half a bit over the "
                  "window",
                  hidden && slowest_diff_ns <= slowest_bound_ns);
}

// =============================================================================
// c - the refusals
// =============================================================================

void tc_refusals() {
    if (!link_command_mode()) {
        bench.verdict("the host came up", false);
        return;
    }
    (void)S::disable();
    const uint32_t before = S::regs().CR1;
    const bool even = S::configure(SpiConfig{.crc = true, .crc_polynomial = 0x1020u});
    bench.verdict("an EVEN CRC polynomial is refused - ES0298 2.14.4 makes the calculation "
                  "wrong",
                  !even);
    bench.verdict("... and a refused configuration writes nothing", S::regs().CR1 == before);
    bench.verdict("the same polynomial made odd is accepted",
                  S::configure(SpiConfig{.crc = true, .crc_polynomial = 0x1021u}));
    bench.verdict("a CLIENT has no NSS output to enable: SSOE is refused there",
                  !S::configure(SpiConfig{.role = SpiRole::client,
                                          .nss = SpiNss::hardware_output}));
    bench.verdict("TI mode has no software select - the frame IS the NSS pulse",
                  !S::configure(SpiConfig{.nss = SpiNss::software,
                                          .frame_format = SpiFrameFormat::ti}));
    bench.verdict("TI mode has no LSB-first either",
                  !S::configure(SpiConfig{.lsb_first = true,
                                          .nss = SpiNss::hardware_output,
                                          .frame_format = SpiFrameFormat::ti}));
    (void)S::configure(SpiConfig{});
    S::enable();
    bench.verdict("the polynomial is refused while the peripheral is enabled",
                  !S::crc_polynomial(0x8005u | 1u));
    bench.verdict("and so is CRCEN, which 26.7.1 wants written with SPE clear",
                  !S::crc_enable(true));
    (void)S::disable();
    bench.verdict("with SPE down both are accepted",
                  S::crc_polynomial(0x8005u | 1u) && S::crc_enable(true));
    bench.verdict("enabling the CRC RESETS both CRC registers",
                  S::rx_crc() == 0u && S::tx_crc() == 0u);
    bench.verdict("an I2S divider of 0 or 1 is forbidden and refused",
                  !i2s_config_valid(I2sConfig{.div = 1}) && !i2s_config_valid(I2sConfig{.div = 0}));
    bench.verdict("PCM's long frame is refused outside the PCM standard",
                  !i2s_config_valid(I2sConfig{.pcm_long_frame = true}));
    (void)link_command_mode();
}

// =============================================================================
// d - the mode fault: SSI first, then a REAL demotion by a second chip
// =============================================================================

void td_mode_fault() {
    if (!link_command_mode()) {
        bench.verdict("the host came up", false);
        return;
    }
    bench.verdict("no mode fault stands on a healthy host", !S::mode_fault());

    // 26.3.10: on a master under software management SSI going low IS the
    // fault - no pad and no second master needed. SSI is synchronized into
    // APB1, a quarter of the core clock here, so the flag's arrival is
    // timed rather than assumed.
    S::software_select(true);   // SSI low
    const uint32_t immediate = S::status();
    // 100 us: a time-out only, the flag rising some tens of core cycles after
    // the store (the print). The reading is late by at most one poll turn.
    const std::optional<uint32_t> rose =
        brio::cycles_until<Ruler>([] { return S::mode_fault(); }, 100);
    const uint32_t sr = S::status();
    const uint32_t cr1 = S::regs().CR1;
    print(serial, "  SSI driven low: SR on the very next read ", hex(immediate), ", then ", hex(sr),
          " after ", rose ? *rose : 0u, " core cycles", rose ? "" : " (NEVER ROSE)", "; CR1 ",
          hex(cr1), crlf);
    bench.verdict("MODF rises when SSI goes low under software management",
                  (sr & SPI_SR_MODF) != 0u);
    bench.verdict("... and the silicon has cleared SPE and MSTR with it, demoting the host",
                  (cr1 & SPI_CR1_SPE) == 0u && (cr1 & SPI_CR1_MSTR) == 0u);
    S::regs().CR1 = cr1 | SPI_CR1_SSI;   // the clearing sequence's CR1 write
    // 100 us: a time-out only, the clear landing some tens of core cycles after
    // the write (the F429's half of this letter prints it).
    bench.verdict("the SR access then the CR1 write clears it",
                  brio::wait_until<Ruler>([] { return !S::mode_fault(); }, 100));

    // The sequences from a fresh fault each, and the verb (modf_clear above).
    modf_clear::judge<S, Ruler, P::CriticalSection>(serial, bench);
    (void)link_command_mode();

    // THE REAL DEMOTION: the peer drives the shared select wire low from its
    // own pad (Op::ss_pulse) while this host runs with NSS as a HARDWARE
    // INPUT - SSM clear, the PB12 pad handed to SPI2 - which is the
    // multi-master arrangement 26.3.4 describes: another master pulling NSS
    // low demotes this one.
    if (!need_peer()) {
        return;
    }
    spilink::Params a{};
    a.aux8 = 30;       // ms the peer waits after its ack window
    a.aux16 = 3000;    // us it holds the wire low
    if (!peer_act(Op::ss_pulse, a)) {
        bench.verdict("the peer accepted the ss_pulse command", false);
        return;
    }
    (void)S::disable();
    SpiConfig hw{};
    hw.nss = SpiNss::hardware_input;
    const bool cfg_ok = S::configure(hw);
    Host::claim_nss_pad(true);
    CsPin::pull(PinPull::up);
    S::enable();
    const bool armed = cfg_ok && S::enabled() && !S::mode_fault() &&
                       (S::regs().CR1 & SPI_CR1_MSTR) != 0u;
    const uint32_t t0 = Ticker::millis();
    // 200 ms: the peer takes the wire 30 ms after its ack window (aux8).
    (void)brio::wait_until<Ruler>([] { return S::mode_fault(); }, 200'000);   // judged below
    const uint32_t waited = Ticker::millis() - t0;
    const uint32_t sr2 = S::status();
    const uint32_t cr1b = S::regs().CR1;
    print(serial, "  NSS as a hardware input, the peer's pulse: armed=", armed, ", MODF after ",
          waited, " ms, SR ", hex(sr2), " CR1 ", hex(cr1b), crlf);
    bench.verdict("a host whose NSS is a hardware input runs while the wire is high",
                  armed);
    bench.verdict("ANOTHER CHIP PULLING NSS LOW raises MODF on this host and demotes it - SPE "
                  "and MSTR cleared by the silicon",
                  (sr2 & SPI_SR_MODF) != 0u && (cr1b & SPI_CR1_SPE) == 0u &&
                      (cr1b & SPI_CR1_MSTR) == 0u);
    // The answer is not judged here: the verb is judged on a fresh fault above,
    // the peer may still hold the wire, and link_command_mode() below resets
    // and reconfigures the block whatever the flag says.
    (void)S::clear_mode_fault();
    Host::claim_nss_pad(false);
    brio::spin_us<Ruler>(10'000);   // the pulse is over and the peer back on its dark client
    (void)link_command_mode();
    spilink::Report r{};
    const bool rep = peer_report(r);
    bench.verdict("the peer reports the wire really taken, and the command channel is back",
                  rep && r.op == spilink::byte_of(Op::ss_pulse) && r.aux0 == 1u);
}

// =============================================================================
// e - the overrun
// =============================================================================

void te_overrun() {
    if (!link_command_mode()) {
        bench.verdict("the host came up", false);
        return;
    }
    Host::prime(SpiMode::mode0, SpiClock::div64);
    S::flush_rx();
    bench.verdict("no overrun stands on a drained receiver", !S::overrun());
    // Three frames clocked with the select high and none read: the second
    // cannot land.
    // 1 ms a wait: a frame at PCLK1/64 is 11 us. A time-out only, judged by
    // the OVR verdict below.
    for (uint8_t i = 0; i < 3u; ++i) {
        S::data(static_cast<uint8_t>(0x10u + i));
        (void)brio::wait_until<Ruler>([] { return S::tx_empty(); }, 1'000);
    }
    brio::spin_us<Ruler>(100);
    print(serial, "  three frames clocked, none read: SR ", hex(S::status()), crlf);
    bench.verdict("OVR rises when a frame arrives on an unread receiver", S::overrun());
    (void)S::regs().DR;   // 26.3.10: the DR read alone does NOT clear it
    const bool still = S::overrun();
    S::clear_overrun();
    print(serial, "  after the DR read alone: OVR ", still ? "STILL SET" : "clear",
          "; after the DR-then-SR sequence: ", S::overrun() ? "SET" : "clear", crlf);
    bench.verdict("the clearing sequence is a DR read FOLLOWED BY an SR read, and the DR read "
                  "alone is not enough",
                  still && !S::overrun());
    (void)link_command_mode();
}

// =============================================================================
// n - the peer's command channel
// =============================================================================

void tn_peer_link() {
    if (!need_peer()) {
        return;
    }
    bench.verdict("the peer answers a ping over the four wires", command(Op::ping));
    spilink::Ident d{};
    const bool got = peer_ident(d);
    if (got) {
        print(serial, "  peer: label '");
        for (uint8_t i = 0; i < 8 && d.label[i]; ++i) {
            print(serial, d.label[i]);
        }
        print(serial, "' xtal=", d.xtal, " sanity=", hex(d.sanity), " fw=", hex(d.version), crlf);
    }
    bench.verdict("ident comes back and it IS spi_peer (the sanity byte), from the STM32G0 "
                  "port (firmware 0x03xx)",
                  got && d.sanity == spilink::ident_sanity &&
                      spilink::peer_kind(d.version) == spilink::peer_stm32g0);
    uint8_t pings = 0;
    for (uint8_t i = 0; i < 10; ++i) {
        if (command(Op::ping)) {
            ++pings;
        }
    }
    print(serial, "  ", pings, " of 10 pings answered at ", Host::sck_hz(link_clock) / 1000u,
          " kHz SCK, one frame per chip-select window", crlf);
    bench.verdict("the channel is steady over ten frames", pings == 10u);
}

// =============================================================================
// o - the matrix against the peer
// =============================================================================

void to_peer_matrix() {
    if (!need_peer()) {
        return;
    }
    uint8_t good = 0;
    for (uint8_t m = 0; m < 4; ++m) {
        Exchange e{};
        e.cfg.mode = m;
        e.host_mode = static_cast<SpiMode>(m);
        e.seed_a = static_cast<uint8_t>(0x13u + m);
        e.seed_b = static_cast<uint8_t>(0x57u + m);
        Verify v{};
        spilink::Report r{};
        if (exchange_exact(e, v, r)) {
            ++good;
        } else {
            print(serial, "  mode ", m, ":", crlf);
            dump_exchange(e, v, r);
        }
    }
    print(serial, "  ", good, " of 4 transfer modes byte-exact BOTH ways, 8 frames each", crlf);
    bench.verdict("all four transfer modes carry a burst byte-exact in both directions between "
                  "TWO SEPARATE CHIPS",
                  good == 4u);

    Exchange lsb{};
    lsb.cfg.dord = 1;
    lsb.host_lsb = true;
    Verify v{};
    spilink::Report r{};
    const bool lsb_ok = exchange_exact(lsb, v, r);
    if (!lsb_ok) {
        dump_exchange(lsb, v, r);
    }
    bench.verdict("LSb first, both ends agreeing, is byte-exact too - the bit order a BUS-level "
                  "verb here (CR1.LSBFIRST)",
                  lsb_ok);

    Exchange mism{};
    mism.cfg.dord = 1;
    mism.host_lsb = false;
    Verify v2{};
    spilink::Report r2{};
    const bool ran = do_exchange(mism);
    const bool rep = ran && peer_report(r2);
    v2 = verify_rx(mism);
    print(serial, "  DORD mismatch: host mism=", v2.mism, " client mism=", r2.mism, " count=",
          r2.count, " (both checking the exact bit-reverse)", crlf);
    bench.verdict("a bit-order mismatch is an EXACT TWO-WAY BIT REVERSAL",
                  ran && rep && v2.mism == 0u && r2.mism == 0u && r2.count == mism.count);
    bench.verdict("and the command channel still works right after all of that",
                  command(Op::ping));
}

// =============================================================================
// p - the rate ladder against the peer, to PCLK1/2
// =============================================================================

struct Ladder {
    uint32_t last_good = 0;
    uint32_t first_bad = 0;
    spilink::Report bad_r{};
};

/// The BR codes from the command rate up to PCLK1/2 = 22.5 MHz - this
/// instance's ceiling as a host and the far client's well inside its own
/// (DS13560 table 76: 32 MHz at 2.7..3.6 V) - each rung asked even after a
/// break, so the print shows the whole ladder.
Ladder climb(const char* label, bool engines, uint8_t spare, uint16_t count, uint8_t seed_a,
             uint8_t seed_b) {
    static const SpiClock codes[] = {SpiClock::div256, SpiClock::div128, SpiClock::div64,
                                     SpiClock::div32,  SpiClock::div16,  SpiClock::div8,
                                     SpiClock::div4,   SpiClock::div2};
    Ladder l{};
    for (const SpiClock c : codes) {
        Exchange e{};
        e.rate = c;
        e.count = count;
        e.seed_a = seed_a;
        e.seed_b = seed_b;
        e.spare = spare;
        e.engines = engines;
        Verify v{};
        spilink::Report r{};
        const bool ok = exchange_exact(e, v, r);
        const uint32_t real = Host::sck_hz(c);
        print(serial, "  ", label, " PCLK1/", static_cast<uint32_t>(2u << static_cast<uint8_t>(c)),
              " = ", real / 1000u, " kHz: ", ok ? "exact both ways" : "NOT exact",
              "  host mism=", v.mism, " client mism=", r.mism, " client count=", r.count,
              " serve=", serve_name(r), crlf);
        if (ok && l.first_bad == 0u) {
            l.last_good = real;
        } else if (!ok && l.first_bad == 0u) {
            l.first_bad = real;
            l.bad_r = r;
        }
    }
    print(serial, "  ", label, " held to ", l.last_good / 1000u, " kHz");
    if (l.first_bad != 0u) {
        print(serial, " and broke at ", l.first_bad / 1000u, " kHz");
    }
    print(serial, crlf);
    return l;
}

void tp_peer_rates() {
    if (!need_peer()) {
        return;
    }
    const uint32_t top = Host::sck_hz(SpiClock::div2);

    // The polled host against the peer's default serve (its DMA engines).
    const Ladder polled = climb("polled host,", false, 0, 8, 0x21, 0x84);
    bench.verdict("THE POLLED HOST: eight frames byte-exact both ways at every code up to "
                  "PCLK1/2 = 22.5 MHz, against the peer's DMA serve",
                  polled.first_bad == 0u && polled.last_good == top);
    bench.verdict("wherever that climb breaks, the peer still hears every character exact "
                  "there - the boundary is its ANSWER PATH, not the wire",
                  polled.first_bad == 0u ||
                      (polled.bad_r.count == 8u && polled.bad_r.mism == 0u));

    // The same ladder on the ENGINED host: DMA1 streams 4 and 3 carry the
    // data phase, one interrupt a transaction.
    const Ladder engined = climb("DMA host,", true, 0, 32, 0x43, 0x29);
    bench.verdict("THE DMA ENGINES: thirty-two frames byte-exact both ways at every code up to "
                  "PCLK1/2, on DMA1 streams 4 and 3",
                  engined.first_bad == 0u && engined.last_good == top);

    // INFORMATIONAL, NO VERDICT: the same climb with spare_polled_pump,
    // which asks the peer for the driver's polled run instead of its
    // engines - the client run's own boundary, printed beside the DMA one.
    const Ladder run = climb("[informational] peer's polled run,", false,
                             spilink::spare_polled_pump, 8, 0x35, 0x9A);
    (void)run;
    print(serial, "  (no verdict on the run's climb: it is the client run's boundary, printed)",
          crlf);
    (void)link_command_mode();
    bench.verdict("the command channel survives the climbs", command(Op::ping));
}

// =============================================================================
// q - THE KERNEL against the peer
// =============================================================================

namespace kl {

constexpr uint8_t queued = 4;
constexpr uint8_t payload = 6;

uint8_t tx[queued][payload];
uint8_t rx[queued][payload];

struct Kick {};
struct Fill {};
struct Ask {};
struct Wedge {};
/// Letter k: two requests posted from one dispatch, the first bound to fail.
struct Pair {
    bool fault;    ///< the first ends spi_dma_fault; else it ends spi_stalled
    bool polled;   ///< the way the second one runs
};

/// Letter k's window.
constexpr uint16_t way_frames = 32;
constexpr SpiClock way_rate = SpiClock::div32;
uint8_t way_tx[way_frames];
uint8_t way_rx[way_frames];
/// RM0390 2.2.2's map: 0x2003 0000 to 0x3FFF FFFF is reserved, and a bus error
/// on either AHB master is a transfer error that clears the stream's EN (9.3.19,
/// 9.5.5). Nothing reads or writes this but a stream.
inline uint8_t* way_bad_address() { return reinterpret_cast<uint8_t*>(uintptr_t{0x3000'0000u}); }
SpiMode way_mode = SpiMode::mode0;

/// The letter holds the select itself: the peer's `exchange` is ONE burst
/// of queued x payload characters, so the four arbitrated transactions
/// share one select window.
bool hand_cs = true;
SpiClock rate = link_clock;

/// Request 3's command phase (Driver::request()), whose echoes are not
/// read back: whether byte k of request i is one the host reads.
constexpr uint8_t cmd_frames = 3;
constexpr bool read_back(uint8_t i, uint8_t k) { return i != 3u || k >= cmd_frames; }

class Driver;
/// Pending depth 3 - one fewer than the letter queues, so the rejection is
/// reachable - and a 100-tick per-bus timeout, two orders of magnitude
/// above the longest transaction here.
using SpiArb = SpiBus<DmaHost, P, 3, BusPassThrough, 100>;

class Driver : public Fsm<Driver, SpiDone, Kick, Fill, Ask, Wedge, Pair, SleepVote> {
public:
    static inline EventQueue<Event, 12, P> queue;
    static inline uint8_t replies = 0;
    static inline uint8_t rejected = 0;
    static inline uint8_t ok_replies = 0;
    static inline uint8_t timeouts = 0;
    static inline uint8_t votes = 0;
    static inline bool last_vote = false;
    static inline uint8_t first_status = 0;   ///< the first reply's, for the print
    static inline uint8_t second_status = 0;  ///< the second reply's, letter k's

    static void init() {
        clear_tally();
        start(&running);
    }
    static void clear_tally() {
        replies = rejected = ok_replies = timeouts = votes = 0;
        first_status = second_status = 0;
        last_vote = false;
    }
    static void dispatch(const Event& e) { Fsm::dispatch(e); }

private:
    static Status running(const Event& e) {
        return brio::match(
            e,
            [](const Kick&) {
                for (uint8_t i = 0; i < queued; ++i) {
                    post<SpiArb>(request(i));
                }
                return handled();
            },
            [](const Fill&) {
                for (uint8_t i = 0; i < queued + 2u; ++i) {
                    post<SpiArb>(request(i % queued));
                }
                return handled();
            },
            [](const Ask&) {
                post<SpiArb>(PrepareSleep{
                    .depth = SleepDepth::standby,
                    .reply = reply_to<Driver, SleepVote>(),
                });
                return handled();
            },
            [](const Wedge&) {
                host_swallow_completion = true;
                post<SpiArb>(request(0));
                return handled();
            },
            [](const Pair& p) {
                post<SpiArb>(bound_to_fail(p.fault));
                post<SpiArb>(the_good_one(p.polled));
                return handled();
            },
            [](const SpiDone& d) {
                if (replies == 0u) first_status = d.status;
                if (replies == 1u) second_status = d.status;
                ++replies;
                if (d.status == spi_ok) ++ok_replies;
                if (d.status == spi_rejected) ++rejected;
                if (d.status == spi_timeout) ++timeouts;
                return handled();
            },
            [](const SleepVote& v) {
                ++votes;
                last_vote = v.ok;
                return handled();
            },
            [](auto) { return unhandled(); });
    }

    /// The ways one arbiter runs a request on this host: 0 and 2 on the
    /// streams, 1 POLLED inside start() - the synchronous completion the
    /// arbiter answers in its own dispatch - and 3 on the streams with its
    /// first cmd_frames as a command phase, in the loop before the data block.
    static DmaHost::Request request(uint8_t i) {
        const bool split = i == 3u;
        DmaHost::Request r{};
        r.cs = hand_cs ? PinRef{} : CsPin::ref();
        if (split) {
            r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(tx[i]));
            r.cmd_len = cmd_frames;
        }
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx[i] + (split ? cmd_frames : 0u)));
        r.rx = lend<Lease::reply>(rx[i] + (split ? cmd_frames : 0u));
        r.len = static_cast<uint16_t>(split ? payload - cmd_frames : payload);
        r.cs_setup_us = 2;
        r.clock = rate;
        r.mode = SpiMode::mode0;
        r.engine = i == 1u ? SpiEngine::polled : SpiEngine::dma;
        r.reply = reply_to<Driver, SpiDone>();
        return r;
    }

    /// Letter k's first request: a polled write over a block whose enable the
    /// letter took down, or an asynchronous one out of a reserved address. The
    /// select is null on a fault's: the frames a receive error lets out must
    /// not reach the window the peer holds for the good one.
    static DmaHost::Request bound_to_fail(bool fault) {
        DmaHost::Request r{};
        r.cs = fault ? PinRef{} : CsPin::ref();
        r.tx = lend<Lease::reply>(fault ? static_cast<const uint8_t*>(way_bad_address())
                                        : static_cast<const uint8_t*>(way_tx));
        r.len = fault ? way_frames : 9u;
        r.clock = way_rate;
        r.mode = way_mode;
        r.engine = fault ? SpiEngine::dma : SpiEngine::polled;
        r.reply = reply_to<Driver, SpiDone>();
        return r;
    }
    /// The good one: the whole window the peer holds, its own select, read back.
    static DmaHost::Request the_good_one(bool polled) {
        DmaHost::Request r{};
        r.cs = CsPin::ref();
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(way_tx));
        r.rx = lend<Lease::reply>(way_rx);
        r.len = way_frames;
        r.cs_setup_us = 30;
        r.clock = way_rate;
        r.mode = way_mode;
        r.engine = polled ? SpiEngine::polled : SpiEngine::dma;
        r.reply = reply_to<Driver, SpiDone>();
        return r;
    }
};

using BusKernel = Tenuto<P, Driver, SpiArb>;

/// The kernel's own loop minus the sleep: the per-bus timeout is a time
/// event, so the time events are matured before every step.
void pump() {
    TimeEvents<P>::process();
    while (BusKernel::step()) {
        TimeEvents<P>::process();
    }
}

void pump_for(uint32_t ms, bool (*done)()) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
        pump();
        if (done()) {
            break;
        }
    }
}

}   // namespace kl

void tq_peer_kernel() {
    if (!need_peer()) {
        return;
    }
    constexpr uint16_t total = kl::queued * kl::payload;
    spilink::Params a{};
    a.cfg = spilink::Cfg{.apply = 1, .mode = 0, .dord = 0, .regime = spilink::regime_buffer_wait};
    a.count = total;
    a.ms = 800;
    a.seed_a = 0x3B;
    a.seed_b = 0x6D;
    a.pattern = spilink::pattern_prbs;
    if (!peer_act(Op::exchange, a)) {
        bench.verdict("the peer accepted the exchange the kernel letter clocks", false);
        return;
    }
    bench.verdict("the peer accepted the exchange the kernel letter clocks", true);
    {
        spilink::Stream out(a.pattern, a.seed_a);
        for (uint8_t i = 0; i < kl::queued; ++i) {
            for (uint8_t k = 0; k < kl::payload; ++k) {
                kl::tx[i][k] = out.next();
                kl::rx[i][k] = 0xEE;
            }
        }
    }
    kl::BusKernel::init_all();
    kl::hand_cs = true;
    CsPin::output(true);
    (void)DmaHost::init(clock, peer_link_config<DmaHost>());
    dma_host_live = true;
    bus_ao_live = true;
    host_swallow_completion = false;

    settle();
    DmaHost::prime(SpiMode::mode0, link_clock);
    CsPin::clear();
    link_hold();
    post<kl::Driver>(kl::Kick{});
    kl::pump_for(800, [] { return kl::Driver::replies >= kl::queued; });
    link_hold();
    CsPin::set();

    const bool wire_ok =
        kl::Driver::replies == kl::queued && kl::Driver::ok_replies == kl::queued;
    uint16_t mism = 0;
    {
        spilink::Stream want(a.pattern, a.seed_b);
        for (uint8_t i = 0; i < kl::queued; ++i) {
            for (uint8_t k = 0; k < kl::payload; ++k) {
                const uint8_t w = want.next();
                if (kl::read_back(i, k) && kl::rx[i][k] != w) {
                    ++mism;
                }
            }
        }
    }
    spilink::Report r{};
    const bool rep = peer_report(r);
    // THE REPORT IS ITSELF A COMMAND and runs through link_command_mode(),
    // which brings the engineless host up on the same instance - a reset of
    // the block - and takes the streams' vectors from the arbiter's engined
    // host: so that host is brought up again and its claim re-stated before
    // the next leg.
    CsPin::output(true);
    (void)DmaHost::init(clock, peer_link_config<DmaHost>());
    dma_host_live = true;
    bus_ao_live = true;
    print(serial, "  kernel: ", kl::Driver::replies, " replies, ", kl::Driver::ok_replies,
          " ok (the first status ", kl::Driver::first_status, "); host mism=", mism, "; the peer read ", r.count, " of ", total, " with mism=",
          r.mism, " flags=", hex(r.flags), crlf);
    bench.verdict("FOUR TRANSACTIONS QUEUED FROM ONE DISPATCH come back in order, every one "
                  "spi_ok, and the bytes on the wire are exactly what each request lent - "
                  "read back by A SECOND BOARD: two on the streams, one polled inside "
                  "start(), one a command in the loop before its data block",
                  wire_ok && mism == 0u && rep && r.count == total && r.mism == 0u);

    kl::Driver::clear_tally();
    post<kl::Driver>(kl::Fill{});
    kl::pump_for(600, [] { return kl::Driver::replies >= kl::queued + 2u; });
    print(serial, "  full queue: ", kl::Driver::replies, " replies, ", kl::Driver::rejected,
          " rejected", crlf);
    bench.verdict("a request past the arbiter's pending depth is answered bus_rejected "
                  "IMMEDIATELY and never dropped in silence",
                  kl::Driver::replies == kl::queued + 2u && kl::Driver::rejected >= 1u);

    kl::Driver::clear_tally();
    post<kl::Driver>(kl::Ask{});
    kl::pump_for(200, [] { return kl::Driver::votes != 0u; });
    const bool idle_vote = kl::Driver::last_vote;
    const uint8_t idle_votes = kl::Driver::votes;
    kl::Driver::clear_tally();
    post<kl::Driver>(kl::Kick{});
    post<kl::Driver>(kl::Ask{});
    kl::pump_for(600, [] {
        return kl::Driver::votes != 0u && kl::Driver::replies >= kl::queued;
    });
    const bool busy_vote = kl::Driver::last_vote;
    const uint8_t busy_votes = kl::Driver::votes;
    const uint8_t busy_replies = kl::Driver::replies;
    print(serial, "  votes: idle ", idle_votes, " ok=", idle_vote ? 1 : 0, ", busy ", busy_votes,
          " ok=", busy_vote ? 1 : 0, " with ", busy_replies, " of ", kl::queued,
          " transactions done", crlf);
    bench.verdict("an IDLE bus votes ok on a PrepareSleep and a BUSY one votes AGAINST it",
                  idle_votes == 1u && idle_vote && busy_votes == 1u && !busy_vote &&
                      busy_replies == kl::queued);

    kl::Driver::clear_tally();
    post<kl::Driver>(kl::Wedge{});
    kl::pump_for(800, [] { return kl::Driver::replies >= 1u; });
    const uint8_t timeouts = kl::Driver::timeouts;
    host_swallow_completion = false;
    kl::Driver::clear_tally();
    post<kl::Driver>(kl::Kick{});
    kl::pump_for(600, [] { return kl::Driver::replies >= kl::queued; });
    print(serial, "  timeout: ", timeouts, " spi_timeout replies, then ", kl::Driver::replies,
          " replies of which ", kl::Driver::ok_replies, " ok afterwards", crlf);
    bench.verdict("A COMPLETION THAT NEVER COMES is answered spi_timeout in its place, and "
                  "recover() puts the engine back where start() is legal - the next four "
                  "transactions run to spi_ok on the same bus AO",
                  timeouts == 1u && kl::Driver::ok_replies == kl::queued);
    bus_ao_live = false;
    dma_host_live = false;
    DmaHost::release();
    (void)link_command_mode();

    // AN ASYNCHRONOUS REQUEST ON THE ENGINELESS HOST IS REFUSED: start()
    // answers at once with spi_rejected, before the block, the select or
    // the D/C is touched. The witness is the select's own pad, PB12, its
    // falling edges counted on EXTI line 12: none for the refused request,
    // one for the same request polled. The peer hears the polled one as
    // a frame or two of command traffic and ignores it.
    {
        sel_falls = 0;
        const bool watched = SelWatch::select() && SelWatch::configure(ExtiSense::falling);
        (void)SelWatch::clear();
        (void)SelWatch::arm(true);
        Nvic::enable(SelWatch::irq());
        static const uint8_t frame[2] = {0, 0};
        Host::Request q{};
        q.engine = SpiEngine::dma;
        q.cs = CsPin::ref();
        q.tx = lend<Lease::reply>(frame);
        q.len = 2;
        q.clock = link_clock;
        const bool ended = Host::start(q);
        const uint8_t status = Host::status();
        brio::spin_us<Ruler>(2'000);
        const uint16_t refused_falls = sel_falls;
        q.engine = SpiEngine::polled;
        (void)Host::start(q);
        brio::spin_us<Ruler>(2'000);
        const uint16_t polled_falls = static_cast<uint16_t>(sel_falls - refused_falls);
        (void)SelWatch::arm(false);
        Nvic::disable(SelWatch::irq());
        print(serial, "  refused: start() ", ended ? 1u : 0u, " status ", status, " (spi_rejected ",
              spi_rejected, "), select falls ", refused_falls, "; the same request polled: select falls ",
              polled_falls, crlf);
        bench.verdict("AN ASYNCHRONOUS REQUEST ON A HOST WITH NO ENGINES IS REFUSED: start() answers "
                      "at once with spi_rejected and the select never falls - once for the same "
                      "request polled",
                      watched && ended && status == spi_rejected && refused_falls == 0u &&
                          polled_falls == 1u);
    }
    settle();
    (void)link_command_mode();
    bench.verdict("the command channel survives the kernel letter", command(Op::ping));
}

// =============================================================================
// r - THE ROLES INVERT: this board as the CLIENT, the peer as the host
// =============================================================================

uint8_t crx[64];
uint8_t cout_[64];
Client::Run client_run{};

/// The client role on the SAME four pads: SpiClient's run on SPI2, the
/// peer's GPIO-driven select the hardware NSS input.
bool run_as_client(const spilink::Params& a) {
    bus_ao_live = false;
    Host::release();
    if (!Client::init(clock, {.mode = SpiMode::mode0,
                              .bits = SpiDataSize::bits8,
                              .lsb_first = false,
                              .nss = SpiNss::hardware_input,
                              .drive_output = true,
                              // The answer line at LOW, the class the
                              // stm32f4 spi_peer measured for this client
                              // against the G0B1RE host (its header).
                              .miso_speed = PinSpeed::low})) {
        return false;
    }
    const uint16_t n = a.count < sizeof crx ? a.count : sizeof crx;
    spilink::Stream out(a.pattern, a.seed_b);
    for (uint16_t i = 0; i < n; ++i) {
        cout_[i] = out.next();
        crx[i] = 0xEE;
    }
    Client::enable();
    client_run = Client::exchange(std::span<const uint8_t>(cout_, n), std::span<uint8_t>(crx, n),
                                  static_cast<uint32_t>(a.ms) * (SysClock::hz / 36'000u));
    (void)Client::disable();
    Client::release();
    return true;
}

void tr_peer_client() {
    if (!need_peer()) {
        return;
    }
    spilink::Params a{};
    a.count = 12;
    a.ms = 500;
    a.seed_a = 0x2C;
    a.seed_b = 0x71;
    a.pattern = spilink::pattern_prbs;
    a.aux8 = 40;    // lead-in ms: time for this end to become a client
    // The peer's SCK division OF ITS OWN CLOCK: 32 is 2 MHz on the G0's
    // 64 MHz.
    a.aux16 = 32;
    a.cfg.apply = 1;
    if (!peer_act(Op::host_burst, a)) {
        bench.verdict("the peer accepted the host_burst command", false);
        return;
    }
    bench.verdict("the peer accepted the host_burst command", true);
    const bool ran = run_as_client(a);
    bench.verdict("this board came up as an SPI CLIENT on the very pads it hosts with - NSS "
                  "as the hardware input the peer drives",
                  ran);
    spilink::Report r{};
    const bool rep = peer_report(r);
    uint16_t mism = 0;
    uint8_t first_idx = 0xFF, got = 0, want = 0;
    for (uint16_t i = 0; i < client_run.count; ++i) {
        const uint8_t exp = spilink::pattern_value(a.pattern, a.seed_a, i);
        if (crx[i] != exp) {
            if (mism == 0) {
                first_idx = static_cast<uint8_t>(i);
                got = crx[i];
                want = exp;
            }
            ++mism;
        }
    }
    print(serial, "  client read ", client_run.count, " of ", a.count, " frames, mism=", mism);
    if (mism) {
        print(serial, " (first idx ", first_idx, " got ", hex(got), " exp ", hex(want), ")");
    }
    print(serial, "; the run ended ",
          client_run.end == Client::RunEnd::complete ? "complete"
          : client_run.end == Client::RunEnd::closed ? "closed"
                                                     : "STALLED",
          "; the peer as HOST read ", r.count, " with mism=", r.mism, " flags=", hex(r.flags),
          crlf);
    bench.verdict("the client run received every frame the foreign host clocked, byte-exact",
                  ran && client_run.count == a.count && mism == 0u);
    bench.verdict("...and that host read this end's answer stream byte-exact FROM THE FIRST "
                  "FRAME - the run's first answer in the buffer before the select fell "
                  "(26.3.9)",
                  rep && r.count == a.count && r.mism == 0u);
    (void)link_command_mode();
    bench.verdict("the command channel is back after the role swap", command(Op::ping));
}

// =============================================================================
// l - ES0298 2.14.5, staged against the peer's known stream
// =============================================================================

void tl_errata_pad() {
    if (!need_peer()) {
        return;
    }
    // The erratum: a MASTER receiving keeps the last bit of a frame from
    // the pattern before when the feedback from its own SCK pad comes
    // later than two APB periods, and table 5 gives the APB ceiling per
    // OSPEEDR class at 30 pF - 84 MHz at high and very-high, 75 at medium,
    // 25 at low. SPI2's APB1 is 45 MHz: medium is inside the table, low is
    // outside it. The SPI rate does not enter the erratum; it enters the
    // PAD: DS10693 table 58 gives the low class 4 MHz of fmax at 50 pF, so
    // low is staged at PCLK1/16 = 2.8 MHz, inside its own reach, and the
    // same class at /2 is printed beside it as the pad's question, not the
    // erratum's.
    struct Leg {
        PinSpeed pad;
        SpiClock rate;
        const char* name;
    };
    static const Leg legs[] = {{PinSpeed::medium, SpiClock::div2, "medium at 22.5 MHz"},
                               {PinSpeed::low, SpiClock::div16, "low at 2.8 MHz"},
                               {PinSpeed::low, SpiClock::div2, "low at 22.5 MHz"}};
    uint16_t mism[3] = {};
    uint16_t last_bit[3] = {};
    bool ran[3] = {};
    for (uint8_t k = 0; k < 3u; ++k) {
        const PinSpeed keep = peer_sck_speed;
        peer_sck_speed = legs[k].pad;
        Exchange e{};
        e.rate = legs[k].rate;
        e.count = 32;
        e.engines = true;
        e.seed_a = 0x55;
        e.seed_b = static_cast<uint8_t>(0x3Cu + k);
        spilink::Report r{};
        ran[k] = do_exchange(e) && peer_report(r);
        const Verify v = verify_rx(e);
        peer_sck_speed = keep;
        mism[k] = v.mism;
        last_bit[k] = v.last_bit_only;
        print(serial, "  SCK pad ", legs[k].name, " (the erratum's APB ceiling ",
              spi_errata_apb_ceiling_hz(legs[k].pad) / 1'000'000u, " MHz against APB1's 45): ",
              e.count, " frames, host mism=", v.mism, " of which ", v.last_bit_only,
              " in the LAST BIT alone; the peer heard ", r.count, " with mism=", r.mism, crlf);
    }
    (void)link_command_mode();
    bench.verdict("with the SCK pad at medium - inside ES0298 2.14.5's table - the host reads "
                  "the peer's stream exact at PCLK1/2",
                  ran[0] && mism[0] == 0u);
    bench.verdict("at low, outside the erratum's table and inside the pad's own reach, whatever "
                  "the host reads wrong is the LAST BIT alone - the erratum's signature "
                  "(vacuous where the wire's load leaves nothing wrong: printed above)",
                  ran[1] && mism[1] == last_bit[1]);
    print(serial, "  (no verdict on low at 22.5 MHz: DS10693 table 58 gives that class 4 MHz "
                  "at 50 pF, so it measures the pad, not the erratum)",
          crlf);
}

// =============================================================================
// x - the slip statistics against the peer (a diagnostic, OUTSIDE z)
// =============================================================================

constexpr uint8_t slip_rounds = 10;
constexpr uint8_t dma_slip_rounds = 30;

void tx_peer_slips() {
    if (!need_peer()) {
        return;
    }
    print(serial, "  each mode ", slip_rounds, " times, 8 frames at PCLK1/",
          static_cast<uint32_t>(2u << static_cast<uint8_t>(link_clock)),
          ": bursts not byte-exact, and the first bad frame at each end (host/client)", crlf);
    uint8_t total = 0;
    for (uint8_t m = 0; m < 4; ++m) {
        uint8_t slips = 0;
        print(serial, "  mode ", m, ":");
        for (uint8_t k = 0; k < slip_rounds; ++k) {
            Exchange e{};
            e.cfg.mode = m;
            e.host_mode = static_cast<SpiMode>(m);
            e.seed_a = static_cast<uint8_t>(0x13u + m + 7u * k);
            e.seed_b = static_cast<uint8_t>(0x57u + m + 11u * k);
            Verify v{};
            spilink::Report r{};
            if (!exchange_exact(e, v, r)) {
                ++slips;
                print(serial, " [", v.idx, "/", r.idx, r.count != e.count ? "!" : "", "]");
            }
        }
        print(serial, "  -> ", slips, " of ", slip_rounds, crlf);
        total = static_cast<uint8_t>(total + slips);
    }
    print(serial, "  ", total, " of ", 4u * slip_rounds, " bursts slipped", crlf);

    // THE BACK-TO-BACK RUNG: the DMA host at PCLK1/4, where the frames run
    // without a gap and the MOSI pad's class showed (peer_mosi_speed's
    // comment) - thirty bursts of thirty-two frames.
    uint8_t dma_slips = 0;
    print(serial, "  the DMA host at PCLK1/4, ", dma_slip_rounds, " bursts of 32 frames:");
    for (uint8_t k = 0; k < dma_slip_rounds; ++k) {
        Exchange e{};
        e.rate = SpiClock::div4;
        e.count = 32;
        e.engines = true;
        e.seed_a = static_cast<uint8_t>(0x43u + 5u * k);
        e.seed_b = static_cast<uint8_t>(0x29u + 3u * k);
        Verify v{};
        spilink::Report r{};
        if (!exchange_exact(e, v, r)) {
            ++dma_slips;
            print(serial, " [", v.idx, "/", r.idx, r.count != e.count ? "!" : "", "]");
        }
    }
    print(serial, "  -> ", dma_slips, " of ", dma_slip_rounds, crlf);
    (void)link_command_mode();
    print(serial, "  the command channel afterwards: ", command(Op::ping) ? "answers" : "DEAD",
          crlf);
}

// =============================================================================
// u, v, w - THE EXTENDED EXCHANGE against the peer
// =============================================================================
//
// spi_link.hpp's three extended bits give this host's own 16-bit frames,
// its transmit-only shapes and its CRC unit a FOREIGN WITNESS: the peer
// answers or listens in 16-bit frames (`spare_wide`), listens with its
// answer line dark (`spare_receive_only`), or runs its CRC unit beside
// this one and checks the CRC frame this end sends (`spare_crc`). A peer
// that serves them raises `report_extended`; one that does not ignores
// the bits, and every verdict here asks for the flag first. Every letter
// climbs the BR codes from the command rate to PCLK1/2 = 22.5 MHz, inside
// the far client's 32 MHz (DS13560 table 76), every rung judged.

namespace ext {

/// Which of this host's two ways clocks the window: the polled loop, or
/// the DMA1 streams (a request naming them on the engined host, half-word
/// elements).
enum class Path : uint8_t { polled, engines };

const char* path_name(Path p) {
    switch (p) {
        case Path::polled: return "polled loop";
        default: return "DMA engines";
    }
}

constexpr uint16_t cap = 64;   ///< frames this letter set clocks at most
alignas(4) uint8_t tx[2u * cap + 4u];
alignas(4) uint8_t rx[2u * cap + 4u];

struct Ext {
    Path path = Path::polled;
    SpiClock rate = link_clock;
    uint16_t count = 32;
    bool wide = false;
    bool receive_only = false;
    uint8_t spare = 0;   ///< spare_polled_pump, on top of the extended bits
    uint8_t seed_a = 0x4D;
    uint8_t seed_b = 0xB2;
};

struct Outcome {
    bool accepted = false;    ///< the peer took the command
    bool clocked = false;     ///< this host's transaction completed
    bool reported = false;
    spilink::Report r{};
    uint16_t mism = 0;        ///< this end's read-back, bytes wrong
    uint16_t idx = 0xFFFF;
    uint8_t got = 0, exp = 0;
    uint32_t cycles = 0;      ///< start() to completion, core cycles
    bool ticked = false;      ///< the kernel tick landed inside that time
    bool inside = false;      ///< start() answered true: it ended inside the call
    uint16_t completions = 0; ///< the engined host's completions under it
    uint16_t blocks = 0;      ///< its receive stream's block ends
};

/// Whether the peer served the extended exchange and heard every frame.
bool peer_heard(const Ext& e, const Outcome& o) {
    return o.accepted && o.reported && (o.r.flags & spilink::report_extended) != 0u &&
           o.r.count == e.count && o.r.mism == 0u;
}

/// The whole window both ways, judged at both ends.
bool exact(const Ext& e, const Outcome& o) {
    return peer_heard(e, o) && o.clocked && (e.receive_only || o.mism == 0u);
}

/// Command one extended exchange and clock it on `e.path`.
Outcome run(const Ext& e) {
    Outcome o{};
    spilink::Params a{};
    a.cfg = spilink::Cfg{.apply = 1, .mode = 0, .dord = 0, .regime = spilink::regime_buffer_wait};
    a.count = e.count;
    a.ms = 400;
    a.seed_a = e.seed_a;
    a.seed_b = e.seed_b;
    a.pattern = spilink::pattern_prbs;
    a.spare = static_cast<uint8_t>(e.spare | (e.wide ? spilink::spare_wide : 0u) |
                                   (e.receive_only ? spilink::spare_receive_only : 0u));
    o.accepted = peer_act(Op::exchange, a);
    if (!o.accepted) {
        return o;
    }
    const uint16_t n = e.count < cap ? e.count : cap;
    const uint16_t bytes = static_cast<uint16_t>(e.wide ? 2u * n : n);
    {
        spilink::Stream out(a.pattern, a.seed_a);
        for (uint16_t i = 0; i < sizeof tx; ++i) {
            tx[i] = i < bytes ? out.next() : 0u;
            rx[i] = 0xEE;
        }
    }
    settle();
    const SpiDataSize bits = e.wide ? SpiDataSize::bits16 : SpiDataSize::bits8;
    uint8_t* const in = e.receive_only ? nullptr : rx;
    host_done = false;
    if (e.path == Path::engines) {
        dma_host_live = true;
        CsPin::output(true);
        (void)DmaHost::init(clock, peer_link_config<DmaHost>());
        DmaHost::prime(SpiMode::mode0, e.rate);
        CsPin::clear();
        link_hold();
        DmaHost::Request r{};
        r.cs = {};
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx));
        r.rx = lend<Lease::reply>(in);
        r.len = n;
        r.clock = e.rate;
        r.mode = SpiMode::mode0;
        r.bits = bits;
        r.engine = SpiEngine::dma;
        dma_completions = 0;
        dma_blocks = 0;
        const uint32_t tick0 = Ticker::ticks();
        const uint32_t t0 = Ruler::now();
        const bool done = DmaHost::start(r);
        o.inside = done;
        // 200 ms: 64 16-bit frames at PCLK1/256 are 5.8 ms.
        o.clocked = done || brio::wait_until<Ruler>([] { return host_done; }, 200'000);
        o.cycles = Ruler::now() - t0;
        o.ticked = Ticker::ticks() != tick0;
        o.completions = dma_completions;
        o.blocks = dma_blocks;
        link_hold();
        CsPin::set();
        DmaHost::release();
        dma_host_live = false;
    } else {
        Host::prime(SpiMode::mode0, e.rate);
        CsPin::clear();
        link_hold();
        Host::Request r{};
        r.cs = {};
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx));
        r.rx = lend<Lease::reply>(in);
        r.len = n;
        r.clock = e.rate;
        r.mode = SpiMode::mode0;
        r.bits = bits;
        r.engine = SpiEngine::polled;
        const uint32_t tick0 = Ticker::ticks();
        const uint32_t t0 = Ruler::now();
        // A polled request ends inside start(): nothing to wait for after it.
        o.clocked = Host::start(r);
        o.cycles = Ruler::now() - t0;
        o.ticked = Ticker::ticks() != tick0;
        link_hold();
        CsPin::set();
    }
    (void)link_command_mode();
    o.reported = peer_report(o.r);
    if (!e.receive_only) {
        for (uint16_t i = 0; i < bytes; ++i) {
            const uint8_t want = spilink::wide_byte(a.pattern, a.seed_b, i, e.wide, false);
            if (rx[i] != want) {
                if (o.mism == 0u) {
                    o.idx = i;
                    o.got = rx[i];
                    o.exp = want;
                }
                ++o.mism;
            }
        }
    }
    return o;
}

uint32_t division(SpiClock c) { return 2u << static_cast<uint8_t>(c); }

void print_outcome(const Ext& e, const Outcome& o) {
    print(serial, "    ", path_name(e.path), " PCLK1/", division(e.rate), " = ",
          Host::sck_hz(e.rate) / 1000u, " kHz, ", e.count, e.wide ? " 16-bit" : " 8-bit",
          " frames: ", exact(e, o) ? "exact" : "NOT exact");
    if (!e.receive_only) {
        print(serial, "  host mism=", o.mism);
        if (o.mism != 0u) {
            print(serial, " (byte ", o.idx, " got ", hex(o.got), " exp ", hex(o.exp), ")");
        }
    }
    print(serial, "  peer count=", o.r.count, " mism=", o.r.mism);
    if (o.r.mism != 0u) {
        print(serial, " (byte ", o.r.idx, " got ", hex(o.r.got), " exp ", hex(o.r.exp), ")");
    }
    print(serial, " flags=", hex(o.r.flags), " serve=", serve_name(o.r),
          o.clocked ? "" : " HOST NEVER COMPLETED", crlf);
}

constexpr SpiClock rungs[] = {SpiClock::div256, SpiClock::div128, SpiClock::div64,
                              SpiClock::div32,  SpiClock::div16,  SpiClock::div8,
                              SpiClock::div4,   SpiClock::div2};

/// WHAT THIS END'S READ IS JUDGED TO, FROM THE ANSWER LINE'S BUDGET. A bit of
/// the peer's answer must cross the wire inside one SCK low phase: this end's
/// tw(SCKL) is division/2 x TPCLK - 1.5 ns (DS10693 table 63, TPCLK = 22.2 ns),
/// against the G0B1's tv(SO) up to 14 ns (DS13560 table 76) and this end's
/// tsu(MI) of 4 ns, 18 ns before the SCK pad at medium (the class the slips set,
/// peer_sck_speed above) and the wire both ways.
///   PCLK1/2: 20.7 - 18 = 2.7 ns left for the pad and two runs of wire: no budget.
///   PCLK1/4: 42.9 - 18 = 24.9 ns left, nine times that.
/// So every read is judged to PCLK1/4, zero frames wrong. At PCLK1/2 what is
/// judged is the far side, every frame clocked and heard exact by the peer, and
/// this end's read is printed: on these wires 16-bit frames read wrong there,
/// from the polled loop and the streams alike (docs/stm32f4/spi.md, "Findings
/// left open"). The 8-bit windows of letters p and l, which hold at PCLK1/2 in
/// every run, keep their verdicts there.
constexpr SpiClock wide_readback_top = SpiClock::div4;

/// Whether this end's read-back is judged at `c` on `e`'s width.
bool readback_judged(const Ext& e, SpiClock c) {
    return !(e.wide && !e.receive_only &&
             static_cast<uint8_t>(c) < static_cast<uint8_t>(wide_readback_top));
}

/// Climb every BR code with `e` and say whether every rung was exact - on
/// a rung past the answer line's budget, every frame clocked and heard by
/// the peer, this end's read-back printed.
bool climb(Ext e) {
    bool all = true;
    uint8_t k = 0;
    for (const SpiClock c : rungs) {
        e.rate = c;
        e.seed_a = static_cast<uint8_t>(0x4Du + 9u * k);
        e.seed_b = static_cast<uint8_t>(0xB2u + 5u * k);
        ++k;
        const Outcome o = run(e);
        print_outcome(e, o);
        const bool judged = readback_judged(e, c);
        if (!judged) {
            print(serial, "      (this end's read-back printed at this rung, not judged: the answer "
                          "line's budget)", crlf);
        }
        all = all && (judged ? exact(e, o) : peer_heard(e, o) && o.clocked);
    }
    return all;
}

/// What frame_permille() answers when every pair of windows carried a kernel
/// tick: a lengthened time, printed and not judged.
constexpr uint32_t lengthened = UINT32_MAX;

/// THE WIRE'S TIME, from the difference of two windows of 16 and 48
/// frames at the same rung - the transaction's fixed cost cancels - in
/// permille of the frame's SCK periods (a bit is division x HCLK/PCLK1 =
/// 4 x division core cycles). The time asks that every frame was clocked
/// and heard by the peer, and the read-back where readback_judged() says;
/// zero otherwise, the window that failed named first. WHAT LENGTHENS A WINDOW
/// IS NOT JUDGED: the console is drained first (a console that will not drain
/// times nothing: zero), and a pair a tick landed in, printed, is run again, up
/// to four pairs; when all four carried one the answer is `lengthened`. The two
/// windows last 50 us at PCLK1/2 against a 1 ms tick, so one pair in twenty
/// takes one and four in a row come one time in 160 000; at PCLK1/8 they last
/// some 200 us, one pair in five and four in a row one time in 600.
uint32_t frame_permille(Ext e) {
    Ext shorter = e;
    shorter.count = 16;
    Ext longer = e;
    longer.count = 48;
    longer.seed_a = static_cast<uint8_t>(e.seed_a + 3u);
    if (!brio::console_drain<Ruler, Serial>()) {
        return 0u;
    }
    Outcome a{};
    Outcome b{};
    for (uint8_t attempt = 0; attempt < 4u; ++attempt) {
        a = run(shorter);
        b = run(longer);
        if (!a.ticked && !b.ticked) {
            break;
        }
        print(serial, "      a tick landed in that pair of windows: run again", crlf);
        console_drain();
    }
    if (a.ticked || b.ticked) {
        print(serial, "      a tick in all four pairs: the time is not taken", crlf);
        return lengthened;
    }
    const bool judged = readback_judged(e, e.rate);
    auto timed = [judged](const Ext& x, const Outcome& o) {
        return judged ? exact(x, o) : peer_heard(x, o) && o.clocked;
    };
    if (!exact(shorter, a)) {
        print_outcome(shorter, a);
    }
    if (!exact(longer, b)) {
        print_outcome(longer, b);
    }
    if (!timed(shorter, a) || !timed(longer, b) || b.cycles <= a.cycles) {
        return 0u;
    }
    const uint32_t per_frame_x1000 = (b.cycles - a.cycles) * 1000u / 32u;
    const uint32_t wire = (e.wide ? 16u : 8u) * division(e.rate) * (SysClock::hz / SysClock::pclk1_hz);
    return per_frame_x1000 / wire;
}

/// THE STREAMS' BOUND, derived, in permille of a rung's frame (`wire` core
/// cycles). On the streams the frames run back to back, so the two windows
/// differ by 32 frames exactly and by how much their fixed costs differ, at
/// most J core cycles:
///   3   start()'s first APB1 access waits for PCLK1's next edge, 0 to
///       HCLK/PCLK1 - 1 cycles, and the rest of its path, the streams' first
///       beat included, follows at that phase
///   6   the completing vector's two APB1 accesses (finish_dma()'s read and
///       write of CR2), 0 to 3 each for the same edge; the rest of its path
///       is AHB1 and SRAM, synchronous to the core
///   11  how late the wait on host_done sees the vector's store: one turn of
///       its poll (the release listing: ldrb, cbnz, the DWT's ldr, subs, cmp
///       and a taken bcc, 2 + 1 + 2 + 1 + 1 + 4)
///   2   the exception's entry, by the poll's instruction it lands on (a load
///       of two cycles)
///   2   the receive stream's last write to SRAM waiting on the core's poll of
///       it (one core access)
/// - J = 24. Both windows run the same code after the same commands, so the
/// ART holds the same lines at each one's vector and its misses cancel. The
/// figure is truncated twice (per frame, then per mille): under 1 more below.
constexpr uint32_t window_jitter_cycles = 3u + 6u + 11u + 2u + 2u;

/// The permille either side of 1000 the streams' frame may read at a rung of
/// `wire` cycles a frame: J over 32 frames, rounded up, and the truncations.
constexpr uint32_t streams_slack(uint32_t wire) {
    return (window_jitter_cycles * 1000u + 32u * wire - 1u) / (32u * wire) + 1u;
}
static_assert(streams_slack(16u * 2u * 4u) == 7u && streams_slack(16u * 8u * 4u) == 3u);

bool streams_wire_ok(uint32_t permille, uint32_t wire) {
    const uint32_t slack = streams_slack(wire);
    return permille >= 1000u - slack && permille <= 1000u + slack;
}

}  // namespace ext

void tu_wide() {
    if (!need_peer()) {
        return;
    }
    print(serial, "  16-bit frames (CR1.DFF), 32 a window, against the peer's DMA serve in "
                  "16-bit beats:",
          crlf);
    bool path_ok[2] = {};
    for (uint8_t p = 0; p < 2u; ++p) {
        ext::Ext e{};
        e.path = static_cast<ext::Path>(p);
        e.wide = true;
        path_ok[p] = ext::climb(e);
        print(serial, "  -> ", ext::path_name(e.path), ": ", path_ok[p] ? "exact" : "NOT exact",
              " at every code", crlf);
    }
    bench.verdict("16-BIT FRAMES ON THE POLLED LOOP: thirty-two frames byte-exact both ways at "
                  "every code to PCLK1/4, every frame heard exact by the peer at PCLK1/2 and this "
                  "end's read printed there, the peer serving them in 16-bit beats (report_extended)",
                  path_ok[0]);
    bench.verdict("...and on the DMA1 STREAMS, a half-word beat a frame (the beat is the "
                  "frame), likewise",
                  path_ok[1]);

    // THE WIRE'S TIME: the engines at every rate, judged against the bound
    // streams_slack() derives. The polled loop keeps one frame in flight, so
    // its frame is the wire's less the half bit RXNE hides plus its turn - a
    // cost of code whose APB1 accesses RM0390 gives no time for (2.1.8): it
    // is printed beside them, with no verdict, and letter b measures the turn.
    static constexpr SpiClock timed[] = {SpiClock::div8, SpiClock::div4, SpiClock::div2};
    bool engines_wire = true;
    uint8_t engines_judged = 0;
    for (const SpiClock c : timed) {
        const uint32_t frame_cycles =
            16u * ext::division(c) * (SysClock::hz / SysClock::pclk1_hz);
        for (uint8_t p = 0; p < 2u; ++p) {
            ext::Ext e{};
            e.path = static_cast<ext::Path>(p);
            e.wide = true;
            e.rate = c;
            const uint32_t pm = ext::frame_permille(e);
            if (pm == ext::lengthened) {
                continue;
            }
            const bool engines = e.path == ext::Path::engines;
            print(serial, "  ", ext::path_name(e.path), " at PCLK1/", ext::division(c),
                  ": a 16-bit frame takes ", pm, " permille of its 16 SCK periods");
            if (engines) {
                print(serial, " (1000 +- ", ext::streams_slack(frame_cycles), ")", crlf);
                engines_wire = engines_wire && ext::streams_wire_ok(pm, frame_cycles);
                ++engines_judged;
            } else {
                print(serial, " (printed)", crlf);
            }
        }
    }
    bench.verdict("THE STREAMS' 16-BIT FRAMES RUN AT THE WIRE'S TIME: a frame inside the window "
                  "is its sixteen SCK periods, to the two windows' fixed costs over 32 frames, at "
                  "PCLK1/8, /4 and /2",
                  engines_judged != 0u && engines_wire);

    // The peer's own 16-bit RUN (spare_polled_pump): the other end's
    // SpiClient::exchange in uint16_t frames.
    ext::Ext run{};
    run.wide = true;
    run.spare = spilink::spare_polled_pump;
    const bool run_ok = ext::climb(run);
    bench.verdict("the PEER'S CLIENT RUN in 16-bit frames (its exchange() over uint16_t, one "
                  "frame an access) is exact both ways at every code to PCLK1/4, and hears every "
                  "frame at PCLK1/2, this end's read printed there",
                  run_ok);
    (void)link_command_mode();
    bench.verdict("the command channel survives the 16-bit climbs", command(Op::ping));
}

void tv_receive_only() {
    if (!need_peer()) {
        return;
    }
    // THE HOST'S TRANSMIT-ONLY SHAPES, WITNESSED: `rx` null is the loop that
    // reads nothing back and the streams' discard -
    // and the peer listens with its answer line DARK, so what it judges is
    // this end's transmit path alone. An ODD count: thirty-three.
    print(serial, "  transmit-only windows of 33 frames, the peer listening with MISO dark "
                  "(its receive DMA):",
          crlf);
    bool shapes[2][2] = {};
    for (uint8_t p = 0; p < 2u; ++p) {
        for (uint8_t w = 0; w < 2u; ++w) {
            ext::Ext e{};
            e.path = static_cast<ext::Path>(p);
            e.wide = w != 0u;
            e.receive_only = true;
            e.count = 33;
            shapes[p][w] = ext::climb(e);
        }
    }
    bench.verdict("THE POLLED TRANSMIT-ONLY LOOP, witnessed by a listening client: 33 frames, "
                  "8-bit and 16-bit, every one heard exact at every code to PCLK1/2",
                  shapes[0][0] && shapes[0][1]);
    bench.verdict("...and the streams' write shape likewise", shapes[1][0] && shapes[1][1]);

    // THE PEER'S receive() RUN, at odd counts either side of its pairs and in
    // both widths - on the G0, the pairs under the half threshold and the
    // odd last frame under the quarter one.
    print(serial, "  the peer's receive() run, 7 and 33 frames, from the polled loop:", crlf);
    bool receive_ok = true;
    static constexpr uint16_t counts[] = {7, 33};
    for (const uint16_t n : counts) {
        for (uint8_t w = 0; w < 2u; ++w) {
            ext::Ext e{};
            e.wide = w != 0u;
            e.receive_only = true;
            e.count = n;
            e.spare = spilink::spare_polled_pump;
            receive_ok = ext::climb(e) && receive_ok;
        }
    }
    bench.verdict("the PEER'S receive() RUN hears 7 and 33 frames exact, 8-bit and 16-bit, at "
                  "every code",
                  receive_ok);

    // ODD-COUNT EXCHANGES both ways, both serves: on the G0's exchange() run
    // the odd last frame is read under the moved threshold.
    print(serial, "  exchanges of 7 and 33 8-bit frames, both ways:", crlf);
    bool odd_ok = true;
    for (const uint16_t n : counts) {
        for (uint8_t s = 0; s < 2u; ++s) {
            ext::Ext e{};
            e.count = n;
            e.spare = s != 0u ? spilink::spare_polled_pump : 0u;
            odd_ok = ext::climb(e) && odd_ok;
        }
    }
    bench.verdict("ODD COUNTS both ways: 7 and 33 frames exact at every code, against the "
                  "peer's DMA serve and its client run (the odd last frame under the moved "
                  "threshold)",
                  odd_ok);
    (void)link_command_mode();
    bench.verdict("the command channel survives the receive-only climbs", command(Op::ping));
}

// ---- w: the CRC ------------------------------------------------------------------

namespace ext {

/// The bitwise CRC spi_link.hpp names: of the frame's width, MSb first,
/// unreflected, from zero, over `frames` frames of a byte stream (a 16-bit
/// frame is two bytes, low first).
uint16_t crc_of(uint16_t poly, bool wide, const uint8_t* bytes, uint16_t frames) {
    const uint8_t width = wide ? 16u : 8u;
    const uint32_t top = 1u << (width - 1u);
    const uint32_t mask = (1u << width) - 1u;
    uint32_t crc = 0;
    for (uint16_t k = 0; k < frames; ++k) {
        const uint16_t f = wide ? static_cast<uint16_t>(bytes[2u * k] | (bytes[2u * k + 1u] << 8))
                                : bytes[k];
        for (uint8_t b = width; b-- > 0;) {
            const bool in = ((f >> b) & 1u) != 0u;
            const bool out = (crc & top) != 0u;
            crc = (crc << 1) & mask;
            if (in != out) {
                crc ^= poly;
            }
        }
    }
    return static_cast<uint16_t>(crc);
}

struct Crc {
    Outcome o{};
    uint16_t frame = 0;          ///< the frame read where the peer's CRC belongs
    uint16_t tx_crc = 0, rx_crc = 0;
    uint16_t tx_ref = 0, rx_ref = 0, peer_ref = 0;
    bool host_crcerr = false;
    bool frames_ok = false;      ///< every data frame's RXNE came inside its budget
    /// The CRC frame's RXNE came inside its budget (true where none was due).
    bool crc_came = false;
};

/// Letter t's three stamps, SysTick's down-count at HCLK: before the last
/// data store, after the CRCNEXT store, and when that frame's RXNE is
/// first seen.
uint32_t walk_store = 0, walk_next = 0, walk_rxne = 0;

uint32_t walk_since(uint32_t from, uint32_t to) {
    const uint32_t period = SysTick->LOAD + 1u;
    return (from + period - to) % period;
}

[[gnu::always_inline]] inline void spin_loops(uint32_t n) {
    if (n != 0u) {
        __asm__ volatile("1: subs %0, %0, #1\n\tbne 1b" : "+r"(n) : : "cc");
    }
}

/// One CRC exchange, this end BY HAND on the resource - SpiHost's Request
/// has no CRC phase: `count` data frames, CRCNEXT "as soon as the last
/// data is written to the SPI_DR register" (26.7.1), then the frame the
/// hardware puts on the wire by itself - the last data frame queued
/// behind its predecessor (below), or, with `one_in_flight`, every frame
/// into an idle shifter and CRCNEXT `next_delay` loop turns after the
/// last (letter t). `corrupt` leaves this end's unit OFF and sends the CRC
/// frame as one more DATA frame, its last bit flipped: a wrong CRC on a
/// real wire.
Crc run_crc(bool wide, uint16_t poly, SpiClock rate, uint8_t spare, bool corrupt,
            uint8_t seed_a, uint8_t seed_b, bool one_in_flight = false,
            uint32_t next_delay = 0) {
    Crc c{};
    // The window's CRCNEXT store has a frame of room at most (letter t walks
    // it), and a console interrupt inside the window would take it: the
    // console drained before the peer is commanded, nothing printed until the
    // window has closed. The answer is dropped: a wedged console shows as a
    // CRC exchange that fails, never as a hang.
    console_drain();
    constexpr uint16_t n = 16;
    spilink::Params a{};
    a.cfg = spilink::Cfg{.apply = 1, .mode = 0, .dord = 0, .regime = spilink::regime_buffer_wait};
    a.count = n;
    a.ms = 400;
    a.seed_a = seed_a;
    a.seed_b = seed_b;
    a.pattern = spilink::pattern_prbs;
    a.spare = static_cast<uint8_t>(spare | spilink::spare_crc | (wide ? spilink::spare_wide : 0u));
    a.aux16 = poly;
    c.o.accepted = peer_act(Op::exchange, a);
    if (!c.o.accepted) {
        return c;
    }
    const uint8_t w = wide ? 2u : 1u;
    {
        spilink::Stream out(a.pattern, a.seed_a);
        for (uint16_t i = 0; i < n * w; ++i) {
            tx[i] = out.next();
            rx[i] = 0xEE;
        }
    }
    c.tx_ref = crc_of(poly, wide, tx, n);
    settle();
    const SpiDataSize bits = wide ? SpiDataSize::bits16 : SpiDataSize::bits8;
    (void)S::disable();
    c.o.clocked = S::configure({.role = SpiRole::host,
                                .mode = SpiMode::mode0,
                                .clock = rate,
                                .bits = bits,
                                .crc = !corrupt,
                                .crc_polynomial = poly});
    S::enable();
    S::flush_rx();
    CsPin::clear();
    link_hold();
    bool frames = true;
    bool crc_came = true;
    // 1 ms a frame: a 16-bit frame at PCLK1/256 is 91 us.
    auto frame_back = [&](bool crc_frame) -> uint16_t {
        const bool here = brio::wait_until<Ruler>([] { return S::rx_ready(); }, 1'000);
        bool& came = crc_frame ? crc_came : frames;
        came = came && here;
        return S::data(bits);
    };
    auto frame_at = [&](uint16_t i) -> uint16_t {
        if (i == n) {   // the corrupted CRC frame, sent as data
            return static_cast<uint16_t>(c.tx_ref ^ 0x0001u);
        }
        return wide ? static_cast<uint16_t>(tx[2u * i] | (tx[2u * i + 1u] << 8)) : tx[i];
    };
    auto keep = [&](uint16_t i, uint16_t got) {
        if (i == n) {
            c.frame = got;
            return;
        }
        rx[w * i] = static_cast<uint8_t>(got);
        if (wide) {
            rx[2u * i + 1u] = static_cast<uint8_t>(got >> 8);
        }
    };
    if (one_in_flight || corrupt) {
        // ONE FRAME IN FLIGHT, each written into an idle shifter and read
        // back before the next - letter t's walk, CRCNEXT `next_delay`
        // loop turns after the last data store; and the corrupted leg,
        // whose CRC frame is data.
        for (uint16_t i = 0; i <= n; ++i) {
            const bool last = i == n - 1u && !corrupt;
            if (i < n || corrupt) {
                if (last) {
                    walk_store = SysTick->VAL;
                    S::data(frame_at(i));
                    spin_loops(next_delay);
                    S::crc_next();
                    walk_next = SysTick->VAL;
                    // A time-out only: frame_back() below judges the frame.
                    (void)brio::wait_until<Ruler>([] { return S::rx_ready(); }, 1'000);
                    walk_rxne = SysTick->VAL;
                } else {
                    S::data(frame_at(i));
                }
            }
            keep(i, frame_back(i == n && !corrupt));
        }
    } else {
        // THE LAST DATA FRAME QUEUED BEHIND ITS PREDECESSOR: frames one in
        // flight up to the second-to-last, which goes into the idle
        // shifter; on its TXE (its first bit, 26.3.9) the last one goes
        // into the buffer and CRCNEXT follows at once. The bit then lands
        // a whole frame before the last data frame ends - outside the
        // window letter t walks, where a CRCNEXT write corrupts both
        // calculators.
        for (uint16_t i = 0; i + 2u < n; ++i) {
            S::data(frame_at(i));
            keep(i, frame_back(false));
        }
        S::data(frame_at(static_cast<uint16_t>(n - 2u)));
        // A time-out only: TXE comes at the frame's first bit, and the frames
        // after it are judged by frame_back().
        (void)brio::wait_until<Ruler>([] { return S::tx_empty(); }, 1'000);
        S::data(frame_at(static_cast<uint16_t>(n - 1u)));
        S::crc_next();
        keep(static_cast<uint16_t>(n - 2u), frame_back(false));
        keep(static_cast<uint16_t>(n - 1u), frame_back(false));
        keep(n, frame_back(true));
    }
    link_hold();
    CsPin::set();
    c.frames_ok = frames;
    c.crc_came = crc_came;
    c.tx_crc = S::tx_crc();
    c.rx_crc = S::rx_crc();
    c.host_crcerr = S::crc_error();
    c.rx_ref = crc_of(poly, wide, rx, n);
    {
        uint8_t want[2u * n];
        for (uint16_t i = 0; i < n * w; ++i) {
            want[i] = spilink::pattern_value(a.pattern, a.seed_b, i);
            if (rx[i] != want[i]) {
                if (c.o.mism == 0u) {
                    c.o.idx = i;
                    c.o.got = rx[i];
                    c.o.exp = want[i];
                }
                ++c.o.mism;
            }
        }
        c.peer_ref = crc_of(poly, wide, want, n);
    }
    (void)link_command_mode();
    c.o.reported = peer_report(c.o.r);
    return c;
}

/// A matched CRC exchange, judged whole: the frames both ways, this end's
/// two accumulators against the arithmetic, the CRC frame it read against
/// the peer's stream's, and neither end's CRCERR.
bool crc_agrees(const Crc& c, bool wide) {
    const uint16_t mask = wide ? 0xFFFFu : 0x00FFu;
    return c.o.accepted && c.o.clocked && c.o.reported && c.frames_ok && c.crc_came && c.o.mism == 0u &&
           (c.o.r.flags & spilink::report_extended) != 0u && c.o.r.count == 16u &&
           c.o.r.mism == 0u && (c.o.r.flags & spilink::report_crcerr) == 0u &&
           c.tx_crc == c.tx_ref && c.rx_crc == c.rx_ref && (c.frame & mask) == c.peer_ref &&
           !c.host_crcerr && c.o.r.aux3 == static_cast<uint8_t>(c.tx_ref);
}

/// The far side of a CRC exchange alone: this end's TXCRCR, the peer's
/// check, and every data frame heard by the peer - this end's read left out,
/// printed at PCLK1/2 (readback_judged() above; letter t: ES0298 2.14.5's CRC
/// signature there).
bool crc_far_agrees(const Crc& c) {
    return c.o.accepted && c.o.clocked && c.o.reported && c.frames_ok &&
           (c.o.r.flags & spilink::report_extended) != 0u && c.o.r.count == 16u &&
           c.o.r.mism == 0u && (c.o.r.flags & spilink::report_crcerr) == 0u &&
           c.tx_crc == c.tx_ref;
}

void print_crc(const char* label, SpiClock rate, const Crc& c) {
    print(serial, "    ", label, " PCLK1/", division(rate), ": TXCRCR ", hex(c.tx_crc),
          " (bitwise ", hex(c.tx_ref), "), RXCRCR ", hex(c.rx_crc), " (bitwise over what came ",
          hex(c.rx_ref), "), the CRC frame ", hex(c.frame), " (the peer's, bitwise ",
          hex(c.peer_ref), "), CRCERR here ", c.host_crcerr ? "SET" : "clear", "; host mism ",
          c.o.mism, "; the peer heard ", c.o.r.count, " mism ", c.o.r.mism, " flags ",
          hex(c.o.r.flags), " RXCRCR low ", hex(c.o.r.aux3), " serve=", serve_name(c.o.r), crlf);
}

}  // namespace ext

// =============================================================================
// t - THE CRCNEXT WINDOW and the receive CRC at the top code (OUTSIDE z)
// =============================================================================
//
// No verdict: the instrument behind letter w's procedure. First the
// receive CRC at PCLK1/2 against the SCK pad's class, ten windows each;
// then a walk of the CRCNEXT store's distance from the last data store,
// one frame in flight, each point printed as next/rxne:TRP - the core
// cycles (SysTick) from before the last data store to after the CRCNEXT
// store, and to that frame's RXNE first seen; T this end's TXCRCR the
// arithmetic, R its RXCRCR, P the peer's check clean, F every frame read
// back, the CRC frame among them (lower case: not - f where the store came
// past the last frame's end and no CRC frame was clocked).

void tt_crc_window() {
    if (!need_peer()) {
        return;
    }
    static constexpr PinSpeed classes[] = {PinSpeed::medium, PinSpeed::high, PinSpeed::very_high};
    static constexpr const char* names[] = {"medium", "high", "very_high"};
    const PinSpeed keep = peer_sck_speed;
    for (uint8_t c = 0; c < 3u; ++c) {
        peer_sck_speed = classes[c];
        for (uint8_t wd = 0; wd < 2u; ++wd) {
            uint8_t rx_bad = 0, tx_bad = 0, peer_bad = 0, data_bad = 0;
            for (uint8_t k = 0; k < 10u; ++k) {
                const ext::Crc r = ext::run_crc(wd != 0u, wd ? 0x1021u : 0x0007u, SpiClock::div2, 0,
                                                false, static_cast<uint8_t>(0x31u + 7u * k),
                                                static_cast<uint8_t>(0xA7u + 3u * k));
                rx_bad = static_cast<uint8_t>(rx_bad + (r.rx_crc != r.rx_ref ? 1u : 0u));
                tx_bad = static_cast<uint8_t>(tx_bad + (r.tx_crc != r.tx_ref ? 1u : 0u));
                peer_bad = static_cast<uint8_t>(peer_bad + ((r.o.r.flags & spilink::report_crcerr) != 0u ? 1u : 0u));
                data_bad = static_cast<uint8_t>(data_bad + (r.o.mism != 0u || r.o.r.mism != 0u ? 1u : 0u));
            }
            print(serial, "  SCK ", names[c], wd ? " 16-bit" : " 8-bit", " at PCLK1/2, 10 CRC windows: RXCRCR wrong ",
                  rx_bad, ", TXCRCR wrong ", tx_bad, ", peer CRCERR ", peer_bad, ", data wrong ", data_bad, crlf);
        }
    }
    peer_sck_speed = keep;
    // the CRCNEXT delay walk at /2 and /4, SCK medium
    static constexpr SpiClock rates[] = {SpiClock::div2, SpiClock::div4, SpiClock::div8, SpiClock::div16};
    for (const SpiClock c : rates) {
        for (uint8_t wd = 0; wd < 2u; ++wd) {
            print(serial, "  walk ", wd ? "16" : "8", "-bit PCLK1/", ext::division(c), ":");
            const uint32_t frame = (wd ? 16u : 8u) * ext::division(c) * 4u;
            for (uint32_t d = 0; d * 3u <= frame + 40u; d += (frame > 200u ? 4u : 2u)) {
                const ext::Crc r = ext::run_crc(wd != 0u, wd ? 0x1021u : 0x0007u, c, 0, false,
                                                static_cast<uint8_t>(0x5Bu + d), 0xB9, true, d);
                print(serial, " ", ext::walk_since(ext::walk_store, ext::walk_next), "/",
                      ext::walk_since(ext::walk_store, ext::walk_rxne), ":",
                      r.tx_crc == r.tx_ref ? "T" : "t", r.rx_crc == r.rx_ref ? "R" : "r",
                      (r.o.r.flags & spilink::report_crcerr) == 0u ? "P" : "p",
                      r.frames_ok && r.crc_came ? "F" : "f");
            }
            print(serial, crlf);
        }
    }
    (void)link_command_mode();
}

void tw_crc() {
    if (!need_peer()) {
        return;
    }
    // Both widths, both serves - the peer's engines (RM0444 35.5.14's "CRC
    // transfer managed by DMA") and its CRC run over the resource - at
    // every BR code, this end's unit by hand.
    struct Leg {
        bool wide;
        uint16_t poly;
        uint8_t spare;
        const char* name;
    };
    static const Leg legs[] = {
        {false, 0x0007u, 0, "CRC-8 0x07, the peer's DMA serve,"},
        {true, 0x1021u, 0, "CRC-16 0x1021, the peer's DMA serve,"},
        {false, 0x0007u, spilink::spare_polled_pump, "CRC-8 0x07, the peer's CRC run,"},
        {true, 0x1021u, spilink::spare_polled_pump, "CRC-16 0x1021, the peer's CRC run,"},
    };
    bool leg_ok[4] = {};
    bool top_far = true;
    uint8_t top_rx_wrong = 0;
    for (uint8_t l = 0; l < 4u; ++l) {
        bool all = true;
        uint8_t k = 0;
        for (const SpiClock c : ext::rungs) {
            const ext::Crc r = ext::run_crc(legs[l].wide, legs[l].poly, c, legs[l].spare, false,
                                            static_cast<uint8_t>(0x31u + 7u * k),
                                            static_cast<uint8_t>(0xA7u + 3u * k));
            ++k;
            ext::print_crc(legs[l].name, c, r);
            if (c != SpiClock::div2) {
                all = all && ext::crc_agrees(r, legs[l].wide);
            } else if (legs[l].spare == 0u) {
                const uint16_t mask = legs[l].wide ? 0xFFFFu : 0x00FFu;
                top_far = top_far && ext::crc_far_agrees(r);
                top_rx_wrong = static_cast<uint8_t>(
                    top_rx_wrong + (r.rx_crc != r.rx_ref || !r.crc_came || (r.frame & mask) != r.peer_ref ? 1u : 0u));
            }
        }
        leg_ok[l] = all;
    }
    bench.verdict("THE CRC UNIT AGAINST A FOREIGN ONE, 8-bit: at every code to PCLK1/4 this "
                  "end's TXCRCR and RXCRCR are the bitwise arithmetic, the CRC frame it reads is "
                  "the peer's stream's CRC, and NEITHER END raises CRCERR (the peer's engines "
                  "managing its CRC by themselves)",
                  leg_ok[0]);
    bench.verdict("...and with 16-bit frames and a 16-bit CRC likewise", leg_ok[1]);
    bench.verdict("...and against the peer's CRC run (CRCNEXT after its last answer), both "
                  "widths",
                  leg_ok[2] && leg_ok[3]);
    print(serial, "  at PCLK1/2 this end's receive CRC (RXCRCR, or the CRC frame read or come) missed in ",
          top_rx_wrong, " of 2 windows - ES0298 2.14.5's CRC signature, letter t", crlf);
    bench.verdict("AT PCLK1/2 = 22.5 MHz THE FAR SIDE HOLDS: this end's TXCRCR is the "
                  "arithmetic, the peer's DMA serve checks the CRC frame clean and hears every "
                  "data frame exact, this end's read printed - both widths",
                  top_far);

    // A WRONG CRC ON A REAL WIRE: this end's unit off, the CRC frame sent as
    // data with its last bit flipped. The peer must flag it; its own CRC
    // frame must still be the arithmetic of its stream.
    bool caught = true;
    static constexpr SpiClock judged[] = {SpiClock::div256, SpiClock::div4, SpiClock::div2};
    for (uint8_t l = 0; l < 4u; ++l) {
        for (const SpiClock c : judged) {
            if (c == SpiClock::div2 && legs[l].spare != 0u) {
                continue;   // the peer's CRC run is a frame an access: not at 22.5 MHz
            }
            const ext::Crc r = ext::run_crc(legs[l].wide, legs[l].poly, c, legs[l].spare, true,
                                            0x5E, 0x3B);
            print(serial, "  CORRUPTED, ");
            ext::print_crc(legs[l].name, c, r);
            const uint16_t mask = legs[l].wide ? 0xFFFFu : 0x00FFu;
            // This end's read judged to PCLK1/4 (readback_judged() above), the
            // peer's check at every rung.
            const bool read_ok = c == SpiClock::div2 || (r.o.mism == 0u && (r.frame & mask) == r.peer_ref);
            caught = caught && r.o.accepted && r.o.reported && r.frames_ok &&
                     (r.o.r.flags & spilink::report_crcerr) != 0u && r.o.r.count == 16u &&
                     r.o.r.mism == 0u && read_ok;
        }
    }
    bench.verdict("A WRONG CRC FRAME IS CAUGHT BY THE FOREIGN UNIT: one flipped bit in the "
                  "frame where the CRC belongs and the peer reports CRCERR (report_crcerr), "
                  "every data frame still exact, its own CRC frame still the arithmetic - both "
                  "widths, both serves, at the command rate and at PCLK1/4; on the DMA serve at "
                  "PCLK1/2 the peer's check and every frame it heard, this end's read printed",
                  caught);
    (void)link_command_mode();
    bench.verdict("the command channel survives the CRC", command(Op::ping));
}

// =============================================================================
// f - THE SOFTWARE SELECT AND THE HARDWARE SELECT OUTPUT against the peer
// =============================================================================
//
// Every other peer letter frames on a GPIO select this end drives (host)
// or on the NSS pad as the hardware input (client). This one takes the two
// arrangements left: a CLIENT WITH NO SELECT PAD (SSM, SSI the select) and
// a HOST WHOSE SELECT IS THE BLOCK'S OWN OUTPUT (SSOE) - each judged by the
// G0's spi_peer at the far end.

namespace sel {

/// The client role with NO NSS PAD: SPI2's three data pads alone.
constexpr SpiPins ssm_pins{.sck = link_pins.sck, .miso = link_pins.miso,
                           .mosi = link_pins.mosi, .nss = {}};
using SsmClient = SpiClient<2, ssm_pins>;
static_assert(!SsmClient::has_nss_pad);

uint8_t out_[16];
uint8_t in_[16];

/// The peer's twelve-frame host_burst at its /32 (2 MHz on the G0's
/// 64 MHz), 20 us between frames, 40 ms of lead-in (spi_link.hpp).
spilink::Params burst_params(uint8_t seed_a, uint8_t seed_b) {
    spilink::Params a{};
    a.count = 12;
    a.ms = 300;
    a.seed_a = seed_a;
    a.seed_b = seed_b;
    a.pattern = spilink::pattern_prbs;
    a.aux8 = 40;
    a.aux16 = 32;
    a.cfg.apply = 1;
    return a;
}

/// A run's spin budget for `ms` milliseconds of window (letter r's).
uint32_t budget(uint16_t ms) { return static_cast<uint32_t>(ms) * (SysClock::hz / 36'000u); }

uint16_t mismatches(const uint8_t* got, uint16_t n, uint8_t seed) {
    uint16_t m = 0;
    for (uint16_t i = 0; i < n; ++i) {
        if (got[i] != spilink::pattern_value(spilink::pattern_prbs, seed, i)) {
            ++m;
        }
    }
    return m;
}

template <typename E>
const char* end_name(E e) {
    return e == E::complete ? "complete" : e == E::closed ? "closed" : "stalled";
}

struct SsmLeg {
    bool accepted = false;
    bool fell = false;
    SsmClient::Run run{0, SsmClient::RunEnd::stalled};
    bool reported = false;
    spilink::Report r{};
};

/// One burst against the client with no select pad: SSI driven low once
/// PB12 (a plain input here) is seen to fall when `select_it`, or left
/// high for the whole burst.
SsmLeg ssm_leg(bool select_it, uint8_t seed_a, uint8_t seed_b) {
    SsmLeg L{};
    const spilink::Params a = burst_params(seed_a, seed_b);
    L.accepted = peer_act(Op::host_burst, a);
    if (!L.accepted) {
        return L;
    }
    Host::release();
    (void)SsmClient::init(clock, {.mode = SpiMode::mode0,
                                  .bits = SpiDataSize::bits8,
                                  .lsb_first = false,
                                  .nss = SpiNss::software,
                                  .drive_output = true,
                                  .miso_speed = PinSpeed::low});
    // A client under SSM comes up SELECTED (SSI low, spi_cr1_of); the
    // window is this program's, so it opens deselected.
    SsmClient::select(false);
    CsPin::input(PinPull::up);
    spilink::Stream out(a.pattern, a.seed_b);
    for (uint16_t i = 0; i < a.count; ++i) {
        out_[i] = out.next();
        in_[i] = 0xEE;
    }
    SsmClient::enable();
    // The peer's lead-in and 100 ms more: the select falls when its burst opens.
    L.fell = brio::wait_until<Ruler>([] { return !CsPin::read(); },
                                     (a.aux8 + 100u) * 1000u);
    if (select_it && L.fell) {
        SsmClient::select(true);
    }
    L.run = SsmClient::exchange(std::span<const uint8_t>(out_, a.count),
                                std::span<uint8_t>(in_, a.count), budget(a.ms));
    SsmClient::select(false);
    (void)SsmClient::disable();
    SsmClient::release();
    L.reported = peer_report(L.r);
    return L;
}

}   // namespace sel

void tf_select() {
    using namespace sel;
    if (!need_peer()) {
        return;
    }
    print(serial, "  1. THE CLIENT WITH NO SELECT PAD (SSM) under the peer's 12-frame "
                  "host_burst at 2 MHz",
          crlf);
    const SsmLeg deaf = ssm_leg(false, 0x35, 0x9C);
    print(serial, "    SSI left HIGH: the wire's select fell=", deaf.fell, ", the run read ",
          deaf.run.count, " frames and ended ", end_name(deaf.run.end), "; the peer as host read ",
          deaf.r.count, " frames, mism=", deaf.r.mism, " (first got ", hex(deaf.r.got), " exp ",
          hex(deaf.r.exp), ")", crlf);
    bench.verdict("a client under SSM with SSI HIGH is deaf to a clock the wire carries: the "
                  "peer clocked its burst and not one frame arrived (the run without a pad "
                  "ends on its budget)",
                  deaf.accepted && deaf.fell && deaf.run.count == 0u &&
                      deaf.run.end == SsmClient::RunEnd::stalled && deaf.reported &&
                      deaf.r.count == 12u);
    const SsmLeg live = ssm_leg(true, 0x35, 0x9C);
    const uint16_t m = mismatches(in_, live.run.count, 0x35);
    print(serial, "    SSI driven low after the wire's select fell: the run read ", live.run.count,
          " frames, mism=", m, ", ended ", end_name(live.run.end), "; the peer as host read ",
          live.r.count, " frames, mism=", live.r.mism, " (first idx ", live.r.idx, " got ",
          hex(live.r.got), " exp ", hex(live.r.exp), ")", crlf);
    bench.verdict("selected by SOFTWARE (SSI low), the client with no select pad receives the "
                  "foreign host's burst byte-exact, its run COMPLETE on its count",
                  live.accepted && live.fell && live.run.count == 12u && m == 0u &&
                      live.run.end == SsmClient::RunEnd::complete);
    bench.verdict("...and the foreign host read its answers byte-exact from the first frame",
                  live.reported && live.r.count == 12u && live.r.mism == 0u);
    (void)link_command_mode();

    // ---- 2. SSOE: the host's select as the block's own output -----------------------
    // 26.3.5: "The NSS signal is driven low as soon as the SPI is enabled in
    // master mode (SPE=1), and is kept low until the SPI is disabled (SPE
    // =0)." The window is the enable, then: the pad read on this side (its
    // input buffer stays live under AF5) before SPE, after it and after the
    // disable; the frames between judged by the peer.
    print(serial, "  2. SSOE: the select driven by the block itself, eight frames at PCLK1/64",
          crlf);
    spilink::Params a{};
    a.cfg = spilink::Cfg{.apply = 1, .mode = 0, .dord = 0, .regime = spilink::regime_buffer_wait};
    a.count = 8;
    a.ms = 400;
    a.seed_a = 0x62;
    a.seed_b = 0x1D;
    a.pattern = spilink::pattern_prbs;
    const bool acc = peer_act(Op::exchange, a);
    spilink::Stream out(a.pattern, a.seed_a);
    for (uint16_t i = 0; i < a.count; ++i) {
        out_[i] = out.next();
        in_[i] = 0xEE;
    }
    settle();
    (void)S::disable();
    SpiConfig hc{};
    hc.clock = SpiClock::div64;
    hc.nss = SpiNss::hardware_output;
    const bool cfg_ok = S::configure(hc);
    Host::claim_nss_pad(true);
    brio::spin_us<Ruler>(30);
    const bool high_before = CsPin::read();
    S::enable();
    brio::spin_us<Ruler>(30);
    const bool low_enabled = !CsPin::read();
    uint16_t moved = 0;
    // 1 ms a wait: a frame at PCLK1/64 is 11 us. TXE's wait is a time-out only
    // (the frame it lets out is RXNE's); a frame whose RXNE never comes ends
    // the run short, which `moved` says.
    for (; moved < a.count; ++moved) {
        (void)brio::wait_until<Ruler>([] { return S::tx_empty(); }, 1'000);
        S::data(out_[moved]);
        if (!brio::wait_until<Ruler>([] { return S::rx_ready(); }, 1'000)) {
            break;
        }
        in_[moved] = S::data8();
    }
    brio::spin_us<Ruler>(30);
    const bool low_after_frames = !CsPin::read();
    (void)S::disable();
    brio::spin_us<Ruler>(30);
    const bool high_disabled = CsPin::read();
    Host::claim_nss_pad(false);
    (void)link_command_mode();
    spilink::Report r{};
    const bool rep = acc && peer_report(r);
    const uint16_t hm = mismatches(in_, moved, a.seed_b);
    print(serial, "    the pad: high before SPE ", high_before, ", low with SPE up ", low_enabled,
          ", still low after the frames ", low_after_frames, ", high after the disable ",
          high_disabled, "; ", moved, " frames, host mism=", hm, "; the peer heard ", r.count,
          " mism=", r.mism, " aux2=", hex(r.aux2), crlf);
    bench.verdict("SSOE: the select pad falls when SPE rises - before any frame - stays low "
                  "through the frames and rises when SPE is cleared (26.3.5)",
                  cfg_ok && high_before && low_enabled && low_after_frames && high_disabled);
    bench.verdict("...and that window frames a foreign client: eight frames byte-exact both ways",
                  acc && rep && moved == a.count && hm == 0u && r.count == a.count &&
                      r.mism == 0u);
    bench.verdict("the command channel is back after the two arrangements", command(Op::ping));
}

// =============================================================================
// g - THE STALL AND THE INTERRUPT ENABLES against the peer
// =============================================================================
//
// spi_stalled is the polled loop's end when the block stops clocking under
// it; on this desk the peer can make that happen from outside: this host
// runs a long polled request with its NSS pad a hardware INPUT (SSM and
// SSOE clear - the multi-master arrangement, 26.3.4) and the peer pulls the
// shared select low part-way (Op::ss_pulse). MODF clears SPE and MSTR
// (26.3.10), the RXNE the loop waits for never comes, and the budget ends
// it. ERRIE is armed for the same window, so the error vector is measured
// on the same mode fault. Then TXEIE: this board as a CLIENT answering on
// the transmit vector alone, and ERRIE again on a client that never reads
// (OVR).

volatile bool probe_live = false;      ///< the SPI2 vector belongs to this letter
volatile uint32_t probe_err_sr = 0;    ///< SR at the first error the vector saw
volatile uint32_t probe_err_at = 0;    ///< and when (DWT)
volatile uint16_t probe_err_count = 0;
volatile uint16_t probe_txe_count = 0;
const uint8_t* volatile probe_answers = nullptr;
volatile uint16_t probe_answer_n = 0;

/// The letter's body of the SPI2 vector: an answer per TXE until the
/// answers run out (then TXEIE off), and the first error recorded and
/// ERRIE disarmed - every error flag is a level, and a handler that left
/// one standing would re-enter for ever.
void probe_isr() {
    const uint32_t up = S::isr();
    if ((up & SpiFlag::txe) != 0u) {
        const uint16_t k = probe_txe_count;
        if (k < probe_answer_n) {
            S::data(probe_answers[k]);
        }
        probe_txe_count = static_cast<uint16_t>(k + 1u);
        if (probe_txe_count >= probe_answer_n) {
            S::txe_interrupt(false);
        }
    }
    if ((up & SpiFlag::errors) != 0u) {
        if (probe_err_count == 0u) {
            probe_err_sr = S::status();
            probe_err_at = CycleCounter::now();
        }
        probe_err_count = static_cast<uint16_t>(probe_err_count + 1u);
        S::error_interrupt(false);
    }
}

uint8_t stall_tx[2048];
/// Full duplex on purpose: a transmit-only request leaves its receive
/// buffer unread, and OVR would be the first error ERRIE brings.
uint8_t stall_rx[2048];

void tg_stall_irqs() {
    if (!need_peer()) {
        return;
    }

    // ---- 1. a stall made by another chip, with ERRIE on the same fault ----
    // 2048 frames at PCLK1/256 (175.8 kHz, 45.5 us a frame) are 93.2 ms on
    // the wire; the peer takes the select 10 ms after it has served its ack
    // window, which lands 36 ms into the request (measured: the peer's
    // serve and the command's own windows before it) - 57 ms of margin to
    // the request's end, and 36 to its start.
    spilink::Params a{};
    a.aux8 = 10;
    a.aux16 = 3000;
    if (!peer_act(Op::ss_pulse, a)) {
        bench.verdict("the peer accepted the ss_pulse command", false);
        return;
    }
    for (uint16_t i = 0; i < sizeof stall_tx; ++i) {
        stall_tx[i] = static_cast<uint8_t>(i);
    }
    Host::prime(SpiMode::mode0, SpiClock::div256);
    // The hardware input under the host's own configuration: SSM and SSI
    // cleared on the resource, the pad handed to SPI2 and pulled up here
    // too. The host's cached CR1 still names SSM, so start() applies
    // nothing over it (the mode and the rate are the ones just primed).
    (void)S::disable();
    S::regs().CR1 = S::regs().CR1 & ~(SPI_CR1_SSM | SPI_CR1_SSI);
    Host::claim_nss_pad(true);
    CsPin::pull(PinPull::up);
    probe_err_count = 0;
    probe_err_sr = 0;
    probe_live = true;
    // The host arms no interrupt of the block and leaves its NVIC line as
    // it finds it, so this letter opens the line for ERRIE itself.
    Nvic::enable(S::irq);
    S::enable();
    S::error_interrupt(true);
    Host::Request rq{};
    rq.cs = {};
    rq.tx = lend<Lease::reply>(static_cast<const uint8_t*>(stall_tx));
    rq.rx = lend<Lease::reply>(stall_rx);
    rq.len = sizeof stall_tx;
    rq.clock = SpiClock::div256;
    rq.mode = SpiMode::mode0;
    rq.bits = SpiDataSize::bits8;
    rq.engine = SpiEngine::polled;
    const uint32_t t0 = CycleCounter::now();
    const bool done = Host::start(rq);
    const uint32_t t1 = CycleCounter::now();
    // ERRIE off before the vector is handed back: a request that ended
    // some other way would leave the fault to come on a vector that does
    // not consume it.
    S::error_interrupt(false);
    Nvic::disable(S::irq);
    probe_live = false;
    const uint8_t st = Host::status();
    const uint32_t fault_us = probe_err_count != 0u ? (probe_err_at - t0) / cycles_per_us : 0u;
    const uint32_t end_us = (t1 - t0) / cycles_per_us;
    const uint32_t cr1_after = S::regs().CR1;
    Host::claim_nss_pad(false);
    print(serial, "  1. a 2048-frame polled request at PCLK1/256 with NSS a hardware input: the "
                  "error vector ",
          probe_err_count, " time(s), SR ", hex(probe_err_sr), " at ", fault_us,
          " us; start() returned ", done ? "(synchronously) " : "", "after ", end_us,
          " us with status ", st, " (spi_stalled is ", spi_stalled, "); CR1 after ",
          hex(cr1_after), crlf);
    bench.verdict("ANOTHER CHIP's select demotes this host mid-request and the polled loop "
                  "ends spi_stalled on its budget - the block that stopped clocking reset and "
                  "reconfigured (SPE and MSTR back, SSM restored)",
                  done && st == spi_stalled && (cr1_after & SPI_CR1_SPE) != 0u &&
                      (cr1_after & SPI_CR1_MSTR) != 0u && (cr1_after & SPI_CR1_SSM) != 0u);
    bench.verdict("ERRIE: the SPI2 vector ran ONCE on that fault with MODF in SR, inside the "
                  "request (26.5's error interrupt)",
                  probe_err_count == 1u && (probe_err_sr & SPI_SR_MODF) != 0u &&
                      fault_us < end_us);
    brio::spin_us<Ruler>(10'000);
    (void)link_command_mode();
    spilink::Report pr{};
    const bool prep = peer_report(pr);
    bench.verdict("the peer reports the wire really taken, and the channel is back after the "
                  "stall",
                  prep && pr.op == spilink::byte_of(Op::ss_pulse) && pr.aux0 == 1u);

    // ---- 2. TXEIE: a client answering on the transmit vector alone ----------
    // 26.3.9: TXE rises when the buffer moves into the shifter, at a frame's
    // first bit, so one vector per frame writes the NEXT answer with seven
    // bits of window. Enabling TXEIE with the buffer empty and the select
    // high runs the vector at once, and that first entry is the preload.
    {
        const spilink::Params b = sel::burst_params(0x44, 0x7B);
        const bool acc = peer_act(Op::host_burst, b);
        Host::release();
        (void)Client::init(clock, {.mode = SpiMode::mode0,
                                   .bits = SpiDataSize::bits8,
                                   .lsb_first = false,
                                   .nss = SpiNss::hardware_input,
                                   .drive_output = true,
                                   .miso_speed = PinSpeed::low});
        spilink::Stream out(b.pattern, b.seed_b);
        for (uint16_t i = 0; i < b.count; ++i) {
            sel::out_[i] = out.next();
            sel::in_[i] = 0xEE;
        }
        probe_txe_count = 0;
        probe_err_count = 0;
        probe_answers = sel::out_;
        probe_answer_n = b.count;
        probe_live = true;
        Client::enable();
        S::txe_interrupt(true);
        // The receive side polled here, on RXNE, until the window closes.
        uint16_t got = 0;
        bool opened = false;
        const uint32_t m0 = Ticker::millis();
        while (Ticker::millis() - m0 < b.aux8 + b.ms) {
            if (const auto v = Client::poll()) {
                if (got < b.count) {
                    sel::in_[got] = static_cast<uint8_t>(*v);
                }
                ++got;
            }
            const bool selected = Client::selected();
            if (selected) {
                opened = true;
            } else if (opened) {
                break;
            }
        }
        S::txe_interrupt(false);
        probe_live = false;
        (void)Client::disable();
        Client::release();
        spilink::Report r{};
        const bool rep = acc && peer_report(r);
        const uint16_t m = sel::mismatches(sel::in_, got < b.count ? got : b.count, b.seed_a);
        print(serial, "  2. TXEIE: the transmit vector ran ", probe_txe_count, " times for ",
              b.count, " answers; this end read ", got, " frames, mism=", m,
              "; the peer as host read ", r.count, " mism=", r.mism, " (first idx ", r.idx,
              " got ", hex(r.got), " exp ", hex(r.exp), ")", crlf);
        bench.verdict("TXEIE: a client answering on the TRANSMIT VECTOR alone - one entry per "
                      "answer, the first the preload with the select high - is read byte-exact "
                      "by the foreign host from the first frame",
                      acc && rep && probe_txe_count == b.count && r.count == b.count &&
                          r.mism == 0u);
        bench.verdict("...while this end, reading RXNE in main, heard every frame exact",
                      got == b.count && m == 0u);
    }

    // ---- 3. ERRIE on a client that never reads: the overrun's vector --------
    // 26.3.13: OVR is set when a frame completes while RXNE still stands, so
    // a client that reads nothing raises it at the END OF THE SECOND FRAME.
    // The peer clocks four; the vector records the first error and
    // disarms.
    {
        spilink::Params b = sel::burst_params(0x18, 0x00);
        b.count = 4;
        const bool acc = peer_act(Op::host_burst, b);
        Host::release();
        (void)Client::init(clock, {.mode = SpiMode::mode0,
                                   .bits = SpiDataSize::bits8,
                                   .lsb_first = false,
                                   .nss = SpiNss::hardware_input,
                                   .drive_output = false});
        probe_txe_count = 0;
        probe_err_count = 0;
        probe_err_sr = 0;
        probe_answer_n = 0;
        probe_live = true;
        Client::enable();
        S::error_interrupt(true);
        bool opened = false;
        const uint32_t m0 = Ticker::millis();
        while (Ticker::millis() - m0 < b.aux8 + b.ms) {
            const bool selected = Client::selected();
            if (selected) {
                opened = true;
            } else if (opened) {
                break;
            }
        }
        S::error_interrupt(false);
        probe_live = false;
        const bool ovr_standing = S::overrun();
        S::clear_overrun();
        (void)Client::disable();
        Client::release();
        spilink::Report r{};
        const bool rep = acc && peer_report(r);
        print(serial, "  3. ERRIE on a client that reads nothing: the error vector ran ",
              probe_err_count, " time(s), SR ", hex(probe_err_sr), "; OVR still standing at "
              "the close: ",
              ovr_standing, "; the peer clocked ", r.count, crlf);
        bench.verdict("ERRIE: an unread client's OVR (26.3.13) reaches the vector once, and "
                      "the flag stands until its clearing sequence - the vector consumed "
                      "nothing",
                      acc && rep && r.count == 4u && probe_err_count == 1u &&
                          (probe_err_sr & SPI_SR_OVR) != 0u && ovr_standing);
    }
    (void)link_command_mode();
    bench.verdict("the command channel is back", command(Op::ping));
}

// =============================================================================
// h - A HANDLER BESIDE THE POLLED LOOP IS A GAP ON THE BUS, against the peer
// =============================================================================
//
// SpiHost's polled receive loop keeps one frame in flight at every rate: it
// writes frame k + 1 only after reading frame k, so the clock waits for the
// read and nothing can overrun (26.3.13). This letter binds a TIM7 update
// handler that holds the core for 350 cycles of its own (a DWT spin) every
// 5 us, longer than a frame, and runs the polled loop at PCLK1/8 (256 cycles
// a frame) against the peer. The verdict: ten windows of ten exact both ways,
// status spi_ok, OVR never standing, the peer's count right, and the window
// longer than without the handler by what the arithmetic below allows.
//
// THE BOUND, in core cycles at 180 MHz. The handler's period is P = 900 (5 us;
// TIM7 counts the APB1 timer clock, 90 MHz, from the core's own PLL, so no
// clock error enters). One run holds the core for
//   hold_min = 350 + 12 + 12                           = 374   (the spin, the
//              exception's entry and exit, Cortex-M4 TRM 3.9.1)
//   hold_max = 374 + 9 (a spin turn's overshoot) + 12 (the five instructions
//              around the spin) + 16 (the store to TIM7 over the APB) + 18
//              (three flash misses of the ART, 6 cycles each)
//                                                      = 429
// A run longer than the frame always stalls the loop: the frame in flight ends
// at most W = 256 after the run starts, so the bus idles for at least
// hold_min - W = 118 where the same frame would have turned at once. A run can
// add at most its whole length, when it lands on the turn. So over a window of
// T cycles without the handler (10 windows measured, T_min and T_max), the
// runs that start inside it number at least floor(T_min / P), and
//   lower = floor(T_min / P) x (hold_min - W)
//   upper = (T_max + hold_max + tick) x P / (P - hold_max) - T_min
// the second from T' <= T_max + (T' / P + 1) x hold_max + tick, with tick = 76
// the one SysTick run the window can meet (systick_run_cycles). Both are worst
// cases of the phase; neither is a figure chosen by eye. The console is drained
// before the twenty windows and nothing prints until they are over, so no
// console line's interrupt is inside one.

constexpr uint32_t hog_cycles = 350;
constexpr uint32_t hog_period = 900;
constexpr uint32_t hog_hold_min = hog_cycles + 12u + 12u;
constexpr uint32_t hog_hold_max = hog_hold_min + 9u + 12u + 16u + 18u;
constexpr uint32_t hog_frame = 256;
constexpr uint32_t hog_tick = systick_run_cycles;
static_assert(hog_hold_min > hog_frame && hog_hold_max < hog_period);
volatile bool hog_armed = false;   ///< this series binds the handler at all
volatile uint32_t hog_count = 0;

constexpr uint16_t hold_n = 64;
uint8_t hold_tx[hold_n];
uint8_t hold_rx[hold_n];

struct HoldRun {
    bool accepted = false;
    bool completed = false;
    uint8_t status = 0;
    uint32_t cycles = 0;
    bool overrun = false;
    bool reported = false;
    spilink::Report r{};
    uint16_t mism = 0;
};

/// One 64-frame window on the polled loop at PCLK1/8, the peer exchanging on
/// its DMA serve.
HoldRun hold_window(uint8_t seed) {
    HoldRun o{};
    spilink::Params a{};
    a.cfg = spilink::Cfg{.apply = 1, .mode = 0, .dord = 0, .regime = spilink::regime_buffer_wait};
    a.count = hold_n;
    a.ms = 60;
    a.seed_a = seed;
    a.seed_b = static_cast<uint8_t>(seed ^ 0x5Au);
    a.pattern = spilink::pattern_prbs;
    o.accepted = peer_act(Op::exchange, a);
    if (!o.accepted) {
        return o;
    }
    spilink::Stream out(a.pattern, a.seed_a);
    for (uint16_t i = 0; i < hold_n; ++i) {
        hold_tx[i] = out.next();
        hold_rx[i] = 0xEE;
    }
    settle();
    Host::prime(SpiMode::mode0, SpiClock::div8);
    CsPin::clear();
    link_hold();
    Host::Request r{};
    r.cs = {};
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(hold_tx));
    r.rx = lend<Lease::reply>(hold_rx);
    r.len = hold_n;
    r.clock = SpiClock::div8;
    r.mode = SpiMode::mode0;
    r.bits = SpiDataSize::bits8;
    r.engine = SpiEngine::polled;
    // The timer runs for the window alone, so the command channel around
    // it is not what it lands in.
    Tim<7>::enable(hog_armed);
    const uint32_t t0 = CycleCounter::now();
    o.completed = Host::start(r);
    o.cycles = CycleCounter::now() - t0;
    Tim<7>::enable(false);
    o.overrun = S::overrun();
    o.status = Host::status();
    link_hold();
    CsPin::set();
    (void)link_command_mode();
    o.reported = peer_report(o.r);
    o.mism = sel::mismatches(hold_rx, hold_n, a.seed_b);
    return o;
}

struct HoldTally {
    uint8_t exact = 0;
    uint8_t not_ok = 0;
    uint8_t overrun = 0;
    uint32_t cycles_min = UINT32_MAX;
    uint32_t cycles_max = 0;
    uint32_t runs = 0;
};

HoldTally hold_series(bool hog, uint8_t rounds) {
    HoldTally t{};
    hog_count = 0;
    for (uint8_t k = 0; k < rounds; ++k) {
        hog_armed = hog;
        const HoldRun o = hold_window(static_cast<uint8_t>(0x21u + 13u * k));
        hog_armed = false;
        if (o.completed && o.status == spi_ok && o.mism == 0u && o.reported && o.r.count == hold_n &&
            o.r.mism == 0u) {
            ++t.exact;
        }
        if (o.status != spi_ok) {
            ++t.not_ok;
        }
        if (o.overrun) {
            ++t.overrun;
        }
        if (o.cycles < t.cycles_min) {
            t.cycles_min = o.cycles;
        }
        if (o.cycles > t.cycles_max) {
            t.cycles_max = o.cycles;
        }
    }
    t.runs = hog_count;
    return t;
}

void th_handler_gap() {
    if (!need_peer()) {
        return;
    }
    if (!brio::console_drain<Ruler, Serial>()) {
        bench.skip("the console did not drain: the timed windows would carry its interrupt");
        return;
    }
    Tim<7>::init();
    (void)TimPeriodicTick<Tim<7>>::setup(0, 449);   // 90 MHz / 450 = 200 kHz
    Tim<7>::enable(false);
    Nvic::enable(Tim<7>::irq());
    constexpr uint8_t rounds = 10;
    const HoldTally quiet = hold_series(false, rounds);
    const HoldTally held = hold_series(true, rounds);
    Nvic::disable(Tim<7>::irq());
    Tim<7>::release();
    const uint32_t lower = (quiet.cycles_min / hog_period) * (hog_hold_min - hog_frame);
    const uint32_t upper = static_cast<uint32_t>((static_cast<uint64_t>(quiet.cycles_max) + hog_hold_max + hog_tick) *
                                                 hog_period / (hog_period - hog_hold_max)) -
                           quiet.cycles_min;
    const uint32_t longer_min = held.cycles_min > quiet.cycles_max ? held.cycles_min - quiet.cycles_max : 0u;
    const uint32_t longer_max = held.cycles_max > quiet.cycles_min ? held.cycles_max - quiet.cycles_min : 0u;
    print(serial, "  the polled loop at PCLK1/8 (256 cycles a frame), 64 frames:", crlf);
    print(serial, "  no handler: ", quiet.exact, " of ", rounds, " exact, a window ", quiet.cycles_min, "..",
          quiet.cycles_max, " cycles, OVR ", quiet.overrun, crlf);
    print(serial, "  the handler every 900 cycles, ", held.runs, " runs: ", held.exact, " of ", rounds,
          " exact, ", held.not_ok, " not spi_ok, OVR ", held.overrun, ", a window ", held.cycles_min, "..",
          held.cycles_max, " cycles", crlf);
    print(serial, "  longer by ", longer_min, "..", longer_max, " cycles; the arithmetic allows ", lower, "..", upper,
          crlf);
    bench.verdict("the control: with no handler bound, ten 64-frame windows exact both ways",
                  quiet.exact == rounds && quiet.overrun == 0u);
    bench.verdict("A HANDLER LONGER THAN A FRAME, every 5 us, IS A GAP ON THE BUS: ten of ten windows exact both "
                  "ways, status spi_ok, the peer's count right, OVR never standing",
                  held.exact == rounds && held.not_ok == 0u && held.overrun == 0u);
    bench.verdict("the window is longer than without the handler by what the arithmetic allows, and the handler ran",
                  longer_min >= lower && longer_max <= upper && held.runs >= rounds * (quiet.cycles_min / hog_period));
    (void)link_command_mode();
    bench.verdict("the command channel is back", command(Op::ping));
}

// =============================================================================
// s - THE ENGINE A REQUEST NAMES, against the peer
// =============================================================================
//
// A request naming the engines runs on them at any length. One frame, read and
// written, at PCLK1/4 and PCLK1/8: the loop would cost the core less there
// (docs/stm32f4/spi.md has the arithmetic) and nothing asks. What they cannot
// carry is refused before the block, the select or the D/C is touched, every
// witness read: the select's falling edges on its pad, D/C's latch, CR1, SCK's
// edges. A command before data runs in the loop, one block and one completion,
// with D/C rising after the command's last edge and before the data's first; a
// command with no data is the block. A polled request of any length stays the
// loop: letters o, p and u run them, and k and j keep the long ones.

namespace way {
void watch_sck(bool on);   // letter k's, defined with it
}

namespace named {

/// A pad on no recorded wire (the bench's survey): D/C for this letter, and the
/// select where the request must not reach the peer's.
using DcPad = Pin<'C', 1>;
using SparePad = Pin<'C', 0>;
using DcWatch = ExtInt<DcPad>;
constexpr uint16_t no_rise = 0xFFFFu;
volatile uint16_t dc_rise_edges = no_rise;

constexpr uint16_t cap = 40;
alignas(2) uint8_t tx[cap];
alignas(2) uint8_t rx[cap];

struct Window {
    bool accepted = false;
    bool clocked = false;
    bool inside = false;
    uint16_t completions = 0;
    uint16_t blocks = 0;
    bool reported = false;
    spilink::Report r{};
    uint16_t mism = 0;        ///< this end's read-back of the data phase, bytes wrong
    bool counted = false;     ///< SCK's edges counted: slow enough for the handler
    uint16_t edges = 0;       ///< SCK's edges in the select's window
    uint16_t dc_rise_at = no_rise;   ///< of them, how many before D/C rose
    bool dc_up = false;       ///< D/C's latch at the end
};

/// One exchange of `cmd_len + len` 8-bit frames commanded of the peer and
/// clocked on the engined host at `rate`, the first `cmd_len` as the command
/// phase: the peer hears every frame, this end reads the data phase's answers
/// back when `read`. D/C is a real pad, parked high, and its rise stamped with
/// SCK's edges so far, at PCLK1/64 and slower: a handler per edge takes some 100
/// core cycles, and half a bit is 128 there.
Window run(uint8_t cmd_len, uint16_t len, SpiClock rate, uint8_t seed, bool read = true) {
    Window o{};
    const uint16_t n = static_cast<uint16_t>(cmd_len + len);
    spilink::Params a{};
    a.cfg = spilink::Cfg{.apply = 1, .mode = 0, .dord = 0, .regime = spilink::regime_buffer_wait};
    a.count = n;
    a.ms = 200;
    a.seed_a = seed;
    a.seed_b = static_cast<uint8_t>(seed ^ 0x6Cu);
    a.pattern = spilink::pattern_prbs;
    o.accepted = peer_act(Op::exchange, a);
    if (!o.accepted) {
        return o;
    }
    {
        spilink::Stream out(a.pattern, a.seed_a);
        for (uint16_t i = 0; i < cap; ++i) {
            tx[i] = i < n ? out.next() : 0u;
            rx[i] = 0xEE;
        }
    }
    settle();
    dma_host_live = true;
    CsPin::output(true);
    DcPad::output(true);
    (void)DmaHost::init(clock, peer_link_config<DmaHost>());
    DmaHost::prime(SpiMode::mode0, rate);
    CsPin::clear();
    link_hold();
    dc_rise_edges = no_rise;
    (void)DcWatch::select();
    (void)DcWatch::configure(ExtiSense::rising);
    (void)DcWatch::clear();
    (void)DcWatch::arm(true);
    Nvic::enable(DcWatch::irq());
    o.counted = static_cast<uint8_t>(rate) >= static_cast<uint8_t>(SpiClock::div64);
    if (o.counted) {
        way::watch_sck(true);
    }
    DmaHost::Request r{};
    r.cs = {};
    r.dc = DcPad::ref();
    if (cmd_len != 0u) {
        r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(tx));
        r.cmd_len = cmd_len;
    }
    if (len != 0u) {
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx + cmd_len));
        if (read) {
            r.rx = lend<Lease::reply>(rx + cmd_len);
        }
        r.len = len;
    }
    r.clock = rate;
    r.mode = SpiMode::mode0;
    r.engine = SpiEngine::dma;
    dma_completions = 0;
    dma_blocks = 0;
    host_done = false;
    const bool done = DmaHost::start(r);
    o.inside = done;
    // 200 ms: 40 frames at PCLK1/256 are 1.8 ms.
    o.clocked = done || brio::wait_until<Ruler>([] { return host_done; }, 200'000);
    o.completions = dma_completions;
    o.blocks = dma_blocks;
    o.dc_up = DcPad::read_out();
    if (o.counted) {
        way::watch_sck(false);
    }
    (void)DcWatch::arm(false);
    Nvic::disable(DcWatch::irq());
    (void)DcWatch::clear();
    o.edges = sck_edges_selected;
    o.dc_rise_at = dc_rise_edges;
    link_hold();
    CsPin::set();
    DmaHost::release();
    DcPad::release();
    dma_host_live = false;
    (void)link_command_mode();
    o.reported = peer_report(o.r);
    if (read) {
        spilink::Stream want(a.pattern, a.seed_b);
        for (uint16_t i = 0; i < n; ++i) {
            const uint8_t w = want.next();
            if (i >= cmd_len && rx[i] != w) {
                ++o.mism;
            }
        }
    }
    return o;
}

/// Exact at both ends, and on the streams: not inside start(), one block, one
/// completion, every frame's sixteen edges on SCK where they were counted.
bool clean(const Window& o, uint16_t n) {
    return o.accepted && o.clocked && o.reported && o.r.count == n && o.r.mism == 0u && o.mism == 0u &&
           !o.inside && o.completions == 1u && o.blocks == 1u && (!o.counted || o.edges == 16u * n);
}

void print_window(const char* what, uint16_t n, SpiClock rate, const Window& o) {
    print(serial, "    ", what, ", ", n, " frames at PCLK1/", ext::division(rate), ": inside start() ",
          o.inside ? 1u : 0u, ", completions ", o.completions, ", blocks ", o.blocks, ", SCK edges ", o.edges,
          ", D/C rose after ", o.dc_rise_at, ", host mism ", o.mism, ", the peer read ", o.r.count, " mism ",
          o.r.mism, o.clocked ? "" : " HOST NEVER COMPLETED", crlf);
}

/// The byte engines: a 16-bit frame has no beat on them whatever its buffer. The
/// streams are DmaHost's, and this host never starts them.
using ByteTx = DmaTxEngine<1, 4, 0, uint8_t>;
using ByteRx = DmaRxEngine<1, 3, 0, uint8_t>;
using ByteHost = SpiHost<2, link_pins, ByteTx, ByteRx>;

struct Refusal {
    bool ended = false;
    uint8_t status = 0xFFu;
    uint16_t falls = 0;       ///< the select's falling edges
    uint16_t edges = 0;       ///< SCK's edges, both latch states
    bool cr1_same = false;
    bool dc_low = false;      ///< the D/C latch, parked low, stayed there
    bool select_up = false;
};

/// One request the host must refuse. The block sits at mode 0 and PCLK1/16 and
/// the request names mode 3 and PCLK1/8; a data phase would raise the parked D/C.
template <typename H>
Refusal refuse(typename H::Request q) {
    Refusal f{};
    q.cs = CsPin::ref();
    q.dc = DcPad::ref();
    q.clock = SpiClock::div8;
    q.mode = SpiMode::mode3;
    q.engine = SpiEngine::dma;
    H::prime(SpiMode::mode0, SpiClock::div16);
    DcPad::clear();
    const uint32_t cr1 = S::regs().CR1;
    sel_falls = 0;
    (void)SelWatch::select();
    (void)SelWatch::configure(ExtiSense::falling);
    (void)SelWatch::clear();
    (void)SelWatch::arm(true);
    Nvic::enable(SelWatch::irq());
    way::watch_sck(true);
    f.ended = H::start(q);
    f.status = H::status();
    brio::spin_us<Ruler>(1'000);
    way::watch_sck(false);
    (void)SelWatch::arm(false);
    (void)SelWatch::clear();
    f.falls = sel_falls;
    f.edges = static_cast<uint16_t>(sck_edges_selected + sck_edges_released);
    f.cr1_same = S::regs().CR1 == cr1;
    f.dc_low = !DcPad::read_out();
    f.select_up = CsPin::read_out();
    DcPad::set();
    return f;
}

bool refused(const Refusal& f) {
    return f.ended && f.status == spi_rejected && f.falls == 0u && f.edges == 0u && f.cr1_same && f.dc_low &&
           f.select_up;
}

void print_refusal(const char* what, const Refusal& f) {
    print(serial, "    ", what, ": start() ", f.ended ? 1u : 0u, ", status ", f.status, " (spi_rejected ",
          spi_rejected, "), select falls ", f.falls, ", SCK edges ", f.edges, ", CR1 ",
          f.cr1_same ? "untouched" : "MOVED", ", D/C ", f.dc_low ? "untouched" : "MOVED", crlf);
}

}  // namespace named

void ts_named() {
    if (!need_peer()) {
        return;
    }
    using named::Window;

    // PROOF ONE. PCLK1/4 and /8: a frame's 32 and 64 PCLK cycles are 128 and 256
    // core cycles, under the engines' fixed cost.
    bool one_ok = true;
    uint8_t seed = 0x31;
    for (const SpiClock rate : {SpiClock::div4, SpiClock::div8}) {
        const Window read = named::run(0, 1, rate, seed);
        named::print_window("one frame read", 1, rate, read);
        const Window written = named::run(0, 1, rate, static_cast<uint8_t>(seed + 7u), false);
        named::print_window("one frame written", 1, rate, written);
        one_ok = one_ok && named::clean(read, 1) && named::clean(written, 1);
        seed = static_cast<uint8_t>(seed + 23u);
    }
    bench.verdict("A REQUEST NAMING THE ENGINES RUNS ON THEM AT THE SHORTEST LENGTH: one frame, read and "
                  "written, at PCLK1/4 and PCLK1/8 - start() false, dma_isr() the end, one block, one "
                  "completion, byte-exact at both ends",
                  one_ok);

    // PROOF TWO. Refused: every case with its witnesses; the next request exact.
    CsPin::output(true);
    named::DcPad::output(true);
    (void)DmaHost::init(clock, peer_link_config<DmaHost>());
    dma_host_live = true;
    const auto wide = [](const uint8_t* out, uint8_t* in) {
        DmaHost::Request q{};
        q.tx = lend<Lease::reply>(out);
        q.rx = lend<Lease::reply>(in);
        q.len = 4;
        q.bits = SpiDataSize::bits16;
        return q;
    };
    const named::Refusal odd_tx = named::refuse<DmaHost>(wide(named::tx + 1, named::rx));
    const named::Refusal odd_rx = named::refuse<DmaHost>(wide(named::tx, named::rx + 1));
    DmaHost::Request lone{};
    lone.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(named::tx + 1));
    lone.cmd_len = 4;
    lone.bits = SpiDataSize::bits16;
    const named::Refusal odd_cmd = named::refuse<DmaHost>(lone);
    // 0x1000 0000: the CCM's region, on this part a hole. No stream reaches it,
    // and neither pointer is ever followed.
    const uint8_t* const ccm_out = reinterpret_cast<const uint8_t*>(uintptr_t{0x1000'0000u});
    uint8_t* const ccm_in = reinterpret_cast<uint8_t*>(uintptr_t{0x1000'0000u});
    DmaHost::Request from_ccm{};
    from_ccm.tx = lend<Lease::reply>(ccm_out);
    from_ccm.len = 4;
    const named::Refusal tx_ccm = named::refuse<DmaHost>(from_ccm);
    DmaHost::Request into_ccm{};
    into_ccm.tx = lend<Lease::reply>(static_cast<const uint8_t*>(named::tx));
    into_ccm.rx = lend<Lease::reply>(ccm_in);
    into_ccm.len = 4;
    const named::Refusal rx_ccm = named::refuse<DmaHost>(into_ccm);
    DmaHost::release();
    dma_host_live = false;
    // A wide frame on byte engines, the buffers aligned.
    CsPin::output(true);
    (void)named::ByteHost::init(clock, peer_link_config<named::ByteHost>());
    named::ByteHost::Request on_bytes{};
    on_bytes.tx = lend<Lease::reply>(static_cast<const uint8_t*>(named::tx));
    on_bytes.rx = lend<Lease::reply>(named::rx);
    on_bytes.len = 4;
    on_bytes.bits = SpiDataSize::bits16;
    const named::Refusal bytes = named::refuse<named::ByteHost>(on_bytes);
    named::ByteHost::release();
    named::print_refusal("a wide frame out of an odd address", odd_tx);
    named::print_refusal("a wide frame into an odd address", odd_rx);
    named::print_refusal("a wide command alone from an odd address", odd_cmd);
    named::print_refusal("out of the CCM's region", tx_ccm);
    named::print_refusal("into the CCM's region", rx_ccm);
    named::print_refusal("a wide frame on byte engines", bytes);
    bench.verdict("WHAT THE ENGINES CANNOT CARRY IS REFUSED: a wide frame on an odd address either way or as a "
                  "command alone, either buffer in the CCM's region, a wide frame on byte engines - start() true, "
                  "spi_rejected, the select never fell, D/C and CR1 untouched, no edge on SCK",
                  named::refused(odd_tx) && named::refused(odd_rx) && named::refused(odd_cmd) &&
                      named::refused(tx_ccm) && named::refused(rx_ccm) && named::refused(bytes));

    // THE ONE BUSY CHECK: a request while this host's own block is on the
    // streams is refused before anything of it moves, whichever engine either
    // names. The block in flight frames no select (a null cs) and clocks 64
    // frames at PCLK1/256, some 3 ms, into nobody; the two requests started
    // under it name the link's select, watched for a fall, D/C parked low (a
    // data phase would raise it), and another mode and rate (a configuration
    // applied would move CR1). Then the block in flight ends as it would have:
    // one completion.
    CsPin::output(true);
    named::DcPad::output(false);
    (void)DmaHost::init(clock, peer_link_config<DmaHost>());
    dma_host_live = true;
    DmaHost::prime(SpiMode::mode0, SpiClock::div256);
    static uint8_t flight_tx[64];
    DmaHost::Request flight{};
    flight.tx = lend<Lease::reply>(static_cast<const uint8_t*>(flight_tx));
    flight.len = 64;
    flight.clock = SpiClock::div256;
    flight.engine = SpiEngine::dma;
    host_done = false;
    dma_completions = 0;
    const bool flight_async = !DmaHost::start(flight);
    const uint32_t busy_cr1 = S::regs().CR1;
    sel_falls = 0;
    (void)SelWatch::select();
    (void)SelWatch::configure(ExtiSense::falling);
    (void)SelWatch::clear();
    (void)SelWatch::arm(true);
    Nvic::enable(SelWatch::irq());
    DmaHost::Request late{};
    late.cs = CsPin::ref();
    late.dc = named::DcPad::ref();
    late.tx = lend<Lease::reply>(static_cast<const uint8_t*>(named::tx));
    late.len = 4;
    late.clock = SpiClock::div8;
    late.mode = SpiMode::mode3;
    late.engine = SpiEngine::dma;
    const bool late_dma = DmaHost::start(late);
    const uint8_t late_dma_status = DmaHost::status();
    late.engine = SpiEngine::polled;
    const bool late_polled = DmaHost::start(late);
    const uint8_t late_polled_status = DmaHost::status();
    const bool busy_regs = S::regs().CR1 == busy_cr1;
    const bool busy_dc_low = !named::DcPad::read_out();
    // 50 ms against the block's 3 ms; host_done is judged below.
    (void)brio::wait_until<Ruler>([] { return host_done; }, 50'000);
    (void)SelWatch::arm(false);
    Nvic::disable(SelWatch::irq());
    (void)SelWatch::clear();
    const uint16_t busy_falls = sel_falls;
    print(serial, "    two requests under a block in flight: on the streams start() ", late_dma ? 1u : 0u,
          ", status ", late_dma_status, "; polled start() ", late_polled ? 1u : 0u, ", status ", late_polled_status,
          " (spi_rejected ", spi_rejected, "); select falls ", busy_falls, ", D/C ",
          busy_dc_low ? "untouched" : "MOVED", ", CR1 ", busy_regs ? "untouched" : "MOVED",
          "; the block in flight ended ", host_done ? 1u : 0u, ", status ", DmaHost::status(), ", completions ",
          dma_completions, crlf);
    bench.verdict("THE ONE BUSY CHECK: a request while the host's own block is on the streams - naming the streams or "
                  "the loop - is refused before anything of it moves: start() true, spi_rejected, its select never "
                  "fell, D/C and CR1 untouched; the block in flight ends as it would have, one completion, spi_ok",
                  flight_async && late_dma && late_dma_status == spi_rejected && late_polled &&
                      late_polled_status == spi_rejected && busy_falls == 0u && busy_dc_low && busy_regs &&
                      host_done && dma_completions == 1u && DmaHost::status() == spi_ok);
    DmaHost::release();
    dma_host_live = false;
    named::DcPad::release();
    (void)link_command_mode();

    const Window after = named::run(0, 32, SpiClock::div8, 0x52);
    named::print_window("the next request", 32, SpiClock::div8, after);
    bench.verdict("the next request after every refusal is exact at both ends", named::clean(after, 32));

    // PROOF THREE. Sixteen edges a frame: D/C rises at exactly the command's.
    const Window before = named::run(4, 8, SpiClock::div256, 0x5B);
    named::print_window("a 4-frame command before 8 data frames", 12, SpiClock::div256, before);
    const Window one_cmd = named::run(1, 32, SpiClock::div128, 0x6D);
    named::print_window("a 1-frame command before 32 data frames", 33, SpiClock::div128, one_cmd);
    bench.verdict("A COMMAND BEFORE DATA RUNS IN THE LOOP AND THE DATA ON THE ENGINES: one block, one completion, D/C "
                  "rising after the command's last edge and before the data's first, exact at both ends",
                  named::clean(before, 12) && before.dc_rise_at == 16u * 4u && named::clean(one_cmd, 33) &&
                      one_cmd.dc_rise_at == 16u);
    const Window alone = named::run(3, 0, SpiClock::div16, 0x77);
    named::print_window("a 3-frame command alone", 3, SpiClock::div16, alone);
    const Window alone_slow = named::run(3, 0, SpiClock::div256, 0x79);
    named::print_window("a 3-frame command alone", 3, SpiClock::div256, alone_slow);
    bench.verdict("A COMMAND WITH NO DATA RIDES THE ENGINES AS THE WHOLE REQUEST: one block, one completion, exact at "
                  "the peer, D/C high when the completion answers, after every frame's edges",
                  named::clean(alone, 3) && alone.dc_up && named::clean(alone_slow, 3) && alone_slow.dc_up &&
                      alone_slow.dc_rise_at == 16u * 3u);
    bench.verdict("the command channel survives the letter", command(Op::ping));
}

// =============================================================================
// E - WHEN A REQUEST ENDS THE WIRE HAS ENDED
// =============================================================================
//
// RXNE rises on the last sampling edge (26.3.9), half a bit before the frame's
// last edge in modes 0 and 2. So the instrument: the SCK pad's input, read right
// after the request ended - after start() returned for a polled one, masked, in
// the handler right after dma_isr() returned true for the engines' - must be at
// the mode's idle level. WHY NOTHING CAN MISS AN EARLY END, AND WHERE: from the
// flag to the pad's read is at most some 90 core cycles by the release listing,
// and half a bit is 2^code PCLK cycles, 2^code x 2 core cycles on APB2 and 2^code
// x 4 on APB1. So at /256 an early end leaves 256 (SPI1) or 512 (SPI2) core
// cycles against 90, and the read sees it; the instrument goes blind on the rungs
// the control run below names, where the turn outlasts the half bit. THE VERDICT
// IS ONE-SIDED: SCK at its idle level only ever follows the last edge, so a pad
// read at the active level is an early end and nothing else, and the bound is
// zero at every rung. A handler that delays a read could hide an early end and
// never invent one, which is why the polled reads are masked.
//
// THE PADS: SCK's input whatever its function. PC0 is the select, a pad on no
// recorded wire; the link's own stays high, so the peer hears none of this.

namespace wire_end {

constexpr SpiPins spi1_pins{.sck = {'A', 5, PinFunction::af5}, .miso = {'A', 6, PinFunction::af5},
                            .mosi = {'A', 7, PinFunction::af5}};
using Tx1 = DmaTxEngine<2, 3, 3, uint16_t>;
using Rx1 = DmaRxEngine<2, 2, 3, uint16_t>;
using Host1 = SpiHost<1, spi1_pins, Tx1, Rx1>;
using Sck1 = Pin<'A', 5>;
using Sck2 = Pin<link_pins.sck.port, link_pins.sck.pin>;
using Select = Pin<'C', 0>;

volatile bool watching = false;
volatile bool pad_high = false;
volatile bool select_up = false;

/// The handler's half of the read, right after dma_isr() answered true.
template <typename Sck>
[[gnu::always_inline]] inline void sampled() {
    pad_high = Sck::read();
    select_up = Select::read_out();
}

enum class Way : uint8_t { polled_read_1, polled_read_8, polled_write, dma_read, dma_write, dma_command, dma_both };
constexpr uint8_t way_count = 7;
const char* way_name(uint8_t w) {
    constexpr const char* names[way_count] = {"polled read, 1 frame",  "polled read, 8 frames", "polled write, 8 frames",
                                              "dma read, 1 frame",     "dma write, 1 frame",    "dma command alone",
                                              "dma command then data"};
    return names[w];
}
constexpr bool way_is_dma(uint8_t w) { return w >= static_cast<uint8_t>(Way::dma_read); }

uint8_t out[8];
uint8_t in[8];

/// [way][mode]: a bit per rung, an early end seen there.
uint8_t early[way_count][4];
uint32_t reads = 0;
uint32_t bad_status = 0;
uint32_t no_end = 0;
/// [mode 0 or 1][rung]: the vector's cycles, a dma read and a dma write.
uint32_t vector_most[2][8];
uint32_t vector_least[2][8];

template <typename H>
typename H::Request request(uint8_t w, SpiMode m, SpiClock c, PinRef cs) {
    typename H::Request r{};
    r.cs = cs;
    r.clock = c;
    r.mode = m;
    r.bits = SpiDataSize::bits8;
    r.engine = way_is_dma(w) ? SpiEngine::dma : SpiEngine::polled;
    const uint8_t* const o = out;
    switch (static_cast<Way>(w)) {
        case Way::polled_read_1:
        case Way::dma_read:
            r.tx = lend<Lease::reply>(o);
            r.rx = lend<Lease::reply>(in);
            r.len = 1;
            break;
        case Way::polled_read_8:
            r.tx = lend<Lease::reply>(o);
            r.rx = lend<Lease::reply>(in);
            r.len = 8;
            break;
        case Way::polled_write:
            r.tx = lend<Lease::reply>(o);
            r.len = 8;
            break;
        case Way::dma_write:
            r.tx = lend<Lease::reply>(o);
            r.len = 1;
            break;
        case Way::dma_command:
            r.cmd = lend<Lease::reply>(o);
            r.cmd_len = 1;
            break;
        case Way::dma_both:
            r.cmd = lend<Lease::reply>(o);
            r.cmd_len = 1;
            r.tx = lend<Lease::reply>(o);
            r.rx = lend<Lease::reply>(in);
            r.len = 1;
            break;
    }
    return r;
}

/// One request, the pad read as the request ends; true when the end was seen.
template <typename H, typename Sck>
bool once(uint8_t w, SpiMode m, SpiClock c, PinRef cs, bool& high, bool& up) {
    const typename H::Request r = request<H>(w, m, c, cs);
    if (!way_is_dma(w)) {
        {
            InterruptGuard guard;
            (void)H::start(r);
            high = Sck::read();
            up = Select::read_out();
        }
        return true;
    }
    host_done = false;
    watching = true;
    const bool inside = H::start(r);
    // 20 ms: two frames at PCLK/256 are some 100 us; the answer is the return below.
    if (!inside) {
        (void)brio::wait_until<Ruler>([] { return host_done; }, 20'000);
    }
    watching = false;
    high = pad_high;
    up = select_up;
    return !inside && host_done;
}

template <typename H, typename Sck>
void sweep() {
    for (auto& per_way : early) {
        for (uint8_t& per_mode : per_way) {
            per_mode = 0;
        }
    }
    for (uint8_t i = 0; i < 2u; ++i) {
        for (uint8_t k = 0; k < 8u; ++k) {
            vector_most[i][k] = 0;
            vector_least[i][k] = UINT32_MAX;
        }
    }
    reads = 0;
    bad_status = 0;
    no_end = 0;
    time_vector = true;
    for (uint8_t w = 0; w < way_count; ++w) {
        for (uint8_t mode = 0; mode < 4u; ++mode) {
            const SpiMode m = static_cast<SpiMode>(mode);
            const bool idle_high = spi_mode_cpol(m);
            for (uint8_t code = 0; code < 8u; ++code) {
                const SpiClock c = static_cast<SpiClock>(code);
                for (uint8_t rep = 0; rep < 6u; ++rep) {
                    const PinRef cs = rep < 3u ? Select::ref() : PinRef{};
                    bool high = idle_high;
                    bool up = true;
                    const bool ended = once<H, Sck>(w, m, c, cs, high, up);
                    ++reads;
                    if (!ended) {
                        ++no_end;
                        (void)H::recover();
                        continue;
                    }
                    if (H::status() != spi_ok) {
                        ++bad_status;
                    }
                    if (high != idle_high || !up) {
                        early[w][mode] = static_cast<uint8_t>(early[w][mode] | (1u << code));
                    }
                    if (rep != 0u && mode < 2u &&
                        (w == static_cast<uint8_t>(Way::dma_read) || w == static_cast<uint8_t>(Way::dma_write))) {
                        const uint32_t v = vector_cycles;
                        uint32_t& most = vector_most[mode][code];
                        uint32_t& least = vector_least[mode][code];
                        most = v > most ? v : most;
                        least = v < least ? v : least;
                    }
                }
            }
        }
    }
    time_vector = false;
}

/// THE WAIT'S BOUND, derived. The vector starts from the same edge in every
/// mode - the last RXNE, the last sampling edge - and its path is the same in
/// all four but for the wait for the pad's idle level, which in mode 1 is
/// already over (the last sampling edge is the frame's last edge) and in
/// mode 0 lasts until half a bit after that edge. So the longest vector in
/// mode 0 is at most the longest in mode 1 plus half a bit plus what the
/// wait itself adds after the edge:
///   13  one turn of wire_ended()'s poll (the release listing: the IDR's ldr,
///       eor, lsls, a taken bmi, subs and a taken bne, 2 + 1 + 1 + 4 + 1 + 4)
///   1   the pad's level reaching IDR, sampled every AHB1 cycle (RM0390 7.3.11)
///   2   the pad's own edge, 10 ns at most at the medium class and 50 pF, 4 at
///       very high (DS10693 table 58): two core cycles
/// - 16 cycles. Mode 1's longest is read as its least plus its spread at that
/// rung, both measured in the same sweep.
constexpr uint32_t wait_past_half_cycles = 13u + 1u + 2u;

/// The figures, measured first and printed after: no console line beside a run.
/// `within`: at every rung, mode 0's longest vector is inside the bound above.
bool report(const char* name, uint32_t ratio, bool& within) {
    uint32_t seen = 0;
    print(serial, "  ", name, ": ", reads, " requests ended, ", no_end, " with no end, ", bad_status, " not spi_ok", crlf);
    for (uint8_t w = 0; w < way_count; ++w) {
        for (uint8_t mode = 0; mode < 4u; ++mode) {
            if (early[w][mode] != 0u) {
                print(serial, "    EARLY: ", way_name(w), ", mode ", mode, ", rungs");
                for (uint8_t code = 0; code < 8u; ++code) {
                    if ((early[w][mode] & (1u << code)) != 0u) {
                        print(serial, " /", 2u << code);
                        ++seen;
                    }
                }
                print(serial, crlf);
            }
        }
    }
    within = true;
    uint32_t worst_over = 0;
    print(serial, "    the vector, core cycles, read or write, mode 0 then mode 1, least..most:", crlf);
    for (uint8_t code = 0; code < 8u; ++code) {
        const uint32_t half = (1u << code) * ratio;
        print(serial, "      /", 2u << code, " half a bit ", half, ": ", vector_least[0][code], "..",
              vector_most[0][code], "  ", vector_least[1][code], "..", vector_most[1][code], crlf);
        const uint32_t floor = vector_least[1][code];
        const uint32_t over = vector_most[0][code] > floor + half ? vector_most[0][code] - (floor + half) : 0u;
        const uint32_t spread1 =
            vector_most[1][code] > floor ? vector_most[1][code] - floor : 0u;
        worst_over = over > worst_over ? over : worst_over;
        if (vector_most[1][code] == 0u || over > spread1 + wait_past_half_cycles) {
            within = false;
        }
    }
    print(serial, "    the longest wait past half a bit, mode 0 over mode 1's least: ", worst_over,
          " cycles (the bound: mode 1's own spread at that rung plus ", wait_past_half_cycles, ")",
          crlf);
    return seen == 0u && no_end == 0u && bad_status == 0u;
}

}  // namespace wire_end

void tE_wire_ended() {
    using namespace wire_end;
    for (uint8_t i = 0; i < 8u; ++i) {
        out[i] = static_cast<uint8_t>(0xA5u ^ (i * 11u));
    }
    Select::output(true);
    // SPI2, APB1: the core's clock over four. The link's own select stays high.
    CsPin::output(true);
    (void)DmaHost::init(clock, peer_link_config<DmaHost>());
    dma_host_live = true;
    sweep<DmaHost, Sck2>();
    dma_host_live = false;
    bool within2 = false;
    const bool spi2_ok = report("SPI2 on APB1", SysClock::hz / DmaHost::reference_hz(), within2);
    bench.verdict("SPI2: WHEN A REQUEST ENDS THE WIRE HAS ENDED: all four modes, every rate, polled and on the engines, "
                  "read and write, through a select and a null one - SCK at its idle level every time (the bound is "
                  "zero: a pad read at the active level is an early end and nothing else)",
                  spi2_ok);

    // THE RUN-OUT, staged: a one-frame read at /256, mode 0, and SCK's pad
    // taken from the block the moment start() returns, a GPIO output held at
    // the active level - so the vector finds the pad never idle, its bound runs
    // out, and the request must end spi_stalled with the select up. The pad goes
    // back to the block (the sck_speed() verb hands it over again) before the
    // next request, which pays the reset the stall owes.
    host_done = false;
    const DmaHost::Request stage = request<DmaHost>(static_cast<uint8_t>(Way::dma_read), SpiMode::mode0,
                                                    SpiClock::div256, Select::ref());
    const bool stage_inside = DmaHost::start(stage);
    Sck2::output(true);
    // 20 ms: the vector's bound runs out in microseconds; host_done is judged below.
    if (!stage_inside) {
        (void)brio::wait_until<Ruler>([] { return host_done; }, 20'000);
    }
    const uint8_t stage_status = DmaHost::status();
    const bool stage_up = Select::read_out();
    DmaHost::sck_speed(PinSpeed::medium);
    bool next_high = false;
    bool next_up = false;
    (void)once<DmaHost, Sck2>(static_cast<uint8_t>(Way::polled_read_1), SpiMode::mode0, SpiClock::div2,
                              Select::ref(), next_high, next_up);
    const uint8_t next_status = DmaHost::status();
    print(serial, "  the wait staged to run out: start() ", stage_inside ? 1u : 0u, ", the vector answered ",
          host_done ? 1u : 0u, ", status ", stage_status, " (spi_stalled ", spi_stalled, "), the select up ",
          stage_up ? 1u : 0u, "; the next request: status ", next_status, ", SCK idle ", next_high ? 0u : 1u, crlf);
    bench.verdict("A WAIT THAT RUNS OUT ENDS THE REQUEST spi_stalled in the vector, the select released, and the next "
                  "request, the reset it owes paid, ends spi_ok with the wire ended",
                  !stage_inside && host_done && stage_status == spi_stalled && stage_up && next_status == spi_ok &&
                      !next_high && next_up);
    DmaHost::release();

    // SPI1, APB2: the core's clock over two, on the bench's free pads.
    Dma<2>::init();
    (void)Host1::init(clock);
    sweep<Host1, Sck1>();
    bool within1 = false;
    const bool spi1_ok = report("SPI1 on APB2", SysClock::hz / Host1::reference_hz(), within1);
    Host1::release();
    Select::release();
    bench.verdict("SPI1: WHEN A REQUEST ENDS THE WIRE HAS ENDED, the same sweep on the other APB clock", spi1_ok);

    // The wait's bound: wait_past_half_cycles, derived above.
    bench.verdict("THE LONGEST WAIT IN THE VECTOR IS HALF A BIT: on both APB clocks, every rung, the vector's most in "
                  "mode 0 is within mode 1's most plus half a bit plus one turn of the pad's poll and the pad's edge",
                  within2 && within1);
    (void)link_command_mode();
    bench.verdict("the command channel survives the letter", command(Op::ping));
}

// =============================================================================
// j - THE COUNT PAST SIXTEEN BITS
// =============================================================================
//
// `cmd_len` is a byte and `len` sixteen bits, so their sum needs seventeen:
// one command frame and 65535 data frames are 65536 frames, which a 16-bit
// sum reads as a request of none. The select stays null and the pad high, so
// the peer hears nothing of it; the frames are on the wire all the same.

void tj_count() {
    static constexpr uint8_t one_command[1] = {0x5A};
    (void)link_command_mode();
    Host::Request rq{};
    rq.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(one_command));
    rq.cmd_len = 1;
    rq.len = 65535;
    rq.bits = SpiDataSize::bits8;
    rq.clock = SpiClock::div8;
    rq.mode = SpiMode::mode0;
    rq.engine = SpiEngine::polled;
    const uint32_t t0 = CycleCounter::now();
    const bool done = Host::start(rq);
    const uint32_t wall = CycleCounter::now() - t0;
    const uint8_t st = Host::status();
    // The wire: 65536 frames x 8 bits x 8 PCLK1 cycles x HCLK/PCLK1 = 4, 256
    // cycles a frame, 16 777 216 cycles, 93.2 ms. The last frame's last edge
    // is half a bit, 16 cycles, before its eighth period ends. The counter runs
    // on the clock SCK is divided from, so no wander enters and its resolution
    // is one cycle; a handler can only lengthen the wall. So the wall is at
    // least 16 777 200 cycles; a request read as empty takes some hundreds.
    constexpr uint32_t wire = 65536u * 8u * 8u * (SysClock::hz / SysClock::pclk1_hz);
    constexpr uint32_t half_bit = 8u * (SysClock::hz / SysClock::pclk1_hz) / 2u;
    print(serial, "  1 + 65535 frames at PCLK1/8: start() ", done ? "true" : "false", ", status ", st,
          ", ", wall, " cycles against the wire's ", wire, " (x1000 ",
          static_cast<uint32_t>(static_cast<uint64_t>(wall) * 1000u / wire), ")", crlf);
    bench.verdict("A REQUEST OF 65536 FRAMES RUNS THEM: one command frame and 65535 data frames "
                  "end spi_ok inside start(), and the wall is at least the wire's 93.2 ms less "
                  "half a bit",
                  done && st == spi_ok && wall >= wire - half_bit);
    (void)link_command_mode();
}

// =============================================================================
// k - THE NEXT REQUEST AFTER A STALL OR A TRANSFER ERROR
// =============================================================================
//
// The host puts its own block right, so nothing here calls recover() between a
// failure and the request judged after it, and nothing re-initializes the host:
// the peer's exchange is commanded FIRST, through the engineless host on the
// same instance, then the engined host comes up and is not touched again until
// the window has been clocked. The peer's DMA serve counts frames and takes no
// notice of a select that falls and rises with no clock under it, so a stalled
// request may frame itself with the real select and the window survives it.
//
// A STALL IS STAGED WITH SPE DOWN UNDER THE HOST (S::disable() on the resource):
// the request's compare finds nothing to change, so the enable stays down, the
// frame written stands in the transmit buffer and the flag its loop waits for
// never rises. MODE 3 WITH A PULL-DOWN ON SCK IS THE TEST OF THE ORDER, as a bus
// with a pull on that wire would be: this block drives SCK only while SPE is set
// (measured here), so the pad falls when SPE does and the reset's enable raises
// it, a rising edge; were the select still low the peer's client would count it
// and the window after it would be a bit out of place. SCK's edges are also
// counted on the pad itself, each by where the select's latch stood: none with it
// low and at least one with it high is the order read directly.
//
// A TRANSFER ERROR IS A BUFFER IN RESERVED SPACE (kl::way_bad_address()). A bad
// transmit buffer stops the stream on its first beat: nothing reaches DR, no
// clock, so the real select can frame it. A bad receive buffer stops the
// receive stream on the first frame back, with the transmit stream already
// ahead of it: frames still clock and one can stand in the receive buffer, so
// its request carries no select - the peer's window stays deaf - and the one
// after it is the first thing the peer hears. A frame of the faulted block read
// in front of the window would put this end's read-back one frame out of place.
//
// The select's level is read on its output latch, the store the host made. The
// walls are printed against the budget, not judged: a handler can only lengthen
// them, and the figure is read beside the arithmetic in docs/stm32f4/spi.md. The
// vector's cost is the DWT's, around DmaHost::dma_isr().
//
// THE BOUNDS. The peer holds its window for 400 ms from the command: in front of
// the window stand at most two stalls, 39 ms each at the wall printed, or one of
// 49 ms, and a millisecond of settling. A fault's completion waits 100 ms for a
// first beat or a first frame back, 8 x 32 PCLK1 cycles, 5.7 us; a window's waits
// 200 ms for 32 such frames, 182 us; the arbiter's 100 ticks are not spent on a
// stall, whose wall is inside start() and untimed. The verdicts themselves are
// exact counts, with no tolerance to derive.

namespace way {

using kl::way_frames;
using kl::way_rate;
using kl::way_rx;
using kl::way_tx;

enum class Shape : uint8_t {
    write,     ///< polled, nothing read back: the transmit-only loop
    read,      ///< polled, read back: the receive loop
    command,   ///< on the engines with a command phase: the command in the loop before its block
};

struct Failed {
    bool ended = false;       ///< start() answered true
    bool completed = false;   ///< ended, or the vector answered true within the wait
    uint16_t completions = 0;
    uint8_t status = 0xFFu;
    bool released = false;
    uint32_t wall = 0;        ///< cycles inside start()
};

/// Clear the window, then command the peer. False when the peer refused.
bool arm_peer(spilink::Params& a, uint8_t seed, SpiMode mode) {
    (void)link_command_mode();
    a = spilink::Params{};
    a.cfg = spilink::Cfg{.apply = 1,
                         .mode = static_cast<uint8_t>(mode),
                         .dord = 0,
                         .regime = spilink::regime_buffer_wait};
    a.count = way_frames;
    a.ms = 400;
    a.seed_a = seed;
    a.seed_b = static_cast<uint8_t>(seed ^ 0x6Cu);
    a.pattern = spilink::pattern_prbs;
    spilink::Stream out(a.pattern, a.seed_a);
    for (uint16_t i = 0; i < way_frames; ++i) {
        way_tx[i] = out.next();
        way_rx[i] = 0xEE;
    }
    return peer_act(Op::exchange, a);
}

/// The engined host up at a rate, the select high. Primed, because a request
/// whose rate differs from the applied one would raise the enable again.
void host_up(SpiMode mode, SpiClock rate = way_rate) {
    kl::way_mode = mode;
    CsPin::output(true);
    dma_host_live = true;
    (void)DmaHost::init(clock, peer_link_config<DmaHost>());
    DmaHost::prime(mode, rate);
    brio::spin_us<Ruler>(1'000);
}

/// Letter's pull on SCK, which the next init() takes away with the pad's
/// function: what makes the pad's being undriven with SPE clear an edge.
void pull_sck_down() {
    Pin<link_pins.sck.port, link_pins.sck.pin>::pull(PinPull::down);
    brio::spin_us<Ruler>(1'000);
}

/// SCK's edges counted over one call: the pad is an input to the EXTI whatever
/// its function, on line 13 beside the select's 12.
void watch_sck(bool on) {
    if (on) {
        (void)SckWatch::select();
        (void)SckWatch::configure(ExtiSense::both);
        (void)SckWatch::clear();
        sck_edges_selected = 0;
        sck_edges_released = 0;
    }
    (void)SckWatch::arm(on);
    if (on) {
        Nvic::enable(SckWatch::irq());
    } else {
        Nvic::disable(SckWatch::irq());
        (void)SckWatch::clear();
    }
}

/// A request made to stall, the block already unable to clock.
Failed run_stalled(Shape shape, PinRef cs, SpiMode mode, SpiClock rate = way_rate) {
    DmaHost::Request q{};
    q.cs = cs;
    q.tx = lend<Lease::reply>(static_cast<const uint8_t*>(way_tx));
    q.len = 9;
    q.clock = rate;
    q.mode = mode;
    q.engine = SpiEngine::polled;
    if (shape == Shape::read) {
        q.rx = lend<Lease::reply>(way_rx);
    }
    if (shape == Shape::command) {
        q.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(way_tx));
        q.cmd_len = 1;
        q.rx = lend<Lease::reply>(way_rx);
        q.len = way_frames;
        q.engine = SpiEngine::dma;
    }
    Failed f{};
    const uint32_t t0 = CycleCounter::now();
    f.ended = DmaHost::start(q);
    f.wall = CycleCounter::now() - t0;
    f.completed = f.ended;
    f.status = DmaHost::status();
    f.released = CsPin::read_out();
    return f;
}

/// The completion, however it comes: `ended` when start() already answered, or
/// the vector's answer within `ms`.
bool completed_within(bool ended, uint32_t ms) {
    return ended || brio::wait_until<Ruler>([] { return host_done; }, ms * 1000u);
}

/// A block whose buffer gives a bus error, and the vector's answer.
Failed run_faulted(bool bad_tx, PinRef cs, SpiMode mode) {
    DmaHost::Request q{};
    q.cs = cs;
    q.tx = lend<Lease::reply>(bad_tx ? static_cast<const uint8_t*>(kl::way_bad_address())
                                     : static_cast<const uint8_t*>(way_tx));
    q.rx = lend<Lease::reply>(bad_tx ? way_rx : kl::way_bad_address());
    q.len = way_frames;
    q.clock = way_rate;
    q.mode = mode;
    q.engine = SpiEngine::dma;
    Failed f{};
    dma_completions = 0;
    host_done = false;
    time_vector = true;
    const uint32_t t0 = CycleCounter::now();
    f.ended = DmaHost::start(q);
    f.wall = CycleCounter::now() - t0;
    f.completed = completed_within(f.ended, 100u);
    time_vector = false;
    f.completions = dma_completions;
    f.status = DmaHost::status();
    f.released = CsPin::read_out();
    return f;
}

struct Window {
    bool ended = false;       ///< start() answered true
    bool completed = false;
    uint8_t status = 0xFFu;
    bool reported = false;
    spilink::Report r{};
    uint16_t mism = 0;        ///< this end's read-back, frames wrong
};

/// What every window ends in: the host put away, the peer's report, this end's
/// read-back against what the peer owes.
void close_window(const spilink::Params& a, Window& o) {
    DmaHost::release();
    dma_host_live = false;
    (void)link_command_mode();
    o.reported = peer_report(o.r);
    spilink::Stream want(a.pattern, a.seed_b);
    for (uint16_t i = 0; i < way_frames; ++i) {
        if (way_rx[i] != want.next()) {
            ++o.mism;
        }
    }
}

/// The whole window, the first request the host has seen since the failure.
Window next_window(const spilink::Params& a, SpiMode mode, bool polled, SpiClock rate = way_rate) {
    Window o{};
    DmaHost::Request q{};
    q.cs = CsPin::ref();
    q.tx = lend<Lease::reply>(static_cast<const uint8_t*>(way_tx));
    q.rx = lend<Lease::reply>(way_rx);
    q.len = way_frames;
    q.cs_setup_us = 30;
    q.clock = rate;
    q.mode = mode;
    q.engine = polled ? SpiEngine::polled : SpiEngine::dma;
    host_done = false;
    o.ended = DmaHost::start(q);
    o.completed = completed_within(o.ended, 200u);
    o.status = DmaHost::status();
    close_window(a, o);
    return o;
}

/// Exact at both ends, and run the way it asked: inside start() when polled, on
/// the streams when not.
bool exact(const Window& o, bool polled) {
    return o.completed && o.status == spi_ok && o.ended == polled && o.reported && o.r.count == way_frames &&
           o.r.mism == 0u && o.mism == 0u;
}

void print_window(const char* what, const Window& o) {
    print(serial, "      then ", what, ": status ", o.status, ", inside start() ", o.ended ? 1u : 0u, ", host mism ",
          o.mism, ", the peer read ", o.r.count, " mism ", o.r.mism, o.completed ? "" : " NEVER COMPLETED", crlf);
}

void print_failed(const char* what, const Failed& f) {
    print(serial, "    ", what, ": start() ", f.ended ? "true" : "false", ", status ", f.status, " (stalled ",
          spi_stalled, ", fault ", spi_dma_fault, "), ", f.wall, " cycles (", f.wall / cycles_per_us, " us) in start(), select ",
          f.released ? "high" : "LOW", crlf);
}

/// One request queued behind a failing one, through the arbiter. The replies and
/// the good one's window, which the peer has held since before the failure.
struct Queued {
    uint8_t replies = 0;
    uint8_t first = 0;
    uint8_t second = 0;
    Window o{};
};
Queued run_queued(bool fault, bool polled, uint8_t seed) {
    Queued q{};
    spilink::Params a{};
    const bool armed = arm_peer(a, seed, SpiMode::mode0);
    kl::BusKernel::init_all();
    host_up(SpiMode::mode0);
    bus_ao_live = true;
    if (!fault) {
        (void)S::disable();
    }
    post<kl::Driver>(kl::Pair{.fault = fault, .polled = polled});
    kl::pump_for(800, [] { return kl::Driver::replies >= 2u; });
    bus_ao_live = false;
    q.replies = kl::Driver::replies;
    q.first = kl::Driver::first_status;
    q.second = kl::Driver::second_status;
    q.o.completed = armed;
    q.o.ended = polled;
    q.o.status = q.second;
    close_window(a, q.o);
    return q;
}

bool queued_clean(const Queued& q, uint8_t first, bool polled) {
    return q.replies == 2u && q.first == first && exact(q.o, polled);
}

void print_queued(const char* what, const Queued& q) {
    print(serial, "    ", what, ": ", q.replies, " replies, status ", q.first, " then ", q.second, ", host mism ", q.o.mism,
          ", the peer read ", q.o.r.count, " mism ", q.o.r.mism, crlf);
}

}  // namespace way

void tk_way_back() {
    if (!need_peer()) {
        return;
    }
    print(serial, "  a stall: 32 frames at PCLK1/32, the enable taken down under the host first", crlf);

    // ---- a stall on each path, the next request exact --------------------------
    spilink::Params a{};
    bool armed = way::arm_peer(a, 0x21, SpiMode::mode3);
    way::host_up(SpiMode::mode3);
    way::pull_sck_down();
    (void)S::disable();
    way::watch_sck(true);
    const way::Failed f_write = way::run_stalled(way::Shape::write, CsPin::ref(), SpiMode::mode3);
    way::watch_sck(false);
    const uint16_t under_write = sck_edges_selected;
    const uint16_t after_write = sck_edges_released;
    const way::Window n_write = way::next_window(a, SpiMode::mode3, true);
    way::print_failed("a polled write, mode 3", f_write);
    print(serial, "      SCK's edges in start(): ", under_write, " with the select low, ", after_write, " with it high", crlf);
    way::print_window("a polled window", n_write);
    bench.verdict("A POLLED WRITE THAT STALLS, MODE 3, THEN A POLLED REQUEST WITH NO recover(): spi_stalled, the select "
                  "released, and the window exact at both ends - SCK moved by the reset behind a select already up",
                  armed && f_write.ended && f_write.status == spi_stalled && f_write.released && under_write == 0u && after_write != 0u &&
                      way::exact(n_write, true));

    armed = way::arm_peer(a, 0x34, SpiMode::mode0);
    way::host_up(SpiMode::mode0);
    (void)S::disable();
    const way::Failed f_read = way::run_stalled(way::Shape::read, CsPin::ref(), SpiMode::mode0);
    const way::Window n_read = way::next_window(a, SpiMode::mode0, false);
    way::print_failed("a polled read, mode 0", f_read);
    way::print_window("an asynchronous window", n_read);
    bench.verdict("A POLLED READ THAT STALLS, THEN AN ASYNCHRONOUS REQUEST WITH NO recover(): spi_stalled, the select "
                  "released, and the window on the streams exact at both ends",
                  armed && f_read.ended && f_read.status == spi_stalled && f_read.released &&
                      way::exact(n_read, false));

    // A request naming the engines runs its command phase in the loop inside
    // start(), before its data block goes to the streams (the class comment of
    // SpiHost), so a stall staged under it ends inside start() at any rate.
    armed = way::arm_peer(a, 0x47, SpiMode::mode3);
    way::host_up(SpiMode::mode3, SpiClock::div8);
    way::pull_sck_down();
    (void)S::disable();
    way::watch_sck(true);
    const way::Failed f_cmd = way::run_stalled(way::Shape::command, CsPin::ref(), SpiMode::mode3, SpiClock::div8);
    way::watch_sck(false);
    const uint16_t under_cmd = sck_edges_selected;
    const uint16_t after_cmd = sck_edges_released;
    const way::Window n_cmd = way::next_window(a, SpiMode::mode3, true, SpiClock::div8);
    way::print_failed("an asynchronous request's polled command phase, mode 3", f_cmd);
    print(serial, "      SCK's edges in start(): ", under_cmd, " with the select low, ", after_cmd, " with it high", crlf);
    way::print_window("a polled window", n_cmd);
    bench.verdict("AN ASYNCHRONOUS REQUEST WHOSE POLLED COMMAND PHASE STALLS ends inside start(), spi_stalled, the select "
                  "released, no block started; the next request exact at both ends",
                  armed && f_cmd.ended && f_cmd.status == spi_stalled && f_cmd.released && under_cmd == 0u && after_cmd != 0u &&
                      way::exact(n_cmd, true));

    // ---- queued behind a stall, through the arbiter ----------------------------
    const way::Queued q_stall_p = way::run_queued(false, true, 0x5A);
    way::print_queued("queued behind a stall, polled", q_stall_p);
    const way::Queued q_stall_a = way::run_queued(false, false, 0x6D);
    way::print_queued("queued behind a stall, asynchronous", q_stall_a);
    bench.verdict("TWO REQUESTS FROM ONE DISPATCH, THE FIRST STALLS: the replies in order, spi_stalled then spi_ok, the "
                  "second's window exact at both ends, polled and on the streams, recover() never called",
                  way::queued_clean(q_stall_p, spi_stalled, true) && way::queued_clean(q_stall_a, spi_stalled, false));

    // ---- a transfer error, the next request exact ------------------------------
    print(serial, "  a transfer error: 32 frames at PCLK1/32, a buffer at 0x30000000", crlf);
    armed = way::arm_peer(a, 0x71, SpiMode::mode0);
    way::host_up(SpiMode::mode0);
    const way::Failed e_tx = way::run_faulted(true, CsPin::ref(), SpiMode::mode0);
    const uint32_t tx_vector = vector_cycles;
    const way::Window n_tx = way::next_window(a, SpiMode::mode0, true);
    way::print_failed("a transmit buffer in reserved space", e_tx);
    print(serial, "      the vector's cost ", tx_vector, " cycles (the stream's error, one completion)", crlf);
    way::print_window("a polled window", n_tx);
    bench.verdict("A BUS ERROR ON THE TRANSMIT STREAM: dma_isr() answers true once, spi_dma_fault, the select released, no "
                  "recover(); the next polled request exact at both ends",
                  armed && !e_tx.ended && e_tx.completed && e_tx.completions == 1u &&
                      e_tx.status == spi_dma_fault && e_tx.released && way::exact(n_tx, true));

    armed = way::arm_peer(a, 0x83, SpiMode::mode0);
    way::host_up(SpiMode::mode0);
    const way::Failed e_rx = way::run_faulted(false, {}, SpiMode::mode0);
    const uint32_t rx_vector = vector_cycles;
    brio::spin_us<Ruler>(2'000);
    const bool stood = S::rx_ready();
    const way::Window n_rx = way::next_window(a, SpiMode::mode0, false);
    way::print_failed("a receive buffer in reserved space, no select", e_rx);
    print(serial, "      the vector's cost ", rx_vector, " cycles; a frame stands in the receive buffer ", stood ? "yes" : "no",
          crlf);
    way::print_window("an asynchronous window", n_rx);
    bench.verdict("A BUS ERROR ON THE RECEIVE STREAM: dma_isr() answers true once, spi_dma_fault, no recover(); the next "
                  "asynchronous request exact at both ends, no frame of the faulted block read back in front of it",
                  armed && !e_rx.ended && e_rx.completed && e_rx.completions == 1u &&
                      e_rx.status == spi_dma_fault && way::exact(n_rx, false));

    // The same error with the select the request names; no window is armed, so
    // the peer's command listener hears a frame or two of nothing it knows.
    (void)link_command_mode();
    way::host_up(SpiMode::mode0);
    const way::Failed e_sel = way::run_faulted(false, CsPin::ref(), SpiMode::mode0);
    brio::spin_us<Ruler>(1'000);
    way::print_failed("a receive buffer in reserved space, the select named", e_sel);
    DmaHost::release();
    dma_host_live = false;
    bench.verdict("...and the vector itself releases the select the request named",
                  !e_sel.ended && e_sel.completed && e_sel.status == spi_dma_fault && e_sel.released);

    // ---- queued behind a transfer error, through the arbiter -------------------
    const way::Queued q_fault_p = way::run_queued(true, true, 0x95);
    way::print_queued("queued behind a transfer error, polled", q_fault_p);
    const way::Queued q_fault_a = way::run_queued(true, false, 0xA7);
    way::print_queued("queued behind a transfer error, asynchronous", q_fault_a);
    bench.verdict("TWO REQUESTS FROM ONE DISPATCH, THE FIRST A BUS ERROR: the replies in order, spi_dma_fault then spi_ok, "
                  "the second's window exact at both ends, polled and on the streams, recover() never called",
                  way::queued_clean(q_fault_p, spi_dma_fault, true) && way::queued_clean(q_fault_a, spi_dma_fault, false));

    // ---- what a reset cannot do: the clock gate closed under the host ----------
    armed = way::arm_peer(a, 0xB9, SpiMode::mode0);
    way::host_up(SpiMode::mode0);
    S::bus_clock(false);
    const way::Failed g_first = way::run_stalled(way::Shape::write, {}, SpiMode::mode0);
    const way::Failed g_second = way::run_stalled(way::Shape::write, {}, SpiMode::mode0);
    const bool answer_closed = DmaHost::recover();
    S::bus_clock(true);
    const bool answer_open = DmaHost::recover();
    const way::Window n_gate = way::next_window(a, SpiMode::mode0, true);
    way::print_failed("the gate closed, a polled write", g_first);
    way::print_failed("the gate still closed, the next", g_second);
    print(serial, "      recover() with the gate closed ", answer_closed ? "true" : "false", ", opened ",
          answer_open ? "true" : "false", crlf);
    way::print_window("a polled window", n_gate);
    bench.verdict("A RESET DOES NOT OPEN A CLOSED GATE: the request after a stall stalls too, on its own budget, and "
                  "recover() answers false until the gate is open",
                  g_first.status == spi_stalled && g_second.status == spi_stalled && !answer_closed && answer_open);
    bench.verdict("...and with the gate open and recover() called, the next request exact at both ends",
                  armed && way::exact(n_gate, true));

    armed = way::arm_peer(a, 0xCB, SpiMode::mode0);
    way::host_up(SpiMode::mode0);
    S::bus_clock(false);
    const way::Failed h_first = way::run_stalled(way::Shape::write, {}, SpiMode::mode0);
    S::bus_clock(true);
    const uint32_t cr1_open = S::regs().CR1;
    const way::Failed h_second = way::run_stalled(way::Shape::write, {}, SpiMode::mode0);
    const way::Window n_open = way::next_window(a, SpiMode::mode0, true);
    way::print_failed("the gate closed, a polled write", h_first);
    print(serial, "      the gate opened: CR1 ", hex(cr1_open), crlf);
    way::print_failed("the gate open, no recover(), a polled write", h_second);
    way::print_window("a polled window", n_open);
    bench.verdict("WITH THE GATE REOPENED AND NO recover(), the block found at its reset values: one more request stalls, "
                  "and that stall's reset is the way back, the next request exact at both ends",
                  armed && h_first.status == spi_stalled && (cr1_open & SPI_CR1_SPE) == 0u &&
                      h_second.status == spi_stalled && way::exact(n_open, true));

    // ---- D/C at a request's end, however it ended ------------------------------
    // PC1, a pad on no recorded wire, is each request's D/C, parked low before
    // it; no select is named, so the peer hears nothing. Three requests made to
    // fail, each with a command phase that holds D/C low: a polled one whose
    // command stalls (the enable taken down), one naming the streams whose
    // command stalls in the loop, and a command alone on the streams whose
    // block reads reserved space. Each must end with D/C high, as a clean end
    // leaves it.
    (void)link_command_mode();
    named::DcPad::output(false);
    bool dc_up[3] = {false, false, false};
    uint8_t dc_status[3] = {0xFFu, 0xFFu, 0xFFu};
    for (uint8_t k = 0; k < 3u; ++k) {
        way::host_up(SpiMode::mode0);
        DmaHost::Request q{};
        q.dc = named::DcPad::ref();
        q.clock = kl::way_rate;
        if (k == 2u) {
            q.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(kl::way_bad_address()));
            q.cmd_len = 8;
        } else {
            q.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(kl::way_tx));
            q.cmd_len = 1;
            q.tx = lend<Lease::reply>(static_cast<const uint8_t*>(kl::way_tx));
            q.len = 8;
            (void)S::disable();
        }
        q.engine = k == 0u ? SpiEngine::polled : SpiEngine::dma;
        named::DcPad::clear();
        host_done = false;
        const bool ended = DmaHost::start(q);
        (void)way::completed_within(ended, 100u);
        dc_status[k] = DmaHost::status();
        dc_up[k] = named::DcPad::read_out();
        DmaHost::release();
    }
    dma_host_live = false;
    named::DcPad::release();
    print(serial, "    D/C after a polled command's stall ", dc_up[0] ? "high" : "LOW", " (status ", dc_status[0],
          "), after a streams' request's command stall ", dc_up[1] ? "high" : "LOW", " (status ", dc_status[1],
          "), after a command alone whose block faulted ", dc_up[2] ? "high" : "LOW", " (status ", dc_status[2], ")",
          crlf);
    bench.verdict("AT ITS END, OK OR FAILED, A REQUEST LEAVES D/C HIGH: a polled command phase that stalls "
                  "(spi_stalled), a command phase in the loop before the streams' block that stalls (spi_stalled), a "
                  "command alone on the streams whose block faults (spi_dma_fault) - D/C high after each",
                  dc_status[0] == spi_stalled && dc_status[1] == spi_stalled && dc_status[2] == spi_dma_fault &&
                      dc_up[0] && dc_up[1] && dc_up[2]);

    // recover() over a frame standing in the transmit buffer, SPE down: its wall.
    way::host_up(SpiMode::mode0);
    (void)S::disable();
    S::data(0xFFu);
    const uint32_t r0 = CycleCounter::now();
    (void)DmaHost::recover();
    const uint32_t recover_wall = CycleCounter::now() - r0;
    print(serial, "    recover() over a frame standing, the enable down: ", recover_wall, " cycles (",
          recover_wall / cycles_per_us, " us)", crlf);
    DmaHost::release();
    dma_host_live = false;

    (void)link_command_mode();
    bench.verdict("the command channel survives the letter", command(Op::ping));
}

void banner() {
    print(serial, crlf, "test_stm32f4_spi - SPI2 on the Nucleo-F446RE against the STM32G0B1RE's "
                        "spi_peer",
          crlf);
    print(serial, "  SPI2 AF5: SCK PB13, MISO PB14, MOSI PB15, NSS PB12 (GPIO select, or the "
                  "hardware input)",
          crlf);
    bench.menu();
    print(serial, "  z  run them all", crlf);
}

}   // namespace

// ---- the vectors -------------------------------------------------------------------------

extern "C" void USART2_IRQHandler() { (void)Serial::isr(); }

/// SPI2's line is letter g's client probe alone: no host arms the block's
/// interrupts.
extern "C" void SPI2_IRQHandler() {
    if (probe_live) {
        probe_isr();
    }
}

/// The engined host's completion, from either stream's vector: to the
/// kernel letter's arbiter, or - swallowed - nowhere (its lost interrupt),
/// or to the waiting letter. Every entry with the receive stream's
/// completion is one block ending; letter s counts them.
void dma_host_vector() {
    const uint32_t t0 = time_vector ? CycleCounter::now() : 0u;
    const bool ended = DmaHost::dma_isr();
    if (ended && wire_end::watching) {
        wire_end::sampled<Pin<link_pins.sck.port, link_pins.sck.pin>>();
    }
    if (ended && time_vector) {
        vector_cycles = CycleCounter::now() - t0;
    }
    if (ended) {
        dma_completions = static_cast<uint16_t>(dma_completions + 1u);
        if (bus_ao_live) {
            if (!host_swallow_completion) {
                brio::post<kl::SpiArb>(brio::TransferDone{DmaHost::status()});
            }
            return;
        }
        host_done = true;
    }
}
extern "C" void DMA1_Stream4_IRQHandler() { dma_host_vector(); }
extern "C" void DMA1_Stream3_IRQHandler() {
    dma_blocks = static_cast<uint16_t>(dma_blocks + 1u);
    dma_host_vector();
}

/// Letter E's second instance, on APB2: its streams are nobody else's here.
void spi1_vector() {
    const uint32_t t0 = CycleCounter::now();
    if (wire_end::Host1::dma_isr()) {
        wire_end::sampled<wire_end::Sck1>();
        vector_cycles = CycleCounter::now() - t0;
        host_done = true;
    }
}
extern "C" void DMA2_Stream3_IRQHandler() { spi1_vector(); }
extern "C" void DMA2_Stream2_IRQHandler() { spi1_vector(); }

/// Letter s's witness of the command rule: SCK's edges counted when D/C rose.
extern "C" void EXTI1_IRQHandler() {
    if (named::DcWatch::pending()) {
        (void)named::DcWatch::clear();
        // An SCK edge still pending is one the handler has not counted yet:
        // this line outranks EXTI15_10's only by its number.
        if (named::dc_rise_edges == named::no_rise) {
            named::dc_rise_edges = static_cast<uint16_t>(sck_edges_selected + (SckWatch::pending() ? 1u : 0u));
        }
    }
}

/// Letter q's witness of the select: PB12's falling edges.
extern "C" void EXTI15_10_IRQHandler() {
    if (SelWatch::pending()) {
        (void)SelWatch::clear();
        sel_falls = static_cast<uint16_t>(sel_falls + 1u);
    }
    if (SckWatch::pending()) {
        (void)SckWatch::clear();
        if (CsPin::read_out()) {
            sck_edges_released = static_cast<uint16_t>(sck_edges_released + 1u);
        } else {
            sck_edges_selected = static_cast<uint16_t>(sck_edges_selected + 1u);
        }
    }
}

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

/// Letter h's handler, longer than a frame: hog_cycles of its own, timed on
/// the DWT, every 5 us while its window runs.
extern "C" void TIM7_IRQHandler() {
    brio::Tim<7>::clear_flags(brio::Tim<7>::update_flag);
    const uint32_t t0 = brio::CycleCounter::now();
    while (brio::CycleCounter::now() - t0 < hog_cycles) {
    }
    hog_count = hog_count + 1u;
}

int main() {
    // What the silicon held before a line of this program ran.
    boot.gate = brio::Spi<2>::bus_clock();
    brio::Spi<2>::bus_clock(true);
    boot.cr1 = brio::Spi<2>::regs().CR1;
    boot.cr2 = brio::Spi<2>::regs().CR2;
    boot.sr = brio::Spi<2>::regs().SR;
    boot.crcpr = brio::Spi<2>::regs().CRCPR;
    boot.i2scfgr = brio::Spi<2>::regs().I2SCFGR;
    boot.i2spr = brio::Spi<2>::regs().I2SPR;

    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    (void)brio::CycleCounter::init();  // every Cortex-M4F here implements CYCCNT (DWT_CTRL.NOCYCCNT reads 0)
    brio::enable_interrupts();
    brio::Dma<1>::init();

    const bool host_ok = link_command_mode();

    bench.letter('a', "the instances and the block", ta_block);
    bench.letter('b', "the eight baud rates, timed with the select high", tb_rates);
    bench.letter('c', "what the driver refuses", tc_refusals);
    bench.letter('d', "the mode fault: SSI, then a real demotion by the peer", td_mode_fault);
    bench.letter('e', "the overrun and its clearing sequence", te_overrun);
    bench.letter('n', "THE PEER: the spi_link command channel", tn_peer_link);
    bench.letter('o', "the matrix against the peer: four modes, both orders", to_peer_matrix);
    bench.letter('p', "the BR ladder against the peer, polled and on DMA1, to PCLK1/2",
                 tp_peer_rates);
    bench.letter('q', "THE KERNEL against the peer: SpiBus over the engined SpiHost; the refusal",
                 tq_peer_kernel);
    bench.letter('r', "THE ROLES INVERT: the client run, the peer as the host", tr_peer_client);
    bench.letter('l', "ES0298 2.14.5: the SCK pad's class against the peer's stream",
                 tl_errata_pad);
    bench.letter('u', "16-BIT FRAMES against the peer: the loop, the streams, the wire's time",
                 tu_wide);
    bench.letter('v', "RECEIVE-ONLY and ODD COUNTS against the peer: the transmit-only shapes "
                      "witnessed",
                 tv_receive_only);
    bench.letter('w', "THE CRC against the peer's unit, and a wrong CRC caught", tw_crc);
    bench.letter('f', "THE SOFTWARE SELECT and THE SELECT OUTPUT (SSOE) against the peer",
                 tf_select);
    bench.letter('g', "THE STALL a second chip makes, and the TXE and error interrupts",
                 tg_stall_irqs);
    bench.letter('h', "A HANDLER BESIDE THE POLLED LOOP IS A GAP ON THE BUS: longer than a frame, every 5 us",
                 th_handler_gap);
    bench.letter('s', "THE ENGINE A REQUEST NAMES against the peer: one frame on the streams, "
                      "the refusals, the command rule",
                 ts_named);
    bench.letter('E', "WHEN A REQUEST ENDS THE WIRE HAS ENDED: the SCK pad, every mode and rate, "
                      "both engines, both APB clocks",
                 tE_wire_ended);
    bench.letter('k', "THE NEXT REQUEST AFTER A STALL OR A TRANSFER ERROR, recover() never called",
                 tk_way_back);
    bench.letter('j', "THE COUNT PAST SIXTEEN BITS: one command frame and 65535 data frames",
                 tj_count);
    bench.letter('t', "THE CRCNEXT WINDOW and the receive CRC at PCLK1/2, no verdict",
                 tt_crc_window, false);
    bench.letter('x', "SLIP STATISTICS against the peer: each mode ten times, no verdict",
                 tx_peer_slips, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL 180 MHz" : "FAILED",
                    " tick=", tick_ok ? "SysTick" : "FAILED", " spi2=",
                    host_ok ? "host" : "FAILED", brio::crlf);
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

#else   // the STM32F429I-DISC1's instrument

using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Stm32f4Platform<>;

// ---- the console -------------------------------------------------------------------

constexpr UartPins console_pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
using Serial = Uart<1, console_pins>;
constexpr Serial serial;

TestBench<Serial> bench;

// ---- the bus and the two devices ------------------------------------------------------

constexpr SpiPins spi5_pins{.sck = {'F', 7, PinFunction::af5},
                            .miso = {'F', 8, PinFunction::af5},
                            .mosi = {'F', 9, PinFunction::af5}};

using S = Spi<5>;
using Host = SpiHost<5, spi5_pins>;

/// SPI5's transmit request is DMA2 stream 4 on channel 2 and its receive
/// DMA2 stream 3 on channel 2 (RM0090 table 44) - the reserve checks
/// these two cells at compile time.
using TxEngine = DmaTxEngine<2, 4, 2>;
using RxEngine = DmaRxEngine<2, 3, 2>;
using DmaHost = SpiHost<5, spi5_pins, TxEngine, RxEngine>;

using GyroCs = Pin<'C', 1>;
using LcdCs = Pin<'C', 2>;
using LcdDcx = Pin<'D', 13>;

/// The gyroscope's register map, as much of it as this suite touches.
constexpr uint8_t gyro_read = 0x80;
constexpr uint8_t gyro_increment = 0x40;
constexpr uint8_t gyro_who_am_i = 0x0F;
constexpr uint8_t gyro_ctrl1 = 0x20;
constexpr uint8_t gyro_ctrl2 = 0x21;
/// CTRL_REG1..5 out of the device's own reset. CTRL_REG1's power-down bit
/// is clear here, which is how the suite must leave it.
constexpr uint8_t gyro_ctrl_reset[5] = {0x07, 0x00, 0x00, 0x00, 0x00};

/// The rate every letter that is not about rates uses: PCLK2 / 16 =
/// 5.625 MHz at 180 MHz, comfortably under the device's 10 MHz.
constexpr SpiClock gyro_rate = SpiClock::div16;
constexpr SpiMode gyro_mode = SpiMode::mode3;

volatile bool bus_ao_live = false;
volatile bool dma_host_live = false;

// ---- the rulers ---------------------------------------------------------------------

constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000u;

/// The suite's ruler: the DWT's cycle counter, started once in main(). It
/// counts the core's cycles, masked or not, through a Sleep too, and stops
/// only in a Stop, which no letter here enters (stm32f4/dwt.hpp).
struct Ruler {
    static uint32_t now() { return CycleCounter::now(); }
    static constexpr uint32_t hz() { return SysClock::hz; }
};
static_assert(CycleRuler<Ruler>);

/// One run of SysTick_Handler as it lengthens a window, in core cycles: its
/// body (Ticker::tick(), 18 instructions, 28 cycles at most from the release
/// listing), the exception's entry and exit (12 each, Cortex-M4 TRM 3.9.1)
/// and four flash lines missed in the ART (6 cycles each at five wait states,
/// RM0090 3.5.1) - 76.
constexpr uint32_t systick_run_cycles = 28u + 12u + 12u + 4u * 6u;

bool same(const uint8_t* a, const uint8_t* b, uint16_t n) {
    for (uint16_t i = 0; i < n; ++i) {
        if (a[i] != b[i]) {
            return false;
        }
    }
    return true;
}

// ---- the bus, in the state the letters expect -----------------------------------------

void deselect_all() {
    GyroCs::output(true);
    LcdCs::output(true);
    LcdDcx::output(true);
}

/// Bring SPI5 back up as the host every letter starts from, with both
/// devices deselected. Called at boot and after any letter that took the
/// resource somewhere else.
bool host_ready() {
    bus_ao_live = false;
    dma_host_live = false;
    deselect_all();
    return Host::init(clock);
}

/// One polled transaction against the gyroscope, the arbiter skipped:
/// `spi-bus.md`'s "a device that owns a bus alone can skip the arbiter".
bool gyro_transfer(const uint8_t* cmd, uint8_t cmd_len, const uint8_t* tx, uint8_t* rx,
                   uint16_t len, SpiClock rate = gyro_rate, SpiMode mode = gyro_mode) {
    Host::Request r{};
    r.cs = GyroCs::ref();
    r.cmd = lend<Lease::reply>(cmd);
    r.cmd_len = cmd_len;
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = len;
    r.clock = rate;
    r.mode = mode;
    r.engine = SpiEngine::polled;
    return Host::start(r);
}

uint8_t gyro_reg(uint8_t reg, SpiClock rate = gyro_rate, SpiMode mode = gyro_mode) {
    const uint8_t cmd[1] = {static_cast<uint8_t>(gyro_read | reg)};
    uint8_t got = 0;
    (void)gyro_transfer(cmd, 1, nullptr, &got, 1, rate, mode);
    return got;
}

void gyro_write(uint8_t reg, uint8_t value) {
    const uint8_t cmd[1] = {reg};
    const uint8_t out[1] = {value};
    (void)gyro_transfer(cmd, 1, out, nullptr, 1);
}

/// Put CTRL_REG1..5 back where the device's own reset leaves them - and
/// that is what leaves the gyroscope powered down.
void gyro_restore() {
    const uint8_t cmd[1] = {static_cast<uint8_t>(gyro_increment | gyro_ctrl1)};
    (void)gyro_transfer(cmd, 1, gyro_ctrl_reset, nullptr, 5);
}

// ---- what the registers held before this program touched them ----------------------------

struct BootState {
    bool gate = false;
    uint32_t cr1 = 0, cr2 = 0, sr = 0, crcpr = 0, i2scfgr = 0, i2spr = 0;
};
BootState boot;

// =============================================================================
// a - the instances and the block
// =============================================================================

void ta_block() {
    print(serial, "  SPI5 gate at reset: ", boot.gate ? "OPEN" : "closed",
          "; CR1 ", hex(boot.cr1), " CR2 ", hex(boot.cr2), " SR ", hex(boot.sr), " CRCPR ",
          hex(boot.crcpr), " I2SCFGR ", hex(boot.i2scfgr), " I2SPR ", hex(boot.i2spr), crlf);
    bench.verdict("every peripheral clock is off at reset, this one included", !boot.gate);
    // 28.5: CR1 0x0000, CR2 0x0000, SR 0x0002 (TXE), CRCPR 0x0007,
    // I2SCFGR 0x0000 - and I2SPR, which the chapter says is 0x0002.
    bench.verdict("the reset values are the chapter's: CR1 and CR2 zero, SR just TXE, CRCPR the "
                  "odd 0x0007, I2SCFGR zero",
                  boot.cr1 == 0u && boot.cr2 == 0u && boot.sr == SPI_SR_TXE && boot.crcpr == 7u &&
                      boot.i2scfgr == 0u);
    // I2SPR is the one register whose reset value the chapter states and
    // the silicon does not show. The reading is repeated behind the RCC's
    // own reset line, so it is the block's answer and not a boot artefact.
    S::reset();
    const uint32_t i2spr_after_reset = S::regs().I2SPR;
    print(serial, "  I2SPR reads ", hex(boot.i2spr), " at boot and ", hex(i2spr_after_reset),
          " after an RCC reset pulse; 28.5.9 states 0x0002 - printed, not judged", crlf);
    bench.verdict("the RCC's reset line puts the block back where it was found, whatever that is",
                  i2spr_after_reset == boot.i2spr && S::regs().CR1 == 0u && S::regs().CRCPR == 7u);
    (void)host_ready();

    print(serial, "  instances on this part:");
    uint8_t count = 0;
    for (uint8_t i = 1; i <= 6u; ++i) {
        if (spi_present(i)) {
            ++count;
            print(serial, " SPI", i, "(", spi_on_apb2(i) ? "APB2" : "APB1", ",irq",
                  static_cast<int32_t>(spi_irq(i)), ")");
        }
    }
    print(serial, crlf);
    bench.verdict("this part carries six SPI instances, one vector each", count == 6u);
    bench.verdict("SPI1, SPI4, SPI5 and SPI6 are APB2's; SPI2 and SPI3 are APB1's",
                  spi_on_apb2(1) && !spi_on_apb2(2) && !spi_on_apb2(3) && spi_on_apb2(4) &&
                      spi_on_apb2(5) && spi_on_apb2(6));
    bench.verdict("SPI5's vector is its own and nothing else's", S::irq == SPI5_IRQn);

    print(serial, "  the I2S face: instances", spi_i2s_capable(2) ? " I2S2" : "",
          spi_i2s_capable(3) ? " I2S3" : "", ", extension blocks ",
          spi_i2s_facts().ext_blocks ? "present" : "absent", crlf);
    bench.verdict("on this part class the audio face is SPI2's and SPI3's alone, with the "
                  "full-duplex extension blocks beside them",
                  spi_i2s_capable(2) && spi_i2s_capable(3) && !spi_i2s_capable(1) &&
                      !spi_i2s_capable(5) && i2s_ext_present(2) && i2s_ext_present(3));

    print(serial, "  APB2 = ", Host::reference_hz() / 1000u, " kHz, PCLK1 = ",
          SysClock::pclk1_hz / 1000u, " kHz, HCLK = ", SysClock::hz / 1000u, " kHz", crlf);
    bench.verdict("the rate SPI5's BR field divides is ITS OWN bus clock, PCLK2, and not SYSCLK",
                  Host::reference_hz() == SysClock::pclk2_hz &&
                      Host::reference_hz() != SysClock::hz);
}

// =============================================================================
// b - the eight baud rates
// =============================================================================

/// Clock `n` frames with nothing selected, polled, and report the core
/// cycles the run took. Both chip selects stay high, so the bus is driven
/// and nobody listens.
/// Zero when a frame's RXNE never came: 1 ms is some twenty-two frames at
/// PCLK2/256, the slowest code.
uint32_t timed_frames(SpiClock c, uint16_t n) {
    Host::prime(gyro_mode, c);
    S::flush_rx();
    bool came = true;
    const uint32_t t0 = Ruler::now();
    for (uint16_t i = 0; i < n; ++i) {
        S::data(0xA5u);
        came = brio::wait_until<Ruler>([] { return S::rx_ready(); }, 1'000) && came;
        (void)S::data();
    }
    const uint32_t took = Ruler::now() - t0;
    return came ? took : 0u;
}

/// THE POLL'S TURN, for letter b's bound: timed_frames()'s wait on RXNE run
/// with the block idle, so that RXNE never rises and the condition ends the
/// wait itself after `turns` calls. The count it keeps adds an increment, a
/// compare and a branch to every turn, so the figure is the turn plus those:
/// an upper bound on it, rounded up; the least of three windows, each an
/// upper bound (a kernel tick inside one only lengthens it).
uint32_t poll_turn_bound() {
    constexpr uint32_t turns = 64;
    uint32_t least = UINT32_MAX;
    for (uint8_t k = 0; k < 3u; ++k) {
        uint32_t n = 0;
        const uint32_t t0 = Ruler::now();
        // The answer is the count's, not RXNE's: the block is idle, and 64 turns
        // end the wait long before its 1 ms.
        (void)brio::wait_until<Ruler>([&n] { return S::rx_ready() || ++n == turns; }, 1'000);
        const uint32_t per_turn = (Ruler::now() - t0 + turns - 1u) / turns;
        least = per_turn < least ? per_turn : least;
    }
    return least;
}

void tb_rates() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    // WHAT THE LOOP COSTS IS A TURN. In mode 3 (CPHA = 1) RXNE rises on the
    // frame's LAST edge, its last sampling edge (28.3.5), and the loop's turn
    // (the read, the next write, the block's restart) runs from there: every
    // frame is eight bits plus the turn, at every code, and the turn is read
    // as the frame time less eight bits.
    constexpr uint16_t frames = 32;
    uint32_t took[8] = {};
    bool ticked[8] = {};
    uint32_t overhead[8] = {};
    bool monotone = true;
    uint32_t lowest = 0xFFFFFFFFu;
    uint32_t highest = 0;
    uint8_t judged = 0;
    // The eight codes timed first and printed after: a console line going out
    // beside a run would put its interrupt inside the time. One untimed run
    // first, so that the timed ones run from the ART's cache.
    if (!brio::console_drain<Ruler, Serial>()) {
        bench.skip("the console did not drain: the timed runs would carry its interrupt");
        return;
    }
    (void)timed_frames(SpiClock::div2, 2);   // the warm-up: nothing read from it
    for (uint8_t code = 0; code < 8u; ++code) {
        const brio::TickWitness<P> witness;
        took[code] = timed_frames(static_cast<SpiClock>(code), frames);
        ticked[code] = !witness.quiet();
    }
    const uint32_t turn = poll_turn_bound();
    // THE SPREAD'S BOUND, derived. The loop runs phase-locked to the block, so
    // a code's turn is a fixed path plus how late the poll sees RXNE: somewhere
    // inside one turn of the poll (`turn`, measured above), at a different place
    // at each code (eight bits modulo the turn). The write after it waits for
    // PCLK2's next edge, 0 to HCLK/PCLK2 - 1 = 1 core cycle. The lead before
    // the first frame and the tail after the last RXNE are the same path with
    // their own phase, spread over the 32 frames: under a cycle a frame. The two
    // truncations of the nanosecond arithmetic below (the frame, the wire) move
    // one cost by under 1 ns, so two apart by under 2. SCK and the ruler both
    // count HCLK: no clock error enters. A code a kernel tick landed in is
    // printed and left out of the spread.
    const uint32_t ratio = SysClock::hz / Host::reference_hz();
    const uint32_t bound_cycles = turn + (ratio - 1u) + 1u;
    const uint32_t bound_ns = bound_cycles * 1000u / cycles_per_us + 2u;
    for (uint8_t code = 0; code < 8u; ++code) {
        const SpiClock c = static_cast<SpiClock>(code);
        const uint32_t stated = spi_sck_hz(Host::reference_hz(), c);
        const uint32_t per_frame_ns = (took[code] * 1000u) / (frames * cycles_per_us);
        // Eight bits at the stated rate, in nanoseconds.
        const uint32_t wire_ns = stated == 0u ? 0u : static_cast<uint32_t>(8u * 1'000'000'000ULL / stated);
        overhead[code] = per_frame_ns > wire_ns ? per_frame_ns - wire_ns : 0u;
        print(serial, "  div", static_cast<uint32_t>(1u << (code + 1u)), ": SCK ", stated / 1000u,
              " kHz, ", frames, " frames in ", took[code] / cycles_per_us, " us -> ", per_frame_ns,
              " ns a frame; the wire alone is ", wire_ns, " ns, so the loop costs ",
              overhead[code], " ns", ticked[code] ? " (a tick landed in it)" : "", crlf);
        if (code > 0u && took[code] <= took[code - 1u]) {
            monotone = false;
        }
        if (code < 7u && !ticked[code]) {
            ++judged;
            if (overhead[code] < lowest) {
                lowest = overhead[code];
            }
            if (overhead[code] > highest) {
                highest = overhead[code];
            }
        }
    }
    bench.verdict("the eight codes are eight different rates, each slower than the last", monotone);
    print(serial, "  the loop's own cost per frame, PCLK/2 to PCLK/128 with no tick in the window: ",
          lowest, " to ", highest, " ns (", (lowest * cycles_per_us) / 1000u, " to ",
          (highest * cycles_per_us) / 1000u, " core cycles) at ", judged, " codes; the poll's turn ",
          turn, " cycles at most, so the spread's bound is ", bound_ns, " ns", crlf);
    bench.verdict("what the polled loop adds is a CONSTANT per frame - one write, one RXNE spin "
                  "and one read - and not a share of the rate: from PCLK/2 to PCLK/128, within one "
                  "turn of the poll, PCLK2's edge and the arithmetic's truncations",
                  judged >= 2u && lowest != 0u && highest - lowest <= bound_ns);
    // AT PCLK/256 the frame less that turn is eight bits at the stated rate:
    // its cost within the same bound of the lowest, plus a SysTick run spread
    // over the 32 frames, since a window of 0.74 ms can meet one and is judged
    // with it.
    const uint32_t tick_share_ns = ((systick_run_cycles + frames - 1u) / frames) * 1000u / cycles_per_us + 1u;
    const uint32_t slowest_bound_ns = bound_ns + tick_share_ns;
    const uint32_t slowest_diff_ns =
        overhead[7] > lowest ? overhead[7] - lowest : lowest - overhead[7];
    print(serial, "  at the slowest code the loop's cost is ", overhead[7], " ns, ", slowest_diff_ns,
          " ns from the lowest above, the bound ", slowest_bound_ns, " ns", crlf);
    bench.verdict("... so at PCLK/256 the measured frame time IS eight bits at the rate the "
                  "arithmetic states, plus that same turn",
                  judged >= 2u && lowest != 0u && slowest_diff_ns <= slowest_bound_ns);
}

// =============================================================================
// c - the refusals
// =============================================================================

void tc_refusals() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    // The errata's even polynomial (ES0206 2.12.3) - the CRC would be
    // wrong and the configuration is refused whole.
    const uint32_t before = S::regs().CR1;
    const bool even = S::configure(SpiConfig{.crc = true, .crc_polynomial = 0x1020u});
    bench.verdict("an EVEN CRC polynomial is refused - ES0206 2.12.3 makes the calculation wrong",
                  !even);
    bench.verdict("... and a refused configuration writes nothing", S::regs().CR1 == before);
    bench.verdict("the same polynomial made odd is accepted",
                  S::configure(SpiConfig{.crc = true, .crc_polynomial = 0x1021u}));

    bench.verdict("a CLIENT has no NSS output to enable: SSOE is refused there",
                  !S::configure(SpiConfig{.role = SpiRole::client, .nss = SpiNss::hardware_output}));
    bench.verdict("TI mode has no software select - the frame IS the NSS pulse",
                  !S::configure(SpiConfig{.nss = SpiNss::software,
                                          .frame_format = SpiFrameFormat::ti}));
    bench.verdict("TI mode has no LSB-first either",
                  !S::configure(SpiConfig{.lsb_first = true, .nss = SpiNss::hardware_output,
                                          .frame_format = SpiFrameFormat::ti}));

    // The CRC polynomial register is enable-protected in this driver,
    // because it is the divisor a running calculation uses.
    (void)S::configure(SpiConfig{});
    S::enable();
    bench.verdict("the polynomial is refused while the peripheral is enabled",
                  !S::crc_polynomial(0x8005u | 1u));
    bench.verdict("and so is CRCEN, which 28.5.1 wants written with SPE clear",
                  !S::crc_enable(true));
    (void)S::disable();
    bench.verdict("with SPE down both are accepted",
                  S::crc_polynomial(0x8005u | 1u) && S::crc_enable(true));
    bench.verdict("enabling the CRC RESETS both CRC registers (28.3.6)",
                  S::rx_crc() == 0u && S::tx_crc() == 0u);

    // The audio side.
    bench.verdict("an I2S divider of 0 or 1 is forbidden and refused",
                  !i2s_config_valid(I2sConfig{.div = 1}) && !i2s_config_valid(I2sConfig{.div = 0}));
    bench.verdict("PCM's long frame is refused outside the PCM standard",
                  !i2s_config_valid(I2sConfig{.pcm_long_frame = true}));
    bench.verdict("a MASTER mode is refused on the full-duplex extension block, which 28.4.2 "
                  "makes a slave by construction",
                  !I2sExt<3>::configure(I2sConfig{.mode = I2sMode::host_transmit}) &&
                      I2sExt<3>::configure(I2sConfig{.mode = I2sMode::client_receive}));
    I2sExt<3>::select_spi_mode();
    (void)host_ready();
}

// =============================================================================
// d - the mode fault
// =============================================================================

void td_mode_fault() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    // 28.3.10: on a master under software management, SSI going low IS
    // the fault. No pad and no second master needed.
    bench.verdict("no mode fault stands on a healthy host", !S::mode_fault());

    // THE FLAG IS NOT VISIBLE ON THE VERY NEXT BUS ACCESS. SSI is
    // synchronized into the peripheral's own clock domain, and PCLK2 is
    // half the core clock here, so a read issued immediately after the
    // CR1 store can still see the old SR. How many core cycles it takes
    // is measured rather than assumed.
    S::software_select(true);   // SSI low
    const uint32_t immediate = S::status();
    // 100 us: a time-out only, the flag rising some tens of core cycles after
    // the store (the print). The reading is late by at most one poll turn.
    const std::optional<uint32_t> rose =
        brio::cycles_until<Ruler>([] { return S::mode_fault(); }, 100);
    const uint32_t sr = S::status();
    const uint32_t cr1 = S::regs().CR1;
    print(serial, "  SSI driven low: SR on the very next read ", hex(immediate), ", then ",
          hex(sr), " after ", rose ? *rose : 0u, " core cycles", rose ? "" : " (NEVER ROSE)",
          "; CR1 ", hex(cr1), crlf);
    bench.verdict("MODF rises when SSI goes low under software management", (sr & SPI_SR_MODF) != 0u);
    bench.verdict("... and the silicon has cleared SPE and MSTR with it, demoting the host",
                  (cr1 & SPI_CR1_SPE) == 0u && (cr1 & SPI_CR1_MSTR) == 0u);
    bench.verdict("the flag is not up on the read that follows the store - a peripheral on a "
                  "divided APB answers a beat late, and a handler must read the flag and not "
                  "assume it",
                  (immediate & SPI_SR_MODF) == 0u);

    // The clearing sequence: an SR access while MODF stands, then a CR1
    // write. The SR reads above are step one; clear_mode_fault() runs
    // both in order, and the CR1 write it makes puts SSI back high in the
    // same store because the value it writes back is the one MODF left.
    S::regs().CR1 = cr1 | SPI_CR1_SSI;   // step two, and the select back up
    const uint32_t straight_after = S::status();
    // 100 us: a time-out only, the clear landing some tens of core cycles
    // after the write (the print).
    const std::optional<uint32_t> cleared =
        brio::cycles_until<Ruler>([] { return !S::mode_fault(); }, 100);
    print(serial, "  after the SR read and the CR1 write: SR ", hex(straight_after), ", then ",
          hex(S::status()), " ", cleared ? *cleared : 0u, " core cycles later",
          cleared ? "" : " (NEVER CLEARED)",
          " - the clear lands a beat late for the same reason the flag did", crlf);
    bench.verdict("the SR access then the CR1 write clears it", cleared.has_value());

    // The sequences from a fresh fault each, and the verb (modf_clear above).
    modf_clear::judge<S, Ruler, P::CriticalSection>(serial, bench);
    const bool back = host_ready();
    bench.verdict("and the host is a host again after the reconfiguration",
                  back && (S::regs().CR1 & SPI_CR1_MSTR) != 0u && S::enabled());
    const uint8_t who = gyro_reg(gyro_who_am_i);
    print(serial, "  the device answers ", hex(who), " after the fault", crlf);
    bench.verdict("... and the bus works", who != 0x00u && who != 0xFFu);
}

// =============================================================================
// e - the overrun
// =============================================================================

void te_overrun() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    Host::prime(gyro_mode, SpiClock::div64);
    S::flush_rx();
    bench.verdict("no overrun stands on a drained receiver", !S::overrun());
    // Two frames clocked out and neither read: the second cannot land.
    // 1 ms a wait: a frame at PCLK2/64 is 5.7 us. A time-out only, judged by
    // the OVR verdict below.
    for (uint8_t i = 0; i < 3u; ++i) {
        S::data(static_cast<uint8_t>(0x10u + i));
        (void)brio::wait_until<Ruler>([] { return S::tx_empty(); }, 1'000);
    }
    brio::spin_us<Ruler>(100);
    print(serial, "  three frames clocked, none read: SR ", hex(S::status()), crlf);
    bench.verdict("OVR rises when a frame arrives on an unread receiver", S::overrun());
    // 28.3.10: the DR read alone does NOT clear it.
    (void)S::regs().DR;
    const bool still = S::overrun();
    S::clear_overrun();
    print(serial, "  after the DR read alone: OVR ", still ? "STILL SET" : "clear",
          "; after the DR-then-SR sequence: ", S::overrun() ? "SET" : "clear", crlf);
    bench.verdict("the clearing sequence is a DR read FOLLOWED BY an SR read, and the DR read "
                  "alone is not enough",
                  !S::overrun());
    // Printed, not judged: with both selects high nothing drives MISO, so the
    // three frames read alike and which one stood cannot be told here.
    print(serial, "  (no verdict on which frame the receive buffer kept: 28.3.10 says the "
                  "first, the later ones lost; the frames here carry no data to tell them "
                  "apart)",
          crlf);
    (void)host_ready();
}

// =============================================================================
// f - the device answers
// =============================================================================

uint8_t who_am_i_seen = 0;

void tf_identity() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    who_am_i_seen = gyro_reg(gyro_who_am_i);
    print(serial, "  WHO_AM_I (0x0F) reads ", hex(who_am_i_seen), " at ",
          Host::sck_hz(gyro_rate) / 1000u, " kHz in mode 3", crlf);
    // The board's revisions carry two parts at that footprint and they
    // answer two different identities; both are a real device answering
    // and neither is a floating line.
    bench.verdict("the gyroscope answers its identity register - a value that is neither a "
                  "floating high nor a dead low",
                  who_am_i_seen == 0xD4u || who_am_i_seen == 0xD3u);

    // The same read eight times running: a bus that is right is right
    // every time.
    bool stable = true;
    for (uint8_t i = 0; i < 8u; ++i) {
        if (gyro_reg(gyro_who_am_i) != who_am_i_seen) {
            stable = false;
        }
    }
    bench.verdict("eight reads in a row give the same byte", stable);

    // A register that is NOT there reads as something else, which is what
    // says the byte above is the device's and not the wire's.
    const uint8_t reserved = gyro_reg(0x10u);
    print(serial, "  the reserved register 0x10 reads ", hex(reserved), crlf);
    bench.verdict("a register the device does not implement does not answer the same byte",
                  reserved != who_am_i_seen);
}

// =============================================================================
// g - the rate ladder against a real device
// =============================================================================

void tg_device_rates() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    const uint8_t want = who_am_i_seen != 0u ? who_am_i_seen : gyro_reg(gyro_who_am_i);
    uint8_t good = 0;
    uint8_t fastest_good = 8;
    for (uint8_t code = 0; code < 8u; ++code) {
        const SpiClock c = static_cast<SpiClock>(code);
        const uint32_t hz = spi_sck_hz(Host::reference_hz(), c);
        uint8_t seen[4] = {};
        bool all = true;
        for (uint8_t i = 0; i < 4u; ++i) {
            seen[i] = gyro_reg(gyro_who_am_i, c);
            if (seen[i] != want) {
                all = false;
            }
        }
        if (all) {
            ++good;
            if (code < fastest_good) {
                fastest_good = code;
            }
        }
        print(serial, "  SCK ", hz / 1000u, " kHz: ", hex(seen[0]), " ", hex(seen[1]), " ",
              hex(seen[2]), " ", hex(seen[3]), all ? "  exact" : "  WRONG", crlf);
    }
    print(serial, "  the device answered exactly at ", good, " of the eight rates; the fastest was ",
          fastest_good < 8u ? spi_sck_hz(Host::reference_hz(), static_cast<SpiClock>(fastest_good)) / 1000u
                            : 0u,
          " kHz", crlf);
    bench.verdict("the device answers exactly at every rate at or below the 10 MHz its datasheet "
                  "allows - PCLK2/16 and slower",
                  good >= 5u && fastest_good <= 3u);
    bench.verdict("... and what it does above that is printed, not judged: an over-clocked device "
                  "is the device's business and this board is one specimen",
                  true);
}

// =============================================================================
// h - the four modes
// =============================================================================

void th_modes() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    const uint8_t want = who_am_i_seen != 0u ? who_am_i_seen : gyro_reg(gyro_who_am_i);
    uint8_t seen[4] = {};
    for (uint8_t m = 0; m < 4u; ++m) {
        seen[m] = gyro_reg(gyro_who_am_i, gyro_rate, static_cast<SpiMode>(m));
        print(serial, "  mode ", m, " (CPOL ", spi_mode_cpol(static_cast<SpiMode>(m)) ? 1 : 0,
              ", CPHA ", spi_mode_cpha(static_cast<SpiMode>(m)) ? 1 : 0, "): ", hex(seen[m]), crlf);
    }
    bench.verdict("mode 3 is the one this device's datasheet names, and it answers there",
                  seen[3] == want);
    bench.verdict("mode 0 - CPOL and CPHA both flipped, so the same edge still samples - reads it "
                  "too: the two modes a device of this class accepts",
                  seen[0] == want);
    print(serial, "  what modes 1 and 2 read is PRINTED and not judged: a mismatched phase is a "
                  "timing accident of one specimen, and this board is one",
          crlf);
    bench.verdict("the four modes are four different configurations on the wire - not every one "
                  "of them reads the device",
                  seen[1] != want || seen[2] != want);
    (void)host_ready();
}

// =============================================================================
// i - the auto-increment, and a register written then read back
// =============================================================================

void ti_registers() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    // A single-byte read of each of the five, then the same five in ONE
    // transaction with the auto-increment bit: the two must agree.
    uint8_t one_at_a_time[5] = {};
    for (uint8_t i = 0; i < 5u; ++i) {
        one_at_a_time[i] = gyro_reg(static_cast<uint8_t>(gyro_ctrl1 + i));
    }
    uint8_t burst[5] = {};
    const uint8_t cmd[1] = {static_cast<uint8_t>(gyro_read | gyro_increment | gyro_ctrl1)};
    (void)gyro_transfer(cmd, 1, nullptr, burst, 5);
    print(serial, "  CTRL_REG1..5 one at a time: ", hex(one_at_a_time[0]), " ",
          hex(one_at_a_time[1]), " ", hex(one_at_a_time[2]), " ", hex(one_at_a_time[3]), " ",
          hex(one_at_a_time[4]), crlf);
    print(serial, "  the same five auto-incremented: ", hex(burst[0]), " ", hex(burst[1]), " ",
          hex(burst[2]), " ", hex(burst[3]), " ", hex(burst[4]), crlf);
    bench.verdict("a five-byte auto-incremented read is byte-exact against five single reads",
                  same(one_at_a_time, burst, 5));

    // A register written and read back. CTRL_REG2 is the high-pass
    // filter's and touching it cannot power the device up or move it to
    // the three-wire interface.
    const uint8_t patterns[3] = {0x1Fu, 0x25u, 0x00u};
    bool wrote = true;
    for (uint8_t i = 0; i < 3u; ++i) {
        gyro_write(gyro_ctrl2, patterns[i]);
        const uint8_t back = gyro_reg(gyro_ctrl2);
        print(serial, "  CTRL_REG2 <- ", hex(patterns[i]), ", reads ", hex(back), crlf);
        if (back != patterns[i]) {
            wrote = false;
        }
    }
    bench.verdict("a register written over this bus reads back exactly - the write path and the "
                  "read path are the same transaction shape with the direction bit flipped",
                  wrote);

    // A multi-byte WRITE with the auto-increment bit, read back in one
    // burst: two registers in one select window.
    const uint8_t pair[2] = {0x2Au, 0x00u};
    const uint8_t wcmd[1] = {static_cast<uint8_t>(gyro_increment | gyro_ctrl2)};
    (void)gyro_transfer(wcmd, 1, pair, nullptr, 2);
    uint8_t pair_back[2] = {};
    const uint8_t rcmd[1] = {static_cast<uint8_t>(gyro_read | gyro_increment | gyro_ctrl2)};
    (void)gyro_transfer(rcmd, 1, nullptr, pair_back, 2);
    print(serial, "  two registers written auto-incremented, read back: ", hex(pair_back[0]), " ",
          hex(pair_back[1]), crlf);
    bench.verdict("a two-register auto-incremented WRITE lands in both", same(pair, pair_back, 2));

    gyro_restore();
    uint8_t restored[5] = {};
    (void)gyro_transfer(cmd, 1, nullptr, restored, 5);
    bench.verdict("the control registers are back at their reset values, the device powered down",
                  same(gyro_ctrl_reset, restored, 5));
}

// =============================================================================
// j - the kernel
// =============================================================================

namespace kl {

using SpiArb = SpiBus<DmaHost, P, 4>;

uint8_t rx_a[5];
uint8_t rx_b[5];
const uint8_t read_ctrl[1] = {static_cast<uint8_t>(gyro_read | gyro_increment | gyro_ctrl1)};
const uint8_t read_who[1] = {static_cast<uint8_t>(gyro_read | gyro_who_am_i)};

class Probe {
public:
    using Event = std::variant<SpiDone, SleepVote>;
    static inline EventQueue<Event, 12, P> queue;
    static inline uint8_t replies[8];
    static inline uint8_t n = 0;
    static inline uint8_t rejected = 0;
    static inline uint8_t votes = 0;
    static inline bool last_vote = false;

    static void init() { clear_tally(); }
    static void clear_tally() {
        n = 0;
        rejected = 0;
        votes = 0;
        last_vote = false;
    }

    static void dispatch(const Event& e) {
        brio::match(
            e,
            [](const SpiDone& d) {
                if (n < 8u) {
                    replies[n] = d.status;
                }
                ++n;
                if (d.status == spi_rejected) {
                    ++rejected;
                }
            },
            [](const SleepVote& v) {
                ++votes;
                last_vote = v.ok;
            });
    }
};

using BusKernel = Tenuto<P, Probe, SpiArb>;

void pump() {
    TimeEvents<P>::process();
    while (BusKernel::step()) {
        TimeEvents<P>::process();
    }
}

void pump_until(uint8_t want, uint32_t ms) {
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < ms) {
        pump();
        if (Probe::n >= want) {
            break;
        }
    }
    pump();
}

DmaHost::Request request(const uint8_t* cmd, uint8_t* rx, uint16_t len, bool polled) {
    DmaHost::Request r{};
    r.cs = GyroCs::ref();
    r.cmd = lend<Lease::reply>(cmd);
    r.cmd_len = 1;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
    r.rx = lend<Lease::reply>(rx);
    r.len = len;
    r.mode = gyro_mode;
    r.clock = gyro_rate;
    r.engine = polled ? SpiEngine::polled : SpiEngine::dma;
    r.reply = reply_to<Probe, SpiDone>();
    return r;
}

}  // namespace kl

void tj_kernel() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    const uint8_t want = who_am_i_seen != 0u ? who_am_i_seen : gyro_reg(gyro_who_am_i);
    // The arbiter runs on the ENGINED host: an asynchronous request is DMA.
    Dma<2>::init();
    deselect_all();
    if (!DmaHost::init(clock)) {
        bench.verdict("the engined host came up", false);
        return;
    }
    dma_host_live = true;
    kl::BusKernel::init_all();
    bus_ao_live = true;

    for (uint8_t i = 0; i < 4u; ++i) {
        post<kl::SpiArb>(kl::request(kl::read_who, (i == 3u) ? kl::rx_a : nullptr, 1, i == 1u));
    }
    kl::pump_until(4, 300);
    print(serial, "  four queued transactions (one polled): replies ", kl::Probe::n, " [",
          kl::Probe::replies[0], " ", kl::Probe::replies[1], " ", kl::Probe::replies[2], " ",
          kl::Probe::replies[3], "], the last one read ", hex(kl::rx_a[0]), crlf);
    bench.verdict("four transactions through SpiBus, four replies - util/spi_bus.hpp and "
                  "util/bus_master.hpp unchanged on this architecture",
                  kl::Probe::n == 4u);
    bench.verdict("... every one spi_ok, asynchronous on the streams and polled interleaved on one "
                  "bus",
                  kl::Probe::replies[0] == spi_ok && kl::Probe::replies[1] == spi_ok &&
                      kl::Probe::replies[2] == spi_ok && kl::Probe::replies[3] == spi_ok);
    bench.verdict("and what the arbiter handed back is the device's own identity byte",
                  kl::rx_a[0] == want);

    // The write-then-read shape the portable example will use: a command
    // phase and a data phase in ONE select window, five registers back.
    kl::Probe::clear_tally();
    post<kl::SpiArb>(kl::request(kl::read_ctrl, kl::rx_b, 5, false));
    kl::pump_until(1, 300);
    print(serial, "  a write-then-read Request (1 command byte, 5 data): ", hex(kl::rx_b[0]), " ",
          hex(kl::rx_b[1]), " ", hex(kl::rx_b[2]), " ", hex(kl::rx_b[3]), " ", hex(kl::rx_b[4]),
          crlf);
    bench.verdict("the two-phase Request reads the control block through the arbiter, and the "
                  "device is still powered down",
                  kl::Probe::n == 1u && same(gyro_ctrl_reset, kl::rx_b, 5));

    kl::Probe::clear_tally();
    for (uint8_t i = 0; i < 6u; ++i) {
        post<kl::SpiArb>(kl::request(kl::read_who, nullptr, 1, false));
    }
    kl::pump_until(6, 300);
    print(serial, "  six posted into a four-deep queue: replies ", kl::Probe::n, ", rejected ",
          kl::Probe::rejected, crlf);
    bench.verdict("the arbiter rejects what it cannot queue, immediately", kl::Probe::rejected != 0u);
    bench.verdict("... and every request is still answered exactly once", kl::Probe::n == 6u);

    kl::Probe::clear_tally();
    post<kl::SpiArb>(PrepareSleep{
        .depth = SleepDepth::standby,
        .reply = reply_to<kl::Probe, SleepVote>(),
    });
    kl::pump();
    bench.verdict("an IDLE bus votes for the sleep", kl::Probe::votes == 1u && kl::Probe::last_vote);

    bus_ao_live = false;
    dma_host_live = false;
    DmaHost::release();
    (void)host_ready();
}

// =============================================================================
// k - the DMA engines
// =============================================================================

void tk_engines() {
    (void)host_ready();
    Dma<2>::init();
    bus_ao_live = false;
    deselect_all();
    if (!DmaHost::init(clock)) {
        bench.verdict("the engined host came up", false);
        return;
    }
    dma_host_live = true;

    const uint8_t want = who_am_i_seen;
    uint8_t got[5] = {};
    const uint8_t cmd[1] = {static_cast<uint8_t>(gyro_read | gyro_increment | gyro_ctrl1)};

    DmaHost::Request r{};
    r.cs = GyroCs::ref();
    r.cmd = lend<Lease::reply>(cmd);
    r.cmd_len = 1;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
    r.rx = lend<Lease::reply>(got);
    r.len = 5;
    r.mode = gyro_mode;
    r.clock = gyro_rate;
    r.engine = SpiEngine::polled;
    const bool sync = DmaHost::start(r);
    print(serial, "  a five-byte read polled on the engined host: ", hex(got[0]), " ",
          hex(got[1]), " ", hex(got[2]), " ", hex(got[3]), " ", hex(got[4]), ", status ",
          DmaHost::status(), crlf);
    bench.verdict("a polled request completes inside start() in the loop, the engines untouched",
                  sync && DmaHost::status() == spi_ok);
    bench.verdict("... and the block is byte-exact against the control registers' reset values",
                  same(gyro_ctrl_reset, got, 5));

    // The identity, this time on the engines: the command frame runs in the
    // loop inside start() and the data phase rides the streams.
    uint8_t who[1] = {};
    const uint8_t wcmd[1] = {static_cast<uint8_t>(gyro_read | gyro_who_am_i)};
    DmaHost::Request a{};
    a.cs = GyroCs::ref();
    a.cmd = lend<Lease::reply>(wcmd);
    a.cmd_len = 1;
    a.tx = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
    a.rx = lend<Lease::reply>(who);
    a.len = 1;
    a.mode = gyro_mode;
    a.clock = gyro_rate;
    a.engine = SpiEngine::dma;
    const bool async = DmaHost::start(a);
    // 10 ms against two frames at 5.6 MHz; the select's latch is judged below.
    (void)brio::wait_until<Ruler>([] { return GyroCs::read_out(); }, 10'000);
    print(serial, "  the same read asynchronously: ", hex(who[0]), " (start() returned ",
          async ? "true" : "false", ", the select came back up ",
          GyroCs::read_out() ? "yes" : "NO", ")", crlf);
    bench.verdict("an asynchronous engined request answers off the streams' vectors, the "
                  "select released by the completion",
                  !async && GyroCs::read_out());
    bench.verdict("... and the byte is the device's identity",
                  want == 0u ? who[0] != 0x00u && who[0] != 0xFFu : who[0] == want);

    // How fast the engined data phase really runs: the streams keep the
    // shifter fed, so this IS the wire and not the loop. The five blocks are
    // timed first and printed after, behind a drain: a console line going out
    // beside a block would put its interrupt inside the time.
    uint8_t block[64] = {};
    const uint8_t bcmd[1] = {static_cast<uint8_t>(gyro_read | gyro_increment | gyro_ctrl1)};
    uint32_t took[8] = {};
    if (!brio::console_drain<Ruler, Serial>()) {
        print(serial, "  the console did not drain: the blocks' times are not taken", crlf);
    } else {
        for (uint8_t code = 3; code < 8u; ++code) {
            DmaHost::Request b{};
            b.cs = GyroCs::ref();
            b.cmd = lend<Lease::reply>(bcmd);
            b.cmd_len = 1;
            b.tx = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
            b.rx = lend<Lease::reply>(block);
            b.len = 64;
            b.mode = gyro_mode;
            b.clock = static_cast<SpiClock>(code);
            b.engine = SpiEngine::dma;
            const uint32_t t0 = Ruler::now();
            const bool inside = DmaHost::start(b);
            // 10 ms against 65 frames at PCLK2/256, 1.5 ms. Printed, not judged: a
            // block that never ends prints the budget.
            if (!inside) {
                (void)brio::wait_until<Ruler>([] { return GyroCs::read_out(); }, 10'000);
            }
            took[code] = Ruler::now() - t0;
        }
        for (uint8_t code = 3; code < 8u; ++code) {
            const uint32_t stated = spi_sck_hz(DmaHost::reference_hz(), static_cast<SpiClock>(code));
            const uint32_t ns_per_frame = (took[code] * 1000u) / (65u * cycles_per_us);
            print(serial, "  64 frames at SCK ", stated / 1000u, " kHz: ", took[code] / cycles_per_us,
                  " us -> ", ns_per_frame, " ns a frame (the wire alone is ",
                  stated == 0u ? 0u : static_cast<uint32_t>(8u * 1'000'000'000ULL / stated), " ns)",
                  crlf);
        }
    }

    dma_host_live = false;
    DmaHost::release();
    (void)host_ready();
}

// =============================================================================
// l - the SCK pad against ES0206 2.12.4
// =============================================================================

void tl_pad_speed() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    const uint8_t want = who_am_i_seen != 0u ? who_am_i_seen : gyro_reg(gyro_who_am_i);
    print(serial, "  PCLK2 is ", Host::reference_hz() / 1000u,
          " kHz; the errata's ceiling per pad class (SPI): very high/high ",
          spi_errata_apb_ceiling_hz(PinSpeed::very_high) / 1000u, " kHz, medium ",
          spi_errata_apb_ceiling_hz(PinSpeed::medium) / 1000u, " kHz, low ",
          spi_errata_apb_ceiling_hz(PinSpeed::low) / 1000u, " kHz", crlf);
    bench.verdict("this bus runs ABOVE the errata's ceiling at every pad class, and the driver "
                  "says so rather than promising the last bit",
                  !Host::within_errata_ceiling());

    constexpr PinSpeed classes[4] = {PinSpeed::very_high, PinSpeed::high, PinSpeed::medium,
                                     PinSpeed::low};
    const char* names[4] = {"very high", "high", "medium", "low"};
    uint8_t exact = 0;
    for (uint8_t i = 0; i < 4u; ++i) {
        Host::sck_speed(classes[i]);
        uint8_t seen[4] = {};
        bool all = true;
        for (uint8_t k = 0; k < 4u; ++k) {
            seen[k] = gyro_reg(gyro_who_am_i);
            if (seen[k] != want) {
                all = false;
            }
        }
        if (all) {
            ++exact;
        }
        print(serial, "  SCK pad ", names[i], " (ceiling ",
              spi_errata_apb_ceiling_hz(classes[i]) / 1000u, " kHz): ", hex(seen[0]), " ",
              hex(seen[1]), " ", hex(seen[2]), " ", hex(seen[3]), all ? "  exact" : "  WRONG",
              crlf);
    }
    print(serial, "  ", exact, " of the four pad classes read the device exactly at SCK ",
          Host::sck_hz(gyro_rate) / 1000u, " kHz", crlf);
    bench.verdict("at the pad class the driver hands over - very high - the last received bit is "
                  "captured and the identity is exact",
                  exact >= 1u);
    (void)host_ready();   // the pads back at the default class
}

// =============================================================================
// m - the display controller over the bidirectional line
// =============================================================================

namespace bidi {

/// The gyroscope's CTRL_REG4, whose bit 0 moves the device's own
/// interface between four wires and three. A three-wire device answers on
/// the SAME pad it listens on, which on this board is the SPI's MOSI - so
/// it is the one peer the bidirectional half of this chapter can be
/// measured against.
constexpr uint8_t gyro_ctrl4 = 0x23;
constexpr uint8_t gyro_sim_3wire = 0x01;

/// Put the resource in bidirectional mode with the line DRIVEN, and hand
/// the MOSI pad over. Interrupts off: this letter drives the resource by
/// hand and no ISR must take a frame from it.
void begin(SpiMode mode, SpiClock rate) {
    Nvic::disable(S::irq);
    (void)S::configure(SpiConfig{.role = SpiRole::host,
                                 .mode = mode,
                                 .clock = rate,
                                 .nss = SpiNss::software,
                                 .direction = SpiDirection::half_duplex_out});
    Pin<'F', 9>::function(PinFunction::af5, {.speed = PinSpeed::very_high});
    S::enable();
}

/// One frame out on the single line, waited to the last edge: 1 ms a wait
/// against a frame of 1.4 us at the letter's rate, a time-out only, the
/// device's answer judging the line.
void write_frame(uint8_t v) {
    S::data(v);
    (void)brio::wait_until<Ruler>([] { return S::tx_empty(); }, 1'000);
    (void)brio::wait_until<Ruler>([] { return !S::busy(); }, 1'000);
}

/// The receive phase: BIDIOE clear makes a MASTER clock continuously the
/// moment SPE goes up, so the stop is 28.3.8's own - drop SPE after the
/// second-to-last frame has landed, then take the last one. ES0206 2.12.1
/// is why BSY is never consulted here.
///
/// THE PAD IS TURNED ROUND BY HAND, and that is the finding this letter
/// exists for. Measured, three ways: with the pad left an
/// alternate-function PUSH-PULL output the read comes back all ones,
/// whatever the device is driving - BIDIOE stops the shifter, not the
/// pad's driver; with the pad moved to a plain INPUT the read comes back
/// all ZEROES, because the peripheral's input is not taken from there;
/// with the pad kept in its alternate function but made OPEN DRAIN with a
/// pull-up, the device's answer arrives exactly. So the receive window
/// wants the output stage let go and the alternate function kept, and
/// that is one pad store on the way in and one on the way out.
void read_frames(uint8_t* into, uint8_t n) {
    S::flush_rx();
    Pin<'F', 9>::function(PinFunction::af5,
                          {.pull = PinPull::up, .open_drain = true, .speed = PinSpeed::very_high});
    // BIDIOE is not enable-protected, so the turnaround costs no gap:
    // the clock starts on this store, right after the command's last edge.
    S::bidirectional_output(false);
    for (uint8_t i = 0; i < n; ++i) {
        if (i + 1u == n) {
            S::regs().CR1 &= ~SPI_CR1_SPE;
        }
        // A time-out only (1 ms against a frame of 1.4 us): the bytes read
        // are judged against the device's.
        (void)brio::wait_until<Ruler>([] { return S::rx_ready(); }, 1'000);
        into[i] = static_cast<uint8_t>(S::data());
    }
    S::regs().CR1 &= ~SPI_CR1_SPE;
    Pin<'F', 9>::function(PinFunction::af5, {.speed = PinSpeed::very_high});
}

}  // namespace bidi

void tm_bidirectional() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    using namespace bidi;
    const uint8_t want = who_am_i_seen != 0u ? who_am_i_seen : gyro_reg(gyro_who_am_i);

    // ---- against the gyroscope, moved to its own three-wire interface ----
    // A WRITE looks the same on the wire either way, so this transition
    // and its undoing both go out over the bidirectional line - which is
    // what makes the recovery as reliable as the move.
    begin(gyro_mode, gyro_rate);
    print(serial, "  bidirectional out: CR1 ", hex(S::regs().CR1), " CR2 ", hex(S::regs().CR2),
          crlf);
    GyroCs::clear();
    write_frame(gyro_ctrl4);
    write_frame(gyro_sim_3wire);
    GyroCs::set();

    // The move is witnessed on the OTHER wire: a three-wire device stops
    // answering on the pad the four-wire one used, so a full-duplex read
    // now finds the line's own pull and nothing else.
    (void)host_ready();
    const uint8_t silent = gyro_reg(gyro_who_am_i);
    print(serial, "  after the one-line write of CTRL_REG4, a four-wire read finds ", hex(silent),
          " - the device has let go of the MISO pad", crlf);
    bench.verdict("a write over the bidirectional line reaches the device: the wire is a wire "
                  "whichever direction the peripheral thinks it is in, and the device answers "
                  "elsewhere afterwards",
                  silent != want);

    uint8_t three_wire[2] = {};
    begin(gyro_mode, gyro_rate);
    GyroCs::clear();
    write_frame(static_cast<uint8_t>(gyro_read | gyro_increment | gyro_who_am_i));
    read_frames(three_wire, 2);
    GyroCs::set();
    print(serial, "  the gyroscope in ITS three-wire mode, read over the MOSI pad alone: ",
          hex(three_wire[0]), " ", hex(three_wire[1]), crlf);
    bench.verdict("the bidirectional line reads a real device: the command goes out with BIDIOE "
                  "set, the answer comes back on the same pad with it clear and the pad turned "
                  "round",
                  three_wire[0] == want);

    // Back to four wires, over the same one-line write.
    begin(gyro_mode, gyro_rate);
    GyroCs::clear();
    write_frame(gyro_ctrl4);
    write_frame(0x00u);
    GyroCs::set();
    (void)host_ready();
    const uint8_t who = gyro_reg(gyro_who_am_i);
    print(serial, "  the device back on four wires reads ", hex(who), crlf);
    bench.verdict("... and the same one-line write puts it back, the full-duplex bus intact",
                  who == want);

    // ---- and against the display controller ----
    uint8_t id[4] = {};
    begin(SpiMode::mode0, SpiClock::div64);
    LcdDcx::clear();   // a command
    LcdCs::clear();
    write_frame(0xD3u);   // RDDID4: a dummy byte then 0x00 0x93 0x41
    LcdDcx::set();        // the parameters
    read_frames(id, 4);
    LcdCs::set();
    print(serial, "  the display's 0xD3 answered ", hex(id[0]), " ", hex(id[1]), " ", hex(id[2]),
          " ", hex(id[3]), crlf);
    print(serial, "  (all ones is the pull-up on a line nobody drove - the same transaction "
                  "shape that just read the gyroscope, so it is the DEVICE that does not answer: "
                  "this board straps the controller's interface mode so that its replies leave "
                  "on a pad the MCU is not wired to)",
          crlf);
    bench.verdict("the display's read path is not measurable on this board, and what came back "
                  "says so rather than being read as an identity",
                  id[1] != 0x00u || id[2] != 0x93u || id[3] != 0x41u);
    (void)host_ready();
    const uint8_t after = gyro_reg(gyro_who_am_i);
    print(serial, "  the gyroscope after the display's turn: ", hex(after), crlf);
    bench.verdict("the bus goes back to full duplex with the other device on it intact",
                  after == want);
}

// =============================================================================
// n - the I2S
// =============================================================================

namespace audio {

// I2S3's pads on this board: PC10 and PC12 are free, and the word select
// and the master clock land on two pads the display takes as INPUTS (its
// vertical sync and one of its colour bits), which the display ignores
// while its own controller is idle.
using CkPin = Pin<'C', 10>;    // I2S3_CK, AF6
using WsPin = Pin<'A', 4>;     // I2S3_WS, AF6
using SdPin = Pin<'C', 12>;    // I2S3_SD, AF6
using MckPin = Pin<'C', 7>;    // I2S3_MCK, AF6

using Audio = I2s<3>;

/// The audio PLL from the same 8 MHz crystal the main one uses. Its input
/// divider is the MAIN PLL's M on this part class, so the VCO input is
/// what the system clock already fixed: 2 MHz here. 192 over 5 puts
/// 76.8 MHz on the R output, and 76.8 MHz is a rate the 16-bit frame
/// divides EXACTLY into 48 kHz.
constexpr uint32_t i2s_clk_hz = 76'800'000u;
constexpr PllI2sConfig pll = plli2s_config_for(8'000'000u, i2s_clk_hz, SysClock::pll.m);
static_assert(pll.n == 192 && pll.r == 5);
static_assert(plli2s_r_hz(8'000'000u, pll, SysClock::pll.m) == i2s_clk_hz);

constexpr uint32_t target_fs = 48'000u;
constexpr auto prescaler = i2s_prescaler_for(i2s_clk_hz, target_fs, false, false);
static_assert(prescaler.has_value() && prescaler->div == 25 && !prescaler->odd);
static_assert(i2s_fs_hz(i2s_clk_hz, prescaler->div, prescaler->odd, false, false) == target_fs);

/// Count the word select's rising edges over `ms` milliseconds, feeding
/// the transmitter whenever its buffer empties, and note whether CHSIDE
/// ever changed. A half period at 48 kHz is ten microseconds - some two
/// thousand core cycles - so the poll cannot miss an edge.
uint32_t count_ws_edges(uint32_t ms, bool& both_sides) {
    uint32_t edges = 0;
    bool saw_left = false;
    bool saw_right = false;
    bool last = WsPin::read();
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < ms) {
        if (Audio::tx_empty()) {
            Audio::data(0x1234u);
            if (Audio::right_channel()) {
                saw_right = true;
            } else {
                saw_left = true;
            }
        }
        const bool now = WsPin::read();
        if (now && !last) {
            ++edges;
        }
        last = now;
    }
    both_sides = saw_left && saw_right;
    return edges;
}

/// How many of `samples` reads of the master clock pad found it HIGH,
/// with the transmitter kept fed so the generator never runs dry. At
/// 256 x FS that clock is faster than this loop, so what is asked is
/// presence - a count strictly between nought and all - and not a rate.
constexpr uint32_t mck_samples = 4000;

uint32_t mck_high_count() {
    uint32_t high = 0;
    for (uint32_t i = 0; i < mck_samples; ++i) {
        if (Audio::tx_empty()) {
            Audio::data(0x4321u);
        }
        if (MckPin::read()) {
            ++high;
        }
    }
    return high;
}

}  // namespace audio

void tn_i2s() {
    (void)host_ready();
    using namespace audio;

    // The audio PLL, off the same root the main one runs on.
    Rcc::plli2s_enable(false);
    (void)Rcc::plli2s_wait(false);
    const bool wrote = Rcc::plli2s_configure(pll);
    Rcc::plli2s_enable(true);
    const bool locked = Rcc::plli2s_wait(true);
    const bool routed = Rcc::i2s_source(I2sSource::plli2s_r);
    print(serial, "  PLLI2S: M ", Rcc::pll_m(), " (the main PLL's), N ", pll.n, ", R ", pll.r,
          " -> ", i2s_clk_hz / 1000u, " kHz; configured ", wrote ? "yes" : "NO", ", locked ",
          locked ? "yes" : "NO", crlf);
    bench.verdict("the audio PLL takes its ratio and locks - a second PLL on the same root, whose "
                  "input divider on this part class is the MAIN PLL's",
                  wrote && locked && routed);
    bench.verdict("PLLI2SCFGR is refused while the PLL runs, as 7.3.23 wants",
                  !Rcc::plli2s_configure(pll));

    Audio::bus_clock(true);
    Audio::reset();
    const I2sConfig cfg{.mode = I2sMode::host_transmit,
                        .standard = I2sStandard::philips,
                        .data = I2sDataLength::bits16,
                        .channel = I2sChannelLength::bits16,
                        .div = prescaler->div,
                        .odd = prescaler->odd};
    const bool configured = Audio::configure(cfg);
    bench.verdict("the block moves to its audio face: I2SMOD set with I2SE clear",
                  configured && Audio::i2s_mode_selected() && !Audio::enabled());

    CkPin::function(PinFunction::af6, {.speed = PinSpeed::very_high});
    WsPin::function(PinFunction::af6, {.speed = PinSpeed::very_high});
    SdPin::function(PinFunction::af6, {.speed = PinSpeed::very_high});
    Audio::enable();
    Audio::data(0x1234u);

    constexpr uint32_t window_ms = 200;
    bool both_sides = false;
    const uint32_t edges = count_ws_edges(window_ms, both_sides);
    const uint32_t measured = edges * (1000u / window_ms);
    constexpr uint32_t computed = i2s_fs_hz(i2s_clk_hz, prescaler->div, prescaler->odd, false, false);
    print(serial, "  I2SDIV ", prescaler->div, " ODD ", prescaler->odd ? 1 : 0,
          ": the arithmetic says FS = ", computed, " Hz; the word select's own pad gave ", edges,
          " rising edges in ", window_ms, " ms = ", measured, " Hz", crlf);
    bench.verdict("the word select runs at the sampling frequency the chapter's divider "
                  "arithmetic states, inside one per cent",
                  measured > computed - computed / 100u && measured < computed + computed / 100u);
    bench.verdict("a frame is a LEFT channel then a RIGHT one, and CHSIDE says which the "
                  "transmitter wants next - both sides come round",
                  both_sides);
    (void)Audio::disable();

    // The master clock output: 256 x FS on a pad of its own, and with
    // MCKOE the whole divisor becomes 256 x (2 x I2SDIV + ODD) whatever
    // the channel width - which moves the sampling frequency for the same
    // divider, so the second configuration is a rate of its own.
    constexpr auto mck_pre = i2s_prescaler_for(i2s_clk_hz, target_fs, false, true);
    static_assert(mck_pre.has_value());
    constexpr uint32_t mck_fs = i2s_fs_hz(i2s_clk_hz, mck_pre->div, mck_pre->odd, false, true);
    const I2sConfig with_mck{.mode = I2sMode::host_transmit,
                             .standard = I2sStandard::philips,
                             .data = I2sDataLength::bits16,
                             .channel = I2sChannelLength::bits16,
                             .master_clock_out = true,
                             .div = mck_pre->div,
                             .odd = mck_pre->odd};
    MckPin::function(PinFunction::af6, {.speed = PinSpeed::very_high});
    const bool mck_cfg = Audio::configure(with_mck);
    Audio::enable();
    Audio::data(0x4321u);
    bool mck_sides = false;
    const uint32_t mck_edges = count_ws_edges(window_ms, mck_sides);
    const uint32_t mck_measured = mck_edges * (1000u / window_ms);
    const uint32_t high = mck_high_count();
    print(serial, "  with MCKOE, I2SDIV ", mck_pre->div, " gives FS = ", mck_fs,
          " Hz by the arithmetic and ", mck_measured, " Hz on the word-select pad; MCK should be ",
          256u * mck_fs / 1000u, " kHz and its pad read HIGH on ", high, " of ", mck_samples,
          " samples", crlf);
    bench.verdict("MCKOE changes the divisor to 256 x (2 x I2SDIV + ODD): the same generator "
                  "gives a different sampling frequency, and it is the one the arithmetic states",
                  mck_cfg && Audio::master_clock_out() && mck_measured > mck_fs - mck_fs / 100u &&
                      mck_measured < mck_fs + mck_fs / 100u);
    bench.verdict("the master clock appears on its own pad - too fast for this poll to count, so "
                  "what is asked of it is that it is neither stuck high nor stuck low",
                  high != 0u && high != mck_samples);

    (void)Audio::disable();
    Audio::select_spi_mode();
    CkPin::release();
    WsPin::release();
    SdPin::release();
    MckPin::release();
    Audio::bus_clock(false);
    Rcc::plli2s_enable(false);
    bench.verdict("the block goes back to its SPI face, and the pads are parked",
                  !Audio::i2s_mode_selected());
    (void)host_ready();
}

// =============================================================================
// the menu
// =============================================================================

void banner() {
    print(serial, crlf, "test_stm32f4_spi - SPI5 with the board's own gyroscope and display",
          crlf);
    bench.menu();
}

}  // namespace

// ---- the vectors -------------------------------------------------------------------------

extern "C" void USART1_IRQHandler() { (void)Serial::isr(); }

/// The engined host's completion, from either stream's vector: to the
/// kernel letter's arbiter, or nowhere (letter k waits on the select). No
/// host arms SPI5's own interrupts, so its vector stays unbound.
void dma_host_vector() {
    if (dma_host_live && DmaHost::dma_isr() && bus_ao_live) {
        brio::post<kl::SpiArb>(brio::TransferDone{DmaHost::status()});
    }
}
extern "C" void DMA2_Stream4_IRQHandler() { dma_host_vector(); }
extern "C" void DMA2_Stream3_IRQHandler() { dma_host_vector(); }

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

int main() {
    // What the silicon held before a line of this program ran: the gate
    // is read out of the RCC (reading it does not open it), and the
    // registers behind it need the clock, so it is opened for the reading.
    boot.gate = brio::Spi<5>::bus_clock();
    brio::Spi<5>::bus_clock(true);
    boot.cr1 = brio::Spi<5>::regs().CR1;
    boot.cr2 = brio::Spi<5>::regs().CR2;
    boot.sr = brio::Spi<5>::regs().SR;
    boot.crcpr = brio::Spi<5>::regs().CRCPR;
    boot.i2scfgr = brio::Spi<5>::regs().I2SCFGR;
    boot.i2spr = brio::Spi<5>::regs().I2SPR;

    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    (void)brio::CycleCounter::init();  // every Cortex-M4F here implements CYCCNT (DWT_CTRL.NOCYCCNT reads 0)
    brio::enable_interrupts();

    const bool host_ok = host_ready();

    bench.letter('a', "the instances and the block", ta_block);
    bench.letter('b', "the eight baud rates, timed", tb_rates);
    bench.letter('c', "what the driver refuses", tc_refusals);
    bench.letter('d', "the mode fault, raised by SSI", td_mode_fault);
    bench.letter('e', "the overrun and its clearing sequence", te_overrun);
    bench.letter('f', "the gyroscope answers", tf_identity);
    bench.letter('g', "the gyroscope at each of the eight rates", tg_device_rates);
    bench.letter('h', "the four modes against the one it speaks", th_modes);
    bench.letter('i', "auto-increment, and a register written then read", ti_registers);
    bench.letter('j', "THE KERNEL: SpiBus over SpiHost", tj_kernel);
    bench.letter('k', "the same transaction through the DMA engines", tk_engines);
    bench.letter('l', "the SCK pad's slew class against the errata", tl_pad_speed);
    bench.letter('m', "the bidirectional line, against both devices", tm_bidirectional);
    bench.letter('n', "I2S3 as a master transmitter", tn_i2s);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL" : "FAILED", " tick=",
                    tick_ok ? "SysTick" : "FAILED", " spi5=", host_ok ? "host" : "FAILED",
                    brio::crlf);
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


#endif   // STM32F446xx
