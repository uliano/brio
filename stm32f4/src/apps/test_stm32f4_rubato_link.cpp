// test_stm32f4_rubato_link - the bench suite of the SPI and UART stacks of
// the STM32F4 stratum under Rubato, the preemptive kernel
// (kernel/rubato.hpp): util/spi_bus.hpp's arbiter over stm32f4/spi.hpp's
// SpiHost and its DMA engines, and util/serial_port.hpp's SerialPort over
// stm32f4/usart.hpp's Uart and its receive engine, run against the desk's
// STM32G0B1RE and its unchanged instruments (stm32g0/src/apps/spi_peer.cpp,
// usart_peer.cpp) while an object of the highest priority preempts them at
// moments the traffic does not choose. Every verdict is the traffic judged
// byte-exact, as the Tenuto suites (test_stm32f4_spi, test_stm32f4_serial)
// judge it; nothing here changes a driver or a service.
//
// A test_<target>_<subject> suite in the bench grammar (util/testbench.hpp)
// that runs inside the kernel it tests, as test_stm32f4_rubato does: the
// program is one Rubato pack, from the highest priority down -
//
//   Pressure  the preempting object: TIM2's update interrupt posts to it
//             every 97, 193 or 389 us (primes, against every period of the
//             traffic), and its dispatch spins 300 or 3000 cycles, so it
//             lands inside whatever the objects below were doing. It counts
//             its dispatches; the interrupt records where it landed (the
//             ceiling it found: which object's dispatch, the walk of the
//             time events with the ceiling held at 0 - a tick there waits
//             for the walk's end -, and whether the arbiter was dispatching
//             a completion at any depth).
//   LineSink  the borrower of the SerialPort's lines (Lease::dispatch),
//             above the port as the loan requires: each line parsed against
//             the sequence this suite numbers and checks, gaps, duplicates
//             and corrupt lines counted, then written over in place (the
//             borrower may write what it is lent; a lender that read it
//             after the post would find the scribble).
//   Client    the SPI client: it takes one select window from the Console
//             (the Window event), posts the host's Request to the arbiter
//             with the select in it and the protocol's 30 us between the
//             select's fall and the first clock, and on the reply decodes
//             the peer's checksummed answer frame (spi_link.hpp) out of the
//             bytes that came back. Every command window comes settle_ms
//             after the window before it, an answer window's included
//             (spi_link.hpp: "between one chip-select window and the next");
//             the peer spends that time going dark again, and a command
//             frame clocked sooner after an answer window is not heard.
//   SpiArb    util/spi_bus.hpp's SpiBus over SpiHost<2, link_pins> with the
//             DMA engines the F4 suite uses on the link (DMA1 streams 4 and
//             3), pending depth 4, a per-bus timeout of 100 ticks. In the
//             pack it is a type derived from the SpiBus whose dispatch notes
//             the event kind around the service's own dispatch - the one
//             thing the landing trace needs that the service does not say.
//   Minor     a second SPI client BELOW the arbiter (letter w).
//   LinePort  SerialPort<Uart<1, ...>, P, LineSink, 80> over the receive
//             engine the serial suite uses on the link (DMA2 stream 2
//             channel 4), the interrupt transmitter beside it.
//   Console   the lowest: it owns USART2 (the ST-LINK virtual COM port),
//             runs the letters in its own dispatch, and is the only object
//             that prints. It is also USART1's ONE TRANSMIT WRITER - the
//             peer's command frames on the polled resource between legs and
//             the numbered lines through the Uart's write_bulk() inside them:
//             the object that sequences the letters is the one that knows
//             when the instance is the command channel and when it is the
//             Uart task, so no hand-over of the transmitter exists to get
//             wrong, and a writer at the bottom of the pack never holds the
//             port or the sink off its receive side.
//
// A letter's waits are the Console's: each turn of a wait is Rubato's base
// turn written out (the matured time events posted with the ceiling held at
// 0, then the scheduler), because the kernel loop is below the Console and
// does not turn while a letter runs - so the arbiter's per-bus timeout is a
// live clock here. The objects' tallies are the one memory the Console
// reads outside events: each written by one object, read by the Console
// after the traffic that wrote it has been answered.
//
// THE WIRES, fixed on the desk (test_stm32f4_spi's and test_stm32f4_serial's
// two-board sets):
//   SPI2   SCK PB13 -> G0 PB3, MISO PB14 <- G0 PB4, MOSI PB15 -> G0 PB5,
//          NSS PB12 (a GPIO select) -> G0 PA15; SCK and MOSI at medium (the
//          F4 suite's measured classes for these jumper wires)
//   USART1 PA9 (TX) -> G0 PA10, PA10 (RX) <- G0 PA9
// The SPI letters (key z) want the peer running `spi_peer`, the UART ones
// (key y) `usart_peer`. NOTHING LIT: no letter drives the LED's pad.
//
// THE PASSES. Every letter runs its traffic seven times: a CONTROL with
// TIM2 silent, then under pressure at 97, 193 and 389 us with a 300-cycle
// spin, and again with a 3000-cycle spin. Every pass is judged on its own;
// a pass under pressure that fails where its control passes is a finding
// about the service, printed with its counters. A pressure pass also judges
// the pressure itself: TIM2 counts at 90 MHz from the crystal the core's
// 180 MHz comes from, so over a pass of T core cycles it interrupts
// floor(T / (180 x period_us)) times, plus or minus one for the two ends of
// the window, and each interrupt is one dispatch, none dropped by
// Pressure's queue (its dispatch, a spin of at most 3000 cycles and the
// scheduler's few hundred, is shorter than the shortest period's 17460).
//
// The letters (z: n o p w i; y: t u v):
//   n  the peer's command channel through the arbiter: a ping, ident (the
//      sanity byte and the STM32G0 port's kind), and ten more pings, every
//      frame a DMA transaction at PCLK1/256 and its answer decoded by the
//      Client.
//   o  the matrix of test_stm32f4_spi's letter o on the DMA engines (32
//      frames a window at PCLK1/32, against the peer's DMA serve): the four
//      modes in 8- and 16-bit frames, LSb first in both widths, and a DORD
//      mismatch as an exact two-way bit reversal - byte-exact at both ends;
//      the data windows' bytes over their cycles printed against the
//      control's.
//   p  letter p's ladders through the arbiter, every BR code from PCLK1/256
//      to PCLK1/2: the POLLED loop (8 frames; pump_read_one where a frame
//      is shorter than write_ahead_min_frame_cycles, pump_read_two - two
//      frames in flight - from /8 down), the RXNE PUMP (16 16-bit frames
//      from an odd address, which keeps them off the engines; two in
//      flight where Host::two_in_flight() says, /8 and /4), and the DMA
//      engines (32 frames). The polled loop runs inside the arbiter's
//      dispatch, so a dispatch above the arbiter holds it off for its whole
//      length, which the host's hold_off_cycles (the image's longest handler
//      or masked window) does not count: where two frames are in flight and
//      the preemption outlasts a frame, the transaction ends spi_overrun -
//      the service seeing it, never handing back a run with a hole in it.
//   w  the arbiter under pressure: the Client above it (20 requests of 6
//      frames on the engines, two outstanding), Minor below it (10 POLLED
//      requests of 8 frames, one outstanding) and, in a pressure pass,
//      Pressure itself (8 requests of 3 frames on the pump, posted from its
//      tick, one outstanding), all at PCLK1/256 inside ONE select window the
//      letter holds - the peer's exchange is one burst (test_stm32f4_spi's
//      letter q) - of constant frames (pattern_fixed, so the wire does not
//      care which requester's stretch came first). Every reply comes back
//      to its requester in its order
//      (at the Client's reply k its window k is complete and its window
//      k + 1 untouched: a requester above the arbiter is answered before
//      the next transaction starts), every one spi_ok, nothing rejected,
//      the per-bus timeout never firing (no spi_timeout, no stale event),
//      and the peer counts every frame exact. Minor's polled windows keep
//      two frames in flight at a frame of 8192 cycles, longer than any
//      preemption here (a 3000-cycle spin and the scheduler).
//   i  dark: every object idle, TIM2 stopped and its line quiet, no time
//      event armed, SPI2 idle with the select high and no DMA request
//      raised, USART1 given back, the LED's pad not an output, the ceiling
//      the Console's own.
//   t  the UART, received: numbered, checked lines (L, five digits, a
//      payload of 8 to 60 characters derived from the number, a checksum)
//      written by the Console and turned around by the peer's ECHO at
//      460800 and 921600 baud, assembled by the SerialPort from the receive
//      engine's ring and lent to LineSink: 200 lines a leg, none lost, none
//      out of order, none corrupt, the port's own counters (ring overruns,
//      ORE, FE, PE, NE, DMA faults, line overflows, torn lines) zero, and
//      the peer's count of the bytes it echoed exact. The writer leaves two
//      frame times after its own last stop bit before the next line: the
//      peer's echo runs at its own divisor of its HSI, up to 0.7 % slower
//      than this end at 921600, and over a line of at most 71 characters
//      that is half a character of backlog, drained by the pause.
//   u  the UART, sent: 300 numbered lines back to back through write_bulk()
//      into the peer's SINK at 460800 and 921600, judged by the peer's
//      report - the bytes counted, their 16-bit sum, FE, PE and ORE zero -
//      and the wire's time printed against the control's.
//   v  the loan under preemption: t's leg at 921600 with LineSink spinning
//      6000 cycles a line, so Pressure lands inside the borrower's dispatch,
//      and the lines STREAMED - line k written once line k - 20 is back -
//      so that the port is told at the receive ring's half lap with the
//      characters still coming and lends its lines while the next ones
//      arrive (the receive stream's count read at the sink's entry and exit
//      says how often it moved; the window's arithmetic beside it): every
//      line exact, every line lent alone (the sink's queue empty at its
//      entry, the sink never nested in itself), the two buffers strictly
//      alternating - the lender re-enters its drain with both buffers free,
//      its in-flight count back at zero - and the lender never reading what
//      the borrower wrote over. A pressure pass judges that Pressure landed
//      inside the borrower at least once: the sink's dispatches add up to
//      200 x 6000 = 1.2 M cycles at least, which is 17 ticks of the longest
//      period (70020 cycles) on average, so a pass with none has a chance
//      of e^-17.
//
// build: boards = f446re
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>
#include <variant>

#include "kernel/borrowed.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/post.hpp"
#include "kernel/rubato.hpp"
#include "kernel/time_event.hpp"
#include "stm32f4/clock.hpp"
#include "stm32f4/dma.hpp"
#include "stm32f4/dwt.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/spi.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/tim.hpp"
#include "stm32f4/usart.hpp"
#include "util/print.hpp"
#include "util/serial_port.hpp"
#include "util/spi_bus.hpp"
#include "util/testbench.hpp"

// THE PROTOCOL HEADERS ARE SHARED, NOT COPIED: the far board's two
// instruments compile the same files.
#include "../../../avrxt/src/apps/spi_link.hpp"
#include "../../../avrxt/src/apps/usart_link.hpp"

/// The Nucleo's HSE is the ST-LINK's 8 MHz clock, taken in bypass; 180 MHz
/// puts APB1 at 45 MHz (SPI2's BR divides it) and APB2 at 90 (USART1's).
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000, brio::HseMode::bypass>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Stm32f4RubatoPlatform<>;
using Ruler = CycleCounter;
constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000u;

// ---- the console -------------------------------------------------------------------

constexpr UartPins console_pins{.tx = {'A', 2, PinFunction::af7}, .rx = {'A', 3, PinFunction::af7}};
using Serial = Uart<2, console_pins>;
constexpr Serial serial;
TwoKeyTestBench<Serial, 'y'> bench;

// ---- the SPI link ------------------------------------------------------------------

constexpr SpiPins link_pins{
    .sck = {'B', 13, PinFunction::af5},
    .miso = {'B', 14, PinFunction::af5},
    .mosi = {'B', 15, PinFunction::af5},
    .nss = {'B', 12, PinFunction::af5},
};
/// SPI2_TX is DMA1 stream 4 channel 0 and SPI2_RX DMA1 stream 3 channel 0
/// (RM0390 table 28); the element is DR's width, so 16-bit frames ride the
/// engines a half-word a beat.
using TxEngine = DmaTxEngine<1, 4, 0, uint16_t>;
using RxEngine = DmaRxEngine<1, 3, 0, uint16_t>;
using Host = SpiHost<2, link_pins, TxEngine, RxEngine>;
using CsPin = Pin<link_pins.nss.port, link_pins.nss.pin>;

// ---- the UART link -----------------------------------------------------------------

constexpr UartPins uart_link_pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
constexpr UartOptions uart_link_opts{.tx_speed = PinSpeed::very_high};
/// USART1_RX on DMA2 stream 2 channel 4 (RM0390 table 29): test_stm32f4_serial's
/// LinkDma, its ring and its transmit ring.
using LinkRxEngine = DmaRxEngine<2, 2, 4>;
using LinkPort = Uart<1, uart_link_pins, 2048, 64, NoDmaEngine, LinkRxEngine, uart_link_opts>;
/// The Uart task owns USART1's two vectors only while a leg runs.
volatile bool link_live = false;

// ---- the pressure's clock ----------------------------------------------------------

/// TIM2: 32 bits on APB1, its timer clock 2 x PCLK1 = 90 MHz - a timer no
/// stack under test touches.
using Pulse = Tim<2>;
constexpr uint32_t pulse_per_us = Pulse::clock_hz(SysClock{}) / 1'000'000u;
static_assert(pulse_per_us == 90u, "TIM2 counts at 90 MHz at this clock");

[[gnu::always_inline]] inline void spin(uint32_t cycles) {
    const uint32_t t0 = Ruler::now();
    while (Ruler::now() - t0 < cycles) {
    }
}

// =============================================================================
// The pack's shared vocabulary and the landing trace
// =============================================================================

struct Tick {};
struct Kick {};

/// The pack's indices, from the top (the Rubato list below).
constexpr uint8_t pressure_index = 0;
constexpr uint8_t sink_index = 1;
constexpr uint8_t client_index = 2;
constexpr uint8_t arb_index = 3;
constexpr uint8_t minor_index = 4;
constexpr uint8_t port_index = 5;
constexpr uint8_t console_index = 6;
constexpr uint8_t landing_slots = 8;   ///< the seven indices, and the loop's base (255)

/// Where TIM2's interrupt found the kernel: the ceiling it read, by slot.
volatile uint32_t landed[landing_slots];
/// ... with the arbiter dispatching a TransferDone at any depth, and of those
/// one a DMA completion posted; and with LineSink's dispatch on the stack.
volatile uint32_t landed_completion = 0;
volatile uint32_t landed_dma_completion = 0;
volatile uint32_t landed_sink_any = 0;
volatile uint32_t pressure_irqs = 0;

/// The arbiter's event kind while its dispatch runs (SpiArb below).
constexpr uint8_t arb_none = 0;
constexpr uint8_t arb_request = 1;
constexpr uint8_t arb_done = 2;
constexpr uint8_t arb_other = 3;
volatile uint8_t arb_event = arb_none;
/// The source of the TransferDone the arbiter dispatches: true the DMA
/// streams' completion, false the SPI vector's (the pump's).
volatile bool done_from_dma = false;
/// LineSink's dispatch is running (it never nests in itself).
volatile uint8_t sink_depth = 0;

// =============================================================================
// The SPI requests of letter w, shared by its three requesters
// =============================================================================

namespace w {

constexpr uint8_t seed_a = 0x5A;   ///< what every window sends (pattern_fixed)
constexpr uint8_t seed_b = 0xC3;   ///< what the peer answers every frame
constexpr uint8_t untouched = 0xEE;

constexpr uint8_t client_n = 20;
constexpr uint8_t client_len = 6;
constexpr uint8_t minor_n = 10;
constexpr uint8_t minor_len = 8;
constexpr uint8_t press_n = 8;
constexpr uint8_t press_len = 3;

alignas(4) uint8_t tx[8] = {seed_a, seed_a, seed_a, seed_a, seed_a, seed_a, seed_a, seed_a};
alignas(4) uint8_t client_rx[client_n][client_len];
alignas(4) uint8_t minor_rx[minor_n][minor_len];
alignas(4) uint8_t press_rx[press_n][press_len];

volatile bool live = false;
volatile bool press_live = false;   ///< Pressure posts its requests (a pressure pass)

struct Tally {
    uint8_t posted = 0;
    uint8_t replies = 0;
    uint8_t ok = 0;
    uint8_t out_of_order = 0;
    uint8_t timeouts = 0;
    uint8_t rejected = 0;
};
volatile Tally client{};
volatile Tally minor{};
volatile Tally press{};

void clear(volatile Tally& t) {
    t.posted = 0;
    t.replies = 0;
    t.ok = 0;
    t.out_of_order = 0;
    t.timeouts = 0;
    t.rejected = 0;
}

bool complete(const uint8_t* slot, uint8_t len) {
    for (uint8_t i = 0; i < len; ++i) {
        if (slot[i] != seed_b) {
            return false;
        }
    }
    return true;
}
bool fresh(const uint8_t* slot, uint8_t len) {
    for (uint8_t i = 0; i < len; ++i) {
        if (slot[i] != untouched) {
            return false;
        }
    }
    return true;
}

/// One of letter w's transactions. THE SELECT IS THE LETTER'S, held low
/// across the whole exchange (test_stm32f4_spi's letter q): the peer's
/// exchange is one burst, so every requester's transaction is a stretch of
/// one window, and every one runs at the window's mode and rate - a rate
/// change is a disable and an enable of the block, which a selected client
/// would see.
Host::Request request(uint8_t* rx, uint8_t len, bool polled, ReplyTo<SpiDone> reply) {
    Host::Request r{};
    r.cs = {};
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx));
    r.rx = lend<Lease::reply>(rx);
    r.len = len;
    r.bits = SpiDataSize::bits8;
    r.polled = polled;
    r.cs_setup_us = 0;
    r.clock = SpiClock::div256;
    r.mode = SpiMode::mode0;
    r.reply = reply;
    return r;
}

/// One reply's bookkeeping: the status, and whether the requester's window
/// k is complete (and, `next_fresh`, its window k + 1 untouched).
void note(volatile Tally& t, const SpiDone& d, bool in_order) {
    if (d.status == spi_ok) {
        t.ok = static_cast<uint8_t>(t.ok + 1u);
    } else if (d.status == spi_timeout) {
        t.timeouts = static_cast<uint8_t>(t.timeouts + 1u);
    } else if (d.status == spi_rejected) {
        t.rejected = static_cast<uint8_t>(t.rejected + 1u);
    }
    if (!in_order) {
        t.out_of_order = static_cast<uint8_t>(t.out_of_order + 1u);
    }
    t.replies = static_cast<uint8_t>(t.replies + 1u);
}

}  // namespace w

// =============================================================================
// SpiArb: the arbiter, its event kind noted for the trace
// =============================================================================

/// The bus: pending depth 4 (the three requesters of letter w keep at most
/// four outstanding: one in flight, three waiting), and a per-bus timeout of
/// 100 ticks - 100 ms, two orders of magnitude above the longest transaction
/// here (48 frames at PCLK1/256, 2.2 ms).
using ArbBase = SpiBus<Host, P, 4, BusPassThrough, 100>;

/// The service as it is; the pack names this type so that its dispatch can
/// say, for the landing trace, which kind of event the service is
/// dispatching. The queue, init() and every verb are the SpiBus's.
struct SpiArb : ArbBase {
    static void dispatch(const Event& e) {
        arb_event = std::holds_alternative<TransferDone>(e)    ? arb_done
                    : std::holds_alternative<Host::Request>(e) ? arb_request
                                                               : arb_other;
        ArbBase::dispatch(e);
        arb_event = arb_none;
    }
};

// =============================================================================
// Pressure
// =============================================================================

volatile uint32_t pressure_spin = 0;

struct Pressure {
    using Event = std::variant<Tick, SpiDone>;
    static inline EventQueue<Event, 4, P> queue;
    static inline volatile uint32_t runs = 0;
    static inline volatile bool outstanding = false;

    static void init() {}
    static void dispatch(const Event& e) {
        match(
            e,
            [](const Tick&) {
                runs = runs + 1u;
                if (w::press_live && !outstanding && w::press.posted < w::press_n) {
                    outstanding = true;
                    const uint8_t k = w::press.posted;
                    w::press.posted = static_cast<uint8_t>(k + 1u);
                    post<SpiArb>(w::request(w::press_rx[k], w::press_len, false,
                                            reply_to<Pressure, SpiDone>()));
                }
                spin(pressure_spin);
            },
            [](const SpiDone& d) {
                const uint8_t k = w::press.replies;
                w::note(w::press, d, k < w::press_n && w::complete(w::press_rx[k], w::press_len));
                outstanding = false;
            });
    }
};

// =============================================================================
// LineSink: the borrower of the port's lines
// =============================================================================

/// Line `seq`, without its end of line: 'L', five digits, ':', a payload of
/// 8 to 60 characters from 'A' to 'z' (never ':', '\r' or '\n'), ':', and
/// the 8-bit sum of everything before it in two hex digits - 70 characters
/// at most, inside the assembler's 79.
constexpr uint8_t line_most = 72;
constexpr uint8_t payload_len(uint16_t seq) { return static_cast<uint8_t>(8u + (static_cast<uint32_t>(seq) * 13u) % 53u); }
constexpr char payload_char(uint16_t seq, uint8_t i) {
    return static_cast<char>('A' + (static_cast<uint32_t>(seq) * 7u + static_cast<uint32_t>(i) * 3u) % 58u);
}
constexpr char hex_digit(uint8_t v) { return static_cast<char>(v < 10u ? '0' + v : 'A' + (v - 10u)); }

uint8_t make_line(uint16_t seq, char* out) {
    uint8_t n = 0;
    out[n++] = 'L';
    uint32_t v = seq;
    for (int i = 4; i >= 0; --i) {
        out[n + static_cast<uint8_t>(i)] = static_cast<char>('0' + v % 10u);
        v /= 10u;
    }
    n = static_cast<uint8_t>(n + 5u);
    out[n++] = ':';
    const uint8_t len = payload_len(seq);
    for (uint8_t i = 0; i < len; ++i) {
        out[n++] = payload_char(seq, i);
    }
    out[n++] = ':';
    uint8_t sum = 0;
    for (uint8_t i = 0; i < n; ++i) {
        sum = static_cast<uint8_t>(sum + static_cast<uint8_t>(out[i]));
    }
    out[n++] = hex_digit(static_cast<uint8_t>(sum >> 4));
    out[n++] = hex_digit(static_cast<uint8_t>(sum & 0x0Fu));
    return n;
}

/// The line parsed: its number, when every character is what make_line()
/// writes for that number.
bool parse_line(const char* s, uint16_t& seq) {
    if (s[0] != 'L') {
        return false;
    }
    uint32_t v = 0;
    for (uint8_t i = 1; i <= 5; ++i) {
        if (s[i] < '0' || s[i] > '9') {
            return false;
        }
        v = v * 10u + static_cast<uint32_t>(s[i] - '0');
    }
    if (s[6] != ':' || v > 0xFFFFu) {
        return false;
    }
    const uint16_t q = static_cast<uint16_t>(v);
    const uint8_t len = payload_len(q);
    for (uint8_t i = 0; i < len; ++i) {
        if (s[7u + i] != payload_char(q, i)) {
            return false;
        }
    }
    const uint8_t at = static_cast<uint8_t>(7u + len);
    if (s[at] != ':') {
        return false;
    }
    uint8_t sum = 0;
    for (uint8_t i = 0; i <= at; ++i) {
        sum = static_cast<uint8_t>(sum + static_cast<uint8_t>(s[i]));
    }
    if (s[at + 1u] != hex_digit(static_cast<uint8_t>(sum >> 4)) ||
        s[at + 2u] != hex_digit(static_cast<uint8_t>(sum & 0x0Fu)) || s[at + 3u] != '\0') {
        return false;
    }
    seq = q;
    return true;
}

struct LineSink {
    using Event = std::variant<LineReceived>;
    /// Two LineReceived in flight at most (serial_port.hpp) and room beside.
    static inline EventQueue<Event, 4, P> queue;

    // The tally, written here, read by the Console between legs.
    static inline volatile uint32_t lines = 0;      ///< every LineReceived
    static inline volatile uint32_t good = 0;
    static inline volatile uint32_t empties = 0;    ///< the leg's leading end of line
    static inline volatile uint32_t corrupt = 0;
    static inline volatile uint32_t gaps = 0;       ///< jumps forward
    static inline volatile uint32_t lost = 0;       ///< the lines the jumps skipped
    static inline volatile uint32_t backwards = 0;  ///< duplicates or out of order
    static inline volatile uint32_t waiting = 0;    ///< another line already queued at entry
    static inline volatile uint32_t same_buffer = 0;   ///< a line in the buffer of the one before
    static inline volatile uint32_t nested = 0;
    static inline volatile uint32_t moved = 0;      ///< the receive stream wrote during the dispatch
    static inline volatile uint32_t busy = 0;       ///< cycles inside the dispatches
    static inline volatile uint32_t spin_cycles = 0;
    static inline uint16_t next = 0;
    static inline const char* last = nullptr;
    static inline const char* buffers[2] = {nullptr, nullptr};
    static inline volatile uint8_t distinct = 0;

    static void reset(uint16_t first) {
        lines = 0;
        good = 0;
        empties = 0;
        corrupt = 0;
        gaps = 0;
        lost = 0;
        backwards = 0;
        waiting = 0;
        same_buffer = 0;
        nested = 0;
        moved = 0;
        busy = 0;
        next = first;
        last = nullptr;
    }

    static void init() {}
    static void dispatch(const Event& e) {
        match(e, [](const LineReceived& l) { take(l.line.get()); });
    }

private:
    static void take(char* s) {
        const uint32_t t0 = Ruler::now();
        if (sink_depth != 0u) {
            nested = nested + 1u;
        }
        sink_depth = static_cast<uint8_t>(sink_depth + 1u);
        const uint32_t at = LinkRxEngine::remaining();
        if (!queue.empty()) {
            waiting = waiting + 1u;
        }
        if (s == last) {
            same_buffer = same_buffer + 1u;
        }
        last = s;
        if (s != buffers[0] && s != buffers[1]) {
            buffers[distinct & 1u] = s;
            distinct = static_cast<uint8_t>(distinct + 1u);
        }
        lines = lines + 1u;
        if (s[0] == '\0') {
            empties = empties + 1u;
        } else {
            uint16_t seq = 0;
            if (!parse_line(s, seq)) {
                corrupt = corrupt + 1u;
            } else if (seq == next) {
                good = good + 1u;
                next = static_cast<uint16_t>(seq + 1u);
            } else if (seq > next) {
                gaps = gaps + 1u;
                lost = lost + (seq - next);
                good = good + 1u;
                next = static_cast<uint16_t>(seq + 1u);
            } else {
                backwards = backwards + 1u;
            }
        }
        spin(spin_cycles);
        // The loan lets the borrower write the line: written over, so that
        // a lender reading it after the post would read this.
        for (char* p = s; *p != '\0'; ++p) {
            *p = 'X';
        }
        if (LinkRxEngine::remaining() != at) {
            moved = moved + 1u;
        }
        sink_depth = static_cast<uint8_t>(sink_depth - 1u);
        busy = busy + (Ruler::now() - t0);
    }
};

// =============================================================================
// Client: the SPI client of the letters n, o and p (and letter w's first)
// =============================================================================

/// One select window, the Console's: the buffers LENT until the Client says
/// it is done (Client::done moves), the bus's settings for this window.
struct Window {
    Borrowed<const uint8_t, Lease::reply> tx;
    Borrowed<uint8_t, Lease::reply> rx;
    uint16_t len = 0;
    SpiClock clock = SpiClock::div256;
    SpiMode mode = SpiMode::mode0;
    SpiDataSize bits = SpiDataSize::bits8;
    bool polled = false;
    bool lsb = false;
    bool decode = false;   ///< look for a spi_link frame in what came back
};

struct Client {
    using Event = std::variant<Window, SpiDone, Kick>;
    static inline EventQueue<Event, 8, P> queue;

    static inline volatile uint32_t done = 0;    ///< windows answered
    static inline volatile uint8_t status = spi_ok;
    static inline volatile uint32_t cycles = 0;  ///< the last window, post to reply
    static inline uint32_t t_post = 0;
    static inline uint8_t* rx = nullptr;
    static inline uint16_t rx_len = 0;
    static inline bool decode = false;
    static inline volatile bool found = false;   ///< the last answer window held a frame
    static inline spilink::Frame frame{};
    static inline uint8_t raw[16];
    static inline uint8_t raw_n = 0;

    static void init() {}
    static void dispatch(const Event& e) {
        match(
            e,
            [](const Window& win) {
                // The bit order is a bus-level verb (SpiHost::bit_order), and
                // the bus is idle: the Console posts a window only after the
                // last one was answered, and in these letters the Client is the
                // bus's one requester.
                if (Host::lsb_first() != win.lsb) {
                    (void)Host::bit_order(win.lsb);
                }
                rx = win.rx.get();
                rx_len = win.len;
                decode = win.decode;
                Host::Request r{};
                r.cs = CsPin::ref();
                r.tx = win.tx;
                r.rx = win.rx;
                r.len = win.len;
                r.bits = win.bits;
                r.polled = win.polled;
                // The 30 us the protocol pays between the select's fall and
                // the first clock (test_stm32f4_spi's link_hold()).
                r.cs_setup_us = 30;
                r.clock = win.clock;
                r.mode = win.mode;
                r.reply = reply_to<Client, SpiDone>();
                t_post = Ruler::now();
                post<SpiArb>(r);
            },
            [](const SpiDone& d) {
                if (w::live) {
                    const uint8_t k = w::client.replies;
                    const bool in_order = k < w::client_n && w::complete(w::client_rx[k], w::client_len) &&
                                          (k + 1u >= w::client_n || w::fresh(w::client_rx[k + 1u], w::client_len));
                    w::note(w::client, d, in_order);
                    post_next_w();
                    return;
                }
                cycles = Ruler::now() - t_post;
                status = d.status;
                found = false;
                if (decode && rx != nullptr) {
                    judge_answer();
                }
                done = done + 1u;
            },
            [](const Kick&) {
                post_next_w();
                post_next_w();
            });
    }

private:
    /// The peer's answer frame out of the bytes that came back (the peer
    /// pads with zeros), and the first sixteen for a failure's print.
    static void judge_answer() {
        spilink::Decoder dec;
        raw_n = 0;
        for (uint16_t i = 0; i < rx_len; ++i) {
            if (raw_n < sizeof raw) {
                raw[raw_n++] = rx[i];
            }
            if (dec.feed(rx[i]) == spilink::Decoder::Result::frame) {
                frame = dec.frame();
                found = true;
                return;
            }
        }
    }

    static void post_next_w() {
        const uint8_t k = w::client.posted;
        if (k >= w::client_n) {
            return;
        }
        w::client.posted = static_cast<uint8_t>(k + 1u);
        post<SpiArb>(w::request(w::client_rx[k], w::client_len, false,
                                reply_to<Client, SpiDone>()));
    }
};

// =============================================================================
// Minor: letter w's requester below the arbiter
// =============================================================================

struct Minor {
    using Event = std::variant<Kick, SpiDone>;
    static inline EventQueue<Event, 4, P> queue;

    static void init() {}
    static void dispatch(const Event& e) {
        match(
            e, [](const Kick&) { post_next(); },
            [](const SpiDone& d) {
                const uint8_t k = w::minor.replies;
                bool in_order = k < w::minor_n;
                for (uint8_t i = 0; in_order && i <= k; ++i) {
                    in_order = w::complete(w::minor_rx[i], w::minor_len);
                }
                w::note(w::minor, d, in_order);
                post_next();
            });
    }

private:
    /// POLLED at PCLK1/256: the request completes inside the arbiter's
    /// dispatch, which runs inside this post (the arbiter is above Minor).
    static void post_next() {
        const uint8_t k = w::minor.posted;
        if (k >= w::minor_n) {
            return;
        }
        w::minor.posted = static_cast<uint8_t>(k + 1u);
        post<SpiArb>(w::request(w::minor_rx[k], w::minor_len, true,
                                reply_to<Minor, SpiDone>()));
    }
};

// =============================================================================
// LinePort, Console, the kernel
// =============================================================================

using LinePort = SerialPort<LinkPort, P, LineSink, 80>;

struct ConsoleEvent {
    uint8_t kind;
};
constexpr uint8_t console_rx = 0;

void console_banner();
void console_turn();

struct Console {
    using Event = ConsoleEvent;
    static inline EventQueue<Event, 4, P> queue;
    static void init() {
        console_banner();
        bench.prompt();
    }
    static void dispatch(const Event& e) {
        if (e.kind == console_rx) {
            console_turn();
        }
    }
};

using K = Rubato<P, Pressure, LineSink, Client, SpiArb, Minor, LinePort, Console>;
using TE = TimeEvents<P>;

/// One turn of Rubato::run()'s loop, written out in the Console's dispatch:
/// the matured time events posted with the ceiling held at 0, then the
/// scheduler at the Console's ceiling.
void base_turn() {
    const uint8_t c = P::ceiling;
    P::ceiling = 0;
    TE::process();
    P::ceiling = c;
    (void)K::activate();
}

/// A wait's turn: the base turn, then 500 to 1523 cycles of the Console's
/// own, the length from an 8-bit LFSR - so that the walk with the ceiling at
/// 0 is a few per cent of a wait and not most of it (a tick landing in the
/// walk waits for its end), and so that the wait's own period never locks
/// to the pressure's and steers where it lands.
uint8_t wait_lfsr = 0xA5;
void wait_turn() {
    base_turn();
    const uint8_t v = wait_lfsr;
    wait_lfsr = static_cast<uint8_t>((v << 1) | (((v >> 7) ^ (v >> 5) ^ (v >> 4) ^ (v >> 3)) & 1u));
    spin(500u + 4u * static_cast<uint32_t>(v));
}

template <typename Pred>
bool wait_until(Pred done, uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (!done()) {
        if (Ticker::millis() - t0 >= ms) {
            return done();
        }
        wait_turn();
    }
    return true;
}

void wait_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
        wait_turn();
    }
}

void wait_us(uint32_t us) { spin(us * cycles_per_us); }

// =============================================================================
// The passes and the pressure
// =============================================================================

struct PassSpec {
    uint16_t period_us;   ///< 0: the control, TIM2 silent
    uint16_t spin;        ///< Pressure's dispatch, in core cycles
};
constexpr PassSpec passes[] = {{0, 0},      {97, 300},   {193, 300}, {389, 300},
                               {97, 3000}, {193, 3000}, {389, 3000}};
constexpr uint8_t pass_count = sizeof passes / sizeof passes[0];

struct PassStats {
    uint32_t wall = 0;        ///< core cycles, TIM2 on to off (the control: the pass)
    uint32_t irqs = 0;
    uint32_t runs = 0;
    uint16_t overflows = 0;
    uint32_t land[landing_slots] = {};
    uint32_t completion = 0;
    uint32_t dma_completion = 0;
    uint32_t sink_any = 0;
};

uint32_t pass_t0 = 0;
uint32_t pass_runs0 = 0;
uint16_t pass_overflows0 = 0;

char label_buf[28];

/// "control: " or "97 us x 300: ", the prefix of a pass's verdicts.
const char* pass_label(const PassSpec& s) {
    uint8_t n = 0;
    auto put = [&n](char c) {
        if (n + 1u < sizeof label_buf) {
            label_buf[n++] = c;
        }
    };
    auto put_num = [&put](uint32_t v) {
        char d[6];
        uint8_t k = 0;
        do {
            d[k++] = static_cast<char>('0' + v % 10u);
            v /= 10u;
        } while (v != 0u && k < sizeof d);
        while (k != 0u) {
            put(d[--k]);
        }
    };
    if (s.period_us == 0u) {
        for (const char* p = "control: "; *p != '\0'; ++p) put(*p);
    } else {
        put_num(s.period_us);
        for (const char* p = " us x "; *p != '\0'; ++p) put(*p);
        put_num(s.spin);
        put(':');
        put(' ');
    }
    label_buf[n] = '\0';
    return label_buf;
}

void pass_begin(const PassSpec& s) {
    for (uint8_t i = 0; i < landing_slots; ++i) {
        landed[i] = 0;
    }
    landed_completion = 0;
    landed_dma_completion = 0;
    landed_sink_any = 0;
    pressure_irqs = 0;
    pass_runs0 = Pressure::runs;
    pass_overflows0 = Pressure::queue.overflows();
    pressure_spin = s.spin;
    if (s.period_us != 0u) {
        Pulse::enable(false);
        Pulse::set_count(0);
        (void)Pulse::set_period(static_cast<uint32_t>(s.period_us) * pulse_per_us - 1u);
        Pulse::clear_flags(Pulse::update_flag);
        Pulse::interrupts(Pulse::update_interrupt, true);
        pass_t0 = Ruler::now();
        Pulse::enable(true);
    } else {
        pass_t0 = Ruler::now();
    }
}

PassStats pass_end(const PassSpec& s) {
    PassStats st{};
    if (s.period_us != 0u) {
        Pulse::enable(false);
        st.wall = Ruler::now() - pass_t0;
        Pulse::interrupts(Pulse::update_interrupt, false);
        Pulse::clear_flags(Pulse::update_flag);
        Nvic::clear_pending(Pulse::irq());
        spin(20'000u);   // the last tick's dispatch, if it was still running
    } else {
        st.wall = Ruler::now() - pass_t0;
    }
    st.irqs = pressure_irqs;
    st.runs = Pressure::runs - pass_runs0;
    st.overflows = static_cast<uint16_t>(Pressure::queue.overflows() - pass_overflows0);
    for (uint8_t i = 0; i < landing_slots; ++i) {
        st.land[i] = landed[i];
    }
    st.completion = landed_completion;
    st.dma_completion = landed_dma_completion;
    st.sink_any = landed_sink_any;
    pressure_spin = 0;
    return st;
}

/// The pass's pressure, printed with where it landed.
void print_pass(const PassSpec& s, const PassStats& st) {
    print(serial, "  [", pass_label(s), "] ", st.wall / (SysClock::hz / 1000u), " ms");
    if (s.period_us == 0u) {
        print(serial, ", TIM2 silent", crlf);
        return;
    }
    print(serial, ", ", st.irqs, " ticks, ", st.runs, " dispatches, ", st.overflows,
          " dropped; landed in the walk (ceiling 0) ", st.land[pressure_index], ", LineSink ",
          st.land[sink_index], ", Client ", st.land[client_index], ", SpiArb ", st.land[arb_index],
          ", Minor ", st.land[minor_index], ", Port ", st.land[port_index], ", Console ",
          st.land[console_index], ", base ", st.land[landing_slots - 1u], crlf);
    print(serial, "    inside a completion's dispatch (any depth) ", st.completion, ", of a DMA "
          "completion ", st.dma_completion, "; LineSink on the stack ", st.sink_any, crlf);
}

/// The pressure was real: one interrupt a period over the pass, plus or
/// minus one for the window's ends (the header's arithmetic), every one
/// dispatched, none dropped.
void judge_pressure(const PassSpec& s, const PassStats& st) {
    if (s.period_us == 0u) {
        bench.verdict(pass_label(s), "TIM2 stayed silent", st.irqs == 0u);
        return;
    }
    const uint32_t period = static_cast<uint32_t>(s.period_us) * cycles_per_us;
    const uint32_t expect = st.wall / period;
    const bool count_ok = st.irqs + 1u >= expect && st.irqs <= expect + 1u;
    bench.verdict(pass_label(s), "Pressure ticked once a period, each tick one dispatch, none dropped",
                  count_ok && st.runs == st.irqs && st.overflows == 0u);
}

// =============================================================================
// The SPI link: spi_link.hpp through the Client and the arbiter
// =============================================================================

namespace sl {

using spilink::Op;

/// The command channel's SCK: PCLK1/256 = 175.8 kHz, 45 us a character.
constexpr SpiClock link_clock = SpiClock::div256;

uint8_t frame_buf[spilink::max_payload + 8];
alignas(4) uint8_t answer_buf[spilink::answer_bytes];
alignas(4) uint8_t dummy_buf[spilink::answer_bytes];
bool quiet = false;

/// One select window through the Client and the arbiter, waited for (at
/// most 500 ms: the longest window here is 2.2 ms). True when it was
/// answered spi_ok.
bool window(const uint8_t* tx, uint8_t* rx, uint16_t n, SpiClock rate = link_clock,
            SpiMode mode = SpiMode::mode0, SpiDataSize bits = SpiDataSize::bits8, bool polled = false,
            bool lsb = false, bool decode = false) {
    if (n == 0u) {
        return true;
    }
    const uint32_t before = Client::done;
    Window win{};
    win.tx = lend<Lease::reply>(tx);
    win.rx = lend<Lease::reply>(rx);
    win.len = n;
    win.clock = rate;
    win.mode = mode;
    win.bits = bits;
    win.polled = polled;
    win.lsb = lsb;
    win.decode = decode;
    post<Client>(win);
    if (!wait_until([before] { return Client::done != before; }, 500)) {
        return false;
    }
    return Client::status == spi_ok;
}

void settle() { wait_ms(spilink::settle_ms); }

/// A command window, settle_ms after whatever window came before it:
/// spi_link.hpp's "between one chip-select window and the next", which the
/// peer spends going dark again after an answer window - a command frame
/// clocked sooner is not heard (measured: every first attempt lost).
void send_frame(Op op, const uint8_t* p, uint8_t len) {
    settle();
    uint8_t n = 0;
    spilink::write_frame(
        [&n](uint8_t b) {
            if (n < sizeof frame_buf) {
                frame_buf[n++] = b;
            }
        },
        op, p, len);
    (void)window(frame_buf, nullptr, n);
}

/// One answer window: answer_bytes dummies, the Client decoding the frame.
bool recv_frame(spilink::Frame& out) {
    for (uint16_t i = 0; i < spilink::answer_bytes; ++i) {
        dummy_buf[i] = 0;
        answer_buf[i] = 0xEE;
    }
    if (!window(dummy_buf, answer_buf, spilink::answer_bytes, link_clock, SpiMode::mode0,
                SpiDataSize::bits8, false, false, true)) {
        return false;
    }
    if (!Client::found) {
        return false;
    }
    out = Client::frame;
    return true;
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

/// Three attempts, each 400 ms after the last - longer than the peer's own
/// answer window bound (the protocol's recovery guarantee).
bool command(Op op, const uint8_t* p = no_payload, uint8_t len = 0) {
    for (uint8_t k = 0; k < 3u; ++k) {
        if (command_once(op, p, len)) {
            return true;
        }
        if (!quiet) {
            print(serial, "    (op ", hex(spilink::byte_of(op)), " attempt ", k, ": status ",
                  Client::status, ", the window carried");
            for (uint8_t i = 0; i < Client::raw_n; ++i) {
                print(serial, " ", hex(Client::raw[i]));
            }
            print(serial, ")", crlf);
        }
        wait_ms(400);
    }
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
    for (uint8_t k = 0; k < 4u; ++k) {
        spilink::Frame f;
        if (query(Op::report, f) && f.op == Op::report_data && f.len == spilink::report_size) {
            r = spilink::get_report(f.data);
            return true;
        }
        wait_ms(60);
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

/// Is the peer there? A named failure, never a hang.
bool need_peer() {
    quiet = true;
    bool ok = false;
    for (uint8_t k = 0; k < 3u && !ok; ++k) {
        ok = command(Op::ping);
    }
    quiet = false;
    if (!ok) {
        print(serial, "  THE PEER DID NOT ANSWER: the STM32G0B1RE must be running `spi_peer` "
                      "(its console '0' forces the dark client back)",
              crlf);
        bench.verdict("the peer answers a ping (the peer board running spi_peer)", false);
    }
    return ok;
}

// ---- one exchange, commanded, clocked, reported ----------------------------------------

constexpr uint16_t cap = 64;   ///< frames a window carries at most
alignas(4) uint8_t xtx[2u * cap + 4u];
alignas(4) uint8_t xrx[2u * cap + 4u];

struct Exchange {
    spilink::Cfg cfg{.apply = 1, .mode = 0, .dord = 0, .regime = spilink::regime_buffer_wait};
    SpiMode host_mode = SpiMode::mode0;
    bool host_lsb = false;
    SpiClock rate = link_clock;
    uint16_t count = 32;     ///< frames
    bool wide = false;
    bool polled = false;
    bool odd = false;        ///< the buffers at an odd address: a 16-bit window off the engines
    uint8_t seed_a = 0x13;
    uint8_t seed_b = 0x57;
};

struct Outcome {
    bool accepted = false;
    bool clocked = false;
    bool reported = false;
    uint8_t status = spi_ok;
    spilink::Report r{};
    uint16_t mism = 0;
    uint16_t idx = 0xFFFF;
    uint8_t got = 0, exp = 0;
    uint32_t cycles = 0;     ///< the data window, post to reply
};

bool reversed(const Exchange& e) { return e.host_lsb != (e.cfg.dord != 0u); }

bool exact(const Exchange& e, const Outcome& o) {
    return o.accepted && o.clocked && o.reported && o.r.count == e.count && o.r.mism == 0u &&
           o.mism == 0u && (!e.wide || (o.r.flags & spilink::report_extended) != 0u);
}

Outcome run(const Exchange& e) {
    Outcome o{};
    spilink::Params a{};
    a.cfg = e.cfg;
    a.count = e.count;
    a.ms = 400;
    a.seed_a = e.seed_a;
    a.seed_b = e.seed_b;
    a.pattern = spilink::pattern_prbs;
    a.spare = e.wide ? spilink::spare_wide : 0u;
    if (reversed(e)) {
        a.flags |= spilink::flag_expect_reversed;
    }
    o.accepted = peer_act(Op::exchange, a);
    if (!o.accepted) {
        return o;
    }
    const uint16_t n = e.count < cap ? e.count : cap;
    const uint16_t bytes = static_cast<uint16_t>(e.wide ? 2u * n : n);
    uint8_t* const tx = xtx + (e.odd ? 1u : 0u);
    uint8_t* const rx = xrx + (e.odd ? 1u : 0u);
    {
        spilink::Stream out(a.pattern, a.seed_a);
        for (uint16_t i = 0; i < bytes; ++i) {
            tx[i] = out.next();
            rx[i] = 0xEE;
        }
    }
    settle();
    o.clocked = window(tx, rx, n, e.rate, e.host_mode, e.wide ? SpiDataSize::bits16 : SpiDataSize::bits8,
                       e.polled, e.host_lsb);
    o.status = Client::status;
    o.cycles = Client::cycles;
    o.reported = peer_report(o.r);
    for (uint16_t i = 0; i < bytes; ++i) {
        const uint8_t want = spilink::wide_byte(a.pattern, a.seed_b, i, e.wide, reversed(e));
        if (rx[i] != want) {
            if (o.mism == 0u) {
                o.idx = i;
                o.got = rx[i];
                o.exp = want;
            }
            ++o.mism;
        }
    }
    return o;
}

uint32_t division(SpiClock c) { return 2u << static_cast<uint8_t>(c); }

void print_outcome(const Exchange& e, const Outcome& o) {
    print(serial, "    mode ", static_cast<uint8_t>(e.host_mode), e.host_lsb ? " LSb" : " MSb",
          e.cfg.dord != 0u ? " (peer LSb)" : "", e.wide ? " 16-bit" : " 8-bit", " PCLK1/",
          division(e.rate), ": status ", o.status, " host mism=", o.mism);
    if (o.mism != 0u) {
        print(serial, " (byte ", o.idx, " got ", hex(o.got), " exp ", hex(o.exp), ")");
    }
    print(serial, " peer count=", o.r.count, " mism=", o.r.mism);
    if (o.r.mism != 0u) {
        print(serial, " (byte ", o.r.idx, " got ", hex(o.r.got), " exp ", hex(o.r.exp), ")");
    }
    print(serial, " flags=", hex(o.r.flags), o.accepted ? "" : " NOT ACCEPTED",
          o.reported ? "" : " NO REPORT", crlf);
}

}  // namespace sl

// =============================================================================
// The UART link: usart_link.hpp on USART1's polled resource, between legs
// =============================================================================

namespace ul {

using U1 = Usart<1>;
using U1Tx = Pin<'A', 9>;
using U1Rx = Pin<'A', 10>;
constexpr uint32_t u1_hz = SysClock::pclk2_hz;

link::Decoder dec;
const uint8_t no_payload[1] = {0};
bool quiet = false;

void flush() {
    for (uint8_t k = 0; k < 4u; ++k) {
        U1::clear_by_read();
    }
}

/// The command channel, 8N1 at 115200 on the polled resource: the block
/// reset and programmed with the pads off it, then the pads handed over
/// (test_stm32f4_serial's dut_link()).
bool command_mode() {
    dec.reset();
    U1Tx::input(PinPull::up);
    U1::bus_clock(true);
    U1::enable(false);
    U1::reset();
    const std::optional<uint16_t> brr = usart_brr(u1_hz, link::command_baud);
    if (!brr || !U1::configure(UartFormat{}, *brr)) {
        return false;
    }
    U1::transmitter(true);
    U1::receiver(true);
    U1::enable(true);
    wait_us(2u * (1'000'000u / link::command_baud) + 20u);
    U1Tx::function(PinFunction::af7, {.speed = PinSpeed::very_high});
    U1Rx::function(PinFunction::af7, {.pull = PinPull::up});
    flush();
    return true;
}

/// The link's pads given back and the block off.
void release() {
    U1::enable(false);
    U1::reset();
    U1::bus_clock(false);
    U1Tx::release();
    U1Rx::release();
}

void put1(uint8_t b) {
    const uint32_t t0 = Ticker::millis();
    while (!U1::tx_empty() && Ticker::millis() - t0 < 50u) {
    }
    U1::write_word(b);
}

void wait_tc() {
    const uint32_t t0 = Ticker::millis();
    while (!U1::tx_complete() && Ticker::millis() - t0 < 50u) {
    }
}

/// A frame, polled: a character every 87 us at 115200, and the longest
/// preemption here is a few thousand cycles - tens of microseconds - so the
/// receive register is always read before the next character lands.
bool recv_frame(link::Frame& out, uint16_t ms) {
    const uint32_t t0 = Ticker::millis();
    dec.reset();
    while (Ticker::millis() - t0 < ms) {
        const uint32_t st = U1::status();
        if ((st & UsartFlag::rxne) == 0u) {
            continue;
        }
        const uint8_t b = U1::read_data();
        if ((st & (UsartFlag::fe | UsartFlag::pe | UsartFlag::ne)) != 0u) {
            dec.reset();
            continue;
        }
        if (dec.feed(b) == link::Decoder::Result::frame) {
            out = dec.frame();
            return true;
        }
    }
    return false;
}

bool command_once(link::Op op, const uint8_t* p, uint8_t len, uint16_t ms) {
    flush();
    link::write_frame(put1, op, p, len);
    wait_tc();
    link::Frame f;
    if (!recv_frame(f, ms)) {
        return false;
    }
    return f.op == link::Op::ack && f.len == 2u && f.data[0] == link::byte_of(op);
}

bool command(link::Op op, const uint8_t* p = no_payload, uint8_t len = 0, uint16_t ms = 80) {
    for (uint8_t k = 0; k < 3u; ++k) {
        if (command_once(op, p, len, ms)) {
            return true;
        }
        (void)command_mode();
        wait_ms(70);
    }
    if (!quiet) {
        print(serial, "    LINK FAILURE op ", hex(link::byte_of(op)),
              ": no ack - is the far board running usart_peer?", crlf);
    }
    return false;
}

bool query(link::Op op, link::Frame& data, uint16_t ms = 80) {
    return command(op, no_payload, 0, ms) && recv_frame(data, ms);
}

bool ensure() {
    quiet = true;
    bool found = false;
    for (uint8_t k = 0; k < 3u && !found; ++k) {
        found = command_mode() && command(link::Op::ping);
    }
    quiet = false;
    if (!found) {
        print(serial, "  THE PEER DID NOT ANSWER: the STM32G0B1RE must be running `usart_peer`", crlf);
        bench.verdict("the peer answers a ping (the peer board running usart_peer)", false);
    }
    return found;
}

bool peer_report(link::Report& r, uint16_t after_ms) {
    (void)command_mode();
    if (after_ms != 0u) {
        wait_ms(after_ms);
    }
    for (uint8_t k = 0; k < 4u; ++k) {
        link::Frame f;
        if (query(link::Op::report, f) && f.op == link::Op::report_data && f.len == link::report_size) {
            r = link::get_report(f.data);
            return true;
        }
        wait_ms(40);
    }
    return false;
}

/// Command an action; the peer opens its window settle_ms + 2 ms after its
/// ack.
bool peer_act(link::Op op, const link::Params& a) {
    uint8_t p[link::max_payload] = {};
    link::put_params(p, a);
    if (!command(op, p, link::params_size)) {
        return false;
    }
    wait_ms(link::settle_ms);
    return true;
}

}  // namespace ul

// =============================================================================
// The UART legs
// =============================================================================

/// The Uart task up at `rate` on USART1, its vectors live.
bool task_up(uint32_t rate) {
    link_live = true;
    if (!LinkPort::init(clock, rate)) {
        link_live = false;
        return false;
    }
    return true;
}

void task_down() {
    LinkPort::release();
    link_live = false;
}

/// What a leg's receive side counted: the transport's counters and the
/// port's, as deltas over the leg.
struct PortCounters {
    uint8_t ring_overruns = 0, ore = 0, fe = 0, pe = 0, ne = 0, dma_faults = 0;
    uint8_t line_overflows = 0;
    uint32_t torn = 0;
    bool clean() const {
        return ring_overruns == 0u && ore == 0u && fe == 0u && pe == 0u && ne == 0u && dma_faults == 0u &&
               line_overflows == 0u && torn == 0u;
    }
};

void print_counters(const PortCounters& c) {
    print(serial, "    port: ring overruns ", c.ring_overruns, ", ORE ", c.ore, ", FE ", c.fe, ", PE ", c.pe,
          ", NE ", c.ne, ", DMA faults ", c.dma_faults, ", line overflows ", c.line_overflows,
          ", torn lines ", c.torn, crlf);
}

char line_buf[line_most];

/// One line out through write_bulk(), the ring topped up as it drains. False
/// when the transmitter stopped taking bytes for 50 ms.
bool write_line(const char* s, uint8_t n) {
    uint32_t sent = 0;
    const uint32_t t0 = Ticker::millis();
    while (sent < n) {
        sent += LinkPort::write_bulk(std::span<const uint8_t>(reinterpret_cast<const uint8_t*>(s) + sent, n - sent));
        if (sent < n && Ticker::millis() - t0 >= 50u) {
            return false;
        }
    }
    return true;
}

struct LegResult {
    bool acted = false, up = false, wrote = false, have = false;
    uint32_t bytes = 0;
    uint16_t sum = 0;
    uint32_t cycles = 0;      ///< the first write to the last line's arrival (t, v) or to tx_idle (u)
    PortCounters c{};
    link::Report r{};
};

uint8_t line_with_eol(uint16_t seq) {
    const uint8_t n = make_line(seq, line_buf);
    line_buf[n] = '\n';
    return static_cast<uint8_t>(n + 1u);
}

/// t and v's leg: `lines` numbered lines from `seq0` through the peer's echo
/// at `rate`, the leg's first byte an end of line that empties the
/// assembler; then the port's counters and the peer's. PACED (t): each line
/// written after the one before has left this end's wire and two frame
/// times more (the header's pacing). STREAMED (v, `window` lines): line k
/// written as soon as line k - window has come back to the sink, so the
/// characters keep arriving while the port drains and the sink borrows; the
/// window bounds what the peer can hold of a backlog to the deficit over
/// `window` lines (the header's arithmetic, twice).
LegResult echo_leg(uint32_t rate, uint16_t lines, uint16_t seq0, uint8_t window = 0) {
    LegResult g{};
    for (uint16_t k = 0; k < lines; ++k) {
        g.bytes += line_with_eol(static_cast<uint16_t>(seq0 + k));
    }
    g.bytes += 1u;   // the leading end of line
    link::Params a{};
    a.cfg.rate = rate;
    a.count = static_cast<uint16_t>(g.bytes);
    a.ms = static_cast<uint16_t>(g.bytes * 10u * 1000u * 2u / rate + 500u);
    g.acted = ul::peer_act(link::Op::echo, a);
    if (!g.acted) {
        return g;
    }
    LineSink::reset(seq0);
    const uint8_t overflows0 = LinePort::line_overflows();
    const uint32_t torn0 = LinePort::torn_lines();
    g.up = task_up(rate);
    if (!g.up) {
        return g;
    }
    wait_ms(6);   // the peer's echo window opens settle_ms + 2 ms after its ack
    const uint32_t gap = 2u * 10u * (SysClock::hz / rate);
    const uint32_t t0 = Ruler::now();
    g.wrote = write_line("\n", 1);
    for (uint16_t k = 0; k < lines && g.wrote; ++k) {
        if (window != 0u && k >= window) {
            const uint32_t back = static_cast<uint32_t>(k - window) + 1u;
            (void)wait_until([back] { return LineSink::good + LineSink::corrupt + LineSink::backwards >= back; }, 20);
        }
        const uint8_t n = line_with_eol(static_cast<uint16_t>(seq0 + k));
        g.wrote = write_line(line_buf, n);
        if (window == 0u) {
            const uint32_t m0 = Ticker::millis();
            while (!LinkPort::tx_idle() && Ticker::millis() - m0 < 50u) {
            }
            spin(gap);
        }
    }
    (void)wait_until([lines] { return LineSink::good + LineSink::corrupt + LineSink::backwards >= lines; }, 300);
    g.cycles = Ruler::now() - t0;
    wait_ms(5);   // the last edge's dispatch, and anything a slip would still deliver
    g.c.ring_overruns = LinkPort::rx_overruns();
    g.c.ore = LinkPort::hw_overruns();
    g.c.fe = LinkPort::frame_errors();
    g.c.pe = LinkPort::parity_errors();
    g.c.ne = LinkPort::noise_errors();
    g.c.dma_faults = LinkPort::dma_faults();
    g.c.line_overflows = static_cast<uint8_t>(LinePort::line_overflows() - overflows0);
    g.c.torn = LinePort::torn_lines() - torn0;
    task_down();
    g.have = ul::peer_report(g.r, 0);
    return g;
}

/// u's leg: `lines` numbered lines back to back into the peer's sink.
LegResult sink_leg(uint32_t rate, uint16_t lines, uint16_t seq0) {
    LegResult g{};
    for (uint16_t k = 0; k < lines; ++k) {
        const uint8_t n = line_with_eol(static_cast<uint16_t>(seq0 + k));
        g.bytes += n;
        for (uint8_t i = 0; i < n; ++i) {
            g.sum = static_cast<uint16_t>(g.sum + static_cast<uint8_t>(line_buf[i]));
        }
    }
    link::Params a{};
    a.cfg.rate = rate;
    a.count = static_cast<uint16_t>(g.bytes);
    a.ms = static_cast<uint16_t>(g.bytes * 10u * 1000u * 2u / rate + 500u);
    g.acted = ul::peer_act(link::Op::sink, a);
    if (!g.acted) {
        return g;
    }
    g.up = task_up(rate);
    if (!g.up) {
        return g;
    }
    wait_ms(6);
    const uint32_t t0 = Ruler::now();
    g.wrote = true;
    for (uint16_t k = 0; k < lines && g.wrote; ++k) {
        const uint8_t n = line_with_eol(static_cast<uint16_t>(seq0 + k));
        g.wrote = write_line(line_buf, n);
    }
    const uint32_t m0 = Ticker::millis();
    while (!LinkPort::tx_idle() && Ticker::millis() - m0 < 100u) {
    }
    g.cycles = Ruler::now() - t0;
    task_down();
    g.have = ul::peer_report(g.r, 0);
    return g;
}

/// The wire's time for `bytes` 8N1 characters at `rate`, in core cycles.
uint32_t wire_cycles(uint32_t bytes, uint32_t rate) {
    return static_cast<uint32_t>((static_cast<uint64_t>(bytes) * 10u * SysClock::hz) / rate);
}

void print_leg(const char* what, uint32_t rate, const LegResult& g) {
    print(serial, "    ", what, " at ", rate, ": ", g.bytes, " bytes in ", g.cycles / cycles_per_us, " us (the wire's ",
          wire_cycles(g.bytes, rate) / cycles_per_us, " us, ",
          g.cycles != 0u ? static_cast<uint32_t>(static_cast<uint64_t>(wire_cycles(g.bytes, rate)) * 1000u / g.cycles) : 0u,
          " permille of it); peer count ", g.r.count, " sum ", hex(g.r.sum), " ferr ", g.r.ferr, " perr ",
          g.r.perr, " ovf ", g.r.ovf, " flags ", hex(g.r.flags), g.acted ? "" : " NOT ACCEPTED",
          g.have ? "" : " NO REPORT", crlf);
}

void print_sink() {
    print(serial, "    sink: ", LineSink::lines, " lines (", LineSink::good, " good, ", LineSink::empties,
          " empty, ", LineSink::corrupt, " corrupt, ", LineSink::gaps, " gaps losing ", LineSink::lost, ", ",
          LineSink::backwards, " backwards), lent with another waiting ", LineSink::waiting,
          ", same buffer twice ", LineSink::same_buffer, ", nested ", LineSink::nested,
          ", the stream moved inside ", LineSink::moved, crlf);
}

// =============================================================================
// n - the peer's command channel
// =============================================================================

void tn_peer_link() {
    for (uint8_t p = 0; p < pass_count; ++p) {
        const PassSpec& s = passes[p];
        pass_begin(s);
        const bool there = sl::need_peer();
        spilink::Ident d{};
        const bool got = there && sl::peer_ident(d);
        uint8_t pings = 0;
        for (uint8_t i = 0; there && i < 10u; ++i) {
            if (sl::command(spilink::Op::ping)) {
                ++pings;
            }
        }
        const PassStats st = pass_end(s);
        print_pass(s, st);
        if (got) {
            print(serial, "    peer: label '");
            for (uint8_t i = 0; i < 8u && d.label[i] != '\0'; ++i) {
                print(serial, d.label[i]);
            }
            print(serial, "' sanity=", hex(d.sanity), " fw=", hex(d.version), "; ", pings, " of 10 pings", crlf);
        }
        bench.verdict(pass_label(s), "ident is spi_peer's (the sanity byte), the STM32G0 port (0x03xx)",
                      got && d.sanity == spilink::ident_sanity &&
                          spilink::peer_kind(d.version) == spilink::peer_stm32g0);
        bench.verdict(pass_label(s), "ten more frames answered, every one through the arbiter", pings == 10u);
        judge_pressure(s, st);
    }
}

// =============================================================================
// o - the matrix
// =============================================================================

/// The matrix's rate: PCLK1/32 = 1.41 MHz on the engines.
constexpr SpiClock matrix_rate = SpiClock::div32;

void to_matrix() {
    uint64_t control_bytes = 0, control_cycles = 0;
    for (uint8_t p = 0; p < pass_count; ++p) {
        const PassSpec& s = passes[p];
        pass_begin(s);
        if (!sl::need_peer()) {
            (void)pass_end(s);
            return;
        }
        uint8_t good = 0, total = 0;
        uint32_t bytes = 0, cycles = 0;
        auto one = [&](sl::Exchange e) {
            e.rate = matrix_rate;
            e.count = 32;
            e.seed_a = static_cast<uint8_t>(0x13u + 7u * total);
            e.seed_b = static_cast<uint8_t>(0x57u + 5u * total);
            const sl::Outcome o = sl::run(e);
            ++total;
            if (sl::exact(e, o)) {
                ++good;
                bytes += e.wide ? 64u : 32u;
                cycles += o.cycles;
            } else {
                sl::print_outcome(e, o);
            }
        };
        for (uint8_t m = 0; m < 4u; ++m) {
            for (uint8_t wide = 0; wide < 2u; ++wide) {
                sl::Exchange e{};
                e.cfg.mode = m;
                e.host_mode = static_cast<SpiMode>(m);
                e.wide = wide != 0u;
                one(e);
            }
        }
        for (uint8_t wide = 0; wide < 2u; ++wide) {
            sl::Exchange e{};
            e.cfg.dord = 1;
            e.host_lsb = true;
            e.wide = wide != 0u;
            one(e);
        }
        {
            sl::Exchange e{};
            e.cfg.dord = 1;   // the peer LSb, this end MSb: each end checks the bit reverse
            one(e);
        }
        const bool ping = sl::command(spilink::Op::ping);
        const PassStats st = pass_end(s);
        print_pass(s, st);
        const uint32_t rate_x10 = cycles != 0u
                                      ? static_cast<uint32_t>(static_cast<uint64_t>(bytes) * SysClock::hz / 100u / cycles)
                                      : 0u;
        if (p == 0u) {
            control_bytes = bytes;
            control_cycles = cycles;
        }
        const uint32_t vs = control_cycles != 0u && cycles != 0u && bytes == control_bytes
                                ? static_cast<uint32_t>(control_cycles * 1000u / cycles)
                                : 0u;
        print(serial, "    ", good, " of ", total, " exchanges exact; the data windows carried ", bytes,
              " bytes in ", cycles / cycles_per_us, " us, ", rate_x10 / 10u, ".", rate_x10 % 10u,
              " kB/s (", vs, " permille of the control's rate)", crlf);
        bench.verdict(pass_label(s), "four modes x two widths, LSb first in both, a DORD mismatch: every "
                                     "window byte-exact at both ends on the DMA engines",
                      good == total);
        bench.verdict(pass_label(s), "the command channel still answers after the matrix", ping);
        judge_pressure(s, st);
    }
}

// =============================================================================
// p - the ladders
// =============================================================================

constexpr SpiClock rungs[] = {SpiClock::div256, SpiClock::div128, SpiClock::div64, SpiClock::div32,
                              SpiClock::div16,  SpiClock::div8,   SpiClock::div4,  SpiClock::div2};

enum class Path : uint8_t { polled, pump, engines };
const char* path_name(Path p) {
    switch (p) {
        case Path::polled: return "polled loop";
        case Path::pump: return "RXNE pump";
        default: return "DMA engines";
    }
}

/// A frame's core cycles at code `c`: bits x 2^(code + 1) PCLK1 cycles,
/// HCLK = 4 x PCLK1.
uint32_t frame_cycles(SpiClock c, bool wide) {
    return (wide ? 16u : 8u) * sl::division(c) * (SysClock::hz / SysClock::pclk1_hz);
}

/// What the polled ladder's failures were, when it had any: cut short by
/// an overrun the host detected (status spi_overrun, the run never handed
/// back with a hole), or something else.
uint8_t polled_cut_short = 0;     ///< rungs that ended spi_overrun
bool polled_failed_otherwise = false;

/// One ladder on one path: every rung judged, the first failure's outcome
/// printed, the two-in-flight rungs named.
bool climb(Path path, uint8_t seed) {
    bool all = true;
    uint8_t k = 0;
    if (path == Path::polled) {
        polled_cut_short = 0;
        polled_failed_otherwise = false;
    }
    for (const SpiClock c : rungs) {
        sl::Exchange e{};
        e.rate = c;
        e.polled = path == Path::polled;
        e.wide = path == Path::pump;
        e.odd = path == Path::pump;
        e.count = path == Path::polled ? 8u : path == Path::pump ? 16u : 32u;
        e.seed_a = static_cast<uint8_t>(seed + 9u * k);
        e.seed_b = static_cast<uint8_t>(seed + 0x40u + 5u * k);
        ++k;
        const sl::Outcome o = sl::run(e);
        const bool ok = sl::exact(e, o);
        if (!ok) {
            const bool two = path == Path::polled
                                 ? frame_cycles(c, false) >= Host::write_ahead_min_frame_cycles
                                 : path == Path::pump && Host::two_in_flight(c, SpiDataSize::bits16);
            print(serial, "    ", path_name(path), two ? " (two frames in flight)" : "", ":", crlf);
            sl::print_outcome(e, o);
            if (path == Path::polled) {
                if (o.status == brio::spi_overrun) {
                    ++polled_cut_short;
                } else {
                    polled_failed_otherwise = true;
                }
            }
        }
        all = all && ok;
    }
    return all;
}

void tp_ladders() {
    print(serial, "  the polled loop keeps two frames in flight from a frame of ",
          Host::write_ahead_min_frame_cycles, " cycles (8-bit: /8 and slower); the pump's 16-bit frames "
          "two in flight at /8 ", Host::two_in_flight(SpiClock::div8, SpiDataSize::bits16) ? "yes" : "no",
          ", /4 ", Host::two_in_flight(SpiClock::div4, SpiDataSize::bits16) ? "yes" : "no", crlf);
    for (uint8_t p = 0; p < pass_count; ++p) {
        const PassSpec& s = passes[p];
        pass_begin(s);
        if (!sl::need_peer()) {
            (void)pass_end(s);
            return;
        }
        const uint16_t over0 = Host::overruns();
        const bool polled = climb(Path::polled, 0x21);
        const bool pump = climb(Path::pump, 0x35);
        const bool engines = climb(Path::engines, 0x43);
        const bool ping = sl::command(spilink::Op::ping);
        const uint16_t overruns = static_cast<uint16_t>(Host::overruns() - over0);
        const PassStats st = pass_end(s);
        print_pass(s, st);
        print(serial, "    the host's overruns over the pass: ", overruns,
              "; polled rungs cut short by a detected overrun: ", polled_cut_short, crlf);
        if (s.period_us == 0u) {
            bench.verdict(pass_label(s), "THE POLLED LOOP through the arbiter: 8 frames byte-exact both ways at "
                                         "every code, two in flight from /8",
                          polled);
        } else {
            // Under pressure the polled loop runs inside the arbiter's
            // dispatch, which every dispatch above the arbiter holds off for
            // its whole length - a length the host's hold-off, the image's
            // longest handler or masked window, does not count. A preemption
            // that outlasts a frame ends the transaction spi_overrun: the
            // service's promise is that it detects it every time and never
            // hands back a run with a hole, and that is what is judged; the
            // count of rungs cut short is printed beside it. The pump and the
            // engines, judged below, carry no such limit.
            bench.verdict(pass_label(s), "THE POLLED LOOP through the arbiter: byte-exact, or cut short only by "
                                         "a detected overrun when a dispatch above the arbiter outlasts a "
                                         "frame - never a run with a hole",
                          polled || !polled_failed_otherwise);
        }
        bench.verdict(pass_label(s), "THE RXNE PUMP: 16 16-bit frames byte-exact at every code, two in "
                                     "flight at /8 and /4",
                      pump);
        bench.verdict(pass_label(s), "THE DMA ENGINES: 32 frames byte-exact at every code to PCLK1/2", engines);
        bench.verdict(pass_label(s), "the command channel survives the ladders", ping);
        judge_pressure(s, st);
    }
}

// =============================================================================
// w - the arbiter under pressure
// =============================================================================

void tw_arbiter() {
    for (uint8_t p = 0; p < pass_count; ++p) {
        const PassSpec& s = passes[p];
        const bool pressing = s.period_us != 0u;
        pass_begin(s);
        if (!sl::need_peer()) {
            (void)pass_end(s);
            return;
        }
        const uint16_t frames = static_cast<uint16_t>(w::client_n * w::client_len + w::minor_n * w::minor_len +
                                                      (pressing ? w::press_n * w::press_len : 0u));
        spilink::Params a{};
        a.cfg = spilink::Cfg{.apply = 1, .mode = 0, .dord = 0, .regime = spilink::regime_buffer_wait};
        a.count = frames;
        a.ms = 2000;
        a.seed_a = w::seed_a;
        a.seed_b = w::seed_b;
        a.pattern = spilink::pattern_fixed;
        const bool acted = sl::peer_act(spilink::Op::exchange, a);
        for (auto& r : w::client_rx) for (uint8_t& b : r) b = w::untouched;
        for (auto& r : w::minor_rx) for (uint8_t& b : r) b = w::untouched;
        for (auto& r : w::press_rx) for (uint8_t& b : r) b = w::untouched;
        w::clear(w::client);
        w::clear(w::minor);
        w::clear(w::press);
        Pressure::outstanding = false;
        const uint8_t rejected0 = SpiArb::rejected_count();
        const uint8_t stale0 = SpiArb::stale_events();
        const uint32_t t0 = Ruler::now();
        bool done = false;
        if (acted) {
            // The window: the mode primed with the select high (a CPOL flip
            // inside an open window is an edge for the client), the 30 us
            // the protocol pays on each side of it. The bus is idle here: no
            // request is outstanding.
            sl::settle();
            Host::prime(SpiMode::mode0, SpiClock::div256);
            CsPin::clear();
            wait_us(30);
            w::live = true;
            w::press_live = pressing;
            post<Client>(Kick{});
            post<Minor>(Kick{});
            done = wait_until(
                [pressing] {
                    return w::client.replies >= w::client_n && w::minor.replies >= w::minor_n &&
                           (!pressing || w::press.replies >= w::press_n);
                },
                2000);
            w::press_live = false;
            w::live = false;
            wait_us(30);
            CsPin::set();
        }
        const uint32_t took = Ruler::now() - t0;
        spilink::Report r{};
        const bool rep = acted && sl::peer_report(r);
        uint16_t host_bad = 0;
        for (auto& row : w::client_rx) for (uint8_t b : row) host_bad = static_cast<uint16_t>(host_bad + (b != w::seed_b));
        for (auto& row : w::minor_rx) for (uint8_t b : row) host_bad = static_cast<uint16_t>(host_bad + (b != w::seed_b));
        if (pressing) {
            for (auto& row : w::press_rx) for (uint8_t b : row) host_bad = static_cast<uint16_t>(host_bad + (b != w::seed_b));
        }
        const uint8_t rejected = static_cast<uint8_t>(SpiArb::rejected_count() - rejected0);
        const uint8_t stale = static_cast<uint8_t>(SpiArb::stale_events() - stale0);
        const PassStats st = pass_end(s);
        print_pass(s, st);
        print(serial, "    ", took / cycles_per_us, " us; Client ", w::client.replies, "/", w::client.posted,
              " replies (", w::client.ok, " ok, ", w::client.out_of_order, " out of order), Minor ",
              w::minor.replies, "/", w::minor.posted, " (", w::minor.ok, " ok, ", w::minor.out_of_order,
              " out of order), Pressure ", w::press.replies, "/", w::press.posted, " (", w::press.ok, " ok, ",
              w::press.out_of_order, " out of order); timeouts ",
              w::client.timeouts + w::minor.timeouts + w::press.timeouts, ", rejected ", rejected, ", stale ", stale,
              "; host bytes wrong ", host_bad, "; peer count ", r.count, " of ", frames, " mism ", r.mism, crlf);
        if (host_bad != 0u) {
            print(serial, "    windows not the peer's:");
            for (uint8_t k = 0; k < w::client_n; ++k) {
                if (!w::complete(w::client_rx[k], w::client_len)) {
                    print(serial, " C", k, "=");
                    for (uint8_t b : w::client_rx[k]) print(serial, hex(b), ",");
                }
            }
            for (uint8_t k = 0; k < w::minor_n; ++k) {
                if (!w::complete(w::minor_rx[k], w::minor_len)) {
                    print(serial, " M", k);
                }
            }
            print(serial, "; peer's first wrong byte ", r.idx, " got ", hex(r.got), crlf);
        }
        const bool all_ok = done && w::client.ok == w::client_n && w::minor.ok == w::minor_n &&
                            (!pressing || w::press.ok == w::press_n);
        bench.verdict(pass_label(s), "every request answered spi_ok, to its own requester, once", all_ok &&
                      w::client.replies == w::client_n && w::minor.replies == w::minor_n &&
                      w::press.replies == (pressing ? w::press_n : 0u));
        bench.verdict(pass_label(s), "every reply in its requester's order (the Client's before its next "
                                     "window starts)",
                      w::client.out_of_order == 0u && w::minor.out_of_order == 0u && w::press.out_of_order == 0u);
        bench.verdict(pass_label(s), "nothing rejected, the per-bus timeout never fired (no spi_timeout, no "
                                     "stale event)",
                      rejected == 0u && stale == 0u && w::client.timeouts == 0u && w::minor.timeouts == 0u &&
                          w::press.timeouts == 0u && w::client.rejected == 0u && w::minor.rejected == 0u &&
                          w::press.rejected == 0u);
        bench.verdict(pass_label(s), "the peer counted every frame exact, and every frame back is the peer's",
                      rep && r.count == frames && r.mism == 0u && host_bad == 0u);
        judge_pressure(s, st);
    }
}

// =============================================================================
// i - dark
// =============================================================================

void ti_dark() {
    bench.verdict("every object above the Console idle: every queue empty",
                  Pressure::queue.empty() && LineSink::queue.empty() && Client::queue.empty() &&
                      SpiArb::queue.empty() && Minor::queue.empty() && LinePort::queue.empty());
    bench.verdict("TIM2 stopped, its update interrupt off and its line quiet",
                  !Pulse::enabled() && (Pulse::interrupts() & Pulse::update_interrupt) == 0u &&
                      !Nvic::pending(Pulse::irq()));
    bench.verdict("no time event is armed (the arbiter's timeout among them)", !TE::next_deadline().has_value());
    bench.verdict("SPI2 idle with the select high and no DMA request raised",
                  !Host::Resource::busy() && (Host::Resource::regs().CR2 & (SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN)) == 0u &&
                      CsPin::is_output() && CsPin::read());
    bench.verdict("USART1 given back: its gate closed, PA9 and PA10 not outputs",
                  !ul::U1::bus_clock() && !ul::U1Tx::is_output() && !ul::U1Rx::is_output() && !link_live);
    bench.verdict("the LED's pad PA5 is not an output", !Pin<'A', 5>::is_output());
    bench.verdict("the ceiling is the Console's and interrupts are enabled",
                  P::ceiling == console_index && P::interrupts_enabled());
}

// =============================================================================
// t, u, v - the UART
// =============================================================================

constexpr uint32_t uart_rates[] = {460'800u, 921'600u};
constexpr uint16_t echo_lines = 200;
constexpr uint16_t sink_lines = 300;

/// The verdicts of one received leg.
void judge_echo(const PassSpec& s, uint32_t rate, uint16_t lines, const LegResult& g) {
    const bool slow = rate == 460'800u;
    bench.verdict(pass_label(s), slow ? "460800: every line arrived, in order, exact"
                                      : "921600: every line arrived, in order, exact",
                  g.acted && g.up && g.wrote && LineSink::good == lines && LineSink::gaps == 0u &&
                      LineSink::backwards == 0u && LineSink::corrupt == 0u && LineSink::empties <= 1u);
    bench.verdict(pass_label(s), slow ? "460800: the port's own counters zero" : "921600: the port's own counters zero",
                  g.c.clean());
    bench.verdict(pass_label(s), slow ? "460800: the peer echoed every byte, clean"
                                      : "921600: the peer echoed every byte, clean",
                  g.have && g.r.count == g.bytes && g.r.ferr == 0u && g.r.perr == 0u && g.r.ovf == 0u);
}

void tt_lines() {
    for (uint8_t p = 0; p < pass_count; ++p) {
        const PassSpec& s = passes[p];
        pass_begin(s);
        if (!ul::ensure()) {
            (void)pass_end(s);
            ul::release();
            return;
        }
        for (uint8_t k = 0; k < 2u; ++k) {
            LineSink::spin_cycles = 0;
            const LegResult g = echo_leg(uart_rates[k], echo_lines, static_cast<uint16_t>(1000u * k + 7u * p));
            print_leg("echo", uart_rates[k], g);
            print_counters(g.c);
            print_sink();
            judge_echo(s, uart_rates[k], echo_lines, g);
        }
        const PassStats st = pass_end(s);
        ul::release();
        print_pass(s, st);
        judge_pressure(s, st);
    }
}

void tu_blast() {
    uint32_t control[2] = {};
    uint32_t control_bytes[2] = {};
    for (uint8_t p = 0; p < pass_count; ++p) {
        const PassSpec& s = passes[p];
        pass_begin(s);
        if (!ul::ensure()) {
            (void)pass_end(s);
            ul::release();
            return;
        }
        for (uint8_t k = 0; k < 2u; ++k) {
            const uint32_t rate = uart_rates[k];
            const LegResult g = sink_leg(rate, sink_lines, static_cast<uint16_t>(2000u * k + 11u * p));
            print_leg("blast", rate, g);
            if (p == 0u) {
                control[k] = g.cycles;
                control_bytes[k] = g.bytes;
            }
            // Per byte: the lines' lengths follow their numbers, which move
            // from pass to pass.
            print(serial, "    a byte's time against the control's: ",
                  g.cycles != 0u && control_bytes[k] != 0u
                      ? static_cast<uint32_t>(static_cast<uint64_t>(control[k]) * g.bytes * 1000u /
                                              (static_cast<uint64_t>(g.cycles) * control_bytes[k]))
                      : 0u,
                  " permille of its rate", crlf);
            bench.verdict(pass_label(s), rate == 460'800u ? "460800: the peer counted every byte of the blast, its "
                                                            "sum exact, FE PE ORE zero"
                                                          : "921600: the peer counted every byte of the blast, its "
                                                            "sum exact, FE PE ORE zero",
                          g.acted && g.up && g.wrote && g.have && g.r.count == g.bytes && g.r.sum == g.sum &&
                              g.r.ferr == 0u && g.r.perr == 0u && g.r.ovf == 0u);
        }
        const PassStats st = pass_end(s);
        ul::release();
        print_pass(s, st);
        judge_pressure(s, st);
    }
}

void tv_loan() {
    constexpr uint32_t rate = 921'600u;
    for (uint8_t p = 0; p < pass_count; ++p) {
        const PassSpec& s = passes[p];
        pass_begin(s);
        if (!ul::ensure()) {
            (void)pass_end(s);
            ul::release();
            return;
        }
        LineSink::spin_cycles = 6000;
        // A window of 20 lines: the receive engine tells the port at an idle
        // line or a half lap (1024 characters), so a window shorter than a
        // half lap stops the writer before the port is told; 20 lines of at
        // most 71 characters are 1420, and at the peer's worst divisor
        // deficit (half a step of its 1/69 at 921600, 0.72 %) a continuous
        // run of them leaves 10.2 characters in its two eight-deep FIFOs.
        const LegResult g = echo_leg(rate, echo_lines, static_cast<uint16_t>(3000u + 13u * p), 20);
        LineSink::spin_cycles = 0;
        const PassStats st = pass_end(s);
        ul::release();
        print_leg("echo, the sink slow", rate, g);
        print_counters(g.c);
        print_sink();
        print(serial, "    the sink's dispatches took ", LineSink::busy / 1000u, " k cycles; the expected ticks "
              "inside them ", s.period_us != 0u ? LineSink::busy / (static_cast<uint32_t>(s.period_us) * cycles_per_us) : 0u,
              crlf);
        print_pass(s, st);
        judge_echo(s, rate, echo_lines, g);
        bench.verdict(pass_label(s), "every line lent alone (none waiting at the sink's entry, the sink never "
                                     "nested), the two buffers strictly alternating",
                      LineSink::waiting == 0u && LineSink::nested == 0u && LineSink::same_buffer == 0u &&
                          LineSink::distinct <= 2u && LineSink::lines >= echo_lines);
        if (s.period_us != 0u) {
            bench.verdict(pass_label(s), "Pressure landed inside the borrower's dispatch", st.land[sink_index] != 0u);
        }
        judge_pressure(s, st);
    }
}

// =============================================================================
// the Console's turn
// =============================================================================

bool clock_ok = false;
bool tick_ok = false;
bool ruler_ok = false;
bool host_ok = false;

void console_banner() {
    print(serial, crlf, "boot: clk=", clock_ok ? "PLL" : "FAILED", " tick=", tick_ok ? "SysTick" : "FAILED",
          " ruler=", ruler_ok ? "DWT" : "FAILED", " spi2=", host_ok ? "host" : "FAILED", crlf);
    print(serial, "test_stm32f4_rubato_link - the SPI and UART stacks under Rubato against the STM32G0B1RE "
                  "(z: spi_peer, y: usart_peer)",
          crlf);
    bench.menu();
}

void console_turn() {
    uint8_t c = 0;
    while (Serial::read_byte(c)) {
        if (c == '\r' || c == '\n') {
            continue;
        }
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') {
            console_banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        bench.prompt();
    }
}

} // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void USART2_IRQHandler() {
    if (Serial::isr()) {
        brio::post<Console>(ConsoleEvent{console_rx});
    }
}

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

extern "C" [[gnu::naked]] void PendSV_Handler() { brio::scheduler_exception<K>(); }
extern "C" [[gnu::naked]] void SVC_Handler() { brio::scheduler_return(); }

/// The pressure: TIM2's update, where it landed, and the post.
extern "C" void TIM2_IRQHandler() {
    Pulse::clear_flags(Pulse::update_flag);
    const uint8_t c = P::ceiling;
    const uint8_t slot = c < landing_slots - 1u ? c : static_cast<uint8_t>(landing_slots - 1u);
    landed[slot] = landed[slot] + 1u;
    if (arb_event == arb_done) {
        landed_completion = landed_completion + 1u;
        if (done_from_dma) {
            landed_dma_completion = landed_dma_completion + 1u;
        }
    }
    if (sink_depth != 0u) {
        landed_sink_any = landed_sink_any + 1u;
    }
    pressure_irqs = pressure_irqs + 1u;
    brio::post<Pressure>(Tick{});
}

extern "C" void SPI2_IRQHandler() {
    if (Host::isr()) {
        done_from_dma = false;
        brio::post<SpiArb>(brio::TransferDone{Host::status()});
    }
}

extern "C" void DMA1_Stream4_IRQHandler() {
    if (Host::dma_isr()) {
        done_from_dma = true;
        brio::post<SpiArb>(brio::TransferDone{Host::status()});
    }
}
extern "C" void DMA1_Stream3_IRQHandler() {
    if (Host::dma_isr()) {
        done_from_dma = true;
        brio::post<SpiArb>(brio::TransferDone{Host::status()});
    }
}

extern "C" void USART1_IRQHandler() {
    if (link_live && LinkPort::isr()) {
        brio::post<LinePort>(brio::RxActivity{});
    }
}
extern "C" void DMA2_Stream2_IRQHandler() {
    if (link_live && LinkPort::dma_isr()) {
        brio::post<LinePort>(brio::RxActivity{});
    }
}

int main() {
    clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    tick_ok = brio::Ticker::init(clock);
    ruler_ok = Ruler::init();
    brio::Dma<1>::init();
    brio::Dma<2>::init();
    CsPin::output(true);
    host_ok = Host::init(clock, Host::Config{.sck_speed = brio::PinSpeed::medium, .mosi_speed = brio::PinSpeed::medium});
    Pulse::init();
    (void)Pulse::configure(brio::TimConfig{.prescaler = 0, .period = 0xFFFF'FFFFu});
    brio::Nvic::enable(Pulse::irq());
    brio::scheduler_init();

    bench.letter('n', "the peer's command channel through the arbiter", tn_peer_link);
    bench.letter('o', "the matrix on the DMA engines: modes, widths, bit orders", to_matrix);
    bench.letter('p', "the ladders: the polled loop, the pump, the engines, every code", tp_ladders);
    bench.letter('w', "the arbiter under pressure: three requesters, one bus", tw_arbiter);
    bench.letter('i', "dark: every object idle, nothing lit", ti_dark);
    bench.second_letter('t', "the UART received: numbered lines through the peer's echo", tt_lines);
    bench.second_letter('u', "the UART sent: a blast of numbered lines into the peer's sink", tu_blast);
    bench.second_letter('v', "the loan under preemption: a slow borrower at 921600", tv_loan);

    (void)serial_ok;
    brio::enable_interrupts();
    K::run();
}
