// test_samc_rubato_link - the bench suite of the SPI and UART stacks of the
// SAM C21 stratum under Rubato, the preemptive kernel (kernel/rubato.hpp):
// util/spi_bus.hpp's arbiter over samc21/spi.hpp's SpiHost, and
// util/line_reader.hpp's LineReader over samc21/sercom.hpp's Uart and its
// interrupt receiver, run against the desk's AVR64EA48 and its unchanged
// instruments (avrxt/src/apps/spi_peer.cpp, usart_peer.cpp) while an object
// of the highest priority preempts them at moments the traffic does not
// choose. Every verdict is the traffic judged byte-exact, as the Tenuto
// suites (test_samc_spi's peer letters, test_samc_uart's second all-key)
// judge it; nothing here changes a driver or a service.
//
// A test_<target>_<subject> suite in the bench grammar (util/testbench.hpp)
// that runs inside the kernel it tests, as test_samc_rubato does: the
// program is one Rubato pack of seven objects - Pressure, LineSink, Client,
// SpiArb, Minor, LinePort and Console, from the highest priority down - and
// everything of it that names no register, the pack, the passes, the
// numbered lines, the window client and the UART legs, is the link suites'
// shared body (avrxt/src/apps/rubato_link.hpp, whose header describes each
// object and the passes). What is this family's is here, each choice from
// DS60001479M and the driver headers:
//
//   ONE SERCOM, TWO PERSONALITIES. SERCOM1 on PA16..PA19 (function C) is
//   the SPI host for key z and the USART for key y, the same four pads and
//   the same one vector (a SERCOM has one NVIC line, 30.6.3), which serves
//   whichever personality is live (spi_live, link_live, never both). Each
//   letter takes the instance and gives it back at its end, and a hand-over
//   goes through the four pads DARK - PORT inputs, no pull - before the next
//   personality's init(): a SERCOM's reset briefly enables it in USART mode
//   with TxD on PAD[0] (samc21/sercom.hpp's reset()), and a configured
//   transmitter drives its pad at the idle level even disabled (31.8.2,
//   measured: docs/samc21/sercom.md), so no init() runs with a pad muxed.
//   Between letters nothing of this board drives a link wire, so the peer
//   image may be changed at any prompt.
//   THE SPI HOST. SpiHost<1> with NO DMA ENGINE SLOT: the SERCOM's DMA
//   requests are offered (32.6.4.1), and the engine is declined - the
//   DMAC's erratum 1.10.4 corrupts the write-back of concurrently
//   triggered channels and hung the full-duplex pair in 4 to 8 % of runs
//   (docs/samc21/dmac.md), so the stratum admits one channel, the UART's
//   transmit. Its two paths: the POLLED loop (32.6.2.6.1, a character
//   ahead: k + 1 written on DRE, k read on RXC) and the RXC PUMP (one
//   interrupt a character, RXC armed and never DRE, k + 2 written as k is
//   read). Neither keeps more than two characters in flight, and the
//   receive buffer is two levels deep (32.2, 32.6.2.7): a dispatch above
//   the arbiter that holds the polled loop off for any length lets frames
//   k and k + 1 complete and SCK stop - a GAP ON THE BUS, never a lost
//   frame - so the verdict under pressure is STRICT, the host's BUFOVF
//   read at every reply and judged never set. The host has no spi_overrun;
//   its one failure is spi_stalled, a polled budget counted in loop turns
//   (200000 + 6000 a frame) that a preemption does not spend, so a stall
//   under pressure is a finding. Characters are 8 or 9 bits (CTRLB.CHSIZE,
//   32.8.2) and the AVR client is 8-bit: byte frames only. The bit order
//   is CTRLA.DORD, part of the host's configuration and not of a request
//   (samc21/spi.hpp's apply() writes CTRLA without it): no LSb-first rows,
//   the DORD mismatch row kept (the peer LSb, this end MSb).
//   THE HOST ADAPTER. The shared body speaks the STM32 hosts' Request (a
//   SpiClock rate, a frame size, a bit-order verb) and this host's Request
//   names a BAUD byte; `Host` below is a thin adapter over SpiHost<1> that
//   carries the body's field names and forwards each request to the
//   driver's start() unchanged in what it moves - one Request copy a
//   transaction, nothing a character. The adapter OWNS THE SELECT: an AVR
//   client reset by SS rising loses a received character its polled loop
//   has not fetched yet (test_samc_spi, "THE CHIP-SELECT HOLD": every
//   command frame lost its checksum), and the driver raises a request's
//   select a microsecond after the last edge, so the adapter primes the
//   mode, lowers the select, spends cs_setup_us and starts the driver with
//   no select of its own; the Client's reply raises it 30 us later
//   (client_replied()), after the reply chain's own few hundred cycles.
//   THE UART. Uart<1> with TxD on PAD[2] (PA18, TXPO 0x1) and RxD on
//   PAD[0] (PA16), the crossed pair, its core on generator 0 (the task's
//   one generator, OSC48M). The receiver is the INTERRUPT receiver into a
//   SkipRing - the stratum has no receive engine (1.10.4) - whose isr()
//   takes every level of the two-level buffer an entry, call-free on a
//   clean character; LineReader drains its ring in place (read_span() and
//   consume(), with rx_skips() as the gap epoch). The port is told at every
//   empty-to-non-empty edge, so no half-lap arithmetic applies. Two
//   transmitters, two Uart types on the one instance, never live at once:
//   the interrupt transmitter (DRE fed a byte an entry) for t, v and the
//   first half of u, and the DMA TRANSMIT ENGINE (DmaTxEngine, channel 0,
//   the stratum's one DMA user, one block for a ring run) for u's second
//   half - the family's one engine under preemption, with no second
//   channel live for 1.10.4 to bite. `LinkPort` below is the one name the
//   shared body writes through, its transmit verbs forwarded to the leg's
//   type and its receive verbs always to the interrupt transmitter's port.
//   THE PRESSURE. TC4's overflow in MATCH FREQUENCY mode (35.6.2.6.1: CC0
//   is TOP, the counter wraps there and raises OVF, no re-arm in the
//   handler), 16 bits on its own generic clock channel (TC4 cannot pair,
//   35.6.2.4, and has TC4_GCLK_ID alone, 35.5.3) from generator 0, the
//   48 MHz the CPU runs on, prescaler 1: 97 / 193 / 389 us are 4656 / 9264
//   / 18672 counts, CC0 one less. Free of SERCOM1, SERCOM5 (the console),
//   the DMAC and SysTick. Its dispatch, a 3000-cycle spin and road 1's 232
//   cycles (docs/samc21/platform.md), is shorter than the shortest period's
//   4656, so no tick finds the last one still queued; at 97 us x 3000 it
//   takes 64 % of the core.
//   THE RULER. The kernel ticker's BasicTicker::cycles(), CPU cycles from
//   SysTick's position and the tick count (ARMv6-M has no cycle counter),
//   48 cycles a two-read (test_samc_rubato's letter d): TC4 and the ruler
//   count one clock, so judge_pressure()'s arithmetic is exact.
//   THE UART KEY'S COMMAND CHANNEL. SERCOM1 polled at 115200 on generator
//   0 between legs (test_samc_uart's dut_up() shape): a character every
//   87 us into the two-level receive buffer, against a longest preemption
//   of 3000 cycles (62.5 us) and the scheduler. Until the peer first
//   answers, RxD carries the SERCOM's pull-down (31.5.1's one pull): the
//   peer searches both its wirings while silent (usart_link.hpp's
//   rendezvous), and the far TxD, an input then, would leave PA16 floating
//   beside the wire this end drives (test_samc_uart's pads_crossed()).
//
// THE WIRES, fixed on the desk (test_samc_spi's peer letters and
// test_samc_uart's two-board half):
//   PA16 (PAD[0]) - EA PE0: MOSI for spi_peer; the EA's TxD into RxD for
//                   usart_peer
//   PA17 (PAD[1]) - EA PE3: the select, a GPIO; unused (dark) in key y
//   PA18 (PAD[2]) - EA PE1: MISO for spi_peer; this board's TxD into the
//                   EA's RxD for usart_peer
//   PA19 (PAD[3]) - EA PE2: SCK; unused (dark) in key y
// plus a dedicated ground, both boards at 5 V, the EA board's PC4-PE0,
// PC5-PE1, PC6-PE2 and PC7-PE3 jumpers fitted. The SPI letters (key z)
// want the peer running `spi_peer`, the UART ones (key y) `usart_peer`.
// NOTHING LIT: no letter drives the LED's pad PB23.
//
// The letters (z: n o p w i; y: t u v), each of z and y run as seven passes
// (the shared body's control and six pressures):
//   n  the peer's command channel through the arbiter: a ping, ident (the
//      sanity byte and the avrxt port's kind), and ten more pings, every
//      frame at BAUD 119 = 200 kHz on the pump and its answer decoded by
//      the Client.
//   o  the matrix on the PUMP (32 frames a window at BAUD 23 = 1 MHz, a
//      rung under the 2 MHz the peer's client is exact to back to back):
//      the four modes in byte frames, and a DORD mismatch (the peer LSb
//      first, this end MSb) as an exact two-way bit reversal - byte-exact at
//      both ends; the data windows' bytes over their cycles printed against
//      the control's.
//   p  the ladders through the arbiter, 32 frames a window, the POLLED loop
//      and the RXC PUMP at BAUD 119, 47, 23 and 11 (200 kHz, 500 kHz,
//      1 MHz, 2 MHz) judged, and the polled loop at 7 and 5 (3 MHz, 4 MHz)
//      printed with its outcomes counted: test_samc_spi's letter d finds
//      the AVR client exact both ways to 2 MHz and wrong in its answers
//      from 3 MHz (the AVR's clarification 3.5.2), 4 MHz past the EA
//      client's CLK_PER/6 = 3.33 MHz. The pump stops at 2 MHz: above it a
//      character is shorter than its handler, which then holds the
//      interrupt level for the whole window (letter x). A frame at BAUD b
//      is 16 (b + 1) core cycles, 192 at 2 MHz: a 3000-cycle preemption of
//      the polled loop is 15.6 frames of a stopped bus. The verdict is
//      STRICT under pressure as in the control, and BUFOVF, read at every
//      reply before any other transaction (client_replied()), never set.
//   x  (OUTSIDE z, no verdict) the pressure's ticks against the pump's
//      windows: forty windows of 32 frames a shape under 97 us x 300 and
//      97 us x 3000 - the pump at 4, 3 and 2 MHz, the polled loop at
//      4 MHz - with TC4's ticks against the ruler's wall, the longest span
//      between two of its entries and the spans past one and a half
//      periods. The instrument behind letter p's pump ceiling.
//   w  the arbiter under pressure: the Client above it (20 requests of 6
//      frames on the pump, two outstanding), Minor below it (10 POLLED
//      requests of 8 frames, one outstanding) and, in a pressure pass,
//      Pressure itself (8 requests of 3 frames on the pump, posted from its
//      tick, one outstanding), all at 200 kHz inside ONE select window the
//      letter holds - the peer's exchange is one burst - of constant frames
//      (pattern_fixed). Every reply to its requester in its order, every
//      one spi_ok, nothing rejected, the per-bus timeout never firing, the
//      peer counting every frame exact.
//   i  dark: every object idle, TC4 stopped and its line quiet, no time
//      event armed, SERCOM1 given back (its line off, neither personality
//      live, the four link pads PORT inputs), the DMA engine unclaimed, the
//      LED's pad not an output, the ceiling the Console's own.
//   t  the UART, received: numbered, checked lines written by the Console
//      and turned around by the peer's ECHO at 460800 and 921600, assembled
//      by the LineReader from the interrupt receiver's ring and lent to
//      LineSink: 200 lines a leg, none lost, none out of order, none
//      corrupt, the port's own counters (ring overruns, BUFOVF, FE, PE, DMA
//      faults, skips of the ring's epoch, line overflows, torn lines) zero,
//      and the peer's count of the bytes it echoed exact. THE ECHO TURNS
//      EACH CHARACTER AROUND ALONE: the avrxt usart_peer waits four bit
//      times after a character's stop bit, sends it, and waits for its own
//      stop bit before it reads the next (usart_peer.cpp's run_echo_as()),
//      so a line written back to back overruns it within a line (letter q:
//      ovf 255). The writer therefore paces its CHARACTERS, 32 bit times
//      apart at least (echo_gap_bits says why), and the characters come
//      back one at a time, the reader told at each. THE WRITER IS THE
//      SLOWER END: this end's BAUD at 48 MHz is 45403 for 921600 (16x
//      arithmetic, 30.6.2.3: 921614 Bd, +16 ppm) and 55470 for 460800
//      (460785, -33 ppm), on OSC48M, 0.60 to 0.64 % slow against the EA's
//      quartz (docs/samc21/sercom.md, letter K's figure); the EA on its
//      20 MHz quartz runs BAUD 87 = 919540 and 174 = 459770 (64 x 20 MHz /
//      (16 x BAUD), -0.22 %). The echo outruns the writer by 0.36 to 0.42 %
//      at both rates, so no rate drift builds a backlog.
//   u  the UART, sent: 300 numbered lines back to back through write_bulk()
//      into the peer's SINK (which reads and counts, its loop some 25
//      cycles a character, usart_peer.cpp) at 460800 and 921600, through
//      the interrupt transmitter and then through the DMA transmit engine,
//      judged by the peer's report - the bytes counted, their 16-bit sum,
//      FE, PE and ORE zero - and the engine's abandoned blocks zero; the
//      wire's time printed against the control's.
//   v  the loan under preemption: t's leg at 921600 with LineSink spinning
//      6000 cycles a line, so Pressure lands inside the borrower's dispatch,
//      and the lines STREAMED - line k written once line k - 20 is back -,
//      the characters two at a time every 64 bit times, the pairs running
//      across the lines' ends, so that a line's end of line goes out with
//      the next line's first character and the port receives while it lends
//      (the receive ring's count read at the sink's entry and exit says how
//      often the stream moved). Two at a time is the peer's limit: its
//      receive buffer is two levels deep (docs/avrxt/usart.md), so it holds
//      the second while it turns the first around. Every line exact, every
//      line lent alone (the sink's queue empty at its entry, the sink never
//      nested in itself), the two buffers strictly alternating, and the
//      lender never reading what the borrower wrote over. THE RING HOLDS
//      THE WINDOW: the writer is at most 20 lines and the line in hand
//      ahead of what the sink has parsed, 21 x 72 = 1512 characters, under
//      the receive ring's 2048, so the ring cannot overflow whatever holds
//      LinePort off. A pressure pass judges that Pressure landed inside the
//      borrower at least once: the sink's dispatches add up to 200 x 6000 =
//      1.2 M cycles at least, 64 ticks of the longest period (18672 cycles)
//      on average, so a pass with none has a chance of e^-64.
//   q  (OUTSIDE y, no verdict) the peer's echo against the writer's pace:
//      40 lines a leg at both rates, written whole as the shared body's
//      echo_leg() writes them, then a character every 12 to 32 bit times -
//      what the peer counted and lost, what the sink took, the leg's time.
//
// RUNNING IT: `brio flash <the SAM board> test_samc_rubato_link`; then, the
// peer on spi_peer, `brio run <board> z`, and the peer on usart_peer,
// `brio run <board> y`.
//
// build: boards = c21j
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
#include "samc21/clock.hpp"
#include "samc21/dmac.hpp"
#include "samc21/nvic.hpp"
#include "samc21/pin.hpp"
#include "samc21/platform.hpp"
#include "samc21/sercom.hpp"
#include "samc21/spi.hpp"
#include "samc21/tc.hpp"
#include "samc21/ticker.hpp"
#include "util/line_reader.hpp"
#include "util/print.hpp"
#include "util/spi_bus.hpp"
#include "util/testbench.hpp"

// THE PROTOCOL HEADERS ARE SHARED, NOT COPIED: the far board's two
// instruments compile the same files.
#include "../../../avrxt/src/apps/spi_link.hpp"
#include "../../../avrxt/src/apps/usart_link.hpp"

/// OSC48M undivided: generator 0, the CPU, the SERCOM's core and TC4.
using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = SamRubatoPlatform;
constexpr uint32_t core_hz = SysClock::hz;
constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000u;

/// The kernel ticker read as CPU cycles (the file header).
struct Ruler {
    [[gnu::always_inline, gnu::flatten]] static uint32_t now() { return Ticker::cycles(); }
};

[[gnu::always_inline]] inline void spin(uint32_t cycles) {
    const uint32_t t0 = Ruler::now();
    while (Ruler::now() - t0 < cycles) {
    }
}

// ---- the console -------------------------------------------------------------------

constexpr UartPads console_pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'B', 30, PinFunction::d},
    .rx_pin = {'B', 31, PinFunction::d},
};
using Serial = Uart<5, console_pads>;
constexpr Serial serial;
TwoKeyTestBench<Serial, 'y'> bench;

// ---- the link's four pads ------------------------------------------------------------

using RxdPin = Pin<'A', 16>;   ///< PAD[0]: MOSI (z), RxD (y)
using CsPin = Pin<'A', 17>;    ///< PAD[1]: the select, a GPIO (z)
using TxdPin = Pin<'A', 18>;   ///< PAD[2]: MISO (z), TxD (y)
using XckPin = Pin<'A', 19>;   ///< PAD[3]: SCK (z)
static_assert(MUX_PA16C_SERCOM1_PAD0 == static_cast<uint8_t>(PinFunction::c));
static_assert(MUX_PA17C_SERCOM1_PAD1 == static_cast<uint8_t>(PinFunction::c));
static_assert(MUX_PA18C_SERCOM1_PAD2 == static_cast<uint8_t>(PinFunction::c));
static_assert(MUX_PA19C_SERCOM1_PAD3 == static_cast<uint8_t>(PinFunction::c));

/// One pad given back: PORT's, an input, no pull.
template <typename X>
void pad_dark() {
    X::release();
    X::input();
}

/// The four link pads dark, before any personality's init() (the file
/// header: no init() runs with a pad muxed).
void link_pads_dark() {
    pad_dark<RxdPin>();
    pad_dark<CsPin>();
    pad_dark<TxdPin>();
    pad_dark<XckPin>();
}

template <typename X>
bool pad_is_dark() {
    return !X::has_function() && !X::is_output();
}

// ---- the SPI link --------------------------------------------------------------------

/// HOST row 0x3 (32.8.1): DO (MOSI) PAD[0], SCK PAD[3], DI (MISO) PAD[2];
/// the SS pad named for the row and never claimed - the select is PA17, a
/// GPIO the adapter drives.
constexpr SpiPads host_pads{
    .data_out = SercomPad::pad0,
    .sck = SercomPad::pad3,
    .ss = SercomPad::pad1,
    .data_in = SercomPad::pad2,
    .data_out_pin = {'A', 16, PinFunction::c},
    .sck_pin = {'A', 19, PinFunction::c},
    .ss_pin = {'A', 17, PinFunction::c},
    .data_in_pin = {'A', 18, PinFunction::c},
};
using Hw = SpiHost<1, host_pads>;

/// The shared body's names for a request's rate and frame size. The rate
/// is this host's BAUD byte (f = 48 MHz / (2 (BAUD + 1)), 32.6.2.3), and a
/// frame is a byte (the file header); bits16 exists for the body's
/// vocabulary and is never posted here.
using SpiClock = uint8_t;
enum class SpiDataSize : uint8_t { bits8, bits16 };

/// The command channel's SCK: BAUD 119 = 200 kHz, 40 us a character
/// (test_samc_spi's command rate).
constexpr SpiClock link_clock = 119;
static_assert(spi_sck_hz(SysClock::hz, link_clock) == 200'000u);
constexpr const char* bus_clock_name = "GCLK0";

/// The adapter (the file header): the shared body's Request over
/// SpiHost<1>'s, the select its own.
struct Host {
    using Resource = Hw::Resource;

    struct Request {
        PinRef cs;
        Borrowed<const uint8_t, Lease::reply> tx;
        Borrowed<uint8_t, Lease::reply> rx;
        uint16_t len = 0;
        SpiDataSize bits = SpiDataSize::bits8;
        bool polled = false;
        uint8_t cs_setup_us = 0;
        SpiClock clock = link_clock;
        SpiMode mode = SpiMode::mode0;
        ReplyTo<SpiDone> reply;
    };

    /// The bit order is CTRLA.DORD, this host's configuration: MSb first
    /// for every request, and a request for LSb first refused.
    static bool lsb_first() { return false; }
    static bool bit_order(bool lsb) { return !lsb; }

    static void prime(SpiMode mode, SpiClock rate) { Hw::prime(mode, rate); }

    /// The arbiter's start(): the mode primed with the select high (a CPOL
    /// flip inside an open window is an edge for the client, Hw::prime()),
    /// the select lowered and its setup spent here, then the driver's
    /// start() with no select of its own, so its end of the transaction
    /// leaves the select low until close().
    static bool start(const Request& r) {
        Hw::prime(r.mode, r.clock);
        sel_ = r.cs;
        sel_.clear();
        if (r.cs_setup_us != 0u) {
            spin(static_cast<uint32_t>(r.cs_setup_us) * cycles_per_us);
        }
        Hw::Request q{};
        q.tx = r.tx;
        q.rx = r.rx;
        q.len = r.len;
        q.polled = r.polled;
        q.baud = r.clock;
        q.mode = r.mode;
        q.reply = r.reply;
        return Hw::start(q);
    }

    [[gnu::always_inline]] static bool isr() { return Hw::isr(); }
    static uint8_t status() { return Hw::status(); }

    /// The per-bus timeout's verb: the select raised first, then the
    /// driver's own recovery.
    static bool recover() {
        sel_.set();
        sel_ = PinRef{};
        return Hw::recover();
    }

    /// The select's hold and its rise, after a window's reply: 30 us more,
    /// test_samc_spi's measured hold, so the peer's polled loop has fetched
    /// the window's last character before SS rises. Main context (the
    /// Client's dispatch); a window without a select of its own passes.
    static void close() {
        if (!sel_.valid()) {
            return;
        }
        spin(30u * cycles_per_us);
        sel_.set();
        sel_ = PinRef{};
    }

    static inline PinRef sel_{};
};

/// SERCOM1 is the SPI host (key z), its vector the host's pump.
volatile bool spi_live = false;

// ---- the UART link -------------------------------------------------------------------

/// The crossed pair: TxD PAD[2], RxD PAD[0] (TXPO 0x1, RXPO 0x0).
constexpr UartPads link_pads{
    .tx = SercomPad::pad2,
    .rx = SercomPad::pad0,
    .tx_pin = {'A', 18, PinFunction::c},
    .rx_pin = {'A', 16, PinFunction::c},
};
/// The interrupt transmitter's port, the receive ring of 2048: letter v's
/// window (the file header). The DMA transmitter's port for u's second
/// half, its receive ring small (the sink echoes nothing).
using PortIrq = Uart<1, link_pads, 2048, 512>;
using PortDma = Uart<1, link_pads, 64, 512, DmaTxEngine>;
/// The Uart task owns SERCOM1's vector only while a leg runs; leg_dma
/// says which of the two ports it is.
volatile bool link_live = false;
volatile bool leg_dma = false;

/// The one name the shared body writes through (the file header): the
/// transmit verbs the leg's port's, the receive verbs the interrupt
/// transmitter's port's, the one LineReader reads.
struct LinkPort {
    static uint32_t write_bulk(std::span<const uint8_t> s) {
        return leg_dma ? PortDma::write_bulk(s) : PortIrq::write_bulk(s);
    }
    static bool tx_idle() { return leg_dma ? PortDma::tx_idle() : PortIrq::tx_idle(); }
    static std::span<const uint8_t> read_span() { return PortIrq::read_span(); }
    static void consume(uint32_t n) { PortIrq::consume(n); }
    static uint32_t rx_skips() { return PortIrq::rx_skips(); }
};

/// What a leg's transport counted, read after the leg (init() clears the
/// counters; the skip epoch is never cleared, so its move is counted from
/// the leg's start).
struct PortCounters {
    uint8_t ring_overruns = 0, bufovf = 0, fe = 0, pe = 0, dma_faults = 0;
    uint32_t skips = 0;
    bool clean() const {
        return ring_overruns == 0u && bufovf == 0u && fe == 0u && pe == 0u && dma_faults == 0u && skips == 0u;
    }
};

// ---- the pressure's clock ----------------------------------------------------------------

using Pulse = Tc<4>;
constexpr uint32_t pulse_per_us = SysClock::hz / 1'000'000u;   // generator 0, prescaler 1
static_assert(389u * pulse_per_us - 1u <= 0xFFFFu, "the longest period fits TC4's 16 bits");
constexpr const char* pressure_name = "TC4";
constexpr const char* peer_name = "the AVR64EA48";

// THE TARGET-FREE HALF IS SHARED, NOT COPIED: every family's link suite
// compiles the same body around its own engines.
#include "../../../avrxt/src/apps/rubato_link.hpp"

// =============================================================================
// What the shared body asks of this family
// =============================================================================

void pressure_arm(uint16_t period_us) {
    (void)Pulse::enable(false);
    (void)Pulse::set_count16(0);
    (void)Pulse::set_cc16(0, static_cast<uint16_t>(static_cast<uint32_t>(period_us) * pulse_per_us - 1u));
    Pulse::clear_flags(Pulse::overflow_flag);
    Pulse::arm(Pulse::overflow_flag);
}
void pressure_go() { (void)Pulse::enable(true); }
void pressure_halt() { (void)Pulse::enable(false); }
void pressure_quiet() {
    Pulse::disarm(Pulse::overflow_flag);
    Pulse::clear_flags(Pulse::overflow_flag);
    Nvic::clear_pending(Pulse::irq());
    spin(20'000u);   // the last tick's dispatch, if it was still running
}

/// The receive ring's count: LinePort, its one consumer, is below LineSink
/// and cannot run inside the sink's dispatch, so there the count moves
/// only with the receiver's writes.
uint32_t rx_stream_position() { return static_cast<uint32_t>(PortIrq::rx_pending()); }

uint32_t division(SpiClock c) { return 2u * (static_cast<uint32_t>(c) + 1u); }

/// The host's BUFOVF at a reply, latched and cleared (it stands until
/// written): the receive shapes keep no more than two characters in flight
/// against a two-level buffer (the file header), so a set flag is a frame
/// the host did not read. Then the select's rise (the adapter's close()).
volatile uint32_t ovf_at_reply = 0;
void client_replied() {
    if (Host::Resource::overflow_flag()) {
        ovf_at_reply = ovf_at_reply + 1u;
        Host::Resource::clear_status(SpiStatus::overflow);
    }
    Host::close();
}

uint32_t skips0 = 0;

PortCounters read_port_counters() {
    PortCounters c{};
    if (leg_dma) {
        c.ring_overruns = PortDma::rx_overruns();
        c.bufovf = PortDma::hw_overruns();
        c.fe = PortDma::frame_errors();
        c.pe = PortDma::parity_errors();
        c.dma_faults = PortDma::dma_faults();
        c.skips = PortDma::rx_skips() - skips0;
    } else {
        c.ring_overruns = PortIrq::rx_overruns();
        c.bufovf = PortIrq::hw_overruns();
        c.fe = PortIrq::frame_errors();
        c.pe = PortIrq::parity_errors();
        c.skips = PortIrq::rx_skips() - skips0;
    }
    return c;
}

void print_port(const PortCounters& c) {
    print(serial, "ring overruns ", c.ring_overruns, ", BUFOVF ", c.bufovf, ", FE ", c.fe, ", PE ", c.pe,
          ", DMA faults ", c.dma_faults, ", skips ", c.skips);
}

// =============================================================================
// The SPI personality
// =============================================================================

bool host_ok = false;

/// SERCOM1 as the SPI host, PA17 its select (high): the pads dark first.
bool spi_up() {
    if (spi_live) {
        return true;
    }
    link_pads_dark();
    CsPin::set();
    CsPin::output();
    host_ok = Hw::init(clock);
    spi_live = host_ok;
    return host_ok;
}

/// Given back: the line off, the instance disabled and gated, the pads and
/// the select dark.
void spi_down() {
    if (!spi_live) {
        return;
    }
    spi_live = false;
    Hw::release();
    link_pads_dark();
}

// =============================================================================
// The UART link: usart_link.hpp on SERCOM1 polled, between legs
// =============================================================================

namespace ul {

using Lk = Sercom<1>;
constexpr SercomBaud cmd_baud = sercom_closest_baud(SysClock::hz, link::command_baud).value();

link::Decoder dec;
const uint8_t no_payload[1] = {0};
bool quiet = false;
/// RxD on the SERCOM's pull-down while the peer searches (the file header).
bool rx_held = false;
/// The instance is the command channel now (reset and clocked by it).
bool cmd_up = false;

/// The command channel, 8N1 at 115200 on generator 0: from another
/// personality the pads go dark and the instance is reset; then the
/// configuration written with the instance disabled, the pads muxed, and
/// the enable (test_samc_uart's dut_up(): a receiver whose pad leaves the
/// mux while it runs reads a start bit).
bool command_mode() {
    dec.reset();
    if (!cmd_up) {
        Nvic::disable(Lk::irq());
        link_pads_dark();
        Lk::bus_clock(true);
        if (!Lk::core_clock(0) || !Lk::reset()) {
            return false;
        }
        cmd_up = true;
    }
    if (!Lk::configure(SercomUartConfig{.pads = link_pads, .baud = cmd_baud.reg, .sampling = cmd_baud.sampling})) {
        return false;
    }
    RxdPin::function(PinFunction::c, {.input_enable = true, .pull = rx_held ? PinPull::down : PinPull::none});
    TxdPin::function(PinFunction::c, {.input_enable = true});
    const bool ok = Lk::enable(true);
    Lk::flush_rx();
    Lk::clear_status(SercomStatus::receive_errors);
    Lk::clear_flags(SercomFlag::all);
    return ok;
}

/// The instance given back and the pads dark.
void release() {
    if (cmd_up) {
        Lk::release();
        cmd_up = false;
    }
    link_pads_dark();
}

void put1(uint8_t b) {
    const uint32_t t0 = Ticker::millis();
    while (!Lk::dre_flag() && Ticker::millis() - t0 < 50u) {
    }
    Lk::data(b);
}

void wait_tc() {
    const uint32_t t0 = Ticker::millis();
    while (!Lk::txc_flag() && Ticker::millis() - t0 < 50u) {
    }
}

/// A frame, polled: a character every 87 us into a two-level buffer, and
/// the longest preemption here 3000 cycles and the scheduler, under 70 us.
/// STATUS read before DATA (31.8.11).
bool recv_frame(link::Frame& out, uint16_t ms) {
    const uint32_t t0 = Ticker::millis();
    dec.reset();
    while (Ticker::millis() - t0 < ms) {
        if (!Lk::rxc_flag()) {
            continue;
        }
        const uint16_t st = Lk::status();
        const uint8_t b = static_cast<uint8_t>(Lk::data());
        const uint16_t err = static_cast<uint16_t>(st & SercomStatus::receive_errors);
        if (err != 0u) {
            Lk::clear_status(err);
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
    Lk::flush_rx();
    Lk::clear_flags(SercomFlag::txc);
    link::write_frame(put1, op, p, len);
    wait_tc();
    link::Frame f;
    if (!recv_frame(f, ms)) {
        return false;
    }
    return f.op == link::Op::ack && f.len == 2u && f.data[0] == link::byte_of(op);
}

/// Three attempts, each after a quiet longer than the peer's own
/// partial-frame reset (the protocol's recovery guarantee).
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

/// The peer found on the crossed pair, from whichever personality: for
/// as long as a peer that stopped believing its wiring takes to search
/// both again (usart_link.hpp's rediscover_ms, and some), RxD held down
/// until it answers.
bool ensure() {
    spi_down();
    quiet = true;
    rx_held = true;
    bool found = false;
    const uint32_t t0 = Ticker::millis();
    while (!found && Ticker::millis() - t0 < static_cast<uint32_t>(link::rediscover_ms) + 2'500u) {
        found = command_mode() && command(link::Op::ping);
    }
    rx_held = false;
    quiet = false;
    if (found) {
        (void)command_mode();   // the pull off: the far TxD drives the wire now
        return true;
    }
    print(serial, "  THE PEER DID NOT ANSWER: the AVR64EA48 must be running `usart_peer`", crlf);
    bench.verdict("the peer answers a ping (the peer board running usart_peer)", false);
    return false;
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
// The Uart task
// =============================================================================

/// The Uart task up at `rate` on SERCOM1, its vector live: the command
/// channel's instance handed over through dark pads (Uart::init() resets
/// it and muxes the pads after its enable).
bool task_up(uint32_t rate) {
    ul::cmd_up = false;
    link_pads_dark();
    link_live = true;
    const bool ok = leg_dma ? PortDma::init(clock, rate) : PortIrq::init(clock, rate);
    skips0 = leg_dma ? PortDma::rx_skips() : PortIrq::rx_skips();
    if (!ok) {
        link_live = false;
    }
    return ok;
}

void task_down() {
    if (leg_dma) {
        PortDma::release();
    } else {
        PortIrq::release();
    }
    link_live = false;
    link_pads_dark();
}

// =============================================================================
// n - the peer's command channel
// =============================================================================

void tn_peer_link() {
    if (!spi_up()) {
        bench.verdict("SERCOM1 comes up as the SPI host", false);
        return;
    }
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
        bench.verdict(pass_label(s), "ident is spi_peer's (the sanity byte), the avrxt port (0x01xx)",
                      got && d.sanity == spilink::ident_sanity &&
                          spilink::peer_kind(d.version) == spilink::peer_avrxt);
        bench.verdict(pass_label(s), "ten more frames answered, every one through the arbiter", pings == 10u);
        judge_pressure(s, st);
    }
    spi_down();
}

// =============================================================================
// o - the matrix
// =============================================================================

/// The matrix's rate: BAUD 23 = 1 MHz on the pump (the file header).
constexpr SpiClock matrix_rate = 23;
static_assert(spi_sck_hz(SysClock::hz, matrix_rate) == 1'000'000u);

void to_matrix() {
    if (!spi_up()) {
        bench.verdict("SERCOM1 comes up as the SPI host", false);
        return;
    }
    uint64_t control_bytes = 0, control_cycles = 0;
    for (uint8_t p = 0; p < pass_count; ++p) {
        const PassSpec& s = passes[p];
        pass_begin(s);
        if (!sl::need_peer()) {
            (void)pass_end(s);
            spi_down();
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
                bytes += 32u;
                cycles += o.cycles;
            } else {
                sl::print_outcome(e, o);
            }
        };
        for (uint8_t m = 0; m < 4u; ++m) {
            sl::Exchange e{};
            e.cfg.mode = m;
            e.host_mode = static_cast<SpiMode>(m);
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
        bench.verdict(pass_label(s), "four modes and a DORD mismatch: every window byte-exact at both ends on "
                                     "the pump",
                      good == total);
        bench.verdict(pass_label(s), "the command channel still answers after the matrix", ping);
        judge_pressure(s, st);
    }
    spi_down();
}

// =============================================================================
// p - the ladders
// =============================================================================

constexpr SpiClock rungs[] = {119, 47, 23, 11, 7, 5};
/// Judged to BAUD 11 = 2 MHz; BAUD 7 = 3 MHz and 5 = 4 MHz printed on the
/// polled loop, its outcome counted (the file header: the peer's client).
/// The pump climbs to 2 MHz alone: above it a character (128 cycles at
/// 3 MHz, 96 at 4) is shorter than the pump's handler, which re-enters at
/// its own exit for the whole window, a polled loop at the interrupt level
/// (docs/samc21/spi.md), and TC4 - the same priority, a higher exception
/// number - waits the window out: letter x measured 10300 cycles between
/// two of its entries there, a tick of the 4656-cycle period merged into
/// the next one's flag, against 5348 at 2 MHz.
constexpr SpiClock judged_top = 11;

/// A frame's core cycles at BAUD `c`: 8 bits x 2 (c + 1).
uint32_t frame_cycles(SpiClock c) { return 8u * division(c); }

/// The informational rungs' outcomes, 3 MHz and 4 MHz, printed after the
/// pass.
uint8_t top_exact[2] = {};
uint8_t top_total[2] = {};

/// One ladder on one path: every judged rung byte-exact, the first failure's
/// outcome printed; the informational rungs counted apart.
bool climb(bool polled, uint8_t seed) {
    bool all = true;
    uint8_t k = 0;
    for (const SpiClock c : rungs) {
        sl::Exchange e{};
        e.rate = c;
        e.polled = polled;
        e.count = 32;
        e.seed_a = static_cast<uint8_t>(seed + 9u * k);
        e.seed_b = static_cast<uint8_t>(seed + 0x40u + 5u * k);
        ++k;
        if (c < judged_top && !polled) {
            continue;   // the pump above its handler-bound rate (judged_top's comment)
        }
        const sl::Outcome o = sl::run(e);
        const bool ok = sl::exact(e, o);
        if (c < judged_top) {
            const uint8_t r = c == 7u ? 0u : 1u;
            ++top_total[r];
            if (ok) {
                ++top_exact[r];
            }
            continue;
        }
        if (!ok) {
            print(serial, "    ", polled ? "polled loop" : "RXC pump", " (a frame ", frame_cycles(c), " cycles):", crlf);
            sl::print_outcome(e, o);
        }
        all = all && ok;
    }
    return all;
}

void tp_ladders() {
    if (!spi_up()) {
        bench.verdict("SERCOM1 comes up as the SPI host", false);
        return;
    }
    print(serial, "  the polled loop and the pump keep two characters in flight against a two-level receive "
                  "buffer; 32 frames a window at BAUD 119, 47, 23, 11 judged, 7 and 5 printed on the polled loop",
          crlf);
    for (uint8_t p = 0; p < pass_count; ++p) {
        const PassSpec& s = passes[p];
        pass_begin(s);
        if (!sl::need_peer()) {
            (void)pass_end(s);
            spi_down();
            return;
        }
        ovf_at_reply = 0;
        top_exact[0] = top_exact[1] = 0;
        top_total[0] = top_total[1] = 0;
        const bool polled = climb(true, 0x21);
        const bool pump = climb(false, 0x39);
        const uint32_t ovf = ovf_at_reply;
        const bool ping = sl::command(spilink::Op::ping);
        const PassStats st = pass_end(s);
        print_pass(s, st);
        print(serial, "    the host's BUFOVF at a reply over the pass: ", ovf, "; the polled loop at 3 MHz exact ",
              top_exact[0], " of ", top_total[0], ", at 4 MHz ", top_exact[1], " of ", top_total[1], crlf);
        // A dispatch above the arbiter holds the polled loop off for its
        // whole length; with two characters in flight against a two-level
        // receive buffer that is a gap on the bus and never a lost frame
        // (the file header), so the verdict under pressure is the control's.
        bench.verdict(pass_label(s), "THE POLLED LOOP through the arbiter: 32 frames byte-exact both ways at "
                                     "every BAUD to 2 MHz",
                      polled);
        bench.verdict(pass_label(s), "THE RXC PUMP: 32 frames byte-exact both ways at every BAUD to 2 MHz", pump);
        bench.verdict(pass_label(s), "the host's BUFOVF never set at a reply: no frame left unread", ovf == 0u);
        bench.verdict(pass_label(s), "the command channel survives the ladders", ping);
        judge_pressure(s, st);
    }
    spi_down();
}

// =============================================================================
// x - the pressure's ticks against the pump's windows (outside z, no verdict)
// =============================================================================

/// TC4's entries stamped while letter x runs: the longest span between two
/// in a row, and how many spans passed one and a half periods (a tick
/// merged into the next one's flag).
volatile bool x_live = false;
volatile uint32_t x_last = 0;
volatile uint32_t x_most = 0;
volatile uint32_t x_long = 0;
volatile uint32_t x_period = 0;

/// The instrument behind letter p's pressure verdict at the fast pump
/// rungs: one shape at a time, forty windows under 97 us x 300 and forty
/// under 97 us x 3000, TC4's ticks against the ruler's wall, the longest
/// span between two of its entries and the spans past one and a half
/// periods printed. No verdict.
void tx_ticks() {
    struct Shape {
        const char* name;
        SpiClock rate;
        bool polled;
    };
    static const Shape shapes[] = {
        {"RXC pump, BAUD 5 (4 MHz)", 5, false},
        {"RXC pump, BAUD 7 (3 MHz)", 7, false},
        {"RXC pump, BAUD 11 (2 MHz)", 11, false},
        {"polled loop, BAUD 5 (4 MHz)", 5, true},
    };
    constexpr uint16_t windows = 40;
    if (!spi_up() || !sl::need_peer()) {
        spi_down();
        return;
    }
    for (const Shape& sh : shapes) {
        for (uint8_t press = 0; press < 2u; ++press) {
            const PassSpec& s = passes[press == 0u ? 1u : 4u];
            x_most = 0;
            x_long = 0;
            x_last = Ruler::now();
            x_period = static_cast<uint32_t>(s.period_us) * cycles_per_us;
            x_live = true;
            pass_begin(s);
            uint16_t bad = 0;
            uint32_t most_window = 0;
            for (uint16_t k = 0; k < windows; ++k) {
                sl::Exchange e{};
                e.rate = sh.rate;
                e.polled = sh.polled;
                e.count = 32;
                e.seed_a = static_cast<uint8_t>(0x31u + 3u * k);
                e.seed_b = static_cast<uint8_t>(0x75u + 7u * k);
                const sl::Outcome o = sl::run(e);
                if (!sl::exact(e, o)) {
                    ++bad;
                }
                if (o.cycles > most_window) {
                    most_window = o.cycles;
                }
            }
            const PassStats st = pass_end(s);
            x_live = false;
            const uint32_t period = static_cast<uint32_t>(s.period_us) * cycles_per_us;
            print(serial, "  ", sh.name, " [", pass_label(s), "]: ", st.irqs, " ticks against ", st.wall / period,
                  " periods of wall; the longest span between two TC4 entries ", x_most, " cycles (a period ", period,
                  "), spans past 1.5 periods ", x_long, "; the longest window post to reply ", most_window,
                  " cycles; ", bad, " of ", windows, " windows not exact", crlf);
        }
    }
    spi_down();
}

// =============================================================================
// w - the arbiter under pressure
// =============================================================================

void tw_arbiter() {
    if (!spi_up()) {
        bench.verdict("SERCOM1 comes up as the SPI host", false);
        return;
    }
    for (uint8_t p = 0; p < pass_count; ++p) {
        const PassSpec& s = passes[p];
        const bool pressing = s.period_us != 0u;
        pass_begin(s);
        if (!sl::need_peer()) {
            (void)pass_end(s);
            spi_down();
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
            // the protocol pays on each side of it - the second the hold the
            // peer's polled loop wants before SS rises (the adapter's
            // close()). The bus is idle here: no request is outstanding.
            sl::settle();
            Host::prime(SpiMode::mode0, link_clock);
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
    spi_down();
}

// =============================================================================
// i - dark
// =============================================================================

void ti_dark() {
    bench.verdict("every object above the Console idle: every queue empty",
                  Pressure::queue.empty() && LineSink::queue.empty() && Client::queue.empty() &&
                      SpiArb::queue.empty() && Minor::queue.empty() && LinePort::queue.empty());
    bench.verdict("TC4 stopped, its overflow interrupt off and its line quiet",
                  !Pulse::enabled() && (Pulse::armed() & Pulse::overflow_flag) == 0u && !Nvic::pending(Pulse::irq()));
    bench.verdict("no time event is armed (the arbiter's timeout among them)", !TE::next_deadline().has_value());
    bench.verdict("SERCOM1 given back: its line off, neither personality live, the four link pads PORT inputs",
                  !Nvic::enabled(Hw::Resource::irq()) && !spi_live && !link_live && !ul::cmd_up &&
                      pad_is_dark<RxdPin>() && pad_is_dark<CsPin>() && pad_is_dark<TxdPin>() &&
                      pad_is_dark<XckPin>());
    bench.verdict("the DMA transmit engine unclaimed", DmaTxEngine::owner() == 0u);
    bench.verdict("the LED's pad PB23 is not an output", !Pin<'B', 23>::is_output());
    bench.verdict("the ceiling is the Console's and interrupts are enabled",
                  P::ceiling == console_index && P::interrupts_enabled());
}

// =============================================================================
// t, u, v - the UART
// =============================================================================

constexpr uint32_t uart_rates[] = {460'800u, 921'600u};
constexpr uint16_t echo_lines = 200;
constexpr uint16_t sink_lines = 300;

/// t and v's leg against this peer: the shared body's echo_leg() with
/// every CHARACTER paced, written `gap_bits` bit times after the one
/// before (a floor: a preemption of the writer only lengthens a gap), or
/// `burst` characters back to back every `burst` x `gap_bits`. The
/// avrxt usart_peer's echo turns each character around alone - four bit
/// times of turnaround guard, the character sent, its last stop bit
/// awaited (usart_peer.cpp's run_echo_as(), line_talk(), line_listen()) -
/// so a run written back to back overruns its two-level receive buffer
/// within a line (measured: ovf 255, a third of a leg's bytes lost). The
/// shared body writes a line whole; this copy differs from it in the
/// writer's loop alone (the report's header change).
LegResult echo_leg_paced(uint32_t rate, uint16_t lines, uint16_t seq0, uint8_t window, uint8_t gap_bits,
                         uint8_t burst = 1) {
    static_assert(echo_lines * 72u * 400u / 1000u + 1000u <= 0xFFFFu, "the peer's deadline fits its 16 bits");
    // One character or two at a time: the peer queues one while it turns
    // the one before around (its receive buffer is two levels deep,
    // docs/avrxt/usart.md).
    LegResult g{};
    for (uint16_t k = 0; k < lines; ++k) {
        g.bytes += line_with_eol(static_cast<uint16_t>(seq0 + k));
    }
    g.bytes += 1u;   // the leading end of line
    link::Params a{};
    a.cfg.rate = rate;
    a.count = static_cast<uint16_t>(g.bytes);
    // The peer's deadline, for a leg that fails: 400 us a character and a
    // second. A paced character's measured cost is the writer's and the
    // reader's per-character dispatch, not the wire's - 57 to 99 us with
    // the pressure silent (letter q), 225 us at worst under 97 us x 3000
    // (letter t at 921600, 2.03 s for 9000 characters) - and 400 us is
    // nearly twice that.
    a.ms = static_cast<uint16_t>(g.bytes * 400u / 1000u + 1000u);
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
    const uint32_t gap = static_cast<uint32_t>(gap_bits) * burst * (core_hz / rate);
    const uint32_t t0 = Ruler::now();
    // The bursts run across the lines' ends - a line's end of line goes out
    // with the next line's first character -, so a character is on its way
    // while the line before it is lent; the last burst flushed at the end.
    char held[2] = {};
    uint8_t held_n = 0;
    auto put = [&held, &held_n, gap, burst](const char* s, uint8_t n) {
        for (uint8_t i = 0; i < n; ++i) {
            held[held_n++] = s[i];
            if (held_n == burst) {
                held_n = 0;
                if (!write_line(held, burst)) {
                    return false;
                }
                spin(gap);
            }
        }
        return true;
    };
    g.wrote = put("\n", 1);
    for (uint16_t k = 0; k < lines && g.wrote; ++k) {
        if (window != 0u && k >= window) {
            const uint32_t back = static_cast<uint32_t>(k - window) + 1u;
            (void)wait_until([back] { return LineSink::good + LineSink::corrupt + LineSink::backwards >= back; }, 20);
        }
        const uint8_t n = line_with_eol(static_cast<uint16_t>(seq0 + k));
        g.wrote = put(line_buf, n);
    }
    if (g.wrote && held_n != 0u) {
        g.wrote = write_line(held, held_n);
    }
    (void)wait_until([lines] { return LineSink::good + LineSink::corrupt + LineSink::backwards >= lines; }, 300);
    g.cycles = Ruler::now() - t0;
    wait_ms(5);   // the last edge's dispatch, and anything a slip would still deliver
    g.c.port = read_port_counters();
    g.c.line_overflows = static_cast<uint8_t>(LinePort::line_overflows() - overflows0);
    g.c.torn = LinePort::torn_lines() - torn0;
    task_down();
    g.have = ul::peer_report(g.r, 0);
    return g;
}

/// The pace t and v write at: 32 bit times a character. The peer's
/// turnaround of one character has a fixed part read from its source -
/// the four-bit guard after the stop bit's middle, the start of its echo
/// within a bit, the ten bits of the echo, its own stop bit awaited: 15
/// bit times - and the loop's own instructions between two looks at
/// RXCIF (the guard's arithmetic, line_talk(), line_listen()), not counted
/// here; 32 is twice the fixed part and leaves the instructions 17 bit
/// times, 369 cycles of its 20 MHz at 921600 and 738 at 460800. Measured,
/// letter q found the peer clean from a character every 12 bit times,
/// where the writer's own cost already spaces them past 30 bit times at
/// 460800 and 50 at 921600 (a character's write and the reader's dispatch
/// for its echo). The two rates' 0.4 % (the file header) moves none of it.
constexpr uint8_t echo_gap_bits = 32;

/// (OUTSIDE y, no verdict) the peer's echo against the writer's pace: 40
/// lines a leg at both rates, written whole as the shared body writes
/// them and then with the characters `gap` bit times apart, the pressure
/// silent - what the peer counted and lost, what the sink took.
void tq_pace() {
    static const uint8_t gaps[] = {12, 14, 16, 18, 20, 24, 32};
    if (!ul::ensure()) {
        ul::release();
        return;
    }
    leg_dma = false;
    for (const uint32_t rate : uart_rates) {
        {
            // The shared body's leg, each line written whole: the row the
            // pace exists for.
            LineSink::spin_cycles = 0;
            const LegResult g = echo_leg(rate, 40, 100u);
            print(serial, "  ", rate, " each line whole (the shared body's echo_leg()): peer count ", g.r.count,
                  " of ", g.bytes, " ferr ", g.r.ferr, " ovf ", g.r.ovf, "; sink ", LineSink::good, " good of 40, ",
                  LineSink::corrupt, " corrupt", crlf);
        }
        for (const uint8_t gap : gaps) {
            LineSink::spin_cycles = 0;
            const LegResult g = echo_leg_paced(rate, 40, 100u, 0, gap);
            print(serial, "  ", rate, " a character every ", gap, " bit times: peer count ", g.r.count, " of ", g.bytes,
                  " ferr ", g.r.ferr, " ovf ", g.r.ovf, "; sink ", LineSink::good, " good of 40, ", LineSink::corrupt,
                  " corrupt; ", g.cycles / cycles_per_us, " us, the wire's ", wire_cycles(g.bytes, rate) / cycles_per_us,
                  crlf);
        }
    }
    ul::release();
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
        leg_dma = false;
        for (uint8_t k = 0; k < 2u; ++k) {
            LineSink::spin_cycles = 0;
            const LegResult g =
                echo_leg_paced(uart_rates[k], echo_lines, static_cast<uint16_t>(1000u * k + 7u * p), 0, echo_gap_bits);
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
    uint32_t control[2][2] = {};
    uint32_t control_bytes[2][2] = {};
    static const char* const said[2][2] = {
        {"460800, the interrupt transmitter: the peer counted every byte of the blast, its sum exact, FE PE ORE "
         "zero",
         "921600, the interrupt transmitter: the peer counted every byte of the blast, its sum exact, FE PE ORE "
         "zero"},
        {"460800, the DMA transmitter: the peer counted every byte of the blast, its sum exact, FE PE ORE zero, "
         "no block abandoned",
         "921600, the DMA transmitter: the peer counted every byte of the blast, its sum exact, FE PE ORE zero, "
         "no block abandoned"},
    };
    for (uint8_t p = 0; p < pass_count; ++p) {
        const PassSpec& s = passes[p];
        pass_begin(s);
        if (!ul::ensure()) {
            (void)pass_end(s);
            ul::release();
            return;
        }
        for (uint8_t e = 0; e < 2u; ++e) {
            leg_dma = e != 0u;
            for (uint8_t k = 0; k < 2u; ++k) {
                const uint32_t rate = uart_rates[k];
                const LegResult g =
                    sink_leg(rate, sink_lines, static_cast<uint16_t>(2000u * k + 500u * e + 11u * p));
                const uint8_t faults = leg_dma ? PortDma::dma_faults() : 0u;
                print_leg(leg_dma ? "blast, DMA transmitter" : "blast, interrupt transmitter", rate, g);
                if (p == 0u) {
                    control[e][k] = g.cycles;
                    control_bytes[e][k] = g.bytes;
                }
                // Per byte: the lines' lengths follow their numbers, which
                // move from pass to pass.
                print(serial, "    a byte's time against the control's: ",
                      g.cycles != 0u && control_bytes[e][k] != 0u
                          ? static_cast<uint32_t>(static_cast<uint64_t>(control[e][k]) * g.bytes * 1000u /
                                                  (static_cast<uint64_t>(g.cycles) * control_bytes[e][k]))
                          : 0u,
                      " permille of its rate; blocks abandoned ", faults, crlf);
                bench.verdict(pass_label(s), said[e][k],
                              g.acted && g.up && g.wrote && g.have && g.r.count == g.bytes && g.r.sum == g.sum &&
                                  g.r.ferr == 0u && g.r.perr == 0u && g.r.ovf == 0u && faults == 0u);
            }
        }
        leg_dma = false;
        const PassStats st = pass_end(s);
        ul::release();
        print_pass(s, st);
        judge_pressure(s, st);
    }
}

void tv_loan() {
    // 921600 and a window of 20 lines: the file header's arithmetic.
    constexpr uint32_t rate = 921'600u;
    constexpr uint8_t window = 20;
    static_assert((window + 1u) * 72u <= 2048u, "the receive ring holds the whole window");
    for (uint8_t p = 0; p < pass_count; ++p) {
        const PassSpec& s = passes[p];
        pass_begin(s);
        if (!ul::ensure()) {
            (void)pass_end(s);
            ul::release();
            return;
        }
        leg_dma = false;
        LineSink::spin_cycles = 6000;
        const LegResult g =
            echo_leg_paced(rate, echo_lines, static_cast<uint16_t>(3000u + 13u * p), window, echo_gap_bits, 2);
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
// the Console's banner
// =============================================================================

bool clock_ok = false;
bool tick_ok = false;
bool pulse_ok = false;

void console_banner() {
    print(serial, crlf, "boot: clk=", clock_ok ? "OSC48M" : "FAILED", " tick=", tick_ok ? "SysTick" : "FAILED",
          " pressure=", pulse_ok ? "TC4" : "FAILED", " ruler=SysTick cycles sercom1=dark between letters", crlf);
    print(serial, "test_samc_rubato_link - the SPI and UART stacks under Rubato against the AVR64EA48 "
                  "(z: spi_peer, y: usart_peer)",
          crlf);
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------
//
// An unbound vector here is a SILENT death - the crt's default handler is
// a spin loop - so every line this suite raises is bound, and the SVC
// vector by the crt's own name, SVCall_Handler.
extern "C" void SERCOM5_Handler() {
    if (Serial::isr()) {
        brio::post<Console>(ConsoleEvent{console_rx});
    }
}

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

extern "C" [[gnu::naked]] void PendSV_Handler() { brio::scheduler_exception<K>(); }
extern "C" [[gnu::naked]] void SVCall_Handler() { brio::scheduler_return(); }

/// The pressure: TC4's overflow, where it landed, and the post.
extern "C" void TC4_Handler() {
    Pulse::clear_flags(Pulse::overflow_flag);
    if (x_live) {
        // Letter x's stamp (its comment): the span since the last entry.
        const uint32_t now = Ruler::now();
        const uint32_t span = now - x_last;
        x_last = now;
        if (span > x_most) {
            x_most = span;
        }
        if (2u * span > 3u * x_period) {
            x_long = x_long + 1u;
        }
    }
    pressure_landed();
}

/// SERCOM1's one line, the live personality's: the SPI host's pump, or the
/// Uart task's receiver and (the interrupt transmitter's) feed. The DMA
/// transmitter's port receives nothing a leg reads, so its edge posts
/// nothing.
extern "C" void SERCOM1_Handler() {
    if (spi_live) {
        if (Host::isr()) {
            done_from_dma = false;
            brio::post<SpiArb>(brio::TransferDone{Host::status()});
        }
    } else if (link_live) {
        if (leg_dma) {
            (void)PortDma::isr();
        } else if (PortIrq::isr()) {
            brio::post<LinePort>(brio::RxActivity{});
        }
    }
}

/// The DMAC's one line: the transmit engine's completion, for the port
/// that holds its claim.
extern "C" void DMAC_Handler() {
    if (brio::DmaTxEngine::owner() != 0u) {
        PortDma::dma_isr();
    }
}

int main() {
    clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    tick_ok = brio::Ticker::init(clock);
    link_pads_dark();
    pulse_ok = Pulse::init(0) &&
               Pulse::configure(brio::TcConfig{.mode = brio::TcMode::count16,
                                               .prescaler = brio::TcPrescaler::div1,
                                               .waveform = brio::TcWaveform::match_frequency});
    brio::Nvic::enable(Pulse::irq());
    brio::scheduler_init();

    bench.letter('n', "the peer's command channel through the arbiter", tn_peer_link);
    bench.letter('o', "the matrix on the pump: four modes, a DORD mismatch", to_matrix);
    bench.letter('p', "the ladders: the polled loop and the pump, every BAUD to 4 MHz", tp_ladders);
    bench.letter('w', "the arbiter under pressure: three requesters, one bus", tw_arbiter);
    bench.letter('i', "dark: every object idle, SERCOM1 given back, nothing lit", ti_dark);
    bench.letter('x', "the pressure's ticks against the pump's windows, no verdict", tx_ticks, false);
    bench.letter('q', "the peer's echo against the writer's pace, no verdict", tq_pace, false);
    bench.second_letter('t', "the UART received: numbered lines through the peer's echo", tt_lines);
    bench.second_letter('u', "the UART sent: a blast into the peer's sink, both transmitters", tu_blast);
    bench.second_letter('v', "the loan under preemption: a slow borrower at 921600", tv_loan);

    (void)serial_ok;
    brio::enable_interrupts();
    K::run();
}
