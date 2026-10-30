// test_x035_rubato_link - the bench suite of the SPI and UART stacks of the
// CH32X035 under Rubato, the preemptive kernel (kernel/rubato.hpp):
// util/spi_bus.hpp's arbiter over ch32x035/spi.hpp's SpiHost and its DMA
// engines, and util/line_reader.hpp's LineReader over ch32x035/usart.hpp's
// Uart and its receive engine, run against the desk's CH32V006K8U6 and its
// unchanged instruments (ch32v00x/src/apps/spi_peer.cpp, usart_peer.cpp)
// while an object of the highest priority preempts them at moments the
// traffic does not choose. Every verdict is the traffic judged byte-exact,
// as test_x035_spi's and test_x035_usart's two-board letters judge it;
// nothing here changes a driver or a service.
//
// A test_<target>_<subject> suite in the bench grammar (util/testbench.hpp)
// that runs inside the kernel it tests, as test_x035_rubato does: the
// program is one Rubato pack of seven objects - Pressure, LineSink, Client,
// SpiArb, Minor, LinePort and Console, from the highest priority down - and
// everything of it that names no register, the pack, the passes, the
// numbered lines, the window client and the UART legs, is the link suites'
// shared body (avrxt/src/apps/rubato_link.hpp, whose header describes each
// object and the passes). What is this chip's is here, each choice from
// the reference manual's chapters (RM 16 the SPI, RM 14 the USART, RM 9
// the DMA, RM 12 the timer) and the driver headers:
//
//   THE SPI HOST. SPI1 on its one column of this package (SCK PA5, MISO
//   PA6, MOSI PA7, the select PA4 a GPIO the Request carries; the peer's
//   is its hardware NSS input), the DMA engines on the instance's own two
//   channels (RM 9.2.3: 2 receives, 3 transmits), half-word beats for
//   16-bit frames, the driver's default placement (InFlash) and hold-off
//   (380 cycles). NO FIFO (16.2.2): one buffer each way, so the receive
//   shapes keep two frames in flight only where a frame outlasts the
//   hold-off and the pump's path to the read, 380 + 57 cycles (spi.hpp,
//   the write-ahead threshold): a byte frame from /64 (512 cycles at
//   HCLK = 48 MHz) and a 16-bit one from /32 (512). An asynchronous data
//   phase rides the engines from dma_min_frames (2) on, a polled one only
//   with an in buffer and from dma_min_frames_polled (22) on, and a 16-bit
//   phase at an odd address never (RM 9.3.6: a wide access ignores the low
//   address bit): so the POLLED LOOP is a window of 8 frames in either
//   width, the PUMP a 16-bit window from an odd address, the ENGINES a
//   window of 32 byte frames. The polled loop runs inside the arbiter's
//   dispatch: a dispatch above the arbiter holds it off for its whole
//   length, which the hold-off does not count, and where two frames are in
//   flight and that outlasts a frame the one-deep receive buffer loses one
//   (16.2.7). THIS HOST'S POLLED RECEIVE LOOP DOES NOT READ OVR (spi.hpp,
//   poll_duplex(): RXNE alone): it waits for the frame the overrun took
//   until the transaction's budget of poll turns runs out, and the
//   transaction ends spi_stalled with the receive buffer flushed - never
//   spi_ok over a run with a hole, because the loop cannot count the
//   frames it asked for without the one that never came. The pump reads
//   OVR after DATAR and ends spi_overrun.
//   THE ANSWER MODE. The CH32V006's client answers in mode 3 and not in
//   mode 0 (its spi_peer's header: mode-0 answers come back a bit early in
//   a window that follows another answer), so it serves spi_link's ack,
//   nak, ident and report windows in mode 3, the protocol's alternate
//   answer mode. The shared body's rendezvous finds it so (its need_peer():
//   a ping in mode 0 fails, the next in mode 3 is answered) and clocks
//   every answer window in mode 3 from there; the command windows stay
//   mode 0.
//   THE UART. USART4 on its default column, PB0 (TX) and PB1 (RX), both
//   engines on the port's channels (RM 9.2.3: 1 transmits, 8 receives) - the
//   receive engine a circular ring of 2048 over the whole storage, never
//   re-armed, the port told at the idle line, at RXNE for a one-frame burst
//   and at the half and full marks of a lap; the transmit engine a block a
//   contiguous run of the ring, write_bulk() pumping it once
//   (docs/ch32x035/usart.md). No receiver time-out on this block. The
//   port's placement the default, InFlash.
//   THE CLOCK. This chip has one clock root, the 48 MHz HSI; the far chip
//   runs its 24 MHz crystal doubled. Both divide 48 MHz by one divisor at
//   every rate (BRR 104 at 460800, 52 at 921600), so the two rates differ by
//   the HSI's error alone - which the UART key measures before its legs
//   (x035_link_dut.hpp's calibrate(): the far chip times 3999 gapless
//   frames of this end at 1 Mbaud against its crystal) and judges against
//   the slack the echo has (letters t and v below).
//   THE DMA RELEASE CONTRACT (docs/ch32x035/dma.md, "A released
//   requester"): a raised request outlives its peripheral's release unless
//   the block is reset - so every hand of USART4 between the polled
//   command channel and the Uart task goes through a reset: Uart::release()
//   pulses it before the gate, the command path's u4_open() pulses it
//   before it programs the port, and task_up() pulses it before the Uart
//   task's init(), the TX pad parked high across it. The SPI host raises
//   its requests around the engines and nowhere else (spi.hpp), and letter
//   i reads every channel the image uses disabled and every request enable
//   clear.
//   THE PRESSURE. TIM2's update interrupt, 16 bits on HCLK = 48 MHz at
//   prescaler 0: 97 / 193 / 389 us are 4656 / 9264 / 18672 counts. Its
//   dispatch, a 3000-cycle spin, road 1 (154 to 160 cycles with the
//   scheduler in SRAM, 307 to 311 from the flash: docs/ch32x035/platform.md)
//   and the way back,
//   is shorter than the shortest period's 4656 cycles, so no tick finds the
//   last one still queued; at 97 us x 3000 it takes 64 % of the core.
//   TIM2's update request is DMA channel 2's row (dma_engine.hpp), the SPI's
//   receive channel, its channel 3 compare channel 1's (USART4's transmit)
//   and its trigger and COM requests channel 8's (USART4's receive ring):
//   every DMA enable of DMAINTENR (bits 8 to 14, 12.4.4) stays clear, and
//   letter i reads them clear.
//   THE RULER. The kernel ticker's composed cycles() (the tick count and
//   the STK counter's position, ch32x035/ticker.hpp), which the shared
//   body's passes and spans need monotone across a tick; spin() reads the
//   STK counter bare, one load, the restart at the compare folded in.
//   THE PLACEMENT. This image runs the scheduler path from SRAM, the
//   project's default (`// build: scheduler` absent):
//   test_x035_rubato_link_flash is the same source with the path in the
//   flash. What the release map shows the compiler carried into SRAM with
//   activate() is the platform page's ("The drivers under the preemptive
//   kernel", docs/ch32x035/platform.md).
//   THE UART KEY'S COMMAND CHANNEL. USART4 polled at 115200 between legs
//   through x035_link_dut.hpp's reviewed helpers: a character every 87 us
//   (4167 cycles), against the longest preemption there, a 3000-cycle spin
//   and the scheduler.
//   THE SUPPLY at boot: the voltage detector at 3.0 V (pwr.hpp); an image
//   that would drive a link pad from a sagging rail registers no letter
//   and says so.
//
// THE WIRES, fixed on the desk (docs/boards/ch32x035-evt-f8u6.md,
// docs/boards/ch32v006k8.md; test_x035_spi's and test_x035_usart's pair):
//   SPI    X035 PA5 - V006 PC5 (SCK), PA7 - PC6 (MOSI), PA6 - PC7 (MISO),
//          PA4 - PC1 (the select); the V006 module's PC6-PC7 loopback
//          jumper OFF
//   UART   X035 PB0 (USART4 TX) -> V006 PB1 (USART2 RX, column 4), X035
//          PB1 <- V006 PB0
// The SPI letters (key z) want the peer running `spi_peer`, the UART ones
// (key y) `usart_peer`. NOTHING LIT: no letter drives the LED's pad PA0.
//
// The letters (z: n o p w i; y: t u v), each of z and y run as seven passes
// (the shared body's control and six pressures):
//   n  the peer's command channel through the arbiter: a ping, ident (the
//      sanity byte, the CH32V006's kind 0x06 and its label), and ten more
//      pings, every frame on the engines at /256 = 187.5 kHz and its answer
//      decoded by the Client.
//   o  the matrix on the DMA engines (32 frames a window at /32 = 1.5 MHz,
//      against the peer's DMA serve): modes 0 and 3 in 8- and 16-bit frames,
//      LSb first in both widths, and a bit-order mismatch as an exact
//      two-way reversal - byte-exact at both ends; the data windows' bytes
//      over their cycles printed against the control's. Modes 1 and 2 are
//      not in it: the CH32V006's client is served in neither direction there
//      (docs/ch32x035/spi.md, "Against a foreign client").
//   p  the ladders through the arbiter, every BR code from /256 to /2: the
//      POLLED LOOP in 16-bit frames (8, from an odd address) and in byte
//      frames (8), the RXNE PUMP in 16-bit frames (16, from an odd address)
//      and the DMA ENGINES (32 byte frames), against the peer's DMA serve.
//      Judged to /2 = 24 MHz, table 3-19's ceiling for both roles, where
//      test_x035_spi's ladder judges both host paths in byte frames. In the
//      control every rung is byte-exact on every path. Under pressure the
//      pump and the engines are, and the polled loop is byte-exact or cut
//      short by a detected failure - spi_stalled, this loop's (above) - at a
//      rung where two frames are in flight: never a run with a hole handed
//      back spi_ok, never a failure where one frame is.
//   w  the arbiter under pressure: the Client above it (20 requests of 6
//      frames, two outstanding, on the engines), Minor below it (10 POLLED
//      requests of 8 frames, one outstanding: under dma_min_frames_polled,
//      the polled loop) and, in a pressure pass, Pressure itself (8 requests
//      of 1 frame, one outstanding: under dma_min_frames, the pump), all at
//      /32 = 1.5 MHz inside ONE select window the letter holds - the peer's
//      exchange is one burst - of constant frames. THE RATE IS w's OWN (the
//      shared body's LinkTraits): at the command rate /256 a byte frame is
//      2048 cycles with two in flight, and a 3000-cycle preemption inside
//      Minor's polled transaction would lose one (p's limit), turning w into
//      a second p; at /32 a byte frame is 256 cycles, ONE in flight (under
//      the 437-cycle threshold), and a preemption of the polled loop is a gap
//      on the bus. Every reply to its requester in its order, every one
//      spi_ok, nothing rejected, the per-bus timeout never firing, the peer
//      counting every frame exact.
//   i  dark: every object idle, TIM2 stopped with its line quiet and its
//      DMA requests off, no time event armed, the SPI host idle with the
//      select high, its requests down and both its channels disabled, the
//      link port given back (gate closed, pads not outputs, both channels
//      disabled), the LED's pad not an output, the ceiling the Console's
//      own.
//   t  the UART, received: numbered, checked lines written by the Console
//      and turned around by the peer's ECHO at 460800 and 921600, assembled
//      by the LineReader from the receive engine's ring and lent to
//      LineSink: 200 lines a leg, none lost, none out of order, none
//      corrupt, the port's own counters (ring overruns, ORE, FE, PE, NE, DMA
//      faults, line overflows, torn lines) zero, and the peer's count of the
//      bytes it echoed exact. The echo is a polled loop with no FIFO
//      (usart_peer.cpp's run_receive: DATAR read at RXNE, written back at
//      TXE), so it holds two characters of a writer's lead. THE PREMISE,
//      judged first: a lead of L ppm builds L x N / 10^6 characters over N,
//      and a leg is at most 200 lines of 72 = 14400 characters, so the echo
//      holds any lead under 2 / 14400 = 138 ppm; the HSI wanders by about 100
//      ppm between runs a second apart (test_x035_usart letter j's three
//      runs), so the lead measured must be under 38 ppm - the HSI measured
//      some 3000 ppm SLOW of the far crystal, a lead of minus 0.3 %, the echo
//      never backing up.
//   u  the UART, sent: 300 numbered lines back to back through write_bulk()
//      and the transmit engine into the peer's SINK at 460800 and 921600,
//      judged by the peer's report - the bytes counted, their 16-bit sum,
//      FE, PE and ORE zero - and the wire's time printed against the
//      control's.
//   v  the loan under preemption: t's leg at 921600 with LineSink spinning
//      6000 cycles a line, so Pressure lands inside the borrower's dispatch,
//      and the lines STREAMED - line k written once line k - 20 is back - so
//      that the writer may run ahead of the echo (on this core it does not:
//      the receive side above the writer takes most of a line's wire time,
//      each line goes out as its own burst and the port is told at its idle
//      line; the print of USART4's vector entries shows it, and
//      docs/ch32x035/platform.md has the figures): every line exact, every
//      line lent alone (the sink's queue
//      empty at its entry, the sink never nested in itself), the two
//      buffers strictly alternating, and the lender never reading what the
//      borrower wrote over. The window keeps at most 20 lines of 72 = 1440
//      characters outstanding, inside the ring's 2048, whatever the core's
//      load. t's premise judged first. A pressure pass judges that Pressure
//      landed inside the borrower at least once: the sink's dispatches add
//      up to 200 x 6000 = 1.2 M cycles at least, 64 ticks of the longest
//      period (18672 cycles) on average, so a pass with none has a chance of
//      e^-64.
//
// build: boards = x035f8
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>
#include <variant>

#include "ch32x035/clock.hpp"
#include "ch32x035/delay.hpp"
#include "ch32x035/dma.hpp"
#include "ch32x035/pfic.hpp"
#include "ch32x035/pin.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/pwr.hpp"
#include "ch32x035/schedule.hpp"
#include "ch32x035/spi.hpp"
#include "ch32x035/ticker.hpp"
#include "ch32x035/tim.hpp"
#include "ch32x035/usart.hpp"
#include "kernel/borrowed.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/post.hpp"
#include "kernel/rubato.hpp"
#include "kernel/time_event.hpp"
#include "util/line_reader.hpp"
#include "util/print.hpp"
#include "util/spi_bus.hpp"
#include "util/testbench.hpp"

// THE PROTOCOL HEADERS ARE SHARED, NOT COPIED: the far board's two
// instruments compile the same files.
#include "../../../avrxt/src/apps/spi_link.hpp"
#include "../../../avrxt/src/apps/usart_link.hpp"

/// The one clock root, the HSI undivided: HCLK = 48 MHz, every bus's.
using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Ch32x035RubatoPlatform<>;
constexpr uint32_t core_hz = SysClock::hz;
constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000u;

/// The composed cycle count: monotone across the ticker's restart, a
/// difference exact under 2^32 cycles (89 s at 48 MHz).
struct Ruler {
    static uint32_t now() { return Ticker::cycles(); }
};

/// The STK counter's period: it counts HCLK and restarts at the ticker's
/// compare every millisecond.
constexpr uint32_t stk_period = SysClock::hz / Ticker::ticks_per_second;

/// A busy wait of `cycles` HCLK cycles: the STK counter read bare, one
/// load a turn, the restart folded in, for a span under half a period; the
/// composed count for a longer one. A preemption longer than a period only
/// lengthens a bare spin.
void spin(uint32_t cycles) {
    if (cycles < stk_period / 2u) {
        const uint32_t t0 = stk()->CNTL;
        for (;;) {
            const uint32_t t = stk()->CNTL;
            const uint32_t d = t >= t0 ? t - t0 : t + stk_period - t0;
            if (d >= cycles) {
                return;
            }
        }
    }
    const uint32_t t0 = Ticker::cycles();
    while (Ticker::cycles() - t0 < cycles) {
    }
}

// ---- the console -------------------------------------------------------------------

/// USART2 on PA2/PA3, the probe's serial, ENGINE-FREE (its channels, 6 and
/// 7, stay unused).
using Serial = Uart<2, P, 64, 128>;
constexpr Serial serial;
TwoKeyTestBench<Serial, 'y'> bench;

// ---- the SPI link ------------------------------------------------------------------

using SpiS = Spi<1>;
/// The instance's own channels, the element a half-word: 16-bit frames ride
/// the engines one DATAR access a frame.
using TxEngine = DmaTxEngine<SpiS::dma_tx_channel, uint16_t>;
using RxEngine = DmaRxEngine<SpiS::dma_rx_channel, uint16_t>;
using Host = SpiHost<1, spi1_default_pins, TxEngine, RxEngine>;
using CsPin = Pin<'A', 4>;
/// The command channel's SCK: /256 of HCLK = 187.5 kHz, 43 us a character
/// (test_x035_spi's link_clock).
constexpr SpiClock link_clock = SpiClock::div256;
constexpr const char* bus_clock_name = "HCLK";

// ---- the UART link -----------------------------------------------------------------

using U4 = Usart<4>;
using Tx4 = Pin<'B', 0>;
using Rx4 = Pin<'B', 1>;
using LinkRxEngine = DmaRxEngine<usart_dma_rx_channel(4)>;
using LinkTxEngine = DmaTxEngine<usart_dma_tx_channel(4)>;
/// A receive ring of 2048: letter v's half lap of 1024 (the file header).
using LinkPort = Uart<4, P, 2048, 256, UartFormat{}, LinkTxEngine, LinkRxEngine>;
using LinkRxChannel = DmaChannel<usart_dma_rx_channel(4)>;
using LinkTxChannel = DmaChannel<usart_dma_tx_channel(4)>;
/// The Uart task owns USART4's vector and its two channels only while a leg
/// runs.
volatile bool link_live = false;
/// The leg's vector entries, USART4's (the idle line, a burst's first frame,
/// the errors) and its two channels' (the ring's half and full marks, a
/// transmit block's end), counted from task_up(): where the port was told.
volatile uint32_t link_irq_entries = 0;
volatile uint32_t link_dma_entries = 0;

/// What a leg's transport counted, read after the leg (init() clears them).
struct PortCounters {
    uint16_t ring_overruns = 0, ore = 0, fe = 0, pe = 0, ne = 0, dma_faults = 0;
    bool clean() const {
        return ring_overruns == 0u && ore == 0u && fe == 0u && pe == 0u && ne == 0u && dma_faults == 0u;
    }
};

// ---- the pressure's clock ----------------------------------------------------------

using Pulse = Tim<2>;
static_assert(tim_clock_hz(SysClock{}) == SysClock::hz, "TIM2 counts HCLK at this clock");
constexpr uint32_t pulse_per_us = SysClock::hz / 1'000'000u;
static_assert(389u * pulse_per_us - 1u <= 0xFFFFu, "the longest period fits TIM2's 16 bits");
constexpr const char* pressure_name = "TIM2";
constexpr const char* peer_name = "the CH32V006K8U6";

// THE LINK'S REVIEWED HELPERS (the UART key's command channel), in their
// own namespace: the shared body below has a wait_ms() of the kernel's.
#include "x035_link_dut.hpp"

/// Letter w's rate and Pressure's length on this host (the file header):
/// /32, one frame in flight, and a request of one frame, under
/// dma_min_frames, so that it takes the pump.
struct LinkTraits {
    static constexpr SpiClock w_rate = SpiClock::div32;
    static constexpr uint8_t w_press_len = 1;
};

// THE TARGET-FREE HALF IS SHARED, NOT COPIED: every family's link suite
// compiles the same body around its own engines.
#include "../../../avrxt/src/apps/rubato_link.hpp"

// =============================================================================
// What the shared body asks of this family
// =============================================================================

void pressure_arm(uint16_t period_us) {
    Pulse::enable(false);
    Pulse::set_count(0);
    (void)Pulse::set_period(static_cast<uint16_t>(static_cast<uint32_t>(period_us) * pulse_per_us - 1u));
    Pulse::clear_flags(Pulse::update_flag);
    Pulse::interrupts(Pulse::update_interrupt, true);
}
void pressure_go() { Pulse::enable(true); }
void pressure_halt() { Pulse::enable(false); }
void pressure_quiet() {
    Pulse::interrupts(Pulse::update_interrupt, false);
    Pulse::clear_flags(Pulse::update_flag);
    Pfic::clear_pending(Pulse::update_irq());
    spin(20'000u);   // the last tick's dispatch, if it was still running
}

uint32_t rx_stream_position() { return LinkRxChannel::count(); }

/// BR's division of HCLK: two to the code plus one (16.3.1).
uint32_t division(SpiClock c) { return 2u << static_cast<uint8_t>(c); }

/// The host reports a lost frame itself, as the transaction's status
/// (letter p); nothing to latch at a reply.
void client_replied() {}

PortCounters read_port_counters() {
    PortCounters c{};
    c.ring_overruns = LinkPort::rx_overruns();
    c.ore = LinkPort::hw_overruns();
    c.fe = LinkPort::frame_errors();
    c.pe = LinkPort::parity_errors();
    c.ne = LinkPort::noise_errors();
    c.dma_faults = LinkPort::dma_faults();
    return c;
}

void print_port(const PortCounters& c) {
    print(serial, "ring overruns ", c.ring_overruns, ", ORE ", c.ore, ", FE ", c.fe, ", PE ", c.pe, ", NE ", c.ne,
          ", DMA faults ", c.dma_faults, "; USART4's vector ", link_irq_entries, " entries, its channels' ",
          link_dma_entries);
}

// =============================================================================
// The UART link: usart_link.hpp on USART4 polled, between legs
// =============================================================================

namespace ul {

/// Find the peer and see that it is the CH32V006's usart_peer on its
/// crystal: a named failure, never a hang.
bool ensure() {
    if (!x035link::ensure_link()) {
        bench.verdict("the peer answers a ping (the peer board running usart_peer)", false);
        return false;
    }
    return true;
}

/// USART4 let go: reset, gated off, both pads floating.
void release() { x035link::u4_release(); }

bool peer_report(link::Report& r, uint16_t after_ms) { return x035link::peer_report(r, after_ms); }

/// Command an action from command mode, whatever the last leg left (a
/// released port is gated off); peer_act() leaves settle_ms after the
/// ack, the peer opening its window settle_ms + 2 ms after it.
bool peer_act(link::Op op, const link::Params& a) {
    (void)x035link::link_command_mode();
    return x035link::peer_act(op, a);
}

}  // namespace ul

// =============================================================================
// The Uart task
// =============================================================================

/// The Uart task up at `rate` on USART4, its vectors live: the TX pad
/// parked high (a USART in reset drives its alternate output low), the
/// block's reset pulsed - whatever the command path left in it goes - then
/// the transport's init() and the RX pad's pull-up.
bool task_up(uint32_t rate) {
    Tx4::output(true);
    U4::bus_clock(true);
    U4::reset();
    link_irq_entries = 0;
    link_dma_entries = 0;
    link_live = true;
    if (!LinkPort::init(clock, rate)) {
        link_live = false;
        return false;
    }
    (void)Rx4::input(PinPull::up);
    return true;
}

/// The transport's release(): the vector off, both engines stopped, the
/// block's reset pulsed BEFORE its gate closes (usart.hpp, the release
/// contract), the pads released.
void task_down() {
    LinkPort::release();
    link_live = false;
}

// =============================================================================
// n - the peer's command channel
// =============================================================================

/// The far chip's spi_peer: the sanity byte, the CH32V006's kind, its label.
bool far_spi_peer(const spilink::Ident& d) {
    return d.sanity == spilink::ident_sanity && spilink::peer_kind(d.version) == spilink::peer_ch32v00x &&
           d.label[0] == 'V' && d.label[1] == '0' && d.label[2] == '0' && d.label[3] == '6';
}

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
        bench.verdict(pass_label(s), "ident is spi_peer's (the sanity byte), the CH32V006's (0x06xx)",
                      got && far_spi_peer(d));
        bench.verdict(pass_label(s), "ten more frames answered, every one through the arbiter", pings == 10u);
        judge_pressure(s, st);
    }
}

// =============================================================================
// o - the matrix
// =============================================================================

/// The matrix's rate: /32 of HCLK = 1.5 MHz on the engines (32 frames,
/// past dma_min_frames, the buffers aligned).
constexpr SpiClock matrix_rate = SpiClock::div32;
static_assert(Host::dma_min_frames <= 32u, "the matrix's windows ride the engines");

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
        // Modes 0 and 3 only: the far client's (the file header).
        for (const uint8_t m : {uint8_t{0}, uint8_t{3}}) {
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
        bench.verdict(pass_label(s), "modes 0 and 3 x two widths, LSb first in both, a DORD mismatch: every "
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
constexpr uint8_t rung_count = sizeof rungs / sizeof rungs[0];

enum class Path : uint8_t { polled16, polled8, pump16, engines };
const char* path_name(Path p) {
    switch (p) {
        case Path::polled16: return "polled loop, 16-bit";
        case Path::polled8: return "polled loop, 8-bit";
        case Path::pump16: return "RXNE pump, 16-bit";
        default: return "DMA engines";
    }
}
bool path_polled(Path p) { return p == Path::polled16 || p == Path::polled8; }
bool path_wide(Path p) { return p == Path::polled16 || p == Path::pump16; }
uint16_t path_frames(Path p) {
    return p == Path::pump16 ? 16u : p == Path::engines ? 32u : 8u;
}

static_assert(8u < Host::dma_min_frames_polled && Host::dma_min_frames <= 32u,
              "letter p's polled windows of eight frames take the polled loop, its 32-frame ones the engines");

/// A frame's core cycles at code `c`: its bits at SCK = HCLK over two to the
/// code plus one (spi.hpp's spi_frame_cycles).
uint32_t frame_cycles(SpiClock c, bool wide) {
    return spi_frame_cycles(c, wide ? SpiDataSize::bits16 : SpiDataSize::bits8);
}

/// Does the host keep two frames in flight on a receive at this code and
/// width (spi.hpp's threshold, write_ahead_from())? A slower rate is a
/// larger code.
bool two_in_flight(SpiClock c, bool wide) {
    const SpiClock from = Host::write_ahead_from(wide ? SpiDataSize::bits16 : SpiDataSize::bits8);
    return static_cast<uint8_t>(c) >= static_cast<uint8_t>(from);
}

/// One pass's polled outcome: the rungs cut short by a detected failure (a
/// bit a rung, bit k for rungs[k]), per polled path, and the cycles the
/// longest of those windows took; and whether any polled failure was
/// something else - another status, a wrong run handed back spi_ok, or a
/// failure where one frame is in flight.
uint8_t cut_short[2] = {};
uint32_t cut_cycles_max = 0;
bool polled_failed_otherwise = false;

/// One ladder on one path: every rung byte-exact, each failure's outcome
/// printed.
bool climb(Path path, uint8_t seed) {
    bool all = true;
    for (uint8_t k = 0; k < rung_count; ++k) {
        const SpiClock c = rungs[k];
        sl::Exchange e{};
        e.rate = c;
        e.polled = path_polled(path);
        e.wide = path_wide(path);
        e.odd = path_wide(path);
        e.count = path_frames(path);
        e.seed_a = static_cast<uint8_t>(seed + 9u * k);
        e.seed_b = static_cast<uint8_t>(seed + 0x40u + 5u * k);
        const sl::Outcome o = sl::run(e);
        const bool ok = sl::exact(e, o);
        if (!ok) {
            const bool two = two_in_flight(c, e.wide);
            print(serial, "    ", path_name(path), two ? " (two frames in flight" : " (one frame in flight",
                  ", a frame ", frame_cycles(c, e.wide), " cycles; the window took ", o.cycles, "):", crlf);
            sl::print_outcome(e, o);
            if (path_polled(path)) {
                if (o.status == spi_stalled && two) {
                    cut_short[path == Path::polled16 ? 0u : 1u] |= static_cast<uint8_t>(1u << k);
                    if (o.cycles > cut_cycles_max) {
                        cut_cycles_max = o.cycles;
                    }
                } else {
                    polled_failed_otherwise = true;
                }
            }
        }
        all = all && ok;
    }
    return all;
}

/// The rungs a bit set names, as "/128 /64".
void print_rungs(uint8_t bits) {
    if (bits == 0u) {
        print(serial, " none");
        return;
    }
    for (uint8_t k = 0; k < rung_count; ++k) {
        if ((bits & (1u << k)) != 0u) {
            print(serial, " /", division(rungs[k]));
        }
    }
}

void tp_ladders() {
    print(serial, "  two frames in flight on a receive from ", bus_clock_name, "/",
          division(Host::write_ahead_from(SpiDataSize::bits8)), " (8-bit) and /",
          division(Host::write_ahead_from(SpiDataSize::bits16)),
          " (16-bit); a frame at /256 is 2048 cycles (8-bit) and 4096 (16-bit), at /64 512 and 1024; "
          "the engines from ", Host::dma_min_frames, " frames, a polled phase from ",
          Host::dma_min_frames_polled, crlf);
    for (uint8_t p = 0; p < pass_count; ++p) {
        const PassSpec& s = passes[p];
        pass_begin(s);
        if (!sl::need_peer()) {
            (void)pass_end(s);
            return;
        }
        cut_short[0] = cut_short[1] = 0;
        cut_cycles_max = 0;
        polled_failed_otherwise = false;
        const bool polled16 = climb(Path::polled16, 0x21);
        const bool polled8 = climb(Path::polled8, 0x2D);
        const bool pump16 = climb(Path::pump16, 0x35);
        const bool engines = climb(Path::engines, 0x43);
        const bool ping = sl::command(spilink::Op::ping);
        const PassStats st = pass_end(s);
        print_pass(s, st);
        print(serial, "    polled rungs cut short (spi_stalled): 16-bit");
        print_rungs(cut_short[0]);
        print(serial, "; 8-bit");
        print_rungs(cut_short[1]);
        print(serial, "; the longest cut window ", cut_cycles_max, " cycles", crlf);
        if (s.period_us == 0u) {
            bench.verdict(pass_label(s), "THE POLLED LOOP through the arbiter: 8 16-bit and 8 8-bit frames "
                                         "byte-exact both ways at every code to /2",
                          polled16 && polled8);
        } else {
            // Under pressure the polled loop runs inside the arbiter's
            // dispatch, which every dispatch above the arbiter holds off for
            // its whole length - a length the host's hold-off does not
            // count. Where two frames are in flight and the preemption
            // outlasts a frame, the one-deep receive buffer loses one, and
            // this loop, which watches RXNE alone, waits for it until its
            // budget runs out: the transaction ends spi_stalled. That it is
            // detected every time and never handed back spi_ok with a hole
            // is what is judged, the rungs cut short printed beside it. One
            // frame in flight cannot overrun: a preemption there is a gap on
            // the bus.
            bench.verdict(pass_label(s), "THE POLLED LOOP through the arbiter: byte-exact, or cut short only by "
                                         "a detected failure (spi_stalled) where two frames are in flight and a "
                                         "dispatch above the arbiter outlasts a frame - never a run with a hole",
                          (polled16 && polled8) || !polled_failed_otherwise);
        }
        bench.verdict(pass_label(s), "THE RXNE PUMP: 16 16-bit frames byte-exact at every code to /2", pump16);
        bench.verdict(pass_label(s), "THE DMA ENGINES: 32 frames byte-exact at every code to /2", engines);
        bench.verdict(pass_label(s), "the command channel survives the ladders", ping);
        judge_pressure(s, st);
    }
}

// =============================================================================
// w - the arbiter under pressure
// =============================================================================

static_assert(w::client_len >= Host::dma_min_frames && w::press_len < Host::dma_min_frames &&
                  w::minor_len < Host::dma_min_frames_polled,
              "letter w: the Client's requests ride the engines, Pressure's the pump, Minor's polled ones the "
              "polled loop (the file header)");
static_assert(spi_write_ahead_safe(link_clock, SpiDataSize::bits8, spi_default_hold_off_cycles) &&
                  !spi_write_ahead_safe(w::rate, SpiDataSize::bits8, spi_default_hold_off_cycles),
              "letter w: at the command rate a byte frame keeps two in flight, at w's rate one (the file header)");

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
            Host::prime(SpiMode::mode0, w::rate);
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

/// The boot's results, printed by the banner.
bool clock_ok = false;
bool tick_ok = false;
bool host_ok = false;
bool supply_ok = false;

// =============================================================================
// i - dark
// =============================================================================

/// A pad's MODE bits (its nibble's low two): 00 is an input.
template <typename Pad>
bool pad_is_output() {
    return (Pad::nibble() & 0x3u) != 0u;
}

using SpiTxChannel = DmaChannel<SpiS::dma_tx_channel>;
using SpiRxChannel = DmaChannel<SpiS::dma_rx_channel>;

/// DMAINTENR's DMA request enables, bits 8 to 14: UDE, CC1DE..CC4DE, COMDE,
/// TDE (12.4.4).
constexpr uint16_t tim_dma_enables = 0x7F00u;

void ti_dark() {
    bench.verdict("every object above the Console idle: every queue empty",
                  Pressure::queue.empty() && LineSink::queue.empty() && Client::queue.empty() &&
                      SpiArb::queue.empty() && Minor::queue.empty() && LinePort::queue.empty());
    bench.verdict("TIM2 stopped, its update interrupt off, its line quiet and none of its DMA requests enabled",
                  !Pulse::enabled() && (Pulse::interrupts() & Pulse::update_interrupt) == 0u &&
                      (Pulse::interrupts() & tim_dma_enables) == 0u && !Pfic::pending(Pulse::update_irq()));
    bench.verdict("no time event is armed (the arbiter's timeout among them)", !TE::next_deadline().has_value());
    bench.verdict("the SPI host idle with the select high, its DMA requests down and both its channels disabled",
                  !SpiS::busy() && (SpiS::regs().CTLR2 & (spi_txdmaen | spi_rxdmaen)) == 0u &&
                      !SpiTxChannel::enabled() && !SpiRxChannel::enabled() && pad_is_output<CsPin>() &&
                      CsPin::read());
    bench.verdict("the link port given back: its gate closed, its pads not outputs, both its channels disabled",
                  !Rcc::enabled(U4::bus, usart_gate_for(4)) && !pad_is_output<Tx4>() && !pad_is_output<Rx4>() &&
                      !LinkRxChannel::enabled() && !LinkTxChannel::enabled() && !link_live);
    bench.verdict("the LED's pad PA0 is not an output", !pad_is_output<Pin<'A', 0>>());
    bench.verdict("HCLK from the 48 MHz HSI, the clock task's answer", clock_ok);
    bench.verdict("the ceiling is the Console's and interrupts are enabled",
                  P::ceiling == console_index && P::interrupts_enabled());
}

// =============================================================================
// t, u, v - the UART
// =============================================================================

constexpr uint32_t uart_rates[] = {460'800u, 921'600u};
constexpr uint16_t echo_lines = 200;
constexpr uint16_t sink_lines = 300;

/// The echo's slack over a leg (the file header): two characters over 200
/// lines of 72, less the HSI's wander between runs a second apart.
constexpr int32_t lead_bound_ppm = static_cast<int32_t>(2u * 1'000'000u / (echo_lines * 72u)) - 100;
static_assert(lead_bound_ppm == 38, "the file header's arithmetic");

/// THE PREMISE of the echo legs: this end's rate against the far crystal,
/// timed by the peer (x035_link_dut.hpp's calibrate leg at 1 Mbaud, a
/// divisor of 48 exact), and the far peer's identity; false (judged) when
/// either is missing.
bool echo_premise() {
    link::Ident d{};
    const bool got = x035link::peer_ident(d);
    bench.verdict("the peer is the CH32V006's usart_peer on its crystal",
                  got && d.sanity == link::ident_sanity && link::peer_kind(d.version) == link::peer_ch32v00x &&
                      d.xtal == 1u);
    link::Cfg c{};
    c.rate = x035link::calib_rate;
    const x035link::Calib k = x035link::calibrate(c, x035link::calib_frames);
    print(serial, "  this end's HSI against the far crystal: ", static_cast<int32_t>(k.ppm),
          " ppm (the lead the echo must hold; under ", lead_bound_ppm, " judged)", crlf);
    const bool ok = got && k.ok && static_cast<int32_t>(k.ppm) < lead_bound_ppm;
    bench.verdict("the writer does not outrun the far echo: its lead under the echo's slack", ok);
    (void)x035link::link_command_mode();
    return ok;
}

void tt_lines() {
    if (!ul::ensure() || !echo_premise()) {
        ul::release();
        return;
    }
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
    // 921600 and a window of 20 lines: the file header's arithmetic.
    constexpr uint32_t rate = 921'600u;
    constexpr uint8_t window = 20;
    static_assert(window * 72u < 2048u, "the window's characters fit the receive ring");
    if (!ul::ensure() || !echo_premise()) {
        ul::release();
        return;
    }
    for (uint8_t p = 0; p < pass_count; ++p) {
        const PassSpec& s = passes[p];
        pass_begin(s);
        if (!ul::ensure()) {
            (void)pass_end(s);
            ul::release();
            return;
        }
        LineSink::spin_cycles = 6000;
        const LegResult g = echo_leg(rate, echo_lines, static_cast<uint16_t>(3000u + 13u * p), window);
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

void console_banner() {
    print(serial, crlf, "boot: clk=", clock_ok ? "HSI48" : "FAILED", " tick=", tick_ok ? "STK" : "FAILED",
          " ruler=STK spi1=", host_ok ? "host" : "FAILED", " supply=", supply_ok ? "above 3.0 V" : "LOW",
          " scheduler=", scheduler_in_ram ? "SRAM" : "flash", crlf);
    if (!supply_ok) {
        print(serial, "SUPPLY UNDER 3.0 V (the PVD at PLS 10): no letter registered, the link pads left floating",
              crlf);
    }
    print(serial, "test_x035_rubato_link - the SPI and UART stacks under Rubato on the ", device::part_name,
          " against ", peer_name, " (z: spi_peer, y: usart_peer)", crlf);
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------

BRIO_CH32_VECTOR(usart2_handler) {
    if (Serial::isr()) {
        brio::post<Console>(ConsoleEvent{console_rx});
    }
}

BRIO_CH32_VECTOR(systick_handler) { brio::Ticker::tick(); }

BRIO_CH32_SCHEDULER_VECTOR(software_handler) { brio::scheduler_trap<K>(); }
BRIO_CH32_SCHEDULER_VECTOR(ecall_m_handler) { brio::scheduler_return<K>(); }

/// The pressure: TIM2's update, where it landed, and the post.
BRIO_CH32_VECTOR(tim2_up_handler) {
    Pulse::clear_flags(Pulse::update_flag);
    pressure_landed();
}

// The SPI host: the instance's vector (the pump) and its two channels'
// (the engines' completion on the receive channel, an error on either).
BRIO_CH32_VECTOR(spi1_handler) {
    if (Host::isr()) {
        done_from_dma = false;
        brio::post<SpiArb>(brio::TransferDone{Host::status()});
    }
}
[[gnu::always_inline]] inline void spi_dma() {
    if (Host::dma_isr()) {
        done_from_dma = true;
        brio::post<SpiArb>(brio::TransferDone{Host::status()});
    }
}
BRIO_CH32_VECTOR(dma1_channel2_handler) { spi_dma(); }
BRIO_CH32_VECTOR(dma1_channel3_handler) { spi_dma(); }

// The link port: the instance's vector (the idle line, a burst's first
// frame, the errors) and its two channels' (the ring's marks, a block's
// completion).
BRIO_CH32_VECTOR(usart4_handler) {
    link_irq_entries = link_irq_entries + 1u;
    if (link_live && LinkPort::isr()) {
        brio::post<LinePort>(brio::RxActivity{});
    }
}
[[gnu::always_inline]] inline void link_dma() {
    link_dma_entries = link_dma_entries + 1u;
    if (link_live && LinkPort::dma_isr()) {
        brio::post<LinePort>(brio::RxActivity{});
    }
}
BRIO_CH32_VECTOR(dma1_channel1_handler) { link_dma(); }
BRIO_CH32_VECTOR(dma1_channel8_handler) { link_dma(); }

int main() {
    clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    tick_ok = brio::Ticker::init(clock);

    // THE SUPPLY before any link pad is driven (the desk's rule).
    brio::Pwr::pvd(brio::PvdLevel::v3_0);
    (void)brio::delay_us(clock, 200);
    supply_ok = !brio::Pwr::supply_low();
    if (supply_ok) {
        CsPin::output(true);   // the far client deselected from boot
        host_ok = Host::init(clock);
        Pulse::init();
        (void)Pulse::configure(brio::TimConfig{.prescaler = 0, .period = 0xFFFFu});
        brio::Pfic::enable(Pulse::update_irq());

        bench.letter('n', "the peer's command channel through the arbiter", tn_peer_link);
        bench.letter('o', "the matrix on the DMA engines: modes 0 and 3, widths, bit orders", to_matrix);
        bench.letter('p', "the ladders: the polled loop both widths, the pump, the engines, every code", tp_ladders);
        bench.letter('w', "the arbiter under pressure: three requesters, one bus", tw_arbiter);
        bench.letter('i', "dark: every object idle, nothing lit", ti_dark);
        bench.second_letter('t', "the UART received: numbered lines through the peer's echo", tt_lines);
        bench.second_letter('u', "the UART sent: a blast of numbered lines into the peer's sink", tu_blast);
        bench.second_letter('v', "the loan under preemption: a slow borrower at 921600", tv_loan);
    }
    brio::scheduler_init();

    (void)serial_ok;
    brio::enable_interrupts();
    K::run();
}
