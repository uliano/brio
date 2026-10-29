// test_vx03_rubato_link - the bench suite of the SPI and UART stacks of the
// CH32V203 and CH32V303 stratum under Rubato, the preemptive kernel
// (kernel/rubato.hpp): util/spi_bus.hpp's arbiter over ch32vx03/spi.hpp's
// SpiHost and its DMA engines, and util/line_reader.hpp's LineReader over
// ch32vx03/usart.hpp's Uart and its receive engine, run against the desk
// partner and its unchanged instruments (ch32vx03/src/apps/spi_peer.cpp,
// usart_peer.cpp) while an object of the highest priority preempts them at
// moments the traffic does not choose. ONE SOURCE, TWO IMAGES: the
// CH32V203C8T6 against the CH32V303VCT6 and the other way round, each
// board the other's peer, as test_vx03_spi's and test_vx03_serial's pair
// letters are. Every verdict is the traffic judged byte-exact, as those
// Tenuto suites judge it; nothing here changes a driver or a service.
//
// A test_<target>_<subject> suite in the bench grammar (util/testbench.hpp)
// that runs inside the kernel it tests, as test_vx03_rubato does: the
// program is one Rubato pack of seven objects - Pressure, LineSink, Client,
// SpiArb, Minor, LinePort and Console, from the highest priority down - and
// everything of it that names no register, the pack, the passes, the
// numbered lines, the window client and the UART legs, is the link suites'
// shared body (avrxt/src/apps/rubato_link.hpp, whose header describes each
// object and the passes). What is this family's is here, each choice from
// the reference manual's chapters (RM 20 the SPI, RM 18 the USART, RM 11
// the DMA) and the driver headers:
//
//   THE SPI HOST. The pair's instance (test_vx03_spi's): SPI2 on its one
//   column on the CH32V203C8 (SCK PB13, MISO PB14, MOSI PB15, the select
//   PB12), SPI3 on its SECOND column on the CH32V303VC (SCK PC10, MISO
//   PC11, MOSI PC12, the select PA4), with the DMA engines on the
//   instance's own two slots (table 11-5: SPI2 on DMA1's 4 and 5, SPI3 on
//   DMA2's 1 and 2), the receive channel at very_high and the transmit at
//   high, half-word beats for 16-bit frames. The select is a GPIO the
//   Request carries (the host's boot configuration is SpiNss::software);
//   the peer's is its hardware NSS input. SCK and MOSI at the medium slew
//   class, the pair's pad class (test_vx03_spi's pair_pad_class, its
//   reason beside it). NO FIFO (20.2.2): one buffer each way, so the
//   receive shapes write ahead - two frames in flight - only where a frame
//   outlasts the image's hold-off and the pump's path to the read, 200 + 25
//   cycles (spi.hpp, THE WRITE-AHEAD THRESHOLD): on SPI2 and SPI3, whose
//   PB1 runs at half of HCLK, a byte frame from /16 (256 cycles) and a
//   16-bit one from /8 (256). A data phase of dma_min_frames (3) or more
//   rides the engines even when it is polled, and a 16-bit phase at an odd
//   address cannot (11.3.6: the controller would round it down), so the
//   POLLED LOOP is a 16-bit window from an odd address or a byte window of
//   two frames, and the PUMP a 16-bit window from an odd address. The
//   polled loop runs inside the arbiter's dispatch: a dispatch above the
//   arbiter holds it off for its whole length, which the hold-off does not
//   count, and where two frames are in flight and the hold-off outlasts a
//   frame the one-deep receive buffer loses one - OVR (20.2.7), which the
//   loop watches beside RXNE and ends the transaction on with spi_overrun,
//   never handing back a run with a hole in it.
//   THE UART. The pair's link port (vx03_link_dut.hpp): USART3 on PB10/PB11
//   on the CH32V203C8, UART4 on its second column, PB0/PB1, on the
//   CH32V303VC, both engines on the port's slots (RM 11.2.3: USART3 on
//   DMA1's 2 and 3, UART4 on DMA2's 5 and 3) - the receive engine a
//   circular ring of 2048 over the whole storage, never re-armed, the port
//   told at the idle line, at RXNE for a one-frame burst and at the half
//   and full marks of a lap; the transmit engine a block a contiguous run
//   of the ring, write_bulk() pumping it once (docs/ch32vx03/usart.md, the
//   transport). No receiver time-out on this block. Both ends on their 8
//   MHz crystals at 144 MHz, PCLK1 72 MHz: every rate is one divisor at
//   both ends, the two crystals within nine ppm (usart.md).
//   THE DMA RELEASE CONTRACT (docs/ch32vx03/dma.md, "A released
//   requester"): a USART's DMA request, once raised, is held until the
//   channel answers it or the block's reset line is pulsed - so every hand
//   between the polled command channel and the Uart task goes through a
//   reset: Uart::release() pulses it before the gate, the command path's
//   open() pulses it before it programs the port, and task_up() pulses it
//   before the Uart task's init(), the TX pad parked high across it. The
//   SPI host's requests do not outlive SPE (the same section), and its
//   engines drop both requests at every transaction's end. Letter i reads
//   every channel the image uses disabled and every request enable clear.
//   THE PRESSURE. TIM2's update interrupt, 16 bits on TIMCLK = HCLK = 144
//   MHz at prescaler 0: 97 / 193 / 389 us are 13968 / 27792 / 56016 counts.
//   Its dispatch, a 3000-cycle spin, road 1 (119 cycles, docs/ch32vx03/
//   platform.md) and the way back, is shorter than the shortest period's
//   13968 cycles, so no tick finds the last one still queued; at 97 us x
//   3000 it takes 21 % of the core. TIM2's update request is a DMA1
//   channel 2 row on the CH32V203 - the link's transmit channel there - so
//   UDE and every CCxDE stay clear, and letter i reads them clear.
//   THE RULER. The kernel ticker's composed cycles() (the tick count and
//   the STK counter's position, ch32vx03/ticker.hpp), which the shared
//   body's passes and spans need monotone across a tick; spin() reads the
//   STK counter bare, one load, the restart at the compare folded in.
//   THE V4F'S FLOAT TAX (the CH32V303). Under ilp32f every vector that
//   calls out is a naked trampoline (qingke/pfic.hpp) and the stratum's
//   vector guard walks each body for an f-register instruction; no object
//   of the pack touches a float, so no handler saves one, and letter i
//   reads mstatus.FS on that part.
//   THE UART KEY'S COMMAND CHANNEL. The link's own instance polled at
//   115200 between legs through vx03_link_dut.hpp's reviewed helpers: a
//   character every 87 us (12500 cycles), against the longest preemption
//   there, a 3000-cycle spin and the scheduler.
//
// THE WIRES, fixed on the desk (docs/boards/weact-ch32v203c8.md,
// docs/boards/ch32v303-evt.md; test_vx03_spi's and test_vx03_serial's
// pair):
//   SPI    V203 PB12 - V303 PA4 (the select), PB13 - PC10 (SCK), PB14 -
//          PC11 (MISO), PB15 - PC12 (MOSI): the pair's flat cable, 33 Ohm
//          on SCK at each end, 56 Ohm at the V203's PB14 and at the V303's
//          PC12, two grounds of its own
//   UART   V203 PB10 (USART3 TX) -> V303 PB1 (UART4 RX), V203 PB11 <- V303
//          PB0
// The SPI letters (key z) want the peer running `spi_peer`, the UART ones
// (key y) `usart_peer`. The CH32V303's PA4 is not 5 V tolerant: that board
// stays powered whenever the CH32V203's runs. NOTHING LIT: no letter drives
// the LED's pad PB2.
//
// The letters (z: n o p w i; y: t u v), each of z and y run as seven passes
// (the shared body's control and six pressures):
//   n  the peer's command channel through the arbiter: a ping, ident (the
//      sanity byte, the stratum's kind 0x05 and the far part's label), and
//      ten more pings, every frame on the engines at /256 of PCLK1 = 281
//      kHz and its answer decoded by the Client.
//   o  the matrix on the DMA engines (32 frames a window at /32 = 2.25
//      MHz, against the peer's DMA serve): the four modes in 8- and 16-bit
//      frames, LSb first in both widths, and a bit-order mismatch as an
//      exact two-way reversal - byte-exact at both ends; the data windows'
//      bytes over their cycles printed against the control's.
//   p  the ladders through the arbiter, every BR code from /256: the
//      POLLED LOOP in 16-bit frames (8, from an odd address) and in byte
//      frames (2, under the engines' crossover), the RXNE PUMP in 16-bit
//      frames (16, from an odd address) and the DMA ENGINES (32 byte
//      frames). Judged to /8 = 9 MHz: from /4 the host's read-back is
//      outside the two datasheets' timing (the client's MISO valid 25 ns
//      after its edge and the host's 5 ns of setup, 30 ns against /4's
//      27.8 ns half period - test_vx03_spi's pair section), so /4 and /2
//      are printed with their counts. In the control every judged rung is
//      byte-exact on every path. Under pressure the pump and the engines
//      are, and the polled loop is byte-exact or cut short by a detected
//      overrun (spi_overrun) at a rung where two frames are in flight - never
//      a run with a hole, never an overrun where one frame is.
//   w  the arbiter under pressure: the Client above it (20 requests of 6
//      frames on the engines, two outstanding), Minor below it (10 POLLED
//      requests of 2 frames, one outstanding, on the polled loop) and, in a
//      pressure pass, Pressure itself (8 requests of 2 frames, one
//      outstanding, on the pump), all at /256 inside ONE select window the
//      letter holds - the peer's exchange is one burst - of constant
//      frames. A data phase of dma_min_frames (3) or more rides the engines
//      here, polled or not, so Minor's and Pressure's requests are two
//      frames (the shared body's LinkTraits): the three paths the design
//      gives the three requesters. Minor's two frames are both in flight
//      at /256, each 4096 cycles, longer than Pressure's longest hold of
//      the polled loop (3000 cycles, road 1's 119 and the way back): a
//      preemption there is a gap on the bus (letter p never cut a
//      4096-cycle frame). Every reply to its requester in its order, every
//      one spi_ok, nothing rejected, the per-bus timeout never firing, the
//      peer counting every frame exact.
//   i  dark: every object idle, TIM2 stopped with its line quiet and its
//      DMA requests off, no time event armed, the SPI host idle with the
//      select high, its requests down and both its channels disabled, the
//      link port given back (gate closed, pads not outputs, both channels
//      disabled), the LED's pad not an output, the ceiling the Console's
//      own; on the CH32V303, mstatus.FS not Dirty.
//   t  the UART, received: numbered, checked lines written by the Console
//      and turned around by the peer's ECHO at 460800 and 921600, assembled
//      by the LineReader from the receive engine's ring and lent to
//      LineSink: 200 lines a leg, none lost, none out of order, none
//      corrupt, the port's own counters (ring overruns, ORE, FE, PE, NE, DMA
//      faults, line overflows, torn lines) zero, and the peer's count of the
//      bytes it echoed exact. The echo is a polled loop with no FIFO
//      (usart_peer.cpp's run_receive: DATAR read at RXNE and written back
//      at TXE), so it holds two characters of a writer's lead; at both
//      rates the two ends divide one PCLK1 by one BRR (156 and 78, +1602
//      ppm each), and the crystals' nine ppm make 0.0006 of a character a
//      71-character line.
//   u  the UART, sent: 300 numbered lines back to back through write_bulk()
//      and the transmit engine into the peer's SINK at 460800 and 921600,
//      judged by the peer's report - the bytes counted, their 16-bit sum,
//      FE, PE and ORE zero - and the wire's time printed against the
//      control's.
//   v  the loan under preemption: t's leg at 921600 with LineSink spinning
//      6000 cycles a line, so Pressure lands inside the borrower's dispatch,
//      and the lines STREAMED - line k written once line k - 20 is back - so
//      that the port is told at the ring's half lap (1024 characters) with
//      the characters still coming and lends its lines while the next ones
//      arrive: every line exact, every line lent alone (the sink's queue
//      empty at its entry, the sink never nested in itself), the two
//      buffers strictly alternating, and the lender never reading what the
//      borrower wrote over. The echo's backlog over the whole leg, 200
//      lines of at most 71 characters at the crystals' nine ppm, is 0.13 of
//      a character, inside its two. A pressure pass judges that Pressure
//      landed inside the borrower at least once: the sink's dispatches add
//      up to 200 x 6000 = 1.2 M cycles at least, 21 ticks of the longest
//      period (56016 cycles) on average, so a pass with none has a chance
//      of e^-21.
//
// build: boards = v203c8,v303vc
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>
#include <variant>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/dma.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/schedule.hpp"
#include "ch32vx03/spi.hpp"
#include "ch32vx03/ticker.hpp"
#include "ch32vx03/tim.hpp"
#include "ch32vx03/usart.hpp"
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
#include "vx03_link_dut.hpp"

/// Both boards on their 8 MHz crystals x18: HCLK 144 MHz, PCLK1 72 MHz -
/// the SPI's PB1 and the link port's bus. THE CRYSTAL, NOT THE HSI: the
/// UART letters' arithmetic is one divisor at both ends of two crystals
/// (the file header), and the HSI's PLL runs half a per cent fast of the
/// crystal's (docs/ch32vx03/platform.md, the tick rate against the host's
/// clock) - against the far echo's two characters of slack, a continuous
/// stream of letter v's length would bury that at 45 characters.
using SysClock = brio::Clock<brio::ClockSource::pll, 144'000'000, 8'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

static_assert(vx03link::present, "the link suite runs on the desk pair's two parts, the CH32V203C8 and the "
                                 "CH32V303VC (the boards line)");
constexpr bool v303 = vx03link::v303;

using P = Ch32vx03RubatoPlatform<>;
constexpr uint32_t core_hz = SysClock::hz;
constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000u;
static_assert(SysClock::pclk1_hz == 72'000'000u, "the pair's PB1 at 72 MHz");

/// The composed cycle count: monotone across the ticker's restart, a
/// difference exact under 2^32 cycles (29.8 s at 144 MHz).
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
inline void spin(uint32_t cycles) {
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

/// USART1 on PA9/PA10, the probe's serial, ENGINE-FREE: its two DMA rows
/// are DMA1's channels 4 and 5 (RM 11.2.3), SPI2's on the CH32V203.
using Serial = Uart<1, P, 64, 128>;
constexpr Serial serial;
TwoKeyTestBench<Serial, 'y'> bench;

// ---- the SPI link ------------------------------------------------------------------

/// SPI3 on its second column where the part has SPI3, SPI2's one column
/// otherwise (the file header).
constexpr uint8_t pair_n = spi_present(3) ? 3u : 2u;
constexpr uint8_t pair_code = pair_n == 3u ? 1u : 0u;
constexpr SpiPins link_pins = spi_pins_for(pair_n, pair_code);
using PairS = Spi<pair_n>;
/// The instance's own slots, the element a half-word: 16-bit frames ride
/// the engines one DATAR access a frame (spi.hpp, THE BEAT IS THE FRAME).
using TxEngine = DmaTxEngine<PairS::dma_tx_slot.controller, PairS::dma_tx_slot.channel, uint16_t>;
using RxEngine = DmaRxEngine<PairS::dma_rx_slot.controller, PairS::dma_rx_slot.channel, uint16_t>;
using Host = SpiHost<pair_n, link_pins, TxEngine, RxEngine>;
using CsPin = Pin<link_pins.nss.port, link_pins.nss.pin>;
/// The command channel's SCK: /256 of PCLK1 = 281.25 kHz, 28 us a
/// character.
constexpr SpiClock link_clock = SpiClock::div256;
constexpr const char* bus_clock_name = "PCLK1";
/// The pair's pad class (test_vx03_spi's pair_pad_class).
constexpr PinSpeed pair_pad_class = PinSpeed::medium;

// ---- the UART link -----------------------------------------------------------------

constexpr uint8_t link_n = vx03link::port_n;
constexpr DmaSlot link_rx_slot = usart_dma_rx_slot(link_n);
constexpr DmaSlot link_tx_slot = usart_dma_tx_slot(link_n);
using LinkRxEngine = DmaRxEngine<link_rx_slot.controller, link_rx_slot.channel>;
using LinkTxEngine = DmaTxEngine<link_tx_slot.controller, link_tx_slot.channel>;
/// A receive ring of 2048: letter v's half lap of 1024 (the file header).
using LinkPort = Uart<link_n, P, 2048, 256, UartFormat{}, LinkTxEngine, LinkRxEngine, vx03link::column>;
using LinkRxChannel = DmaChannel<link_rx_slot.controller, link_rx_slot.channel>;
using LinkTxChannel = DmaChannel<link_tx_slot.controller, link_tx_slot.channel>;
/// The Uart task owns the link port's vector and its two channels' only
/// while a leg runs.
volatile bool link_live = false;

/// What a leg's transport counted, read after the leg (init() clears them).
struct PortCounters {
    uint16_t ring_overruns = 0, ore = 0, fe = 0, pe = 0, ne = 0, dma_faults = 0;
    bool clean() const {
        return ring_overruns == 0u && ore == 0u && fe == 0u && pe == 0u && ne == 0u && dma_faults == 0u;
    }
};

// ---- the pressure's clock ----------------------------------------------------------

using Pulse = Tim<2>;
static_assert(Pulse::clock_hz(SysClock{}) == SysClock::hz, "TIM2 counts HCLK at this clock");
constexpr uint32_t pulse_per_us = SysClock::hz / 1'000'000u;
static_assert(389u * pulse_per_us - 1u <= 0xFFFFu, "the longest period fits TIM2's 16 bits");
constexpr const char* pressure_name = "TIM2";
constexpr const char* peer_name = v303 ? "the CH32V203C8T6" : "the CH32V303VCT6";

/// Letter w's lengths on this host (the file header): Minor's polled
/// requests and Pressure's two frames each, under dma_min_frames, so that
/// they take the polled loop and the pump.
struct LinkTraits {
    static constexpr uint8_t w_minor_len = 2;
    static constexpr uint8_t w_press_len = 2;
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
    (void)Pulse::set_period(static_cast<uint32_t>(period_us) * pulse_per_us - 1u);
    Pulse::clear_flags(Pulse::update_flag);
    Pulse::interrupts(Pulse::update_interrupt, true);
}
void pressure_go() { Pulse::enable(true); }
void pressure_halt() { Pulse::enable(false); }
void pressure_quiet() {
    Pulse::interrupts(Pulse::update_interrupt, false);
    Pulse::clear_flags(Pulse::update_flag);
    Pfic::clear_pending(Pulse::irq());
    spin(20'000u);   // the last tick's dispatch, if it was still running
}

uint32_t rx_stream_position() { return LinkRxChannel::remaining(); }

uint32_t division(SpiClock c) { return spi_division(c); }

/// The host reports a lost frame itself, as the transaction's status
/// (spi_overrun, letter p); nothing to latch at a reply.
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
          ", DMA faults ", c.dma_faults);
}

// =============================================================================
// The UART link: usart_link.hpp on the link port's polled resource, between legs
// =============================================================================

namespace ul {

/// The reviewed command path (vx03_link_dut.hpp): every wait, retry and
/// pad hand-over of the UART key's command channel is its.
using D = vx03link::Dut<Serial, SysClock>;

bool ensure() {
    if (D::ensure()) {
        return true;
    }
    bench.verdict("the peer answers a ping (the peer board running usart_peer)", false);
    return false;
}

void release() { D::release(); }

bool peer_report(link::Report& r, uint16_t after_ms) { return D::report(r, after_ms); }

/// Command an action from command mode; D::act() leaves settle_ms after
/// the ack, the peer opening its window settle_ms + 2 ms after it.
bool peer_act(link::Op op, const link::Params& a) { return D::act(op, a); }

}  // namespace ul

// =============================================================================
// The Uart task
// =============================================================================

/// The Uart task up at `rate` on the link port, its vectors live: the TX
/// pad parked high (a port in reset drives nothing, and the CH32V303's pad
/// then reads low: vx03_link_dut.hpp's open()), the block's reset pulsed -
/// whatever the command path left in it goes - then the transport's init()
/// and the RX pad's pull-up (test_vx03_serial's link_up()).
bool task_up(uint32_t rate) {
    ul::D::WTx::output(true);
    ul::D::W::bus_clock(true);
    ul::D::W::reset();
    link_live = true;
    if (!LinkPort::init(clock, rate)) {
        link_live = false;
        return false;
    }
    ul::D::WRx::input(PinPull::up);
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

/// The far chip's spi_peer: the sanity byte, the stratum's kind and the
/// other series' part number as its label (vx03_peer.hpp).
bool far_spi_peer(const spilink::Ident& d) {
    return d.sanity == spilink::ident_sanity && spilink::peer_kind(d.version) == spilink::peer_ch32vx03 &&
           d.label[0] == 'V' && d.label[1] == vx03link::far_series[0] && d.label[2] == vx03link::far_series[1] &&
           d.label[3] == vx03link::far_series[2];
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
        bench.verdict(pass_label(s), "ident is spi_peer's (the sanity byte), the CH32V port (0x05xx) on the far part",
                      got && far_spi_peer(d));
        bench.verdict(pass_label(s), "ten more frames answered, every one through the arbiter", pings == 10u);
        judge_pressure(s, st);
    }
}

// =============================================================================
// o - the matrix
// =============================================================================

/// The matrix's rate: /32 of PCLK1 = 2.25 MHz on the engines (32 frames,
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
constexpr uint8_t rung_count = sizeof rungs / sizeof rungs[0];
/// Judged to /8 = 9 MHz (the file header: the read-back's timing bound
/// from the two datasheets); /4 and /2 printed, each path's outcome counted.
constexpr SpiClock judged_top = SpiClock::div8;

enum class Path : uint8_t { polled16, polled8, pump16, engines };
constexpr uint8_t path_count = 4;
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
    return p == Path::polled16 ? 8u : p == Path::polled8 ? 2u : p == Path::pump16 ? 16u : 32u;
}

static_assert(2u < Host::dma_min_frames && Host::dma_min_frames <= 32u,
              "letter p's byte windows of two frames take the polled loop, its 32-frame ones the engines");

/// A frame's core cycles at code `c`: bits x the division, HCLK = 2 x PCLK1.
uint32_t frame_cycles(SpiClock c, bool wide) {
    return (wide ? 16u : 8u) * division(c) * (SysClock::hz / SysClock::pclk1_hz);
}

/// Does the host keep two frames in flight on a receive at this code and
/// width (spi.hpp's threshold, write_ahead_from())?
bool two_in_flight(SpiClock c, bool wide) {
    const std::optional<SpiClock> from = Host::write_ahead_from(wide ? SpiDataSize::bits16 : SpiDataSize::bits8);
    return from && static_cast<uint8_t>(c) >= static_cast<uint8_t>(*from);
}

/// One pass's polled outcome: the rungs cut short by a detected overrun
/// (a bit a rung, bit k for rungs[k]), per polled path; and whether any
/// polled failure was something else - another status, a wrong run handed
/// back spi_ok, or an overrun where one frame is in flight.
uint8_t cut_short[2] = {};
bool polled_failed_otherwise = false;
uint8_t top_exact[2] = {};
uint8_t top_total[2] = {};

/// One ladder on one path: every judged rung byte-exact, the first failure's
/// outcome printed; the informational rungs counted apart.
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
        const bool judged = division(c) >= division(judged_top);
        if (!judged) {
            const uint8_t r = c == SpiClock::div4 ? 0u : 1u;
            ++top_total[r];
            if (ok) {
                ++top_exact[r];
            }
            continue;
        }
        if (!ok) {
            const bool two = two_in_flight(c, e.wide);
            print(serial, "    ", path_name(path), two ? " (two frames in flight" : " (one frame in flight",
                  ", a frame ", frame_cycles(c, e.wide), " cycles):", crlf);
            sl::print_outcome(e, o);
            if (path_polled(path)) {
                if (o.status == brio::spi_overrun && two) {
                    cut_short[path == Path::polled16 ? 0u : 1u] |= static_cast<uint8_t>(1u << k);
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
          division(*Host::write_ahead_from(SpiDataSize::bits8)), " (8-bit) and /",
          division(*Host::write_ahead_from(SpiDataSize::bits16)),
          " (16-bit); a frame at /256 is 4096 cycles (8-bit) and 8192 (16-bit), at /16 256 and 512; "
          "the engines from ", Host::dma_min_frames, " frames; /4 and /2 printed, not judged",
          crlf);
    for (uint8_t p = 0; p < pass_count; ++p) {
        const PassSpec& s = passes[p];
        pass_begin(s);
        if (!sl::need_peer()) {
            (void)pass_end(s);
            return;
        }
        cut_short[0] = cut_short[1] = 0;
        polled_failed_otherwise = false;
        top_exact[0] = top_exact[1] = 0;
        top_total[0] = top_total[1] = 0;
        const bool polled16 = climb(Path::polled16, 0x21);
        const bool polled8 = climb(Path::polled8, 0x2D);
        const bool pump16 = climb(Path::pump16, 0x35);
        const bool engines = climb(Path::engines, 0x43);
        const bool ping = sl::command(spilink::Op::ping);
        const PassStats st = pass_end(s);
        print_pass(s, st);
        print(serial, "    polled rungs cut short by a detected overrun: 16-bit");
        print_rungs(cut_short[0]);
        print(serial, "; 8-bit");
        print_rungs(cut_short[1]);
        print(serial, "; /4 exact on ", top_exact[0], " of ", top_total[0], " paths, /2 on ", top_exact[1], " of ",
              top_total[1], crlf);
        if (s.period_us == 0u) {
            bench.verdict(pass_label(s), "THE POLLED LOOP through the arbiter: 8 16-bit and 2 8-bit frames "
                                         "byte-exact both ways at every code to /8",
                          polled16 && polled8);
        } else {
            // Under pressure the polled loop runs inside the arbiter's
            // dispatch, which every dispatch above the arbiter holds off for
            // its whole length - a length the host's hold-off does not
            // count. Where two frames are in flight and the preemption
            // outlasts a frame, the one-deep receive buffer loses one and
            // the transaction ends spi_overrun: the service's promise is that
            // it detects it every time and never hands back a run with a
            // hole, and that is what is judged, the rungs cut short printed
            // beside it. One frame in flight cannot overrun: a preemption
            // there is a gap on the bus.
            bench.verdict(pass_label(s), "THE POLLED LOOP through the arbiter: byte-exact, or cut short only by "
                                         "a detected overrun where two frames are in flight and a dispatch above "
                                         "the arbiter outlasts a frame - never a run with a hole",
                          (polled16 && polled8) || !polled_failed_otherwise);
        }
        bench.verdict(pass_label(s), "THE RXNE PUMP: 16 16-bit frames byte-exact at every code to /8", pump16);
        bench.verdict(pass_label(s), "THE DMA ENGINES: 32 frames byte-exact at every code to /8", engines);
        bench.verdict(pass_label(s), "the command channel survives the ladders", ping);
        judge_pressure(s, st);
    }
}

// =============================================================================
// w - the arbiter under pressure
// =============================================================================

static_assert(w::client_len >= Host::dma_min_frames && w::minor_len < Host::dma_min_frames &&
                  w::press_len < Host::dma_min_frames,
              "letter w: the Client's requests ride the engines, Minor's polled ones the polled loop and "
              "Pressure's the pump (the file header)");
static_assert(8u * spi_division(w::rate) * (SysClock::hz / SysClock::pclk1_hz) == 4096u,
              "letter w: a byte frame at its rate is 4096 cycles (the file header)");

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

/// The boot's three results, printed by the banner.
bool clock_ok = false;
bool tick_ok = false;
bool host_ok = false;

// =============================================================================
// i - dark
// =============================================================================

/// A pad's MODE bits (its nibble's low two): 00 is an input.
template <typename Pad>
bool pad_is_output() {
    return (Pad::nibble() & 0x3u) != 0u;
}

uint32_t read_mstatus() {
    uint32_t v;
    __asm__ volatile("csrr %0, mstatus" : "=r"(v));
    return v;
}

using SpiTxChannel = DmaChannel<PairS::dma_tx_slot.controller, PairS::dma_tx_slot.channel>;
using SpiRxChannel = DmaChannel<PairS::dma_rx_slot.controller, PairS::dma_rx_slot.channel>;

void ti_dark() {
    bench.verdict("every object above the Console idle: every queue empty",
                  Pressure::queue.empty() && LineSink::queue.empty() && Client::queue.empty() &&
                      SpiArb::queue.empty() && Minor::queue.empty() && LinePort::queue.empty());
    const uint16_t timer_dma = static_cast<uint16_t>(Pulse::update_dma | Pulse::trigger_dma | Pulse::compare_dma(0) |
                                                     Pulse::compare_dma(1) | Pulse::compare_dma(2) |
                                                     Pulse::compare_dma(3));
    bench.verdict("TIM2 stopped, its update interrupt off, its line quiet and none of its DMA requests enabled",
                  !Pulse::enabled() && (Pulse::interrupts() & Pulse::update_interrupt) == 0u &&
                      (Pulse::interrupts() & timer_dma) == 0u && !Pfic::pending(Pulse::irq()));
    bench.verdict("no time event is armed (the arbiter's timeout among them)", !TE::next_deadline().has_value());
    bench.verdict("the SPI host idle with the select high, its DMA requests down and both its channels disabled",
                  !Host::busy() && (PairS::regs().CTLR2 & (spi_txdmaen | spi_rxdmaen)) == 0u &&
                      !SpiTxChannel::enabled() && !SpiRxChannel::enabled() && pad_is_output<CsPin>() &&
                      CsPin::read());
    bench.verdict("the link port given back: its gate closed, its pads not outputs, both its channels disabled",
                  !Rcc::enabled(ul::D::W::bus, usart_gate_for(link_n)) && !pad_is_output<ul::D::WTx>() &&
                      !pad_is_output<ul::D::WRx>() && !LinkRxChannel::enabled() && !LinkTxChannel::enabled() &&
                      !link_live);
    bench.verdict("the LED's pad PB2 is not an output", !pad_is_output<Pin<'B', 2>>());
    bench.verdict("HCLK from the 8 MHz crystal, the UART letters' premise", clock_ok);
    bench.verdict("the ceiling is the Console's and interrupts are enabled",
                  P::ceiling == console_index && P::interrupts_enabled());
    if constexpr (device::has_fpu) {
        // FS, mstatus[14:13]: 3 Dirty. The crt leaves it Dirty (its fcsr
        // write), the scheduler's entry saves and cleans it at the first
        // preemption, and nothing of the pack touches a float after that.
        const uint32_t fs = (read_mstatus() >> 13) & 0x3u;
        print(serial, "  mstatus.FS = ", fs, " (0 off, 1 Initial, 2 Clean, 3 Dirty)", crlf);
        bench.verdict("the float unit not Dirty: no float written since the last preemption's save", fs != 3u);
    }
}

// =============================================================================
// t, u, v - the UART
// =============================================================================

constexpr uint32_t uart_rates[] = {460'800u, 921'600u};
constexpr uint16_t echo_lines = 200;
constexpr uint16_t sink_lines = 300;

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
    // 921600 and a window of 20 lines: the file header's arithmetic.
    constexpr uint32_t rate = 921'600u;
    constexpr uint8_t window = 20;
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
    print(serial, crlf, "boot: clk=", clock_ok ? "HSE8x18=144MHz" : "FAILED (no crystal)", " tick=", tick_ok ? "STK" : "FAILED",
          " ruler=STK spi", pair_n, "=", host_ok ? "host" : "FAILED", crlf);
    print(serial, "test_vx03_rubato_link - the SPI and UART stacks under Rubato on the ", device::part_name,
          " against ", peer_name, " (z: spi_peer, y: usart_peer)", crlf);
    bench.menu();
}

/// DMA2's gate, the CH32V303's (its SPI3 and UART4 rows); a part with one
/// controller forms no Dma<2> at all.
template <bool on = v303>
void open_second_controller() {
    if constexpr (on) {
        Dma<on ? 2u : 1u>::open();
    }
}

} // namespace

// ---- target glue ------------------------------------------------------------
//
// Each vector is the binding of one part: the other part's body is empty.
// A vector that posts is BRIO_CH32_VECTOR, a naked trampoline under the
// CH32V303's ilp32f; one the CH32V203 alone uses is bound LEAF, which is
// BRIO_CH32_VECTOR exactly under the V4B's ilp32 and a true leaf (an empty
// body) on the CH32V303.

BRIO_CH32_VECTOR(usart1_handler) {
    if (Serial::isr()) {
        brio::post<Console>(ConsoleEvent{console_rx});
    }
}

BRIO_CH32_LEAF_VECTOR(systick_handler) { brio::Ticker::tick(); }

BRIO_CH32_SCHEDULER_VECTOR(software_handler) { brio::scheduler_trap<K>(); }
BRIO_CH32_SCHEDULER_VECTOR(ecall_m_handler) { brio::scheduler_return(); }

/// The pressure: TIM2's update, where it landed, and the post.
BRIO_CH32_VECTOR(tim2_handler) {
    Pulse::clear_flags(Pulse::update_flag);
    pressure_landed();
}

// The SPI host: the instance's vector (the pump) and its two channels'
// (the engines' completion on the receive channel, an error on either).
BRIO_CH32_LEAF_VECTOR(spi2_handler) {
    if constexpr (!v303) {
        if (Host::isr()) {
            done_from_dma = false;
            brio::post<SpiArb>(brio::TransferDone{Host::status()});
        }
    }
}
BRIO_CH32_VECTOR(spi3_handler) {
    if constexpr (v303) {
        if (Host::isr()) {
            done_from_dma = false;
            brio::post<SpiArb>(brio::TransferDone{Host::status()});
        }
    }
}
/// The CH32V203's SPI2: DMA1's channel 4 receives, 5 transmits.
BRIO_CH32_LEAF_VECTOR(dma1_channel4_handler) {
    if constexpr (!v303) {
        if (Host::dma_rx_isr()) {
            done_from_dma = true;
            brio::post<SpiArb>(brio::TransferDone{Host::status()});
        }
    }
}
BRIO_CH32_LEAF_VECTOR(dma1_channel5_handler) {
    if constexpr (!v303) {
        if (Host::dma_tx_isr()) {
            done_from_dma = true;
            brio::post<SpiArb>(brio::TransferDone{Host::status()});
        }
    }
}
/// The CH32V303's SPI3: DMA2's channel 1 receives, 2 transmits.
BRIO_CH32_VECTOR(dma2_channel1_handler) {
    if constexpr (v303) {
        if (Host::dma_rx_isr()) {
            done_from_dma = true;
            brio::post<SpiArb>(brio::TransferDone{Host::status()});
        }
    }
}
BRIO_CH32_VECTOR(dma2_channel2_handler) {
    if constexpr (v303) {
        if (Host::dma_tx_isr()) {
            done_from_dma = true;
            brio::post<SpiArb>(brio::TransferDone{Host::status()});
        }
    }
}

// The link port: the instance's vector (the idle line, a burst's first
// frame, the errors) and its two channels' (the ring's marks, a block's
// completion).
BRIO_CH32_LEAF_VECTOR(usart3_handler) {
    if constexpr (!v303) {
        if (link_live && LinkPort::isr()) {
            brio::post<LinePort>(brio::RxActivity{});
        }
    }
}
BRIO_CH32_LEAF_VECTOR(dma1_channel3_handler) {
    if constexpr (!v303) {
        if (link_live && LinkPort::dma_isr()) {
            brio::post<LinePort>(brio::RxActivity{});
        }
    }
}
BRIO_CH32_LEAF_VECTOR(dma1_channel2_handler) {
    if constexpr (!v303) {
        if (link_live && LinkPort::dma_isr()) {
            brio::post<LinePort>(brio::RxActivity{});
        }
    }
}
BRIO_CH32_VECTOR(uart4_handler) {
    if constexpr (v303) {
        if (link_live && LinkPort::isr()) {
            brio::post<LinePort>(brio::RxActivity{});
        }
    }
}
BRIO_CH32_VECTOR(dma2_channel3_handler) {
    if constexpr (v303) {
        if (link_live && LinkPort::dma_isr()) {
            brio::post<LinePort>(brio::RxActivity{});
        }
    }
}
BRIO_CH32_VECTOR(dma2_channel5_handler) {
    if constexpr (v303) {
        if (link_live && LinkPort::dma_isr()) {
            brio::post<LinePort>(brio::RxActivity{});
        }
    }
}

int main() {
    clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    tick_ok = brio::Ticker::init(clock);
    brio::Dma<1>::open();
    open_second_controller();
    CsPin::output(true);
    host_ok = Host::init(clock);
    Host::pad_speed(pair_pad_class);
    Pulse::init();
    (void)Pulse::configure(brio::TimConfig{.prescaler = 0, .period = 0xFFFFu});
    brio::Pfic::enable(Pulse::irq());
    brio::scheduler_init();

    bench.letter('n', "the peer's command channel through the arbiter", tn_peer_link);
    bench.letter('o', "the matrix on the DMA engines: modes, widths, bit orders", to_matrix);
    bench.letter('p', "the ladders: the polled loop both widths, the pump, the engines, every code", tp_ladders);
    bench.letter('w', "the arbiter under pressure: three requesters, one bus", tw_arbiter);
    bench.letter('i', "dark: every object idle, nothing lit", ti_dark);
    bench.second_letter('t', "the UART received: numbered lines through the peer's echo", tt_lines);
    bench.second_letter('u', "the UART sent: a blast of numbered lines into the peer's sink", tu_blast);
    bench.second_letter('v', "the loan under preemption: a slow borrower at 921600", tv_loan);

    (void)serial_ok;
    brio::enable_interrupts();
    K::run();
}
