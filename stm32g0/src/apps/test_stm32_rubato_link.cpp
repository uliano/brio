// test_stm32_rubato_link - the bench suite of the SPI and UART stacks of the
// STM32G0 stratum under Rubato, the preemptive kernel (kernel/rubato.hpp):
// util/spi_bus.hpp's arbiter over stm32g0/spi.hpp's SpiHost and its DMA
// engines, and util/line_reader.hpp's LineReader over stm32g0/usart.hpp's
// Uart and its receive engine, run against the desk's Nucleo-F446RE and
// its unchanged instruments (stm32f4/src/apps/spi_peer.cpp, usart_peer.cpp)
// while an object of the highest priority preempts them at moments the
// traffic does not choose. Every verdict is the traffic judged byte-exact,
// as the Tenuto suites (test_stm32_spi's peer letters, test_stm32_serial's
// second all-key) judge it; nothing here changes a driver or a service.
//
// A test_<target>_<subject> suite in the bench grammar (util/testbench.hpp)
// that runs inside the kernel it tests, as test_stm32_rubato does: the
// program is one Rubato pack of seven objects - Pressure, LineSink, Client,
// SpiArb, Minor, LinePort and Console, from the highest priority down - and
// everything of it that names no register, the pack, the passes, the
// numbered lines, the window client and the UART legs, is the link suites'
// shared body (avrxt/src/apps/rubato_link.hpp, whose header describes each
// object and the passes). What is this family's is here, each choice from
// RM0444 and the driver headers:
//
//   THE SPI HOST. SpiHost<1> on PB3/PB4/PB5 with the half-word DMA engines
//   on DMA1 channels 1 (transmit, armed for its errors alone) and 2
//   (receive, the transaction's completion) - test_stm32_spi's engined
//   host. Its paths: the polled loop in its receive shape, frame by frame
//   below packed_min_frames (6) and PACKED from it (two frames an access to
//   DR, 35.5.9); the RXNE pump; the engines from dma_min_frames (8). A
//   16-bit window at an odd address goes to the pump (10.4.3: the DMA
//   cannot read a half-word across one), which keeps letter p's pump
//   ladder off the engines. THE FIFO IS THE WRITE-AHEAD (35.5.9): the
//   receive shapes keep no more frames in flight than the RXFIFO holds -
//   three byte frames, two half-words, four packed - so a dispatch above
//   the arbiter that holds the polled loop off for any length leaves a GAP
//   ON THE BUS and never a lost frame; the host has no spi_overrun, its
//   one failure spi_stalled, a wait budget counted in poll turns that a
//   preemption does not spend (spi.hpp, "The host task"). MOSI goes out
//   at `high`, one class below the driver's default (docs/stm32g0/spi.md,
//   "The peer link": the very-high class slips the rising-edge modes at
//   the F446's client). The clock is static - a DynamicClock must not
//   switch with a transaction in flight - and the host stays InFlash.
//   SPI1's vector is the arbiter's for the image's whole life: nothing
//   inits the host after main(), and Host::prime() in letter w is the one
//   direct verb.
//   THE UART. Uart<1> on PA9/PA10 with the receive engine on DMA1 channel
//   3, a circular ring of 1024 (HardwareRing): isr() takes IDLE, dma_isr()
//   the half and full marks, and the RXFIFO stays on under the engine with
//   RXFT at seven eighths, its kick counted by rx_kicks() (docs/stm32g0/
//   usart.md, "The receive side"); the interrupt transmitter beside it.
//   OVER16, the default: OVER8 buys this divisor no resolution - its BRR
//   drops USARTDIV's lowest bit (33.5.7: BRR[2:0] = USARTDIV[3:0] >> 1),
//   so 128 MHz / 139 is 128 MHz / 138, the same 64 MHz / 69 - and halves
//   the receiver's samples a bit. Channels 2 and 3 share
//   DMA1_Channel2_3's vector, which calls both bodies.
//   THE PRESSURE. TIM14's update interrupt, 16 bits on TIMPCLK = HCLK =
//   64 MHz (stm32g0/clock.hpp pins the APB prescaler at 1) at prescaler
//   0: 97 / 193 / 389 us are 6208 / 12352 / 24896 counts. Its dispatch, a
//   3000-cycle spin and a road-2 post of 517 (docs/stm32g0/platform.md),
//   is shorter than the shortest period's 6208 cycles, so no tick finds
//   the last one still queued; at 97 us x 3000 it takes 48 % of the core.
//   THE RULER. TIM2 free-running at HCLK, one load of CNT, a count a core
//   cycle - this core has no DWT (ARMv6-M); the pressure's timer and the
//   ruler count one clock, so judge_pressure()'s arithmetic is exact.
//   THE UART KEY'S COMMAND CHANNEL. USART1 on its polled resource at
//   115200, FIFO on, between legs (test_stm32_serial's dut_link()): a
//   character every 87 us, against a longest preemption of 3000 cycles
//   (47 us) - the receive FIFO holds eight.
//
// THE WIRES, fixed on the desk (test_stm32_spi's peer letters and
// test_stm32_serial's two-board set; the same fitting test_stm32f4_rubato_link
// runs on, the roles exchanged):
//   SPI1    SCK PB3 -> F446 PB13, MISO PB4 <- F446 PB14, MOSI PB5 -> F446
//           PB15, the select PA15 (a GPIO) -> F446 PB12
//   USART1  PA9 (TX) -> F446 PA10, PA10 (RX) <- F446 PA9
// plus a ground. The SPI letters (key z) want the peer running `spi_peer`,
// the UART ones (key y) `usart_peer`. NOTHING LIT: no letter drives the
// LED's pad.
//
// The letters (z: n o p w i; y: t u v; x outside both), each of z and y
// run as seven passes (the shared body's control and six pressures):
//   n  the peer's command channel through the arbiter: a ping, ident (the
//      sanity byte and the STM32F4 port's kind), and ten more pings, every
//      frame at PCLK/256 = 250 kHz and its answer decoded by the Client.
//   o  the matrix on the DMA engines (32 frames a window at PCLK/32 =
//      2 MHz, against the peer's DMA serve): the four modes in 8- and
//      16-bit frames, LSb first in both widths, and a DORD mismatch as an
//      exact two-way bit reversal - byte-exact at both ends; the data
//      windows' bytes over their cycles printed against the control's.
//   p  the ladders through the arbiter, every BR code from PCLK/256 up:
//      the POLLED loop PACKED (8 frames) and FRAME BY FRAME (5), the RXNE
//      PUMP in byte frames (7, under the engines' crossover) and in 16-bit
//      frames (16 from an odd address), and the DMA engines (32 frames).
//      Judged to PCLK/8 = 8 MHz; printed, each path's outcome counted,
//      PCLK/2 = 32 MHz (above the F446 client's 22.5 MHz, DS10693 table 63,
//      where test_stm32_spi's letter p prints it too) and PCLK/4 = 16 MHz,
//      which slips on this fitting between one window in twenty and one in
//      a hundred and sixty, with the pressure's timer silent as often as
//      under it, on every path, the engines' included, which no preemption
//      reaches (letter x). The
//      polled loop runs inside the arbiter's dispatch, so a dispatch above
//      it holds it off for its whole length; on this host that costs the
//      bus a gap and nothing else, so the verdict is STRICT under pressure
//      as in the control - every judged rung byte-exact on every path -
//      and the host's OVR flag, read at every reply before any other
//      transaction (client_replied()), never set.
//   x  (OUTSIDE z, no verdict) the gap statistics: forty windows of one
//      shape with the pressure's timer silent and forty under 97 us x
//      3000, the windows not byte-exact counted - the 16-bit pump at
//      PCLK/4 and PCLK/8, the 16-bit polled loop and engines at PCLK/4,
//      the 8-bit pump at PCLK/4. The instrument behind letter p's judged
//      ceiling.
//   w  the arbiter under pressure: the Client above it (20 requests of 6
//      frames on the engines, two outstanding), Minor below it (10 POLLED
//      requests of 8 frames, one outstanding) and, in a pressure pass,
//      Pressure itself (8 requests of 3 frames on the pump, posted from its
//      tick, one outstanding), all at PCLK/256 inside ONE select window the
//      letter holds - the peer's exchange is one burst - of constant frames
//      (pattern_fixed). Every reply to its requester in its order, every
//      one spi_ok, nothing rejected, the per-bus timeout never firing, the
//      peer counting every frame exact.
//   i  dark: every object idle, TIM14 stopped and its line quiet, no time
//      event armed, SPI1 idle with the select high and both its engines
//      idle, USART1 given back, the LED's pad not an output, the ceiling
//      the Console's own.
//   t  the UART, received: numbered, checked lines written by the Console
//      and turned around by the peer's ECHO at 460800 and 921600, assembled
//      by the LineReader from the receive engine's ring and lent to
//      LineSink: 200 lines a leg, none lost, none out of order, none
//      corrupt, the port's own counters (ring overruns, ORE, FE, PE, NE,
//      DMA faults, RXFT kicks, line overflows, torn lines) zero, and the
//      peer's count of the bytes it echoed exact. THE ECHO HAS NO FIFO
//      (stm32f4/src/apps/usart_peer.cpp: it reads the data register at
//      RXNE and writes the character back when TXE allows), so what it can
//      hold of a writer's lead is its transmit data register and its
//      receive data register - two characters - before an overrun. At
//      921600: this end 64 MHz / 69 = 927536 Bd, +0.64 % of the divisor
//      and the HSI16's +0.28 % (usart.md's baud matrix: +9472 ppm
//      measured); the F446 90 MHz / (16 x 6.125) = 918367, -0.35 % on its
//      crystal; the writer 1.27 % ahead, 0.9 of a character over a line of
//      at most 71 - under the two - drained by the two frame times the
//      writer leaves after its last stop bit. At 460800: 64 MHz / 139 =
//      460432 Bd, +0.20 % with the HSI, against the F446's 90 MHz /
//      (16 x 12.1875) = 461538, +0.16 %: 0.04 %, nothing a line.
//   u  the UART, sent: 300 numbered lines back to back through write_bulk()
//      into the peer's SINK at 460800 and 921600, judged by the peer's
//      report - the bytes counted, their 16-bit sum, FE, PE and ORE zero -
//      and the wire's time printed against the control's. A receiver
//      resynchronises on every start bit, so the 1.27 % at 921600 is inside
//      its tolerance (RM0390 25.4.5) and builds no backlog.
//   v  the loan under preemption: t's leg at 460800 with LineSink spinning
//      6000 cycles a line, so Pressure lands inside the borrower's dispatch,
//      and the lines STREAMED - line k written once line k - 5 is back - so
//      that the port lends its lines while the next ones arrive (the
//      receive stream's count read at the sink's entry and exit says how
//      often it moved): every line exact, every line lent alone (the sink's
//      queue empty at its entry, the sink never nested in itself), the two
//      buffers strictly alternating, and the lender never reading what the
//      borrower wrote over. THE RATE AND THE WINDOW ARE THE ECHO'S: a
//      writer that never pauses builds a backlog at the echo of its whole
//      lead times the leg, so the window must STOP the writer between the
//      port's edges. The port is told at an idle line or at a half lap of
//      the ring (512 characters); once told, the writer is at most 5 lines
//      (355 characters) and the line the mark split (70) ahead of what the
//      sink holds, 425 < 512, so it stops before the next mark and the
//      silence tells the port again: no continuous run is longer than some
//      427 characters. At 460800 the lead is 0.04 % at the HSI's mean and
//      0.09 % at a few hundred ppm of its wander: 0.17 to 0.38 of a
//      character at the echo, inside half of one; at 921600 the 1.27 %
//      would be 5.4, past the two. A run that long still crosses a half-lap mark more
//      often than not, so both of the engine's edges lend lines. A pressure
//      pass judges that Pressure landed inside the borrower at least once:
//      the sink's dispatches add up to 200 x 6000 = 1.2 M cycles at least,
//      48 ticks of the longest period (24896 cycles) on average, so a pass
//      with none has a chance of e^-48.
//
// build: boards = g0b1re
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
#include "stm32g0/clock.hpp"
#include "stm32g0/dma.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/platform.hpp"
#include "stm32g0/spi.hpp"
#include "stm32g0/ticker.hpp"
#include "stm32g0/tim.hpp"
#include "stm32g0/usart.hpp"
#include "util/line_reader.hpp"
#include "util/print.hpp"
#include "util/spi_bus.hpp"
#include "util/testbench.hpp"

// THE PROTOCOL HEADERS ARE SHARED, NOT COPIED: the far board's two
// instruments compile the same files.
#include "../../../avrxt/src/apps/spi_link.hpp"
#include "../../../avrxt/src/apps/usart_link.hpp"

/// The PLL on the HSI16 at 64 MHz, static: the SPI host's BR and the
/// USART's divisor divide one fixed PCLK = HCLK.
using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Stm32g0RubatoPlatform<>;
constexpr uint32_t core_hz = SysClock::hz;
constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000u;

/// TIM2 free-running at HCLK: one load of CNT, a count a cycle.
using Clock32 = Tim<2>;
struct Ruler {
    [[gnu::always_inline]] static uint32_t now() { return Clock32::count(); }
};

[[gnu::always_inline]] inline void spin(uint32_t cycles) {
    const uint32_t t0 = Ruler::now();
    while (Ruler::now() - t0 < cycles) {
    }
}

// ---- the console -------------------------------------------------------------------

constexpr UartPins console_pins{.tx = {'A', 2, PinFunction::af1}, .rx = {'A', 3, PinFunction::af1}};
using Serial = Uart<2, console_pins>;
constexpr Serial serial;
TwoKeyTestBench<Serial, 'y'> bench;

// ---- the SPI link ------------------------------------------------------------------

/// SPI1 at AF0 (DS13560 tables 13 and 15); PA15 is the select, a GPIO the
/// Request carries.
constexpr SpiPins link_pins{
    .sck = {'B', 3, PinFunction::af0},
    .miso = {'B', 4, PinFunction::af0},
    .mosi = {'B', 5, PinFunction::af0},
    .nss = {'A', 15, PinFunction::af0},
};
/// The engines' element is DR's widest access, so 16-bit frames ride them
/// a half-word a beat and byte frames a byte.
using TxEngine = DmaTxEngine<1, 1, uint16_t>;
using RxEngine = DmaRxEngine<1, 2, uint16_t>;
using Host = SpiHost<1, link_pins, TxEngine, RxEngine>;
using CsPin = Pin<link_pins.nss.port, link_pins.nss.pin>;
/// The command channel's SCK: PCLK/256 = 250 kHz, 32 us a character.
constexpr SpiClock link_clock = SpiClock::div256;
constexpr const char* bus_clock_name = "PCLK";

// ---- the UART link -----------------------------------------------------------------

constexpr UartPins uart_link_pins{.tx = {'A', 9, PinFunction::af1}, .rx = {'A', 10, PinFunction::af1}};
constexpr UartOptions uart_link_opts{.tx_speed = PinSpeed::very_high};
/// USART1_RX on DMA1 channel 3 (test_stm32_serial's LinkDma), a ring of
/// 1024: letter v's half lap of 512 (the header's arithmetic).
using LinkRxEngine = DmaRxEngine<1, 3>;
using LinkPort = Uart<1, uart_link_pins, 1024, 64, NoDmaEngine, LinkRxEngine, uart_link_opts>;
/// The Uart task owns USART1's vector and channel 3's half of the shared
/// DMA vector only while a leg runs.
volatile bool link_live = false;

/// What a leg's transport counted, read after the leg (init() clears them).
struct PortCounters {
    uint8_t ring_overruns = 0, ore = 0, fe = 0, pe = 0, ne = 0, dma_faults = 0, kicks = 0;
    bool clean() const {
        return ring_overruns == 0u && ore == 0u && fe == 0u && pe == 0u && ne == 0u && dma_faults == 0u &&
               kicks == 0u;
    }
};

// ---- the pressure's clock ----------------------------------------------------------

using Pulse = Tim<14>;
constexpr uint32_t pulse_per_us = SysClock::hz / 1'000'000u;   // TIMPCLK = HCLK
static_assert(389u * pulse_per_us - 1u <= 0xFFFFu, "the longest period fits TIM14's 16 bits");
constexpr const char* pressure_name = "TIM14";
constexpr const char* peer_name = "the STM32F446RE";

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
    Nvic::clear_pending(Pulse::irq());
    spin(20'000u);   // the last tick's dispatch, if it was still running
}

uint32_t rx_stream_position() { return LinkRxEngine::remaining(); }

uint32_t division(SpiClock c) { return spi_division(c); }

/// The host's OVR flag at a reply: the receive shapes never let it rise
/// (the file header), the transmit-only shape clears what it raised in its
/// own tail, so a set flag here is a frame the host did not read back.
volatile uint32_t ovr_at_reply = 0;
void client_replied() {
    if (Host::Resource::overrun()) {
        ovr_at_reply = ovr_at_reply + 1u;
    }
}

PortCounters read_port_counters() {
    PortCounters c{};
    c.ring_overruns = LinkPort::rx_overruns();
    c.ore = LinkPort::hw_overruns();
    c.fe = LinkPort::frame_errors();
    c.pe = LinkPort::parity_errors();
    c.ne = LinkPort::noise_errors();
    c.dma_faults = LinkPort::dma_faults();
    c.kicks = LinkPort::rx_kicks();
    return c;
}

void print_port(const PortCounters& c) {
    print(serial, "ring overruns ", c.ring_overruns, ", ORE ", c.ore, ", FE ", c.fe, ", PE ", c.pe, ", NE ", c.ne,
          ", DMA faults ", c.dma_faults, ", RXFT kicks ", c.kicks);
}

// =============================================================================
// The UART link: usart_link.hpp on USART1's polled resource, between legs
// =============================================================================

namespace ul {

using U1 = Usart<1>;
using U1Tx = Pin<'A', 9>;
using U1Rx = Pin<'A', 10>;
constexpr uint32_t u1_hz = SysClock::pclk_hz;
static_assert(usart_bus_clock(1).apb2, "USART1's gate is in APBENR2 (letter i reads it)");

link::Decoder dec;
const uint8_t no_payload[1] = {0};
bool quiet = false;

void flush() {
    U1::request(UsartRequest::flush_receive);
    U1::clear_flags(UsartClear::receive_errors | UsartClear::idle);
}

/// The command channel, 8N1 at 115200 on the polled resource, the FIFO on:
/// the block reset and programmed with the pads off it (PA9 an input on its
/// pull-up, so the line stays high through TE's idle frame), then the pads
/// handed over (test_stm32_serial's dut_link()).
bool command_mode() {
    dec.reset();
    Nvic::disable(U1::irq());
    U1Tx::input(PinPull::up);
    U1::bus_clock(true);
    U1::enable(false);
    U1::reset();
    (void)U1::kernel_clock(UsartClock::pclk);
    const std::optional<uint16_t> brr = usart_brr(u1_hz, link::command_baud);
    if (!brr || !U1::configure(UartFormat{}, *brr) || !U1::fifo(true)) {
        return false;
    }
    U1::enable(true);
    wait_us(2u * (1'000'000u / link::command_baud) + 20u);
    U1Tx::function(PinFunction::af1, {.speed = PinSpeed::very_high});
    U1Rx::function(PinFunction::af1, {.pull = PinPull::up});
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
    while (!U1::flag(UsartFlag::txfnf) && Ticker::millis() - t0 < 50u) {
    }
    U1::write_word(b);
}

void wait_tc() {
    const uint32_t t0 = Ticker::millis();
    while (!U1::flag(UsartFlag::tc) && Ticker::millis() - t0 < 50u) {
    }
}

/// A frame, polled: a character every 87 us at 115200 into an eight-deep
/// FIFO, and the longest preemption here is 3000 cycles and the scheduler
/// - under 60 us - so the FIFO never fills.
bool recv_frame(link::Frame& out, uint16_t ms) {
    const uint32_t t0 = Ticker::millis();
    dec.reset();
    while (Ticker::millis() - t0 < ms) {
        const uint32_t st = U1::status();
        if ((st & UsartFlag::rxne) == 0u) {
            if ((st & UsartFlag::ore) != 0u) {
                U1::clear_flags(UsartClear::ore);
            }
            continue;
        }
        if ((st & UsartFlag::receive_errors) != 0u) {
            U1::clear_flags(st & UsartClear::receive_errors);   // before the pop
        }
        const uint8_t b = U1::read_data();
        if ((st & UsartFlag::receive_errors) != 0u) {
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

bool ensure() {
    quiet = true;
    bool found = false;
    for (uint8_t k = 0; k < 3u && !found; ++k) {
        found = command_mode() && command(link::Op::ping);
    }
    quiet = false;
    if (!found) {
        print(serial, "  THE PEER DID NOT ANSWER: the STM32F446RE must be running `usart_peer`", crlf);
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
// The Uart task
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
        bench.verdict(pass_label(s), "ident is spi_peer's (the sanity byte), the STM32F4 port (0x04xx)",
                      got && d.sanity == spilink::ident_sanity &&
                          spilink::peer_kind(d.version) == spilink::peer_stm32f4);
        bench.verdict(pass_label(s), "ten more frames answered, every one through the arbiter", pings == 10u);
        judge_pressure(s, st);
    }
}

// =============================================================================
// o - the matrix
// =============================================================================

/// The matrix's rate: PCLK/32 = 2 MHz on the engines (32 frames, past
/// dma_min_frames).
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
/// Judged to PCLK/8 = 8 MHz. PCLK/2 = 32 MHz is above the F446 client's
/// specified 22.5 MHz (DS10693 table 63); PCLK/4 = 16 MHz slips on this
/// fitting between one window in twenty and one in a hundred and sixty,
/// with the pressure's timer silent as with it, the engines' windows
/// included, which no preemption reaches (letter x, the document's
/// figures) - a rate whose verdict would be the wire's.
/// Both printed, each path's outcome counted, not judged.
constexpr SpiClock judged_top = SpiClock::div8;

enum class Path : uint8_t { packed, by_frame, pump8, pump16, engines };
const char* path_name(Path p) {
    switch (p) {
        case Path::packed: return "polled loop, packed";
        case Path::by_frame: return "polled loop, frame by frame";
        case Path::pump8: return "RXNE pump, 8-bit";
        case Path::pump16: return "RXNE pump, 16-bit";
        default: return "DMA engines";
    }
}

/// A frame's core cycles at code `c`: bits x the division, HCLK = PCLK.
uint32_t frame_cycles(SpiClock c, bool wide) { return (wide ? 16u : 8u) * division(c); }

static_assert(Host::packed_min_frames <= 8u && 5u < Host::packed_min_frames,
              "letter p's polled windows are one packed and one frame by frame");
static_assert(Host::dma_min_frames <= 32u && 7u < Host::dma_min_frames,
              "letter p's engine windows ride the engines, its 8-bit pump windows the pump");

/// The informational rungs' outcomes, PCLK/4 and PCLK/2, printed after the
/// pass.
uint8_t top_exact[2] = {};
uint8_t top_total[2] = {};

/// One ladder on one path: every judged rung byte-exact, the first failure's
/// outcome printed; the informational rungs counted apart.
bool climb(Path path, uint8_t seed) {
    bool all = true;
    uint8_t k = 0;
    for (const SpiClock c : rungs) {
        sl::Exchange e{};
        e.rate = c;
        e.polled = path == Path::packed || path == Path::by_frame;
        e.wide = path == Path::pump16;
        e.odd = path == Path::pump16;
        e.count = path == Path::packed     ? 8u
                  : path == Path::by_frame ? 5u
                  : path == Path::pump8    ? 7u
                  : path == Path::pump16   ? 16u
                                           : 32u;
        e.seed_a = static_cast<uint8_t>(seed + 9u * k);
        e.seed_b = static_cast<uint8_t>(seed + 0x40u + 5u * k);
        ++k;
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
            print(serial, "    ", path_name(path), " (a frame ", frame_cycles(c, e.wide), " cycles):", crlf);
            sl::print_outcome(e, o);
        }
        all = all && ok;
    }
    return all;
}

void tp_ladders() {
    print(serial, "  the polled loop packs from ", Host::packed_min_frames, " frames (", Host::ahead_bytes,
          " byte frames in flight frame by frame, four packed), the pump keeps ", Host::ahead_halfwords,
          " half-words in flight, the engines take a phase of ", Host::dma_min_frames,
          " frames or more; PCLK/4 and PCLK/2 printed, not judged", crlf);
    for (uint8_t p = 0; p < pass_count; ++p) {
        const PassSpec& s = passes[p];
        pass_begin(s);
        if (!sl::need_peer()) {
            (void)pass_end(s);
            return;
        }
        ovr_at_reply = 0;
        top_exact[0] = top_exact[1] = 0;
        top_total[0] = top_total[1] = 0;
        const bool packed = climb(Path::packed, 0x21);
        const bool by_frame = climb(Path::by_frame, 0x2D);
        const bool pump8 = climb(Path::pump8, 0x39);
        const bool pump16 = climb(Path::pump16, 0x35);
        const bool engines = climb(Path::engines, 0x43);
        const uint32_t ovr = ovr_at_reply;
        const bool ping = sl::command(spilink::Op::ping);
        const PassStats st = pass_end(s);
        print_pass(s, st);
        print(serial, "    the host's OVR at a reply over the pass: ", ovr, "; PCLK/4 exact on ", top_exact[0],
              " of ", top_total[0], " paths, PCLK/2 on ", top_exact[1], " of ", top_total[1], crlf);
        // A dispatch above the arbiter holds the polled loop off for its
        // whole length; with the FIFO as the write-ahead that is a gap on
        // the bus and never a lost frame (the file header), so the verdict
        // under pressure is the control's.
        bench.verdict(pass_label(s), "THE POLLED LOOP, PACKED, through the arbiter: 8 frames byte-exact both "
                                     "ways at every code to PCLK/8",
                      packed);
        bench.verdict(pass_label(s), "THE POLLED LOOP, FRAME BY FRAME: 5 frames byte-exact both ways at every "
                                     "code to PCLK/8",
                      by_frame);
        bench.verdict(pass_label(s), "THE RXNE PUMP: 7 8-bit frames byte-exact at every code to PCLK/8", pump8);
        bench.verdict(pass_label(s), "THE RXNE PUMP: 16 16-bit frames byte-exact at every code to PCLK/8", pump16);
        bench.verdict(pass_label(s), "THE DMA ENGINES: 32 frames byte-exact at every code to PCLK/8", engines);
        bench.verdict(pass_label(s), "the host's OVR flag never set at a reply: no frame left unread", ovr == 0u);
        bench.verdict(pass_label(s), "the command channel survives the ladders", ping);
        judge_pressure(s, st);
    }
}

// =============================================================================
// x - the gap statistics (outside z, no verdict)
// =============================================================================

/// The instrument behind letter p's pump finding: one shape at a time,
/// forty windows with the pressure's timer silent and forty under 97 us x
/// 3000 (the heaviest gaps a preemption leaves on the bus), the windows
/// that were not byte-exact counted and the first printed. No verdict: a
/// table of which width, path and rate a long gap between two frames
/// breaks, for the document.
void tx_gaps() {
    struct Shape {
        const char* name;
        SpiClock rate;
        bool wide, polled, odd;
        uint16_t count;
    };
    static const Shape shapes[] = {
        {"RXNE pump, 16-bit, PCLK/4", SpiClock::div4, true, false, true, 16},
        {"RXNE pump, 16-bit, PCLK/8", SpiClock::div8, true, false, true, 16},
        {"polled loop, 16-bit, PCLK/4", SpiClock::div4, true, true, false, 16},
        {"RXNE pump, 8-bit, PCLK/4 (7 frames, under the engines' crossover)", SpiClock::div4, false, false, false, 7},
        {"DMA engines, 16-bit, PCLK/4", SpiClock::div4, true, false, false, 32},
    };
    static_assert(7u < Host::dma_min_frames, "the 8-bit shape rides the pump");
    constexpr uint16_t windows = 40;
    if (!sl::need_peer()) {
        return;
    }
    for (const Shape& sh : shapes) {
        for (uint8_t press = 0; press < 2u; ++press) {
            const PassSpec& s = passes[press == 0u ? 0u : 4u];
            pass_begin(s);
            uint16_t bad = 0;
            bool shown = false;
            for (uint16_t k = 0; k < windows; ++k) {
                sl::Exchange e{};
                e.rate = sh.rate;
                e.wide = sh.wide;
                e.polled = sh.polled;
                e.odd = sh.odd;
                e.count = sh.count;
                e.seed_a = static_cast<uint8_t>(0x31u + 3u * k);
                e.seed_b = static_cast<uint8_t>(0x75u + 7u * k);
                const sl::Outcome o = sl::run(e);
                if (!sl::exact(e, o)) {
                    ++bad;
                    if (!shown) {
                        sl::print_outcome(e, o);
                        shown = true;
                    }
                }
            }
            const PassStats st = pass_end(s);
            print(serial, "  ", sh.name, " [", pass_label(s), "]: ", bad, " of ", windows,
                  " windows not exact; Pressure landed in SpiArb ", st.land[arb_index], ", inside a completion ",
                  st.completion, crlf);
        }
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
}

// =============================================================================
// i - dark
// =============================================================================

void ti_dark() {
    bench.verdict("every object above the Console idle: every queue empty",
                  Pressure::queue.empty() && LineSink::queue.empty() && Client::queue.empty() &&
                      SpiArb::queue.empty() && Minor::queue.empty() && LinePort::queue.empty());
    bench.verdict("TIM14 stopped, its update interrupt off and its line quiet",
                  !Pulse::enabled() && (Pulse::interrupts() & Pulse::update_interrupt) == 0u &&
                      !Nvic::pending(Pulse::irq()));
    bench.verdict("no time event is armed (the arbiter's timeout among them)", !TE::next_deadline().has_value());
    // The engined host keeps CR2's TXDMAEN and RXDMAEN set for its whole
    // life (its boot configuration, spi.hpp's boot_config()), and a
    // channel whose block ended stays enabled with its count at zero until
    // the next block's restart (stm32g0/dma.hpp): idle is the transmit
    // engine with no block in flight and both channels' counts at zero, so
    // no request can move a beat.
    bench.verdict("SPI1 idle with the select high, both its engines idle with their counts at zero",
                  !Host::Resource::busy() && !TxEngine::busy() &&
                      DmaChannel<1, TxEngine::channel>::count() == 0u &&
                      DmaChannel<1, RxEngine::channel>::count() == 0u && CsPin::is_output() && CsPin::read());
    bench.verdict("USART1 given back: its gate closed, PA9 and PA10 not outputs",
                  !Rcc::apb2_clock(usart_bus_clock(1).mask) && !ul::U1Tx::is_output() && !ul::U1Rx::is_output() &&
                      !link_live);
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
    // 460800 and a window of 5 lines: the file header's arithmetic.
    constexpr uint32_t rate = 460'800u;
    constexpr uint8_t window = 5;
    static_assert(window * 71u + 70u < 1024u / 2u, "the window stops the writer inside a half lap");
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

bool clock_ok = false;
bool tick_ok = false;
bool host_ok = false;

void console_banner() {
    print(serial, crlf, "boot: clk=", clock_ok ? "PLL64" : "FAILED", " tick=", tick_ok ? "SysTick" : "FAILED",
          " ruler=TIM2 spi1=", host_ok ? "host" : "FAILED", crlf);
    print(serial, "test_stm32_rubato_link - the SPI and UART stacks under Rubato against the STM32F446RE "
                  "(z: spi_peer, y: usart_peer)",
          crlf);
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void BRIO_STM32G0_USART2_HANDLER() {
    if (Serial::isr()) {
        brio::post<Console>(ConsoleEvent{console_rx});
    }
}

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

extern "C" [[gnu::naked]] void PendSV_Handler() { brio::scheduler_exception<K>(); }
extern "C" [[gnu::naked]] void SVC_Handler() { brio::scheduler_return(); }

/// The pressure: TIM14's update, where it landed, and the post.
extern "C" void TIM14_IRQHandler() {
    Pulse::clear_flags(Pulse::update_flag);
    pressure_landed();
}

extern "C" void SPI1_IRQHandler() {
    if (Host::isr()) {
        done_from_dma = false;
        brio::post<SpiArb>(brio::TransferDone{Host::status()});
    }
}

/// Channel 1, the transmit engine: armed for its errors alone.
extern "C" void DMA1_Channel1_IRQHandler() {
    if (Host::dma_isr()) {
        done_from_dma = true;
        brio::post<SpiArb>(brio::TransferDone{Host::status()});
    }
}

/// Channels 2 and 3 share this line (RM0444 table 61): the SPI's receive
/// engine and the link's receive ring. Each body answers for its own
/// channel, so both run on every entry.
extern "C" void DMA1_Channel2_3_IRQHandler() {
    if (Host::dma_isr()) {
        done_from_dma = true;
        brio::post<SpiArb>(brio::TransferDone{Host::status()});
    }
    if (link_live && LinkPort::dma_isr()) {
        brio::post<LinePort>(brio::RxActivity{});
    }
}

extern "C" void USART1_IRQHandler() {
    if (link_live && LinkPort::isr()) {
        brio::post<LinePort>(brio::RxActivity{});
    }
}

int main() {
    clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    tick_ok = brio::Ticker::init(clock);
    Clock32::init();
    (void)Clock32::configure(brio::TimConfig{.prescaler = 0, .period = 0xFFFF'FFFFu});
    Clock32::enable(true);
    CsPin::output(true);
    host_ok = Host::init(clock, Host::Config{.mosi_speed = brio::PinSpeed::high});
    Pulse::init();
    (void)Pulse::configure(brio::TimConfig{.prescaler = 0, .period = 0xFFFFu});
    brio::Nvic::enable(Pulse::irq());
    brio::scheduler_init();

    bench.letter('n', "the peer's command channel through the arbiter", tn_peer_link);
    bench.letter('o', "the matrix on the DMA engines: modes, widths, bit orders", to_matrix);
    bench.letter('p', "the ladders: the polled loop both shapes, the pump, the engines, every code", tp_ladders);
    bench.letter('w', "the arbiter under pressure: three requesters, one bus", tw_arbiter);
    bench.letter('i', "dark: every object idle, nothing lit", ti_dark);
    bench.letter('x', "the gap statistics: which shape a long gap between frames breaks, no verdict", tx_gaps,
                 false);
    bench.second_letter('t', "the UART received: numbered lines through the peer's echo", tt_lines);
    bench.second_letter('u', "the UART sent: a blast of numbered lines into the peer's sink", tu_blast);
    bench.second_letter('v', "the loan under preemption: a slow borrower at 460800", tv_loan);

    (void)serial_ok;
    brio::enable_interrupts();
    K::run();
}
