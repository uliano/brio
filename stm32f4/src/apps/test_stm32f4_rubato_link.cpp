// test_stm32f4_rubato_link - the bench suite of the SPI and UART stacks of
// the STM32F4 stratum under Rubato, the preemptive kernel
// (kernel/rubato.hpp): util/spi_bus.hpp's arbiter over stm32f4/spi.hpp's
// SpiHost and its DMA engines, and util/line_reader.hpp's LineReader over
// stm32f4/usart.hpp's Uart and its receive engine, run against the desk's
// STM32G0B1RE and its unchanged instruments (stm32g0/src/apps/spi_peer.cpp,
// usart_peer.cpp) while an object of the highest priority preempts them at
// moments the traffic does not choose. Every verdict is the traffic judged
// byte-exact, as the Tenuto suites (test_stm32f4_spi, test_stm32f4_serial)
// judge it; nothing here changes a driver or a service.
//
// A test_<target>_<subject> suite in the bench grammar (util/testbench.hpp)
// that runs inside the kernel it tests, as test_stm32f4_rubato does: the
// program is one Rubato pack of seven objects - Pressure, LineSink, Client,
// SpiArb, Minor, LinePort and Console, from the highest priority down -
// and everything of it that names no register, the pack, the passes, the
// numbered lines, the window client and the UART legs, is the link suites'
// shared body (avrxt/src/apps/rubato_link.hpp, whose header describes each
// object and the passes). What is this family's is here: the clock, the
// SpiHost on SPI2 with the DMA engines the F4 suite uses on the link (DMA1
// streams 4 and 3), the Uart on USART1 with its receive engine (DMA2
// stream 2 channel 4) and the interrupt transmitter beside it, the
// pressure on TIM2's update interrupt, the DWT ruler, the command channel
// of the UART key on USART1's polled resource, the letters and the vectors.
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
//      460800 and 921600 baud, assembled by the LineReader from the receive
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
#include "util/line_reader.hpp"
#include "util/print.hpp"
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
constexpr uint32_t core_hz = SysClock::hz;
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
/// The command channel's SCK: PCLK1/256 = 175.8 kHz, 45 us a character.
constexpr SpiClock link_clock = SpiClock::div256;
constexpr const char* bus_clock_name = "PCLK1";

// ---- the UART link -----------------------------------------------------------------

constexpr UartPins uart_link_pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
constexpr UartOptions uart_link_opts{.tx_speed = PinSpeed::very_high};
/// USART1_RX on DMA2 stream 2 channel 4 (RM0390 table 29): test_stm32f4_serial's
/// LinkDma, its ring and its transmit ring.
using LinkRxEngine = DmaRxEngine<2, 2, 4>;
using LinkPort = Uart<1, uart_link_pins, 2048, 64, NoDmaEngine, LinkRxEngine, uart_link_opts>;
/// The Uart task owns USART1's two vectors only while a leg runs.
volatile bool link_live = false;

/// What a leg's transport counted, read after the leg (init() clears them).
struct PortCounters {
    uint8_t ring_overruns = 0, ore = 0, fe = 0, pe = 0, ne = 0, dma_faults = 0;
    bool clean() const {
        return ring_overruns == 0u && ore == 0u && fe == 0u && pe == 0u && ne == 0u && dma_faults == 0u;
    }
};

// ---- the pressure's clock ----------------------------------------------------------

/// TIM2: 32 bits on APB1, its timer clock 2 x PCLK1 = 90 MHz - a timer no
/// stack under test touches.
using Pulse = Tim<2>;
constexpr uint32_t pulse_per_us = Pulse::clock_hz(SysClock{}) / 1'000'000u;
static_assert(pulse_per_us == 90u, "TIM2 counts at 90 MHz at this clock");
constexpr const char* pressure_name = "TIM2";
constexpr const char* peer_name = "the STM32G0B1RE";

[[gnu::always_inline]] inline void spin(uint32_t cycles) {
    const uint32_t t0 = Ruler::now();
    while (Ruler::now() - t0 < cycles) {
    }
}

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

uint32_t division(SpiClock c) { return 2u << static_cast<uint8_t>(c); }

/// The host counts its overruns itself (Host::overruns(), letter p).
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
    return (wide ? 16u : 8u) * division(c) * (SysClock::hz / SysClock::pclk1_hz);
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
    pressure_landed();
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