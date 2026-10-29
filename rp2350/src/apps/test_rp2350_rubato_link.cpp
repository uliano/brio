// test_rp2350_rubato_link - the bench suite of the SPI and UART stacks of
// the RP2350 stratum under Rubato, the preemptive kernel (kernel/rubato.hpp),
// ONE SOURCE FOR BOTH PROCESSOR PAIRS: util/spi_bus.hpp's arbiter over
// rp2350/spi.hpp's SpiHost<1> and its DMA engines, and util/line_reader.hpp's
// LineReader over rp2350/uart.hpp's Uart<1> and its interrupt receiver, run
// on the WeAct RP2350B - its Cortex-M33 half and its Hazard3 half, one image
// each - against the desk partner, the WeAct RP2040, and its unchanged
// instruments (rp2040/src/apps/spi_peer.cpp and uart_peer.cpp) while an
// object of the highest priority preempts them at moments the traffic does
// not choose. Every verdict is the traffic judged byte-exact; nothing here
// changes a driver or a service.
//
// A test_<target>_<subject> suite in the bench grammar (util/testbench.hpp)
// that runs inside the kernel it tests, as test_rp2350_rubato does: the
// program is one Rubato pack of seven objects - Pressure, LineSink, Client,
// SpiArb, Minor, LinePort and Console, from the highest priority down. What
// names no register is the link suites' shared body
// (avrxt/src/apps/rubato_link.hpp); what the two Raspberry Pi chips share
// over the PL022 and the PL011 is rp2040/src/apps/rp_rubato_link.hpp, whose
// header says what each path is, why the polled loop's verdict is strict
// here, and why the UART key is reduced. What is this chip's:
//
//   THE HALVES apart only where rp2350/core.hpp tells them apart: the
//   scheduler's vectors (BRIO_RP2350_SCHEDULER_VECTORS, PendSV and SVC on
//   the Cortex-M33, the machine software interrupt on Hazard3), and the
//   kernel's timebase behind the one name isr_systick (SysTick on one half,
//   the platform timer's comparator on the other, rp2350/ticker.hpp).
//   THE CLOCK. The PLL at 150 MHz on the crystal, static: clk_peri is
//   clk_sys, so a frame at divisor d is 8 or 16 x d core cycles.
//   THE SPI HOST. SpiHost<1> on GP30 (SCK), GP31 (TX, 47 Ohm on its way to
//   the partner), GP28 (RX), GP41 the select as a GPIO; half-word engines
//   on DMA channels 6 and 7 reporting on DMA_IRQ_1 (test_rp2350_spi's
//   LinkDmaHost); dma_min_frames 10 on this chip (rp2350/spi.hpp).
//   THE PRESSURE. TIMER1's alarm 0 with the timer's source on clk_sys
//   (rp2350/timer.hpp, datasheet 12.8.1.1), re-armed in its handler at
//   absolute counts - the last match plus the period - so the period does
//   not drift with the handler's entry: 97 / 193 / 389 us are 14550 / 28950
//   / 58350 counts. Its dispatch, a 3000-cycle spin and a road-2 post of
//   303 on the Cortex-M33 and 323 on Hazard3 (docs/rp2350/platform.md), is
//   shorter than the shortest period's 14550 cycles; at 97 us x 3000 it
//   takes 21 % of the core. The alternative, a PWM slice's wrap
//   (rp2350/pwm.hpp), is declined: here the ruler and the edge are one
//   counter. The kernel's timebase is neither.
//   THE RULER. TIMER1 itself: a count a clk_sys cycle, one load through the
//   APB bridge on both architectures, a compiler barrier on either side
//   (test_rp2350_rubato's). The pressure's count is exact.
//   THE PARTNER'S RAIL FIRST (pair_power.hpp): an RP2040 pad is not
//   failsafe, so every letter that drives a link pad - the SPI host's, the
//   select, UART1's transmitter, the mode line - is registered through the
//   guard, and boot drives none of them. No core-1 pass: the extra pass
//   with Pressure fed by core 1 is the RP2040's (its suite).
//
// THE WIRES, fixed on the desk (docs/boards/weact-rp2350b.md, the pair's ten
// and two grounds): SPI1 GP30 -> RP2040 GP10 (SCK), GP31 -> GP8 (this end's
// TX), GP28 <- GP11 (the peer's TX), GP41 -> GP29 (the select, 1.5 kOhm to
// the RP2040's 3V3); UART1 GP4 -> GP5, GP5 <- GP4; the mode line GP21 -> the
// peer's GP20; GP36/GP37, the I2C pair, the rail's sense. Key z wants the
// RP2040 running `spi_peer`, key y `uart_peer`. NOTHING LIT: the LED's pad
// GP25 is never driven (letter i checks it), and the peer's LED follows the
// mode line, which key y holds low.
//
// The letters (z: n o p w i; y: t v), each run as seven passes (the shared
// body's control and six pressures):
//   n  the peer's command channel through the arbiter: a ping, ident (the
//      sanity byte, the RP2040's kind 0x08, its crystal), ten more pings,
//      every frame at clk_peri / 128 = 1.17 MHz.
//   o  the matrix on the DMA engines (32 frames a window at clk_peri / 32 =
//      4.7 MHz, against the peer's DMA serve): the four modes in 8- and
//      16-bit frames, the client MSb and LSb first - the LSb rows an exact
//      two-way bit reversal against this MSb-only host - byte-exact at both
//      ends; the data windows' rate printed against the control's, with
//      the XIP cache's misses over the pass.
//   p  the ladders through the arbiter at every even divisor from 256 down
//      to the PIO client's ceiling, clk_peri / 16 = 9.4 MHz (derived in
//      rp_rubato_link.hpp), on six paths: the polled loop in 8-bit frames
//      (9) and in 16-bit frames off the engines (32 from an odd address),
//      the pump the same two ways, the engines (32 byte frames) and the
//      engines under a polled request (32 half-words); /14 and /12
//      printed. STRICT under pressure as in the control, the host's overrun
//      flag never set at a reply.
//   w  the arbiter under pressure: the Client above it (20 requests of 6
//      frames on the pump, two outstanding), Minor below it (10 POLLED
//      requests of 8, one outstanding) and, in a pressure pass, Pressure
//      itself (8 requests of 3 on the pump, from its tick), all at the
//      command rate inside ONE select window the letter holds, of constant
//      frames: every reply to its requester in its order, every one
//      spi_ok, nothing rejected, no timeout, the peer counting every frame.
//   i  dark: every object idle, TIMER1's alarm 0 disarmed and its line
//      quiet, no time event armed, SPI1 and UART1 given back, the mode line
//      released, GP25 not an output, the ceiling the Console's own.
//   t  the UART received, at 115200 (the peer's one rate): 200 numbered,
//      checked lines a leg written by the Console and turned around by the
//      peer's ECHO, assembled by the LineReader from the interrupt
//      receiver's ring and lent to LineSink: none lost, none out of order,
//      none corrupt, the port's own counters (ring overruns, OE, FE, PE, BE,
//      skips) zero.
//   v  the loan under preemption: t's leg with LineSink holding each line
//      sixteen character times and the lines streamed five ahead, so that
//      Pressure lands inside the borrower's dispatch and the port lends
//      while the next line arrives: every line exact, every line lent
//      alone, the two buffers strictly alternating (the arithmetic in
//      rp_rubato_link.hpp).
//
// build: boards = weact2350b,weact2350b-rv
// build: monitor_speed = 115200

#include <stdint.h>

#include <atomic>
#include <span>
#include <variant>

#include "kernel/borrowed.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/post.hpp"
#include "kernel/rubato.hpp"
#include "kernel/time_event.hpp"
#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/dma.hpp"
#include "rp2350/flash.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/schedule.hpp"
#include "rp2350/spi.hpp"
#include "rp2350/ticker.hpp"
#include "rp2350/timer.hpp"
#include "rp2350/uart.hpp"
#include "util/line_reader.hpp"
#include "util/print.hpp"
#include "util/spi_bus.hpp"
#include "util/testbench.hpp"

// THE PROTOCOL HEADERS ARE SHARED, NOT COPIED: the far board's two
// instruments compile the first; the second is the shared body's vocabulary.
#include "../../../avrxt/src/apps/spi_link.hpp"
#include "../../../avrxt/src/apps/usart_link.hpp"

#include "pair_power.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 150'000'000UL>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Rp2350RubatoPlatform<>;
using Edge = Timer<1>;
constexpr uint32_t core_hz = SysClock::hz;
constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000u;
static_assert(SysClock::hz % 1'000'000u == 0u, "a whole number of cycles per microsecond");

/// TIMER1 on clk_sys: one load, a count a cycle; a compiler barrier on
/// either side, so that no plain load or store is scheduled across a stamp.
struct Ruler {
    [[gnu::always_inline]] static uint32_t now() {
        std::atomic_signal_fence(std::memory_order_seq_cst);
        const uint32_t t = Edge::now_low();
        std::atomic_signal_fence(std::memory_order_seq_cst);
        return t;
    }
};

[[gnu::always_inline]] inline void spin(uint32_t cycles) {
    const uint32_t t0 = Ruler::now();
    while (Ruler::now() - t0 < cycles) {
    }
}

// ---- the console -------------------------------------------------------------------

constexpr UartPins console_pins{.tx = {0, PinFunction::uart}, .rx = {1, PinFunction::uart}};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;
TwoKeyTestBench<Serial, 'y'> bench;

// ---- the links ---------------------------------------------------------------------

constexpr SpiPins link_host_pins{.sck = 30, .tx = 31, .rx = 28};
using Host = SpiHost<1, link_host_pins, DmaTxEngine<6, uint16_t, 1>, DmaRxEngine<7, uint16_t, 1>>;
using CsPin = Pin<41>;
using SckPin = Pin<30>;
using TxPin = Pin<31>;
using ModeLine = Pin<21>;
constexpr UartPins uart_link_pins{.tx = {4, PinFunction::uart}, .rx = {5, PinFunction::uart}};

constexpr const char* pressure_name = "TIMER1's alarm 0";
constexpr const char* peer_name = "the WeAct RP2040";
constexpr uint8_t partner_kind = spilink::peer_rp2040;
constexpr uint32_t partner_sys_hz = 125'000'000;

/// The core-1 pass is the RP2040's (this file's header).
constexpr bool core1_pass = false;
volatile bool pressure_core1 = false;

// THE BODY THE TWO RASPBERRY PI CHIPS SHARE, which includes the one every
// family shares.
#include "../../../rp2040/src/apps/rp_rubato_link.hpp"

// =============================================================================
// The pressure: TIMER1's alarm 0, re-armed at absolute counts
// =============================================================================

volatile bool pressure_running = false;
volatile uint32_t pressure_next = 0;
uint32_t pressure_period = 0;

void pressure_arm(uint16_t period_us) {
    pressure_running = false;
    Edge::disarm<0>();
    Edge::clear<0>();
    pressure_period = static_cast<uint32_t>(period_us) * cycles_per_us;
    Irq::clear_pending(Edge::irq<0>());
    Edge::interrupt<0>(true);
}

void pressure_go() {
    pressure_next = Edge::now_low() + pressure_period;
    pressure_running = true;
    Edge::alarm<0>(pressure_next);
}

/// The flag first: a handler that runs after it re-arms nothing, one that
/// ran before it re-armed what the disarm then takes back.
void pressure_halt() {
    pressure_running = false;
    Edge::disarm<0>();
}

void pressure_quiet() {
    Edge::interrupt<0>(false);
    Edge::disarm<0>();
    Edge::clear<0>();
    Irq::clear_pending(Edge::irq<0>());
    spin(20'000u);   // the last tick's dispatch, if it was still running
}

bool pressure_silent() {
    return !pressure_running && !Edge::armed<0>() && !Edge::raised<0>() && !Irq::pending(Edge::irq<0>());
}
const char* pressure_silent_text() { return "TIMER1's alarm 0 disarmed, its flag low and its line quiet"; }

// =============================================================================
// the Console's banner
// =============================================================================

constexpr bool hazard3 = core_kind == CoreKind::hazard3;

bool clock_ok = false;
bool tick_ok = false;
bool ruler_ok = false;
bool dma_ok = false;

void console_banner() {
    print(serial, crlf, "boot: clk=", clock_ok ? "PLL150" : "FAILED", " tick=", tick_ok ? "on" : "FAILED",
          " ruler=", ruler_ok ? "TIMER1 on clk_sys" : "FAILED", " dma=", dma_ok ? "released" : "FAILED", crlf);
    print(serial, "test_rp2350_rubato_link (", hazard3 ? "Hazard3" : "Cortex-M33",
          ") - the SPI and UART stacks under Rubato against the WeAct RP2040 (z: spi_peer, y: uart_peer)", crlf);
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------
//
// ONE NAME, BOTH ARCHITECTURES, for every vector but the scheduler's, which
// the macro binds for the half the image is built for.
extern "C" void isr_uart0() {
    if (Serial::isr()) {
        brio::post<Console>(ConsoleEvent{console_rx});
    }
}

extern "C" void isr_systick() { brio::Ticker::tick(); }

BRIO_RP2350_SCHEDULER_VECTORS(K)

/// The pressure: TIMER1's alarm 0, re-armed from the last match while the
/// pass runs, where it landed, and the post.
extern "C" void isr_timer1_0() {
    Edge::clear<0>();
    if (pressure_running) {
        pressure_next = pressure_next + pressure_period;
        Edge::alarm<0>(pressure_next);
    }
    pressure_landed();
}

extern "C" void isr_spi1() {
    if (Host::isr()) {
        done_from_dma = false;
        brio::post<SpiArb>(brio::TransferDone{Host::status()});
    }
}

/// DMA_IRQ_1, the engines' line: the receive block's completion (the
/// transmit engine reports its errors alone).
extern "C" void isr_dma_1() {
    if (Host::dma_isr()) {
        done_from_dma = true;
        brio::post<SpiArb>(brio::TransferDone{Host::status()});
    }
}

extern "C" void isr_uart1() { link_rx_vector(); }

int main() {
    clock_ok = SysClock::init();
    dma_ok = brio::Dma::init();
    const bool serial_ok = Serial::init(clock, 115200);
    tick_ok = brio::Ticker::init(clock);
    ruler_ok = Edge::init(clock);
    Edge::source(brio::TimerSource::sysclk);
    brio::Irq::enable(Edge::irq<0>());
    brio::scheduler_init();

    // Every letter that drives a link pad behind the partner-rail guard.
    bench.letter('n', "the peer's command channel through the arbiter", pair::guarded<bench, clock, tn_peer_link>);
    bench.letter('o', "the matrix on the DMA engines: modes, widths, the client's bit orders",
                 pair::guarded<bench, clock, to_matrix>);
    bench.letter('p', "the ladders: the polled loop, the pump, the engines, every rung",
                 pair::guarded<bench, clock, tp_ladders>);
    bench.letter('w', "the arbiter under pressure: three requesters, one bus", pair::guarded<bench, clock, tw_arbiter>);
    bench.letter('i', "dark: every object idle, nothing lit", ti_dark);
    bench.second_letter('t', "the UART received: numbered lines through the peer's echo at 115200",
                        pair::guarded<bench, clock, tt_lines>);
    bench.second_letter('v', "the loan under preemption: a slow borrower at 115200", pair::guarded<bench, clock, tv_loan>);

    (void)serial_ok;
    brio::enable_interrupts();
    K::run();
}
