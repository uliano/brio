// test_rp2040_rubato_link - the bench suite of the SPI and UART stacks of
// the RP2040 stratum under Rubato, the preemptive kernel (kernel/rubato.hpp):
// util/spi_bus.hpp's arbiter over rp2040/spi.hpp's SpiHost<1> and its DMA
// engines, and util/line_reader.hpp's LineReader over rp2040/uart.hpp's
// Uart<1> and its interrupt receiver, run on the WeAct RP2040 against the
// desk partner, the WeAct RP2350B, and its unchanged instruments
// (rp2350/src/apps/spi_peer.cpp and uart_peer.cpp, on its Cortex-M33 half)
// while an object of the highest priority preempts them at moments the
// traffic does not choose. Every verdict is the traffic judged byte-exact;
// nothing here changes a driver or a service.
//
// A test_<target>_<subject> suite in the bench grammar (util/testbench.hpp)
// that runs inside the kernel it tests, as test_rp2040_rubato does: core 0's
// program is one Rubato pack of seven objects - Pressure, LineSink, Client,
// SpiArb, Minor, LinePort and Console, from the highest priority down. What
// names no register is the link suites' shared body
// (avrxt/src/apps/rubato_link.hpp: the pack, the passes, the numbered lines,
// the window client, the UART legs); what the two Raspberry Pi chips share
// over the PL022 and the PL011 - the host's paths, the ladders, the
// interrupt receiver, the reduced UART key against the mode-line peer - is
// rp_rubato_link.hpp beside this file, whose header says what each path is
// and why the polled loop's verdict is strict here. What is this chip's:
//
//   THE CLOCK. The PLL at 125 MHz on the crystal, static: clk_peri is
//   clk_sys, so a frame at divisor d is 8 or 16 x d core cycles.
//   THE SPI HOST. SpiHost<1> on GP10 (SCK), GP11 (TX), GP8 (RX), GP29 the
//   select as a GPIO; half-word engines on DMA channels 6 and 7 reporting
//   on DMA_IRQ_1 (test_rp2040_spi's LinkDmaHost); dma_min_frames 13 on this
//   chip (rp2040/spi.hpp).
//   THE PRESSURE. PWM slice 7's wrap interrupt (rp2040/pwm.hpp's
//   PwmPeriodicTick shape: free-running, divider 1, no pad handed over -
//   the slice's pads GP14/GP15 carry nothing of the link or the console),
//   periodic in clk_sys counts with nothing re-armed in the handler: 97 /
//   193 / 389 us are TOP + 1 = 12125 / 24125 / 48625 counts, inside its 16
//   bits. Declined: an alarm of the system timer re-armed each tick, a
//   microsecond's resolution and a register write a tick, where the wrap is
//   the edge the block offers. Its dispatch, a 3000-cycle spin and a road-2
//   post of 465 (docs/rp2040/platform.md), is shorter than the shortest
//   period's 12125 cycles; at 97 us x 3000 it takes 25 % of the core.
//   THE RULER. Core 0's SysTick composed with its tick count
//   (CoreTicker<0>::cycles(), cortexm/ticker.hpp): this core has no DWT and
//   this chip no 32-bit timer on clk_sys. The PWM and the ruler count one
//   clock, so the pressure's count is exact.
//   CORE 1'S PASS (letters o and t, after the six pressures): core 1
//   launched into a Tenuto program of one object, Bell, which alarm 1 of
//   the system timer posts to every 97 us - re-armed at absolute counts in
//   the alarm's handler, so the period does not drift - and whose dispatch
//   sends a Tick to Pressure on core 0 through its inbox (util/inbox.hpp):
//   each preemption on core 0 then arrives as road 1 from the doorbell's
//   handler, and core 1, fetching its program from the flash, shares the
//   XIP cache with the requests core 0 runs from it. The system timer's
//   microsecond and clk_sys come from one crystal, so 97 us is 12125
//   cycles and the pressure's count is judged as the others; Pressure
//   spins 3000. The doorbell's handler is the inbox drain written out
//   (Inboxes<Pressure>::isr()'s two steps), so that it knows how many
//   crossed and records where each landed.
//
// THE WIRES, fixed on the desk (docs/boards/weact-rp2040.md, the pair's ten
// and two grounds): SPI1 GP10 -> RP2350 GP30 (SCK), GP11 -> GP28 (this
// end's TX), GP8 <- GP31 (the peer's TX, 47 Ohm at its end), GP29 -> GP41
// (the select, 1.5 kOhm to this board's 3V3); UART1 GP4 -> GP5, GP5 <- GP4;
// the mode line GP21 -> the peer's GP20. Key z wants the RP2350B running
// `spi_peer`, key y `uart_peer`. NOTHING LIT: the LED's pad GP25 is never
// driven (letter i checks it), and the peer's LED follows the mode line,
// which key y holds low.
//
// The letters (z: n o p w i; y: t v), each run as seven passes (the shared
// body's control and six pressures) and o and t an eighth from core 1:
//   n  the peer's command channel through the arbiter: a ping, ident (the
//      sanity byte, the RP2350's kind 0x09, its crystal), ten more pings,
//      every frame at clk_peri / 128 = 977 kHz.
//   o  the matrix on the DMA engines (32 frames a window at clk_peri / 32 =
//      3.9 MHz, against the peer's DMA serve): the four modes in 8- and
//      16-bit frames, the client MSb and LSb first - the LSb rows an exact
//      two-way bit reversal against this MSb-only host - byte-exact at both
//      ends; the data windows' rate printed against the control's, with
//      the XIP cache's misses over the pass.
//   p  the ladders through the arbiter at every even divisor from 256 down
//      to the PIO client's ceiling, clk_peri / 12 = 10.4 MHz (derived in
//      rp_rubato_link.hpp), on six paths: the polled loop in 8-bit frames
//      (9) and in 16-bit frames off the engines (32 from an odd address),
//      the pump the same two ways, the engines (32 byte frames) and the
//      engines under a polled request (32 half-words); /10 and /8 printed.
//      STRICT under pressure as in the control, the host's overrun flag
//      never set at a reply.
//   w  the arbiter under pressure: the Client above it (20 requests of 6
//      frames on the pump, two outstanding), Minor below it (10 POLLED
//      requests of 8, one outstanding) and, in a pressure pass, Pressure
//      itself (8 requests of 3 on the pump, from its tick), all at the
//      command rate inside ONE select window the letter holds, of constant
//      frames: every reply to its requester in its order, every one
//      spi_ok, nothing rejected, no timeout, the peer counting every frame.
//   i  dark: every object idle, the PWM slice stopped and its line quiet,
//      core 1 parked and alarm 1 disarmed, no time event armed, SPI1 and
//      UART1 given back, the mode line released, GP25 not an output, the
//      ceiling the Console's own.
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
// build: boards = weact2040
// build: monitor_speed = 115200

#include <stdint.h>

#include <atomic>
#include <span>
#include <variant>

#include "kernel/borrowed.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/post.hpp"
#include "kernel/rubato.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time_event.hpp"
#include "rp2040/clock.hpp"
#include "rp2040/dma.hpp"
#include "rp2040/flash.hpp"
#include "rp2040/multicore.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/schedule.hpp"
#include "rp2040/pwm.hpp"
#include "rp2040/spi.hpp"
#include "rp2040/ticker.hpp"
#include "rp2040/timer.hpp"
#include "rp2040/uart.hpp"
#include "util/inbox.hpp"
#include "util/line_reader.hpp"
#include "util/print.hpp"
#include "util/spi_bus.hpp"
#include "util/testbench.hpp"

// THE PROTOCOL HEADERS ARE SHARED, NOT COPIED: the far board's two
// instruments compile the first; the second is the shared body's vocabulary.
#include "../../../avrxt/src/apps/spi_link.hpp"
#include "../../../avrxt/src/apps/usart_link.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 125'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Rp2040RubatoPlatform<0>;
constexpr uint32_t core_hz = SysClock::hz;
constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000u;
static_assert(SysClock::hz % 1'000'000u == 0u, "a whole number of cycles per microsecond");

/// Core 0's SysTick composed with its tick count: a count a cycle.
struct Ruler {
    [[gnu::always_inline]] static uint32_t now() { return CoreTicker<0>::cycles(); }
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

constexpr SpiPins link_host_pins{.sck = 10, .tx = 11, .rx = 8};
using Host = SpiHost<1, link_host_pins, DmaTxEngine<6, uint16_t, 1>, DmaRxEngine<7, uint16_t, 1>>;
using CsPin = Pin<29>;
using SckPin = Pin<10>;
using TxPin = Pin<11>;
using ModeLine = Pin<21>;
constexpr UartPins uart_link_pins{.tx = {4, PinFunction::uart}, .rx = {5, PinFunction::uart}};

constexpr const char* pressure_name = "PWM slice 7";
constexpr const char* peer_name = "the WeAct RP2350B";
constexpr uint8_t partner_kind = spilink::peer_rp2350;
constexpr uint32_t partner_sys_hz = 150'000'000;

/// Letters o and t run a pass with Pressure fed by core 1 (decision 6).
constexpr bool core1_pass = true;
volatile bool pressure_core1 = false;

// THE BODY THE TWO RASPBERRY PI CHIPS SHARE, which includes the one every
// family shares.
#include "rp_rubato_link.hpp"

// =============================================================================
// The pressure: PWM slice 7's wrap, or core 1's sends
// =============================================================================

using PressSlice = PwmSlice<7>;
constexpr uint32_t press_per_us = SysClock::hz / 1'000'000u;   // divider 1: a count a clk_sys cycle
static_assert(389u * press_per_us <= 0x10000u, "the longest period fits the slice's 16 bits at divider 1");

// ---- core 1: a Tenuto program of one object ------------------------------------------

// THE SECOND CORE AS A PRESSURE SOURCE IS SHARED, NOT COPIED: the RP2040's
// I2C suite under Rubato runs the same pass.
#include "core1_bell.hpp"

// ---- the shared body's four verbs -----------------------------------------------------

void pressure_arm(uint16_t period_us) {
    if (pressure_core1) {
        bell_arm(period_us);
        return;
    }
    PressSlice::enable(false);
    (void)PressSlice::configure({.mode = PwmDivMode::free_running,
                                 .divider = {},
                                 .top = static_cast<uint16_t>(static_cast<uint32_t>(period_us) * press_per_us - 1u)});
    PressSlice::counter(0);
    PressSlice::clear_pending();
    Nvic::clear_pending(Pwm::irq());
    PressSlice::interrupt(true);
}

void pressure_go() {
    if (pressure_core1) {
        bell_go();
        return;
    }
    PressSlice::enable(true);
}

void pressure_halt() {
    if (pressure_core1) {
        bell_halt();
        return;
    }
    PressSlice::enable(false);
}

void pressure_quiet() {
    if (pressure_core1) {
        bell_quiet();
        return;
    }
    PressSlice::interrupt(false);
    PressSlice::clear_pending();
    Nvic::clear_pending(Pwm::irq());
    spin(20'000u);   // the last tick's dispatch, if it was still running
}

bool pressure_silent() {
    return !PressSlice::enabled() && (Pwm::interrupts() & PressSlice::bit) == 0u && !Nvic::pending(Pwm::irq()) &&
           bell_silent();
}
const char* pressure_silent_text() {
    return "PWM slice 7 stopped, its interrupt off and its line quiet; core 1 parked, alarm 1 disarmed, the inbox "
           "empty";
}

// =============================================================================
// x - (OUTSIDE z, no verdict) where a pressure pass's XIP misses come from
// =============================================================================

/// The instrument behind letter o's miss counts: the XIP cache's misses
/// over 300 ms of the Console's own waits, then over the matrix's sixteen
/// exchanges (letter o's windows, 32 frames at clk_peri / 32), with the
/// pressure arranged seven ways - the slice ticking with its interrupt on
/// or off, Pressure spinning or not, core 1 idle in its Tenuto loop or
/// ticking the bell. No verdict: a table for the document. The counters are
/// cleared before each reading (they saturate at 32 bits).
struct XMode {
    const char* name;
    bool pwm;       ///< the slice counting at 97 us
    bool pwm_irq;   ///< and its wrap interrupt on
    bool core1;     ///< core 1 launched into its Tenuto program
    bool bell;      ///< and its alarm ticking the bell at 97 us
    uint16_t spin;  ///< Pressure's dispatch
};
const XMode xmodes[] = {
    {"nothing", false, false, false, false, 0},
    {"the slice at 97 us, Pressure spinning 3000", true, true, false, false, 3000},
    {"the slice at 97 us, its interrupt off", true, false, false, false, 0},
    {"the slice at 97 us, Pressure spinning 0", true, true, false, false, 0},
    {"core 1 idle in its Tenuto loop, nothing ticking", false, false, true, false, 0},
    {"core 1 idle, the slice at 97 us, Pressure spinning 3000", true, true, true, false, 3000},
    {"core 1's bell at 97 us, Pressure spinning 3000", false, false, true, true, 3000},
};

void x_on(const XMode& m) {
    pressure_spin = m.spin;
    if (m.core1) {
        core1_up = core1_launch();
    }
    if (m.pwm) {
        (void)PressSlice::configure(
            {.mode = PwmDivMode::free_running, .divider = {}, .top = static_cast<uint16_t>(97u * press_per_us - 1u)});
        PressSlice::counter(0);
        PressSlice::clear_pending();
        Nvic::clear_pending(Pwm::irq());
        PressSlice::interrupt(m.pwm_irq);
        PressSlice::enable(true);
    }
    if (m.bell && core1_up) {
        bell_period_us = 97;
        bell_next = Timer::now_low() + bell_period_us;
        bell_running = true;
        Timer::alarm(bell_alarm, bell_next);
    }
}

void x_off(const XMode& m) {
    bell_running = false;
    Timer::disarm(bell_alarm);
    PressSlice::enable(false);
    PressSlice::interrupt(false);
    PressSlice::clear_pending();
    Nvic::clear_pending(Pwm::irq());
    spin(20'000u);
    if (m.core1) {
        (void)core1_park();
        Timer::interrupt(bell_alarm, false);
        Timer::clear(bell_alarm);
        core1_up = false;
    }
    pressure_spin = 0;
}

void tx_xip() {
    if (!spi_up()) {
        print(serial, "  SPI1 did not come up", crlf);
        spi_down();
        return;
    }
    if (!sl::need_peer()) {
        spi_down();
        return;
    }
    for (const XMode& m : xmodes) {
        x_on(m);
        const uint32_t runs0 = Pressure::runs;
        Xip::reset_counters();
        wait_ms(300);
        const uint32_t idle_hits = Xip::hits();
        const uint32_t idle_acc = Xip::accesses();
        Xip::reset_counters();
        uint8_t good = 0;
        for (uint8_t k = 0; k < 16u; ++k) {
            sl::Exchange e{};
            e.cfg.mode = static_cast<uint8_t>(k & 3u);
            e.host_mode = static_cast<SpiMode>(k & 3u);
            e.cfg.dord = static_cast<uint8_t>((k >> 3) & 1u);
            e.wide = ((k >> 2) & 1u) != 0u;
            e.rate = matrix_rate;
            e.count = 32;
            e.seed_a = static_cast<uint8_t>(0x13u + 7u * k);
            e.seed_b = static_cast<uint8_t>(0x57u + 5u * k);
            if (sl::exact(e, sl::run(e))) {
                ++good;
            }
        }
        const uint32_t mx_hits = Xip::hits();
        const uint32_t mx_acc = Xip::accesses();
        const uint32_t runs = Pressure::runs - runs0;
        x_off(m);
        print(serial, "  ", m.name, ": 300 ms of waits ", idle_acc - idle_hits, " misses of ", idle_acc,
              " accesses; the matrix ", mx_acc - mx_hits, " misses of ", mx_acc, " (", good,
              " of 16 exact); Pressure ran ", runs, " times", crlf);
    }
    spi_down();
}

// =============================================================================
// the Console's banner
// =============================================================================

bool clock_ok = false;
bool timer_ok = false;
bool tick_ok = false;
bool dma_ok = false;

void console_banner() {
    print(serial, crlf, "boot: clk=", clock_ok ? "PLL125" : "FAILED", " timer=", timer_ok ? "1us" : "FAILED",
          " tick=", tick_ok ? "SysTick" : "FAILED", " dma=", dma_ok ? "released" : "FAILED", crlf);
    print(serial, "test_rp2040_rubato_link - the SPI and UART stacks under Rubato against the WeAct RP2350B "
                  "(z: spi_peer, y: uart_peer)",
          crlf);
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_uart0() {
    if (Serial::isr()) {
        brio::post<Console>(ConsoleEvent{console_rx});
    }
}

/// One SysTick vector, two cores: each ticks its own ticker.
extern "C" void isr_systick() {
    if (P::core_id() == 0u) {
        brio::CoreTicker<0>::tick();
    } else {
        brio::CoreTicker<1>::tick();
    }
}

/// Core 0's Rubato alone: the single PendSV body (core 1 under Tenuto never
/// pends PendSV, rp2040/schedule.hpp).
extern "C" [[gnu::naked]] void isr_pendsv() { brio::scheduler_exception<K>(); }
extern "C" [[gnu::naked]] void isr_svcall() { brio::scheduler_return(); }

/// The pressure: slice 7's wrap, where it landed, and the post.
extern "C" void isr_pwm_wrap() {
    PressSlice::clear_pending();
    pressure_landed();
}

/// Core 1's alarm: re-armed at absolute counts while the pass runs, the
/// post to its Bell.
extern "C" void isr_timer_1() { bell_alarm_isr(); }

/// Core 0's doorbell: Inboxes<Pressure>::isr() written out - the bells
/// popped, then the inbox drained into local posts - with each event that
/// crossed recorded where it landed.
extern "C" void isr_sio_proc0() { bell_doorbell_isr<note_landing>(); }

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
    timer_ok = brio::Timer::init(clock);
    dma_ok = brio::Dma::init();
    const bool serial_ok = Serial::init(clock, 115200);
    tick_ok = brio::CoreTicker<0>::init(clock);
    (void)brio::Pwm::reset();
    brio::Nvic::enable(brio::Pwm::irq());   // core 0's NVIC alone
    Drain0::enable();
    brio::scheduler_init();

    bench.letter('n', "the peer's command channel through the arbiter", tn_peer_link);
    bench.letter('o', "the matrix on the DMA engines: modes, widths, the client's bit orders", to_matrix);
    bench.letter('p', "the ladders: the polled loop, the pump, the engines, every rung", tp_ladders);
    bench.letter('w', "the arbiter under pressure: three requesters, one bus", tw_arbiter);
    bench.letter('i', "dark: every object idle, core 1 parked, nothing lit", ti_dark);
    bench.letter('x', "where a pressure pass's XIP misses come from, no verdict", tx_xip, false);
    bench.second_letter('t', "the UART received: numbered lines through the peer's echo at 115200", tt_lines);
    bench.second_letter('v', "the loan under preemption: a slow borrower at 115200", tv_loan);

    (void)serial_ok;
    brio::enable_interrupts();
    K::run();
}
