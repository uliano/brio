// test_rp2350_rubato_i2c - the bench suite of the RP2350 stratum's I2C stack
// under Rubato, the preemptive kernel (kernel/rubato.hpp), ONE SOURCE FOR
// BOTH PROCESSOR PAIRS: util/i2c_bus.hpp's arbiter over rp2350/i2c.hpp's
// I2cHost<0> and its DMA engines, run on the WeAct RP2350B - its Cortex-M33
// half and its Hazard3 half, one image each - against the desk partner, the
// WeAct RP2040, and its unchanged instrument (rp2040/src/apps/twi_peer.cpp)
// while an object of the highest priority preempts it at moments the traffic
// does not choose. Every verdict is the traffic judged byte-exact and every
// status the one the wire produced, as the Tenuto suite's peer letters
// (test_rp2350_i2c) judge them; nothing here changes a driver or a service.
//
// A test_<target>_<subject> suite in the bench grammar (util/testbench.hpp)
// that runs inside the kernel it tests: the program is one Rubato pack of
// five objects - Pressure, Client, I2cArb, Minor and Console, from the
// highest priority down. What names no register is the I2C suites' shared
// body (avrxt/src/apps/rubato_i2c.hpp); what the two Raspberry Pi chips
// share over the DW_apb_i2c and this peer is
// rp2040/src/apps/rp_rubato_i2c.hpp, whose header says where a preemption
// reaches a tenure on this block and how far. What is this chip's:
//
//   THE HALVES apart only where rp2350/core.hpp tells them apart: the
//   scheduler's vectors (BRIO_RP2350_SCHEDULER_VECTORS) and the kernel's
//   timebase behind the one name isr_systick (rp2350/ticker.hpp).
//   THE CLOCK. The PLL at 150 MHz on the crystal, static: ic_clk is clk_sys.
//   THE HOST. I2cHost<0> on GP37 (SCL) and GP36 (SDA) with its engines on
//   DMA channels 4 (the command words) and 5 (the bytes), both on DMA_IRQ_0
//   (test_rp2350_i2c's engined host).
//   THE PARTNER'S RAIL FIRST (pair_power.hpp): an RP2040 pad is not
//   failsafe, and the host's pads carry this chip's own pull-ups, so the
//   host is NOT brought up at boot. Every pass's bring-up (the shared body's
//   I2cTraits::bring_up) reads the RP2040's rail on GP36/GP37 first and
//   brings the host up only when it reads up - at each pass's opening,
//   which falls between two of letter h's holds and never inside one, the
//   sense taking both pads from the block for its 70 us - and every letter
//   ends by giving the host back: the block in reset, both pads isolated at
//   their reset state.
//   THE PRESSURE. TIMER1's alarm 0 with the timer's source on clk_sys
//   (rp2350/timer.hpp, datasheet 12.8.1.1), re-armed in its handler at
//   absolute counts: 97 / 193 / 389 us are 14550 / 28950 / 58350 counts. Its
//   dispatch, a 3000-cycle spin and a road-2 post of 303 on the Cortex-M33
//   and 323 on Hazard3 (docs/rp2350/platform.md), is shorter than the
//   shortest period; at 97 us x 3000 it takes 21 % of the core.
//   THE RULER. TIMER1 itself: a count a clk_sys cycle, one load through the
//   APB bridge on both architectures, a compiler barrier on either side.
//   The pressure's count is exact. No core-1 letters: the extra pass with
//   Pressure fed by core 1 is the RP2040's (its suite).
//
// THE WIRES, fixed on the desk (docs/boards/weact-rp2350b.md, wires 5 and
// 6): GP36 (SDA) to the RP2040's GP2 and GP37 (SCL) to its GP3, each line
// pulled up to the RP2040's 3V3. The RP2040 runs `twi_peer`. NOTHING LIT: the
// LED's pad GP25 is never driven (letter i checks it).
//
// The letters (z: n o p q w h i; x and s outside it), each of n o p q w h
// run as seven passes (the shared body's control and six pressures):
//   n  the command channel through the arbiter at 100 kHz: a ping, ident
//      (the sanity byte and the RP2040's kind, 0x08), ten pings.
//   o  the tenure shapes at 400 kHz on both of the host's paths.
//   p  100 kHz, 400 kHz and 1 MHz; the peer's commanded stretch on a read
//      of 8 at 100 kHz (eight holds).
//   q  the vocabulary (rp_rubato_i2c.hpp's version).
//   w  three requesters of three priorities, the close sixteen bytes long
//      (rp_rubato_i2c.hpp's version).
//   h  the held wire, with Pressure's probes at every tick.
//   i  dark: every object idle, the arbiter's vote, no time event armed,
//      TIMER1's alarm 0 disarmed, the host given back (the block in reset,
//      both pads at no function), both lines high through the partner's
//      pull-ups, GP25 not an output.
//   x  (outside z) the shared letter x: 400 writes of 16, the landings
//      inside start() counted.
//   s  (outside z) start()'s fill measured, and the stretch by its
//      arithmetic at 400 kHz and 1 MHz.
//
// build: boards = weact2350b,weact2350b-rv
// build: monitor_speed = 115200

#include <stdint.h>

#include <atomic>
#include <iterator>
#include <variant>

#include "kernel/borrowed.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/post.hpp"
#include "kernel/rubato.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/dma.hpp"
#include "rp2350/flash.hpp"
#include "rp2350/i2c.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/schedule.hpp"
#include "rp2350/ticker.hpp"
#include "rp2350/timer.hpp"
#include "rp2350/uart.hpp"
#include "util/i2c_bus.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

// THE PROTOCOL HEADERS ARE SHARED, NOT COPIED: the far board's instrument
// compiles twi_link.hpp too, and the controller's end of it is one reviewed
// file.
#include "../../../avrxt/src/apps/twi_link_client.hpp"

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

// ---- the console -------------------------------------------------------------------

constexpr UartPins console_pins{.tx = {0, PinFunction::uart}, .rx = {1, PinFunction::uart}};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;
TestBench<Serial, 16> bench;

// ---- the I2C host --------------------------------------------------------------------

/// I2C0 on the pair's wires: GP37 (SCL), GP36 (SDA).
constexpr I2cPins link_pins{.scl = 37, .sda = 36};
using TxEngine = DmaTxEngine<4, uint16_t>;
using RxEngine = DmaRxEngine<5>;
using Host = I2cHost<0, link_pins, TxEngine, RxEngine>;
using SdaPin = Pin<36>;
using SclPin = Pin<37>;

constexpr uint8_t peer_kind = twilink::peer_rp2040;
constexpr const char* pressure_name = "TIMER1's alarm 0";
constexpr const char* peer_name = "the WeAct RP2040";

/// The core-1 letters are the RP2040's (this file's header).
constexpr bool core1_pass = false;
volatile bool pressure_core1 = false;

/// The host brought up per pass behind the partner's rail and given back
/// at every letter's end (this file's header).
constexpr bool host_kept = false;
bool host_up = false;

bool rp_bring_up() {
    if (!pair::partner_rail(clock).up()) {
        print(serial, "  THE PARTNER IS UNPOWERED: GP36/GP37 read low through their pull-ups to the RP2040's 3V3 - "
                      "the host is not brought up (an RP2040 pad is not failsafe)",
              crlf);
        host_up = false;
        return false;
    }
    host_up = Host::init(clock);
    return host_up;
}

void rp_letter_end() {
    if (host_up) {
        Host::release();
        host_up = false;
    }
}

// THE BODY THE TWO RASPBERRY PI CHIPS SHARE, which includes the one every
// family shares.
#include "../../../rp2040/src/apps/rp_rubato_i2c.hpp"

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

/// Letter i's binding half: the alarm silent, the host given back, both
/// lines high through the partner's pull-ups (the rail's sense, which
/// drives nothing), the LED's pad not an output.
const char* binding_dark() {
    if (pressure_running || Edge::armed<0>() || Edge::raised<0>() || Irq::pending(Edge::irq<0>())) {
        return "TIMER1's alarm 0 armed, its flag up or its line pending";
    }
    if (host_up || Host::Resource::released()) {
        return "the host not given back: its block out of reset between letters";
    }
    if (SdaPin::function() != PinFunction::none || SclPin::function() != PinFunction::none) {
        return "a link pad still at a function between letters";
    }
    const pair::PartnerRail rail = pair::partner_rail(clock);
    if (!rail.sda || !rail.scl) {
        return "a line reads low through its pull-up to the RP2040's 3V3";
    }
    if (Pin<25>::is_output()) {
        return "the LED's pad GP25 is an output";
    }
    return nullptr;
}

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
          " ruler=", ruler_ok ? "TIMER1 on clk_sys" : "FAILED", " dma=", dma_ok ? "released" : "FAILED",
          " i2c0=behind the rail", crlf);
    print(serial, "test_rp2350_rubato_i2c (", hazard3 ? "Hazard3" : "Cortex-M33",
          ") - the I2C stack under Rubato against the WeAct RP2040 (twi_peer)", crlf);
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
/// pass runs, when and where it landed, and the post.
extern "C" void isr_timer1_0() {
    Edge::clear<0>();
    if (pressure_running) {
        pressure_next = pressure_next + pressure_period;
        Edge::alarm<0>(pressure_next);
    }
    rp_landing_stamp();
    pressure_landed();
}

/// Every tenure's end, STOP_DET or TX_ABRT: the host's one completion.
extern "C" void isr_i2c0() {
    if (Host::isr()) {
        done_from_dma = false;
        brio::post<I2cArb>(brio::TransferDone{Host::status()});
    }
}

/// DMA_IRQ_0, the engines' line: the transmit block's end hands the pump
/// the last command, and a channel's bus error ends the tenure.
extern "C" void isr_dma_0() {
    if (Host::dma_isr()) {
        done_from_dma = true;
        brio::post<I2cArb>(brio::TransferDone{Host::status()});
    }
}

int main() {
    clock_ok = SysClock::init();
    dma_ok = brio::Dma::init();
    const bool serial_ok = Serial::init(clock, 115200);
    tick_ok = brio::Ticker::init(clock);
    ruler_ok = Edge::init(clock);
    Edge::source(brio::TimerSource::sysclk);
    brio::Irq::enable(Edge::irq<0>());
    brio::scheduler_init();

    // Every letter's pass brings the host up behind the partner's rail
    // (I2cTraits::bring_up) and every letter gives it back (rp_letter<>).
    bench.letter('n', "the command channel through the arbiter", rp_letter<tn_command_channel>);
    bench.letter('o', "the tenure shapes on both of the host's paths", rp_letter<to_shapes>);
    bench.letter('p', "the speeds, and the peer's commanded stretch", rp_letter<tp_speeds>);
    bench.letter('q', "the vocabulary under pressure: deaf, a refused byte, a refused last byte",
                 rp_letter<tq_vocabulary_rp>);
    bench.letter('w', "the arbiter with three requesters of three priorities", rp_letter<tw_arbiter_rp>);
    bench.letter('h', "the held wire, with requests arriving while it is held", rp_letter<th_held_wire>);
    bench.letter('i', "dark: every object idle, the host given back, nothing lit", rp_letter<ti_dark>);
    bench.letter('x', "the landings inside start() counted: 400 writes silent and pressed", rp_letter<tx_fill>,
                 false);
    bench.letter('s', "start()'s fill measured, and the stretch by its arithmetic", rp_letter<ts_fill>, false);

    (void)serial_ok;
    brio::enable_interrupts();
    K::run();
}
