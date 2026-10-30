// test_rp2040_rubato_i2c - the bench suite of the RP2040 stratum's I2C stack
// under Rubato, the preemptive kernel (kernel/rubato.hpp): util/i2c_bus.hpp's
// arbiter over rp2040/i2c.hpp's I2cHost<1> and its DMA engines, run on the
// WeAct RP2040 against the desk partner, the WeAct RP2350B, and its unchanged
// instrument (rp2350/src/apps/twi_peer.cpp, on its Cortex-M33 half) while an
// object of the highest priority preempts it at moments the traffic does not
// choose. Every verdict is the traffic judged byte-exact and every status the
// one the wire produced, as the Tenuto suite's peer letters (test_rp2040_i2c)
// judge them; nothing here changes a driver or a service.
//
// A test_<target>_<subject> suite in the bench grammar (util/testbench.hpp)
// that runs inside the kernel it tests: core 0's program is one Rubato pack
// of five objects - Pressure, Client, I2cArb, Minor and Console, from the
// highest priority down. What names no register is the I2C suites' shared
// body (avrxt/src/apps/rubato_i2c.hpp: the pack, the passes, the command
// channel, the letters); what the two Raspberry Pi chips share over the
// DW_apb_i2c and this peer - the shapes, the in-tenure stretch and its
// instrument, letters q, w and s - is rp_rubato_i2c.hpp beside this file,
// whose header says where a preemption reaches a tenure on this block and
// how far. What is this chip's:
//
//   THE CLOCK. The PLL at 125 MHz on the crystal, static: ic_clk is clk_sys,
//   and the timing table the host solves at init() never moves.
//   THE HOST. I2cHost<1> on GP3 (SCL) and GP2 (SDA) with its engines on DMA
//   channels 4 (the command words) and 5 (the bytes), both on DMA_IRQ_0
//   (test_rp2040_i2c's engined host), brought up once at boot and kept: the
//   pull-ups are this board's own rail, and the partner's pads are failsafe.
//   THE PRESSURE. PWM slice 7's wrap interrupt, free-running at divider 1,
//   no pad handed over (the slice's GP14/GP15 carry nothing here): 97 / 193
//   / 389 us are TOP + 1 = 12125 / 24125 / 48625 counts. Its dispatch, a
//   3000-cycle spin and a road-2 post of 465 (docs/rp2040/platform.md), is
//   shorter than the shortest period; at 97 us x 3000 it takes 25 % of the
//   core.
//   THE RULER. Core 0's SysTick composed with its tick count
//   (CoreTicker<0>::cycles(), cortexm/ticker.hpp): this core has no DWT.
//   The slice and the ruler count one clock, so the pressure's count is
//   exact.
//   CORE 1'S LETTERS (c and e): letters o and x with every pressure pass fed
//   by core 1 - a Tenuto program of one object ticked by alarm 1 of the
//   system timer, sending each Tick through the inbox's doorbell
//   (core1_bell.hpp) - so that core 1's flash fetches share the XIP cache
//   start() runs from.
//
// THE WIRES, fixed on the desk (docs/boards/weact-rp2040.md, wires 5 and 6):
// GP2 (SDA) to the RP2350's GP36 and GP3 (SCL) to its GP37, each line pulled
// up to this board's 3V3. The RP2350B runs `twi_peer`. NOTHING LIT: the LED's
// pad GP25 is never driven (letter i checks it).
//
// The letters (z: n o p q w h c i; x, e and s outside it), each of n o p q w
// h run as seven passes (the shared body's control and six pressures):
//   n  the command channel through the arbiter at 100 kHz: a ping, ident
//      (the sanity byte and the RP2350's kind, 0x09), ten pings.
//   o  the tenure shapes at 400 kHz on both of the host's paths, one counted
//      serve a pass, every read byte-exact, the peer's tallies exact.
//   p  100 kHz, 400 kHz and 1 MHz: a read of 16, a 1+4, a write of 16; the
//      peer's commanded stretch on a read of 8 at 100 kHz (eight holds).
//   q  the vocabulary: deaf, the third of six refused, the last written byte
//      of a 2+4 refused (rp_rubato_i2c.hpp's version).
//   w  three requesters of three priorities at 400 kHz, the close sixteen
//      bytes long (rp_rubato_i2c.hpp's version).
//   h  the held wire, with Pressure's probes at every tick.
//   c  letter o with every pressure pass fed by core 1.
//   i  dark: every object idle, the arbiter's vote, no time event armed, the
//      slice stopped, core 1 parked, the host idle with its mask at zero and
//      its DMA requests off, the engines' channels idle, both lines high,
//      GP25 not an output.
//   x  (outside z) the shared letter x: 400 writes of 16, the landings inside
//      start() counted; e the same fed by core 1.
//   s  (outside z) start()'s fill measured, and the stretch by its
//      arithmetic at 400 kHz and 1 MHz.
//
// build: boards = weact2040
// build: monitor_speed = 115200

#include <stdint.h>

#include <atomic>
#include <iterator>
#include <variant>

#include "kernel/borrowed.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/post.hpp"
#include "kernel/rubato.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "rp2040/clock.hpp"
#include "rp2040/dma.hpp"
#include "rp2040/flash.hpp"
#include "rp2040/i2c.hpp"
#include "rp2040/multicore.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/pwm.hpp"
#include "rp2040/schedule.hpp"
#include "rp2040/ticker.hpp"
#include "rp2040/timer.hpp"
#include "rp2040/uart.hpp"
#include "util/i2c_bus.hpp"
#include "util/inbox.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

// THE PROTOCOL HEADERS ARE SHARED, NOT COPIED: the far board's instrument
// compiles twi_link.hpp too, and the controller's end of it is one reviewed
// file.
#include "../../../avrxt/src/apps/twi_link_client.hpp"

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

// ---- the console -------------------------------------------------------------------

constexpr UartPins console_pins{.tx = {0, PinFunction::uart}, .rx = {1, PinFunction::uart}};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;
TestBench<Serial, 16> bench;

// ---- the I2C host --------------------------------------------------------------------

/// I2C1 on the pair's wires: GP3 (SCL), GP2 (SDA).
constexpr I2cPins link_pins{.scl = 3, .sda = 2};
using TxEngine = DmaTxEngine<4, uint16_t>;
using RxEngine = DmaRxEngine<5>;
using Host = I2cHost<1, link_pins, TxEngine, RxEngine>;
using SdaPin = Pin<2>;
using SclPin = Pin<3>;

constexpr uint8_t peer_kind = twilink::peer_rp2350;
constexpr const char* pressure_name = "PWM slice 7";
constexpr const char* peer_name = "the WeAct RP2350B";

/// Letters c and e run o and x with Pressure fed by core 1.
constexpr bool core1_pass = true;
volatile bool pressure_core1 = false;

/// The host is brought up at boot and kept (the file header).
constexpr bool host_kept = true;
bool rp_bring_up() { return true; }
void rp_letter_end() {}

// THE BODY THE TWO RASPBERRY PI CHIPS SHARE, which includes the one every
// family shares.
#include "rp_rubato_i2c.hpp"

// THE SECOND CORE AS A PRESSURE SOURCE IS SHARED, NOT COPIED: the link suite
// under Rubato runs the same pass.
#include "core1_bell.hpp"

// =============================================================================
// The pressure: PWM slice 7's wrap, or core 1's sends
// =============================================================================

using PressSlice = PwmSlice<7>;
constexpr uint32_t press_per_us = SysClock::hz / 1'000'000u;   // divider 1: a count a clk_sys cycle
static_assert(389u * press_per_us <= 0x10000u, "the longest period fits the slice's 16 bits at divider 1");

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

const char* binding_dark() {
    if (PressSlice::enabled() || (Pwm::interrupts() & PressSlice::bit) != 0u || Nvic::pending(Pwm::irq())) {
        return "PWM slice 7 running, its interrupt on or its line pending";
    }
    if (!bell_silent()) {
        return "core 1 not parked, alarm 1 armed or the inbox not empty";
    }
    if (Pin<25>::is_output()) {
        return "the LED's pad GP25 is an output";
    }
    return nullptr;
}

// =============================================================================
// the Console's banner
// =============================================================================

bool clock_ok = false;
bool timer_ok = false;
bool tick_ok = false;
bool dma_ok = false;
bool host_ok = false;

void console_banner() {
    print(serial, crlf, "boot: clk=", clock_ok ? "PLL125" : "FAILED", " timer=", timer_ok ? "1us" : "FAILED",
          " tick=", tick_ok ? "SysTick" : "FAILED", " dma=", dma_ok ? "released" : "FAILED",
          " i2c1=", host_ok ? "host" : "FAILED", crlf);
    print(serial, "test_rp2040_rubato_i2c - the I2C stack under Rubato against the WeAct RP2350B (twi_peer)", crlf);
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

/// The pressure: slice 7's wrap, when and where it landed, and the post.
extern "C" void isr_pwm_wrap() {
    PressSlice::clear_pending();
    rp_landing_stamp();
    pressure_landed();
}

extern "C" void isr_timer_1() { bell_alarm_isr(); }

extern "C" void isr_sio_proc0() { bell_doorbell_isr<i2c_note_landing>(); }

/// Every tenure's end, STOP_DET or TX_ABRT: the host's one completion.
extern "C" void isr_i2c1() {
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
    timer_ok = brio::Timer::init(clock);
    dma_ok = brio::Dma::init();
    const bool serial_ok = Serial::init(clock, 115200);
    tick_ok = brio::CoreTicker<0>::init(clock);
    (void)brio::Pwm::reset();
    brio::Nvic::enable(brio::Pwm::irq());   // core 0's NVIC alone
    Drain0::enable();
    host_ok = Host::init(clock);
    brio::scheduler_init();

    bench.letter('n', "the command channel through the arbiter", rp_letter<tn_command_channel>);
    bench.letter('o', "the tenure shapes on both of the host's paths", rp_letter<to_shapes>);
    bench.letter('p', "the speeds, and the peer's commanded stretch", rp_letter<tp_speeds>);
    bench.letter('q', "the vocabulary under pressure: deaf, a refused byte, a refused last byte",
                 rp_letter<tq_vocabulary_rp>);
    bench.letter('w', "the arbiter with three requesters of three priorities", rp_letter<tw_arbiter_rp>);
    bench.letter('h', "the held wire, with requests arriving while it is held", rp_letter<th_held_wire>);
    bench.letter('c', "the tenure shapes with every pressure pass fed by core 1", rp_letter<tc_core1_shapes>);
    bench.letter('i', "dark: every object idle, core 1 parked, nothing lit", rp_letter<ti_dark>);
    bench.letter('x', "the landings inside start() counted: 400 writes silent and pressed", rp_letter<tx_fill>,
                 false);
    bench.letter('e', "letter x fed by core 1", rp_letter<te_core1_fill>, false);
    bench.letter('s', "start()'s fill measured, and the stretch by its arithmetic", rp_letter<ts_fill>, false);

    (void)serial_ok;
    brio::enable_interrupts();
    K::run();
}
