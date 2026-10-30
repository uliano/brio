// test_samc_rubato_i2c - the bench suite of the SAM C21 stratum's I2C stack
// under Rubato, the preemptive kernel (kernel/rubato.hpp): util/i2c_bus.hpp's
// arbiter over samc21/i2c.hpp's I2cHost on SERCOM3, run against the desk's
// AVR64EA48 and its unchanged instrument (avrxt/src/apps/twi_peer.cpp) while
// an object of the highest priority preempts it at moments the traffic does
// not choose. Every verdict is the traffic judged byte-exact and every
// status the one the wire produced, as the Tenuto suite's peer letters
// (test_samc_i2c) judge them; nothing here changes a driver or a service.
//
// A test_<target>_<subject> suite in the bench grammar (util/testbench.hpp)
// that runs inside the kernel it tests: the program is one Rubato pack of
// five objects - Pressure, Client, I2cArb, Minor and Console, from the
// highest priority down - and everything of it that names no register, the
// pack, the passes, the command channel and the letters, is the I2C suites'
// shared body (avrxt/src/apps/rubato_i2c.hpp, whose header describes each
// object, each letter and where preemption can reach a tenure). What is
// this family's is here, each choice from DS60001479M ch. 33 and the driver
// header:
//
//   THE HOST. I2cHost<3> on PA22 (PAD[0], SDA) and PA23 (PAD[1], SCL),
//   function C, its core clock generator 0 - OSC48M undivided, the CPU's
//   rate - and the node's 166 ns rise given to init() (test_samc_i2c's
//   figure: the formula of 33.6.2.4.1 subtracts the rise, so a budget longer
//   than the wire's runs SCL fast). Its engine, from the chapter
//   (samc21/i2c.hpp's class comment, docs/samc21/i2c.md): THE BYTE PUMP
//   ALONE, one interrupt a byte - MB after a byte or an address went out, SB
//   after one came in - each holding SCL until software answers, "unlimited
//   time to respond" (33.10.6); smart mode always on, so a pumped read byte
//   is one DATA load, and a read's last byte closed by the STOP with its
//   NACK first, then the DATA read (33.10.10); a write's repeated START into
//   its read phase at the write's last MB. NO ENGINE SLOT: both DMA requests
//   are declined over erratum 1.10.4 (docs/samc21/dmac.md) and the automatic
//   length with them, so letter o's table is the pump's alone and says so
//   shape by shape. start() copies the Request's fields, applies the speed
//   (a cached disable / enable pair and a forced IDLE when it moves) and
//   ends on the ADDR store behind a SYSOP wait counted in spins: nothing is
//   on the wire before that last store, so a preemption inside start() is a
//   later START - a gap between tenures - and inside a tenure the host holds
//   SCL while MB or SB waits for a late vector.
//   THE PREDICTION: strict at every speed and pass; letter x's landings
//   inside start() find the bus monitor not yet the owner (STATUS.BUSSTATE,
//   33.6.2.3), or the owner only in start()'s last instructions after the
//   ADDR store.
//   THE HELD WIRE. A START into a line the peer holds low from a pin PARKS
//   in hardware (33.6.2.4.2) - the bus reads BUSY for the hold's whole
//   length - and on this silicon the parked START does not fire when the
//   hold releases (docs/samc21/i2c.md, "A tenure into a busy bus parks in
//   hardware"), so the arbiter's 20 ms answers it, i2c_timeout, and its
//   recover() - the init() tail re-run from the cached configuration, the
//   bus state forced to IDLE - is the way out. A START issued after a
//   forced IDLE (a recover(), or a speed change, whose enable re-boots the
//   state machine) goes out into the held line and loses its address:
//   i2c_arb_lost - and with BUSERR beside the loss the engine answers
//   i2c_bus_error, "the protocol-violation flavour of the same loss"
//   (samc21/i2c.hpp's settle()): this family's third signature, measured
//   on a probe whose START met the peer's return to command mode just
//   after the release. Every one judged by the requirement.
//   THE PRESSURE. TC4's overflow in MATCH FREQUENCY mode (35.6.2.6.1: CC0 is
//   TOP, the counter wraps there and raises OVF), 16 bits on its own generic
//   clock channel from generator 0, prescaler 1: 97 / 193 / 389 us are 4656
//   / 9264 / 18672 counts, CC0 one less - the link suite's. Free of SERCOM3
//   (the bus under test), SERCOM5 (the console) and SysTick. Its tick's
//   dispatch, a 3000-cycle spin, and road 1's 251 (docs/samc21/platform.md)
//   are shorter than the shortest period's 4656 cycles, so no tick finds
//   the last one still queued; at 97 us x 3000 it takes 64 % of the core.
//   Letter h's flood adds probes whose whole cost is far more than their
//   post: a probe that loses its START at once into the held line is a
//   whole cycle, some 3500 cycles here, more than a 97 us period leaves
//   beside a 3000-cycle tick - which is why the shared body paces the
//   flood by the Console's turns. TC4 and SERCOM3 sit at the same NVIC priority,
//   so neither nests over the other; the pump's handler (some 180 cycles,
//   docs/samc21/i2c.md's cost table) cannot hold TC4 off past a period,
//   because the host holds SCL while it runs and the next byte's interrupt
//   is a byte time later at least (432 cycles at 1 MHz).
//   THE RULER. The kernel ticker's BasicTicker::cycles(), CPU cycles from
//   SysTick's position and the tick count (ARMv6-M has no cycle counter),
//   48 cycles a two-read (test_samc_rubato's letter d): TC4 and the ruler
//   count one clock, so judge_pressure()'s arithmetic is exact. OSC48M runs
//   0.60 to 0.64 % slow against the peer's quartz (docs/samc21/sercom.md),
//   which every bound below that compares the two clocks allows for.
//
// THE PEER, unchanged: twi_peer on the AVR64EA48, kind 0x01, its TWI0 on PA2
// (SDA) and PA3 (SCL) a POLLED client at 5 V on its 20 MHz quartz. It
// answers an address match at once and spends a commanded hold before
// every data byte it takes or gives - BEFORE the flag is cleared, where the
// TWI stretches SCL (twi_peer.cpp's run_serve) - and ends a counted serve
// on the STOP of the tenure that reached the count, never at the byte that
// reaches it. Its hold_sda pins SDA low from PORT for the action's deadline
// (`ms`; aux8 0, so no SCL edge releases it). No extended repertoire
// (host_read): the letter set is unaffected. Speeds: 100 kHz, 400 kHz and
// 1 MHz, what test_samc_i2c's letter f judges against it on the pad.
//
// THE WIRES, fixed on the desk (test_samc_i2c's): this board's PA22 (SDA)
// to the EA's PA2, PA23 (SCL) to its PA3, the EA's PC2 and PC3 (TWI0's
// dual-mode pads, inputs under twi_peer) on the same two nodes, 1.5 kOhm
// from each line to +5 V, a dedicated ground; both boards at 5 V. The link
// suite's four wires PA16..PA19 stay fitted and untouched: no letter here
// claims SERCOM1 or those pads.
// NOTHING LIT: no letter drives the LED's pad PB23.
//
// THE BOUNDS this family adds to the shared body's (each verdict's
// arithmetic is the body's, these are its inputs):
//   - the commanded stretch, 1000 us a byte at 100 kHz: the peer's holds
//     inside a WRITE of 8 are eight, one before each byte's acknowledge (the
//     address answered at once), and inside a READ of 8 eight, one before
//     each byte it gives. Its ninth hold, at the controller's closing NACK,
//     falls after the read's STOP and into the write that follows it at
//     once - whose address match the peer's TWI holds until the hold is
//     spent: measured, the read 8.9 ms and the write 9.7 to 10.0 - so a
//     lower bound counts eight for each. The AVR's
//     delay_us_runtime() waits the microseconds asked or longer
//     (avrxt/delay.hpp); 8 ms of the peer's quartz read as at most 0.64 %
//     less on this end's OSC48M, 51 us, under the nine byte times the wire
//     adds (810 us at 100 kHz) - so the bound is the holds alone.
//   - letter h's hold: 60 ms by the peer's quartz, at least 55 ms from the
//     ack's arrival here (the body's arithmetic), the 0.64 % a few hundred
//     microseconds of it.
//
// The letters (z: n o p q w h i; x and u outside it), each but i, x and u
// run as seven passes (the shared body's control and six pressures):
//   n  the command channel through the arbiter at 100 kHz: a ping, ident
//      (the sanity byte and the avrxt port's kind, 0x01), ten pings, every
//      tenure i2c_ok.
//   o  the tenure shapes at 400 kHz, every one on the pump (I2cTraits below
//      names each shape's handling), one counted serve a pass: every status
//      i2c_ok, every read byte-exact, the peer's count, sum and address
//      matches exact.
//   p  100 kHz, 400 kHz and 1 MHz: a read of 16, a 1+4 and a write of 16
//      byte-exact, the wire's time printed; the peer's commanded stretch,
//      1000 us a byte at 100 kHz, priced from below by its holds.
//   q  the vocabulary: a deaf client's i2c_nack_addr, the third of six
//      refused (i2c_nack_data), the last written byte of a 2+4 refused
//      (i2c_nack_data, the read half never begun), the next tenure i2c_ok
//      after each.
//   w  three requesters of three priorities at 400 kHz: the Client's
//      write-then-reads of 1+6, Minor's writes of two, Pressure's probes:
//      in order, i2c_ok, nothing rejected, no timeout, the peer's tallies
//      exact.
//   h  the held wire (the peer holds SDA 60 ms) with Pressure's flood of
//      probes, paced by the Console's turns: answered in its place, every
//      request once, the next tenure i2c_ok and the arbiter idle; then the
//      hold met at another speed (above); i2c_bus_error among this family's
//      signatures (I2cTraits::held_bus_error).
//   THE STACK is painted and read around every pass (print_host()), its
//   high-water printed with the pass: Rubato runs every dispatch on the one
//   main stack, a pack of five holds at most five runs of the scheduler
//   (kernel/rubato.hpp, "The stack"), and letter u bounds the high-water by
//   them under the heaviest load this suite makes. THE SERVES carry
//   flag_stop_interrupt (the shared body's serve()): this peer sees a STOP
//   only with it.
//   i  dark: every object idle, the arbiter's vote, no time event armed,
//      TC4 stopped and its line quiet, the host idle with ERROR alone armed,
//      no flag standing, its line enabled and not pending, the bus monitor
//      IDLE and both lines high on their pads.
//   x  (outside z) 400 writes of 16 silent and 400 under 97 us x 3000:
//      the landings inside start() counted, and of them those with the bus
//      monitor reading OWNER.
//   u  (outside z) letter h's held wire at 97 us x 3000 with the flood
//      UNPACED: for 1000 ticks TC4's handler marks the Console's turn
//      itself, so Pressure posts a probe at every tick. Every probe is at
//      least a post, the arbiter's dispatch and its reply's road 2 into
//      Pressure, more than a 97 us period leaves beside the 3000-cycle
//      tick, so the pack above the Console stays busy for the whole window
//      and the Console, the flood's pacer, does not turn (98 ms between two
//      of its turns, measured): the timeout of the first probe, parked on
//      the held line, is a time event its turns walk, so it does not
//      mature either, and the probes after it fill the pending FIFO
//      (i2c_rejected) and then the arbiter's queue, which drops them - the
//      load under which a scheduler that nested a run at every landing
//      between two dispatches would grow its stack without bound. The
//      stack is painted before the hold and read
//      after it: its high-water within the five runs' arithmetic
//      (unpaced_stack_bound's comment), the window whole, a guard at 16 KB
//      that would have ended it unreached. An overload drops what it cannot
//      queue, so the held wire's requirement is not letter h's: every probe
//      answered at most once, in the vocabulary, or counted among the posts
//      the arbiter's full queue dropped; and the overload over, the next
//      tenure i2c_ok and the arbiter idle.
//
// RUNNING IT: `brio flash <the SAM board> test_samc_rubato_i2c`, the peer
// on twi_peer; then `brio run <board> z`, and x or u by name with
// `brio run <board> u --expect=" pass, "`.
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include <iterator>
#include <variant>

#include "kernel/borrowed.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/post.hpp"
#include "kernel/rubato.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "samc21/clock.hpp"
#include "samc21/i2c.hpp"
#include "samc21/nvic.hpp"
#include "samc21/pin.hpp"
#include "samc21/platform.hpp"
#include "samc21/sercom.hpp"
#include "samc21/tc.hpp"
#include "samc21/ticker.hpp"
#include "util/i2c_bus.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

// THE PROTOCOL HEADERS ARE SHARED, NOT COPIED: the far board's instrument
// compiles twi_link.hpp too, and the controller's end of it is one reviewed
// file.
#include "../../../avrxt/src/apps/twi_link_client.hpp"

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
TestBench<Serial> bench;

// ---- the I2C host --------------------------------------------------------------------

/// SERCOM3's PAD[0] (SDA) and PAD[1] (SCL) on PA22 and PA23, function C:
/// both on table 6-7's I2C-capable list, the pair test_samc_i2c drives.
constexpr I2cPads host_pads{
    .sda_pin = {'A', 22, PinFunction::c},
    .scl_pin = {'A', 23, PinFunction::c},
};
static_assert(MUX_PA22C_SERCOM3_PAD0 == static_cast<uint8_t>(PinFunction::c));
static_assert(MUX_PA23C_SERCOM3_PAD1 == static_cast<uint8_t>(PinFunction::c));
using Host = I2cHost<3, host_pads>;
using Raw = Host::Resource;
using SdaPin = Pin<'A', 22>;
using SclPin = Pin<'A', 23>;

/// The node's rise, 1.5 kOhm to 5 V: test_samc_i2c's figure (the file
/// header).
constexpr uint32_t bus_rise_ns = 166;

constexpr uint8_t peer_kind = twilink::peer_avrxt;
/// The three speeds the pair's Tenuto suite judges on the pad (its letter
/// f): Sm, Fm and Fm+, every one producible on a 48 MHz core
/// (I2cTraits::bring_up() asks the host).
constexpr I2cSpeed speeds[] = {I2cSpeed::standard_100k, I2cSpeed::fast_400k, I2cSpeed::fast_plus_1m};
constexpr uint32_t speed_hz(I2cSpeed s) { return i2c_speed_hz(s); }

bool host_ok = false;

/// What the shared body changes for this host and this peer.
struct Shape {
    uint8_t tx_len;
    uint8_t rx_len;
    const char* path;
};
struct I2cTraits {
    /// Letter o's table: the body's default shapes, every one on the pump -
    /// this host has no engine (erratum 1.10.4) - each with what the pump
    /// does for it. Ends on a plain write (the shared body's counted serve).
    static constexpr Shape shapes[] = {
        {0, 0, "pump: the probe, the address with W, one MB and the STOP"},
        {0, 1, "pump: one SB, the STOP with its NACK first, then the DATA read (33.10.10)"},
        {0, 2, "pump: SB a byte, smart mode's DATA read, the last closed by STOP + NACK"},
        {0, 3, "pump: the same"},
        {0, 8, "pump: the same"},
        {0, 32, "pump: the same"},
        {1, 1, "pump: one MB, the repeated START at the write's last MB, then one SB"},
        {1, 2, "pump: one MB, the repeated START, SB a byte"},
        {2, 8, "pump: MB a byte, the repeated START, SB a byte"},
        {1, 0, "pump: MB at the address and at the byte, the STOP at the last"},
        {2, 0, "pump: MB a byte, the STOP at the last"},
        {3, 0, "pump: the same"},
        {8, 0, "pump: the same"},
        {32, 0, "pump: the same"},
    };
    /// The AVR64EA48 peer's holds inside the stretched tenures (its
    /// run_serve, the file header's arithmetic): one before each data byte
    /// it takes or gives, none at the address match. A write of 8: eight
    /// inside. A read of 8: the eight gives inside, the hold at the closing
    /// NACK uncounted.
    static constexpr uint8_t stretch_write_holds = 8;
    static constexpr uint8_t stretch_read_holds = 8;
    /// A START into the held line after a forced IDLE loses its address,
    /// and with BUSERR beside the loss the engine answers i2c_bus_error,
    /// "the protocol-violation flavour of the same loss" (samc21/i2c.hpp's
    /// settle()): letter h counts it among the held wire's signatures.
    static constexpr bool held_bus_error = true;
    /// Before the wire: the host came up, and every speed the letters name
    /// is producible on this core (speed_ok(): a speed it cannot make is a
    /// request refused i2c_rejected, which no letter expects).
    static bool bring_up() {
        if (!host_ok) {
            return false;
        }
        for (const I2cSpeed s : speeds) {
            if (!Host::speed_ok(s)) {
                return false;
            }
        }
        return true;
    }
};

// ---- the pressure's clock ----------------------------------------------------------

using Pulse = Tc<4>;
constexpr uint32_t pulse_per_us = SysClock::hz / 1'000'000u;   // generator 0, prescaler 1
static_assert(389u * pulse_per_us - 1u <= 0xFFFFu, "the longest period fits TC4's 16 bits");
constexpr const char* pressure_name = "TC4";
constexpr const char* peer_name = "the AVR64EA48";

// THE TARGET-FREE HALF IS SHARED, NOT COPIED: every family's I2C suite
// compiles the same body around its own host.
#include "../../../avrxt/src/apps/rubato_i2c.hpp"

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

/// STATUS.BUSSTATE reads OWNER: this host issued the START the bus monitor
/// saw and no STOP since (33.6.2.3).
bool host_on_wire() { return Raw::bus_state() == I2cBusState::owner; }

const char* host_dark() {
    if (Pulse::enabled() || (Pulse::armed() & Pulse::overflow_flag) != 0u || Nvic::pending(Pulse::irq())) {
        return "TC4 running, its overflow interrupt armed or its line pending";
    }
    if (!Host::idle()) {
        return "the host's engine is not between tenures";
    }
    // Every exit of the engine leaves ERROR alone armed (finish(), recover(),
    // init()). The resource has no reading of INTENSET (I2cm<n> has none, the
    // report proposes one), so the register is read here, the one suite read.
    if (Raw::regs().SERCOM_INTENSET != I2cmFlag::error) {
        return "the host's interrupt enables are not at rest (ERROR alone)";
    }
    if ((Raw::flags() & I2cmFlag::all) != 0u) {
        return "an MB, SB or ERROR flag stands";
    }
    if (!Nvic::enabled(Raw::irq()) || Nvic::pending(Raw::irq())) {
        return "SERCOM3's line is off or pending";
    }
    if (Raw::bus_state() != I2cBusState::idle) {
        return "the bus monitor does not read IDLE";
    }
    if (!SclPin::read() || !SdaPin::read()) {
        return "a line reads low on its pad";
    }
    return nullptr;
}

// ---- the stack's high-water, painted -------------------------------------------

/// THE STACK IS MEASURED IN EVERY PASS. Rubato runs every dispatch on the
/// one main stack (docs/design/kernel.md, "Rubato"), and on this core a
/// stack that outgrows the RAM above .bss writes into it with no fault to
/// say so - the variables the verdicts read among the first. So the free
/// RAM between the end of .bss and the stack pointer is painted with a
/// pattern before every pass and read after it: the lowest word the pass
/// changed is the pass's high-water, printed with the pass and kept as
/// the worst. The paint and the read run after pass_end() and outside the
/// pass's wall, from the Console's dispatch, below whose frame nothing is
/// live but an interrupt's transient one.
extern "C" char __bss_end;     // samc21/ld/samc21j18a.ld: the end of .bss
extern "C" char __stack_top;   // and the top of RAM, where the stack starts
constexpr uint32_t paint_word = 0xDEADBEEFu;
uint32_t stack_worst = 0;

uint32_t* paint_floor() {
    // A margin of 64 bytes above .bss: the first overflow is seen before
    // it reaches a variable.
    return reinterpret_cast<uint32_t*>((reinterpret_cast<uintptr_t>(&__bss_end) + 64u + 3u) & ~uintptr_t{3});
}

void paint_stack() {
    uintptr_t sp;
    asm volatile("mov %0, sp" : "=r"(sp));
    for (uint32_t* w = paint_floor(); reinterpret_cast<uintptr_t>(w) + 512u < sp; ++w) {
        *w = paint_word;
    }
}

/// The bytes of stack the last pass used, from the top of RAM.
uint32_t stack_used() {
    const uint32_t* w = paint_floor();
    while (reinterpret_cast<uintptr_t>(w) < reinterpret_cast<uintptr_t>(&__stack_top) && *w == paint_word) {
        ++w;
    }
    return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&__stack_top) - reinterpret_cast<uintptr_t>(w));
}

/// The SAM's engine counts nothing beside its statuses (no engine, no
/// erratum to count): what this suite adds is the pass's stack and the
/// bus monitor's state at the print. The stack is read and the RAM painted
/// again for the next pass.
void print_host() {
    const uint32_t used = stack_used();
    stack_worst = used > stack_worst ? used : stack_worst;
    print(serial, "; stack ", used, " bytes (the worst ", stack_worst, "); BUSSTATE ",
          static_cast<uint8_t>(Raw::bus_state()));
    paint_stack();
}

bool clock_ok = false;
bool tick_ok = false;
bool pulse_ok = false;

void console_banner() {
    print(serial, crlf, "boot: clk=", clock_ok ? "OSC48M" : "FAILED", " tick=", tick_ok ? "SysTick" : "FAILED",
          " pressure=", pulse_ok ? "TC4" : "FAILED", " ruler=SysTick cycles sercom3=", host_ok ? "host" : "FAILED",
          crlf);
    print(serial, "test_samc_rubato_i2c - the I2C stack under Rubato against the AVR64EA48 (twi_peer)", crlf);
    bench.menu();
}

// =============================================================================
// u - outside z: the held wire under the flood unpaced, the stack bounded
// =============================================================================

/// Letter u's window, counted in ticks of TC4 while letter h's flood is
/// live: at each one TC4's handler marks the Console's turn itself, so
/// Pressure posts a probe at every tick - the flood the shared body paces,
/// unpaced. 1000 ticks of 97 us are 97 ms: past the hold's 60 ms and the
/// Console's 70 ms wait for the release, when the core is overloaded the
/// Console's own turns cannot come at all.
constexpr uint16_t unpaced_window = 1000;
/// The window's guard: a tick that finds the main stack more than 16 KB
/// below the top of RAM ends the window, so that a kernel whose stack grows
/// with the load stops the flood with half the RAM still free (.bss ends
/// below 0x20000900 in this image, the stack starts at 0x20008000).
constexpr uint32_t unpaced_guard_bytes = 16384;
volatile uint16_t unpaced_ticks = 0;     ///< the window's ticks still to grant
volatile uint16_t unpaced_given = 0;     ///< the ticks the window granted
volatile bool unpaced_guard_hit = false;
volatile uint32_t unpaced_sp_low = 0xFFFF'FFFFu;   ///< the lowest MSP the window's ticks found

/// TC4's share of the window, from its handler: the stack pointer the tick
/// interrupted, the guard, and the mark that lets Pressure's dispatch of
/// this tick post a probe.
void unpaced_tick() {
    const uint32_t sp = __get_MSP();
    if (sp < unpaced_sp_low) {
        unpaced_sp_low = sp;
    }
    if (sp < reinterpret_cast<uintptr_t>(&__stack_top) - unpaced_guard_bytes) {
        unpaced_guard_hit = true;
        unpaced_ticks = 0;
        return;
    }
    unpaced_ticks = static_cast<uint16_t>(unpaced_ticks - 1u);
    unpaced_given = static_cast<uint16_t>(unpaced_given + 1u);
    h::turned = true;
}

/// THE STACK'S BOUND under the preemptive kernel, by the release listing
/// of this image (each function's pushes and `sub sp` summed, the deepest
/// callee followed, an indirect call charged its deepest candidate). A
/// pack of five holds at most five runs (kernel/rubato.hpp, "The stack"):
/// the loop's own, whose dispatch is the Console's, and four nested ones,
/// each started at a dispatch of an index below the last and serving
/// above it - Minor's, the arbiter's, the Client's and Pressure's, the
/// deepest chain there is. From the top of RAM:
///   - the Console's chain: Reset_Handler 8, main() with the kernel loop
///     inlined 96, the loop's activate() 88, the letter's call through
///     TestBench's handle() and run() 56, and this letter with everything
///     it calls 588 (its deepest chain: held_tenure(), the command
///     channel's frame receipt, a tenure posted and its buffers cleared):
///     836;
///   - each of the four nested runs: what starts it - an exception frame
///     over the code it preempts (32, and 4 when the core aligns it) and
///     PendSV's two words (8): 44, or nothing for the Console's base turn,
///     which calls activate() itself - then activate()'s frame, 88, and the
///     deepest a dispatch reaches below it, 180 (the arbiter's state
///     machine down to the SERCOM's synchronization wait; every other
///     object's dispatch reaches less): 312 a run, 1248 for the four;
///   - over Pressure's dispatch, whose index 0 no post can preempt, one
///     interrupt: its frame, 36, and the deepest handler, SERCOM3's 60
///     (the pump's synchronization wait): 96.
/// The run at a ceiling of 0 a handler preempting PendSV's first
/// instruction can leave (cortexm/schedule.hpp) takes the place of the
/// topmost run: the delayed run's 44, the empty run's 44 and 88 and an
/// interrupt's 96 are 272, under the topmost full run's 312 + 96. The
/// fake frame (32) and the exit's SVC frame (32) each stand where an
/// activate() frame of 88 would. So the high-water is at most
/// 836 + 1248 + 96 = 2180 bytes.
constexpr uint32_t unpaced_console = 8u + 96u + 88u + 56u + 588u;
constexpr uint32_t unpaced_stack_bound = unpaced_console + 4u * (44u + 88u + 180u) + 36u + 60u;
static_assert(unpaced_stack_bound == 2180u, "the arithmetic beside it");

void tu_unpaced() {
    const PassSpec s = pass_at(4);   // 97 us x 3000
    begin(s);
    if (!need_peer()) {
        (void)pass_end(s);
        return;
    }
    const uint8_t stale0 = I2cArb::stale_events();
    const uint16_t dropped0 = I2cArb::queue.overflows();
    unpaced_given = 0;
    unpaced_guard_hit = false;
    unpaced_sp_low = 0xFFFF'FFFFu;
    paint_stack();
    unpaced_ticks = unpaced_window;
    const HeldOutcome o = held_tenure(speeds[0], true);
    unpaced_ticks = 0;
    const uint32_t used = stack_used();
    const uint8_t stale = static_cast<uint8_t>(I2cArb::stale_events() - stale0);
    const uint16_t dropped = static_cast<uint16_t>(I2cArb::queue.overflows() - dropped0);
    const PassStats st = pass_end(s);
    print_pass(s, st);
    const uint32_t sp_depth =
        unpaced_sp_low == 0xFFFF'FFFFu ? 0u : reinterpret_cast<uintptr_t>(&__stack_top) - unpaced_sp_low;
    print(serial, "  the flood unpaced for ", unpaced_given, " of ", unpaced_window, " ticks",
          unpaced_guard_hit ? " (CUT BY THE STACK GUARD)" : "", "; the hold's stack high-water ", used,
          " bytes (the bound ", unpaced_stack_bound, "), the deepest stack pointer a tick found ", sp_depth,
          " bytes below the top", crlf);
    print(serial, "    the Console's turns ", us_of(o.turn_gap), " us apart at most; Pressure posted ", h::press.posted,
          ", answered ", h::press.replies, " (ok ", h::press.ok, ", rejected ", h::press.rejected, ", timeout ",
          h::press.timeouts, ", arb_lost ", h::press.arb_lost, ", nack_addr ", h::press.nack_addr, ", bus_error ",
          h::press.bus_errors, ", other ", h::press.other, "), the arbiter's queue full for ", dropped,
          " posts; the Client's tenure into the hold: status ", o.status, status_name(o.status), ", ",
          us_of(o.cycles), " us from its post, ", us_of(o.from_start), " from its start(); stale ", stale,
          "; next tenure ", o.next, ", the vote ", o.idle ? "idle" : "NOT IDLE", crlf);
    bench.verdict(pass_label(s), "the flood ran unpaced, a probe at every tick of the window, the stack guard "
                                 "never reached",
                  unpaced_given == unpaced_window && !unpaced_guard_hit && h::press.posted >= unpaced_given);
    bench.verdict(pass_label(s), "the stack's high-water within the pack's five runs (the arithmetic beside "
                                 "the bound)",
                  used <= unpaced_stack_bound);
    // An overload drops what it cannot queue (kernel/post.hpp: a full
    // queue drops and counts), so the accounting is the requirement: no
    // probe answered twice, none answered outside the vocabulary, and every
    // one answered or counted among the arbiter's dropped posts.
    bench.verdict(pass_label(s), "every probe answered at most once and in the vocabulary, the rest counted "
                                 "dropped by the arbiter's full queue",
                  h::press.replies <= h::press.posted && h::press.other == 0u &&
                      h::press.ok_early == 0u && h::press.nack_early == 0u &&
                      static_cast<uint32_t>(h::press.replies) + dropped >= h::press.posted);
    bench.verdict(pass_label(s), "the overload over, the next tenure i2c_ok and the arbiter idle (its own vote)",
                  o.next == i2c_ok && o.idle);
    judge_pressure(s, st);
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
    if (unpaced_ticks != 0u && h::live) {
        unpaced_tick();   // letter u's window
    }
    pressure_landed();
}

/// Every byte of a tenure and its end: the pump's one vector, posting the
/// arbiter's TransferDone at the completion edge (the STOP sent, a NACK, a
/// loss or an error). There is no DMA vector: the host has no engine.
extern "C" void SERCOM3_Handler() {
    if (Host::isr()) {
        done_from_dma = false;
        brio::post<I2cArb>(brio::TransferDone{Host::status()});
    }
}

int main() {
    clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    tick_ok = brio::Ticker::init(clock);
    host_ok = Host::init(clock, bus_rise_ns);
    pulse_ok = Pulse::init(0) &&
               Pulse::configure(brio::TcConfig{.mode = brio::TcMode::count16,
                                               .prescaler = brio::TcPrescaler::div1,
                                               .waveform = brio::TcWaveform::match_frequency});
    brio::Nvic::enable(Pulse::irq());
    brio::scheduler_init();

    bench.letter('n', "the command channel through the arbiter", tn_command_channel);
    bench.letter('o', "the tenure shapes, every one on the pump", to_shapes);
    bench.letter('p', "the three speeds, and the peer's commanded stretch", tp_speeds);
    bench.letter('q', "the vocabulary under pressure: deaf, a refused byte, a refused last byte", tq_vocabulary);
    bench.letter('w', "the arbiter with three requesters of three priorities", tw_arbiter);
    bench.letter('h', "the held wire, with requests arriving while it is held", th_held_wire);
    bench.letter('i', "dark: every object idle, nothing lit", ti_dark);
    bench.letter('x', "the landings inside start() counted: 400 writes silent and pressed", tx_fill, false);
    bench.letter('u', "the held wire under the flood unpaced at 97 us x 3000, the stack bounded", tu_unpaced,
                 false);

    (void)serial_ok;
    paint_stack();
    brio::enable_interrupts();
    K::run();
}
