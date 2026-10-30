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
//   dispatch, a 3000-cycle spin, and road 1's 232 (docs/samc21/platform.md)
//   are shorter than the shortest period's 4656 cycles, so no tick finds
//   the last one still queued; at 97 us x 3000 it takes 64 % of the core.
//   Letter h's flood adds a probe a tick, whose whole cost is far more than
//   its post (flood_bill below): the letter therefore floods only where
//   the tick and the flood fit the period. TC4 and SERCOM3 sit at the same NVIC priority,
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
// The letters (z: n o p q w h i; x outside it), each but i and x run as
// seven passes (the shared body's control and six pressures):
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
//   h  the held wire (the peer holds SDA 60 ms) with Pressure's probes at
//      every tick where the flood fits the period: answered in its place,
//      every request once, the next tenure i2c_ok and the arbiter idle; then
//      the hold met at another speed (above). THIS FAMILY'S LETTER, NOT THE
//      SHARED ONE (th_held_wire_samc() below): the shared letter floods in
//      every pressure pass, and at 97 us x 3000 the flood's bill and the
//      spin pass the period, so the pack below the Console never empties -
//      the Console, the one object that ends the flood, never runs again,
//      and under Rubato the stack then grows by a scheduler level at a time
//      until it has written over the whole RAM (THE STACK below). The
//      letter here is the shared one with the flood dropped from that one
//      pass, and i2c_bus_error added to the signatures (above).
//   THE STACK is painted and read around every pass (print_host()), its
//   high-water printed with the pass: Rubato runs every dispatch on the one
//   main stack, and a road-1 request landing in activate()'s scan between
//   two dispatches - the ceiling back at the scan's own - nests a whole
//   scheduler level at that same ceiling, which unwinds only when every
//   queue below it has drained; so the depth grows with the load (1.2 KB in
//   a control pass, 2.6 KB under letter h's flood at 97 us x 300, 19.6 KB at
//   97 us x 900, past the RAM at 97 us x 1000), not with the pack's length.
//   THE SERVES carry flag_stop_interrupt (struct Host below): this peer sees
//   a STOP only with it, and the shared body's counted serves lack it.
//   i  dark: every object idle, the arbiter's vote, no time event armed,
//      TC4 stopped and its line quiet, the host idle with ERROR alone armed,
//      no flag standing, its line enabled and not pending, the bus monitor
//      IDLE and both lines high on their pads.
//   x  (outside z) 400 writes of 16 silent and 400 under 97 us x 3000:
//      the landings inside start() counted, and of them those with the bus
//      monitor reading OWNER.
//
// RUNNING IT: `brio flash <the SAM board> test_samc_rubato_i2c`, the peer
// on twi_peer; then `brio run <board> z`.
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
using Engine = I2cHost<3, host_pads>;

/// THE STOP INTERRUPT FOR THE AVR PEER'S SERVES - a workaround for the
/// shared body, which is frozen (the lane's report proposes the change).
/// The AVR64EA48's twi_peer sees a STOP only with PIEN on, which an action
/// asks for by twilink::flag_stop_interrupt (twi_link.hpp; every other
/// family's peer polls its STOP flag and maps the flag to nothing), so a
/// counted serve posted without it never ends on its count - it runs to
/// its deadline, and every command sent before then meets a peer still
/// serving. The body builds its serves without the flag, so this type is
/// the engine with ONE thing added in front of start(): a write to the
/// command address that carries a `serve` frame goes out with
/// flag_stop_interrupt set in its Params and the frame's checksum made
/// again, from a copy of the frame in a buffer of this file's (the body's
/// own buffer is lent const). Every other tenure passes as it came: on a
/// data tenure the addition is one compare of the address, inside
/// letter x's start() count.
struct Host : Engine {
    static bool start(const Request& r) {
        // [magic, op, len, the 16 bytes of Params, checksum]: flags at 3 + 9.
        constexpr uint8_t frame_len = 3u + twilink::params_size + 1u;
        if (r.addr == twilink::command_addr && r.tx_len == frame_len) {
            const uint8_t* const f = r.tx.get();
            if (f[0] == twilink::magic && f[1] == twilink::byte_of(twilink::Op::serve) &&
                f[2] == twilink::params_size) {
                for (uint8_t i = 0; i < frame_len; ++i) {
                    serve_frame_[i] = f[i];
                }
                serve_frame_[3u + 9u] = static_cast<uint8_t>(serve_frame_[3u + 9u] | twilink::flag_stop_interrupt);
                serve_frame_[frame_len - 1u] = twilink::checksum(f[1], f[2], serve_frame_ + 3u);
                Request q = r;
                q.tx = lend<Lease::reply>(static_cast<const uint8_t*>(serve_frame_));
                return Engine::start(q);
            }
        }
        return Engine::start(r);
    }

private:
    static inline uint8_t serve_frame_[3u + twilink::params_size + 1u] = {};
};
static_assert(
    [] {
        twilink::Params a{};
        a.flags = 0x5A;
        uint8_t p[twilink::params_size] = {};
        twilink::put_params(p, a);
        return p[9] == 0x5A;
    }(),
    "the serve frame's layout: Params' flags at payload byte 9");
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
// h - the held wire, this family's way (the shared letter stopped)
// =============================================================================

/// The bus errors the SERCOM3 vector saw complete, and of them those whose
/// edge fell while the peer still held SDA (h::still_held(), the shared
/// body's arithmetic) - written by the vector, read by letter h.
volatile uint16_t bus_errors = 0;
volatile uint16_t bus_errors_held = 0;


/// What one flood probe costs this core end to end while the peer holds
/// SDA - its post, the arbiter's dispatch of it, a START that loses its
/// address at once after the recovery, the completion's vector and
/// dispatch, the reply's road 2 into Pressure - read off the bench: the
/// stack's high-water, flat at 1.3 to 1.8 KB with the flood at 193 and
/// 389 us under 3000 cycles, ran away at 97 us between a spin of 900
/// cycles (19.6 KB) and 1000 (past the 30 KB of RAM): 4656 - 1000 - 232 =
/// 3424, rounded up.
constexpr uint32_t flood_bill = 3500;
constexpr uint32_t road1 = 232;   // docs/samc21/platform.md
/// The flood runs in a pass whose tick, its spin, the flood's bill and the
/// road that brings it fit inside the period; elsewhere the pass meets the
/// held wire without it.
constexpr bool flood_fits(const PassSpec& s) {
    return s.period_us != 0u && s.spin + flood_bill + road1 < static_cast<uint32_t>(s.period_us) * cycles_per_us;
}
static_assert(flood_fits(passes[1]) && flood_fits(passes[2]) && flood_fits(passes[3]) && !flood_fits(passes[4]) &&
                  flood_fits(passes[5]) && flood_fits(passes[6]),
              "the flood runs in every pressure pass but 97 us x 3000");

/// The SAM's held-wire signatures: the shared body's two, and i2c_bus_error
/// - ARBLOST with BUSERR beside it, which the engine reports as the bus
/// error it also is (samc21/i2c.hpp's settle(): "the protocol-violation
/// flavour of the same loss").
bool held_signature_samc(uint8_t st) { return held_signature(st) || st == i2c_bus_error; }

void th_held_wire_samc() {
    // The shared letter is the reference this one departs from: kept
    // compiled, not run (the file header).
    static_cast<void>(&th_held_wire);
    for (uint8_t p = 0; p < pass_count; ++p) {
        const PassSpec& s = passes[p];
        const bool pressing = s.period_us != 0u;
        const bool flood = flood_fits(s);
        begin(s);
        if (!need_peer()) {
            (void)pass_end(s);
            return;
        }
        I2cArb::draining_requests = 0;
        const uint8_t stale0 = I2cArb::stale_events();
        const uint16_t be0 = bus_errors, beh0 = bus_errors_held;
        const HeldOutcome o = held_tenure(speeds[0], flood);
        const uint8_t stale = static_cast<uint8_t>(I2cArb::stale_events() - stale0);
        const uint16_t be = static_cast<uint16_t>(bus_errors - be0);
        const uint16_t beh = static_cast<uint16_t>(bus_errors_held - beh0);
        const PassStats st = pass_end(s);
        print_pass(s, st);
        print(serial, "    the held tenure: status ", o.status,
              o.status == i2c_timeout ? " (i2c_timeout)"
                                      : (o.status == i2c_arb_lost ? " (i2c_arb_lost)"
                                                                  : (o.status == i2c_bus_error ? " (i2c_bus_error)" : "")),
              " in ", us_of(o.cycles), " us; ", flood ? "the flood" : "NO FLOOD (its bill past the period)",
              ": Pressure posted ", h::press.posted, ", answered ", h::press.replies, " (ok ", h::press.ok,
              ", of them while held ", h::press.ok_early, "; rejected ", h::press.rejected, ", timeout ",
              h::press.timeouts, ", arb_lost ", h::press.arb_lost, ", nack_addr ", h::press.nack_addr, " (",
              h::press.nack_early, " while held), other ", h::press.other, " last ", h::press.other_status,
              "); bus errors ", be, " (", beh, " while held); Requests in the draining window ",
              I2cArb::draining_requests, ", stale ", stale, "; next tenure ", o.next, ", the vote ",
              o.idle ? "idle" : "NOT IDLE", crlf);
        bench.verdict(pass_label(s), "the held tenure answered in its place, never i2c_ok - i2c_timeout, i2c_arb_lost "
                                     "or i2c_bus_error, this family's signatures",
                      o.acted && held_signature_samc(o.status));
        bench.verdict(pass_label(s), "... at the arbiter's limit or sooner (22 ms, the letter's arithmetic)",
                      o.acted && o.cycles <= held_answer_ms * (core_hz / 1000u));
        bench.verdict(pass_label(s), "every request Pressure posted answered exactly once, i2c_rejected, i2c_timeout, "
                                     "i2c_arb_lost or i2c_bus_error while held, after the release i2c_ok or "
                                     "i2c_nack_addr",
                      o.drained && h::press.replies == h::press.posted &&
                          (h::press.other == 0u || h::press.other_status == i2c_bus_error) &&
                          h::press.other + (o.status == i2c_bus_error ? 1u : 0u) == be &&
                          h::press.ok_early == 0u && h::press.nack_early == 0u && (flood || h::press.posted == 0u));
        bench.verdict(pass_label(s), "after the release the next tenure i2c_ok, the arbiter idle with nothing "
                                     "pending (its own vote)",
                      o.next == i2c_ok && o.idle);
        judge_pressure(s, st);
        (void)pressing;
    }
    if (!need_peer()) {
        return;
    }
    const HeldOutcome o = held_tenure(speeds[1], false);
    print(serial, "  the hold met at ", speed_hz(speeds[1]) / 1000u, " kHz after commands at ",
          speed_hz(speeds[0]) / 1000u, ": status ", o.status, " in ", us_of(o.cycles), " us; next tenure ", o.next,
          crlf);
    bench.verdict("a rate change into the held wire: answered in its place, never i2c_ok, at the limit or sooner, "
                  "the next tenure i2c_ok",
                  o.acted && held_signature_samc(o.status) && o.cycles <= held_answer_ms * (core_hz / 1000u) &&
                      o.next == i2c_ok && o.idle);
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
    pressure_landed();
}

/// Every byte of a tenure and its end: the pump's one vector, posting the
/// arbiter's TransferDone at the completion edge (the STOP sent, a NACK, a
/// loss or an error). There is no DMA vector: the host has no engine.
extern "C" void SERCOM3_Handler() {
    if (Host::isr()) {
        if (Host::status() == brio::i2c_bus_error) {
            bus_errors = static_cast<uint16_t>(bus_errors + 1u);
            if (h::live && h::still_held(Ruler::now())) {
                bus_errors_held = static_cast<uint16_t>(bus_errors_held + 1u);
            }
        }
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
    bench.letter('h', "the held wire, with requests arriving while it is held", th_held_wire_samc);
    bench.letter('i', "dark: every object idle, nothing lit", ti_dark);
    bench.letter('x', "the landings inside start() counted: 400 writes silent and pressed", tx_fill, false);

    (void)serial_ok;
    paint_stack();
    brio::enable_interrupts();
    K::run();
}
