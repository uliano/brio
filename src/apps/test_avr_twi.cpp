// test_avr_twi - the TWI (I2C) test SUITE for the AVR DA/DB target.
//
// SINGLE BOARD (a..j, `z`): the route table and its refusals, the three
// bus speeds MEASURED on SCL against the chapter's own baud equations,
// the COMBINED loop (a host and a client on one instance and one pin
// pair, the host addressing its own client) and the DUAL loop (the
// client moved to the route's second pin pair, the desk's bus node
// closing the circuit), the whole address-match space proven by who
// ACKs, the chapter's host cases M1..M3 and client cases S1..S3, both
// Smart modes, Quick Command counted in SCL edges, the bus state
// machine driven by a bit-banged injector on a third tap of the same
// bus, the two ISR bodies, and a clock rebase under traffic with the
// I2cBus/BusMaster stack riding the engine.
//
// Reference test of avrdx/twi.hpp (docs/avrdx/twi.md): keep it passing.
//
// Bench diagnostic, NOT a kernel app (sequential, blocking). Console on
// USART2 ALT1 (PF4/PF5) at 460800 - USART2 is never reconfigured here.
//
// Wires: ONE open-drain I2C bus with 1.5k pull-ups to +5 V, two taps of
// THIS board on it:
//     SDA node   PA2 + PC2        SCL node   PA3 + PC3
// PA2/PA3 is TWI0's DEFAULT (and ALT1) pin pair - the host, and the
// client of the combined loop. PC2/PC3 is that route's DUAL pair - the
// client of the dual loop (PORTC is the MVIO domain: VDDIO2 must be
// powered, and at the same 5 V as VDD). While Dual mode is OFF that same
// pair is plain GPIO on the same node, and test h bit-bangs it as an
// INJECTOR (drive low = pull the open-drain line down, release = input):
// the only way one board can put a foreign START, a foreign STOP or a
// protocol violation on its own bus. TWI1's dual pair PB2/PB3 would be a
// third tap; this desk does not fit it, and test h says so out loud
// instead of assuming either way. TWI1 stays DISABLED throughout.
//
// TWO BOARDS (k..s, `y`): board B runs `twi_peer` and taps the same node
// with its own PA2/PA3. The DUT drives it IN BAND over the bus itself -
// a write tenure to `twilink::command_addr` carries one command frame, a
// read tenure collects the answer (src/apps/twi_link.hpp) - and then
// measures what a SECOND, independent chip does to the wire: clock
// stretching, an injected address or data NACK, multi-host arbitration
// with both boards combined, the collision case S4 with two clients on
// one address, a really stuck SDA against `Twi<0>::unstick()`, a General
// Call answered by two chips, the three bus speeds and a clock rebase.
// The peer's command-mode client answers ONE address exactly, with no
// General Call, no mask, no PMEN and no PIEN, so the single-board half
// above cannot see it at all - which is why `z` scores its full count
// with board B attached and running.
//
// Commands: ? for the menu, z = the single-board half, y = the two-board
// half.

// pio: monitor_speed = 460800

#include <avr/interrupt.h>
#include <stdint.h>

#include "avrdx/clock.hpp"
#include "avrdx/delay.hpp"
#include "avrdx/evsys.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/tcb.hpp"
#include "avrdx/twi.hpp"
#include "avrdx/usart.hpp"
#include "avrdx/userrow.hpp"
#include "kernel/active_object.hpp"
#include "kernel/event_queue.hpp"
#include "util/print.hpp"

#include "twi_link.hpp"

using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = AvrPlatform;

using Serial = Uart<2, Route::alt1>;
constexpr Serial serial;

using T = Twi<0>;
using T1 = Twi<1>;

using Sda = Pin<'A', 2>;
using Scl = Pin<'A', 3>;
using DualSda = Pin<'C', 2>;
using DualScl = Pin<'C', 3>;
// The bit-bang INJECTOR: the route's DUAL pin pair, PC2/PC3, which is
// plain GPIO whenever Dual mode is off - and which the dual loop of
// test c proves is electrically the same node as PA2/PA3.
using InjSda = DualSda;
using InjScl = DualScl;
// PORTB's pair is TWI1's dual pair and would be a third tap; this desk
// does not fit it (test h says so out loud rather than assuming).
using InjB2 = Pin<'B', 2>;

using Host = TwiHost<0, TwiRoute::def>;
using Client = TwiClient<0, TwiRoute::def>;
using DualClient = TwiClient<0, TwiRoute::def, true>;

// The SCL instruments: PORTA pin events live on channels 0-1 (evsys.hpp).
// One channel feeds two TCBs - a capture meter and an edge counter.
using ChScl = EventChannel<0>;
using Meter = Tcb<0>;
using SclFreq = FrequencyMeter<Meter>;
using SclLow = PulseWidthMeter<Meter>;
using Stopwatch = Tcb<1>;
using SclCount = PulseCounter<Stopwatch>;
// The two-board half needs a stopwatch longer than a TCB's 2.7 ms: a
// millisecond of clock stretch per byte runs a transaction well past
// that. TCB2 + TCB3 as one 32-bit counter at CLK_PER (178 s at 24 MHz),
// on two channels of their own.
using WatchLo = Tcb<2>;
using WatchHi = Tcb<3>;
using Watch = CascadedCounter<WatchLo, WatchHi>;
using ChCarry = EventChannel<2>;
using ChSnap = EventChannel<3>;

// The arbiter that rides the engine (util/i2c_bus.hpp).
using Bus = I2cBus<Host, P>;
static_assert(ActiveObject<Bus>,
              "the I2cBus/BusMaster stratum must still fit over this engine");

using DynClock = DynamicClock<SysClock, Serial, Host, Client, SclFreq>;

// The addresses this suite uses. 0x42 is the client under test; 0x77 is
// held by nobody on this bus and is the address every NACK case sends to.
constexpr uint8_t client_addr = 0x42;
constexpr uint8_t absent_addr = 0x77;

// ---- verdicts ----------------------------------------------------------------

uint8_t passed = 0, failed = 0;

void verdict(const char* name, bool ok) {
    if (ok) ++passed; else ++failed;
    print(serial, "  ", ok ? "PASS" : "FAIL", "  ", name, crlf);
}
void verdict(const char* a, const char* b, bool ok) {
    if (ok) ++passed; else ++failed;
    print(serial, "  ", ok ? "PASS" : "FAIL", "  ", a, b, crlf);
}
bool near(int32_t a, int32_t b, int32_t tol) {
    const int32_t d = a > b ? a - b : b - a;
    return d <= tol;
}

// ---- the client half, serviced by polling (and by the vector in test i) ------

volatile uint16_t cl_addr_hits = 0;
volatile uint16_t cl_stops = 0;
volatile uint16_t cl_data_events = 0;
volatile uint16_t cl_ints = 0;
volatile uint8_t cl_rx[16];
volatile uint8_t cl_rx_n = 0;
volatile uint8_t cl_sent = 0;
volatile uint8_t cl_last_addr = 0;
volatile bool cl_dir_read = false;
volatile bool cl_saw_nack = false;
uint8_t cl_nack_after = 0xFF;      ///< NACK the n-th data byte received (1-based)
uint8_t cl_tx_seed = 0xA0;         ///< the client answers seed, seed+1, ...
bool cl_read_address = false;      ///< read SDATA at the address match (test d)
bool cl_fixed_tx = false;          ///< answer cl_tx_seed every time (test o)

void cl_reset() {
    cl_addr_hits = 0;
    cl_stops = 0;
    cl_data_events = 0;
    cl_ints = 0;
    cl_rx_n = 0;
    cl_sent = 0;
    cl_saw_nack = false;
    cl_last_addr = 0xFF;
}

/// One pass of the client protocol (29.3.2.3): S1 receive, S2 transmit,
/// S3 stop. Written against the TASK's verbs, so both the combined and
/// the dual client run the same code.
template <typename C>
void service_client() {
    const auto s = C::isr();
    if (s.address_or_stop()) {
        ++cl_ints;
        if (s.is_address()) {
            ++cl_addr_hits;
            cl_dir_read = s.host_reading();
            cl_sent = 0;
            if (cl_read_address) cl_last_addr = C::last_address();
            C::respond(TwiAck::ack);
        } else {
            ++cl_stops;
            C::complete();
        }
        return;
    }
    if (!s.data()) return;
    ++cl_ints;
    ++cl_data_events;
    if (s.host_reading()) {
        if (cl_sent != 0 && s.nack()) {
            cl_saw_nack = true;
            C::complete();
            return;
        }
        C::transmit(cl_fixed_tx ? cl_tx_seed : static_cast<uint8_t>(cl_tx_seed + cl_sent));
        ++cl_sent;
        return;
    }
    const bool nack = cl_nack_after != 0xFF &&
                      static_cast<uint8_t>(cl_rx_n + 1) >= cl_nack_after;
    const uint8_t v = C::receive(nack ? TwiAck::nack : TwiAck::ack);
    if (cl_rx_n < 16) cl_rx[cl_rx_n] = v;
    ++cl_rx_n;
}

/// MSTATUS as it stood on the pass that completed the last request -
/// the only place RXACK and the bus state of a failure can be READ, since
/// the engine's isr() clears what it acts on.
uint8_t last_mstatus = 0;

/// Pump both halves until the HOST raises a flag; returns MSTATUS (0 on
/// a time-out).
template <typename C>
uint8_t pump_to_host_flag(uint32_t spins = 200'000u) {
    for (uint32_t i = 0; i < spins; ++i) {
        service_client<C>();
        const uint8_t s = T::host_status();
        if (s & (TWI_RIF_bm | TWI_WIF_bm | TWI_ARBLOST_bm | TWI_BUSERR_bm)) return s;
    }
    return 0;
}

/// One whole Request through the engine, pumped by hand (the vectors
/// stay silent in the polled tests). Returns the engine's status, or
/// 0xFE when nothing ever completed.
template <typename C>
uint8_t run_request(const Host::Request& r, bool with_client = true,
                    uint32_t spins = 400'000u) {
    cl_reset();
    if (Host::start(r)) return Host::status();
    for (uint32_t i = 0; i < spins; ++i) {
        if (with_client) service_client<C>();
        const uint8_t s = T::host_status();
        if ((s & (TWI_RIF_bm | TWI_WIF_bm | TWI_ARBLOST_bm | TWI_BUSERR_bm))) last_mstatus = s;
        if ((s & (TWI_RIF_bm | TWI_WIF_bm | TWI_ARBLOST_bm | TWI_BUSERR_bm)) &&
            Host::isr()) {
            // Let the client see the STOP that just went out.
            if (with_client) {
                for (uint16_t k = 0; k < 4000; ++k) service_client<C>();
            }
            return Host::status();
        }
    }
    return 0xFE;
}

// ---- the injector: PC2/PC3 as plain GPIO on the same open-drain bus ---------

/// Open drain by hand: an OUTPUT with OUT = 0 pulls the line down, an
/// INPUT lets the pull-up take it back. The OUT bits are never set, so
/// these two pins are left exactly as errata 2.15.1 wants any future
/// TWI1 owner to find them.
void inj_park() {
    InjSda::input();
    InjSda::clear();
    InjScl::input();
    InjScl::clear();
}
void inj_sda(bool low) { if (low) InjSda::output(); else InjSda::input(); }
void inj_scl(bool low) { if (low) InjScl::output(); else InjScl::input(); }

constexpr uint16_t inj_us = 5;

/// A foreign START, then both lines released WITHOUT a STOP (SDA rises
/// while SCL is low, which is a data bit, not a Stop condition): the bus
/// is left electrically idle but the state machine has seen a START.
void inject_start_and_leave() {
    inj_sda(true);
    delay_us(clock, inj_us);
    inj_scl(true);
    delay_us(clock, inj_us);
    inj_sda(false);
    delay_us(clock, inj_us);
    inj_scl(false);
    delay_us(clock, inj_us);
}

/// n more clock pulses on SCL, SDA left to the pull-up (all ones).
void inject_clocks(uint8_t n) {
    for (uint8_t i = 0; i < n; ++i) {
        inj_scl(true);
        delay_us(clock, inj_us);
        inj_scl(false);
        delay_us(clock, inj_us);
    }
}

/// A foreign START and a whole nine-bit frame of ones, then silence: the
/// bus is left Busy at a BYTE BOUNDARY, which is what a real host that
/// walked away would leave behind.
void inject_start_and_byte() {
    inject_start_and_leave();      // START plus the first clock
    inject_clocks(8);              // ... and the other eight of the frame
}

/// A proper STOP: SCL low and SDA low, release SCL, then release SDA.
void inject_stop() {
    inj_scl(true);
    delay_us(clock, inj_us);
    inj_sda(true);
    delay_us(clock, inj_us);
    inj_scl(false);
    delay_us(clock, inj_us);
    inj_sda(false);
    delay_us(clock, inj_us);
}

/// The protocol violation of 29.5.6: a START directly followed by a STOP
/// (SDA down and back up with SCL held high throughout).
void inject_start_stop() {
    inj_sda(true);
    delay_us(clock, inj_us);
    inj_sda(false);
    delay_us(clock, inj_us);
}

// ---- instruments -------------------------------------------------------------

volatile uint16_t meter_min = 0xFFFF;
volatile uint16_t meter_max = 0;
volatile uint16_t meter_caps = 0;
volatile uint8_t meter_mode = 0;        // 0 = period, 1 = low width

void meter_reset() {
    cli();
    meter_min = 0xFFFF;
    meter_max = 0;
    meter_caps = 0;
    sei();
}

void meter_stop() {
    Meter::enable_capt_interrupt(false);
    Meter::disable();
}

/// Everything back to a known quiet state.
void quiesce() {
    T::release();
    T1::release();
    meter_stop();
    Stopwatch::disable();
    WatchLo::disable();
    WatchHi::disable();
    ChScl::off();
    ChCarry::off();
    ChSnap::off();
    inj_park();
    cl_reset();
    cl_nack_after = 0xFF;
    cl_read_address = false;
    cl_fixed_tx = false;
    cl_tx_seed = 0xA0;
}

/// The host, polled: the engine arms RIEN/WIEN by design, and the polled
/// tests take them straight back down so the vector never fires.
bool host_polled(TwiSpeed s = TwiSpeed::standard_100k, Host::Options o = {}) {
    o.speed = s;
    const bool ok = Host::init(clock, o);
    T::enable_read_interrupt(false);
    T::enable_write_interrupt(false);
    return ok;
}

bool client_polled(uint8_t addr = client_addr, Client::Options o = {}) {
    o.address = addr;
    return Client::init(clock, o);
}

/// The wire sanity that must precede every conclusion on this desk: with
/// both TWIs disabled, both taps of both nodes must read HIGH.
bool wire_sane(bool loud) {
    T::release();
    T1::release();
    inj_park();
    delay_us(clock, 1000);
    const bool a2 = Sda::read(), a3 = Scl::read();
    const bool c2 = DualSda::read(), c3 = DualScl::read();
    const bool b2 = (PORTB.IN & PIN2_bm) != 0, b3 = (PORTB.IN & PIN3_bm) != 0;
    if (loud) {
        print(serial, "  bus at rest: SDA node PA2=", a2 ? 1 : 0, " PC2=", c2 ? 1 : 0,
              " | SCL node PA3=", a3 ? 1 : 0, " PC3=", c3 ? 1 : 0,
              " | PORTB PB2=", b2 ? 1 : 0, " PB3=", b3 ? 1 : 0, crlf);
    }
    return a2 && a3 && c2 && c3;
}

/// The connectivity proof the injector rests on: pulling the injector's
/// SDA (SCL) down must pull the host's PA2 (PA3) down with it, because
/// they are one open-drain node.
bool node_tied(bool loud) {
    inj_park();
    delay_us(clock, 200);
    inj_sda(true);
    delay_us(clock, 200);
    const bool sda_down = !Sda::read();
    inj_sda(false);
    inj_scl(true);
    delay_us(clock, 200);
    const bool scl_down = !Scl::read();
    inj_scl(false);
    delay_us(clock, 200);
    const bool back_up = Sda::read() && Scl::read();
    if (loud) {
        print(serial, "  injector pulls the SDA node down = ", sda_down ? 1 : 0,
              ", the SCL node down = ", scl_down ? 1 : 0,
              ", both release = ", back_up ? 1 : 0, crlf);
    }
    return sda_down && scl_down && back_up;
}

/// Is PORTB's pair a third tap of this bus? On this desk it is NOT
/// fitted, and the suite says so instead of assuming either way.
void report_portb_tap() {
    PORTB.OUTCLR = PIN2_bm | PIN3_bm;
    PORTB.DIRSET = PIN2_bm;
    delay_us(clock, 200);
    const bool pulls_sda = !Sda::read();
    const bool pb2_low = !InjB2::read();
    PORTB.DIRCLR = PIN2_bm;
    PORTB.DIRSET = PIN3_bm;
    delay_us(clock, 200);
    const bool pulls_scl = !Scl::read();
    PORTB.DIRCLR = PIN3_bm;
    delay_us(clock, 200);
    print(serial, "  PORTB tap: PB2 driven low reads ", pb2_low ? 0 : 1,
          " at its own pin and pulls the SDA node to ", pulls_sda ? 0 : 1,
          "; PB3 pulls the SCL node to ", pulls_scl ? 0 : 1, " -> PB2/PB3 ",
          (pulls_sda && pulls_scl) ? "ARE" : "are NOT", " tied to this bus", crlf);
}

// ---- a: routes, pins and the refusals ----------------------------------------

void ta_routes() {
    print(serial, "a routes: PORTMUX, the pin claim and its errata hygiene, the dual "
                  "pair, the refusals", crlf);
    quiesce();

    verdict("the pulled-up bus reads high on all six taps", wire_sane(true));

    // ERRATA DB 2.15.1 / DA 2.14.1 as code: a PORT.OUT bit left at '1'
    // on SDA or SCL would hold that line high under an enabled TWI. Both
    // init paths clear it - so set it first and watch it go.
    Sda::set();
    Scl::set();
    const bool out_before = (PORTA.OUT & (PIN2_bm | PIN3_bm)) == (PIN2_bm | PIN3_bm);
    verdict("PORT.OUT deliberately dirtied on PA2/PA3", out_before);
    verdict("TWI0 DEFAULT host init", host_polled());
    verdict("init cleared PORT.OUT on both pins (errata 2.15.1 / 2.14.1)",
            (PORTA.OUT & (PIN2_bm | PIN3_bm)) == 0);
    verdict("init left both pins inputs for the open-drain override",
            !Sda::is_output() && !Scl::is_output());
    verdict("PORTMUX reads DEFAULT", T::routed() == TwiRoute::def);
    verdict("route() remembers it", T::route() == TwiRoute::def);
    verdict("the host half is enabled", T::host_enabled() && !T::client_enabled());
    verdict("the bus was declared idle", T::bus_state() == TwiBusState::idle);

    T::release();
    verdict("release disables both halves", !T::host_enabled() && !T::client_enabled());
    verdict("release leaves the pins inputs with OUT = 0",
            !Sda::is_output() && !Scl::is_output() &&
            (PORTA.OUT & (PIN2_bm | PIN3_bm)) == 0);
    verdict("release returns PORTMUX to DEFAULT (the TWI has no pinless route)",
            T::routed() == TwiRoute::def);

    // ALT1 shares DEFAULT's pin pair; ALT2 moves the bus to PC2/PC3.
    verdict("TWI0 ALT1 init", T::init({.route = TwiRoute::alt1}, SysClock::hz));
    verdict("ALT1 reads back", T::routed() == TwiRoute::alt1);
    verdict("ALT1 is the same PA2/PA3 pair", T::sda(TwiRoute::alt1).port == 'A' &&
                                             T::sda(TwiRoute::alt1).pin == 2);
    DualSda::set();
    DualScl::set();
    verdict("TWI0 ALT2 init (the bus moves to PC2/PC3)",
            T::init({.route = TwiRoute::alt2}, SysClock::hz));
    verdict("ALT2 reads back", T::routed() == TwiRoute::alt2);
    verdict("ALT2 cleared PORT.OUT on PC2/PC3",
            (PORTC.OUT & (PIN2_bm | PIN3_bm)) == 0);
    T::release();

    // The DUAL pair: the client on PC2/PC3 while the host keeps PA2/PA3.
    DualSda::set();
    DualScl::set();
    verdict("TWI0 DEFAULT + dual client init",
            T::init({.route = TwiRoute::def, .client = true, .address = client_addr,
                     .dual = true}, SysClock::hz));
    verdict("dual mode is on", T::dual_mode());
    verdict("the dual claim cleared PORT.OUT on PC2/PC3",
            (PORTC.OUT & (PIN2_bm | PIN3_bm)) == 0);
    T::release();
    verdict("release turned dual mode off", !T::dual_mode());

    // What must be refused at run time (the compile-time twins live in
    // test/family/neg/).
    verdict("1 MHz without FMPEN refused (29.3.3.1: Fm+ is a PAD setting)",
            !T::init({.speed = TwiSpeed::fast_plus_1m}, SysClock::hz));
    verdict("1 MHz WITH FMPEN accepted",
            T::init({.fm_plus = true, .speed = TwiSpeed::fast_plus_1m}, SysClock::hz));
    T::release();
    verdict("an instance with neither half enabled refused",
            !T::init({.host = false}, SysClock::hz));
    verdict("an 8-bit client address refused",
            !T::init({.client = true, .address = 0x80}, SysClock::hz));
    verdict("dual mode without a client refused",
            !T::init({.dual = true}, SysClock::hz));
    // TWI1 exists here; its ALT1 route has no dual pair on a 48-pin part
    // (PB6/PB7 are 64-pin), so a dual client there is refused at run time.
    verdict("TWI1 ALT1 dual client refused on 48 pins (PB6/PB7 absent)",
            !T1::init({.route = TwiRoute::alt1, .client = true, .address = 0x20,
                       .dual = true}, SysClock::hz));
    verdict("TWI1 ALT2 exists on 48 pins (PB2/PB3)", twi_route_exists(1, TwiRoute::alt2));
    T1::release();

    // The route table this package really has.
    verdict("TWI0 DEFAULT has its dual pair (PC2/PC3)", twi_has_dual(0, TwiRoute::def));
    verdict("TWI0 ALT1's dual pair is PC6/PC7 here",
            twi_dual_pin(0, TwiRoute::alt1, TwiSignal::sda).port == 'C' &&
            twi_dual_pin(0, TwiRoute::alt1, TwiSignal::sda).pin == 6 &&
            twi_has_dual(0, TwiRoute::alt1));
    verdict("TWI1 DEFAULT's dual pair is PB2/PB3 here", twi_has_dual(1, TwiRoute::def));
    verdict("TWI1 ALT1 has no dual pair here", !twi_has_dual(1, TwiRoute::alt1));

    quiesce();
}

// ---- b: the three bus speeds, measured on SCL ---------------------------------

/// Address a device nobody holds, over and over: nine SCL clocks per
/// attempt, no client to stretch anything. The meter's MINIMUM is the
/// undisturbed period.
void nack_traffic(uint8_t n) {
    for (uint8_t i = 0; i < n; ++i) {
        T::address_write(absent_addr);
        for (uint32_t s = 0; s < 100'000u; ++s) {
            if (T::write_flag()) break;
        }
        T::host_command(TwiHostCmd::stop);
        delay_us(clock, 60);
    }
}

uint16_t measure(uint8_t mode) {
    meter_mode = mode;
    if (mode == 0) SclFreq::init(clock, ChScl{}, TcbClock::div1);
    else SclLow::init(clock, ChScl{}, TcbClock::div1, true);
    meter_reset();
    nack_traffic(8);
    meter_stop();
    return meter_min;
}

uint16_t last_rise_ns = 0;
uint16_t last_fall_ns = 0;

void measure_speed(TwiSpeed s, const char* name, uint16_t rise = 0, uint16_t fall = 0) {
    if (!host_polled(s, {.rise_ns = rise, .fall_ns = fall})) {
        verdict("host init ", name, false);
        return;
    }
    const uint8_t baud = T::baud();
    const uint16_t floor_ticks = static_cast<uint16_t>(twi_period_ticks(SysClock::hz, baud, 0));
    const uint16_t period = measure(0);
    const uint16_t low = measure(1);
    const uint16_t reg_low = twi_low_ticks(baud);           // (BAUD + 5), equation 29-4
    // The two bus properties this measurement resolves: the rise adds to
    // the period beyond the register's floor, the fall is subtracted
    // from the register's low time (equations 29-2 and 29-4).
    const uint32_t rise_ns = (period > floor_ticks ? period - floor_ticks : 0) * 1000ul / 24ul;
    const uint32_t fall_ns = (reg_low > low ? reg_low - low : 0) * 1000ul / 24ul;
    const uint32_t low_ns = low * 1000ul / 24ul;
    const uint32_t hz = period ? SysClock::hz / period : 0;
    last_rise_ns = static_cast<uint16_t>(rise_ns);
    last_fall_ns = static_cast<uint16_t>(fall_ns);
    print(serial, "  ", name, ": MBAUD=", baud, " period=", period, " ticks (register "
          "floor ", floor_ticks, ") -> ", hz, " Hz; tLOW=", low, " of ", reg_low,
          " ticks = ", low_ns, " ns (floor ", twi_low_min_ns(s), "); measured tR=",
          rise_ns, " ns tOF=", fall_ns, " ns", crlf);
    verdict("the period never runs under the register's floor ", name,
            period >= floor_ticks);
    verdict("tLOW is equation 29-4 minus a bounded fall time ", name,
            low <= reg_low && low + 8 >= reg_low);
    verdict("tLOW meets the mode's specified minimum ", name, low_ns >= twi_low_min_ns(s));
    verdict("SCL does not exceed the mode's nominal rate ", name, hz <= twi_scl_hz(s));
    T::release();
}

void tb_speeds() {
    print(serial, "b the three bus speeds on SCL (PA3 pin event -> TCB period and "
                  "low-width meters), against equations 29-2..29-5", crlf);
    quiesce();
    ChScl::source(EvPin<Scl>{});
    measure_speed(TwiSpeed::standard_100k, "Sm 100 kHz");
    const uint16_t desk_rise = last_rise_ns;
    const uint16_t desk_fall = last_fall_ns;
    measure_speed(TwiSpeed::fast_400k, "Fm 400 kHz");
    measure_speed(TwiSpeed::fast_plus_1m, "Fm+ 1 MHz");

    // The default charges the specification's worst case, because that
    // is all a driver can know. A bus that DECLARES its own measured
    // edges gets the speed back - and must still meet the floor.
    const uint16_t rise = static_cast<uint16_t>(desk_rise + 40);   // one tick of margin
    const uint16_t fall = static_cast<uint16_t>(desk_fall + 40);
    print(serial, "  this desk measures tR ~", desk_rise, " ns and tOF ~", desk_fall,
          " ns; declaring ", rise, "/", fall, " ns:", crlf);
    measure_speed(TwiSpeed::standard_100k, "Sm 100 kHz, timing declared", rise, fall);

    // Fast-mode Plus is the pads as much as the divider, and the engine
    // turns FMPEN on with the speed.
    verdict("host at 1 MHz", host_polled(TwiSpeed::fast_plus_1m));
    verdict("the engine turned FMPEN on for Fm+", T::fm_plus());
    verdict("a 1 MHz request without FMPEN is not expressible: the resource refuses it",
            !T::init({.speed = TwiSpeed::fast_plus_1m}, SysClock::hz));
    verdict("host back at 100 kHz", host_polled(TwiSpeed::standard_100k));
    verdict("the engine turned FMPEN off again", !T::fm_plus());
    print(serial, "  actual_scl_hz at 100 kHz: ", Host::actual_scl_hz(0), " Hz (tR = 0), ",
          Host::actual_scl_hz(twi_rise_budget_ns(TwiSpeed::standard_100k)),
          " Hz (the 1000 ns budget), ", Host::actual_scl_hz(desk_rise),
          " Hz (this desk's measured rise)", crlf);
    verdict("actual_scl_hz never claims more than the nominal rate",
            Host::actual_scl_hz(0) <= 100'000u);
    verdict("CLK_PER is at least four times SCL", Host::clock_ok());
    quiesce();
}

// ---- c: the combined loop and the dual loop -----------------------------------

uint8_t tx_buf[8];
uint8_t rx_buf[8];

/// Every Request shape against a client of this same instance.
template <typename C>
void shape_matrix(const char* where) {
    // write
    for (uint8_t i = 0; i < 5; ++i) tx_buf[i] = static_cast<uint8_t>(0x31 + i);
    uint8_t st = run_request<C>({client_addr, tx_buf, 5, nullptr, 0, {}});
    bool ok = st == i2c_ok && cl_rx_n == 5 && cl_addr_hits == 1;
    for (uint8_t i = 0; i < 5; ++i) ok = ok && cl_rx[i] == tx_buf[i];
    if (!ok) {
        print(serial, "    write: status=", st, " addr_hits=", cl_addr_hits,
              " rx_n=", cl_rx_n, " first=", hex(cl_rx[0]), crlf);
    }
    verdict("write: five bytes reach the client ", where, ok);
    verdict("the client saw the STOP ", where, cl_stops >= 1);

    // read
    cl_tx_seed = 0x50;
    for (uint8_t i = 0; i < 8; ++i) rx_buf[i] = 0;
    st = run_request<C>({client_addr, nullptr, 0, rx_buf, 4, {}});
    ok = st == i2c_ok && cl_addr_hits == 1 && cl_dir_read;
    for (uint8_t i = 0; i < 4; ++i) ok = ok && rx_buf[i] == static_cast<uint8_t>(0x50 + i);
    if (!ok) {
        print(serial, "    read: status=", st, " got", hex(rx_buf[0]), " ", hex(rx_buf[1]),
              " ", hex(rx_buf[2]), " ", hex(rx_buf[3]), " sent=", cl_sent, crlf);
    }
    verdict("read: four bytes come back from the client ", where, ok);
    verdict("the client saw the host's closing NACK ", where, cl_saw_nack);

    // write-then-read (repeated START)
    cl_tx_seed = 0x80;
    tx_buf[0] = 0x11;
    tx_buf[1] = 0x22;
    for (uint8_t i = 0; i < 8; ++i) rx_buf[i] = 0;
    st = run_request<C>({client_addr, tx_buf, 2, rx_buf, 3, {}});
    ok = st == i2c_ok && cl_rx_n == 2 && cl_rx[0] == 0x11 && cl_rx[1] == 0x22 &&
         cl_addr_hits == 2;
    for (uint8_t i = 0; i < 3; ++i) ok = ok && rx_buf[i] == static_cast<uint8_t>(0x80 + i);
    if (!ok) {
        print(serial, "    w-then-r: status=", st, " addr_hits=", cl_addr_hits,
              " rx_n=", cl_rx_n, " back ", hex(rx_buf[0]), " ", hex(rx_buf[1]), " ",
              hex(rx_buf[2]), crlf);
    }
    verdict("write-then-read: one tenure, two address packets ", where, ok);

    // probe
    st = run_request<C>({client_addr, nullptr, 0, nullptr, 0, {}});
    verdict("probe: the address alone, ACKed ", where,
            st == i2c_ok && cl_addr_hits == 1 && cl_rx_n == 0);
    st = run_request<C>({absent_addr, nullptr, 0, nullptr, 0, {}});
    verdict("probe of an address nobody holds: nack_addr ", where,
            st == i2c_nack_addr && cl_addr_hits == 0);
}

void tc_loops() {
    print(serial, "c the COMBINED loop (host and client on PA2/PA3) and the DUAL loop "
                  "(client moved to PC2/PC3)", crlf);
    quiesce();

    verdict("host init", host_polled());
    verdict("client init joins the running host", client_polled());
    verdict("both halves are enabled", T::host_enabled() && T::client_enabled());
    verdict("the client is not in dual mode", !T::dual_mode());
    shape_matrix<Client>("(combined)");

    quiesce();
    verdict("host init for the dual loop", host_polled());
    verdict("dual client init on PC2/PC3", DualClient::init(clock, {.address = client_addr}));
    verdict("dual mode is on", T::dual_mode());
    verdict("the dual client's pins are PC2/PC3",
            DualClient::sda_pin().port == 'C' && DualClient::sda_pin().pin == 2);
    shape_matrix<DualClient>("(dual)");
    quiesce();
}

// ---- d: the address-match space ----------------------------------------------

/// Probe an address and say whether anybody ACKed.
template <typename C>
bool acked(uint8_t addr) {
    return run_request<C>({addr, nullptr, 0, nullptr, 0, {}}) == i2c_ok;
}

void td_addresses() {
    print(serial, "d the address-match space, proven by who ACKs", crlf);
    quiesce();
    cl_read_address = true;
    verdict("host init", host_polled());

    verdict("client at 0x42", client_polled(client_addr));
    verdict("0x42 ACKs", acked<Client>(client_addr));
    verdict("the client stored the address it matched", cl_last_addr == client_addr);
    verdict("0x43 NACKs", !acked<Client>(0x43));
    verdict("0x00 (general call) NACKs while the bit is clear", !acked<Client>(0x00));

    verdict("client at 0x42 with the General Call bit",
            client_polled(client_addr, {.general_call = true}));
    verdict("0x42 still ACKs", acked<Client>(client_addr));
    verdict("0x00 (general call) now ACKs", acked<Client>(0x00));
    print(serial, "  the general call arrived as address ", hex(cl_last_addr), crlf);

    // A mask: the SET bits are the address bits the match logic IGNORES.
    verdict("client at 0x40 with mask 0x03 (a range)",
            client_polled(0x40, {.address_mask = 0x03}));
    bool range = true;
    for (uint8_t a = 0x40; a <= 0x43; ++a) range = range && acked<Client>(a);
    verdict("0x40..0x43 all ACK", range);
    verdict("0x44 NACKs", !acked<Client>(0x44));
    verdict("0x3F NACKs", !acked<Client>(0x3F));

    // ADDREN = 1: the same field is a SECOND exact address.
    verdict("client at 0x42 with a second address 0x55",
            client_polled(client_addr, {.address_mask = 0x55, .second_address = true}));
    verdict("0x42 ACKs", acked<Client>(client_addr));
    verdict("0x55 ACKs", acked<Client>(0x55));
    verdict("0x43 NACKs (no masking now)", !acked<Client>(0x43));
    verdict("0x54 NACKs", !acked<Client>(0x54));

    // PMEN answers everything.
    verdict("client promiscuous", client_polled(client_addr, {.promiscuous = true}));
    bool all = true;
    const uint8_t probes[4] = {0x08, 0x33, absent_addr, 0x7E};
    for (uint8_t i = 0; i < 4; ++i) all = all && acked<Client>(probes[i]);
    verdict("0x08, 0x33, 0x77 and 0x7E all ACK", all);
    print(serial, "  the last promiscuous match arrived as address ", hex(cl_last_addr), crlf);
    verdict("the client reports the address it was called on", cl_last_addr == 0x7E);

    quiesce();
}

// ---- e: the chapter's cases, and the NACKs -----------------------------------

void te_cases() {
    print(serial, "e the host cases M1..M3 and the client cases S1..S3, and the two "
                  "NACKs a host can meet", crlf);
    quiesce();
    verdict("host init", host_polled());
    verdict("client init", client_polled());

    // M1: address+W ACKed -> WIF, RXACK = 0, CLKHOLD = 1, bus Owner.
    cl_reset();
    T::address_write(client_addr);
    uint8_t st = pump_to_host_flag<Client>();
    print(serial, "  M1 MSTATUS=", hex(st), crlf);
    verdict("M1: WIF set, ACK received, clock held, bus Owner",
            (st & TWI_WIF_bm) && !(st & TWI_RXACK_bm) && (st & TWI_CLKHOLD_bm) &&
            (st & TWI_BUSSTATE_gm) == TWI_BUSSTATE_OWNER_gc);
    verdict("S1: the client matched the address with DIR = 0",
            cl_addr_hits == 1 && !cl_dir_read);
    T::host_command(TwiHostCmd::stop);
    for (uint16_t k = 0; k < 4000; ++k) service_client<Client>();
    verdict("S3: the client saw the STOP (AP = 0)", cl_stops == 1);

    // M2: address+R ACKed -> the client owns the bus, a byte follows.
    cl_reset();
    cl_tx_seed = 0x9A;
    T::address_read(client_addr);
    st = pump_to_host_flag<Client>();
    print(serial, "  M2 MSTATUS=", hex(st), " first byte=", hex(T::host_read()), crlf);
    verdict("M2: the read raised RIF, not WIF", (st & TWI_RIF_bm) && !(st & TWI_WIF_bm));
    verdict("S2: the client matched the address with DIR = 1",
            cl_addr_hits == 1 && cl_dir_read && cl_sent >= 1);
    T::host_command(TwiHostCmd::stop, TwiAck::nack);
    for (uint16_t k = 0; k < 4000; ++k) service_client<Client>();

    // M3: nobody answers -> WIF and RXACK both set.
    cl_reset();
    T::address_write(absent_addr);
    st = pump_to_host_flag<Client>();
    print(serial, "  M3 MSTATUS=", hex(st), crlf);
    verdict("M3: WIF set and RXACK says NACK", (st & TWI_WIF_bm) && (st & TWI_RXACK_bm));
    verdict("M3: no client matched", cl_addr_hits == 0);
    T::host_command(TwiHostCmd::stop);
    delay_us(clock, 200);

    // The engine's two NACK verdicts.
    verdict("the engine reports nack_addr for an absent device",
            run_request<Client>({absent_addr, tx_buf, 2, nullptr, 0, {}}) == i2c_nack_addr);

    // A client that NACKs the second data byte: the host must call it
    // nack_data, and the wire must carry only what was accepted.
    cl_nack_after = 2;
    for (uint8_t i = 0; i < 5; ++i) tx_buf[i] = static_cast<uint8_t>(0xC0 + i);
    const uint8_t nd = run_request<Client>({client_addr, tx_buf, 5, nullptr, 0, {}});
    print(serial, "  client NACK after 2 bytes: engine says ", nd, " (i2c_nack_data = ",
          i2c_nack_data, "), client took ", cl_rx_n, " bytes", crlf);
    verdict("the engine reports nack_data", nd == i2c_nack_data);
    verdict("the client stopped the write where it NACKed", cl_rx_n == 2);
    verdict("the two bytes it did take are the right ones",
            cl_rx[0] == 0xC0 && cl_rx[1] == 0xC1);
    cl_nack_after = 0xFF;

    // RXACK on the client side: after the host's last read byte.
    cl_reset();
    cl_tx_seed = 0x70;
    (void)run_request<Client>({client_addr, nullptr, 0, rx_buf, 3, {}});
    verdict("RXACK on the client: it saw the host's closing NACK", cl_saw_nack);

    quiesce();
}

// ---- f: the two Smart modes ---------------------------------------------------

uint8_t mcmd_writes = 0;

/// A host-side read done by hand, counting every MCMD strobe. With Smart
/// mode the acknowledge action rides the MDATA read and only the closing
/// STOP is a command.
uint8_t manual_read(uint8_t addr, uint8_t* dst, uint8_t len, bool smart) {
    mcmd_writes = 0;
    T::host_smart(smart);
    cl_reset();
    T::address_read(addr);
    uint8_t st = pump_to_host_flag<Client>();
    if (!(st & TWI_RIF_bm)) return 0xFE;
    for (uint8_t i = 0; i < len; ++i) {
        const bool last = static_cast<uint8_t>(i + 1) >= len;
        if (smart) {
            T::ack_action(last ? TwiAck::nack : TwiAck::ack);
            dst[i] = T::host_read();
        } else {
            dst[i] = T::host_read();
            if (!last) {
                T::host_command(TwiHostCmd::recv_trans, TwiAck::ack);
                ++mcmd_writes;
            }
        }
        if (last) break;
        st = pump_to_host_flag<Client>();
        if (!(st & TWI_RIF_bm)) return 0xFD;
    }
    T::host_command(TwiHostCmd::stop, TwiAck::nack);
    ++mcmd_writes;
    for (uint16_t k = 0; k < 4000; ++k) service_client<Client>();
    T::host_smart(false);
    return i2c_ok;
}

void tf_smart() {
    print(serial, "f Smart mode on both sides: the acknowledge that rides a data "
                  "access, and the flag that clears itself", crlf);
    quiesce();
    verdict("host init", host_polled());
    verdict("client init", client_polled());

    uint8_t plain[4] = {0, 0, 0, 0};
    uint8_t smart[4] = {0, 0, 0, 0};
    cl_tx_seed = 0x60;
    const uint8_t r1 = manual_read(client_addr, plain, 4, false);
    const uint8_t explicit_cmds = mcmd_writes;
    cl_tx_seed = 0x60;
    const uint8_t r2 = manual_read(client_addr, smart, 4, true);
    const uint8_t smart_cmds = mcmd_writes;
    print(serial, "  four bytes read: explicit needs ", explicit_cmds,
          " MCMD strobes, Smart mode ", smart_cmds, crlf);
    bool same = r1 == i2c_ok && r2 == i2c_ok;
    for (uint8_t i = 0; i < 4; ++i) {
        same = same && plain[i] == smart[i] && plain[i] == static_cast<uint8_t>(0x60 + i);
    }
    verdict("both host modes read the same four bytes", same);
    verdict("Smart mode spends three fewer MCMD strobes",
            explicit_cmds == 4 && smart_cmds == 1);

    // The acknowledge action is NOT performed on an MDATA write (29.5.5):
    // a host Smart-mode WRITE behaves exactly like the explicit one.
    T::host_smart(true);
    for (uint8_t i = 0; i < 3; ++i) tx_buf[i] = static_cast<uint8_t>(0xE0 + i);
    const uint8_t w = run_request<Client>({client_addr, tx_buf, 3, nullptr, 0, {}});
    bool wok = w == i2c_ok && cl_rx_n == 3;
    for (uint8_t i = 0; i < 3; ++i) wok = wok && cl_rx[i] == tx_buf[i];
    verdict("Smart mode does not disturb a host WRITE (ACKACT is not sent on a "
            "DATA write)", wok);
    T::host_smart(false);

    // Client Smart mode: touching SDATA clears DIF by itself.
    verdict("client re-init with Smart mode",
            client_polled(client_addr, {.smart = true}));
    verdict("SCTRLA.SMEN is set", T::client_smart());
    cl_reset();
    T::address_write(client_addr);
    uint8_t st = 0;
    bool dif_cleared_by_data = false;
    for (uint32_t i = 0; i < 200'000u && st == 0; ++i) {
        const auto s = Client::isr();
        if (s.address_or_stop() && s.is_address()) {
            ++cl_addr_hits;
            Client::respond(TwiAck::ack);
        }
        st = T::host_status() & (TWI_WIF_bm | TWI_RIF_bm);
    }
    T::host_write(0x5C);                       // one data byte to the client
    for (uint32_t i = 0; i < 200'000u; ++i) {
        if (T::data_flag()) break;
    }
    const bool dif_before = T::data_flag();
    const uint8_t got = T::client_read();      // Smart mode: this clears DIF and ACKs
    dif_cleared_by_data = !T::data_flag();
    print(serial, "  client Smart mode: DIF before the SDATA read = ", dif_before ? 1 : 0,
          ", after = ", T::data_flag() ? 1 : 0, ", byte = ", hex(got), crlf);
    verdict("the client's DIF clears on the SDATA access alone",
            dif_before && dif_cleared_by_data && got == 0x5C);
    T::host_command(TwiHostCmd::stop);
    delay_us(clock, 500);

    // And a whole exchange through the task's Smart-mode path.
    verdict("client Smart mode moves a whole write",
            run_request<Client>({client_addr, tx_buf, 3, nullptr, 0, {}}) == i2c_ok &&
            cl_rx_n == 3 && cl_rx[0] == 0xE0);
    quiesce();
}

// ---- g: Quick Command ---------------------------------------------------------

uint16_t count_edges_of(void (*body)()) {
    SclCount::init(ChScl{});
    SclCount::reset();
    body();
    const uint16_t c = SclCount::count();
    Stopwatch::disable();
    return c;
}

void tg_quick() {
    print(serial, "g Quick Command (QCEN): the address packet IS the transaction, "
                  "counted in SCL edges", crlf);
    quiesce();
    ChScl::source(EvPin<Scl>{});
    verdict("host init", host_polled());
    verdict("client init", client_polled());

    // The instrument's own offset first: an idle bus must count nothing,
    // so every edge below is traffic.
    const uint16_t idle_edges = count_edges_of([] { delay_us(clock, 500); });
    print(serial, "  the counter armed on an idle (high) SCL reads ", idle_edges,
          " edges with no traffic", crlf);
    verdict("the edge counter has no arming offset on this line", idle_edges == 0);

    // The reference, and the model: a transaction of N nine-bit frames
    // shows 9N + 1 rising edges, because the STOP condition itself needs
    // SCL to go back HIGH before SDA is released (figure 29-3).
    tx_buf[0] = 0x5A;
    tx_buf[1] = 0xA5;
    uint16_t edges = count_edges_of([] {
        (void)run_request<Client>({client_addr, tx_buf, 1, nullptr, 0, {}});
    });
    const uint16_t two_frames = edges;
    edges = count_edges_of([] {
        (void)run_request<Client>({client_addr, tx_buf, 2, nullptr, 0, {}});
    });
    const uint16_t three_frames = edges;
    print(serial, "  a one-byte write: ", two_frames, " SCL rising edges; a two-byte "
          "write: ", three_frames, " (model 9N + 1: the STOP's own rise)", crlf);
    verdict("a one-byte write is two 9-bit frames plus the STOP's rise",
            two_frames == 19);
    verdict("a two-byte write is three of them plus the same one rise",
            three_frames == 28);

    // Quick Command, write direction.
    Host::quick_command(true);
    verdict("QCEN is set", T::quick_command());
    uint8_t st = 0;
    edges = count_edges_of([] {
        (void)run_request<Client>({client_addr, tx_buf, 4, nullptr, 0, {}});
    });
    st = Host::status();
    print(serial, "  a quick command (W): ", edges, " SCL rising edges - one 9-bit "
          "frame and the STOP's rise, status ", st, crlf);
    verdict("the quick command is ONE frame however long the data spans are",
            edges == 10 && st == i2c_ok && cl_rx_n == 0);
    verdict("the client still matched the address", cl_addr_hits == 1);

    // Both directions at the resource level: the R/W bit picks the flag.
    cl_reset();
    T::address_write(client_addr);
    uint8_t s = pump_to_host_flag<Client>();
    print(serial, "  QCEN write direction MSTATUS=", hex(s), crlf);
    verdict("QCEN + W raises WIF and not RIF", (s & TWI_WIF_bm) && !(s & TWI_RIF_bm));
    T::host_command(TwiHostCmd::stop);
    delay_us(clock, 500);

    cl_reset();
    T::address_read(client_addr);
    s = pump_to_host_flag<Client>();
    print(serial, "  QCEN read direction MSTATUS=", hex(s), crlf);
    verdict("QCEN + R raises RIF and not WIF", (s & TWI_RIF_bm) && !(s & TWI_WIF_bm));
    T::host_command(TwiHostCmd::stop);
    delay_us(clock, 500);

    Host::quick_command(false);
    verdict("QCEN off again, an ordinary write moves its byte",
            run_request<Client>({client_addr, tx_buf, 1, nullptr, 0, {}}) == i2c_ok &&
            cl_rx_n == 1 && cl_rx[0] == 0x5A);
    quiesce();
}

// ---- h: the bus state machine and the injector --------------------------------

/// Microseconds until BUSSTATE leaves Busy (0xFFFF = never within the
/// budget). The stopwatch is a free-running TCB at CLK_PER.
uint16_t time_to_idle(uint32_t budget_us) {
    Stopwatch::init({.mode = TcbMode::capture, .clock = TcbClock::div1, .compare = 0});
    Stopwatch::count(0);
    const uint32_t spins = budget_us * 6u;      // the loop is well under a microsecond
    for (uint32_t i = 0; i < spins; ++i) {
        if (T::bus_state() == TwiBusState::idle) {
            const uint16_t t = Stopwatch::count();
            Stopwatch::disable();
            return static_cast<uint16_t>(t / 24u);
        }
    }
    Stopwatch::disable();
    return 0xFFFF;
}

void th_busstate() {
    print(serial, "h the bus state machine, driven by a bit-banged injector on the "
                  "route's dual pair (PC2/PC3, Dual mode off)", crlf);
    quiesce();
    report_portb_tap();
    verdict("the injector's pins really are the host's own bus node", node_tied(true));
    verdict("host init", host_polled());
    verdict("client init", client_polled());
    verdict("the bus starts Idle", T::bus_state() == TwiBusState::idle);

    // A foreign START seen by an idle host: Idle -> Busy.
    inject_start_and_leave();
    const bool busy = T::bus_state() == TwiBusState::busy;
    print(serial, "  after an injected START, BUSSTATE = ",
          static_cast<uint8_t>(T::bus_state()), " (3 = Busy)", crlf);
    verdict("an externally generated START makes the bus Busy", busy);
    verdict("no time-out configured: it stays Busy", time_to_idle(1000) == 0xFFFF);

    // A foreign STOP puts it back.
    inject_stop();
    delay_us(clock, 50);
    verdict("an externally generated STOP makes it Idle again",
            T::bus_state() == TwiBusState::idle);

    // The three time-out settings, measured.
    const TwiTimeout tos[3] = {TwiTimeout::us50, TwiTimeout::us100, TwiTimeout::us200};
    const char* names[3] = {"50 us", "100 us", "200 us"};
    const uint16_t nominal[3] = {50, 100, 200};
    for (uint8_t i = 0; i < 3; ++i) {
        T::timeout(tos[i]);
        inject_start_and_leave();
        const bool went_busy = T::bus_state() == TwiBusState::busy;
        const uint16_t t = time_to_idle(4000);
        print(serial, "  TIMEOUT ", names[i], ": Busy -> Idle after ", t, " us", crlf);
        verdict("the inactive-bus time-out returns the bus to Idle at ", names[i],
                went_busy && t != 0xFFFF && t >= nominal[i] / 2 && t <= nominal[i] * 3);
    }
    T::timeout(TwiTimeout::disabled);
    inject_stop();
    T::force_idle();

    // A START directly followed by a STOP is the protocol violation of
    // 29.5.6 (the detector needs CLK_PER >= 4 x SCL, which this suite has).
    T::clear_host_flags(TWI_BUSERR_bm | TWI_WIF_bm | TWI_RIF_bm);
    const bool clean_before = !T::bus_error();
    inject_start_stop();
    delay_us(clock, 200);
    const uint8_t st = T::host_status();
    print(serial, "  after START-then-STOP, MSTATUS=", hex(st), crlf);
    verdict("a START directly followed by a STOP raises BUSERR",
            clean_before && (st & TWI_BUSERR_bm));
    const bool client_saw = T::client_bus_error();
    print(serial, "  the client half saw BUSERR: ", client_saw ? 1 : 0, crlf);
    T::clear_host_flags(TWI_BUSERR_bm | TWI_WIF_bm | TWI_RIF_bm);
    T::clear_client_flags(TWI_BUSERR_bm | TWI_COLL_bm | TWI_APIF_bm | TWI_DIF_bm);
    T::force_idle();
    verdict("BUSERR clears by writing a one to it", !T::bus_error());

    // A host told to start on a Busy bus WAITS (29.3.2.2.3).
    inject_start_and_leave();
    verdict("the bus is Busy again", T::bus_state() == TwiBusState::busy);
    cl_reset();
    T::address_write(client_addr);
    bool waited = true;
    for (uint16_t i = 0; i < 2000; ++i) {
        delay_us(clock, 1);
        service_client<Client>();          // the client must be free to answer
        if (T::write_flag()) { waited = false; break; }
    }
    verdict("the host held its START while the bus was Busy (2 ms)", waited);
    verdict("nothing reached the client", cl_addr_hits == 0);
    // Releasing THAT bus with a hand-made STOP is itself a violation: the
    // injected START was followed by a clock, so the STOP lands inside a
    // byte. The host calls it BUSERR and drops the transaction it was
    // holding - measured, not assumed.
    inject_stop();
    const uint8_t imm = T::host_status();
    print(serial, "  a STOP injected mid-byte: MSTATUS=", hex(imm), ", client matches=",
          cl_addr_hits, crlf);
    verdict("a STOP inside a byte is a violation: BUSERR, and the address still has "
            "not gone out", (imm & TWI_BUSERR_bm) && cl_addr_hits == 0);

    // The START the host has been holding all this time is still
    // pending: clearing BUSERR and declaring the bus idle lets it go.
    T::clear_host_flags(TWI_BUSERR_bm | TWI_WIF_bm | TWI_RIF_bm);
    cl_reset();
    T::force_idle();
    const uint8_t after = pump_to_host_flag<Client>();
    print(serial, "  once the bus is Idle again: MSTATUS=", hex(after),
          ", client matches=", cl_addr_hits, crlf);
    verdict("the held transaction survives the wait and goes out on the idle bus",
            (after & TWI_WIF_bm) && !(after & TWI_RXACK_bm) && cl_addr_hits == 1);
    T::host_command(TwiHostCmd::stop);
    for (uint16_t k = 0; k < 4000; ++k) service_client<Client>();

    // The other legal way out of a bus somebody else left Busy: the
    // inactive-bus time-out (29.3.2.2.2) releases the held transaction
    // without any software help at all.
    Host::recover();
    delay_us(clock, 500);
    inject_start_and_byte();
    verdict("the bus is Busy for the third time", T::bus_state() == TwiBusState::busy);
    cl_reset();
    T::address_write(client_addr);
    T::timeout(TwiTimeout::us200);
    const uint8_t after2 = pump_to_host_flag<Client>();
    print(serial, "  after the 200 us time-out: MSTATUS=", hex(after2),
          ", client matches=", cl_addr_hits, crlf);
    verdict("the time-out released the bus and the held address went out",
            (after2 & TWI_WIF_bm) && !(after2 & TWI_RXACK_bm) && cl_addr_hits == 1);
    T::host_command(TwiHostCmd::stop);
    T::timeout(TwiTimeout::disabled);
    for (uint16_t k = 0; k < 4000; ++k) service_client<Client>();

    // The recovery verb the errata prescribe instead of FLUSH.
    Host::recover();
    verdict("recover() (an ENABLE cycle, not FLUSH) leaves the bus Idle",
            T::bus_state() == TwiBusState::idle && T::host_enabled());
    verdict("and the bus still works after it",
            run_request<Client>({client_addr, tx_buf, 1, nullptr, 0, {}}) == i2c_ok);

    quiesce();
    verdict("the injector pins are parked as inputs with OUT = 0 (errata hygiene for "
            "the next owner of the dual pair)",
            !InjSda::is_output() && !InjScl::is_output() &&
            (PORTC.OUT & (PIN2_bm | PIN3_bm)) == 0 &&
            (PORTB.OUT & (PIN2_bm | PIN3_bm)) == 0);
}

// ---- i: the two ISR bodies ----------------------------------------------------

volatile bool isr_mode = false;
volatile bool bus_mode = false;    ///< the completion also goes to the arbiter (test j)
volatile bool host_done = false;
volatile uint16_t host_ints = 0;

void ti_interrupts() {
    print(serial, "i the two vectors: one host interrupt per byte, the client's "
                  "APIF/DIF discipline, no storms", crlf);
    quiesce();
    verdict("host init", host_polled());
    verdict("client init", Client::init(clock, {.address = client_addr,
                                                .stop_interrupt = true,
                                                .data_interrupt = true,
                                                .address_interrupt = true}));
    cl_reset();
    cli();
    host_done = false;
    host_ints = 0;
    isr_mode = true;
    T::enable_read_interrupt(true);
    T::enable_write_interrupt(true);
    sei();

    for (uint8_t i = 0; i < 6; ++i) tx_buf[i] = static_cast<uint8_t>(0x21 + i);
    bool started = !Host::start({client_addr, tx_buf, 6, nullptr, 0, {}});
    for (uint32_t i = 0; i < 400'000u && !host_done; ++i) {}
    delay_us(clock, 2000);
    const uint16_t hi = host_ints;
    const uint16_t ci = cl_ints;
    const uint16_t hits = cl_addr_hits;
    const uint16_t stops = cl_stops;
    const uint16_t data = cl_data_events;
    const uint8_t rxn = cl_rx_n;
    isr_mode = false;
    T::enable_read_interrupt(false);
    T::enable_write_interrupt(false);

    print(serial, "  a six-byte write: ", hi, " host interrupts, ", ci,
          " client interrupts (", hits, " address, ", data, " data, ", stops, " stop)", crlf);
    verdict("the transaction ran on the vectors", started && host_done &&
            Host::status() == i2c_ok);
    verdict("one host interrupt per byte plus the address (7)", hi == 7);
    verdict("the client got address + six data + stop (8)", ci == 8 && hits == 1 &&
            data == 6 && stops == 1);
    bool ok = rxn == 6;
    for (uint8_t i = 0; i < 6; ++i) ok = ok && cl_rx[i] == tx_buf[i];
    verdict("and the bytes are exact", ok);

    // No storm: with the transaction over and the flags served, the
    // vectors must be silent.
    const uint16_t h0 = host_ints, c0 = cl_ints;
    cli();
    T::enable_read_interrupt(true);
    T::enable_write_interrupt(true);
    sei();
    delay_us(clock, 5000);
    T::enable_read_interrupt(false);
    T::enable_write_interrupt(false);
    print(serial, "  5 ms idle with both enables on: ", host_ints - h0, " host, ",
          cl_ints - c0, " client interrupts", crlf);
    verdict("an idle bus raises no interrupt at all",
            host_ints == h0 && cl_ints == c0);

    quiesce();
}

// ---- j: a rebase under traffic, and the arbiter -------------------------------

/// A tiny AO: somewhere for the arbiter's reply to land.
struct Sink {
    using Event = I2cDone;
    static inline EventQueue<Event, 4, P> queue;
};

bool rebase_step(uint32_t hz, const char* what) {
    for (uint8_t i = 0; i < 4; ++i) tx_buf[i] = static_cast<uint8_t>(hz / 1'000'000u + i);
    const uint8_t st = run_request<Client>({client_addr, tx_buf, 4, nullptr, 0, {}});
    bool ok = st == i2c_ok && cl_rx_n == 4;
    for (uint8_t i = 0; i < 4; ++i) ok = ok && cl_rx[i] == tx_buf[i];
    verdict("the exchange is exact ", what, ok);

    const uint8_t baud = T::baud();
    ChScl::source(EvPin<Scl>{});
    meter_mode = 0;
    SclFreq::init(DynClock{}, ChScl{}, TcbClock::div1);
    meter_reset();
    nack_traffic(6);
    meter_stop();
    const uint16_t period = meter_min;
    const uint16_t floor_ticks = static_cast<uint16_t>(twi_period_ticks(hz, baud, 0));
    const uint16_t budget = static_cast<uint16_t>(
        twi_period_ticks(hz, baud, twi_rise_budget_ns(TwiSpeed::standard_100k)));
    print(serial, "  ", what, ": CLK_PER = ", hz, " Hz, MBAUD = ", baud, ", SCL period ",
          period, " ticks = ", period ? hz / period : 0, " Hz (window ", floor_ticks,
          "..", budget, "), actual_scl_hz(0) = ", Host::actual_scl_hz(0), crlf);
    const bool tracked = period >= floor_ticks && period <= budget;
    verdict("MBAUD was re-derived and SCL stayed in the window ", what, tracked);
    return ok && tracked;
}

void tj_rebase() {
    print(serial, "j 24 -> 12 -> 24 MHz under traffic, then the I2cBus/BusMaster stack "
                  "over the engine", crlf);
    quiesce();
    verdict("DynamicClock init (boot = the crystal)", DynClock::init());
    verdict("host init on the dynamic clock", Host::init(DynClock{}));
    T::enable_read_interrupt(false);
    T::enable_write_interrupt(false);
    verdict("client init on the dynamic clock",
            Client::init(DynClock{}, {.address = client_addr}));

    (void)rebase_step(24'000'000u, "at 24 MHz");
    verdict("switch to 12 MHz", DynClock::set(12'000'000u));
    (void)rebase_step(12'000'000u, "at 12 MHz");
    verdict("back to 24 MHz", DynClock::set(24'000'000u));
    (void)rebase_step(24'000'000u, "back at 24 MHz");

    // The arbiter, on the real bus, against this instance's own client.
    Bus::init();
    cl_reset();
    cl_tx_seed = 0x33;
    for (uint8_t i = 0; i < 8; ++i) rx_buf[i] = 0;
    cli();
    host_done = false;
    isr_mode = true;
    bus_mode = true;
    T::enable_read_interrupt(true);
    T::enable_write_interrupt(true);
    sei();
    tx_buf[0] = 0x7B;
    post<Bus>(Host::Request{client_addr, tx_buf, 1, rx_buf, 2, reply_to<Sink, I2cDone>()});
    bool replied = false;
    uint8_t status = 0xFF;
    for (uint32_t i = 0; i < 800'000u && !replied; ++i) {
        service_client<Client>();
        if (const auto e = Bus::queue.pop()) Bus::dispatch(*e);
        if (const auto r = Sink::queue.pop()) {
            replied = true;
            status = r->status;
        }
    }
    isr_mode = false;
    bus_mode = false;
    T::enable_read_interrupt(false);
    T::enable_write_interrupt(false);
    print(serial, "  the arbitrated write-then-read replied ", status, " (i2c_ok = ",
          i2c_ok, "), read back ", hex(rx_buf[0]), " ", hex(rx_buf[1]), crlf);
    verdict("one arbitrated request end to end through I2cBus",
            replied && status == i2c_ok && cl_rx_n == 1 && cl_rx[0] == 0x7B &&
            rx_buf[0] == 0x33 && rx_buf[1] == 0x34);
    verdict("the arbiter rejected nothing", Bus::rejected_count() == 0);

    quiesce();
}

// ==== TWO BOARDS (k..s): board B runs twi_peer ================================
//
// The command channel is the bus itself: a WRITE tenure to
// twilink::command_addr carries one command frame, a READ tenure of
// twilink::response_bytes collects the answer. Ack before act - the peer
// prepares its acknowledgement and enters the commanded action only when
// that read has completed, so the DUT knows it is armed before it puts a
// measured edge on the wire.

using twilink::Op;

uint8_t link_tx[twilink::max_payload + 4];
uint8_t link_rx[twilink::response_bytes];
twilink::Decoder link_dec;
twilink::Frame link_ans{};
uint16_t link_naks = 0;

void wait_ms(uint16_t ms) { delay_us(clock, static_cast<uint32_t>(ms) * 1000u); }

/// One READ tenure, decoded into `link_ans`.
bool link_collect() {
    for (uint8_t i = 0; i < twilink::response_bytes; ++i) link_rx[i] = 0;
    if (run_request<Client>({twilink::command_addr, nullptr, 0, link_rx,
                             twilink::response_bytes, {}}) != i2c_ok) {
        return false;
    }
    link_dec.reset();
    for (uint8_t i = 0; i < twilink::response_bytes; ++i) {
        if (link_dec.feed(link_rx[i]) == twilink::Decoder::Result::frame) {
            link_ans = link_dec.frame();
            return true;
        }
    }
    return false;
}

/// One command: the frame out, the answer back. RAW - no retry, no
/// verdict on what came back (test k needs the nak as an answer).
bool link_once(Op op, const uint8_t* p, uint8_t len) {
    uint8_t n = 0;
    twilink::write_frame(
        [&](uint8_t b) {
            if (n < sizeof link_tx) link_tx[n++] = b;
        },
        op, p, len);
    if (run_request<Client>({twilink::command_addr, link_tx, n, nullptr, 0, {}}) != i2c_ok) {
        return false;
    }
    wait_ms(1);
    return link_collect();
}

/// A command that must be acknowledged, retried three times (a peer
/// still finishing a previous action's deadline is the normal reason a
/// first attempt is not heard).
bool link_cmd(Op op, const uint8_t* p = nullptr, uint8_t len = 0) {
    for (uint8_t attempt = 0; attempt < 3; ++attempt) {
        if (link_once(op, p, len) && link_ans.op == Op::ack &&
            link_ans.len >= 1 && link_ans.data[0] == twilink::byte_of(op)) {
            return true;
        }
        wait_ms(20);
    }
    return false;
}

/// Command an action and give the peer its arming margin.
bool peer_action(Op op, const twilink::Params& a) {
    uint8_t p[twilink::params_size];
    twilink::put_params(p, a);
    if (!link_cmd(op, p, twilink::params_size)) return false;
    wait_ms(twilink::arm_ms);
    return true;
}

/// What the peer saw. The ack comes first, the data frame with a second
/// read.
bool peer_report(twilink::Report& out) {
    if (!link_cmd(Op::report)) return false;
    wait_ms(2);
    if (!link_collect() || link_ans.op != Op::report_data) return false;
    out = twilink::get_report(link_ans.data);
    return true;
}

void print_report(const twilink::Report& r) {
    print(serial, "    peer: op=", hex(r.op), " count=", r.count, " (rx ", r.aux0,
          " tx ", r.aux1, ") hits=", r.addr_hits, " stops=", r.stops, " flags=",
          hex(r.flags), " sum=", hex(r.sum), " first=", hex(r.first), " lastaddr=",
          hex(r.last_addr), " SSTATUS=", hex(r.sstatus), " MSTATUS=", hex(r.mstatus),
          " ms=", r.ms, crlf);
}

/// The peer back in command mode after an action that ended early.
void peer_settle() { wait_ms(12); }

/// The two-board standing configuration: this board's host, polled, at
/// 100 kHz, with the command channel proven alive.
bool link_up() {
    if (!host_polled()) return false;
    return link_cmd(Op::ping);
}

// ---- k: bring-up over the wire -----------------------------------------------

void tk_bringup() {
    print(serial, "k the command channel: ping, ident, a corrupted frame, a truncated "
                  "one, and the retry", crlf);
    quiesce();
    verdict("host init", host_polled());
    verdict("the peer answers a ping", link_cmd(Op::ping));

    verdict("ident acknowledged", link_cmd(Op::ident));
    bool got = link_collect() && link_ans.op == Op::ident_data;
    twilink::Ident d{};
    if (got) d = twilink::get_ident(link_ans.data);
    print(serial, "  peer ident: label=", d.label[0], d.label[1], d.label[2], d.label[3],
          d.label[4], d.label[5], " xtal=", d.xtal, " sanity=", hex(d.sanity),
          " fw=", hex(d.version), crlf);
    verdict("the ident frame came back", got);
    verdict("its sanity byte says twi_peer", d.sanity == twilink::ident_sanity);
    verdict("the peer names itself brio-b",
            d.label[0] == 'b' && d.label[1] == 'r' && d.label[2] == 'i' &&
            d.label[3] == 'o' && d.label[4] == '-' && d.label[5] == 'b');
    verdict("the peer's 24 MHz crystal started (its byte turnaround is the "
            "crystal's)", d.xtal == 1);

    // A frame whose checksum does not match must be NAKed by op, and the
    // channel must be usable straight afterwards.
    uint8_t n = 0;
    twilink::write_frame(
        [&](uint8_t b) {
            if (n < sizeof link_tx) link_tx[n++] = b;
        },
        Op::ping, nullptr, 0);
    link_tx[n - 1] ^= 0xFFu;                  // the checksum, ruined
    bool naked = false;
    if (run_request<Client>({twilink::command_addr, link_tx, n, nullptr, 0, {}}) == i2c_ok) {
        wait_ms(1);
        naked = link_collect() && link_ans.op == Op::nak && link_ans.len >= 1 &&
                link_ans.data[0] == twilink::byte_of(Op::ping);
    }
    verdict("a corrupted frame is NAKed, naming the op it claimed", naked);
    verdict("and the channel still works", link_cmd(Op::ping));

    // Half a frame, then a STOP: the peer resets its decoder at every
    // address packet, so the next command cannot be eaten by it.
    link_tx[0] = 0xC3;
    link_tx[1] = twilink::byte_of(Op::ping);
    link_tx[2] = 0;
    const uint8_t st = run_request<Client>({twilink::command_addr, link_tx, 3,
                                            nullptr, 0, {}});
    verdict("a truncated frame is accepted on the wire", st == i2c_ok);
    verdict("and does not wedge the channel: the next ping is answered",
            link_cmd(Op::ping));

    // An op the peer does not have: NAKed by name, not obeyed.
    uint8_t p[1] = {0};
    const bool answered = link_once(static_cast<Op>(0x7Eu), p, 1);
    verdict("an unknown op is NAKed by name, never acknowledged",
            answered && link_ans.op == Op::nak && link_ans.len >= 1 &&
            link_ans.data[0] == 0x7Eu);
    verdict("and the channel is alive after it", link_cmd(Op::ping));

    quiesce();
}

// ---- l: clock stretching by a foreign client ---------------------------------

struct Stretch {
    uint32_t ticks = 0;      ///< the whole tenure, CLK_PER ticks
    uint16_t low_max = 0;    ///< the longest SCL low, CLK_PER ticks
    uint16_t low_min = 0;
    uint8_t status = 0xFE;
    bool clkhold = false;    ///< the HOST held the clock while its own flag stood
    bool owner = false;
};

constexpr uint8_t stretch_bytes = 6;

Stretch stretch_round(uint16_t hold_us) {
    Stretch res{};
    twilink::Params a{};
    a.count = stretch_bytes;
    a.ms = 400;
    a.addr = twilink::command_addr;
    a.hold_us = hold_us;
    a.flags = twilink::flag_stop_interrupt;
    if (!peer_action(Op::serve, a)) return res;

    for (uint8_t i = 0; i < stretch_bytes; ++i) tx_buf[i] = static_cast<uint8_t>(0x50 + i);
    meter_mode = 1;
    SclLow::init(clock, ChScl{}, TcbClock::div1, true);
    meter_reset();
    Watch::reset();
    Host::start({twilink::command_addr, tx_buf, stretch_bytes, nullptr, 0, {}});
    for (uint32_t i = 0; i < 4'000'000u; ++i) {
        const uint8_t s = T::host_status();
        if (s & (TWI_RIF_bm | TWI_WIF_bm)) {
            if (s & TWI_CLKHOLD_bm) res.clkhold = true;
            if ((s & TWI_BUSSTATE_gm) == TWI_BUSSTATE_OWNER_gc) res.owner = true;
        }
        if ((s & (TWI_RIF_bm | TWI_WIF_bm | TWI_ARBLOST_bm | TWI_BUSERR_bm)) &&
            Host::isr()) {
            break;
        }
    }
    res.ticks = Watch::read();
    meter_stop();
    res.low_max = meter_max;
    res.low_min = meter_min;
    res.status = Host::status();
    peer_settle();
    return res;
}

void tl_stretch() {
    print(serial, "l clock stretching: the peer holds DIF for a commanded time per "
                  "byte, and the wire shows it", crlf);
    quiesce();
    verdict("link up", link_up());
    ChScl::source(EvPin<Scl>{});
    Watch::init(TcbClock::div1, ChCarry{}, ChSnap{});

    const uint16_t tiers[4] = {0, 100, 1000, 0};
    const char* names[4] = {"no hold", "100 us/byte", "1 ms/byte", "no hold again"};
    Stretch r[4];
    for (uint8_t i = 0; i < 4; ++i) {
        r[i] = stretch_round(tiers[i]);
        print(serial, "  ", names[i], ": ", r[i].ticks, " ticks = ", r[i].ticks / 24u,
              " us for ", stretch_bytes, " bytes; SCL low min ", r[i].low_min,
              " max ", r[i].low_max, " ticks (", r[i].low_max / 24u,
              " us); status ", r[i].status, crlf);
        verdict("the stretched write still lands exactly ", names[i],
                r[i].status == i2c_ok);
    }

    // The elongation is the peer's hold times the number of data bytes:
    // the address packet is answered from APIF, which this action does
    // not hold.
    const uint32_t base = r[0].ticks;
    for (uint8_t i = 1; i <= 2; ++i) {
        const uint32_t want = static_cast<uint32_t>(tiers[i]) * stretch_bytes * 24u;
        const uint32_t got = r[i].ticks > base ? r[i].ticks - base : 0;
        print(serial, "  elongation at ", names[i], ": ", got, " ticks, model ", want,
              " (", stretch_bytes, " bytes x ", tiers[i], " us)", crlf);
        verdict("the tenure grows by count x hold ", names[i],
                got > want - want / 5u && got < want + want / 5u + 3000u);
        verdict("and the longest SCL low is that hold, not the divider's ", names[i],
                r[i].low_max > static_cast<uint16_t>(tiers[i] * 20u));
    }

    verdict("the HOST holds the clock while its own flag stands (CLKHOLD)",
            r[0].clkhold && r[2].clkhold);
    verdict("the bus reads Owner throughout", r[0].owner && r[2].owner);
    print(serial, "  the peer's own software turnaround, unstretched: SCL low up to ",
          r[0].low_max, " ticks = ", r[0].low_max * 1000ul / 24ul, " ns (the register's "
          "own low time is ", twi_low_ticks(T::baud()), " ticks)", crlf);
    verdict("an unstretched exchange right after the 1 ms one is back to nominal",
            r[3].low_max < r[2].low_max / 4u && r[3].ticks < base * 2u);

    twilink::Report rep{};
    if (peer_report(rep)) print_report(rep);
    verdict("the peer took all six bytes", rep.aux0 == stretch_bytes);
    quiesce();
}

// ---- m: NACK injection -------------------------------------------------------

void tm_nacks() {
    print(serial, "m the two NACKs a host can meet, injected by a board that is alive "
                  "and listening", crlf);
    quiesce();
    verdict("link up", link_up());

    // (1) ADDRESS NACK: the peer parks its client where nobody sends.
    twilink::Params a{};
    a.ms = 250;
    a.flags = twilink::flag_deaf | twilink::flag_stop_interrupt;
    verdict("the peer commanded deaf (its address parked elsewhere)",
            peer_action(Op::serve, a));
    last_mstatus = 0;
    uint8_t st = run_request<Client>({twilink::command_addr, tx_buf, 2, nullptr, 0, {}});
    print(serial, "  a write to a deaf board: engine says ", st, " (i2c_nack_addr = ",
          i2c_nack_addr, "), MSTATUS at the failure = ", hex(last_mstatus), crlf);
    verdict("the engine reports nack_addr", st == i2c_nack_addr);
    verdict("MSTATUS shows WIF with RXACK: nobody acknowledged the address",
            (last_mstatus & TWI_WIF_bm) && (last_mstatus & TWI_RXACK_bm));
    wait_ms(280);
    twilink::Report rep{};
    if (peer_report(rep)) print_report(rep);
    verdict("the peer matched nothing at all while it was deaf", rep.addr_hits == 0);
    verdict("and it is answering its own address again", link_cmd(Op::ping));

    // (2) DATA NACK at a commanded byte.
    const uint8_t nack_at = 3;
    a = twilink::Params{};
    a.ms = 300;
    a.addr = twilink::command_addr;
    a.count = nack_at;
    a.nack_at = nack_at;
    a.flags = twilink::flag_stop_interrupt;
    verdict("the peer armed to NACK its third byte", peer_action(Op::serve, a));
    for (uint8_t i = 0; i < 6; ++i) tx_buf[i] = static_cast<uint8_t>(0x91 + i);
    last_mstatus = 0;
    st = run_request<Client>({twilink::command_addr, tx_buf, 6, nullptr, 0, {}});
    print(serial, "  a six-byte write into that: engine says ", st, " (i2c_nack_data = ",
          i2c_nack_data, "), MSTATUS = ", hex(last_mstatus), crlf);
    verdict("the engine reports nack_data", st == i2c_nack_data);
    verdict("MSTATUS shows RXACK on a DATA byte, not on the address",
            (last_mstatus & TWI_WIF_bm) && (last_mstatus & TWI_RXACK_bm));
    peer_settle();
    if (peer_report(rep)) print_report(rep);
    verdict("the peer accepted exactly the three bytes it acknowledged",
            rep.aux0 == nack_at);
    verdict("and says it NACKed on purpose", (rep.flags & twilink::report_nacked) != 0);
    verdict("the bytes it kept are the ones that were sent",
            rep.first == 0x91 &&
            rep.sum == static_cast<uint16_t>(0x91 + 0x92 + 0x93));

    // (3) The host's own closing NACK, seen from the client side.
    a = twilink::Params{};
    a.ms = 300;
    a.addr = twilink::command_addr;
    a.count = 4;
    a.seed = 0x64;
    a.flags = twilink::flag_stop_interrupt;
    verdict("the peer armed to serve four bytes", peer_action(Op::serve, a));
    for (uint8_t i = 0; i < 8; ++i) rx_buf[i] = 0;
    st = run_request<Client>({twilink::command_addr, nullptr, 0, rx_buf, 4, {}});
    bool exact = st == i2c_ok;
    for (uint8_t i = 0; i < 4; ++i) exact = exact && rx_buf[i] == static_cast<uint8_t>(0x64 + i);
    verdict("four bytes read back exactly from the peer", exact);
    peer_settle();
    if (peer_report(rep)) print_report(rep);
    print(serial, "  the peer's SSTATUS at the host's closing NACK = ",
          hex(rep.mstatus), " (RXACK = ", (rep.mstatus & TWI_RXACK_bm) ? 1 : 0, ")", crlf);
    verdict("the client saw the host's closing NACK as RXACK",
            (rep.mstatus & TWI_RXACK_bm) != 0);

    quiesce();
}

// ---- n: multi-host arbitration -----------------------------------------------

/// One arbitration round, made DETERMINISTIC by the silicon itself.
///
/// A software rendezvous cannot put two STARTs on a bus inside the
/// hundred nanoseconds a genuine race needs. The hardware can: a host
/// told to start while the bus is BUSY holds its START and releases it
/// by itself the moment the bus goes Idle (29.3.2.2.3). So the injector
/// makes the bus Busy with a foreign START and a whole frame, BOTH
/// boards arm a START into that, and the injected STOP releases the two
/// of them on the same edge. Which one wins is then pure wired-AND: the
/// smaller address byte survives the first differing bit - so aiming the
/// two hosts at addresses in either order chooses the loser.
struct ArbRound {
    uint8_t status = 0xFE;      ///< this board's engine verdict
    uint8_t mstatus = 0;        ///< MSTATUS on the pass that ended it
    uint8_t state = 0;          ///< BUSSTATE the instant the round ended
    uint8_t rx_n = 0;           ///< bytes THIS board's client received
    uint8_t rx0 = 0;
    uint16_t hits = 0;
};

ArbRound arb_round(uint8_t a_target, uint16_t t_us) {
    ArbRound r{};
    cl_reset();
    inject_start_and_byte();               // the bus is Busy for BOTH boards
    if (T::bus_state() != TwiBusState::busy) {
        inject_stop();
        return r;
    }
    T::clear_host_flags(TWI_ARBLOST_bm | TWI_BUSERR_bm | TWI_WIF_bm | TWI_RIF_bm);
    for (uint8_t i = 0; i < 5; ++i) tx_buf[i] = static_cast<uint8_t>(0xD0 + i);
    (void)Host::start({a_target, tx_buf, 5, nullptr, 0, {}});   // held: the bus is Busy
    delay_us(clock, t_us);                 // the peer arms into the same Busy bus
    inject_stop();                         // both held STARTs go out together

    for (uint32_t i = 0; i < 1'000'000u; ++i) {
        service_client<Client>();
        const uint8_t s = T::host_status();
        if (!(s & (TWI_RIF_bm | TWI_WIF_bm | TWI_ARBLOST_bm | TWI_BUSERR_bm))) continue;
        r.mstatus = s;
        r.state = static_cast<uint8_t>(s & TWI_BUSSTATE_gm);
        if (Host::isr()) break;
    }
    r.status = Host::status();
    // Whoever won, the loser's client is about to be written to.
    for (uint32_t k = 0; k < 200'000u; ++k) service_client<Client>();
    r.rx_n = cl_rx_n;
    r.rx0 = cl_rx[0];
    r.hits = cl_addr_hits;
    return r;
}

void tn_arbitration() {
    print(serial, "n multi-host arbitration: both boards combined, both STARTs released "
                  "by the same edge", crlf);
    quiesce();
    verdict("link up", link_up());
    verdict("this board's client joins its host (combined)",
            client_polled(twilink::dut_addr));

    const uint16_t offsets[5] = {60, 120, 250, 500, 900};
    uint8_t a_lost = 0, b_lost = 0, rounds = 0, a_won_clean = 0;

    for (uint8_t dir = 0; dir < 2; ++dir) {
        // dir 0: this board aims at the HIGHER address and must lose.
        // dir 1: it aims at the LOWER one and must win.
        const uint8_t peer_client = dir == 0 ? twilink::command_addr : twilink::low_addr;
        const uint8_t a_target = peer_client;
        print(serial, "  round set ", dir == 0 ? "A->0x6B vs B->0x2C (A must lose)"
                                               : "A->0x11 vs B->0x2C (B must lose)", crlf);
        for (uint8_t k = 0; k < 5; ++k) {
            twilink::Params a{};
            a.ms = 600;
            a.addr = peer_client;
            a.target = twilink::dut_addr;
            a.count = 4;
            a.seed = 0x40;
            a.flags = twilink::flag_stop_interrupt;
            a.aux16 = 0;
            if (!peer_action(Op::arb, a)) {
                verdict("the peer accepted the arbitration command", false);
                continue;
            }
            const ArbRound r = arb_round(a_target, offsets[k]);
            twilink::Report rep{};
            const bool got = peer_report(rep);
            ++rounds;
            const bool a_arb = r.status == i2c_arb_lost;
            const bool b_arb = got && (rep.flags & twilink::report_arblost) != 0;
            if (a_arb) ++a_lost;
            if (b_arb) ++b_lost;
            if (!a_arb && r.status == i2c_ok) ++a_won_clean;
            print(serial, "    T=", offsets[k], " us: A status=", r.status,
                  " MSTATUS=", hex(r.mstatus), " state=", r.state, " A rx=", r.rx_n,
                  " (", hex(r.rx0), ") | B flags=", hex(rep.flags), " rx=", rep.aux0,
                  " sent=", rep.aux1, " hits=", rep.addr_hits, crlf);

            if (dir == 0) {
                verdict("A lost the bus: the engine reports arb_lost", a_arb);
                verdict("A's MSTATUS carries ARBLOST with WIF",
                        (r.mstatus & TWI_ARBLOST_bm) && (r.mstatus & TWI_WIF_bm));
                verdict("A's bus went Busy: it belongs to the winner now",
                        r.state == TWI_BUSSTATE_BUSY_gc);
                verdict("A's transaction did NOT land: the peer's client saw nothing",
                        got && rep.addr_hits == 0);
                verdict("the winner's four bytes reached A's client intact",
                        r.rx_n == 4 && r.rx0 == 0x40 && r.hits == 1);
                verdict("and the peer says its own host completed",
                        got && (rep.flags & twilink::report_host_ok) != 0);
            } else {
                verdict("A won the bus: its write completed", r.status == i2c_ok);
                verdict("B lost it: the peer reports ARBLOST", b_arb);
                verdict("nothing of B's write reached A's client", r.rx_n == 0);
                verdict("A's five bytes reached the peer's client intact",
                        got && rep.aux0 == 5 && rep.first == 0xD0 && rep.addr_hits == 1);
            }
            // The loser's retry on an Idle bus must simply work.
            wait_ms(5);
            const uint8_t retry = run_request<Client>({twilink::command_addr, tx_buf, 2,
                                                       nullptr, 0, {}});
            verdict("the loser's retry on the idle bus succeeds", retry == i2c_ok);
            peer_settle();
        }
    }
    print(serial, "  tally over ", rounds, " rounds: A lost ", a_lost, ", B lost ",
          b_lost, ", A won cleanly ", a_won_clean, crlf);
    verdict("ARBLOST was positively observed on BOTH boards", a_lost >= 5 && b_lost >= 5);
    quiesce();
}

// ---- o: the client collision, case S4 ----------------------------------------

void to_collision() {
    print(serial, "o case S4: two clients on ONE address, one host reading - the wire is "
                  "the AND of the two answers", crlf);
    quiesce();
    verdict("link up", link_up());
    verdict("this board's client takes the shared address",
            client_polled(twilink::shared_addr));
    cl_fixed_tx = true;

    // The client whose byte has a HIGH bit where the other's is low
    // cannot drive it: that one collides, the other does not.
    const uint8_t mine[2] = {0xFF, 0x55};
    const uint8_t theirs[2] = {0x55, 0xFF};
    for (uint8_t round = 0; round < 2; ++round) {
        cl_tx_seed = mine[round];
        T::clear_client_flags(TWI_COLL_bm);
        twilink::Params a{};
        a.ms = 300;
        a.addr = twilink::shared_addr;
        a.seed = theirs[round];
        a.count = 4;
        a.flags = twilink::flag_stop_interrupt;
        verdict("the peer's client joined the shared address", peer_action(Op::coll, a));

        for (uint8_t i = 0; i < 8; ++i) rx_buf[i] = 0;
        const uint8_t st = run_request<Client>({twilink::shared_addr, nullptr, 0,
                                                rx_buf, 4, {}});
        const bool a_coll = T::collision();
        const uint16_t served = cl_sent;
        const uint16_t a_hits = cl_addr_hits;   // the report command resets these
        twilink::Report rep{};
        peer_settle();
        const bool got = peer_report(rep);
        const bool b_coll = got && (rep.flags & twilink::report_coll) != 0;
        print(serial, "  A serves ", hex(mine[round]), ", B serves ", hex(theirs[round]),
              ": host read ", hex(rx_buf[0]), " ", hex(rx_buf[1]), " ", hex(rx_buf[2]),
              " ", hex(rx_buf[3]), " (status ", st, "); A COLL=", a_coll ? 1 : 0,
              " matched ", a_hits, " served ", served, " | B COLL=", b_coll ? 1 : 0,
              " matched ", rep.addr_hits, " served ", rep.aux0, crlf);
        verdict("both clients acknowledged the address", st == i2c_ok && a_hits == 1 &&
                got && rep.addr_hits >= 1);
        verdict("the host read the wired-AND of the two answers",
                rx_buf[0] == static_cast<uint8_t>(mine[round] & theirs[round]));
        if (mine[round] == 0xFF) {
            verdict("A raised COLL: it could not put a high bit against B's low", a_coll);
            verdict("B did not: every bit it sent survived", !b_coll);
        } else {
            verdict("B raised COLL: it could not put a high bit against A's low", b_coll);
            verdict("A did not: every bit it sent survived", !a_coll);
        }
    }

    // COLL clears on any Start condition (29.5.12) - so a collision that
    // is not read before the next START is gone.
    cl_tx_seed = 0xFF;
    T::clear_client_flags(TWI_COLL_bm);
    twilink::Params a{};
    a.ms = 300;
    a.addr = twilink::shared_addr;
    a.seed = 0x55;
    a.count = 4;
    a.flags = twilink::flag_stop_interrupt;
    verdict("one more collision armed", peer_action(Op::coll, a));
    (void)run_request<Client>({twilink::shared_addr, nullptr, 0, rx_buf, 4, {}});
    const bool coll_now = T::collision();
    (void)run_request<Client>({twilink::shared_addr, nullptr, 0, rx_buf, 2, {}});
    const bool coll_after = T::collision();
    print(serial, "  COLL after the colliding read = ", coll_now ? 1 : 0,
          ", after the NEXT Start condition = ", coll_after ? 1 : 0, crlf);
    verdict("COLL stood after the collision", coll_now);
    verdict("and cleared itself on the next Start condition", !coll_after);
    peer_settle();
    quiesce();
}

// ---- p: bus recovery, the WIRE and not the peripheral -------------------------

void tp_recovery() {
    print(serial, "p unstick(): nine clocks and a STOP against a client that is really "
                  "holding SDA down", crlf);
    quiesce();
    verdict("link up", link_up());

    // The negative arm first: what the remedy costs on a healthy bus.
    const uint8_t healthy = T::unstick();
    print(serial, "  unstick() on a healthy idle bus returned ", healthy,
          " pulses (SDA was already high)", crlf);
    verdict("unstick() on a healthy bus needs no pulse at all", healthy == 0);
    verdict("it leaves the bus Idle", T::bus_state() == TwiBusState::idle);
    verdict("and the channel still works after it", link_cmd(Op::ping));

    // The stuck arm: the peer holds SDA low from PORT and lets go after
    // a commanded number of SCL falling edges - which is exactly what a
    // client interrupted mid-byte does, and what makes the pulse count a
    // measurement rather than a hope.
    const uint8_t release_after = 4;
    twilink::Params a{};
    a.ms = 500;
    a.aux8 = release_after;
    verdict("the peer commanded to hold SDA low", peer_action(Op::hold_sda, a));
    wait_ms(2);
    const bool sda_low = !Sda::read();
    const bool scl_high = Scl::read();
    print(serial, "  with the peer holding: SDA reads ", sda_low ? 0 : 1, ", SCL reads ",
          scl_high ? 1 : 0, crlf);
    verdict("SDA is held low while SCL is free", sda_low && scl_high);

    // A host cannot even produce a START on that bus.
    T::clear_host_flags(TWI_WIF_bm | TWI_RIF_bm | TWI_BUSERR_bm | TWI_ARBLOST_bm);
    T::address_write(twilink::command_addr);
    bool went_out = false;
    for (uint16_t i = 0; i < 2000; ++i) {
        delay_us(clock, 1);
        if (T::write_flag()) { went_out = true; break; }
    }
    print(serial, "  a host told to start on it: MSTATUS=", hex(T::host_status()),
          " after 2 ms, WIF=", went_out ? 1 : 0, crlf);
    verdict("the address never goes out while SDA is held", !went_out);

    const uint8_t pulses = T::unstick();
    const bool freed = Sda::read();
    print(serial, "  unstick() took ", pulses, " SCL pulses (the peer was told to let go "
          "after ", release_after, "); SDA now reads ", freed ? 1 : 0, crlf);
    verdict("unstick() clocked the line free", pulses != T::unstick_failed && freed);
    verdict("and it took no more than nine pulses", pulses <= 9);
    verdict("the peer released on the clock it was told to",
            pulses == release_after || pulses == release_after + 1);
    verdict("the bus is Idle again", T::bus_state() == TwiBusState::idle);
    wait_ms(20);
    verdict("and a normal exchange runs on it", link_cmd(Op::ping));

    // The unrecoverable arm: a line held by something that is not
    // counting clocks. unstick() must SAY so rather than pretend.
    a = twilink::Params{};
    a.ms = 400;
    a.aux8 = 0;                          // only the deadline releases it
    verdict("the peer commanded to hold SDA until its deadline",
            peer_action(Op::hold_sda, a));
    wait_ms(2);
    const uint8_t failed = T::unstick();
    print(serial, "  unstick() against a line nothing will release returned ", failed,
          " (unstick_failed = ", T::unstick_failed, "), SDA reads ",
          Sda::read() ? 1 : 0, crlf);
    verdict("unstick() reports failure instead of claiming success",
            failed == T::unstick_failed);
    verdict("SDA is still down, which is the honest answer", !Sda::read());
    wait_ms(450);
    verdict("once the peer lets go, SDA is back up", Sda::read());
    verdict("the bus works again", link_cmd(Op::ping));

    verdict("unstick() left the route's pins inputs with OUT = 0 (errata hygiene)",
            !Sda::is_output() && !Scl::is_output());
    quiesce();
}

// ---- q: the General Call, answered by two chips ------------------------------

void tq_general_call() {
    print(serial, "q the General Call across two boards, and a quick command a real "
                  "second chip acknowledges", crlf);
    quiesce();
    verdict("link up", link_up());
    verdict("this board's client answers 0x2C and the General Call",
            client_polled(twilink::dut_addr, {.general_call = true}));
    cl_read_address = true;

    twilink::Params a{};
    a.ms = 400;
    a.addr = twilink::command_addr;
    a.count = 3;
    a.flags = twilink::flag_general_call | twilink::flag_stop_interrupt;
    verdict("the peer's client answers the General Call too",
            peer_action(Op::serve, a));
    for (uint8_t i = 0; i < 3; ++i) tx_buf[i] = static_cast<uint8_t>(0x71 + i);
    uint8_t st = run_request<Client>({0x00, tx_buf, 3, nullptr, 0, {}});
    const uint8_t local_n = cl_rx_n;
    const uint8_t local_addr = cl_last_addr;
    const uint8_t local0 = cl_rx[0];
    peer_settle();
    twilink::Report rep{};
    bool got = peer_report(rep);
    if (got) print_report(rep);
    print(serial, "  general call: status ", st, "; this board took ", local_n,
          " bytes on address ", hex(local_addr), crlf);
    verdict("the general call is acknowledged", st == i2c_ok);
    verdict("THIS board's client received it", local_n == 3 && local0 == 0x71 &&
                                               local_addr == 0x00);
    verdict("and so did the peer's, at the same time",
            got && rep.aux0 == 3 && rep.first == 0x71 && rep.last_addr == 0x00);

    // General Call OFF at the peer: the same call must miss it while its
    // own address still answers - both inside ONE action, so there is no
    // doubt the client was alive throughout.
    a = twilink::Params{};
    a.ms = 500;
    a.addr = twilink::command_addr;
    a.count = 3;
    a.flags = twilink::flag_stop_interrupt;      // no flag_general_call
    verdict("the peer commanded General Call OFF", peer_action(Op::serve, a));
    st = run_request<Client>({0x00, tx_buf, 3, nullptr, 0, {}});
    const uint8_t gc_local = cl_rx_n;
    wait_ms(3);
    for (uint8_t i = 0; i < 3; ++i) tx_buf[i] = static_cast<uint8_t>(0x81 + i);
    const uint8_t st2 = run_request<Client>({twilink::command_addr, tx_buf, 3,
                                             nullptr, 0, {}});
    peer_settle();
    got = peer_report(rep);
    if (got) print_report(rep);
    verdict("the general call is still acknowledged - by THIS board's client",
            st == i2c_ok && gc_local == 3);
    verdict("the exact address still reaches the peer", st2 == i2c_ok);
    verdict("but the peer heard ONLY the exact address, not the general call",
            got && rep.addr_hits == 1 && rep.last_addr == twilink::command_addr &&
            rep.aux0 == 3 && rep.first == 0x81);

    // A quick command with a real chip at the other end.
    a = twilink::Params{};
    a.ms = 300;
    a.addr = twilink::command_addr;
    a.count = 0;
    a.flags = twilink::flag_stop_interrupt;
    verdict("the peer armed to answer address packets", peer_action(Op::serve, a));
    ChScl::source(EvPin<Scl>{});
    Host::quick_command(true);
    const uint16_t edges = count_edges_of([] {
        (void)run_request<Client>({twilink::command_addr, tx_buf, 4, nullptr, 0, {}});
    });
    const uint8_t qst = Host::status();
    last_mstatus = 0;
    const uint8_t qabsent = run_request<Client>({absent_addr, nullptr, 0, nullptr, 0, {}});
    Host::quick_command(false);
    print(serial, "  quick command to a real second chip: ", edges, " SCL rising edges, "
          "status ", qst, "; to an address nobody holds: ", qabsent, crlf);
    verdict("a quick command across the wire is one frame plus the STOP's rise",
            edges == 10 && qst == i2c_ok);
    verdict("and one to an empty address is a nack_addr", qabsent == i2c_nack_addr);
    wait_ms(320);
    quiesce();
}

// ---- r: the three speeds against a real client -------------------------------

void speed_round(TwiSpeed s, const char* name) {
    twilink::Params a{};
    a.ms = 400;
    a.addr = twilink::command_addr;
    // The action must span BOTH tenures below - the write-then-read and
    // the plain read - or it ends at the first STOP and the second read
    // is answered by the command-mode client instead: four written plus
    // eight served.
    a.count = 12;
    a.seed = 0x20;
    a.flags = twilink::flag_stop_interrupt;
    if (!peer_action(Op::serve, a)) {
        verdict("the peer armed for ", name, false);
        return;
    }
    for (uint8_t i = 0; i < 4; ++i) tx_buf[i] = static_cast<uint8_t>(0xB1 + i);
    for (uint8_t i = 0; i < 8; ++i) rx_buf[i] = 0;

    meter_mode = 0;
    SclFreq::init(clock, ChScl{}, TcbClock::div1);
    meter_reset();
    const uint8_t st = run_request<Client>({twilink::command_addr, tx_buf, 4,
                                            rx_buf, 4, {}, s});
    const uint16_t period = meter_min;
    meter_stop();
    const uint8_t baud = T::baud();

    meter_mode = 1;
    SclLow::init(clock, ChScl{}, TcbClock::div1, true);
    meter_reset();
    const uint8_t st2 = run_request<Client>({twilink::command_addr, nullptr, 0,
                                             rx_buf + 4, 4, {}, s});
    const uint16_t low_max = meter_max;
    const uint16_t low_min = meter_min;
    meter_stop();

    bool exact = st == i2c_ok && st2 == i2c_ok;
    for (uint8_t i = 0; i < 4; ++i) exact = exact && rx_buf[i] == static_cast<uint8_t>(0x20 + i);
    for (uint8_t i = 0; i < 4; ++i) {
        exact = exact && rx_buf[4 + i] == static_cast<uint8_t>(0x24 + i);
    }
    const uint32_t hz = period ? SysClock::hz / period : 0;
    const uint16_t floor_ticks = static_cast<uint16_t>(twi_period_ticks(SysClock::hz, baud, 0));
    print(serial, "  ", name, ": MBAUD=", baud, " min period ", period, " ticks = ", hz,
          " Hz (register floor ", floor_ticks, "); SCL low min ", low_min, " max ",
          low_max, " ticks = ", low_max * 1000ul / 24ul, " ns", crlf);
    print(serial, "    write-then-read (", st, ") gave ", hex(rx_buf[0]), " ",
          hex(rx_buf[1]), " ", hex(rx_buf[2]), " ", hex(rx_buf[3]), ", the read (", st2,
          ") gave ", hex(rx_buf[4]), " ", hex(rx_buf[5]), " ", hex(rx_buf[6]), " ",
          hex(rx_buf[7]), crlf);
    verdict("the exchange is exact against the peer at ", name, exact);
    verdict("SCL never runs faster than the register allows at ", name,
            period >= floor_ticks);
    verdict("nor above the mode's nominal rate at ", name, hz <= twi_scl_hz(s));
    const uint16_t nominal_low = twi_low_ticks(baud);
    print(serial, "    the peer's turnaround stretches the longest SCL low to ",
          low_max, " ticks against the register's ", nominal_low, crlf);
    peer_settle();
}

void tr_speeds() {
    print(serial, "r the three bus speeds against a REAL client on the other board", crlf);
    quiesce();
    verdict("link up", link_up());
    ChScl::source(EvPin<Scl>{});
    speed_round(TwiSpeed::standard_100k, "Sm 100 kHz");
    speed_round(TwiSpeed::fast_400k, "Fm 400 kHz");
    speed_round(TwiSpeed::fast_plus_1m, "Fm+ 1 MHz");
    verdict("the host is back at 100 kHz for the command channel",
            link_cmd(Op::ping));
    quiesce();
}

// ---- s: a clock rebase under two-board traffic -------------------------------

bool two_board_step(uint32_t hz, const char* what) {
    twilink::Params a{};
    a.ms = 400;
    a.addr = twilink::command_addr;
    a.count = 8;
    a.seed = static_cast<uint8_t>(hz / 1'000'000u);
    a.flags = twilink::flag_stop_interrupt;
    if (!peer_action(Op::serve, a)) {
        verdict("the peer answered ", what, false);
        return false;
    }
    for (uint8_t i = 0; i < 4; ++i) tx_buf[i] = static_cast<uint8_t>(0xE1 + i);
    for (uint8_t i = 0; i < 8; ++i) rx_buf[i] = 0;
    const uint8_t st = run_request<Client>({twilink::command_addr, tx_buf, 4,
                                            rx_buf, 4, {}});
    bool exact = st == i2c_ok;
    for (uint8_t i = 0; i < 4; ++i) {
        exact = exact && rx_buf[i] == static_cast<uint8_t>(a.seed + i);
    }
    const uint8_t baud = T::baud();
    print(serial, "  ", what, ": CLK_PER = ", hz, " Hz, MBAUD = ", baud,
          ", actual_scl_hz(0) = ", Host::actual_scl_hz(0), " Hz, actual_scl_hz(166 ns) = ",
          Host::actual_scl_hz(166), " Hz; exchange status ", st, crlf);
    verdict("the exchange across the wire is exact ", what, exact);
    verdict("MBAUD was re-derived for this clock ", what,
            Host::actual_scl_hz(0) <= 100'000u && Host::actual_scl_hz(0) > 80'000u);
    peer_settle();
    return exact;
}

void ts_rebase() {
    print(serial, "s 24 -> 12 -> 24 MHz with the command channel and the peer's client "
                  "live on the wire", crlf);
    quiesce();
    verdict("DynamicClock init (boot = the crystal)", DynClock::init());
    verdict("host init on the dynamic clock", Host::init(DynClock{}));
    T::enable_read_interrupt(false);
    T::enable_write_interrupt(false);
    verdict("the peer answers at 24 MHz", link_cmd(Op::ping));

    (void)two_board_step(24'000'000u, "at 24 MHz");
    verdict("switch to 12 MHz", DynClock::set(12'000'000u));
    verdict("the command channel survived the switch", link_cmd(Op::ping));
    (void)two_board_step(12'000'000u, "at 12 MHz");
    verdict("back to 24 MHz", DynClock::set(24'000'000u));
    verdict("the command channel survived the switch back", link_cmd(Op::ping));
    (void)two_board_step(24'000'000u, "back at 24 MHz");

    quiesce();
}

// ---- the menu ----------------------------------------------------------------

using TestFn = void (*)();
struct Test { char key; TestFn fn; };
constexpr Test tests[] = {
    {'a', ta_routes}, {'b', tb_speeds}, {'c', tc_loops}, {'d', td_addresses},
    {'e', te_cases}, {'f', tf_smart}, {'g', tg_quick}, {'h', th_busstate},
    {'i', ti_interrupts}, {'j', tj_rebase},
    {'k', tk_bringup}, {'l', tl_stretch}, {'m', tm_nacks}, {'n', tn_arbitration},
    {'o', to_collision}, {'p', tp_recovery}, {'q', tq_general_call}, {'r', tr_speeds},
    {'s', ts_rebase},
};
constexpr char single_board[] = "abcdefghij";
constexpr char two_board[] = "klmnopqrs";

void run(TestFn fn) {
    passed = failed = 0;
    fn();
    print(serial, "  -> ", passed, " pass, ", failed, " fail", crlf, crlf);
}

void run_set(const char* keys) {
    uint16_t tp = 0, tf = 0;
    for (const char* k = keys; *k != 0; ++k) {
        for (const Test& t : tests) {
            if (t.key != *k) continue;
            run(t.fn);
            tp += passed;
            tf += failed;
        }
    }
    print(serial, "ALL: ", tp, " pass, ", tf, " fail", crlf);
}

void help() {
    print(serial, "test_avr_twi: a routes | b bus speeds | c combined and dual loops | "
                  "d address match | e the chapter's cases | f smart modes | "
                  "g quick command | h bus state and the injector | i isr bodies | "
                  "j rebase and the arbiter    -> z = all of a..j", crlf);
    print(serial, "  TWO BOARDS (board B running twi_peer): k bring-up | l clock "
                  "stretching | m NACK injection | n multi-host arbitration | "
                  "o collision S4 | p bus recovery | q general call | r the three "
                  "speeds | s rebase    -> y = all of k..s", crlf);
    print(serial, "  ONE open-drain bus, three taps of this board: PA2/PA3 (the host and "
                  "the combined client), PC2/PC3 (the DUAL client, and the bit-bang "
                  "injector of tests h and n while Dual mode is off) and PB2/PB3; board "
                  "B taps it with its own PA2/PA3. 1.5k pull-ups to +5 V; TWI1 stays "
                  "disabled.", crlf);
}

} // namespace

ISR(USART2_RXC_vect) { (void)Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }

ISR(TWI0_TWIM_vect) {
    ++host_ints;
    if (!isr_mode) {
        // Never expected: the polled tests keep RIEN/WIEN down. Silence
        // the source rather than storm.
        T::enable_read_interrupt(false);
        T::enable_write_interrupt(false);
        return;
    }
    if (Host::isr()) {
        host_done = true;
        if (bus_mode) post<Bus>(TransferDone{Host::status()});
    }
}

ISR(TWI0_TWIS_vect) { service_client<Client>(); }

ISR(TCB0_INT_vect) {
    const uint16_t t = meter_mode == 0 ? SclFreq::period_ticks() : SclLow::width_ticks();
    ++meter_caps;
    if (meter_caps <= 2) return;        // the arming capture is not traffic
    if (t < meter_min) meter_min = t;
    if (t > meter_max) meter_max = t;
}

int main() {
    const bool xtal = SysClock::init();
    Serial::init(clock, 460800);
    sei();
    auto board = board_id();
    if (board.empty()) board = "?";
    print(serial, crlf, "test_avr_twi - TWI/I2C test suite (board ", board,
          ", clk=", xtal ? "XTAL" : "OSCHF",
          " 24 MHz, silicon rev ", hex(SYSCFG.REVID), ")", crlf);
    (void)wire_sane(true);
    help();
    print(serial, "> ");
    for (;;) {
        uint8_t c;
        if (!Serial::read_byte(c)) continue;
        if (c == '\r' || c == '\n') continue;
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') { help(); }
        else if (c == 'z' || c == 'Z') { run_set(single_board); }
        else if (c == 'y' || c == 'Y') { run_set(two_board); }
        else {
            bool found = false;
            for (const Test& t : tests) if (t.key == c) { run(t.fn); found = true; }
            if (!found) print(serial, "? for help", crlf);
        }
        print(serial, "> ");
    }
}
