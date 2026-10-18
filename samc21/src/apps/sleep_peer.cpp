// sleep_peer - the INSTRUMENT half of test_avr_sleep's two-board set on
// the SAM C21: the scriptable peer the DUT drives IN BAND over the
// one-wire command channel while it stops its own clocks, speaking
// sleep_link.hpp - the one wire format every architecture on this link
// compiles.
//
// WHY IT EXISTS. A sleeping chip cannot time its own wake-up: the counter
// that would do it is exactly the one the mode stops. So the ruler lives
// here: this board raises the stimulus edge and starts nothing - the
// stopwatch runs free and the stimulus is put on the pad BY A COMPARE
// MATCH of that stopwatch, and the DUT's echo CAPTURES the same counter.
// The gap between the compare value and the capture is the measurement,
// with no software anywhere in its path.
//
// Not a kernel app: one blocking loop that polls SERCOM1, decodes a
// command frame, acknowledges it and becomes for a bounded moment what
// the DUT needs - a train of stimulus edges, one byte on the command
// wire at a foreign baud (the DUT's start-of-frame wake), or one timed
// I2C write tenure against the DUT's TWI client. Every action carries a
// count and a millisecond deadline after which command mode is restored
// BY ITSELF (the protocol's recovery guarantee).
//
// WIRING (the desk's four link wires and its I2C pair; no topology is
// guessed):
//   PA16  the command channel: one wire to the DUT's PE0, the AVR's TXD
//         pad under LBME, 8N1 at slink::command_baud. SERCOM1 PAD[0]
//         under function C, the receiver always on it; the transmitter
//         is moved ONTO it only to answer (TXPO = PAD[0]) and back to
//         PAD[2] after, whose pad (PA18) is never muxed: 31.6.2 lets a
//         transmitter drive only a pad its PMUX gives it, and a disabled
//         transmitter still drives its pad at idle (docs/samc21/
//         sercom.md), so moving TxD's PAD is the release - the AVR end
//         drives the wire push-pull when it talks.
//   PA19  this board drives, the DUT senses (its PE2): the STIMULUS,
//         idling low. Under PORT; its rising edge is the PORT's event
//         input 0 acting SET (28.6.4), fed by the stopwatch's CC0 match
//         on an asynchronous channel.
//   PA17  the DUT drives (its PE3), this board senses: the ECHO. EXTINT1
//         sensed on a HIGH LEVEL, so its event output is a copy of the
//         pad; on an asynchronous channel into the stopwatch's event
//         input, it captures COUNT into CC1.
//   PA18  the DUT's PE1, spare in this protocol: an input with no pull.
//   PA22/PA23  the I2C pair (SERCOM3), the host of the twi_write tenure.
//
// THE STOPWATCH: TC0+TC1 as one 32-bit counter of the 24 MHz CRYSTAL
// (generator 2), so a tick is 41.7 ns - the unit sleep_link.hpp's
// reports are in (the DUT prints 24 ticks to the microsecond). Its fixed
// offsets - the PORT's SET action "up to three clock cycles after" the
// event (28.6.4), the EIC level's path to the capture - are constant and
// cancel against the DUT's awake baseline, which every figure is read
// against.
//
// Console: SERCOM5 PB30/PB31 at 115200, observability only.
//   ? help | i status and counters | 0 back to command mode | 3 trace
//
// The ident label is the die serial's first word in hex (this family has
// no USERROW label); ident.clock says whether the crystal started -
// without it generator 2 falls back to OSC48M / 2, half a per cent off.
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include "samc21/clock.hpp"
#include "samc21/delay.hpp"
#include "samc21/eic.hpp"
#include "samc21/evsys.hpp"
#include "samc21/i2c.hpp"
#include "samc21/nvm.hpp"
#include "samc21/pin.hpp"
#include "samc21/sercom.hpp"
#include "samc21/tc.hpp"
#include "samc21/ticker.hpp"
#include "util/print.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED: pure encoding, not one
// register, and every architecture on this link compiles the same file.
#include "../../../avrxt/src/apps/sleep_link.hpp"

using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock{};

namespace {

using namespace brio;
using slink::Op;

constexpr UartPads console_pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'B', 30, PinFunction::d},
    .rx_pin = {'B', 31, PinFunction::d},
};
using Console = Uart<5, console_pads>;
constexpr Console console;

// ---------------------------------------------------------------------------
// The pads
// ---------------------------------------------------------------------------

using LinePin = Pin<'A', 16>;   ///< SERCOM1 PAD[0]: the command wire
using EchoPin = Pin<'A', 17>;   ///< EXTINT1: the DUT's echo
using SparePin = Pin<'A', 18>;  ///< SERCOM1 PAD[2], never muxed here
using StimPin = Pin<'A', 19>;   ///< the stimulus, under PORT
using LinkPort = Port<'A'>;
using EchoLine = ExtInt<EchoPin>;

static_assert(MUX_PA16C_SERCOM1_PAD0 == static_cast<uint8_t>(PinFunction::c));
static_assert(EchoLine::line == 1u, "PA17 is EXTINT1");

// ---------------------------------------------------------------------------
// The crystal: generator 2, the reference of the link and of the ruler
// ---------------------------------------------------------------------------

constexpr uint8_t gen_xtal = 2;
constexpr uint32_t ref_hz = 24'000'000UL;

// ---------------------------------------------------------------------------
// The link: listening with TxD on PAD[2] (unmuxed), answering with TxD on
// PAD[0] - the same pad as RxD
// ---------------------------------------------------------------------------

using Link = Sercom<1>;

constexpr UartPads listen_pads{
    .tx = SercomPad::pad2,
    .rx = SercomPad::pad0,
    .tx_pin = {'A', 18, PinFunction::c},
    .rx_pin = {'A', 16, PinFunction::c},
};
constexpr UartPads talk_pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad0,
    .tx_pin = {'A', 16, PinFunction::c},
    .rx_pin = {'A', 16, PinFunction::c},
};
static_assert(uart_pads_loop_back(talk_pads));

constexpr SercomBaud command_baud = sercom_closest_baud(ref_hz, slink::command_baud).value();

// ---------------------------------------------------------------------------
// The stopwatch and its two event paths
// ---------------------------------------------------------------------------

using Watch = Tc<0>;
constexpr uint8_t ev_stim = 0;   ///< TC0 MC0 -> PORT event input 0 (SET PA19)
constexpr uint8_t ev_echo = 1;   ///< EXTINT1 -> TC0's event input (capture CC1)

constexpr TcConfig watch_cfg{.mode = TcMode::count32, .prescaler = TcPrescaler::div1,
                             .waveform = TcWaveform::normal_frequency,
                             .capture_enable = 0x2};

/// The lead from the COUNT read (two READSYNC crossings, about 300 CPU
/// cycles) to the compare that raises the stimulus, with the compare's
/// own write-synchronization inside it: 40 us.
constexpr uint32_t lead_ticks = 960;

bool watch_up() {
    if (!Watch::init(gen_xtal) || !Watch::configure(watch_cfg) ||
        !Watch::event_config(watch_cfg, TcEventConfig{.action = TcEventAction::off,
                                                      .input_enable = true,
                                                      .match_out = 0x1})) {
        return false;
    }
    if (!Eic::init() || !Eic::clock_select(EicClock::ulp32k)) {
        return false;
    }
    EchoLine::claim(PinPull::down);
    if (!Eic::configure_line(EchoLine::line,
                             EicLineConfig{.sense = EicSense::high, .event_out = true}) ||
        !Eic::enable(true)) {
        return false;
    }
    // Table 29-3: the PORT's and the TC's event users take the
    // asynchronous path only.
    if (!Evsys::connect(LinkPort::event_user(0), ev_stim,
                        EventChannelConfig{.generator = Watch::match_generator(0),
                                           .path = EventPath::asynchronous}) ||
        !Evsys::connect(Watch::event_user, ev_echo,
                        EventChannelConfig{.generator = EchoLine::event_generator,
                                           .path = EventPath::asynchronous})) {
        return false;
    }
    return Watch::enable(true);
}

/// Both capture stages empty and the flags down, so the next stamp is
/// this shot's (35.6.2.8: a capture arriving while MC1 stands is dropped).
void echo_arm() {
    (void)Watch::cc32(1);
    (void)Watch::cc32(1);
    Watch::clear_flags(Watch::match_flag(1) | Watch::error_flag);
}

bool echo_captured() { return (Watch::flags() & Watch::match_flag(1)) != 0u; }

// ---------------------------------------------------------------------------
// The I2C host of the twi_write tenure
// ---------------------------------------------------------------------------

constexpr I2cPads bus_pads{
    .sda_pin = {'A', 22, PinFunction::c},
    .scl_pin = {'A', 23, PinFunction::c},
};
using Host = I2cHost<3, bus_pads>;
/// The desk's node, metered by the AVR's TCBs (docs/samc21/i2c.md).
constexpr uint32_t bus_rise_ns = 166;

/// The engine's completion, written by SERCOM3_Handler.
volatile bool xfer_done = false;
volatile uint8_t xfer_status = i2c_ok;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

constexpr uint16_t firmware_version = 0x0201;   ///< the SAM C21's sleep_peer

bool crystal_ok = false;
bool watch_ok = false;
slink::Decoder decoder;
slink::Report last_report;
uint32_t gaps[slink::max_shots];
uint8_t gap_n = 0;
uint32_t commands = 0, naks = 0, actions = 0;
bool trace = false;
char label8[9] = {};

/// When the acknowledgement of the command being executed left the
/// line. The DUT reconfigures slink::settle_ms after that same instant,
/// so every action waits a little longer before touching anything.
uint32_t ack_ms = 0;
uint32_t last_byte_ms = 0;

void settle() {
    while (Ticker::millis() - ack_ms < static_cast<uint32_t>(slink::settle_ms) + 2u) {
    }
}

void wait_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

/// Microseconds of any length: delay_us() serves less than a SysTick
/// period, so a longer wait is chunked.
void wait_us(uint32_t us) {
    while (us >= 900u) {
        (void)delay_us(clock, 900u);
        us -= 900u;
    }
    if (us != 0u) {
        (void)delay_us(clock, us);
    }
}

// ---------------------------------------------------------------------------
// The link
// ---------------------------------------------------------------------------

constexpr uint32_t dre_spins = 400'000u;

bool link_apply(const SercomUartConfig& c) {
    (void)Link::enable(false);
    return Link::configure(c) && Link::enable(true);
}

/// Command mode: the receiver on PA16 at the command baud, TxD on the
/// unmuxed PAD[2] - nothing of this board drives the wire.
bool command_mode() {
    const bool ok = link_apply({.pads = listen_pads, .baud = command_baud.reg,
                                .sampling = command_baud.sampling});
    LinePin::function(PinFunction::c, {.input_enable = true});
    SparePin::release();
    SparePin::input();
    Link::flush_rx();
    decoder.reset();
    return ok;
}

/// TxD onto the wire at `b`, the receiver off (it would hear this board).
bool line_talk(const SercomBaud& b) {
    return link_apply({.pads = talk_pads, .baud = b.reg, .sampling = b.sampling,
                       .rx = false});
}

[[gnu::always_inline]] inline bool put_byte(uint8_t v) {
    for (uint32_t n = dre_spins; !Link::dre_flag(); --n) {
        if (n == 0u) return false;
    }
    Link::data(v);
    return true;
}

/// The last stop bit gone: TXC up, bounded.
void wait_tx_done() {
    for (uint32_t n = dre_spins; !Link::txc_flag() && n != 0u; --n) {
    }
}

void put_link(uint8_t b) { (void)put_byte(b); }

/// Four bit times at the command baud before an answer: RXC rises at the
/// middle of the stop bit, half a bit before the sender's own TXC, and
/// the DUT listens again only after its line_listen(). 35 us.
void answer(Op op, const uint8_t* p, uint8_t len) {
    wait_us(35u);
    if (line_talk(command_baud)) {
        slink::write_frame(put_link, op, p, len);
        wait_tx_done();
    }
    (void)command_mode();
}

void ack(Op op, uint8_t sum, bool good) {
    const uint8_t p[2] = {slink::byte_of(op), sum};
    answer(good ? Op::ack : Op::nak, p, 2);
}

// ---------------------------------------------------------------------------
// The actions
// ---------------------------------------------------------------------------

/// `count` stimulus edges on PA19, `period_ms` apart, each timed to the
/// DUT's echo on PA17. A shot whose echo never arrives is stored as
/// slink::no_capture and counted as a miss.
slink::Report run_pulse(const slink::Params& a) {
    slink::Report r{};
    settle();
    gap_n = 0;
    const uint16_t n = a.count > slink::max_shots ? slink::max_shots : a.count;
    StimPin::clear();
    StimPin::output();
    (void)LinkPort::configure_event(0, PortEventConfig{.pin = StimPin::pin_number,
                                                       .action = PortEventAction::set,
                                                       .enable = true});
    const uint32_t t0 = Ticker::millis();
    if (a.delay_ms) {
        wait_ms(a.delay_ms);
    }
    for (uint16_t i = 0; i < n; ++i) {
        if (i) {
            const uint32_t due = t0 + a.delay_ms + static_cast<uint32_t>(i) * a.period_ms;
            while (static_cast<int32_t>(Ticker::millis() - due) < 0) {
            }
        }
        // The echo must be back down, or there is no rising edge to
        // capture. Give the DUT a bounded moment to lower it.
        const uint32_t tl = Ticker::millis();
        while (EchoPin::read() && Ticker::millis() - tl < 5u) {
        }
        echo_arm();
        Watch::clear_flags(Watch::match_flag(0));
        const uint32_t at = Watch::count32() + lead_ticks;
        (void)Watch::set_cc32(0, at);              // THE STIMULUS EDGE, in hardware

        const uint32_t ts = Ticker::millis();
        bool fired = false;
        bool got = false;
        while (Ticker::millis() - ts < a.deadline_ms) {
            fired = fired || (Watch::flags() & Watch::match_flag(0)) != 0u;
            if (echo_captured()) {
                got = true;
                break;
            }
        }
        const uint32_t ticks = got && fired ? Watch::cc32(1) - at : slink::no_capture;
        // hold_us is a FLOOR on the pulse width, not its length: an idle
        // DUT answers in a microsecond, and a missed shot must never be
        // blamed on a pulse too short to latch.
        if (a.hold_us) wait_us(a.hold_us);
        StimPin::clear();
        if (gap_n < slink::max_shots) gaps[gap_n++] = ticks;
        if (ticks != slink::no_capture) {
            ++r.hits;
            r.ticks = ticks;
        } else {
            ++r.misses;
        }
        ++r.count;
    }
    LinkPort::release_events();
    if (r.misses) r.flags |= slink::report_timed_out;
    return r;
}

/// Arm the stopwatch and wait for ONE echo: the wire-sanity probe the
/// DUT runs before any sleep is involved.
slink::Report run_capture(const slink::Params& a) {
    slink::Report r{};
    settle();
    gap_n = 0;
    if (a.delay_ms) wait_ms(a.delay_ms);
    echo_arm();
    const uint32_t zero = Watch::count32();
    const uint32_t ts = Ticker::millis();
    bool got = false;
    while (Ticker::millis() - ts < a.deadline_ms) {
        if (echo_captured()) {
            got = true;
            break;
        }
    }
    r.count = 1;
    if (got) {
        r.ticks = Watch::cc32(1) - zero;
        r.hits = 1;
        gaps[gap_n++] = r.ticks;
    } else {
        r.misses = 1;
        r.flags |= slink::report_timed_out;
        gaps[gap_n++] = slink::no_capture;
    }
    return r;
}

/// One byte on the command wire at a foreign baud, after a delay: the
/// DUT's start-of-frame wake-up. Fire and restore.
slink::Report run_sfd_byte(const slink::Params& a) {
    slink::Report r{};
    settle();
    const auto b = sercom_closest_baud(ref_hz, a.rate);
    if (!b) {
        r.flags |= slink::report_failed;
        return r;
    }
    if (a.delay_ms) wait_ms(a.delay_ms);
    if (!line_talk(*b)) {
        r.flags |= slink::report_failed;
        return r;
    }
    (void)put_byte(a.value);
    wait_tx_done();
    r.count = 1;
    r.aux = static_cast<uint8_t>(b->reg >> 8);
    r.status = static_cast<uint8_t>(b->reg);
    return r;
}

/// One host write tenure against the DUT's TWI client, timed on the
/// stopwatch: from the COUNT read before the engine's start to the one
/// after its completion. Both reads are the same double READSYNC, so
/// their cost is a constant the DUT's awake tenure carries too.
slink::Report run_twi_write(const slink::Params& a) {
    slink::Report r{};
    settle();
    if (a.delay_ms) wait_ms(a.delay_ms);
    const I2cSpeed speed = a.rate >= 1'000'000u ? I2cSpeed::fast_plus_1m
                           : a.rate >= 400'000u ? I2cSpeed::fast_400k
                                                : I2cSpeed::standard_100k;
    if (!Host::init(clock, bus_rise_ns)) {
        r.flags |= slink::report_failed;
        return r;
    }
    uint8_t tx[16];
    const uint8_t n = a.count > 16 ? 16 : static_cast<uint8_t>(a.count);
    for (uint8_t i = 0; i < n; ++i) tx[i] = static_cast<uint8_t>(a.value + i);

    uint8_t status = 0xFE;
    xfer_done = false;
    const uint32_t t_start = Watch::count32();
    if (Host::start({.addr = a.addr,
                     .tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx)),
                     .tx_len = n,
                     .rx = {},
                     .rx_len = 0,
                     .reply = {},
                     .speed = speed})) {
        status = Host::status();   // the degenerate synchronous failure
    } else {
        // The engine runs from its own vector (init() enables the NVIC
        // line; SERCOM3_Handler below), the completion a flag spun on here.
        const uint32_t ts = Ticker::millis();
        while (!xfer_done && Ticker::millis() - ts < a.deadline_ms) {
        }
        if (xfer_done) {
            status = xfer_status;
        }
    }
    r.ticks = Watch::count32() - t_start;
    r.status = status;
    r.count = n;
    if (status == i2c_ok) r.flags |= slink::report_acked;
    if (status == 0xFE) r.flags |= slink::report_timed_out;
    Host::release();
    return r;
}

// ---------------------------------------------------------------------------
// The command handler
// ---------------------------------------------------------------------------

void serve_gaps(uint8_t start) {
    uint8_t p[slink::gaps_header + 4u * slink::gaps_per_frame] = {};
    uint8_t k = 0;
    while (k < slink::gaps_per_frame && static_cast<uint8_t>(start + k) < gap_n) {
        slink::put32(p + slink::gaps_header + 4u * k, gaps[start + k]);
        ++k;
    }
    p[0] = start;
    p[1] = k;
    answer(Op::gaps_data, p, static_cast<uint8_t>(slink::gaps_header + 4u * k));
}

void handle(const slink::Frame& f) {
    ++commands;
    const Op op = f.op;
    if (trace) {
        print(console, "  [cmd op=", hex(slink::byte_of(op)), " len=", f.len, "]", crlf);
    }
    if (op == Op::ping) {
        ack(op, f.sum, true);
        return;
    }
    if (op == Op::ident) {
        ack(op, f.sum, true);
        slink::Ident d{};
        for (uint8_t i = 0; i < 8; ++i) d.label[i] = label8[i];
        d.clock = crystal_ok ? slink::clock_crystal : slink::clock_oschf;
        d.sanity = slink::ident_sanity;
        d.version = firmware_version;
        uint8_t p[slink::ident_size];
        slink::put_ident(p, d);
        answer(Op::ident_data, p, slink::ident_size);
        return;
    }
    if (op == Op::report) {
        ack(op, f.sum, true);
        uint8_t p[slink::report_size];
        slink::put_report(p, last_report);
        answer(Op::report_data, p, slink::report_size);
        return;
    }
    if (op == Op::gaps) {
        ack(op, f.sum, true);
        serve_gaps(f.len >= 1 ? f.data[0] : 0);
        return;
    }
    if (f.len < slink::params_size) {
        ack(op, f.sum, false);
        ++naks;
        return;
    }
    ack(op, f.sum, true);
    ack_ms = Ticker::millis();
    ++actions;

    const slink::Params a = slink::get_params(f.data);
    slink::Report r{};
    const uint32_t started = Ticker::millis();
    switch (op) {
        case Op::pulse: r = run_pulse(a); break;
        case Op::capture: r = run_capture(a); break;
        case Op::sfd_byte: r = run_sfd_byte(a); break;
        case Op::twi_write: r = run_twi_write(a); break;
        default: r.flags |= slink::report_failed; break;
    }
    if (!(r.flags & slink::report_failed)) r.flags |= slink::report_ran;
    const uint32_t took = Ticker::millis() - started;
    r.ms = took > 65535u ? 65535u : static_cast<uint16_t>(took);
    r.op = slink::byte_of(op);
    last_report = r;
    (void)command_mode();
}

// ---------------------------------------------------------------------------
// The console
// ---------------------------------------------------------------------------

void help() {
    print(console, "sleep_peer: ? help | i status | 0 command mode | 3 trace", crlf);
}

void status() {
    print(console, "  link BAUD=", Link::baud_reg(), " crystal=", crystal_ok ? "up" : "DOWN",
          " stopwatch=", watch_ok ? "up" : "DOWN", crlf);
    print(console, "  commands=", commands, " actions=", actions, " naks=", naks, crlf);
    print(console, "  PA19 stimulus=", StimPin::read() ? 1 : 0,
          " PA17 echo=", EchoPin::read() ? 1 : 0, crlf);
    print(console, "  last report: op=", hex(last_report.op), " flags=",
          hex(last_report.flags), " count=", last_report.count,
          " hits=", last_report.hits, " misses=", last_report.misses,
          " ticks=", last_report.ticks, " status=", hex(last_report.status),
          " ms=", last_report.ms, crlf);
    for (uint8_t i = 0; i < gap_n; ++i) {
        print(console, "    shot ", i, ": ", gaps[i], " ticks", crlf);
    }
}

}  // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void SERCOM5_Handler() { (void)Console::isr(); }
/// The I2C host's vector: Host::init() enables the line, so it is bound
/// whether or not a tenure runs.
extern "C" void SERCOM3_Handler() {
    if (Host::isr()) {
        xfer_status = Host::status();
        xfer_done = true;
    }
}

int main() {
    SysClock::init();
    Console::init(clock, 115200);
    Ticker::init(clock);
    brio::enable_interrupts();

    const uint32_t w0 = DeviceSerial::read().word[0];
    for (uint8_t i = 0; i < 8; ++i) {
        const uint8_t nib = static_cast<uint8_t>((w0 >> (28u - 4u * i)) & 0xFu);
        label8[i] = static_cast<char>(nib < 10 ? '0' + nib : 'a' + nib - 10);
    }

    // The crystal, on generator 2: the reference of the link and the
    // ruler. Without it, OSC48M / 2 stands in and ident.clock says so.
    crystal_ok = Xosc::init(XoscConfig{.hz = ref_hz, .startup = 4, .on_demand = false}) &&
                 Gclk<gen_xtal>::configure(GclkConfig{.source = GclkSource::xosc});
    if (!crystal_ok) {
        (void)Gclk<gen_xtal>::configure(GclkConfig{.source = GclkSource::osc48m, .div = 2});
    }
    StimPin::clear();
    StimPin::output();
    Evsys::bus_clock(true);
    watch_ok = watch_up();

    Link::bus_clock(true);
    (void)Link::core_clock(gen_xtal);
    (void)Link::reset();

    print(console, crlf, "sleep_peer - sleep instrument peer (SAM C21 board ", label8,
          ", xtal=", crystal_ok ? "up" : "DOWN", ", fw ", hex(firmware_version), ")", crlf);
    print(console, "link SERCOM1 on the one-wire PA16, 8N1 @ ", slink::command_baud,
          "; PA19 = stimulus out (a CC0 match), PA17 = echo in (captured into CC1), "
          "24 MHz crystal ticks", crlf);
    print(console, "I2C host SERCOM3 on PA22/PA23 for the address-match wake tenure", crlf);
    if (!watch_ok) {
        print(console, "WARNING: the stopwatch did not come up", crlf);
    }
    (void)command_mode();
    help();
    print(console, "> ");

    for (;;) {
        uint8_t c;
        if (Console::read_byte(c)) {
            if (c != '\r' && c != '\n') {
                print(console, static_cast<char>(c), crlf);
                if (c == '?') help();
                else if (c == 'i') status();
                else if (c == '0') {
                    (void)command_mode();
                    StimPin::clear();
                    StimPin::output();
                    print(console, "  command mode", crlf);
                } else if (c == '3') {
                    trace = !trace;
                    print(console, trace ? "  trace on" : "  trace off", crlf);
                } else print(console, "? for help", crlf);
                print(console, "> ");
            }
        }
        // A frame that stops half way must not eat the next one: a quiet
        // line for longer than any inter-byte gap resets the reassembly.
        if (decoder.partial() && Ticker::millis() - last_byte_ms > 50u) {
            decoder.reset();
        }
        if (!Link::rxc_flag()) continue;
        const uint16_t st = Link::status();
        const uint16_t d = Link::data();
        last_byte_ms = Ticker::millis();
        if ((st & SercomStatus::receive_errors) != 0u) {
            Link::clear_status(st & SercomStatus::receive_errors);
            decoder.reset();
            continue;
        }
        switch (decoder.feed(static_cast<uint8_t>(d))) {
            case slink::Decoder::Result::frame:
                handle(decoder.frame());
                last_byte_ms = Ticker::millis();
                break;
            case slink::Decoder::Result::bad_checksum:
                ack(decoder.pending_op(), decoder.frame().sum, false);
                ++naks;
                break;
            default: break;
        }
    }
}
