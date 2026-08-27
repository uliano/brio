// sleep_peer - the INSTRUMENT half of the SLEEP campaign: board B, the
// scriptable peer that test_avr_sleep (board A, the DUT) drives IN BAND
// over the PE0 one-wire link while A is stopping its own clocks.
//
// WHY IT EXISTS. A sleeping chip cannot time its own wake-up: the only
// clock that survives power-down is the PIT's, and the counter that
// could measure the restart is exactly the one the mode stops
// (test_avr_sleep f says so). So the ruler lives on this board. B
// drives the stimulus edge on PE2, zeroes a 32-bit CLK_PER stopwatch on
// the same instruction, and the DUT's wake-up ISR answers with an edge
// on PE3 that CAPTURES that stopwatch through the event system - no
// software is in the measurement path on this side at all.
//
// It is deliberately not a kernel app: one blocking loop that polls the
// link, decodes a command frame (src/apps/sleep_link.hpp), acknowledges
// it and then becomes for a bounded moment whatever the DUT needs -
// a train of stimulus edges, one byte at a foreign baud on the shared
// wire (the DUT's start-of-frame wake), or one timed I2C write tenure
// against the DUT's TWI client. Every action carries a count and a
// millisecond deadline after which command mode is restored BY ITSELF.
//
// WIRING (fixed; no topology discovery here, unlike usart_peer):
//   PE0  command channel, one wire between the two USART4 TXD pads,
//        LBME both ends, 8N1 at slink::command_baud
//   PE1  spare
//   PE2  B drives, A senses: the stimulus (idles low)
//   PE3  A drives, B senses: the echo, captured in hardware
//   PA2/PA3  the desk I2C bus (SDA/SCL, 1.5k to +5 V), shared with A
//
// THE CLOCK. This board's 24 MHz crystal does not start; CLK_PER comes
// from OSCHF at 24 MHz, which is a per-cent-class reference. Every
// latency measured here is a microsecond-to-millisecond figure, so that
// accuracy is ample - but the banner and the `ident` answer both say
// which source is really in force, and the DUT prints it.
//
// Console: USART2 ALT1 (PF4/PF5) at 460800, observability only.
//   ? help | i status and counters | 0 back to command mode | 3 trace

// build: monitor_speed = 460800

#include <avr/interrupt.h>
#include <stdint.h>

#include "avrdx/clock.hpp"
#include "avrdx/delay.hpp"
#include "avrdx/evsys.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/tcb.hpp"
#include "avrdx/ticker.hpp"
#include "avrdx/twi.hpp"
#include "avrdx/usart.hpp"
#include "avrdx/userrow.hpp"
#include "util/print.hpp"

#include "sleep_link.hpp"

using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;
using slink::Op;

using Console = Uart<2, Route::alt1>;
constexpr Console console;

using Link = Usart<4>;
using LinePin = Pin<'E', 0>;      ///< TXD pad: the shared command wire
using StimPin = Pin<'E', 2>;      ///< B drives: the DUT's wake-up stimulus
using EchoPin = Pin<'E', 3>;      ///< A drives: the DUT's answer

// The stopwatch: two TCBs as one 32-bit CLK_PER counter. Its capture is
// wired to the ECHO pin's event, so the number the DUT is judged on is
// latched by silicon at the edge, not read by a loop.
using WatchLo = Tcb<1>;
using WatchHi = Tcb<2>;
using Watch = CascadedCounter<WatchLo, WatchHi>;
using ChCarry = EventChannel<0>;
using ChSnap = EventChannel<4>;   ///< PORTE pin events live on channels 4-5

using Host = TwiHost<0, TwiRoute::def>;

constexpr uint16_t firmware_version = 0x0101;

bool crystal_ok = false;
slink::Decoder decoder;
slink::Report last_report;
uint32_t gaps[slink::max_shots];
uint8_t gap_n = 0;
uint32_t commands = 0, naks = 0, actions = 0;
bool trace = false;

/// When the acknowledgement of the command being executed left the
/// line. The DUT reconfigures slink::settle_ms after that same instant,
/// so every action waits a little longer before touching anything.
uint32_t ack_ms = 0;
uint32_t last_byte_ms = 0;

void settle() {
    while (Ticker::millis() - ack_ms < static_cast<uint32_t>(slink::settle_ms) + 2u) {
    }
}

// ---- the link -------------------------------------------------------------------
// One wire between the two TXD pads: LBME at both ends, the transmitter
// taken up only to answer, a pull-up holding the line in between.

/// Half-duplex turnaround guard. RXCIF is raised at the MIDDLE of the
/// stop bit - half a bit BEFORE the sender's TXCIF - so an end that
/// answers at once starts its start bit while the other end is still
/// transmitting with its receiver off. Wait four bit times, derived
/// from the BAUD register in force.
void turnaround_guard() {
    const uint32_t bit_cycles =
        (static_cast<uint32_t>(Link::baud_reg()) * Link::samples()) / 64u;
    delay_cycles(4u * (bit_cycles ? bit_cycles : 1u));
}

void line_talk() {
    turnaround_guard();
    Link::enable_rx(false);
    PORTE.OUTSET = LinePin::mask;
    PORTE.DIRSET = LinePin::mask;
    Link::enable_tx(true);
    Link::clear_txc();          // so line_listen waits for THIS burst
}

/// Let the last frame leave, hand the wire back to the pull-up and
/// listen again. It OWNS the wait for the line to go idle - TXCIF is
/// write-one-to-clear, so a caller that waits too would spin out its
/// budget deaf.
void line_listen(bool wait = true) {
    if (wait) (void)Link::wait_line_idle(200'000u);
    Link::enable_tx(false);
    PORTE.DIRCLR = LinePin::mask;
    pinctrl_of('E', 0) |= PORT_PULLUPEN_bm;
    Link::flush_rx();
    Link::enable_rx(true);
}

/// Command mode: async 8N1 at slink::command_baud on the shared wire.
bool command_mode() {
    const bool ok = Link::init({.route = UsartRoute::def,
                                .baud = usart_baud_reg(SysClock::hz, slink::command_baud),
                                .tx = false,
                                .loop_back = true});
    PORTE.DIRCLR = LinePin::mask;
    pinctrl_of('E', 0) |= PORT_PULLUPEN_bm;
    Link::flush_rx();
    Link::clear_txc();
    decoder.reset();
    return ok;
}

void put_link(uint8_t b) { (void)Link::send(b, 200'000u); }

void answer(Op op, const uint8_t* p, uint8_t len) {
    line_talk();
    slink::write_frame(put_link, op, p, len);
    line_listen();
}

void ack(Op op, uint8_t sum, bool good) {
    const uint8_t p[2] = {slink::byte_of(op), sum};
    answer(good ? Op::ack : Op::nak, p, 2);
}

// ---- the stopwatch ----------------------------------------------------------------

/// Build the 32-bit CLK_PER counter. The snapshot channel is left
/// unsourced here: `arm_echo_capture()` gives it the echo pin, and the
/// timed actions that have no echo leave it off so that Watch::read()'s
/// software strobe still produces a clean edge (a channel already held
/// high by a pin LEVEL cannot be strobed).
void watch_init() {
    Watch::init(TcbClock::div1, ChCarry{}, ChSnap{});
    ChSnap::off();
    Watch::reset();
}

void arm_echo_capture() {
    ChSnap::source(EvPin<EchoPin>{});
    WatchLo::clear_capt();
    WatchHi::clear_capt();
}

void free_run_capture() {
    ChSnap::off();
    WatchLo::clear_capt();
    WatchHi::clear_capt();
}

bool echo_captured() { return WatchLo::capt_flag() && WatchHi::capt_flag(); }

/// Zero the counter without an ENABLE cycle: the stimulus edge follows
/// within a handful of instructions, and that fixed offset is part of
/// every number this board reports (the DUT's tests subtract an AWAKE
/// baseline measured the same way, so it cancels in every delta).
[[gnu::always_inline]] inline void watch_zero() {
    WatchLo::count(0);
    WatchHi::count(0);
}

// ---- the actions --------------------------------------------------------------------

/// `count` stimulus edges on PE2, `period_ms` apart, each timed to the
/// DUT's echo on PE3. A shot whose echo never arrives is stored as
/// slink::no_capture and counted as a miss.
slink::Report run_pulse(const slink::Params& a) {
    slink::Report r{};
    settle();
    gap_n = 0;
    const uint16_t n = a.count > slink::max_shots ? slink::max_shots : a.count;
    StimPin::clear();
    StimPin::output();
    EchoPin::input();
    arm_echo_capture();

    const uint32_t t0 = Ticker::millis();
    if (a.delay_ms) {
        while (Ticker::millis() - t0 < a.delay_ms) {
        }
    }
    for (uint16_t i = 0; i < n; ++i) {
        if (i) {
            const uint32_t due = t0 + a.delay_ms + static_cast<uint32_t>(i) * a.period_ms;
            while (static_cast<int32_t>(Ticker::millis() - due) < 0) {
            }
        }
        // The echo pin must be back down, or there is no rising edge to
        // capture. Give the DUT a bounded moment to lower it.
        const uint32_t tl = Ticker::millis();
        while (EchoPin::read() && Ticker::millis() - tl < 5u) {
        }
        WatchLo::clear_capt();
        WatchHi::clear_capt();
        const uint8_t saved = SREG;
        cli();
        watch_zero();
        PORTE.OUTSET = StimPin::mask;         // THE STIMULUS EDGE
        SREG = saved;

        const uint32_t ts = Ticker::millis();
        bool got = false;
        while (Ticker::millis() - ts < a.deadline_ms) {
            if (echo_captured()) { got = true; break; }
        }
        const uint32_t ticks = got ? Watch::captured() : slink::no_capture;
        // hold_us is a FLOOR on the pulse width, not its length: an idle
        // DUT answers in a microsecond and the loop above would drop the
        // stimulus almost at once, so a missed shot could never be blamed
        // on a pulse too short to latch.
        if (a.hold_us) delay_us(clock, static_cast<uint32_t>(a.hold_us));
        PORTE.OUTCLR = StimPin::mask;
        if (gap_n < slink::max_shots) gaps[gap_n++] = ticks;
        if (got) { ++r.hits; r.ticks = ticks; } else { ++r.misses; }
        ++r.count;
    }
    free_run_capture();
    if (r.misses) r.flags |= slink::report_timed_out;
    return r;
}

/// Arm the stopwatch and wait for ONE echo: the wire-sanity probe the
/// DUT runs before any sleep is involved.
slink::Report run_capture(const slink::Params& a) {
    slink::Report r{};
    settle();
    gap_n = 0;
    EchoPin::input();
    arm_echo_capture();
    if (a.delay_ms) delay_us(clock, static_cast<uint32_t>(a.delay_ms) * 1000u);
    WatchLo::clear_capt();
    WatchHi::clear_capt();
    watch_zero();
    const uint32_t ts = Ticker::millis();
    bool got = false;
    while (Ticker::millis() - ts < a.deadline_ms) {
        if (echo_captured()) { got = true; break; }
    }
    r.count = 1;
    if (got) {
        r.ticks = Watch::captured();
        r.hits = 1;
        gaps[gap_n++] = r.ticks;
    } else {
        r.misses = 1;
        r.flags |= slink::report_timed_out;
        gaps[gap_n++] = slink::no_capture;
    }
    free_run_capture();
    return r;
}

/// One byte on the shared PE0 wire at a foreign baud, after a delay:
/// the DUT's start-of-frame wake-up. Fire and restore - this board
/// cannot hear an answer while it is transmitting, and does not try.
slink::Report run_sfd_byte(const slink::Params& a) {
    slink::Report r{};
    settle();
    const uint16_t baud = usart_baud_reg(SysClock::hz, a.rate);
    if (baud == 0) {
        r.flags |= slink::report_failed;
        return r;
    }
    if (a.delay_ms) delay_us(clock, static_cast<uint32_t>(a.delay_ms) * 1000u);
    if (!Link::init({.route = UsartRoute::def, .baud = baud,
                     .tx = false, .loop_back = true})) {
        r.flags |= slink::report_failed;
        return r;
    }
    line_talk();
    (void)Link::send(a.value, 400'000u);
    (void)Link::wait_line_idle(400'000u);
    line_listen(false);
    r.count = 1;
    r.aux = static_cast<uint8_t>(baud >> 8);
    r.status = static_cast<uint8_t>(baud);
    return r;
}

/// One host write tenure against the DUT's TWI client, timed on the
/// wire. Start = the address byte leaving MADDR, stop = the engine
/// reporting the transaction complete (the STOP has been commanded).
/// The bus belongs to the office: the host half is released again as
/// soon as the tenure is over.
slink::Report run_twi_write(const slink::Params& a) {
    slink::Report r{};
    settle();
    if (a.delay_ms) delay_us(clock, static_cast<uint32_t>(a.delay_ms) * 1000u);

    const TwiSpeed speed = a.rate >= 1'000'000u ? TwiSpeed::fast_plus_1m
                         : a.rate >= 400'000u   ? TwiSpeed::fast_400k
                                                : TwiSpeed::standard_100k;
    if (!Host::init(clock, {.speed = speed})) {
        r.flags |= slink::report_failed;
        return r;
    }
    Twi<0>::enable_read_interrupt(false);     // this peer pumps the engine by hand
    Twi<0>::enable_write_interrupt(false);

    uint8_t tx[16];
    const uint8_t n = a.count > 16 ? 16 : static_cast<uint8_t>(a.count);
    for (uint8_t i = 0; i < n; ++i) tx[i] = static_cast<uint8_t>(a.value + i);

    free_run_capture();
    watch_zero();
    (void)Host::start({a.addr, lend<Lease::reply>(tx), n, {}, 0, {}, speed});
    uint8_t status = 0xFE;
    const uint32_t ts = Ticker::millis();
    while (Ticker::millis() - ts < a.deadline_ms) {
        const uint8_t s = Twi<0>::host_status();
        if ((s & (TWI_RIF_bm | TWI_WIF_bm | TWI_ARBLOST_bm | TWI_BUSERR_bm)) &&
            Host::isr()) {
            status = Host::status();
            break;
        }
    }
    r.ticks = Watch::read();
    r.status = status;
    r.count = n;
    if (status == i2c_ok) r.flags |= slink::report_acked;
    if (status == 0xFE) r.flags |= slink::report_timed_out;
    Host::release();
    return r;
}

// ---- the command handler -------------------------------------------------------

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
        auto board = board_id();
        for (uint8_t i = 0; i < 8 && i < board.size(); ++i) d.label[i] = board[i];
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

// ---- the console ---------------------------------------------------------------

void help() {
    print(console, "sleep_peer: ? help | i status | 0 command mode | 3 trace", crlf);
}

void status() {
    print(console, "  link BAUD=", Link::baud_reg(), " route=",
          static_cast<uint8_t>(Link::routed()), crlf);
    print(console, "  commands=", commands, " actions=", actions, " naks=", naks, crlf);
    print(console, "  PE2 stimulus=", StimPin::read() ? 1 : 0,
          " PE3 echo=", EchoPin::read() ? 1 : 0, crlf);
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

ISR(USART2_RXC_vect) { (void)Console::rxc(); }
ISR(USART2_DRE_vect) { Console::dre(); }
ISR(RTC_PIT_vect) { Ticker::pit(); }

// All four TCB vectors are bound as a net: an unbound vector on this
// core is a jump to 0, a silent reset loop.
ISR(TCB0_INT_vect) { (void)Tcb<0>::take_flags(); }
ISR(TCB1_INT_vect) { (void)WatchLo::take_flags(); }
ISR(TCB2_INT_vect) { (void)WatchHi::take_flags(); }
ISR(TCB3_INT_vect) { (void)Tcb<3>::take_flags(); }

// The TWI engine here is PUMPED BY HAND (RIEN/WIEN are cleared right
// after Host::init), but both vectors are bound anyway: an unbound one
// is a reset loop, and a stray flag during a tenure would find it.
ISR(TWI0_TWIM_vect) { (void)Host::isr(); }
ISR(TWI0_TWIS_vect) { (void)Twi<0>::take_client(); }

int main() {
    crystal_ok = SysClock::init();
    Console::init(clock, 460800);
    Ticker::init();

    StimPin::clear();
    StimPin::output();
    EchoPin::input();
    watch_init();
    sei();

    auto board = board_id();
    if (board.empty()) board = "?";
    print(console, crlf, "sleep_peer - sleep instrument peer (board ", board,
          ", clk=", crystal_ok ? "XTAL" : "OSCHF", " 24 MHz, silicon rev ",
          hex(SYSCFG.REVID), ", fw ", hex(firmware_version), ")", crlf);
    print(console, "link USART4 default on the SHARED PE0 wire, 8N1 @ ",
          slink::command_baud, "; PE2 = stimulus out, PE3 = echo in (captured)", crlf);
    print(console, "I2C host on PA2/PA3 for the address-match wake tenure", crlf);
    if (!crystal_ok) {
        print(console, "NOTE: the 24 MHz crystal did not start - CLK_PER and every tick "
                       "reported here come from OSCHF (per-cent class, ample for these "
                       "latencies)", crlf);
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
                    decoder.reset();
                    (void)command_mode();
                    StimPin::clear();
                    StimPin::output();
                    print(console, "  command mode", crlf);
                }
                else if (c == '3') {
                    trace = !trace;
                    print(console, trace ? "  trace on" : "  trace off", crlf);
                }
                else print(console, "? for help", crlf);
                print(console, "> ");
            }
        }

        // A frame that stops half way must not eat the next one: a quiet
        // line for longer than any inter-byte gap resets the reassembly.
        if (decoder.partial() && Ticker::millis() - last_byte_ms > 50u) {
            decoder.reset();
        }
        if (Link::rxc_flag()) {
            const UsartFrame f = Link::receive();
            last_byte_ms = Ticker::millis();
            if (!f.clean()) {
                decoder.reset();
                continue;
            }
            switch (decoder.feed(static_cast<uint8_t>(f.data))) {
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
}
