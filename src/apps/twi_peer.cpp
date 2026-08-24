// twi_peer - the INSTRUMENT half of the TWI campaign: board B, the
// scriptable second chip on the I2C bus that test_avr_twi (board A, the
// DUT) drives IN BAND over the very bus both are testing.
//
// It is deliberately not a kernel app: one blocking loop around the
// polled TwiClient surface that answers exactly one address, decodes a
// command frame (src/apps/twi_link.hpp), acknowledges it and then
// becomes for a bounded moment whatever the DUT needs at the other end
// of the wire:
//
//   - a client that CLOCK STRETCHES by a commanded number of
//     microseconds per data byte;
//   - a client that NACKs the n-th byte of a write, or that is not
//     there at all (its address parked elsewhere) so the DUT meets an
//     address NACK from a board that is otherwise alive;
//   - a client that answers the General Call address as well as its own,
//     or that stops answering it;
//   - a second HOST racing the DUT for the bus: both boards run
//     COMBINED (host and client on the same pins), both arm a START
//     while the bus is Busy, and the hardware releases the two of them
//     on the same edge - real wired-AND arbitration, with the winner
//     decided by which address byte is smaller;
//   - a second CLIENT sharing ONE address with the DUT's own, so a read
//     is served by two devices at once and the one that tries to put a
//     high bit against the other's low raises COLL (case S4);
//   - a STUCK client: SDA held low from PORT, released after a
//     commanded number of SCL falling edges, which is what makes the
//     DUT's Twi<0>::unstick() a measurement instead of a hope.
//
// Every action carries a byte count and a millisecond deadline, after
// which the peer restores its command-mode client BY ITSELF.
//
// COEXISTENCE - the one thing to understand before touching this file.
// I2C addresses, so this peer needs none of the SPI campaign's dark
// listener gymnastics: in command mode its client answers ONE address
// (twilink::command_addr, which no test of the DUT's single-board half
// sends, general-calls or mask-matches), with General Call off, no mask,
// no second address, PMEN off, Smart mode off and even PIEN off - so the
// only thing on this board that can raise a flag while `test_avr_twi z`
// runs is an address packet naming that address. The proof is that `z`
// still scores its full 175 with this firmware attached and running.
//
// Bus: TWI0 DEFAULT, SDA PA2 / SCL PA3, on the desk's one open-drain
// node with 1.5k pull-ups to +5 V and a dedicated GND wire between the
// boards.
//
// Console: USART2 ALT1 (PF4/PF5) at 460800, observability only.
//   ? help | i status and counters | 0 back to command mode | 3 trace
//
// The banner reports the USERROW board label and whether THIS board's
// 24 MHz crystal started: the peer's CLK_PER decides how fast its polled
// client can turn a byte around before it starts stretching, so its
// clock is itself a bench fact and the DUT asks for it with `ident`.

// pio: monitor_speed = 460800

#include <avr/interrupt.h>
#include <stdint.h>

#include "avrdx/clock.hpp"
#include "avrdx/delay.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/ticker.hpp"
#include "avrdx/twi.hpp"
#include "avrdx/usart.hpp"
#include "avrdx/userrow.hpp"
#include "util/print.hpp"

#include "twi_link.hpp"

using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;
using twilink::Op;

using Console = Uart<2, Route::alt1>;
constexpr Console console;

using T = Twi<0>;
using Client = TwiClient<0, TwiRoute::def>;
using Host = TwiHost<0, TwiRoute::def>;

using Sda = Pin<'A', 2>;
using Scl = Pin<'A', 3>;

constexpr uint16_t firmware_version = 0x0100;

bool crystal_ok = false;
bool trace = false;

twilink::Decoder decoder;
twilink::Report last_report;

uint32_t commands = 0, actions = 0, naks = 0, answers = 0;

// The response the DUT's next READ tenure will collect, and what to do
// once it has collected it.
uint8_t resp_[twilink::response_bytes];
uint8_t resp_len_ = 0;
uint8_t resp_pos_ = 0;
bool read_done_ = false;
Op pending_ = Op::ping;
bool has_pending_ = false;
twilink::Params pending_params_{};

const uint8_t cpu = cycles_per_us(SysClock::hz);

// ---- the two standing configurations -------------------------------------------

/// COMMAND MODE: one exact address, General Call off, no mask, no second
/// address, PMEN off, Smart mode off, PIEN off. Everything returns here.
bool go_command() {
    // The arbitration action leaves the HOST half up; taking it down
    // here is what makes Client::init re-write the whole instance
    // instead of joining a running host in combined mode.
    T::host_enable(false);
    const bool ok = Client::init(clock, {.address = twilink::command_addr,
                                         .general_call = false,
                                         .address_mask = 0,
                                         .second_address = false,
                                         .promiscuous = false,
                                         .smart = false,
                                         .stop_interrupt = false});
    decoder.reset();
    resp_pos_ = 0;
    read_done_ = false;
    return ok;
}

/// What an action asked this client to become. `flag_deaf` parks the
/// address where nobody sends, which is how an ADDRESS NACK is produced.
bool apply_client(const twilink::Params& a) {
    const uint8_t addr = (a.flags & twilink::flag_deaf)
                             ? twilink::deaf_addr
                             : (a.addr ? a.addr : twilink::command_addr);
    return Client::init(clock, {.address = addr,
                                .general_call = (a.flags & twilink::flag_general_call) != 0,
                                .smart = false,
                                .stop_interrupt = (a.flags & twilink::flag_stop_interrupt) != 0});
}

// ---- the command channel --------------------------------------------------------

void prepare(const twilink::Frame& f);
void prepare_nak(Op op, uint8_t sum);

/// One pass of the command-mode client protocol. A WRITE tenure carries
/// one command frame (the decoder is reset at the address packet, so a
/// truncated frame can never eat the next one); a READ tenure collects
/// the prepared response, and the host's closing NACK is what says the
/// answer has been taken.
void service_command() {
    const auto s = Client::isr();
    if (s.address_or_stop()) {
        if (s.is_address()) {
            if (s.host_reading()) resp_pos_ = 0;
            else decoder.reset();
            Client::respond(TwiAck::ack);
        } else {
            Client::complete();
        }
        return;
    }
    if (!s.data()) return;
    if (s.host_reading()) {
        if (resp_pos_ != 0 && s.nack()) {   // RXACK is stale before the first byte
            Client::complete();
            read_done_ = true;
            return;
        }
        Client::transmit(resp_pos_ < resp_len_ ? resp_[resp_pos_++] : 0x00u);
        return;
    }
    const uint8_t v = Client::receive(TwiAck::ack);
    switch (decoder.feed(v)) {
        case twilink::Decoder::Result::frame:
            // A frame that checks out but names an op this instrument
            // does not have is NAKed, not silently dropped: on an
            // ADDRESSED bus there is nobody else the answer could
            // disturb, and a nak tells the DUT the difference between
            // "not understood" and "not heard".
            if (twilink::is_command(decoder.frame().op)) prepare(decoder.frame());
            else prepare_nak(decoder.frame().op, decoder.frame().sum);
            break;
        case twilink::Decoder::Result::bad_checksum:
            prepare_nak(decoder.pending_op(), decoder.frame().sum);
            break;
        default: break;
    }
}

void set_response(Op op, const uint8_t* p, uint8_t len) {
    uint8_t n = 0;
    twilink::write_frame(
        [&](uint8_t b) {
            if (n < sizeof resp_) resp_[n++] = b;
        },
        op, p, len);
    resp_len_ = n;
    resp_pos_ = 0;
    ++answers;
}

void prepare_nak(Op op, uint8_t sum) {
    const uint8_t p[2] = {twilink::byte_of(op), sum};
    set_response(Op::nak, p, 2);
    has_pending_ = false;
    ++naks;
}

/// ACK BEFORE ACT: the acknowledgement is prepared here and goes out on
/// the DUT's next read; the action itself starts only once that read has
/// completed, so the DUT knows the peer is armed before it puts a single
/// measured edge on the wire.
void prepare(const twilink::Frame& f) {
    ++commands;
    if (trace) {
        print(console, "  [cmd op=", hex(twilink::byte_of(f.op)), " len=", f.len, "]", crlf);
    }
    if (twilink::is_action(f.op) && f.len < twilink::params_size) {
        prepare_nak(f.op, f.sum);
        return;
    }
    const uint8_t p[2] = {twilink::byte_of(f.op), f.sum};
    set_response(Op::ack, p, 2);
    pending_ = f.op;
    has_pending_ = true;
    if (twilink::is_action(f.op)) pending_params_ = twilink::get_params(f.data);
}

// ---- the actions ------------------------------------------------------------------

/// Be a client: the whole point of the stretch, NACK, General Call and
/// deafness cases. The hold is applied BEFORE the flag is cleared, which
/// is exactly where the silicon stretches SCL.
twilink::Report run_serve(const twilink::Params& a) {
    twilink::Report r{};
    if (!apply_client(a)) return r;
    const uint16_t hold = a.hold_us;
    uint16_t rx = 0, tx = 0;
    // RXACK is a LIVE bit that survives a transaction: at the first data
    // byte of a new one it still carries the previous host's closing
    // NACK. So the "has the host had enough" test counts bytes served
    // SINCE THE LAST ADDRESS PACKET, not since the action began - `tx`
    // is the stream index and must not be reset.
    uint16_t served_here = 0;
    bool first_taken = false;
    const uint32_t t0 = Ticker::millis();
    for (;;) {
        if (Ticker::millis() - t0 >= a.ms) { r.flags |= twilink::report_timed_out; break; }
        if (a.count != 0 && static_cast<uint16_t>(rx + tx) >= a.count && r.stops != 0) break;
        const auto s = Client::isr();
        if (s.address_or_stop()) {
            if (s.is_address()) {
                ++r.addr_hits;
                served_here = 0;
                r.last_addr = Client::last_address();
                Client::respond(TwiAck::ack);
            } else {
                if (r.stops < 255) ++r.stops;
                Client::complete();
            }
            continue;
        }
        if (!s.data()) continue;
        if (hold) delay_us_runtime(cpu, hold);
        if (s.host_reading()) {
            if (served_here != 0 && s.nack()) {
                // SSTATUS sampled AT the host's closing NACK: RXACK is
                // a live bit, and the copy taken when the action ends
                // carries whatever the last transaction left behind.
                r.mstatus = s.status;
                Client::complete();
                continue;
            }
            Client::transmit(twilink::pattern_value(a.pattern, a.seed, tx));
            ++tx;
            ++served_here;
            continue;
        }
        const bool nack = a.nack_at != 0 && static_cast<uint16_t>(rx + 1) >= a.nack_at;
        const uint8_t v = Client::receive(nack ? TwiAck::nack : TwiAck::ack);
        if (nack) r.flags |= twilink::report_nacked;
        if (!first_taken) { r.first = v; first_taken = true; }
        r.sum = static_cast<uint16_t>(r.sum + v);
        ++rx;
    }
    r.count = static_cast<uint16_t>(rx + tx);
    r.aux0 = static_cast<uint8_t>(rx);
    r.aux1 = static_cast<uint8_t>(tx);
    r.sstatus = T::client_status();
    if (Client::collision()) r.flags |= twilink::report_coll;
    if (Client::bus_error()) r.flags |= twilink::report_buserr;
    return r;
}

/// The multi-host case, and the reason it is deterministic.
///
/// A software rendezvous cannot put two STARTs on a bus inside the
/// hundred nanoseconds that a genuine race needs. The hardware can: a
/// host told to start while the bus is BUSY HOLDS its START and releases
/// it by itself the moment the bus goes Idle (29.3.2.2.3). So both
/// boards arm a START against a bus the DUT's bit-bang injector has made
/// Busy, and the DUT's injected STOP releases BOTH of them on the same
/// edge. What follows is real wired-AND arbitration on the address, and
/// the winner is simply whichever board is transmitting the smaller
/// address byte - which is what lets the DUT choose the loser and prove
/// the case in BOTH directions.
twilink::Report run_arb(const twilink::Params& a) {
    twilink::Report r{};
    if (!apply_client(a)) return r;
    if (!Host::init(clock, {.speed = TwiSpeed::standard_100k})) return r;
    T::enable_read_interrupt(false);
    T::enable_write_interrupt(false);

    const uint32_t t0 = Ticker::millis();
    // Wait for the DUT's injector to make the bus Busy, then arm.
    bool armed = false;
    while (Ticker::millis() - t0 < a.ms) {
        if (T::bus_state() == TwiBusState::busy) { armed = true; break; }
    }
    if (!armed) {
        r.flags |= twilink::report_timed_out;
        r.mstatus = T::host_status();
        return r;
    }
    if (a.aux16) delay_us_runtime(cpu, a.aux16);
    T::clear_host_flags(TWI_ARBLOST_bm | TWI_BUSERR_bm | TWI_WIF_bm | TWI_RIF_bm);
    T::address_write(a.target);             // held until the bus is Idle again
    r.flags |= twilink::report_host_ran;

    uint16_t sent = 0, rx = 0;
    bool done = false;
    bool first_taken = false;
    while (!done && Ticker::millis() - t0 < a.ms) {
        // The client half must stay serviced: if THIS board loses, the
        // other one's bytes are about to arrive here.
        const auto s = Client::isr();
        if (s.address_or_stop()) {
            if (s.is_address()) {
                ++r.addr_hits;
                r.last_addr = Client::last_address();
                Client::respond(TwiAck::ack);
            } else {
                if (r.stops < 255) ++r.stops;
                Client::complete();
            }
        } else if (s.data()) {
            if (s.host_reading()) {
                Client::transmit(twilink::pattern_value(a.pattern, a.seed, 0));
            } else {
                const uint8_t v = Client::receive(TwiAck::ack);
                if (!first_taken) { r.first = v; first_taken = true; }
                r.sum = static_cast<uint16_t>(r.sum + v);
                ++rx;
            }
        }

        const uint8_t m = T::host_status();
        if (m & TWI_ARBLOST_bm) {
            r.flags |= twilink::report_arblost;
            r.mstatus = m;
            T::clear_host_flags(TWI_ARBLOST_bm | TWI_WIF_bm);
            done = true;
        } else if (m & TWI_BUSERR_bm) {
            r.flags |= twilink::report_buserr;
            r.mstatus = m;
            T::clear_host_flags(TWI_BUSERR_bm | TWI_WIF_bm);
            T::force_idle();
            done = true;
        } else if (m & TWI_WIF_bm) {
            if (m & TWI_RXACK_bm) {         // nobody there, or the byte refused
                r.mstatus = m;
                T::host_command(TwiHostCmd::stop);
                done = true;
            } else if (sent < a.count) {
                T::host_write(twilink::pattern_value(a.pattern, a.seed, sent));
                ++sent;
            } else {
                r.mstatus = m;
                r.flags |= twilink::report_host_ok;
                T::host_command(TwiHostCmd::stop);
                done = true;
            }
        }
    }
    if (!done) r.flags |= twilink::report_timed_out;
    // Whoever won, let the client see the closing STOP and drain.
    const uint32_t t1 = Ticker::millis();
    while (Ticker::millis() - t1 < 3u) {
        const auto s = Client::isr();
        if (s.address_or_stop()) {
            if (s.is_address()) {
                ++r.addr_hits;
                r.last_addr = Client::last_address();
                Client::respond(TwiAck::ack);
            } else {
                if (r.stops < 255) ++r.stops;
                Client::complete();
            }
        } else if (s.data()) {
            if (s.host_reading()) {
                Client::transmit(twilink::pattern_value(a.pattern, a.seed, 0));
            } else {
                const uint8_t v = Client::receive(TwiAck::ack);
                if (!first_taken) { r.first = v; first_taken = true; }
                r.sum = static_cast<uint16_t>(r.sum + v);
                ++rx;
            }
        }
    }
    r.count = rx;
    r.aux0 = static_cast<uint8_t>(rx);
    r.aux1 = static_cast<uint8_t>(sent);
    r.sstatus = T::client_status();
    if (Client::collision()) r.flags |= twilink::report_coll;
    return r;
}

/// Case S4: this client shares ONE address with the DUT's own, both ACK
/// the same read and both put a byte on the wire. SDA is the AND of the
/// two, so the client that tried a high bit against the other's low
/// cannot drive it, raises COLL and un-drives for the rest of the
/// transaction. COLL is sampled INSIDE the loop: it clears on the next
/// Start condition, so a look taken afterwards can miss it entirely.
twilink::Report run_coll(const twilink::Params& a) {
    twilink::Report r{};
    if (!apply_client(a)) return r;
    uint16_t tx = 0, served_here = 0;
    const uint32_t t0 = Ticker::millis();
    for (;;) {
        if (Ticker::millis() - t0 >= a.ms) { r.flags |= twilink::report_timed_out; break; }
        // Ending at the STOP of the transaction that was asked for keeps
        // the peer available: the DUT collects the report immediately
        // instead of waiting out the deadline.
        if (a.count != 0 && tx >= a.count && r.stops != 0) break;
        if (Client::collision()) r.flags |= twilink::report_coll;
        const auto s = Client::isr();
        if (s.address_or_stop()) {
            if (s.is_address()) {
                ++r.addr_hits;
                served_here = 0;
                r.last_addr = Client::last_address();
                Client::respond(TwiAck::ack);
            } else {
                if (r.stops < 255) ++r.stops;
                Client::complete();
            }
            continue;
        }
        if (!s.data()) continue;
        if (s.collision()) r.flags |= twilink::report_coll;
        if (s.host_reading()) {
            // RXACK survives the previous transaction: count what was
            // served SINCE the last address packet.
            if (served_here != 0 && s.nack()) { Client::complete(); continue; }
            Client::transmit(a.seed);
            ++tx;
            ++served_here;
            continue;
        }
        const uint8_t v = Client::receive(TwiAck::ack);
        r.sum = static_cast<uint16_t>(r.sum + v);
    }
    r.count = tx;
    r.aux0 = static_cast<uint8_t>(tx);
    r.sstatus = T::client_status();
    if (Client::collision()) r.flags |= twilink::report_coll;
    Client::clear_collision();
    return r;
}

/// A STUCK client, bit-banged: SDA held low from PORT with the TWI out
/// of the way, released after `aux8` SCL falling edges (0 = only the
/// deadline releases it). That is what a client interrupted mid-byte
/// really does, and it is what makes the DUT's unstick() a measurement -
/// the line comes back because the recovery CLOCKED it back, not because
/// a timer happened to expire.
///
/// Open drain by hand, as everywhere on this bus: an OUTPUT over a clear
/// OUT bit pulls the line down, an INPUT lets the pull-up have it. The
/// OUT bits are never set, so the pins are left exactly as errata 2.15.1
/// wants the next owner of PA2/PA3 to find them.
twilink::Report run_hold_sda(const twilink::Params& a) {
    twilink::Report r{};
    T::release();
    Sda::clear();
    Scl::clear();
    Scl::input();
    Sda::output();                          // SDA low: nobody can START now
    r.flags |= twilink::report_host_ran;

    // The edge sampling has to be TIGHT - unstick() pulses at 100 kHz,
    // so SCL is low for five microseconds at a time - which is why the
    // deadline is only looked at every 256 samples instead of every one.
    const uint32_t t0 = Ticker::millis();
    uint16_t falls = 0;
    bool prev = Scl::read();
    uint8_t guard = 0;
    for (;;) {
        const bool now = Scl::read();
        if (prev && !now) {
            ++falls;
            if (a.aux8 != 0 && falls >= a.aux8) break;
        }
        prev = now;
        if (++guard == 0 && Ticker::millis() - t0 >= a.ms) {
            r.flags |= twilink::report_timed_out;
            break;
        }
    }
    Sda::input();                           // released
    r.count = falls;
    r.aux0 = static_cast<uint8_t>(falls);
    r.aux1 = Sda::read() ? 1u : 0u;
    return r;
}

/// Board B off the wire entirely, for as long as the deadline says: the
/// control case for every measurement the DUT takes with the peer
/// attached.
twilink::Report run_quiet(const twilink::Params& a) {
    twilink::Report r{};
    T::release();
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < a.ms) {
    }
    r.count = 1;
    return r;
}

void run_pending() {
    const Op op = pending_;
    has_pending_ = false;

    if (op == Op::ping) return;
    if (op == Op::ident) {
        twilink::Ident d{};
        auto board = board_id();
        for (uint8_t i = 0; i < 8 && i < board.size(); ++i) d.label[i] = board[i];
        d.xtal = crystal_ok ? 1 : 0;
        d.sanity = twilink::ident_sanity;
        d.version = firmware_version;
        uint8_t p[twilink::ident_size];
        twilink::put_ident(p, d);
        set_response(Op::ident_data, p, twilink::ident_size);
        return;
    }
    if (op == Op::report) {
        uint8_t p[twilink::report_size];
        twilink::put_report(p, last_report);
        set_response(Op::report_data, p, twilink::report_size);
        return;
    }

    ++actions;
    const twilink::Params a = pending_params_;
    const uint32_t started = Ticker::millis();
    twilink::Report r{};
    switch (op) {
        case Op::serve: r = run_serve(a); break;
        case Op::arb: r = run_arb(a); break;
        case Op::coll: r = run_coll(a); break;
        case Op::hold_sda: r = run_hold_sda(a); break;
        case Op::quiet: r = run_quiet(a); break;
        default: break;
    }
    r.flags |= twilink::report_ran;
    const uint32_t took = Ticker::millis() - started;
    r.ms = took > 255u ? 255u : static_cast<uint8_t>(took);
    r.op = twilink::byte_of(op);
    last_report = r;
    (void)go_command();
}

// ---- the console --------------------------------------------------------------------

void help() {
    print(console, "twi_peer: ? help | i status | 0 back to command mode | 3 trace", crlf);
}

void status() {
    print(console, "  TWI0 DEFAULT, route=", static_cast<uint8_t>(T::routed()),
          " host=", T::host_enabled(), " client=", T::client_enabled(),
          " addr=", hex(T::client_address()), " gc=", T::general_call(),
          " pmen=", T::promiscuous(), " dual=", T::dual_mode(), crlf);
    print(console, "  SDA reads ", Sda::read() ? "high" : "LOW", ", SCL reads ",
          Scl::read() ? "high" : "LOW", ", MSTATUS=", hex(T::host_status()),
          " SSTATUS=", hex(T::client_status()), crlf);
    print(console, "  commands=", commands, " actions=", actions, " naks=", naks,
          " answers=", answers, crlf);
    print(console, "  last report: op=", hex(last_report.op), " count=", last_report.count,
          " rx/tx=", last_report.aux0, "/", last_report.aux1,
          " hits=", last_report.addr_hits, " stops=", last_report.stops,
          " flags=", hex(last_report.flags), crlf);
    print(console, "               sum=", hex(last_report.sum), " first=",
          hex(last_report.first), " lastaddr=", hex(last_report.last_addr),
          " SSTATUS=", hex(last_report.sstatus), " MSTATUS=", hex(last_report.mstatus),
          " ms=", last_report.ms, crlf);
}

}  // namespace

ISR(USART2_RXC_vect) { (void)Console::rxc(); }
ISR(USART2_DRE_vect) { Console::dre(); }
ISR(RTC_PIT_vect) { Ticker::pit(); }

int main() {
    crystal_ok = SysClock::init();
    Console::init(clock, 460800);
    Ticker::init();
    sei();

    auto board = board_id();
    if (board.empty()) board = "?";
    print(console, crlf, "twi_peer - TWI instrument client (board ", board,
          ", clk=", crystal_ok ? "XTAL" : "OSCHF", " 24 MHz, silicon rev ",
          hex(SYSCFG.REVID), ", fw ", hex(firmware_version), ")", crlf);
    print(console, "client TWI0 DEFAULT: SDA PA2, SCL PA3; command address ",
          hex(twilink::command_addr), " exactly - no general call, no mask, no PMEN, "
          "no PIEN", crlf);
    if (!crystal_ok) {
        print(console, "WARNING: the 24 MHz crystal did NOT start - this client turns a "
                       "byte around at OSCHF's rate, not the crystal's", crlf);
    }
    (void)go_command();
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
                    has_pending_ = false;
                    (void)go_command();
                    print(console, "  command-mode client restored", crlf);
                } else if (c == '3') {
                    trace = !trace;
                    print(console, trace ? "  trace on" : "  trace off", crlf);
                } else {
                    print(console, "? for help", crlf);
                }
                print(console, "> ");
            }
        }

        service_command();

        if (read_done_) {
            read_done_ = false;
            if (has_pending_) run_pending();
        }
    }
}
