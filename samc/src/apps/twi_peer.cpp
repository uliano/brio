// twi_peer - the INSTRUMENT half of the I2C campaign on the SAM C21:
// the scriptable second chip on the bus that test_samc_i2c (the DUT)
// drives IN BAND over the very bus both are testing.
//
// A PORT OF avrdx/src/apps/twi_peer.cpp TO THE SECOND ARCHITECTURE,
// over the SAME protocol header (twi_link.hpp, relative path - one
// source of truth for the wire format). What changed is what the
// silicon changed:
//
//  - ONE SERCOM IS HOST *OR* CLIENT (CTRLA.MODE), never both: the
//    AVR's COMBINED mode (host and client halves live at once) does
//    not exist here, so the `arb` action SWITCHES the instance to host
//    for its bounded moment and back - during a host action this
//    board's client is simply absent, which the choreography already
//    tolerates (the DUT waits out the action's deadline either way).
//  - THE CLIENT STRETCHES BY CONSTRUCTION: AMATCH and DRDY hold SCL
//    until software answers (33.10.6), so the commanded per-byte hold
//    is simply a wait spent BEFORE the answer - no register knob.
//  - flag_stop_interrupt maps to nothing: PREC is a polled flag here
//    and the Stop count is kept whenever the action loop sees it.
//  - THE CORE RUNS AT 48 MHz (generator 0), AND THAT IS THE CLEAN
//    WIRE'S BET: the filterless-I2C finding (samc/i2c.md) was the
//    seven-wire bundle's crosstalk, and this desk's I2C pair is short
//    and separate. If this peer answers the command channel, the
//    glitch wall is gone with the bundle; if it sits deaf, the ladder
//    gets re-measured before anything else.
//
// COEXISTENCE is the protocol header's own argument, unchanged: the
// command channel is ONE exact client address (0x6B) - mask 0, no
// general call - so nothing the DUT's wireless letters do can wake
// this board.
//
// Bus: SERCOM3 function C - SDA PA22 = PAD[0], SCL PA23 = PAD[1] - on
// the two-wire node with external pull-ups and the dedicated GND.
//
// Console: SERCOM5 PB30/PB31 at 115200, observability only.
//   ? help | i status and counters | 0 back to command mode | 3 trace
//
// The ident label is the die serial's first word in hex (the spi_peer
// precedent - this family's identity is factory-programmed); ident.xtal
// reports whether the 24 MHz crystal started (probed once at boot).
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include "samc/clock.hpp"
#include "samc/delay.hpp"
#include "samc/i2c.hpp"
#include "samc/nvic.hpp"
#include "samc/nvm.hpp"
#include "samc/pin.hpp"
#include "samc/sercom.hpp"
#include "samc/ticker.hpp"
#include "util/print.hpp"

// THE PROTOCOL IS THE AVR CAMPAIGN'S, AND IT IS NOT COPIED (the
// spi_link ruling: pure encoding, both architectures compile the same
// file).
#include "../../../avrdx/src/apps/twi_link.hpp"

using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock{};

namespace {

using namespace brio;
using twilink::Op;

constexpr UartPads console_pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'B', 30, PinFunction::d},
    .rx_pin = {'B', 31, PinFunction::d},
};
using Console = Uart<5, console_pads>;
constexpr Console console;

constexpr I2cPads bus_pads{
    .sda_pin = {'A', 22, PinFunction::c},
    .scl_pin = {'A', 23, PinFunction::c},
};

/// The clean wire's bet - see the header comment.
constexpr uint8_t core_gen = 0;
constexpr uint32_t bus_rise_ns = 300;

using Client = I2cClient<3, bus_pads, core_gen>;
using Cs = I2cs<3>;
using Host = I2cHost<3, bus_pads, core_gen>;

using Sda = Pin<'A', 22>;
using Scl = Pin<'A', 23>;

constexpr uint16_t firmware_version = 0x0200;   ///< 0x01xx = the AVR peer

bool crystal_ok = false;
bool trace = false;

twilink::Decoder decoder;
twilink::Report last_report;

uint32_t commands = 0, actions = 0, naks = 0, answers = 0;

uint8_t resp_[twilink::response_bytes];
uint8_t resp_len_ = 0;
uint8_t resp_pos_ = 0;
bool read_done_ = false;
Op pending_ = Op::ping;
bool has_pending_ = false;
twilink::Params pending_params_{};

/// The host action's completion, set by SERCOM3_Handler while the
/// instance wears its HOST face.
volatile bool host_live = false;
volatile bool host_done = false;
volatile uint8_t host_status = i2c_ok;

// ---- waits ----------------------------------------------------------------------

void wait_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

/// A microsecond hold of arbitrary length: delay_us is capped below a
/// tick BY DESIGN, and this instrument's commanded stretches (2 ms per
/// byte in the suite) are exactly the blocking waits the cap exists to
/// keep out of KERNEL programs - this is not one, so the hold is spent
/// in chunks the cap admits.
void hold_us(uint32_t us) {
    while (us >= 500u) {
        (void)delay_us(clock, 500u);
        us -= 500u;
    }
    if (us != 0u) {
        (void)delay_us(clock, us);
    }
}

// ---- the two standing configurations --------------------------------------------

/// COMMAND MODE: one exact address, nothing else. Everything returns
/// here.
bool go_command() {
    host_live = false;
    const bool ok = Client::init(clock, {.address_mode = I2cAddressMode::mask,
                                         .address = twilink::command_addr,
                                         .second = 0,
                                         .general_call = false});
    decoder.reset();
    resp_pos_ = 0;
    read_done_ = false;
    return ok;
}

/// What an action asked this client to become.
bool apply_client(const twilink::Params& a) {
    const uint8_t addr = (a.flags & twilink::flag_deaf)
                             ? twilink::deaf_addr
                             : (a.addr ? a.addr : twilink::command_addr);
    return Client::init(clock, {.address_mode = I2cAddressMode::mask,
                                .address = addr,
                                .second = 0,
                                .general_call =
                                    (a.flags & twilink::flag_general_call) != 0});
}

// ---- the command channel --------------------------------------------------------

void prepare(const twilink::Frame& f);
void prepare_nak(Op op, uint8_t sum);

/// One pass of the command-mode client protocol, on the samc verbs:
/// AMATCH resets the decoder (write) or the response cursor (read),
/// DRDY moves one byte each way, and the host's closing NACK - gated
/// past erratum 1.17.22's first-DRDY blind spot - ends the read tenure
/// through CMD 0x2.
void service_command() {
    if (Client::addressed()) {
        if (Client::host_reads()) {
            resp_pos_ = 0;
        } else {
            decoder.reset();
        }
        Client::answer_address(true);
        return;
    }
    if (!Client::data_ready()) {
        return;
    }
    if (Client::host_reads()) {
        const bool first = Client::first_drdy();
        if (!first && Client::host_nacked()) {
            Client::end_transaction();
            read_done_ = true;
            return;
        }
        Client::give(resp_pos_ < resp_len_ ? resp_[resp_pos_++] : 0x00u);
        return;
    }
    const uint8_t v = Client::take(true);
    switch (decoder.feed(v)) {
        case twilink::Decoder::Result::frame:
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

/// ACK BEFORE ACT, the protocol's own choreography: the action starts
/// only once the DUT has collected the acknowledgement.
void prepare(const twilink::Frame& f) {
    ++commands;
    if (trace) {
        print(console, "  [cmd op=", hex(twilink::byte_of(f.op)), " len=", f.len, "]",
              crlf);
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

// ---- the actions ----------------------------------------------------------------

/// Be a client: stretch, NACK, General Call, deafness - and, through
/// `coll`, the shared-address fixed-byte flavour. The commanded hold is
/// spent BEFORE the answer, which is exactly where this silicon
/// stretches SCL (AMATCH/DRDY hold the clock until software speaks).
twilink::Report run_serve(const twilink::Params& a, bool fixed_byte) {
    twilink::Report r{};
    if (!apply_client(a)) return r;
    uint16_t rx = 0, tx = 0;
    bool first_taken = false;
    const uint32_t t0 = Ticker::millis();
    for (;;) {
        if (Ticker::millis() - t0 >= a.ms) {
            r.flags |= twilink::report_timed_out;
            break;
        }
        if (a.count != 0 && static_cast<uint16_t>(rx + tx) >= a.count && r.stops != 0) {
            break;
        }
        if (Client::addressed()) {
            ++r.addr_hits;
            r.last_addr = Client::matched_byte();
            Client::answer_address(true);
            continue;
        }
        if (Client::stop_seen()) {
            if (r.stops < 255) ++r.stops;
            Client::clear_stop();
            continue;
        }
        if (!Client::data_ready()) {
            continue;
        }
        if (a.hold_us) hold_us(a.hold_us);
        if (Client::host_reads()) {
            const bool first = Client::first_drdy();
            if (!first && Client::host_nacked()) {
                // SSTATUS sampled AT the closing NACK - RXNACK is live
                // and a later copy carries another tenure's answer (the
                // AVR peer's own lesson, held here too).
                r.mstatus = static_cast<uint8_t>(Client::raw_status());
                Client::end_transaction();
                continue;
            }
            Client::give(fixed_byte ? a.seed
                                    : twilink::pattern_value(a.pattern, a.seed, tx));
            ++tx;
            continue;
        }
        const bool nack = a.nack_at != 0 && static_cast<uint16_t>(rx + 1) >= a.nack_at;
        const uint8_t v = Client::take(!nack);
        if (nack) r.flags |= twilink::report_nacked;
        if (!first_taken) {
            r.first = v;
            first_taken = true;
        }
        r.sum = static_cast<uint16_t>(r.sum + v);
        ++rx;
    }
    r.count = static_cast<uint16_t>(rx + tx);
    r.aux0 = static_cast<uint8_t>(rx);
    r.aux1 = static_cast<uint8_t>(tx);
    r.sstatus = static_cast<uint8_t>(Client::raw_status());
    if (Client::collision()) r.flags |= twilink::report_coll;
    return r;
}

/// The HOST action - and on this silicon it is a MODE SWITCH, not a
/// second half (see the header). The shape serves both of the suite's
/// needs with one code path: with a LEAD (aux16) it is letter h's
/// foreign host (wait for the DUT's traffic to prove the rendezvous,
/// give the DUT its window to become a client, then write the burst);
/// with lead 0 the ADDR write lands while the bus is still BUSY and
/// PARKS IN HARDWARE - the held START, which is the arbitration race's
/// own arming (measured on this silicon by the campaign: a tenure into
/// a busy bus parks, and ARBLOST is a status the engine reports).
twilink::Report run_arb(const twilink::Params& a) {
    twilink::Report r{};
    Client::release();
    host_live = true;
    if (!Host::init(clock, bus_rise_ns)) {
        host_live = false;
        (void)go_command();
        return r;
    }

    // The rendezvous: the DUT makes the bus busy (its own tenure, or a
    // bit-banged hold) and this end catches it.
    const uint32_t t0 = Ticker::millis();
    bool armed = false;
    while (Ticker::millis() - t0 < a.ms) {
        if (I2cm<3>::bus_state() == I2cBusState::busy) {
            armed = true;
            break;
        }
    }
    if (!armed) {
        r.flags |= twilink::report_timed_out;
        host_live = false;
        (void)go_command();
        return r;
    }
    if (a.aux16) {
        // The lead-in, in MILLISECOND chunks (it arrives in us).
        wait_ms(a.aux16 / 1000u);
    }

    static uint8_t burst[32];
    const uint8_t n = a.count < sizeof burst ? static_cast<uint8_t>(a.count)
                                             : static_cast<uint8_t>(sizeof burst);
    for (uint8_t i = 0; i < n; ++i) {
        burst[i] = twilink::pattern_value(a.pattern, a.seed, i);
    }
    host_done = false;
    r.flags |= twilink::report_host_ran;
    const bool sync_fail = Host::start({.addr = a.target,
                                        .tx = lend<Lease::reply>(
                                            static_cast<const uint8_t*>(burst)),
                                        .tx_len = n,
                                        .rx = {},
                                        .rx_len = 0,
                                        .reply = {},
                                        .speed = I2cSpeed::standard_100k});
    if (sync_fail) {
        r.mstatus = Host::status();
    } else {
        while (!host_done && Ticker::millis() - t0 < a.ms) {
        }
        if (!host_done) {
            r.flags |= twilink::report_timed_out;
            Host::release();
        } else {
            r.mstatus = host_status;
            if (host_status == i2c_ok) r.flags |= twilink::report_host_ok;
            if (host_status == i2c_arb_lost) r.flags |= twilink::report_arblost;
            if (host_status == i2c_bus_error) r.flags |= twilink::report_buserr;
            r.aux1 = n;
        }
    }
    host_live = false;
    (void)go_command();
    return r;
}

/// A STUCK client, bit-banged: SDA held low from PORT with the SERCOM
/// off the pads, released after `aux8` SCL falling edges (0 = only the
/// deadline). Open drain by hand: an OUTPUT over a clear OUT bit pulls
/// the line down, an INPUT gives it back to the pull-up.
twilink::Report run_hold_sda(const twilink::Params& a) {
    twilink::Report r{};
    Client::release();
    Sda::clear();
    Scl::clear();
    Scl::input();
    Sda::output();
    r.flags |= twilink::report_host_ran;

    const uint32_t t0 = Ticker::millis();
    uint16_t falls = 0;
    bool prev = Scl::read();
    uint16_t guard = 0;
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
    Sda::input();
    r.count = falls;
    r.aux0 = static_cast<uint8_t>(falls);
    r.aux1 = Sda::read() ? 1u : 0u;
    return r;
}

/// This board off the wire entirely for the deadline: the control case.
twilink::Report run_quiet(const twilink::Params& a) {
    twilink::Report r{};
    Client::release();
    wait_ms(a.ms);
    r.count = 1;
    return r;
}

void run_pending() {
    const Op op = pending_;
    has_pending_ = false;

    if (op == Op::ping) return;
    if (op == Op::ident) {
        twilink::Ident d{};
        const uint32_t w0 = DeviceSerial::read().word[0];
        for (uint8_t i = 0; i < 8; ++i) {
            const uint8_t nib = static_cast<uint8_t>((w0 >> (28u - 4u * i)) & 0xFu);
            d.label[i] = static_cast<char>(nib < 10 ? '0' + nib : 'a' + nib - 10);
        }
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
        case Op::serve: r = run_serve(a, false); break;
        case Op::coll: r = run_serve(a, true); break;
        case Op::arb: r = run_arb(a); break;
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

// ---- the console ----------------------------------------------------------------

void help() {
    print(console, "twi_peer: ? help | i status | 0 back to command mode | 3 trace",
          crlf);
}

void status() {
    print(console, "  SERCOM3 fn C, client enabled=", Cs::enabled(),
          " ADDR=", hex(Cs::addr_reg()), " CTRLA=", hex(Cs::ctrla()), crlf);
    print(console, "  SDA reads ", Sda::read() ? "high" : "LOW", ", SCL reads ",
          Scl::read() ? "high" : "LOW", ", SSTATUS=", hex(Cs::status()),
          " INTFLAG=", hex(Cs::flags()), crlf);
    print(console, "  commands=", commands, " actions=", actions, " naks=", naks,
          " answers=", answers, crlf);
    print(console, "  last report: op=", hex(last_report.op),
          " count=", last_report.count, " rx/tx=", last_report.aux0, "/",
          last_report.aux1, " hits=", last_report.addr_hits,
          " stops=", last_report.stops, " flags=", hex(last_report.flags), crlf);
    print(console, "               sum=", hex(last_report.sum),
          " first=", hex(last_report.first), " lastaddr=", hex(last_report.last_addr),
          " SSTATUS=", hex(last_report.sstatus),
          " MSTATUS=", hex(last_report.mstatus), " ms=", last_report.ms, crlf);
}

}  // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void SERCOM5_Handler() { (void)Console::isr(); }

/// One vector for the SERCOM: the client is POLLED (nothing armed), so
/// the only interrupts here are the HOST action's engine.
extern "C" void SERCOM3_Handler() {
    if (host_live) {
        if (Host::isr()) {
            host_status = Host::status();
            host_done = true;
        }
        return;
    }
    // A stray level with neither face armed: disarm it rather than spin.
    brio::I2cm<3>::enable_interrupt(brio::I2cmFlag::all, false);
    Cs::enable_interrupt(brio::I2csFlag::all, false);
}

int main() {
    SysClock::init();
    Console::init(clock, 115200);
    Ticker::init(clock);
    brio::enable_interrupts();

    // GCLK_SERCOM_SLOW (channel shared by SERCOM0..4), routed as the
    // suite routes it: 33.5.3 wants it configured before I2C use.
    (void)brio::GclkChannel::connect(brio::Sercom<3>::gclk_slow_id(), core_gen);

    // The crystal probe, for ident.xtal (the spi_peer arrangement).
    crystal_ok = brio::Xosc::init(brio::XoscConfig{.hz = 24'000'000UL, .startup = 4,
                                                   .on_demand = false});

    char label8[9] = {};
    {
        const uint32_t w0 = brio::DeviceSerial::read().word[0];
        for (uint8_t i = 0; i < 8; ++i) {
            const uint8_t nib = static_cast<uint8_t>((w0 >> (28u - 4u * i)) & 0xFu);
            label8[i] = static_cast<char>(nib < 10 ? '0' + nib : 'a' + nib - 10);
        }
    }
    print(console, crlf, "twi_peer - I2C instrument client (SAM C21 board ", label8,
          ", xtal=", crystal_ok ? "up" : "DOWN", ", fw ", hex(firmware_version), ")",
          crlf);
    print(console, "client SERCOM3 fn C: SDA PA22, SCL PA23; command address ",
          hex(twilink::command_addr), " exactly - no general call, no mask; the "
          "core runs at 48 MHz (the clean wire's bet)", crlf);
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
