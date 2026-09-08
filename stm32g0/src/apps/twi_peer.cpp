// twi_peer - the INSTRUMENT half of the I2C campaign on the STM32G0:
// the scriptable second chip on the bus that test_stm32_i2c (the DUT, on
// the OTHER board) drives IN BAND over the very bus both are testing.
//
// A PORT OF samc21/src/apps/twi_peer.cpp TO THE THIRD ARCHITECTURE, over
// the SAME protocol header (twi_link.hpp, relative path - one source of
// truth for the wire format, three architectures compiling it). What
// changed is what the silicon changed:
//
//  - ONE I2C IS HOST *OR* TARGET, as on the SAM: this peripheral has one
//    CR2 and one state machine, so the `arb` action SWITCHES the
//    instance to controller for its bounded moment and back. During a
//    host action this board's target is simply absent, which the
//    choreography already tolerates (the DUT waits out the action's
//    deadline either way).
//  - THE TARGET STRETCHES BY CONSTRUCTION: ADDR, RXNE and TXIS hold SCL
//    until software answers (32.4.8), so the commanded per-byte hold is
//    simply a wait spent BEFORE the answer - no register knob.
//  - A TARGET CANNOT NACK ITS OWN ADDRESS here: the address is
//    acknowledged by the hardware before ADDR rises, so "deaf" is a
//    re-init at `deaf_addr` (the samc21 port's answer, for the same
//    reason). Refusing a byte is a DATA-phase act and rides TARGET BYTE
//    CONTROL - SBC with NBYTES re-armed to one per byte, the decision
//    taken at TCR, which is the pump test_stm32_i2c proved on this
//    silicon and this one follows to the letter.
//  - flag_stop_interrupt maps to nothing: STOPF is a polled flag here
//    and the Stop count is kept whenever the action loop sees it.
//  - A TARGET TRANSMITTER IS ASKED FOR ONE BYTE MORE THAN THE
//    CONTROLLER TAKES, and the tally has to know: TXIS rises as soon as
//    TXDR empties, so the byte standing there when the closing NACK
//    arrives never reaches the wire - flush() throws it away (32.4.8)
//    and run_serve() gives it back, or this instrument would report
//    nine bytes served for eight clocked. Its twin: THE LAST DATA
//    BYTE'S STRETCH FALLS OUTSIDE THE CONTROLLER'S OWN TENURE (a write
//    is over once its last byte is in the shifter), so the commanded
//    hold is spent at the ADDRESS MATCH as well as before every data
//    byte - which is where 2 ms a byte prices an 8-byte tenure at the
//    16 ms the model predicts instead of 14.
//  - THE CLIENT'S SPEED IS ITS SDADEL/SCLDEL AND NOT A RATE. 32.4.8
//    makes those the delays a TARGET applies, so `speed` names the
//    FASTEST bus this end expects to sit on. The DUT's letter `q` walks
//    100k, 400k and 1M against this one configuration and the peer
//    cannot know which rung is coming, so it is solved for Fm+ - the
//    fastest of the three - and the same delays serve the slower rungs.
//
// COEXISTENCE is the protocol header's own argument, unchanged: the
// command channel is ONE exact target address (0x6B) - no mask, no
// general call, no second address - so nothing the DUT's wireless
// letters do can wake this board.
//
// Bus: I2C1 at AF6, open drain, the SAME PIN NAMES the DUT hosts on -
//   PB8  SCL
//   PB9  SDA
// on the two-wire node with its external 2.2 kOhm pull-ups and a
// dedicated GND. The kernel clock is PCLK (64 MHz).
//
// Console: USART2 PA2/PA3 at 115200 on the ST-LINK's virtual COM port,
// observability only.
//   ? help | i status and counters | 0 back to command mode | 3 trace
//
// The ident label is the 96-bit unique device ID's first word in hex
// (RM0444 41.1 through brio::DeviceUid - the spi_peer arrangement);
// ident.xtal is ALWAYS 0, because a Nucleo-64 fits no HSE crystal and
// this stratum has no HSE root at all.
//
// build: boards = g071rb,g0b1re
// build: monitor_speed = 115200

#include <stdint.h>

#include "stm32g0/clock.hpp"
#include "stm32g0/delay.hpp"
#include "stm32g0/flash.hpp"
#include "stm32g0/i2c.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/ticker.hpp"
#include "stm32g0/usart.hpp"
#include "util/print.hpp"

// THE PROTOCOL IS THE AVR CAMPAIGN'S, AND IT IS NOT COPIED (the
// spi_link ruling: pure encoding, three architectures compile the same
// file).
#include "../../../avrdx/src/apps/twi_link.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
constexpr SysClock clock{};

namespace {

using namespace brio;
using twilink::Op;

constexpr UartPins console_pins{
    .tx = {'A', 2, PinFunction::af1},
    .rx = {'A', 3, PinFunction::af1},
};
using Console = Uart<2, console_pins>;
constexpr Console console;

constexpr I2cPins bus_pins{
    .scl = {'B', 8, PinFunction::af6},
    .sda = {'B', 9, PinFunction::af6},
};

/// The one configuration a target that cannot know the coming rung must
/// carry - see the header.
constexpr I2cSpeed client_speed = I2cSpeed::fast_plus_1m;

using Client = I2cClient<1, bus_pins>;
using Host = I2cHost<1, bus_pins>;
using Raw = I2c<1>;

using SclPin = Pin<bus_pins.scl.port, bus_pins.scl.pin>;
using SdaPin = Pin<bus_pins.sda.port, bus_pins.sda.pin>;

constexpr uint16_t firmware_version = 0x0300;   ///< 0x01xx AVR, 0x02xx SAM

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

/// The host action's completion, set by I2C1_IRQHandler while the
/// instance wears its CONTROLLER face.
volatile bool host_live = false;
volatile bool host_done = false;
volatile uint8_t host_status = i2c_ok;

// ---- waits ----------------------------------------------------------------------

void wait_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

/// A microsecond hold of arbitrary length: delay_us is capped below one
/// SysTick period BY DESIGN, and this instrument's commanded stretches
/// (2 ms per byte in the suite) are exactly the blocking waits that cap
/// exists to keep out of KERNEL programs - this is not one, so the hold
/// is spent in chunks the cap admits.
void hold_us(uint32_t us) {
    while (us >= 500u) {
        (void)delay_us(clock, 500u);
        us -= 500u;
    }
    if (us != 0u) {
        (void)delay_us(clock, us);
    }
}

/// The low byte of I2C_ISR - TXE, TXIS, RXNE, ADDR, NACKF, STOPF, TC and
/// TCR - which is what fits the protocol's one-byte status fields and is
/// the half that says what a tenure was doing.
uint8_t status_byte() { return static_cast<uint8_t>(Raw::flags() & 0xFFu); }

// ---- the two standing configurations --------------------------------------------

/// COMMAND MODE: one exact address, nothing else. Everything returns
/// here.
bool go_command() {
    host_live = false;
    const bool ok = Client::init(clock,
                                 {.own = twilink::command_addr,
                                  .mode = I2cAddressMode::seven_bit,
                                  .enable = true},
                                 client_speed);
    decoder.reset();
    resp_pos_ = 0;
    read_done_ = false;
    return ok;
}

/// What an action asked this target to become.
bool apply_client(const twilink::Params& a) {
    host_live = false;
    const uint16_t addr = (a.flags & twilink::flag_deaf)
                              ? twilink::deaf_addr
                              : (a.addr ? a.addr : twilink::command_addr);
    const bool ok = Client::init(clock,
                                 {.own = addr,
                                  .mode = I2cAddressMode::seven_bit,
                                  .enable = true},
                                 client_speed);
    if (a.flags & twilink::flag_general_call) {
        Client::general_call(true);
    }
    return ok;
}

// ---- the command channel --------------------------------------------------------

void prepare(const twilink::Frame& f);
void prepare_nak(Op op, uint8_t sum);

/// One pass of the command-mode target protocol, POLLED, on the
/// I2cClient verbs. THE ORDER IS THE ONE THE CAMPAIGN PAID FOR: RXNE
/// before STOPF (a sweep that clears STOPF first eats the edge the
/// tenure is waiting for), and the closing NACK of a read is what says
/// "the answer has been collected" - ack-before-act's own trigger.
void service_command() {
    if (Client::addressed()) {
        if (Client::host_reads()) {
            resp_pos_ = 0;
            Client::flush();   // 32.4.8: a stale TXDR byte would go out first
        } else {
            decoder.reset();
        }
        Client::answer_address();   // this is also what RELEASES SCL
        return;
    }
    if (Client::data_ready()) {
        const uint8_t v = Client::take();
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
        return;
    }
    if (Client::data_wanted()) {
        Client::give(resp_pos_ < resp_len_ ? resp_[resp_pos_++] : 0x00u);
        return;
    }
    if (Client::host_nacked()) {
        Client::clear_nack();
        Client::flush();
        read_done_ = true;
        return;
    }
    if (Client::stop_seen()) {
        Client::clear_stop();
        return;
    }
    // Every error flag is a LEVEL: clear it, or a pump that merely looks
    // at it comes back to the same one for ever.
    if ((Raw::flags() & I2cFlag::errors) != 0u) {
        Client::clear(I2cClear::errors);
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

/// Be a target: stretch, NACK, General Call, deafness - and, through
/// `coll`, the shared-address fixed-byte flavour. The commanded hold is
/// spent BEFORE the answer, which is exactly where this silicon
/// stretches SCL (the flag holds the clock until software speaks).
///
/// THE REFUSAL RIDES TARGET BYTE CONTROL and its ordering is the one
/// test_stm32_i2c's own pump proved: NBYTES armed to ONE at the address
/// match, the decision taken at TCR with the received count as it stands
/// BEFORE this byte's RXNE increments it, and TCIE never armed - the
/// flag is polled, because an armed transfer-complete interrupt with no
/// owner is a level the ICR cannot clear.
twilink::Report run_serve(const twilink::Params& a, bool fixed_byte) {
    twilink::Report r{};
    if (!apply_client(a)) return r;
    const bool refusing = a.nack_at != 0;
    if (refusing) {
        (void)Client::byte_control(true);
    }
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
            r.last_addr = Client::matched_address();
            if (refusing && !Client::host_reads()) {
                // Byte control: one byte at a time, so each can be refused.
                (void)Raw::reload(1, true, false);
            }
            // THE ADDRESS MATCH IS A STRETCH TOO, and the commanded hold
            // is spent here as well as before every data byte: ADDR holds
            // SCL exactly as RXNE and TXIS do (32.4.8), and the LAST data
            // byte's hold falls AFTER the controller's STOP - a
            // controller's write tenure is over once its last byte is in
            // the shifter, whatever the target does with it afterwards.
            // Measured: without this the wall time of an 8-byte tenure
            // carries seven holds and not eight.
            if (a.hold_us) hold_us(a.hold_us);
            Client::answer_address();
            continue;
        }
        if ((Raw::flags() & I2cFlag::transfer_reload) != 0u) {
            // Byte control's own event: the byte is in, SCL is held
            // between the eighth and the ninth pulse, and CR2.NACK
            // decides. The count is the one BEFORE this byte's RXNE.
            const bool refuse = static_cast<uint16_t>(rx + 1u) >= a.nack_at;
            if (refuse) r.flags |= twilink::report_nacked;
            (void)Client::answer_byte(!refuse);
            continue;
        }
        if (Client::data_ready()) {
            if (a.hold_us) hold_us(a.hold_us);
            const uint8_t v = Client::take();
            if (!first_taken) {
                r.first = v;
                first_taken = true;
            }
            r.sum = static_cast<uint16_t>(r.sum + v);
            ++rx;
            continue;
        }
        if (Client::data_wanted()) {
            if (a.hold_us) hold_us(a.hold_us);
            Client::give(fixed_byte ? a.seed
                                    : twilink::pattern_value(a.pattern, a.seed, tx));
            ++tx;
            continue;
        }
        if (Client::host_nacked()) {
            // The flags sampled AT the closing NACK - a copy taken when
            // the action ends carries whatever the last tenure left
            // behind (the AVR peer's own lesson, held here too).
            r.mstatus = status_byte();
            // AND THE TALLY GIVES ONE BYTE BACK. A target transmitter is
            // asked for the NEXT byte as soon as TXDR empties, so it has
            // always loaded one more than the controller took: the byte
            // standing in TXDR when the closing NACK arrives never
            // reaches the wire, and flush() is what throws it away
            // (32.4.8). Counting it would report nine bytes served for
            // eight clocked.
            if (tx != 0u) --tx;
            Client::clear_nack();
            Client::flush();
            continue;
        }
        if (Client::stop_seen()) {
            if (r.stops < 255) ++r.stops;
            Client::clear_stop();
            continue;
        }
        if ((Raw::flags() & I2cFlag::errors) != 0u) {
            if ((Raw::flags() & I2cFlag::arb_lost) != 0u) {
                r.flags |= twilink::report_arblost;
            }
            if ((Raw::flags() & I2cFlag::bus_error) != 0u) {
                r.flags |= twilink::report_buserr;
            }
            Client::clear(I2cClear::errors);
        }
    }
    if (refusing) {
        (void)Client::byte_control(false);
    }
    r.count = static_cast<uint16_t>(rx + tx);
    r.aux0 = static_cast<uint8_t>(rx);
    r.aux1 = static_cast<uint8_t>(tx);
    r.sstatus = status_byte();
    return r;
}

/// THE HOST ACTION, and on this silicon it is a ROLE SWITCH and not a
/// second half (see the header). The rendezvous is the bus going BUSY -
/// the DUT's own tenure, or a line it holds - and after the lead-in this
/// end writes one tenure of its own. IMPLEMENTED AND NOT EXERCISED BY
/// ANY LETTER of the G0 host suite today: it is the instrument a
/// deterministic two-controller arbitration race would need.
twilink::Report run_arb(const twilink::Params& a) {
    twilink::Report r{};
    Client::release();
    host_live = true;
    if (!Host::init(clock)) {
        host_live = false;
        (void)go_command();
        return r;
    }

    // The rendezvous: the DUT makes the bus busy and this end catches it.
    const uint32_t t0 = Ticker::millis();
    bool armed = false;
    while (Ticker::millis() - t0 < a.ms) {
        if (Raw::busy()) {
            armed = true;
            break;
        }
    }
    if (!armed) {
        r.flags |= twilink::report_timed_out;
        host_live = false;
        Host::release();
        (void)go_command();
        return r;
    }
    if (a.aux16) hold_us(a.aux16);

    static uint8_t burst[32];
    const uint8_t n = a.count < sizeof burst ? static_cast<uint8_t>(a.count)
                                             : static_cast<uint8_t>(sizeof burst);
    for (uint8_t i = 0; i < n; ++i) {
        burst[i] = twilink::pattern_value(a.pattern, a.seed, i);
    }
    host_done = false;
    r.flags |= twilink::report_host_ran;
    const bool sync_done = Host::start({.addr = a.target,
                                        .tx = lend<Lease::reply>(
                                            static_cast<const uint8_t*>(burst)),
                                        .tx_len = n,
                                        .rx = {},
                                        .rx_len = 0,
                                        .reply = {},
                                        .speed = I2cSpeed::standard_100k});
    if (sync_done) {
        host_status = Host::status();
        host_done = true;
    } else {
        while (!host_done && Ticker::millis() - t0 < a.ms) {
        }
    }
    if (!host_done) {
        r.flags |= twilink::report_timed_out;
        (void)Host::recover();
    } else {
        r.mstatus = host_status;
        if (host_status == i2c_ok) r.flags |= twilink::report_host_ok;
        if (host_status == i2c_arb_lost) r.flags |= twilink::report_arblost;
        if (host_status == i2c_bus_error) r.flags |= twilink::report_buserr;
        r.aux1 = n;
    }
    host_live = false;
    Host::release();
    (void)go_command();
    return r;
}

/// A STUCK target, bit-banged: SDA held low from GPIO with the I2C off
/// the pads, released after `aux8` SCL falling edges (0 = only the
/// deadline). Open drain by hand: an OUTPUT over a clear ODR pulls the
/// line down, an INPUT gives it back to the external pull-up.
twilink::Report run_hold_sda(const twilink::Params& a) {
    twilink::Report r{};
    Client::release();
    SclPin::input();
    SdaPin::output(false, {.open_drain = true});
    r.flags |= twilink::report_host_ran;

    const uint32_t t0 = Ticker::millis();
    uint16_t falls = 0;
    bool prev = SclPin::read();
    uint16_t guard = 0;
    for (;;) {
        const bool now = SclPin::read();
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
    SdaPin::input();
    r.count = falls;
    r.aux0 = static_cast<uint8_t>(falls);
    r.aux1 = SdaPin::read() ? 1u : 0u;
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
        const uint32_t w0 = DeviceUid::read().word[0];
        for (uint8_t i = 0; i < 8; ++i) {
            const uint8_t nib = static_cast<uint8_t>((w0 >> (28u - 4u * i)) & 0xFu);
            d.label[i] = static_cast<char>(nib < 10 ? '0' + nib : 'a' + nib - 10);
        }
        d.xtal = 0;               // no HSE on a Nucleo-64, and no HSE root
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
    print(console, "  I2C1 AF6, enabled=", Raw::enabled(), " ISR=", hex(Raw::flags()),
          " own address=", hex(Raw::own_address()), crlf);
    print(console, "  SDA reads ", SdaPin::read() ? "high" : "LOW", ", SCL reads ",
          SclPin::read() ? "high" : "LOW", ", bus busy=", Raw::busy(), crlf);
    print(console, "  commands=", commands, " actions=", actions, " naks=", naks,
          " answers=", answers, crlf);
    print(console, "  last report: op=", hex(last_report.op),
          " count=", last_report.count, " rx/tx=", last_report.aux0, "/",
          last_report.aux1, " hits=", last_report.addr_hits,
          " stops=", last_report.stops, " flags=", hex(last_report.flags), crlf);
    print(console, "               sum=", hex(last_report.sum),
          " first=", hex(last_report.first), " lastaddr=", hex(last_report.last_addr),
          " ISR-lo=", hex(last_report.sstatus),
          " at-NACK=", hex(last_report.mstatus), " ms=", last_report.ms, crlf);
}

}  // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void BRIO_STM32G0_USART2_HANDLER() { (void)Console::isr(); }

/// One vector for the instance: the TARGET is POLLED (nothing armed), so
/// the only interrupts here are the HOST action's engine. A stray level
/// with neither face armed is disarmed and swept rather than spun on.
extern "C" void I2C1_IRQHandler() {
    if (host_live) {
        if (Host::isr()) {
            host_status = Host::status();
            host_done = true;
        }
        return;
    }
    brio::I2c<1>::interrupt(brio::I2cInterrupt::all, false);
    brio::I2c<1>::clear(brio::I2cClear::all);
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool console_ok = Console::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    char label8[9] = {};
    {
        const uint32_t w0 = brio::DeviceUid::read().word[0];
        for (uint8_t i = 0; i < 8; ++i) {
            const uint8_t nib = static_cast<uint8_t>((w0 >> (28u - 4u * i)) & 0xFu);
            label8[i] = static_cast<char>(nib < 10 ? '0' + nib : 'a' + nib - 10);
        }
    }
    const bool client_ok = go_command();

    print(console, crlf, "twi_peer - I2C instrument target (STM32G0 board ", label8,
          ", xtal=none (no HSE fitted; the HSE root is unbuilt), fw ",
          hex(firmware_version), ")", crlf);
    print(console, "target I2C1 AF6: SCL PB8, SDA PB9; command address ",
          hex(twilink::command_addr), " exactly - no general call, no mask; the "
          "kernel clock is PCLK at 64 MHz and the delays are solved for Fm+", crlf);
    print(console, "boot: clk=", clock_ok ? "PLL 64 MHz" : "FAILED",
          " tick=", tick_ok ? "SysTick" : "FAILED",
          " console=", console_ok ? "USART2" : "FAILED",
          " target=", client_ok ? "up" : "FAILED", crlf);
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
                    print(console, "  command-mode target restored", crlf);
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
