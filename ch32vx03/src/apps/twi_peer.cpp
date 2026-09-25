// twi_peer - a scriptable second chip on the bus for the other end of a
// two-board I2C test, on a CH32V203 or a CH32V303: the far board is the
// DUT and the bus controller, and it drives this one IN BAND over the very
// bus both are testing.
//
// The wire format is twi_link.hpp, included by relative path: one file is
// the single source of truth for every board that speaks it, whatever its
// architecture. What this silicon makes of it - the STM32F1's I2C under
// WCH's register names, the event machine of RM ch. 19:
//
//  - ONE I2C IS HOST *OR* TARGET HERE. `I2cHost::init()` and
//    `I2cClient::init()` both begin with the instance's RCC reset, which
//    takes the other face's own address and ACK with it, so the `arb`
//    action SWITCHES the instance to controller for its bounded moment and
//    back. During a host action this board's target is simply absent,
//    which the choreography already tolerates (the DUT waits out the
//    action's deadline either way).
//  - THE TARGET IS POLLED. `I2cClientOptions::interrupts` false leaves the
//    three interrupt enables down and the two vectors dark, so every flag
//    below belongs to the loop, and a commanded hold is a wait the loop
//    spends where it stands.
//  - THE TARGET STRETCHES BY CONSTRUCTION, ONE BYTE LATE. ADDR holds SCL
//    until the STAR1-then-STAR2 read, and a receiver holds it at BTF (RM
//    19.4) - one byte deeper than it sounds: this block takes the NEXT
//    byte into its shift register while the previous one still sits in
//    DATAR. So the hold is spent BEFORE the answer, where the stretch is,
//    and at the ADDRESS MATCH as well as before every data byte, because
//    the last byte's hold falls after the controller's own tenure is over.
//  - A RECEIVED BYTE IS TAKEN BEFORE AN ADDRESS IS ANSWERED. A
//    write-then-read's last written byte may still stand in DATAR when the
//    repeated START's address match rises, and every pass asks for RxNE
//    FIRST so that a report counts what the hardware delivered - the
//    question the suite's late-START letter puts to a CH32 target. A
//    PRECAUTION ON THIS PART AND NOT A CURE: with the address answered
//    first, a CH32V203C8T6 target still took every byte of that letter's
//    twelve probes (measured under a CH32V303VCT6 host).
//  - THE STUCK LINE IS TAKEN ONCE THE WIRE IS AT REST. `hold_sda` starts
//    on the closing NACK of the read that collected its ack, a bit time
//    before the DUT's STOP. Taken the moment the action started, the
//    DUT's next tenure never reached the wire for the whole hold (measured:
//    it was still parked when this end's deadline let the line go, so
//    unstick() found nothing to clock); taken under the STOP, it keeps
//    that STOP off the wire and the DUT's controller stays the bus's
//    master with the STOP pending (measured on the CH32V303VCT6: CTLR1
//    0x201, STAR2 MSL and BUSY), its next tenure waiting behind it. Taken
//    after forty microseconds of quiet wire, it is a START condition the
//    DUT sees like any other, the DUT's next START goes out at its
//    controller's tick (below) into the held line and loses at its first
//    one bit, and unstick() clocks the rest: the four falling edges the
//    suite asks for are that START's and three pulses.
//  - A START ON A BUSY BUS IS NOT HELD UNTIL THE BUS FREES. A CH32 START
//    set while SDA is held low and SCL stands high reaches the wire at the
//    controller's next TICK - about 85 us after the edge that made the bus
//    busy, or a multiple of 80 us after that - without waiting for a STOP
//    (measured on the CH32V303VCT6 alone, and this board's START inside
//    the DUT's window left on the same beat, 325 us after the window
//    opened), and PE dropped and raised by two consecutive stores does
//    not stop it. A START set on a FREE bus - or pending when a STOP
//    frees it - is committed: it reaches SDA some 5 us later, whatever
//    the wire does meanwhile. So `arb` cannot do
//    what the ST peers do, arm its START inside the DUT's window and let
//    the DUT's STOP release it: armed there it goes out into the window,
//    where the DUT's SDA wins every bit this end sends as a one. It
//    watches the pads instead and runs the engine's start() the instant
//    SDA rises under a high SCL, within the 5 us both controllers then
//    spend before either drives SDA: the DUT's START, set just before its
//    STOP, and this one leave together, and the wired-AND decides in both
//    directions (measured: 0x2C won over 0x6B, 0x11 over 0x2C).
//  - A TARGET CANNOT NACK ITS OWN ADDRESS: the address is acknowledged
//    from CTLR1.ACK before ADDR rises, so "deaf" is a re-init at
//    `deaf_addr`. Refusing a DATA byte is CTLR1.ACK too, and this chapter
//    has no per-byte decision point: ACK governs the byte whose ninth pulse
//    has not yet come, so the refusal is a bit set IN ADVANCE, down once
//    the byte before the refused one has been taken, and `report_nacked`
//    says the acknowledge was held down for that byte (the DUT's
//    `i2c_nack_data` is the other half of the measurement). ACK goes back
//    up at every Stop - but 19.12.6 raises STOPF for a Stop "after the
//    response", so after a refused byte it is the action's deadline, and
//    the command-mode re-init behind it, that puts ACK back.
//  - A TARGET TRANSMITTER IS ASKED FOR ONE BYTE MORE THAN THE CONTROLLER
//    TAKES: DATAR empties into the shifter and TxE rises again, so the byte
//    standing in DATAR when the closing NACK arrives never reaches the
//    wire. `I2cClient::flush()` throws it away - the PE cycle that spares
//    the addresses and the timing - and the tally gives it back. The give
//    pump is GATED ON A READ TENURE, so nothing is written into DATAR while
//    no controller is reading.
//  - flag_stop_interrupt maps to nothing: STOPF is a polled flag here and
//    the Stop count is kept whenever the action loop sees it.
//  - THE TARGET'S TIMING IS FREQ AND NOT A RATE. CKCFGR is the
//    controller's divider; what a target needs is CTLR2.FREQ carrying PB1
//    in whole megahertz, because the block generates its timing from it
//    (19.12.2). This end sits at 48 MHz of FREQ, and the DUT may walk
//    whatever rungs it likes against it.
//  - THIS BOARD DOES NOT HOLD THE BUS UP. The pulls of this family are the
//    INPUT driver's and an output mode disables them (RM 10.2.7,
//    ch32vx03/pin.md), the open-drain alternate function included, so
//    while the I2C owns PB6 and PB7 nothing here pulls them: the wire
//    wants resistors of its own - the two 4.7 kOhm
//    pull-ups the CH32V303 evaluation board carries on its PB6-PB10 and
//    PB7-PB11 wires are one pair that serves. The two actions that take the
//    pads away from the peripheral, `hold_sda` and `quiet`, leave them
//    floating.
//
// COEXISTENCE is the protocol header's own argument: the command channel
// is ONE exact target address (0x6B) - no second address, no general call
// - so nothing the DUT's single-board letters do can wake this board.
//
// Bus: I2C1 on its default column, open drain -
//   PB6  SCL
//   PB7  SDA
// and a common GND. PB1 runs at 48 MHz (HCLK 96 MHz from the HSI's PLL),
// inside 19.12.2's 4..60 MHz window.
//
// Console: USART1 PA9/PA10 at 115200 - the probe's serial, the suites' own
// port - observability only.
//   ? help | i status and counters | 0 back to command mode | 3 trace
//
// The ident label is the part number without its CH32 prefix ("V203C8",
// "V303VC"): with two parts of this stratum on one bus, WHICH PART is the
// target is the thing measured, and the die's identifier is on this
// console's banner instead. ident.xtal is ALWAYS 0: the clock is the
// HSI's PLL and no crystal is started.
//
// build: boards = v203c6,v203c8,v303vc
// build: monitor_speed = 115200

#include <stdint.h>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/delay.hpp"
#include "ch32vx03/i2c.hpp"
#include "ch32vx03/nvm.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/ticker.hpp"
#include "ch32vx03/usart.hpp"
#include "kernel/borrowed.hpp"
#include "util/print.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED: pure encoding, not one
// register, and every architecture on this link compiles the same file.
#include "../../../avrdx/src/apps/twi_link.hpp"

namespace {

using namespace brio;
using twilink::Op;

using P = Ch32vx03Platform<>;
using SysClock = Clock<ClockSource::pll, 96'000'000>;
constexpr SysClock clock;

using Console = Uart<1, P, 64, 128>;
constexpr Console console;

using Client = I2cClient<1>;
using Host = I2cHost<1>;
using Raw = I2c<1>;

using SclPin = Pin<i2c_default_pins<1>.scl.port, i2c_default_pins<1>.scl.pin>;
using SdaPin = Pin<i2c_default_pins<1>.sda.port, i2c_default_pins<1>.sda.pin>;

constexpr uint16_t firmware_version = 0x0500;   ///< see twi_link.hpp's Ident

// The ident label drops the first four characters of the part number,
// which is the prefix every part of this stratum's table carries.
static_assert(device::part_name[0] == 'C' && device::part_name[1] == 'H' &&
                  device::part_name[2] == '3' && device::part_name[3] == '2',
              "twi_peer: the ident label is the part number after its CH32 prefix");

bool trace = false;

twilink::Decoder decoder;
twilink::Report last_report;

uint32_t commands = 0, actions = 0, naks = 0, answers = 0, flushes = 0;

uint8_t resp_[twilink::response_bytes];
uint8_t resp_len_ = 0;
uint8_t resp_pos_ = 0;
/// IS A READ TENURE UNDER WAY? The give pump is gated on it (the header):
/// only inside one is a byte ever written into DATAR.
bool serving_ = false;
uint8_t own_addr_ = twilink::command_addr;
bool read_done_ = false;
Op pending_ = Op::ping;
bool has_pending_ = false;
twilink::Params pending_params_{};

/// The host action's completion, set by the two I2C1 vectors while the
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

/// A microsecond hold of any length: delay_us() is capped below one tick
/// period BY DESIGN, and this instrument's commanded stretches
/// (milliseconds per byte) are exactly the blocking waits that cap keeps
/// out of KERNEL programs - this is not one, so the hold is spent in
/// chunks the cap admits.
void hold_us(uint32_t us) {
    while (us >= 500u) {
        (void)delay_us(clock, 500u);
        us -= 500u;
    }
    if (us != 0u) {
        (void)delay_us(clock, us);
    }
}

/// The low byte of STAR1 - SB, ADDR, BTF, ADD10, STOPF, RxNE and TxE -
/// which is what fits the protocol's one-byte status fields and is the
/// half that says what a tenure was doing.
uint8_t status_byte() { return static_cast<uint8_t>(Raw::status1() & 0xFFu); }

/// The DUT's window, on the pads: `stop` false waits for SDA LOW under a
/// high SCL (the START condition it makes by hand), `stop` true for SDA
/// HIGH under a high SCL (the STOP that closes it). The deadline is read
/// once every 4096 polls, so a polled edge is seen within a fraction of a
/// microsecond.
bool wait_wire(uint32_t t0, uint32_t ms, bool stop) {
    uint16_t guard = 0;
    for (;;) {
        if (SdaPin::read() == stop && SclPin::read()) {
            return true;
        }
        ++guard;
        if ((guard & 0x0FFFu) == 0u && Ticker::millis() - t0 >= ms) {
            return false;
        }
    }
}

/// THE WIRE AT REST: both lines high without a break for `quiet_us`
/// microseconds, within `ms` of `t0`. The quiet span is longer than any
/// SCL high half a controller on this bus makes (five microseconds at
/// 100 kHz), so a tenure in flight never passes for an idle bus.
bool wait_wire_quiet(uint32_t t0, uint32_t ms, uint16_t quiet_us) {
    uint16_t quiet = 0;
    while (Ticker::millis() - t0 < ms) {
        if (SclPin::read() && SdaPin::read()) {
            (void)delay_us(clock, 1);
            ++quiet;
            if (quiet >= quiet_us) {
                return true;
            }
        } else {
            quiet = 0;
        }
    }
    return false;
}

// ---- the two standing configurations ----------------------------------------------

/// Bring the target up at one exact address, POLLED (the header).
bool bring_up_client(uint8_t addr, bool general_call) {
    host_live = false;
    own_addr_ = addr;
    const bool ok = Client::init(clock, {.own = addr, .general_call = general_call},
                                 {.no_stretch = false, .interrupts = false});
    serving_ = false;
    return ok;
}

/// COMMAND MODE: one exact address, nothing else. Everything returns here.
bool go_command() {
    const bool ok = bring_up_client(twilink::command_addr, false);
    decoder.reset();
    resp_pos_ = 0;
    read_done_ = false;
    return ok;
}

/// What an action asked this target to become.
bool apply_client(const twilink::Params& a) {
    const uint8_t addr = (a.flags & twilink::flag_deaf) != 0u
                             ? twilink::deaf_addr
                             : (a.addr != 0u ? a.addr : twilink::command_addr);
    return bring_up_client(addr, (a.flags & twilink::flag_general_call) != 0u);
}

/// This block does not report the matched address as a byte: STAR2 says
/// WHICH own address it was, and the value is the one this end configured.
/// CALLED ONLY RIGHT AFTER answer_address(): the bare STAR2 read underneath
/// clears an ADDR that rose since the last read of STAR1 (19.12.6), and
/// there the STAR1 read before it is answer_address()'s own.
uint8_t matched_address() { return Client::general_call_matched() ? 0x00u : own_addr_; }

/// The closing NACK of a read tenure: the flag cleared, the give pump
/// stopped, and the byte the shifter asked for beyond it dropped (the
/// header). True when there was one to drop, which the console counts as
/// `dropped=`.
bool close_read() {
    Client::clear_nack();
    serving_ = false;
    const bool dropped = Client::flush();
    if (dropped) {
        ++flushes;
    }
    return dropped;
}

// ---- the command channel --------------------------------------------------------

void prepare(const twilink::Frame& f);
void prepare_nak(Op op, uint8_t sum);

/// One pass of the command-mode target protocol, POLLED, on the
/// I2cClient verbs. THE ORDER IS LOAD-BEARING: a received byte first (the
/// header), then the address match (the only place the DIRECTION can be
/// read, because the sequence that reports it is the sequence that clears
/// ADDR), then the closing NACK before the give pump - it is what says
/// "the answer has been collected", ack-before-act's own trigger.
void service_command() {
    if (Client::data_ready()) {
        const uint8_t v = Client::take();
        switch (decoder.feed(v)) {
            case twilink::Decoder::Result::frame:
                if (twilink::is_command(decoder.frame().op)) {
                    prepare(decoder.frame());
                } else {
                    prepare_nak(decoder.frame().op, decoder.frame().sum);
                }
                break;
            case twilink::Decoder::Result::bad_checksum:
                prepare_nak(decoder.pending_op(), decoder.frame().sum);
                break;
            default: break;
        }
        return;
    }
    if (Client::addressed()) {
        // EVT1: the STAR1-then-STAR2 read answers the match, RELEASES SCL
        // and returns the direction in one act.
        if (Client::answer_address()) {
            resp_pos_ = 0;   // a read: the prepared answer, from its first byte
            serving_ = true;
        } else {
            decoder.reset();   // a write: one tenure is exactly one frame
            serving_ = false;
        }
        return;
    }
    if (Client::host_nacked()) {
        (void)close_read();
        read_done_ = true;
        return;
    }
    if (serving_ && Client::data_wanted()) {
        uint8_t v = 0x00;
        if (resp_pos_ < resp_len_) {
            v = resp_[resp_pos_];
            ++resp_pos_;
        }
        Client::give(v);
        return;
    }
    if (Client::stop_seen()) {
        serving_ = false;
        Client::clear_stop();
        return;
    }
    // Every error flag is a LEVEL: clear it, or a pump that merely looks at
    // it comes back to the same one for ever. THE CLOSING NACK IS ONE OF
    // THEM, and it may rise between the check above and this one: swallowed
    // here it would cost a whole command, so it is answered wherever it is
    // found and only the rest is merely cleared.
    const uint16_t errs = static_cast<uint16_t>(Raw::status1() & i2c_errors);
    if (errs != 0u) {
        if ((errs & i2c_af) != 0u) {
            (void)close_read();
            read_done_ = true;
        }
        Raw::clear_errors(errs);
    }
}

void set_response(Op op, const uint8_t* p, uint8_t len) {
    uint8_t n = 0;
    twilink::write_frame(
        [&](uint8_t b) {
            if (n < sizeof resp_) {
                resp_[n++] = b;
            }
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

/// ACK BEFORE ACT, the protocol's own choreography: the action starts only
/// once the DUT has collected the acknowledgement.
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
    if (twilink::is_action(f.op)) {
        pending_params_ = twilink::get_params(f.data);
    }
}

// ---- the actions ----------------------------------------------------------------

/// The refusal, in this block's own terms: CTLR1.ACK governs the byte whose
/// ninth pulse has not yet come, so it goes down once the byte BEFORE the
/// refused one has been taken and stays down from there (the other peers
/// refuse from `nack_at` onwards too).
void arm_ack(twilink::Report& r, const twilink::Params& a, uint16_t received) {
    if (a.nack_at == 0u) {
        return;
    }
    const bool ack = static_cast<uint16_t>(received + 1u) < a.nack_at;
    if (!ack) {
        r.flags |= twilink::report_nacked;
    }
    Client::acknowledge(ack);
}

/// Be a target: stretch, NACK, General Call, deafness - and, through
/// `coll`, the shared-address fixed-byte flavour. The commanded hold is
/// spent BEFORE the answer, which is where this silicon stretches SCL, one
/// byte deep (the header).
twilink::Report run_serve(const twilink::Params& a, bool fixed_byte) {
    twilink::Report r{};
    if (!apply_client(a)) {
        return r;
    }
    uint16_t rx = 0, tx = 0;
    bool first_taken = false;
    // THE NACK THAT CLOSES A READ, wherever it is found: the flags sampled
    // AT it - a copy taken when the action ends carries whatever the last
    // tenure left behind - and the tally gives back the byte dropped.
    const auto closing_nack = [&] {
        r.mstatus = status_byte();
        if (close_read() && tx != 0u) {
            --tx;
        }
    };
    const uint32_t t0 = Ticker::millis();
    for (;;) {
        if (Ticker::millis() - t0 >= a.ms) {
            r.flags |= twilink::report_timed_out;
            break;
        }
        if (a.count != 0u && static_cast<uint16_t>(rx + tx) >= a.count && r.stops != 0u) {
            break;
        }
        if (Client::data_ready()) {
            if (a.hold_us != 0u) {
                hold_us(a.hold_us);
            }
            const uint8_t v = Client::take();
            if (!first_taken) {
                r.first = v;
                first_taken = true;
            }
            r.sum = static_cast<uint16_t>(r.sum + v);
            ++rx;
            arm_ack(r, a, rx);
            continue;
        }
        if (Client::addressed()) {
            ++r.addr_hits;
            // THE ADDRESS MATCH IS A STRETCH TOO - ADDR holds SCL exactly as
            // BTF does - and the commanded hold is spent here as well as
            // before every data byte, because the LAST byte's hold falls
            // after the controller's own tenure is over.
            if (a.hold_us != 0u) {
                hold_us(a.hold_us);
            }
            const bool reads = Client::answer_address();
            serving_ = reads;
            r.last_addr = matched_address();
            // The refusal is a RECEIVER's: arming it on a read tenure would
            // refuse the next address and report a NACK that never had a
            // byte to land on.
            if (!reads) {
                arm_ack(r, a, rx);
            }
            continue;
        }
        if (Client::host_nacked()) {
            closing_nack();
            continue;
        }
        if (serving_ && Client::data_wanted()) {
            if (a.hold_us != 0u) {
                hold_us(a.hold_us);
            }
            Client::give(fixed_byte ? a.seed : twilink::pattern_value(a.pattern, a.seed, tx));
            ++tx;
            continue;
        }
        if (Client::stop_seen()) {
            if (r.stops < 255u) {
                ++r.stops;
            }
            serving_ = false;
            Client::clear_stop();
            // ACK BACK UP, or an address would be refused as well and the
            // instrument would go deaf for the rest of the action.
            Client::acknowledge(true);
            continue;
        }
        const uint16_t errs = static_cast<uint16_t>(Raw::status1() & i2c_errors);
        if (errs != 0u) {
            if ((errs & i2c_af) != 0u) {
                closing_nack();
            }
            if ((errs & i2c_arlo) != 0u) {
                r.flags |= twilink::report_arblost;
            }
            if ((errs & i2c_berr) != 0u) {
                r.flags |= twilink::report_buserr;
            }
            Raw::clear_errors(errs);
        }
    }
    Client::acknowledge(true);
    r.count = static_cast<uint16_t>(rx + tx);
    r.aux0 = static_cast<uint8_t>(rx);
    r.aux1 = static_cast<uint8_t>(tx);
    r.sstatus = status_byte();
    return r;
}

/// THE HOST ACTION, and on this silicon it is a ROLE SWITCH and not a
/// second half (the header). The rendezvous is the DUT's window - SDA
/// pulled low by hand under a high SCL, a START condition every controller
/// on the wire sees - and the STOP that closes it. This end's START is set
/// on that STOP and never inside the window, because a CH32 START on a busy
/// bus whose clock stands still goes out at the controller's tick, into the
/// window (the header).
twilink::Report run_arb(const twilink::Params& a) {
    twilink::Report r{};
    host_live = true;
    if (!Host::init(clock)) {
        host_live = false;
        (void)go_command();
        return r;
    }

    // The bytes first, so nothing but the start() itself stands between
    // the STOP and the START.
    static uint8_t burst[32];
    const uint8_t n = a.count < sizeof burst ? static_cast<uint8_t>(a.count)
                                             : static_cast<uint8_t>(sizeof burst);
    for (uint8_t i = 0; i < n; ++i) {
        burst[i] = twilink::pattern_value(a.pattern, a.seed, i);
    }

    // THE WIRE AT REST FIRST. This action starts on the closing NACK of the
    // read that collected its ack, a bit time before the DUT's STOP: a
    // window watched for from there would open on that tail. And a BUSY
    // left standing over the idle wire by the role switch is 19.12.1's own
    // case, taken out by the chapter's SWRST before the start() that reads
    // it.
    const uint32_t t0 = Ticker::millis();
    bool window = false;
    bool stop_seen = false;
    if (wait_wire_quiet(t0, a.ms, 40u)) {
        if (Raw::busy()) {
            (void)Host::recover();
        }
        window = wait_wire(t0, a.ms, false);
    }
    if (window) {
        if (a.aux16 != 0u) {
            hold_us(a.aux16);
        }
        stop_seen = wait_wire(t0, a.ms, true);
    }
    if (!stop_seen) {
        r.flags |= twilink::report_timed_out;
        host_live = false;
        (void)go_command();
        return r;
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
        if (host_status == i2c_ok) {
            r.flags |= twilink::report_host_ok;
        }
        if (host_status == i2c_arb_lost) {
            r.flags |= twilink::report_arblost;
        }
        if (host_status == i2c_bus_error) {
            r.flags |= twilink::report_buserr;
        }
        r.aux1 = n;
    }
    host_live = false;
    (void)go_command();
    return r;
}

/// A STUCK target, by hand: SDA held low from the port with the I2C off
/// the pads, released after `aux8` SCL falling edges (0 = only the
/// deadline does). Open drain by hand: an open-drain OUTPUT over a clear
/// output bit pulls the line down, a floating INPUT gives it back to the
/// wire's resistors.
twilink::Report run_hold_sda(const twilink::Params& a) {
    twilink::Report r{};
    Client::release();
    SclPin::input();
    // SDA IS TAKEN ONCE THE WIRE IS AT REST: after the DUT's STOP, and
    // not inside the ninth clock or under the STOP (the header).
    const uint32_t t0 = Ticker::millis();
    (void)wait_wire_quiet(t0, a.ms, 40u);
    SdaPin::output(false, PinDrive::open_drain);
    r.flags |= twilink::report_host_ran;

    uint16_t falls = 0;
    bool prev = SclPin::read();
    uint16_t guard = 0;
    for (;;) {
        const bool now = SclPin::read();
        if (prev && !now) {
            ++falls;
            if (a.aux8 != 0u && falls >= a.aux8) {
                break;
            }
        }
        prev = now;
        ++guard;
        if (guard == 0u && Ticker::millis() - t0 >= a.ms) {
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

/// This board off the wire for the deadline: the control case. The
/// peripheral is released and the two pads float.
twilink::Report run_quiet(const twilink::Params& a) {
    twilink::Report r{};
    Client::release();
    wait_ms(a.ms);
    r.count = 1;
    return r;
}

/// The part number after its CH32 prefix, NUL padded to the label's eight
/// characters (the header).
void ident_label(char out[8]) {
    const char* name = device::part_name + 4;
    uint8_t i = 0;
    for (; i < 8u && name[i] != '\0'; ++i) {
        out[i] = name[i];
    }
    for (; i < 8u; ++i) {
        out[i] = '\0';
    }
}

void run_pending() {
    const Op op = pending_;
    has_pending_ = false;

    if (op == Op::ping) {
        return;
    }
    if (op == Op::ident) {
        twilink::Ident d{};
        ident_label(d.label);
        d.xtal = 0;   // the HSI's PLL: no crystal is started
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
    r.ms = took > 255u ? uint8_t{255} : static_cast<uint8_t>(took);
    r.op = twilink::byte_of(op);
    last_report = r;
    (void)go_command();
}

// ---- the console ----------------------------------------------------------------

void help() {
    print(console, "twi_peer: ? help | i status | 0 back to command mode | 3 trace", crlf);
}

/// AN IDLE-BUS COMMAND: `busy()` is a bare STAR2 read, which consumes an
/// address match that rises between the loop's last look at STAR1 and it
/// (19.12.6). Typed between tenures, as a human does, it costs nothing.
void status() {
    print(console, "  I2C1 PB6/PB7, enabled=", Raw::enabled(), " STAR1=", hex(Raw::status1()),
          " own address=", hex(own_addr_), crlf);
    print(console, "  SDA reads ", SdaPin::read() ? "high" : "LOW", ", SCL reads ",
          SclPin::read() ? "high" : "LOW", ", bus busy=", Raw::busy(), crlf);
    print(console, "  commands=", commands, " actions=", actions, " naks=", naks,
          " answers=", answers, " dropped=", flushes, crlf);
    print(console, "  last report: op=", hex(last_report.op), " count=", last_report.count,
          " rx/tx=", last_report.aux0, "/", last_report.aux1, " hits=", last_report.addr_hits,
          " stops=", last_report.stops, " flags=", hex(last_report.flags), crlf);
    print(console, "               sum=", hex(last_report.sum), " first=", hex(last_report.first),
          " lastaddr=", hex(last_report.last_addr), " STAR1-lo=", hex(last_report.sstatus),
          " at-NACK=", hex(last_report.mstatus), " ms=", last_report.ms, crlf);
}

}  // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Console::isr(); }

/// The instance's TWO vectors. The TARGET is POLLED (its three interrupt
/// enables are down and its lines dark), so the only owner here is the
/// HOST action's engine. Anything else that reaches a vector is DISARMED
/// AND NOTHING IS CLEARED: every flag this instrument lives on belongs to
/// the loop, and a handler that consumed one would take a whole tenure
/// with it.
extern "C" BRIO_CH32_INTERRUPT void i2c1_ev_handler() {
    if (host_live) {
        if (Host::isr()) {
            host_status = Host::status();
            host_done = true;
        }
        return;
    }
    Raw::event_interrupt(false);
    Raw::buffer_interrupt(false);
}

extern "C" BRIO_CH32_INTERRUPT void i2c1_er_handler() {
    if (host_live) {
        if (Host::error_isr()) {
            host_status = Host::status();
            host_done = true;
        }
        return;
    }
    Raw::error_interrupt(false);
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool console_ok = Console::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    char label[9] = {};
    ident_label(label);
    const brio::DeviceUid uid = brio::DeviceUid::read();
    const bool client_ok = go_command();

    print(console, crlf, "twi_peer - I2C instrument target (", brio::device::part_name,
          ", die ", hex(uid.word[0]), " ", hex(uid.word[1]), ", label '", label, "', fw ",
          hex(firmware_version), ")", crlf);
    print(console, "target I2C1: SCL PB6, SDA PB7, open drain with NO pull of this board's own "
                   "- the wire wants resistors; command address ",
          hex(twilink::command_addr),
          " exactly - no general call, no second address; FREQ carries PB1 at ",
          SysClock::pclk1_hz / 1'000'000u, " MHz", crlf);
    print(console, "boot: clk=", clock_ok ? "PLL 96 MHz" : "FAILED",
          " tick=", tick_ok ? "STK" : "FAILED", " console=", console_ok ? "USART1" : "FAILED",
          " target=", client_ok ? "up" : "FAILED", crlf);
    help();
    print(console, "> ");

    for (;;) {
        uint8_t c = 0;
        if (Console::read_byte(c)) {
            if (c != '\r' && c != '\n') {
                print(console, static_cast<char>(c), crlf);
                if (c == '?') {
                    help();
                } else if (c == 'i') {
                    status();
                } else if (c == '0') {
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
            if (has_pending_) {
                run_pending();
            }
        }
    }
}
