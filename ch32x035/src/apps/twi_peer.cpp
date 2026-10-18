// twi_peer - a scriptable second chip on the bus for the other end of a
// two-board I2C test, on a CH32X035: the far board is the DUT and the bus
// controller, and it drives this one IN BAND over the very bus both are
// testing.
//
// The wire format is twi_link.hpp, included by relative path: one file is
// the single source of truth for every board that speaks it, whatever its
// architecture. What this silicon makes of it - the STM32F1's I2C under
// WCH's register names, the event machine of RM ch. 15:
//
//  - THE BUS IS I2C1 ON ITS COLUMN 010 (8.3.2.1): SCL PC16, SDA PC17 - the
//    USB pads, the one column of the CH32X035F8U6 off the debug port, and
//    the desk's I2C wires to the CH32V006's column 3. Open drain, which on
//    this series only the I2C itself makes (8.2.8).
//  - THE PULL-UPS HANG FROM PB3, AND THIS IMAGE HOLDS THEM UP. A 1.5 kOhm
//    resistor from each line to PB3, never to VCC: PC17 is this board's
//    boot pin, and a high level on it at power-up enters WCH's loader. PB3
//    floats from every reset; this program drives it high, push-pull, from
//    the supply check at boot to the end of its run - command mode, every
//    action, `quiet` and `hold_sda` included - so the bus is pulled up
//    exactly while an image that uses it runs.
//  - ONE I2C IS HOST *OR* TARGET HERE. `I2cHost::init()` and
//    `I2cClient::init()` both begin with the instance's RCC reset, which
//    takes the other face's own address and ACK with it, so the `arb` and
//    `host_read` actions SWITCH the instance to controller for their
//    bounded moment and back. During a host action this board's target is
//    simply absent, which the choreography already tolerates (the DUT
//    waits out the action's deadline either way).
//  - THE HOST KEEPS THE DEFAULT PLACEMENT, its event pump in the flash
//    (util/placement.hpp): its tenures are the `arb` and `host_read`
//    actions' few bytes, the wire stretched while an entry runs, and SRAM
//    would buy core share this peer has no use for.
//  - THE TARGET IS POLLED. `I2cClientOptions::interrupts` false leaves the
//    three interrupt enables and the two PFIC lines down, so every flag
//    below belongs to the loop, and a commanded hold is a wait the loop
//    spends where it stands. The two vectors stay bound and DISARM
//    whatever reaches them.
//  - THE TARGET STRETCHES BY CONSTRUCTION, ONE BYTE LATE. ADDR holds SCL
//    until the STAR1-then-STAR2 read, and a receiver holds it at BTF
//    (15.4) - one byte deeper than it sounds: this block takes the NEXT
//    byte into its shift register while the previous one still sits in
//    DATAR (docs/ch32x035/i2c.md). So the hold is spent BEFORE the answer,
//    where the stretch is, and at the ADDRESS MATCH as well as before every
//    data byte, because the last byte's hold falls after the controller's
//    own tenure is over.
//  - A RECEIVED BYTE IS TAKEN BEFORE AN ADDRESS IS ANSWERED. A
//    write-then-read's last written byte may still stand in DATAR when the
//    repeated START's address match rises, and every pass asks for RxNE
//    FIRST so that a report counts what the hardware delivered - the
//    question the DUT's repeated-START letter puts to a CH32 target.
//  - THE STUCK LINE IS TAKEN ONCE THE WIRE IS AT REST. `hold_sda` starts
//    on the closing NACK of the read that collected its ack, a bit time
//    before the DUT's STOP: SDA is taken after forty microseconds of
//    quiet wire, a START condition the DUT sees like any other, and the
//    DUT's next tenure meets the held line. The line is pulled down by a
//    push-pull output (no open-drain GPIO on this series), the one level
//    an I2C node may drive.
//  - A START IS NOT ARMED INSIDE THE DUT'S WINDOW. `arb` and `host_read`
//    (the extended repertoire: this host READING the DUT's client) watch
//    the pads instead and run the engine's start() the instant SDA rises
//    under a high SCL - the STOP that closes the DUT's window - so the
//    DUT's START, set just before its STOP, and this one leave together
//    and the wired-AND decides. This die lets a START out into a held line
//    (docs/ch32x035/i2c.md), so armed inside the window it would lose at
//    its first one bit; on the STOP it races fairly.
//  - A TARGET CANNOT NACK ITS OWN ADDRESS: the address is acknowledged
//    from CTLR1.ACK before ADDR rises, so "deaf" is a re-init at
//    `deaf_addr`. Refusing a DATA byte is CTLR1.ACK too, and this chapter
//    has no per-byte decision point: ACK governs the byte whose ninth pulse
//    has not yet come, so the refusal is a bit set IN ADVANCE, down once
//    the byte before the refused one has been taken, and `report_nacked`
//    says the acknowledge was held down for that byte. ACK goes back up
//    at every Stop the loop sees, and the command-mode re-init behind the
//    action's deadline puts it back in any case.
//  - A TARGET TRANSMITTER IS ASKED FOR ONE BYTE MORE THAN THE CONTROLLER
//    TAKES: DATAR empties into the shifter and TxE rises again, so the byte
//    standing in DATAR when the closing NACK arrives never reaches the
//    wire. `I2cClient::flush()` throws it away - the PE cycle that spares
//    the addresses and the timing - and the tally gives it back; the
//    console counts the drops as `dropped=`, and the `counters` query
//    hands them to the DUT. The give pump is GATED ON A READ TENURE, so
//    nothing is written into DATAR while no controller is reading.
//  - flag_stop_interrupt maps to nothing: STOPF is a polled flag here and
//    the Stop count is kept whenever the action loop sees it. This block
//    raises STOPF on a Stop it was not addressed in as well, whatever ACK
//    says (docs/ch32x035/i2c.md); the loop clears it like any other.
//  - THE TARGET'S TIMING IS FREQ AND NOT A RATE. CKCFGR is the
//    controller's divider; what a target needs is CTLR2.FREQ carrying HCLK
//    in whole megahertz (15.9.2). This end sits at 48 MHz of FREQ, and the
//    DUT may walk whatever rungs it likes against it.
//
// COEXISTENCE is the protocol header's own argument: the command channel
// is ONE exact target address (0x6B) - no second address, no general call
// - so nothing the DUT's single-board letters do can wake this board.
//
// THE SUPPLY at boot (the desk's rule for a board that drives a link
// pad): the voltage detector at 3.0 V (pwr.hpp); below it the peer leaves
// PB3 and the bus pads floating and says so on its console.
//
// Console: USART2 on PA2/PA3 at 115200 - the probe's serial -
// observability only.
//   ? help | i status and counters | 0 back to command mode | 3 trace
//
// The ident label is the part number after its CH32 prefix ("X035F8U6");
// ident.xtal is ALWAYS 0: this series has no crystal oscillator, and the
// clock is the HSI's 48 MHz.
//
// build: boards = x035f8
// build: monitor_speed = 115200

#include <stdint.h>

#include "ch32x035/clock.hpp"
#include "ch32x035/delay.hpp"
#include "ch32x035/i2c.hpp"
#include "ch32x035/pfic.hpp"
#include "ch32x035/pin.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/pwr.hpp"
#include "ch32x035/ticker.hpp"
#include "ch32x035/usart.hpp"
#include "kernel/borrowed.hpp"
#include "util/print.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED: pure encoding, not one
// register, and every architecture on this link compiles the same file.
#include "../../../avrxt/src/apps/twi_link.hpp"

namespace {

using namespace brio;
using twilink::Op;

using P = Ch32x035Platform<>;
/// The HSI undivided, this series' only root (the header).
using SysClock = Clock<ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

using Console = Uart<2, P>;
constexpr Console console;

/// I2C1's column 010 (the header): the part's first column off the debug
/// port, the driver's default.
constexpr I2cPins bus_pins = i2c1_default_pins;
static_assert(bus_pins.remap == 2u && bus_pins.scl == Pad{'C', 16} && bus_pins.sda == Pad{'C', 17},
              "8.3.2.1: I2C1_RM 010 is SCL PC16, SDA PC17 - the desk's I2C wires");
using Client = I2cClient<1, bus_pins>;
using Host = I2cHost<1, bus_pins>;
using Raw = I2c<1>;
using SclPin = Pin<'C', 16>;
using SdaPin = Pin<'C', 17>;
/// THE BUS'S PULL-UP RAIL (the header).
using Rail = Pin<'B', 3>;

constexpr uint16_t firmware_version =
    static_cast<uint16_t>((twilink::peer_ch32x035 << 8) | 0x01u);   ///< twi_link.hpp's Ident

static_assert(device::part_name[0] == 'C' && device::part_name[1] == 'H' &&
                  device::part_name[2] == '3' && device::part_name[3] == '2',
              "twi_peer: the ident label is the part number after its CH32 prefix");

bool supply_ok = false;
bool trace = false;

twilink::Decoder decoder;
twilink::Report last_report;

uint32_t commands = 0, actions = 0, naks = 0, answers = 0, flushes = 0;
/// The drops of the last action (the `counters` query).
uint16_t last_drops = 0;

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
/// What reached a vector while the target was polled - counted, disarmed.
volatile uint32_t stray_vectors = 0;

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
/// which is what fits the protocol's one-byte status fields.
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

/// THE WIRE AT REST, WATCHED THROUGHOUT: both lines high at every look for
/// `us` microseconds; false at the first look that finds either low. Four
/// looks (two INDR loads each) between two readings of the cycle counter,
/// so no two looks are more than some forty HCLK cycles apart - under a
/// microsecond at 48 MHz, shorter than the shortest SCL low a controller
/// on this bus makes (1.3 us in fast mode), so a tenure in flight is seen
/// at its first clock.
bool wire_rests(uint32_t us) {
    const uint32_t span = SysClock::hz / 1'000'000u * us;
    const uint32_t t0 = Ticker::cycles();
    do {
        for (uint8_t k = 0; k < 4u; ++k) {
            if (!SclPin::read() || !SdaPin::read()) {
                return false;
            }
        }
    } while (Ticker::cycles() - t0 < span);
    return true;
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
/// CALLED ONLY RIGHT AFTER answer_address() (a bare STAR2 read clears an
/// ADDR that rose since the last read of STAR1, 15.10.6).
uint8_t matched_address() { return Client::general_call_matched() ? 0x00u : own_addr_; }

/// The closing NACK of a read tenure: the flag cleared, the give pump
/// stopped, and the byte the shifter asked for beyond it dropped (the
/// header). True when there was one to drop.
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

/// One pass of the command-mode target protocol, POLLED. THE ORDER IS
/// LOAD-BEARING: a received byte first (the header), then the address
/// match (the only place the DIRECTION can be read, because the sequence
/// that reports it is the sequence that clears ADDR), then the closing NACK
/// before the give pump - it is what says "the answer has been collected",
/// ack-before-act's own trigger.
void service_command() {
    if (Client::data_ready()) {
        const uint8_t v = Client::take();
        switch (decoder.feed(v)) {
            case twilink::Decoder::Result::frame:
                if (twilink::is_extended_command(decoder.frame().op)) {
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
    // THEM, and it may rise between the check above and this one: it is
    // answered wherever it is found, and only the rest is merely cleared.
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
    if (twilink::is_extended_action(f.op) && f.len < twilink::params_size) {
        prepare_nak(f.op, f.sum);
        return;
    }
    const uint8_t p[2] = {twilink::byte_of(f.op), f.sum};
    set_response(Op::ack, p, 2);
    pending_ = f.op;
    has_pending_ = true;
    if (twilink::is_extended_action(f.op)) {
        pending_params_ = twilink::get_params(f.data);
    }
}

// ---- the actions ----------------------------------------------------------------

/// The refusal, in this block's own terms: CTLR1.ACK governs the byte whose
/// ninth pulse has not yet come, so it goes down once the byte BEFORE the
/// refused one has been taken and stays down from there.
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
    // AT it, and the tally gives back the byte dropped.
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
            // BTF does - and the commanded hold is spent here as well.
            if (a.hold_us != 0u) {
                hold_us(a.hold_us);
            }
            const bool reads = Client::answer_address();
            serving_ = reads;
            r.last_addr = matched_address();
            // The refusal is a RECEIVER's: armed on a read tenure it would
            // refuse the next address instead.
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
            continue;
        }
        // THE COUNTED EXIT, looked for only with no flag pending: `count`
        // bytes moved and THE WIRE AT REST after them, watched for forty
        // microseconds - longer than any SCL high half a controller makes
        // (five at 100 kHz), so a tenure in flight never passes for its end.
        // NOT A STOP: a target transmitter's sequence ends in the closing
        // NACK (RM 15.4's EVT3_2) where a receiver's ends in STOPF (EVT4), and
        // a read of eight closed by NACK and Stop - the flush's PE cycle
        // between the two - moves this loop's Stop count not at all
        // (measured: the counted serve the DUT suites' letter d judges, which
        // with a Stop as its witness waited out the deadline). Nor the moment
        // the count is reached: a transmitter is asked for its bytes ahead of
        // the wire, and a re-init then would take the last one out of DATAR.
        if (a.count != 0u && static_cast<uint16_t>(rx + tx) >= a.count && wire_rests(40u)) {
            break;
        }
    }
    Client::acknowledge(true);
    r.count = static_cast<uint16_t>(rx + tx);
    r.aux0 = static_cast<uint8_t>(rx);
    r.aux1 = static_cast<uint8_t>(tx);
    r.sstatus = status_byte();
    return r;
}

/// THE HOST ACTION, a ROLE SWITCH (the header). The rendezvous is the
/// DUT's window - SDA pulled low by hand under a high SCL, a START
/// condition every controller on the wire sees - and the STOP that closes
/// it; this end's start() runs on that STOP. `arb` writes the pattern to
/// `target`; `host_read` reads `count` bytes from it, `aux8` times back to
/// back, and the report carries the bytes read (twi_link.hpp).
twilink::Report run_host(const twilink::Params& a, bool read) {
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

    // THE WIRE AT REST FIRST: this action starts on the closing NACK of the
    // read that collected its ack, a bit time before the DUT's STOP, and a
    // window watched for from there would open on that tail. A BUSY the
    // role switch left standing over the idle wire is cleared by the
    // engine's recover() before the start() that reads it.
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

    r.flags |= twilink::report_host_ran;
    // ONE TENURE, waited out on the vectors' completion within the deadline.
    const auto tenure = [&]() -> bool {
        host_done = false;
        const bool sync_done =
            read ? Host::start({.addr = a.target,
                                .tx = {},
                                .tx_len = 0,
                                .rx = lend<Lease::reply>(static_cast<uint8_t*>(burst)),
                                .rx_len = n,
                                .reply = {},
                                .speed = I2cSpeed::standard_100k})
                 : Host::start({.addr = a.target,
                                .tx = lend<Lease::reply>(static_cast<const uint8_t*>(burst)),
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
        return host_done;
    };
    // host_read's `aux8` reads, back to back (twi_link.hpp); arb writes once.
    const uint8_t tenures = read ? (a.aux8 == 0u ? uint8_t{1} : (a.aux8 > 4u ? uint8_t{4} : a.aux8))
                                 : uint8_t{1};
    for (uint8_t k = 0; k < tenures; ++k) {
        if (read) {
            for (uint8_t i = 0; i < n; ++i) {
                burst[i] = 0xEE;
            }
        }
        if (!tenure()) {
            r.flags |= twilink::report_timed_out;
            (void)Host::recover();
            break;
        }
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
        if (!read || host_status != i2c_ok) {
            break;
        }
        ++r.addr_hits;
        r.count = static_cast<uint16_t>(r.count + n);
        if (k == 0u) {
            r.first = burst[0];
        }
        r.aux0 = burst[0];
        for (uint8_t i = 0; i < n; ++i) {
            r.sum = static_cast<uint16_t>(r.sum + burst[i]);
        }
    }
    host_live = false;
    (void)go_command();
    return r;
}

/// A STUCK target, by hand: SDA held low from the port with the I2C off
/// the pads, released after `aux8` SCL falling edges (0 = only the
/// deadline does). THIS SERIES HAS NO OPEN-DRAIN GPIO (8.2.8: the open
/// drain is the I2C's own), so the line is pulled down by a PUSH-PULL
/// output over a clear bit - which only ever drives the level every other
/// node on the wire may drive - and a floating INPUT gives it back to the
/// rail's resistors. The rail stays up throughout.
twilink::Report run_hold_sda(const twilink::Params& a) {
    twilink::Report r{};
    Client::release();
    SclPin::input();
    // SDA IS TAKEN ONCE THE WIRE IS AT REST (the header).
    const uint32_t t0 = Ticker::millis();
    (void)wait_wire_quiet(t0, a.ms, 40u);
    SdaPin::output(false);
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

/// This board's I2C off the wire for the deadline: the control case. The
/// peripheral is released and the two pads float on the rail, which stays
/// up - the bus keeps its pull-ups whoever is quiet.
twilink::Report run_quiet(const twilink::Params& a) {
    twilink::Report r{};
    Client::release();
    wait_ms(a.ms);
    r.count = 1;
    return r;
}

/// The part number after its CH32 prefix, NUL padded to the label's eight
/// characters.
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
        d.xtal = 0;   // this series has no crystal oscillator
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
    if (op == Op::counters) {
        twilink::Counters k{};
        k.drops = last_drops;
        k.drops_total = static_cast<uint16_t>(flushes);
        k.stray = static_cast<uint16_t>(stray_vectors);
        k.commands = static_cast<uint16_t>(commands);
        uint8_t p[twilink::counters_size];
        twilink::put_counters(p, k);
        set_response(Op::counters_data, p, twilink::counters_size);
        return;
    }

    ++actions;
    const twilink::Params a = pending_params_;
    const uint32_t started = Ticker::millis();
    const uint32_t flushes0 = flushes;
    twilink::Report r{};
    switch (op) {
        case Op::serve: r = run_serve(a, false); break;
        case Op::coll: r = run_serve(a, true); break;
        case Op::arb: r = run_host(a, false); break;
        case Op::host_read: r = run_host(a, true); break;
        case Op::hold_sda: r = run_hold_sda(a); break;
        case Op::quiet: r = run_quiet(a); break;
        default: break;
    }
    r.flags |= twilink::report_ran;
    last_drops = static_cast<uint16_t>(flushes - flushes0);
    const uint32_t took = Ticker::millis() - started;
    r.ms = took > 255u ? uint8_t{255} : static_cast<uint8_t>(took);
    r.op = twilink::byte_of(op);
    last_report = r;
    (void)go_command();
}

// ---- the supply ------------------------------------------------------------------

/// THE SUPPLY at boot (the desk's rule for an image that drives a link
/// pad): the voltage detector at PLS 10, 3.0 V falling (pwr.hpp), given
/// 200 us before PVD0 is trusted - the manual states no settling time.
bool supply_above_floor() {
    Pwr::pvd(PvdLevel::v3_0);
    (void)delay_us(clock, 200);
    return !Pwr::supply_low();
}

// ---- the console ----------------------------------------------------------------

void help() {
    print(console, "twi_peer: ? help | i status | 0 back to command mode | 3 trace", crlf);
}

/// AN IDLE-BUS COMMAND: `busy()` is a bare STAR2 read, which consumes an
/// address match that rises between the loop's last look at STAR1 and it
/// (15.10.7). Typed between tenures, as a human does, it costs nothing.
void status() {
    print(console, "  I2C1 PC16/PC17, rail PB3 ", supply_ok ? "high" : "FLOATING", ", enabled=",
          Raw::enabled(), " STAR1=", hex(Raw::status1()),
          " own address=", hex(own_addr_), crlf);
    print(console, "  SDA reads ", SdaPin::read() ? "high" : "LOW", ", SCL reads ",
          SclPin::read() ? "high" : "LOW", ", bus busy=", Raw::busy(), crlf);
    print(console, "  commands=", commands, " actions=", actions, " naks=", naks,
          " answers=", answers, " dropped=", flushes, " stray vectors=", stray_vectors, crlf);
    print(console, "  last report: op=", hex(last_report.op), " count=", last_report.count,
          " rx/tx=", last_report.aux0, "/", last_report.aux1, " hits=", last_report.addr_hits,
          " stops=", last_report.stops, " flags=", hex(last_report.flags), crlf);
    print(console, "               sum=", hex(last_report.sum), " first=", hex(last_report.first),
          " lastaddr=", hex(last_report.last_addr), " STAR1-lo=", hex(last_report.sstatus),
          " at-NACK=", hex(last_report.mstatus), " ms=", last_report.ms, crlf);
}

}  // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
extern "C" BRIO_CH32_INTERRUPT void usart2_handler() { (void)Console::isr(); }

/// The instance's TWO vectors. The TARGET is POLLED, so the only owner here
/// is the HOST action's engine. Anything else that reaches a vector is
/// DISARMED AND NOTHING IS CLEARED: every flag this instrument lives on
/// belongs to the loop, and a handler that consumed one would take a whole
/// tenure with it.
extern "C" BRIO_CH32_INTERRUPT void i2c1_ev_handler() {
    if (host_live) {
        if (Host::isr()) {
            host_status = Host::status();
            host_done = true;
        }
        return;
    }
    stray_vectors = stray_vectors + 1u;
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
    stray_vectors = stray_vectors + 1u;
    Raw::error_interrupt(false);
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool console_ok = Console::init(clock, 115200);
    const bool tick_ok = Ticker::init(clock);
    enable_interrupts();

    // THE SUPPLY FIRST, THE RAIL ONLY THEN (the header): PB3 floats from
    // the reset until here, and stays floating on a low supply.
    supply_ok = supply_above_floor();
    if (supply_ok) {
        Rail::output(true);
    }

    char label[9] = {};
    ident_label(label);
    const bool client_ok = supply_ok && go_command();

    print(console, crlf, "twi_peer - I2C instrument target (", device::part_name, ", label '",
          label, "', fw ", hex(firmware_version), ")", crlf);
    print(console, "target I2C1 column 010: SCL PC16, SDA PC17, open drain; the pull-ups are "
                   "1.5 kOhm from each line to PB3, driven high by this image for its whole run; "
                   "command address ",
          hex(twilink::command_addr), " exactly - no general call, no second address; FREQ ",
          SysClock::hz / 1'000'000u, " MHz", crlf);
    print(console, "boot: clk=", clock_ok ? "HSI48" : "FAILED", " tick=", tick_ok ? "STK" : "FAILED",
          " console=", console_ok ? "USART2" : "FAILED", " supply=", supply_ok ? "above 3.0 V" : "LOW",
          " rail=", supply_ok ? "high" : "floating", " target=", client_ok ? "up" : "OFF", crlf);
    if (!supply_ok) {
        print(console, "SUPPLY UNDER 3.0 V (the PVD at PLS 10): PB3 and the bus pads stay floating",
              crlf);
    }
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
                    if (supply_ok) {
                        (void)go_command();
                    }
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

        if (!supply_ok) {
            continue;
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
