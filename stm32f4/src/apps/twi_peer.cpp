// twi_peer - a scriptable second chip on the bus for the other end of a
// two-board I2C test: the far board is the DUT and the bus controller,
// and it drives this one IN BAND over the very bus both are testing.
//
// The wire format is twi_link.hpp, included by relative path: one file
// is the single source of truth for every board that speaks it,
// whatever its architecture. What this silicon makes of it:
//
//  - ONE I2C IS HOST *OR* TARGET HERE. The chapter itself allows both
//    halves at once - a controller that loses the arbitration becomes a
//    target and may be addressed in the same tenure (27.3.3) - but this
//    stratum's two tasks cannot stand together: `I2cHost::init()` pulses
//    the peripheral's RCC reset line, which clears OAR1 and OAR2, and
//    every `start()` clears CR1.ACK, with which no own address is
//    answered. So the `arb` action SWITCHES the instance to controller
//    for its bounded moment and back. During a host action this board's
//    target is simply absent, which the choreography already tolerates
//    (the DUT waits out the action's deadline either way).
//  - THE ROLE SWITCH NEEDS NO RELEASE, and that matters on this board:
//    every init of either face begins with that same reset, so the pads
//    can stay in their alternate function from boot to boot of a role -
//    and they are the wire's only pull-ups (below).
//  - THE TARGET STRETCHES BY CONSTRUCTION, ONE BYTE LATE. ADDR holds SCL
//    until the SR1-then-SR2 read, and a receiver holds it at BTF - which
//    is one byte deeper than it sounds: this block takes the NEXT byte
//    into its shift register while the previous one still sits in DR, so
//    a commanded hold SHORTER THAN A BYTE TIME never reaches the wire at
//    all. The same one-byte lead is on the transmit side. The hold is
//    therefore spent BEFORE the answer, exactly where the stretch is, and
//    what a controller measures is the excess over one byte time.
//  - A TARGET CANNOT NACK ITS OWN ADDRESS: the address is acknowledged
//    from CR1.ACK before ADDR rises, so "deaf" is a re-init at
//    `deaf_addr`. Refusing a DATA byte is CR1.ACK too, and this chapter
//    has no per-byte decision point (no transfer-reload, no byte
//    control): ACK governs the byte whose ninth pulse has not yet come,
//    so the refusal is a bit set IN ADVANCE - down once the byte before
//    the refused one has been taken. `report_nacked` therefore says the
//    acknowledge was held down for that byte, not that a ninth pulse was
//    watched; the DUT's own `i2c_nack_data` is the other half of that
//    measurement. And because ACK down also refuses an ADDRESS, it goes
//    back up at every Stop, or the instrument would go deaf for good.
//  - A TARGET TRANSMITTER IS ASKED FOR ONE BYTE MORE THAN THE CONTROLLER
//    TAKES: DR empties into the shifter and TxE rises again, so the byte
//    standing in DR when the closing NACK arrives never reaches the wire.
//    This chapter has no verb that throws it away, so the instrument does
//    two things. In COMMAND MODE it never loads it: the protocol's read
//    is exactly `response_bytes` long, so exactly that many are given and
//    the twenty-first TxE is left standing. In an ACTION, where the
//    controller's count is not known here, the byte is loaded and then
//    DROPPED BY A PE CYCLE (27.6.1: with PE down "the internal state
//    machines are reset and the communication control bits as well as the
//    status bits come back to their reset value", while CR2, OAR1, OAR2,
//    CCR and TRISE are untouched - so the address and the timing survive),
//    and the tally gives the byte back. BOTH HALVES ARE MEASURED NOW: the
//    command channel's twenty-byte read closes cleanly on the
//    controller's NACK, so 27.3.3's BTF stretch falls AFTER the ninth
//    pulse; and a controller at the far end reads the same answer twice
//    running across the PE cycle, so the address and the timing really do
//    survive it. The console's `i` counts the drops as `dropped=`.
//  - AND THE GIVE PUMP IS GATED ON A READ TENURE, not on the flag alone.
//    TxE stands between tenures too, and a pump that only looks at it
//    writes a byte into DR while the bus is idle - after which THE NEXT
//    READ WEDGES: the target holds SCL low for ever with SR1 reading
//    zero, SR2 showing BUSY and TRA, and DR full, and nothing short of
//    the block's own RCC reset lets the line go (measured from both ends,
//    with the controller's trail showing the clock stopping right after
//    the address acknowledge). So `serving_` follows the direction of the
//    last address match and is cleared at every NACK and every Stop, and
//    only inside a read tenure is DR ever written.
//  - flag_stop_interrupt maps to nothing: STOPF is a polled flag here and
//    the Stop count is kept whenever the action loop sees it.
//  - THE TARGET'S TIMING IS FREQ AND NOT A RATE. `I2cClient::init()`
//    solves the standard-mode row and writes all three registers, but CCR
//    is the CONTROLLER's divider and a target does not use it: what a
//    target needs is CR2.FREQ carrying the APB1 clock in whole megahertz,
//    because 27.6.2 makes that the number the block generates its data
//    setup and hold times from. So this end sits at 45 MHz of FREQ and
//    the DUT may walk whatever rungs it likes against it.
//  - THE INTERNAL PULL-UPS ARE THIS BOARD'S CONTRIBUTION TO THE WIRE.
//    PUPDR applies in alternate-function mode, so the two pads can hold
//    the bus up with some tens of kiloohms - far too weak for a fast edge
//    and exactly enough for a short two-board link with no resistors on
//    it. The driver's pad claim writes PUPDR to `none` (its own comment
//    says the pull-ups are external), so every init here is followed by
//    `Pin::pull()` on both pads, and the two actions that take the pads
//    away from the peripheral - `hold_sda` and `quiet` - park them as
//    INPUTS WITH THEIR PULL-UPS rather than releasing them, because a
//    release would take the bus's only pull-ups with it and the DUT would
//    measure a dead wire instead of an absent client.
//
// COEXISTENCE is the protocol header's own argument: the command channel
// is ONE exact target address (0x6B) - no mask, no general call, no
// second address - so nothing the DUT's single-board letters do can wake
// this board. ENDUAL, the second own address this block offers, is on the
// resource and is not what the `coll` action means: that case is two
// boards answering ONE address, so the peer simply takes `shared_addr` in
// OAR1 and the wired-AND on the wire is the experiment.
//
// Bus: I2C1 at AF4, open drain, the pads a Nucleo-64 carries on its
// Arduino header -
//   PB8  SCL
//   PB9  SDA
// pulled up by the port alone (see above), on a two-wire node with a
// dedicated GND. The kernel clock is APB1 (45 MHz at 180 MHz of SYSCLK).
//
// Console: USART2 PA2/PA3 at 115200 on the ST-LINK's virtual COM port,
// observability only.
//   ? help | i status and counters | 0 back to command mode | 3 trace
//
// The ident label is the 96-bit unique device ID's first word in hex
// (RM0390 34.1 through brio::DeviceUid); ident.xtal is ALWAYS 0, because
// a Nucleo-64 fits no HSE crystal - the 8 MHz this board runs its PLL
// from is the ST-LINK's clock output, taken in bypass mode.
//
// build: boards = f446re
// build: monitor_speed = 115200

#include <stdint.h>

#include "stm32f4/clock.hpp"
#include "stm32f4/delay.hpp"
#include "stm32f4/flash.hpp"
#include "stm32f4/i2c.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/usart.hpp"
#include "util/print.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED: pure encoding, not one
// register, and every architecture on this link compiles the same file.
#include "../../../avrdx/src/apps/twi_link.hpp"

using SysClock =
    brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000, brio::HseMode::bypass>;
constexpr SysClock clock{};

namespace {

using namespace brio;
using twilink::Op;

constexpr UartPins console_pins{
    .tx = {'A', 2, PinFunction::af7},
    .rx = {'A', 3, PinFunction::af7},
};
using Console = Uart<2, console_pins>;
constexpr Console console;

/// DS10693 table 11: I2C1_SCL on PB8 and I2C1_SDA on PB9 at AF4 - the
/// pair a Nucleo-64 brings out as its Arduino D15/D14.
constexpr I2cPins bus_pins{
    .scl = {'B', 8, PinFunction::af4},
    .sda = {'B', 9, PinFunction::af4},
};

using Client = I2cClient<1, bus_pins>;
using Host = I2cHost<1, bus_pins>;
using Raw = I2c<1>;

using SclPin = Pin<bus_pins.scl.port, bus_pins.scl.pin>;
using SdaPin = Pin<bus_pins.sda.port, bus_pins.sda.pin>;

constexpr uint16_t firmware_version = 0x0400;   ///< see twi_link.hpp's Ident

bool trace = false;

twilink::Decoder decoder;
twilink::Report last_report;

uint32_t commands = 0, actions = 0, naks = 0, answers = 0, flushes = 0;

uint8_t resp_[twilink::response_bytes];
uint8_t resp_len_ = 0;
uint8_t resp_pos_ = 0;
uint8_t given_ = 0;          ///< bytes handed to DR in the tenure under way
/// IS A READ TENURE UNDER WAY? The give pump is gated on it, because a
/// byte written into DR with no controller reading WEDGES THE NEXT READ
/// (the header's finding): TxE stands between tenures too, and a pump
/// that only looks at the flag fills DR while the bus is idle.
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

/// A microsecond hold of arbitrary length: delay_us is capped below one
/// SysTick period BY DESIGN, and this instrument's commanded stretches
/// (milliseconds per byte) are exactly the blocking waits that cap exists
/// to keep out of KERNEL programs - this is not one, so the hold is spent
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

/// The low byte of SR1 - SB, ADDR, BTF, ADD10, STOPF, RxNE and TxE -
/// which is what fits the protocol's one-byte status fields and is the
/// half that says what a tenure was doing.
uint8_t status_byte() { return static_cast<uint8_t>(Raw::status1() & 0xFFu); }

// ---- the pads and the two standing configurations -------------------------------

/// The two pads' pull-ups, put back after every claim: this board holds
/// the bus up by itself (see the header) and the driver's pad claim
/// writes PUPDR to `none`.
void hold_bus_up() {
    SclPin::pull(PinPull::up);
    SdaPin::pull(PinPull::up);
}

/// THE TARGET IS POLLED HERE, and `I2cClient::init()` arms all three
/// interrupt enables. Left on, ADDR alone would storm the event vector
/// with no owner, and the per-byte hold this instrument is asked for has
/// no place in an ISR. So the three enables go down; the NVIC lines stay
/// as the driver left them and the two handlers at the bottom sweep
/// anything that slips past.
void mute_client_vectors() {
    Raw::event_interrupt(false);
    Raw::buffer_interrupt(false);
    Raw::error_interrupt(false);
}

/// Bring the target up at one exact address, with the pull-ups back.
bool bring_up_client(uint8_t addr, bool general_call) {
    host_live = false;
    own_addr_ = addr;
    const bool ok = Client::init(clock, {.own = addr, .general_call = general_call});
    mute_client_vectors();
    hold_bus_up();
    given_ = 0;
    serving_ = false;
    return ok;
}

/// COMMAND MODE: one exact address, nothing else. Everything returns
/// here.
bool go_command() {
    const bool ok = bring_up_client(twilink::command_addr, false);
    decoder.reset();
    resp_pos_ = 0;
    read_done_ = false;
    return ok;
}

/// What an action asked this target to become.
bool apply_client(const twilink::Params& a) {
    const uint8_t addr = (a.flags & twilink::flag_deaf)
                             ? twilink::deaf_addr
                             : (a.addr ? a.addr : twilink::command_addr);
    return bring_up_client(addr, (a.flags & twilink::flag_general_call) != 0u);
}

/// This block does not report the matched address as a byte: SR2 says
/// WHICH own address it was and the value is the one this end
/// configured. CALLED ONLY RIGHT AFTER answer_address(), because the bare
/// SR2 read underneath is what would clear an ADDR that rose since the
/// last read of SR1 (27.6.7) - and there the preceding SR1 read is
/// answer_address()'s own, already spent on its own SR2 read.
uint8_t matched_address() {
    return Client::general_call_matched() ? 0x00u : own_addr_;
}

/// The byte a target transmitter was asked for and the controller never
/// took - see the header. TAKEN AT THE END OF EVERY READ TENURE, not
/// only when TxE says DR is full: the flag cannot see the SHIFTER, and a
/// tenure closed with TxE standing has been measured to leave a byte
/// inside anyway - after which the next read HANGS, this end holding SCL
/// with no flag raised at either end.
///
/// PE IS DOWN FOR A FEW BUS READS AND NOT ONE INSTRUCTION LONGER. The
/// block wants the LEVEL - two adjacent stores are one bus cycle and the
/// state machine does not always see them - but a controller that puts
/// its next START on the wire microseconds after the one it just closed
/// finds a deaf target if the window is a microsecond wide (measured:
/// the next tenure's address matched and the clock then stopped). Three
/// reads of the register are the shortest window that is still a level.
/// True when DR was visibly full, which is what the console counts as
/// `dropped=`.
bool drop_unclocked_byte() {
    const bool had_one = !Client::data_wanted();
    Raw::disable();
    (void)Raw::enabled();
    (void)Raw::enabled();
    (void)Raw::enabled();
    Raw::enable();
    Raw::ack(true);   // a control bit, so the PE cycle took it down
    if (had_one) {
        ++flushes;
    }
    return had_one;
}

// ---- the command channel --------------------------------------------------------

void prepare(const twilink::Frame& f);
void prepare_nak(Op op, uint8_t sum);

/// One pass of the command-mode target protocol, POLLED, on the
/// I2cClient verbs. THE ORDER IS LOAD-BEARING: the address match first
/// (it is the only place the DIRECTION can be read, because the sequence
/// that reports it is the sequence that clears ADDR), then the data
/// flags, and the closing NACK of a read is what says "the answer has
/// been collected" - ack-before-act's own trigger.
void service_command() {
    if (Client::addressed()) {
        // EV1: the SR1-then-SR2 read answers the match, RELEASES SCL and
        // returns the direction in one act.
        if (Client::answer_address()) {
            resp_pos_ = 0;   // a read: the prepared answer, from its first byte
            given_ = 0;
            serving_ = true;
        } else {
            decoder.reset();   // a write: one tenure is exactly one frame
            serving_ = false;
        }
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
    // THE CLOSING NACK BEFORE THE GIVE PUMP: AF says the controller has
    // stopped taking, and TxE stands for a while yet - a pump that looked
    // at TxE first would feed a tenure that is already over.
    if (Client::host_nacked()) {
        Client::clear_nack();
        serving_ = false;
        (void)drop_unclocked_byte();
        read_done_ = true;
        return;
    }
    // EXACTLY `response_bytes` ARE GIVEN AND NO MORE: the protocol's read
    // is that long, and the byte after the last one would be loaded into
    // DR and never clocked out (see the header).
    if (serving_ && Client::data_wanted() && given_ < twilink::response_bytes) {
        Client::give(resp_pos_ < resp_len_ ? resp_[resp_pos_++] : 0x00u);
        ++given_;
        return;
    }
    if (Client::stop_seen()) {
        serving_ = false;
        Client::clear_stop();
        return;
    }
    // Every error flag is a LEVEL: clear it, or a pump that merely looks
    // at it comes back to the same one for ever. THE CLOSING NACK IS ONE
    // OF THEM, and it may rise in the microseconds between the check
    // above and this one: swallowed here it would cost a whole command,
    // because the action starts on it (measured: two commands in six
    // acknowledged and never run). So the NACK is answered wherever it
    // is found, and only the rest is merely cleared.
    const uint32_t errs = Raw::status1() & I2cFlag::errors;
    if (errs != 0u) {
        if ((errs & I2cFlag::ack_failure) != 0u) {
            serving_ = false;
            (void)drop_unclocked_byte();
            read_done_ = true;
        }
        Raw::clear_errors(errs);
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

// ---- the actions ----------------------------------------------------------------

/// The refusal, in this block's own terms: CR1.ACK governs the byte whose
/// ninth pulse has not yet come, so it goes down once the byte BEFORE the
/// refused one has been taken and stays down from there (the other peers
/// refuse from `nack_at` onwards too).
void arm_ack(twilink::Report& r, const twilink::Params& a, uint16_t received) {
    if (a.nack_at == 0) {
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
/// spent BEFORE the answer, which is where this silicon stretches SCL,
/// one byte deep (see the header).
twilink::Report run_serve(const twilink::Params& a, bool fixed_byte) {
    twilink::Report r{};
    if (!apply_client(a)) {
        return r;
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
            // THE ADDRESS MATCH IS A STRETCH TOO - ADDR holds SCL exactly
            // as BTF does - and the commanded hold is spent here as well
            // as before every data byte, because the LAST byte's hold
            // falls after the controller's own tenure is over.
            if (a.hold_us) hold_us(a.hold_us);
            const bool reads = Client::answer_address();
            serving_ = reads;
            r.last_addr = matched_address();
            // The refusal is a RECEIVER's: arming it on a read tenure
            // would refuse the next address and report a NACK that never
            // had a byte to land on.
            if (!reads) arm_ack(r, a, rx);
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
            arm_ack(r, a, rx);
            continue;
        }
        if (Client::host_nacked()) {
            // BEFORE THE GIVE PUMP, and the flags sampled AT the closing
            // NACK - a copy taken when the action ends carries whatever
            // the last tenure left behind.
            r.mstatus = status_byte();
            serving_ = false;
            Client::clear_nack();
            // AND THE TALLY GIVES ONE BYTE BACK when there was one to
            // drop: it was loaded into DR and never clocked out.
            if (drop_unclocked_byte() && tx != 0u) {
                --tx;
            }
            continue;
        }
        if (serving_ && Client::data_wanted()) {
            if (a.hold_us) hold_us(a.hold_us);
            Client::give(fixed_byte ? a.seed : twilink::pattern_value(a.pattern, a.seed, tx));
            ++tx;
            continue;
        }
        if (Client::stop_seen()) {
            if (r.stops < 255) ++r.stops;
            serving_ = false;
            Client::clear_stop();
            // ACK BACK UP, or an address would be refused as well and the
            // instrument would go deaf for the rest of the action.
            Client::acknowledge(true);
            continue;
        }
        if ((Raw::status1() & I2cFlag::errors) != 0u) {
            const uint32_t s1 = Raw::status1();
            if ((s1 & I2cFlag::arbitration_lost) != 0u) {
                r.flags |= twilink::report_arblost;
            }
            if ((s1 & I2cFlag::bus_error) != 0u) {
                r.flags |= twilink::report_buserr;
            }
            Raw::clear_errors(I2cFlag::errors);
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
/// second half (see the header). The rendezvous is the bus going BUSY -
/// the DUT's own tenure, or a line it holds - and after the lead-in this
/// end writes one tenure of its own; the engine's `start()` waits a
/// moment for BUSY to fall and then sets START, which the hardware itself
/// holds back until the bus is free, so the two controllers' STARTs meet
/// where the arbitration is decided.
twilink::Report run_arb(const twilink::Params& a) {
    twilink::Report r{};
    host_live = true;
    if (!Host::init(clock)) {
        host_live = false;
        (void)go_command();
        return r;
    }
    // THE PULL-UPS FIRST, THEN THE STATE MACHINE AGAIN. init() claims the
    // pads (PUPDR back to `none`) and only then pulses SWRST, so BUSY -
    // which is not a stored bit but "SDA or SCL seen low", cleared only by
    // a Stop (27.6.7) - is sampled over a wire that has no pull-up for
    // those few microseconds. A bus left BUSY would fire the rendezvous
    // below with no DUT in sight, so the pulls go on and recover() takes
    // the machine through SWRST once more, over a wire with a level.
    hold_bus_up();
    (void)Host::recover();

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
    (void)go_command();
    return r;
}

/// A STUCK target, bit-banged: SDA held low from GPIO with the I2C off
/// the pads, released after `aux8` SCL falling edges (0 = only the
/// deadline). Open drain by hand: an OUTPUT over a clear ODR pulls the
/// line down, an INPUT WITH ITS PULL-UP gives it back to the wire - and
/// the pull-up is this board's, so SCL keeps its own throughout.
twilink::Report run_hold_sda(const twilink::Params& a) {
    twilink::Report r{};
    Client::release();
    SclPin::input(PinPull::up);
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
    SdaPin::input(PinPull::up);
    r.count = falls;
    r.aux0 = static_cast<uint8_t>(falls);
    r.aux1 = SdaPin::read() ? 1u : 0u;
    return r;
}

/// This board off the wire for the deadline: the control case. The
/// peripheral is released and nothing is driven or answered - but the two
/// pads stay INPUTS WITH THEIR PULL-UPS, because they are the only ones
/// this bus has and a wire with no level is not the experiment.
twilink::Report run_quiet(const twilink::Params& a) {
    twilink::Report r{};
    Client::release();
    SclPin::input(PinPull::up);
    SdaPin::input(PinPull::up);
    wait_ms(a.ms);
    r.count = 1;
    return r;
}

void ident_label(char out[8]) {
    const uint32_t w0 = DeviceUid::read().word[0];
    for (uint8_t i = 0; i < 8; ++i) {
        const uint8_t nib = static_cast<uint8_t>((w0 >> (28u - 4u * i)) & 0xFu);
        out[i] = static_cast<char>(nib < 10 ? '0' + nib : 'a' + nib - 10);
    }
}

void run_pending() {
    const Op op = pending_;
    has_pending_ = false;

    if (op == Op::ping) return;
    if (op == Op::ident) {
        twilink::Ident d{};
        ident_label(d.label);
        d.xtal = 0;   // no HSE crystal on a Nucleo-64: the PLL runs off a bypass
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
    print(console, "twi_peer: ? help | i status | 0 back to command mode | 3 trace", crlf);
}

/// AN IDLE-BUS COMMAND: `busy()` is a bare SR2 read, which consumes an
/// address match that rises between the loop's last look at SR1 and it
/// (27.6.7). Typed between tenures, as a human does, it costs nothing.
void status() {
    print(console, "  I2C1 AF4, enabled=", Raw::enabled(), " SR1=", hex(Raw::status1()),
          " own address=", hex(own_addr_), crlf);
    print(console, "  SDA reads ", SdaPin::read() ? "high" : "LOW", ", SCL reads ",
          SclPin::read() ? "high" : "LOW", ", bus busy=", Raw::busy(), crlf);
    print(console, "  commands=", commands, " actions=", actions, " naks=", naks,
          " answers=", answers, " dropped=", flushes, crlf);
    print(console, "  last report: op=", hex(last_report.op), " count=", last_report.count,
          " rx/tx=", last_report.aux0, "/", last_report.aux1, " hits=", last_report.addr_hits,
          " stops=", last_report.stops, " flags=", hex(last_report.flags), crlf);
    print(console, "               sum=", hex(last_report.sum), " first=", hex(last_report.first),
          " lastaddr=", hex(last_report.last_addr), " SR1-lo=", hex(last_report.sstatus),
          " at-NACK=", hex(last_report.mstatus), " ms=", last_report.ms, crlf);
}

}  // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void USART2_IRQHandler() { (void)Console::isr(); }

/// The instance's TWO vectors. The TARGET is POLLED (its three interrupt
/// enables are down), so the only owner here is the HOST action's engine.
/// What can still reach a vector is the sliver between `I2cClient::init()`
/// arming the three enables and the mute that follows it: that is
/// DISARMED AND NOTHING IS CLEARED, because every flag this instrument
/// lives on belongs to the loop below and a handler that consumed one
/// would take a whole tenure with it.
extern "C" void I2C1_EV_IRQHandler() {
    if (host_live) {
        if (Host::isr()) {
            host_status = Host::status();
            host_done = true;
        }
        return;
    }
    brio::I2c<1>::event_interrupt(false);
    brio::I2c<1>::buffer_interrupt(false);
}

extern "C" void I2C1_ER_IRQHandler() {
    if (host_live) {
        if (Host::error_isr()) {
            host_status = Host::status();
            host_done = true;
        }
        return;
    }
    brio::I2c<1>::error_interrupt(false);
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool console_ok = Console::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    char label8[9] = {};
    ident_label(label8);
    const bool client_ok = go_command();

    print(console, crlf, "twi_peer - I2C instrument target (STM32F446RE board ", label8,
          ", xtal=none (the PLL runs off an 8 MHz bypass), fw ", hex(firmware_version), ")", crlf);
    print(console, "target I2C1 AF4: SCL PB8, SDA PB9, both pulled up by the PORT alone; "
                   "command address ", hex(twilink::command_addr),
          " exactly - no general call, no mask, no second address; FREQ carries APB1 at 45 MHz",
          crlf);
    print(console, "boot: clk=", clock_ok ? "PLL 180 MHz" : "FAILED",
          " tick=", tick_ok ? "SysTick" : "FAILED", " console=", console_ok ? "USART2" : "FAILED",
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
