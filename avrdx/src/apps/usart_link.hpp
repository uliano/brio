// usart_link.hpp - the board-to-board bench protocol of the USART
// suites: what test_avr_serial (the DUT) tells usart_peer (the
// instrument) over the very link both are testing.
//
// APP-LEVEL BENCH TOOLING, not framework. It sits next to the two apps
// that share it and is included by its plain name; nothing in
// brio knows it exists, and nothing here touches a register -
// it is pure encoding, so both apps can compile it and the host could
// too.
//
// THE FRAME
//
//   magic (0xB7) | op | len | payload[len] | checksum
//
// The checksum is the 8-bit sum of magic, op, len and every payload
// byte, XORed with 0xA5 (so an all-zero run does not check out). The
// peer answers EVERY command with one frame before acting:
//
//   ack  payload = {op, checksum}      the command was understood
//   nak  payload = {op, checksum}      the checksum did not match
//
// and the two query commands (ident, report) follow the ack with a
// second frame carrying the data. Nothing else is ever unsolicited:
// after a bounded action the peer goes back to command mode and WAITS
// to be asked for its report, so a DUT that lost track can simply keep
// pinging until it is answered.
//
// THE RECOVERY GUARANTEE
//
// Command mode is async 8N1 at `command_baud` on both boards. Every
// command that changes the link carries a bound - a frame count and a
// millisecond deadline - after which the peer restores command mode BY
// ITSELF. Whatever desynchronizes, both ends are back in command mode
// after the bound plus the quiet interval, and the DUT's client retries
// its command before failing a test.
//
// WHAT A PEER CANNOT DO, IT REFUSES. A peer whose silicon cannot serve
// an action's Cfg (or the action itself) still acks the frame, sets
// `report_cfg_failed` in that action's Report, and is back in command
// mode at once. A refusal is a capability the peer has not got, never a
// measurement: the DUT that reads it in the report SKIPS the leg with the
// reason and judges nothing of it (test_avr_serial's peer_report() and
// verdict()). The version's high byte (Ident) lets a DUT EXPECT a
// capability; the refusal is what decides it.
//
// TIME IN THE PAYLOAD. Every field that carries "cycles" or "ticks"
// counts TICKS OF A 24 MHz CLOCK, whatever the peer's core runs at - the
// glitch width, the bitbang bit, the Report's span. Each peer's own header
// states how its time base becomes these ticks (the avrdx peer's are its
// CPU cycles on the 24 MHz crystal, and its span two cascaded TCBs at
// CLK_PER; a peer on another clock converts its timer's counts).
//
// THE DARK DISCIPLINE, every peer's obligation. The link's pads are also
// the pads the DUT's SINGLE-board half measures - its loops, its pull
// walks, its bit-bangers - so a peer that is not being addressed must
// look like nothing on the wire:
//
//   - while it is SEARCHING (no frame has checked out, or the channel has
//     been quiet for rediscover_ms) its transmitter is released, its TXD
//     pad an input, and it holds NO PULL on any link pad: the far board's
//     single-board letters walk those wires with their own pulls. A
//     floating receiver may invent bytes; the magic, the length bound, the
//     checksum and the quiet-line reset absorb them;
//   - a pull-up on its receive pad, or its transmitter idling high, only
//     once a frame has PROVEN the wiring;
//   - a frame whose checksum fails is nak'ed only while the peer is
//     ENGAGED - within engage_ms of the last frame that checked out or of
//     the end of the action it commanded. A peer that has never been
//     addressed answers nothing, whatever noise decodes, so the far
//     board's own loop traffic on a shared wire is never answered.
//
// THE UNIFORM ACTION PAYLOAD
//
// Every action command carries the same 24-byte payload: a `Cfg` (what
// the peer's link should become) followed by a bound and a handful of
// per-op fields. One layout for all of them keeps both ends short; the
// fields an op does not use are simply zero. `send_mpcm` appends
// (address, count) pairs after it.

#pragma once

#include <stdint.h>

namespace link {

inline constexpr uint8_t magic = 0xB7;
inline constexpr uint8_t max_payload = 32;

/// Command mode: what both boards fall back to, always.
inline constexpr uint32_t command_baud = 115'200;

/// HOW THE TWO BOARDS ARE WIRED. The expected wiring is the crossed
/// full-duplex pair (each board's PE0 to the other's PE1, PE1 to the
/// other's PE0, PE2 to PE2), but a single wire between the two TXD
/// pads is a perfectly good link too -
/// it is the one-wire bus of 27.3.3.2.6, and with LBME each end hears
/// the pad. Both apps support both and FIND OUT which one is on the
/// desk (the peer alternates its command-mode configuration until a
/// frame arrives; the DUT pings in one topology, then the other), so a
/// desk that is not wired as expected reports a topology instead of
/// looking like a dead protocol. That is the two avrdx apps' arrangement
/// (the AVR's own LBME one-wire bus); a peer whose desk fixes the
/// crossed pair serves full duplex alone and goes onto a shared wire only
/// inside onewire and collide - a peer that guessed "shared" there would
/// drive the wrong wire.
///
/// On a shared line the discipline is: LBME on both ends, push-pull
/// transmitters, TXEN enabled only while transmitting, a PORT pull-up
/// holding the line during the turnaround, and the receiver disabled
/// while transmitting so the loop-back echo is not decoded. Nothing
/// needs a slower rate: the line is driven except between frames.
enum class Topology : uint8_t {
    full_duplex = 0,   ///< TXD -> RXD each way (the crossed pair)
    shared = 1,        ///< one wire between the two TXD pads
};

/// How long the peer stays in one command-mode topology before trying
/// the other, while no frame has arrived.
inline constexpr uint16_t rendezvous_ms = 400;

/// How long a PROVEN topology may stay silent before the peer stops
/// believing in it and goes back to alternating. This is what makes a
/// re-jumpered desk converge on its own: the wiring changes, the
/// command channel goes quiet, and after this long the peer starts
/// looking again. Long enough that no gap inside a running test set
/// reaches it (the longest bounded action is two seconds), short enough
/// that a human moving three wires does not have to think about it.
inline constexpr uint16_t rediscover_ms = 3000;

/// How long a frame that checked out (or the end of the action it
/// commanded) keeps the peer ENGAGED, and so willing to nak a frame that
/// does not (the dark discipline above). Shorter than rediscover_ms, so a
/// peer that naks always still believes its wiring; long enough for a
/// command/answer/action round trip of the DUT's client.
inline constexpr uint16_t engage_ms = 2000;
static_assert(engage_ms < rediscover_ms,
              "an engaged peer must still believe the wiring it would answer on");

/// The peer reconfigures right after its ack has left the line; the DUT
/// waits this long before doing the same, so the peer is always ready
/// first.
inline constexpr uint16_t settle_ms = 3;

/// The instrument's repertoire.
enum class Op : uint8_t {
    ping = 0x01,         ///< rendezvous: ack only
    ident = 0x02,        ///< ack + Ident (label, crystal flag, sanity byte)
    set_link = 0x03,     ///< apply a Cfg and idle until the bound
    echo = 0x04,         ///< echo every frame back until the bound
    sink = 0x05,         ///< receive silently until the bound
    send = 0x06,         ///< generate `count` frames of a pattern
    send_mpcm = 0x07,    ///< address frame + data frames, per group
    blast = 0x08,        ///< `count` frames with no inter-frame gap
    brk = 0x09,          ///< hold TXD low for `count` bit times
    autobaud_tx = 0x0A,  ///< break + sync char + payload at a foreign rate
    bitbang = 0x0B,      ///< a cycle-counted frame on TXD, one cell stretched
    glitch = 0x0C,       ///< sub-bit low pulses on an idle line
    drive_rxd = 0x0D,    ///< bit-bang a frame on the peer's OWN RXD pin
    onewire = 0x0E,      ///< join the desk's shared wire half-duplex until the bound
    collide = 0x0F,      ///< transmit into the DUT's frame, on purpose
    report = 0x10,       ///< ack + Report of the last action
    standby = 0x11,      ///< let go of the line entirely for `ms`
    calibrate = 0x12,    ///< time the DUT's gapless frames against the peer's clock
    hold_pad = 0x13,     ///< drive one named link pad at a level, the USART let go
    stage = 0x14,        ///< a drawn frame train: a pulse inside each frame, gaps in bit times

    ack = 0x40,
    nak = 0x41,
    ident_data = 0x42,
    report_data = 0x43,
};

constexpr uint8_t byte_of(Op op) { return static_cast<uint8_t>(op); }

/// The 8-bit sum of the whole frame, XORed so that a run of zeros does
/// not check out.
constexpr uint8_t checksum(uint8_t op, uint8_t len, const uint8_t* p) {
    uint8_t s = static_cast<uint8_t>(magic + op + len);
    for (uint8_t i = 0; i < len; ++i) {
        s = static_cast<uint8_t>(s + p[i]);
    }
    return static_cast<uint8_t>(s ^ 0xA5);
}

struct Frame {
    Op op = Op::ping;
    uint8_t len = 0;
    uint8_t sum = 0;                    ///< the checksum this frame carried
    uint8_t data[max_payload] = {};
};

/// Byte-at-a-time reassembly. Nothing here blocks and nothing here
/// allocates: feed() is called from whatever poll loop owns the USART.
class Decoder {
public:
    enum class Result : uint8_t { none, frame, bad_checksum, too_long };

    Result feed(uint8_t b) {
        switch (state_) {
            case 0:
                if (b == magic) { state_ = 1; sum_ = b; }
                return Result::none;
            case 1:
                f_.op = static_cast<Op>(b);
                sum_ = static_cast<uint8_t>(sum_ + b);
                state_ = 2;
                return Result::none;
            case 2:
                if (b > max_payload) { state_ = 0; return Result::too_long; }
                f_.len = b;
                sum_ = static_cast<uint8_t>(sum_ + b);
                idx_ = 0;
                state_ = b == 0 ? 4 : 3;
                return Result::none;
            case 3:
                f_.data[idx_++] = b;
                sum_ = static_cast<uint8_t>(sum_ + b);
                if (idx_ >= f_.len) state_ = 4;
                return Result::none;
            default:
                state_ = 0;
                f_.sum = b;
                if (b != static_cast<uint8_t>(sum_ ^ 0xA5)) return Result::bad_checksum;
                return Result::frame;
        }
    }

    void reset() { state_ = 0; idx_ = 0; }
    bool partial() const { return state_ != 0; }
    const Frame& frame() const { return f_; }
    /// The op of the frame being reassembled - what a nak must name.
    Op pending_op() const { return f_.op; }

private:
    uint8_t state_ = 0;
    uint8_t idx_ = 0;
    uint8_t sum_ = 0;
    Frame f_{};
};

/// Write one frame through a `void put(uint8_t)` callable.
template <typename Put>
void write_frame(Put put, Op op, const uint8_t* p, uint8_t len) {
    const uint8_t o = byte_of(op);
    put(magic);
    put(o);
    put(len);
    for (uint8_t i = 0; i < len; ++i) put(p[i]);
    put(checksum(o, len, p));
}

// ---- little-endian payload helpers -------------------------------------------

constexpr void put16(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
}
constexpr uint16_t get16(const uint8_t* p) {
    return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
}
constexpr void put32(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    p[2] = static_cast<uint8_t>(v >> 16);
    p[3] = static_cast<uint8_t>(v >> 24);
}
constexpr uint32_t get32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

// ---- the link configuration --------------------------------------------------

/// `Cfg::mode`: which protocol the peer's link should speak.
enum class Mode : uint8_t { async = 0, sync_client = 1, sync_host = 2, ircom = 3 };

/// `Cfg::bits`: the character size, with the two nine-bit orders spelt
/// out (the AVR's 9BITL and 9BITH differ only in which data register
/// shifts the FIFO, and a peer must agree with the DUT).
inline constexpr uint8_t bits9_low = 9;
inline constexpr uint8_t bits9_high = 10;

inline constexpr uint8_t flag_clk2x = 0x01;
inline constexpr uint8_t flag_invert_xck = 0x02;
inline constexpr uint8_t flag_mpcm = 0x04;

/// `Cfg::stop`: 1 and 2 count whole stop bits; a value with stop_halves
/// set counts HALF bits in its low bits - stop_half (0.5) and
/// stop_one_half (1.5), the frames a USART with the four-code STOP field
/// sends. A peer that has not got them refuses them.
inline constexpr uint8_t stop_halves = 0x80;
inline constexpr uint8_t stop_half = stop_halves | 1u;
inline constexpr uint8_t stop_one_half = stop_halves | 3u;

/// A frame's length on the wire in HALF bit times: start, data, parity
/// and stop - what calibrate's nominal span is counted in.
constexpr uint16_t frame_half_bits(uint8_t bits, uint8_t parity, uint8_t stop) {
    const uint8_t data = bits >= bits9_low ? 9u : bits;
    const uint8_t stop_halfs = (stop & stop_halves) != 0u ? static_cast<uint8_t>(stop & 0x0Fu)
                                                          : static_cast<uint8_t>(2u * stop);
    return static_cast<uint16_t>(2u * (1u + data + (parity != 0u ? 1u : 0u)) + stop_halfs);
}

/// What the peer's link should become. `apply == 0` leaves it alone
/// (the action runs at whatever the last command left behind).
struct Cfg {
    uint8_t apply = 1;
    Mode mode = Mode::async;
    uint8_t bits = 8;
    uint8_t parity = 0;              ///< 0 none, 1 even, 2 odd
    uint8_t stop = 1;
    uint8_t flags = 0;
    uint32_t rate = command_baud;    ///< baud, or XCK hertz in the sync modes
    uint8_t txpl = 0;                ///< IRCOM TXPLCTRL
    uint8_t rxpl = 0;                ///< IRCOM RXPLCTRL
};

inline constexpr uint8_t cfg_size = 12;

constexpr void put_cfg(uint8_t* p, const Cfg& c) {
    p[0] = c.apply;
    p[1] = static_cast<uint8_t>(c.mode);
    p[2] = c.bits;
    p[3] = c.parity;
    p[4] = c.stop;
    p[5] = c.flags;
    put32(p + 6, c.rate);
    p[10] = c.txpl;
    p[11] = c.rxpl;
}

constexpr Cfg get_cfg(const uint8_t* p) {
    Cfg c{};
    c.apply = p[0];
    c.mode = static_cast<Mode>(p[1]);
    c.bits = p[2];
    c.parity = p[3];
    c.stop = p[4];
    c.flags = p[5];
    c.rate = get32(p + 6);
    c.txpl = p[10];
    c.rxpl = p[11];
    return c;
}

// ---- the uniform action payload ----------------------------------------------

/// `Params::pattern` for `send` and `blast`.
inline constexpr uint8_t pattern_counting = 0;   ///< value, value+1, value+2, ...
inline constexpr uint8_t pattern_fixed = 1;      ///< value every time
inline constexpr uint8_t pattern_prbs = 2;       ///< an 8-bit LFSR seeded with value

/// `Params::value` for `hold_pad`: which pad of the peer's link, by the
/// role the pad plays in the link (its TXD, its RXD, its XCK), whatever
/// pin carries it on that peer.
inline constexpr uint8_t pad_txd = 0;
inline constexpr uint8_t pad_rxd = 1;
inline constexpr uint8_t pad_xck = 2;

/// Every action command carries exactly this. What an op does not use
/// stays zero; the meaning of the shared fields per op:
///
///   echo/sink/set_link/onewire  count = frames to process, ms = deadline
///   standby                     ms = how long to stay off the wire
///   send/blast                  count = frames to generate, value/pattern;
///                               for `send`, aux16 = extra lead-in in
///                               milliseconds before the first frame
///   blast (repeated)            aux16 = repeats of the `count` frames (0
///                               and 1 both mean once), aux8 = the gap
///                               between two repeats in 100 us units; the
///                               pattern RUNS ON across repeats (frame i of
///                               repeat k is pattern frame k * count + i),
///                               so a lost burst is a jump in the sequence;
///                               a repeated blast also ends at the ms
///                               deadline, checked between repeats, and
///                               the Report counts every frame generated
///                               (modulo 2^16)
///   brk                         count = bit times to hold TXD low
///   autobaud_tx                 value = the sync character, aux8 = payload
///                               frames, aux16 = break bit times
///   bitbang                     count = frames, value = the byte, cell =
///                               which bit cell to distort (0 = start,
///                               1..8 = data, 9 = stop, -1 = none), pct =
///                               by how much, aux16 = bit times of break
///                               ahead of the frames (0 = none)
///   glitch                      count = pulses, aux16 = pulse width in
///                               24 MHz ticks (the header's TIME), ms =
///                               deadline, aux8 = gap in microseconds
///   drive_rxd                   count = frames, value = the byte
///   collide                     value = the byte, aux16 = microseconds to
///                               wait after the trigger frame
///   onewire/collide             the peer joins THE DESK'S SHARED WIRE on
///                               whichever of its pads reaches it (a peer
///                               with a pin swap puts its single wire on
///                               the pad that meets the DUT's), open drain,
///                               taking the wire only to reply
///   calibrate                   count = frames the DUT sends GAPLESS at
///                               cfg.rate once settle_ms has passed after
///                               the ack, ms = deadline. The peer stamps
///                               each frame as it completes and reports
///                               the first-to-last span (Report::span)
///                               and the DUT's rate error against the
///                               peer's clock (Report::aux16, calibrate's
///                               entry). A peer whose Ident::xtal is 0
///                               ADOPTS the DUT as its reference - trims
///                               its oscillator and carries the residual
///                               in every rate, tick and edge it computes
///                               from then on - and reports xtal = 2; a
///                               crystal peer adopts nothing and reports
///                               the DUT's error
///   hold_pad                    value = the pad, by its LINK ROLE (pad_txd,
///                               pad_rxd, pad_xck), aux8 = the level (0 low,
///                               anything else high), ms = how long. The
///                               peer lets its USART go, drives that pad
///                               push-pull at the level for ms, gives it
///                               back undriven, and returns to command
///                               mode (the dark discipline's pulls): a
///                               far LEVEL on a wire whose USART function
///                               would not hold one (a synchronous host's
///                               XCK runs while its instance is enabled). Report::count = 1
///                               once held, aux16 = the pad's own readback
///                               at the end of the hold (1 high, 0 low) -
///                               a wire held the other way by the DUT shows
///                               there. A peer without that pad refuses it.
///   stage                       a FRAME TRAIN DRAWN WHOLE by the peer's
///                               waveform engine at cfg.rate - the shapes a
///                               clean UART cannot make and the errata name:
///                               count = frames, value/pattern = their data
///                               (pattern_value's sequence; a ninth data bit
///                               is 0), the frame from cfg.bits (5..8, or a
///                               nine), cfg.parity and cfg.stop (1 or 2);
///                               cell = the cell a PULSE sits in, in every
///                               frame (0 = the start bit, 1..bits = the
///                               data, then the parity bit where there is
///                               one, then the stop bits; -1 = no pulse),
///                               pct = where the pulse starts inside that
///                               cell, in per cent of a bit (0..99), aux8 =
///                               its width in per cent of a bit (0 = no
///                               pulse): the line driven to the level
///                               OPPOSITE the cell's for that long, whatever
///                               it crosses; aux16 = the idle between two
///                               frames in SIXTEENTHS of a bit at cfg.rate
///                               (0 = back to back). The frames' rate is
///                               the peer's time base, not a divisor's: a
///                               rate a per cent off a receiver's is that
///                               per cent. Report::count and sum = the
///                               frames drawn, aux16 = a bit's length in
///                               24 MHz ticks (bitbang's). A peer whose
///                               engine cannot draw the pulse that narrow,
///                               or the train in its edge table, refuses.
struct Params {
    Cfg cfg{};
    uint16_t count = 0;
    uint16_t ms = 200;
    uint8_t value = 0;
    uint8_t pattern = pattern_counting;
    int8_t cell = -1;
    int8_t pct = 0;
    uint16_t aux16 = 0;
    uint8_t aux8 = 0;
    uint8_t groups = 0;      ///< send_mpcm: how many (address, count) pairs follow
};

inline constexpr uint8_t params_size = 24;

constexpr void put_params(uint8_t* p, const Params& a) {
    put_cfg(p, a.cfg);
    put16(p + 12, a.count);
    put16(p + 14, a.ms);
    p[16] = a.value;
    p[17] = a.pattern;
    p[18] = static_cast<uint8_t>(a.cell);
    p[19] = static_cast<uint8_t>(a.pct);
    put16(p + 20, a.aux16);
    p[22] = a.aux8;
    p[23] = a.groups;
}

constexpr Params get_params(const uint8_t* p) {
    Params a{};
    a.cfg = get_cfg(p);
    a.count = get16(p + 12);
    a.ms = get16(p + 14);
    a.value = p[16];
    a.pattern = p[17];
    a.cell = static_cast<int8_t>(p[18]);
    a.pct = static_cast<int8_t>(p[19]);
    a.aux16 = get16(p + 20);
    a.aux8 = p[22];
    a.groups = p[23];
    return a;
}

// ---- the answers ---------------------------------------------------------------

inline constexpr uint8_t ident_size = 12;
inline constexpr uint8_t ident_sanity = 0x5B;   ///< "this firmware is usart_peer"

struct Ident {
    char label[8] = {};      ///< the peer's board label, NUL padded
    /// 1 when the peer's TIME BASE is crystal-referenced (its own crystal
    /// started, or its core clock was measured against one at boot), 0
    /// when its rates come from an RC oscillator's nominal value, 2 when
    /// that RC oscillator has ADOPTED a DUT as its reference through
    /// calibrate (trimmed, the residual carried) - as good as the DUT's
    /// clock and no better.
    uint8_t xtal = 0;
    uint8_t sanity = 0;
    /// The peer's firmware version. THE HIGH BYTE NAMES THE PEER
    /// IMPLEMENTATION and is allocated here, once, for every
    /// architecture that speaks this protocol - as spi_link.hpp
    /// allocates its own: 0x01 the avrdx peer, 0x02 the samc21 peer, 0x03
    /// the stm32g0 peer, 0x04 the stm32f4 peer, 0x05 the ch32vx03 peer,
    /// 0x06 the ch32v00x peer, 0x07 the
    /// ch32x035 peer, 0x08 the rp2040 peer, 0x09 the rp2350 peer (one
    /// kind for both of its architectures, the label naming the half).
    /// The low byte is that peer's own revision. A DUT may use the high
    /// byte to expect a capability, never to decide correctness: what a peer
    /// can do it answers with, and what it cannot it refuses.
    uint16_t version = 0;
};

/// The peer implementations Ident::version's high byte names.
inline constexpr uint8_t peer_avrdx = 0x01;
inline constexpr uint8_t peer_samc21 = 0x02;
inline constexpr uint8_t peer_stm32g0 = 0x03;
inline constexpr uint8_t peer_stm32f4 = 0x04;
inline constexpr uint8_t peer_ch32vx03 = 0x05;
inline constexpr uint8_t peer_ch32v00x = 0x06;
inline constexpr uint8_t peer_ch32x035 = 0x07;
inline constexpr uint8_t peer_rp2040 = 0x08;
inline constexpr uint8_t peer_rp2350 = 0x09;   ///< both architectures; the label names the half

/// Which implementation answered: the version's high byte.
constexpr uint8_t peer_kind(uint16_t version) { return static_cast<uint8_t>(version >> 8); }

constexpr void put_ident(uint8_t* p, const Ident& d) {
    for (uint8_t i = 0; i < 8; ++i) p[i] = static_cast<uint8_t>(d.label[i]);
    p[8] = d.xtal;
    p[9] = d.sanity;
    put16(p + 10, d.version);
}

constexpr Ident get_ident(const uint8_t* p) {
    Ident d{};
    for (uint8_t i = 0; i < 8; ++i) d.label[i] = static_cast<char>(p[i]);
    d.xtal = p[8];
    d.sanity = p[9];
    d.version = get16(p + 10);
    return d;
}

/// The Report's length on the wire. A DUT and a peer built from two
/// different generations of this header disagree on it, and the DUT's
/// report query then fails loudly (its client accepts this length alone)
/// - peers and DUTs are flashed from one tree.
inline constexpr uint8_t report_size = 16;

inline constexpr uint8_t report_timed_out = 0x01;   ///< the deadline ended it
inline constexpr uint8_t report_mismatch = 0x02;    ///< a frame was not what was expected
inline constexpr uint8_t report_cfg_failed = 0x04;  ///< the Cfg was refused by the driver
inline constexpr uint8_t report_ran = 0x08;         ///< the action really ran
/// send and blast: aux16 CARRIES the peer's rate - 0 then means exact to
/// the ppm. Clear (a peer that cannot tell, or an older one), aux16 = 0
/// means not known.
inline constexpr uint8_t report_rate_stated = 0x10;

/// What the peer saw while it was not in command mode.
struct Report {
    uint16_t count = 0;      ///< frames processed (received or generated)
    uint16_t sum = 0;        ///< running 16-bit sum of the data values
    uint8_t ferr = 0;
    uint8_t perr = 0;
    uint8_t ovf = 0;
    uint8_t flags = 0;
    /// Per-op extra: the BAUD register the peer used (brk, autobaud_tx),
    /// a bit's nominal length in 24 MHz ticks (bitbang), the pulse width
    /// it was asked for (glitch), the byte it collided with (collide),
    /// the held pad's readback at the end of the hold (hold_pad);
    /// calibrate: the DUT's character rate against the peer's clock in
    /// parts per million as an int16 (calibrate_ppm()), positive when the
    /// DUT runs fast, saturated at the int16 range; send and blast: the
    /// rate the PEER's transmitter really ran at against the cfg.rate it
    /// was asked for, in ppm as an int16 (rate_error_ppm()), positive when
    /// the peer runs fast - from its divisor and its time base (crystal,
    /// tracked or adopted), with report_rate_stated set in `flags`; a
    /// peer that cannot tell (or an older one) leaves the flag clear and
    /// aux16 0, and a DUT judges such a blast at its own loose bound;
    /// stage: a bit's length in 24 MHz ticks.
    uint16_t aux16 = 0;
    uint8_t op = 0;          ///< which action this report belongs to
    uint8_t ms = 0;          ///< how long it took, capped at 255
    /// sink, echo and calibrate: 24 MHz ticks from the FIRST to the LAST
    /// character the peer received (each stamped when the peer saw it
    /// complete), so (count - 1) frame times of the DUT's transmitter as
    /// a FOREIGN receiver measured them - on the peer's clock as it was
    /// before calibrate adopted anything. 0 = not measured (another op,
    /// fewer than two characters, or a peer that cannot time them).
    uint32_t span = 0;
};

/// calibrate's span had the DUT's clock been exact: `frames - 1` frames of
/// `half_bits` half bit times at `rate`, in 24 MHz ticks.
constexpr uint32_t nominal_span(uint32_t rate, uint16_t half_bits, uint16_t frames) {
    if (rate == 0u || frames < 2u) return 0u;
    return static_cast<uint32_t>((static_cast<uint64_t>(frames - 1u) * half_bits * 12'000'000ull +
                                  rate / 2u) / rate);
}

/// calibrate's error: the DUT's rate against the peer's clock in ppm,
/// positive when the measured span is SHORTER than the nominal one (the
/// DUT fast), saturated to int16.
constexpr int16_t calibrate_ppm(uint32_t nominal, uint32_t measured) {
    if (nominal == 0u || measured == 0u) return 0;
    const int64_t ppm = (static_cast<int64_t>(nominal) - static_cast<int64_t>(measured)) * 1'000'000 /
                        static_cast<int64_t>(measured);
    return static_cast<int16_t>(ppm > 32767 ? 32767 : (ppm < -32768 ? -32768 : ppm));
}
static_assert(nominal_span(115'200, 20, 101) == 208'333u, "100 8N1 frames at 115200: 8.68 ms");
static_assert(calibrate_ppm(1'000'000, 990'000) == 10'101, "a span 1 % short is a DUT 1.01 % fast");

/// send and blast's Report::aux16: a transmitter whose real rate is
/// `num / den` baud, against the `asked` rate, in ppm (positive = fast),
/// saturated to int16. The peer states its rate as a fraction because it
/// is one: its kernel clock over its divisor (in the divisor's own units).
constexpr int16_t rate_error_ppm(uint64_t num, uint64_t den, uint32_t asked) {
    if (den == 0u || asked == 0u) return 0;
    const int64_t base = static_cast<int64_t>(asked) * static_cast<int64_t>(den);
    const int64_t ppm = (static_cast<int64_t>(num) - base) * 1'000'000 / base;
    return static_cast<int16_t>(ppm > 32767 ? 32767 : (ppm < -32768 ? -32768 : ppm));
}
static_assert(rate_error_ppm(90'000'000u, 16u * 781u, 7'200u) == 320, "90 MHz / (16 x 781) vs 7200 baud");
static_assert(rate_error_ppm(64'000'000u, 10u, 6'400'000u) == 0, "an exact divisor is 0 ppm");

constexpr void put_report(uint8_t* p, const Report& r) {
    put16(p, r.count);
    put16(p + 2, r.sum);
    p[4] = r.ferr;
    p[5] = r.perr;
    p[6] = r.ovf;
    p[7] = r.flags;
    put16(p + 8, r.aux16);
    p[10] = r.op;
    p[11] = r.ms;
    put32(p + 12, r.span);
}

constexpr Report get_report(const uint8_t* p) {
    Report r{};
    r.count = get16(p);
    r.sum = get16(p + 2);
    r.ferr = p[4];
    r.perr = p[5];
    r.ovf = p[6];
    r.flags = p[7];
    r.aux16 = get16(p + 8);
    r.op = p[10];
    r.ms = p[11];
    r.span = get32(p + 12);
    return r;
}

/// stage's longest frame in cells: a start, nine data bits, a parity bit
/// and two stop bits.
inline constexpr uint8_t stage_cells_most = 13;

/// stage's cell numbering: the first stop cell of a frame of `bits` data
/// bits (bits9_low and bits9_high count nine), with a parity cell or not -
/// where a pulse inside the stop bit goes.
constexpr int8_t stage_stop_cell(uint8_t bits, uint8_t parity) {
    return static_cast<int8_t>(1u + (bits >= bits9_low ? 9u : bits) + (parity != 0u ? 1u : 0u));
}
static_assert(stage_stop_cell(8, 0) == 9 && stage_stop_cell(8, 1) == 10 && stage_stop_cell(bits9_low, 0) == 10,
              "8N1's stop is the tenth cell");

/// stage's frame, cell by cell: whether cell `k` of a frame carrying `v`
/// in `data` bits (5..9) with `parity` (0 none, 1 even, 2 odd) is HIGH -
/// the start bit low, the data LSB first, the parity bit, the stop bits
/// (any `k` past the parity) high.
constexpr bool stage_cell_high(uint16_t v, uint8_t data, uint8_t parity, uint8_t k) {
    if (k == 0u) {
        return false;
    }
    if (k <= data) {
        return ((v >> (k - 1u)) & 1u) != 0u;
    }
    if (k == data + 1u && parity != 0u) {
        uint8_t ones = 0;
        for (uint8_t i = 0; i < data; ++i) {
            ones = static_cast<uint8_t>(ones + ((v >> i) & 1u));
        }
        return ((ones & 1u) != 0u) != (parity == 2u);   // even: the bit makes the count even
    }
    return true;
}
static_assert(!stage_cell_high(0x55, 8, 0, 0) && stage_cell_high(0x55, 8, 0, 1) && !stage_cell_high(0x55, 8, 0, 2) &&
                  stage_cell_high(0x55, 8, 0, 9),
              "0x55 8N1: start low, LSB first, stop high");
static_assert(!stage_cell_high(0x03, 8, 1, 9) && stage_cell_high(0x03, 8, 2, 9) && stage_cell_high(0x07, 8, 1, 9),
              "the parity cell: even makes the ones even, odd makes them odd");

/// The next value of the shared 8-bit LFSR (x^8 + x^6 + x^5 + x^4 + 1),
/// so both boards generate the same "random-ish" stream from one seed.
constexpr uint8_t prbs_next(uint8_t v) {
    const uint8_t bit = static_cast<uint8_t>(((v >> 7) ^ (v >> 5) ^ (v >> 4) ^ (v >> 3)) & 1u);
    return static_cast<uint8_t>((v << 1) | bit);
}

/// The `pattern` generator as a running stream: next() is frame 0, 1,
/// 2, ... of pattern_value() at a constant cost a frame, which is what a
/// blast that has to be gapless needs (pattern_value() walks the LFSR
/// from the seed every call).
class Stream {
public:
    constexpr Stream(uint8_t pattern, uint8_t seed)
        : pattern_(pattern), v_(pattern == pattern_prbs ? (seed ? seed : 0xACu) : seed) {}

    constexpr uint8_t next() {
        const uint8_t out = v_;
        if (pattern_ == pattern_prbs) v_ = prbs_next(v_);
        else if (pattern_ != pattern_fixed) v_ = static_cast<uint8_t>(v_ + 1);
        return out;
    }

private:
    uint8_t pattern_;
    uint8_t v_;
};

/// The value the `pattern` generator produces for frame `i`, masked to
/// the character size by the caller.
constexpr uint8_t pattern_value(uint8_t pattern, uint8_t seed, uint16_t i) {
    if (pattern == pattern_fixed) return seed;
    if (pattern == pattern_prbs) {
        uint8_t v = seed ? seed : 0xACu;
        for (uint16_t k = 0; k < i; ++k) v = prbs_next(v);
        return v;
    }
    return static_cast<uint8_t>(seed + i);
}

/// The stream and the indexed form are one sequence: checked here, at
/// compile time, for every pattern over more than one LFSR period.
constexpr bool stream_is_pattern_value() {
    for (uint8_t pattern = 0; pattern <= pattern_prbs; ++pattern) {
        Stream st(pattern, 0x5Du);
        for (uint16_t i = 0; i < 300; ++i) {
            if (st.next() != pattern_value(pattern, 0x5Du, i)) return false;
        }
    }
    return true;
}
static_assert(stream_is_pattern_value(), "Stream must produce pattern_value's sequence");

}  // namespace link
