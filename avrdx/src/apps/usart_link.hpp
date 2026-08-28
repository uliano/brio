// usart_link.hpp - the board-to-board bench protocol of the USART
// campaign: what test_avr_serial (board A, the DUT) tells usart_peer
// (board B, the instrument) over the very link both are testing.
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

/// HOW THE TWO BOARDS ARE WIRED. The campaign's wiring is the crossed
/// full-duplex pair (A.PE0-B.PE1, A.PE1-B.PE0, A.PE2-B.PE2), but a
/// single wire between the two TXD pads is a perfectly good link too -
/// it is the one-wire bus of 27.3.3.2.6, and with LBME each end hears
/// the pad. Both apps support both and FIND OUT which one is on the
/// desk (the peer alternates its command-mode configuration until a
/// frame arrives; the DUT pings in one topology, then the other), so a
/// desk that is not wired as expected reports a topology instead of
/// looking like a dead protocol.
///
/// On a shared line the discipline is: LBME on both ends, push-pull
/// transmitters, TXEN enabled only while transmitting, a PORT pull-up
/// holding the line during the turnaround, and the receiver disabled
/// while transmitting so the loop-back echo is not decoded. Nothing
/// needs a slower rate: the line is driven except between frames.
enum class Topology : uint8_t {
    full_duplex = 0,   ///< TXD -> RXD each way (the campaign wiring)
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
    onewire = 0x0E,      ///< join the shared line half-duplex until the bound
    collide = 0x0F,      ///< transmit into the DUT's frame, on purpose
    report = 0x10,       ///< ack + Report of the last action
    standby = 0x11,      ///< let go of the line entirely for `ms`

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

/// Every action command carries exactly this. What an op does not use
/// stays zero; the meaning of the shared fields per op:
///
///   echo/sink/set_link/onewire  count = frames to process, ms = deadline
///   standby                     ms = how long to stay off the wire
///   send/blast                  count = frames to generate, value/pattern;
///                               for `send`, aux16 = extra lead-in in
///                               milliseconds before the first frame
///   brk                         count = bit times to hold TXD low
///   autobaud_tx                 value = the sync character, aux8 = payload
///                               frames, aux16 = break bit times
///   bitbang                     count = frames, value = the byte, cell =
///                               which bit cell to distort (0 = start,
///                               1..8 = data, 9 = stop, -1 = none), pct =
///                               by how much, aux16 = bit times of break
///                               ahead of the frames (0 = none)
///   glitch                      count = pulses, aux16 = pulse width in CPU
///                               cycles, ms = deadline, aux8 = gap in
///                               microseconds
///   drive_rxd                   count = frames, value = the byte
///   collide                     value = the byte, aux16 = microseconds to
///                               wait after the trigger frame
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
    char label[8] = {};      ///< the peer's USERROW board label, NUL padded
    uint8_t xtal = 0;        ///< did the peer's 24 MHz crystal start?
    uint8_t sanity = 0;
    uint16_t version = 0;
};

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

inline constexpr uint8_t report_size = 12;

inline constexpr uint8_t report_timed_out = 0x01;   ///< the deadline ended it
inline constexpr uint8_t report_mismatch = 0x02;    ///< a frame was not what was expected
inline constexpr uint8_t report_cfg_failed = 0x04;  ///< the Cfg was refused by the driver
inline constexpr uint8_t report_ran = 0x08;         ///< the action really ran

/// What the peer saw while it was not in command mode.
struct Report {
    uint16_t count = 0;      ///< frames processed (received or generated)
    uint16_t sum = 0;        ///< running 16-bit sum of the data values
    uint8_t ferr = 0;
    uint8_t perr = 0;
    uint8_t ovf = 0;
    uint8_t flags = 0;
    uint16_t aux16 = 0;      ///< per-op extra (e.g. the BAUD the peer used)
    uint8_t op = 0;          ///< which action this report belongs to
    uint8_t ms = 0;          ///< how long it took, capped at 255
};

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
    return r;
}

/// The next value of the shared 8-bit LFSR (x^8 + x^6 + x^5 + x^4 + 1),
/// so both boards generate the same "random-ish" stream from one seed.
constexpr uint8_t prbs_next(uint8_t v) {
    const uint8_t bit = static_cast<uint8_t>(((v >> 7) ^ (v >> 5) ^ (v >> 4) ^ (v >> 3)) & 1u);
    return static_cast<uint8_t>((v << 1) | bit);
}

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

}  // namespace link
