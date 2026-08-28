// twi_link.hpp - the board-to-board bench protocol of the TWI campaign:
// what test_avr_twi (board A, the DUT and the bus HOST) tells twi_peer
// (board B, the instrument and a bus CLIENT) over the very bus both are
// testing.
//
// APP-LEVEL BENCH TOOLING, not framework. It sits next to the two apps
// that share it and is included by its plain name; nothing in
// brio knows it exists, and nothing here touches a register -
// it is pure encoding, so both apps compile it and so could the host.
//
// THE FRAME
//
//   magic (0xC3) | op | len | payload[len] | checksum
//
// The checksum is the 8-bit sum of magic, op, len and every payload
// byte, XORed with 0xA5 (so an all-zero run does not check out).
//
// THE COMMAND CHANNEL - why I2C needs no dark listener
//
// The SPI campaign's peer had to stay mute because SPI has no
// addressing: every byte the DUT clocked reached the client, and a
// listener that answered would have fought the DUT pin for pin. I2C
// addresses, so the command channel is simply a CLIENT ADDRESS
// (`command_addr` below) that no test of the DUT's single-board half
// ever sends, general-calls or mask-matches. In command mode the peer's
// client is maximally deaf: that one address exactly, General Call OFF,
// no mask, no second address, PMEN off, Smart mode off, and even the
// Stop interrupt off - so the ONLY thing that raises a flag on board B
// while the DUT runs `z` is an address packet naming `command_addr`.
// That is the whole coexistence argument, and `test_avr_twi z` scoring
// its full 175 with this firmware attached is its proof.
//
// REQUEST / RESPONSE. One command is two bus tenures:
//
//   1. a host WRITE to `command_addr` carrying one command frame. The
//      peer resets its decoder at the address packet, so one write
//      transaction is exactly one frame and a truncated one can never
//      eat the next;
//   2. a host READ of `response_bytes` from `command_addr`. The peer
//      serves the response frame it prepared, padding the tail with
//      zeros (the decoder ignores them), and takes the host's closing
//      NACK as "the answer has been collected".
//
// ACK BEFORE ACT. The peer answers every frame with an ack (or a nak,
// for a checksum that did not match) and only then runs what the frame
// asked for: the action starts when the ACK READ completes, so the DUT
// knows the peer is armed before it puts a single measured edge on the
// wire. The two queries - ident and report - prepare their data frame
// as the action, so the DUT collects them with a second read.
//
// THE RECOVERY GUARANTEE. Every action carries a frame/byte count and a
// millisecond deadline; the peer restores the deaf command-mode client
// BY ITSELF at the bound, whatever happened on the wire. A DUT test that
// hangs therefore costs one deadline, not a power cycle.

#pragma once

#include <stdint.h>

namespace twilink {

inline constexpr uint8_t magic = 0xC3;
inline constexpr uint8_t max_payload = 32;

/// The peer's command-mode client address. Chosen so that NO test of
/// test_avr_twi's single-board half (a..j) addresses it, mask-matches it
/// or general-calls it: that half uses 0x42, 0x43, 0x00, 0x40..0x44,
/// 0x3F, 0x55, 0x54, 0x08, 0x33, 0x77, 0x7E and 0x20.
inline constexpr uint8_t command_addr = 0x6B;

/// The DUT's own client address during the two-board half - what the
/// peer's HOST addresses when both boards drive the bus at once.
inline constexpr uint8_t dut_addr = 0x2C;

/// The address BOTH boards' clients take for the collision case (S4):
/// two clients, one address, one host reading from it.
inline constexpr uint8_t shared_addr = 0x39;

/// A low address the peer's client can be told to take, so that the two
/// hosts of the arbitration test can be made to lose in EITHER
/// direction: the host transmitting the smaller address byte wins the
/// wired-AND, so swapping which board aims at the lower address swaps
/// the winner. 0x11 < `dut_addr` < `command_addr`.
inline constexpr uint8_t low_addr = 0x11;

/// Where a "deaf" peer parks its client: an address nobody on this bus
/// sends, so an addressed write meets no ACK at all.
inline constexpr uint8_t deaf_addr = 0x7A;

/// How many bytes the DUT reads for one response frame. The longest
/// answer is a Report: magic + op + len + 16 + checksum = 20.
inline constexpr uint8_t response_bytes = 20;

/// Milliseconds the DUT waits after collecting an ack before it puts
/// measured traffic on the wire: the peer enters its action the moment
/// that read completes, and this is the margin over its re-init.
inline constexpr uint8_t arm_ms = 3;

/// The instrument's repertoire.
enum class Op : uint8_t {
    ping = 0x01,        ///< rendezvous: ack only
    ident = 0x02,       ///< ack + Ident (label, crystal flag, sanity byte)
    report = 0x03,      ///< ack + Report of the last action

    serve = 0x10,       ///< be a client: stretch, NACK, general call, or go deaf
    arb = 0x11,         ///< combined: a client AND a host racing the DUT for the bus
    coll = 0x12,        ///< a client sharing ONE address with the DUT's own (S4)
    hold_sda = 0x13,    ///< a stuck client: SDA held low from PORT, released by SCL
    quiet = 0x14,       ///< the TWI released entirely: board B off the wire

    ack = 0x40,
    nak = 0x41,
    ident_data = 0x42,
    report_data = 0x43,
};

constexpr uint8_t byte_of(Op op) { return static_cast<uint8_t>(op); }

/// Is this an op the instrument knows?
constexpr bool is_command(Op op) {
    switch (op) {
        case Op::ping:
        case Op::ident:
        case Op::report:
        case Op::serve:
        case Op::arb:
        case Op::coll:
        case Op::hold_sda:
        case Op::quiet: return true;
        default: return false;
    }
}

/// Does this op carry a Params payload?
constexpr bool is_action(Op op) {
    switch (op) {
        case Op::serve:
        case Op::arb:
        case Op::coll:
        case Op::hold_sda:
        case Op::quiet: return true;
        default: return false;
    }
}

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
/// allocates: feed() is called from whatever loop owns the wire.
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

// ---- the uniform action payload ----------------------------------------------

/// `Params::flags`.
///
/// `flag_general_call` sets SADDR[0] on the peer's client, so it answers
/// address 0x00 as well as its own - the multi-client General Call case.
/// `flag_deaf` parks the client at `deaf_addr` instead of the address
/// the command names, which is how an ADDRESS NACK is produced against a
/// board that is otherwise alive and listening.
/// `flag_stop_interrupt` turns PIEN on for the duration of the action,
/// so the peer counts the Stop conditions it saw (in command mode PIEN
/// is off: see the header comment on deafness).
/// `flag_host` brings the host half up as well - COMBINED mode, the
/// configuration the arbitration case needs on both boards at once.
inline constexpr uint8_t flag_general_call = 0x01;
inline constexpr uint8_t flag_deaf = 0x02;
inline constexpr uint8_t flag_stop_interrupt = 0x04;
inline constexpr uint8_t flag_host = 0x08;

/// `Params::pattern` for the bytes the client serves to a reading host.
inline constexpr uint8_t pattern_counting = 0;   ///< seed, seed+1, seed+2, ...
inline constexpr uint8_t pattern_fixed = 1;      ///< seed every time

/// Every action command carries exactly this. What an op does not use
/// stays zero; the meaning of the shared fields per op:
///
///   serve      count = data bytes after which the action may end early,
///              ms = deadline, addr = the client address to take,
///              seed/pattern = what it answers a read with, hold_us =
///              the CLOCK STRETCH it adds to every data byte, nack_at =
///              which received byte it NACKs, flags as above
///   arb        addr = the client address to take, target = the address
///              its HOST writes to, count = bytes that host writes,
///              seed/pattern = those bytes, ms = deadline, aux16 =
///              microseconds to wait after the bus goes Busy before
///              arming the held START
///   coll       addr = the address SHARED with the DUT's own client,
///              seed = the byte it serves (fixed), ms = deadline
///   hold_sda   aux16 = microseconds to hold SDA low at most, aux8 =
///              how many SCL falling edges release it (0 = only the
///              deadline does), ms = deadline
///   quiet      ms = how long board B stays off the wire entirely
struct Params {
    uint16_t count = 0;
    uint16_t ms = 200;
    uint8_t addr = 0;        ///< 0 = keep the command address
    uint8_t seed = 0;
    uint16_t hold_us = 0;    ///< per-data-byte clock stretch
    uint8_t nack_at = 0;     ///< NACK the n-th received data byte, 1-based
    uint8_t flags = 0;
    uint8_t aux8 = 0;
    uint8_t target = 0;      ///< the address this peer's HOST addresses
    uint16_t aux16 = 0;
    uint8_t pattern = pattern_counting;
    uint8_t spare = 0;
};

inline constexpr uint8_t params_size = 16;

constexpr void put_params(uint8_t* p, const Params& a) {
    put16(p, a.count);
    put16(p + 2, a.ms);
    p[4] = a.addr;
    p[5] = a.seed;
    put16(p + 6, a.hold_us);
    p[8] = a.nack_at;
    p[9] = a.flags;
    p[10] = a.aux8;
    p[11] = a.target;
    put16(p + 12, a.aux16);
    p[14] = a.pattern;
    p[15] = a.spare;
}

constexpr Params get_params(const uint8_t* p) {
    Params a{};
    a.count = get16(p);
    a.ms = get16(p + 2);
    a.addr = p[4];
    a.seed = p[5];
    a.hold_us = get16(p + 6);
    a.nack_at = p[8];
    a.flags = p[9];
    a.aux8 = p[10];
    a.target = p[11];
    a.aux16 = get16(p + 12);
    a.pattern = p[14];
    a.spare = p[15];
    return a;
}

/// The value the `pattern` generator produces for byte `i`.
constexpr uint8_t pattern_value(uint8_t pattern, uint8_t seed, uint16_t i) {
    if (pattern == pattern_fixed) return seed;
    return static_cast<uint8_t>(seed + i);
}

// ---- the answers ---------------------------------------------------------------

inline constexpr uint8_t ident_size = 12;
inline constexpr uint8_t ident_sanity = 0x7B;   ///< "this firmware is twi_peer"

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

inline constexpr uint8_t report_size = 16;

inline constexpr uint8_t report_ran = 0x01;         ///< the action really ran
inline constexpr uint8_t report_timed_out = 0x02;   ///< the deadline ended it
inline constexpr uint8_t report_coll = 0x04;        ///< SSTATUS.COLL was seen (S4)
inline constexpr uint8_t report_buserr = 0x08;      ///< a bus error was seen
inline constexpr uint8_t report_arblost = 0x10;     ///< this board's HOST lost the bus
inline constexpr uint8_t report_host_ran = 0x20;    ///< its host really put a START out
inline constexpr uint8_t report_host_ok = 0x40;     ///< and completed its write
inline constexpr uint8_t report_nacked = 0x80;      ///< it NACKed a byte on purpose

/// What the peer saw while it was not in command mode.
struct Report {
    uint16_t count = 0;      ///< data bytes moved (received plus served)
    uint16_t sum = 0;        ///< 8-bit-wide running sum of the RECEIVED bytes
    uint16_t addr_hits = 0;  ///< address packets its client matched
    uint8_t flags = 0;
    uint8_t last_addr = 0;   ///< the address of the last match (SDATA)
    uint8_t first = 0;       ///< the first byte it received
    uint8_t sstatus = 0;     ///< SSTATUS at the end of the action
    /// The host half's MSTATUS at the end - except for `serve`, which
    /// has no host and puts here the SSTATUS sampled AT the instant the
    /// reading host's closing NACK arrived (RXACK is a live bit, and a
    /// copy taken when the action ends carries whatever the last
    /// transaction happened to leave behind).
    uint8_t mstatus = 0;
    uint8_t aux0 = 0;
    uint8_t aux1 = 0;
    uint8_t stops = 0;       ///< Stop conditions seen (needs flag_stop_interrupt)
    uint8_t op = 0;          ///< which action this report belongs to
    uint8_t ms = 0;          ///< how long it took, capped at 255
};

constexpr void put_report(uint8_t* p, const Report& r) {
    put16(p, r.count);
    put16(p + 2, r.sum);
    put16(p + 4, r.addr_hits);
    p[6] = r.flags;
    p[7] = r.last_addr;
    p[8] = r.first;
    p[9] = r.sstatus;
    p[10] = r.mstatus;
    p[11] = r.aux0;
    p[12] = r.aux1;
    p[13] = r.stops;
    p[14] = r.op;
    p[15] = r.ms;
}

constexpr Report get_report(const uint8_t* p) {
    Report r{};
    r.count = get16(p);
    r.sum = get16(p + 2);
    r.addr_hits = get16(p + 4);
    r.flags = p[6];
    r.last_addr = p[7];
    r.first = p[8];
    r.sstatus = p[9];
    r.mstatus = p[10];
    r.aux0 = p[11];
    r.aux1 = p[12];
    r.stops = p[13];
    r.op = p[14];
    r.ms = p[15];
    return r;
}

}  // namespace twilink
