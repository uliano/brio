// sleep_link.hpp - the board-to-board bench protocol of the SLEEP
// campaign: what test_avr_sleep (board A, the DUT) tells sleep_peer
// (board B, the instrument) while A is about to stop its clocks.
//
// APP-LEVEL BENCH TOOLING, not framework. It sits next to the two apps
// that share it and is included by its plain name; nothing in
// brio knows it exists, and nothing here touches a register -
// it is pure encoding.
//
// WHY A SECOND BOARD AT ALL. Every latency this campaign measures is
// the time between an event OUTSIDE the sleeping chip and that chip's
// first instruction afterwards. A sleeping device cannot time its own
// wake-up: the counter that would do it is exactly the one the mode
// stops (test_avr_sleep f says so in as many words). So the ruler moves
// off-chip - board B drives the stimulus, board B starts the stopwatch
// on the same edge, and board B captures the DUT's echo IN HARDWARE.
//
// THE FRAME
//
//   magic (0x5E) | op | len | payload[len] | checksum
//
// The checksum is the 8-bit sum of magic, op, len and every payload
// byte, XORed with 0xA5 (so an all-zero run does not check out). The
// peer answers EVERY command with one frame before acting:
//
//   ack  payload = {op, checksum}      the command was understood
//   nak  payload = {op, checksum}      the checksum did not match
//
// and the query commands (ident, report, gaps) follow the ack with a
// second frame carrying the data. Nothing else is ever unsolicited:
// after a bounded action the peer goes back to command mode and WAITS
// to be asked for its numbers.
//
// THE WIRES (fixed - this campaign does NO topology discovery)
//
//   PE0   the command channel: one wire between the two USART4 TXD
//         pads, LBME at both ends, 8N1 at command_baud. It is also the
//         line the SFD test wakes on.
//   PE1   spare.
//   PE2   B drives, A senses: the STIMULUS. B's rising edge starts its
//         stopwatch and is what has to wake the sleeping DUT.
//   PE3   A drives, B senses: the ECHO. A's wake-up ISR raises it as
//         its very first instruction; on B that edge is an event that
//         CAPTURES the 32-bit stopwatch - no polling anywhere in the
//         measurement path.
//
// THE RECOVERY GUARANTEE
//
// Command mode is async 8N1 at `command_baud` on both boards. Every
// command carries a bound - a count and a millisecond deadline - after
// which the peer restores command mode BY ITSELF. Whatever
// desynchronizes, both ends are back in command mode after the bound
// plus the quiet interval, and the DUT's client retries its command
// before failing a test.

#pragma once

#include <stdint.h>

namespace slink {

inline constexpr uint8_t magic = 0x5E;
inline constexpr uint8_t max_payload = 48;

/// Command mode: what both boards fall back to, always.
inline constexpr uint32_t command_baud = 115'200;

/// The peer reconfigures right after its ack has left the line; the DUT
/// waits this long before doing the same, so the peer is always ready
/// first.
inline constexpr uint16_t settle_ms = 3;

/// How many stopwatch results the peer keeps from one action.
inline constexpr uint8_t max_shots = 12;
/// How many of them fit in one `gaps_data` frame.
inline constexpr uint8_t gaps_per_frame = 8;

/// A shot whose echo never arrived.
inline constexpr uint32_t no_capture = 0xFFFFFFFFu;

/// The instrument's repertoire.
enum class Op : uint8_t {
    ping = 0x01,       ///< rendezvous: ack only
    ident = 0x02,      ///< ack + Ident (label, clock, sanity byte)
    report = 0x03,     ///< ack + Report of the last action
    gaps = 0x04,       ///< ack + Gaps page starting at payload[0]

    pulse = 0x10,      ///< N stimulus edges on PE2, each timed to A's echo
    capture = 0x11,    ///< arm the stopwatch and wait for ONE echo (wire sanity)
    sfd_byte = 0x12,   ///< one byte on the PE0 one-wire at a foreign baud
    twi_write = 0x13,  ///< one host write tenure on the desk I2C bus, timed

    ack = 0x40,
    nak = 0x41,
    ident_data = 0x42,
    report_data = 0x43,
    gaps_data = 0x44,
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

// ---- the uniform action payload ----------------------------------------------

/// Every action command carries exactly this. What an op does not use
/// stays zero; the meaning of the shared fields per op:
///
///   pulse      count = stimulus edges, delay_ms before the first,
///              period_ms between them, deadline_ms per shot,
///              hold_us = the longest the stimulus is held high
///   capture    delay_ms before arming, deadline_ms to wait for one echo
///   sfd_byte   delay_ms before the byte, rate = its baud, value = it
///   twi_write  delay_ms before the tenure, count = data bytes,
///              addr = the 7-bit client address, value = the first
///              byte (the rest count up), rate = the SCL hertz asked
struct Params {
    uint16_t count = 1;
    uint16_t delay_ms = 100;
    uint16_t period_ms = 100;
    uint16_t deadline_ms = 200;
    uint32_t rate = command_baud;
    uint8_t value = 0;
    uint8_t addr = 0;
    uint16_t hold_us = 2000;
};

inline constexpr uint8_t params_size = 16;

constexpr void put_params(uint8_t* p, const Params& a) {
    put16(p, a.count);
    put16(p + 2, a.delay_ms);
    put16(p + 4, a.period_ms);
    put16(p + 6, a.deadline_ms);
    put32(p + 8, a.rate);
    p[12] = a.value;
    p[13] = a.addr;
    put16(p + 14, a.hold_us);
}

constexpr Params get_params(const uint8_t* p) {
    Params a{};
    a.count = get16(p);
    a.delay_ms = get16(p + 2);
    a.period_ms = get16(p + 4);
    a.deadline_ms = get16(p + 6);
    a.rate = get32(p + 8);
    a.value = p[12];
    a.addr = p[13];
    a.hold_us = get16(p + 14);
    return a;
}

// ---- the answers ---------------------------------------------------------------

inline constexpr uint8_t ident_size = 12;
inline constexpr uint8_t ident_sanity = 0x7C;   ///< "this firmware is sleep_peer"

inline constexpr uint8_t clock_oschf = 0;
inline constexpr uint8_t clock_crystal = 1;

struct Ident {
    char label[8] = {};      ///< the peer's USERROW board label, NUL padded
    uint8_t clock = 0;       ///< clock_oschf / clock_crystal: what CLK_PER runs on
    uint8_t sanity = 0;
    uint16_t version = 0;
};

constexpr void put_ident(uint8_t* p, const Ident& d) {
    for (uint8_t i = 0; i < 8; ++i) p[i] = static_cast<uint8_t>(d.label[i]);
    p[8] = d.clock;
    p[9] = d.sanity;
    put16(p + 10, d.version);
}

constexpr Ident get_ident(const uint8_t* p) {
    Ident d{};
    for (uint8_t i = 0; i < 8; ++i) d.label[i] = static_cast<char>(p[i]);
    d.clock = p[8];
    d.sanity = p[9];
    d.version = get16(p + 10);
    return d;
}

inline constexpr uint8_t report_size = 16;

inline constexpr uint8_t report_ran = 0x01;        ///< the action really ran
inline constexpr uint8_t report_timed_out = 0x02;  ///< a deadline ended it
inline constexpr uint8_t report_failed = 0x04;     ///< the peer could not set it up
inline constexpr uint8_t report_acked = 0x08;      ///< every TWI byte was acknowledged

/// What the peer measured while the DUT was not running.
struct Report {
    uint8_t op = 0;          ///< which action this report belongs to
    uint8_t flags = 0;
    uint16_t count = 0;      ///< shots fired / bytes written
    uint16_t hits = 0;       ///< echoes captured
    uint16_t misses = 0;     ///< shots whose echo never came
    uint32_t ticks = 0;      ///< the last (or only) measurement, CLK_PER ticks
    uint16_t ms = 0;         ///< how long the whole action took
    uint8_t status = 0;      ///< per-op: the I2C outcome code, ...
    uint8_t aux = 0;
};

constexpr void put_report(uint8_t* p, const Report& r) {
    p[0] = r.op;
    p[1] = r.flags;
    put16(p + 2, r.count);
    put16(p + 4, r.hits);
    put16(p + 6, r.misses);
    put32(p + 8, r.ticks);
    put16(p + 12, r.ms);
    p[14] = r.status;
    p[15] = r.aux;
}

constexpr Report get_report(const uint8_t* p) {
    Report r{};
    r.op = p[0];
    r.flags = p[1];
    r.count = get16(p + 2);
    r.hits = get16(p + 4);
    r.misses = get16(p + 6);
    r.ticks = get32(p + 8);
    r.ms = get16(p + 12);
    r.status = p[14];
    r.aux = p[15];
    return r;
}

/// `gaps_data`: p[0] = the index this page starts at, p[1] = how many
/// 32-bit values follow. A value of `no_capture` is a shot the DUT
/// never answered.
inline constexpr uint8_t gaps_header = 2;

}  // namespace slink
