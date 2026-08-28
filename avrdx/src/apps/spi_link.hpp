// spi_link.hpp - the board-to-board bench protocol of the SPI campaign:
// what test_avr_spi (board A, the DUT and the bus HOST) tells spi_peer
// (board B, the instrument and the bus CLIENT) over the very bus both
// are testing.
//
// APP-LEVEL BENCH TOOLING, not framework. It sits next to the two apps
// that share it and is included by its plain name; nothing in
// brio knows it exists, and nothing here touches a register -
// it is pure encoding, so both apps can compile it and the host could
// too.
//
// THE FRAME
//
//   magic (0xB8) | op | len | payload[len] | checksum
//
// The checksum is the 8-bit sum of magic, op, len and every payload
// byte, XORed with 0xA5 (so an all-zero run does not check out).
//
// THE COMMAND CHANNEL
//
// SPI mode 0, MSb first, CLK_PER/`command_division` (750 kHz at
// 24 MHz - still inside a 24 MHz client's CLK_PER/6 ceiling when the
// DUT rebases to 12 MHz). The host owns SS as a plain GPIO chip select
// (it runs with CTRLB.SSD = 1, so the peripheral leaves the pin alone)
// and drives it low for exactly one frame; between bytes it waits
// `gap_us` so the client's polled loop can drain the byte it just
// received and load the next answer.
//
// REQUEST / RESPONSE. SPI is host-clocked, so the client can never
// speak first. One exchange is therefore three phases:
//
//   1. the host sends a command frame in ONE chip-select window;
//   2. it releases SS and waits `settle_ms` while the client decodes
//      the frame and PREPARES its answer (a client loads what the next
//      clock will shift out; nothing leaves the board until the host
//      clocks it);
//   3. it opens a second window and clocks `answer_bytes` dummies at
//      the same pace, feeding its own decoder until the answer frame
//      appears or the count runs out.
//
// Every command is answered by an ack (or a nak, for a frame whose
// checksum did not match), and the two queries - ident and report -
// follow their ack with a second data frame served in a third window
// by the same mechanics.
//
// THE DARK LISTENER
//
// The four wires this protocol runs on are also the pins the DUT's
// SINGLE-board half measures, and that half drives SS low at times
// (its chip-select tests) while it is also driving MISO from its own
// PORT. A client that answered then would fight it pin for pin. So the
// client is, by default, DARK: it is configured with `drive_miso =
// false` and hears MOSI, SCK and SS without ever driving the answer
// line. It re-configures itself with MISO driven ONLY after a frame
// with a valid checksum has decoded, serves that one answer window and
// goes dark again.
//
// Two more gates keep the single-board half's traffic from ever waking
// the answer line, because that traffic IS decoded - the client shifts
// every byte the DUT clocks, in whatever mode the DUT happens to be
// testing, and a long enough run of it will eventually put `magic` on
// the wire:
//
//   - an op the protocol does not know (`is_command` is false) is
//     dropped without a word, valid checksum or not;
//   - a frame whose checksum FAILS is nak'ed only while the peer is
//     ENGAGED - only within `engage_ms` of the last frame that did
//     check out. A board that has never been addressed answers
//     nothing, whatever noise decodes.
//
// The client also keeps a PULL-UP on its own SS pin at all times, so
// the shared select wire never floats low while the host leaves its end
// an input.
//
// THE RECOVERY GUARANTEE
//
// Every action op carries a frame count and a millisecond deadline; the
// peer restores the dark command-mode client BY ITSELF at the bound,
// whatever happened on the wire. The host retries a command three times
// before failing a test.
//
// THE EXCHANGE, AND THE DUMMY BYTE
//
// The workhorse action: both boards generate a deterministic byte
// stream from a seed (the shared LFSR below), the host sends P_A(i) and
// the client answers P_B(i). The client preloads P_B(0) while SS is
// still high and writes P_B(i+1) as soon as byte i has arrived.
//
// What the host reads back depends on the client's buffering regime:
//
//   normal, and buffer mode WITH BUFWR   rx[i] = P_B(i)
//   buffer mode WITHOUT BUFWR            rx[0] = a DUMMY (the shift
//                                        register's leftover: the first
//                                        write went into the transmit
//                                        buffer, not the shifter), then
//                                        rx[i] = P_B(i-1)
//
// That asymmetry is one of the things under test, so the host measures
// the dummy instead of assuming it.

#pragma once

#include <stdint.h>

namespace spilink {

inline constexpr uint8_t magic = 0xB8;
inline constexpr uint8_t max_payload = 32;

/// The command channel's SCK: CLK_PER divided by this.
inline constexpr uint8_t command_division = 32;

/// Microseconds the host waits between two bytes of a command or answer
/// window, so the client's polled loop can turn the byte around.
inline constexpr uint8_t gap_us = 20;

/// Milliseconds between one chip-select window and the next: the client
/// reconfigures and preloads inside this.
inline constexpr uint8_t settle_ms = 5;

/// How many dummy bytes the host clocks looking for one answer frame.
inline constexpr uint16_t answer_bytes = 48;

/// The client's bound on one answer window (it goes dark after this
/// whatever the host does).
inline constexpr uint16_t serve_ms = 300;

/// How long a frame that checked out keeps the peer ENGAGED, and so
/// willing to nak a frame that does not. Long enough to cover a whole
/// command/answer/action round trip, short enough that the peer is mute
/// again well before the DUT's single-board half starts.
inline constexpr uint16_t engage_ms = 4000;

/// The instrument's repertoire.
enum class Op : uint8_t {
    ping = 0x01,        ///< rendezvous: ack only
    ident = 0x02,       ///< ack + Ident (label, crystal flag, sanity byte)
    report = 0x03,      ///< ack + Report of the last action
    exchange = 0x10,    ///< the workhorse: `count` bytes both ways, per Cfg
    sink_slow = 0x11,   ///< receive without draining: the loss semantics
    ss_pulse = 0x12,    ///< drive the shared SS wire low from PORT (a real demotion)
    mspi = 0x13,        ///< self-selected client against the DUT's USART Host SPI

    ack = 0x40,
    nak = 0x41,
    ident_data = 0x42,
    report_data = 0x43,
};

constexpr uint8_t byte_of(Op op) { return static_cast<uint8_t>(op); }

/// Is this an op the instrument knows? Everything else is dropped in
/// silence, checksum or no checksum - the answer line must never wake
/// up for noise the single-board half happens to put on MOSI.
constexpr bool is_command(Op op) {
    switch (op) {
        case Op::ping:
        case Op::ident:
        case Op::report:
        case Op::exchange:
        case Op::sink_slow:
        case Op::ss_pulse:
        case Op::mspi: return true;
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
/// allocates: feed() is called from whatever poll loop owns the SPI.
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

// ---- the client's configuration ----------------------------------------------

/// `Cfg::regime`: which buffering the client runs in (CTRLB.BUFEN and
/// CTRLB.BUFWR). The three are not cosmetic - they decide whether the
/// FIRST byte the host clocks out is the client's answer or the shift
/// register's leftover (see the header comment).
inline constexpr uint8_t regime_normal = 0;       ///< BUFEN = 0
inline constexpr uint8_t regime_buffer = 1;       ///< BUFEN = 1, BUFWR = 0: a dummy leads
inline constexpr uint8_t regime_buffer_wait = 2;  ///< BUFEN = 1, BUFWR = 1: no dummy

/// What the client's link should become. `apply == 0` leaves it alone.
struct Cfg {
    uint8_t apply = 1;
    uint8_t mode = 0;      ///< the four SPI transfer modes, 0..3
    uint8_t dord = 0;      ///< 1 = LSb first
    uint8_t regime = regime_normal;
};

inline constexpr uint8_t cfg_size = 4;

constexpr void put_cfg(uint8_t* p, const Cfg& c) {
    p[0] = c.apply;
    p[1] = c.mode;
    p[2] = c.dord;
    p[3] = c.regime;
}

constexpr Cfg get_cfg(const uint8_t* p) {
    Cfg c{};
    c.apply = p[0];
    c.mode = p[1];
    c.dord = p[2];
    c.regime = p[3];
    return c;
}

// ---- the uniform action payload ----------------------------------------------

/// `Params::pattern` for both directions of an exchange.
inline constexpr uint8_t pattern_counting = 0;   ///< seed, seed+1, seed+2, ...
inline constexpr uint8_t pattern_fixed = 1;      ///< seed every time
inline constexpr uint8_t pattern_prbs = 2;       ///< an 8-bit LFSR seeded with seed

/// `Params::flags`.
///
/// `flag_wrcol` walks the client across the WRITE-COLLISION BOUNDARY
/// from both sides, in one burst. A write to DATA is a collision only
/// while the shifter is RUNNING; in the gap the host leaves between two
/// bytes it is an ordinary write that replaces the answer already
/// loaded. So the client writes `wrcol_marker` over its own answer
/// twice:
///
///   after byte `wrcol_gap_at`   still in the gap - ACCEPTED, and the
///                               marker goes out in place of the answer
///   after byte `wrcol_hold_at`  after watching SCK leave its idle level,
///                               so the transfer has demonstrably started
///                               - IGNORED, the answer goes out intact,
///                               and WRCOL comes up
///
/// The two positions must be read off the wire, so this flag is for
/// CONSTANT streams (`pattern_fixed`) only: with a marker replacing one
/// answer there is no sensible way to keep a generated stream aligned.
/// NORMAL mode only, too - in buffer mode that bit of INTFLAGS is TXCIF.
///
/// `flag_expect_reversed` tells the client that the host is clocking the
/// opposite bit order, so what arrives is the bit-reverse of P_A(i) -
/// that is what makes a DORD mismatch an EXACT two-way check instead of
/// a shrug.
/// `flag_skip_write` asks the client to MISS one load: after byte
/// `skip_at` it writes nothing at all, so the next transfer finds the
/// shift register still holding the byte that just came in. What the
/// host reads in that one position is the silicon's answer to "what does
/// a client that was too slow send?", and the stream resumes one place
/// late afterwards.
/// `flag_feed_tx` is for `sink_slow` in buffer mode: the client keeps
/// its TRANSMITTER fed while still never reading a byte. 28.5.5 makes
/// that the deciding condition for BUFOVF - "if there is no transmit
/// data, the Buffer Overflow will not be set before the start of a new
/// serial transfer" - so the same flood with and without this flag is
/// the experiment that isolates the clause.
inline constexpr uint8_t flag_wrcol = 0x01;
inline constexpr uint8_t flag_expect_reversed = 0x02;
inline constexpr uint8_t flag_skip_write = 0x04;
inline constexpr uint8_t flag_feed_tx = 0x08;

/// Which received byte `flag_skip_write` refuses to answer.
inline constexpr uint8_t skip_at = 3;

/// The two sides of the write-collision boundary, and the byte the
/// client writes over its own answer at each of them.
inline constexpr uint8_t wrcol_gap_at = 1;
inline constexpr uint8_t wrcol_hold_at = 4;
inline constexpr uint8_t wrcol_marker = 0x3C;

/// Every action command carries exactly this. What an op does not use
/// stays zero; the meaning of the shared fields per op:
///
///   exchange   count = bytes, ms = deadline, seed_a/seed_b/pattern =
///              the two streams, cfg = the client's configuration,
///              flags as above
///   sink_slow  count = bytes the host will send with NO gap, ms =
///              deadline, cfg = the regime whose loss is being measured
///   ss_pulse   aux8 = milliseconds to wait after the ack window before
///              taking the wire, aux16 = microseconds to hold SS low
///   mspi       count/ms/seeds/pattern as for exchange; cfg.mode mirrors
///              the host's UCPHA (bit 0) and the SCK inversion its
///              PORT.INVEN puts on XCK (bit 1), cfg.dord its UDORD;
///              aux8 = the LEAD-IN in milliseconds. The USART's Host SPI
///              mode has no client select, so the client self-selects
///              with INVEN on its own pulled-up SS pin and there is no
///              chip-select edge to frame the burst: both ends instead
///              wait out the same lead-in, at whose end the client
///              flushes whatever the host's pin handover put on the
///              wire and re-preloads its first answer.
struct Params {
    Cfg cfg{};
    uint16_t count = 0;
    uint16_t ms = 200;
    uint8_t seed_a = 0;
    uint8_t seed_b = 0;
    uint8_t pattern = pattern_prbs;
    uint8_t flags = 0;
    uint16_t aux16 = 0;
    uint8_t aux8 = 0;
    uint8_t spare = 0;
};

inline constexpr uint8_t params_size = 16;

constexpr void put_params(uint8_t* p, const Params& a) {
    put_cfg(p, a.cfg);
    put16(p + 4, a.count);
    put16(p + 6, a.ms);
    p[8] = a.seed_a;
    p[9] = a.seed_b;
    p[10] = a.pattern;
    p[11] = a.flags;
    put16(p + 12, a.aux16);
    p[14] = a.aux8;
    p[15] = a.spare;
}

constexpr Params get_params(const uint8_t* p) {
    Params a{};
    a.cfg = get_cfg(p);
    a.count = get16(p + 4);
    a.ms = get16(p + 6);
    a.seed_a = p[8];
    a.seed_b = p[9];
    a.pattern = p[10];
    a.flags = p[11];
    a.aux16 = get16(p + 12);
    a.aux8 = p[14];
    a.spare = p[15];
    return a;
}

// ---- the answers ---------------------------------------------------------------

inline constexpr uint8_t ident_size = 12;
inline constexpr uint8_t ident_sanity = 0x5C;   ///< "this firmware is spi_peer"

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
inline constexpr uint8_t report_cfg_failed = 0x04;  ///< the Cfg was refused by the driver
inline constexpr uint8_t report_wrcol = 0x08;       ///< WRCOL was seen (normal mode)
inline constexpr uint8_t report_bufovf = 0x10;      ///< BUFOVF was seen (buffer mode)

/// What the client saw while it was not in command mode. The four `aux`
/// bytes are per-op raw evidence:
///
///   exchange   aux0 = INTFLAGS at the end of the burst
///   mspi       aux0 = INTFLAGS at the end of the burst
///   sink_slow  count = bytes still RETAINED when the burst was over,
///              sum = the OR of every INTFLAGS sampled DURING it (a flag
///              that comes and goes is invisible to a look taken after),
///              got = INTFLAGS before the drain, exp = INTFLAGS after,
///              aux0..aux3 = the first four reads of DATA
///   ss_pulse   aux0 = 1 when the wire was really taken
struct Report {
    uint16_t count = 0;      ///< bytes the client received (see above for sink_slow)
    uint16_t sum = 0;        ///< running 8-bit-wide sum of them
    uint16_t mism = 0;       ///< how many differed from what was expected
    uint8_t flags = 0;
    uint8_t idx = 0;         ///< index of the first mismatch
    uint8_t got = 0;         ///< the byte that was there
    uint8_t exp = 0;         ///< the byte that should have been
    uint8_t aux0 = 0;
    uint8_t aux1 = 0;
    uint8_t aux2 = 0;
    uint8_t aux3 = 0;
    uint8_t op = 0;          ///< which action this report belongs to
    uint8_t ms = 0;          ///< how long it took, capped at 255
};

constexpr void put_report(uint8_t* p, const Report& r) {
    put16(p, r.count);
    put16(p + 2, r.sum);
    put16(p + 4, r.mism);
    p[6] = r.flags;
    p[7] = r.idx;
    p[8] = r.got;
    p[9] = r.exp;
    p[10] = r.aux0;
    p[11] = r.aux1;
    p[12] = r.aux2;
    p[13] = r.aux3;
    p[14] = r.op;
    p[15] = r.ms;
}

constexpr Report get_report(const uint8_t* p) {
    Report r{};
    r.count = get16(p);
    r.sum = get16(p + 2);
    r.mism = get16(p + 4);
    r.flags = p[6];
    r.idx = p[7];
    r.got = p[8];
    r.exp = p[9];
    r.aux0 = p[10];
    r.aux1 = p[11];
    r.aux2 = p[12];
    r.aux3 = p[13];
    r.op = p[14];
    r.ms = p[15];
    return r;
}

// ---- the shared byte streams ----------------------------------------------------

/// The next value of the shared 8-bit LFSR (x^8 + x^6 + x^5 + x^4 + 1),
/// so both boards generate the same "random-ish" stream from one seed.
constexpr uint8_t prbs_next(uint8_t v) {
    const uint8_t bit = static_cast<uint8_t>(((v >> 7) ^ (v >> 5) ^ (v >> 4) ^ (v >> 3)) & 1u);
    return static_cast<uint8_t>((v << 1) | bit);
}

/// The value the `pattern` generator produces for byte `i`.
constexpr uint8_t pattern_value(uint8_t pattern, uint8_t seed, uint16_t i) {
    if (pattern == pattern_fixed) return seed;
    if (pattern == pattern_prbs) {
        uint8_t v = seed ? seed : 0xACu;
        for (uint16_t k = 0; k < i; ++k) v = prbs_next(v);
        return v;
    }
    return static_cast<uint8_t>(seed + i);
}

/// The same stream as running STATE instead of a formula. `pattern_value`
/// replays the LFSR from the seed at every call, which is O(i) - and a
/// client has to read a byte, check it and load the next answer INSIDE
/// the host's inter-byte gap. Replaying from the seed costs more with
/// every byte and eventually overruns that gap, at which point the
/// client's write is ignored and the shared shift register sends the
/// byte it just received back instead. Both ends walk i in order, so
/// both ends generate with this and keep `pattern_value` for checking
/// after the fact.
class Stream {
public:
    constexpr Stream() = default;
    constexpr Stream(uint8_t pattern, uint8_t seed)
        : pattern_(pattern), v_(pattern == pattern_prbs ? (seed ? seed : 0xACu) : seed) {}

    constexpr uint8_t next() {
        const uint8_t out = v_;
        if (pattern_ == pattern_prbs) v_ = prbs_next(v_);
        else if (pattern_ != pattern_fixed) v_ = static_cast<uint8_t>(v_ + 1);
        return out;
    }

private:
    uint8_t pattern_ = pattern_counting;
    uint8_t v_ = 0;
};

/// A byte with its bits in the opposite order - what an end reading MSb
/// first sees of a byte an end wrote LSb first.
constexpr uint8_t bit_reverse(uint8_t v) {
    uint8_t r = 0;
    for (uint8_t i = 0; i < 8; ++i) {
        r = static_cast<uint8_t>((r << 1) | ((v >> i) & 1u));
    }
    return r;
}

}  // namespace spilink
