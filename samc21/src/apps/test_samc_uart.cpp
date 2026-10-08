// test_samc_uart - the SERCOM USART transport (DS60001479M ch. 30/31),
// both shapes of it, byte for byte.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over a
// serial console; z runs the self-contained ones and prints the ALL: line
// bin/brio judges.
//
// WHAT THIS SUITE IS FOR. samc21/sercom.hpp's Uart is built in two shapes
// - interrupt on both directions, or the DMA transmit engine (samc21/
// dmac.hpp's DmaTxEngine) beside the interrupt receiver - and what is
// checked here is the BYTES through each of them, on both sides at once,
// with a pattern in which a lost, duplicated or reordered byte is
// IDENTIFIED and not merely counted. The engine is one channel, channel 0,
// claimed by one owner at a time (erratum 1.10.4, DS80000740S, leaves the
// DMAC one lane): letter d exercises the claim as a value, letter x the
// refusal of a second owner as the panic it is, and letter h carries the
// duplex link at a rate where the shape matters.
//
// THE PATTERN is a 32-bit xorshift, low byte per step, seeded the same at
// both ends (brio stress runs the identical arithmetic). The
// board can therefore verify a received stream WITHOUT a return path, and
// the host can verify an echoed one - so every leg is checked at both
// ends, and the position of the first wrong byte is a number.
//
// TWO WIRES. The console's SERCOM5 reaches the host through the board's
// CH340, and the letters that stream traffic over it sit OUTSIDE z, driven
// by brio stress - the only peer that can speak a frame format or a rate
// the board is not using, and so the only way to reach the receiver's
// error paths at all. The second wire needs no peer: SERCOM1 with its
// receiver on its own transmitter's pad (PAD[0], PA16 under function C),
// 31.6.3.8's loop-back "through the pad" - nothing is wired to PA16. z
// runs what the board decides alone: the baud arithmetic, the frame
// encoding, the ring contract, the engine's facts and its claim, and on
// the loop the wire's idle, the interrupt receiver's edge and its skip
// past a loss, and the level loop at the generator's top rate.
//
// THE CHOREOGRAPHY every host letter follows, so the console survives:
//
//   1. the board prints one line, "  HOST <op> <mode> <baud> <format>
//      <ms> <n>", through the ordinary console, and drains it;
//   2. it releases the console transport and brings up the one under
//      test, at the rate and format it just announced, then settles;
//   3. the window runs; the host pumps for LESS than the window, so the
//      wire is QUIET before the board speaks again - that silence is what
//      separates the stream from the report, and without it the report is
//      read as payload and every count is a lie;
//   4. the board hands the console back at 115200 8N1 and prints its
//      verdicts.
//
// What is exercised, letter by letter - a..d, q, r and t need nothing
// outside the board and are what z runs; e..p and s stream traffic over the
// console and are driven by brio stress; x stops the board and runs by its
// name alone:
//   a  the baud generator's arithmetic
//   b  every frame format, written and read back
//   c  the transmit ring's contract under pressure
//   d  the transmit engine's facts and its claim: one owner at a time
//   e  echo through the plain interrupt transport
//   f  echo with the DMA transmitter
//   h  echo at 1 and 2 Mbaud: the DMA transmitter beside the interrupt
//      receiver
//   i  receive-only, sustained
//   j  transmit-only, sustained, through the DMA transmitter
//   k  the same traffic at 115200, 1 M and 3 Mbaud
//   l  the frame-format matrix, host-driven
//   m  a MISMATCHED frame and the recovery from it
//   n  ring pressure on the interrupt receiver: a consumer that drains at
//      once against one held past the ring - every gap in what the lazy
//      one is handed carried by a move of the skip epoch, none silent
//   p  bursty traffic with idle gaps, the DMA transmitter echoing
//   q  tx_idle() on the loop: never before the last stop bit, within a bit
//      after it, for the plain transport and the transmit engine
//   r  the interrupt receiver on the loop: one edge from the vector a
//      burst; a consumer held back past the ring - the loss counted, the
//      skip epoch moved at the first look after it and nothing the ring
//      held across it handed out; the consumer's release recovering
//   s  errors injected under the interrupt receiver (host): each hit
//      character dropped, counted and skipped past
//   t  the interrupt receiver at the generator's top rate on the loop:
//      every level an entry
//   x  THE ONE-USER RULE AS A PANIC: a second engined transport's init()
//      while the loop's holds the claim - the board stops, and the boot
//      banner after the next reset prints the breadcrumb (code 2,
//      assert_failed; context 0xD6, the claim of SERCOM5 refused)
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>
#include <span>

#include "kernel/panic.hpp"
#include "samc21/clock.hpp"
#include "samc21/dmac.hpp"
#include "samc21/nvic.hpp"
#include "samc21/pin.hpp"
#include "samc21/sercom.hpp"
#include "samc21/ticker.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

constexpr UartPads console_pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'B', 30, PinFunction::d},
    .rx_pin = {'B', 31, PinFunction::d},
};

// 512 each way: a ring of 512 holds 511 bytes, 44 ms of a 115200 stream -
// the span letter n's lazy consumer is held past - and the four
// instantiations, two here and two on the loop below, fit comfortably in
// 32 KB of SRAM.
constexpr uint32_t rx_ring = 512;
constexpr uint32_t tx_ring = 512;

using UPlain = Uart<5, console_pads, rx_ring, tx_ring>;
using UTxDma = Uart<5, console_pads, rx_ring, tx_ring, DmaTxEngine>;

constexpr UPlain plain;

using Sc5 = UPlain::Resource;

TestBench<UPlain, 20> bench;

using brio::crlf;
using brio::print;

/// The engine's owner id each engined transport claims with: its SERCOM
/// number plus one (samc21/dmac.hpp, DmaTxEngine::claim()).
constexpr uint8_t console_owner = static_cast<uint8_t>(Sc5::index + 1u);

// =============================================================================
// The two transports behind one set of verbs
// =============================================================================
//
// THE TWO INSTANTIATIONS ARE NEVER LIVE AT ONCE. Each has its own rings
// and counters, and init() resets and reconfigures SERCOM5 from scratch,
// so switching is a clean handover. The engine is ONE set of static
// members shared by every Uart that names it, and each Uart's dma_isr()
// consumes its OWN ring by a completed block's length - so DMAC_Handler
// tells exactly one transport about a completion: the one that holds the
// claim (DmaTxEngine::owner()).

// The numbers are brio stress's protocol.
enum class Mode : uint8_t { plain = 0, txdma = 1 };
Mode live = Mode::plain;

const char* mode_name(Mode m) { return m == Mode::txdma ? "dmaTX+irqRX" : "irqTX+irqRX"; }

bool mode_init(Mode m, uint32_t baud, const UartFormat& fmt = {}) {
    live = m;
    return m == Mode::txdma ? UTxDma::init(clock, baud, fmt) : UPlain::init(clock, baud, fmt);
}

void mode_release() {
    if (live == Mode::txdma) {
        UTxDma::release();
    } else {
        UPlain::release();
    }
}

bool mode_tx_idle() { return live == Mode::txdma ? UTxDma::tx_idle() : UPlain::tx_idle(); }

uint32_t mode_write_bulk(const uint8_t* p, uint32_t n) {
    const std::span<const uint8_t> s(p, n);
    return live == Mode::txdma ? UTxDma::write_bulk(s) : UPlain::write_bulk(s);
}

uint32_t mode_read_bulk(uint8_t* p, uint32_t n) {
    const std::span<uint8_t> s(p, n);
    return live == Mode::txdma ? UTxDma::read_bulk(s) : UPlain::read_bulk(s);
}

/// An EMPTY bulk write: it queues nothing and returns zero, but it takes
/// sercom.hpp's blocked-transmitter path on the way out - which is how a
/// loop that is WAITING for the ring to drain (rather than filling it)
/// still gives the transport its push, and its repair.
void mode_nudge() { (void)mode_write_bulk(nullptr, 0); }

void mode_clear_errors() {
    if (live == Mode::txdma) {
        UTxDma::clear_errors();
    } else {
        UPlain::clear_errors();
    }
}

struct ErrCounts {
    uint8_t rx_overrun, frame, parity, hw_overrun, dma_faults;
};

/// The skip epoch of the live transport, modulo 2^8 (never cleared: a leg
/// reads its difference).
uint32_t mode_skips() { return live == Mode::txdma ? UTxDma::rx_skips() : UPlain::rx_skips(); }

ErrCounts mode_errors() {
    if (live == Mode::txdma) {
        return {UTxDma::rx_overruns(), UTxDma::frame_errors(), UTxDma::parity_errors(),
                UTxDma::hw_overruns(), UTxDma::dma_faults()};
    }
    return {UPlain::rx_overruns(), UPlain::frame_errors(), UPlain::parity_errors(),
            UPlain::hw_overruns(), UPlain::dma_faults()};
}

// =============================================================================
// The stream, and the clock
// =============================================================================

/// The 32-bit xorshift both ends run. brio stress holds the same
/// three shifts and the same seed; if either moves, every letter that
/// verifies a stream stops meaning anything, so neither does.
constexpr uint32_t lfsr_seed = 0x12345678UL;

[[gnu::always_inline]] inline uint8_t lfsr_next(uint32_t& s) {
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return static_cast<uint8_t>(s & 0xFFu);
}

/// A cycle count that spans ticks: the kernel ticker's own cycles(), the
/// tick count and the counter's position composed right across the reload
/// (util/cycle_count.hpp); it wraps at 2^32, so only differences are used.
uint32_t cycles_now() {
    return Ticker::cycles();
}

uint32_t ms_cycles(uint32_t ms) { return (SysClock::hz / 1000u) * ms; }

void spin_ms(uint32_t ms) {
    const uint32_t t0 = cycles_now();
    const uint32_t want = ms_cycles(ms);
    while (cycles_now() - t0 < want) {
    }
}

/// Bounded drain, nudging as it waits. True when the transport emptied.
bool drain(uint32_t spins = 20'000'000UL) {
    while (!mode_tx_idle() && spins-- != 0u) {
        mode_nudge();
    }
    if (!mode_tx_idle()) {
        return false;
    }
    spins = 2'000'000UL;
    while (!Sc5::txc_flag() && spins-- != 0u) {
    }
    return true;
}

// =============================================================================
// Raw observation of the transmit engine
// =============================================================================
//
// TAKEN while the transport under test is still live, PRINTED afterwards
// through the console: the state that matters is the state at the end of
// the window, and printing it out of the transport being measured would
// perturb the very thing being read. The channel's own registers are read
// only while a transport holds the engine - the engine brings the block's
// clock up at its arm() and stops it at its release().

struct EngineSnapshot {
    uint8_t sercom_flags, sercom_armed;
    uint16_t sercom_status;
    uint8_t owner;
    bool busy, waiting;
    uint16_t in_flight;
    uint32_t faults;
};

EngineSnapshot snapshot() {
    EngineSnapshot s{};
    s.sercom_flags = Sc5::flags();
    s.sercom_armed = Sc5::armed();
    s.sercom_status = Sc5::status();
    s.owner = DmaTxEngine::owner();
    s.busy = DmaTxEngine::busy();
    s.in_flight = DmaTxEngine::in_flight();
    s.waiting = s.owner != 0u && DmaTxEngine::waiting();
    s.faults = DmaTxEngine::faults();
    return s;
}

void dump_engine(const EngineSnapshot& s) {
    print(plain, "  sercom intflag=", s.sercom_flags, " intenset=", s.sercom_armed,
          " status=", s.sercom_status, crlf);
    print(plain, "  engine owner=", s.owner, " busy=", s.busy ? 1u : 0u, " in_flight=",
          s.in_flight, " channel waiting=", s.waiting ? 1u : 0u, " blocks abandoned=", s.faults,
          crlf);
}

// =============================================================================
// The host-driven window
// =============================================================================

enum class Op : uint8_t { echo, sink, source, burst };

const char* op_name(Op o) {
    switch (o) {
        case Op::echo: return "echo";
        case Op::sink: return "sink";
        case Op::source: return "source";
        default: return "burst";
    }
}

struct Leg {
    Op op = Op::echo;
    Mode mode = Mode::plain;
    uint32_t baud = 115200;
    UartFormat format{};
    uint32_t window_ms = 1200;
    uint32_t count = 0;         ///< bytes to emit, for `source`

    /// What the board TELLS the host to use. Normally the same as its
    /// own settings; letter m deliberately makes them differ, which is
    /// the only way a receiver's error paths can be reached at all - a
    /// board cannot send itself a broken frame.
    bool announce_other = false;
    uint32_t announce_baud = 115200;
    UartFormat announce_format{};
};

struct LegResult {
    uint32_t received;
    uint32_t echoed;
    uint32_t dropped;
    uint32_t sent;
    uint32_t first_bad;   ///< 1-based position of the first byte off the stream
    uint32_t skips;       ///< how far rx_skips() moved over the window
    bool drained;
    ErrCounts err;
};

uint8_t hop[256];

char format_bits(const UartFormat& f) {
    switch (f.bits) {
        case UartBits::five: return '5';
        case UartBits::six: return '6';
        case UartBits::seven: return '7';
        case UartBits::nine: return '9';
        default: return '8';
    }
}
char format_parity(const UartFormat& f) {
    switch (f.parity) {
        case UartParity::even: return 'E';
        case UartParity::odd: return 'O';
        default: return 'N';
    }
}

/// Steps 1 and 2 of the choreography: the HOST line, the console drained
/// and released, the transport under test brought up and settled, its
/// counters cleared.
void open_leg(const Leg& leg) {
    const uint32_t say_baud = leg.announce_other ? leg.announce_baud : leg.baud;
    const UartFormat say_format = leg.announce_other ? leg.announce_format : leg.format;
    print(plain, "  HOST ", op_name(leg.op), " ", static_cast<uint32_t>(leg.mode),
          " ", say_baud, " ", format_bits(say_format), format_parity(say_format),
          say_format.two_stop ? '2' : '1', " ", leg.window_ms, " ", leg.count, crlf);
    (void)drain();
    mode_release();

    (void)mode_init(leg.mode, leg.baud, leg.format);
    spin_ms(120);
    mode_clear_errors();
    DmaTxEngine::clear_faults();
}

/// One host-driven leg, start to finish (see the choreography at the top).
LegResult run_leg(const Leg& leg) {
    LegResult r{};
    open_leg(leg);
    const uint32_t skips0 = mode_skips();

    // A frame narrower than eight bits carries only its low bits, so the
    // pattern is compared through the same mask the wire applies.
    uint8_t mask = 0xFF;
    switch (leg.format.bits) {
        case UartBits::five: mask = 0x1F; break;
        case UartBits::six: mask = 0x3F; break;
        case UartBits::seven: mask = 0x7F; break;
        default: break;
    }

    uint32_t s_rx = lfsr_seed;
    uint32_t s_tx = lfsr_seed;
    const uint32_t t0 = cycles_now();
    const uint32_t window = ms_cycles(leg.window_ms);
    uint32_t turns = 0;
    uint8_t out[128];
    uint32_t out_have = 0;
    uint32_t out_done = 0;

    while (cycles_now() - t0 < window) {
        // ---- the receive half ------------------------------------------
        if (leg.op != Op::source) {
            const uint32_t got = mode_read_bulk(hop, sizeof hop);
            if (got != 0u) {
                for (uint32_t i = 0; i < got; ++i) {
                    const uint8_t want = static_cast<uint8_t>(lfsr_next(s_rx) & mask);
                    if (hop[i] != want && r.first_bad == 0u) {
                        r.first_bad = r.received + i + 1u;
                    }
                }
                r.received += got;
                if (leg.op == Op::echo || leg.op == Op::burst) {
                    const uint32_t put = mode_write_bulk(hop, got);
                    r.echoed += put;
                    r.dropped += got - put;
                }
            } else {
                mode_nudge();
            }
        }

        // ---- the transmit half -----------------------------------------
        //
        // The generator runs ONCE per byte and the ring may take only
        // part of a batch, so the unsent tail is kept and offered again -
        // an xorshift cannot be rewound, and re-deriving it would put a
        // gap in the very stream the host is checking.
        if (leg.op == Op::source && (out_done < out_have || r.sent < leg.count)) {
            if (out_done == out_have) {
                const uint32_t left = leg.count - r.sent;
                out_have = left < sizeof out ? left : sizeof out;
                out_done = 0;
                for (uint32_t i = 0; i < out_have; ++i) {
                    out[i] = static_cast<uint8_t>(lfsr_next(s_tx) & mask);
                }
            }
            const uint32_t put = mode_write_bulk(out + out_done, out_have - out_done);
            out_done += put;
            r.sent += put;
            if (put == 0u) {
                mode_nudge();
            }
        }

        // ---- bursty traffic: idle gaps between windows -------------------
        if (leg.op == Op::burst) {
            ++turns;
            if ((turns & 0x1FFu) == 0u) {
                spin_ms(4);
            }
        }
    }

    r.drained = drain();
    r.err = mode_errors();
    r.skips = (mode_skips() - skips0) & 0xFFu;
    return r;
}

/// Come back to the plain console.
void back_to_console() {
    mode_release();
    (void)mode_init(Mode::plain, 115200);
    spin_ms(150);
}

void report(const Leg& leg, const LegResult& r) {
    print(plain, "  ", op_name(leg.op), " ", mode_name(leg.mode), " ", leg.baud,
          " ", format_bits(leg.format), format_parity(leg.format),
          leg.format.two_stop ? '2' : '1', ": received=", r.received,
          " sent=", r.sent, " echoed=", r.echoed, " dropped=", r.dropped,
          " first_bad=", r.first_bad, crlf);
    print(plain, "     rx_overrun=", r.err.rx_overrun, " frame=", r.err.frame,
          " parity=", r.err.parity, " hw_overrun=", r.err.hw_overrun,
          " dma_faults=", r.err.dma_faults, " skips=", r.skips,
          " drained=", r.drained ? "yes" : "NO", crlf);
}

// =============================================================================
// z - what the board can decide on its own
// =============================================================================

void ta_baud() {
    static constexpr uint32_t ref = 48'000'000;
    static_assert(sercom_baud_reg(ref, 3'000'000).value() == 0,
                  "3 Mbaud is f_ref/16 exactly: the fastest this mode reaches");
    static_assert(sercom_baud_reg(ref, 1'500'000).value() == 32768,
                  "1.5 Mbaud is exactly half of it");
    static_assert(!sercom_baud_reg(ref, 3'000'001).has_value(),
                  "one bit per second past f_ref/16 is out of the mode's range");
    static_assert(sercom_min_ref_hz(115200) == 1'843'200);

    bench.verdict("3 Mbaud is BAUD 0 - f_ref/16, the mode's own ceiling",
                  sercom_baud_reg(ref, 3'000'000) == 0);
    bench.verdict("a rate past the ceiling is refused, not clamped",
                  !sercom_baud_reg(ref, 3'000'001).has_value());
    bench.verdict("BAUD 0 is legal, so a refusal cannot be spelled as zero",
                  sercom_baud_reg(ref, 3'000'000).has_value());

    // WHAT THE BUDGET IS, and it is not a guess about "close enough".
    // BAUD is a 16-bit fraction of the whole range, so one step is
    // f_ref / (65536 x S) = 45.8 Hz at 48 MHz AT EVERY RATE - an absolute
    // quantum, not a relative one. Rounding to nearest therefore bounds
    // the absolute error at HALF a step, 23 Hz, whether the rate is 9600
    // or 3 M; and that is why a relative budget is the wrong claim
    // entirely. At 9600 those same 13 Hz are 1444 ppm and at 3 Mbaud the
    // error is zero, and both are the same arithmetic doing its best.
    static constexpr uint32_t half_step_hz = ref / (65536UL * 16UL * 2UL) + 1u;
    static constexpr uint32_t rates[] = {
        9600, 115200, 460800, 921600, 1'000'000, 2'000'000, 3'000'000,
    };
    bool within_half_step = true;
    for (uint32_t baud : rates) {
        const auto reg = sercom_baud_reg(ref, baud);
        if (!reg) {
            within_half_step = false;
            continue;
        }
        const uint32_t got = sercom_actual_baud(ref, *reg);
        const uint32_t err = got > baud ? got - baud : baud - got;
        const uint32_t ppm = (err * 1000UL) / (baud / 1000UL);
        print(plain, "  ", baud, " -> BAUD ", *reg, " -> ", got, " Hz (", err,
              " Hz, ", ppm, " ppm)", crlf);
        if (err > half_step_hz) {
            within_half_step = false;
        }
    }
    print(plain, "  one BAUD step is ", ref / (65536UL * 16UL),
          " Hz at this reference, so half a step is ", half_step_hz, " Hz", crlf);
    bench.verdict("every rate lands within HALF a BAUD step, at any rate",
                  within_half_step);

    const uint32_t actual = UPlain::actual_baud(SysClock::hz);
    print(plain, "  the console right now: BAUD ", Sc5::baud_reg(), " = ", actual,
          " Hz", crlf);
    bench.verdict("the console's own divisor reads back as 115200 +- 0.3%",
                  actual > 114'850u && actual < 115'550u);
    bench.verdict("min_hz_for states the mode's condition, not a guess",
                  UPlain::min_hz_for(115200) == 1'843'200u);
    bench.verdict("can_baud refuses what the generator cannot make",
                  UPlain::can_baud(48'000'000, 3'000'000) &&
                      !UPlain::can_baud(48'000'000, 4'000'000));
}

void tb_format() {
    // Every frame the driver claims, written into the peripheral and read
    // back out of CTRLA and CTRLB. The console is re-initialized for each
    // one; the verdicts are collected first and printed afterwards,
    // because a verdict printed at 7E2 would arrive as noise.
    struct Row {
        UartBits bits;
        UartParity parity;
        bool two_stop;
        uint32_t chsize;
    };
    static constexpr Row rows[] = {
        {UartBits::eight, UartParity::none, false, 0x0},
        {UartBits::eight, UartParity::even, false, 0x0},
        {UartBits::eight, UartParity::odd, true, 0x0},
        {UartBits::seven, UartParity::none, false, 0x7},
        {UartBits::seven, UartParity::even, true, 0x7},
        {UartBits::six, UartParity::odd, false, 0x6},
        {UartBits::five, UartParity::none, true, 0x5},
        {UartBits::nine, UartParity::none, false, 0x1},
    };
    bool chsize_ok = true, parity_ok = true, stop_ok = true, dord_ok = true;
    uint32_t ctrla_seen = 0, ctrlb_seen = 0;
    for (const Row& row : rows) {
        const UartFormat f{.bits = row.bits, .parity = row.parity,
                           .two_stop = row.two_stop};
        (void)UPlain::init(clock, 115200, f);
        const uint32_t ctrla = Sc5::regs().SERCOM_CTRLA;
        const uint32_t ctrlb = Sc5::regs().SERCOM_CTRLB;
        ctrla_seen = ctrla;
        ctrlb_seen = ctrlb;
        if (((ctrlb & SERCOM_USART_INT_CTRLB_CHSIZE_Msk) >>
             SERCOM_USART_INT_CTRLB_CHSIZE_Pos) != row.chsize) {
            chsize_ok = false;
        }
        const bool form_parity =
            ((ctrla & SERCOM_USART_INT_CTRLA_FORM_Msk) >>
             SERCOM_USART_INT_CTRLA_FORM_Pos) ==
            SERCOM_USART_INT_CTRLA_FORM_USART_FRAME_WITH_PARITY_Val;
        const bool pmode_odd = (ctrlb & SERCOM_USART_INT_CTRLB_PMODE_Msk) != 0u;
        if (form_parity != (row.parity != UartParity::none) ||
            pmode_odd != (row.parity == UartParity::odd)) {
            parity_ok = false;
        }
        if (((ctrlb & SERCOM_USART_INT_CTRLB_SBMODE_Msk) != 0u) != row.two_stop) {
            stop_ok = false;
        }
        if ((ctrla & SERCOM_USART_INT_CTRLA_DORD_Msk) == 0u) {
            dord_ok = false;
        }
    }
    (void)UPlain::init(clock, 115200);
    spin_ms(20);

    print(plain, "  last of the sweep: CTRLA=", hex(ctrla_seen), " CTRLB=",
          hex(ctrlb_seen), "; back at 8N1", crlf);
    bench.verdict("CHSIZE carries the chapter's own non-contiguous codes",
                  chsize_ok);
    bench.verdict("parity is TWO registers and both follow the one enum",
                  parity_ok);
    bench.verdict("CTRLB.SBMODE follows two_stop", stop_ok);
    bench.verdict("DORD is LSB-first by default, against the reset value",
                  dord_ok);
    bench.verdict("the console survived the whole sweep",
                  Sc5::enabled() && UPlain::actual_baud(SysClock::hz) > 114'000u);
}

void tc_ring() {
    // The transmit ring's contract, exercised through the transport with
    // the wire deliberately too slow to help: capacity is size - 1, a
    // refused byte is REFUSED and not silently dropped, and write_bulk
    // fills exactly the room there is. Every one of those refusals goes
    // through the blocked-transmitter nudge, so this also proves that
    // path cannot wedge.
    (void)drain();
    // The filler is a printable dot: these bytes really do go out on the
    // console wire, and a binary ramp would paint the transcript.
    uint32_t queued = 0;
    while (queued < tx_ring + 64u && UPlain::write_byte('.')) {
        ++queued;
    }
    const bool full_refused = !UPlain::write_byte(0) || queued >= tx_ring + 64u;
    print(plain, "  the ring took ", queued,
          " bytes before refusing (capacity ", tx_ring - 1u,
          " plus whatever left on the wire meanwhile)", crlf);
    bench.verdict("a full ring refuses rather than overwriting", full_refused);
    bench.verdict("it held at least its stated capacity", queued >= tx_ring - 1u);
    bench.verdict("the refusal path drains rather than wedging", drain());

    static uint8_t block[tx_ring + 32];
    for (uint32_t i = 0; i < sizeof block; ++i) {
        block[i] = '.';
    }
    (void)drain();
    const uint32_t placed =
        UPlain::write_bulk(std::span<const uint8_t>(block, sizeof block));
    print(plain, "  write_bulk placed ", placed, " of ", sizeof block, crlf);
    bench.verdict("write_bulk fills the ring and stops at its edge",
                  placed >= tx_ring - 1u && placed < sizeof block);
    bench.verdict("an empty run is a legal nudge that queues nothing",
                  UPlain::write_bulk(std::span<const uint8_t>()) == 0);
    bench.verdict("and the whole lot goes out", drain());
}

/// Two owner ids no Uart of this image claims with (a Uart's is its SERCOM
/// number plus one: 2 on the loop, 6 on the console), so letter d's claims
/// can never be mistaken for a transport's.
constexpr uint8_t owner_a = 9;
constexpr uint8_t owner_b = 10;

void td_engine() {
    bench.verdict("the plain transport names no engine", !UPlain::has_tx_engine);
    bench.verdict("the engined one names the transmit engine", UTxDma::has_tx_engine);
    bench.verdict("the engine is one channel, channel 0",
                  DmaTxEngine::present && DmaTxEngine::channel == 0u);
    bench.verdict("an engineless transport reports no DMA faults, for free",
                  UPlain::dma_faults() == 0);
    // The trigger code is the device header's, read per instance by
    // samc21/sercom.hpp beside the instance's other constants.
    bench.verdict("SERCOM5's TX trigger code is the header's",
                  Sc5::dma_tx_trigger() == SERCOM5_DMAC_ID_TX);

    // THE ONE-USER RULE AS A VALUE: the claim the engine's arm() makes,
    // exercised directly with owners no transport uses. The console runs
    // plain under z, so nobody holds the engine on entry. (A refused claim
    // inside arm() is the panic letter x proves; claim() alone answers.)
    const uint8_t on_entry = DmaTxEngine::owner();
    const bool a_first = DmaTxEngine::claim(owner_a);
    const bool b_refused = !DmaTxEngine::claim(owner_b);
    const uint8_t held_by = DmaTxEngine::owner();
    const bool a_again = DmaTxEngine::claim(owner_a);
    DmaTxEngine::release(owner_a);
    const uint8_t after_a = DmaTxEngine::owner();
    const bool b_now = DmaTxEngine::claim(owner_b);
    const uint8_t held_by_b = DmaTxEngine::owner();
    DmaTxEngine::release(owner_b);
    const uint8_t after_b = DmaTxEngine::owner();
    print(plain, "  owner on entry ", on_entry, "; claim(", owner_a, ") ", a_first, ", claim(",
          owner_b, ") ", !b_refused, " while ", held_by, " holds, claim(", owner_a, ") again ",
          a_again, "; after release(", owner_a, ") ", after_a, "; claim(", owner_b, ") ", b_now,
          ", held by ", held_by_b, "; after release(", owner_b, ") ", after_b, crlf);
    bench.verdict("nobody holds the engine while the console runs plain", on_entry == 0u);
    bench.verdict("a free engine is claimed", a_first && held_by == owner_a);
    bench.verdict("a second owner is refused while the first holds it", b_refused);
    bench.verdict("the holder's own claim again is granted", a_again);
    bench.verdict("release() gives the claim back", after_a == 0u);
    bench.verdict("and the other owner may take it then (one user at a time)",
                  b_now && held_by_b == owner_b);
    bench.verdict("its release leaves nobody holding the engine", after_b == 0u);
}

// =============================================================================
// The host letters
// =============================================================================

void run_and_report(const Leg& leg, bool lossless) {
    const LegResult r = run_leg(leg);
    const EngineSnapshot s = snapshot();
    back_to_console();
    report(leg, r);
    dump_engine(s);

    bench.verdict("the transport drained at the end of the window", r.drained);
    bench.verdict("something crossed the wire", r.received != 0u || r.sent != 0u);
    if (lossless) {
        bench.verdict("every received byte was on the stream, in order",
                      r.first_bad == 0);
        bench.verdict("nothing was dropped on the way back", r.dropped == 0);
        bench.verdict("no hardware overrun", r.err.hw_overrun == 0);
    } else {
        print(plain, "  (lossy allowed at this rate: loss must be counted, never silent)",
              crlf);
        bench.verdict("the stream is contiguous well past the start",
                      r.first_bad == 0 || r.first_bad > 256);
        bench.verdict("the transmitter was not left claiming a dead block",
                      !s.busy);
    }
}

Leg base_leg(Op op, Mode m) {
    Leg leg{};
    leg.op = op;
    leg.mode = m;
    return leg;
}

void te_plain_echo() { run_and_report(base_leg(Op::echo, Mode::plain), true); }
void tf_txdma_echo() { run_and_report(base_leg(Op::echo, Mode::txdma), true); }
/// THE DUPLEX LINK. The Uart's one engine slot is the transmit one
/// (samc21/sercom.hpp), so a link that wants bulk both ways takes the
/// engine on the transmit side and keeps the receiver on RXC. Two rates,
/// two claims:
///  - at 1 Mbaud the shape is LOSSLESS - every byte back, in order,
///    nothing dropped, no hardware overrun;
///  - at 2 Mbaud the receiver takes every level an entry and keeps the
///    stream but for a character or three when another handler holds it
///    past two frames (the transmit engine's completion and the tick
///    together), and the echo gains on the board's slow clock (below):
///    loss is allowed and SILENCE is not, as in letter k.
/// At both, NO TRANSMIT BLOCK MAY BE ABANDONED: the engine's channel is
/// the only one the stratum drives, so erratum 1.10.4 has nothing to
/// corrupt it with, and an abandoned block is the dead-block predicate
/// firing on a live one - which a test of the engine's own "in flight" and
/// the flags alone does at 2 Mbaud, where main context is preempted for a
/// character time across a completion (samc21/sercom.hpp,
/// nudge_blocked_tx()).
void th_duplex_echo() {
    static constexpr uint32_t rates[] = {1'000'000, 2'000'000};
    for (uint32_t rate : rates) {
        Leg leg = base_leg(Op::echo, Mode::txdma);
        leg.baud = rate;
        const LegResult r = run_leg(leg);
        const EngineSnapshot s = snapshot();
        back_to_console();
        report(leg, r);
        dump_engine(s);
        bench.verdict("the transport drained at the end of the window", r.drained);
        bench.verdict("something crossed the wire", r.received != 0u);
        if (rate <= 1'000'000u) {
            bench.verdict("nothing was dropped on the way back", r.dropped == 0);
        } else {
            // THE ECHO CAN FIND ITS TRANSMIT RING FULL at 2 Mbaud: 0 to 18
            // characters a window in six runs, consistent with the board
            // sending back slower than the bridge sends in - OSC48M is half
            // a per cent off nominal on this board (docs/boards/
            // samc21j.md), and half a per cent of the 105000 bytes a
            // window carries is the ring's 511. Dropped is allowed there;
            // uncounted is not.
            bench.verdict("what the echo could not hold was counted",
                          r.echoed + r.dropped == r.received);
        }
        if (rate <= 1'000'000u) {
            bench.verdict("every received byte was on the stream, in order",
                          r.first_bad == 0);
            bench.verdict("no hardware overrun", r.err.hw_overrun == 0);
        } else {
            print(plain, "  (at 2 Mbaud the interrupt receiver is the limit - loss "
                         "is allowed, silence is not)", crlf);
            bench.verdict("nothing was lost silently at this rate",
                          r.first_bad == 0 || r.err.hw_overrun != 0 ||
                              r.err.rx_overrun != 0);
        }
        bench.verdict("no transmit block was abandoned (one channel, nothing to "
                      "corrupt it)",
                      r.err.dma_faults == 0);
    }
}

/// RX-only sustained: the host pumps, the board verifies against its own
/// copy of the generator and answers nothing at all.
void ti_sink() { run_and_report(base_leg(Op::sink, Mode::plain), true); }

/// TX-only sustained: the board emits the generator's stream through the
/// DMA transmitter, the host verifies it.
void tj_source() {
    Leg leg = base_leg(Op::source, Mode::txdma);
    leg.count = 12000;
    leg.window_ms = 1400;
    run_and_report(leg, true);
}

void tk_rates() {
    // The same echo at three rates through the plain transport, which is
    // the shape that has to work everywhere. 3 Mbaud is the generator's
    // ceiling and the wire's measured limit (docs/samc21/sercom.md).
    static constexpr uint32_t rates[] = {115200, 1'000'000, 3'000'000};
    for (uint32_t rate : rates) {
        Leg leg = base_leg(Op::echo, Mode::plain);
        leg.baud = rate;
        leg.window_ms = 900;
        const LegResult r = run_leg(leg);
        back_to_console();
        report(leg, r);
        bench.verdict("the rate carried a stream at all", r.received != 0u);
        // WHAT MAY BE CLAIMED AT WHICH RATE, and it is not the same
        // claim. Through the interrupt transport an echo is lossless to
        // about 1 Mbaud; above that the FILLER gives way - one RXC
        // interrupt per byte is 300k/s at 3 Mbaud, more than the two-deep
        // FIFO survives - and the loss lands in the HARDWARE, which is
        // measured in docs/samc21/sercom.md. So the fast rate is not asked
        // to be lossless; it is asked to be HONEST: every byte that did
        // not arrive must show up in a counter, never in silence.
        if (rate <= 1'000'000u) {
            bench.verdict("byte-exact at this rate", r.first_bad == 0);
        } else {
            const bool accounted =
                r.first_bad == 0 || r.err.hw_overrun != 0 || r.err.rx_overrun != 0;
            print(plain, "  (above 1 Mbaud the receiver's own filler is the "
                         "limit - loss is expected, silence is not)", crlf);
            bench.verdict("nothing was lost silently at this rate", accounted);
        }
    }
}

void tl_formats() {
    // The format matrix against a host that can speak all of them: every
    // frame this driver claims and pyserial can produce, byte-exact both
    // directions.
    struct Row { UartBits bits; UartParity parity; bool two_stop; };
    static constexpr Row rows[] = {
        {UartBits::eight, UartParity::even, false},
        {UartBits::eight, UartParity::odd, false},
        {UartBits::eight, UartParity::none, true},
        {UartBits::seven, UartParity::even, false},
        {UartBits::seven, UartParity::none, true},
    };
    for (const Row& row : rows) {
        Leg leg = base_leg(Op::echo, Mode::plain);
        leg.format = {.bits = row.bits, .parity = row.parity,
                      .two_stop = row.two_stop};
        leg.window_ms = 900;
        const LegResult r = run_leg(leg);
        back_to_console();
        report(leg, r);
        bench.verdict("the frame carried the stream", r.received != 0u);
        bench.verdict("byte-exact through this frame", r.first_bad == 0);
        bench.verdict("and no receive error was raised",
                      r.err.frame == 0 && r.err.parity == 0);
    }
}

void tm_mismatch() {
    // THE CONTROL THAT MAKES THE FORMAT LETTER MEAN SOMETHING: the same
    // stream, sent by a host that is deliberately wrong. A receiver's
    // error paths cannot be reached any other way - a board cannot send
    // itself a broken frame - and an error path that is implemented and
    // never exercised is exactly the kind of claim this house does not
    // make.
    //
    // TWO PROVOCATIONS, and the first one is the sure thing. A rate the
    // receiver is not using puts its sampling point inside the wrong bit
    // for most characters, so the stop bit is read as a zero and FERR is
    // raised: half the nominal rate is far outside any tolerance and
    // cannot be mistaken for luck. The second is the FRAME mismatch, and
    // it is reported rather than asserted - see below.
    Leg fast = base_leg(Op::sink, Mode::plain);
    fast.window_ms = 700;
    fast.announce_other = true;
    fast.announce_baud = 57600;   // the host's rate; the board stays at 115200
    const LegResult wrong_rate = run_leg(fast);
    back_to_console();
    print(plain, "  host at 57600 into a 115200 receiver: received=",
          wrong_rate.received, " frame=", wrong_rate.err.frame,
          " parity=", wrong_rate.err.parity, " hw_overrun=",
          wrong_rate.err.hw_overrun, "  (the counters are bytes and wrap at 255)",
          crlf);
    bench.verdict("a wrong RATE shows: framing errors are raised and counted",
                  wrong_rate.err.frame != 0);

    // The frame mismatch: the host adds a parity bit the receiver is not
    // expecting, so the receiver samples that bit where its stop bit
    // belongs. PRINTED AND NOT VERDICTED - what it produces depends on
    // whether the bridge sends characters back to back or with gaps,
    // which is the HOST's scheduling and not a property of this silicon.
    Leg framed = base_leg(Op::sink, Mode::plain);
    framed.window_ms = 700;
    framed.announce_other = true;
    framed.announce_baud = 115200;
    framed.announce_format = {.bits = UartBits::eight, .parity = UartParity::even};
    const LegResult wrong_frame = run_leg(framed);
    back_to_console();
    print(plain, "  host at 8E1 into an 8N1 receiver: received=",
          wrong_frame.received, " frame=", wrong_frame.err.frame,
          " parity=", wrong_frame.err.parity, "  (printed, not judged: what an",
          crlf,
          "   extra bit costs depends on the sender's own gaps)", crlf);

    Leg good = base_leg(Op::sink, Mode::plain);
    good.window_ms = 700;
    const LegResult ok = run_leg(good);
    back_to_console();
    report(good, ok);
    bench.verdict("the receiver recovers: byte-exact at the right frame",
                  ok.first_bad == 0 && ok.received != 0u);
    bench.verdict("and no error is left standing",
                  ok.err.frame == 0 && ok.err.parity == 0);
}

// ---- n - an eager consumer against a lazy one --------------------------------

/**
 * The judge of a consumer that may be skipped past a loss
 * (util/ring.hpp's SkipRing): every run handed out either follows on from
 * the run before - byte for byte the pattern's next - or comes after a
 * move of the skip epoch, behind which the stream's place is found again
 * by the first sync_len bytes that match the pattern at one place ahead
 * (sixteen bytes: a chance match is 2^-128). A run off the pattern with no
 * move before it is a SILENT JOIN - the one thing the ring promises never
 * to hand out.
 */
struct StreamCheck {
    static constexpr uint8_t sync_len = 16;
    /// Further than the host sends in a window (115200 baud, under a
    /// second): a place not found this far ahead is lost.
    static constexpr uint32_t search_most = 32768;

    uint32_t s = lfsr_seed;       ///< the generator before the next byte expected
    uint32_t epoch = 0;           ///< the skip epoch at the last look, modulo 2^8
    uint32_t delivered = 0;       ///< bytes the consumer was handed
    uint32_t first_silent = 0;    ///< 1-based delivered index of a silent join
    uint32_t gaps = 0;            ///< moves of the epoch between two looks
    uint32_t jumped = 0;          ///< stream positions the resyncs skipped
    uint32_t resyncs = 0;
    uint32_t lost = 0;            ///< resyncs that found no place
    bool syncing = false;
    uint8_t held = 0;
    uint8_t buf[sync_len]{};

    void start(uint32_t epoch_now) { epoch = epoch_now & 0xFFu; }

    /// One look: the run it handed out (empty included) and the epoch
    /// read right after it.
    void look(std::span<const uint8_t> run, uint32_t epoch_now) {
        const uint32_t e = epoch_now & 0xFFu;
        if (e != epoch) {
            gaps += (e - epoch) & 0xFFu;
            epoch = e;
            syncing = true;
            held = 0;
        }
        for (const uint8_t b : run) {
            ++delivered;
            if (syncing) {
                buf[held++] = b;
                if (held == sync_len) {
                    resync();
                }
            } else if (b != lfsr_next(s) && first_silent == 0u) {
                first_silent = delivered;
            }
        }
    }

    void resync() {
        held = 0;
        uint32_t c = s;
        for (uint32_t ahead = 0; ahead < search_most; ++ahead) {
            uint32_t t = c;
            uint8_t k = 0;
            while (k < sync_len && lfsr_next(t) == buf[k]) {
                ++k;
            }
            if (k == sync_len) {
                s = t;
                jumped += ahead;
                ++resyncs;
                syncing = false;
                return;
            }
            (void)lfsr_next(c);
        }
        ++lost;   // still syncing: the next sixteen bytes try again
    }
};

struct ConsumerLeg {
    StreamCheck check;
    ErrCounts err;
    uint32_t skips;   ///< how far rx_skips() moved, modulo 2^8
    uint32_t holds;   ///< hold phases the window held
    bool drained;
};

/// The host pumps (sink, 115200); the console's interrupt receiver fills
/// its ring and the consumer takes it IN PLACE, a run a look (read_span()
/// and consume(), SerialPort's shape). EAGER looks at every turn; LAZY
/// looks for 150 ms and then holds 100 ms - longer than the 44 ms the
/// ring's 511 bytes last at this rate, so every hold the host pumps
/// through overflows it.
ConsumerLeg consumer_leg(bool lazy) {
    Leg leg = base_leg(Op::sink, Mode::plain);
    leg.window_ms = 1400;
    open_leg(leg);
    ConsumerLeg r{};
    const uint32_t skips0 = UPlain::rx_skips();
    r.check.start(skips0);
    const uint32_t look_c = ms_cycles(150);
    const uint32_t hold_c = ms_cycles(100);
    const uint32_t t0 = cycles_now();
    const uint32_t window = ms_cycles(leg.window_ms);
    bool holding = false;
    uint32_t phase_t0 = t0;
    for (uint32_t now = t0; now - t0 < window; now = cycles_now()) {
        if (lazy && now - phase_t0 >= (holding ? hold_c : look_c)) {
            holding = !holding;
            phase_t0 = now;
            r.holds += holding ? 1u : 0u;
        }
        if (holding) {
            continue;
        }
        const auto run = UPlain::read_span();
        r.check.look(run, UPlain::rx_skips());
        UPlain::consume(static_cast<uint32_t>(run.size()));
    }
    // The last look, for what a final hold left standing.
    for (;;) {
        const auto run = UPlain::read_span();
        r.check.look(run, UPlain::rx_skips());
        if (run.empty()) {
            break;
        }
        UPlain::consume(static_cast<uint32_t>(run.size()));
    }
    r.drained = drain();
    r.err = mode_errors();
    r.skips = (UPlain::rx_skips() - skips0) & 0xFFu;
    return r;
}

void tn_pressure() {
    for (const bool lazy : {false, true}) {
        const ConsumerLeg r = consumer_leg(lazy);
        back_to_console();
        const StreamCheck& c = r.check;
        print(plain, "  ", lazy ? "lazy (150 ms looking, 100 ms held)" : "eager (a look a turn)",
              ": delivered ", c.delivered, ", holds ", r.holds, ", gaps ", c.gaps, " (skips ",
              r.skips, "), resyncs ", c.resyncs, " jumping ", c.jumped, " positions, lost ",
              c.lost, ", silent join at ", c.first_silent, crlf);
        print(plain, "     rx_overrun=", r.err.rx_overrun, " (modulo 256) frame=", r.err.frame,
              " parity=", r.err.parity, " hw_overrun=", r.err.hw_overrun, " drained=",
              r.drained ? "yes" : "NO", crlf);
        bench.verdict("bytes crossed", c.delivered != 0u);
        bench.verdict("no silent join: every run follows on, or the epoch moved before it",
                      c.first_silent == 0u);
        bench.verdict("no hardware overrun", r.err.hw_overrun == 0u);
        if (lazy) {
            bench.verdict("the held ring overflowed and the loss was told: the epoch moved",
                          c.gaps != 0u);
            bench.verdict("behind every gap the stream was found again", c.lost == 0u);
        } else {
            bench.verdict("drained at once, nothing lost: the skip epoch still",
                          c.gaps == 0u && r.skips == 0u);
        }
    }
}

/// Bursty traffic with idle gaps - the shape a console really sees: the
/// board's echo pauses 4 ms every 512 turns, the interrupt receiver's ring
/// holds what arrives meanwhile, and the transmit engine starts a block
/// onto an idle wire after every pause.
void tp_burst() {
    Leg leg = base_leg(Op::burst, Mode::txdma);
    leg.window_ms = 1400;
    run_and_report(leg, true);
}


// =============================================================================
// The loop: SERCOM1 listening on its own transmitter's pad
// =============================================================================
//
// 31.6.3.8: RXPO and TXPO naming one pad put the receiver on the
// transmitter's own signal, through the pad. PAD[0] of SERCOM1 is PA16
// under function C; nothing is wired to it. Two transports over it, never
// live at once (`loop_live`).

constexpr UartPads loop_pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad0,
    .tx_pin = {'A', 16, PinFunction::c},
    .rx_pin = {'A', 16, PinFunction::c},
};

using LPlain = Uart<1, loop_pads, 512, 512>;
using LTxDma = Uart<1, loop_pads, 512, 512, DmaTxEngine>;
using Sc1 = LPlain::Resource;
constexpr uint8_t loop_owner = static_cast<uint8_t>(Sc1::index + 1u);

enum class Loop : uint8_t { none, plain, txdma };
volatile Loop loop_live = Loop::none;
/// The receive edges the loop's vector returned.
volatile uint32_t loop_edges = 0;

uint32_t bit_cycles(uint32_t baud) { return SysClock::hz / baud; }

/// A few frames of idle line after the transport comes up.
void loop_settle(uint32_t baud) {
    const uint32_t t0 = cycles_now();
    while (cycles_now() - t0 < 40u * bit_cycles(baud)) {
    }
}

/// Polled transmission through the resource, for a sender at the wire's
/// own rate beside the interrupt receiver: DATA written whenever DRE
/// stands (31.6.2.5).
[[gnu::always_inline]] inline bool loop_send_polled(uint8_t b) {
    if (!Sc1::dre_flag()) {
        return false;
    }
    Sc1::data(b);
    return true;
}

// ---- q - tx_idle() on the pad ------------------------------------------------

struct IdleProbe {
    bool idle_before_init_write;   ///< a port that has sent nothing reads idle
    bool idle_seen;
    bool last_seen;                ///< the probe read the last byte itself
    int32_t after_last;            ///< cycles from the last byte's flag to tx_idle()'s true
    uint32_t turn;                 ///< one turn of the probe, in cycles
    uint32_t received;
};

/// n bytes through U; once the transport has handed the hardware what
/// SERCOM1's own vector must feed (`handed`), that vector's NVIC line is
/// held off and the probe takes the receiver's levels itself, every
/// level a turn, asking tx_idle() every turn - the TURN at which the last
/// byte's flag is read (its stop bit, 31.6.2.6) and the turn at which
/// tx_idle() first answers true are compared, and a turn measured in
/// cycles over the whole probe. The DMAC's vector stays live: a transmit
/// engine's completion is what releases its ring.
template <typename U, typename Handed>
IdleProbe probe_idle(uint32_t baud, uint32_t n, Handed handed) {
    IdleProbe r{};
    (void)U::init(clock, baud);
    loop_settle(baud);
    r.idle_before_init_write = U::tx_idle();
    uint32_t s = lfsr_seed;
    uint8_t out[64];
    for (uint32_t i = 0; i < n; ++i) {
        out[i] = lfsr_next(s);
    }
    (void)U::write_bulk(std::span<const uint8_t>(out, n));
    uint32_t spins = 0;
    while (!handed() && spins++ < 4'000'000u) {
        asm volatile("" ::: "memory");
    }
    Nvic::disable(Sc1::irq());
    uint32_t got = static_cast<uint32_t>(U::rx_pending());
    uint32_t turn_last = 0;
    uint32_t turn_idle = 0;
    uint32_t turn = 0;
    const uint32_t c0 = cycles_now();
    for (; turn < 400'000u; ++turn) {
        while (Sc1::rxc_flag()) {
            (void)Sc1::data();
            if (++got == n) {
                turn_last = turn;
                r.last_seen = true;
            }
        }
        if (!r.idle_seen && U::tx_idle()) {
            turn_idle = turn;
            r.idle_seen = true;
        }
        if (r.idle_seen && got >= n) {
            break;
        }
    }
    const uint32_t c1 = cycles_now();
    Nvic::enable(Sc1::irq());
    r.turn = turn != 0u ? (c1 - c0) / turn : 0u;
    r.received = got;
    r.after_last = (static_cast<int32_t>(turn_idle) - static_cast<int32_t>(turn_last)) *
                   static_cast<int32_t>(r.turn);
    return r;
}

void report_idle(const char* what, uint32_t baud, const IdleProbe& r) {
    print(plain, "  ", what, " at ", baud, ": received ", r.received, ", tx_idle() read true ",
          r.after_last, " cycles after the receiver's last byte, a probe turn ", r.turn,
          ", a bit ", bit_cycles(baud), ", a frame ", 10u * bit_cycles(baud), crlf);
}

/// THE RESOLUTION. The receiver's flag for the last character and the
/// transmitter's TXC rise within a few tens of cycles of each other at
/// every rate measured, either first, and the probe reads both once a
/// turn. So tx_idle() is judged to within a turn: not before the
/// receiver's last byte by more than one, and after it by no more than a
/// bit or a turn, whichever is longer. A tx_idle() that answered on the
/// ring alone leads the receiver by the last character still in DATA -
/// a frame or more, several turns at every rate.
void judge_idle(uint32_t baud, const IdleProbe& r) {
    bench.verdict("a port that has sent nothing reads idle", r.idle_before_init_write);
    bench.verdict("tx_idle() turned true", r.idle_seen);
    bench.verdict("the probe read the last byte arrive", r.last_seen);
    const int32_t turn = static_cast<int32_t>(r.turn);
    const int32_t bit = static_cast<int32_t>(bit_cycles(baud));
    bench.verdict("not before the last character left (within a probe turn)",
                  r.idle_seen && r.last_seen && r.after_last >= -turn);
    bench.verdict("within a bit time after it (or a probe turn, the longer)",
                  r.idle_seen && r.last_seen && r.after_last <= (bit > turn ? bit : turn));
}

/// Does a channel's beat into DATA clear TXC, as a CPU write does
/// (31.8.8)? TXC is left standing by one byte sent and drained, a block
/// is started, and TXC is read once the block's first beats have landed
/// and long before its end.
void txc_under_the_engine() {
    loop_live = Loop::txdma;
    (void)LTxDma::init(clock, 115'200u);
    loop_settle(115'200u);
    static const uint8_t one[1] = {0x55};
    static uint8_t block[32];
    (void)LTxDma::write_bulk(std::span<const uint8_t>(one, 1));
    uint32_t spins = 0;
    while (!Sc1::txc_flag() && spins++ < 400'000u) {
    }
    const bool stood = Sc1::txc_flag();
    (void)LTxDma::write_bulk(std::span<const uint8_t>(block, sizeof block));
    const uint32_t t0 = cycles_now();
    while (cycles_now() - t0 < 3u * 10u * bit_cycles(115'200u)) {
    }
    const bool after = Sc1::txc_flag();
    spins = 0;
    while (!LTxDma::tx_idle() && spins++ < 4'000'000u) {
    }
    LTxDma::release();
    loop_live = Loop::none;
    print(plain, "  TXC standing before the block ", stood ? 1u : 0u,
          ", three frames into it ", after ? 1u : 0u, crlf);
    bench.verdict("a channel's beat into DATA clears TXC, as a CPU write does",
                  stood && !after);
}

void tq_tx_idle() {
    txc_under_the_engine();
    constexpr uint32_t n = 48;
    for (const uint32_t baud : {115'200u, 1'000'000u}) {
        loop_live = Loop::plain;
        const IdleProbe r = probe_idle<LPlain>(
            baud, n, [] { return (Sc1::armed() & SercomFlag::dre) == 0u; });
        LPlain::release();
        loop_live = Loop::none;
        report_idle("plain transport", baud, r);
        judge_idle(baud, r);
    }
    for (const uint32_t baud : {115'200u, 1'000'000u, 3'000'000u}) {
        loop_live = Loop::txdma;
        // SERCOM1's vector feeds nothing here: its line is held off at once.
        const IdleProbe r = probe_idle<LTxDma>(baud, n, [] { return true; });
        LTxDma::release();
        loop_live = Loop::none;
        report_idle("transmit engine", baud, r);
        judge_idle(baud, r);
    }
}

// ---- r - the interrupt receiver's edge and its skip --------------------------

/// A burst of `n` bytes of the generator `s`, sent polled at the wire's
/// rate into the loop while nothing drains the ring - the receiver is
/// LPlain's interrupt alone - and waited out to the last stop bit and a
/// few frames more, so the vector has taken it all. The bytes sent.
uint32_t loop_burst(uint32_t baud, uint32_t& s, uint32_t n) {
    uint32_t sent = 0;
    uint32_t spins = 0;
    uint8_t next = lfsr_next(s);
    while (sent < n && spins++ < 20'000'000u) {
        if (loop_send_polled(next)) {
            ++sent;
            if (sent < n) {
                next = lfsr_next(s);
            }
        }
    }
    spins = 0;
    while (!Sc1::txc_flag() && spins++ < 400'000u) {
    }
    loop_settle(baud);
    return sent;
}

/// Every run the ring hands out, checked against the generator `s`.
struct Taken {
    uint32_t got, bad;
};
Taken loop_take(uint32_t& s) {
    Taken t{};
    for (;;) {
        const auto run = LPlain::read_span();
        if (run.empty()) {
            break;
        }
        for (const uint8_t b : run) {
            t.bad += b != lfsr_next(s) ? 1u : 0u;
        }
        t.got += static_cast<uint32_t>(run.size());
        LPlain::consume(static_cast<uint32_t>(run.size()));
    }
    return t;
}

void tr_receiver_edge() {
    // THE EDGE: the vector's return is the ring's empty -> non-empty across
    // an entry, so a burst into an empty ring that nobody drains is ONE
    // edge however long it is, and the next burst after the drain one more.
    constexpr uint32_t n = 256;
    for (const uint32_t baud : {1'000'000u, 3'000'000u}) {
        loop_live = Loop::plain;
        (void)LPlain::init(clock, baud);
        loop_settle(baud);
        const uint32_t skips0 = LPlain::rx_skips();
        uint32_t s_tx = lfsr_seed;
        uint32_t s_rx = lfsr_seed;
        const uint32_t e0 = loop_edges;
        uint32_t sent = loop_burst(baud, s_tx, n);
        const uint32_t e1 = loop_edges;
        Taken t = loop_take(s_rx);
        sent += loop_burst(baud, s_tx, n);
        const uint32_t e2 = loop_edges;
        const Taken t2 = loop_take(s_rx);
        t.got += t2.got;
        t.bad += t2.bad;
        const uint8_t hw = LPlain::hw_overruns();
        const uint8_t over = LPlain::rx_overruns();
        const uint32_t skips = (LPlain::rx_skips() - skips0) & 0xFFu;
        LPlain::release();
        loop_live = Loop::none;
        print(plain, "  ", baud, " baud, two bursts of ", n, ": sent ", sent, ", received ", t.got,
              ", wrong ", t.bad, ", edges ", e1 - e0, " then ", e2 - e1, ", hw_overrun ", hw,
              ", rx_overrun ", over, ", skips ", skips, crlf);
        bench.verdict("both bursts crossed the loop, byte-exact and in order",
                      sent == 2u * n && t.got == 2u * n && t.bad == 0u);
        bench.verdict("a burst into an empty ring is ONE edge from the vector",
                      e1 - e0 == 1u && e2 - e1 == 1u);
        bench.verdict("nothing lost, the skip epoch still",
                      hw == 0u && over == 0u && skips == 0u);
    }

    // THE CONSUMER HELD BACK past the ring (512: 511 held): 128 bytes taken
    // as they come, then 700 sent with nobody draining - 511 held, 189
    // dropped on the full ring, each counted and told to the ring - then
    // the consumer's first look, then 128 more taken as they come.
    constexpr uint32_t baud = 1'000'000u;
    constexpr uint32_t before = 128;
    constexpr uint32_t held = 700;
    constexpr uint32_t after = 128;
    constexpr uint32_t holds = 511;
    loop_live = Loop::plain;
    (void)LPlain::init(clock, baud);
    loop_settle(baud);
    const uint32_t skips0 = LPlain::rx_skips();
    uint32_t s_tx = lfsr_seed;
    uint32_t s_rx = lfsr_seed;
    (void)loop_burst(baud, s_tx, before);
    const Taken a = loop_take(s_rx);
    const uint8_t over0 = LPlain::rx_overruns();
    (void)loop_burst(baud, s_tx, held);
    const uint32_t pending = static_cast<uint32_t>(LPlain::rx_pending());
    const uint32_t dropped = static_cast<uint8_t>(LPlain::rx_overruns() - over0);
    const uint32_t skips_held = (LPlain::rx_skips() - skips0) & 0xFFu;
    // The look: the drop stands, so the ring skips to its head - the 511
    // it held are behind the drop and are never handed out.
    const auto look = LPlain::read_span();
    const bool look_empty = look.empty();
    const uint32_t skips_look = (LPlain::rx_skips() - skips0) & 0xFFu;
    for (uint32_t i = 0; i < held; ++i) {
        (void)lfsr_next(s_rx);   // what the consumer will never see
    }
    (void)loop_burst(baud, s_tx, after);
    const Taken c = loop_take(s_rx);
    const uint32_t skips_end = (LPlain::rx_skips() - skips0) & 0xFFu;
    const uint8_t hw = LPlain::hw_overruns();
    LPlain::release();
    loop_live = Loop::none;
    print(plain, "  ", baud, " baud, the consumer held back: before ", a.got, " (wrong ", a.bad,
          "); held ", pending, " of ", held, ", rx_overrun ", dropped, ", skips ", skips_held,
          "; the first look ", look_empty ? "empty" : "NOT EMPTY", ", skips ", skips_look,
          "; after ", c.got, " (wrong ", c.bad, "), skips ", skips_end, ", hw_overrun ", hw, crlf);
    bench.verdict("before the hold: every byte, in order", a.got == before && a.bad == 0u);
    bench.verdict("the ring held its 511 and counted the rest lost",
                  pending == holds && dropped == held - holds && hw == 0u);
    bench.verdict("the loss waits for the consumer: no skip before it looks", skips_held == 0u);
    bench.verdict("its first look skips: nothing the ring held across the loss handed out",
                  look_empty && skips_look == 1u);
    bench.verdict("the release recovers: the next burst whole, in order, the epoch still",
                  c.got == after && c.bad == 0u && skips_end == 1u);
}

// ---- s - errors injected under the interrupt receiver (host) ------------------

/// The host sends `n` bytes of the pattern at 8E1 into a receiver at 8N1
/// (brio stress's `poke`, at half the window): every frame whose parity
/// bit is zero lands that zero where the receiver samples its stop bit,
/// a frame error, its eight data bits intact. The interrupt receiver
/// drops each hit character, counts it and tells the ring, whose next
/// look skips what it holds: the consumer here reads as fast as it can,
/// so most skips find the ring empty - at most n - K delivered, in order,
/// the counter K, one skip a loss at most.
struct ErrorLeg {
    uint32_t received;
    bool in_order;         ///< the received bytes are the pattern's, in order, some skipped
    ErrCounts err;
    uint32_t skips;        ///< how far rx_skips() moved
};

ErrorLeg error_leg(uint32_t n) {
    constexpr uint32_t window_ms = 1400;
    print(plain, "  HOST poke ", static_cast<uint32_t>(Mode::plain), " 115200 8E1 ", window_ms,
          " ", n, crlf);
    (void)drain();
    mode_release();
    (void)mode_init(Mode::plain, 115200);
    spin_ms(120);
    mode_clear_errors();
    const uint32_t skips0 = mode_skips();
    static uint8_t got[300];
    ErrorLeg r{};
    const uint32_t t0 = cycles_now();
    const uint32_t window = ms_cycles(window_ms);
    while (cycles_now() - t0 < window) {
        if (r.received < sizeof got) {
            r.received += mode_read_bulk(got + r.received, sizeof got - r.received);
        }
    }
    r.err = mode_errors();
    r.skips = (mode_skips() - skips0) & 0xFFu;
    back_to_console();
    r.in_order = true;
    uint32_t at = 0;   // pattern position
    uint32_t sy = lfsr_seed;
    for (uint32_t i = 0; i < r.received; ++i) {
        while (at < n && lfsr_next(sy) != got[i]) {
            ++at;
        }
        if (at >= n) {
            r.in_order = false;
            break;
        }
        ++at;
    }
    return r;
}

void ts_errors() {
    constexpr uint32_t n = 256;
    // What a frame-per-character reading predicts: the bytes whose even
    // parity bit is zero (an even count of ones).
    uint32_t s = lfsr_seed;
    uint32_t zero_parity = 0;
    for (uint32_t i = 0; i < n; ++i) {
        const uint8_t b = lfsr_next(s);
        zero_parity += (__builtin_popcount(b) & 1) == 0 ? 1u : 0u;
    }

    const ErrorLeg i = error_leg(n);
    print(plain, "  interrupt receiver: received ", i.received, " of ", n, ", frame ", i.err.frame,
          " parity ", i.err.parity, " hw_overrun ", i.err.hw_overrun, " (", zero_parity,
          " characters carry a zero parity bit)", crlf);
    bench.verdict("the interrupt receiver counted K and delivered n - K at most",
                  i.err.frame != 0u && i.received + i.err.frame <= n);
    print(plain, "  the skip epoch moved ", i.skips, crlf);
    bench.verdict("the skip epoch moved, once a dropped character at most",
                  i.skips != 0u && i.skips <= static_cast<uint32_t>(i.err.frame + i.err.parity));
    bench.verdict("what it delivered is the pattern, in order, the hit ones skipped",
                  i.in_order);
}

// ---- t - the interrupt receiver at the top rate ------------------------------

void tt_levels() {
    constexpr uint32_t baud = 3'000'000u;
    constexpr uint32_t n = 400;
    loop_live = Loop::plain;
    (void)LPlain::init(clock, baud);
    loop_settle(baud);
    uint32_t s_tx = lfsr_seed;
    uint32_t s_rx = lfsr_seed;
    uint32_t sent = 0;
    uint32_t received = 0;
    uint32_t bad = 0;
    uint8_t next = lfsr_next(s_tx);
    uint32_t spins = 0;
    // The ring holds 511: the stream is drained as it comes, between the
    // polled writes, so the sender keeps the wire full.
    while ((sent < n || received < n) && spins++ < 20'000'000u) {
        if (sent < n && loop_send_polled(next)) {
            ++sent;
            next = lfsr_next(s_tx);
        }
        uint8_t b = 0;
        if (LPlain::read_byte(b)) {
            bad += b != lfsr_next(s_rx) ? 1u : 0u;
            ++received;
        }
    }
    const uint8_t hw = LPlain::hw_overruns();
    LPlain::release();
    loop_live = Loop::none;
    print(plain, "  3 Mbaud, polled sender at the wire's rate: sent ", sent, ", received ",
          received, ", wrong ", bad, ", hw_overrun ", hw, crlf);
    bench.verdict("the interrupt receiver keeps a 3 Mbaud stream: every byte, in order",
                  received == n && bad == 0u);
    bench.verdict("no hardware overrun", hw == 0u);
}

// ---- x - the one-user rule as a panic ----------------------------------------

/// THE REFUSAL, AS IT IS ENFORCED: the loop's engined transport holds the
/// claim, and the console's engined transport's init() - a second owner's
/// arm() - must panic (kernel/panic.hpp: the breadcrumb written with
/// PanicCode::assert_failed and dma_claim_refused_context(6) = 0xD6, then
/// the stop with interrupts masked). The proof is read at the next boot,
/// where main() takes the record and the banner prints it. Reached past
/// the init only when the refusal did NOT happen: everything is given
/// back and the verdict is a failure.
void tx_refusal() {
    print(plain, "  the loop's engined transport (SERCOM1, owner ", loop_owner,
          ") takes the claim; then the console's", crlf,
          "  engined init (SERCOM5, owner ", console_owner,
          ") must PANIC: THE BOARD STOPS HERE, interrupts masked.", crlf,
          "  The proof is the breadcrumb at the next boot: the banner after a reset prints", crlf,
          "  code ", static_cast<uint32_t>(PanicCode::assert_failed), " (assert_failed), context ",
          hex(dma_claim_refused_context(console_owner)), crlf);
    (void)drain();
    loop_live = Loop::txdma;
    const bool loop_up = LTxDma::init(clock, 115'200u);
    const bool loop_holds = DmaTxEngine::owner() == loop_owner;
    if (!loop_up || !loop_holds) {
        LTxDma::release();
        loop_live = Loop::none;
        print(plain, "  the loop's engined transport did not come up (init ", loop_up,
              ", owner ", DmaTxEngine::owner(), ")", crlf);
        bench.verdict("the loop's engined transport holds the claim", false);
        return;
    }
    mode_release();
    const bool up = mode_init(Mode::txdma, 115200);
    // Past this line only when the second owner's arm() was not refused.
    const uint8_t owner = DmaTxEngine::owner();
    UTxDma::release();
    LTxDma::release();
    loop_live = Loop::none;
    (void)mode_init(Mode::plain, 115200);
    spin_ms(150);
    print(plain, "  the console's engined init RETURNED (", up, ", owner ", owner,
          "): no panic", crlf);
    bench.verdict("a second engined transport's init() panics while the claim is held", false);
}

// =============================================================================
// The menu
// =============================================================================

/// The previous boot's panic record, taken once at boot (fetch-and-clear).
std::optional<PanicRecord> boot_record;

void print_boot_record() {
    if (!boot_record) {
        print(plain, "  no panic record from the previous boot", crlf);
        return;
    }
    const bool assert_failed =
        boot_record->code == static_cast<uint8_t>(PanicCode::assert_failed);
    print(plain, "  PANIC RECORD from the previous boot: magic ", hex(boot_record->magic),
          " (valid), code ", static_cast<uint32_t>(boot_record->code),
          assert_failed ? " (assert_failed)" : "", ", context ",
          hex(boot_record->context), crlf);
}

void banner() {
    print(plain, crlf,
          "test_samc_uart - SAMC21J18A SERCOM5 USART (ch. 30/31), clk=",
          SysClock::hz, " Hz", crlf,
          "  letters e..p and s need brio stress on the other end; x stops the board", crlf);
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

extern "C" void SERCOM1_Handler() {
    switch (loop_live) {
        case Loop::plain:
            if (LPlain::isr()) {
                loop_edges = loop_edges + 1u;
            }
            break;
        case Loop::txdma: (void)LTxDma::isr(); break;
        default: break;
    }
}

extern "C" void SERCOM5_Handler() {
    if (live == Mode::txdma) {
        (void)UTxDma::isr();
    } else {
        (void)UPlain::isr();
    }
}

// THE ENGINE TELLS ITS OWNER ALONE - see the note beside `live`: the
// completion goes to the transport that holds the claim, and to no other.
extern "C" void DMAC_Handler() {
    const uint8_t owner = brio::DmaTxEngine::owner();
    if (owner == loop_owner) {
        LTxDma::dma_isr();
    } else if (owner == console_owner) {
        UTxDma::dma_isr();
    }
}

int main() {
    // Taken FIRST and once: the record is fetch-and-clear.
    boot_record = brio::take_panic_record<brio::SamPlatform>();

    const bool clock_ok = SysClock::init();
    const bool tick_ok = brio::Ticker::init(clock);

    const bool serial_ok = mode_init(Mode::plain, 115200);
    brio::enable_interrupts();

    bench.letter('a', "the baud generator's arithmetic", ta_baud);
    bench.letter('b', "every frame format, written and read back", tb_format);
    bench.letter('c', "the transmit ring's contract under pressure", tc_ring);
    bench.letter('d', "the transmit engine's facts and its claim", td_engine);

    bench.letter('e', "echo, plain transport (host)", te_plain_echo, false);
    bench.letter('f', "echo, DMA transmitter (host)", tf_txdma_echo, false);
    bench.letter('h', "echo at 1 and 2 Mbaud, DMA TX beside irq RX (host)",
                 th_duplex_echo, false);
    bench.letter('i', "receive-only sustained (host)", ti_sink, false);
    bench.letter('j', "transmit-only sustained (host)", tj_source, false);
    bench.letter('k', "115200 / 1 M / 3 Mbaud (host)", tk_rates, false);
    bench.letter('l', "the frame-format matrix (host)", tl_formats, false);
    bench.letter('m', "a MISMATCHED frame, and the recovery (host)", tm_mismatch,
                 false);
    bench.letter('n', "ring pressure: an eager consumer and a lazy one (host)", tn_pressure,
                 false);
    bench.letter('p', "bursty traffic with idle gaps (host)", tp_burst, false);
    bench.letter('q', "tx_idle() on the loop: the wire's idle", tq_tx_idle);
    bench.letter('r', "the interrupt receiver on the loop: its edge and its skip",
                 tr_receiver_edge);
    bench.letter('s', "errors under the interrupt receiver (host)", ts_errors, false);
    bench.letter('t', "the interrupt receiver at 3 Mbaud on the loop", tt_levels);
    bench.letter('x', "the one-user rule as a PANIC (stops the board)", tx_refusal, false);

    if (serial_ok) {
        print(plain, crlf, "boot: clk=", clock_ok ? "OSC48M" : "FAILED",
              " tick=", tick_ok ? "SysTick" : "FAILED", crlf);
        print_boot_record();
        banner();
    }
    bench.prompt();

    for (;;) {
        uint8_t c = 0;
        if (!UPlain::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        print(plain, static_cast<char>(c), crlf);
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(plain, "unknown letter (? for the menu)", crlf);
        }
        bench.prompt();
    }
}
