// test_samc_uart - the SERCOM USART transport (DS60001479M ch. 30/31),
// all three shapes of it, byte for byte.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over a
// serial console; z runs the self-contained ones and prints the ALL: line
// bin/brio judges.
//
// WHAT THIS SUITE IS FOR. samc21/sercom.hpp's Uart can be built in three
// shapes - interrupt on both directions, or DMA on ONE of them - and what
// is checked here is the BYTES through each of them, on both sides at
// once, with a pattern in which a lost, duplicated or reordered byte is
// IDENTIFIED and not merely counted. The fourth shape, DMA on both, is a
// compile error (erratum 1.10.4, uart_engines_not_concurrent()): letter
// d states the refusal as a value, and letter h carries the duplex link
// the refusal leaves at a rate where the choice matters.
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
// encoding, the ring contract, the engines' register facts, and on the
// loop the wire's idle, the receive engine's edge and the level loop at
// the generator's top rate.
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
// console and are driven by brio stress:
//   a  the baud generator's arithmetic
//   b  every frame format, written and read back
//   c  the transmit ring's contract under pressure
//   d  the engines' compile-time and register facts
//   e  echo through the plain interrupt transport
//   f  echo with the DMA transmitter
//   g  echo with the DMA receiver
//   h  echo at 1 and 2 Mbaud: the DMA transmitter beside the interrupt
//      receiver, the duplex shape erratum 1.10.4 leaves
//   i  receive-only, sustained
//   j  transmit-only, sustained
//   k  the same traffic at 115200, 1 M and 3 Mbaud
//   l  the frame-format matrix, host-driven
//   m  a MISMATCHED frame and the recovery from it
//   n  ring pressure: eager against lazy harvests
//   p  bursty traffic with idle gaps
//   q  tx_idle() on the loop: never before the last stop bit, within a bit
//      after it, for the plain transport and the transmit engine
//   r  the receive engine on the loop: the edge from the vector at each
//      half of the ring, the block boundary under a stream at the wire's
//      rate, the tail by the owner's ask; the owner asking at every turn;
//      a consumer held back past the ring - the stall, the loss counted in
//      the skip epoch before the bytes after it are visible, the re-arm by
//      the consumer's release
//   s  errors injected under the receive engine and the interrupt
//      receiver (host): no byte taken to clear an error
//   t  the interrupt receiver at the generator's top rate on the loop:
//      every level an entry
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>

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

// 512 each way: the receive engine's blocks are its halves, 256 bytes,
// and three instantiations - with the loop's three below - fit
// comfortably in 32 KB of SRAM.
constexpr uint32_t rx_ring = 512;
constexpr uint32_t tx_ring = 512;

constexpr uint8_t ch_tx = 6;
constexpr uint8_t ch_rx = 7;

using UPlain = Uart<5, console_pads, rx_ring, tx_ring>;
using UTxDma = Uart<5, console_pads, rx_ring, tx_ring, DmaTxEngine<ch_tx>, NoDmaEngine>;
using URxDma = Uart<5, console_pads, rx_ring, tx_ring, NoDmaEngine, DmaRxEngine<ch_rx>>;
// The fourth shape, both engines, does not compile: erratum 1.10.4.

constexpr UPlain plain;

using Sc5 = UPlain::Resource;
using Led = Pin<'B', 23>;

TestBench<UPlain, 20> bench;

using brio::crlf;
using brio::print;

// =============================================================================
// The three transports behind one set of verbs
// =============================================================================
//
// THE THREE INSTANTIATIONS ARE NEVER LIVE AT ONCE. Each has its own rings
// and counters, and init() resets and reconfigures SERCOM5 from scratch,
// so switching is a clean handover. Two of them name the same DMA
// channel, and an engine is a set of STATIC members - one object per
// channel, shared by every Uart that names it - so the interrupt handler
// must tell exactly ONE of them about a completion. Telling two calls
// DmaTxEngine::complete() twice: the second call clears `busy_` while the
// block the first one just started is still in flight, and the next
// pump_tx() reprograms a RUNNING channel. That cost this suite an hour
// and three letters' worth of false evidence, which is why `live` gates
// the dispatch in DMAC_Handler().

// The numbers are brio stress's protocol (its mode 3, both engines, is
// a shape this transport refuses).
enum class Mode : uint8_t { plain = 0, txdma = 1, rxdma = 2 };
Mode live = Mode::plain;
/// The receive edges the engine's vector returned (dma_isr()) in a leg.
volatile uint32_t rx_vector_edges = 0;

const char* mode_name(Mode m) {
    switch (m) {
        case Mode::plain: return "irqTX+irqRX";
        case Mode::txdma: return "dmaTX+irqRX";
        default: return "irqTX+dmaRX";
    }
}

bool mode_init(Mode m, uint32_t baud, const UartFormat& fmt = {}) {
    live = m;
    switch (m) {
        case Mode::plain: return UPlain::init(clock, baud, fmt);
        case Mode::txdma: return UTxDma::init(clock, baud, fmt);
        default: return URxDma::init(clock, baud, fmt);
    }
}

void mode_release() {
    switch (live) {
        case Mode::plain: UPlain::release(); break;
        case Mode::txdma: UTxDma::release(); break;
        default: URxDma::release(); break;
    }
}

bool mode_tx_idle() {
    switch (live) {
        case Mode::plain: return UPlain::tx_idle();
        case Mode::txdma: return UTxDma::tx_idle();
        default: return URxDma::tx_idle();
    }
}

uint32_t mode_write_bulk(const uint8_t* p, uint32_t n) {
    const std::span<const uint8_t> s(p, n);
    switch (live) {
        case Mode::plain: return UPlain::write_bulk(s);
        case Mode::txdma: return UTxDma::write_bulk(s);
        default: return URxDma::write_bulk(s);
    }
}

uint32_t mode_read_bulk(uint8_t* p, uint32_t n) {
    const std::span<uint8_t> s(p, n);
    switch (live) {
        case Mode::plain: return UPlain::read_bulk(s);
        case Mode::txdma: return UTxDma::read_bulk(s);
        default: return URxDma::read_bulk(s);
    }
}



void mode_harvest() {
    switch (live) {
        case Mode::rxdma: (void)URxDma::harvest(); break;
        default: break;
    }
}

/// An EMPTY bulk write: it queues nothing and returns zero, but it takes
/// sercom.hpp's blocked-transmitter path on the way out - which is how a
/// loop that is WAITING for the ring to drain (rather than filling it)
/// still gives the transport its push, and its repair.
void mode_nudge() { (void)mode_write_bulk(nullptr, 0); }

void mode_clear_errors() {
    switch (live) {
        case Mode::plain: UPlain::clear_errors(); break;
        case Mode::txdma: UTxDma::clear_errors(); break;
        default: URxDma::clear_errors(); break;
    }
}

struct ErrCounts {
    uint8_t rx_overrun, frame, parity, hw_overrun, dma_faults;
};

/// The skip epoch of the live transport (never cleared: a leg reads its
/// difference).
uint32_t mode_skips() {
    switch (live) {
        case Mode::plain: return UPlain::rx_skips();
        case Mode::txdma: return UTxDma::rx_skips();
        default: return URxDma::rx_skips();
    }
}

ErrCounts mode_errors() {
    switch (live) {
        case Mode::plain:
            return {UPlain::rx_overruns(), UPlain::frame_errors(),
                    UPlain::parity_errors(), UPlain::hw_overruns(),
                    UPlain::dma_faults()};
        case Mode::txdma:
            return {UTxDma::rx_overruns(), UTxDma::frame_errors(),
                    UTxDma::parity_errors(), UTxDma::hw_overruns(),
                    UTxDma::dma_faults()};
        default:
            return {URxDma::rx_overruns(), URxDma::frame_errors(),
                    URxDma::parity_errors(), URxDma::hw_overruns(),
                    URxDma::dma_faults()};
    }
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

void spin_ms(uint32_t ms) {
    const uint32_t t0 = cycles_now();
    const uint32_t want = (SysClock::hz / 1000u) * ms;
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
// Raw observation of the two engines
// =============================================================================
//
// TAKEN while the transport under test is still live, PRINTED afterwards
// through the console: the state that matters is the state at the end of
// the window, and printing it out of the transport being measured would
// perturb the very thing being read.

struct EngineSnapshot {
    uint8_t sercom_flags, sercom_armed;
    uint16_t sercom_status;
    uint32_t busych, pendch;
    bool tx_enabled, rx_enabled;
    uint8_t tx_status, rx_status;
    DmaDescriptor tx_loaded, rx_loaded, tx_wb, rx_wb;
    bool tx_own, rx_own;
    uint32_t tx_violations, rx_violations, rx_timeouts, tx_engine_faults;
    bool tx_busy;
};

EngineSnapshot snapshot() {
    EngineSnapshot s{};
    s.sercom_flags = Sc5::flags();
    s.sercom_armed = Sc5::armed();
    s.sercom_status = Sc5::status();
    s.busych = Dmac::busy_channels();
    s.pendch = Dmac::pending_channels();
    s.tx_enabled = DmaChannel<ch_tx>::enabled();
    s.rx_enabled = DmaChannel<ch_rx>::enabled();
    s.tx_status = DmaChannel<ch_tx>::status();
    s.rx_status = DmaChannel<ch_rx>::status();
    s.tx_loaded = DmaChannel<ch_tx>::loaded();
    s.rx_loaded = DmaChannel<ch_rx>::loaded();
    s.tx_wb = Dmac::read_write_back(ch_tx);
    s.rx_wb = Dmac::read_write_back(ch_rx);
    s.tx_own = DmaChannel<ch_tx>::write_back_ok();
    s.rx_own = DmaChannel<ch_rx>::write_back_ok();
    s.tx_violations = DmaChannel<ch_tx>::violations();
    s.rx_violations = DmaChannel<ch_rx>::violations();
    s.rx_timeouts = DmaChannel<ch_rx>::suspend_timeouts();
    s.tx_engine_faults = DmaTxEngine<ch_tx>::faults();
    s.tx_busy = DmaTxEngine<ch_tx>::busy();
    return s;
}

void print_descriptor(const char* what, const DmaDescriptor& d) {
    print(plain, "        ", what, " ctrl=", hex(d.btctrl), " cnt=", d.btcnt,
          " src=", hex(d.srcaddr), " dst=", hex(d.dstaddr), crlf);
}

void dump_engines(const EngineSnapshot& s) {
    print(plain, "  sercom intflag=", s.sercom_flags, " intenset=", s.sercom_armed,
          " status=", s.sercom_status, " | dmac busych=", s.busych,
          " pendch=", s.pendch, crlf);
    print(plain, "  ch", ch_tx, " (tx) enabled=", s.tx_enabled ? 1u : 0u,
          " chstatus=", s.tx_status, " write-back own=", s.tx_own ? "yes" : "no",
          " blocks abandoned=", s.tx_engine_faults, crlf);
    print_descriptor("loaded", s.tx_loaded);
    print_descriptor("wrback", s.tx_wb);
    print(plain, "  ch", ch_rx, " (rx) enabled=", s.rx_enabled ? 1u : 0u,
          " chstatus=", s.rx_status, " write-back own=", s.rx_own ? "yes" : "no",
          crlf);
    print_descriptor("loaded", s.rx_loaded);
    print_descriptor("wrback", s.rx_wb);
    print(plain, "  refused readings tx=", s.tx_violations, " rx=", s.rx_violations,
          " rx suspend timeouts=", s.rx_timeouts, crlf);
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
    /// How often the owner asks the RX engine for the TAIL. The run is
    /// published by the engine's vector at every filled half of the ring
    /// (dma_isr()); what no completion reports - the tail of a stream that
    /// stops short of a block's end - is the owner's ask, and a lazy one:
    /// letter n measures what an eager cadence costs.
    uint32_t harvest_us = 2000;

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
    uint32_t vector_edges;   ///< the receive engine's edges from its vector
    uint32_t asks;           ///< the owner's harvest() calls
    uint32_t skips;          ///< how far rx_skips() moved over the window
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

/// One host-driven leg, start to finish (see the choreography at the top).
LegResult run_leg(const Leg& leg) {
    LegResult r{};

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
    const uint32_t skips0 = mode_skips();
    rx_vector_edges = 0;
    DmaChannel<ch_tx>::clear_counters();
    DmaChannel<ch_rx>::clear_counters();
    DmaTxEngine<ch_tx>::clear_faults();

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
    const uint32_t window = (SysClock::hz / 1000u) * leg.window_ms;
    const uint32_t harvest_gap = (SysClock::hz / 1'000'000u) * leg.harvest_us;
    uint32_t next_harvest = t0;
    uint32_t turns = 0;
    uint8_t out[128];
    uint32_t out_have = 0;
    uint32_t out_done = 0;

    while (cycles_now() - t0 < window) {
        const uint32_t now = cycles_now();
        if (static_cast<int32_t>(now - next_harvest) >= 0) {
            mode_harvest();
            if (leg.mode == Mode::rxdma) {
                ++r.asks;
            }
            next_harvest = now + harvest_gap;
        }

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
    // The interrupt receiver's epoch is its SkipRing's, modulo 2^8.
    r.skips = leg.mode != Mode::rxdma ? (mode_skips() - skips0) & 0xFFu : mode_skips() - skips0;
    r.vector_edges = rx_vector_edges;
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
    if (leg.mode == Mode::rxdma) {
        print(plain, "     the engine's edges from its vector ", r.vector_edges,
              ", the owner's asks ", r.asks, crlf);
    }
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

void td_engines() {
    bench.verdict("the plain transport names no engine",
                  !UPlain::has_tx_engine && !UPlain::has_rx_engine);
    bench.verdict("the TX-only one names exactly one",
                  UTxDma::has_tx_engine && !UTxDma::has_rx_engine);
    bench.verdict("the RX-only one names the other",
                  !URxDma::has_tx_engine && URxDma::has_rx_engine);
    // THE FOURTH SHAPE IS A COMPILE ERROR, so what the board can show of
    // it is the predicate the Uart's static_assert asks: two engines on
    // one transport are two channels triggered concurrently, erratum
    // 1.10.4's condition, and the pair is refused at the template
    // argument (test/family_samc21/neg/uart_both_engines.cpp fails to
    // compile for that reason).
    bench.verdict("both engines on one transport are refused (erratum 1.10.4)",
                  !uart_engines_not_concurrent<DmaTxEngine<ch_tx>, DmaRxEngine<ch_rx>>() &&
                      uart_engines_not_concurrent<DmaTxEngine<ch_tx>, NoDmaEngine>() &&
                      uart_engines_not_concurrent<NoDmaEngine, DmaRxEngine<ch_rx>>());
    bench.verdict("an engineless transport reports no DMA faults, for free",
                  UPlain::dma_faults() == 0);

    // The trigger codes are the device header's, read the same way from
    // both sides of the wall: samc21/sercom.hpp names them per instance,
    // samc21/dmac.hpp spells the same table from the DMAC's side.
    bench.verdict("SERCOM5's RX trigger code is the header's",
                  Sc5::dma_rx_trigger() == SERCOM5_DMAC_ID_RX);
    bench.verdict("SERCOM5's TX trigger code is the header's",
                  Sc5::dma_tx_trigger() == SERCOM5_DMAC_ID_TX);
    bench.verdict("and dmac.hpp's own table agrees",
                  dma_trigger_sercom_rx<5>() == Sc5::dma_rx_trigger() &&
                      dma_trigger_sercom_tx<5>() == Sc5::dma_tx_trigger());

    // A channel has exactly ONE pending-trigger bit: a software trigger
    // that races a real one still pending is LOST (25.8.8), and the
    // register says so about itself - it reads back set exactly when a
    // trigger was lost. (One that lands after a beat has started is a
    // second beat, which is why the transport kicks neither direction:
    // docs/samc21/sercom.md.)
    DmaChannel<ch_tx>::clear_trigger_lost();
    bench.verdict("no trigger is recorded lost on an idle channel",
                  !DmaChannel<ch_tx>::trigger_lost());
}

// =============================================================================
// The host letters
// =============================================================================

void run_and_report(const Leg& leg, bool lossless) {
    const LegResult r = run_leg(leg);
    const EngineSnapshot s = snapshot();
    back_to_console();
    report(leg, r);
    dump_engines(s);

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
                      !s.tx_busy);
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
/// The receive engine publishes from its vector and re-arms in the same
/// handler, so the block boundary is no gap: lossless, as the interrupt
/// receiver is.
void tg_rxdma_echo() { run_and_report(base_leg(Op::echo, Mode::rxdma), true); }
/// THE DUPLEX LINK THE ERRATUM LEAVES. Both engines on one SERCOM are
/// refused (erratum 1.10.4), so a link that wants bulk both ways takes
/// the engine on the transmit side and keeps the receiver on RXC. Two
/// rates, two claims:
///  - at 1 Mbaud the shape is LOSSLESS - every byte back, in order,
///    nothing dropped, no hardware overrun;
///  - at 2 Mbaud the receiver takes every level an entry and keeps the
///    stream but for a character or three when another handler holds it
///    past two frames (the transmit engine's completion and the tick
///    together), and the echo gains on the board's slow clock (below):
///    loss is allowed and SILENCE is not, as in letter k.
/// At both, NO TRANSMIT BLOCK MAY BE ABANDONED: the engine's channel is
/// the only one in this image, so erratum 1.10.4 has nothing to corrupt
/// it with, and an abandoned block is the dead-block predicate firing on
/// a live one - which a test of the engine's own "in flight" and the
/// flags alone did at 2 Mbaud, where main context is preempted for a
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
        dump_engines(s);
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

void tn_pressure() {
    // THE OWNER'S ASK, both extremes, through the transport that has the
    // most to lose. The engine's vector publishes every filled half of the
    // ring and re-arms; the owner's harvest() is for the tail. A lazy ask
    // (2 ms) is the shape letter g runs; an eager one (50 us) asks some
    // four hundred times a block and races the completion at every boundary.
    // Both must be byte-exact, with no loss and the skip epoch still, and
    // neither may wedge.
    static constexpr uint32_t cadences[] = {50, 2000};
    for (uint32_t us : cadences) {
        Leg leg = base_leg(Op::echo, Mode::rxdma);
        leg.harvest_us = us;
        leg.window_ms = 900;
        const LegResult r = run_leg(leg);
        const EngineSnapshot s = snapshot();
        back_to_console();
        print(plain, "  harvest every ", us, " us:", crlf);
        report(leg, r);
        bench.verdict("the transport drained, whatever the cadence", r.drained);
        bench.verdict("the transmitter is not left claiming a dead block",
                      !s.tx_busy);
        bench.verdict("bytes crossed", r.received != 0u);
        bench.verdict("byte-exact, whatever the cadence", r.first_bad == 0);
        bench.verdict("nothing lost: no overrun, the skip epoch still",
                      r.err.hw_overrun == 0u && r.err.rx_overrun == 0u && r.skips == 0u);
    }
}

void tp_burst() {
    // Bursty traffic with idle gaps - the shape a console really sees,
    // and the one an RX engine is worst at: a burst that stops short of
    // a block's end has no edge on this silicon (no idle detector, no
    // receiver time-out), so the owner's ask carries the tail's latency.
    // The bytes are all there either way.
    Leg leg = base_leg(Op::burst, Mode::rxdma);
    leg.window_ms = 1400;
    run_and_report(leg, true);
}


// =============================================================================
// The loop: SERCOM1 listening on its own transmitter's pad
// =============================================================================
//
// 31.6.3.8: RXPO and TXPO naming one pad put the receiver on the
// transmitter's own signal, through the pad. PAD[0] of SERCOM1 is PA16
// under function C; nothing is wired to it. Three transports over it,
// never live at once (`loop_live`), each on channels of its own.

constexpr UartPads loop_pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad0,
    .tx_pin = {'A', 16, PinFunction::c},
    .rx_pin = {'A', 16, PinFunction::c},
};
constexpr uint8_t ch_loop_tx = 8;
constexpr uint8_t ch_loop_rx = 9;

using LPlain = Uart<1, loop_pads, 512, 512>;
using LTxDma = Uart<1, loop_pads, 512, 512, DmaTxEngine<ch_loop_tx>, NoDmaEngine>;
/// A 1024 ring: the engine's blocks are its halves, 512 bytes.
using LRxDma = Uart<1, loop_pads, 1024, 512, NoDmaEngine, DmaRxEngine<ch_loop_rx>>;
using Sc1 = LPlain::Resource;

enum class Loop : uint8_t { none, plain, txdma, rxdma };
volatile Loop loop_live = Loop::none;
/// The receive edges the loop's vectors returned.
volatile uint32_t loop_edges = 0;

uint32_t bit_cycles(uint32_t baud) { return SysClock::hz / baud; }

/// A few frames of idle line after the transport comes up.
void loop_settle(uint32_t baud) {
    const uint32_t t0 = cycles_now();
    while (cycles_now() - t0 < 40u * bit_cycles(baud)) {
    }
}

/// Polled transmission through the resource, for a sender at the wire's
/// own rate beside a receive engine (the pair of engines is refused):
/// DATA written whenever DRE stands (31.6.2.5).
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

// ---- r - the receive engine's edge from the vector -------------------------

/// A stream of `n` bytes sent polled at the wire's rate into the receive
/// engine, in one of three shapes of its consumer:
///
///   edge   on every edge the vector returns the thread drains the ring,
///          checking each byte against the pattern - and sends nothing
///          while it does, so the line pauses but never a block boundary
///          goes unserved by the handler;
///   eager  the owner ASKS at every turn of the sending loop (harvest(),
///          the run checked as it is published): the ask racing the
///          completion at every block boundary, the shape an eager clock
///          lost characters in;
///   held   the consumer drains NOTHING until the sender is done, so the
///          ring fills, the engine stalls and the receiver overflows in
///          the hardware; the drain then checks the ring's bytes, the
///          consumer's release re-arms the engine, and the skip epoch is
///          read the moment the first character published after the loss
///          is visible.
///
/// The tail - what no block completion reports - is the owner's ask at
/// the end.
enum class Consumer : uint8_t { edge, eager, held };

struct EngineStream {
    uint32_t sent, received, bad, edges, asks;
    uint8_t hw_overruns, rx_overruns;
    uint32_t skips;        ///< rx_skips() moved by this much over the stream
    bool skip_first;       ///< (held) it had moved when the bytes after the loss became visible
    uint32_t held_exact;   ///< (held) the ring's bytes before the loss, equal to the pattern
};

EngineStream engine_stream(uint32_t baud, uint32_t n, Consumer how = Consumer::edge) {
    EngineStream r{};
    loop_live = Loop::rxdma;
    (void)LRxDma::init(clock, baud);
    loop_settle(baud);
    loop_edges = 0;
    const uint32_t skips0 = LRxDma::rx_skips();
    uint32_t s_tx = lfsr_seed;
    uint32_t s_rx = lfsr_seed;
    uint32_t seen = 0;
    const auto drain_ring = [&] {
        for (;;) {
            const auto run = LRxDma::read_span();
            if (run.empty()) {
                break;
            }
            for (const uint8_t b : run) {
                r.bad += b != lfsr_next(s_rx) ? 1u : 0u;
            }
            r.received += static_cast<uint32_t>(run.size());
            LRxDma::consume(static_cast<uint32_t>(run.size()));
        }
    };
    uint8_t next = lfsr_next(s_tx);
    uint32_t spins = 0;
    while (r.sent < n && spins++ < 20'000'000u) {
        if (loop_send_polled(next)) {
            ++r.sent;
            next = lfsr_next(s_tx);
        }
        if (how == Consumer::eager) {
            ++r.asks;
            (void)LRxDma::harvest();
            drain_ring();
        } else if (how == Consumer::edge && loop_edges != seen) {
            seen = loop_edges;
            drain_ring();
        }
    }
    if (how == Consumer::held) {
        // The ring as the stall left it: every byte of it the pattern's
        // first ones. Then the release re-arms; whatever is published
        // after it follows the loss, and the epoch must already have moved.
        spins = 0;
        while (!Sc1::txc_flag() && spins++ < 400'000u) {
        }
        for (;;) {
            const auto run = LRxDma::read_span();
            if (run.empty()) {
                break;
            }
            for (const uint8_t b : run) {
                r.held_exact += b == lfsr_next(s_rx) ? 1u : 0u;
            }
            r.received += static_cast<uint32_t>(run.size());
            LRxDma::consume(static_cast<uint32_t>(run.size()));
        }
        const uint32_t t1 = cycles_now();
        while (cycles_now() - t1 < 20u * 10u * bit_cycles(baud)) {
        }
        ++r.asks;
        (void)LRxDma::harvest();
        const auto after = LRxDma::read_span();
        r.skip_first = !after.empty() && LRxDma::rx_skips() != skips0;
        r.received += static_cast<uint32_t>(after.size());
        LRxDma::consume(static_cast<uint32_t>(after.size()));
    }
    spins = 0;
    while (!Sc1::txc_flag() && spins++ < 400'000u) {
    }
    // The tail: the stream stopped short of a block's end, and this
    // silicon reports no idle line - the owner asks.
    const uint32_t t0 = cycles_now();
    while (cycles_now() - t0 < 20u * 10u * bit_cycles(baud)) {
    }
    ++r.asks;
    (void)LRxDma::harvest();
    if (how != Consumer::held) {
        drain_ring();
    }
    r.edges = loop_edges;
    r.hw_overruns = LRxDma::hw_overruns();
    r.rx_overruns = LRxDma::rx_overruns();
    r.skips = LRxDma::rx_skips() - skips0;
    LRxDma::release();
    loop_live = Loop::none;
    return r;
}

void tr_engine_edge() {
    constexpr uint32_t n = 1300;   // two blocks of 512 and a tail of 276
    for (const uint32_t baud : {1'000'000u, 3'000'000u}) {
        const EngineStream r = engine_stream(baud, n);
        print(plain, "  ", baud, " baud: sent ", r.sent, ", received ", r.received, ", wrong ",
              r.bad, ", edges from the vector ", r.edges, ", asks ", r.asks, ", hw_overrun ",
              r.hw_overruns, ", rx_overrun ", r.rx_overruns, ", skips ", r.skips, crlf);
        bench.verdict("the whole stream crossed the loop", r.sent == n && r.received == n);
        bench.verdict("byte-exact and in order", r.bad == 0u);
        bench.verdict("each filled half of the ring was an edge from the vector",
                      r.edges >= n / 512u);
        bench.verdict("no byte lost at a block boundary (re-armed in the handler)",
                      r.hw_overruns == 0u && r.rx_overruns == 0u);
        bench.verdict("the skip epoch did not move", r.skips == 0u);
    }
    // THE ASK AT ANY CADENCE: harvest() at every turn of the sending loop,
    // racing the completion at both block boundaries.
    for (const uint32_t baud : {1'000'000u, 3'000'000u}) {
        const EngineStream r = engine_stream(baud, n, Consumer::eager);
        print(plain, "  ", baud, " baud, the owner asking at every turn: received ", r.received,
              ", wrong ", r.bad, ", asks ", r.asks, ", edges from the vector ", r.edges,
              ", hw_overrun ", r.hw_overruns, ", skips ", r.skips, crlf);
        bench.verdict("asked at every turn, the whole stream crossed", r.received == n);
        bench.verdict("byte-exact and in order", r.bad == 0u);
        bench.verdict("no loss, and the skip epoch did not move",
                      r.hw_overruns == 0u && r.skips == 0u);
    }
    // THE CONSUMER HELD BACK past the ring (1024 bytes): the engine stalls,
    // the receiver overflows, the loss is counted before the first
    // character after it is visible, and the release re-arms.
    {
        constexpr uint32_t baud = 1'000'000u;
        const EngineStream r = engine_stream(baud, n, Consumer::held);
        print(plain, "  ", baud, " baud, the consumer held back: received ", r.received,
              " (the ring's ", r.held_exact, " equal to the pattern), rx_overrun ",
              r.rx_overruns, ", hw_overrun ", r.hw_overruns, ", skips ", r.skips, crlf);
        bench.verdict("the ring held the stream's first 1024 bytes", r.held_exact == 1024u);
        bench.verdict("the stall was counted (rx_overruns) and the overflow (hw_overruns)",
                      r.rx_overruns == 1u && r.hw_overruns != 0u);
        bench.verdict("the consumer's release re-armed: characters published after it",
                      r.received > 1024u);
        bench.verdict("the skip epoch had moved when they became visible", r.skip_first);
    }
}


// ---- s - errors injected under the receive engine (host) ---------------------

/// The host sends `n` bytes of the pattern at 8E1 into a receiver at 8N1
/// (brio stress's `poke`, at half the window): every frame whose parity
/// bit is zero lands that zero where the receiver samples its stop bit,
/// a frame error, its eight data bits intact. Under the RECEIVE ENGINE
/// the channel reads every character - an error is counted from STATUS,
/// which clears by being written (31.8.9), never by a read of DATA - so
/// all n bytes arrive, byte-exact, and the count is the engine's
/// granularity: one per run that saw errors. Under the INTERRUPT RECEIVER
/// each hit character is dropped, counted and told to the ring, whose next
/// look skips what it holds: the consumer here reads as fast as it can, so
/// most skips find the ring empty - at most n - K delivered, in order, the
/// counter K, one skip a loss at most.
struct ErrorLeg {
    uint32_t received;
    uint32_t edges;        ///< the engine's edges from its vector
    uint32_t exact;        ///< bytes equal to the pattern at their position (engine)
    bool in_order;         ///< the received bytes are the pattern's, in order, some skipped
    ErrCounts err;
    uint32_t skips;        ///< how far rx_skips() moved
};

ErrorLeg error_leg(Mode m, uint32_t n) {
    constexpr uint32_t window_ms = 1400;
    print(plain, "  HOST poke ", static_cast<uint32_t>(m), " 115200 8E1 ", window_ms, " ", n,
          crlf);
    (void)drain();
    mode_release();
    (void)mode_init(m, 115200);
    spin_ms(120);
    mode_clear_errors();
    const uint32_t skips0 = mode_skips();
    rx_vector_edges = 0;
    static uint8_t got[300];
    ErrorLeg r{};
    const uint32_t t0 = cycles_now();
    const uint32_t window = (SysClock::hz / 1000u) * window_ms;
    while (cycles_now() - t0 < window) {
        if (r.received < sizeof got) {
            r.received += mode_read_bulk(got + r.received, sizeof got - r.received);
        }
    }
    // The owner's ask, once, for whatever no completion reported (nothing,
    // on a burst of exactly one block).
    mode_harvest();
    if (r.received < sizeof got) {
        r.received += mode_read_bulk(got + r.received, sizeof got - r.received);
    }
    r.err = mode_errors();
    // The interrupt receiver's epoch is its SkipRing's, modulo 2^8.
    r.skips = m != Mode::rxdma ? (mode_skips() - skips0) & 0xFFu : mode_skips() - skips0;
    r.edges = rx_vector_edges;
    back_to_console();
    uint32_t sx = lfsr_seed;
    r.in_order = true;
    uint32_t at = 0;   // pattern position
    uint32_t sy = lfsr_seed;
    for (uint32_t i = 0; i < r.received && i < n; ++i) {
        r.exact += got[i] == lfsr_next(sx) ? 1u : 0u;
    }
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
    // A whole block of the engine (half of the 512 ring): the run is
    // published by the engine's own vector at its completion, and no
    // owner's ask is on the path being proven.
    constexpr uint32_t n = 256;
    // What a frame-per-character reading predicts: the bytes whose even
    // parity bit is zero (an even count of ones).
    uint32_t s = lfsr_seed;
    uint32_t zero_parity = 0;
    for (uint32_t i = 0; i < n; ++i) {
        const uint8_t b = lfsr_next(s);
        zero_parity += (__builtin_popcount(b) & 1) == 0 ? 1u : 0u;
    }

    const ErrorLeg e = error_leg(Mode::rxdma, n);
    print(plain, "  receive engine: received ", e.received, " of ", n, ", equal to the pattern ",
          e.exact, ", frame ", e.err.frame, " parity ", e.err.parity, " hw_overrun ",
          e.err.hw_overrun, " (", zero_parity, " characters carry a zero parity bit)", crlf);
    bench.verdict("under the engine every character arrived: none taken by a clear",
                  e.received == n);
    bench.verdict("each one byte-exact and in its place", e.exact == n);
    bench.verdict("the errors were counted (one per run the engine published)",
                  e.err.frame != 0u);
    bench.verdict("published by the engine's vector, not the owner's ask", e.edges >= 1u);
    print(plain, "  the skip epoch moved ", e.skips, crlf);
    bench.verdict("the skip epoch moved (the run is delivered, not clean)", e.skips != 0u);

    const ErrorLeg i = error_leg(Mode::plain, n);
    print(plain, "  interrupt receiver: received ", i.received, ", frame ", i.err.frame,
          " parity ", i.err.parity, " hw_overrun ", i.err.hw_overrun, crlf);
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

// =============================================================================
// The menu
// =============================================================================
void banner() {
    print(plain, crlf,
          "test_samc_uart - SAMC21J18A SERCOM5 USART (ch. 30/31), clk=",
          SysClock::hz, " Hz", crlf,
          "  letters e..p need brio stress on the other end", crlf);
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
        case Loop::rxdma: (void)LRxDma::isr(); break;
        default: break;
    }
}

extern "C" void SERCOM5_Handler() {
    switch (live) {
        case Mode::plain: (void)UPlain::isr(); break;
        case Mode::txdma: (void)UTxDma::isr(); break;
        default: (void)URxDma::isr(); break;
    }
}

// ONLY THE LIVE TRANSPORT MAY BE TOLD - see the note beside `live`.
extern "C" void DMAC_Handler() {
    while (const auto irq = brio::Dmac::take_pending()) {
        const uint8_t ch = irq->channel;
        if (loop_live == Loop::txdma && ch == ch_loop_tx) {
            (void)LTxDma::dma_isr(ch);
            continue;
        }
        if (loop_live == Loop::rxdma && ch == ch_loop_rx) {
            if (LRxDma::dma_isr(ch)) {
                loop_edges = loop_edges + 1u;
            }
            continue;
        }
        switch (live) {
            case Mode::txdma: (void)UTxDma::dma_isr(ch); break;
            case Mode::rxdma:
                if (URxDma::dma_isr(ch)) {
                    rx_vector_edges = rx_vector_edges + 1u;
                }
                break;
            default: break;
        }
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();

    const bool dma_ok = brio::Dmac::init();
    brio::Nvic::enable(brio::Dmac::irq());

    const bool serial_ok = mode_init(Mode::plain, 115200);
    brio::enable_interrupts();

    bench.letter('a', "the baud generator's arithmetic", ta_baud);
    bench.letter('b', "every frame format, written and read back", tb_format);
    bench.letter('c', "the transmit ring's contract under pressure", tc_ring);
    bench.letter('d', "the engines' compile-time and register facts", td_engines);

    bench.letter('e', "echo, plain transport (host)", te_plain_echo, false);
    bench.letter('f', "echo, DMA transmitter (host)", tf_txdma_echo, false);
    bench.letter('g', "echo, DMA receiver (host)", tg_rxdma_echo, false);
    bench.letter('h', "echo at 1 and 2 Mbaud, DMA TX beside irq RX (host)",
                 th_duplex_echo, false);
    bench.letter('i', "receive-only sustained (host)", ti_sink, false);
    bench.letter('j', "transmit-only sustained (host)", tj_source, false);
    bench.letter('k', "115200 / 1 M / 3 Mbaud (host)", tk_rates, false);
    bench.letter('l', "the frame-format matrix (host)", tl_formats, false);
    bench.letter('m', "a MISMATCHED frame, and the recovery (host)", tm_mismatch,
                 false);
    bench.letter('n', "ring pressure: eager and lazy harvests (host)", tn_pressure,
                 false);
    bench.letter('p', "bursty traffic with idle gaps (host)", tp_burst, false);
    bench.letter('q', "tx_idle() on the loop: the wire's idle", tq_tx_idle);
    bench.letter('r', "the receive engine on the loop: the edge from the vector", tr_engine_edge);
    bench.letter('s', "errors under the receive engine and the interrupt receiver (host)",
                 ts_errors, false);
    bench.letter('t', "the interrupt receiver at 3 Mbaud on the loop", tt_levels);

    if (serial_ok) {
        print(plain, crlf, "boot: clk=", clock_ok ? "OSC48M" : "FAILED",
              " tick=", tick_ok ? "SysTick" : "FAILED",
              " dmac=", dma_ok ? "up" : "FAILED", crlf);
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
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(plain, "unknown letter (? for the menu)", crlf);
        }
        bench.prompt();
    }
}
