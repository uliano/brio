// test_samc_uart - the SERCOM USART transport (DS60001479M ch. 30/31),
// all four shapes of it, byte for byte.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over a
// serial console; z runs the self-contained ones and prints the ALL: line
// tools/bench.py judges.
//
// WHAT THIS SUITE IS FOR. samc/sercom.hpp's Uart can be built in four
// shapes - interrupt or DMA on each direction, independently - and until
// now nothing verified the BYTES through any of them: test_samc_dma
// exercises the DMAC's mechanics and serial_speed measures throughput,
// but neither checks a stream against what was sent. The letters below do
// exactly that, on both sides at once, with a pattern in which a lost,
// duplicated or reordered byte is IDENTIFIED and not merely counted.
//
// THE PATTERN is a 32-bit xorshift, low byte per step, seeded the same at
// both ends (tools/uart_stress.py runs the identical arithmetic). The
// board can therefore verify a received stream WITHOUT a return path, and
// the host can verify an echoed one - so every leg is checked at both
// ends, and the position of the first wrong byte is a number.
//
// WHY MOST LETTERS NEED THE HOST. This SERCOM's only wire is the console
// itself: there is no second port, no loop-back bit in this silicon (the
// AVR's LBME has no twin here) and the bench has no jumpers. A byte can
// only be checked against something OUTSIDE the chip, so the letters that
// stream traffic sit OUTSIDE z, driven by tools/uart_stress.py - which is
// also the only thing that can speak a frame format the board is not
// using, and therefore the only way to reach the receiver's error paths
// at all. z keeps what the board can decide alone: the baud arithmetic,
// the frame encoding, the ring contract, the engines' register facts.
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
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>

#include "samc/clock.hpp"
#include "samc/dmac.hpp"
#include "samc/nvic.hpp"
#include "samc/pin.hpp"
#include "samc/sercom.hpp"
#include "samc/ticker.hpp"
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

// 512 each way: big enough that a harvest cadence of a few hundred
// microseconds keeps up at 1 Mbaud, small enough that four
// instantiations fit comfortably in 32 KB of SRAM.
constexpr uint32_t rx_ring = 512;
constexpr uint32_t tx_ring = 512;

constexpr uint8_t ch_tx = 6;
constexpr uint8_t ch_rx = 7;

using UPlain = Uart<5, console_pads, rx_ring, tx_ring>;
using UTxDma = Uart<5, console_pads, rx_ring, tx_ring, DmaTxEngine<ch_tx>, NoDmaEngine>;
using URxDma = Uart<5, console_pads, rx_ring, tx_ring, NoDmaEngine, DmaRxEngine<ch_rx>>;
using UDuplex =
    Uart<5, console_pads, rx_ring, tx_ring, DmaTxEngine<ch_tx>, DmaRxEngine<ch_rx>>;

constexpr UPlain plain;

using Sc5 = UPlain::Resource;
using Led = Pin<'B', 23>;

TestBench<UPlain, 20> bench;

using brio::crlf;
using brio::print;

// =============================================================================
// The four transports behind one set of verbs
// =============================================================================
//
// THE FOUR INSTANTIATIONS ARE NEVER LIVE AT ONCE. Each has its own rings
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

enum class Mode : uint8_t { plain = 0, txdma = 1, rxdma = 2, duplex = 3 };
Mode live = Mode::plain;

const char* mode_name(Mode m) {
    switch (m) {
        case Mode::plain: return "irqTX+irqRX";
        case Mode::txdma: return "dmaTX+irqRX";
        case Mode::rxdma: return "irqTX+dmaRX";
        default: return "dmaTX+dmaRX";
    }
}

bool mode_init(Mode m, uint32_t baud, const UartFormat& fmt = {}) {
    live = m;
    switch (m) {
        case Mode::plain: return UPlain::init(clock, baud, fmt);
        case Mode::txdma: return UTxDma::init(clock, baud, fmt);
        case Mode::rxdma: return URxDma::init(clock, baud, fmt);
        default: return UDuplex::init(clock, baud, fmt);
    }
}

void mode_release() {
    switch (live) {
        case Mode::plain: UPlain::release(); break;
        case Mode::txdma: UTxDma::release(); break;
        case Mode::rxdma: URxDma::release(); break;
        default: UDuplex::release(); break;
    }
}

bool mode_tx_idle() {
    switch (live) {
        case Mode::plain: return UPlain::tx_idle();
        case Mode::txdma: return UTxDma::tx_idle();
        case Mode::rxdma: return URxDma::tx_idle();
        default: return UDuplex::tx_idle();
    }
}

uint32_t mode_write_bulk(const uint8_t* p, uint32_t n) {
    const std::span<const uint8_t> s(p, n);
    switch (live) {
        case Mode::plain: return UPlain::write_bulk(s);
        case Mode::txdma: return UTxDma::write_bulk(s);
        case Mode::rxdma: return URxDma::write_bulk(s);
        default: return UDuplex::write_bulk(s);
    }
}

uint32_t mode_read_bulk(uint8_t* p, uint32_t n) {
    const std::span<uint8_t> s(p, n);
    switch (live) {
        case Mode::plain: return UPlain::read_bulk(s);
        case Mode::txdma: return UTxDma::read_bulk(s);
        case Mode::rxdma: return URxDma::read_bulk(s);
        default: return UDuplex::read_bulk(s);
    }
}

void mode_harvest() {
    switch (live) {
        case Mode::rxdma: (void)URxDma::harvest(); break;
        case Mode::duplex: (void)UDuplex::harvest(); break;
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
        case Mode::rxdma: URxDma::clear_errors(); break;
        default: UDuplex::clear_errors(); break;
    }
}

struct ErrCounts {
    uint8_t rx_overrun, frame, parity, hw_overrun, dma_faults;
};

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
        case Mode::rxdma:
            return {URxDma::rx_overruns(), URxDma::frame_errors(),
                    URxDma::parity_errors(), URxDma::hw_overruns(),
                    URxDma::dma_faults()};
        default:
            return {UDuplex::rx_overruns(), UDuplex::frame_errors(),
                    UDuplex::parity_errors(), UDuplex::hw_overruns(),
                    UDuplex::dma_faults()};
    }
}

// =============================================================================
// The stream, and the clock
// =============================================================================

/// The 32-bit xorshift both ends run. tools/uart_stress.py holds the same
/// three shifts and the same seed; if either moves, every letter that
/// verifies a stream stops meaning anything, so neither does.
constexpr uint32_t lfsr_seed = 0x12345678UL;

[[gnu::always_inline]] inline uint8_t lfsr_next(uint32_t& s) {
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return static_cast<uint8_t>(s & 0xFFu);
}

/// The SAM suites' shared cycle-resolution stopwatch: SysTick counts CPU
/// cycles down inside a millisecond the Ticker counts up, read in a loop
/// that rejects a sample straddling the tick.
uint32_t cycles_now() {
    const uint32_t reload = SysTick->LOAD;
    for (;;) {
        const uint32_t t0 = Ticker::ticks();
        const uint32_t val = SysTick->VAL;
        const uint32_t t1 = Ticker::ticks();
        if (t0 == t1) {
            return t0 * (reload + 1u) + (reload - val);
        }
    }
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
    uint32_t harvest_us = 200;  ///< how often the owner asks the RX engine

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
          " dma_faults=", r.err.dma_faults,
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

void td_engines() {
    bench.verdict("the plain transport names no engine",
                  !UPlain::has_tx_engine && !UPlain::has_rx_engine);
    bench.verdict("the TX-only one names exactly one",
                  UTxDma::has_tx_engine && !UTxDma::has_rx_engine);
    bench.verdict("the RX-only one names the other",
                  !URxDma::has_tx_engine && URxDma::has_rx_engine);
    bench.verdict("the duplex one names both",
                  UDuplex::has_tx_engine && UDuplex::has_rx_engine);
    bench.verdict("an engineless transport reports no DMA faults, for free",
                  UPlain::dma_faults() == 0);

    // The trigger codes are the device header's, read the same way from
    // both sides of the wall: samc/sercom.hpp names them per instance,
    // samc/dmac.hpp spells the same table from the DMAC's side.
    bench.verdict("SERCOM5's RX trigger code is the header's",
                  Sc5::dma_rx_trigger() == SERCOM5_DMAC_ID_RX);
    bench.verdict("SERCOM5's TX trigger code is the header's",
                  Sc5::dma_tx_trigger() == SERCOM5_DMAC_ID_TX);
    bench.verdict("and dmac.hpp's own table agrees",
                  dma_trigger_sercom_rx<5>() == Sc5::dma_rx_trigger() &&
                      dma_trigger_sercom_tx<5>() == Sc5::dma_tx_trigger());

    // A channel has exactly ONE pending-trigger bit, which is what makes
    // the standing-request kick safe: a kick that races a real hardware
    // trigger is LOST, not doubled (25.8.8). The register says so about
    // itself - it reads back set exactly when a trigger was lost.
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
        // A DMA receiver publishes only what a harvest has taken, and it
        // has no run to fill between a block ending and the next harvest
        // re-arming it. A gap there is the contract, not a defect, so the
        // letter MEASURES it instead of pretending it away.
        print(plain, "  (lossy by contract: the RX engine publishes at harvest "
                     "granularity and is idle between blocks)", crlf);
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
void tg_rxdma_echo() { run_and_report(base_leg(Op::echo, Mode::rxdma), false); }
void th_duplex_echo() { run_and_report(base_leg(Op::echo, Mode::duplex), false); }

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
    // ceiling and the wire's measured limit (docs/samc/sercom.md).
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
        // measured in docs/samc/sercom.md. So the fast rate is not asked
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
    // RING PRESSURE, both extremes, through the transport that has the
    // most to lose. A lazy cadence gives the RX engine a whole block to
    // fill between asks; an eager one asks far more often than any
    // application would. NEITHER MAY WEDGE.
    static constexpr uint32_t cadences[] = {50, 2000};
    for (uint32_t us : cadences) {
        Leg leg = base_leg(Op::echo, Mode::duplex);
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
    }
}

void tp_burst() {
    // Bursty traffic with idle gaps - the shape a console really sees,
    // and the one an RX engine is worst at: nothing tells it a byte has
    // arrived, so an idle line is exactly where the harvest cadence
    // carries the whole latency.
    Leg leg = base_leg(Op::burst, Mode::duplex);
    leg.window_ms = 1400;
    run_and_report(leg, false);
}

// =============================================================================
// The menu
// =============================================================================
void banner() {
    print(plain, crlf,
          "test_samc_uart - SAMC21J18A SERCOM5 USART (ch. 30/31), clk=",
          SysClock::hz, " Hz", crlf,
          "  letters e..p need tools/uart_stress.py on the other end", crlf);
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

extern "C" void SERCOM5_Handler() {
    switch (live) {
        case Mode::plain: (void)UPlain::isr(); break;
        case Mode::txdma: (void)UTxDma::isr(); break;
        case Mode::rxdma: (void)URxDma::isr(); break;
        default: (void)UDuplex::isr(); break;
    }
}

// ONLY THE LIVE TRANSPORT MAY BE TOLD - see the note beside `live`.
extern "C" void DMAC_Handler() {
    while (const auto irq = brio::Dmac::take_pending()) {
        const uint8_t ch = irq->channel;
        switch (live) {
            case Mode::txdma: (void)UTxDma::dma_isr(ch); break;
            case Mode::rxdma: (void)URxDma::dma_isr(ch); break;
            case Mode::duplex: (void)UDuplex::dma_isr(ch); break;
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
    bench.letter('h', "echo, both engines (host)", th_duplex_echo, false);
    bench.letter('i', "receive-only sustained (host)", ti_sink, false);
    bench.letter('j', "transmit-only sustained (host)", tj_source, false);
    bench.letter('k', "115200 / 1 M / 3 Mbaud (host)", tk_rates, false);
    bench.letter('l', "the frame-format matrix (host)", tl_formats, false);
    bench.letter('m', "a MISMATCHED frame, and the recovery (host)", tm_mismatch,
                 false);
    bench.letter('n', "ring pressure: eager and lazy harvests (host)", tn_pressure,
                 false);
    bench.letter('p', "bursty traffic with idle gaps (host)", tp_burst, false);

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
