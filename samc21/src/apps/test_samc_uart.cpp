// test_samc_uart - the SERCOM USART transport (DS60001479M ch. 30/31),
// both shapes of it, byte for byte.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over a
// serial console; z runs the self-contained ones and prints the ALL: line
// bin/brio judges, and y - the SECOND SET - runs the two-board letters
// against the far board's usart_peer, failing loudly without one (z never
// runs them, and runs with the far board dark).
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
// receiver on its own transmitter's pad (PAD[2], PA18 under function C),
// 31.6.3.8's loop-back "through the pad" - nothing may drive PA18 but
// this board (a far board's receiver on it, an input, is fine). z
// runs what the board decides alone: the baud arithmetic, the frame
// encoding, the ring contract, the engine's facts and its claim, and on
// the loop the wire's idle, the interrupt receiver's edge and its skip
// past a loss, the level loop at the generator's top rate, and what a
// released transport leaves the next owner of the DMA channel.
//
// A LOOP PROVES THE BYTES, NEVER THE RATE: its receiver samples on the
// divisor its transmitter shifts on. So every letter that loops or sets a
// rate also times a run of frames through the transmit engine against the
// wire's time at the rate ASKED, prints the reading in thousandths and
// judges it within -1 % and +3 % (wire_permille(), in the loop's section).
// The host letters need no such reading: their far end is the bridge's
// own divisor, and a wrong rate there is a broken stream.
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
// What is exercised, letter by letter - a..d, g, q, r, t and w need
// nothing outside the board and are what z runs; e..p and s stream traffic over the
// console and are driven by brio stress; x stops the board and runs by its
// name alone:
//   a  the baud generator's arithmetic, and its ladder - 9600 baud to
//      3 Mbaud - timed on the loop's wire
//   b  every frame format, written and read back, and each one's frame
//      timed on the loop's wire against its own bits
//   c  the transmit ring's contract under pressure
//   d  the transmit engine's facts and its claim: one owner at a time;
//      its level read back from the armed channel
//   g  the RESOURCE past the console's regime, polled on the loop: every
//      sampling regime and the synchronous host timed on the wire, RXEN
//      and TXEN switched at run time (a disabled transmitter still drives
//      its pad, read under the pad's pull-down), the IrDA coder's pulse
//      sense on the pad
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
//      after it, for the plain transport and the transmit engine; the
//      loop's rates timed
//   r  the interrupt receiver on the loop: one edge from the vector a
//      burst; a consumer held back past the ring - the loss counted, the
//      skip epoch moved at the first look after it and nothing the ring
//      held across it handed out; the consumer's release recovering;
//      the loop's rates timed
//   s  errors injected under the interrupt receiver (host): each hit
//      character dropped, counted and skipped past
//   t  the interrupt receiver at the generator's top rate on the loop:
//      every level an entry; the rate timed
//   w  A RELEASED REQUESTER: the transmit engine's trigger standing, the
//      transport released three ways (disabled and gated, gated with
//      ENABLE set, Uart::release() with its reset), the one channel bound
//      again - to no trigger, to the released SERCOM's own, and by the
//      next Uart - and what each first enable moves (the release contract,
//      docs/design/dma.md)
//   x  THE ONE-USER RULE AS A PANIC: a second engined transport's init()
//      while the loop's holds the claim - the board stops, and the boot
//      banner after the next reset prints the breadcrumb (code 2,
//      assert_failed; context 0xD6, the claim of SERCOM5 refused)
//
// THE TWO-BOARD LETTERS (y; SERCOM1 on the desk's four wires against an
// AVR128DA48's usart_peer - the section below says how the wires are used):
//   J  the link and the far peer: who answers, on what time base
//   K  THE RATE judged by a foreign receiver on a crystal: every regime of
//      the generator, 2400 baud to 3 Mbaud, a gapless burst timed by the
//      far end against the error the register predicts; each rung's echo
//   L  the frame matrix against a different UART, the stop-bit pairings,
//      MSB first both ways
//   M  errors with known causes: parity, a break, a flood read late under
//      IBON 0 and 1; start-of-frame detection's RXS ahead of RXC
//   N  the operational range from the far USART's own generator; the
//      bit-banger's stretched cell and lone pulses against the receiver's
//      votes (SAMPA 0 and 2), measured on the pad
//   O  auto-baud from a foreign sender (rates learned, a wrong sync, the
//      break threshold, erratum 1.17.2's missing stop bit by revision);
//      the LIN host's header timed on its own pad and read by the far end
//   Q  the synchronous roles on a real XCK, both polarities, both ends
//   S  IrDA both ways (the far receiver inverts its pin itself), RXPL
//      against the far coder's pulses, the coding against plain frames
//   C  handshaking: CTS against levels the far board holds on XCK, RTS on its pad
//   R  RS-485: TE timed on its own pad on the shared wire, GTIME 0..7
//   W  the one-wire bus on the shared wire, and COLDEN against the far
//      collide: the abort, and what restarts the transmitter after it
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include <initializer_list>
#include <optional>
#include <span>

#include "kernel/panic.hpp"
#include "samc21/clock.hpp"
#include "samc21/dmac.hpp"
#include "samc21/dsu.hpp"
#include "samc21/nvic.hpp"
#include "samc21/pin.hpp"
#include "samc21/sercom.hpp"
#include "samc21/ticker.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

// The two-board half's protocol: shared with the far board's usart_peer,
// pure encoding and no register.
#include "../../../avrxt/src/apps/usart_link.hpp"

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

// 'z' runs the board's own letters, 'y' the two-board ones (the second
// set: they need the far board's usart_peer and fail without it).
TwoKeyTestBench<UPlain, 'y', 32> bench;

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

// THE RATE ON THE WIRE, which a loop cannot tell (the loop's section below
// holds the instrument): letters a and b time their rates and frames
// there before the loop letters do.
uint32_t wire_permille(uint32_t baud, const UartFormat& format = {});
bool wire_ok(uint32_t permille);
uint32_t frame_bits(const UartFormat& f);

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

    // THE LADDER ON THE WIRE: each rung's divisor driving the loop's
    // transmitter, a run of frames timed against its wire time at the rate
    // ASKED - the arithmetic above is checked against its own inverse, and
    // only the wire can tell a divisor that is wrong in both.
    bool ladder_ok = true;
    print(plain, "  the ladder on the loop, a run of frames in thousandths of its wire time:");
    for (uint32_t baud : rates) {
        const uint32_t pm = wire_permille(baud);
        print(plain, " ", baud, " ", pm);
        ladder_ok = ladder_ok && wire_ok(pm);
    }
    print(plain, crlf);
    bench.verdict("EVERY RUNG RUNS AT THE RATE ASKED: 9600 baud to 3 Mbaud, a run of frames on the "
                  "loop in its wire time within -1 % and +3 % (the divisor's own rounding is 0.14 % "
                  "at 9600 and less above)",
                  ladder_ok);
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
    // What the console still holds - the letter before this one's last
    // lines - goes out before the first re-init clears the ring.
    (void)drain();
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

    // THE FRAME ON THE WIRE: the same formats on the loop, a run of each
    // timed against its frame's own length at 115200 - start, data,
    // parity and stop bits - which the register readback above cannot
    // show.
    bool frames_ok = true;
    print(plain, "  the frames on the loop at 115200, in thousandths of their wire time:");
    for (const Row& row : rows) {
        const UartFormat f{.bits = row.bits, .parity = row.parity, .two_stop = row.two_stop};
        const uint32_t pm = wire_permille(115200, f);
        print(plain, " ", format_bits(f), format_parity(f), f.two_stop ? '2' : '1', " (",
              frame_bits(f), " bits) ", pm);
        frames_ok = frames_ok && wire_ok(pm);
    }
    print(plain, crlf);
    bench.verdict("every frame takes its own length on the wire: a run of each format on the loop "
                  "within -1 % and +3 % of its bits at 115200",
                  frames_ok);
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

// The loop's engined transport for a reading of the armed engine (letter
// d); the loop's section below holds the transport.
bool arm_loop_engine();
void release_loop_engine();

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

    // THE LEVEL, read back from the armed channel: the loop's engined
    // transport holds the engine for the reading and gives it back.
    std::optional<uint8_t> level;
    if (arm_loop_engine()) {
        level = DmaTxEngine::priority();
        release_loop_engine();
    }
    print(plain, "  the armed channel's level: ", level ? static_cast<uint32_t>(*level) : 0xFFu,
          level ? "" : " (not enabled in CTRL)", crlf);
    bench.verdict("the transmit engine arms at level 2 of four, the rule's NEXT for a transmit, "
                  "with that level enabled in CTRL",
                  level.has_value() && *level == DmaTxEngine::level && DmaTxEngine::level == 2u);
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
// transmitter's own signal, through the pad. PAD[2] of SERCOM1 (TXPO 0x1,
// RXPO 0x2) is PA18 under function C - of the two pads TxD can sit on
// (31.8.1), the one whose wire, on a desk that links this SERCOM to an
// AVR's USART4, meets that board's RxD - an input while the far board
// runs its USART peer or its dark console, so the loop meets no second
// driver there. Two transports over it, never live at once (`loop_live`).

constexpr UartPads loop_pads{
    .tx = SercomPad::pad2,
    .rx = SercomPad::pad2,
    .tx_pin = {'A', 18, PinFunction::c},
    .rx_pin = {'A', 18, PinFunction::c},
};

using LPlain = Uart<1, loop_pads, 512, 512>;
using LTxDma = Uart<1, loop_pads, 512, 512, DmaTxEngine>;
using Sc1 = LPlain::Resource;
constexpr uint8_t loop_owner = static_cast<uint8_t>(Sc1::index + 1u);

enum class Loop : uint8_t { none, plain, txdma, link_dma };
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

bool arm_loop_engine() {
    loop_live = Loop::txdma;
    if (LTxDma::init(clock, 115'200u)) {
        return true;
    }
    release_loop_engine();
    return false;
}

void release_loop_engine() {
    LTxDma::release();
    loop_live = Loop::none;
}

// ---- the rate on the wire ----------------------------------------------------
//
// A LOOP CANNOT TELL A WRONG RATE: its receiver samples on the divisor its
// transmitter shifts on, so a divisor wrong at both ends is a slower or a
// faster loop and nothing else. So every letter that loops or sets a rate
// times a RUN of frames against the wire's time at the rate it ASKED -
// never at the rate the divisor produced, which is what a wrong divisor
// would hide - and prints the reading in thousandths of that time.
//
// The run goes through the transmit engine, x 1.00 of the wire at every
// rate this generator makes (the interrupt transmitter is the CPU's at
// 3 Mbaud, x 2.05: docs/samc21/sercom.md), with the loop's receiver
// unserved - SERCOM1's line held off, the overflow in its buffer cleared
// by the release - so the run is the transmitter's alone. It is timed on
// the ticker's cycle count (one core cycle, OSC48M: the same oscillator as
// the generator, so the reading judges the divisor and not the oscillator)
// from the first block's start to the last stop bit (tx_idle(): TXC), a
// hundredth of a second of frames - sixteen at least, 1024 at most, where
// the run's fixed cost of some tens of cycles is a thousandth of a 3 Mbaud
// run.

/// The bits of one frame: start, data, parity, stop.
uint32_t frame_bits(const UartFormat& f) {
    uint32_t data = 8;
    switch (f.bits) {
        case UartBits::five: data = 5; break;
        case UartBits::six: data = 6; break;
        case UartBits::seven: data = 7; break;
        case UartBits::nine: data = 9; break;
        default: break;
    }
    return 1u + data + (f.parity != UartParity::none ? 1u : 0u) + (f.two_stop ? 2u : 1u);
}

/// The verdict on a reading: never faster than the rate asked by more than
/// the ruler's own grain, at most three per cent slower.
bool wire_ok(uint32_t permille) { return permille >= 990u && permille <= 1030u; }

uint32_t wire_permille(uint32_t baud, const UartFormat& format) {
    static uint8_t run[1024];
    const uint32_t bits = frame_bits(format);
    uint32_t frames = baud / (100u * bits);
    frames = frames < 16u ? 16u : (frames > sizeof run ? static_cast<uint32_t>(sizeof run) : frames);
    for (uint32_t i = 0; i < frames; ++i) {
        run[i] = static_cast<uint8_t>(0x55u + i);
    }
    loop_live = Loop::txdma;
    if (!LTxDma::init(clock, baud, format)) {
        loop_live = Loop::none;
        return 0;
    }
    loop_settle(baud);
    Nvic::disable(Sc1::irq());   // the receiver unserved: the run is the transmitter's
    uint32_t q = LTxDma::write_bulk(std::span<const uint8_t>(run, frames));
    const uint32_t t0 = cycles_now();   // the first block started: its first beat is in DATA
    const uint64_t wire =
        static_cast<uint64_t>(bits) * frames * SysClock::hz / baud;
    while (q < frames) {
        q += LTxDma::write_bulk(std::span<const uint8_t>(run + q, frames - q));
    }
    while (!LTxDma::tx_idle()) {
        if (cycles_now() - t0 > 2u * static_cast<uint32_t>(wire)) {
            break;
        }
    }
    const uint32_t took = cycles_now() - t0;
    LTxDma::release();
    loop_live = Loop::none;
    return static_cast<uint32_t>(1000ull * took / wire);
}

/// The rungs a loop letter ran at, each timed: printed, and judged all at
/// once.
bool wire_rungs(std::initializer_list<uint32_t> rates) {
    bool ok = true;
    print(plain, "  on the wire, a run of frames in thousandths of its time at the rate asked:");
    for (const uint32_t baud : rates) {
        const uint32_t pm = wire_permille(baud);
        print(plain, " ", baud, " ", pm);
        ok = ok && wire_ok(pm);
    }
    print(plain, crlf);
    return ok;
}

// ---- g - the resource's clocking and its directions, on the loop -------------
//
// What the RESOURCE offers past the console's regime (docs/samc21/sercom.md),
// each driven POLLED on the loop's pad - the transmitter fed whenever DRE
// stands, every frame read back as it lands - and timed from the first
// DATA write to TXC against the run's wire time at the rate ASKED, on the
// core clock that is generator 0's own, so the reading judges the divisor:
//  - every sampling regime: 16x and 8x on both generators, 3x arithmetic,
//    at rates up to 2 Mbaud - a polled turn that both feeds DATA and reads
//    the loop back is the CPU's, not the wire's, past that (measured:
//    3 and 4 Mbaud read 1015 and 1033 whatever the regime);
//  - a synchronous host hearing itself through the pad (the internal XCK,
//    PA19 not muxed: nothing leaves the chip but TxD);
//  - RXEN and TXEN switched on a running instance: a disabled receiver
//    takes nothing; a disabled transmitter sends nothing - and does NOT
//    release its pad, which is the measured fact here: read under the
//    pad's pull, a pull-DOWN under the SERCOM's mux (31.5.1), the pad
//    falls with the instance disabled or the pad handed to PORT, and
//    rises again when it is handed back to the SERCOM with the
//    transmitter still off - driven, not floating. THE CONDITION: the pad
//    meets the far board's RxD, which must hold no pull - its dark
//    console, or its USART peer while that SEARCHES (no pull on any link
//    pad, usart_link.hpp's dark discipline), both qualify; a far board
//    pulling its RxD up would make the reading its pull's;
//  - the IrDA coder's pulse sense, which the loop cannot hear (31.6.3.3:
//    "the polarity of the transmitter and receiver are opposite"): the
//    pad's idle level read, and a 0x00 frame sampled, high in its pulses.

using LoopPad = Pin<'A', 18>;

/// The resource up on the loop's pad under `c`, polled: no interrupt.
bool resource_up(const SercomUartConfig& c) {
    Nvic::disable(Sc1::irq());
    loop_live = Loop::none;
    Sc1::bus_clock(true);
    if (!Sc1::core_clock(0) || !Sc1::reset() || !Sc1::configure(c) || !Sc1::enable(true)) {
        return false;
    }
    LoopPad::function(PinFunction::c, {.input_enable = true});
    loop_settle(115200);
    Sc1::flush_rx();
    return true;
}

void resource_down() {
    Sc1::release();
    LoopPad::release();
    LoopPad::input();
}

struct ResourceRun {
    bool up = false;
    uint32_t sent = 0;
    uint32_t got = 0;
    uint32_t bad = 0;
    uint32_t permille = 0;
};

/// `frames` frames out and back through the pad, `bits` a frame, at the
/// rate `baud` asked. The frame's value walks so a slip is a mismatch.
ResourceRun resource_run(const SercomUartConfig& c, uint32_t baud, uint32_t bits) {
    ResourceRun r{};
    r.up = resource_up(c);
    if (!r.up) {
        resource_down();
        return r;
    }
    uint32_t frames = baud / (100u * bits);
    frames = frames < 16u ? 16u : (frames > 256u ? 256u : frames);
    const uint64_t wire = static_cast<uint64_t>(bits) * frames * SysClock::hz / baud;
    // The bound is a count of turns, not a clock read: the ticker's cycle
    // count costs some tens of cycles, and a turn has to be shorter than
    // a 3x frame at 4 Mbaud (120 cycles) for the run to be the wire's.
    while (!Sc1::dre_flag()) {
    }
    Sc1::data(0x35u);
    const uint32_t t0 = cycles_now();
    r.sent = 1;
    for (uint32_t turns = 64u * static_cast<uint32_t>(wire); r.got < frames && turns != 0u;
         --turns) {
        if (r.sent < frames && Sc1::dre_flag()) {
            Sc1::data(static_cast<uint8_t>(0x35u + r.sent));
            ++r.sent;
        }
        if (Sc1::rxc_flag()) {
            const uint16_t st = Sc1::status();
            const uint8_t d = static_cast<uint8_t>(Sc1::data());
            if ((st & SercomStatus::receive_errors) != 0u) {
                Sc1::clear_status(st & SercomStatus::receive_errors);
                ++r.bad;
            } else if (d != static_cast<uint8_t>(0x35u + r.got)) {
                ++r.bad;
            }
            ++r.got;
        }
    }
    const uint32_t spin0 = cycles_now();
    while (!Sc1::txc_flag() && cycles_now() - spin0 < ms_cycles(2)) {
    }
    const uint32_t took = cycles_now() - t0;
    r.permille = static_cast<uint32_t>(1000ull * took / wire);
    resource_down();
    return r;
}

struct RegimeRung {
    UartSampling sampling;
    uint32_t baud;
    const char* name;
};

void tg_resource() {
    print(plain, "  regime        rate     BAUD    sent got bad  wire (thousandths)", crlf);
    const RegimeRung rungs[] = {
        {UartSampling::x16_arithmetic, 115200, "16x arith"},
        {UartSampling::x16_fractional, 3750, "16x frac"},
        {UartSampling::x16_fractional, 115200, "16x frac"},
        {UartSampling::x8_arithmetic, 2'000'000, "8x arith"},
        {UartSampling::x8_fractional, 2'000'000, "8x frac"},
        {UartSampling::x3_arithmetic, 2'000'000, "3x arith"},
    };
    bool every = true, timed = true;
    for (const RegimeRung& g : rungs) {
        const auto reg = sercom_uart_baud_reg(SysClock::hz, g.baud, g.sampling);
        if (!reg) {
            every = false;
            continue;
        }
        const ResourceRun r = resource_run(
            {.pads = loop_pads, .baud = *reg, .sampling = g.sampling}, g.baud, 10);
        print(plain, "  ", g.name, "  ", g.baud, "  ", *reg, "  ", r.sent, " ", r.got, " ", r.bad,
              "  ", r.permille, crlf);
        every = every && r.up && r.got == r.sent && r.sent >= 16u && r.bad == 0u;
        timed = timed && wire_ok(r.permille);
    }
    bench.verdict("every sampling regime carries its frames through the pad, byte-exact", every);
    bench.verdict("and each runs at the rate asked (16x and 8x on both generators, 3x)", timed);

    // The synchronous host, both clock polarities, the internal XCK at 1 MHz.
    {
        const uint8_t reg = sercom_sync_baud_reg(SysClock::hz, 1'000'000).value();
        bool both = true, both_timed = true;
        for (uint8_t pol = 0; pol < 2; ++pol) {
            const ResourceRun r = resource_run({.pads = loop_pads, .baud = reg,
                                                .synchronous = true, .invert_clock = pol != 0},
                                               1'000'000, 10);
            print(plain, "  sync host 1 MHz, CPOL ", pol, ": sent ", r.sent, " got ", r.got,
                  " bad ", r.bad, ", wire ", r.permille, crlf);
            both = both && r.up && r.got == r.sent && r.sent >= 16u && r.bad == 0u;
            both_timed = both_timed && wire_ok(r.permille);
        }
        bench.verdict("a synchronous host hears its own frames through the pad, either CPOL",
                      both);
        bench.verdict("at f_ref / (2 (BAUD + 1)): 1 MHz from BAUD 23", both_timed);
    }

    const uint16_t reg115 = sercom_baud_reg(SysClock::hz, 115200).value();
    const SercomUartConfig plain_cfg{.pads = loop_pads, .baud = reg115};

    // RXEN at run time: a disabled receiver takes nothing; on again, all.
    {
        bool up = resource_up(plain_cfg);
        up = Sc1::enable_rx(false) && up;
        const bool rx_off = !Sc1::rx_enabled();
        for (uint8_t i = 0; i < 4; ++i) {
            while (!Sc1::dre_flag()) {
            }
            Sc1::data(static_cast<uint8_t>(0xA0u + i));
        }
        spin_ms(1);
        const bool heard_off = Sc1::rxc_flag();
        up = Sc1::enable_rx(true) && up;
        uint8_t got = 0;
        for (uint8_t i = 0; i < 4; ++i) {
            while (!Sc1::dre_flag()) {
            }
            Sc1::data(static_cast<uint8_t>(0xB0u + i));
            const uint32_t w0 = cycles_now();
            while (!Sc1::rxc_flag() && cycles_now() - w0 < ms_cycles(1)) {
            }
            if (Sc1::rxc_flag() && static_cast<uint8_t>(Sc1::data()) == 0xB0u + i) ++got;
        }
        resource_down();
        print(plain, "  RXEN off: rx_enabled ", rx_off ? 0 : 1, ", heard ", heard_off ? 1 : 0,
              "; on again: ", got, " of 4", crlf);
        bench.verdict("enable_rx(false): the receiver takes nothing", up && rx_off && !heard_off);
        bench.verdict("enable_rx(true): every frame after it read back", up && got == 4u);
    }

    // TXEN at run time: the pad driven high, then TRI-STATED under the
    // SERCOM's pull-down (31.5.1); a write while off sends nothing. The
    // pull itself is proven first, on the pad with the instance disabled.
    {
        bool up = resource_up(plain_cfg);
        LoopPad::function(PinFunction::c, {.input_enable = true, .pull = PinPull::down});
        (void)Sc1::enable(false);
        spin_ms(1);
        const bool high_disabled = LoopPad::read();
        up = Sc1::enable(true) && up;
        spin_ms(1);
        const bool high_on = LoopPad::read();
        up = Sc1::enable_rx(false) && up;
        up = Sc1::enable_tx(false) && up;
        spin_ms(1);
        const bool high_both_off = LoopPad::read();
        up = Sc1::enable_rx(true) && up;
        spin_ms(1);
        const bool off_flag = !Sc1::tx_enabled();
        const bool high_off = LoopPad::read();
        // Driven, or floating at the level it had? Take the pad to PORT
        // under the same pull-down - it falls - and hand it back to the
        // SERCOM, the transmitter still off: a floating pad stays low, a
        // driven one rises.
        LoopPad::release();
        LoopPad::input(PinPull::down);
        spin_ms(1);
        const bool high_released = LoopPad::read();
        LoopPad::function(PinFunction::c, {.input_enable = true, .pull = PinPull::down});
        spin_ms(1);
        const bool high_remuxed = LoopPad::read();
        print(plain, "  the pad under its pull-down: instance disabled ",
              high_disabled ? "HIGH" : "low", ", TXEN and RXEN off ",
              high_both_off ? "HIGH" : "low", "; TXEN off, the pad to PORT ",
              high_released ? "HIGH" : "low", " and back to the SERCOM ",
              high_remuxed ? "HIGH" : "low", crlf);
        Sc1::flush_rx();
        Sc1::data(0x5A);
        spin_ms(1);
        const bool sent_off = Sc1::rxc_flag();
        Sc1::flush_rx();
        up = Sc1::enable_tx(true) && up;
        spin_ms(1);
        const bool high_again = LoopPad::read();
        const bool queued_went = Sc1::rxc_flag();
        resource_down();
        print(plain, "  TXEN on: pad ", high_on ? "high" : "LOW", "; off: tx_enabled ",
              off_flag ? 0 : 1, ", pad ", high_off ? "HIGH" : "low", ", a write sent ",
              sent_off ? 1 : 0, "; on again: pad ", high_again ? "high" : "LOW",
              ", the write while off went out ", queued_went ? 1 : 0, crlf);
        bench.verdict("the transmitter drives its idle high over the pad's pull-down", up && high_on);
        bench.verdict("the pull-down is live under the mux: the disabled instance's pad falls",
                      !high_disabled);
        bench.verdict("enable_tx(false) does NOT release TxD: the pad stays DRIVEN high "
                      "(to PORT it falls, back to the SERCOM it rises, the transmitter off)",
                      off_flag && high_off && high_both_off && !high_released && high_remuxed);
        bench.verdict("and a write while the transmitter is off puts nothing on the wire",
                      !sent_off);
        bench.verdict("enable_tx(true) drives the idle high again", high_again);
    }

    // The IrDA coder's sense: the pad's idle level, and a 0x00 frame
    // sampled at 9600 (nine 0 bits, so 9 x 3/16 of a bit HIGH if a 0 is a
    // high pulse: 17 % of the frame's ten bits).
    {
        const uint16_t reg = sercom_baud_reg(SysClock::hz, 9600).value();
        const bool up = resource_up({.pads = loop_pads, .baud = reg, .irda = true});
        spin_ms(2);
        const bool idle_high = LoopPad::read();
        Sc1::flush_rx();
        uint32_t samples = 0, high = 0;
        while (!Sc1::dre_flag()) {
        }
        Sc1::data(0x00);
        const uint32_t t0 = cycles_now();
        const uint32_t frame = 10u * bit_cycles(9600);
        while (cycles_now() - t0 < frame) {
            ++samples;
            if (LoopPad::read()) ++high;
        }
        spin_ms(1);
        bool clean = false;
        for (uint8_t k = 0; k < 4 && Sc1::rxc_flag(); ++k) {
            const uint16_t st = Sc1::status();
            const uint8_t d = static_cast<uint8_t>(Sc1::data());
            if ((st & SercomStatus::receive_errors) == 0u && d == 0x00u) clean = true;
            Sc1::clear_status(SercomStatus::receive_errors);
        }
        resource_down();
        const uint32_t pc = samples ? (100u * high) / samples : 0u;
        print(plain, "  IrDA at 9600: the pad idles ", idle_high ? "HIGH" : "LOW",
              "; a 0x00 frame high in ", high, " of ", samples, " samples (", pc,
              " %, 17 % if a 0 is a high pulse of 3/16); read back clean: ", clean ? 1 : 0, crlf);
        bench.verdict("IrDA: TxD idles LOW and a 0 bit leaves as a HIGH pulse of about 3/16",
                      up && !idle_high && pc >= 12u && pc <= 22u);
        bench.verdict("so the coder's own receiver, which takes a LOW pulse as a 0, cannot "
                      "read it through the pad", !clean);
    }
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
    // The receiver is read on both sides of tx_idle(), so the longest gap
    // between two reads is half a turn and not a whole one: at 3 Mbaud the
    // two-level buffer holds two frames, 320 cycles, and a turn that read
    // it once lost a character in some runs, whenever the transmit
    // engine's completion interrupt landed in it.
    auto take = [&] {
        while (Sc1::rxc_flag()) {
            (void)Sc1::data();
            if (++got == n) {
                turn_last = turn;
                r.last_seen = true;
            }
        }
    };
    for (; turn < 400'000u; ++turn) {
        take();
        if (!r.idle_seen && U::tx_idle()) {
            turn_idle = turn;
            r.idle_seen = true;
        }
        take();
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
    bench.verdict("and the loop runs at the rates asked: 115200, 1 and 3 Mbaud, a run of frames in "
                  "its wire time within -1 % and +3 %",
                  wire_rungs({115'200u, 1'000'000u, 3'000'000u}));
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
    bench.verdict("and the loop runs at the rates asked: 1 and 3 Mbaud, a run of frames in its "
                  "wire time within -1 % and +3 %",
                  wire_rungs({1'000'000u, 3'000'000u}));
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
    bench.verdict("and the loop runs at the rate asked: 3 Mbaud, a run of frames in its wire time "
                  "within -1 % and +3 %",
                  wire_rungs({3'000'000u}));
}

// ---- w - a released requester: what reaches the next owner -------------------
//
// The release contract (docs/design/dma.md): a peripheral done with a
// channel leaves no request behind for the next owner. On this controller
// the request a channel serves is ITS TRIGGER SELECTION's - CHCTRLB.TRIGSRC
// names one peripheral line - and the transmitter's line is a level, "set
// when the transmit buffer is empty" (31.6.4.1). Two things could outlive
// a release: a rise the channel kept from before, and the level in the
// gated SERCOM. Each is staged with the transmit engine's trigger standing
// - the last block over, its channel disabled by its end (25.6.2.6), DRE
// up behind it - and the release is run three ways: the SERCOM disabled
// and gated, as the release did before the reset (staged here by hand,
// the resource's verbs in its order); gated with ENABLE left set, a
// release that skips the disable too; and Uart::release(), which resets
// the SERCOM before the gate. Then the one channel is bound again:
//
//   1. to NO trigger (TRIGSRC 0), its destination a word of RAM, and a
//      block started: anything that moves is a rise the release and the
//      next arm() let through;
//   2. to the RELEASED SERCOM's own trigger, the instance still gated: a
//      beat that moves is a request the gated instance holds;
//   3. by the next Uart on the same SERCOM, the whole hand-over: its
//      first block on the loop, every byte, in order, nothing more.
//
// And a control first, on whether the channel keeps a rise at all: DATA
// filled by hand behind the engine's last block - DRE falling and rising
// again while the channel is disabled, then low - and a block of one
// started: a kept rise would fire its beat into a full DATA at the enable.

/// The owner id letter w's hand-staged bindings claim with: no Uart's (a
/// Uart's is its SERCOM number plus one).
constexpr uint8_t probe_owner = 11;
/// What a hand-staged binding's channel writes: a word of RAM, the low
/// byte a beat's.
volatile uint32_t probe_sink = 0;
constexpr uint32_t probe_mark = 0xA5A5A5A5u;
/// Interrupt flags the DMAC raised while a hand-staged binding held the
/// engine (DMAC_Handler takes them for it).
volatile uint8_t probe_flags = 0;
const uint8_t probe_run[4] = {0x5A, 0x5B, 0x5C, 0x5D};

struct Bound {
    bool armed;
    bool moved;        ///< the sink was written
    uint8_t beat;      ///< its low byte, a beat's
    bool busy;         ///< the engine's block still claimed
    bool waiting;      ///< the channel enabled with nothing pending or moving
    uint8_t flags;     ///< what the DMAC raised
};

/// The one channel bound by hand to `trigger`, its destination the sink,
/// a block of four started: what moved within a millisecond. The block is
/// then abandoned and the claim given back.
Bound bind_probe(uint8_t trigger) {
    Bound b{};
    probe_sink = probe_mark;
    probe_flags = 0;
    b.armed = DmaTxEngine::arm(probe_owner, &probe_sink, trigger);
    if (b.armed) {
        (void)DmaTxEngine::start(std::span<const uint8_t>(probe_run, sizeof probe_run));
        spin_ms(1);
        b.moved = probe_sink != probe_mark;
        b.beat = static_cast<uint8_t>(probe_sink & 0xFFu);
        b.busy = DmaTxEngine::busy();
        b.waiting = DmaTxEngine::waiting();
        b.flags = probe_flags;
        (void)DmaTxEngine::abandon();
        DmaTxEngine::clear_faults();
    }
    DmaTxEngine::release(probe_owner);
    return b;
}

void print_bound(const char* what, const Bound& b) {
    print(plain, "    bound to ", what, ": armed ", b.armed ? 1u : 0u, ", moved ",
          b.moved ? 1u : 0u, " (the sink's low byte ", hex(b.beat), "), busy ", b.busy ? 1u : 0u,
          ", waiting ", b.waiting ? 1u : 0u, ", DMAC flags ", hex(b.flags), crlf);
}

/// The loop's engined transport up, a run out and drained to its last
/// stop bit, the receiver's ring emptied: the engine's trigger standing.
bool stand_trigger(uint32_t baud, uint32_t n) {
    loop_live = Loop::txdma;
    if (!LTxDma::init(clock, baud)) {
        return false;
    }
    loop_settle(baud);
    static uint8_t out[64];
    const uint32_t len = n < sizeof out ? n : static_cast<uint32_t>(sizeof out);
    for (uint32_t i = 0; i < len; ++i) {
        out[i] = static_cast<uint8_t>(0x30u + i);
    }
    (void)LTxDma::write_bulk(std::span<const uint8_t>(out, len));
    uint32_t spins = 0;
    while (!LTxDma::tx_idle() && spins++ < 4'000'000u) {
    }
    loop_settle(baud);
    uint8_t sink[64];
    while (LTxDma::read_bulk(std::span<uint8_t>(sink, sizeof sink)) != 0u) {
    }
    return LTxDma::tx_idle() && Sc1::dre_flag();
}

/// The three releases letter w compares.
enum class Way : uint8_t { disabled, enabled, reset };

/// A release staged by hand: the engine's release, then the resource's
/// teardown WITHOUT CTRLA.SWRST - interrupts off, disabled unless
/// `keep_enabled`, the core clock and the bus clock gated - then the pin,
/// in Uart::release()'s order.
void release_unreset(bool keep_enabled) {
    DmaTxEngine::release(loop_owner);
    Nvic::disable(Sc1::irq());
    Sc1::enable_interrupt(SercomFlag::all, false);
    if (!keep_enabled) {
        (void)Sc1::enable(false);
    }
    GclkChannel::disconnect(Sc1::gclk_core_id());
    Sc1::bus_clock(false);
    Pin<'A', 18>::release();
}

/// What a released SERCOM1 holds, read through its bus clock alone (APB
/// registers answer without the core clock) and gated again.
struct Held {
    uint8_t flags;
    uint32_t ctrla;
};
Held read_released() {
    Sc1::bus_clock(true);
    const Held h{Sc1::flags(), Sc1::regs().SERCOM_CTRLA};
    Sc1::bus_clock(false);
    return h;
}

struct Handover {
    uint32_t got;
    uint32_t bad;
    uint8_t faults;
    bool idle;
};

/// The next owner: the Uart on SERCOM1 again, a run of 256 out on the loop
/// and read back - every byte, in order, nothing more a millisecond after.
Handover next_uart(uint32_t baud) {
    Handover h{};
    loop_live = Loop::txdma;
    if (!LTxDma::init(clock, baud)) {
        loop_live = Loop::none;
        return h;
    }
    loop_settle(baud);
    uint32_t s = lfsr_seed;
    static uint8_t out[256];
    for (uint8_t& b : out) {
        b = lfsr_next(s);
    }
    (void)LTxDma::write_bulk(std::span<const uint8_t>(out, sizeof out));
    uint32_t spins = 0;
    while (!LTxDma::tx_idle() && spins++ < 4'000'000u) {
    }
    h.idle = LTxDma::tx_idle();
    loop_settle(baud);
    spin_ms(1);
    uint32_t r = lfsr_seed;
    uint8_t in[64];
    for (;;) {
        const uint32_t got = LTxDma::read_bulk(std::span<uint8_t>(in, sizeof in));
        if (got == 0u) {
            break;
        }
        for (uint32_t i = 0; i < got; ++i) {
            h.bad += in[i] != lfsr_next(r) ? 1u : 0u;
        }
        h.got += got;
    }
    h.faults = LTxDma::dma_faults();
    LTxDma::release();
    loop_live = Loop::none;
    return h;
}

void tw_released_requester() {
    // ---- the control: does the channel keep a rise? ----------------------
    constexpr uint32_t slow = 115'200u;
    const bool stood = stand_trigger(slow, 16);
    // DATA filled by hand: the first write moves to the idle shifter at
    // once (DRE falls and rises again while the channel is disabled), the
    // second stays in DATA behind it: DRE low.
    Sc1::data(0xC1u);
    uint32_t spins = 0;
    while (!Sc1::dre_flag() && spins++ < 10'000u) {
    }
    Sc1::data(0xC2u);
    const bool dre_low = !Sc1::dre_flag();
    static const uint8_t one[1] = {0xB1u};
    (void)LTxDma::write_bulk(std::span<const uint8_t>(one, 1));
    spins = 0;
    while (!LTxDma::tx_idle() && spins++ < 4'000'000u) {
    }
    loop_settle(slow);
    uint8_t seen[8] = {};
    const uint32_t n_seen = LTxDma::read_bulk(std::span<uint8_t>(seen, sizeof seen));
    LTxDma::release();
    loop_live = Loop::none;
    print(plain, "  control at ", slow, ": trigger standing ", stood ? 1u : 0u,
          ", DATA filled by hand (DRE low ", dre_low ? 1u : 0u,
          "), a block of one started: the loop read ", n_seen, " bytes:");
    for (uint32_t i = 0; i < n_seen; ++i) {
        print(plain, " ", hex(seen[i]));
    }
    print(plain, crlf);

    // ---- the three releases, then the channel bound again ----------------
    constexpr uint32_t baud = 1'000'000u;
    static const char* const names[] = {"disabled and gated, no reset",
                                        "gated with ENABLE set, no reset",
                                        "Uart::release(), the reset before the gate"};
    Bound none[3]{};
    Bound own[3]{};
    Held held[3]{};
    Handover next[3]{};
    bool stood_all = true;
    for (uint8_t k = 0; k < 3u; ++k) {
        const Way way = static_cast<Way>(k);
        stood_all = stand_trigger(baud, 64) && stood_all;
        if (way == Way::reset) {
            LTxDma::release();
        } else {
            release_unreset(way == Way::enabled);
        }
        loop_live = Loop::none;
        held[k] = read_released();
        none[k] = bind_probe(0u);
        own[k] = bind_probe(Sc1::dma_tx_trigger());
        next[k] = next_uart(baud);
    }
    bool none_still = true;
    bool next_whole = true;
    for (uint8_t k = 0; k < 3u; ++k) {
        print(plain, "  released ", names[k], ": the gated SERCOM1 reads INTFLAG ",
              hex(held[k].flags), ", CTRLA ", hex(held[k].ctrla), crlf);
        print_bound("no trigger", none[k]);
        print_bound("the released SERCOM1's own", own[k]);
        print(plain, "    the next Uart on SERCOM1: ", next[k].got, " of 256 back, ", next[k].bad,
              " off the pattern, faults ", next[k].faults, next[k].idle ? "" : ", NOT idle", crlf);
        none_still = none_still && none[k].armed && !none[k].moved;
        next_whole = next_whole && next[k].got == 256u && next[k].bad == 0u &&
                     next[k].faults == 0u && next[k].idle;
    }
    bench.verdict("the trigger stood at every release: the last block over and DRE up",
                  stood && stood_all);
    bench.verdict("THE CHANNEL KEEPS NO RISE: one that came and went while it was disabled fired "
                  "no beat at its enable - the block of one waited for DATA to empty, and the loop "
                  "read 0xC1, 0xC2, 0xB1 in order",
                  dre_low && n_seen == 3u && seen[0] == 0xC1u && seen[1] == 0xC2u &&
                      seen[2] == 0xB1u);
    bench.verdict("a disable withdraws the transmit request: the gated instance reads DRE clear, "
                  "its configuration standing, and the channel bound to its trigger moves nothing",
                  (held[0].flags & SercomFlag::dre) == 0u && held[0].ctrla != 0u &&
                      own[0].armed && !own[0].moved);
    bench.verdict("A GATED CLOCK HOLDS A REQUEST LEFT STANDING: gated with ENABLE set, the instance "
                  "reads DRE up behind the gate, and the channel bound to its trigger takes ONE "
                  "stray beat - the next block's first byte - and waits",
                  (held[1].flags & SercomFlag::dre) != 0u && own[1].armed && own[1].moved &&
                      own[1].beat == probe_run[0] && own[1].waiting);
    bench.verdict("Uart::release() leaves the instance reset behind the gate - INTFLAG and CTRLA "
                  "zero - and the channel bound to its trigger moves nothing",
                  held[2].flags == 0u && held[2].ctrla == 0u && own[2].armed && !own[2].moved);
    bench.verdict("bound to no trigger after any of the three releases, the channel moves nothing "
                  "at its enable",
                  none_still);
    bench.verdict("the next Uart's first block after any of them: 256 of 256 in order, no fault - "
                  "the hand-over needs nothing of the next owner",
                  next_whole);
    bench.verdict("and the loop runs at the rate asked: 1 Mbaud, a run of frames in its wire time "
                  "within -1 % and +3 %",
                  wire_rungs({baud}));
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
// THE TWO-BOARD HALF: SERCOM1 on the desk's four wires, a far UART on a
// crystal at the other end (the second all-key, y)
// =============================================================================
//
// THE FAR END is an AVR128DA48 running avrxt's usart_peer (usart_link.hpp,
// kind 0x01): its USART4 on the crossed pair, its time base its own 24 MHz
// quartz clock, driven IN BAND over the very link under test - the client half
// below is test_avr_serial's, speaking the same protocol header. What it
// gives this suite that the loop and the host bridge cannot: a FOREIGN
// receiver on a crystal, which times this SERCOM's frames (the Report's
// span), a different UART for the frame matrix, a cycle-counted bit-banger
// for the errors no clean UART makes, a synchronous partner on a real XCK,
// an IrDA coder of the other sense, a shared wire with a second
// transmitter on it, and a foreign auto-baud sender.
//
// THE WIRES (B-C, fixed by the desk): PA16 = SERCOM1 PAD[0] - the far
// TxD (PE0); PA18 = PAD[2] - the far RxD (PE1); PA19 = PAD[3] - the far
// XCK (PE2); PA17 = PAD[1] - the far PE3. The crossed pair is TxD on
// PAD[2] (TXPO 0x1), RxD on PAD[0], XCK on PAD[3] beside TxD. THE SHARED
// WIRE is PA16 - PE0, both boards' one-wire pad: TxD and RxD on PAD[0]
// (TXPO 0x0, RXPO 0x0) to talk, TxD moved to the unmuxed PAD[2] (TXPO 0x1,
// PA18 released to PORT) to listen - a SERCOM transmitter drives its pad
// at the idle level even disabled (docs/samc21/sercom.md, measured), so a
// pad that must carry no driver is taken from it. The RS-485 and one-wire
// letters run there: they first stay silent for link::rediscover_ms, so
// the far peer stops believing the crossed pair and drives nothing on PE0,
// find it again on the shared wire, and hand it back the same way.
//
// THE CORE CLOCK of every two-board configuration is GCLK generator 2 on
// the 24 MHz XOSC (the CPU stays on OSC48M): the far end's crystal is the
// reference these letters are measured against, and with both rates from
// crystals the span judges this SERCOM's divisor. One rung of the rate
// letter runs on generator 0 instead - OSC48M, the console's regime - and
// reads that oscillator's own offset.
//
// THE DISCIPLINE (docs/design R1, one driver per wire): on the crossed pair
// this board drives PA18 (and PA19 as the synchronous host) and nothing
// the far board drives; on the shared wire it drives PA16 only while it
// talks. A leg the peer REFUSES (report_cfg_failed) is SKIPPED with the
// reason, never judged. Without a peer the first verdict of every letter
// fails, loudly.

// The protocol header is shared, not copied (it is pure encoding).
using Lk = Sercom<1>;
using RxdPin = Pin<'A', 16>;    // PAD[0]: RxD; the shared wire
using Pad1Pin = Pin<'A', 17>;   // PAD[1]
using TxdPin = Pin<'A', 18>;    // PAD[2]: TxD; TE; RTS
using XckPin = Pin<'A', 19>;    // PAD[3]: XCK; CTS
using LinkPort = Port<'A'>;
static_assert(MUX_PA16C_SERCOM1_PAD0 == static_cast<uint8_t>(PinFunction::c));
static_assert(MUX_PA17C_SERCOM1_PAD1 == static_cast<uint8_t>(PinFunction::c));
static_assert(MUX_PA18C_SERCOM1_PAD2 == static_cast<uint8_t>(PinFunction::c));
static_assert(MUX_PA19C_SERCOM1_PAD3 == static_cast<uint8_t>(PinFunction::c));

constexpr uint32_t link_pins = RxdPin::mask | Pad1Pin::mask | TxdPin::mask | XckPin::mask;

/// The crossed pair: TxD PAD[2], RxD PAD[0], XCK PAD[3].
constexpr UartPads link_pads{
    .tx = SercomPad::pad2,
    .rx = SercomPad::pad0,
    .tx_pin = {'A', 18, PinFunction::c},
    .rx_pin = {'A', 16, PinFunction::c},
};
/// The shared wire, talking: both directions on PAD[0].
constexpr UartPads wire_pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad0,
    .tx_pin = {'A', 16, PinFunction::c},
    .rx_pin = {'A', 16, PinFunction::c},
};
static_assert(uart_xck_pad(link_pads.tx) == SercomPad::pad3);

constexpr uint8_t gen_xtal = 2;
constexpr uint32_t xtal_hz = 24'000'000UL;
constexpr SercomBaud command_baud = sercom_closest_baud(xtal_hz, link::command_baud).value();
static_assert(command_baud.sampling == UartSampling::x16_arithmetic && command_baud.reg == 60503);

bool xtal_tried = false;
bool xtal_ok = false;

/// The crystal on generator 2, started once (the z letters never use it).
bool xtal_up() {
    if (!xtal_tried) {
        xtal_tried = true;
        xtal_ok = Xosc::init(XoscConfig{.hz = xtal_hz, .startup = 4, .on_demand = false}) &&
                  Gclk<gen_xtal>::configure(GclkConfig{.source = GclkSource::xosc});
    }
    return xtal_ok;
}

// ---- the link's state ------------------------------------------------------

link::Topology topo = link::Topology::full_duplex;
bool topo_known = false;
bool link_quiet = false;
/// The peer refused the leg in hand (usart_link.hpp: report_cfg_failed):
/// every judgement of it prints SKIP and counts nothing until the next
/// peer_act() or letter.
bool leg_refused = false;
link::Decoder dec;
/// When the last answer of the peer was heard: the start of a silence.
uint32_t last_answer_ms = 0;

bool shared_line() { return topo == link::Topology::shared; }

/// The configuration the DUT's end is in, and the bit time it runs at.
SercomUartConfig cur{};
uint32_t cur_bit_cycles = SysClock::hz / link::command_baud;

void judge(const char* name, bool ok) {
    if (leg_refused) {
        print(plain, "  SKIP  ", name, crlf);
        return;
    }
    bench.verdict(name, ok);
}
void judge(const char* a, const char* b, bool ok) {
    if (leg_refused) {
        print(plain, "  SKIP  ", a, b, crlf);
        return;
    }
    bench.verdict(a, b, ok);
}

/// Decimal for a verdict label (one per expression).
char num_buf[12];
const char* num(uint32_t v) {
    uint8_t i = 11;
    num_buf[i] = 0;
    if (v == 0u) {
        num_buf[--i] = '0';
    }
    while (v != 0u && i != 0u) {
        num_buf[--i] = static_cast<char>('0' + (v % 10u));
        v /= 10u;
    }
    return &num_buf[i];
}

void spin_us(uint32_t us) {
    const uint32_t t0 = cycles_now();
    const uint32_t want = us * (SysClock::hz / 1'000'000u);
    while (cycles_now() - t0 < want) {
    }
}

// ---- the pads --------------------------------------------------------------

/// All four link pads to PORT, inputs with no pull: nothing on the wires.
void pads_dark() {
    RxdPin::release();
    Pad1Pin::release();
    TxdPin::release();
    XckPin::release();
    LinkPort::dir_clear(link_pins);
    RxdPin::input();
    Pad1Pin::input();
    TxdPin::input();
    XckPin::input();
}

/// One pad to PORT, an input with no pull.
template <typename P>
void pad_dark() {
    P::release();
    LinkPort::dir_clear(P::mask);
    P::input();
}

/// The crossed pair: RxD and TxD to the SERCOM, XCK to it for a
/// synchronous role, PA17 dark. Called with the instance DISABLED
/// (dut_up()): a receiver whose pad leaves the mux while it runs reads the
/// unmuxed input low - a start bit, and a frame of garbage after the
/// flush (measured: a spurious 0xFF ahead of every echo at 115200).
///
/// WHILE THE FAR PEER SEARCHES (`rx_held`), RxD carries the SERCOM's
/// PULL-DOWN (31.5.1: the one pull a SERCOM input has). The far TxD is then
/// an input, and with no pull at either end the PE0 - PA16 wire floats
/// beside PE1 - PA18, which this end drives: the far receiver, listening
/// to PE0 through its loop-back in its shared dwell, decoded a ping off the
/// floating wire, proved the SHARED wiring and answered there - and then
/// heard nothing on the crossed pair until it gave that wiring up
/// (measured, intermittently: a letter whose first command after the ping
/// went unanswered). Once the far end has answered on the crossed pair it
/// drives the wire, and the pull comes off: held low, every release of the
/// far TxD in a reconfiguration was a falling edge and a frame of garbage
/// here (measured: one stray 0xFE-like frame behind every far burst).
bool rx_held = false;
void pads_crossed(bool sync) {
    LinkPort::dir_clear(link_pins);
    RxdPin::function(PinFunction::c, {.input_enable = true,
                                      .pull = rx_held ? PinPull::down : PinPull::none});
    TxdPin::function(PinFunction::c, {.input_enable = true});
    if (sync) {
        XckPin::function(PinFunction::c, {.input_enable = true});
    } else {
        pad_dark<XckPin>();
    }
    pad_dark<Pad1Pin>();
}

/// The shared wire: PA16 to the SERCOM, the other three dark - PA18, the
/// far RxD's wire, held low by a PORT pull-down, so the far receiver in
/// its crossed-pair dwell decodes nothing off this end's traffic on the
/// wire beside it (pads_crossed(), the same floating pair the other way
/// round). PA16 itself is the far pull-up's once that end has proved the
/// shared wire.
void pads_wire() {
    LinkPort::dir_clear(link_pins);
    RxdPin::function(PinFunction::c, {.input_enable = true});
    pad_dark<Pad1Pin>();
    pad_dark<TxdPin>();
    TxdPin::input(PinPull::down);
    pad_dark<XckPin>();
}

// ---- the resource ----------------------------------------------------------

/// SERCOM1 polled for the two-board letters - its vector off, the loop's
/// transports down, its core on the crystal. False when the crystal did
/// not start (every two-board figure would be OSC48M's).
bool link_begin() {
    Nvic::disable(Lk::irq());
    loop_live = Loop::none;
    if (!xtal_up()) {
        return false;
    }
    Lk::bus_clock(true);
    return Lk::core_clock(gen_xtal) && Lk::reset();
}

/// SERCOM1 handed back: reset, clocks off, every link pad dark.
void link_end() {
    Lk::release();
    pads_dark();
}

/// Write `c`, the instance left disabled (configure()'s contract).
bool write(const SercomUartConfig& c) {
    return c.collision_detect
               ? Lk::configure(c, SercomClocks{.apb_hz = SysClock::hz, .core_hz = xtal_hz})
               : Lk::configure(c);
}

/// Enable after write(). On an external clock the enable's second half
/// waits for XCK (docs/samc21/sercom.md): bounded short, judged by the
/// enable bit.
bool start(const SercomUartConfig& c) {
    if (c.external_clock) {
        (void)Lk::enable(true, 64u);
        return Lk::enabled();
    }
    return Lk::enable(true);
}

/// Write `c` and enable, the pads as they are.
bool apply(const SercomUartConfig& c) { return write(c) && start(c); }

/// The shared wire's two halves of one configuration: TxD on PAD[0] to
/// talk, on the unmuxed PAD[2] to listen.
SercomUartConfig talk_cfg(const SercomUartConfig& c) {
    SercomUartConfig t = c;
    t.pads = wire_pads;
    return t;
}
SercomUartConfig listen_cfg(const SercomUartConfig& c) {
    SercomUartConfig l = c;
    l.pads = link_pads;
    l.collision_detect = false;
    return l;
}

/// One frame out, DRE bounded.
[[gnu::always_inline]] inline bool put_frame(uint16_t v) {
    for (uint32_t n = 400'000u; !Lk::dre_flag(); --n) {
        if (n == 0u) {
            return false;
        }
    }
    Lk::data(v);
    return true;
}

/// The last stop bit gone: TXC, bounded (only after a frame was written).
/// Whether it came is kept for a link failure's report.
bool tx_done_seen = true;
void wait_tx_done() {
    const uint32_t t0 = cycles_now();
    while (!Lk::txc_flag() && cycles_now() - t0 < ms_cycles(20)) {
    }
    tx_done_seen = Lk::txc_flag();
}

/// Four bit times before taking the line: RXC rises in the middle of the
/// far stop bit, half a bit before the far transmitter lets go.
void turnaround_guard() {
    const uint32_t t0 = cycles_now();
    while (cycles_now() - t0 < 4u * cur_bit_cycles) {
    }
}

/// Take the line. On the crossed pair the transmitter is already the
/// SERCOM's; on the shared wire TxD comes onto PAD[0].
void line_talk() {
    turnaround_guard();
    if (shared_line()) {
        (void)apply(talk_cfg(cur));
    }
    Lk::clear_flags(SercomFlag::txc);
}

/// Give it back: the last stop bit out, and on the shared wire TxD off
/// the pad and what the loop heard of it dropped.
void line_listen() {
    wait_tx_done();
    if (shared_line()) {
        (void)apply(listen_cfg(cur));
        Lk::flush_rx();
    }
}

/// The DUT's end as `c` says, at `rate` bits a second (the guard's unit).
bool dut_up(const SercomUartConfig& c, uint32_t rate) {
    cur = c;
    cur_bit_cycles = SysClock::hz / (rate != 0u ? rate : link::command_baud);
    // The pads move with the instance disabled, between the write and the
    // enable (pads_crossed()).
    const SercomUartConfig w = shared_line() ? listen_cfg(c) : c;
    bool ok = write(w);
    if (shared_line()) {
        pads_wire();
    } else {
        pads_crossed(c.synchronous);
    }
    ok = ok && start(w);
    Lk::flush_rx();
    Lk::clear_status(SercomStatus::receive_errors | SercomStatus::sync_field_error |
                     SercomStatus::collision);
    Lk::clear_flags(SercomFlag::all);
    return ok;
}

/// Command mode: 8N1 at link::command_baud on the crystal.
bool link_command_mode() {
    const bool ok = dut_up({.pads = link_pads, .baud = command_baud.reg,
                            .sampling = command_baud.sampling},
                           link::command_baud);
    dec.reset();
    return ok;
}

// ---- the protocol's client half ---------------------------------------------

void put_link(uint8_t b) { (void)put_frame(b); }

/// One received character, its STATUS read first (31.8.11) and cleared.
struct Got {
    bool any = false;
    bool clean = false;
    uint16_t data = 0;
    uint16_t status = 0;
};
[[gnu::always_inline]] inline Got take() {
    Got g{};
    if (!Lk::rxc_flag()) {
        return g;
    }
    g.any = true;
    g.status = Lk::status();
    g.data = Lk::data();
    const uint16_t err = static_cast<uint16_t>(g.status & SercomStatus::receive_errors);
    if (err != 0u) {
        Lk::clear_status(err);
    }
    g.clean = err == 0u;
    return g;
}

/// What the last receive window heard, for a failure's report: a lost
/// command with bytes on the line is a decoding problem, one with none a
/// peer that never answered.
uint8_t raw_seen[12];
uint8_t raw_n = 0;

bool recv_frame(link::Frame& out, uint16_t ms) {
    const uint32_t t0 = Ticker::millis();
    dec.reset();
    raw_n = 0;
    while (Ticker::millis() - t0 < ms) {
        const Got g = take();
        if (!g.any) {
            continue;
        }
        if (raw_n < sizeof raw_seen) {
            raw_seen[raw_n++] = g.clean ? static_cast<uint8_t>(g.data) : 0xEEu;
        }
        if (!g.clean) {
            dec.reset();
            continue;
        }
        if (dec.feed(static_cast<uint8_t>(g.data)) == link::Decoder::Result::frame) {
            out = dec.frame();
            last_answer_ms = Ticker::millis();
            return true;
        }
    }
    return false;
}

const uint8_t no_payload[1] = {0};

bool command_once(link::Op op, const uint8_t* p, uint8_t len, uint16_t ms) {
    Lk::flush_rx();
    line_talk();
    link::write_frame(put_link, op, p, len);
    line_listen();
    link::Frame f;
    if (!recv_frame(f, ms)) {
        return false;
    }
    return f.op == link::Op::ack && f.len == 2 && f.data[0] == link::byte_of(op);
}

/// The recovery guarantee: three attempts, each after a quiet interval
/// longer than the peer's own resynchronization.
bool command(link::Op op, const uint8_t* p = no_payload, uint8_t len = 0, uint16_t ms = 80) {
    uint8_t first_n = 0;
    uint8_t first_seen[sizeof raw_seen];
    for (uint8_t k = 0; k < 3; ++k) {
        if (command_once(op, p, len, ms)) {
            return true;
        }
        if (k == 0u) {
            first_n = raw_n;
            for (uint8_t i = 0; i < raw_n; ++i) first_seen[i] = raw_seen[i];
        }
        (void)link_command_mode();
        spin_ms(70);
    }
    if (!link_quiet) {
        print(plain, "    LINK FAILURE op ", hex(link::byte_of(op)), " (",
              shared_line() ? "shared wire" : "crossed pair", "), heard ", first_n, ":");
        for (uint8_t i = 0; i < first_n; ++i) print(plain, " ", hex(first_seen[i]));
        print(plain, " (CTRLA ", hex(Lk::regs().SERCOM_CTRLA), " BAUD ", Lk::baud_reg(),
              " SYNCBUSY ", hex(Lk::regs().SERCOM_SYNCBUSY), " INTFLAG ", hex(Lk::flags()),
              " STATUS ", hex(Lk::status()), ", the frame's TXC ", tx_done_seen ? "seen" : "NOT seen",
              ", XOSC ", Xosc::ready() ? "ready" : "NOT READY", ", pads ",
              RxdPin::read() ? "PA16 high " : "PA16 low ", TxdPin::read() ? "PA18 high" : "PA18 low",
              ")", crlf);
    }
    (void)link_command_mode();
    return false;
}

bool query(link::Op op, link::Frame& data, uint16_t ms = 80) {
    if (!command(op, no_payload, 0, ms)) {
        return false;
    }
    return recv_frame(data, ms);
}

/// Find the peer on the desk's crossed pair - for as long as a peer that
/// still believes the shared wire takes to give it up and search again.
bool ensure_link() {
    if (!link_begin()) {
        print(plain, "  the 24 MHz crystal did not start: no two-board figure means anything",
              crlf);
        return false;
    }
    link_quiet = true;
    if (!topo_known) {
        topo = link::Topology::full_duplex;
    }
    rx_held = !shared_line();
    (void)link_command_mode();
    const uint32_t t0 = Ticker::millis();
    const uint32_t most = static_cast<uint32_t>(link::rediscover_ms) + 2'500u;
    while (Ticker::millis() - t0 < most) {
        if (command(link::Op::ping)) {
            topo_known = true;
            link_quiet = false;
            rx_held = false;
            (void)link_command_mode();
            return true;
        }
    }
    rx_held = false;
    link_quiet = false;
    topo_known = false;
    print(plain, "  the peer did not answer on the ", shared_line() ? "shared wire" : "crossed pair",
          ": the far board must run avrxt's usart_peer", crlf);
    return false;
}

bool peer_ident(link::Ident& d) {
    link::Frame f;
    if (!query(link::Op::ident, f) || f.op != link::Op::ident_data || f.len != link::ident_size) {
        return false;
    }
    d = link::get_ident(f.data);
    return true;
}

/// The report of the last action, after the action's bound has had time.
/// A refusal raises leg_refused, says so, and the leg is skipped.
bool peer_report(link::Report& r, uint16_t wait_ms) {
    (void)link_command_mode();
    if (wait_ms != 0u) {
        spin_ms(wait_ms);
    }
    for (uint8_t k = 0; k < 4; ++k) {
        link::Frame f;
        const bool answered = query(link::Op::report, f) && f.op == link::Op::report_data;
        if (answered && f.len != link::report_size) {
            print(plain, "  the peer's REPORT is ", f.len, " bytes, this suite reads ",
                  link::report_size, " - flash both boards from one tree", crlf);
            return false;
        }
        if (answered) {
            r = link::get_report(f.data);
            if ((r.flags & link::report_cfg_failed) != 0u && !leg_refused) {
                leg_refused = true;
                print(plain, "  SKIPPED, no verdict claimed: the peer REFUSED op ", hex(r.op),
                      " (report_cfg_failed) - a configuration its silicon cannot serve", crlf);
            }
            return true;
        }
        spin_ms(40);
    }
    return false;
}

/// Command an action; the peer takes up its end link::settle_ms after its
/// ack, so this end waits that long before taking up its own.
bool peer_act(link::Op op, const link::Params& a) {
    leg_refused = false;
    uint8_t p[link::max_payload] = {};
    link::put_params(p, a);
    if (!command(op, p, link::params_size)) {
        return false;
    }
    spin_ms(link::settle_ms);
    return true;
}

/// The peer opens its window link::settle_ms + 2 ms after its ack and
/// flushes what this end's reconfiguration left on the line: transmit
/// after it.
void wait_for_peer_window() { spin_ms(6); }

/// Milliseconds a frame count needs at a rate, generously.
uint16_t bound_ms(uint32_t rate, uint32_t frames) {
    const uint64_t bits = 12ull * frames;
    const uint64_t ms = (bits * 1000u * 4u) / (rate != 0u ? rate : 1u) + 80u;
    return ms > 4000u ? static_cast<uint16_t>(4000u) : static_cast<uint16_t>(ms);
}

// ---- the shared wire ----------------------------------------------------------

/// Silence for link::rediscover_ms after a fresh answer: the peer stops
/// believing the wiring it last proved and searches, driving nothing.
void wait_rediscovery() {
    (void)command(link::Op::ping);
    const uint32_t from = last_answer_ms;
    const uint32_t quiet = static_cast<uint32_t>(link::rediscover_ms) + 600u;
    while (Ticker::millis() - from < quiet) {
    }
}

/// Onto the shared wire: the crossed pair let go, the peer found on PE0.
bool enter_shared() {
    if (!ensure_link()) {
        return false;
    }
    if (shared_line()) {
        return true;
    }
    print(plain, "  (silent ", link::rediscover_ms, " ms, then the peer on the shared wire)", crlf);
    wait_rediscovery();
    topo = link::Topology::shared;
    (void)link_command_mode();
    link_quiet = true;
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 3'000u) {
        if (command(link::Op::ping)) {
            link_quiet = false;
            topo_known = true;
            return true;
        }
    }
    link_quiet = false;
    topo = link::Topology::full_duplex;
    topo_known = false;
    print(plain, "  the peer did not answer on the shared wire", crlf);
    return false;
}

/// Back to the crossed pair, the same way.
void leave_shared() {
    if (!shared_line()) {
        return;
    }
    wait_rediscovery();
    topo = link::Topology::full_duplex;
    (void)ensure_link();
}

// ---- the DUT's end, from a Cfg ----------------------------------------------

UartBits bits_of(uint8_t n) {
    switch (n) {
        case 5: return UartBits::five;
        case 6: return UartBits::six;
        case 7: return UartBits::seven;
        case link::bits9_low:
        case link::bits9_high: return UartBits::nine;
        default: return UartBits::eight;
    }
}
UartParity parity_of(uint8_t n) {
    return n == 1u ? UartParity::even : (n == 2u ? UartParity::odd : UartParity::none);
}
uint16_t mask_of(uint8_t bits) {
    const uint8_t n = bits >= link::bits9_low ? 9u : bits;
    return static_cast<uint16_t>((1u << n) - 1u);
}

/// The DUT's configuration for a Cfg, its mode naming the DUT's OWN role.
std::optional<SercomUartConfig> cfg_of(const link::Cfg& c) {
    SercomUartConfig u{.pads = link_pads,
                       .format = {.bits = bits_of(c.bits), .parity = parity_of(c.parity),
                                  .two_stop = c.stop >= 2u}};
    switch (c.mode) {
        case link::Mode::sync_host: {
            const auto reg = sercom_sync_baud_reg(xtal_hz, c.rate);
            if (!reg) return {};
            u.synchronous = true;
            u.baud = *reg;
            u.invert_clock = (c.flags & link::flag_invert_xck) != 0u;
            break;
        }
        case link::Mode::sync_client:
            u.synchronous = true;
            u.external_clock = true;
            u.invert_clock = (c.flags & link::flag_invert_xck) != 0u;
            break;
        case link::Mode::ircom: {
            const auto reg = sercom_baud_reg(xtal_hz, c.rate);
            if (!reg) return {};
            u.baud = *reg;
            u.irda = true;
            u.rx_pulse = c.rxpl;
            break;
        }
        default: {
            const auto b = sercom_closest_baud(xtal_hz, c.rate,
                                               (c.flags & link::flag_clk2x) != 0u ? 8 : 16);
            if (!b) return {};
            u.baud = b->reg;
            u.sampling = b->sampling;
            break;
        }
    }
    if (!sercom_uart_config_valid(u)) {
        return {};
    }
    return u;
}

bool dut_link(const link::Cfg& c) {
    const auto u = cfg_of(c);
    return u && dut_up(*u, c.rate);
}

/// What came in over a window, read live (two levels of buffer hold
/// nothing longer).
struct Drain {
    uint16_t frames = 0;
    uint16_t ferr = 0;
    uint16_t perr = 0;
    uint16_t ovf = 0;
    uint16_t first = 0;
    uint16_t last = 0;
    uint16_t sum = 0;
};
void count(Drain& d, const Got& g) {
    if (d.frames == 0u) {
        d.first = g.data;
    }
    ++d.frames;
    if ((g.status & SercomStatus::frame_error) != 0u) ++d.ferr;
    if ((g.status & SercomStatus::parity_error) != 0u) ++d.perr;
    if ((g.status & SercomStatus::overflow) != 0u) ++d.ovf;
    d.last = g.data;
    d.sum = static_cast<uint16_t>(d.sum + g.data);
}
Drain collect(uint32_t ms) {
    Drain d{};
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
        const Got g = take();
        if (g.any) {
            count(d, g);
        }
    }
    return d;
}

/// Lock-step against the peer's echo: one frame out, its echo back.
uint16_t echo_first_sent = 0, echo_first_got = 0;
uint16_t echo_got[8] = {};
uint16_t echo_mismatches(uint8_t bits, uint16_t n) {
    const uint16_t mask = mask_of(bits);
    uint16_t bad = 0;
    wait_for_peer_window();
    echo_first_sent = echo_first_got = 0;
    const uint32_t wait = 40u * cur_bit_cycles + ms_cycles(2);
    for (uint16_t i = 0; i < n; ++i) {
        const uint16_t v = static_cast<uint16_t>((0x5Au + i * 37u) & mask);
        line_talk();
        const bool sent = put_frame(v);
        line_listen();
        Got g{};
        const uint32_t t0 = cycles_now();
        while (sent && cycles_now() - t0 < wait) {
            g = take();
            if (g.any) {
                break;
            }
        }
        if (i < 8u) {
            echo_got[i] = g.any ? static_cast<uint16_t>(g.data | (g.clean ? 0u : 0x8000u)) : 0xFFFFu;
        }
        if (!g.any || !g.clean || (g.data & mask) != v) {
            if (bad == 0u) {
                echo_first_sent = v;
                echo_first_got = g.any ? g.data : 0xFFFFu;
            }
            ++bad;
        }
    }
    return bad;
}

/// `n` frames gapless - DATA refilled the moment DRE stands - and the
/// last stop bit out. The values walk from `v0`; their sum is returned.
///
/// MASKED, the measurement's own feeder: the transmit buffer gives one
/// frame of slack, 160 cycles at 3 Mbaud, and one SysTick entry outlasts
/// it (measured: a burst fed unmasked ran -850 ppm at 3 Mbaud against the
/// far crystal, the same burst masked runs as the register predicts). The
/// ticker's ticks of the run are lost - its milliseconds lag the wall
/// after a long burst, which only lengthens the waits that follow.
uint16_t burst(uint32_t n, uint16_t v0, uint16_t mask) {
    uint16_t sum = 0;
    {
        SamPlatform::CriticalSection cs;
        for (uint32_t i = 0; i < n; ++i) {
            const uint16_t v = static_cast<uint16_t>((v0 + i) & mask);
            while (!Lk::dre_flag()) {
            }
            Lk::data(v);
            sum = static_cast<uint16_t>(sum + v);
        }
    }
    wait_tx_done();
    return sum;
}

/// THE GAPLESS RUN for the rate letter: the Uart's transmit engine on the
/// link's pads (the stratum's one DMA user, samc21/dmac.hpp), DATA fed by
/// the channel's beats as DRE rises - x 1.00 of the wire at every rate the
/// generator makes, where a polled feeder is the CPU's at 3 Mbaud: one
/// SysTick entry outlasts the one frame of slack the transmit buffer gives
/// it (measured: -850 to -955 ppm at 3 Mbaud, polled). The task comes up
/// as the console's regime and the rung's configuration is written under
/// it - the generator and the regime the letter judges -, the engine left
/// armed. `n` frames of 0x30, 0x31, ... and their sum, or nullopt.
using LkDma = Uart<1, link_pads, 64, 512, DmaTxEngine>;

std::optional<uint16_t> engine_burst(uint32_t n, const SercomUartConfig& u, uint8_t generator) {
    loop_live = Loop::link_dma;
    bool ok = LkDma::init(clock, 115'200u);
    if (ok) {
        (void)Lk::enable(false);
        ok = Lk::core_clock(generator) && Lk::configure(u) && Lk::enable(true);
    }
    uint8_t chunk[128];
    uint16_t sum = 0;
    uint32_t q = 0;
    const uint32_t t0 = cycles_now();
    const uint32_t most = ms_cycles(4000);
    while (ok && q < n) {
        const uint32_t k = n - q < sizeof chunk ? n - q : static_cast<uint32_t>(sizeof chunk);
        for (uint32_t i = 0; i < k; ++i) {
            chunk[i] = static_cast<uint8_t>(0x30u + q + i);
            sum = static_cast<uint16_t>(sum + chunk[i]);
        }
        uint32_t done = 0;
        while (done < k && cycles_now() - t0 < most) {
            done += LkDma::write_bulk(std::span<const uint8_t>(chunk + done, k - done));
        }
        ok = done == k;
        q += k;
    }
    while (ok && !LkDma::tx_idle() && cycles_now() - t0 < most) {
        (void)LkDma::write_bulk(std::span<const uint8_t>());
    }
    ok = ok && LkDma::tx_idle() && LkDma::dma_faults() == 0u;
    LkDma::release();
    loop_live = Loop::none;
    ok = link_begin() && ok;
    if (!ok) {
        return {};
    }
    return sum;
}

void print_ppm(int32_t ppm) {
    if (ppm < 0) {
        print(plain, "-", static_cast<uint32_t>(-ppm));
    } else {
        print(plain, "+", static_cast<uint32_t>(ppm));
    }
}

int32_t abs32(int32_t v) { return v < 0 ? -v : v; }

/// Bit times in hundredths, from 24 MHz ticks at a rate.
uint32_t centibits(uint32_t ticks, uint32_t rate) {
    return static_cast<uint32_t>((static_cast<uint64_t>(ticks) * rate * 100u) / xtal_hz);
}

// =============================================================================
// J - the link and the peer
// =============================================================================

void tJ_link() {
    const DsuDeviceId id = Dsu::device_id();
    print(plain, "  this die: DID ", hex(id.raw), ", silicon revision ", id.revision_letter(), crlf);
    const bool up = ensure_link();
    bench.verdict("the far board's usart_peer answers on the crossed pair", up);
    if (!up) {
        link_end();
        return;
    }
    link::Ident d{};
    const bool have = peer_ident(d);
    char label[9] = {};
    for (uint8_t i = 0; i < 8; ++i) {
        label[i] = d.label[i];
    }
    print(plain, "  peer: label '", label, "', kind ", hex(link::peer_kind(d.version)), ", fw ",
          hex(d.version), ", xtal ", d.xtal, ", sanity ", hex(d.sanity), crlf);
    bench.verdict("it identifies as usart_peer", have && d.sanity == link::ident_sanity);
    bench.verdict("the avrxt peer (kind 0x01), its time base its crystal",
                  have && link::peer_kind(d.version) == link::peer_avrxt && d.xtal == 1u);
    link_end();
}

// =============================================================================
// K - THE RATE, judged by a foreign receiver
// =============================================================================
//
// A LOOP PROVES THE BYTES, NEVER THE RATE. Here the far USART receives a
// GAPLESS burst - DATA refilled as DRE rises, so the frames sit back to
// back - and stamps every character as it completes on its own crystal
// (the Report's span, first to last: count - 1 frames of this
// transmitter). The span against the run's length at the rate ASKED is
// this SERCOM's rate error in ppm, and it is judged against the error the
// register itself predicts (table 30-2's equations at the reference, in
// 64 bits) PLUS THE TWO CRYSTALS' OFFSET, within what the instrument can
// tell. The offset is read first, on the 1 Mbaud fractional rung, whose
// register is exact on this generator (1.5: predicted 0 ppm) and whose
// 30000 frames make the finest grain of any rung, so its reading IS the
// far crystal against this one - judged only to be under 200 ppm, the
// crystals' parts unknown here (the far board's quartz may sit a hundred
// ppm from this one: the AVR64EA48 board's does); every other
// rung is judged against its prediction plus that offset, within the two
// readings' grain - each of a span's two stamps late by up to one sample
// of the far receiver (a sixteenth of a bit, an eighth under its CLK2X)
// and one idle turn of the far loop (64 ticks, its release listing) - and
// 10 ppm for the crystals' drift across the letter (the AVR's letter x
// reads the pair to 3 ppm over ten runs). The runs are long enough, 300
// ms or 30000 frames, that the grain stays near that budget. Every regime
// the generator offers: 16x and 8x on both generators, 3x arithmetic,
// from 2400 baud to 3 Mbaud, and one rung on
// generator 0 - OSC48M, the console's own regime - whose reading is that
// oscillator's offset, judged inside table 31-3's +-2 %. Each rung then
// echoes eight frames the far UART sends back: the receiver at that rate.

struct RateRung {
    UartSampling sampling;
    uint32_t rate;
    uint8_t generator;
};

/// "16x frac 115200", "OSC48M 16x arith 115200": a rung's label.
char rung_buf[32];
const char* rung_name(const RateRung& g) {
    char* p = rung_buf;
    auto put = [&p](const char* t) {
        while (*t != 0 && p < rung_buf + sizeof rung_buf - 1) *p++ = *t++;
    };
    if (g.generator == 0u) put("OSC48M ");
    put(uart_samples(g.sampling) == 16u ? "16x " : (uart_samples(g.sampling) == 8u ? "8x " : "3x "));
    put(uart_fractional(g.sampling) ? "frac " : "arith ");
    put(num(g.rate));
    *p = 0;
    return rung_buf;
}

/// SERCOM1's core moved to another generator, the instance stopped first:
/// a channel switched under a running instance would clock it through the
/// change.
void link_core(uint8_t generator) {
    (void)Lk::enable(false);
    (void)Lk::core_clock(generator);
}

/// The rate a register produces, against the one asked, in ppm (64 bits:
/// the register's step is far below a ppm at the arithmetic generator).
int32_t predicted_ppm(uint32_t ref, uint16_t reg, UartSampling s, uint32_t rate) {
    const uint64_t samples = uart_samples(s);
    uint64_t num = 0, den = 0;   // actual = num / den
    if (uart_fractional(s)) {
        const uint64_t eighths = 8ull * (reg & SERCOM_USART_INT_BAUD_FRAC_BAUD_Msk) +
                                 ((reg & SERCOM_USART_INT_BAUD_FRAC_FP_Msk) >>
                                  SERCOM_USART_INT_BAUD_FRAC_FP_Pos);
        num = 8ull * ref;
        den = samples * eighths;
    } else {
        num = static_cast<uint64_t>(ref) * (65536u - reg);
        den = 65536ull * samples;
    }
    const int64_t a = static_cast<int64_t>((num * 1'000'000ull) / den);
    const int64_t want = static_cast<int64_t>(rate) * 1'000'000;
    return static_cast<int32_t>(((a - want) * 1'000'000) / want);
}

void tK_rate() {
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        link_end();
        return;
    }
    const RateRung rungs[] = {
        {UartSampling::x16_fractional, 1'000'000, gen_xtal},   // first: the crystals' offset
        {UartSampling::x16_arithmetic, 2'400, gen_xtal},
        {UartSampling::x16_arithmetic, 9'600, gen_xtal},
        {UartSampling::x16_fractional, 3'750, gen_xtal},
        {UartSampling::x16_arithmetic, 115'200, gen_xtal},
        {UartSampling::x16_fractional, 115'200, gen_xtal},
        {UartSampling::x16_arithmetic, 921'600, gen_xtal},
        {UartSampling::x16_arithmetic, 1'000'000, gen_xtal},
        {UartSampling::x16_arithmetic, 1'500'000, gen_xtal},
        {UartSampling::x8_arithmetic, 2'000'000, gen_xtal},
        {UartSampling::x8_fractional, 2'000'000, gen_xtal},
        {UartSampling::x8_arithmetic, 3'000'000, gen_xtal},
        {UartSampling::x3_arithmetic, 1'000'000, gen_xtal},
        {UartSampling::x3_arithmetic, 2'000'000, gen_xtal},
        {UartSampling::x3_arithmetic, 3'000'000, gen_xtal},
        {UartSampling::x16_arithmetic, 115'200, 0},
    };
    print(plain, "  rung  BAUD  frames  far count/errors  span  ppm (predicted)  echo", crlf);
    int32_t offset = 0;           ///< the far crystal against this one, ppm
    int32_t offset_grain = 0;     ///< the reference reading's own grain, ppm
    bool offset_read = false;
    for (const RateRung& g : rungs) {
        const uint32_t ref = g.generator == 0u ? SysClock::hz : xtal_hz;
        const auto reg = sercom_uart_baud_reg(ref, g.rate, g.sampling);
        if (!reg) {
            bench.verdict(num(g.rate), ": the generator cannot express the rate", false);
            continue;
        }
        link::Cfg pc{};
        pc.rate = g.rate;
        pc.flags = g.rate > 1'500'000u ? link::flag_clk2x : 0u;
        const uint32_t n32 = (g.rate * 3u) / 100u;   // 300 ms of 10-bit frames
        const uint16_t n = static_cast<uint16_t>(n32 < 16u ? 16u : (n32 > 30000u ? 30000u : n32));

        // The gapless burst into the far sink.
        link::Params a{};
        a.cfg = pc;
        a.count = n;
        a.ms = bound_ms(g.rate, n);
        const bool sent = peer_act(link::Op::sink, a);
        const SercomUartConfig u{.pads = link_pads, .baud = *reg, .sampling = g.sampling};
        bool up = false;
        uint16_t sum = 0;
        if (sent) {
            if (g.generator == 0u) {
                link_core(0);
            }
            up = dut_up(u, g.rate);
            wait_for_peer_window();
            sum = burst(n, 0x30u, 0xFFu);
            if (g.generator == 0u) {
                link_core(gen_xtal);
            }
        }
        link::Report r{};
        const bool have = sent && peer_report(r, 10);
        const uint32_t nominal = link::nominal_span(g.rate, link::frame_half_bits(8, 0, 1), n);
        const int32_t ppm = have && r.span != 0u ? link::calibrate_ppm(nominal, r.span) : 0;
        const int32_t want = predicted_ppm(ref, *reg, g.sampling, g.rate);
        const uint32_t far_samples = g.rate > 1'500'000u ? 8u : 16u;
        const uint32_t grain = 2u * (xtal_hz / g.rate / far_samples + 64u);
        const int32_t grain_ppm =
            have && r.span != 0u ? static_cast<int32_t>((1'000'000ull * grain) / r.span) : 0;
        const bool reference = !offset_read && g.generator != 0u && g.rate == 1'000'000u &&
                               uart_fractional(g.sampling) && uart_samples(g.sampling) == 16u;
        const int32_t tol = reference ? 200 : grain_ppm + offset_grain + 10;

        // The echo: the far UART sends this end's frames back.
        link::Params e{};
        e.cfg = pc;
        e.count = 8;
        e.ms = bound_ms(g.rate, 16);
        uint16_t bad = 8;
        link::Report er{};
        bool ehave = false;
        if (peer_act(link::Op::echo, e)) {
            if (g.generator == 0u) {
                link_core(0);
            }
            if (dut_up(u, g.rate)) {
                bad = echo_mismatches(8, 8);
            }
            if (g.generator == 0u) {
                link_core(gen_xtal);
            }
            ehave = peer_report(er, 10);
        }

        print(plain, "  ", rung_name(g), " ", *reg, " ", n, "  ", have ? r.count : 0u, "/",
              have ? static_cast<uint32_t>(r.ferr + r.perr + r.ovf) : 0u, "  ", have ? r.span : 0u,
              "  ");
        print_ppm(ppm);
        print(plain, " (");
        print_ppm(want);
        if (!reference && g.generator != 0u) {
            print(plain, " ");
            print_ppm(offset);
        }
        print(plain, ", within ", static_cast<uint32_t>(tol), ")  echo ", 8u - bad, "/8, far ",
              ehave ? er.count : 0u, "/", ehave ? er.ferr : 0u, crlf);
        if (bad != 0u) {
            print(plain, "    first echo mismatch: sent ", hex(echo_first_sent), " got ",
                  hex(echo_first_got), "; all:");
            for (uint8_t i = 0; i < 8u; ++i) print(plain, " ", hex(echo_got[i]));
            print(plain, crlf);
        }
        const bool counted = up && have && r.count == n && r.sum == sum && r.ferr == 0u &&
                             r.perr == 0u && r.ovf == 0u;
        judge(rung_name(g), ": the far receiver takes every frame of the gapless burst",
              counted);
        if (g.generator == 0u) {
            judge(rung_name(g), ": OSC48M within table 31-3's recommended 2 %",
                  counted && abs32(ppm) <= 20'000);
        } else if (reference) {
            offset = ppm - want;
            offset_grain = grain_ppm;
            offset_read = counted && r.span != 0u;
            judge(rung_name(g), ": an exact register, so the reading is the far crystal against "
                                "this one - under 200 ppm",
                  offset_read && abs32(offset) <= tol);
        } else {
            judge(rung_name(g), ": the far crystal times the rate the register predicts, its "
                                "offset added",
                  counted && r.span != 0u && offset_read && abs32(ppm - want - offset) <= tol);
        }
        judge(rung_name(g), ": and this receiver takes the far echo",
              bad == 0u && ehave && er.count == 8u && er.ferr == 0u);

        // At the top rate, the same run through the Uart's transmit engine
        // (blocks of half its 512-byte ring), PRINTED: what a block
        // boundary costs the wire where one frame of slack is 3.3 us.
        if (g.rate >= 3'000'000u) {
            link::Params b = a;
            const bool sunk = peer_act(link::Op::sink, b);
            uint16_t esum = 0;
            bool eup = false;
            if (sunk) {
                wait_for_peer_window();
                const auto es = engine_burst(n, u, g.generator);
                eup = es.has_value();
                esum = es.value_or(0);
            }
            link::Report br{};
            const bool bhave = sunk && peer_report(br, 10);
            const int32_t eppm = bhave && br.span != 0u ? link::calibrate_ppm(nominal, br.span) : 0;
            print(plain, "    through the transmit engine: far count ", bhave ? br.count : 0u,
                  " sum ", (bhave && br.sum == esum && eup) ? "right" : "WRONG", ", ");
            print_ppm(eppm);
            print(plain, " ppm", crlf);
        }
    }
    link_end();
}

// =============================================================================
// L - the frame matrix against a different UART
// =============================================================================

struct FmtCase {
    uint8_t bits;
    uint8_t parity;
    uint8_t stop;
    const char* name;
};

uint8_t reverse8(uint8_t v) {
    uint8_t r = 0;
    for (uint8_t i = 0; i < 8u; ++i) {
        r = static_cast<uint8_t>((r << 1) | ((v >> i) & 1u));
    }
    return r;
}

void tL_frames() {
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        link_end();
        return;
    }
    const FmtCase cases[] = {
        {5, 0, 1, "5N1"}, {6, 2, 2, "6O2"}, {7, 1, 1, "7E1"}, {8, 2, 2, "8O2"},
        {8, 1, 2, "8E2"}, {8, 0, 2, "8N2"}, {link::bits9_low, 0, 1, "9N1"},
        {link::bits9_low, 1, 1, "9E1"},
    };
    for (const FmtCase& fc : cases) {
        link::Cfg c{};
        c.rate = 115'200;
        c.bits = fc.bits;
        c.parity = fc.parity;
        c.stop = fc.stop;
        link::Params a{};
        a.cfg = c;
        a.count = 6;
        a.ms = bound_ms(115'200, 20);
        if (!peer_act(link::Op::echo, a)) {
            bench.verdict(fc.name, ": the peer did not take the echo", false);
            continue;
        }
        const bool up = dut_link(c);
        const uint16_t bad = up ? echo_mismatches(fc.bits, 6) : 6u;
        link::Report r{};
        const bool have = peer_report(r, 10);
        print(plain, "  ", fc.name, ": wrong ", bad, ", peer count ", have ? r.count : 0u,
              " ferr ", have ? r.ferr : 0u, " perr ", have ? r.perr : 0u, crlf);
        judge(fc.name, " both ways across the wire",
              up && bad == 0u && have && r.count == 6u && r.ferr == 0u && r.perr == 0u);
    }

    // SBMODE is the transmitter's: a receiver never looks past its first
    // stop bit (31.6.2.6), so both asymmetric pairings are clean.
    for (uint8_t k = 0; k < 2u; ++k) {
        link::Cfg pc{};
        pc.rate = 115'200;
        pc.stop = k == 0u ? 2u : 1u;
        link::Cfg dc = pc;
        dc.stop = k == 0u ? 1u : 2u;
        link::Params a{};
        a.cfg = pc;
        a.count = 6;
        a.ms = bound_ms(115'200, 20);
        const bool sent = peer_act(link::Op::echo, a);
        const bool up = sent && dut_link(dc);
        const uint16_t bad = up ? echo_mismatches(8, 6) : 6u;
        link::Report r{};
        const bool have = peer_report(r, 10);
        judge(k == 0u ? "one stop bit here receives the far two"
                      : "two stop bits here are received by the far one",
              up && bad == 0u && have && r.count == 6u && r.ferr == 0u);
    }

    // CTRLA.DORD: MSB first against a far UART that is LSB first - each
    // byte arrives bit-reversed, both ways.
    {
        link::Cfg pc{};
        pc.rate = 115'200;
        link::Params a{};
        a.cfg = pc;
        a.count = 8;
        a.ms = bound_ms(115'200, 16);
        a.value = 0x01;
        a.pattern = link::pattern_counting;
        const bool sent = peer_act(link::Op::send, a);
        const bool up = sent && dut_up({.pads = link_pads,
                                        .format = {.lsb_first = false},
                                        .baud = command_baud.reg,
                                        .sampling = command_baud.sampling},
                                       115'200);
        uint8_t right = 0;
        uint8_t k = 0;
        const uint32_t t0 = Ticker::millis();
        while (Ticker::millis() - t0 < bound_ms(115'200, 16)) {
            const Got g = take();
            if (!g.any) continue;
            if (g.clean && static_cast<uint8_t>(g.data) == reverse8(static_cast<uint8_t>(0x01u + k)))
                ++right;
            ++k;
        }
        print(plain, "  MSB first, receiving: ", k, " frames, ", right, " bit-reversed", crlf);
        judge("DORD = 0 receives a far LSB-first byte bit-reversed", up && k == 8u && right == 8u);

        a.cfg = pc;
        a.count = 8;
        const bool sunk = peer_act(link::Op::sink, a);
        const bool up2 = sunk && dut_up({.pads = link_pads,
                                         .format = {.lsb_first = false},
                                         .baud = command_baud.reg,
                                         .sampling = command_baud.sampling},
                                        115'200);
        uint16_t want = 0;
        if (up2) {
            wait_for_peer_window();
            for (uint8_t i = 0; i < 8u; ++i) {
                const uint8_t v = static_cast<uint8_t>(0x01u + i);
                (void)put_frame(v);
                want = static_cast<uint16_t>(want + reverse8(v));
            }
            wait_tx_done();
        }
        link::Report r{};
        const bool have = peer_report(r, 10);
        print(plain, "  MSB first, sending: peer count ", have ? r.count : 0u, " sum ",
              hex(have ? r.sum : 0u), " want ", hex(want), crlf);
        judge("and sends MSB first: the far byte is the reversed one",
              up2 && have && r.count == 8u && r.sum == want && r.ferr == 0u);
    }
    link_end();
}

// =============================================================================
// M - errors with known causes; the receiver's buffer; start-of-frame
// =============================================================================

void tM_errors() {
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        link_end();
        return;
    }
    // 1. Odd parity into an even receiver: every frame PERR, its data readable.
    {
        link::Cfg pc{};
        pc.rate = 115'200;
        pc.parity = 2;
        link::Cfg dc = pc;
        dc.parity = 1;
        link::Params a{};
        a.cfg = pc;
        a.count = 6;
        a.ms = bound_ms(115'200, 12);
        a.value = 0x5A;
        const bool sent = peer_act(link::Op::send, a);
        const bool up = sent && dut_link(dc);
        const Drain d = collect(bound_ms(115'200, 12));
        print(plain, "  odd into even: frames ", d.frames, " perr ", d.perr, " ferr ", d.ferr,
              " last ", hex(d.last), crlf);
        judge("every frame of the wrong parity raises PERR", up && d.frames == 6u && d.perr == 6u);
        judge("and its data is still read (STATUS before DATA, 31.8.11)",
              d.frames == 6u && d.last == 0x5Fu);
    }
    // 2. A break: a plain receiver takes it as a character of zeros with FERR.
    {
        link::Cfg pc{};
        pc.rate = 115'200;
        link::Params a{};
        a.cfg = pc;
        a.count = 20;
        a.ms = 200;
        const bool sent = peer_act(link::Op::brk, a);
        const bool up = sent && dut_link(pc);
        const Drain d = collect(60);
        print(plain, "  a break of 20 bit times: frames ", d.frames, " ferr ", d.ferr, " data ",
              hex(d.first), crlf);
        judge("a break arrives as 0x00 with FERR",
              up && d.frames >= 1u && d.ferr >= 1u && d.first == 0x00u);
    }
    // 3. A gapless flood into a receiver that reads late, IBON off and on.
    for (uint8_t ibon = 0; ibon < 2u; ++ibon) {
        link::Cfg pc{};
        pc.rate = 115'200;
        link::Params a{};
        a.cfg = pc;
        a.count = 8;
        a.ms = bound_ms(115'200, 16);
        a.value = 0xB0;
        const bool sent = peer_act(link::Op::blast, a);
        const bool up = sent && dut_up({.pads = link_pads, .baud = command_baud.reg,
                                        .sampling = command_baud.sampling,
                                        .immediate_overflow = ibon != 0u},
                                       115'200);
        spin_ms(bound_ms(115'200, 16));
        // STATUS read before and after every DATA read, and cleared only
        // at the end: under IBON = 0 the overflow is raised "after the
        // received data is read" (31.6.2.6.2), which a clear behind each
        // read would wipe before it is seen.
        const uint16_t before = Lk::status();
        uint16_t vals[6] = {};
        uint16_t sts[6] = {};
        uint16_t after[6] = {};
        uint8_t k = 0;
        while (Lk::rxc_flag() && k < 6u) {
            sts[k] = Lk::status();
            vals[k] = Lk::data();
            after[k] = Lk::status();
            ++k;
        }
        Lk::clear_status(SercomStatus::receive_errors);
        print(plain, "  blast of 8 read late, IBON ", ibon, ": STATUS ", hex(before),
              "; each survivor as STATUS before / DATA / STATUS after:");
        uint8_t ovf_at = 0xFF;
        for (uint8_t i = 0; i < k; ++i) {
            print(plain, " ", hex(sts[i]), "/", hex(vals[i]), "/", hex(after[i]));
            if (ovf_at == 0xFFu && ((sts[i] | after[i]) & SercomStatus::overflow) != 0u) {
                ovf_at = i;
            }
        }
        print(plain, crlf);
        judge(ibon ? "IBON: BUFOVF stands before any read"
                   : "IBON 0: BUFOVF is raised as the data is read, not before",
              up && (ibon ? (before & SercomStatus::overflow) != 0u
                          : (before & SercomStatus::overflow) == 0u && ovf_at != 0xFFu));
        judge("the two buffer levels keep the two oldest frames, the shift register the newest",
              k == 3u && vals[0] == 0xB0u && vals[1] == 0xB1u && vals[2] == 0xB7u);
    }
    // 4. Start-of-frame detection (31.6.3.9): RXS at the start bit, RXC a
    //    frame later; and with SFDE off, no RXS at all.
    for (uint8_t sfde = 0; sfde < 2u; ++sfde) {
        link::Cfg pc{};
        pc.rate = 9'600;
        link::Params a{};
        a.cfg = pc;
        a.count = 1;
        a.ms = 100;
        a.value = 0x00;
        a.pattern = link::pattern_fixed;
        a.aux16 = 2;   // two more milliseconds before the frame
        const auto reg = sercom_closest_baud(xtal_hz, 9'600).value();
        const bool sent = peer_act(link::Op::send, a);
        const bool up = sent && dut_up({.pads = link_pads, .baud = reg.reg, .sampling = reg.sampling,
                                        .start_of_frame = sfde ? UartStartOfFrame::wake_on_rxs
                                                               : UartStartOfFrame::off},
                                       9'600);
        uint32_t t_rxs = 0, t_rxc = 0;
        bool rxs = false, rxc = false;
        const uint32_t t0 = cycles_now();
        while (cycles_now() - t0 < ms_cycles(40) && !rxc) {
            const uint8_t f = Lk::flags();
            if (!rxs && (f & SercomFlag::rxs) != 0u) {
                t_rxs = cycles_now();
                rxs = true;
            }
            if ((f & SercomFlag::rxc) != 0u) {
                t_rxc = cycles_now();
                rxc = true;
            }
        }
        (void)take();
        if (sfde) {
            const uint32_t lead = rxs && rxc ? t_rxc - t_rxs : 0u;
            const uint32_t cb = (lead * 100u) / (SysClock::hz / 9'600u);
            print(plain, "  SFDE: RXS ", rxs ? 1 : 0, ", RXC ", rxc ? 1 : 0, ", RXS ahead by ",
                  cb / 100u, ".", cb % 100u < 10u ? "0" : "", cb % 100u, " bit times", crlf);
            judge("SFDE raises RXS at the start bit, about 9.5 bit times ahead of RXC",
                  up && rxs && rxc && cb >= 850u && cb <= 1000u);
        } else {
            judge("without SFDE no RXS rises", up && rxc && !rxs);
        }
        Lk::clear_flags(SercomFlag::rxs);
    }
    link_end();
}

// ---- the pad log: edges timed by a loop's own turns ------------------------
//
// A polled stamp on this core reads the ticker's composed cycle count, some
// tens of cycles a read, with the SysTick entries inside it. THE PAD LOG
// counts the turns of one tight loop instead, masked so every turn costs
// the same, and stamps each change of the watched pins (PORT A's IN) or of
// the watched SERCOM1 flags (INTFLAG) with the turn it was seen in - a
// grain of a few cycles. Its unit is calibrated by a signal's own bit
// edges, so neither OSC48M's offset nor the loop's cost enters a figure:
// every result is in bit times of the signal, or of a reference frame of
// this board's own crystal-timed transmitter logged the same way. The
// ticker's ticks of a masked window are lost (its milliseconds lag after
// it, which only lengthens the waits that follow).

struct PadLog {
    static constexpr uint8_t most = 48;
    uint32_t turn[most] = {};
    uint32_t pins[most] = {};
    uint8_t flags[most] = {};
    uint8_t n = 0;
};

/// Wait (at most `wait_turns`) for the watched pins to leave their level,
/// then log every change for `turns` turns; entry 0 is that first change.
[[gnu::noinline]] void pad_log(PadLog& g, uint32_t pins, uint8_t flag_mask, uint32_t wait_turns,
                               uint32_t turns) {
    g.n = 0;
    SamPlatform::CriticalSection cs;
    const uint32_t v0 = LinkPort::in() & pins;
    uint32_t v = v0;
    for (uint32_t i = 0; i < wait_turns && v == v0; ++i) {
        v = LinkPort::in() & pins;
    }
    if (v == v0) {
        return;
    }
    uint8_t f = static_cast<uint8_t>(Lk::flags() & flag_mask);
    g.turn[0] = 0;
    g.pins[0] = v;
    g.flags[0] = f;
    // EVERY TURN COSTS THE SAME: it stores into the next slot whether or
    // not anything changed, and only a change moves the index - a turn
    // that recorded an edge in a branch of its own was longer than one
    // that did not (measured: a 26-bit break read 0.6 % long against the
    // sync field's bits, an edge every bit).
    uint32_t i = 1;
    for (uint32_t t = 1; t < turns; ++t) {
        const uint32_t nv = LinkPort::in() & pins;
        const uint8_t nf = static_cast<uint8_t>(Lk::flags() & flag_mask);
        g.turn[i] = t;
        g.pins[i] = nv;
        g.flags[i] = nf;
        const uint32_t diff = (nv ^ v) | static_cast<uint32_t>(nf ^ f);
        i += static_cast<uint32_t>(diff != 0u) & static_cast<uint32_t>(i < PadLog::most - 1u);
        v = nv;
        f = nf;
    }
    g.n = static_cast<uint8_t>(i);
}

/// The same with this end transmitting: `frames` written into DATA as DRE
/// rises from inside the loop, the first before the wait - every turn tests
/// for room, so the feeding costs every turn alike.
[[gnu::noinline]] void pad_log_feeding(PadLog& g, uint32_t pins, uint8_t flag_mask,
                                       const uint8_t* frames, uint8_t count, uint32_t turns) {
    g.n = 0;
    SamPlatform::CriticalSection cs;
    const uint32_t v0 = LinkPort::in() & pins;
    const uint8_t f0 = static_cast<uint8_t>(Lk::flags() & flag_mask);
    uint8_t sent = 0;
    uint32_t v = v0;
    uint8_t f = f0;
    uint32_t i = 0;
    for (uint32_t t = 0; t < turns; ++t) {
        // DRE read on EVERY turn, not behind the count's test: a turn that
        // reads it is some 9 % longer than one that does not (measured:
        // RS-485's guard read 1.09 bits a GTIME step when the turns after
        // the last write skipped the read).
        const uint32_t go = static_cast<uint32_t>(Lk::dre_flag()) &
                            static_cast<uint32_t>(sent < count);
        if (go != 0u) {
            Lk::data(frames[sent]);
            ++sent;
        }
        const uint32_t nv = LinkPort::in() & pins;
        const uint8_t nf = static_cast<uint8_t>(Lk::flags() & flag_mask);
        g.turn[i] = t;
        g.pins[i] = nv;
        g.flags[i] = nf;
        const uint32_t diff = (nv ^ v) | static_cast<uint32_t>(nf ^ f);
        i += static_cast<uint32_t>(diff != 0u) & static_cast<uint32_t>(i < PadLog::most - 1u);
        v = nv;
        f = nf;
    }
    g.n = static_cast<uint8_t>(i);
}

/// Turns, in thousandths of a bit of `bit_turns_q8` (turns a bit, x 256).
uint32_t millibits(uint32_t turns, uint32_t bit_turns_q8) {
    return bit_turns_q8 == 0u ? 0u
                              : static_cast<uint32_t>((256'000ull * turns) / bit_turns_q8);
}

void print_millibits(uint32_t mb) {
    print(plain, mb / 1000u, ".", mb % 1000u < 100u ? "0" : "", mb % 1000u < 10u ? "0" : "",
          mb % 1000u);
}

/// THE REFERENCE BIT: this board's own transmitter sends 0x55 at `rate` on
/// PA18 (its crossed-pair TxD, crystal-timed) - ten edges a bit apart - and
/// the log of it is turns a bit, x 256. 0 when the frame was not seen.
uint32_t reference_bit_q8(uint32_t rate) {
    const auto b = sercom_closest_baud(xtal_hz, rate).value();
    if (!dut_up({.pads = link_pads, .baud = b.reg, .sampling = b.sampling}, rate)) {
        return 0;
    }
    spin_ms(2);
    PadLog g{};
    Lk::data(0x55);
    pad_log(g, TxdPin::mask, 0, 20'000u, 20'000u);
    wait_tx_done();
    if (g.n < 10u) {
        return 0;
    }
    return ((g.turn[9] - g.turn[0]) * 256u) / 9u;
}

// =============================================================================
// N - the far UART's rate errors, and the bit-banger's cells and pulses,
// against the receiver's votes
// =============================================================================
//
// THE OPERATIONAL RANGE (31.6.2.6.4): frames of zeros from the far USART at
// 9600 baud off by a known amount - its generator steps a hundredth of a
// per cent there, so the rate is exact - into this receiver at 9600: clean
// throughout table 31-3's range for D = 8 (96.00 % to 105.26 %), broken
// past it on both sides (a slow frame's stop bit voted low, FERR; a fast
// frame of zeros has its last data bit voted in the stop bit, a wrong
// byte). THE VOTES (31.6.3.10): the far bit-banger shapes what no clean
// UART makes - one cell stretched, a lone low pulse - and the pad log
// measures what it really put on the wire, in bits of this receiver's own
// rate; the longest shape still read one way and the shortest read the
// other bracket the instant the receiver decides, which 31.6.3.10 puts at
// the majority of samples 7-8-9 of 16 under SAMPA 0 and 11-12-13 under
// SAMPA 2 - the section numbers the samples without saying where the count
// starts, and its note lets the count start up to one sample early, so a
// decision at the vote's middle lies between 6/16 and 8/16 of a bit under
// SAMPA 0 (10/16 and 12/16 under SAMPA 2), the same in a stop bit as in a
// start bit; and SAMPA 2 decides four samples, a quarter bit, later.

void tN_waveforms() {
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        link_end();
        return;
    }
    const auto r9600 = sercom_closest_baud(xtal_hz, 9'600).value();
    const SercomUartConfig base{.pads = link_pads, .baud = r9600.reg, .sampling = r9600.sampling};

    // 1. The operational range, from the far USART's own generator.
    {
        const int8_t errs[] = {0, 2, 3, 4, 5, 6, 7, 8, -2, -3, -4, -5, -6, -7, -8};
        int8_t fast = 0, slow = 0;
        bool inside = true;
        for (const int8_t e : errs) {
            link::Cfg pc{};
            pc.rate = static_cast<uint32_t>(9600 + (9600 * e) / 100);
            link::Params a{};
            a.cfg = pc;
            a.count = 8;
            a.ms = bound_ms(pc.rate, 30);
            a.value = 0x00;
            a.pattern = link::pattern_fixed;
            const bool sent = peer_act(link::Op::send, a);
            const bool up = sent && dut_up(base, 9'600);
            const Drain d = collect(bound_ms(pc.rate, 30));
            const bool clean = up && d.frames == 8u && d.ferr == 0u && d.sum == 0u;
            print(plain, "  ", e >= 0 ? "+" : "-", static_cast<uint32_t>(e >= 0 ? e : -e),
                  " %: frames ", d.frames, " ferr ", d.ferr, " data sum ", hex(d.sum), crlf);
            if (!clean) {
                if (e > 0 && fast == 0) fast = e;
                if (e < 0 && slow == 0) slow = e;
            }
            if (e >= -4 && e <= 5 && !clean) inside = false;
        }
        print(plain, "  frames of zeros break from ", static_cast<int32_t>(slow), " % slow and +",
              static_cast<int32_t>(fast), " % fast (table 31-3, D = 8: -4.00 % / +5.26 %)", crlf);
        bench.verdict("every rate error inside table 31-3's range reads clean", inside);
        bench.verdict("and a rate error past it breaks the frame on both sides",
                      fast != 0 && fast <= 8 && slow != 0 && slow >= -8);
    }

    // The reference bit, for the shapes the bit-banger makes.
    const uint32_t ref = reference_bit_q8(9'600);
    print(plain, "  the reference bit: ", ref / 256u, " turns of the pad log", crlf);
    bench.verdict("this board's own 9600 frame is logged on its pad", ref != 0u);
    if (ref == 0u) {
        link_end();
        return;
    }

    // 2. One cell stretched: the frame's low run - the start bit, eight
    //    zeros, cell 5 longer - measured on the pad; the stop bit is voted
    //    low (FERR) once the run outlasts the stop bit's decision.
    {
        const int8_t pcts[] = {20, 26, 30, 33, 36, 39, 42, 45, 50, 60};
        uint32_t clean_most = 0, broken_least = 0xFFFFFFFFu;
        for (const int8_t p : pcts) {
            link::Params a{};
            a.cfg.rate = 9'600;
            a.count = 1;
            a.ms = 200;
            a.value = 0x00;
            a.cell = 5;
            a.pct = p;
            if (!peer_act(link::Op::bitbang, a)) {
                continue;
            }
            (void)dut_up(base, 9'600);
            PadLog g{};
            pad_log(g, RxdPin::mask, 0, 2'000'000u, 40'000u);
            spin_ms(2);
            const Drain d = collect(5);
            const uint32_t run = g.n >= 2u ? millibits(g.turn[1] - g.turn[0], ref) : 0u;
            print(plain, "  cell 5 +", static_cast<uint32_t>(p), " %: a low run of ");
            print_millibits(run);
            print(plain, " bits, frames ", d.frames, " ferr ", d.ferr, crlf);
            if (run == 0u || d.frames != 1u) continue;
            if (d.ferr == 0u && run > clean_most) clean_most = run;
            if (d.ferr != 0u && run < broken_least) broken_least = run;
        }
        // The decision lies BETWEEN the longest clean run and the shortest
        // broken one - the ladder's step is the measurement's grain, and the
        // decision itself moves by a sample (1/16 bit, 62.5 thousandths)
        // with the start edge's phase against the receiver's clock - so
        // what is judged is that the bracket meets the vote's window and is
        // no wider than two samples: a midpoint near the window's edge would
        // fail on the grain alone. The same sample of phase lets two runs
        // closer than it come out in either order (9.438 bits read clean
        // and 9.398 broken, measured), and the pad log's turn - 1/75 of a
        // bit, 14 thousandths - blurs each run's length once more: the
        // bracket may cross by 63 + 14 = 77 thousandths.
        print(plain, "  read clean up to a low of ");
        print_millibits(clean_most);
        print(plain, " bits, FERR from ");
        print_millibits(broken_least);
        print(plain, ": the stop bit is decided between them", crlf);
        bench.verdict("the stop bit is decided at its vote, 9 6/16 to 9 8/16 bits from the start",
                      clean_most != 0u && broken_least != 0xFFFFFFFFu &&
                          clean_most <= broken_least + 77u && clean_most <= 9'500u &&
                          broken_least >= 9'375u &&
                          (broken_least < clean_most || broken_least - clean_most <= 125u));
    }

    // 3. A lone low pulse: a start bit only once it outlasts the start
    //    bit's decision. Widths in thirty-seconds of a 9600 bit.
    {
        uint32_t decided[2] = {0, 0};
        const uint8_t sampa[2] = {0, 2};
        for (uint8_t w = 0; w < 2u; ++w) {
            SercomUartConfig u = base;
            u.sample_adjust = sampa[w];
            uint32_t rejected_most = 0, taken_least = 0xFFFFFFFFu;
            print(plain, "  SAMPA ", sampa[w], ", width in thousandths of a bit: taken?");
            for (uint8_t k = 8; k <= 28u; ++k) {
                link::Params a{};
                a.cfg.rate = 9'600;
                a.count = 1;
                a.ms = 200;
                a.aux16 = static_cast<uint16_t>((2500u * k) / 32u);
                a.aux8 = 1;
                if (!peer_act(link::Op::glitch, a)) {
                    continue;
                }
                (void)dut_up(u, 9'600);
                PadLog g{};
                pad_log(g, RxdPin::mask, 0, 2'000'000u, 2'000u);
                spin_ms(3);
                const Drain d = collect(2);
                const uint32_t width = g.n >= 2u ? millibits(g.turn[1] - g.turn[0], ref) : 0u;
                const bool taken = d.frames != 0u;
                print(plain, " ", width, taken ? "+" : "-");
                if (width == 0u) continue;
                if (!taken && width > rejected_most) rejected_most = width;
                if (taken && width < taken_least) taken_least = width;
            }
            decided[w] = (rejected_most + taken_least) / 2u;
            print(plain, crlf, "    ignored up to ");
            print_millibits(rejected_most);
            print(plain, ", taken from ");
            print_millibits(taken_least);
            print(plain, ": the start bit decided at ");
            print_millibits(decided[w]);
            print(plain, " of a bit", crlf);
        }
        bench.verdict("SAMPA 0 decides a start bit at its vote, 6/16 to 8/16 of a bit",
                      decided[0] >= 375u && decided[0] <= 500u);
        // Each decided instant is one frame's, its count started up to a
        // sample early (31.6.3.10's note): the shift is four samples to
        // one either way.
        bench.verdict("SAMPA 2 decides four samples later, a quarter bit (to a sample)",
                      decided[1] >= decided[0] + 188u && decided[1] <= decided[0] + 312u);
    }
    link_end();
}

// =============================================================================
// O - auto-baud and the LIN header
// =============================================================================
//
// The receiver in auto-baud form (FORM 0x4, 16x fractional, 31.6.3.4)
// against the far board's autobaud_tx - a break of `aux16` bit times held
// on its crystal, the sync character and four payload frames at a rate
// this board was never told; and the transmitter in LIN host form (FORM
// 0x2, 31.6.3.5) generating the header into the far board's plain
// receiver, whose span times the break, the two delays and the sync.

struct AbRun {
    bool up = false;
    bool brk = false;
    bool isf = false;
    bool error = false;
    uint16_t reg = 0;
    Drain d{};
};

AbRun autobaud_listen(uint32_t fallback, uint32_t ms) {
    AbRun r{};
    const uint16_t reg = sercom_fractional_baud_reg(xtal_hz, fallback).value_or(0);
    r.up = reg != 0u && dut_up({.pads = link_pads, .baud = reg,
                                .sampling = UartSampling::x16_fractional,
                                .form = UartForm::auto_baud},
                               fallback);
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
        const uint8_t f = Lk::flags();
        if ((f & SercomFlag::rxbrk) != 0u) {
            r.brk = true;
            Lk::clear_flags(SercomFlag::rxbrk);
        }
        if ((f & SercomFlag::error) != 0u) {
            r.error = true;
            Lk::clear_flags(SercomFlag::error);
        }
        if ((Lk::status() & SercomStatus::sync_field_error) != 0u) {
            r.isf = true;
            Lk::clear_status(SercomStatus::sync_field_error);
        }
        const Got g = take();
        if (g.any) {
            count(r.d, g);
        }
    }
    r.reg = Lk::baud_reg();
    return r;
}

uint32_t eighths_of(uint16_t reg) {
    return 8u * (reg & SERCOM_USART_INT_BAUD_FRAC_BAUD_Msk) +
           ((reg & SERCOM_USART_INT_BAUD_FRAC_FP_Msk) >> SERCOM_USART_INT_BAUD_FRAC_FP_Pos);
}

void print_eighths(uint32_t e) { print(plain, e / 8u, ".", (e % 8u) * 125u); }

void tO_autobaud() {
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        link_end();
        return;
    }
    const DsuDeviceId id = Dsu::device_id();

    // 1. A foreign rate learned, from the rate itself and from half of it.
    const uint32_t rates[] = {9'600, 57'600, 115'200, 123'456};
    for (const uint32_t rate : rates) {
        for (uint8_t half = 0; half < 2u; ++half) {
            const uint32_t fallback = half ? rate / 2u : rate;
            link::Params a{};
            a.cfg.rate = rate;
            a.ms = bound_ms(rate, 40);
            a.value = 0x55;
            a.aux16 = 26;   // 26 far bits: 13 at half the rate, past the 11 a break needs
            a.aux8 = 4;
            const bool sent = peer_act(link::Op::autobaud_tx, a);
            const AbRun r = sent ? autobaud_listen(fallback, static_cast<uint32_t>(a.ms) + 20u)
                                 : AbRun{};
            const uint32_t want = (8u * xtal_hz) / (16u * rate);
            const uint32_t got = eighths_of(r.reg);
            const bool payload = r.d.frames >= 4u && r.d.last == 0xA3u;
            print(plain, "  ", rate, " from ", fallback, ": RXBRK ", r.brk ? 1 : 0, " ISF ",
                  r.isf ? 1 : 0, ", BAUD learned ");
            print_eighths(got);
            print(plain, " (the rate's ");
            print_eighths(want);
            print(plain, "), frames ", r.d.frames, " last ", hex(r.d.last), " ferr ", r.d.ferr,
                  crlf);
            judge(num(rate), half ? " baud learned from half its rate" : " baud learned",
                  r.up && r.brk && !r.isf && (got > want ? got - want : want - got) <= 1u);
            judge(num(rate), " baud: the payload read at the learned rate",
                  payload && r.d.ferr == 0u);
        }
    }

    // 2. A sync that is not 0x55: ISF and ERROR, BAUD left alone.
    {
        link::Params a{};
        a.cfg.rate = 9'600;
        a.ms = bound_ms(9'600, 40);
        a.value = 0x53;
        a.aux16 = 20;
        a.aux8 = 2;
        const bool sent = peer_act(link::Op::autobaud_tx, a);
        const AbRun r = sent ? autobaud_listen(9'600, static_cast<uint32_t>(a.ms) + 20u) : AbRun{};
        const uint16_t fallback = sercom_fractional_baud_reg(xtal_hz, 9'600).value();
        print(plain, "  sync 0x53: RXBRK ", r.brk ? 1 : 0, " ISF ", r.isf ? 1 : 0, " ERROR ",
              r.error ? 1 : 0, " BAUD ", hex(r.reg), " (the fallback ", hex(fallback), ")", crlf);
        judge("a sync character that is not 0x55 raises ISF and ERROR",
              r.up && r.brk && r.isf && r.error);
        judge("and leaves BAUD as it was", r.reg == fallback);
    }

    // 3. The break threshold: more than 11 nominal bit times (31.6.3.4).
    {
        bool brk[2] = {false, false};
        const uint16_t lows[2] = {10, 12};
        for (uint8_t k = 0; k < 2u; ++k) {
            link::Params a{};
            a.cfg.rate = 9'600;
            a.ms = bound_ms(9'600, 40);
            a.value = 0x55;
            a.aux16 = lows[k];
            a.aux8 = 2;
            const bool sent = peer_act(link::Op::autobaud_tx, a);
            const AbRun r =
                sent ? autobaud_listen(9'600, static_cast<uint32_t>(a.ms) + 20u) : AbRun{};
            brk[k] = r.brk;
            print(plain, "  a low of ", lows[k], " bit times: RXBRK ", r.brk ? 1 : 0, crlf);
        }
        judge("a low of 10 bit times is no break", !brk[0]);
        judge("a low of 12 is", brk[1]);
    }

    // 4. Erratum 1.17.2: a sync character with its stop bit missing - the
    //    bit-banger stretches 0x55's last data bit (a 0) over the stop
    //    bit's sample - is flagged as ISF or FERR from revision F on.
    {
        const SercomUartConfig u{.pads = link_pads,
                                 .baud = sercom_fractional_baud_reg(xtal_hz, 9'600).value(),
                                 .sampling = UartSampling::x16_fractional,
                                 .form = UartForm::auto_baud};
        bool flagged[2] = {false, false};
        bool brk[2] = {false, false};
        for (uint8_t k = 0; k < 2u; ++k) {
            link::Params a{};
            a.cfg.rate = 9'600;
            a.count = 1;
            a.ms = 300;
            a.value = 0x55;
            a.cell = k == 0u ? -1 : 8;
            a.pct = k == 0u ? 0 : 100;
            a.aux16 = 20;
            const bool sent = peer_act(link::Op::bitbang, a);
            if (sent) {
                (void)dut_up(u, 9'600);
            }
            bool isf = false, ferr = false;
            const uint32_t t0 = Ticker::millis();
            while (sent && Ticker::millis() - t0 < 60u) {
                if ((Lk::flags() & SercomFlag::rxbrk) != 0u) {
                    brk[k] = true;
                    Lk::clear_flags(SercomFlag::rxbrk);
                }
                const uint16_t st = Lk::status();
                if ((st & SercomStatus::sync_field_error) != 0u) isf = true;
                if ((st & SercomStatus::frame_error) != 0u) ferr = true;
                const Got g = take();
                if (g.any && (g.status & SercomStatus::frame_error) != 0u) ferr = true;
            }
            flagged[k] = isf || ferr;
            print(plain, "  sync ", k == 0u ? "whole" : "with its stop bit missing", ": RXBRK ",
                  brk[k] ? 1 : 0, " ISF ", isf ? 1 : 0, " FERR ", ferr ? 1 : 0, crlf);
        }
        const bool checks = sercom_autobaud_checks_stop(id.revision);
        print(plain, "  this die is revision ", id.revision_letter(),
              checks ? ": erratum 1.17.2 does not apply" : ": erratum 1.17.2 applies", crlf);
        judge("a whole sync after a break raises neither", brk[0] && !flagged[0]);
        judge(checks ? "a missing stop bit in the sync is flagged (erratum 1.17.2 not on this die)"
                     : "a missing stop bit goes unflagged (erratum 1.17.2)",
              brk[1] && flagged[1] == checks);
    }

    // 5. The LIN host's header at 19200, on its own pad and into the far
    //    plain receiver. On the pad (the pad log, PA18 read under the mux):
    //    the break's length, the delay to the sync and the delay from the
    //    sync's stop bit to the identifier, each in bits of the sync field
    //    itself - its 0x55 an edge a bit. At the far end: the break a
    //    character of zeros with FERR, then 0x55 and the identifier.
    {
        constexpr uint32_t rate = 19'200;
        const uint16_t reg = sercom_fractional_baud_reg(xtal_hz, rate).value();
        struct HdrCase { uint8_t brklen; uint8_t hdrdly; };
        const HdrCase cases[] = {{0, 0}, {1, 0}, {2, 0}, {3, 0}, {0, 1}, {0, 2}, {0, 3}};
        for (const HdrCase& h : cases) {
            link::Params a{};
            a.cfg.rate = rate;
            a.count = 3;
            a.ms = bound_ms(rate, 12);
            const bool sent = peer_act(link::Op::sink, a);
            const bool up = sent && dut_up({.pads = link_pads, .baud = reg,
                                            .sampling = UartSampling::x16_fractional,
                                            .form = UartForm::lin_host, .lin_break = h.brklen,
                                            .lin_header_delay = h.hdrdly},
                                           rate);
            bool txe = false;
            PadLog g{};
            if (up) {
                wait_for_peer_window();
                (void)Lk::lin_command(LinCommand::header_on_write);
                Lk::data(0x3C);
                pad_log(g, TxdPin::mask, 0, 20'000u, 40'000u);
                wait_tx_done();
                spin_ms(1);
                txe = (Lk::status() & SercomStatus::transmitter_empty) != 0u;
            }
            link::Report r{};
            const bool have = peer_report(r, 10);
            // Edges: 0 the break's fall, 1 its rise, 2..11 the sync (a bit
            // apart), 12 the identifier's start bit.
            const bool shaped = g.n >= 13u;
            const uint32_t bit_q8 = shaped ? ((g.turn[11] - g.turn[2]) * 256u) / 9u : 0u;
            const uint32_t brk = shaped ? millibits(g.turn[1] - g.turn[0], bit_q8) : 0u;
            const uint32_t d1 = shaped ? millibits(g.turn[2] - g.turn[1], bit_q8) : 0u;
            const uint32_t sync_to_id = shaped ? millibits(g.turn[12] - g.turn[2], bit_q8) : 0u;
            const uint32_t d2 = sync_to_id > 10'000u ? sync_to_id - 10'000u : 0u;
            const uint32_t want_brk = 1000u * lin_break_bits(h.brklen);
            const uint32_t want_d1 = 1000u * lin_break_to_sync_bits(h.hdrdly);
            const uint32_t want_d2 = 1000u * lin_sync_to_id_gap_bits(h.hdrdly);
            print(plain, "  header BRKLEN ", h.brklen, " HDRDLY ", h.hdrdly, ": break ");
            print_millibits(brk);
            print(plain, " (", want_brk / 1000u, "), to the sync ");
            print_millibits(d1);
            print(plain, " (", want_d1 / 1000u, "), sync to identifier ");
            print_millibits(d2);
            print(plain, " (", want_d2 / 1000u, ", the table's ", h.hdrdly == 0u ? 1u : 4u,
                  ") bits; far count ", have ? r.count : 0u,
                  " ferr ", have ? r.ferr : 0u, " sum ", hex(have ? r.sum : 0u), ", TXE ",
                  txe ? 1 : 0, crlf);
            const auto near = [](uint32_t a, uint32_t b) { return (a > b ? a - b : b - a) <= 100u; };
            judge("a LIN header reaches a plain receiver as break, 0x55, identifier",
                  up && have && r.count == 3u && r.ferr == 1u && r.sum == 0x55u + 0x3Cu);
            judge("its break is CTRLC.BRKLEN's (a tenth of a bit)", shaped && near(brk, want_brk));
            judge("its break-to-sync delay is CTRLC.HDRDLY's (31.8.3)",
                  shaped && near(d1, want_d1));
            judge("and the sync-to-identifier gap that delay less a bit (the silicon's, "
                  "not 31.8.3's table)", shaped && near(d2, want_d2));
            judge("STATUS.TXE stands once it is out", txe);
        }
        // LINCMD 0x1: the break on DATA's write, sync and identifier written after.
        {
            link::Params a{};
            a.cfg.rate = rate;
            a.count = 3;
            a.ms = bound_ms(rate, 12);
            const bool sent = peer_act(link::Op::sink, a);
            const bool up = sent && dut_up({.pads = link_pads, .baud = reg,
                                            .sampling = UartSampling::x16_fractional,
                                            .form = UartForm::lin_host},
                                           rate);
            if (up) {
                wait_for_peer_window();
                (void)Lk::lin_command(LinCommand::break_on_write);
                (void)put_frame(0x00);
                (void)put_frame(0x55);
                (void)put_frame(0x3C);
                wait_tx_done();
            }
            link::Report r{};
            const bool have = peer_report(r, 10);
            const uint32_t got = have ? centibits(r.span, rate) : 0u;
            print(plain, "  break by software (LINCMD 0x1): peer count ", have ? r.count : 0u,
                  " ferr ", have ? r.ferr : 0u, " sum ", hex(have ? r.sum : 0u), ", span ",
                  got / 100u, ".", got % 100u < 10u ? "0" : "", got % 100u, " bits", crlf);
            judge("LINCMD 0x1: a break on the write, then the caller's sync and identifier",
                  up && have && r.count == 3u && r.ferr == 1u && r.sum == 0x55u + 0x3Cu);
        }
    }
    link_end();
}

// =============================================================================
// Q - the synchronous roles on a real XCK
// =============================================================================

void tQ_sync() {
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        link_end();
        return;
    }
    const uint32_t xcks[] = {100'000, 1'000'000};
    // This board drives XCK; the far client follows and sinks. The far
    // INVEN on XCK swaps its edges as CPOL swaps these (DS40002247B
    // 27.3.3.1.1, DS60001479M 31.6.2.3.1).
    for (const uint32_t xck : xcks) {
        for (uint8_t inv = 0; inv < 2u; ++inv) {
            link::Cfg pc{};
            pc.mode = link::Mode::sync_client;
            pc.rate = xck;
            pc.flags = inv ? link::flag_invert_xck : 0u;
            link::Params a{};
            a.cfg = pc;
            a.count = 8;
            a.ms = bound_ms(xck, 40);
            if (!peer_act(link::Op::sink, a)) {
                bench.verdict(num(xck), " Hz host: the peer did not take the sink", false);
                continue;
            }
            link::Cfg dc = pc;
            dc.mode = link::Mode::sync_host;
            const bool up = dut_link(dc);
            uint16_t sum = 0;
            if (up) {
                wait_for_peer_window();
                sum = burst(8, 0x30u, 0xFFu);
            }
            link::Report r{};
            const bool have = peer_report(r, 10);
            print(plain, "  host ", xck, " Hz CPOL ", inv, ": peer count ", have ? r.count : 0u,
                  " sum ", hex(have ? r.sum : 0u), " want ", hex(sum), " ferr ",
                  have ? r.ferr : 0u, crlf);
            judge(num(xck), inv ? " Hz host, CPOL 1, its frames clocked into the far client"
                                : " Hz host, CPOL 0, its frames clocked into the far client",
                  up && have && r.count == 8u && r.sum == sum && r.ferr == 0u);
        }
    }
    // The far board drives XCK; this end is the client.
    const uint32_t client_xcks[] = {100'000, 1'000'000, 4'000'000, 12'000'000};
    for (const uint32_t xck : client_xcks) {
        link::Cfg pc{};
        pc.mode = link::Mode::sync_host;
        pc.rate = xck;
        link::Params a{};
        a.cfg = pc;
        a.count = 8;
        a.ms = bound_ms(xck, 60);
        a.value = 0x60;
        if (!peer_act(link::Op::send, a)) {
            bench.verdict(num(xck), " Hz client: the peer did not take the send", false);
            continue;
        }
        link::Cfg dc = pc;
        dc.mode = link::Mode::sync_client;
        const bool up = dut_link(dc);
        const Drain d = collect(static_cast<uint32_t>(a.ms) + 40u);
        uint16_t want = 0;
        for (uint8_t i = 0; i < 8u; ++i) want = static_cast<uint16_t>(want + 0x60u + i);
        print(plain, "  client at ", xck, " Hz: frames ", d.frames, " sum ", hex(d.sum), " want ",
              hex(want), " ferr ", d.ferr, crlf);
        if (xck <= 4'000'000u) {
            judge(num(xck), " Hz client: every frame on the far clock",
                  up && d.frames == 8u && d.sum == want && d.ferr == 0u);
        }
    }
    link_end();
}

// =============================================================================
// S - IrDA both ways, and RXPL
// =============================================================================
//
// THE SENSE: this coder transmits a 0 as a HIGH pulse on a line idling
// LOW and takes a LOW pulse as a 0 (31.6.3.3, measured by letter g); the
// far board's IRCOM pins carry the inverted values - a 0 a LOW pulse on an
// idle-high line - so its frames reach this receiver as they are, and this
// coder's reach it only through its receive pin's INVEN, which the far
// peer sets itself when it finds its line idling low (avrxt usart_peer,
// match_ircom_sense()). RXPL (31.8.5) counts the core clock: a pulse is a
// 0 when it lasts at least RXPL + 2 cycles of the 24 MHz generator.

void tS_irda() {
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        link_end();
        return;
    }
    // 1. Both ways, the far coder echoing this one.
    {
        link::Cfg pc{};
        pc.mode = link::Mode::ircom;
        pc.rate = 115'200;
        link::Params a{};
        a.cfg = pc;
        a.count = 6;
        a.ms = bound_ms(115'200, 24);
        const bool sent = peer_act(link::Op::echo, a);
        const bool up = sent && dut_link(pc);
        const uint16_t bad = up ? echo_mismatches(8, 6) : 6u;
        link::Report r{};
        const bool have = peer_report(r, 10);
        print(plain, "  IrDA 115200 both ways: wrong ", bad, ", peer count ", have ? r.count : 0u,
              " ferr ", have ? r.ferr : 0u, crlf);
        judge("this coder's frames reach the far receiver (its pin inverted)",
              have && r.count == 6u && r.ferr == 0u);
        judge("and the far coder's echo reaches this one as it is", up && bad == 0u);
    }
    // 2. RXPL against the far coder's 3/16 pulses (39 cycles of 24 MHz at
    //    115200) and against its fixed pulse of 60 cycles (TXPLCTRL 60).
    struct PlCase { uint8_t txpl; uint8_t first; uint8_t last; uint8_t pulse; };
    const PlCase pls[] = {{0x00, 28, 46, 39}, {0x3C, 50, 66, 60}};
    for (const PlCase& pl : pls) {
        uint8_t first_block = 0;
        uint8_t last_pass = 0;
        print(plain, "  RXPL against a ", pl.pulse, "-cycle pulse:");
        for (uint8_t rx = pl.first; rx <= pl.last; rx = static_cast<uint8_t>(rx + 2u)) {
            link::Cfg pc{};
            pc.mode = link::Mode::ircom;
            pc.rate = 115'200;
            pc.txpl = pl.txpl;
            link::Params a{};
            a.cfg = pc;
            a.count = 6;
            a.ms = bound_ms(115'200, 24);
            a.value = 0x41;
            if (!peer_act(link::Op::send, a)) {
                continue;
            }
            link::Cfg dc{};
            dc.mode = link::Mode::ircom;
            dc.rate = 115'200;
            dc.rxpl = rx;
            const bool up = dut_link(dc);
            const Drain d = collect(bound_ms(115'200, 24));
            const bool clean = up && d.frames == 6u && d.ferr == 0u &&
                               d.sum == static_cast<uint16_t>(6u * 0x41u + 15u);
            print(plain, " ", rx, ":", clean ? "ok" : "no");
            if (clean) last_pass = rx;
            if (!clean && first_block == 0u) first_block = rx;
        }
        print(plain, crlf, "    taken up to RXPL ", last_pass, ", refused from ", first_block, crlf);
        // The edge: the last RXPL taken, plus two, against the pulse. A
        // pulse of L generator cycles is counted floor(L) or one fewer with
        // its phase, and the ladder's even rungs take the largest even RXPL
        // at most that count less two - so a pulse of its nominal length
        // lands the edge between -2 and 0. The far coder's pulse is
        // quantized to its own clock: 3/16 of a 115200 bit is 32.55 cycles
        // of a 20 MHz AVR EA, so 32 or 33, 38.4 or 39.6 of this generator -
        // one count lower still (-3, measured). The edge lies within two
        // of -1.
        const int32_t edge = static_cast<int32_t>(last_pass) + 2 - static_cast<int32_t>(pl.pulse);
        judge("a pulse is a 0 while it lasts RXPL + 2 core cycles (31.8.5, to two cycles)",
              last_pass != 0u && first_block != 0u && edge >= -3 && edge <= 1);
    }
    // 3. The coding means something: a plain far receiver cannot read this
    //    coder, and this receiver cannot read a plain far transmitter.
    {
        link::Cfg pc{};
        pc.rate = 115'200;
        link::Params a{};
        a.cfg = pc;
        a.count = 6;
        a.ms = bound_ms(115'200, 24);
        const bool sent = peer_act(link::Op::sink, a);
        link::Cfg dc{};
        dc.mode = link::Mode::ircom;
        dc.rate = 115'200;
        const bool up = sent && dut_link(dc);
        uint16_t sum = 0;
        if (up) {
            wait_for_peer_window();
            sum = burst(6, 0x41u, 0xFFu);
        }
        link::Report r{};
        const bool have = peer_report(r, 10);
        print(plain, "  this coder into a plain far receiver: count ", have ? r.count : 0u,
              " sum ", hex(have ? r.sum : 0u), " want ", hex(sum), crlf);
        judge("a plain receiver cannot read the pulse-coded frames",
              up && have && !(r.count == 6u && r.sum == sum && r.ferr == 0u));

        a.value = 0x41;
        const bool sent2 = peer_act(link::Op::send, a);
        const bool up2 = sent2 && dut_link(dc);
        const Drain d = collect(bound_ms(115'200, 24));
        print(plain, "  a plain far transmitter into this decoder: frames ", d.frames, " ferr ",
              d.ferr, crlf);
        judge("and the decoder cannot read plain frames",
              up2 && !(d.frames == 6u && d.ferr == 0u &&
                       d.sum == static_cast<uint16_t>(6u * 0x41u + 15u)));
    }
    link_end();
}

// =============================================================================
// C - hardware handshaking: CTS gating the transmitter, RTS on its own pad
// =============================================================================
//
// TXPO 0x2 (31.6.3.2) puts RTS on PAD[2] - PA18 - and CTS on PAD[3] -
// PA19, the desk's XCK wire - with TxD on PAD[0]. THE WIRES REACH THE
// PADS, NOT A FLOW-CONTROLLED LINK: TxD on PAD[0] would drive PA16, the far
// board's TxD, and the far USART has no handshaking (an AVR DA/DB USART
// has XDIR, not RTS/CTS). So TxD's pad is left to PORT - the transmitter
// shifts into nothing, and TXC says whether it did -, and the far board
// is asked for CTS LEVELS on the XCK wire with usart_link.hpp's hold_pad:
// its XCK pad let go by its USART and driven low, then high - a level its
// synchronous host could not present, since that XCK RUNS while the
// instance is enabled. A level counts as PRESENTED only if it holds
// against this end's pull the other way - PORT's pull-up against a low,
// pull-down against a high - and the far pad read the same level back at
// the end of the hold, because a far pin that does not drive leaves the
// wire floating, and a floating CTS reads whatever it likes (measured: a
// "high" that let a frame go). A level not presented, or a peer that
// refuses the op, skips its leg with that reason. CTS low is judged a
// second time on this end's own pull-down. RTS is read on its own pad:
// low while the receiver is enabled with room, high with it disabled.

void tC_flow() {
    if (!ensure_link()) {
        bench.verdict("the peer is reachable", false);
        link_end();
        return;
    }
    const SercomUartConfig fc{.pads = {.tx = SercomPad::pad0, .rx = SercomPad::pad1,
                                       .tx_pin = {'A', 16, PinFunction::c},
                                       .rx_pin = {'A', 17, PinFunction::c}},
                              .baud = command_baud.reg, .sampling = command_baud.sampling,
                              .pinout = UartPinout::flow_control};
    /// The handshaking configuration with TxD's and RxD's pads left dark,
    /// RTS and CTS muxed, CTS with `cts_pull`.
    const auto flow_up = [&fc](PinPull cts_pull) {
        bool ok = write(fc);
        LinkPort::dir_clear(link_pins);
        pad_dark<RxdPin>();
        pad_dark<Pad1Pin>();
        TxdPin::function(PinFunction::c, {.input_enable = true});
        XckPin::function(PinFunction::c, {.input_enable = true, .pull = cts_pull});
        ok = start(fc) && ok;
        Lk::clear_flags(SercomFlag::all);
        return ok;
    };
    /// A frame written: did it leave within a millisecond?
    const auto frame_leaves = []() {
        Lk::clear_flags(SercomFlag::txc);
        Lk::data(0x5A);
        spin_ms(1);
        return Lk::txc_flag();
    };

    // 1. The far levels, each commanded with hold_pad on the far XCK pad
    //    and probed against this end's opposite pull before it is judged.
    bool presented[2] = {false, false};
    for (uint8_t high = 0; high < 2u; ++high) {
        link::Params a{};
        a.value = link::pad_xck;
        a.aux8 = high;
        a.ms = 120;
        const bool sent = peer_act(link::Op::hold_pad, a);
        // Two hundred readings over two milliseconds: a far pin that does
        // not drive, or drives a running clock, reads both ways.
        pad_dark<XckPin>();
        XckPin::input(high ? PinPull::down : PinPull::up);
        spin_us(200);
        uint16_t highs = 0;
        for (uint16_t k = 0; k < 200u; ++k) {
            if (XckPin::read()) ++highs;
            spin_us(10);
        }
        pad_dark<XckPin>();
        const bool probed = sent && (high ? highs == 200u : highs == 0u);
        bool cts = false, went = false;
        if (probed) {
            const bool up = flow_up(PinPull::none);
            spin_ms(1);
            cts = (Lk::status() & SercomStatus::cts) != 0u;
            went = up && frame_leaves();
        }
        link::Report r{};
        const bool rep = peer_report(r, 150);
        // The far readback closes the loop: the level it drove is the
        // level its own pad read at the end of the hold.
        presented[high] = probed && rep && r.count == 1u && r.aux16 == high;
        print(plain, "  far XCK held ", high ? "HIGH" : "LOW", ": against a pull-",
              high ? "down" : "up", " the wire reads high ", highs, " times in 200, the far "
              "pad read back ", r.aux16);
        if (presented[high]) {
            print(plain, ", STATUS.CTS ", cts ? 1 : 0, ", a frame ", went ? "left" : "HELD", crlf);
            if (high) {
                judge("CTS held high by the far board holds the transmitter (31.6.3.2)",
                      cts && !went);
            } else {
                judge("CTS held low by the far board lets a frame go", !cts && went);
            }
        } else {
            print(plain, crlf);
            if (!leg_refused) {
                print(plain, "  SKIPPED, no verdict claimed: the far XCK does not hold the "
                             "wire ", high ? "high" : "low", " against this end's pull", crlf);
            }
        }
    }

    // 2. CTS held low by this end's own pull-down (31.5.1), the far pin let
    //    go: a frame leaves, and CTSIC stays down - CTS never changed.
    {
        const bool up = flow_up(PinPull::down);
        spin_ms(1);
        const bool cts = (Lk::status() & SercomStatus::cts) != 0u;
        const bool went = up && frame_leaves();
        const bool ctsic = (Lk::flags() & SercomFlag::ctsic) != 0u;
        print(plain, "  CTS on this end's pull-down: STATUS.CTS ", cts ? 1 : 0, ", a frame ",
              went ? "left" : "HELD", ", CTSIC ", ctsic ? 1 : 0, crlf);
        bench.verdict("CTS low lets a frame go", up && !cts && went);
    }
    if (!presented[1]) {
        print(plain, "  (CTS high: not judged - the far board presented no level)", crlf);
    }

    // 3. RTS on its own pad: the receiver enabled with room, then disabled.
    {
        const bool up = flow_up(PinPull::down);
        spin_us(100);
        const bool rts_on = TxdPin::read();
        (void)Lk::enable_rx(false);
        spin_us(100);
        const bool rts_off = TxdPin::read();
        print(plain, "  RTS: receiver enabled ", rts_on ? "high" : "low", ", disabled ",
              rts_off ? "high" : "low", crlf);
        bench.verdict("RTS is low while the receiver has room, high once it is disabled",
                      up && !rts_on && rts_off);
    }
    link_end();
}

// =============================================================================
// R - RS-485: TE on its own pad, on the shared wire
// =============================================================================
//
// TXPO 0x3 puts TxD on PAD[0] - PA16, the shared wire, the far board
// listening through its loop-back - and TE on PAD[2], PA18, read back on
// its own pad (PORT IN under the mux): high from the first start bit to
// the last stop bit's end and CTRLC.GTIME bit times more, TXC raised as it
// falls (31.6.3.6). The far receiver confirms the frames.

/// The turn of the first logged change of `bit` to `high` (or of a flag),
/// or nullopt.
std::optional<uint32_t> first_change(const PadLog& g, uint32_t bit, bool high, uint8_t from = 0) {
    for (uint8_t i = from; i < g.n; ++i) {
        const bool now = (g.pins[i] & bit) != 0u;
        const bool before = i == 0u ? !now : (g.pins[i - 1u] & bit) != 0u;
        if (now != before && now == high) return g.turn[i];
    }
    return {};
}
std::optional<uint32_t> last_change(const PadLog& g, uint32_t bit, bool high) {
    for (uint8_t i = g.n; i-- > 1u;) {
        const bool now = (g.pins[i] & bit) != 0u;
        const bool before = (g.pins[i - 1u] & bit) != 0u;
        if (now != before && now == high) return g.turn[i];
    }
    return {};
}
std::optional<uint32_t> flag_rise(const PadLog& g, uint8_t flag) {
    for (uint8_t i = 1; i < g.n; ++i) {
        if ((g.flags[i] & flag) != 0u && (g.flags[i - 1u] & flag) == 0u) return g.turn[i];
    }
    return {};
}

/// A signed difference of two turns, in thousandths of a bit.
int32_t signed_millibits(uint32_t a, uint32_t b, uint32_t bit_q8) {
    return a >= b ? static_cast<int32_t>(millibits(a - b, bit_q8))
                  : -static_cast<int32_t>(millibits(b - a, bit_q8));
}

void print_signed_millibits(int32_t mb) {
    if (mb < 0) {
        print(plain, "-");
        mb = -mb;
    }
    print_millibits(static_cast<uint32_t>(mb));
}

void tR_rs485() {
    if (!enter_shared()) {
        bench.verdict("the peer is reachable on the shared wire", false);
        leave_shared();
        link_end();
        return;
    }
    // ONE frame of 0x55 a case, logged with the plain pad log (every turn
    // alike): its own edges a bit apart are the log's bit, TE's two edges
    // and TXC's rise are timed against them. The pad log's turn is some 35
    // cycles: a 70th of a bit at 19200.
    struct TeCase { uint32_t rate; UartSampling sampling; uint8_t gtime; };
    const TeCase cases[] = {
        {19'200, UartSampling::x16_fractional, 0}, {19'200, UartSampling::x16_fractional, 1},
        {19'200, UartSampling::x16_fractional, 3}, {19'200, UartSampling::x16_fractional, 7},
        {19'200, UartSampling::x16_arithmetic, 7}, {9'600, UartSampling::x16_fractional, 7},
        {57'600, UartSampling::x16_arithmetic, 7},
    };
    for (const TeCase& tc : cases) {
        const uint32_t rate = tc.rate;
        link::Params a{};
        a.cfg.rate = rate;
        a.count = 1;
        a.ms = bound_ms(rate, 16);
        const bool sent = peer_act(link::Op::sink, a);
        bool up = false, low_before = false;
        PadLog g{};
        if (sent) {
            // Listening first, as every configuration on this wire is.
            up = dut_up({.pads = link_pads, .baud = command_baud.reg,
                         .sampling = command_baud.sampling}, rate);
            wait_for_peer_window();
            const SercomUartConfig te{
                .pads = wire_pads,
                .baud = sercom_uart_baud_reg(xtal_hz, rate, tc.sampling).value_or(0),
                .sampling = tc.sampling, .pinout = UartPinout::rs485, .guard_time = tc.gtime};
            up = write(te) && up;
            TxdPin::function(PinFunction::c, {.input_enable = true});
            up = start(te) && up;
            spin_us(50);
            low_before = !TxdPin::read();
            Lk::clear_flags(SercomFlag::txc);
            // The frame written by the log's own first turn, once it has read
            // the pads' idle levels: written before, its start bit can beat
            // the log's first reading and the log then starts a bit late
            // (measured: TE "a bit ahead" of a start bit the log never saw).
            const uint8_t one[1] = {0x55};
            pad_log_feeding(g, RxdPin::mask | TxdPin::mask, SercomFlag::txc, one, 1, 12'000u);
            wait_tx_done();
            pad_dark<TxdPin>();
            (void)apply(listen_cfg(cur));
            Lk::flush_rx();
        }
        link::Report r{};
        const bool have = peer_report(r, 10);
        // TxD: the start bit's fall, and 0x55's last edge, the stop bit's
        // rise, nine bits on.
        const auto fall0 = first_change(g, RxdPin::mask, false);
        const auto stop = last_change(g, RxdPin::mask, true);
        const auto te_up = first_change(g, TxdPin::mask, true);
        const auto te_down = te_up ? first_change(g, TxdPin::mask, false) : std::nullopt;
        const auto txc = flag_rise(g, SercomFlag::txc);
        const bool shaped = fall0 && stop && te_up && te_down && txc && *stop > *fall0;
        const uint32_t bit_q8 = shaped ? ((*stop - *fall0) * 256u) / 9u : 0u;
        const int32_t lead = shaped ? signed_millibits(*fall0, *te_up, bit_q8) : 0;
        const uint32_t end = shaped ? *fall0 + (10u * bit_q8) / 256u : 0u;   // the stop bit's end
        const int32_t guard = shaped ? signed_millibits(*te_down, end, bit_q8) : 0;
        const int32_t txc_after = shaped ? signed_millibits(*txc, *te_down, bit_q8) : 0;
        print(plain, "  ", rate, uart_fractional(tc.sampling) ? " frac" : " arith", " GTIME ",
              tc.gtime, ": TE up ");
        print_signed_millibits(lead);
        print(plain, " bits ahead of the start bit, down ");
        print_signed_millibits(guard);
        print(plain, " bits after the stop bit; TXC ");
        print_signed_millibits(txc_after);
        print(plain, " bits after TE's fall; far count ", have ? r.count : 0u, " data ",
              hex(have ? r.sum : 0u), crlf);
        // A twentieth of a bit, and three turns of the log: each edge is
        // seen to a turn, and the stop bit's end is reckoned from two.
        const int32_t grain = 50 + (bit_q8 != 0u ? static_cast<int32_t>((3u * 256'000u) / bit_q8) : 0);
        const auto within = [grain](int32_t v, int32_t want) {
            return v >= want - grain && v <= want + grain;
        };
        judge("TE idles low on its pad", up && low_before);
        judge(num(tc.gtime), ": TE rises with the start bit",
              shaped && within(lead, 0));
        judge(num(tc.gtime), ": and falls GTIME bit times after the stop bit",
              shaped && within(guard, 1000 * static_cast<int32_t>(tc.gtime)));
        judge(num(tc.gtime), ": TXC rises as TE falls (31.6.3.6)", shaped && within(txc_after, 0));
        judge(num(tc.gtime), ": the far receiver takes the frame",
              have && r.count == 1u && r.sum == 0x55u && r.ferr == 0u);
    }

    // Gapless frames: TE does not fall between them - one rise and one
    // fall for four frames, GTIME 2 (the feeding log: edges counted, not
    // timed).
    {
        link::Params a{};
        a.cfg.rate = 19'200;
        a.count = 4;
        a.ms = bound_ms(19'200, 16);
        const bool sent = peer_act(link::Op::sink, a);
        uint8_t rises = 0, falls = 0;
        bool up = false;
        if (sent) {
            up = dut_up({.pads = link_pads, .baud = command_baud.reg,
                         .sampling = command_baud.sampling}, 19'200);
            wait_for_peer_window();
            const auto b = sercom_closest_baud(xtal_hz, 19'200).value();
            const SercomUartConfig te{.pads = wire_pads, .baud = b.reg, .sampling = b.sampling,
                                      .pinout = UartPinout::rs485, .guard_time = 2};
            up = write(te) && up;
            TxdPin::function(PinFunction::c, {.input_enable = true});
            up = start(te) && up;
            spin_us(50);
            const uint8_t data[4] = {0x70, 0x71, 0x72, 0x73};
            PadLog g{};
            pad_log_feeding(g, TxdPin::mask, 0, data, 4, 12'000u);
            wait_tx_done();
            for (uint8_t i = 0; i < g.n; ++i) {
                const bool now = (g.pins[i] & TxdPin::mask) != 0u;
                const bool before = i == 0u ? false : (g.pins[i - 1u] & TxdPin::mask) != 0u;
                if (now && !before) ++rises;
                if (!now && before) ++falls;
            }
            pad_dark<TxdPin>();
            (void)apply(listen_cfg(cur));
            Lk::flush_rx();
        }
        link::Report r{};
        const bool have = peer_report(r, 10);
        print(plain, "  four gapless frames: TE rose ", rises, " and fell ", falls,
              " times; far count ", have ? r.count : 0u, " sum ", hex(have ? r.sum : 0u), crlf);
        judge("TE stays up across gapless frames: one rise, one fall",
              up && rises == 1u && falls == 1u);
        judge("and the far receiver takes all four",
              have && r.count == 4u && r.sum == 0x70u + 0x71u + 0x72u + 0x73u && r.ferr == 0u);
    }
    leave_shared();
    link_end();
}

// =============================================================================
// W - the one-wire bus, and collision detection, on the shared wire
// =============================================================================
//
// Half duplex on PA16 against the far onewire op, which answers every
// frame with its complement, open drain, after the command channel's
// turnaround; then COLDEN (31.6.3.7) against the far collide op, which
// pulls the wire low into this board's frames on purpose: the transfer
// aborted, the buffer flushed, TXEN cleared, COLL with ERROR and TXC.
// Erratum 1.17.12 wants the APB clock above the core one - 48 MHz over the
// crystal's 24 here, stated to configure() (SercomClocks). The collision is
// a driver fight by its nature - this transmitter is push-pull, the far one
// pulls low - so it is run at 250 kbaud, where it lasts until the next
// bit's vote, four microseconds at most.

void tW_onewire() {
    if (!enter_shared()) {
        bench.verdict("the peer is reachable on the shared wire", false);
        leave_shared();
        link_end();
        return;
    }
    // 1. The half-duplex exchange, COLDEN armed throughout.
    {
        constexpr uint32_t rate = 19'200;
        link::Params a{};
        a.cfg.rate = rate;
        a.count = 4;
        a.ms = 600;
        const bool sent = peer_act(link::Op::onewire, a);
        const auto reg = sercom_closest_baud(xtal_hz, rate).value();
        const bool up = sent && dut_up({.pads = link_pads, .baud = reg.reg,
                                        .sampling = reg.sampling, .collision_detect = true},
                                       rate);
        bool echoed = true, answered = true, coll = false;
        uint16_t first_back = 0xFFFF;
        wait_for_peer_window();
        for (uint8_t i = 0; i < 4u && up; ++i) {
            const uint8_t v = static_cast<uint8_t>(0xA5u + i);
            line_talk();
            Lk::flush_rx();
            (void)put_frame(v);
            wait_tx_done();
            const Got own = take();
            echoed = echoed && own.any && own.clean && static_cast<uint8_t>(own.data) == v;
            coll = coll || (Lk::status() & SercomStatus::collision) != 0u;
            (void)apply(listen_cfg(cur));
            Got back{};
            const uint32_t t0 = cycles_now();
            while (cycles_now() - t0 < ms_cycles(4)) {
                back = take();
                if (back.any) break;
            }
            const bool ok = back.any && back.clean &&
                            static_cast<uint8_t>(back.data) == static_cast<uint8_t>(v ^ 0xFFu);
            if (!ok && answered) first_back = back.any ? back.data : 0xFFFFu;
            answered = answered && ok;
        }
        link::Report r{};
        const bool have = peer_report(r, 20);
        print(plain, "  one-wire 19200: own echo ", echoed ? "clean" : "WRONG", ", answers ",
              answered ? "right" : "WRONG", " (first wrong ", hex(first_back), "), COLL ",
              coll ? 1 : 0, ", peer count ", have ? r.count : 0u, crlf);
        judge("every frame is heard back through this end's own pad", up && echoed);
        judge("the far end answers each half duplex on the same wire", up && answered);
        judge("COLDEN raises nothing on a clean exchange", up && !coll);
        judge("the far end took the four frames", have && r.count == 4u && r.ferr == 0u);
    }
    // 2. The collision.
    {
        constexpr uint32_t rate = 250'000;
        link::Params c{};
        c.cfg.rate = rate;
        c.ms = 400;
        c.value = 0x00;
        c.aux16 = 10;
        const bool armed = peer_act(link::Op::collide, c);
        const auto reg = sercom_closest_baud(xtal_hz, rate).value();
        const SercomUartConfig u{.pads = link_pads, .baud = reg.reg, .sampling = reg.sampling,
                                 .collision_detect = true};
        const bool up = armed && dut_up(u, rate);
        bool trigger_clean = false, coll = false, error = false, txc = false, tx_off = false;
        bool flushed = false;
        bool restarts_reenable = false, restarts_clear = false, restarts_reconf = false;
        uint8_t own = 0, sent = 0;
        if (up) {
            wait_for_peer_window();
            (void)apply(talk_cfg(cur));
            Lk::flush_rx();
            // THREE frames and no more: the trigger, the frame of ones the far
            // end collides with, and one waiting in DATA behind it - the one
            // the collision must flush. Nothing written after: a write would
            // fill DATA again, and TXC rises only on an empty DATA (31.8.8).
            // The flags and the wire are LOGGED through it (the pad log):
            // when ERROR rises, and whether TXC ever does.
            const uint8_t three[3] = {0xA5, 0xFF, 0x33};
            PadLog trace{};
            pad_log_feeding(trace, RxdPin::mask,
                            SercomFlag::txc | SercomFlag::error | SercomFlag::dre, three, 3,
                            3'000u);
            sent = 3;
            for (uint8_t i = 0; i < trace.n; ++i) {
                if ((trace.flags[i] & SercomFlag::error) != 0u) {
                    print(plain, "    the log from ERROR on (turn: PA16, TXC/ERROR/DRE):");
                    for (uint8_t j = i > 0u ? static_cast<uint8_t>(i - 1u) : 0u; j < trace.n; ++j) {
                        print(plain, " ", trace.turn[j], ":", (trace.pins[j] & RxdPin::mask) != 0u ? 1 : 0,
                              "/", (trace.flags[j] & SercomFlag::txc) != 0u ? 1 : 0,
                              (trace.flags[j] & SercomFlag::error) != 0u ? 1 : 0,
                              (trace.flags[j] & SercomFlag::dre) != 0u ? 1 : 0);
                    }
                    print(plain, crlf);
                    break;
                }
            }
            for (uint8_t k = 0; k < 4u; ++k) {
                const Got g = take();
                if (!g.any) break;
                if (own == 0u) trigger_clean = g.clean && static_cast<uint8_t>(g.data) == 0xA5u;
                ++own;
            }
            const uint16_t st = Lk::status();
            coll = (st & SercomStatus::collision) != 0u;
            error = (Lk::flags() & SercomFlag::error) != 0u;
            txc = Lk::txc_flag();
            (void)Lk::wait_sync(SERCOM_USART_INT_SYNCBUSY_CTRLB_Msk);
            tx_off = !Lk::tx_enabled();
            // FLUSHED: the frame DATA held at the collision does not go out
            // when the transmitter comes back (the far end is listening
            // again by now, its collide op over).
            Lk::flush_rx();
            const bool dre_before = Lk::dre_flag();
            const bool reenabled = Lk::enable_tx(true);
            spin_us(200);
            flushed = !Lk::rxc_flag();
            const Got back = take();
            spin_ms(1);
            const bool dre_later = Lk::dre_flag();
            Lk::data(0x5A);
            spin_us(200);
            const Got fresh = take();
            const bool txc_fresh = Lk::txc_flag();
            print(plain, "    after the collision: DRE ", dre_before ? 1 : 0, "; enable_tx(true) ",
                  reenabled ? "synchronized" : "TIMED OUT", ", TXEN ", Lk::tx_enabled() ? 1 : 0,
                  ", a frame out at the re-enable: ", back.any ? "YES " : "none ",
                  back.any ? hex(back.data) : hex(0u), ", DRE a millisecond on ",
                  dre_later ? 1 : 0, "; a fresh 0x5A written: ", fresh.any ? "heard " : "NOT heard ",
                  fresh.any ? hex(fresh.data) : hex(0u), ", TXC ", txc_fresh ? 1 : 0, crlf);
            // COLL and ERROR cleared, the transmitter cycled: does it restart?
            Lk::clear_status(SercomStatus::collision);
            Lk::clear_flags(SercomFlag::error | SercomFlag::txc);
            (void)Lk::enable_tx(false);
            (void)Lk::enable_tx(true);
            spin_us(200);
            const bool dre_cleared = Lk::dre_flag();
            (void)take();
            Lk::data(0x5B);
            spin_us(200);
            const Got after_clear = take();
            // And the instance reconfigured.
            (void)apply(talk_cfg(cur));
            Lk::flush_rx();
            const bool dre_reconf = Lk::dre_flag();
            Lk::data(0x5C);
            spin_us(200);
            const Got after_reconf = take();
            restarts_clear = after_clear.any && static_cast<uint8_t>(after_clear.data) == 0x5Bu;
            restarts_reconf = after_reconf.any && static_cast<uint8_t>(after_reconf.data) == 0x5Cu;
            print(plain, "    COLL and ERROR cleared, TXEN cycled: DRE ", dre_cleared ? 1 : 0,
                  ", a fresh 0x5B ", restarts_clear ? "heard" : "NOT heard",
                  "; the instance reconfigured: DRE ", dre_reconf ? 1 : 0, ", a fresh 0x5C ",
                  restarts_reconf ? "heard" : "NOT heard", crlf);
            restarts_reenable = fresh.any && static_cast<uint8_t>(fresh.data) == 0x5Au;
            (void)apply(listen_cfg(cur));
            Lk::flush_rx();
        }
        link::Report r{};
        const bool have = peer_report(r, 20);
        print(plain, "  collision at 250000: trigger echoed ", trigger_clean ? "clean" : "WRONG",
              ", frames heard ", own, " of ", sent, " written, COLL ", coll ? 1 : 0, " ERROR ",
              error ? 1 : 0, " TXC ", txc ? 1 : 0, " TXEN ", tx_off ? 0 : 1, "; peer took ",
              have ? r.count : 0u, " and sent ", hex(have ? r.aux16 : 0u), crlf);
        judge("the trigger frame is clean on the wire", up && trigger_clean);
        judge("a far transmitter pulling low into this frame raises COLL and ERROR",
              up && coll && error);
        judge("the transmitter is disabled: TXEN cleared (31.6.3.7)", up && tx_off);
        print(plain, "    TXC ", txc ? "raised" : "NOT raised", " after the collision; restarted by TXEN alone ",
              restarts_reenable ? "yes" : "no", ", by COLL cleared and TXEN cycled ",
              restarts_clear ? "yes" : "no", ", by a reconfiguration ", restarts_reconf ? "yes" : "no",
              crlf);
        judge("a reconfiguration restarts the transmitter", up && restarts_reconf);
        judge("the frame waiting in DATA was flushed: nothing leaves at the re-enable",
              up && coll && sent == 3u && flushed);
    }
    leave_shared();
    link_end();
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
          "  letters e..p and s need brio stress on the other end; x stops the board;", crlf,
          "  J..W need the far board's usart_peer on SERCOM1's four wires", crlf);
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
        case Loop::link_dma: (void)LkDma::isr(); break;
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
        // SERCOM1 has two engined transports, never live at once.
        if (loop_live == Loop::link_dma) {
            LkDma::dma_isr();
        } else {
            LTxDma::dma_isr();
        }
    } else if (owner == console_owner) {
        UTxDma::dma_isr();
    } else {
        // Letter w's hand-staged bindings: the flags taken and kept, so a
        // completion nobody consumes cannot hold the line up.
        probe_flags = static_cast<uint8_t>(probe_flags | brio::DmaTxEngine::take_interrupt());
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
    bench.letter('g', "the resource's regimes and its directions, on the loop", tg_resource);

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
    bench.letter('w', "a released requester: what reaches the next owner of the channel",
                 tw_released_requester);
    bench.letter('x', "the one-user rule as a PANIC (stops the board)", tx_refusal, false);

    bench.second_letter('J', "the link and the far peer", tJ_link);
    bench.second_letter('K', "the rate, judged by a foreign receiver on a crystal", tK_rate);
    bench.second_letter('L', "the frame matrix against a different UART", tL_frames);
    bench.second_letter('M', "errors with known causes, the buffer, start-of-frame", tM_errors);
    bench.second_letter('N', "the bit-banger: the operational range, a cell, the start vote",
                        tN_waveforms);
    bench.second_letter('O', "auto-baud from a foreign sender, and the LIN header", tO_autobaud);
    bench.second_letter('Q', "the synchronous roles on a real XCK", tQ_sync);
    bench.second_letter('S', "IrDA both ways, and RXPL", tS_irda);
    bench.second_letter('C', "hardware handshaking: CTS against a far level, RTS on its pad",
                        tC_flow);
    bench.second_letter('R', "RS-485: TE timed on its own pad (the shared wire)", tR_rs485);
    bench.second_letter('W', "one-wire and collision detection (the shared wire)", tW_onewire);

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
