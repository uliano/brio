// test_x035_spi - the reference bench suite for the SPI of the CH32X035
// (RM ch. 16): ch32x035/spi.hpp over the chapter - the block, the rate
// ladder on the wire, the high-speed read, the CRC, the refusals, the
// pump, the polled loops and the DMA engines on channels 2 and 3 - and,
// across the desk's link to a CH32V006 running spi_peer, the command
// channel, the four modes and both bit orders against a foreign client,
// the ladder against it, util/spi_bus.hpp's arbiter, THIS CHIP AS THE
// CLIENT of a foreign host in every mode and order (the chapter's two
// client restrictions measured), and MODF from a second chip.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test.
//
// THE CONSOLE IS USART2 on PA2/PA3 over the probe's serial. The instance
// under test is SPI1 on its one column of this package, wired straight to
// the CH32V006's SPI1 column 0:
//
//   PA5 (SCK)   - PC5        PA7 (MOSI) - PC6
//   PA6 (MISO)  - PC7        PA4 (NSS)  - PC1, the V006's hardware select
//
// The suite has TWO SETS:
//
//  - `z`, THE SINGLE-BOARD LETTERS (a, b, c), runnable with the far board
//    dark - on its console, or on its spi_peer, whose dark listener drives
//    nothing and is selected by nothing: PA4 is this end's GPIO select,
//    held HIGH through every one of them, and what goes out on SCK and
//    MOSI reaches a deselected client or a floating pad. MISO is read
//    floating and never judged.
//  - `y`, THE TWO-BOARD LETTERS (n, o, p, k, q, r, m, w), against the
//    CH32V006 running spi_peer on its 24 MHz crystal, commanded IN BAND
//    over the bus under test through avrdx/src/apps/spi_link.hpp - FAILING
//    loudly when nobody answers. That peer serves its answer windows in
//    mode 3, spi_link's alternate answer mode, and the rendezvous finds it.
//
// What is exercised, letter by letter:
//   a  the block, WIRELESS: table 16-2's reset values, the control words,
//      the enable protection MEASURED field by field, HSCR's "not
//      readable" read, ODEN, the select an SSOE host drives, the transmit
//      CRC against a bitwise reference, and the pump, the polled loops and
//      the engines each run to completion with nothing on MISO
//   b  THE RATE LADDER ON THE WIRE, the select high: a frame inside an
//      engined block timed (the difference of two block lengths) against
//      its SCK periods at every BR code, 8 and 16 bits; THE HIGH-SPEED
//      READ's rate at every code (16.3.1's FHCLK/(BR+2) against 16.3.8's
//      "/2 only"); and THE HOST'S COSTS - the engines' fixed cost, the
//      pump's frame, the polled loops' frames - IN BOTH PLACEMENTS, the
//      suite's hosts in SRAM and their flash twins, each judged against
//      the constants the driver states for its placement, with the
//      hold-off this image's handlers measure, and the pump with two
//      frames in flight under the console's interrupts
//   c  the refusals: the chapter's, the high-speed read off /2 at run
//      time, a ceiling under HCLK/256, and the client configurations this
//      die does not serve
//   n  (y) THE PEER: the command channel, ident (the CH32V006, kind 0x06),
//      ten pings
//   o  (y) the matrix: this host in the four modes and both bit orders
//      against the foreign client on both of its serves, each direction
//      judged where that client is served (modes 0 and 3), a bit-order
//      mismatch as an exact two-way reversal, and the receive CRC over the
//      foreign frames
//   p  (y) THE LADDER against the peer, /256 to /2 on the polled host and
//      on the engines, both of the peer's serves, /2 = 24 MHz with and
//      without the high-speed read, and the high-speed read's own ladder
//   k  (y) THE CLIENT'S RUNS: the peer's answer, exchange and receive runs
//      (its driver's SpiClient runs), 8- and 16-bit frames, under the
//      polled host - a gap after every frame, the control - and back to
//      back under the engines, /64 to /2, judged to the pair's 6 MHz
//   q  (y) THE KERNEL: SpiBus over SpiHost, four transactions queued in
//      one select window, read back by the second chip
//   r  (y) THE ROLES INVERT: this chip as the CLIENT under the peer's
//      host_burst, in all four modes and both bit orders - 16.2.3's "mode
//      2 or mode 3" and 16.3.1's "LSB only as host" measured
//   m  (y) MODF from a second chip: a host whose NSS is a hardware input,
//      demoted four times by the peer's ss_pulse on PA4, and the clearing
//      sequence; and ODEN - SCK on a wire with no pull-up, NSS on the
//      peer's
//   w  (y) THE WAVEFORMS: this chip as the logic analyser of its four link
//      pads - both hosts' SCK and MOSI, both clients' MISO, every mode
//   x  (by name) the slip statistics: each mode ten times
//
// THE SUPPLY at boot: the voltage detector at 3.0 V (pwr.hpp); an image
// that drives a link pad from a sagging rail refuses every letter and
// says so.
//
// build: boards = x035f8
// build: monitor_speed = 115200

#include <stdint.h>

#include <span>
#include <variant>

#include "ch32x035/clock.hpp"
#include "ch32x035/delay.hpp"
#include "ch32x035/dma.hpp"
#include "ch32x035/pfic.hpp"
#include "ch32x035/pin.hpp"
#include "ch32x035/platform.hpp"
#include "ch32x035/pwr.hpp"
#include "ch32x035/spi.hpp"
#include "ch32x035/ticker.hpp"
#include "ch32x035/usart.hpp"
#include "kernel/post.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time.hpp"
#include "util/placement.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/spi_bus.hpp"
#include "util/testbench.hpp"

#include "../../../avrdx/src/apps/spi_link.hpp"

using P = brio::Ch32x035Platform<>;
using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<2, P>;
constexpr Serial serial;

using S = Spi<1>;
// THE SUITE OPTS INTO SRAM for its two hosts (util/placement.hpp's InRam):
// its loop verdicts are the wire's. Their FLASH TWINS - the default a
// program gets, the same instance and engines - are letter b's other
// column, so that every cost the driver states is measured both ways.
using Host = SpiHost<1, spi1_default_pins, NoDmaEngine, NoDmaEngine, spi_default_hold_off_cycles, InRam>;
using DmaHost = SpiHost<1, spi1_default_pins, DmaTxEngine<3, uint16_t>, DmaRxEngine<2, uint16_t>,
                        spi_default_hold_off_cycles, InRam>;
using FlashHost = SpiHost<1>;
using FlashDmaHost = SpiHost<1, spi1_default_pins, DmaTxEngine<3, uint16_t>, DmaRxEngine<2, uint16_t>>;
using Client = SpiClient<1>;
using CsPin = Pin<'A', 4>;
using SckPin = Pin<'A', 5>;
using MisoPin = Pin<'A', 6>;
using MosiPin = Pin<'A', 7>;

TwoKeyTestBench<Serial, 'y'> bench;

/// Which task owns SPI1 and its two channels: the vectors serve it.
enum class Owner : uint8_t { host, dma_host, flash_host, flash_dma_host, bus_ao, none };
volatile Owner owner = Owner::host;
volatile bool host_done = false;
volatile uint32_t spi_isr_entries = 0;
volatile uint32_t dma_isr_entries = 0;

/// The image's own hold-off, measured: the longest body each handler ran,
/// stamped on the STK at its first and last statement.
volatile uint32_t console_body_max = 0;
volatile uint32_t tick_body_max = 0;
volatile uint32_t spi_body_max = 0;

// Word-aligned: a 16-bit request on the engines needs half-word boundaries.
alignas(4) uint8_t tx_buf[512];
alignas(4) uint8_t rx_buf[512];
uint8_t cmd_buf[4];

/// A transaction that never answered: the suite's own word.
constexpr uint8_t no_answer = 200;

uint32_t cycles_now() { return Ticker::cycles(); }

/// The STK's own count, for a handler's body: one load each side.
[[gnu::always_inline]] inline uint32_t stamp() { return stk()->CNTL; }
[[gnu::always_inline]] inline uint32_t stamp_span(uint32_t t0, uint32_t t1) {
    return t1 >= t0 ? t1 - t0 : t1 + stk()->CMPLR + 1u - t0;
}

void console_drain() {
    while (!Serial::tx_idle()) {
    }
    (void)delay_us(clock, 300);
}

void settle_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

void fill_pattern(uint8_t* p, uint16_t n, uint8_t seed) {
    for (uint16_t i = 0; i < n; ++i) {
        p[i] = static_cast<uint8_t>(seed + i * 7u + (i >> 3));
    }
}

bool same(const uint8_t* a, const uint8_t* b, uint16_t n) {
    for (uint16_t i = 0; i < n; ++i) {
        if (a[i] != b[i]) {
            return false;
        }
    }
    return true;
}

/// The owner the vectors serve for a host type.
template <typename H>
constexpr Owner owner_of() {
    if constexpr (std::is_same_v<H, Host>) {
        return Owner::host;
    } else if constexpr (std::is_same_v<H, DmaHost>) {
        return Owner::dma_host;
    } else if constexpr (std::is_same_v<H, FlashHost>) {
        return Owner::flash_host;
    } else {
        static_assert(std::is_same_v<H, FlashDmaHost>);
        return Owner::flash_dma_host;
    }
}

/// The plain host, PA4 its GPIO select held high.
template <typename H = Host>
void host_ready() {
    owner = owner_of<H>();
    (void)H::init(clock);
    CsPin::output(true);
}

template <typename H = DmaHost>
void dma_ready(bool high_speed = false) {
    owner = owner_of<H>();
    (void)H::init(clock, {.high_speed_read = high_speed});
    CsPin::output(true);
}

template <typename H = Host>
uint8_t host_xfer(PinRef cs, const uint8_t* cmd, uint8_t cmd_len, const uint8_t* tx, uint8_t* rx, uint16_t len,
                  SpiMode mode, SpiClock rate, SpiDataSize bits, bool polled) {
    typename H::Request r{};
    r.cs = cs;
    r.cmd = lend<Lease::reply>(cmd);
    r.cmd_len = cmd_len;
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = len;
    r.mode = mode;
    r.clock = rate;
    r.bits = bits;
    r.polled = polled;
    host_done = false;
    spi_isr_entries = 0;
    if (H::start(r)) {
        return H::status();
    }
    for (uint32_t i = 0; i < 8'000'000UL && !host_done; ++i) {
    }
    if (!host_done) {
        print(serial, "    STALL: STATR=", hex(S::status()), " CTLR1=", hex(S::regs().CTLR1), " CTLR2=",
              hex(S::regs().CTLR2), " isr entries ", spi_isr_entries, crlf);
        (void)H::recover();
        return no_answer;
    }
    return H::status();
}

template <typename H = DmaHost>
uint8_t dma_xfer(PinRef cs, const uint8_t* cmd, uint8_t cmd_len, const uint8_t* tx, uint8_t* rx, uint16_t len,
                 SpiClock rate, SpiDataSize bits, bool polled, SpiMode mode = SpiMode::mode0) {
    typename H::Request r{};
    r.cs = cs;
    r.cmd = lend<Lease::reply>(cmd);
    r.cmd_len = cmd_len;
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = len;
    r.mode = mode;
    r.clock = rate;
    r.bits = bits;
    r.polled = polled;
    host_done = false;
    if (H::start(r)) {
        return H::status();
    }
    for (uint32_t i = 0; i < 8'000'000UL && !host_done; ++i) {
    }
    if (!host_done) {
        print(serial, "    STALL (dma): STATR=", hex(S::status()), " ch2 flags=", hex(DmaChannel<2>::flags()),
              " ch3 flags=", hex(DmaChannel<3>::flags()), crlf);
        (void)H::recover();
        return no_answer;
    }
    return H::status();
}

/// The SCK divisor a BR code ASKS for, HCLK / 2^(BR + 1) (16.3.1).
uint32_t asked_divisor(SpiClock rate) { return 2u << static_cast<uint8_t>(rate); }

/// ONE FRAME's time inside an engined block, from the difference of a
/// `big_n`- and a `small_n`-frame block, in HCLK cycles x 1000 per frame:
/// the transaction's fixed cost cancels. The best of three of each; the
/// select is the GPIO PA4, held HIGH (the request names no select).
template <typename H = DmaHost>
uint32_t frame_x1000(SpiClock rate, SpiMode mode, SpiDataSize bits, uint16_t big_n, uint16_t small_n,
                     uint32_t* fixed = nullptr) {
    auto best = [&](uint16_t n) {
        uint32_t least = 0xFFFFFFFFu;
        for (uint8_t k = 0; k < 3u; ++k) {
            const uint32_t t0 = cycles_now();
            (void)dma_xfer<H>({}, nullptr, 0, tx_buf, rx_buf, n, rate, bits, false, mode);
            const uint32_t took = cycles_now() - t0;
            least = took < least ? took : least;
        }
        return least;
    };
    const uint32_t big = best(big_n);
    const uint32_t small = best(small_n);
    if (big <= small) {
        return 0;
    }
    const uint32_t frames = static_cast<uint32_t>(big_n - small_n);
    const uint32_t d = big - small;
    const uint32_t f1000 = (d / frames) * 1000u + ((d % frames) * 1000u) / frames;
    if (fixed != nullptr) {
        const uint32_t wire_small = (f1000 / 1000u) * small_n;
        *fixed = small > wire_small ? small - wire_small : 0u;
    }
    return f1000;
}

uint32_t permille_of(uint32_t f1000, uint32_t width, uint32_t divisor) { return f1000 / (width * divisor); }

bool wire_ok(uint32_t permille) { return permille >= 990u && permille <= 1030u; }

// ===========================================================================
// The peer: spi_link.hpp over the bus under test
// ===========================================================================

using spilink::Op;

/// The command channel's SCK: HCLK/256 = 187.5 kHz, a character 43 us.
constexpr SpiClock link_clock = SpiClock::div256;

/// THE ANSWER WINDOWS' MODE. spi_link clocks them in mode 0; a peer whose
/// client cannot transmit in mode 0 - the CH32V006, measured (its
/// spi_peer's header) - serves them in mode 3, and the rendezvous finds
/// which (ensure_link): the command windows stay mode 0 either way.
SpiMode answer_mode = SpiMode::mode0;

uint8_t frame_buf[spilink::max_payload + 8];
uint8_t answer_buf[spilink::answer_bytes];
uint8_t raw_seen[16];
uint8_t raw_n = 0;
spilink::Decoder dec;
bool link_quiet = false;

/// The hold around each select window: the protocol owns the select for
/// these windows (the Request carries a null PinRef) and pays 30 us on
/// each side, for the client's loop to turn the edge around.
void link_hold() { (void)delay_us(clock, 30); }

/// SPI1 as the command channel's host, PA4 as its GPIO chip select.
bool link_command_mode() {
    owner = Owner::host;
    const bool ok = Host::init(clock);
    CsPin::output(true);
    dec.reset();
    return ok;
}

/// One protocol window: prime, select, hold, the transaction, hold,
/// deselect. THE MODE IS PRIMED BEFORE THE SELECT FALLS - a CPOL flip in
/// an open window is an edge the selected client counts.
bool link_xfer(const uint8_t* tx, uint8_t* rx, uint16_t n, SpiClock rate = link_clock,
               SpiMode mode = SpiMode::mode0, SpiDataSize bits = SpiDataSize::bits8) {
    if (n == 0) {
        return true;
    }
    Host::prime(mode, rate, bits);
    CsPin::clear();
    link_hold();
    Host::Request r{};
    r.cs = {};
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
    r.cmd_len = 0;
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = n;
    r.clock = rate;
    r.mode = mode;
    r.bits = bits;
    r.polled = true;
    const bool done = Host::start(r);
    link_hold();
    CsPin::set();
    return done;
}

void send_frame(Op op, const uint8_t* p, uint8_t len) {
    uint8_t n = 0;
    spilink::write_frame(
        [&](uint8_t b) {
            if (n < sizeof frame_buf) {
                frame_buf[n++] = b;
            }
        },
        op, p, len);
    (void)link_xfer(frame_buf, nullptr, n);
}

/// One answer window: `answer_bytes` dummies in one transaction, then the
/// decoder walks what came back (the peer pads its frame with zeros).
bool recv_frame(spilink::Frame& out) {
    dec.reset();
    raw_n = 0;
    for (uint16_t i = 0; i < spilink::answer_bytes; ++i) {
        answer_buf[i] = 0xEE;
    }
    (void)link_xfer(nullptr, answer_buf, spilink::answer_bytes, link_clock, answer_mode);
    for (uint16_t i = 0; i < spilink::answer_bytes; ++i) {
        if (raw_n < 16) {
            raw_seen[raw_n++] = answer_buf[i];
        }
        if (dec.feed(answer_buf[i]) == spilink::Decoder::Result::frame) {
            out = dec.frame();
            return true;
        }
    }
    return false;
}

void settle() { settle_ms(spilink::settle_ms); }

bool command_once(Op op, const uint8_t* p, uint8_t len) {
    send_frame(op, p, len);
    settle();
    spilink::Frame f;
    if (!recv_frame(f)) {
        return false;
    }
    return f.op == Op::ack && f.len == 2 && f.data[0] == spilink::byte_of(op);
}

const uint8_t no_payload[1] = {0};

/// Three attempts, each separated by longer than the peer's own answer
/// window bound - the protocol's recovery guarantee.
bool command(Op op, const uint8_t* p = no_payload, uint8_t len = 0) {
    uint8_t first_n = 0;
    uint8_t first_seen[16];
    for (uint8_t k = 0; k < 3; ++k) {
        if (command_once(op, p, len)) {
            return true;
        }
        if (k == 0) {
            first_n = raw_n;
            for (uint8_t i = 0; i < raw_n; ++i) {
                first_seen[i] = raw_seen[i];
            }
        }
        (void)link_command_mode();
        settle_ms(400);
    }
    if (link_quiet) {
        return false;
    }
    print(serial, "    LINK FAILURE op ", hex(spilink::byte_of(op)), ": the first answer window carried");
    if (first_n == 0) {
        print(serial, " nothing");
    }
    for (uint8_t i = 0; i < first_n; ++i) {
        print(serial, " ", hex(first_seen[i]));
    }
    print(serial, crlf, "      the far board must be running `spi_peer`; its console '0' forces the dark client "
                        "back.", crlf);
    (void)link_command_mode();
    return false;
}

bool query(Op op, spilink::Frame& data) {
    if (!command(op)) {
        return false;
    }
    settle();
    return recv_frame(data);
}

bool peer_ident(spilink::Ident& d) {
    spilink::Frame f;
    if (!query(Op::ident, f) || f.op != Op::ident_data || f.len != spilink::ident_size) {
        return false;
    }
    d = spilink::get_ident(f.data);
    return true;
}

bool peer_report(spilink::Report& r) {
    (void)link_command_mode();
    for (uint8_t k = 0; k < 4; ++k) {
        spilink::Frame f;
        if (query(Op::report, f) && f.op == Op::report_data && f.len == spilink::report_size) {
            r = spilink::get_report(f.data);
            return true;
        }
        settle_ms(60);
    }
    return false;
}

bool peer_act(Op op, const spilink::Params& a) {
    uint8_t p[spilink::params_size];
    spilink::put_params(p, a);
    if (!command(op, p, spilink::params_size)) {
        return false;
    }
    settle();
    return true;
}

/// Is the peer there at all? Every letter that needs it asks first, so an
/// absent peer is a NAMED failure and never a hang.
bool ensure_link() {
    link_quiet = true;
    (void)link_command_mode();
    // The answer mode that worked last first, then the other.
    const SpiMode tries[2] = {answer_mode, answer_mode == SpiMode::mode0 ? SpiMode::mode3 : SpiMode::mode0};
    for (const SpiMode m : tries) {
        answer_mode = m;
        for (uint8_t k = 0; k < 2; ++k) {
            if (command(Op::ping)) {
                link_quiet = false;
                return true;
            }
        }
    }
    link_quiet = false;
    print(serial, "  THE PEER DID NOT ANSWER. The far board must be running `spi_peer`; its console '0' "
                  "forces the dark client back. Check the four wires in this file's header.", crlf);
    return false;
}

bool need_peer() {
    if (ensure_link()) {
        return true;
    }
    bench.verdict("the peer answers a ping (the CH32V006 running spi_peer)", false);
    return false;
}

// ---- one exchange, commanded and then clocked ------------------------------

/// One exchange. Mode 3 by default: the mode the desk's CH32V006 client
/// answers exactly in (letter o's matrix walks the others).
struct Exchange {
    spilink::Cfg cfg{.apply = 1, .mode = 3, .dord = 0, .regime = spilink::regime_buffer_wait};
    SpiMode host_mode = SpiMode::mode3;
    bool host_lsb = false;
    SpiClock rate = link_clock;
    uint16_t count = 8;
    uint8_t seed_a = 0x13;
    uint8_t seed_b = 0x57;
    uint8_t pattern = spilink::pattern_prbs;
    uint8_t flags = 0;
    uint8_t spare = 0;   ///< spilink::spare_polled_pump forces the peer's polled run
    bool wide = false;   ///< 16-bit frames (spilink::spare_wide), `count` frames
    uint16_t ms = 400;
    bool engines = false;      ///< this end clocks the data on its DMA engines
    bool high_speed = false;   ///< the high-speed read asked (at /2)
    bool raw_hsrxen = false;   ///< HSRXEN set by hand at whatever code `rate` is
};

/// The most BYTES an exchange carries (64 frames of 8 bits, 64 of 16).
constexpr uint8_t max_exchange = 128;
alignas(4) uint8_t xtx[max_exchange];
alignas(4) uint8_t xrx[max_exchange];

/// The bytes an exchange's frames take.
uint16_t exchange_bytes(const Exchange& e) {
    const uint16_t b = static_cast<uint16_t>(e.wide ? 2u * e.count : e.count);
    return b < max_exchange ? b : max_exchange;
}

bool do_exchange(const Exchange& e) {
    spilink::Params a{};
    a.cfg = e.cfg;
    a.count = e.count;
    a.ms = e.ms;
    a.seed_a = e.seed_a;
    a.seed_b = e.seed_b;
    a.pattern = e.pattern;
    a.flags = e.flags;
    a.spare = static_cast<uint8_t>(e.spare | (e.wide ? spilink::spare_wide : 0u));
    if (e.host_lsb != (e.cfg.dord != 0)) {
        a.flags |= spilink::flag_expect_reversed;
    }
    if (!peer_act(Op::exchange, a)) {
        return false;
    }
    spilink::Stream out(e.pattern, e.seed_a);
    for (uint8_t i = 0; i < max_exchange; ++i) {
        xtx[i] = 0;
        xrx[i] = 0xEE;
    }
    const uint16_t bytes = exchange_bytes(e);
    const uint16_t n = static_cast<uint16_t>(e.wide ? bytes / 2u : bytes);   // frames
    const SpiDataSize bits = e.wide ? SpiDataSize::bits16 : SpiDataSize::bits8;
    for (uint16_t i = 0; i < bytes; ++i) {
        xtx[i] = out.next();
    }
    settle();
    if (e.engines) {
        dma_ready(e.high_speed);
        if (e.host_lsb) {
            (void)DmaHost::bit_order(true);
        }
        DmaHost::prime(e.host_mode, e.rate);
        if (e.raw_hsrxen) {
            S::high_speed_read_raw(true);   // under the code in force (letter b: HCLK/(BR+2))
        }
        (void)delay_us(clock, 20);
        CsPin::clear();
        link_hold();
        (void)dma_xfer({}, nullptr, 0, xtx, xrx, n, e.rate, bits, false, e.host_mode);
        link_hold();
        CsPin::set();
        S::high_speed_read_raw(false);
    } else {
        if (e.host_lsb) {
            (void)Host::bit_order(true);
        }
        (void)link_xfer(xtx, xrx, n, e.rate, e.host_mode, bits);
    }
    (void)link_command_mode();
    return true;
}

struct Verify {
    uint16_t mism = 0;
    uint8_t idx = 0xFF;
    uint8_t got = 0;
    uint8_t exp = 0;
};

Verify verify_rx(const Exchange& e) {
    Verify v{};
    const bool reversed = e.host_lsb != (e.cfg.dord != 0);
    const uint16_t bytes = exchange_bytes(e);
    for (uint16_t i = 0; i < bytes; ++i) {
        const uint8_t exp = spilink::wide_byte(e.pattern, e.seed_b, i, e.wide, reversed);
        if (xrx[i] != exp) {
            if (v.mism == 0) {
                v.idx = static_cast<uint8_t>(i);
                v.got = xrx[i];
                v.exp = exp;
            }
            ++v.mism;
        }
    }
    return v;
}

bool exchange_exact(const Exchange& e, Verify& v, spilink::Report& r) {
    const bool ran = do_exchange(e);
    const bool rep = ran && peer_report(r);
    v = verify_rx(e);
    return ran && rep && v.mism == 0 && r.mism == 0 && r.count == e.count;
}

void dump_exchange(const Exchange& e, const Verify& v, const spilink::Report& r) {
    print(serial, "    host mism=", v.mism, " (first idx ", v.idx, " got ", hex(v.got), " exp ", hex(v.exp),
          "), client count=", r.count, " mism=", r.mism, " (first idx ", r.idx, " got ", hex(r.got), " exp ",
          hex(r.exp), ") flags=", hex(r.flags), crlf);
    print(serial, "    read:");
    for (uint16_t i = 0; i < e.count && i < 16u; ++i) {
        print(serial, " ", hex(xrx[i]));
    }
    print(serial, crlf);
}

/// The chapter's CRC, bitwise: MSB first, no reflection, init 0.
uint16_t crc_reference(uint16_t poly, uint8_t width, const uint8_t* data, uint16_t n) {
    uint32_t crc = 0;
    const uint32_t top = 1u << (width - 1u);
    const uint32_t mask = (1u << width) - 1u;
    for (uint16_t k = 0; k < n; ++k) {
        for (uint8_t b = 8; b-- > 0;) {
            const bool in = ((data[k] >> b) & 1u) != 0u;
            const bool out = (crc & top) != 0u;
            crc = (crc << 1) & mask;
            if (in != out) {
                crc ^= poly;
            }
        }
    }
    return static_cast<uint16_t>(crc);
}

/// Frames by hand on the resource: write, spin RXNE, read. For the CRC.
bool frame_by_hand(uint8_t v, uint8_t& got, bool last_crc_next) {
    S::data(v);
    if (last_crc_next) {
        S::crc_next();
    }
    uint32_t spins = 400'000u;
    while (!S::rxne() && spins-- != 0u) {
    }
    got = static_cast<uint8_t>(S::data());
    return spins != 0u;
}

bool wait_not_busy() {
    uint32_t spins = 400'000u;
    while (S::busy() && spins-- != 0u) {
    }
    return spins != 0u;
}

// ===========================================================================
// a - the block, wireless
// ===========================================================================

void ta_block() {
    owner = Owner::none;
    CsPin::output(true);
    Pfic::disable(S::irq());
    SckPin::release();
    MosiPin::release();
    MisoPin::release();
    S::bus_clock(true);
    S::reset();
    const uint16_t c1 = S::regs().CTLR1, c2 = S::regs().CTLR2, st = S::status(), poly = S::regs().CRCR;
    const uint16_t rc = S::regs().RCRCR, tc = S::regs().TCRCR, hs = S::hscr_register();
    print(serial, "  reset: CTLR1=", hex(c1), " CTLR2=", hex(c2), " STATR=", hex(st), " CRCR=", hex(poly),
          " RCRCR=", hex(rc), " TCRCR=", hex(tc), " HSCR=", hex(hs), crlf);
    bench.verdict("the reset values are table 16-2's (STATR 0x0002: TXE up, CRCR 0x0007, the rest zero)",
                  c1 == 0u && c2 == 0u && st == 0x0002u && poly == 0x0007u && rc == 0u && tc == 0u);

    const bool cfg = S::configure({.role = SpiRole::host, .mode = SpiMode::mode3, .clock = SpiClock::div16,
                                   .bits = SpiDataSize::bits16});
    print(serial, "  mode 3 host /16 16-bit: CTLR1=", hex(S::regs().CTLR1), crlf);
    bench.verdict("the control word lands as spelled (CPHA, CPOL, MSTR, BR 3, SSM, SSI, DFF)",
                  cfg && S::regs().CTLR1 == (spi_cpha | spi_cpol | spi_mstr | (3u << 3) | spi_ssm | spi_ssi |
                                             spi_dff));
    bench.verdict("the arithmetic: /2 is 24 MHz (table 3-19's ceiling), /256 is 187.5 kHz, 1 MHz asks /64, below "
                  "187.5 kHz is refused",
                  spi_sck_hz(SysClock::hz, SpiClock::div2) == 24'000'000UL &&
                      spi_rate_for(SysClock::hz, 1'000'000UL) == SpiClock::div64 &&
                      !spi_rate_for(SysClock::hz, 100'000UL).has_value());

    // THE ENABLE PROTECTION, MEASURED FIELD BY FIELD: 16.3.1 says DFF and
    // CRCEN "can only be written when SPE is 0" and MSTR, CPOL, CPHA and
    // LSBFIRST must not be "modified during communication". A raw write to
    // each with SPE set (the pads floating: nothing reaches the wire).
    (void)S::configure({.role = SpiRole::host, .mode = SpiMode::mode0, .clock = SpiClock::div16,
                        .bits = SpiDataSize::bits8});
    S::enable();
    const uint16_t base = S::regs().CTLR1;
    struct Field {
        const char* name;
        uint16_t bit;
    };
    const Field fields[] = {{"DFF", spi_dff},           {"CRCEN", spi_crcen}, {"CPOL", spi_cpol},
                            {"CPHA", spi_cpha},         {"LSBFIRST", spi_lsbfirst}, {"BR0", 1u << 3},
                            {"MSTR", spi_mstr},         {"BIDIMODE", spi_bidimode}};
    uint8_t taken = 0;
    print(serial, "  with SPE set, a write lands on:");
    for (const auto& f : fields) {
        S::regs().CTLR1 = static_cast<uint16_t>(base ^ f.bit);
        const bool landed = ((S::regs().CTLR1 ^ base) & f.bit) != 0u;
        S::regs().CTLR1 = base;
        if (landed) {
            ++taken;
        }
        print(serial, " ", f.name, landed ? "=yes" : "=NO");
    }
    print(serial, crlf, "  -> ", taken, " of 8 fields take a write under SPE", crlf);
    bench.verdict("the enable protection was measured field by field (a finding either way)", true);
    (void)S::disable();
    S::reset();

    // HSCR: 16.3.8 makes HSRXEN write-only ("not readable").
    (void)S::configure({.role = SpiRole::host, .clock = SpiClock::div2});
    S::high_speed_read_raw(true);
    const uint16_t hs_read = S::hscr_register();
    S::high_speed_read_raw(false);
    const uint16_t hs_clear = S::hscr_register();
    print(serial, "  HSCR written 1 reads ", hex(hs_read), ", written 0 reads ", hex(hs_clear),
          " - the copy says ", S::high_speed_read() ? "on" : "off", crlf);
    bench.verdict("HSCR's read was measured with the bit set and clear (16.3.8: \"not readable\"; the "
                  "resource keeps a copy)",
                  !S::high_speed_read());
    bench.verdict("the high-speed read is refused off /2 and on a client, at run time",
                  [] {
                      (void)S::configure({.role = SpiRole::host, .clock = SpiClock::div4});
                      const bool off_div2 = !S::high_speed_read(true);
                      (void)S::configure({.role = SpiRole::client, .nss = SpiNss::software});
                      const bool client = !S::high_speed_read(true);
                      return off_div2 && client && !S::high_speed_read();
                  }());

    // ODEN (16.3.2) lands and reads back.
    S::reset();
    (void)S::configure({.role = SpiRole::host, .open_drain = true});
    const bool oden = (S::regs().CTLR2 & spi_oden) != 0u;
    S::open_drain(false);
    bench.verdict("ODEN lands in CTLR2 bit 15 and reads back", oden && !S::open_drain());

    // THE SELECT AN SSOE HOST DRIVES (16.2.1: "the NSS pin will be actively
    // pulled low when the SPI host sends output outward"): PA4 handed to
    // the block, a host with SSOE at /256 (a frame 2048 cycles), the pad
    // read through INDR - idle with SPE set, inside a frame, after it with
    // SPE still set, across two frames written back to back (one in the
    // shifter, the next in the buffer: inside the second), after both, and
    // with SPE clear. SCK and MOSI stay floating, so the far end sees a
    // select and no clock.
    S::reset();
    (void)S::configure({.role = SpiRole::host, .clock = SpiClock::div256, .nss = SpiNss::hardware_output});
    Host::claim_nss_pad(true);
    (void)delay_us(clock, 20);
    const bool off_before = CsPin::read();
    S::enable();
    (void)delay_us(clock, 20);
    const bool idle_level = CsPin::read();
    S::data(0x00);
    (void)delay_us(clock, 10);
    const bool frame_level = CsPin::read();
    (void)wait_not_busy();
    (void)delay_us(clock, 5);
    const bool after_frame = CsPin::read();
    S::data(0x00);
    while (!S::txe()) {
    }
    S::data(0x00);   // into the buffer behind the frame shifting
    (void)delay_us(clock, 60);   // past the first frame's 43 us: inside the second
    const bool second_level = CsPin::read();
    (void)wait_not_busy();
    (void)delay_us(clock, 5);
    const bool after_two = CsPin::read();
    S::flush_rx();
    (void)S::disable();
    (void)delay_us(clock, 20);
    const bool after = CsPin::read();
    S::reset();
    CsPin::output(true);
    print(serial, "  SSOE on PA4 (push-pull): SPE clear ", off_before ? "high" : "LOW", ", SPE set and idle ",
          idle_level ? "high" : "LOW", ", in a frame ", frame_level ? "high" : "LOW", ", after it ",
          after_frame ? "high" : "LOW", ", in the second of two back to back ", second_level ? "high" : "LOW",
          ", after both ", after_two ? "high" : "LOW", ", SPE cleared ", after ? "high" : "LOW", crlf);
    bench.verdict("SSOE takes the NSS pad LOW at the host's first frame - not at SPE (\"when the SPI host sends "
                  "output\", 16.2.1) - keeps it low between frames while SPE stands, and gives it back HIGH when SPE "
                  "is cleared",
                  off_before && idle_level && !frame_level && !after_frame && !second_level && !after_two && after);

    // THE TRANSMIT CRC, WIRELESS: TCRCR covers what was SENT, so it needs
    // no loop - a host in mode 0 at /64 with CRC-8, six frames by hand,
    // CRCNEXT after the last; the receive side reads a floating MISO and
    // is not judged here (letter o judges it over a foreign chip's frames).
    S::reset();
    (void)S::configure({.role = SpiRole::host, .mode = SpiMode::mode0, .clock = SpiClock::div64,
                        .bits = SpiDataSize::bits8, .crc = true, .crc_polynomial = 0x0007u});
    SckPin::function();
    MosiPin::function();
    MisoPin::input();
    S::enable();
    S::flush_rx();
    uint8_t data[6];
    uint8_t got = 0;
    bool frames_ok = true;
    for (uint8_t i = 0; i < 6u; ++i) {
        data[i] = static_cast<uint8_t>(0x31u + i * 7u);
        frames_ok = frame_by_hand(data[i], got, i == 5u) && frames_ok;
    }
    uint32_t spins = 400'000u;
    while (!S::rxne() && spins-- != 0u) {
    }
    (void)S::data();   // the CRC frame's own read-back
    frames_ok = wait_not_busy() && frames_ok;
    const uint16_t tx_crc = S::tx_crc();
    const uint16_t ref = crc_reference(0x0007u, 8, data, 6);
    print(serial, "  CRC8 over six frames sent: TXCRCR ", hex(tx_crc), " (bitwise ", hex(ref), ")", crlf);
    bench.verdict("THE TRANSMIT CRC IS THE ARITHMETIC: TXCRCR over six frames is what a bitwise loop over the "
                  "same polynomial computes",
                  frames_ok && tx_crc == ref);
    (void)S::disable();
    S::reset();

    // EVERY PATH COMPLETES WITH NOTHING ON MISO, the select held high: the
    // pump, the polled loops and the engines each run to their end.
    host_ready();
    fill_pattern(tx_buf, 16, 0x21);
    const uint8_t pumped = host_xfer(CsPin::ref(), nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode0,
                                     SpiClock::div16, SpiDataSize::bits8, false);
    const uint32_t pump_entries = spi_isr_entries;
    const uint8_t polled = host_xfer(CsPin::ref(), nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode3,
                                     SpiClock::div2, SpiDataSize::bits16, true);
    const uint8_t written = host_xfer(CsPin::ref(), cmd_buf, 2, tx_buf, nullptr, 16, SpiMode::mode0,
                                      SpiClock::div2, SpiDataSize::bits8, true);
    dma_ready();
    const uint8_t dma = dma_xfer(CsPin::ref(), nullptr, 0, tx_buf, rx_buf, 64, SpiClock::div4,
                                 SpiDataSize::bits8, false);
    const uint8_t dma_polled = dma_xfer(CsPin::ref(), cmd_buf, 1, tx_buf, rx_buf, 32, SpiClock::div2,
                                        SpiDataSize::bits8, true);
    DmaHost::release();
    host_ready();
    print(serial, "  wireless completions: pump=", pumped, " (", pump_entries, " entries) polled=", polled,
          " written=", written, " dma=", dma, " dma polled=", dma_polled, " cs released=", CsPin::read_out(),
          crlf);
    bench.verdict("the pump, the two polled loops and the engines each run a transaction to completion with "
                  "MISO floating (sixteen entries for sixteen frames), the select released",
                  pumped == spi_ok && pump_entries == 16u && polled == spi_ok && written == spi_ok &&
                      dma == spi_ok && dma_polled == spi_ok && CsPin::read_out());
}

// ===========================================================================
// b - the rate ladder on the wire, the high-speed read, the host's costs
// ===========================================================================

/// THE INSTRUCTION FETCH, measured: the transmit-only loop's own shape over
/// the resource, once in the flash and once in SRAM (`.fast_text`, a plain
/// function's name for it, which the linker script carries in .data and the
/// crt copies with it), 256 and 32 frames at the same rate.
template <int where>
[[gnu::noinline]] void tx_loop_copy(const uint8_t* p, uint32_t n) {
    SpiRegs& r = S::regs();
    do {
        while ((r.STATR & spi_txe) == 0u) {
        }
        r.DATAR = *p++;
    } while (--n != 0u);
    while ((r.STATR & spi_txe) == 0u) {
    }
    while ((r.STATR & spi_bsy) != 0u) {
    }
    (void)r.DATAR;
    (void)r.STATR;
}
[[gnu::section(".fast_text"), gnu::noinline]] void tx_loop_sram(const uint8_t* p, uint32_t n) {
    SpiRegs& r = S::regs();
    do {
        while ((r.STATR & spi_txe) == 0u) {
        }
        r.DATAR = *p++;
    } while (--n != 0u);
    while ((r.STATR & spi_txe) == 0u) {
    }
    while ((r.STATR & spi_bsy) != 0u) {
    }
    (void)r.DATAR;
    (void)r.STATR;
}

/// Cycles a frame (x10) of a loop over 256 and 32 frames at `rate`.
template <typename F>
uint32_t loop_frame_x10(F loop, SpiClock rate) {
    Host::prime(SpiMode::mode0, rate);
    auto best = [&](uint32_t n) {
        uint32_t least = 0xFFFFFFFFu;
        for (uint8_t k = 0; k < 3u; ++k) {
            const uint32_t t0 = cycles_now();
            loop(tx_buf, n);
            const uint32_t took = cycles_now() - t0;
            least = took < least ? took : least;
        }
        return least;
    };
    return (best(256) - best(32)) * 10u / 224u;
}

void print_x10(uint32_t v) { print(serial, v / 10u, ".", v % 10u); }

/// THE HOST'S COSTS for one placement (a plain host and an engined one of
/// it): the engines' fixed cost at /4, the pump's frame at /2 and /16 and
/// its fixed cost, the polled loops' frames at /2, /4, /16, /8 and /64 and
/// the receive loop's fixed cost at /4, a zero-length and a 3-frame polled
/// request - frames x10.
struct PathCosts {
    uint32_t engine_fixed;
    uint32_t pump_div2, pump_div16, pump_fixed;
    uint32_t rd[5], wr[5], rd_fixed;
    uint32_t empty, three;
};

template <typename H, typename D>
PathCosts measure_costs() {
    PathCosts c{};
    dma_ready<D>();
    (void)frame_x1000<D>(SpiClock::div4, SpiMode::mode0, SpiDataSize::bits8, 256, 32, &c.engine_fixed);
    D::release();
    // The pump's frame: a pumped receive of 256 and of 32 frames at /2,
    // where the handler bounds the bus - the difference over 224.
    host_ready<H>();
    auto pump_time = [](uint16_t n, SpiClock rate) {
        uint32_t least = 0xFFFFFFFFu;
        for (uint8_t k = 0; k < 3u; ++k) {
            const uint32_t t0 = cycles_now();
            (void)host_xfer<H>({}, nullptr, 0, tx_buf, rx_buf, n, SpiMode::mode0, rate, SpiDataSize::bits8, false);
            const uint32_t took = cycles_now() - t0;
            least = took < least ? took : least;
        }
        return least;
    };
    const uint32_t pump32_div2 = pump_time(32, SpiClock::div2);
    c.pump_div2 = (pump_time(256, SpiClock::div2) - pump32_div2) * 10u / 224u;
    c.pump_fixed = pump32_div2 - 32u * c.pump_div2 / 10u;
    c.pump_div16 = (pump_time(256, SpiClock::div16) - pump_time(32, SpiClock::div16)) * 10u / 224u;
    // The polled loops: the receive loop and the transmit-only loop, 256
    // frames each against 32.
    auto polled_time = [](uint16_t n, SpiClock rate, bool read) {
        uint32_t least = 0xFFFFFFFFu;
        for (uint8_t k = 0; k < 3u; ++k) {
            const uint32_t t0 = cycles_now();
            (void)host_xfer<H>({}, nullptr, 0, tx_buf, read ? rx_buf : nullptr, n, SpiMode::mode0, rate,
                               SpiDataSize::bits8, true);
            const uint32_t took = cycles_now() - t0;
            least = took < least ? took : least;
        }
        return least;
    };
    const SpiClock rates[5] = {SpiClock::div2, SpiClock::div4, SpiClock::div16, SpiClock::div8, SpiClock::div64};
    for (uint8_t i = 0; i < 5u; ++i) {
        const uint32_t rd32 = polled_time(32, rates[i], true);
        c.rd[i] = (polled_time(256, rates[i], true) - rd32) * 10u / 224u;
        c.wr[i] = (polled_time(256, rates[i], false) - polled_time(32, rates[i], false)) * 10u / 224u;
        if (i == 1u) {
            c.rd_fixed = rd32 - 32u * c.rd[i] / 10u;
        }
    }
    // THE INSTRUMENT AND A SMALL REQUEST: a zero-length request through the
    // same wrapper (start() answers at once - the wrapper and the stopwatch
    // alone), and a 3-frame polled request (a command and two data frames,
    // D/C unscripted, the rate unchanged) at /16 against its wire's 384.
    auto small_time = [](uint16_t len, uint8_t cmd_len) {
        uint32_t least = 0xFFFFFFFFu;
        for (uint8_t k = 0; k < 5u; ++k) {
            const uint32_t t0 = cycles_now();
            (void)host_xfer<H>({}, cmd_buf, cmd_len, tx_buf, rx_buf, len, SpiMode::mode0, SpiClock::div16,
                               SpiDataSize::bits8, true);
            const uint32_t took = cycles_now() - t0;
            least = took < least ? took : least;
        }
        return least;
    };
    (void)small_time(1, 0);   // the rate and mode in force: the compare's unchanged path from here on
    c.empty = small_time(0, 0);
    c.three = small_time(2, 1);
    H::release();
    return c;
}

/// `got` within `pct` percent of `want`.
bool within(uint32_t got, uint32_t want, uint32_t pct) {
    const uint32_t tol = want * pct / 100u;
    return got + tol >= want && got <= want + tol;
}

/// The costs measured against the constants the host states for its
/// placement: the pump's frame within 10 % and the dead bus within 2
/// cycles - the placement's own - and the fixed costs, which carry the
/// wrapper's layout (the host's comment), within 30 %.
template <typename H>
bool costs_match(const PathCosts& c) {
    const uint32_t gap = c.rd[1] > 320u ? (c.rd[1] - 320u + 5u) / 10u : 0u;
    return within(c.engine_fixed, H::engine_fixed_cycles, 30u) &&
           within(c.pump_div2 / 10u, H::pump_frame_cycles, 10u) &&
           within(c.pump_fixed, H::pump_fixed_cycles, 30u) && within(c.rd_fixed, H::polled_fixed_cycles, 30u) &&
           gap + 2u >= H::polled_gap_cycles && gap <= H::polled_gap_cycles + 2u;
}

void tb_ladder() {
    dma_ready();
    fill_pattern(tx_buf, 512, 0x3C);
    console_drain();
    // THE LADDER: a frame inside a 256- and a 32-frame block, every code.
    uint8_t exact = 0;
    print(serial, "  a frame inside an engined block, in thousandths of its SCK periods at the rate asked "
                  "(8 bits):",
          crlf, "   ");
    for (uint8_t code = 0; code < 8u; ++code) {
        const SpiClock rate = static_cast<SpiClock>(code);
        const uint32_t f = frame_x1000(rate, SpiMode::mode0, SpiDataSize::bits8, 256, 32);
        const uint32_t pm = permille_of(f, 8, asked_divisor(rate));
        print(serial, " /", asked_divisor(rate), " ", pm);
        if (wire_ok(pm)) {
            ++exact;
        }
    }
    print(serial, crlf);
    const uint32_t w16_div4 = permille_of(frame_x1000(SpiClock::div4, SpiMode::mode0, SpiDataSize::bits16, 128, 16),
                                          16, 4);
    const uint32_t w16_div16 =
        permille_of(frame_x1000(SpiClock::div16, SpiMode::mode3, SpiDataSize::bits16, 128, 16), 16, 16);
    print(serial, "  16-bit frames: /4 ", w16_div4, ", /16 (mode 3) ", w16_div16, crlf);
    bench.verdict("THE ENGINES RUN AT THE RATE ASKED at every BR code, /2 to /256: a frame inside a block takes "
                  "its SCK periods at HCLK over the divisor, -1 % to +3 %",
                  exact == 8u);
    bench.verdict("and in 16-bit frames at /4 and /16", wire_ok(w16_div4) && wire_ok(w16_div16));

    // THE HIGH-SPEED READ'S RATE at every code, the bit set by hand after
    // the code is in force (the host itself never sets it off /2): 16.3.1
    // says FHCLK/(BR+2), 16.3.8 that the mode is "only valid" at /2.
    print(serial, "  HSRXEN set: a frame in HCLK cycles x1000 at each code (powers of two: 16 x 2^BR; "
                  "16.3.1's FHCLK/(BR+2): 8 x (BR+2)):",
          crlf, "   ");
    uint8_t powers = 0;
    uint8_t plus_two = 0;
    for (uint8_t code = 0; code < 8u; ++code) {
        const SpiClock rate = static_cast<SpiClock>(code);
        DmaHost::prime(SpiMode::mode0, rate);
        S::high_speed_read_raw(true);
        const uint32_t f = frame_x1000(rate, SpiMode::mode0, SpiDataSize::bits8, 256, 32);
        S::high_speed_read_raw(false);
        const uint32_t pow2 = 8u * asked_divisor(rate) * 1000u;
        const uint32_t p2 = 8u * (code + 2u) * 1000u;
        if (f >= pow2 * 99u / 100u && f <= pow2 * 103u / 100u) {
            ++powers;
        }
        if (f >= p2 * 99u / 100u && f <= p2 * 103u / 100u) {
            ++plus_two;
        }
        print(serial, " BR", code, " ", f);
    }
    print(serial, crlf, "  -> ", powers, " of 8 codes at the powers of two, ", plus_two,
          " of 8 at FHCLK/(BR+2)", crlf);
    bench.verdict("the high-speed read's rate was measured at every code (16.3.1 against 16.3.8: a finding "
                  "either way)",
                  powers + plus_two >= 1u);
    bench.verdict("and at /2, where both readings agree, the frame is its 16 cycles with the bit set",
                  [] {
                      DmaHost::prime(SpiMode::mode0, SpiClock::div2);
                      S::high_speed_read_raw(true);
                      const uint32_t f = frame_x1000(SpiClock::div2, SpiMode::mode0, SpiDataSize::bits8, 256, 32);
                      S::high_speed_read_raw(false);
                      return f >= 15'840u && f <= 16'480u;
                  }());

    // THE HOST'S COSTS, both placements: the suite's hosts in SRAM, then
    // their flash twins - the same measurement, the constants each states.
    const PathCosts ram = measure_costs<Host, DmaHost>();
    const PathCosts flash = measure_costs<FlashHost, FlashDmaHost>();
    print(serial, "  THE HOST'S COSTS, SRAM | flash (cycles; x.y a frame):", crlf);
    print(serial, "    the engines' fixed cost at /4 (the 32-frame block less 32 frames): ", ram.engine_fixed, " | ",
          flash.engine_fixed, " (constants ", Host::engine_fixed_cycles, " | ", FlashHost::engine_fixed_cycles, ")",
          crlf);
    print(serial, "    the pump a frame at /2 (the wire 16): ");
    print_x10(ram.pump_div2);
    print(serial, " | ");
    print_x10(flash.pump_div2);
    print(serial, "; at /16 (the wire 128): ");
    print_x10(ram.pump_div16);
    print(serial, " | ");
    print_x10(flash.pump_div16);
    print(serial, " (constants ", Host::pump_frame_cycles, " | ", FlashHost::pump_frame_cycles, ")", crlf);
    print(serial, "    the pump's fixed cost (the 32-frame line less 32 frames): ", ram.pump_fixed, " | ",
          flash.pump_fixed, " (constants ", Host::pump_fixed_cycles, " | ", FlashHost::pump_fixed_cycles, ")",
          crlf);
    print(serial, "    the receive loop a frame /2 /4 /16 (the wire 16, 32, 128): ");
    print_x10(ram.rd[0]);
    print(serial, " ");
    print_x10(ram.rd[1]);
    print(serial, " ");
    print_x10(ram.rd[2]);
    print(serial, " | ");
    print_x10(flash.rd[0]);
    print(serial, " ");
    print_x10(flash.rd[1]);
    print(serial, " ");
    print_x10(flash.rd[2]);
    print(serial, "; /64, two in flight (the wire 512): ");
    print_x10(ram.rd[4]);
    print(serial, " | ");
    print_x10(flash.rd[4]);
    print(serial, crlf, "    its fixed cost at /4: ", ram.rd_fixed, " | ", flash.rd_fixed, " (constants ",
          Host::polled_fixed_cycles, " | ", FlashHost::polled_fixed_cycles, "), its dead bus a frame at /4 ");
    print_x10(ram.rd[1] > 320u ? ram.rd[1] - 320u : 0u);
    print(serial, " | ");
    print_x10(flash.rd[1] > 320u ? flash.rd[1] - 320u : 0u);
    print(serial, " (constants ", Host::polled_gap_cycles, " | ", FlashHost::polled_gap_cycles, ")", crlf);
    print(serial, "    the transmit-only loop a frame /2 /4 /8 /16: ");
    print_x10(ram.wr[0]);
    print(serial, " ");
    print_x10(ram.wr[1]);
    print(serial, " ");
    print_x10(ram.wr[3]);
    print(serial, " ");
    print_x10(ram.wr[2]);
    print(serial, " | ");
    print_x10(flash.wr[0]);
    print(serial, " ");
    print_x10(flash.wr[1]);
    print(serial, " ");
    print_x10(flash.wr[3]);
    print(serial, " ");
    print_x10(flash.wr[2]);
    print(serial, crlf, "    a zero-length request through the suite's wrapper: ", ram.empty, " | ", flash.empty,
          "; a 3-frame polled request at /16: ", ram.three, " | ", flash.three,
          " (the wire's 384 and the wrapper inside it)", crlf);
    bench.verdict("the receive loop keeps two frames in flight from /64 and runs there at the wire (3 %), "
                  "in both placements",
                  ram.rd[4] <= 5274u && flash.rd[4] <= 5274u);
    bench.verdict("THE TRANSMIT-ONLY LOOP IN SRAM is within 5 % of the wire at /2 and /4 and at the wire "
                  "(3 %) from /8",
                  ram.wr[0] <= 168u && ram.wr[1] <= 336u && ram.wr[3] <= 659u && ram.wr[2] <= 1319u);
    // From the flash at /2 the loop's turn fetched behind the wait states
    // is the bound, and where the loop sits decides it (20.0 and 28.0
    // cycles a frame measured in two builds): printed, not judged.
    bench.verdict("and from the flash within 5 % of the wire at /4 and at the wire from /8",
                  flash.wr[1] <= 336u && flash.wr[3] <= 659u && flash.wr[2] <= 1319u);
    bench.verdict("SRAM shortens the pump's frame and the receive loop's at /2",
                  ram.pump_div2 < flash.pump_div2 && ram.rd[0] < flash.rd[0]);
    bench.verdict("THE CONSTANTS FOLLOW THE PLACEMENT: the pump's frame within 10 % of the one its host "
                  "states, the dead bus within 2 cycles, each fixed cost within 30 %",
                  costs_match<Host>(ram) && costs_match<FlashHost>(flash));
    host_ready();
    const uint32_t flash2 = loop_frame_x10(tx_loop_copy<0>, SpiClock::div2);
    const uint32_t sram2 = loop_frame_x10(tx_loop_sram, SpiClock::div2);
    const uint32_t flash4 = loop_frame_x10(tx_loop_copy<0>, SpiClock::div4);
    const uint32_t sram4 = loop_frame_x10(tx_loop_sram, SpiClock::div4);
    print(serial, "  THE FETCH: the bare transmit loop a frame, from the flash /2 ", flash2 / 10u, ".", flash2 % 10u,
          " /4 ", flash4 / 10u, ".", flash4 % 10u, "; from SRAM /2 ", sram2 / 10u, ".", sram2 % 10u, " /4 ",
          sram4 / 10u, ".", sram4 % 10u, " cycles", crlf);

    // THE HOLD-OFF THIS IMAGE MEASURES: the longest body of the console's
    // and the tick's handlers so far, stamped on the STK.
    print(serial, "  this image's handlers, the longest body: console ", console_body_max, " cycles, tick ",
          tick_body_max, ", the SPI pump ", spi_body_max, " (the host's default hold-off ",
          spi_default_hold_off_cycles, ")", crlf);
    bench.verdict("the image's longest handler body is under the host's default hold-off",
                  console_body_max < spi_default_hold_off_cycles && tick_body_max < spi_default_hold_off_cycles);

    // THE PUMP WITH TWO FRAMES IN FLIGHT under the console's interrupts: a
    // line printed and NOT drained, then a pumped receive of 256 frames at
    // the fastest code the default keeps two in flight at.
    const SpiClock two_from = Host::write_ahead_from(SpiDataSize::bits8);
    print(serial, "  two frames in flight from /", asked_divisor(two_from), ": 256 frames pumped under a console "
                  "line in flight ...",
          crlf);
    const uint32_t t0 = cycles_now();
    const uint8_t st = host_xfer({}, nullptr, 0, tx_buf, rx_buf, 256, SpiMode::mode0, two_from, SpiDataSize::bits8,
                                 false);
    const uint32_t took = cycles_now() - t0;
    const uint32_t wire = 256u * 8u * asked_divisor(two_from);
    console_drain();
    print(serial, "  ... status ", st, ", ", took, " cycles against the wire's ", wire, crlf);
    bench.verdict("the pump keeps two in flight with the console's handler live: spi_ok, and within 3 % of the "
                  "wire",
                  st == spi_ok && took <= wire + wire / 33u + 2000u);
    host_ready();
}

// ===========================================================================
// c - the refusals
// ===========================================================================

void tc_refusals() {
    bench.verdict("the chapter's refusals: CRC in half duplex, a zero polynomial, a client with SSOE, the "
                  "high-speed read off /2 or on a client",
                  !spi_config_valid({.direction = SpiDirection::half_duplex_out, .crc = true}) &&
                      !spi_config_valid({.crc = true, .crc_polynomial = 0}) &&
                      !spi_config_valid({.role = SpiRole::client, .nss = SpiNss::hardware_output}) &&
                      !spi_config_valid({.clock = SpiClock::div4, .high_speed_read = true}) &&
                      !spi_config_valid({.role = SpiRole::client, .clock = SpiClock::div2, .high_speed_read = true}));
    bench.verdict("configure() writes nothing it refuses (CTLR1 unchanged)", [] {
        S::bus_clock(true);
        S::reset();
        (void)S::configure({.role = SpiRole::host, .clock = SpiClock::div8});
        const uint16_t before = S::regs().CTLR1;
        const bool refused = !S::configure({.crc = true, .crc_polynomial = 0});
        return refused && S::regs().CTLR1 == before;
    }());
    bench.verdict("a ceiling under HCLK/256 is refused by init()", !Host::init(clock, 100'000u));
    host_ready();
    bench.verdict("a ceiling is honoured: 6 MHz asks /8, and a request at /2 runs at /8",
                  [] {
                      const bool ok = Host::init(clock, 6'000'000u);
                      const bool c = Host::ceiling_clock() == SpiClock::div8 &&
                                     Host::sck_hz(SpiClock::div2) == 6'000'000u;
                      return ok && c;
                  }());
    host_ready();
    uint8_t refused = 0;
    uint8_t served = 0;
    for (uint8_t m = 0; m < 4u; ++m) {
        for (uint8_t o = 0; o < 2u; ++o) {
            const Client::Config c{.mode = static_cast<SpiMode>(m), .lsb_first = o != 0u};
            if (Client::config_served(c)) {
                ++served;
            } else {
                ++refused;
            }
        }
    }
    print(serial, "  the client answering: ", served, " of 8 mode and order combinations served, ", refused,
          " refused (letter r's measurement); a listener served in all", crlf);
    bench.verdict("a client that only LISTENS is served in every mode and order (16.2.3)", [] {
        for (uint8_t m = 0; m < 4u; ++m) {
            for (uint8_t o = 0; o < 2u; ++o) {
                if (!Client::config_served({.mode = static_cast<SpiMode>(m), .lsb_first = o != 0u,
                                            .drive_output = false})) {
                    return false;
                }
            }
        }
        return true;
    }());
    bench.verdict("SpiClient::init() refuses what config_served() refuses, and the block is left untouched",
                  [] {
                      for (uint8_t m = 0; m < 4u; ++m) {
                          for (uint8_t o = 0; o < 2u; ++o) {
                              const Client::Config c{.mode = static_cast<SpiMode>(m), .lsb_first = o != 0u};
                              if (!Client::config_served(c) && Client::init(clock, c)) {
                                  return false;
                              }
                          }
                      }
                      return true;
                  }());
    host_ready();
}

// ===========================================================================
// n - the peer's command channel
// ===========================================================================

void tn_peer_link() {
    if (!need_peer()) {
        return;
    }
    bench.verdict("the peer answers a ping over the four wires", true);
    spilink::Ident d{};
    const bool got = peer_ident(d);
    if (got) {
        print(serial, "  peer: label '");
        for (uint8_t i = 0; i < 8 && d.label[i]; ++i) {
            print(serial, d.label[i]);
        }
        print(serial, "' xtal=", d.xtal, " sanity=", hex(d.sanity), " fw=", hex(d.version), crlf);
    } else {
        print(serial, "  the ident window carried:");
        for (uint8_t i = 0; i < raw_n; ++i) {
            print(serial, " ", hex(raw_seen[i]));
        }
        print(serial, crlf);
    }
    bench.verdict("ident comes back and it IS spi_peer (the sanity byte), the CH32V006's (kind 0x06)",
                  got && d.sanity == spilink::ident_sanity && spilink::peer_kind(d.version) == spilink::peer_ch32v00x);
    uint8_t pings = 0;
    for (uint8_t i = 0; i < 10; ++i) {
        if (command(Op::ping)) {
            ++pings;
        }
    }
    print(serial, "  ", pings, " of 10 pings answered at ", Host::sck_hz(link_clock) / 1000u,
          " kHz SCK, one frame per chip-select window, the answers clocked in mode ",
          static_cast<uint8_t>(answer_mode), crlf);
    bench.verdict("the channel is steady over ten frames", pings == 10u);
}

// ===========================================================================
// o - the matrix against the peer
// ===========================================================================

void to_peer_matrix() {
    if (!need_peer()) {
        return;
    }
    // EVERY MODE AND ORDER, on the peer's DMA serve and on its polled run:
    // what this host SENDS judged by the foreign client's report, what it
    // READS judged here against the client's stream. The two directions are
    // two facts: a receiving client need only match the polarity (16.2.3),
    // an answering one is the chapter's restricted case.
    uint8_t heard = 0, read_ok = 0, read_m3 = 0;
    for (uint8_t m = 0; m < 4; ++m) {
        for (uint8_t o = 0; o < 2; ++o) {
            for (uint8_t serve = 0; serve < 2; ++serve) {
                Exchange e{};
                e.cfg.mode = m;
                e.cfg.dord = o;
                e.host_lsb = o != 0;
                e.host_mode = static_cast<SpiMode>(m);
                e.spare = serve != 0u ? spilink::spare_polled_pump : 0;
                e.seed_a = static_cast<uint8_t>(0x13u + m + 4u * o + 16u * serve);
                e.seed_b = static_cast<uint8_t>(0x57u + m + 4u * o + 16u * serve);
                const bool ran = do_exchange(e);
                spilink::Report r{};
                const bool rep = ran && peer_report(r);
                const Verify v = verify_rx(e);
                const bool to_client = rep && r.count == e.count && r.mism == 0u;
                const bool to_host = ran && v.mism == 0u;
                print(serial, "  mode ", m, o != 0u ? " LSb" : " MSb", serve != 0u ? " polled" : " dma   ",
                      ": the client heard ", r.count, "/8 mism ", r.mism);
                if (r.mism != 0u) {
                    print(serial, " (idx ", r.idx, " got ", hex(r.got), " exp ", hex(r.exp), ")");
                }
                print(serial, ", this host read mism ", v.mism);
                if (v.mism != 0u) {
                    print(serial, " (idx ", v.idx, " got ", hex(v.got), " exp ", hex(v.exp), ")");
                }
                print(serial, crlf);
                if (m == 0u || m == 3u) {
                    heard = static_cast<uint8_t>(heard + (to_client ? 1u : 0u));
                    read_ok = static_cast<uint8_t>(read_ok + (to_host ? 1u : 0u));
                }
                if (m == 3u) {
                    read_m3 = static_cast<uint8_t>(read_m3 + (to_host ? 1u : 0u));
                }
            }
        }
    }
    print(serial, "  -> modes 0 and 3: the client heard 8 frames exact in ", heard, " of 8 runs, this host read its "
                  "answers exact in ", read_ok, " of 8 (", read_m3, " of 4 in mode 3); modes 1 and 2 printed - the "
                  "CH32V006 client is not served there either way (letter w's waveforms: this host's MOSI is the "
                  "standard's in every mode)",
          crlf);
    bench.verdict("THIS HOST'S FRAMES reach a FOREIGN client exact in modes 0 and 3, both bit orders, both of its "
                  "serves - the modes the CH32V006 client is served in",
                  heard == 8u);
    bench.verdict("and this host READS the foreign client's answers exact there", read_ok == 8u);

    Exchange mism{};
    mism.cfg.mode = 3;
    mism.host_mode = SpiMode::mode3;
    mism.cfg.dord = 1;      // the client LSb first
    mism.host_lsb = false;  // this end MSb first
    Verify v2{};
    spilink::Report r2{};
    const bool ran = do_exchange(mism);
    const bool rep = ran && peer_report(r2);
    v2 = verify_rx(mism);
    print(serial, "  DORD mismatch: host mism=", v2.mism, " client mism=", r2.mism, " count=", r2.count, crlf);
    bench.verdict("a bit-order mismatch is an EXACT TWO-WAY BIT REVERSAL", ran && rep && v2.mism == 0u &&
                                                                            r2.mism == 0u && r2.count == mism.count);

    // THE RECEIVE CRC over a foreign chip's frames: the peer serves eight
    // frames of its stream; this end clocks them by hand with CRCEN on and
    // reads both calculators with BSY clear - no CRCNEXT (the peer counts
    // eight frames and no ninth).
    spilink::Params a{};
    a.cfg = spilink::Cfg{.apply = 1, .mode = 3, .dord = 0, .regime = spilink::regime_buffer_wait};
    a.count = 8;
    a.ms = 400;
    a.seed_a = 0x44;
    a.seed_b = 0x9A;
    a.pattern = spilink::pattern_prbs;
    bool crc_ok = false;
    if (peer_act(Op::exchange, a)) {
        settle();
        owner = Owner::none;
        Host::release();
        S::bus_clock(true);
        S::reset();
        (void)S::configure({.role = SpiRole::host, .mode = SpiMode::mode3, .clock = link_clock,
                            .bits = SpiDataSize::bits8, .crc = true, .crc_polynomial = 0x0007u});
        SckPin::function();
        MosiPin::function();
        MisoPin::input();
        S::enable();
        S::flush_rx();
        (void)delay_us(clock, 20);   // SCK at its new idle level before the select
        uint8_t sent[8], got[8];
        spilink::Stream out(a.pattern, a.seed_a);
        CsPin::clear();
        link_hold();
        bool frames = true;
        for (uint8_t i = 0; i < 8u; ++i) {
            sent[i] = out.next();
            frames = frame_by_hand(sent[i], got[i], false) && frames;
            (void)delay_us(clock, spilink::gap_us);
        }
        frames = wait_not_busy() && frames;
        link_hold();
        CsPin::set();
        const uint16_t rx_crc = S::rx_crc();
        const uint16_t tx_crc = S::tx_crc();
        uint8_t want[8];
        for (uint8_t i = 0; i < 8u; ++i) {
            want[i] = spilink::pattern_value(a.pattern, a.seed_b, i);
        }
        const uint16_t rx_ref = crc_reference(0x0007u, 8, got, 8);
        const uint16_t tx_ref = crc_reference(0x0007u, 8, sent, 8);
        (void)S::disable();
        S::reset();
        print(serial, "  CRC8 over the peer's eight frames: RXCRCR ", hex(rx_crc), " (bitwise over what came ",
              hex(rx_ref), "), TXCRCR ", hex(tx_crc), " (", hex(tx_ref), "), the frames ",
              same(got, want, 8) ? "the peer's stream" : "NOT the peer's stream", crlf);
        crc_ok = frames && same(got, want, 8) && rx_crc == rx_ref && tx_crc == tx_ref;
    }
    (void)link_command_mode();
    bench.verdict("THE RECEIVE CRC IS THE ARITHMETIC over a foreign chip's frames: RXCRCR is the bitwise CRC of "
                  "the peer's stream, TXCRCR of this end's",
                  crc_ok);
    bench.verdict("and the command channel still works after all of that", command(Op::ping));
}

// ===========================================================================
// p - the rate ladder against the peer
// ===========================================================================

void tp_peer_rates() {
    if (!need_peer()) {
        return;
    }
    // The BR codes, from the command rate up, on the polled host (8 frames)
    // and on the engines (32), against the peer's DMA serve and its polled
    // run - every rung printed, the verdicts on the rungs table 3-19's 24
    // MHz admits for both roles.
    struct Rung {
        bool polled_host;
        bool polled_peer;
    };
    const Rung serves[4] = {{true, false}, {false, false}, {true, true}, {false, true}};
    const char* names[4] = {"polled host / peer DMA", "engined host / peer DMA", "polled host / peer polled",
                            "engined host / peer polled"};
    uint8_t exact_to[4] = {8, 8, 8, 8};   // the slowest code index that broke, 8 = none
    for (uint8_t s = 0; s < 4u; ++s) {
        print(serial, "  ", names[s], ":");
        for (uint8_t code = 8; code-- > 0;) {
            Exchange e{};
            e.rate = static_cast<SpiClock>(code);
            e.count = serves[s].polled_host ? 8 : 32;
            e.engines = !serves[s].polled_host;
            e.spare = serves[s].polled_peer ? spilink::spare_polled_pump : 0;
            e.seed_a = static_cast<uint8_t>(0x21 + code);
            e.seed_b = static_cast<uint8_t>(0x84 + code);
            Verify v{};
            spilink::Report r{};
            const bool ok = exchange_exact(e, v, r);
            print(serial, " /", asked_divisor(e.rate), ok ? " ok" : " BAD");
            if (!ok) {
                print(serial, "(h", v.mism, " c", r.mism, "/", r.count, ")");
                if (exact_to[s] == 8u) {
                    exact_to[s] = code;
                }
            }
        }
        print(serial, crlf);
    }
    // /2 = 24 MHz with the high-speed read, both host paths, the peer's DMA.
    Exchange hs{};
    hs.rate = SpiClock::div2;
    hs.count = 32;
    hs.engines = true;
    hs.high_speed = true;
    hs.seed_a = 0x5D;
    hs.seed_b = 0xC3;
    Verify hv{};
    spilink::Report hr{};
    const bool hs_ok = exchange_exact(hs, hv, hr);
    hs.high_speed = false;
    Verify nv{};
    spilink::Report nr{};
    const bool nhs_ok = exchange_exact(hs, nv, nr);
    print(serial, "  /2 = 24 MHz on the engines, peer DMA: with HSRXEN ", hs_ok ? "exact" : "NOT exact", " (host mism ",
          hv.mism, ", client ", hr.mism, "/", hr.count, "), without ", nhs_ok ? "exact" : "NOT exact", " (host mism ",
          nv.mism, ", client ", nr.mism, "/", nr.count, ")", crlf);
    if (!nhs_ok) {
        dump_exchange(hs, nv, nr);
    }
    // HSRXEN's own ladder (letter b: HCLK/(BR+2) at every code), BR 1..4 =
    // /3, /4, /5, /6, against the peer's DMA serve: what a finer ladder
    // would give a device whose limit falls between two powers of two.
    print(serial, "  HSRXEN's own ladder on the engines, peer DMA:");
    for (uint8_t code = 1; code <= 4u; ++code) {
        Exchange l{};
        l.rate = static_cast<SpiClock>(code);
        l.count = 32;
        l.engines = true;
        l.raw_hsrxen = true;
        l.seed_a = static_cast<uint8_t>(0x60 + code);
        l.seed_b = static_cast<uint8_t>(0x90 + code);
        Verify lv{};
        spilink::Report lr{};
        const bool ok = exchange_exact(l, lv, lr);
        print(serial, " /", code + 2u, ok ? " ok" : " BAD");
        if (!ok) {
            print(serial, "(h", lv.mism, " c", lr.mism, "/", lr.count, ")");
        }
    }
    print(serial, crlf);
    // The verdicts: both host paths to the top of the ladder against the
    // peer's DMA serve; the peer's polled run is ITS ceiling (the
    // CH32V006's exchange run from its flash, letter k), judged to /8.
    const auto held_to_div8 = [](uint8_t broke) { return broke == 8u || broke < 2u; };
    bench.verdict("BOTH HOST PATHS, the polled loop and the engines, are exact against the peer's DMA serve at every "
                  "code to /2 = 24 MHz (table 3-19's ceiling for both roles)",
                  exact_to[0] == 8u && exact_to[1] == 8u);
    bench.verdict("and against the peer's polled run to /8 = 6 MHz, the run's own ceiling printed above",
                  held_to_div8(exact_to[2]) && held_to_div8(exact_to[3]));
    bench.verdict("/2 = 24 MHz with the high-speed read on is exact too", hs_ok);
    (void)link_command_mode();
    bench.verdict("the command channel survives the climb", command(Op::ping));
}

// ===========================================================================
// k - THE CLIENT'S RUNS: the CH32V006's polled runs, both widths, both hosts
// ===========================================================================

/// The SCK of a BR code at this end's 48 MHz, in kHz.
uint32_t sck_khz(SpiClock rate) { return 48'000u / asked_divisor(rate); }

void tk_peer_runs() {
    if (!need_peer()) {
        return;
    }
    // The far client's three runs (its driver's SpiClient::answer, exchange
    // and receive, asked by spare_polled_pump), 32 frames of 8 bits and of
    // 16, under this end's two hosts: the POLLED loop, one frame in flight
    // and a gap after every frame - the control - and the ENGINES, frames
    // BACK TO BACK, the run's own window. Every rung from /64 to /2 printed;
    // the verdicts within the pair's 6 MHz ceiling (/8), the rungs above it
    // printed alone. An exchange is judged both ways, an answer run by what
    // this end read (the client counts the answers it wrote), a receive run
    // by the client's report (its MISO dark).
    struct Kind {
        const char* name;
        uint8_t spare;
        bool host_judges;
        bool client_judges;
    };
    const Kind kinds[3] = {
        {"exchange", spilink::spare_polled_pump, true, true},
        {"answer  ", static_cast<uint8_t>(spilink::spare_polled_pump | spilink::spare_answer_only), true, false},
        {"receive ", static_cast<uint8_t>(spilink::spare_polled_pump | spilink::spare_receive_only), false, true},
    };
    constexpr uint8_t slowest = static_cast<uint8_t>(SpiClock::div64);
    constexpr uint8_t judged_to = static_cast<uint8_t>(SpiClock::div8);   // 6 MHz, the pair's ceiling
    // exact_to[kind][wide][back_to_back]: the fastest code at which that run
    // and every slower rung were exact (slowest + 1 = not even /64).
    uint8_t exact_to[3][2][2] = {};
    bool extended = true;
    for (uint8_t w = 0; w < 2u; ++w) {
        for (uint8_t b = 0; b < 2u; ++b) {
            for (uint8_t k = 0; k < 3u; ++k) {
                print(serial, "  ", kinds[k].name, w != 0u ? " 16-bit" : "  8-bit",
                      b != 0u ? " back to back:" : " gapped      :");
                uint8_t fastest = slowest + 1u;
                bool broken = false;
                for (uint8_t code = slowest + 1u; code-- > 0;) {
                    Exchange e{};
                    e.rate = static_cast<SpiClock>(code);
                    e.count = 32;
                    e.wide = w != 0u;
                    e.engines = b != 0u;
                    e.spare = kinds[k].spare;
                    e.seed_a = static_cast<uint8_t>(0x31u + code + 8u * k + 32u * w);
                    e.seed_b = static_cast<uint8_t>(0xA4u + code + 8u * k + 32u * w);
                    const bool ran = do_exchange(e);
                    spilink::Report r{};
                    const bool rep = ran && peer_report(r);
                    const Verify v = verify_rx(e);
                    if (e.wide && rep && (r.flags & spilink::report_extended) == 0u) {
                        extended = false;
                    }
                    const bool host_ok = !kinds[k].host_judges || v.mism == 0u;
                    const bool client_ok = !kinds[k].client_judges || r.mism == 0u;
                    const bool ok = rep && host_ok && client_ok && r.count == e.count;
                    print(serial, " /", asked_divisor(e.rate), ok ? " ok" : " BAD");
                    if (!ok) {
                        // The mismatches each end counted, and each end's
                        // first wrong byte: its index, what came, what was due.
                        print(serial, "(h", v.mism, " c", r.mism, "/", r.count);
                        if (v.mism != 0u) {
                            print(serial, " h@", v.idx, " ", hex(v.got), "/", hex(v.exp));
                        }
                        if (r.mism != 0u) {
                            print(serial, " c@", r.idx, " ", hex(r.got), "/", hex(r.exp));
                        }
                        print(serial, ")");
                        broken = true;
                    } else if (!broken) {
                        fastest = code;
                    }
                }
                exact_to[k][w][b] = fastest;
                print(serial, crlf, "      -> exact to ");
                if (fastest > slowest) {
                    print(serial, "nothing", crlf);
                } else {
                    print(serial, "/", asked_divisor(static_cast<SpiClock>(fastest)), " = ",
                          sck_khz(static_cast<SpiClock>(fastest)), " kHz, a frame ",
                          asked_divisor(static_cast<SpiClock>(fastest)) * (w != 0u ? 16u : 8u),
                          " cycles of this end's 48 MHz on the wire", crlf);
                }
            }
        }
    }
    const auto held = [&](uint8_t k, uint8_t w, uint8_t b) { return exact_to[k][w][b] <= judged_to; };
    bench.verdict("the peer serves 16-bit frames (spare_wide, report_extended)", extended);
    bool gapped = true, back = true;
    for (uint8_t k = 0; k < 3u; ++k) {
        for (uint8_t w = 0; w < 2u; ++w) {
            gapped = gapped && held(k, w, 0);
            back = back && held(k, w, 1);
        }
    }
    bench.verdict("THE CLIENT'S RUNS under the gapped host: answer, exchange and receive exact at every rung to "
                  "/8 = 6 MHz, 8- and 16-bit frames",
                  gapped);
    bench.verdict("and BACK TO BACK under the engines, the run's own window, the same", back);
    (void)link_command_mode();
    bench.verdict("the command channel survives the climb", command(Op::ping));
}

// ===========================================================================
// q - THE KERNEL against the peer
// ===========================================================================

namespace kl {

using SpiArb = SpiBus<Host, P, 4>;

class Probe {
public:
    using Event = std::variant<SpiDone, SleepVote>;
    static inline EventQueue<Event, 12, P> queue;
    static inline uint8_t replies[8];
    static inline uint8_t n = 0;

    static void init() { n = 0; }
    static void dispatch(const Event& e) {
        brio::match(
            e,
            [](const SpiDone& d) {
                if (n < 8u) {
                    replies[n] = d.status;
                }
                ++n;
            },
            [](const SleepVote&) {});
    }
};

using BusKernel = Tenuto<P, Probe, SpiArb>;

void pump() {
    TimeEvents<P>::process();
    while (BusKernel::step()) {
        TimeEvents<P>::process();
    }
}

void pump_until(uint8_t want, uint32_t ms) {
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < ms) {
        pump();
        if (Probe::n >= want) {
            break;
        }
    }
    pump();
}

Host::Request request(const uint8_t* tx, uint8_t* rx, bool polled) {
    Host::Request r{};
    r.cs = {};   // the letter holds the select across the four
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
    r.cmd_len = 0;
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = 8;
    r.mode = SpiMode::mode3;
    r.clock = link_clock;
    r.polled = polled;
    r.reply = reply_to<Probe, SpiDone>();
    return r;
}

}  // namespace kl

void tq_peer_kernel() {
    if (!need_peer()) {
        return;
    }
    constexpr uint16_t total = 4u * 8u;
    spilink::Params a{};
    a.cfg = spilink::Cfg{.apply = 1, .mode = 3, .dord = 0, .regime = spilink::regime_buffer_wait};
    a.count = total;
    a.ms = 800;
    a.seed_a = 0x3B;
    a.seed_b = 0x6D;
    a.pattern = spilink::pattern_prbs;
    if (!peer_act(Op::exchange, a)) {
        bench.verdict("the peer accepted the exchange the kernel letter clocks", false);
        return;
    }
    bench.verdict("the peer accepted the exchange the kernel letter clocks", true);
    static uint8_t qtx[4][8];
    static uint8_t qrx[4][8];
    {
        spilink::Stream out(a.pattern, a.seed_a);
        for (uint8_t i = 0; i < 4u; ++i) {
            for (uint8_t k = 0; k < 8u; ++k) {
                qtx[i][k] = out.next();
                qrx[i][k] = 0xEE;
            }
        }
    }
    kl::BusKernel::init_all();
    owner = Owner::bus_ao;
    settle();
    Host::prime(SpiMode::mode3, link_clock);
    (void)delay_us(clock, 20);
    CsPin::clear();
    link_hold();
    for (uint8_t i = 0; i < 4u; ++i) {
        post<kl::SpiArb>(kl::request(qtx[i], qrx[i], i == 2u));
    }
    kl::pump_until(4, 800);
    link_hold();
    CsPin::set();
    owner = Owner::host;
    uint16_t mism = 0;
    {
        spilink::Stream want(a.pattern, a.seed_b);
        for (uint8_t i = 0; i < 4u; ++i) {
            for (uint8_t k = 0; k < 8u; ++k) {
                if (qrx[i][k] != want.next()) {
                    ++mism;
                }
            }
        }
    }
    const bool replies_ok = kl::Probe::n == 4u && kl::Probe::replies[0] == spi_ok &&
                            kl::Probe::replies[1] == spi_ok && kl::Probe::replies[2] == spi_ok &&
                            kl::Probe::replies[3] == spi_ok;
    spilink::Report r{};
    const bool rep = peer_report(r);
    print(serial, "  kernel: ", kl::Probe::n, " replies; host mism=", mism, "; the peer read ", r.count, " of ", total,
          " with mism=", r.mism, " flags=", hex(r.flags), crlf);
    bench.verdict("FOUR TRANSACTIONS QUEUED FROM ONE DISPATCH (one polled among three pumped) come back in order, "
                  "every one spi_ok, and the bytes on the wire are what each request lent - read back by A SECOND "
                  "CHIP, util/spi_bus.hpp and kernel/ unchanged",
                  replies_ok && mism == 0u && rep && r.count == total && r.mism == 0u);
    (void)link_command_mode();
    bench.verdict("the command channel is back after the kernel's window", command(Op::ping));
}

// ===========================================================================
// r - THE ROLES INVERT: this chip as the CLIENT, the peer as the host
// ===========================================================================

uint8_t crx[64];
uint8_t ctx[64];

struct ClientRun {
    bool ran = false;
    uint16_t got = 0;
    uint16_t mism = 0;
    uint8_t idx = 0xFF, read = 0, want = 0;
    spilink::Report peer{};
};

/// One burst with this chip as the client in `mode` and `lsb`, the
/// RESOURCE configured by hand (the measurement includes what SpiClient
/// refuses): the NSS pad PA4 the hardware select, MISO driven, the run's
/// first answer preloaded with the select high.
ClientRun client_burst(uint8_t mode, bool lsb, uint8_t seed_a, uint8_t seed_b, uint16_t div) {
    ClientRun out{};
    spilink::Params a{};
    a.count = 12;
    a.ms = 500;
    a.seed_a = seed_a;
    a.seed_b = seed_b;
    a.pattern = spilink::pattern_prbs;
    a.aux8 = 40;            // lead-in ms: time for this end to become a client
    a.aux16 = div;          // the peer's SCK division of its own 48 MHz
    a.cfg.apply = 1;
    a.cfg.mode = mode;
    a.cfg.dord = lsb ? 1 : 0;
    if (!peer_act(Op::host_burst, a)) {
        return out;
    }
    owner = Owner::none;
    Host::release();
    CsPin::input(PinPull::up);
    S::bus_clock(true);
    S::reset();
    (void)S::configure({.role = SpiRole::client, .mode = static_cast<SpiMode>(mode), .bits = SpiDataSize::bits8,
                        .lsb_first = lsb, .nss = SpiNss::hardware_input});
    SckPin::input();
    MosiPin::input();
    MisoPin::function();
    S::flush_rx();
    Pfic::disable(S::irq());
    spilink::Stream answers(a.pattern, a.seed_b);
    for (uint16_t i = 0; i < a.count; ++i) {
        ctx[i] = answers.next();
        crx[i] = 0xEE;
    }
    S::enable();
    const Client::Run run = Client::exchange(std::span<const uint8_t>(ctx, a.count), std::span<uint8_t>(crx, a.count),
                                             SysClock::hz / 8u);
    out.got = run.count;
    out.ran = true;
    // The far host closes its select, then this end lets the pads go.
    const uint32_t t0 = Ticker::millis();
    while (!CsPin::read() && Ticker::millis() - t0 < 100u) {
    }
    (void)S::disable();
    S::reset();
    MisoPin::release();
    for (uint16_t i = 0; i < out.got; ++i) {
        const uint8_t exp = spilink::pattern_value(a.pattern, a.seed_a, i);
        if (crx[i] != exp) {
            if (out.mism == 0) {
                out.idx = static_cast<uint8_t>(i);
                out.read = crx[i];
                out.want = exp;
            }
            ++out.mism;
        }
    }
    settle_ms(20);
    (void)peer_report(out.peer);
    return out;
}

void tr_peer_client() {
    if (!need_peer()) {
        return;
    }
    // EVERY MODE AND ORDER, the peer as host at 1.5 MHz: what this chip
    // RECEIVES (judged here against the peer's stream) and what it ANSWERS
    // (judged by the peer's report of what it read).
    uint8_t received = 0, answered = 0, admitted_ok = 0, admitted = 0, refused_broken = 0, refused = 0;
    for (uint8_t m = 0; m < 4u; ++m) {
        for (uint8_t o = 0; o < 2u; ++o) {
            const ClientRun c = client_burst(m, o != 0u, static_cast<uint8_t>(0x2C + m + 8u * o),
                                             static_cast<uint8_t>(0x71 + m + 8u * o), 32);
            const bool rx_ok = c.ran && c.got == 12u && c.mism == 0u;
            const bool tx_ok = c.ran && c.peer.count == 12u && c.peer.mism == 0u;
            print(serial, "  mode ", m, o != 0u ? " LSb" : " MSb", ": received ", c.got, "/12 mism ", c.mism);
            if (c.mism != 0u) {
                print(serial, " (idx ", c.idx, " got ", hex(c.read), " exp ", hex(c.want), ")");
            }
            print(serial, "; the host read our answers ", c.peer.count, "/12 mism ", c.peer.mism);
            if (c.peer.mism != 0u) {
                print(serial, " (idx ", c.peer.idx, " got ", hex(c.peer.got), " exp ", hex(c.peer.exp), ")");
            }
            print(serial, crlf);
            received = static_cast<uint8_t>(received + (rx_ok ? 1u : 0u));
            answered = static_cast<uint8_t>(answered + (tx_ok ? 1u : 0u));
            if (spi_client_answers(static_cast<SpiMode>(m), o != 0u)) {
                ++admitted;
                admitted_ok = static_cast<uint8_t>(admitted_ok + (rx_ok && tx_ok ? 1u : 0u));
            } else {
                ++refused;
                refused_broken = static_cast<uint8_t>(refused_broken + (tx_ok ? 0u : 1u));
            }
        }
    }
    print(serial, "  -> received exact in ", received, " of 8, answered exact in ", answered, " of 8; the driver "
                  "admits ", admitted, " as an answering client",
          crlf);
    bench.verdict("THIS CHIP AS CLIENT RECEIVES a foreign host's burst exact in every mode and both orders "
                  "(16.2.3: a receiver need only match the polarity)",
                  received == 8u);
    bench.verdict("it ANSWERS exact in every combination SpiClient admits (spi_client_answers)",
                  admitted != 0u && admitted_ok == admitted);
    bench.verdict("and every combination SpiClient refuses was MEASURED broken on the wire - a refusal that "
                  "follows the measurement",
                  refused_broken == refused);
    (void)link_command_mode();
    bench.verdict("the command channel is back after the role swaps", command(Op::ping));
}

// ===========================================================================
// The analyser: this chip's four link pads sampled through INDR (letters
// m and w)
// ===========================================================================

constexpr uint16_t wave_samples = 2048;
uint8_t wave[wave_samples];   ///< bit 0 NSS (PA4), 1 SCK (PA5), 2 MISO (PA6), 3 MOSI (PA7)

[[gnu::noinline]] uint32_t sample_wave(uint16_t n) {
    const uint32_t t0 = cycles_now();
    for (uint16_t i = 0; i < n; ++i) {
        wave[i] = static_cast<uint8_t>(Port<'A'>::in() >> 4);
    }
    return cycles_now() - t0;
}

/// The same, the DMA requests raised at sample `kick`: a block whose
/// first edge must not fall before the first sample.
[[gnu::noinline]] void sample_wave_kick(uint16_t n, uint16_t kick) {
    for (uint16_t i = 0; i < n; ++i) {
        if (i == kick) {
            S::dma_requests(true, true);
        }
        wave[i] = static_cast<uint8_t>(Port<'A'>::in() >> 4);
    }
}

struct WaveEdges {
    uint8_t sck_idle = 0;
    uint16_t sck_edges = 0;
    uint16_t after_rising = 0;
    uint16_t after_falling = 0;
};

/// Every change of the data line `data` (a bit of wave[]) filed under the
/// last SCK edge before it; changes before the first edge, or while NSS is
/// high when `framed`, are not counted.
WaveEdges wave_edges(uint16_t n, uint8_t data, bool framed) {
    WaveEdges e{};
    e.sck_idle = static_cast<uint8_t>((wave[0] >> 1) & 1u);
    uint8_t last = 0;   // 0 none, 1 rising, 2 falling
    for (uint16_t i = 1; i < n; ++i) {
        const uint8_t a = wave[i - 1], b = wave[i];
        if (framed && (b & 1u) != 0u) {
            last = 0;
            continue;
        }
        if (((a ^ b) & 2u) != 0u) {
            last = (b & 2u) != 0u ? 1u : 2u;
            ++e.sck_edges;
        }
        if (((a ^ b) & (1u << data)) != 0u) {
            if (last == 1u) {
                ++e.after_rising;
            } else if (last == 2u) {
                ++e.after_falling;
            }
        }
    }
    return e;
}

/// The data line read at the standard SAMPLING edge of `mode` (rising in
/// modes 0 and 3, falling in 1 and 2) - its level just before the edge -
/// MSb first, the first `n_bytes` bytes of the first framed run, and the
/// data line's level when NSS fell.
struct WaveBytes {
    uint8_t bytes[3] = {};
    uint8_t count = 0;
    uint8_t at_select = 0xFF;
};

WaveBytes wave_decode(uint16_t n, uint8_t data, uint8_t mode, bool framed) {
    WaveBytes w{};
    const bool sample_rising = mode == 0u || mode == 3u;
    uint8_t bit = 0;
    uint8_t acc = 0;
    for (uint16_t i = 1; i < n && w.count < 3u; ++i) {
        const uint8_t a = wave[i - 1], b = wave[i];
        if (framed) {
            if ((a & 1u) != 0u && (b & 1u) == 0u) {
                w.at_select = static_cast<uint8_t>((b >> data) & 1u);
            }
            if ((b & 1u) != 0u) {
                if (w.count != 0u || bit != 0u) {
                    break;   // the first framed run is over
                }
                continue;
            }
        }
        if (((a ^ b) & 2u) == 0u) {
            continue;
        }
        const bool rising = (b & 2u) != 0u;
        if (rising != sample_rising) {
            continue;
        }
        acc = static_cast<uint8_t>((acc << 1) | ((a >> data) & 1u));
        if (++bit == 8u) {
            w.bytes[w.count++] = acc;
            bit = 0;
            acc = 0;
        }
    }
    return w;
}

void print_decoded(const WaveBytes& w) {
    print(serial, " decoded");
    for (uint8_t i = 0; i < w.count; ++i) {
        print(serial, " ", hex(w.bytes[i]));
    }
    if (w.at_select != 0xFFu) {
        print(serial, ", the line ", w.at_select ? "high" : "low", " at the select");
    }
}

/// "rising", "falling", "both" or "none": where the data line changed.
const char* edge_name(const WaveEdges& e) {
    return e.after_rising != 0u && e.after_falling == 0u   ? "RISING"
           : e.after_falling != 0u && e.after_rising == 0u ? "FALLING"
           : e.after_rising != 0u                          ? "BOTH"
                                                           : "none";
}

/// The standard's edge for a mode: falling in 0 and 3, rising in 1 and 2.
bool standard_edge(uint8_t mode, const WaveEdges& e) {
    const bool rising = mode == 1u || mode == 2u;
    return rising ? (e.after_rising != 0u && e.after_falling == 0u)
                  : (e.after_falling != 0u && e.after_rising == 0u);
}

// ===========================================================================
// m - MODF from a second chip; ODEN's NSS on the peer's pull-up
// ===========================================================================

/// One demotion: the peer's ss_pulse into a host whose NSS is a hardware
/// INPUT (SSM and SSOE clear), the pad pulled up (16.2.1, 16.2.7).
struct Demotion {
    bool accepted = false;
    bool at_enable = false;
    bool modf = false;
    uint32_t t_modf = 0;
    uint32_t t_high = 0;
    uint16_t ctlr1 = 0;
    uint16_t st_before = 0;
    uint16_t st_after = 0;
    uint8_t sequences = 0;   ///< clearing sequences until MODF read clear (0 = never in four)
};

Demotion demote_once() {
    Demotion d{};
    spilink::Params a{};
    a.aux8 = 30;      // ms after the ack window before the peer takes the wire
    a.aux16 = 2000;   // us held low
    a.ms = 200;
    d.accepted = peer_act(Op::ss_pulse, a);
    if (!d.accepted) {
        return d;
    }
    owner = Owner::none;
    Host::release();
    S::bus_clock(true);
    S::reset();
    (void)S::configure({.role = SpiRole::host, .clock = link_clock, .nss = SpiNss::hardware_input});
    Host::claim_nss_pad<SpiNss::hardware_input>(true);
    (void)delay_us(clock, 50);
    S::enable();
    d.at_enable = S::mode_fault();
    const uint32_t t0 = Ticker::millis();
    while (!S::mode_fault() && Ticker::millis() - t0 < 300u) {
    }
    d.t_modf = Ticker::millis() - t0;
    d.modf = S::mode_fault();
    d.ctlr1 = S::regs().CTLR1;
    // The pad back high for a whole millisecond: the pulse is over.
    const uint32_t t1 = Ticker::millis();
    uint32_t high_since = t1;
    while (Ticker::millis() - t1 < 100u) {
        if (!CsPin::read()) {
            high_since = Ticker::millis();
        } else if (Ticker::millis() - high_since > 1u) {
            break;
        }
    }
    d.t_high = Ticker::millis() - t0;
    d.st_before = S::status();
    for (uint8_t k = 1; k <= 4u; ++k) {
        S::clear_mode_fault();
        if (k == 1u) {
            d.st_after = S::status();
        }
        if (!S::mode_fault()) {
            d.sequences = k;
            break;
        }
    }
    S::reset();
    Host::claim_nss_pad<SpiNss::hardware_input>(false);
    (void)link_command_mode();
    return d;
}

void tm_peer_modf() {
    if (!need_peer()) {
        return;
    }
    uint8_t accepted = 0, raised = 0, demoted = 0, first = 0, within_two = 0;
    for (uint8_t k = 0; k < 4u; ++k) {
        const Demotion d = demote_once();
        print(serial, "  pulse ", k, ": MODF ", d.at_enable ? "ALREADY SET" : "clear", " at the enable, ",
              d.modf ? "SET" : "never", " ", d.t_modf, " ms later, CTLR1 ", hex(d.ctlr1), ", the pad high again from ",
              d.t_high, " ms; STATR ", hex(d.st_before), " before the clearing sequence, ", hex(d.st_after),
              " after it; cleared by sequence ", d.sequences, crlf);
        accepted = static_cast<uint8_t>(accepted + (d.accepted ? 1u : 0u));
        raised = static_cast<uint8_t>(raised + (d.modf && !d.at_enable && d.t_modf >= 20u ? 1u : 0u));
        demoted = static_cast<uint8_t>(demoted + (d.modf && (d.ctlr1 & (spi_mstr | spi_spe)) == 0u ? 1u : 0u));
        first = static_cast<uint8_t>(first + (d.sequences == 1u ? 1u : 0u));
        within_two = static_cast<uint8_t>(within_two + (d.sequences == 1u || d.sequences == 2u ? 1u : 0u));
    }
    bench.verdict("the peer accepted four ss_pulses", accepted == 4u);
    bench.verdict("MODF comes up when a SECOND CHIP pulls this host's NSS low (16.2.7), and not before - four of four",
                  raised == 4u);
    bench.verdict("and it demotes the host: MSTR and SPE cleared by the hardware, four of four", demoted == 4u);
    print(serial, "  the clearing sequence (a STATR read, a CTLR1 write) took at the first try in ", first, " of 4", crlf);
    bench.verdict("16.2.7's sequence, a STATR read then a CTLR1 write, clears MODF once the pad is back high, four "
                  "of four",
                  first == 4u);

    // ODEN (16.3.2), on a wire whose SCK and MOSI carry no pull-up and whose
    // NSS carries the peer's (about 45 kOhm, its dark listener's): an SSOE
    // host at /256 in mode 0, open drain and then push-pull, one 0x5A
    // frame, the four pads sampled through the frame - then SPE cleared and
    // NSS read.
    (void)link_command_mode();
    const bool linked = command(Op::ping);   // the peer dark again, its PC1 pulled up
    struct OdenRun {
        uint16_t sck_rises = 0;
        bool nss_low_in_frame = false;
        bool nss_after = false;
    };
    auto oden_run = [](bool open_drain) {
        OdenRun o{};
        owner = Owner::none;
        Host::release();
        S::bus_clock(true);
        S::reset();
        (void)S::configure({.role = SpiRole::host, .mode = SpiMode::mode0, .clock = link_clock,
                            .nss = SpiNss::hardware_output, .open_drain = open_drain});
        Host::claim_nss_pad(true);
        SckPin::function();
        MosiPin::function();
        MisoPin::input();
        S::enable();
        (void)delay_us(clock, 20);
        S::data(0x5A);
        (void)sample_wave(wave_samples);
        for (uint16_t i = 1; i < wave_samples; ++i) {
            if ((wave[i] & 1u) == 0u) {
                o.nss_low_in_frame = true;
            }
            if ((wave[i - 1] & 2u) == 0u && (wave[i] & 2u) != 0u) {
                ++o.sck_rises;
            }
        }
        (void)wait_not_busy();
        S::flush_rx();
        (void)S::disable();
        (void)delay_us(clock, 50);
        o.nss_after = CsPin::read();
        S::reset();
        CsPin::output(true);
        return o;
    };
    const OdenRun od = oden_run(true);
    const OdenRun pp = oden_run(false);
    print(serial, "  SSOE, one 0x5A frame at /256: open drain - SCK rose ", od.sck_rises, " times, NSS ",
          od.nss_low_in_frame ? "low" : "NOT low", " in the frame, ", od.nss_after ? "high" : "LOW",
          " after SPE cleared; push-pull - SCK rose ", pp.sck_rises, " times, NSS ",
          pp.nss_low_in_frame ? "low" : "NOT low", ", ", pp.nss_after ? "high" : "LOW", crlf);
    bench.verdict("ODEN's SCK is open drain: with no pull-up on the wire it never rises, where push-pull rises at "
                  "every one of a frame's eight clocks",
                  linked && od.sck_rises == 0u && pp.sck_rises == 8u);
    bench.verdict("and ODEN's NSS is pulled low in the frame and comes back high through the far end's pull-up",
                  od.nss_low_in_frame && od.nss_after);
    (void)link_command_mode();
    bench.verdict("the command channel is back after the demotion", command(Op::ping));
}

// ===========================================================================
// w - THE WAVEFORMS, this chip as the logic analyser of its own four pads
// ===========================================================================
//
// Which end of a broken mode is the one off the standard? Neither the
// chapter nor a loop can say (a block that shifts on the wrong edge reads
// its own frames back exact), so the CPU samples PA4..PA7 through INDR in
// a tight loop while a frame is on the wire at HCLK/256 - some hundred
// samples a bit - and every change of a data line is filed under the last
// SCK edge before it. The standard (Motorola) data line changes on the
// FALLING edges in modes 0 and 3 and on the RISING edges in modes 1 and 2.

void tw_waveforms() {
    // w.1 THIS HOST'S OUTPUTS, the far client deselected (PA4 high): SCK and
    // MOSI of a 0x5A frame, in each mode.
    uint8_t host_std = 0;
    for (uint8_t m = 0; m < 4u; ++m) {
        host_ready();
        Host::prime(static_cast<SpiMode>(m), SpiClock::div256);
        (void)delay_us(clock, 50);
        S::data(0x5A);
        const uint32_t took = sample_wave(wave_samples);
        console_drain();
        S::flush_rx();
        const WaveEdges e = wave_edges(wave_samples, 3, false);
        print(serial, "  w.1 this host, mode ", m, ": SCK idles ", e.sck_idle ? "high" : "low", ", ", e.sck_edges,
              " edges; MOSI changes on ", edge_name(e), " (", e.after_rising, "/", e.after_falling, "), a sample every ",
              took / wave_samples, " cycles;");
        print_decoded(wave_decode(wave_samples, 3, m, false));
        print(serial, " (sent 0x5A)", crlf);
        const WaveBytes d = wave_decode(wave_samples, 3, m, false);
        host_std = static_cast<uint8_t>(host_std + (standard_edge(m, e) && e.sck_idle == (m >= 2u ? 1u : 0u) &&
                                                            d.count != 0u && d.bytes[0] == 0x5Au
                                                        ? 1u
                                                        : 0u));
    }
    bench.verdict("THIS HOST's SCK idles at CPOL, its MOSI changes on the standard edge and reads, at the standard "
                  "sampling edge, the frame it sent - in every mode",
                  host_std == 4u);
    if (!need_peer()) {
        return;
    }
    // w.2 THE FOREIGN CLIENT'S MISO: an exchange on this host's engines,
    // the CPU sampling meanwhile, the peer's DMA serve.
    uint8_t client_std = 0, host_framed = 0, client_ok = 0;
    for (uint8_t m = 0; m < 4u; ++m) {
        spilink::Params a{};
        a.cfg = spilink::Cfg{.apply = 1, .mode = m, .dord = 0, .regime = spilink::regime_buffer_wait};
        a.count = 8;
        a.ms = 400;
        a.seed_a = 0x33;
        a.seed_b = 0x5A;
        a.pattern = spilink::pattern_prbs;
        if (!peer_act(Op::exchange, a)) {
            continue;
        }
        settle();
        // The block by hand on this host's engines, the requests raised from
        // inside the sampling loop so its first edge is sampled.
        owner = Owner::none;
        Host::release();
        S::bus_clock(true);
        S::reset();
        (void)S::configure({.role = SpiRole::host, .mode = static_cast<SpiMode>(m), .clock = SpiClock::div256});
        SckPin::function();
        MosiPin::function();
        MisoPin::input();
        S::enable();
        S::flush_rx();
        CsPin::output(true);
        {
            using HTx = DmaTxEngine<3, uint16_t>;
            using HRx = DmaRxEngine<2, uint16_t>;
            HTx::arm(S::data_address(), 0u, DmaPriority::high);
            HRx::arm(S::data_address(), 0u, DmaPriority::very_high);
            fill_pattern(tx_buf, 8, 0x11);
            (void)HRx::start(std::span<uint8_t>(rx_buf, 8));
            (void)HTx::start(std::span<const uint8_t>(tx_buf, 8));
            (void)delay_us(clock, 30);
            CsPin::clear();
            link_hold();
            sample_wave_kick(wave_samples, 8);
            const uint32_t t0 = Ticker::millis();
            while (!HRx::full() && Ticker::millis() - t0 < 50u) {
                (void)HRx::take();
            }
            link_hold();
            CsPin::set();
            S::dma_requests(false, false);
            HTx::stop();
            HRx::stop();
        }
        (void)link_command_mode();
        const WaveEdges e = wave_edges(wave_samples, 2, true);
        print(serial, "  w.2 the CH32V006 client, mode ", m, ": MISO changes on ", edge_name(e), " (", e.after_rising,
              "/", e.after_falling, ");");
        print_decoded(wave_decode(wave_samples, 2, m, true));
        print(serial, "; MOSI");
        const WaveBytes mo = wave_decode(wave_samples, 3, m, true);
        const WaveBytes mi = wave_decode(wave_samples, 2, m, true);
        print_decoded(mo);
        host_framed = static_cast<uint8_t>(host_framed + (mo.count >= 2u && mo.bytes[0] == tx_buf[0] &&
                                                                  mo.bytes[1] == tx_buf[1]
                                                              ? 1u
                                                              : 0u));
        if (m == 0u || m == 3u) {
            client_ok = static_cast<uint8_t>(client_ok + (mi.count >= 2u &&
                                                                  mi.bytes[0] == spilink::pattern_value(a.pattern, a.seed_b, 0) &&
                                                                  mi.bytes[1] == spilink::pattern_value(a.pattern, a.seed_b, 1)
                                                              ? 1u
                                                              : 0u));
        }
        print(serial, " (owed ", hex(spilink::pattern_value(a.pattern, a.seed_b, 0)), " ",
              hex(spilink::pattern_value(a.pattern, a.seed_b, 1)), ", sent ", hex(tx_buf[0]), " ", hex(tx_buf[1]), ")",
              crlf);
        client_std = static_cast<uint8_t>(client_std + (standard_edge(m, e) ? 1u : 0u));
        spilink::Report rep{};
        (void)peer_report(rep);
    }
    print(serial, "  -> the foreign client's MISO on the standard edge in ", client_std, " of 4 modes", crlf);
    bench.verdict("inside a block on this host's engines its MOSI carries the frames lent, back to back, in every mode",
                  host_framed == 4u);
    bench.verdict("and the CH32V006 client's MISO carries its stream in modes 0 and 3", client_ok == 2u);
    // w.3 THE FOREIGN HOST'S OUTPUTS: the peer's host_burst at 187.5 kHz,
    // every pad of this chip a floating input (it drives nothing).
    uint8_t far_std = 0;
    for (uint8_t m = 0; m < 4u; ++m) {
        spilink::Params a{};
        a.count = 4;
        a.ms = 300;
        a.seed_a = 0x5A;
        a.seed_b = 0x00;
        a.pattern = spilink::pattern_fixed;
        a.aux8 = 30;
        a.aux16 = 256;
        a.cfg.apply = 1;
        a.cfg.mode = m;
        if (!peer_act(Op::host_burst, a)) {
            continue;
        }
        owner = Owner::none;
        Host::release();
        CsPin::input();
        const uint32_t t0 = Ticker::millis();
        while (CsPin::read() && Ticker::millis() - t0 < 200u) {
        }
        (void)sample_wave(wave_samples);
        while (!CsPin::read() && Ticker::millis() - t0 < 400u) {
        }
        settle_ms(20);
        const WaveEdges e = wave_edges(wave_samples, 3, true);
        print(serial, "  w.3 the CH32V006 host, mode ", m, ": SCK idles ", e.sck_idle ? "high" : "low",
              ", MOSI changes on ", edge_name(e), " (", e.after_rising, "/", e.after_falling, ");");
        print_decoded(wave_decode(wave_samples, 3, m, true));
        print(serial, " (sent 0x5A)", crlf);
        far_std = static_cast<uint8_t>(far_std + (standard_edge(m, e) ? 1u : 0u));
        spilink::Report rep{};
        (void)peer_report(rep);
    }
    print(serial, "  -> the foreign host's MOSI on the standard edge in ", far_std, " of 4 modes", crlf);
    // w.4 THIS CHIP AS CLIENT: its MISO under the peer's host_burst, the
    // answers on this chip's engines so the CPU is free to sample.
    using CTx = DmaTxEngine<3, uint16_t>;
    using CRx = DmaRxEngine<2, uint16_t>;
    uint8_t own_std = 0, own_lsb_dead = 0;
    for (uint8_t k = 0; k < 8u; ++k) {
        const uint8_t m = static_cast<uint8_t>(k & 3u);
        const bool lsb = k >= 4u;
        spilink::Params a{};
        a.count = 4;
        a.ms = 300;
        a.seed_a = 0x00;
        a.seed_b = 0x5A;
        a.pattern = spilink::pattern_fixed;
        a.aux8 = 30;
        a.aux16 = 256;
        a.cfg.apply = 1;
        a.cfg.mode = m;
        a.cfg.dord = lsb ? 1 : 0;
        if (!peer_act(Op::host_burst, a)) {
            continue;
        }
        owner = Owner::none;
        Host::release();
        CsPin::input(PinPull::up);
        S::bus_clock(true);
        S::reset();
        (void)S::configure({.role = SpiRole::client, .mode = static_cast<SpiMode>(m), .lsb_first = lsb,
                            .nss = SpiNss::hardware_input});
        SckPin::input();
        MosiPin::input();
        MisoPin::function();
        CTx::arm(S::data_address(), 0u, DmaPriority::high);
        CRx::arm(S::data_address(), 0u, DmaPriority::very_high);
        for (uint8_t i = 0; i < 4u; ++i) {
            ctx[i] = 0x5A;
        }
        (void)CRx::start(std::span<uint8_t>(crx, 4));
        S::dma_requests(false, true);
        (void)CTx::start(std::span<const uint8_t>(ctx, 4));
        S::dma_requests(true, true);
        S::enable();
        const uint32_t t0 = Ticker::millis();
        while (CsPin::read() && Ticker::millis() - t0 < 200u) {
        }
        (void)sample_wave(wave_samples);
        while (!CsPin::read() && Ticker::millis() - t0 < 400u) {
        }
        S::dma_requests(false, false);
        CTx::stop();
        CRx::stop();
        (void)S::disable();
        S::reset();
        MisoPin::release();
        settle_ms(20);
        const WaveEdges e = wave_edges(wave_samples, 2, true);
        print(serial, "  w.4 this chip as client, mode ", m, lsb ? " LSb" : " MSb", ": MISO changes on ", edge_name(e),
              " (", e.after_rising, "/", e.after_falling, ");");
        print_decoded(wave_decode(wave_samples, 2, m, true));
        print(serial, " (owed 0x5A), received ", hex(crx[0]), " ", hex(crx[1]), crlf);
        const WaveBytes d = wave_decode(wave_samples, 2, m, true);
        if (lsb) {
            own_lsb_dead = static_cast<uint8_t>(own_lsb_dead + (e.after_rising == 0u && e.after_falling == 0u ? 1u : 0u));
        } else {
            own_std = static_cast<uint8_t>(own_std + (standard_edge(m, e) && d.count != 0u && d.bytes[0] == 0x5Au ? 1u : 0u));
        }
        spilink::Report rep{};
        (void)peer_report(rep);
    }
    print(serial, "  -> this chip's client MISO, MSb first, on the standard edge and carrying its answer in ", own_std,
          " of 4 modes; LSb first a line that never moves in ", own_lsb_dead, " of 4", crlf);
    bench.verdict("THIS CHIP AS CLIENT drives MISO on the standard edge with its answer in every mode, MSb first "
                  "(16.2.3's \"mode 2 or mode 3\" does not hold on this die)",
                  own_std == 4u);
    bench.verdict("and LSb first its MISO never moves, in every mode (16.3.1: \"LSB is only supported by SPI as "
                  "host\" - for what a client sends)",
                  own_lsb_dead == 4u);
    (void)link_command_mode();
    bench.verdict("the command channel is back after the waveforms", command(Op::ping));
}

// ===========================================================================
// x - the slip statistics (a diagnostic, by name)
// ===========================================================================

void tx_peer_slips() {
    if (!need_peer()) {
        return;
    }
    print(serial, "  each mode ten times, 8 frames at HCLK/256: bursts not byte-exact (first bad host/client)", crlf);
    uint8_t total = 0;
    for (uint8_t m = 0; m < 4; ++m) {
        uint8_t slips = 0;
        print(serial, "  mode ", m, ":");
        for (uint8_t k = 0; k < 10u; ++k) {
            Exchange e{};
            e.cfg.mode = m;
            e.host_mode = static_cast<SpiMode>(m);
            e.seed_a = static_cast<uint8_t>(0x13u + m + 7u * k);
            e.seed_b = static_cast<uint8_t>(0x57u + m + 11u * k);
            Verify v{};
            spilink::Report r{};
            if (!exchange_exact(e, v, r)) {
                ++slips;
                print(serial, " [", v.idx, "/", r.idx, r.count != e.count ? "!" : "", "]");
            }
        }
        print(serial, "  -> ", slips, " of 10", crlf);
        total = static_cast<uint8_t>(total + slips);
    }
    print(serial, "  ", total, " of 40 bursts slipped (a probe: no verdict)", crlf);
    (void)link_command_mode();
    print(serial, "  the command channel afterwards: ", command(Op::ping) ? "answers" : "DEAD", crlf);
}

bool supply_ok = true;

void banner() {
    print(serial, crlf, "test_x035_spi on ", device::part_name, " - SPI1 on PA4..PA7", crlf, crlf);
    print(serial, "  supply at boot: ", supply_ok ? "above 3.0 V" : "UNDER 3.0 V, every letter refused",
          " (the voltage detector at PLS 10)", crlf, crlf);
    bench.menu();
    print(serial, crlf);
}

}  // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() {
    const uint32_t t0 = stamp();
    brio::Ticker::tick();
    const uint32_t d = stamp_span(t0, stamp());
    if (d > tick_body_max) {
        tick_body_max = d;
    }
}

extern "C" BRIO_CH32_INTERRUPT void usart2_handler() {
    const uint32_t t0 = stamp();
    (void)Serial::isr();
    const uint32_t d = stamp_span(t0, stamp());
    if (d > console_body_max) {
        console_body_max = d;
    }
}

extern "C" BRIO_CH32_INTERRUPT void spi1_handler() {
    const uint32_t t0 = stamp();
    spi_isr_entries = spi_isr_entries + 1u;
    switch (owner) {
        case Owner::dma_host:
            if (DmaHost::isr()) {
                host_done = true;
            }
            break;
        case Owner::flash_dma_host:
            if (FlashDmaHost::isr()) {
                host_done = true;
            }
            break;
        case Owner::flash_host:
            if (FlashHost::isr()) {
                host_done = true;
            }
            break;
        case Owner::bus_ao:
            if (Host::isr()) {
                brio::post<kl::SpiArb>(brio::TransferDone{Host::status()});
            }
            break;
        case Owner::host:
            if (Host::isr()) {
                host_done = true;
            }
            break;
        default:
            S::rxne_interrupt(false);
            S::txe_interrupt(false);
            S::error_interrupt(false);
            break;
    }
    const uint32_t d = stamp_span(t0, stamp());
    if (d > spi_body_max) {
        spi_body_max = d;
    }
}

[[gnu::always_inline]] inline void spi_dma() {
    dma_isr_entries = dma_isr_entries + 1u;
    if (owner == Owner::dma_host && DmaHost::dma_isr()) {
        host_done = true;
    } else if (owner == Owner::flash_dma_host && FlashDmaHost::dma_isr()) {
        host_done = true;
    }
}
extern "C" BRIO_CH32_INTERRUPT void dma1_channel2_handler() { spi_dma(); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel3_handler() { spi_dma(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    // THE SUPPLY before any link pad is driven (the desk's rule).
    brio::Pwr::pvd(brio::PvdLevel::v3_0);
    (void)brio::delay_us(clock, 200);
    supply_ok = !brio::Pwr::supply_low();
    if (supply_ok) {
        host_ready();   // PA4 high: the far client deselected from boot
    }

    bench.letter('a', "the block, wireless: reset values, the control words, the enable protection, HSCR, "
                      "ODEN, SSOE, the transmit CRC, every path",
                 ta_block);
    bench.letter('b', "THE RATE LADDER on the wire, the high-speed read's rate, the host's costs", tb_ladder);
    bench.letter('c', "the refusals", tc_refusals);
    bench.second_letter('n', "THE PEER: the command channel, ident, ten pings", tn_peer_link);
    bench.second_letter('o', "the matrix: four modes, both orders, a mismatch, the receive CRC", to_peer_matrix);
    bench.second_letter('p', "THE LADDER against the peer, both host paths, both serves, /2 with the high-speed read",
                        tp_peer_rates);
    bench.second_letter('k', "THE CLIENT'S RUNS: the peer's answer, exchange and receive, both widths, gapped and "
                             "back to back",
                        tk_peer_runs);
    bench.second_letter('q', "THE KERNEL against the peer: four transactions in one select window", tq_peer_kernel);
    bench.second_letter('r', "THE ROLES INVERT: this chip as the client in every mode and order", tr_peer_client);
    bench.second_letter('m', "MODF from a second chip, and ODEN's NSS on its pull-up", tm_peer_modf);
    bench.second_letter('w', "THE WAVEFORMS: this chip as the analyser of both chips' SCK and data edges", tw_waveforms);
    bench.letter('x', "slip statistics against the peer: each mode ten times (no verdict)", tx_peer_slips, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "HSI48" : "FAILED", " tick=",
                    tick_ok ? "STK" : "FAILED", " supply=", supply_ok ? "above 3.0 V" : "LOW", brio::crlf);
        if (!supply_ok) {
            brio::print(serial, "SUPPLY UNDER 3.0 V (the PVD at PLS 10): every letter refused, the link pads left "
                                "floating",
                        brio::crlf);
        }
        banner();
        bench.prompt();
    }

    for (;;) {
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        brio::print(serial, static_cast<char>(c), brio::crlf);
        if (c == '?') {
            banner();
        } else if (!supply_ok) {
            brio::print(serial, "refused: the supply is under 3.0 V", brio::crlf);
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        brio::print(serial, "  stack: ", brio::stack_untouched(), " B never touched", brio::crlf);
        bench.prompt();
    }
}
