// test_ch32_spi - the reference bench suite for the CH32V00x's SPI
// chapter: ch32v00x/spi.hpp over RM ch. 16 - the block, the polled path
// at every rate, the pump, the DMA engines, the hardware CRC, the client
// on its hardware select, MODF - and util/spi_bus.hpp's arbiter with not
// one line changed, every frame on the wire witnessed by A SECOND CHIP.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// THE PEER is the desk's CH32X035 running `spi_peer` (kind 0x07),
// commanded IN BAND over the bus under test through
// avrxt/src/apps/spi_link.hpp, included by relative path: one source of
// truth for the wire format on every architecture. SPI1's column 0 to the
// CH32X035's SPI1 column 0:
//
//   PC5 (SCK)   - PA5        PC6 (MOSI) - PA7
//   PC7 (MISO)  - PA6        PC1 (NSS)  - PA4, the select wire
//
// PC1 is this end's GPIO select for every letter that talks to the peer,
// its HARDWARE NSS input for the two that need one (r, the client; m,
// MODF), and held HIGH through the single-board letters - which select
// PC3, a pad with no wire, so the peer's dark listener is never selected
// by them and what goes out on SCK and MOSI reaches a deselected client.
// The peer FAILS loudly when it does not answer. That peer answers its
// windows in spi_link's mode 0 (its client answers in every mode MSb
// first) and serves the EXTENDED EXCHANGE - 16-bit frames, the CRC with
// its polynomial, a receive-only run - which is how the 16-bit frames,
// the CRC unit and the transmit-only loop of this host get a foreign
// witness.
//
// What is exercised, letter by letter:
//   a  the block, WIRELESS: the reset values, the control words, the
//      refusals, the enable protection MEASURED field by field, the
//      arithmetic, and the pump, the polled path and the engines each
//      run to completion with nothing driving MISO
//   b  THE PUMP against the peer: the four modes at 8 and 16 bits,
//      sixteen frames each through the ISR, both directions judged, one
//      interrupt a frame; a command phase then a data phase; a read with
//      no out buffer
//   c  THE POLLED PATH AT EVERY RATE, /2 to /256, 64 frames each with the
//      peer deselected, the burst timed - none faster than its wire
//   d  THE DMA ENGINES on channels 2 and 3 against the peer: blocks with
//      and without a command phase, ONE DMA INTERRUPT a transaction,
//      16-bit frames in half-word beats, an odd buffer's fallback to the
//      pump, a polled request at its own width after an asynchronous one
//      at the other, the 0xFF cell; and THE RATE ON THE WIRE - a frame
//      inside an engined block timed against its SCK periods, the peer
//      deselected
//   e  THE HARDWARE CRC against a foreign CRC unit: 8 and 16 bits, each
//      end's TXCRCR and RXCRCR the bitwise arithmetic, each end checking
//      the other's CRC frame - and a polynomial mismatch caught at both
//   f  THE KERNEL: SpiBus (= BusMaster) over SpiHost, replies in order,
//      the rejection, both votes (the peer deselected)
//   h  THE STALLED EXIT, wireless: a polled request over a block held in
//      reset ends spi_stalled within its budget, and recover() after it
//   n  THE PEER: the command channel, ident (kind 0x07), ten pings
//   o  the matrix: this host in the four modes against the foreign
//      client on both of its serves, LSb first as an exact two-way
//      reversal against its MSb-first client, LSb first both ends where
//      the client only listens
//   p  THE LADDER, /256 to /2, the polled host and the engines, against
//      the peer's DMA serve and its polled run; this host's transmit-only
//      loop heard whole by a client that only listens
//   q  THE KERNEL against the peer: four transactions queued from one
//      dispatch in ONE select window, read back by the second chip
//   r  THE ROLES INVERT: this chip as the CLIENT on its HARDWARE NSS
//      (PC1) under the peer's host_burst - modes 0 and 3 judged both
//      ways, MSb and LSb first, 16-bit frames; modes 1 and 2 refused by
//      the driver
//   m  MODF from a second chip: a host whose NSS is a hardware input,
//      demoted four times by the peer's ss_pulse, and the clearing
//      sequence
//   x  (outside z) the slip statistics: each mode ten times
//
// build: boards = v006k8,v003f4
// build: groups = a,b,c,d,g,e,f,h,n,o,p,q,r,m,x,s
// build: groups.v006k8 = abcdgefh,nopqrmxs
// (on the CH32V006 the suite whole comes within 5 % of the 40 KB of program
// flash, so it builds as two images there: letters a to h, and the
// command channel, the matrix, the ladder, the kernel against the peer, the
// client, MODF and the two statistics letters)
// build: monitor_speed = 115200

#include <stdint.h>

#include <variant>

#include "ch32v00x/clock.hpp"
#include "ch32v00x/delay.hpp"
#include "ch32v00x/dma.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/pin.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/spi.hpp"
#include "ch32v00x/ticker.hpp"
#include "ch32v00x/usart.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/post.hpp"
#include "kernel/time.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/spi_bus.hpp"
#include "util/testbench.hpp"

#include "../../../avrxt/src/apps/spi_link.hpp"

using P = brio::Ch32v00xPlatform<>;
using SysClock = brio::Clock<brio::ClockSource::pll, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<1, P, 64, 128>;
constexpr Serial serial;

using S = Spi<1>;
using Host = SpiHost<1>;
// The engines at DATAR's width: 8- and 16-bit frames both on them.
using DmaTx = DmaTxEngine<3, uint16_t>;
using DmaRx = DmaRxEngine<2, uint16_t>;
using DmaHost = SpiHost<1, spi1_default_pins, DmaTx, DmaRx>;
using SckPin = Pin<'C', 5>;
using MosiPin = Pin<'C', 6>;
using MisoPin = Pin<'C', 7>;
/// The select wire to the peer (SPI1's NSS pad of column 0).
using CsPin = Pin<'C', 1>;
/// The single-board letters' select: a pad with no wire.
using LoneCs = Pin<'C', 3>;

static_assert(spi1_default_pins.nss == Pad{'C', 1} && spi1_default_pins.sck == Pad{'C', 5} &&
                  spi1_default_pins.mosi == Pad{'C', 6} && spi1_default_pins.miso == Pad{'C', 7},
              "SPI1's column 0: NSS PC1, SCK PC5, MOSI PC6, MISO PC7");

TestBench<Serial> bench;

/// What THIS image carries (a group image on the CH32V003 carries a few
/// letters): the vectors and the window serve only what some carried
/// letter starts, so a group image links no host it never runs.
constexpr bool carries(char key) { return selection_carries(test_letters, key); }
/// The engines: a's completion, d, g, p, s.
constexpr bool image_engines = carries('a') || carries('d') || carries('g') || carries('p') || carries('s');
/// The plain host's pump: a's completion, b, and the kernel's f and q.
constexpr bool image_pump = carries('a') || carries('b') || carries('f') || carries('q');
/// The arbiter: f and q.
constexpr bool image_kernel = carries('f') || carries('q');

volatile bool host_done = false;
volatile bool dma_host_live = false;
volatile bool bus_ao_live = false;
volatile uint32_t spi_isr_entries = 0;
volatile uint32_t dma_isr_entries = 0;

// Word-aligned: a 16-bit request on the engines needs half-word
// boundaries (RM 8.3.6), and letter d makes an odd one on purpose.
alignas(4) uint8_t tx_buf[256];
alignas(4) uint8_t rx_buf[256];
uint8_t cmd_buf[4];

/// A transaction that never answered: the suite's own word.
constexpr uint8_t no_answer = 200;

void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    (void)delay_us(clock, 500);
}

/// A cycle count that spans ticks: the kernel ticker's own cycles(), the
/// tick count and the counter's position composed right across the reload
/// (util/cycle_count.hpp); it wraps at 2^32, so only differences are used.
uint32_t cycles_now() {
    return Ticker::cycles();
}

void fill_pattern(uint8_t* p, uint16_t n, uint8_t seed) {
    for (uint16_t i = 0; i < n; ++i) {
        p[i] = static_cast<uint8_t>(seed + i * 7u + (i >> 3));
    }
}

/// The plain host, the peer's select held HIGH and the lone select too.
void host_ready() {
    dma_host_live = false;
    bus_ao_live = false;
    (void)Host::init(clock);
    CsPin::output(true);
    LoneCs::output(true);
}

/// The host with its engines, the same two selects high.
void dma_ready() {
    dma_host_live = true;
    bus_ao_live = false;
    (void)DmaHost::init(clock);
    CsPin::output(true);
    LoneCs::output(true);
}

/// One transaction through the plain host, waited out. The status, or
/// no_answer.
uint8_t host_xfer(PinRef cs, const uint8_t* cmd, uint8_t cmd_len, const uint8_t* tx, uint8_t* rx, uint16_t len,
                  SpiMode mode, SpiClock rate, SpiDataSize bits, bool polled) {
    Host::Request r{};
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
    dma_isr_entries = 0;
    if (Host::start(r)) {
        return Host::status();
    }
    for (uint32_t i = 0; i < 2'000'000UL && !host_done; ++i) {
    }
    if (!host_done) {
        print(serial, "    STALL: STATR=", hex(S::status()), " CTLR1=", hex(S::regs().CTLR1),
              " CTLR2=", hex(S::regs().CTLR2), " isr entries ", spi_isr_entries, crlf);
        (void)Host::recover();
        return no_answer;
    }
    return Host::status();
}

uint8_t dma_xfer(PinRef cs, const uint8_t* cmd, uint8_t cmd_len, const uint8_t* tx, uint8_t* rx, uint16_t len,
                 SpiClock rate, SpiDataSize bits, bool polled, SpiMode mode = SpiMode::mode0) {
    DmaHost::Request r{};
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
    dma_isr_entries = 0;
    if (DmaHost::start(r)) {
        return DmaHost::status();
    }
    for (uint32_t i = 0; i < 2'000'000UL && !host_done; ++i) {
    }
    if (!host_done) {
        print(serial, "    STALL (dma): STATR=", hex(S::status()), " ch2 flags=",
              hex(DmaChannel<2>::flags()), " ch3 flags=", hex(DmaChannel<3>::flags()), crlf);
        (void)DmaHost::recover();
        return no_answer;
    }
    return DmaHost::status();
}

void settle_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

/// The SCK divisor a BR code ASKS for, HCLK / 2^(BR + 1) (16.3.1) - the
/// rate the wire is judged against, never the host's own arithmetic.
uint32_t asked_divisor(SpiClock rate) { return 2u << static_cast<uint8_t>(rate); }

/// THE RATE ON THE WIRE: the time ONE FRAME takes inside an engined block,
/// from the difference of a `big_n`- and a `small_n`-frame block, so the
/// transaction's fixed cost cancels; in thousandths of the frame's SCK
/// periods at the rate ASKED. With no buffer named (the default, 2048 and
/// 256 frames) the transmit engine pours the host's fixed 0xFF cell and
/// the receive engine writes its discard cell. No gap is added: a frame
/// written while another shifts follows it with none (16.2.2), so a frame
/// is its bits alone. The best of three of each block; the arithmetic in
/// 32 bits (the CH32V003 has no multiplier). The lone select: the peer is
/// never selected by these blocks.
uint32_t frame_permille(SpiClock rate, SpiMode mode, SpiDataSize bits, const uint8_t* tx = nullptr,
                        uint8_t* rx = nullptr, uint16_t big_n = 2048, uint16_t small_n = 256) {
    auto best = [&](uint16_t n) {
        uint32_t least = 0xFFFFFFFFu;
        for (uint8_t k = 0; k < 3u; ++k) {
            const uint32_t t0 = cycles_now();
            (void)dma_xfer(LoneCs::ref(), nullptr, 0, tx, rx, n, rate, bits, false, mode);
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
    const uint32_t frame_x1000 = (d / frames) * 1000u + ((d % frames) * 1000u) / frames;
    const uint32_t width = bits == SpiDataSize::bits16 ? 16u : 8u;
    return frame_x1000 / (width * asked_divisor(rate));
}

bool wire_ok(uint32_t permille) { return permille >= 990u && permille <= 1030u; }

// ===========================================================================
// The peer: spi_link.hpp over the bus under test
// ===========================================================================

using spilink::Op;

/// The command channel's SCK: HCLK/256 = 187.5 kHz. A character is
/// 43 us, an order of magnitude more than the peer's polled command
/// listener needs to turn one around.
constexpr SpiClock link_clock = SpiClock::div256;

uint8_t frame_buf[spilink::max_payload + 8];
uint8_t answer_buf[spilink::answer_bytes];
uint8_t raw_seen[16];
uint8_t raw_n = 0;
spilink::Decoder dec;
bool link_quiet = false;

/// The hold around each select window: the protocol owns the chip select
/// for these windows (the Request carries a null PinRef) and pays 30 us on
/// each side, for the client's loop to turn the edge around.
void link_hold() { (void)delay_us(clock, 30); }

/// SPI1 as the command channel's host, PC1 as its GPIO chip select.
bool link_command_mode() {
    dma_host_live = false;
    bus_ao_live = false;
    const bool ok = Host::init(clock);
    CsPin::output(true);
    LoneCs::output(true);
    dec.reset();
    return ok;
}

/// One protocol window: prime, select, hold, the transaction, hold,
/// deselect. THE MODE IS PRIMED BEFORE THE SELECT FALLS - a CPOL flip
/// inside an open window is one extra edge and the selected client
/// counts it into the frame (SpiHost::prime()'s own comment).
bool link_xfer(const uint8_t* tx, uint8_t* rx, uint16_t n) {
    if (n == 0) {
        return true;
    }
    Host::prime(SpiMode::mode0, link_clock);
    CsPin::clear();
    link_hold();
    Host::Request r{};
    r.cs = {};
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
    r.cmd_len = 0;
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = n;
    r.clock = link_clock;
    r.mode = SpiMode::mode0;
    r.bits = SpiDataSize::bits8;
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

/// One answer window: `answer_bytes` dummies in a single transaction
/// (a null out buffer: the engine's own 0xFF, which is no frame's magic
/// and costs no RAM on a 2 KB part), then the decoder walks what came
/// back (the peer pads its frame with zeros, which the decoder ignores).
bool recv_frame(spilink::Frame& out) {
    dec.reset();
    raw_n = 0;
    for (uint16_t i = 0; i < spilink::answer_bytes; ++i) {
        answer_buf[i] = 0xEE;
    }
    (void)link_xfer(nullptr, answer_buf, spilink::answer_bytes);
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
/// window bound, so a peer that was serving into nothing is certainly
/// dark again before the retry - the protocol's own recovery guarantee.
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

/// Is the peer there at all? Every letter that needs it asks first, so
/// an absent peer is a NAMED failure and never a hang.
bool ensure_link() {
    link_quiet = true;
    (void)link_command_mode();
    for (uint8_t k = 0; k < 3; ++k) {
        if (command(Op::ping)) {
            link_quiet = false;
            return true;
        }
    }
    link_quiet = false;
    print(serial, "  THE PEER DID NOT ANSWER: the far board must run `spi_peer` (its console '0' forces the dark "
          "client back); the four wires are in this file's header.", crlf);
    return false;
}

bool need_peer() {
    if (ensure_link()) {
        return true;
    }
    bench.verdict("the peer answers a ping (the CH32X035 running spi_peer)", false);
    return false;
}

// ---- one exchange, commanded and then clocked ------------------------------

/// How this host clocks an exchange's data window.
enum class Path : uint8_t {
    polled,          ///< the plain host, polled inside start()
    pump,            ///< the plain host, the RXNE interrupt
    engines,         ///< the engines, the receive channel's interrupt
    engines_polled,  ///< the engines, spun inside start()
};

const char* path_name(Path p) {
    switch (p) {
        case Path::polled: return "polled";
        case Path::pump: return "pump";
        case Path::engines: return "engines";
        default: return "engines polled";
    }
}

struct Exchange {
    spilink::Cfg cfg{.apply = 1, .mode = 0, .dord = 0, .regime = spilink::regime_buffer_wait};
    SpiMode host_mode = SpiMode::mode0;
    bool host_lsb = false;
    SpiClock rate = link_clock;
    Path path = Path::polled;
    uint16_t count = 8;      ///< frames, the command phase's included
    uint8_t cmd_len = 0;     ///< of them, sent as the request's command phase (8-bit frames)
    bool wide = false;       ///< 16-bit frames (spi_link's spare_wide)
    bool no_out = false;     ///< the data phase with no out buffer: 0xFF dummies
    bool no_in = false;      ///< the data phase with no in buffer: the transmit-only path
    uint8_t offset = 0;      ///< the buffers' offset (an odd one on purpose)
    uint8_t seed_a = 0x13;
    uint8_t seed_b = 0x57;
    uint8_t pattern = spilink::pattern_prbs;
    uint8_t flags = 0;
    uint8_t spare = 0;
    uint16_t ms = 400;
    // What happened, filled by do_exchange():
    uint8_t status = no_answer;
    uint32_t spi_entries = 0;
    uint32_t dma_entries = 0;
};

bool reversed_of(const Exchange& e) { return e.host_lsb != (e.cfg.dord != 0); }
uint8_t width_of(const Exchange& e) { return e.wide ? 2u : 1u; }

/// The data window of an exchange, on the path asked: the mode and bit
/// order primed BEFORE the select falls, the select held by hand around
/// the one request.
uint8_t window(const Exchange& e, const uint8_t* cmd, const uint8_t* tx, uint8_t* rx, uint16_t len) {
    const bool engines = image_engines && (e.path == Path::engines || e.path == Path::engines_polled);
    const SpiDataSize bits = e.wide ? SpiDataSize::bits16 : SpiDataSize::bits8;
    uint8_t st = no_answer;
    if constexpr (image_engines) {
        if (engines) {
            dma_ready();
            (void)DmaHost::bit_order(e.host_lsb);
            DmaHost::prime(e.host_mode, e.rate, bits);
        }
    }
    if (!engines) {
        (void)link_command_mode();
        (void)Host::bit_order(e.host_lsb);
        Host::prime(e.host_mode, e.rate, bits);
    }
    (void)delay_us(clock, 20);   // SCK at its idle level before the select
    CsPin::clear();
    link_hold();
    if constexpr (image_engines) {
        if (engines) {
            st = dma_xfer({}, cmd, e.cmd_len, tx, rx, len, e.rate, bits, e.path == Path::engines_polled,
                          e.host_mode);
        }
    }
    if (!engines) {
        st = host_xfer({}, cmd, e.cmd_len, tx, rx, len, e.host_mode, e.rate, bits, e.path == Path::polled);
    }
    link_hold();
    CsPin::set();
    if constexpr (image_engines) {
        if (engines) {
            DmaHost::release();
            dma_host_live = false;
        }
    }
    (void)link_command_mode();
    return st;
}

bool do_exchange(Exchange& e) {
    spilink::Params a{};
    a.cfg = e.cfg;
    a.count = e.count;
    a.ms = e.ms;
    a.seed_a = e.seed_a;
    a.seed_b = e.seed_b;
    a.pattern = e.pattern;
    a.flags = e.flags;
    a.spare = e.spare;
    if (e.wide) {
        a.spare |= spilink::spare_wide;
    }
    // A bit-order mismatch is not a shrug: told about it, each end
    // checks the EXACT bit-reverse of what the other sent.
    if (reversed_of(e)) {
        a.flags |= spilink::flag_expect_reversed;
    }
    e.status = no_answer;
    if (!peer_act(Op::exchange, a)) {
        return false;
    }
    const uint16_t bytes = static_cast<uint16_t>(e.count * width_of(e));
    spilink::Stream out(e.pattern, e.seed_a);
    for (uint16_t i = 0; i < sizeof tx_buf; ++i) {
        tx_buf[i] = 0;
        rx_buf[i] = 0xEE;
    }
    for (uint16_t i = 0; i < bytes && e.offset + i < sizeof tx_buf; ++i) {
        tx_buf[e.offset + i] = out.next();
    }
    settle();
    const uint16_t lead = static_cast<uint16_t>(e.cmd_len * width_of(e));
    e.status = window(e, tx_buf + e.offset, e.no_out ? nullptr : tx_buf + e.offset + lead,
                      e.no_in ? nullptr : rx_buf + e.offset + lead, static_cast<uint16_t>(e.count - e.cmd_len));
    e.spi_entries = spi_isr_entries;
    e.dma_entries = dma_isr_entries;
    return true;
}

struct Verify {
    uint16_t mism = 0;
    uint8_t idx = 0xFF;
    uint8_t got = 0;
    uint8_t exp = 0;
};

/// What this end read, byte by byte in memory order, against the client's
/// stream: the data phase's bytes alone (the command's echo is discarded).
Verify verify_rx(const Exchange& e) {
    Verify v{};
    if (e.no_in) {
        return v;
    }
    const uint16_t bytes = static_cast<uint16_t>(e.count * width_of(e));
    for (uint16_t i = static_cast<uint16_t>(e.cmd_len * width_of(e)); i < bytes; ++i) {
        const uint8_t exp = spilink::wide_byte(e.pattern, e.seed_b, i, e.wide, reversed_of(e));
        const uint8_t got = rx_buf[e.offset + i];
        if (got != exp) {
            if (v.mism == 0) {
                v.idx = static_cast<uint8_t>(i < 255u ? i : 255u);
                v.got = got;
                v.exp = exp;
            }
            ++v.mism;
        }
    }
    return v;
}

/// The extended exchange is the peer's to serve, and its report says so.
bool needs_extended(const Exchange& e) {
    return e.wide || (e.spare & (spilink::spare_crc | spilink::spare_receive_only)) != 0u;
}

bool extended_ok(const Exchange& e, const spilink::Report& r) {
    return !needs_extended(e) || (r.flags & spilink::report_extended) != 0u;
}

bool exchange_exact(Exchange& e, Verify& v, spilink::Report& r) {
    const bool ran = do_exchange(e);
    const bool rep = ran && peer_report(r);
    v = verify_rx(e);
    return ran && rep && e.status == spi_ok && v.mism == 0 && r.mism == 0 && r.count == e.count && extended_ok(e, r);
}

void dump_exchange(const Exchange& e, const Verify& v, const spilink::Report& r) {
    print(serial, "    status ", e.status, ", host mism=", v.mism, " (first idx ", v.idx, " got ", hex(v.got), " exp ",
          hex(v.exp), "), client count=", r.count, " mism=", r.mism, " (first idx ", r.idx, " got ", hex(r.got),
          " exp ", hex(r.exp), ") flags=", hex(r.flags), " aux=", hex(r.aux0), " ", r.aux1, " ", hex(r.aux2), crlf);
    if (needs_extended(e) && (r.flags & spilink::report_extended) == 0u) {
        print(serial, "    no report_extended: the peer does not serve the extended exchange", crlf);
    }
    if (e.no_in) {
        return;
    }
    const uint16_t bytes = static_cast<uint16_t>(e.count * width_of(e));
    print(serial, "    read:");
    for (uint16_t i = 0; i < bytes && i < 16u; ++i) {
        print(serial, " ", hex(rx_buf[e.offset + i]));
    }
    print(serial, crlf);
}

/// One exchange judged and printed on one line; the dump when it was not
/// exact.
bool exchange_line(Exchange& e, const char* label) {
    Verify v{};
    spilink::Report r{};
    const bool ok = exchange_exact(e, v, r);
    print(serial, "  ", label, ok ? ": exact both ways (" : ": NOT EXACT (", path_name(e.path), " /",
          asked_divisor(e.rate), ", ", e.count, e.wide ? "x16" : "x8", " bits; irqs ", e.spi_entries, " SPI ",
          e.dma_entries, " DMA)", crlf);
    if (!ok) {
        dump_exchange(e, v, r);
    }
    return ok;
}

// ===========================================================================
// a - the block, wireless
// ===========================================================================

void ta_block() {
    dma_host_live = false;
    bus_ao_live = false;
    CsPin::output(true);   // the peer deselected through the whole letter
    LoneCs::output(true);
    Pfic::disable(S::irq());
    S::bus_clock(true);
    S::reset();
    print(serial, "  reset: CTLR1=", hex(S::regs().CTLR1), " CTLR2=", hex(S::regs().CTLR2),
          " STATR=", hex(S::status()), " CRCR=", hex(S::regs().CRCR), " HSCR=", hex(S::regs().HSCR),
          crlf);
    bench.verdict("the reset values are table 16-2's (STATR 0x0002: TXE up, CRCR 0x0007)",
                  S::regs().CTLR1 == 0u && S::regs().CTLR2 == 0u && S::status() == 0x0002u &&
                      S::regs().CRCR == 0x0007u);

    const bool cfg = S::configure({.role = SpiRole::host, .mode = SpiMode::mode3,
                                   .clock = SpiClock::div16, .bits = SpiDataSize::bits16});
    print(serial, "  mode 3 host /16 16-bit: CTLR1=", hex(S::regs().CTLR1), crlf);
    bench.verdict("the control word lands as spelled (CPHA, CPOL, MSTR, BR 3, SSM, SSI, DFF)",
                  cfg && S::regs().CTLR1 == (spi_cpha | spi_cpol | spi_mstr | (3u << 3) | spi_ssm |
                                             spi_ssi | spi_dff));
    bench.verdict("the refusals: CRC in half duplex, a zero polynomial, a client with SSOE",
                  !spi_config_valid({.direction = SpiDirection::half_duplex_out, .crc = true}) &&
                      !spi_config_valid({.crc = true, .crc_polynomial = 0}) &&
                      !spi_config_valid({.role = SpiRole::client, .nss = SpiNss::hardware_output}));
    bench.verdict("the arithmetic: /2 is 24 MHz, /256 is 187.5 kHz, 1 MHz asks /64, below "
                  "187.5 kHz is refused",
                  spi_sck_hz(SysClock::hz, SpiClock::div2) == 24'000'000UL &&
                      spi_rate_for(SysClock::hz, 1'000'000UL) == SpiClock::div64 &&
                      !spi_rate_for(SysClock::hz, 100'000UL).has_value());

    // THE ENABLE PROTECTION, MEASURED FIELD BY FIELD: 16.3.1 says DFF
    // and CRCEN "can only be written when SPE is 0" and BR/CPOL/CPHA/
    // MSTR "cannot be modified during communication". A raw write to
    // each with SPE set, read back.
    (void)S::configure({.role = SpiRole::host, .mode = SpiMode::mode0, .clock = SpiClock::div16,
                        .bits = SpiDataSize::bits8});
    S::enable();
    const uint16_t base = S::regs().CTLR1;
    struct Field {
        const char* name;
        uint16_t bit;
    };
    const Field fields[] = {{"DFF", spi_dff}, {"CRCEN", spi_crcen}, {"CPOL", spi_cpol},
                            {"CPHA", spi_cpha}, {"LSBFIRST", spi_lsbfirst}, {"BR0", 1u << 3},
                            {"MSTR", spi_mstr}};
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
    print(serial, crlf, "  -> ", taken, " of 7 fields take a write under SPE; the driver's refusals "
          "are the protection where the silicon has none", crlf);
    bench.verdict("the enable protection was measured field by field (a finding either way)",
                  true);
    (void)S::disable();
    bench.verdict("disable() drains and drops SPE", !S::enabled());
    S::reset();

    // THE ENGINE COMPLETES WITH NOTHING DRIVING MISO: the pump, the polled
    // path and the DMA engines each run a transaction to its end on the
    // lone select, the peer deselected - what comes back is not judged
    // here (the peer letters do that), only that every path ENDS and
    // releases the select.
    host_ready();
    fill_pattern(tx_buf, 16, 0x21);
    const uint8_t pumped = host_xfer(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode0,
                                     SpiClock::div16, SpiDataSize::bits8, false);
    const uint32_t pump_entries = spi_isr_entries;
    const uint8_t polled = host_xfer(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode3,
                                     SpiClock::div2, SpiDataSize::bits16, true);
    dma_ready();
    const uint8_t dma = dma_xfer(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 64, SpiClock::div4,
                                 SpiDataSize::bits8, false);
    const uint8_t dma_polled = dma_xfer(LoneCs::ref(), cmd_buf, 1, tx_buf, rx_buf, 32, SpiClock::div2,
                                        SpiDataSize::bits8, true);
    DmaHost::release();
    dma_host_live = false;
    print(serial, "  wireless completions: pump=", pumped, " (", pump_entries, " isr entries) polled=",
          polled, " dma=", dma, " dma polled=", dma_polled, " lone select released=", LoneCs::read_out(),
          " peer's select high=", CsPin::read_out(), crlf);
    bench.verdict("the pump (sixteen entries for sixteen frames), the polled path and the engines each complete "
                  "with nothing driving MISO",
                  pumped == spi_ok && pump_entries == 16u && polled == spi_ok && dma == spi_ok &&
                      dma_polled == spi_ok && LoneCs::read_out() && CsPin::read_out());
    host_ready();
}

// ===========================================================================
// b - THE PUMP against the peer
// ===========================================================================

void tb_pump() {
    if (!need_peer()) {
        return;
    }
    // THE FOUR MODES AT 8 AND 16 BITS through the ISR pump, sixteen frames
    // each: what this host sends judged by the foreign client's report,
    // what it reads judged here against the client's stream. The peer's
    // client answers in every mode MSb first (its header).
    uint8_t exact = 0;
    uint8_t one_a_frame = 0;
    for (uint8_t m = 0; m < 4u; ++m) {
        for (uint8_t w = 0; w < 2u; ++w) {
            Exchange e{};
            e.path = Path::pump;
            e.rate = SpiClock::div16;
            e.count = 16;
            e.wide = w != 0u;
            e.cfg.mode = m;
            e.host_mode = static_cast<SpiMode>(m);
            e.seed_a = static_cast<uint8_t>(0x10u * m + w + 1u);
            e.seed_b = static_cast<uint8_t>(0x91u + 0x10u * m + w);
            char label[] = "mode 0  8-bit";
            label[5] = static_cast<char>('0' + m);
            if (w != 0u) {
                label[7] = '1';
                label[8] = '6';
            }
            if (exchange_line(e, label)) {
                ++exact;
            }
            if (e.spi_entries == 16u) {
                ++one_a_frame;
            }
        }
    }
    bench.verdict("the four modes at 8 and 16 bits, sixteen frames each through the ISR pump, exact BOTH ways "
                  "against a foreign client",
                  exact == 8u);
    bench.verdict("one interrupt per frame (sixteen entries for sixteen frames, every run)", one_a_frame == 8u);

    // A command phase then a data phase, on the pump: the client hears all
    // ten frames, this end keeps the data phase's answers and discards the
    // command's echo.
    Exchange c{};
    c.path = Path::pump;
    c.rate = SpiClock::div8;
    c.count = 10;
    c.cmd_len = 2;
    c.seed_a = 0x77;
    c.seed_b = 0x3E;
    bench.verdict("a two-frame command then eight data frames: the client heard ten, this end read the data, the "
                  "echo discarded",
                  exchange_line(c, "command + data"));
    // Read with no out buffer: 0xFF dummies go out, the client hears them.
    Exchange d{};
    d.path = Path::pump;
    d.rate = SpiClock::div8;
    d.count = 8;
    d.no_out = true;
    d.pattern = spilink::pattern_fixed;
    d.seed_a = 0xFF;
    d.seed_b = 0x5A;
    bench.verdict("a read with no out buffer clocks 0xFF dummies - the client heard eight 0xFF - and reads the "
                  "client's answers",
                  exchange_line(d, "no out buffer"));
    bench.verdict("the select is released after every window", CsPin::read_out());
}

// ===========================================================================
// c - the polled path at every rate
// ===========================================================================

void tc_rates() {
    host_ready();
    uint8_t complete = 0;
    uint8_t never_early = 0;
    for (uint8_t code = 0; code < 8u; ++code) {
        const SpiClock rate = static_cast<SpiClock>(code);
        fill_pattern(tx_buf, 64, static_cast<uint8_t>(0xA0u + code));
        console_drain();
        const uint32_t t0 = cycles_now();
        const uint8_t st = host_xfer(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 64, SpiMode::mode0, rate,
                                     SpiDataSize::bits8, true);
        const uint32_t cycles = cycles_now() - t0;
        // The wire's own time for 64 frames of 8 bits at the SCK ASKED.
        const uint32_t sck = SysClock::hz / asked_divisor(rate);
        const uint32_t wire_cycles = 64UL * 8UL * asked_divisor(rate);
        print(serial, "  /", 2u << code, " (", sck / 1000u, " kHz): 64 frames polled in ", cycles,
              " cycles (", cycles / 64u, " per frame, the wire alone ", wire_cycles / 64u, "), status ", st, crlf);
        if (st == spi_ok) {
            ++complete;
        }
        if (cycles >= wire_cycles) {
            ++never_early;
        }
    }
    bench.verdict("all eight BR codes carry 64 frames through the polled receive loop to spi_ok", complete == 8u);
    bench.verdict("and none is faster than its wire (64 frames of eight SCK periods at the rate asked; letter g "
                  "times the wire)",
                  never_early == 8u);
    bench.verdict("the select is released after the polled transactions, the peer's never taken",
                  LoneCs::read_out() && CsPin::read_out());
}

// ===========================================================================
// d - the DMA engines
// ===========================================================================

/// One of letter d's exchanges, as a row: the path, the frames, the
/// width, the buffers' shape and the two seeds.
struct DmaRow {
    const char* name;
    Path path;
    uint8_t count;
    uint8_t cmd_len;
    bool wide;
    bool no_out;
    uint8_t offset;
    uint8_t seed_a;
    uint8_t seed_b;
};

void td_dma() {
    if (!need_peer()) {
        return;
    }
    // THE PAIR'S CEILING: the foreign client hears and answers exact at
    // every pattern to /8 = 6 MHz, and from /4 not while its own MISO
    // stands high (letter s) - so the exchanges run at /8, and the rate
    // this host's engines keep is letter g's, on the wire alone. The rows
    // run in order: the polled 16-bit block follows an asynchronous 8-bit
    // one and the polled 8-bit one an asynchronous 16-bit one, which is
    // what judges the frame step a polled request on the engines sets.
    const DmaRow rows[] = {
        {"128 frames", Path::engines, 128, 0, false, false, 0, 0x3C, 0xC3},
        {"command + 32", Path::engines, 33, 1, false, false, 0, 0x0B, 0x5A},
        {"64 polled", Path::engines_polled, 64, 0, false, false, 0, 0x99, 0x17},
        {"32 of 16 bits", Path::engines, 32, 0, true, false, 0, 0x42, 0x24},
        {"16-bit, odd buffers", Path::engines, 16, 0, true, false, 1, 0x24, 0x81},
        {"asynchronous 8-bit", Path::engines, 32, 0, false, false, 0, 0x61, 0x16},
        {"then POLLED 16-bit", Path::engines_polled, 24, 0, true, false, 0, 0x62, 0x26},
        {"asynchronous 16-bit", Path::engines, 24, 0, true, false, 0, 0x63, 0x36},
        {"then POLLED 8-bit", Path::engines_polled, 32, 0, false, false, 0, 0x64, 0x46},
        {"no out, 8-bit", Path::engines, 16, 0, false, true, 0, 0xFF, 0xA5},
        {"no out, 16-bit", Path::engines, 8, 0, true, true, 0, 0xFF, 0x3C},
    };
    constexpr uint8_t n = sizeof rows / sizeof rows[0];
    bool ok[n];
    uint32_t spi_irqs[n];
    uint32_t dma_irqs[n];
    for (uint8_t i = 0; i < n; ++i) {
        const DmaRow& w = rows[i];
        Exchange e{};
        e.path = w.path;
        e.rate = SpiClock::div8;
        e.count = w.count;
        e.cmd_len = w.cmd_len;
        e.wide = w.wide;
        e.no_out = w.no_out;
        e.offset = w.offset;
        e.seed_a = w.seed_a;
        e.seed_b = w.seed_b;
        e.pattern = w.no_out ? spilink::pattern_fixed : spilink::pattern_prbs;
        ok[i] = exchange_line(e, w.name);
        spi_irqs[i] = e.spi_entries;
        dma_irqs[i] = e.dma_entries;
    }
    bench.verdict("a 128-frame block through both engines, exact BOTH ways", ok[0]);
    bench.verdict("ONE DMA interrupt, no SPI one: the receive block's end proves the transmit's",
                  dma_irqs[0] == 1u && spi_irqs[0] == 0u);
    bench.verdict("with no transfer fault on either channel", DmaTx::faults() == 0u && DmaRx::faults() == 0u);
    bench.verdict("a command frame on the pump, 32 data frames on the engines: all 33 heard, the data read exact",
                  ok[1]);
    bench.verdict("a POLLED request on the engines completes inside start(), exact",
                  ok[2] && spi_irqs[2] == 0u);
    bench.verdict("16-bit frames ride the engines in half-word beats (one DMA interrupt), exact against a foreign "
                  "client",
                  ok[3] && spi_irqs[3] == 0u && dma_irqs[3] == 1u);
    bench.verdict("16-bit frames from an ODD buffer fall back to the pump (sixteen ISR entries), exact",
                  ok[4] && spi_irqs[4] == 16u);
    bench.verdict("a POLLED request on the engines moves its OWN width after an asynchronous one at the other, "
                  "both orders",
                  ok[5] && ok[6] && ok[7] && ok[8]);
    bench.verdict("a read with no out buffer clocks the 0xFF cell (heard so by the client) at 8 and 16 bits",
                  ok[9] && ok[10]);
    host_ready();
}

// ===========================================================================
// g - the rate on the wire
// ===========================================================================

void tg_wire() {
    // THE RATE ON THE WIRE inside a block, at the rates asked, the peer
    // deselected: the frames from tx_buf into rx_buf, both in SRAM - the
    // difference of a 256- and a 32-frame block (128 and 16 at 16 bits,
    // the buffer's half-words).
    dma_ready();
    console_drain();
    const uint32_t m0_div2 = frame_permille(SpiClock::div2, SpiMode::mode0, SpiDataSize::bits8, tx_buf, rx_buf, 256, 32);
    const uint32_t m0_div4 = frame_permille(SpiClock::div4, SpiMode::mode0, SpiDataSize::bits8, tx_buf, rx_buf, 256, 32);
    const uint32_t m0_div16 = frame_permille(SpiClock::div16, SpiMode::mode0, SpiDataSize::bits8, tx_buf, rx_buf, 256, 32);
    const uint32_t m0_div64 = frame_permille(SpiClock::div64, SpiMode::mode0, SpiDataSize::bits8, tx_buf, rx_buf, 256, 32);
    const uint32_t m3_div16 = frame_permille(SpiClock::div16, SpiMode::mode3, SpiDataSize::bits8, tx_buf, rx_buf, 256, 32);
    const uint32_t w16_div4 = frame_permille(SpiClock::div4, SpiMode::mode0, SpiDataSize::bits16, tx_buf, rx_buf, 128, 16);
    const uint32_t w16_div16 = frame_permille(SpiClock::div16, SpiMode::mode0, SpiDataSize::bits16, tx_buf, rx_buf, 128, 16);
    print(serial, "  a frame inside an engined block, buffers in SRAM, in thousandths of its SCK periods at the rate "
          "asked: mode 0 /2 ", m0_div2, ", /4 ", m0_div4, ", /16 ", m0_div16, ", /64 ", m0_div64, "; mode 3 /16 ",
          m3_div16, "; 16-bit /4 ", w16_div4, ", /16 ", w16_div16, crlf);
    bench.verdict("THE ENGINES RUN AT THE RATE ASKED: a frame in a block takes its SCK periods - /2 /4 /16 /64 "
                  "mode 0, /16 mode 3, /4 /16 16-bit - each -1 % to +3 %",
                  wire_ok(m0_div2) && wire_ok(m0_div4) && wire_ok(m0_div16) && wire_ok(m0_div64) &&
                      wire_ok(m3_div16) && wire_ok(w16_div4) && wire_ok(w16_div16));
    // And with no buffer named - the host's own cells, 2048 and 256 frames:
    // the transmit engine pours the 0xFF dummy, the receive engine writes
    // its discard cell.
    const uint32_t cell_div2 = frame_permille(SpiClock::div2, SpiMode::mode0, SpiDataSize::bits8);
    const uint32_t cell_div4 = frame_permille(SpiClock::div4, SpiMode::mode0, SpiDataSize::bits8);
    const uint32_t cell16_div4 = frame_permille(SpiClock::div4, SpiMode::mode0, SpiDataSize::bits16);
    print(serial, "  the same with no buffer named (the host's dummy and discard cells): /2 ", cell_div2, ", /4 ",
          cell_div4, "; 16-bit /4 ", cell16_div4, crlf);
    bench.verdict("and a block with no buffer named runs at the wire too: /2 and /4, 8 and 16 bits, each -1 % to "
                  "+3 % of its SCK periods",
                  wire_ok(cell_div2) && wire_ok(cell_div4) && wire_ok(cell16_div4));
    DmaHost::release();
    dma_host_live = false;
    host_ready();
}

// ===========================================================================
// e - the hardware CRC against a foreign CRC unit
// ===========================================================================

/// The chapter's CRC, bitwise over FRAMES of `bits` bits, each MSb first:
/// no reflection, init 0 (16.2.5). A frame of a byte buffer is one byte,
/// or two low-first.
uint16_t crc_reference(uint16_t poly, bool wide, const uint8_t* data, uint16_t frames) {
    const uint8_t width = wide ? 16u : 8u;
    uint32_t crc = 0;
    const uint32_t top = 1u << (width - 1u);
    const uint32_t mask = (1u << width) - 1u;
    for (uint16_t k = 0; k < frames; ++k) {
        const uint16_t f = wide ? static_cast<uint16_t>(data[2u * k] | (data[2u * k + 1u] << 8)) : data[k];
        for (uint8_t b = width; b-- > 0;) {
            const bool in = ((f >> b) & 1u) != 0u;
            const bool out = (crc & top) != 0u;
            crc = (crc << 1) & mask;
            if (in != out) {
                crc ^= poly;
            }
        }
    }
    return static_cast<uint16_t>(crc);
}

struct CrcRun {
    bool accepted = false;
    bool frames_ok = false;      ///< every frame came back within its budget
    bool stream_ok = false;      ///< and they were the peer's stream
    uint16_t tx_crc = 0, rx_crc = 0, frame = 0;
    uint16_t tx_ref = 0, rx_ref = 0, peer_ref = 0;
    bool crcerr = false;
    spilink::Report peer{};
    bool reported = false;
};

/// Eight frames to the peer with both CRC units on: this end by hand on
/// the resource (CRCNEXT after the last data frame, the CRC frame read
/// back), the peer's crc run with `peer_poly`.
CrcRun crc_exchange(bool wide, uint16_t poly, uint16_t peer_poly, uint8_t seed_a, uint8_t seed_b) {
    CrcRun c{};
    constexpr uint16_t n = 8;
    spilink::Params a{};
    a.cfg = spilink::Cfg{.apply = 1, .mode = 0, .dord = 0, .regime = spilink::regime_buffer_wait};
    a.count = n;
    a.ms = 400;
    a.seed_a = seed_a;
    a.seed_b = seed_b;
    a.pattern = spilink::pattern_prbs;
    a.spare = static_cast<uint8_t>(spilink::spare_crc | (wide ? spilink::spare_wide : 0u));
    a.aux16 = peer_poly;
    c.accepted = peer_act(Op::exchange, a);
    if (!c.accepted) {
        return c;
    }
    const SpiDataSize bits = wide ? SpiDataSize::bits16 : SpiDataSize::bits8;
    const uint8_t w = wide ? 2u : 1u;
    uint8_t sent[2 * n], got[2 * n];
    {
        spilink::Stream out(a.pattern, a.seed_a);
        for (uint16_t i = 0; i < n * w; ++i) {
            sent[i] = out.next();
        }
    }
    settle();
    // The resource by hand: a host in mode 0 at /64 with the CRC on, the
    // pads to the peripheral.
    dma_host_live = false;
    bus_ao_live = false;
    Pfic::disable(S::irq());
    Host::release();
    S::bus_clock(true);
    S::reset();
    (void)S::configure({.role = SpiRole::host, .mode = SpiMode::mode0, .clock = SpiClock::div64,
                        .bits = bits, .crc = true, .crc_polynomial = poly});
    SckPin::function();
    MosiPin::function();
    MisoPin::input();
    S::enable();
    S::flush_rx();
    CsPin::output(true);
    (void)delay_us(clock, 20);
    CsPin::clear();
    link_hold();
    // Each frame written, spun on RXNE, read back, a gap behind it for the
    // client's run; CRCNEXT right after the last data frame is written;
    // then the CRC frame itself comes back.
    bool frames = true;
    for (uint16_t i = 0; i < n; ++i) {
        const uint16_t v = wide ? static_cast<uint16_t>(sent[2u * i] | (sent[2u * i + 1u] << 8)) : sent[i];
        S::data(bits, v);
        if (i == n - 1u) {
            S::crc_next();
        }
        uint32_t spins = 200'000u;
        while (!S::rxne() && spins-- != 0u) {
        }
        frames = frames && spins != 0u;
        const uint16_t r = S::data(bits);
        got[w * i] = static_cast<uint8_t>(r);
        if (wide) {
            got[2u * i + 1u] = static_cast<uint8_t>(r >> 8);
        }
        (void)delay_us(clock, spilink::gap_us);
    }
    uint32_t spins = 200'000u;
    while (!S::rxne() && spins-- != 0u) {
    }
    frames = frames && spins != 0u;
    c.frame = S::data(bits);
    spins = 200'000u;
    while (S::busy() && spins-- != 0u) {
    }
    link_hold();
    CsPin::set();
    c.tx_crc = S::tx_crc();
    c.rx_crc = S::rx_crc();
    c.crcerr = S::crc_error();
    (void)S::disable();
    S::reset();
    c.frames_ok = frames;
    uint8_t want[2 * n];
    for (uint16_t i = 0; i < n * w; ++i) {
        want[i] = spilink::pattern_value(a.pattern, a.seed_b, i);
    }
    c.stream_ok = true;
    for (uint16_t i = 0; i < n * w; ++i) {
        c.stream_ok = c.stream_ok && got[i] == want[i];
    }
    c.tx_ref = crc_reference(poly, wide, sent, n);
    c.rx_ref = crc_reference(poly, wide, got, n);
    c.peer_ref = crc_reference(peer_poly, wide, want, n);
    (void)link_command_mode();
    c.reported = peer_report(c.peer);
    return c;
}

void print_crc(const char* label, const CrcRun& c) {
    print(serial, "  ", label, ": TXCRCR ", hex(c.tx_crc), " (bitwise ", hex(c.tx_ref), "), RXCRCR ", hex(c.rx_crc),
          " (bitwise over what came ", hex(c.rx_ref), "), the CRC frame read ", hex(c.frame), " (the peer's, bitwise ",
          hex(c.peer_ref), "), CRCERR here ", c.crcerr ? "SET" : "clear", "; the peer heard ", c.peer.count,
          " mism ", c.peer.mism, " flags ", hex(c.peer.flags), " (CRCERR there ",
          (c.peer.flags & spilink::report_crcerr) != 0u ? "SET" : "clear", ", its RXCRCR low byte ", hex(c.peer.aux3),
          ")", crlf);
}

void te_crc() {
    if (!need_peer()) {
        return;
    }
    const CrcRun c8 = crc_exchange(false, 0x0007u, 0x0007u, 0x31, 0xA7);
    print_crc("CRC-8 (0x07), eight frames", c8);
    const CrcRun c16 = crc_exchange(true, 0x8005u, 0x8005u, 0x52, 0x2D);
    print_crc("CRC-16 (0x8005), eight 16-bit frames", c16);
    auto agree = [](const CrcRun& c) {
        return c.accepted && c.frames_ok && c.stream_ok && c.reported &&
               (c.peer.flags & spilink::report_extended) != 0u && c.peer.count == 8u && c.peer.mism == 0u;
    };
    bench.verdict("the peer served both CRC exchanges (report_extended) and the frames went both ways exact",
                  agree(c8) && agree(c16));
    bench.verdict("THE HARDWARE CRC IS THE ARITHMETIC at both widths: TXCRCR over what went, RXCRCR over what "
                  "came",
                  c8.tx_crc == c8.tx_ref && c8.rx_crc == c8.rx_ref && c16.tx_crc == c16.tx_ref &&
                      c16.rx_crc == c16.rx_ref);
    bench.verdict("A FOREIGN CRC UNIT AGREES: the CRC frame read is the peer's bitwise CRC, CRCERR down here",
                  c8.frame == c8.peer_ref && !c8.crcerr && c16.frame == c16.peer_ref && !c16.crcerr);
    bench.verdict("and the peer checked THIS end's CRC frame and found it right (no CRCERR there)",
                  (c8.peer.flags & spilink::report_crcerr) == 0u && (c16.peer.flags & spilink::report_crcerr) == 0u);
    // THE CHECK IS REAL: the two units on different polynomials - each
    // end's CRC frame is then wrong for the other, and both must say so.
    const CrcRun bad = crc_exchange(false, 0x0007u, 0x0031u, 0x44, 0x9A);
    print_crc("CRC-8 here 0x07, the peer 0x31", bad);
    bench.verdict("a POLYNOMIAL MISMATCH is caught at BOTH ends: CRCERR set here and in the peer's report",
                  bad.accepted && bad.reported && bad.crcerr && (bad.peer.flags & spilink::report_crcerr) != 0u);
    host_ready();
    bench.verdict("the command channel still works after the CRC", command(Op::ping));
}

// ===========================================================================
// f - the kernel
// ===========================================================================

namespace kl {

using SpiArb = SpiBus<Host, P, 4>;

uint8_t out_a[8];
uint8_t out_b[8];
uint8_t in_a[8];

class Probe {
public:
    using Event = std::variant<SpiDone, SleepVote>;
    static inline EventQueue<Event, 12, P> queue;
    static inline uint8_t replies[8];
    static inline uint8_t n = 0;
    static inline uint8_t rejected = 0;
    static inline uint8_t votes = 0;
    static inline bool last_vote = false;

    static void init() { clear_tally(); }
    static void clear_tally() {
        n = 0;
        rejected = 0;
        votes = 0;
        last_vote = false;
    }

    static void dispatch(const Event& e) {
        brio::match(
            e,
            [](const SpiDone& d) {
                if (n < 8u) {
                    replies[n] = d.status;
                }
                ++n;
                if (d.status == spi_rejected) {
                    ++rejected;
                }
            },
            [](const SleepVote& v) {
                ++votes;
                last_vote = v.ok;
            });
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

/// The peer letter holds the select down across its queued
/// transactions by hand (the peer answers ONE burst per select
/// window), and runs them at the link's rate; the single-board letter
/// selects the lone pad.
bool hand_cs = false;
SpiClock rate = SpiClock::div16;

Host::Request request(const uint8_t* tx, uint8_t* rx, bool polled) {
    Host::Request r{};
    r.cs = hand_cs ? PinRef{} : LoneCs::ref();
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
    r.cmd_len = 0;
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = 8;
    r.mode = SpiMode::mode0;
    r.clock = rate;
    r.polled = polled;
    r.reply = reply_to<Probe, SpiDone>();
    return r;
}

}  // namespace kl

void tf_kernel() {
    host_ready();
    kl::BusKernel::init_all();
    bus_ao_live = true;

    fill_pattern(kl::out_a, 8, 0x01);
    fill_pattern(kl::out_b, 8, 0x02);
    for (uint8_t i = 0; i < 4u; ++i) {
        post<kl::SpiArb>(kl::request(kl::out_a, (i == 3u) ? kl::in_a : nullptr, i == 1u));
    }
    kl::pump_until(4, 300);
    print(serial, "  four queued transactions (one polled): replies ", kl::Probe::n, " [",
          kl::Probe::replies[0], " ", kl::Probe::replies[1], " ", kl::Probe::replies[2], " ",
          kl::Probe::replies[3], "]", crlf);
    bench.verdict("four transactions through SpiBus, four replies - util/spi_bus.hpp and "
                  "util/bus_master.hpp unchanged on this architecture",
                  kl::Probe::n == 4u);
    bench.verdict("... every one spi_ok, ISR-pumped and polled interleaved on one bus (what came back is the "
                  "peer letter q's to judge)",
                  kl::Probe::replies[0] == spi_ok && kl::Probe::replies[1] == spi_ok &&
                      kl::Probe::replies[2] == spi_ok && kl::Probe::replies[3] == spi_ok);

    kl::Probe::clear_tally();
    for (uint8_t i = 0; i < 6u; ++i) {
        post<kl::SpiArb>(kl::request(kl::out_b, nullptr, false));
    }
    kl::pump_until(6, 300);
    print(serial, "  six posted into a four-deep queue: replies ", kl::Probe::n, ", rejected ",
          kl::Probe::rejected, crlf);
    bench.verdict("the arbiter rejects what it cannot queue, immediately", kl::Probe::rejected != 0u);
    bench.verdict("... and every request is still answered exactly once", kl::Probe::n == 6u);

    kl::Probe::clear_tally();
    post<kl::SpiArb>(PrepareSleep{
        .depth = SleepDepth::standby,
        .reply = reply_to<kl::Probe, SleepVote>(),
    });
    kl::pump();
    bench.verdict("an IDLE bus votes for the sleep", kl::Probe::votes == 1u && kl::Probe::last_vote);
    kl::Probe::clear_tally();
    post<kl::SpiArb>(kl::request(kl::out_a, nullptr, false));
    post<kl::SpiArb>(PrepareSleep{
        .depth = SleepDepth::standby,
        .reply = reply_to<kl::Probe, SleepVote>(),
    });
    kl::pump();
    kl::pump_until(2, 100);
    print(serial, "  the vote from a BUSY bus: ", kl::Probe::votes, " vote(s), last ",
          kl::Probe::last_vote ? "yes" : "no", crlf);
    bench.verdict("a BUSY bus votes against it", kl::Probe::votes == 1u && !kl::Probe::last_vote);
    bus_ao_live = false;
    bench.verdict("the peer's select stayed high through the letter", CsPin::read_out());
}

// ===========================================================================
// h - the stalled exit, wireless
// ===========================================================================
//
// spi_stalled is the polled path's answer to a flag that never comes - a
// clock that stopped, a block that is dead - spent from ONE budget for the
// whole transaction (poll_budget: the frames' cycles plus two, in poll
// turns of several instructions each). Provoked here with SPI1 HELD IN
// RESET under a polled duplex request: its registers ignore every write,
// RXNE never rises. The lone select (PC3) frames it, the peer's deselected.

void th_stalled() {
    host_ready();
    fill_pattern(tx_buf, 16, 0x5A);
    rcc()->PB2PRSTR |= rcc_pb2_spi1;
    const uint32_t t0 = cycles_now();
    const uint8_t st = host_xfer(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode0,
                                 SpiClock::div16, SpiDataSize::bits8, true);
    const uint32_t took = cycles_now() - t0;
    rcc()->PB2PRSTR &= ~rcc_pb2_spi1;
    const bool rec = Host::recover();
    const uint8_t after = host_xfer(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode0,
                                    SpiClock::div16, SpiDataSize::bits8, true);
    print(serial, "  a polled request of 16 at /16 with the block held in reset: status ", st, " (spi_stalled ",
          spi_stalled, ") after ", took, " cycles (the wire's 2048); recover() ", rec ? 1 : 0,
          ", the next request ", after, crlf);
    bench.verdict("A POLLED REQUEST OVER A BLOCK HELD IN RESET ends spi_stalled - the budget runs out, the call "
                  "returns - within ten milliseconds (480 000 cycles), and after recover() the next request is "
                  "spi_ok",
                  st == spi_stalled && took < 480'000u && rec && after == spi_ok);
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
    }
    bench.verdict("ident comes back and it IS spi_peer (the sanity byte), the CH32X035's (kind 0x07)",
                  got && d.sanity == spilink::ident_sanity && spilink::peer_kind(d.version) == spilink::peer_ch32x035);

    uint8_t pings = 0;
    for (uint8_t i = 0; i < 10; ++i) {
        if (command(Op::ping)) {
            ++pings;
        }
    }
    print(serial, "  ", pings, " of 10 pings answered at ", Host::sck_hz(link_clock) / 1000u,
          " kHz SCK, one frame per chip-select window, the answers in mode 0", crlf);
    bench.verdict("the channel is steady over ten frames", pings == 10u);
}

// ===========================================================================
// o - the matrix against the peer
// ===========================================================================

void to_peer_matrix() {
    if (!need_peer()) {
        return;
    }
    // EVERY MODE, MSb first, on the peer's DMA serve and on its polled run:
    // what this host SENDS judged by the foreign client's report, what it
    // READS judged here against the client's stream.
    uint8_t good = 0;
    for (uint8_t serve = 0; serve < 2u; ++serve) {
        for (uint8_t m = 0; m < 4u; ++m) {
            Exchange e{};
            e.cfg.mode = m;
            e.host_mode = static_cast<SpiMode>(m);
            e.spare = serve != 0u ? spilink::spare_polled_pump : 0u;
            e.seed_a = static_cast<uint8_t>(0x13u + m + 16u * serve);
            e.seed_b = static_cast<uint8_t>(0x57u + m + 16u * serve);
            char label[] = "mode 0 MSb, the peer's dma   ";
            label[5] = static_cast<char>('0' + m);
            if (serve != 0u) {
                label[23] = 'r';
                label[24] = 'u';
                label[25] = 'n';
            }
            if (exchange_line(e, label)) {
                ++good;
            }
        }
    }
    bench.verdict("all four modes exact in BOTH directions between TWO CHIPS, on both of the client's serves",
                  good == 8u);

    // LSb FIRST HERE against the client's MSb first: each end checks the
    // EXACT bit-reverse of what the other sent.
    uint8_t rev = 0;
    for (uint8_t m = 0; m < 4u; ++m) {
        Exchange e{};
        e.cfg.mode = m;
        e.host_mode = static_cast<SpiMode>(m);
        e.host_lsb = true;
        e.seed_a = static_cast<uint8_t>(0x21u + m);
        e.seed_b = static_cast<uint8_t>(0x65u + m);
        char label[] = "mode 0 LSb here, MSb there";
        label[5] = static_cast<char>('0' + m);
        if (exchange_line(e, label)) {
            ++rev;
        }
    }
    bench.verdict("LSb first against an MSb-first client: an EXACT TWO-WAY BIT REVERSAL in every mode",
                  rev == 4u);

    // LSb FIRST AT BOTH ENDS: the peer's client cannot ANSWER LSb first
    // (16.3.1, measured on its die) and refuses it, saying so; it LISTENS
    // LSb first, and that is the judgement.
    Exchange both{};
    both.cfg.dord = 1;
    both.host_lsb = true;
    spilink::Report rr{};
    Verify vv{};
    (void)exchange_exact(both, vv, rr);
    const bool refused = (rr.flags & spilink::report_cfg_failed) != 0u;
    print(serial, "  LSb first at both ends, answering: the peer ", refused ? "REFUSED it (report_cfg_failed)" : "ran it",
          ", flags ", hex(rr.flags), crlf);
    uint8_t heard = 0;
    for (uint8_t m = 0; m < 4u; ++m) {
        Exchange e{};
        e.cfg.mode = m;
        e.cfg.dord = 1;
        e.host_mode = static_cast<SpiMode>(m);
        e.host_lsb = true;
        e.no_in = true;
        e.spare = spilink::spare_receive_only;
        e.seed_a = static_cast<uint8_t>(0x35u + m);
        char label[] = "mode 0 LSb both ends, the client listening";
        label[5] = static_cast<char>('0' + m);
        if (exchange_line(e, label)) {
            ++heard;
        }
    }
    bench.verdict("LSb first at both ends: the peer refuses to ANSWER so (its die cannot) and HEARS every frame "
                  "exact, every mode",
                  refused && heard == 4u);
    bench.verdict("and the command channel still works right after all of that", command(Op::ping));
}

// ===========================================================================
// p - the rate ladder against the peer
// ===========================================================================

void tp_peer_rates() {
    if (!need_peer()) {
        return;
    }
    // The BR codes, from the command rate up, on the polled host (8
    // frames) and on the engines (32), against the peer's DMA serve and
    // its polled run, and this host's TRANSMIT-ONLY paths (no in buffer)
    // against the peer listening on its receive engine - every rung
    // printed.
    struct Rung {
        Path path;
        uint8_t spare;
        bool no_in;
    };
    const Rung rungs[6] = {{Path::polled, 0u, false},
                           {Path::engines, 0u, false},
                           {Path::polled, spilink::spare_polled_pump, false},
                           {Path::engines, spilink::spare_polled_pump, false},
                           {Path::polled, spilink::spare_receive_only, true},
                           {Path::engines, spilink::spare_receive_only, true}};
    const char* names[6] = {"polled host / peer DMA     ", "engined host / peer DMA    ",
                            "polled host / peer run     ", "engined host / peer run    ",
                            "polled write / peer listens", "engined write / peer listens"};
    uint8_t broke_at[6] = {8, 8, 8, 8, 8, 8};   // the slowest code that broke, 8 = none
    for (uint8_t s = 0; s < 6u; ++s) {
        print(serial, "  ", names[s], ":");
        for (uint8_t code = 8; code-- > 0;) {
            Exchange e{};
            e.rate = static_cast<SpiClock>(code);
            e.path = rungs[s].path;
            e.count = rungs[s].path == Path::polled ? 8u : 32u;
            e.spare = rungs[s].spare;
            e.no_in = rungs[s].no_in;
            e.seed_a = static_cast<uint8_t>(0x21u + code + 8u * s);
            e.seed_b = static_cast<uint8_t>(0x84u + code + 8u * s);
            Verify v{};
            spilink::Report r{};
            const bool ok = exchange_exact(e, v, r);
            print(serial, " /", asked_divisor(e.rate), ok ? " ok" : " BAD");
            if (!ok) {
                print(serial, "(h", v.mism, " c", r.mism, "/", r.count, " f", hex(r.flags), " t", r.aux1, ")");
                if (broke_at[s] == 8u) {
                    broke_at[s] = code;
                }
            }
        }
        print(serial, crlf);
    }
    // THE PAIR'S CEILING (letter s): the foreign client is exact at every
    // pattern to /8 = 6 MHz, and from /4 not while its own MISO stands
    // high; listening, with its MISO floating, to /4 = 12 MHz. The rungs
    // above are printed; the verdicts stop at the ceiling.
    const auto held_to = [](uint8_t broke, uint8_t code) { return broke == 8u || broke < code; };
    bench.verdict("BOTH HOST PATHS exact both ways against the peer's DMA serve at every code to /8 = 6 MHz, the "
                  "pair's ceiling",
                  held_to(broke_at[0], 2) && held_to(broke_at[1], 2));
    bench.verdict("this host's TRANSMIT-ONLY paths reach a listening client exact at every code to /4 = 12 MHz",
                  held_to(broke_at[4], 1) && held_to(broke_at[5], 1));
    // The peer's polled run is ITS ceiling - the CH32X035 client's run,
    // each answer reloaded from code fetched from the flash on the TXE of
    // the frame before it - printed above: behind this polled host's
    // gaps, and behind the engines' frames back to back.
    bench.verdict("and against the peer's polled run to /16 behind this polled host, to /32 behind the engines' "
                  "back-to-back frames (its ceiling)",
                  held_to(broke_at[2], 3) && held_to(broke_at[3], 4));
    (void)link_command_mode();
    bench.verdict("the command channel survives the climb", command(Op::ping));
}

// ===========================================================================
// q - THE KERNEL against the peer
// ===========================================================================

void tq_peer_kernel() {
    if (!need_peer()) {
        return;
    }
    // The peer answers ONE burst of four x eight characters, so the
    // letter holds the select down across the four arbitrated
    // transactions (kl::hand_cs) at the link's rate.
    constexpr uint16_t total = 4u * 8u;
    spilink::Params a{};
    a.cfg = spilink::Cfg{.apply = 1, .mode = 0, .dord = 0, .regime = spilink::regime_buffer_wait};
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
    kl::hand_cs = true;
    kl::rate = link_clock;
    bus_ao_live = true;
    settle();
    Host::prime(SpiMode::mode0, link_clock);
    CsPin::clear();
    link_hold();
    for (uint8_t i = 0; i < 4u; ++i) {
        post<kl::SpiArb>(kl::request(qtx[i], qrx[i], i == 2u));
    }
    kl::pump_until(4, 800);
    link_hold();
    CsPin::set();
    bus_ao_live = false;
    kl::hand_cs = false;
    kl::rate = SpiClock::div16;

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
    print(serial, "  kernel: ", kl::Probe::n, " replies; host mism=", mism, "; the peer read ", r.count, " of ",
          total, " with mism=", r.mism, " flags=", hex(r.flags), crlf);
    bench.verdict("FOUR TRANSACTIONS QUEUED FROM ONE DISPATCH (one polled) come back in order, spi_ok, the bytes on "
                  "the wire what each lent - read back by A SECOND CHIP",
                  replies_ok && mism == 0u && rep && r.count == total && r.mism == 0u);
    (void)link_command_mode();
    bench.verdict("the command channel is back after the kernel's window", command(Op::ping));
}

// ===========================================================================
// r - THE ROLES INVERT: this chip as the CLIENT on its hardware select
// ===========================================================================

/// SPI1's own pads carry the wire, so the client role is the SAME four:
/// MISO driven, SCK and MOSI inputs, and NSS - PC1 - the HARDWARE select
/// (SSM clear), which the peer drives low for its whole burst: a frame is
/// shifted only while the pad is low, so the peer's pads changing hands
/// during its lead-in are no clock edge here.
using LinkClient = SpiClient<1>;

uint8_t crx[64];

struct ClientBurst {
    bool accepted = false;
    bool admitted = false;   ///< SpiClient::init took the configuration
    bool ran = false;
    uint16_t frames = 0;     ///< received
    uint16_t mism = 0;       ///< bytes received that were not the peer's stream
    uint8_t idx = 0xFF, got = 0, want = 0;
    spilink::Report peer{};
    bool reported = false;
};

/// One burst with this chip as the client in `mode`, `lsb`, `wide`, 12
/// frames from the peer's host at its HCLK/32 (1.5 MHz).
ClientBurst client_burst(uint8_t mode, bool lsb, bool wide, uint8_t seed_a, uint8_t seed_b) {
    ClientBurst out{};
    const uint16_t count = wide ? 8u : 12u;
    const LinkClient::Config cfg{.mode = static_cast<SpiMode>(mode),
                                 .bits = wide ? SpiDataSize::bits16 : SpiDataSize::bits8,
                                 .lsb_first = lsb,
                                 .nss = SpiNss::hardware_input,
                                 .drive_output = true};
    out.admitted = LinkClient::config_served(cfg);
    if (!out.admitted) {
        return out;   // refused before a word goes on the wire
    }
    spilink::Params a{};
    a.count = count;
    a.ms = 500;
    a.seed_a = seed_a;
    a.seed_b = seed_b;
    a.pattern = spilink::pattern_prbs;
    a.aux8 = 40;    // lead-in ms: time for this end to become a client
    a.aux16 = 32;   // the peer's SCK division of its own 48 MHz: 1.5 MHz
    a.cfg.apply = 1;
    a.cfg.mode = mode;
    a.cfg.dord = lsb ? 1 : 0;
    a.spare = wide ? spilink::spare_wide : 0u;
    out.accepted = peer_act(Op::host_burst, a);
    if (!out.accepted) {
        return out;
    }
    dma_host_live = false;
    bus_ao_live = false;
    Host::release();
    CsPin::input(PinPull::up);
    if (!LinkClient::init(clock, cfg)) {
        out.admitted = false;
        return out;
    }
    Pfic::disable(S::irq());   // polled: the vector stays with the host's pump
    // ONE FRAME AHEAD: one buffer on this silicon, so the first answer
    // goes in before the host's clock can arrive (the select still high)
    // and every frame the shifter takes frees the buffer for the next.
    spilink::Stream answers(a.pattern, a.seed_b);
    auto next_frame = [&]() -> uint16_t {
        const uint8_t lo = answers.next();
        return wide ? static_cast<uint16_t>(lo | (answers.next() << 8)) : lo;
    };
    LinkClient::enable(next_frame());
    uint16_t queued = 1;
    const uint32_t t0 = Ticker::millis();
    const uint8_t w = wide ? 2u : 1u;
    while (out.frames < count && static_cast<uint16_t>(out.frames * w) < sizeof crx && Ticker::millis() - t0 < 300u) {
        if (queued < count && queued <= out.frames + 1u && LinkClient::writable()) {
            LinkClient::write(next_frame());
            ++queued;
        }
        const auto v = LinkClient::poll();
        if (!v) {
            continue;
        }
        crx[w * out.frames] = static_cast<uint8_t>(*v);
        if (wide) {
            crx[w * out.frames + 1u] = static_cast<uint8_t>(*v >> 8);
        }
        ++out.frames;
    }
    out.ran = true;
    // The far host closes its select, then this end lets the pads go.
    while (!CsPin::read() && Ticker::millis() - t0 < 600u) {
    }
    (void)LinkClient::disable();
    LinkClient::release();
    for (uint16_t i = 0; i < out.frames * w; ++i) {
        const uint8_t exp = spilink::pattern_value(a.pattern, a.seed_a, i);
        if (crx[i] != exp) {
            if (out.mism == 0) {
                out.idx = static_cast<uint8_t>(i);
                out.got = crx[i];
                out.want = exp;
            }
            ++out.mism;
        }
    }
    settle_ms(20);
    (void)link_command_mode();
    out.reported = peer_report(out.peer);
    return out;
}

void print_burst(const char* label, const ClientBurst& c, uint16_t count) {
    print(serial, "  ", label, ": ");
    if (!c.admitted) {
        print(serial, "REFUSED by SpiClient (spi_client_mode_served)", crlf);
        return;
    }
    print(serial, "received ", c.frames, "/", count, " mism ", c.mism);
    if (c.mism != 0u) {
        print(serial, " (idx ", c.idx, " got ", hex(c.got), " exp ", hex(c.want), ")");
    }
    print(serial, "; the peer as HOST read our answers ", c.peer.count, "/", count, " mism ", c.peer.mism);
    if (c.peer.mism != 0u) {
        print(serial, " (idx ", c.peer.idx, " got ", hex(c.peer.got), " exp ", hex(c.peer.exp), ")");
    }
    print(serial, " flags ", hex(c.peer.flags), crlf);
}

bool burst_exact(const ClientBurst& c, uint16_t count) {
    return c.admitted && c.accepted && c.ran && c.frames == count && c.mism == 0u && c.reported &&
           c.peer.count == count && c.peer.mism == 0u;
}

void tr_peer_client() {
    if (!need_peer()) {
        return;
    }
    // Modes 0 and 3, MSb first: the modes this part's client is served in.
    uint8_t exact = 0;
    for (uint8_t m = 0; m < 4u; m = static_cast<uint8_t>(m + 3u)) {
        const ClientBurst c = client_burst(m, false, false, static_cast<uint8_t>(0x2C + m), static_cast<uint8_t>(0x71 + m));
        char label[] = "mode 0 MSb";
        label[5] = static_cast<char>('0' + m);
        print_burst(label, c, 12);
        exact = static_cast<uint8_t>(exact + (burst_exact(c, 12) ? 1u : 0u));
    }
    bench.verdict("THIS CHIP AS CLIENT ON ITS HARDWARE NSS, modes 0 and 3: every frame of a foreign host exact, its "
                  "answers exact FROM THE FIRST FRAME (the preload)",
                  exact == 2u);
    // Modes 1 and 2: refused by the driver before anything is commanded.
    uint8_t refused = 0;
    for (uint8_t m = 1; m < 3u; ++m) {
        const ClientBurst c = client_burst(m, false, false, 0x11, 0x22);
        char label[] = "mode 0 MSb";
        label[5] = static_cast<char>('0' + m);
        print_burst(label, c, 12);
        refused = static_cast<uint8_t>(refused + (!c.admitted ? 1u : 0u));
    }
    bench.verdict("modes 1 and 2 are REFUSED by SpiClient on this part (spi_client_mode_served: measured served "
                  "neither way)",
                  device::part != Ch32Part::v006 || refused == 2u);
    // LSb first, modes 0 and 3: the peer's host clocks LSb first too.
    uint8_t lsb_exact = 0;
    for (uint8_t m = 0; m < 4u; m = static_cast<uint8_t>(m + 3u)) {
        const ClientBurst c = client_burst(m, true, false, static_cast<uint8_t>(0x3D + m), static_cast<uint8_t>(0x82 + m));
        char label[] = "mode 0 LSb";
        label[5] = static_cast<char>('0' + m);
        print_burst(label, c, 12);
        lsb_exact = static_cast<uint8_t>(lsb_exact + (burst_exact(c, 12) ? 1u : 0u));
    }
    bench.verdict("and LSb first, both ends agreeing, in modes 0 and 3: exact both ways", lsb_exact == 2u);
    // 16-bit frames, mode 3.
    const ClientBurst w = client_burst(3, false, true, 0x4E, 0x93);
    print_burst("mode 3 MSb 16-bit", w, 8);
    bench.verdict("and 16-bit frames in mode 3: exact both ways, the peer serving the extended host_burst",
                  burst_exact(w, 8) && (w.peer.flags & spilink::report_extended) != 0u);
    (void)link_command_mode();
    bench.verdict("the command channel is back after the role swaps", command(Op::ping));
}

// ===========================================================================
// m - MODF from a second chip
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
    dma_host_live = false;
    bus_ao_live = false;
    Host::release();
    Pfic::disable(S::irq());
    S::bus_clock(true);
    S::reset();
    (void)S::configure({.role = SpiRole::host, .clock = link_clock, .nss = SpiNss::hardware_input});
    // The NSS pad an INPUT with its pull-up - the multi-host arrangement's
    // pad (16.2.1) - and nothing else of the block on a pad.
    CsPin::input(PinPull::up);
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
    (void)link_command_mode();
    return d;
}

void tm_peer_modf() {
    if (!need_peer()) {
        return;
    }
    uint8_t accepted = 0, raised = 0, demoted = 0, first = 0;
    for (uint8_t k = 0; k < 4u; ++k) {
        const Demotion d = demote_once();
        print(serial, "  pulse ", k, ": MODF ", d.at_enable ? "ALREADY SET" : "clear", " at the enable, ",
              d.modf ? "SET" : "never", " ", d.t_modf, " ms later, CTLR1 ", hex(d.ctlr1),
              ", the pad high again from ", d.t_high, " ms; STATR ", hex(d.st_before),
              " before the clearing sequence, ", hex(d.st_after), " after it; cleared by sequence ", d.sequences,
              crlf);
        accepted = static_cast<uint8_t>(accepted + (d.accepted ? 1u : 0u));
        raised = static_cast<uint8_t>(raised + (d.modf && !d.at_enable && d.t_modf >= 20u ? 1u : 0u));
        demoted = static_cast<uint8_t>(demoted + (d.modf && (d.ctlr1 & (spi_mstr | spi_spe)) == 0u ? 1u : 0u));
        first = static_cast<uint8_t>(first + (d.sequences == 1u ? 1u : 0u));
    }
    bench.verdict("the peer accepted four ss_pulses", accepted == 4u);
    bench.verdict("MODF comes up when a SECOND CHIP pulls this host's NSS low (16.2.7), and not before - four of "
                  "four",
                  raised == 4u);
    bench.verdict("and it demotes the host: MSTR and SPE cleared by the hardware, four of four", demoted == 4u);
    bench.verdict("16.2.7's sequence, a STATR read then a CTLR1 write, clears MODF once the pad is back high, four "
                  "of four",
                  first == 4u);
    bench.verdict("the command channel is back after the demotions", command(Op::ping));
}

// ===========================================================================
// s - what bounds the foreign client's rate (a diagnostic, OUTSIDE z)
// ===========================================================================

/// Does driving pad A move pad B? A driven both ways as a GPIO, B read
/// with its pull set AGAINST the driven level: a wire beats a pull, a
/// floating pad merely echoing its neighbour does not.
template <typename A, typename B>
bool joined() {
    B::input(PinPull::down);
    A::output(true);
    (void)delay_us(clock, 5);
    const bool high = B::read();
    B::input(PinPull::up);
    A::clear();
    (void)delay_us(clock, 5);
    const bool low = !B::read();
    A::release();
    B::release();
    return high && low;
}

/// /8, /4 and /2, sixteen frames on this host's engines against the peer's
/// DMA serve, one property varied at a time: what the client's MISO does
/// (quiet low, quiet high, a toggle every bit with MOSI quiet), the mode,
/// the client listening alone (MISO not driven), the client's polled run.
/// The peer's console sets what its MISO pad does in a listening serve
/// (f floating, l a GPIO low, h a GPIO high). First the wires: no two of
/// the link's pads joined, and the MISO wire's load, the peer dark.
void ts_peer_ceiling() {
    if (!need_peer()) {
        return;
    }
    (void)link_command_mode();
    Host::release();
    print(serial, "  pads joined (MOSI-MISO, SCK-MISO, NSS-MISO): ", joined<MosiPin, MisoPin>(), " ",
          joined<SckPin, MisoPin>(), " ", joined<CsPin, MisoPin>());
    MisoPin::input(PinPull::up);
    (void)delay_us(clock, 20);
    const bool up = MisoPin::read();
    MisoPin::input(PinPull::down);
    (void)delay_us(clock, 20);
    const bool down = MisoPin::read();
    MisoPin::release();
    print(serial, "; the MISO wire pulled up reads ", up, ", pulled down ", down, crlf);
    (void)link_command_mode();
    struct Case {
        const char* name;
        uint8_t pattern, seed_a, seed_b, mode, spare;
    };
    constexpr uint8_t fixed = spilink::pattern_fixed;
    const Case cases[] = {
        {"prbs both ways      ", spilink::pattern_prbs, 0x13, 0x57, 0, 0},
        {"MISO quiet low      ", fixed, 0x55, 0x00, 0, 0},
        {"MISO quiet high     ", fixed, 0x55, 0xFF, 0, 0},
        {"MOSI quiet, MISO 55 ", fixed, 0x00, 0x55, 0, 0},
        {"prbs mode 1         ", spilink::pattern_prbs, 0x23, 0x67, 1, 0},
        {"prbs mode 3         ", spilink::pattern_prbs, 0x43, 0x87, 3, 0},
        {"prbs, client listens", spilink::pattern_prbs, 0x63, 0xA7, 0, spilink::spare_receive_only},
        {"prbs, the peer's run", spilink::pattern_prbs, 0x73, 0xB7, 0, spilink::spare_polled_pump},
    };
    for (uint8_t code = 3; code-- > 0;) {
        print(serial, "  /", 2u << code, ":", crlf);
        for (const Case& c : cases) {
            Exchange e{};
            e.rate = static_cast<SpiClock>(code);
            e.path = Path::engines;
            e.count = 16;
            e.pattern = c.pattern;
            e.seed_a = c.seed_a;
            e.seed_b = c.seed_b;
            e.cfg.mode = c.mode;
            e.host_mode = static_cast<SpiMode>(c.mode);
            e.spare = c.spare;
            e.no_in = c.spare == spilink::spare_receive_only;
            Verify v{};
            spilink::Report r{};
            const bool ok = exchange_exact(e, v, r);
            print(serial, "    ", c.name, ok ? " exact" : " BAD  ", " client heard ", r.count, " mism ", r.mism,
                  " STATR ", hex(r.aux0), "; host mism ", v.mism, crlf);
        }
    }
}

// ===========================================================================
// x - the slip statistics against the peer (a diagnostic, OUTSIDE z)
// ===========================================================================

constexpr uint8_t slip_rounds = 10;

void tx_peer_slips() {
    if (!need_peer()) {
        return;
    }
    print(serial, "  each mode ", slip_rounds, " times, 8 frames at HCLK/256: bursts not exact, and the "
          "first bad frame at each end (host/client)", crlf);
    uint8_t total = 0;
    for (uint8_t m = 0; m < 4; ++m) {
        uint8_t slips = 0;
        print(serial, "  mode ", m, ":");
        for (uint8_t k = 0; k < slip_rounds; ++k) {
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
        print(serial, "  -> ", slips, " of ", slip_rounds, crlf);
        total = static_cast<uint8_t>(total + slips);
    }
    print(serial, "  ", total, " of ", 4u * slip_rounds, " bursts slipped (a probe: no verdict)", crlf);
    (void)link_command_mode();
    print(serial, "  the command channel afterwards: ", command(Op::ping) ? "answers" : "DEAD", crlf);
}

void banner() {
    print(serial, crlf, "test_ch32_spi - ", device::part_name, " SPI (RM ch. 16) against the CH32X035's spi_peer",
          crlf);
    print(serial, "  PC5 SCK, PC6 MOSI, PC7 MISO, PC1 the select to the peer's PA5, PA7, PA6, PA4; PC3 the "
                  "single-board letters' select (no wire)",
          crlf);
    bench.menu();
}

} // namespace

BRIO_CH32_VECTOR(systick_handler) { brio::Ticker::tick(); }
BRIO_CH32_VECTOR(usart1_handler) { (void)Serial::isr(); }

/// Whichever task this image carries owns SPI1's vector; a level nobody
/// owns is disarmed rather than looped on.
BRIO_CH32_VECTOR(spi1_handler) {
    spi_isr_entries = spi_isr_entries + 1u;
    if constexpr (image_engines) {
        if (dma_host_live) {
            if (DmaHost::isr()) {
                host_done = true;
            }
            return;
        }
    }
    if constexpr (image_kernel) {
        if (bus_ao_live) {
            if (Host::isr()) {
                brio::post<kl::SpiArb>(brio::TransferDone{Host::status()});
            }
            return;
        }
    }
    if constexpr (image_pump) {
        if (Host::isr()) {
            host_done = true;
        }
    } else {
        S::rxne_interrupt(false);
    }
}

[[gnu::always_inline]] inline void spi_dma() {
    dma_isr_entries = dma_isr_entries + 1u;
    if constexpr (image_engines) {
        if (dma_host_live && DmaHost::dma_isr()) {
            host_done = true;
        }
    }
}
BRIO_CH32_VECTOR(dma1_channel2_handler) { spi_dma(); }
BRIO_CH32_VECTOR(dma1_channel3_handler) { spi_dma(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();
    host_ready();   // PC1 high: the peer deselected from boot

    bench.letter('a', "the block, wireless: reset values, the control words, the refusals, "
                      "the enable protection measured", ta_block);
    bench.letter('b', "THE PUMP against the peer: four modes, 8 and 16 bits, a command phase", tb_pump);
    bench.letter('c', "the polled path at every rate, timed", tc_rates);
    bench.letter('d', "THE DMA ENGINES on channels 2 and 3 against the peer: one interrupt, the 16-bit beat, the "
                      "odd fallback, the polled step", td_dma);
    bench.letter('g', "THE RATE ON THE WIRE: a frame inside an engined block, the peer deselected", tg_wire);
    bench.letter('e', "THE HARDWARE CRC against a foreign CRC unit, 8 and 16 bits, a mismatch caught", te_crc);
    bench.letter('f', "THE KERNEL: SpiBus over SpiHost, the rejection, the votes", tf_kernel);
    bench.letter('h', "the stalled exit: a polled request over a block held in reset", th_stalled);
    bench.letter('n', "THE PEER: the spi_link command channel, ident, ten pings", tn_peer_link);
    bench.letter('o', "the matrix against the peer: four modes, both serves, both bit orders", to_peer_matrix);
    bench.letter('p', "the BR ladder against the peer, both host paths, both serves, the transmit-only path",
                 tp_peer_rates);
    bench.letter('q', "THE KERNEL against the peer: four transactions in one select window", tq_peer_kernel);
    bench.letter('r', "THE ROLES INVERT: this chip as the client on its hardware NSS", tr_peer_client);
    bench.letter('m', "MODF from a second chip", tm_peer_modf);
    bench.letter('s', "what bounds the foreign client's rate: its MISO, the mode, the width (no verdict)",
                 ts_peer_ceiling, false);
    bench.letter('x', "slip statistics against the peer: each mode ten times (no verdict)", tx_peer_slips, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL48" : "FAILED",
                    " tick=", tick_ok ? "STK" : "FAILED", brio::crlf);
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
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        brio::print(serial, "  stack: ", brio::stack_untouched(), " B never touched", brio::crlf);
        bench.prompt();
    }
}
