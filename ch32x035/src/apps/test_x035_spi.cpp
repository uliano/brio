// test_x035_spi - the reference bench suite for the SPI of the CH32X035
// (RM ch. 16): ch32x035/spi.hpp over the chapter - the block, the rate
// ladder on the wire, the high-speed read, the CRC, the refusals, the
// polled loops and the DMA engines on channels 2 and 3, the host's two ways
// to run a request (docs/design/spi-bus.md) - and,
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
//  - `z`, THE SINGLE-BOARD LETTERS (a, b, c, e, f, g, i), runnable with the far board
//    dark - on its console, or on its spi_peer, whose dark listener drives
//    nothing and is selected by nothing: PA4 is this end's GPIO select,
//    held HIGH through every one of them, and what goes out on SCK and
//    MOSI reaches a deselected client or a floating pad. MISO is read
//    floating and never judged.
//  - `y`, THE TWO-BOARD LETTERS (n, o, p, k, q, r, m, w), against the
//    CH32V006 running spi_peer on its 24 MHz crystal, commanded IN BAND
//    over the bus under test through avrxt/src/apps/spi_link.hpp - FAILING
//    loudly when nobody answers. That peer serves its answer windows in
//    mode 3, spi_link's alternate answer mode, and the rendezvous finds it.
//
// What is exercised, letter by letter:
//   a  the block, WIRELESS: table 16-2's reset values, the control words,
//      the enable protection MEASURED field by field, HSCR's "not
//      readable" read, ODEN, the select an SSOE host drives, the transmit
//      CRC against a bitwise reference, and the two engines - the polled
//      loops, a polled request on the engined host, the DMA engines - to
//      completion with nothing on MISO
//   i  THE ENGINE A REQUEST NAMES, wireless: one frame on the engines at /2, read
//      and written; THE COMMAND RULE, D/C sampled by the tick: a command in the
//      loop then the data on the engines, and a command alone on them; THE
//      REFUSALS, each one the engines cannot carry answered spi_rejected before
//      anything moves, and the next requests
//   g  WHEN A REQUEST ENDS, THE WIRE HAS ENDED, wireless: the SCK pad against
//      the mode's idle level at the flag the end is read from and right after
//      start() or dma_isr() answered, all four modes, /256 and /64, read and
//      write, both engines, both placements, with a select and through a null
//      one
//   b  THE RATE LADDER ON THE WIRE, the select high: a frame inside an
//      engined block timed (the difference of two block lengths) against
//      its SCK periods at every BR code, 8 and 16 bits; THE HIGH-SPEED
//      READ's rate at every code (16.3.1's FHCLK/(BR+2) against 16.3.8's
//      "/2 only"); and THE HOST'S COSTS - the engines' fixed cost, the
//      polled loops' frames and fixed cost - IN BOTH PLACEMENTS, the
//      suite's hosts in SRAM and their flash twins, THE LENGTH FROM WHICH THE
//      ENGINES COST THE CORE LESS measured against the arithmetic the page's
//      guidance is, what this image's handlers measure, and THE RECEIVE LOOP'S FRAME TABLE
//      (spi.frame): every code, 8 and 16 bits, modes 0 and 1, in both
//      placements, measured first and printed after
//   c  the refusals: the chapter's, the high-speed read off /2 at run
//      time, a ceiling under HCLK/256, a request naming the engines on a host
//      with none (nothing on the wire), and the client
//      configurations this die does not serve
//   f  THE FAULT EXITS, wireless: spi_stalled over a block held in reset and
//      one whose enable was taken down under the host, a command phase in the
//      loop in front of a block on the engines among them, both placements, the host
//      putting the block back with no recover(); a real transfer error, a block
//      running past the end of a memory, the same; a closed clock gate, which a
//      reset does not open, and a block over each hole in the map; a polled
//      request on the engined host over a receive channel a timer's held
//      request leaves dead (its loop, spi_ok, no channel touched), the
//      engines over it answered by the caller's bound and
//      recover(); and the block's own transmit request let go four ways -
//      release()'s reset pulse leaving the next owner of channel 3 nothing
//   e  OVR AND ITS CLEAR, wireless, measured against 16.2.7 on the resource:
//      what raises the flag, which reads clear it, flush_rx()
//   n  (y) THE PEER: the command channel, ident (the CH32V006, kind 0x06),
//      ten pings
//   o  (y) the matrix: this host in the four modes and both bit orders
//      against the foreign client on both of its serves, what the client
//      hears judged in all four, what this host reads where that client
//      answers (modes 2 and 3, CPOL = 1; in modes 0 and 1 it listens
//      alone), a bit-order mismatch as an exact
//      two-way reversal, the receive CRC over the foreign frames, and which
//      frame the buffer keeps when one is lost
//   p  (y) THE LADDER against the peer, /256 to /2 on the polled host and
//      on the engines, both of the peer's serves, /2 = 24 MHz judged with
//      the high-speed read as the driver's default has it and printed
//      without, and the high-speed read's own ladder
//   k  (y) THE CLIENT'S RUNS: the peer's answer, exchange and receive runs
//      (its driver's SpiClient runs), 8- and 16-bit frames, under the
//      polled host - a gap after every frame, the control - and back to
//      back under the engines, /64 to /2, judged to the pair's 6 MHz
//   t  (y) THE NEXT REQUEST AFTER A FAILURE, with no recover(): a stall and a
//      transfer error, directly and queued through SpiBus, the next request
//      polled and on the engines exact both ways, in both placements
//   q  (y) THE KERNEL: SpiBus over SpiHost and its engines, four
//      transactions queued in one select window - one polled, three on the
//      engines, one of those a command in the loop with its data on them, and
//      one a command alone - read back by the second chip
//   r  (y) THE ROLES INVERT: this chip as the CLIENT under the peer's
//      host_burst, in all four modes and both bit orders - 16.2.3's "mode
//      2 or mode 3" and 16.3.1's "LSB only as host" measured
//   v  (y) THE CLIENT FROM ITS VECTOR: letter r's bursts served by spi1's
//      vector (RXNEIE, TXEIE, SpiClient::isr()) in every admitted
//      combination, judged both ways, the entries and the body printed
//   h  (y) A HANDLER BESIDE THE POLLED LOOP IS A GAP ON THE BUS: the tick's
//      handler dragged past a frame, ten windows of 128 frames at /64 exact
//      both ways, spi_ok, OVR never standing, the window longer by what the
//      arithmetic allows
//   m  (y) MODF from a second chip: a host whose NSS is a hardware input,
//      demoted four times by the peer's ss_pulse on PA4, and the clearing
//      sequence; and ODEN - SCK on a wire with no pull-up, NSS on the
//      peer's
//   w  (y) THE WAVEFORMS: this chip as the logic analyser of its four link
//      pads - both hosts' SCK and MOSI, both clients' MISO, every mode
//   s  (by name) the /2 frame and the core's share of the bus: letter b's
//      engined frame at /2 twenty times with the completion polled in SRAM
//      and twenty with the core waiting off the bus
//   d  (by name) THE WITNESS, wireless, printing a table and judging nothing:
//      on the resource, with a frame waiting behind the one shifting, which of
//      RXNE and TXE a poll from SRAM sees first at each frame's end, and how
//      far apart, in the four modes, both widths, /8 to /256
//   x  (by name) the slip statistics: each mode ten times
//
// THE SUPPLY at boot: the voltage detector at 3.0 V (pwr.hpp); an image
// that drives a link pad from a sagging rail refuses every letter and
// says so.
//
// THE SUITE IS TWO IMAGES (design/overview.md, "A suite's image fits the
// family's smallest chip"): whole it comes within the 5 % of the 62 KB
// every part of the series has, so test_x035_spi-1 carries the wireless
// letters and test_x035_spi-2 the two-board ones (`y`) and the slip
// statistics.
// build: boards = x035f8
// build: groups = abcdefsgi,nopkqrvmwxht
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
#include "ch32x035/tim.hpp"
#include "ch32x035/usart.hpp"
#include "kernel/post.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time.hpp"
#include "util/placement.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/spi_bus.hpp"
#include "util/testbench.hpp"

#include "../../../avrxt/src/apps/spi_link.hpp"

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
using Host = SpiHost<1, spi1_default_pins, NoDmaEngine, NoDmaEngine, InRam>;
using DmaHost = SpiHost<1, spi1_default_pins, DmaTxEngine<3, uint16_t>, DmaRxEngine<2, uint16_t>, InRam>;
using FlashHost = SpiHost<1>;
using FlashDmaHost = SpiHost<1, spi1_default_pins, DmaTxEngine<3, uint16_t>, DmaRxEngine<2, uint16_t>>;
using Client = SpiClient<1>;

using DmaTx = DmaTxEngine<3, uint16_t>;
using CsPin = Pin<'A', 4>;
/// The failed request's select: a pad with no wire, so the peer never sees it.
using LoneCs = Pin<'A', 1>;
/// The D/C line of letter i's command rule: a pad with no wire either.
using DcPin = Pin<'A', 0>;
using SckPin = Pin<'A', 5>;
using MisoPin = Pin<'A', 6>;
using MosiPin = Pin<'A', 7>;

TwoKeyTestBench<Serial, 'y'> bench;

/// Which task owns SPI1 and its two channels: the vectors serve it.
enum class Owner : uint8_t { host, dma_host, flash_host, flash_dma_host, bus_ao, client_vectors, none };
volatile Owner owner = Owner::host;
volatile bool host_done = false;
volatile uint32_t spi_isr_entries = 0;
volatile uint32_t dma_isr_entries = 0;

/// The image's own handlers, measured: the longest body each ran, stamped on
/// the STK at its first and last statement.
volatile uint32_t console_body_max = 0;
volatile uint32_t tick_body_max = 0;
/// Letter h's drag: empty turns the tick's handler spends after its stamps,
/// so tick_body_max stays the handler's own.
volatile uint32_t tick_drag = 0;
/// Letter i's witness of D/C: while `dc_watch` stands the tick's handler counts
/// the level it finds on the pad.
volatile bool dc_watch = false;
volatile uint32_t dc_low_ticks = 0;
volatile uint32_t dc_high_ticks = 0;
/// Letter g's: what the vector read right after dma_isr() answered true, the
/// SCK pad and the cycle it read it on.
volatile bool sck_at_end = false;
volatile uint32_t end_stamp = 0;

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
    r.engine = polled ? SpiEngine::polled : SpiEngine::dma;
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
    r.engine = polled ? SpiEngine::polled : SpiEngine::dma;
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

/// A request of `n` byte frames naming the engines at `rate`, read or written, the
/// select the lone pad: true when start() handed it to them (and the vector
/// ended it), false when it ended inside start(); `status` what it ended with.
template <typename D = DmaHost>
bool went_to_engines(uint16_t n, SpiClock rate, bool read, uint8_t& status) {
    typename D::Request r{};
    r.cs = LoneCs::ref();
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
    r.rx = lend<Lease::reply>(read ? static_cast<uint8_t*>(rx_buf) : nullptr);
    r.len = n;
    r.mode = SpiMode::mode0;
    r.clock = rate;
    r.bits = SpiDataSize::bits8;
    r.engine = SpiEngine::dma;
    host_done = false;
    dma_isr_entries = 0;
    if (D::start(r)) {
        status = D::status();
        return false;
    }
    for (uint32_t i = 0; i < 8'000'000UL && !host_done; ++i) {
    }
    status = host_done ? D::status() : no_answer;
    if (!host_done) {
        (void)D::recover();
    }
    return true;
}

/// A request that must be refused, on a port of RAM: a select and a D/C that
/// any store on the path would mark. It asks mode 3 at /2 of a host primed in
/// mode 0 at /16. True when start() answered at once with spi_rejected, neither
/// pad was touched, the control word stands, SCK holds its mode 0 idle and the
/// block is idle: nothing reached the wire.
template <typename D>
bool refused(const uint8_t* cmd, uint8_t cmd_len, const uint8_t* tx, uint8_t* rx, uint16_t len, SpiDataSize bits) {
    D::prime(SpiMode::mode0, SpiClock::div16);
    const uint16_t word = S::regs().CTLR1;
    GpioRegs port{};
    typename D::Request r{};
    r.cs = PinRef{&port, 1u};
    r.dc = PinRef{&port, 2u};
    r.cmd = lend<Lease::reply>(cmd);
    r.cmd_len = cmd_len;
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = len;
    r.mode = SpiMode::mode3;
    r.clock = SpiClock::div2;
    r.bits = bits;
    r.engine = SpiEngine::dma;
    host_done = false;
    dma_isr_entries = 0;
    const bool done = D::start(r);
    const bool untouched = port.BSHR == 0u && port.BCR == 0u;
    const uint16_t st = S::status();
    return done && D::status() == spi_rejected && untouched && S::regs().CTLR1 == word && !SckPin::read() &&
           (st & spi_txe) != 0u && (st & spi_bsy) == 0u && dma_isr_entries == 0u;
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
    r.engine = SpiEngine::polled;
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

/// Letter h's witnesses of the last polled window: its cycles around the
/// request and its select, and OVR read right after it.
uint32_t window_cycles = 0;
bool window_ovr = false;

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
        const uint32_t t0 = cycles_now();
        (void)link_xfer(xtx, xrx, n, e.rate, e.host_mode, bits);
        window_cycles = cycles_now() - t0;
        window_ovr = (S::status() & spi_ovr) != 0u;
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

    // EVERY WAY COMPLETES WITH NOTHING ON MISO, the select held high: the
    // two polled loops; a polled request on the engined host, the loop at any
    // length with no channel interrupt; the engines at 64 frames.
    host_ready();
    fill_pattern(tx_buf, 80, 0x21);
    const uint8_t polled = host_xfer(CsPin::ref(), nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode3,
                                     SpiClock::div2, SpiDataSize::bits16, true);
    const uint8_t written = host_xfer(CsPin::ref(), cmd_buf, 2, tx_buf, nullptr, 16, SpiMode::mode0,
                                      SpiClock::div2, SpiDataSize::bits8, true);
    dma_ready();
    const uint8_t dma = dma_xfer(CsPin::ref(), nullptr, 0, tx_buf, rx_buf, 64, SpiClock::div4,
                                 SpiDataSize::bits8, false);
    const uint32_t e0 = dma_isr_entries;
    const uint8_t dma_polled = dma_xfer(CsPin::ref(), cmd_buf, 1, tx_buf, rx_buf, 80, SpiClock::div2,
                                        SpiDataSize::bits8, true);
    const uint32_t dma_polled_entries = dma_isr_entries - e0;
    DmaHost::release();
    host_ready();
    print(serial, "  wireless completions: polled=", polled, " written=", written, " dma=", dma, " engined host polled=",
          dma_polled, " (", dma_polled_entries, " channel interrupts)", crlf);
    bench.verdict("the two polled loops, a polled request on the engined host (no channel interrupt) and the engines "
                  "each run a transaction to completion with MISO floating, the select released",
                  polled == spi_ok && written == spi_ok && dma == spi_ok && dma_polled == spi_ok &&
                      dma_polled_entries == 0u && CsPin::read_out());
}

// ===========================================================================
// i - the engine a request names, wireless
// ===========================================================================

void ti_engine_rules() {
    dma_ready();
    LoneCs::output(true);
    fill_pattern(tx_buf, 64, 0x21);
    // SpiEngine::dma MEANS DMA AT THE SHORTEST LENGTH: one frame at /2, read and
    // written, where the loop costs the core far less (the guidance, letter b):
    // start() false, one interrupt, the vector the end.
    uint8_t one_read_status = no_answer;
    const bool one_read = went_to_engines(1, SpiClock::div2, true, one_read_status);
    const uint32_t one_read_entries = dma_isr_entries;
    uint8_t one_written_status = no_answer;
    const bool one_written = went_to_engines(1, SpiClock::div2, false, one_written_status);
    const uint32_t one_written_entries = dma_isr_entries;
    print(serial, "  one frame at /2 naming the engines: read ", one_read ? "on them" : "INSIDE start()", ", status ",
          one_read_status, ", ", one_read_entries, " interrupts; written ", one_written ? "on them" : "INSIDE start()",
          ", status ", one_written_status, ", ", one_written_entries, " interrupts", crlf);
    bench.verdict("ONE FRAME NAMING THE ENGINES RIDES THEM, read and written at /2: start() false, one DMA interrupt "
                  "the end, spi_ok",
                  one_read && one_read_status == spi_ok && one_read_entries == 1u && one_written &&
                      one_written_status == spi_ok && one_written_entries == 1u);

    // THE COMMAND RULE, D/C sampled by the tick: 64 frames at /256 are 131 072
    // cycles, 2.7 periods of the tick's 48 000, so at least two ticks land in a
    // phase of 64 frames however the phases fall. A command then data: the command
    // runs in the loop inside start(), D/C low on every tick that lands there; the
    // data phase on the engines after start() has returned, D/C high on every tick
    // that lands there, and one interrupt ends it. A command alone: start() false,
    // D/C low through the block, high at the end.
    DcPin::output(true);
    auto command_request = [](uint16_t len) {
        DmaHost::Request r{};
        r.cs = LoneCs::ref();
        r.dc = DcPin::ref();
        r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
        r.cmd_len = 64;
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
        r.rx = lend<Lease::reply>(static_cast<uint8_t*>(rx_buf));
        r.len = len;
        r.clock = SpiClock::div256;
        r.engine = SpiEngine::dma;
        return r;
    };
    dc_low_ticks = 0;
    dc_high_ticks = 0;
    dc_watch = true;
    host_done = false;
    dma_isr_entries = 0;
    const bool both_inside = DmaHost::start(command_request(64));
    const uint32_t low_in_loop = dc_low_ticks;
    const uint32_t high_in_loop = dc_high_ticks;
    const uint32_t entries_in_loop = dma_isr_entries;
    dc_low_ticks = 0;
    dc_high_ticks = 0;
    for (uint32_t i = 0; i < 8'000'000UL && !host_done; ++i) {
    }
    const bool both_done = host_done;
    const uint32_t low_on_engines = dc_low_ticks;
    const uint32_t high_on_engines = dc_high_ticks;
    const uint32_t both_entries = dma_isr_entries;
    const uint8_t both_status = DmaHost::status();
    const bool both_up = DcPin::read_out() && LoneCs::read_out();
    print(serial, "  a command of 64 then 64 of data at /256: start() ", both_inside ? "TRUE" : "false",
          ", in the loop D/C low on ", low_in_loop, " ticks and high on ", high_in_loop, ", ", entries_in_loop,
          " interrupts; on the engines low on ", low_on_engines, " and high on ", high_on_engines, ", ", both_entries,
          " interrupts, status ", both_status, crlf);
    bench.verdict("A COMMAND THEN DATA: the command in the loop inside start(), D/C low there; the data on the engines, "
                  "D/C high there; one interrupt, spi_ok, D/C and the select high at the end",
                  !both_inside && both_done && low_in_loop >= 2u && high_in_loop == 0u && entries_in_loop == 0u &&
                      low_on_engines == 0u && high_on_engines >= 2u && both_entries == 1u && both_status == spi_ok &&
                      both_up);
    dc_low_ticks = 0;
    dc_high_ticks = 0;
    host_done = false;
    dma_isr_entries = 0;
    const bool alone_inside = DmaHost::start(command_request(0));
    const uint32_t alone_entries_at_return = dma_isr_entries;
    const bool alone_low_at_return = !DcPin::read_out();
    for (uint32_t i = 0; i < 8'000'000UL && !host_done; ++i) {
    }
    dc_watch = false;
    const bool alone_done = host_done;
    const uint32_t alone_low = dc_low_ticks;
    const uint32_t alone_high = dc_high_ticks;
    const uint32_t alone_entries = dma_isr_entries;
    const uint8_t alone_status = DmaHost::status();
    const bool alone_up = DcPin::read_out() && LoneCs::read_out();
    DcPin::release();
    print(serial, "  a command of 64 alone at /256: start() ", alone_inside ? "TRUE" : "false", ", D/C low on ", alone_low,
          " ticks and high on ", alone_high, ", ", alone_entries, " interrupts, status ", alone_status, crlf);
    bench.verdict("A COMMAND ALONE RIDES THE ENGINES: start() false with D/C low and no interrupt yet, D/C low on every "
                  "tick of the block, one interrupt the end, spi_ok, D/C and the select high",
                  !alone_inside && alone_done && alone_entries_at_return == 0u && alone_low_at_return &&
                      alone_low >= 2u && alone_high == 0u && alone_entries == 1u && alone_status == spi_ok && alone_up);

    // WHAT THE ENGINES CANNOT CARRY IS REFUSED, before the block, the select or the
    // D/C is touched (refused()): 16-bit frames with the out buffer, the in buffer
    // or a command alone off the half-word boundary (9.3.6), 16-bit frames on engines
    // of bytes, and a transmit engine another owner holds, whose claim would refuse
    // the launch and leave the receive block waiting for ever. A command in front
    // of a data phase runs in the loop, from any address: that one is not refused.
    constexpr SpiDataSize wide = SpiDataSize::bits16;
    const bool odd_out = refused<DmaHost>(nullptr, 0, tx_buf + 1, rx_buf, 8, wide);
    const bool odd_in = refused<DmaHost>(nullptr, 0, tx_buf, rx_buf + 1, 8, wide);
    const bool odd_command = refused<DmaHost>(tx_buf + 1, 4, nullptr, nullptr, 0, wide);
    const bool claimed = DmaTx::claim();
    const bool held = refused<DmaHost>(nullptr, 0, tx_buf, rx_buf, 8, SpiDataSize::bits8);
    DmaTx::unclaim();
    const uint8_t odd_command_with_data = dma_xfer(LoneCs::ref(), tx_buf + 1, 2, tx_buf, rx_buf, 8, SpiClock::div16, wide,
                                                   false);
    const uint32_t odd_command_entries = dma_isr_entries;
    const uint8_t next_polled = dma_xfer(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 8, SpiClock::div16, wide, true);
    const uint8_t next_engines = dma_xfer(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 8, SpiClock::div16, wide, false);
    DmaHost::release();
    using ByteHost = SpiHost<1, spi1_default_pins, DmaTxEngine<3, uint8_t>, DmaRxEngine<2, uint8_t>>;
    (void)ByteHost::init(clock);
    const bool on_bytes = refused<ByteHost>(nullptr, 0, tx_buf, rx_buf, 8, wide);
    ByteHost::release();
    host_ready();
    print(serial, "  refused: odd out ", odd_out ? "yes" : "NO", ", odd in ", odd_in ? "yes" : "NO", ", odd command alone ",
          odd_command ? "yes" : "NO", ", engines of bytes ", on_bytes ? "yes" : "NO", ", a held transmit engine ",
          held ? "yes" : "NO", "; an odd command before aligned data ", odd_command_with_data, " (", odd_command_entries,
          " interrupt); then polled ", next_polled, ", on the engines ", next_engines, crlf);
    bench.verdict("WHAT THE ENGINES CANNOT CARRY IS REFUSED: 16-bit frames off the half-word boundary, out, in or a "
                  "command alone, on engines of bytes, and over a held transmit engine: spi_rejected at once, select "
                  "and D/C never touched, the control word, SCK and the block as they were",
                  odd_out && odd_in && odd_command && on_bytes && claimed && held);
    bench.verdict("AND THE NEXT REQUESTS RUN: an odd command before aligned data in the loop (one interrupt), then "
                  "polled and on the engines, each spi_ok",
                  odd_command_with_data == spi_ok && odd_command_entries == 1u && next_polled == spi_ok &&
                      next_engines == spi_ok);
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
/// it): the engines' fixed cost at /4; the plain host's polled loops'
/// frames at /2, /4, /16, /8 and /64 and the receive loop's fixed cost at
/// /4, a zero-length and a 3-frame polled request - frames x10; and, on the
/// engined host, through the call the engines' fixed cost is taken
/// through, E - F, F the intercept of two polled lengths, and the length from
/// which the engines cost the core less, measured and by the arithmetic.
struct PathCosts {
    uint32_t engine_fixed;
    uint32_t rd[5], wr[5], rd_fixed;
    uint32_t empty, three;
    /// The engined host's polled lines through dma_xfer<D>, a frame (x10)
    /// and F, at the three shapes (edge_shapes); E - F for a read and a write;
    /// the edge measured and the edge the arithmetic gives on these figures.
    uint32_t line_frame[3], line_fixed[3];
    uint32_t over_read, over_written;
    uint16_t edge_meas[3], edge_calc[3];
};

/// The shapes the edge is measured at: a read at /4 (the receive loop's
/// dead bus) and at /2 (its turn), a write at /2 (the transmit-only loop's
/// turn).
struct EdgeShape {
    SpiClock rate;
    bool read;
};
constexpr EdgeShape edge_shapes[3] = {{SpiClock::div4, true}, {SpiClock::div2, true}, {SpiClock::div2, false}};

template <typename H, typename D>
PathCosts measure_costs() {
    PathCosts c{};
    dma_ready<D>();
    (void)frame_x1000<D>(SpiClock::div4, SpiMode::mode0, SpiDataSize::bits8, 256, 32, &c.engine_fixed);
    // THE ENGINED HOST'S POLLED LINES through the same call the engines'
    // fixed cost was taken through (dma_xfer<D>, so the call's own cycles
    // stand on both sides and cancel), the loops: 24 frames and 8, the
    // frame the difference over 16, F the intercept, the 8-frame line less its
    // frames, the best of five each.
    auto polled_same = [](uint16_t n, SpiClock rate, bool read) {
        uint32_t least = 0xFFFFFFFFu;
        for (uint8_t j = 0; j < 5u; ++j) {
            const uint32_t t0 = cycles_now();
            (void)dma_xfer<D>({}, nullptr, 0, tx_buf, read ? rx_buf : nullptr, n, rate, SpiDataSize::bits8, true);
            const uint32_t took = cycles_now() - t0;
            least = took < least ? took : least;
        }
        return least;
    };
    for (uint8_t k = 0; k < 3u; ++k) {
        const EdgeShape sh = edge_shapes[k];
        const uint32_t p8 = polled_same(8, sh.rate, sh.read);
        const uint32_t p24 = polled_same(24, sh.rate, sh.read);
        c.line_frame[k] = p24 > p8 ? (p24 - p8) * 10u / 16u : 0u;
        c.line_fixed[k] = p8 > 8u * c.line_frame[k] / 10u ? p8 - 8u * c.line_frame[k] / 10u : 0u;
        // The arithmetic on these figures: ceil((E - F) / frame).
        const uint32_t over = c.engine_fixed > c.line_fixed[k] ? c.engine_fixed - c.line_fixed[k] : 0u;
        c.edge_calc[k] = c.line_frame[k] != 0u
                             ? static_cast<uint16_t>((over * 10u + c.line_frame[k] - 1u) / c.line_frame[k])
                             : uint16_t{0};
        // THE EDGE MEASURED: the fewest frames whose polled line costs at
        // least the engines' fixed cost, walked upward from five under the
        // arithmetic's count to five over it.
        c.edge_meas[k] = 0;
        const uint16_t from = c.edge_calc[k] > 5u ? static_cast<uint16_t>(c.edge_calc[k] - 5u) : uint16_t{1};
        for (uint16_t n = from; n < c.edge_calc[k] + 6u && c.edge_meas[k] == 0u; ++n) {
            if (polled_same(n, sh.rate, sh.read) >= c.engine_fixed) {
                c.edge_meas[k] = n;
            }
        }
    }
    c.over_read = c.engine_fixed > c.line_fixed[0] ? c.engine_fixed - c.line_fixed[0] : 0u;
    c.over_written = c.engine_fixed > c.line_fixed[2] ? c.engine_fixed - c.line_fixed[2] : 0u;
    D::release();
    host_ready<H>();
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

/// The measured edges within one frame of the arithmetic on the same
/// figures - the count quantizes, the best of five keeps a cycle or two of
/// noise.
bool edges_match(const PathCosts& c) {
    for (uint8_t k = 0; k < 3u; ++k) {
        if (c.edge_meas[k] == 0u || c.edge_meas[k] + 1u < c.edge_calc[k] || c.edge_meas[k] > c.edge_calc[k] + 1u) {
            return false;
        }
    }
    return true;
}

/// A frame's time on the wire in HCLK cycles (16.3.1's BR table, HSRXEN clear).
constexpr uint32_t frame_wire(SpiClock c, SpiDataSize bits) {
    return static_cast<uint32_t>(spi_data_bits(bits)) << (static_cast<uint8_t>(c) + 1u);
}

// ---- spi.frame: the receive loop's frame against the wire --------------------
//
// THE TABLE THE DRIVER'S DOCUMENT QUOTES: every code /2 to /256, 8 and 16 bits,
// modes 0 and 1, the select null, MISO floating. Two lengths, 256 frames and 64
// or, at the slow codes, what stays under 40 000 cycles and a quarter of it; the
// frame their difference over the frames between them; the best of five runs,
// each with interrupts masked (no handler in the figure, and a polled request
// needs no vector) from a tick's edge, so the ruler sees no restart under the
// mask; MEASURED FIRST AND PRINTED AFTER: no console line beside a run.

constexpr uint8_t frame_codes = 8;   ///< /2 .. /256
/// [width][mode][code]: the frame in hundredths of a cycle.
uint32_t frame_x100[2][2][frame_codes];
uint8_t frame_bad = 0;

/// The longer length of a row.
constexpr uint16_t frame_long(uint32_t wire) {
    const uint32_t n = 40'000u / (wire + 64u);
    return static_cast<uint16_t>(n > 256u ? 256u : n);
}

/// The least wall of `frames` read at this code, width and mode, of five runs;
/// the first run, which applies the rate, discarded.
template <typename H>
uint32_t frame_best(SpiClock c, SpiDataSize bits, SpiMode m, uint16_t frames) {
    typename H::Request r{};
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(rx_buf));
    r.len = frames;
    r.clock = c;
    r.mode = m;
    r.bits = bits;
    r.engine = SpiEngine::polled;
    uint32_t best = 0xFFFFFFFFu;
    for (uint8_t run = 0; run < 6u; ++run) {
        const uint32_t tick = Ticker::millis();
        while (Ticker::millis() == tick) {
        }
        uint32_t wall = 0;
        {
            InterruptGuard guard;
            const uint32_t t0 = cycles_now();
            (void)H::start(r);
            wall = cycles_now() - t0;
        }
        if (H::status() != spi_ok && frame_bad != 255u) {
            ++frame_bad;
        }
        if (run != 0u && wall < best) {
            best = wall;
        }
    }
    return best;
}

/// The table of one host: measured first, printed after.
template <typename H>
void frame_table(const char* name) {
    frame_bad = 0;
    console_drain();
    host_ready<H>();
    for (uint8_t wi = 0; wi < 2u; ++wi) {
        const SpiDataSize bits = wi == 0u ? SpiDataSize::bits8 : SpiDataSize::bits16;
        for (uint8_t mi = 0; mi < 2u; ++mi) {
            const SpiMode mode = mi == 0u ? SpiMode::mode0 : SpiMode::mode1;
            for (uint8_t ci = 0; ci < frame_codes; ++ci) {
                const SpiClock c = static_cast<SpiClock>(ci);
                const uint16_t big = frame_long(frame_wire(c, bits));
                const uint16_t small = big / 4u;
                const uint32_t w_small = frame_best<H>(c, bits, mode, small);
                const uint32_t w_big = frame_best<H>(c, bits, mode, big);
                frame_x100[wi][mi][ci] = (w_big - w_small) * 100u / (big - small);
            }
        }
    }
    print(serial, "  spi.frame ", name, ": bad status ", frame_bad, crlf);
    for (uint8_t wi = 0; wi < 2u; ++wi) {
        const SpiDataSize bits = wi == 0u ? SpiDataSize::bits8 : SpiDataSize::bits16;
        for (uint8_t mi = 0; mi < 2u; ++mi) {
            for (uint8_t ci = 0; ci < frame_codes; ++ci) {
                const uint32_t wire = frame_wire(static_cast<SpiClock>(ci), bits);
                const uint32_t per = frame_x100[wi][mi][ci];
                const uint32_t over = per > wire * 100u ? per - wire * 100u : 0u;
                print(serial, "  spi.frame ", name, " bits", wi == 0u ? 8u : 16u, " mode", mi, " div", 2u << ci, " wire ",
                      wire, " frame ", per / 100u, '.', per % 100u < 10u ? "0" : "", per % 100u, " over ", over / 100u,
                      '.', over % 100u < 10u ? "0" : "", over % 100u, crlf);
                console_drain();
            }
        }
    }
}

void tb_ladder() {
    dma_ready();
    fill_pattern(tx_buf, 512, 0x3C);
    console_drain();
    // THE LADDER: a frame inside a 256- and a 32-frame block, every code,
    // measured with the console quiet and printed after: a 256-frame block at
    // /2 lasts about what a console character does at 115200, so a line going
    // out beside it lengthens that rung (docs/ch32x035/spi.md, "The host above
    // the wire").
    uint32_t ladder[8];
    for (uint8_t code = 0; code < 8u; ++code) {
        const SpiClock rate = static_cast<SpiClock>(code);
        const uint32_t f = frame_x1000(rate, SpiMode::mode0, SpiDataSize::bits8, 256, 32);
        ladder[code] = permille_of(f, 8, asked_divisor(rate));
    }
    const uint32_t w16_div4 = permille_of(frame_x1000(SpiClock::div4, SpiMode::mode0, SpiDataSize::bits16, 128, 16),
                                          16, 4);
    const uint32_t w16_div16 =
        permille_of(frame_x1000(SpiClock::div16, SpiMode::mode3, SpiDataSize::bits16, 128, 16), 16, 16);
    uint8_t exact = 0;
    print(serial, "  a frame inside an engined block, in thousandths of its SCK periods at the rate asked "
                  "(8 bits):",
          crlf, "   ");
    for (uint8_t code = 0; code < 8u; ++code) {
        print(serial, " /", asked_divisor(static_cast<SpiClock>(code)), " ", ladder[code]);
        if (wire_ok(ladder[code])) {
            ++exact;
        }
    }
    print(serial, crlf);
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
        console_drain();
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
                      console_drain();
                      DmaHost::prime(SpiMode::mode0, SpiClock::div2);
                      S::high_speed_read_raw(true);
                      const uint32_t f = frame_x1000(SpiClock::div2, SpiMode::mode0, SpiDataSize::bits8, 256, 32);
                      S::high_speed_read_raw(false);
                      return f >= 15'840u && f <= 16'480u;
                  }());

    // THE HOST'S COSTS, both placements: the suite's hosts in SRAM, then their
    // flash twins - the same measurement. The engine a request runs on is the
    // program's to name; these figures are what the page's guidance is drawn from.
    console_drain();
    const PathCosts ram = measure_costs<Host, DmaHost>();
    const PathCosts flash = measure_costs<FlashHost, FlashDmaHost>();
    print(serial, "  THE HOST'S COSTS, SRAM | flash (cycles; x.y a frame):", crlf);
    print(serial, "    the engines' fixed cost at /4 (the 32-frame block less 32 frames): ", ram.engine_fixed, " | ",
          flash.engine_fixed, crlf);
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
    print(serial, "; /64 (the wire 512): ");
    print_x10(ram.rd[4]);
    print(serial, " | ");
    print_x10(flash.rd[4]);
    print(serial, crlf, "    its fixed cost at /4 through the plain host's call: ", ram.rd_fixed, " | ", flash.rd_fixed,
          ", its dead bus a frame at /4 ");
    print_x10(ram.rd[1] > 320u ? ram.rd[1] - 320u : 0u);
    print(serial, " | ");
    print_x10(flash.rd[1] > 320u ? flash.rd[1] - 320u : 0u);
    print(serial, crlf);
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
    print(serial, crlf);
    print(serial, "    a zero-length request through the suite's wrapper: ", ram.empty, " | ", flash.empty,
          "; a 3-frame polled request at /16: ", ram.three, " | ", flash.three,
          " (the wire's 384 and the wrapper inside it)", crlf);
    console_drain();
    print(serial, "    THE ENGINED HOST, SRAM | flash, through the engines' call: E - F read ", ram.over_read, " | ",
          flash.over_read, ", written ", ram.over_written, " | ", flash.over_written, crlf,
          "      a polled frame x10 / F: read /4 ");
    auto line = [](const PathCosts& c, uint8_t k) { print(serial, c.line_frame[k], "/", c.line_fixed[k]); };
    line(ram, 0);
    print(serial, ", read /2 ");
    line(ram, 1);
    print(serial, ", write /2 ");
    line(ram, 2);
    print(serial, " | ");
    line(flash, 0);
    print(serial, ", ");
    line(flash, 1);
    print(serial, ", ");
    line(flash, 2);
    print(serial, crlf);
    console_drain();
    print(serial, "      THE LENGTH FROM WHICH THE ENGINES COST LESS, frames - measured/arithmetic: ");
    auto edge = [](const PathCosts& c, uint8_t k) { print(serial, c.edge_meas[k], "/", c.edge_calc[k], " "); };
    for (uint8_t k = 0; k < 3u; ++k) {
        edge(ram, k);
    }
    print(serial, "| ");
    for (uint8_t k = 0; k < 3u; ++k) {
        edge(flash, k);
    }
    print(serial, "(read /4, read /2, write /2)", crlf);
    bench.verdict("THE TRANSMIT-ONLY LOOP IN SRAM is at the wire (3 %) at /2, /4, /8 and /16",
                  ram.wr[0] <= 165u && ram.wr[1] <= 330u && ram.wr[3] <= 659u && ram.wr[2] <= 1319u);
    // From the flash at /2 the loop's turn fetched behind the wait states
    // is the bound - 24.0 cycles a frame with its function on a word, 28.0
    // half a word off (the host aligns it): the fetch's figure.
    bench.verdict("and from the flash within 5 % of the wire at /4 and at the wire from /8",
                  flash.wr[1] <= 336u && flash.wr[3] <= 659u && flash.wr[2] <= 1319u);
    bench.verdict("SRAM shortens the receive loop's frame at /2", ram.rd[0] < flash.rd[0]);
    bench.verdict("THE GUIDANCE IS THE ARITHMETIC'S: the fewest frames a polled transaction spends the engines' measured "
                  "fixed cost on is within one frame of ceil((E - F) / frame) on the same figures, read at /4 and /2 and "
                  "written at /2, in both placements",
                  edges_match(ram) && edges_match(flash));
    host_ready();
    console_drain();
    const uint32_t flash2 = loop_frame_x10(tx_loop_copy<0>, SpiClock::div2);
    const uint32_t sram2 = loop_frame_x10(tx_loop_sram, SpiClock::div2);
    const uint32_t flash4 = loop_frame_x10(tx_loop_copy<0>, SpiClock::div4);
    const uint32_t sram4 = loop_frame_x10(tx_loop_sram, SpiClock::div4);
    print(serial, "  THE FETCH: the bare transmit loop a frame, from the flash /2 ", flash2 / 10u, ".", flash2 % 10u,
          " /4 ", flash4 / 10u, ".", flash4 % 10u, "; from SRAM /2 ", sram2 / 10u, ".", sram2 % 10u, " /4 ",
          sram4 / 10u, ".", sram4 % 10u, " cycles", crlf);

    // THIS IMAGE'S HANDLERS: the longest body of the console's and the tick's
    // so far, stamped on the STK.
    print(serial, "  this image's handlers, the longest body: console ", console_body_max, " cycles, tick ",
          tick_body_max, crlf);

    frame_table<Host>("sram");
    frame_table<FlashHost>("flash");
    host_ready();
}

// ===========================================================================
// g - when a request ends, the wire has ended
// ===========================================================================
//
// THE RULE. The select rises after the last SCK edge, and what tells a caller the
// request is over - start() true, dma_isr() true - comes after that edge too, also
// for a caller framing the select itself. 16.2.2 raises RXNE at "the last sampling
// clock edge of the data word", which in CPHA = 0 is half a bit before the frame's
// last edge, so the rule is a question this letter puts to the silicon; the receive
// channel's completion follows RXNE and asks it too.
//
// WHAT IT READS: the SCK pad's input (INDR, the pad itself, an alternate function
// or not) against the mode's idle level, CPOL. Three places:
//  - AT THE FLAG, interrupts masked, one frame: the first STATR load that shows
//    RXNE, the pad read in the next instruction; the first INTFR load that shows
//    the receive channel's completion, the same. Each poll also reads the pad, so
//    the pass sees SCK move or says it did not, and the polls over the frame's
//    cycles give the instrument's own grain. The polls from the flag to the pad's
//    idle level are the early part, if there is one.
//  - RIGHT AFTER THE REQUEST, on both engines: the pad read on the instruction after
//    start() returned true, or in the vector after dma_isr() returned true, with a
//    select and through a null one.
//
// THE BOUND, the arithmetic. Half a bit is the code's divisor over two in HCLK
// cycles: 128 at /256, 32 at /64. A poll turn is the frame's cycles over the polls a
// frame the letter prints, 29 to 57 cycles from the flash on these parts, and the pad
// read on the flag's own turn comes a few cycles after the load that showed it. So
// at /256 an early flag leaves the pad off its idle level for two polls at the least
// and cannot be missed; at /64 it is seen on most passes and not on every one; from
// /32 up (16 cycles or less) the instrument can no longer see it, and those rungs
// rest on the same block doing the same thing. After the request the host's tail
// stands between the flag and the read: in start() some twenty instructions,
// counted in the listing, under the half bit at /256, so that read sees an early
// end there too; in the vector more than the half bit at every rung (printed), so
// that read proves the order and not the flag's timing, which is why the flag is
// read as well (docs/ch32x035/spi.md). Any read off the idle level fails the verdict: no
// tolerance, a pad has two levels.
constexpr uint32_t flag_spins = 200'000UL;
/// Cycles from the channel's flag seen in channel_pass() to the vector's read.
uint32_t vector_read_after = 0;

struct FlagPass {
    bool at_idle = false;      ///< the pad at its idle level at the flag
    bool moved = false;        ///< seen off the idle level before the flag
    uint16_t polls_to_idle = 0;
    uint32_t polls = 0;
    bool came = false;
};

/// One frame by hand on the resource, masked, the host primed in this mode and at this code.
template <SpiDataSize bits>
FlagPass rxne_pass(SpiMode m, SpiClock c) {
    Host::prime(m, c, bits);
    S::flush_rx();
    const bool idle = spi_mode_cpol(m);
    FlagPass f{};
    InterruptGuard guard;
    S::regs().DATAR = 0xA5u;
    uint32_t spins = flag_spins;
    while ((S::regs().STATR & spi_rxne) == 0u && --spins != 0u) {
        f.moved = f.moved || (SckPin::read() != idle);
        ++f.polls;
    }
    f.came = spins != 0u;
    f.at_idle = SckPin::read() == idle;
    while (SckPin::read() != idle && f.polls_to_idle < 1000u) {
        ++f.polls_to_idle;
    }
    (void)S::regs().DATAR;
    (void)S::regs().STATR;
    return f;
}

/// One frame on the engines, masked: the vector cannot run, so the channel's flag
/// is polled, and the pending vector ends the request at the guard's end.
FlagPass channel_pass(SpiMode m, SpiClock c) {
    DmaHost::prime(m, c);
    const bool idle = spi_mode_cpol(m);
    DmaHost::Request r{};
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(rx_buf));
    r.len = 1;
    r.mode = m;
    r.clock = c;
    r.engine = SpiEngine::dma;
    host_done = false;
    FlagPass f{};
    uint32_t t0 = 0;
    {
        InterruptGuard guard;
        (void)DmaHost::start(r);
        uint32_t spins = flag_spins;
        while ((DmaChannel<2>::flags() & DmaFlag::complete) == 0u && --spins != 0u) {
            f.moved = f.moved || (SckPin::read() != idle);
            ++f.polls;
        }
        f.came = spins != 0u;
        f.at_idle = SckPin::read() == idle;
        while (SckPin::read() != idle && f.polls_to_idle < 1000u) {
            ++f.polls_to_idle;
        }
        t0 = cycles_now();
    }
    for (uint32_t i = 0; i < 8'000'000UL && !host_done; ++i) {
    }
    vector_read_after = end_stamp - t0;
    return f;
}

/// The polled and engined requests of one rung on host `H` (DmaHost from SRAM,
/// FlashDmaHost from the flash): 2 frames, the four
/// modes, read and write, a select and a null one, three rounds so that a tick that
/// lands on one read cannot hide it. Counts what ended with SCK off its idle level.
template <typename H>
void end_of_request(SpiClock c, uint16_t& asked, uint16_t& off_polled, uint16_t& off_engines, uint16_t& bad_answer) {
    for (uint8_t round = 0; round < 3u; ++round) {
        for (uint8_t m = 0; m < 4u; ++m) {
            const SpiMode mode = static_cast<SpiMode>(m);
            const bool idle = spi_mode_cpol(mode);
            for (uint8_t shape = 0; shape < 4u; ++shape) {
                const bool read = (shape & 1u) != 0u;
                const bool named = (shape & 2u) != 0u;
                H::prime(mode, c);
                typename H::Request r{};
                r.cs = named ? LoneCs::ref() : PinRef{};
                r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
                r.rx = lend<Lease::reply>(read ? static_cast<uint8_t*>(rx_buf) : nullptr);
                r.len = 2;
                r.mode = mode;
                r.clock = c;
                r.engine = SpiEngine::polled;
                const bool polled_done = H::start(r);
                const bool polled_level = SckPin::read();
                const uint8_t polled_status = H::status();
                r.engine = SpiEngine::dma;
                host_done = false;
                const bool inside = H::start(r);
                for (uint32_t i = 0; i < 8'000'000UL && !host_done; ++i) {
                }
                ++asked;
                if (polled_level != idle) {
                    ++off_polled;
                }
                if (sck_at_end != idle) {
                    ++off_engines;
                }
                if (!polled_done || polled_status != spi_ok || inside || !host_done || H::status() != spi_ok ||
                    !LoneCs::read_out()) {
                    ++bad_answer;
                }
            }
        }
    }
}

void tg_wire_end() {
    fill_pattern(tx_buf, 16, 0x5A);
    LoneCs::output(true);
    const SpiClock rungs[2] = {SpiClock::div256, SpiClock::div64};
    uint8_t all_moved = 0;
    uint8_t flag_idle = 0;
    uint8_t channel_idle = 0;
    uint8_t flag_passes = 0;
    host_ready();
    for (const SpiClock c : rungs) {
        for (uint8_t m = 0; m < 4u; ++m) {
            const FlagPass f = rxne_pass<SpiDataSize::bits8>(static_cast<SpiMode>(m), c);
            const FlagPass w = rxne_pass<SpiDataSize::bits16>(static_cast<SpiMode>(m), c);
            print(serial, "  RXNE /", 2u << static_cast<uint8_t>(c), " mode ", m, ": at the flag SCK ",
                  f.at_idle ? "idle" : "OFF IDLE", ", ", f.polls_to_idle, " polls to idle, ", f.polls,
                  " polls a frame; 16-bit ", w.at_idle ? "idle" : "OFF IDLE", ", ", w.polls_to_idle, crlf);
            console_drain();
            flag_passes = static_cast<uint8_t>(flag_passes + 2u);
            if (f.came && f.moved && w.came && w.moved) {
                all_moved = static_cast<uint8_t>(all_moved + 2u);
            }
            if (f.at_idle) {
                ++flag_idle;
            }
            if (w.at_idle) {
                ++flag_idle;
            }
        }
    }
    DmaHost::release();
    dma_ready();
    uint32_t vector_worst = 0;
    for (const SpiClock c : rungs) {
        for (uint8_t m = 0; m < 4u; ++m) {
            const FlagPass f = channel_pass(static_cast<SpiMode>(m), c);
            vector_worst = vector_read_after > vector_worst ? vector_read_after : vector_worst;
            print(serial, "  the receive channel /", 2u << static_cast<uint8_t>(c), " mode ", m, ": at its flag SCK ",
                  f.at_idle ? "idle" : "OFF IDLE", ", ", f.polls_to_idle, " polls to idle, ", f.polls, " polls a frame",
                  crlf);
            console_drain();
            if (f.came && f.moved) {
                ++all_moved;
            }
            if (f.at_idle) {
                ++channel_idle;
            }
        }
    }
    print(serial, "  the vector reads the pad at most ", vector_worst, " cycles after the flag was seen", crlf);
    bench.verdict("THE PAD IS THE WIRE: on every pass, /256 and /64 and all four modes, a poll saw SCK off its idle level "
                  "before the flag came, and the flag came",
                  all_moved == flag_passes + 8u);
    bench.verdict("RXNE DOES NOT LEAD THE LAST EDGE: at the flag SCK is at the mode's idle level, 8 and 16 bits, all four "
                  "modes, /256 and /64",
                  flag_idle == flag_passes);
    bench.verdict("NOR DOES THE RECEIVE CHANNEL'S COMPLETION: the same on its flag", channel_idle == 8u);
    uint16_t asked = 0;
    uint16_t off_polled = 0;
    uint16_t off_engines = 0;
    uint16_t bad = 0;
    for (const SpiClock c : rungs) {
        end_of_request<DmaHost>(c, asked, off_polled, off_engines, bad);
    }
    DmaHost::release();
    dma_ready<FlashDmaHost>();
    for (const SpiClock c : rungs) {
        end_of_request<FlashDmaHost>(c, asked, off_polled, off_engines, bad);
    }
    FlashDmaHost::release();
    host_ready();
    print(serial, "  ", asked, " pairs of requests: SCK off idle after start() ", off_polled, ", after dma_isr() ",
          off_engines, ", a bad answer or a select down ", bad, crlf);
    bench.verdict("WHEN A POLLED REQUEST ENDS THE WIRE HAS ENDED: SCK at its idle level on the instruction after start(), "
                  "all four modes, /256 and /64, read and write, a select and a null one, both placements",
                  asked != 0u && off_polled == 0u);
    bench.verdict("AND WHEN ONE ON THE ENGINES ENDS: the same in the vector, right after dma_isr() answered true",
                  asked != 0u && off_engines == 0u);
    bench.verdict("every request of the pass answered spi_ok, start() true on the loop and false on the engines, the "
                  "select high",
                  asked != 0u && bad == 0u);
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
    // SpiEngine::dma IS DMA: the engineless host refuses a request naming
    // SpiEngine::dma, at once and before it touches anything - asked in mode 3 at
    // /2 after the block was primed in mode 0 at /16, the select PA4 in
    // the request: start() answers true with spi_rejected, PA4 stays high,
    // CTLR1 is the word it was (no CPOL flip on SCK), and SCK reads its mode
    // 0 idle, low.
    {
        static_assert(!Host::has_dma && DmaHost::has_dma, "the suite's two hosts, without and with engines");
        Host::prime(SpiMode::mode0, SpiClock::div16);
        const uint16_t c1 = S::regs().CTLR1;
        Host::Request r{};
        r.cs = CsPin::ref();
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
        r.rx = lend<Lease::reply>(static_cast<uint8_t*>(rx_buf));
        r.len = 16;
        r.clock = SpiClock::div2;
        r.mode = SpiMode::mode3;
        r.engine = SpiEngine::dma;
        const bool done = Host::start(r);
        const uint8_t st = Host::status();
        const bool cs_high = CsPin::read_out() && CsPin::read();
        const bool word_kept = S::regs().CTLR1 == c1;
        const bool sck_idle = !SckPin::read();
        print(serial, "  a request naming the engines on the engineless host: start() ", done ? "true" : "FALSE", ", status ",
              st, " (spi_rejected ", spi_rejected, "), the select ", cs_high ? "high" : "LOW", ", CTLR1 ",
              word_kept ? "unchanged" : "REWRITTEN", ", SCK ", sck_idle ? "low" : "HIGH", crlf);
        bench.verdict("A REQUEST NAMING THE ENGINES ON A HOST WITH NONE IS REFUSED: start() answers at once "
                      "spi_rejected, the select never falls, the control word and SCK are untouched",
                      done && st == spi_rejected && cs_high && word_kept && sck_idle);
        // and the same host, polled, runs the same request.
        r.engine = SpiEngine::polled;
        const bool polled_done = Host::start(r);
        bench.verdict("and the same request marked polled runs on it, spi_ok", polled_done && Host::status() == spi_ok);
    }
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
// f - the fault exits, recover() and release()'s reset pulse (wireless)
// ===========================================================================
//
// Every exit of the host that no healthy wire takes, provoked on purpose
// with the select high (the far client deselected, R2): spi_stalled - a
// polled request whose flag never comes, the block HELD IN RESET under it
// (its registers ignore every write, RXNE never rises) -, spi_dma_fault - a
// polled request on the engines over a DEAD receive channel, TIM1's
// compare request held on DMA channel 2 (its channel 1's row, figure 9-1)
// with no channel to take it, its gate closed with no reset (the I2C
// suite's letter p measured the hold on this die) -, the asynchronous
// engines over the same dead channel answered by the caller's bound and
// recover(); and the block's own transmit request left on channel 3 under
// TXDMAEN, let go four ways, release() the one that must leave nothing.

using DeadRx = Tim<1>;   ///< its channel 1's compare request: DMA channel 2
constexpr uint8_t dead_rx_ch = 0;
static_assert(DeadRx::compare_dma_channel(dead_rx_ch) == 2, "figure 9-1: TIM1_CH1 rides channel 2 with SPI1_RX");

/// A compare request RAISED AND HELD: CCxDE set, the counter run from zero
/// at HCLK past one match (10 counts in, a period of 1000: some fifty
/// microseconds, two matches), stopped, the gate closed with no reset.
template <class T>
void hold_compare_request(uint8_t ch) {
    T::bus_clock(true);
    TimRegs& r = T::regs();
    r.PSC = 0;
    r.ATRLR = 999;
    r.CHCVR[ch] = 10;
    r.CNT = 0;
    r.DMAINTENR = static_cast<uint16_t>(1u << (9u + ch));
    r.CTLR1 = tim_cen;
    (void)delay_us(clock, 50);
    r.CTLR1 = 0;
    T::bus_clock(false);
}

/// The release contract on the timer: the reset line, then the gate.
template <class T>
void free_compare_request() {
    T::bus_clock(true);
    T::reset();
    T::bus_clock(false);
}

/// The NEXT OWNER of a channel: a transmit block of four bytes into a RAM
/// cell, request-paced, asked by nothing - any item it moves is a request
/// still standing on the channel's OR. Its count read 300 us after the
/// enable.
volatile uint8_t sink_cell = 0;
template <uint8_t ch>
uint16_t next_owner_moves() {
    static const uint8_t four[4] = {1, 2, 3, 4};
    using C = DmaChannel<ch>;
    C::stop();
    (void)C::prepare(DmaTransfer{.peripheral = &sink_cell, .memory = const_cast<uint8_t*>(four), .count = 4,
                                 .config = {.direction = DmaDirection::memory_to_peripheral,
                                            .memory_increment = true, .priority = DmaPriority::high}});
    C::enable(true);
    (void)delay_us(clock, 300);
    const uint16_t moved = static_cast<uint16_t>(4u - C::count());
    C::stop();
    return moved;
}

// THE STALLS. spi_stalled is the polled path's answer to a flag that never comes -
// a clock that stopped, a block that is dead - spent from ONE budget for the
// whole transaction. The host releases the select and puts the block back itself,
// so the next request needs no recover(). Staged two ways a reset cures: SPI1 HELD
// IN RESET, its registers ignoring every write and RXNE never rising, the host's
// own pulse being what lets the line go; and SPE TAKEN DOWN under the host, a frame
// left standing in the transmit buffer and TXE never rising again, the host's
// compare seeing no change because the request's clock, mode and width are primed
// first. Then the one failure a reset does not cure, the gate closed, a transfer
// error, and a block pointed at each hole in the map. What came back is letter t's
// to judge, against the peer.
//
// THE WALL'S BOUND. At /16 and 8 bits a frame's budget is 2^7 turns, so sixteen
// frames spend (16 + 2) x 128 = 2304 turns, and a command phase of one frame
// (1 + 2) x 128 = 384. The dearest turn measured is 25.4 cycles (from the flash; a
// load, a test, a count and two branches), and moving a loop half a fetch word
// moves it by up to 6 (docs/ch32x035/spi.md, "The fetch"): 32 cycles a turn. On
// top, 3 000 cycles: two ticks' handlers in the span, the host's way in, and the
// reset. So 2304 x 32 + 3000 = 76 728 and 384 x 32 + 3000 = 15 288.
constexpr uint32_t wall_of_sixteen = 2304UL * 32UL + 3000UL;
constexpr uint32_t wall_of_command = 384UL * 32UL + 3000UL;

/// The ways a request is made to stall, and the two a transfer error takes.
enum class Stage : uint8_t { held_in_reset, spe_down_write, spe_down_command, fault_rx, fault_tx };

const char* stage_name(Stage s) {
    switch (s) {
        case Stage::held_in_reset: return "a read over a block held in reset";
        case Stage::spe_down_write: return "a write with SPE taken down";
        case Stage::spe_down_command: return "a command phase with SPE taken down";
        case Stage::fault_rx: return "a transfer error on the receive channel";
        default: return "a transfer error on the transmit channel";
    }
}

/// The stall staged, on the lone select at /16, `mode` already primed (the
/// caller's): the status, the wall from start() to its return in `took`.
template <typename H>
uint8_t stall_stage(Stage s, SpiMode mode, uint32_t& took) {
    constexpr SpiDataSize bits = SpiDataSize::bits8;
    uint8_t st = no_answer;
    uint32_t t0 = 0;
    switch (s) {
        case Stage::held_in_reset:
            rcc()->APB2PRSTR = rcc()->APB2PRSTR | rcc_pb2_spi1;
            t0 = cycles_now();
            st = dma_xfer<H>(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 16, SpiClock::div16, bits, true, mode);
            break;
        case Stage::spe_down_write:
            S::regs().CTLR1 = static_cast<uint16_t>(S::regs().CTLR1 & ~spi_spe);
            t0 = cycles_now();
            st = dma_xfer<H>(LoneCs::ref(), nullptr, 0, tx_buf, nullptr, 16, SpiClock::div16, bits, true, mode);
            break;
        default:
            // Sixty-four frames for the engines; the one command frame runs in
            // the loop in front of them.
            S::regs().CTLR1 = static_cast<uint16_t>(S::regs().CTLR1 & ~spi_spe);
            t0 = cycles_now();
            st = dma_xfer<H>(LoneCs::ref(), cmd_buf, 1, tx_buf, rx_buf, 64, SpiClock::div16, bits, false, mode);
            break;
    }
    took = cycles_now() - t0;
    return st;
}

/// Each stall path on one placement of the engined host, each followed at once
/// by a polled and an asynchronous request. Mode 3: SCK idles high, so the reset
/// moves its pad, with the select already released.
template <typename H>
void stall_paths(uint8_t& good_stalls, uint8_t& good_next) {
    constexpr SpiMode mode = SpiMode::mode3;
    for (const Stage s : {Stage::held_in_reset, Stage::spe_down_write, Stage::spe_down_command}) {
        dma_ready<H>();
        H::prime(mode, SpiClock::div16);
        dma_isr_entries = 0;
        uint32_t wall = 0;
        const uint8_t stalled = stall_stage<H>(s, mode, wall);
        const uint32_t stall_entries = dma_isr_entries;
        const bool lone_high = LoneCs::read_out();
        const uint32_t bound = s == Stage::spe_down_command ? wall_of_command : wall_of_sixteen;
        const uint8_t next_polled = dma_xfer<H>(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 16, SpiClock::div16,
                                                SpiDataSize::bits8, true, mode);
        const uint32_t polled_entries = dma_isr_entries;
        dma_isr_entries = 0;
        const uint8_t next_async = dma_xfer<H>(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 16, SpiClock::div16,
                                               SpiDataSize::bits8, false, mode);
        const uint32_t async_entries = dma_isr_entries;
        H::release();
        print(serial, "  ", H::placed_in_ram ? "SRAM: " : "flash: ", stage_name(s), ": status ", stalled, " after ",
              wall, " cycles (the bound ", bound, "), select ", lone_high ? "high" : "LOW", ", channel interrupts ",
              stall_entries, "; then polled ", next_polled, " (", polled_entries, " interrupts), asynchronous ",
              next_async, " (", async_entries, ")", crlf);
        if (stalled == spi_stalled && wall < bound && lone_high && stall_entries == 0u) {
            ++good_stalls;
        }
        if (next_polled == spi_ok && polled_entries == 0u && next_async == spi_ok && async_entries == 1u) {
            ++good_next;
        }
    }
}

/// The ends of the memories: RM 9.2.1 promises a transfer error for a reserved
/// address, and on this silicon it is these that give it, where a hole elsewhere
/// in the map completes as an ordinary block (the survey below prints both). A
/// store past the SRAM raises it too, but on two of the family's parts the store
/// wraps onto the SRAM's first bytes (docs, the DMA page), so the suite stores
/// past the flash, where no programming is armed and nothing lands.
constexpr uint32_t past_sram = 0x20000000u + device::sram_bytes;
constexpr uint32_t past_flash = device::flash_bytes;
constexpr uint16_t faulted_frames = 32;

/// A block a real transfer error stops, thirty-two frames at /256 on the lone
/// select. On the receive channel the first frame's store falls past the flash,
/// so the channel stops one frame in; on the transmit channel the buffer's last
/// eight bytes are the SRAM's (stack, read and not written), so it stops eight
/// frames in.
template <typename H>
typename H::Request fault_request(Stage s, SpiMode mode) {
    typename H::Request r{};
    r.cs = LoneCs::ref();
    if (s == Stage::fault_rx) {
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
        r.rx = lend<Lease::reply>(reinterpret_cast<uint8_t*>(past_flash));
    } else {
        r.tx = lend<Lease::reply>(reinterpret_cast<const uint8_t*>(past_sram - 8u));
        r.rx = lend<Lease::reply>(static_cast<uint8_t*>(rx_buf));
    }
    r.len = faulted_frames;
    r.mode = mode;
    r.clock = link_clock;
    r.engine = SpiEngine::dma;
    return r;
}

/// The bound on a completion the vector signals: the suite's own spin.
bool await_done() {
    for (uint32_t i = 0; i < 8'000'000UL && !host_done; ++i) {
    }
    return host_done;
}

/// The transfer errors, each followed at once by a polled and an asynchronous
/// request; what came back is letter t's to judge.
template <typename H>
bool refused_after_fault() {
    typename H::Request r{};
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf + 1));
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(rx_buf));
    r.len = 8;
    r.clock = link_clock;
    r.bits = SpiDataSize::bits16;
    r.engine = SpiEngine::dma;
    return H::start(r) && H::status() == spi_rejected;
}

template <typename H>
void real_faults(uint8_t& good_faults, uint8_t& good_next) {
    constexpr SpiMode mode = SpiMode::mode3;
    for (const Stage s : {Stage::fault_rx, Stage::fault_tx}) {
        dma_ready<H>();
        H::prime(mode, link_clock);
        const typename H::Request f = fault_request<H>(s, mode);
        host_done = false;
        dma_isr_entries = 0;
        const bool inside = H::start(f);
        const bool answered = await_done();
        const uint8_t st = H::status();
        const uint32_t entries = dma_isr_entries;
        const bool lone_high = LoneCs::read_out();
        const uint8_t next_polled = dma_xfer<H>(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 16, link_clock,
                                                SpiDataSize::bits8, true, mode);
        const uint8_t next_async = dma_xfer<H>(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 16, link_clock,
                                               SpiDataSize::bits8, false, mode);
        H::release();
        print(serial, "  ", H::placed_in_ram ? "SRAM: " : "flash: ", stage_name(s), ": dma_isr() ",
              answered ? "true" : "never", ", status ", st, " (spi_dma_fault ", spi_dma_fault, "), ", entries,
              " interrupts, select ", lone_high ? "high" : "LOW", "; then polled ", next_polled, ", asynchronous ",
              next_async, crlf);
        if (!inside && answered && st == spi_dma_fault && entries == 1u && lone_high) {
            ++good_faults;
        }
        if (next_polled == spi_ok && next_async == spi_ok) {
            ++good_next;
        }
    }

    // A REFUSED REQUEST LEAVES THE RESET OWED: it touches nothing, so ERRIE, set by
    // hand after the fault with the PFIC line left closed, stands through it and
    // falls to the reset the next prime() pays. And what the owed reset costs:
    // prime() on the configuration it holds does nothing else, so the same call
    // with the reset owed and with nothing owed differ by the reset.
    dma_ready<H>();
    H::prime(mode, link_clock);
    const typename H::Request f = fault_request<H>(Stage::fault_rx, mode);
    host_done = false;
    (void)H::start(f);
    (void)await_done();
    S::error_interrupt(true);
    const bool still_refused = refused_after_fault<H>();
    const bool errie_through = (S::regs().CTLR2 & spi_errie) != 0u;
    uint32_t t0 = cycles_now();
    H::prime(mode, link_clock);
    const uint32_t owed = cycles_now() - t0;
    const bool errie_gone = (S::regs().CTLR2 & spi_errie) == 0u;
    t0 = cycles_now();
    H::prime(mode, link_clock);
    const uint32_t paid = cycles_now() - t0;
    H::release();
    print(serial, "  ", H::placed_in_ram ? "SRAM: " : "flash: ", "a refused request after a fault ",
          still_refused ? "refused" : "NOT REFUSED", ", the block ", errie_through ? "untouched" : "TOUCHED", ", prime() "
          "with the reset owed ", owed, " cycles", errie_gone ? "" : " (NO RESET)", ", with nothing owed ", paid, crlf);
    if (still_refused && errie_through && errie_gone) {
        ++good_next;
    }
}

/// Each hole, read and written by the engines through a real request.
struct HoleSurvey {
    uint8_t blocks = 0;
    uint8_t faults = 0;
    uint8_t hung = 0;
};

void hole_line(HoleSurvey& h, const char* way, uint32_t address, uint8_t st) {
    print(serial, "  a block ", way, " ", hex(address), ": status ", st, crlf);
    ++h.blocks;
    if (st == spi_dma_fault) {
        ++h.faults;
    } else if (st != spi_ok) {
        ++h.hung;
    }
}

void stalls_and_holes() {
    LoneCs::output(true);

    // 1. The host with no engines, a polled read of sixteen at /16.
    host_ready<FlashHost>();
    rcc()->APB2PRSTR = rcc()->APB2PRSTR | rcc_pb2_spi1;
    const uint32_t t0 = cycles_now();
    const uint8_t st = host_xfer<FlashHost>(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode0,
                                            SpiClock::div16, SpiDataSize::bits8, true);
    const uint32_t took = cycles_now() - t0;
    const bool released = LoneCs::read_out();
    const uint16_t ctlr1 = S::regs().CTLR1;
    const uint16_t statr = S::status();
    const uint8_t after = host_xfer<FlashHost>(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode0,
                                               SpiClock::div16, SpiDataSize::bits8, true);
    print(serial, "  a polled read of 16 at /16 with the block held in reset: status ", st, " (spi_stalled ",
          spi_stalled, ") after ", took, " cycles (the bound ", wall_of_sixteen, ", the wire's 2048), select ",
          released ? "high" : "LOW", ", CTLR1 ", hex(ctlr1), " STATR ", hex(statr), ", the next request ", after, crlf);
    bench.verdict("A POLLED REQUEST OVER A BLOCK HELD IN RESET ends spi_stalled within its derived bound, the select "
                  "released",
                  st == spi_stalled && took < wall_of_sixteen && released);
    bench.verdict("the host has put the block back: the reset line let go, SPE and MSTR up, nothing in the receive "
                  "buffer, and with no recover() the next request is spi_ok",
                  (ctlr1 & (spi_spe | spi_mstr)) == (spi_spe | spi_mstr) && (statr & (spi_rxne | spi_ovr)) == 0u &&
                      after == spi_ok);

    // 2. The host with engines, every stall path, in both placements.
    uint8_t good_stalls = 0;
    uint8_t good_next = 0;
    stall_paths<FlashDmaHost>(good_stalls, good_next);
    stall_paths<DmaHost>(good_stalls, good_next);
    bench.verdict("EVERY STALL PATH ON THE ENGINED HOST, IN BOTH PLACEMENTS, the polled command phase in front of an "
                  "asynchronous block among them, ends spi_stalled inside start() within its bound, the select "
                  "released, no channel interrupt",
                  good_stalls == 6u);
    bench.verdict("and each time, with no recover(), the next polled request and the next asynchronous one, on the "
                  "engines with their one interrupt, are spi_ok",
                  good_next == 6u);

    // 3. A CLOSED GATE, which a reset does not open. Each request stalls on its
    // own budget; the gate opened, at most one more stall puts the block back.
    host_ready<FlashHost>();
    S::bus_clock(false);
    const uint16_t closed_ctlr1 = S::regs().CTLR1;
    const uint16_t closed_statr = S::status();
    uint32_t tg = cycles_now();
    const uint8_t gate_1 = host_xfer<FlashHost>(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode0,
                                                SpiClock::div16, SpiDataSize::bits8, true);
    const uint32_t gate_1_took = cycles_now() - tg;
    tg = cycles_now();
    const uint8_t gate_2 = host_xfer<FlashHost>(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode0,
                                                SpiClock::div16, SpiDataSize::bits8, true);
    const uint32_t gate_2_took = cycles_now() - tg;
    tg = cycles_now();
    const bool drained = S::disable();
    const uint32_t drain_took = cycles_now() - tg;
    S::bus_clock(true);
    const uint8_t open_1 = host_xfer<FlashHost>(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode0,
                                                SpiClock::div16, SpiDataSize::bits8, true);
    const uint8_t open_2 = host_xfer<FlashHost>(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode0,
                                                SpiClock::div16, SpiDataSize::bits8, true);
    print(serial, "  the gate closed: CTLR1 reads ", hex(closed_ctlr1), ", STATR ", hex(closed_statr),
          "; two requests ", gate_1, " after ", gate_1_took, " and ", gate_2, " after ", gate_2_took,
          " cycles; Spi::disable() ", drained ? "true" : "false", " after ", drain_took,
          " cycles; the gate opened, requests ", open_1, " and ", open_2, crlf);
    bench.verdict("A CLOSED GATE IS NOT CURED: every request behind it stalls, each on its own budget within the bound",
                  gate_1 == spi_stalled && gate_2 == spi_stalled && gate_1_took < wall_of_sixteen &&
                      gate_2_took < wall_of_sixteen);
    bench.verdict("the gate opened, the host needs no recover(): at most one more stall, and the next request is spi_ok",
                  (open_1 == spi_ok || open_1 == spi_stalled) && open_2 == spi_ok);
    FlashHost::release();

    // 4. A TRANSFER ERROR, real, with no recover() after it, in both placements.
    // The channels' stop in the middle of a block, the select released from the
    // vector, the next requests at once. Exact against the peer in letter t.
    uint8_t good_faults = 0;
    uint8_t good_after_fault = 0;
    real_faults<FlashDmaHost>(good_faults, good_after_fault);
    real_faults<DmaHost>(good_faults, good_after_fault);
    bench.verdict("A BLOCK RUNNING PAST THE END OF A MEMORY, read or written, is a transfer error: dma_isr() true once, "
                  "spi_dma_fault, the select released",
                  good_faults == 4u);
    bench.verdict("and with no recover() the next polled request and the next on the engines are spi_ok; a refused "
                  "request behind a fault leaves the block alone and the reset owed, the next prime() pays it",
                  good_after_fault == 6u);

    // 5. THE REST OF THE MAP: a real request whose buffer sits on a hole
    // (test_x035_dma letter d's five, read and written), and for reading two more
    // classes - the closed gate of TIM1 and the end of the flash. Printed, and
    // judged for what must hold either way: a block ends.
    HoleSurvey holes{};
    constexpr uint32_t both_ways[] = {0x30000000u, 0x40030000u, 0x50000000u, 0x60000000u, 0xA0000000u};
    constexpr uint32_t read_only[] = {0x40012C00u, device::flash_bytes};
    DeadRx::bus_clock(false);
    dma_ready<FlashDmaHost>();
    for (const uint32_t a : both_ways) {
        hole_line(holes, "read from", a,
                  dma_xfer<FlashDmaHost>(LoneCs::ref(), nullptr, 0, reinterpret_cast<const uint8_t*>(a), rx_buf, 16,
                                         SpiClock::div16, SpiDataSize::bits8, false));
        hole_line(holes, "written into", a,
                  dma_xfer<FlashDmaHost>(LoneCs::ref(), nullptr, 0, tx_buf, reinterpret_cast<uint8_t*>(a), 16,
                                         SpiClock::div16, SpiDataSize::bits8, false));
    }
    for (const uint32_t a : read_only) {
        hole_line(holes, "read from", a,
                  dma_xfer<FlashDmaHost>(LoneCs::ref(), nullptr, 0, reinterpret_cast<const uint8_t*>(a), rx_buf, 16,
                                         SpiClock::div16, SpiDataSize::bits8, false));
    }
    const uint8_t after_holes = dma_xfer<FlashDmaHost>(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 16, SpiClock::div16,
                                                       SpiDataSize::bits8, false);
    FlashDmaHost::release();
    print(serial, "  ", holes.blocks, " blocks over the rest of the map: ", holes.faults, " transfer errors, ",
          holes.hung, " with no answer", crlf);
    bench.verdict("every one of those blocks ended, and the next asynchronous request is spi_ok",
                  holes.hung == 0u && after_holes == spi_ok);
    LoneCs::release();
}

void tf_faults() {
    fill_pattern(tx_buf, 64, 0x5A);
    stalls_and_holes();

    // 2. A POLLED REQUEST OVER A DEAD CHANNEL: 64 frames on the engined host
    // with the receive channel 2 dead: the loop, which touches no channel,
    // ends spi_ok with no channel interrupt. The hold proved first by a next
    // owner (one stray item, the channel then dead).
    const uint16_t control2 = next_owner_moves<2>();
    hold_compare_request<DeadRx>(dead_rx_ch);
    const uint16_t held2 = next_owner_moves<2>();
    dma_ready<FlashDmaHost>();
    uint32_t t0 = cycles_now();
    const uint32_t polled_e0 = dma_isr_entries;
    const uint8_t st_dead_polled = dma_xfer<FlashDmaHost>({}, nullptr, 0, tx_buf, rx_buf, 64, SpiClock::div16,
                                                          SpiDataSize::bits8, true);
    const uint32_t dead_polled_cycles = cycles_now() - t0;
    const uint32_t dead_polled_entries = dma_isr_entries - polled_e0;
    // 3. THE ASYNCHRONOUS ENGINES over the same dead channel: no completion
    // edge ever comes; the caller's bound (dma_xfer's spin) and recover().
    const uint8_t st_async = dma_xfer<FlashDmaHost>({}, nullptr, 0, tx_buf, rx_buf, 64, SpiClock::div16,
                                                    SpiDataSize::bits8, false);
    free_compare_request<DeadRx>();
    const bool rec2 = FlashDmaHost::recover();
    const uint8_t st_live_polled = dma_xfer<FlashDmaHost>({}, nullptr, 0, tx_buf, rx_buf, 64, SpiClock::div16,
                                                          SpiDataSize::bits8, true);
    const uint8_t st_live_async = dma_xfer<FlashDmaHost>({}, nullptr, 0, tx_buf, rx_buf, 64, SpiClock::div16,
                                                         SpiDataSize::bits8, false);
    FlashDmaHost::release();
    print(serial, "  next owner of channel 2, nothing raised ", control2, ", after TIM1's held request ", held2,
          " (of 4); over the dead channel: a polled request ", st_dead_polled, " after ", dead_polled_cycles,
          " cycles, ", dead_polled_entries, " channel interrupts, the asynchronous engines ", st_async, " (no answer ",
          no_answer,
          "); TIM1 reset and recover() ", rec2 ? 1 : 0, ": polled ", st_live_polled, ", asynchronous ",
          st_live_async, crlf);
    bench.verdict("with nothing raised the next owner of channel 2 moves nothing; after TIM1's held request "
                  "it takes one stray item - the dead channel staged",
                  control2 == 0u && held2 == 1u);
    bench.verdict("A POLLED REQUEST ON THE ENGINED HOST OVER THE DEAD CHANNEL runs its loop: spi_ok, no channel "
                  "interrupt, a polled request depending on no channel and no vector",
                  st_dead_polled == spi_ok && dead_polled_entries == 0u);
    bench.verdict("the asynchronous engines over it never claim a completion: the caller's bound answers it, "
                  "and once TIM1 is reset and recover() has run both paths complete spi_ok",
                  st_async == no_answer && rec2 && st_live_polled == spi_ok && st_live_async == spi_ok);

    // 4. THE RELEASE PULSE: the block's own transmit request. SPE up as a
    // host with nothing written - TXE stands - and TXDMAEN set with no
    // channel enabled: the request rises on channel 3. Let go four ways,
    // each on a fresh raise: none (the request still asserted: the staging
    // proved), the gate closed alone, SPE cleared and the gate closed, and
    // release() - the reset line before the gate - which must leave the
    // next owner nothing.
    uint16_t moved[4] = {};
    for (uint8_t way = 0; way < 4u; ++way) {
        dma_ready<FlashDmaHost>();
        FlashDmaHost::prime(SpiMode::mode0, SpiClock::div16);
        DmaChannel<3>::stop();
        S::dma_requests(true, false);
        (void)delay_us(clock, 20);
        if (way == 1u) {
            S::bus_clock(false);
        } else if (way == 2u) {
            (void)S::disable();
            S::bus_clock(false);
        } else if (way == 3u) {
            FlashDmaHost::release();
        }
        moved[way] = next_owner_moves<3>();
        S::bus_clock(true);
        S::reset();
        FlashDmaHost::release();
    }
    print(serial, "  the block's own request on channel 3 (TXE under TXDMAEN), the next owner moved: left live ",
          moved[0], ", the gate closed ", moved[1], ", SPE cleared and the gate closed ", moved[2],
          ", after release() ", moved[3], " (of 4)", crlf);
    bench.verdict("TXE under TXDMAEN raised the block's own request on channel 3: left live, the next owner "
                  "takes an item",
                  moved[0] >= 1u);
    bench.verdict("release() - the reset line before the gate - leaves the next owner nothing", moved[3] == 0u);
    bench.verdict("and no way of letting go hands the next owner more than the one item a held request is",
                  moved[1] <= 1u && moved[2] <= 1u);
    host_ready<Host>();
}

// ===========================================================================
// s - the /2 frame and the core's share of the bus (a diagnostic, by name)
// ===========================================================================
//
// Letter b's engined frame at /2 once read 1045 thousandths of the wire
// where its other runs read 1006 to 1020. The core cannot sleep through
// the block to take itself off the bus - in this die's Sleep the DMA moves
// nothing (docs/ch32x035/dma.md) - so the two waits are compared awake:
// the suite's own, a poll of the completion flag in SRAM every few cycles,
// and one OFF THE BUS, delay_us() on the core's own STK between flag
// looks five microseconds apart, the block's end stamped by the DMA
// vector itself. A frame inside the block is the difference of a 256- and
// a 32-frame block, as letter b takes it, twenty times each way.

volatile uint32_t dma_done_at = 0;

/// One engined block of `n` frames at /2, its time from start() to the
/// completion: polled in SRAM (`quiet` false) or waited off the bus.
uint32_t block_at_div2(uint16_t n, bool quiet) {
    FlashDmaHost::Request r{};
    r.engine = SpiEngine::dma;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(rx_buf));
    r.len = n;
    r.mode = SpiMode::mode0;
    r.clock = SpiClock::div2;
    r.bits = SpiDataSize::bits8;
    host_done = false;
    const uint32_t t0 = cycles_now();
    if (FlashDmaHost::start(r)) {
        return cycles_now() - t0;
    }
    if (quiet) {
        for (uint32_t i = 0; i < 2000u && !host_done; ++i) {
            (void)delay_us(clock, 5);
        }
        return host_done ? dma_done_at - t0 : 0u;
    }
    for (uint32_t i = 0; i < 8'000'000UL && !host_done; ++i) {
    }
    return host_done ? cycles_now() - t0 : 0u;
}

void ts_bus_share() {
    dma_ready<FlashDmaHost>();
    fill_pattern(tx_buf, 512, 0x3C);
    console_drain();
    for (uint8_t way = 0; way < 2u; ++way) {
        uint32_t lo = 0xFFFFFFFFu, hi = 0, over = 0;
        for (uint8_t k = 0; k < 20u; ++k) {
            uint32_t big = 0xFFFFFFFFu, small = 0xFFFFFFFFu;
            for (uint8_t j = 0; j < 3u; ++j) {
                const uint32_t b = block_at_div2(256, way != 0u);
                const uint32_t s = block_at_div2(32, way != 0u);
                big = b < big ? b : big;
                small = s < small ? s : small;
            }
            const uint32_t pm = big > small ? (big - small) * 1000u / (224u * 16u) : 0u;
            lo = pm < lo ? pm : lo;
            hi = pm > hi ? pm : hi;
            if (!wire_ok(pm)) {
                ++over;
            }
        }
        print(serial, way != 0u ? "  waited OFF the bus (delay_us, the vector's stamp): "
                                : "  the completion flag polled in SRAM:                ",
              "the /2 frame ", lo, " to ", hi, " thousandths of the wire in 20 runs, ", over,
              " outside -1..+3 %", crlf);
    }
    FlashDmaHost::release();
    host_ready<Host>();
    bench.verdict("ran", true);
}

// ===========================================================================
// d - does RXNE say TXE? the witness (wireless, by name, no verdict)
// ===========================================================================
//
// On the resource, frame k shifting while frame k + 1 waits in the transmit
// buffer. At frame k's end two flags rise: RXNE, at "the last sample
// clock edge of the data word", and TXE, "when the data has reached the shift
// register" (16.2.2). The chapter does not order them. This letter asks the
// silicon: the host's own transmit side, nothing on the wire but its own
// clock - the select high, MISO floating and never read as data - so no
// client is needed. Each turn polls STATR from SRAM until one load shows
// either flag and files the frame under what that load showed - RXNE alone,
// TXE alone, both - and with one alone reads STATR four times more back to
// back, the first of them to show the other flag being how far apart the two
// rose, in loads. Then the DATAR read and the next write. Interrupts are
// masked over a run: a handler landing in the poll would file the frame under
// "both" and say nothing about the block. What a line can resolve is the
// load's and the poll's period, printed under the table.

struct WitnessRun {
    uint16_t rxne_alone = 0;   ///< a load showed RXNE with TXE low
    uint16_t together = 0;     ///< the first load showing either showed both
    uint16_t txe_alone = 0;    ///< a load showed TXE with RXNE low
    uint16_t lost = 0;         ///< OVR after the DATAR read
    uint16_t stuck = 0;        ///< a flag never came within the turn's bound
    uint16_t next_load[5] = {};   ///< the other flag at the 1st..4th load after, [4] later
};

/// One run of `count` frames (`count` >= 3), each written behind the one
/// shifting, into `w`.
[[gnu::noinline, gnu::flatten, gnu::section(".fast_text")]] void witness_run(uint16_t count, WitnessRun& w) {
    SpiRegs& regs = S::regs();
    constexpr uint32_t bound = 200'000u;   // far past a 4096-cycle frame at any turn
    constexpr uint32_t both = spi_rxne | spi_txe;
    regs.DATAR = 0xFFFFu;
    for (uint32_t i = bound; (regs.STATR & spi_txe) == 0u && i != 0u; --i) {
    }
    regs.DATAR = 0xFFFFu;
    for (uint32_t to_write = count - 2u; to_write != 0u; --to_write) {
        uint32_t st;
        uint32_t i = bound;
        do {
            st = regs.STATR;
        } while ((st & both) == 0u && --i != 0u);
        // Back to back, kept and judged afterwards: nothing between the
        // loads but the core's own fetch and bus.
        const uint32_t s1 = regs.STATR;
        const uint32_t s2 = regs.STATR;
        const uint32_t s3 = regs.STATR;
        const uint32_t s4 = regs.STATR;
        if (i == 0u) {
            ++w.stuck;
            return;
        }
        const uint32_t seen = st & both;
        if (seen == both) {
            ++w.together;
        } else {
            const uint32_t want = seen ^ both;
            if (seen == spi_rxne) {
                ++w.rxne_alone;
            } else {
                ++w.txe_alone;
            }
            const uint8_t at = (s1 & want) != 0u ? 0u : (s2 & want) != 0u ? 1u : (s3 & want) != 0u ? 2u : (s4 & want) != 0u ? 3u : 4u;
            ++w.next_load[at];
            for (i = bound; (regs.STATR & want) == 0u && i != 0u; --i) {
            }
        }
        (void)regs.DATAR;
        if ((regs.STATR & spi_ovr) != 0u) {
            ++w.lost;
        }
        regs.DATAR = 0xFFFFu;
    }
    // The two frames still in flight: bounded, their answers thrown away.
    for (uint8_t left = 2; left != 0u; --left) {
        for (uint32_t i = bound; (regs.STATR & spi_rxne) == 0u && i != 0u; --i) {
        }
        (void)regs.DATAR;
        (void)regs.STATR;
    }
}

/// The periods a line resolves: one of witness_run()'s back-to-back loads
/// and one turn of its poll, x10, with TXE and RXNE both low in a frame at
/// /256 for the poll and any state for the loads.
[[gnu::noinline, gnu::section(".fast_text")]] uint32_t witness_load_x10() {
    SpiRegs& regs = S::regs();
    const uint32_t t0 = stamp();
    for (uint8_t k = 0; k < 4u; ++k) {
        (void)regs.STATR; (void)regs.STATR; (void)regs.STATR; (void)regs.STATR;
        (void)regs.STATR; (void)regs.STATR; (void)regs.STATR; (void)regs.STATR;
        (void)regs.STATR; (void)regs.STATR; (void)regs.STATR; (void)regs.STATR;
        (void)regs.STATR; (void)regs.STATR; (void)regs.STATR; (void)regs.STATR;
    }
    return stamp_span(t0, stamp()) * 10u / 64u;
}
[[gnu::noinline, gnu::section(".fast_text")]] uint32_t witness_poll_x10() {
    SpiRegs& regs = S::regs();
    constexpr uint32_t both = spi_rxne | spi_txe;
    regs.DATAR = 0xFFFFu;
    for (uint32_t i = 200'000u; (regs.STATR & spi_txe) == 0u && i != 0u; --i) {
    }
    regs.DATAR = 0xFFFFu;   // one behind the other: both flags low until frame 0's end
    const uint32_t t0 = stamp();
    uint32_t turns = 0;
    uint32_t i = 200'000u;
    uint32_t st;
    do {
        st = regs.STATR;
        ++turns;
    } while ((st & both) == 0u && --i != 0u);
    const uint32_t took = stamp_span(t0, stamp());
    for (uint8_t left = 2; left != 0u; --left) {
        for (uint32_t j = 200'000u; (regs.STATR & spi_rxne) == 0u && j != 0u; --j) {
        }
        (void)regs.DATAR;
        (void)regs.STATR;
    }
    return took * 10u / turns;
}

void td_witness() {
    constexpr uint16_t frames_a_run = 32;
    constexpr uint8_t runs = 4;
    const SpiClock rates[6] = {SpiClock::div8,  SpiClock::div16,  SpiClock::div32,
                               SpiClock::div64, SpiClock::div128, SpiClock::div256};
    host_ready();
    print(serial, "  each line ", runs, " runs of ", frames_a_run, " frames, one behind the other, interrupts masked: ",
          runs * (frames_a_run - 2u), " frame ends a line, filed by what the first load to show a flag showed", crlf);
    uint32_t rxne_total = 0;
    uint32_t ends_total = 0;
    uint16_t spread[5] = {};
    for (uint8_t wide = 0; wide < 2u; ++wide) {
        const SpiDataSize bits = wide != 0u ? SpiDataSize::bits16 : SpiDataSize::bits8;
        for (uint8_t m = 0; m < 4u; ++m) {
            for (const SpiClock rate : rates) {
                WitnessRun w{};
                Host::prime(static_cast<SpiMode>(m), rate, bits);
                console_drain();
                for (uint8_t r = 0; r < runs; ++r) {
                    InterruptGuard guard;
                    witness_run(frames_a_run, w);
                }
                print(serial, "  mode ", m, wide != 0u ? " 16-bit /" : "  8-bit /", asked_divisor(rate),
                      ": RXNE alone ", w.rxne_alone, ", both ", w.together, ", TXE alone ", w.txe_alone,
                      "; the other flag at the next load ", w.next_load[0], ", 2nd ", w.next_load[1], ", 3rd ",
                      w.next_load[2], ", 4th ", w.next_load[3], ", later ", w.next_load[4]);
                if (w.lost != 0u || w.stuck != 0u) {
                    print(serial, ", LOST ", w.lost, ", stuck ", w.stuck);
                }
                print(serial, crlf);
                if (w.lost == 0u && w.stuck == 0u) {
                    rxne_total += w.rxne_alone;
                    ends_total += static_cast<uint32_t>(w.rxne_alone) + w.together + w.txe_alone;
                    for (uint8_t k = 0; k < 5u; ++k) {
                        spread[k] = static_cast<uint16_t>(spread[k] + w.next_load[k]);
                    }
                }
            }
        }
    }
    uint32_t load_x10 = 0;
    {
        InterruptGuard guard;
        load_x10 = witness_load_x10();
    }
    Host::prime(SpiMode::mode0, SpiClock::div256, SpiDataSize::bits8);
    uint32_t poll_x10 = 0;
    {
        InterruptGuard guard;
        poll_x10 = witness_poll_x10();
    }
    print(serial, "  over the lines with nothing lost: RXNE with TXE low at ", rxne_total, " of ", ends_total,
          " frame ends; the other flag at the next load ", spread[0], ", 2nd ", spread[1], ", 3rd ", spread[2],
          ", 4th ", spread[3], ", later ", spread[4], crlf);
    print(serial, "  a back-to-back load ", load_x10 / 10u, ".", load_x10 % 10u, " cycles, a turn of the poll ",
          poll_x10 / 10u, ".", poll_x10 % 10u, crlf);
    host_ready();
}

// ===========================================================================
// e - OVR and its clear, on the resource (wireless)
// ===========================================================================
//
// What the resource's two sequences rest on, because 16.2.7 says otherwise:
// "Reading the data register and then reading the status register will
// eliminate this bit". Two frames clocked with nothing read, the first left in
// the buffer, the second lost; then the ways a program goes at the flag, judged
// at /32 (MISO is not read as data). The flag falls within a load of the read
// that clears it, not at it - two compressed loads back to back, 2.2 cycles
// apart, read it still up - so every judged load here comes a microsecond after
// the read before it.

/// Two frames at /32 with nothing read: frame 0 in the buffer, frame 1 lost
/// under OVR. The frames take 512 cycles and the delay 15 us, 720 cycles.
void two_frames_unread() {
    SpiRegs& regs = S::regs();
    regs.DATAR = 0x11u;
    for (uint32_t i = 2000u; (regs.STATR & spi_txe) == 0u && i != 0u; --i) {
    }
    regs.DATAR = 0x22u;
    (void)delay_us(clock, 15);
}

void te_overrun() {
    host_ready();
    Host::prime(SpiMode::mode0, SpiClock::div32, SpiDataSize::bits8);
    SpiRegs& regs = S::regs();

    // Past a DATAR read, the load that judges it.
    auto settled = [&regs] {
        (void)delay_us(clock, 1);
        return static_cast<uint16_t>(regs.STATR);
    };
    // A load that shows it, then a read.
    S::flush_rx();
    two_frames_unread();
    const uint16_t raised = regs.STATR;
    (void)regs.DATAR;
    const uint16_t shown_then_read = settled();
    // 16.2.7's order, and everything else the chapter offers for the flag;
    // then a read, that load having shown it, the buffer empty.
    two_frames_unread();
    (void)regs.DATAR;
    const uint16_t read_first = settled();
    const uint16_t read_first_again = regs.STATR;
    regs.STATR = static_cast<uint16_t>(~spi_ovr);
    const uint16_t wrote_zero = regs.STATR;
    regs.CTLR1 = static_cast<uint16_t>(regs.CTLR1 & ~spi_spe);
    regs.CTLR1 = static_cast<uint16_t>(regs.CTLR1 | spi_spe);
    const uint16_t spe_toggled = regs.STATR;
    (void)regs.DATAR;
    const uint16_t read_empty = settled();
    two_frames_unread();
    S::flush_rx();
    const uint16_t flushed = settled();
    print(serial, "  two frames unread at /32: STATR ", hex(raised), "; a read after that load, STATR ",
          hex(shown_then_read), crlf, "  with no load first, DATAR then STATR ", hex(read_first), " ",
          hex(read_first_again), ", a zero written to OVR ", hex(wrote_zero), ", SPE down and up ", hex(spe_toggled),
          ", a read of the empty buffer after those loads ", hex(read_empty), crlf, "  flush_rx() ", hex(flushed), crlf);
    bench.verdict("OVR RISES on a frame completed with one standing unread, and a read after a load that showed it "
                  "clears it",
                  (raised & (spi_ovr | spi_rxne)) == (spi_ovr | spi_rxne) &&
                      (shown_then_read & (spi_ovr | spi_rxne)) == 0u);
    bench.verdict("16.2.7'S ORDER DOES NOT CLEAR IT ON THIS DIE: DATAR then STATR leaves it standing with RXNE down, and "
                  "so do a zero written to it and SPE down and up; a read after those loads clears it, the buffer empty",
                  (read_first & spi_ovr) != 0u && (read_first & spi_rxne) == 0u && (read_first_again & spi_ovr) != 0u &&
                      (wrote_zero & spi_ovr) != 0u && (spe_toggled & spi_ovr) != 0u && (read_empty & spi_ovr) == 0u);
    bench.verdict("flush_rx() leaves neither flag after a frame unread and a frame lost",
                  (flushed & (spi_ovr | spi_rxne)) == 0u);
    host_ready();
}

// ===========================================================================
// h - a handler longer than a frame beside the polled loop
// ===========================================================================
//
// THE RECEIVE LOOP KEEPS ONE FRAME IN FLIGHT and this host owns the clock, so
// the clock waits for the read: a handler that outlasts a frame is a gap on the
// bus, and no frame can be lost to it. The witness is a second chip: ten windows
// of 128 frames at /64 against the peer (mode 3, the one it answers in), what
// each end read judged byte for byte, under the tick's handler dragged by an
// empty loop past four frames.
//
// THE WINDOW IS LONGER THAN THE TICK: 128 frames of 512 cycles are 65 536 cycles
// of wire and the ticks stand P = 48 000 apart, so every window takes a tick.
//
// THE BOUND on what the handler adds, derived, in cycles. H_min..H_max is the
// dragged handler's hold and H0 the undragged one's, each the gap a main-context
// loop of ruler reads sees, the shortest such gap R being that loop's own turn.
// A landing costs the window at least H - W - C, the handler having used up to
// a frame W = 512 and the loop's turn C the loop would have waited anyway, and
// at most H + C, the loop's way back to its read. Q_min..Q_max is the undragged
// window, which already holds undragged ticks:
//
//   lower = floor(Q_min / P) x (H_min - R - W - C) - ceil(Q_max / P) x H0
//   upper = ceil((Q_max + 2 x H_max) / P) x (H_max + C)
//
// C = 44 is the worst row of letter b's spi.frame table over the wire.

constexpr uint16_t hold_frames = 128;
constexpr uint32_t hold_wire_frame = 512;
constexpr uint32_t hold_turn = 44;
constexpr uint32_t tick_period = 48'000u;
constexpr uint32_t hold_drag = 100;
constexpr uint8_t hold_rounds = 10;

struct HoldGaps {
    uint32_t own_turn = 0xFFFFFFFFu;   ///< R, the loop's own turn
    uint32_t least = 0xFFFFFFFFu;      ///< the shortest gap over `over` cycles
    uint32_t most = 0;                 ///< the longest gap
};

/// The gaps a main-context loop of ruler reads sees over three ticks.
HoldGaps hold_gaps(uint32_t over) {
    HoldGaps g{};
    const uint32_t from = cycles_now();
    uint32_t last = from;
    for (;;) {
        const uint32_t now = cycles_now();
        const uint32_t gap = now - last;
        last = now;
        g.own_turn = gap < g.own_turn ? gap : g.own_turn;
        g.most = gap > g.most ? gap : g.most;
        if (gap > over && gap < g.least) {
            g.least = gap;
        }
        if (now - from > 150'000u) {
            return g;
        }
    }
}

struct HoldTally {
    uint8_t exact = 0;
    uint8_t not_ok = 0;
    uint8_t overrun = 0;
    uint32_t cycles_min = 0xFFFFFFFFu;
    uint32_t cycles_max = 0;
};

HoldTally hold_series(uint32_t drag) {
    HoldTally t{};
    for (uint8_t k = 0; k < hold_rounds; ++k) {
        Exchange e{};
        e.rate = SpiClock::div64;
        e.count = hold_frames;
        e.seed_a = static_cast<uint8_t>(0x21u + 13u * k);
        e.seed_b = static_cast<uint8_t>(e.seed_a ^ 0x6Cu);
        Verify v{};
        spilink::Report r{};
        console_drain();
        tick_drag = drag;
        const bool exact = exchange_exact(e, v, r);
        tick_drag = 0;
        const uint8_t st = Host::status();
        if (exact) {
            ++t.exact;
        } else {
            dump_exchange(e, v, r);
        }
        if (st != spi_ok) {
            ++t.not_ok;
        }
        if (window_ovr) {
            ++t.overrun;
        }
        t.cycles_min = window_cycles < t.cycles_min ? window_cycles : t.cycles_min;
        t.cycles_max = window_cycles > t.cycles_max ? window_cycles : t.cycles_max;
    }
    return t;
}

void th_handler_gap() {
    if (!need_peer()) {
        return;
    }
    console_drain();
    const HoldGaps plain = hold_gaps(0xFFFFFFFFu);
    tick_drag = hold_drag;
    const HoldGaps dragged = hold_gaps(hold_wire_frame);
    tick_drag = 0;
    const HoldTally quiet = hold_series(0);
    const HoldTally held = hold_series(hold_drag);
    const uint32_t cost_min = dragged.least - dragged.own_turn - hold_wire_frame - hold_turn;
    const uint32_t floor_landings = quiet.cycles_min / tick_period;
    const uint32_t ceil_landings = (quiet.cycles_max + tick_period - 1u) / tick_period;
    const uint32_t floor_part = floor_landings * cost_min;
    const uint32_t quiet_part = ceil_landings * plain.most;
    const uint32_t lower = floor_part > quiet_part ? floor_part - quiet_part : 0u;
    const uint32_t upper = (quiet.cycles_max + 2u * dragged.most + tick_period - 1u) / tick_period *
                           (dragged.most + hold_turn);
    const uint32_t longer_min = held.cycles_min > quiet.cycles_max ? held.cycles_min - quiet.cycles_max : 0u;
    const uint32_t longer_max = held.cycles_max > quiet.cycles_min ? held.cycles_max - quiet.cycles_min : 0u;
    print(serial, "  the polled loop at /64, ", hold_frames, " frames; the tick holds main context off ", plain.most,
          " cycles, dragged ", dragged.least, "..", dragged.most, " (a read's turn ", dragged.own_turn, ")", crlf);
    print(serial, "  undragged: ", quiet.exact, " of ", hold_rounds, " exact, a window ", quiet.cycles_min, "..",
          quiet.cycles_max, " cycles, OVR ", quiet.overrun, crlf);
    print(serial, "  dragged: ", held.exact, " of ", hold_rounds, " exact, ", held.not_ok, " not spi_ok, OVR ",
          held.overrun, ", a window ", held.cycles_min, "..", held.cycles_max, " cycles", crlf);
    print(serial, "  longer by ", longer_min, "..", longer_max, " cycles; the arithmetic allows ", lower, "..", upper,
          crlf);
    bench.verdict("the control: ten windows of 128 frames exact both ways under the undragged tick",
                  quiet.exact == hold_rounds && quiet.overrun == 0u);
    bench.verdict("A HANDLER LONGER THAN A FRAME IS A GAP ON THE BUS: ten of ten windows exact both ways, status "
                  "spi_ok, the peer's count right, OVR never standing",
                  dragged.least > 2u * hold_wire_frame && held.exact == hold_rounds && held.not_ok == 0u &&
                      held.overrun == 0u);
    bench.verdict("the window is longer than without the drag by what the arithmetic allows",
                  longer_min >= lower && longer_max <= upper);
    bench.verdict("the command channel is back", command(Op::ping));
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
    uint8_t heard = 0, read_ok = 0;
    for (uint8_t m = 0; m < 4; ++m) {
        for (uint8_t o = 0; o < 2; ++o) {
            for (uint8_t serve = 0; serve < 2; ++serve) {
                Exchange e{};
                e.cfg.mode = m;
                e.cfg.dord = o;
                e.host_lsb = o != 0;
                e.host_mode = static_cast<SpiMode>(m);
                e.spare = serve != 0u ? spilink::spare_polled_pump : 0;
                if (m < 2u) {
                    // CPOL = 0: the CH32V006 client listens alone (it answers
                    // with CPOL = 1 only, its RM 16.2.3, its document).
                    e.spare = static_cast<uint8_t>(e.spare | spilink::spare_receive_only);
                }
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
                if (m >= 2u) {
                    print(serial, ", this host read mism ", v.mism);
                    if (v.mism != 0u) {
                        print(serial, " (idx ", v.idx, " got ", hex(v.got), " exp ", hex(v.exp), ")");
                    }
                } else {
                    print(serial, ", the client listening");
                }
                print(serial, crlf);
                heard = static_cast<uint8_t>(heard + (to_client ? 1u : 0u));
                if (m >= 2u) {
                    read_ok = static_cast<uint8_t>(read_ok + (to_host ? 1u : 0u));
                }
            }
        }
    }
    print(serial, "  -> the client heard 8 frames exact in ", heard, " of 16 runs; in modes 2 and 3 this host read its "
                  "answers exact in ", read_ok, " of 8; in modes 0 and 1 the client listens alone - the CH32V006 "
                  "client answers with CPOL = 1 only (its RM 16.2.3, its document)",
          crlf);
    bench.verdict("THIS HOST'S FRAMES reach a FOREIGN client exact in all four modes, both bit orders, both of its "
                  "serves",
                  heard == 16u);
    bench.verdict("and this host READS the foreign client's answers exact in modes 2 and 3, the modes the CH32V006 "
                  "client answers in",
                  read_ok == 8u);

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

    // WHICH FRAME THE BUFFER KEEPS when one is lost, which 16.2.7 does not
    // say and the host's two-in-flight loop relies on: two of the peer's
    // frames clocked with nothing read, the first read after both are in, the
    // rest of the exchange by hand. Frame 0 in the buffer, frame 1 gone; frame
    // 2 comes as the next frame read. The wait is 120 us against a frame's
    // 43 at /256; the peer's stream differs frame to frame in all eight seeds.
    uint8_t kept_first = 0;
    for (uint8_t k = 0; k < 8u; ++k) {
        spilink::Params lp{};
        lp.cfg = spilink::Cfg{.apply = 1, .mode = 3, .dord = 0, .regime = spilink::regime_buffer_wait};
        lp.count = 8;
        lp.ms = 400;
        lp.seed_a = static_cast<uint8_t>(0x44u + k);
        lp.seed_b = static_cast<uint8_t>(0x9Au + 3u * k);
        lp.pattern = spilink::pattern_prbs;
        if (!peer_act(Op::exchange, lp)) {
            continue;
        }
        settle();
        Host::prime(SpiMode::mode3, link_clock);
        (void)delay_us(clock, 20);
        CsPin::clear();
        link_hold();
        SpiRegs& regs = S::regs();
        S::flush_rx();
        spilink::Stream out(lp.pattern, lp.seed_a);
        regs.DATAR = out.next();
        for (uint32_t i = 200'000u; (regs.STATR & spi_txe) == 0u && i != 0u; --i) {
        }
        regs.DATAR = out.next();
        (void)delay_us(clock, 120);
        const uint16_t st = regs.STATR;
        const uint8_t first = static_cast<uint8_t>(regs.DATAR);
        uint8_t third = 0;
        bool frames = true;
        for (uint8_t f = 2; f < 8u; ++f) {
            uint8_t g = 0;
            frames = frame_by_hand(out.next(), g, false) && frames;
            third = f == 2u ? g : third;
            (void)delay_us(clock, spilink::gap_us);
        }
        frames = wait_not_busy() && frames;
        link_hold();
        CsPin::set();
        if (frames && (st & spi_ovr) != 0u && first == spilink::pattern_value(lp.pattern, lp.seed_b, 0) &&
            third == spilink::pattern_value(lp.pattern, lp.seed_b, 2)) {
            ++kept_first;
        }
        (void)link_command_mode();
    }
    print(serial, "  two frames unread, OVR up: the buffer held the first in ", kept_first, " of 8", crlf);
    bench.verdict("A LOST FRAME IS THE LATER ONE: OVR up, the buffer keeps the frame that stood unread, and the frame "
                  "after the lost one reads exact",
                  kept_first == 8u);
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
            // The engines against the peer's DMA serve are judged as a
            // program gets them: HSRXEN at /2, the driver's default (the
            // polled host's init() leaves it on as well). Without it a
            // client's answer at /2 has no margin (datasheet table 3-19),
            // so the bit's absence is the print below and never a verdict.
            e.high_speed = !serves[s].polled_host && !serves[s].polled_peer && e.rate == SpiClock::div2;
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
    // /2 = 24 MHz on the engines with the high-speed read, the peer's DMA, and
    // the same window without it: printed, no verdict, what the bit buys.
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
                  "code to /2 = 24 MHz (table 3-19's ceiling for both roles), the high-speed read as the driver's "
                  "default has it",
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

/// The arbiter over the engined host: an asynchronous request is DMA.
using SpiArb = SpiBus<DmaHost, P, 4>;

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

DmaHost::Request request(const uint8_t* tx, uint8_t* rx, bool polled) {
    DmaHost::Request r{};
    r.cs = {};   // the letter holds the select across the four
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
    r.cmd_len = 0;
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = 8;
    r.mode = SpiMode::mode3;
    r.clock = link_clock;
    r.engine = polled ? SpiEngine::polled : SpiEngine::dma;
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
    dma_ready();
    owner = Owner::bus_ao;
    settle();
    DmaHost::prime(SpiMode::mode3, link_clock);
    (void)delay_us(clock, 20);
    CsPin::clear();
    link_hold();
    // Request 2 is polled (eight frames: the receive loop); 0 rides the engines; 1
    // is a command of four frames, which runs in the loop inside start(), and a
    // data block of the other four on the engines, whose answers alone it reads;
    // and 3 is a command of eight alone, on the engines as the whole request, whose
    // answers nobody reads.
    for (uint8_t i = 0; i < 4u; ++i) {
        DmaHost::Request r = kl::request(qtx[i], qrx[i], i == 2u);
        if (i == 1u) {
            r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(qtx[1]));
            r.cmd_len = 4;
            r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(qtx[1] + 4));
            r.rx = lend<Lease::reply>(qrx[1] + 4);
            r.len = 4;
        }
        if (i == 3u) {
            r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(qtx[3]));
            r.cmd_len = 8;
            r.len = 0;
        }
        post<kl::SpiArb>(r);
    }
    kl::pump_until(4, 800);
    link_hold();
    CsPin::set();
    DmaHost::release();
    host_ready();
    owner = Owner::host;
    uint16_t mism = 0;
    {
        spilink::Stream want(a.pattern, a.seed_b);
        for (uint8_t i = 0; i < 4u; ++i) {
            for (uint8_t k = 0; k < 8u; ++k) {
                const uint8_t w = want.next();
                if ((i == 1u && k < 4u) || i == 3u) {
                    continue;   // a command's answers, discarded
                }
                if (qrx[i][k] != w) {
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
    bench.verdict("FOUR TRANSACTIONS QUEUED FROM ONE DISPATCH (one polled among three on the engines, one of those a "
                  "command in the loop with its data on them, one a command alone) come back in order, every one spi_ok, and the bytes on "
                  "the wire are what each request lent - read back by A SECOND CHIP, util/spi_bus.hpp and kernel/ "
                  "unchanged",
                  replies_ok && mism == 0u && rep && r.count == total && r.mism == 0u);
    (void)link_command_mode();
    bench.verdict("the command channel is back after the kernel's window", command(Op::ping));
}

// ===========================================================================
// t - the next request after a failure, against the peer
// ===========================================================================
//
// After spi_stalled or spi_dma_fault the next request starts on a clean block,
// whoever posted it and with no recover(). What says so is a second chip: the
// next request's frames, exact both ways. The exchange is commanded FIRST, then
// the host is brought up and never touched again, so that nothing but the host
// itself can have cleaned the block - no init(), no recover(), no prime() - and
// the request frames its own select on the peer's wire, its setup time the
// link's hold. The failed request frames a pad with no wire, so the peer never sees it.
// Mode 3, whose idle SCK the reset moves, and the mode the peer answers in.
//
// The transfer errors are real (fault_request(), past the end of a memory, letter f's stage), so the next
// request meets frames of the faulted block still on the wire: thirty-two frames
// at /256, 2048 cycles each, with the channels stopped after the first frame or
// the eighth, and a frame shifting and perhaps another waiting in the transmit
// buffer when the next start() runs. The verdict that its first frame read is its
// own is the verdict that they did not come back in front of it.

constexpr uint16_t next_frames = 16;
constexpr SpiMode next_mode = SpiMode::mode3;

alignas(4) uint8_t next_tx[next_frames];
alignas(4) uint8_t next_rx[next_frames];

/// The peer commanded to one exchange of `next_frames`, and this end's buffers
/// made ready for it.
bool arm_peer(spilink::Params& a, uint8_t seed) {
    a.cfg = spilink::Cfg{.apply = 1, .mode = 3, .dord = 0, .regime = spilink::regime_buffer_wait};
    a.count = next_frames;
    a.ms = 400;
    a.seed_a = seed;
    a.seed_b = static_cast<uint8_t>(seed + 0x40u);
    a.pattern = spilink::pattern_prbs;
    spilink::Stream out(a.pattern, a.seed_a);
    for (uint16_t i = 0; i < next_frames; ++i) {
        next_tx[i] = out.next();
        next_rx[i] = 0xEE;
    }
    return peer_act(Op::exchange, a);
}

/// What the peer said of the exchange, and what this end read, against the
/// streams. The channel is the plain host's, so the failed host is given up.
bool exchange_judged(const spilink::Params& a, uint16_t& mism, spilink::Report& r) {
    mism = 0;
    spilink::Stream want(a.pattern, a.seed_b);
    for (uint16_t i = 0; i < next_frames; ++i) {
        if (next_rx[i] != want.next()) {
            ++mism;
        }
    }
    const bool rep = peer_report(r);
    return rep && mism == 0u && r.count == next_frames && r.mism == 0u;
}

template <typename H>
typename H::Request next_request(bool polled) {
    typename H::Request r{};
    r.cs = CsPin::ref();
    r.cs_setup_us = 30;   // the link's hold (link_hold())
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(next_tx));
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(next_rx));
    r.len = next_frames;
    r.mode = next_mode;
    r.clock = link_clock;
    r.engine = polled ? SpiEngine::polled : SpiEngine::dma;
    return r;
}

/// One failure and the request after it, directly on the host. Status, select,
/// the next request's way and its frames judged; true when all of it holds. With
/// `framed` the next request is a caller's that frames its own select: prime()
/// first, which pays what is owed, the select by hand around a request naming none.
template <typename H>
bool failure_then_next(Stage s, bool next_polled, uint8_t seed, bool framed = false) {
    spilink::Params a{};
    if (!arm_peer(a, seed)) {
        print(serial, "  the peer refused the exchange", crlf);
        return false;
    }
    dma_ready<H>();
    LoneCs::output(true);
    settle();
    uint8_t st = no_answer;
    bool failed_right = false;
    if (s == Stage::fault_rx || s == Stage::fault_tx) {
        H::prime(next_mode, link_clock);
        const uint32_t faults_before = DmaTx::faults();
        const typename H::Request f = fault_request<H>(s, next_mode);
        host_done = false;
        dma_isr_entries = 0;
        const bool inside = H::start(f);
        const bool answered = await_done();
        st = H::status();
        failed_right = !inside && answered && dma_isr_entries == 1u && st == spi_dma_fault &&
                       DmaTx::faults() == faults_before + 1u;
    } else {
        uint32_t wall = 0;
        H::prime(next_mode, SpiClock::div16);
        st = stall_stage<H>(s, next_mode, wall);
        failed_right = st == spi_stalled;
    }
    const bool selects_high = LoneCs::read_out() && CsPin::read_out();

    // THE NEXT REQUEST, at once: nothing has touched the host since it failed.
    typename H::Request n = next_request<H>(next_polled);
    if (framed) {
        n.cs = {};
        n.cs_setup_us = 0;
        H::prime(next_mode, link_clock);
        (void)delay_us(clock, 20);   // SCK at its idle level before the select
        CsPin::clear();
        link_hold();
    }
    host_done = false;
    dma_isr_entries = 0;
    const bool inside = H::start(n);
    const bool answered = inside || await_done();
    const uint8_t next_st = answered ? H::status() : no_answer;
    const bool way_right = next_polled ? (inside && dma_isr_entries == 0u) : (!inside && dma_isr_entries == 1u);
    (void)delay_us(clock, 30);
    CsPin::set();
    H::release();

    uint16_t mism = 0;
    spilink::Report r{};
    const bool exact = exchange_judged(a, mism, r);
    print(serial, "  ", H::placed_in_ram ? "SRAM: " : "flash: ", stage_name(s), ": status ", st, ", selects ",
          selects_high ? "high" : "LOW", "; next ", next_polled ? "polled" : "asynchronous", " ", next_st,
          ", host mism ", mism, ", the peer read ", r.count, " mism ", r.mism, crlf);
    return failed_right && selects_high && next_st == spi_ok && way_right && exact;
}

namespace kt {

/// One client posts two requests from one dispatch: the first fails, the second
/// is good. Replies in order, the second's frames exact, no recover().
bool queued_behind(Stage s, bool next_polled, uint8_t seed) {
    spilink::Params a{};
    if (!arm_peer(a, seed)) {
        print(serial, "  the peer refused the exchange", crlf);
        return false;
    }
    const bool fault = s == Stage::fault_rx || s == Stage::fault_tx;
    kl::BusKernel::init_all();
    dma_ready<DmaHost>();
    owner = Owner::bus_ao;
    LoneCs::output(true);
    settle();
    DmaHost::Request first = fault_request<DmaHost>(s, next_mode);
    DmaHost::Request second = next_request<DmaHost>(next_polled);
    first.reply = reply_to<kl::Probe, SpiDone>();
    second.reply = reply_to<kl::Probe, SpiDone>();
    if (fault) {
        DmaHost::prime(next_mode, link_clock);
    } else {
        // The block held in reset, a polled read of sixteen at /16.
        first.engine = SpiEngine::polled;
        first.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
        first.rx = lend<Lease::reply>(static_cast<uint8_t*>(rx_buf));
        first.len = 16;
        first.clock = SpiClock::div16;
        DmaHost::prime(next_mode, SpiClock::div16);
        rcc()->APB2PRSTR = rcc()->APB2PRSTR | rcc_pb2_spi1;
    }
    post<kl::SpiArb>(first);
    post<kl::SpiArb>(second);
    kl::pump_until(2, 400);
    const bool selects_high = LoneCs::read_out() && CsPin::read_out();
    (void)delay_us(clock, 30);
    DmaHost::release();
    owner = Owner::host;
    uint16_t mism = 0;
    spilink::Report r{};
    const bool exact = exchange_judged(a, mism, r);
    const uint8_t first_st = kl::Probe::replies[0];
    const uint8_t second_st = kl::Probe::replies[1];
    print(serial, "  queued behind ", stage_name(s), ": ", kl::Probe::n, " replies [", first_st, " ", second_st,
          "], the second ", next_polled ? "polled" : "asynchronous", ", host mism ", mism, ", the peer read ", r.count,
          " mism ", r.mism, crlf);
    return kl::Probe::n == 2u && first_st == (fault ? spi_dma_fault : spi_stalled) && second_st == spi_ok &&
           selects_high && exact;
}

}  // namespace kt

void tt_next_after_failure() {
    if (!need_peer()) {
        return;
    }
    fill_pattern(tx_buf, faulted_frames, 0x5A);
    cmd_buf[0] = 0x2C;

    uint8_t stalls = 0;
    uint8_t seed = 0x11;
    for (const Stage s : {Stage::held_in_reset, Stage::spe_down_write, Stage::spe_down_command}) {
        for (const bool next_polled : {true, false}) {
            if (failure_then_next<FlashDmaHost>(s, next_polled, seed)) {
                ++stalls;
            }
            seed = static_cast<uint8_t>(seed + 5u);
        }
    }
    bench.verdict("A STALL, THEN THE NEXT REQUEST WITH NO recover(): spi_stalled, the select released, then a polled "
                  "and an asynchronous request exact both ways, read back by a second chip; three stall paths, six "
                  "cases",
                  stalls == 6u);

    uint8_t faults = 0;
    for (const Stage s : {Stage::fault_rx, Stage::fault_tx}) {
        for (const bool next_polled : {true, false}) {
            if (failure_then_next<FlashDmaHost>(s, next_polled, seed)) {
                ++faults;
            }
            seed = static_cast<uint8_t>(seed + 5u);
        }
    }
    // A caller framing its own select: prime() has paid the reset, with the select
    // still high, so in mode 3 the SCK pad it moves reaches no selected device.
    for (const bool next_polled : {true, false}) {
        if (failure_then_next<FlashDmaHost>(Stage::fault_rx, next_polled, seed, true)) {
            ++faults;
        }
        seed = static_cast<uint8_t>(seed + 5u);
    }
    bench.verdict("A REAL TRANSFER ERROR IN THE MIDDLE OF A BLOCK, THEN THE NEXT REQUEST WITH NO recover(): dma_isr() "
                  "true once, spi_dma_fault, the select released, the transmit engine's fault counted once, then a "
                  "polled and an asynchronous request exact, no frame of the faulted block in front of them; six cases, "
                  "two of them framing their own select after prime()",
                  faults == 6u);

    uint8_t in_ram = 0;
    for (const Stage s : {Stage::held_in_reset, Stage::fault_rx}) {
        for (const bool next_polled : {true, false}) {
            if (failure_then_next<DmaHost>(s, next_polled, seed)) {
                ++in_ram;
            }
            seed = static_cast<uint8_t>(seed + 5u);
        }
    }
    bench.verdict("the same with the polled loops in SRAM: a stall and a transfer error, four cases", in_ram == 4u);

    uint8_t queued = 0;
    for (const Stage s : {Stage::held_in_reset, Stage::fault_rx, Stage::fault_tx}) {
        for (const bool next_polled : {true, false}) {
            if (kt::queued_behind(s, next_polled, seed)) {
                ++queued;
            }
            seed = static_cast<uint8_t>(seed + 5u);
        }
    }
    bench.verdict("QUEUED BEHIND A FAILURE, THROUGH SpiBus: two requests posted from one dispatch, replies in order, "
                  "the failure's code then spi_ok, the second's frames exact, no recover(); six cases",
                  queued == 6u);
    (void)link_command_mode();
    bench.verdict("the command channel is back after the letter", command(Op::ping));
    LoneCs::release();
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
/// The vectors' client (letter v): what spi1's glue answers from and
/// takes into, and its tally.
struct VecClient {
    uint16_t n;
    uint16_t sent;
    uint16_t got;
    uint32_t entries;
    uint32_t both;
    uint32_t nones;
    uint32_t cycles_max;
};
volatile VecClient vc{};

/// One burst with this chip as the client in `mode` and `lsb`, served by
/// the polled exchange run or, with `vectors`, FROM SPI1'S VECTOR (letter
/// v): RXNEIE and TXEIE up, the glue writing the next answer on TXE and
/// taking the frame on RXNE.
ClientRun client_burst(uint8_t mode, bool lsb, uint8_t seed_a, uint8_t seed_b, uint16_t div, bool vectors = false) {
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
    if (vectors) {
        vc.n = a.count;
        vc.sent = 0;
        vc.got = 0;
        vc.entries = 0;
        vc.both = 0;
        vc.nones = 0;
        vc.cycles_max = 0;
        owner = Owner::client_vectors;
        S::rxne_interrupt(true);
        S::txe_interrupt(true);
        Pfic::enable(S::irq());
        S::enable();   // TXE stands: the vector loads the first answer at once
        const uint32_t t0 = Ticker::millis();
        while (vc.got < a.count && Ticker::millis() - t0 < 300u) {
        }
        S::rxne_interrupt(false);
        S::txe_interrupt(false);
        Pfic::disable(S::irq());
        owner = Owner::none;
        out.got = vc.got;
    } else {
        S::enable();
        const Client::Run run = Client::exchange(std::span<const uint8_t>(ctx, a.count),
                                                 std::span<uint8_t>(crx, a.count), SysClock::hz / 8u);
        out.got = run.count;
    }
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
// v - this chip as the CLIENT served from its VECTOR
// ===========================================================================
//
// Letter r's bursts again, the client's flags delivered by spi1 (RXNEIE and
// TXEIE, the body SpiClient::isr() and the glue in the vector) instead of
// the polled run: every combination SpiClient admits as an answering
// client, the peer's host at 1.5 MHz (a frame 256 cycles of this end's 48
// MHz, against an entry of about a hundred). Judged both ways as r judges;
// the entries per frame and the longest body printed.

void tv_client_vectors() {
    if (!need_peer()) {
        return;
    }
    uint8_t admitted = 0, exact = 0;
    uint32_t entries = 0, frames = 0, nones = 0, longest = 0;
    for (uint8_t m = 0; m < 4u; ++m) {
        for (uint8_t o = 0; o < 2u; ++o) {
            if (!spi_client_answers(static_cast<SpiMode>(m), o != 0u)) {
                continue;
            }
            ++admitted;
            const ClientRun c = client_burst(m, o != 0u, static_cast<uint8_t>(0x3C + m + 8u * o),
                                             static_cast<uint8_t>(0x81 + m + 8u * o), 32, true);
            const bool ok = c.ran && c.got == 12u && c.mism == 0u && c.peer.count == 12u && c.peer.mism == 0u;
            print(serial, "  mode ", m, o != 0u ? " LSb" : " MSb", " from the vector: received ", c.got,
                  "/12 mism ", c.mism, "; the host read our answers ", c.peer.count, "/12 mism ", c.peer.mism,
                  "; entries ", vc.entries, " (", vc.both, " with both flags, ", vc.nones, " with none), the "
                  "longest body ", vc.cycles_max, " cycles",
                  crlf);
            exact = static_cast<uint8_t>(exact + (ok ? 1u : 0u));
            entries += vc.entries;
            frames += c.got;
            nones += vc.nones;
            longest = vc.cycles_max > longest ? vc.cycles_max : longest;
        }
    }
    print(serial, "  -> ", exact, " of ", admitted, " admitted combinations exact both ways; ", entries, " entries for ",
          frames, " frames, the longest body ", longest, " cycles", crlf);
    bench.verdict("THE CLIENT FROM ITS VECTOR receives a foreign host's burst and answers it exact in every "
                  "combination SpiClient admits",
                  admitted != 0u && exact == admitted);
    bench.verdict("every entry found a flag it serves (no entry with none)", nones == 0u);
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

/// The same, one frame written at sample `kick`: the frame's first SCK
/// edge comes some cycles after the write - before the sampler's first
/// look when the write comes first (letter w measured it) - so the line's
/// idle level is read before the frame starts.
[[gnu::noinline]] uint32_t sample_wave_frame(uint16_t n, uint16_t kick, uint16_t frame) {
    const uint32_t t0 = cycles_now();
    for (uint16_t i = 0; i < n; ++i) {
        if (i == kick) {
            S::data(frame);
        }
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
        const uint32_t took = sample_wave_frame(wave_samples, 8, 0x5A);
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
        if (m >= 2u) {   // the modes the CH32V006 client answers in (CPOL = 1)
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
    bench.verdict("and the CH32V006 client's MISO carries its stream in modes 2 and 3, the modes it answers in",
                  client_ok == 2u);
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

BRIO_CH32_VECTOR(systick_handler) {
    const uint32_t t0 = stamp();
    brio::Ticker::tick();
    const uint32_t d = stamp_span(t0, stamp());
    if (d > tick_body_max) {
        tick_body_max = d;
    }
    for (uint32_t k = tick_drag; k != 0u; --k) {
        __asm__ volatile("");
    }
    if (dc_watch) {
        if (DcPin::read_out()) {
            dc_high_ticks = dc_high_ticks + 1u;
        } else {
            dc_low_ticks = dc_low_ticks + 1u;
        }
    }
}

BRIO_CH32_VECTOR(usart2_handler) {
    const uint32_t t0 = stamp();
    (void)Serial::isr();
    const uint32_t d = stamp_span(t0, stamp());
    if (d > console_body_max) {
        console_body_max = d;
    }
}

BRIO_CH32_VECTOR(spi1_handler) {
    const uint32_t t0 = stamp();
    spi_isr_entries = spi_isr_entries + 1u;
    // The hosts arm no interrupt of the block (asynchronous is DMA): the
    // vector is the client's, letter v's.
    switch (owner) {
        case Owner::client_vectors: {
            // THE CLIENT FROM ITS VECTOR (letter v): the next answer on TXE -
            // one ahead, the buffer's frame of window - the frame on RXNE.
            const uint32_t up = Client::isr();
            vc.entries = vc.entries + 1u;
            if ((up & SpiFlag::rxne) != 0u) {
                const uint8_t v = static_cast<uint8_t>(S::regs().DATAR);
                if (vc.got < sizeof crx) {
                    crx[vc.got] = v;
                }
                vc.got = static_cast<uint16_t>(vc.got + 1u);
            }
            if ((up & SpiFlag::txe) != 0u) {
                if (vc.sent < vc.n) {
                    S::regs().DATAR = ctx[vc.sent];
                    vc.sent = static_cast<uint16_t>(vc.sent + 1u);
                } else {
                    S::txe_interrupt(false);
                }
            }
            if ((up & (SpiFlag::rxne | SpiFlag::txe)) == (SpiFlag::rxne | SpiFlag::txe)) {
                vc.both = vc.both + 1u;
            } else if ((up & (SpiFlag::rxne | SpiFlag::txe)) == 0u) {
                vc.nones = vc.nones + 1u;
            }
            const uint32_t d = stamp_span(t0, stamp());
            if (d > vc.cycles_max) {
                vc.cycles_max = d;
            }
            break;
        }
        default:
            S::rxne_interrupt(false);
            S::txe_interrupt(false);
            S::error_interrupt(false);
            break;
    }
}

/// What letter g reads in the vector, right after dma_isr() answered true.
[[gnu::always_inline]] inline void read_the_end() {
    sck_at_end = SckPin::read();
    end_stamp = cycles_now();
}

[[gnu::always_inline]] inline void spi_dma() {
    dma_isr_entries = dma_isr_entries + 1u;
    if (owner == Owner::dma_host && DmaHost::dma_isr()) {
        read_the_end();
        host_done = true;
    } else if (owner == Owner::bus_ao && DmaHost::dma_isr()) {
        brio::post<kl::SpiArb>(brio::TransferDone{DmaHost::status()});
    } else if (owner == Owner::flash_dma_host && FlashDmaHost::dma_isr()) {
        read_the_end();
        dma_done_at = brio::Ticker::cycles();
        host_done = true;
    }
}
BRIO_CH32_VECTOR(dma1_channel2_handler) { spi_dma(); }
BRIO_CH32_VECTOR(dma1_channel3_handler) { spi_dma(); }

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
    bench.letter('i', "THE ENGINE A REQUEST NAMES, wireless: one frame on the engines, the command rule, the refusals",
                 ti_engine_rules);
    bench.letter('g', "WHEN A REQUEST ENDS THE WIRE HAS ENDED, wireless: SCK against its idle level at the flag and "
                      "after the request, both engines, both placements", tg_wire_end);
    bench.letter('s', "the /2 frame and the core's share of the bus: twenty runs polled in SRAM and waited "
                      "off the bus (by name)", ts_bus_share, false);
    bench.letter('d', "THE WITNESS: which of RXNE and TXE comes first, a frame behind the one shifting, a table (by name)",
                 td_witness, false);
    bench.letter('e', "OVR AND ITS CLEAR on the resource: what raises it, which reads clear it, flush_rx()", te_overrun);
    bench.letter('f', "THE FAULT EXITS: every stall path and a real transfer error with no recover() after them, a "
                      "closed gate, the rest of the map, a polled request over a dead channel, release()'s pulse",
                 tf_faults);
    bench.second_letter('n', "THE PEER: the command channel, ident, ten pings", tn_peer_link);
    bench.second_letter('o', "the matrix: four modes, both orders, a mismatch, the receive CRC, the frame kept",
                        to_peer_matrix);
    bench.second_letter('p', "THE LADDER against the peer, both host paths, both serves, /2 with the high-speed read",
                        tp_peer_rates);
    bench.second_letter('k', "THE CLIENT'S RUNS: the peer's answer, exchange and receive, both widths, gapped and "
                             "back to back",
                        tk_peer_runs);
    bench.second_letter('q', "THE KERNEL against the peer: four transactions in one select window", tq_peer_kernel);
    bench.second_letter('t', "THE NEXT REQUEST AFTER A FAILURE against the peer: a stall and a transfer error, direct "
                             "and through SpiBus, exact with no recover()",
                        tt_next_after_failure);
    bench.second_letter('r', "THE ROLES INVERT: this chip as the client in every mode and order", tr_peer_client);
    bench.second_letter('v', "THIS CHIP AS THE CLIENT FROM ITS VECTOR under the peer's host_burst",
                        tv_client_vectors);
    bench.second_letter('h', "A HANDLER BESIDE THE POLLED LOOP IS A GAP ON THE BUS: ten windows exact under a dragged tick",
                        th_handler_gap);
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
