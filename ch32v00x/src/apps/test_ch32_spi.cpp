// test_ch32_spi - the reference bench suite for the CH32V00x's SPI
// chapter: ch32v00x/spi.hpp over RM ch. 16 - the block, the polled path
// at every rate, the DMA engines at any length with their rules (what they
// cannot carry is refused, a command in front of a data phase runs in the
// loop), the hardware CRC, the client on its hardware select, MODF - and
// util/spi_bus.hpp's arbiter with not one line changed, every frame on the
// wire witnessed by A SECOND CHIP.
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
//      arithmetic; THE REFUSAL - a request naming the engines on the host
//      with none answered spi_rejected before anything moves; the polled path, a polled request
//      on the engined host and the engines, each run to completion with
//      nothing driving MISO; no SPI interrupt armed anywhere
//   i  THE ENGINE A REQUEST NAMES, wireless: one frame on the engines at /2, read
//      and written; THE COMMAND RULE, D/C sampled by the tick: a command in the
//      loop then the data on the engines, and a command alone on them; THE
//      REFUSALS, each one the engines cannot carry answered spi_rejected before
//      anything moves, and the next requests; THE ONE BUSY CHECK, a polled
//      request and one naming the engines refused over a transfer in flight
//   l  THE RELEASE CONTRACT ON THE SAME OWNER, wireless: SPI1's receive request
//      left standing across release() (RXDMAEN set, a frame in DATAR, the
//      gate closed), then init() again and a read of eight on the engines -
//      spi_ok, one interrupt, nothing left in the receive buffer; the control,
//      the reset left out, showing the held request as one item
//   j  THE ENGINES OF BYTES, wireless, in an image of its own: 16-bit frames
//      on a host whose engines are of uint8_t refused before anything moves
//   b  THE ENGINES against the peer: the four modes at 8 and 16 bits, sixteen
//      frames each, both directions judged, one DMA interrupt a transaction;
//      a command phase in the loop inside start() then the data block, at two
//      lengths, and a command alone on the engines; a read with no out buffer;
//      one frame read and one written on the engines
//   c  THE POLLED PATH AT EVERY RATE, /2 to /256, 64 frames each with the
//      peer deselected, the burst timed - none faster than its wire - and
//      the receive loop's frame at /64 and /128 within the wire and its turn
//   d  THE DMA ENGINES on channels 2 and 3 against the peer: blocks with
//      and without a command phase, ONE DMA INTERRUPT a transaction,
//      16-bit frames in half-word beats, an odd buffer's 16-bit request
//      refused and the next request exact, a polled request on the engined host
//      running its loop at its own width after an asynchronous one at the
//      other, the 0xFF cell
//   g  THE RATE ON THE WIRE - a frame inside an engined block timed
//      against its SCK periods, the peer deselected - and THE HOST'S
//      COSTS: E, F and the frame through one call, for each placement, and
//      the length from which the engines cost the core less, measured against
//      the arithmetic the page's guidance is
//   e  THE HARDWARE CRC against a foreign CRC unit: 8 and 16 bits, each
//      end's TXCRCR and RXCRCR the bitwise arithmetic, each end checking
//      the other's CRC frame - and a polynomial mismatch caught at both
//   f  THE KERNEL: SpiBus (= BusMaster) over the engined SpiHost, replies
//      in order, the rejection, both votes; over the engineless one, an
//      asynchronous request answered spi_rejected (the peer deselected)
//   h  THE STALLED EXIT, wireless: a block held in reset and a block whose
//      enable was taken down under the host, each stalling a polled request
//      and a polled command phase in front of an asynchronous block; the
//      status, the select released, the block put back by the host alone, the
//      next requests spi_ok with no recover(). A real transfer error, a block
//      running past the end of a memory, the same. A closed clock gate, which a
//      reset does not open, and a block over each hole in the map
//   t  THE NEXT REQUEST AFTER A FAILURE, against the peer, with no recover():
//      a stall and a transfer error, directly and queued through SpiBus, the
//      next request polled and asynchronous exact both ways, in both
//      placements
//   v  OVR AND ITS CLEAR, wireless, measured against 16.2.7 on the resource:
//      what raises the flag, which reads clear it, flush_rx(), the polled
//      write's tail
//   k  A HANDLER BESIDE THE POLLED LOOP IS A GAP ON THE BUS, against the
//      peer: the tick's handler dragged past a frame, ten windows of 128
//      frames at /64 exact both ways, spi_ok, OVR never standing, the
//      window longer by what the arithmetic allows
//   w  WHEN A REQUEST ENDS, THE WIRE HAS ENDED, wireless: the SCK pad against
//      the mode's idle level at the flag the end is read from and right after
//      start() or dma_isr() answered, all four modes, /256 and /64, read and
//      write, both engines, with a select and through a null one
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
//      (PC1) under the peer's host_burst - the answering client in modes
//      2 and 3 judged both ways, MSb and LSb first, a first answer bit of
//      0 and of 1, 16-bit frames; modes 0 and 1 refused by the driver as
//      an answering client (spi_client_may_answer)
//   m  MODF from a second chip: a host whose NSS is a hardware input,
//      demoted four times by the peer's ss_pulse, and the clearing
//      sequence
//   x  (outside z) the slip statistics: each mode ten times
//
// build: boards = v006k8,v003f4
// build: groups = a,b,c,d,g,e,f,h,k,n,o,p,q,r,m,x,s,v,t,w,i,j,l
// build: groups.v006k8 = abcdhi,gefv,knopqrmxs,twl,j
// (on the CH32V006 the suite whole passes the 40 KB of program flash, so it
// builds as five images there: the block, the engines against the peer, the
// polled path and the stalled exit; the rate on the wire with the host's
// costs, the CRC, the kernel and OVR's clear; the command channel, the matrix,
// the ladder, the handler beside the polled loop, the kernel against the
// peer, the client, MODF and the two statistics letters; the next request
// after a failure with the end of the wire and the release contract on the
// same owner; and the engines of bytes, letter
// j, whose binding of channels 2 and 3 is the only one of its image that is
// armed)
// build: monitor_speed = 115200

#include <stdint.h>

#include <type_traits>
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
/// The same engined host with its polled loops in SRAM (util/placement.hpp's
/// InRam): letter g's second placement.
using RamDmaHost = SpiHost<1, spi1_default_pins, DmaTx, DmaRx, InRam>;
static_assert(!Host::has_dma && DmaHost::has_dma, "the suite's two hosts, without and with engines");
using SckPin = Pin<'C', 5>;
using MosiPin = Pin<'C', 6>;
using MisoPin = Pin<'C', 7>;
/// The select wire to the peer (SPI1's NSS pad of column 0).
using CsPin = Pin<'C', 1>;
/// The single-board letters' select: a pad with no wire.
using LoneCs = Pin<'C', 3>;
/// The D/C line of letter i's command rule: a pad with no wire either.
using DcPin = Pin<'C', 4>;

static_assert(spi1_default_pins.nss == Pad{'C', 1} && spi1_default_pins.sck == Pad{'C', 5} &&
                  spi1_default_pins.mosi == Pad{'C', 6} && spi1_default_pins.miso == Pad{'C', 7},
              "SPI1's column 0: NSS PC1, SCK PC5, MOSI PC6, MISO PC7");

TestBench<Serial> bench;

/// What THIS image carries (a group image on the CH32V003 carries a few
/// letters): the vectors and the window serve only what some carried
/// letter starts, so a group image links no host it never runs.
constexpr bool carries(char key) { return selection_carries(test_letters, key); }
/// The engines: a, b, d, g, i, l, p, s, w, the kernel's f and q, and h's and t's next requests.
constexpr bool image_engines = carries('a') || carries('b') || carries('d') || carries('g') || carries('i') ||
                               carries('l') || carries('p') || carries('s') || carries('w') || carries('f') ||
                               carries('q') || carries('h') || carries('t');
/// The arbiter: f, q and t's queued cases, which the 16 KB part's image has no
/// room for and whose board has no peer on its wires for them to run against.
constexpr bool image_queue = carries('t') && device::flash_bytes > 16u * 1024u;
constexpr bool image_kernel = carries('f') || carries('q') || image_queue;
/// The engined host with its polled loops in SRAM, the second placement of
/// letters g, t and w: carried where the image has room - the 16 KB part's would
/// overflow.
constexpr bool image_ram_twin =
    (carries('g') || carries('t') || carries('w')) && device::flash_bytes > 16u * 1024u;

volatile bool host_done = false;
volatile bool dma_host_live = false;
volatile bool ram_host_live = false;
volatile bool bus_ao_live = false;
volatile uint32_t spi_isr_entries = 0;
/// Turns of an empty loop the tick's handler spends after its own work:
/// letter k's handler longer than a frame, zero everywhere else.
volatile uint32_t tick_drag = 0;
volatile uint32_t dma_isr_entries = 0;
/// Letter i's witness of D/C: while `dc_watch` stands the tick's handler counts
/// the level it finds on the pad.
/// The completion's vector takes the watch down with the D/C's rise: no tick
/// pending behind that handler reads it.
volatile bool dc_watch = false;
volatile uint32_t dc_low_ticks = 0;
volatile uint32_t dc_high_ticks = 0;
/// Letter w's: what the vector read right after dma_isr() answered true, the
/// SCK pad and the cycle it read it on.
volatile bool sck_at_end = false;
volatile uint32_t end_stamp = 0;

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
    ram_host_live = false;
    bus_ao_live = false;
    (void)Host::init(clock);
    CsPin::output(true);
    LoneCs::output(true);
}

/// A host with its engines (DmaHost or RamDmaHost), the same two selects
/// high, the DMA vectors routed to it.
template <typename D = DmaHost>
void dma_ready() {
    bus_ao_live = false;
    dma_host_live = std::is_same_v<D, DmaHost>;
    ram_host_live = !std::is_same_v<D, DmaHost>;
    (void)D::init(clock);
    CsPin::output(true);
    LoneCs::output(true);
}

/// The DMA vectors routed nowhere: the engined host given back.
template <typename D = DmaHost>
void dma_done() {
    D::release();
    dma_host_live = false;
    ram_host_live = false;
}

/// The D/C the two helpers' requests carry: none unless a letter names one
/// (letter h, which reads it after every failure).
PinRef xfer_dc{};

/// One transaction through the plain host, waited out. The status, or
/// no_answer.
uint8_t host_xfer(PinRef cs, const uint8_t* cmd, uint8_t cmd_len, const uint8_t* tx, uint8_t* rx, uint16_t len,
                  SpiMode mode, SpiClock rate, SpiDataSize bits, bool polled) {
    Host::Request r{};
    r.cs = cs;
    r.dc = xfer_dc;
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

template <typename D = DmaHost>
uint8_t dma_xfer(PinRef cs, const uint8_t* cmd, uint8_t cmd_len, const uint8_t* tx, uint8_t* rx, uint16_t len,
                 SpiClock rate, SpiDataSize bits, bool polled, SpiMode mode = SpiMode::mode0) {
    typename D::Request r{};
    r.cs = cs;
    r.dc = xfer_dc;
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
    dma_isr_entries = 0;
    if (D::start(r)) {
        return D::status();
    }
    for (uint32_t i = 0; i < 2'000'000UL && !host_done; ++i) {
    }
    if (!host_done) {
        print(serial, "    STALL (dma): STATR=", hex(S::status()), " ch2 flags=",
              hex(DmaChannel<2>::flags()), " ch3 flags=", hex(DmaChannel<3>::flags()), crlf);
        (void)D::recover();
        return no_answer;
    }
    return D::status();
}

/// Did a request of `n` frames naming the engines at `rate`, read or written, go
/// to them (start() false, the vector's completion) or end inside start()? Its
/// status in `status`.
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
    for (uint32_t i = 0; i < 2'000'000UL && !host_done; ++i) {
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

/// THE ONE BUSY CHECK, asked while a transfer of the host's own is on its
/// engines: a request naming `polled` or the engines, its select and D/C on a
/// port of RAM, mode 3 at /2. True when start() answered at once with
/// spi_rejected, neither pad was touched and the control word stands (the
/// transfer in flight keeps its mode and rate). Touches neither host_done nor
/// the interrupt count, which belong to the transfer in flight.
template <typename D>
bool busy_refused(bool polled) {
    const uint16_t word = S::regs().CTLR1;
    GpioRegs port{};
    typename D::Request r{};
    r.cs = PinRef{&port, 1u};
    r.dc = PinRef{&port, 2u};
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
    r.len = 4;
    r.mode = SpiMode::mode3;
    r.clock = SpiClock::div2;
    r.engine = polled ? SpiEngine::polled : SpiEngine::dma;
    const bool done = D::start(r);
    const bool untouched = port.BSHR == 0u && port.BCR == 0u;
    return done && D::status() == spi_rejected && untouched && S::regs().CTLR1 == word;
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
    ram_host_live = false;
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
    engines,         ///< the engined host, naming the engines: their block and their interrupt
    engines_polled,  ///< the engined host, a polled request: the loops inside start(), the
                     ///< engines untouched
};

const char* path_name(Path p) {
    switch (p) {
        case Path::polled: return "polled";
        case Path::engines: return "on the engines";
        default: return "engined host polled";
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

/// Letter k's witnesses of the last polled window: its cycles around the
/// request, and OVR read right after it.
uint32_t window_cycles = 0;
bool window_ovr = false;

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
        const uint32_t t0 = cycles_now();
        st = host_xfer({}, cmd, e.cmd_len, tx, rx, len, e.host_mode, e.rate, bits, true);
        if constexpr (carries('k')) {
            window_cycles = cycles_now() - t0;
            window_ovr = (S::status() & spi_ovr) != 0u;
        } else {
            (void)t0;
        }
    }
    link_hold();
    CsPin::set();
    if constexpr (image_engines) {
        if (engines) {
            dma_done();
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

    // SpiEngine::dma IS DMA: the engineless host refuses a request naming
    // SpiEngine::dma, at once and before it touches anything - asked in mode 3 at
    // /2 after the block was primed in mode 0 at /16, the lone select in
    // the request: start() answers true with spi_rejected, the select stays
    // high, CTLR1 is the word it was (no CPOL flip on SCK: apply() never
    // ran, and every frame comes after it), and SCK reads its mode 0 idle,
    // low. The same request naming SpiEngine::polled then runs on the same host.
    host_ready();
    spi_isr_entries = 0;
    fill_pattern(tx_buf, 64, 0x21);
    Host::prime(SpiMode::mode0, SpiClock::div16);
    const uint16_t c1 = S::regs().CTLR1;
    {
        Host::Request r{};
        r.cs = LoneCs::ref();
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
        r.rx = lend<Lease::reply>(static_cast<uint8_t*>(rx_buf));
        r.len = 16;
        r.clock = SpiClock::div2;
        r.mode = SpiMode::mode3;
        r.engine = SpiEngine::dma;
        const bool done = Host::start(r);
        const uint8_t st = Host::status();
        const bool cs_high = LoneCs::read_out() && LoneCs::read();
        const bool word_kept = S::regs().CTLR1 == c1;
        const bool sck_idle = !SckPin::read();
        print(serial, "  a request naming the engines on the engineless host: start() ", done ? "true" : "FALSE", ", status ",
              st, " (spi_rejected ", spi_rejected, "), the select ", cs_high ? "high" : "LOW", ", CTLR1 ",
              word_kept ? "unchanged" : "REWRITTEN", ", SCK ", sck_idle ? "low" : "HIGH", crlf);
        bench.verdict("A REQUEST NAMING THE ENGINES ON A HOST WITH NONE IS REFUSED: start() answers at once "
                      "spi_rejected, the select never falls, the control word and SCK are untouched",
                      done && st == spi_rejected && cs_high && word_kept && sck_idle);
        r.engine = SpiEngine::polled;
        const bool polled_done = Host::start(r);
        bench.verdict("and the same request marked polled runs on it, spi_ok", polled_done && Host::status() == spi_ok);
    }

    // EVERY WAY COMPLETES WITH NOTHING ON MISO, the peer deselected: the
    // two polled loops; a polled request on the engined host, the loop at
    // any length with no DMA interrupt; the engines at 64 frames. What comes
    // back is not judged here (the peer letters do that), only that every way
    // ENDS and releases the select.
    const uint8_t polled = host_xfer(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode3, SpiClock::div2,
                                     SpiDataSize::bits16, true);
    const uint8_t written = host_xfer(LoneCs::ref(), cmd_buf, 2, tx_buf, nullptr, 16, SpiMode::mode0,
                                      SpiClock::div2, SpiDataSize::bits8, true);
    dma_ready();
    const uint8_t dma = dma_xfer(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 64, SpiClock::div4, SpiDataSize::bits8,
                                 false);
    const uint32_t dma_entries = dma_isr_entries;
    const uint8_t dma_polled = dma_xfer(LoneCs::ref(), cmd_buf, 1, tx_buf, rx_buf, 32, SpiClock::div2,
                                        SpiDataSize::bits8, true);
    const uint32_t dma_polled_entries = dma_isr_entries;
    print(serial, "  wireless completions: polled=", polled, " written=", written, " dma=", dma, " (", dma_entries,
          " DMA interrupt) engined host polled=", dma_polled, " (", dma_polled_entries, ")", crlf);
    bench.verdict("the two polled loops, a polled request on the engined host (no DMA interrupt) and the engines "
                  "(one DMA interrupt) each run a transaction to completion with nothing driving MISO, the selects "
                  "released",
                  polled == spi_ok && written == spi_ok && dma == spi_ok && dma_entries == 1u && dma_polled == spi_ok &&
                      dma_polled_entries == 0u && LoneCs::read_out() && CsPin::read_out());

    dma_done();
    host_ready();
    bench.verdict("NO SPI INTERRUPT: the host arms none of the block's (SPI1's PFIC line disabled, CTLR2's enables "
                  "clear) and SPI1's vector was never entered",
                  !Pfic::enabled(S::irq()) && (S::regs().CTLR2 & (spi_rxneie | spi_txeie | spi_errie)) == 0u &&
                      spi_isr_entries == 0u);
}

// ===========================================================================
// i - the engine a request names, wireless
// ===========================================================================

void ti_engine_rules() {
    dma_ready();
    fill_pattern(tx_buf, 64, 0x21);
    // SpiEngine::dma MEANS DMA AT THE SHORTEST LENGTH: one frame at /2, read and
    // written, where the loop costs the core far less (the guidance, letter g):
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
    for (uint32_t i = 0; i < 2'000'000UL && !host_done; ++i) {
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
    // D/C rises at the command's end and start() returns after the launch, some
    // hundreds of cycles against the tick's 48 000: at most one tick in that
    // span reads it high "in the loop", in about one run in fifty.
    bench.verdict("A COMMAND THEN DATA: the command in the loop inside start(), D/C low there; the data on the engines, "
                  "D/C high there; one interrupt, spi_ok, D/C and the select high at the end",
                  !both_inside && both_done && low_in_loop >= 2u && high_in_loop <= 1u && entries_in_loop == 0u &&
                      low_on_engines == 0u && high_on_engines >= 2u && both_entries == 1u && both_status == spi_ok &&
                      both_up);
    dc_low_ticks = 0;
    dc_high_ticks = 0;
    host_done = false;
    dma_isr_entries = 0;
    dc_watch = true;
    const bool alone_inside = DmaHost::start(command_request(0));
    const uint32_t alone_entries_at_return = dma_isr_entries;
    const bool alone_low_at_return = !DcPin::read_out();
    for (uint32_t i = 0; i < 2'000'000UL && !host_done; ++i) {
    }
    dc_watch = false;
    const bool alone_done = host_done;
    const uint32_t alone_low = dc_low_ticks;
    const uint32_t alone_high = dc_high_ticks;
    const uint32_t alone_entries = dma_isr_entries;
    const uint8_t alone_status = DmaHost::status();
    const bool alone_up = DcPin::read_out() && LoneCs::read_out();
    print(serial, "  a command of 64 alone at /256: start() ", alone_inside ? "TRUE" : "false", ", D/C low on ", alone_low,
          " ticks and high on ", alone_high, ", ", alone_entries, " interrupts, status ", alone_status, crlf);
    bench.verdict("A COMMAND ALONE RIDES THE ENGINES: start() false with D/C low and no interrupt yet, D/C low on every "
                  "tick of the block, one interrupt the end, spi_ok, D/C and the select high",
                  !alone_inside && alone_done && alone_entries_at_return == 0u && alone_low_at_return &&
                      alone_low >= 2u && alone_high == 0u && alone_entries == 1u && alone_status == spi_ok && alone_up);

    // WHAT THE ENGINES CANNOT CARRY IS REFUSED, before the block, the select or the
    // D/C is touched (refused()): 16-bit frames with the out buffer, the in buffer
    // or a command alone off the half-word boundary (8.3.6). A command in front
    // of a data phase runs in the loop, from any address: that one is not refused.
    constexpr SpiDataSize wide = SpiDataSize::bits16;
    const bool odd_out = refused<DmaHost>(nullptr, 0, tx_buf + 1, rx_buf, 8, wide);
    const bool odd_in = refused<DmaHost>(nullptr, 0, tx_buf, rx_buf + 1, 8, wide);
    const bool odd_command = refused<DmaHost>(tx_buf + 1, 4, nullptr, nullptr, 0, wide);

    // THE ONE BUSY CHECK: a write of 64 frames at /256 on the engines (some
    // 131 000 cycles), and while it runs a polled request and one naming the
    // engines, each refused before anything moves; the transfer in flight then
    // ends spi_ok with its one interrupt - the refusals' status did not leak
    // into its own.
    DmaHost::prime(SpiMode::mode0, SpiClock::div256);
    typename DmaHost::Request flight{};
    flight.cs = LoneCs::ref();
    flight.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
    flight.len = 64;
    flight.clock = SpiClock::div256;
    flight.engine = SpiEngine::dma;
    host_done = false;
    dma_isr_entries = 0;
    const bool flight_inside = DmaHost::start(flight);
    const bool busy_polled = busy_refused<DmaHost>(true);
    const bool busy_dma = busy_refused<DmaHost>(false);
    for (uint32_t i = 0; i < 2'000'000UL && !host_done; ++i) {
    }
    const bool flight_done = host_done;
    const uint8_t flight_status = DmaHost::status();
    const uint32_t flight_entries = dma_isr_entries;
    const uint8_t odd_command_with_data = dma_xfer(LoneCs::ref(), tx_buf + 1, 2, tx_buf, rx_buf, 8, SpiClock::div16, wide,
                                                   false);
    const uint32_t odd_command_entries = dma_isr_entries;
    const uint8_t next_polled = dma_xfer(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 8, SpiClock::div16, wide, true);
    const uint8_t next_engines = dma_xfer(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 8, SpiClock::div16, wide, false);
    dma_done();
    host_ready();
    print(serial, "  refused: odd out ", odd_out ? "yes" : "NO", ", odd in ", odd_in ? "yes" : "NO", ", odd command alone ",
          odd_command ? "yes" : "NO", "; over a transfer in flight: polled ", busy_polled ? "refused" : "NOT REFUSED",
          ", on the engines ", busy_dma ? "refused" : "NOT REFUSED", ", the transfer then ", flight_status, " with ",
          flight_entries, " interrupt(s); an odd command before aligned data ", odd_command_with_data, " (",
          odd_command_entries, " interrupt); then polled ", next_polled, ", on the engines ", next_engines, crlf);
    bench.verdict("WHAT THE ENGINES CANNOT CARRY IS REFUSED: 16-bit frames off the half-word boundary, out, in or a "
                  "command alone: spi_rejected at once, select and D/C never touched, the control word, SCK and the "
                  "block as they were",
                  odd_out && odd_in && odd_command);
    bench.verdict("THE ONE BUSY CHECK: over a transfer of the host's own in flight a polled request and one naming "
                  "the engines are refused, spi_rejected, select, D/C and the control word untouched; the transfer "
                  "ends spi_ok with its one interrupt",
                  !flight_inside && busy_polled && busy_dma && flight_done && flight_status == spi_ok &&
                      flight_entries == 1u);
    bench.verdict("AND THE NEXT REQUESTS RUN: an odd command before aligned data in the loop (one interrupt), then "
                  "polled and on the engines, each spi_ok",
                  odd_command_with_data == spi_ok && odd_command_entries == 1u && next_polled == spi_ok &&
                      next_engines == spi_ok);
}

// ===========================================================================
// l - the release contract on the same owner: released, then initialized
// ===========================================================================
//
// docs/design/dma.md's release contract: release() gates the clock and pulses
// no reset; init() starts from the block's reset line, the gate just opened
// and no channel armed. Staged on SPI1's receive request, wireless (the lone
// select, the peer deselected): the engined host up, RXDMAEN raised by hand
// with the receive channel bound and not enabled, one frame written by hand,
// so RXNE stands under RXDMAEN with no channel to take it; then
// DmaHost::release(), which drains, takes SPE down and closes the gate with
// RXDMAEN and RXNE left as they were. Then the SAME owner initialized again
// and a read of eight frames on the engines: spi_ok with its one interrupt,
// and nothing left in the receive buffer after it - a stray first item would
// have completed the receive block one frame early and left the last frame
// standing in DATAR (RXNE), whatever MISO carried. THE CONTROL: the same
// staging, the gate opened with no reset and the receive channel armed by hand
// for SPI1's receive, its count read with nothing clocked: the one item it moves
// is the request the reset withdraws (RXNE and RXDMAEN kept under the gate). A
// held request is taken within a few bus cycles of the enable, so 3 ms of
// settling leave nothing in flight, and the count is exact.

using SpiRxOwner = DmaOwner<DmaPeriph::spi, 1, DmaSignal::rx>;

/// The staging: RXNE standing under RXDMAEN with no channel enabled, then
/// release().
void raise_and_release_spi() {
    dma_ready();
    DmaHost::prime(SpiMode::mode0, SpiClock::div16);
    S::dma_requests(false, true);
    S::data(SpiDataSize::bits8, 0x5A);
    (void)delay_us(clock, 50);   // a frame at /16: 128 cycles
    dma_done();
}

void tl_release_reinit() {
    host_ready();
    LoneCs::output(true);
    constexpr uint8_t runs = 3;
    uint8_t good = 0;
    for (uint8_t run = 0; run < runs; ++run) {
        raise_and_release_spi();
        dma_ready();
        const uint8_t st = dma_xfer(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 8, SpiClock::div16,
                                    SpiDataSize::bits8, false);
        const uint32_t entries = dma_isr_entries;
        (void)delay_us(clock, 50);
        const uint16_t statr = S::status();
        dma_done();
        print(serial, "  run ", run, ": init() again, a read of 8 on the engines: status ", st, ", ", entries,
              " interrupt(s), STATR after ", hex(statr), crlf);
        if (st == spi_ok && entries == 1u && (statr & (spi_rxne | spi_ovr)) == 0u) {
            ++good;
        }
    }

    // THE CONTROL: the same staging, the gate opened with no reset, the
    // receive channel armed by hand for SPI1's receive, nothing clocked.
    uint8_t held = 0;
    uint16_t moved_last = 0;
    for (uint8_t run = 0; run < runs; ++run) {
        raise_and_release_spi();
        S::bus_clock(true);
        using Rx2 = DmaRxEngine<2, uint16_t>;
        Rx2::arm<SpiRxOwner>(S::data_address(), 0u, DmaPriority::very_high);
        (void)Rx2::start(std::span<uint8_t>(rx_buf, 4));
        settle_ms(3);
        moved_last = Rx2::take();
        Rx2::stop();
        S::reset();
        S::bus_clock(false);
        print(serial, "  control run ", run, ": the gate opened with no reset, the channel armed: ", moved_last,
              " item(s) moved with nothing clocked", crlf);
        if (moved_last == 1u) {
            ++held;
        }
    }
    host_ready();
    bench.verdict("RELEASED WITH RXDMAEN SET AND A FRAME STANDING, THEN INITIALIZED AGAIN: a read of eight on the "
                  "engines ends spi_ok with its one interrupt and nothing left in the receive buffer",
                  good == runs);
    bench.verdict("THE CONTROL: the same staging with the reset left out moves one item, the held request",
                  held == runs);
}

// ===========================================================================
// j - the engines of bytes
// ===========================================================================
//
// THE BYTE-BOUND BINDING: a host whose engines are of uint8_t carries 8-bit
// frames alone, and a request in 16-bit frames naming them is refused before
// anything moves. Its own image: the suite's host takes engines of uint16_t,
// which carry both widths, and a channel is bound by one engine type an
// image (one owner a channel, util/dma_owner.hpp; the engine's bookkeeping is
// its type's).

void tj_byte_engines() {
    using ByteHost = SpiHost<1, spi1_default_pins, DmaTxEngine<3, uint8_t>, DmaRxEngine<2, uint8_t>>;
    (void)ByteHost::init(clock);
    const bool on_bytes = refused<ByteHost>(nullptr, 0, tx_buf, rx_buf, 8, SpiDataSize::bits16);
    ByteHost::release();
    host_ready();
    print(serial, "  16-bit frames on engines of bytes: refused ", on_bytes ? "yes" : "NO", crlf);
    bench.verdict("16-BIT FRAMES ON ENGINES OF BYTES ARE REFUSED: spi_rejected at once, select and D/C never "
                  "touched, the control word, SCK and the block as they were",
                  on_bytes);
}

// ===========================================================================
// b - the engines against the peer
// ===========================================================================

void tb_async() {
    if (!need_peer()) {
        return;
    }
    // THE FOUR MODES AT 8 AND 16 BITS on the engines, sixteen frames each
    // at /16: what this host sends judged by the foreign client's report, what it
    // reads judged here against the client's stream. The peer's client
    // answers in every mode MSb first (its header).
    uint8_t exact = 0;
    uint8_t one_a_block = 0;
    for (uint8_t m = 0; m < 4u; ++m) {
        for (uint8_t w = 0; w < 2u; ++w) {
            Exchange e{};
            e.path = Path::engines;
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
            if (e.dma_entries == 1u && e.spi_entries == 0u) {
                ++one_a_block;
            }
        }
    }
    bench.verdict("the four modes at 8 and 16 bits, sixteen frames each on the engines, exact BOTH ways against a "
                  "foreign client",
                  exact == 8u);
    bench.verdict("one DMA interrupt a transaction and no SPI one, every run", one_a_block == 8u);

    // A command phase then a data phase: the command in the loop inside start(),
    // the data on the engines, one interrupt, whatever the command's length. A
    // command alone rides the engines. The client hears every frame, this end
    // keeps the data phase's answers and discards the command's echo.
    Exchange c{};
    c.path = Path::engines;
    c.rate = SpiClock::div8;
    c.count = 10;
    c.cmd_len = 2;
    c.seed_a = 0x77;
    c.seed_b = 0x3E;
    const bool short_cmd = exchange_line(c, "2 command + 8 data");
    bench.verdict("two command frames in the loop inside start(), eight data frames on the engines (one DMA "
                  "interrupt): the client heard ten, this end read the data, the echo discarded",
                  short_cmd && c.dma_entries == 1u);
    Exchange k{};
    k.path = Path::engines;
    k.rate = SpiClock::div8;
    k.count = 32;
    k.cmd_len = 8;
    k.seed_a = 0x2B;
    k.seed_b = 0x4D;
    const bool block_cmd = exchange_line(k, "8 command + 24 data");
    bench.verdict("eight command frames in the loop too, then 24 data frames on the engines (one DMA interrupt): the "
                  "client heard 32, this end read the data",
                  block_cmd && k.dma_entries == 1u);
    Exchange ca{};
    ca.path = Path::engines;
    ca.rate = SpiClock::div8;
    ca.count = 8;
    ca.cmd_len = 8;
    ca.seed_a = 0x35;
    ca.seed_b = 0x6C;
    const bool cmd_alone = exchange_line(ca, "8 command alone");
    bench.verdict("a command with no data phase rides the engines as the whole request (one DMA interrupt): the client "
                  "heard eight",
                  cmd_alone && ca.dma_entries == 1u);
    // Read with no out buffer: the 0xFF cell goes out, the client hears it.
    Exchange d{};
    d.path = Path::engines;
    d.rate = SpiClock::div8;
    d.count = 8;
    d.no_out = true;
    d.pattern = spilink::pattern_fixed;
    d.seed_a = 0xFF;
    d.seed_b = 0x5A;
    bench.verdict("a read with no out buffer clocks the 0xFF cell - the client heard eight 0xFF - and reads the "
                  "client's answers",
                  exchange_line(d, "no out buffer"));
    // ONE FRAME against the peer at /8, where the loop costs the core less than
    // the engines (the guidance, letter g): on the engines all the same, one
    // interrupt, exact both ways, and the written frame heard.
    Exchange q{};
    q.path = Path::engines;
    q.rate = SpiClock::div8;
    q.count = 1;
    q.seed_a = 0x19;
    q.seed_b = 0xC4;
    const bool one_read = exchange_line(q, "one frame read");
    bench.verdict("ONE FRAME READ ON THE ENGINES: one DMA interrupt, no SPI one, exact both ways",
                  one_read && q.dma_entries == 1u && q.spi_entries == 0u);
    Exchange qw{};
    qw.path = Path::engines;
    qw.rate = SpiClock::div8;
    qw.count = 1;
    qw.no_in = true;
    qw.seed_a = 0x2D;
    qw.seed_b = 0x92;
    const bool one_written = exchange_line(qw, "one frame written");
    bench.verdict("ONE FRAME WRITTEN ON THE ENGINES: one DMA interrupt, the client heard it", one_written &&
                  qw.dma_entries == 1u && qw.spi_entries == 0u);
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

    // THE RECEIVE LOOP'S FRAME at /64 and /128, one frame in flight: the
    // difference of a 256- and a 32-frame request over 224 frames, the best
    // of three each, against the wire's 512 and 1024 cycles. The bound is
    // derived, in tenths of a cycle a frame. Below: the wire, less 1 for the
    // tenth's rounding. Above: the wire, the loop's dead bus (bench_ch32's
    // spi.frame table: its worst row, 38 cycles, rounded up to 40), and the
    // tick's share - 224 frames at /128 are 234 000 cycles at most, 5 ticks
    // 48 000 apart, one handler with its prologue about 200 cycles, 1000
    // cycles over 224 frames, 5 cycles a frame; 3 at /64, which spans half
    // as many.
    auto frame_x10 = [](SpiClock rate) {
        auto best = [rate](uint16_t n) {
            uint32_t least = 0xFFFFFFFFu;
            for (uint8_t k = 0; k < 3u; ++k) {
                const uint32_t t0 = cycles_now();
                (void)host_xfer(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, n, SpiMode::mode0, rate,
                                SpiDataSize::bits8, true);
                const uint32_t took = cycles_now() - t0;
                least = took < least ? took : least;
            }
            return least;
        };
        const uint32_t small = best(32);
        const uint32_t big = best(256);
        return big > small ? (big - small) * 10u / 224u : 0u;
    };
    console_drain();
    const uint32_t f64 = frame_x10(SpiClock::div64);
    const uint32_t f128 = frame_x10(SpiClock::div128);
    print(serial, "  the receive loop: /64 ", f64 / 10u, ".", f64 % 10u, " cycles a frame (the wire 512), /128 ",
          f128 / 10u, ".", f128 % 10u, " (1024)", crlf);
    bench.verdict("THE RECEIVE LOOP'S FRAME IS THE WIRE AND ITS TURN AT MOST, one frame in flight: /64 and /128",
                  f64 >= 5119u && f64 <= 5120u + (40u + 3u) * 10u && f128 >= 10239u &&
                      f128 <= 10240u + (40u + 5u) * 10u);

    host_ready();
    bench.verdict("the select is released after the polled transactions, the peer's never taken",
                  LoneCs::read_out() && CsPin::read_out());
}

// ===========================================================================
// k - a handler longer than a frame beside the polled loop
// ===========================================================================
//
// THE RECEIVE LOOP KEEPS ONE FRAME IN FLIGHT and this host owns the clock, so
// the clock waits for the read: a handler that outlasts a frame is a gap on the
// bus, and no frame can be lost to it. The witness is a second chip: ten windows
// of 128 frames at /64 against the peer, what each end read judged byte for
// byte, under the tick's handler dragged by an empty loop past four frames.
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
// C = 40 is the worst row of bench_ch32's spi.frame table over the wire, 38, rounded up.

constexpr uint16_t hold_frames = 128;
constexpr uint32_t hold_wire_frame = 512;
constexpr uint32_t hold_turn = 40;
constexpr uint32_t tick_period = SysClock::hz / 1000u;
constexpr uint32_t hold_drag = 150;

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

constexpr uint8_t hold_rounds = 10;

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
        if (exact) {
            ++t.exact;
        } else {
            dump_exchange(e, v, r);
        }
        if (e.status != spi_ok) {
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

void tk_handler_gap() {
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
    // run in order: the polled 16-bit request follows an asynchronous 8-bit
    // one and the polled 8-bit one an asynchronous 16-bit one, which is
    // what judges the width a polled request's loop takes on the engined
    // host. Every row naming the engines rides them, the polled rows run the
    // loop at any length. In front of the fifth row a 16-bit request off the
    // half-word boundary is refused, on the lone select, and the row is the next
    // request.
    const DmaRow rows[] = {
        {"128 frames", Path::engines, 128, 0, false, false, 0, 0x3C, 0xC3},
        {"command + 32", Path::engines, 33, 1, false, false, 0, 0x0B, 0x5A},
        {"64 polled", Path::engines_polled, 64, 0, false, false, 0, 0x99, 0x17},
        {"32 of 16 bits", Path::engines, 32, 0, true, false, 0, 0x42, 0x24},
        {"after a refusal", Path::engines, 16, 0, true, false, 0, 0x24, 0x81},
        {"asynchronous 8-bit", Path::engines, 32, 0, false, false, 0, 0x61, 0x16},
        {"then POLLED 16-bit", Path::engines_polled, 32, 0, true, false, 0, 0x62, 0x26},
        {"asynchronous 16-bit", Path::engines, 24, 0, true, false, 0, 0x63, 0x36},
        {"then POLLED 8-bit", Path::engines_polled, 32, 0, false, false, 0, 0x64, 0x46},
        {"no out, 8-bit", Path::engines, 16, 0, false, true, 0, 0xFF, 0xA5},
        {"no out, 16-bit", Path::engines, 8, 0, true, true, 0, 0xFF, 0x3C},
    };
    constexpr uint8_t n = sizeof rows / sizeof rows[0];
    bool ok[n];
    uint32_t spi_irqs[n];
    uint32_t dma_irqs[n];
    bool odd_refused = false;
    for (uint8_t i = 0; i < n; ++i) {
        const DmaRow& w = rows[i];
        if (i == 4u) {
            dma_ready();
            odd_refused = refused<DmaHost>(nullptr, 0, tx_buf + 1, rx_buf + 1, 16, SpiDataSize::bits16);
            dma_done();
        }
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
    bench.verdict("a command frame in the loop inside start(), 32 data frames on the engines (one DMA interrupt): all 33 "
                  "heard, the data read exact",
                  ok[1] && dma_irqs[1] == 1u);
    bench.verdict("a POLLED request on the engined host runs its loop inside start(), exact, no interrupt at all",
                  ok[2] && spi_irqs[2] == 0u && dma_irqs[2] == 0u);
    bench.verdict("16-bit frames ride the engines in half-word beats (one DMA interrupt), exact against a foreign "
                  "client",
                  ok[3] && spi_irqs[3] == 0u && dma_irqs[3] == 1u);
    bench.verdict("16-bit frames from an ODD buffer, which the engines cannot carry, are refused with nothing on the "
                  "wire, and the next request is exact against the foreign client",
                  odd_refused && ok[4] && spi_irqs[4] == 0u && dma_irqs[4] == 1u);
    bench.verdict("a POLLED request on the engined host moves its OWN width after an asynchronous one at the other, "
                  "both orders",
                  ok[5] && ok[6] && ok[7] && ok[8]);
    bench.verdict("a read with no out buffer clocks the 0xFF cell (heard so by the client) at 8 and 16 bits",
                  ok[9] && ok[10]);
    host_ready();
}


/// THE HOST'S COSTS for one placement, on its engined host and through ONE
/// call (dma_xfer<D>), so that the call's own cycles stand on both sides of
/// every difference and cancel: E, the engines' fixed cost at /4 (a
/// 32-frame read block less its frames, the frame from a 256-frame one, the
/// best of five each);
/// the polled lines, the loops: 14 frames and
/// 6, the frame the difference over 8, F the intercept, the 6-frame line less
/// its frames, the best of five each - at three shapes (cost_shapes); E - F
/// for a read and a write; and the length from which the engines cost the core
/// less, measured and by the arithmetic.
struct PathCosts {
    uint32_t engine_fixed;
    uint32_t line_frame[3], line_fixed[3];   ///< a frame x10, F
    uint32_t over_read, over_written;
    uint16_t edge_meas[3], edge_calc[3];
};

/// The shapes: a read at /4 (the receive loop's dead bus) and at /2 (its
/// turn), a write at /2 (the transmit-only loop's turn).
struct CostShape {
    SpiClock rate;
    bool read;
};
constexpr CostShape cost_shapes[3] = {{SpiClock::div4, true}, {SpiClock::div2, true}, {SpiClock::div2, false}};

template <typename D>
PathCosts measure_costs() {
    PathCosts c{};
    dma_ready<D>();
    auto block = [](uint16_t n) {
        uint32_t least = 0xFFFFFFFFu;
        for (uint8_t k = 0; k < 5u; ++k) {
            const uint32_t t0 = cycles_now();
            (void)dma_xfer<D>({}, nullptr, 0, tx_buf, rx_buf, n, SpiClock::div4, SpiDataSize::bits8, false);
            const uint32_t took = cycles_now() - t0;
            least = took < least ? took : least;
        }
        return least;
    };
    const uint32_t b32 = block(32);
    const uint32_t b256 = block(256);
    const uint32_t f1000 = b256 > b32 ? (b256 - b32) * 1000u / 224u : 0u;
    const uint32_t wire32 = f1000 * 32u / 1000u;
    c.engine_fixed = b32 > wire32 ? b32 - wire32 : 0u;
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
        const CostShape sh = cost_shapes[k];
        const uint32_t p6 = polled_same(6, sh.rate, sh.read);
        const uint32_t p14 = polled_same(14, sh.rate, sh.read);
        c.line_frame[k] = p14 > p6 ? (p14 - p6) * 10u / 8u : 0u;
        c.line_fixed[k] = p6 > 6u * c.line_frame[k] / 10u ? p6 - 6u * c.line_frame[k] / 10u : 0u;
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
    dma_done<D>();
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

void print_costs(const char* label, const PathCosts& c) {
    print(serial, "  ", label, ": E ", c.engine_fixed, "; E - F read ", c.over_read, ", written ", c.over_written, crlf);
    console_drain();
    auto line = [&c](uint8_t k) {
        print(serial, c.line_frame[k] / 10u, ".", c.line_frame[k] % 10u, "/", c.line_fixed[k]);
    };
    print(serial, "    a polled frame / F: read /4 ");
    line(0);
    print(serial, ", read /2 ");
    line(1);
    print(serial, ", write /2 ");
    line(2);
    print(serial, crlf);
    console_drain();
    print(serial, "    THE LENGTH FROM WHICH THE ENGINES COST LESS, frames - measured/arithmetic: ");
    for (uint8_t k = 0; k < 3u; ++k) {
        print(serial, c.edge_meas[k], "/", c.edge_calc[k], " ");
    }
    print(serial, "(read /4, read /2, write /2)", crlf);
    console_drain();
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
    dma_done();

    // THE HOST'S COSTS: the flash placement (the default), and on the CH32V006
    // the SRAM one. The engine a request runs on is the program's to name; these
    // figures are what the page's guidance is drawn from.
    fill_pattern(tx_buf, 256, 0x3C);
    const PathCosts flash = measure_costs<DmaHost>();
    print_costs("THE COSTS, loops in the flash", flash);
    bool ram_edges = true;
    if constexpr (image_ram_twin) {
        const PathCosts ram = measure_costs<RamDmaHost>();
        print_costs("THE COSTS, loops in SRAM", ram);
        ram_edges = edges_match(ram);
    }
    bench.verdict("THE GUIDANCE IS THE ARITHMETIC'S: the fewest frames a polled transaction spends the engines' measured "
                  "fixed cost on is within one frame of ceil((E - F) / frame) on the same figures, read at /4 and /2 and "
                  "written at /2",
                  edges_match(flash) && ram_edges);
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

/// The arbiter over the engined host, and one over the host with none.
using SpiArb = SpiBus<DmaHost, P, 4>;
using PlainArb = SpiBus<Host, P, 2>;

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

using BusKernel = Tenuto<P, Probe, SpiArb, PlainArb>;

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

template <typename H = DmaHost>
typename H::Request request(const uint8_t* tx, uint8_t* rx, bool polled, uint16_t len = 8) {
    typename H::Request r{};
    r.cs = hand_cs ? PinRef{} : LoneCs::ref();
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
    r.cmd_len = 0;
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = len;
    r.mode = SpiMode::mode0;
    r.clock = rate;
    r.engine = polled ? SpiEngine::polled : SpiEngine::dma;
    r.reply = reply_to<Probe, SpiDone>();
    return r;
}

}  // namespace kl

void tf_kernel() {
    // The three requests below name the engines, so they are in flight on them.
    dma_ready();
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
    bench.verdict("... every one spi_ok, the engines and the polled loop interleaved on one bus (what came back is "
                  "the peer letter q's to judge)",
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
    // The busy bus: a 64-frame write on the engines, 8192 cycles of wire at
    // /16, still in flight when the vote is asked right behind it.
    kl::Probe::clear_tally();
    post<kl::SpiArb>(kl::request(tx_buf, nullptr, false, 64));
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
    dma_done();

    // THE REFUSAL THROUGH THE ARBITER: over the host with no engines, an
    // asynchronous request is answered spi_rejected in the dispatch that
    // receives it and the bus is left free - a polled one queued behind it
    // runs, spi_ok.
    host_ready();
    kl::Probe::clear_tally();
    post<kl::PlainArb>(kl::request<Host>(kl::out_a, nullptr, false));
    post<kl::PlainArb>(kl::request<Host>(kl::out_a, kl::in_a, true));
    kl::pump_until(2, 100);
    print(serial, "  over the engineless host: an asynchronous request answered ", kl::Probe::replies[0],
          " (spi_rejected ", spi_rejected, "), the polled one behind it ", kl::Probe::replies[1], crlf);
    bench.verdict("THE ARBITER OVER A HOST WITH NO ENGINES answers an asynchronous request spi_rejected, and the "
                  "polled request behind it runs spi_ok",
                  kl::Probe::n == 2u && kl::Probe::replies[0] == spi_rejected && kl::Probe::replies[1] == spi_ok);
    bench.verdict("the peer's select stayed high through the letter", CsPin::read_out());
}

// ===========================================================================
// h - the stalled exit, wireless
// ===========================================================================
//
// spi_stalled is the polled path's answer to a flag that never comes - a
// clock that stopped, a block that is dead - spent from ONE count of polls for
// the whole transaction (take_budget(): twice the wire's time for the frames,
// in polls of at least four cycles, and 256 polls). The host releases the
// D/C and the select and puts the block back itself, so the next request needs
// no recover(); every request here carries the D/C (PC4), read after each
// failure. Staged two ways a reset
// cures: SPI1 HELD IN RESET, its registers ignoring every write and RXNE never
// rising, the host's own pulse being what lets the line go; and SPE TAKEN DOWN
// under the host, a frame left standing in the transmit buffer and TXE never
// rising again, the host's compare seeing no change because the request's
// clock, mode and width are primed first. Then the one failure a reset does
// not cure, the gate closed, a transfer error, and a block pointed at each hole
// in the map. The lone select (PC3) frames every request, the peer's deselected.
// What came back is letter t's to judge, against the peer.
//
// THE WALL'S BOUNDS, the host's budget (take_budget()): at /16 and 8 bits a frame
// is 128 cycles on the wire and the shift 6, so sixteen frames are 16 x 64 + 256 =
// 1 280 polls, and a command phase of one frame 64 + 256 = 320. A poll is at least
// four cycles, so the wall from the call to its return is at least 5 120 and 1 280
// cycles (twice the wire's 2 048 and 128, and the 256 polls), a handler only
// lengthening it. It is at most those polls at the dearest turn measured on the
// loops, 31 cycles (docs/ch32v00x/spi.md, "The loops are aligned to the fetch's word"),
// 39680 and 9920, and 3 000 cycles more for two ticks' handlers in the
// span, the host's way in and the reset. So 5 120 to 42680, and 1 280 to 12920.
constexpr uint32_t polls_of_sixteen = 16UL * 64UL + 256UL;
constexpr uint32_t polls_of_command = 64UL + 256UL;
constexpr uint32_t bound_of_sixteen = polls_of_sixteen * 4UL;
constexpr uint32_t bound_of_command = polls_of_command * 4UL;
constexpr uint32_t wall_of_sixteen = polls_of_sixteen * 31UL + 3000UL;
constexpr uint32_t wall_of_command = polls_of_command * 31UL + 3000UL;

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
            rcc()->PB2PRSTR |= rcc_pb2_spi1;
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

/// The ends of the memories: RM 8.2.1 promises a transfer error for a reserved
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
    r.dc = xfer_dc;
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
    for (uint32_t i = 0; i < 2'000'000UL && !host_done; ++i) {
    }
    return host_done;
}

/// A refusal with no prime of its own: 16-bit frames off the half-word boundary.
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

/// The transfer errors, each followed at once by a polled and an asynchronous
/// request; what came back is letter t's to judge. Then a refused request behind
/// a third fault: `good_next` counts it, a third next.
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
        const bool lone_high = LoneCs::read_out() && DcPin::read_out();
        const uint8_t next_polled = dma_xfer<H>(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 16, link_clock,
                                                SpiDataSize::bits8, true, mode);
        const uint8_t next_async = dma_xfer<H>(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 16, link_clock,
                                               SpiDataSize::bits8, false, mode);
        H::release();
        print(serial, "  ", H::placed_in_ram ? "SRAM: " : "flash: ", stage_name(s), ": dma_isr() ",
              answered ? "true" : "never", ", status ", st, " (spi_dma_fault ", spi_dma_fault, "), ", entries,
              " interrupts, D/C and select ", lone_high ? "high" : "LOW", "; then polled ", next_polled, ", asynchronous ",
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

void th_stalled() {
    host_ready();
    fill_pattern(tx_buf, 16, 0x5A);
    DcPin::output(true);
    xfer_dc = DcPin::ref();

    // 1. The host with no engines, a polled read of sixteen at /16.
    rcc()->PB2PRSTR |= rcc_pb2_spi1;
    const uint32_t t0 = cycles_now();
    const uint8_t st = host_xfer(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode0,
                                 SpiClock::div16, SpiDataSize::bits8, true);
    const uint32_t took = cycles_now() - t0;
    const bool released = LoneCs::read_out() && DcPin::read_out();
    const uint16_t ctlr1 = S::regs().CTLR1;
    const uint16_t statr = S::status();
    const uint8_t after = host_xfer(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode0,
                                    SpiClock::div16, SpiDataSize::bits8, true);
    print(serial, "  a polled read of 16 at /16 with the block held in reset: status ", st, " (spi_stalled ",
          spi_stalled, ") after ", took, " cycles (", bound_of_sixteen, " to ", wall_of_sixteen,
          ", the wire's 2048), D/C and select ", released ? "high" : "LOW", ", CTLR1 ", hex(ctlr1), " STATR ", hex(statr), ", the next request ", after, crlf);
    bench.verdict("A POLLED REQUEST OVER A BLOCK HELD IN RESET ends spi_stalled within its derived bound, never "
                  "before it, the D/C and the select released",
                  st == spi_stalled && took >= bound_of_sixteen && took < wall_of_sixteen && released);
    bench.verdict("the host has put the block back: the reset line let go, SPE and MSTR up, nothing in the receive "
                  "buffer, and with no recover() the next request is spi_ok",
                  (ctlr1 & (spi_spe | spi_mstr)) == (spi_spe | spi_mstr) && (statr & (spi_rxne | spi_ovr)) == 0u &&
                      after == spi_ok);

    // 2. The host with engines, every stall path, each followed at once by a
    // polled and an asynchronous request. Mode 3: SCK idles high, so the reset
    // moves its pad, with the select already released.
    constexpr SpiMode mode = SpiMode::mode3;
    uint8_t good_stalls = 0;
    uint8_t good_next = 0;
    for (const Stage s : {Stage::held_in_reset, Stage::spe_down_write, Stage::spe_down_command}) {
        dma_ready();
        DmaHost::prime(mode, SpiClock::div16);
        uint32_t wall = 0;
        const uint8_t stalled = stall_stage<DmaHost>(s, mode, wall);
        const uint32_t stall_entries = dma_isr_entries;
        const bool lone_high = LoneCs::read_out() && DcPin::read_out();
        const uint32_t floor = s == Stage::spe_down_command ? bound_of_command : bound_of_sixteen;
        const uint32_t bound = s == Stage::spe_down_command ? wall_of_command : wall_of_sixteen;
        const uint8_t next_polled = dma_xfer(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 16, SpiClock::div16,
                                             SpiDataSize::bits8, true, mode);
        const uint32_t polled_entries = dma_isr_entries;
        const uint8_t next_async = dma_xfer(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 16, SpiClock::div16,
                                            SpiDataSize::bits8, false, mode);
        const uint32_t async_entries = dma_isr_entries;
        dma_done();
        print(serial, "  ", stage_name(s), ": status ", stalled, " after ", wall, " cycles (", floor, " to ", bound,
              "), D/C and select ", lone_high ? "high" : "LOW", ", channel interrupts ", stall_entries, "; then polled ",
              next_polled, " (", polled_entries, " interrupts), asynchronous ", next_async, " (", async_entries, ")",
              crlf);
        if (stalled == spi_stalled && wall >= floor && wall < bound && lone_high && stall_entries == 0u) {
            ++good_stalls;
        }
        if (next_polled == spi_ok && polled_entries == 0u && next_async == spi_ok && async_entries == 1u) {
            ++good_next;
        }
    }
    bench.verdict("EVERY STALL PATH ON THE ENGINED HOST, the polled command phase in front of an asynchronous block "
                  "among them, ends spi_stalled inside start() within its bound and never before it, the D/C and "
                  "the select released, no channel interrupt",
                  good_stalls == 3u);
    bench.verdict("and each time, with no recover(), the next polled request and the next asynchronous one, on the "
                  "engines with their one interrupt, are spi_ok",
                  good_next == 3u);

    // 3. A CLOSED GATE, which a reset does not open. Each request stalls on its
    // own budget; the gate opened, the block has lost its registers to the
    // pulse under the closed gate, so the first request stalls once more and
    // puts it back, and the one after it runs.
    host_ready();
    S::bus_clock(false);
    const uint16_t closed_ctlr1 = S::regs().CTLR1;
    const uint16_t closed_statr = S::status();
    uint32_t tg = cycles_now();
    const uint8_t gate_1 = host_xfer(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode0, SpiClock::div16,
                                     SpiDataSize::bits8, true);
    const uint32_t gate_1_took = cycles_now() - tg;
    tg = cycles_now();
    const uint8_t gate_2 = host_xfer(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode0, SpiClock::div16,
                                     SpiDataSize::bits8, true);
    const uint32_t gate_2_took = cycles_now() - tg;
    tg = cycles_now();
    const bool drained = S::disable();
    const uint32_t drain_took = cycles_now() - tg;
    S::bus_clock(true);
    const uint8_t open_1 = host_xfer(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode0, SpiClock::div16,
                                     SpiDataSize::bits8, true);
    const uint8_t open_2 = host_xfer(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode0, SpiClock::div16,
                                     SpiDataSize::bits8, true);
    print(serial, "  the gate closed: CTLR1 reads ", hex(closed_ctlr1), ", STATR ", hex(closed_statr),
          "; two requests ", gate_1, " after ", gate_1_took, " and ", gate_2, " after ", gate_2_took,
          " cycles; Spi::disable() ", drained ? "true" : "false", " after ", drain_took,
          " cycles; the gate opened, requests ", open_1, " and ", open_2, crlf);
    bench.verdict("A CLOSED GATE IS NOT CURED: every request behind it stalls, each on its own bound",
                  gate_1 == spi_stalled && gate_2 == spi_stalled && gate_1_took < wall_of_sixteen &&
                      gate_2_took < wall_of_sixteen);
    bench.verdict("the gate opened, the host needs no recover(): at most one more stall, and the next request is spi_ok",
                  (open_1 == spi_ok || open_1 == spi_stalled) && open_2 == spi_ok);

    // 4. A TRANSFER ERROR, real, with no recover() after it. The channels' stop in
    // the middle of a block, the select released from the vector, the next
    // requests at once. Exact against the peer in letter t.
    uint8_t good_faults = 0;
    uint8_t good_after_fault = 0;
    real_faults<DmaHost>(good_faults, good_after_fault);
    bench.verdict("A BLOCK RUNNING PAST THE END OF A MEMORY, read or written, is a transfer error: dma_isr() true once, "
                  "spi_dma_fault, the D/C and the select released",
                  good_faults == 2u);
    bench.verdict("and with no recover() the next polled request and the next on the engines are spi_ok; a refused "
                  "request behind a fault leaves the block alone and the reset owed, the next prime() pays it",
                  good_after_fault == 3u);

    // 5. THE REST OF THE MAP: a real request whose buffer sits on a hole
    // (test_ch32_dma letter d's five, read and written), and for reading three
    // more classes - a core-private address, the closed gate of TIM1 and the end
    // of the flash. Printed, and judged for what must hold either way: a block
    // ends.
    HoleSurvey holes{};
    constexpr uint32_t both_ways[] = {0x30000000u, 0x40030000u, 0x50000000u, 0x60000000u, 0xF0000000u};
    constexpr uint32_t read_only[] = {0xE000E000u, 0x40012C00u, device::flash_bytes};
    rcc()->PB2PCENR &= ~rcc_pb2_tim1;
    dma_ready();
    for (const uint32_t a : both_ways) {
        hole_line(holes, "read from", a,
                  dma_xfer(LoneCs::ref(), nullptr, 0, reinterpret_cast<const uint8_t*>(a), rx_buf, 16, SpiClock::div16,
                           SpiDataSize::bits8, false));
        hole_line(holes, "written into", a,
                  dma_xfer(LoneCs::ref(), nullptr, 0, tx_buf, reinterpret_cast<uint8_t*>(a), 16, SpiClock::div16,
                           SpiDataSize::bits8, false));
    }
    for (const uint32_t a : read_only) {
        hole_line(holes, "read from", a,
                  dma_xfer(LoneCs::ref(), nullptr, 0, reinterpret_cast<const uint8_t*>(a), rx_buf, 16, SpiClock::div16,
                           SpiDataSize::bits8, false));
    }
    const uint8_t after_holes = dma_xfer(LoneCs::ref(), nullptr, 0, tx_buf, rx_buf, 16, SpiClock::div16,
                                         SpiDataSize::bits8, false);
    dma_done();
    print(serial, "  ", holes.blocks, " blocks over the rest of the map: ", holes.faults, " transfer errors, ",
          holes.hung, " with no answer", crlf);
    bench.verdict("every one of those blocks ended, and the next asynchronous request is spi_ok",
                  holes.hung == 0u && after_holes == spi_ok);
    // The engines count what was staged here; letter d judges that it has none.
    DmaTx::clear_faults();
    DmaRx::clear_faults();
    xfer_dc = PinRef{};
    host_ready();
}

// ===========================================================================
// v - OVR and its clear, on the resource (wireless)
// ===========================================================================
//
// 16.2.7 (14.2.7 on the CH32V003): DATAR then STATR clears OVR. Two frames
// clocked with nothing read, the first left in the buffer, the second lost
// under OVR; then the ways a program goes at the flag, each from a block just
// reset, judged at /32 with MISO floating (the frames' data is not read). Every
// judged load comes a microsecond after the read or load before it, so a flag
// that falls a load late is not read as standing. A FLAG TEST HAS NO TOLERANCE:
// each verdict reads STATR's bits after the frames have certainly ended, and the
// arithmetic is two_frames_unread()'s.

/// Two frames at /32 with nothing read: 8 x 32 = 256 cycles each, 512 in all
/// from the first write, against the delay's 15 us = 720 cycles at 48 MHz.
void two_frames_unread() {
    SpiRegs& regs = S::regs();
    (void)Host::recover();   // OVR and RXNE down whatever the last sequence left
    regs.DATAR = 0x11u;
    for (uint32_t i = 2000u; (regs.STATR & spi_txe) == 0u && i != 0u; --i) {
    }
    regs.DATAR = 0x22u;
    (void)delay_us(clock, 15);
}

void tv_overrun() {
    host_ready();
    Host::prime(SpiMode::mode0, SpiClock::div32, SpiDataSize::bits8);
    SpiRegs& regs = S::regs();
    auto settled = [&regs] {
        (void)delay_us(clock, 1);
        return static_cast<uint16_t>(regs.STATR);
    };

    // A load that shows the flag, then a read, then two loads.
    two_frames_unread();
    const uint16_t raised = regs.STATR;
    (void)regs.DATAR;
    const uint16_t shown_read_1 = settled();
    const uint16_t shown_read_2 = settled();

    // 16.2.7's order, no load first: a read, then two loads.
    two_frames_unread();
    (void)regs.DATAR;
    const uint16_t order_1 = settled();
    const uint16_t order_2 = settled();

    // A read again after those loads, the buffer empty.
    (void)regs.DATAR;
    const uint16_t order_read = settled();

    // RW0 (16.3.3): a zero written.
    two_frames_unread();
    regs.STATR = static_cast<uint16_t>(~spi_ovr);
    const uint16_t wrote_zero = settled();

    // The verbs.
    two_frames_unread();
    S::clear_overrun();
    const uint16_t verb_clear = settled();
    two_frames_unread();
    S::flush_rx();
    const uint16_t verb_flush = settled();

    // The host's polled write: the answers overrun the buffer from the second
    // frame, and its tail leaves neither flag. Sixteen frames at /8.
    (void)Host::recover();
    fill_pattern(tx_buf, 16, 0x5A);
    const uint8_t write_status = host_xfer(LoneCs::ref(), nullptr, 0, tx_buf, nullptr, 16, SpiMode::mode0,
                                           SpiClock::div8, SpiDataSize::bits8, true);
    const uint16_t after_write = settled();
    (void)Host::recover();

    print(serial, "  two frames unread at /32: STATR ", hex(raised), "; a read after that load, then two loads ",
          hex(shown_read_1), " ", hex(shown_read_2), crlf, "  no load first, a read then two loads ", hex(order_1), " ",
          hex(order_2), ", a read again ", hex(order_read), crlf, "  a zero written to OVR ", hex(wrote_zero),
          ", clear_overrun() ", hex(verb_clear), ", flush_rx() ", hex(verb_flush), crlf,
          "  a polled write of 16 frames, status ", write_status, ": STATR ", hex(after_write), crlf);

    const uint16_t both = spi_ovr | spi_rxne;
    bench.verdict("OVR RISES on a frame completed with one standing unread, the buffer still full",
                  (raised & both) == both);
    if constexpr (device::part == Ch32Part::v003) {
        bench.verdict("ON THE CH32V003 16.2.7'S ORDER DOES NOT CLEAR IT: a read then loads leave it standing, RXNE down; "
                      "a read after a load that showed it clears it, the buffer full or empty",
                      (order_1 & both) == spi_ovr && (order_2 & both) == spi_ovr && (order_read & both) == 0u &&
                          (shown_read_1 & both) == 0u && (shown_read_2 & both) == 0u);
    } else {
        bench.verdict("ON THE CH32V006 16.2.7'S ORDER CLEARS IT: after a read, the first load still shows it, RXNE down, "
                      "and the next does not, with or without a load before the read",
                      (order_1 & both) == spi_ovr && (order_2 & both) == 0u && (order_read & both) == 0u &&
                          (shown_read_1 & both) == spi_ovr && (shown_read_2 & both) == 0u);
    }
    bench.verdict("A ZERO WRITTEN TO OVR CLEARS NOTHING (the register table's RW0)", (wrote_zero & both) == both);
    bench.verdict("clear_overrun() and flush_rx() leave neither flag after a frame unread and a frame lost",
                  (verb_clear & both) == 0u && (verb_flush & both) == 0u);
    bench.verdict("a polled write leaves neither flag, its answers having overrun the buffer",
                  write_status == spi_ok && (after_write & both) == 0u);
}

// ===========================================================================
// w - when a request ends, the wire has ended
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
// read as well (docs/ch32v00x/spi.md). Any read off the idle level fails the verdict: no
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
    for (uint32_t i = 0; i < 2'000'000UL && !host_done; ++i) {
    }
    vector_read_after = end_stamp - t0;
    return f;
}

/// The polled and engined requests of one rung on host `H`: 2 frames, the four
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
                for (uint32_t i = 0; i < 2'000'000UL && !host_done; ++i) {
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

void tw_wire_end() {
    dma_ready();
    fill_pattern(tx_buf, 16, 0x5A);
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
    dma_done();
    if constexpr (image_ram_twin) {
        dma_ready<RamDmaHost>();
        for (const SpiClock c : rungs) {
            end_of_request<RamDmaHost>(c, asked, off_polled, off_engines, bad);
        }
        dma_done<RamDmaHost>();
    }
    host_ready();
    print(serial, "  ", asked, " pairs of requests: SCK off idle after start() ", off_polled, ", after dma_isr() ",
          off_engines, ", a bad answer or a select down ", bad, crlf);
    bench.verdict("WHEN A POLLED REQUEST ENDS THE WIRE HAS ENDED: SCK at its idle level on the instruction after start(), "
                  "all four modes, /256 and /64, read and write, a select and a null one",
                  asked != 0u && off_polled == 0u);
    bench.verdict("AND WHEN ONE ON THE ENGINES ENDS: the same in the vector, right after dma_isr() answered true",
                  asked != 0u && off_engines == 0u);
    bench.verdict("every request of the pass answered spi_ok, start() true on the loop and false on the engines, the "
                  "select high",
                  asked != 0u && bad == 0u);
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
    // THE PAIR'S CEILING (letter s), from the far chip's datasheet, table
    // 3-19: a client's answer is valid up to 25 ns after its shifting edge
    // and this host needs 5 ns of setup, 30 ns - inside the 41.7 ns half
    // period of /4 = 12 MHz, outside the 20.8 ns of /2. So an answer is
    // read to /4; what this host sends (its own data valid 5 ns after its
    // edge, the client's setup 5 ns) is heard to /2. The rungs above are
    // printed; the verdicts stop at those ceilings.
    const auto held_to = [](uint8_t broke, uint8_t code) { return broke == 8u || broke < code; };
    bench.verdict("BOTH HOST PATHS exact both ways against the peer's DMA serve at every code to /4 = 12 MHz, the "
                  "ceiling of a read (table 3-19)",
                  held_to(broke_at[0], 1) && held_to(broke_at[1], 1));
    bench.verdict("this host's TRANSMIT-ONLY paths reach a listening client exact at every code to /2 = 24 MHz",
                  held_to(broke_at[4], 0) && held_to(broke_at[5], 0));
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
    // transactions (kl::hand_cs) at the link's rate - on the engined host,
    // three of them asynchronous on its engines, one polled.
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
    dma_ready();
    kl::BusKernel::init_all();
    kl::hand_cs = true;
    kl::rate = link_clock;
    bus_ao_live = true;
    settle();
    DmaHost::prime(SpiMode::mode0, link_clock);
    CsPin::clear();
    link_hold();
    for (uint8_t i = 0; i < 4u; ++i) {
        post<kl::SpiArb>(kl::request(qtx[i], qrx[i], i == 2u));
    }
    kl::pump_until(4, 800);
    link_hold();
    CsPin::set();
    bus_ao_live = false;
    dma_done();
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
// t - the next request after a failure, against the peer
// ===========================================================================
//
// After spi_stalled or spi_dma_fault the next request starts on a clean block,
// whoever posted it and with no recover(). What says so is a second chip: the
// next request's frames, exact both ways. The exchange is commanded FIRST, then
// the host is brought up and never touched again, so that nothing but the host
// itself can have cleaned the block - no init(), no recover(), no prime() - and
// the request frames its own select on the peer's wire, its setup time the
// link's hold. The failed request frames the lone select, so the peer never sees it.
// Mode 3, whose idle SCK the reset moves.
//
// The transfer errors are real (fault_request(), past the end of a memory, the host's own letter h's stage), so the next
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
    bus_ao_live = true;
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
        rcc()->PB2PRSTR |= rcc_pb2_spi1;
    }
    post<kl::SpiArb>(first);
    post<kl::SpiArb>(second);
    kl::pump_until(2, 400);
    const bool selects_high = LoneCs::read_out() && CsPin::read_out();
    (void)delay_us(clock, 30);
    DmaHost::release();
    bus_ao_live = false;
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
            if (failure_then_next<DmaHost>(s, next_polled, seed)) {
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
            if (failure_then_next<DmaHost>(s, next_polled, seed)) {
                ++faults;
            }
            seed = static_cast<uint8_t>(seed + 5u);
        }
    }
    // A caller framing its own select: prime() has paid the reset, with the select
    // still high, so in mode 3 the SCK pad it moves reaches no selected device.
    for (const bool next_polled : {true, false}) {
        if (failure_then_next<DmaHost>(Stage::fault_rx, next_polled, seed, true)) {
            ++faults;
        }
        seed = static_cast<uint8_t>(seed + 5u);
    }
    bench.verdict("A REAL TRANSFER ERROR IN THE MIDDLE OF A BLOCK, THEN THE NEXT REQUEST WITH NO recover(): dma_isr() "
                  "true once, spi_dma_fault, the select released, the transmit engine's fault counted once, then a "
                  "polled and an asynchronous request exact, no frame of the faulted block in front of them; six cases, "
                  "two of them framing their own select after prime()",
                  faults == 6u);

    if constexpr (image_ram_twin) {
        uint8_t in_ram = 0;
        for (const Stage s : {Stage::held_in_reset, Stage::fault_rx}) {
            for (const bool next_polled : {true, false}) {
                if (failure_then_next<RamDmaHost>(s, next_polled, seed)) {
                    ++in_ram;
                }
                seed = static_cast<uint8_t>(seed + 5u);
            }
        }
        bench.verdict("the same with the polled loops in SRAM: a stall and a transfer error, four cases", in_ram == 4u);
    }

    if constexpr (image_queue) {
        uint8_t queued = 0;
        for (const Stage s : {Stage::held_in_reset, Stage::fault_rx, Stage::fault_tx}) {
            for (const bool next_polled : {true, false}) {
                if (kt::queued_behind(s, next_polled, seed)) {
                    ++queued;
                }
                seed = static_cast<uint8_t>(seed + 5u);
            }
        }
        bench.verdict("QUEUED BEHIND A FAILURE, THROUGH SpiBus: two requests posted from one dispatch, replies in "
                      "order, the failure's code then spi_ok, the second's frames exact, no recover(); six cases",
                      queued == 6u);
    }
    (void)link_command_mode();
    bench.verdict("the command channel is back after the letter", command(Op::ping));
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
    Pfic::disable(S::irq());   // polled: SPI1's vector stays dark
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
        print(serial, "REFUSED by SpiClient (spi_client_may_answer)", crlf);
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
    // Modes 2 and 3 (CPOL = 1, the answering client's, 16.2.3), MSb first,
    // the first answer bit 0 (0x7x) and 1 (0xDx): the bit the CPOL = 0
    // modes slip on.
    uint8_t exact = 0;
    for (uint8_t m = 2; m < 4u; ++m) {
        for (uint8_t b = 0; b < 2u; ++b) {
            const ClientBurst c = client_burst(m, false, false, static_cast<uint8_t>(0x2C + m),
                                               static_cast<uint8_t>((b != 0u ? 0xD1u : 0x71u) + m));
            char label[] = "mode 0 MSb, first bit 0";
            label[5] = static_cast<char>('0' + m);
            label[22] = static_cast<char>('0' + b);
            print_burst(label, c, 12);
            exact = static_cast<uint8_t>(exact + (burst_exact(c, 12) ? 1u : 0u));
        }
    }
    bench.verdict("THIS CHIP AS CLIENT ON ITS HARDWARE NSS, modes 2 and 3: every frame of a foreign host exact, its "
                  "answers exact FROM THE FIRST FRAME (the preload), whatever the first answer bit",
                  exact == 4u);
    // Modes 0 and 1 as an answering client: refused by the driver before
    // anything is commanded.
    uint8_t refused = 0;
    for (uint8_t m = 0; m < 2u; ++m) {
        const ClientBurst c = client_burst(m, false, false, 0x11, 0x22);
        char label[] = "mode 0 MSb";
        label[5] = static_cast<char>('0' + m);
        print_burst(label, c, 12);
        refused = static_cast<uint8_t>(refused + (!c.admitted ? 1u : 0u));
    }
    bench.verdict("an ANSWERING client in modes 0 and 1 is REFUSED by SpiClient (spi_client_may_answer: 16.2.3's "
                  "CPOL = 1, measured)",
                  refused == 2u);
    // LSb first, modes 2 and 3: the peer's host clocks LSb first too.
    uint8_t lsb_exact = 0;
    for (uint8_t m = 2; m < 4u; ++m) {
        const ClientBurst c = client_burst(m, true, false, static_cast<uint8_t>(0x3D + m), static_cast<uint8_t>(0x82 + m));
        char label[] = "mode 0 LSb";
        label[5] = static_cast<char>('0' + m);
        print_burst(label, c, 12);
        lsb_exact = static_cast<uint8_t>(lsb_exact + (burst_exact(c, 12) ? 1u : 0u));
    }
    bench.verdict("and LSb first, both ends agreeing, in modes 2 and 3: exact both ways", lsb_exact == 2u);
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

/// /8, /4 and /2, 32 frames on this host's engines against the peer's DMA serve, one property varied
/// at a time: what the client's MISO does
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
            e.count = 32;
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

BRIO_CH32_VECTOR(systick_handler) {
    brio::Ticker::tick();
    if constexpr (carries('i')) {
        if (dc_watch) {
            if (DcPin::read_out()) {
                dc_high_ticks = dc_high_ticks + 1u;
            } else {
                dc_low_ticks = dc_low_ticks + 1u;
            }
        }
    }
    for (uint32_t k = tick_drag; k != 0u; --k) {
        __asm__ volatile("");
    }
}
BRIO_CH32_VECTOR(usart1_handler) { (void)Serial::isr(); }

/// No task of this image arms SPI1's interrupt - the host has none and the
/// client letter runs polled - so an entry here is a finding: counted (letter
/// a judges zero) and every enable of the block dropped.
BRIO_CH32_VECTOR(spi1_handler) {
    spi_isr_entries = spi_isr_entries + 1u;
    S::rxne_interrupt(false);
    S::txe_interrupt(false);
    S::error_interrupt(false);
}

/// What letter w reads in the vector, right after dma_isr() answered true.
[[gnu::always_inline]] inline void read_the_end() {
    if constexpr (carries('w')) {
        sck_at_end = SckPin::read();
        end_stamp = cycles_now();
    }
}

/// The two channels' body: the arbiter's engined host while a kernel letter
/// runs, else the suite's engined host in either placement. Spelled once
/// and called from both vectors: two inline copies of it would cost the
/// CH32V003's images half a kilobyte.
[[gnu::noinline]] void spi_dma() {
    dma_isr_entries = dma_isr_entries + 1u;
    if constexpr (image_kernel) {
        if (bus_ao_live) {
            if (DmaHost::dma_isr()) {
                brio::post<kl::SpiArb>(brio::TransferDone{DmaHost::status()});
            }
            return;
        }
    }
    if constexpr (image_ram_twin) {
        if (ram_host_live) {
            if (RamDmaHost::dma_isr()) {
                read_the_end();
                host_done = true;
                dc_watch = false;
            }
            return;
        }
    }
    if constexpr (image_engines) {
        if (dma_host_live && DmaHost::dma_isr()) {
            read_the_end();
            host_done = true;
            dc_watch = false;
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
    bench.letter('i', "THE ENGINE A REQUEST NAMES, wireless: one frame on the engines, the command rule, the refusals",
                 ti_engine_rules);
    bench.letter('j', "the engines of bytes, wireless: 16-bit frames refused", tj_byte_engines);
    bench.letter('l', "the release contract on the same owner: released with a request standing, then "
                      "initialized again", tl_release_reinit);
    bench.letter('b', "THE ENGINES against the peer: four modes, 8 and 16 bits, a command in the loop, a command "
                      "alone, one frame", tb_async);
    bench.letter('c', "the polled path at every rate, timed", tc_rates);
    bench.letter('d', "THE DMA ENGINES on channels 2 and 3 against the peer: one interrupt, the 16-bit beat, the "
                      "odd refusal, the polled step", td_dma);
    bench.letter('g', "THE RATE ON THE WIRE: a frame inside an engined block, the peer deselected", tg_wire);
    bench.letter('e', "THE HARDWARE CRC against a foreign CRC unit, 8 and 16 bits, a mismatch caught", te_crc);
    bench.letter('f', "THE KERNEL: SpiBus over SpiHost, the rejection, the votes", tf_kernel);
    bench.letter('h', "the stalled exit and the transfer error, wireless: the next requests with no recover(), a "
                      "closed gate, the rest of the map",
                 th_stalled);
    bench.letter('v', "OVR AND ITS CLEAR on the resource: what raises it, which reads clear it, flush_rx()", tv_overrun);
    bench.letter('w', "WHEN A REQUEST ENDS THE WIRE HAS ENDED, wireless: SCK against its idle level at the flag and "
                      "after the request, both engines", tw_wire_end);
    bench.letter('k', "A HANDLER BESIDE THE POLLED LOOP IS A GAP ON THE BUS: ten windows exact under a dragged tick",
                 tk_handler_gap);
    bench.letter('t', "THE NEXT REQUEST AFTER A FAILURE against the peer: a stall and a transfer error, direct and "
                      "through SpiBus, exact with no recover()",
                 tt_next_after_failure);
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
