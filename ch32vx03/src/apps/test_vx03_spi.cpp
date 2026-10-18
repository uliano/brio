// test_vx03_spi - the reference bench suite for the SPI chapter of the
// CH32V203 and the CH32V303: ch32vx03/spi.hpp over RM ch. 20, every
// instance, both roles, the DMA engines, the hardware CRC and util/
// spi_bus.hpp's arbiter with not one line changed. Letters a, b, c and e
// are both series' single-board letters; h and i are the CH32V303's third
// instance against its second, registered on the part that has SPI3 and
// compiled out of every other image; d, j, k, l and m are THE PAIR's,
// against the desk partner. The I2S face is test_vx03_i2s's.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// THE SUITE HAS TWO SETS (util/testbench.hpp's second set):
//
//  - `z`, THE SINGLE-BOARD LETTERS (a, b, c, e, and h, i on the CH32V303),
//    runnable with the far board dark - on its console, or on its
//    spi_peer, whose dark listener drives nothing and is selected by
//    nothing: the pair's select is never driven low by them.
//  - `y`, THE PAIR'S LETTERS (d, j, k, l, m), against the far board
//    running `spi_peer`, commanded IN BAND over the bus under test through
//    avrxt/src/apps/spi_link.hpp, included by relative path: one source of
//    truth for the wire format on every architecture. They FAIL loudly
//    when nobody answers. `x`, by name, is the slip statistics behind
//    letter d's matrix (no verdict).
//
// WHAT IT MEASURES WITH:
//
//  - SPI1's own LOOPBACK on the CH32V203C8T6's board: a strap from PA7
//    (MOSI) to PA6 (MISO) through PA8, so the host's own frames come back.
//    PA8 is the third pad of that node and this suite NEVER drives it: it
//    is left a floating input for the whole run. The strap is probed at
//    every letter that needs it (a plain output on PA7 read on PA6 against
//    the opposite pull), never cached. SPI1 runs SOFTWARE SELECT ONLY: its
//    NSS pad PA4 is left an input (on the CH32V303 board it is the pair's
//    select wire).
//  - THE PAIR: the CH32V203C8T6's SPI2 (NSS PB12, SCK PB13, MISO PB14,
//    MOSI PB15) wired to the CH32V303VCT6's SPI3 on its SECOND column (NSS
//    PA4, SCK PC10, MISO PC11, MOSI PC12), both selects hardware NSS pads,
//    over the pair's cable with its series resistors and its two grounds
//    beside SCK (docs/boards/). Each board's suite
//    links on its own end; the far board runs `spi_peer`.
//  - On the CH32V303VCT6's evaluation board, four jumpers between SPI2's
//    column and SPI3's default one - PB12-PA15, PB13-PB3, PB14-PB4,
//    PB15-PB5, each looked for before it is used - for letters h and i.
//
// THE PADS. PA4..PA8 and PA15/PB3/PB4/PB5 (SPI1's two columns), PB12..
// PB15 (SPI2), PA4/PC10/PC11/PC12 (SPI3's second column, the CH32V303's
// pair link; R6: SPI3's two columns are one AFIO field, so every pair
// letter there starts with column 0's pads floating). NEVER TOUCHED:
// PA9/PA10 (the console), PA13/PA14 (the debug port), PA11/PA12 (the USB
// pads), PC14/PC15 and PD0/PD1 (the crystals), PA0 (the KEY), PB6/PB7 and
// PB10/PB11 (the pair's other wires) - and PB2, the LED, left undriven, as
// in every suite of this target.
//
// What is exercised, letter by letter:
//   a  THE RATES: every BR code on both instances read back, with the
//      SCK of SPI1's SECOND COLUMN measured on the pad itself - PB3 is
//      TIM2's channel 2 under TIM2's partial remap, and a pad driven by
//      one peripheral still reaches another's input; the fastest code,
//      at half the timer clock, judged only on a pad that carries no
//      wire - none faster than its wire at the rate asked - and the
//      effective bit rate of a saturated block on each instance; then
//      THE RATE ON THE WIRE, no wire needed: a frame inside an engined
//      block timed on the core's counter (the difference of a 2048- and a
//      256-frame block) against its SCK periods at the instance's own bus
//      over the division, SPI1 at /4, /16, /64 and in 16-bit frames, SPI2
//      at /2 and /16 - a loop shares one clock at both ends and cannot
//      tell a wrong one
//   b  THE HOST WITH ITS MISO HELD BY ITS OWN PORT, no wire: frames of
//      all ones and all zeros read back, both widths, both bit orders,
//      the four modes read back out of CTLR1
//   c  THE LOOPBACK on SPI1 (the CH32V203C8T6's strap): what no peer on
//      this desk can judge - the pump's interrupt a frame, a command phase
//      and a read with no out buffer, the ladder above the pair's ceiling
//      to the strap's 36 MHz, 4 KB at the fastest code that passes clean,
//      both DMA engines with 16-bit frames on them as half-words over
//      aligned buffers and on the pump one byte off, and SpiBus (=
//      BusMaster) over the whole of it. The four modes, both orders and the
//      CRC are the pair's (d, k), judged by a foreign client
//   e  THE FLAGS AND THE VECTORS: every flag with the sequence that
//      clears it, the bidirectional line mode on one pad, the
//      receive-only host and how it is stopped, the I2S register where
//      the part has no I2S
// and on the CH32V303RC and VC alone, over four wires between SPI2's
// column and SPI3's default one - PB12-PA15, PB13-PB3, PB14-PB4,
// PB15-PB5, each looked for before it is used:
//   h  THE HIGH-SPEED READ: a 256-byte block each way through the four
//      DMA channels (SPI2's on DMA1, SPI3's on DMA2) at BR /4 as the
//      baseline, then at /2 without HSRXEN and with it - each side's wrong
//      frames counted and sorted by kind, a frame read ONE BIT LATE being
//      a sample taken before the answer's edge, /4 and /2 with the mode
//      timed against the wire - and the verb's refusal at
//      /4; then SPI1's /2, 72 MHz of SCK: SPI1 on its second column -
//      SPI3's default pads, so the same wires - polled frame by frame over
//      SPI2 as a client, /8 as the baseline, /2 without HSRXEN and with
//      it, both counts printed and not judged because the far end takes a
//      clock at its own bus rate there - and, with no wire, whether this
//      die keeps HSRXEN2, the lot's second mode, with a fourth run where
//      it does
//   i  16-BIT FRAMES THROUGH THE ENGINES: a block of half-words each way
//      with every channel moving half-words, and the same bytes as 8-bit
//      frames beside it, at /2 without and with HSRXEN, /4, /8 and /16 -
//      the cost per frame and per byte against the core's counter and the
//      frames compared - a measurement, the transfer granularity being a
//      question the library has not settled - and every row's blocks
//      judged against the wire at the rate asked
// and the pair's, `y`, on the instance each part links (SPI2 on the
// CH32V203C8, SPI3's second column on the CH32V303VC), each bound derived
// in the section's comment from the two datasheets' SPI tables and the
// peer's loops:
//   d  THE PEER: ping, ident (kind 0x05, the far part's label, its
//      crystal), ten pings; the four modes and both bit orders against the
//      far client at each slew class of this end's pads; a bit-order
//      mismatch as an exact two-way reversal; the dummy MEASURED (a late
//      first answer); sink_slow; ss_pulse; a ten-second stress
//   j  THE LADDER: /256 to /2 on the polled host, the pump and the engines
//      against the far DMA serve, and the polled host against the far
//      POLLED run, every row's cycles beside the wire's; /2 with HSRXEN
//   k  THE EXTENDED EXCHANGE: 16-bit frames on every shape and both far
//      serves, the CRC at both widths against a foreign unit and a
//      polynomial mismatch caught at both ends, and this end's WRITE shape
//      witnessed by a far client that only listens
//   l  THE ROLES INVERT: this chip's SpiClient runs under the far host's
//      host_burst - its MISO's slew class walked, the four modes and both
//      orders, 16-bit frames, and the answer line handed back after
//   m  THE HARDWARE SELECT: MODF raised on this host's NSS input by the
//      far chip's ss_pulse, the demotion and its vector, the reset pulse
//      that clears it; and SSOE framing the far client
//   x  (by name) THE SLIP STATISTICS: each mode four times at /256, /64
//      and /8, on both far serves, behind the polled host and the pump,
//      and with the far client listening only - no verdict
//
// build: boards = v203c6,v203c8,v303vc
// build: groups = abef,c
// build: groups.v203c8 = abcef,djklmnox
// build: monitor_speed = 115200

#include <stdint.h>

#include <variant>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/delay.hpp"
#include "ch32vx03/dma.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/spi.hpp"
#include "ch32vx03/ticker.hpp"
#include "ch32vx03/tim.hpp"
#include "ch32vx03/usart.hpp"
#include "kernel/post.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/spi_bus.hpp"
#include "util/testbench.hpp"

#include "../../../avrxt/src/apps/spi_link.hpp"

namespace {

using namespace brio;

using P = Ch32vx03Platform<>;
using SysClock = Clock<ClockSource::pll, 144'000'000>;
constexpr SysClock clock;

using Serial = Uart<1, P, 64, 128>;
constexpr Serial serial;

TwoKeyTestBench<Serial, 'y'> bench;

// ---------------------------------------------------------------------------
// The two instances, their pads and their engines
// ---------------------------------------------------------------------------

/// The SECOND instance, letters a and b's: SPI2 where the part has it;
/// SPI1 otherwise, so the suite still LINKS on a part of the 32 KB tier
/// that carries one SPI.
constexpr uint8_t second_spi = device::spi_count >= 2u ? 2u : 1u;
constexpr bool has_two_spi = device::spi_count >= 2u;

using S1 = Spi<1>;
using S2 = Spi<second_spi>;

/// SPI1's default column, the loopback's: NSS PA4, SCK PA5, MISO PA6,
/// MOSI PA7.
constexpr SpiPins loop_pins = spi_pins_for(1, 0);
/// SPI1's SECOND column, letter a's ruler: NSS PA15, SCK PB3, MISO PB4,
/// MOSI PB5 - PB3 is where TIM2's channel 2 lands under TIM2's partial
/// remap, so the clock can be counted on the very pad that carries it.
constexpr SpiPins moved_pins = spi_pins_for(1, 1);
/// SPI2's one column: NSS PB12, SCK PB13, MISO PB14, MOSI PB15.
constexpr SpiPins second_pins = spi_pins_for(second_spi, 0);

using Host1 = SpiHost<1, loop_pins>;
using Dma1 = SpiHost<1, loop_pins, DmaTxEngine<1, S1::dma_tx_channel>,
                     DmaRxEngine<1, S1::dma_rx_channel>>;
using Moved = SpiHost<1, moved_pins>;
using Spi2Host = SpiHost<second_spi, second_pins>;
using Spi2Dma = SpiHost<second_spi, second_pins, DmaTxEngine<1, S2::dma_tx_channel>,
                        DmaRxEngine<1, S2::dma_rx_channel>>;

using SckPad = Pin<loop_pins.sck.port, loop_pins.sck.pin>;
using MisoPad = Pin<loop_pins.miso.port, loop_pins.miso.pin>;
using MosiPad = Pin<loop_pins.mosi.port, loop_pins.mosi.pin>;
using NssPad = Pin<loop_pins.nss.port, loop_pins.nss.pin>;
/// The third pad of the strapped node. Named ONLY so the suite can put
/// it back to a floating input and prove it is one; never driven.
using ThirdPad = Pin<'A', 8>;

/// THE PAIR'S LINK, the second set's instance: SPI2's one column on the
/// CH32V203C8 and RB, SPI3's SECOND column (NSS PA4, SCK PC10, MISO PC11,
/// MOSI PC12) on the CH32V303RC and VC - the desk pair's four wires
/// (docs/boards/), both selects hardware NSS pads. A part with SPI1 alone
/// links nothing and registers none of the letters; the names below are
/// formed on SPI1 there so that every part compiles them.
constexpr uint8_t pair_spi = spi_present(3) ? 3u : (has_two_spi ? 2u : 0u);
constexpr bool has_pair = pair_spi != 0u;
constexpr uint8_t pair_n = has_pair ? pair_spi : 1u;
constexpr uint8_t pair_code = pair_spi == 3u ? 1u : 0u;
constexpr SpiPins pair_pins = spi_pins_for(pair_n, pair_code);
using PairS = Spi<pair_n>;
using PairTx = DmaTxEngine<PairS::dma_tx_slot.controller, PairS::dma_tx_slot.channel, uint16_t>;
using PairRx = DmaRxEngine<PairS::dma_rx_slot.controller, PairS::dma_rx_slot.channel, uint16_t>;
using PairHost = SpiHost<pair_n, pair_pins>;
using PairDma = SpiHost<pair_n, pair_pins, PairTx, PairRx>;
using PairClient = SpiClient<pair_n, pair_pins>;
using PairNss = Pin<pair_pins.nss.port, pair_pins.nss.pin>;
using PairSck = Pin<pair_pins.sck.port, pair_pins.sck.pin>;
using PairMiso = Pin<pair_pins.miso.port, pair_pins.miso.pin>;
using PairMosi = Pin<pair_pins.mosi.port, pair_pins.mosi.pin>;
/// THE PAIR'S PAD CLASS, every pad either end drives on the link - this
/// end's SCK and MOSI as the host, its MISO as the client, and spi_peer's
/// answer line: the MEDIUM class, 10 MHz. The two datasheets' I/O AC
/// tables (the CH32V203's 4-21, the CH32V303's 4-23) give its edge as
/// 25 ns at most into 50 pF, inside the 27.8 ns half period of /4 = 18
/// MHz, the top rung the section below judges; the slow class's 125 ns
/// fits no rung above /32, and the fast class's 8 ns is the loudest edge
/// on the wire. Letter d's walk (this end's SCK and MOSI) and letter l's
/// (this end's MISO) print all three: through the pair's cable - SCK with
/// a series resistor at each end and two grounds of its own, MISO with one
/// at the CH32V203's end and MOSI with one at the CH32V303's
/// (docs/boards/) - every class carries every mode exact.
constexpr PinSpeed pair_pad_class = PinSpeed::medium;

/// A transaction that never answered: the suite's own word.
constexpr uint8_t no_answer = 200;

// ---------------------------------------------------------------------------
// The rulers
// ---------------------------------------------------------------------------

/// A cycle count that spans ticks: the kernel ticker's own cycles(), the
/// tick count and the counter's position composed right across the reload
/// (util/cycle_count.hpp); it wraps at 2^32, so only differences are used.
uint32_t cycles_now() {
    return Ticker::cycles();
}

void settle_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    (void)delay_us(clock, 500);
}

// ---------------------------------------------------------------------------
// The buffers (sized for the 10 KB tier: a 4 KB burst is this pattern
// sent sixteen times over, not a 4 KB array)
// ---------------------------------------------------------------------------

constexpr uint16_t chunk = 256;
/// Half-word aligned, so a 16-bit request over them is the engines' to
/// carry (ch32vx03/spi.hpp, "THE BEAT IS THE FRAME"); letter c moves one
/// byte in for the misaligned case.
alignas(4) uint8_t tx_buf[chunk];
alignas(4) uint8_t rx_buf[chunk];
uint8_t cmd_buf[4];

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

// ---------------------------------------------------------------------------
// The straps, probed and never cached
// ---------------------------------------------------------------------------

/// Drive one pad, read the other against the OPPOSITE pull: a floating
/// pad that merely follows its neighbour is not taken for a strap, and
/// a wire beats a pull. Both pads are left floating afterwards.
template <typename Driver, typename Reader>
bool pads_linked() {
    Reader::input(PinPull::down);
    Driver::output(true);
    (void)delay_us(clock, 20);
    const bool high = Reader::read();
    Reader::input(PinPull::up);
    Driver::clear();
    (void)delay_us(clock, 20);
    const bool low = !Reader::read();
    Driver::release();
    Reader::release();
    return high && low;
}

/// PA8 is the third pad of the MOSI/MISO node and must stay an input for
/// the whole run.
void third_pad_idle() { ThirdPad::release(); }

/// Whether PB3 - the pad letter a counts SPI1's clock on - carries a wire
/// to PB13, SPI2's clock pad: the link between SPI2 and SPI3 on the
/// CH32V303 evaluation board, looked for where the part has SPI3 (whose
/// default clock pad PB3 is) and absent everywhere else. The pads are
/// named through `on` so that no part forms them without it.
template <bool on = spi_present(3)>
bool sck_pad_wired() {
    if constexpr (on) {
        return pads_linked<Pin<'B', on ? 13 : 13>, Pin<'B', on ? 3 : 3>>();
    } else {
        return false;
    }
}

bool need_loop() {
    third_pad_idle();
    if (pads_linked<MosiPad, MisoPad>()) {
        return true;
    }
    print(serial, "  SKIPPED, no verdict claimed: no strap between PA7 (MOSI) and PA6 (MISO) "
                  "- probed now",
          crlf);
    return false;
}

// ---------------------------------------------------------------------------
// The plain host, waited out
// ---------------------------------------------------------------------------

volatile bool host_done = false;
volatile uint32_t spi1_isr_entries = 0;
volatile uint32_t spi1_dma_entries = 0;   ///< SPI1's two channel vectors, together
volatile bool dma_host_live = false;
volatile bool bus_ao_live = false;

void host_ready() {
    dma_host_live = false;
    bus_ao_live = false;
    (void)Host1::init(clock);
    NssPad::release();   // the select pad stays an input (the header)
}

uint8_t host_xfer(const uint8_t* cmd, uint8_t cmd_len, const uint8_t* tx, uint8_t* rx, uint16_t len,
                  SpiMode mode, SpiClock rate, SpiDataSize bits, bool polled) {
    Host1::Request r{};
    r.cs = {};   // software select: the suite frames nothing with PA4 (the header)
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
    spi1_isr_entries = 0;
    if (Host1::start(r)) {
        return Host1::status();
    }
    for (uint32_t i = 0; i < 4'000'000UL && !host_done; ++i) {
    }
    if (!host_done) {
        print(serial, "    STALL: STATR=", hex(S1::status()), " CTLR1=", hex(S1::regs().CTLR1),
              " CTLR2=", hex(S1::regs().CTLR2), " isr entries ", spi1_isr_entries, crlf);
        (void)Host1::recover();
        return no_answer;
    }
    return Host1::status();
}

uint8_t dma_xfer(const uint8_t* cmd, uint8_t cmd_len, const uint8_t* tx, uint8_t* rx, uint16_t len,
                 SpiClock rate, SpiDataSize bits, bool polled) {
    Dma1::Request r{};
    r.cs = {};
    r.cmd = lend<Lease::reply>(cmd);
    r.cmd_len = cmd_len;
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = len;
    r.mode = SpiMode::mode0;
    r.clock = rate;
    r.bits = bits;
    r.polled = polled;
    host_done = false;
    if (Dma1::start(r)) {
        return Dma1::status();
    }
    for (uint32_t i = 0; i < 4'000'000UL && !host_done; ++i) {
    }
    if (!host_done) {
        print(serial, "    STALL (dma): STATR=", hex(S1::status()),
              " ch", S1::dma_rx_channel, " flags=", hex(DmaChannel<1, S1::dma_rx_channel>::flags()),
              " ch", S1::dma_tx_channel, " flags=", hex(DmaChannel<1, S1::dma_tx_channel>::flags()),
              crlf);
        (void)Dma1::recover();
        return no_answer;
    }
    return Dma1::status();
}

// ---------------------------------------------------------------------------
// The rate on the wire
// ---------------------------------------------------------------------------

/// One polled block of `n` frames through an ENGINED host `H`, neither
/// buffer named - the transmit engine pouring its fixed cell and the
/// receive engine its sink - in core cycles. The channels' vectors finish
/// a polled block on this family, so they are the engined hosts' meanwhile.
template <typename H>
uint32_t engined_block(uint16_t n, SpiClock rate, SpiDataSize bits) {
    typename H::Request r{};
    r.cs = {};
    r.len = n;
    r.mode = SpiMode::mode0;
    r.clock = rate;
    r.bits = bits;
    r.polled = true;
    host_done = false;
    const uint32_t t0 = cycles_now();
    (void)H::start(r);
    return cycles_now() - t0;
}

/**
 * THE RATE ON THE WIRE, which a loop alone cannot tell - both ends of a
 * loop share the one clock: the time ONE FRAME takes inside an engined
 * block, from the difference of a 2048-frame and a 256-frame block (the
 * best of three of each) so that the transaction's fixed cost cancels, in
 * thousandths of its SCK periods at the rate ASKED - the bus clock the
 * instance divides, `bus_hz`, over the code's division - on the core's
 * counter, which counts HCLK. A host whose transmit buffer is refilled
 * while TXE stands keeps a complete data flow (20.2.2) - the clock does not
 * pause between frames - so nothing is added for a gap.
 */
template <typename H>
uint32_t frame_permille(uint32_t bus_hz, SpiClock rate, SpiDataSize bits) {
    auto best = [&](uint16_t n) {
        uint32_t least = 0xFFFF'FFFFUL;
        for (uint8_t k = 0; k < 3u; ++k) {
            const uint32_t c = engined_block<H>(n, rate, bits);
            least = c < least ? c : least;
        }
        return least;
    };
    const uint32_t big = best(2048);
    const uint32_t small = best(256);
    const uint32_t width = bits == SpiDataSize::bits16 ? 16u : 8u;
    const uint64_t want = 1792ULL * width * spi_division(rate) * SysClock::hz / bus_hz;
    return want == 0u ? 0u : static_cast<uint32_t>(1000ULL * (big - small) / want);
}

/// The verdict on a reading: -1 % to +3 % of the wire at the rate asked.
bool wire_ok(uint32_t permille) { return permille >= 990u && permille <= 1030u; }

/// A block's time in thousandths of its wire at the rate asked - `frames`
/// frames of `width` bits at PB1 over `division` - which a link between
/// two instances of one bus cannot tell wrong.
uint32_t block_permille(uint32_t cycles, uint32_t frames, uint32_t width, uint32_t division) {
    const uint64_t wire = static_cast<uint64_t>(frames) * width * division * SysClock::hz /
                          SysClock::pclk1_hz;
    return wire == 0u ? 0u : static_cast<uint32_t>(1000ULL * cycles / wire);
}

// ===========================================================================
// a - the rates, and the clock measured on its own pad
// ===========================================================================

/// TIM2 as an EDGE COUNTER on PB3: the partial remap (column 1) puts
/// TIM2's channel 2 on that pad, the channel is configured as an input,
/// and the slave controller's external clock mode 1 makes TI2FP2's
/// rising edges the counter's clock. The pad stays the SPI's alternate
/// output throughout - a pad driven by one peripheral still reaches
/// another's input on this family.
using SckCounter = Tim<2>;

bool sck_counter_arm() {
    SckCounter::init();
    if (!SckCounter::remap(1)) {
        return false;
    }
    if (!SckCounter::configure({.prescaler = 0, .period = 0xFFFF})) {
        return false;
    }
    if (!SckCounter::capture_channel(1, {.select = TimChannelSelect::direct,
                                         .polarity = TimCapturePolarity::rising,
                                         .prescaler = TimCapturePrescaler::every,
                                         .filter = 0,
                                         .enable = true})) {
        return false;
    }
    if (!SckCounter::slave({.mode = TimSlaveMode::external_clock1, .trigger = TimTrigger::ti2})) {
        return false;
    }
    SckCounter::enable(true);
    return true;
}

void ta_rates() {
    // ---- the register's own vocabulary, read back on both instances ----
    uint8_t read_back = 0;
    S1::bus_clock(true);
    S1::reset();
    for (uint8_t code = 0; code < 8u; ++code) {
        (void)S1::configure({.role = SpiRole::host, .clock = static_cast<SpiClock>(code)});
        if (S1::clock() == static_cast<SpiClock>(code)) {
            ++read_back;
        }
    }
    if constexpr (has_two_spi) {
        S2::bus_clock(true);
        S2::reset();
        for (uint8_t code = 0; code < 8u; ++code) {
            (void)S2::configure({.role = SpiRole::host, .clock = static_cast<SpiClock>(code)});
            if (S2::clock() == static_cast<SpiClock>(code)) {
                ++read_back;
            }
        }
    }
    bench.verdict("every BR code reads back out of CTLR1 on every instance this part has",
                  read_back == (has_two_spi ? 16u : 8u));

    print(serial, "  SPI1 divides PB2 = ", SysClock::pclk2_hz / 1'000'000u,
          " MHz, SPI2 divides PB1 = ", SysClock::pclk1_hz / 1'000'000u,
          " MHz: the same code is two frequencies", crlf);

    // ---- the clock counted on the pad that carries it ----
    // A wire on PB3 is a load the fastest code has to drive: looked for
    // first, while the pad is still a plain one.
    const bool pb3_wired = sck_pad_wired();
    if (!sck_counter_arm()) {
        bench.verdict("TIM2's channel 2 takes the partial remap onto PB3", false);
        return;
    }
    bench.verdict("TIM2's channel 2 takes the partial remap onto PB3, SPI1's second column's "
                  "clock pad",
                  SckCounter::remap() == 1u);

    if (!Moved::init(clock)) {
        bench.verdict("SPI1 comes up on its second column (PA15/PB3/PB4/PB5)", false);
        return;
    }
    bench.verdict("SPI1 comes up as a host on its SECOND column - the one remap this family's "
                  "SPI has (table 10-32)",
                  S1::enabled() && Afio::remap_code(Remap::spi1) == 1u);
    bench.verdict("and its BR field divides ITS OWN BUS: SPI1 is the PB2 instance, so its "
                  "reference is PCLK2 and not the system clock by coincidence",
                  Moved::reference_hz() == SysClock::pclk2_hz);

    constexpr uint16_t frames = 64;
    fill_pattern(tx_buf, frames, 0x5A);
    uint8_t exact_edges = 0;
    uint8_t never_early = 0;
    for (uint8_t code = 0; code < 8u; ++code) {
        const SpiClock rate = static_cast<SpiClock>(code);
        SckCounter::set_count(0);
        console_drain();
        const uint32_t t0 = cycles_now();
        Moved::Request r{};
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
        r.len = frames;
        r.clock = rate;
        r.polled = true;
        (void)Moved::start(r);
        const uint32_t cycles = cycles_now() - t0;
        const uint32_t edges = SckCounter::count();
        const uint32_t want = 8UL * frames;
        const uint32_t sck = Moved::sck_hz(rate);
        // The wire's own time for the burst at the rate ASKED - PB2 over the
        // code's division, not what the driver believes - in core cycles.
        const uint32_t wire = static_cast<uint32_t>(
            (8ULL * frames * spi_division(rate) * SysClock::hz) / SysClock::pclk2_hz);
        if (cycles >= wire) {
            ++never_early;
        }
        print(serial, "  /", spi_division(rate), " (", sck / 1000u, " kHz): ", edges,
              " rising edges on PB3 (want ", want, "), ", cycles, " cycles for ", frames,
              " frames (the wire alone ", wire, ")", crlf);
        // At /2 SCK is HALF THE TIMER CLOCK, the external clock mode's own
        // ceiling: a pad with a wire on it is a load that decides the count
        // there, so on such a pad that one row is printed and not judged.
        if (edges == want || (code == 0u && pb3_wired)) {
            ++exact_edges;
        }
    }
    print(serial, "  the counter is TI2FP2 through the slave controller's external clock mode "
                  "1, and the pad is the SPI's alternate OUTPUT throughout",
          crlf);
    if (pb3_wired) {
        print(serial, "  PB3 carries the wire to PB13: the /2 row - SCK at half the timer "
                      "clock, over that load - is printed and not judged",
              crlf);
    }
    bench.verdict("THE CLOCK IS COUNTED ON ITS OWN PAD: eight rising edges per 8-bit frame at "
                  "EVERY BR code, the fastest included where the pad carries no wire - a pad "
                  "driven by one peripheral reaches another's input, and the counter follows SCK "
                  "to half the timer clock",
                  exact_edges == 8u);
    bench.verdict("and no code is faster than its wire: 64 frames take at least eight SCK "
                  "periods each at the rate asked, PB2 over the division (the polled path's own "
                  "cost the rest; the engines below time the wire)",
                  never_early == 8u);

    SckCounter::enable(false);
    SckCounter::release();
    Moved::release();
    (void)Afio::remap(Remap::spi1, 0);

    // ---- the effective bit rate of a saturated block, per instance ----
    if constexpr (has_two_spi) {
        (void)Spi2Host::init(clock);
        fill_pattern(tx_buf, chunk, 0x11);
        for (uint8_t code = 2; code < 8u; code += 2u) {
            const SpiClock rate = static_cast<SpiClock>(code);
            console_drain();
            const uint32_t t0 = cycles_now();
            Spi2Host::Request r{};
            r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
            r.len = chunk;
            r.clock = rate;
            r.polled = true;
            (void)Spi2Host::start(r);
            const uint32_t cycles = cycles_now() - t0;
            const uint32_t bits = 8UL * chunk;
            const uint32_t measured = cycles == 0u ? 0u
                                                   : static_cast<uint32_t>(
                                                         (static_cast<uint64_t>(bits) *
                                                          SysClock::hz) /
                                                         cycles);
            print(serial, "  SPI2 /", spi_division(rate), ": nominal ",
                  Spi2Host::sck_hz(rate) / 1000u, " kHz, ", chunk,
                  " bytes polled measured ", measured / 1000u, " kbit/s", crlf);
        }
        bench.verdict("SPI2's polled bursts run at its OWN bus's divisions - the effective bit "
                      "rate follows PB1 and not the system clock",
                      Spi2Host::reference_hz() == SysClock::pclk1_hz);
        Spi2Host::release();
    } else {
        print(serial, "  this part has one SPI: no second instance to time", crlf);
    }

    // ---- THE RATE ON THE WIRE, inside an engined block, per instance ----
    // No wire wanted: the transmit engine clocks the frames out whatever
    // MISO reads. SPI1 divides PB2, SPI2 PB1 - the bus a loop could not
    // tell wrong.
    Dma<1>::open();
    dma_host_live = true;
    (void)Dma1::init(clock);
    NssPad::release();   // the select pad stays an input (the header)
    const uint32_t s1_d4 = frame_permille<Dma1>(SysClock::pclk2_hz, SpiClock::div4,
                                                SpiDataSize::bits8);
    const uint32_t s1_d16 = frame_permille<Dma1>(SysClock::pclk2_hz, SpiClock::div16,
                                                 SpiDataSize::bits8);
    const uint32_t s1_d64 = frame_permille<Dma1>(SysClock::pclk2_hz, SpiClock::div64,
                                                 SpiDataSize::bits8);
    const uint32_t s1_w4 = frame_permille<Dma1>(SysClock::pclk2_hz, SpiClock::div4,
                                                SpiDataSize::bits16);
    Dma1::release();
    print(serial, "  a frame inside an engined block, in thousandths of its wire at the rate "
                  "asked: SPI1 /4 ",
          s1_d4, ", /16 ", s1_d16, ", /64 ", s1_d64, ", 16-bit /4 ", s1_w4, crlf);
    bool engines_on_time = wire_ok(s1_d4) && wire_ok(s1_d16) && wire_ok(s1_d64) &&
                           wire_ok(s1_w4);
    if constexpr (has_two_spi) {
        (void)Spi2Dma::init(clock);
        const uint32_t s2_d2 = frame_permille<Spi2Dma>(SysClock::pclk1_hz, SpiClock::div2,
                                                       SpiDataSize::bits8);
        const uint32_t s2_d16 = frame_permille<Spi2Dma>(SysClock::pclk1_hz, SpiClock::div16,
                                                        SpiDataSize::bits8);
        Spi2Dma::release();
        print(serial, "  SPI2 /2 ", s2_d2, ", /16 ", s2_d16, crlf);
        engines_on_time = engines_on_time && wire_ok(s2_d2) && wire_ok(s2_d16);
    }
    dma_host_live = false;
    bench.verdict("THE ENGINES RUN AT THE RATE ASKED: a frame inside a block takes its SCK "
                  "periods at the instance's own bus over the division - SPI1 at /4, /16 and /64 "
                  "and in 16-bit frames, SPI2 at /2 and /16 where the part has it - each -1 % to "
                  "+3 % on the core's counter",
                  engines_on_time);
    host_ready();
}

// ===========================================================================
// b - the host with its MISO held by its own port
// ===========================================================================

void tb_held() {
    third_pad_idle();
    // SPI1 as a host with SCK alone handed to the peripheral: MOSI is
    // RELEASED (PA7, PA6 and PA8 are one node on this board, so a driven
    // MOSI would fight the port) and PA6 is an ordinary output whose
    // level is what every frame reads back.
    Pfic::disable(S1::irq);
    S1::bus_clock(true);
    S1::reset();
    (void)S1::remap(0);
    SckPad::function();
    MosiPad::release();
    NssPad::release();

    struct Case {
        SpiMode mode;
        SpiDataSize bits;
        bool lsb;
    };
    constexpr Case cases[] = {
        {SpiMode::mode0, SpiDataSize::bits8, false},  {SpiMode::mode1, SpiDataSize::bits8, false},
        {SpiMode::mode2, SpiDataSize::bits8, false},  {SpiMode::mode3, SpiDataSize::bits8, false},
        {SpiMode::mode0, SpiDataSize::bits16, false}, {SpiMode::mode0, SpiDataSize::bits8, true},
    };
    uint8_t ones = 0, zeros = 0, modes = 0;
    for (const Case& c : cases) {
        (void)S1::configure({.role = SpiRole::host,
                             .mode = c.mode,
                             .clock = SpiClock::div64,
                             .bits = c.bits,
                             .lsb_first = c.lsb});
        S1::enable();
        S1::flush_rx();
        if (S1::mode() == c.mode && S1::bits() == c.bits) {
            ++modes;
        }
        const uint16_t mask = spi_frame_mask(c.bits);
        for (uint8_t level = 0; level < 2u; ++level) {
            MisoPad::output(level != 0u);
            (void)delay_us(clock, 5);
            S1::flush_rx();
            S1::data(c.bits, 0x00);
            uint32_t spins = 400'000u;
            while (!S1::rxne() && spins-- != 0u) {
            }
            const uint16_t got = S1::data(c.bits);
            if (level != 0u && got == mask) {
                ++ones;
            }
            if (level == 0u && got == 0u) {
                ++zeros;
            }
        }
        (void)S1::disable();
    }
    print(serial, "  six configurations x two levels: ", ones, " read all ones, ", zeros,
          " read all zeros, ", modes, " of 6 read their own mode and width back", crlf);
    bench.verdict("A HOST CLOCKS ITSELF: with MISO held high by the port every frame comes back "
                  "all ones, at both widths, both bit orders and all four modes",
                  ones == 6u);
    bench.verdict("... and held low, all zeros - the receive path is the pad's level and not a "
                  "leftover",
                  zeros == 6u);
    bench.verdict("the four modes and both widths read back out of CTLR1", modes == 6u);

    // A frame written with no clock configured at all is NOT sent: the
    // host's clock is SPE's, and BSY with SPE down is the question.
    (void)S1::disable();
    const bool idle = !S1::busy();
    bench.verdict("with SPE down the block is not busy - the host's clock is the enable's",
                  idle);

    MisoPad::release();
    host_ready();
}

// ===========================================================================
// c - the loopback: the pump, the ladder above the pair, the engines, the kernel
// ===========================================================================

namespace kl {

using SpiArb = SpiBus<Host1, P, 4>;

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

Host1::Request request(const uint8_t* tx, uint8_t* rx, bool polled) {
    Host1::Request r{};
    r.cs = {};
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = 8;
    r.mode = SpiMode::mode0;
    r.clock = SpiClock::div32;
    r.polled = polled;
    r.reply = reply_to<Probe, SpiDone>();
    return r;
}

}  // namespace kl

void tc_loop() {
    if (!need_loop()) {
        return;
    }
    host_ready();

    // ---- the pump: one interrupt a frame ----
    // The four modes, both widths and both bit orders are the pair's (d, l),
    // judged by a foreign client; here the pump's own count.
    fill_pattern(tx_buf, 32, 0x20);
    for (uint16_t i = 0; i < 32; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint8_t ps0 = host_xfer(nullptr, 0, tx_buf, rx_buf, 32, SpiMode::mode0, SpiClock::div16,
                                  SpiDataSize::bits8, false);
    bench.verdict("THE LOOPBACK ON THE PUMP: 32 frames come back byte-exact", ps0 == spi_ok && same(tx_buf, rx_buf, 32));
    bench.verdict("the pump runs on RXNE: one interrupt per frame and no more", spi1_isr_entries == 32u);

    // ---- a command phase and a read with no out buffer ----
    cmd_buf[0] = 0x9F;
    fill_pattern(tx_buf, 16, 0x71);
    for (uint16_t i = 0; i < 16; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint8_t cs = host_xfer(cmd_buf, 1, tx_buf, rx_buf, 16, SpiMode::mode0, SpiClock::div16,
                                 SpiDataSize::bits8, false);
    bench.verdict("a command frame then a data phase, the data read back exact",
                  cs == spi_ok && same(tx_buf, rx_buf, 16));
    for (uint16_t i = 0; i < 8; ++i) {
        rx_buf[i] = 0;
    }
    const uint8_t rd = host_xfer(nullptr, 0, nullptr, rx_buf, 8, SpiMode::mode0, SpiClock::div16,
                                 SpiDataSize::bits8, true);
    bool ff = true;
    for (uint16_t i = 0; i < 8; ++i) {
        ff = ff && rx_buf[i] == 0xFFu;
    }
    bench.verdict("a read with no out buffer clocks 0xFF dummies and reads them back on the loop",
                  rd == spi_ok && ff);

    // ---- the ladder, and 4 KB at the fastest code that is clean ----
    uint32_t best = 0;
    uint32_t first_bad = 0;
    uint8_t never_early = 0;
    for (uint8_t code = 8; code-- > 0;) {
        const SpiClock rate = static_cast<SpiClock>(code);
        fill_pattern(tx_buf, chunk, static_cast<uint8_t>(0xA0u + code));
        for (uint16_t i = 0; i < chunk; ++i) {
            rx_buf[i] = 0xEE;
        }
        console_drain();
        const uint32_t t0 = cycles_now();
        const uint8_t st = host_xfer(nullptr, 0, tx_buf, rx_buf, chunk, SpiMode::mode0, rate,
                                     SpiDataSize::bits8, true);
        const uint32_t cycles = cycles_now() - t0;
        const bool ok = st == spi_ok && same(tx_buf, rx_buf, chunk);
        // A polled read keeps one frame in flight at the top codes and is
        // slower than its wire there; no code may be faster than it.
        const uint64_t wire = 8ULL * chunk * spi_division(rate) * SysClock::hz / SysClock::pclk2_hz;
        if (cycles >= wire) {
            ++never_early;
        }
        print(serial, "  /", spi_division(rate), " (", Host1::sck_hz(rate) / 1000u, " kHz): ",
              chunk, " bytes polled in ", cycles, " cycles (", cycles / chunk, " per frame)",
              ok ? "  byte-exact" : "  MISMATCH", crlf);
        if (ok && first_bad == 0u) {
            best = Host1::sck_hz(rate);
        } else if (!ok && first_bad == 0u) {
            first_bad = Host1::sck_hz(rate);
        }
    }
    print(serial, "  the loop is clean to ", best / 1000u, " kHz");
    if (first_bad != 0u) {
        print(serial, " and breaks at ", first_bad / 1000u, " kHz");
    }
    print(serial, crlf);
    bench.verdict("the strap carries every BR code the block offers, or names the one it does "
                  "not",
                  best >= 1'000'000UL);
    bench.verdict("and no code is faster than its wire: 256 frames take at least eight SCK "
                  "periods each at the rate asked, PB2 over the division (letter a times the "
                  "wire itself, on the engines)",
                  never_early == 8u);

    // 4 KB at the fastest code that passed: the same chunk sixteen times,
    // which is what a 10 KB part can afford.
    const auto top = spi_rate_for(Host1::reference_hz(), best);
    uint16_t bad_chunks = 0;
    if (top) {
        console_drain();
        const uint32_t t0 = cycles_now();
        for (uint8_t k = 0; k < 16u; ++k) {
            fill_pattern(tx_buf, chunk, static_cast<uint8_t>(k * 13u + 3u));
            for (uint16_t i = 0; i < chunk; ++i) {
                rx_buf[i] = 0xEE;
            }
            const uint8_t st = host_xfer(nullptr, 0, tx_buf, rx_buf, chunk, SpiMode::mode0, *top,
                                         SpiDataSize::bits8, true);
            if (st != spi_ok || !same(tx_buf, rx_buf, chunk)) {
                ++bad_chunks;
            }
        }
        const uint32_t cycles = cycles_now() - t0;
        print(serial, "  4096 bytes at ", best / 1000u, " kHz in ", cycles / 1000u,
              " thousand cycles, ", bad_chunks, " of 16 chunks wrong", crlf);
    }
    bench.verdict("FOUR KILOBYTES at the fastest code the strap carries, byte-exact",
                  top.has_value() && bad_chunks == 0u);

    // ---- the DMA engines ----
    dma_host_live = true;
    bus_ao_live = false;
    Dma<1>::open();
    (void)Dma1::init(clock);
    fill_pattern(tx_buf, chunk, 0x3C);
    for (uint16_t i = 0; i < chunk; ++i) {
        rx_buf[i] = 0xEE;
    }
    console_drain();
    uint32_t t0 = cycles_now();
    const uint8_t ds = dma_xfer(nullptr, 0, tx_buf, rx_buf, chunk, SpiClock::div4,
                                SpiDataSize::bits8, false);
    uint32_t cycles = cycles_now() - t0;
    print(serial, "  ", chunk, " bytes at /4 on the engines (channels ", S1::dma_tx_channel,
          " and ", S1::dma_rx_channel, "): status ", ds, " in ", cycles, " cycles (",
          cycles / chunk, " per frame; the wire alone 32)",
          same(tx_buf, rx_buf, chunk) ? "  byte-exact" : "  MISMATCH", crlf);
    bench.verdict("A BLOCK THROUGH BOTH ENGINES completes spi_ok, byte-exact, on the two "
                  "channels table 11-5 wires to this instance",
                  ds == spi_ok && same(tx_buf, rx_buf, chunk));

    cmd_buf[0] = 0x0B;
    fill_pattern(tx_buf, 32, 0x5A);
    for (uint16_t i = 0; i < 32; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint8_t cds = dma_xfer(cmd_buf, 1, tx_buf, rx_buf, 32, SpiClock::div8,
                                 SpiDataSize::bits8, false);
    bench.verdict("a command frame on the pump then 32 data frames on the engines, exact",
                  cds == spi_ok && same(tx_buf, rx_buf, 32));

    // THE BEAT IS THE FRAME: sixteen 16-bit frames over half-word aligned
    // buffers ride the engines as half-words, one channel interrupt for the
    // transaction and no SPI one; the same frames over buffers one byte
    // off take the pump, an interrupt a frame.
    fill_pattern(tx_buf, 32, 0x42);
    for (uint16_t i = 0; i < 32; ++i) {
        rx_buf[i] = 0xEE;
    }
    spi1_isr_entries = 0;
    spi1_dma_entries = 0;
    const uint8_t hs = dma_xfer(nullptr, 0, tx_buf, rx_buf, 16, SpiClock::div8,
                                SpiDataSize::bits16, false);
    const uint32_t hs_isr = spi1_isr_entries;
    const uint32_t hs_dma = spi1_dma_entries;
    const bool hs_same = same(tx_buf, rx_buf, 32);
    fill_pattern(tx_buf + 1, 32, 0x24);
    for (uint16_t i = 0; i < 33; ++i) {
        rx_buf[i] = 0xEE;
    }
    spi1_isr_entries = 0;
    spi1_dma_entries = 0;
    const uint8_t ps = dma_xfer(nullptr, 0, tx_buf + 1, rx_buf + 1, 16, SpiClock::div8,
                                SpiDataSize::bits16, false);
    const uint32_t ps_isr = spi1_isr_entries;
    const uint32_t ps_dma = spi1_dma_entries;
    print(serial, "  16 frames of 16 bits on the engined host: aligned ", hs_isr, " SPI and ",
          hs_dma, " channel interrupt(s); one byte off ", ps_isr, " SPI and ", ps_dma,
          " channel interrupt(s)", crlf);
    bench.verdict("16-bit frames over aligned buffers ride the engines as half-words: one "
                  "channel interrupt, no SPI one, exact",
                  hs == spi_ok && hs_same && hs_isr == 0u && hs_dma == 1u);
    bench.verdict("over buffers one byte off they fall back to the pump: sixteen SPI "
                  "interrupts, no channel one, exact",
                  ps == spi_ok && same(tx_buf + 1, rx_buf + 1, 32) && ps_isr == 16u &&
                      ps_dma == 0u);
    Dma1::release();
    dma_host_live = false;

    // ---- the kernel ----
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
    bench.verdict("... every one spi_ok, ISR-pumped and polled interleaved on one bus",
                  kl::Probe::replies[0] == spi_ok && kl::Probe::replies[1] == spi_ok &&
                      kl::Probe::replies[2] == spi_ok && kl::Probe::replies[3] == spi_ok);
    bench.verdict("and the last one's read-back is the pattern, through the arbiter",
                  same(kl::out_a, kl::in_a, 8));

    kl::Probe::clear_tally();
    for (uint8_t i = 0; i < 6u; ++i) {
        post<kl::SpiArb>(kl::request(kl::out_b, nullptr, false));
    }
    kl::pump_until(6, 300);
    print(serial, "  six posted into a four-deep queue: replies ", kl::Probe::n, ", rejected ",
          kl::Probe::rejected, crlf);
    bench.verdict("the arbiter rejects what it cannot queue, immediately",
                  kl::Probe::rejected != 0u);
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
    host_ready();
}

// ===========================================================================
// e - the flags, the vectors, the line modes, the disable rule
// ===========================================================================

volatile uint32_t err_isr_entries = 0;
volatile uint32_t err_isr_flags = 0;
volatile bool err_mode = false;

void te_flags() {
    third_pad_idle();
    Pfic::disable(S1::irq);
    S1::bus_clock(true);
    S1::reset();
    (void)S1::remap(0);

    // ---- the reset values (table 20-1) ----
    const bool reset_ok = S1::regs().CTLR1 == 0u && S1::regs().CTLR2 == 0u &&
                          S1::regs().STATR == 0x0002u && S1::crc_polynomial() == 0x0007u;
    print(serial, "  after the RCC reset pulse: CTLR1=", hex(S1::regs().CTLR1), " CTLR2=",
          hex(S1::regs().CTLR2), " STATR=", hex(S1::status()), " CRCR=",
          hex(S1::crc_polynomial()), crlf);
    bench.verdict("the reset values are the chapter's: TXE up alone in STATR, the polynomial 7",
                  reset_ok);

    // ---- the overrun, and the sequence that clears it ----
    const bool loop = pads_linked<MosiPad, MisoPad>();
    if (loop) {
        (void)S1::configure({.role = SpiRole::host, .clock = SpiClock::div64});
        SckPad::function();
        MosiPad::function();
        MisoPad::input();
        S1::enable();
        S1::flush_rx();
        for (uint8_t i = 0; i < 3u; ++i) {
            S1::data8(static_cast<uint8_t>(0x40u + i));
            uint32_t spins = 400'000u;
            while (!S1::txe() && spins-- != 0u) {
            }
        }
        uint32_t spins = 400'000u;
        while (S1::busy() && spins-- != 0u) {
        }
        const bool ovr = S1::overrun();
        S1::clear_overrun();
        const bool gone = !S1::overrun();
        print(serial, "  three frames clocked with nothing read: OVR ", ovr ? "set" : "CLEAR",
              ", after the DATAR-then-STATR read ", gone ? "clear" : "STILL SET", crlf);
        bench.verdict("THE OVERRUN IS A LEVEL WITH A SEQUENCE: frames arriving on a full buffer "
                      "raise OVR, and a DATAR read followed by a STATR read is what clears it",
                      ovr && gone);
        (void)S1::disable();
    } else {
        print(serial, "  no strap between PA7 and PA6: the overrun wants a frame to arrive", crlf);
    }

    // ---- the CRC error flag: the one rc_w0 bit of STATR ----
    (void)S1::configure({.role = SpiRole::host,
                         .clock = SpiClock::div64,
                         .crc = true,
                         .crc_polynomial = 0x0007u});
    S1::regs().STATR = static_cast<uint16_t>(S1::regs().STATR | spi_crcerr);
    const bool crc_set = S1::crc_error();
    S1::clear_crc_error();
    const bool crc_gone = !S1::crc_error();
    print(serial, "  CRCERR written by hand: ", crc_set ? "it takes a one" : "read-only to a one",
          ", and a zero into its bit ", crc_gone ? "clears it" : "does not", crlf);
    bench.verdict("CRCERR is the one flag of STATR a program writes, and it is cleared by a ZERO "
                  "into its bit and ones everywhere else",
                  crc_gone);

    // ---- the mode fault: SSI dropped under software management ----
    (void)S1::configure({.role = SpiRole::host, .clock = SpiClock::div64,
                         .nss = SpiNss::software});
    S1::enable();
    S1::software_select(true);   // SSI low: "the SSI bit is cleared" of 20.2.7
    (void)delay_us(clock, 5);
    const bool modf = S1::mode_fault();
    const bool demoted = S1::role() == SpiRole::client || !S1::enabled();
    print(serial, "  SSI dropped on a software-managed host: MODF ", modf ? "set" : "clear",
          ", CTLR1=", hex(S1::regs().CTLR1), " STATR=", hex(S1::status()), " (the block ",
          demoted ? "was demoted" : "kept its role", ")", crlf);
    bench.verdict("THE MODE FAULT IS THE DEMOTION: clearing SSI under software management raises "
                  "MODF",
                  modf);
    bench.verdict("... and the silicon takes the host's role away with it, which is why the "
                  "engine's select is a GPIO and SPI1's strapped NSS pad is never handed over",
                  demoted);

    // WHICH SEQUENCE PUTS MODF DOWN. 20.2.7 names two steps - "first
    // perform a read or write operation to STATR, and then write to
    // CTLR1" - and this walks the variants of them in order, reporting
    // the first that works rather than assuming the words.
    uint8_t cleared_by = 0;
    const char* names[] = {"none of them",
                           "STATR read, then CTLR1 written with the value it holds",
                           "STATR read, then the CTLR1 write that RAISES SPE",
                           "STATR read, then CTLR1 written with SSI set",
                           "SSI raised first, then the STATR read and the CTLR1 write",
                           "STATR WRITTEN (not read), then the CTLR1 write",
                           "the RCC reset pulse"};
    for (uint8_t k = 1; k <= 6u && cleared_by == 0u; ++k) {
        if (!S1::mode_fault()) {
            break;
        }
        switch (k) {
            case 1:
                (void)S1::status();
                S1::regs().CTLR1 = S1::regs().CTLR1;
                break;
            // MODF has cleared SPE with MSTR, so the write that puts SPE
            // back IS a CTLR1 write with something in it - which is the
            // one shape the chapter's "then write to CTLR1" does not
            // spell out.
            case 2:
                (void)S1::status();
                S1::regs().CTLR1 = static_cast<uint16_t>(S1::regs().CTLR1 | spi_spe);
                break;
            case 3:
                (void)S1::status();
                S1::regs().CTLR1 = static_cast<uint16_t>(S1::regs().CTLR1 | spi_ssi);
                break;
            case 4:
                S1::software_select(false);
                (void)S1::status();
                S1::regs().CTLR1 = S1::regs().CTLR1;
                break;
            case 5:
                S1::regs().STATR = S1::regs().STATR;
                S1::regs().CTLR1 = S1::regs().CTLR1;
                break;
            default:
                S1::reset();
                break;
        }
        if (!S1::mode_fault()) {
            cleared_by = k;
        }
    }
    print(serial, "  MODF goes down on: ", names[cleared_by], crlf);
    bench.verdict("MODF HAS A WAY DOWN, and the suite names which of six sequences it is - "
                  "20.2.7's own two steps are measured here and not taken on trust",
                  cleared_by != 0u);
    (void)S1::disable();

    // ---- the interrupt: the error line reaches the vector ----
    S1::reset();
    (void)S1::configure({.role = SpiRole::host, .clock = SpiClock::div64,
                         .nss = SpiNss::software});
    err_isr_entries = 0;
    err_isr_flags = 0;
    err_mode = true;
    S1::error_interrupt(true);
    S1::enable();
    Pfic::enable(S1::irq);
    S1::software_select(true);
    (void)delay_us(clock, 200);
    Pfic::disable(S1::irq);
    err_mode = false;
    S1::error_interrupt(false);
    S1::clear_mode_fault();
    print(serial, "  the error line: ", err_isr_entries, " entr", err_isr_entries == 1u ? "y" : "ies",
          " carrying ", hex(err_isr_flags), crlf);
    bench.verdict("SPI1's vector carries the error sources under ERRIE, and the ISR body hands "
                  "back exactly the raised-and-enabled ones",
                  err_isr_entries >= 1u && (err_isr_flags & SpiFlag::mode_fault) != 0u);
    (void)S1::disable();

    // ---- the bidirectional line mode: one pad, both directions ----
    if (loop) {
        S1::reset();
        (void)S1::configure({.role = SpiRole::host,
                             .mode = SpiMode::mode0,
                             .clock = SpiClock::div64,
                             .direction = SpiDirection::half_duplex_out});
        SckPad::function();
        MosiPad::function();
        MisoPad::release();   // the one data line is MOSI here
        S1::enable();
        S1::data8(0xC3);
        uint32_t spins = 400'000u;
        while (S1::busy() && spins-- != 0u) {
        }
        const bool sent = !S1::busy();
        // Turn around: the same pad, listening. The strap holds the level
        // the port puts on PA6, and BIDIMODE's input is MOSI.
        S1::half_duplex_output(false);
        MisoPad::output(true);   // the node held high through the strap
        (void)delay_us(clock, 10);
        S1::flush_rx();
        spins = 400'000u;
        while (!S1::rxne() && spins-- != 0u) {
        }
        const bool got_high = S1::rxne() && S1::data8() == 0xFFu;
        MisoPad::output(false);
        (void)delay_us(clock, 10);
        S1::flush_rx();
        spins = 400'000u;
        while (!S1::rxne() && spins-- != 0u) {
        }
        const bool got_low = S1::rxne() && S1::data8() == 0x00u;
        print(serial, "  bidirectional on ONE pad: the frame went out (", sent ? "BSY fell" : "BSY stood",
              "), and with BIDIOE cleared the same pad read ", got_high ? "all ones" : "not ones",
              " then ", got_low ? "all zeros" : "not zeros", crlf);
        bench.verdict("THE ONE-WIRE LINE MODE: BIDIMODE with BIDIOE drives a frame out of MOSI, "
                      "and clearing BIDIOE turns the same pad into the receiver - the clock runs "
                      "on either side of the turn",
                      sent && got_high && got_low);
        MisoPad::release();
        (void)S1::disable();

        // ---- the receive-only host and how it is stopped ----
        S1::reset();
        (void)S1::configure({.role = SpiRole::host,
                             .mode = SpiMode::mode0,
                             .clock = SpiClock::div128,
                             .direction = SpiDirection::receive_only});
        SckPad::function();
        MosiPad::release();
        MisoPad::output(true);
        S1::enable();
        // The clock free-runs the moment SPE is up: nothing is written.
        uint32_t frames = 0;
        for (uint32_t k = 0; k < 4u; ++k) {
            uint32_t spins = 400'000u;
            while (!S1::rxne() && spins-- != 0u) {
            }
            if (S1::rxne()) {
                (void)S1::data8();
                ++frames;
            }
        }
        const bool busy_stands = S1::busy();
        const auto last = S1::stop_receive_only(SpiDataSize::bits8);
        const bool stopped = !S1::enabled();
        print(serial, "  receive-only host: ", frames,
              " frames arrived with nothing written, BSY ",
              busy_stands ? "STANDS while SPE is up" : "was down",
              ", the stop verb handed back ",
              last ? "the last frame" : "no frame", " and dropped SPE", crlf);
        bench.verdict("A RECEIVE-ONLY HOST CLOCKS ITSELF: with RXONLY set the clock runs as long "
                      "as SPE stands, so frames arrive with nothing written",
                      frames == 4u);
        bench.verdict("... and it is stopped by dropping SPE and reading the last frame out, "
                      "never by waiting for a BSY that the running clock keeps up",
                      stopped);
        MisoPad::release();
        (void)S1::disable();
    } else {
        print(serial, "  the line modes and the receive-only host want the PA7-PA6 strap", crlf);
    }

    // ---- the I2S register where the part has no I2S ----
    S1::reset();
    const bool i2s_bit = S1::i2s_config_writable();
    if constexpr (device::has_i2s) {
        print(serial, "  SPI1's SPI_I2S_CFGR's I2SMOD ", i2s_bit ? "TAKES a write" : "reads back zero",
              "; the I2S face is SPI2's and SPI3's on this part (test_vx03_i2s)", crlf);
    } else {
        print(serial, "  SPI_I2S_CFGR's I2SMOD ", i2s_bit ? "TAKES a write" : "reads back zero",
              " on this part; the datasheet's resource table names no I2S for this series and no "
              "package bonds an audio signal",
              crlf);
    }
    bench.verdict("the I2S face is reported as the silicon answers, not as the four-family "
                  "chapter promises",
                  true);
    S1::bus_clock(false);
    host_ready();
}

// ===========================================================================
// The CH32V303's third instance against its second (letters f..i)
// ===========================================================================
//
// Every name below hangs on the letters' template parameter and every
// body is `if constexpr` on it, so a part without SPI3 forms none of it and
// carries none of it - its state included.

constexpr bool has_spi3 = spi_present(3);

/// The pads of the two ends: SPI2's one column and SPI3's default one,
/// wired pad to pad on the evaluation board.
template <bool on>
struct Link {
    /// The two resources, named through `on` so that nothing of SPI3 is
    /// formed on a part without it.
    using S1 = Spi<on ? 1u : 1u>;
    using S2 = Spi<on ? 2u : 2u>;
    using S3 = Spi<on ? 3u : 3u>;
    static constexpr SpiPins two = spi_pins_for(2, 0);
    static constexpr SpiPins three = spi_pins_for(3, 0);
    /// SPI1's second column: the very pads of SPI3's default one, so the
    /// same four wires reach SPI2 from SPI1 while SPI3 is gated off.
    static constexpr SpiPins one_b = spi_pins_for(1, 1);
    static_assert(one_b.nss == three.nss && one_b.sck == three.sck &&
                      one_b.miso == three.miso && one_b.mosi == three.mosi,
                  "test_vx03_spi: SPI1's second column is SPI3's default one (tables 10-32 and "
                  "10-33) - letter h's second half rides on that");
    using Sel2 = Pin<two.nss.port, two.nss.pin>;
    using Sck2 = Pin<two.sck.port, two.sck.pin>;
    using Miso2 = Pin<two.miso.port, two.miso.pin>;
    using Mosi2 = Pin<two.mosi.port, two.mosi.pin>;
    using Sel3 = Pin<three.nss.port, three.nss.pin>;
    using Sck3 = Pin<three.sck.port, three.sck.pin>;
    using Miso3 = Pin<three.miso.port, three.miso.pin>;
    using Mosi3 = Pin<three.mosi.port, three.mosi.pin>;

    static void all_released() {
        Pfic::disable(S2::irq);
        Pfic::disable(S3::irq);
        S2::bus_clock(true);
        S2::reset();
        S2::bus_clock(false);
        S3::bus_clock(true);
        S3::reset();
        (void)S3::remap(0);
        S3::bus_clock(false);
        Sel2::release();
        Sck2::release();
        Miso2::release();
        Mosi2::release();
        Sel3::release();
        Sck3::release();
        Miso3::release();
        Mosi3::release();
    }

    /// The four wires, each driven from one end and read at the other
    /// against the opposite pull.
    static bool wired() {
        const bool sel = pads_linked<Sel2, Sel3>();
        const bool sck = pads_linked<Sck2, Sck3>();
        const bool miso = pads_linked<Miso2, Miso3>();
        const bool mosi = pads_linked<Mosi2, Mosi3>();
        print(serial, "  the wires PB12-PA15 ", sel ? "in place" : "ABSENT", ", PB13-PB3 ",
              sck ? "in place" : "ABSENT", ", PB14-PB4 ", miso ? "in place" : "ABSENT",
              ", PB15-PB5 ", mosi ? "in place" : "ABSENT", crlf);
        return sel && sck && miso && mosi;
    }
};

// ---------------------------------------------------------------------------
// h and i - blocks through the four channels
// ---------------------------------------------------------------------------

/// A block each way between SPI2 (host) and SPI3 (client), every frame
/// moved by a channel: DMA1's 4 and 5 for SPI2, DMA2's 1 and 2 for SPI3.
/// The channels are polled - no vector - and put away after.
struct BlockResult {
    bool done;
    uint32_t cycles;
    uint16_t host_wrong;
    uint16_t client_wrong;
    /// How the wrong frames are wrong. `host_late`: the host read the
    /// client's frame ONE BIT LATE - shifted one place toward the least
    /// significant end with the previous frame's last bit on top, which
    /// is what a sample taken before the answer's edge reads in MSB-first
    /// order. `client_shifted`: the client took the host's PREVIOUS frame,
    /// a frame lost or gained. The first wrong pair of each side is kept
    /// as printed evidence (expected, then read).
    uint16_t host_late;
    uint16_t client_shifted;
    uint16_t host_first[2];
    uint16_t client_first[2];
};

/// A frame the host sampled one bit early against the client's edge: the
/// client's frame `now` shifted one place right, `before`'s last bit on
/// top (MSB first, the order every block here runs in).
template <typename Elem>
constexpr Elem one_bit_late(Elem now, Elem before) {
    constexpr uint32_t top = 8u * sizeof(Elem) - 1u;
    return static_cast<Elem>((static_cast<uint32_t>(now) >> 1) |
                             ((static_cast<uint32_t>(before) & 1u) << top));
}

/// The four buffers of a block compared, frame by frame, into `res`.
template <typename Elem>
void judge_block(BlockResult& res, const Elem* host_out, const Elem* host_in,
                 const Elem* client_out, const Elem* client_in, uint16_t n) {
    for (uint16_t i = 0; i < n; ++i) {
        if (host_in[i] != client_out[i]) {
            if (res.host_wrong == 0u) {
                res.host_first[0] = client_out[i];
                res.host_first[1] = host_in[i];
            }
            ++res.host_wrong;
            if (i > 0u && host_in[i] == one_bit_late<Elem>(client_out[i], client_out[i - 1u])) {
                ++res.host_late;
            }
        }
        if (client_in[i] != host_out[i]) {
            if (res.client_wrong == 0u) {
                res.client_first[0] = host_out[i];
                res.client_first[1] = client_in[i];
            }
            ++res.client_wrong;
            if (i > 0u && client_in[i] == host_out[i - 1u]) {
                ++res.client_shifted;
            }
        }
    }
}

template <bool on, typename Elem>
struct BlockBuffers {
    static constexpr uint16_t count = 256u / sizeof(Elem);
    static inline Elem host_out[count];
    static inline Elem host_in[count];
    static inline Elem client_out[count];
    static inline Elem client_in[count];
};

template <bool on, typename Elem>
BlockResult block_run(SpiClock rate, bool high_speed) {
    using L = Link<on>;
    using B = BlockBuffers<on, Elem>;
    using H = typename L::S2;
    using C = typename L::S3;
    using HostTx = DmaChannel<H::dma_tx_slot.controller, H::dma_tx_slot.channel>;
    using HostRx = DmaChannel<H::dma_rx_slot.controller, H::dma_rx_slot.channel>;
    using ClientTx = DmaChannel<C::dma_tx_slot.controller, C::dma_tx_slot.channel>;
    using ClientRx = DmaChannel<C::dma_rx_slot.controller, C::dma_rx_slot.channel>;
    constexpr bool wide = sizeof(Elem) == 2u;
    constexpr DmaWidth w = wide ? DmaWidth::half : DmaWidth::byte;
    constexpr SpiDataSize bits = wide ? SpiDataSize::bits16 : SpiDataSize::bits8;
    constexpr uint16_t n = B::count;
    for (uint16_t i = 0; i < n; ++i) {
        B::host_out[i] = static_cast<Elem>(0x5AC3u + i * 0x0101u);
        B::client_out[i] = static_cast<Elem>(0x0F1Eu ^ (i * 0x0302u));
        B::host_in[i] = 0;
        B::client_in[i] = 0;
    }
    const DmaChannelConfig in_cfg{.direction = DmaDirection::peripheral_to_memory,
                                  .peripheral_width = w, .memory_width = w,
                                  .priority = DmaPriority::very_high};
    const DmaChannelConfig out_cfg{.direction = DmaDirection::memory_to_peripheral,
                                   .peripheral_width = w, .memory_width = w,
                                   .priority = DmaPriority::high};

    // The client: SPI3 framed by its NSS pad, both its requests on DMA2.
    C::bus_clock(true);
    C::reset();
    (void)C::remap(0);
    (void)C::configure({.role = SpiRole::client, .bits = bits, .nss = SpiNss::hardware_input});
    L::Sck3::input();
    L::Mosi3::input();
    L::Sel3::input(PinPull::up);
    L::Miso3::function();
    (void)ClientRx::load({.peripheral = C::data_address(), .memory = B::client_in, .count = n,
                          .config = in_cfg});
    (void)ClientTx::load({.peripheral = C::data_address(), .memory = B::client_out, .count = n,
                          .config = out_cfg});
    C::dma_requests(true, true);
    C::enable();

    // The host: SPI2, a GPIO select, the high-speed read where asked.
    H::bus_clock(true);
    H::reset();
    (void)H::configure({.role = SpiRole::host, .clock = rate, .bits = bits,
                        .nss = SpiNss::software});
    if (high_speed) {
        (void)H::high_speed_read(true);
    }
    L::Sel2::output(true);
    L::Sck2::function();
    L::Mosi2::function();
    L::Miso2::input();
    (void)HostRx::load({.peripheral = H::data_address(), .memory = B::host_in, .count = n,
                        .config = in_cfg});
    (void)HostTx::load({.peripheral = H::data_address(), .memory = B::host_out, .count = n,
                        .config = out_cfg});
    L::Sel2::clear();
    (void)delay_us(clock, 2);
    const uint32_t t0 = cycles_now();
    H::enable();
    H::dma_requests(true, true);
    bool done = false;
    for (uint32_t spins = 0; spins < 2'000'000UL; ++spins) {
        if (HostRx::flag(DmaFlag::complete)) {
            done = true;
            break;
        }
    }
    const uint32_t cycles = cycles_now() - t0;
    (void)delay_us(clock, 5);
    L::Sel2::set();

    H::dma_requests(false, false);
    C::dma_requests(false, false);
    HostTx::stop();
    HostRx::stop();
    ClientTx::stop();
    ClientRx::stop();
    (void)H::high_speed_read(false);
    (void)H::disable();
    (void)C::disable();

    BlockResult res{done, cycles, 0, 0, 0, 0, {0, 0}, {0, 0}};
    judge_block<Elem>(res, B::host_out, B::host_in, B::client_out, B::client_in, n);
    L::all_released();
    return res;
}

/// SPI1 on its SECOND column as a POLLED host at `rate` over SPI2 as a
/// client both of whose requests a channel serves (DMA1's 4 and 5): frame
/// by frame, the gap between two frames the polled loop's, so what is
/// measured is the round trip INSIDE a frame at SPI1's own rate - 72 MHz of
/// SCK at /2 on its 144 MHz bus, the rate the high-speed read is for - and
/// not a client's reload. SPI3, whose default pads these are, stays gated
/// off; SPI1's column, clock and mode are put back before the return.
/// `mode2` adds HSRXEN2 on top of HSRXEN, for a die the probe found it on.
template <bool on>
BlockResult spi1_top_run(SpiClock rate, bool high_speed, bool mode2 = false) {
    using L = Link<on>;
    using B = BlockBuffers<on, uint8_t>;
    using H = typename L::S1;
    using C = typename L::S2;
    using ClientTx = DmaChannel<C::dma_tx_slot.controller, C::dma_tx_slot.channel>;
    using ClientRx = DmaChannel<C::dma_rx_slot.controller, C::dma_rx_slot.channel>;
    constexpr uint16_t n = B::count;
    for (uint16_t i = 0; i < n; ++i) {
        B::host_out[i] = static_cast<uint8_t>(0x5Au + i * 0x0Bu);
        B::client_out[i] = static_cast<uint8_t>(0xC3u ^ (i * 0x07u));
        B::host_in[i] = 0;
        B::client_in[i] = 0;
    }
    const DmaChannelConfig in_cfg{.direction = DmaDirection::peripheral_to_memory,
                                  .peripheral_width = DmaWidth::byte,
                                  .memory_width = DmaWidth::byte,
                                  .priority = DmaPriority::very_high};
    const DmaChannelConfig out_cfg{.direction = DmaDirection::memory_to_peripheral,
                                   .peripheral_width = DmaWidth::byte,
                                   .memory_width = DmaWidth::byte,
                                   .priority = DmaPriority::high};

    // The client: SPI2 framed by its NSS pad, its first answer loaded by
    // the channel before the host's first clock.
    C::bus_clock(true);
    C::reset();
    (void)C::configure({.role = SpiRole::client, .nss = SpiNss::hardware_input});
    L::Sck2::input();
    L::Mosi2::input();
    L::Sel2::input(PinPull::up);
    L::Miso2::function();
    (void)ClientRx::load({.peripheral = C::data_address(), .memory = B::client_in, .count = n,
                          .config = in_cfg});
    (void)ClientTx::load({.peripheral = C::data_address(), .memory = B::client_out, .count = n,
                          .config = out_cfg});
    C::dma_requests(true, true);
    C::enable();

    // The host: SPI1 on code 1, a GPIO select on PA15, one frame at a time.
    H::bus_clock(true);
    H::reset();
    bool done = H::remap(1);
    (void)H::configure({.role = SpiRole::host, .clock = rate, .nss = SpiNss::software});
    if (high_speed) {
        done = H::high_speed_read(true) && done;
    }
    if (mode2) {
        done = H::high_speed_read2(true, SysClock::pclk2_hz) && done;
    }
    L::Sel3::output(true);
    L::Sck3::function();
    L::Mosi3::function();
    L::Miso3::input();
    L::Sel3::clear();
    (void)delay_us(clock, 2);
    H::enable();
    for (uint16_t i = 0; done && i < n; ++i) {
        H::data8(B::host_out[i]);
        uint32_t spins = 100'000u;
        while (!H::rxne() && spins != 0u) {
            --spins;
        }
        if (spins == 0u) {
            done = false;
        } else {
            B::host_in[i] = H::data8();
        }
    }
    (void)delay_us(clock, 5);
    L::Sel3::set();

    C::dma_requests(false, false);
    ClientTx::stop();
    ClientRx::stop();
    (void)H::high_speed_read(false);
    (void)H::disable();
    (void)C::disable();
    (void)H::remap(0);
    H::reset();
    H::bus_clock(false);

    BlockResult res{done, 0, 0, 0, 0, 0, {0, 0}, {0, 0}};
    judge_block<uint8_t>(res, B::host_out, B::host_in, B::client_out, B::client_in, n);
    L::all_released();
    return res;
}

/// One block's line: done or stalled, the core cycles where timed, and
/// each side's wrong frames with what kind of wrong they are.
template <bool on>
void print_block(const char* label, const BlockResult& r, bool timed) {
    print(serial, "    ", label, ": ", r.done ? "done" : "STALLED");
    if (timed) {
        print(serial, " in ", r.cycles, " cycles");
    }
    print(serial, ", host ", r.host_wrong, " wrong");
    if (r.host_wrong != 0u) {
        print(serial, " (", r.host_late, " one bit late; first ", hex(r.host_first[0]), " read ",
              hex(r.host_first[1]), ")");
    }
    print(serial, ", client ", r.client_wrong, " wrong");
    if (r.client_wrong != 0u) {
        print(serial, " (", r.client_shifted, " the previous frame; first ",
              hex(r.client_first[0]), " read ", hex(r.client_first[1]), ")");
    }
    print(serial, crlf);
}

template <bool on = has_spi3>
void th_high_speed_read() {
    if constexpr (on) {
        using L = Link<on>;
        L::all_released();
        // The verb's own rule first, with no wire: taken at /2, refused at
        // /4, the code read out of the register it stands in.
        L::S2::bus_clock(true);
        L::S2::reset();
        (void)L::S2::configure({.role = SpiRole::host, .clock = SpiClock::div2});
        const bool at2 = L::S2::high_speed_read(true);
        (void)L::S2::high_speed_read(false);
        (void)L::S2::configure({.role = SpiRole::host, .clock = SpiClock::div4});
        const bool at4 = L::S2::high_speed_read(true);
        L::S2::bus_clock(false);
        bench.verdict("the high-speed read is taken at BR /2 and refused at /4 - the one code "
                      "every lot of every class reads alike",
                      at2 && !at4);
        // HSRXEN2 is a lot's bit and wants a bus of 120 MHz or more, which
        // only SPI1's reaches: the die's answer to the probe, no wire.
        L::S1::bus_clock(true);
        L::S1::reset();
        (void)L::S1::configure({.role = SpiRole::host, .clock = SpiClock::div2});
        const bool mode2 = L::S1::high_speed_read2(true, SysClock::pclk2_hz);
        (void)L::S1::high_speed_read(false);
        L::S1::reset();
        L::S1::bus_clock(false);
        print(serial, "  HSRXEN2 on SPI1, its bus at ", SysClock::pclk2_hz / 1'000'000u, " MHz: ",
              mode2 ? "KEPT - this die's lot has the second high-speed mode"
                    : "not kept - this die's lot has no HSRXEN2",
              crlf);
        bench.verdict("HSRXEN2 is reported as the die answers the probe - 20.4.10 gives it to "
                      "some lots of this class, and no register says which",
                      true);
        if (!L::wired()) {
            bench.verdict("the high-speed read's effect wants the four wires, and says so", true);
            return;
        }
        const uint32_t pclk1 = SysClock::pclk1_hz;
        const BlockResult base = block_run<on, uint8_t>(SpiClock::div4, false);
        const BlockResult plain = block_run<on, uint8_t>(SpiClock::div2, false);
        const BlockResult fast = block_run<on, uint8_t>(SpiClock::div2, true);
        print(serial, "  256 bytes each way, SPI2 host over SPI3 client, every frame a "
                      "channel's; SCK ", pclk1 / 4u / 1000u, " kHz at /4 and ",
              pclk1 / 2u / 1000u, " kHz at /2:", crlf);
        print_block<on>("/4", base, true);
        print_block<on>("/2 without HSRXEN", plain, true);
        print_block<on>("/2 with HSRXEN", fast, true);
        // The block's time against its wire at the rate asked - PB1 over
        // the division, 256 frames - the channels' start inside it.
        const uint32_t pm4 = block_permille(base.cycles, 256, 8, 4);
        const uint32_t pm2 = block_permille(fast.cycles, 256, 8, 2);
        print(serial, "  in thousandths of the wire at the rate asked: /4 ", pm4,
              ", /2 with HSRXEN ", pm2, crlf);
        bench.verdict("and the blocks run at the rate asked: 256 frames at /4 and at /2 in their "
                      "SCK periods at PB1 over the division, -1 % to +3 % on the core's counter",
                      wire_ok(pm4) && wire_ok(pm2));
        bench.verdict("at BR /4 the wires carry the block exactly both ways - the baseline the /2 "
                      "runs are read against",
                      base.done && base.host_wrong == 0u && base.client_wrong == 0u);
        bench.verdict("at /2 - 36 MHz of SCK over the wires - the block with HSRXEN is exact both "
                      "ways; the run without it is printed beside it, its wrong frames sorted by "
                      "kind (a frame read one bit late is a sample taken before the answer's "
                      "edge)",
                      plain.done && fast.done && fast.host_wrong == 0u && fast.client_wrong == 0u);

        // SPI2's /2 is 36 MHz of SCK - PB1 runs at 72 - a rate a strap
        // already carries clean on the CH32V203; the mode is for SPI1's /2,
        // 72 MHz, reached here through SPI1's second column, whose four pads
        // are SPI3's default ones and so ride the same wires to SPI2.
        const uint32_t pclk2 = SysClock::pclk2_hz;
        const BlockResult top_base = spi1_top_run<on>(SpiClock::div8, false);
        const BlockResult top_plain = spi1_top_run<on>(SpiClock::div2, false);
        const BlockResult top_fast = spi1_top_run<on>(SpiClock::div2, true);
        print(serial, "  256 bytes, SPI1 on PA15/PB3/PB4/PB5 polled over SPI2's client; SCK ",
              pclk2 / 8u / 1000u, " kHz at /8 and ", pclk2 / 2u / 1000u, " kHz at /2:", crlf);
        print_block<on>("/8", top_base, false);
        print_block<on>("/2 without HSRXEN", top_plain, false);
        print_block<on>("/2 with HSRXEN", top_fast, false);
        if (mode2) {
            const BlockResult top_two = spi1_top_run<on>(SpiClock::div2, true, true);
            print_block<on>("/2 with HSRXEN and HSRXEN2", top_two, false);
        }
        bench.verdict("SPI1 on its second column carries the block exactly both ways at /8 over "
                      "SPI3's pads - the path the /2 runs are read against",
                      top_base.done && top_base.host_wrong == 0u && top_base.client_wrong == 0u);
        // At 72 MHz of SCK the far end is SPI2 on a 72 MHz bus, taking a
        // clock at its own bus rate: what IT received is printed beside
        // what the host read, and neither count is judged - a far end that
        // does not keep up says nothing about the host's read mode.
        bench.verdict("at SPI1's /2 - 72 MHz of SCK - both runs complete, frame by frame, and "
                      "their counts are printed and not judged: the client's own count says "
                      "whether the far end kept up with a clock at its bus rate",
                      top_plain.done && top_fast.done);
        L::all_released();
    }
}

template <bool on = has_spi3>
void ti_wide_frames() {
    if constexpr (on) {
        using L = Link<on>;
        L::all_released();
        if (!L::wired()) {
            bench.verdict("the 16-bit block wants the four wires, and says so", true);
            return;
        }
        // Five rows: /2 without the high-speed read and with it, then /4,
        // /8 and /16. Each moves the same 256 bytes twice - as 128
        // half-word frames and as 256 byte frames - so the cost per frame
        // and per byte are read against each other at one SCK.
        struct Row {
            SpiClock rate;
            uint8_t divisor;
            bool high_speed;
            const char* name;
        };
        const Row rows[5] = {{SpiClock::div2, 2, false, "/2 without HSRXEN"},
                             {SpiClock::div2, 2, true, "/2 with HSRXEN"},
                             {SpiClock::div4, 4, false, "/4"},
                             {SpiClock::div8, 8, false, "/8"},
                             {SpiClock::div16, 16, false, "/16"}};
        constexpr uint32_t halves = BlockBuffers<on, uint16_t>::count;
        constexpr uint32_t bytes = BlockBuffers<on, uint8_t>::count;
        bool exact = true;
        bool timed = true;
        for (const Row& row : rows) {
            const BlockResult wide = block_run<on, uint16_t>(row.rate, row.high_speed);
            const BlockResult narrow = block_run<on, uint8_t>(row.rate, row.high_speed);
            const uint32_t wire = 16u * row.divisor * (SysClock::hz / SysClock::pclk1_hz);
            // Tenths of a core cycle: the cost per frame is the wire's
            // plus whatever the engines add, and a tenth shows it.
            const uint32_t per_frame = (wide.cycles * 10u) / halves;
            const uint32_t per_byte_wide = (wide.cycles * 10u) / (2u * halves);
            const uint32_t per_byte_narrow = (narrow.cycles * 10u) / bytes;
            print(serial, "  ", row.name, ": a 16-bit frame ", per_frame / 10u, ".",
                  per_frame % 10u, " core cycles against the wire's ", wire, "; a byte ",
                  per_byte_wide / 10u, ".", per_byte_wide % 10u, " in 16-bit frames and ",
                  per_byte_narrow / 10u, ".", per_byte_narrow % 10u, " in 8-bit ones", crlf);
            print_block<on>("128 half-words", wide, true);
            print_block<on>("256 bytes", narrow, true);
            const bool judged = row.divisor != 2u || row.high_speed;
            if (judged && !(wide.done && wide.host_wrong == 0u && wide.client_wrong == 0u &&
                            narrow.done && narrow.host_wrong == 0u && narrow.client_wrong == 0u)) {
                exact = false;
            }
            const uint32_t pm_wide = block_permille(wide.cycles, halves, 16, row.divisor);
            const uint32_t pm_narrow = block_permille(narrow.cycles, bytes, 8, row.divisor);
            print(serial, "    in thousandths of the wire at the rate asked: ", pm_wide,
                  " in 16-bit frames, ", pm_narrow, " in 8-bit ones", crlf);
            if (!wire_ok(pm_wide) || !wire_ok(pm_narrow)) {
                timed = false;
            }
        }
        bench.verdict("half-words through all four channels - DMA1's for SPI2, DMA2's for SPI3, "
                      "every one moving 16-bit items into 16-bit frames - and the same bytes as "
                      "8-bit frames cross exactly both ways at /4, /8 and /16 and at /2 with the "
                      "high-speed read; the costs are printed as measured, the row without the "
                      "mode beside them",
                      exact);
        bench.verdict("and every row runs at the rate asked: both widths' blocks in their SCK "
                      "periods at PB1 over the division, -1 % to +3 % on the core's counter",
                      timed);
        L::all_released();
    }
}

/// The CH32V303's letters, registered where the part has SPI3.
template <bool on = has_spi3>
void register_spi3_letters() {
    if constexpr (on) {
        bench.letter('h', "the high-speed read: /2 with and without HSRXEN, /4 as the baseline",
                     th_high_speed_read<>);
        bench.letter('i', "16-bit frames through the four DMA channels", ti_wide_frames<>);
    }
}

// ===========================================================================
// The pair: d, j, k, l, m - the second set, against the far board's spi_peer
// ===========================================================================
//
// THE OTHER END IS THE DESK PARTNER running `spi_peer` (the CH32V303 for a
// CH32V203 DUT, the CH32V203 for a CH32V303 DUT), commanded IN BAND over
// the bus under test through avrxt/src/apps/spi_link.hpp. Every letter here
// FAILS loudly when the peer does not answer: it is in the second set, which
// `z` never runs.
//
// THE BOUNDS, derived from the two datasheets' SPI tables (the CH32V203's
// table 4-25, the CH32V303's table 4-27) and the two ends' loops:
//  - THE CLIENT HEARS from /4 down: the host's MOSI is valid 5 ns after its
//    edge (tV(MO)) and the client wants 5 ns of setup (tSU(SI)), 10 ns
//    inside the 13.9 ns half period even of /2 = 36 MHz, the client's own
//    ceiling in both tables - but the same tables let SCK rise and fall in
//    20 ns into 30 pF (tr(SCK), tf(SCK)), so a half period holds a whole
//    edge from 25 MHz down: /4 = 18 MHz is judged, /2 printed. The far
//    client's report is judged there wherever its serve keeps up (below).
//  - THE HOST READS BACK where the half period holds the client's answer:
//    the client's MISO is valid 25 ns after its edge (tV(SO)) and the host
//    wants 5 ns (tSU(MI)), 30 ns - inside the 55.6 ns half period of /8 =
//    9 MHz, outside the 27.8 ns of /4 = 18 MHz and the 13.9 ns of /2. So
//    this end's read-back is judged from /8 down; /4, /2 and /2 with
//    HSRXEN (20.4.10's late sampling, whose delay the chapter does not
//    state) are printed.
//  - THE FAR CLIENT'S SERVES: its DMA engines take a beat in some six
//    cycles (docs/ch32vx03/dma.md), two a frame, against the 64 core cycles
//    of an 8-bit frame at /4 - every judged code, its core leaving the bus
//    to them inside the window (spi_peer's header); its POLLED RUN is fifteen
//    instructions a frame with four APB accesses in the release listing
//    (docs/ch32vx03/spi.md), at most 46 cycles counted at two cycles an
//    instruction and six an access, plus the close's second read when the
//    select glitches (64 empty turns and a pad load, about 70 cycles), and a
//    frame must hold two such turns (a look that misses its flag is a whole
//    turn late): 232 cycles - an 8-bit frame from /16 (256), a 16-bit one
//    from /8 (256). Above that the run's rung is printed.

using spilink::Op;

/// The command channel's SCK: /256 of PB1 = 281.25 kHz, a character every
/// 28 us - an order of magnitude more than the peer's polled command
/// listener needs to turn one around.
constexpr SpiClock link_clock = SpiClock::div256;

uint8_t frame_buf[spilink::max_payload + 8];
uint8_t answer_buf[spilink::answer_bytes];
uint8_t raw_seen[16];
uint8_t raw_n = 0;
spilink::Decoder dec;
bool link_quiet = false;

volatile bool pair_pump_live = false;
volatile bool pair_dma_live = false;
volatile bool pair_err_live = false;
volatile uint32_t pair_err_entries = 0;
volatile uint32_t pair_err_flags = 0;

/// The hold around each select window: the engine releases the select
/// about a microsecond after the last edge, and a client whose
/// transaction the select edge frames loses a character it has not
/// fetched yet - so the protocol owns the chip select for these windows
/// (the Request carries a null PinRef) and pays 30 us on each side.
void link_hold() { (void)delay_us(clock, 30); }

/// R6, ONE COLUMN PER INSTANCE AT A TIME: on the CH32V303 the pair's SPI3
/// column 1 shares its remap field with column 0, whose pads carry the
/// evaluation board's jumpers to SPI2 - and are SPI1's second column too.
/// Every pair letter starts with both instances reset and those pads
/// floating; the host's init() then writes code 1.
template <bool on = (pair_spi == 3u)>
void pair_pads_alone() {
    if constexpr (on) {
        Link<on>::all_released();
    }
}

bool link_command_mode() {
    pair_pump_live = false;
    pair_dma_live = false;
    const bool ok = PairHost::init(clock);
    PairNss::output(true);
    dec.reset();
    return ok;
}

/// One protocol window: prime, select, hold, the transaction, hold,
/// deselect. THE MODE IS PRIMED BEFORE THE SELECT FALLS - a CPOL flip
/// inside an open window is one extra edge and the selected client
/// counts it into the frame.
bool link_xfer(const uint8_t* tx, uint8_t* rx, uint16_t n, SpiClock rate = link_clock,
               SpiMode mode = SpiMode::mode0) {
    if (n == 0) {
        return true;
    }
    PairHost::prime(mode, rate);
    PairNss::clear();
    link_hold();
    PairHost::Request r{};
    r.cs = {};
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = n;
    r.clock = rate;
    r.mode = mode;
    r.bits = SpiDataSize::bits8;
    r.polled = true;
    const bool done = PairHost::start(r);
    link_hold();
    PairNss::set();
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

/// One answer window: `answer_bytes` dummies in a single transaction (a
/// null out buffer: the engine's own 0xFF, which is no frame's magic),
/// then the decoder walks what came back.
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
    print(serial, crlf,
          "      the far board must be running `spi_peer`; its console '0' forces the dark client back.", crlf);
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

/// Every pair letter asks first; a peer that does not answer is a FAILED
/// verdict - this is the second set, which needs the far board.
bool need_peer() {
    pair_pads_alone();
    PairHost::pad_speed(pair_pad_class);
    PairDma::pad_speed(pair_pad_class);
    link_quiet = true;
    (void)link_command_mode();
    for (uint8_t k = 0; k < 3; ++k) {
        if (command(Op::ping)) {
            link_quiet = false;
            return true;
        }
    }
    link_quiet = false;
    bench.verdict("THE PEER answers a ping on the pair's four wires (the far board running spi_peer; its "
                  "console '0' forces the dark client back)",
                  false);
    PairHost::release();
    PairNss::release();
    return false;
}

/// Every pair letter ends with its pads floating and its instance off.
void pair_done() {
    pair_pump_live = false;
    pair_dma_live = false;
    PairHost::release();
    PairNss::release();
    pair_pads_alone();
}

// ---- one exchange, commanded and then clocked ------------------------------

/// How this end clocks the exchange: the polled request, the ISR pump, or
/// the DMA engines (a polled request whose data phase rides them).
enum class Shape : uint8_t { polled, pump, engines };

const char* shape_name(Shape s) {
    return s == Shape::polled ? "polled" : (s == Shape::pump ? "pump" : "engines");
}

struct Exchange {
    spilink::Cfg cfg{.apply = 1, .mode = 0, .dord = 0, .regime = spilink::regime_buffer_wait};
    SpiMode host_mode = SpiMode::mode0;
    bool host_lsb = false;
    SpiClock rate = SpiClock::div64;
    uint16_t count = 32;   ///< frames
    uint8_t seed_a = 0x13;
    uint8_t seed_b = 0x57;
    uint8_t pattern = spilink::pattern_prbs;
    uint8_t flags = 0;
    uint8_t spare = 0;
    uint16_t ms = 400;
    Shape shape = Shape::polled;
    bool read = true;        ///< this end reads back (an in buffer)
    bool high_speed = false; ///< HSRXEN under /2
};

struct Outcome {
    bool ran = false;        ///< the peer acked the action and the transaction finished
    bool reported = false;
    uint8_t status = 0;
    uint32_t cycles = 0;     ///< start() to the transaction's end, core cycles
    uint16_t mism = 0;       ///< this end's read-back, bytes wrong
    uint8_t idx = 0xFF;
    uint8_t got = 0;
    uint8_t exp = 0;
    spilink::Report r{};
};

constexpr uint16_t max_exchange = 64;   ///< frames; twice the bytes for 16-bit ones
alignas(4) uint8_t xtx[2u * max_exchange];
alignas(4) uint8_t xrx[2u * max_exchange];

bool wide_of(const Exchange& e) { return (e.spare & spilink::spare_wide) != 0u; }

/// The transaction itself on the shape asked, the select framed by this
/// end, the time from start() to its end on the core's counter.
uint8_t clock_out(const Exchange& e, uint16_t n, uint32_t& cycles) {
    const SpiDataSize bits = wide_of(e) ? SpiDataSize::bits16 : SpiDataSize::bits8;
    uint8_t st = no_answer;
    if (e.shape == Shape::engines) {
        pair_dma_live = true;
        (void)PairDma::init(clock);
        PairNss::output(true);
        (void)PairDma::bit_order(e.host_lsb);
        PairDma::prime(e.host_mode, e.rate, bits);
        if (e.high_speed) {
            (void)PairS::high_speed_read(true);
        }
        PairNss::clear();
        link_hold();
        PairDma::Request r{};
        r.cs = {};
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(xtx));
        r.rx = lend<Lease::reply>(e.read ? xrx : nullptr);
        r.len = n;
        r.clock = e.rate;
        r.mode = e.host_mode;
        r.bits = bits;
        r.polled = true;
        host_done = false;
        const uint32_t t0 = cycles_now();
        (void)PairDma::start(r);
        cycles = cycles_now() - t0;
        st = PairDma::status();
        link_hold();
        PairNss::set();
        (void)PairS::high_speed_read(false);
        PairDma::release();
        pair_dma_live = false;
        return st;
    }
    (void)PairHost::bit_order(e.host_lsb);
    PairHost::prime(e.host_mode, e.rate, bits);
    if (e.high_speed) {
        (void)PairS::high_speed_read(true);
    }
    PairNss::clear();
    link_hold();
    PairHost::Request r{};
    r.cs = {};
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(xtx));
    r.rx = lend<Lease::reply>(e.read ? xrx : nullptr);
    r.len = n;
    r.clock = e.rate;
    r.mode = e.host_mode;
    r.bits = bits;
    r.polled = e.shape == Shape::polled;
    host_done = false;
    pair_pump_live = e.shape == Shape::pump;
    const uint32_t t0 = cycles_now();
    if (PairHost::start(r)) {
        cycles = cycles_now() - t0;
        st = PairHost::status();
    } else {
        for (uint32_t i = 0; i < 4'000'000UL && !host_done; ++i) {
        }
        cycles = cycles_now() - t0;
        st = host_done ? PairHost::status() : no_answer;
        if (!host_done) {
            (void)PairHost::recover();
        }
    }
    pair_pump_live = false;
    link_hold();
    PairNss::set();
    (void)PairS::high_speed_read(false);
    (void)PairHost::bit_order(false);
    return st;
}

/// Command the exchange, clock it, read the peer's report, judge this end's
/// read-back against the far stream in spi_link.hpp's first row (a
/// preloaded client: rx[i] = P_B(i)).
Outcome run_exchange(const Exchange& e) {
    Outcome o{};
    const bool wide = wide_of(e);
    spilink::Params a{};
    a.cfg = e.cfg;
    a.count = e.count;
    a.ms = e.ms;
    a.seed_a = e.seed_a;
    a.seed_b = e.seed_b;
    a.pattern = e.pattern;
    a.flags = e.flags;
    a.spare = e.spare;
    // A bit-order mismatch is not a shrug: told about it, each end checks
    // the EXACT bit-reverse of what the other sent.
    if (e.host_lsb != (e.cfg.dord != 0)) {
        a.flags |= spilink::flag_expect_reversed;
    }
    if (!peer_act(Op::exchange, a)) {
        return o;
    }
    const uint16_t n = e.count < max_exchange ? e.count : max_exchange;
    const uint16_t bytes = static_cast<uint16_t>(wide ? 2u * n : n);
    {
        spilink::Stream out(e.pattern, e.seed_a);
        for (uint16_t i = 0; i < sizeof xtx; ++i) {
            xtx[i] = 0;
            xrx[i] = 0xEE;
        }
        for (uint16_t i = 0; i < bytes; ++i) {
            xtx[i] = out.next();
        }
    }
    settle();
    o.status = clock_out(e, n, o.cycles);
    o.ran = o.status == spi_ok;
    (void)link_command_mode();
    o.reported = peer_report(o.r);
    const bool reversed = e.host_lsb != (e.cfg.dord != 0);
    if (e.read) {
        for (uint16_t i = 0; i < bytes; ++i) {
            const uint8_t want = spilink::wide_byte(e.pattern, e.seed_b, i, wide, reversed);
            if (xrx[i] != want) {
                if (o.mism == 0u) {
                    o.idx = static_cast<uint8_t>(i);
                    o.got = xrx[i];
                    o.exp = want;
                }
                ++o.mism;
            }
        }
    }
    return o;
}

/// The far client heard every frame exact.
bool heard(const Exchange& e, const Outcome& o) {
    return o.ran && o.reported && o.r.count == e.count && o.r.mism == 0u &&
           (o.r.flags & spilink::report_ran) != 0u;
}
/// ... and this end read the far stream back exact.
bool exact(const Exchange& e, const Outcome& o) { return heard(e, o) && (!e.read || o.mism == 0u); }

void dump(const Outcome& o) {
    print(serial, "      read:");
    for (uint8_t i = 0; i < 16u; ++i) {
        print(serial, " ", hex(xrx[i]));
    }
    print(serial, crlf, "      sent:");
    for (uint8_t i = 0; i < 16u; ++i) {
        print(serial, " ", hex(xtx[i]));
    }
    print(serial, crlf);
    print(serial, "      status ", o.status, ", here mism=", o.mism, " (first idx ", o.idx, " got ", hex(o.got),
          " exp ", hex(o.exp), "), the far client count=", o.r.count, " mism=", o.r.mism, " (idx ", o.r.idx,
          " got ", hex(o.r.got), " exp ", hex(o.r.exp), ") flags=", hex(o.r.flags), " aux=", hex(o.r.aux0), " ",
          o.r.aux1, " ", hex(o.r.aux2), crlf);
}

/// The wire's time for `frames` frames of `bits` at the code, in core
/// cycles: PB1's share of HCLK is two at 144 MHz.
uint32_t wire_cycles(SpiClock rate, uint16_t frames, uint8_t bits) {
    return static_cast<uint32_t>(static_cast<uint64_t>(frames) * bits * spi_division(rate) * SysClock::hz /
                                 SysClock::pclk1_hz);
}

/// Where the far client's hearing is judged (the section's bounds): from /4.
bool heard_judged(SpiClock rate) { return static_cast<uint8_t>(rate) >= static_cast<uint8_t>(SpiClock::div4); }
/// Where the host's read-back is judged (the section's bounds): from /8.
bool read_judged(SpiClock rate) { return static_cast<uint8_t>(rate) >= static_cast<uint8_t>(SpiClock::div8); }
/// Where the far client's POLLED run is judged: an 8-bit frame from /16, a
/// 16-bit one from /8.
bool run_judged(SpiClock rate, bool wide) {
    return static_cast<uint8_t>(rate) >= static_cast<uint8_t>(wide ? SpiClock::div8 : SpiClock::div16);
}

// ===========================================================================
// d - the peer: the command channel, the matrix, the dummy, the loss, the select
// ===========================================================================

void td_peer() {
    if (!need_peer()) {
        return;
    }
    bench.verdict("the peer answers a ping over the pair's four wires", true);

    spilink::Ident d{};
    const bool got = peer_ident(d);
    char label[9] = {};
    for (uint8_t i = 0; i < 8; ++i) {
        label[i] = d.label[i];
    }
    if (got) {
        print(serial, "  peer: label '", label, "' xtal=", d.xtal, " sanity=", hex(d.sanity), " fw=", hex(d.version),
              crlf);
    }
    bench.verdict("ident comes back and it IS spi_peer of this stratum (the sanity byte, kind 0x05), on its "
                  "crystal - the far part of the desk pair",
                  got && d.sanity == spilink::ident_sanity && spilink::peer_kind(d.version) == spilink::peer_ch32vx03 &&
                      d.xtal == 1u && d.label[0] == 'V');

    uint8_t pings = 0;
    for (uint8_t i = 0; i < 10; ++i) {
        if (command(Op::ping)) {
            ++pings;
        }
    }
    print(serial, "  ", pings, " of 10 pings answered at ", PairHost::sck_hz(link_clock) / 1000u,
          " kHz SCK, one frame per chip-select window", crlf);
    bench.verdict("the command channel is steady over ten frames", pings == 10u);

    // ---- the four modes and both bit orders, at each of this end's pad classes ----
    //
    // A data line's edge coupled into the clock is one sampling edge too
    // many on a bus of loose wires, so the matrix is a measurement of the
    // PADS as much as of the block: it is walked at each slew class of this
    // end's SCK and MOSI, and judged at the class every pair letter runs at
    // (pair_pad_class, its reason beside it). THE BOUND IS ZERO: at /64,
    // 1.125 MHz, the half period is 444 ns, and the far client's answer
    // (valid 25 ns after its edge, tV(SO)), this end's setup (5 ns,
    // tSU(MI)) and a medium-class edge (25 ns into 50 pF) take 55 ns of it,
    // so one frame wrong in any mode, either way, is a defect and not the
    // timing's margin. The far client answers in every mode at that class
    // too (spi_peer's boot choice).
    constexpr PinSpeed speeds[] = {PinSpeed::fast, PinSpeed::medium, PinSpeed::slow};
    const char* speed_names[] = {"fast (50 MHz)", "medium (10 MHz)", "slow (2 MHz)"};
    uint8_t good_at_class = 0;
    for (uint8_t si = 0; si < 3u; ++si) {
        PairHost::pad_speed(speeds[si]);
        PairDma::pad_speed(speeds[si]);
        (void)link_command_mode();
        uint8_t good = 0;
        for (uint8_t m = 0; m < 4; ++m) {
            for (uint8_t lsb = 0; lsb < 2; ++lsb) {
                Exchange e{};
                e.cfg.mode = m;
                e.cfg.dord = lsb;
                e.host_mode = static_cast<SpiMode>(m);
                e.host_lsb = lsb != 0u;
                e.seed_a = static_cast<uint8_t>(0x13u + 2u * m + lsb);
                e.seed_b = static_cast<uint8_t>(0x57u + 2u * m + lsb);
                const Outcome o = run_exchange(e);
                if (exact(e, o)) {
                    ++good;
                } else {
                    print(serial, "    ", speed_names[si], " mode ", m, lsb ? " LSb" : " MSb", " first:", crlf);
                    dump(o);
                }
            }
        }
        print(serial, "  SCK and MOSI at ", speed_names[si], ": ", good,
              " of 8 (four modes x two bit orders) exact both ways at ", PairHost::sck_hz(SpiClock::div64) / 1000u,
              " kHz, the far client on its engines", crlf);
        if (speeds[si] == pair_pad_class) {
            good_at_class = good;
        }
    }
    PairHost::pad_speed(pair_pad_class);
    PairDma::pad_speed(pair_pad_class);
    (void)link_command_mode();
    bench.verdict("ALL FOUR TRANSFER MODES AND BOTH BIT ORDERS carry 32 frames exact both ways between the two "
                  "chips, the far client ANSWERING in every one, at the pair's pad class",
                  good_at_class == 8u);

    Exchange mism{};
    mism.cfg.dord = 1;      // the far client LSb first
    mism.host_lsb = false;  // this end MSb first
    const Outcome mo = run_exchange(mism);
    print(serial, "  bit-order mismatch: here mism=", mo.mism, ", the far client mism=", mo.r.mism, " of ", mo.r.count,
          " (both checking the exact bit-reverse)", crlf);
    bench.verdict("a bit-order mismatch is an EXACT TWO-WAY BIT REVERSAL - each end reads the other's frames "
                  "with their bits in the opposite order",
                  exact(mism, mo));

    // ---- THE DUMMY, measured and not assumed ----
    //
    // Told not to preload (regime_normal), the far client starts its run
    // once the first frame has begun - its first answer written inside
    // that frame's first bit: the peer's turn from the SCK edge to the
    // store is some tens of cycles, against the 256 cycles of /256's half
    // period. What goes out is this silicon's answer to "what does a
    // client too slow for the first frame send?".
    {
        Exchange e{};
        e.cfg.regime = spilink::regime_normal;   // the far client does NOT preload
        e.rate = link_clock;
        e.count = 16;
        e.seed_a = 0x2B;
        e.seed_b = 0x6D;
        const Outcome o = run_exchange(e);
        uint16_t aligned = 0;   // later frames that are the far stream in place
        uint16_t late = 0;      // ... and one frame late
        for (uint16_t i = 1; i < e.count; ++i) {
            if (xrx[i] == spilink::pattern_value(e.pattern, e.seed_b, i)) {
                ++aligned;
            }
            if (xrx[i] == spilink::pattern_value(e.pattern, e.seed_b, static_cast<uint16_t>(i - 1u))) {
                ++late;
            }
        }
        const uint8_t first = spilink::pattern_value(e.pattern, e.seed_b, 0);
        const bool bit_late = (xrx[0] & 0x7Fu) == (first >> 1);
        print(serial, "    read:");
        for (uint8_t i = 0; i < 8u; ++i) {
            print(serial, " ", hex(xrx[i]));
        }
        print(serial, crlf, "    want:");
        for (uint8_t i = 0; i < 8u; ++i) {
            print(serial, " ", hex(spilink::pattern_value(e.pattern, e.seed_b, i)));
        }
        print(serial, crlf);
        print(serial, "  with NO preload the first frame read back is ", hex(xrx[0]), " (the stream starts ", hex(first),
              ": its top seven bits one place down, the shifter's leftover above them - ",
              bit_late ? "yes" : "NO", "); of the ", e.count - 1u, " later frames ", aligned, " are the stream in place, ",
              late, " one place late", crlf);
        uint16_t bit_behind = 0;   // later frames one BIT late: the frame before's last bit on top
        for (uint16_t i = 1; i < e.count; ++i) {
            const uint8_t now = spilink::pattern_value(e.pattern, e.seed_b, i);
            const uint8_t before = spilink::pattern_value(e.pattern, e.seed_b, static_cast<uint16_t>(i - 1u));
            if (xrx[i] == static_cast<uint8_t>((now >> 1) | ((before & 1u) << 7))) {
                ++bit_behind;
            }
        }
        print(serial, "    ", bit_behind, " of the later frames one BIT late; the far client heard ", o.r.count,
              " mism ", o.r.mism, crlf);
        // THE CLIENT RESTARTS ITS FRAME AT THE LATE LOAD (both classes, the
        // same frames from either peer): from there it is one bit behind the
        // host both ways - every later answer one bit late, and the window's
        // last frame never completes (n - 1 heard).
        const bool far_frame_restarted = o.ran && o.reported && bit_behind == e.count - 1u &&
                                         o.r.count == e.count - 1u;
        bench.verdict("THE DUMMY IS MEASURED: a client whose first answer comes inside the first frame has it in "
                      "the SHIFTER AT ONCE, at the next bit - the first frame the leftover's top bit over the "
                      "answer's seven - and RESTARTS its frame there: one bit behind the host both ways, every "
                      "later answer one bit late and the last frame unfinished",
                      bit_late && far_frame_restarted);
    }

    // ---- sink_slow: what a client that never drains does ----
    {
        spilink::Params a{};
        a.cfg = spilink::Cfg{.apply = 1, .mode = 0, .dord = 0, .regime = spilink::regime_normal};
        a.count = 32;
        a.ms = 120;
        if (peer_act(Op::sink_slow, a)) {
            settle_ms(10);
            fill_pattern(xtx, max_exchange, 0x91);
            (void)link_xfer(xtx, xrx, 16);
            spilink::Report sr{};
            const bool rp = peer_report(sr);
            print(serial, "  sink_slow: the far client retained ", sr.count, " frame(s), its status during the flood ",
                  hex(sr.sum), ", after the drain ", hex(sr.exp), crlf);
            bench.verdict("A CLIENT THAT NEVER DRAINS keeps ONE frame and raises the overrun - no FIFO behind it "
                          "(figure 20-1), seen on the far chip",
                          rp && sr.count == 1u && (sr.flags & spilink::report_bufovf) != 0u);
        } else {
            bench.verdict("the peer accepted sink_slow", false);
        }
    }

    // ---- ss_pulse: the select seen from this end ----
    {
        spilink::Params a{};
        a.aux8 = 60;      // the lead-in, long enough for this end to let go
        a.aux16 = 4000;   // microseconds the peer holds the wire down
        // THE COMMAND GOES OUT FIRST, and only then does this end let the
        // wire go: the command travels over the bus this host owns.
        if (peer_act(Op::ss_pulse, a)) {
            PairHost::release();
            PairNss::input(PinPull::up);
            bool saw_low = false;
            const uint32_t t0 = Ticker::millis();
            while (Ticker::millis() - t0 < 300u) {
                if (!PairNss::read()) {
                    saw_low = true;
                    break;
                }
            }
            print(serial, "  ss_pulse: the shared select ", saw_low ? "went LOW" : "stayed high",
                  " while the peer drove it", crlf);
            bench.verdict("THE SELECT IS A REAL WIRE: the peer pulling it low is seen at this end", saw_low);
        } else {
            bench.verdict("the peer accepted ss_pulse", false);
        }
        settle_ms(20);
        (void)link_command_mode();
    }

    // ---- ten seconds of traffic, counted at both ends ----
    {
        uint32_t rounds = 0, bad = 0, frames = 0;
        const uint32_t t0 = Ticker::millis();
        while (Ticker::millis() - t0 < 10'000u) {
            Exchange e{};
            e.count = 32;
            e.rate = SpiClock::div16;
            e.shape = (rounds & 1u) != 0u ? Shape::engines : Shape::polled;
            e.seed_a = static_cast<uint8_t>(0x11u + rounds);
            e.seed_b = static_cast<uint8_t>(0x77u + rounds);
            if (!exact(e, run_exchange(e))) {
                ++bad;
            }
            frames += e.count;
            ++rounds;
        }
        print(serial, "  ten seconds: ", rounds, " exchanges at ", PairHost::sck_hz(SpiClock::div16) / 1000u,
              " kHz (polled and engined in turn), ", frames, " frames, ", bad, " not exact at one end or the other",
              crlf);
        bench.verdict("A TEN-SECOND STRESS between the two chips: every exchange exact at both ends",
                      rounds > 0u && bad == 0u);
    }

    (void)link_command_mode();
    bench.verdict("the command channel is back after all of that", command(Op::ping));
    pair_done();
}

// ===========================================================================
// j - the ladder: every code, every host shape, both of the far serves
// ===========================================================================

void tj_ladder() {
    if (!need_peer()) {
        return;
    }
    print(serial, "  the far client's hearing is judged from /4, this end's read-back from /8, the far polled "
                  "run from /16 (the section's bounds); cycles are this end's start() to the end, the wire's "
                  "beside them",
          crlf);
    bool heard_all = true;
    bool read_all = true;
    bool run_all = true;
    for (uint8_t code = 8; code-- > 0;) {
        const SpiClock rate = static_cast<SpiClock>(code);
        print(serial, "  /", spi_division(rate), " = ", PairHost::sck_hz(rate) / 1000u, " kHz, 64 frames, the wire ",
              wire_cycles(rate, 64, 8), " cycles:", crlf);
        for (uint8_t k = 0; k < 4u; ++k) {
            Exchange e{};
            e.rate = rate;
            e.count = 64;
            e.seed_a = static_cast<uint8_t>(0x21u + code + 16u * k);
            e.seed_b = static_cast<uint8_t>(0x84u + code + 16u * k);
            // k: the polled host, the pump, the engines - against the far
            // engines - and the polled host against the far POLLED run.
            e.shape = k == 1u ? Shape::pump : (k == 2u ? Shape::engines : Shape::polled);
            const bool far_run = k == 3u;
            if (far_run) {
                e.spare = spilink::spare_polled_pump;
            }
            const Outcome o = run_exchange(e);
            const bool h = heard(e, o);
            const bool x = exact(e, o);
            print(serial, "    ", shape_name(e.shape), far_run ? " / far run    " : " / far engines", ": ",
                  x ? "exact both ways" : (h ? "heard, read back WRONG" : "NOT heard"), ", ", o.cycles, " cycles (",
                  o.cycles / 64u, " a frame)", crlf);
            if (!x) {
                dump(o);
            }
            if (far_run) {
                if (run_judged(rate, false)) {
                    run_all = run_all && h && (!read_judged(rate) || x);
                }
            } else {
                if (heard_judged(rate)) {
                    heard_all = heard_all && h;
                }
                if (read_judged(rate)) {
                    read_all = read_all && x;
                }
            }
        }
        if (rate == SpiClock::div2) {
            for (uint8_t k = 0; k < 2u; ++k) {
                Exchange e{};
                e.rate = rate;
                e.count = 64;
                e.high_speed = true;
                e.shape = k == 0u ? Shape::polled : Shape::engines;
                e.seed_a = 0x5D;
                e.seed_b = 0xC3;
                const Outcome o = run_exchange(e);
                print(serial, "    ", shape_name(e.shape), " with HSRXEN / far engines: ",
                      exact(e, o) ? "exact both ways" : (heard(e, o) ? "heard, read back WRONG" : "NOT heard"), ", ",
                      o.cycles, " cycles (printed, not judged)", crlf);
                if (!exact(e, o)) {
                    dump(o);
                }
            }
        }
    }
    bench.verdict("THE FAR CLIENT HEARS EVERY CODE from /256 to /4, from every host shape of this end - the "
                  "polled request, the pump and the engines - on its DMA serve",
                  heard_all);
    bench.verdict("THIS END READS THE FAR ANSWERS BACK EXACT from /8 down on every shape, the datasheets' answer "
                  "timing inside the half period",
                  read_all);
    bench.verdict("THE FAR CLIENT'S POLLED RUN keeps up from /16 down behind the polled host (its turn twice "
                  "inside a frame)",
                  run_all);
    (void)link_command_mode();
    bench.verdict("the command channel is back after the ladder", command(Op::ping));
    pair_done();
}

// ===========================================================================
// k - the extended exchange: 16-bit frames, the CRC, the receive-only window
// ===========================================================================

/// The chapter's CRC over FRAMES, bitwise: each frame's value MSb first
/// from its top bit, unreflected, from zero - `bytes` low byte first for
/// 16-bit frames (spi_link.hpp's frame i = P(2i) | P(2i + 1) << 8).
uint16_t crc_frames(uint16_t poly, bool wide, const uint8_t* bytes, uint16_t frames) {
    const uint8_t width = wide ? 16u : 8u;
    uint32_t crc = 0;
    const uint32_t top = 1UL << (width - 1u);
    const uint32_t mask = (1UL << width) - 1UL;
    for (uint16_t k = 0; k < frames; ++k) {
        const uint32_t v = wide ? (static_cast<uint32_t>(bytes[2u * k]) | (static_cast<uint32_t>(bytes[2u * k + 1u]) << 8))
                                : bytes[k];
        for (uint8_t b = width; b-- > 0;) {
            const bool in = ((v >> b) & 1u) != 0u;
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
    bool frames_ok = false;   ///< every frame came back within its budget
    bool stream_ok = false;   ///< and they were the peer's stream
    uint16_t tx_crc = 0, rx_crc = 0, frame = 0;
    uint16_t tx_ref = 0, rx_ref = 0, peer_ref = 0;
    bool crcerr = false;
    spilink::Report peer{};
    bool reported = false;
};

/// Eight frames to the peer with both CRC units on: this end by hand on
/// the resource (CRCNEXT after the last data frame, the CRC frame read
/// back), the peer's CRC run with `peer_poly`.
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
    // The resource by hand: a host in mode 0 at /64 with the CRC on.
    PairHost::release();
    PairS::bus_clock(true);
    PairS::reset();
    (void)PairS::remap(pair_code);
    (void)PairS::configure({.role = SpiRole::host, .mode = SpiMode::mode0, .clock = SpiClock::div64,
                            .bits = bits, .crc = true, .crc_polynomial = poly});
    PairSck::function();
    PairMosi::function();
    PairMiso::input();
    PairS::enable();
    PairS::flush_rx();
    PairNss::output(true);
    (void)delay_us(clock, 20);
    PairNss::clear();
    link_hold();
    // Each frame written, spun on RXNE, read back, a gap behind it for the
    // client's run; CRCNEXT right after the last data frame is written
    // (20.2.5); then the CRC frame itself comes back.
    bool frames = true;
    for (uint16_t i = 0; i < n; ++i) {
        const uint16_t v = wide ? static_cast<uint16_t>(sent[2u * i] | (sent[2u * i + 1u] << 8)) : sent[i];
        PairS::data(bits, v);
        if (i == n - 1u) {
            PairS::crc_next();
        }
        uint32_t spins = 200'000u;
        while (!PairS::rxne() && spins-- != 0u) {
        }
        frames = frames && spins != 0u;
        const uint16_t r = PairS::data(bits);
        got[w * i] = static_cast<uint8_t>(r);
        if (wide) {
            got[2u * i + 1u] = static_cast<uint8_t>(r >> 8);
        }
        (void)delay_us(clock, spilink::gap_us);
    }
    uint32_t spins = 200'000u;
    while (!PairS::rxne() && spins-- != 0u) {
    }
    frames = frames && spins != 0u;
    c.frame = PairS::data(bits);
    spins = 200'000u;
    while (PairS::busy() && spins-- != 0u) {
    }
    link_hold();
    PairNss::set();
    c.tx_crc = PairS::tx_crc();
    c.rx_crc = PairS::rx_crc();
    c.crcerr = PairS::crc_error();
    (void)PairS::disable();
    PairS::reset();
    c.frames_ok = frames;
    uint8_t want[2 * n];
    for (uint16_t i = 0; i < n * w; ++i) {
        want[i] = spilink::pattern_value(a.pattern, a.seed_b, i);
    }
    c.stream_ok = true;
    for (uint16_t i = 0; i < n * w; ++i) {
        c.stream_ok = c.stream_ok && got[i] == want[i];
    }
    c.tx_ref = crc_frames(poly, wide, sent, n);
    c.rx_ref = crc_frames(poly, wide, got, n);
    c.peer_ref = crc_frames(peer_poly, wide, want, n);
    c.reported = peer_report(c.peer);
    return c;
}

void print_crc(const char* label, const CrcRun& c) {
    print(serial, "  ", label, ": TCRCR ", hex(c.tx_crc), " (bitwise ", hex(c.tx_ref), "), RCRCR ", hex(c.rx_crc),
          " (bitwise over what came ", hex(c.rx_ref), "), the CRC frame read ", hex(c.frame), " (the peer's, bitwise ",
          hex(c.peer_ref), "), CRCERR here ", c.crcerr ? "SET" : "clear", "; the peer heard ", c.peer.count, " mism ",
          c.peer.mism, " flags ", hex(c.peer.flags), " (CRCERR there ",
          (c.peer.flags & spilink::report_crcerr) != 0u ? "SET" : "clear", ", its RCRCR low byte ", hex(c.peer.aux3),
          ")", crlf);
}

void tk_extended() {
    if (!need_peer()) {
        return;
    }
    // ---- 16-bit frames, every shape, both of the far serves ----
    {
        uint8_t ok = 0;
        uint8_t ext = 0;
        constexpr uint8_t runs = 8;
        for (uint8_t k = 0; k < runs; ++k) {
            Exchange e{};
            e.spare = spilink::spare_wide;
            e.count = 32;
            e.rate = (k & 1u) != 0u ? SpiClock::div8 : SpiClock::div32;
            e.shape = k < 2u ? Shape::polled : (k < 4u ? Shape::pump : (k < 6u ? Shape::engines : Shape::polled));
            if (k >= 6u) {
                // the far POLLED run, at its 16-bit bound and below (the
                // section's bounds)
                e.spare = static_cast<uint8_t>(e.spare | spilink::spare_polled_pump);
                e.rate = (k & 1u) != 0u ? SpiClock::div8 : SpiClock::div32;
            }
            e.cfg.mode = (k & 2u) != 0u ? 3u : 0u;
            e.host_mode = static_cast<SpiMode>(e.cfg.mode);
            e.seed_a = static_cast<uint8_t>(0x31u + k);
            e.seed_b = static_cast<uint8_t>(0xA7u + k);
            const Outcome o = run_exchange(e);
            if ((o.r.flags & spilink::report_extended) != 0u) {
                ++ext;
            }
            if (exact(e, o)) {
                ++ok;
            } else {
                print(serial, "    16-bit ", shape_name(e.shape), (k >= 6u) ? " / far run" : " / far engines", " /",
                      spi_division(e.rate), " mode ", e.cfg.mode, ":", crlf);
                dump(o);
            }
        }
        print(serial, "  16-bit frames: ", ok, " of ", runs,
              " exchanges of 32 frames exact both ways (polled, pump, engines against the far engines; the polled "
              "host against the far run), report_extended in ",
              ext, crlf);
        bench.verdict("16-BIT FRAMES cross exact both ways on every shape of this end and both far serves, and the "
                      "peer says it served the extended exchange",
                      ok == runs && ext == runs);
    }

    // ---- the CRC, against a foreign unit ----
    {
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
        bench.verdict("THE HARDWARE CRC IS THE ARITHMETIC at both widths: TCRCR over what went, RCRCR over what came",
                      c8.tx_crc == c8.tx_ref && c8.rx_crc == c8.rx_ref && c16.tx_crc == c16.tx_ref &&
                          c16.rx_crc == c16.rx_ref);
        bench.verdict("A FOREIGN CRC UNIT AGREES: the CRC frame read is the peer's bitwise CRC, CRCERR down here",
                      c8.frame == c8.peer_ref && !c8.crcerr && c16.frame == c16.peer_ref && !c16.crcerr);
        bench.verdict("and the peer checked THIS end's CRC frame and found it right (no CRCERR there)",
                      c8.reported && c16.reported && (c8.peer.flags & spilink::report_crcerr) == 0u &&
                          (c16.peer.flags & spilink::report_crcerr) == 0u);
        // THE CHECK IS REAL: the two units on different polynomials - each
        // end's CRC frame is then wrong for the other, and both must say so.
        const CrcRun bad = crc_exchange(false, 0x0007u, 0x0031u, 0x44, 0x9A);
        print_crc("CRC-8 here 0x07, the peer 0x31", bad);
        bench.verdict("a POLYNOMIAL MISMATCH is caught at BOTH ends: CRCERR set here and in the peer's report",
                      bad.accepted && bad.reported && bad.crcerr && (bad.peer.flags & spilink::report_crcerr) != 0u);
        (void)link_command_mode();
    }

    // ---- the receive-only window: this end's WRITE witnessed ----
    {
        uint8_t ok = 0;
        constexpr uint8_t runs = 6;
        for (uint8_t k = 0; k < runs; ++k) {
            Exchange e{};
            e.spare = static_cast<uint8_t>(spilink::spare_receive_only | ((k & 1u) != 0u ? spilink::spare_wide : 0u));
            e.count = 64;
            e.read = false;   // a WRITE: no in buffer, two frames in flight at every code
            e.rate = k < 2u ? SpiClock::div16 : (k < 4u ? SpiClock::div4 : SpiClock::div2);
            e.shape = (k & 2u) != 0u ? Shape::engines : Shape::polled;
            e.seed_a = static_cast<uint8_t>(0x61u + k);
            const Outcome o = run_exchange(e);
            const uint8_t w = (k & 1u) != 0u ? 16u : 8u;
            print(serial, "    receive-only, this end a ", shape_name(e.shape), " write of 64 ", w, "-bit frames at /",
                  spi_division(e.rate), ": the peer heard ", o.r.count, " mism ", o.r.mism, ", ", o.cycles,
                  " cycles against the wire's ", wire_cycles(e.rate, 64, w), crlf);
            if (heard(e, o) && (o.r.flags & spilink::report_extended) != 0u) {
                ++ok;
            } else {
                dump(o);
            }
        }
        bench.verdict("THE WRITE SHAPE IS WITNESSED: a client that only listens (receive-only, MISO dark) hears this "
                      "end's two-in-flight writes exact, polled and engined, both widths, to /2",
                      ok == runs);
    }
    (void)link_command_mode();
    bench.verdict("the command channel is back after the extended exchange", command(Op::ping));
    pair_done();
}

// ===========================================================================
// l - the roles invert: this chip's client runs under the far host
// ===========================================================================

alignas(4) uint8_t c_out[2u * max_exchange];
alignas(4) uint8_t c_in[2u * max_exchange];

struct ClientRun {
    bool accepted = false;
    PairClient::Run run{};
    uint16_t mism = 0;
    spilink::Report r{};
    bool reported = false;
    bool released = false;
};

/// This end as the CLIENT: the peer becomes the host after its lead-in and
/// clocks `count` frames of seed_a, one polled frame a request gap_us
/// apart, under a select it frames from its own NSS pad; this end answers
/// seed_b through SpiClient::exchange() - one frame ahead, preloaded with
/// the select high - and judges what it heard.
ClientRun as_client(SpiMode mode, bool lsb, bool wide, PinSpeed miso, uint8_t seed_a, uint8_t seed_b,
                    bool listen = false) {
    ClientRun c{};
    constexpr uint16_t n = 16;
    spilink::Params a{};
    a.cfg = spilink::Cfg{.apply = 1, .mode = static_cast<uint8_t>(mode), .dord = static_cast<uint8_t>(lsb ? 1u : 0u),
                         .regime = spilink::regime_buffer_wait};
    a.count = n;
    a.ms = 400;
    a.seed_a = seed_a;
    a.seed_b = seed_b;
    a.pattern = spilink::pattern_prbs;
    a.aux8 = 40;    // the lead-in: this end becomes a client inside it
    a.aux16 = 64;   // the instrument's SCK: at most HCLK/64 on its bus
    a.spare = wide ? spilink::spare_wide : uint8_t{0};
    c.accepted = peer_act(Op::host_burst, a);
    if (!c.accepted) {
        return c;
    }
    const uint8_t w = wide ? 2u : 1u;
    {
        spilink::Stream out(a.pattern, a.seed_b);
        for (uint16_t i = 0; i < n * w; ++i) {
            c_out[i] = out.next();
            c_in[i] = 0xEE;
        }
    }
    PairHost::release();
    PairNss::release();
    PairClient::pad_speed(miso);
    const bool up = PairClient::init(clock, {.mode = mode,
                                             .bits = wide ? SpiDataSize::bits16 : SpiDataSize::bits8,
                                             .lsb_first = lsb,
                                             .nss = SpiNss::hardware_input,
                                             .drive_output = !listen});
    Pfic::disable(PairS::irq);   // polled: the run is the whole of it
    if (up && listen) {
        PairClient::enable();
        const uint32_t spins = 900u * (SysClock::hz / 30'000u);
        c.run = PairClient::receive(std::span<uint8_t>(c_in, n), spins);
    } else if (up) {
        PairClient::enable();
        const uint32_t spins = 900u * (SysClock::hz / 30'000u);   // the lead-in and the burst, with room
        if (wide) {
            c.run = PairClient::exchange(std::span<const uint16_t>(reinterpret_cast<const uint16_t*>(c_out), n),
                                         std::span<uint16_t>(reinterpret_cast<uint16_t*>(c_in), n), spins);
        } else {
            c.run = PairClient::exchange(std::span<const uint8_t>(c_out, n), std::span<uint8_t>(c_in, n), spins);
        }
    }
    // The dark listener's half: the answer line handed back to the bus.
    const uint32_t t0 = Ticker::millis();
    while (PairClient::selected() && Ticker::millis() - t0 < 100u) {
    }
    PairClient::drive_output(false);
    c.released = !PairClient::output_driven();
    PairClient::release();
    PairClient::pad_speed(PinSpeed::fast);
    {
        const bool reversed = false;
        for (uint16_t i = 0; i < n * w; ++i) {
            if (c_in[i] != spilink::wide_byte(a.pattern, a.seed_a, i, wide, reversed)) {
                ++c.mism;
            }
        }
    }
    settle_ms(20);
    c.reported = peer_report(c.r);
    return c;
}

bool client_exact(const ClientRun& c) {
    return c.accepted && c.run.count == 16u && c.mism == 0u && c.reported && c.r.count == 16u && c.r.mism == 0u &&
           c.released;
}
/// A listening client: what it heard alone is judged.
bool client_heard(const ClientRun& c) { return c.accepted && c.run.count == 16u && c.mism == 0u; }

void tl_client() {
    if (!need_peer()) {
        return;
    }
    // ---- the answer line's slew class, at mode 0 ----
    // Walked at all three, judged at the pair's (pair_pad_class, its reason
    // beside it); the bound is letter d's: at the instrument's /64 the
    // answer's timing takes 55 ns of a 444 ns half period, so one frame
    // wrong is a defect.
    constexpr PinSpeed speeds[] = {PinSpeed::fast, PinSpeed::medium, PinSpeed::slow};
    const char* names[] = {"fast (50 MHz)", "medium (10 MHz)", "slow (2 MHz)"};
    bool class_ok = false;
    for (uint8_t si = 0; si < 3u; ++si) {
        uint8_t good = 0;
        for (uint8_t k = 0; k < 3u; ++k) {
            const ClientRun c = as_client(SpiMode::mode0, false, false, speeds[si], static_cast<uint8_t>(0x3Bu + k),
                                          static_cast<uint8_t>(0x6Du + k));
            if (client_exact(c)) {
                ++good;
            }
        }
        print(serial, "  this end's MISO at ", names[si], ": ", good, " of 3 bursts exact both ways", crlf);
        if (speeds[si] == pair_pad_class) {
            class_ok = good == 3u;
        }
    }
    bench.verdict("THIS CLIENT'S MISO AT THE PAIR'S PAD CLASS carries every burst exact both ways - the class the "
                  "letters below and the far peer use (the other two printed above)",
                  class_ok);

    // ---- every mode, both orders, and 16-bit frames ----
    uint8_t good = 0;
    for (uint8_t m = 0; m < 4; ++m) {
        for (uint8_t lsb = 0; lsb < 2; ++lsb) {
            const ClientRun c = as_client(static_cast<SpiMode>(m), lsb != 0u, false, pair_pad_class,
                                          static_cast<uint8_t>(0x45u + 2u * m + lsb), static_cast<uint8_t>(0x9Au + 2u * m + lsb));
            if (client_exact(c)) {
                ++good;
            } else {
                print(serial, "    mode ", m, lsb ? " LSb" : " MSb", ": the run took ", c.run.count, " (end ",
                      static_cast<uint8_t>(c.run.end), "), here mism=", c.mism, ", the far host read ", c.r.count,
                      " mism ", c.r.mism, crlf);
            }
        }
    }
    print(serial, "  this chip as the CLIENT: ", good, " of 8 (four modes x two orders) bursts exact both ways", crlf);
    bench.verdict("THE ROLES INVERT: this chip's SpiClient answers ONE FRAME AHEAD under the far chip's host in all "
                  "four modes and both bit orders, both ends agreeing frame for frame, and hands its MISO back after",
                  good == 8u);
    // The receive-only run, its MISO never driven: what it hears alone.
    uint8_t heard = 0;
    for (uint8_t m = 0; m < 4; ++m) {
        const ClientRun c = as_client(static_cast<SpiMode>(m), false, false, pair_pad_class,
                                      static_cast<uint8_t>(0x51u + m), static_cast<uint8_t>(0xA1u + m), true);
        if (client_heard(c)) {
            ++heard;
        }
    }
    bench.verdict("... and LISTENING (receive(), its MISO dark) this client hears all four modes exact",
                  heard == 4u);
    uint8_t wide_good = 0;
    for (uint8_t m = 0; m < 4; m += 3) {
        const ClientRun c = as_client(static_cast<SpiMode>(m), false, true, pair_pad_class, static_cast<uint8_t>(0x71u + m),
                                      static_cast<uint8_t>(0x1Fu + m));
        if (client_exact(c)) {
            ++wide_good;
        } else {
            print(serial, "    16-bit mode ", m, ": the run took ", c.run.count, " (end ", static_cast<uint8_t>(c.run.end),
                  "), here mism=", c.mism, ", the far host read ", c.r.count, " mism ", c.r.mism, " flags ",
                  hex(c.r.flags), crlf);
        }
    }
    bench.verdict("... and in 16-bit frames (modes 0 and 3), the run over half-words", wide_good == 2u);
    (void)link_command_mode();
    bench.verdict("the command channel is back after the roles inverted", command(Op::ping));
    pair_done();
}

// ===========================================================================
// m - the hardware select: MODF from a second chip, and SSOE framing a client
// ===========================================================================

void tm_select() {
    if (!need_peer()) {
        return;
    }
    // ---- MODF: a host whose NSS pad is the multi-host INPUT ----
    {
        spilink::Params a{};
        a.aux8 = 60;      // the lead-in
        a.aux16 = 2000;   // the far chip holds the select low this long
        const bool acked = peer_act(Op::ss_pulse, a);
        PairHost::release();
        PairS::bus_clock(true);
        PairS::reset();
        (void)PairS::remap(pair_code);
        (void)PairS::configure({.role = SpiRole::host, .clock = SpiClock::div64, .nss = SpiNss::hardware_input});
        PairSck::function();
        PairMosi::function();
        PairMiso::input();
        PairNss::input(PinPull::up);   // table 10-5: the NSS input pulled or floating
        pair_err_entries = 0;
        pair_err_flags = 0;
        pair_err_live = true;
        PairS::error_interrupt(true);
        Pfic::enable(PairS::irq);
        PairS::enable();
        const bool host_before = PairS::role() == SpiRole::host && PairS::enabled() && !PairS::mode_fault();
        const uint32_t t0 = Ticker::millis();
        uint32_t at = 0;
        while (Ticker::millis() - t0 < 300u) {
            if (PairS::mode_fault()) {
                at = Ticker::millis() - t0;
                break;
            }
        }
        const bool modf = PairS::mode_fault();
        const uint16_t ctlr1 = PairS::regs().CTLR1;
        const uint16_t statr = PairS::status();
        settle_ms(10);   // the pulse is over and the pad high again
        const bool chapter_clears = PairS::clear_mode_fault();
        PairS::error_interrupt(false);
        pair_err_live = false;
        Pfic::disable(PairS::irq);
        PairS::reset();
        const bool reset_clears = !PairS::mode_fault();
        print(serial, "  MODF from the far chip's ss_pulse: ", modf ? "SET" : "not seen", " ", at,
              " ms after this end's enable (the peer's lead-in ", a.aux8, " ms), CTLR1=", hex(ctlr1), " STATR=",
              hex(statr), ", the error vector entered ", pair_err_entries, " time(s) with ", hex(pair_err_flags),
              "; 20.2.7's sequence ", chapter_clears ? "cleared it" : "LEFT IT STANDING", ", the reset pulse ",
              reset_clears ? "cleared it" : "did not", crlf);
        bench.verdict("A SECOND HOST ON THE HARDWARE NSS INPUT: the far chip's low select raises MODF on this "
                      "host",
                      acked && host_before && modf);
        bench.verdict("... it is the DEMOTION - MSTR and SPE gone with it - and it reaches the instance's vector "
                      "under ERRIE",
                      (ctlr1 & spi_mstr) == 0u && (ctlr1 & spi_spe) == 0u && pair_err_entries >= 1u &&
                          (pair_err_flags & SpiFlag::mode_fault) != 0u);
        bench.verdict("... and the block's RCC reset pulse puts it down (the chapter's sequence is printed, the "
                      "finding of letter e)",
                      reset_clears);
        PairSck::release();
        PairMosi::release();
        PairNss::release();
        settle_ms(20);
        (void)link_command_mode();
    }

    // ---- SSOE: the host's own select pad framing the far client ----
    {
        constexpr uint16_t n = 16;
        spilink::Params a{};
        a.cfg = spilink::Cfg{.apply = 1, .mode = 0, .dord = 0, .regime = spilink::regime_buffer_wait};
        a.count = n;
        a.ms = 400;
        a.seed_a = 0x4E;
        a.seed_b = 0xB1;
        a.pattern = spilink::pattern_prbs;
        const bool acked = peer_act(Op::exchange, a);
        {
            spilink::Stream out(a.pattern, a.seed_a);
            for (uint16_t i = 0; i < n; ++i) {
                xtx[i] = out.next();
                xrx[i] = 0xEE;
            }
        }
        settle();
        PairHost::release();
        PairNss::release();
        PairS::bus_clock(true);
        PairS::reset();
        (void)PairS::remap(pair_code);
        (void)PairS::configure({.role = SpiRole::host, .clock = SpiClock::div64, .nss = SpiNss::hardware_output});
        PairSck::function();
        PairMosi::function();
        PairMiso::input();
        PairNss::function();   // SSOE drives it while SPE stands
        const bool high_before = PairNss::read();
        PairS::enable();
        (void)delay_us(clock, 30);
        const bool low_enabled = !PairNss::read();
        bool frames = true;
        for (uint16_t i = 0; i < n; ++i) {
            PairS::data8(xtx[i]);
            uint32_t spins = 200'000u;
            while (!PairS::rxne() && spins-- != 0u) {
            }
            frames = frames && spins != 0u;
            xrx[i] = PairS::data8();
            (void)delay_us(clock, spilink::gap_us);
        }
        const bool low_between = !PairNss::read();
        (void)PairS::disable();
        (void)delay_us(clock, 30);
        const bool high_after = PairNss::read();
        PairS::reset();
        PairSck::release();
        PairMosi::release();
        PairNss::release();
        uint16_t mism = 0;
        for (uint16_t i = 0; i < n; ++i) {
            if (xrx[i] != spilink::pattern_value(a.pattern, a.seed_b, i)) {
                ++mism;
            }
        }
        spilink::Report r{};
        settle_ms(20);
        const bool rp = peer_report(r);
        print(serial, "  SSOE: the select pad ", high_before ? "high" : "LOW", " before SPE, ", low_enabled ? "LOW" : "high",
              " with SPE up, ", low_between ? "LOW" : "high", " between frames, ", high_after ? "high" : "LOW",
              " after SPE came down; this end read ", n - mism, " of ", n, " exact, the far client heard ", r.count,
              " mism ", r.mism, crlf);
        bench.verdict("SSOE IS THE HOST'S OWN SELECT: the pad high under SPE until the first frame, low from it to "
                      "SPE down, and the far client it frames carries the exchange exact both ways",
                      acked && frames && high_before && !low_enabled && low_between && high_after && mism == 0u &&
                          rp && r.count == n && r.mism == 0u);
        settle_ms(20);
        (void)link_command_mode();
    }
    bench.verdict("the command channel is back after the select's letters", command(Op::ping));
    pair_done();
}

// ===========================================================================
// f - a polled wait that runs out, and the transmit interrupt (z, no wire)
// ===========================================================================

/// Letter f's transmit stream, fed from SPI1's vector one frame a TXE.
volatile bool txe_mode = false;
volatile uint16_t txe_next = 0;
volatile uint32_t txe_entries = 0;
constexpr uint16_t txe_frames = 64;

/// THE POLLED WAIT'S LOOP, counted in the release listing (poll_write's
/// wait for TXE: the STATR load, the bit test, the budget's decrement and
/// its test, the branch back - five instructions, one an APB load): a spin
/// costs at least its five cycles and, the load waiting on PB2, no more
/// than four times that; the tick's 1 kHz handler adds well under one per
/// cent. So a budget of S spins ends between 5 S and 20 S cycles.
constexpr uint32_t spin_min_cycles = 5;
constexpr uint32_t spin_max_cycles = 20;

void tf_stalled_txe() {
    third_pad_idle();
    // ---- spi_stalled: the polled wait that runs out ----
    // The engine up and primed for the request's own word - so start()
    // writes nothing on the way in - and THEN SPI1's gate closed under it:
    // every STATR load reads zero, so TXE never comes.
    constexpr uint16_t frames = 16;
    const bool up = Host1::init(clock);
    Host1::prime(SpiMode::mode0, SpiClock::div64);
    fill_pattern(tx_buf, frames, 0x31);
    Host1::Request r{};
    r.cs = {};
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
    r.len = frames;
    r.mode = SpiMode::mode0;
    r.clock = SpiClock::div64;
    r.polled = true;
    S1::bus_clock(false);
    const uint32_t t0 = cycles_now();
    const bool sync = Host1::start(r);
    const uint32_t wall = cycles_now() - t0;
    const uint8_t st = Host1::status();
    S1::bus_clock(true);
    const uint32_t spins = 100'000u + 2'048u * frames;   // SpiHost's polled_budget(16)
    // The gate open again: the same request runs to its end.
    const bool sync2 = Host1::start(r);
    const uint8_t st2 = Host1::status();
    print(serial, "  a polled write of ", frames, " frames with SPI1's gate closed: ",
          sync ? "returned" : "DID NOT RETURN", " with status ", st, " (spi_stalled is ", spi_stalled, ") after ",
          wall, " cycles - ", wall / spins, " a spin over the budget's ", spins, "; the gate open, the same request ",
          st2 == spi_ok ? "spi_ok" : "FAILED", crlf);
    bench.verdict("A POLLED WAIT THAT RUNS OUT ends the transaction spi_stalled within its one spin budget - "
                  "between 5 and 20 cycles a spin, the loop's own five instructions",
                  up && sync && st == spi_stalled && wall >= spin_min_cycles * spins &&
                      wall <= spin_max_cycles * spins);
    bench.verdict("... and the engine is left where start() is legal: with the gate open the same request "
                  "runs spi_ok",
                  sync2 && st2 == spi_ok);
    Host1::release();

    // ---- TXEIE: a transmit-only stream fed by the transmit interrupt ----
    // SPI1 bare, no pad handed over, its CRC unit on: the vector writes the
    // next frame each time TXE rises and silences TXEIE after the last, so
    // the stream is judged by TCRCR - the CRC of every frame the shift
    // register sent - against a bitwise loop over the buffer, and timed:
    // at /64 a frame is 512 core cycles and a handler a small fraction of
    // it, so the frames go out back to back.
    S1::bus_clock(true);
    S1::reset();
    (void)S1::configure({.role = SpiRole::host, .clock = SpiClock::div64, .crc = true, .crc_polynomial = 0x07u});
    fill_pattern(tx_buf, txe_frames, 0x5C);
    txe_next = 0;
    txe_entries = 0;
    txe_mode = true;
    S1::enable();
    Pfic::clear_pending(S1::irq);
    Pfic::enable(S1::irq);
    const uint32_t t1 = cycles_now();
    S1::txe_interrupt(true);
    for (uint32_t i = 0; i < 4'000'000UL && txe_mode; ++i) {
    }
    for (uint32_t i = 0; i < 400'000UL && S1::busy(); ++i) {
    }
    const uint32_t took = cycles_now() - t1;
    const bool finished = !txe_mode;
    Pfic::disable(S1::irq);
    txe_mode = false;
    const uint16_t got = S1::tx_crc();
    const uint16_t want = crc_frames(0x07u, false, tx_buf, txe_frames);
    (void)S1::disable();
    S1::reset();
    S1::bus_clock(false);
    // 64 frames of 512 cycles back to back, plus the first entry's
    // latency and the last frame's own shift after the vector let go: the
    // span lies between the frames' time and that plus one frame and the
    // handlers' slack (2000 cycles: two tick entries and the vector's
    // sixty-five).
    const uint32_t frame_cycles = 8u * 64u * (SysClock::hz / SysClock::pclk2_hz);
    const uint32_t wire = txe_frames * frame_cycles;
    print(serial, "  TXEIE on a host: ", txe_entries, " entries for ", txe_frames, " frames, TCRCR ", hex(got),
          " against ", hex(want), ", ", took, " cycles against ", wire, " of wire", crlf);
    bench.verdict("THE TRANSMIT INTERRUPT ON A HOST: one entry a frame and one more to silence it, every frame "
                  "out in order (the CRC unit's sum the buffer's)",
                  finished && txe_entries == txe_frames + 1u && got == want);
    bench.verdict("... and the stream back to back: the span within one frame and 2000 cycles of the frames' "
                  "own time",
                  took >= wire && took <= wire + frame_cycles + 2000u);
    host_ready();
}

// ===========================================================================
// n - recover() with the far chip holding the bus (y)
// ===========================================================================

volatile bool pair_bus_live = false;

namespace kt {

/// The per-bus timeout: twice the longest transaction this letter asks
/// for (the 2000 dummies below at /256 of PB1, 57 ms) - so only a
/// transaction that will never end runs into it.
constexpr uint32_t timeout_ms = 120;
using PairArb = SpiBus<PairHost, P, 2, BusPassThrough, ticks_from_ms<P>(timeout_ms)>;

class Probe {
public:
    using Event = std::variant<SpiDone>;
    static inline EventQueue<Event, 4, P> queue;
    static inline uint8_t n = 0;
    static inline uint8_t last = 0;
    static inline uint32_t at_ms = 0;

    static void init() { n = 0; }
    static void dispatch(const Event& e) {
        brio::match(e, [](const SpiDone& d) {
            last = d.status;
            at_ms = Ticker::millis();
            ++n;
        });
    }
};

using BusKernel = Tenuto<P, Probe, PairArb>;

void pump() {
    TimeEvents<P>::process();
    while (BusKernel::step()) {
        TimeEvents<P>::process();
    }
}

/// `len` frames of dummies at /256 with nothing read back - a write, and
/// no select of its own: the select wire is the far chip's in this letter.
PairHost::Request dummies(uint16_t len) {
    PairHost::Request r{};
    r.cs = {};
    r.len = len;
    r.mode = SpiMode::mode0;
    r.clock = SpiClock::div256;
    r.polled = false;
    r.reply = reply_to<Probe, SpiDone>();
    return r;
}

}  // namespace kt

void tn_recover() {
    if (!need_peer()) {
        return;
    }
    // THE STAGING: this host's NSS pad made the multi-host INPUT (SSM
    // cleared under the engine, the request's own word primed first so
    // start() rewrites nothing) and the far chip's ss_pulse taking that
    // wire low 30 ms after its ack - in the middle of a pumped write of
    // 2000 frames (57 ms). MODF takes SPE and MSTR away, the pump's RXNE
    // never comes again, and no completion is posted: the arbiter's
    // timeout is what answers, and recover() what brings the engine back.
    spilink::Params a{};
    a.aux8 = 30;
    a.aux16 = 2000;
    const bool acked = peer_act(Op::ss_pulse, a);
    (void)PairHost::init(clock);
    PairHost::prime(SpiMode::mode0, SpiClock::div256);
    PairNss::input(PinPull::up);
    (void)PairS::disable();
    PairS::regs().CTLR1 = static_cast<uint16_t>(PairS::regs().CTLR1 & ~spi_ssm);
    PairS::enable();
    kt::BusKernel::init_all();
    pair_bus_live = true;
    const uint32_t t0 = Ticker::millis();
    post<kt::PairArb>(kt::dummies(2000));
    uint32_t modf_at = 0;
    while (Ticker::millis() - t0 < 600u && kt::Probe::n == 0u) {
        kt::pump();
        if (modf_at == 0u && PairS::mode_fault()) {
            modf_at = Ticker::millis() - t0;
        }
    }
    const uint8_t first = kt::Probe::last;
    const uint32_t answered = kt::Probe::at_ms - t0;
    const bool back = PairS::role() == SpiRole::host && PairS::enabled() && !PairS::mode_fault() &&
                      (PairS::regs().CTLR1 & spi_ssm) != 0u;
    // The same arbiter carries the next transaction to its end.
    kt::Probe::n = 0;
    post<kt::PairArb>(kt::dummies(16));
    const uint32_t t1 = Ticker::millis();
    while (Ticker::millis() - t1 < 200u && kt::Probe::n == 0u) {
        kt::pump();
    }
    const uint8_t second = kt::Probe::n != 0u ? kt::Probe::last : no_answer;
    pair_bus_live = false;
    print(serial, "  a pumped write of 2000 frames, the far chip's select pulse 30 ms into it: MODF ",
          modf_at != 0u ? "seen " : "NOT SEEN", modf_at, " ms in; the arbiter answered ", first,
          " (spi_timeout is ", spi_timeout, ") ", answered, " ms after the post; the block afterwards ",
          back ? "a host again, enabled, software-selected" : "NOT BACK", "; the next request ", second, crlf);
    PairNss::release();
    settle_ms(20);
    (void)link_command_mode();
    // The bound: the arbiter's timeout counts from the post, so the answer
    // comes at 120 ms of ticks plus one tick of its arming and the pump's
    // turn - never before 120, and well before 200.
    bench.verdict("A HOST DEMOTED MID-TRANSACTION BY A SECOND DEVICE ON ITS SELECT is answered spi_timeout by "
                  "the timed arbiter, 120 to 200 ms after the post, MODF seen near the far chip's 30 ms",
                  acked && modf_at >= 20u && modf_at <= 60u && first == spi_timeout && answered >= kt::timeout_ms &&
                      answered < 200u);
    bench.verdict("... and recover() puts the engine back where start() is legal - the block a software-selected "
                  "host again, MODF down - and the same arbiter carries the next transaction spi_ok",
                  back && second == spi_ok);
    bench.verdict("the command channel is back after the staged demotion", command(Op::ping));
    pair_done();
}

// ===========================================================================
// o - rebase(): the engine under a DynamicClock (y)
// ===========================================================================

/// The suite's own rate first - so the pack starts where main() left the
/// machine - then two lower PLL rates on the same HSI. The console and the
/// tick are users so the letter speaks at every rate, and the pair's host
/// is the one this letter rebases.
using Dyn = DynamicClock<Rates<Clock<ClockSource::pll, 144'000'000>, Clock<ClockSource::pll, 96'000'000>,
                               Clock<ClockSource::pll, 48'000'000>>,
                         Ticker, Serial, PairHost>;

/// The ceiling this letter declares: 6 MHz, which /8 honours on a 48 MHz
/// bus and /16 on a 72 MHz one.
constexpr uint32_t rebase_ceiling = 6'000'000;

void to_rebase() {
    if (!need_peer()) {
        return;
    }
    bool books = true;
    bool wires = true;
    for (uint8_t i = 0; i < Dyn::rate_count; ++i) {
        // The ceiling stands from an init() at the rate in force; then the
        // switch, whose fan-out is the rebase() this letter is about.
        console_drain();
        (void)PairHost::init(Dyn{}, rebase_ceiling);
        const bool switched = Dyn::set_index(i);
        const uint32_t hz = Dyn::rate_hz(i);
        const uint32_t bus = Dyn::rate_pclk1_hz(i);
        // What rebase() must have computed, from first principles.
        uint8_t want_code = 0;
        while (want_code < 7u && bus / (2UL << want_code) > rebase_ceiling) {
            ++want_code;
        }
        const uint8_t ratio = static_cast<uint8_t>(hz / bus);
        const auto ceiling = PairHost::ceiling_clock();
        const auto ahead8 = PairHost::write_ahead_from(SpiDataSize::bits8);
        const auto ahead16 = PairHost::write_ahead_from(SpiDataSize::bits16);
        const bool book = switched && PairHost::reference_hz() == bus && PairHost::hclk_hz() == hz && ceiling &&
                          static_cast<uint8_t>(*ceiling) == want_code &&
                          PairHost::sck_hz(SpiClock::div2) == bus / (2UL << want_code) && ahead8 && ahead16 &&
                          static_cast<uint8_t>(*ahead8) == spi_write_ahead_code<spi_default_hold_off_cycles>(8, ratio) &&
                          static_cast<uint8_t>(*ahead16) == spi_write_ahead_code<spi_default_hold_off_cycles>(16, ratio);
        books = books && book;
        const uint32_t engine_bus = PairHost::reference_hz();
        const uint32_t div2_hz = PairHost::sck_hz(SpiClock::div2);
        // A request at /2 against the far client: clamped to the ceiling,
        // never faster than it on the core's counter (each of its 256
        // bits at least HCLK / 6 MHz cycles long), every frame exact both
        // ways.
        Exchange e{};
        e.rate = SpiClock::div2;
        e.count = 32;
        const Outcome o = run_exchange(e);
        const uint32_t floor_cycles = static_cast<uint32_t>(256ULL * hz / rebase_ceiling);
        const bool wire = exact(e, o) && o.cycles >= floor_cycles;
        wires = wires && wire;
        console_drain();
        print(serial, "  ", hz / 1'000'000u, " MHz (bus ", bus / 1'000'000u, "): the engine's bus ",
              engine_bus / 1'000'000u, " MHz, the ceiling /", ceiling ? spi_division(*ceiling) : 0u,
              " (code ", want_code, " asked), /2 runs at ", div2_hz / 1000u,
              " kHz, two in flight from /", ahead8 ? spi_division(*ahead8) : 0u, " and /",
              ahead16 ? spi_division(*ahead16) : 0u, "; 32 frames at /2: ", o.cycles, " cycles (", floor_cycles,
              " at least), ", exact(e, o) ? "exact both ways" : "NOT EXACT", crlf);
    }
    console_drain();
    (void)Dyn::set_index(0);
    (void)link_command_mode();
    bench.verdict("REBASE() FOLLOWS THE CLOCK: at 144, 96 and 48 MHz the engine's bus rate, the declared ceiling's "
                  "code, the rate /2 really runs at and both write-ahead codes are what the new bus makes them",
                  books);
    bench.verdict("... and at each rate a request at /2 runs clamped to the ceiling - never faster than it on the "
                  "core's counter - exact both ways against the far client",
                  wires);
    bench.verdict("the suite's own rate is in force again, the command channel answering", command(Op::ping));
    pair_done();
}

// ===========================================================================
// x - the slip statistics (by name): each mode against both far serves
// ===========================================================================

/// Each transfer mode four times on each far serve - its engines and its
/// polled run - behind the polled host at /64 and behind the pump, the
/// count of exchanges not exact printed per cell. No verdict: the
/// instrument behind letter d's matrix.
void tx_slips() {
    if (!need_peer()) {
        return;
    }
    PairHost::pad_speed(pair_pad_class);
    constexpr SpiClock rates[] = {SpiClock::div256, SpiClock::div64, SpiClock::div8};
    for (uint8_t ri = 0; ri < 3u; ++ri)
    for (uint8_t m = 0; m < 4; ++m) {
        print(serial, "  /", spi_division(rates[ri]), " mode ", m, ":");
        for (uint8_t cell = 0; cell < 5u; ++cell) {
            uint8_t bad = 0;
            uint16_t worst_count = 0xFFFF;
            for (uint8_t k = 0; k < 4u; ++k) {
                Exchange e{};
                e.cfg.mode = m;
                e.host_mode = static_cast<SpiMode>(m);
                e.rate = rates[ri];
                e.shape = (cell & 1u) != 0u ? Shape::pump : Shape::polled;
                if ((cell & 2u) != 0u) {
                    e.spare = spilink::spare_polled_pump;
                }
                if (cell == 4u) {
                    // the far client LISTENING, its MISO dark
                    e.spare = spilink::spare_receive_only;
                    e.read = false;
                    e.shape = Shape::polled;
                }
                e.seed_a = static_cast<uint8_t>(0x21u + 16u * m + 4u * cell + k);
                e.seed_b = static_cast<uint8_t>(0x93u + 16u * m + 4u * cell + k);
                const Outcome o = run_exchange(e);
                if (!exact(e, o)) {
                    ++bad;
                    if (o.r.count < worst_count) {
                        worst_count = o.r.count;
                    }
                }
            }
            print(serial, "  ", cell == 4u ? "write/far listening " : ((cell & 1u) != 0u ? "pump" : "polled"),
                  cell == 4u ? "" : ((cell & 2u) != 0u ? "/far run " : "/far dma "), bad, " of 4 bad");
            if (bad != 0u) {
                print(serial, " (the far client heard as few as ", worst_count, ")");
            }
        }
        print(serial, crlf);
    }
    pair_done();
}

/// The pair's letters, the second set, registered where the part links.
template <bool on = has_pair>
void register_pair_letters() {
    if constexpr (on) {
        bench.second_letter('d', "THE PEER: the command channel, the matrix, the dummy, the loss, the select, a "
                                 "stress",
                            td_peer);
        bench.second_letter('j', "THE LADDER: every code, every host shape, the far engines and the far run",
                            tj_ladder);
        bench.second_letter('k', "THE EXTENDED EXCHANGE: 16-bit frames, the CRC, the receive-only window",
                            tk_extended);
        bench.second_letter('l', "THE ROLES INVERT: this chip's client runs under the far host", tl_client);
        bench.second_letter('m', "THE HARDWARE SELECT: MODF from a second chip, SSOE framing the far client",
                            tm_select);
        bench.second_letter('n', "RECOVER(): a host demoted mid-transaction by the far chip, the timed arbiter",
                            tn_recover);
        bench.second_letter('o', "REBASE(): the engine at three rates of a DynamicClock against the far client",
                            to_rebase);
        bench.letter('x', "the slip statistics against the peer: each mode, both far serves (no verdict)", tx_slips,
                     false);
    }
}

// ===========================================================================
// The menu
// ===========================================================================

void banner() {
    print(serial, crlf, "test_vx03_spi - the SPI of RM ch. 20", crlf,
          "  SPI1: the PA7-PA6 strap is the loopback (PA8 is the node's third pad and is never "
          "driven); PA4 is left an input, so SPI1 is software-selected only",
          crlf);
    if constexpr (has_pair) {
        print(serial, "  THE PAIR on SPI", pair_spi, " column ", pair_code, ": NSS P", pair_pins.nss.port,
              pair_pins.nss.pin, ", SCK P", pair_pins.sck.port, pair_pins.sck.pin, ", MISO P", pair_pins.miso.port,
              pair_pins.miso.pin, ", MOSI P", pair_pins.mosi.port, pair_pins.mosi.pin,
              " to the far board running `spi_peer` - the letters of `y`, which FAIL without it", crlf);
    }
    if constexpr (has_spi3) {
        print(serial, "  SPI3: PA15/PB3/PB4/PB5 wired to SPI2's PB12..PB15 for letters h and i, "
                      "each wire looked for first",
              crlf);
    }
    bench.menu();
}

}  // namespace

BRIO_CH32_LEAF_VECTOR(systick_handler) { brio::Ticker::tick(); }
BRIO_CH32_LEAF_VECTOR(usart1_handler) { (void)Serial::isr(); }

BRIO_CH32_VECTOR(spi1_handler) {
    spi1_isr_entries = spi1_isr_entries + 1u;
    if (txe_mode) {
        // Letter f's transmit stream: the next frame on each TXE, the
        // interrupt silenced after the last.
        txe_entries = txe_entries + 1u;
        if ((S1::isr() & SpiFlag::txe) != 0u) {
            const uint16_t k = txe_next;
            if (k < txe_frames) {
                S1::data8(tx_buf[k]);
                txe_next = static_cast<uint16_t>(k + 1u);
            } else {
                S1::txe_interrupt(false);
                txe_mode = false;
            }
        }
        return;
    }
    if (err_mode) {
        err_isr_entries = err_isr_entries + 1u;
        err_isr_flags = err_isr_flags | S1::isr();
        S1::error_interrupt(false);
        return;
    }
    if (dma_host_live) {
        // The ENGINED host: only its command phase runs on this pump,
        // and isr() is what hands the data phase over to the channels.
        if (Dma1::isr()) {
            host_done = true;
        }
        return;
    }
    if (bus_ao_live) {
        if (Host1::isr()) {
            brio::post<kl::SpiArb>(brio::TransferDone{Host1::status()});
        }
        return;
    }
    if (Host1::isr()) {
        host_done = true;
    }
}

BRIO_CH32_VECTOR(spi2_handler) {
    if constexpr (pair_spi == 2u) {
        if (pair_err_live) {
            pair_err_entries = pair_err_entries + 1u;
            pair_err_flags = pair_err_flags | PairS::isr();
            PairS::error_interrupt(false);
            return;
        }
    }
    // SPI2's pump: letter a's host, and on the CH32V203 the pair's (the
    // same class there - SPI2's one column is the pair's link) - under
    // letter n's timed arbiter there.
    if constexpr (pair_spi == 2u) {
        if (pair_bus_live) {
            if (PairHost::isr()) {
                brio::post<kt::PairArb>(brio::TransferDone{PairHost::status()});
            }
            return;
        }
    }
    if (Spi2Host::isr()) {
        host_done = true;
    }
}

/// SPI3's vector, the CH32V303's: the pair's pump and its error source. On
/// the other series the body is empty and nothing in the vector table
/// names it.
BRIO_CH32_VECTOR(spi3_handler) {
    if constexpr (pair_spi == 3u) {
        if (pair_err_live) {
            pair_err_entries = pair_err_entries + 1u;
            pair_err_flags = pair_err_flags | PairS::isr();
            PairS::error_interrupt(false);
            return;
        }
        if (pair_bus_live) {
            if (PairHost::isr()) {
                brio::post<kt::PairArb>(brio::TransferDone{PairHost::status()});
            }
            return;
        }
        if (pair_pump_live && PairHost::isr()) {
            host_done = true;
        }
    }
}

/// SPI1's engines: one vector per channel on this family, each servicing
/// its own - the receive channel's the transaction's one interrupt, the
/// transmit channel's armed for an error alone.
BRIO_CH32_VECTOR(dma1_channel2_handler) {
    spi1_dma_entries = spi1_dma_entries + 1u;
    if (dma_host_live && Dma1::dma_rx_isr()) {
        host_done = true;
    }
}
BRIO_CH32_VECTOR(dma1_channel3_handler) {
    spi1_dma_entries = spi1_dma_entries + 1u;
    if (dma_host_live && Dma1::dma_tx_isr()) {
        host_done = true;
    }
}

/// SPI2's engines, the same way round: letter a's, and on the CH32V203 the
/// pair's (the same class there).
BRIO_CH32_VECTOR(dma1_channel4_handler) {
    if ((dma_host_live || pair_dma_live) && Spi2Dma::dma_rx_isr()) {
        host_done = true;
    }
}
BRIO_CH32_VECTOR(dma1_channel5_handler) {
    if ((dma_host_live || pair_dma_live) && Spi2Dma::dma_tx_isr()) {
        host_done = true;
    }
}

/// SPI3's engines on DMA2, the CH32V303's pair; nothing names them on the
/// other series.
BRIO_CH32_VECTOR(dma2_channel1_handler) {
    if constexpr (pair_spi == 3u) {
        if (pair_dma_live && PairDma::dma_rx_isr()) {
            host_done = true;
        }
    }
}
BRIO_CH32_VECTOR(dma2_channel2_handler) {
    if constexpr (pair_spi == 3u) {
        if (pair_dma_live && PairDma::dma_tx_isr()) {
            host_done = true;
        }
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    third_pad_idle();
    // The pair's engines' controller: DMA1 for SPI2, DMA2 for SPI3.
    Dma<PairS::dma_tx_slot.controller>::open();

    bench.letter('a', "the rates: every BR code, and the clock counted on its own pad", ta_rates);
    bench.letter('b', "the host with its MISO held by its own port, no wire", tb_held);
    bench.letter('c', "THE LOOPBACK on SPI1: the pump, the ladder to 36 MHz, the engines, the arbiter",
                 tc_loop);
    bench.letter('e', "the flags, the vectors, the line modes, the disable rule", te_flags);
    bench.letter('f', "a polled wait that runs out, and the transmit interrupt", tf_stalled_txe);
    register_spi3_letters();
    register_pair_letters();

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "PLL144" : "FAILED",
              " tick=", tick_ok ? "STK" : "FAILED", crlf);
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
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        print(serial, "  stack: ", brio::stack_untouched(), " B never touched", crlf);
        bench.prompt();
    }
}
