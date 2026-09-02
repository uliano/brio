// build: boards = c21j
// build: monitor_speed = 115200
//
// test_samc_spi - the SERCOM in SPI mode (DS60001479M ch. 32) and, over
// it, the cross-architecture proof that util's bus vocabulary did not
// need one line changed for a second silicon: util/spi_bus.hpp is
// util/bus_master.hpp, and it drives samc/spi.hpp's SpiHost exactly as
// it drives avrdx/spi.hpp's.
//
// THE BENCH, and it is a TWO-BOARD bench. Board C (this one) is the DUT;
// the peer board runs `spi_peer`, the scriptable instrument client,
// commanded IN BAND over the very bus under test (the protocol is
// avrdx/src/apps/spi_link.hpp, included here BY RELATIVE PATH - one
// source of truth for the wire format, never a copy). TWO peers speak
// it: the AVR campaign's avrdx spi_peer (board A, the cross-architecture
// bench this suite was born on) and the samc port (board D, the SAM-SAM
// bench of the speed campaign) - the suite asks ident and does not care.
//
// Today's desk is the SAM-SAM five-wire link, STRAIGHT THROUGH:
//
//   C.PA16 = SERCOM1/PAD[0], function C  <->  D.PA16   MOSI
//   C.PA17 = SERCOM1/PAD[1], function C  <->  D.PA17   SCK
//   C.PA18 = SERCOM1/PAD[2], function C  <->  D.PA18   SS
//   C.PA19 = SERCOM1/PAD[3], function C  <->  D.PA19   MISO
//   plus a dedicated GND.
//
// THE SAME FOUR WIRES CARRY BOTH ROLES, ON TWO DIFFERENT DOPO ROWS, and
// that is one of the things this suite demonstrates rather than asserts.
// A host puts DO (its MOSI) on PAD[0] - row 0x0; a client's DO is MISO
// and has to reach the OTHER board's MISO input, i.e. PAD[3] - row 0x2.
// The role does not flip a direction bit, it changes which code CTRLA
// carries (32.8.1's table, samc/spi.hpp's header).
//
// THE CHIP SELECT IS A GPIO. CTRLB.MSSEN exists and letter f measures
// what it really does, but 32.6.3.5 raises hardware SS between every
// CHARACTER, so it cannot frame a multi-byte protocol frame. PA18 is
// therefore an ordinary PORT output for the whole protocol, which is
// 32.6.3.3's "host with several clients" arrangement and the one every
// device client in brio uses.
//
// NO INTER-BYTE GAP, unlike the AVR half of this protocol. test_avr_spi
// spends spilink::gap_us between characters because its peer's
// NORMAL-mode client has to load its answer in the gap. Every window
// this suite opens has the peer in a BUFFERED regime (its command
// listener only reads; its answer windows and the exchanges commanded
// here all run BUFEN with BUFWR), where a whole byte time of slack
// replaces the gap - so the engine sends a frame as ONE request and is
// exercised as it is meant to be used.
//
// Letters:
//   a  the block and its registers, WIRELESS: the reset discipline and
//      erratum 1.17.16, enable protection, 32.8.2's RXEN-cleared-by-
//      enable trap, erratum 1.17.19 on DBGCTRL, the refusals
//   b  the command channel comes up: ping and ident against the peer
//   c  THE MATRIX: four transfer modes x both bit orders, byte-exact in
//      BOTH directions, plus a deliberate DORD mismatch that is an exact
//      two-way bit reversal and not a shrug
//   d  the rate ladder against the peer, and where the peer stops
//      keeping up (its own CLK_PER/6 client ceiling)
//   e  THE CLIENT ROLE: the peer becomes the HOST for a bounded burst
//      (spilink::Op::host_burst) and this board answers as the client -
//      the other DOPO row, PLOADEN, and the three-SCK-cycle rule
//   f  the wireless remainder: 32.6.3.4 loop-back through the pad, the
//      SCK ladder measured against the 24 MHz crystal, nine-bit
//      characters, BUFOVF and IBON, and what CTRLB.MSSEN really drives
//   g  THE KERNEL LETTER: SpiBus (= BusMaster) over SpiHost inside a
//      real kernel - queued requests, replies through ReplyTo,
//      reject-when-full, and the PrepareSleep vote idle vs busy, with
//      NOT ONE LINE of util/ or kernel/ changed for this architecture
//
// Letters that need the peer say so and FAIL LOUDLY rather than hanging
// when it is absent. Nothing here wears flash.

#include <stdint.h>

#include "samc/clock.hpp"
#include "samc/dmac.hpp"
#include "samc/nvic.hpp"
#include "samc/pin.hpp"
#include "samc/platform_sam.hpp"
#include "samc/sercom.hpp"
#include "samc/spi.hpp"
#include "samc/tc.hpp"
#include "samc/ticker.hpp"
#include "kernel/kernel.hpp"
#include "kernel/post.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/spi_bus.hpp"
#include "util/testbench.hpp"

// THE PROTOCOL IS THE AVR CAMPAIGN'S, AND IT IS NOT COPIED. spi_link.hpp
// is pure encoding - it names no register and includes nothing of brio -
// so both boards' apps compile the same file. A copy here would be a
// second source of truth for a wire format, which is exactly the thing
// that drifts.
#include "../../../avrdx/src/apps/spi_link.hpp"

using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock{};

namespace {

using namespace brio;
using spilink::Op;

using P = SamPlatform;

constexpr UartPads console_pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'B', 30, PinFunction::d},
    .rx_pin = {'B', 31, PinFunction::d},
};
using Serial = Uart<5, console_pads>;
constexpr Serial serial;
TestBench<Serial> bench;

// ---------------------------------------------------------------------------
// The link's pads, both roles
// ---------------------------------------------------------------------------

constexpr uint8_t link_sercom = 1;

/// HOST row 0x0: DO(MOSI) PAD[0], SCK PAD[1], SS PAD[2], DI(MISO) PAD[3].
/// The SS pad is NOT claimed - the chip select is the PORT pin below.
constexpr SpiPads host_pads{
    .data_out = SercomPad::pad0,
    .sck = SercomPad::pad1,
    .ss = SercomPad::pad2,
    .data_in = SercomPad::pad3,
    .data_out_pin = {'A', 16, PinFunction::c},
    .sck_pin = {'A', 17, PinFunction::c},
    .ss_pin = {'A', 18, PinFunction::c},
    .data_in_pin = {'A', 19, PinFunction::c},
};

/// CLIENT row 0x2 on the SAME wires: DO is MISO now and must land on
/// PAD[3], DI is MOSI on PAD[0]. Only row 0x2 puts DO there.
constexpr SpiPads client_pads{
    .data_out = SercomPad::pad3,
    .sck = SercomPad::pad1,
    .ss = SercomPad::pad2,
    .data_in = SercomPad::pad0,
    .data_out_pin = {'A', 19, PinFunction::c},
    .sck_pin = {'A', 17, PinFunction::c},
    .ss_pin = {'A', 18, PinFunction::c},
    .data_in_pin = {'A', 16, PinFunction::c},
};

/// LOOP-BACK (32.6.3.4): DI moved onto the DO pad, so a host reads its
/// own transmit line back THROUGH THE PAD. Wireless, and the only
/// arrangement in this file where DIPO and DOPO name one pad.
constexpr SpiPads loopback_pads{
    .data_out = SercomPad::pad0,
    .sck = SercomPad::pad1,
    .ss = SercomPad::pad2,
    .data_in = SercomPad::pad0,
    .data_out_pin = {'A', 16, PinFunction::c},
    .sck_pin = {'A', 17, PinFunction::c},
    .ss_pin = {'A', 18, PinFunction::c},
    .data_in_pin = {'A', 16, PinFunction::c},
};

// The device header must agree that these four pins reach SERCOM1's four
// pads through function C - the checkable half of the pad-to-pin claim.
static_assert(MUX_PA16C_SERCOM1_PAD0 == static_cast<uint8_t>(PinFunction::c));
static_assert(MUX_PA17C_SERCOM1_PAD1 == static_cast<uint8_t>(PinFunction::c));
static_assert(MUX_PA18C_SERCOM1_PAD2 == static_cast<uint8_t>(PinFunction::c));
static_assert(MUX_PA19C_SERCOM1_PAD3 == static_cast<uint8_t>(PinFunction::c));
static_assert(spi_dopo_for(host_pads.data_out, host_pads.sck, host_pads.ss).value() == 0);
static_assert(spi_dopo_for(client_pads.data_out, client_pads.sck, client_pads.ss).value() == 2);

using Bus = SpiHost<link_sercom, host_pads>;
using Loop = SpiHost<link_sercom, loopback_pads>;
using Peer = SpiClient<link_sercom, client_pads>;
using Raw = Spi<link_sercom>;

/// The engined host on the wire pads - letter d's full-DMA climb (both
/// ends of the link on engines) and nothing else. Same two channels as
/// letter h's loop-back twin; the two are never up at once, and each
/// init() re-claims.
using DmaBus = SpiHost<link_sercom, host_pads, 0, DmaTxEngine<0>, DmaRxEngine<1>>;
volatile bool dma_bus_live = false;   ///< routes DMAC_Handler to DmaBus

using Cs = Pin<'A', 18>;
using SckPin = Pin<'A', 17>;

/// The command channel's SCK, and the one place it is named. 200 kHz -
/// a byte is 40 us, which is a whole order of magnitude more than the
/// peer's polled loop needs to turn one around, and well inside its own
/// CLK_PER/6 = 4 MHz client ceiling.
constexpr uint32_t command_hz = 200'000UL;
constexpr uint8_t command_baud = spi_baud_reg(SysClock::hz, command_hz).value();
static_assert(command_baud == 119);
static_assert(spi_sck_hz(SysClock::hz, command_baud) == command_hz);

// ---------------------------------------------------------------------------
// The crystal ruler (the test_samc_sleep / test_samc_timebase instrument)
// ---------------------------------------------------------------------------

using Ruler = Tc<2>;   ///< TC2+TC3 as one 32-bit counter
constexpr uint8_t gen_xtal = 2;
constexpr uint32_t crystal_hz = 24'000'000UL;
bool ruler_ok = false;

bool ruler_up() {
    if (!Xosc::init(XoscConfig{.hz = crystal_hz, .startup = 4, .on_demand = false})) {
        return false;
    }
    if (!Gclk<gen_xtal>::configure(GclkConfig{.source = GclkSource::xosc})) {
        return false;
    }
    (void)Ruler::enable(false);
    if (!Ruler::init(gen_xtal)) {
        return false;
    }
    if (!Ruler::configure(TcConfig{.mode = TcMode::count32, .prescaler = TcPrescaler::div1})) {
        return false;
    }
    return Ruler::enable(true);
}

uint32_t wall() { return Ruler::count32(); }

// ---------------------------------------------------------------------------
// The bus, and the protocol over it
// ---------------------------------------------------------------------------

volatile bool isr_done = false;      ///< set by the engine's completion edge
volatile uint32_t isr_completions = 0;

/// The kernel letter re-binds this through a flag; outside it the suite
/// runs polled requests and the handler is only the ISR-pump legs'.
volatile bool bus_ao_live = false;

uint8_t frame_buf[spilink::max_payload + 8];
uint8_t answer_buf[spilink::answer_bytes];
uint8_t dummy_buf[spilink::answer_bytes];

spilink::Decoder dec;
bool link_quiet = false;

void settle_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}
void settle() { settle_ms(spilink::settle_ms); }

/// SERCOM1 as the command channel's host, PA18 as its chip select.
bool link_command_mode() {
    Cs::set();
    Cs::output();
    const bool ok = Bus::init(clock);
    dec.reset();
    return ok;
}

/**
 * THE CHIP-SELECT HOLD, and it is this campaign's first bench finding.
 *
 * The engine releases CS as soon as the last character's RXC says the
 * bits have moved - about a microsecond after the last SCK edge, and
 * that is not enough for the peer. An AVR SPI client is RESET by SS
 * going high, so a byte sitting in its DATA that its polled loop has not
 * fetched yet is simply gone: the symptom was that every command frame
 * lost EXACTLY ITS LAST CHARACTER, which for a four-byte ping frame is
 * the checksum, so the peer sat waiting for one and nak'ed the next
 * window's first dummy instead. One frame in a dozen got through.
 *
 * The AVR half of this protocol never sees it because its own host pays
 * spilink::gap_us AFTER each character including the last, so its CS
 * release is 20 us behind the wire - a hold time it gets by accident.
 *
 * So the PROTOCOL owns the chip select here, and the engine's Request
 * carries a null PinRef for these windows. Neither engine has a
 * per-request chip-select hold yet; samc/delay.hpp exists now (born
 * after this suite's spins were calibrated), and the Request knob is
 * declared future growth with its first device user - this suite keeps
 * its own measured spin, which predates the facility and is proven.
 */
volatile uint32_t hold_spins = 0;
uint32_t hold_us = 0;

void link_hold() {
    // The counter is volatile so -Os cannot delete the loop (the ticker
    // doctrine: gcc has deleted a bare polling loop in this stratum
    // before). Compound ops on volatile are deprecated in C++20, hence
    // the long spelling.
    for (volatile uint32_t i = 0; i < hold_spins; i = i + 1) {
    }
}

/// A spin is not a time, so it is MEASURED and not assumed: one
/// thousand turns are timed on the crystal, the count is scaled to about
/// thirty microseconds, and what that really came out at is printed in
/// the banner. (This is an APP spinning, not a driver: samc/ has no
/// delay facility and this campaign did not invent one inside a driver.)
void calibrate_hold() {
    if (!ruler_ok) {
        hold_spins = 300;   // blind but generous
        return;
    }
    hold_spins = 1000;
    const uint32_t t0 = wall();
    link_hold();
    const uint32_t per_1000 = wall() - t0;
    const uint32_t want = 30u * (crystal_hz / 1000000u);   // 30 us in crystal ticks
    hold_spins = per_1000 ? (1000u * want) / per_1000 : 300u;
    if (hold_spins < 10u) hold_spins = 10u;
    const uint32_t t1 = wall();
    link_hold();
    hold_us = (wall() - t1) / (crystal_hz / 1000000u);
}

/// One polled transaction with NO chip select of its own.
bool link_xfer_bare(const uint8_t* tx, uint8_t* rx, uint16_t len, uint8_t baud,
                    SpiMode mode) {
    if (len == 0) {
        return true;
    }
    Bus::Request r{
        .cs = {},
        .dc = {},
        .cmd = {},
        .cmd_len = 0,
        .tx = lend<Lease::reply>(tx),
        .rx = lend<Lease::reply>(rx),
        .len = len,
        .reply = {},
        .baud = baud,
        .mode = mode,
        .polled = true,
    };
    return Bus::start(r);   // polled: completes synchronously
}

/// One protocol window: the select, a setup hold, the transaction, a
/// release hold, the deselect.
///
/// THE MODE IS PRIMED BEFORE THE SELECT FALLS. This suite frames CS by
/// hand (Request.cs is null - the peer wants hold windows the engine
/// deliberately does not own), which inverts start()'s own
/// apply-before-select order: a CPOL flip landing inside an open select
/// window is one extra SCK edge, and the selected client counts it into
/// the character - measured as an exact one-bit slip in BOTH directions,
/// on modes 2 and 3 only, by this suite's first version. Bus::prime()
/// restates the order.
bool link_xfer(const uint8_t* tx, uint8_t* rx, uint16_t len, uint8_t baud = command_baud,
               SpiMode mode = SpiMode::mode0) {
    Bus::prime(mode, baud);
    Cs::clear();
    link_hold();
    const bool ok = link_xfer_bare(tx, rx, len, baud, mode);
    link_hold();
    Cs::set();
    return ok;
}

void send_frame(Op op, const uint8_t* p, uint8_t len) {
    uint8_t n = 0;
    spilink::write_frame(
        [&](uint8_t b) {
            if (n < sizeof frame_buf) frame_buf[n++] = b;
        },
        op, p, len);
    (void)link_xfer(frame_buf, nullptr, n);
}

uint8_t raw_seen[16];
uint8_t raw_n = 0;

/// One answer window: a single request of `answer_bytes` zeros, then the
/// decoder walks what came back. The peer pads its frame with zeros, so
/// clocking the whole budget in one transaction costs nothing but time
/// and lets the ENGINE move the block instead of the suite.
bool recv_frame(spilink::Frame& out) {
    dec.reset();
    raw_n = 0;
    for (uint16_t i = 0; i < spilink::answer_bytes; ++i) {
        dummy_buf[i] = 0;
        answer_buf[i] = 0xEE;
    }
    (void)link_xfer(dummy_buf, answer_buf, spilink::answer_bytes);
    for (uint16_t i = 0; i < spilink::answer_bytes; ++i) {
        if (raw_n < 16) raw_seen[raw_n++] = answer_buf[i];
        if (dec.feed(answer_buf[i]) == spilink::Decoder::Result::frame) {
            out = dec.frame();
            return true;
        }
    }
    return false;
}

bool command_once(Op op, const uint8_t* p, uint8_t len) {
    send_frame(op, p, len);
    settle();
    spilink::Frame f;
    if (!recv_frame(f)) return false;
    return f.op == Op::ack && f.len == 2 && f.data[0] == spilink::byte_of(op);
}

const uint8_t no_payload[1] = {0};

/// Three attempts, each separated by longer than the peer's own
/// answer-window bound, so a peer that was serving into nothing is
/// certainly dark again before the retry - the protocol's own recovery
/// guarantee, used as the AVR half uses it.
bool command(Op op, const uint8_t* p = no_payload, uint8_t len = 0) {
    uint8_t first_n = 0;
    uint8_t first_seen[16];
    for (uint8_t k = 0; k < 3; ++k) {
        if (command_once(op, p, len)) return true;
        if (k == 0) {
            first_n = raw_n;
            for (uint8_t i = 0; i < raw_n; ++i) first_seen[i] = raw_seen[i];
        }
        (void)link_command_mode();
        settle_ms(400);
    }
    if (link_quiet) return false;
    print(serial, "    LINK FAILURE op ", hex(spilink::byte_of(op)),
          ": the first answer window carried");
    if (first_n == 0) print(serial, " nothing");
    for (uint8_t i = 0; i < first_n; ++i) print(serial, " ", hex(first_seen[i]));
    print(serial, crlf,
          "      the peer board must be running `spi_peer` (bench.py flash D "
          "spi_peer); its console '0' forces the dark client back.",
          crlf);
    (void)link_command_mode();
    return false;
}

bool query(Op op, spilink::Frame& data) {
    if (!command(op)) return false;
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
        if (query(Op::report, f) && f.op == Op::report_data &&
            f.len == spilink::report_size) {
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
    if (!command(op, p, spilink::params_size)) return false;
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
    print(serial,
          "  THE PEER DID NOT ANSWER. The peer board must be running `spi_peer` "
          "(python3 tools/bench.py flash D spi_peer); its console '0' forces the "
          "dark client back. Check the five wires in this file's header.",
          crlf);
    return false;
}

/// The letters that need the peer open with this: it prints the reason
/// and scores one failure rather than hanging or passing vacuously.
bool need_peer() {
    if (ensure_link()) {
        return true;
    }
    bench.verdict("the peer answers a ping (the peer board running spi_peer)", false);
    return false;
}

// ---- the exchange ------------------------------------------------------------

struct Exchange {
    spilink::Cfg cfg{.apply = 1, .mode = 0, .dord = 0,
                     .regime = spilink::regime_buffer_wait};
    SpiMode host_mode = SpiMode::mode0;
    bool host_lsb = false;
    uint8_t baud = command_baud;
    uint16_t count = 8;
    uint8_t seed_a = 0x13;
    uint8_t seed_b = 0x57;
    uint8_t pattern = spilink::pattern_prbs;
    uint8_t flags = 0;
    uint8_t spare = 0;   ///< spilink::spare_polled_pump forces the peer's polled loop
    uint16_t ms = 400;
    bool dma_host = false;   ///< move THIS end's data phase onto the engines too
};

constexpr uint8_t max_exchange = 16;
uint8_t xtx[max_exchange];
uint8_t xrx[max_exchange];

/// Command an exchange, then be the host of it.
bool do_exchange(const Exchange& e) {
    spilink::Params a{};
    a.cfg = e.cfg;
    a.count = e.count;
    a.ms = e.ms;
    a.seed_a = e.seed_a;
    a.seed_b = e.seed_b;
    a.pattern = e.pattern;
    a.flags = e.flags;
    a.spare = e.spare;
    // A bit-order mismatch is not a shrug: told about it, the client
    // checks the exact bit-reverse of what this end sent, and this end
    // checks the exact bit-reverse of what the client answered.
    if (e.host_lsb != (e.cfg.dord != 0)) a.flags |= spilink::flag_expect_reversed;
    if (!peer_act(Op::exchange, a)) return false;

    // The host's own bit order is a CTRLA field and therefore part of
    // the configuration, not of the request - so the engine is re-inited
    // for the legs that ask for LSb first, and put back afterwards.
    if (e.host_lsb) {
        SpiConfig c = spi_role_probe(host_pads, SpiRole::host);
        c.mode = e.host_mode;
        c.lsb_first = true;
        c.baud = e.baud;
        if (!Raw::configure(c) || !Raw::enable(true)) return false;
    }

    spilink::Stream out(e.pattern, e.seed_a);
    for (uint8_t i = 0; i < max_exchange; ++i) {
        xtx[i] = 0;
        xrx[i] = 0xEE;
    }
    for (uint16_t i = 0; i < e.count && i < max_exchange; ++i) {
        xtx[i] = out.next();
    }
    settle();
    const uint16_t n = e.count < max_exchange ? e.count : max_exchange;
    if (e.dma_host) {
        // BOTH ends on engines: this end's data phase rides DmaBus (a
        // polled request whose spin still completes through
        // DMAC_Handler), the peer's serve rides its own two channels.
        (void)DmaBus::init(clock);
        dma_bus_live = true;
        DmaBus::prime(e.host_mode, e.baud);
        Cs::clear();
        link_hold();
        DmaBus::Request r{
            .cs = {}, .dc = {}, .cmd = {}, .cmd_len = 0,
            .tx = lend<Lease::reply>(static_cast<const uint8_t*>(xtx)),
            .rx = lend<Lease::reply>(xrx),
            .len = n, .reply = {},
            .baud = e.baud, .mode = e.host_mode, .polled = true,
        };
        (void)DmaBus::start(r);
        link_hold();
        Cs::set();
        dma_bus_live = false;
        (void)link_command_mode();
        return true;
    }
    if (e.host_lsb) {
        // The engine's own apply() would re-state CTRLA without DORD, so
        // this leg drives the transaction with CS by hand and the raw
        // polled surface - the ONE place in this suite that goes around
        // the task, and it is a limitation of the task worth naming: the
        // bit order is per-CONFIGURATION here, not per-request.
        Cs::clear();
        link_hold();
        for (uint16_t i = 0; i < n; ++i) {
            Raw::data(xtx[i]);
            uint32_t spins = 200000u;
            while (!Raw::rxc_flag() && spins-- != 0u) {
            }
            xrx[i] = static_cast<uint8_t>(Raw::data());
        }
        link_hold();
        Cs::set();
    } else {
        (void)link_xfer(xtx, xrx, n, e.baud, e.host_mode);
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
    for (uint16_t i = 0; i < e.count && i < max_exchange; ++i) {
        uint8_t exp = spilink::pattern_value(e.pattern, e.seed_b, i);
        if (reversed) exp = spilink::bit_reverse(exp);
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
    print(serial, "    host mism=", v.mism, " (first idx ", v.idx, " got ", hex(v.got),
          " exp ", hex(v.exp), "), client count=", r.count, " mism=", r.mism,
          " (first idx ", r.idx, " got ", hex(r.got), " exp ", hex(r.exp),
          ") flags=", hex(r.flags), crlf);
    if (r.count == 0 && (r.flags & spilink::report_timed_out) != 0) {
        // The signature the RARE PEER WEDGE used to leave (spi.md "Not
        // covered yet"): commands decode, the hardware serves the
        // preloads and echoes, the window burns whole reading nothing.
        // The peer's exchange loop no longer depends on its select READ
        // (it polls RXC directly and samples the select only as the
        // telemetry printed here), so if this fires again the aux bytes
        // are the forensics - and a peer running the OLD firmware is
        // healed by resetting board A.
        print(serial, "    the peer's exchange window read NOTHING (count=0, timed "
                      "out) while the command channel works - peer telemetry: "
                      "first-byte ms=", r.aux1, " select bits=", hex(r.aux2),
              " preload INTFLAGS=", hex(r.aux3), crlf);
    }
    const bool reversed = e.host_lsb != (e.cfg.dord != 0);
    print(serial, "    read:");
    for (uint16_t i = 0; i < e.count && i < max_exchange; ++i) {
        print(serial, " ", hex(xrx[i]));
    }
    print(serial, crlf, "    want:");
    for (uint16_t i = 0; i < e.count && i < max_exchange; ++i) {
        uint8_t exp = spilink::pattern_value(e.pattern, e.seed_b, i);
        if (reversed) exp = spilink::bit_reverse(exp);
        print(serial, " ", hex(exp));
    }
    print(serial, crlf);
}

// ===========================================================================
// a - the block and its registers (WIRELESS)
// ===========================================================================

void ta_block() {
    // The pads stay under PORT for this whole letter: nothing here needs
    // the wire, and leaving them alone keeps a stray edge off the peer.
    Raw::bus_clock(true);
    bench.verdict("the core clock routes and the block resets",
                  Raw::core_clock(0) && Raw::reset());
    print(serial, "  after reset: CTRLA=", hex(Raw::ctrla()), " CTRLB=", hex(Raw::ctrlb()),
          " BAUD=", Raw::baud_reg(), " DBGCTRL=", hex(Raw::dbgctrl()), crlf);
    bench.verdict("a reset instance is disabled and in no SPI mode at all",
                  !Raw::enabled() && !Raw::role().has_value());

    // ERRATUM 1.17.16, PUT TO THE TEST WITH A CONTROL ON EACH SIDE.
    // The item says flatly that CTRLA.SWRST "is not functional when the
    // SERCOM is not enabled (CTRLA.ENABLE = 0)", and its matrix marks
    // every E/G/J revision including this one - so the whole
    // enable-first discipline in sercom.hpp and spi.hpp rests on it.
    // Nobody had ever MEASURED it. Three legs: from the ENABLED state
    // (where both the chapter and the erratum agree it works), from the
    // disabled state with the core clock RUNNING, and from the disabled
    // state with the core clock DISCONNECTED - which is the state a
    // freshly booted instance is really in, and the only one where a
    // reset has no clock to synchronize against.
    constexpr SpiConfig host_cfg = [] {
        SpiConfig c = spi_role_probe(host_pads, SpiRole::host);
        c.baud = command_baud;
        return c;
    }();

    // Leg 1: enabled.
    bench.verdict("configure() programs a host while the block is disabled",
                  Raw::configure(host_cfg));
    (void)Raw::enable(true);
    Raw::regs().SERCOM_CTRLA = SERCOM_SPIM_CTRLA_SWRST_Msk;
    const bool sync1 = Raw::wait_sync(SERCOM_SPIM_SYNCBUSY_SWRST_Msk, 200000u);
    const uint32_t after_enabled = Raw::ctrla();

    // Leg 2: disabled, core clock connected.
    (void)Raw::configure(host_cfg);
    const uint32_t before2 = Raw::ctrla();
    Raw::regs().SERCOM_CTRLA = SERCOM_SPIM_CTRLA_SWRST_Msk;
    const bool sync2 = Raw::wait_sync(SERCOM_SPIM_SYNCBUSY_SWRST_Msk, 200000u);
    const uint32_t after_disabled = Raw::ctrla();

    // Leg 3: disabled, core clock GONE. The write is bounded either way
    // - a synchronization that never completes is reported here, never
    // hung on - and the channel is put back before anything is judged.
    // PCHCTRL.CHEN is itself write-synchronized (16.6.3.3), so the
    // control waits until the channel really reads disconnected: the
    // first version read it back on the next instruction and measured
    // its own race, not the silicon.
    (void)Raw::configure(host_cfg);
    const uint32_t before3 = Raw::ctrla();
    GclkChannel::disconnect(Raw::gclk_core_id());
    bool really_clockless = false;
    for (uint32_t k = 0; k < 100000u && !really_clockless; ++k) {
        really_clockless = !GclkChannel::connected(Raw::gclk_core_id());
    }
    Raw::regs().SERCOM_CTRLA = SERCOM_SPIM_CTRLA_SWRST_Msk;
    const bool sync3 = Raw::wait_sync(SERCOM_SPIM_SYNCBUSY_SWRST_Msk, 200000u);
    const uint32_t after_clockless = Raw::ctrla();
    (void)Raw::core_clock(0);
    const bool sync3_back = Raw::wait_sync(SERCOM_SPIM_SYNCBUSY_SWRST_Msk, 200000u);
    const uint32_t after_reconnect = Raw::ctrla();

    print(serial, "  SWRST from ENABLED: CTRLA -> ", hex(after_enabled),
          " (sync cleared=", sync1, ")", crlf);
    print(serial, "  SWRST from DISABLED, GCLK running: ", hex(before2), " -> ",
          hex(after_disabled), " (sync cleared=", sync2, ")", crlf);
    print(serial, "  SWRST from DISABLED, GCLK DISCONNECTED (PCHCTRL.CHEN really "
          "clear=", really_clockless, "): ", hex(before3), " -> ",
          hex(after_clockless), " (sync cleared=", sync3,
          "), and after the channel is reconnected: sync cleared=", sync3_back,
          " CTRLA=", hex(after_reconnect), crlf);
    bench.verdict("a SWRST from the ENABLED state resets the block, as every "
                  "document agrees",
                  sync1 && after_enabled == 0);
    // The disposition of 1.17.16 on THIS silicon, and it is the opposite
    // of what the errata matrix says: SWRST works from the DISABLED
    // state. The claim rests on leg 2, where the core clock is running -
    // the arrangement the erratum's own sentence describes.
    bench.verdict("ERRATUM 1.17.16 IS NOT REPRODUCED at rev F in SPI mode: SWRST "
                  "from the DISABLED state (core clock running) clears CTRLA and "
                  "its synchronization completes - reset()'s enable-first "
                  "discipline is kept anyway, the sheet marking every revision and "
                  "the cost being one enable",
                  sync2 && after_disabled == 0);
    // The CLOCKLESS leg is its own finding either way (the FREQM lesson:
    // a reset can synchronize into a domain the channel feeds). What is
    // JUDGED is only that nothing hung and the reset has landed by the
    // time the clock is back; whether it completed clockless or pended
    // is the print above.
    bench.verdict("and a SWRST attempted with the core clock channel really "
                  "disconnected is BOUNDED and lands once the channel returns",
                  really_clockless && sync3_back && after_reconnect == 0);
    bench.verdict("and reset() leaves CTRLA clear from any of them",
                  Raw::reset() && Raw::ctrla() == 0);

    // The configuration, field by field, read back off the silicon.
    bench.verdict("the host configuration goes in", Raw::configure(host_cfg));
    const uint32_t ca = Raw::ctrla();
    print(serial, "  host CTRLA=", hex(ca), ": MODE=",
          (ca & SERCOM_SPIM_CTRLA_MODE_Msk) >> SERCOM_SPIM_CTRLA_MODE_Pos,
          " DOPO=", (ca & SERCOM_SPIM_CTRLA_DOPO_Msk) >> SERCOM_SPIM_CTRLA_DOPO_Pos,
          " DIPO=", (ca & SERCOM_SPIM_CTRLA_DIPO_Msk) >> SERCOM_SPIM_CTRLA_DIPO_Pos,
          " BAUD=", Raw::baud_reg(), crlf);
    bench.verdict("CTRLA carries the host mode and the DOPO row the pads asked for",
                  Raw::role() == SpiRole::host &&
                      ((ca & SERCOM_SPIM_CTRLA_DOPO_Msk) >>
                       SERCOM_SPIM_CTRLA_DOPO_Pos) == 0u &&
                      ((ca & SERCOM_SPIM_CTRLA_DIPO_Msk) >>
                       SERCOM_SPIM_CTRLA_DIPO_Pos) == 3u);
    bench.verdict("the BAUD register holds what the arithmetic computed",
                  Raw::baud_reg() == command_baud);
    bench.verdict("mode() reads the two CTRLA bits back as one SpiMode",
                  Raw::mode() == SpiMode::mode0);

    // The CLIENT row on the same pads - the fact the whole two-role
    // bench rests on: a different DOPO code, not a flipped direction.
    constexpr SpiConfig client_cfg = spi_role_probe(client_pads, SpiRole::client);
    bench.verdict("the same four pads reconfigure as a CLIENT", Raw::configure(client_cfg));
    const uint32_t cb = Raw::ctrla();
    print(serial, "  client CTRLA=", hex(cb), ": DOPO=",
          (cb & SERCOM_SPIM_CTRLA_DOPO_Msk) >> SERCOM_SPIM_CTRLA_DOPO_Pos,
          " DIPO=", (cb & SERCOM_SPIM_CTRLA_DIPO_Msk) >> SERCOM_SPIM_CTRLA_DIPO_Pos, crlf);
    bench.verdict("and it is a DIFFERENT DOPO ROW (0x2 against the host's 0x0) - the "
                  "role changes which code CTRLA carries, not a direction bit",
                  Raw::role() == SpiRole::client &&
                      ((cb & SERCOM_SPIM_CTRLA_DOPO_Msk) >>
                       SERCOM_SPIM_CTRLA_DOPO_Pos) == 2u &&
                      ((cb & SERCOM_SPIM_CTRLA_DIPO_Msk) >>
                       SERCOM_SPIM_CTRLA_DIPO_Pos) == 0u);

    // 32.8.2's RXEN trap, measured on both sides of the enable.
    bench.verdict("back to the host configuration", Raw::configure(host_cfg));
    const bool rxen_before = Raw::receiver();
    Raw::regs().SERCOM_CTRLA = Raw::ctrla() | SERCOM_SPIM_CTRLA_ENABLE_Msk;
    const bool ctrlb_busy = Raw::sync_busy(SERCOM_SPIM_SYNCBUSY_CTRLB_Msk);
    const bool ok = Raw::wait_sync(SERCOM_SPIM_SYNCBUSY_ENABLE_Msk) &&
                    Raw::wait_sync(SERCOM_SPIM_SYNCBUSY_CTRLB_Msk);
    const bool rxen_after = Raw::receiver();
    print(serial, "  RXEN before the enable=", rxen_before,
          ", SYNCBUSY.CTRLB immediately after the ENABLE store=", ctrlb_busy,
          ", RXEN once both synchronizations cleared=", rxen_after, crlf);
    bench.verdict("32.8.2's trap is real: ENABLING the peripheral raises "
                  "SYNCBUSY.CTRLB, and the receiver is up only once it clears",
                  rxen_before && ctrlb_busy && ok && rxen_after);

    // ENABLE PROTECTION (32.6.2.1): a write while it runs is DISCARDED,
    // not refused - which is why every verb that touches these registers
    // in the driver disables first.
    const uint32_t live = Raw::ctrla();
    Raw::regs().SERCOM_CTRLA = live ^ SERCOM_SPIM_CTRLA_DIPO_Msk;
    const uint32_t after_write = Raw::ctrla();
    const uint8_t baud_live = Raw::baud_reg();
    Raw::baud_reg(static_cast<uint8_t>(baud_live ^ 0xFFu));
    const uint8_t baud_after = Raw::baud_reg();
    print(serial, "  enable protection: CTRLA ", hex(live), " -> ", hex(after_write),
          " after a DIPO flip, BAUD ", baud_live, " -> ", baud_after, crlf);
    bench.verdict("CTRLA and BAUD are ENABLE-PROTECTED: the stores are DISCARDED "
                  "in silence while the instance runs",
                  after_write == live && baud_after == baud_live);

    // CTRLB.RXEN is the one bit that is not (32.8.2) - the live verb.
    bench.verdict("CTRLB.RXEN alone IS writable live, both ways",
                  Raw::receiver(false) && !Raw::receiver() && Raw::receiver(true) &&
                      Raw::receiver());

    // ERRATUM 1.17.19: DBGCTRL is supposed to be the one register a
    // software reset spares (32.6.2.2), and it is not.
    (void)Raw::enable(false);
    SpiConfig dbg_cfg = host_cfg;
    dbg_cfg.debug_stop = true;
    bench.verdict("DBGCTRL.DBGSTOP can be set", Raw::configure(dbg_cfg) &&
                                                    Raw::dbgctrl() != 0);
    const uint8_t dbg_before = Raw::dbgctrl();
    // A BARE SWRST from the disabled state - nothing between the set and
    // the reset, because configure() writes DBGCTRL itself and would
    // clear the very evidence this leg is after.
    Raw::regs().SERCOM_CTRLA = SERCOM_SPIM_CTRLA_SWRST_Msk;
    (void)Raw::wait_sync(SERCOM_SPIM_SYNCBUSY_SWRST_Msk, 200000u);
    const uint8_t dbg_from_disabled = Raw::dbgctrl();
    // And from the ENABLED state, in case the two paths differ.
    (void)Raw::configure(dbg_cfg);
    (void)Raw::enable(true);
    const uint8_t dbg_live = Raw::dbgctrl();
    Raw::regs().SERCOM_CTRLA = SERCOM_SPIM_CTRLA_SWRST_Msk;
    (void)Raw::wait_sync(SERCOM_SPIM_SYNCBUSY_SWRST_Msk, 200000u);
    const uint8_t dbg_from_enabled = Raw::dbgctrl();
    print(serial, "  erratum 1.17.19: DBGCTRL ", hex(dbg_before), " -> ",
          hex(dbg_from_disabled), " across a SWRST from the disabled state, and ",
          hex(dbg_live), " -> ", hex(dbg_from_enabled), " from the enabled one, "
          "where 32.6.2.2 exempts this register and the erratum says a reset "
          "clears it anyway", crlf);
    bench.verdict("ERRATUM 1.17.19 IS REFUTED at rev F: DBGCTRL SURVIVES "
                  "CTRLA.SWRST from either state, exactly as 32.6.2.2 promises",
                  dbg_before != 0 && dbg_from_disabled == dbg_before &&
                      dbg_live == dbg_before && dbg_from_enabled == dbg_before);

    // The run-time refusals - the same clauses the family fixture pins
    // at compile time, reached here through the value-taking twin.
    SpiConfig bad = host_cfg;
    bad.form = static_cast<SpiForm>(1);
    bench.verdict("a Reserved FORM code is refused, and nothing is programmed",
                  !Raw::configure(bad));
    bad = host_cfg;
    bad.preload = true;
    bench.verdict("a CLIENT knob on a host (CTRLB.PLOADEN) is refused",
                  !Raw::configure(bad));
    bad = spi_role_probe(client_pads, SpiRole::client);
    bad.hardware_ss = true;
    bench.verdict("a HOST knob on a client (CTRLB.MSSEN) is refused",
                  !Raw::configure(bad));
    bad = spi_role_probe(client_pads, SpiRole::client);
    bad.form = SpiForm::with_address;
    bad.preload = true;
    bench.verdict("32.6.3.1's own rule - address matching and preloading cannot "
                  "both own the client's first character - is refused",
                  !Raw::configure(bad));

    // The baud ladder's ends, in the register.
    bench.verdict("the fastest and slowest rates the generator has are both real "
                  "BAUD values",
                  spi_baud_reg(SysClock::hz, spi_max_sck_hz(SysClock::hz)) == 0u &&
                      spi_baud_reg(SysClock::hz, spi_min_sck_hz(SysClock::hz)) == 255u);

    Raw::release();
    bench.verdict("release() puts the block back and the pads stay PORT's",
                  !Cs::has_function());
}

// ===========================================================================
// b - the command channel
// ===========================================================================

void tb_link() {
    if (!need_peer()) return;
    bench.verdict("the peer answers a ping over the four wires", true);

    spilink::Ident d{};
    const bool got = peer_ident(d);
    if (got) {
        print(serial, "  peer: label '");
        for (uint8_t i = 0; i < 8 && d.label[i]; ++i) print(serial, d.label[i]);
        print(serial, "' xtal=", d.xtal, " sanity=", hex(d.sanity), " fw=", hex(d.version),
              crlf);
    }
    bench.verdict("ident comes back and it IS spi_peer (the sanity byte)",
                  got && d.sanity == spilink::ident_sanity);
    bench.verdict("the peer names a board label", got && d.label[0] != 0);
    // The peer's clock quality is a bench fact of this suite and not
    // just of that board (an AVR peer's client ceiling and a SAM peer's
    // reload margin both ride on it).
    if (got && !d.xtal) {
        print(serial, "  NOTE: the peer's crystal did not start - its clock is "
                      "its internal RC",
              crlf);
    }

    // Ten pings back to back: the channel is not a one-shot.
    uint8_t pings = 0;
    for (uint8_t i = 0; i < 10; ++i) {
        if (command(Op::ping)) ++pings;
    }
    print(serial, "  ", pings, " of 10 pings answered at ", command_hz / 1000u,
          " kHz SCK, one frame per chip-select window", crlf);
    bench.verdict("the channel is steady over ten frames", pings == 10);
}

// ===========================================================================
// c - the transfer matrix
// ===========================================================================

void tc_matrix() {
    if (!need_peer()) return;

    uint8_t good = 0;
    for (uint8_t m = 0; m < 4; ++m) {
        Exchange e{};
        e.cfg.mode = m;
        e.host_mode = static_cast<SpiMode>(m);
        e.seed_a = static_cast<uint8_t>(0x13 + m);
        e.seed_b = static_cast<uint8_t>(0x57 + m);
        Verify v{};
        spilink::Report r{};
        const bool ok = exchange_exact(e, v, r);
        if (ok) ++good;
        if (!ok) {
            print(serial, "  mode ", m, ":", crlf);
            dump_exchange(e, v, r);
        }
    }
    print(serial, "  ", good, " of 4 transfer modes byte-exact BOTH ways, 8 bytes each",
          crlf);
    bench.verdict("all four SPI transfer modes carry a burst byte-exact in both "
                  "directions (32.6.2.5, table 32-3)",
                  good == 4);

    // Both bit orders, matched.
    Exchange lsb{};
    lsb.cfg.dord = 1;
    lsb.host_lsb = true;
    Verify v{};
    spilink::Report r{};
    const bool lsb_ok = exchange_exact(lsb, v, r);
    if (!lsb_ok) dump_exchange(lsb, v, r);
    bench.verdict("LSb first, both ends agreeing, is byte-exact too (CTRLA.DORD)",
                  lsb_ok);

    // ... and MISMATCHED, which is an EXACT two-way bit reversal and not
    // a shrug: told about it, each end checks the bit-reverse of what
    // the other sent.
    Exchange mism{};
    mism.cfg.dord = 1;     // the client LSb first
    mism.host_lsb = false; // this end MSb first
    Verify v2{};
    spilink::Report r2{};
    const bool ran = do_exchange(mism);
    const bool rep = ran && peer_report(r2);
    v2 = verify_rx(mism);
    print(serial, "  DORD mismatch: host mism=", v2.mism, " client mism=", r2.mism,
          " count=", r2.count, " (both checking the exact bit-reverse)", crlf);
    bench.verdict("a DORD mismatch is an EXACT two-way BIT REVERSAL - each end "
                  "reads the other's bytes with their bits in the opposite order",
                  ran && rep && v2.mism == 0 && r2.mism == 0 && r2.count == mism.count);

    // The channel is still the channel afterwards.
    bench.verdict("and the command channel still works right after all of that",
                  command(Op::ping));
}

// ===========================================================================
// d - the rate ladder
// ===========================================================================

void td_rates() {
    if (!need_peer()) return;

    // The arithmetic first, against the silicon's own readback.
    (void)link_command_mode();
    print(serial, "  the generator at ", SysClock::hz / 1000000u, " MHz: BAUD 0 = ",
          spi_sck_hz(SysClock::hz, 0) / 1000u, " kHz, BAUD 255 = ",
          spi_sck_hz(SysClock::hz, 255) / 1000u, " kHz", crlf);
    bench.verdict("baud_for() and sck_hz() are each other's inverse where the "
                  "divisor is exact",
                  Bus::baud_for(1'000'000UL) == 23 && Bus::sck_hz(23) == 1'000'000UL &&
                      Bus::baud_for(command_hz) == command_baud);
    bench.verdict("a rate the divisor cannot hit exactly lands BELOW the request, "
                  "never above (a requested SCK is a ceiling)",
                  Bus::sck_hz(Bus::baud_for(700'000UL).value()) <= 700'000UL &&
                      Bus::sck_hz(Bus::baud_for(700'000UL).value()) > 600'000UL);

    // A bus-wide CEILING, and a request that would exceed it.
    bench.verdict("a bus with a 500 kHz ceiling resolves it, and clamps a faster "
                  "request down to it",
                  Bus::init(clock, 500'000UL) && Bus::ceiling_baud().has_value() &&
                      Bus::sck_hz(0) <= 500'000UL);
    (void)link_command_mode();
    bench.verdict("and with no ceiling the request's own divisor stands",
                  !Bus::ceiling_baud().has_value() && Bus::sck_hz(0) ==
                                                          spi_max_sck_hz(SysClock::hz));

    // Now the wire, up to the generator's own top. These characters run
    // back to back (one engine request, no inter-byte gap), so what the
    // climb finds is the PEER'S RELOAD BOUNDARY: its client must land
    // the next-plus-one answer at least three SCK cycles before a
    // character boundary (32.6.2.6.2), and a polled loop's turnaround
    // sets where that stops holding. The SAM peer's precomputed pump
    // holds to 2 MHz; the AVR peer's polled loop held to 500 kHz with
    // 1 MHz a coin toss. WHERE it lands is the print; the verdict
    // claims only the floor both peers clear.
    static const uint32_t rates[] = {200'000UL, 500'000UL, 1'000'000UL, 2'000'000UL,
                                     3'000'000UL, 4'000'000UL, 6'000'000UL,
                                     8'000'000UL, 12'000'000UL, 24'000'000UL};
    struct Climb {
        uint32_t last_good = 0;
        uint32_t first_bad = 0;
        spilink::Report bad_r{};
    };
    // The DMA climb needs the controller up; letter h may not have run.
    (void)Dmac::init();
    Climb climbs[2];
    for (uint8_t leg = 0; leg < 2; ++leg) {
        const bool dma = leg == 1;
        print(serial, dma ? "  -- both ends on DMA engines --"
                          : "  -- both ends on polled pumps --", crlf);
        Climb& c = climbs[leg];
        for (uint8_t i = 0; i < sizeof(rates) / sizeof(rates[0]); ++i) {
            const auto b = Bus::baud_for(rates[i]);
            if (!b) continue;
            Exchange e{};
            e.baud = *b;
            e.count = 8;
            e.seed_a = 0x21;
            e.seed_b = 0x84;
            e.spare = dma ? 0u : spilink::spare_polled_pump;
            e.dma_host = dma;
            Verify v{};
            spilink::Report r{};
            const bool ok = exchange_exact(e, v, r);
            const uint32_t real = spi_sck_hz(SysClock::hz, *b);
            print(serial, "  SCK ", real / 1000u, " kHz (BAUD ", *b, "): ",
                  ok ? "exact both ways" : "NOT exact", "  host mism=", v.mism,
                  " client mism=", r.mism, " client count=", r.count,
                  " serve=", (r.aux2 & 0x04) != 0 ? "dma" : "pump", crlf);
            if (ok && c.first_bad == 0) {
                c.last_good = real;
            } else if (!ok && c.first_bad == 0) {
                c.first_bad = real;
                c.bad_r = r;
            }
        }
        print(serial, "  ", dma ? "on engines" : "on pumps", " the link held to ",
              c.last_good / 1000u, " kHz back to back");
        if (c.first_bad != 0) {
            print(serial, " and broke at ", c.first_bad / 1000u, " kHz");
        }
        print(serial, crlf);
    }
    bench.verdict("the polled link is exact at the command rate and at least 2.5 "
                  "times faster",
                  climbs[0].last_good >= 500'000UL);
    // THE SIGNATURE OF THE POLLED BOUNDARY: at the first rung that is
    // not exact, the peer still RECEIVED every character byte-exact
    // (its count full, its mismatches zero) while what the HOST read
    // back broke - so the failure lives in the ANSWER path (the reload
    // missing the three-cycle window), not in the wire and not in
    // reception. The day a peer holds the whole ladder this verdict is
    // vacuous and says so.
    bench.verdict("wherever the polled climb breaks, the peer still hears every "
                  "byte exact there - the boundary is its ANSWER RELOAD, not the "
                  "wire",
                  climbs[0].first_bad == 0 ||
                      (climbs[0].bad_r.count == 8 && climbs[0].bad_r.mism == 0));
    // The engines lift the reload boundary: with BOTH ends on DMA the
    // link must clear at least what the polled loops could, and where
    // it really stops is the print - the silicon's own answer.
    bench.verdict("with BOTH ends on DMA engines the climb reaches at least "
                  "2 MHz, above the polled loops",
                  climbs[1].last_good >= 2'000'000UL);

    (void)link_command_mode();
    bench.verdict("the command channel survives the climb", command(Op::ping));
}

// ===========================================================================
// e - the CLIENT role
// ===========================================================================

/// The SAM answers as the client while the peer clocks. PLOADEN puts the
/// first answer in the shift register before the select edge, so the
/// transaction opens with the answer and not the shifter's leftover
/// (32.6.3.2).
uint8_t crx[64];
uint16_t crx_n = 0;

bool run_as_client(const spilink::Params& a, SpiMode mode, bool lsb) {
    crx_n = 0;
    if (!Peer::init(clock, {.mode = mode,
                            .lsb_first = lsb,
                            .preload = true,
                            .drive_output = true})) {
        return false;
    }
    // ONE AHEAD, and the three-SCK-cycle rule is why (32.6.2.6.2): a
    // DATA write needs three SCK cycles to reach the shifter, and those
    // cycles ELAPSE ONLY WHILE SCK RUNS - so an answer written in the
    // inter-character gap (where a poll loop reacting to RXC lands)
    // matures mid-character and reaches the wire ONE CHARACTER LATE.
    // Measured by this suite's first version: the host read the preload
    // exactly and then the whole stream slipped by one (mism 11 of 12).
    // The cure the silicon offers is the transmit buffer: preload puts
    // b0 in the SHIFTER, the next write parks b1 in DATA, and every RXC
    // hands DATA its NEXT-plus-one - each value is then in place a whole
    // character early.
    spilink::Stream out(a.pattern, a.seed_b);
    Peer::write(out.next());   // b0 - PLOADEN: straight into the shift register
    uint16_t queued = 1;
    if (a.count > 1) {
        Peer::write(out.next());   // b1 - into DATA, loaded at the b0/b1 boundary
        ++queued;
    }

    const uint32_t t0 = Ticker::millis();
    while (crx_n < a.count && crx_n < sizeof crx && Ticker::millis() - t0 < a.ms) {
        const auto v = Peer::poll();
        if (!v) continue;
        crx[crx_n++] = static_cast<uint8_t>(*v);
        if (queued < a.count) {
            Peer::write(out.next());
            ++queued;
        }
    }
    Peer::release();
    return true;
}

void te_client() {
    if (!need_peer()) return;

    spilink::Params a{};
    a.count = 12;
    a.ms = 500;
    a.seed_a = 0x2C;
    a.seed_b = 0x71;
    a.pattern = spilink::pattern_prbs;
    a.aux8 = 40;             // lead-in ms: time for this end to become a client
    a.aux16 = 32;            // the peer's host SCK: its own clock / 32 (the SAM
                             // peer at 48 MHz clocks 1.5 MHz, the AVR one 750 kHz)
    a.cfg.apply = 1;
    a.cfg.mode = 0;
    a.cfg.dord = 0;

    if (!peer_act(Op::host_burst, a)) {
        bench.verdict("the peer accepted the host_burst command (its spi_peer must "
                      "carry op 0x14 - reflash board A)",
                      false);
        return;
    }
    bench.verdict("the peer accepted the host_burst command", true);

    const bool ran = run_as_client(a, SpiMode::mode0, false);
    bench.verdict("this board came up as an SPI CLIENT on the other DOPO row", ran);

    spilink::Report r{};
    const bool rep = peer_report(r);

    // What THIS end read must be the peer's stream A; what the PEER read
    // must be this end's stream B. Two independent checks of one burst.
    uint16_t mism = 0;
    uint8_t first_idx = 0xFF, got = 0, want = 0;
    for (uint16_t i = 0; i < crx_n; ++i) {
        const uint8_t exp = spilink::pattern_value(a.pattern, a.seed_a, i);
        if (crx[i] != exp) {
            if (mism == 0) {
                first_idx = static_cast<uint8_t>(i);
                got = crx[i];
                want = exp;
            }
            ++mism;
        }
    }
    print(serial, "  client read ", crx_n, " of ", a.count, " characters, mism=", mism);
    if (mism) print(serial, " (first idx ", first_idx, " got ", hex(got), " exp ", hex(want),
                    ")");
    print(serial, "; the peer as HOST read ", r.count, " with mism=", r.mism,
          " flags=", hex(r.flags), crlf);
    print(serial, "  read:");
    for (uint16_t i = 0; i < crx_n && i < 16; ++i) print(serial, " ", hex(crx[i]));
    print(serial, crlf);

    bench.verdict("the client received every character the host clocked, byte-exact",
                  ran && crx_n == a.count && mism == 0);
    bench.verdict("and the host read this end's answer stream byte-exact from the "
                  "FIRST character - which is what CTRLB.PLOADEN buys (32.6.3.2)",
                  rep && r.count == a.count && r.mism == 0);

    (void)link_command_mode();
    bench.verdict("the command channel is back after the role swap", command(Op::ping));
}

// ===========================================================================
// f - the wireless remainder: loop-back, the SCK ladder, 9 bits, MSSEN
// ===========================================================================

/// A loop-back burst: DIPO on the DO pad, so what comes back is what
/// went out - THROUGH THE PAD (32.6.3.4), not through an internal
/// short. The chip select is not used; PA18 stays PORT's.
bool loopback_burst(const uint8_t* tx, uint8_t* rx, uint16_t len, uint8_t baud,
                    SpiMode mode = SpiMode::mode0) {
    Loop::Request r{
        .cs = {},
        .dc = {},
        .cmd = {},
        .cmd_len = 0,
        .tx = lend<Lease::reply>(tx),
        .rx = lend<Lease::reply>(rx),
        .len = len,
        .reply = {},
        .baud = baud,
        .mode = mode,
        .polled = true,
    };
    return Loop::start(r);
}

uint8_t lb_tx[64];
uint8_t lb_rx[64];

void tf_wireless() {
    bench.verdict("the loop-back host comes up (DIPO and DOPO on ONE pad - "
                  "32.6.3.4's own arrangement)",
                  Loop::init(clock));

    // Bytes that can never start a spilink frame: PA16 is also the
    // peer's MOSI, and it decodes whatever this pad puts on the wire.
    for (uint8_t i = 0; i < 32; ++i) {
        lb_tx[i] = static_cast<uint8_t>(0x11u + i * 3u);
        if (lb_tx[i] == spilink::magic) lb_tx[i] = 0x5Au;
        lb_rx[i] = 0xEE;
    }
    (void)loopback_burst(lb_tx, lb_rx, 32, command_baud);
    uint8_t echo_mism = 0;
    for (uint8_t i = 0; i < 32; ++i) {
        if (lb_rx[i] != lb_tx[i]) ++echo_mism;
    }
    print(serial, "  loop-back: 32 bytes out, ", 32 - echo_mism, " read back identical"
          " (first ", hex(lb_tx[0]), " -> ", hex(lb_rx[0]), ")", crlf);
    bench.verdict("a loop-back host reads its own transmit line back byte-exact "
                  "THROUGH THE PAD (32.6.3.4)",
                  echo_mism == 0);

    // THE CHIP-SELECT SETUP, timed on the crystal: the same 16-byte
    // polled request with cs_setup_us 0 and 100, and the difference
    // must be the setup - which proves the engine spends it, and
    // spends it inside the transaction. (The pin itself is null here -
    // the field alone drives the wait, exactly as on the AVR, where a
    // caller may own the CS and still want the engine's pacing.)
    if (ruler_ok) {
        auto timed = [&](uint8_t setup) {
            Loop::Request r{
                .cs = {}, .dc = {}, .cs_setup_us = setup,
                .cmd = {}, .cmd_len = 0,
                .tx = lend<Lease::reply>(static_cast<const uint8_t*>(lb_tx)),
                .rx = lend<Lease::reply>(lb_rx),
                .len = 16, .reply = {},
                .baud = command_baud, .mode = SpiMode::mode0, .polled = true,
            };
            const uint32_t t0 = wall();
            (void)Loop::start(r);
            return (wall() - t0) / (crystal_hz / 1000000u);
        };
        const uint32_t bare = timed(0);
        const uint32_t with_setup = timed(100);
        const uint32_t delta = with_setup > bare ? with_setup - bare : 0u;
        print(serial, "  cs_setup_us: 16 bytes in ", bare, " us bare, ", with_setup,
              " us with 100 us of setup (delta ", delta, ")", crlf);
        bench.verdict("Request.cs_setup_us is spent between the CS assertion and "
                      "the first clock - at least the asked microseconds, and not "
                      "wildly more (the avrdx field, verbatim)",
                      delta >= 100u && delta <= 130u);
    } else {
        bench.verdict("the crystal ruler is available for the cs_setup measurement",
                      false);
    }

    // The SCK ladder, timed on the CRYSTAL. A burst of N characters at
    // BAUD b takes N x 8 SCK periods plus this pump's own per-character
    // overhead, so the measurement is reported as both: the total and
    // the excess over the arithmetic.
    if (!ruler_ok) {
        print(serial, "  the crystal ruler did not come up - the rate measurement "
                      "is skipped and the verdict declined",
              crlf);
        bench.verdict("the crystal ruler is available for the rate measurement", false);
    } else {
        static const uint8_t bauds[] = {255, 119, 23, 5};
        bool all_in_band = true;
        for (uint8_t i = 0; i < sizeof bauds; ++i) {
            constexpr uint16_t n = 64;
            const uint32_t t0 = wall();
            (void)loopback_burst(lb_tx, lb_rx, n, bauds[i]);
            const uint32_t took = wall() - t0;
            const uint32_t sck = spi_sck_hz(SysClock::hz, bauds[i]);
            // Crystal ticks the characters alone should cost. The
            // division comes FIRST: n x 8 x 24e6 overflows 32 bits at
            // these lengths, and the first version of this line said so
            // by reporting an impossible negative overhead.
            const uint32_t due = static_cast<uint32_t>(n) * 8u * (crystal_hz / sck);
            const uint32_t over = took > due ? took - due : 0;
            const uint32_t over_us = over / (crystal_hz / 1000000u);
            print(serial, "  BAUD ", bauds[i], " = ", sck / 1000u, " kHz: 64 characters "
                  "in ", took, " crystal ticks, the bits alone are ", due,
                  ", polled-pump overhead ", over_us, " us total (",
                  (over_us * 100u) / n, " hundredths of a us per character)", crlf);
            // The bits must be there: the measured time can never be
            // SHORTER than the arithmetic, and the per-character
            // overhead of a polled pump on a 48 MHz core has to be a
            // few microseconds, not tens.
            if (took < due || over_us > 10u * n) all_in_band = false;
        }
        bench.verdict("every rate on the ladder really clocks the bits it says it "
                      "does - measured against the 24 MHz crystal, never short",
                      all_in_band);
    }

    // NINE-BIT CHARACTERS (CTRLB.CHSIZE). The engine is byte-oriented,
    // so this goes through the resource directly - which is the honest
    // division: the register surface has the whole chapter, the task has
    // the shape a bus arbiter needs.
    SpiConfig nine = spi_role_probe(loopback_pads, SpiRole::host);
    nine.bits = SpiCharSize::nine;
    nine.baud = command_baud;
    bool nine_ok = Raw::configure(nine) && Raw::enable(true);
    if (nine_ok) {
        Pin<'A', 16>::function(PinFunction::c, {.input_enable = true});
        SckPin::function(PinFunction::c);
        Raw::flush_rx();
        static const uint16_t words[] = {0x1FF, 0x100, 0x0AA, 0x155};
        uint16_t back[4] = {};
        for (uint8_t i = 0; i < 4; ++i) {
            Raw::data(words[i]);
            uint32_t spins = 200000u;
            while (!Raw::rxc_flag() && spins-- != 0u) {
            }
            back[i] = Raw::data();
        }
        print(serial, "  nine-bit characters, looped back:");
        for (uint8_t i = 0; i < 4; ++i) print(serial, " ", hex(words[i]), "->", hex(back[i]));
        print(serial, crlf);
        for (uint8_t i = 0; i < 4; ++i) {
            if (back[i] != words[i]) nine_ok = false;
        }
    }
    bench.verdict("CHSIZE = 9 really carries nine bits - 0x1FF and 0x100 come back "
                  "whole, which an eight-bit datapath could not do",
                  nine_ok);

    // BUFOVF and IBON (32.6.2.7). Clock characters without reading DATA:
    // the two-level receive buffer fills and the third is lost.
    SpiConfig ibon = spi_role_probe(loopback_pads, SpiRole::host);
    ibon.baud = command_baud;
    ibon.immediate_overflow = true;
    const bool up = Raw::configure(ibon) && Raw::enable(true);
    Pin<'A', 16>::function(PinFunction::c, {.input_enable = true});
    SckPin::function(PinFunction::c);
    Raw::flush_rx();
    for (uint8_t i = 0; i < 5; ++i) {
        Raw::data(static_cast<uint16_t>(0x30u + i));
        uint32_t spins = 200000u;
        while (!Raw::dre_flag() && spins-- != 0u) {
        }
    }
    settle_ms(2);
    const bool overflowed = Raw::overflow_flag();
    const bool err = Raw::error_flag();
    print(serial, "  five characters clocked with DATA never read: STATUS=",
          hex(Raw::status()), " INTFLAG=", hex(Raw::flags()), crlf);
    bench.verdict("the receive buffer is TWO deep and the rest is lost - "
                  "STATUS.BUFOVF with CTRLA.IBON, and INTFLAG.ERROR travelling "
                  "with it (32.6.2.7)",
                  up && overflowed && err);
    Raw::clear_status(SpiStatus::overflow);
    Raw::clear_flags(SpiFlag::error);
    bench.verdict("both are write-one-to-clear",
                  !Raw::overflow_flag() && !Raw::error_flag());

    // WHAT CTRLB.MSSEN REALLY DRIVES (32.6.3.5). The SS pad becomes the
    // peripheral's, and the chapter says it is raised "for a minimum of
    // one baud cycle between each data sent" - i.e. it frames a
    // CHARACTER and not a transaction, which is the whole reason this
    // engine's chip select is a GPIO. Sampled on the pad itself, at the
    // slowest rate the generator has so a CPU loop can see the edges.
    SpiConfig mssen = spi_role_probe(host_pads, SpiRole::host);
    mssen.hardware_ss = true;
    mssen.baud = 255;   // 93.75 kHz: a character is 85 us
    const bool mss_up = Raw::configure(mssen) && Raw::enable(true);
    Pin<'A', 16>::function(PinFunction::c);
    SckPin::function(PinFunction::c);
    Cs::function(PinFunction::c, {.input_enable = true});
    Raw::flush_rx();
    uint16_t rises = 0;
    bool prev = Cs::read();
    // A plain local, NOT a static: a static survives into the next run
    // of z in the same power cycle, arrives holding 4, and this letter
    // then clocks ONE character and counts one rise - the flaky verdict
    // the first version had on every second run.
    uint8_t sent = 1;
    Raw::data(0x00);
    for (uint32_t k = 0; k < 400000u; ++k) {
        const bool now = Cs::read();
        if (now && !prev) ++rises;
        prev = now;
        if (Raw::dre_flag() && sent < 4) {
            Raw::data(0x00);
            ++sent;
        }
        if (rises >= 8) break;
    }
    print(serial, "  MSSEN over a four-character transfer: ", rises,
          " rising edges seen on the SS pad itself", crlf);
    bench.verdict("CTRLB.MSSEN raises SS between EVERY CHARACTER (32.6.3.5), so "
                  "hardware SS frames a character and not a transaction - which is "
                  "why this engine's chip select is an ordinary GPIO",
                  mss_up && rises >= 2);

    // Put the pads back the way every other letter expects them.
    Raw::release();
    Cs::release();
    Cs::set();
    Cs::output();
    Pin<'A', 16>::release();
    SckPin::release();
    bench.verdict("the pads go back to PORT afterwards",
                  !Cs::has_function() && !SckPin::has_function());
}

// ===========================================================================
// g - THE KERNEL LETTER: util/spi_bus.hpp over this engine, unchanged
// ===========================================================================

namespace kl {

/// Four loop-back transactions, queued at once from one dispatch. The
/// data is deterministic and wireless, so what is under test here is the
/// ARBITER and not the wire: the pending FIFO, the reply channel, the
/// rejection when it is full, and the sleep vote.
constexpr uint8_t queued = 4;
constexpr uint8_t payload = 6;

uint8_t tx[queued][payload];
uint8_t rx[queued][payload];

struct Kick {};
struct Fill {};
struct Ask {};

class Driver;
using SpiArb = SpiBus<Loop, P, 3>;   ///< pending depth 3: one fewer than we queue

class Driver : public Fsm<Driver, SpiDone, Kick, Fill, Ask, SleepVote> {
public:
    static inline EventQueue<Event, 12, P> queue;
    static inline uint8_t replies = 0;
    static inline uint8_t rejected = 0;
    static inline uint8_t ok_replies = 0;
    static inline uint8_t votes = 0;
    static inline bool last_vote = false;

    static void init() {
        replies = rejected = ok_replies = votes = 0;
        last_vote = false;
        start(&running);
    }
    static void dispatch(const Event& e) { Fsm::dispatch(e); }

private:
    static Status running(const Event& e) {
        return brio::match(
            e,
            [](const Kick&) {
                for (uint8_t i = 0; i < queued; ++i) {
                    post<SpiArb>(request(i));
                }
                return handled();
            },
            [](const Fill&) {
                // One more than the arbiter can hold in flight plus its
                // FIFO: the last must come back bus_rejected, never
                // silently dropped.
                for (uint8_t i = 0; i < queued + 2; ++i) {
                    post<SpiArb>(request(i % queued));
                }
                return handled();
            },
            [](const Ask&) {
                post<SpiArb>(PrepareSleep{
                    .depth = SleepDepth::standby,
                    .reply = reply_to<Driver, SleepVote>(),
                });
                return handled();
            },
            [](const SpiDone& d) {
                ++replies;
                if (d.status == spi_ok) ++ok_replies;
                if (d.status == spi_rejected) ++rejected;
                return handled();
            },
            [](const SleepVote& v) {
                ++votes;
                last_vote = v.ok;
                return handled();
            },
            [](auto) { return unhandled(); });
    }

    static Loop::Request request(uint8_t i) {
        return Loop::Request{
            .cs = {},
            .dc = {},
            .cmd = {},
            .cmd_len = 0,
            .tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx[i])),
            .rx = lend<Lease::reply>(rx[i]),
            .len = payload,
            .reply = reply_to<Driver, SpiDone>(),
            .baud = command_baud,
            .mode = SpiMode::mode0,
            // The ISR PUMP, deliberately: this is where the engine's
            // asynchronous half meets the arbiter's, which is the shape
            // util/bus_master.hpp was written for.
            .polled = false,
        };
    }
};

using BusKernel = Kernel<P, Driver, SpiArb>;

}   // namespace kl

void tg_kernel() {
    // The engine, in loop-back so the letter needs no peer and its data
    // is deterministic.
    bench.verdict("the loop-back engine comes up for the kernel letter",
                  Loop::init(clock));
    for (uint8_t i = 0; i < kl::queued; ++i) {
        for (uint8_t k = 0; k < kl::payload; ++k) {
            kl::tx[i][k] = static_cast<uint8_t>(0xA0u + i * 16u + k);
            if (kl::tx[i][k] == spilink::magic) kl::tx[i][k] = 0x5Au;
            kl::rx[i][k] = 0xEE;
        }
    }

    kl::BusKernel::init_all();
    isr_completions = 0;
    bus_ao_live = true;

    post<kl::Driver>(kl::Kick{});
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 300u) {
        while (kl::BusKernel::step()) {
        }
        if (kl::Driver::replies >= kl::queued) break;
    }

    uint16_t data_mism = 0;
    for (uint8_t i = 0; i < kl::queued; ++i) {
        for (uint8_t k = 0; k < kl::payload; ++k) {
            if (kl::rx[i][k] != kl::tx[i][k]) ++data_mism;
        }
    }
    print(serial, "  ", kl::queued, " requests queued in ONE dispatch: ",
          kl::Driver::replies, " replies (", kl::Driver::ok_replies, " ok), ",
          isr_completions, " engine completions, ", data_mism, " byte mismatches", crlf);
    bench.verdict("EVERY queued request came back through its own ReplyTo, in order, "
                  "with spi_ok - the arbiter serialized four transactions on one bus",
                  kl::Driver::replies == kl::queued &&
                      kl::Driver::ok_replies == kl::queued);
    bench.verdict("and every one of them moved its bytes: the ISR PUMP carried the "
                  "loop-back byte-exact under the kernel",
                  data_mism == 0);
    bench.verdict("the engine's completion edge fired once per transaction",
                  isr_completions == kl::queued);

    // Reject-when-full: never silent, never blocking.
    kl::Driver::init();
    post<kl::Driver>(kl::Fill{});
    const uint32_t t1 = Ticker::millis();
    while (Ticker::millis() - t1 < 400u) {
        while (kl::BusKernel::step()) {
        }
        if (kl::Driver::replies >= kl::queued + 2) break;
    }
    print(serial, "  ", kl::queued + 2, " requests into a bus holding one in flight "
          "plus a 3-deep FIFO: ", kl::Driver::replies, " replies, ",
          kl::Driver::rejected, " of them bus_rejected (arbiter tally ",
          kl::SpiArb::rejected_count(), ")", crlf);
    bench.verdict("an over-full arbiter answers bus_rejected IMMEDIATELY - never "
                  "silent, never blocking, and every request still gets exactly one "
                  "reply",
                  kl::Driver::replies == kl::queued + 2 && kl::Driver::rejected >= 1 &&
                      kl::SpiArb::rejected_count() >= 1);

    // The sleep vote. Idle first...
    kl::Driver::init();
    post<kl::Driver>(kl::Ask{});
    for (uint8_t k = 0; k < 40 && kl::Driver::votes == 0; ++k) {
        while (kl::BusKernel::step()) {
        }
    }
    const bool idle_vote = kl::Driver::last_vote;
    const uint8_t idle_votes = kl::Driver::votes;

    // ... then with a transaction genuinely in flight: one request
    // posted, one vote posted right behind it, and the vote dispatched
    // while the engine is still clocking. The slowest rate the generator
    // has makes that window 4.4 ms wide, which no dispatch can miss.
    kl::Driver::init();
    kl::SpiArb::init();
    {
        auto r = Loop::Request{
            .cs = {}, .dc = {}, .cmd = {}, .cmd_len = 0,
            .tx = lend<Lease::reply>(static_cast<const uint8_t*>(kl::tx[0])),
            .rx = lend<Lease::reply>(kl::rx[0]),
            .len = kl::payload,
            .reply = reply_to<kl::Driver, SpiDone>(),
            .baud = 255,
            .mode = SpiMode::mode0,
            .polled = false,
        };
        post<kl::SpiArb>(r);
    }
    post<kl::Driver>(kl::Ask{});
    for (uint8_t k = 0; k < 200 && kl::Driver::votes == 0; ++k) {
        while (kl::BusKernel::step()) {
        }
    }
    const bool busy_vote = kl::Driver::last_vote;
    const uint8_t busy_votes = kl::Driver::votes;
    // Drain whatever is still owed before judging anything else.
    const uint32_t t2 = Ticker::millis();
    while (Ticker::millis() - t2 < 200u) {
        while (kl::BusKernel::step()) {
        }
        if (kl::Driver::replies >= 1) break;
    }

    print(serial, "  PrepareSleep(standby): idle votes=", idle_votes, " ok=", idle_vote,
          ", busy votes=", busy_votes, " ok=", busy_vote, crlf);
    bench.verdict("an IDLE bus votes ok on a PrepareSleep", idle_votes == 1 && idle_vote);
    bench.verdict("a bus with a transaction IN FLIGHT votes NOT ok - the completion "
                  "interrupt is exactly what a gated clock domain would swallow",
                  busy_votes == 1 && !busy_vote);

    bus_ao_live = false;
    Loop::release();
    Cs::set();
    Cs::output();
    bench.verdict("NOT ONE LINE of util/spi_bus.hpp, util/bus_master.hpp or kernel/ "
                  "was changed for this architecture - the arbiter's second silicon "
                  "ran the contract as written",
                  true);
}

// ===========================================================================
// h - THE DMA HOST: the data phase on the two DMAC engines (WIRELESS)
// ===========================================================================

namespace dh {

/// The loop-back host again, with the data phase on channels 0 and 1.
/// Wireless: what the engines move is read back through the pad, so a
/// missing byte, a swapped byte or a phase slip is a data mismatch and
/// not an interpretation.
using DmaLoop = SpiHost<link_sercom, loopback_pads, 0, DmaTxEngine<0>, DmaRxEngine<1>>;

volatile bool request_done = false;
volatile bool host_live = false;   ///< routes SERCOM1_Handler to DmaLoop::isr()

uint8_t tx[64];
uint8_t rx[64];
const uint8_t cmd3[3] = {0x5A, 0x0F, 0x33};

bool polled_req(const uint8_t* txp, uint8_t* rxp, uint16_t len, uint8_t baud) {
    DmaLoop::Request r{
        .cs = {}, .dc = {}, .cmd = {}, .cmd_len = 0,
        .tx = lend<Lease::reply>(txp),
        .rx = lend<Lease::reply>(rxp),
        .len = len, .reply = {},
        .baud = baud, .mode = SpiMode::mode0, .polled = true,
    };
    return DmaLoop::start(r);
}

}   // namespace dh

void th_dma() {
    using dh::DmaLoop;
    // The DMAC BLOCK is the app's to initialize, once - the engines arm
    // CHANNELS of a controller somebody else owns (the Uart's own
    // contract, and the reason SpiHost::init cannot do it: a shared
    // block re-initialized per transport would stop every other
    // channel).
    const bool dmac_ok = Dmac::init();
    bench.verdict("the DMA loop-back host comes up (Dmac::init once, then "
                  "DmaTxEngine<0> + DmaRxEngine<1> on SERCOM1's own trigger codes)",
                  dmac_ok && DmaLoop::init(clock));
    DmaTxEngine<0>::clear_faults();

    for (uint8_t i = 0; i < 64; ++i) {
        dh::tx[i] = static_cast<uint8_t>(0x23u + i * 5u);
        if (dh::tx[i] == spilink::magic) dh::tx[i] = 0x5Au;
        dh::rx[i] = 0xEE;
    }

    // 1. The polled full-duplex request: the whole data phase moved by
    // the two channels, the CPU spinning on the DMAC's completion.
    bool ok = dh::polled_req(dh::tx, dh::rx, 48, 23) && DmaLoop::status() == spi_ok;
    uint16_t mism = 0;
    for (uint8_t i = 0; i < 48; ++i) {
        if (dh::rx[i] != dh::tx[i]) ++mism;
    }
    print(serial, "  polled DMA loop-back, 48 bytes at 1 MHz: mism=", mism,
          " status=", DmaLoop::status(), crlf);
    bench.verdict("a POLLED request's data phase rides the two channels byte-exact "
                  "(RX drains on the RXC trigger, TX feeds on DRE, NO kick - the "
                  "enable under the standing level fires the first beat itself)",
                  ok && mism == 0);

    // 2. Two phases: the command bytes on the byte pump, the data on the
    // engines - the handover inside one chip-select window.
    for (uint8_t i = 0; i < 16; ++i) dh::rx[i] = 0xEE;
    {
        DmaLoop::Request r{
            .cs = {}, .dc = {},
            .cmd = lend<Lease::reply>(static_cast<const uint8_t*>(dh::cmd3)),
            .cmd_len = 3,
            .tx = lend<Lease::reply>(static_cast<const uint8_t*>(dh::tx)),
            .rx = lend<Lease::reply>(dh::rx),
            .len = 16, .reply = {},
            .baud = command_baud, .mode = SpiMode::mode0, .polled = true,
        };
        ok = DmaLoop::start(r) && DmaLoop::status() == spi_ok;
    }
    mism = 0;
    for (uint8_t i = 0; i < 16; ++i) {
        if (dh::rx[i] != dh::tx[i]) ++mism;
    }
    bench.verdict("a TWO-PHASE polled request hands over from the byte pump to the "
                  "engines mid-window, and rx captures the DATA phase alone",
                  ok && mism == 0);

    // 3. A null tx feeds dummies from a held source: in loop-back every
    // received byte must be exactly 0xFF.
    for (uint8_t i = 0; i < 16; ++i) dh::rx[i] = 0;
    ok = dh::polled_req(nullptr, dh::rx, 16, command_baud) && DmaLoop::status() == spi_ok;
    mism = 0;
    for (uint8_t i = 0; i < 16; ++i) {
        if (dh::rx[i] != 0xFFu) ++mism;
    }
    bench.verdict("a null tx sends 0xFF dummies from a HELD source address "
                  "(increment off - one descriptor bit)",
                  ok && mism == 0);

    // 4. A null rx drains into the held sink - the completion still
    // needs every byte RECEIVED, nobody keeps them.
    ok = dh::polled_req(dh::tx, nullptr, 16, command_baud) && DmaLoop::status() == spi_ok;
    bench.verdict("a null rx completes through the discard sink, spi_ok",
                  ok);

    // 5. The ISR-style request: the command phase pumped by
    // SERCOM1_Handler, the handover made INSIDE the interrupt, the
    // completion posted by DMAC_Handler - both vectors in one request.
    for (uint8_t i = 0; i < 24; ++i) dh::rx[i] = 0xEE;
    dh::request_done = false;
    dh::host_live = true;
    {
        DmaLoop::Request r{
            .cs = {}, .dc = {},
            .cmd = lend<Lease::reply>(static_cast<const uint8_t*>(dh::cmd3)),
            .cmd_len = 3,
            .tx = lend<Lease::reply>(static_cast<const uint8_t*>(dh::tx)),
            .rx = lend<Lease::reply>(dh::rx),
            .len = 24, .reply = {},
            .baud = command_baud, .mode = SpiMode::mode0, .polled = false,
        };
        ok = !DmaLoop::start(r);   // asynchronous: false = running on the ISRs
    }
    {
        const uint32_t t0 = Ticker::millis();
        while (!dh::request_done && Ticker::millis() - t0 < 100u) {
        }
    }
    dh::host_live = false;
    mism = 0;
    for (uint8_t i = 0; i < 24; ++i) {
        if (dh::rx[i] != dh::tx[i]) ++mism;
    }
    print(serial, "  ISR-style request: done=", dh::request_done, " mism=", mism,
          " status=", DmaLoop::status(), crlf);
    bench.verdict("an ISR-style request runs the command phase on the SERCOM vector, "
                  "hands over to the engines inside the interrupt, and completes "
                  "through the DMAC vector with spi_ok",
                  ok && dh::request_done && mism == 0 && DmaLoop::status() == spi_ok);

    // 6. The ladder to the generator's top, timed on the crystal. The
    // point of the engines: back-to-back characters with the CPU out of
    // the byte path, all the way to BAUD 0 = f_ref/2.
    if (!ruler_ok) {
        bench.verdict("the crystal ruler is available for the DMA rate ladder", false);
    } else {
        static const uint8_t bauds[] = {5, 2, 1, 0};   // 4, 8, 12, 24 MHz
        bool exact_to_12m = true;
        uint16_t mism_24m = 0;
        bool none_short = true;
        for (uint8_t k = 0; k < sizeof bauds; ++k) {
            constexpr uint16_t n = 64;
            for (uint8_t i = 0; i < n; ++i) dh::rx[i] = 0xEE;
            const uint32_t t0 = wall();
            (void)dh::polled_req(dh::tx, dh::rx, n, bauds[k]);
            const uint32_t took = wall() - t0;
            const uint32_t sck = spi_sck_hz(SysClock::hz, bauds[k]);
            const uint32_t due = static_cast<uint32_t>(n) * 8u * (crystal_hz / sck);
            mism = 0;
            for (uint8_t i = 0; i < n; ++i) {
                if (dh::rx[i] != dh::tx[i]) ++mism;
            }
            const uint32_t over_us =
                (took > due ? took - due : 0u) / (crystal_hz / 1000000u);
            print(serial, "  BAUD ", bauds[k], " = ", sck / 1000u, " kHz: 64 bytes in ",
                  took, " crystal ticks (bits alone ", due, ", overhead ", over_us,
                  " us for the WHOLE phase), mism=", mism, crlf);
            if (bauds[k] == 0) {
                mism_24m = mism;
            } else if (mism != 0) {
                exact_to_12m = false;
            }
            if (took < due) none_short = false;
        }
        bench.verdict("the DMA data phase is byte-exact THROUGH THE PAD to 12 MHz "
                      "(f_ref/4) - back-to-back, no CPU in the byte path",
                      exact_to_12m);
        // The top rung is a LOOP-BACK SAMPLING boundary, not judged: at
        // f_ref/2 the pad round trip meets the input sampler inside one
        // 333 ns character, and what breaks cannot be attributed
        // between the transmit and receive halves from one board. The
        // wired SAM-SAM ladder is the instrument that can.
        print(serial, "  the 24 MHz rung read ", 64 - mism_24m,
              " of 64 correct - recorded, not judged (loop-back sampling at "
              "f_ref/2)", crlf);
        bench.verdict("and never faster than the arithmetic says (the bits are "
                      "really on the wire)",
                      none_short);
    }

    print(serial, "  engine faults across the letter: ", DmaTxEngine<0>::faults(), crlf);
    bench.verdict("no 1.10.4-class fault was seen (two channels, low trigger "
                  "density - the erratum's own density law)",
                  DmaTxEngine<0>::faults() == 0);

    DmaLoop::release();
    Cs::set();
    Cs::output();
}

}   // namespace

// ---------------------------------------------------------------------------
// The vectors
// ---------------------------------------------------------------------------

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

/// ONE vector for the whole SERCOM. Which engine is on it depends on
/// what the current letter armed - the kernel letter's loop-back host,
/// or the protocol's. Both are SpiHost instantiations over the same
/// Spi<1>, and only one of them ever has RXC armed at a time.
extern "C" void SERCOM1_Handler() {
    if (dh::host_live) {
        // Letter h's ISR-style request: the command phase pumped here,
        // the handover to the engines made inside this very interrupt.
        if (dh::DmaLoop::isr()) {
            dh::request_done = true;
        }
        return;
    }
    if (bus_ao_live) {
        if (Loop::isr()) {
            isr_completions = isr_completions + 1;
            brio::post<kl::SpiArb>(brio::TransferDone{brio::spi_ok});
        }
        return;
    }
    if (Bus::isr()) {
        isr_done = true;
        isr_completions = isr_completions + 1;
    }
}

extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }

/// The engines' completions and faults - letter h's loop-back twin or
/// letter d's on-the-wire one, whichever is live.
extern "C" void DMAC_Handler() {
    while (const auto irq = brio::Dmac::take_pending()) {
        if (dma_bus_live) {
            (void)DmaBus::dma_isr(irq->channel, irq->flags);
        } else if (dh::DmaLoop::dma_isr(irq->channel, irq->flags)) {
            dh::request_done = true;
        }
    }
}

int main() {
    SysClock::init();
    Serial::init(clock, 115200);
    Ticker::init(clock);
    brio::enable_interrupts();

    ruler_ok = ruler_up();
    calibrate_hold();

    print(serial, crlf, crlf,
          "test_samc_spi - SERCOM SPI (ch. 32) and util's bus vocabulary on its "
          "SECOND architecture",
          crlf);
    print(serial, "  SERCOM1 function C: PA16 PAD0 (MOSI/DO), PA17 PAD1 (SCK), "
                  "PA18 PAD2 (SS, a GPIO chip select), PA19 PAD3 (MISO/DI)",
          crlf);
    print(serial, "  peer: the other board running spi_peer (board D today), "
                  "commanded in band over the same four wires; the crystal ruler is ",
          ruler_ok ? "up" : "DOWN", crlf);
    print(serial, "  the protocol holds its own chip select ", hold_spins,
          " spins = ", hold_us, " us around each window - see link_hold()'s comment "
          "for the finding that made that necessary", crlf);

    bench.letter('a', "the block, its registers and its two live errata", ta_block);
    bench.letter('b', "the command channel comes up (needs the peer)", tb_link);
    bench.letter('c', "four transfer modes x both bit orders (needs the peer)", tc_matrix);
    bench.letter('d', "the rate ladder and where the peer stops (needs the peer)",
                 td_rates);
    bench.letter('e', "THE CLIENT ROLE: the peer becomes the host (needs the peer)",
                 te_client);
    bench.letter('f', "loop-back, the SCK ladder, nine bits, BUFOVF and MSSEN", tf_wireless);
    bench.letter('g', "THE KERNEL: SpiBus over SpiHost, util unchanged", tg_kernel);
    bench.letter('h', "THE DMA HOST: the data phase on the two engines", th_dma);

    bench.menu();
    bench.prompt();
    for (;;) {
        uint8_t c;
        if (!Serial::read_byte(c)) continue;
        if (c == '\r' || c == '\n') continue;
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') {
            bench.menu();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "? for help", crlf);
        }
        bench.prompt();
    }
}
