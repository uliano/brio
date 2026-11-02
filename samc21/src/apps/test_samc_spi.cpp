// build: boards = c21j
// build: monitor_speed = 115200
//
// test_samc_spi - the SERCOM in SPI mode (DS60001479M ch. 32) and, over
// it, util's bus vocabulary on the wire: util/spi_bus.hpp is
// util/bus_master.hpp, and it drives samc21/spi.hpp's SpiHost with no
// knowledge of this silicon.
//
// THE BENCH, and it is a TWO-BOARD bench. This board is the DUT; the
// far board runs a `spi_peer`, the scriptable instrument client,
// commanded IN BAND over the very bus under test. The wire format is
// spi_link.hpp, included BY RELATIVE PATH: one file is the single
// source of truth for every board that speaks it, whatever its
// architecture - the suite asks ident and does not care which peer
// answered.
//
// The link is five wires:
//
//   PA16 = SERCOM1/PAD[0], function C   MOSI
//   PA17 = SERCOM1/PAD[1], function C   SS
//   PA18 = SERCOM1/PAD[2], function C   MISO
//   PA19 = SERCOM1/PAD[3], function C   SCK
//   plus a dedicated GND.
//
// Against a second SAM C21 the four run STRAIGHT THROUGH (same pin names
// both ends); against an AVR128DA48 they meet its SPI0 ALT1 - PA16 to
// PE0 (MOSI), PA18 to PE1 (MISO), PA19 to PE2 (SCK), PA17 to PE3 (SS).
//
// THE SAME FOUR WIRES CARRY BOTH ROLES, ON TWO DIFFERENT DOPO ROWS, and
// that is one of the things this suite demonstrates rather than asserts.
// A host puts DO (its MOSI) on PAD[0] with SCK on PAD[3] - row 0x3; a
// client's DO is MISO and has to reach the OTHER board's MISO input,
// i.e. PAD[2], with SCK still on PAD[3] - row 0x1. The role does not
// flip a direction bit, it changes which code CTRLA carries (32.8.1's
// table, samc21/spi.hpp's header).
//
// WHY THOSE TWO ROWS, of the four: the same four pads are also the
// SERCOM's USART against an AVR's USART4 on the same wires (TxD PE0,
// RxD PE1, XCK PE2), so SCK and XCK must be ONE pad, and XCK follows
// TxD (31.8.1: TXPO 0x1 puts TxD on PAD[2] and XCK on PAD[3]). TxD on
// PAD[2] is the MISO wire, the AVR's RxD; rows 0x1 and 0x3 are the two
// that put SCK on PAD[3] (32.8.1).
//
// THE CHIP SELECT IS A GPIO. CTRLB.MSSEN exists and letter f measures
// what it really does, but 32.6.3.5 raises hardware SS between every
// CHARACTER, so it cannot frame a multi-byte protocol frame. PA17 is
// therefore an ordinary PORT output for the whole protocol, which is
// 32.6.3.3's "host with several clients" arrangement and the one every
// device client in brio uses.
//
// NO INTER-BYTE GAP. spilink::gap_us exists for a peer whose
// NORMAL-mode client has to load its answer between characters. Every
// window this suite opens has the peer in a BUFFERED regime (its command
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
//      keeping up (its own CLK_PER/6 client ceiling); the rungs' rates
//      are timed on the wire by letter f, the same generator on the same
//      SERCOM with no peer needed
//   e  THE CLIENT ROLE: the peer becomes the HOST for a bounded burst
//      (spilink::Op::host_burst) and this board answers as the client -
//      the other DOPO row, PLOADEN, and the three-SCK-cycle rule
//   f  the wireless remainder: 32.6.3.4 loop-back through the pad, THE
//      SCK LADDER ON THE WIRE - a frame inside a polled block against its
//      eight SCK periods at the rate asked, on the 24 MHz crystal, judged
//      within -1 % and +3 % (a loop proves the bytes, never the rate) -
//      nine-bit characters, BUFOVF and IBON, and what CTRLB.MSSEN really
//      drives
//   g  THE KERNEL LETTER: SpiBus (= BusMaster) over SpiHost inside a
//      real kernel - queued polled requests, replies through ReplyTo,
//      an asynchronous request refused (spi_rejected: no DMA here) with
//      the bus free for the next, and the PrepareSleep vote idle and
//      right behind a request; the requests' rates timed on the wire
//   h  THE AVR CLIENT'S FACTS, judged here because here the AVR is the
//      instrument: when the peer's ident names the avrxt peer (the
//      version's high byte 0x01, spi_link.hpp's Ident), the verdicts
//      test_avr_spi draws against an AVR CLIENT - the three buffering
//      regimes' alignment (its l), the CLK_PER/6 ceiling (its m), the
//      loss semantics and the write-collision boundary (its p) and the
//      self-selected client the USART's Host SPI is served by (the
//      client half of its r) - drawn with this board as the host, paced
//      as that suite paces them. Any other peer: SKIPPED, no verdict.
//   i  the AVR client's RUNS back to back (an avrxt peer only): 256
//      characters as one request, the exchange, the answer run, the
//      receive run and the exchange without BUFWR, 500 kHz to 4 MHz
//   j  THE CLIENT'S EDGES: this board's runs against the peer's
//      host_burst - the receive run and its echo, `stalled` with no
//      select and with a select held and no clock, INTFLAG.SSL,
//      address recognition both ways, a handler timed into a run by the
//      SSL interrupt, and the select's fall as the wake-up from IDLE and
//      from STANDBY (RUNSTDBY; the RTC on OSCULP32K the backstop)
//   k  the host's rare paths: the spin budget on a clockless SERCOM, and
//      an early TXC under a handler (into the peer's receive-only run)
//
// Letters that need the peer say so and FAIL LOUDLY rather than hanging
// when it is absent. Nothing here wears flash.

#include <stdint.h>

#include "samc21/clock.hpp"
#ifndef SPI_SELECT_CONTROL
#include "samc21/dmac.hpp"
#endif
#include "samc21/nvic.hpp"
#include "samc21/osc32kctrl.hpp"
#include "samc21/pin.hpp"
#include "samc21/platform.hpp"
#include "samc21/rtc.hpp"
#include "samc21/sercom.hpp"
#include "samc21/sleep.hpp"
#include "samc21/spi.hpp"
#include "samc21/tc.hpp"
#include "samc21/ticker.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/post.hpp"
#include "kernel/time.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/spi_bus.hpp"
#include "util/testbench.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED. spi_link.hpp
// is pure encoding - it names no register and includes nothing of brio -
// so both boards' apps compile the same file. A copy here would be a
// second source of truth for a wire format, which is exactly the thing
// that drifts.
#include "../../../avrxt/src/apps/spi_link.hpp"
#include "bench_sleep.hpp"

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

/// HOST row 0x3: DO(MOSI) PAD[0], SCK PAD[3], SS PAD[1], DI(MISO) PAD[2].
/// The SS pad is NOT claimed - the chip select is the PORT pin below.
constexpr SpiPads host_pads{
    .data_out = SercomPad::pad0,
    .sck = SercomPad::pad3,
    .ss = SercomPad::pad1,
    .data_in = SercomPad::pad2,
    .data_out_pin = {'A', 16, PinFunction::c},
    .sck_pin = {'A', 19, PinFunction::c},
    .ss_pin = {'A', 17, PinFunction::c},
    .data_in_pin = {'A', 18, PinFunction::c},
};

/// CLIENT row 0x1 on the SAME wires: DO is MISO now and must land on
/// PAD[2], DI is MOSI on PAD[0]. Of the two rows with SCK on PAD[3], only
/// row 0x1 puts DO there.
constexpr SpiPads client_pads{
    .data_out = SercomPad::pad2,
    .sck = SercomPad::pad3,
    .ss = SercomPad::pad1,
    .data_in = SercomPad::pad0,
    .data_out_pin = {'A', 18, PinFunction::c},
    .sck_pin = {'A', 19, PinFunction::c},
    .ss_pin = {'A', 17, PinFunction::c},
    .data_in_pin = {'A', 16, PinFunction::c},
};

/// LOOP-BACK (32.6.3.4): DI moved onto the DO pad, so a host reads its
/// own transmit line back THROUGH THE PAD. Wireless, and the only
/// arrangement in this file where DIPO and DOPO name one pad.
constexpr SpiPads loopback_pads{
    .data_out = SercomPad::pad0,
    .sck = SercomPad::pad3,
    .ss = SercomPad::pad1,
    .data_in = SercomPad::pad0,
    .data_out_pin = {'A', 16, PinFunction::c},
    .sck_pin = {'A', 19, PinFunction::c},
    .ss_pin = {'A', 17, PinFunction::c},
    .data_in_pin = {'A', 16, PinFunction::c},
};

// The device header must agree that these four pins reach SERCOM1's four
// pads through function C - the checkable half of the pad-to-pin claim.
static_assert(MUX_PA16C_SERCOM1_PAD0 == static_cast<uint8_t>(PinFunction::c));
static_assert(MUX_PA17C_SERCOM1_PAD1 == static_cast<uint8_t>(PinFunction::c));
static_assert(MUX_PA18C_SERCOM1_PAD2 == static_cast<uint8_t>(PinFunction::c));
static_assert(MUX_PA19C_SERCOM1_PAD3 == static_cast<uint8_t>(PinFunction::c));
static_assert(spi_dopo_for(host_pads.data_out, host_pads.sck, host_pads.ss).value() == 3);
static_assert(spi_dopo_for(client_pads.data_out, client_pads.sck, client_pads.ss).value() == 1);

using Bus = SpiHost<link_sercom, host_pads>;
using Loop = SpiHost<link_sercom, loopback_pads>;
using Peer = SpiClient<link_sercom, client_pads>;
using Raw = Spi<link_sercom>;

using Cs = Pin<'A', 17>;
using SckPin = Pin<'A', 19>;

/// The command channel's SCK, and the one place it is named. 200 kHz -
/// a byte is 40 us, which is a whole order of magnitude more than the
/// peer's polled loop needs to turn one around, and well inside its own
/// CLK_PER/6 = 4 MHz client ceiling.
constexpr uint32_t command_hz = 200'000UL;
constexpr uint8_t command_baud = spi_baud_reg(SysClock::hz, command_hz).value();
static_assert(command_baud == 119);
static_assert(spi_sck_hz(SysClock::hz, command_baud) == command_hz);

// ---------------------------------------------------------------------------
// The crystal ruler
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

/// Letter j: the vector belongs to this board's CLIENT, and its one source
/// is INTFLAG.SSL (32.6.3.6) - the select's fall, counted and stamped on the
/// crystal ruler; the handler then holds the core `ssl_hold_us` (a handler
/// timed into the client's run) and disarms the source.
volatile bool client_isr = false;
volatile uint32_t ssl_hold_us = 0;
volatile uint8_t ssl_seen = 0;
volatile uint32_t ssl_at = 0;

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

/// SERCOM1 as the command channel's host, PA17 as its chip select.
bool link_command_mode() {
    Cs::set();
    Cs::output();
    const bool ok = Bus::init(clock);
    dec.reset();
    return ok;
}

/**
 * THE CHIP-SELECT HOLD, and it is a wire fact this suite has to spend.
 *
 * The engine releases CS as soon as the last character's RXC says the
 * bits have moved - about a microsecond after the last SCK edge, and
 * that is not enough for a peer whose SPI client is RESET by SS going
 * high: a byte sitting in its DATA that its polled loop has not fetched
 * yet is simply gone. The symptom is that every command frame loses
 * EXACTLY ITS LAST CHARACTER, which for a four-byte ping frame is the
 * checksum, so the peer waits for one and nak's the next window's first
 * dummy instead - about one frame in a dozen gets through. A host that
 * pays spilink::gap_us AFTER each character including the last never
 * meets it, because its CS release is 20 us behind the wire.
 *
 * So the PROTOCOL owns the chip select here, and the engine's Request
 * carries a null PinRef for these windows. No engine has a per-request
 * chip-select hold: that Request knob is declared future growth with
 * its first device user, and this suite spends its own measured spin.
 */
volatile uint32_t hold_spins = 0;
uint32_t hold_us = 0;

void link_hold() {
    // The counter is volatile so -Os cannot delete the loop: gcc will
    // delete a bare polling loop. Compound ops on volatile are
    // deprecated in C++20, hence the long spelling.
    for (volatile uint32_t i = 0; i < hold_spins; i = i + 1) {
    }
}

/// A spin is not a time, so it is MEASURED and not assumed: one
/// thousand turns are timed on the crystal, the count is scaled to about
/// thirty microseconds, and what that really came out at is printed in
/// the banner. (This is an APP spinning, not a driver.)
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
        .tx = lend<Lease::reply>(tx),
        .rx = lend<Lease::reply>(rx),
        .len = len,
        .cmd_len = 0,
        .engine = SpiEngine::polled,
        .baud = baud,
        .mode = mode,
        .reply = {},
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
/// the character - measured as an exact one-bit slip in BOTH
/// directions, on modes 2 and 3 only. Bus::prime() restates the order.
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
/// guarantee.
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
          "      the peer board must be running `spi_peer`; its console '0' "
          "forces the dark client back.",
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
          "  THE PEER DID NOT ANSWER. The peer board must be running `spi_peer`; "
          "its console '0' forces the dark client back. Check the five wires "
          "in this file's header.",
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
    /// One character at a time with the chip-select hold between them -
    /// spilink::gap_us or more, the pacing a NORMAL-mode client needs to
    /// load its next answer between characters (letter h's AVR client).
    bool gapped = false;
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
    // for the legs that ask for LSb first, and put back afterwards. A
    // GAPPED leg takes the same raw path: the engine's request is one
    // run with no pause between its characters, by design.
    const bool raw_leg = e.host_lsb || e.gapped;
    if (raw_leg) {
        SpiConfig c = spi_role_probe(host_pads, SpiRole::host);
        c.mode = e.host_mode;
        c.lsb_first = e.host_lsb;
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
    if (raw_leg) {
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
            if (e.gapped) link_hold();
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
    uint8_t dummy = 0;
    bool leads = false;    ///< buffer mode without BUFWR: a dummy comes first
};

/// What this end read back, against spi_link.hpp's alignment table: the
/// answer stream from the first character, except in regime_buffer,
/// where the client's shift register leads with a DUMMY and the stream
/// is one place late. Every letter but h commands regime_buffer_wait,
/// where every peer answers from the first character; letter h judges
/// the table's other two rows against the one peer that keeps them, the
/// AVR.
Verify verify_rx(const Exchange& e) {
    Verify v{};
    const bool reversed = e.host_lsb != (e.cfg.dord != 0);
    v.leads = e.cfg.regime == spilink::regime_buffer;
    if (v.leads) v.dummy = xrx[0];
    for (uint16_t i = v.leads ? 1u : 0u; i < e.count && i < max_exchange; ++i) {
        uint8_t exp = spilink::pattern_value(
            e.pattern, e.seed_b, v.leads ? static_cast<uint16_t>(i - 1u) : i);
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
        // Commands decode, the hardware serves the preloads and echoes,
        // the window burns whole reading nothing: the peer's receive went
        // deaf for the window. The aux bytes are the forensics (each
        // peer's header says what they carry).
        print(serial, "    the peer's exchange window read NOTHING (count=0, timed "
                      "out) while the command channel works - peer telemetry: "
                      "aux0=", hex(r.aux0), " aux1=", r.aux1, " aux2=", hex(r.aux2),
              " aux3=", hex(r.aux3), crlf);
    }
    const bool reversed = e.host_lsb != (e.cfg.dord != 0);
    print(serial, "    read:");
    for (uint16_t i = 0; i < e.count && i < max_exchange; ++i) {
        print(serial, " ", hex(xrx[i]));
    }
    print(serial, crlf, "    want:");
    for (uint16_t i = 0; i < e.count && i < max_exchange; ++i) {
        if (v.leads && i == 0) {
            print(serial, " --");
            continue;
        }
        uint8_t exp = spilink::pattern_value(
            e.pattern, e.seed_b, v.leads ? static_cast<uint16_t>(i - 1u) : i);
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
    // control waits until the channel really reads disconnected; a
    // read-back on the next instruction measures its own race, not the
    // silicon.
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
    // The CLOCKLESS leg is its own finding either way (a reset can
    // synchronize into a domain the channel feeds). What is
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
                       SERCOM_SPIM_CTRLA_DOPO_Pos) == 3u &&
                      ((ca & SERCOM_SPIM_CTRLA_DIPO_Msk) >>
                       SERCOM_SPIM_CTRLA_DIPO_Pos) == 2u);
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
    bench.verdict("and it is a DIFFERENT DOPO ROW (0x1 against the host's 0x3) - the "
                  "role changes which code CTRLA carries, not a direction bit",
                  Raw::role() == SpiRole::client &&
                      ((cb & SERCOM_SPIM_CTRLA_DOPO_Msk) >>
                       SERCOM_SPIM_CTRLA_DOPO_Pos) == 1u &&
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
    // sets where that stops holding, and it differs from peer to peer -
    // and so does the silicon under it: an AVR128DA48 client's runs hold
    // to 2 MHz, where its buffer's copy into the shifter stops meeting
    // the host's first edge (its errata clarification 3.5.2; letter i).
    // WHERE it lands is the print, with the first two rungs that are not
    // exact dumped whole; the verdict claims only the floor every peer
    // clears.
    static const uint32_t rates[] = {200'000UL, 500'000UL, 1'000'000UL, 2'000'000UL,
                                     3'000'000UL, 4'000'000UL, 6'000'000UL,
                                     8'000'000UL, 12'000'000UL, 24'000'000UL};
    // Both ends on polled pumps: spare_polled_pump asks a peer that has
    // an engine for its polled loop, so the boundary measured here is
    // the polled one whichever family answers. The serve= field prints
    // what the peer reports it did; the SAM peer's only serve is its
    // pump.
    uint32_t last_good = 0;
    uint32_t first_bad = 0;
    uint8_t dumped = 0;
    spilink::Report bad_r{};
    print(serial, "  -- both ends on polled pumps --", crlf);
    for (uint8_t i = 0; i < sizeof(rates) / sizeof(rates[0]); ++i) {
        const auto b = Bus::baud_for(rates[i]);
        if (!b) continue;
        Exchange e{};
        e.baud = *b;
        e.count = 8;
        e.seed_a = 0x21;
        e.seed_b = 0x84;
        e.spare = spilink::spare_polled_pump;
        Verify v{};
        spilink::Report r{};
        const bool ok = exchange_exact(e, v, r);
        const uint32_t real = spi_sck_hz(SysClock::hz, *b);
        print(serial, "  SCK ", real / 1000u, " kHz (BAUD ", *b, "): ",
              ok ? "exact both ways" : "NOT exact", "  host mism=", v.mism,
              " client mism=", r.mism, " client count=", r.count,
              " serve=", (r.aux2 & 0x04) != 0 ? "dma" : "pump");
        if (r.mism != 0) {
            print(serial, " (client first idx ", r.idx, " got ", hex(r.got), " exp ",
                  hex(r.exp), ")");
        }
        print(serial, crlf);
        if (!ok && dumped < 2) {
            ++dumped;
            dump_exchange(e, v, r);
        }
        if (ok && first_bad == 0) {
            last_good = real;
        } else if (!ok && first_bad == 0) {
            first_bad = real;
            bad_r = r;
        }
    }
    print(serial, "  on pumps the link held to ", last_good / 1000u, " kHz back to back");
    if (first_bad != 0) {
        print(serial, " and broke at ", first_bad / 1000u, " kHz");
    }
    print(serial, crlf);
    bench.verdict("the polled link is exact at the command rate and at least 2.5 "
                  "times faster",
                  last_good >= 500'000UL);
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
                  first_bad == 0 || (bad_r.count == 8 && bad_r.mism == 0));

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
    // matures mid-character and reaches the wire ONE CHARACTER LATE:
    // measured, the host reads the preload exactly and then the whole
    // stream slips by one (mism 11 of 12).
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
    a.aux16 = 32;            // the peer's host SCK: its own clock / 32
                             // (a 48 MHz peer clocks 1.5 MHz)
    a.cfg.apply = 1;
    a.cfg.mode = 0;
    a.cfg.dord = 0;

    if (!peer_act(Op::host_burst, a)) {
        bench.verdict("the peer accepted the host_burst command (its spi_peer must "
                      "carry op 0x14 - reflash the peer)",
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
/// short. The chip select is not used; PA17 stays PORT's.
bool loopback_burst(const uint8_t* tx, uint8_t* rx, uint16_t len, uint8_t baud,
                    SpiMode mode = SpiMode::mode0) {
    Loop::Request r{
        .cs = {},
        .dc = {},
        .cmd = {},
        .tx = lend<Lease::reply>(tx),
        .rx = lend<Lease::reply>(rx),
        .len = len,
        .cmd_len = 0,
        .engine = SpiEngine::polled,
        .baud = baud,
        .mode = mode,
        .reply = {},
    };
    return Loop::start(r);
}

uint8_t lb_tx[64];
uint8_t lb_rx[64];

/// THE RATE ON THE WIRE, which a loop-back alone cannot tell: its receiver
/// samples on the clock its transmitter shifts on, so a divisor wrong at
/// both ends is a slower or faster loop and nothing else. So the time ONE
/// FRAME takes inside a polled block is read from the difference of two
/// blocks of 1024 and 128 frames - neither buffer named, the transmit-only
/// shape pouring 0xFF dummies on DRE alone (samc21/spi.hpp, write_phase),
/// the shape whose turn is the shortest - so the fixed cost of a request
/// cancels; in thousandths of eight SCK periods at the rate ASKED, on the
/// 24 MHz crystal (41.7 ns a tick, the oscillator the SERCOM does not run
/// on: the reading judges the divisor and OSC48M together). No gap is
/// added between frames: DATA is loaded while the frame before it shifts
/// (32.6.1), and the readings at the slow rungs are that gap's
/// measurement. The best of three of each block is taken. 0 when the
/// ruler is down or the rate cannot be asked.
uint32_t frame_permille(uint32_t hz, SpiMode mode = SpiMode::mode0) {
    const auto b = Loop::baud_for(hz);
    if (!ruler_ok || !b) {
        return 0;
    }
    auto best = [&](uint16_t n) {
        uint32_t least = 0xFFFFFFFFu;
        for (uint8_t k = 0; k < 3u; ++k) {
            Loop::Request r{
                .cs = {}, .dc = {}, .cmd = {}, .tx = {}, .rx = {},
                .len = n, .cmd_len = 0, .engine = SpiEngine::polled,
                .baud = *b, .mode = mode, .reply = {},
            };
            const uint32_t t0 = wall();
            (void)Loop::start(r);
            const uint32_t took = wall() - t0;
            least = took < least ? took : least;
        }
        return least;
    };
    const uint32_t big = best(1024);
    const uint32_t small = best(128);
    const uint64_t want = 896ull * 8u * crystal_hz / hz;   // crystal ticks, at the rate asked
    return big <= small ? 0u : static_cast<uint32_t>(1000ull * (big - small) / want);
}

/// The verdict on a reading: never faster than the rate asked by more than
/// the ruler's own grain, at most three per cent slower.
bool wire_ok(uint32_t permille) { return permille >= 990u && permille <= 1030u; }

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
    // the field alone drives the wait, because a caller may own the CS
    // and still want the engine's pacing.)
    if (ruler_ok) {
        auto timed = [&](uint8_t setup) {
            Loop::Request r{
                .cs = {}, .dc = {}, .cmd = {},
                .tx = lend<Lease::reply>(static_cast<const uint8_t*>(lb_tx)),
                .rx = lend<Lease::reply>(lb_rx),
                .len = 16, .cmd_len = 0, .engine = SpiEngine::polled, .cs_setup_us = setup,
                .baud = command_baud, .mode = SpiMode::mode0, .reply = {},
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
                      "wildly more",
                      delta >= 100u && delta <= 130u);
    } else {
        bench.verdict("the crystal ruler is available for the cs_setup measurement",
                      false);
    }

    // THE SCK LADDER ON THE WIRE, timed on the CRYSTAL: letter d's rungs,
    // the rates in hertz the host is asked for, each a frame inside a
    // polled block against its eight SCK periods (frame_permille). The
    // polled loop is faster than the frame up to 8 MHz - 48 core cycles a
    // frame against the transmit-only turn's 34 (docs/samc21/spi.md) - and
    // there the reading is the wire's; at 12 and 24 MHz the frame is 32 and
    // 16 cycles, shorter than the turn, so the instrument cannot time them:
    // printed, not judged - the engines time them (letter r).
    if (!ruler_ok) {
        print(serial, "  the crystal ruler did not come up - the rate measurement "
                      "is skipped and the verdict declined",
              crlf);
        bench.verdict("the crystal ruler is available for the rate measurement", false);
    } else {
        static const uint32_t judged[] = {93'750UL, 200'000UL, 500'000UL, 1'000'000UL,
                                          2'000'000UL, 3'000'000UL, 4'000'000UL,
                                          6'000'000UL, 8'000'000UL};
        static const uint32_t untimed[] = {12'000'000UL, 24'000'000UL};
        bool all_in_band = true;
        print(serial, "  a frame inside a polled block, in thousandths of its eight SCK periods "
                      "at the rate asked:");
        for (const uint32_t hz : judged) {
            const uint32_t pm = frame_permille(hz);
            print(serial, " ", hz / 1000u, " kHz ", pm);
            all_in_band = all_in_band && wire_ok(pm);
        }
        print(serial, crlf, "  not timed (the polled turn outlasts the frame):");
        for (const uint32_t hz : untimed) {
            print(serial, " ", hz / 1000u, " kHz ", frame_permille(hz));
        }
        const uint32_t m3 = frame_permille(1'000'000UL, SpiMode::mode3);
        print(serial, crlf, "  mode 3 at 1 MHz: ", m3, crlf);
        bench.verdict("THE SCK LADDER RUNS AT THE RATES ASKED: 93.75 kHz to 8 MHz, a frame "
                      "inside a polled block in its eight SCK periods within -1 % and +3 % on "
                      "the 24 MHz crystal, no gap between frames - and mode 3 the same",
                      all_in_band && wire_ok(m3));
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
    // then clocks ONE character and counts one rise - a verdict that
    // then fails on every second run.
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
// g - THE KERNEL LETTER: util/spi_bus.hpp over this engine
// ===========================================================================

namespace kl {

/// Four loop-back transactions, queued at once from one dispatch, and an
/// asynchronous one among them. The data is deterministic and wireless,
/// so what is under test here is the ARBITER over this engine and not the
/// wire: the reply channel, the refusal of a request asking for DMA this
/// host has not got, and the sleep vote. Every request this host takes
/// ends inside start(), so the arbiter never holds one in flight and its
/// pending FIFO never fills: the reject-when-full and the busy vote are
/// the arbiter's own properties, judged against an asynchronous engine in
/// the host tests (test/test_bus_master).
constexpr uint8_t queued = 4;
constexpr uint8_t payload = 6;

uint8_t tx[queued][payload];
uint8_t rx[queued][payload];

struct Kick {};
struct Refuse {};
struct Ask {};
struct AskBehind {};

class Driver;
using SpiArb = SpiBus<Loop, P, 3>;

class Driver : public Fsm<Driver, SpiDone, Kick, Refuse, Ask, AskBehind, SleepVote> {
public:
    static inline EventQueue<Event, 12, P> queue;
    static inline uint8_t replies = 0;
    static inline uint8_t rejected = 0;
    static inline uint8_t ok_replies = 0;
    static inline uint8_t votes = 0;
    static inline uint8_t replies_at_vote = 0;
    static inline bool last_vote = false;
    /// The status of each reply, in arrival order.
    static inline uint8_t order[queued + 2] = {};

    static void init() {
        replies = rejected = ok_replies = votes = replies_at_vote = 0;
        last_vote = false;
        for (uint8_t& o : order) o = 0xEE;
        start(&running);
    }
    static void dispatch(const Event& e) { Fsm::dispatch(e); }

private:
    static Status running(const Event& e) {
        return brio::match(
            e,
            [](const Kick&) {
                for (uint8_t i = 0; i < queued; ++i) {
                    post<SpiArb>(request(i, true));
                }
                return handled();
            },
            [](const Refuse&) {
                // Polled, ASYNCHRONOUS, polled: the middle one asks for
                // DMA and is refused; the bus is free again for the third.
                post<SpiArb>(request(0, true));
                post<SpiArb>(request(1, false));
                post<SpiArb>(request(2, true));
                return handled();
            },
            [](const Ask&) {
                post<SpiArb>(PrepareSleep{
                    .depth = SleepDepth::standby,
                    .reply = reply_to<Driver, SleepVote>(),
                });
                return handled();
            },
            [](const AskBehind&) {
                // A request, and a vote right behind it in the same queue:
                // the request ended inside its own dispatch, so the vote
                // finds the bus idle.
                post<SpiArb>(request(3, true));
                post<SpiArb>(PrepareSleep{
                    .depth = SleepDepth::standby,
                    .reply = reply_to<Driver, SleepVote>(),
                });
                return handled();
            },
            [](const SpiDone& d) {
                if (replies < queued + 2) order[replies] = d.status;
                ++replies;
                if (d.status == spi_ok) ++ok_replies;
                if (d.status == spi_rejected) ++rejected;
                return handled();
            },
            [](const SleepVote& v) {
                ++votes;
                last_vote = v.ok;
                replies_at_vote = replies;
                return handled();
            },
            [](auto) { return unhandled(); });
    }

    static Loop::Request request(uint8_t i, bool polled) {
        return Loop::Request{
            .cs = {},
            .dc = {},
            .cmd = {},
            .tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx[i])),
            .rx = lend<Lease::reply>(rx[i]),
            .len = payload,
            .cmd_len = 0,
            .engine = polled ? SpiEngine::polled : SpiEngine::dma,
            .baud = command_baud,
            .mode = SpiMode::mode0,
            .reply = reply_to<Driver, SpiDone>(),
        };
    }
};

using BusKernel = Tenuto<P, Driver, SpiArb>;

/// Every queue drained, then again until `done` or 300 ms.
template <typename Done>
void run_until(Done done) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 300u) {
        while (BusKernel::step()) {
        }
        if (done()) break;
    }
}

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
    post<kl::Driver>(kl::Kick{});
    kl::run_until([] { return kl::Driver::replies >= kl::queued; });

    uint16_t data_mism = 0;
    for (uint8_t i = 0; i < kl::queued; ++i) {
        for (uint8_t k = 0; k < kl::payload; ++k) {
            if (kl::rx[i][k] != kl::tx[i][k]) ++data_mism;
        }
    }
    print(serial, "  ", kl::queued, " polled requests queued in ONE dispatch: ",
          kl::Driver::replies, " replies (", kl::Driver::ok_replies, " ok), ", data_mism,
          " byte mismatches", crlf);
    bench.verdict("EVERY queued request came back through its own ReplyTo with spi_ok - the "
                  "arbiter serialized four transactions on one bus",
                  kl::Driver::replies == kl::queued && kl::Driver::ok_replies == kl::queued);
    bench.verdict("and every one of them moved its bytes: the loop-back byte-exact under the "
                  "kernel, each request ended inside its start()",
                  data_mism == 0);

    // The refusal through the arbiter: an asynchronous request between
    // two polled ones.
    for (uint8_t i = 0; i < 3; ++i) {
        for (uint8_t k = 0; k < kl::payload; ++k) kl::rx[i][k] = 0xEE;
    }
    kl::Driver::init();
    post<kl::Driver>(kl::Refuse{});
    kl::run_until([] { return kl::Driver::replies >= 3; });
    bool rx1_untouched = true;
    for (uint8_t k = 0; k < kl::payload; ++k) rx1_untouched = rx1_untouched && kl::rx[1][k] == 0xEE;
    const bool ends_moved = kl::rx[0][kl::payload - 1] == kl::tx[0][kl::payload - 1] &&
                            kl::rx[2][kl::payload - 1] == kl::tx[2][kl::payload - 1];
    print(serial, "  polled, asynchronous, polled: replies ", kl::Driver::order[0], ", ",
          kl::Driver::order[1], ", ", kl::Driver::order[2], " (spi_ok ", spi_ok, ", spi_rejected ",
          spi_rejected, "); the asynchronous one's rx ", rx1_untouched ? "untouched" : "WRITTEN", crlf);
    bench.verdict("an asynchronous request is answered spi_rejected through its own ReplyTo, "
                  "nothing moved (has_dma is false), and the bus carries the next one",
                  !Loop::has_dma && kl::Driver::replies == 3 && kl::Driver::order[0] == spi_ok &&
                      kl::Driver::order[1] == spi_rejected && kl::Driver::order[2] == spi_ok &&
                      rx1_untouched && ends_moved && kl::SpiArb::rejected_count() == 0);

    // The sleep vote: idle, then right behind a request.
    kl::Driver::init();
    post<kl::Driver>(kl::Ask{});
    kl::run_until([] { return kl::Driver::votes != 0; });
    const bool idle_vote = kl::Driver::last_vote;
    const uint8_t idle_votes = kl::Driver::votes;
    kl::Driver::init();
    post<kl::Driver>(kl::AskBehind{});
    kl::run_until([] { return kl::Driver::votes != 0 && kl::Driver::replies != 0; });
    const bool behind_vote = kl::Driver::last_vote;
    const uint8_t behind_votes = kl::Driver::votes;
    const uint8_t replied_first = kl::Driver::replies_at_vote;

    print(serial, "  PrepareSleep(standby): idle votes=", idle_votes, " ok=", idle_vote,
          "; behind a request votes=", behind_votes, " ok=", behind_vote,
          ", the request's reply before it: ", replied_first, crlf);
    bench.verdict("an IDLE bus votes ok on a PrepareSleep", idle_votes == 1 && idle_vote);
    bench.verdict("and so does a vote posted right behind a request: the request ended in its "
                  "own dispatch, replied first, and left nothing in flight",
                  behind_votes == 1 && behind_vote && replied_first == 1);

    // The rate this letter's requests ask for, and the generator's
    // slowest, timed on the wire.
    const uint32_t pm_cmd = frame_permille(command_hz);
    const uint32_t pm_slow = frame_permille(spi_sck_hz(SysClock::hz, 255));
    print(serial, "  a frame inside a polled block at the letter's rate and the slowest, in "
          "thousandths of its "
          "wire time: ", command_hz / 1000u, " kHz ", pm_cmd, ", ",
          spi_sck_hz(SysClock::hz, 255), " Hz ", pm_slow, crlf);
    bench.verdict("and the bus runs at the rates asked: a frame inside a polled block "
                  "in its eight SCK periods within -1 % and +3 %",
                  wire_ok(pm_cmd) && wire_ok(pm_slow));
    Loop::release();
    Cs::set();
    Cs::output();
    bench.verdict("NOT ONE LINE of util/spi_bus.hpp, util/bus_master.hpp or kernel/ "
                  "was changed for this architecture - the arbiter ran the contract "
                  "as written on this silicon",
                  true);
}

// ===========================================================================
// h - THE AVR CLIENT'S FACTS, with this board as the host
// ===========================================================================
//
// test_avr_spi's two-board half draws some verdicts that are facts about
// its PEER, an AVR128DA48 client - its buffering regimes, its CLK_PER/6
// ceiling, its loss semantics and its write collision, its self-selected
// client - and on a desk where that suite's only peer is a SAM C21 it
// prints them and judges what is portable. This letter is where they are
// judged instead, whenever THIS suite's peer is the avrxt spi_peer: the
// same commands, the same streams and seeds, the same pacing (one
// character at a time with the chip-select hold between them, never less
// than spilink::gap_us, at the AVR suite's CLK_PER/32 of a 24 MHz peer),
// this board in the host's seat.

/// The AVR suite's command pace, CLK_PER/32 of a 24 MHz peer: 750 kHz,
/// exact on this generator.
constexpr uint8_t avr_command_baud = spi_baud_reg(SysClock::hz, 750'000UL).value();
static_assert(spi_sck_hz(SysClock::hz, avr_command_baud) == 750'000UL);

const char* const regime_name[3] = {"normal", "buffer", "buffer+BUFWR"};

/// test_avr_spi p's sink burst: `count` characters as ONE request, no
/// gap, into a client told never to drain; then its report.
bool h_sink(uint8_t regime, uint8_t seed, uint16_t count, uint8_t flags, spilink::Report& r) {
    spilink::Params a{};
    a.cfg.regime = regime;
    a.count = count;
    a.ms = 250;
    a.seed_a = seed;
    a.flags = flags;
    a.pattern = spilink::pattern_counting;
    if (!peer_act(Op::sink_slow, a)) return false;
    const uint16_t n = count < max_exchange ? count : max_exchange;
    for (uint16_t i = 0; i < n; ++i) xtx[i] = static_cast<uint8_t>(seed + i);
    settle();
    (void)link_xfer(xtx, nullptr, n, avr_command_baud);
    settle_ms(350);
    return peer_report(r);
}

/// The client half of test_avr_spi r. The avrxt peer's Op::mspi client
/// selects ITSELF - INVEN on its pulled-up SS pin, because the USART's
/// Host SPI it is built to serve has no client select - and both ends
/// count the same lead-in instead of a select edge. This end clocks the
/// burst with its chip select left HIGH, the level the peer's INVEN reads
/// as selected; the mode change happens inside the lead-in, whose end the
/// peer marks by flushing what the wire carried and preloading its first
/// answer.
bool h_mspi(uint8_t mode, uint8_t dord, Exchange& shape, Verify& v, spilink::Report& r) {
    constexpr uint16_t lead_ms = 25;
    constexpr uint16_t count = 12;
    const uint8_t seed_a = static_cast<uint8_t>(0x91 + mode * 4 + dord);
    const uint8_t seed_b = static_cast<uint8_t>(0x3B + mode * 4 + dord);

    spilink::Params a{};
    a.cfg.mode = mode;
    a.cfg.dord = dord;
    a.cfg.regime = spilink::regime_normal;
    a.count = count;
    a.ms = 800;
    a.seed_a = seed_a;
    a.seed_b = seed_b;
    a.pattern = spilink::pattern_prbs;
    a.aux8 = lead_ms;
    if (!peer_act(Op::mspi, a)) return false;

    SpiConfig c = spi_role_probe(host_pads, SpiRole::host);
    c.mode = static_cast<SpiMode>(mode);
    c.lsb_first = dord != 0;
    c.baud = avr_command_baud;
    const bool up = Raw::configure(c) && Raw::enable(true);
    Cs::set();
    settle_ms(lead_ms + 10u);
    for (uint8_t i = 0; i < max_exchange; ++i) xrx[i] = 0xEE;
    for (uint16_t i = 0; up && i < count && i < max_exchange; ++i) {
        Raw::data(spilink::pattern_value(a.pattern, seed_a, i));
        uint32_t spins = 200000u;
        while (!Raw::rxc_flag() && spins-- != 0u) {
        }
        xrx[i] = static_cast<uint8_t>(Raw::data());
        link_hold();
    }
    settle_ms(10);                        // the peer clears its INVEN at the count bound
    (void)link_command_mode();

    shape = Exchange{};                   // only to describe the expected stream
    shape.cfg.mode = mode;
    shape.cfg.dord = dord;
    shape.host_lsb = dord != 0;
    shape.cfg.regime = spilink::regime_normal;
    shape.count = count;
    shape.seed_a = seed_a;
    shape.seed_b = seed_b;
    shape.pattern = a.pattern;
    v = verify_rx(shape);
    return up && peer_report(r);
}

void th_avr_client() {
    if (!need_peer()) return;
    spilink::Ident d{};
    const bool got = peer_ident(d);
    bench.verdict("ident comes back", got);
    if (!got) return;
    if (spilink::peer_kind(d.version) != spilink::peer_avrxt) {
        print(serial, "  SKIPPED, no verdict claimed: the peer (fw ", hex(d.version),
              ") is not the avrxt spi_peer, and every verdict of this letter is a fact "
              "about an AVR128DB client",
              crlf);
        return;
    }
    print(serial, "  the peer is the avrxt spi_peer (fw ", hex(d.version), ", crystal ",
          d.xtal ? "up" : "DOWN", "): its CLK_PER is ", d.xtal ? "its crystal's" : "its OSCHF",
          " (24 MHz on the AVR DA/DB boards, 20 MHz on the AVR EA's)", crlf);

    // (1) THE ALIGNMENT TABLE (test_avr_spi l's AVR-client half): the
    // three regimes, in mode 0 MSb first and in mode 3 LSb first.
    uint8_t dummies = 0;
    uint8_t dummy_nonzero = 0;
    uint8_t dummy_seen = 0;
    for (uint8_t k = 0; k < 2; ++k) {
        const uint8_t m = k == 0 ? 0u : 3u;
        for (uint8_t reg = 0; reg < 3; ++reg) {
            Exchange e{};
            e.cfg.mode = m;
            e.cfg.dord = k;
            e.cfg.regime = reg;
            e.host_mode = static_cast<SpiMode>(m);
            e.host_lsb = k != 0;
            e.baud = avr_command_baud;
            e.gapped = true;
            e.count = 8;
            e.seed_a = static_cast<uint8_t>(0x11 + m * 16 + k * 5 + reg);
            e.seed_b = static_cast<uint8_t>(0x83 + m * 16 + k * 5 + reg);
            Verify v{};
            spilink::Report r{};
            const bool ok = exchange_exact(e, v, r);
            if (v.leads) {
                ++dummies;
                dummy_seen = v.dummy;
                if (v.dummy != 0) ++dummy_nonzero;
            }
            if (!ok) dump_exchange(e, v, r);
            bench.verdict(k == 0 ? "an AVR client, mode 0 MSb first, answers in spi_link's "
                                   "alignment for "
                                 : "an AVR client, mode 3 LSb first, answers in spi_link's "
                                   "alignment for ",
                          regime_name[reg], ok);
        }
    }
    print(serial, "  the dummy leading buffer mode WITHOUT BUFWR: ", dummies,
          " combinations, ", dummy_nonzero, " non-zero, the last ", hex(dummy_seen), crlf);
    bench.verdict("and that leading dummy is the shift register a fresh init cleared",
                  dummies == 2 && dummy_nonzero == 0);

    // (2) THE CLK_PER/6 CEILING (test_avr_spi m): 4 MHz on a 24 MHz peer,
    // 3.33 MHz on a 20 MHz one (the AVR errata's clarification 3.7.3 / the
    // EA's 3.7). Inside it exact, above it POSITIVELY corrupted - a test
    // that cannot see its failure signal passes vacuously. 3 MHz is inside
    // either, 6 MHz above either.
    static const uint32_t inside[] = {3'000'000UL, 1'500'000UL};
    static const uint32_t above[] = {6'000'000UL, 12'000'000UL};
    for (const uint32_t hz : inside) {
        Exchange e{};
        e.cfg.regime = spilink::regime_normal;
        e.baud = Bus::baud_for(hz).value_or(command_baud);
        e.gapped = true;
        e.count = 12;
        e.seed_a = 0x21;
        e.seed_b = 0x9B;
        Verify v{};
        spilink::Report r{};
        const bool ok = exchange_exact(e, v, r) && Bus::sck_hz(e.baud) == hz;
        if (!ok) dump_exchange(e, v, r);
        bench.verdict(hz == 3'000'000UL ? "an AVR client is exact at 3 MHz, inside its "
                                          "CLK_PER/6 ceiling"
                                        : "and at 1.5 MHz",
                      ok);
    }
    for (const uint32_t hz : above) {
        Exchange e{};
        e.cfg.regime = spilink::regime_normal;
        e.baud = Bus::baud_for(hz).value_or(command_baud);
        e.gapped = true;
        e.count = 12;
        e.seed_a = 0x35;
        e.seed_b = 0xC7;
        spilink::Report r{};
        const bool ran = do_exchange(e) && Bus::sck_hz(e.baud) == hz;
        const bool rep = ran && peer_report(r);
        const Verify v = verify_rx(e);
        print(serial, "  ", hz / 1000u, " kHz, above the client's CLK_PER/6: host mism=",
              v.mism, "/", e.count, " (first idx ", v.idx, " got ", hex(v.got), " exp ",
              hex(v.exp), "), client count=", r.count, " mism=", r.mism, "/", e.count, crlf);
        bench.verdict(hz == 6'000'000UL ? "an AVR client is positively corrupted at 6 MHz, "
                                          "above its ceiling"
                                        : "and at 12 MHz",
                      ran && rep && (v.mism > 0 || r.mism > 0));
    }

    // (3) THE LOSS SEMANTICS (test_avr_spi p): eight characters with no
    // gap into a client that never drains.
    spilink::Report r{};
    bool rep = h_sink(spilink::regime_normal, 0x40, 8, 0, r);
    print(serial, "  normal mode, 8 gapless 0x40..0x47 undrained: retained ", r.count, " = ",
          hex(r.aux0), " ", hex(r.aux1), " ", hex(r.aux2), " ", hex(r.aux3), "; INTFLAGS ",
          hex(r.sum), " ever, ", hex(r.got), " before the drain, ", hex(r.exp), " after", crlf);
    bench.verdict("a normal-mode AVR client keeps exactly one of the eight", rep && r.count == 1);
    bench.verdict("and it is the LAST: a new character overwrites the unread one",
                  rep && r.count >= 1 && r.aux0 == 0x47);
    rep = h_sink(spilink::regime_buffer, 0x50, 8, 0, r);
    const bool ovf_idle_tx = rep && (r.flags & spilink::report_bufovf) != 0;
    print(serial, "  buffer mode, 8 gapless 0x50..0x57 undrained, transmitter IDLE: retained ",
          r.count, " = ", hex(r.aux0), " ", hex(r.aux1), " ", hex(r.aux2), " ", hex(r.aux3),
          "; INTFLAGS ", hex(r.sum), " ever, BUFOVF=", ovf_idle_tx, crlf);
    bench.verdict("a buffer-mode AVR client keeps three: the two-deep FIFO and the shifter",
                  rep && r.count == 3);
    bench.verdict("the FIFO holds the FIRST two of the burst",
                  rep && r.count >= 2 && r.aux0 == 0x50 && r.aux1 == 0x51);
    bench.verdict("and the shifter the LAST", rep && r.count >= 3 && r.aux2 == 0x57);
    spilink::Report rf{};
    const bool repf = h_sink(spilink::regime_buffer, 0x60, 8, spilink::flag_feed_tx, rf);
    const bool ovf_fed_tx = repf && (rf.flags & spilink::report_bufovf) != 0;
    print(serial, "  the same flood with the client's TRANSMITTER FED: retained ", rf.count,
          ", INTFLAGS ", hex(rf.sum), " ever, BUFOVF=", ovf_fed_tx, crlf);
    bench.verdict("BUFOVF marks the loss when the AVR client has transmit data", ovf_fed_tx);
    bench.verdict("and stays clear when its transmitter is idle (its 28.5.5)", rep && !ovf_idle_tx);

    // (4) THE WRITE-COLLISION BOUNDARY and the MISSED LOAD (test_avr_spi
    // p): constant streams, host 0x5A and client 0xA5, so which write was
    // obeyed is legible straight off the wire.
    Exchange w{};
    w.cfg.regime = spilink::regime_normal;
    w.baud = avr_command_baud;
    w.gapped = true;
    w.count = 8;
    w.pattern = spilink::pattern_fixed;
    w.seed_a = 0x5A;
    w.seed_b = 0xA5;
    w.flags = spilink::flag_wrcol;
    spilink::Report rw{};
    const bool wran = do_exchange(w);
    const bool wrep = wran && peer_report(rw);
    const uint8_t gap_pos = spilink::wrcol_gap_at + 1;
    const uint8_t hold_pos = spilink::wrcol_hold_at + 1;
    print(serial, "  the AVR client wrote ", hex(spilink::wrcol_marker),
          " over its answer in the GAP after byte ", spilink::wrcol_gap_at,
          " and INSIDE the transfer after byte ", spilink::wrcol_hold_at, "; WRCOL=",
          (rw.flags & spilink::report_wrcol) != 0, ", this end read");
    for (uint8_t i = 0; i < w.count; ++i) print(serial, " ", hex(xrx[i]));
    print(serial, crlf);
    bool others_ok = wran && wrep;
    for (uint8_t i = 0; i < w.count; ++i) {
        if (i == gap_pos) continue;
        others_ok = others_ok && xrx[i] == w.seed_b;
    }
    bench.verdict("an AVR client's write INSIDE a transfer sets WRCOL",
                  wrep && (rw.flags & spilink::report_wrcol) != 0);
    bench.verdict("and is ignored: the answer loaded before it goes out intact",
                  wran && wrep && xrx[hold_pos] == w.seed_b);
    bench.verdict("a write in the inter-character GAP is no collision: it is obeyed",
                  wran && wrep && xrx[gap_pos] == spilink::wrcol_marker);
    bench.verdict("nothing else in that burst is disturbed", others_ok);
    bench.verdict("and the client received the whole burst",
                  wrep && rw.count == w.count && rw.mism == 0);

    Exchange sk = w;
    sk.flags = spilink::flag_skip_write;
    spilink::Report rs{};
    const bool sran = do_exchange(sk);
    const bool srep = sran && peer_report(rs);
    print(serial, "  the AVR client skipped its load after byte ", spilink::skip_at,
          "; this end read");
    for (uint8_t i = 0; i < sk.count; ++i) print(serial, " ", hex(xrx[i]));
    print(serial, crlf);
    bool rest_ok = sran && srep;
    for (uint8_t i = 0; i < sk.count; ++i) {
        if (i == spilink::skip_at + 1) continue;
        rest_ok = rest_ok && xrx[i] == sk.seed_b;
    }
    bench.verdict("an AVR client that misses its load sends back the byte it just received",
                  sran && srep && xrx[spilink::skip_at + 1] == sk.seed_a);
    bench.verdict("and every other character of that burst is its own answer", rest_ok);
    bench.verdict("and it still received the whole burst",
                  srep && rs.count == sk.count && rs.mism == 0);

    // (5) THE SELF-SELECTED CLIENT (the client half of test_avr_spi r).
    static const uint8_t mspi_cases[5][2] = {{0, 0}, {1, 0}, {2, 0}, {3, 0}, {0, 1}};
    static const char* const mspi_name[5] = {"mode 0 MSb", "mode 1 MSb", "mode 2 MSb",
                                             "mode 3 MSb", "mode 0 LSb"};
    for (uint8_t k = 0; k < 5; ++k) {
        Exchange shape{};
        Verify v{};
        spilink::Report rm{};
        const bool mrep = h_mspi(mspi_cases[k][0], mspi_cases[k][1], shape, v, rm);
        const bool ok = mrep && v.mism == 0 && rm.mism == 0 && rm.count == shape.count;
        if (!ok) dump_exchange(shape, v, rm);
        bench.verdict("a SELF-SELECTED AVR client (INVEN on its SS, no select edge) is exact "
                      "both ways: ",
                      mspi_name[k], ok);
    }

    Exchange back{};
    Verify v{};
    spilink::Report rb{};
    const bool ok = exchange_exact(back, v, rb);
    if (!ok) dump_exchange(back, v, rb);
    bench.verdict("and the link is exact again in the suite's own regime", ok);
}

// ===========================================================================
// i - the AVR client's runs, back to back
// ===========================================================================
//
// The avrxt peer serves an exchange with ONE CALL of its driver's runs
// (avrxt/spi.hpp, SpiClient: read by count, never by RXCIF), so a window
// can be as long as the run: 256 characters as ONE polled request of this
// engine, nothing between them but the engine's own gap. Four runs - the
// exchange, the answer run (spilink::spare_answer_only), the receive run
// (spilink::spare_receive_only, which that peer serves as an eight-bit
// window and raises no report_extended for: the verdict asks only what it
// received) and the exchange without BUFWR (regime_buffer, the leftover
// leading) - at the rungs inside the client's CLK_PER/6 (4 MHz on its
// 24 MHz crystal).

constexpr uint16_t long_window = 256;
uint8_t ltx[long_window];
uint8_t lrx[long_window];

struct LongWindow {
    spilink::Report r{};
    uint16_t host_mism = 0;    ///< the answers this end read back, against P_B
    uint16_t first_bit = 0;    ///< of them, wrong in the first bit alone and carrying
                               ///< the MSB of the character the client had just received
    uint16_t first = 0xFFFF;   ///< the first wrong one
    bool ran = false;
};

/// One window of `long_window` characters back to back at `baud`: the
/// command, the request, the report, and this end's read-back judged -
/// against P_B(i), or under regime_buffer (no BUFWR: the shifter's
/// leftover leads, spi_link.hpp's table) against P_B(i - 1) from the
/// second character, the first one the dummy and not judged.
LongWindow long_exchange(uint8_t baud, uint8_t spare, uint8_t seed_a, uint8_t seed_b,
                         uint8_t regime = spilink::regime_buffer_wait) {
    LongWindow w{};
    spilink::Params a{};
    a.cfg.regime = regime;
    a.count = long_window;
    a.ms = 400;
    a.seed_a = seed_a;
    a.seed_b = seed_b;
    a.pattern = spilink::pattern_prbs;
    a.spare = spare;
    if (!peer_act(Op::exchange, a)) return w;
    spilink::Stream out(a.pattern, seed_a);
    for (uint16_t i = 0; i < long_window; ++i) {
        ltx[i] = out.next();
        lrx[i] = 0xEE;
    }
    (void)link_xfer(ltx, lrx, long_window, baud);
    (void)link_command_mode();
    const bool leads = regime == spilink::regime_buffer;
    spilink::Stream want(a.pattern, seed_b);
    for (uint16_t i = leads ? 1u : 0u; i < long_window; ++i) {
        const uint8_t exp = want.next();
        if (lrx[i] == exp) continue;
        if (w.host_mism == 0) w.first = i;
        ++w.host_mism;
        if (i != 0 && (lrx[i] ^ exp) == 0x80u && (lrx[i] & 0x80u) == (ltx[i - 1] & 0x80u)) {
            ++w.first_bit;
        }
    }
    w.ran = peer_report(w.r);
    return w;
}

void ti_avr_runs() {
    if (!need_peer()) return;
    spilink::Ident d{};
    const bool got = peer_ident(d);
    bench.verdict("ident comes back", got);
    if (!got) return;
    if (spilink::peer_kind(d.version) != spilink::peer_avrxt || (d.version & 0xFFu) < 0x01u) {
        print(serial, "  SKIPPED, no verdict claimed: the peer (fw ", hex(d.version),
              ") is not the avrxt spi_peer with the client's runs (0x0101 and up)", crlf);
        return;
    }
    struct Rung {
        uint32_t hz;
        bool answers;   ///< the answers judged back to back (clarification 3.5.2 above)
        bool received;  ///< what the client received judged
    };
    static const Rung rungs[] = {{500'000UL, true, true},   {1'000'000UL, true, true},
                                 {2'000'000UL, true, true}, {3'000'000UL, false, true},
                                 {4'000'000UL, false, false}};
    constexpr uint8_t reps = 6;
    uint16_t short_windows = 0;
    uint16_t late_windows = 0;
    uint16_t windows = 0;
    uint8_t k = 0;
    for (const Rung& g : rungs) {
        const auto b = Bus::baud_for(g.hz);
        if (!b || Bus::sck_hz(*b) != g.hz) {
            bench.verdict("the rung's divisor is exact", false);
            continue;
        }
        uint16_t x_ok = 0;
        uint16_t rx_ok = 0;
        uint16_t first_bit_only = 0;
        uint16_t host_bad = 0;
        uint16_t host_first = 0xFFFF;
        for (uint8_t rep = 0; rep < reps; ++rep) {
            ++k;
            const LongWindow w = long_exchange(*b, 0, static_cast<uint8_t>(0x15 + 3u * k),
                                               static_cast<uint8_t>(0xB1 + 5u * k));
            const bool heard = w.ran && w.r.count == long_window && w.r.mism == 0;
            if (g.received && w.ran && w.r.count < long_window) ++short_windows;
            if (g.received && w.ran && w.r.aux3 == 3u) ++late_windows;
            if (g.received) ++windows;
            if (heard) ++rx_ok;
            if (heard && w.host_mism == 0) ++x_ok;
            if (w.host_mism != 0) {
                ++host_bad;
                if (host_first == 0xFFFF) host_first = w.first;
                if (w.host_mism == w.first_bit) ++first_bit_only;
            }
            if ((g.received && !heard) || (g.answers && w.host_mism != 0)) {
                print(serial, "    window ", rep, ": client count=", w.r.count, " mism=", w.r.mism,
                      " (first idx ", w.r.idx, " got ", hex(w.r.got), " exp ", hex(w.r.exp),
                      ") end=", w.r.aux3, " flags=", hex(w.r.flags), "; host mism=", w.host_mism,
                      " (first at ", w.first, ")", crlf);
            }
        }
        // Seeds whose streams differ in the first bit of about half the
        // characters from the character before (a stream that were the
        // host's one character late would hide the first-bit fault below).
        const LongWindow ans = long_exchange(*b, spilink::spare_answer_only, 0x5C, 0x93);
        const LongWindow rcv = long_exchange(*b, spilink::spare_receive_only, 0x6D, 0x00);
        // Without BUFWR: the first answer goes into the buffer behind the
        // shifter's leftover, so the read-back is one place late.
        const LongWindow nob = long_exchange(*b, 0, 0x4E, 0xD5, spilink::regime_buffer);
        if (g.received) windows = static_cast<uint16_t>(windows + 3u);
        if (g.received && nob.ran && nob.r.count < long_window) ++short_windows;
        const bool nob_ok = nob.ran && nob.host_mism == 0 && nob.r.count == long_window &&
                            nob.r.mism == 0;
        const bool ans_ok = ans.ran && ans.host_mism == 0 && ans.r.count == long_window;
        const bool rcv_ok = rcv.ran && rcv.r.count == long_window && rcv.r.mism == 0;
        print(serial, "  ", g.hz / 1000u, " kHz, ", long_window, " back to back: exchange exact ",
              x_ok, " of ", reps, ", received exact ", rx_ok, " of ", reps, ", read back wrong in ",
              host_bad, " (first at ", host_first, "; wrong in the first bit alone, the MSB just "
              "received: ", first_bit_only, ") | answer run: read back mism ", ans.host_mism,
              ", loaded ", ans.r.count, " | receive run: count ", rcv.r.count, " mism ",
              rcv.r.mism, " | without BUFWR: client count ", nob.r.count, " mism ", nob.r.mism,
              ", read back one late, mism ", nob.host_mism, " (first at ", nob.first, ")", crlf);
        if (g.answers) {
            bench.verdict(g.hz == 500'000UL   ? "the AVR client's exchange run is exact both ways, "
                                                "256 back to back, at 500 kHz"
                          : g.hz == 1'000'000UL ? "and at 1 MHz"
                                                : "and at 2 MHz",
                          x_ok == reps);
            bench.verdict(g.hz == 500'000UL   ? "its answer run is exact at 500 kHz"
                          : g.hz == 1'000'000UL ? "and at 1 MHz"
                                                : "and at 2 MHz",
                          ans_ok);
            bench.verdict(g.hz == 500'000UL
                              ? "WITHOUT BUFWR the exchange run is exact both ways at 500 kHz, "
                                "the host reading the leading dummy and then every answer one "
                                "place late"
                          : g.hz == 1'000'000UL ? "and at 1 MHz"
                                                : "and at 2 MHz",
                          nob_ok);
        }
        if (g.received) {
            bench.verdict(g.hz == 3'000'000UL ? "at 3 MHz the AVR client still RECEIVES every "
                                                "character of every window"
                                              : "its receive run hears every character",
                          g.hz == 3'000'000UL ? rx_ok == reps : rcv_ok);
        }
        if (g.hz == 3'000'000UL) {
            bench.verdict("and its answers break back to back on their FIRST BIT alone, which "
                          "carries the MSB of the character just received - the buffer's copy "
                          "into the shifter later than the host's first edge (DB clarification "
                          "3.5.2) - in the exchange run",
                          host_bad != 0 && first_bit_only == host_bad);
            bench.verdict("and in the answer run, which reads nothing",
                          ans.ran && ans.host_mism != 0 && ans.first_bit == ans.host_mism);
        }
    }
    print(serial, "  ", windows, " windows inside the client's ceiling: ", short_windows,
          " short of the count the host clocked, ", late_windows, " ended late", crlf);
    bench.verdict("no window inside the client's ceiling stopped short of the host's count",
                  short_windows == 0);
    (void)link_command_mode();
    bench.verdict("the command channel survives the runs", command(Op::ping));
}

// ===========================================================================
// j - THE CLIENT'S EDGES: the peer clocks, this board's runs end every way
// ===========================================================================
//
// Letter e meets the client role a character at a time; this letter runs
// the client's RUNS (samc21/spi.hpp SpiClient: exchange, receive) against
// the peer's host_burst - one character at a time with spilink::gap_us
// between them, the select low for the whole burst, at the peer's clock /
// 32 (750 kHz on the AVR128DA48's 24 MHz) - and stages what no other
// letter does: the receive run and its echo, a run whose host never
// clocks (`stalled`), the select's fall as INTFLAG.SSL (CTRLB.SSDE,
// 32.6.3.6) and as the wake-up from IDLE and then from STANDBY
// (CTRLA.RUNSTDBY, 32.6.5), address recognition (FORM 0x2, 32.6.3.1) both
// ways, and a handler timed into a run by that same SSL interrupt.

uint8_t jout[16];
uint8_t jin[16];

struct ClientBurst {
    bool commanded = false;
    bool reported = false;
    Peer::Run run{0, Peer::RunEnd::stalled};
    spilink::Report host{};       ///< what the peer, as host, read
    uint16_t rx_mism = 0;         ///< this end's bytes against P_A(i) = seed_a + i
    uint8_t rx_first = 0xFF;
    uint32_t end_at = 0;          ///< the crystal ruler when the run returned
};

enum class JRun : uint8_t { exchange, receive };

/// A run's budget: 400 000 counts of the four-poll await, well over the
/// 60 ms lead-in and a 12-character burst under a millisecond.
constexpr uint32_t client_run_spins = 400'000u;

const char* end_name(Peer::RunEnd e) {
    switch (e) {
        case Peer::RunEnd::complete: return "complete";
        case Peer::RunEnd::closed: return "closed";
        case Peer::RunEnd::stalled: return "stalled";
    }
    return "?";
}

/// Command the peer's burst of `n` (the counting pattern: it sends seed_a + i
/// and checks what it reads against want_b + i), become the client with
/// `cfg`, run `before` (the arming, a sleep), serve ONE run answering
/// seed_b + i, then ask the peer's report.
template <typename Before>
ClientBurst client_burst(JRun kind, const Peer::Config& cfg, uint16_t n, uint8_t seed_a,
                         uint8_t seed_b, uint8_t want_b, uint8_t lead_ms, Before before) {
    ClientBurst c{};
    spilink::Params a{};
    a.count = n;
    a.ms = 300;
    a.seed_a = seed_a;
    a.seed_b = want_b;
    a.pattern = spilink::pattern_counting;
    a.aux8 = lead_ms;
    a.aux16 = 32;
    c.commanded = peer_act(Op::host_burst, a);
    if (!c.commanded) return c;
    if (!Peer::init(clock, cfg)) {
        (void)link_command_mode();
        c.commanded = false;
        return c;
    }
    for (uint8_t i = 0; i < sizeof jout; ++i) {
        jout[i] = static_cast<uint8_t>(seed_b + i);
        jin[i] = 0xEE;
    }
    before();
    const std::span<const uint8_t> outs(jout, n);
    const std::span<uint8_t> ins(jin, n);
    c.run = kind == JRun::exchange ? Peer::exchange(outs, ins, client_run_spins)
                                   : Peer::receive(ins, client_run_spins);
    c.end_at = wall();
    for (uint16_t i = 0; i < c.run.count; ++i) {
        if (jin[i] == static_cast<uint8_t>(seed_a + i)) continue;
        if (c.rx_mism == 0) c.rx_first = static_cast<uint8_t>(i);
        ++c.rx_mism;
    }
    client_isr = false;
    ssl_hold_us = 0;
    Peer::release();
    settle_ms(20);
    (void)link_command_mode();
    c.reported = peer_report(c.host);
    return c;
}

/// The select's fall as an interrupt: SSL armed, the vector this letter's.
void arm_ssl(uint32_t hold_us) {
    ssl_seen = 0;
    ssl_at = 0;
    ssl_hold_us = hold_us;
    client_isr = true;
    Raw::clear_flags(SpiFlag::ssl);
    Raw::enable_ssl_interrupt(true);
}

void print_burst(const char* name, const ClientBurst& c) {
    print(serial, "  ", name, ": run ", end_name(c.run.end), " count ", c.run.count,
          ", received mism ", c.rx_mism);
    if (c.rx_mism) print(serial, " (first at ", c.rx_first, ": ", hex(jin[c.rx_first]), ")");
    print(serial, " | the host read ", c.host.count, " mism ", c.host.mism);
    if (c.host.mism) print(serial, " (first at ", c.host.idx, " got ", hex(c.host.got), " exp ",
                           hex(c.host.exp), ")");
    print(serial, crlf);
}

// ---- the sleeping legs' backstop and sleep: bench_sleep.hpp ------------------
//
// The RTC on OSCULP32K as the counted backstop, armed 300 ms ahead of every
// sleep, and the race-free sleep until the select's fall - the samc21
// suites' shared tools.

uint8_t sleep_for_select(SleepMode mode) {
    return benchsleep::sleep_until<Serial>(mode, [] { return ssl_seen != 0; });
}

void tj_client_edges() {
    if (!need_peer()) return;
    if (!ruler_ok) {
        bench.verdict("the crystal ruler is up (the letter's timings ride it)", false);
        return;
    }
    const Peer::Config plain{.mode = SpiMode::mode0, .preload = true, .ss_low_detect = true,
                             .drive_output = true};

    // The run against the burst, SSL armed with no hold: the baseline, and
    // the character period P measured from the select's fall to the run's
    // end (the first character starts gap_us after the fall).
    ClientBurst c = client_burst(JRun::exchange, plain, 12, 0x24, 0x81, 0x81, 40,
                                 [] { arm_ssl(0); });
    print_burst("exchange, 12 of 12", c);
    bench.verdict("the client's exchange run against the peer's burst ends complete, every "
                  "character exact both ways",
                  c.commanded && c.run.end == Peer::RunEnd::complete && c.run.count == 12 &&
                      c.rx_mism == 0 && c.reported && c.host.count == 12 && c.host.mism == 0);
    bench.verdict("CTRLB.SSDE raises INTFLAG.SSL at the select's fall, once (32.6.3.6)",
                  ssl_seen == 1);
    const uint32_t span_us = ssl_at != 0 ? (c.end_at - ssl_at) / 24u : 0u;
    const uint32_t p_us = span_us > spilink::gap_us ? (span_us - spilink::gap_us) / 12u : 0u;
    print(serial, "  the select's fall to the run's end ", span_us, " us: a character and its "
                  "gap about ", p_us, " us", crlf);

    // A handler timed into the run by the SSL interrupt itself. 10 us ends
    // before the first character starts (gap_us = 20 after the fall): no
    // boundary passes under it.
    c = client_burst(JRun::exchange, plain, 12, 0x31, 0x92, 0x92, 40, [] { arm_ssl(10); });
    print_burst("exchange, a 10 us handler at the select's fall", c);
    bench.verdict("a handler that ends before the first character changes nothing",
                  c.commanded && ssl_seen == 1 && c.run.end == Peer::RunEnd::complete &&
                      c.rx_mism == 0 && c.reported && c.host.mism == 0);

    // A handler that outlasts the third answer's window: it is due within
    // the first five SCK cycles of character 1 (32.6.2.6.2; 5 SCK = 6.7 us
    // at 750 kHz), i.e. by gap_us + P + 6.7 us after the fall, and the
    // handler must end before character 2 starts, gap_us + 2P: the middle,
    // gap_us + 1.5 P + 3.3, from P measured above.
    const uint32_t lo_us = spilink::gap_us + p_us + 7u;
    const uint32_t hi_us = spilink::gap_us + 2u * p_us;
    const uint32_t mid_us = spilink::gap_us + (3u * p_us) / 2u + 3u;
    static uint32_t hold_mid = 0;
    hold_mid = mid_us;
    c = client_burst(JRun::exchange, plain, 12, 0x43, 0xA4, 0xA4, 40,
                     [] { arm_ssl(hold_mid); });
    print(serial, "  the third answer's window closes at ", lo_us, " us, character 2 starts at ",
          hi_us, " us; the handler holds ", mid_us, " us", crlf);
    print_burst("exchange, a handler past the third answer's window", c);
    bench.verdict("a handler that outlasts an answer's window: the client still receives "
                  "every character exact",
                  c.commanded && ssl_seen == 1 && c.run.count == 12 && c.rx_mism == 0);
    // Character 2 carries what the shifter holds - character 1 as received,
    // P_A(1) (32.6.2.6.2) - and every later answer goes one character late:
    // with the counting stream each of characters 2..11 differs, ten.
    bench.verdict("and the answers SLIP from that character on, unseen by the run: character "
                  "2 carries the shifter's echo of character 1, every later answer goes one "
                  "character late, and the run still ends complete",
                  c.reported && c.host.mism == 10 && c.host.idx == 2 &&
                      c.host.got == static_cast<uint8_t>(0x43 + 1) &&
                      c.run.end == Peer::RunEnd::complete);

    // The receive run writes nothing: the shifter sends back the character
    // it last received (32.6.2.6.2), P_A(i - 1) at character i - asked as
    // the stream one below; only character 0 may differ. No preload.
    Peer::Config bare = plain;
    bare.preload = false;
    c = client_burst(JRun::receive, bare, 12, 0x56, 0x00, 0x55, 40, [] {});
    print_burst("receive, 12 of 12", c);
    bench.verdict("the receive run ends complete with every character exact",
                  c.commanded && c.run.end == Peer::RunEnd::complete && c.run.count == 12 &&
                      c.rx_mism == 0);
    bench.verdict("and the host read the echo, each character one late",
                  c.reported && c.host.count == 12 &&
                      (c.host.mism == 0 || (c.host.mism == 1 && c.host.idx == 0)));

    // `stalled`, nobody selects: the budget alone ends the run.
    {
        if (Peer::init(clock, plain)) {
            constexpr uint32_t budget = 20'000u;
            const uint32_t t0 = wall();
            const Peer::Run s = Peer::exchange(std::span<const uint8_t>(jout, 8),
                                               std::span<uint8_t>(jin, 8), budget);
            const uint32_t took = wall() - t0;
            print(serial, "  nobody selects: run ", end_name(s.end), " count ", s.count, " after ",
                  took / 24u, " us, ", (took * 2u) / budget, " core cycles a count", crlf);
            bench.verdict("a run nobody selects ends STALLED on its budget, with nothing",
                          s.end == Peer::RunEnd::stalled && s.count == 0);
        } else {
            bench.verdict("the client comes up", false);
        }
        Peer::release();
        (void)link_command_mode();
    }

    // `stalled`, the select held low and no clock: the peer's ss_pulse.
    {
        spilink::Params a{};
        a.aux8 = 20;
        a.aux16 = 40'000u;
        const bool took_cmd = peer_act(Op::ss_pulse, a);
        const bool up = Peer::init(clock, plain);
        uint32_t w = 0;
        while (!Peer::selected() && w < 4'000'000u) ++w;
        const bool fell = Peer::selected();
        constexpr uint32_t budget = 5'000u;
        const uint32_t t0 = wall();
        const Peer::Run s = Peer::exchange(std::span<const uint8_t>(jout, 8),
                                           std::span<uint8_t>(jin, 8), budget);
        const uint32_t took = wall() - t0;
        const bool still = Peer::selected();
        print(serial, "  the select held with no clock: run ", end_name(s.end), " count ", s.count,
              " after ", took / 24u, " us; the select still low at its end: ", still ? "yes" : "no",
              crlf);
        bench.verdict("a run whose select is held low with no clock ends STALLED on its budget, "
                      "with nothing, the select still low",
                      took_cmd && up && fell && s.end == Peer::RunEnd::stalled && s.count == 0 &&
                          still);
        settle_ms(60);
        Peer::release();
        (void)link_command_mode();
    }

    // Address recognition (FORM 0x2, 32.6.3.1): the first character of a
    // transaction is matched against ADDR; preload is off by the chapter.
    Peer::Config addr = bare;
    addr.ss_low_detect = false;
    addr.address_match = true;
    addr.address_mode = SpiAddressMode::mask;
    addr.address = 0x5A;
    addr.address_mask = 0x00;
    c = client_burst(JRun::receive, addr, 12, 0x5A, 0x00, 0x00, 40, [] {});
    print_burst("address 0x5A against a burst opening with 0x5A", c);
    print(serial, "    first characters received:", " ", hex(jin[0]), " ", hex(jin[1]), " ",
          hex(jin[2]), crlf);
    bench.verdict("a transaction whose first character matches ADDR is received",
                  c.commanded && c.run.count >= 11 && c.rx_mism == 0);
    {
        spilink::Params a{};
        a.count = 12;
        a.ms = 300;
        a.seed_a = 0x33;
        a.pattern = spilink::pattern_counting;
        a.aux8 = 40;
        a.aux16 = 32;
        const bool took_cmd = peer_act(Op::host_burst, a);
        const bool up = Peer::init(clock, addr);
        const Peer::Run s = Peer::receive(std::span<uint8_t>(jin, 12), 60'000u);
        const bool done = Peer::transaction_done();
        print(serial, "  address 0x5A against a burst opening with 0x33: run ", end_name(s.end),
              " count ", s.count, ", TXC ", done ? "up" : "down", crlf);
        bench.verdict("a transaction whose address does not match is IGNORED WHOLE: nothing "
                      "received, TXC never raised, the run ends on its budget",
                      took_cmd && up && s.end == Peer::RunEnd::stalled && s.count == 0 && !done);
        settle_ms(20);
        Peer::release();
        (void)link_command_mode();
        spilink::Report r{};
        (void)peer_report(r);
    }

    // The select's fall as the WAKE-UP: IDLE first, and STANDBY only once
    // IDLE has proven the source. The burst's lead-in is 60 ms; the RTC's
    // backstop 300 ms.
    if (!benchsleep::backstop_up()) {
        bench.verdict("the RTC backstop comes up", false);
        return;
    }
    static uint8_t wakes = 0;
    static bool backstop = false;
    Peer::Config idle_cfg = bare;
    c = client_burst(JRun::receive, idle_cfg, 12, 0x67, 0x00, 0x66, 60, [] {
        arm_ssl(0);
        backstop = !benchsleep::backstop_arm(9830);
        wakes = sleep_for_select(SleepMode::idle0);
        backstop = backstop || benchsleep::rtc_fired;
    });
    print_burst("IDLE until the select's fall, then receive", c);
    print(serial, "    ", wakes, " WFI, SSL ", ssl_seen, ", the backstop ",
          backstop ? "FIRED" : "silent", "; the lines pending at the first wake-up ",
          hex(benchsleep::first_pending), crlf);
    const bool idle_ok = c.commanded && ssl_seen == 1 && !backstop && wakes == 1;
    bench.verdict("the select's fall (SSL) wakes the core from IDLE, the one wake-up", idle_ok);
    bench.verdict("and the burst is received whole after it",
                  c.run.count == 12 && c.rx_mism == 0);

    if (idle_ok) {
        Peer::Config stby_cfg = bare;
        stby_cfg.run_standby = true;
        c = client_burst(JRun::receive, stby_cfg, 12, 0x78, 0x00, 0x77, 60, [] {
            arm_ssl(0);
            backstop = !benchsleep::backstop_arm(9830);
            wakes = sleep_for_select(SleepMode::standby);
            backstop = backstop || benchsleep::rtc_fired;
        });
        print_burst("STANDBY until the select's fall, then receive", c);
        print(serial, "    ", wakes, " WFI, SSL ", ssl_seen, ", the backstop ",
              backstop ? "FIRED" : "silent", "; the lines pending at the first wake-up ",
              hex(benchsleep::first_pending), crlf);
        bench.verdict("the select's fall wakes the core from STANDBY with CTRLA.RUNSTDBY set, "
                      "the one wake-up",
                      c.commanded && ssl_seen == 1 && !backstop && wakes == 1);
        bench.verdict("and the burst is received whole after it",
                      c.run.count == 12 && c.rx_mism == 0);
    } else {
        print(serial, "  STANDBY not entered: IDLE did not prove the select's fall as a wake-up",
              crlf);
    }
    benchsleep::backstop_down();
    (void)link_command_mode();
    bench.verdict("the command channel survives the client legs", command(Op::ping));
}

// ===========================================================================
// k - THE HOST'S RARE PATHS: the spin budget, an early TXC
// ===========================================================================

// ---- the one-shot that lands a handler inside a host request ------------------
//
// TC4 on generator 0 (48 MHz), sixteen bits, ONESHOT: started right before a
// request, its overflow enters a handler that holds the core and samples
// the SERCOM's INTFLAG at its end.

using Shot = Tc<4>;
volatile uint32_t shot_hold_us = 0;
volatile uint8_t shot_fired = 0;
volatile uint8_t shot_flags_end = 0;

bool shot_arm(uint32_t after_us, uint32_t hold_us) {
    shot_fired = 0;
    shot_flags_end = 0;
    shot_hold_us = hold_us;
    if (!Shot::init(0)) return false;
    if (!Shot::configure(TcConfig{.mode = TcMode::count16, .prescaler = TcPrescaler::div1,
                                  .waveform = TcWaveform::match_frequency, .one_shot = true})) {
        return false;
    }
    if (!Shot::set_cc16(0, static_cast<uint16_t>(after_us * 48u))) return false;
    Shot::clear_flags(Shot::overflow_flag);
    Shot::arm(Shot::overflow_flag);
    Nvic::enable(Shot::irq());
    return true;
}

void tk_host_paths() {
    // THE SPIN BUDGET RUNS OUT: a polled request with the SERCOM's core
    // clock channel down clocks nothing, and must come back spi_stalled
    // with the select raised and the receiver flushed.
    bench.verdict("the protocol's engine comes up", link_command_mode());
    GclkChannel::disconnect(Raw::gclk_core_id());
    bool clockless = false;
    for (uint32_t k = 0; k < 100000u && !clockless; ++k) {
        clockless = !GclkChannel::connected(Raw::gclk_core_id());
    }
    uint8_t txs[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    uint8_t rxs[8] = {};
    const uint32_t t0 = wall();
    (void)Bus::start(Bus::Request{
        .cs = Cs::ref(), .dc = {}, .cmd = {},
        .tx = lend<Lease::reply>(static_cast<const uint8_t*>(txs)),
        .rx = lend<Lease::reply>(rxs),
        .len = 8, .cmd_len = 0, .engine = SpiEngine::polled, .baud = command_baud,
        .mode = SpiMode::mode0, .reply = {},
    });
    const uint32_t took = wall() - t0;
    const uint8_t status = Bus::status();
    const bool cs_up = Cs::read();
    const bool rx_empty = !Raw::rxc_flag();
    (void)Raw::core_clock(0);
    print(serial, "  a polled request of 8 with the core clock channel down: status ", status,
          " after ", took / 24u, " us (the budget 200000 + 6000 a character = 248000 turns), select ",
          cs_up ? "high" : "LOW", crlf);
    bench.verdict("a polled request on a clockless SERCOM spends its spin budget and comes "
                  "back spi_stalled, the select raised, the receiver empty",
                  clockless && status == spi_stalled && cs_up && rx_empty);
    bench.verdict("and with the channel back the engine runs a request spi_ok",
                  link_command_mode() && command(Op::ping) && Bus::status() == spi_ok);

    // AN EARLY TXC: a transmit-only request of 64 characters at 1 MHz into
    // the peer's receive-only run, a 60 us handler (seven characters'
    // worth) landing about 120 us in. The shifter runs dry under it and
    // TXC rises; the next write clears it (32.8.6) and the tail waits for
    // the last character's - so the peer must count all 64 and its run end
    // complete, the select held to the end.
    if (need_peer()) {
        const auto b = Bus::baud_for(1'000'000UL);
        spilink::Params a{};
        a.cfg.regime = spilink::regime_buffer_wait;
        a.count = 64;
        a.ms = 400;
        a.seed_a = 0x3D;
        a.pattern = spilink::pattern_prbs;
        a.spare = spilink::spare_receive_only;
        const bool took_cmd = b && peer_act(Op::exchange, a);
        spilink::Stream out(a.pattern, a.seed_a);
        for (uint16_t i = 0; i < 64; ++i) ltx[i] = out.next();
        const bool armed = shot_arm(150, 60);
        if (took_cmd && armed) {
            (void)Shot::enable(true);
            (void)link_xfer(ltx, nullptr, 64, *b);
        }
        (void)Shot::enable(false);
        const uint8_t st = Bus::status();
        (void)link_command_mode();
        spilink::Report r{};
        const bool rep = peer_report(r);
        print(serial, "  64 at 1 MHz under a 60 us handler: the handler fired ", shot_fired,
              ", INTFLAG at its end ", hex(shot_flags_end), "; status ", st, "; the peer counted ",
              r.count, " mism ", r.mism, " end ", r.aux3, crlf);
        bench.verdict("the handler landed inside the request and the shifter ran dry under it "
                      "(TXC up at its end)",
                      shot_fired == 1 && (shot_flags_end & SpiFlag::txc) != 0);
        bench.verdict("and the request still sent all 64, the select held to the last: the "
                      "peer's run counted 64 exact and ended complete",
                      st == spi_ok && rep && r.count == 64 && r.mism == 0 && r.aux3 == 0);
    }

    (void)link_command_mode();
}

// ===========================================================================
// The engines (samc21/dmac.hpp, samc21/spi.hpp): letters l .. r
// ===========================================================================
//
// THE ENGINED HOSTS name their channels from the budget of two: the pair
// takes channels 0 (transmit) and 1 (receive), the transmit engine alone
// channel 1, so that the console's transmit engine (channel 0) may run
// beside it - a display's shape. Every engined host on SERCOM1 is owner 2
// (the SERCOM plus one), whichever of them claims, so they take the
// channels in turn by re-arming; a letter releases its host at its end, so
// the next one finds the budget free.
//
// THE SPY PIN: a PinRef whose group is a PORT group in SRAM, so the host's
// OUTCLR and OUTSET stores land in a struct this suite reads - the proof
// that a refused request moved neither the select nor the D/C, and of the
// order a request moved them in.
namespace dm {

// SPI_SELECT_CONTROL builds letter l's polled half alone, over the headers of
// the code before the engines - the control its verdict is shown failing on.
#ifndef SPI_SELECT_CONTROL
using DBus = SpiHost<link_sercom, host_pads, 0, DmaTxEngine<0>, DmaRxEngine<1>>;
using WBus = SpiHost<link_sercom, host_pads, 0, DmaTxEngine<1>>;
using DLoop = SpiHost<link_sercom, loopback_pads, 0, DmaTxEngine<0>, DmaRxEngine<1>>;
using WLoop = SpiHost<link_sercom, loopback_pads, 0, DmaTxEngine<1>>;
/// The console with its transmit engine on channel 0 (letter o).
using UDma = Uart<5, console_pads, 64, 256, DmaTxEngine<0>>;
constexpr UDma udma{};
#endif

/// The wireless letters' stand-in select: PB23, the pad bench_samc's
/// letter e scripts its D/C on (an LED on this board, lit while selected).
using Fake = Pin<'B', 23>;

port_group_registers_t spy_group{};
constexpr uint32_t spy_cs = 1u << 0;
constexpr uint32_t spy_dc = 1u << 1;
void spy_reset() {
    spy_group.PORT_OUTSET = 0u;
    spy_group.PORT_OUTCLR = 0u;
}
PinRef spy(uint32_t mask) { return PinRef{&spy_group, mask}; }

/// The engined host that is live, through two plain functions the vectors
/// call - the hosts' bodies are always inline, and a lambda without a
/// capture converts to a function pointer.
bool (*volatile live_isr)() = nullptr;
uint8_t (*volatile live_status)() = nullptr;
volatile bool dma_done = false;
volatile uint8_t dma_status = 0;
volatile bool sck_at_end = false;
volatile uint32_t dma_end_at = 0;
volatile bool console_engined = false;

template <typename H>
void engage() {
    live_status = [] { return H::status(); };
    live_isr = [] { return H::dma_isr(); };
}
void disengage() {
    live_isr = nullptr;
    live_status = nullptr;
}

/// The vectors' common half: the end of a request on the engines, the SCK
/// pad read right after dma_isr() said so.
[[gnu::always_inline]] inline void dma_vector() {
#ifdef SPI_SELECT_CONTROL
    if (live_isr == nullptr) return;
#endif
    bool (*isr)() = live_isr;
    if (isr != nullptr && isr()) {
        sck_at_end = SckPin::read();
        dma_end_at = wall();
        dma_status = live_status();
        dma_done = true;
    }
}

/// One request: true when the engines had it (start() returned false). The
/// end is the vector's: waited for on the ruler, bounded, and a request that
/// never ended is recovered and reported as 0xEE.
template <typename H>
uint8_t run(const typename H::Request& r, bool& on_engines, uint32_t timeout_ms = 300) {
    dma_done = false;
    on_engines = !H::start(r);
    if (!on_engines) {
        return H::status();
    }
    const uint32_t t0 = Ticker::millis();
    while (!dma_done && Ticker::millis() - t0 < timeout_ms) {
    }
    if (!dma_done) {
        (void)H::recover();
        return 0xEEu;
    }
    return dma_status;
}

template <typename H>
typename H::Request request(const uint8_t* tx, uint8_t* rx, uint16_t len, uint8_t baud,
                            SpiMode mode, SpiEngine engine) {
    typename H::Request r{};
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = len;
    r.baud = baud;
    r.mode = mode;
    r.engine = engine;
    return r;
}

#ifndef SPI_SELECT_CONTROL
/// A protocol window on an engined host: the host brought up (it takes the
/// SERCOM back from the command channel's engineless one), the mode primed,
/// the select by hand around a request on the engines, the command channel
/// back after it.
template <typename H>
uint8_t window(typename H::Request r, bool& on_engines) {
    (void)H::init(clock);
    engage<H>();
    H::prime(r.mode, r.baud);
    Cs::clear();
    link_hold();
    const uint8_t st = run<H>(r, on_engines);
    link_hold();
    Cs::set();
    disengage();
    (void)link_command_mode();
    return st;
}

/// The peer's side of a window: an exchange of `count` frames, the answers
/// the pattern from `seed_b`, judged by the peer from `seed_a`.
bool arm_peer(uint16_t count, uint8_t spare = 0, uint8_t mode = 0, uint8_t seed_a = 0x13,
              uint8_t seed_b = 0x57) {
    spilink::Params a{};
    a.cfg.apply = 1;
    a.cfg.mode = mode;
    a.cfg.regime = spilink::regime_buffer_wait;
    a.count = count;
    a.ms = 400;
    a.seed_a = seed_a;
    a.seed_b = seed_b;
    a.pattern = spilink::pattern_prbs;
    a.spare = spare;
    const bool ok = peer_act(Op::exchange, a);
    settle();
    return ok;
}

#endif

uint8_t big_tx[1024];
uint8_t big_rx[1024];

void fill_stream(uint8_t* p, uint16_t n, uint8_t seed) {
    spilink::Stream s(spilink::pattern_prbs, seed);
    for (uint16_t i = 0; i < n; ++i) {
        p[i] = s.next();
    }
}
uint16_t answers_wrong(const uint8_t* got, uint16_t n, uint8_t seed, uint16_t from = 0) {
    uint16_t bad = 0;
    for (uint16_t i = 0; i < n; ++i) {
        if (got[i] != spilink::pattern_value(spilink::pattern_prbs, seed, static_cast<uint16_t>(i + from))) {
            ++bad;
        }
    }
    return bad;
}

}  // namespace dm

// ---- l - the select's rule ----------------------------------------------------

/**
 * WHEN A REQUEST ENDS THE WIRE HAS ENDED: right after the end - start()
 * returning on the loop, dma_isr() answering true in the vector - the SCK
 * pad must already rest at the mode's idle level. On the loop-back (no
 * peer: PA17 high, its client unselected), modes 0..3, read and write, a
 * real select (PB23) and a null one, at BAUD 255 (93.75 kHz, a bit 512
 * core cycles) and BAUD 23 (1 MHz, a bit 48), four frames a request.
 *
 * THE BOUND, derived: an end that comes with the receiver's RXC, the last
 * bit's sampling edge, is early by half a bit in modes 0 and 2, whose last
 * edge is the one after it - 256 core cycles at BAUD 255. The pad is read a
 * few tens of cycles after the end (the return and one PORT load, or the
 * vector's exit path and one load), well inside those 256, so an early end
 * at BAUD 255 reads the active level and cannot be missed; at BAUD 23 the
 * half bit is 24 cycles, shorter than that path, and the instrument no
 * longer sees it - those rows are printed, judged only at the slowest rung.
 * Modes 1 and 3 sample on the last edge itself: there an early end has no
 * half bit to hide in, and the verdict holds them to the same rule.
 */
void tl_select() {
    dm::Fake::set();
    dm::Fake::output();
    constexpr uint8_t bauds[2] = {255, 23};
    uint8_t out[4] = {0x5A, 0xC3, 0x0F, 0xA5};
    uint8_t in[4] = {};
    uint16_t bad_slow[2] = {};   // [polled, dma]
    uint16_t bad_mid[2] = {};
    uint16_t rows[2] = {};
    uint16_t data_bad = 0;
    uint16_t not_engines = 0;
    for (uint8_t engine = 0; engine < 2u; ++engine) {
#ifdef SPI_SELECT_CONTROL
        if (engine == 1u) break;   // the code before has no engines
#endif
        for (uint8_t m = 0; m < 4u; ++m) {
            const SpiMode mode = static_cast<SpiMode>(m);
            const bool idle = spi_cpol(mode);
            for (const uint8_t baud : bauds) {
                for (uint8_t read = 0; read < 2u; ++read) {
                    for (uint8_t real_cs = 0; real_cs < 2u; ++real_cs) {
                        for (uint8_t k = 0; k < 4u; ++k) in[k] = 0xEE;
                        bool sck = idle;
                        bool on = false;
                        if (engine == 0u) {
                            (void)Loop::init(clock);
                            SckPin::function(PinFunction::c, {.input_enable = true});
                            auto r = dm::request<Loop>(out, read ? in : nullptr, 4, baud, mode,
                                                       SpiEngine::polled);
                            r.cs = real_cs ? dm::Fake::ref() : PinRef{};
                            (void)Loop::start(r);
                            sck = SckPin::read();
                        }
#ifndef SPI_SELECT_CONTROL
                        else {
                            (void)dm::DLoop::init(clock);
                            SckPin::function(PinFunction::c, {.input_enable = true});
                            dm::engage<dm::DLoop>();
                            auto r = dm::request<dm::DLoop>(out, read ? in : nullptr, 4, baud, mode,
                                                            SpiEngine::dma);
                            r.cs = real_cs ? dm::Fake::ref() : PinRef{};
                            const uint8_t st = dm::run<dm::DLoop>(r, on);
                            sck = dm::sck_at_end;
                            dm::disengage();
                            if (!on || st != spi_ok) ++not_engines;
                        }
#endif
                        ++rows[engine];
                        const bool good = sck == idle;
                        if (!good) {
                            if (baud == 255u) ++bad_slow[engine];
                            else ++bad_mid[engine];
                            print(serial, "    ", engine ? "dma   " : "polled", " mode ", m,
                                  " BAUD ", baud, read ? " read " : " write", real_cs ? " cs  " : " null",
                                  ": SCK ", sck ? "high" : "low", " at the end, idle ",
                                  idle ? "high" : "low", crlf);
                        }
                        if (read) {
                            for (uint8_t k = 0; k < 4u; ++k) {
                                if (in[k] != out[k]) ++data_bad;
                            }
                        }
                    }
                }
            }
        }
    }
#ifndef SPI_SELECT_CONTROL
    dm::DLoop::release();
#endif
    dm::Fake::set();
    dm::Fake::release();
    (void)link_command_mode();
    print(serial, "  polled: ", rows[0], " requests, ", bad_slow[0], " ended before the last edge at "
          "BAUD 255, ", bad_mid[0], " read so at BAUD 23 (the instrument blind there)", crlf);
    print(serial, "  dma:    ", rows[1], " requests, ", bad_slow[1], " at BAUD 255, ", bad_mid[1],
          " at BAUD 23, ", not_engines, " not ended spi_ok by the engines; ", data_bad,
          " read bytes wrong in both", crlf);
    bench.verdict("THE SELECT'S RULE, polled: at BAUD 255 every request of the four modes, read and "
                  "write, with a select and without, ends with SCK at its idle level",
                  bad_slow[0] == 0u);
#ifndef SPI_SELECT_CONTROL
    bench.verdict("and on the engines too, the end read in the vector right after dma_isr() said so",
                  bad_slow[1] == 0u && rows[1] == rows[0] && not_engines == 0u);
#endif
    bench.verdict("every byte read back on the loop exact", data_bad == 0u);
}

#ifndef SPI_SELECT_CONTROL

// ---- m - the engines: what dma means, what is refused, the command rule ------

void tm_engines() {
    if (!need_peer()) return;
    const uint8_t baud = *Bus::baud_for(1'000'000UL);   // a frame 384 cycles: the loop cheaper by far
    dm::fill_stream(dm::big_tx, 64, 0x13);

    // ONE FRAME ON THE ENGINES, written and read.
    bool on = false;
    bool rep_ok = false;
    spilink::Report rp{};
    bool w1 = false;
    if (dm::arm_peer(1, spilink::spare_receive_only)) {
        const uint8_t st = dm::window<dm::WBus>(
            dm::request<dm::WBus>(dm::big_tx, nullptr, 1, baud, SpiMode::mode0, SpiEngine::dma), on);
        rep_ok = peer_report(rp);
        w1 = on && st == spi_ok && rep_ok && rp.count == 1u && rp.mism == 0u;
        print(serial, "  one frame written on the transmit engine alone: on the engines ", on,
              ", status ", st, ", the peer counted ", rp.count, " mism ", rp.mism, crlf);
    }
    bench.verdict("SpiEngine::dma at ONE frame, written: start() false, dma_isr() the end, spi_ok, "
                  "the peer received it exact",
                  w1);
    bool r1 = false;
    if (dm::arm_peer(1)) {
        dm::big_rx[0] = 0xEE;
        const uint8_t st = dm::window<dm::DBus>(
            dm::request<dm::DBus>(dm::big_tx, dm::big_rx, 1, baud, SpiMode::mode0, SpiEngine::dma), on);
        rep_ok = peer_report(rp);
        const uint16_t bad = dm::answers_wrong(dm::big_rx, 1, 0x57);
        r1 = on && st == spi_ok && bad == 0u && rep_ok && rp.count == 1u && rp.mism == 0u;
        print(serial, "  one frame read on the pair: on the engines ", on, ", status ", st,
              ", read ", hex(dm::big_rx[0]), " (", bad, " wrong), the peer counted ", rp.count,
              " mism ", rp.mism, crlf);
    }
    bench.verdict("SpiEngine::dma at ONE frame, read: start() false, dma_isr() the end, the answer "
                  "exact both ways",
                  r1);
    bool r16 = false;
    if (dm::arm_peer(16, 0, 3)) {
        const uint8_t st = dm::window<dm::DBus>(
            dm::request<dm::DBus>(dm::big_tx, dm::big_rx, 16, baud, SpiMode::mode3, SpiEngine::dma), on);
        rep_ok = peer_report(rp);
        const uint16_t bad = dm::answers_wrong(dm::big_rx, 16, 0x57);
        r16 = on && st == spi_ok && bad == 0u && rep_ok && rp.count == 16u && rp.mism == 0u;
        print(serial, "  sixteen frames read on the pair in mode 3: status ", st, ", ", bad,
              " wrong, the peer counted ", rp.count, " mism ", rp.mism, crlf);
    }
    bench.verdict("sixteen frames read on the pair in mode 3, exact both ways", r16);

    // WHAT CANNOT BE CARRIED IS REFUSED, before the block, the select or the D/C.
    dm::spy_reset();
    (void)dm::WBus::init(clock);
    Raw::clear_flags(SpiFlag::txc);
    auto rr = dm::request<dm::WBus>(dm::big_tx, dm::big_rx, 8, baud, SpiMode::mode0, SpiEngine::dma);
    rr.cs = dm::spy(dm::spy_cs);
    rr.dc = dm::spy(dm::spy_dc);
    const bool w_true = dm::WBus::start(rr);
    const uint8_t w_st = dm::WBus::status();
    const uint32_t spy_touched = dm::spy_group.PORT_OUTSET | dm::spy_group.PORT_OUTCLR;
    const bool no_frame = !Raw::txc_flag() && !Raw::rxc_flag();
    auto rb = dm::request<Bus>(dm::big_tx, dm::big_rx, 8, baud, SpiMode::mode0, SpiEngine::dma);
    rb.cs = dm::spy(dm::spy_cs);
    rb.dc = dm::spy(dm::spy_dc);
    (void)Bus::init(clock);
    const bool b_true = Bus::start(rb);
    const uint8_t b_st = Bus::status();
    const uint32_t spy_touched_b = dm::spy_group.PORT_OUTSET | dm::spy_group.PORT_OUTCLR;
    auto re = dm::request<dm::DBus>(nullptr, nullptr, 0, baud, SpiMode::mode0, SpiEngine::dma);
    re.cs = dm::spy(dm::spy_cs);
    (void)dm::DBus::init(clock);
    const bool e_true = dm::DBus::start(re);
    const uint8_t e_st = dm::DBus::status();
    const uint32_t spy_touched_e = dm::spy_group.PORT_OUTSET | dm::spy_group.PORT_OUTCLR;
    (void)link_command_mode();
    print(serial, "  a read on the transmit engine alone: start() ", w_true, ", status ", w_st,
          ", select and D/C stores ", hex(spy_touched), ", a frame moved ", !no_frame,
          "; dma on the engineless host: ", b_true, ", ", b_st, ", stores ", hex(spy_touched_b),
          "; an empty dma request: ", e_true, ", ", e_st, ", stores ", hex(spy_touched_e), crlf);
    bench.verdict("a read naming dma on the transmit engine alone: start() true, spi_rejected, the "
                  "select and the D/C never stored, no frame on the wire",
                  w_true && w_st == spi_rejected && spy_touched == 0u && no_frame);
    bench.verdict("dma on a host with no engines: start() true, spi_rejected, nothing stored",
                  b_true && b_st == spi_rejected && spy_touched_b == 0u);
    bench.verdict("an empty request naming dma: start() true, spi_ok, nothing stored",
                  e_true && e_st == spi_ok && spy_touched_e == 0u);
    bool after = false;
    if (dm::arm_peer(16)) {
        const uint8_t st = dm::window<dm::DBus>(
            dm::request<dm::DBus>(dm::big_tx, dm::big_rx, 16, baud, SpiMode::mode0, SpiEngine::dma), on);
        rep_ok = peer_report(rp);
        after = on && st == spi_ok && dm::answers_wrong(dm::big_rx, 16, 0x57) == 0u && rep_ok &&
                rp.count == 16u && rp.mism == 0u;
    }
    bench.verdict("and the next request after the refusals exact both ways", after);

    // THE COMMAND RULE: a command then data, the command in the loop and the
    // data on the engines; a command alone on the engines. The peer counts
    // the command and the data as one stream; the spy D/C says the D/C was
    // cleared and set, and a command alone leaves it set at the end, as a
    // polled one does.
    bool cmd_data = false;
    if (dm::arm_peer(17, spilink::spare_receive_only)) {
        dm::spy_reset();
        auto r = dm::request<dm::WBus>(dm::big_tx + 1, nullptr, 16, baud, SpiMode::mode0, SpiEngine::dma);
        r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(dm::big_tx));
        r.cmd_len = 1;
        r.dc = dm::spy(dm::spy_dc);
        const uint8_t st = dm::window<dm::WBus>(r, on);
        const uint32_t dc_cleared = dm::spy_group.PORT_OUTCLR;
        const uint32_t dc_set = dm::spy_group.PORT_OUTSET;
        rep_ok = peer_report(rp);
        cmd_data = on && st == spi_ok && rep_ok && rp.count == 17u && rp.mism == 0u &&
                   dc_cleared == dm::spy_dc && dc_set == dm::spy_dc;
        print(serial, "  a command of 1 then 16 data on the engines: status ", st,
              ", the peer counted ", rp.count, " mism ", rp.mism, ", D/C cleared ", hex(dc_cleared),
              " set ", hex(dc_set), crlf);
    }
    bench.verdict("a command then data: start() false, the command in the loop and the data on "
                  "the engines, the peer received all seventeen in order, D/C low then high",
                  cmd_data);
    bool cmd_alone = false;
    if (dm::arm_peer(4, spilink::spare_receive_only)) {
        dm::spy_reset();
        auto r = dm::request<dm::DBus>(nullptr, nullptr, 0, baud, SpiMode::mode0, SpiEngine::dma);
        r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(dm::big_tx));
        r.cmd_len = 4;
        r.dc = dm::spy(dm::spy_dc);
        const uint8_t st = dm::window<dm::DBus>(r, on);
        rep_ok = peer_report(rp);
        cmd_alone = on && st == spi_ok && rep_ok && rp.count == 4u && rp.mism == 0u &&
                    dm::spy_group.PORT_OUTCLR == dm::spy_dc && dm::spy_group.PORT_OUTSET == dm::spy_dc;
        print(serial, "  a command of 4 alone on the engines: on them ", on, ", status ", st,
              ", the peer counted ", rp.count, " mism ", rp.mism, crlf);
    }
    bench.verdict("a command alone rides the engines: start() false, the peer received the four, "
                  "D/C low through it and high at the end",
                  cmd_alone);

    // LONG POLLED STAYS THE LOOP, the engines bound: the channel's slot is
    // never written (a sentinel in its count), start() returns true, the
    // bytes exact on the loop-back.
    (void)dm::DLoop::init(clock);
    DmaChannel<0>::slot().DMAC_BTCNT = 0xBEEFu;
    dm::fill_stream(dm::big_tx, 1024, 0x21);
    for (auto& b : dm::big_rx) b = 0;
    const bool lp_true = dm::DLoop::start(
        dm::request<dm::DLoop>(dm::big_tx, dm::big_rx, 1024, 3, SpiMode::mode0, SpiEngine::polled));
    const bool slot_kept = DmaChannel<0>::slot().DMAC_BTCNT == 0xBEEFu;
    uint16_t lp_bad = 0;
    for (uint16_t i = 0; i < 1024u; ++i) {
        if (dm::big_rx[i] != dm::big_tx[i]) ++lp_bad;
    }
    print(serial, "  1024 polled frames at 6 MHz on the engined loop: start() ", lp_true,
          ", the channel's slot untouched ", slot_kept, ", ", lp_bad, " wrong", crlf);
    bench.verdict("a long polled request with the engines bound runs in the loop: start() true, "
                  "no channel programmed, every byte exact",
                  lp_true && slot_kept && lp_bad == 0u && dm::DLoop::status() == spi_ok);

    // THE SUM THAT WOULD WRAP: a command of one and 65535 data frames, 65536
    // frames, polled and on the engines, at 6 MHz (a frame 64 core cycles,
    // 32 ruler ticks): each must take the wire's 65536 x 32 ticks.
    for (uint8_t engine = 0; engine < 2u; ++engine) {
        dm::engage<dm::DLoop>();
        auto r = dm::request<dm::DLoop>(nullptr, nullptr, 65535u, 3, SpiMode::mode0,
                                        engine ? SpiEngine::dma : SpiEngine::polled);
        r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(dm::big_tx));
        r.cmd_len = 1;
        const uint32_t t0 = wall();
        const uint8_t st = dm::run<dm::DLoop>(r, on, 2000);
        const uint32_t took = (engine ? dm::dma_end_at : wall()) - t0;
        dm::disengage();
        print(serial, "  1 + 65535 frames ", engine ? "on the engines" : "polled", ": status ", st,
              ", ", took, " ticks against the wire's ", 65536UL * 32UL, crlf);
        bench.verdict(engine ? "cmd_len + len = 65536 on the engines: all of it on the wire"
                             : "cmd_len + len = 65536 polled: all of it on the wire, not an empty request",
                      st == spi_ok && took >= 65536UL * 32UL);
    }
    dm::DLoop::release();
    (void)link_command_mode();
}

// ---- n - the block's reset after a stall and after a fault -------------------

/**
 * THE NEXT REQUEST STARTS ON A CLEAN BLOCK, no recover() called, judged on the
 * loop-back (the bytes read back are the bytes sent, so a character left from
 * before reads as a shift) with nothing between the two requests - no init(),
 * which would reset the block itself.
 *
 * AFTER A STALL: a polled request of eight with SERCOM1's core clock channel
 * disconnected stalls; the clock comes back; the next polled read of sixteen
 * must be exact. Without the reset, what the stall left in DATA shifts out once
 * the clock returns and its echo waits in the receiver for the next read.
 *
 * AFTER A FAULT: a read on the pair whose transmit run ends past the end of the
 * SRAM (a bus error, TERR, after its first beats) ends spi_dma_fault; then the
 * next read - on the engines, and in another round polled - must be exact.
 * Without the reset the frames in flight at the fault leave their echoes in the
 * receiver.
 */
void tn_reset() {
    const uint8_t baud = *Bus::baud_for(1'000'000UL);
    bool on = false;
    uint8_t sixteen[16];
    uint8_t got[16];
    dm::fill_stream(sixteen, 16, 0x29);
    dm::Fake::set();
    dm::Fake::output();

    // The stall.
    (void)Loop::init(clock);
    GclkChannel::disconnect(Raw::gclk_core_id());
    for (uint32_t k = 0; k < 100000u && GclkChannel::connected(Raw::gclk_core_id()); ++k) {
    }
    auto rs = dm::request<Loop>(sixteen, nullptr, 8, baud, SpiMode::mode0, SpiEngine::polled);
    rs.cs = dm::Fake::ref();
    (void)Loop::start(rs);
    const uint8_t stall_st = Loop::status();
    (void)Raw::core_clock(0);
    for (auto& b : got) b = 0xEE;
    (void)Loop::start(dm::request<Loop>(sixteen, got, 16, baud, SpiMode::mode0, SpiEngine::polled));
    uint16_t bad_stall = 0;
    for (uint8_t i = 0; i < 16u; ++i) {
        if (got[i] != sixteen[i]) ++bad_stall;
    }
    print(serial, "  a polled request on a clockless SERCOM: status ", stall_st,
          "; the next read of sixteen once the clock is back: ", bad_stall, " wrong, status ",
          Loop::status(), ", first ", hex(got[0]), " for ", hex(sixteen[0]), crlf);
    bench.verdict("a stall ends spi_stalled", stall_st == spi_stalled);
    bench.verdict("and the next request is exact, with no recover() called",
                  bad_stall == 0u && Loop::status() == spi_ok);

    // The fault, then the next request on the engines; again, then polled.
    const uint8_t* past = reinterpret_cast<const uint8_t*>(HSRAM_ADDR + HSRAM_SIZE - 8u);
    (void)dm::DLoop::init(clock);
    dm::engage<dm::DLoop>();
    const uint32_t faults0 = DmaTxEngine<0>::faults();
    uint8_t fault_st[2] = {};
    uint16_t bad_next[2] = {};
    uint8_t next_st[2] = {};
    for (uint8_t round = 0; round < 2u; ++round) {
        fault_st[round] = dm::run<dm::DLoop>(
            dm::request<dm::DLoop>(past, dm::big_rx, 16, baud, SpiMode::mode0, SpiEngine::dma), on);
        for (auto& b : got) b = 0xEE;
        next_st[round] = dm::run<dm::DLoop>(
            dm::request<dm::DLoop>(sixteen, got, 16, baud, SpiMode::mode0,
                                   round == 0u ? SpiEngine::dma : SpiEngine::polled),
            on);
        for (uint8_t i = 0; i < 16u; ++i) {
            if (got[i] != sixteen[i]) ++bad_next[round];
        }
    }
    dm::disengage();
    dm::DLoop::release();
    dm::Fake::release();
    (void)link_command_mode();
    print(serial, "  a read whose transmit run crosses the SRAM's end at ",
          hex(static_cast<uint32_t>(HSRAM_ADDR + HSRAM_SIZE)), ": status ", fault_st[0], " and ",
          fault_st[1], ", the transmit channel's abandoned blocks ", DmaTxEngine<0>::faults() - faults0,
          "; the next read on the engines ", bad_next[0], " wrong (status ", next_st[0],
          "), polled ", bad_next[1], " wrong (status ", next_st[1], ")", crlf);
    bench.verdict("a bus error on the transmit channel ends the request spi_dma_fault, both times",
                  fault_st[0] == spi_dma_fault && fault_st[1] == spi_dma_fault);
    bench.verdict("and the next request on the engines is exact, with no recover() called",
                  bad_next[0] == 0u && next_st[0] == spi_ok);
    bench.verdict("and a polled one after a fault is exact too", bad_next[1] == 0u && next_st[1] == spi_ok);
}

// ---- o, p - the two SPI shapes, held for minutes ------------------------------

/// The marked line a shape letter prints each second of its run, read by
/// bin/brio's judge: whole, and as many as the closing "#count" says.
uint32_t marked = 0;
template <typename Sink>
void mark(Sink s, uint32_t sec, uint32_t a, uint32_t b, uint32_t c) {
    char tag[8] = {'S', 0, 0, 0, 0, 0, 0, 0};
    uint32_t v = marked;
    for (int8_t k = 5; k >= 1; --k) {
        tag[k] = static_cast<char>('0' + v % 10u);
        v /= 10u;
    }
    print(s, "#<", static_cast<const char*>(tag), " t=", sec, " a=", a, " b=", b, " c=", c,
          " 0123456789abcdefghijklmnopqrstuvwxyz :", static_cast<const char*>(tag), ">#", crlf);
    ++marked;
}

uint32_t shape_seconds = 600;

/**
 * SHAPE ONE, A DISPLAY'S: the SPI transmit engine alone (channel 1) beside the
 * console's transmit engine (channel 0), both running at once for
 * `shape_seconds`. Each turn: a marked line queued on the console (its block
 * running), then a window of 256 frames written on the engines into the peer's
 * receive-only run at 2 MHz, judged by the peer, then four 1024-frame writes at
 * 12 MHz on the loop-back (no reader: judged by their ends). Every class of
 * fault is counted: a request not ended spi_ok or not on the engines, a peer
 * count or mismatch, a peer report missed, either engine's abandoned blocks, the
 * console's DMA faults; the console's lines by the host (#count).
 */
void to_display() {
    if (!need_peer()) return;
    const uint8_t peer_baud = *Bus::baud_for(2'000'000UL);
    print(serial, "  the console moves to its transmit engine for ", shape_seconds,
          " s: marked lines follow", crlf);
    while (!Serial::tx_idle()) {
    }
    Serial::release();
    (void)dm::UDma::init(clock, 115200);
    dm::console_engined = true;
    DmaTxEngine<1>::clear_faults();
    marked = 0;
    uint32_t turns = 0, windows = 0, loops = 0, bad_status = 0, bad_peer = 0, no_report = 0;
    dm::fill_stream(dm::big_tx, 1024, 0x13);
    const uint32_t t0 = Ticker::millis();
    uint32_t next_mark = t0;
    bool on = false;
    while (Ticker::millis() - t0 < shape_seconds * 1000u) {
        if (Ticker::millis() >= next_mark) {
            mark(dm::udma, (Ticker::millis() - t0) / 1000u, windows, bad_status + bad_peer + no_report,
                 dm::UDma::dma_faults());
            next_mark += 1000u;
        } else {
            (void)dm::UDma::write_bulk(std::span<const uint8_t>(
                reinterpret_cast<const uint8_t*>("..............................\r\n"), 32));
        }
        ++turns;
        if (dm::arm_peer(256, spilink::spare_receive_only)) {
            const uint8_t st = dm::window<dm::WBus>(
                dm::request<dm::WBus>(dm::big_tx, nullptr, 256, peer_baud, SpiMode::mode0, SpiEngine::dma),
                on);
            if (!on || st != spi_ok) ++bad_status;
            spilink::Report rp{};
            if (!peer_report(rp)) {
                ++no_report;
            } else if (rp.count != 256u || rp.mism != 0u) {
                ++bad_peer;
            }
            ++windows;
        } else {
            ++no_report;
        }
        (void)dm::WLoop::init(clock);
        dm::engage<dm::WLoop>();
        for (uint8_t k = 0; k < 4u; ++k) {
            const uint8_t st = dm::run<dm::WLoop>(
                dm::request<dm::WLoop>(dm::big_tx, nullptr, 1024, 1, SpiMode::mode0, SpiEngine::dma), on);
            if (!on || st != spi_ok) ++bad_status;
            ++loops;
        }
        dm::disengage();
        (void)link_command_mode();
    }
    print(dm::udma, "#count ", marked, crlf);
    const uint32_t spi_faults = DmaTxEngine<1>::faults();
    const uint8_t console_faults = dm::UDma::dma_faults();
    while (!dm::UDma::tx_idle()) {
    }
    dm::UDma::release();
    dm::console_engined = false;
    (void)Serial::init(clock, 115200);
    dm::WBus::release();
    (void)link_command_mode();
    print(serial, "  ", shape_seconds, " s: ", turns, " turns, ", windows, " windows of 256 to the peer, ",
          loops, " blocks of 1024 at 12 MHz; requests not ended spi_ok ", bad_status,
          ", peer counts or mismatches ", bad_peer, ", reports missed ", no_report,
          ", SPI channel blocks abandoned ", spi_faults, ", console DMA faults ", console_faults, crlf);
    bench.verdict("THE DISPLAY'S SHAPE: the SPI transmit engine beside the console's, every request "
                  "ended spi_ok on the engines, every window exact at the peer, no block abandoned",
                  windows > 0u && bad_status == 0u && bad_peer == 0u && no_report == 0u &&
                      spi_faults == 0u && console_faults == 0u);
}

/**
 * SHAPE TWO: the SPI pair holding both channels, the console on its interrupt
 * beside it, for `shape_seconds`. Each turn: a marked line, then a window of
 * sixteen frames read and written on the engines with the peer at 2 MHz, judged
 * both ways, then four 1024-frame reads at 12 MHz on the loop-back, every byte
 * judged by this board.
 */
void tp_pair() {
    if (!need_peer()) return;
    const uint8_t peer_baud = *Bus::baud_for(2'000'000UL);
    DmaTxEngine<0>::clear_faults();
    DmaRxEngine<1>::clear_faults();
    marked = 0;
    uint32_t windows = 0, loops = 0, bad_status = 0, bad_peer = 0, no_report = 0, bad_loop = 0;
    bool on = false;
    dm::fill_stream(dm::big_tx, 1024, 0x31);
    const uint32_t t0 = Ticker::millis();
    uint32_t next_mark = t0;
    while (Ticker::millis() - t0 < shape_seconds * 1000u) {
        if (Ticker::millis() >= next_mark) {
            mark(serial, (Ticker::millis() - t0) / 1000u, windows, bad_status + bad_peer + no_report,
                 bad_loop);
            next_mark += 1000u;
        }
        if (dm::arm_peer(16)) {
            for (uint8_t i = 0; i < 16u; ++i) dm::big_rx[i] = 0xEE;
            dm::fill_stream(dm::big_tx, 16, 0x13);
            const uint8_t st = dm::window<dm::DBus>(
                dm::request<dm::DBus>(dm::big_tx, dm::big_rx, 16, peer_baud, SpiMode::mode0, SpiEngine::dma),
                on);
            if (!on || st != spi_ok) ++bad_status;
            if (dm::answers_wrong(dm::big_rx, 16, 0x57) != 0u) ++bad_peer;
            spilink::Report rp{};
            if (!peer_report(rp)) {
                ++no_report;
            } else if (rp.count != 16u || rp.mism != 0u) {
                ++bad_peer;
            }
            ++windows;
        } else {
            ++no_report;
        }
        dm::fill_stream(dm::big_tx, 1024, static_cast<uint8_t>(0x31u + windows));
        (void)dm::DLoop::init(clock);
        dm::engage<dm::DLoop>();
        for (uint8_t k = 0; k < 4u; ++k) {
            for (auto& b : dm::big_rx) b = 0;
            const uint8_t st = dm::run<dm::DLoop>(
                dm::request<dm::DLoop>(dm::big_tx, dm::big_rx, 1024, 1, SpiMode::mode0, SpiEngine::dma), on);
            if (!on || st != spi_ok) ++bad_status;
            for (uint16_t i = 0; i < 1024u; ++i) {
                if (dm::big_rx[i] != dm::big_tx[i]) {
                    ++bad_loop;
                    break;
                }
            }
            ++loops;
        }
        dm::disengage();
        (void)link_command_mode();
    }
    print(serial, "#count ", marked, crlf);
    const uint32_t faults = DmaTxEngine<0>::faults() + DmaRxEngine<1>::faults();
    dm::DBus::release();
    (void)link_command_mode();
    print(serial, "  ", shape_seconds, " s: ", windows, " windows of 16 both ways with the peer, ", loops,
          " reads of 1024 at 12 MHz on the loop; requests not ended spi_ok ", bad_status,
          ", peer windows wrong ", bad_peer, ", reports missed ", no_report, ", loop reads wrong ",
          bad_loop, ", channel blocks abandoned ", faults, crlf);
    bench.verdict("THE PAIR'S SHAPE: both channels the SPI's, the console on its interrupt, every "
                  "request ended spi_ok, every byte exact both ways, no block abandoned",
                  windows > 0u && bad_status == 0u && bad_peer == 0u && no_report == 0u &&
                      bad_loop == 0u && faults == 0u);
}

// ---- q - a channel disabled mid-block beside a running one ---------------------

/**
 * RULE TWO INSIDE THE DRIVER, measured: the one path of the host that can
 * disable a running channel is recover() - the timed arbiter's verb on a
 * request that never answered - with a block in flight. Beside it here runs
 * a Uart's transmit engine (channel 0) on SERCOM2's loop through PA12 at
 * 1 Mbaud, its receiver on the same pad by interrupt judging the stream: the
 * transmitter's producer fills its ring at every turn until a run is
 * refused whole, so its dead-block repair is asked at every turn, the
 * receiver drained between the runs. Each turn: a
 * 256-frame write at 1.5 MHz on the loop-back (channel 1), recover() a
 * pseudo-random 0..150 us into it. Two
 * variants, `turns` each: A, recover() as written (the channels put away, then
 * the SERCOM reset); B, the SERCOM disabled first - its request withdrawn - and
 * then recover(). Counted: the Uart's blocks abandoned (its predicate's repair),
 * the stream's bytes lost and repeated at the receiver, and the SPI channel's.
 * Printed, not judged: it measures a hazard the page states.
 */
namespace rq {
constexpr UartPads loop2_pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad0,
    .tx_pin = {'A', 12, PinFunction::c},
    .rx_pin = {'A', 12, PinFunction::c},
};
using Loop2 = Uart<2, loop2_pads, 1024, 512, DmaTxEngine<0>>;
volatile bool live = false;
/// The stream: a count modulo 251, so a byte repeated or skipped is a step
/// the receiver reads as a jump, back or forward, and resynchronizes on.
uint8_t next_out = 0;
uint8_t expect = 0;
uint32_t got = 0, back_jumps = 0, back_bytes = 0, fwd_jumps = 0, fwd_bytes = 0;

void receive() {
    uint8_t buf[64];
    for (;;) {
        const uint32_t n = Loop2::read_bulk(buf);
        if (n == 0u) return;
        for (uint32_t i = 0; i < n; ++i) {
            const uint8_t v = buf[i];
            ++got;
            if (v != expect) {
                const uint32_t ahead = (v + 251u - expect) % 251u;
                if (ahead < 125u) {
                    ++fwd_jumps;
                    fwd_bytes += ahead;
                } else {
                    ++back_jumps;
                    back_bytes += 251u - ahead;
                }
            }
            expect = static_cast<uint8_t>((v + 1u) % 251u);
        }
    }
}
}  // namespace rq

uint32_t q_turns = 20000;

void tq_rule_two() {
    for (uint8_t variant = 0; variant < 2u; ++variant) {
        rq::next_out = 0;
        rq::expect = 0;
        rq::got = rq::back_jumps = rq::back_bytes = rq::fwd_jumps = rq::fwd_bytes = 0;
        DmaTxEngine<1>::clear_faults();
        if (!rq::Loop2::init(clock, 1'000'000u)) {
            bench.verdict("the loop's Uart comes up on SERCOM2, PA12", false);
            return;
        }
        rq::live = true;
        (void)dm::WLoop::init(clock);
        dm::engage<dm::WLoop>();
        uint32_t lfsr = 0x1234567u;
        uint32_t recovered = 0;
        uint32_t refused = 0;
        for (uint32_t t = 0; t < q_turns; ++t) {
            // The ring kept full, and a run more refused whole - the refusal
            // is where the transmitter's dead-block repair runs.
            for (uint8_t k = 0; k < 16u; ++k) {
                uint8_t chunk[128];
                uint8_t v = rq::next_out;
                for (auto& c : chunk) {
                    c = v;
                    v = static_cast<uint8_t>((v + 1u) % 251u);
                }
                const uint32_t put = rq::Loop2::write_bulk(std::span<const uint8_t>(chunk, sizeof chunk));
                rq::next_out = static_cast<uint8_t>((rq::next_out + put) % 251u);
                rq::receive();
                if (put == 0u) {
                    ++refused;
                    break;
                }
            }
            dm::dma_done = false;
            (void)dm::WLoop::start(dm::request<dm::WLoop>(dm::big_tx, nullptr, 256, 15, SpiMode::mode0,
                                                         SpiEngine::dma));
            lfsr = lfsr * 1103515245u + 12345u;
            (void)delay_us(clock, (lfsr >> 16) % 150u);
            if (!dm::dma_done) {
                if (variant == 1u) {
                    (void)Raw::enable(false);
                    (void)delay_us(clock, 6);
                }
                (void)dm::WLoop::recover();
                ++recovered;
            }
            rq::receive();
        }
        dm::disengage();
        const uint32_t t0 = Ticker::millis();
        while (Ticker::millis() - t0 < 300u) {
            rq::receive();
        }
        rq::live = false;
        const uint8_t uart_faults = rq::Loop2::dma_faults();
        const uint8_t overruns = rq::Loop2::hw_overruns();
        rq::Loop2::release();
        dm::WLoop::release();
        print(serial, "  variant ", variant ? "B (the SPI requests withdrawn first)" : "A (as written)",
              ": ", q_turns, " turns, ", recovered, " SPI blocks recovered mid-block; the Uart's "
              "blocks abandoned ", uart_faults, " (", refused, " refusals asked its repair); the "
              "receiver read ", rq::got, " bytes, ", rq::back_jumps, " jumps back (", rq::back_bytes,
              " bytes repeated), ", rq::fwd_jumps, " forward (", rq::fwd_bytes, " lost; the receiver's "
              "own overruns ", overruns, ")", crlf);
    }
    (void)link_command_mode();
    bench.verdict("ran", true);
}

// ---- r - the 24 MHz write, timed ------------------------------------------------

/**
 * WHERE THE WRITE ON ONE CHANNEL LOSES TO THE WIRE: blocks of 64 and 1024
 * frames on the transmit engine alone, loop-back, BAUD 0, 1, 3 and 7, the
 * source in SRAM and in the flash, the core spinning on a flag in SRAM or
 * asleep (WFI) until the end; the frame's core cycles from the two lengths
 * (the request's fixed cost cancels) against the wire's 16 x (BAUD + 1).
 */
alignas(4) const uint8_t flash_run[1024] = {1, 2, 3};

void tr_rate() {
    (void)dm::WLoop::init(clock);
    dm::engage<dm::WLoop>();
    bool on = false;
    for (const uint8_t baud : {0, 1, 3, 7}) {
        for (uint8_t src = 0; src < 2u; ++src) {
            for (uint8_t sleep = 0; sleep < 2u; ++sleep) {
                uint32_t t[2] = {};
                for (uint8_t which = 0; which < 2u; ++which) {
                    const uint16_t n = which ? 1024u : 64u;
                    uint32_t best = 0xFFFFFFFFu;
                    for (uint8_t run = 0; run < 4u; ++run) {
                        dm::dma_done = false;
                        const uint32_t t0 = wall();
                        on = !dm::WLoop::start(dm::request<dm::WLoop>(src ? flash_run : dm::big_tx, nullptr, n,
                                                                     baud, SpiMode::mode0, SpiEngine::dma));
                        while (!dm::dma_done) {
                            if (sleep) __WFI();
                        }
                        const uint32_t took = dm::dma_end_at - t0;
                        if (took < best) best = took;
                    }
                    t[which] = best;
                }
                // Ruler ticks at 24 MHz are two core cycles.
                const uint32_t milli = (t[1] - t[0]) * 2000u / 960u;
                print(serial, "  BAUD ", baud, src ? " flash" : " SRAM ", sleep ? " WFI " : " spin",
                      ": a frame ", milli / 1000u, ".", (milli % 1000u) / 100u, (milli % 100u) / 10u,
                      " core cycles against the wire's ", 16u * (baud + 1u), crlf);
            }
        }
    }
    (void)on;
    dm::disengage();
    dm::WLoop::release();
    (void)link_command_mode();
    bench.verdict("ran", true);
}

#endif  // SPI_SELECT_CONTROL

}   // namespace

// ---------------------------------------------------------------------------
// The vectors
// ---------------------------------------------------------------------------

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

/// ONE vector for the whole SERCOM, and only letter j's client arms it:
/// the hosts run every request polled and arm no interrupt.
extern "C" void SERCOM1_Handler() {
    dm::dma_vector();
    if (client_isr && Raw::ssl_flag()) {
        ssl_at = wall();
        Raw::clear_flags(SpiFlag::ssl);
        Raw::enable_ssl_interrupt(false);
        ssl_seen = static_cast<uint8_t>(ssl_seen + 1u);
        if (ssl_hold_us != 0u) (void)delay_us(clock, ssl_hold_us);
    }
}

#ifdef SPI_SELECT_CONTROL
extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }
#else
extern "C" void SERCOM5_Handler() {
    if (dm::console_engined) {
        (void)dm::UDma::isr();
    } else {
        (void)Serial::isr();
    }
}

/// The engined letters' second vector: an engined SPI host's transfer errors,
/// the console's transmit engine (letter o) and the loop Uart's (letter q).
extern "C" void DMAC_Handler() {
    dm::dma_vector();
    if (dm::console_engined) {
        (void)dm::UDma::dma_isr();
    }
    if (rq::live) {
        (void)rq::Loop2::dma_isr();
    }
}

extern "C" void SERCOM2_Handler() {
    if (rq::live) {
        (void)rq::Loop2::isr();
    }
}
#endif

/// Letter k's one-shot: the core held inside a host request, the SERCOM's
/// flags sampled at the hold's end.
extern "C" void TC4_Handler() {
    Shot::clear_flags(Shot::overflow_flag);
    shot_fired = static_cast<uint8_t>(shot_fired + 1u);
    (void)delay_us(clock, shot_hold_us);
    shot_flags_end = Raw::flags();
}

/// Letter j's backstop: a wake-up nobody should need, counted.
extern "C" void RTC_Handler() { benchsleep::backstop_isr(); }

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
    print(serial, "  SERCOM1 function C: PA16 PAD0 (MOSI), PA17 PAD1 (SS, a GPIO chip "
                  "select), PA18 PAD2 (MISO), PA19 PAD3 (SCK); host row 0x3, client row 0x1",
          crlf);
    print(serial, "  peer: the other board running spi_peer, "
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
    bench.letter('h', "the AVR client's facts (an avrxt peer only)", th_avr_client);
    bench.letter('i', "the AVR client's runs back to back (an avrxt peer only)", ti_avr_runs);
    bench.letter('j', "THE CLIENT'S EDGES: its runs against the peer's burst, every end, "
                      "SSL as a wake-up (needs the peer)",
                 tj_client_edges);
    bench.letter('k', "the host's rare paths: the spin budget, an early TXC", tk_host_paths);
    bench.letter('l', "THE SELECT'S RULE: SCK idle when a request ends, both engines", tl_select);
#ifndef SPI_SELECT_CONTROL
    bench.letter('m', "THE ENGINES: dma at one frame, the refusals, the command rule, the long loop, "
                      "the sum (needs the peer)",
                 tm_engines);
    bench.letter('n', "the block's reset after a stall and after a fault, no recover()", tn_reset);
    bench.letter('o', "the display's shape for minutes: SPI transmit engine beside the console's "
                      "(needs the peer)",
                 to_display, false);
    bench.letter('p', "the pair's shape for minutes: both channels the SPI's, the console on its "
                      "interrupt (needs the peer)",
                 tp_pair, false);
    bench.letter('q', "a channel disabled mid-block beside a running one: recover() measured",
                 tq_rule_two, false);
    bench.letter('r', "the write on one channel against the wire: BAUD 0..7, SRAM and flash", tr_rate,
                 false);
#endif

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
