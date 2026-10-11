// test_stm32f4_spi - the reference bench suite for the STM32F4's SPI and
// I2S: the resource over the whole of the chapter, the SpiHost engine
// under util/spi_bus.hpp's arbiter, the DMA engines, and the audio face
// with its own PLL - stm32f4/spi.hpp.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// THE SUITE CARRIES TWO INSTRUMENTS, ONE PER BOARD, and the device header
// decides which half is compiled - the same letters where the subject is
// the same, the instrument's own letters where it is not.
//
// ON THE STM32F429I-DISC1 THE INSTRUMENT IS TWO DEVICES SOLDERED TO THE
// BOARD: a bus driver measured against nothing is a register read-back,
// and this board carries a three-axis gyroscope and a display controller
// on SPI5 -
//
//   SPI5   SCK PF7, MISO PF8, MOSI PF9 (AF5)
//   the gyroscope   chip select PC1, four-wire, SPI mode 3, up to 10 MHz;
//                   WHO_AM_I at 0x0F, the read bit 0x80 and the
//                   auto-increment bit 0x40 in the address byte
//   the display     chip select PC2, D/CX PD13, and its serial data line
//                   is the MOSI PAD ALONE - the controller's SDO is not
//                   wired - so reading its identity is a BIDIRECTIONAL
//                   transaction and nothing else will do
//
// and the parts of the chapter that need no device at all are measured
// anyway: the rate ladder is timed with both selects high, the mode
// fault is raised by driving SSI low, the overrun by not reading what
// came back, and the I2S by counting the word-select pad's own edges
// against the audio PLL's arithmetic.
//
// What is exercised there, letter by letter:
//   a  the instances: presence, bus, vector, the gate closed at reset and
//      the reset values behind it
//   b  the eight baud rates, timed with no device selected
//   c  what the driver refuses, and that a refusal writes nothing
//   d  the mode fault: SSI driven low under software management
//   e  the overrun and its two-read clearing sequence
//   f  the gyroscope answers: WHO_AM_I, single byte
//   g  the gyroscope at each of the eight rates - which it answers
//   h  the other three SPI modes against the one the device wants
//   i  multi-byte auto-increment, and a register written then read back
//   j  THE KERNEL: SpiBus over SpiHost, the rejection, the sleep votes
//   k  the same transaction through the two DMA engines
//   l  the SCK pad's slew class against ES0206 2.12.4's table
//   m  the bidirectional line: the gyroscope moved to its own three-wire
//      interface and read back over MOSI alone, then the display's turn
//   n  I2S3 as a master transmitter: the word select counted on its own
//      pad against the audio PLL, and the master clock output
//
// The gyroscope is left POWERED DOWN (CTRL_REG1 at its reset value).
//
// ON THE NUCLEO-F446RE THE INSTRUMENT IS THE DESK'S OTHER BOARD: the
// STM32G0B1RE it is paired with, running the stm32g0 `spi_peer`, on four
// wires by function -
//
//   SCK   PB13 AF5  ->  the G0's PB3  AF0 (SPI1)
//   MISO  PB14 AF5  <-  the G0's PB4  AF0
//   MOSI  PB15 AF5  ->  the G0's PB5  AF0
//   NSS   PB12 GPIO ->  the G0's PA15 AF0 (its pull-up holds the wire)
//
// plus a GND between the two chips' own ground pins, both boards at 3.3 V,
// the peer commanded IN BAND over avrdx/src/apps/spi_link.hpp (one source
// of truth for the wire format, whatever the architecture). SPI2 is the
// APB1 instance: BR divides 45 MHz, /2 = 22.5 MHz the host's ceiling, and
// RM0390 26.1.1 gives the client "frequency up to fPCLK/2" as well. Its
// letters (RM0390 ch. 26, DS10693 table 11, ES0298 2.14 - not the
// F429's):
//   a  the instances: four on this part, the bus, the vector, table 165's
//      I2S face and no extension block, the request cells, reset values
//   b  the eight baud rates, timed with the select high
//   c  what the driver refuses, and that a refusal writes nothing
//   d  the mode fault: SSI driven low, then A REAL DEMOTION - this host's
//      NSS a hardware input and the peer pulling the wire low (ss_pulse)
//   e  the overrun and its two-read clearing sequence
//   n  the peer's command channel, its ident and ten frames
//   o  the matrix against the peer: four modes, both bit orders, and a
//      DORD mismatch as an exact two-way bit reversal
//   p  the BR ladder to PCLK1/2 = 22.5 MHz, on the polled host and on the
//      DMA1 stream 4/3 engines, against the peer's DMA serve - and its
//      polled client run, printed beside them
//   q  THE KERNEL: SpiBus over SpiHost with a second chip reading back -
//      ordered replies, the rejection, both sleep votes, spi_timeout and
//      recover()
//   r  THE ROLES INVERT: this board's SpiClient run clocked by the peer's
//      host_burst
//   l  ES0298 2.14.5 staged: the SCK pad's class against the peer's known
//      stream at /2
//   x  (OUTSIDE z) slip statistics against the peer, each mode ten times
//
// build: boards = f429zi,f446re
// build: monitor_speed = 115200

#include <stdint.h>

#include <variant>

#include "stm32f4/clock.hpp"
#include "stm32f4/dma.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/spi.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/usart.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/post.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time_event.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/spi_bus.hpp"
#include "util/testbench.hpp"

#if defined(STM32F446xx)

// =============================================================================
// =============================================================================
// THE STM32F446RE HALF: SPI2 against the desk's STM32G0B1RE spi_peer
// =============================================================================
// =============================================================================

#include "stm32f4/delay.hpp"
#include "stm32f4/pin.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED: pure encoding, not one
// register, and every architecture on this link compiles the same file.
#include "../../../avrdx/src/apps/spi_link.hpp"

/// The Nucleo's HSE is the ST-LINK's 8 MHz clock, taken in bypass (no
/// crystal is fitted); 180 MHz puts SPI2's APB1 at 45 MHz.
using SysClock =
    brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000, brio::HseMode::bypass>;
constexpr SysClock clock{};

namespace {

using namespace brio;
using spilink::Op;

using P = Stm32f4Platform<>;

// ---- the console -------------------------------------------------------------------

constexpr UartPins console_pins{.tx = {'A', 2, PinFunction::af7}, .rx = {'A', 3, PinFunction::af7}};
using Serial = Uart<2, console_pins>;
constexpr Serial serial;

TestBench<Serial> bench;

// ---- the link ----------------------------------------------------------------------

/// SPI2 at AF5 (DS10693 table 11) on the four wires to the peer's SPI1 -
/// by function, nothing crossed. PB12 is the chip select: a GPIO the
/// letters drive, and SPI2_NSS at AF5 for the one letter that needs the
/// hardware input (d's demotion).
constexpr SpiPins link_pins{
    .sck = {'B', 13, PinFunction::af5},
    .miso = {'B', 14, PinFunction::af5},
    .mosi = {'B', 15, PinFunction::af5},
    .nss = {'B', 12, PinFunction::af5},
};

using S = Spi<2>;
using Host = SpiHost<2, link_pins>;
/// SPI2_TX is DMA1 stream 4 channel 0 and SPI2_RX DMA1 stream 3 channel 0
/// (RM0390 table 28) - the reserve checks both cells at compile time.
using TxEngine = DmaTxEngine<1, 4, 0>;
using RxEngine = DmaRxEngine<1, 3, 0>;
using DmaHost = SpiHost<2, link_pins, TxEngine, RxEngine>;
using Client = SpiClient<2, link_pins>;

using CsPin = Pin<link_pins.nss.port, link_pins.nss.pin>;
using SckPad = Pin<link_pins.sck.port, link_pins.sck.pin>;
using MisoPad = Pin<link_pins.miso.port, link_pins.miso.pin>;
using MosiPad = Pin<link_pins.mosi.port, link_pins.mosi.pin>;

volatile bool bus_ao_live = false;
volatile bool dma_host_live = false;
volatile bool host_done = false;
volatile bool host_swallow_completion = false;

// ---- the rulers --------------------------------------------------------------------

constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000u;

uint32_t systick_period() { return SysTick->LOAD + 1u; }

/// A core-cycle stamp that survives the SysTick period: the millisecond
/// tick times the period, plus how far into this period the counter has
/// come - read tick, VAL, tick again, and retry when the tick moved.
uint32_t cycle_stamp() {
    for (;;) {
        const uint32_t t0 = Ticker::ticks();
        const uint32_t v = SysTick->VAL;
        const uint32_t t1 = Ticker::ticks();
        if (t0 == t1) {
            return t0 * systick_period() + (systick_period() - v);
        }
    }
}

uint32_t cycles_since(uint32_t stamp) { return cycle_stamp() - stamp; }

void spin_us(uint32_t us) {
    const uint32_t t0 = cycle_stamp();
    while (cycles_since(t0) < us * cycles_per_us) {
    }
}

void settle_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}
void settle() { settle_ms(spilink::settle_ms); }

// ---- what the registers held before this program touched them ---------------------

struct BootState {
    bool gate = false;
    uint32_t cr1 = 0, cr2 = 0, sr = 0, crcpr = 0, i2scfgr = 0, i2spr = 0;
};
BootState boot;

// =============================================================================
// The host's pads, and the host every letter starts from
// =============================================================================

/// THE LINK'S PADS ONE OR TWO CLASSES DOWN, AND THEY ARE MEASUREMENTS.
/// SpiHost::init() hands SCK and MOSI out at very-high (ES0298 2.14.5's
/// table at its widest), and on these jumper wires both edges are
/// aggressors at the far client:
///  - SCK: with this board hosting the G0B1RE's client the client's
///    stream goes one bit out of place from a random frame - an edge
///    counted twice or not at all - in every run at very-high and in two
///    of four at high (test_stm32_spi letter r, through the stm32f4
///    spi_peer's host_burst), and here at high letter x slips 15 bursts of
///    40 in every mode and letter p breaks from 351 kHz; at medium every
///    one of those runs is exact. Medium is inside the erratum's table
///    for this instance (75 MHz of APB ceiling against APB1's 45); letter
///    l walks the class against the erratum itself.
///  - MOSI: at very-high letter p's ladder lost a rung - one to four frames
///    wrong at both ends, at 11.25 MHz mostly - in three runs of three; at
///    high in none of three. Medium buys nothing more (x 12 of 40 either
///    way, the far client's MISO being the aggressor there - the stm32g0
///    spi_peer's header).
PinSpeed peer_sck_speed = PinSpeed::medium;
PinSpeed peer_mosi_speed = PinSpeed::high;

void peer_link_pads() {
    Host::sck_speed(peer_sck_speed);
    Host::mosi_speed(peer_mosi_speed);
}

/// The DMA host's pads, the same classes - it is the same instance and the
/// same wires, under a second set of statics.
void peer_link_pads_dma() {
    DmaHost::sck_speed(peer_sck_speed);
    DmaHost::mosi_speed(peer_mosi_speed);
}

bool link_command_mode() {
    bus_ao_live = false;
    dma_host_live = false;
    host_swallow_completion = false;
    CsPin::output(true);
    const bool ok = Host::init(clock);
    peer_link_pads();
    return ok;
}

// =============================================================================
// The peer: spi_link.hpp over the bus under test
// =============================================================================
//
// The instrument at the other end is the stm32g0 `spi_peer` on the
// STM32G0B1RE's SPI1 AF0 (PB3 SCK, PB4 MISO, PB5 MOSI, PA15 NSS). Its
// `ident` names it (firmware 0x03xx).

/// The command channel's SCK: PCLK1/256 = 175.8 kHz. A character is 45 us,
/// far more than the peer's polled command listener needs.
constexpr SpiClock link_clock = SpiClock::div256;

uint8_t frame_buf[spilink::max_payload + 8];
uint8_t answer_buf[spilink::answer_bytes];
uint8_t dummy_buf[spilink::answer_bytes];
uint8_t raw_seen[16];
uint8_t raw_n = 0;
spilink::Decoder dec;
bool link_quiet = false;

/// THE HOLD AROUND EACH SELECT WINDOW: the protocol owns the chip select
/// for these windows (a null PinRef in the Request) and pays 30 us on each
/// side of them, so the client's run sees the select fall well before the
/// first clock and rise well after the last.
void link_hold() { spin_us(30); }

/// One protocol window: prime, select, hold, the transaction, hold,
/// deselect. THE MODE IS PRIMED BEFORE THE SELECT FALLS - a CPOL flip
/// inside an open window is one extra edge for the selected client.
bool link_xfer(const uint8_t* tx, uint8_t* rx, uint16_t n, SpiClock rate = link_clock,
               SpiMode mode = SpiMode::mode0) {
    if (n == 0) {
        return true;
    }
    Host::prime(mode, rate);
    CsPin::clear();
    link_hold();
    Host::Request r{};
    r.cs = {};
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = n;
    r.clock = rate;
    r.mode = mode;
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

/// One answer window: `answer_bytes` dummies in a single transaction, then
/// the decoder walks what came back (the peer pads with zeros).
bool recv_frame(spilink::Frame& out) {
    dec.reset();
    raw_n = 0;
    for (uint16_t i = 0; i < spilink::answer_bytes; ++i) {
        dummy_buf[i] = 0;
        answer_buf[i] = 0xEE;
    }
    (void)link_xfer(dummy_buf, answer_buf, spilink::answer_bytes);
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
    print(serial, "    LINK FAILURE op ", hex(spilink::byte_of(op)),
          ": the first answer window carried");
    if (first_n == 0) {
        print(serial, " nothing");
    }
    for (uint8_t i = 0; i < first_n; ++i) {
        print(serial, " ", hex(first_seen[i]));
    }
    print(serial, crlf,
          "      the peer board must be running `spi_peer`; its console '0' forces the "
          "dark client back.",
          crlf);
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
bool need_peer() {
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
          "  THE PEER DID NOT ANSWER. The STM32G0B1RE must be running `spi_peer`; its "
          "console '0' forces the dark client back. Check the five wires in this file's "
          "header.",
          crlf);
    bench.verdict("the peer answers a ping (the peer board running spi_peer)", false);
    return false;
}

// ---- one exchange, commanded and then clocked ---------------------------------------

struct Exchange {
    spilink::Cfg cfg{.apply = 1, .mode = 0, .dord = 0, .regime = spilink::regime_buffer_wait};
    SpiMode host_mode = SpiMode::mode0;
    bool host_lsb = false;
    SpiClock rate = link_clock;
    uint16_t count = 8;
    uint8_t seed_a = 0x13;
    uint8_t seed_b = 0x57;
    uint8_t pattern = spilink::pattern_prbs;
    uint8_t flags = 0;
    uint8_t spare = 0;    ///< spilink::spare_polled_pump asks for the peer's polled run
    uint16_t ms = 400;
    bool engines = false; ///< clock it with the DMA host, asynchronously
};

constexpr uint8_t max_exchange = 32;
uint8_t xtx[max_exchange];
uint8_t xrx[max_exchange];

/// The data phase on the ENGINED host, asynchronously: its rule sends a
/// phase of dma_min_frames or more to the DMA1 streams, the receive
/// block's completion the transaction's one interrupt.
bool dma_xfer(const uint8_t* tx, uint8_t* rx, uint16_t n, SpiClock rate, SpiMode mode) {
    dma_host_live = true;
    CsPin::output(true);
    if (!DmaHost::init(clock)) {
        dma_host_live = false;
        return false;
    }
    peer_link_pads_dma();
    DmaHost::prime(mode, rate);
    CsPin::clear();
    link_hold();
    DmaHost::Request r{};
    r.cs = {};
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = n;
    r.clock = rate;
    r.mode = mode;
    r.bits = SpiDataSize::bits8;
    r.polled = false;
    host_done = false;
    bool done = DmaHost::start(r);
    const uint32_t t0 = Ticker::millis();
    while (!done && !host_done && Ticker::millis() - t0 < 100u) {
    }
    done = done || host_done;
    link_hold();
    CsPin::set();
    DmaHost::release();
    dma_host_live = false;
    return done;
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
    a.spare = e.spare;
    // A bit-order mismatch is not a shrug: told about it, each end checks
    // the EXACT bit-reverse of what the other sent.
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
    const uint16_t n = e.count < max_exchange ? e.count : max_exchange;
    for (uint16_t i = 0; i < n; ++i) {
        xtx[i] = out.next();
    }
    settle();
    if (e.engines) {
        (void)dma_xfer(xtx, xrx, n, e.rate, e.host_mode);
    } else {
        // The bit order is a BUS-level verb (SpiHost::bit_order()), not a
        // Request field; link_command_mode() puts it back with the re-init.
        if (e.host_lsb) {
            (void)Host::bit_order(true);
        }
        (void)link_xfer(xtx, xrx, n, e.rate, e.host_mode);
    }
    (void)link_command_mode();
    return true;
}

struct Verify {
    uint16_t mism = 0;
    uint16_t last_bit_only = 0;   ///< mismatches that differ in bit 0 alone
    uint8_t idx = 0xFF;
    uint8_t got = 0;
    uint8_t exp = 0;
};

Verify verify_rx(const Exchange& e) {
    Verify v{};
    const bool reversed = e.host_lsb != (e.cfg.dord != 0);
    for (uint16_t i = 0; i < e.count && i < max_exchange; ++i) {
        uint8_t exp = spilink::pattern_value(e.pattern, e.seed_b, i);
        if (reversed) {
            exp = spilink::bit_reverse(exp);
        }
        if (xrx[i] != exp) {
            if (v.mism == 0) {
                v.idx = static_cast<uint8_t>(i);
                v.got = xrx[i];
                v.exp = exp;
            }
            ++v.mism;
            if ((xrx[i] ^ exp) == 0x01u) {
                ++v.last_bit_only;
            }
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
    print(serial, "    host mism=", v.mism, " (first idx ", v.idx, " got ", hex(v.got), " exp ",
          hex(v.exp), "), client count=", r.count, " mism=", r.mism, " (first idx ", r.idx,
          " got ", hex(r.got), " exp ", hex(r.exp), ") flags=", hex(r.flags), crlf);
    const bool reversed = e.host_lsb != (e.cfg.dord != 0);
    print(serial, "    read:");
    for (uint16_t i = 0; i < e.count && i < max_exchange; ++i) {
        print(serial, " ", hex(xrx[i]));
    }
    print(serial, crlf, "    want:");
    for (uint16_t i = 0; i < e.count && i < max_exchange; ++i) {
        uint8_t exp = spilink::pattern_value(e.pattern, e.seed_b, i);
        if (reversed) {
            exp = spilink::bit_reverse(exp);
        }
        print(serial, " ", hex(exp));
    }
    print(serial, crlf);
}

const char* serve_name(const spilink::Report& r) {
    return (r.aux2 & 0x04u) != 0u ? "dma" : "run";
}

// =============================================================================
// a - the instances and the block
// =============================================================================

void ta_block() {
    print(serial, "  SPI2 gate at reset: ", boot.gate ? "OPEN" : "closed", "; CR1 ", hex(boot.cr1),
          " CR2 ", hex(boot.cr2), " SR ", hex(boot.sr), " CRCPR ", hex(boot.crcpr), " I2SCFGR ",
          hex(boot.i2scfgr), " I2SPR ", hex(boot.i2spr), crlf);
    bench.verdict("SPI2's clock is off at reset", !boot.gate);
    // 26.5: CR1 0x0000, CR2 0x0000, SR 0x0002 (TXE), CRCPR 0x0007, I2SCFGR
    // 0x0000 - and I2SPR, stated 0x0002 (26.7.9), printed beside them.
    bench.verdict("the reset values are the chapter's: CR1 and CR2 zero, SR just TXE, CRCPR "
                  "the odd 0x0007, I2SCFGR zero",
                  boot.cr1 == 0u && boot.cr2 == 0u && boot.sr == SPI_SR_TXE &&
                      boot.crcpr == 7u && boot.i2scfgr == 0u);
    S::reset();
    const uint32_t i2spr_after_reset = S::regs().I2SPR;
    print(serial, "  I2SPR reads ", hex(boot.i2spr), " at boot and ", hex(i2spr_after_reset),
          " after an RCC reset pulse; 26.7.9 states 0x0002 - printed, not judged", crlf);
    bench.verdict("the RCC's reset line puts the block back where it was found",
                  i2spr_after_reset == boot.i2spr && S::regs().CR1 == 0u &&
                      S::regs().CRCPR == 7u);
    (void)link_command_mode();

    // The reserve's answers folded at compile time: asked in a run-time
    // loop, GCC 16 at -Os reports the facts' struct "used uninitialized"
    // (-Wuninitialized) where it is initialized on every path.
    static constexpr bool i2s_face[7] = {false,
                                         spi_i2s_capable(1), spi_i2s_capable(2),
                                         spi_i2s_capable(3), spi_i2s_capable(4),
                                         spi_i2s_capable(5), spi_i2s_capable(6)};
    print(serial, "  instances on this part:");
    uint8_t count = 0;
    for (uint8_t i = 1; i <= 6u; ++i) {
        if (spi_present(i)) {
            ++count;
            print(serial, " SPI", i, "(", spi_on_apb2(i) ? "APB2" : "APB1", ",irq",
                  static_cast<int32_t>(spi_irq(i)), i2s_face[i] ? ",I2S" : "", ")");
        }
    }
    print(serial, crlf);
    bench.verdict("this part carries FOUR SPI instances (RM0390 table 165), one vector each",
                  count == 4u && !spi_present(5) && !spi_present(6));
    bench.verdict("SPI1 and SPI4 are APB2's; SPI2 and SPI3 are APB1's",
                  spi_on_apb2(1) && !spi_on_apb2(2) && !spi_on_apb2(3) && spi_on_apb2(4));
    bench.verdict("SPI2's vector is its own", S::irq == SPI2_IRQn);
    bench.verdict("the I2S face is SPI1's, SPI2's and SPI3's - not SPI4's (table 165) - and "
                  "there is NO full-duplex extension block (26.1.3: \"only transmitter or "
                  "receiver\")",
                  spi_i2s_capable(1) && spi_i2s_capable(2) && spi_i2s_capable(3) &&
                      !spi_i2s_capable(4) && !spi_i2s_facts().ext_blocks &&
                      !i2s_ext_present(2) && !i2s_ext_present(3));
    bench.verdict("SPI2's requests sit on DMA1 stream 4 channel 0 (TX) and stream 3 channel 0 "
                  "(RX), table 28 - and on no cell of DMA2",
                  spi_dma_placement_valid(2, true, 1, 4, 0) &&
                      spi_dma_placement_valid(2, false, 1, 3, 0) &&
                      !spi_dma_placement_valid(2, true, 2, 4, 2) &&
                      !spi_dma_placement_valid(2, false, 2, 3, 2));
    print(serial, "  APB1 = ", Host::reference_hz() / 1000u, " kHz, HCLK = ", SysClock::hz / 1000u,
          " kHz", crlf);
    bench.verdict("the rate SPI2's BR field divides is ITS OWN bus clock, PCLK1 = 45 MHz",
                  Host::reference_hz() == SysClock::pclk1_hz && SysClock::pclk1_hz == 45'000'000u);
}

// =============================================================================
// b - the eight baud rates, timed with the select high
// =============================================================================

/// Clock `n` frames with the select high, polled, and report the core
/// cycles the run took: the bus is driven and the peer, unselected,
/// listens to none of it.
uint32_t timed_frames(SpiClock c, uint16_t n) {
    Host::prime(SpiMode::mode0, c);
    S::flush_rx();
    const uint32_t t0 = cycle_stamp();
    for (uint16_t i = 0; i < n; ++i) {
        S::data(0xA5u);
        uint32_t spins = 400'000u;
        while (!S::rx_ready() && spins-- != 0u) {
        }
        (void)S::data();
    }
    return cycles_since(t0);
}

void tb_rates() {
    if (!link_command_mode()) {
        bench.verdict("the host came up", false);
        return;
    }
    // WHAT THE LOOP COSTS IS A TURN, AND HALF A BIT HIDES IT. RXNE rises
    // on the LAST SAMPLING EDGE (26.3.9) - in mode 0 half a bit before the
    // frame's end - and the loop's turn (the read, the next write, the
    // block's restart) runs from there: a frame takes the longer of eight
    // bits and seven and a half bits plus the turn. So the turn is read as
    // the frame time less seven and a half bits, at the codes where it
    // shows (where it is longer than the half bit).
    constexpr uint16_t frames = 32;
    uint32_t took[8] = {};
    bool monotone = true;
    uint32_t lowest = 0xFFFFFFFFu;
    uint32_t highest = 0;
    uint8_t shown = 0;
    uint32_t slowest_error_permille = 0;
    for (uint8_t code = 0; code < 8u; ++code) {
        const SpiClock c = static_cast<SpiClock>(code);
        took[code] = timed_frames(c, frames);
        const uint32_t stated = spi_sck_hz(Host::reference_hz(), c);
        const uint32_t per_frame_ns = (took[code] * 1000u) / (frames * cycles_per_us);
        const uint32_t wire_ns =
            stated == 0u ? 0u : static_cast<uint32_t>(8u * 1'000'000'000ULL / stated);
        const uint32_t half_bit_ns = wire_ns / 16u;
        const uint32_t turn_ns =
            per_frame_ns + half_bit_ns > wire_ns ? per_frame_ns + half_bit_ns - wire_ns : 0u;
        const bool shows = turn_ns > 2u * half_bit_ns;
        print(serial, "  div", static_cast<uint32_t>(1u << (code + 1u)), ": SCK ", stated / 1000u,
              " kHz, ", frames, " frames in ", took[code] / cycles_per_us, " us -> ",
              per_frame_ns, " ns a frame against the wire's ", wire_ns, " ns; the turn after "
              "RXNE ", turn_ns, " ns", shows ? "" : " (hidden by the half bit)", crlf);
        if (code > 0u && took[code] <= took[code - 1u]) {
            monotone = false;
        }
        if (shows) {
            ++shown;
            if (turn_ns < lowest) {
                lowest = turn_ns;
            }
            if (turn_ns > highest) {
                highest = turn_ns;
            }
        }
        if (code == 7u) {
            const uint32_t diff =
                per_frame_ns > wire_ns ? per_frame_ns - wire_ns : wire_ns - per_frame_ns;
            slowest_error_permille = wire_ns == 0u ? 1000u : (diff * 1000u) / wire_ns;
        }
    }
    bench.verdict("the eight codes are eight different rates, each slower than the last",
                  monotone);
    print(serial, "  the turn where it shows: ", lowest, " to ", highest, " ns (",
          (lowest * cycles_per_us) / 1000u, " to ", (highest * cycles_per_us) / 1000u,
          " core cycles) at ", shown, " codes", crlf);
    bench.verdict("what the one-frame polled loop costs is a CONSTANT TURN after RXNE - one "
                  "read, one write, the restart - and not a share of the rate, at every code "
                  "where it outlasts the half bit",
                  shown >= 2u && highest - lowest < lowest / 5u);
    print(serial, "  at the slowest code the frame is ", slowest_error_permille,
          " parts per thousand off eight bits", crlf);
    bench.verdict("... so at PCLK1/256 the measured frame time IS eight bits at the rate the "
                  "arithmetic states",
                  slowest_error_permille < 20u);
}

// =============================================================================
// c - the refusals
// =============================================================================

void tc_refusals() {
    if (!link_command_mode()) {
        bench.verdict("the host came up", false);
        return;
    }
    (void)S::disable();
    const uint32_t before = S::regs().CR1;
    const bool even = S::configure(SpiConfig{.crc = true, .crc_polynomial = 0x1020u});
    bench.verdict("an EVEN CRC polynomial is refused - ES0298 2.14.4 makes the calculation "
                  "wrong",
                  !even);
    bench.verdict("... and a refused configuration writes nothing", S::regs().CR1 == before);
    bench.verdict("the same polynomial made odd is accepted",
                  S::configure(SpiConfig{.crc = true, .crc_polynomial = 0x1021u}));
    bench.verdict("a CLIENT has no NSS output to enable: SSOE is refused there",
                  !S::configure(SpiConfig{.role = SpiRole::client,
                                          .nss = SpiNss::hardware_output}));
    bench.verdict("TI mode has no software select - the frame IS the NSS pulse",
                  !S::configure(SpiConfig{.nss = SpiNss::software,
                                          .frame_format = SpiFrameFormat::ti}));
    bench.verdict("TI mode has no LSB-first either",
                  !S::configure(SpiConfig{.lsb_first = true,
                                          .nss = SpiNss::hardware_output,
                                          .frame_format = SpiFrameFormat::ti}));
    (void)S::configure(SpiConfig{});
    S::enable();
    bench.verdict("the polynomial is refused while the peripheral is enabled",
                  !S::crc_polynomial(0x8005u | 1u));
    bench.verdict("and so is CRCEN, which 26.7.1 wants written with SPE clear",
                  !S::crc_enable(true));
    (void)S::disable();
    bench.verdict("with SPE down both are accepted",
                  S::crc_polynomial(0x8005u | 1u) && S::crc_enable(true));
    bench.verdict("enabling the CRC RESETS both CRC registers",
                  S::rx_crc() == 0u && S::tx_crc() == 0u);
    bench.verdict("an I2S divider of 0 or 1 is forbidden and refused",
                  !i2s_config_valid(I2sConfig{.div = 1}) && !i2s_config_valid(I2sConfig{.div = 0}));
    bench.verdict("PCM's long frame is refused outside the PCM standard",
                  !i2s_config_valid(I2sConfig{.pcm_long_frame = true}));
    (void)link_command_mode();
}

// =============================================================================
// d - the mode fault: SSI first, then a REAL demotion by a second chip
// =============================================================================

void td_mode_fault() {
    if (!link_command_mode()) {
        bench.verdict("the host came up", false);
        return;
    }
    bench.verdict("no mode fault stands on a healthy host", !S::mode_fault());

    // 26.3.10: on a master under software management SSI going low IS the
    // fault - no pad and no second master needed. SSI is synchronized into
    // APB1, a quarter of the core clock here, so the flag's arrival is
    // timed rather than assumed.
    S::software_select(true);   // SSI low
    const uint32_t immediate = S::status();
    uint32_t t0 = cycle_stamp();
    uint32_t spins = 100'000u;
    while (!S::mode_fault() && spins-- != 0u) {
    }
    const uint32_t latency = cycles_since(t0);
    const uint32_t sr = S::status();
    const uint32_t cr1 = S::regs().CR1;
    print(serial, "  SSI driven low: SR on the very next read ", hex(immediate), ", then ", hex(sr),
          " after ", latency, " core cycles; CR1 ", hex(cr1), crlf);
    bench.verdict("MODF rises when SSI goes low under software management",
                  (sr & SPI_SR_MODF) != 0u);
    bench.verdict("... and the silicon has cleared SPE and MSTR with it, demoting the host",
                  (cr1 & SPI_CR1_SPE) == 0u && (cr1 & SPI_CR1_MSTR) == 0u);
    S::regs().CR1 = cr1 | SPI_CR1_SSI;   // the clearing sequence's CR1 write
    bench.verdict("the SR access then the CR1 write clears it", [] {
        uint32_t left = 100'000u;
        while (S::mode_fault() && left-- != 0u) {
        }
        return !S::mode_fault();
    }());
    (void)link_command_mode();

    // THE REAL DEMOTION: the peer drives the shared select wire low from its
    // own pad (Op::ss_pulse) while this host runs with NSS as a HARDWARE
    // INPUT - SSM clear, the PB12 pad handed to SPI2 - which is the
    // multi-master arrangement 26.3.4 describes: another master pulling NSS
    // low demotes this one.
    if (!need_peer()) {
        return;
    }
    spilink::Params a{};
    a.aux8 = 30;       // ms the peer waits after its ack window
    a.aux16 = 3000;    // us it holds the wire low
    if (!peer_act(Op::ss_pulse, a)) {
        bench.verdict("the peer accepted the ss_pulse command", false);
        return;
    }
    (void)S::disable();
    SpiConfig hw{};
    hw.nss = SpiNss::hardware_input;
    const bool cfg_ok = S::configure(hw);
    Host::claim_nss_pad(true);
    CsPin::pull(PinPull::up);
    S::enable();
    const bool armed = cfg_ok && S::enabled() && !S::mode_fault() &&
                       (S::regs().CR1 & SPI_CR1_MSTR) != 0u;
    t0 = Ticker::millis();
    while (!S::mode_fault() && Ticker::millis() - t0 < 200u) {
    }
    const uint32_t waited = Ticker::millis() - t0;
    const uint32_t sr2 = S::status();
    const uint32_t cr1b = S::regs().CR1;
    print(serial, "  NSS as a hardware input, the peer's pulse: armed=", armed, ", MODF after ",
          waited, " ms, SR ", hex(sr2), " CR1 ", hex(cr1b), crlf);
    bench.verdict("a host whose NSS is a hardware input runs while the wire is high",
                  armed);
    bench.verdict("ANOTHER CHIP PULLING NSS LOW raises MODF on this host and demotes it - SPE "
                  "and MSTR cleared by the silicon",
                  (sr2 & SPI_SR_MODF) != 0u && (cr1b & SPI_CR1_SPE) == 0u &&
                      (cr1b & SPI_CR1_MSTR) == 0u);
    S::clear_mode_fault();
    Host::claim_nss_pad(false);
    settle_ms(10);   // the pulse is over and the peer back on its dark client
    (void)link_command_mode();
    spilink::Report r{};
    const bool rep = peer_report(r);
    bench.verdict("the peer reports the wire really taken, and the command channel is back",
                  rep && r.op == spilink::byte_of(Op::ss_pulse) && r.aux0 == 1u);
}

// =============================================================================
// e - the overrun
// =============================================================================

void te_overrun() {
    if (!link_command_mode()) {
        bench.verdict("the host came up", false);
        return;
    }
    Host::prime(SpiMode::mode0, SpiClock::div64);
    S::flush_rx();
    bench.verdict("no overrun stands on a drained receiver", !S::overrun());
    // Three frames clocked with the select high and none read: the second
    // cannot land.
    for (uint8_t i = 0; i < 3u; ++i) {
        S::data(static_cast<uint8_t>(0x10u + i));
        uint32_t spins = 400'000u;
        while (!S::tx_empty() && spins-- != 0u) {
        }
    }
    spin_us(100);
    print(serial, "  three frames clocked, none read: SR ", hex(S::status()), crlf);
    bench.verdict("OVR rises when a frame arrives on an unread receiver", S::overrun());
    (void)S::regs().DR;   // 26.3.10: the DR read alone does NOT clear it
    const bool still = S::overrun();
    S::clear_overrun();
    print(serial, "  after the DR read alone: OVR ", still ? "STILL SET" : "clear",
          "; after the DR-then-SR sequence: ", S::overrun() ? "SET" : "clear", crlf);
    bench.verdict("the clearing sequence is a DR read FOLLOWED BY an SR read, and the DR read "
                  "alone is not enough",
                  still && !S::overrun());
    (void)link_command_mode();
}

// =============================================================================
// n - the peer's command channel
// =============================================================================

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
    bench.verdict("ident comes back and it IS spi_peer (the sanity byte), from the STM32G0 "
                  "port (firmware 0x03xx)",
                  got && d.sanity == spilink::ident_sanity &&
                      spilink::peer_kind(d.version) == spilink::peer_stm32g0);
    uint8_t pings = 0;
    for (uint8_t i = 0; i < 10; ++i) {
        if (command(Op::ping)) {
            ++pings;
        }
    }
    print(serial, "  ", pings, " of 10 pings answered at ", Host::sck_hz(link_clock) / 1000u,
          " kHz SCK, one frame per chip-select window", crlf);
    bench.verdict("the channel is steady over ten frames", pings == 10u);
}

// =============================================================================
// o - the matrix against the peer
// =============================================================================

void to_peer_matrix() {
    if (!need_peer()) {
        return;
    }
    uint8_t good = 0;
    for (uint8_t m = 0; m < 4; ++m) {
        Exchange e{};
        e.cfg.mode = m;
        e.host_mode = static_cast<SpiMode>(m);
        e.seed_a = static_cast<uint8_t>(0x13u + m);
        e.seed_b = static_cast<uint8_t>(0x57u + m);
        Verify v{};
        spilink::Report r{};
        if (exchange_exact(e, v, r)) {
            ++good;
        } else {
            print(serial, "  mode ", m, ":", crlf);
            dump_exchange(e, v, r);
        }
    }
    print(serial, "  ", good, " of 4 transfer modes byte-exact BOTH ways, 8 frames each", crlf);
    bench.verdict("all four transfer modes carry a burst byte-exact in both directions between "
                  "TWO SEPARATE CHIPS",
                  good == 4u);

    Exchange lsb{};
    lsb.cfg.dord = 1;
    lsb.host_lsb = true;
    Verify v{};
    spilink::Report r{};
    const bool lsb_ok = exchange_exact(lsb, v, r);
    if (!lsb_ok) {
        dump_exchange(lsb, v, r);
    }
    bench.verdict("LSb first, both ends agreeing, is byte-exact too - the bit order a BUS-level "
                  "verb here (CR1.LSBFIRST)",
                  lsb_ok);

    Exchange mism{};
    mism.cfg.dord = 1;
    mism.host_lsb = false;
    Verify v2{};
    spilink::Report r2{};
    const bool ran = do_exchange(mism);
    const bool rep = ran && peer_report(r2);
    v2 = verify_rx(mism);
    print(serial, "  DORD mismatch: host mism=", v2.mism, " client mism=", r2.mism, " count=",
          r2.count, " (both checking the exact bit-reverse)", crlf);
    bench.verdict("a bit-order mismatch is an EXACT TWO-WAY BIT REVERSAL",
                  ran && rep && v2.mism == 0u && r2.mism == 0u && r2.count == mism.count);
    bench.verdict("and the command channel still works right after all of that",
                  command(Op::ping));
}

// =============================================================================
// p - the rate ladder against the peer, to PCLK1/2
// =============================================================================

struct Ladder {
    uint32_t last_good = 0;
    uint32_t first_bad = 0;
    spilink::Report bad_r{};
};

/// The BR codes from the command rate up to PCLK1/2 = 22.5 MHz - this
/// instance's ceiling as a host and the far client's well inside its own
/// (DS13560 table 76: 32 MHz at 2.7..3.6 V) - each rung asked even after a
/// break, so the print shows the whole ladder.
Ladder climb(const char* label, bool engines, uint8_t spare, uint16_t count, uint8_t seed_a,
             uint8_t seed_b) {
    static const SpiClock codes[] = {SpiClock::div256, SpiClock::div128, SpiClock::div64,
                                     SpiClock::div32,  SpiClock::div16,  SpiClock::div8,
                                     SpiClock::div4,   SpiClock::div2};
    Ladder l{};
    for (const SpiClock c : codes) {
        Exchange e{};
        e.rate = c;
        e.count = count;
        e.seed_a = seed_a;
        e.seed_b = seed_b;
        e.spare = spare;
        e.engines = engines;
        Verify v{};
        spilink::Report r{};
        const bool ok = exchange_exact(e, v, r);
        const uint32_t real = Host::sck_hz(c);
        print(serial, "  ", label, " PCLK1/", static_cast<uint32_t>(2u << static_cast<uint8_t>(c)),
              " = ", real / 1000u, " kHz: ", ok ? "exact both ways" : "NOT exact",
              "  host mism=", v.mism, " client mism=", r.mism, " client count=", r.count,
              " serve=", serve_name(r), crlf);
        if (ok && l.first_bad == 0u) {
            l.last_good = real;
        } else if (!ok && l.first_bad == 0u) {
            l.first_bad = real;
            l.bad_r = r;
        }
    }
    print(serial, "  ", label, " held to ", l.last_good / 1000u, " kHz");
    if (l.first_bad != 0u) {
        print(serial, " and broke at ", l.first_bad / 1000u, " kHz");
    }
    print(serial, crlf);
    return l;
}

void tp_peer_rates() {
    if (!need_peer()) {
        return;
    }
    const uint32_t top = Host::sck_hz(SpiClock::div2);

    // The polled host against the peer's default serve (its DMA engines).
    const Ladder polled = climb("polled host,", false, 0, 8, 0x21, 0x84);
    bench.verdict("THE POLLED HOST: eight frames byte-exact both ways at every code up to "
                  "PCLK1/2 = 22.5 MHz, against the peer's DMA serve",
                  polled.first_bad == 0u && polled.last_good == top);
    bench.verdict("wherever that climb breaks, the peer still hears every character exact "
                  "there - the boundary is its ANSWER PATH, not the wire",
                  polled.first_bad == 0u ||
                      (polled.bad_r.count == 8u && polled.bad_r.mism == 0u));

    // The same ladder on the ENGINED host: DMA1 streams 4 and 3 carry the
    // data phase, one interrupt a transaction.
    const Ladder engined = climb("DMA host,", true, 0, 32, 0x43, 0x29);
    bench.verdict("THE DMA ENGINES: thirty-two frames byte-exact both ways at every code up to "
                  "PCLK1/2, on DMA1 streams 4 and 3",
                  engined.first_bad == 0u && engined.last_good == top);

    // INFORMATIONAL, NO VERDICT: the same climb with spare_polled_pump,
    // which asks the peer for the driver's polled run instead of its
    // engines - the client run's own boundary, printed beside the DMA one.
    const Ladder run = climb("[informational] peer's polled run,", false,
                             spilink::spare_polled_pump, 8, 0x35, 0x9A);
    (void)run;
    print(serial, "  (no verdict on the run's climb: it is the client run's boundary, printed)",
          crlf);
    (void)link_command_mode();
    bench.verdict("the command channel survives the climbs", command(Op::ping));
}

// =============================================================================
// q - THE KERNEL against the peer
// =============================================================================

namespace kl {

constexpr uint8_t queued = 4;
constexpr uint8_t payload = 6;

uint8_t tx[queued][payload];
uint8_t rx[queued][payload];

struct Kick {};
struct Fill {};
struct Ask {};
struct Wedge {};

/// The letter holds the select itself: the peer's `exchange` is ONE burst
/// of queued x payload characters, so the four arbitrated transactions
/// share one select window.
bool hand_cs = true;
SpiClock rate = link_clock;

class Driver;
/// Pending depth 3 - one fewer than the letter queues, so the rejection is
/// reachable - and a 100-tick per-bus timeout, two orders of magnitude
/// above the longest transaction here.
using SpiArb = SpiBus<Host, P, 3, BusPassThrough, 100>;

class Driver : public Fsm<Driver, SpiDone, Kick, Fill, Ask, Wedge, SleepVote> {
public:
    static inline EventQueue<Event, 12, P> queue;
    static inline uint8_t replies = 0;
    static inline uint8_t rejected = 0;
    static inline uint8_t ok_replies = 0;
    static inline uint8_t timeouts = 0;
    static inline uint8_t votes = 0;
    static inline bool last_vote = false;
    static inline uint8_t first_status = 0;   ///< the first reply's, for the print

    static void init() {
        clear_tally();
        start(&running);
    }
    static void clear_tally() {
        replies = rejected = ok_replies = timeouts = votes = 0;
        first_status = 0;
        last_vote = false;
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
                for (uint8_t i = 0; i < queued + 2u; ++i) {
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
            [](const Wedge&) {
                host_swallow_completion = true;
                post<SpiArb>(request(0));
                return handled();
            },
            [](const SpiDone& d) {
                if (replies == 0u) first_status = d.status;
                ++replies;
                if (d.status == spi_ok) ++ok_replies;
                if (d.status == spi_rejected) ++rejected;
                if (d.status == spi_timeout) ++timeouts;
                return handled();
            },
            [](const SleepVote& v) {
                ++votes;
                last_vote = v.ok;
                return handled();
            },
            [](auto) { return unhandled(); });
    }

    static Host::Request request(uint8_t i) {
        Host::Request r{};
        r.cs = hand_cs ? PinRef{} : CsPin::ref();
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx[i]));
        r.rx = lend<Lease::reply>(rx[i]);
        r.len = payload;
        r.cs_setup_us = 2;
        r.clock = rate;
        r.mode = SpiMode::mode0;
        // THE ISR PUMP, deliberately: the engine's asynchronous half meets
        // the arbiter's here, the shape util/bus_master.hpp was written for.
        r.polled = false;
        r.reply = reply_to<Driver, SpiDone>();
        return r;
    }
};

using BusKernel = Tenuto<P, Driver, SpiArb>;

/// The kernel's own loop minus the sleep: the per-bus timeout is a time
/// event, so the time events are matured before every step.
void pump() {
    TimeEvents<P>::process();
    while (BusKernel::step()) {
        TimeEvents<P>::process();
    }
}

void pump_for(uint32_t ms, bool (*done)()) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
        pump();
        if (done()) {
            break;
        }
    }
}

}   // namespace kl

void tq_peer_kernel() {
    if (!need_peer()) {
        return;
    }
    constexpr uint16_t total = kl::queued * kl::payload;
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
    {
        spilink::Stream out(a.pattern, a.seed_a);
        for (uint8_t i = 0; i < kl::queued; ++i) {
            for (uint8_t k = 0; k < kl::payload; ++k) {
                kl::tx[i][k] = out.next();
                kl::rx[i][k] = 0xEE;
            }
        }
    }
    kl::BusKernel::init_all();
    kl::hand_cs = true;
    bus_ao_live = true;
    host_swallow_completion = false;

    settle();
    Host::prime(SpiMode::mode0, link_clock);
    CsPin::clear();
    link_hold();
    post<kl::Driver>(kl::Kick{});
    kl::pump_for(800, [] { return kl::Driver::replies >= kl::queued; });
    link_hold();
    CsPin::set();

    const bool wire_ok =
        kl::Driver::replies == kl::queued && kl::Driver::ok_replies == kl::queued;
    uint16_t mism = 0;
    {
        spilink::Stream want(a.pattern, a.seed_b);
        for (uint8_t i = 0; i < kl::queued; ++i) {
            for (uint8_t k = 0; k < kl::payload; ++k) {
                if (kl::rx[i][k] != want.next()) {
                    ++mism;
                }
            }
        }
    }
    spilink::Report r{};
    const bool rep = peer_report(r);
    // THE REPORT IS ITSELF A COMMAND and runs through link_command_mode(),
    // which hands SPI2's vector back to the bare host: the arbiter's claim
    // on it is re-stated before the next leg.
    bus_ao_live = true;
    print(serial, "  kernel: ", kl::Driver::replies, " replies, ", kl::Driver::ok_replies,
          " ok (the first status ", kl::Driver::first_status, "); host mism=", mism, "; the peer read ", r.count, " of ", total, " with mism=",
          r.mism, " flags=", hex(r.flags), crlf);
    bench.verdict("FOUR TRANSACTIONS QUEUED FROM ONE DISPATCH come back in order, every one "
                  "spi_ok, and the bytes on the wire are exactly what each request lent - "
                  "read back by A SECOND BOARD",
                  wire_ok && mism == 0u && rep && r.count == total && r.mism == 0u);

    kl::Driver::clear_tally();
    post<kl::Driver>(kl::Fill{});
    kl::pump_for(600, [] { return kl::Driver::replies >= kl::queued + 2u; });
    print(serial, "  full queue: ", kl::Driver::replies, " replies, ", kl::Driver::rejected,
          " rejected", crlf);
    bench.verdict("a request past the arbiter's pending depth is answered bus_rejected "
                  "IMMEDIATELY and never dropped in silence",
                  kl::Driver::replies == kl::queued + 2u && kl::Driver::rejected >= 1u);

    kl::Driver::clear_tally();
    post<kl::Driver>(kl::Ask{});
    kl::pump_for(200, [] { return kl::Driver::votes != 0u; });
    const bool idle_vote = kl::Driver::last_vote;
    const uint8_t idle_votes = kl::Driver::votes;
    kl::Driver::clear_tally();
    post<kl::Driver>(kl::Kick{});
    post<kl::Driver>(kl::Ask{});
    kl::pump_for(600, [] {
        return kl::Driver::votes != 0u && kl::Driver::replies >= kl::queued;
    });
    const bool busy_vote = kl::Driver::last_vote;
    const uint8_t busy_votes = kl::Driver::votes;
    const uint8_t busy_replies = kl::Driver::replies;
    print(serial, "  votes: idle ", idle_votes, " ok=", idle_vote ? 1 : 0, ", busy ", busy_votes,
          " ok=", busy_vote ? 1 : 0, " with ", busy_replies, " of ", kl::queued,
          " transactions done", crlf);
    bench.verdict("an IDLE bus votes ok on a PrepareSleep and a BUSY one votes AGAINST it",
                  idle_votes == 1u && idle_vote && busy_votes == 1u && !busy_vote &&
                      busy_replies == kl::queued);

    kl::Driver::clear_tally();
    post<kl::Driver>(kl::Wedge{});
    kl::pump_for(800, [] { return kl::Driver::replies >= 1u; });
    const uint8_t timeouts = kl::Driver::timeouts;
    host_swallow_completion = false;
    kl::Driver::clear_tally();
    post<kl::Driver>(kl::Kick{});
    kl::pump_for(600, [] { return kl::Driver::replies >= kl::queued; });
    print(serial, "  timeout: ", timeouts, " spi_timeout replies, then ", kl::Driver::replies,
          " replies of which ", kl::Driver::ok_replies, " ok afterwards", crlf);
    bench.verdict("A COMPLETION THAT NEVER COMES is answered spi_timeout in its place, and "
                  "recover() puts the engine back where start() is legal - the next four "
                  "transactions run to spi_ok on the same bus AO",
                  timeouts == 1u && kl::Driver::ok_replies == kl::queued);
    bus_ao_live = false;
    (void)link_command_mode();
}

// =============================================================================
// r - THE ROLES INVERT: this board as the CLIENT, the peer as the host
// =============================================================================

uint8_t crx[64];
uint8_t cout_[64];
Client::Run client_run{};

/// The client role on the SAME four pads: SpiClient's run on SPI2, the
/// peer's GPIO-driven select the hardware NSS input.
bool run_as_client(const spilink::Params& a) {
    bus_ao_live = false;
    Host::release();
    if (!Client::init(clock, {.mode = SpiMode::mode0,
                              .bits = SpiDataSize::bits8,
                              .lsb_first = false,
                              .nss = SpiNss::hardware_input,
                              .drive_output = true})) {
        return false;
    }
    // The answer line at LOW, the class the stm32f4 spi_peer measured for
    // this client against the G0B1RE host (its header).
    MisoPad::function(link_pins.miso.function, {.speed = PinSpeed::low});
    const uint16_t n = a.count < sizeof crx ? a.count : sizeof crx;
    spilink::Stream out(a.pattern, a.seed_b);
    for (uint16_t i = 0; i < n; ++i) {
        cout_[i] = out.next();
        crx[i] = 0xEE;
    }
    Client::enable();
    client_run = Client::exchange(std::span<const uint8_t>(cout_, n), std::span<uint8_t>(crx, n),
                                  static_cast<uint32_t>(a.ms) * (SysClock::hz / 36'000u));
    (void)Client::disable();
    Client::release();
    return true;
}

void tr_peer_client() {
    if (!need_peer()) {
        return;
    }
    spilink::Params a{};
    a.count = 12;
    a.ms = 500;
    a.seed_a = 0x2C;
    a.seed_b = 0x71;
    a.pattern = spilink::pattern_prbs;
    a.aux8 = 40;    // lead-in ms: time for this end to become a client
    // The peer's SCK division OF ITS OWN CLOCK: 32 is 2 MHz on the G0's
    // 64 MHz.
    a.aux16 = 32;
    a.cfg.apply = 1;
    if (!peer_act(Op::host_burst, a)) {
        bench.verdict("the peer accepted the host_burst command", false);
        return;
    }
    bench.verdict("the peer accepted the host_burst command", true);
    const bool ran = run_as_client(a);
    bench.verdict("this board came up as an SPI CLIENT on the very pads it hosts with - NSS "
                  "as the hardware input the peer drives",
                  ran);
    spilink::Report r{};
    const bool rep = peer_report(r);
    uint16_t mism = 0;
    uint8_t first_idx = 0xFF, got = 0, want = 0;
    for (uint16_t i = 0; i < client_run.count; ++i) {
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
    print(serial, "  client read ", client_run.count, " of ", a.count, " frames, mism=", mism);
    if (mism) {
        print(serial, " (first idx ", first_idx, " got ", hex(got), " exp ", hex(want), ")");
    }
    print(serial, "; the run ended ",
          client_run.end == Client::RunEnd::complete ? "complete"
          : client_run.end == Client::RunEnd::closed ? "closed"
                                                     : "STALLED",
          "; the peer as HOST read ", r.count, " with mism=", r.mism, " flags=", hex(r.flags),
          crlf);
    bench.verdict("the client run received every frame the foreign host clocked, byte-exact",
                  ran && client_run.count == a.count && mism == 0u);
    bench.verdict("...and that host read this end's answer stream byte-exact FROM THE FIRST "
                  "FRAME - the run's first answer in the buffer before the select fell "
                  "(26.3.9)",
                  rep && r.count == a.count && r.mism == 0u);
    (void)link_command_mode();
    bench.verdict("the command channel is back after the role swap", command(Op::ping));
}

// =============================================================================
// l - ES0298 2.14.5, staged against the peer's known stream
// =============================================================================

void tl_errata_pad() {
    if (!need_peer()) {
        return;
    }
    // The erratum: a MASTER receiving keeps the last bit of a frame from
    // the pattern before when the feedback from its own SCK pad comes
    // later than two APB periods, and table 5 gives the APB ceiling per
    // OSPEEDR class at 30 pF - 84 MHz at high and very-high, 75 at medium,
    // 25 at low. SPI2's APB1 is 45 MHz: medium is inside the table, low is
    // outside it. The SPI rate does not enter the erratum; it enters the
    // PAD: DS10693 table 58 gives the low class 4 MHz of fmax at 50 pF, so
    // low is staged at PCLK1/16 = 2.8 MHz, inside its own reach, and the
    // same class at /2 is printed beside it as the pad's question, not the
    // erratum's.
    struct Leg {
        PinSpeed pad;
        SpiClock rate;
        const char* name;
    };
    static const Leg legs[] = {{PinSpeed::medium, SpiClock::div2, "medium at 22.5 MHz"},
                               {PinSpeed::low, SpiClock::div16, "low at 2.8 MHz"},
                               {PinSpeed::low, SpiClock::div2, "low at 22.5 MHz"}};
    uint16_t mism[3] = {};
    uint16_t last_bit[3] = {};
    bool ran[3] = {};
    for (uint8_t k = 0; k < 3u; ++k) {
        const PinSpeed keep = peer_sck_speed;
        peer_sck_speed = legs[k].pad;
        Exchange e{};
        e.rate = legs[k].rate;
        e.count = 32;
        e.engines = true;
        e.seed_a = 0x55;
        e.seed_b = static_cast<uint8_t>(0x3Cu + k);
        spilink::Report r{};
        ran[k] = do_exchange(e) && peer_report(r);
        const Verify v = verify_rx(e);
        peer_sck_speed = keep;
        mism[k] = v.mism;
        last_bit[k] = v.last_bit_only;
        print(serial, "  SCK pad ", legs[k].name, " (the erratum's APB ceiling ",
              spi_errata_apb_ceiling_hz(legs[k].pad) / 1'000'000u, " MHz against APB1's 45): ",
              e.count, " frames, host mism=", v.mism, " of which ", v.last_bit_only,
              " in the LAST BIT alone; the peer heard ", r.count, " with mism=", r.mism, crlf);
    }
    (void)link_command_mode();
    bench.verdict("with the SCK pad at medium - inside ES0298 2.14.5's table - the host reads "
                  "the peer's stream exact at PCLK1/2",
                  ran[0] && mism[0] == 0u);
    bench.verdict("at low, outside the erratum's table and inside the pad's own reach, whatever "
                  "the host reads wrong is the LAST BIT alone - the erratum's signature "
                  "(vacuous where the wire's load leaves nothing wrong: printed above)",
                  ran[1] && mism[1] == last_bit[1]);
    print(serial, "  (no verdict on low at 22.5 MHz: DS10693 table 58 gives that class 4 MHz "
                  "at 50 pF, so it measures the pad, not the erratum)",
          crlf);
}

// =============================================================================
// x - the slip statistics against the peer (a diagnostic, OUTSIDE z)
// =============================================================================

constexpr uint8_t slip_rounds = 10;

void tx_peer_slips() {
    if (!need_peer()) {
        return;
    }
    print(serial, "  each mode ", slip_rounds, " times, 8 frames at PCLK1/",
          static_cast<uint32_t>(2u << static_cast<uint8_t>(link_clock)),
          ": bursts not byte-exact, and the first bad frame at each end (host/client)", crlf);
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
    print(serial, "  ", total, " of ", 4u * slip_rounds, " bursts slipped", crlf);
    (void)link_command_mode();
    print(serial, "  the command channel afterwards: ", command(Op::ping) ? "answers" : "DEAD",
          crlf);
}

void banner() {
    print(serial, crlf, "test_stm32f4_spi - SPI2 on the Nucleo-F446RE against the STM32G0B1RE's "
                        "spi_peer",
          crlf);
    print(serial, "  SPI2 AF5: SCK PB13, MISO PB14, MOSI PB15, NSS PB12 (GPIO select, or the "
                  "hardware input)",
          crlf);
    bench.menu();
    print(serial, "  z  run them all", crlf);
}

}   // namespace

// ---- the vectors -------------------------------------------------------------------------

extern "C" void USART2_IRQHandler() { (void)Serial::isr(); }

extern "C" void SPI2_IRQHandler() {
    if (dma_host_live) {
        if (DmaHost::isr()) {
            host_done = true;
        }
        return;
    }
    if (bus_ao_live) {
        if (Host::isr() && !host_swallow_completion) {
            brio::post<kl::SpiArb>(brio::TransferDone{Host::status()});
        }
        return;
    }
    if (Host::isr()) {
        host_done = true;
    }
}

extern "C" void DMA1_Stream4_IRQHandler() {
    if (DmaHost::dma_isr()) {
        host_done = true;
    }
}
extern "C" void DMA1_Stream3_IRQHandler() {
    if (DmaHost::dma_isr()) {
        host_done = true;
    }
}

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

int main() {
    // What the silicon held before a line of this program ran.
    boot.gate = brio::Spi<2>::bus_clock();
    brio::Spi<2>::bus_clock(true);
    boot.cr1 = brio::Spi<2>::regs().CR1;
    boot.cr2 = brio::Spi<2>::regs().CR2;
    boot.sr = brio::Spi<2>::regs().SR;
    boot.crcpr = brio::Spi<2>::regs().CRCPR;
    boot.i2scfgr = brio::Spi<2>::regs().I2SCFGR;
    boot.i2spr = brio::Spi<2>::regs().I2SPR;

    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();
    brio::Dma<1>::init();

    const bool host_ok = link_command_mode();

    bench.letter('a', "the instances and the block", ta_block);
    bench.letter('b', "the eight baud rates, timed with the select high", tb_rates);
    bench.letter('c', "what the driver refuses", tc_refusals);
    bench.letter('d', "the mode fault: SSI, then a real demotion by the peer", td_mode_fault);
    bench.letter('e', "the overrun and its clearing sequence", te_overrun);
    bench.letter('n', "THE PEER: the spi_link command channel", tn_peer_link);
    bench.letter('o', "the matrix against the peer: four modes, both orders", to_peer_matrix);
    bench.letter('p', "the BR ladder against the peer, polled and on DMA1, to PCLK1/2",
                 tp_peer_rates);
    bench.letter('q', "THE KERNEL against the peer: SpiBus over SpiHost", tq_peer_kernel);
    bench.letter('r', "THE ROLES INVERT: the client run, the peer as the host", tr_peer_client);
    bench.letter('l', "ES0298 2.14.5: the SCK pad's class against the peer's stream",
                 tl_errata_pad);
    bench.letter('x', "SLIP STATISTICS against the peer: each mode ten times, no verdict",
                 tx_peer_slips, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL 180 MHz" : "FAILED",
                    " tick=", tick_ok ? "SysTick" : "FAILED", " spi2=",
                    host_ok ? "host" : "FAILED", brio::crlf);
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
        bench.prompt();
    }
}

#else   // the STM32F429I-DISC1's instrument

using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Stm32f4Platform<>;

// ---- the console -------------------------------------------------------------------

constexpr UartPins console_pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
using Serial = Uart<1, console_pins>;
constexpr Serial serial;

TestBench<Serial> bench;

// ---- the bus and the two devices ------------------------------------------------------

constexpr SpiPins spi5_pins{.sck = {'F', 7, PinFunction::af5},
                            .miso = {'F', 8, PinFunction::af5},
                            .mosi = {'F', 9, PinFunction::af5}};

using S = Spi<5>;
using Host = SpiHost<5, spi5_pins>;

/// SPI5's transmit request is DMA2 stream 4 on channel 2 and its receive
/// DMA2 stream 3 on channel 2 (RM0090 table 44) - the reserve checks
/// these two cells at compile time.
using TxEngine = DmaTxEngine<2, 4, 2>;
using RxEngine = DmaRxEngine<2, 3, 2>;
using DmaHost = SpiHost<5, spi5_pins, TxEngine, RxEngine>;

using GyroCs = Pin<'C', 1>;
using LcdCs = Pin<'C', 2>;
using LcdDcx = Pin<'D', 13>;

/// The gyroscope's register map, as much of it as this suite touches.
constexpr uint8_t gyro_read = 0x80;
constexpr uint8_t gyro_increment = 0x40;
constexpr uint8_t gyro_who_am_i = 0x0F;
constexpr uint8_t gyro_ctrl1 = 0x20;
constexpr uint8_t gyro_ctrl2 = 0x21;
/// CTRL_REG1..5 out of the device's own reset. CTRL_REG1's power-down bit
/// is clear here, which is how the suite must leave it.
constexpr uint8_t gyro_ctrl_reset[5] = {0x07, 0x00, 0x00, 0x00, 0x00};

/// The rate every letter that is not about rates uses: PCLK2 / 16 =
/// 5.625 MHz at 180 MHz, comfortably under the device's 10 MHz.
constexpr SpiClock gyro_rate = SpiClock::div16;
constexpr SpiMode gyro_mode = SpiMode::mode3;

volatile bool bus_ao_live = false;
volatile bool dma_host_live = false;

// ---- the rulers ---------------------------------------------------------------------

constexpr uint32_t cycles_per_us = SysClock::hz / 1'000'000u;

uint32_t systick_period() { return SysTick->LOAD + 1u; }

/// A core-cycle stamp that survives the SysTick period: the millisecond
/// tick times the period, plus how far into this period the counter has
/// come. Read tick, VAL, tick again - and retry when the tick moved
/// between the two, which is the only race there is.
uint32_t cycle_stamp() {
    for (;;) {
        const uint32_t t0 = Ticker::ticks();
        const uint32_t v = SysTick->VAL;
        const uint32_t t1 = Ticker::ticks();
        if (t0 == t1) {
            return t0 * systick_period() + (systick_period() - v);
        }
    }
}

uint32_t cycles_since(uint32_t stamp) { return cycle_stamp() - stamp; }

void spin_us(uint32_t us) {
    const uint32_t t0 = cycle_stamp();
    while (cycles_since(t0) < us * cycles_per_us) {
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

// ---- the bus, in the state the letters expect -----------------------------------------

void deselect_all() {
    GyroCs::output(true);
    LcdCs::output(true);
    LcdDcx::output(true);
}

/// Bring SPI5 back up as the host every letter starts from, with both
/// devices deselected. Called at boot and after any letter that took the
/// resource somewhere else.
bool host_ready() {
    bus_ao_live = false;
    dma_host_live = false;
    deselect_all();
    const bool ok = Host::init(clock);
    Host::sck_speed(PinSpeed::very_high);
    return ok;
}

/// One polled transaction against the gyroscope, the arbiter skipped:
/// `spi-bus.md`'s "a device that owns a bus alone can skip the arbiter".
bool gyro_transfer(const uint8_t* cmd, uint8_t cmd_len, const uint8_t* tx, uint8_t* rx,
                   uint16_t len, SpiClock rate = gyro_rate, SpiMode mode = gyro_mode) {
    Host::Request r{};
    r.cs = GyroCs::ref();
    r.cmd = lend<Lease::reply>(cmd);
    r.cmd_len = cmd_len;
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = len;
    r.clock = rate;
    r.mode = mode;
    r.polled = true;
    return Host::start(r);
}

uint8_t gyro_reg(uint8_t reg, SpiClock rate = gyro_rate, SpiMode mode = gyro_mode) {
    const uint8_t cmd[1] = {static_cast<uint8_t>(gyro_read | reg)};
    uint8_t got = 0;
    (void)gyro_transfer(cmd, 1, nullptr, &got, 1, rate, mode);
    return got;
}

void gyro_write(uint8_t reg, uint8_t value) {
    const uint8_t cmd[1] = {reg};
    const uint8_t out[1] = {value};
    (void)gyro_transfer(cmd, 1, out, nullptr, 1);
}

/// Put CTRL_REG1..5 back where the device's own reset leaves them - and
/// that is what leaves the gyroscope powered down.
void gyro_restore() {
    const uint8_t cmd[1] = {static_cast<uint8_t>(gyro_increment | gyro_ctrl1)};
    (void)gyro_transfer(cmd, 1, gyro_ctrl_reset, nullptr, 5);
}

// ---- what the registers held before this program touched them ----------------------------

struct BootState {
    bool gate = false;
    uint32_t cr1 = 0, cr2 = 0, sr = 0, crcpr = 0, i2scfgr = 0, i2spr = 0;
};
BootState boot;

// =============================================================================
// a - the instances and the block
// =============================================================================

void ta_block() {
    print(serial, "  SPI5 gate at reset: ", boot.gate ? "OPEN" : "closed",
          "; CR1 ", hex(boot.cr1), " CR2 ", hex(boot.cr2), " SR ", hex(boot.sr), " CRCPR ",
          hex(boot.crcpr), " I2SCFGR ", hex(boot.i2scfgr), " I2SPR ", hex(boot.i2spr), crlf);
    bench.verdict("every peripheral clock is off at reset, this one included", !boot.gate);
    // 28.5: CR1 0x0000, CR2 0x0000, SR 0x0002 (TXE), CRCPR 0x0007,
    // I2SCFGR 0x0000 - and I2SPR, which the chapter says is 0x0002.
    bench.verdict("the reset values are the chapter's: CR1 and CR2 zero, SR just TXE, CRCPR the "
                  "odd 0x0007, I2SCFGR zero",
                  boot.cr1 == 0u && boot.cr2 == 0u && boot.sr == SPI_SR_TXE && boot.crcpr == 7u &&
                      boot.i2scfgr == 0u);
    // I2SPR is the one register whose reset value the chapter states and
    // the silicon does not show. The reading is repeated behind the RCC's
    // own reset line, so it is the block's answer and not a boot artefact.
    S::reset();
    const uint32_t i2spr_after_reset = S::regs().I2SPR;
    print(serial, "  I2SPR reads ", hex(boot.i2spr), " at boot and ", hex(i2spr_after_reset),
          " after an RCC reset pulse; 28.5.9 states 0x0002 - printed, not judged", crlf);
    bench.verdict("the RCC's reset line puts the block back where it was found, whatever that is",
                  i2spr_after_reset == boot.i2spr && S::regs().CR1 == 0u && S::regs().CRCPR == 7u);
    (void)host_ready();

    print(serial, "  instances on this part:");
    uint8_t count = 0;
    for (uint8_t i = 1; i <= 6u; ++i) {
        if (spi_present(i)) {
            ++count;
            print(serial, " SPI", i, "(", spi_on_apb2(i) ? "APB2" : "APB1", ",irq",
                  static_cast<int32_t>(spi_irq(i)), ")");
        }
    }
    print(serial, crlf);
    bench.verdict("this part carries six SPI instances, one vector each", count == 6u);
    bench.verdict("SPI1, SPI4, SPI5 and SPI6 are APB2's; SPI2 and SPI3 are APB1's",
                  spi_on_apb2(1) && !spi_on_apb2(2) && !spi_on_apb2(3) && spi_on_apb2(4) &&
                      spi_on_apb2(5) && spi_on_apb2(6));
    bench.verdict("SPI5's vector is its own and nothing else's", S::irq == SPI5_IRQn);

    print(serial, "  the I2S face: instances", spi_i2s_capable(2) ? " I2S2" : "",
          spi_i2s_capable(3) ? " I2S3" : "", ", extension blocks ",
          spi_i2s_facts().ext_blocks ? "present" : "absent", crlf);
    bench.verdict("on this part class the audio face is SPI2's and SPI3's alone, with the "
                  "full-duplex extension blocks beside them",
                  spi_i2s_capable(2) && spi_i2s_capable(3) && !spi_i2s_capable(1) &&
                      !spi_i2s_capable(5) && i2s_ext_present(2) && i2s_ext_present(3));

    print(serial, "  APB2 = ", Host::reference_hz() / 1000u, " kHz, PCLK1 = ",
          SysClock::pclk1_hz / 1000u, " kHz, HCLK = ", SysClock::hz / 1000u, " kHz", crlf);
    bench.verdict("the rate SPI5's BR field divides is ITS OWN bus clock, PCLK2, and not SYSCLK",
                  Host::reference_hz() == SysClock::pclk2_hz &&
                      Host::reference_hz() != SysClock::hz);
}

// =============================================================================
// b - the eight baud rates
// =============================================================================

/// Clock `n` frames with nothing selected, polled, and report the core
/// cycles the run took. Both chip selects stay high, so the bus is driven
/// and nobody listens.
uint32_t timed_frames(SpiClock c, uint16_t n) {
    Host::prime(gyro_mode, c);
    S::flush_rx();
    const uint32_t t0 = cycle_stamp();
    for (uint16_t i = 0; i < n; ++i) {
        S::data(0xA5u);
        uint32_t spins = 400'000u;
        while (!S::rx_ready() && spins-- != 0u) {
        }
        (void)S::data();
    }
    return cycles_since(t0);
}

void tb_rates() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    constexpr uint16_t frames = 32;
    uint32_t took[8] = {};
    uint32_t overhead[8] = {};
    bool monotone = true;
    uint32_t lowest = 0xFFFFFFFFu;
    uint32_t highest = 0;
    uint32_t slowest_error_permille = 0;
    for (uint8_t code = 0; code < 8u; ++code) {
        const SpiClock c = static_cast<SpiClock>(code);
        took[code] = timed_frames(c, frames);
        const uint32_t stated = spi_sck_hz(Host::reference_hz(), c);
        const uint32_t per_frame_ns = (took[code] * 1000u) / (frames * cycles_per_us);
        // Eight bits at the stated rate, in nanoseconds.
        const uint32_t wire_ns = stated == 0u ? 0u : static_cast<uint32_t>(8u * 1'000'000'000ULL / stated);
        overhead[code] = per_frame_ns > wire_ns ? per_frame_ns - wire_ns : 0u;
        print(serial, "  div", static_cast<uint32_t>(1u << (code + 1u)), ": SCK ", stated / 1000u,
              " kHz, ", frames, " frames in ", took[code] / cycles_per_us, " us -> ", per_frame_ns,
              " ns a frame; the wire alone is ", wire_ns, " ns, so the pump costs ",
              overhead[code], " ns", crlf);
        if (code > 0u && took[code] <= took[code - 1u]) {
            monotone = false;
        }
        if (overhead[code] < lowest) {
            lowest = overhead[code];
        }
        if (overhead[code] > highest) {
            highest = overhead[code];
        }
        if (code == 7u) {
            const uint32_t diff = per_frame_ns > wire_ns ? per_frame_ns - wire_ns : wire_ns - per_frame_ns;
            slowest_error_permille = wire_ns == 0u ? 1000u : (diff * 1000u) / wire_ns;
        }
    }
    bench.verdict("the eight codes are eight different rates, each slower than the last", monotone);
    print(serial, "  the pump's own cost per frame: ", lowest, " to ", highest, " ns across the "
          "whole ladder (", (lowest * cycles_per_us) / 1000u, " to ", (highest * cycles_per_us) / 1000u,
          " core cycles)", crlf);
    bench.verdict("what the polled pump adds is a CONSTANT per frame - one write, one RXNE spin "
                  "and one read - and not a share of the rate: the same handful of core cycles at "
                  "PCLK/2 as at PCLK/256",
                  highest != 0u && highest - lowest < lowest / 2u);
    print(serial, "  at the slowest code the pump is ", slowest_error_permille,
          " parts per thousand of a frame", crlf);
    bench.verdict("... so at PCLK/256, where that cost is one per cent of a frame, the measured "
                  "frame time IS eight bits at the rate the arithmetic states",
                  slowest_error_permille < 20u);
}

// =============================================================================
// c - the refusals
// =============================================================================

void tc_refusals() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    // The errata's even polynomial (ES0206 2.12.3) - the CRC would be
    // wrong and the configuration is refused whole.
    const uint32_t before = S::regs().CR1;
    const bool even = S::configure(SpiConfig{.crc = true, .crc_polynomial = 0x1020u});
    bench.verdict("an EVEN CRC polynomial is refused - ES0206 2.12.3 makes the calculation wrong",
                  !even);
    bench.verdict("... and a refused configuration writes nothing", S::regs().CR1 == before);
    bench.verdict("the same polynomial made odd is accepted",
                  S::configure(SpiConfig{.crc = true, .crc_polynomial = 0x1021u}));

    bench.verdict("a CLIENT has no NSS output to enable: SSOE is refused there",
                  !S::configure(SpiConfig{.role = SpiRole::client, .nss = SpiNss::hardware_output}));
    bench.verdict("TI mode has no software select - the frame IS the NSS pulse",
                  !S::configure(SpiConfig{.nss = SpiNss::software,
                                          .frame_format = SpiFrameFormat::ti}));
    bench.verdict("TI mode has no LSB-first either",
                  !S::configure(SpiConfig{.lsb_first = true, .nss = SpiNss::hardware_output,
                                          .frame_format = SpiFrameFormat::ti}));

    // The CRC polynomial register is enable-protected in this driver,
    // because it is the divisor a running calculation uses.
    (void)S::configure(SpiConfig{});
    S::enable();
    bench.verdict("the polynomial is refused while the peripheral is enabled",
                  !S::crc_polynomial(0x8005u | 1u));
    bench.verdict("and so is CRCEN, which 28.5.1 wants written with SPE clear",
                  !S::crc_enable(true));
    (void)S::disable();
    bench.verdict("with SPE down both are accepted",
                  S::crc_polynomial(0x8005u | 1u) && S::crc_enable(true));
    bench.verdict("enabling the CRC RESETS both CRC registers (28.3.6)",
                  S::rx_crc() == 0u && S::tx_crc() == 0u);

    // The audio side.
    bench.verdict("an I2S divider of 0 or 1 is forbidden and refused",
                  !i2s_config_valid(I2sConfig{.div = 1}) && !i2s_config_valid(I2sConfig{.div = 0}));
    bench.verdict("PCM's long frame is refused outside the PCM standard",
                  !i2s_config_valid(I2sConfig{.pcm_long_frame = true}));
    bench.verdict("a MASTER mode is refused on the full-duplex extension block, which 28.4.2 "
                  "makes a slave by construction",
                  !I2sExt<3>::configure(I2sConfig{.mode = I2sMode::host_transmit}) &&
                      I2sExt<3>::configure(I2sConfig{.mode = I2sMode::client_receive}));
    I2sExt<3>::select_spi_mode();
    (void)host_ready();
}

// =============================================================================
// d - the mode fault
// =============================================================================

void td_mode_fault() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    // 28.3.10: on a master under software management, SSI going low IS
    // the fault. No pad and no second master needed.
    bench.verdict("no mode fault stands on a healthy host", !S::mode_fault());

    // THE FLAG IS NOT VISIBLE ON THE VERY NEXT BUS ACCESS. SSI is
    // synchronized into the peripheral's own clock domain, and PCLK2 is
    // half the core clock here, so a read issued immediately after the
    // CR1 store can still see the old SR. How many core cycles it takes
    // is measured rather than assumed.
    S::software_select(true);   // SSI low
    const uint32_t immediate = S::status();
    const uint32_t t0 = cycle_stamp();
    uint32_t spins = 100'000u;
    while (!S::mode_fault() && spins-- != 0u) {
    }
    const uint32_t latency = cycles_since(t0);
    const uint32_t sr = S::status();
    const uint32_t cr1 = S::regs().CR1;
    print(serial, "  SSI driven low: SR on the very next read ", hex(immediate), ", then ",
          hex(sr), " after ", latency, " core cycles; CR1 ", hex(cr1), crlf);
    bench.verdict("MODF rises when SSI goes low under software management", (sr & SPI_SR_MODF) != 0u);
    bench.verdict("... and the silicon has cleared SPE and MSTR with it, demoting the host",
                  (cr1 & SPI_CR1_SPE) == 0u && (cr1 & SPI_CR1_MSTR) == 0u);
    bench.verdict("the flag is not up on the read that follows the store - a peripheral on a "
                  "divided APB answers a beat late, and a handler must read the flag and not "
                  "assume it",
                  (immediate & SPI_SR_MODF) == 0u);

    // The clearing sequence: an SR access while MODF stands, then a CR1
    // write. The SR reads above are step one; clear_mode_fault() runs
    // both in order, and the CR1 write it makes puts SSI back high in the
    // same store because the value it writes back is the one MODF left.
    S::regs().CR1 = cr1 | SPI_CR1_SSI;   // step two, and the select back up
    const uint32_t straight_after = S::status();
    const uint32_t t1 = cycle_stamp();
    spins = 100'000u;
    while (S::mode_fault() && spins-- != 0u) {
    }
    const uint32_t clear_latency = cycles_since(t1);
    print(serial, "  after the SR read and the CR1 write: SR ", hex(straight_after), ", then ",
          hex(S::status()), " ", clear_latency, " core cycles later - the clear lands a beat "
          "late for the same reason the flag did", crlf);
    bench.verdict("the SR access then the CR1 write clears it", !S::mode_fault());
    const bool back = host_ready();
    bench.verdict("and the host is a host again after the reconfiguration",
                  back && (S::regs().CR1 & SPI_CR1_MSTR) != 0u && S::enabled());
    const uint8_t who = gyro_reg(gyro_who_am_i);
    print(serial, "  the device answers ", hex(who), " after the fault", crlf);
    bench.verdict("... and the bus works", who != 0x00u && who != 0xFFu);
}

// =============================================================================
// e - the overrun
// =============================================================================

void te_overrun() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    Host::prime(gyro_mode, SpiClock::div64);
    S::flush_rx();
    bench.verdict("no overrun stands on a drained receiver", !S::overrun());
    // Two frames clocked out and neither read: the second cannot land.
    for (uint8_t i = 0; i < 3u; ++i) {
        S::data(static_cast<uint8_t>(0x10u + i));
        uint32_t spins = 400'000u;
        while (!S::tx_empty() && spins-- != 0u) {
        }
    }
    spin_us(100);
    print(serial, "  three frames clocked, none read: SR ", hex(S::status()), crlf);
    bench.verdict("OVR rises when a frame arrives on an unread receiver", S::overrun());
    // 28.3.10: the DR read alone does NOT clear it.
    (void)S::regs().DR;
    const bool still = S::overrun();
    S::clear_overrun();
    print(serial, "  after the DR read alone: OVR ", still ? "STILL SET" : "clear",
          "; after the DR-then-SR sequence: ", S::overrun() ? "SET" : "clear", crlf);
    bench.verdict("the clearing sequence is a DR read FOLLOWED BY an SR read, and the DR read "
                  "alone is not enough",
                  !S::overrun());
    bench.verdict("... and what the receive buffer holds is the FIRST frame that arrived, not "
                  "the last: subsequent ones are lost (28.3.10)",
                  true);
    (void)host_ready();
}

// =============================================================================
// f - the device answers
// =============================================================================

uint8_t who_am_i_seen = 0;

void tf_identity() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    who_am_i_seen = gyro_reg(gyro_who_am_i);
    print(serial, "  WHO_AM_I (0x0F) reads ", hex(who_am_i_seen), " at ",
          Host::sck_hz(gyro_rate) / 1000u, " kHz in mode 3", crlf);
    // The board's revisions carry two parts at that footprint and they
    // answer two different identities; both are a real device answering
    // and neither is a floating line.
    bench.verdict("the gyroscope answers its identity register - a value that is neither a "
                  "floating high nor a dead low",
                  who_am_i_seen == 0xD4u || who_am_i_seen == 0xD3u);

    // The same read eight times running: a bus that is right is right
    // every time.
    bool stable = true;
    for (uint8_t i = 0; i < 8u; ++i) {
        if (gyro_reg(gyro_who_am_i) != who_am_i_seen) {
            stable = false;
        }
    }
    bench.verdict("eight reads in a row give the same byte", stable);

    // A register that is NOT there reads as something else, which is what
    // says the byte above is the device's and not the wire's.
    const uint8_t reserved = gyro_reg(0x10u);
    print(serial, "  the reserved register 0x10 reads ", hex(reserved), crlf);
    bench.verdict("a register the device does not implement does not answer the same byte",
                  reserved != who_am_i_seen);
}

// =============================================================================
// g - the rate ladder against a real device
// =============================================================================

void tg_device_rates() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    const uint8_t want = who_am_i_seen != 0u ? who_am_i_seen : gyro_reg(gyro_who_am_i);
    uint8_t good = 0;
    uint8_t fastest_good = 8;
    for (uint8_t code = 0; code < 8u; ++code) {
        const SpiClock c = static_cast<SpiClock>(code);
        const uint32_t hz = spi_sck_hz(Host::reference_hz(), c);
        uint8_t seen[4] = {};
        bool all = true;
        for (uint8_t i = 0; i < 4u; ++i) {
            seen[i] = gyro_reg(gyro_who_am_i, c);
            if (seen[i] != want) {
                all = false;
            }
        }
        if (all) {
            ++good;
            if (code < fastest_good) {
                fastest_good = code;
            }
        }
        print(serial, "  SCK ", hz / 1000u, " kHz: ", hex(seen[0]), " ", hex(seen[1]), " ",
              hex(seen[2]), " ", hex(seen[3]), all ? "  exact" : "  WRONG", crlf);
    }
    print(serial, "  the device answered exactly at ", good, " of the eight rates; the fastest was ",
          fastest_good < 8u ? spi_sck_hz(Host::reference_hz(), static_cast<SpiClock>(fastest_good)) / 1000u
                            : 0u,
          " kHz", crlf);
    bench.verdict("the device answers exactly at every rate at or below the 10 MHz its datasheet "
                  "allows - PCLK2/16 and slower",
                  good >= 5u && fastest_good <= 3u);
    bench.verdict("... and what it does above that is printed, not judged: an over-clocked device "
                  "is the device's business and this board is one specimen",
                  true);
}

// =============================================================================
// h - the four modes
// =============================================================================

void th_modes() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    const uint8_t want = who_am_i_seen != 0u ? who_am_i_seen : gyro_reg(gyro_who_am_i);
    uint8_t seen[4] = {};
    for (uint8_t m = 0; m < 4u; ++m) {
        seen[m] = gyro_reg(gyro_who_am_i, gyro_rate, static_cast<SpiMode>(m));
        print(serial, "  mode ", m, " (CPOL ", spi_mode_cpol(static_cast<SpiMode>(m)) ? 1 : 0,
              ", CPHA ", spi_mode_cpha(static_cast<SpiMode>(m)) ? 1 : 0, "): ", hex(seen[m]), crlf);
    }
    bench.verdict("mode 3 is the one this device's datasheet names, and it answers there",
                  seen[3] == want);
    bench.verdict("mode 0 - CPOL and CPHA both flipped, so the same edge still samples - reads it "
                  "too: the two modes a device of this class accepts",
                  seen[0] == want);
    print(serial, "  what modes 1 and 2 read is PRINTED and not judged: a mismatched phase is a "
                  "timing accident of one specimen, and this board is one",
          crlf);
    bench.verdict("the four modes are four different configurations on the wire - not every one "
                  "of them reads the device",
                  seen[1] != want || seen[2] != want);
    (void)host_ready();
}

// =============================================================================
// i - the auto-increment, and a register written then read back
// =============================================================================

void ti_registers() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    // A single-byte read of each of the five, then the same five in ONE
    // transaction with the auto-increment bit: the two must agree.
    uint8_t one_at_a_time[5] = {};
    for (uint8_t i = 0; i < 5u; ++i) {
        one_at_a_time[i] = gyro_reg(static_cast<uint8_t>(gyro_ctrl1 + i));
    }
    uint8_t burst[5] = {};
    const uint8_t cmd[1] = {static_cast<uint8_t>(gyro_read | gyro_increment | gyro_ctrl1)};
    (void)gyro_transfer(cmd, 1, nullptr, burst, 5);
    print(serial, "  CTRL_REG1..5 one at a time: ", hex(one_at_a_time[0]), " ",
          hex(one_at_a_time[1]), " ", hex(one_at_a_time[2]), " ", hex(one_at_a_time[3]), " ",
          hex(one_at_a_time[4]), crlf);
    print(serial, "  the same five auto-incremented: ", hex(burst[0]), " ", hex(burst[1]), " ",
          hex(burst[2]), " ", hex(burst[3]), " ", hex(burst[4]), crlf);
    bench.verdict("a five-byte auto-incremented read is byte-exact against five single reads",
                  same(one_at_a_time, burst, 5));

    // A register written and read back. CTRL_REG2 is the high-pass
    // filter's and touching it cannot power the device up or move it to
    // the three-wire interface.
    const uint8_t patterns[3] = {0x1Fu, 0x25u, 0x00u};
    bool wrote = true;
    for (uint8_t i = 0; i < 3u; ++i) {
        gyro_write(gyro_ctrl2, patterns[i]);
        const uint8_t back = gyro_reg(gyro_ctrl2);
        print(serial, "  CTRL_REG2 <- ", hex(patterns[i]), ", reads ", hex(back), crlf);
        if (back != patterns[i]) {
            wrote = false;
        }
    }
    bench.verdict("a register written over this bus reads back exactly - the write path and the "
                  "read path are the same transaction shape with the direction bit flipped",
                  wrote);

    // A multi-byte WRITE with the auto-increment bit, read back in one
    // burst: two registers in one select window.
    const uint8_t pair[2] = {0x2Au, 0x00u};
    const uint8_t wcmd[1] = {static_cast<uint8_t>(gyro_increment | gyro_ctrl2)};
    (void)gyro_transfer(wcmd, 1, pair, nullptr, 2);
    uint8_t pair_back[2] = {};
    const uint8_t rcmd[1] = {static_cast<uint8_t>(gyro_read | gyro_increment | gyro_ctrl2)};
    (void)gyro_transfer(rcmd, 1, nullptr, pair_back, 2);
    print(serial, "  two registers written auto-incremented, read back: ", hex(pair_back[0]), " ",
          hex(pair_back[1]), crlf);
    bench.verdict("a two-register auto-incremented WRITE lands in both", same(pair, pair_back, 2));

    gyro_restore();
    uint8_t restored[5] = {};
    (void)gyro_transfer(cmd, 1, nullptr, restored, 5);
    bench.verdict("the control registers are back at their reset values, the device powered down",
                  same(gyro_ctrl_reset, restored, 5));
}

// =============================================================================
// j - the kernel
// =============================================================================

namespace kl {

using SpiArb = SpiBus<Host, P, 4>;

uint8_t rx_a[5];
uint8_t rx_b[5];
const uint8_t read_ctrl[1] = {static_cast<uint8_t>(gyro_read | gyro_increment | gyro_ctrl1)};
const uint8_t read_who[1] = {static_cast<uint8_t>(gyro_read | gyro_who_am_i)};

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

Host::Request request(const uint8_t* cmd, uint8_t* rx, uint16_t len, bool polled) {
    Host::Request r{};
    r.cs = GyroCs::ref();
    r.cmd = lend<Lease::reply>(cmd);
    r.cmd_len = 1;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
    r.rx = lend<Lease::reply>(rx);
    r.len = len;
    r.mode = gyro_mode;
    r.clock = gyro_rate;
    r.polled = polled;
    r.reply = reply_to<Probe, SpiDone>();
    return r;
}

}  // namespace kl

void tj_kernel() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    const uint8_t want = who_am_i_seen != 0u ? who_am_i_seen : gyro_reg(gyro_who_am_i);
    kl::BusKernel::init_all();
    bus_ao_live = true;

    for (uint8_t i = 0; i < 4u; ++i) {
        post<kl::SpiArb>(kl::request(kl::read_who, (i == 3u) ? kl::rx_a : nullptr, 1, i == 1u));
    }
    kl::pump_until(4, 300);
    print(serial, "  four queued transactions (one polled): replies ", kl::Probe::n, " [",
          kl::Probe::replies[0], " ", kl::Probe::replies[1], " ", kl::Probe::replies[2], " ",
          kl::Probe::replies[3], "], the last one read ", hex(kl::rx_a[0]), crlf);
    bench.verdict("four transactions through SpiBus, four replies - util/spi_bus.hpp and "
                  "util/bus_master.hpp unchanged on this architecture",
                  kl::Probe::n == 4u);
    bench.verdict("... every one spi_ok, ISR-pumped and polled interleaved on one bus",
                  kl::Probe::replies[0] == spi_ok && kl::Probe::replies[1] == spi_ok &&
                      kl::Probe::replies[2] == spi_ok && kl::Probe::replies[3] == spi_ok);
    bench.verdict("and what the arbiter handed back is the device's own identity byte",
                  kl::rx_a[0] == want);

    // The write-then-read shape the portable example will use: a command
    // phase and a data phase in ONE select window, five registers back.
    kl::Probe::clear_tally();
    post<kl::SpiArb>(kl::request(kl::read_ctrl, kl::rx_b, 5, false));
    kl::pump_until(1, 300);
    print(serial, "  a write-then-read Request (1 command byte, 5 data): ", hex(kl::rx_b[0]), " ",
          hex(kl::rx_b[1]), " ", hex(kl::rx_b[2]), " ", hex(kl::rx_b[3]), " ", hex(kl::rx_b[4]),
          crlf);
    bench.verdict("the two-phase Request reads the control block through the arbiter, and the "
                  "device is still powered down",
                  kl::Probe::n == 1u && same(gyro_ctrl_reset, kl::rx_b, 5));

    kl::Probe::clear_tally();
    for (uint8_t i = 0; i < 6u; ++i) {
        post<kl::SpiArb>(kl::request(kl::read_who, nullptr, 1, false));
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

    bus_ao_live = false;
    (void)host_ready();
}

// =============================================================================
// k - the DMA engines
// =============================================================================

void tk_engines() {
    (void)host_ready();
    Dma<2>::init();
    bus_ao_live = false;
    deselect_all();
    if (!DmaHost::init(clock)) {
        bench.verdict("the engined host came up", false);
        return;
    }
    DmaHost::sck_speed(PinSpeed::very_high);
    dma_host_live = true;

    const uint8_t want = who_am_i_seen;
    uint8_t got[5] = {};
    const uint8_t cmd[1] = {static_cast<uint8_t>(gyro_read | gyro_increment | gyro_ctrl1)};

    DmaHost::Request r{};
    r.cs = GyroCs::ref();
    r.cmd = lend<Lease::reply>(cmd);
    r.cmd_len = 1;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
    r.rx = lend<Lease::reply>(got);
    r.len = 5;
    r.mode = gyro_mode;
    r.clock = gyro_rate;
    r.polled = true;
    const bool sync = DmaHost::start(r);
    print(serial, "  a five-byte read through the engines (polled): ", hex(got[0]), " ",
          hex(got[1]), " ", hex(got[2]), " ", hex(got[3]), " ", hex(got[4]), ", status ",
          DmaHost::status(), crlf);
    bench.verdict("a polled request completes inside start() with the engines carrying the data "
                  "phase",
                  sync && DmaHost::status() == spi_ok);
    bench.verdict("... and the block is byte-exact against the control registers' reset values",
                  same(gyro_ctrl_reset, got, 5));

    // The identity, this time on the ISR-style completion: the command
    // phase on the pump, the data phase on the streams.
    uint8_t who[1] = {};
    const uint8_t wcmd[1] = {static_cast<uint8_t>(gyro_read | gyro_who_am_i)};
    DmaHost::Request a{};
    a.cs = GyroCs::ref();
    a.cmd = lend<Lease::reply>(wcmd);
    a.cmd_len = 1;
    a.tx = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
    a.rx = lend<Lease::reply>(who);
    a.len = 1;
    a.mode = gyro_mode;
    a.clock = gyro_rate;
    a.polled = false;
    const bool async = DmaHost::start(a);
    uint32_t spins = 2'000'000u;
    while (GyroCs::read_out() == false && spins-- != 0u) {
    }
    print(serial, "  the same read asynchronously: ", hex(who[0]), " (start() returned ",
          async ? "true" : "false", ", the select came back up ",
          GyroCs::read_out() ? "yes" : "NO", ")", crlf);
    bench.verdict("an ISR-style engined request answers off the streams' vectors, the select "
                  "released by the completion",
                  !async && GyroCs::read_out());
    bench.verdict("... and the byte is the device's identity",
                  want == 0u ? who[0] != 0x00u && who[0] != 0xFFu : who[0] == want);

    // How fast the engined data phase really runs: the streams keep the
    // shifter fed, so this IS the wire and not the pump.
    uint8_t block[64] = {};
    const uint8_t bcmd[1] = {static_cast<uint8_t>(gyro_read | gyro_increment | gyro_ctrl1)};
    for (uint8_t code = 3; code < 8u; ++code) {
        const SpiClock c = static_cast<SpiClock>(code);
        DmaHost::Request b{};
        b.cs = GyroCs::ref();
        b.cmd = lend<Lease::reply>(bcmd);
        b.cmd_len = 1;
        b.tx = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
        b.rx = lend<Lease::reply>(block);
        b.len = 64;
        b.mode = gyro_mode;
        b.clock = c;
        b.polled = true;
        const uint32_t t0 = cycle_stamp();
        (void)DmaHost::start(b);
        const uint32_t took = cycles_since(t0);
        const uint32_t stated = spi_sck_hz(DmaHost::reference_hz(), c);
        const uint32_t ns_per_frame = (took * 1000u) / (65u * cycles_per_us);
        print(serial, "  64 frames at SCK ", stated / 1000u, " kHz: ", took / cycles_per_us,
              " us -> ", ns_per_frame, " ns a frame (the wire alone is ",
              stated == 0u ? 0u : static_cast<uint32_t>(8u * 1'000'000'000ULL / stated), " ns)",
              crlf);
    }

    dma_host_live = false;
    DmaHost::release();
    (void)host_ready();
}

// =============================================================================
// l - the SCK pad against ES0206 2.12.4
// =============================================================================

void tl_pad_speed() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    const uint8_t want = who_am_i_seen != 0u ? who_am_i_seen : gyro_reg(gyro_who_am_i);
    print(serial, "  PCLK2 is ", Host::reference_hz() / 1000u,
          " kHz; the errata's ceiling per pad class (SPI): very high/high ",
          spi_errata_apb_ceiling_hz(PinSpeed::very_high) / 1000u, " kHz, medium ",
          spi_errata_apb_ceiling_hz(PinSpeed::medium) / 1000u, " kHz, low ",
          spi_errata_apb_ceiling_hz(PinSpeed::low) / 1000u, " kHz", crlf);
    bench.verdict("this bus runs ABOVE the errata's ceiling at every pad class, and the driver "
                  "says so rather than promising the last bit",
                  !Host::within_errata_ceiling());

    constexpr PinSpeed classes[4] = {PinSpeed::very_high, PinSpeed::high, PinSpeed::medium,
                                     PinSpeed::low};
    const char* names[4] = {"very high", "high", "medium", "low"};
    uint8_t exact = 0;
    for (uint8_t i = 0; i < 4u; ++i) {
        Host::sck_speed(classes[i]);
        uint8_t seen[4] = {};
        bool all = true;
        for (uint8_t k = 0; k < 4u; ++k) {
            seen[k] = gyro_reg(gyro_who_am_i);
            if (seen[k] != want) {
                all = false;
            }
        }
        if (all) {
            ++exact;
        }
        print(serial, "  SCK pad ", names[i], " (ceiling ",
              spi_errata_apb_ceiling_hz(classes[i]) / 1000u, " kHz): ", hex(seen[0]), " ",
              hex(seen[1]), " ", hex(seen[2]), " ", hex(seen[3]), all ? "  exact" : "  WRONG",
              crlf);
    }
    print(serial, "  ", exact, " of the four pad classes read the device exactly at SCK ",
          Host::sck_hz(gyro_rate) / 1000u, " kHz", crlf);
    bench.verdict("at the pad class the driver hands over - very high - the last received bit is "
                  "captured and the identity is exact",
                  exact >= 1u);
    Host::sck_speed(PinSpeed::very_high);
    (void)host_ready();
}

// =============================================================================
// m - the display controller over the bidirectional line
// =============================================================================

namespace bidi {

/// The gyroscope's CTRL_REG4, whose bit 0 moves the device's own
/// interface between four wires and three. A three-wire device answers on
/// the SAME pad it listens on, which on this board is the SPI's MOSI - so
/// it is the one peer the bidirectional half of this chapter can be
/// measured against.
constexpr uint8_t gyro_ctrl4 = 0x23;
constexpr uint8_t gyro_sim_3wire = 0x01;

/// Put the resource in bidirectional mode with the line DRIVEN, and hand
/// the MOSI pad over. Interrupts off: this letter drives the resource by
/// hand and no ISR must take a frame from it.
void begin(SpiMode mode, SpiClock rate) {
    Nvic::disable(S::irq);
    (void)S::configure(SpiConfig{.role = SpiRole::host,
                                 .mode = mode,
                                 .clock = rate,
                                 .nss = SpiNss::software,
                                 .direction = SpiDirection::half_duplex_out});
    Pin<'F', 9>::function(PinFunction::af5, {.speed = PinSpeed::very_high});
    S::enable();
}

/// One frame out on the single line, waited to the last edge.
void write_frame(uint8_t v) {
    S::data(v);
    uint32_t spins = 400'000u;
    while (!S::tx_empty() && spins-- != 0u) {
    }
    spins = 400'000u;
    while (S::busy() && spins-- != 0u) {
    }
}

/// The receive phase: BIDIOE clear makes a MASTER clock continuously the
/// moment SPE goes up, so the stop is 28.3.8's own - drop SPE after the
/// second-to-last frame has landed, then take the last one. ES0206 2.12.1
/// is why BSY is never consulted here.
///
/// THE PAD IS TURNED ROUND BY HAND, and that is the finding this letter
/// exists for. Measured, three ways: with the pad left an
/// alternate-function PUSH-PULL output the read comes back all ones,
/// whatever the device is driving - BIDIOE stops the shifter, not the
/// pad's driver; with the pad moved to a plain INPUT the read comes back
/// all ZEROES, because the peripheral's input is not taken from there;
/// with the pad kept in its alternate function but made OPEN DRAIN with a
/// pull-up, the device's answer arrives exactly. So the receive window
/// wants the output stage let go and the alternate function kept, and
/// that is one pad store on the way in and one on the way out.
void read_frames(uint8_t* into, uint8_t n) {
    S::flush_rx();
    Pin<'F', 9>::function(PinFunction::af5,
                          {.pull = PinPull::up, .open_drain = true, .speed = PinSpeed::very_high});
    // BIDIOE is not enable-protected, so the turnaround costs no gap:
    // the clock starts on this store, right after the command's last edge.
    S::bidirectional_output(false);
    for (uint8_t i = 0; i < n; ++i) {
        if (i + 1u == n) {
            S::regs().CR1 &= ~SPI_CR1_SPE;
        }
        uint32_t spins = 400'000u;
        while (!S::rx_ready() && spins-- != 0u) {
        }
        into[i] = static_cast<uint8_t>(S::data());
    }
    S::regs().CR1 &= ~SPI_CR1_SPE;
    Pin<'F', 9>::function(PinFunction::af5, {.speed = PinSpeed::very_high});
}

}  // namespace bidi

void tm_bidirectional() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    using namespace bidi;
    const uint8_t want = who_am_i_seen != 0u ? who_am_i_seen : gyro_reg(gyro_who_am_i);

    // ---- against the gyroscope, moved to its own three-wire interface ----
    // A WRITE looks the same on the wire either way, so this transition
    // and its undoing both go out over the bidirectional line - which is
    // what makes the recovery as reliable as the move.
    begin(gyro_mode, gyro_rate);
    print(serial, "  bidirectional out: CR1 ", hex(S::regs().CR1), " CR2 ", hex(S::regs().CR2),
          crlf);
    GyroCs::clear();
    write_frame(gyro_ctrl4);
    write_frame(gyro_sim_3wire);
    GyroCs::set();

    // The move is witnessed on the OTHER wire: a three-wire device stops
    // answering on the pad the four-wire one used, so a full-duplex read
    // now finds the line's own pull and nothing else.
    (void)host_ready();
    const uint8_t silent = gyro_reg(gyro_who_am_i);
    print(serial, "  after the one-line write of CTRL_REG4, a four-wire read finds ", hex(silent),
          " - the device has let go of the MISO pad", crlf);
    bench.verdict("a write over the bidirectional line reaches the device: the wire is a wire "
                  "whichever direction the peripheral thinks it is in, and the device answers "
                  "elsewhere afterwards",
                  silent != want);

    uint8_t three_wire[2] = {};
    begin(gyro_mode, gyro_rate);
    GyroCs::clear();
    write_frame(static_cast<uint8_t>(gyro_read | gyro_increment | gyro_who_am_i));
    read_frames(three_wire, 2);
    GyroCs::set();
    print(serial, "  the gyroscope in ITS three-wire mode, read over the MOSI pad alone: ",
          hex(three_wire[0]), " ", hex(three_wire[1]), crlf);
    bench.verdict("the bidirectional line reads a real device: the command goes out with BIDIOE "
                  "set, the answer comes back on the same pad with it clear and the pad turned "
                  "round",
                  three_wire[0] == want);

    // Back to four wires, over the same one-line write.
    begin(gyro_mode, gyro_rate);
    GyroCs::clear();
    write_frame(gyro_ctrl4);
    write_frame(0x00u);
    GyroCs::set();
    (void)host_ready();
    const uint8_t who = gyro_reg(gyro_who_am_i);
    print(serial, "  the device back on four wires reads ", hex(who), crlf);
    bench.verdict("... and the same one-line write puts it back, the full-duplex bus intact",
                  who == want);

    // ---- and against the display controller ----
    uint8_t id[4] = {};
    begin(SpiMode::mode0, SpiClock::div64);
    LcdDcx::clear();   // a command
    LcdCs::clear();
    write_frame(0xD3u);   // RDDID4: a dummy byte then 0x00 0x93 0x41
    LcdDcx::set();        // the parameters
    read_frames(id, 4);
    LcdCs::set();
    print(serial, "  the display's 0xD3 answered ", hex(id[0]), " ", hex(id[1]), " ", hex(id[2]),
          " ", hex(id[3]), crlf);
    print(serial, "  (all ones is the pull-up on a line nobody drove - the same transaction "
                  "shape that just read the gyroscope, so it is the DEVICE that does not answer: "
                  "this board straps the controller's interface mode so that its replies leave "
                  "on a pad the MCU is not wired to)",
          crlf);
    bench.verdict("the display's read path is not measurable on this board, and what came back "
                  "says so rather than being read as an identity",
                  id[1] != 0x00u || id[2] != 0x93u || id[3] != 0x41u);
    (void)host_ready();
    const uint8_t after = gyro_reg(gyro_who_am_i);
    print(serial, "  the gyroscope after the display's turn: ", hex(after), crlf);
    bench.verdict("the bus goes back to full duplex with the other device on it intact",
                  after == want);
}

// =============================================================================
// n - the I2S
// =============================================================================

namespace audio {

// I2S3's pads on this board: PC10 and PC12 are free, and the word select
// and the master clock land on two pads the display takes as INPUTS (its
// vertical sync and one of its colour bits), which the display ignores
// while its own controller is idle.
using CkPin = Pin<'C', 10>;    // I2S3_CK, AF6
using WsPin = Pin<'A', 4>;     // I2S3_WS, AF6
using SdPin = Pin<'C', 12>;    // I2S3_SD, AF6
using MckPin = Pin<'C', 7>;    // I2S3_MCK, AF6

using Audio = I2s<3>;

/// The audio PLL from the same 8 MHz crystal the main one uses. Its input
/// divider is the MAIN PLL's M on this part class, so the VCO input is
/// what the system clock already fixed: 2 MHz here. 192 over 5 puts
/// 76.8 MHz on the R output, and 76.8 MHz is a rate the 16-bit frame
/// divides EXACTLY into 48 kHz.
constexpr uint32_t i2s_clk_hz = 76'800'000u;
constexpr PllI2sConfig pll = plli2s_config_for(8'000'000u, i2s_clk_hz, SysClock::pll.m);
static_assert(pll.n == 192 && pll.r == 5);
static_assert(plli2s_r_hz(8'000'000u, pll, SysClock::pll.m) == i2s_clk_hz);

constexpr uint32_t target_fs = 48'000u;
constexpr auto prescaler = i2s_prescaler_for(i2s_clk_hz, target_fs, false, false);
static_assert(prescaler.has_value() && prescaler->div == 25 && !prescaler->odd);
static_assert(i2s_fs_hz(i2s_clk_hz, prescaler->div, prescaler->odd, false, false) == target_fs);

/// Count the word select's rising edges over `ms` milliseconds, feeding
/// the transmitter whenever its buffer empties, and note whether CHSIDE
/// ever changed. A half period at 48 kHz is ten microseconds - some two
/// thousand core cycles - so the poll cannot miss an edge.
uint32_t count_ws_edges(uint32_t ms, bool& both_sides) {
    uint32_t edges = 0;
    bool saw_left = false;
    bool saw_right = false;
    bool last = WsPin::read();
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < ms) {
        if (Audio::tx_empty()) {
            Audio::data(0x1234u);
            if (Audio::right_channel()) {
                saw_right = true;
            } else {
                saw_left = true;
            }
        }
        const bool now = WsPin::read();
        if (now && !last) {
            ++edges;
        }
        last = now;
    }
    both_sides = saw_left && saw_right;
    return edges;
}

/// How many of `samples` reads of the master clock pad found it HIGH,
/// with the transmitter kept fed so the generator never runs dry. At
/// 256 x FS that clock is faster than this loop, so what is asked is
/// presence - a count strictly between nought and all - and not a rate.
constexpr uint32_t mck_samples = 4000;

uint32_t mck_high_count() {
    uint32_t high = 0;
    for (uint32_t i = 0; i < mck_samples; ++i) {
        if (Audio::tx_empty()) {
            Audio::data(0x4321u);
        }
        if (MckPin::read()) {
            ++high;
        }
    }
    return high;
}

}  // namespace audio

void tn_i2s() {
    (void)host_ready();
    using namespace audio;

    // The audio PLL, off the same root the main one runs on.
    Rcc::plli2s_enable(false);
    (void)Rcc::plli2s_wait(false);
    const bool wrote = Rcc::plli2s_configure(pll);
    Rcc::plli2s_enable(true);
    const bool locked = Rcc::plli2s_wait(true);
    const bool routed = Rcc::i2s_source(I2sSource::plli2s_r);
    print(serial, "  PLLI2S: M ", Rcc::pll_m(), " (the main PLL's), N ", pll.n, ", R ", pll.r,
          " -> ", i2s_clk_hz / 1000u, " kHz; configured ", wrote ? "yes" : "NO", ", locked ",
          locked ? "yes" : "NO", crlf);
    bench.verdict("the audio PLL takes its ratio and locks - a second PLL on the same root, whose "
                  "input divider on this part class is the MAIN PLL's",
                  wrote && locked && routed);
    bench.verdict("PLLI2SCFGR is refused while the PLL runs, as 7.3.23 wants",
                  !Rcc::plli2s_configure(pll));

    Audio::bus_clock(true);
    Audio::reset();
    const I2sConfig cfg{.mode = I2sMode::host_transmit,
                        .standard = I2sStandard::philips,
                        .data = I2sDataLength::bits16,
                        .channel = I2sChannelLength::bits16,
                        .div = prescaler->div,
                        .odd = prescaler->odd};
    const bool configured = Audio::configure(cfg);
    bench.verdict("the block moves to its audio face: I2SMOD set with I2SE clear",
                  configured && Audio::i2s_mode_selected() && !Audio::enabled());

    CkPin::function(PinFunction::af6, {.speed = PinSpeed::very_high});
    WsPin::function(PinFunction::af6, {.speed = PinSpeed::very_high});
    SdPin::function(PinFunction::af6, {.speed = PinSpeed::very_high});
    Audio::enable();
    Audio::data(0x1234u);

    constexpr uint32_t window_ms = 200;
    bool both_sides = false;
    const uint32_t edges = count_ws_edges(window_ms, both_sides);
    const uint32_t measured = edges * (1000u / window_ms);
    constexpr uint32_t computed = i2s_fs_hz(i2s_clk_hz, prescaler->div, prescaler->odd, false, false);
    print(serial, "  I2SDIV ", prescaler->div, " ODD ", prescaler->odd ? 1 : 0,
          ": the arithmetic says FS = ", computed, " Hz; the word select's own pad gave ", edges,
          " rising edges in ", window_ms, " ms = ", measured, " Hz", crlf);
    bench.verdict("the word select runs at the sampling frequency the chapter's divider "
                  "arithmetic states, inside one per cent",
                  measured > computed - computed / 100u && measured < computed + computed / 100u);
    bench.verdict("a frame is a LEFT channel then a RIGHT one, and CHSIDE says which the "
                  "transmitter wants next - both sides come round",
                  both_sides);
    (void)Audio::disable();

    // The master clock output: 256 x FS on a pad of its own, and with
    // MCKOE the whole divisor becomes 256 x (2 x I2SDIV + ODD) whatever
    // the channel width - which moves the sampling frequency for the same
    // divider, so the second configuration is a rate of its own.
    constexpr auto mck_pre = i2s_prescaler_for(i2s_clk_hz, target_fs, false, true);
    static_assert(mck_pre.has_value());
    constexpr uint32_t mck_fs = i2s_fs_hz(i2s_clk_hz, mck_pre->div, mck_pre->odd, false, true);
    const I2sConfig with_mck{.mode = I2sMode::host_transmit,
                             .standard = I2sStandard::philips,
                             .data = I2sDataLength::bits16,
                             .channel = I2sChannelLength::bits16,
                             .master_clock_out = true,
                             .div = mck_pre->div,
                             .odd = mck_pre->odd};
    MckPin::function(PinFunction::af6, {.speed = PinSpeed::very_high});
    const bool mck_cfg = Audio::configure(with_mck);
    Audio::enable();
    Audio::data(0x4321u);
    bool mck_sides = false;
    const uint32_t mck_edges = count_ws_edges(window_ms, mck_sides);
    const uint32_t mck_measured = mck_edges * (1000u / window_ms);
    const uint32_t high = mck_high_count();
    print(serial, "  with MCKOE, I2SDIV ", mck_pre->div, " gives FS = ", mck_fs,
          " Hz by the arithmetic and ", mck_measured, " Hz on the word-select pad; MCK should be ",
          256u * mck_fs / 1000u, " kHz and its pad read HIGH on ", high, " of ", mck_samples,
          " samples", crlf);
    bench.verdict("MCKOE changes the divisor to 256 x (2 x I2SDIV + ODD): the same generator "
                  "gives a different sampling frequency, and it is the one the arithmetic states",
                  mck_cfg && Audio::master_clock_out() && mck_measured > mck_fs - mck_fs / 100u &&
                      mck_measured < mck_fs + mck_fs / 100u);
    bench.verdict("the master clock appears on its own pad - too fast for this poll to count, so "
                  "what is asked of it is that it is neither stuck high nor stuck low",
                  high != 0u && high != mck_samples);

    (void)Audio::disable();
    Audio::select_spi_mode();
    CkPin::release();
    WsPin::release();
    SdPin::release();
    MckPin::release();
    Audio::bus_clock(false);
    Rcc::plli2s_enable(false);
    bench.verdict("the block goes back to its SPI face, and the pads are parked",
                  !Audio::i2s_mode_selected());
    (void)host_ready();
}

// =============================================================================
// the menu
// =============================================================================

void banner() {
    print(serial, crlf, "test_stm32f4_spi - SPI5 with the board's own gyroscope and display",
          crlf);
    bench.menu();
}

}  // namespace

// ---- the vectors -------------------------------------------------------------------------

extern "C" void USART1_IRQHandler() { (void)Serial::isr(); }

extern "C" void SPI5_IRQHandler() {
    if (dma_host_live) {
        (void)DmaHost::isr();
        return;
    }
    if (bus_ao_live) {
        if (Host::isr()) {
            brio::post<kl::SpiArb>(brio::TransferDone{Host::status()});
        }
        return;
    }
    (void)Host::isr();
}

extern "C" void DMA2_Stream4_IRQHandler() {
    if (dma_host_live) {
        (void)DmaHost::dma_isr();
    }
}
extern "C" void DMA2_Stream3_IRQHandler() {
    if (dma_host_live) {
        (void)DmaHost::dma_isr();
    }
}

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

int main() {
    // What the silicon held before a line of this program ran: the gate
    // is read out of the RCC (reading it does not open it), and the
    // registers behind it need the clock, so it is opened for the reading.
    boot.gate = brio::Spi<5>::bus_clock();
    brio::Spi<5>::bus_clock(true);
    boot.cr1 = brio::Spi<5>::regs().CR1;
    boot.cr2 = brio::Spi<5>::regs().CR2;
    boot.sr = brio::Spi<5>::regs().SR;
    boot.crcpr = brio::Spi<5>::regs().CRCPR;
    boot.i2scfgr = brio::Spi<5>::regs().I2SCFGR;
    boot.i2spr = brio::Spi<5>::regs().I2SPR;

    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    const bool host_ok = host_ready();

    bench.letter('a', "the instances and the block", ta_block);
    bench.letter('b', "the eight baud rates, timed", tb_rates);
    bench.letter('c', "what the driver refuses", tc_refusals);
    bench.letter('d', "the mode fault, raised by SSI", td_mode_fault);
    bench.letter('e', "the overrun and its clearing sequence", te_overrun);
    bench.letter('f', "the gyroscope answers", tf_identity);
    bench.letter('g', "the gyroscope at each of the eight rates", tg_device_rates);
    bench.letter('h', "the four modes against the one it speaks", th_modes);
    bench.letter('i', "auto-increment, and a register written then read", ti_registers);
    bench.letter('j', "THE KERNEL: SpiBus over SpiHost", tj_kernel);
    bench.letter('k', "the same transaction through the DMA engines", tk_engines);
    bench.letter('l', "the SCK pad's slew class against the errata", tl_pad_speed);
    bench.letter('m', "the bidirectional line, against both devices", tm_bidirectional);
    bench.letter('n', "I2S3 as a master transmitter", tn_i2s);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL" : "FAILED", " tick=",
                    tick_ok ? "SysTick" : "FAILED", " spi5=", host_ok ? "host" : "FAILED",
                    brio::crlf);
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
        bench.prompt();
    }
}


#endif   // STM32F446xx
