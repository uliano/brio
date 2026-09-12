// test_ch32_spi - the reference bench suite for the CH32V00x's SPI
// chapter: ch32v00x/spi.hpp over RM ch. 16, the host role on ONE
// JUMPER, the pump and the polled path, the DMA engines, the hardware
// CRC, and util/spi_bus.hpp's arbiter with not one line changed.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// TWO INSTRUMENTS ON THE SAME PADS, never both. This part has ONE SPI.
//
// THE LOOPBACK (letters b..f): the host's own MOSI is jumpered to its
// own MISO and every frame clocked out comes straight back - which
// proves the pump, the frame sizes, the four modes, every rate, the
// engines and the CRC byte for byte, with no second board.
//
//   PC6 (MOSI)  <->  PC7 (MISO)     one jumper
//   PC5 (SCK)   free; PC1 (NSS) free - the engine's select is a GPIO
//
// THE PEER BOARD (letters n..r, x): a second chip running `spi_peer`
// (a SAM C21 on SERCOM1 function C, or a Nucleo-G0 - the peer's ident
// says which), commanded IN BAND over the bus under test through
// avrdx/src/apps/spi_link.hpp, included by relative path: one source
// of truth for the wire format on every architecture. Five wires, the
// peer at 3.3 V (PC7 is not a 5 V pad):
//
//   PC5 (SCK)   ->  the peer's SCK   (a SAM C21: PA17)
//   PC6 (MOSI)  ->  the peer's MOSI  (PA16)
//   PC7 (MISO)  <-  the peer's MISO  (PA19)
//   PC3 (CS)    ->  the peer's SS    (PA18, its own pull-up holds it high)
//   GND         <-> GND
//
// Every letter PROBES the jumper first (PC6 driven both ways as a
// GPIO, PC7 read against the opposite pull; never a cached answer, the
// desk changes hands between letters): with it on the desk the peer
// cannot be (MOSI tied to MISO), so the loop letters run and the peer
// letters skip; without it the peer letters ask the peer for a ping
// and FAIL loudly when it does not answer, while the loop letters
// decline. The chip select of every request is PC3, a plain output.
//
// What is exercised, letter by letter:
//   a  the block, WIRELESS: the reset values, the control words, the
//      refusals, the enable protection MEASURED field by field (which
//      of CTLR1's fields take a write with SPE set), the arithmetic
//   b  THE LOOPBACK ON THE PUMP: the four modes at 8 and 16 bits,
//      sixteen frames each through the ISR, byte-exact
//   c  THE POLLED PATH AT EVERY RATE, /2 to /256, 64 bytes each, the
//      burst timed on the STK and the frame period reported
//   d  THE DMA ENGINES on channels 2 and 3: 128-byte blocks with and
//      without a command phase, timed, and the 16-bit fallback to the
//      pump
//   e  THE HARDWARE CRC through the loop: TXCRCR against a bitwise
//      reference, RXCRCR equal to it, CRCERR down
//   f  THE KERNEL: SpiBus (= BusMaster) over SpiHost, replies in order,
//      the rejection, both votes
//   n  THE PEER: the spi_link command channel, ident, ten pings
//   o  the matrix against the peer: four modes, both bit orders, and a
//      bit-order mismatch as an exact two-way reversal
//   p  the BR ladder against the peer, and where it breaks
//   q  THE KERNEL against the peer: four transactions queued from one
//      dispatch in ONE select window, read back by the second chip
//   r  THE ROLES INVERT: this board as the CLIENT (software-selected:
//      the NSS pad PC1 is the I2C's SDA on this desk), the peer as the
//      host clocking a burst
//   x  (outside z) the slip statistics: each mode ten times
//
// build: boards = v006k8,v003f4
// build: groups = abe,cd,f,no,p,q,rx
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
#include "kernel/kernel.hpp"
#include "kernel/post.hpp"
#include "kernel/time.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/spi_bus.hpp"
#include "util/testbench.hpp"

#include "../../../avrdx/src/apps/spi_link.hpp"

using P = brio::Ch32v00xPlatform<>;
using SysClock = brio::Clock<brio::ClockSource::pll, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<1, P, 64, 128>;
constexpr Serial serial;
using Led = Pin<'C', 0>;

using S = Spi<1>;
using Host = SpiHost<1>;
using DmaHost = SpiHost<1, spi1_default_pins, DmaTxEngine<3>, DmaRxEngine<2>>;
using SckPin = Pin<'C', 5>;
using MosiPin = Pin<'C', 6>;
using MisoPin = Pin<'C', 7>;
using CsPin = Pin<'C', 3>;

TestBench<Serial> bench;

volatile bool host_done = false;
volatile bool dma_host_live = false;
volatile bool bus_ao_live = false;
volatile uint32_t spi_isr_entries = 0;

uint8_t tx_buf[256];
uint8_t rx_buf[256];
uint8_t cmd_buf[4];

bool loop_present = false;

/// A transaction that never answered: the suite's own word.
constexpr uint8_t no_answer = 200;

void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    (void)delay_us(clock, 500);
}

/// The STK as a stopwatch: cycles since the tick's last reload, folded
/// with the tick count.
uint32_t cycles_now() {
    const uint32_t period = stk()->CMP + 1u;
    for (;;) {
        const uint32_t t0 = Ticker::ticks();
        const uint32_t cnt = stk()->CNT;
        const uint32_t t1 = Ticker::ticks();
        if (t0 == t1) {
            return t0 * period + cnt;
        }
    }
}

/// Is the jumper there? MOSI driven both ways as a GPIO, MISO read
/// with its pull set AGAINST the driven level: a floating pad merely
/// echoing its neighbour (measured on the CH32V003F4P6, PC7 following
/// PC6 with no wire) is not taken for a jumper, and a wire beats a pull.
bool probe_loop() {
    MisoPin::input(PinPull::down);
    MosiPin::output(true);
    (void)delay_us(clock, 5);
    const bool high = MisoPin::read();
    MisoPin::input(PinPull::up);
    MosiPin::clear();
    (void)delay_us(clock, 5);
    const bool low = !MisoPin::read();
    MosiPin::release();
    MisoPin::release();
    return high && low;
}

/// The jumper is PROBED AGAIN at every letter, never trusted from the
/// boot: the desk changes hands between letters (measured - the wires
/// moved to the peer under a running image, and a cached "present"
/// sent the loop letters against the peer).
bool need_loop() {
    loop_present = probe_loop();
    if (loop_present) {
        return true;
    }
    print(serial, "  SKIPPED, no verdict claimed: no loopback jumper between PC6 (MOSI) and "
                  "PC7 (MISO) - probed now",
          crlf);
    return false;
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

void host_ready() {
    dma_host_live = false;
    bus_ao_live = false;
    (void)Host::init(clock);
    CsPin::output(true);
}

/// One transaction through the plain host, waited out. The status, or
/// no_answer.
uint8_t host_xfer(const uint8_t* cmd, uint8_t cmd_len, const uint8_t* tx, uint8_t* rx, uint16_t len,
                  SpiMode mode, SpiClock rate, SpiDataSize bits, bool polled) {
    Host::Request r{};
    r.cs = CsPin::ref();
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

uint8_t dma_xfer(const uint8_t* cmd, uint8_t cmd_len, const uint8_t* tx, uint8_t* rx, uint16_t len,
                 SpiClock rate, SpiDataSize bits, bool polled) {
    DmaHost::Request r{};
    r.cs = CsPin::ref();
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

/// The hold around each select window: the engine releases the select
/// about a microsecond after the last edge, and a client whose
/// transaction the select edge resets loses a character it has not
/// fetched yet - so the protocol owns the chip select for these windows
/// (the Request carries a null PinRef) and pays 30 us on each side.
void link_hold() { (void)delay_us(clock, 30); }

/// SPI1 as the command channel's host, PC3 as its GPIO chip select.
bool link_command_mode() {
    dma_host_live = false;
    bus_ao_live = false;
    const bool ok = Host::init(clock);
    CsPin::output(true);
    dec.reset();
    return ok;
}

/// One protocol window: prime, select, hold, the transaction, hold,
/// deselect. THE MODE IS PRIMED BEFORE THE SELECT FALLS - a CPOL flip
/// inside an open window is one extra edge and the selected client
/// counts it into the frame (SpiHost::prime()'s own comment).
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
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
    r.cmd_len = 0;
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
    print(serial, crlf, "      the peer board must be running `spi_peer`; its console '0' forces the dark client "
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
    print(serial, "  THE PEER DID NOT ANSWER. The peer board must be running `spi_peer`; its console '0' "
          "forces the dark client back. Check the five wires in this file's header.", crlf);
    return false;
}

/// The opening line of every letter whose instrument is the PEER BOARD.
/// The loop jumper and the peer share MOSI and MISO, so a desk carrying
/// the jumper cannot be carrying the peer: that is a topology and it
/// skips, exactly as the loop letters skip on the other desk. An absent
/// peer with no jumper is a different thing - firmware to flash, or a
/// wire - and fails loudly.
bool need_peer() {
    loop_present = probe_loop();
    if (loop_present) {
        print(serial, "  SKIPPED, no verdict claimed: this letter's instrument is the PEER BOARD running "
                      "`spi_peer`, and the loopback jumper PC6-PC7 is on the desk - the two instruments share "
                      "the pads. Letters b..f are the instrument this desk has.",
              crlf);
        return false;
    }
    if (ensure_link()) {
        return true;
    }
    bench.verdict("the peer answers a ping (the peer board running spi_peer)", false);
    return false;
}

// ---- one exchange, commanded and then clocked ------------------------------

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
    uint8_t spare = 0;   ///< spilink::spare_polled_pump forces the peer's polled loop
    uint16_t ms = 400;
};

constexpr uint8_t max_exchange = 32;
uint8_t xtx[max_exchange];
uint8_t xrx[max_exchange];

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
    // A bit-order mismatch is not a shrug: told about it, each end
    // checks the EXACT bit-reverse of what the other sent.
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
    // The bit order is a BUS-LEVEL verb here too (SpiHost::bit_order()),
    // so this end states it and link_command_mode() puts it back with
    // the re-init.
    if (e.host_lsb) {
        (void)Host::bit_order(true);
    }
    (void)link_xfer(xtx, xrx, n, e.rate, e.host_mode);
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

// ===========================================================================
// a - the block, wireless
// ===========================================================================

void ta_block() {
    dma_host_live = false;
    bus_ao_live = false;
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

    // THE ENGINE COMPLETES WITH NOTHING ON MISO: the pump, the polled
    // path and the DMA engines each run a transaction to its end - what
    // comes back is not judged here (the loop letters do that), only
    // that every path ENDS and releases the select.
    host_ready();
    fill_pattern(tx_buf, 16, 0x21);
    const uint8_t pumped = host_xfer(nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode0, SpiClock::div16,
                                     SpiDataSize::bits8, false);
    const uint32_t pump_entries = spi_isr_entries;
    const uint8_t polled = host_xfer(nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode3, SpiClock::div2,
                                     SpiDataSize::bits16, true);
    dma_host_live = true;
    (void)DmaHost::init(clock);
    CsPin::output(true);
    const uint8_t dma = dma_xfer(nullptr, 0, tx_buf, rx_buf, 64, SpiClock::div4, SpiDataSize::bits8, false);
    const uint8_t dma_polled = dma_xfer(cmd_buf, 1, tx_buf, rx_buf, 32, SpiClock::div2, SpiDataSize::bits8, true);
    DmaHost::release();
    dma_host_live = false;
    print(serial, "  wireless completions: pump=", pumped, " (", pump_entries, " isr entries) polled=",
          polled, " dma=", dma, " dma polled=", dma_polled, " cs released=", CsPin::read_out(), crlf);
    bench.verdict("the ISR pump, the polled path and the DMA engines each run a transaction to "
                  "completion with MISO floating (sixteen entries for sixteen frames)",
                  pumped == spi_ok && pump_entries == 16u && polled == spi_ok && dma == spi_ok &&
                      dma_polled == spi_ok && CsPin::read_out());
    host_ready();
    print(serial, "  loopback jumper PC6-PC7: ", loop_present ? "present" : "ABSENT", crlf);
}

// ===========================================================================
// b - the loopback on the pump
// ===========================================================================

void tb_pump() {
    if (!need_loop()) {
        return;
    }
    host_ready();
    const SpiMode modes[] = {SpiMode::mode0, SpiMode::mode1, SpiMode::mode2, SpiMode::mode3};
    uint8_t exact = 0;
    for (uint8_t m = 0; m < 4u; ++m) {
        for (uint8_t w = 0; w < 2u; ++w) {
            const SpiDataSize bits = w == 0u ? SpiDataSize::bits8 : SpiDataSize::bits16;
            const uint16_t frames = 16;
            const uint16_t bytes = static_cast<uint16_t>(frames * (w + 1u));
            fill_pattern(tx_buf, bytes, static_cast<uint8_t>(0x10u * m + w));
            for (uint16_t i = 0; i < bytes; ++i) {
                rx_buf[i] = 0xEE;
            }
            const uint8_t st = host_xfer(nullptr, 0, tx_buf, rx_buf, frames, modes[m], SpiClock::div16,
                                         bits, false);
            const bool ok = st == spi_ok && same(tx_buf, rx_buf, bytes);
            print(serial, "  mode ", m, w == 0u ? "  8-bit" : " 16-bit", ": status=", st, " isr entries ",
                  spi_isr_entries, ok ? "  byte-exact" : "  MISMATCH", crlf);
            if (ok) {
                ++exact;
            }
        }
    }
    bench.verdict("the four modes at 8 and 16 bits, sixteen frames each through the ISR pump, "
                  "byte-exact",
                  exact == 8u);
    bench.verdict("one interrupt per frame (sixteen entries for sixteen frames)",
                  spi_isr_entries == 16u);

    // A command phase then a data phase, on the pump: the command's echo
    // is discarded, the data phase read back.
    cmd_buf[0] = 0x9F;
    cmd_buf[1] = 0x00;
    fill_pattern(tx_buf, 8, 0x77);
    for (uint16_t i = 0; i < 8; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint8_t st = host_xfer(cmd_buf, 2, tx_buf, rx_buf, 8, SpiMode::mode0, SpiClock::div8,
                                 SpiDataSize::bits8, false);
    bench.verdict("a two-frame command phase then eight data frames: the data read back exact, "
                  "the command's echo discarded",
                  st == spi_ok && same(tx_buf, rx_buf, 8));
    // Read with no out buffer: 0xFF dummies go out and come back.
    for (uint16_t i = 0; i < 8; ++i) {
        rx_buf[i] = 0x00;
    }
    const uint8_t rd = host_xfer(nullptr, 0, nullptr, rx_buf, 8, SpiMode::mode0, SpiClock::div8,
                                 SpiDataSize::bits8, false);
    bool ff = true;
    for (uint16_t i = 0; i < 8; ++i) {
        ff = ff && rx_buf[i] == 0xFFu;
    }
    bench.verdict("a read with no out buffer clocks 0xFF dummies (and reads them back on the loop)",
                  rd == spi_ok && ff);
    bench.verdict("the chip select is released after every transaction", CsPin::read_out());
}

// ===========================================================================
// c - the polled path at every rate
// ===========================================================================

void tc_rates() {
    if (!need_loop()) {
        return;
    }
    host_ready();
    uint8_t exact = 0;
    for (uint8_t code = 0; code < 8u; ++code) {
        const SpiClock rate = static_cast<SpiClock>(code);
        fill_pattern(tx_buf, 64, static_cast<uint8_t>(0xA0u + code));
        for (uint16_t i = 0; i < 64; ++i) {
            rx_buf[i] = 0xEE;
        }
        console_drain();
        const uint32_t t0 = cycles_now();
        const uint8_t st = host_xfer(nullptr, 0, tx_buf, rx_buf, 64, SpiMode::mode0, rate,
                                     SpiDataSize::bits8, true);
        const uint32_t cycles = cycles_now() - t0;
        const bool ok = st == spi_ok && same(tx_buf, rx_buf, 64);
        // The wire's own time for 64 frames of 8 bits at this SCK.
        const uint32_t sck = spi_sck_hz(SysClock::hz, rate);
        const uint32_t wire_cycles = (64UL * 8UL * SysClock::hz) / sck;
        print(serial, "  /", 2u << code, " (", sck / 1000u, " kHz): 64 bytes polled in ", cycles,
              " cycles (", cycles / 64u, " per frame, the wire alone ", wire_cycles / 64u, ")",
              ok ? "  byte-exact" : "  MISMATCH", crlf);
        if (ok) {
            ++exact;
        }
    }
    bench.verdict("all eight BR codes carry 64 bytes byte-exact on the polled path", exact == 8u);
    bench.verdict("the chip select is released after the polled transactions too", CsPin::read_out());
}

// ===========================================================================
// d - the DMA engines
// ===========================================================================


void td_dma() {
    if (!need_loop()) {
        return;
    }
    dma_host_live = true;
    bus_ao_live = false;
    (void)DmaHost::init(clock);
    CsPin::output(true);

    fill_pattern(tx_buf, 128, 0x3C);
    for (uint16_t i = 0; i < 128; ++i) {
        rx_buf[i] = 0xEE;
    }
    console_drain();
    uint32_t t0 = cycles_now();
    const uint8_t st = dma_xfer(nullptr, 0, tx_buf, rx_buf, 128, SpiClock::div2, SpiDataSize::bits8, false);
    const uint32_t cycles = cycles_now() - t0;
    print(serial, "  128 bytes at /2 on the engines: status=", st, " in ", cycles, " cycles (",
          cycles / 128u, " per frame; the wire alone 16)", same(tx_buf, rx_buf, 128) ? "  byte-exact"
                                                                                       : "  MISMATCH",
          crlf);
    bench.verdict("a 128-byte block through both engines completes spi_ok, byte-exact",
                  st == spi_ok && same(tx_buf, rx_buf, 128));
    bench.verdict("with no transfer fault on either channel",
                  DmaTxEngine<3>::faults() == 0u && DmaRxEngine<2>::faults() == 0u);

    // The command phase runs on the pump and hands over to the engines.
    cmd_buf[0] = 0x0B;
    fill_pattern(tx_buf, 32, 0x5A);
    for (uint16_t i = 0; i < 32; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint8_t cs = dma_xfer(cmd_buf, 1, tx_buf, rx_buf, 32, SpiClock::div4, SpiDataSize::bits8, false);
    bench.verdict("a command frame on the pump then 32 data frames on the engines, exact",
                  cs == spi_ok && same(tx_buf, rx_buf, 32));

    // Polled with engines: the block spun inside start().
    fill_pattern(tx_buf, 64, 0x99);
    for (uint16_t i = 0; i < 64; ++i) {
        rx_buf[i] = 0xEE;
    }
    console_drain();
    t0 = cycles_now();
    const uint8_t ps = dma_xfer(nullptr, 0, tx_buf, rx_buf, 64, SpiClock::div2, SpiDataSize::bits8, true);
    const uint32_t pcycles = cycles_now() - t0;
    print(serial, "  64 bytes polled on the engines: status=", ps, " in ", pcycles, " cycles", crlf);
    bench.verdict("a POLLED request on the engines completes inside start(), exact",
                  ps == spi_ok && same(tx_buf, rx_buf, 64));

    // 16-bit frames fall back to the pump.
    fill_pattern(tx_buf, 32, 0x42);
    for (uint16_t i = 0; i < 32; ++i) {
        rx_buf[i] = 0xEE;
    }
    spi_isr_entries = 0;
    const uint8_t hs = dma_xfer(nullptr, 0, tx_buf, rx_buf, 16, SpiClock::div8, SpiDataSize::bits16, false);
    bench.verdict("16-bit frames fall back to the pump on an engined host (sixteen ISR entries), exact",
                  hs == spi_ok && same(tx_buf, rx_buf, 32) && spi_isr_entries == 16u);
    // A read with no out buffer: the fixed dummy cell.
    for (uint16_t i = 0; i < 16; ++i) {
        rx_buf[i] = 0;
    }
    const uint8_t rd = dma_xfer(nullptr, 0, nullptr, rx_buf, 16, SpiClock::div8, SpiDataSize::bits8, false);
    bool ff = true;
    for (uint16_t i = 0; i < 16; ++i) {
        ff = ff && rx_buf[i] == 0xFFu;
    }
    bench.verdict("a read with no out buffer clocks the fixed 0xFF cell through the transmit engine",
                  rd == spi_ok && ff);
    DmaHost::release();
    dma_host_live = false;
    host_ready();
}

// ===========================================================================
// e - the hardware CRC
// ===========================================================================

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

void te_crc() {
    if (!need_loop()) {
        return;
    }
    dma_host_live = false;
    bus_ao_live = false;
    Pfic::disable(S::irq());
    Host::release();
    S::bus_clock(true);
    S::reset();
    // The resource by hand: a host in mode 0 at /64 with CRC-8 on the
    // default polynomial, the pads to the peripheral.
    (void)S::configure({.role = SpiRole::host, .mode = SpiMode::mode0, .clock = SpiClock::div64,
                        .bits = SpiDataSize::bits8, .crc = true, .crc_polynomial = 0x0007u});
    Pin<'C', 5>::function();
    MosiPin::function();
    MisoPin::input();
    S::enable();
    S::flush_rx();

    constexpr uint16_t n = 6;
    uint8_t data[n];
    uint8_t got[n];
    for (uint16_t i = 0; i < n; ++i) {
        data[i] = static_cast<uint8_t>(0x31u + i * 7u);
    }
    // Each frame written, spun on RXNE, read back; CRCNEXT right after
    // the last data frame; then the CRC frame itself comes back.
    for (uint16_t i = 0; i < n; ++i) {
        S::data8(data[i]);
        if (i == n - 1u) {
            S::crc_next();
        }
        uint32_t spins = 200'000u;
        while (!S::rxne() && spins-- != 0u) {
        }
        got[i] = S::data8();
    }
    uint32_t spins = 200'000u;
    while (!S::rxne() && spins-- != 0u) {
    }
    const uint8_t crc_frame = S::data8();
    spins = 200'000u;
    while (S::busy() && spins-- != 0u) {
    }
    const uint16_t tx_crc = S::tx_crc();
    const uint16_t rx_crc = S::rx_crc();
    const bool err = S::crc_error();
    const uint16_t ref = crc_reference(0x0007u, 8, data, n);
    print(serial, "  CRC8 over six frames: TXCRCR ", hex(tx_crc), " (software ", hex(ref), "), RXCRCR ",
          hex(rx_crc), ", the CRC frame read back ", hex(crc_frame), ", CRCERR ", err ? "SET" : "clear",
          crlf);
    bench.verdict("the six data frames came back exact through the loop", same(data, got, n));
    bench.verdict("THE HARDWARE CRC IS THE ARITHMETIC: TXCRCR is what a bitwise loop over the "
                  "same polynomial computes",
                  tx_crc == ref);
    bench.verdict("the receiver's RXCRCR over the looped-back frames is the same number, the CRC "
                  "frame it read is that value, and CRCERR stands down",
                  rx_crc == ref && crc_frame == static_cast<uint8_t>(ref) && !err);
    (void)S::disable();
    host_ready();
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

using BusKernel = Kernel<P, Probe, SpiArb>;

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
/// window), and runs them at the link's rate.
bool hand_cs = false;
SpiClock rate = SpiClock::div16;

Host::Request request(const uint8_t* tx, uint8_t* rx, bool polled) {
    Host::Request r{};
    r.cs = hand_cs ? PinRef{} : CsPin::ref();
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
    if (!need_loop()) {
        return;
    }
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
}

// ===========================================================================
// n - the peer's command channel
// ===========================================================================

void tn_peer_link() {
    if (!need_peer()) {
        return;
    }
    bench.verdict("the peer answers a ping over the five wires", true);

    spilink::Ident d{};
    const bool got = peer_ident(d);
    if (got) {
        print(serial, "  peer: label '");
        for (uint8_t i = 0; i < 8 && d.label[i]; ++i) {
            print(serial, d.label[i]);
        }
        print(serial, "' xtal=", d.xtal, " sanity=", hex(d.sanity), " fw=", hex(d.version), crlf);
    }
    bench.verdict("ident comes back and it IS spi_peer (the sanity byte), from a SECOND BOARD of another "
                  "architecture speaking the same wire format",
                  got && d.sanity == spilink::ident_sanity);

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

// ===========================================================================
// o - the matrix against the peer
// ===========================================================================

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
        const bool ok = exchange_exact(e, v, r);
        if (ok) {
            ++good;
        } else {
            print(serial, "  mode ", m, ":", crlf);
            dump_exchange(e, v, r);
        }
    }
    print(serial, "  ", good, " of 4 transfer modes byte-exact BOTH ways, 8 frames each", crlf);
    bench.verdict("all four transfer modes carry a burst byte-exact in both directions between TWO "
                  "SEPARATE CHIPS",
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
    bench.verdict("LSb first, both ends agreeing, is byte-exact too - the bit order a BUS-level verb "
                  "(CTLR1.LSBFIRST) rather than a Request field",
                  lsb_ok);

    Exchange mism{};
    mism.cfg.dord = 1;      // the client LSb first
    mism.host_lsb = false;  // this end MSb first
    Verify v2{};
    spilink::Report r2{};
    const bool ran = do_exchange(mism);
    const bool rep = ran && peer_report(r2);
    v2 = verify_rx(mism);
    print(serial, "  DORD mismatch: host mism=", v2.mism, " client mism=", r2.mism, " count=", r2.count,
          " (both checking the exact bit-reverse)", crlf);
    bench.verdict("a bit-order mismatch is an EXACT TWO-WAY BIT REVERSAL - each end reads the other's "
                  "bytes with their bits in the opposite order",
                  ran && rep && v2.mism == 0u && r2.mism == 0u && r2.count == mism.count);
    bench.verdict("and the command channel still works right after all of that", command(Op::ping));
}

// ===========================================================================
// p - the rate ladder against the peer
// ===========================================================================

void tp_peer_rates() {
    if (!need_peer()) {
        return;
    }
    // The BR codes, not a frequency ladder: a Request's clock IS a
    // division of HCLK, so this walks the register's own vocabulary
    // from the command rate upwards and prints what each one really
    // is. The top rung, HCLK/2 = 24 MHz, is only ASKED when everything
    // below it was exact - the ladder's job is to FIND the boundary,
    // and a rung above a break measures nothing.
    uint32_t last_good = 0;
    uint32_t first_bad = 0;
    spilink::Report bad_r{};
    for (uint8_t code = 8; code-- > 0;) {
        const SpiClock c = static_cast<SpiClock>(code);
        if (c == SpiClock::div2 && first_bad != 0u) {
            break;
        }
        Exchange e{};
        e.rate = c;
        e.count = 8;
        e.seed_a = 0x21;
        e.seed_b = 0x84;
        Verify v{};
        spilink::Report r{};
        const bool ok = exchange_exact(e, v, r);
        const uint32_t real = Host::sck_hz(c);
        print(serial, "  SCK HCLK/", 2u << code, " = ", real / 1000u, " kHz: ", ok ? "exact both ways" : "NOT exact",
              "  host mism=", v.mism, " client mism=", r.mism, " client count=", r.count, " serve=",
              (r.aux2 & 0x04u) != 0u ? "dma" : "pump", crlf);
        if (ok && first_bad == 0u) {
            last_good = real;
        } else if (!ok && first_bad == 0u) {
            first_bad = real;
            bad_r = r;
        }
    }
    print(serial, "  the link to the peer held to ", last_good / 1000u, " kHz");
    if (first_bad != 0u) {
        print(serial, " and broke at ", first_bad / 1000u, " kHz");
    }
    print(serial, crlf);
    bench.verdict("the link is exact at the command rate and at least four times faster",
                  last_good >= 750'000UL);
    // THE SIGNATURE OF THE PEER'S BOUNDARY: at the first rung that is
    // not exact the client still RECEIVED every character (its count
    // full, its mismatches zero) while what this end read back broke -
    // so the failure lives in the ANSWER path and not in the wire.
    // Vacuous, and says so, on the day a peer holds the whole ladder.
    bench.verdict("wherever the climb breaks, the peer still hears every character exact there - the "
                  "boundary is its ANSWER RELOAD, not the wire",
                  first_bad == 0u || (bad_r.count == 8u && bad_r.mism == 0u));
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
        post<kl::SpiArb>(kl::request(qtx[i], qrx[i], false));
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
    bench.verdict("FOUR TRANSACTIONS QUEUED FROM ONE DISPATCH come back in order, every one spi_ok, and the "
                  "bytes on the wire are exactly what each request lent - read back by A SECOND BOARD, with "
                  "not one line of util/spi_bus.hpp, util/bus_master.hpp or kernel/ changed for it",
                  replies_ok && mism == 0u && rep && r.count == total && r.mism == 0u);
    (void)link_command_mode();
    bench.verdict("the command channel is back after the kernel's window", command(Op::ping));
}

// ===========================================================================
// r - THE ROLES INVERT: this board as the CLIENT, the peer as the host
// ===========================================================================

/// SPI1's own pads carry the wire, so the client role is the SAME
/// three data pins with MISO driven and SCK and MOSI taken as inputs.
/// The select is SOFTWARE (SSM, this end selecting itself): the NSS
/// pad of this family's default column is PC1, which is the I2C's SDA
/// on a desk carrying both buses, so the peer's SS wire lands on PC3 -
/// read as an input for the record, driving nothing while the peer
/// hosts.
using PeerClient = SpiClient<1>;

uint8_t crx[64];
uint16_t crx_n = 0;

bool run_as_client(const spilink::Params& a) {
    crx_n = 0;
    dma_host_live = false;
    bus_ao_live = false;
    Pfic::disable(S::irq());
    CsPin::input(PinPull::up);
    if (!PeerClient::init(clock, {.mode = SpiMode::mode0, .bits = SpiDataSize::bits8, .lsb_first = false,
                                  .nss = SpiNss::software, .drive_output = true})) {
        CsPin::output(true);
        return false;
    }
    Pfic::disable(S::irq());   // polled: the vector stays with the host's pump
    // THE SELECT EDGE IS THE START, even software-selected: the peer
    // reconfigures its pads from client to host during the lead-in,
    // and a client already counting takes that as a clock edge (measured:
    // every frame one bit late, 0x16 for 0x2C). So the shifter stays off
    // until the peer's SS falls on PC3, and its 20 us of setup before
    // the first byte is when the first answer goes in.
    const uint32_t t0 = Ticker::millis();
    while (CsPin::read() && Ticker::millis() - t0 < a.ms) {
    }
    if (CsPin::read()) {
        (void)PeerClient::disable();
        CsPin::output(true);
        return false;
    }
    PeerClient::select(true);
    // ONE FRAME AHEAD: one buffer on this silicon, so the first answer
    // goes in before the host's clock can arrive and every received
    // frame loads the next.
    spilink::Stream out(a.pattern, a.seed_b);
    PeerClient::enable(out.next());
    uint16_t queued = 1;
    // The buffer is free again the moment the shifter takes a frame:
    // the next answer goes in on TXE, never more than one ahead of
    // what has arrived.
    while (crx_n < a.count && crx_n < sizeof crx && Ticker::millis() - t0 < a.ms) {
        if (queued < a.count && queued <= crx_n + 1u && PeerClient::writable()) {
            PeerClient::write(out.next());
            ++queued;
        }
        const auto v = PeerClient::poll();
        if (!v) {
            continue;
        }
        crx[crx_n++] = static_cast<uint8_t>(*v);
    }
    const bool selected_seen = !CsPin::read();
    (void)selected_seen;
    (void)PeerClient::disable();
    CsPin::output(true);
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
    // The peer's own SCK division OF ITS OWN CLOCK: 1.5 MHz on a 48 MHz SAM.
    a.aux16 = 32;
    a.cfg.apply = 1;
    a.cfg.mode = 0;
    a.cfg.dord = 0;
    if (!peer_act(Op::host_burst, a)) {
        bench.verdict("the peer accepted the host_burst command (its spi_peer must carry op 0x14)", false);
        return;
    }
    bench.verdict("the peer accepted the host_burst command", true);

    const bool ran = run_as_client(a);
    bench.verdict("this board came up as an SPI CLIENT on the very pads it hosts with, software-selected "
                  "on the peer's select edge",
                  ran);
    spilink::Report r{};
    const bool rep = peer_report(r);

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
    print(serial, "  client read ", crx_n, " of ", a.count, " frames, mism=", mism);
    if (mism) {
        print(serial, " (first idx ", first_idx, " got ", hex(got), " exp ", hex(want), ")");
    }
    print(serial, "; the peer as HOST read ", r.count, " with mism=", r.mism, " flags=", hex(r.flags), crlf);
    print(serial, "  read:");
    for (uint16_t i = 0; i < crx_n && i < 16u; ++i) {
        print(serial, " ", hex(crx[i]));
    }
    print(serial, crlf);
    bench.verdict("the client received every frame the foreign host clocked, byte-exact",
                  ran && crx_n == a.count && mism == 0u);
    bench.verdict("...and that host read this end's answer stream byte-exact FROM THE FIRST FRAME, which "
                  "is what the preload before the first clock buys",
                  rep && r.count == a.count && r.mism == 0u);
    (void)link_command_mode();
    bench.verdict("the command channel is back after the role swap", command(Op::ping));
}

// ===========================================================================
// x - the slip statistics against the peer (a diagnostic, OUTSIDE z)
// ===========================================================================

constexpr uint8_t slip_rounds = 10;

void tx_peer_slips() {
    if (!need_peer()) {
        return;
    }
    print(serial, "  each mode ", slip_rounds, " times, 8 frames at HCLK/256: bursts not byte-exact, and the "
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
    print(serial, crlf, "test_ch32_spi - ", device::part_name, " SPI (RM ch. 16): the loopback jumper or a peer",
          crlf);
    print(serial, "  PC6 (MOSI) <-> PC7 (MISO) for letters b..f: jumper ", loop_present ? "PRESENT" : "ABSENT",
          "; PC5/PC6/PC7 + PC3 to a peer running spi_peer for letters n..r", crlf);
    bench.menu();
}

} // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }

extern "C" BRIO_CH32_INTERRUPT void spi1_handler() {
    spi_isr_entries = spi_isr_entries + 1u;
    if (dma_host_live) {
        if (DmaHost::isr()) {
            host_done = true;
        }
        return;
    }
    if (bus_ao_live) {
        if (Host::isr()) {
            brio::post<kl::SpiArb>(brio::TransferDone{Host::status()});
        }
        return;
    }
    if (Host::isr()) {
        host_done = true;
    }
}

extern "C" BRIO_CH32_INTERRUPT void dma1_channel2_handler() {
    if (dma_host_live && DmaHost::dma_isr()) {
        host_done = true;
    }
}
extern "C" BRIO_CH32_INTERRUPT void dma1_channel3_handler() {
    if (dma_host_live && DmaHost::dma_isr()) {
        host_done = true;
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();
    loop_present = probe_loop();

    bench.letter('a', "the block, wireless: reset values, the control words, the refusals, "
                      "the enable protection measured", ta_block);
    bench.letter('b', "THE LOOPBACK on the pump: four modes, 8 and 16 bits, a command phase",
                 tb_pump);
    bench.letter('c', "the polled path at every rate, timed", tc_rates);
    bench.letter('d', "THE DMA ENGINES on channels 2 and 3, and the 16-bit fallback", td_dma);
    bench.letter('e', "the hardware CRC against a bitwise reference", te_crc);
    bench.letter('f', "THE KERNEL: SpiBus over SpiHost, the rejection, the votes", tf_kernel);
    bench.letter('n', "THE PEER: the spi_link command channel, ident, ten pings", tn_peer_link);
    bench.letter('o', "the matrix against the peer: four modes, both bit orders, a mismatch", to_peer_matrix);
    bench.letter('p', "the BR ladder against the peer, and where it breaks", tp_peer_rates);
    bench.letter('q', "THE KERNEL against the peer: four transactions in one select window", tq_peer_kernel);
    bench.letter('r', "THE ROLES INVERT: this board as the client, the peer as the host", tr_peer_client);
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
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        brio::print(serial, "  stack: ", brio::stack_untouched(), " B never touched", brio::crlf);
        bench.prompt();
    }
}
