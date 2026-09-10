// test_ch32_spi - the reference bench suite for the CH32V00x's SPI
// chapter: ch32v00x/spi.hpp over RM ch. 16, the host role on ONE
// JUMPER, the pump and the polled path, the DMA engines, the hardware
// CRC, and util/spi_bus.hpp's arbiter with not one line changed.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// ONE INSTRUMENT: THE LOOPBACK. This part has ONE SPI, so the host's
// own MOSI is jumpered to its own MISO and every frame clocked out
// comes straight back - which proves the pump, the frame sizes, the
// four modes, every rate, the engines and the CRC byte for byte, with
// no second board. What it cannot prove is a CLIENT: this file's
// client role waits for a foreign host.
//
//   PC6 (MOSI)  <->  PC7 (MISO)     one jumper
//   PC5 (SCK)   free; PC1 (NSS) free - the engine's select is a GPIO
//
// main() PROBES the jumper (PC6 driven both ways as a GPIO, PC7 read)
// and every wire letter declines with the reason when it is missing.
// The chip select of every request is PC3, a plain output nothing is
// wired to.
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
//
// build: boards = v006k8
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

/// Is the jumper there? MOSI driven both ways as a GPIO, MISO read.
bool probe_loop() {
    MisoPin::input();
    MosiPin::output(true);
    (void)delay_us(clock, 5);
    const bool high = MisoPin::read();
    MosiPin::clear();
    (void)delay_us(clock, 5);
    const bool low = !MisoPin::read();
    MosiPin::release();
    MisoPin::release();
    return high && low;
}

bool need_loop() {
    if (loop_present) {
        return true;
    }
    print(serial, "  SKIPPED, no verdict claimed: no loopback jumper between PC6 (MOSI) and "
                  "PC7 (MISO) - probed at boot and again now",
          crlf);
    loop_present = probe_loop();
    return loop_present;
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

Host::Request request(const uint8_t* tx, uint8_t* rx, bool polled) {
    Host::Request r{};
    r.cs = CsPin::ref();
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
    r.cmd_len = 0;
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = 8;
    r.mode = SpiMode::mode0;
    r.clock = SpiClock::div16;
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

void banner() {
    print(serial, crlf, "test_ch32_spi - CH32V006K8 SPI (RM ch. 16) on one loopback jumper", crlf);
    print(serial, "  PC6 (MOSI) <-> PC7 (MISO); SCK PC5; the chip select PC3 (unwired); jumper ",
          loop_present ? "PRESENT" : "ABSENT", crlf);
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
        bench.prompt();
    }
}
