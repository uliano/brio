// test_rp2040_spi - the reference bench suite for the RP2040's SPI
// (rp2040/spi.hpp over datasheet 4.4, the ARM PL022): the host on its
// loop-back, the pump and the polled path, the DMA engines, and
// util/spi_bus.hpp's arbiter with not one line changed - and, across the
// desk's link to the WeAct RP2350B running spi_peer, the host against a
// FOREIGN client in every mode, width and serve, the rate ladder against
// it, the arbiter read back by a second chip, THIS CHIP AS THE CLIENT of a
// foreign PL022 host, and the client's answer pad in its three states.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// TWO INSTRUMENTS:
//
//  - THE LOOP-BACK (a..e): the PL022's LBM feeds the transmitter into the
//    receiver with nothing on the wire, which proves the pump, the frame
//    sizes, the four modes, every rate, the engines and the arbiter with
//    no second peripheral. SPI0 hosts on GP18/19/16 with GP17 as its
//    select; no wire is on those pads.
//  - THE DESK PARTNER (n, o, p, q, r, m and x): SPI1 on GP10 (SCK), GP11
//    (TX), GP8 (RX) with GP29 the select - 1.5 kOhm to this board's 3V3 -
//    wired to the WeAct RP2350B's GP30 / GP28 / GP31 / GP41
//    (docs/boards/weact-rp2040.md). That board runs `spi_peer`
//    (rp2350/src/apps/spi_peer.cpp, on either half: its client a PIO
//    program, its host a PL022), commanded IN BAND over the bus under test
//    through avrdx/src/apps/spi_link.hpp; the letters are the ones both
//    Raspberry Pi suites share, rp_spi_pair.hpp beside this file, whose
//    header says what they ask of a suite and derives the ceiling letter p
//    judges to. FAILING loudly when nobody answers.
//
// THE RULER IS THE SYSTEM TIMER (rp2040/timer.hpp).
//
// What is exercised, letter by letter:
//   a  the block, WIRELESS: table 279 as the driver states it, the rate
//      arithmetic (62.5 Mbit/s at the top, the fastest setting under a
//      ceiling), the reset state, the refusals; then the loop-back
//      proven on the pump, sixteen frames in batches
//   b  THE LOOP-BACK ON THE PUMP: the four modes at 8 and 16 bits,
//      sixteen frames each, byte-exact; a command phase then a data
//      phase; a read with no out buffer; the select released
//   c  THE POLLED PATH AT EVERY NAMED RATE, div2 to div256, 64 frames
//      each, timed on the ruler, the frame period reported - none faster
//      than its wire at the rate asked
//   d  THE DMA ENGINES on the loop - half-word engines, SSPDR's width:
//      128-byte blocks with and without a command phase, timed, in ONE
//      DMA interrupt a transaction (the transmit engine reports errors
//      alone); the polled variant; 16-bit frames riding the same engines
//      with no SPI interrupt; and the pump taking a 16-bit request whose
//      buffers are not half-word aligned; and THE RATE ON THE WIRE - a
//      frame inside an engined block timed (the difference of a 2048- and
//      a 256-frame block) against its SCK periods at the rate asked, in
//      modes 3 and 0 and at 16 bits
//   e  THE KERNEL: SpiBus (= BusMaster) over SpiHost, replies in order,
//      the rejection, both sleep votes
//   n  THE PEER: ident (kind 0x09, on its crystal), ten pings, and the
//      dark listener woken by nothing
//   o  THE MATRIX: four modes, the client MSb and LSb first, 8- and 16-bit
//      frames, both of the peer's serves, each one held window; the
//      receive-only and answer-only serves; the CRC refused
//   p  THE LADDER: even divisors 256 to 2 on the polled host and the
//      engines, judged to the PIO client's derived ceiling
//   q  THE KERNEL against the peer: four transactions in one held window,
//      read back by the second chip
//   r  THE ROLES INVERT: this chip's PL022 client under the peer's PL022
//      host, its select the block's own frame signal - four modes, both
//      widths, three rates
//   m  THE PAD FACTS: the client's answer pad driven, under SOD and
//      released, each under a foreign host's burst
//   x  (outside z) the slip statistics: each mode ten times, two rates
//
// build: boards = pico,weact2040
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>
#include <variant>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time_event.hpp"
#include "rp2040/clock.hpp"
#include "rp2040/dma.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/pio.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/spi.hpp"
#include "rp2040/ticker.hpp"
#include "rp2040/timer.hpp"
#include "rp2040/uart.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/spi_bus.hpp"
#include "util/testbench.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED: one file for every board
// that speaks it.
#include "../../../avrdx/src/apps/spi_link.hpp"

namespace {

using namespace brio;

using SysClock = Clock<ClockSource::pll, 125'000'000>;
constexpr SysClock clock;
using P = Rp2040Platform<>;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;

TestBench<Serial, 16> bench;

// SPI0 hosts on GP18/19/16 with GP17 as the select, on the loop-back:
// no wire is on those pads.
constexpr SpiPins host_pins{.sck = 18, .tx = 19, .rx = 16};
using Host = SpiHost<0, host_pins>;
using DmaHost = SpiHost<0, host_pins, DmaTxEngine<4, uint16_t>, DmaRxEngine<5, uint16_t>>;
using CsPin = Pin<17>;

// SPI1 on the pair's wires (the header): the host with its GPIO select, the
// same with half-word engines on channels 6 and 7 (DMA_IRQ_1), the client
// on its select pad.
constexpr SpiPins link_host_pins{.sck = 10, .tx = 11, .rx = 8};
constexpr SpiPins link_client_pins{.sck = 10, .tx = 11, .rx = 8, .cs = 29};
using LinkHost = SpiHost<1, link_host_pins>;
using LinkDmaHost = SpiHost<1, link_host_pins, DmaTxEngine<6, uint16_t, 1>, DmaRxEngine<7, uint16_t, 1>>;
using LinkClient = SpiClient<1, link_client_pins>;
using LinkCsPin = Pin<29>;
using LinkTxPin = Pin<11>;
constexpr uint8_t partner_kind = spilink::peer_rp2350;
constexpr const char* partner_text = "the WeAct RP2350B";
constexpr uint32_t partner_sys_hz = 150'000'000;
/// The window counter of letter r: PIO1's machine 0 on the select pad.
using CountBlk = Pio<1>;
using CountSm = PioSm<1, 0>;
using GaugeSm = PioSm<1, 1>;
constexpr uint8_t count_cs_index = 29;
void open_count_window() {}
/// nSSPOE reaches the client's answer pad: measured by letter m.
constexpr bool client_pad_follows_select = false;

// Which task owns each instance's vector at the moment.
enum class Spi0Owner : uint8_t { none, host, dma_host };
volatile Spi0Owner spi0_owner = Spi0Owner::none;
volatile uint32_t spi0_isr_entries = 0;
volatile uint32_t dma_isr_entries = 0;
volatile bool transfer_done = false;
volatile uint8_t transfer_status = 0;
bool bus_ao_live = false;

uint32_t us_now() { return Timer::now_low(); }

alignas(4) uint8_t tx_buf[256];
alignas(4) uint8_t rx_buf[256];
uint8_t cmd_buf[4];

void fill_pattern(uint8_t* p, uint16_t n, uint8_t seed) {
    for (uint16_t i = 0; i < n; ++i) {
        p[i] = static_cast<uint8_t>(seed + 3u * i);
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

// ---- the host's transactions ----------------------------------------------------
void host_ready() {
    spi0_owner = Spi0Owner::none;
    bus_ao_live = false;
    (void)Host::init(clock);
    CsPin::output(true);
    spi0_owner = Spi0Owner::host;
}

/// One transaction through a host, waited out. The status, or 0xFE.
template <typename H>
uint8_t xfer(const uint8_t* cmd, uint8_t cmd_len, const uint8_t* tx, uint8_t* rx, uint16_t len,
             SpiMode mode, SpiClock rate, SpiDataSize bits, bool polled, PinRef cs = CsPin::ref()) {
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
    transfer_done = false;
    spi0_isr_entries = 0;
    dma_isr_entries = 0;
    if (H::start(r)) {
        return H::status();
    }
    const uint32_t t0 = us_now();
    while (!transfer_done && us_now() - t0 < 200'000u) {
    }
    return transfer_done ? transfer_status : 0xFEu;
}

/// THE RATE ON THE WIRE, which a loop alone cannot tell: the time ONE FRAME
/// takes inside an engined block, from the difference of two polled blocks
/// of 2048 and 256 frames - neither buffer named, the transmit engine
/// pouring its fixed cell and the receive engine its sink - so the fixed
/// cost of a transaction cancels; in thousandths of `bits` SCK periods
/// plus `gap_halves` half periods at the rate ASKED (clk_peri over the
/// divisor). The best of three of each block is taken.
uint32_t frame_permille(SpiClock rate, SpiMode mode, SpiDataSize bits, uint32_t gap_halves) {
    auto best = [&](uint16_t n) {
        uint32_t least = 0xFFFFFFFFu;
        for (uint8_t k = 0; k < 3u; ++k) {
            const uint32_t t0 = us_now();
            (void)xfer<DmaHost>(nullptr, 0, nullptr, nullptr, n, mode, rate, bits, true);
            const uint32_t took = us_now() - t0;
            least = took < least ? took : least;
        }
        return least;
    };
    const uint32_t big = best(2048);
    const uint32_t small = best(256);
    const uint32_t width = bits == SpiDataSize::bits16 ? 16u : 8u;
    // The frames' time in nanoseconds at the rate asked, 1792 of them.
    const uint64_t want_ns = 1792ull * (2u * width + gap_halves) * rate.divisor() * 1'000'000'000ull /
                             (2ull * SysClock::pclk_hz);
    const uint64_t got_ns = static_cast<uint64_t>(big - small) * 1000u;
    return want_ns == 0u ? 0u : static_cast<uint32_t>(1000ull * got_ns / want_ns);
}

bool wire_ok(uint32_t permille) { return permille >= 990u && permille <= 1030u; }

// =============================================================================
// a - the block, wireless
// =============================================================================
void ta_block() {
    static_assert(spi_sck_pin(0, 18) && spi_tx_pin(0, 19) && spi_rx_pin(0, 16) && spi_cs_pin(1, 9));
    bench.verdict("table 279 as the driver states it: SPI0 on 2/6/18/22 (SCK), 3/7/19/23 (TX), "
                  "0/4/16/20 (RX); SPI1 on 10/14/26, 11/15/27, 8/12/24/28",
                  spi_sck_pin(0, 2) && spi_sck_pin(0, 22) && spi_tx_pin(0, 7) && spi_rx_pin(0, 20) &&
                      spi_sck_pin(1, 14) && spi_sck_pin(1, 26) && spi_tx_pin(1, 27) && spi_rx_pin(1, 28) &&
                      !spi_sck_pin(0, 10) && !spi_tx_pin(1, 19) && !spi_pins_valid(0, {.sck = 10, .tx = 19}));
    const auto top = spi_clock_for(SysClock::pclk_hz, 62'500'000);
    const auto ten = spi_clock_for(SysClock::pclk_hz, 10'000'000);
    const auto one = spi_clock_for(SysClock::pclk_hz, 1'000'000);
    print(serial, "  the fastest under 62.5 MHz: ", top ? top->cpsdvsr : 0u, " x ", top ? top->scr + 1u : 0u, " = ",
          top ? spi_sck_hz(SysClock::pclk_hz, *top) : 0u, " Hz; under 10 MHz: ", ten ? spi_sck_hz(SysClock::pclk_hz, *ten) : 0u,
          "; under 1 MHz: ", one ? spi_sck_hz(SysClock::pclk_hz, *one) : 0u, crlf);
    bench.verdict("the rate chooser: clk_peri / 2 at the top, 8.93 MHz under 10, 992 kHz under 1, "
                  "and nothing under 1 kHz",
                  top && top->divisor() == 2u && ten && spi_sck_hz(SysClock::pclk_hz, *ten) == 8'928'571u &&
                      one && spi_sck_hz(SysClock::pclk_hz, *one) == 992'063u &&
                      !spi_clock_for(SysClock::pclk_hz, 1000).has_value());
    using S = Pl022<0>;
    const bool reset_ok = S::reset();
    print(serial, "  SPI0 after reset: SSPSR=", hex(S::flags()), " enabled=", S::enabled(), " bits=", S::bits(),
          crlf);
    bench.verdict("the block's reset state: both FIFOs empty, the transmit one not full, disabled",
                  reset_ok && (S::flags() & (SpiFlag::tx_empty | SpiFlag::tx_not_full)) ==
                                  (SpiFlag::tx_empty | SpiFlag::tx_not_full) && !S::rx_not_empty() && !S::enabled());
    bench.verdict("a 3-bit and a 17-bit frame, an odd prescaler, a host with its output disabled are refused",
                  !S::configure({.bits = 3}) && !S::configure({.bits = 17}) &&
                      !S::configure({.clock = SpiClock{3, 0}}) && !S::configure({.output_disabled = true}));
    bench.verdict("a 12-bit TI configuration is taken and reads back",
                  S::configure({.format = SpiFormat::ti, .bits = 12}) && S::bits() == 12u);
    S::enable(true);
    bench.verdict("and nothing is configured while enabled", !S::configure({}));
    S::enable(false);

    // THE INSTRUMENT: the loop-back on the pump.
    host_ready();
    Host::loopback(true);
    fill_pattern(tx_buf, 16, 0x21);
    const uint8_t st = xfer<Host>(nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode0, SpiClocks::div16,
                                  SpiDataSize::bits8, false);
    print(serial, "  the loop-back, sixteen frames on the pump: status ", st, ", ", spi0_isr_entries,
          " interrupt entries, ", same(tx_buf, rx_buf, 16) ? "byte-exact" : "MISMATCH", ", cs released=",
          CsPin::read_out(), crlf);
    bench.verdict("LBM feeds the transmitter into the receiver: sixteen frames byte-exact with nothing "
                  "on the wire, in fewer interrupts than frames (the FIFOs batch them)",
                  st == spi_ok && same(tx_buf, rx_buf, 16) && spi0_isr_entries < 16u && spi0_isr_entries >= 2u);
}

// =============================================================================
// b - the loop-back on the pump
// =============================================================================
void tb_pump() {
    host_ready();
    Host::loopback(true);
    const SpiMode modes[] = {SpiMode::mode0, SpiMode::mode1, SpiMode::mode2, SpiMode::mode3};
    uint8_t exact = 0;
    for (uint8_t m = 0; m < 4u; ++m) {
        for (uint8_t w = 0; w < 2u; ++w) {
            const SpiDataSize bits = w == 0u ? SpiDataSize::bits8 : SpiDataSize::bits16;
            const uint16_t bytes = static_cast<uint16_t>(16u * (w + 1u));
            fill_pattern(tx_buf, bytes, static_cast<uint8_t>(0x10u * m + w));
            for (uint16_t i = 0; i < bytes; ++i) {
                rx_buf[i] = 0xEE;
            }
            const uint8_t st = xfer<Host>(nullptr, 0, tx_buf, rx_buf, 16, modes[m], SpiClocks::div16, bits, false);
            const bool ok = st == spi_ok && same(tx_buf, rx_buf, bytes);
            print(serial, "  mode ", m, w == 0u ? "  8-bit" : " 16-bit", ": status=", st, " entries ",
                  spi0_isr_entries, ok ? "  byte-exact" : "  MISMATCH", crlf);
            if (ok) {
                ++exact;
            }
        }
    }
    bench.verdict("the four modes at 8 and 16 bits, sixteen frames each through the pump, byte-exact",
                  exact == 8u);
    cmd_buf[0] = 0x9F;
    cmd_buf[1] = 0x00;
    fill_pattern(tx_buf, 8, 0x77);
    for (uint16_t i = 0; i < 8; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint8_t st = xfer<Host>(cmd_buf, 2, tx_buf, rx_buf, 8, SpiMode::mode0, SpiClocks::div8,
                                  SpiDataSize::bits8, false);
    bench.verdict("a two-frame command phase then eight data frames: the data read back exact, the "
                  "command's echo discarded",
                  st == spi_ok && same(tx_buf, rx_buf, 8));
    for (uint16_t i = 0; i < 8; ++i) {
        rx_buf[i] = 0x00;
    }
    const uint8_t rd = xfer<Host>(nullptr, 0, nullptr, rx_buf, 8, SpiMode::mode0, SpiClocks::div8,
                                  SpiDataSize::bits8, false);
    bool ff = true;
    for (uint16_t i = 0; i < 8; ++i) {
        ff = ff && rx_buf[i] == 0xFFu;
    }
    bench.verdict("a read with no out buffer clocks 0xFF dummies (and reads them back on the loop)",
                  rd == spi_ok && ff);
    bench.verdict("the chip select is released after every transaction", CsPin::read_out());
    bench.verdict("an empty request completes on the spot",
                  xfer<Host>(nullptr, 0, nullptr, nullptr, 0, SpiMode::mode0, SpiClocks::div8, SpiDataSize::bits8, false) == spi_ok);
}

// =============================================================================
// c - the polled path at every named rate
// =============================================================================
void tc_rates() {
    host_ready();
    Host::loopback(true);
    const SpiClock rates[] = {SpiClocks::div2, SpiClocks::div4, SpiClocks::div8, SpiClocks::div16,
                              SpiClocks::div32, SpiClocks::div64, SpiClocks::div128, SpiClocks::div256};
    uint8_t exact = 0;
    uint8_t never_early = 0;
    for (uint8_t k = 0; k < 8u; ++k) {
        fill_pattern(tx_buf, 64, static_cast<uint8_t>(0xA0u + k));
        for (uint16_t i = 0; i < 64; ++i) {
            rx_buf[i] = 0xEE;
        }
        const uint32_t t0 = us_now();
        const uint8_t st = xfer<Host>(nullptr, 0, tx_buf, rx_buf, 64, SpiMode::mode0, rates[k], SpiDataSize::bits8, true);
        const uint32_t took = us_now() - t0;
        const bool ok = st == spi_ok && same(tx_buf, rx_buf, 64);
        const uint32_t wire_ns = 8u * 1000u * rates[k].divisor() / 125u;   // eight bits at clk_peri / divisor
        print(serial, "  div", rates[k].divisor(), " (", Host::sck_hz(rates[k]) / 1000u, " kHz): 64 frames in ", took,
              " us, ", took * 1000u / 64u, " ns a frame (the wire alone ", wire_ns, ")", ok ? "  exact" : "  MISMATCH",
              crlf);
        if (ok) {
            ++exact;
        }
        if (took * 1000u >= 64u * wire_ns) {
            ++never_early;
        }
    }
    bench.verdict("every named rate from clk_peri / 2 to / 256 carries 64 frames byte-exact on the polled path",
                  exact == 8u);
    bench.verdict("and none is faster than its wire: 64 frames take at least eight SCK periods each at "
                  "the rate asked (the polled path's own cost the rest; letter d times the wire)",
                  never_early == 8u);
}

// =============================================================================
// d - the DMA engines on the loop
// =============================================================================
void td_dma() {
    spi0_owner = Spi0Owner::none;
    (void)DmaHost::init(clock);
    CsPin::output(true);
    DmaHost::loopback(true);
    spi0_owner = Spi0Owner::dma_host;
    fill_pattern(tx_buf, 128, 0x31);
    for (uint16_t i = 0; i < 128; ++i) {
        rx_buf[i] = 0xEE;
    }
    uint32_t t0 = us_now();
    const uint8_t st = xfer<DmaHost>(nullptr, 0, tx_buf, rx_buf, 128, SpiMode::mode0, SpiClocks::div4, SpiDataSize::bits8, false);
    const uint32_t took = us_now() - t0;
    const uint32_t dma_entries = dma_isr_entries;
    print(serial, "  128 bytes on the engines at div4 (31.25 MHz): status ", st, " in ", took, " us (the wire alone ",
          128u * 8u * 4u / 125u, "), ", same(tx_buf, rx_buf, 128) ? "byte-exact" : "MISMATCH", ", cs released=",
          CsPin::read_out(), ", ", dma_entries, " DMA interrupt(s)", crlf);
    bench.verdict("a 128-byte block through the two engines, ISR-completed, byte-exact and the select released",
                  st == spi_ok && same(tx_buf, rx_buf, 128) && CsPin::read_out());
    bench.verdict("ONE DMA interrupt for the transaction: the receive block's, the transmit engine "
                  "reporting errors alone",
                  dma_entries == 1u);
    cmd_buf[0] = 0x0B;
    fill_pattern(tx_buf, 32, 0x41);
    for (uint16_t i = 0; i < 32; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint8_t st2 = xfer<DmaHost>(cmd_buf, 1, tx_buf, rx_buf, 32, SpiMode::mode0, SpiClocks::div8, SpiDataSize::bits8, false);
    bench.verdict("a command frame on the pump, then 32 data frames on the engines: exact",
                  st2 == spi_ok && same(tx_buf, rx_buf, 32));
    fill_pattern(tx_buf, 64, 0x51);
    for (uint16_t i = 0; i < 64; ++i) {
        rx_buf[i] = 0xEE;
    }
    t0 = us_now();
    const uint8_t st3 = xfer<DmaHost>(cmd_buf, 1, tx_buf, rx_buf, 64, SpiMode::mode0, SpiClocks::div2, SpiDataSize::bits8, true);
    const uint32_t took3 = us_now() - t0;
    print(serial, "  polled on the engines at div2: status ", st3, " in ", took3, " us", crlf);
    bench.verdict("a polled request on the engines completes inside start(), exact",
                  st3 == spi_ok && same(tx_buf, rx_buf, 64));
    fill_pattern(tx_buf, 32, 0x61);
    for (uint16_t i = 0; i < 32; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint8_t st4 = xfer<DmaHost>(nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode3, SpiClocks::div16, SpiDataSize::bits16, false);
    bench.verdict("16-bit frames ride the half-word engines: exact, no SPI interrupt, one DMA interrupt",
                  st4 == spi_ok && same(tx_buf, rx_buf, 32) && spi0_isr_entries == 0u && dma_isr_entries == 1u);
    fill_pattern(tx_buf, 40, 0x71);
    for (uint16_t i = 0; i < 40; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint8_t st6 = xfer<DmaHost>(nullptr, 0, tx_buf + 1, rx_buf + 1, 16, SpiMode::mode3, SpiClocks::div16, SpiDataSize::bits16, false);
    bench.verdict("a 16-bit request whose buffers are not half-word aligned goes to the pump instead, exact",
                  st6 == spi_ok && same(tx_buf + 1, rx_buf + 1, 32) && spi0_isr_entries != 0u && dma_isr_entries == 0u);
    for (uint16_t i = 0; i < 16; ++i) {
        rx_buf[i] = 0x00;
    }
    const uint8_t st5 = xfer<DmaHost>(nullptr, 0, nullptr, rx_buf, 16, SpiMode::mode0, SpiClocks::div16, SpiDataSize::bits8, false);
    bool ff = true;
    for (uint16_t i = 0; i < 16; ++i) {
        ff = ff && rx_buf[i] == 0xFFu;
    }
    bench.verdict("a read with no out buffer goes through the transmit engine's fixed 0xFF cell",
                  st5 == spi_ok && ff);
    // THE RATE ON THE WIRE inside a block: mode 3 runs frame to frame with
    // no gap (SPH = 1), mode 0 with the 1.5 SCK periods the block's frame
    // gap takes (docs/rp2040/spi.md, the engines' bench findings).
    const uint32_t m3_div4 = frame_permille(SpiClocks::div4, SpiMode::mode3, SpiDataSize::bits8, 0);
    const uint32_t m3_div16 = frame_permille(SpiClocks::div16, SpiMode::mode3, SpiDataSize::bits8, 0);
    const uint32_t m3_div64 = frame_permille(SpiClocks::div64, SpiMode::mode3, SpiDataSize::bits8, 0);
    const uint32_t m0_div16 = frame_permille(SpiClocks::div16, SpiMode::mode0, SpiDataSize::bits8, 3);
    const uint32_t w16_div16 = frame_permille(SpiClocks::div16, SpiMode::mode3, SpiDataSize::bits16, 0);
    print(serial, "  a frame inside an engined block, in thousandths of its wire time at the rate asked: "
          "mode 3 div4 ", m3_div4, ", div16 ", m3_div16, ", div64 ", m3_div64, "; mode 0 div16 (with "
          "1.5 SCK of gap) ", m0_div16, "; 16-bit mode 3 div16 ", w16_div16, crlf);
    bench.verdict("THE ENGINES RUN AT THE RATE ASKED: a frame inside a block takes its SCK periods at "
                  "clk_peri over the divisor - div4, div16 and div64 in mode 3, div16 in mode 0 with "
                  "its gap and in 16-bit frames - each -1 % to +3 %",
                  wire_ok(m3_div4) && wire_ok(m3_div16) && wire_ok(m3_div64) && wire_ok(m0_div16) &&
                      wire_ok(w16_div16));
    spi0_owner = Spi0Owner::none;
    DmaHost::release();
}

// =============================================================================
// e - the kernel over the bus
// =============================================================================
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
        match(e,
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

Host::Request request(const uint8_t* tx, uint8_t* rx, bool polled) {
    Host::Request r{};
    r.cs = CsPin::ref();
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(nullptr));
    r.cmd_len = 0;
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = 8;
    r.mode = SpiMode::mode0;
    r.clock = SpiClocks::div16;
    r.polled = polled;
    r.reply = reply_to<Probe, SpiDone>();
    return r;
}

}  // namespace kl

void te_kernel() {
    host_ready();
    Host::loopback(true);
    kl::BusKernel::init_all();
    bus_ao_live = true;
    fill_pattern(kl::out_a, 8, 0x01);
    fill_pattern(kl::out_b, 8, 0x02);
    for (uint8_t i = 0; i < 4u; ++i) {
        post<kl::SpiArb>(kl::request(kl::out_a, (i == 3u) ? kl::in_a : nullptr, i == 1u));
    }
    kl::pump_until(4, 300);
    print(serial, "  four queued transactions (one polled): replies ", kl::Probe::n, " [", kl::Probe::replies[0], " ",
          kl::Probe::replies[1], " ", kl::Probe::replies[2], " ", kl::Probe::replies[3], "]", crlf);
    bench.verdict("four transactions through SpiBus, four replies - util/spi_bus.hpp and util/bus_master.hpp "
                  "unchanged on this architecture",
                  kl::Probe::n == 4u);
    bench.verdict("... every one spi_ok, ISR-pumped and polled interleaved on one bus",
                  kl::Probe::replies[0] == spi_ok && kl::Probe::replies[1] == spi_ok &&
                      kl::Probe::replies[2] == spi_ok && kl::Probe::replies[3] == spi_ok);
    bench.verdict("and the last one's read-back is the pattern, through the arbiter", same(kl::out_a, kl::in_a, 8));
    kl::Probe::clear_tally();
    for (uint8_t i = 0; i < 6u; ++i) {
        post<kl::SpiArb>(kl::request(kl::out_b, nullptr, false));
    }
    kl::pump_until(6, 300);
    print(serial, "  six posted into a four-deep queue: replies ", kl::Probe::n, ", rejected ", kl::Probe::rejected, crlf);
    bench.verdict("the arbiter rejects what it cannot queue, immediately", kl::Probe::rejected != 0u);
    bench.verdict("... and every request is still answered exactly once", kl::Probe::n == 6u);
    kl::Probe::clear_tally();
    post<kl::SpiArb>(PrepareSleep{.depth = SleepDepth::standby, .reply = reply_to<kl::Probe, SleepVote>()});
    kl::pump();
    bench.verdict("an IDLE bus votes for the sleep", kl::Probe::votes == 1u && kl::Probe::last_vote);
    kl::Probe::clear_tally();
    post<kl::SpiArb>(kl::request(kl::out_a, nullptr, false));
    post<kl::SpiArb>(PrepareSleep{.depth = SleepDepth::standby, .reply = reply_to<kl::Probe, SleepVote>()});
    kl::pump();
    kl::pump_until(2, 100);
    bench.verdict("a BUSY bus votes against it", kl::Probe::votes == 1u && !kl::Probe::last_vote);
    bus_ao_live = false;
}

// THE TWO-BOARD LETTERS, over the names above - in a namespace of their
// own, where CsPin and TxPin are the link's.
namespace link {
using CsPin = LinkCsPin;
using TxPin = LinkTxPin;
#include "rp_spi_pair.hpp"
}  // namespace link

void banner() {
    print(serial, crlf, "test_rp2040_spi - the RP2040 SPI (datasheet 4.4, PL022): SPI0 hosts on GP18/19/16 + GP17, "
          "on the loop-back; clk_peri=", SysClock::pclk_hz, " Hz", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_uart0() { (void)Serial::isr(); }
extern "C" void isr_systick() { brio::Ticker::tick(); }
extern "C" void isr_spi0() {
    spi0_isr_entries = spi0_isr_entries + 1u;
    switch (spi0_owner) {
        case Spi0Owner::host:
            if (Host::isr()) {
                transfer_status = Host::status();
                transfer_done = true;
                if (bus_ao_live) {
                    brio::post<kl::SpiArb>(brio::TransferDone{Host::status()});
                }
            }
            break;
        case Spi0Owner::dma_host:
            if (DmaHost::isr()) {
                transfer_status = DmaHost::status();
                transfer_done = true;
            }
            break;
        default:
            brio::Pl022<0>::interrupts(brio::SpiInterrupt::all, false);
            break;
    }
}
extern "C" void isr_spi1() { link::pair_spi_isr(); }
extern "C" void isr_dma_1() { link::pair_dma_isr(); }
extern "C" void isr_dma_0() {
    dma_isr_entries = dma_isr_entries + 1u;
    if (spi0_owner == Spi0Owner::dma_host && DmaHost::dma_isr()) {
        transfer_status = DmaHost::status();
        transfer_done = true;
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool timer_ok = brio::Timer::init(clock);
    const bool dma_ok = brio::Dma::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    bench.letter('a', "the block, wireless; the loop-back proven", ta_block);
    bench.letter('b', "the loop-back on the pump: modes, widths, phases", tb_pump);
    bench.letter('c', "the polled path at every named rate", tc_rates);
    bench.letter('d', "the DMA engines on the loop", td_dma);
    bench.letter('e', "the kernel: SpiBus over SpiHost", te_kernel);
    bench.letter('n', "the peer: ident, the command channel, the dark listener", link::tn_peer);
    bench.letter('o', "the matrix against the PIO client", link::to_matrix);
    bench.letter('p', "the ladder, judged to the client's ceiling", link::tp_ladder);
    bench.letter('q', "the kernel read back by the peer", link::tq_kernel);
    bench.letter('r', "the roles invert: this PL022 as the client", link::tr_roles);
    bench.letter('m', "the client's answer pad: driven, SOD, released", link::tm_pads);
    bench.letter('x', "slip statistics (outside z)", link::tx_slips, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL125" : "FAILED", " timer=", timer_ok ? "1us" : "FAILED",
                    " dma=", dma_ok ? "released" : "FAILED", " tick=", tick_ok ? "SysTick" : "FAILED", brio::crlf);
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
