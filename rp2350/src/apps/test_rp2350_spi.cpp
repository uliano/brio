// test_rp2350_spi - the reference bench suite for this chip's SPI
// (rp2350/spi.hpp over datasheet 12.3, the ARM PL022 at revision r1p4):
// the host on its loop-back and on four wires to the chip's OTHER
// instance as the client, the pump and the polled path, the DMA engines,
// the roles inverted, and util/spi_bus.hpp's arbiter with not one line
// changed.
//
// ONE SOURCE, BOTH ARCHITECTURES. Every letter runs on the Cortex-M33
// pair and on the Hazard3 pair, built from the rp2350-arm-* and
// rp2350-riscv-* presets. Nothing in this chapter differs between them:
// the block is one, its line is one system line under one numbering
// (3.8.4.2), and the handler names are the same on both.
//
// TWO INSTRUMENTS. THE LOOP-BACK (letters a..e): the PL022's LBM feeds
// the transmitter into the receiver with nothing on the wire, which
// proves the pump, the frame sizes, the four modes, every rate, the
// engines and the arbiter with no second peripheral. THE WIRE (letters
// f..j): SPI0 as the host and SPI1 as the client on ONE BOARD, four of
// this board's standing self-links:
//
//   GP19 (SPI0 TX)  ->  GP8  (SPI1 RX)
//   GP11 (SPI1 TX)  ->  GP16 (SPI0 RX)
//   GP18 (SPI0 SCK) ->  GP10 (SPI1 SCK)
//   GP17 (the host's chip select, a GPIO)  ->  GP9 (SPI1 CSn, the pad)
//
// The roles invert on the same four wires (letter j): SPI1 hosts with
// GP9 as its GPIO select, SPI0 listens with GP17 as its select pad. The
// client is served from ITS OWN interrupt: what the host clocks is read
// there and the next answers written ahead, so the host's transactions -
// polled or pumped - see a client that keeps up.
//
// THE RULER IS TIMER0 (rp2350/timer.hpp): 64 bits of microseconds on a
// tick generator dividing clk_ref, independent of clk_sys. The platform
// timer (rp2350/mtime.hpp) is started too, because it is what
// rp2350/delay.hpp counts a Request's `cs_setup_us` on.
//
// THE QUESTION THIS CHAPTER OWES THE OTHER CHIP, AND THE ONE PLACE THE
// CHAPTER ANSWERS ITSELF TWICE. 12.3.1 says the output enable of SSPTXD
// is controlled by the block's nSSPOE here, and that the block tristates
// its output when deselected in client mode; the idle-level list printed
// under every Motorola mode and under Microwire (12.3.4.10 to 12.3.4.14)
// says instead, in the same words each time, that nSSPOE "is not
// connected to the pad in RP2350" - which is what the RP2040 did too
// (4.4.3.14 there). Letter h points an instrument at exactly that: a line
// is called FLOATING when a pull-down and a pull-up read it differently,
// which is the only E9-proof way to tell a released pad from a driven one
// on this stepping - and its verdicts state what the pads answered.
//
// THE DMA LETTERS ARE APART ON PURPOSE (d and i): rp2350/dma.hpp is
// itself young, so a fault in an engine must not be able to fail a
// verdict about the bus. Every other letter runs with both engine slots
// empty.
//
// What is exercised, letter by letter:
//   a  the block, WIRELESS: table 645 as the driver states it, the rate
//      arithmetic at this clk_peri (75 Mbit/s at the top), the PrimeCell
//      identity against the device description, the reset state, the
//      refusals; then the loop-back proven on the pump
//   b  THE LOOP-BACK ON THE PUMP: the four modes at 8 and 16 bits,
//      sixteen frames each, byte-exact; a command phase then a data
//      phase; a read with no out buffer; the select released; and
//      cs_setup_us MEASURED on the ruler
//   c  THE POLLED PATH AT EVERY NAMED RATE, div2 to div256, 64 frames
//      each, timed on the ruler, the frame period reported
//   d  THE DMA ENGINES on the loop: 128-byte blocks with and without a
//      command phase, timed, the polled variant, and the 16-bit
//      fallback to the pump
//   e  THE KERNEL: SpiBus (= BusMaster) over SpiHost, replies in order,
//      the rejection, both sleep votes
//   f  THE WIRE: sixteen frames both ways in modes 1 and 3 and at 16
//      bits, the client's answers written eight ahead, its select pad
//      read - and modes 0 and 2 as the PL022 slave has them: ONE frame
//      per select window (SPH = 0 frames on the select)
//   g  the rate ladder on the wire, div2 downwards: where the client
//      (clk_peri / 12 at most, 12.5 MHz here) keeps up and where it
//      does not
//   h  THE TRANSMIT PAD'S OUTPUT ENABLE, which 12.3.1 claims changed:
//      the floating instrument proved both ways, a released pad, a
//      client deselected, SOD - and the overrun beside them
//   i  THE DMA ENGINES ON THE WIRE: 64 bytes each way through the host's
//      two engines, the client on its interrupt
//   j  THE ROLES INVERT on the same wires: SPI1 hosts, SPI0 listens
//
// build: boards = weact2350b,weact2350b-rv
// build: monitor_speed = 115200

#include <stdint.h>

#include <variant>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time_event.hpp"
#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/dma.hpp"
#include "rp2350/mtime.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/spi.hpp"
#include "rp2350/ticker.hpp"
#include "rp2350/timer.hpp"
#include "rp2350/uart.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/spi_bus.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using SysClock = Clock<ClockSource::pll, 150'000'000UL>;
constexpr SysClock clock;
using P = Rp2350Platform<>;
using Ruler = Timer<0>;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;
using Led = Pin<25>;

TestBench<Serial, 16> bench;

// SPI0 hosts on GP18/19/16 with GP17 as the select; SPI1 listens on
// GP10/11/8 with GP9 as its select pad. And the other way round.
constexpr SpiPins host_pins{.sck = 18, .tx = 19, .rx = 16};
constexpr SpiPins client_pins{.sck = 10, .tx = 11, .rx = 8, .cs = 9};
constexpr SpiPins host_b_pins{.sck = 10, .tx = 11, .rx = 8};
constexpr SpiPins client_a_pins{.sck = 18, .tx = 19, .rx = 16, .cs = 17};
using Host = SpiHost<0, host_pins>;
using DmaHost = SpiHost<0, host_pins, DmaTxEngine<4>, DmaRxEngine<5>>;
using Client = SpiClient<1, client_pins>;
using HostB = SpiHost<1, host_b_pins>;
using ClientA = SpiClient<0, client_a_pins>;
using CsPin = Pin<17>;
using CsPinB = Pin<9>;
using HostRxPad = Pin<16>;    ///< the host's answer line, the floating instrument's pad
using ClientTxPad = Pin<11>;  ///< the client's answer pad, at the other end of that wire

// Which task owns each instance's vector at the moment.
enum class Spi0Owner : uint8_t { none, host, dma_host, client_a };
enum class Spi1Owner : uint8_t { none, client, host_b };
volatile Spi0Owner spi0_owner = Spi0Owner::none;
volatile Spi1Owner spi1_owner = Spi1Owner::none;
volatile uint32_t spi0_isr_entries = 0;
volatile uint32_t spi1_isr_entries = 0;
volatile bool transfer_done = false;
volatile uint8_t transfer_status = 0;
bool bus_ao_live = false;

uint32_t us_now() { return Ruler::now_low(); }
void spin_us(uint32_t us) {
    const uint32_t t0 = us_now();
    while (us_now() - t0 < us) {
    }
}

uint8_t tx_buf[256];
uint8_t rx_buf[256];
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

// ---- the floating instrument -------------------------------------------------
//
// A LINE IS FLOATING WHEN THE TWO PULLS DISAGREE ABOUT IT. Reading a
// level once says nothing on this stepping: erratum RP2350-E9 makes a pad
// with its input buffer enabled leak high against its own pull-down, so
// an idle high is history and not a measurement. Reading it TWICE, once
// with each pull, is E9-proof - a pad something drives reads the same
// level both times, a pad nothing drives follows whichever pull is on -
// and the pull-down read takes E9's own workaround (the buffer enabled
// for the read alone, `Pin::read_pulsed()`), which is the only way a
// pull-down holds a pad here.
//
// It touches the PAD register alone, so the pin keeps whatever function
// it was handed; `restore_pull_up` puts it back as a host's receive pad.
template <typename Pad>
bool line_floating() {
    Pad::pull(PinPull::down);
    Pad::input_enable(false);
    spin_us(50);
    const bool with_pull_down = Pad::read_pulsed();
    Pad::pull(PinPull::up);
    Pad::input_enable(true);
    spin_us(50);
    const bool with_pull_up = Pad::read();
    return with_pull_down != with_pull_up;
}
template <typename Pad>
bool line_level() {
    Pad::pull(PinPull::up);
    Pad::input_enable(true);
    spin_us(50);
    return Pad::read();
}

// ---- the client, served from its interrupt ----------------------------------
// What it will answer (frame k) and what it heard, on either instance.
uint16_t client_answers[64];
uint16_t client_heard[64];
volatile uint16_t client_next = 0;    // the next answer to queue
volatile uint16_t client_count = 0;   // frames heard
uint16_t client_limit = 0;            // answers to queue in all

template <typename C>
void client_serve() {
    while (const auto f = C::poll()) {
        if (client_count < 64u) {
            client_heard[client_count] = *f;
        }
        client_count = client_count + 1u;
    }
    while (client_next < client_limit && C::writable()) {
        C::write(client_answers[client_next]);
        client_next = client_next + 1u;
    }
}

/// Bring the client up on `C` with `limit` answers of pattern `seed`,
/// eight of them in the FIFO before the host's clock.
template <typename C>
bool client_ready(uint16_t limit, uint16_t seed, SpiMode mode, uint8_t bits, bool drive = true) {
    client_limit = limit;
    client_next = 0;
    client_count = 0;
    for (uint16_t i = 0; i < 64u; ++i) {
        client_answers[i] = static_cast<uint16_t>((seed + 5u * i) & ((1u << bits) - 1u));
        client_heard[i] = 0xEEEEu;
    }
    if (!C::init(clock, {.mode = mode, .bits = bits, .drive_output = drive})) {
        return false;
    }
    C::interrupts(SpiInterrupt::rx | SpiInterrupt::rx_timeout, true);
    C::enable(client_answers[0]);
    client_next = 1;
    client_serve<C>();
    return true;
}

// ---- the host's transactions ----------------------------------------------------
void host_ready() {
    spi0_owner = Spi0Owner::none;
    bus_ao_live = false;
    (void)Host::init(clock);
    (void)CsPin::output(true);
    spi0_owner = Spi0Owner::host;
}

/// One transaction through a host, waited out. The status, or 0xFE.
template <typename H>
uint8_t xfer(const uint8_t* cmd, uint8_t cmd_len, const uint8_t* tx, uint8_t* rx, uint16_t len,
             SpiMode mode, SpiClock rate, SpiDataSize bits, bool polled, PinRef cs = CsPin::ref(),
             uint8_t cs_setup_us = 0) {
    typename H::Request r{};
    r.cs = cs;
    r.cs_setup_us = cs_setup_us;
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
    if (H::start(r)) {
        return H::status();
    }
    const uint32_t t0 = us_now();
    while (!transfer_done && us_now() - t0 < 200'000u) {
    }
    return transfer_done ? transfer_status : 0xFEu;
}

// =============================================================================
// a - the block, wireless
// =============================================================================
void ta_block() {
    bench.verdict("table 645 as the driver states it: the groups of four are RX, CSn, SCK, TX and "
                  "the instance is bit 3 of the pad - SPI0 on 2/6/18/22/34/38 (SCK), SPI1 on "
                  "10/14/26/30/42/46, with no second function column anywhere",
                  spi_sck_pin(0, 2) && spi_sck_pin(0, 22) && spi_sck_pin(0, 38) &&
                      spi_tx_pin(0, 7) && spi_rx_pin(0, 32) && spi_cs_pin(0, 33) &&
                      spi_sck_pin(1, 14) && spi_sck_pin(1, 46) && spi_tx_pin(1, 47) &&
                      spi_rx_pin(1, 28) && !spi_sck_pin(0, 10) && !spi_tx_pin(1, 19) &&
                      !spi_pins_valid(0, {.sck = 10, .tx = 19}));
    const auto top = spi_clock_for(SysClock::pclk_hz, 75'000'000);
    const auto twelfth = spi_clock_for(SysClock::pclk_hz, 12'500'000);
    const auto one = spi_clock_for(SysClock::pclk_hz, 1'000'000);
    print(serial, "  clk_peri=", SysClock::pclk_hz, " Hz; the fastest under 75 MHz: ",
          top ? top->cpsdvsr : 0u, " x ", top ? top->scr + 1u : 0u, " = ",
          top ? spi_sck_hz(SysClock::pclk_hz, *top) : 0u, " Hz; under 12.5 MHz: ",
          twelfth ? spi_sck_hz(SysClock::pclk_hz, *twelfth) : 0u, "; under 1 MHz: ",
          one ? spi_sck_hz(SysClock::pclk_hz, *one) : 0u, crlf);
    print(serial, "  (12.3.4.4's worked example names 70.5 Mbit/s at a clk_peri of 150 MHz; its own "
                  "equation gives clk_peri / 2, which is what the chooser computes)", crlf);
    bench.verdict("the rate chooser at 150 MHz: clk_peri / 2 at the top (75 MHz), the client's "
                  "twelfth exact at 12.5 MHz, 1 MHz exact, and nothing under 2306 Hz (254 x 256)",
                  top && top->divisor() == 2u && spi_sck_hz(SysClock::pclk_hz, *top) == 75'000'000u &&
                      twelfth && spi_sck_hz(SysClock::pclk_hz, *twelfth) == 12'500'000u &&
                      one && spi_sck_hz(SysClock::pclk_hz, *one) == 1'000'000u &&
                      !spi_clock_for(SysClock::pclk_hz, 2'306).has_value());
    using S = Pl022<0>;
    const bool reset_ok = S::reset();
    print(serial, "  SPI0 after reset: SSPSR=", hex(S::flags()), " enabled=", S::enabled(),
          " bits=", S::bits(), crlf);
    bench.verdict("the block's reset state: both FIFOs empty, the transmit one not full, disabled",
                  reset_ok && (S::flags() & (SpiFlag::tx_empty | SpiFlag::tx_not_full)) ==
                                  (SpiFlag::tx_empty | SpiFlag::tx_not_full) &&
                      !S::rx_not_empty() && !S::enabled());
    // The PrimeCell identity, which the device description states as the
    // registers' own reset values (12.3.5). SPI1 is taken out of reset
    // before its copy is read: a block this controller still holds is
    // not there to be read either.
    const bool s1_ok = Pl022<1>::reset();
    const uint32_t pid0 = S::regs().SSPPERIPHID0;
    const uint32_t pid1 = S::regs().SSPPERIPHID1;
    const uint32_t pid2 = S::regs().SSPPERIPHID2;
    const uint32_t pid3 = S::regs().SSPPERIPHID3;
    const uint32_t cid = (S::regs().SSPPCELLID3 << 24) | (S::regs().SSPPCELLID2 << 16) |
                         (S::regs().SSPPCELLID1 << 8) | S::regs().SSPPCELLID0;
    print(serial, "  SSPPERIPHID 0..3 = ", hex(pid0), " ", hex(pid1), " ", hex(pid2), " ", hex(pid3),
          " (the revision field reads ", (pid2 >> 4) & 0xFu, "); SSPPCELLID = ", hex(cid), crlf);
    bench.verdict("the PrimeCell identification registers read what this chip's own description "
                  "states for them, and SPI1's read the same",
                  pid0 == SPI_SSPPERIPHID0_RESET && pid1 == SPI_SSPPERIPHID1_RESET &&
                      pid2 == SPI_SSPPERIPHID2_RESET && pid3 == SPI_SSPPERIPHID3_RESET &&
                      cid == ((SPI_SSPPCELLID3_RESET << 24) | (SPI_SSPPCELLID2_RESET << 16) |
                              (SPI_SSPPCELLID1_RESET << 8) | SPI_SSPPCELLID0_RESET) &&
                      s1_ok && Pl022<1>::regs().SSPPERIPHID2 == pid2);
    Pl022<1>::hold();   // back where the reset controller had it
    bench.verdict("a 3-bit and a 17-bit frame, an odd prescaler, a host with its output disabled "
                  "are refused",
                  !S::configure({.bits = 3}) && !S::configure({.bits = 17}) &&
                      !S::configure({.clock = SpiClock{3, 0}}) &&
                      !S::configure({.output_disabled = true}));
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
          " interrupt entries, ", same(tx_buf, rx_buf, 16) ? "byte-exact" : "MISMATCH",
          ", cs released=", CsPin::read_out(), crlf);
    bench.verdict("LBM feeds the transmitter into the receiver: sixteen frames byte-exact with "
                  "nothing on the wire, in fewer interrupts than frames (the FIFOs batch them)",
                  st == spi_ok && same(tx_buf, rx_buf, 16) && spi0_isr_entries < 16u &&
                      spi0_isr_entries >= 2u);
    Host::release();
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
            const uint8_t st = xfer<Host>(nullptr, 0, tx_buf, rx_buf, 16, modes[m],
                                          SpiClocks::div16, bits, false);
            const bool ok = st == spi_ok && same(tx_buf, rx_buf, bytes);
            print(serial, "  mode ", m, w == 0u ? "  8-bit" : " 16-bit", ": status=", st,
                  " entries ", spi0_isr_entries, ok ? "  byte-exact" : "  MISMATCH", crlf);
            if (ok) {
                ++exact;
            }
        }
    }
    bench.verdict("the four modes at 8 and 16 bits, sixteen frames each through the pump, "
                  "byte-exact",
                  exact == 8u);
    cmd_buf[0] = 0x9F;
    cmd_buf[1] = 0x00;
    fill_pattern(tx_buf, 8, 0x77);
    for (uint16_t i = 0; i < 8; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint8_t st = xfer<Host>(cmd_buf, 2, tx_buf, rx_buf, 8, SpiMode::mode0, SpiClocks::div8,
                                  SpiDataSize::bits8, false);
    bench.verdict("a two-frame command phase then eight data frames: the data read back exact, "
                  "the command's echo discarded",
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
                  xfer<Host>(nullptr, 0, nullptr, nullptr, 0, SpiMode::mode0, SpiClocks::div8,
                             SpiDataSize::bits8, false) == spi_ok);

    // cs_setup_us, on the platform timer (rp2350/delay.hpp). The same
    // sixteen frames twice, once with 200 us of settling asked for before
    // the first clock.
    fill_pattern(tx_buf, 16, 0x55);
    // THE MEASUREMENT IS A DIFFERENCE BETWEEN TWO TRANSACTIONS, so the
    // two must cost the same but for the wait - and the FIRST polled
    // transaction of a cold image does not: it pays the XIP cache's fill
    // for the polled path, which no letter before this one has walked.
    // Measured, that fill is worth some six microseconds on the Hazard3
    // half and under one on the Cortex-M33 one, which is three per cent
    // of the 200 us being weighed. One unjudged round warms it.
    (void)xfer<Host>(nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode0, SpiClocks::div16,
                     SpiDataSize::bits8, true);
    uint32_t t0 = us_now();
    const uint8_t bare = xfer<Host>(nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode0,
                                    SpiClocks::div16, SpiDataSize::bits8, true);
    const uint32_t bare_us = us_now() - t0;
    t0 = us_now();
    const uint8_t settled = xfer<Host>(nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode0,
                                       SpiClocks::div16, SpiDataSize::bits8, true, CsPin::ref(), 200);
    const uint32_t settled_us = us_now() - t0;
    print(serial, "  a polled transaction of sixteen frames: ", bare_us, " us bare, ", settled_us,
          " us with cs_setup_us=200 (the difference ", settled_us - bare_us, ")", crlf);
    // The wait is served on the platform timer and measured on TIMER0:
    // two counters of the same microsecond with unrelated phases, so the
    // difference is the 200 asked for plus at most a count from each.
    bench.verdict("cs_setup_us is spent between the select and the first clock, on the platform "
                  "timer: 200 us asked for, and the difference between two otherwise identical "
                  "transactions is 198 to 205 us",
                  bare == spi_ok && settled == spi_ok && settled_us > bare_us &&
                      settled_us - bare_us >= 198u && settled_us - bare_us <= 205u);
    Host::release();
}

// =============================================================================
// c - the polled path at every named rate
// =============================================================================
void tc_rates() {
    host_ready();
    Host::loopback(true);
    const SpiClock rates[] = {SpiClocks::div2, SpiClocks::div4, SpiClocks::div8, SpiClocks::div16,
                              SpiClocks::div32, SpiClocks::div64, SpiClocks::div128,
                              SpiClocks::div256};
    uint8_t exact = 0;
    for (uint8_t k = 0; k < 8u; ++k) {
        fill_pattern(tx_buf, 64, static_cast<uint8_t>(0xA0u + k));
        for (uint16_t i = 0; i < 64; ++i) {
            rx_buf[i] = 0xEE;
        }
        const uint32_t t0 = us_now();
        const uint8_t st = xfer<Host>(nullptr, 0, tx_buf, rx_buf, 64, SpiMode::mode0, rates[k],
                                      SpiDataSize::bits8, true);
        const uint32_t took = us_now() - t0;
        const bool ok = st == spi_ok && same(tx_buf, rx_buf, 64);
        // Eight bits at clk_peri / divisor, in nanoseconds.
        const uint32_t wire_ns = 8u * 1000u * rates[k].divisor() / 150u;
        print(serial, "  div", rates[k].divisor(), " (", Host::sck_hz(rates[k]) / 1000u,
              " kHz): 64 frames in ", took, " us, ", took * 1000u / 64u,
              " ns a frame (the wire alone ", wire_ns, ")", ok ? "  exact" : "  MISMATCH", crlf);
        if (ok) {
            ++exact;
        }
    }
    bench.verdict("every named rate from clk_peri / 2 to / 256 carries 64 frames byte-exact on "
                  "the polled path",
                  exact == 8u);
    Host::release();
}

// =============================================================================
// d - the DMA engines on the loop
// =============================================================================
void td_dma() {
    spi0_owner = Spi0Owner::none;
    (void)DmaHost::init(clock);
    (void)CsPin::output(true);
    DmaHost::loopback(true);
    spi0_owner = Spi0Owner::dma_host;
    fill_pattern(tx_buf, 128, 0x31);
    for (uint16_t i = 0; i < 128; ++i) {
        rx_buf[i] = 0xEE;
    }
    uint32_t t0 = us_now();
    const uint8_t st = xfer<DmaHost>(nullptr, 0, tx_buf, rx_buf, 128, SpiMode::mode0,
                                     SpiClocks::div4, SpiDataSize::bits8, false);
    const uint32_t took = us_now() - t0;
    print(serial, "  128 bytes on the engines at div4 (37.5 MHz): status ", st, " in ", took,
          " us (the wire alone ", 128u * 8u * 4u / 150u, "), ",
          same(tx_buf, rx_buf, 128) ? "byte-exact" : "MISMATCH", ", cs released=",
          CsPin::read_out(), crlf);
    bench.verdict("a 128-byte block through the two engines, ISR-completed, byte-exact and the "
                  "select released",
                  st == spi_ok && same(tx_buf, rx_buf, 128) && CsPin::read_out());
    cmd_buf[0] = 0x0B;
    fill_pattern(tx_buf, 32, 0x41);
    for (uint16_t i = 0; i < 32; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint8_t st2 = xfer<DmaHost>(cmd_buf, 1, tx_buf, rx_buf, 32, SpiMode::mode0,
                                      SpiClocks::div8, SpiDataSize::bits8, false);
    bench.verdict("a command frame on the pump, then 32 data frames on the engines: exact",
                  st2 == spi_ok && same(tx_buf, rx_buf, 32));
    fill_pattern(tx_buf, 64, 0x51);
    for (uint16_t i = 0; i < 64; ++i) {
        rx_buf[i] = 0xEE;
    }
    t0 = us_now();
    const uint8_t st3 = xfer<DmaHost>(cmd_buf, 1, tx_buf, rx_buf, 64, SpiMode::mode0,
                                      SpiClocks::div2, SpiDataSize::bits8, true);
    const uint32_t took3 = us_now() - t0;
    print(serial, "  polled on the engines at div2 (75 MHz): status ", st3, " in ", took3, " us",
          crlf);
    bench.verdict("a polled request on the engines completes inside start(), exact",
                  st3 == spi_ok && same(tx_buf, rx_buf, 64));
    fill_pattern(tx_buf, 32, 0x61);
    for (uint16_t i = 0; i < 32; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint8_t st4 = xfer<DmaHost>(nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode3,
                                      SpiClocks::div16, SpiDataSize::bits16, false);
    bench.verdict("16-bit frames fall back to the pump on an engined host, exact",
                  st4 == spi_ok && same(tx_buf, rx_buf, 32) && spi0_isr_entries != 0u);
    for (uint16_t i = 0; i < 16; ++i) {
        rx_buf[i] = 0x00;
    }
    const uint8_t st5 = xfer<DmaHost>(nullptr, 0, nullptr, rx_buf, 16, SpiMode::mode0,
                                      SpiClocks::div16, SpiDataSize::bits8, false);
    bool ff = true;
    for (uint16_t i = 0; i < 16; ++i) {
        ff = ff && rx_buf[i] == 0xFFu;
    }
    bench.verdict("a read with no out buffer goes through the transmit engine's fixed 0xFF cell",
                  st5 == spi_ok && ff);
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
    print(serial, "  four queued transactions (one polled): replies ", kl::Probe::n, " [",
          kl::Probe::replies[0], " ", kl::Probe::replies[1], " ", kl::Probe::replies[2], " ",
          kl::Probe::replies[3], "]", crlf);
    bench.verdict("four transactions through SpiBus, four replies - util/spi_bus.hpp and "
                  "util/bus_master.hpp unchanged on both of this chip's architectures",
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
    post<kl::SpiArb>(PrepareSleep{.depth = SleepDepth::standby,
                                  .reply = reply_to<kl::Probe, SleepVote>()});
    kl::pump();
    bench.verdict("an IDLE bus votes for the sleep", kl::Probe::votes == 1u && kl::Probe::last_vote);
    kl::Probe::clear_tally();
    post<kl::SpiArb>(kl::request(kl::out_a, nullptr, false));
    post<kl::SpiArb>(PrepareSleep{.depth = SleepDepth::standby,
                                  .reply = reply_to<kl::Probe, SleepVote>()});
    kl::pump();
    kl::pump_until(2, 100);
    bench.verdict("a BUSY bus votes against it", kl::Probe::votes == 1u && !kl::Probe::last_vote);
    bus_ao_live = false;
    spi0_owner = Spi0Owner::none;
    Host::release();
}

// =============================================================================
// f - the wire: host SPI0, client SPI1
// =============================================================================

/// One wire transaction of `n` frames at `mode`/`bits`/`rate`: the host
/// sends pattern `seed`, the client answers its own; both directions
/// compared. Returns how many of the two directions were exact (0..2).
uint8_t wire_round(uint16_t n, SpiMode mode, uint8_t bits, SpiClock rate, uint8_t seed, bool polled,
                   uint16_t* frames_ns = nullptr, bool trace = true) {
    spi1_owner = Spi1Owner::none;
    (void)client_ready<Client>(n, static_cast<uint16_t>(0x100u + seed), mode, bits);
    spi1_isr_entries = 0;
    spi1_owner = Spi1Owner::client;
    const SpiDataSize size = bits == 16u ? SpiDataSize::bits16 : SpiDataSize::bits8;
    const uint16_t bytes = static_cast<uint16_t>(n * (bits == 16u ? 2u : 1u));
    fill_pattern(tx_buf, bytes, seed);
    for (uint16_t i = 0; i < bytes; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint32_t t0 = us_now();
    const uint8_t st = xfer<Host>(nullptr, 0, tx_buf, rx_buf, n, mode, rate, size, polled);
    const uint32_t took = us_now() - t0;
    if (frames_ns != nullptr) {
        *frames_ns = static_cast<uint16_t>(took * 1000u / n);
    }
    spin_us(200);   // let the client's interrupt take the tail
    bool heard = st == spi_ok && client_count == n;
    bool answered = st == spi_ok;
    int32_t first_bad_rx = -1;
    int32_t first_bad_heard = -1;
    for (uint16_t i = 0; i < n; ++i) {
        const uint16_t sent = bits == 16u
                                  ? static_cast<uint16_t>(tx_buf[2u * i] | (tx_buf[2u * i + 1u] << 8))
                                  : tx_buf[i];
        const uint16_t got = bits == 16u
                                 ? static_cast<uint16_t>(rx_buf[2u * i] | (rx_buf[2u * i + 1u] << 8))
                                 : rx_buf[i];
        if (client_heard[i] != sent && first_bad_heard < 0) { first_bad_heard = i; }
        if (got != client_answers[i] && first_bad_rx < 0) { first_bad_rx = i; }
        heard = heard && client_heard[i] == sent;
        answered = answered && got == client_answers[i];
    }
    if (trace && (!heard || !answered)) {
        print(serial, "    [status ", st, " client isr entries ", spi1_isr_entries, " overrun ",
              Client::overrun(), "; host rx first bad at ", first_bad_rx, ": ",
              first_bad_rx >= 0 ? hex(rx_buf[first_bad_rx]) : hex(0u), " for ",
              first_bad_rx >= 0 ? hex(client_answers[first_bad_rx]) : hex(0u),
              "; heard first bad at ", first_bad_heard, ": ",
              first_bad_heard >= 0 ? hex(client_heard[first_bad_heard]) : hex(0u), " for ",
              first_bad_heard >= 0 ? hex(tx_buf[first_bad_heard]) : hex(0u), "]", crlf);
    }
    spi1_owner = Spi1Owner::none;
    Client::release();
    return static_cast<uint8_t>((heard ? 1u : 0u) + (answered ? 1u : 0u));
}

void tf_wire() {
    host_ready();
    Host::loopback(false);
    // ONE CORE SERVES BOTH ENDS here: the host's interrupt and the
    // client's compete for it, so the pumped rounds run slowly enough
    // that the client's interrupt has time to refill its answers between
    // frames, and the polled rounds - whose host takes no interrupt -
    // run one rung faster. THE FIRST INTERRUPT OF A COLD ROUTINE PAYS
    // THE CACHE FILL of the flash it runs from, so one unjudged round
    // warms both paths.
    (void)wire_round(16, SpiMode::mode1, 8, SpiClocks::div64, 0x2F, false, nullptr, false);
    uint8_t exact = 0;
    const SpiMode phased[] = {SpiMode::mode1, SpiMode::mode3};
    for (uint8_t k = 0; k < 4u; ++k) {
        const SpiMode m = phased[k & 1u];
        const bool polled = k >= 2u;
        const SpiClock rate = polled ? SpiClocks::div32 : SpiClocks::div64;
        const uint8_t r = wire_round(16, m, 8, rate, static_cast<uint8_t>(0x30u + k), polled);
        print(serial, "  mode ", static_cast<uint8_t>(m), " at 8 bits, ",
              polled ? "polled at 4.7 MHz" : "pumped at 2.3 MHz", ": the client heard ",
              client_count, " frames, ",
              r == 2u ? "both directions exact" : r == 1u ? "ONE DIRECTION WRONG" : "BOTH WRONG",
              crlf);
        exact = static_cast<uint8_t>(exact + r);
    }
    bench.verdict("sixteen frames both ways in modes 1 and 3, pumped and polled, the client eight "
                  "answers ahead from its interrupt: every direction exact",
                  exact == 8u);
    const uint8_t w = wire_round(16, SpiMode::mode3, 16, SpiClocks::div64, 0x40, false);
    bench.verdict("sixteen 16-bit frames both ways", w == 2u);
    const uint8_t s = wire_round(4, SpiMode::mode1, 8, SpiClocks::div32, 0x50, true);
    bench.verdict("four frames, fewer than a FIFO: both ways (the tail by the receive timeout)",
                  s == 2u);
    // SPH = 0: the PL022 slave frames on the select and takes one frame
    // per window.
    (void)wire_round(16, SpiMode::mode0, 8, SpiClocks::div32, 0x38, true, nullptr, false);
    const uint16_t heard0 = client_count;
    (void)wire_round(16, SpiMode::mode2, 8, SpiClocks::div32, 0x3A, true, nullptr, false);
    print(serial, "  modes 0 and 2 under one select window of sixteen frames: the client heard ",
          heard0, " and ", client_count, crlf);
    bench.verdict("with SPH = 0 the PL022 client takes ONE frame per select window (the fact a "
                  "multi-frame transaction under a held select must know)",
                  heard0 == 1u && client_count == 1u);
    spi1_owner = Spi1Owner::none;
    (void)client_ready<Client>(4, 0x60, SpiMode::mode0, 8);
    bench.verdict("the client's select pad reads not-selected between transactions",
                  !Client::selected());
    Client::release();
    spi0_owner = Spi0Owner::none;
    Host::release();
}

// =============================================================================
// g - the ladder on the wire
// =============================================================================
void tg_ladder() {
    host_ready();
    Host::loopback(false);
    const SpiClock rates[] = {SpiClocks::div2, SpiClocks::div4, SpiClocks::div8, SpiClock{2, 5},
                              SpiClocks::div16, SpiClocks::div32, SpiClocks::div64};
    const char* names[] = {"div2 (75 MHz)",   "div4 (37.5)",  "div8 (18.75)",
                           "div12 (12.5, the client's ceiling)", "div16 (9.4)", "div32 (4.7)",
                           "div64 (2.3)"};
    uint8_t first_good = 0xFF;
    for (uint8_t k = 0; k < 7u; ++k) {
        uint16_t ns = 0;
        const uint8_t r = wire_round(32, SpiMode::mode1, 8, rates[k],
                                     static_cast<uint8_t>(0x70u + k), true, &ns, false);
        print(serial, "  ", names[k],
              ": ", r == 2u ? "both ways exact" : r == 1u ? "one way wrong" : "both ways wrong",
              " (", ns, " ns a frame)", crlf);
        if (r == 2u && first_good == 0xFFu) {
            first_good = k;
        }
    }
    print(serial, "  the fastest rung that held both ways: ",
          first_good == 0xFFu ? "none" : names[first_good],
          " (12.3.4.4 puts the client's ceiling at clk_peri / 12; where the silicon still keeps "
          "up above it is what this line reports)", crlf);
    bench.verdict("the wire holds both ways at the client's clk_peri / 12 ceiling (12.5 MHz) and "
                  "below",
                  first_good != 0xFFu && first_good <= 3u);
    spi0_owner = Spi0Owner::none;
    Host::release();
}

// =============================================================================
// h - the transmit pad's output enable, and the overrun
// =============================================================================
void th_output_enable() {
    host_ready();
    Host::loopback(false);
    spi1_owner = Spi1Owner::none;

    // THE INSTRUMENT, PROVED BOTH WAYS on the GP11 -> GP16 wire before
    // anything is asked of it: the client's answer pad driven low by
    // software, then released altogether.
    (void)ClientTxPad::output(false);
    const bool driven_says_floating = line_floating<HostRxPad>();
    const bool driven_level = line_level<HostRxPad>();
    (void)ClientTxPad::release();
    const bool released_says_floating = line_floating<HostRxPad>();
    print(serial, "  the instrument on GP11 -> GP16: a pad driven low reads floating=",
          driven_says_floating, " level=", driven_level, "; a released pad reads floating=",
          released_says_floating, crlf);
    bench.verdict("a line is called FLOATING when a pull-down and a pull-up read it differently: "
                  "driven low it is not, released it is - the instrument, E9-proof and proved "
                  "both ways",
                  !driven_says_floating && !driven_level && released_says_floating);

    // A DARK CLIENT: the answer pad released, the block listening.
    (void)client_ready<Client>(16, 0x80, SpiMode::mode1, 8, false);
    spi1_owner = Spi1Owner::client;
    fill_pattern(tx_buf, 16, 0x81);
    for (uint16_t i = 0; i < 16; ++i) {
        rx_buf[i] = 0x00;
    }
    const uint8_t st = xfer<Host>(nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode1, SpiClocks::div32,
                                  SpiDataSize::bits8, true);
    bool ff = true;
    for (uint16_t i = 0; i < 16; ++i) {
        ff = ff && rx_buf[i] == 0xFFu;
    }
    spin_us(200);
    print(serial, "  a dark client (its answer pad released): the host read ", hex(rx_buf[0]), " ",
          hex(rx_buf[1]), "..., the client heard ", client_count, " frames", crlf);
    bench.verdict("a dark client answers nothing (the host's pull-up reads 0xFF) and still hears "
                  "everything",
                  st == spi_ok && ff && client_count == 16u);

    // THE PAD ON THE PERIPHERAL, THE CLIENT DESELECTED. THE CHAPTER SAYS
    // BOTH THINGS: 12.3.1 has the output enable of SSPTXD controlled by
    // nSSPOE and the block tristating when deselected in client mode,
    // while the idle-level list under every one of the four Motorola
    // modes (12.3.4.10 to 12.3.4.14) says in the same words each time
    // that nSSPOE "is not connected to the pad in RP2350". The pads
    // decide.
    Client::drive_output(true);
    spin_us(200);
    const bool idle_floating = line_floating<HostRxPad>();
    const bool idle_level = line_level<HostRxPad>();
    print(serial, "  the client's pad on the peripheral and its select high: the line reads "
                  "floating=", idle_floating, " level=", idle_level, crlf);
    bench.verdict("a DESELECTED client leaves its transmit pad DRIVEN: 12.3.1 promises a tristate, "
                  "and the idle list of each Motorola mode says nSSPOE is not connected to the pad "
                  "on this chip - the pads agree with the second, as they do on the RP2040",
                  !idle_floating);

    // SOD, the block's own slave-output-disable bit, with the pad on the
    // peripheral: the same question asked of the bit rather than of the
    // select.
    Client::sod(true);
    spin_us(200);
    const bool sod_floating = line_floating<HostRxPad>();
    const bool sod_level = line_level<HostRxPad>();
    client_count = 0;
    client_next = 0;
    client_serve<Client>();
    for (uint16_t i = 0; i < 16; ++i) {
        rx_buf[i] = 0x00;
    }
    (void)xfer<Host>(nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode1, SpiClocks::div32,
                     SpiDataSize::bits8, true);
    bool sod_ff = true;
    for (uint16_t i = 0; i < 16; ++i) {
        sod_ff = sod_ff && rx_buf[i] == 0xFFu;
    }
    print(serial, "  SOD set with the pad on the peripheral: the line reads floating=", sod_floating,
          " level=", sod_level, ", the host read ", hex(rx_buf[0]), " ", hex(rx_buf[1]), " ",
          hex(rx_buf[2]), " (the answers would be ", hex(client_answers[0]), " ",
          hex(client_answers[1]), " ", hex(client_answers[2]), ")", crlf);
    bench.verdict("SOD leaves the transmit pad DRIVEN too - nSSPOE reaches no pad here either - but "
                  "it does keep the block's answers off the wire: the host clocks a steady HIGH "
                  "(0xFF) and not the frames waiting in the client's FIFO",
                  !sod_floating && sod_level && sod_ff);

    // Under SOD the transmit FIFO is not consumed on the RP2040; the
    // client comes up again either way for the round that judges it.
    Client::sod(false);
    spi1_owner = Spi1Owner::none;
    (void)client_ready<Client>(16, 0x80, SpiMode::mode1, 8, true);
    spi1_owner = Spi1Owner::client;
    for (uint16_t i = 0; i < 16; ++i) {
        rx_buf[i] = 0x00;
    }
    const uint8_t st2 = xfer<Host>(nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode1, SpiClocks::div32,
                                   SpiDataSize::bits8, true);
    bool answered = st2 == spi_ok;
    for (uint16_t i = 0; i < 16; ++i) {
        answered = answered && rx_buf[i] == client_answers[i];
    }
    print(serial, "  SOD clear and the client up again: status ", st2, ", the host read ",
          hex(rx_buf[0]), " ", hex(rx_buf[1]), " ...", crlf);
    bench.verdict("SOD clear and the client up again, it answers", answered);

    // The overrun: the client's interrupt silenced, sixteen frames into
    // its eight-deep FIFO.
    spi1_owner = Spi1Owner::none;
    Client::clear_overrun();
    (void)xfer<Host>(nullptr, 0, tx_buf, nullptr, 16, SpiMode::mode1, SpiClocks::div32,
                     SpiDataSize::bits8, true);
    const bool over = Client::overrun();
    Client::clear_overrun();
    print(serial, "  sixteen frames at a client nobody reads: overrun=", over, ", cleared=",
          !Client::overrun(), crlf);
    bench.verdict("a client that does not read overruns its FIFO, says so, and the flag clears by "
                  "writing one",
                  over && !Client::overrun());
    Client::release();
    spi0_owner = Spi0Owner::none;
    Host::release();
}

// =============================================================================
// i - the DMA engines on the wire
// =============================================================================
void ti_dma_wire() {
    spi0_owner = Spi0Owner::none;
    (void)DmaHost::init(clock);
    (void)CsPin::output(true);
    DmaHost::loopback(false);
    spi0_owner = Spi0Owner::dma_host;
    spi1_owner = Spi1Owner::none;
    (void)client_ready<Client>(64, 0x90, SpiMode::mode1, 8, true);
    spi1_owner = Spi1Owner::client;
    fill_pattern(tx_buf, 64, 0x91);
    for (uint16_t i = 0; i < 64; ++i) {
        rx_buf[i] = 0xEE;
    }
    // div64 (2.3 MHz): the client's interrupt refills its answers four at
    // a time, and the engines clock without a gap (letter f says why).
    const uint8_t st = xfer<DmaHost>(nullptr, 0, tx_buf, rx_buf, 64, SpiMode::mode1,
                                     SpiClocks::div64, SpiDataSize::bits8, false);
    spin_us(200);
    bool heard = client_count == 64u;
    bool answered = st == spi_ok;
    for (uint16_t i = 0; i < 64u; ++i) {
        heard = heard && client_heard[i] == tx_buf[i];
        answered = answered && rx_buf[i] == client_answers[i];
    }
    int32_t bad = -1;
    for (uint16_t i = 0; i < 64u && bad < 0; ++i) {
        if (rx_buf[i] != client_answers[i]) { bad = i; }
    }
    print(serial, "  64 bytes on the engines at 2.3 MHz: status ", st, ", the client heard ",
          client_count, " (", heard ? "exact" : "WRONG", "), the host read its answers ",
          answered ? "exact" : "WRONG", " (first bad at ", bad, ": ",
          bad >= 0 ? hex(rx_buf[bad]) : hex(0u), " for ",
          bad >= 0 ? hex(client_answers[bad]) : hex(0u), ")", crlf);
    bench.verdict("the host's two engines and the client's interrupt: both directions exact",
                  heard && answered);
    spi1_owner = Spi1Owner::none;
    Client::release();
    spi0_owner = Spi0Owner::none;
    DmaHost::release();
}

// =============================================================================
// j - the roles invert
// =============================================================================
void tj_roles() {
    spi0_owner = Spi0Owner::none;
    spi1_owner = Spi1Owner::none;
    (void)client_ready<ClientA>(16, 0xA0, SpiMode::mode3, 8, true);
    spi0_owner = Spi0Owner::client_a;
    (void)HostB::init(clock);
    (void)CsPinB::output(true);
    spi1_owner = Spi1Owner::host_b;
    fill_pattern(tx_buf, 16, 0xA1);
    for (uint16_t i = 0; i < 16; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint8_t st = xfer<HostB>(nullptr, 0, tx_buf, rx_buf, 16, SpiMode::mode3, SpiClocks::div32,
                                   SpiDataSize::bits8, true, CsPinB::ref());
    spin_us(200);
    bool heard = client_count == 16u;
    bool answered = st == spi_ok;
    for (uint16_t i = 0; i < 16u; ++i) {
        heard = heard && client_heard[i] == tx_buf[i];
        answered = answered && rx_buf[i] == client_answers[i];
    }
    print(serial, "  SPI1 hosting on GP9's select, SPI0 listening on GP17: the client heard ",
          client_count, " (", heard ? "exact" : "WRONG", "), the host read ",
          answered ? "exact" : "WRONG", crlf);
    bench.verdict("the roles invert on the same four wires: both directions exact",
                  heard && answered);
    spi1_owner = Spi1Owner::none;
    HostB::release();
    spi0_owner = Spi0Owner::none;
    ClientA::release();
}

void banner() {
    print(serial, crlf,
          "test_rp2350_spi - the RP2350 SPI (datasheet 12.3, the PL022): SPI0 hosts on "
          "GP18/19/16 + GP17, SPI1 listens on GP10/11/8 + GP9; clk_peri=",
          SysClock::pclk_hz, " Hz, core=",
          core_kind == CoreKind::hazard3 ? "Hazard3" : "Cortex-M33", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
//
// The instance lines and the DMA line, bound by the names the crt gives
// them - the same names on both architectures, through the Arm vector
// table on one and Hazard3's own dispatch on the other.

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
        case Spi0Owner::client_a:
            (void)ClientA::isr();
            client_serve<ClientA>();
            break;
        default:
            brio::Pl022<0>::interrupts(brio::SpiInterrupt::all, false);
            break;
    }
}

extern "C" void isr_spi1() {
    spi1_isr_entries = spi1_isr_entries + 1u;
    switch (spi1_owner) {
        case Spi1Owner::client:
            (void)Client::isr();
            client_serve<Client>();
            break;
        case Spi1Owner::host_b:
            if (HostB::isr()) {
                transfer_status = HostB::status();
                transfer_done = true;
            }
            break;
        default:
            brio::Pl022<1>::interrupts(brio::SpiInterrupt::all, false);
            break;
    }
}

extern "C" void isr_dma_0() {
    if (spi0_owner == Spi0Owner::dma_host && DmaHost::dma_isr()) {
        transfer_status = DmaHost::status();
        transfer_done = true;
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool timer_ok = brio::Timer<0>::init(clock);
    const bool ruler_ok = brio::Mtime::start(clock);
    const bool dma_ok = brio::Dma::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    (void)Led::output(false);
    brio::DmaLine<0>::enable();
    brio::enable_interrupts();

    bench.letter('a', "the block, wireless; the loop-back proven", ta_block);
    bench.letter('b', "the loop-back on the pump; cs_setup_us measured", tb_pump);
    bench.letter('c', "the polled path at every named rate", tc_rates);
    bench.letter('d', "the DMA engines on the loop", td_dma);
    bench.letter('e', "the kernel: SpiBus over SpiHost", te_kernel);
    bench.letter('f', "the wire: SPI0 hosts, SPI1 listens", tf_wire);
    bench.letter('g', "the rate ladder on the wire", tg_ladder);
    bench.letter('h', "the transmit pad's output enable, and the overrun", th_output_enable);
    bench.letter('i', "the DMA engines on the wire", ti_dma_wire);
    bench.letter('j', "the roles invert", tj_roles);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL150" : "FAILED",
                    " timer0=", timer_ok ? "1us" : "FAILED", " mtime=", ruler_ok ? "1us" : "FAILED",
                    " dma=", dma_ok ? "released" : "FAILED", " tick=", tick_ok ? "on" : "FAILED",
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
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        bench.prompt();
    }
}
