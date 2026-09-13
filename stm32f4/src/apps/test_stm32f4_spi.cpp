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
// THE INSTRUMENT IS TWO DEVICES SOLDERED TO THE BOARD, which is why this
// suite builds for the STM32F429I-DISC1 alone: a bus driver measured
// against nothing is a register read-back, and this board carries a
// three-axis gyroscope and a display controller on SPI5 -
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
// What is exercised, letter by letter:
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
// build: boards = f429zi
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
