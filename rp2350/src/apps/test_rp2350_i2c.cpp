// test_rp2350_i2c - the reference bench suite for this chip's I2C
// (rp2350/i2c.hpp over datasheet 12.2, the Synopsys DW_apb_i2c v2.03a):
// the block WIRELESS, and THIS CHIP'S PADS - the isolation latch, the pad
// register the driver writes, the unstick's hand-over - on the high half
// of the bank, above GP31. ONE SOURCE, BOTH ARCHITECTURES: nothing in
// chapter 12.2 is a processor's, so every verdict below is expected to
// read the same on the Cortex-M33 half and on the Hazard3 half.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// THE WIRE IS THE DESK PARTNER'S. I2C0 on GP36 (SDA) / GP37 (SCL) is
// wired to the WeAct RP2040's I2C1, each line with 1.5 kOhm to THAT
// board's 3V3 (docs/boards/weact-rp2350b.md). A tenure's far end is that
// board's peer, never this chip's other instance; what this suite judges
// on the wire needs no far end - the pads, and an address nobody on the
// bus answers. Letter j reads the pull-ups first through
// rp2350/src/apps/pair_power.hpp (erratum RP2350-E9's own workaround: the
// pad's pull-down, its input buffer pulsed), which is also the proof the
// RP2040 is powered, and drives nothing when they read low.
//
// THE RULER IS SYSTEM TIMER 0 (rp2350/timer.hpp). THE UNSTICK HAS A
// RULER OF ITS OWN - the platform timer, which rp2350/delay.hpp counts
// because it is the one counter both architectures have - and main
// starts it, on both halves, because on the Cortex-M33 half nothing else
// does.
//
// What is exercised, letter by letter:
//   a  the block, WIRELESS: 9.4 as the driver states it over the whole
//      bank, the timing arithmetic at this clk_sys (table 1053's floors
//      reproduced), the reset state, the FIFO depths the block reports,
//      the refusals, the enable and the disable through IC_ENABLE_STATUS
//   j  THE PADS, which is where this chip differs from its predecessor,
//      on GP36/GP37: the link's pull-ups read the way E9 forces, the
//      isolation latch dropped by the driver's own hand-over, the whole
//      pad register read back against 12.2.1.3, an address phase nobody
//      answers carried by them, the unstick's hand-over through the HIGH
//      half's SIO registers and back, and both pads left isolated again
//      by release()
//
// build: boards = weact2350b,weact2350b-rv
// build: monitor_speed = 115200

#include <stdint.h>

#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/i2c.hpp"
#include "rp2350/mtime.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/ticker.hpp"
#include "rp2350/timer.hpp"
#include "rp2350/uart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

#include "pair_power.hpp"

namespace {

using namespace brio;

using SysClock = Clock<ClockSource::pll, 150'000'000>;
constexpr SysClock clock;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;
using Ruler = Timer<0>;

TestBench<Serial> bench;

// I2C0 on the pair's wires: GP37 (SCL), GP36 (SDA).
constexpr I2cPins host_pins{.scl = 37, .sda = 36};
using Host = I2cHost<0, host_pins>;
using SdaPad = Pin<36>;
using SclPad = Pin<37>;
/// An address nothing on the pair's bus answers: the RP2040's peer
/// listens at 0x6B alone (avrdx/src/apps/twi_link.hpp), and its dark
/// images listen at none.
constexpr uint8_t nobody = 0x42;

volatile bool host_live = false;
volatile bool transfer_done = false;
volatile uint8_t transfer_status = 0;

uint32_t us_now() { return Ruler::now_low(); }
const char* arch_name() { return core_kind == CoreKind::hazard3 ? "Hazard3" : "Cortex-M33"; }

void host_ready() {
    host_live = false;
    (void)Host::init(clock);
    host_live = true;
}

/// One address-only tenure (the probe) through the host, waited out. The
/// status, or 0xFE for a tenure that never answered (the engine
/// recover()ed).
uint8_t probe(uint8_t addr) {
    Host::Request r{};
    r.addr = addr;
    r.speed = I2cSpeed::fast_400k;
    transfer_done = false;
    if (Host::start(r)) {
        return Host::status();
    }
    const uint32_t t0 = us_now();
    while (!transfer_done && us_now() - t0 < 200'000u) {
    }
    if (!transfer_done) {
        (void)Host::recover();
        return 0xFEu;
    }
    return transfer_status;
}


// =============================================================================
// a - the block, wireless
// =============================================================================
void ta_block() {
    bench.verdict("9.4 as the driver states it over the WHOLE bank: I2C0 SDA on 0/4/../44 and SCL on "
                  "1/5/../45, I2C1 SDA on 2/6/../46 and SCL on 3/7/../47, one function column and no "
                  "second one",
                  i2c_sda_pin(0, 0) && i2c_sda_pin(0, 44) && i2c_scl_pin(0, 1) && i2c_scl_pin(0, 45) &&
                      i2c_sda_pin(1, 2) && i2c_sda_pin(1, 46) && i2c_scl_pin(1, 3) && i2c_scl_pin(1, 47) &&
                      !i2c_sda_pin(0, 2) && !i2c_scl_pin(1, 13) && !i2c_sda_pin(1, 48) &&
                      !i2c_pins_valid(0, {.scl = 12, .sda = 13}) &&
                      Rp2350DwApbI2c::pad_function == PinFunction::i2c);
    for (uint8_t s = 0; s < i2c_speed_count; ++s) {
        const auto sp = static_cast<I2cSpeed>(s);
        const auto t = i2c_timing_for(SysClock::hz, sp);
        print(serial, "  ", i2c_speed_hz(sp) / 1000u, " kHz at ", SysClock::hz / 1'000'000u, " MHz: HCNT ",
              t ? t->hcnt : 0u, " LCNT ", t ? t->lcnt : 0u, " SPKLEN ", t ? t->spklen : 0u, " hold ",
              t ? t->sda_tx_hold : 0u, " setup ", t ? t->sda_setup : 0u, " -> ",
              t ? i2c_scl_hz(SysClock::hz, *t) : 0u, " Hz on an ideal wire", crlf);
    }
    bench.verdict("the timing arithmetic: the three speeds solved at this clk_sys produce their asked rate "
                  "on an ideal wire (the SPKLEN + 8 cycles the block adds subtracted from the counts)",
                  i2c_scl_hz(SysClock::hz, *i2c_timing_for(SysClock::hz, I2cSpeed::standard_100k)) == 100'000u &&
                      i2c_scl_hz(SysClock::hz, *i2c_timing_for(SysClock::hz, I2cSpeed::fast_400k)) >= 399'000u &&
                      i2c_scl_hz(SysClock::hz, *i2c_timing_for(SysClock::hz, I2cSpeed::fast_plus_1m)) == 1'000'000u);
    bench.verdict("table 1053's floors reproduced: fast mode at 12 MHz is (LCNT 15, HCNT 6), standard at "
                  "2.7 MHz is (12, 6), fast-mode-plus needs 32 MHz, a hertz below each is refused",
                  i2c_timing_for(12'000'000u, I2cSpeed::fast_400k)->lcnt == 15u &&
                      i2c_timing_for(12'000'000u, I2cSpeed::fast_400k)->hcnt == 6u &&
                      i2c_timing_for(2'700'000u, I2cSpeed::standard_100k)->lcnt == 12u &&
                      !i2c_timing_for(11'999'999u, I2cSpeed::fast_400k).has_value() &&
                      !i2c_timing_for(31'999'999u, I2cSpeed::fast_plus_1m).has_value() &&
                      i2c_timing_for(32'000'000u, I2cSpeed::fast_plus_1m).has_value());
    using S = DwApbI2c<0>;
    const bool reset_ok = S::reset();
    const uint32_t con = S::con();
    print(serial, "  I2C0 after reset: IC_CON=", hex(con), " SPKLEN=", S::timing(I2cSpeed::fast_400k).spklen,
          " SDA_HOLD=", S::timing(I2cSpeed::fast_400k).sda_tx_hold, " enabled=", S::enabled(),
          " status=", hex(S::flags()), " IC_COMP_PARAM_1=", hex(S::regs().IC_COMP_PARAM_1),
          " line=", static_cast<uint32_t>(S::irq()), crlf);
    bench.verdict("the block's reset state: a host with its client half disabled, fast speed code, restarts "
                  "enabled (IC_CON 0x65), the spike filter at 7, the hold at 1, disabled, both FIFOs empty - "
                  "and the component parameter register reading zero, as 12.2 leaves it",
                  reset_ok && con == 0x65u && S::timing(I2cSpeed::fast_400k).spklen == 7u &&
                      S::timing(I2cSpeed::fast_400k).sda_tx_hold == 1u && !S::enabled() && S::tx_empty() &&
                      !S::rx_not_empty() && S::regs().IC_COMP_PARAM_1 == 0u);
    bench.verdict("the two interrupt lines are 3.2's, which are NOT the RP2040's 23 and 24: I2C0_IRQ is 36 "
                  "and I2C1_IRQ is 37, the same numbers under both architectures (3.8.4.2)",
                  static_cast<uint32_t>(DwApbI2c<0>::irq()) == 36u &&
                      static_cast<uint32_t>(DwApbI2c<1>::irq()) == 37u);
    // The transmit FIFO's depth, measured: entries queued under
    // TX_CMD_BLOCK until the FIFO says full, then flushed by a disable.
    S::enable();
    S::command_block(true);
    uint8_t depth = 0;
    while (S::tx_not_full() && depth < 64u) {
        S::push(i2c_write_entry(depth));
        ++depth;
    }
    const uint8_t counted = S::tx_count();
    const bool over = (S::raw_pending() & I2cInterrupt::tx_over) == 0u;
    S::push(i2c_write_entry(0xFF));   // one too many
    const bool over_now = (S::raw_pending() & I2cInterrupt::tx_over) != 0u;
    (void)S::disable();
    S::command_block(false);
    S::clear_all();
    print(serial, "  the transmit FIFO under TX_CMD_BLOCK: full after ", depth, " entries (TXFLR ", counted,
          "), TX_OVER ", over_now ? "raised" : "NOT raised", " on the next; after the disable TXFLR ",
          S::tx_count(), crlf);
    bench.verdict("the transmit FIFO is sixteen deep (12.2.1), measured under TX_CMD_BLOCK: full at sixteen, "
                  "TX_OVER on the seventeenth, flushed by the disable",
                  depth == 16u && counted == 16u && over && over_now && S::tx_count() == 0u);
    const bool cfg = S::configure({.speed = I2cSpeed::fast_plus_1m}) &&
                     S::timing(*i2c_timing_for(SysClock::hz, I2cSpeed::fast_plus_1m)) && S::target(0x42);
    const I2cTiming back = S::timing(I2cSpeed::fast_plus_1m);
    const I2cTiming asked = *i2c_timing_for(SysClock::hz, I2cSpeed::fast_plus_1m);
    print(serial, "  a configuration read back: HCNT ", back.hcnt, " LCNT ", back.lcnt, " (asked for ",
          asked.hcnt, " and ", asked.lcnt, "), TAR ", hex(S::target()), crlf);
    bench.verdict("a configuration, a timing and a target are taken with the block disabled and read back "
                  "as they were written",
                  cfg && back.hcnt == asked.hcnt && back.lcnt == asked.lcnt &&
                      back.spklen == asked.spklen && S::target() == 0x42u &&
                      (S::con() & I2C_IC_CON_SPEED_BITS) == (2u << I2C_IC_CON_SPEED_LSB));
    S::enable();
    const bool running = S::running();
    const bool refused = !S::configure({}) && !S::timing(back) && !S::target(0x10) && !S::own_address(0x11);
    (void)S::target(0x10);   // ignored by the hardware too
    bench.verdict("enabled: IC_ENABLE_STATUS reports it, and the configuration, the timing, the target and "
                  "the own address are refused",
                  running && refused && S::target() == 0x42u);
    const uint32_t t0 = us_now();
    const bool off = S::disable();
    const uint32_t took = us_now() - t0;
    print(serial, "  disable on an idle bus: ", off ? "reported" : "NOT reported", " in ", took, " us", crlf);
    bench.verdict("the disable of an idle block is reported by IC_ENABLE_STATUS within a few microseconds",
                  off && took < 20u);
    S::hold();
}

// =============================================================================
// j - the pads, and the isolation latch this chip added
// =============================================================================
void tj_pads() {
    const pair::PartnerRail rail = pair::partner_rail(clock);
    print(serial, "  GP36 and GP37 on their own pull-down, the buffer pulsed (E9's workaround): SDA ",
          rail.sda ? "HIGH" : "low", ", SCL ", rail.scl ? "HIGH" : "low", crlf);
    bench.verdict("THE LINK'S PULL-UPS READ THE WAY E9 FORCES: both lines high through 1.5 kOhm to the "
                  "RP2040's 3V3 against the pad's own pull-down - the partner powered, the pads free to drive",
                  rail.sda && rail.scl);
    if (!rail.up()) {
        print(serial, "  THE PARTNER IS UNPOWERED: no link pad is driven (an RP2040 pad is not failsafe)", crlf);
        return;
    }
    host_ready();
    const uint32_t pad36 = Gpio::pad(36);
    const uint32_t pad37 = Gpio::pad(37);
    const bool fn_ok = SdaPad::function() == PinFunction::i2c && SclPad::function() == PinFunction::i2c;
    const bool iso_ok = !SdaPad::isolated() && !SclPad::isolated();
    const bool ie_ok = SdaPad::input_enabled() && SclPad::input_enabled();
    const uint32_t wanted = pad_value(i2c_pad_config);
    print(serial, "  after init: GP36 pad=", hex(pad36), " GP37 pad=", hex(pad37), " (the driver asks for ",
          hex(wanted), "), FUNCSEL ", static_cast<uint32_t>(SdaPad::function()), "/",
          static_cast<uint32_t>(SclPad::function()), ", isolated ", SdaPad::isolated(), "/", SclPad::isolated(),
          crlf);
    bench.verdict("THE HAND-OVER DROPS THE ISOLATION LATCH: both pads read back FUNCSEL 3 with ISO clear, "
                  "the input buffer on, and the whole pad register exactly what 12.2.1.3 asks for - the "
                  "pull-up, the slew limit, the Schmitt trigger",
                  fn_ok && iso_ok && ie_ok && pad36 == wanted && pad37 == wanted);
    // The pads carry an address phase: the host clocks nine SCL periods
    // and reads the ACK slot high, which needs both pads working - SCL
    // driven and released, SDA released and read back.
    const uint8_t st = probe(nobody);
    print(serial, "  the probe of ", hex(nobody), ", which nothing on the bus answers: status ", st, crlf);
    bench.verdict("... and the pads so configured carry an address phase: nobody home is i2c_nack_addr",
                  st == i2c_nack_addr);
    // The unstick takes the pads by hand - through the HIGH half's SIO
    // registers, these being GP36 and GP37 - and gives them back.
    const uint8_t cleared = Host::unstick();
    const uint32_t pad36_after = Gpio::pad(36);
    const uint32_t pad37_after = Gpio::pad(37);
    const bool fn_back = SdaPad::function() == PinFunction::i2c && SclPad::function() == PinFunction::i2c;
    const uint8_t st2 = probe(nobody);
    print(serial, "  after unstick() (", cleared, "): GP36 pad=", hex(pad36_after), " GP37 pad=", hex(pad37_after),
          ", the next probe ", st2, crlf);
    bench.verdict("the unstick reads a healthy bus above GP31 (0), takes both pads to SIO and hands them back "
                  "to the block with the same pad register: the probe after it lands unchanged",
                  cleared == 0u && pad36_after == wanted && pad37_after == wanted && fn_back && st2 == i2c_nack_addr);
    host_live = false;
    Host::release();
    const uint32_t pad36_released = Gpio::pad(36);
    print(serial, "  after release(): GP36 pad=", hex(pad36_released), " FUNCSEL ",
          static_cast<uint32_t>(SdaPad::function()), crlf);
    bench.verdict("release() puts the pads back where a reset leaves them (9.11's 0x116: isolated, the "
                  "input buffer off, a pull-down) with no owner",
                  pad36_released == PADS_BANK0_GPIO0_RESET && SdaPad::isolated() && !SdaPad::input_enabled() &&
                      SdaPad::function() == PinFunction::none && SclPad::isolated() &&
                      SclPad::function() == PinFunction::none);
}

void banner() {
    print(serial, crlf, "test_rp2350_i2c - the RP2350 I2C (datasheet 12.2, DW_apb_i2c v2.03a) on the ",
          arch_name(), " half: the block wireless, I2C0's pads on GP37/GP36 (the pair's wires); clk_sys=",
          SysClock::hz, " Hz", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_uart0() { (void)Serial::isr(); }
extern "C" void isr_systick() { brio::Ticker::tick(); }
extern "C" void isr_i2c0() {
    if (host_live) {
        if (Host::isr()) {
            transfer_status = Host::status();
            transfer_done = true;
        }
    } else {
        brio::DwApbI2c<0>::interrupts_only(0);
        brio::DwApbI2c<0>::clear_all();
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool timer_ok = brio::Timer<0>::init(clock);
    // The unstick's half-bits are paced by the platform timer
    // (rp2350/delay.hpp), which the Hazard3 half's ticker starts and the
    // Cortex-M33 half's does not. Starting it here is idempotent and
    // makes the two halves the same program.
    const bool ruler_ok = brio::Mtime::start(clock);
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    bench.letter('a', "the block, wireless", ta_block);
    bench.letter('j', "the pads and the isolation latch, above GP31", tj_pads);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: arch=", arch_name(), " clk=", clock_ok ? "PLL150" : "FAILED",
                    " timer0=", timer_ok ? "1us" : "FAILED", " ruler=", ruler_ok ? "1us" : "FAILED",
                    " tick=", tick_ok ? "on" : "FAILED", brio::crlf);
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
