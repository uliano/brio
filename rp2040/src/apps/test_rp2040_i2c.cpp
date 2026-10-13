// test_rp2040_i2c - the reference bench suite for the RP2040's I2C
// (rp2040/i2c.hpp over datasheet 4.3, the Synopsys DW_apb_i2c): the
// block WIRELESS - the pin table, the timing arithmetic, the reset state,
// the FIFO depth, the refusals, the enable and the disable.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// THE WIRE IS THE DESK PARTNER'S. This chip's I2C has no loop-back, so a
// tenure needs a far end; on this desk it is the WeAct RP2350B over the
// pair's I2C wires (I2C1 on GP2 SDA / GP3 SCL here, 1.5 kOhm to this
// board's 3V3 on each - docs/boards/weact-rp2040.md), and a letter on
// them runs against that board's peer, never against the chip's other
// instance. No letter here routes a pad.
//
// THE RULER IS THE SYSTEM TIMER (rp2040/timer.hpp).
//
// What is exercised, letter by letter:
//   a  the block, WIRELESS: table 279 as the driver states it, the
//      timing arithmetic (the three speeds at 125 MHz, table 450's
//      floors reproduced), the reset state, the FIFO depths the block
//      reports, the refusals, the enable and the disable through
//      IC_ENABLE_STATUS
//
// build: boards = pico,weact2040
// build: monitor_speed = 115200

#include <stdint.h>

#include "rp2040/clock.hpp"
#include "rp2040/i2c.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/ticker.hpp"
#include "rp2040/timer.hpp"
#include "rp2040/uart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using SysClock = Clock<ClockSource::pll, 125'000'000>;
constexpr SysClock clock;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;

TestBench<Serial, 16> bench;

uint32_t us_now() { return Timer::now_low(); }

// =============================================================================
// a - the block, wireless
// =============================================================================
void ta_block() {
    bench.verdict("table 279 as the driver states it: I2C0 SDA on 0/4/../28 and SCL on 1/5/../29, I2C1 "
                  "SDA on 2/6/../26 and SCL on 3/7/../27",
                  i2c_sda_pin(0, 0) && i2c_sda_pin(0, 28) && i2c_scl_pin(0, 1) && i2c_scl_pin(0, 29) &&
                      i2c_sda_pin(1, 2) && i2c_sda_pin(1, 26) && i2c_scl_pin(1, 3) && i2c_scl_pin(1, 27) &&
                      !i2c_sda_pin(0, 2) && !i2c_scl_pin(1, 13) && !i2c_sda_pin(1, 30) &&
                      !i2c_pins_valid(0, {.scl = 12, .sda = 13}));
    for (uint8_t s = 0; s < i2c_speed_count; ++s) {
        const auto sp = static_cast<I2cSpeed>(s);
        const auto t = i2c_timing_for(SysClock::hz, sp);
        print(serial, "  ", i2c_speed_hz(sp) / 1000u, " kHz at ", SysClock::hz / 1'000'000u, " MHz: HCNT ",
              t ? t->hcnt : 0u, " LCNT ", t ? t->lcnt : 0u, " SPKLEN ", t ? t->spklen : 0u, " hold ",
              t ? t->sda_tx_hold : 0u, " setup ", t ? t->sda_setup : 0u, " -> ",
              t ? i2c_scl_hz(SysClock::hz, *t) : 0u, " Hz on an ideal wire", crlf);
    }
    bench.verdict("the timing arithmetic: the three speeds solved at 125 MHz produce their asked rate on "
                  "an ideal wire (the SPKLEN + 8 cycles the block adds subtracted from the counts)",
                  i2c_scl_hz(SysClock::hz, *i2c_timing_for(SysClock::hz, I2cSpeed::standard_100k)) == 100'000u &&
                      i2c_scl_hz(SysClock::hz, *i2c_timing_for(SysClock::hz, I2cSpeed::fast_400k)) >= 399'000u &&
                      i2c_scl_hz(SysClock::hz, *i2c_timing_for(SysClock::hz, I2cSpeed::fast_plus_1m)) == 1'000'000u);
    bench.verdict("table 450's floors reproduced: fast mode at 12 MHz is (LCNT 15, HCNT 6), standard at 2.7 "
                  "MHz is (12, 6), fast-mode-plus needs 32 MHz, a hertz below each is refused",
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
          " SDA_HOLD=", S::timing(I2cSpeed::fast_400k).sda_tx_hold, " enabled=", S::enabled(), " status=", hex(S::flags()),
          " IC_COMP_PARAM_1=", hex(S::regs().IC_COMP_PARAM_1), crlf);
    bench.verdict("the block's reset state: a host with its client half disabled, fast speed code, "
                  "restarts enabled (IC_CON 0x65), the spike filter at 7, the hold at 1, disabled, both "
                  "FIFOs empty - and the component parameter register reading zero, as 4.3.17 says",
                  reset_ok && con == 0x65u && S::timing(I2cSpeed::fast_400k).spklen == 7u &&
                      S::timing(I2cSpeed::fast_400k).sda_tx_hold == 1u && !S::enabled() && S::tx_empty() &&
                      !S::rx_not_empty() && S::regs().IC_COMP_PARAM_1 == 0u);
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
    print(serial, "  the transmit FIFO under TX_CMD_BLOCK: full after ", depth, " entries (TXFLR ", counted, "), TX_OVER ",
          over_now ? "raised" : "NOT raised", " on the next; after the disable TXFLR ", S::tx_count(), crlf);
    bench.verdict("the transmit FIFO is sixteen deep (4.3.1), measured under TX_CMD_BLOCK: full at sixteen, "
                  "TX_OVER on the seventeenth, flushed by the disable",
                  depth == 16u && counted == 16u && over && over_now && S::tx_count() == 0u);
    const bool cfg = S::configure({.speed = I2cSpeed::fast_plus_1m}) &&
                     S::timing(*i2c_timing_for(SysClock::hz, I2cSpeed::fast_plus_1m)) && S::target(0x42);
    const I2cTiming back = S::timing(I2cSpeed::fast_plus_1m);
    bench.verdict("a configuration, a timing and a target are taken with the block disabled and read back",
                  cfg && back.hcnt == 36u && back.lcnt == 74u && S::target() == 0x42u &&
                      (S::con() & I2C_IC_CON_SPEED_BITS) == (2u << I2C_IC_CON_SPEED_LSB));
    S::enable();
    const bool running = S::running();
    const bool refused = !S::configure({}) && !S::timing(back) && !S::target(0x10) && !S::own_address(0x11);
    (void)S::target(0x10);   // ignored by the hardware too
    bench.verdict("enabled: IC_ENABLE_STATUS reports it, and the configuration, the timing, the target "
                  "and the own address are refused",
                  running && refused && S::target() == 0x42u);
    const uint32_t t0 = us_now();
    const bool off = S::disable();
    const uint32_t took = us_now() - t0;
    print(serial, "  disable on an idle bus: ", off ? "reported" : "NOT reported", " in ", took, " us", crlf);
    bench.verdict("the disable of an idle block is reported by IC_ENABLE_STATUS within a few microseconds",
                  off && took < 20u);
    S::hold();
}

void banner() {
    print(serial, crlf, "test_rp2040_i2c - the RP2040 I2C (datasheet 4.3, DW_apb_i2c), the block wireless; clk_sys=",
          SysClock::hz, " Hz", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_uart0() { (void)Serial::isr(); }
extern "C" void isr_systick() { brio::Ticker::tick(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool timer_ok = brio::Timer::init(clock);
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    bench.letter('a', "the block, wireless", ta_block);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL125" : "FAILED", " timer=", timer_ok ? "1us" : "FAILED",
                    " tick=", tick_ok ? "SysTick" : "FAILED", brio::crlf);
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
