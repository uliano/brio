// test_ch32_clock - the reference bench suite for the CH32V00x's RCC
// chapter: the tree as it stands, the LSI, the HSI trim, the DynamicClock
// walking the HPRE ladder with the ticker and the console rebased at
// every rung, the system clock monitor, the MCO, the peripheral gates.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// NOTHING TO WIRE for `z`. The rate ladder's proof is the console
// itself: at every rung the USART is rebased to the new HCLK and prints
// a line, and a line that reads clean means HCLK landed within the
// UART's tolerance of what the clock type claims. A counter on PC4
// (the MCO) would put a number on it - that is a desk job for a scope.
//
// What is exercised, letter by letter:
//   a  the tree at boot: the PLL as the root, HPRE at 1, HSI on with
//      its factory calibration and centred trim, the LSI off, the gates
//      the console opened
//   b  the LSI: on, ready within a bounded wait (timed), off again
//   c  the HSI trim: one step up and down takes and the console still
//      reads, then the centre restored
//   d  the DynamicClock ladder: 48 -> 24 -> 16 -> 12 -> 6 -> 3 MHz and
//      back up, the ticker and the console rebased at each rung, ten
//      500-us waits still ten ticks, the wait states following the rate
//   e  the system clock monitor and the MCO
//   f  the peripheral gates: a clock opened, seen, reset, closed
//
// build: boards = v006k8
// build: monitor_speed = 115200

#include <stdint.h>

#include "ch32v00x/clock.hpp"
#include "ch32v00x/delay.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/pin.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/ticker.hpp"
#include "ch32v00x/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using P = brio::Ch32v00xPlatform<>;

namespace {

using namespace brio;

using Serial = Uart<1, P>;
constexpr Serial serial;
using Led = Pin<'C', 0>;

// The dynamic clock: the PLL's 48 MHz as the root, the ticker and the
// console as the users it rebases.
using Boot = Clock<ClockSource::pll, 48'000'000>;
using SysClock = DynamicClock<Boot, Ticker, Serial>;
constexpr SysClock clock;

TestBench<Serial> bench;

uint32_t cycles_now() { return stk()->CNT; }
uint32_t cycles_between(uint32_t from, uint32_t to) {
    const uint32_t period = stk()->CMP + 1u;
    return to >= from ? to - from : to + period - from;
}

void console_drain() {
    while (!Serial::tx_idle()) {
    }
    (void)delay_us(clock, 300);
}

// ---------------------------------------------------------------------------
// a - the tree at boot
// ---------------------------------------------------------------------------
void ta_tree() {
    print(serial, "  CTLR=", hex(rcc()->CTLR), " CFGR0=", hex(rcc()->CFGR0),
          " RSTSCKR=", hex(rcc()->RSTSCKR), crlf);
    print(serial, "  HSI cal=", Rcc::hsi_calibration(), " trim=", Rcc::hsi_trim(),
          " PLL on=", Rcc::pll_on(), " ready=", Rcc::pll_ready(),
          " SWS=", hex(Rcc::sysclk_source()), " HPRE=", Rcc::hpre_code(), crlf);
    bench.verdict("the PLL is the system clock", Rcc::sysclk_source() == rcc_sws_pll);
    bench.verdict("HPRE is 1 (the boot rate is the root's)", Rcc::hpre_code() == 0u);
    bench.verdict("the HSI is on and ready (the PLL's source)", Rcc::hsi_on() && Rcc::hsi_ready());
    bench.verdict("its trim sits at the centre (16)", Rcc::hsi_trim() == 16u);
    bench.verdict("the PLL is on and locked", Rcc::pll_on() && Rcc::pll_ready());
    bench.verdict("the LSI is off at boot", !Rcc::lsi_on());
    bench.verdict("two wait states stand for 48 MHz", (flash_ctl()->ACTLR & 0x3u) == 2u);
    bench.verdict("the console opened USART1's and PD's gates",
                  Rcc::clock(Bus::pb2, rcc_pb2_usart1 | rcc_pb2_gpiod));
    bench.verdict("SYSCLK has not failed", !Rcc::clock_failed());
}

// ---------------------------------------------------------------------------
// b - the LSI
// ---------------------------------------------------------------------------
void tb_lsi() {
    Rcc::lsi(true);
    const uint32_t c0 = cycles_now();
    uint32_t turns = 0;
    while (!Rcc::lsi_ready() && turns < 1'000'000u) {
        ++turns;
    }
    const uint32_t ready_cycles = cycles_between(c0, cycles_now());
    print(serial, "  LSI ready after ", turns, " polls (", ready_cycles, " cycles, ",
          ready_cycles / 48u, " us if within one tick)", crlf);
    bench.verdict("the LSI starts and reports ready", Rcc::lsi_on() && Rcc::lsi_ready());
    bench.verdict("within a bounded wait", turns < 1'000'000u);
    Rcc::lsi(false);
    (void)delay_us(clock, 100);   // three LSI cycles is 23 us
    bench.verdict("and stops: ready drops after LSION is cleared",
                  !Rcc::lsi_on() && !Rcc::lsi_ready());
}

// ---------------------------------------------------------------------------
// c - the HSI trim
// ---------------------------------------------------------------------------
void tc_trim() {
    const uint8_t centre = Rcc::hsi_trim();
    Rcc::hsi_trim(static_cast<uint8_t>(centre + 1u));
    const uint8_t up = Rcc::hsi_trim();
    console_drain();
    print(serial, "  trim ", centre, " -> ", up, ": this line was sent at the nudged rate", crlf);
    console_drain();
    Rcc::hsi_trim(static_cast<uint8_t>(centre - 1u));
    const uint8_t down = Rcc::hsi_trim();
    print(serial, "  trim ", up, " -> ", down, ": and this one at the other", crlf);
    console_drain();
    Rcc::hsi_trim(centre);
    bench.verdict("the trim takes a step up and a step down", up == centre + 1u && down == centre - 1u);
    bench.verdict("and is back at the centre", Rcc::hsi_trim() == centre);
}

// ---------------------------------------------------------------------------
// d - the ladder
// ---------------------------------------------------------------------------
bool rung(uint32_t hz) {
    console_drain();
    const bool switched = SysClock::set(hz);
    if (!switched) {
        return false;
    }
    // Ten 500-us waits are ten ticks whatever the rate: the ticker and
    // the delay table were both rebased.
    const uint32_t t0 = Ticker::ticks();
    bool served = true;
    for (uint8_t i = 0; i < 20u; ++i) {
        served = delay_us(clock, 500) && served;
    }
    const uint32_t took = Ticker::ticks() - t0;
    const uint32_t latency = flash_ctl()->ACTLR & 0x3u;
    print(serial, "  ", hz / 1000u, " kHz: HPRE code ", Rcc::hpre_code(), ", ",
          latency, " wait state(s), 20 x 500 us = ", took, " ticks, baud ",
          Serial::actual_baud(SysClock::hz()), crlf);
    return served && took >= 10u && took <= 12u &&
           latency == flash_latency_for(hz) && SysClock::hz() == hz &&
           SysClock::rate_hz(SysClock::rate_index()) == hz;
}

void td_ladder() {
    bool ok = true;
    for (const uint32_t hz : {24'000'000u, 16'000'000u, 12'000'000u, 6'000'000u, 3'000'000u,
                              6'000'000u, 12'000'000u, 24'000'000u, 48'000'000u}) {
        ok = rung(hz) && ok;
    }
    bench.verdict("every rung of the ladder switched, timed and printed clean", ok);
    bench.verdict("a rate no divider reaches is refused, nothing changed",
                  !SysClock::set(10'000'000u) && SysClock::hz() == 48'000'000u);
    bench.verdict("back at 48 MHz with two wait states",
                  Rcc::hpre_code() == 0u && (flash_ctl()->ACTLR & 0x3u) == 2u);
}

// ---------------------------------------------------------------------------
// e - the monitor and the MCO
// ---------------------------------------------------------------------------
void te_monitor() {
    Rcc::clear_clock_failed();
    Rcc::monitor(true);
    (void)delay_us(clock, 500);
    bench.verdict("the system clock monitor turns on", Rcc::monitor());
    bench.verdict("and sees no failure on a running clock", !Rcc::clock_failed());
    Rcc::monitor(false);
    bench.verdict("and off", !Rcc::monitor());

    Rcc::mco(rcc_mco_sysclk);
    bench.verdict("the MCO selects SYSCLK (PC4, a scope's job to see)", Rcc::mco() == rcc_mco_sysclk);
    Rcc::mco(rcc_mco_hsi);
    bench.verdict("and the HSI", Rcc::mco() == rcc_mco_hsi);
    Rcc::mco(0);
    bench.verdict("and off again", Rcc::mco() == 0u);
}

// ---------------------------------------------------------------------------
// f - the gates
// ---------------------------------------------------------------------------
void tf_gates() {
    constexpr uint32_t tim2 = 1UL << 0;   // RCC_PB1PCENR.TIM2EN
    Rcc::clock(Bus::pb1, tim2, true);
    bench.verdict("TIM2's clock opens on PB1", Rcc::clock(Bus::pb1, tim2));
    Rcc::reset(Bus::pb1, tim2);
    bench.verdict("its reset pulses and clears", (rcc()->PB1PRSTR & tim2) == 0u);
    Rcc::clock(Bus::pb1, tim2, false);
    bench.verdict("and the gate closes", !Rcc::clock(Bus::pb1, tim2));
    bench.verdict("the SRAM's HB gate is open (the reset value)",
                  Rcc::clock(Bus::hb, 1UL << 2));
}

void banner() {
    print(serial, crlf, "test_ch32_clock - CH32V006K8 (PLL 48 MHz root, DynamicClock over HPRE)", crlf);
    bench.menu();
}

} // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the tree at boot", ta_tree);
    bench.letter('b', "the LSI", tb_lsi);
    bench.letter('c', "the HSI trim", tc_trim);
    bench.letter('d', "the DynamicClock ladder", td_ladder);
    bench.letter('e', "the system clock monitor and the MCO", te_monitor);
    bench.letter('f', "the peripheral gates", tf_gates);

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
