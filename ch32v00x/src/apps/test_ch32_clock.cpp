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
//   h  THE CRYSTAL AS SYSCLK ITSELF, in its own image (on the CH32V006K8
//      the second group image, booted on the HSE at 24 MHz with no PLL):
//      the tree as the boot left it - SWS the HSE, the PLL never on, the
//      wait states for 24 MHz - and the console alive at 115200 on it;
//      an image booted on the PLL says so and claims nothing
//   k  (by name) a bracket of 30 000 ticks between two console lines, for
//      a host that time-stamps them: the root's rate against the host's
//      clock
//
// build: boards = v006k8,v003f4
// build: groups.v006k8 = abcgdefk,hk
// build: monitor_speed = 115200

#include <stdint.h>

#include <type_traits>

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

// The dynamic clock: the PLL's 48 MHz as the root, the ticker and the
// console as the users it rebases. The root's source is the 24 MHz
// crystal both bench boards carry on PA1/PA2 (docs/boards/), so every
// letter runs on the HSE through the PLL and the console alive at
// 115200 is the crystal clock's first proof; the other apps of the
// project stay on the HSI, which every board has. AN IMAGE THAT CARRIES
// LETTER h AND NOT a boots on the crystal ITSELF, 24 MHz with no PLL -
// the root letter h is about; every other image on the PLL.
constexpr bool hse_image = brio::test_letters != nullptr && brio::selection_carries(brio::test_letters, 'h') &&
                           !brio::selection_carries(brio::test_letters, 'a');
using PllBoot = Clock<ClockSource::pll, 48'000'000, 24'000'000>;
using HseBoot = Clock<ClockSource::crystal, 24'000'000, 24'000'000>;
using Boot = std::conditional_t<hse_image, HseBoot, PllBoot>;
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
    bench.verdict("the HSI is on and ready", Rcc::hsi_on() && Rcc::hsi_ready());
    if constexpr (Boot::uses_hse) {
        print(serial, "  HSE on=", Rcc::hse_on(), " ready=", Rcc::hse_ready(), " PLLSRC=", Rcc::pll_from_hse(),
              " PCFR1=", hex(afio_pcfr1()), crlf);
        bench.verdict("the HSE is on and ready, and the PLL's source (PLLSRC)",
                      Rcc::hse_on() && Rcc::hse_ready() && Rcc::pll_from_hse());
        bench.verdict("the crystal pads are the oscillator's (AFIO's PA1/PA2 bit, each part's sense)",
                      !Afio::pa1_pa2_are_gpio());
    } else {
        bench.verdict("the PLL's source is the HSI (PLLSRC clear)", !Rcc::pll_from_hse());
    }
    bench.verdict("its trim sits at the centre (16)", Rcc::hsi_trim() == 16u);
    bench.verdict("the PLL is on and locked", Rcc::pll_on() && Rcc::pll_ready());
    bench.verdict("the LSI is off at boot", !Rcc::lsi_on());
    bench.verdict("the part's wait states stand for 48 MHz", (flash_ctl()->ACTLR & 0x3u) == flash_latency_for(48'000'000));
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
    print(serial, "  trim ", centre, " -> ", up, ": this line was sent at the same rate - the root is the crystal", crlf);
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
    bench.verdict("back at 48 MHz with the part's wait states",
                  Rcc::hpre_code() == 0u && (flash_ctl()->ACTLR & 0x3u) == flash_latency_for(48'000'000));
}

// ---------------------------------------------------------------------------
// e - the monitor and the MCO
// ---------------------------------------------------------------------------
void te_monitor() {
    if constexpr (Rcc::has_monitor) {
        Rcc::clear_clock_failed();
        Rcc::monitor(true);
        (void)delay_us(clock, 500);
        bench.verdict("the system clock monitor turns on", Rcc::monitor());
        bench.verdict("and sees no failure on a running clock", !Rcc::clock_failed());
        Rcc::monitor(false);
        bench.verdict("and off", !Rcc::monitor());
    } else {
        Rcc::monitor(true);
        bench.verdict("this part has no system clock monitor: the verb writes nothing and reads false",
                      !Rcc::monitor() && !Rcc::clock_failed());
    }

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

// ---------------------------------------------------------------------------
// g - the HSE
// ---------------------------------------------------------------------------
void tg_hse() {
    // The crystal feeds the PLL that runs this suite (letter a reads the
    // tree); what is left to measure is the clock security system on it.
    Rcc::clear_css_failed();
    Rcc::css(true);
    (void)delay_us(clock, 500);
    const bool armed = Rcc::css();
    const bool quiet = !Rcc::css_failed();
    Rcc::css(false);
    print(serial, "  CSS armed=", armed, " CSSF=", !quiet, " (a crystal that fails would switch SYSCLK to the HSI, turn the "
          "HSE and the PLL off and raise the NMI)", crlf);
    bench.verdict("the clock security system arms on a ready HSE and reports no failure", armed && quiet);
    bench.verdict("and disarms", !Rcc::css());
    // The HSE's part is the register's: PLLSRC, and the crystal pads
    // handed to the oscillator.
    bench.verdict("the PLL's source is the HSE and the crystal pads are the oscillator's",
                  Rcc::pll_from_hse() && !Afio::pa1_pa2_are_gpio());
}

// ---------------------------------------------------------------------------
// h - the crystal as SYSCLK itself
// ---------------------------------------------------------------------------
void th_hse_sysclk() {
    if constexpr (!hse_image) {
        print(serial, "  SKIPPED, no verdict claimed: this image boots on the PLL; the crystal as SYSCLK is the "
                      "image that carries letter h without a", crlf);
    } else {
        print(serial, "  CTLR=", hex(rcc()->CTLR), " CFGR0=", hex(rcc()->CFGR0), " ACTLR=", hex(flash_ctl()->ACTLR),
              crlf);
        bench.verdict("SYSCLK is the HSE itself (SWS), HPRE 1: HCLK is the crystal's 24 MHz",
                      Rcc::sysclk_source() == rcc_sws_hse && Rcc::hpre_code() == 0u);
        bench.verdict("the HSE is on and ready, its pads the oscillator's, no bypass",
                      Rcc::hse_on() && Rcc::hse_ready() && !Rcc::hse_bypass() && !Afio::pa1_pa2_are_gpio());
        bench.verdict("the PLL was never turned on", !Rcc::pll_on());
        bench.verdict("the wait states stand for 24 MHz", (flash_ctl()->ACTLR & 0x3u) == flash_latency_for(24'000'000));
        bench.verdict("SYSCLK has not failed", !Rcc::clock_failed());
    }
}

// ---------------------------------------------------------------------------
// k - a bracket of ticks for the host
// ---------------------------------------------------------------------------
/// 30 000 ticks - 30 s on the tick the root clocks - between two console
/// lines: a host that time-stamps the lines as they arrive has the root's
/// rate against its own clock (NTP-disciplined), to the arrival jitter of
/// the probe's bridge over 30 s.
void tk_bracket() {
    constexpr uint32_t span = 30'000;
    print(serial, "  bracket ", span, " ticks at ", Boot::hz / 1'000'000u, " MHz: the host's time between this "
                  "line and the next", crlf);
    console_drain();
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < span) {
    }
    print(serial, "  bracket done", crlf);
    console_drain();
    bench.verdict("the bracket ran to its count", Ticker::ticks() - t0 >= span);
}

void banner() {
    print(serial, crlf, "test_ch32_clock - ", device::part_name,
          hse_image ? " (the 24 MHz crystal as SYSCLK, no PLL, DynamicClock over HPRE)"
                    : " (PLL 48 MHz root from the 24 MHz crystal, DynamicClock over HPRE)",
          crlf);
    bench.menu();
}

} // namespace

BRIO_CH32_VECTOR(systick_handler) { brio::Ticker::tick(); }
BRIO_CH32_VECTOR(usart1_handler) { (void)Serial::isr(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    bench.letter('a', "the tree at boot", ta_tree);
    bench.letter('b', "the LSI", tb_lsi);
    bench.letter('c', "the HSI trim", tc_trim);
    bench.letter('g', "the HSE: the crystal as the PLL's source, the clock security system on it", tg_hse);
    bench.letter('d', "the DynamicClock ladder", td_ladder);
    bench.letter('e', "the system clock monitor and the MCO", te_monitor);
    bench.letter('f', "the peripheral gates", tf_gates);
    bench.letter('h', "the crystal as SYSCLK itself, no PLL (its own image)", th_hse_sysclk);
    bench.letter('k', "a bracket of 30 000 ticks for a host to time (by name)", tk_bracket, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? (hse_image ? "HSE24" : "PLL48") : "FAILED",
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
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        brio::print(serial, "  stack: ", brio::stack_untouched(), " B never touched", brio::crlf);
        bench.prompt();
    }
}
