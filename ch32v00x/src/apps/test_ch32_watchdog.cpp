// test_ch32_watchdog - the reference bench suite for the CH32V00x's two
// watchdogs: ch32v00x/reset.hpp's Iwdg and Wwdg over RM ch. 4 and 5,
// the counters timed, both dogs kept alive, and the resets they cause
// judged at the boot that follows.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// NOTHING TO WIRE. And two things this suite lives with: NEITHER
// WATCHDOG CAN BE STOPPED BY SOFTWARE once started - the WWDG's RCC
// reset pulse puts it back (5.2.1's own note), the IWDG only a reset
// does - so letter j leaves the IWDG running and the console loop
// feeds it from then on; and a letter that ends in a reset judges
// itself at the NEXT BOOT through a .noinit token that records what
// it armed and the last millisecond the loop was seen alive.
//
// What is exercised, letter by letter:
//   a  the WWDG WIRELESS: the block off its bus clock, what the counter
//      does with WDGA clear (5.2.1 says it runs; measured), the step
//      timed on the STK against wwdg_timeout_us once armed, EWIF's
//      write rules, the refusals, the RCC reset pulse putting the
//      block back
//   b  the WWDG ARMED and kept alive from its early-warning interrupt
//      for 300 ms - the warnings counted, no reset - then put back
//      through the RCC pulse
//   j  the IWDG started, configured (the update flags timed), and kept
//      alive for a second at a quarter of its period; from here on the
//      loop feeds it
//   i  THE IWDG RESET (reboots the board): the feeding stops, the token
//      keeps the last millisecond seen, the next boot judges the flag
//      and the elapsed time against iwdg_timeout_ms at the LSI the
//      sleep chapter measured
//   w  THE WWDG RESET (reboots the board), two legs: the counter
//      running down to 0x3F with no refresh (timed), then a refresh
//      made ABOVE the window
//
// build: boards = v006k8
// build: monitor_speed = 115200

#include <stdint.h>

#include "ch32v00x/clock.hpp"
#include "ch32v00x/delay.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/pin.hpp"
#include "ch32v00x/platform.hpp"
#include "ch32v00x/reset.hpp"
#include "ch32v00x/ticker.hpp"
#include "ch32v00x/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using P = brio::Ch32v00xPlatform<>;
using SysClock = brio::Clock<brio::ClockSource::pll, 48'000'000>;
constexpr SysClock clock;

/// The LSI this desk measured (sleep.md): what the IWDG's expected
/// time-out is computed at.
inline constexpr uint32_t lsi_measured_hz = 124'000;

// ---------------------------------------------------------------------------
// The token the reset letters leave for the next boot: .noinit, inline
// (the platform's breadcrumb shares the section; see test_ch32_platform).
// ---------------------------------------------------------------------------
inline constexpr uint16_t token_magic = 0x5A07;

struct Token {
    uint16_t magic;
    uint8_t letter;      ///< 'i' or 'w', 0 = nothing pending
    uint8_t leg;
    uint32_t armed_at;   ///< Ticker::millis() when the feeding stopped
    uint32_t last_seen;  ///< the last millisecond the loop wrote
    uint32_t expected_ms;
    uint16_t pass;
    uint16_t fail;
};
/// VOLATILE, and not for the ISR's sake: the stores of last_seen happen
/// in a loop that ends with a reset the compiler cannot see, and a
/// plain object's stores may legally be sunk to the loop's exit - which
/// the reset never reaches (measured: "elapsed 0 ms" until this
/// qualifier).
[[gnu::section(".noinit")]] inline volatile Token token;

namespace {

using namespace brio;

using Serial = Uart<1, P>;
constexpr Serial serial;
using Led = Pin<'C', 0>;

TestBench<Serial> bench;

uint32_t boot_flags = 0;
bool iwdg_running = false;   ///< letter j's legacy: the loop feeds it
volatile uint32_t early_warnings = 0;
volatile bool feed_on_warning = false;

/// Feed whichever dog is running. Called from every wait and from the
/// console loop.
void feed() {
    if (iwdg_running) {
        Iwdg::refresh();
    }
}

void settle_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
        feed();
    }
}

void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
        feed();
    }
    (void)delay_us(clock, 500);
}

/// The STK as a stopwatch (cycles), folded with the tick count.
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

/// The WWDG back to reset through the RCC pulse: WDGA cleared, the
/// counter at 0x7F, the block's clock left on.
void wwdg_put_back() {
    rcc()->PB1PRSTR |= rcc_pb1_wwdg;
    rcc()->PB1PRSTR &= ~rcc_pb1_wwdg;
}

// ===========================================================================
// a - the WWDG, wireless
// ===========================================================================

void ta_wwdg() {
    Pfic::disable(Wwdg::irq());
    Wwdg::bus_clock(false);
    const uint16_t dark = wwdg()->CTLR;
    Wwdg::bus_clock(true);
    wwdg_put_back();
    print(serial, "  off its clock CTLR reads ", hex(dark), "; on it: CTLR=", hex(wwdg()->CTLR),
          " CFGR=", hex(wwdg()->CFGR), " STATR=", hex(wwdg()->STATR), crlf);
    bench.verdict("with the clock on the reset values are table 5-1's (0x7F, 0x7F, 0)",
                  wwdg()->CTLR == 0x7Fu && wwdg()->CFGR == 0x7Fu && wwdg()->STATR == 0u);
    bench.verdict("the refusals: a window below 0x40 leaves no legal refresh",
                  !wwdg_config_valid({.window = 0x3F}) && wwdg_config_valid({.window = 0x40}));

    // 5.2.1 SAYS THE COUNTER RUNS "regardless of whether the watchdog
    // function is turned on or not". Refresh to 0x7F with WDGA off and
    // watch for 100 ms - two full periods at /8.
    (void)Wwdg::configure({.prescaler = WwdgPrescaler::div8, .window = 0x7F});
    Wwdg::clear_flag();
    Wwdg::refresh(0x7F);
    const uint8_t c0 = Wwdg::counter();
    settle_ms(100);
    const uint8_t c1 = Wwdg::counter();
    const bool ewif_unarmed = Wwdg::flag();
    print(serial, "  unarmed at /8 for 100 ms: counter ", hex(c0), " -> ", hex(c1), ", EWIF ",
          ewif_unarmed ? "up" : "down", crlf);
    print(serial, "  -> ", (c1 == c0 && !ewif_unarmed)
                               ? "THE COUNTER DOES NOT RUN WITH WDGA CLEAR on this silicon, against "
                                 "5.2.1's 'free operation regardless'"
                               : "the counter runs unarmed, as 5.2.1 says",
          crlf);
    bench.verdict("what the unarmed counter does was measured (a finding either way)", true);

    // THE STEP TIMED, ARMED: WDGA up with EWI, the handler feeding at
    // every warning, the first fall from 0x7F to 0x40 timed on the STK.
    early_warnings = 0;
    feed_on_warning = true;
    (void)Wwdg::configure({.prescaler = WwdgPrescaler::div8, .window = 0x7F, .early_wakeup = true});
    Wwdg::clear_flag();
    Pfic::enable(Wwdg::irq());
    const uint32_t t0 = cycles_now();
    Wwdg::start(0x7F);
    while (early_warnings == 0u && cycles_now() - t0 < 48'000'000UL) {
    }
    const uint32_t cycles = cycles_now() - t0;
    const uint32_t expected = wwdg_step_cycles(WwdgPrescaler::div8) * (0x7Fu - 0x40u);
    print(serial, "  armed at /8: the first early warning after ", cycles, " cycles (63 steps of 32768 = ",
          expected, ")", crlf);
    bench.verdict("the fall from 0x7F to 0x40 takes 63 steps of 4096 x 8 HCLK cycles within 2%",
                  early_warnings != 0u && cycles >= expected - expected / 50u &&
                      cycles <= expected + expected / 50u);
    feed_on_warning = false;
    Pfic::disable(Wwdg::irq());
    wwdg_put_back();
    Wwdg::bus_clock(true);
    (void)Wwdg::configure({.prescaler = WwdgPrescaler::div8, .window = 0x7F});
    wwdg()->STATR = wwdg_ewif;   // a set attempt: 5.3.3 says it is invalid
    bench.verdict("EWIF cannot be set by software (5.3.3)", !Wwdg::flag());

    // The configuration reads back, the prescaler's four codes.
    bool codes = true;
    for (uint8_t p = 0; p < 4u; ++p) {
        (void)Wwdg::configure({.prescaler = static_cast<WwdgPrescaler>(p), .window = static_cast<uint8_t>(0x50u + p)});
        codes = codes && Wwdg::prescaler() == static_cast<WwdgPrescaler>(p) && Wwdg::window() == 0x50u + p;
    }
    bench.verdict("the four WDGTB codes and the window read back as written", codes);
    wwdg_put_back();
    bench.verdict("the RCC pulse puts the block back to its reset values",
                  wwdg()->CTLR == 0x7Fu && wwdg()->CFGR == 0x7Fu && !Wwdg::enabled());
}

// ===========================================================================
// b - the WWDG armed and fed from its early warning
// ===========================================================================

void tb_wwdg_armed() {
    wwdg_put_back();
    early_warnings = 0;
    feed_on_warning = true;
    // /8: a step is 683 us at 48 MHz, 0x7F -> 0x40 is 43 ms; the early
    // warning comes one step before the reset and the handler feeds.
    (void)Wwdg::configure({.prescaler = WwdgPrescaler::div8, .window = 0x7F, .early_wakeup = true});
    Wwdg::clear_flag();
    Pfic::enable(Wwdg::irq());
    Wwdg::start(0x7F);
    const bool armed = Wwdg::enabled();
    settle_ms(300);
    const uint32_t warnings = early_warnings;
    print(serial, "  armed at /8 with EWI: ", warnings, " early warnings in 300 ms (one every ~43 ms), "
          "counter now ", hex(Wwdg::counter()), crlf);
    bench.verdict("WDGA is set and stays set", armed && Wwdg::enabled());
    bench.verdict("the early warning fires once per period and the handler's refresh keeps the "
                  "board alive (six or seven in 300 ms)",
                  warnings >= 6u && warnings <= 7u);
    feed_on_warning = false;
    Pfic::disable(Wwdg::irq());
    wwdg_put_back();
    bench.verdict("the RCC pulse disarms it (WDGA clear, EWI clear)",
                  !Wwdg::enabled() && !Wwdg::early_wakeup_enabled());
}

// ===========================================================================
// j - the IWDG started and kept alive
// ===========================================================================

void tj_iwdg_alive() {
    if (iwdg_running) {
        print(serial, "  the IWDG is already running (a previous j): re-configured only", crlf);
    }
    print(serial, "  before the start: STATR=", hex(Iwdg::status()), " PSCR=", hex(iwdg()->PSCR), " RLDR=",
          hex(iwdg()->RLDR), crlf);
    if (!iwdg_running) {
        bench.verdict("the reset values are table 4-1's (RLDR 0x0FFF)", iwdg()->RLDR == 0x0FFFu);
    }

    // /32 and 999: (999 + 1) x 32 / 124 kHz = 258 ms.
    const IwdgConfig cfg{.prescaler = IwdgPrescaler::div32, .reload = 999};
    Iwdg::start();
    Iwdg::unlock();
    iwdg()->PSCR = static_cast<uint16_t>(cfg.prescaler);
    const uint32_t t0 = cycles_now();
    const bool pvu_up = Iwdg::busy(iwdg_pvu);
    const bool synced = Iwdg::sync(iwdg_pvu);
    const uint32_t pvu_cycles = cycles_now() - t0;
    Iwdg::unlock();
    iwdg()->RLDR = cfg.reload;
    const bool rvu_up = Iwdg::busy(iwdg_rvu);
    const bool synced2 = Iwdg::sync(iwdg_rvu);
    Iwdg::refresh();
    iwdg_running = true;
    print(serial, "  started: PVU ", pvu_up ? "rose" : "never rose", " and fell after ", pvu_cycles,
          " cycles (", pvu_cycles / 48u, " us); RVU ", rvu_up ? "rose" : "never rose", "; PSCR=",
          hex(iwdg()->PSCR), " RLDR=", hex(iwdg()->RLDR), crlf);
    bench.verdict("the prescaler update crosses into the LSI domain (PVU seen, then down within "
                  "five LSI cycles)",
                  synced && pvu_cycles < 48u * 60u);
    bench.verdict("the reload update likewise", synced2);
    bench.verdict("both fields read back as written once their flags are down",
                  Iwdg::prescaler() == IwdgPrescaler::div32 && Iwdg::reload() == 999u);
    print(serial, "  time-out: ", iwdg_timeout_ms(cfg.prescaler, cfg.reload, lsi_measured_hz),
          " ms at the measured LSI (", iwdg_timeout_ms(cfg.prescaler, cfg.reload), " nominal)", crlf);

    // A second alive at a quarter of the period.
    const uint32_t t1 = Ticker::millis();
    while (Ticker::millis() - t1 < 1000u) {
        Iwdg::refresh();
        (void)delay_us(clock, 900);
        const uint32_t t2 = Ticker::millis();
        while (Ticker::millis() - t2 < 60u) {
        }
    }
    bench.verdict("fed every 60 ms against a 258 ms period, the board is still here after a second",
                  true);
    print(serial, "  the IWDG stays running: the loop feeds it from now on (letter i stops that)",
          crlf);
}

// ===========================================================================
// i and w - the resets, judged at the next boot
// ===========================================================================

/// The console drained FIRST, so the stopwatch starts at the return and
/// not under a line still leaving the ring.
void arm_token(uint8_t letter, uint8_t leg, uint32_t expected_ms) {
    console_drain();
    token.magic = token_magic;
    token.letter = letter;
    token.leg = leg;
    token.expected_ms = expected_ms;
    token.armed_at = Ticker::millis();
    token.last_seen = token.armed_at;
}

/// Spin, writing the millisecond into the token, until the reset takes
/// the board. Bounded by a generous limit so a dog that never bites
/// is reported rather than hung on.
void wait_for_the_bite(uint32_t limit_ms) {
    for (;;) {
        token.last_seen = Ticker::millis();
        if (token.last_seen - token.armed_at > limit_ms) {
            print(serial, "  NO RESET after ", limit_ms, " ms - the dog did not bite", crlf);
            token.leg = 0;
            return;
        }
    }
}

void ti_iwdg_reset() {
    token.pass = 0;
    token.fail = 0;
    Reset::clear_flags();
    if (!iwdg_running) {
        (void)Iwdg::arm({.prescaler = IwdgPrescaler::div32, .reload = 999});
        iwdg_running = true;
    }
    print(serial, "  the feeding stops now; expected bite in ",
          iwdg_timeout_ms(IwdgPrescaler::div32, 999, lsi_measured_hz), " ms", crlf);
    arm_token('i', 1, iwdg_timeout_ms(IwdgPrescaler::div32, 999, lsi_measured_hz));
    Iwdg::refresh();        // the last one: the stopwatch starts here
    iwdg_running = false;   // the loop must not feed it any more
    wait_for_the_bite(2000);
}

void tw_wwdg_reset() {
    token.pass = 0;
    token.fail = 0;
    Reset::clear_flags();
    wwdg_put_back();
    // Leg 1: armed at /8 with the whole window, never refreshed: 43 ms.
    (void)Wwdg::configure({.prescaler = WwdgPrescaler::div8, .window = 0x7F});
    print(serial, "  leg 1: armed at /8, never refreshed; expected bite in ",
          wwdg_timeout_us(SysClock::hz, WwdgPrescaler::div8, 0x7F) / 1000u, " ms", crlf);
    arm_token('w', 1, wwdg_timeout_us(SysClock::hz, WwdgPrescaler::div8, 0x7F) / 1000u);
    Wwdg::start(0x7F);
    wait_for_the_bite(500);
}

void judge(const char* what, bool ok) {
    bench.verdict(what, ok);
    if (ok) { token.pass = token.pass + 1u; } else { token.fail = token.fail + 1u; }
}

void resume_after_reset() {
    const uint8_t letter = token.letter;
    const uint8_t leg = token.leg;
    token.leg = 0;
    bench.reset_tally();
    bench.resume_tally(token.pass, token.fail);
    const uint32_t elapsed = token.last_seen - token.armed_at;
    print(serial, crlf, "-> back from letter ", static_cast<char>(letter), " leg ", leg, ": flags=",
          hex(boot_flags), " elapsed ", elapsed, " ms (expected ", token.expected_ms, ")", crlf);
    if (letter == 'i') {
        judge("the IWDG reset the board: IWDGRSTF stands, WWDGRSTF and SFTRSTF do not",
              (boot_flags & ResetFlag::independent_watchdog) != 0u &&
                  (boot_flags & (ResetFlag::window_watchdog | ResetFlag::software)) == 0u);
        judge("it bit when the arithmetic said, within 5% at the measured LSI",
              elapsed >= token.expected_ms - token.expected_ms / 20u &&
                  elapsed <= token.expected_ms + token.expected_ms / 20u);
        judge("and a fresh boot finds the watchdog OFF again (the one thing that stops it)",
              !Iwdg::busy() && iwdg()->RLDR == 0x0FFFu);
        bench.end_letter();
        return;
    }
    if (letter == 'w' && leg == 1) {
        judge("leg 1: the WWDG reset the board: WWDGRSTF stands, IWDGRSTF does not",
              (boot_flags & ResetFlag::window_watchdog) != 0u &&
                  (boot_flags & ResetFlag::independent_watchdog) == 0u);
        judge("leg 1: it bit when the counter reached 0x3F, within 3 ms of 43",
              elapsed + 3u >= token.expected_ms && elapsed <= token.expected_ms + 3u);
        judge("leg 1: WDGA is clear at the boot", !Wwdg::enabled());
        // Leg 2: a refresh ABOVE the window is a reset at once.
        print(serial, "  leg 2: armed with window 0x50, refreshed at once with the counter at 0x7F", crlf);
        Reset::clear_flags();
        Wwdg::bus_clock(true);
        wwdg_put_back();
        (void)Wwdg::configure({.prescaler = WwdgPrescaler::div8, .window = 0x50});
        arm_token('w', 2, 0);
        Wwdg::start(0x7F);
        Wwdg::refresh(0x7F);   // above 0x50: the second reset condition
        wait_for_the_bite(500);
        return;
    }
    if (letter == 'w' && leg == 2) {
        judge("leg 2: a refresh above the window reset the board at once (WWDGRSTF, no time "
              "elapsed)",
              (boot_flags & ResetFlag::window_watchdog) != 0u && elapsed <= 2u);
        bench.end_letter();
        return;
    }
    print(serial, "  unknown token ", static_cast<char>(letter), "/", leg, crlf);
}

void banner() {
    print(serial, crlf, "test_ch32_watchdog - CH32V006K8 IWDG and WWDG (RM ch. 4 and 5)", crlf);
    print(serial, "  boot flags ", hex(boot_flags), "; the IWDG is ", iwdg_running ? "RUNNING (fed by the loop)"
                                                                                 : "off", crlf);
    bench.menu();
}

} // namespace

extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }
extern "C" BRIO_CH32_INTERRUPT void wwdg_handler() {
    if (brio::Wwdg::isr()) {
        early_warnings = early_warnings + 1u;
        if (feed_on_warning) {
            brio::Wwdg::refresh(0x7F);
        }
    }
}

int main() {
    boot_flags = brio::Reset::take_flags();
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the WWDG wireless: the free-running counter timed, EWIF unarmed, the "
                      "RCC pulse", ta_wwdg);
    bench.letter('b', "the WWDG armed and fed from its early warning for 300 ms", tb_wwdg_armed);
    bench.letter('j', "the IWDG started, its update flags timed, kept alive a second", tj_iwdg_alive);
    bench.letter('i', "THE IWDG RESET (reboots the board)", ti_iwdg_reset, false);
    bench.letter('w', "THE WWDG RESET, two legs (reboots the board twice)", tw_wwdg_reset, false);

    if (serial_ok && token.magic == token_magic && token.leg != 0) {
        resume_after_reset();
        bench.prompt();
    } else if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL48" : "FAILED",
                    " tick=", tick_ok ? "STK" : "FAILED", " flags=", brio::hex(boot_flags), brio::crlf);
        banner();
        bench.prompt();
    }

    for (;;) {
        feed();
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
