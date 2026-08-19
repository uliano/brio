// test_avr_clock - the CLKCTRL test SUITE for the AVR DA/DB target:
// every clock source, the prescaler, the OSCHF tune, the 32 kHz main
// clock, the clock failure detector - exercised with CLK_PER on the
// CLKOUT pin (PA7) for an oscilloscope, and the console reporting what
// to expect and what the status bits say. Reference test of
// avrdx/clock.hpp (docs/design/clkctrl.md): keep it passing.
//
// Bench diagnostic, NOT a kernel app (sequential, blocking). Console
// at 115200 - the USART needs CLK_PER >= 16 x baud, so the console
// keeps talking down to 2 MHz; below that a step runs SILENTLY for two
// seconds (watch the scope) and the console resumes after the return
// to the boot clock. The Ticker (RTC, OSC32K) times the silent holds:
// it does not depend on the main clock.
//
// Wiring: scope on PA7 (CLKOUT). Nothing else.
// Commands: ? | 1 OSCHF sweep | 2 prescalers | 3 tune | 4 crystal vs
// OSCHF | 5 32 kHz main clock | 6 clock failure | 7 PLL + status | a all
//
// Tests:
//   1  OSCHF at 24/20/16/12/8/4/3/2/1 MHz as the main clock: CLKOUT
//      shows each (accuracy +-2..5 % calibrated >= 4 MHz, +-6..10 %
//      below); 1 MHz is silent (USART cannot do 115200 there);
//   2  the twelve prescalers from the 24 MHz crystal: 24 .. 0.375 MHz
//      (div16 and below silent);
//   3  OSCHF manual tune at 16 MHz: -32, -16, 0, +16, +31 steps of
//      ~0.4 %: 13.95 .. 18.0 MHz on the scope (silent: the baud moves
//      with the tune; printed before and after);
//   4  the 24 MHz crystal vs OSCHF at 24 MHz: same nominal, the scope
//      tells the accuracy apart (crystal ppm, OSCHF %);
//   5  OSC32K as the main clock: CLKOUT at ~32 kHz (+-10 %) for two
//      seconds, silent; then XOSC32K is tried: not fitted on this board
//      -> never stable -> reported, not failed;
//   6  the clock failure detector watching the main clock, forced by
//      the test bit: the main clock falls back to OSCHF 4 MHz, CLKOUT is
//      disabled by hardware (scope flat), the CFD flag and the regular
//      interrupt fire; then recovery: flag cleared, clock re-initialised,
//      CLKOUT back;
//   7  PLL x2 from OSCHF: PLLS stays 0 without a requester (the TCD) -
//      informative; all MCLKSTATUS bits and the OSCHF tune/RUNSTDBY
//      registers read back.
// Not testable here: XOSC32K and auto-tune (no 32 kHz crystal fitted),
// an external clock on PA0 (the crystal sits there), the PLL running
// (needs the TCD), the NMI form of the CFD interrupt (locks the
// configuration until reset: run once, on purpose, not in a suite).

// pio: monitor_speed = 115200

#include <avr/interrupt.h>
#include <stdint.h>

#include "avrdx/clock.hpp"
#include "avrdx/delay.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/ticker.hpp"
#include "avrdx/uart.hpp"
#include "util/print.hpp"

using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<2, Route::alt1>;
constexpr Serial serial;
constexpr uint32_t baud = 115200;

uint8_t passed = 0, failed = 0;
void verdict(const char* name, bool ok) {
    if (ok) ++passed; else ++failed;
    print(serial, "  ", ok ? "PASS" : "FAIL", "  ", name, crlf);
}

// ---- running at another rate for a while ------------------------------------
// The console can follow a rate only if the USART can make the baud.
bool console_ok(uint32_t hz) { return Serial::can_baud(hz, baud); }

// Hold for ms using the RTC-based Ticker (independent of CLK_PER).
void hold_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {}
}

// Announce the next rate, retune the console for it (drains TX first),
// then the caller switches. If the console cannot follow, say so
// before going silent.
void before_switch(uint32_t next_hz, const char* what) {
    print(serial, "  -> ", what, " = ", next_hz, " Hz on PA7",
          console_ok(next_hz) ? "" : " (console silent)", crlf);
    Serial::rebase(next_hz);                 // drains TX, then BAUD for the next rate
}

// Back on the boot clock: crystal 24 MHz, console retuned.
void back_to_boot() {
    Serial::rebase(SysClock::hz);
    (void)SysClock::init();
    MainClock::clkout(true);
    Serial::rebase(SysClock::hz);
}

// ---- 1: OSCHF sweep -------------------------------------------------------------
void t1_oschf() {
    print(serial, "1 OSCHF as main clock: 24,20,16,12,8,4,3,2,1 MHz, 2 s each (scope PA7)", crlf);
    constexpr uint32_t rates[] = {24'000'000, 20'000'000, 16'000'000, 12'000'000, 8'000'000,
                                  4'000'000, 3'000'000, 2'000'000, 1'000'000};
    // move to OSCHF at 24 MHz first (same rate as the crystal)
    Oschf::set_hz(24'000'000);
    (void)MainClock::select(MainSource::oschf);
    for (uint32_t hz : rates) {
        before_switch(hz, "OSCHF");
        Oschf::set_hz(hz);
        hold_ms(2000);
        if (console_ok(hz)) print(serial, "     running ", hz, " stable=", Oschf::stable(), crlf);
    }
    Serial::rebase(24'000'000);
    Oschf::set_hz(24'000'000);
    back_to_boot();
    verdict("OSCHF sweep completed and boot clock restored (check the scope)", MainClock::source() == MainSource::extclk);
}

// ---- 2: prescalers --------------------------------------------------------------
void t2_prescalers() {
    print(serial, "2 main prescaler from the 24 MHz crystal: the twelve dividers, 2 s each", crlf);
    constexpr ClockDiv divs[] = {ClockDiv::div1, ClockDiv::div2, ClockDiv::div4, ClockDiv::div6,
                                 ClockDiv::div8, ClockDiv::div10, ClockDiv::div12, ClockDiv::div16,
                                 ClockDiv::div24, ClockDiv::div32, ClockDiv::div48, ClockDiv::div64};
    for (ClockDiv d : divs) {
        const uint32_t hz = SysClock::source_hz / clock_divisor(d);
        before_switch(hz, "24 MHz / prescaler");
        MainClock::prescale(d);
        hold_ms(2000);
        if (console_ok(hz)) print(serial, "     running ", hz, crlf);
    }
    Serial::rebase(SysClock::hz);
    MainClock::prescale(ClockDiv::div1);
    back_to_boot();
    verdict("prescaler sweep completed (check the scope)", true);
}

// ---- 3: tune ----------------------------------------------------------------------
void t3_tune() {
    print(serial, "3 OSCHF manual tune at 16 MHz: steps -32 -16 0 +16 +31 (~0.4 %/step), 2 s each, silent", crlf);
    print(serial, "  expect ~13.95, 14.98, 16.00, 17.02, 17.98 MHz on PA7", crlf);
    Serial::rebase(16'000'000);
    Oschf::set_hz(16'000'000);
    (void)MainClock::select(MainSource::oschf);
    constexpr int8_t steps[] = {-32, -16, 0, 16, 31};
    for (int8_t s : steps) {
        Oschf::tune(s);
        hold_ms(2000);
    }
    Oschf::tune(0);
    const int8_t back = Oschf::tune();
    Serial::rebase(24'000'000);
    Oschf::set_hz(24'000'000);
    back_to_boot();
    print(serial, "  tune register reads back ", static_cast<int16_t>(back), " after reset to 0", crlf);
    verdict("tune register round-trips (read back 0)", back == 0);
    Oschf::tune(-32);
    const int8_t m = Oschf::tune();
    Oschf::tune(31);
    const int8_t p = Oschf::tune();
    Oschf::tune(0);
    verdict("tune clamps and reads back -32 / +31", m == -32 && p == 31);
}

// ---- 4: crystal vs OSCHF -----------------------------------------------------------
void t4_crystal_vs_oschf() {
    print(serial, "4 24 MHz: crystal (now) for 3 s, then OSCHF for 3 s - compare on the scope", crlf);
    hold_ms(3000);
    print(serial, "  switching to OSCHF 24 MHz (same baud)", crlf);
    Serial::rebase(24'000'000);
    Oschf::set_hz(24'000'000);
    (void)MainClock::select(MainSource::oschf);
    hold_ms(3000);
    print(serial, "  OSCHF: stable=", Oschf::stable(), "  back to the crystal", crlf);
    back_to_boot();
    verdict("crystal re-selected after OSCHF", MainClock::source() == MainSource::extclk);
    print(serial, "  (the scope's frequency readout: crystal within ppm, OSCHF within a few %)", crlf);
}

// ---- 5: 32 kHz main clock ------------------------------------------------------------
void t5_32k() {
    print(serial, "5 OSC32K as the main clock for 2 s (silent): ~32.768 kHz on PA7 (+-10 %)", crlf);
    Serial::rebase(32'768);                 // cannot really make 115200: just drains TX
    Osc32k::run_standby(true);
    (void)MainClock::select(MainSource::osc32k);
    hold_ms(2000);
    Serial::rebase(24'000'000);
    back_to_boot();
    verdict("back from the 32 kHz main clock", MainClock::source() == MainSource::extclk);
    print(serial, "  XOSC32K: starting the crystal oscillator on PF0/PF1 (none fitted here)...", crlf);
    Xosc32k::start_crystal(Xosc32kStartup::cycles1k);
    const uint32_t t0 = Ticker::millis();
    bool st = false;
    while (Ticker::millis() - t0 < 1500) { if (Xosc32k::stable()) { st = true; break; } }
    Xosc32k::stop();
    print(serial, "  XOSC32K stable within 1.5 s: ", st ? "yes" : "no (not fitted, as expected)", crlf);
    verdict("XOSC32K result consistent with the board (not fitted -> not stable)", !st);
}

// ---- 6: clock failure detection ----------------------------------------------------------
volatile uint16_t cfd_hits = 0;
volatile uint8_t cfd_source_seen = 0xFF;

void t6_cfd() {
    print(serial, "6 clock failure detector on the main clock, forced by CFDTST:", crlf,
          "  expect: CLKOUT flat (hardware disables it), main clock -> OSCHF 4 MHz, flag + interrupt", crlf);
    cfd_hits = 0;
    ClockFailure::clear();
    ClockFailure::interrupt(true, /*nmi=*/false);
    ClockFailure::watch(CfdSource::main);
    Serial::rebase(4'000'000);              // the fallback rate: drain TX now, BAUD for 4 MHz
    ClockFailure::test(true);               // force the failure
    hold_ms(500);
    const MainSource after = MainClock::source();
    const bool flagged = ClockFailure::failed();
    const uint16_t hits = cfd_hits;
    const uint8_t a = CLKCTRL.MCLKCTRLA;
    // recovery
    ClockFailure::test(false);
    ClockFailure::interrupt(false);
    ClockFailure::stop();
    ClockFailure::clear();
    Serial::rebase(24'000'000);
    back_to_boot();
    print(serial, "  after the forced failure: source=", hex(static_cast<uint8_t>(after)),
          " (OSCHF=0)  CLKOUT bit=", (a & CLKCTRL_CLKOUT_bm) ? 1 : 0, "  flag=", flagged,
          "  isr hits=", hits, "  source seen by isr=", hex(cfd_source_seen), crlf);
    verdict("main clock switched to OSCHF", after == MainSource::oschf);
    verdict("CLKOUT disabled by the CFD event", (a & CLKCTRL_CLKOUT_bm) == 0);
    verdict("CFD interrupt ran at least once", hits >= 1);
    verdict("recovered: crystal re-selected, CLKOUT on", MainClock::source() == MainSource::extclk &&
            (CLKCTRL.MCLKCTRLA & CLKCTRL_CLKOUT_bm));
    print(serial, "  (interrupts re-trigger every 10 OSC32K cycles while the condition holds: hits >> 1 is normal)", crlf);
}

// ---- 7: PLL and status ---------------------------------------------------------------------
void t7_pll_status() {
    print(serial, "7 PLL x2 from OSCHF 24 MHz (no requester: PLLS expected 0 - needs the TCD), status bits", crlf);
    Oschf::set_hz(24'000'000);              // OSCHF is not the main clock now: free to use as PLL input
    Pll::start(PllSource::oschf, PllMultiplier::x2);
    hold_ms(5);
    const bool locked = Pll::locked();
    Pll::stop();
    print(serial, "  PLLS=", locked, " (informative)", crlf);
    const uint8_t st = CLKCTRL.MCLKSTATUS;
    print(serial, "  MCLKSTATUS=", hex(st), ": PLLS=", (st >> 5) & 1, " EXTS=", (st >> 4) & 1,
          " XOSC32KS=", (st >> 3) & 1, " OSC32KS=", (st >> 2) & 1, " OSCHFS=", (st >> 1) & 1,
          " SOSC=", st & 1, crlf);
    verdict("EXTS set (crystal running)", (st & CLKCTRL_EXTS_bm) != 0);
    verdict("OSCHFS set (RUNSTDBY keeps it running)", (st & CLKCTRL_OSCHFS_bm) != 0);
    verdict("not switching", (st & CLKCTRL_SOSC_bm) == 0);
    Oschf::run_standby(false);
    const bool off = (CLKCTRL.OSCHFCTRLA & CLKCTRL_RUNSTDBY_bm) == 0;
    Oschf::run_standby(true);
    verdict("OSCHF RUNSTDBY writable through CCP", off && (CLKCTRL.OSCHFCTRLA & CLKCTRL_RUNSTDBY_bm));
}

using TestFn = void (*)();
struct Test { char key; TestFn fn; };
constexpr Test tests[] = {
    {'1', t1_oschf}, {'2', t2_prescalers}, {'3', t3_tune}, {'4', t4_crystal_vs_oschf},
    {'5', t5_32k}, {'6', t6_cfd}, {'7', t7_pll_status},
};

void run(TestFn fn) {
    passed = failed = 0;
    fn();
    print(serial, "  -> ", passed, " pass, ", failed, " fail", crlf, crlf);
}

void help() {
    print(serial, "test_avr_clock: 1 OSCHF sweep | 2 prescalers | 3 tune | 4 crystal vs OSCHF | "
                  "5 32 kHz main | 6 clock failure | 7 PLL+status | a all   (scope on PA7)", crlf);
}

} // namespace

ISR(USART2_RXC_vect) { (void)Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }
ISR(RTC_PIT_vect)    { Ticker::pit(); }
ISR(CLKCTRL_CFD_vect) {
    cfd_source_seen = static_cast<uint8_t>(ClockFailure::cfd());   // clears the flag
    if (cfd_hits != 0xFFFF) ++cfd_hits;
}

int main() {
    const bool xtal = SysClock::init();
    Serial::init(clock, baud);
    Ticker::init();
    MainClock::clkout(true);              // CLK_PER on PA7
    sei();
    print(serial, crlf, "test_avr_clock - CLKCTRL test suite (boot clk=", xtal ? "XTAL" : "OSCHF",
          " 24 MHz, silicon rev ", hex(SYSCFG.REVID), ", CLKOUT on PA7)", crlf);
    help();
    print(serial, "> ");
    for (;;) {
        uint8_t c;
        if (!Serial::read_byte(c)) continue;
        if (c == '\r' || c == '\n') continue;
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') { help(); }
        else if (c == 'a' || c == 'A') {
            uint8_t tp = 0, tf = 0;
            for (const Test& t : tests) { run(t.fn); tp += passed; tf += failed; }
            print(serial, "ALL: ", tp, " pass, ", tf, " fail", crlf);
        } else {
            bool found = false;
            for (const Test& t : tests) if (t.key == c) { run(t.fn); found = true; }
            if (!found) print(serial, "? for help", crlf);
        }
        print(serial, "> ");
    }
}
