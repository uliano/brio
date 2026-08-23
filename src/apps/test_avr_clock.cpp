// test_avr_clock - the CLKCTRL test SUITE for the AVR DA/DB target:
// every clock source, the prescaler, the OSCHF tune, the 32 kHz main
// clock, the clock failure detector - exercised with CLK_PER on the
// CLKOUT pin (PA7) for an oscilloscope, and the console reporting what
// to expect and what the status bits say. Reference test of
// avrdx/clock.hpp (docs/avrdx/clkctrl.md): keep it passing.
//
// Bench diagnostic, NOT a kernel app (sequential, blocking). Console
// at 9600 baud on purpose: the USART needs CLK_PER >= 16 x baud, so at
// 9600 it keeps talking down to 153.6 kHz - every OSCHF rate, every
// prescaler (375 kHz included), and the tune sweep (the console is
// retuned to the EXPECTED tuned rate, within ~1 %). Only the 32 kHz
// main clock is silent: 1200 baud would fit arithmetically, but the
// OSC32K is +-10 % and a UART tolerates ~4 %. The RTC-based Ticker
// times the holds (it does not depend on the main clock); at 32 kHz it
// is paused (a 1024 Hz ISR cannot be served in 32 cycles).
//
// Wiring: scope on PA7 (CLKOUT). Nothing else.
// Commands: ? | 1 OSCHF sweep | 2 prescalers | 3 tune | 4 crystal vs
// OSCHF | 5 32 kHz main clock | 6 clock failure | 7 PLL + status | a all
// | s step mode (each hold waits for a key: read the scope at leisure)
//
// Tests:
//   1  OSCHF at 24/20/16/12/8/4/3/2/1 MHz as the main clock: CLKOUT
//      shows each (accuracy +-2..5 % calibrated >= 4 MHz, +-6..10 %
//      below), the console follows all of them;
//   2  the twelve prescalers from the 24 MHz crystal: 24 .. 0.375 MHz,
//      the console follows all of them;
//   3  OSCHF manual tune at 16 MHz: -32, -16, 0, +16, +31 steps of
//      nominal 0.4 %; bench (A5, scope): 14.56 / 15.22 / 15.97 / ~17.0 /
//      17.96 MHz - about 0.28 %/step downward, 0.4 %/step upward; the
//      console is retuned to the MEASURED rate at each step and keeps
//      talking;
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
//      informative; all MCLKSTATUS bits (OSCHFS reads 0 while OSCHF is
//      idle even with RUNSTDBY: the status follows the request) and the
//      OSCHF tune/RUNSTDBY registers read back.
// Not testable here: XOSC32K and auto-tune (no 32 kHz crystal fitted),
// an external clock on PA0 (the crystal sits there), the PLL running
// (needs the TCD), the NMI form of the CFD interrupt (locks the
// configuration until reset: run once, on purpose, not in a suite).

// pio: monitor_speed = 9600

#include <avr/interrupt.h>
#include <stdint.h>

#include "avrdx/clock.hpp"
#include "avrdx/delay.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/ticker.hpp"
#include "avrdx/usart.hpp"
#include "avrdx/userrow.hpp"
#include "util/print.hpp"

using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<2, Route::alt1>;
constexpr Serial serial;
constexpr uint32_t baud = 9600;

uint8_t passed = 0, failed = 0;
void verdict(const char* name, bool ok) {
    if (ok) ++passed; else ++failed;
    print(serial, "  ", ok ? "PASS" : "FAIL", "  ", name, crlf);
}

// ---- running at another rate for a while ------------------------------------
// The console can follow a rate only if the USART can make the baud.
bool console_ok(uint32_t hz) { return Serial::can_baud(hz, baud); }

// Step mode ('s' toggles): every hold waits for a key ('n' or any
// other) instead of a fixed time, so the scope can be read and noted
// at leisure - when the console can talk at the current rate; 60 s
// fallback. Timed otherwise.
bool step_mode = false;
uint32_t current_hz = SysClock::hz;      // what the console is tuned for right now

// Hold using the RTC-based Ticker (independent of CLK_PER).
void hold_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    if (step_mode && console_ok(current_hz)) {
        print(serial, "     [n] next", crlf);
        uint8_t c;
        while (Ticker::millis() - t0 < 60000) {
            if (Serial::read_byte(c)) return;
        }
        return;
    }
    while (Ticker::millis() - t0 < ms) {}
}

// Announce the next rate, retune the console for it (drains TX first),
// then the caller switches. If the console cannot follow, say so
// before going silent.
void before_switch(uint32_t next_hz, const char* what) {
    print(serial, "  -> ", what, " = ", next_hz, " Hz on PA7",
          console_ok(next_hz) ? "" : " (console silent)", crlf);
    Serial::rebase(next_hz);                 // drains TX, then BAUD for the next rate
    current_hz = next_hz;
}

// Back on the boot clock: crystal 24 MHz, console retuned.
void back_to_boot() {
    Serial::rebase(SysClock::hz);
    (void)SysClock::init();
    MainClock::clkout(true);
    Serial::rebase(SysClock::hz);
    current_hz = SysClock::hz;
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
    print(serial, "3 OSCHF manual tune at 16 MHz: steps -32 -16 0 +16 +31 (~0.4 %/step), 2 s each", crlf);
    print(serial, "  bench (A5): 14.56, 15.22, 15.97, ~17.0, 17.96 MHz on PA7 - asymmetric, not 0.4 %/step", crlf);
    Serial::rebase(16'000'000);
    current_hz = 16'000'000;
    Oschf::set_hz(16'000'000);
    (void)MainClock::select(MainSource::oschf);
    // The console is retuned to the rate each step REALLY gives on this
    // part (bench, rev A5, scope): the curve is not 0.4 %/step - about
    // 0.28 %/step downward, 0.4 %/step upward. The data sheet's linear
    // 0.4 % is printed next to it for reference.
    struct Step { int8_t steps; uint32_t measured_hz; };
    constexpr Step table[] = {{-32, 14'560'000}, {-16, 15'220'000}, {0, 15'970'000},
                              {16, 17'020'000}, {31, 17'960'000}};
    for (const Step& st : table) {
        const int8_t s = st.steps;
        const uint32_t linear = static_cast<uint32_t>(16'000'000LL + 64'000LL * s);
        const uint32_t expected = st.measured_hz;
        print(serial, "  tune ", static_cast<int16_t>(s), " -> bench ~", expected, " Hz (0.4 %/step would be ",
              linear, ")", crlf);
        Serial::rebase(expected);
        current_hz = expected;
        Oschf::tune(s);
        hold_ms(2000);
        print(serial, "     (console alive at the tuned rate)", crlf);
    }
    Serial::rebase(16'000'000);
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
    Serial::rebase(32'768);                 // cannot make the baud there: just drains TX
    current_hz = 32'768;                    // hold() stays timed: the console cannot talk
    Osc32k::run_standby(true);
    // At 32 kHz a 1024 Hz tick interrupt would never return (32 cycles
    // per tick): pause it, wait in raw cycles, resume after the return.
    Ticker::pause();
    (void)MainClock::select(MainSource::osc32k);
    delay_cycles(2u * 32768u);              // 2 s at 32.768 kHz
    Serial::rebase(24'000'000);
    (void)SysClock::init();
    Ticker::resume();
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
    // The data sheet says the fallback is OSCHF "changed back to its Reset
    // frequency" (4 MHz). Bench question: does FRQSEL really move? Capture
    // OSCHFCTRLA right after the event; the console is retuned for 4 MHz
    // on the data sheet's word, and this hold is TIMED (no key) because
    // we cannot know the rate until we read the register back.
    Serial::rebase(4'000'000);
    ClockFailure::test(true);               // force the failure
    const bool saved_step = step_mode;
    step_mode = false;
    hold_ms(500);
    step_mode = saved_step;
    const MainSource after = MainClock::source();
    const bool flagged = ClockFailure::failed();
    const uint16_t hits = cfd_hits;
    const uint8_t a = CLKCTRL.MCLKCTRLA;
    const uint8_t oschf_after = CLKCTRL.OSCHFCTRLA;    // FRQSEL bits 5:2: 0x3 = 4 MHz, 0x9 = 24 MHz
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
    const uint8_t frqsel = static_cast<uint8_t>((oschf_after >> 2) & 0x0F);
    print(serial, "  OSCHF FRQSEL after the event: ", hex(frqsel),
          " (0x3 = 4 MHz = the data sheet's reset frequency, 0x9 = 24 MHz = unchanged)", crlf);
    verdict("OSCHF frequency after CFD fallback recorded (see line above)", true);
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
    // Bench finding: OSCHFS reads 0 while OSCHF is not the main clock and
    // nobody requests it, RUNSTDBY notwithstanding (the status follows
    // the request, like the register note says of the signal); it reads
    // 1 whenever OSCHF is the main clock (test 4).
    print(serial, "  OSCHFS=", (st >> 1) & 1, " with OSCHF idle and RUNSTDBY on (finding: status follows the request)", crlf);
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
                  "5 32 kHz main | 6 clock failure | 7 PLL+status | a all | s step mode   (scope on PA7)", crlf);
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
    auto board = board_id();
    if (board.empty()) board = "?";
    print(serial, crlf, "test_avr_clock - CLKCTRL test suite (board ", board,
          ", boot clk=", xtal ? "XTAL" : "OSCHF",
          " 24 MHz, silicon rev ", hex(SYSCFG.REVID), ", CLKOUT on PA7)", crlf);
    help();
    print(serial, "> ");
    for (;;) {
        uint8_t c;
        if (!Serial::read_byte(c)) continue;
        if (c == '\r' || c == '\n') continue;
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') { help(); }
        else if (c == 's' || c == 'S') {
            step_mode = !step_mode;
            print(serial, "step mode ", step_mode ? "ON: each hold waits for a key" : "OFF: timed holds", crlf);
        }
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
