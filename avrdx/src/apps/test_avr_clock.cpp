// test_avr_clock - the CLKCTRL test SUITE for the AVR DA/DB target:
// every clock source, the prescaler, the OSCHF tune, the 32 kHz main
// clock, the clock failure detector - exercised with CLK_PER on the
// CLKOUT pin (PA7) for an oscilloscope where the board leaves PA7 free,
// and the console reporting what to expect and what the status bits
// say. Reference test of avrdx/clock.hpp (docs/avrdx/clkctrl.md): keep
// it passing.
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
// THE TWO BENCH BOARDS differ where this suite looks, and one board per
// part sits on the desk (the manifest's board type IS the part), so the
// part decides at compile time:
//   - the AVR128DB48 board: the 24 MHz crystal on XOSCHF (PA0/PA1), PA7
//     free for CLKOUT, no 32.768 kHz crystal;
//   - the AVR128DA48 board: a 24 MHz quartz oscillator into EXTCLK (PA0),
//     PA7 carrying 100 nF to ground - CLKOUT is NEVER enabled there, the
//     pad would source and sink its short-circuit current for as long as
//     CLK_PER ran into the capacitor - and a 32.768 kHz crystal on
//     PF0/PF1, which letters 8 and 9 measure.
//
// Wiring: scope on PA7 (CLKOUT) on the DB board. Nothing else.
// Commands: ? | 1 OSCHF sweep | 2 prescalers | 3 tune | 4 crystal vs
// OSCHF | 5 32 kHz main clock | 6 clock failure | 7 PLL + status
// | 8 auto-tune | 9 the 32 kHz crystal as the main clock | a all
// | s step mode (each hold waits for a key: read the scope at leisure)
//
// Tests:
//   1  OSCHF at 24/20/16/12/8/4/3/2/1 MHz as the main clock: CLKOUT
//      shows each (accuracy +-2..5 % calibrated >= 4 MHz, +-6..10 %
//      below), the console follows all of them;
//   2  the twelve prescalers from the 24 MHz source: 24 .. 0.375 MHz,
//      the console follows all of them;
//   3  OSCHF manual tune at 16 MHz: -32, -16, 0, +16, +31 steps of
//      nominal 0.4 %; measured on rev A5 with a scope: 14.56 / 15.22 /
//      15.97 / ~17.0 / 17.96 MHz - about 0.28 %/step downward and
//      0.4 %/step upward; the console is retuned to the MEASURED rate
//      at each step and keeps talking;
//   4  the 24 MHz source (the DB's crystal, the DA's external clock) vs
//      OSCHF at 24 MHz: same nominal, the scope tells the accuracy apart
//      (crystal ppm, OSCHF %);
//   5  OSC32K as the main clock: CLKOUT at ~32 kHz (+-10 %) for two
//      seconds, silent; then XOSC32K is started forced on: it must come
//      up on the board that fits the crystal and must not on the other;
//   6  the clock failure detector (DB only: the DA has none) watching the
//      main clock, forced by the test bit: the main clock falls back to
//      OSCHF 4 MHz, CLKOUT is disabled by hardware (scope flat), the CFD
//      flag and the regular interrupt fire; then recovery: flag cleared,
//      clock re-initialised, CLKOUT back;
//   7  PLL x2 from OSCHF: PLLS stays 0 without a requester (the TCD) -
//      informative; all MCLKSTATUS bits (OSCHFS reads 0 while OSCHF is
//      idle even with RUNSTDBY: the status follows the request) and the
//      OSCHF tune/RUNSTDBY registers read back;
//   8  auto-tune (the board with the 32.768 kHz crystal), on two rulers:
//      A - CLK_PER the external clock, OSCHF clocking the TCD whose
//      32768-cycle period on WOA (PA4) a TCB times: a TUNE sweep around
//      24 MHz (printed: at the nearest step OSCHF runs at the external
//      clock's own rate, docs/avrdx/clkctrl.md), and the auto-tune with no
//      crystal running, which must leave OSCHF where TUNE put it; B -
//      CLK_PER OSCHF itself, timed by a TCB cascade against the RTC
//      counting the crystal: detuned by -16 steps, then AUTOTUNE must
//      bring it within one tune step, keep it there with the external
//      clock requested by a TCD, and leave it there once cleared;
//   9  Clock<xosc32k> as the main clock (the same board): the init must
//      report the crystal, and CLK_PER is timed against the 24 MHz
//      external clock through the TCD (EXTCLK, 1048576 cycles a period on
//      WOA) and a TCB counting CLK_PER - the ratio within the two
//      crystals' tolerance, where OSC32K would be per cent off.
// Not testable here: an external clock on the DB's PA0 (its crystal sits
// there), the NMI form of the CFD interrupt (locks the configuration
// until reset: run once, on purpose, not in a suite). The PLL running is
// test_avr_tcd's letter f, where the TCD requests it.

// build: boards = db48,da48
// build: monitor_speed = 9600

#include <avr/interrupt.h>
#include <stdint.h>

#include "avrdx/clock.hpp"
#include "avrdx/delay.hpp"
#include "avrdx/evsys.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/rtc.hpp"
#include "avrdx/tcb.hpp"
#include "avrdx/tcd.hpp"
#include "avrdx/ticker.hpp"
#include "avrdx/usart.hpp"
#include "avrdx/userrow.hpp"
#include "util/print.hpp"

using SysClock = brio::Clock<brio::has_xoschf ? brio::ClockSource::crystal
                                              : brio::ClockSource::external,
                             24'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<2, Route::alt1>;
constexpr Serial serial;
constexpr uint32_t baud = 9600;

// The two bench boards (the header): one per part, so the part decides.
/// PA7 is free for CLKOUT (the DB board) - or carries 100 nF (the DA's).
constexpr bool board_clkout = has_xoschf;
/// A 32.768 kHz crystal sits on PF0/PF1 (the DA board) - or none (the DB's).
constexpr bool board_xosc32k = !has_xoschf;

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
    if constexpr (board_clkout) {
        print(serial, "  -> ", what, " = ", next_hz, " Hz on PA7",
              console_ok(next_hz) ? "" : " (console silent)", crlf);
    } else {
        print(serial, "  -> ", what, " = ", next_hz, " Hz",
              console_ok(next_hz) ? "" : " (console silent)", crlf);
    }
    Serial::rebase(next_hz);                 // drains TX, then BAUD for the next rate
    current_hz = next_hz;
}

// Back on the boot clock: the 24 MHz source, console retuned.
void back_to_boot() {
    Serial::rebase(SysClock::hz);
    (void)SysClock::init();
    if constexpr (board_clkout) MainClock::clkout(true);
    Serial::rebase(SysClock::hz);
    current_hz = SysClock::hz;
}

// ---- 1: OSCHF sweep -------------------------------------------------------------
void t1_oschf() {
    if constexpr (board_clkout) {
        print(serial, "1 OSCHF as main clock: 24,20,16,12,8,4,3,2,1 MHz, 2 s each (scope PA7)", crlf);
    } else {
        print(serial, "1 OSCHF as main clock: 24,20,16,12,8,4,3,2,1 MHz, 2 s each", crlf);
    }
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
    if constexpr (has_xoschf) {
        print(serial, "2 main prescaler from the 24 MHz crystal: the twelve dividers, 2 s each", crlf);
    } else {
        print(serial, "2 main prescaler from the 24 MHz external clock: the twelve dividers, 2 s each", crlf);
    }
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
    if constexpr (board_clkout) {
        print(serial, "  bench (A5): 14.56, 15.22, 15.97, ~17.0, 17.96 MHz on PA7 - asymmetric, not 0.4 %/step", crlf);
    } else {
        print(serial, "  bench (DB A5): 14.56, 15.22, 15.97, ~17.0, 17.96 MHz - asymmetric, not 0.4 %/step", crlf);
    }
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
    if constexpr (has_xoschf) {
        print(serial, "4 24 MHz: crystal (now) for 3 s, then OSCHF for 3 s - compare on the scope", crlf);
    } else {
        print(serial, "4 24 MHz: external clock (now) for 3 s, then OSCHF for 3 s", crlf);
    }
    hold_ms(3000);
    print(serial, "  switching to OSCHF 24 MHz (same baud)", crlf);
    Serial::rebase(24'000'000);
    Oschf::set_hz(24'000'000);
    (void)MainClock::select(MainSource::oschf);
    hold_ms(3000);
    if constexpr (has_xoschf) {
        print(serial, "  OSCHF: stable=", Oschf::stable(), "  back to the crystal", crlf);
    } else {
        print(serial, "  OSCHF: stable=", Oschf::stable(), "  back to the external clock", crlf);
    }
    back_to_boot();
    verdict(has_xoschf ? "crystal re-selected after OSCHF" : "external clock re-selected after OSCHF",
            MainClock::source() == MainSource::extclk);
    if constexpr (board_clkout) {
        print(serial, "  (the scope's frequency readout: crystal within ppm, OSCHF within a few %)", crlf);
    }
}

// ---- 5: 32 kHz main clock ------------------------------------------------------------
void t5_32k() {
    if constexpr (board_clkout) {
        print(serial, "5 OSC32K as the main clock for 2 s (silent): ~32.768 kHz on PA7 (+-10 %)", crlf);
    } else {
        print(serial, "5 OSC32K as the main clock for 2 s (silent): ~32.768 kHz (+-10 %)", crlf);
    }
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
    if constexpr (board_xosc32k) {
        print(serial, "  XOSC32K: starting the crystal oscillator on PF0/PF1 (fitted here)...", crlf);
    } else {
        print(serial, "  XOSC32K: starting the crystal oscillator on PF0/PF1 (none fitted here)...", crlf);
    }
    // Forced on: unrequested, a fitted crystal would never read stable
    // either, and the verdict below could not tell the two boards apart.
    Xosc32k::start_crystal(Xosc32kStartup::cycles1k, false, true);
    const uint32_t t0 = Ticker::millis();
    bool st = false;
    while (Ticker::millis() - t0 < 1500) { if (Xosc32k::stable()) { st = true; break; } }
    Xosc32k::stop();
    if constexpr (board_xosc32k) {
        print(serial, "  XOSC32K stable within 1.5 s: ", st ? "yes" : "no", crlf);
        verdict("XOSC32K result consistent with the board (fitted -> stable)", st);
    } else {
        print(serial, "  XOSC32K stable within 1.5 s: ", st ? "yes" : "no (not fitted, as expected)", crlf);
        verdict("XOSC32K result consistent with the board (not fitted -> not stable)", !st);
    }
}

// ---- 6: clock failure detection ----------------------------------------------------------
// The detector exists on the DB only; clock.hpp compiles the block out
// on the DA (no CLKCTRL_CFDEN_bm in its header) and so does this letter.
#if defined(CLKCTRL_CFDEN_bm)
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
#else
void t6_cfd() {
    print(serial, "6 clock failure detector: SKIP - this part has none (CLKCTRL.MCLKCTRLC "
                  "and the CFD flag are AVR DB only)", crlf);
}
#endif

// ---- 7: PLL and status ---------------------------------------------------------------------
void t7_pll_status() {
    print(serial, "7 PLL x2 from OSCHF 24 MHz (no requester: PLLS expected 0 - needs the TCD), status bits", crlf);
    Oschf::set_hz(24'000'000);              // OSCHF is not the main clock now: free to use as PLL input
    (void)Pll::start(PllSource::oschf, PllMultiplier::x2);
    hold_ms(5);
    const bool locked = Pll::locked();
    Pll::stop();
    print(serial, "  PLLS=", locked, " (informative)", crlf);
    const uint8_t st = CLKCTRL.MCLKSTATUS;
    print(serial, "  MCLKSTATUS=", hex(st), ": PLLS=", (st >> 5) & 1, " EXTS=", (st >> 4) & 1,
          " XOSC32KS=", (st >> 3) & 1, " OSC32KS=", (st >> 2) & 1, " OSCHFS=", (st >> 1) & 1,
          " SOSC=", st & 1, crlf);
    verdict(has_xoschf ? "EXTS set (crystal running)" : "EXTS set (external clock running)",
            (st & CLKCTRL_EXTS_bm) != 0);
    // MEASURED: OSCHFS reads 0 while OSCHF is not the main clock and
    // nobody requests it, RUNSTDBY notwithstanding - the status follows
    // the REQUEST, exactly as the register note says of the signal; it
    // reads 1 whenever OSCHF is the main clock (test 4).
    print(serial, "  OSCHFS=", (st >> 1) & 1, " with OSCHF idle and RUNSTDBY on (finding: status follows the request)", crlf);
    verdict("not switching", (st & CLKCTRL_SOSC_bm) == 0);
    Oschf::run_standby(false);
    const bool off = (CLKCTRL.OSCHFCTRLA & CLKCTRL_RUNSTDBY_bm) == 0;
    Oschf::run_standby(true);
    verdict("OSCHF RUNSTDBY writable through CCP", off && (CLKCTRL.OSCHFCTRLA & CLKCTRL_RUNSTDBY_bm));
}


// ---- 8, 9: the 32.768 kHz crystal (the board that fits one) ------------------------
// The ruler of both letters: a TCD period on WOA (PA4, the DEFAULT route,
// half duty) read back as a pin event by a TCB counting CLK_PER. The TCD
// counts a known number of cycles of ITS clock per period and the TCB
// counts CLK_PER across the same period, so the reading IS the ratio of
// the two clocks. In frequency mode the TCB restarts at each edge, so the
// sum of consecutive captures telescopes: n periods summed carry the
// quantization of one capture (two ticks), not of n.
using Ruler = Tcd<0>;
using RulerPin = Pin<'A', 4>;
using ChRuler = EventChannel<0>;          // PORTA pin events: channels 0-1
using MeterTcb = Tcb<0>;
using Meter = FrequencyMeter<MeterTcb>;

/// The TCD in one ramp, 4096 counter ticks a period, WOA high for half.
bool ruler_start(TcdClock src, TcdSyncPrescaler sp, TcdCountPrescaler cp) {
    TcdConfig c{};
    c.route = TcdRoute::def;
    c.clock = src;
    c.sync_prescaler = sp;
    c.count_prescaler = cp;
    c.waveform = TcdWaveform::one_ramp;
    c.compare_a_set = 100;
    c.compare_a_clear = 2147;
    c.compare_b_set = 2148;
    c.compare_b_clear = 4095;
    c.enable_woa = true;
    if (!Ruler::init(c)) return false;
    ChRuler::source(EvPin<RulerPin>{});
    // The meter's init turns its CAPT interrupt on and no TCB vector is
    // bound here: armed and closed again with interrupts off.
    cli();
    Meter::init(clock, ChRuler{});
    MeterTcb::enable_capt_interrupt(false);
    sei();
    return true;
}

void ruler_stop() {
    (void)Ruler::release();
    MeterTcb::disable();
    ChRuler::off();
}

/// The sum of `n` consecutive periods in CLK_PER ticks, after `discard`
/// captures (the first closes a period begun before the meter was
/// armed); 0 when the signal stopped - `spins` bounds the wait for each.
uint32_t ruler_sum(uint8_t n, uint8_t discard, uint32_t spins) {
    MeterTcb::clear_capt();
    uint32_t sum = 0;
    for (uint8_t i = 0; i < static_cast<uint8_t>(n + discard); ++i) {
        uint32_t left = spins;
        while (!MeterTcb::capt_flag()) {
            if (--left == 0) return 0;
        }
        const uint16_t t = Meter::period_ticks();
        if (i >= discard) sum += t;
    }
    return sum;
}

/// The clock under test against CLK_PER in parts per million: a reading
/// of `ticks` where the nominal rate would have read `nominal / 1e6`
/// (positive: the clock under test runs fast).
int32_t ppm_of(uint32_t ticks, uint64_t nominal_ticks_x1e6) {
    if (ticks == 0) return INT32_MIN;
    const int64_t num = static_cast<int64_t>(nominal_ticks_x1e6) -
                        static_cast<int64_t>(ticks) * 1'000'000LL;
    return static_cast<int32_t>(num / static_cast<int64_t>(ticks));
}

/// XOSC32K started forced on (nothing requests it yet) and waited for,
/// at most 1.5 s on the Ticker.
bool xosc32k_up() {
    Xosc32k::start_crystal(Xosc32kStartup::cycles1k, false, true);
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 1500) {
        if (Xosc32k::stable()) return true;
    }
    return false;
}

// One tune step of OSCHF is 0.4 % (DS40002183A table 37-10, %CAL), and the
// auto-tune corrects by one step whenever its 1 MHz versus 1.024 kHz check
// sees an error (11.3.5): a tuned OSCHF sits within one step of the
// target. The two crystals add their tolerances - a 32.768 kHz tuning
// fork and the board's 24 MHz oscillator, neither part number on record,
// 100 ppm each allowed - and the reading's quantization is two ticks in
// 32 x 32768 (0.06 ppm).
constexpr int32_t one_step_ppm = 4000;
constexpr int32_t crystals_ppm = 200;

// The second ruler of letter 8: the RTC counting the 32.768 kHz crystal,
// its overflow every 32768 cycles latching a TCB cascade at CLK_PER - so
// CLK_PER itself (OSCHF, as the main clock) is what is timed, against the
// crystal the auto-tune uses, with no other clock requested.
using WideLo = Tcb<1>;
using WideHi = Tcb<2>;
using Wide = CascadedCounter<WideLo, WideHi>;
using ChCarry = EventChannel<2>;
using ChSnap = EventChannel<3>;

/// CLK_PER ticks across `n` overflows of a 32768-cycle RTC period; 0 when
/// a stamp did not come (a spin bound per stamp).
uint32_t xtal_window(uint8_t n) {
    auto stamp = [](uint32_t& out) {
        WideLo::clear_capt();
        WideHi::clear_capt();
        for (uint32_t i = 0; i < 20'000'000u; ++i) {
            if (WideLo::capt_flag() && WideHi::capt_flag()) {
                out = Wide::captured();
                return true;
            }
        }
        return false;
    };
    uint32_t first = 0, last = 0;
    if (!stamp(first)) return 0;
    for (uint8_t i = 0; i < n; ++i) {
        if (!stamp(last)) return 0;
    }
    return last - first;
}

/// OSCHF (the main clock) against the crystal in ppm, over `n` seconds.
int32_t oschf_vs_xtal(uint8_t n) {
    const uint32_t t = xtal_window(n);
    if (t == 0) return INT32_MIN;
    const int64_t want = static_cast<int64_t>(n) * 24'000'000LL;
    return static_cast<int32_t>((static_cast<int64_t>(t) - want) * 1'000'000LL / want);
}

void t8_autotune() {
    print(serial, "8 OSCHF auto-tune against the 32.768 kHz crystal", crlf);
    Xosc32k::stop();
    (void)Oschf::set_hz(24'000'000);
    Oschf::autotune(false);
    Oschf::tune(0);

    // A: CLK_PER = the external clock, the TCD on OSCHF. The TUNE sweep is
    // printed, not judged: at the step nearest 24 MHz OSCHF runs at the
    // external clock's rate exactly on this board (clkctrl.md: it locks to
    // EXTCLK while EXTCLK is CLK_PER) - a fact of the die's calibration
    // and the coupling, with no bound of record to judge it by.
    constexpr uint8_t n = 8;
    constexpr uint64_t nominal = static_cast<uint64_t>(n) * 32768u * 1'000'000u;
    const bool ruler = ruler_start(TcdClock::oschf, TcdSyncPrescaler::div8, TcdCountPrescaler::div1);
    verdict("the TCD ruler runs (TCD on OSCHF, TCB at CLK_PER)", ruler);
    int32_t sweep[13];
    for (int8_t t = -6; t <= 6; ++t) {
        Oschf::tune(t);
        delay_us(clock, 2000);
        sweep[t + 6] = ppm_of(ruler_sum(n, 2, 400'000u), nominal);
    }
    // The auto-tune with no crystal running: 11.5.5 says the crystal must
    // be enabled for it to work - so OSCHF stays where TUNE -16 put it.
    Oschf::tune(-16);
    delay_us(clock, 2000);
    const int32_t a_detuned = ppm_of(ruler_sum(n, 2, 400'000u), nominal);
    Oschf::autotune(true);
    hold_ms(1000);
    const int32_t a_no_xtal = ppm_of(ruler_sum(n, 2, 400'000u), nominal);
    Oschf::autotune(false);
    Oschf::tune(0);
    ruler_stop();
    print(serial, "  A: OSCHF on the TCD against the external clock as CLK_PER, ppm by TUNE:",
          crlf, "   ");
    for (int8_t t = -6; t <= 6; ++t) print(serial, " ", static_cast<int16_t>(t), ":", sweep[t + 6]);
    print(serial, crlf, "     TUNE -16 ", a_detuned, "; AUTOTUNE on with no 32k crystal running ",
          a_no_xtal, crlf);
    verdict("without a running crystal the auto-tune leaves OSCHF where TUNE put it "
            "(within one step)",
            a_detuned != INT32_MIN && a_no_xtal != INT32_MIN &&
                a_no_xtal - a_detuned > -one_step_ppm && a_no_xtal - a_detuned < one_step_ppm);

    // B: CLK_PER = OSCHF, timed against the crystal through the RTC; the
    // external clock requested by a TCD only where a leg says so. Nothing
    // is printed until B ends: TUNE -16 is beyond the console's tolerance.
    const bool xosc = xosc32k_up();
    verdict("XOSC32K runs (started forced on, stable within 1.5 s)", xosc);
    if (!xosc) {
        Xosc32k::stop();
        return;
    }
    RtcClock::select(RtcSource::xosc32k);
    (void)Rtc::init({.prescaler = RtcPrescaler::div1, .period = 32767});
    Wide::init(TcbClock::div1, ChCarry{}, ChSnap{});
    ChSnap::source(Rtc::OvfEvent{});
    Wide::reset();
    TcdConfig ext{};
    ext.clock = TcdClock::extclk;
    ext.compare_b_clear = 4095;
    auto request_extclk = [&ext](bool on) {
        if (on) (void)Ruler::init(ext);
        else (void)Ruler::release();
    };
    while (!Serial::tx_idle()) {
    }
    delay_us(clock, 2000);
    (void)MainClock::select(MainSource::oschf);
    const int32_t b_free = oschf_vs_xtal(2);
    request_extclk(true);
    const int32_t b_free_ext = oschf_vs_xtal(2);
    request_extclk(false);
    Oschf::tune(-16);
    const int32_t b_detuned = oschf_vs_xtal(2);
    Oschf::autotune(true);
    hold_ms(1500);
    const int32_t b_tuned = oschf_vs_xtal(2);
    request_extclk(true);
    const int32_t b_tuned_ext = oschf_vs_xtal(2);
    request_extclk(false);
    // AUTOTUNE cleared: TUNE takes the auto-tune's last value (11.5.6) -
    // but not at once. Timed here on the cascade: from the clear to the
    // first read that differs from the -16 written before.
    const int8_t before = Oschf::tune();
    Oschf::autotune(false);
    const uint32_t c0 = Wide::read();
    uint32_t c1 = c0;
    int8_t settled = before;
    while (c1 - c0 < 12'000'000u) {                  // half a second of CLK_PER
        settled = Oschf::tune();
        c1 = Wide::read();
        if (settled != before) break;
    }
    const int32_t b_held = oschf_vs_xtal(2);
    const int8_t settled_later = Oschf::tune();
    Oschf::tune(0);
    (void)SysClock::init();                            // the external clock again
    Rtc::disable();
    WideLo::disable();
    WideHi::disable();
    ChSnap::off();
    ChCarry::off();
    // Back to OSC32K by NAME: Ticker::init() alone takes preferred(), and
    // XOSC32KS still reads 1 for a while after the oscillator is stopped
    // - a PIT on a stopped crystal would freeze every Ticker hold after.
    RtcClock::select(RtcSource::osc32k);
    Xosc32k::stop();
    Ticker::init(RtcSource::osc32k);

    // The driver's clear waits the copy out (clock.hpp's autotune()): a
    // TUNE written at once after it must stay written. Eight tries, each a
    // value the auto-tune's own (near 0 here) is not.
    uint8_t kept = 0;
    for (uint8_t i = 0; i < 8; ++i) {
        Oschf::autotune(true);
        delay_us(clock, 10'000);
        Oschf::autotune(false);
        Oschf::tune(5);
        delay_us(clock, 10'000);
        if (Oschf::tune() == 5) ++kept;
        Oschf::tune(0);
    }
    print(serial, "  B: OSCHF as CLK_PER against the crystal (RTC), ppm: TUNE 0 ", b_free,
          " (a TCD requesting the external clock: ", b_free_ext, "); TUNE -16 ", b_detuned,
          "; AUTOTUNE on ", b_tuned, " (the TCD on the external clock: ", b_tuned_ext,
          "); AUTOTUNE off ", b_held, crlf,
          "     TUNE after AUTOTUNE off: ", static_cast<int16_t>(before), " until ",
          (c1 - c0) / 24u, " us later, then ", static_cast<int16_t>(settled),
          "; two seconds on: ", static_cast<int16_t>(settled_later), crlf,
          "     a TUNE of 5 written at once after the clear, then read 10 ms on: kept ", kept,
          " of 8", crlf);
    verdict("every reading came back",
            b_free != INT32_MIN && b_free_ext != INT32_MIN && b_detuned != INT32_MIN &&
                b_tuned != INT32_MIN && b_tuned_ext != INT32_MIN && b_held != INT32_MIN);
    // 16 steps of at least the DB's measured 0.28 %/step downward: 4.5 %,
    // less a factory error of a few tenths of a per cent.
    verdict("TUNE -16 takes OSCHF more than 2 % below the crystal", b_detuned < -20'000);
    verdict("the auto-tune brings it back within one tune step of the crystal (4000 ppm)",
            b_tuned > -one_step_ppm && b_tuned < one_step_ppm);
    verdict("and holds it there with a TCD requesting the external clock",
            b_tuned_ext > -one_step_ppm && b_tuned_ext < one_step_ppm);
    verdict("AUTOTUNE cleared, OSCHF keeps the tuned rate (within one step)",
            b_held > -one_step_ppm && b_held < one_step_ppm);
    // The auto-tune's value is the step nearest 24 MHz: within the
    // factory calibration's few per cent of 0, 8 steps either way.
    verdict("TUNE reads the auto-tune's value once the clear has returned",
            settled == settled_later && settled > -8 && settled < 8);
    verdict("a TUNE written right after the clear is kept (the driver waits the copy out)",
            kept == 8);
}

void t9_xosc32k_main() {
    print(serial, "9 Clock<xosc32k> as the main clock: CLK_PER timed against the 24 MHz external "
                  "clock (TCD on EXTCLK, 1048576 cycles a period on PA4) - silent for about 4 s",
          crlf);
    Xosc32k::stop();                        // the init starts it from scratch
    const bool ruler = ruler_start(TcdClock::extclk, TcdSyncPrescaler::div8, TcdCountPrescaler::div32);
    verdict("the ruler runs (TCD on EXTCLK, TCB at CLK_PER)", ruler);
    if (!ruler) return;

    using Xtal32 = Clock<ClockSource::xosc32k, 32768>;
    constexpr uint8_t n = 32;
    Serial::rebase(32'768);                 // cannot make the baud there: just drains TX
    current_hz = 32'768;
    Ticker::pause();                        // a 1024 Hz tick cannot be served at 32 kHz
    const bool init_ok = Xtal32::init();
    const MainSource src = MainClock::source();
    // At 32768 Hz a period of the ruler is 1432 CLK_PER cycles: a spin of
    // the wait is about eight, so 20000 spins outlast a stopped signal by
    // seconds. Two captures straddle the switch and are dropped.
    const uint32_t sum = ruler_sum(n, 2, 20'000u);
    const bool stable = Xosc32k::stable();
    Serial::rebase(24'000'000);
    (void)SysClock::init();
    Ticker::resume();
    back_to_boot();
    ruler_stop();
    Xosc32k::stop();

    // f_PER = sum x 24 MHz / (n x 1048576): its error against 32768 Hz.
    const int64_t want = static_cast<int64_t>(n) * 1'048'576LL * 32'768LL;   // sum x 24e6 at nominal
    const int64_t got = static_cast<int64_t>(sum) * 24'000'000LL;
    const int32_t ppm = sum ? static_cast<int32_t>((got - want) * 1'000'000LL / want) : INT32_MIN;
    print(serial, "  init ", init_ok ? "true" : "false", ", source ",
          hex(static_cast<uint8_t>(src)), " (XOSC32K = ", hex(static_cast<uint8_t>(MainSource::xosc32k)),
          "), XOSC32KS ", stable, "; ", n, " periods = ", sum, " CLK_PER ticks (nominal 45813): ",
          ppm, " ppm", crlf);
    verdict("Clock<xosc32k>::init() reports the crystal", init_ok);
    verdict("the main clock read XOSC32K while it ran", src == MainSource::xosc32k && stable);
    // Bound: the two crystals' 200 ppm (letter 8) plus two ticks of
    // quantization in 45813 (44 ppm) - where OSC32K is +-10 %.
    verdict("CLK_PER is 32768 Hz against the external clock (within 250 ppm)",
            sum != 0 && ppm > -250 && ppm < 250);
    verdict("back on the external clock", MainClock::source() == MainSource::extclk);
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
    if constexpr (board_xosc32k) {
        print(serial, "test_avr_clock: 1 OSCHF sweep | 2 prescalers | 3 tune | 4 external clock vs "
                      "OSCHF | 5 32 kHz main | 6 clock failure | 7 PLL+status | 8 auto-tune | "
                      "9 XOSC32K main | a all | s step mode", crlf);
    } else {
        print(serial, "test_avr_clock: 1 OSCHF sweep | 2 prescalers | 3 tune | 4 crystal vs OSCHF | "
                      "5 32 kHz main | 6 clock failure | 7 PLL+status | a all | s step mode   (scope on PA7)", crlf);
    }
}

} // namespace

ISR(USART2_RXC_vect) { (void)Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }
ISR(RTC_PIT_vect)    { Ticker::pit(); }
#if defined(CLKCTRL_CFDEN_bm)
ISR(CLKCTRL_CFD_vect) {
    cfd_source_seen = static_cast<uint8_t>(ClockFailure::cfd());   // clears the flag
    if (cfd_hits != 0xFFFF) cfd_hits = cfd_hits + 1;
}
#endif

int main() {
    const bool xtal = SysClock::init();
    Serial::init(clock, baud);
    Ticker::init();
    if constexpr (board_clkout) MainClock::clkout(true);   // CLK_PER on PA7
    sei();
    auto board = board_id();
    if (board.empty()) board = "?";
    if constexpr (board_clkout) {
        print(serial, crlf, "test_avr_clock - CLKCTRL test suite (board ", board,
              ", boot clk=", xtal ? (has_xoschf ? "XTAL" : "EXTCLK") : "OSCHF",
              " 24 MHz, silicon rev ", hex(SYSCFG.REVID), ", CLKOUT on PA7)", crlf);
    } else {
        print(serial, crlf, "test_avr_clock - CLKCTRL test suite (board ", board,
              ", boot clk=", xtal ? (has_xoschf ? "XTAL" : "EXTCLK") : "OSCHF",
              " 24 MHz, silicon rev ", hex(SYSCFG.REVID), ", CLKOUT off: PA7 carries 100 nF)",
              crlf);
    }
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
            if constexpr (board_xosc32k) {
                run(t8_autotune); tp += passed; tf += failed;
                run(t9_xosc32k_main); tp += passed; tf += failed;
            }
            print(serial, "ALL: ", tp, " pass, ", tf, " fail", crlf);
        } else {
            bool found = false;
            for (const Test& t : tests) if (t.key == c) { run(t.fn); found = true; }
            if constexpr (board_xosc32k) {
                if (c == '8') { run(t8_autotune); found = true; }
                if (c == '9') { run(t9_xosc32k_main); found = true; }
            }
            if (!found) print(serial, "? for help", crlf);
        }
        print(serial, "> ");
    }
}
