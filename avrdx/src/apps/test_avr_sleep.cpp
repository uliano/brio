// test_avr_sleep - the SLEEP test SUITE for the AVR DA/DB target: the
// three sleep modes of SLPCTRL (avrdx/sleep.hpp) and the voltage
// regulator's sleep profile, measured on the silicon instead of read
// off tables 13-2..13-5 (which errata DS80000915F 3.4.1 rewrites).
//
// What it proves. The register surface and its two enforced rules (the
// erratum 2.2.4 NOP before every CTRLA store, and HTLLEN refused while
// a TWI client or the CCL could still try to wake the device). That
// IDLE through the new verb behaves like the platform's own idle().
// That STANDBY really stops CLK_PER and that the PIT still wakes from
// it. WHICH RUNSTDBY DECIDES: two different bits carry that name, a
// peripheral's and each oscillator's, and the matrix of test d sweeps
// both against two clock sources - the PERIPHERAL's is the one that
// keeps a counter alive (its request revives the whole clock chain),
// the OSCILLATOR's only buys the wake-up time test g then measures.
// That
// POWER-DOWN stops even the RTC counter while its PIT half keeps
// running, and that a PORT pin wakes from both deep modes with the
// pad driven by the device's own event system (no wires: EVSYS is
// alive in every sleep mode, so a PIT divider routed to EVOUT on PD2
// is a wake-up source the board builds for itself).
//
// The TWO-BOARD half (set y) measures what one board cannot: HOW LONG a
// wake-up takes. A sleeping chip cannot time its own return - the only
// clock power-down leaves running is the PIT's, and the counter that
// would measure the restart is exactly the one the mode stops (test f).
// So the ruler moves off-chip: board B (src/apps/sleep_peer.cpp) drives
// a stimulus edge, starts a 32-bit CLK_PER stopwatch on the same
// instruction, and CAPTURES it in hardware when this board's wake-up
// ISR answers with an edge of its own. Tests h..m then time idle,
// standby and power-down against every clock configuration that
// matters, wake this board on a start bit and on a TWI address match,
// and read the numbers back over the same link.
//
// Reference test of avrdx/sleep.hpp (docs/avrdx/platform.md): keep it
// passing.
//
// Bench diagnostic, NOT a kernel app (sequential, blocking). Console on
// USART2 ALT1 (PF4/PF5) at 460800. This suite OWNS the RTC block: no
// Ticker runs here, because the PIT period is the instrument (125 ms)
// and the counter half is one of the things measured - which is also
// why every bound in the two-board half is a spin count and not a
// millisecond.
//
// WIRING. The single-board half (z) needs NOTHING beyond the desk's
// standing state - but PD1/PD2 MUST BE FREE of the bus jumpers, because
// test e drives PD2 from the event system and senses its own edge on
// the pad. The two-board half (y) needs board B running sleep_peer and
// the standing desk wiring: PE0 the shared one-wire command channel
// (both USART4 TXD pads, LBME), PE2 B's stimulus into this board, PE3
// this board's echo into B's capture, and the office I2C bus on
// PA2/PA3 for test m. PE1 is spare. Event channels 0 (the PIT divider
// to the pad), 1 (the PIT divider into the CCL, test n), 4 and 5 (the
// 32-bit stopwatch's carry and snapshot).
//
// Tests e, f, k, m and n enter POWER-DOWN. Every such sleep is bracketed
// by a watchdog and a .noinit token, so a mode that fails to wake resets
// the board and SAYS SO at the next boot instead of hanging the bench.
//
// Commands: ? | a the register surface | b idle through enter() |
// c standby is real | d the RUNSTDBY chain | e a pin wakes from the
// deep modes | f power-down stops the rest | g the voltage regulator |
// n the CCL as a wake-up source | z = all of those
//   two-board (board B = sleep_peer): h the link and the two wires |
//   i the awake baseline and idle | j standby, timed from outside |
//   k power-down, timed from outside | l start-of-frame | m a TWI
//   address match | y = all of h..m

// build: monitor_speed = 460800

#include <avr/interrupt.h>
#include <stdint.h>

#include "avrdx/ccl.hpp"
#include "avrdx/clock.hpp"
#include "avrdx/delay.hpp"
#include "avrdx/evsys.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/platform_avr.hpp"
#include "avrdx/reset.hpp"
#include "avrdx/rtc.hpp"
#include "avrdx/sleep.hpp"
#include "avrdx/tcb.hpp"
#include "avrdx/twi.hpp"
#include "avrdx/usart.hpp"
#include "avrdx/userrow.hpp"
#include "util/print.hpp"

#include "sleep_link.hpp"

using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
constexpr SysClock clock;

/// The escape hatch of the two power-down tests. A mode that does not
/// wake is caught by the watchdog, and this .noinit word is what tells
/// the next boot which sleep swallowed the program.
///
/// `inline` on purpose: gcc gives an inline variable with a section
/// attribute a COMDAT group and a plain one none, and the two section
/// types cannot merge - AvrPlatform's own panic_record_ is a static
/// inline member in .noinit, so this must be inline too or the link
/// fails.
struct Token {
    uint16_t magic;
    uint8_t stage;      ///< which sleep was in flight (0 = none)
};
[[gnu::section(".noinit")]] inline Token token;

namespace {

using namespace brio;

using P = AvrPlatform;
using Serial = Uart<2, Route::alt1>;
constexpr Serial serial;

constexpr uint16_t token_magic = 0x51EE;

// ---- the instruments -----------------------------------------------------------
// A 32-bit CLK_PER counter (a TCB cascade) is the "did the main clock
// run?" probe; the RTC counter is the "did the 32 kHz domain run?" one;
// the PIT is the waker and, at 4096 cycles of CLK_RTC, the wall clock:
// one period = 125 ms = 3'000'000 CLK_PER ticks at 24 MHz.
using Waker = Tcb<0>;                ///< a wake source that is NOT the PIT
using WatchLo = Tcb<1>;
using WatchHi = Tcb<2>;
using Watch = CascadedCounter<WatchLo, WatchHi>;
using ChCarry = EventChannel<4>;
using ChSnap = EventChannel<5>;
using ChPad = EventChannel<0>;       ///< a PIT divider onto the pad (test e)
using ChCcl = EventChannel<1>;       ///< a PIT divider into a LUT (test n)
using PadPin = Pin<'D', 2>;          ///< EVOUTD, and a fully asynchronous sense pin

// ---- the two-board half's wires and peripherals -----------------------------------
// PE0 is the shared command wire (both boards' USART4 TXD pads, LBME);
// PE2 is board B's stimulus into this chip - and it is a Px2 pin, one of
// the two FULLY ASYNCHRONOUS positions of every port (port.md), which is
// what lets an edge on it wake this device from power-down at all; PE3 is
// this chip's echo back into B's hardware capture.
using U4 = Usart<4>;
using LinePin = Pin<'E', 0>;
using WakePin = Pin<'E', 2>;
using EchoPin = Pin<'E', 3>;
using Client = TwiClient<0, TwiRoute::def>;
using WakeLut = Lut<0>;
constexpr uint8_t client_addr = 0x42;

constexpr uint32_t crystal_hz = SysClock::hz;
constexpr PitPeriod pit_period = PitPeriod::cyc4096;     // 125 ms at 32.768 kHz
constexpr uint16_t pit_cycles_used = 4096;
/// CLK_PER ticks in one PIT period, nominal (OSC32K's own error moves it).
constexpr uint32_t ticks_per_pit = crystal_hz / (32768u / pit_cycles_used);

// ---- shared with the ISRs ---------------------------------------------------------
volatile uint16_t pit_irqs = 0;
volatile uint16_t pad_irqs = 0;
volatile uint16_t waker_irqs = 0;
volatile uint16_t rtc_irqs = 0;
volatile uint16_t wake_irqs = 0;      ///< PORTE: board B's stimulus on PE2
volatile uint16_t ccl_irqs = 0;       ///< the CCL LUT interrupt (test n)
volatile uint16_t sfd_irqs = 0;       ///< USART4 RXSIF: a start bit in standby
volatile uint16_t twis_addr = 0;      ///< TWI client: address matches
volatile uint16_t twis_stops = 0;     ///< TWI client: STOP conditions
volatile uint8_t twis_rx[8];
volatile uint8_t twis_n = 0;

// ---- tiny test harness ------------------------------------------------------------
uint16_t passed = 0, failed = 0;

void verdict(const char* name, bool ok) {
    if (ok) ++passed; else ++failed;
    print(serial, "  ", ok ? "PASS" : "FAIL", "  ", name, crlf);
}
bool near_u32(uint32_t a, uint32_t b, uint32_t tol) {
    return (a > b ? a - b : b - a) <= tol;
}

void console_drain() {
    while (!Serial::tx_idle()) {
    }
    delay_us(clock, 2000);            // the shift register, generously
}

// ---- the stopwatch ----------------------------------------------------------------

/// (Re)build the 32-bit CLK_PER counter. `standby` sets RUNSTDBY on
/// BOTH halves - the cascade task does not expose the flag, so the two
/// configurations are written here through the resource's own config
/// struct (the same two the task writes, plus run_standby).
void watch_init(bool standby) {
    Watch::init(TcbClock::div1, ChCarry{}, ChSnap{});
    if (standby) {
        WatchHi::init({.mode = TcbMode::capture, .clock = TcbClock::event, .compare = 0,
                       .event_input = true, .cascade = true, .run_standby = true});
        WatchLo::init({.mode = TcbMode::capture, .clock = TcbClock::div1, .compare = 0,
                       .event_input = true, .run_standby = true});
    }
    Watch::reset();
}

// ---- the main clock ---------------------------------------------------------------
// Nothing may be printed between these two: the console's baud is
// programmed for 24 MHz and OSCHF is only within a per cent of the
// crystal - good enough to keep a byte, not good enough to trust a
// whole test's output to.

/// CLK_PER from OSCHF at 24 MHz, with the oscillator's own RUNSTDBY as
/// asked.
bool use_oschf(bool osc_standby) {
    (void)Oschf::set_hz(24'000'000);
    Oschf::run_standby(osc_standby);
    return MainClock::select(MainSource::oschf);
}

/// CLK_PER from the crystal again, restarted with the RUNSTDBY flag
/// asked. XOSCHFCTRLA's fields are read-only while the oscillator is
/// enabled, so the crystal is reconfigured from OSCHF and only then
/// selected back.
///
/// BENCH FACT (found here): an oscillator whose RUNSTDBY is CLEAR does
/// not run merely because it is enabled - it runs when something
/// REQUESTS it. Enabling the crystal with RUNSTDBY = 0 while CLK_PER
/// comes from OSCHF therefore leaves MCLKSTATUS.EXTS clear forever, and
/// waiting for it is a dead end. Selecting the crystal IS the request:
/// the switch is what starts it, and MCLKSTATUS.SOSC stays set until it
/// is stable - which is why the wait moves into select() here, with a
/// bound generous enough for the 4096-cycle start-up.
bool use_crystal(bool osc_standby) {
    if (MainClock::source() == MainSource::extclk) {
        if (!use_oschf(true)) return false;
    }
    Xoschf::start_crystal(crystal_hz, XoschfStartup::cycles4k, osc_standby);
    if (osc_standby && !Xoschf::wait_stable()) return false;
    return MainClock::select(MainSource::extclk, 0x00FFFFFFu) && Xoschf::stable();
}

// ---- one sleep, measured ----------------------------------------------------------

struct SleepRun {
    uint32_t ticks;      ///< CLK_PER ticks the stopwatch counted across the sleep
    uint16_t rtc;        ///< RTC counter ticks (32 kHz) across it
    uint16_t pits;       ///< PIT interrupts taken
    uint16_t pads;       ///< PORTD interrupts taken
    uint16_t wakers;     ///< TCB0 interrupts taken
    uint16_t ccls;       ///< CCL interrupts taken (test n)
    uint16_t rtc0;       ///< the RTC counter before the sleep
    uint16_t rtc1;       ///< and immediately after the wake (see test f: stale)
};

/// Wait for the next PIT interrupt: a sleep entered right after one
/// spans a whole period, with no boundary to race.
void pit_sync() {
    const uint16_t p0 = pit_irqs;
    sei();
    for (uint32_t i = 0; i < 4'000'000u && pit_irqs == p0; ++i) {
    }
}

/// Sleep once in `m` and report what moved. `sync` waits for a PIT edge
/// first (test e turns it off: there the PIT interrupt is disabled on
/// purpose). `spin` replaces the sleep with a busy wait for the same
/// wake-up - the baseline every latency figure is measured against.
SleepRun sleep_once(SleepMode m, bool sync = true, bool spin = false) {
    console_drain();
    if (sync) pit_sync();
    SleepRun r{};
    cli();
    const uint32_t t0 = Watch::read();
    const uint16_t c0 = Rtc::count();
    const uint16_t p0 = pit_irqs, d0 = pad_irqs, w0 = waker_irqs, l0 = ccl_irqs;
    sei();
    if (spin) {
        while (pit_irqs == p0 && pad_irqs == d0 && waker_irqs == w0) {
        }
    } else {
        Sleep::enter(m);
    }
    cli();
    r.ticks = Watch::read() - t0;
    r.rtc1 = Rtc::count();
    r.rtc0 = c0;
    r.rtc = static_cast<uint16_t>(r.rtc1 - c0);
    r.pits = static_cast<uint16_t>(pit_irqs - p0);
    r.pads = static_cast<uint16_t>(pad_irqs - d0);
    r.wakers = static_cast<uint16_t>(waker_irqs - w0);
    r.ccls = static_cast<uint16_t>(ccl_irqs - l0);
    sei();
    return r;
}

/// The same, in POWER-DOWN, with the watchdog as the escape hatch: a
/// mode that never wakes resets the board, and `stage` is what the next
/// boot prints instead of the bench hanging.
SleepRun power_down_once(uint8_t stage, bool sync = true) {
    console_drain();
    if (sync) pit_sync();
    token.magic = token_magic;
    token.stage = stage;
    (void)Watchdog::arm(WdtTime::s4);
    const SleepRun r = sleep_once(SleepMode::power_down, false);
    (void)Watchdog::off();
    token.magic = 0;
    token.stage = 0;
    return r;
}

// ---- the state every test starts from ----------------------------------------------

void quiesce() {
    console_drain();                  // it moves the main clock: no byte in flight
    Sleep::disarm();
    (void)Watchdog::off();
    Vreg::power(VregPower::normal);
    (void)Vreg::high_temp_low_leakage(false);
    Ccl::disable();
    WakeLut::disable();
    WakeLut::sense(LutSense::none);
    WakeLut::EventA::unlisten();
    ChCcl::off();
    Twi<1>::client_enable(false);
    Twi<0>::client_enable(false);     // test m's address-match wake source

    EvOut<PadPin>::unlisten();
    ChPad::off();
    PadPin::configure({});
    PadPin::clear_flag();

    // The two-board half's pins: the stimulus stops being a wake-up
    // source, the echo parks low. Nothing here touches USART4 - the
    // command channel is the two-board tests' own business.
    WakePin::configure({});
    EchoPin::clear();
    EchoPin::output();
    (void)Port<'E'>::take_flags();

    (void)use_crystal(true);          // the board's boot configuration

    Waker::enable_capt_interrupt(false);
    Waker::disable();
    Rtc::disable();
    Rtc::enable_ovf_interrupt(false);
    Rtc::enable_cmp_interrupt(false);
    RtcClock::select(RtcSource::osc32k);
    Pit::init(pit_period);            // enabled, interrupt on: the waker
    watch_init(false);
    rtc_irqs = 0;
    waker_irqs = rtc_irqs;
    pad_irqs = waker_irqs;
    pit_irqs = pad_irqs;
    sfd_irqs = 0;
    ccl_irqs = sfd_irqs;
    wake_irqs = ccl_irqs;
    twis_stops = 0;
    twis_addr = twis_stops;
    twis_n = 0;
    sei();
}

// ---- a the register surface ----------------------------------------------------------
// No sleeping here: what a program can ask SLPCTRL for, and the two
// rules the driver enforces rather than merely documents.
void ta_surface() {
    print(serial, "a SLPCTRL and VREGCTRL: the surface, the NOP and the HTLLEN interlock", crlf);
    quiesce();

    verdict("nothing is armed at rest", !Sleep::armed());
    Sleep::arm(SleepMode::idle);
    verdict("arm(idle): SEN set, SMODE reads back idle",
            Sleep::armed() && Sleep::armed_mode() == SleepMode::idle);
    Sleep::arm(SleepMode::standby);
    verdict("arm(standby) reads back",
            Sleep::armed() && Sleep::armed_mode() == SleepMode::standby);
    Sleep::arm(SleepMode::power_down);
    verdict("arm(power_down) reads back",
            Sleep::armed() && Sleep::armed_mode() == SleepMode::power_down);
    Sleep::disarm();
    verdict("disarm() clears SEN", !Sleep::armed());
    verdict("and the platform's own readback agrees", !P::sleep_armed());

    // Errata 2.2.4: a store to an address >= 0x40 immediately followed
    // by a write to SLPCTRL.CTRLA loses that write. Every store in
    // sleep.hpp is preceded by the documented NOP, so the sequence
    // below - a TCB counter write (0x0A1A) and an arm() with nothing in
    // between - must still arm. (That the NOP is really adjacent to the
    // store is a property of the generated code, checked in
    // firmware.lst, not from here.)
    Waker::count(0);
    Sleep::arm(SleepMode::standby);
    verdict("a store above 0x40 immediately before arm() does not eat it "
            "(erratum 2.2.4's NOP)",
            Sleep::armed() && Sleep::armed_mode() == SleepMode::standby);
    Waker::count(0);
    Sleep::disarm();
    verdict("and the same holds for disarm()", !Sleep::armed());

    // VREGCTRL is the block's one CCP-protected register.
    verdict("the regulator starts in its Normal profile", Vreg::power() == VregPower::normal);
    Vreg::power(VregPower::performance);
    verdict("PMODE = performance reads back (a CCP write really landed)",
            Vreg::power() == VregPower::performance);
    Vreg::power(VregPower::normal);
    verdict("and back to normal", Vreg::power() == VregPower::normal);

    verdict("HTLLEN starts clear", !Vreg::high_temp_low_leakage());
    verdict("with no TWI client and no CCL, enabling HTLLEN is accepted",
            Vreg::high_temp_low_leakage(true) && Vreg::high_temp_low_leakage());
    verdict("PMODE survived the HTLLEN write", Vreg::power() == VregPower::normal);
    verdict("and disabling it is accepted",
            Vreg::high_temp_low_leakage(false) && !Vreg::high_temp_low_leakage());

    // The interlock, both halves. The CCL is enabled with no LUT
    // configured (it touches no pin at all); the TWI half is exercised
    // on TWI1, whose default pins PF2/PF3 go nowhere on this board -
    // the desk's I2C bus is TWI0's and is never touched here.
    Ccl::enable();
    verdict("the CCL is enabled", Ccl::enabled());
    verdict("HTLLEN is REFUSED while the CCL could wake the device",
            !Vreg::high_temp_low_leakage(true));
    verdict("and nothing was written", !Vreg::high_temp_low_leakage());
    Ccl::disable();
    verdict("with the CCL off it is accepted again", Vreg::high_temp_low_leakage(true));
    (void)Vreg::high_temp_low_leakage(false);

    Twi<1>::client_enable(true);
    verdict("a TWI client is enabled", Twi<1>::client_enabled());
    verdict("HTLLEN is REFUSED while a TWI address match could wake the device",
            !Vreg::high_temp_low_leakage(true));
    verdict("and nothing was written", !Vreg::high_temp_low_leakage());
    Twi<1>::client_enable(false);
    verdict("with the client off it is accepted again", Vreg::high_temp_low_leakage(true));
    verdict("disabling HTLLEN is never refused", Vreg::high_temp_low_leakage(false));
    print(serial, "  the interlock is chapter 13's own warning made enforceable: with "
                  "HTLLEN set, the TWI address match and the CCL are not wake-up "
                  "sources any more.", crlf);
    quiesce();
}

// ---- b IDLE through the new verb ------------------------------------------------------
// The platform's idle() is the kernel's hook and owns the six-cycle
// measurement (test_avr_platform f). What is proven here is that
// Sleep::enter(idle) is the same thing under an application's control:
// it stops the CPU, comes back on an interrupt, and disarms itself.
void tb_idle() {
    print(serial, "b IDLE through Sleep::enter(): the CPU stops, an interrupt returns it", crlf);
    quiesce();

    // A TCB alarm every 2 ms is the wake source: the PIT's 125 ms would
    // make the loop below take a quarter of a minute.
    verdict("the 2 ms alarm started", PeriodicTick<Waker>::init(clock, 500u));
    Pit::enable_interrupt(false);            // one wake source at a time

    console_drain();
    uint32_t asleep_turns = 0, awake_turns = 0;
    bool woke = true, disarmed = true, ints_on = true;
    constexpr uint8_t n = 16;
    for (uint8_t i = 0; i < n; ++i) {
        cli();
        const uint16_t w0 = waker_irqs;
        sei();
        Sleep::enter(SleepMode::idle);
        if (Sleep::armed()) disarmed = false;
        if (!P::interrupts_enabled()) ints_on = false;
        cli();
        if (waker_irqs == w0) woke = false;
        sei();
        ++asleep_turns;
    }
    verdict("enter(idle) returned only after an interrupt fired", woke);
    verdict("SLPCTRL is disarmed on return", disarmed);
    verdict("interrupts are still enabled on return", ints_on);
    verdict("that is 16 sleeps", asleep_turns == n);

    // The CPU really stopped: over the same span (16 alarms) a counter
    // that only advances while the CPU runs turns once per wake asleep
    // and thousands of times awake.
    volatile uint32_t work = 0;
    cli();
    uint16_t w0 = waker_irqs;
    sei();
    while (static_cast<uint16_t>(waker_irqs - w0) < n) {
        work = work + 1;
    }
    awake_turns = work;
    print(serial, "  over 16 alarm periods: ", asleep_turns, " loop turns asleep (one per "
                  "wake), ", awake_turns, " awake", crlf);
    verdict("the loop is frozen while the CPU sleeps", asleep_turns <= n);
    verdict("the same span awake turns it thousands of times",
            awake_turns > 100u * asleep_turns);

    // Parity with the kernel's own hook, on the same alarm.
    const uint32_t before = Watch::read();
    P::idle();
    const uint32_t after = Watch::read();
    verdict("the platform's idle() still works next to the new verb", after != before);
    verdict("and leaves SLPCTRL disarmed too", !P::sleep_armed());
    quiesce();
}

// ---- c standby is real -----------------------------------------------------------------
void tc_standby() {
    print(serial, "c STANDBY: CLK_PER stops, the PIT still wakes", crlf);
    quiesce();
    // The stopwatch has no RUNSTDBY and the crystal has no requester
    // asking for it: nothing should count.
    const SleepRun spun = sleep_once(SleepMode::standby, true, true);
    const SleepRun slept = sleep_once(SleepMode::standby);

    print(serial, "  one PIT period (", pit_cycles_used, " CLK_RTC cycles = 125 ms): "
                  "awake the stopwatch counted ", spun.ticks, " CLK_PER ticks, asleep ",
          slept.ticks, " (nominal ", ticks_per_pit, " per period)", crlf);
    verdict("the PIT interrupt is what came back", slept.pits == 1);
    verdict("awake, the stopwatch counts the whole period",
            near_u32(spun.ticks, ticks_per_pit, ticks_per_pit / 8));
    verdict("asleep, it counts almost nothing: CLK_PER was gone",
            slept.ticks < ticks_per_pit / 100);
    print(serial, "  what it did count is the tail after the wake-up: ", slept.ticks,
          " ticks = ", (slept.ticks * 1000u) / (crystal_hz / 1000u), " us of code "
          "between the wake and the read", crlf);
    verdict("standby left SLPCTRL disarmed", !Sleep::armed());
    quiesce();
}

// ---- d the RUNSTDBY chain ---------------------------------------------------------------
// RUNSTDBY on a peripheral asks for its clock to be kept alive in
// standby; each oscillator has a RUNSTDBY of its own. Which of the two
// decides? Both ends are swept here - two sources x the oscillator flag
// x the peripheral flag - and every leg is one 125 ms sleep with the
// PIT as the waker. The source column is read back from CLKCTRL at the
// moment of the measurement, so a clock move that did not happen shows
// up in the table instead of being assumed.
struct Leg {
    const char* what;
    uint32_t ticks;
    uint8_t source;
};

const char* source_name(uint8_t s) {
    return s == static_cast<uint8_t>(MainSource::oschf) ? "OSCHF"
         : s == static_cast<uint8_t>(MainSource::extclk) ? "XOSCHF"
         : "other";
}

void td_chain() {
    print(serial, "d the RUNSTDBY chain: peripheral flag x oscillator flag x source", crlf);
    quiesce();
    Leg legs[5];
    auto leg = [](const char* what) -> Leg {
        const uint32_t t = sleep_once(SleepMode::standby).ticks;
        return {what, t, static_cast<uint8_t>(MainClock::source())};
    };

    // 1. the baseline of test c, on the crystal: no peripheral request.
    watch_init(false);
    legs[0] = leg("crystal (RUNSTDBY on) + TCB RUNSTDBY off");

    // 2. the peripheral asks; the crystal was started with its own
    //    RUNSTDBY set (that is what Clock<crystal>::init does).
    watch_init(true);
    legs[1] = leg("crystal (RUNSTDBY on) + TCB RUNSTDBY on");

    // 3. the same, with the crystal's own RUNSTDBY cleared.
    console_drain();
    const bool re_xtal = use_crystal(false);
    watch_init(true);
    legs[2] = leg("crystal (RUNSTDBY off) + TCB RUNSTDBY on");

    // 4./5. the same two on OSCHF. The console is silent across both
    //       switches: its baud belongs to the crystal.
    console_drain();
    const bool on_oschf = use_oschf(false);
    watch_init(true);
    legs[3] = leg("OSCHF (RUNSTDBY off) + TCB RUNSTDBY on");
    const bool oschf_std = use_oschf(true);
    watch_init(true);
    legs[4] = leg("OSCHF (RUNSTDBY on) + TCB RUNSTDBY on");

    // A wake source that is NOT the PIT: a TCB interrupt, from a TCB
    // whose chain is complete. Table 13-4 lists TCB among standby's
    // wake-up sources - for the instances left running.
    (void)Waker::init({.mode = TcbMode::periodic, .clock = TcbClock::div1,
                       .compare = 48000, .run_standby = true});     // 2 ms
    Waker::enable_capt_interrupt(true);
    Pit::enable_interrupt(false);
    const SleepRun by_tcb = sleep_once(SleepMode::standby, false);
    Pit::enable_interrupt(true);
    Waker::enable_capt_interrupt(false);
    Waker::disable();

    // And the same TCB with its RUNSTDBY cleared: it cannot wake what
    // it cannot count. The PIT is the backstop that ends the sleep.
    (void)Waker::init({.mode = TcbMode::periodic, .clock = TcbClock::div1,
                       .compare = 48000});
    Waker::enable_capt_interrupt(true);
    const SleepRun by_pit = sleep_once(SleepMode::standby);
    Waker::enable_capt_interrupt(false);
    Waker::disable();

    console_drain();
    const bool back = use_crystal(true);

    verdict("every clock move of the sweep landed",
            re_xtal && on_oschf && oschf_std && back);
    for (const Leg& l : legs) {
        print(serial, "  ", l.what, ": ", l.ticks, " CLK_PER ticks in 125 ms (nominal ",
              ticks_per_pit, ", source in force ", source_name(l.source), ")", crlf);
    }
    bool counted[5];
    for (uint8_t i = 0; i < 5; ++i) {
        counted[i] = near_u32(legs[i].ticks, ticks_per_pit, ticks_per_pit / 8);
    }
    verdict("without the peripheral's RUNSTDBY nothing counts, whatever the "
            "oscillator does", !counted[0]);
    verdict("with BOTH ends armed the counter runs through the sleep",
            counted[1] && counted[4]);
    // The measured rule, and it is NOT symmetric: the PERIPHERAL's flag
    // decides. Its request is what keeps CLK_PER - and the oscillator
    // behind it - alive in standby; the oscillator's own RUNSTDBY adds
    // nothing while something is requesting it (what it does buy shows
    // up in test g, as wake-up latency).
    verdict("the peripheral's RUNSTDBY ALONE is enough on the crystal: its "
            "request keeps the oscillator running", counted[2]);
    verdict("and the same on OSCHF", counted[3]);
    verdict("the two sources are told apart by the tick count itself (this "
            "board's OSCHF is the slower of the two)", legs[4].ticks < legs[1].ticks);
    verdict("a TCB whose chain is complete wakes the CPU from standby by itself",
            by_tcb.wakers >= 1 && by_tcb.pits == 0);
    verdict("a TCB whose RUNSTDBY is clear cannot: the PIT had to end that sleep",
            by_pit.wakers == 0 && by_pit.pits == 1);
    print(serial, "  the TCB wake ended after ", by_tcb.ticks, " CLK_PER ticks (its 2 ms "
                  "period is 48000)", crlf);
    quiesce();
}

// ---- e a pin wakes from the deep modes ---------------------------------------------------
// Zero wires: EVSYS is alive in every sleep mode (table 13-2), so a PIT
// divider routed to EVOUT on PD2 drives that pad while the CPU sleeps,
// and PD2's own sense (Px2 is one of the two fully asynchronous pins of
// every port) sees the edge the device made for itself.
void te_pin_wake() {
    print(serial, "e a PORT pin wakes from standby and from power-down (no wires: the "
                  "PIT drives PD2 through EVOUT)", crlf);
    quiesce();

    ChPad::source(EvPitDiv<1024>{});          // 32 Hz square: an edge every 15.6 ms
    PadPin::configure({.sense = PinSense::both});
    EvOut<PadPin>::listen(ChPad{});
    Pit::enable_interrupt(false);             // the PIT must not be the one waking us
    PadPin::clear_flag();

    console_drain();
    delay_us(clock, 40'000u);                 // an edge has certainly passed
    const uint16_t d0 = pad_irqs;
    verdict("the pad really toggles: its interrupt fires with the CPU awake", d0 > 0);

    const SleepRun standby = sleep_once(SleepMode::standby, false);
    verdict("standby was ended by the pin, not by anything else",
            standby.pads >= 1 && standby.pits == 0 && standby.wakers == 0);
    print(serial, "  standby: ", standby.pads, " pad interrupt(s), stopwatch ",
          standby.ticks, " ticks (frozen)", crlf);

    // Power-down. The PIT interrupt goes back on as a BACKSTOP with a
    // long period (1 s): if the pad were dead in this mode the sleep
    // would still end, and the two counters say which source did it.
    Pit::period(PitPeriod::cyc32768);
    Pit::enable_interrupt(true);
    PadPin::clear_flag();
    const SleepRun pd = power_down_once(1, false);
    print(serial, "  power-down: ", pd.pads, " pad interrupt(s), ", pd.pits,
          " PIT interrupt(s) of the 1 s backstop, stopwatch ", pd.ticks, " ticks", crlf);
    verdict("power-down was ended by the pin, before the 1 s backstop could fire",
            pd.pads >= 1 && pd.pits == 0);
    print(serial, "  so the event system really drives a pad while every clock but the "
                  "32 kHz one is stopped, and a fully asynchronous pin sense picks it "
                  "up from power-down.", crlf);
    quiesce();
}

// ---- f power-down stops the rest -----------------------------------------------------------
// Both chains complete: OSCHF with RUNSTDBY under a TCB with RUNSTDBY,
// and the RTC counter with RUNSTDBY on the 32 kHz domain. In standby
// both count; in power-down neither the counter nor CLK_PER does, and
// the PIT alone comes back.
//
// One bench fact shapes the measurement: RTC.CNT read IMMEDIATELY after
// a wake still carries the value it had when the CPU went to sleep. The
// read is synchronized into CLK_PER (26.10) and that path needs the
// clock back plus a CLK_RTC edge, so the counter is read here after a
// settling delay - the raw read is kept and printed as its own finding.
void tf_power_down() {
    print(serial, "f POWER-DOWN: only the PIT survives - the RTC COUNTER does not", crlf);
    quiesce();

    (void)Rtc::init({.prescaler = RtcPrescaler::div1, .period = 0xFFFF,
                     .run_standby = true});
    console_drain();
    (void)use_oschf(true);
    watch_init(true);

    const SleepRun standby = sleep_once(SleepMode::standby);
    delay_us(clock, 1000);
    const uint16_t standby_settled = static_cast<uint16_t>(Rtc::count() - standby.rtc0);
    const SleepRun pd = power_down_once(2);
    delay_us(clock, 1000);
    const uint16_t pd_settled = static_cast<uint16_t>(Rtc::count() - pd.rtc0);

    console_drain();
    (void)use_crystal(true);

    print(serial, "  standby:    RTC counter +", standby.rtc, " ticks read at once, +",
          standby_settled, " read 1 ms later; stopwatch +", standby.ticks,
          " CLK_PER ticks", crlf);
    print(serial, "  power-down: RTC counter +", pd.rtc, " / +", pd_settled,
          " the same two ways; stopwatch +", pd.ticks, " ticks; PIT interrupts ",
          pd.pits, crlf);
    verdict("in standby the RTC counter runs (RUNSTDBY): one PIT period of "
            "CLK_RTC ticks", near_u32(standby_settled, pit_cycles_used, 256));
    verdict("in standby a fully armed chain keeps CLK_PER alive",
            near_u32(standby.ticks, ticks_per_pit, ticks_per_pit / 8));
    verdict("in power-down the RTC COUNTER stops even with RUNSTDBY set",
            pd_settled < 256);
    verdict("in power-down CLK_PER stops whatever any RUNSTDBY says",
            pd.ticks < ticks_per_pit / 100);
    verdict("and the PIT half of the same block still fires", pd.pits == 1);
    verdict("CNT read at the instant of the wake is STALE: it still reads the "
            "value it had at the SLEEP instruction", standby.rtc < 8);
    // The two counters measure the same interval in two clock domains,
    // which is what makes OSC32K's own error a number here.
    const uint32_t rtc_hz = standby.ticks
        ? static_cast<uint32_t>((static_cast<uint64_t>(pit_cycles_used) * 24'000'000u) /
                                standby.ticks)
        : 0;
    print(serial, "  the same 4096 CLK_RTC cycles measured ", standby.ticks,
          " CLK_PER ticks, so CLK_RTC ran at ", rtc_hz, " Hz (nominal 32768)", crlf);
    print(serial, "  the wake-up latency out of power-down is NOT measurable from inside "
                  "this chip: the only clock that survives the mode is the PIT's, and the "
                  "counter that could time the restart is exactly the one it stops.", crlf);
    quiesce();
}

// ---- g the voltage regulator ----------------------------------------------------------------
// What one board can say about VREGCTRL: that PMODE is a configuration
// and not a pulse (it survives a sleep), that HTLLEN does not cost the
// PIT its wake-up, and how long a wake-up out of standby takes. The
// 32 kHz domain is the only clock that keeps running across a standby
// sleep, so the RTC counter is the only ruler available: one tick is
// 30.5 us.
//
// The ruler: CNT is zeroed just after a PIT interrupt, so the NEXT one
// falls at CNT = 4096 plus whatever the wake-up cost. CNT is read after
// a fixed settling delay (test f: the read path is stale at the instant
// of the wake), and the same sequence measured AWAKE is the baseline
// that removes the settle and the cost of zeroing CNT. What the
// difference then shows is what the sleep itself cost - which is where
// an oscillator's own RUNSTDBY finally earns its keep: it is not what
// keeps a peripheral counting (test d), it is what saves the restart.
void tg_vreg() {
    print(serial, "g VREGCTRL: the profile across a sleep, HTLLEN with the PIT, and the "
                  "wake-up delay in 32 kHz ticks", crlf);
    quiesce();
    (void)Rtc::init({.prescaler = RtcPrescaler::div1, .period = 0xFFFF,
                     .run_standby = true});

    auto wake_ticks = [](bool spin) -> int16_t {
        console_drain();
        pit_sync();
        cli();
        Rtc::count(0);
        sei();
        (void)sleep_once(SleepMode::standby, false, spin);
        delay_us(clock, 1000);
        return static_cast<int16_t>(static_cast<int16_t>(Rtc::count()) -
                                    static_cast<int16_t>(pit_cycles_used));
    };

    // The sweep: the baseline, then the two sources x the oscillator's
    // own RUNSTDBY x the regulator's profile. The pairs that matter are
    // the ones where the oscillator really has to start again - that is
    // where both flags can show what they cost.
    const int16_t awake = wake_ticks(true);
    Vreg::power(VregPower::normal);
    const int16_t xtal_on = wake_ticks(false);
    Vreg::power(VregPower::performance);
    const int16_t xtal_on_perf = wake_ticks(false);
    verdict("PMODE survived a standby sleep", Vreg::power() == VregPower::performance);

    // The crystal with its own RUNSTDBY cleared: nothing requests it
    // while the CPU sleeps, so it stops and has to start again.
    console_drain();
    const bool xtal_free = use_crystal(false);
    Vreg::power(VregPower::normal);
    const int16_t xtal_off = wake_ticks(false);
    Vreg::power(VregPower::performance);
    const int16_t xtal_off_perf = wake_ticks(false);

    console_drain();
    const bool on_oschf = use_oschf(true);
    Vreg::power(VregPower::normal);
    const int16_t oschf_on = wake_ticks(false);
    const bool oschf_free = use_oschf(false);
    const int16_t oschf_off = wake_ticks(false);
    Vreg::power(VregPower::performance);
    const int16_t oschf_off_perf = wake_ticks(false);
    console_drain();
    Vreg::power(VregPower::normal);
    const bool back = use_crystal(true);

    verdict("every clock move of the sweep landed",
            xtal_free && on_oschf && oschf_free && back);
    print(serial, "  CNT - 4096 after the wake, in CLK_RTC ticks of 30.5 us (awake ",
          awake, " is the baseline: the settle and the CNT zeroing)", crlf);
    print(serial, "    crystal, its RUNSTDBY on:  ", xtal_on, " normal, ", xtal_on_perf,
          " performance", crlf);
    print(serial, "    crystal, its RUNSTDBY off: ", xtal_off, " normal, ", xtal_off_perf,
          " performance", crlf);
    print(serial, "    OSCHF,   its RUNSTDBY on:  ", oschf_on, " normal", crlf);
    print(serial, "    OSCHF,   its RUNSTDBY off: ", oschf_off, " normal, ", oschf_off_perf,
          " performance", crlf);
    verdict("waking from standby never costs less than staying awake",
            xtal_on >= awake && xtal_on_perf >= awake && xtal_off >= awake &&
                xtal_off_perf >= awake && oschf_on >= awake && oschf_off >= awake &&
                oschf_off_perf >= awake);
    verdict("an oscillator kept alive by its OWN RUNSTDBY costs nothing to wake "
            "from", xtal_on <= awake + 2 && oschf_on <= awake + 2);
    verdict("one that has to start again costs real time, and the crystal costs "
            "much more than OSCHF", xtal_off > oschf_off && oschf_off > oschf_on);
    print(serial, "  that is the whole point of an oscillator's RUNSTDBY: test d showed "
                  "it is not what keeps a peripheral counting - this is what it buys.",
          crlf);
    print(serial, "  the regulator's profile moves the restarting cases by ",
          xtal_off - xtal_off_perf, " (crystal) and ", oschf_off - oschf_off_perf,
          " (OSCHF) ticks; a difference of zero or one tick is below this ruler's "
          "resolution and is reported, not claimed.", crlf);
    // The one place a single board can SEE the regulator: an OSCHF
    // restart out of standby is two costs in series, the oscillator's
    // and the regulator's, and PMODE = FULL pays the second one in
    // advance. On the crystal the oscillator's own start-up buries it.
    verdict("PMODE = performance really shortens a wake-up that has to restart "
            "OSCHF", oschf_off - oschf_off_perf >= 4);
    verdict("and it cannot shorten the crystal's own start-up",
            xtal_off_perf > oschf_off_perf);

    // HTLLEN shortens power-down's wake-up list to the PORT pin, BOD
    // VLM, MVIO and the PIT. The PIT half of that is testable here.
    verdict("HTLLEN can be set (no TWI client, no CCL)", Vreg::high_temp_low_leakage(true));
    const SleepRun pd = power_down_once(3);
    verdict("with HTLLEN set, the PIT still wakes the device from power-down",
            pd.pits == 1);
    verdict("and HTLLEN is still set on the way out", Vreg::high_temp_low_leakage());
    verdict("clearing it is accepted", Vreg::high_temp_low_leakage(false));
    print(serial, "  the other half of HTLLEN - that the TWI address match and the CCL "
                  "stop waking the device - is not testable from one board: it is an "
                  "ABSENCE, and the driver refuses the combination that would need it.",
          crlf);
    quiesce();
}

// ==============================================================================
//  THE TWO-BOARD HALF (h..m, set y)
//
//  Board B runs sleep_peer and is driven IN BAND over the shared PE0
//  wire (src/apps/sleep_link.hpp: magic, opcode, checksum, ack before
//  act, every action bounded and self-restoring). The command channel is
//  8N1 at slink::command_baud with LBME at both ends: this board takes
//  the wire only to talk and hands it back to the pull-up to listen.
//
//  THE MEASUREMENT, once, so no test has to re-explain it:
//
//    B zeroes a 32-bit CLK_PER stopwatch and raises PE2 in the same
//    instruction pair. That edge is what has to wake this chip. This
//    chip's PORTE ISR raises PE3 as its first statement; on B that edge
//    is an event that latches the stopwatch into a capture register.
//    The number that comes back is therefore
//
//      B's stimulus -> this chip's wake-up -> ISR prologue -> the store
//
//    in B's CLK_PER ticks (24 MHz nominal from ITS OSCHF - a per-cent
//    class reference, ample for microsecond-to-millisecond figures). The
//    fixed part - B's zero-to-edge gap, the wire, this chip's vector
//    latency and ISR prologue - is measured AWAKE in test i and is the
//    baseline every sleep figure is read against.
// ==============================================================================

const uint8_t no_payload[1] = {0};
slink::Decoder dec;

/// This suite owns the RTC block, so there is no Ticker to ask the time
/// of: every bound in the link client is a spin count. One iteration of
/// the polling loops below is a handful of cycles at 24 MHz, so this is
/// a generous "at least a millisecond" - a bound only has to outlast the
/// peer's own deadline, never to be accurate.
constexpr uint32_t spins_per_ms = 2000;

void put4(uint8_t b) { (void)U4::send(b, 200'000u); }

/// Half-duplex turnaround guard. RXCIF is raised at the MIDDLE of the
/// stop bit - half a bit BEFORE the sender's TXCIF - so an end that
/// answers at once starts its start bit while the other end is still
/// transmitting with its receiver off. Four bit times, from the BAUD
/// register in force.
void turnaround_guard() {
    const uint32_t bit_cycles =
        (static_cast<uint32_t>(U4::baud_reg()) * U4::samples()) / 64u;
    delay_cycles(4u * (bit_cycles ? bit_cycles : 1u));
}

/// Take the shared wire: receiver off (LBME would otherwise decode our
/// own echo), pin driven, transmitter on.
void line_talk() {
    turnaround_guard();
    U4::enable_rx(false);
    LinePin::set();
    LinePin::output();
    U4::enable_tx(true);
    U4::clear_txc();            // so line_listen waits for THIS burst
}

/// Give it back to the pull-up and listen again. It OWNS the wait for
/// the line to go idle: TXCIF is write-one-to-clear, so a caller that
/// waits first and then calls this would spin out its budget deaf.
void line_listen(bool wait = true) {
    if (wait) (void)U4::wait_line_idle(200'000u);
    U4::enable_tx(false);
    LinePin::input();
    pinctrl_of('E', 0) |= PORT_PULLUPEN_bm;
    U4::flush_rx();
    U4::enable_rx(true);
}

/// Command mode, the state everything returns to: 8N1 at the command
/// baud, receiver listening to the TXD pad through LBME, the pad itself
/// released to the pull-up.
bool link_command_mode() {
    const bool ok = U4::init({.route = UsartRoute::def,
                              .baud = usart_baud_reg(SysClock::hz, slink::command_baud),
                              .tx = false,
                              .loop_back = true});
    LinePin::input();
    pinctrl_of('E', 0) |= PORT_PULLUPEN_bm;
    U4::flush_rx();
    U4::clear_txc();
    dec.reset();
    return ok;
}

bool link_quiet = false;      ///< suppress the failure dump while probing

bool recv_frame(slink::Frame& out, uint16_t ms) {
    dec.reset();
    const uint32_t budget = static_cast<uint32_t>(ms) * spins_per_ms;
    for (uint32_t i = 0; i < budget; ++i) {
        if (!U4::rxc_flag()) continue;
        const UsartFrame f = U4::receive();
        if (!f.clean()) { dec.reset(); continue; }
        if (dec.feed(static_cast<uint8_t>(f.data)) == slink::Decoder::Result::frame) {
            out = dec.frame();
            return true;
        }
    }
    return false;
}

bool command_once(slink::Op op, const uint8_t* p, uint8_t len, uint16_t ms) {
    U4::flush_rx();
    line_talk();
    slink::write_frame(put4, op, p, len);
    line_listen();
    slink::Frame f;
    if (!recv_frame(f, ms)) return false;
    return f.op == slink::Op::ack && f.len == 2 && f.data[0] == slink::byte_of(op);
}

/// The recovery guarantee in action: three attempts, each separated by a
/// quiet interval longer than the peer's own reassembly timeout.
bool command(slink::Op op, const uint8_t* p = no_payload, uint8_t len = 0,
             uint16_t ms = 80) {
    for (uint8_t k = 0; k < 3; ++k) {
        if (command_once(op, p, len, ms)) return true;
        (void)link_command_mode();
        delay_us(clock, 70'000);
    }
    if (!link_quiet) {
        print(serial, "    LINK FAILURE op ", hex(slink::byte_of(op)),
              ": board B did not acknowledge. It is sleep_peer on the shared PE0 wire; "
              "'0' on its console forces command mode.", crlf);
    }
    (void)link_command_mode();
    delay_us(clock, 70'000);
    return false;
}

bool query(slink::Op op, slink::Frame& data, const uint8_t* p = no_payload,
           uint8_t len = 0, uint16_t ms = 80) {
    if (!command(op, p, len, ms)) return false;
    return recv_frame(data, ms);
}

bool ensure_link() {
    link_quiet = true;
    (void)link_command_mode();
    const bool up = command(slink::Op::ping);
    link_quiet = false;
    if (!up) {
        print(serial, "  board B did not answer on the shared PE0 wire. It must be "
                      "running sleep_peer; '0' on its console forces command mode.", crlf);
    }
    return up;
}

bool peer_ident(slink::Ident& d) {
    slink::Frame f;
    if (!query(slink::Op::ident, f) || f.op != slink::Op::ident_data ||
        f.len != slink::ident_size) {
        return false;
    }
    d = slink::get_ident(f.data);
    return true;
}

/// Ask for the report of the last action, after giving the peer's bound
/// time to expire.
bool peer_report(slink::Report& r, uint16_t wait_ms) {
    (void)link_command_mode();
    if (wait_ms) delay_us(clock, static_cast<uint32_t>(wait_ms) * 1000u);
    for (uint8_t k = 0; k < 4; ++k) {
        slink::Frame f;
        if (query(slink::Op::report, f) && f.op == slink::Op::report_data &&
            f.len == slink::report_size) {
            r = slink::get_report(f.data);
            return true;
        }
        delay_us(clock, 40'000);
    }
    return false;
}

/// Command an action and wait out the rendezvous: the peer reconfigures
/// as soon as its acknowledgement has left the line.
bool peer_act(slink::Op op, const slink::Params& a) {
    uint8_t p[slink::params_size] = {};
    slink::put_params(p, a);
    if (!command(op, p, slink::params_size)) return false;
    delay_us(clock, static_cast<uint32_t>(slink::settle_ms) * 1000u);
    return true;
}

// ---- one measured train ---------------------------------------------------------

struct Train {
    uint32_t ticks[slink::max_shots] = {};
    uint8_t n = 0;          ///< values fetched
    uint16_t hits = 0;      ///< echoes the peer captured
    uint16_t woke = 0;      ///< stimuli this board answered
};

uint16_t train_woke = 0;

/// Arm the wires and tell the peer to fire `shots` stimuli. It waits
/// `lead_ms` before the first one, which is the window this board has to
/// move its main clock and get into the answering loop.
bool train_arm(uint8_t shots, uint16_t lead_ms = 300, uint16_t period_ms = 100,
               uint16_t deadline_ms = 300) {
    EchoPin::clear();
    EchoPin::output();
    WakePin::configure({.sense = PinSense::rising});
    WakePin::clear_flag();
    (void)Port<'E'>::take_flags();
    cli();
    wake_irqs = 0;
    sei();
    slink::Params a{};
    a.count = shots;
    a.delay_ms = lead_ms;
    a.period_ms = period_ms;
    a.deadline_ms = deadline_ms;
    a.hold_us = 300;
    return peer_act(slink::Op::pulse, a);
}

/// Answer every stimulus of the train, either awake or from `m`. The PIT
/// is the backstop: a stimulus that never arrives costs at most a few of
/// its periods, and its own wake-ups are simply slept through again.
void train_answer(bool sleeping, SleepMode m, uint8_t shots) {
    train_woke = 0;
    for (uint8_t i = 0; i < shots; ++i) {
        cli();
        const uint16_t d0 = wake_irqs;
        sei();
        bool got = false;
        for (uint8_t k = 0; k < 8 && !got; ++k) {
            if (sleeping) {
                Sleep::enter(m);
            } else {
                for (uint32_t s = 0; s < 1'500'000u && wake_irqs == d0; ++s) {
                }
            }
            cli();
            got = wake_irqs != d0;
            sei();
        }
        if (got) ++train_woke;
        // The echo has to stay up long enough for the peer's event path
        // to latch it, and then go back down: the next shot needs a
        // RISING edge on the same wire.
        delay_us(clock, 50);
        EchoPin::clear();
    }
}

/// Fetch the peer's numbers. Must run with the main clock back on the
/// crystal - the command channel's BAUD belongs to it.
bool train_collect(Train& t) {
    t.n = 0;
    t.woke = train_woke;
    slink::Report r{};
    if (!peer_report(r, 20)) return false;
    t.hits = r.hits;
    const uint8_t want = r.count > slink::max_shots ? slink::max_shots
                                                    : static_cast<uint8_t>(r.count);
    while (t.n < want) {
        const uint8_t p[1] = {t.n};
        slink::Frame f;
        if (!query(slink::Op::gaps, f, p, 1) || f.op != slink::Op::gaps_data ||
            f.len < slink::gaps_header) {
            return false;
        }
        const uint8_t start = f.data[0], k = f.data[1];
        if (k == 0 || start != t.n) return false;
        for (uint8_t i = 0; i < k && t.n < slink::max_shots; ++i) {
            t.ticks[t.n++] = slink::get32(f.data + slink::gaps_header + 4u * i);
        }
    }
    return true;
}

/// The middle value of the shots that were captured, or slink::no_capture
/// when none was. A median, not a mean: one PIT wake-up landing on top of
/// a stimulus must not move the figure the verdict reads.
uint32_t median_of(const Train& t) {
    uint32_t s[slink::max_shots];
    uint8_t k = 0;
    for (uint8_t i = 0; i < t.n; ++i) {
        if (t.ticks[i] != slink::no_capture) s[k++] = t.ticks[i];
    }
    if (k == 0) return slink::no_capture;
    for (uint8_t i = 1; i < k; ++i) {
        const uint32_t x = s[i];
        uint8_t j = i;
        while (j > 0 && s[j - 1] > x) { s[j] = s[j - 1]; --j; }
        s[j] = x;
    }
    return s[k / 2];
}

/// B's CLK_PER ticks, in the unit that reads best. Its clock is OSCHF at
/// 24 MHz nominal, so one tick is 41.7 ns.
void print_ticks(uint32_t ticks) {
    if (ticks == slink::no_capture) { print(serial, "MISS"); return; }
    print(serial, ticks, " ticks = ");
    if (ticks < 24'000u) print(serial, (ticks * 125u) / 3u, " ns");
    else print(serial, ticks / 24u, " us");
}

void print_train(const char* what, const Train& t) {
    print(serial, "    ", what, ": median ");
    print_ticks(median_of(t));
    print(serial, "  (", t.woke, " woke, ", t.hits, " captured of ", t.n, ":");
    for (uint8_t i = 0; i < t.n; ++i) {
        print(serial, " ");
        if (t.ticks[i] == slink::no_capture) print(serial, "-");
        else print(serial, t.ticks[i]);
    }
    print(serial, ")", crlf);
}

constexpr uint8_t train_shots = 8;

// ---- the clock configurations the deep modes are measured against ---------------
//
// Four of them, and the fourth is the one the bench found and the data
// sheet does not spell out. VREGCTRL's AUTO profile drops the regulator
// to low power "in Standby and Power-Down, and whenever OSC32K is the
// only clock running" (13.3.5) - so an oscillator left RUNNING by its
// own RUNSTDBY keeps the regulator at full drive even when it is not the
// main clock source, and the wake-up never pays the regulator's
// start-up. `oschf_alone` stops the crystal outright, which is the only
// way to see that cost in standby.
enum class LegClock : uint8_t {
    xtal_restart,       ///< crystal main clock, its own RUNSTDBY clear
    xtal_kept,          ///< crystal main clock, kept alive by its RUNSTDBY
    oschf_xtal_alive,   ///< OSCHF main clock, the crystal still oscillating
    oschf_alone,        ///< OSCHF main clock, every other oscillator stopped
};

bool apply_leg_clock(LegClock c) {
    switch (c) {
        case LegClock::xtal_restart: {
            const bool ok = use_crystal(false);
            Oschf::run_standby(false);      // after: use_crystal() hops through OSCHF
            return ok;
        }
        case LegClock::xtal_kept:
            return use_crystal(true);
        case LegClock::oschf_xtal_alive:
            return use_oschf(false);
        default: {
            const bool ok = use_oschf(false);
            Xoschf::stop();
            return ok;
        }
    }
}

/// One whole leg: arm the peer, move the clock, answer the train, put
/// everything back and fetch the numbers. Nothing is printed between the
/// two console_drain() calls - the console's BAUD belongs to the crystal.
/// `stage` non-zero brackets the sleeps with the watchdog and the
/// .noinit token, which is what every power-down leg asks for.
bool run_leg(LegClock c, bool perf, SleepMode m, uint8_t stage, Train& t) {
    if (!train_arm(train_shots)) return false;
    console_drain();
    const bool moved = apply_leg_clock(c);
    if (perf) Vreg::power(VregPower::performance);
    if (stage) {
        token.magic = token_magic;
        token.stage = stage;
        (void)Watchdog::arm(WdtTime::s8);
    }
    train_answer(true, m, train_shots);
    if (stage) {
        (void)Watchdog::off();
        token.magic = 0;
        token.stage = 0;
    }
    Vreg::power(VregPower::normal);
    console_drain();
    const bool back = use_crystal(true);
    return train_collect(t) && moved && back;
}

// ---- h the link and the two measurement wires ------------------------------------
// Wire sanity BEFORE any sleep is involved: the command channel both
// ways, a corrupted frame refused, and each of the two measurement wires
// exercised on its own with this board wide awake.
void th_link() {
    print(serial, "h the link and the two measurement wires (nothing sleeps here)", crlf);
    quiesce();
    const bool up = ensure_link();
    verdict("board B answers a ping on the shared PE0 wire", up);
    if (!up) return;

    slink::Ident d{};
    const bool got = peer_ident(d);
    verdict("it identifies itself", got);
    if (got) {
        char label[9] = {};
        for (uint8_t i = 0; i < 8; ++i) label[i] = d.label[i];
        print(serial, "  peer: board ", label, ", CLK_PER from ",
              d.clock == slink::clock_crystal ? "XTAL" : "OSCHF", ", fw ",
              hex(d.version), crlf);
        verdict("board B runs sleep_peer", d.sanity == slink::ident_sanity);
        verdict("and it says which clock its stopwatch really counts",
                d.clock == slink::clock_crystal || d.clock == slink::clock_oschf);
    }

    // A frame whose checksum does not add up is refused, and the link
    // survives the desync.
    U4::flush_rx();
    line_talk();
    put4(slink::magic);
    put4(slink::byte_of(slink::Op::ping));
    put4(0);
    put4(0x00);                                  // not the checksum
    line_listen();
    slink::Frame bad;
    verdict("a corrupted frame is refused with a nak, not obeyed",
            recv_frame(bad, 120) && bad.op == slink::Op::nak);
    (void)link_command_mode();
    verdict("and the link works again straight after", command(slink::Op::ping));

    // PE3: this board drives, B captures. The stopwatch pipeline is
    // proven end to end with no sleep anywhere near it.
    slink::Params a{};
    a.delay_ms = 5;
    a.deadline_ms = 300;
    EchoPin::clear();
    EchoPin::output();
    const bool armed = peer_act(slink::Op::capture, a);
    if (armed) {
        delay_us(clock, 40'000);
        EchoPin::set();
        delay_us(clock, 500);
        EchoPin::clear();
    }
    slink::Report r{};
    const bool have = armed && peer_report(r, 40);
    verdict("an edge this board puts on PE3 is CAPTURED by the peer's stopwatch",
            have && r.hits == 1);
    if (have) {
        print(serial, "  the peer measured ");
        print_ticks(r.ticks);
        print(serial, " from arming to that edge (a software delay here, not a "
                      "latency: it proves the path)", crlf);
    }

    // PE2: B drives, this board senses - awake, through the very ISR
    // every later test measures.
    Train t{};
    bool ok = train_arm(3, 250);
    if (ok) train_answer(false, SleepMode::idle, 3);
    ok = ok && train_collect(t);
    verdict("the link carried the stimulus train", ok);
    verdict("every stimulus on PE2 reached this board's PORTE interrupt",
            ok && t.woke == 3);
    verdict("and every echo came back to the peer's capture", ok && t.hits == 3);
    if (ok) print_train("awake, three shots", t);
    print(serial, "  both wires and the stopwatch are proven: everything below is a "
                  "sleep mode measured through this same path.", crlf);
    quiesce();
}

// ---- i the awake baseline and IDLE ----------------------------------------------
// The baseline is the whole fixed cost of the measurement: the peer's
// zero-to-edge gap, the wire, this chip's interrupt latency and the ISR
// prologue up to the store that raises PE3. IDLE then adds the data
// sheet's six CLK_PER cycles (13.3.3.2) and nothing else - which at
// 24 MHz is 250 ns, well under this ruler's noise. So the verdict is a
// CLASS, not an equality: idle must not cost a microsecond more than
// staying awake, and the exact ticks are printed for both.
void ti_idle_latency() {
    print(serial, "i the awake baseline and the cost of IDLE, timed from board B", crlf);
    quiesce();
    if (!ensure_link()) { verdict("board B is reachable", false); return; }
    Pit::period(PitPeriod::cyc8192);          // a 250 ms backstop, out of the way

    Train awake{}, idle{};
    bool ok = train_arm(train_shots);
    if (ok) train_answer(false, SleepMode::idle, train_shots);
    ok = ok && train_collect(awake);
    bool ok2 = ok && train_arm(train_shots);
    if (ok2) train_answer(true, SleepMode::idle, train_shots);
    ok2 = ok2 && train_collect(idle);

    verdict("both trains got through the link", ok && ok2);
    if (!(ok && ok2)) { quiesce(); return; }
    print_train("awake  ", awake);
    print_train("idle   ", idle);
    const uint32_t ba = median_of(awake), bi = median_of(idle);
    verdict("awake, every stimulus was answered and captured",
            awake.woke == train_shots && awake.hits == train_shots);
    verdict("from IDLE, every stimulus was answered and captured",
            idle.woke == train_shots && idle.hits == train_shots);
    verdict("the awake baseline is a couple of microseconds: interrupt latency "
            "plus the ISR prologue", ba != slink::no_capture && ba < 240u);
    verdict("IDLE adds less than a microsecond to it (the six-cycle wake is "
            "250 ns at 24 MHz)",
            bi != slink::no_capture && ba != slink::no_capture && bi < ba + 24u);
    print(serial, "  IDLE - awake = ", static_cast<int32_t>(bi) - static_cast<int32_t>(ba),
          " of B's ticks (41.7 ns each); the six-cycle figure itself is measured "
          "on-chip by test_avr_platform f.", crlf);
    quiesce();
}

// ---- j STANDBY, timed from outside ----------------------------------------------
// Phase one proved WHICH RUNSTDBY keeps a peripheral counting and got at
// the wake-up cost only through the 30.5 us RTC ruler. Here board B
// measures six configurations in 41.7 ns ticks, and the sixth one is the
// finding: the regulator costs nothing until the device is allowed to
// let go of every oscillator. Nothing is printed while the main clock is
// not the crystal - the console's BAUD belongs to it.
void tj_standby_latency() {
    print(serial, "j STANDBY wake-up latency against six clock configurations", crlf);
    quiesce();
    if (!ensure_link()) { verdict("board B is reachable", false); return; }
    Pit::period(PitPeriod::cyc8192);

    struct Leg { LegClock clk; bool perf; const char* what; };
    constexpr Leg legs[] = {
        {LegClock::xtal_restart, false, "crystal, its RUNSTDBY off (it must start again)"},
        {LegClock::xtal_kept, false, "crystal, its RUNSTDBY on  (kept alive)         "},
        {LegClock::oschf_xtal_alive, false, "OSCHF, the crystal still running, normal      "},
        {LegClock::oschf_xtal_alive, true, "OSCHF, the crystal still running, performance "},
        {LegClock::oschf_alone, false, "OSCHF ALONE, every other oscillator off, normal"},
        {LegClock::oschf_alone, true, "OSCHF ALONE, every other oscillator off, perf. "},
    };
    Train t[6];
    uint32_t med[6];
    bool ok = true, all_woke = true;
    for (uint8_t i = 0; i < 6; ++i) {
        ok = ok && run_leg(legs[i].clk, legs[i].perf, SleepMode::standby, 0, t[i]);
        med[i] = median_of(t[i]);
        if (t[i].woke != train_shots) all_woke = false;
    }
    verdict("all six trains got through the link and every clock move landed", ok);
    if (!ok) { quiesce(); return; }
    for (uint8_t i = 0; i < 6; ++i) print_train(legs[i].what, t[i]);
    verdict("every configuration woke on every stimulus", all_woke);
    verdict("a crystal that has to restart costs the milliseconds phase one "
            "inferred from 48 RTC ticks (0.8 .. 3 ms)",
            med[0] != slink::no_capture && med[0] > 19'200u && med[0] < 72'000u);
    verdict("a crystal kept alive by its own RUNSTDBY costs almost nothing "
            "(under 50 us)", med[1] != slink::no_capture && med[1] < 1'200u);
    verdict("OSCHF restarting with the crystal still oscillating is the data "
            "sheet's own 24-30 us and NOTHING MORE (10 .. 60 us here)",
            med[2] != slink::no_capture && med[2] > 240u && med[2] < 1'440u);
    verdict("and the regulator's profile does not move it by a microsecond: "
            "there is no regulator start-up to pay for",
            med[3] != slink::no_capture &&
                (med[3] > med[2] ? med[3] - med[2] : med[2] - med[3]) < 24u);
    verdict("stop the crystal too and the SAME OSCHF restart suddenly costs "
            "hundreds of microseconds (100 .. 800 us)",
            med[4] != slink::no_capture && med[4] > 2'400u && med[4] < 19'200u);
    verdict("because that cost is the REGULATOR, and PMODE = performance pays it "
            "in advance: back to the bare oscillator restart",
            med[5] != slink::no_capture && med[5] < 1'440u && med[5] * 3u < med[4]);
    print(serial, "  that is the standby rule this bench found: VREGCTRL's AUTO profile "
                  "drops the regulator only when OSC32K is the last clock left, so an "
                  "oscillator held up by its own RUNSTDBY buys a fast wake-up for the "
                  "OTHER source as well.", crlf);
    print(serial, "  the ordering: kept-alive < OSCHF beside a running crystal = OSCHF "
                  "alone with PMODE = performance < OSCHF alone < a crystal restart.",
          crlf);
    quiesce();
}

// ---- k POWER-DOWN, timed from outside --------------------------------------------
// The gap phase one could not close. PE2 is a Px2 pin - one of the two
// FULLY ASYNCHRONOUS positions of every port - which is what makes an
// edge on it a wake-up source with every clock in the device stopped.
// The watchdog and the .noinit token bracket every train: a mode that
// never wakes resets the board and says so at the next boot.
//
// Power-down has no oscillator to hold the regulator up - RUNSTDBY buys
// nothing here, table 13-3 stops every domain but the PIT's - so this is
// where PMODE earns its keep unconditionally.
void tk_power_down_latency() {
    print(serial, "k POWER-DOWN wake-up latency on a fully asynchronous pin", crlf);
    quiesce();
    if (!ensure_link()) { verdict("board B is reachable", false); return; }
    Pit::period(PitPeriod::cyc8192);

    struct Leg { LegClock clk; bool perf; uint8_t stage; const char* what; };
    constexpr Leg legs[] = {
        {LegClock::xtal_kept, false, 4, "crystal main clock                       "},
        {LegClock::oschf_alone, false, 5, "OSCHF main clock, normal regulator       "},
        {LegClock::oschf_alone, true, 6, "OSCHF main clock, PMODE = performance    "},
    };
    Train t[3];
    uint32_t med[3];
    bool ok = true, all_woke = true;
    for (uint8_t i = 0; i < 3; ++i) {
        ok = ok && run_leg(legs[i].clk, legs[i].perf, SleepMode::power_down,
                           legs[i].stage, t[i]);
        med[i] = median_of(t[i]);
        if (t[i].woke != train_shots) all_woke = false;
    }
    verdict("all three trains got through the link and every clock move landed", ok);
    if (!ok) { quiesce(); return; }
    for (uint8_t i = 0; i < 3; ++i) print_train(legs[i].what, t[i]);
    verdict("a pin edge wakes this device from POWER-DOWN, every time, on all "
            "three configurations", all_woke);
    verdict("out of power-down a crystal main clock costs milliseconds (0.5 .. 6 ms)",
            med[0] != slink::no_capture && med[0] > 12'000u && med[0] < 144'000u);
    verdict("OSCHF out of power-down costs hundreds of microseconds (100 .. 800 us): "
            "the oscillator's 24 us plus the regulator's own start-up",
            med[1] != slink::no_capture && med[1] > 2'400u && med[1] < 19'200u);
    verdict("PMODE = performance removes the regulator's share and leaves the bare "
            "oscillator restart (under 60 us)",
            med[2] != slink::no_capture && med[2] < 1'440u);
    verdict("which is a factor of several, not a rounding: this is the one place "
            "the regulator's profile is worth its quiescent current",
            med[2] * 3u < med[1]);
    print(serial, "  the crystal's start-up buries the regulator entirely: its restart "
                  "is the same milliseconds in standby and in power-down.", crlf);
    quiesce();
}

// ---- l start-of-frame detection --------------------------------------------------
// usart.md's one bench gap: SFDEN needs a device that is really in
// standby, and until there was a second board there was nothing to send
// the frame. Errata 2.16.2 makes it a pair of verbs rather than a
// configuration field - armed on the way into the sleep, disarmed on the
// way out - and that is exactly the shape used here. That the detector
// works at all through LBME, listening to the TXD pad of a shared wire,
// is itself a fact of this desk and not of the data sheet.
//
// 27.3.4.2 says a high-to-low transition powers the oscillator up and
// "the rest of the data frame can be received, provided that the baud
// rate is slow enough concerning the oscillator start-up time". The legs
// below take that sentence apart into TWO independent questions, each
// with its own designed experiment, against the three main-clock restart
// times test j has already measured: a crystal (about 1.8 ms), OSCHF
// with the regulator asleep (about 300 us) and OSCHF with PMODE =
// performance (about 24 us).
//
//   DID IT WAKE? The detector needs the line to still be LOW when the
//   peripheral clock is back. So each restart time is probed with two
//   bytes at one rate: 0xFF holds the line low for the START BIT ALONE,
//   0x00 for nine bit times. If the low level is what matters, 0x00
//   wakes where 0xFF does not, at every restart time - and the rate that
//   separates them is the restart time itself.
//
//   DID THE FRAME SURVIVE? Only if the clock is back before the middle
//   of the start bit, where the receiver takes its samples: half a bit.
enum class SfdClock : uint8_t { xtal_kept, xtal_restart, oschf_slow, oschf_fast };

struct SfdLeg {
    uint32_t baud;
    uint8_t value;
    SfdClock clk;
    SleepMode mode;
    const char* what;
};

bool apply_sfd_clock(SfdClock c) {
    switch (c) {
        case SfdClock::xtal_kept: return use_crystal(true);
        case SfdClock::xtal_restart: return use_crystal(false);
        default: {
            const bool ok = use_oschf(false);
            Xoschf::stop();          // test j: a running crystal holds the regulator up
            if (c == SfdClock::oschf_fast) Vreg::power(VregPower::performance);
            return ok;
        }
    }
}

/// One SFD leg. `woke` and `intact` come back by reference; false means
/// the link or a clock move failed.
bool sfd_once(const SfdLeg& leg, bool& woke, bool& intact, bool& arrived,
              uint8_t& got) {
    const uint8_t value = leg.value;
    (void)link_command_mode();
    slink::Params a{};
    a.value = value;
    a.rate = leg.baud;
    a.delay_ms = 60;
    a.deadline_ms = 200;
    if (!peer_act(slink::Op::sfd_byte, a)) return false;

    // This board's end of the shared wire at the peer's rate: receiver
    // only, listening to the TXD pad through LBME.
    const uint16_t br = usart_baud_reg(SysClock::hz, leg.baud);
    if (!U4::init({.route = UsartRoute::def, .baud = br, .tx = false, .loop_back = true})) {
        return false;
    }
    LinePin::input();
    pinctrl_of('E', 0) |= PORT_PULLUPEN_bm;
    U4::flush_rx();
    U4::clear_rxs();
    cli();
    sfd_irqs = 0;
    sei();
    U4::enable_rxs_interrupt(true);

    console_drain();
    const bool moved = apply_sfd_clock(leg.clk);
    // Armed only around the transition, and only here: with SFDEN set in
    // Active mode a read of RXDATA restarts the frame (errata 2.16.2).
    U4::arm_start_of_frame();
    cli();
    const uint16_t s0 = sfd_irqs;
    sei();
    for (uint8_t k = 0; k < 4; ++k) {
        Sleep::enter(leg.mode);
        cli();
        const bool done = sfd_irqs != s0;
        sei();
        if (done) break;
    }
    U4::disarm_start_of_frame();
    U4::enable_rxs_interrupt(false);
    // The wake happens at the START BIT: the rest of the frame is still
    // on the wire. Give it three character times before asking.
    delay_us(clock, (30u * 1'000'000u) / leg.baud + 300u);
    const bool have = U4::rxc_flag();
    const UsartFrame f = have ? U4::receive() : UsartFrame{};
    Vreg::power(VregPower::normal);
    const bool back = use_crystal(true);
    cli();
    woke = sfd_irqs != s0;
    sei();
    arrived = have;
    intact = have && f.clean() && static_cast<uint8_t>(f.data) == value;
    got = have ? static_cast<uint8_t>(f.data) : 0;
    (void)link_command_mode();
    return moved && back;
}

constexpr uint8_t sfd_legs = 11;

void tl_sfd() {
    print(serial, "l USART start-of-frame detection: a start bit on the shared wire "
                  "wakes this board (SFD through LBME is itself a fact of this desk)",
          crlf);
    quiesce();
    if (!ensure_link()) { verdict("board B is reachable", false); return; }
    // The PIT is only a backstop here, and it must stay out of the way:
    // a device that is AWAKE in the PIT's own vector when the start bit
    // arrives is not in standby, and would look like a failed wake-up.
    // One second between its interrupts against a byte due in sixty
    // milliseconds makes that coincidence vanish.
    Pit::period(PitPeriod::cyc32768);

    constexpr SfdLeg legs[sfd_legs] = {
        {9'600, 0x5A, SfdClock::xtal_kept, SleepMode::standby,
         "clock kept alive      9600   0x5A (no restart at all)         "},
        {115'200, 0x5A, SfdClock::xtal_kept, SleepMode::standby,
         "clock kept alive      115200 0x5A (no restart at all)         "},
        {460'800, 0x5A, SfdClock::xtal_kept, SleepMode::standby,
         "clock kept alive      460800 0x5A (no restart at all)         "},
        {9'600, 0x5A, SfdClock::oschf_fast, SleepMode::standby,
         "OSCHF alone, perf.    9600   0x5A (24 us vs a 52 us half-bit) "},
        {115'200, 0xFF, SfdClock::oschf_fast, SleepMode::standby,
         "OSCHF alone, perf.    115200 0xFF (24 us vs 8.7 us of low)    "},
        {115'200, 0x00, SfdClock::oschf_fast, SleepMode::standby,
         "OSCHF alone, perf.    115200 0x00 (24 us vs 78 us of low)     "},
        {9'600, 0xFF, SfdClock::oschf_slow, SleepMode::standby,
         "OSCHF alone, normal   9600   0xFF (300 us vs 104 us of low)   "},
        {9'600, 0x00, SfdClock::oschf_slow, SleepMode::standby,
         "OSCHF alone, normal   9600   0x00 (300 us vs 938 us of low)   "},
        {2'400, 0xFF, SfdClock::xtal_restart, SleepMode::standby,
         "crystal restarts      2400   0xFF (1.8 ms vs 417 us of low)   "},
        {2'400, 0x00, SfdClock::xtal_restart, SleepMode::standby,
         "crystal restarts      2400   0x00 (1.8 ms vs 3.75 ms of low)  "},
        {115'200, 0x5A, SfdClock::xtal_kept, SleepMode::idle,
         "IDLE, clock running   115200 0x5A                             "},
    };
    bool woke[sfd_legs] = {}, intact[sfd_legs] = {}, arrived[sfd_legs] = {};
    uint8_t got[sfd_legs] = {};
    bool ok = true;
    for (uint8_t i = 0; i < sfd_legs; ++i) {
        ok = ok && sfd_once(legs[i], woke[i], intact[i], arrived[i], got[i]);
    }
    verdict("the peer sent every commanded byte and the link survived", ok);
    for (uint8_t i = 0; i < sfd_legs; ++i) {
        print(serial, "    ", legs[i].what, ": ", woke[i] ? "WOKE   " : "no wake",
              "  frame ", intact[i] ? "INTACT" : (arrived[i] ? "GARBLED" : "lost   "),
              " (got ", hex(got[i]), ", sent ", hex(legs[i].value), ")", crlf);
    }
    verdict("with the main clock kept alive a start bit wakes this board from "
            "STANDBY at every rate", woke[0] && woke[1] && woke[2]);
    verdict("and the WAKING FRAME itself survives, all three rates",
            intact[0] && intact[1] && intact[2]);
    verdict("a restart well inside the start bit's first half keeps the frame too: "
            "24 us against a 52 us half-bit at 9600", woke[3] && intact[3]);
    verdict("THE WAKE-UP RULE, at a 24 us restart: 0xFF (low for 8.7 us) does not "
            "wake, 0x00 (low for 78 us) does", !woke[4] && woke[5]);
    verdict("the same rule at a 300 us restart: 104 us of low is not enough, "
            "938 us is", !woke[6] && woke[7]);
    verdict("and at a 1.8 ms crystal restart: 417 us of low is not enough, 3.75 ms "
            "is", !woke[8] && woke[9]);
    print(serial, "  so the detector does not latch an EDGE: it needs the line still "
                  "LOW when the peripheral clock comes back, which is why the byte's "
                  "own bit pattern decides whether a slow-restarting device wakes at "
                  "all.", crlf);
    verdict("every wake that arrived after the middle of the start bit cost the "
            "frame", !intact[5] && !intact[7] && !intact[9]);
    verdict("from IDLE the start-of-frame flag never fires - RXSIF is set only in "
            "Standby (27.5.5) - and the frame is simply received",
            !woke[10] && intact[10]);
    verdict("SFDEN is disarmed again on the way out, as errata 2.16.2 demands",
            !U4::start_of_frame_armed());
    quiesce();
}

// ---- m a TWI address match wakes the device --------------------------------------
// This board becomes a client at 0x42 on the office bus (TWI0 default,
// PA2/PA3) and sleeps with it enabled; board B addresses it. The
// SCL stretch from the match to the first serviced byte IS the wake-up
// latency, and it is visible in the tenure's wall time, which B measures.
//
// The HTLLEN half of chapter 13's warning is asserted in situ here: with
// a client enabled the driver REFUSES to arm high-temperature low
// leakage, so the subset of power-down that has no TWI wake-up is
// enforced by construction and never entered by accident.
bool twi_tenure(SleepMode m, bool sleeping, uint8_t first, slink::Report& r) {
    cli();
    twis_stops = 0;
    twis_addr = twis_stops;
    twis_n = 0;
    sei();
    slink::Params a{};
    a.count = 3;
    a.addr = client_addr;
    a.value = first;
    a.rate = 100'000u;
    a.delay_ms = 120;
    a.deadline_ms = 300;
    if (!peer_act(slink::Op::twi_write, a)) return false;

    // THE ADDRESS MATCH IS THE ONLY TWI WAKE-UP SOURCE: table 13-4 lists
    // it and nothing else of the peripheral, so the DATA interrupts that
    // carry the rest of the frame cannot wake this device. Going back to
    // sleep after the match would therefore stall the tenure with the
    // host holding SCL - measured, and it is why this loop sleeps only
    // until the match and then STAYS AWAKE while the vector serves the
    // frame to its closing STOP.
    if (sleeping) {
        if (m == SleepMode::power_down) {
            token.magic = token_magic;
            token.stage = 7;
            (void)Watchdog::arm(WdtTime::s4);
        }
        cli();
        const uint16_t a0 = twis_addr;
        const uint16_t s0 = twis_stops;
        sei();
        for (uint8_t k = 0; k < 6; ++k) {
            Sleep::enter(m);
            cli();
            const bool matched = twis_addr != a0;
            sei();
            if (matched) break;
        }
        for (uint32_t i = 0; i < 2'000'000u && twis_stops == s0; ++i) {
        }
        if (m == SleepMode::power_down) {
            (void)Watchdog::off();
            token.magic = 0;
            token.stage = 0;
        }
    }
    // Awake or not, let the closing STOP land before the numbers are read.
    delay_us(clock, 20'000);
    return peer_report(r, 60);
}

void tm_twi_wake() {
    print(serial, "m a TWI address match wakes this board from standby and from "
                  "power-down (board B is the host on the office bus)", crlf);
    quiesce();
    if (!ensure_link()) { verdict("board B is reachable", false); return; }
    Pit::period(PitPeriod::cyc8192);

    verdict("the client comes up at 0x42 on TWI0's default pins",
            Client::init(clock, {.address = client_addr,
                                 .stop_interrupt = true,
                                 .data_interrupt = true,
                                 .address_interrupt = true}));
    verdict("HTLLEN is REFUSED in situ, with the wake-up source it would silence "
            "actually enabled", !Vreg::high_temp_low_leakage(true));
    verdict("and nothing was written", !Vreg::high_temp_low_leakage());

    slink::Report awake{}, standby{}, pd{};
    bool ok = twi_tenure(SleepMode::idle, false, 0x10, awake);
    const uint8_t awake_n = twis_n;
    const bool awake_ok = twis_rx[0] == 0x10 && twis_rx[1] == 0x11 && twis_rx[2] == 0x12;
    ok = ok && twi_tenure(SleepMode::standby, true, 0x20, standby);
    const uint8_t standby_n = twis_n;
    const bool standby_ok = twis_rx[0] == 0x20 && twis_rx[1] == 0x21 && twis_rx[2] == 0x22;
    ok = ok && twi_tenure(SleepMode::power_down, true, 0x30, pd);
    const uint8_t pd_n = twis_n;
    const bool pd_ok = twis_rx[0] == 0x30 && twis_rx[1] == 0x31 && twis_rx[2] == 0x32;

    verdict("the link carried all three tenures", ok);
    if (!ok) { Client::release(); quiesce(); return; }
    print(serial, "    awake      : ", awake_n, " bytes here, peer status ",
          hex(awake.status), ", tenure ");
    print_ticks(awake.ticks);
    print(serial, crlf, "    standby    : ", standby_n, " bytes here, peer status ",
          hex(standby.status), ", tenure ");
    print_ticks(standby.ticks);
    print(serial, crlf, "    power-down : ", pd_n, " bytes here, peer status ",
          hex(pd.status), ", tenure ");
    print_ticks(pd.ticks);
    print(serial, crlf);

    verdict("awake, the three bytes arrive intact and every one is acknowledged",
            awake_n == 3 && awake_ok && (awake.flags & slink::report_acked));
    verdict("STANDBY: the address match wakes this board and the frame is served "
            "in full", standby_n == 3 && standby_ok &&
                       (standby.flags & slink::report_acked));
    verdict("POWER-DOWN: it wakes there too (HTLLEN clear), frame intact",
            pd_n == 3 && pd_ok && (pd.flags & slink::report_acked));
    // The client holds SCL from the address match until the first
    // serviced byte, so the whole wake-up is paid by the HOST on the
    // wire - and it is the SAME wake-up test j and test k measured on a
    // pin, seen from the other side of the bus.
    verdict("waking from STANDBY with the main clock kept alive costs the bus "
            "nothing measurable: the tenure is the awake one to within 10 us",
            standby.ticks > awake.ticks ? standby.ticks - awake.ticks < 240u
                                        : awake.ticks - standby.ticks < 240u);
    verdict("waking from POWER-DOWN costs the bus the whole crystal restart - the "
            "0.8 .. 3 ms test k measured on a pin",
            pd.ticks > awake.ticks && pd.ticks - awake.ticks > 19'200u &&
                pd.ticks - awake.ticks < 72'000u);
    print(serial, "  the stretch over the awake baseline: standby +",
          static_cast<int32_t>(standby.ticks / 24u) -
              static_cast<int32_t>(awake.ticks / 24u),
          " us, power-down +", (pd.ticks - awake.ticks) / 24u,
          " us. At 100 kHz an SCL bit is 10 us, which is why a standby wake of "
          "under two microseconds is invisible on the wire and a crystal restart "
          "is not.", crlf);

    Client::release();
    quiesce();
    verdict("the client is released and HTLLEN is settable again",
            Vreg::high_temp_low_leakage(true));
    (void)Vreg::high_temp_low_leakage(false);
}

// ---- n the CCL as a wake-up source (single board) ---------------------------------
// ccl.md's parked item. Table 13-4 lists the CCL among standby's wake-up
// sources when it has RUNSTDBY, and among power-down's only when the
// path through it is FULLY ASYNCHRONOUS - FILTSEL = 0 and EDGEDET = 0.
// All three shapes are built here on the one signal that survives every
// sleep mode: a PIT divider through the event system, which needs no
// wires. What the bench then shows is that the restriction is really
// about WHICH CLOCK the LUT was given: a filter running on OSC32K keeps
// working in power-down because that domain never stops, while the same
// filter on CLK_PER does not.
struct CclShape {
    LutFilter filter;
    LutClock clock;
    const char* what;
};

bool ccl_shape(const CclShape& s) {
    Ccl::disable();
    const bool ok = WakeLut::init({.in0 = LutInput::event_a,
                                   .truth = lut_truth([](bool a, bool, bool) { return a; }),
                                   .filter = s.filter,
                                   .edge_detect = false,
                                   .clock = s.clock,
                                   .interrupt = LutSense::rising});
    WakeLut::event_a_on(ChCcl{});
    Ccl::enable(true);                        // RUNSTDBY
    delay_us(clock, 20'000);
    return ok;
}

void tn_ccl_wake() {
    print(serial, "n the CCL as a wake-up source: which shape wakes from standby, and "
                  "which from power-down", crlf);
    quiesce();
    Pit::period(PitPeriod::cyc32768);         // a 1 s backstop, far from the CCL's rate
    Pit::enable_interrupt(true);
    ChCcl::source(EvPitDiv<64>{});            // 512 Hz square: an edge every ~1 ms

    constexpr CclShape filtered_32k = {LutFilter::filter, LutClock::osc32k,
                                       "clocked by OSC32K and FILTERED"};
    constexpr CclShape filtered_per = {LutFilter::filter, LutClock::clk_per,
                                       "clocked by CLK_PER and FILTERED"};
    constexpr CclShape async_path = {LutFilter::none, LutClock::osc32k,
                                     "FILTSEL = 0, EDGEDET = 0 (fully asynchronous)"};

    // 1. STANDBY, the RUNSTDBY-relevant shape: the LUT is clocked and
    //    filtered, so it needs its clock domain kept alive.
    verdict("a clocked, filtered LUT fed by the PIT divider comes up",
            ccl_shape(filtered_32k));
    cli();
    const uint16_t c0 = ccl_irqs;
    sei();
    verdict("it really toggles: its interrupt fires with the CPU awake", c0 > 0);
    const SleepRun standby = sleep_once(SleepMode::standby, false);
    verdict("the CCL wakes this board from STANDBY, before the 1 s backstop could",
            standby.pits == 0 && standby.ccls > 0);

    // 2. POWER-DOWN, three shapes. Every count below is taken INSIDE the
    //    measured sleep (SleepRun), so "which source ended it" is read off
    //    the same window and not off whatever ran after the wake.
    const SleepRun pd_32k = power_down_once(8, false);
    const bool cfg_per = ccl_shape(filtered_per);
    const SleepRun pd_per = power_down_once(9, false);
    const bool cfg_async = ccl_shape(async_path);
    const SleepRun pd_async = power_down_once(10, false);

    verdict("both other shapes come up too", cfg_per && cfg_async);
    print(serial, "  standby,    ", filtered_32k.what, ": ", standby.pits,
          " PIT / ", standby.ccls, " CCL interrupt(s)", crlf);
    print(serial, "  power-down, ", filtered_32k.what, ": ", pd_32k.pits,
          " PIT / ", pd_32k.ccls, " CCL interrupt(s)", crlf);
    print(serial, "  power-down, ", filtered_per.what, ": ", pd_per.pits,
          " PIT / ", pd_per.ccls, " CCL interrupt(s)", crlf);
    print(serial, "  power-down, ", async_path.what, ": ", pd_async.pits,
          " PIT / ", pd_async.ccls, " CCL interrupt(s)", crlf);

    verdict("a FULLY ASYNCHRONOUS CCL path wakes this board from POWER-DOWN, "
            "before the backstop - table 13-4's note 4",
            pd_async.pits == 0 && pd_async.ccls > 0);
    // The single CCL interrupt that follows this one is not a wake-up:
    // the frozen filter catches up the instant CLK_PER is back, which is
    // after the backstop has already ended the sleep. Which source ENDED
    // it is what the PIT count says.
    verdict("a FILTERED LUT clocked from CLK_PER does NOT wake from POWER-DOWN: "
            "that clock is gone and the 1 s backstop is what ends the sleep",
            pd_per.pits > 0);
    verdict("but a filtered LUT clocked from OSC32K DOES, because the 32 kHz "
            "domain is exactly what power-down keeps for the PIT",
            pd_32k.pits == 0 && pd_32k.ccls > 0);
    print(serial, "  so note 4's 'fully asynchronous' is really a statement about the "
                  "LUT's CLOCK: any path whose clock survives the mode survives with "
                  "it, and on this silicon OSC32K does.", crlf);
    quiesce();
}

// ---- the menu -------------------------------------------------------------------------------

using TestFn = void (*)();
struct Test { char key; TestFn fn; };
constexpr Test tests[] = {
    {'a', ta_surface}, {'b', tb_idle}, {'c', tc_standby}, {'d', td_chain},
    {'e', te_pin_wake}, {'f', tf_power_down}, {'g', tg_vreg}, {'n', tn_ccl_wake},
    {'h', th_link}, {'i', ti_idle_latency}, {'j', tj_standby_latency},
    {'k', tk_power_down_latency}, {'l', tl_sfd}, {'m', tm_twi_wake},
};
constexpr char all_keys[] = "abcdefgn";      ///< z: the single-board half
constexpr char two_board_keys[] = "hijklm";  ///< y: the two-board half

void run(TestFn fn) {
    passed = failed = 0;
    fn();
    print(serial, "  -> ", passed, " pass, ", failed, " fail", crlf, crlf);
}

void run_set(const char* keys) {
    uint16_t tp = 0, tf = 0;
    for (const char* k = keys; *k != 0; ++k) {
        for (const Test& t : tests) {
            if (t.key != *k) continue;
            run(t.fn);
            tp = static_cast<uint16_t>(tp + passed);
            tf = static_cast<uint16_t>(tf + failed);
        }
    }
    print(serial, "ALL: ", tp, " pass, ", tf, " fail", crlf);
}

void help() {
    print(serial, "test_avr_sleep: a the register surface | b idle through enter() | "
                  "c standby is real | d the RUNSTDBY chain | e a pin wakes from the deep "
                  "modes | f power-down stops the rest | g the voltage regulator | "
                  "n the CCL as a wake-up source    -> z = all of those", crlf);
    print(serial, "  two-board (board B = sleep_peer): h the link and the two wires | "
                  "i the awake baseline and idle | j standby timed from outside | "
                  "k power-down timed from outside | l start-of-frame | "
                  "m a TWI address match    -> y = all of h..m", crlf);
}

} // namespace

ISR(USART2_RXC_vect) { (void)Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }

ISR(RTC_PIT_vect) {
    (void)Pit::take_flag();
    pit_irqs = pit_irqs + 1;
}
ISR(RTC_CNT_vect) {
    (void)Rtc::take_flags();
    rtc_irqs = rtc_irqs + 1;
}
ISR(PORTD_PORT_vect) {
    (void)Port<'D'>::take_flags();
    pad_irqs = pad_irqs + 1;
}

/// THE WAKE-UP ISR OF THE TWO-BOARD HALF, and the one place in this
/// suite where the ORDER of two statements is a measurement. The echo
/// goes up FIRST - a single-cycle SBI on a low-I/O VPORT register - so
/// everything board B reports is the silicon's wake-up plus this
/// vector's own latency and prologue, and nothing else. The flag clear
/// and the counter follow.
ISR(PORTE_PORT_vect) {
    EchoPin::set();
    (void)Port<'E'>::take_flags();
    wake_irqs = wake_irqs + 1;
}

/// USART4's RXC vector carries RXSIF too (27.5.6): with SFDEN armed and
/// RXSIE on, THIS is what a start bit on the shared wire wakes into. It
/// must not read RXDATA - errata 2.16.2 says a read during a reception
/// restarts the frame - so it only clears the start flag and counts.
ISR(USART4_RXC_vect) {
    if (U4::rxs_flag()) {
        U4::clear_rxs();
        sfd_irqs = sfd_irqs + 1;
    }
}

/// The TWI client of test m, serviced entirely from its vector: the
/// address match that wakes the device and every byte of the frame are
/// one uninterrupted piece of work, so the stretch board B measures is
/// the wake-up and nothing else.
ISR(TWI0_TWIS_vect) {
    const auto s = Client::isr();
    if (s.address_or_stop()) {
        if (s.is_address()) {
            twis_addr = twis_addr + 1;
            twis_n = 0;
            Client::respond(TwiAck::ack);
        } else {
            twis_stops = twis_stops + 1;
            Client::complete();
        }
    } else if (s.data()) {
        if (s.host_reading()) {
            Client::transmit(0x5A);
        } else {
            const uint8_t v = Client::receive(TwiAck::ack);
            if (twis_n < 8) {
                twis_rx[twis_n] = v;
                twis_n = twis_n + 1;
            }
        }
    }
}

ISR(CCL_CCL_vect) {
    (void)Ccl::take_flags();
    ccl_irqs = ccl_irqs + 1;
}

// All four TCB vectors are bound as a net: an unbound vector on this
// core is a jump to 0, i.e. a silent reset loop, and the deep sleeps
// here are exactly the place where a stray flag would find one.
ISR(TCB0_INT_vect) {
    (void)Waker::take_flags();
    waker_irqs = waker_irqs + 1;
}
ISR(TCB1_INT_vect) { (void)WatchLo::take_flags(); }
ISR(TCB2_INT_vect) { (void)WatchHi::take_flags(); }
ISR(TCB3_INT_vect) { (void)Tcb<3>::take_flags(); }

int main() {
    const ResetFlags why = Reset::take_flags();
    const bool hung = token.magic == token_magic && token.stage != 0;
    const uint8_t hung_stage = token.stage;
    token.magic = 0;
    token.stage = 0;

    const bool xtal = SysClock::init();
    Serial::init(clock, 460800);
    RtcClock::select(RtcSource::osc32k);
    Pit::init(pit_period);
    sei();

    auto board = board_id();
    if (board.empty()) board = "?";
    print(serial, crlf, "test_avr_sleep - sleep test suite (board ", board,
          ", clk=", xtal ? "XTAL" : "OSCHF", " 24 MHz, silicon rev ", hex(SYSCFG.REVID),
          ", RSTFR=", hex(why.raw), ")", crlf);
    print(serial, "  PD1/PD2 must be free of the bus jumpers: test e drives PD2 from the "
                  "event system and senses its own edge.", crlf);
    print(serial, "  the two-board set y needs board B running sleep_peer: PE0 the shared "
                  "command wire, PE2 its stimulus in, PE3 the echo out, PA2/PA3 the bus.",
          crlf);
    if (hung) {
        print(serial, "  PREVIOUS RUN DID NOT WAKE: a power-down sleep (stage ", hung_stage,
              ") was ended by the watchdog", why.watchdog ? "" : " - though RSTFR does not "
              "name the watchdog", ". That is a FINDING, not a pass.", crlf);
    }
    help();

    print(serial, "> ");
    for (;;) {
        uint8_t c;
        if (!Serial::read_byte(c)) continue;
        if (c == '\r' || c == '\n') continue;
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') { help(); }
        else if (c == 'z' || c == 'Z') { run_set(all_keys); }
        else if (c == 'y' || c == 'Y') { run_set(two_board_keys); }
        else {
            bool found = false;
            for (const Test& t : tests) if (t.key == c) { run(t.fn); found = true; }
            if (!found) print(serial, "? for help", crlf);
        }
        print(serial, "> ");
    }
}
