// test_samc_clock - the reference bench suite for samc/clock.hpp's two
// newest roots: the external crystal oscillator (XOSC) and the
// fractional DPLL (FDPLL96M), DS60001479M ch. 20.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the driver
// under it.
//
// NOTHING TO WIRE - and for once that sentence is not about jumpers.
// The board's 24 MHz crystal on PA14/PA15 has been on the board since
// bring-up and has never been switched on; OSCCTRL claims XIN and XOUT
// by itself when the oscillator is enabled (20.5.1), so arming it is a
// register write and nothing else.
//
// THE INSTRUMENT IS AGAIN samc/freqm.hpp, and the measurement is
// arranged so the answers are RATIOS wherever a ratio is what is being
// claimed:
//   - generator 5 carries the crystal divided by 512 (46875 Hz) and is
//     the frequency meter's REFERENCE, because the meter needs its
//     reference to be the slower clock and a big ratio is what buys
//     resolution: 255 reference periods against a 48 MHz measurand is a
//     count of 261120, i.e. about 4 ppm a step;
//   - measuring OSC48M against that reference weighs the INTERNAL RC on
//     the crystal's scale, which is the one number this bench has never
//     had;
//   - measuring the DPLL against the same reference weighs the DPLL's
//     own arithmetic against the crystal it is locked to, so the
//     crystal's absolute error cancels out of the verdict entirely.
//
// What is exercised, letter by letter:
//   a  the block, the arithmetic and the refusals - no oscillator moved
//   b  THE CRYSTAL, armed for the first time on this board: start-up
//      time measured, then the internal RC weighed against it, and
//      OSCULP32K weighed against both
//   c  the clock failure detector: a real failure induced with no wire,
//      the safe-clock switch observed, and the recovery
//   d  the DPLL: locked to the crystal, three ratios including a
//      fractional one, the on-the-fly ratio change with its two errata,
//      the output prescaler, and the GCLK reference path
//   e  THE CPU RUNNING FROM THE DPLL - generator 0 moved to the loop and
//      moved back, with the console alive throughout
//   f  GCLK's two divider regimes, settled against the chapter's one
//      ambiguous sentence - no oscillator involved, only a ratio
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "samc/clock.hpp"
#include "samc/freqm.hpp"
#include "samc/nvm.hpp"
#include "samc/osc32kctrl.hpp"
#include "samc/pin.hpp"
#include "samc/sercom.hpp"
#include "samc/tc.hpp"
#include "samc/ticker.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

constexpr UartPads console_pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'B', 30, PinFunction::d},
    .rx_pin = {'B', 31, PinFunction::d},
};
using Serial = Uart<5, console_pads>;
constexpr Serial serial;
using Led = Pin<'B', 23>;

TestBench<Serial> bench;

using brio::crlf;
using brio::print;

// ---------------------------------------------------------------------------
// The board, and the generators this suite builds
// ---------------------------------------------------------------------------

/// What is soldered to XIN/XOUT. Everything the suite predicts is
/// predicted from this number and from nothing else.
constexpr uint32_t crystal_hz = 24'000'000UL;

/// STARTUP 0x4 = 16 OSCULP32K cycles, about 488 us by table 20-5. Long
/// enough for a 24 MHz crystal on this board and short enough that the
/// measurement below is dominated by the crystal, not by the counter.
constexpr XoscStartup crystal_startup = 4;

constexpr uint8_t gen_sys = 0;      ///< OSC48M, the CPU's own generator
constexpr uint8_t gen_ref_1m = 3;   ///< OSC48M / 48 = 1 MHz, the DPLL's GCLK reference
constexpr uint8_t gen_dpll = 4;     ///< whatever the DPLL is producing
constexpr uint8_t gen_xtal = 5;     ///< the crystal divided by 512
constexpr uint8_t gen_ulp = 6;      ///< OSCULP32K, the second opinion

using GenRef1M = Gclk<gen_ref_1m>;
using GenDpll = Gclk<gen_dpll>;
using GenXtal = Gclk<gen_xtal>;
using GenUlp = Gclk<gen_ulp>;

/// The crystal reference is built with the LINEAR divider, not with
/// DIVSEL: 24 MHz / 250 = 96000 Hz exactly, and 16.6.2.7's linear rule
/// is the one regime of that field nobody has ever disagreed about.
/// (Letter f settles the other one.) A 96 kHz reference against a
/// 48 MHz measurand is a count of 127500 at REFNUM 255 - about 8 ppm a
/// step, which is finer than any oscillator here deserves.
constexpr uint32_t xtal_div = 250;
constexpr uint32_t xtal_ref_hz = crystal_hz / xtal_div;   // 96000

using Stopwatch = Tc<2>;
constexpr uint32_t sys_hz = SysClock::hz;
/// Two stopwatch rates, because the two things being timed are three
/// decades apart: a crystal start-up is milliseconds, a DPLL lock is
/// tens of microseconds.
constexpr uint32_t fine_hz = sys_hz / 16u;      // 3 MHz, 1/3 us a tick
constexpr uint32_t coarse_hz = sys_hz / 1024u;  // 46875 Hz, 21.3 us a tick

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void wait_ms(uint32_t ms) {
    const uint32_t deadline = Ticker::millis() + ms;
    while (static_cast<int32_t>(Ticker::millis() - deadline) < 0) {
    }
}

// NB the printed lines below never contain "->": tools/bench.py's judge
// looks for that arrow to find a letter's tally line, and a stray one in
// a report line ends the capture early. Learned on the bench.
bool near(uint32_t got, uint32_t want, uint32_t band) {
    return got + band >= want && want + band >= got;
}

/// Deviation from a nominal value in parts per million, unsigned - the
/// sign is printed separately where it matters.
uint32_t ppm_off(uint32_t got, uint32_t want) {
    const uint64_t d = got > want ? got - want : want - got;
    return static_cast<uint32_t>((d * 1'000'000ULL) / want);
}

bool stopwatch_up(TcPrescaler p) {
    return Stopwatch::init(gen_sys) &&
           Stopwatch::configure(TcConfig{.mode = TcMode::count16, .prescaler = p}) &&
           Stopwatch::enable(true);
}

/**
 * Measure `measured` against `reference`, and return the RAW COUNT -
 * measured periods per `refnum` reference periods.
 *
 * The count is what the verdicts compare, not a frequency: a count is
 * what the silicon produced and a frequency is a count multiplied by
 * something this program only believes.
 */
std::optional<uint32_t> measure_count(uint8_t measured, uint8_t reference,
                                      uint8_t refnum) {
    const FreqmConfig cfg{
        .measured_generator = measured,
        .reference_generator = reference,
        .refnum = refnum,
    };
    if (!Freqm::init(cfg)) {
        return std::nullopt;
    }
    const auto count = Freqm::measure();
    Freqm::release();
    if (!count || *count == 0u) {
        return std::nullopt;
    }
    return count;
}

/// The measurand's frequency in hertz, given what the caller believes
/// the reference to be.
constexpr uint32_t count_to_hz(uint32_t count, uint32_t reference_hz, uint8_t refnum) {
    return static_cast<uint32_t>(
        (static_cast<uint64_t>(count) * reference_hz) / refnum);
}

constexpr uint8_t refnum_full = 255;

/// Predicted count for a measurand of `hz` against the crystal-derived
/// reference. Integer arithmetic in 64 bits: 255 x 96 MHz overflows 32.
constexpr uint32_t expect_count(uint32_t hz) {
    return static_cast<uint32_t>((static_cast<uint64_t>(hz) * refnum_full) /
                                 xtal_ref_hz);
}

/// Start the crystal if it is not already running, and put generator 5
/// on it divided by 512. Every letter that measures anything calls this,
/// so each letter stands alone and `z` re-runs in one power-on.
bool crystal_up() {
    if (!Xosc::enabled() || !Xosc::ready()) {
        if (!Xosc::init(XoscConfig{.hz = crystal_hz, .startup = crystal_startup})) {
            return false;
        }
    }
    if (!GenXtal::configure(
            GclkConfig{.source = GclkSource::xosc, .div = xtal_div})) {
        return false;
    }
    return GenXtal::enabled();
}

/**
 * Hand the crystal back, IN THE ONLY ORDER THAT WORKS.
 *
 * 16.6.2.6 releases a generator's old source only once the NEW one is
 * ready, so a generator still pointed at a stopped oscillator can never
 * be moved again. Every generator this suite built on the crystal or on
 * the DPLL is therefore moved to OSC48M - which is always running -
 * BEFORE anything is stopped. The order is the test.
 */
bool crystal_down() {
    const bool moved =
        GenDpll::configure(GclkConfig{.source = GclkSource::osc48m}) &&
        GenXtal::configure(GclkConfig{.source = GclkSource::osc48m});
    (void)Fdpll::stop();
    Xosc::stop();
    return moved;
}

// =============================================================================
// a - the block, the arithmetic and the refusals
// =============================================================================
void ta_block() {
    // The block's own surface. OSCCTRL has no enable and no reset: it is
    // always powered, and its APB clock is on out of reset.
    print(serial, "  OSCCTRL status=", hex(Oscctrl::status()),
          " intflag=", hex(Oscctrl::flags()), crlf);
    bench.verdict("OSC48M reports ready - it is what the CPU is running on",
                  (Oscctrl::status() & OSCCTRL_STATUS_OSC48MRDY_Msk) != 0u);
    bench.verdict("the OSCCTRL interrupt is IRQ 0, shared with four other blocks",
                  Oscctrl::irq() == OSCCTRL_IRQn);

    Oscctrl::arm(OscctrlFlag::xosc_ready);
    bench.verdict("an interrupt source arms", (Oscctrl::armed() & OscctrlFlag::xosc_ready) != 0u);
    Oscctrl::disarm(OscctrlFlag::all);
    bench.verdict("and disarms", Oscctrl::armed() == 0u);

    Oscctrl::failure_event(true);
    bench.verdict("EVCTRL.CFDEO reads back", Oscctrl::failure_event());
    Oscctrl::failure_event(false);
    bench.verdict("and clears again", !Oscctrl::failure_event());
    print(serial, "  the CFD's EVSYS generator code is ",
          Oscctrl::failure_generator, crlf);

    // The XOSC refusals that no register could express.
    bench.verdict("a crystal past the chapter's 32 MHz ceiling is refused",
                  !Xosc::config_valid(XoscConfig{.hz = 40'000'000}));
    bench.verdict("and one below its 0.4 MHz floor",
                  !Xosc::config_valid(XoscConfig{.hz = 100'000}));
    bench.verdict("a STARTUP past its four-bit field is refused",
                  !Xosc::config_valid(XoscConfig{.hz = crystal_hz, .startup = 16}));
    bench.verdict("this board's 24 MHz crystal is a legal configuration",
                  Xosc::config_valid(XoscConfig{.hz = crystal_hz}));
    bench.verdict("and the gain chooser picks the top code for it",
                  xosc_gain_for(crystal_hz) == XoscGain::up_to_32mhz);
    bench.verdict("the safe clock is chosen no faster than the crystal",
                  (sys_hz >> cfd_prescaler_for(crystal_hz, sys_hz)) <= crystal_hz);

    // The DPLL refusals. The DCO's floor is the one worth stating twice:
    // the output prescaler divides what the DCO made and is not a way
    // below 48 MHz.
    bench.verdict("a DPLL asked for a 24 MHz DCO is refused",
                  !Fdpll::config_valid(FdpllConfig{.reference = DpllReference::gclk,
                                                   .reference_hz = 2'000'000,
                                                   .ldr = 11}));
    bench.verdict("a reference past 2 MHz is refused",
                  !Fdpll::config_valid(FdpllConfig{.reference = DpllReference::gclk,
                                                   .reference_hz = 4'000'000,
                                                   .ldr = 11}));
    bench.verdict("DPLLCTRLB.DIV on a reference that has no divider is refused",
                  !Fdpll::config_valid(FdpllConfig{.reference = DpllReference::gclk,
                                                   .reference_hz = 2'000'000,
                                                   .xosc_div = 1,
                                                   .ldr = 23}));

    // The ratio arithmetic, which letter d then checks against silicon.
    constexpr DpllRatio r48 = dpll_ratio_for(2'000'000, 48'000'000);
    constexpr DpllRatio r49 = dpll_ratio_for(2'000'000, 49'000'000);
    bench.verdict("48 MHz from a 2 MHz reference is LDR 23, no fraction",
                  r48.ldr == 23 && r48.frac == 0 && r48.exact);
    bench.verdict("49 MHz is the same LDR and eight sixteenths",
                  r49.ldr == 23 && r49.frac == 8 && r49.exact &&
                      r49.actual_hz == 49'000'000);
    bench.verdict("a rate off the sixteenth grid comes back INEXACT, with the "
                  "frequency it really produces",
                  !dpll_ratio_for(2'000'000, 48'050'000).exact &&
                      dpll_ratio_for(2'000'000, 48'050'000).actual_hz == 48'000'000);

    bench.verdict("the DPLL is off, as this board has always left it",
                  !Fdpll::enabled());
    print(serial, "  XOSCCTRL=", hex(Xosc::reg()), " (reset value 0x0080)", crlf);
}

// =============================================================================
// b - the crystal, armed for the first time
// =============================================================================
//
// The board has carried this crystal since bring-up with nothing ever
// switched on. What comes out is two numbers that have never been
// measured here: how long it takes to start, and what the internal RC
// looks like beside it.
void tb_crystal() {
    bench.verdict("the coarse stopwatch comes up", stopwatch_up(TcPrescaler::div1024));

    // Start from a stopped oscillator whatever a previous letter left,
    // so the start-up time measured is a real start-up.
    (void)GenXtal::configure(GclkConfig{.source = GclkSource::osc48m});
    Xosc::stop();
    wait_ms(5);
    bench.verdict("the oscillator is stopped before it is timed", !Xosc::enabled());

    const uint16_t t0 = Stopwatch::count16();
    const bool started =
        Xosc::init(XoscConfig{.hz = crystal_hz, .startup = crystal_startup});
    const uint16_t t1 = Stopwatch::count16();
    bench.verdict("THE 24 MHz CRYSTAL STARTS - the first time on this board",
                  started);
    bench.verdict("and STATUS.XOSCRDY says so", Xosc::ready());

    const uint32_t ticks = static_cast<uint16_t>(t1 - t0);
    const uint32_t start_us =
        static_cast<uint32_t>((static_cast<uint64_t>(ticks) * 1'000'000ULL) / coarse_hz);
    print(serial, "  start-up: ", ticks, " coarse ticks = ", start_us,
          " us, against table 20-5's ", xosc_startup_us(crystal_startup),
          " us of MASKING for STARTUP ", crystal_startup, crlf);
    // The masking counter is a floor, not the answer: the oscillator has
    // to be swinging when the mask lifts, so the measurement can only be
    // longer. A measurement SHORTER than the mask would mean the mask is
    // not what the chapter says.
    bench.verdict("the start-up is at least the programmed masking time",
                  start_us + 100u >= xosc_startup_us(crystal_startup));
    // A band, not a coin toss: a 24 MHz crystal on this board starts in
    // well under a millisecond of its own, and the masking counter adds
    // its half. What would be a FINDING is tens of milliseconds - that
    // is a crystal fighting its load capacitors.
    bench.verdict("and is milliseconds, not tens of milliseconds",
                  start_us < 3000u);

    bench.verdict("generator 5 takes the crystal, divided by 250", crystal_up());

    // MEASUREMENT ONE: the internal RC on the crystal's scale.
    const auto rc = measure_count(gen_sys, gen_xtal, refnum_full);
    bench.verdict("OSC48M measures against the crystal", rc.has_value());

    // MEASUREMENT TWO: the same RC on OSCULP32K's scale - a second
    // witness with a different oscillator behind it. OSCULP32K is
    // running always (21.6.6); nothing has to be started.
    bench.verdict("generator 6 takes OSCULP32K",
                  GenUlp::configure(GclkConfig{.source = GclkSource::osculp32k}));
    const uint8_t ulp_refnum = Freqm::refnum_for(sys_hz / 32768u);
    const auto rc_ulp = measure_count(gen_sys, gen_ulp, ulp_refnum);
    bench.verdict("OSC48M measures against OSCULP32K too", rc_ulp.has_value());

    if (rc && rc_ulp) {
        const uint32_t rc_hz = count_to_hz(*rc, xtal_ref_hz, refnum_full);
        const uint32_t rc_ulp_hz = count_to_hz(*rc_ulp, 32768u, ulp_refnum);
        // Turned around: what the crystal measures if OSC48M is believed
        // to be exactly 48 MHz. The same reading, read the other way.
        const uint32_t xtal_hz =
            static_cast<uint32_t>((static_cast<uint64_t>(sys_hz) * refnum_full *
                                   xtal_div) / *rc);

        print(serial, "  count=", *rc, " (predicted ", expect_count(sys_hz), ")",
              crlf);
        print(serial, "  OSC48M against the CRYSTAL      : ", rc_hz, " Hz, ",
              ppm_off(rc_hz, sys_hz), " ppm ", rc_hz > sys_hz ? "FAST" : "slow",
              crlf);
        print(serial, "  OSC48M against OSCULP32K        : ", rc_ulp_hz, " Hz, ",
              ppm_off(rc_ulp_hz, sys_hz), " ppm ",
              rc_ulp_hz > sys_hz ? "FAST" : "slow", crlf);
        print(serial, "  the crystal, if OSC48M were exact: ", xtal_hz, " Hz, ",
              ppm_off(xtal_hz, crystal_hz), " ppm from 24 MHz", crlf);

        // THE HEADLINE. A quartz crystal is tens of ppm; an open-loop RC
        // is per mille. So the two determinations of OSC48M must
        // DISAGREE by about as much as the RC is off, and the one made
        // against the crystal is the one to believe.
        bench.verdict("OSC48M lands within 1% of nominal on the crystal's scale",
                      ppm_off(rc_hz, sys_hz) < 10'000u);
        bench.verdict("OSCULP32K's verdict on the same oscillator is FAR "
                      "coarser - it is an ultra-low-power RC, not a reference",
                      ppm_off(rc_ulp_hz, sys_hz) > ppm_off(rc_hz, sys_hz));

        // The crystal read back through the RC has to agree with the RC
        // read back through the crystal - it is one ratio, and this is
        // the arithmetic checking itself.
        bench.verdict("the two readings of one ratio agree",
                      near(ppm_off(xtal_hz, crystal_hz), ppm_off(rc_hz, sys_hz),
                           200u));

        // AND THE CONSEQUENCE FOR EVERYTHING MEASURED HERE BEFORE. Every
        // absolute frequency this stratum has reported was a ratio
        // against OSC48M multiplied by a NOMINAL 48 MHz - the frequency
        // meter's own suites, and the watchdog timing in
        // test_samc_platform, which rides SysTick and therefore the same
        // oscillator. The crystal is the first scale here that does not
        // come from that RC, so OSCULP32K can be weighed properly:
        // measured against OSC48M as before, but multiplied by what
        // OSC48M REALLY is.
        const uint32_t ulp_nominal_scale =
            count_to_hz(*rc_ulp, 32768u, ulp_refnum);
        const uint32_t ulp_hz = static_cast<uint32_t>(
            (static_cast<uint64_t>(ulp_refnum) * rc_hz) / *rc_ulp);
        print(serial, "  OSCULP32K on the CRYSTAL's scale : ", ulp_hz, " Hz (",
              ppm_off(ulp_hz, 32768u), " ppm from 32768)", crlf);
        print(serial, "  ... where the 48 MHz nominal made it look like ",
              static_cast<uint32_t>((static_cast<uint64_t>(ulp_refnum) * sys_hz) /
                                    *rc_ulp),
              " Hz", crlf);
        (void)ulp_nominal_scale;
        bench.verdict("OSCULP32K is nearer nominal than the RC-scaled readings "
                      "of earlier suites made it look",
                      ppm_off(ulp_hz, 32768u) <
                          ppm_off(static_cast<uint32_t>(
                                      (static_cast<uint64_t>(ulp_refnum) * sys_hz) /
                                      *rc_ulp),
                                  32768u));
    }

    Stopwatch::release();
    bench.verdict("the generators are moved off the crystal before it stops",
                  crystal_down());
}

// =============================================================================
// c - the clock failure detector, with a real failure
// =============================================================================
//
// A failure with no wire, and the trick is the chapter's own: XTALEN
// chooses between a crystal amplifier across XIN/XOUT and a DIGITAL
// INPUT on XIN alone (20.6.2). Clear it with a crystal attached and
// nothing is driving XIN - so the XOSC output stops having edges, which
// is exactly what the detector is built to notice.
//
// NO GENERATOR IS SOURCED FROM THE XOSC while this runs, deliberately: a
// generator on a source that stops is unroutable until reset
// (16.6.2.6), and this letter is about the detector, not about surviving
// that.
void tc_failure_detector() {
    bench.verdict("the crystal is running", crystal_up());
    // Everything off the crystal again - the detector does not need a
    // consumer, and a consumer would be the one thing at risk.
    bench.verdict("generator 5 is moved back to OSC48M first",
                  GenXtal::configure(GclkConfig{.source = GclkSource::osc48m}));

    const CfdPrescaler p = cfd_prescaler_for(crystal_hz, sys_hz);
    print(serial, "  safe clock = OSC48M / 2^", p, " = ", sys_hz >> p, " Hz", crlf);

    // 20.6.3: start the safe clock's source BEFORE the detector. On this
    // family OSC48M is the CPU's own clock, so it always is - the driver
    // refuses the configuration if it ever is not.
    bench.verdict("the safe clock's source is ready, as 20.6.3 requires",
                  Osc48m::ready());
    bench.verdict("the detector is armed with the crystal already running",
                  Xosc::init(XoscConfig{.hz = crystal_hz,
                                        .startup = crystal_startup,
                                        .failure_detector = true,
                                        .cfd_prescaler = p}));
    bench.verdict("CFDEN reads back", Xosc::failure_detector());
    bench.verdict("CFDPRESC reads back", Xosc::cfd_prescaler() == p);

    Oscctrl::clear_flags(OscctrlFlag::xosc_fail);
    wait_ms(5);
    bench.verdict("a healthy crystal raises no failure", !Xosc::failing());
    bench.verdict("and no latched flag either",
                  (Oscctrl::flags() & OscctrlFlag::xosc_fail) == 0u);
    bench.verdict("nor has anything switched to the safe clock",
                  !Xosc::switched_to_safe_clock());

    // THE FAILURE. XTALEN cleared: XIN becomes a digital input with
    // nothing driving it.
    OSCCTRL_REGS->OSCCTRL_XOSCCTRL = static_cast<uint16_t>(
        Xosc::reg() & static_cast<uint16_t>(~OSCCTRL_XOSCCTRL_XTALEN_Msk));
    wait_ms(20);

    const bool failing = Xosc::failing();
    const bool latched = (Oscctrl::flags() & OscctrlFlag::xosc_fail) != 0u;
    const bool switched = Xosc::switched_to_safe_clock();
    print(serial, "  with XTALEN cleared: STATUS.XOSCFAIL=", failing ? 1 : 0,
          " INTFLAG=", latched ? 1 : 0, " STATUS.XOSCCKSW=", switched ? 1 : 0,
          crlf);
    bench.verdict("A REAL CLOCK FAILURE IS DETECTED, with no wire touched",
                  failing);
    bench.verdict("and it latches in INTFLAG, where STATUS only tracks",
                  latched);
    bench.verdict("the detector switched the XOSC output to the safe clock - "
                  "erratum 1.22.1 is the N-family row, not this silicon",
                  switched);

    // THE RECOVERY. The amplifier back, the crystal given time, then
    // SWBEN - which the hardware clears when the switch happens.
    OSCCTRL_REGS->OSCCTRL_XOSCCTRL =
        static_cast<uint16_t>(Xosc::reg() | OSCCTRL_XOSCCTRL_XTALEN_Msk);
    wait_ms(20);
    Oscctrl::clear_flags(OscctrlFlag::xosc_fail);
    wait_ms(5);
    bench.verdict("with the amplifier back the failure clears", !Xosc::failing());

    Xosc::switch_back();
    wait_ms(5);
    bench.verdict("SWBEN is consumed by the hardware", !Xosc::switch_back_pending());
    bench.verdict("and the output is back on the crystal",
                  !Xosc::switched_to_safe_clock());

    // The event output, which samc/evsys.hpp can carry but nothing here
    // consumes - the code is published, the fabric is another driver's.
    Oscctrl::failure_event(true);
    bench.verdict("the failure event output enables", Oscctrl::failure_event());
    Oscctrl::failure_event(false);

    Xosc::failure_detector(false);
    bench.verdict("the detector disarms again", !Xosc::failure_detector());
    bench.verdict("the crystal is handed back cleanly", crystal_down());
}

// =============================================================================
// d - the DPLL, locked to the crystal
// =============================================================================
void td_dpll() {
    bench.verdict("the crystal is running", crystal_up());
    bench.verdict("the fine stopwatch comes up", stopwatch_up(TcPrescaler::div16));

    // ERRATUM 1.3.4: the internal lock timer's own clock has to be
    // running or an on-the-fly ratio change does nothing. OSCULP32K on
    // generator 6 is what it wants.
    bench.verdict("generator 6 takes OSCULP32K for the lock timer",
                  GenUlp::configure(GclkConfig{.source = GclkSource::osculp32k}));
    bench.verdict("GCLK_DPLL_32K is connected - erratum 1.3.4's precondition",
                  Fdpll::lock_timer_clock(gen_ulp));

    // 24 MHz / (2 x 6) = 2 MHz reference, x 24 = 48 MHz DCO.
    constexpr FdpllConfig cfg48{
        .reference = DpllReference::xosc,
        .reference_hz = crystal_hz,
        .xosc_div = 5,
        .ldr = 23,
    };
    static_assert(Fdpll::divided_reference_hz(cfg48) == 2'000'000);
    static_assert(Fdpll::dco_hz(cfg48) == 48'000'000);

    const uint16_t t0 = Stopwatch::count16();
    const bool up = Fdpll::init<cfg48>();
    const uint16_t t1 = Stopwatch::count16();
    bench.verdict("THE DPLL LOCKS TO THE CRYSTAL", up);
    bench.verdict("and DPLLSTATUS.LOCK confirms it", Fdpll::wait_locked());
    bench.verdict("with no lock time-out raised", !Fdpll::timed_out());

    const uint32_t lock_us = static_cast<uint32_t>(
        (static_cast<uint64_t>(static_cast<uint16_t>(t1 - t0)) * 1'000'000ULL) /
        fine_hz);
    print(serial, "  enable to CLKRDY: ", lock_us,
          " us, against table 45-52's 25..35 us at a 2 MHz reference", crlf);
    bench.verdict("the lock time is in the tens of microseconds the table "
                  "gives for a 2 MHz reference",
                  lock_us < 500u);

    bench.verdict("generator 4 takes the DPLL",
                  GenDpll::configure(GclkConfig{.source = GclkSource::dpll96m}));

    // THE MEASUREMENT THAT SETTLES THE ARITHMETIC. Both the DPLL and the
    // reference come from the same crystal, so the crystal's own error
    // cancels: what is left is the ratio the LDR/LDRFRAC pair claims.
    const auto c48 = measure_count(gen_dpll, gen_xtal, refnum_full);
    bench.verdict("the DPLL measures against the crystal", c48.has_value());
    if (c48) {
        const uint32_t want = expect_count(Fdpll::output_hz(cfg48));
        print(serial, "  LDR 23  gives count ", *c48, ", predicted ", want, " (",
              count_to_hz(*c48, xtal_ref_hz, refnum_full), " Hz)", crlf);
        bench.verdict("48 MHz from 24 MHz is exact to the meter's resolution",
                      near(*c48, want, 50u));
    }

    // THE FRACTIONAL RATIO, changed ON THE FLY. Two errata meet here:
    // 1.3.4 (the lock timer, connected above) and 1.3.3 (the completion
    // shows in INTFLAG.DPLLLDRTO and NOT in the status bit of the same
    // name, which is what Fdpll::ratio_updated reads).
    const bool ratio_ok = Fdpll::set_ratio(23, 8);
    bench.verdict("an on-the-fly ratio change reports complete through "
                  "INTFLAG - erratum 1.3.3's workaround as code",
                  ratio_ok);
    // 1.3.3 the other way round: the STATUS copy is the one that lies.
    const bool status_bit = (Oscctrl::status() & OSCCTRL_STATUS_DPLLLDRTO_Msk) != 0u;
    print(serial, "  after the change: INTFLAG.DPLLLDRTO=1, STATUS.DPLLLDRTO=",
          status_bit ? 1 : 0, "  (erratum 1.3.3 says the status bit stays 0)",
          crlf);
    (void)Fdpll::wait_locked();
    const auto c49 = measure_count(gen_dpll, gen_xtal, refnum_full);
    bench.verdict("the fractional ratio measures", c49.has_value());
    if (c49) {
        const uint32_t want = expect_count(49'000'000UL);
        print(serial, "  LDR 23 + 8/16 gives count ", *c49, ", predicted ", want,
              " (", count_to_hz(*c49, xtal_ref_hz, refnum_full), " Hz)", crlf);
        bench.verdict("A SIXTEENTH OF THE REFERENCE IS A REAL STEP: 49 MHz, "
                      "not 48 and not 50",
                      near(*c49, want, 50u));
    }

    // The top of the DCO's range, brought back down by the OUTPUT
    // PRESCALER - the distinction the block diagram makes and the text
    // does not repeat.
    constexpr FdpllConfig cfg96{
        .reference = DpllReference::xosc,
        .reference_hz = crystal_hz,
        .xosc_div = 5,
        .ldr = 47,
        .prescaler = DpllPrescaler::div2,
    };
    static_assert(Fdpll::dco_hz(cfg96) == 96'000'000);
    static_assert(Fdpll::output_hz(cfg96) == 48'000'000);
    bench.verdict("the DPLL restarts with a 96 MHz DCO and a divide-by-two",
                  Fdpll::init<cfg96>());
    const auto c96 = measure_count(gen_dpll, gen_xtal, refnum_full);
    bench.verdict("and measures", c96.has_value());
    if (c96) {
        const uint32_t want = expect_count(48'000'000UL);
        print(serial, "  DCO 96 MHz / 2 gives count ", *c96, ", predicted ", want,
              crlf);
        bench.verdict("a 96 MHz DCO divided by two IS 48 MHz out",
                      near(*c96, want, 50u));
    }
    bench.verdict("the prescaler moves on a running loop",
                  Fdpll::prescaler(DpllPrescaler::div4));
    const auto c24 = measure_count(gen_dpll, gen_xtal, refnum_full);
    bench.verdict("the quartered output measures", c24.has_value());
    if (c24) {
        const uint32_t want = expect_count(24'000'000UL);
        print(serial, "  DCO 96 MHz / 4 gives count ", *c24, ", predicted ", want,
              crlf);
        bench.verdict("and it is a quarter of the DCO", near(*c24, want, 50u));
    }

    // THE GCLK REFERENCE PATH. A generator is the third reference the
    // block offers, and it is the one that makes the DPLL reachable on a
    // board with no crystal at all.
    bench.verdict("generator 3 becomes a 1 MHz reference",
                  GenRef1M::configure(GclkConfig{.source = GclkSource::osc48m,
                                                 .div = 48}));
    bench.verdict("GCLK_DPLL is pointed at it", Fdpll::reference_clock(gen_ref_1m));
    constexpr FdpllConfig cfg_gclk{
        .reference = DpllReference::gclk,
        .reference_hz = 1'000'000,
        .ldr = 47,
    };
    static_assert(Fdpll::dco_hz(cfg_gclk) == 48'000'000);
    bench.verdict("the DPLL locks to a GENERATOR instead of the crystal",
                  Fdpll::init<cfg_gclk>());
    const auto cg = measure_count(gen_dpll, gen_xtal, refnum_full);
    bench.verdict("and measures", cg.has_value());
    if (cg) {
        // This one is NOT crystal-derived, so the crystal's own error is
        // in the answer and the band has to allow for the RC that made
        // the reference. A per-cent band is the honest one.
        const uint32_t want = expect_count(48'000'000UL);
        print(serial, "  GCLK reference 1 MHz x 48 gives count ", *cg,
              ", predicted ", want,
              "  (this path is RC-derived: the band is per cent, not ppm)", crlf);
        bench.verdict("48 MHz from a 1 MHz generator, inside the RC's own error",
                      near(*cg, want, want / 50u));
    }

    // THE LOCK TIMER, which table 20-3 describes and no sentence of the
    // chapter names as an outcome. With LTIME non-zero the output clock
    // is released when the TIMER reaches zero rather than when the loop
    // locks, so the interesting question is what INTFLAG.DPLLLTO then
    // means: a failure to lock, or simply the timer finishing.
    const bool lt_up = Fdpll::init<FdpllConfig{.reference = DpllReference::gclk,
                                               .reference_hz = 1'000'000,
                                               .ldr = 47,
                                               .lock_time = DpllLockTime::ms8}>();
    const bool lt_locked = Fdpll::locked();
    const bool lt_flag = Fdpll::timed_out();
    print(serial, "  LTIME 8 ms: CLKRDY=", lt_up ? 1 : 0, " LOCK=",
          lt_locked ? 1 : 0, " INTFLAG.DPLLLTO=", lt_flag ? 1 : 0, crlf);
    bench.verdict("a lock time-out configuration still produces an output "
                  "clock", lt_up);
    bench.verdict("and the loop is genuinely locked underneath it", lt_locked);

    Stopwatch::release();
    GclkChannel::disconnect(Fdpll::gclk_reference);
    GclkChannel::disconnect(Fdpll::gclk_lock_timer);
    bench.verdict("everything is moved off the DPLL and the crystal before "
                  "either stops",
                  crystal_down());
    bench.verdict("and the DPLL is off", !Fdpll::enabled());
}

// =============================================================================
// e - the CPU running from the DPLL
// =============================================================================
//
// The question this letter answers is not "can the DPLL make 48 MHz" -
// letter d settled that - but whether generator 0 can be moved onto it
// and back with the program still running. It is deliberately NOT a
// permanent retarget: `Clock<>` still implements the internal
// oscillator only, and which root CLK_MAIN takes is a design decision
// reserved for the day this target gets a DynamicClock.
//
// The safety argument, in full, because a wrong move here bricks the
// board until a power cycle:
//   - the DPLL is proven to be producing 48 MHz BEFORE generator 0 is
//     moved, by the same measurement letter d makes;
//   - LBYPASS is set (erratum 1.25.1, the driver's default), so a
//     spurious unlock cannot gate the clock away underneath the CPU;
//   - OSC48M is never stopped, so moving BACK always has a ready source
//     to move to, which is what 16.6.2.6 requires;
//   - both rates are 48 MHz, so the flash wait states are already right
//     for either and 27.5.2 has nothing to order.
void te_cpu_on_dpll() {
    bench.verdict("the crystal is running", crystal_up());
    bench.verdict("the lock timer's clock is connected",
                  GenUlp::configure(GclkConfig{.source = GclkSource::osculp32k}) &&
                      Fdpll::lock_timer_clock(gen_ulp));

    constexpr FdpllConfig cfg48{
        .reference = DpllReference::xosc,
        .reference_hz = crystal_hz,
        .xosc_div = 5,
        .ldr = 23,
    };
    bench.verdict("the DPLL is up at 48 MHz", Fdpll::init<cfg48>());
    bench.verdict("and locked", Fdpll::wait_locked());
    bench.verdict("the lock bypass is set, so a spurious unlock cannot take "
                  "the CPU's clock away - erratum 1.25.1",
                  (OSCCTRL_REGS->OSCCTRL_DPLLCTRLB & OSCCTRL_DPLLCTRLB_LBYPASS_Msk) != 0u);

    bench.verdict("generator 4 takes the DPLL",
                  GenDpll::configure(GclkConfig{.source = GclkSource::dpll96m}));
    const auto before = measure_count(gen_dpll, gen_xtal, refnum_full);
    const uint32_t want = expect_count(48'000'000UL);
    bench.verdict("the DPLL is PROVEN at 48 MHz before anything is moved",
                  before.has_value() && near(*before, want, 50u));

    if (!before || !near(*before, want, 50u)) {
        // The precondition failed: do not move the CPU's clock onto a
        // loop that is not doing what it claims.
        print(serial, "  DPLL not proven - the switch is NOT attempted", crlf);
        bench.verdict("the switch is skipped rather than risked", false);
        (void)crystal_down();
        return;
    }

    // The flash wait states are already at the 48 MHz setting, because
    // that is what the CPU has been running at all along. Stated as a
    // verdict so a future rate change cannot quietly skip 27.5.2.
    bench.verdict("the flash wait states already cover 48 MHz",
                  FlashWaitStates::get() >= FlashWaitStates::for_hz(sys_hz));

    const uint32_t ms_before = Ticker::millis();

    // ---- THE SWITCH ----
    const bool moved = Gclk<gen_sys>::configure(GclkConfig{.source = GclkSource::dpll96m});
    const bool on_dpll = Gclk<gen_sys>::source() == GclkSource::dpll96m;

    // Measured from the other side: generator 0 is now the DPLL, and the
    // crystal-derived reference does not care what the CPU runs on.
    const auto during = measure_count(gen_sys, gen_xtal, refnum_full);

    // ---- AND BACK ----
    const bool back = Gclk<gen_sys>::configure(GclkConfig{.source = GclkSource::osc48m});
    const bool on_rc = Gclk<gen_sys>::source() == GclkSource::osc48m;

    bench.verdict("GENERATOR 0 MOVES ONTO THE DPLL - the CPU is running from "
                  "the crystal-locked loop",
                  moved && on_dpll);
    bench.verdict("generator 0 moves back to OSC48M", back && on_rc);

    if (during) {
        print(serial, "  CLK_CPU on the DPLL: count ", *during, ", predicted ",
              want, " (", count_to_hz(*during, xtal_ref_hz, refnum_full), " Hz)",
              crlf);
        bench.verdict("the CPU clock measured while it WAS the DPLL is the "
                      "crystal ratio, not the RC's",
                      near(*during, want, 50u));
    }

    const uint32_t ms_after = Ticker::millis();
    print(serial, "  SysTick advanced ", ms_after - ms_before,
          " ms across the round trip", crlf);
    bench.verdict("the tick kept running across both switches",
                  ms_after > ms_before);

    GclkChannel::disconnect(Fdpll::gclk_lock_timer);
    bench.verdict("everything is moved off the DPLL and the crystal before "
                  "either stops",
                  crystal_down());
    bench.verdict("the CPU is back on OSC48M and the DPLL is off",
                  Gclk<gen_sys>::source() == GclkSource::osc48m && !Fdpll::enabled());
}

// =============================================================================
// f - GCLK's two divider regimes, settled
// =============================================================================
//
// This letter measures no oscillator at all: generator 5 is sourced from
// OSC48M, the SAME clock generator 0 runs on, so the ratio the meter
// counts is the DIVIDER and nothing else - no reference error can enter.
//
// WHAT IS IN DOUBT. 16.8.3 says DIVSEL = 1 divides by 2^(N+1) "where N
// is the Division Factor Bits for the selected generator", which reads
// as a FIXED 512 on the eight-bit generators with DIV ignored - and that
// is what clock.hpp's GclkConfig said, on one earlier measurement. The
// SAMD-era reading of the same sentence is 2^(DIV+1). Three DIV values
// tell the two apart: at DIV = 8 they agree on 512, at DIV = 0 one says
// 2 and the other 512, at DIV = 3 one says 16 and the other 512.
void tf_divider() {
    // Generator 5 off the crystal and onto OSC48M, undivided first.
    bench.verdict("generator 5 takes OSC48M undivided",
                  GenXtal::configure(GclkConfig{.source = GclkSource::osc48m}));
    const auto c1 = measure_count(gen_sys, gen_xtal, refnum_full);
    bench.verdict("an undivided generator measures", c1.has_value());
    if (c1) {
        print(serial, "  DIVSEL=0 DIV=0   gives count ", *c1, " (ratio ",
              *c1 / refnum_full, ")", crlf);
        bench.verdict("DIV 0 with DIVSEL clear is UNDIVIDED, as 16.6.2.7 says",
                      near(*c1, refnum_full, 2u));
    }

    // The linear regime, which nobody disputes and which this suite's
    // own crystal reference depends on.
    bench.verdict("generator 5 takes OSC48M divided by 250",
                  GenXtal::configure(GclkConfig{.source = GclkSource::osc48m,
                                                .div = 250}));
    const auto c250 = measure_count(gen_sys, gen_xtal, refnum_full);
    bench.verdict("the linearly divided generator measures", c250.has_value());
    if (c250) {
        print(serial, "  DIVSEL=0 DIV=250 gives count ", *c250, " (ratio ",
              *c250 / refnum_full, ")", crlf);
        bench.verdict("the linear divider divides by DIV exactly",
                      near(*c250, 250u * refnum_full, 2u));
    }

    // THE REGIME IN DOUBT, at three points.
    uint32_t ratio0 = 0;
    uint32_t ratio3 = 0;
    uint32_t ratio8 = 0;
    for (uint16_t div : {uint16_t{0}, uint16_t{3}, uint16_t{8}}) {
        if (!GenXtal::configure(GclkConfig{.source = GclkSource::osc48m,
                                           .div = div,
                                           .div_pow2 = true})) {
            continue;
        }
        const auto c = measure_count(gen_sys, gen_xtal, refnum_full);
        if (!c) {
            continue;
        }
        const uint32_t ratio = *c / refnum_full;
        print(serial, "  DIVSEL=1 DIV=", div, "   gives count ", *c, " (ratio ",
              ratio, ", 2^(DIV+1) would be ", 1u << (div + 1u),
              ", a fixed 512 would be 512)", crlf);
        if (div == 0) ratio0 = ratio;
        if (div == 3) ratio3 = ratio;
        if (div == 8) ratio8 = ratio;
    }

    bench.verdict("all three DIVSEL measurements completed",
                  ratio0 != 0u && ratio3 != 0u && ratio8 != 0u);
    if (ratio0 != 0u && ratio3 != 0u && ratio8 != 0u) {
        const bool power_of_div = ratio0 == 2u && ratio3 == 16u && ratio8 == 512u;
        const bool fixed_512 = ratio0 == 512u && ratio3 == 512u && ratio8 == 512u;
        bench.verdict("the two readings of 16.8.3 are told apart - exactly one "
                      "of them describes this silicon",
                      power_of_div != fixed_512);
        print(serial, "  DIVSEL divides by ",
              power_of_div ? "2^(DIV+1) - the DIV value DOES count"
                           : (fixed_512 ? "a FIXED 512, DIV ignored"
                                        : "NEITHER rule - see the counts above"),
              crlf);
        bench.verdict("and it is 2^(DIV+1): DIV 0 halves, DIV 3 divides by 16, "
                      "DIV 8 by 512",
                      power_of_div);
    }


    // AND THE OTHER HALF OF THE SAME CLAIM. Generator 1 is the only one
    // with a SIXTEEN-bit DIV field (table 16-3), so if DIVSEL really
    // divided by 2^(field width + 1) it would divide generator 1 by
    // 131072 whatever DIV said. Under 2^(DIV+1) the same DIV = 8 gives
    // 512 here as it did on generator 5. The two answers are three
    // decades apart, so REFNUM is dropped to 100: 100 x 131072 would
    // still fit VALUE's 24 bits, and 255 x 131072 would not.
    constexpr uint8_t refnum_slow = 100;
    bench.verdict("generator 1 - the 16-bit one - takes OSC48M with DIVSEL "
                  "and DIV 8",
                  Gclk<1>::configure(GclkConfig{.source = GclkSource::osc48m,
                                                .div = 8,
                                                .div_pow2 = true}));
    const auto g1 = measure_count(gen_sys, 1, refnum_slow);
    bench.verdict("generator 1 measures", g1.has_value());
    if (g1) {
        const uint32_t ratio = *g1 / refnum_slow;
        print(serial, "  generator 1, DIVSEL=1 DIV=8 gives count ", *g1,
              " (ratio ", ratio,
              "; 2^(DIV+1) would be 512, 2^(width+1) would be 131072)", crlf);
        bench.verdict("the wide generator obeys the SAME rule - DIV counts, "
                      "the field width does not",
                      near(ratio, 512u, 2u));
    }
    (void)Gclk<1>::enable(false);

    bench.verdict("generator 5 is left undivided on OSC48M",
                  GenXtal::configure(GclkConfig{.source = GclkSource::osc48m}));
}

void banner() {
    print(serial, crlf,
          "test_samc_clock - SAMC21J18A XOSC + FDPLL96M (ch. 20), the 24 MHz "
          "crystal measured by FREQM, clk=", SysClock::hz, " Hz", crlf);
    bench.menu();
}

} // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the block, the arithmetic and the refusals", ta_block);
    bench.letter('b', "the 24 MHz crystal, armed and weighed", tb_crystal);
    bench.letter('c', "the clock failure detector, with a real failure",
                 tc_failure_detector);
    bench.letter('d', "the DPLL locked to the crystal", td_dpll);
    bench.letter('e', "the CPU running from the DPLL, and back", te_cpu_on_dpll);
    bench.letter('f', "GCLK's two divider regimes, settled", tf_divider);

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "OSC48M" : "FAILED",
              " tick=", tick_ok ? "SysTick" : "FAILED", crlf);
        banner();
    }
    bench.prompt();

    for (;;) {
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        print(serial, static_cast<char>(c), crlf);
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        bench.prompt();
    }
}
