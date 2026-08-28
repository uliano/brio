// test_samc_osc32k - the reference bench suite for samc/osc32kctrl.hpp.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the driver
// under it.
//
// NOTHING TO WIRE. Every oscillator here is on the die.
//
// THE INSTRUMENT IS THE FREQUENCY METER, which is why this suite could
// not have been written before samc/freqm.hpp existed. Each oscillator
// is routed to a GCLK generator and measured against OSC48M - a clock
// the build knows to a permille - so what comes out is hertz and not a
// ratio nobody can check. Three drivers meet in letter b: samc/nvm.hpp
// reads the production trim out of the NVM calibration area,
// samc/osc32kctrl.hpp writes it into the oscillator, and samc/freqm.hpp
// says what difference it made.
//
// What is exercised, letter by letter:
//   a  the block: the RTC clock select, the status and interrupt
//      surface, and the refusals the registers alone could not express
//   b  THE FACTORY TRIM, measured: OSC32K started untrimmed and then
//      retrimmed with the production value, with the meter on both
//   c  the three roots side by side, in hertz, and OSCULP32K's trim
//      shown to move it
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include "samc/clock.hpp"
#include "samc/freqm.hpp"
#include "samc/nvm.hpp"
#include "samc/osc32kctrl.hpp"
#include "samc/pin.hpp"
#include "samc/sercom.hpp"
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

/// The generator this suite points at whichever slow oscillator it is
/// measuring. Generator 0 (OSC48M) stays the reference.
constexpr uint8_t gen_slow = 5;
using GenSlow = Gclk<gen_slow>;

/// Measure whatever generator 5 is sourced from, in hertz, by making it
/// the MEASURAND and OSC48M the reference - the meter needs the
/// reference to be the slower of the two, so the ratio is read the other
/// way up. Returns nothing if the measurement did not complete.
std::optional<uint32_t> measure_slow_hz() {
    const uint8_t refnum = Freqm::refnum_for(SysClock::hz / 32768u);
    const FreqmConfig cfg{
        .measured_generator = 0,
        .reference_generator = gen_slow,
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
    // count = REFNUM x f_48M / f_slow  =>  f_slow = REFNUM x f_48M / count
    return static_cast<uint32_t>(
        (static_cast<uint64_t>(refnum) * SysClock::hz) / *count);
}

/// A few milliseconds for an oscillator or a generator to settle. The
/// sink is volatile so the loop survives -Os; the COUNTER is not, because
/// incrementing a volatile is deprecated in this dialect.
void settle() {
    volatile uint32_t sink = 0;
    for (uint32_t i = 0; i < 200'000UL; ++i) {
        sink = sink + 1u;
    }
}

/// Point generator 5 at a source and give it a moment to settle.
bool route_slow(GclkSource src) {
    if (!GenSlow::configure(GclkConfig{.source = src})) {
        return false;
    }
    settle();
    return GenSlow::enabled();
}

constexpr uint32_t nominal_hz = 32768;

uint32_t per_mille_off(uint32_t hz) {
    const uint32_t d = hz > nominal_hz ? hz - nominal_hz : nominal_hz - hz;
    return (d * 1000u) / nominal_hz;
}

// =============================================================================
// a - the block
// =============================================================================
void ta_block() {
    // RTCCTRL lives in THIS chapter even though the RTC is in another -
    // a thing worth asserting, because it is where a reader would not
    // look for it.
    Osc32kctrl::rtc_clock(RtcClock::ulp_32k);
    bench.verdict("the RTC's clock select reads back",
                  Osc32kctrl::rtc_clock() == RtcClock::ulp_32k);
    Osc32kctrl::rtc_clock(RtcClock::osc_1k);
    bench.verdict("and follows a second choice",
                  Osc32kctrl::rtc_clock() == RtcClock::osc_1k);
    Osc32kctrl::rtc_clock(RtcClock::ulp_1k);

    bench.verdict("OSCULP32K carries a factory trim, not zero",
                  Osculp32k::calib() != 0u);
    bench.verdict("and it is not locked, so this suite may move it",
                  !Osculp32k::locked());

    print(serial, "  status=", hex(Osc32kctrl::status()),
          " osculp trim=", Osculp32k::calib(),
          " osc32k factory trim=", Osc32k::factory_calib(), crlf);

    // THE FACTORY TRIM MUST BE A REAL VALUE. If the NVM calibration area
    // read all ones this whole suite would be measuring nonsense, so it
    // is checked before anything depends on it.
    bench.verdict("the OSC32K production trim reads as a real value, not "
                  "erased flash",
                  Osc32k::factory_calib() != 0x7Fu);

    // The refusals the registers alone could not express.
    bench.verdict("an oscillator with both outputs disabled is refused",
                  !Osc32k::config_valid(
                      Osc32kConfig{.enable_32k = false, .enable_1k = false}));
    bench.verdict("a trim past the seven-bit field is refused",
                  !Osc32k::config_valid(Osc32kConfig{.calib = 0x80}));
    bench.verdict("and so is a startup past its own field",
                  !Osc32k::config_valid(Osc32kConfig{.startup = 8}));
    bench.verdict("OSCULP32K refuses a trim past its five-bit field",
                  !Osculp32k::calib(0x20));

    // XOSC32K: THE BOARD HAS NO 32 kHz CRYSTAL, so this is what a
    // missing crystal looks like - a bounded wait that ends with the
    // ready flag still low, reported rather than hung on.
    const bool crystal = Xosc32k::init(Xosc32kConfig{.crystal = true}, 4'000'000UL);
    Xosc32k::stop();
    print(serial, "  XOSC32K start attempt -> ", crystal ? "READY" : "never ready",
          " (this board carries no 32 kHz crystal)", crlf);
    bench.verdict("a crystal that is not there is a false return and not a hang",
                  !crystal);
}

// =============================================================================
// b - the factory trim, measured
// =============================================================================
//
// 21.5.9 says the production calibration "MUST be loaded from the NVM
// Software Calibration Area into OSC32K.CALIB by software to achieve
// specified accuracy". That is a claim about how much the trim is worth,
// and the meter can settle it: start the oscillator untrimmed, measure,
// retrim with the production value, measure again.
void tb_factory_trim() {
    const uint8_t factory = Osc32k::factory_calib();

    // Untrimmed: CALIB left at zero, which is what a caller who never
    // read 21.5.9 would get.
    bench.verdict("OSC32K starts with the trim at zero",
                  Osc32k::init(Osc32kConfig{.calib = 0, .enable_32k = true}));
    bench.verdict("and reports itself ready", Osc32k::ready());
    bench.verdict("with the 32 kHz output enabled, or nothing could reach it",
                  (Osc32k::reg() & OSC32KCTRL_OSC32K_EN32K_Msk) != 0u);

    bench.verdict("the generator takes OSC32K as its source",
                  route_slow(GclkSource::osc32k));
    const auto untrimmed = measure_slow_hz();
    bench.verdict("and the untrimmed oscillator measures", untrimmed.has_value());

    // Trimmed with the production value, waiting for the commit that
    // 21.6.4 ties to STATUS.OSC32KRDY.
    bench.verdict("retrimming with the production value succeeds",
                  Osc32k::retrim(factory));
    bench.verdict("and the trim reads back", Osc32k::calib() == factory);
    const auto trimmed = measure_slow_hz();
    bench.verdict("the trimmed oscillator measures", trimmed.has_value());

    if (untrimmed && trimmed) {
        print(serial, "  OSC32K trim ", 0, " -> ", *untrimmed, " Hz (",
              per_mille_off(*untrimmed), " per mille off nominal)", crlf);
        print(serial, "  OSC32K trim ", factory, " -> ", *trimmed, " Hz (",
              per_mille_off(*trimmed), " per mille off nominal)", crlf);

        // THE VERDICT THAT MATTERS: the trim is worth something. If the
        // production value did not move the oscillator measurably,
        // 21.5.9's insistence would be empty.
        const uint32_t moved = *trimmed > *untrimmed ? *trimmed - *untrimmed
                                                     : *untrimmed - *trimmed;
        bench.verdict("the production trim MOVES the oscillator - 21.5.9 is "
                      "not decoration",
                      moved > nominal_hz / 200u);
        bench.verdict("and it moves it TOWARDS nominal",
                      per_mille_off(*trimmed) < per_mille_off(*untrimmed));
    }

    // POINT THE GENERATOR AWAY BEFORE STOPPING THE OSCILLATOR, and the
    // order is not tidiness. 16.6.2.6 releases a generator's old source
    // only once the new one is ready, so a generator still sourced from
    // a STOPPED oscillator cannot be moved: the GENCTRL write never
    // synchronizes and every later measurement returns nothing. The
    // first version of this suite stopped OSC32K here and letter c then
    // failed to route anything at all.
    bench.verdict("the generator is moved to an oscillator that is running "
                  "BEFORE the old one stops",
                  route_slow(GclkSource::osculp32k));
    Osc32k::stop();
    bench.verdict("the oscillator stops again", !Osc32k::enabled());
}

// =============================================================================
// c - the three roots side by side
// =============================================================================
void tc_roots() {
    const uint8_t factory = Osc32k::factory_calib();
    // Start from a generator sourced from something that is certainly
    // running, whatever the previous letter left behind.
    (void)route_slow(GclkSource::osculp32k);

    // OSCULP32K, as the factory left it.
    const uint8_t ulp_factory = Osculp32k::calib();
    bench.verdict("the generator takes OSCULP32K", route_slow(GclkSource::osculp32k));
    const auto ulp = measure_slow_hz();
    bench.verdict("OSCULP32K measures", ulp.has_value());

    // OSCULP32K retrimmed: the calibration is user-overridable (21.6.5),
    // and moving it must move the clock. This is also the oscillator the
    // WATCHDOG runs on, so the effect is not academic.
    const uint8_t moved_trim =
        ulp_factory > 8u ? static_cast<uint8_t>(ulp_factory - 8u)
                         : static_cast<uint8_t>(ulp_factory + 8u);
    bench.verdict("OSCULP32K accepts an override of its trim",
                  Osculp32k::calib(moved_trim));
    settle();
    const auto ulp_moved = measure_slow_hz();
    (void)Osculp32k::calib(ulp_factory);   // put it back before anything else runs
    bench.verdict("and the override changes the clock", ulp_moved.has_value());

    // OSC32K, trimmed properly this time.
    bench.verdict("OSC32K starts trimmed",
                  Osc32k::init(Osc32kConfig{.calib = factory, .enable_32k = true}));
    bench.verdict("the generator takes OSC32K", route_slow(GclkSource::osc32k));
    const auto osc = measure_slow_hz();
    bench.verdict("OSC32K measures", osc.has_value());
    // Away first, then stop - see letter b.
    (void)route_slow(GclkSource::osculp32k);
    Osc32k::stop();

    if (ulp && ulp_moved && osc) {
        print(serial, "  OSCULP32K trim ", ulp_factory, " -> ", *ulp, " Hz (",
              per_mille_off(*ulp), " per mille off)", crlf);
        print(serial, "  OSCULP32K trim ", moved_trim, " -> ", *ulp_moved,
              " Hz - the override moved it by ",
              *ulp_moved > *ulp ? *ulp_moved - *ulp : *ulp - *ulp_moved, " Hz",
              crlf);
        print(serial, "  OSC32K    trim ", factory, " -> ", *osc, " Hz (",
              per_mille_off(*osc), " per mille off)", crlf);

        const uint32_t d = *ulp_moved > *ulp ? *ulp_moved - *ulp : *ulp - *ulp_moved;
        bench.verdict("the OSCULP32K trim is a real knob, not a stored byte",
                      d > nominal_hz / 500u);

        // A VERDICT THIS SUITE DELIBERATELY DOES NOT MAKE. 21.6.5 says
        // OSCULP32K "should be preferred to the OSC32K whenever the
        // power requirements are prevalent over frequency stability and
        // accuracy", and an earlier version of this test read that as a
        // promise that a trimmed OSC32K lands nearer nominal. It does
        // not: on this die at room temperature the two come out within a
        // couple of per mille of each other and the ORDER FLIPS between
        // runs - so asserting it made a coin toss into a verdict. The
        // chapter is talking about stability across conditions, which a
        // single measurement at one operating point cannot see.
        //
        // What IS assertable is that both trimmed oscillators are near
        // nominal, which is the claim an application actually depends on.
        //
        // THE BAND IS 3%, NOT 1%, AND THE REFERENCE IS WHY. This number
        // is a ratio against OSC48M times a NOMINAL 48 MHz, and OSC48M
        // is itself an RC: measured 5100 ppm SLOW on this die
        // (test_samc_clock, against the crystal) with a +-5% standard
        // calibration spec and a thermal wander of its own. The first
        // version's 1% band was therefore mostly consumed by the
        // REFERENCE's error - it passed at +9.3 per mille one power-on
        // and failed at +11.5 the next, with the oscillator under test
        // blameless both times. 3% still fails an untrimmed OSC32K
        // (+44%) by an order of magnitude, which is this verdict's
        // actual job; the crystal-scale truth about these oscillators
        // lives in test_samc_clock and docs/samc/clock.md.
        bench.verdict("both trimmed oscillators land within 3% of nominal "
                      "(measured through OSC48M, whose own error dominates)",
                      per_mille_off(*osc) < 30u && per_mille_off(*ulp) < 30u);
        print(serial, "  the two are within ",
              per_mille_off(*osc) > per_mille_off(*ulp)
                  ? per_mille_off(*osc) - per_mille_off(*ulp)
                  : per_mille_off(*ulp) - per_mille_off(*osc),
              " per mille of each other - which of them is nearer nominal "
              "flips between runs, so this suite does not judge it", crlf);
    }

    bench.verdict("OSCULP32K's trim is back where the factory left it",
                  Osculp32k::calib() == ulp_factory);
}

void banner() {
    print(serial, crlf, "test_samc_osc32k - SAMC21J18A OSC32KCTRL (ch. 21) "
          "measured by FREQM, clk=", SysClock::hz, " Hz", crlf);
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

    bench.letter('a', "the block, the RTC select and the refusals", ta_block);
    bench.letter('b', "the factory trim, measured", tb_factory_trim);
    bench.letter('c', "the three roots side by side", tc_roots);

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
