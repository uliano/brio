// test_rp2350_clock - the reference bench suite for the RP2350's CLOCK
// TREE (rp2350/clock.hpp, datasheet chapter 8): the crystal, the ring
// oscillator with its randomiser, the low-power oscillator, both PLLs,
// the glitchless generators and the four with an aux mux alone, the tick
// generators, the frequency counter, the top-level gates, the resus
// circuit, the GPIO clock pins - and the drivers that live on clk_sys
// (the kernel timebase, delay_us, the console) through every switch of
// it.
//
// Every letter runs on BOTH ARCHITECTURES from this one source and is
// judged the same way. Where a verdict must say something different of
// the two halves, its own text says it - which happens twice here: the
// kernel timebase (SysTick on clk_sys under the Cortex-M33, the platform
// timer's comparator on a tick off clk_ref under Hazard3, which is why a
// rate change rebases one and not the other) and the ruler that judges
// delay_us, which on the Hazard3 half is the same counter the timebase
// rides.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test.
//
// NOTHING TO WIRE for `z`. The console is a Debug Probe's UART bridge on
// GP0 (TX) / GP1 (RX) = UART0. The two letters outside `z` use the clock
// link to a peer board: GPOUT0 (GP21) of one into GPIN0 (GP20) of the
// other, one wire each way and a common ground.
//
// TWO RULERS, AND THEY SHARE A CRYSTAL. The FREQUENCY COUNTER (8.1.4)
// counts any root or generator against clk_ref, which is the crystal
// after init(): every rate below is "so many crystal periods", the
// chip's own ratio check. The PLATFORM TIMER (rp2350/mtime.hpp) counts
// microseconds off a tick divided from the same clk_ref: it times the
// switches and judges the kernel timebase. Neither judges the crystal
// itself - that is what the two-board letters are for, one board's
// crystal counted by the other's.
//
// THE SWITCH DISCIPLINE, the same in every letter that moves clk_sys:
// the console is drained and its divisor set for the rate to come
// (Serial::rebase, BEFORE the switch - on PeriSource::sys clk_peri
// follows clk_sys), then Clock::init() of the new rate, then the
// timebase rebased; a failure of the switch is followed by a return to
// the boot rate before anything is printed. `z` always ends where it
// started: 150 MHz on the PLL, clk_peri on clk_sys.
//
// What is exercised, letter by letter:
//   a  the boot rate: the crystal STABLE at the delay init() wrote, the
//      PLL LOCKED at the ratio pll_config_for chose, the two glitchless
//      muxes where init() left them, clk_peri on clk_sys; then the
//      COUNTER on every root and generator this image has running
//   b  the crystal alone: Clock<crystal, 12 MHz> and back to the PLL,
//      the clk_sys count at each end, the switch durations on the ruler
//   c  the other PLL rates: 48, 100 and 133 MHz, each locked at its own
//      ratio (the registers read back against pll_config_for), counted
//      on clk_sys and on pll_sys, then back to 150
//   d  clk_peri ON THE CRYSTAL while clk_sys moves: PeriSource::crystal
//      at 150 MHz (the console re-divided for 12 MHz once), then clk_sys
//      to 48 MHz and back with NO further touch of the UART - every line
//      of this letter after the first switch is the proof
//   e  delay_us at every rate: a 900 us and a 100 us wait bracketed on
//      the ruler (at least, never early), a hundred waits judged against
//      the KERNEL TICKS as well, a whole tick refused, and a clock with
//      no rate refused
//   f  the kernel timebase across rates: 100 ticks against the ruler at
//      12, 48 and 150 MHz
//   g  the ring oscillator: counted, stopped (the counter reports it
//      dead), restarted, its divider doubled, its range stepped up and
//      back, and THE RANDOMISER this chip adds turned on and off - every
//      one of them counted
//   h  the divider under a crystal-fed console: clk_sys / 2 and a
//      fractional 2.5, the console alive through both; and the PLL asked
//      for a VCO below its range, with clk_sys parked on clk_ref
//   i  THE PARK-FIRST INIT, from every starting tree a previous image
//      can leave: clk_sys on the ring oscillator, on the crystal, on the
//      USB PLL, and clk_sys parked on a clk_ref that is the 32 kHz
//      low-power oscillator - init() comes back from each
//   j  the low-power oscillator: counted against the crystal, its
//      declared rate read, its trim moved to both ends and restored
//   k  the USB PLL and the three generators it feeds: clk_usb, clk_adc
//      and clk_hstx counted, a divider on clk_hstx, then all of them
//      stopped and counted dead
//   l  the tick generators: the six of them, the running one's cycle
//      count against the crystal's megahertz, one started and stopped,
//      and a count that does not fit the field refused
//   m  the top-level clock gates: one destination's wake gate closed and
//      ENABLED read back, then reopened
//   n  the frequency counter itself: one rate at three interval codes,
//      the hardware's own PASS / SLOW / FAST verdict between two bounds,
//      and a source that is not there reported dead
//   q  the resus circuit: armed, FORCED, clk_sys found on clk_ref, the
//      resus cleared and the tree restored
//
//   o  (by name only) THE CLOCK OUTPUT: GPOUT0 on GP21 carries the
//      crystal divided by 12 - 1 MHz - and stays on when the letter
//      returns, for the peer board to count
//   p  (by name only) THE CLOCK INPUT: GPIN0 on GP20 counted against
//      this board's crystal; with the peer's output on the wire the
//      count is the RATIO OF THE TWO CRYSTALS, judged within 0.1 %
//      (two consumer crystals are each within 50 ppm); with nothing on
//      the wire the counter reports the source dead
//
// build: boards = weact2350b,weact2350b-rv
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/delay.hpp"
#include "rp2350/mtime.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/ticker.hpp"
#include "rp2350/uart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using Boot = Clock<ClockSource::pll, 150'000'000UL>;
using R12 = Clock<ClockSource::crystal, 12'000'000UL>;
using R48 = Clock<ClockSource::pll, 48'000'000UL>;
using R100 = Clock<ClockSource::pll, 100'000'000UL>;
using R133 = Clock<ClockSource::pll, 133'000'000UL>;
using X150 = Clock<ClockSource::pll, 150'000'000UL, 12'000'000UL, PeriSource::crystal>;
using X48 = Clock<ClockSource::pll, 48'000'000UL, 12'000'000UL, PeriSource::crystal>;
constexpr Boot clock;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;

using Led = Pin<25>;

TestBench<Serial> bench;

/// The USB PLL's 48 MHz, which letters i and k both want.
constexpr PllConfig usb_pll = pll_config_for(Boot::xtal_hz, clk_usb_hz);

uint32_t us_now() { return Mtime::micros(); }

void console_drain() {
    const uint32_t t0 = us_now();
    while (!Serial::tx_idle() && us_now() - t0 < 200'000u) {
    }
}

/// Within `permille` per mille of `expected`.
bool near(uint32_t value, uint32_t expected, uint32_t permille) {
    const uint32_t tolerance = static_cast<uint32_t>(
        (static_cast<uint64_t>(expected) * permille) / 1000u);
    return value + tolerance >= expected && value <= expected + tolerance;
}

const char* source_name(CountSource s) {
    switch (s) {
        case CountSource::xosc: return "xosc";
        case CountSource::clk_ref: return "clk_ref";
        case CountSource::pll_sys: return "pll_sys";
        case CountSource::pll_usb: return "pll_usb";
        case CountSource::clk_sys: return "clk_sys";
        case CountSource::clk_peri: return "clk_peri";
        case CountSource::clk_usb: return "clk_usb";
        case CountSource::clk_adc: return "clk_adc";
        case CountSource::clk_hstx: return "clk_hstx";
        case CountSource::rosc: return "rosc";
        case CountSource::rosc_ph: return "rosc_ph";
        case CountSource::lposc: return "lposc";
        case CountSource::gpin0: return "gpin0";
        case CountSource::gpin1: return "gpin1";
        default: return "?";
    }
}

/// Count `what`, print it, and say whether it sits within `permille` of
/// `expected` (0 expected = print only, no verdict).
bool count(const char* label, CountSource what, uint32_t expected, uint32_t permille = 1) {
    const std::optional<uint32_t> hz = Boot::count_hz(what);
    if (!hz) {
        print(serial, "  ", label, " ", source_name(what), ": DEAD (STATUS.DIED)", crlf);
        if (expected != 0u) {
            bench.verdict(label, " counts", false);
        }
        return false;
    }
    print(serial, "  ", label, " ", source_name(what), " = ", *hz, " Hz");
    if (expected == 0u) {
        print(serial, crlf);
        return true;
    }
    print(serial, " (", expected, " claimed)", crlf);
    const bool ok = near(*hz, expected, permille);
    bench.verdict(label, " counts within tolerance of the claim", ok);
    return ok;
}

/// The bare number, for a letter that prints its own line.
uint32_t counted(CountSource what) {
    const std::optional<uint32_t> hz = Boot::count_hz(what);
    return hz ? *hz : 0u;
}

/// The switch discipline (the file header): the console re-divided for
/// the rate to come, the switch timed, the timebase rebased. False, and
/// the board back at the boot rate, when init() failed.
template <typename C>
bool switch_to(C, const char* name, uint32_t* took_us = nullptr) {
    Serial::rebase(C::pclk_hz);
    const uint32_t t0 = us_now();
    const bool ok = C::init();
    const uint32_t took = us_now() - t0;
    if (!ok) {
        Serial::rebase(Boot::pclk_hz);
        (void)Boot::init();
        Ticker::rebase(Boot::hz);
        print(serial, "  switch to ", name, " FAILED after ", took, " us: back at the boot rate",
              crlf);
        return false;
    }
    Ticker::rebase(C::hz);
    if (took_us != nullptr) {
        *took_us = took;
    }
    return true;
}

/// Back to the boot clock from wherever a letter left the tree, with the
/// console and the timebase put right. Nothing is printed before it.
bool restore_boot() {
    Serial::rebase(Boot::pclk_hz);
    const bool ok = Boot::init();
    Ticker::rebase(Boot::hz);
    return ok;
}

// -----------------------------------------------------------------------------
void ta_boot() {
    print(serial, "  XOSC: ", Xosc::enabled() ? "ENABLED " : "off ",
          Xosc::stable() ? "STABLE" : "unstable", " delay=", Xosc::startup_delay(), " (",
          Boot::startup_delay, " written) range code ", Xosc::range(), crlf);
    bench.verdict("the crystal is enabled and stable at the startup delay init() wrote",
                  Xosc::enabled() && Xosc::stable() && Xosc::startup_delay() == Boot::startup_delay);
    const PllConfig cfg = PllSys::config();
    print(serial, "  PLL_SYS: ", PllSys::locked() ? "LOCKED" : "unlocked", " refdiv=", cfg.refdiv,
          " fbdiv=", cfg.fbdiv, " postdiv=", cfg.postdiv1, "x", cfg.postdiv2, " (", Boot::pll.fbdiv,
          " / ", Boot::pll.postdiv1, " x ", Boot::pll.postdiv2, " chosen), lock lost since init: ",
          PllSys::lock_lost() ? "YES" : "no", crlf);
    bench.verdict("the PLL is locked at the ratio pll_config_for chose",
                  PllSys::locked() && cfg == Boot::pll);
    bench.verdict("and it has not lost that lock since init() cleared the sticky flag",
                  !PllSys::lock_lost());
    print(serial, "  clk_sys mux: ", Clocks::sys_source() == 1u ? "aux" : "clk_ref", " div=",
          Clocks::sys_divider(), ".", Clocks::sys_divider_frac(), "; clk_ref mux: ",
          Clocks::ref_source(), " div=", Clocks::ref_divider(), "; clk_peri: ",
          Clocks::peri_enabled() ? "enabled" : "OFF", " div=", Clocks::peri_divider(), crlf);
    bench.verdict("clk_sys is on its aux (the PLL) undivided and clk_peri is enabled on clk_sys",
                  Clocks::sys_source() == 1u && Clocks::sys_divider() == 1u &&
                      Clocks::peri_enabled() && Clocks::peri_source() == PeriAux::clk_sys);
    bench.verdict("clk_ref is on the crystal, undivided, as init() left it",
                  Clocks::ref_source() == static_cast<uint8_t>(RefSource::xosc) &&
                      Clocks::ref_divider() == 1u);
    bench.verdict("the ring oscillator is still running (the resus circuit's clock, and what "
                  "init() parks on)",
                  Rosc::running());
    // ENABLE and FREQ_RANGE are twelve-bit passwords in both
    // oscillators, and BADWRITE is what says one was written wrongly.
    // It is also a flag no processor reset clears, so what stands here
    // is history: the letter prints it, clears it, and runs the WHOLE of
    // init() again to judge this image's own writes.
    const bool inherited_xosc = Xosc::badwrite();
    const bool inherited_rosc = Rosc::badwrite();
    console_drain();
    Xosc::clear_badwrite();
    Rosc::clear_badwrite();
    const bool again = Boot::init();
    Ticker::rebase(Boot::hz);
    print(serial, "  BADWRITE as this boot inherited it: xosc ",
          inherited_xosc ? "SET" : "clear", ", rosc ", inherited_rosc ? "SET" : "clear",
          "; cleared and init() run again: xosc ", Xosc::badwrite() ? "SET" : "clear", ", rosc ",
          Rosc::badwrite() ? "SET" : "clear", crlf);
    bench.verdict("a whole re-run of init() raises neither oscillator's BADWRITE - every write "
                  "of its CTRL is a whole word with both passwords legal",
                  again && !Xosc::badwrite() && !Rosc::badwrite());

    count("the crystal", CountSource::xosc, Boot::xtal_hz);
    count("clk_ref", CountSource::clk_ref, Boot::xtal_hz);
    count("the PLL", CountSource::pll_sys, Boot::hz);
    count("clk_sys", CountSource::clk_sys, Boot::hz);
    count("clk_peri", CountSource::clk_peri, Boot::pclk_hz);
    count("the ring oscillator", CountSource::rosc, 0);
    count("the low-power oscillator", CountSource::lposc, 0);
}

// -----------------------------------------------------------------------------
void tb_crystal() {
    uint32_t down = 0;
    uint32_t up = 0;
    if (!switch_to(R12{}, "the crystal alone", &down)) {
        bench.verdict("Clock<crystal, 12 MHz>::init()", false);
        return;
    }
    print(serial, "  at 12 MHz on the crystal: the switch took ", down, " us", crlf);
    bench.verdict("Clock<crystal>::init() leaves clk_sys on clk_ref (the glitchless mux, "
                  "no aux)",
                  Clocks::sys_source() == 0u);
    count("clk_sys on the crystal", CountSource::clk_sys, R12::hz);
    count("clk_peri following it", CountSource::clk_peri, R12::pclk_hz);
    bench.verdict("the PLL is no longer what clk_sys runs on, and stays locked (nothing "
                  "stopped it)",
                  PllSys::locked());

    if (!switch_to(Boot{}, "the PLL at 150 MHz", &up)) {
        bench.verdict("back to the PLL", false);
        return;
    }
    print(serial, "  back at 150 MHz: the switch took ", up, " us", crlf);
    count("clk_sys back on the PLL", CountSource::clk_sys, Boot::hz);
    bench.verdict("both switches took under a millisecond (the crystal was kept, "
                  "not restarted)",
                  down < 1000u && up < 1000u);
}

// -----------------------------------------------------------------------------
template <typename C>
void pll_rate(C c, const char* name) {
    uint32_t took = 0;
    if (!switch_to(c, name, &took)) {
        bench.verdict(name, false);
        return;
    }
    const PllConfig cfg = PllSys::config();
    print(serial, "  ", name, ": fbdiv=", cfg.fbdiv, " postdiv=", cfg.postdiv1, "x", cfg.postdiv2,
          " (VCO ", static_cast<uint32_t>(C::xtal_hz / 1'000'000u) * C::pll.fbdiv,
          " MHz), the switch took ", took, " us", crlf);
    bench.verdict(name, ": locked at pll_config_for's ratio", PllSys::locked() && cfg == C::pll);
    count(name, CountSource::pll_sys, C::hz);
    count(name, CountSource::clk_sys, C::hz);
}

void tc_rates() {
    pll_rate(R48{}, "48 MHz");
    pll_rate(R100{}, "100 MHz");
    pll_rate(R133{}, "133 MHz");
    if (!switch_to(Boot{}, "150 MHz (the ceiling)")) {
        bench.verdict("back to 150 MHz", false);
        return;
    }
    count("back at 150 MHz", CountSource::clk_sys, Boot::hz);
}

// -----------------------------------------------------------------------------
void td_peri() {
    if (!switch_to(X150{}, "150 MHz with clk_peri on the crystal")) {
        bench.verdict("PeriSource::crystal", false);
        return;
    }
    print(serial, "  the console now divides 12 MHz (", X150::pclk_hz, " Hz claimed for clk_peri)",
          crlf);
    bench.verdict("clk_peri is on the crystal", Clocks::peri_source() == PeriAux::xosc);
    count("clk_peri on the crystal", CountSource::clk_peri, X150::pclk_hz);
    count("clk_sys meanwhile", CountSource::clk_sys, X150::hz);

    // clk_sys moves; the UART is NOT touched (pclk_hz is the same).
    console_drain();
    const uint32_t t0 = us_now();
    const bool ok = X48::init();
    const uint32_t took = us_now() - t0;
    Ticker::rebase(X48::hz);
    print(serial, "  clk_sys moved to 48 MHz in ", took, " us with the UART untouched - this "
          "line is the proof", crlf);
    bench.verdict("Clock<pll, 48 MHz, crystal-fed clk_peri>::init() switched", ok);
    count("clk_peri, unchanged", CountSource::clk_peri, X48::pclk_hz);
    count("clk_sys at 48", CountSource::clk_sys, X48::hz);

    console_drain();
    const bool back = X150::init();
    Ticker::rebase(X150::hz);
    print(serial, "  and back to 150 MHz, the UART still untouched", crlf);
    bench.verdict("back at 150 MHz on the crystal-fed clk_peri", back);
    count("clk_sys at 150", CountSource::clk_sys, X150::hz);

    if (!switch_to(Boot{}, "the boot clock")) {
        bench.verdict("back to clk_peri on clk_sys", false);
        return;
    }
    bench.verdict("clk_peri back on clk_sys", Clocks::peri_source() == PeriAux::clk_sys);
}

// -----------------------------------------------------------------------------
template <typename C>
void delay_at(C c, const char* name) {
    if (!switch_to(c, name)) {
        bench.verdict(name, false);
        return;
    }
    (void)delay_us(c, 5);   // the cold call, spent
    const uint32_t t0 = us_now();
    const bool long_ok = delay_us(c, 900);
    const uint32_t took = us_now() - t0;
    const uint32_t s0 = us_now();
    const bool short_ok = delay_us(c, 100);
    const uint32_t short_took = us_now() - s0;
    // The ruler steps once a microsecond and its phase against the call
    // is unknown, so the wait itself is one count long; the bracket adds
    // two more reads and the calls, at this rate's price.
    const uint32_t overhead_us = 2u + 500'000'000u / C::hz;
    print(serial, "  at ", name, ": delay_us(900) took ", took, " us, delay_us(100) took ",
          short_took, " us (", overhead_us, " us of grain and bracket allowed)", crlf);
    bench.verdict(name, ": 900 us at least, and the grain at most over",
                  long_ok && took >= 900u && took <= 900u + overhead_us);
    bench.verdict(name, ": 100 us at least, the same grain",
                  short_ok && short_took >= 100u && short_took <= 100u + overhead_us);
    bench.verdict(name, ": a whole kernel tick (1000 us) is refused", !delay_us(c, 1000));
}

void te_delay() {
    delay_at(R12{}, "12 MHz");
    delay_at(R48{}, "48 MHz");
    delay_at(R100{}, "100 MHz");
    delay_at(R133{}, "133 MHz");
    delay_at(Boot{}, "150 MHz");
    if (!switch_to(Boot{}, "150 MHz")) {
        bench.verdict("back to 150 MHz", false);
        return;
    }
    // A hundred waits judged against the OTHER timebase - independent of
    // the ruler on the Cortex-M33 half, where the kernel tick is SysTick
    // on clk_sys; the same counter on the Hazard3 half, where it is the
    // platform timer's comparator, and the verdict is then arithmetic.
    console_drain();
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() == t0) {
    }
    const uint32_t start = Ticker::ticks();
    for (uint16_t i = 0; i < 100u; ++i) {
        (void)delay_us(clock, 900);
    }
    const uint32_t ticks = Ticker::ticks() - start;
    print(serial, "  a hundred 900 us waits spanned ", ticks, " kernel ticks (90 at least)", crlf);
    bench.verdict("a hundred 900 us waits are at least 90 ms of kernel time, and under 95",
                  ticks >= 90u && ticks < 95u);
    bench.verdict("a clock that claims no rate is refused, and spends nothing",
                  !delay_us(delay_rate(0u), 10u));
    bench.verdict("a wait of nothing is served at once", delay_us(clock, 0u));
}

// -----------------------------------------------------------------------------
template <typename C>
void ticker_at(C c, const char* name) {
    if (!switch_to(c, name)) {
        bench.verdict(name, false);
        return;
    }
    console_drain();
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() == t0) {
    }
    const uint32_t u0 = us_now();
    const uint32_t t1 = Ticker::ticks();
    while (Ticker::ticks() - t1 < 100u) {
    }
    const uint32_t span = us_now() - u0;
    print(serial, "  at ", name, ": 100 ticks spanned ", span, " us", crlf);
    bench.verdict(name, ": the timebase keeps 1 ms per tick within 0.1 % (rebased on the "
                  "Cortex-M33 half, untouched on the Hazard3 one)",
                  near(span, 100'000u, 1));
}

void tf_ticker() {
    ticker_at(R12{}, "12 MHz");
    ticker_at(R48{}, "48 MHz");
    ticker_at(Boot{}, "150 MHz");
}

// -----------------------------------------------------------------------------
void tg_rosc() {
    Rosc::clear_badwrite();
    const uint8_t boot_div = Rosc::divider();
    const uint32_t before = counted(CountSource::rosc);
    print(serial, "  the ring oscillator: ", before, " Hz, range code ",
          hex(Rosc::range_code()), ", divider ", boot_div, ", randomised: ",
          Rosc::randomised() ? "yes" : "no", crlf);
    bench.verdict("the ring oscillator counts inside the range the datasheet guarantees "
                  "(4.6 to 19.6 MHz at the boot settings)",
                  before >= 4'600'000u && before <= 19'600'000u);

    Rosc::stop();
    const std::optional<uint32_t> dead = Boot::count_hz(CountSource::rosc, 10);
    print(serial, "  stopped: ", Rosc::running() ? "still ENABLED" : "ENABLED clear",
          ", the counter says ", dead ? "a count of " : "DEAD (STATUS.DIED)", dead ? *dead : 0u,
          dead ? " Hz" : "", crlf);
    bench.verdict("Rosc::stop() clears ENABLED", !Rosc::running());
    bench.verdict("the counter reports a stopped source dead or under 1 kHz",
                  !dead || *dead < 1000u);
    const bool restarted = Rosc::start();
    const uint32_t after = counted(CountSource::rosc);
    print(serial, "  restarted: ", restarted ? "STABLE" : "not stable", ", ", after, " Hz", crlf);
    bench.verdict("Rosc::start() brings it back stable", restarted && Rosc::running());
    bench.verdict("and it counts within 20 % of before", near(after, before, 200));

    // The divider: doubling it halves the output. Nothing is clocked
    // from this oscillator here - clk_ref is on the crystal - so a
    // glitch on the way costs nothing.
    Rosc::divider(static_cast<uint8_t>(boot_div * 2u));
    const uint32_t halved = counted(CountSource::rosc);
    print(serial, "  divider ", boot_div, " -> ", Rosc::divider(), ": ", halved, " Hz", crlf);
    bench.verdict("the output divider halves the rate within 2 %", near(halved * 2u, after, 20));
    Rosc::divider(boot_div);

    // The stage range, one step at a time (8.3.5): up does not glitch,
    // down does, and MEDIUM is about 1.33 times LOW.
    Rosc::range(RoscRange::medium);
    const uint32_t medium = counted(CountSource::rosc);
    Rosc::range(RoscRange::low);
    const uint32_t back_low = counted(CountSource::rosc);
    print(serial, "  range LOW ", after, " Hz -> MEDIUM ", medium, " Hz -> LOW ", back_low, " Hz",
          crlf);
    bench.verdict("the MEDIUM range is faster than LOW, and LOW comes back where it was",
                  medium > after && near(back_low, after, 50));

    // THE RANDOMISER, which the RP2040 had not: the first two stages'
    // drive strengths taken from an LFSR, worth up to 22 per cent in
    // this range.
    Rosc::seed(0xA5A5'1234u);
    Rosc::randomise(true, true);
    const uint32_t random_both = counted(CountSource::rosc);
    Rosc::randomise(true, false);
    const uint32_t random_one = counted(CountSource::rosc);
    Rosc::randomise(false, false);
    const uint32_t plain = counted(CountSource::rosc);
    print(serial, "  randomiser: both stages ", random_both, " Hz, one ", random_one,
          " Hz, neither ", plain, " Hz", crlf);
    bench.verdict("randomising both stages raises the rate, one stage less, and clearing it "
                  "puts the rate back",
                  random_both > plain && random_one > plain && random_both >= random_one &&
                      near(plain, after, 50));
    bench.verdict("no write of this letter was refused for a bad password - CTRL is written "
                  "whole, never through an atomic alias",
                  !Rosc::badwrite());
    bench.verdict("and the oscillator is back at the divider and range it booted with",
                  Rosc::divider() == boot_div && near(counted(CountSource::rosc), before, 50));
}

// -----------------------------------------------------------------------------
void th_divider_and_refusal() {
    if (!switch_to(X150{}, "150 MHz with clk_peri on the crystal")) {
        bench.verdict("PeriSource::crystal", false);
        return;
    }
    console_drain();
    Clocks::sys_divider(2);
    Ticker::rebase(X150::hz / 2u);
    print(serial, "  clk_sys divided by 2 under a crystal-fed console: this line is the proof",
          crlf);
    count("clk_sys / 2", CountSource::clk_sys, X150::hz / 2u);
    count("clk_peri meanwhile", CountSource::clk_peri, X150::pclk_hz);

    // The fractional half of the divider, which the RP2040's clk_sys had
    // in 256ths and this one has in 65536ths: 2.5 is 60 MHz.
    console_drain();
    Clocks::sys_divider(2, 0x8000u);
    Ticker::rebase(X150::hz / 5u * 2u);
    print(serial, "  clk_sys divided by 2.5 (the 16.16 divider): this line too", crlf);
    count("clk_sys / 2.5", CountSource::clk_sys, 60'000'000u, 2);
    bench.verdict("the fractional half of the divider reads back as written",
                  Clocks::sys_divider() == 2u && Clocks::sys_divider_frac() == 0x8000u);
    Clocks::sys_divider(1);
    Ticker::rebase(X150::hz);
    count("clk_sys undivided again", CountSource::clk_sys, X150::hz);

    // The PLL asked for the impossible, with clk_sys parked on clk_ref
    // (the crystal, 12 MHz) so nothing runs on the PLL meanwhile.
    console_drain();
    const bool parked = Clocks::sys_from_ref();
    Ticker::rebase(X150::xtal_hz);
    bench.verdict("clk_sys parks on clk_ref", parked);
    const PllConfig low_vco{.refdiv = 1, .fbdiv = 16, .postdiv1 = 7, .postdiv2 = 7};   // 192 MHz VCO
    const uint32_t t0 = us_now();
    const bool locked = PllSys::init(low_vco);
    const uint32_t took = us_now() - t0;
    print(serial, "  the PLL at a 192 MHz VCO (below the 750 MHz floor): LOCK ",
          locked ? "ROSE" : "did not rise", " in ", took, " us (printed, not judged: the "
          "datasheet promises nothing outside the range)", crlf);

    const bool restored = restore_boot();
    bench.verdict("the boot clock restored after the parking and the refusal", restored);
    count("clk_sys restored", CountSource::clk_sys, Boot::hz);
    bench.verdict("clk_peri back on clk_sys", Clocks::peri_source() == PeriAux::clk_sys);
}

// -----------------------------------------------------------------------------
/// One starting tree: the caller has already put it there and drained
/// the console; init() must come back from it.
void park_first_from(const char* what, uint32_t expected_from_hz) {
    const uint32_t from = counted(CountSource::clk_sys);
    const uint32_t t0 = us_now();
    const bool ok = restore_boot();
    const uint32_t took = us_now() - t0;
    const uint32_t now = counted(CountSource::clk_sys);
    print(serial, "  from ", what, ": clk_sys was ", from, " Hz, init() took ", took,
          " us, clk_sys is now ", now, " Hz", crlf);
    bench.verdict("init() comes back to 150 MHz from ", what,
                  ok && near(now, Boot::hz, 1) &&
                      (expected_from_hz == 0u || near(from, expected_from_hz, 50)));
}

void ti_park_first() {
    // 1. clk_sys on the ring oscillator - the tree the bootrom leaves.
    console_drain();
    (void)Clocks::sys_from_aux(SysAux::rosc);
    park_first_from("the ring oscillator", 0u);

    // 2. clk_sys on the crystal through its AUX mux (not through
    //    clk_ref): another thing a previous image can leave standing.
    console_drain();
    (void)Clocks::sys_from_aux(SysAux::xosc);
    park_first_from("the crystal on the aux mux", Boot::xtal_hz);

    // 3. clk_sys on the USB PLL at 48 MHz - a PLL this image did not
    //    lock for clk_sys, which is exactly the case init() must survive.
    console_drain();
    if (PllUsb::init(usb_pll) && Clocks::sys_from_aux(SysAux::pll_usb)) {
        park_first_from("the USB PLL at 48 MHz", clk_usb_hz);
    } else {
        (void)restore_boot();
        bench.verdict("the USB PLL as clk_sys", false);
    }
    PllUsb::stop();

    // 4. THE EXTREME: clk_sys parked on clk_ref and clk_ref on the
    //    32 kHz low-power oscillator. The machine runs at 32768 Hz for
    //    the length of init()'s first steps - nothing may be printed
    //    there, and on the Hazard3 half the kernel timebase's own tick
    //    slows with clk_ref until init() puts it back.
    console_drain();
    (void)Clocks::sys_from_ref();
    const bool on_lposc = Clocks::ref_select(RefSource::lposc);
    const bool ok = restore_boot();
    const uint32_t now = counted(CountSource::clk_sys);
    print(serial, "  from clk_sys parked on a clk_ref that is the 32 kHz low-power "
          "oscillator: clk_sys is now ", now, " Hz", crlf);
    bench.verdict("clk_ref took the low-power oscillator (the fourth root this chip adds)",
                  on_lposc);
    bench.verdict("init() comes back to 150 MHz from a 32 kHz machine",
                  ok && near(now, Boot::hz, 1));
    bench.verdict("and clk_ref is the crystal again", counted(CountSource::clk_ref) != 0u &&
                                                      near(counted(CountSource::clk_ref),
                                                           Boot::xtal_hz, 1));
}

// -----------------------------------------------------------------------------
void tj_lposc() {
    const uint8_t trim = Lposc::trim();
    print(serial, "  the low-power oscillator: trim ", trim, ", mode ", Lposc::mode(),
          ", declared ", Lposc::declared_hz(), " Hz (", Lposc::freq_khz_int(), " kHz + ",
          Lposc::freq_khz_frac(), "/65536)", crlf);
    const uint32_t at_reset = counted(CountSource::lposc);
    print(serial, "  counted against the crystal: ", at_reset, " Hz (32768 nominal, plus or "
          "minus 20 % untrimmed)", crlf);
    bench.verdict("it runs, and within the 20 % the datasheet allows an untrimmed part",
                  near(at_reset, lposc_nominal_hz, 200));
    bench.verdict("what it TELLS the always-on timer is its reset value, 32.768 kHz",
                  near(Lposc::declared_hz(), lposc_nominal_hz, 1));

    Lposc::trim(0u);
    const uint32_t slowest = counted(CountSource::lposc);
    Lposc::trim(63u);
    const uint32_t fastest = counted(CountSource::lposc);
    Lposc::trim(trim);
    const uint32_t back = counted(CountSource::lposc);
    print(serial, "  trim 0 -> ", slowest, " Hz, trim 63 -> ", fastest, " Hz, trim ", trim,
          " -> ", back, " Hz", crlf);
    bench.verdict("the trim moves the oscillator both ways and comes back where it was",
                  slowest < at_reset && fastest > at_reset && near(back, at_reset, 20));
    bench.verdict("no write of this letter was refused for a bad password (the POWMAN "
                  "registers take one in their top sixteen bits)",
                  !Lposc::bad_password());
}

// -----------------------------------------------------------------------------
void tk_usb_pll() {
    const bool locked = PllUsb::init(usb_pll);
    const PllConfig cfg = PllUsb::config();
    print(serial, "  PLL_USB: ", locked ? "LOCKED" : "unlocked", " fbdiv=", cfg.fbdiv, " postdiv=",
          cfg.postdiv1, "x", cfg.postdiv2, " (VCO ",
          static_cast<uint32_t>(Boot::xtal_hz / 1'000'000u) * usb_pll.fbdiv, " MHz)", crlf);
    bench.verdict("the USB PLL locks at 48 MHz on pll_config_for's ratio",
                  locked && cfg == usb_pll);
    count("the USB PLL", CountSource::pll_usb, clk_usb_hz);

    const bool usb_on = Clocks::usb_select(UsbAux::pll_usb);
    const bool adc_on = Clocks::adc_select(AdcAux::pll_usb);
    bench.verdict("clk_usb and clk_adc start on it (stopped, selected, restarted, their "
                  "ENABLED polled both ways)",
                  usb_on && adc_on && Clocks::usb_enabled() && Clocks::adc_enabled());
    count("clk_usb", CountSource::clk_usb, clk_usb_hz);
    count("clk_adc", CountSource::clk_adc, clk_adc_hz);

    // clk_hstx, the generator this chip has and the RP2040 had not.
    const bool hstx_on = Clocks::hstx_select(HstxAux::clk_sys);
    bench.verdict("clk_hstx starts on clk_sys", hstx_on && Clocks::hstx_enabled());
    count("clk_hstx", CountSource::clk_hstx, Boot::hz);
    (void)Clocks::hstx_select(HstxAux::clk_sys, 3u);
    print(serial, "  clk_hstx divided by ", Clocks::hstx_divider(), ":", crlf);
    count("clk_hstx / 3", CountSource::clk_hstx, Boot::hz / 3u, 2);

    Clocks::hstx_stop();
    Clocks::adc_stop();
    Clocks::usb_stop();
    PllUsb::stop();
    const std::optional<uint32_t> usb_dead = Boot::count_hz(CountSource::clk_usb, 10);
    const std::optional<uint32_t> hstx_dead = Boot::count_hz(CountSource::clk_hstx, 10);
    print(serial, "  stopped: clk_usb ", usb_dead ? "counts " : "DEAD", usb_dead ? *usb_dead : 0u,
          ", clk_hstx ", hstx_dead ? "counts " : "DEAD", hstx_dead ? *hstx_dead : 0u, crlf);
    bench.verdict("a stopped generator counts dead or zero, and its PLL is powered down",
                  (!usb_dead || *usb_dead == 0u) && (!hstx_dead || *hstx_dead == 0u) &&
                      PllUsb::stopped());
}

// -----------------------------------------------------------------------------
void tl_ticks() {
    const uint32_t want = tick_cycles_per_us(Boot::ref_hz);
    print(serial, "  the RISC-V platform timer's generator: ",
          TickGenerator<TickConsumer::riscv>::running() ? "RUNNING" : "stopped", ", ",
          TickGenerator<TickConsumer::riscv>::cycles(), " cycles of clk_ref per tick (", want,
          " for a microsecond off this crystal), countdown at ",
          TickGenerator<TickConsumer::riscv>::count(), crlf);
    bench.verdict("the ruler's tick generator runs at one microsecond of clk_ref",
                  TickGenerator<TickConsumer::riscv>::running() &&
                      TickGenerator<TickConsumer::riscv>::cycles() == want);
    print(serial, "  the other five: proc0 ",
          TickGenerator<TickConsumer::proc0>::running() ? "on" : "off", ", proc1 ",
          TickGenerator<TickConsumer::proc1>::running() ? "on" : "off", ", timer0 ",
          TickGenerator<TickConsumer::timer0>::running() ? "on" : "off", ", timer1 ",
          TickGenerator<TickConsumer::timer1>::running() ? "on" : "off", ", watchdog ",
          TickGenerator<TickConsumer::watchdog>::running() ? "on" : "off", crlf);

    // Core 1's generator is nobody's in this image: started, read back,
    // stopped again.
    using Spare = TickGenerator<TickConsumer::proc1>;
    const bool started = Spare::start(want);
    const uint32_t cycles = Spare::cycles();
    Spare::stop();
    bench.verdict("a generator nothing uses starts at the count it was given and stops again",
                  started && cycles == want && !Spare::running());
    bench.verdict("a count that does not fit the nine-bit field is refused, and a zero one",
                  !Spare::start(512u) && !Spare::start(0u));
}

// -----------------------------------------------------------------------------
void tm_gates() {
    const SleepClocks before = Clocks::wake_enables();
    const SleepClocks live = Clocks::enabled();
    print(serial, "  the wake gates: ", before.en0, " / ", before.en1, "; what stands now: ",
          live.en0, " / ", live.en1, crlf);
    bench.verdict("every gate is open, as it is at reset",
                  before == sleep_clocks_all && live == sleep_clocks_all);

    // One destination nothing in this image clocks: the SHA-256 block.
    constexpr SleepClocks sha{CLOCKS_WAKE_EN0_CLK_SYS_SHA256_BITS, 0u};
    Clocks::wake_enables(before & ~sha);
    const SleepClocks shut = Clocks::enabled();
    Clocks::wake_enables(before);
    const SleepClocks reopened = Clocks::enabled();
    print(serial, "  with the SHA-256 block's gate shut, ENABLED0 reads ", shut.en0,
          "; reopened, ", reopened.en0, crlf);
    bench.verdict("closing one destination's wake gate stops that clock and nothing else",
                  (shut.en0 & sha.en0) == 0u && (shut.en0 | sha.en0) == before.en0);
    bench.verdict("and opening it again puts every gate back", reopened == sleep_clocks_all);
}

// -----------------------------------------------------------------------------
void tn_counter() {
    // The same rate at three windows: the coarse ones are coarse by the
    // table of 8.1.4, and the grain is what the verdict allows.
    for (uint8_t interval = 5; interval <= 15u; interval = static_cast<uint8_t>(interval + 5u)) {
        const std::optional<uint32_t> hz = Boot::count_hz(CountSource::clk_sys, interval);
        const uint32_t grain = 2'048'000UL >> interval;
        print(serial, "  clk_sys over ", fc_interval_us(interval), " us (interval ", interval,
              "): ", hz ? *hz : 0u, " Hz, the table's grain ", grain, " Hz", crlf);
        bench.verdict("the count is within the interval's own grain of 150 MHz",
                      hz && *hz + grain + 1u >= Boot::hz && *hz <= Boot::hz + grain + 1u);
    }

    // The hardware's own verdict between two bounds.
    const FcMeasurement pass = FreqCounter::measure(CountSource::clk_sys, Boot::xtal_hz, 10,
                                                    149'000u, 151'000u);
    const FcMeasurement fast = FreqCounter::measure(CountSource::clk_sys, Boot::xtal_hz, 10,
                                                    1'000u, 2'000u);
    const FcMeasurement slow = FreqCounter::measure(CountSource::clk_sys, Boot::xtal_hz, 10,
                                                    400'000u, 500'000u);
    print(serial, "  the test mode on clk_sys: in 149..151 MHz -> pass ", pass.pass ? 1 : 0,
          "; against 1..2 MHz -> fast ", fast.fast ? 1 : 0, "; against 400..500 MHz -> slow ",
          slow.slow ? 1 : 0, crlf);
    bench.verdict("the counter's own PASS, FAST and SLOW answer the bounds they were given",
                  pass.pass && !pass.fast && !pass.slow && fast.fast && !fast.pass &&
                      slow.slow && !slow.pass);

    // A source that is not there: GPIN1's pad carries no clock function
    // in this image, so the multiplexer sees nothing at all.
    const FcMeasurement nothing = FreqCounter::measure(CountSource::gpin1, Boot::xtal_hz, 10, 0u,
                                                       CLOCKS_FC0_MAX_KHZ_BITS);
    print(serial, "  a source with nothing on it (gpin1): ", nothing.died ? "DIED" : "counted ",
          nothing.died ? 0u : nothing.hz, crlf);
    bench.verdict("a source that is not running is reported dead, or counts zero",
                  nothing.died || nothing.hz == 0u);
    const uint32_t t0 = us_now();
    const bool idle = FreqCounter::stop();
    const uint32_t took = us_now() - t0;
    print(serial, "  handed back: the counter went idle in ", took, " us", crlf);
    bench.verdict("the counter is idle once it is handed back - it finishes the window it is "
                  "in first",
                  idle && !FreqCounter::running());
}

// -----------------------------------------------------------------------------
void tq_resus() {
    bench.verdict("nothing has resuscitated this clk_sys yet", !Resus::resussed());
    console_drain();
    // The timeout is the longest the eight-bit field takes: a healthy
    // clk_sys must never trip it, and only the FORCE below does.
    Resus::enable(255u);
    Resus::force();
    // The forced selection puts clk_sys on clk_ref - the crystal - so
    // the console's divisor is wrong until the tree is restored, and
    // nothing may be printed here. RESUSSED is polled rather than read
    // once: the machine is at 12 MHz now, and the status is the block's
    // own clk_ref cycles away.
    uint32_t polls = 0;
    while (!Resus::resussed() && polls < 10'000u) {
        ++polls;
    }
    const bool resussed = Resus::resussed();
    const uint32_t forced_hz = counted(CountSource::clk_sys);
    Resus::unforce();
    Resus::clear();
    const bool restored = restore_boot();
    Resus::disable();
    print(serial, "  forced: RESUSSED ", resussed ? "stood after " : "did not stand in ", polls,
          " polls, clk_sys was ", forced_hz, " Hz (clk_ref, the crystal) - then cleared and the "
          "tree rebuilt", crlf);
    bench.verdict("a forced resus moves clk_sys onto clk_ref, which is the crystal",
                  near(forced_hz, Boot::xtal_hz, 1));
    bench.verdict("cleared, the tree comes back to 150 MHz", restored);
    count("clk_sys after the resus", CountSource::clk_sys, Boot::hz);
    bench.verdict("the circuit is disarmed again, and its status clear",
                  !Resus::enabled() && !Resus::resussed());
    bench.verdict("the resus interrupt is not armed by any of this (the line is the "
                  "program's to bind)",
                  !Resus::interrupt() && !Resus::pending());
}

// -----------------------------------------------------------------------------
void to_output() {
    const bool ok = ClockOut<0>::init(GpoutSource::xosc, 12);
    print(serial, "  GPOUT0 on GP", ClockOut<0>::pin, ": the crystal / 12 = 1 MHz, ",
          ClockOut<0>::enabled() ? "ENABLED" : "off", " and ",
          ClockOut<0>::running() ? "RUNNING" : "stopped",
          " - left on for the peer board to count", crlf);
    bench.verdict("ClockOut<0>::init(xosc, 12) started the generator",
                  ok && ClockOut<0>::enabled() && ClockOut<0>::running());
    bench.verdict("GP21 is on the clock function",
                  Pin<ClockOut<0>::pin>::function() == PinFunction::gpck);
    bench.verdict("the source and the divider read back as written",
                  ClockOut<0>::source() == GpoutSource::xosc && ClockOut<0>::divider() == 12u);
    bench.verdict("a zero divider is refused", !ClockOut<1>::init(GpoutSource::xosc, 0));
}

void tp_input() {
    (void)ClockIn<0>::init();
    const std::optional<uint32_t> hz = Boot::count_hz(ClockIn<0>::count_source);
    if (!hz || *hz == 0u) {
        print(serial, "  GPIN0 on GP", ClockIn<0>::pin, ": DEAD - nothing on the wire (the peer's "
              "clock output not running, or the link not wired)", crlf);
        bench.verdict("the peer's crystal arrives on GP20", false);
        (void)ClockIn<0>::release();
        return;
    }
    // The count is the peer's crystal / 12 in this crystal's units: the
    // ratio of the two.
    print(serial, "  GPIN0 on GP", ClockIn<0>::pin, " = ", *hz, " Hz = the peer's crystal / 12, "
          "counted against this board's crystal", crlf);
    bench.verdict("the peer's crystal arrives on GP20 within 0.1 % of 1 MHz (the ratio of the "
                  "two crystals)",
                  near(*hz, 1'000'000u, 1));
    (void)ClockIn<0>::release();
}

void banner() {
    print(serial, crlf, "test_rp2350_clock - the RP2350 clock tree (datasheet 8), boot clk=",
          Boot::hz, " Hz on the PLL, crystal ", Boot::xtal_hz, " Hz, on ",
          core_kind == CoreKind::cortex_m33 ? "the Cortex-M33" : "Hazard3", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_uart0() { (void)Serial::isr(); }
extern "C" void isr_systick() { brio::Ticker::tick(); }

int main() {
    const bool clock_ok = Boot::init();
    const bool ruler_ok = brio::Mtime::start(clock);
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    (void)Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the boot rate, the tree as init() left it, every clock counted", ta_boot);
    bench.letter('b', "the crystal alone and back", tb_crystal);
    bench.letter('c', "the other PLL rates: 48, 100, 133 MHz", tc_rates);
    bench.letter('d', "clk_peri on the crystal while clk_sys moves", td_peri);
    bench.letter('e', "delay_us at every rate", te_delay);
    bench.letter('f', "the kernel timebase rebased across rates", tf_ticker);
    bench.letter('g', "the ring oscillator: stopped, divided, ranged, randomised", tg_rosc);
    bench.letter('h', "the divider under a crystal-fed console; the PLL below its floor",
                 th_divider_and_refusal);
    bench.letter('i', "the park-first init, from every tree a previous image can leave",
                 ti_park_first);
    bench.letter('j', "the low-power oscillator, counted and trimmed", tj_lposc);
    bench.letter('k', "the USB PLL and the clk_usb / clk_adc / clk_hstx generators", tk_usb_pll);
    bench.letter('l', "the tick generators of the TICKS block", tl_ticks);
    bench.letter('m', "the top-level clock gates", tm_gates);
    bench.letter('n', "the frequency counter itself: intervals, bounds, a dead source",
                 tn_counter);
    bench.letter('q', "the resus circuit, armed and forced", tq_resus);
    bench.letter('o', "GPOUT0 on GP21 = the crystal / 12, left running", to_output, false);
    bench.letter('p', "GPIN0 on GP20 counted: the peer board's crystal", tp_input, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL150" : "FAILED",
                    " ruler=", ruler_ok ? "1us" : "FAILED", " tick=", tick_ok ? "1kHz" : "FAILED",
                    brio::crlf);
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
