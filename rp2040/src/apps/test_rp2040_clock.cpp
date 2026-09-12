// test_rp2040_clock - the reference bench suite for the RP2040's CLOCK
// TREE (rp2040/clock.hpp, datasheet 2.15 to 2.18): the crystal, the
// system PLL, the glitchless generators, clk_peri's independence, the
// ring oscillator, the frequency counter, the GPIO clock pins, and the
// drivers that live on clk_sys (the SysTick ticker, delay_us, a UART)
// through every switch of it.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test.
//
// NOTHING TO WIRE for `z`. The console is the Debug Probe's UART bridge
// on GP0 (TX) / GP1 (RX) = UART0. The two letters outside `z` use the
// clock link between two boards: GPOUT0 (GP21) of one into GPIN0 (GP20)
// of the other, one wire and a common ground.
//
// TWO RULERS. The FREQUENCY COUNTER (2.15.4) counts any root or
// generator against clk_ref, which is the crystal after init(): every
// rate below is "so many crystal periods", the chip's own ratio check.
// The SYSTEM TIMER (rp2040/timer.hpp) is the crystal's microseconds:
// it times the switches and judges delay_us and the ticker at each
// rate. Neither judges the crystal itself: that is what the two-board
// letters are for (one board's crystal counted by the other's).
//
// THE SWITCH DISCIPLINE, the same in every letter that moves clk_sys:
// the console is drained and its divisor set for the rate to come
// (Serial::rebase, BEFORE the switch - on PeriSource::sys clk_peri
// follows clk_sys), then Clock::init() of the new rate, then the
// ticker rebased; a failure of the switch is followed by a return to
// the boot rate before anything is printed. `z` always ends where it
// started: 125 MHz on the PLL, clk_peri on clk_sys.
//
// What is exercised, letter by letter:
//   a  the boot rate: the crystal STABLE at the delay init() wrote,
//      the PLL LOCKED at the ratio pll_config_for chose, the two
//      glitchless muxes where init() left them, clk_peri on clk_sys;
//      then the COUNTER on every root and generator - xosc, clk_ref,
//      pll_sys, clk_sys, clk_peri, rosc - each within 0.05 % of what
//      the tree claims (the ring oscillator only printed)
//   b  the crystal alone: Clock<crystal, 12 MHz> and back to the PLL,
//      the clk_sys count at each end, the switch durations on the timer
//   c  the other PLL rates: 48, 100 and 133 MHz, each locked at its
//      own ratio (the registers read back against pll_config_for),
//      counted on clk_sys and on pll_sys, then back to 125
//   d  clk_peri ON THE CRYSTAL while clk_sys moves: PeriSource::crystal
//      at 125 MHz (the console re-divided for 12 MHz once), then
//      clk_sys to 48 MHz and back with NO further touch of the UART -
//      every line of this letter after the first switch is the proof
//      that the UART kept its rate; clk_peri counted at 12 MHz
//      throughout
//   e  delay_us at every rate: 12, 48, 100, 125, 133 MHz, a 900 us
//      and a 100 us delay bracketed by the timer (at least, never
//      early, the bracket's own overhead - some 500 CPU cycles of
//      calls and timer reads - allowed at each rate's price), the
//      cold first call spent before the measure; a whole tick, 1000 us,
//      refused at every rate (the cap)
//   f  the SysTick ticker rebased across rates: 100 ticks against the
//      timer at 12, 48 and 133 MHz, within 0.1 %
//   g  the ring oscillator: counted, stopped (the counter reports it
//      DEAD), restarted, counted again within 20 % of before
//   h  the divider under a crystal-fed console: clk_sys / 2 with
//      clk_peri on the crystal, the console alive through it; and the
//      PLL asked for a VCO below its range, with clk_sys parked on
//      clk_ref meanwhile (what LOCK does then is printed, not judged);
//      the boot clock restored and counted
//
//   o  (by name only) THE CLOCK OUTPUT: GPOUT0 on GP21 carries the
//      crystal divided by 12 - 1 MHz - and stays on when the letter
//      returns, for the other board's `p`
//   p  (by name only) THE CLOCK INPUT: GPIN0 on GP20 counted against
//      this board's crystal; with the other board's `o` on the wire the
//      count is the RATIO OF THE TWO CRYSTALS, judged within 0.1 %
//      (two consumer crystals are each within 50 ppm); with nothing
//      on the wire the counter reports the source dead
//
// build: boards = pico,weact2040
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "rp2040/clock.hpp"
#include "rp2040/delay.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/ticker.hpp"
#include "rp2040/timer.hpp"
#include "rp2040/uart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using Boot = Clock<ClockSource::pll, 125'000'000>;
using R12 = Clock<ClockSource::crystal, 12'000'000>;
using R48 = Clock<ClockSource::pll, 48'000'000>;
using R100 = Clock<ClockSource::pll, 100'000'000>;
using R133 = Clock<ClockSource::pll, 133'000'000>;
using X125 = Clock<ClockSource::pll, 125'000'000, 12'000'000, PeriSource::crystal>;
using X48 = Clock<ClockSource::pll, 48'000'000, 12'000'000, PeriSource::crystal>;
constexpr Boot clock;
using P = Rp2040Platform<>;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;

using Led = Pin<25>;

TestBench<Serial> bench;

uint32_t us_now() { return Timer::now_low(); }

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
        case CountSource::clk_sys: return "clk_sys";
        case CountSource::clk_peri: return "clk_peri";
        case CountSource::rosc: return "rosc";
        case CountSource::gpin0: return "gpin0";
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

/// The switch discipline (the file header): the console re-divided for
/// the rate to come, the switch timed, the ticker rebased. False, and
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

// -----------------------------------------------------------------------------
void ta_boot() {
    print(serial, "  XOSC: ", Xosc::enabled() ? "ENABLED " : "off ", Xosc::stable() ? "STABLE" : "unstable",
          " delay=", Xosc::startup_delay(), " (", Boot::startup_delay, " written)", crlf);
    bench.verdict("the crystal is enabled and stable at the startup delay init() wrote",
                  Xosc::enabled() && Xosc::stable() && Xosc::startup_delay() == Boot::startup_delay);
    const PllConfig cfg = PllSys::config();
    print(serial, "  PLL_SYS: ", PllSys::locked() ? "LOCKED" : "unlocked", " refdiv=", cfg.refdiv,
          " fbdiv=", cfg.fbdiv, " postdiv=", cfg.postdiv1, "x", cfg.postdiv2, " (", Boot::pll.fbdiv,
          " / ", Boot::pll.postdiv1, " x ", Boot::pll.postdiv2, " chosen)", crlf);
    bench.verdict("the PLL is locked at the ratio pll_config_for chose",
                  PllSys::locked() && cfg == Boot::pll);
    print(serial, "  clk_sys mux: ", Clocks::sys_source() == 1u ? "aux" : "clk_ref", " div=",
          Clocks::sys_divider(), "; clk_peri: ", Clocks::peri_enabled() ? "enabled" : "OFF",
          " on ", Clocks::peri_source() == PeriAux::clk_sys ? "clk_sys" : "another source", crlf);
    bench.verdict("clk_sys is on its aux (the PLL) undivided and clk_peri is enabled on clk_sys",
                  Clocks::sys_source() == 1u && Clocks::sys_divider() == 1u &&
                      Clocks::peri_enabled() && Clocks::peri_source() == PeriAux::clk_sys);
    bench.verdict("the ring oscillator is still running (the resus circuit's and the "
                  "bootrom's clock)",
                  Rosc::running());

    count("the crystal", CountSource::xosc, Boot::xtal_hz);
    count("clk_ref", CountSource::clk_ref, Boot::xtal_hz);
    count("the PLL", CountSource::pll_sys, Boot::hz);
    count("clk_sys", CountSource::clk_sys, Boot::hz);
    count("clk_peri", CountSource::clk_peri, Boot::pclk_hz);
    count("the ring oscillator", CountSource::rosc, 0);
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

    if (!switch_to(Boot{}, "the PLL at 125 MHz", &up)) {
        bench.verdict("back to the PLL", false);
        return;
    }
    print(serial, "  back at 125 MHz: the switch took ", up, " us", crlf);
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
          " (VCO ", static_cast<uint32_t>(C::xtal_hz / 1'000'000u) * C::pll.fbdiv, " MHz), the switch took ",
          took, " us", crlf);
    bench.verdict(name, ": locked at pll_config_for's ratio", PllSys::locked() && cfg == C::pll);
    count(name, CountSource::pll_sys, C::hz);
    count(name, CountSource::clk_sys, C::hz);
}

void tc_rates() {
    pll_rate(R48{}, "48 MHz");
    pll_rate(R100{}, "100 MHz");
    pll_rate(R133{}, "133 MHz (the ceiling)");
    if (!switch_to(Boot{}, "125 MHz")) {
        bench.verdict("back to 125 MHz", false);
        return;
    }
    count("back at 125 MHz", CountSource::clk_sys, Boot::hz);
}

// -----------------------------------------------------------------------------
void td_peri() {
    if (!switch_to(X125{}, "125 MHz with clk_peri on the crystal")) {
        bench.verdict("PeriSource::crystal", false);
        return;
    }
    print(serial, "  the console now divides 12 MHz (", X125::pclk_hz, " Hz claimed for clk_peri)",
          crlf);
    bench.verdict("clk_peri is on the crystal", Clocks::peri_source() == PeriAux::xosc);
    count("clk_peri on the crystal", CountSource::clk_peri, X125::pclk_hz);
    count("clk_sys meanwhile", CountSource::clk_sys, X125::hz);

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
    const bool back = X125::init();
    Ticker::rebase(X125::hz);
    print(serial, "  and back to 125 MHz, the UART still untouched", crlf);
    bench.verdict("back at 125 MHz on the crystal-fed clk_peri", back);
    count("clk_sys at 125", CountSource::clk_sys, X125::hz);

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
    // The bracket's overhead: two timer reads and the calls, about 500
    // CPU cycles, at this rate's price plus the timer's 1 us grain.
    const uint32_t overhead_us = 500'000'000u / C::hz + 1u;
    print(serial, "  at ", name, ": delay_us(900) took ", took, " us, delay_us(100) took ",
          short_took, " us (", overhead_us, " us of bracket allowed)", crlf);
    bench.verdict(name, ": 900 us at least, and the bracket's overhead at most over",
                  long_ok && took >= 900u && took <= 900u + overhead_us);
    bench.verdict(name, ": 100 us at least, the same bracket",
                  short_ok && short_took >= 100u && short_took <= 100u + overhead_us);
    bench.verdict(name, ": a whole tick (1000 us) is refused", !delay_us(c, 1000));
}

void te_delay() {
    delay_at(R12{}, "12 MHz");
    delay_at(R48{}, "48 MHz");
    delay_at(R100{}, "100 MHz");
    delay_at(Boot{}, "125 MHz");
    delay_at(R133{}, "133 MHz");
    if (!switch_to(Boot{}, "125 MHz")) {
        bench.verdict("back to 125 MHz", false);
    }
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
    bench.verdict(name, ": the rebased ticker keeps 1 ms per tick within 0.1 %",
                  near(span, 100'000u, 1));
}

void tf_ticker() {
    ticker_at(R12{}, "12 MHz");
    ticker_at(R48{}, "48 MHz");
    ticker_at(R133{}, "133 MHz");
    if (!switch_to(Boot{}, "125 MHz")) {
        bench.verdict("back to 125 MHz", false);
        return;
    }
    ticker_at(Boot{}, "125 MHz");
}

// -----------------------------------------------------------------------------
void tg_rosc() {
    const std::optional<uint32_t> before = Boot::count_hz(CountSource::rosc);
    print(serial, "  the ring oscillator: ", before ? *before : 0u, " Hz", crlf);
    bench.verdict("the ring oscillator counts somewhere in its range (1..12 MHz at the "
                  "reset settings)",
                  before && *before >= 1'000'000u && *before <= 12'000'000u);
    Rosc::stop();
    print(serial, "  stopped: ", Rosc::running() ? "still ENABLED" : "ENABLED clear", crlf);
    bench.verdict("Rosc::stop() clears ENABLED", !Rosc::running());
    const std::optional<uint32_t> dead = Boot::count_hz(CountSource::rosc, 10);
    print(serial, "  the counter on the stopped oscillator: ",
          dead ? "a count of " : "DEAD (STATUS.DIED)", dead ? *dead : 0u, dead ? " Hz" : "", crlf);
    bench.verdict("the counter reports a stopped source dead or under 1 kHz",
                  !dead || *dead < 1000u);
    const bool restarted = Rosc::start();
    const std::optional<uint32_t> after = Boot::count_hz(CountSource::rosc);
    print(serial, "  restarted: ", restarted ? "STABLE" : "not stable", ", ", after ? *after : 0u,
          " Hz", crlf);
    bench.verdict("Rosc::start() brings it back stable", restarted && Rosc::running());
    bench.verdict("and it counts within 20 % of before", before && after &&
                                                          near(*after, *before, 200));
}

// -----------------------------------------------------------------------------
void th_divider_and_refusal() {
    if (!switch_to(X125{}, "125 MHz with clk_peri on the crystal")) {
        bench.verdict("PeriSource::crystal", false);
        return;
    }
    console_drain();
    Clocks::sys_divider(2);
    Ticker::rebase(X125::hz / 2u);
    print(serial, "  clk_sys divided by 2 under a crystal-fed console: this line is the proof",
          crlf);
    count("clk_sys / 2", CountSource::clk_sys, X125::hz / 2u);
    count("clk_peri meanwhile", CountSource::clk_peri, X125::pclk_hz);
    Clocks::sys_divider(1);
    Ticker::rebase(X125::hz);
    count("clk_sys undivided again", CountSource::clk_sys, X125::hz);

    // The PLL asked for the impossible, with clk_sys parked on clk_ref
    // (the crystal, 12 MHz) so nothing runs on the PLL meanwhile.
    console_drain();
    const bool parked = Clocks::sys_from_ref();
    Ticker::rebase(X125::xtal_hz);
    bench.verdict("clk_sys parks on clk_ref", parked);
    const PllConfig low_vco{.refdiv = 1, .fbdiv = 16, .postdiv1 = 7, .postdiv2 = 7};   // 192 MHz VCO
    const uint32_t t0 = us_now();
    const bool locked = PllSys::init(low_vco);
    const uint32_t took = us_now() - t0;
    print(serial, "  the PLL at a 192 MHz VCO (below the 750 MHz floor): LOCK ",
          locked ? "ROSE" : "did not rise", " in ", took, " us (printed, not judged: the "
          "datasheet promises nothing outside the range)", crlf);

    const bool restored = switch_to(Boot{}, "the boot clock");
    bench.verdict("the boot clock restored after the parking and the refusal", restored);
    count("clk_sys restored", CountSource::clk_sys, Boot::hz);
    bench.verdict("clk_peri back on clk_sys", Clocks::peri_source() == PeriAux::clk_sys);
}

// -----------------------------------------------------------------------------
void to_output() {
    const bool ok = ClockOut<0>::init(GpoutSource::xosc, 12);
    print(serial, "  GPOUT0 on GP", ClockOut<0>::pin, ": the crystal / 12 = 1 MHz, ",
          ClockOut<0>::enabled() ? "ENABLED" : "off", " - left running for the other board's p",
          crlf);
    bench.verdict("ClockOut<0>::init(xosc, 12) enabled the generator", ok && ClockOut<0>::enabled());
    bench.verdict("GP21 is on the clock function",
                  Pin<ClockOut<0>::pin>::function() == PinFunction::clock);
    bench.verdict("a zero divider is refused", !ClockOut<1>::init(GpoutSource::xosc, 0));
}

void tp_input() {
    ClockIn<0>::init();
    const std::optional<uint32_t> hz = Boot::count_hz(ClockIn<0>::count_source);
    if (!hz) {
        print(serial, "  GPIN0 on GP", ClockIn<0>::pin, ": DEAD - nothing on the wire (the other "
              "board's o not run, or the link not wired)", crlf);
        bench.verdict("the other board's crystal arrives on GP20", false);
        ClockIn<0>::release();
        return;
    }
    // The count is other_crystal / 12 in this crystal's units: the ratio.
    print(serial, "  GPIN0 on GP", ClockIn<0>::pin, " = ", *hz, " Hz = the other board's crystal "
          "/ 12, counted against this board's crystal", crlf);
    bench.verdict("the other board's crystal arrives on GP20 within 0.1 % of 1 MHz (the "
                  "ratio of the two crystals)",
                  near(*hz, 1'000'000u, 1));
    ClockIn<0>::release();
}

void banner() {
    print(serial, crlf, "test_rp2040_clock - the RP2040 clock tree (datasheet 2.15-2.18), boot clk=",
          Boot::hz, " Hz on the PLL, crystal ", Boot::xtal_hz, " Hz", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_uart0() { (void)Serial::isr(); }
extern "C" void isr_systick() { brio::Ticker::tick(); }

int main() {
    const bool clock_ok = Boot::init();
    const bool timer_ok = brio::Timer::init(clock);
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the boot rate, the tree as init() left it, every clock counted", ta_boot);
    bench.letter('b', "the crystal alone and back", tb_crystal);
    bench.letter('c', "the other PLL rates: 48, 100, 133 MHz", tc_rates);
    bench.letter('d', "clk_peri on the crystal while clk_sys moves", td_peri);
    bench.letter('e', "delay_us at every rate", te_delay);
    bench.letter('f', "the ticker rebased across rates", tf_ticker);
    bench.letter('g', "the ring oscillator stopped and restarted", tg_rosc);
    bench.letter('h', "the divider under a crystal-fed console; the PLL refused", th_divider_and_refusal);
    bench.letter('o', "GPOUT0 on GP21 = the crystal / 12, left running", to_output, false);
    bench.letter('p', "GPIN0 on GP20 counted: the other board's crystal", tp_input, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL125" : "FAILED",
                    " timer=", timer_ok ? "1us" : "FAILED", " tick=",
                    tick_ok ? "SysTick" : "FAILED", brio::crlf);
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
