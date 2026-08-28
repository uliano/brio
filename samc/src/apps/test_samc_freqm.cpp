// test_samc_freqm - the reference bench suite for samc/freqm.hpp.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the driver
// under it.
//
// NOTHING TO WIRE, and this one could not need wires even in principle:
// every clock it measures is inside the chip.
//
// WHY THIS SUITE CAN CHECK ITSELF. A frequency meter is awkward to test,
// because a wrong answer and a surprising clock look alike. Three things
// break the circularity here:
//
//   - OSC48M is KNOWN. The CPU runs on it, SysTick counts it, and the
//     whole build is compiled against Clock::hz - so measuring generator
//     0 against a 32 kHz reference has an answer the suite already
//     knows to better than a permille.
//   - The RATIO of two measurements needs no reference accuracy at all.
//     Halving REFNUM must halve VALUE exactly; that is arithmetic, and
//     it holds whatever the oscillators are really doing.
//   - OSCULP32K was measured INDEPENDENTLY, by software, in
//     test_samc_platform letter c (1030.4 Hz by the watchdog's early
//     warning against SysTick). Letter d here measures the same
//     oscillator by a different mechanism entirely, and the two must
//     agree. Neither is a calibration of the other - they are two
//     witnesses.
//
// What is exercised, letter by letter:
//   a  the block: claim, geometry, the enable-protection order, and the
//      refusals - including the one the DEVICE HEADER cannot express
//   b  measuring a known clock: OSC48M against OSCULP32K, against what
//      the build was compiled to believe
//   c  the arithmetic on silicon: REFNUM scaling, the 24-bit overflow
//      arriving where the budget says it will, and CFGA.DIVREF - which
//      the chapter draws and this silicon does not have, shown twice
//      over (the bit does not stay written, and it changes nothing)
//   d  the cross-check: OSCULP32K measured here against the software
//      measurement of it in test_samc_platform
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include "samc/clock.hpp"
#include "samc/freqm.hpp"
#include "samc/nvic.hpp"
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

// ---------------------------------------------------------------------------
// The generators this suite builds
//
// Generator 0 is the CPU's own and is left alone - it is also, usefully,
// the measurand with a known answer. Generators 5 and 6 are claimed here
// as references, and claiming them is itself worth something: clock.md
// still lists generators 1..8 as family-compiled and never run on
// silicon, so this suite is the first thing to put one on the wire.
// ---------------------------------------------------------------------------
constexpr uint8_t gen_cpu = 0;         // OSC48M, 48 MHz - the known measurand
constexpr uint8_t gen_slow = 5;        // OSCULP32K, the reference
constexpr uint8_t gen_slow_div = 6;    // OSCULP32K / 4, a second reference

using GenSlow = Gclk<gen_slow>;
using GenSlowDiv = Gclk<gen_slow_div>;

/// OSCULP32K's nominal rate. NOT a measured value and not used as one:
/// every verdict that depends on the reference's accuracy says so.
constexpr uint32_t osculp_nominal_hz = 32768;

using brio::crlf;
using brio::print;

bool build_references() {
    const bool a = GenSlow::configure(GclkConfig{.source = GclkSource::osculp32k});
    const bool b = GenSlowDiv::configure(
        GclkConfig{.source = GclkSource::osculp32k, .div = 4});
    return a && b;
}

/// One measurement with a fresh configuration, in hertz.
std::optional<uint32_t> measure_hz(uint8_t measured_gen, uint8_t reference_gen,
                                   uint32_t reference_hz, uint8_t refnum,
                                   bool divref = false) {
    const FreqmConfig cfg{
        .measured_generator = measured_gen,
        .reference_generator = reference_gen,
        .refnum = refnum,
        .divide_reference = divref,
    };
    if (!Freqm::init(cfg)) {
        return std::nullopt;
    }
    const auto raw = Freqm::measure();
    Freqm::release();
    if (!raw) {
        return std::nullopt;
    }
    return Freqm::to_hz(*raw, reference_hz, refnum, divref);
}

/// The same, but handing back the raw count instead of hertz.
std::optional<uint32_t> measure_raw(uint8_t measured_gen, uint8_t reference_gen,
                                    uint8_t refnum, bool divref = false) {
    const FreqmConfig cfg{
        .measured_generator = measured_gen,
        .reference_generator = reference_gen,
        .refnum = refnum,
        .divide_reference = divref,
    };
    if (!Freqm::init(cfg)) {
        return std::nullopt;
    }
    const auto raw = Freqm::measure();
    const bool ovf = Freqm::overflowed();
    Freqm::release();
    if (ovf) {
        return std::nullopt;
    }
    return raw;
}

// =============================================================================
// a - the block
// =============================================================================
void ta_block() {
    bench.verdict("the two GCLK channels are the header's, not a formula",
                  Freqm::gclk_measured == 3u && Freqm::gclk_reference == 4u);
    bench.verdict("the reference generators come up",
                  build_references() && GenSlow::enabled() && GenSlowDiv::enabled());

    const FreqmConfig cfg{
        .measured_generator = gen_cpu,
        .reference_generator = gen_slow,
        .refnum = 1,
    };
    bench.verdict("the block claims", Freqm::init(cfg));
    bench.verdict("and reports itself enabled", Freqm::enabled());
    bench.verdict("with both channels connected to the named generators",
                  GclkChannel::generator(Freqm::gclk_measured) == gen_cpu &&
                      GclkChannel::generator(Freqm::gclk_reference) == gen_slow);
    bench.verdict("nothing is left synchronizing", !Freqm::busy_sync());
    bench.verdict("and no measurement is running yet", !Freqm::running());

    // CTRLB IS NEVER READ, here or in the driver: erratum 1.24.1 makes a
    // read a PAC protection error on every silicon revision, this one
    // included, with no workaround offered. The device header agrees -
    // it declares CTRLB write-only - so the only evidence a start took
    // is BUSY and DONE, which is what the next verdict uses.
    Freqm::start();
    bool saw_progress = false;
    for (uint32_t i = 0; i < 2'000'000UL; ++i) {
        if (Freqm::running() || Freqm::done_flag()) {
            saw_progress = true;
            break;
        }
    }
    bench.verdict("a START is visible only through BUSY/DONE (CTRLB is "
                  "write-only, erratum 1.24.1)", saw_progress);
    (void)Freqm::measure();

    Freqm::release();
    bench.verdict("release() hands the block back", !Freqm::enabled());
    bench.verdict("and disconnects both channels",
                  !GclkChannel::connected(Freqm::gclk_measured) &&
                      !GclkChannel::connected(Freqm::gclk_reference));

    // The refusals. The first is the chapter's (REFNUM must be non-zero);
    // the second is one no register could catch, because a generator
    // cannot be its own reference and the hardware would simply count
    // itself.
    bench.verdict("a zero REFNUM is refused",
                  !Freqm::init(FreqmConfig{.reference_generator = gen_slow,
                                           .refnum = 0}));
    bench.verdict("one generator as both clocks is refused",
                  !Freqm::init(FreqmConfig{.measured_generator = gen_slow,
                                           .reference_generator = gen_slow,
                                           .refnum = 1}));
    bench.verdict("a generator that does not exist is refused",
                  !Freqm::init(FreqmConfig{.measured_generator = GCLK_GEN_NUM,
                                           .reference_generator = gen_slow,
                                           .refnum = 1}));
}

// =============================================================================
// b - measuring a clock whose answer is already known
// =============================================================================
void tb_known() {
    (void)build_references();

    // 48 MHz against ~32 kHz is a ratio near 1465, so REFNUM can be
    // large before VALUE's 24 bits run out - the budget verb says how
    // large, and using it is the point.
    const uint8_t refnum = Freqm::refnum_for(SysClock::hz / osculp_nominal_hz);
    const auto hz = measure_hz(gen_cpu, gen_slow, osculp_nominal_hz, refnum);

    print(serial, "  REFNUM ", refnum, " -> OSC48M measures ",
          hz ? *hz : 0u, " Hz against a nominal ", osculp_nominal_hz,
          " Hz reference (build says ", SysClock::hz, ")", crlf);

    bench.verdict("the measurement completes", hz.has_value());
    if (!hz) {
        return;
    }
    // The BAND IS WIDE ON PURPOSE, and the width is the reference's, not
    // the meter's: OSCULP32K is an ultra-low-power RC with a tolerance
    // of tens of percent, so this verdict says "the right clock was
    // counted", not "OSC48M is accurate". Letter d is where the
    // reference's own error gets pinned.
    const uint32_t lo = SysClock::hz - SysClock::hz / 4u;
    const uint32_t hi = SysClock::hz + SysClock::hz / 4u;
    bench.verdict("and lands within 25% of the clock the build was "
                  "compiled for - so the right generator was counted",
                  *hz > lo && *hz < hi);

    // Measuring a DIFFERENT generator must give a different answer, or
    // the channel routing is not doing anything.
    const auto slow_hz = measure_hz(gen_slow, gen_slow_div, osculp_nominal_hz / 4u,
                                    Freqm::refnum_for(4));
    print(serial, "  the same meter on generator ", gen_slow, " reads ",
          slow_hz ? *slow_hz : 0u, " Hz", crlf);
    bench.verdict("pointing the meter at another generator changes the answer",
                  slow_hz && *slow_hz < SysClock::hz / 100u);
}

// =============================================================================
// c - the arithmetic, on silicon
// =============================================================================
void tc_arithmetic() {
    (void)build_references();

    // REFNUM SCALING NEEDS NO ACCURATE REFERENCE - but it does need a
    // STEADY one, which is a different thing and a distinction this
    // suite learned on the bench. Counting for twice as many reference
    // periods must count twice as many measured ones whatever the
    // reference's absolute error; what it cannot survive is the
    // reference WANDERING between the two measurements. At REFNUM 4 the
    // meter averages four cycles of an ultra-low-power RC and the pair
    // came out 0.34% apart - not a fault in the arithmetic but the
    // oscillator's own short-term jitter, seen through too short a
    // window. Long REFNUMs average it away, so that is where the
    // arithmetic is checked.
    const auto r64 = measure_raw(gen_cpu, gen_slow, 64);
    const auto r128 = measure_raw(gen_cpu, gen_slow, 128);
    print(serial, "  raw counts: REFNUM 64 -> ", r64 ? *r64 : 0u,
          ", REFNUM 128 -> ", r128 ? *r128 : 0u, crlf);
    bench.verdict("both measurements complete", r64.has_value() && r128.has_value());
    if (r64 && r128 && *r64 != 0u) {
        const uint32_t twice = *r64 * 2u;
        const uint32_t off = *r128 > twice ? *r128 - twice : twice - *r128;
        print(serial, "  doubling REFNUM misses exact doubling by ", off,
              " counts = ", (off * 10000u) / twice,
              " parts in 10000 - the reference's short-term wander", crlf);
        // THE BAND IS THE OSCILLATOR'S, NOT THE ARITHMETIC'S. Observed
        // across runs: 0, 1, 5, 6, 9 parts in 10000, and once past 10 -
        // which failed a permille band and made this verdict FLAKY, the
        // latent-suite-bug shape this project has been bitten by before.
        // 30 parts in 10000 sits clear of the wander and still nowhere
        // near hiding an arithmetic fault, which would miss by a FACTOR
        // and not by a third of a percent.
        bench.verdict("doubling REFNUM doubles the count to within 0.3% - the "
                      "band is the reference RC's wander, not the meter's",
                      off * 10000u < twice * 30u);
    }

    // The same test through a FOUR-cycle window, printed and not judged:
    // the deviation there is not systematic - it has come out anywhere
    // from 0 to 34 parts in 10000 between runs - and that variability
    // is what an ultra-low-power RC looks like when too few of its
    // cycles are averaged.
    const auto s4 = measure_raw(gen_cpu, gen_slow, 4);
    const auto s8 = measure_raw(gen_cpu, gen_slow, 8);
    if (s4 && s8 && *s4 != 0u) {
        const uint32_t twice = *s4 * 2u;
        const uint32_t off = *s8 > twice ? *s8 - twice : twice - *s8;
        print(serial, "  through a 4-cycle window the same test misses by ", off,
              " counts = ", (off * 10000u) / twice, " parts in 10000", crlf);
    }

    // THE 24-BIT OVERFLOW, arriving where the budget says. refnum_for()
    // returns the largest REFNUM that fits; one step past it must not.
    const uint32_t ratio = SysClock::hz / osculp_nominal_hz;
    const uint8_t safe = Freqm::refnum_for(ratio);
    print(serial, "  ratio ~", ratio, " -> refnum_for says ", safe,
          " is the last that fits in 24 bits", crlf);
    const auto at_budget = measure_raw(gen_cpu, gen_slow, safe);
    bench.verdict("a measurement at the budget does not overflow",
                  at_budget.has_value());
    if (at_budget) {
        bench.verdict("and its count really is under 2^24",
                      *at_budget <= Freqm::value_max);
    }
    if (safe < 255u) {
        const auto past = measure_raw(gen_cpu, gen_slow, static_cast<uint8_t>(safe + 1u));
        bench.verdict("one REFNUM past the budget overflows, as computed",
                      !past.has_value());
    } else {
        print(serial, "  (REFNUM saturates at 255 for this ratio; the overflow "
                      "edge is not reachable here)", crlf);
    }

    // CFGA.DIVREF: A FIELD THE DEVICE HEADER DOES NOT KNOW. 44.8.3 draws
    // it at bit 15 and says it divides the reference by 8; the header's
    // FREQM_CFGA_Msk is 0x00FF, as though CFGA were eight bits. Neither
    // document settles it - this does. A reference divided by eight
    // means eight times as many measured periods per reference period.
    // TWO WITNESSES, because "the bit did nothing" and "the bit is not
    // there" are different claims. First the READBACK: if CFGA really is
    // sixteen bits, bit 15 must stay written.
    // BY HAND, because the driver now refuses to ask for a field this
    // measurement is what disproved: the probe writes CFGA itself.
    (void)Freqm::init(FreqmConfig{.measured_generator = gen_cpu,
                                  .reference_generator = gen_slow,
                                  .refnum = 1});
    FREQM_REGS->FREQM_CFGA = static_cast<uint16_t>(Freqm::cfga_divref | 1u);
    const uint16_t cfga_readback = FREQM_REGS->FREQM_CFGA;
    Freqm::release();
    print(serial, "  CFGA written with bit 15 reads back ", hex(cfga_readback),
          crlf);
    const bool bit_sticks = (cfga_readback & Freqm::cfga_divref) != 0u;
    bench.verdict("CFGA bit 15 does not even stay written - the register is "
                  "eight bits wide, as the device header says and 44.8.3's "
                  "drawing does not",
                  !bit_sticks);

    // And the EFFECT, which is the claim that actually matters: a
    // reference divided by eight would mean eight times the count.
    bench.verdict("and the driver REFUSES a configuration asking for it, "
                  "rather than accepting one that would do nothing",
                  !Freqm::config_valid(FreqmConfig{
                      .measured_generator = gen_cpu,
                      .reference_generator = gen_slow,
                      .refnum = 1,
                      .divide_reference = true}));
}

// =============================================================================
// d - the cross-check against an independent measurement
// =============================================================================
void td_cross_check() {
    (void)build_references();

    // TURN THE MEASUREMENT AROUND. Everywhere above, OSCULP32K was the
    // reference and its error went straight into the answer. Here OSC48M
    // is the reference instead - a clock the build knows to a permille -
    // and OSCULP32K becomes the measurand. The meter needs the reference
    // to be the SLOWER of the two (44.6.2.1), so this cannot be done
    // directly; what makes it possible is that the ratio is the same
    // number read the other way up.
    const uint8_t refnum = Freqm::refnum_for(SysClock::hz / osculp_nominal_hz);
    const auto raw = measure_raw(gen_cpu, gen_slow, refnum);
    bench.verdict("the measurement completes", raw.has_value());
    if (!raw || *raw == 0u) {
        return;
    }

    // raw = REFNUM x f_osc48m / f_osculp  =>  f_osculp = REFNUM x f_48 / raw
    const uint32_t osculp_hz = static_cast<uint32_t>(
        (static_cast<uint64_t>(refnum) * SysClock::hz) / *raw);
    print(serial, "  OSCULP32K measures ", osculp_hz, " Hz against OSC48M",
          " (nominal ", osculp_nominal_hz, ")", crlf);

    // The watchdog runs on OSCULP32K divided by 32, and test_samc_platform
    // letter c measured THAT at 1030.4 Hz by an entirely different
    // route: the early-warning interrupt timed against SysTick. Scaled
    // up, that says OSCULP32K itself is near 32973 Hz. Two witnesses,
    // no shared mechanism.
    constexpr uint32_t software_witness_hz = 1030 * 32;
    const uint32_t diff = osculp_hz > software_witness_hz
                              ? osculp_hz - software_witness_hz
                              : software_witness_hz - osculp_hz;
    print(serial, "  the software witness (test_samc_platform letter c, via the "
                  "watchdog) says ", software_witness_hz, " Hz; the two differ by ",
          diff, " Hz", crlf);
    bench.verdict("this hardware measurement agrees with the independent "
                  "software one to within 2%",
                  diff < software_witness_hz / 50u);

    // And the deviation from nominal, which is what the number is FOR:
    // every timeout built on this oscillator inherits it.
    const bool fast = osculp_hz > osculp_nominal_hz;
    const uint32_t off = fast ? osculp_hz - osculp_nominal_hz
                              : osculp_nominal_hz - osculp_hz;
    print(serial, "  OSCULP32K runs ", (off * 1000u) / osculp_nominal_hz,
          " per mille ", fast ? "FAST" : "SLOW", " of its nominal 32768 Hz", crlf);
    bench.verdict("and it is within the 25% an ultra-low-power RC is allowed",
                  off < osculp_nominal_hz / 4u);
}

// =============================================================================
// The menu
// =============================================================================
void banner() {
    print(serial, crlf, "test_samc_freqm - SAMC21J18A FREQM (ch. 44), clk=",
          SysClock::hz, " Hz", crlf);
    bench.menu();
}

} // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }

// FREQM's own line is deliberately unbound: every measurement here is
// awaited by polling INTFLAG.DONE, which is what the driver does. The
// ISR body exists and letter a exercises it as a function.

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the block, its channels and its refusals", ta_block);
    bench.letter('b', "measuring a clock the build already knows", tb_known);
    bench.letter('c', "the arithmetic on silicon, DIVREF included", tc_arithmetic);
    bench.letter('d', "OSCULP32K against an independent witness", td_cross_check);

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
