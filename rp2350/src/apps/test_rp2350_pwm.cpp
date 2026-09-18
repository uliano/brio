// test_rp2350_pwm - the reference bench suite for this chip's PWM block
// (rp2350/pwm.hpp over datasheet 12.5): the twelve slices wireless (the
// arithmetic, the lockstep start, the phase nudges, THE TWO INTERRUPT
// LINES this chip added), on two wires between the chip's own slices (an
// output of one slice into the B input of another: the frequency and the
// duty measured by the block itself), and - for the FOUR SLICES THE
// RP2040 HAD NOT, whose pads no wire of this desk reaches - the wrap as a
// repeating timer and the pad read back through the single-cycle IO.
//
// ONE SOURCE, BOTH ARCHITECTURES. Every letter runs on the Cortex-M33
// pair and on the Hazard3 pair, built from the rp2350-arm-* and
// rp2350-riscv-* presets. Nothing in this chapter differs between them:
// the block is one, the two interrupt lines are two system lines under
// one numbering (3.8.4.2), and the handler names are the same on both.
//
// THREE WIRES, all part of the standing self-links of this board:
//
//   GP13 (slice 6 B, an output)  ->  GP15 (slice 7 B, an INPUT)
//   GP17 (slice 0 B, an output)  ->  GP9  (slice 4 B, an INPUT)
//   GP12 (slice 6 A, an output) <->  GP14 (slice 7 A, no input at all)
//
// Only a B pin is a PWM input (table 1130), which is why the two
// measuring wires end on an odd GPIO. The measuring slice counts the
// edges of the wire (the frequency) or the divided clock while the wire
// is high (the duty) over a window the system timer times. The third wire
// ends on an A pin, which no slice can read - but SIO reads every pad
// whoever owns it, so GP14 is where this suite watches the A output of a
// complementary pair while its B is measured on GP15, and the DEAD TIME
// between them is one word read per sample.
//
// THE RULER IS TIMER0 (rp2350/timer.hpp): 64 bits of microseconds on a
// tick generator dividing clk_ref, independent of clk_sys, which is what
// every frequency below is judged against.
//
// What is exercised, letter by letter:
//   a  the block, WIRELESS: table 1130 as the driver states it in both
//      halves, the period arithmetic and the chooser at this clk_sys, the
//      divider's own refusals, the reset state, a configuration read
//      back, the counter running and stopped, twelve slices started in
//      lockstep, the phase nudged one count forward and back
//   b  THE TWO INTERRUPT LINES: a slice's wrap counted on IRQ0 and
//      another's on IRQ1 at 1 kHz and 100 kHz, the two masked statuses
//      apart, the force on each line, a slice moved from one to the other
//   c  THE WIRE 6B -> 7B: the frequency at 1 kHz, 100 kHz and 1 MHz
//      counted by edges; the duty at 0, 25, 50, 75 and 100 % counted by
//      level - the two ends glitch-free
//   d  phase-correct mode halving the frequency, the inverted output
//      complementing the duty, a level change taken at the wrap
//   e  THE PAIR: 6A and 6B with a dead time of 100 counts, B measured on
//      the wire, and the dead time itself measured on both pads at once
//   f  THE SECOND WIRE 0B -> 4B: another slice measured, and two outputs
//      started in lockstep
//   g  THE DIVIDER LADDER on the wire: one, two and a half, ten, the top
//      of the range and 256 - and the fraction on top of 256 that this
//      chapter forbids
//   h  THE LAMP: util/rgb_lamp.hpp over three outputs, all three measured
//   j  THE FOUR SLICES THIS CHIP ADDED: 8..11 as repeating timers on both
//      lines, and slice 8's and slice 11's outputs on GPIO 32..47 sampled
//      through SIO - including the two pads that carry ONE output
//   y  (outside z) THE INSTRUMENTS: a pad sampled through a period, the
//      counter read at a growing delay after a nudge, the level written
//      mid-period followed on the pad
//
// build: boards = weact2350b,weact2350b-rv
// build: monitor_speed = 115200

#include <stdint.h>

#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/pwm.hpp"
#include "rp2350/ticker.hpp"
#include "rp2350/timer.hpp"
#include "rp2350/uart.hpp"
#include "util/print.hpp"
#include "util/rgb_lamp.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using SysClock = Clock<ClockSource::pll, 150'000'000>;
constexpr SysClock clock;
using P = Rp2350Platform<>;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;

using Led = Pin<25>;
using Ruler = Timer<0>;

TestBench<Serial> bench;

// The outputs under test and the inputs that measure them.
using Out6B = PwmOutput<13, 999>;      // slice 6 B on GP13 -> GP15
using Out0B = PwmOutput<17, 1499>;     // slice 0 B on GP17 -> GP9
using Edges7 = PwmEdgeCounter<15>;     // slice 7 counting GP15's edges
using Level7 = PwmLevelCounter<15>;    // slice 7 counting while GP15 is high
using Edges4 = PwmEdgeCounter<9>;      // slice 4 on GP9
using Level4 = PwmLevelCounter<9>;
using Pair6 = PwmPair<12, 13, 999>;    // slice 6 A on GP12, B on GP13

// The two interrupt lines, one task each.
using Tick3 = PwmPeriodicTick<3, 0>;
using Tick4 = PwmPeriodicTick<4, 1>;

// The four slices this chip added, as repeating timers - two on each
// line. On the QFN-60 this is all they can be; here it is what they are
// when their pads are wanted elsewhere.
using Tick8 = PwmPeriodicTick<8, 0>;
using Tick9 = PwmPeriodicTick<9, 1>;
using Tick10 = PwmPeriodicTick<10, 0>;
using Tick11 = PwmPeriodicTick<11, 1>;

volatile uint32_t wraps0[pwm_slice_count];
volatile uint32_t wraps1[pwm_slice_count];
volatile uint32_t entries0 = 0;
volatile uint32_t entries1 = 0;

uint32_t us_now() { return Ruler::now_low(); }
void spin_us(uint32_t us) {
    const uint32_t t0 = us_now();
    while (us_now() - t0 < us) {
    }
}
void clear_wraps() {
    for (uint8_t i = 0; i < pwm_slice_count; ++i) {
        wraps0[i] = 0;
        wraps1[i] = 0;
    }
    entries0 = 0;
    entries1 = 0;
}

/// An exact 1 kHz and an exact 100 kHz at this clk_sys: 150 MHz over 24
/// is 6.25 MHz, and 6250 counts of it are a millisecond; 150 MHz over one
/// gives 1500 counts in ten microseconds.
constexpr PwmDivider khz1_div{24, 0};
constexpr uint16_t khz1_top = 6249;
constexpr PwmDivider khz100_div{1, 0};
constexpr uint16_t khz100_top = 1499;

/// The level counter's full scale over a window: clk_sys / 256 cycles per
/// second, times the window.
constexpr uint32_t level_window_us = 100'000;
constexpr PwmDivider level_div{0, 0};   // 256
constexpr uint32_t level_full =
    static_cast<uint32_t>((static_cast<uint64_t>(SysClock::hz) / 256u) * level_window_us / 1'000'000u);

/// Edges on `Counter` over `us`: the frequency in Hz.
template <typename Counter>
uint32_t measure_hz(uint32_t us) {
    Counter::restart();
    Counter::run(true);
    spin_us(us);
    Counter::run(false);
    return static_cast<uint32_t>(static_cast<uint64_t>(Counter::count()) * 1'000'000u / us);
}
/// The high fraction on `Counter` over the level window, in per mille.
template <typename Counter>
uint32_t measure_duty_pm() {
    Counter::restart();
    Counter::run(true);
    spin_us(level_window_us);
    Counter::run(false);
    return static_cast<uint32_t>(static_cast<uint64_t>(Counter::count()) * 1000u / level_full);
}
bool within(uint32_t got, uint32_t want, uint32_t per_mille) {
    const uint32_t tol = static_cast<uint32_t>(static_cast<uint64_t>(want) * per_mille / 1000u);
    return got + tol >= want && got <= want + tol;
}
/// The tolerance a SAMPLED duty wants: an absolute band in per mille,
/// because the instrument's error is a share of ONE period out of the
/// window's - a fixed number of per mille, the same at every duty - and
/// not a share of the value read.
bool within_pm(uint32_t got, uint32_t want, uint32_t band) {
    return got + band >= want && got <= want + band;
}

/// Two pads of the HIGH half sampled in ONE word read over `us`: the high
/// fraction of each in per mille, and how many samples the two disagreed
/// on. A sampled duty and not a counted one - its error is the sampling
/// loop's own - but the disagreement count is exact, both bits coming
/// from the same read.
struct TwoPads {
    uint32_t a_pm;
    uint32_t b_pm;
    uint32_t differed;
    uint32_t samples;
};
TwoPads sample_pair_hi(uint8_t pin_a, uint8_t pin_b, uint32_t us) {
    const uint32_t mask_a = 1UL << (pin_a - 32u);
    const uint32_t mask_b = 1UL << (pin_b - 32u);
    uint32_t high_a = 0;
    uint32_t high_b = 0;
    uint32_t differed = 0;
    uint32_t samples = 0;
    const uint32_t t0 = us_now();
    while (us_now() - t0 < us) {
        const uint32_t w = Gpio::in_hi();
        const bool a = (w & mask_a) != 0u;
        const bool b = (w & mask_b) != 0u;
        if (a) { ++high_a; }
        if (b) { ++high_b; }
        if (a != b) { ++differed; }
        ++samples;
    }
    if (samples == 0u) {
        return TwoPads{0, 0, 0, 0};
    }
    return TwoPads{static_cast<uint32_t>(static_cast<uint64_t>(high_a) * 1000u / samples),
                   static_cast<uint32_t>(static_cast<uint64_t>(high_b) * 1000u / samples), differed, samples};
}

/// One pad of the LOW half sampled over `us`: its high fraction in per
/// mille. SIO reads a pad whoever owns it, so this measures an output
/// nothing on this desk can count.
uint32_t sample_pad_pm(uint8_t pin, uint32_t us) {
    const uint32_t mask = 1UL << pin;
    uint32_t high = 0;
    uint32_t samples = 0;
    const uint32_t t0 = us_now();
    while (us_now() - t0 < us) {
        if ((Gpio::in() & mask) != 0u) { ++high; }
        ++samples;
    }
    return samples == 0u ? 0u : static_cast<uint32_t>(static_cast<uint64_t>(high) * 1000u / samples);
}

/// The same for two pads of the LOW half, counting the samples where BOTH
/// are high and where BOTH are low - which is what a dead time is.
struct PairOverlap {
    uint32_t both_high_pm;
    uint32_t both_low_pm;
    uint32_t a_pm;
    uint32_t samples;
};
PairOverlap sample_overlap(uint8_t pin_a, uint8_t pin_b, uint32_t us) {
    const uint32_t mask_a = 1UL << pin_a;
    const uint32_t mask_b = 1UL << pin_b;
    uint32_t both_high = 0;
    uint32_t both_low = 0;
    uint32_t high_a = 0;
    uint32_t samples = 0;
    const uint32_t t0 = us_now();
    while (us_now() - t0 < us) {
        const uint32_t w = Gpio::in();
        const bool a = (w & mask_a) != 0u;
        const bool b = (w & mask_b) != 0u;
        if (a && b) { ++both_high; }
        if (!a && !b) { ++both_low; }
        if (a) { ++high_a; }
        ++samples;
    }
    if (samples == 0u) {
        return PairOverlap{0, 0, 0, 0};
    }
    return PairOverlap{static_cast<uint32_t>(static_cast<uint64_t>(both_high) * 1000u / samples),
                       static_cast<uint32_t>(static_cast<uint64_t>(both_low) * 1000u / samples),
                       static_cast<uint32_t>(static_cast<uint64_t>(high_a) * 1000u / samples), samples};
}

/// Whether a wire is fitted: the output pin driven both ways as a GPIO,
/// the input pin following. Driven and not floating, which is the only
/// honest way to read a pad on this stepping (erratum RP2350-E9).
template <uint8_t out_pin, uint8_t in_pin>
bool wire_present() {
    (void)Pin<in_pin>::input(PinPull::none);
    (void)Pin<out_pin>::output(false);
    spin_us(20);
    const bool low = !Pin<in_pin>::read();
    Pin<out_pin>::set();
    spin_us(20);
    const bool high = Pin<in_pin>::read();
    (void)Pin<out_pin>::release();
    (void)Pin<in_pin>::release();
    return low && high;
}
bool wire_6b_7b() {
    if (wire_present<13, 15>()) {
        return true;
    }
    bench.verdict("DECLINED: GP13 -> GP15 is not wired (the I2C self-link's SCL wire)", false);
    return false;
}
bool wire_0b_4b() {
    if (wire_present<17, 9>()) {
        return true;
    }
    bench.verdict("DECLINED: GP17 -> GP9 is not wired (the SPI self-link's select wire)", false);
    return false;
}
bool wire_6a_7a() {
    if (wire_present<12, 14>()) {
        return true;
    }
    bench.verdict("DECLINED: GP12 -> GP14 is not wired (the I2C self-link's SDA wire)", false);
    return false;
}

void all_off() {
    Pwm::interrupts<0>(Pwm::all_slices, false);
    Pwm::interrupts<1>(Pwm::all_slices, false);
    Pwm::stop(Pwm::all_slices);
    Out6B::release();
    Out0B::release();
    (void)Pin<12>::release();
    (void)Pin<14>::release();
    (void)Pin<15>::release();
    (void)Pin<9>::release();
}

// =============================================================================
// a - the block, wireless
// =============================================================================
void ta_block() {
    bench.verdict("table 1130 as the driver states it: GPIO 0 is 0A, 1 is 0B, 13 is 6B, 15 is 7B, "
                  "16 is 0A again, 31 is 7B - and the half this chip added, 32 is 8A, 39 is 11B, "
                  "40 is 8A again, 47 is 11B; only an odd GPIO is an input",
                  pwm_slice_of(0) == 0u && !pwm_pin_is_b(0) && pwm_pin_is_b(1) &&
                      pwm_slice_of(13) == 6u && pwm_pin_is_b(13) && pwm_slice_of(15) == 7u &&
                      pwm_slice_of(16) == 0u && pwm_slice_of(31) == 7u && pwm_slice_of(32) == 8u &&
                      pwm_slice_of(39) == 11u && pwm_pin_is_b(39) && pwm_slice_of(40) == 8u &&
                      pwm_slice_of(47) == 11u);

    const auto khz1 = pwm_config_for(SysClock::hz, 1000);
    const auto hz1 = pwm_config_for(SysClock::hz, 1);
    print(serial, "  at ", SysClock::hz / 1'000'000u, " MHz: TOP 149 at one is ",
          pwm_output_hz(SysClock::hz, {.top = 149}), " Hz; 1 kHz at TOP 65534 wants ",
          khz1 ? khz1->divider.sixteenths() : 0u, " sixteenths -> ",
          khz1 ? pwm_output_hz(SysClock::hz, *khz1) : 0u, " Hz; the floor is ",
          pwm_output_hz(SysClock::hz, {.divider = {0, 0}}), " Hz; 1 Hz is ",
          hz1 ? "reachable" : "refused", crlf);
    bench.verdict("the period arithmetic: TOP 149 at one is 1 MHz, 500 kHz phase-correct; 1 kHz at "
                  "the finest TOP is 989 Hz at the nearest divider; the floor is 8 Hz; 1 Hz is "
                  "refused (a system timer's job)",
                  pwm_output_hz(SysClock::hz, {.top = 149}) == 1'000'000u &&
                      pwm_output_hz(SysClock::hz, {.top = 149, .phase_correct = true}) == 500'000u &&
                      khz1 && pwm_output_hz(SysClock::hz, *khz1) == 989u &&
                      pwm_output_hz(SysClock::hz, {.divider = {0, 0}}) == 8u && !hz1);

    using S = PwmSlice<3>;
    const bool reset_ok = Pwm::reset();
    print(serial, "  slice 3 after reset: CSR=", hex(S::csr()), " DIV=", hex(S::div()), " TOP=",
          hex(S::top()), " CC=", hex(S::cc()), " CTR=", S::counter(), " EN=", hex(Pwm::running()),
          crlf);
    bench.verdict("the block's reset state: every slice disabled, the divider at one, TOP 0xFFFF, "
                  "the levels and the counter at zero",
                  reset_ok && S::csr() == 0u && S::div() == 0x10u && S::top() == 0xFFFFu &&
                      S::cc() == 0u && S::counter() == 0u && Pwm::running() == 0u);

    const bool cfg = S::configure({.mode = PwmDivMode::level_high, .divider = {3, 4}, .top = 999,
                                   .phase_correct = true, .invert_b = true});
    const PwmSliceConfig back = S::config();
    S::levels(100, 200);
    bench.verdict("a configuration reads back field by field (level mode, 3 and 4/16, TOP 999, "
                  "phase-correct, B inverted, the levels)",
                  cfg && back.mode == PwmDivMode::level_high && back.divider == PwmDivider{3, 4} &&
                      back.top == 999u && back.phase_correct && !back.invert_a && back.invert_b &&
                      S::level(0) == 100u && S::level(1) == 200u);
    bench.verdict("and two dividers are refused before any register moves: a sixteenth past the "
                  "field, and ANY fraction on top of DIV_INT 0 - which 12.5.2.6 forbids in so many "
                  "words, so the top of this chip's range is 256 exactly",
                  !S::configure({.divider = {1, 16}}) && !S::configure({.divider = {0, 1}}) &&
                      !pwm_divider_of(4097).has_value() && pwm_divider_of(4096).has_value());

    (void)S::configure({.divider = {2, 0}, .top = 0xFFFF});
    S::counter(0);
    S::enable(true);
    spin_us(100);
    const uint16_t c1 = S::counter();
    S::enable(false);
    const uint16_t c2 = S::counter();
    spin_us(100);
    const uint16_t c3 = S::counter();
    print(serial, "  the counter at divider 2: ", c1, " after 100 us running, then ", c2, " and ",
          c3, " stopped", crlf);
    bench.verdict("the counter runs when enabled (about 7500 counts in 100 us at divider 2 off a "
                  "150 MHz clk_sys) and stands when disabled",
                  within(c1, 7500, 100) && c2 == c3);

    // TWELVE bits in the global enable, where the RP2040 had eight.
    Pwm::start(Pwm::all_slices);
    const uint16_t all_up = Pwm::running();
    Pwm::stop(Pwm::all_slices);
    const uint16_t all_down = Pwm::running();
    print(serial, "  EN with every slice started: ", hex(all_up), " (the block has ",
          pwm_slice_count, " slices)", crlf);
    bench.verdict("the global enable carries twelve bits and every one of them answers",
                  all_up == 0x0FFFu && all_down == 0u && Pwm::all_slices == 0x0FFFu);

    // Lockstep: two slices with equal configuration started by the global
    // enable read equal counters.
    using T = PwmSlice<5>;
    (void)S::configure({.divider = {2, 0}, .top = 0xFFFF});
    (void)T::configure({.divider = {2, 0}, .top = 0xFFFF});
    S::counter(0);
    T::counter(0);
    Pwm::start(S::bit | T::bit);
    spin_us(50);
    // The lag between the two counters, free of the read's own delay: two
    // pairs in either order, averaged.
    auto lag_now = [] {
        const uint16_t a1 = S::counter();
        const uint16_t b1 = T::counter();
        const uint16_t b2 = T::counter();
        const uint16_t a2 = S::counter();
        return (static_cast<int32_t>(b1) - static_cast<int32_t>(a1) + static_cast<int32_t>(b2) -
                static_cast<int32_t>(a2)) / 2;
    };
    const int32_t lag = lag_now();
    print(serial, "  slices 3 and 5 started together: lag ", lag, " counts after 50 us", crlf);
    bench.verdict("two slices started by the global enable run in lockstep: their counters agree "
                  "within a count",
                  Pwm::running() == (S::bit | T::bit) && lag >= -1 && lag <= 1);

    (void)S::advance_phase();
    (void)S::retard_phase();
    spin_us(100);
    const int32_t lag0 = lag_now();
    const bool adv = S::advance_phase();
    spin_us(100);
    const int32_t lag1 = lag_now();
    const bool ret = S::retard_phase();
    spin_us(100);
    const int32_t lag2 = lag_now();
    print(serial, "  lag ", lag0, "; after one advance ", lag1, "; after one retard ", lag2, crlf);
    bench.verdict("a phase advance puts slice 3 one count ahead of slice 5, a retard brings it "
                  "back (12.5.2.8, at a divider above one)",
                  adv && ret && lag1 == lag0 - 1 && lag2 == lag0);
    Pwm::stop(Pwm::all_slices);
}

// =============================================================================
// b - the two interrupt lines
// =============================================================================
void tb_lines() {
    Pwm::stop(Pwm::all_slices);
    Pwm::clear_pending(Pwm::all_slices);
    clear_wraps();

    // One slice on each line, at an exact kilohertz, counted together.
    (void)Tick3::setup(khz1_div, khz1_top);
    (void)Tick4::setup(khz1_div, khz1_top);
    const uint32_t t0 = us_now();
    spin_us(200'000);
    const uint32_t n3 = wraps0[3];
    const uint32_t n4 = wraps1[4];
    const uint32_t took = us_now() - t0;
    const uint32_t e0 = entries0;
    const uint32_t e1 = entries1;
    Tick3::stop();
    Tick4::stop();
    print(serial, "  1 kHz on both lines over ", took, " us: slice 3 on IRQ0 ", n3,
          " wraps in ", e0, " entries of isr_pwm_wrap_0; slice 4 on IRQ1 ", n4, " wraps in ", e1,
          " entries of isr_pwm_wrap_1", crlf);
    bench.verdict("TWO SHARED INTERRUPT LINES, which the RP2040 had not (12.5.1.1): one slice's "
                  "wrap counted on IRQ0 and another's on IRQ1, 200 of each in 200 ms, and each "
                  "handler entered only for its own line",
                  n3 >= 199u && n3 <= 201u && n4 >= 199u && n4 <= 201u && e0 >= 199u &&
                      e0 <= 201u && e1 >= 199u && e1 <= 201u);
    bench.verdict("and no slice crossed: line 0's handler never saw slice 4, line 1's never saw "
                  "slice 3",
                  wraps0[4] == 0u && wraps1[3] == 0u);

    // The faster rate, on line 1 alone.
    clear_wraps();
    (void)Tick4::setup(khz100_div, khz100_top);
    spin_us(10'000);
    const uint32_t n5 = wraps1[4];
    Tick4::stop();
    print(serial, "  100 kHz wrap on IRQ1: ", n5, " interrupts in 10 ms", crlf);
    bench.verdict("at 100 kHz, 1000 interrupts in 10 ms, two either way", n5 >= 998u && n5 <= 1002u);

    // The masked statuses apart, and the force on each line. INTF raises
    // the LINE and no raw flag, so the handler would re-enter without end:
    // both lines are closed at the controller for the check.
    Pwm::stop(Pwm::all_slices);
    Irq::disable(Pwm::irq<0>());
    Irq::disable(Pwm::irq<1>());
    Pwm::clear_pending(Pwm::all_slices);
    Pwm::interrupts<0>(PwmSlice<3>::bit, true);
    Pwm::interrupts<1>(PwmSlice<4>::bit, true);
    const uint16_t inte0 = Pwm::interrupts<0>();
    const uint16_t inte1 = Pwm::interrupts<1>();
    Pwm::force<0>(PwmSlice<3>::bit, true);
    const uint16_t ints0 = Pwm::pending<0>();
    const uint16_t ints1 = Pwm::pending<1>();
    const bool raw_clean = Pwm::raw_pending() == 0u;
    Pwm::force<0>(PwmSlice<3>::bit, false);
    const bool gone = Pwm::pending<0>() == 0u;
    print(serial, "  INTE0=", hex(inte0), " INTE1=", hex(inte1), "; a force on line 0: INTS0=",
          hex(ints0), " INTS1=", hex(ints1), " INTR=", hex(Pwm::raw_pending()), crlf);
    bench.verdict("the two lines have an enable register each and they keep their own slices",
                  inte0 == PwmSlice<3>::bit && inte1 == PwmSlice<4>::bit);
    bench.verdict("a force on ONE line appears in that line's status alone, with no raw flag, and "
                  "leaves with the force",
                  ints0 == PwmSlice<3>::bit && ints1 == 0u && raw_clean && gone);

    // The same slice moved from one line to the other: the line is a
    // property of the enable register and of nothing in the slice.
    Pwm::interrupts<0>(Pwm::all_slices, false);
    Pwm::interrupts<1>(Pwm::all_slices, false);
    Irq::enable(Pwm::irq<0>());
    Irq::enable(Pwm::irq<1>());
    clear_wraps();
    (void)PwmPeriodicTick<3, 1>::setup(khz1_div, khz1_top);
    spin_us(50'000);
    PwmPeriodicTick<3, 1>::stop();
    const uint32_t moved0 = wraps0[3];
    const uint32_t moved1 = wraps1[3];
    print(serial, "  slice 3 enabled on IRQ1 instead: ", moved1, " wraps on line 1, ", moved0,
          " on line 0", crlf);
    bench.verdict("a slice moved to the other line is served there and nowhere else - the line is "
                  "the enable register's business, not the slice's",
                  moved1 >= 49u && moved1 <= 51u && moved0 == 0u);
    Pwm::stop(Pwm::all_slices);
    Pwm::clear_pending(Pwm::all_slices);
}

// =============================================================================
// c - the wire: the frequency and the duty
// =============================================================================
void tc_wire() {
    if (!wire_6b_7b()) {
        return;
    }
    Pwm::stop(Pwm::all_slices);
    uint8_t ok = 0;
    const uint32_t rates[] = {1000, 100'000, 1'000'000};
    for (uint8_t k = 0; k < 3u; ++k) {
        // TOP by rate so the divider lands on a whole number: 6249 at
        // 1 kHz (divider 24), 1499 at 100 kHz (1), 149 at 1 MHz (1).
        bool up = false;
        if (k == 0u) { up = PwmOutput<13, khz1_top>::setup_hz(clock, 1000); PwmOutput<13, khz1_top>::duty(3125); }
        if (k == 1u) { up = PwmOutput<13, khz100_top>::setup_hz(clock, 100'000); PwmOutput<13, khz100_top>::duty(750); }
        if (k == 2u) { up = PwmOutput<13, 149>::setup_hz(clock, 1'000'000); PwmOutput<13, 149>::duty(75); }
        (void)Edges7::setup();
        const uint32_t window = k == 0u ? 500'000u : 50'000u;
        const uint32_t hz = measure_hz<Edges7>(window);
        const bool good = up && within(hz, rates[k], 10);
        print(serial, "  ", rates[k], " Hz asked: ", hz, " Hz counted on GP15 over ",
              window / 1000u, " ms", good ? "" : "  OUT", crlf);
        if (good) {
            ++ok;
        }
        Edges7::release();
    }
    bench.verdict("the frequency counted by edges on the wire at 1 kHz, 100 kHz and 1 MHz, each "
                  "within one per cent",
                  ok == 3u);

    (void)Out6B::setup_hz(clock, 10'000);
    (void)Level7::setup(level_div);
    const uint16_t levels[] = {0, 250, 500, 750, 1000};
    uint8_t exact = 0;
    for (uint8_t k = 0; k < 5u; ++k) {
        Out6B::duty(levels[k]);
        spin_us(300);
        const uint32_t pm = measure_duty_pm<Level7>();
        const uint32_t want = levels[k];   // per mille of 1000
        const bool good = (want == 0u && pm == 0u) || (want == 1000u && pm >= 999u) ||
                          (want != 0u && want != 1000u && within(pm, want, 10));
        print(serial, "  level ", levels[k], " of 1000: ", pm, " per mille high on GP15",
              good ? "" : "  OUT", crlf);
        if (good) {
            ++exact;
        }
    }
    bench.verdict("the duty counted by level on the wire at 0, 25, 50, 75 and 100 %: each within "
                  "one per cent, the two ends exactly 0 and full (glitch-free, 12.5.2.2)",
                  exact == 5u);
    Level7::release();
    all_off();
}

// =============================================================================
// d - phase-correct, inversion, the buffered level
// =============================================================================
void td_modes() {
    if (!wire_6b_7b()) {
        return;
    }
    Pwm::stop(Pwm::all_slices);
    const auto ten_k = pwm_config_for(SysClock::hz, 10'000, 999);   // 15 exactly
    (void)Out6B::setup(ten_k->divider, true);
    Out6B::duty(500);
    (void)Edges7::setup();
    const uint32_t hz_pc = measure_hz<Edges7>(50'000);
    Edges7::release();
    print(serial, "  phase-correct at the 10 kHz divider (", ten_k->divider.sixteenths() / 16u,
          "): ", hz_pc, " Hz on the wire", crlf);
    bench.verdict("phase-correct mode halves the frequency of the same TOP and divider: 5 kHz",
                  within(hz_pc, 5000, 10));

    (void)Level7::setup(level_div);
    Out6B::duty(250);
    spin_us(300);
    const uint32_t pm_pc = measure_duty_pm<Level7>();
    bench.verdict("... and its duty is the level's fraction still: 25 %", within(pm_pc, 250, 10));

    (void)Out6B::setup_hz(clock, 10'000, false, true);
    Out6B::duty(250);
    spin_us(300);
    const uint32_t pm_inv = measure_duty_pm<Level7>();
    print(serial, "  inverted at level 250: ", pm_inv, " per mille high", crlf);
    bench.verdict("the inverted output complements the duty: level 250 reads 75 % high",
                  within(pm_inv, 750, 10));

    // The buffered level: a change written mid-period lands at the wrap -
    // the pad shows the OLD duty until then. Eight trials. TOP 999 at 256
    // is 1.71 ms a period, 1.71 us a count.
    (void)Out6B::setup(PwmDivider{0, 0});
    using S = PwmSlice<6>;
    uint8_t buffered = 0;
    for (uint8_t trial = 0; trial < 8u; ++trial) {
        Out6B::duty(100);
        spin_us(5'000);
        const uint16_t soft = S::level(1);
        // ONE read per iteration: two reads straddling the wrap (999,
        // then 0) would satisfy a two-read condition at once.
        uint16_t at = 0;
        do {
            at = S::counter();
        } while (at < 200u || at > 300u);
        const bool pre = Pin<15>::read();
        Out6B::duty(900);
        const bool post = Pin<15>::read();
        while (S::counter() < 400u) {
        }
        const bool mid = Pin<15>::read();
        while (S::counter() > 100u) {   // the wrap
        }
        spin_us(180);   // count ~105, inside the new level
        const bool after = Pin<15>::read();
        const bool ok = !pre && !post && !mid && after;
        if (ok) {
            ++buffered;
        } else {
            print(serial, "  trial ", trial, ": software copy ", soft, ", at count ", at,
                  " the pad ", pre ? "HIGH" : "low", " before the write and ",
                  post ? "HIGH" : "low", " after, ", mid ? "HIGH" : "low", " at 400, ",
                  after ? "high" : "LOW", " past the wrap", crlf);
        }
    }
    print(serial, "  a level written mid-period (old 100, new 900): ", buffered,
          " of 8 trials showed the old duty until the wrap", crlf);
    bench.verdict("a level written mid-period is taken at the wrap and not before (the double "
                  "buffer of 12.5.2.3), eight of eight",
                  buffered == 8u);
    Level7::release();
    all_off();
}

// =============================================================================
// e - the pair, and the dead time measured
// =============================================================================
void te_pair() {
    if (!wire_6b_7b()) {
        return;
    }
    Pwm::stop(Pwm::all_slices);
    const auto cfg = pwm_config_for(SysClock::hz, 10'000, 999);
    bool ok = cfg && Pair6::setup(cfg->divider, 100);
    Pair6::duty(400);
    (void)Level7::setup(level_div);
    spin_us(300);
    const uint32_t pm_b = measure_duty_pm<Level7>();
    const uint16_t lb = Pair6::Slice::level(1);
    print(serial, "  the pair at level 400 with 100 counts of dead time: B's level ", lb,
          ", B high ", pm_b, " per mille", crlf);
    bench.verdict("the pair: A at 400 of 1000, B inverted at 400 + 100, so B is high 50 % of the "
                  "period - the dead time by arithmetic, no dead-time unit existing here",
                  ok && lb == 500u && within(pm_b, 500, 10));

    Pair6::duty(1000);
    spin_us(300);
    const uint32_t pm_full = measure_duty_pm<Level7>();
    bench.verdict("at the full level B never rises (its level clamped at TOP + 1)", pm_full == 0u);

    ok = cfg && Pair6::setup(cfg->divider, 100, true);
    Pair6::duty(400);
    spin_us(300);
    const uint32_t pm_pc = measure_duty_pm<Level7>();
    print(serial, "  phase-correct: B high ", pm_pc, " per mille", crlf);
    bench.verdict("phase-correct: the same fractions, the dead time now at both transitions",
                  ok && within(pm_pc, 500, 10));
    Level7::release();

    // THE DEAD TIME ITSELF, on both pads at once: A reaches GP14 over the
    // third wire and B reaches GP15 over the second, and one SIO word read
    // samples them in the same instant. At TOP 999 and divider 256 a count
    // is 1.71 us, so 100 counts of dead time is 171 us of a 1.71 ms period
    // - a tenth of it, twice in a phase-correct period and once in a plain
    // one, and NEVER a moment with both outputs high.
    if (!wire_6a_7a()) {
        all_off();
        return;
    }
    (void)Pair6::setup(PwmDivider{0, 0}, 100);
    Pair6::duty(400);
    (void)Pin<14>::input(PinPull::none);
    (void)Pin<15>::input(PinPull::none);
    spin_us(5000);
    const PairOverlap o = sample_overlap(14, 15, 100'000);
    print(serial, "  A on GP12->GP14 and B on GP13->GP15 sampled together, ", o.samples,
          " samples: A high ", o.a_pm, " per mille, both high ", o.both_high_pm, ", both low ",
          o.both_low_pm, crlf);
    bench.verdict("THE DEAD TIME MEASURED: the two outputs of the pair are never high at the same "
                  "sample, and both are low for a tenth of the period - the 100 counts of dead "
                  "time in a period of 1000",
                  o.both_high_pm == 0u && within_pm(o.both_low_pm, 100, 30) &&
                      within_pm(o.a_pm, 400, 30));
    (void)Pair6::release();
    all_off();
}

// =============================================================================
// f - the second wire, and two outputs in lockstep
// =============================================================================
void tf_second_wire() {
    if (!wire_0b_4b()) {
        return;
    }
    Pwm::stop(Pwm::all_slices);
    (void)Out0B::setup_hz(clock, 1000);   // TOP 1499, divider 100
    Out0B::duty(750);
    (void)Edges4::setup();
    const uint32_t hz = measure_hz<Edges4>(500'000);
    Edges4::release();
    (void)Level4::setup(level_div);
    const uint32_t pm = measure_duty_pm<Level4>();
    Level4::release();
    print(serial, "  slice 0 B on GP17 -> GP9: ", hz, " Hz, ", pm, " per mille high", crlf);
    bench.verdict("the second wire: slice 0's B output at 1 kHz and 50 % measured by slice 4",
                  within(hz, 1000, 10) && within(pm, 500, 10));

    if (wire_present<13, 15>()) {
        // Both outputs at 1 kHz from equal configurations, started
        // together: their counters keep a FIXED relationship - each
        // divider keeps its own phase across a stop and a start.
        (void)PwmOutput<13, 1499>::setup(PwmDivider{100, 0});
        (void)Out0B::setup(PwmDivider{100, 0});
        Pwm::stop(Pwm::all_slices);
        PwmSlice<6>::counter(0);
        PwmSlice<0>::counter(0);
        Pwm::start(PwmSlice<6>::bit | PwmSlice<0>::bit);
        int32_t lags[3];
        for (uint8_t k = 0; k < 3u; ++k) {
            spin_us(100'000);
            // Read in both orders: the read's own delay cancels.
            const uint16_t a1 = PwmSlice<6>::counter();
            const uint16_t b1 = PwmSlice<0>::counter();
            const uint16_t b2 = PwmSlice<0>::counter();
            const uint16_t a2 = PwmSlice<6>::counter();
            lags[k] = (static_cast<int32_t>(b1) - static_cast<int32_t>(a1) +
                       static_cast<int32_t>(b2) - static_cast<int32_t>(a2)) / 2;
        }
        print(serial, "  slices 6 and 0 started together at divider 100: lag ", lags[0], ", ",
              lags[1], ", ", lags[2], " counts at 100, 200 and 300 ms", crlf);
        int32_t lo = lags[0];
        int32_t hi = lags[0];
        for (uint8_t k = 1; k < 3u; ++k) {
            lo = lags[k] < lo ? lags[k] : lo;
            hi = lags[k] > hi ? lags[k] : hi;
        }
        bench.verdict("two outputs started by the global enable keep a fixed phase relationship, "
                      "within two counts, unchanged over 300 ms",
                      hi - lo <= 1 && lo >= -2 && hi <= 2);
    }
    all_off();
}

// =============================================================================
// g - the divider ladder
// =============================================================================
void tg_ladder() {
    if (!wire_6b_7b()) {
        return;
    }
    Pwm::stop(Pwm::all_slices);
    const PwmDivider ladder[] = {{1, 0}, {2, 8}, {10, 0}, {255, 15}, {0, 0}};
    uint8_t ok = 0;
    for (uint8_t k = 0; k < 5u; ++k) {
        (void)PwmOutput<13, 199>::setup(ladder[k]);
        PwmOutput<13, 199>::duty(100);
        (void)Edges7::setup();
        const uint32_t window = ladder[k].sixteenths() >= 160u ? 400'000u : 40'000u;
        const uint32_t hz = measure_hz<Edges7>(window);
        const uint32_t want = pwm_output_hz(SysClock::hz, {.divider = ladder[k], .top = 199});
        const bool good = within(hz, want, 10);
        print(serial, "  divider ", ladder[k].sixteenths() / 16u, " + ",
              ladder[k].sixteenths() % 16u, "/16: ", hz, " Hz counted, ", want, " computed",
              good ? "" : "  OUT", crlf);
        if (good) {
            ++ok;
        }
        Edges7::release();
    }
    bench.verdict("the divider ladder on the wire at TOP 199 - one, two and a half, ten, 255 and "
                  "15/16, 256 - each frequency within one per cent of the arithmetic (the "
                  "fractional divider averages)",
                  ok == 5u);
    bench.verdict("and 256 is the END of the ladder here: a fraction on top of DIV_INT 0 is "
                  "refused by the driver before any register moves, which is 12.5.2.6's own rule",
                  !PwmOutput<13, 199>::setup(PwmDivider{0, 8}));
    all_off();
}

// =============================================================================
// h - the lamp over three outputs
// =============================================================================
void th_lamp() {
    if (!wire_6b_7b() || !wire_0b_4b() || !wire_6a_7a()) {
        return;
    }
    Pwm::stop(Pwm::all_slices);
    using R = PwmOutput<12, 999>;
    using G = PwmOutput<13, 999>;
    using B = PwmOutput<17, 999>;
    using Lamp = RgbLamp<R, G, B>;
    (void)R::setup_hz(clock, 10'000);
    G::attach();
    (void)B::setup_hz(clock, 10'000);
    Lamp::show({255, 128, 64});
    (void)Level7::setup(level_div);
    (void)Level4::setup(level_div);
    (void)Pin<14>::input(PinPull::none);   // R reaches this pad over the third wire
    spin_us(300);
    const uint32_t g_pm = measure_duty_pm<Level7>();
    const uint32_t b_pm = measure_duty_pm<Level4>();
    const uint32_t r_pm = sample_pad_pm(14, 50'000);
    print(serial, "  RgbLamp{255, 128, 64} over three outputs: R (GP12 -> GP14) ", r_pm,
          " per mille sampled, G (GP13) ", g_pm, " per mille, B (GP17) ", b_pm,
          " per mille, R's level ", R::duty(), crlf);
    bench.verdict("util/rgb_lamp.hpp over three PwmOutputs: 128 of 255 reads 50 %, 64 reads 25 %, "
                  "255 sets the full level - and that full level is a pad held HIGH with no gap, "
                  "which is the glitch-free 100 % of 12.5.2.2 seen on a third output",
                  within(g_pm, 502, 10) && within(b_pm, 251, 12) && R::duty() == 1000u &&
                      r_pm >= 995u);
    Lamp::off();
    spin_us(300);
    const uint32_t r_off = sample_pad_pm(14, 50'000);
    bench.verdict("... and off() is 0 % on all three: two counted on their wires, the third "
                  "sampled on its own",
                  measure_duty_pm<Level7>() == 0u && measure_duty_pm<Level4>() == 0u &&
                      r_off == 0u);
    Level7::release();
    Level4::release();
    all_off();
}

// =============================================================================
// j - the four slices this chip added
// =============================================================================

/// ONE OUTPUT of a slice above the seventh, on THE TWO PADS that carry it
/// (table 1130's repeat: GP32 and GP40 are both slice 8's A). No wire of
/// this desk reaches those pads, and no slice can read an A output
/// anyway, so what measures them is SIO - which reads a pad whoever owns
/// it - and both bits come from ONE word read, which is what makes
/// "the same signal appears on both" an exact verdict rather than a
/// sampled one. The whole half is behind the package fact the stratum
/// states, and the pins are template parameters so that this letter
/// compiles for the package that bonds none of them.
template <uint8_t pin, uint8_t pin_alt, bool bonded>
void high_slice_pads(const char* what) {
    if constexpr (bonded) {
        Pwm::stop(Pwm::all_slices);
        using Out = PwmOutput<pin, 999>;
        using OutAlt = PwmOutput<pin_alt, 999>;
        const bool up = Out::setup(PwmDivider{0, 0});   // 586 Hz, 1.71 us a count
        OutAlt::attach();
        const uint16_t levels[] = {250, 500, 750};
        uint8_t good = 0;
        uint32_t differed = 0;
        uint32_t samples = 0;
        for (uint8_t k = 0; k < 3u; ++k) {
            Out::duty(levels[k]);
            spin_us(5000);
            const TwoPads s = sample_pair_hi(pin, pin_alt, 60'000);
            differed += s.differed;
            samples += s.samples;
            const bool ok = within_pm(s.a_pm, levels[k], 25) && within_pm(s.b_pm, levels[k], 25);
            print(serial, "  ", what, " at level ", levels[k], " of 1000: GP", pin, " ", s.a_pm,
                  " per mille, GP", pin_alt, " ", s.b_pm, " per mille of ", s.samples, " samples",
                  ok ? "" : "  OUT", crlf);
            if (ok) {
                ++good;
            }
        }
        bench.verdict("a slice above the seventh drives a pad on this package: three levels "
                      "sampled on its output, each within 25 per mille of the level asked for",
                      up && good == 3u);
        bench.verdict("AND ONE OUTPUT SELECTED ON TWO GPIOs APPEARS ON BOTH (12.5.2): the second "
                      "pad carries the same signal, and in every sample of one word read the two "
                      "agreed",
                      differed == 0u && samples > 1000u);
        Out::release();
        OutAlt::release();
        Pwm::stop(Pwm::all_slices);
    } else {
        (void)what;
    }
}

void tj_new_slices() {
    Pwm::stop(Pwm::all_slices);
    Pwm::clear_pending(Pwm::all_slices);
    clear_wraps();

    // (1) As repeating timers with no pad - two on each line. This is what
    // slices 8..11 ARE in the QFN-60 package, where no pad of theirs is
    // bonded, and it is 12.5.1.1's own reason for the second line.
    (void)Tick8::setup(khz1_div, khz1_top);
    (void)Tick9::setup(khz1_div, khz1_top);
    (void)Tick10::setup(khz1_div, khz1_top);
    (void)Tick11::setup(khz1_div, khz1_top);
    const uint32_t t0 = us_now();
    spin_us(200'000);
    const uint32_t took = us_now() - t0;
    const uint32_t n8 = wraps0[8];
    const uint32_t n9 = wraps1[9];
    const uint32_t n10 = wraps0[10];
    const uint32_t n11 = wraps1[11];
    Tick8::stop();
    Tick9::stop();
    Tick10::stop();
    Tick11::stop();
    print(serial, "  slices 8..11 at 1 kHz over ", took, " us: ", n8, " ", n9, " ", n10, " ", n11,
          " wraps (8 and 10 on IRQ0, 9 and 11 on IRQ1)", crlf);
    bench.verdict("THE FOUR SLICES THIS CHIP ADDED run as repeating timers, two on each interrupt "
                  "line, 200 wraps each in 200 ms - which is all they can be in the QFN-60, whose "
                  "pads stop at GP29",
                  n8 >= 199u && n8 <= 201u && n9 >= 199u && n9 <= 201u && n10 >= 199u &&
                      n10 <= 201u && n11 >= 199u && n11 <= 201u);
    bench.verdict("their requests are the top four rows of the DREQ table, one per slice",
                  PwmSlice<8>::dreq == Dreq::pwm_wrap8 && PwmSlice<11>::dreq == Dreq::pwm_wrap11);

    // (2) Their PADS - the two each output has - where the package bonds
    // them.
    if (pwm_slice_has_pads(8)) {
        high_slice_pads<32, 40, pwm_slice_has_pads(8)>("slice 8's A output");
        high_slice_pads<38, 46, pwm_slice_has_pads(11)>("slice 11's A output");
    } else {
        bench.verdict("DECLINED: this image is built for the package whose pads stop at GP29, "
                      "where slices 8..11 reach no output at all and the timers above are all "
                      "they can be",
                      false);
    }
}

// =============================================================================
// y - the instruments (diagnostics, outside z)
// =============================================================================
void ty_probe() {
    using S6 = PwmSlice<6>;
    Pwm::stop(Pwm::all_slices);

    // (1) The polarity: GP15 sampled through a period at 1.71 us a count.
    struct Case { const char* name; bool invert; uint16_t level; };
    const Case cases[] = {{"plain 250", false, 250},
                          {"inverted 250", true, 250},
                          {"plain 100", false, 100},
                          {"inverted 0", true, 0}};
    for (const Case& c : cases) {
        (void)Out6B::setup(PwmDivider{0, 0}, false, c.invert);
        Out6B::duty(c.level);
        (void)Pin<15>::input(PinPull::none);
        spin_us(5000);
        uint32_t high = 0;
        uint32_t total = 0;
        uint16_t first_low_at = 0xFFFF;
        uint16_t first_high_at = 0xFFFF;
        while (S6::counter() != 0u) {
        }
        for (;;) {
            const uint16_t ctr = S6::counter();
            const bool h = Pin<15>::read();
            if (h) {
                ++high;
                if (first_high_at == 0xFFFFu) { first_high_at = ctr; }
            } else if (first_low_at == 0xFFFFu) {
                first_low_at = ctr;
            }
            ++total;
            if (ctr >= 990u) { break; }
        }
        print(serial, "  ", c.name, ": CSR=", hex(S6::csr()), " CC=", hex(S6::cc()), " high ",
              high * 1000u / total, " per mille of ", total, " samples; first high at count ",
              first_high_at, ", first low at ", first_low_at, crlf);
    }
    Out6B::release();
    (void)Pin<15>::release();

    // (2) One read at a growing delay after a fresh nudge: stale on the
    // first read, or stale for a while?
    using S = PwmSlice<3>;
    using T = PwmSlice<5>;
    Pwm::stop(Pwm::all_slices);
    (void)S::configure({.divider = {2, 0}, .top = 0xFFFF});
    (void)T::configure({.divider = {2, 0}, .top = 0xFFFF});
    S::counter(0);
    T::counter(0);
    Pwm::start(S::bit | T::bit);
    spin_us(2000);
    const uint32_t delays[] = {0, 1, 2, 5, 10, 50, 100, 1000};
    print(serial, "  one read after an advance, at +us: ");
    for (uint32_t d : delays) {
        (void)S::advance_phase();
        (void)S::retard_phase();   // net zero
        spin_us(d);
        const uint16_t a = S::counter();
        const uint16_t b = T::counter();
        print(serial, d, ":", static_cast<int32_t>(b) - static_cast<int32_t>(a), " ");
        spin_us(2000);
    }
    print(serial, crlf, "  one read after a start, at +us: ");
    for (uint32_t d : delays) {
        Pwm::stop(S::bit | T::bit);
        S::counter(0);
        T::counter(0);
        Pwm::start(S::bit | T::bit);
        spin_us(d);
        const uint16_t a = S::counter();
        const uint16_t b = T::counter();
        print(serial, d, ":", static_cast<int32_t>(b) - static_cast<int32_t>(a), " ");
    }
    print(serial, crlf);
    Pwm::stop(Pwm::all_slices);

    // (3) The level written mid-period, the pad sampled through.
    (void)Out6B::setup(PwmDivider{0, 0});
    Out6B::duty(100);
    (void)Pin<15>::input(PinPull::none);
    spin_us(5000);
    while (S6::counter() < 200u || S6::counter() > 300u) {
    }
    const uint16_t at_write = S6::counter();
    const bool before = Pin<15>::read();
    Out6B::duty(900);
    const bool after = Pin<15>::read();
    uint16_t rose_at = 0xFFFF;
    uint16_t fell_at = 0xFFFF;
    bool last = after;
    const uint32_t t0 = us_now();
    while (us_now() - t0 < 4000u) {
        const uint16_t ctr = S6::counter();
        const bool h = Pin<15>::read();
        if (h && !last && rose_at == 0xFFFFu) { rose_at = ctr; }
        if (!h && last && fell_at == 0xFFFFu) { fell_at = ctr; }
        last = h;
    }
    print(serial, "  level 100 -> 900 written at count ", at_write, ": the pad ",
          before ? "high" : "low", " before, ", after ? "high" : "low", " after; then rose at "
          "count ", rose_at, ", fell at ", fell_at, "; CC=", hex(S6::cc()), crlf);
    Out6B::release();
    (void)Pin<15>::release();
    Pwm::stop(Pwm::all_slices);
}

const char* arch_name() { return core_kind == CoreKind::hazard3 ? "Hazard3" : "Cortex-M33"; }

void banner() {
    print(serial, crlf, "test_rp2350_pwm - the PWM block (datasheet 12.5) on ", arch_name(),
          ": twelve slices, two interrupt lines; slice 6 B on GP13 -> GP15 (slice 7 B), slice 0 B "
          "on GP17 -> GP9 (slice 4 B), slice 6 A on GP12 -> GP14; clk_sys=", SysClock::hz, " Hz",
          crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
//
// THE TWO WRAP VECTORS, bound by the names the crt gives the lines - the
// same names on both architectures, through the Arm vector table on one
// and Hazard3's own dispatch on the other.

extern "C" void isr_pwm_wrap_0() {
    entries0 = entries0 + 1u;
    const uint16_t up = brio::Pwm::isr<0>();
    for (uint8_t i = 0; i < brio::pwm_slice_count; ++i) {
        if ((up & (1u << i)) != 0u) {
            wraps0[i] = wraps0[i] + 1u;
        }
    }
}
extern "C" void isr_pwm_wrap_1() {
    entries1 = entries1 + 1u;
    const uint16_t up = brio::Pwm::isr<1>();
    for (uint8_t i = 0; i < brio::pwm_slice_count; ++i) {
        if ((up & (1u << i)) != 0u) {
            wraps1[i] = wraps1[i] + 1u;
        }
    }
}

extern "C" void isr_uart0() { (void)Serial::isr(); }
extern "C" void isr_systick() { brio::Ticker::tick(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool timer_ok = brio::Timer<0>::init(clock);
    const bool pwm_ok = brio::Pwm::reset();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    (void)Led::output(false);
    brio::Irq::enable(brio::Pwm::irq<0>());
    brio::Irq::enable(brio::Pwm::irq<1>());
    brio::enable_interrupts();

    bench.letter('a', "the block, wireless", ta_block);
    bench.letter('b', "the two interrupt lines", tb_lines);
    bench.letter('c', "the wire: frequency and duty", tc_wire);
    bench.letter('d', "phase-correct, inversion, the buffered level", td_modes);
    bench.letter('e', "the pair, and its dead time measured", te_pair);
    bench.letter('f', "the second wire, lockstep", tf_second_wire);
    bench.letter('g', "the divider ladder", tg_ladder);
    bench.letter('h', "the lamp over three outputs", th_lamp);
    bench.letter('j', "the four slices this chip added", tj_new_slices);
    bench.letter('y', "the instruments (diagnostics, outside z)", ty_probe, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL150" : "FAILED",
                    " timer0=", timer_ok ? "1us" : "FAILED", " pwm=",
                    pwm_ok ? "released" : "FAILED", " tick=", tick_ok ? "on" : "FAILED",
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
