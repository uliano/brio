// test_rp2040_pwm - the reference bench suite for the RP2040's PWM
// block (rp2040/pwm.hpp over datasheet 4.5): the slices wireless (the
// arithmetic, the lockstep start, the phase nudges, the wrap as a tick
// and as a DMA pace), and on two wires between the chip's own slices
// (an output of one slice into the B input of another: the frequency
// and the duty measured by the block itself), the pair with its dead
// time, the divider ladder, util/rgb_lamp.hpp over three outputs.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp). It is a REFERENCE test.
//
// TWO WIRES, both part of the standing self-links of this board:
//
//   GP13 (slice 6 B, an output)  ->  GP15 (slice 7 B, an INPUT)
//   GP17 (slice 0 B, an output)  ->  GP9  (slice 4 B, an INPUT)
//
// Only a B pin is a PWM input (table 515), which is why both wires end
// on an odd GPIO. The measuring slice counts the edges of the wire
// (the frequency) or the divided clock while the wire is high (the
// duty) over a window the system timer times. With a wire missing its
// input reads low and the letter declines with the reason.
//
// THE RULER IS THE SYSTEM TIMER (rp2040/timer.hpp).
//
// What is exercised, letter by letter:
//   a  the block, WIRELESS: table 515 as the driver states it, the
//      period arithmetic and the chooser, the reset state, a
//      configuration read back, the counter running and stopped, two
//      slices started in lockstep by the global enable, the phase
//      nudged one count forward and back, the forced interrupt
//   b  THE WRAP AS A TICK, wireless: a slice's wrap interrupt counted
//      against the timer at 1 kHz and 100 kHz; THE DMA PACED BY THE
//      WRAP: a 64-word level table streamed into CC, one per period
//   c  THE WIRE 6B -> 7B: the frequency at 1 kHz, 100 kHz and 1 MHz
//      counted by edges; the duty at 0, 25, 50, 75 and 100 % counted by
//      level - the two ends glitch-free
//   d  phase-correct mode halving the frequency, the inverted output
//      complementing the duty, a level change taken at the wrap
//   e  THE PAIR: 6A and 6B with a dead time of 100 counts, B measured
//      on the wire in both modes
//   f  THE SECOND WIRE 0B -> 4B: another slice measured, and the two
//      outputs started in lockstep
//   g  THE DIVIDER LADDER on the wire: one, two and a half, ten, the
//      top of the range and 256, each frequency within one per cent
//   h  THE LAMP: util/rgb_lamp.hpp over three outputs, two of them
//      measured on the wires
//   y  (outside z) THE INSTRUMENTS: the pad sampled through a period
//      for the polarity, the level counter on the same signals, the
//      edges counted against the wraps, the counter read at a growing
//      delay after a start, a nudge and a plain write, a level written
//      mid-period followed on the pad
//
// build: boards = pico,weact2040
// build: monitor_speed = 115200

#include <stdint.h>

#include "rp2040/clock.hpp"
#include "rp2040/dma.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/pwm.hpp"
#include "rp2040/ticker.hpp"
#include "rp2040/timer.hpp"
#include "rp2040/uart.hpp"
#include "util/print.hpp"
#include "util/rgb_lamp.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using SysClock = Clock<ClockSource::pll, 125'000'000>;
constexpr SysClock clock;

constexpr UartPins console_pins{
    .tx = {0, PinFunction::uart},
    .rx = {1, PinFunction::uart},
};
using Serial = Uart<0, console_pins>;
constexpr Serial serial;
using Led = Pin<25>;

TestBench<Serial, 16> bench;

// The outputs under test and the inputs that measure them.
using Out6B = PwmOutput<13, 999>;      // slice 6 B on GP13 -> GP15
using Out0B = PwmOutput<17, 1249>;     // slice 0 B on GP17 -> GP9
using Edges7 = PwmEdgeCounter<15>;     // slice 7 counting GP15's edges
using Level7 = PwmLevelCounter<15>;    // slice 7 counting while GP15 is high
using Edges4 = PwmEdgeCounter<9>;      // slice 4 on GP9
using Level4 = PwmLevelCounter<9>;
using Pair6 = PwmPair<12, 13, 999>;    // slice 6 A on GP12, B on GP13
using Tick3 = PwmPeriodicTick<3>;
using Stream = DmaTxEngine<6, uint32_t>;

volatile uint32_t wraps[8];
volatile uint32_t wrap_entries = 0;
volatile bool stream_done = false;

uint32_t us_now() { return Timer::now_low(); }
void spin_us(uint32_t us) {
    const uint32_t t0 = us_now();
    while (us_now() - t0 < us) {
    }
}
void clear_wraps() {
    for (uint8_t i = 0; i < 8u; ++i) {
        wraps[i] = 0;
    }
    wrap_entries = 0;
}

/// The level counter's full-scale over a window: clk_sys / 256 cycles
/// per microsecond, times the window.
constexpr uint32_t level_window_us = 100'000;
constexpr PwmDivider level_div{0, 0};   // 256
constexpr uint32_t level_full = static_cast<uint32_t>((static_cast<uint64_t>(SysClock::hz) / 256u) * level_window_us / 1'000'000u);

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

/// Whether a wire is fitted: the output pin driven both ways as a GPIO,
/// the input pin following.
template <uint8_t out_pin, uint8_t in_pin>
bool wire_present() {
    Pin<in_pin>::input(PinPull::none);
    Pin<out_pin>::output(false);
    spin_us(20);
    const bool low = !Pin<in_pin>::read();
    Pin<out_pin>::set();
    spin_us(20);
    const bool high = Pin<in_pin>::read();
    Pin<out_pin>::release();
    Pin<in_pin>::release();
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

void all_off() {
    Pwm::interrupts(0xFF, false);
    Pwm::stop(0xFF);
    Out6B::release();
    Out0B::release();
    Pin<12>::release();
    Pin<15>::release();
    Pin<9>::release();
}

// =============================================================================
// a - the block, wireless
// =============================================================================
void ta_block() {
    bench.verdict("table 515 as the driver states it: GPIO 0 is 0A, 1 is 0B, 13 is 6B, 15 is 7B, 16 is 0A "
                  "again, 29 is 6B; only an odd GPIO is an input",
                  pwm_slice_of(0) == 0u && !pwm_pin_is_b(0) && pwm_pin_is_b(1) && pwm_slice_of(13) == 6u &&
                      pwm_pin_is_b(13) && pwm_slice_of(15) == 7u && pwm_slice_of(16) == 0u && pwm_slice_of(29) == 6u &&
                      pwm_pin_is_b(29));
    const auto khz1 = pwm_config_for(SysClock::hz, 1000);
    const auto hz1 = pwm_config_for(SysClock::hz, 1);
    print(serial, "  at ", SysClock::hz / 1'000'000u, " MHz: TOP 124 at one is ", pwm_output_hz(SysClock::hz, {.top = 124}),
          " Hz; 1 kHz at TOP 65534 wants ", khz1 ? khz1->divider.sixteenths() : 0u, " sixteenths -> ",
          khz1 ? pwm_output_hz(SysClock::hz, *khz1) : 0u, " Hz; the floor is ",
          pwm_output_hz(SysClock::hz, {.divider = {0, 0}}), " Hz; 1 Hz is ", hz1 ? "reachable" : "refused", crlf);
    bench.verdict("the period arithmetic: TOP 124 at one is 1 MHz, 500 kHz phase-correct; 1 kHz at the finest "
                  "TOP is 984 Hz at the nearest divider; the floor is 7 Hz; 1 Hz is refused",
                  pwm_output_hz(SysClock::hz, {.top = 124}) == 1'000'000u &&
                      pwm_output_hz(SysClock::hz, {.top = 124, .phase_correct = true}) == 500'000u && khz1 &&
                      pwm_output_hz(SysClock::hz, *khz1) == 984u && pwm_output_hz(SysClock::hz, {.divider = {0, 0}}) == 7u &&
                      !hz1);
    using S = PwmSlice<3>;
    const bool reset_ok = Pwm::reset();
    print(serial, "  slice 3 after reset: CSR=", hex(S::csr()), " DIV=", hex(S::div()), " TOP=", hex(S::top()), " CC=",
          hex(S::cc()), " CTR=", S::counter(), " EN=", hex(Pwm::running()), crlf);
    bench.verdict("the block's reset state: every slice disabled, the divider at one, TOP 0xFFFF, the levels "
                  "and the counter at zero",
                  reset_ok && S::csr() == 0u && S::div() == 0x10u && S::top() == 0xFFFFu && S::cc() == 0u &&
                      S::counter() == 0u && Pwm::running() == 0u);
    const bool cfg = S::configure({.mode = PwmDivMode::level_high, .divider = {3, 4}, .top = 999, .phase_correct = true,
                                   .invert_b = true});
    const PwmSliceConfig back = S::config();
    S::levels(100, 200);
    bench.verdict("a configuration reads back field by field (level mode, 3 and 4/16, TOP 999, phase-correct, "
                  "B inverted, the levels), and an invalid divider is refused",
                  cfg && back.mode == PwmDivMode::level_high && back.divider == PwmDivider{3, 4} && back.top == 999u &&
                      back.phase_correct && !back.invert_a && back.invert_b && S::level(0) == 100u && S::level(1) == 200u &&
                      !S::configure({.divider = {1, 16}}));
    (void)S::configure({.divider = {2, 0}, .top = 0xFFFF});
    S::counter(0);
    S::enable(true);
    spin_us(100);
    const uint16_t c1 = S::counter();
    S::enable(false);
    const uint16_t c2 = S::counter();
    spin_us(100);
    const uint16_t c3 = S::counter();
    print(serial, "  the counter at divider 2: ", c1, " after 100 us running, then ", c2, " and ", c3, " stopped", crlf);
    bench.verdict("the counter runs when enabled (about 6250 counts in 100 us at divider 2) and stands when "
                  "disabled",
                  within(c1, 6250, 100) && c2 == c3);
    // Lockstep: two slices with equal configuration started by the
    // global enable read equal counters.
    using T = PwmSlice<5>;
    (void)T::configure({.divider = {2, 0}, .top = 0xFFFF});
    S::counter(0);
    T::counter(0);
    Pwm::start(S::bit | T::bit);
    spin_us(50);
    // The lag between the two counters, free of the read's own delay:
    // two pairs in either order, averaged.
    auto lag_now = [] {
        const uint16_t a1 = S::counter();
        const uint16_t b1 = T::counter();
        const uint16_t b2 = T::counter();
        const uint16_t a2 = S::counter();
        return (static_cast<int32_t>(b1) - static_cast<int32_t>(a1) + static_cast<int32_t>(b2) - static_cast<int32_t>(a2)) / 2;
    };
    const int32_t lag = lag_now();
    print(serial, "  slices 3 and 5 started together: lag ", lag, " counts after 50 us", crlf);
    bench.verdict("two slices started by the global enable run in lockstep: their counters agree within a count",
                  Pwm::running() == (S::bit | T::bit) && lag >= -1 && lag <= 1);
    // A warming pair: the first read after the first nudge of a run
    // answers late by some 45 counts (measured), the ones after are
    // current.
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
    bench.verdict("a phase advance puts slice 3 one count ahead of slice 5, a retard brings it back",
                  adv && ret && lag1 == lag0 - 1 && lag2 == lag0);
    Pwm::stop(0xFF);
    Pwm::clear_pending(0xFF);
    // INTF forces the LINE (INTS = INTR & INTE | INTF), not the raw
    // flag: the handler would re-enter without end, so the NVIC line
    // is closed for the check.
    Nvic::disable(Pwm::irq());
    Pwm::force(S::bit, true);
    const bool forced = S::pending() && !S::raw_pending();
    Pwm::force(S::bit, false);
    const bool gone = !S::pending();
    Nvic::enable(Pwm::irq());
    bench.verdict("the forced interrupt appears in the status with no raw flag and leaves with the force",
                  forced && gone && !S::raw_pending());
}

// =============================================================================
// b - the wrap as a tick, and as a DMA pace
// =============================================================================
void tb_wrap() {
    Pwm::stop(0xFF);
    clear_wraps();
    (void)Tick3::setup(PwmDivider{2, 0}, 62499);   // exactly 1 kHz
    const uint32_t t0 = us_now();
    spin_us(200'000);
    const uint32_t n1 = wraps[3];
    const uint32_t took = us_now() - t0;
    Tick3::stop();
    print(serial, "  1 kHz wrap: ", n1, " interrupts in ", took, " us (TOP ", Tick3::Slice::top(), ", divider ",
          Tick3::Slice::config().divider.sixteenths(), "/16)", crlf);
    bench.verdict("a slice's wrap at 1 kHz counted for 200 ms on the timer: 200 interrupts, one either way",
                  n1 >= 199u && n1 <= 201u);
    clear_wraps();
    (void)Tick3::setup(PwmDivider{1, 0}, 1249);    // exactly 100 kHz
    spin_us(10'000);
    const uint32_t n2 = wraps[3];
    Tick3::stop();
    print(serial, "  100 kHz wrap: ", n2, " interrupts in 10 ms", crlf);
    bench.verdict("at 100 kHz, 1000 interrupts in 10 ms, two either way", n2 >= 998u && n2 <= 1002u);
    // The DMA paced by the wrap: 64 levels into CC, one per period.
    using S = PwmSlice<3>;
    static uint32_t table[64];
    for (uint32_t i = 0; i < 64u; ++i) {
        table[i] = (i * 100u) & 0xFFFFu;   // A rises 100 per period
    }
    (void)S::configure({.divider = {1, 0}, .top = 12499});   // 10 kHz
    S::levels(0, 0);
    S::counter(0);
    S::clear_pending();
    Stream::arm(S::cc_address(), S::dreq);
    stream_done = false;
    clear_wraps();
    S::interrupt(true);
    const bool started = Stream::start(table, 64);
    const uint32_t t1 = us_now();
    S::enable(true);
    while (!stream_done && us_now() - t1 < 20'000u) {
    }
    const uint32_t took2 = us_now() - t1;
    spin_us(200);
    const uint32_t wraps_seen = wraps[3];
    S::enable(false);
    S::interrupt(false);
    const uint16_t last = S::level(0);
    print(serial, "  64 levels streamed on the wrap request at 10 kHz: ", stream_done ? "done" : "NOT done", " in ", took2,
          " us, wraps ", wraps_seen, ", the last level ", last, " (expected ", table[63] & 0xFFFFu, ")", crlf);
    bench.verdict("a 64-word level table streamed into CC by a DMA channel paced by the wrap: one word per "
                  "period, 6.4 ms, the last level in place",
                  started && stream_done && within(took2, 6400, 30) && last == (table[63] & 0xFFFFu) && wraps_seen >= 63u &&
                      wraps_seen <= 66u);
    Stream::stop();
}

// =============================================================================
// c - the wire: the frequency and the duty
// =============================================================================
void tc_wire() {
    if (!wire_6b_7b()) {
        return;
    }
    Pwm::stop(0xFF);
    uint8_t ok = 0;
    const uint32_t rates[] = {1000, 100'000, 1'000'000};
    for (uint8_t k = 0; k < 3u; ++k) {
        // TOP by rate so the divider lands on a whole number: 62499 at
        // 1 kHz (divider 2), 1249 at 100 kHz (1), 124 at 1 MHz (1).
        bool up = false;
        if (k == 0u) { up = PwmOutput<13, 62499>::setup_hz(clock, 1000); PwmOutput<13, 62499>::duty(31250); }
        if (k == 1u) { up = PwmOutput<13, 1249>::setup_hz(clock, 100'000); PwmOutput<13, 1249>::duty(625); }
        if (k == 2u) { up = PwmOutput<13, 124>::setup_hz(clock, 1'000'000); PwmOutput<13, 124>::duty(62); }
        (void)Edges7::setup();
        const uint32_t window = k == 0u ? 500'000u : 50'000u;
        const uint32_t hz = measure_hz<Edges7>(window);
        const bool good = up && within(hz, rates[k], 10);
        print(serial, "  ", rates[k], " Hz asked: ", hz, " Hz counted on GP15 over ", window / 1000u, " ms", good ? "" : "  OUT",
              crlf);
        if (good) {
            ++ok;
        }
        Edges7::release();
    }
    bench.verdict("the frequency counted by edges on the wire at 1 kHz, 100 kHz and 1 MHz, each within one per cent",
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
        const bool good = (want == 0u && pm == 0u) || (want == 1000u && pm >= 999u) || (want != 0u && want != 1000u && within(pm, want, 10));
        print(serial, "  level ", levels[k], " of 1000: ", pm, " per mille high on GP15", good ? "" : "  OUT", crlf);
        if (good) {
            ++exact;
        }
    }
    bench.verdict("the duty counted by level on the wire at 0, 25, 50, 75 and 100 %: each within one per cent, "
                  "the two ends exactly 0 and full (glitch-free, 4.5.2.2)",
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
    Pwm::stop(0xFF);
    const auto ten_k = pwm_config_for(SysClock::hz, 10'000, 999);   // 12 and 8/16
    (void)Out6B::setup(ten_k->divider, true);
    Out6B::duty(500);
    (void)Edges7::setup();
    const uint32_t hz_pc = measure_hz<Edges7>(50'000);
    Edges7::release();
    print(serial, "  phase-correct at the 10 kHz divider: ", hz_pc, " Hz on the wire", crlf);
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
    bench.verdict("the inverted output complements the duty: level 250 reads 75 % high", within(pm_inv, 750, 10));
    // The buffered level: a change written mid-period lands at the
    // wrap - the pad shows the OLD duty until then. Eight trials.
    (void)Out6B::setup(PwmDivider{0, 0});   // TOP 999 at 256: 2 ms periods, 2 us a count
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
        spin_us(210);   // count ~105, inside the new level
        const bool after = Pin<15>::read();
        const bool ok = !pre && !post && !mid && after;
        if (ok) {
            ++buffered;
        } else {
            print(serial, "  trial ", trial, ": software copy ", soft, ", at count ", at, " the pad ", pre ? "HIGH" : "low",
                  " before the write and ", post ? "HIGH" : "low", " after, ", mid ? "HIGH" : "low", " at 400, ",
                  after ? "high" : "LOW", " past the wrap", crlf);
        }
    }
    print(serial, "  a level written mid-period (old 100, new 900): ", buffered, " of 8 trials showed the old duty until the wrap",
          crlf);
    bench.verdict("a level written mid-period is taken at the wrap and not before (the double buffer), eight of eight",
                  buffered == 8u);
    Level7::release();
    all_off();
}

// =============================================================================
// e - the pair
// =============================================================================
void te_pair() {
    if (!wire_6b_7b()) {
        return;
    }
    Pwm::stop(0xFF);
    const auto cfg = pwm_config_for(SysClock::hz, 10'000, 999);
    bool ok = cfg && Pair6::setup(cfg->divider, 100);
    Pair6::duty(400);
    (void)Level7::setup(level_div);
    spin_us(300);
    const uint32_t pm_b = measure_duty_pm<Level7>();
    const uint16_t lb = Pair6::Slice::level(1);
    print(serial, "  the pair at level 400 with 100 counts of dead time: B's level ", lb, ", B high ", pm_b, " per mille", crlf);
    bench.verdict("the pair: A at 400 of 1000, B inverted at 400 + 100, so B is high 50 % of the period - the "
                  "dead time by arithmetic",
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
    bench.verdict("phase-correct: the same fractions, the dead time now at both transitions", ok && within(pm_pc, 500, 10));
    Level7::release();
    Pair6::release();
    all_off();
}

// =============================================================================
// f - the second wire, and two outputs in lockstep
// =============================================================================
void tf_second_wire() {
    if (!wire_0b_4b()) {
        return;
    }
    Pwm::stop(0xFF);
    (void)Out0B::setup_hz(clock, 1000);   // TOP 1249, divider 100
    Out0B::duty(625);
    (void)Edges4::setup();
    const uint32_t hz = measure_hz<Edges4>(500'000);
    Edges4::release();
    (void)Level4::setup(level_div);
    const uint32_t pm = measure_duty_pm<Level4>();
    Level4::release();
    print(serial, "  slice 0 B on GP17 -> GP9: ", hz, " Hz, ", pm, " per mille high", crlf);
    bench.verdict("the second wire: slice 0's B output at 1 kHz and 50 % measured by slice 4", within(hz, 1000, 10) && within(pm, 500, 10));
    if (wire_present<13, 15>()) {
        // Both outputs at 1 kHz from equal configurations, started
        // together: their counters agree.
        (void)PwmOutput<13, 1249>::setup(PwmDivider{100, 0});
        (void)Out0B::setup(PwmDivider{100, 0});
        Pwm::stop(0xFF);
        PwmSlice<6>::counter(0);
        PwmSlice<0>::counter(0);
        Pwm::start(PwmSlice<6>::bit | PwmSlice<0>::bit);
        // The relationship is FIXED, not zero: each divider keeps its
        // own phase across a stop and a start (measured: two counts
        // apart at divider 100, and the same two counts a second later).
        int32_t lags[3];
        for (uint8_t k = 0; k < 3u; ++k) {
            spin_us(100'000);
            // Read in both orders: the read's own delay cancels.
            const uint16_t a1 = PwmSlice<6>::counter();
            const uint16_t b1 = PwmSlice<0>::counter();
            const uint16_t b2 = PwmSlice<0>::counter();
            const uint16_t a2 = PwmSlice<6>::counter();
            lags[k] = (static_cast<int32_t>(b1) - static_cast<int32_t>(a1) + static_cast<int32_t>(b2) - static_cast<int32_t>(a2)) / 2;
        }
        print(serial, "  slices 6 and 0 started together at divider 100: lag ", lags[0], ", ", lags[1], ", ", lags[2],
              " counts at 100, 200 and 300 ms", crlf);
        int32_t lo = lags[0];
        int32_t hi = lags[0];
        for (uint8_t k = 1; k < 3u; ++k) {
            lo = lags[k] < lo ? lags[k] : lo;
            hi = lags[k] > hi ? lags[k] : hi;
        }
        bench.verdict("two outputs started by the global enable keep a fixed phase relationship, within two counts, "
                      "unchanged over 300 ms",
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
    Pwm::stop(0xFF);
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
        print(serial, "  divider ", ladder[k].sixteenths() / 16u, " + ", ladder[k].sixteenths() % 16u, "/16: ", hz, " Hz counted, ",
              want, " computed", good ? "" : "  OUT", crlf);
        if (good) {
            ++ok;
        }
        Edges7::release();
    }
    bench.verdict("the divider ladder on the wire at TOP 199 - one, two and a half, ten, 255 and 15/16, 256 - "
                  "each frequency within one per cent of the arithmetic (the fractional divider averages)",
                  ok == 5u);
    all_off();
}

// =============================================================================
// h - the lamp
// =============================================================================
void th_lamp() {
    if (!wire_6b_7b() || !wire_0b_4b()) {
        return;
    }
    Pwm::stop(0xFF);
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
    spin_us(300);
    const uint32_t g_pm = measure_duty_pm<Level7>();
    const uint32_t b_pm = measure_duty_pm<Level4>();
    print(serial, "  RgbLamp{255, 128, 64} over three outputs: G (GP13) ", g_pm, " per mille, B (GP17) ", b_pm, " per mille, R's level ",
          R::duty(), crlf);
    bench.verdict("util/rgb_lamp.hpp over three PwmOutputs: 128 of 255 reads 50 %, 64 reads 25 %, 255 sets the full level",
                  within(g_pm, 502, 10) && within(b_pm, 251, 12) && R::duty() == 1000u);
    Lamp::off();
    spin_us(300);
    bench.verdict("... and off() is 0 % on both wires", measure_duty_pm<Level7>() == 0u && measure_duty_pm<Level4>() == 0u);
    Level7::release();
    Level4::release();
    all_off();
}

// =============================================================================
// y - polarity, edges against wraps, the settle (diagnostics, outside z)
// =============================================================================
void ty_probe() {
    using S6 = PwmSlice<6>;
    Pwm::stop(0xFF);
    // (1) The polarity: GP15 sampled through a period at 2 us a count.
    struct Case { const char* name; bool invert; uint16_t level; };
    const Case cases[] = {{"plain 250", false, 250}, {"inverted 250", true, 250}, {"plain 100", false, 100}, {"inverted 0", true, 0}};
    for (const Case& c : cases) {
        (void)Out6B::setup(PwmDivider{0, 0}, false, c.invert);
        Out6B::duty(c.level);
        Pin<15>::input(PinPull::none);
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
            if (h) { ++high; if (first_high_at == 0xFFFFu) { first_high_at = ctr; } }
            else if (first_low_at == 0xFFFFu) { first_low_at = ctr; }
            ++total;
            if (ctr >= 990u) { break; }
        }
        print(serial, "  ", c.name, ": CSR=", hex(S6::csr()), " CC=", hex(S6::cc()), " high ", high * 1000u / total,
              " per mille of ", total, " samples; first high at count ", first_high_at, ", first low at ", first_low_at, crlf);
    }
    Out6B::release();
    Pin<15>::release();
    // (1b) The level counter on the same signals, the count read twice.
    for (const Case& c : cases) {
        (void)Out6B::setup(PwmDivider{12, 8}, false, c.invert);   // 10 kHz
        Out6B::duty(c.level);
        (void)Level7::setup(level_div);
        spin_us(500);
        Level7::restart();
        Level7::run(true);
        spin_us(level_window_us);
        Level7::run(false);
        const uint16_t n1 = Level7::count();
        spin_us(10);
        const uint16_t n2 = Level7::count();
        print(serial, "  level counter on ", c.name, ": ", n1, " then ", n2, " of ", level_full, "; slice 7 CSR=", hex(PwmSlice<7>::csr()),
              " DIV=", hex(PwmSlice<7>::div()), " CTR=", PwmSlice<7>::counter(), crlf);
        Level7::release();
    }
    Out6B::release();
    // (2) Edges counted against wraps counted, 40 ms.
    const PwmDivider divs[] = {{1, 0}, {1, 0}, {2, 8}};
    const uint16_t tops[] = {199, 1249, 199};
    for (uint8_t k = 0; k < 3u; ++k) {
        Pwm::stop(0xFF);
        (void)S6::configure({.divider = divs[k], .top = tops[k]});
        S6::levels(0, static_cast<uint16_t>((tops[k] + 1u) / 2u));
        Pin<13>::function(PinFunction::pwm);
        (void)Edges7::setup();
        S6::counter(0);
        S6::clear_pending();
        clear_wraps();
        S6::interrupt(true);
        Edges7::restart();
        Pwm::start(S6::bit | PwmSlice<7>::bit);
        spin_us(40'000);
        Pwm::stop(S6::bit | PwmSlice<7>::bit);
        S6::interrupt(false);
        spin_us(10);
        print(serial, "  TOP ", tops[k], " div ", divs[k].sixteenths(), "/16: wraps ", wraps[6], ", edges ", Edges7::count(),
              " (", pwm_output_hz(SysClock::hz, {.divider = divs[k], .top = tops[k]}) * 40u / 1000u, " expected in 40 ms)", crlf);
        Edges7::release();
    }
    Pin<13>::release();
    // (3) One read at a growing delay after a fresh nudge, and after a
    // fresh start: stale on the first read, or stale for a while?
    using S = PwmSlice<3>;
    using T = PwmSlice<5>;
    Pwm::stop(0xFF);
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
        (void)S::retard_phase();   // net zero: the lag should read 2
        spin_us(d);
        const uint16_t a = S::counter();
        const uint16_t b = T::counter();
        print(serial, d, ":", static_cast<int32_t>(b) - static_cast<int32_t>(a), " ");
        spin_us(2000);
    }
    print(serial, crlf, "  two reads after a nudge, the second: ");
    for (uint8_t i = 0; i < 4u; ++i) {
        (void)S::advance_phase();
        (void)S::retard_phase();
        (void)S::counter();
        const uint16_t a = S::counter();
        const uint16_t b = T::counter();
        print(serial, static_cast<int32_t>(b) - static_cast<int32_t>(a), " ");
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
    print(serial, crlf, "  one read after a plain write to TOP (the same value), at +us: ");
    for (uint32_t d : delays) {
        S::top(0xFFFF);
        spin_us(d);
        const uint16_t a = S::counter();
        const uint16_t b = T::counter();
        print(serial, d, ":", static_cast<int32_t>(b) - static_cast<int32_t>(a), " ");
        spin_us(2000);
    }
    print(serial, crlf);
    Pwm::stop(0xFF);
    // (4) Letter g's own sequence with the wraps counted alongside.
    for (uint8_t k = 0; k < 2u; ++k) {
        Pwm::stop(0xFF);
        (void)PwmOutput<13, 199>::setup(k == 0u ? PwmDivider{1, 0} : PwmDivider{2, 8});
        PwmOutput<13, 199>::duty(100);
        (void)Edges7::setup();
        clear_wraps();
        S6::clear_pending();
        S6::interrupt(true);
        Edges7::restart();
        Edges7::run(true);
        spin_us(40'000);
        Edges7::run(false);
        S6::interrupt(false);
        const uint16_t e1 = Edges7::count();
        spin_us(10);
        const uint16_t e2 = Edges7::count();
        print(serial, "  g's sequence, divider ", k == 0u ? 1u : 2u, ": wraps ", wraps[6], ", edges ", e1, " then ", e2, crlf);
        Edges7::release();
        PwmOutput<13, 199>::release();
    }
    // (5) The level written mid-period, the pad sampled through.
    Pwm::stop(0xFF);
    (void)Out6B::setup(PwmDivider{0, 0});   // 2 us a count
    Out6B::duty(100);
    Pin<15>::input(PinPull::none);
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
    print(serial, "  level 100 -> 900 written at count ", at_write, ": the pad ", before ? "high" : "low", " before, ",
          after ? "high" : "low", " after; then rose at count ", rose_at, ", fell at ", fell_at, "; CC=", hex(S6::cc()), crlf);
    Out6B::release();
    Pin<15>::release();
    Pwm::stop(0xFF);
}

void banner() {
    print(serial, crlf, "test_rp2040_pwm - the RP2040 PWM (datasheet 4.5): slice 6 B on GP13 -> GP15 (slice 7 B), "
          "slice 0 B on GP17 -> GP9 (slice 4 B); clk_sys=", SysClock::hz, " Hz", crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void isr_uart0() { (void)Serial::isr(); }
extern "C" void isr_systick() { brio::Ticker::tick(); }
extern "C" void isr_pwm_wrap() {
    wrap_entries = wrap_entries + 1u;
    const uint8_t up = brio::Pwm::isr();
    for (uint8_t i = 0; i < 8u; ++i) {
        if ((up & (1u << i)) != 0u) {
            wraps[i] = wraps[i] + 1u;
        }
    }
}
extern "C" void isr_dma_0() {
    const uint8_t f = Stream::service();
    if ((f & Stream::flag_complete) != 0u) {
        (void)Stream::complete();
        stream_done = true;
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool timer_ok = brio::Timer::init(clock);
    const bool dma_ok = brio::Dma::init();
    const bool pwm_ok = brio::Pwm::reset();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::Nvic::enable(brio::Pwm::irq());
    brio::enable_interrupts();

    bench.letter('a', "the block, wireless", ta_block);
    bench.letter('b', "the wrap as a tick and as a DMA pace", tb_wrap);
    bench.letter('c', "the wire: frequency and duty", tc_wire);
    bench.letter('d', "phase-correct, inversion, the buffered level", td_modes);
    bench.letter('e', "the pair with its dead time", te_pair);
    bench.letter('f', "the second wire, lockstep", tf_second_wire);
    bench.letter('g', "the divider ladder", tg_ladder);
    bench.letter('h', "the lamp over three outputs", th_lamp);
    bench.letter('y', "polarity, edges against wraps, the settle (diagnostics, outside z)", ty_probe, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL125" : "FAILED", " timer=", timer_ok ? "1us" : "FAILED",
                    " dma=", dma_ok ? "released" : "FAILED", " pwm=", pwm_ok ? "released" : "FAILED", " tick=",
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
