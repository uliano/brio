// test_samc_tcc - the reference bench suite for samc/tcc.hpp.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the driver
// under it.
//
// NOTHING TO WIRE. The TCC is the richest timer of this family and the
// one whose whole point - complementary outputs, patterns and fault
// shutdown - is about what reaches a PIN, so the wireless toolkit the
// earlier suites built is what makes it measurable at all:
//   - a pad a peripheral DRIVES can be read back through PORT.IN with
//     the input buffer on (test_samc_tc), which turns five of TCC0's
//     eight outputs into instruments;
//   - a pad under an INPUT-ONLY function still follows its internal
//     pull (test_samc_eic), which is how a fault is raised and lowered
//     from inside the chip, through the EIC and an event channel;
//   - a timer counting another timer's EVENTS is a frequency meter with
//     no wire in it (test_samc_tc), which is what settles a PWM rate and
//     a dithered average;
//   - and a second TC, free-running at 3 MHz, is the stopwatch that
//     measures a dead time of a few hundred microseconds.
//
// The five TCC0 outputs this board can see, out of the device header:
//   PA08 function E = TCC0/WO0     PA09 function E = TCC0/WO1
//   PA22 function F = TCC0/WO4     PA12 function F = TCC0/WO6
//   PA16 function A = EIC EXTINT0  (the fault stimulus, not an output)
//
// What is exercised, letter by letter:
//   a  the block: three instances that are NOT copies of each other,
//      the pad map keyed by pad AND function, every refusal, and the
//      enable-protection split that lets WAVE change under a runner
//   b  the counter: 24 bits against TCC2's 16, READSYNC, the prescaler,
//      direction, stop/retrigger and one-shot
//   c  PWM: single-slope duty off the pads, frequency off a TC counting
//      overflow events, and the DUAL-SLOPE period arithmetic measured
//      rather than trusted
//   d  double buffering, and the three errata that live in it - 1.21.6
//      (clear the flag twice), 1.21.8 (LUPD does not protect PER while
//      counting down) and 1.21.10 (ALOCK is dead)
//   e  dithering: a fractional period, measured as an average frequency
//   f  the waveform extension: dead time measured in microseconds, the
//      output matrix, and a LIVE swap
//   g  pattern generation, buffered and unbuffered
//   h  recoverable faults: clamp, keep, restart, both halt modes and the
//      capture action, raised from a pin through the event system
//   i  non-recoverable faults: the counter stopped and the outputs
//      forced to a level the application chose
//   j  ramp operations: RAMP2's alternating cycles seen as duty, and the
//      index command holding one of them
//   k  capture (PPW) and counting on an event
//   l  host/client: TCC1's counter driven by TCC0's
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include "samc/clock.hpp"
#include "samc/eic.hpp"
#include "samc/evsys.hpp"
#include "samc/nvic.hpp"
#include "samc/pin.hpp"
#include "samc/sercom.hpp"
#include "samc/tc.hpp"
#include "samc/tcc.hpp"
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

TestBench<Serial, 16> bench;

using brio::crlf;
using brio::print;

// ---------------------------------------------------------------------------
// The pins, the timers and the clocks
// ---------------------------------------------------------------------------
using Wo0Pin = Pin<'A', 8>;
using Wo1Pin = Pin<'A', 9>;
using Wo4Pin = Pin<'A', 22>;
using Wo6Pin = Pin<'A', 12>;
using EicPad = Pin<'A', 16>;

using Wo0 = TccWo<Wo0Pin, PinFunction::e>;   // TCC0 / WO0
using Wo1 = TccWo<Wo1Pin, PinFunction::e>;   // TCC0 / WO1
using Wo4 = TccWo<Wo4Pin, PinFunction::f>;   // TCC0 / WO4
using Wo6 = TccWo<Wo6Pin, PinFunction::f>;   // TCC0 / WO6
using EicLine = ExtInt<EicPad>;              // EXTINT0

using Timer = Tcc<0>;
using Timer1 = Tcc<1>;
using Timer2c = Tcc<2>;
using EventCounter = Tc<0>;   // counts another timer's events
using Stopwatch = Tc<2>;      // 48 MHz / 16 = 3 MHz, one tick = 1/3 us

constexpr uint32_t sys_hz = SysClock::hz;

// Generator 0 is OSC48M itself, so every number computed against it is
// exact arithmetic and not a measurement of an oscillator.
constexpr uint8_t fast_gen = 0;

// A SLOW generator for the waveform-extension letter: the dead-time
// counter counts UNPRESCALED GCLK_TCC cycles and is only 8 bits wide, so
// at 48 MHz its whole range is 5.3 us - too short to see from software.
// 48 MHz / 240 = 200 kHz makes one dead-time step 5 us.
constexpr uint8_t slow_gen = 7;
constexpr uint16_t slow_div = 240;
constexpr uint32_t slow_hz = sys_hz / slow_div;   // 200000
using SlowGclk = Gclk<slow_gen>;

// The event fabric. Channel 0 carries the EIC line, channel 1 a timer's
// overflow. Their generic clock only matters for the synchronous paths;
// every channel here is asynchronous, as erratum 1.21.9 requires for
// every TCC user.
constexpr uint8_t ev_pin_channel = 0;
constexpr uint8_t ev_ovf_channel = 1;
constexpr uint8_t ev_gen = 6;
using EvGclk = Gclk<ev_gen>;

// The stopwatch: 48 MHz / 16 = 3 MHz, so one tick is 1/3 microsecond and
// a 16-bit counter spans 21.8 ms.
constexpr uint32_t stopwatch_hz = sys_hz / 16u;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void wait_ms(uint32_t ms) {
    const uint32_t deadline = Ticker::millis() + ms;
    while (static_cast<int32_t>(Ticker::millis() - deadline) < 0) {
    }
}

bool near(uint32_t got, uint32_t want, uint32_t band) {
    return got + band >= want && want + band >= got;
}

/// Sample a pad through PORT.IN many times and report how many reads
/// were high, in parts per thousand - the crude but honest way to read a
/// duty cycle with no instrument.
template <class P>
uint32_t duty_permille(uint32_t samples = 40'000UL) {
    uint32_t high = 0;
    for (uint32_t i = 0; i < samples; ++i) {
        if (P::read()) {
            ++high;
        }
    }
    return (high * 1000UL) / samples;
}

/// Both pads sampled in the same pass, so "were they ever high at the
/// same time" is a question about the SAME instants.
uint32_t both_high_count(uint32_t samples = 40'000UL) {
    uint32_t both = 0;
    for (uint32_t i = 0; i < samples; ++i) {
        if (Wo0Pin::read() && Wo4Pin::read()) {
            ++both;
        }
    }
    return both;
}

/// Bring the 3 MHz stopwatch up.
bool stopwatch_up() {
    return Stopwatch::init(fast_gen) &&
           Stopwatch::configure(TcConfig{.mode = TcMode::count16,
                                         .prescaler = TcPrescaler::div16}) &&
           Stopwatch::enable(true);
}

/**
 * The length, in stopwatch ticks, of the next window in which BOTH
 * complementary outputs are low - which is what a dead time IS. Zero
 * when no such window turns up inside the spin bound, which is the
 * honest answer for a dead time of zero.
 *
 * Both timestamps are taken AFTER their loop exits, so the cost of
 * reading the stopwatch (a READSYNC command and two synchronizations)
 * appears at both ends and cancels out of the difference.
 */
uint32_t both_low_ticks() {
    constexpr uint32_t limit = 300'000UL;
    uint32_t i = 0;
    while (!(Wo0Pin::read() || Wo4Pin::read())) {
        if (++i > limit) {
            return 0;
        }
    }
    i = 0;
    while (Wo0Pin::read() || Wo4Pin::read()) {
        if (++i > limit) {
            return 0;
        }
    }
    const uint16_t t0 = Stopwatch::count16();
    i = 0;
    while (!(Wo0Pin::read() || Wo4Pin::read())) {
        if (++i > limit) {
            return 0;
        }
    }
    const uint16_t t1 = Stopwatch::count16();
    return static_cast<uint16_t>(t1 - t0);
}

/// How often STATUS.IDX reads high over `ms` milliseconds, in parts per
/// thousand. A ramp cycle is a counter period, so the window has to span
/// many of them - a fixed number of reads spans whatever the console and
/// the synchronization happen to cost that day.
uint32_t ramp_index_permille(uint32_t ms, bool hold_each_read = false) {
    uint32_t high = 0;
    uint32_t total = 0;
    const uint32_t started = Ticker::millis();
    while (Ticker::millis() - started < ms) {
        if (hold_each_read) {
            (void)Tcc<0>::ramp_index_command(TccRampIndexCommand::hold);
        }
        if (Tcc<0>::ramp_index()) {
            ++high;
        }
        ++total;
    }
    return total != 0u ? (high * 1000ul) / total : 0u;
}

/// Bring the event fabric up with its own clock, and point the counting
/// TC at one generator on an asynchronous channel.
bool count_events_from(uint8_t generator) {
    Evsys::bus_clock(true);
    if (!EvGclk::configure(GclkConfig{.source = GclkSource::osculp32k}) ||
        !GclkChannel::connect(Evsys::gclk_id(ev_ovf_channel), ev_gen)) {
        return false;
    }
    if (!EventCounter::init(fast_gen) ||
        !EventCounter::configure(TcConfig{.mode = TcMode::count16}) ||
        !EventCounter::event_config(TcConfig{.mode = TcMode::count16},
                                    TcEventConfig{.action = TcEventAction::count,
                                                  .input_enable = true})) {
        return false;
    }
    if (!Evsys::connect(EventCounter::event_user, ev_ovf_channel,
                        EventChannelConfig{.generator = generator,
                                           .path = EventPath::asynchronous})) {
        return false;
    }
    return EventCounter::enable(true);
}

/// How many events arrived in `ms` milliseconds. The window is opened
/// and closed by two counter accesses with nothing printed between them.
uint32_t events_in(uint32_t ms) {
    (void)EventCounter::set_count16(0);
    wait_ms(ms);
    return EventCounter::count16();
}

void release_event_counter() {
    Evsys::disconnect(EventCounter::event_user);
    GclkChannel::disconnect(Evsys::gclk_id(ev_ovf_channel));
    EventCounter::release();
}

/// The EIC line, sensed on a LEVEL so its event output is a COPY of the
/// pad rather than a pulse - which is what a fault input wants. The pad
/// moves under its own internal pull, whose direction is the OUT bit.
bool fault_pin_up() {
    Evsys::bus_clock(true);
    if (!Eic::init() || !Eic::clock_select(EicClock::ulp32k)) {
        return false;
    }
    EicPad::input(PinPull::down);
    EicLine::claim(PinPull::down);
    return Eic::configure_line(EicLine::line,
                               EicLineConfig{.sense = EicSense::high,
                                             .event_out = true}) &&
           Eic::enable(true);
}

void fault_pin_down() {
    EicPad::clear();
    (void)Eic::enable(false);
    Eic::release();
    EicPad::configure({});
}

void raise_fault() {
    EicPad::set();
    wait_ms(2);
}
void clear_fault_input() {
    EicPad::clear();
    wait_ms(2);
}

/// Hand every output pad this suite uses back to PORT, quiet.
void release_pads() {
    Wo0::release();
    Wo1::release();
    Wo4::release();
    Wo6::release();
    Wo0Pin::configure({});
    Wo1Pin::configure({});
    Wo4Pin::configure({});
    Wo6Pin::configure({});
}

/// TCC0 as a plain single-slope PWM generator on the slow generator,
/// with WO0 and WO4 claimed: the shape letters f..j all start from.
///  period = (top + 1) / slow_hz.
bool slow_pwm_up(uint32_t top, uint32_t duty, const TccWaveExtConfig& wex = {},
                 const TccWaveConfig& wave = TccWaveConfig{
                     .waveform = TccWaveform::normal_pwm}) {
    if (!SlowGclk::configure(GclkConfig{.source = GclkSource::osc48m,
                                        .div = slow_div})) {
        return false;
    }
    if (!Timer::init(slow_gen)) {
        return false;
    }
    if (!Timer::configure(TccConfig{}) || !Timer::wave_extension(wex) ||
        !Timer::wave(wave)) {
        return false;
    }
    if (!Timer::set_period(top) || !Timer::set_cc(0, duty) ||
        !Timer::set_cc(1, duty)) {
        return false;
    }
    return Timer::enable(true);
}

// =============================================================================
// a - the block: three instances that are not copies of each other
// =============================================================================
void ta_block() {
    print(serial, "  instances: TCC0 ", Timer::counter_bits, " bit, ",
          Timer::cc_count, " CC, ", Timer::wo_count, " WO, EXT ",
          Timer::extension_code, "; TCC1 ", Timer1::counter_bits, "/",
          Timer1::cc_count, "/", Timer1::wo_count, "/", Timer1::extension_code,
          "; TCC2 ", Timer2c::counter_bits, "/", Timer2c::cc_count, "/",
          Timer2c::wo_count, "/", Timer2c::extension_code, crlf);

    bench.verdict("three instances, and NOT copies of each other - 24/24/16 "
                  "bits, 4/2/2 channels, 8/4/2 outputs, all of it the device "
                  "header's own TCCn_* constants",
                  tcc_count() == 3u && Timer::counter_bits == 24u &&
                      Timer2c::counter_bits == 16u && Timer::cc_count == 4u &&
                      Timer1::cc_count == 2u && Timer::wo_count == 8u &&
                      Timer2c::wo_count == 2u);
    bench.verdict("the counter MAX follows the width: 0xFFFFFF against 0xFFFF",
                  Timer::max_count == 0xFFFFFFul &&
                      Timer2c::max_count == 0xFFFFul);

    print(serial, "  GCLK channels: TCC0=", Timer::gclk_id, " TCC1=",
          Timer1::gclk_id, " TCC2=", Timer2c::gclk_id, crlf);
    bench.verdict("TCC0 and TCC1 SHARE a generic clock channel (36.5.3), so "
                  "neither can run at a rate the other does not",
                  Timer::gclk_id == Timer1::gclk_id &&
                      Timer::gclk_id != Timer2c::gclk_id);
    bench.verdict("TCC0 is the pair HOST and TCC1 the CLIENT - the one "
                  "instance allowed to set CTRLA.MSYNC (36.6.4)",
                  Timer::is_pair_host && Timer1::is_pair_client &&
                      !Timer2c::is_pair_host && !Timer2c::is_pair_client);

    // THE FIVE EXTENSION UNITS, each from its own header constant, and
    // TCCn_EXT cross-checked against them.
    bench.verdict("TCC0 has all five extension units, TCC1 only pattern and "
                  "dithering, TCC2 none",
                  Timer::has_dead_time && Timer::has_output_matrix &&
                      Timer::has_swap && Timer::has_pattern &&
                      Timer::has_dithering && !Timer1::has_dead_time &&
                      Timer1::has_pattern && Timer1::has_dithering &&
                      !Timer2c::has_pattern && !Timer2c::has_dithering);
    const uint8_t ext0 = static_cast<uint8_t>(
        (Timer::has_output_matrix ? 1u : 0u) | (Timer::has_dead_time ? 2u : 0u) |
        (Timer::has_swap ? 4u : 0u) | (Timer::has_pattern ? 8u : 0u) |
        (Timer::has_dithering ? 16u : 0u));
    bench.verdict("and TCCn_EXT is exactly those five bits - OTMX, DTI, SWAP, "
                  "PG, DITHERING from bit 0 up",
                  ext0 == Timer::extension_code && Timer1::extension_code == 24u &&
                      Timer2c::extension_code == 0u);
    bench.verdict("TCC0 has four dead-time slices, each driving WO[x] and "
                  "WO[x+4]; the other two have none",
                  Timer::slice_count == 4u && Timer1::slice_count == 0u);

    // THE PAD MAP, keyed by pad AND function - the fact that separates
    // it from the TC's.
    print(serial, "  PA08: function E -> TCC", Wo0::timer, "/WO", Wo0::output,
          ", function F -> TCC", TccWo<Wo0Pin, PinFunction::f>::timer, "/WO",
          TccWo<Wo0Pin, PinFunction::f>::output, crlf);
    bench.verdict("ONE PAD, TWO OUTPUTS OF TWO DIFFERENT INSTANCES: PA08 is "
                  "TCC0/WO0 under function E and TCC1/WO2 under function F",
                  Wo0::timer == 0u && Wo0::output == 0u &&
                      TccWo<Wo0Pin, PinFunction::f>::timer == 1u &&
                      TccWo<Wo0Pin, PinFunction::f>::output == 2u);
    bench.verdict("the four outputs this suite reads are TCC0's WO0, WO1, WO4 "
                  "and WO6",
                  Wo1::timer == 0u && Wo1::output == 1u && Wo4::timer == 0u &&
                      Wo4::output == 4u && Wo6::timer == 0u && Wo6::output == 6u);
    bench.verdict("and a pad with no TCC output on a function has none",
                  !tcc_wo_exists<'A', 22, 'e'> && !tcc_wo_exists<'B', 23, 'f'>);

    // The EVSYS and DMAC vocabularies, read from the header rather than
    // counted out: the generator codes are NOT evenly spaced.
    print(serial, "  EVSYS gens: TCC0 OVF ", Timer::overflow_generator, " TRG ",
          Timer::retrigger_generator, " CNT ", Timer::count_generator, " MC0 ",
          Timer::match_generator(0), "; TCC1 OVF ", Timer1::overflow_generator,
          "; TCC2 OVF ", Timer2c::overflow_generator, crlf);
    print(serial, "  EVSYS users: TCC0 EV0 ", Timer::event_user(0), " MC0 ",
          Timer::match_user(0), "; DMAC TCC0 OVF trigger ",
          Timer::dma_trigger_overflow, crlf);
    bench.verdict("the generator codes are NOT evenly spaced - TCC0 spends "
                  "seven of them and the other two five each, which is why "
                  "they are read from the header and not computed",
                  Timer1::overflow_generator - Timer::overflow_generator == 7 &&
                      Timer2c::overflow_generator - Timer1::overflow_generator == 5);
    bench.verdict("and a recoverable fault's user IS its channel's event input "
                  "(36.6.3.5)",
                  Timer::fault_user(TccFault::a) == Timer::match_user(0) &&
                      Timer::fault_user(TccFault::b) == Timer::match_user(1));

    // THE REFUSALS.
    bench.verdict("ERRATUM 1.21.10 AS A COMPILE-TIME REFUSAL: ALOCK is not "
                  "functional on any revision and has no workaround, so the "
                  "driver never writes it",
                  !tcc_config_valid(0, TccConfig{.auto_lock = true}));
    bench.verdict("dithering is refused on the instance without the unit",
                  tcc_config_valid(0, TccConfig{.resolution = TccResolution::dither64}) &&
                      !tcc_config_valid(2, TccConfig{.resolution = TccResolution::dither64}));
    bench.verdict("MSYNC is refused anywhere but on the pair client",
                  tcc_config_valid(1, TccConfig{.host_sync = true}) &&
                      !tcc_config_valid(0, TccConfig{.host_sync = true}));
    bench.verdict("dead time and the output matrix are refused on TCC1",
                  !tcc_wave_ext_valid(1, TccWaveExtConfig{.dead_time_enable = 1}) &&
                      !tcc_wave_ext_valid(
                          1, TccWaveExtConfig{
                                 .output_matrix = TccOutputMatrix::broadcast_cc0}));
    bench.verdict("pattern generation is refused on TCC2",
                  tcc_pattern_valid(0, 0xFF, 0x0F) && !tcc_pattern_valid(2, 1, 1));
    bench.verdict("RAMP2C is refused - 36.8.17 makes it a variant-L mode, "
                  "which also puts erratum 1.21.11 out of reach here",
                  !tcc_wave_valid(0, TccWaveConfig{.ramp = TccRamp::ramp2_critical}));
    bench.verdict("and a capture action with no capture channel is refused",
                  !tcc_event_config_valid(
                      0, TccConfig{},
                      TccEventConfig{.action1 = TccEvent1Action::period_pulse_width}));

    // ENABLE-PROTECTION, and the register that is NOT under it.
    bench.verdict("the timer comes up disabled", Timer::init(fast_gen) &&
                                                     !Timer::enabled());
    bench.verdict("and configures", Timer::configure(TccConfig{
                      .prescaler = TccPrescaler::div1024}));
    bench.verdict("it enables", Timer::enable(true));
    bench.verdict("NOW CTRLA, WEXCTRL, DRVCTRL, FCTRLA and EVCTRL are refused "
                  "- all five are enable-protected (36.6.2.1)",
                  !Timer::configure(TccConfig{}) &&
                      !Timer::wave_extension(TccWaveExtConfig{}) &&
                      !Timer::drive(TccDriveConfig{}) &&
                      !Timer::fault(TccFault::a, TccFaultConfig{}) &&
                      !Timer::event_config(TccConfig{}, TccEventConfig{}));
    bench.verdict("BUT WAVE IS TAKEN, running - it is write-synchronized and "
                  "NOT enable-protected, which is what lets a polarity or a "
                  "swap change under a live waveform",
                  Timer::wave(TccWaveConfig{.waveform = TccWaveform::normal_pwm,
                                            .polarity = 0x5}) &&
                      Timer::waveform() == TccWaveform::normal_pwm &&
                      (Timer::wave_reg() & TCC_WAVE_POL_Msk) == TCC_WAVE_POL(0x5));

    bench.verdict("a software reset clears it all and leaves the TCC disabled",
                  Timer::reset() && !Timer::enabled() &&
                      Timer::wave_reg() == 0u && Timer::ctrla() == 0u);
    Timer::release();
}

// =============================================================================
// b - the counter
// =============================================================================
void tb_counter() {
    bench.verdict("TCC0 comes up", Timer::init(fast_gen));

    // THE 24-BIT COUNTER. At 48 MHz / 16 = 3 MHz, 200 ms is 600000
    // ticks: far past what 16 bits hold and far short of 24 bits' own
    // 16.7 million. A delta, with nothing printed inside the window.
    bench.verdict("it configures at /16 and runs",
                  Timer::configure(TccConfig{.prescaler = TccPrescaler::div16}) &&
                      Timer::set_period(Timer::max_count) &&
                      Timer::set_count(0) && Timer::enable(true));
    const uint32_t before = Timer::count();
    wait_ms(200);
    const uint32_t wide = Timer::count() - before;
    const uint32_t expect_wide = sys_hz / 16u / 5u;
    print(serial, "  200 ms at 3 MHz: COUNT advanced ", wide, " (exact ",
          expect_wide, "), which is ", wide >> 16, " times past 65535", crlf);
    bench.verdict("THE COUNTER IS GENUINELY 24 BITS WIDE - it advanced far "
                  "past what 16 bits could hold without wrapping",
                  wide > 0x10000ul);
    bench.verdict("and it counts at the rate it was given, within 1 %",
                  near(wide, expect_wide, expect_wide / 100u + 100u));

    // Reading COUNT is a COMMAND (36.6.7).
    const uint32_t a = Timer::count();
    const uint32_t b = Timer::count();
    bench.verdict("two READSYNC'd reads of a running counter differ", a != b);
    bench.verdict("and the raw accessor returns what the last command "
                  "fetched - which is why it is spelled raw",
                  Timer::count_raw() == b);

    // THE PRESCALER RATIO, measured rather than assumed.
    uint32_t delta[2] = {0, 0};
    const TccPrescaler scales[2] = {TccPrescaler::div1024, TccPrescaler::div256};
    bool ok = true;
    constexpr uint32_t window_ms = 200;
    for (uint8_t i = 0; i < 2u; ++i) {
        ok = ok && Timer::enable(false) &&
             Timer::configure(TccConfig{.prescaler = scales[i]}) &&
             Timer::set_count(0) && Timer::enable(true);
        const uint32_t t0 = Timer::count();
        wait_ms(window_ms);
        delta[i] = Timer::count() - t0;
    }
    print(serial, "  ", window_ms, " ms at /1024: ", delta[0], " ticks, at "
          "/256: ", delta[1], " ticks, ratio x1000 = ",
          delta[0] != 0u ? (delta[1] * 1000ul) / delta[0] : 0ul, crlf);
    bench.verdict("the counter configures and runs at two prescaler settings",
                  ok);
    bench.verdict("and /256 counts FOUR TIMES as fast",
                  delta[0] != 0u && near((delta[1] * 1000ul) / delta[0], 4000u, 40u));

    // Direction, live (36.6.2.4).
    (void)Timer::set_count(30'000);
    bench.verdict("the direction reverses under a running counter",
                  Timer::count_down(true) && Timer::counting_down());
    const uint32_t down_a = Timer::count();
    wait_ms(20);
    bench.verdict("and the counter now counts DOWN", Timer::count() < down_a);
    bench.verdict("back up again",
                  Timer::count_down(false) && !Timer::counting_down());

    // STOP and RETRIGGER.
    bench.verdict("the STOP command stops it", Timer::stop() && Timer::stopped());
    const uint32_t frozen = Timer::count();
    wait_ms(20);
    bench.verdict("and the counter is frozen", Timer::count() == frozen);
    bench.verdict("RETRIGGER restarts it, and STATUS.STOP clears itself",
                  Timer::retrigger() && !Timer::stopped());
    wait_ms(20);
    bench.verdict("the counter moves again", Timer::count() != frozen);

    // PER IS TOP IN EVERY MODE BUT MFRQ - the register the TC only has
    // in 8-bit mode.
    bench.verdict("a short period is set and the counter is bounded by it",
                  Timer::enable(false) &&
                      Timer::configure(TccConfig{.prescaler = TccPrescaler::div1024}) &&
                      Timer::set_period(999) && Timer::set_count(0) &&
                      Timer::enable(true));
    // The counter cycle at 46875 Hz with PER = 999 is 21 ms, so the
    // window has to be several of those - a fixed number of READSYNC'd
    // reads would only ever sample a slice of one.
    uint32_t highest = 0;
    {
        const uint32_t started = Ticker::millis();
        while (Ticker::millis() - started < 100u) {
            const uint32_t c = Timer::count();
            if (c > highest) {
                highest = c;
            }
        }
    }
    print(serial, "  with PER = 999 the highest COUNT seen over 100 ms - "
          "nearly five counter cycles - was ", highest, crlf);
    bench.verdict("PER IS TOP - the counter climbs to it and never past it, in "
                  "a mode where the TC would have had to spend a compare "
                  "channel to get a period at all",
                  highest <= 999u && highest > 900u);
    bench.verdict("and the overflow flag stands", (Timer::flags() &
                                                   Timer::overflow_flag) != 0u);

    // ONE-SHOT.
    Timer::clear_flags(0xFFFFFFFFul);
    bench.verdict("one-shot is armed live", Timer::one_shot(true) &&
                                                Timer::retrigger());
    wait_ms(50);
    bench.verdict("and the counter stops itself at the first overflow, with no "
                  "software in the path",
                  Timer::stopped() &&
                      (Timer::flags() & Timer::overflow_flag) != 0u);
    (void)Timer::one_shot(false);
    Timer::release();

    // TCC2's SIXTEEN BITS, seen from the other side: 36.8.15 says "the
    // excess bits are read zero", and a write is the way to ask.
    bench.verdict("TCC2 comes up", Timer2c::init(fast_gen) &&
                                       Timer2c::configure(TccConfig{}));
    (void)Timer2c::set_count(0x123456ul);
    const uint32_t narrow_read = Timer2c::count();
    (void)Timer::init(fast_gen);
    (void)Timer::configure(TccConfig{});
    (void)Timer::set_count(0x123456ul);
    const uint32_t wide_read = Timer::count();
    print(serial, "  0x123456 written to COUNT reads back as ", wide_read,
          " on TCC0 and ", narrow_read, " on TCC2", crlf);
    bench.verdict("TCC2'S COUNTER IS SIXTEEN BITS AND THE EXCESS READS ZERO "
                  "(36.8.15), where TCC0 keeps all 24",
                  wide_read == 0x123456ul && narrow_read == 0x3456ul);
    Timer::release();
    Timer2c::release();
}

// =============================================================================
// c - PWM: duty off the pads, frequency off a TC, and the dual-slope
//     period arithmetic MEASURED
// =============================================================================
constexpr uint32_t pwm_top = 199;
constexpr uint32_t pwm_hz = sys_hz / 256u / (pwm_top + 1u);   // 937.5 -> 937

void tc_pwm() {
    bench.verdict("TCC0 comes up and drives WO0",
                  Timer::init(fast_gen) &&
                      Timer::configure(TccConfig{.prescaler = TccPrescaler::div256}) &&
                      Timer::wave(TccWaveConfig{.waveform = TccWaveform::normal_pwm}) &&
                      Timer::set_period(pwm_top) && Timer::set_cc(0, 0) &&
                      Timer::enable(true));
    Wo0::claim();

    struct Case {
        uint32_t duty;
        uint32_t expect;
        const char* what;
    };
    const Case cases[] = {
        {0, 0, "zero duty is a pad held low"},
        {pwm_top + 1u, 1000, "a duty at the period is a pad held high"},
        {(pwm_top + 1u) / 2u, 500, "half duty is a pad high half the time"},
        {(pwm_top + 1u) / 4u, 250, "and a quarter is a quarter"},
    };
    for (const auto& c : cases) {
        (void)Timer::set_cc_buffer(0, c.duty);
        wait_ms(20);
        const uint32_t measured = duty_permille<Wo0Pin>();
        print(serial, "  CC0 ", c.duty, "/", pwm_top + 1u, ": PA08 high ",
              measured, " per mille (expected ~", c.expect, ")", crlf);
        bench.verdict("", c.what,
                      c.expect == 0
                          ? measured == 0u
                          : (c.expect >= 1000u ? measured >= 990u
                                               : near(measured, c.expect, 40u)));
    }

    // THE FREQUENCY, with no wire and no scope: a TC counts TCC0's
    // overflow events.
    (void)Timer::set_cc_buffer(0, (pwm_top + 1u) / 2u);
    (void)Timer::enable(false);
    bench.verdict("TCC0's overflow becomes an event",
                  Timer::event_config(TccConfig{.prescaler = TccPrescaler::div256},
                                      TccEventConfig{.overflow_out = true}) &&
                      Timer::enable(true));
    bench.verdict("and a TC counts it on an ASYNCHRONOUS channel - the only "
                  "path erratum 1.21.9 leaves open to a TCC",
                  count_events_from(Timer::overflow_generator));

    const uint32_t single = events_in(1000);
    print(serial, "  single-slope: ", single, " overflows in 1 s (48 MHz / 256 "
          "/ ", pwm_top + 1u, " = ", pwm_hz, " Hz)", crlf);
    bench.verdict("THE SINGLE-SLOPE PWM FREQUENCY IS GCLK / (N x (PER+1)), one "
                  "timer counting another's events, within 1 %",
                  near(single, pwm_hz, pwm_hz / 100u + 2u));

    // THE DUAL-SLOPE ARITHMETIC, measured rather than trusted - the AVR
    // TCD campaign found the datasheet's printed dual-slope formula off
    // by one, and this is the same question asked of this silicon.
    // 36.6.2.5.6 prints fPWM_DS = fGCLK / (2 N PER); the natural
    // alternative, if the counter visited TOP and ZERO both, would be
    // 2 (PER + 1). Over two seconds they are eight counts apart.
    bench.verdict("the waveform switches to dual-slope, live",
                  Timer::wave(TccWaveConfig{
                      .waveform = TccWaveform::dual_slope_bottom}));
    const uint32_t dual = events_in(2000);
    // Both predictions for a two-second window, in whole overflows.
    const uint32_t expect_2per = (2ul * sys_hz) / (2ul * 256ul * pwm_top);
    const uint32_t expect_2per_plus =
        (2ul * sys_hz) / (2ul * 256ul * (pwm_top + 1u));
    print(serial, "  dual-slope: ", dual, " overflows in 2 s; the chapter's "
          "2 x N x PER predicts ", expect_2per, ", a 2 x N x (PER+1) period "
          "would give ", expect_2per_plus, crlf);
    bench.verdict("THE CHAPTER'S DUAL-SLOPE FORMULA IS EXACT ON THIS SILICON: "
                  "the period is 2 x PER counter ticks and NOT 2 x (PER+1) - "
                  "the AVR TCD's printed formula was off by exactly that one, "
                  "so it was worth two seconds of counting to find out",
                  near(dual, expect_2per, 3u) && !near(dual, expect_2per_plus, 3u));

    release_event_counter();
    release_pads();
    Timer::release();
}

// =============================================================================
// d - double buffering, and the three errata that live in it
// =============================================================================
void td_buffers() {
    // 48 MHz / 64 = 750 kHz, PER 999 -> a 750 Hz waveform: slow enough for
    // the counter to be read and fast enough for the pad sampler to see
    // twenty-five periods in one pass.
    constexpr uint32_t buf_top = 999;
    constexpr uint32_t low_duty = 250;
    constexpr uint32_t high_duty = 750;

    bench.verdict("the 3 MHz stopwatch comes up", stopwatch_up());
    bench.verdict("TCC0 comes up driving WO0 at 25 %",
                  Timer::init(fast_gen) &&
                      Timer::configure(TccConfig{.prescaler = TccPrescaler::div64}) &&
                      Timer::wave(TccWaveConfig{.waveform = TccWaveform::normal_pwm}) &&
                      Timer::set_period(buf_top) && Timer::set_cc(0, low_duty) &&
                      Timer::enable(true));
    Wo0::claim();
    wait_ms(20);
    const uint32_t duty_start = duty_permille<Wo0Pin>();

    // THE PAD IS THE WITNESS, not the register: a compare value only
    // means something once the waveform generator is using it.
    Timer::clear_flags(0xFFFFFFFFul);
    const bool wrote = Timer::set_cc_buffer(0, high_duty);
    const bool valid_after_write = Timer::cc_buffer_valid(0);
    const bool busy_after_write = Timer::cc_sync_busy(0);
    wait_ms(20);
    const uint32_t duty_taken = duty_permille<Wo0Pin>();
    print(serial, "  LUPD clear: duty ", duty_start, " -> ", duty_taken,
          " per mille after writing CCBUF0; CCBUFV0 ",
          valid_after_write ? "1" : "0", ", SYNCBUSY.CC0 ",
          busy_after_write ? "1" : "0", " on the way out", crlf);
    bench.verdict("a buffered write reaches the WAVEFORM at the next UPDATE "
                  "condition - which is what keeps a live duty change "
                  "glitch-free",
                  wrote && near(duty_start, low_duty, 40u) &&
                      near(duty_taken, high_duty, 40u));
    bench.verdict("and it returns AT ONCE with STATUS.CCBUFV0 standing to say "
                  "a value is waiting - the buffered setters in this driver do "
                  "not wait, because SYNCBUSY.CCx does not clear until the "
                  "update TAKES the buffer",
                  valid_after_write && busy_after_write);

    // HOW LONG THE WAIT WOULD HAVE BEEN, measured: the whole reason the
    // setter does not take it.
    (void)Timer::update();
    const uint16_t base0 = Stopwatch::count16();
    const uint16_t base1 = Stopwatch::count16();
    const uint16_t w0 = Stopwatch::count16();
    (void)Timer::set_cc_buffer(0, high_duty);
    const bool waited = Timer::sync_wait(TCC_SYNCBUSY_CC0_Msk);
    const uint16_t w1 = Stopwatch::count16();
    const uint32_t wait_ticks =
        static_cast<uint16_t>(w1 - w0) - static_cast<uint16_t>(base1 - base0);
    print(serial, "  waiting SYNCBUSY.CC0 out after a buffered write took ",
          wait_ticks, " stopwatch ticks (", wait_ticks / 3u, " us) and "
          "returned ", waited ? "true" : "false", "; the PWM period is 1333 us",
          crlf);
    bench.verdict("SYNCBUSY.CCx STANDS UNTIL THE UPDATE CONSUMES THE BUFFER, "
                  "not merely for the clock-domain crossing: the wait is a "
                  "fraction of a whole PWM period, and no chapter says so",
                  waited && wait_ticks > 100u && wait_ticks < 4200u);

    // A WRITE ISSUED INSIDE THAT WINDOW IS DISCARDED, which is why the
    // setter refuses rather than overwriting.
    bench.verdict("the update lock is taken", Timer::lock_update(true) &&
                                                  Timer::update_locked());
    const bool first = Timer::set_cc_buffer(0, low_duty);
    const bool second = Timer::set_cc_buffer(0, 900);
    const uint32_t buf_reads = Timer::cc_buffer(0);
    print(serial, "  with LUPD set: the first buffered write returned ",
          first ? "true" : "false", ", a second one ",
          second ? "true" : "false", ", and CCBUF0 holds ", buf_reads, crlf);
    bench.verdict("A SECOND BUFFERED WRITE INSIDE THE SAME WINDOW IS "
                  "DISCARDED BY THE SILICON - so the setter refuses it "
                  "instead, and false means 'the last value has not been "
                  "taken yet'",
                  first && !second && buf_reads == low_duty);

    wait_ms(20);
    const uint32_t duty_locked = duty_permille<Wo0Pin>();
    const uint32_t cc_reads_locked = Timer::cc(0);
    print(serial, "  LUPD set: duty stays ", duty_locked,
          " per mille, while CC0 READS ", cc_reads_locked, crlf);
    bench.verdict("with CTRLB.LUPD set the WAVEFORM does not change, however "
                  "many update conditions go by",
                  near(duty_locked, high_duty, 40u));
    bench.verdict("A READ OF CCx WHILE A BUFFERED WRITE IS PENDING RETURNS THE "
                  "BUFFERED VALUE, NOT THE ONE THE WAVEFORM IS USING - the pad "
                  "and the register disagree, and 36.8.19 never warns of it",
                  cc_reads_locked == low_duty &&
                      near(duty_locked, high_duty, 40u));

    (void)Timer::update();
    wait_ms(20);
    const uint32_t duty_forced = duty_permille<Wo0Pin>();
    print(serial, "  after the UPDATE command: duty ", duty_forced,
          " per mille", crlf);
    bench.verdict("and the UPDATE COMMAND takes the buffer anyway - 36.6.2.6's "
                  "note, that a software update acts independently of LUPD",
                  near(duty_forced, low_duty, 40u));

    // ERRATUM 1.21.6: clearing a buffer-valid flag releases SYNCBUSY too
    // early, and the workaround is to clear it twice.
    (void)Timer::set_cc_buffer(0, 123);
    const bool set_before = Timer::cc_buffer_valid(0);
    const bool cleared = Timer::clear_buffer_valid(TCC_STATUS_CCBUFV0_Msk) &&
                         !Timer::cc_buffer_valid(0);
    bench.verdict("a buffer-valid flag can be cleared by hand while the update "
                  "is locked, and the DOUBLE clear erratum 1.21.6 asks for "
                  "leaves it clear",
                  set_before && cleared);
    print(serial, "  1.21.6's failure mode - SYNCBUSY released before the "
          "register behind the flag is restored - is not staged here: the "
          "workaround is unconditional and costs one store.", crlf);
    (void)Timer::lock_update(false);
    Wo0::release();
    Timer::release();
    Stopwatch::release();

    // ERRATUM 1.21.8: "in down-counting mode the Lock Update bit does not
    // protect against a PER register update from the PERBUF register".
    // The witness is the OVERFLOW RATE - a period of 1000 gives 750
    // overflows a second at this prescaler and one of 556 gives 1350 -
    // because the PER REGISTER READ cannot be a witness here: a pending
    // buffered write makes it read the buffer (measured above).
    struct Leg {
        bool down;
        TccWaveform wave;
        const char* what;
        uint32_t overflows;
        uint32_t per_reads;
    };
    Leg legs[3] = {
        {false, TccWaveform::normal_pwm, "NPWM counting up", 0, 0},
        {true, TccWaveform::normal_pwm, "NPWM counting down", 0, 0},
        {true, TccWaveform::normal_frequency, "NFRQ counting down", 0, 0},
    };
    bool leg_ok = true;
    for (auto& leg : legs) {
        const TccConfig cfg{.prescaler = TccPrescaler::div64,
                            .count_down = leg.down,
                            .lock_update = true};
        leg_ok = leg_ok && Timer::init(fast_gen) && Timer::configure(cfg) &&
                 Timer::event_config(cfg, TccEventConfig{.overflow_out = true}) &&
                 Timer::wave(TccWaveConfig{.waveform = leg.wave}) &&
                 Timer::set_period(buf_top) && Timer::set_cc(0, low_duty) &&
                 Timer::set_count(leg.down ? buf_top : 0u) && Timer::enable(true) &&
                 count_events_from(Timer::overflow_generator);
        (void)Timer::set_period_buffer(555);
        leg.overflows = events_in(500) * 2u;   // per second
        leg.per_reads = Timer::period();
        release_event_counter();
        Timer::release();
    }
    for (const auto& leg : legs) {
        print(serial, "  ", leg.what, ": ", leg.overflows,
              " overflows/s (PER 999 gives 750, PER 555 gives 1350); the PER "
              "register reads ", leg.per_reads, crlf);
    }
    bench.verdict("THE UPDATE LOCK PROTECTS THE PERIOD WHILE COUNTING UP",
                  leg_ok && near(legs[0].overflows, 750u, 40u));
    bench.verdict("AND IT PROTECTS IT COUNTING DOWN TOO, in both NPWM and "
                  "NFRQ: ERRATUM 1.21.8 DID NOT REPRODUCE on this silicon "
                  "with the waveform as the witness, three modes tried",
                  near(legs[1].overflows, 750u, 40u) &&
                      near(legs[2].overflows, 750u, 40u));
    bench.verdict("what DOES happen in all three is the register read of fact "
                  "8 - PER reads back the buffered 555 while the counter is "
                  "still using 1000, which is exactly the trap a reader "
                  "checking PER would fall into and call 1.21.8",
                  legs[0].per_reads == 555u && legs[1].per_reads == 555u &&
                      legs[2].per_reads == 555u);

    // ERRATUM 1.21.10: ALOCK does nothing. The driver refuses the bit,
    // so the measurement writes CTRLA by hand - which is the only way to
    // ask the silicon whether the refusal is deserved.
    bench.verdict("TCC0 comes back up with a 500-tick period",
                  Timer::init(fast_gen) &&
                      Timer::configure(TccConfig{.prescaler = TccPrescaler::div1024}) &&
                      Timer::wave(TccWaveConfig{.waveform = TccWaveform::normal_pwm}) &&
                      Timer::set_period(499));
    Timer::regs().TCC_CTRLA |= TCC_CTRLA_ALOCK_Msk;
    const bool alock_written =
        (Timer::regs().TCC_CTRLA & TCC_CTRLA_ALOCK_Msk) != 0u;
    (void)Timer::lock_update(false);
    (void)Timer::set_count(0);
    (void)Timer::enable(true);
    wait_ms(100);   // nine overflows at 46875 Hz / 500
    const bool lupd_after = Timer::update_locked();
    print(serial, "  CTRLA.ALOCK written back as ", alock_written ? "1" : "0",
          "; CTRLB.LUPD after ~9 overflows: ", lupd_after ? "1" : "0",
          " (36.8.1 says ALOCK sets LUPD on each overflow)", crlf);
    bench.verdict("ERRATUM 1.21.10 CONFIRMED: the ALOCK bit WRITES, and does "
                  "nothing - LUPD does not follow an overflow, which is the "
                  "one thing 36.8.1 says it should do",
                  alock_written && !lupd_after);

    release_pads();
    Timer::release();
}

// =============================================================================
// e - dithering: a fractional period, measured as an average frequency
// =============================================================================
void te_dither() {
    // 48 MHz / 1024 = 46875 Hz. With DITH6 the frame is 64 PWM cycles
    // and PER's low six bits are the number of extra clocks to spread
    // over it, so the AVERAGE period is (PER + DITHERCY/64) ticks.
    constexpr uint32_t per_value = 99;
    constexpr uint32_t tick_hz = sys_hz / 1024u;

    bench.verdict("TCC0 comes up with DITH6 selected",
                  Timer::init(fast_gen) &&
                      Timer::configure(TccConfig{
                          .prescaler = TccPrescaler::div1024,
                          .resolution = TccResolution::dither64}) &&
                      Timer::wave(TccWaveConfig{.waveform = TccWaveform::normal_pwm}));
    bench.verdict("the packed period keeps the value in the high bits and the "
                  "extra cycles in the low six",
                  tcc_dither(per_value, 32, TccResolution::dither64) ==
                      ((per_value << 6) | 32u));
    bench.verdict("it runs, and its overflows reach a counting TC",
                  Timer::set_period(tcc_dither(per_value, 0,
                                               TccResolution::dither64)) &&
                      Timer::set_cc(0, tcc_dither(per_value / 2u, 0,
                                                  TccResolution::dither64)) &&
                      Timer::event_config(
                          TccConfig{.prescaler = TccPrescaler::div1024,
                                    .resolution = TccResolution::dither64},
                          TccEventConfig{.overflow_out = true}) &&
                      Timer::enable(true) &&
                      count_events_from(Timer::overflow_generator));

    // Three dithering settings, each measured over two seconds. The
    // predictions are 2 s x 46875 / (100 + d/64), which for d = 0, 32
    // and 63 are 937.5, 932.8 and 928.4 Hz - well apart at two seconds.
    const uint8_t extras[3] = {0, 32, 63};
    uint32_t counted[3] = {0, 0, 0};
    uint32_t predicted[3] = {0, 0, 0};
    for (uint8_t i = 0; i < 3u; ++i) {
        (void)Timer::set_period_buffer(
            tcc_dither(per_value, extras[i], TccResolution::dither64));
        wait_ms(50);
        counted[i] = events_in(2000);
        // 2 s x tick_hz x 64 / (64 (PER+1) + DITHERCY), in whole ticks.
        predicted[i] = (2ul * tick_hz * 64ul) /
                       (64ul * (per_value + 1u) + extras[i]);
    }
    for (uint8_t i = 0; i < 3u; ++i) {
        print(serial, "  DITHERCY ", extras[i], "/64: ", counted[i],
              " overflows in 2 s (predicted ", predicted[i], ")", crlf);
    }
    bench.verdict("DITHERING BUYS A FRACTIONAL PERIOD: the average frequency "
                  "follows PER + DITHERCY/64 to better than half a percent, "
                  "with an integer counter underneath",
                  near(counted[0], predicted[0], predicted[0] / 200u + 3u) &&
                      near(counted[1], predicted[1], predicted[1] / 200u + 3u) &&
                      near(counted[2], predicted[2], predicted[2] / 200u + 3u));
    bench.verdict("and the three settings are strictly ordered - more extra "
                  "cycles is a longer period is a lower frequency",
                  counted[0] > counted[1] && counted[1] > counted[2]);
    print(serial, "  erratum 1.21.7 (dithering plus external RETRIGGER events "
          "stretches or shrinks pulses) is a caller obligation, not tested: "
          "nothing here retriggers a dithering TCC.", crlf);

    release_event_counter();
    Timer::release();
}

// =============================================================================
// f - the waveform extension: dead time, the output matrix, and a live swap
// =============================================================================
constexpr uint32_t slow_top = 1999;   // 200 kHz / 2000 = 100 Hz
// The dead-time letter needs a period long enough to hold two windows of
// up to 255 GCLK_TCC cycles, which is why it runs at 100 Hz. Every other
// letter that reads a duty off a pad wants the OPPOSITE - the sampler
// takes about 32 ms for 40000 reads, so a 100 Hz waveform gives it three
// periods and a 1 kHz one gives it thirty. Hence two periods, and the
// sample count raised where the slow one is unavoidable.
constexpr uint32_t pad_top = 199;     // 200 kHz / 200 = 1 kHz
constexpr uint32_t slow_samples = 400'000UL;

void tf_extension() {
    bench.verdict("the 3 MHz stopwatch comes up", stopwatch_up());
    bench.verdict("TCC0 comes up on a 200 kHz generator with dead-time slice 0 "
                  "enabled - so WO0 becomes the low side and WO4 the high side "
                  "of one complementary pair (36.6.3.7)",
                  slow_pwm_up(slow_top, (slow_top + 1u) / 2u,
                              TccWaveExtConfig{.dead_time_enable = 0x1,
                                               .dead_time_low = 60,
                                               .dead_time_high = 180}));
    Wo0::claim();
    Wo4::claim();
    wait_ms(20);

    const uint32_t d_low = duty_permille<Wo0Pin>(slow_samples);
    const uint32_t d_high = duty_permille<Wo4Pin>(slow_samples);
    const uint32_t both = both_high_count(slow_samples);
    // Each period spends DTLS + DTHS = 240 of its 2000 counter ticks with
    // both outputs off, so the two duties sum to 880 per mille and not to
    // 1000 - which is the dead time seen from the other side.
    print(serial, "  at 50 % duty: PA08 (low side) high ", d_low,
          " per mille, PA22 (high side) high ", d_high, " (sum ",
          d_low + d_high, ", predicted 880); both high in ", both, " of ",
          slow_samples, " samples", crlf);
    bench.verdict("THE PAIR IS COMPLEMENTARY AND THE TWO ARE NEVER HIGH "
                  "TOGETHER - which is the whole promise of the dead-time "
                  "unit; the 120 per mille missing from the sum is the dead "
                  "time itself",
                  both == 0u && d_low > 400u && d_high > 350u &&
                      near(d_low + d_high, 880u, 40u));

    // THE DEAD TIME ITSELF, in microseconds. Each period has two
    // both-low windows, DTLS long and DTHS long; eight samples catch
    // both, so the shortest and the longest are the two registers.
    uint32_t shortest = 0xFFFFFFFFul;
    uint32_t longest = 0;
    for (uint8_t i = 0; i < 8u; ++i) {
        const uint32_t t = both_low_ticks();
        if (t != 0u) {
            if (t < shortest) {
                shortest = t;
            }
            if (t > longest) {
                longest = t;
            }
        }
    }
    const uint32_t expect_lo = (60ul * stopwatch_hz) / slow_hz;    // 900 ticks
    const uint32_t expect_hi = (180ul * stopwatch_hz) / slow_hz;   // 2700 ticks
    print(serial, "  dead time measured: shortest window ", shortest,
          " stopwatch ticks, longest ", longest, " (DTLS 60 -> ", expect_lo,
          ", DTHS 180 -> ", expect_hi, "; one tick = 1/3 us)", crlf);
    bench.verdict("THE TWO DEAD TIMES ARE EXACTLY THE TWO REGISTERS, in "
                  "GCLK_TCC cycles: 60 cycles on the rising edge and 180 on "
                  "the falling one, measured 300 us and 900 us apart",
                  near(shortest, expect_lo, expect_lo / 20u + 10u) &&
                      near(longest, expect_hi, expect_hi / 20u + 10u));

    // AND THE UNIT THE CHAPTER CLAIMS: 36.8.7 says GCLK_TCC cycles, not
    // prescaled ones. A prescaler of 4 makes the PWM four times slower;
    // if the dead time moved with it, the chapter would be wrong.
    (void)Timer::enable(false);
    (void)Timer::configure(TccConfig{.prescaler = TccPrescaler::div4});
    (void)Timer::set_period(slow_top / 4u);
    (void)Timer::set_cc(0, (slow_top / 4u + 1u) / 2u);
    (void)Timer::enable(true);
    wait_ms(20);
    uint32_t p_short = 0xFFFFFFFFul;
    for (uint8_t i = 0; i < 8u; ++i) {
        const uint32_t t = both_low_ticks();
        if (t != 0u && t < p_short) {
            p_short = t;
        }
    }
    print(serial, "  with the counter prescaled by 4, the shortest window is "
          "still ", p_short, " ticks", crlf);
    bench.verdict("THE DEAD TIME IS COUNTED IN UNPRESCALED GCLK_TCC CYCLES, "
                  "exactly as 36.8.7 says: a prescaler that changes the PWM "
                  "rate fourfold does not move it at all",
                  near(p_short, expect_lo, expect_lo / 20u + 10u));

    // THE OUTPUT MATRIX. With OTMX = per-channel and four channels, WO0
    // carries CC0 and WO6 carries CC2; with OTMX = broadcast, both carry
    // CC0. Two pads are enough to see the difference.
    Wo0::release();
    Wo4::release();
    bench.verdict("TCC0 restarts at 1 kHz with the dead-time unit off and two "
                  "different compare values",
                  slow_pwm_up(pad_top, (pad_top + 1u) / 4u));
    (void)Timer::set_cc(2, (3ul * (pad_top + 1u)) / 4u);
    Wo0::claim();
    Wo6::claim();
    wait_ms(20);
    const uint32_t m0_default = duty_permille<Wo0Pin>();
    const uint32_t m6_default = duty_permille<Wo6Pin>();

    (void)Timer::enable(false);
    const bool matrix_set = Timer::wave_extension(TccWaveExtConfig{
        .output_matrix = TccOutputMatrix::broadcast_cc0});
    (void)Timer::enable(true);
    wait_ms(20);
    const uint32_t m0_broadcast = duty_permille<Wo0Pin>();
    const uint32_t m6_broadcast = duty_permille<Wo6Pin>();
    print(serial, "  OTMX per-channel: PA08 (WO0) ", m0_default, ", PA12 (WO6) ",
          m6_default, " per mille; OTMX broadcast: ", m0_broadcast, ", ",
          m6_broadcast, crlf);
    bench.verdict("the default matrix spreads the channels modulo their count "
                  "- WO0 shows CC0's 25 % and WO6 shows CC2's 75 %",
                  matrix_set && near(m0_default, 250u, 40u) &&
                      near(m6_default, 750u, 40u));
    bench.verdict("and OTMX = 0x2 PUTS CC0 ON EVERY OUTPUT, which is what a "
                  "stepper's commutation needs",
                  near(m0_broadcast, 250u, 40u) && near(m6_broadcast, 250u, 40u));

    // THE SWAP, applied LIVE - the demonstration that WAVE is
    // write-synchronized and not enable-protected.
    Wo6::release();
    Wo0::release();
    bench.verdict("TCC0 restarts as a complementary pair at 25 % duty",
                  slow_pwm_up(pad_top, (pad_top + 1u) / 4u,
                              TccWaveExtConfig{.dead_time_enable = 0x1,
                                               .dead_time_low = 20,
                                               .dead_time_high = 20}));
    Wo0::claim();
    Wo4::claim();
    wait_ms(20);
    const uint32_t s0_before = duty_permille<Wo0Pin>();
    const uint32_t s4_before = duty_permille<Wo4Pin>();
    const bool swapped = Timer::wave(TccWaveConfig{
        .waveform = TccWaveform::normal_pwm, .swap = 0x1});
    wait_ms(20);
    const uint32_t s0_after = duty_permille<Wo0Pin>();
    const uint32_t s4_after = duty_permille<Wo4Pin>();
    print(serial, "  before SWAP0: PA08 ", s0_before, ", PA22 ", s4_before,
          " per mille; after: ", s0_after, ", ", s4_after, crlf);
    bench.verdict("SWAP0 EXCHANGES THE LOW AND HIGH SIDES OF THE PAIR - and it "
                  "was written to a RUNNING timer, because WAVE is the one "
                  "configuration register this chapter does not "
                  "enable-protect",
                  swapped && s0_before < 400u && s4_before > 600u &&
                      s0_after > 600u && s4_after < 400u);

    release_pads();
    Timer::release();
    Stopwatch::release();
}

// =============================================================================
// g - pattern generation
// =============================================================================
void tg_pattern() {
    bench.verdict("TCC0 runs a 50 % waveform on WO0 and WO4",
                  slow_pwm_up(pad_top, (pad_top + 1u) / 2u));
    Wo0::claim();
    Wo4::claim();
    wait_ms(20);
    const uint32_t free0 = duty_permille<Wo0Pin>();
    const uint32_t free4 = duty_permille<Wo4Pin>();

    // PATT sits AFTER the swap stage, so it beats everything the
    // waveform generator produced.
    const bool patt = Timer::pattern(0x11, 0x01);
    wait_ms(20);
    const uint32_t patt0 = duty_permille<Wo0Pin>();
    const uint32_t patt4 = duty_permille<Wo4Pin>();
    print(serial, "  free-running: PA08 ", free0, ", PA22 ", free4,
          " per mille; with PGE = 0x11 and PGV = 0x01: ", patt0, ", ", patt4,
          crlf);
    bench.verdict("both outputs were toggling", near(free0, 500u, 60u) &&
                                                    near(free4, 500u, 60u));
    bench.verdict("PATTERN GENERATION OVERRIDES THE WAVEFORM: the two enabled "
                  "outputs are pinned to their PGV levels, one high and one "
                  "low, whatever the compare unit is doing",
                  patt && patt0 >= 990u && patt4 == 0u);

    // The BUFFERED pattern, which is what "synchronized bit pattern"
    // means: taken at the next update condition, not on the store.
    (void)Timer::lock_update(true);
    const bool buffered = Timer::pattern_buffer(0x11, 0x10);
    const bool buf_valid = Timer::pattern_buffer_valid();
    const bool patt_busy = Timer::pattern_sync_busy();
    wait_ms(20);
    const bool valid_later = Timer::pattern_buffer_valid();
    const uint32_t held0 = duty_permille<Wo0Pin>();
    (void)Timer::lock_update(false);
    wait_ms(20);
    const uint32_t taken0 = duty_permille<Wo0Pin>();
    const uint32_t taken4 = duty_permille<Wo4Pin>();
    print(serial, "  PATTBUF = 0x10 with LUPD set: PA08 still ", held0,
          "; after the lock is released: PA08 ", taken0, ", PA22 ", taken4, crlf);
    print(serial, "  STATUS.PATTBUFV right after the write ",
          buf_valid ? "1" : "0", " (SYNCBUSY.PATT ", patt_busy ? "1" : "0",
          "), and 20 ms later ", valid_later ? "1" : "0", crlf);
    bench.verdict("a buffered pattern is HELD while the update is locked, and "
                  "the pads keep the pattern already in force",
                  buffered && held0 >= 990u);
    bench.verdict("STATUS.PATTBUFV LAGS ITS OWN WRITE, where STATUS.CCBUFVx "
                  "does not: the pattern flag only appears once PATTBUF has "
                  "crossed into the counter's clock domain, so a caller that "
                  "reads it immediately reads zero",
                  !buf_valid && patt_busy && valid_later);
    bench.verdict("and lands at the next update condition, which is what makes "
                  "a commutation step atomic across the pins",
                  taken0 == 0u && taken4 >= 990u);

    bench.verdict("clearing PGE gives the outputs back to the waveform "
                  "generator",
                  Timer::pattern(0, 0));
    wait_ms(20);
    const uint32_t back0 = duty_permille<Wo0Pin>();
    bench.verdict("and they toggle again", near(back0, 500u, 60u));

    release_pads();
    Timer::release();
}

// =============================================================================
// h - recoverable faults, raised from a pin through the event system
// =============================================================================
//
// 36.6.3.5: the two recoverable faults ARE the first two channel event
// inputs, MCE0 and MCE1, and their event channels "must be configured as
// asynchronous". Erratum 1.21.9 requires the same of every TCC user, so
// there is only one legal shape and this is it: an EIC line sensed on a
// LEVEL - so the event is a COPY of the pad - across an asynchronous
// channel into TCC0's MC0 user.

bool fault_pwm_up(const TccFaultConfig& fa) {
    if (!SlowGclk::configure(GclkConfig{.source = GclkSource::osc48m,
                                        .div = slow_div})) {
        return false;
    }
    if (!Timer::init(slow_gen)) {
        return false;
    }
    const TccConfig cfg{.capture_enable = 0x4};   // CC2 for the capture action
    if (!Timer::configure(cfg) || !Timer::fault(TccFault::a, fa) ||
        !Timer::event_config(cfg,
                             TccEventConfig{.match_in = tcc_fault_input_bit(
                                                TccFault::a)})) {
        return false;
    }
    if (!Timer::wave(TccWaveConfig{.waveform = TccWaveform::normal_pwm}) ||
        !Timer::set_period(pad_top) ||
        !Timer::set_cc(0, (pad_top + 1u) / 2u)) {
        return false;
    }
    if (!Evsys::connect(Timer::fault_user(TccFault::a), ev_pin_channel,
                        EventChannelConfig{.generator = EicLine::event_generator,
                                           .path = EventPath::asynchronous})) {
        return false;
    }
    return Timer::enable(true);
}

void th_recoverable() {
    bench.verdict("the EIC turns PA16's level into an event", fault_pin_up());
    bench.verdict("TCC0 runs a 50 % waveform with recoverable fault A taking "
                  "that event",
                  fault_pwm_up(TccFaultConfig{.source = TccFaultSource::event}));
    Wo0::claim();
    wait_ms(20);

    const uint32_t clean = duty_permille<Wo0Pin>();
    Timer::clear_flags(0xFFFFFFFFul);
    raise_fault();
    const bool input_seen = Timer::fault_input(TccFault::a);
    const bool state_seen = Timer::fault_state(TccFault::a);
    const bool flag_seen = (Timer::flags() & Timer::fault_a_flag) != 0u;
    const uint32_t clamped = duty_permille<Wo0Pin>();
    clear_fault_input();
    wait_ms(20);
    const uint32_t released = duty_permille<Wo0Pin>();
    print(serial, "  PA08 duty: free ", clean, " per mille, with the fault "
          "held ", clamped, ", after release ", released, "; FAULTAIN ",
          input_seen ? "1" : "0", " FAULTA ", state_seen ? "1" : "0",
          " INTFLAG ", flag_seen ? "1" : "0", crlf);
    bench.verdict("the waveform was running", near(clean, 500u, 60u));
    bench.verdict("A PIN EDGE, THROUGH THE EIC AND AN ASYNCHRONOUS EVENT "
                  "CHANNEL, CLAMPS THE COMPARE CHANNEL'S OUTPUT - the input is "
                  "visible as a level, the state as a latch, and the interrupt "
                  "flag rises with it",
                  input_seen && state_seen && flag_seen && clamped == 0u);
    bench.verdict("and the clamp is released with the fault condition, which "
                  "is what makes it RECOVERABLE",
                  near(released, 500u, 60u));

    // IS EVCTRL.MCEIx REQUIRED? The fault input IS the channel's event
    // input, so it should be - and nothing in 36.6.3.5 says it in those
    // words. The register is enable-protected, so the question is asked
    // by rebuilding without the bit.
    Evsys::disconnect(Timer::fault_user(TccFault::a));
    Timer::release();
    (void)SlowGclk::configure(GclkConfig{.source = GclkSource::osc48m,
                                         .div = slow_div});
    (void)Timer::init(slow_gen);
    (void)Timer::configure(TccConfig{});
    (void)Timer::fault(TccFault::a, TccFaultConfig{.source = TccFaultSource::event});
    (void)Timer::event_config(TccConfig{}, TccEventConfig{.match_in = 0});
    (void)Timer::wave(TccWaveConfig{.waveform = TccWaveform::normal_pwm});
    (void)Timer::set_period(pad_top);
    (void)Timer::set_cc(0, (pad_top + 1u) / 2u);
    (void)Evsys::connect(Timer::fault_user(TccFault::a), ev_pin_channel,
                         EventChannelConfig{.generator = EicLine::event_generator,
                                            .path = EventPath::asynchronous});
    (void)Timer::enable(true);
    wait_ms(20);
    raise_fault();
    const bool without_mcei = Timer::fault_state(TccFault::a);
    const uint32_t without_duty = duty_permille<Wo0Pin>();
    clear_fault_input();
    print(serial, "  with EVCTRL.MCEI0 CLEAR the same stimulus gives FAULTA ",
          without_mcei ? "1" : "0", " and a duty of ", without_duty,
          " per mille", crlf);
    bench.verdict("EVCTRL.MCEI0 IS THE GATE ON A RECOVERABLE FAULT'S INPUT - "
                  "the fault input IS that channel's event input, and with the "
                  "bit clear the same event does nothing at all. 36.6.3.5 "
                  "never says it in those words",
                  !without_mcei && near(without_duty, 500u, 60u));

    // ERRATUM 1.21.9, asked directly: the same generator on a
    // SYNCHRONOUS channel.
    Evsys::disconnect(Timer::fault_user(TccFault::a));
    Timer::release();
    (void)Timer::init(slow_gen);
    {
        const TccConfig cfg{};
        (void)Timer::configure(cfg);
        (void)Timer::fault(TccFault::a,
                           TccFaultConfig{.source = TccFaultSource::event});
        (void)Timer::event_config(
            cfg, TccEventConfig{.match_in = tcc_fault_input_bit(TccFault::a)});
    }
    (void)Timer::wave(TccWaveConfig{.waveform = TccWaveform::normal_pwm});
    (void)Timer::set_period(pad_top);
    (void)Timer::set_cc(0, (pad_top + 1u) / 2u);
    const bool sync_connected =
        Evsys::connect(Timer::fault_user(TccFault::a), ev_pin_channel,
                       EventChannelConfig{.generator = EicLine::event_generator,
                                          .path = EventPath::synchronous,
                                          .edge = EventEdge::both});
    (void)Timer::enable(true);
    wait_ms(20);
    raise_fault();
    const bool sync_fault = Timer::fault_state(TccFault::a);
    const uint32_t sync_duty = duty_permille<Wo0Pin>();
    clear_fault_input();
    print(serial, "  the SAME generator on a SYNCHRONOUS channel (erratum "
          "1.21.9 forbids it): connect ", sync_connected ? "ok" : "refused",
          ", FAULTA ", sync_fault ? "1" : "0", ", duty ", sync_duty,
          " per mille", crlf);
    bench.verdict("ERRATUM 1.21.9 MEASURED, not taken on trust: the same pad, "
                  "the same generator and the same fault configuration on a "
                  "SYNCHRONOUS event channel leave the waveform untouched - "
                  "the asynchronous path is the only one a TCC hears",
                  sync_connected && !sync_fault && near(sync_duty, 500u, 60u));
    Evsys::disconnect(Timer::fault_user(TccFault::a));
    Evsys::release_channel(ev_pin_channel);
    Timer::release();

    // RESTART, and the two HALT modes.
    bench.verdict("the fault is rebuilt with the RESTART action",
                  fault_pwm_up(TccFaultConfig{.source = TccFaultSource::event,
                                              .restart = true}));
    wait_ms(20);
    Timer::clear_flags(0xFFFFFFFFul);
    raise_fault();
    const bool restart_flag = (Timer::flags() & Timer::retrigger_flag) != 0u;
    clear_fault_input();
    Evsys::disconnect(Timer::fault_user(TccFault::a));
    Timer::release();

    bench.verdict("and again without it", fault_pwm_up(TccFaultConfig{
                      .source = TccFaultSource::event}));
    wait_ms(20);
    Timer::clear_flags(0xFFFFFFFFul);
    raise_fault();
    const bool no_restart_flag = (Timer::flags() & Timer::retrigger_flag) != 0u;
    clear_fault_input();
    print(serial, "  INTFLAG.TRG after a fault: with RESTART ",
          restart_flag ? "1" : "0", ", without ", no_restart_flag ? "1" : "0",
          crlf);
    bench.verdict("THE RESTART ACTION IS A RETRIGGER: the fault raises "
                  "INTFLAG.TRG exactly when FCTRLA.RESTART is set, and not "
                  "otherwise",
                  restart_flag && !no_restart_flag);
    Evsys::disconnect(Timer::fault_user(TccFault::a));
    Timer::release();

    // HARDWARE HALT: the counter stops while the fault is present and
    // resumes by itself.
    bench.verdict("the fault is rebuilt with the HARDWARE halt action",
                  fault_pwm_up(TccFaultConfig{.source = TccFaultSource::event,
                                              .halt = TccFaultHalt::hardware}));
    // THE WITNESS IS THE OVERFLOW FLAG, not two reads of COUNT: at 1 kHz
    // the counter wraps every millisecond, so a halted counter shows no
    // overflow in thirty of them and a running one cannot help showing
    // one. (Two reads of COUNT are not a witness here at all - see the
    // note the software-halt leg prints.)
    wait_ms(20);
    raise_fault();
    Timer::clear_flags(0xFFFFFFFFul);
    wait_ms(30);
    const bool overflowed_halted =
        (Timer::flags() & Timer::overflow_flag) != 0u;
    // For the record, and as a warning: two READSYNC'd reads of a HALTED
    // counter are NOT a witness. READSYNC is a CTRLB command that has to
    // cross into a clock domain the halt has stopped, so it returns
    // whatever it last managed to fetch.
    const uint32_t halted_a = Timer::count();
    wait_ms(20);
    const uint32_t halted_b = Timer::count();
    clear_fault_input();
    Timer::clear_flags(0xFFFFFFFFul);
    wait_ms(30);
    const bool overflowed_free = (Timer::flags() & Timer::overflow_flag) != 0u;
    print(serial, "  hardware halt: overflows in 30 ms with the fault held: ",
          overflowed_halted ? "yes" : "none", "; after release: ",
          overflowed_free ? "yes" : "none", crlf);
    print(serial, "  (and two READSYNC'd reads of the HALTED counter gave ",
          halted_a, " then ", halted_b, " - which is why the overflow flag is "
          "the witness here and COUNT is not)", crlf);
    bench.verdict("THE HARDWARE HALT FREEZES THE COUNTER for as long as the "
                  "fault is present - thirty counter cycles go by with no "
                  "overflow at all - and lets it go the moment it is not",
                  !overflowed_halted && overflowed_free);
    Evsys::disconnect(Timer::fault_user(TccFault::a));
    Timer::release();

    // SOFTWARE HALT: the counter stays halted after the input goes, until
    // STATUS.FAULTA is cleared by software.
    bench.verdict("the fault is rebuilt with the SOFTWARE halt action and a "
                  "capture into CC2",
                  fault_pwm_up(TccFaultConfig{.source = TccFaultSource::event,
                                              .halt = TccFaultHalt::software,
                                              .capture = TccFaultCapture::capture,
                                              .capture_channel = 2}));
    wait_ms(20);
    (void)Timer::cc(2);
    raise_fault();
    const uint32_t captured = Timer::cc(2);
    clear_fault_input();
    Timer::clear_flags(0xFFFFFFFFul);
    wait_ms(30);
    const bool still_halted = (Timer::flags() & Timer::overflow_flag) == 0u;
    const bool state_stands = Timer::fault_state(TccFault::a);
    const bool cleared = Timer::clear_fault_state(TccFault::a);
    Timer::clear_flags(0xFFFFFFFFul);
    wait_ms(30);
    const bool sw_released = (Timer::flags() & Timer::overflow_flag) != 0u;
    print(serial, "  software halt: the fault captured COUNT = ", captured,
          " into CC2; after the input went low, overflows in 30 ms: ",
          still_halted ? "none" : "yes", ", STATUS.FAULTA ",
          state_stands ? "1" : "0", "; after the software clear: ",
          sw_released ? "yes" : "none", crlf);
    bench.verdict("THE CAPTURE ACTION TIMESTAMPS THE FAULT into the channel "
                  "FCTRLA.CHSEL names - a plain CAPT, which erratum 1.21.5 "
                  "does not touch",
                  captured > 0u && captured <= pad_top);
    bench.verdict("THE SOFTWARE HALT OUTLIVES THE FAULT: the counter stays "
                  "frozen after the input is gone, the fault STATE stands, "
                  "and only a write to STATUS.FAULTA releases it (36.8.14)",
                  still_halted && state_stands && cleared && sw_released);

    Evsys::disconnect(Timer::fault_user(TccFault::a));
    Evsys::release_channel(ev_pin_channel);
    release_pads();
    Timer::release();
    fault_pin_down();
}

// =============================================================================
// i - non-recoverable faults
// =============================================================================
//
// The other half of the fault system: the counter EVENT inputs TCE0 and
// TCE1 with EVACT = FAULT (36.6.3.6). Where a recoverable fault clamps
// one channel and lets go, this one stops the counter and forces EVERY
// enabled output to a level the application chose in DRVCTRL - which is
// the "instant and predictable shut down" the chapter opens with.

bool non_recoverable_up(uint8_t nre, uint8_t nrv) {
    if (!SlowGclk::configure(GclkConfig{.source = GclkSource::osc48m,
                                        .div = slow_div})) {
        return false;
    }
    if (!Timer::init(slow_gen)) {
        return false;
    }
    const TccConfig cfg{};
    if (!Timer::configure(cfg) ||
        !Timer::drive(TccDriveConfig{.fault_output_enable = nre,
                                     .fault_output_value = nrv}) ||
        !Timer::event_config(cfg, TccEventConfig{.action0 = TccEvent0Action::fault,
                                                 .input0_enable = true})) {
        return false;
    }
    if (!Timer::wave(TccWaveConfig{.waveform = TccWaveform::normal_pwm}) ||
        !Timer::set_period(pad_top) ||
        !Timer::set_cc(0, (pad_top + 1u) / 2u)) {
        return false;
    }
    if (!Evsys::connect(Timer::event_user(0), ev_pin_channel,
                        EventChannelConfig{.generator = EicLine::event_generator,
                                           .path = EventPath::asynchronous})) {
        return false;
    }
    return Timer::enable(true);
}

void ti_non_recoverable() {
    bench.verdict("the EIC turns PA16's level into an event", fault_pin_up());
    // WO0 and WO4 both driven under the fault: WO0 to 1, WO4 to 0.
    bench.verdict("TCC0 runs with EVACT0 = FAULT and DRVCTRL naming what the "
                  "outputs must become",
                  non_recoverable_up(0x11, 0x01));
    Wo0::claim();
    Wo4::claim();
    wait_ms(20);

    const uint32_t free0 = duty_permille<Wo0Pin>();
    const uint32_t free4 = duty_permille<Wo4Pin>();
    Timer::clear_flags(0xFFFFFFFFul);
    raise_fault();
    const bool input_seen = Timer::non_recoverable_input(0);
    const bool state_seen = Timer::non_recoverable_state(0);
    const bool flag_seen = (Timer::flags() & Timer::non_recoverable_flag(0)) != 0u;
    const uint32_t held0 = duty_permille<Wo0Pin>();
    const uint32_t held4 = duty_permille<Wo4Pin>();
    // The counter's own witness is the OVERFLOW FLAG: at 1 kHz a running
    // counter cannot go 30 ms without one, and a stopped one cannot
    // produce one. (Reading COUNT is not a witness while the counter is
    // stopped - letter h prints why.)
    Timer::clear_flags(Timer::overflow_flag);
    wait_ms(30);
    const bool overflowed_stopped =
        (Timer::flags() & Timer::overflow_flag) != 0u;
    print(serial, "  free: PA08 ", free0, ", PA22 ", free4, " per mille; under "
          "the fault: ", held0, ", ", held4, "; overflows in 30 ms: ",
          overflowed_stopped ? "yes" : "none", "; FAULT0IN ",
          input_seen ? "1" : "0", " FAULT0 ", state_seen ? "1" : "0",
          " INTFLAG ", flag_seen ? "1" : "0", crlf);
    bench.verdict("both outputs were toggling", near(free0, 500u, 60u) &&
                                                    near(free4, 500u, 60u));
    bench.verdict("A NON-RECOVERABLE FAULT FORCES EVERY ENABLED OUTPUT TO ITS "
                  "OWN NRV LEVEL - one high and one low, chosen per output - "
                  "and stops the counter dead while it does it",
                  input_seen && state_seen && flag_seen && held0 >= 990u &&
                      held4 == 0u && !overflowed_stopped);

    // The state is latched: dropping the input is not enough.
    clear_fault_input();
    wait_ms(20);
    const bool latched = Timer::non_recoverable_state(0);
    const uint32_t still0 = duty_permille<Wo0Pin>();
    const bool cleared = Timer::clear_non_recoverable(0);
    wait_ms(20);
    const uint32_t back0 = duty_permille<Wo0Pin>();
    const uint32_t back4 = duty_permille<Wo4Pin>();
    print(serial, "  after the input went low: FAULT0 still ",
          latched ? "1" : "0", ", PA08 ", still0, "; after the software clear: ",
          back0, " and PA22 ", back4, crlf);
    bench.verdict("THE STATE IS A LATCH, NOT A LEVEL: dropping the input "
                  "leaves the drivers shut down, exactly as a safety shutdown "
                  "should",
                  latched && still0 >= 990u);
    bench.verdict("and a write to STATUS.FAULT0 gives the outputs back to the "
                  "waveform generator",
                  cleared && near(back0, 500u, 60u) && near(back4, 500u, 60u));

    // The other polarity, so the verdict is two-sided rather than a
    // coincidence of one bit pattern.
    Evsys::disconnect(Timer::event_user(0));
    Wo0::release();
    Wo4::release();
    Timer::release();
    bench.verdict("the same fault with the opposite NRV",
                  non_recoverable_up(0x11, 0x10));
    Wo0::claim();
    Wo4::claim();
    wait_ms(20);
    raise_fault();
    const uint32_t inv0 = duty_permille<Wo0Pin>();
    const uint32_t inv4 = duty_permille<Wo4Pin>();
    clear_fault_input();
    (void)Timer::clear_non_recoverable(0);
    print(serial, "  with NRV = 0x10 the same fault gives PA08 ", inv0,
          ", PA22 ", inv4, " per mille", crlf);
    bench.verdict("THE LEVELS ARE THE APPLICATION'S, PER OUTPUT: swapping the "
                  "two NRV bits swaps the two pads",
                  inv0 == 0u && inv4 >= 990u);

    Evsys::disconnect(Timer::event_user(0));
    Evsys::release_channel(ev_pin_channel);
    release_pads();
    Timer::release();
    fault_pin_down();
}

// =============================================================================
// j - ramp operations
// =============================================================================
//
// 36.6.3.4: in RAMP2 two consecutive counter cycles are interleaved -
// "in cycle A, odd channel output is disabled, and in cycle B, even
// channel output is disabled". Two pads carrying CC0 and CC1 turn that
// sentence into a number: each output is live in half the cycles, so a
// 50 % duty becomes a 25 % duty measured over many cycles.

void tj_ramp() {
    bench.verdict("TCC0 runs CC0 on WO0 and CC1 on WO1, both at 50 %, in "
                  "RAMP1",
                  slow_pwm_up(pad_top, (pad_top + 1u) / 2u, TccWaveExtConfig{},
                              TccWaveConfig{.waveform = TccWaveform::normal_pwm,
                                            .ramp = TccRamp::ramp1}));
    Wo0::claim();
    Wo1::claim();
    wait_ms(20);
    const uint32_t r1_wo0 = duty_permille<Wo0Pin>();
    const uint32_t r1_wo1 = duty_permille<Wo1Pin>();
    const uint32_t idx_ramp1 = ramp_index_permille(50);

    // RAMP2, applied live - WAVE again.
    const bool ramped = Timer::wave(TccWaveConfig{
        .waveform = TccWaveform::normal_pwm, .ramp = TccRamp::ramp2});
    wait_ms(20);
    const uint32_t r2_wo0 = duty_permille<Wo0Pin>();
    const uint32_t r2_wo1 = duty_permille<Wo1Pin>();
    const uint32_t idx_ramp2 = ramp_index_permille(50);
    print(serial, "  RAMP1: PA08 ", r1_wo0, ", PA09 ", r1_wo1,
          " per mille, IDX high ", idx_ramp1, " per mille of a 50 ms window; "
          "RAMP2: ", r2_wo0, ", ", r2_wo1, ", IDX high ", idx_ramp2, crlf);
    bench.verdict("in RAMP1 both channels are live every cycle, and STATUS.IDX "
                  "always reads zero (36.8.14)",
                  near(r1_wo0, 500u, 60u) && near(r1_wo1, 500u, 60u) &&
                      idx_ramp1 == 0u);
    bench.verdict("RAMP2 INTERLEAVES TWO CYCLES: the ramp index toggles, and "
                  "each output is live in only half of them - a 50 % compare "
                  "measures 25 % on the pad",
                  ramped && idx_ramp2 > 300u && idx_ramp2 < 700u &&
                      near(r2_wo0, 250u, 50u) && near(r2_wo1, 250u, 50u));

    // THE INDEX COMMAND: HOLD keeps the next cycle the same as the
    // current one, so the index stops toggling. It is taken at the next
    // update condition and cleared there - so it has to be re-issued to
    // hold for more than one cycle, which is what this loop does.
    const uint32_t idx_held = ramp_index_permille(50, true);
    (void)Timer::ramp_index_command(TccRampIndexCommand::off);
    wait_ms(20);
    const uint32_t idx_free = ramp_index_permille(50);
    print(serial, "  with IDXCMD = HOLD re-issued at every read, IDX high ",
          idx_held, " per mille; with the command cancelled through CTRLBCLR, ",
          idx_free, crlf);
    bench.verdict("THE RAMP INDEX COMMAND HOLDS THE CYCLE: a repeated HOLD "
                  "pins the index to one value for fifty milliseconds",
                  idx_held < 50u || idx_held > 950u);
    bench.verdict("AND A COMMAND IS CANCELLED THROUGH CTRLBCLR, NOT BY WRITING "
                  "ZERO TO CTRLBSET - 'writing zero to this bit group has no "
                  "effect' on BOTH halves of CTRLB, which is the opposite of "
                  "how DIR, LUPD and ONESHOT behave in the same register",
                  idx_free > 300u && idx_free < 700u);

    release_pads();
    Timer::release();
}

// =============================================================================
// k - capture, and counting on an event
// =============================================================================
void tk_capture() {
    bench.verdict("the EIC turns PA16's level into an event", fault_pin_up());

    // PPW capture: period into CC0, pulse width into CC1, both fed by
    // the TCE1 event input. At 48 MHz / 1024 = 46875 Hz one tick is
    // 21.3 us, so a 20 ms / 30 ms wave is 938 and 2344 ticks.
    constexpr uint32_t capture_hz = sys_hz / 1024u;
    const TccConfig cfg{.prescaler = TccPrescaler::div1024, .capture_enable = 0x3};
    bench.verdict("TCC0 comes up as a period and pulse-width meter",
                  Timer::init(fast_gen) && Timer::configure(cfg) &&
                      Timer::event_config(
                          cfg, TccEventConfig{
                                   .action1 = TccEvent1Action::period_pulse_width,
                                   .input1_enable = true}) &&
                      Timer::enable(true));
    bench.verdict("and listens to the EIC line on an asynchronous channel",
                  Evsys::connect(Timer::event_user(1), ev_pin_channel,
                                 EventChannelConfig{
                                     .generator = EicLine::event_generator,
                                     .path = EventPath::asynchronous}));

    uint32_t period = 0;
    uint32_t width = 0;
    Timer::clear_flags(0xFFFFFFFFul);
    for (uint8_t i = 0; i < 4u; ++i) {
        EicPad::set();
        wait_ms(20);
        EicPad::clear();
        wait_ms(30);
        period = Timer::cc(0);
        width = Timer::cc(1);
    }
    const bool clean = (Timer::flags() & Timer::error_flag) == 0u;
    const uint32_t expect_period = (capture_hz * 50u) / 1000u;
    const uint32_t expect_width = (capture_hz * 20u) / 1000u;
    print(serial, "  captured period ", period, " ticks (expected ~",
          expect_period, "), width ", width, " ticks (expected ~", expect_width,
          "); INTFLAG.ERR ", clean ? "0" : "1", crlf);
    bench.verdict("PPW PUTS THE PERIOD IN CC0 - a pin level, through the EIC "
                  "and an asynchronous channel, into a capture register",
                  near(period, expect_period, expect_period / 20u + 4u));
    bench.verdict("and the pulse width in CC1",
                  near(width, expect_width, expect_width / 20u + 4u));

    // Unread captures are dropped and raise INTFLAG.ERR (36.6.2.7).
    Timer::clear_flags(0xFFFFFFFFul);
    for (uint8_t i = 0; i < 4u; ++i) {
        EicPad::set();
        wait_ms(20);
        EicPad::clear();
        wait_ms(30);
    }
    const bool overran = (Timer::flags() & Timer::error_flag) != 0u;
    bench.verdict("CAPTURES LEFT UNREAD ARE DROPPED and raise INTFLAG.ERR, "
                  "while a reader that drains CCx keeps up with no error at "
                  "all - reading CCx IS the acknowledgement (36.8.13)",
                  clean && overran);

    Evsys::disconnect(Timer::event_user(1));
    Evsys::release_channel(ev_pin_channel);
    Timer::release();
    fault_pin_down();

    // COUNTING ON AN EVENT (EVACT0 = COUNTEV): TCC0 counts TCC2's
    // overflows, which is the same wireless frequency meter the TC
    // suite built, with the two roles exchanged.
    constexpr uint32_t src_top = 4999;                     // 46875 / 5000
    constexpr uint32_t src_hz = sys_hz / 1024u / (src_top + 1u);   // 9 Hz
    bench.verdict("TCC2 free-runs and puts its overflow on the event system",
                  Timer2c::init(fast_gen) &&
                      Timer2c::configure(TccConfig{.prescaler = TccPrescaler::div1024}) &&
                      Timer2c::event_config(
                          TccConfig{.prescaler = TccPrescaler::div1024},
                          TccEventConfig{.overflow_out = true}) &&
                      Timer2c::wave(TccWaveConfig{}) &&
                      Timer2c::set_period(src_top) && Timer2c::enable(true));
    Evsys::bus_clock(true);
    bench.verdict("TCC0 is set up to COUNT events rather than clock ticks",
                  Timer::init(fast_gen) && Timer::configure(TccConfig{}) &&
                      Timer::event_config(
                          TccConfig{},
                          TccEventConfig{.action0 = TccEvent0Action::count,
                                         .input0_enable = true}) &&
                      Timer::set_period(Timer::max_count) &&
                      Evsys::connect(Timer::event_user(0), ev_ovf_channel,
                                     EventChannelConfig{
                                         .generator = Timer2c::overflow_generator,
                                         .path = EventPath::asynchronous}) &&
                      Timer::enable(true));
    (void)Timer::set_count(0);
    wait_ms(2000);
    const uint32_t counted = Timer::count();
    print(serial, "  TCC2 overflows counted by TCC0 in 2 s: ", counted,
          " (48 MHz / 1024 / ", src_top + 1u, " = ", src_hz, " Hz)", crlf);
    bench.verdict("A TCC COUNTS ANOTHER TIMER'S EVENTS, with the prescaler "
                  "bypassed (36.6.2.3) - the counter used as a counter",
                  near(counted, 2u * src_hz, 2u));

    Evsys::disconnect(Timer::event_user(0));
    Evsys::release_channel(ev_ovf_channel);
    Timer::release();
    Timer2c::release();
}

// =============================================================================
// l - host and client: TCC1's counter driven by TCC0's
// =============================================================================
void tl_host_client() {
    // 36.6.4 is three sentences long and says exactly this much: two
    // instances sharing a generic clock can be linked, and "the Client
    // TCC instance will synchronize the CC CHANNELS to the Host counter".
    // Not the counter - the channels. So the witness is a MATCH RATE.
    //
    // The host runs at 48 MHz / 256 / 200 = 937 cycles a second and the
    // client, left to itself, at 48 MHz / 1024 / 1000 = 47. Each has a
    // compare value below both periods, so each matches once per cycle,
    // and counting the client's MC0 events says whose cycle it is on.
    constexpr uint32_t host_top = 199;
    constexpr uint32_t client_top = 999;
    constexpr uint32_t host_hz = sys_hz / 256u / (host_top + 1u);      // 937
    constexpr uint32_t client_hz = sys_hz / 1024u / (client_top + 1u); // 46
    const TccConfig host_cfg{.prescaler = TccPrescaler::div256};
    const TccConfig free_cfg{.prescaler = TccPrescaler::div1024};
    const TccConfig linked_cfg{.prescaler = TccPrescaler::div1024,
                               .host_sync = true};

    bench.verdict("both halves come up on the generic clock channel they "
                  "share - which is the precondition 36.6.4 opens with",
                  Timer::init(fast_gen) && Timer1::init(fast_gen) &&
                      Timer::gclk_id == Timer1::gclk_id);
    bench.verdict("the host runs at 937 cycles a second",
                  Timer::configure(host_cfg) &&
                      Timer::wave(TccWaveConfig{.waveform = TccWaveform::normal_pwm}) &&
                      Timer::set_period(host_top) && Timer::set_cc(0, 100) &&
                      Timer::enable(true));

    // First WITHOUT the link.
    bench.verdict("and the client, unlinked, at 46",
                  Timer1::configure(free_cfg) &&
                      Timer1::event_config(free_cfg,
                                           TccEventConfig{.match_out = 0x1,
                                                          .overflow_out = true}) &&
                      Timer1::wave(TccWaveConfig{.waveform = TccWaveform::normal_pwm}) &&
                      Timer1::set_period(client_top) && Timer1::set_cc(0, 100) &&
                      Timer1::enable(true));
    bench.verdict("a TC counts the client's channel-0 matches",
                  count_events_from(Timer1::match_generator(0)));
    const uint32_t free_matches = events_in(1000);
    const bool free_slave = Timer1::is_client();
    release_event_counter();
    (void)count_events_from(Timer1::overflow_generator);
    const uint32_t free_overflows = events_in(1000);
    release_event_counter();

    // Then WITH it.
    bench.verdict("TCC1 takes CTRLA.MSYNC - the bit only a pair client may "
                  "set (36.8.1)",
                  Timer1::enable(false) && Timer1::configure(linked_cfg) &&
                      Timer1::event_config(linked_cfg,
                                           TccEventConfig{.match_out = 0x1,
                                                          .overflow_out = true}) &&
                      Timer1::wave(TccWaveConfig{.waveform = TccWaveform::normal_pwm}) &&
                      Timer1::set_period(client_top) && Timer1::set_cc(0, 100) &&
                      Timer1::enable(true));
    const bool linked_slave = Timer1::is_client();
    bench.verdict("the counting TC listens again",
                  count_events_from(Timer1::match_generator(0)));
    const uint32_t linked_matches = events_in(1000);
    release_event_counter();
    (void)count_events_from(Timer1::overflow_generator);
    const uint32_t linked_overflows = events_in(1000);
    release_event_counter();

    print(serial, "  unlinked: ", free_matches, " client matches and ",
          free_overflows, " client overflows per second (its own cycle is ",
          client_hz, " Hz), STATUS.SLAVE ", free_slave ? "1" : "0", crlf);
    print(serial, "  linked:   ", linked_matches, " client matches and ",
          linked_overflows, " client overflows per second (the host's cycle is ",
          host_hz, " Hz), STATUS.SLAVE ", linked_slave ? "1" : "0", crlf);

    bench.verdict("unlinked, the client matches on its own slow cycle",
                  !free_slave && near(free_matches, client_hz, 3u));
    bench.verdict("STATUS.SLAVE FOLLOWS CTRLA.MSYNC (36.8.14)",
                  linked_slave && !free_slave);
    bench.verdict("LINKED, THE CLIENT'S COMPARE CHANNELS ARE ON THE HOST'S "
                  "COUNTER: the same CC0 now matches 937 times a second "
                  "instead of 46, which is what 36.6.4's 'more synchronized CC "
                  "channels' buys",
                  near(linked_matches, host_hz, host_hz / 50u + 2u));
    bench.verdict("AND THE CLIENT'S OWN COUNTER IS UNTOUCHED - MSYNC moves the "
                  "CHANNELS, not COUNT, which is exactly what 36.6.4 says and "
                  "nothing more; a reader expecting the two COUNT registers to "
                  "track would find they never do",
                  near(free_overflows, client_hz, 3u) &&
                      near(linked_overflows, client_hz, 3u));

    Timer1::release();
    Timer::release();
}

void banner() {
    print(serial, crlf,
          "test_samc_tcc - SAMC21J18A TCC (ch. 36): three unequal timers, "
          "dead time, patterns, ramps and the two fault systems, wireless, "
          "clk=", SysClock::hz, " Hz", crlf);
    bench.menu();
}

} // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    bench.letter('a', "the block: three unequal instances and every refusal",
                 ta_block);
    bench.letter('b', "the counter: 24 bits, the prescaler, one-shot", tb_counter);
    bench.letter('c', "PWM: duty off the pads, frequency off a TC, dual slope",
                 tc_pwm);
    bench.letter('d', "double buffering, and the three errata inside it",
                 td_buffers);
    bench.letter('e', "dithering: a fractional period, measured", te_dither);
    bench.letter('f', "the waveform extension: dead time, matrix, live swap",
                 tf_extension);
    bench.letter('g', "pattern generation, buffered and unbuffered", tg_pattern);
    bench.letter('h', "recoverable faults: clamp, restart, halt, capture",
                 th_recoverable);
    bench.letter('i', "non-recoverable faults: stop and force the outputs",
                 ti_non_recoverable);
    bench.letter('j', "ramp operations: two interleaved cycles", tj_ramp);
    bench.letter('k', "capture (PPW) and counting on an event", tk_capture);
    bench.letter('l', "host and client: TCC1's counter driven by TCC0's",
                 tl_host_client);

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
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        bench.prompt();
    }
}
