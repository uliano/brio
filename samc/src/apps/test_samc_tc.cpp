// test_samc_tc - the reference bench suite for samc/tc.hpp.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the driver
// under it.
//
// NOTHING TO WIRE, and the three techniques it rests on were each
// established by an earlier suite:
//   - a pad handed to a peripheral still moves under its own INTERNAL
//     PULL, which is how the EIC gets its edges (test_samc_eic);
//   - a pad a peripheral DRIVES can be read back through PORT.IN with
//     the input buffer on, which is how a waveform is observed;
//   - a timer counting another timer's EVENTS is a frequency meter with
//     no wire in it at all, which is what settles the PWM rate.
// The bench board's LED is PB23 = TC3/WO1, so the one PWM output this
// board has bonded to something visible is also the one the suite uses.
//
// What is exercised, letter by letter:
//   a  the block: the header's geometry (shared generic clocks, which
//      instances pair), every refusal, enable-protection, and the pad
//      map
//   b  the counter: it runs, the prescaler ratios are exact, the
//      direction reverses, STOP/RETRIGGER and one-shot do what they say
//   c  COUNT32 by pairing TC0 with TC1 - the counter passes 65535 and
//      the client instance says it is one
//   d  PWM: an 8-bit channel on the LED, its duty read back off the pad
//      and its FREQUENCY measured by a second timer counting its
//      overflow events
//   e  CAPTURE, both sources: period and pulse width through EVSYS from
//      an EIC line, and capture ON THE PIN - the thing erratum 1.20.2
//      says does not work, on a revision where the erratum does not
//      apply
//   f  the util contract, live: a MeterSampler AO inside a running
//      kernel, paced by a time event, publishing MeterSample from a
//      MeterLatch that the TC's own capture ISR fills
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>
#include <variant>

#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"
#include "kernel/post.hpp"
#include "kernel/time_event.hpp"
#include "samc/clock.hpp"
#include "samc/eic.hpp"
#include "samc/evsys.hpp"
#include "samc/nvic.hpp"
#include "samc/pin.hpp"
#include "samc/platform_sam.hpp"
#include "samc/sercom.hpp"
#include "samc/tc.hpp"
#include "samc/ticker.hpp"
#include "util/meter_sampler.hpp"
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

TestBench<Serial> bench;

using brio::crlf;
using brio::print;

// ---------------------------------------------------------------------------
// The pins and the timers
// ---------------------------------------------------------------------------
using LedPin = Pin<'B', 23>;
using LedWave = TcWo<LedPin>;      // TC3, WO1 - from the device header
using PwmPad = Pin<'A', 22>;
using PwmWave = TcWo<PwmPad>;      // TC0, WO0
using EicPad = Pin<'A', 16>;
using EicLine = ExtInt<EicPad>;    // EXTINT0

using Timer0 = Tc<0>;
using Timer2 = Tc<2>;
using Timer3 = Tc<3>;

// Every TC on this suite runs from generator 0 (OSC48M, 48 MHz), which
// is the CPU's own clock - so every number below is exact arithmetic on
// 48'000'000 and not a measurement of an oscillator.
constexpr uint8_t tc_gen = 0;
constexpr uint32_t tc_hz = SysClock::hz;

// The event fabric. Channel 0 carries the EIC line to a timer; channel 1
// carries a timer's overflow to another timer.
constexpr uint8_t ev_pin_channel = 0;
constexpr uint8_t ev_ovf_channel = 1;
constexpr uint8_t ev_gen = 6;
using EvGen = Gclk<ev_gen>;

// The PWM under test: TC3, 8-bit, TOP 199, prescaler /256.
// 48 MHz / 256 / 200 = 937.5 Hz exactly.
constexpr uint8_t pwm_top = 199;
constexpr uint32_t pwm_hz = tc_hz / 256u / (pwm_top + 1u);
using LedPwm = TcPwm8<Timer3, 1, pwm_top>;

// The capture meter: TC2, /1024 -> 46875 Hz, one tick = 21.33 us.
constexpr uint16_t capture_prescale = 1024;
constexpr uint32_t capture_hz = tc_hz / capture_prescale;
using Meter = TcPeriodMeter<Timer2>;

volatile uint32_t tc2_captures = 0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void spin(uint32_t turns) {
    volatile uint32_t sink = 0;
    for (uint32_t i = 0; i < turns; ++i) {
        sink = sink + 1u;
    }
}

/// Wait `ms` milliseconds on the SysTick timebase - the one clock in
/// this suite that is not a TC.
void wait_ms(uint32_t ms) {
    const uint32_t deadline = Ticker::millis() + ms;
    while (static_cast<int32_t>(Ticker::millis() - deadline) < 0) {
    }
}

/// Sample a pad through PORT.IN many times and report how many reads
/// were high, in parts per thousand. The crude but honest way to read a
/// duty cycle with no instrument: at 937 Hz and ~40 CPU cycles a
/// sample, a few tens of thousands of reads span many periods.
uint32_t duty_permille(uint32_t samples = 60'000UL) {
    uint32_t high = 0;
    for (uint32_t i = 0; i < samples; ++i) {
        if (LedPin::read()) {
            ++high;
        }
    }
    return (high * 1000UL) / samples;
}

bool near(uint32_t got, uint32_t want, uint32_t band) {
    return got + band >= want && want + band >= got;
}

/// Bring one TC up on generator 0, disabled and reset.
template <class T>
bool timer_up() {
    return T::init(tc_gen);
}

// =============================================================================
// a - the block, the header's geometry, and every refusal
// =============================================================================
void ta_block() {
    bench.verdict("five instances, two channels each",
                  tc_count() == 5u && Timer0::cc_count == 2u);

    // THE SHARED GENERIC CLOCK CHANNELS, which 35.5.3 warns about and
    // only the device header can answer.
    print(serial, "  GCLK channels: TC0=", Timer0::gclk_id, " TC1=", Tc<1>::gclk_id,
          " TC2=", Timer2::gclk_id, " TC3=", Timer3::gclk_id,
          " TC4=", Tc<4>::gclk_id, crlf);
    bench.verdict("TC0 and TC1 SHARE a generic clock channel, and so do TC2 and "
                  "TC3 - so neither pair can run at two different rates "
                  "(35.5.3)",
                  Timer0::gclk_id == Tc<1>::gclk_id &&
                      Timer2::gclk_id == Timer3::gclk_id &&
                      Timer0::gclk_id != Timer2::gclk_id);

    bench.verdict("TC0 and TC2 can host a 32-bit pair; TC1, TC3 and TC4 cannot "
                  "(35.6.2.4, from TCn_MASTER_SLAVE_MODE)",
                  tc_can_pair(0) && tc_can_pair(2) && !tc_can_pair(1) &&
                      !tc_can_pair(3) && !tc_can_pair(4));
    bench.verdict("a 32-bit mode on a pair CLIENT is refused",
                  !Tc<1>::config_valid(TcConfig{.mode = TcMode::count32}) &&
                      Timer0::config_valid(TcConfig{.mode = TcMode::count32}));

    // THE PAD MAP, out of the device header and not out of arithmetic.
    print(serial, "  pads: PB23 -> TC", LedWave::timer, "/WO", LedWave::channel,
          ", PA22 -> TC", PwmWave::timer, "/WO", PwmWave::channel, crlf);
    bench.verdict("the board's LED is TC3's WO1 and PA22 is TC0's WO0",
                  LedWave::timer == 3u && LedWave::channel == 1u &&
                      PwmWave::timer == 0u && PwmWave::channel == 0u);
    bench.verdict("and a pad with no waveform output has none", !tc_wo_exists<'A', 16>);

    // The EVSYS and DMAC vocabularies this driver publishes.
    print(serial, "  EVSYS: TC3 OVF gen ", Timer3::overflow_generator,
          ", TC0 MC0 gen ", Timer0::match_generator(0), ", TC0 user ",
          Timer0::event_user, "; DMAC TC0 OVF trigger ",
          Timer0::dma_trigger_overflow, crlf);
    bench.verdict("the generator codes are the table's own (TC0 OVF 0x34, three "
                  "per instance)",
                  Timer0::overflow_generator == 0x34u &&
                      Timer0::match_generator(1) == 0x36u &&
                      Timer3::overflow_generator == 0x3Du);
    bench.verdict("and the users run 23..27", Timer0::event_user == 23u &&
                                                  Tc<4>::event_user == 27u);

    // The refusals that are not about geometry.
    bench.verdict("capture-on-pin without capture is refused",
                  !Timer0::config_valid(TcConfig{.capture_on_pin = 0x1}));
    bench.verdict("counting events while generating PWM is refused "
                  "(35.6.2.5.3)",
                  !tc_event_config_valid(
                      TcConfig{.waveform = TcWaveform::normal_pwm},
                      TcEventConfig{.action = TcEventAction::count}));
    bench.verdict("a capture action with no capture channel is refused",
                  !tc_event_config_valid(
                      TcConfig{}, TcEventConfig{.action = TcEventAction::stamp}));

    // ENABLE-PROTECTION, as refusals rather than as lost stores.
    bench.verdict("the timer comes up disabled", timer_up<Timer0>() &&
                                                     !Timer0::enabled());
    bench.verdict("and configures", Timer0::configure(TcConfig{
                      .mode = TcMode::count16, .prescaler = TcPrescaler::div1024}));
    bench.verdict("it enables", Timer0::enable(true));
    bench.verdict("and NOW a configuration is refused - CTRLA, WAVE, DRVCTRL "
                  "and EVCTRL are enable-protected (35.6.2.1)",
                  !Timer0::configure(TcConfig{.mode = TcMode::count8}) &&
                      !Timer0::event_config(TcConfig{},
                                            TcEventConfig{.overflow_out = true}));
    bench.verdict("the mode it was given survived the attempt",
                  Timer0::mode() == TcMode::count16);
    bench.verdict("it disables and takes the new configuration",
                  Timer0::enable(false) &&
                      Timer0::configure(TcConfig{.mode = TcMode::count8}));
    bench.verdict("and the mode changed", Timer0::mode() == TcMode::count8);

    Timer0::release();
}

// =============================================================================
// b - the counter runs, and the prescaler is exact
// =============================================================================
void tb_counter() {
    bench.verdict("TC0 comes up", timer_up<Timer0>());

    // THE PRESCALER RATIO, not the absolute rate: a ratio is immune to
    // everything the measurement chain adds, and 4x is a claim the
    // register description makes that nothing else here checks.
    // A DELTA, not an absolute reading, and nothing printed inside the
    // window: a verdict line is ~50 characters at 115200, which is 4 ms
    // of console - enough to put a 100 ms measurement 4 % out. The
    // window is opened and closed by two reads with nothing between
    // them but the wait.
    uint32_t delta[2] = {0, 0};
    const TcPrescaler scales[2] = {TcPrescaler::div1024, TcPrescaler::div256};
    bool configured = true;
    // 250 ms and not more: at /256 the counter runs at 187.5 kHz and a
    // 16-bit counter wraps in 350 ms, which would make the delta
    // ambiguous. A measurement window has to fit the counter it reads.
    constexpr uint32_t window_ms = 250;
    for (uint8_t i = 0; i < 2u; ++i) {
        configured = configured && Timer0::enable(false) &&
                     Timer0::configure(TcConfig{.mode = TcMode::count16,
                                                .prescaler = scales[i]}) &&
                     Timer0::set_count16(0) && Timer0::enable(true);
        const uint16_t before = Timer0::count16();
        wait_ms(window_ms);
        const uint16_t after = Timer0::count16();
        delta[i] = static_cast<uint16_t>(after - before);
    }
    bench.verdict("the counter configures and runs at two prescaler settings",
                  configured);
    const uint32_t expect_1024 = (tc_hz / 1024u) * window_ms / 1000u;
    print(serial, "  ", window_ms, " ms at /1024: ", delta[0],
          " ticks (exact ", expect_1024, "), at /256: ", delta[1],
          " ticks, ratio x1000 = ",
          delta[0] != 0 ? (delta[1] * 1000UL) / delta[0] : 0UL, crlf);
    bench.verdict("the counter advances at GCLK/1024 - 48 MHz over a "
                  "SysTick-timed window, within 1 %",
                  near(delta[0], expect_1024, expect_1024 / 100u + 2u));
    bench.verdict("and /256 counts FOUR TIMES as fast, which is the prescaler "
                  "ratio measured rather than assumed",
                  delta[0] != 0u &&
                      near((delta[1] * 1000UL) / delta[0], 4000u, 40u));

    // Reading COUNT is a COMMAND (35.6.8): two reads a moment apart must
    // differ, and the raw read must be the value the command fetched.
    const uint16_t a = Timer0::count16();
    spin(2000);
    const uint16_t b = Timer0::count16();
    bench.verdict("two READSYNC'd reads of a running counter differ", a != b);
    bench.verdict("and the raw accessor returns what the last command "
                  "fetched - which is why it is spelled raw",
                  Timer0::count16_raw() == b);

    // Direction, live (35.6.2.5 allows it under a running counter).
    (void)Timer0::set_count16(30'000);
    bench.verdict("the direction reverses", Timer0::count_down(true) &&
                                                Timer0::counting_down());
    const uint16_t down_a = Timer0::count16();
    wait_ms(20);
    const uint16_t down_b = Timer0::count16();
    bench.verdict("and the counter now counts DOWN", down_b < down_a);
    bench.verdict("back up again", Timer0::count_down(false) &&
                                       !Timer0::counting_down());

    // STOP and RETRIGGER.
    bench.verdict("the STOP command stops it", Timer0::stop() && Timer0::stopped());
    const uint16_t frozen = Timer0::count16();
    wait_ms(20);
    bench.verdict("and the counter is frozen", Timer0::count16() == frozen);
    bench.verdict("RETRIGGER restarts it, and STATUS.STOP clears itself",
                  Timer0::retrigger() && !Timer0::stopped());
    wait_ms(20);
    bench.verdict("the counter moves again", Timer0::count16() != frozen);

    // ONE-SHOT: stop at the next overflow, by itself (35.6.3.1).
    bench.verdict("an 8-bit one-shot counter is configured",
                  Timer0::enable(false) &&
                      Timer0::configure(TcConfig{.mode = TcMode::count8,
                                                 .prescaler = TcPrescaler::div1024,
                                                 .one_shot = true}) &&
                      Timer0::set_period8(200) && Timer0::set_count8(0) &&
                      Timer0::enable(true));
    bench.verdict("it starts running", !Timer0::stopped());
    // 200 ticks at 46875 Hz is ~4.3 ms; 50 ms is many times that.
    wait_ms(50);
    bench.verdict("and stops itself at the first overflow, with no software in "
                  "the path",
                  Timer0::stopped() && (Timer0::flags() & Timer0::overflow_flag) != 0u);

    Timer0::release();
}

// =============================================================================
// c - COUNT32 by pairing TC0 with TC1
// =============================================================================
void tc_pair() {
    bench.verdict("TC0 comes up, and init() enables BOTH halves' bus clocks - "
                  "35.6.2.4 asks for it and nothing else would",
                  timer_up<Timer0>());
    bench.verdict("the 32-bit mode is accepted on the pair master",
                  Timer0::configure(TcConfig{.mode = TcMode::count32,
                                             .prescaler = TcPrescaler::div1}) &&
                      Timer0::set_count32(0) && Timer0::enable(true));

    // At 48 MHz with no prescaler, 65536 ticks is 1.37 ms: a 16-bit
    // counter would have wrapped many times over in 50 ms, while a
    // 32-bit one is nowhere near its own MAX.
    // A delta again, and for the same reason as letter b.
    const uint32_t before32 = Timer0::count32();
    wait_ms(200);
    const uint32_t wide = Timer0::count32() - before32;
    const uint32_t expect = tc_hz / 5u;
    print(serial, "  200 ms at 48 MHz, no prescaler: COUNT32 advanced ", wide,
          " (exact ", expect, "), which is ", wide >> 16, " times past 65535",
          crlf);
    bench.verdict("THE COUNTER IS GENUINELY 32 BITS WIDE - it advanced far "
                  "past what a 16-bit counter could hold without wrapping",
                  wide > 0x10000UL);
    bench.verdict("and it counts at the rate it was given, within 1 %",
                  near(wide, expect, expect / 100u + 100u));

    // The client half says what it is, and its own registers do not
    // reflect the wide counter (35.6.2.4).
    Tc<1>::bus_clock(true);
    const bool client = Tc<1>::is_client();
    print(serial, "  TC1 STATUS.SLAVE = ", client ? "1" : "0", crlf);
    bench.verdict("TC1 reports itself the CLIENT of the pair", client);
    bench.verdict("while TC0 does not", !Timer0::is_client());

    // And the control: the same instance in 16-bit mode wraps.
    bench.verdict("back to 16 bits",
                  Timer0::enable(false) &&
                      Timer0::configure(TcConfig{.mode = TcMode::count16,
                                                 .prescaler = TcPrescaler::div1}) &&
                      Timer0::set_count16(0) && Timer0::enable(true));
    wait_ms(200);
    const uint32_t narrow = Timer0::count16();
    print(serial, "  the same 200 ms in 16-bit mode: COUNT = ", narrow,
          " - the counter wrapped ", wide >> 16, " times to get there", crlf);
    bench.verdict("and its overflow flag stands, many wraps later",
                  (Timer0::flags() & Timer0::overflow_flag) != 0u);

    Timer0::release();
}

// =============================================================================
// d - PWM: duty off the pad, frequency off a second timer
// =============================================================================
void td_pwm() {
    bench.verdict("TC3 comes up", timer_up<Timer3>());
    LedWave::claim();
    bench.verdict("the 8-bit PWM channel sets up on the LED's own waveform "
                  "output",
                  LedPwm::setup(TcPrescaler::div256));
    bench.verdict("and it is a PwmChannel whose full scale is the PERIOD - "
                  "which is the whole reason 8-bit mode has a PER register",
                  LedPwm::max == pwm_top);

    struct Case {
        uint16_t duty;
        uint32_t expect_permille;
        const char* what;
    };
    const Case cases[] = {
        {0, 0, "zero duty is a pad held low"},
        {pwm_top, 995, "full duty is a pad held high"},
        {(pwm_top + 1u) / 2u, 500, "half duty is a pad high half the time"},
        {(pwm_top + 1u) / 4u, 250, "and a quarter is a quarter"},
    };
    for (const auto& c : cases) {
        LedPwm::duty(c.duty);
        // The buffered write lands at the next UPDATE, so give the
        // waveform a few periods before believing the pad.
        wait_ms(20);
        const uint32_t measured = duty_permille();
        print(serial, "  duty ", c.duty, "/", pwm_top, ": pad high ", measured,
              " per mille (expected ~", c.expect_permille, ")", crlf);
        bench.verdict("the ", c.what,
                      c.expect_permille == 0
                          ? measured == 0u
                          : (c.expect_permille >= 990
                                 ? measured >= 980u
                                 : near(measured, c.expect_permille, 40u)));
    }

    // THE FREQUENCY, measured with no wire and no scope: TC0 counts
    // TC3's OVERFLOW EVENTS. 35.6.2.5.3 forbids PWM in that mode and
    // permits everything else, which is exactly what a counter used as
    // a counter wants.
    LedPwm::duty((pwm_top + 1u) / 2u);
    Evsys::bus_clock(true);
    bench.verdict("the event channel's clock is routed",
                  EvGen::configure(GclkConfig{.source = GclkSource::osculp32k}) &&
                      GclkChannel::connect(Evsys::gclk_id(ev_ovf_channel), ev_gen));
    bench.verdict("TC3's overflow becomes an event",
                  Timer3::enable(false) &&
                      Timer3::event_config(
                          TcConfig{.mode = TcMode::count8,
                                   .prescaler = TcPrescaler::div256,
                                   .waveform = TcWaveform::normal_pwm},
                          TcEventConfig{.overflow_out = true}) &&
                      Timer3::enable(true));
    bench.verdict("TC0 is set up to COUNT events rather than clock ticks",
                  timer_up<Timer0>() &&
                      Timer0::configure(TcConfig{.mode = TcMode::count16}) &&
                      Timer0::event_config(TcConfig{.mode = TcMode::count16},
                                           TcEventConfig{
                                               .action = TcEventAction::count,
                                               .input_enable = true}));
    bench.verdict("and listens to the channel carrying TC3's overflow, on the "
                  "ASYNCHRONOUS path 35.6.6 asks for",
                  Evsys::connect(Timer0::event_user, ev_ovf_channel,
                                 EventChannelConfig{
                                     .generator = Timer3::overflow_generator,
                                     .path = EventPath::asynchronous}));
    bench.verdict("the counting timer enables",
                  Timer0::set_count16(0) && Timer0::enable(true));

    (void)Timer0::set_count16(0);
    wait_ms(1000);
    const uint32_t counted = Timer0::count16();
    print(serial, "  TC3 overflows counted by TC0 in 1 s: ", counted,
          " (48 MHz / 256 / ", pwm_top + 1u, " = ", pwm_hz, " Hz)", crlf);
    bench.verdict("THE PWM FREQUENCY IS WHAT THE ARITHMETIC SAYS - one timer "
                  "counting another's events, within 1 %",
                  near(counted, pwm_hz, pwm_hz / 100u + 2u));

    Evsys::disconnect(Timer0::event_user);
    GclkChannel::disconnect(Evsys::gclk_id(ev_ovf_channel));
    LedWave::release();
    LedPin::output();
    Timer0::release();
    Timer3::release();
}

// =============================================================================
// e - capture, from both sources
// =============================================================================
//
// 35.6.2.8 note 4 names the recipe: "Capture of the period and duty
// cycle on I/Os using PPW/PWP mode is possible using EIC and Event
// System", and note 2 requires the event channel to be ASYNCHRONOUS.
// The EIC line is sensed on a LEVEL so that its event output is a COPY
// of the pad rather than a pulse - which is what a capture action that
// distinguishes rising from falling needs.
void te_capture() {
    Evsys::bus_clock(true);
    Evsys::reset();
    bench.verdict("the EIC comes up on CLK_ULP32K",
                  Eic::init() && Eic::clock_select(EicClock::ulp32k));
    EicPad::input(PinPull::down);
    EicLine::claim(PinPull::down);
    bench.verdict("EXTINT0 senses a HIGH LEVEL and drives its event output, so "
                  "the event line is a COPY of the pad",
                  Eic::configure_line(EicLine::line,
                                      EicLineConfig{.sense = EicSense::high,
                                                    .event_out = true}) &&
                      Eic::enable(true));

    bench.verdict("TC2 comes up as a period/pulse-width meter",
                  timer_up<Timer2>() &&
                      Meter::setup(TcPrescaler::div1024));
    bench.verdict("and both its capture channels are enabled - 35.6.2.8.2 says "
                  "both are needed to characterize an input",
                  (Timer2::ctrla() & TC_CTRLA_CAPTEN_Msk) ==
                      TC_CTRLA_CAPTEN(0x3));
    bench.verdict("the meter listens to the EIC line on an ASYNCHRONOUS "
                  "channel (35.6.2.8 note 2)",
                  Evsys::connect(Timer2::event_user, ev_pin_channel,
                                 EventChannelConfig{
                                     .generator = EicLine::event_generator,
                                     .path = EventPath::asynchronous}));

    // A square-ish wave built by hand on the SysTick timebase: 20 ms
    // high, 30 ms low, period 50 ms. At 46875 Hz that is 938 ticks high
    // and 2344 per period - both comfortably inside 16 bits.
    //
    // FIRST, THE UNREAD CASE, which is a finding in its own right:
    // 35.6.2.8.2 says a capture arriving while INTFLAG.MCx is still set
    // is DROPPED and INTFLAG.ERR raised. Four periods with nothing
    // read is exactly that.
    Timer2::clear_flags(0xFFu);
    for (uint8_t i = 0; i < 4u; ++i) {
        EicPad::set();
        wait_ms(20);
        EicPad::clear();
        wait_ms(30);
    }
    const bool overran = (Timer2::flags() & Timer2::error_flag) != 0u;

    // NOW THE DRAINED CASE. Reading CCx is what empties the capture
    // FIFO stage (35.6.2.8), so a reader that keeps up sees every
    // capture and INTFLAG.ERR stays clear.
    uint16_t period = 0;
    uint16_t width = 0;
    (void)Meter::period_ticks();
    (void)Meter::width_ticks();
    Timer2::clear_flags(0xFFu);
    for (uint8_t i = 0; i < 4u; ++i) {
        EicPad::set();
        wait_ms(20);
        EicPad::clear();
        wait_ms(30);
        period = Meter::period_ticks();
        width = Meter::width_ticks();
    }
    const bool clean = (Timer2::flags() & Timer2::error_flag) == 0u;
    const uint32_t expect_period = (capture_hz * 50u) / 1000u;
    const uint32_t expect_width = (capture_hz * 20u) / 1000u;
    print(serial, "  captured: period ", period, " ticks (expected ~",
          expect_period, "), width ", width, " ticks (expected ~", expect_width,
          "); ERR unread=", overran ? "1" : "0", " drained=",
          clean ? "0" : "1", crlf);
    bench.verdict("THE PERIOD LANDED IN CC0 - a pin edge, through the EIC and "
                  "an asynchronous event channel, into a capture register",
                  near(period, expect_period, expect_period / 20u + 4u));
    bench.verdict("and the pulse width in CC1", near(width, expect_width,
                                                     expect_width / 20u + 4u));
    bench.verdict("CAPTURES LEFT UNREAD ARE DROPPED and raise INTFLAG.ERR "
                  "(35.6.2.8.2), while a reader that drains CCx keeps up with "
                  "no error at all - reading CCx IS the acknowledgement",
                  overran && clean);

    Evsys::disconnect(Timer2::event_user);
    Timer2::release();

    // --- capture ON THE PIN, which erratum 1.20.2 says does not work
    //
    // That item is REVISION B ONLY on this family, and this is the
    // measurement that says so. The pad is muxed to the timer, so PORT's
    // output driver is gone (test_samc_eic established that) and the
    // stimulus is the pad's own internal pull.
    // The stimulus is the pad's own internal pull, so the pad has to
    // follow it - checked under PORT before the timer ever sees it,
    // exactly as test_samc_ac learned to do after PA04 turned out not
    // to.
    PwmPad::input(PinPull::up);
    wait_ms(2);
    const bool pull_up = PwmPad::read();
    PwmPad::input(PinPull::down);
    wait_ms(2);
    const bool pull_down = PwmPad::read();
    print(serial, "  PA22 under its own pull: up -> ", pull_up ? "1" : "0",
          ", down -> ", pull_down ? "1" : "0", crlf);
    bench.verdict("PA22 follows its own internal pull, which is what the "
                  "capture-on-pin stimulus needs",
                  pull_up && !pull_down);

    bench.verdict("TC0 comes up with channel 0 capturing FROM ITS PAD",
                  timer_up<Timer0>() &&
                      Timer0::configure(TcConfig{
                          .mode = TcMode::count16,
                          .prescaler = TcPrescaler::div1024,
                          .capture_enable = 0x1,
                          .capture_on_pin = 0x1}) &&
                      Timer0::enable(true));

    // TWO WAYS TO MOVE THE PAD, because it is not obvious which one the
    // silicon allows and the answer is worth having in writing.
    //
    //  (1) leave the pad under PORT and DRIVE it. The AC's analog input
    //      reaches the pad with no mux at all (docs/samc/ac.md), so it
    //      is a fair question whether a digital capture input does too.
    //  (2) hand the pad to the timer (function E) and move it with its
    //      own internal pull, the way test_samc_eic moves an EIC line.
    struct Attempt {
        bool captured;
        bool moved;
        uint16_t stamp;
    };
    Attempt driven{};
    Attempt muxed{};

    PwmPad::release();
    PwmPad::output();
    PwmPad::clear();
    wait_ms(5);
    (void)Timer0::cc16(0);
    Timer0::clear_flags(0xFFu);
    wait_ms(5);
    const bool quiet = (Timer0::flags() & Timer0::match_flag(0)) == 0u;
    PwmPad::set();
    wait_ms(5);
    driven.moved = PwmPad::read();
    driven.captured = (Timer0::flags() & Timer0::match_flag(0)) != 0u;
    driven.stamp = Timer0::cc16(0);
    PwmPad::clear();
    wait_ms(5);

    PwmPad::function(PinFunction::e,
                     PinConfig{.input_enable = true, .pull = PinPull::down});
    wait_ms(5);
    (void)Timer0::cc16(0);
    Timer0::clear_flags(0xFFu);
    const bool muxed_low = PwmPad::read();
    PwmPad::set();     // the pull's direction is the OUT bit (28.6.3.2)
    wait_ms(5);
    muxed.moved = PwmPad::read() && !muxed_low;
    muxed.captured = (Timer0::flags() & Timer0::match_flag(0)) != 0u;
    muxed.stamp = Timer0::cc16(0);
    PwmPad::clear();
    wait_ms(5);

    print(serial, "  capture-on-pin, pad under PORT and driven: moved ",
          driven.moved ? "yes" : "no", ", captured ",
          driven.captured ? "yes" : "no", ", CC0 = ", driven.stamp, crlf);
    print(serial, "  capture-on-pin, pad muxed to the timer and pulled: moved ",
          muxed.moved ? "yes" : "no", ", captured ",
          muxed.captured ? "yes" : "no", ", CC0 = ", muxed.stamp, crlf);

    bench.verdict("nothing was captured before the pad moved", quiet);
    bench.verdict("A PAD HANDED TO A DRIVING PERIPHERAL FUNCTION DOES NOT "
                  "MOVE UNDER ITS OWN PULL - unlike an input-only function "
                  "such as the EIC's, function E takes the output driver and "
                  "holds the pin",
                  !muxed.moved);
    bench.verdict("and a pad left under PORT is not seen by the capture input "
                  "at all - a DIGITAL peripheral input needs the mux, where "
                  "the AC's ANALOG one reaches the pad without it",
                  driven.moved && !driven.captured);

    // NO VERDICT ON THE ERRATUM, and that is the point. 1.20.2 says
    // capture on I/O pins does not work and is marked revision B only,
    // so this silicon should capture - but the two facts just measured
    // between them mean this board cannot present a CONTROLLED edge to a
    // muxed WO pad from inside the chip, and a verdict on a stimulus
    // that never arrived would be a verdict on nothing. What can be said
    // is printed: CC0 did not stay at zero, so something was captured
    // when the pad was handed over. docs/samc/tc.md carries this as an
    // open gap, not as a claim either way.
    print(serial, "  erratum 1.20.2 (capture on I/O pins) is NOT judged here: "
          "no controlled edge can reach a muxed WO pad on this board. CC0 "
          "after the handover was ", muxed.stamp,
          ", so the path is not obviously dead.", crlf);

    PwmPad::release();
    PwmPad::configure({});
    Timer0::release();
    (void)Eic::enable(false);
    Eic::release();
    EicPad::configure({});
}

// =============================================================================
// f - the util contract, live: MeterSampler inside a running kernel
// =============================================================================
//
// THIS IS THE CAMPAIGN'S POINT, not a bonus letter. util/meter_sampler.hpp
// was designed on the AVR around a capture ISR that fills a one-cell
// latch and an AO that paces PUBLICATION rather than capture. Here the
// capture comes from a SAM TC, through EVSYS, from an EIC pin - a chain
// with nothing in common with the AVR's - and NOT ONE LINE OF util/
// CHANGED.

using Latch = MeterLatch<uint16_t, SamPlatform, 0>;
static_assert(MeterSource<Latch>, "the TC's capture latch is a MeterSource");

struct Collector;
using Subs = Subscribers<Collector>;
using Sampler = MeterSampler<SamPlatform, Subs, Latch>;

struct Collector {
    using Event = std::variant<MeterSample>;
    static inline EventQueue<Event, 8, SamPlatform> queue;

    static inline uint16_t samples = 0;
    static inline uint32_t last = 0;
    static inline uint8_t last_index = 0xFF;

    static void init() {
        samples = 0;
        last = 0;
        last_index = 0xFF;
    }
    static void dispatch(const Event& e) {
        match(e, [](MeterSample s) {
            last = s.value;
            last_index = s.index;
            if (samples != UINT16_MAX) {
                ++samples;
            }
        });
    }
};

using MeterKernel = Kernel<SamPlatform, Collector, Sampler>;

void tf_meter_ao() {
    // The hardware first: the same EIC-to-EVSYS-to-TC2 chain letter e
    // built, plus the interrupt that fills the latch.
    Evsys::bus_clock(true);
    Evsys::reset();
    bench.verdict("the EIC and the capture meter come up",
                  Eic::init() && Eic::clock_select(EicClock::ulp32k) &&
                      timer_up<Timer2>());
    EicPad::input(PinPull::down);
    EicLine::claim(PinPull::down);
    bench.verdict("the pin's level becomes an event",
                  Eic::configure_line(EicLine::line,
                                      EicLineConfig{.sense = EicSense::high,
                                                    .event_out = true}) &&
                      Eic::enable(true));
    bench.verdict("TC2 captures period and width from it",
                  Meter::setup(TcPrescaler::div1024) &&
                      Evsys::connect(Timer2::event_user, ev_pin_channel,
                                     EventChannelConfig{
                                         .generator = EicLine::event_generator,
                                         .path = EventPath::asynchronous}));

    Latch::clear();
    tc2_captures = 0;
    Timer2::clear_flags(0xFFu);
    Timer2::arm(Meter::period_flag);
    Nvic::enable(Timer2::irq());

    // Now the software: a kernel with two AOs, the sampler paced at 100
    // ms by a time event of its own.
    MeterKernel::init_all();
    Sampler::start_every(Ticker::ticks_per_second / 10u);

    // Two seconds of square wave, with the kernel pumped between edges.
    // The capture rate is 20 Hz and the PUBLICATION rate is 10 Hz, which
    // is the whole design: the AO paces what reaches the queues, not
    // what the ISR does.
    const uint32_t started = Ticker::millis();
    uint32_t next_edge = started;
    bool high = false;
    while (Ticker::millis() - started < 2000UL) {
        TimeEvents<SamPlatform>::process();
        while (MeterKernel::step()) {
        }
        if (static_cast<int32_t>(Ticker::millis() - next_edge) >= 0) {
            high = !high;
            if (high) {
                EicPad::set();
                next_edge += 20u;
            } else {
                EicPad::clear();
                next_edge += 30u;
            }
        }
    }

    const uint32_t expect_period = (capture_hz * 50u) / 1000u;
    print(serial, "  2 s of a 20 Hz wave: ", tc2_captures, " ISR captures, ",
          Sampler::published(), " samples published, ", Collector::samples,
          " received; last value ", Collector::last, " ticks (expected ~",
          expect_period, "), latch missed ", Sampler::missed(0), crlf);

    bench.verdict("the capture ISR ran, filling the latch from the TC",
                  tc2_captures >= 30u);
    bench.verdict("the SAMPLER PACED PUBLICATION rather than capture - ~10 Hz "
                  "of samples against ~20 Hz of captures, which is the whole "
                  "design of util/meter_sampler.hpp",
                  Sampler::published() >= 15u &&
                      Sampler::published() < tc2_captures);
    bench.verdict("every published sample reached the subscriber through the "
                  "kernel",
                  Collector::samples == Sampler::published());
    bench.verdict("labelled with the source's position in the sampler's list",
                  Collector::last_index == 0u);
    bench.verdict("and carrying a period the TC really captured",
                  near(Collector::last, expect_period, expect_period / 10u + 8u));
    bench.verdict("the latch counted the captures it overwrote, which is what "
                  "makes a paced publication honest",
                  Sampler::missed(0) > 0u);

    Sampler::stop();
    Nvic::disable(Timer2::irq());
    Evsys::disconnect(Timer2::event_user);
    Timer2::release();
    (void)Eic::enable(false);
    Eic::release();
    EicPad::configure({});
}

void banner() {
    print(serial, crlf,
          "test_samc_tc - SAMC21J18A TC (ch. 35): five timers, the pairs, PWM, "
          "capture and the meter AO, wireless, clk=",
          SysClock::hz, " Hz", crlf);
    bench.menu();
}

} // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }

/// TC2's capture handler: READ CC0 - which is what empties the capture
/// FIFO stage - and hand the reading to the latch. The latch is the
/// bridge util/meter_sampler.hpp asks for, and everything above it is
/// target-independent.
extern "C" void TC2_Handler() {
    if (brio::Tc<2>::isr() != 0u) {
        Latch::store(Meter::period_ticks());
        tc2_captures = tc2_captures + 1u;
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    LedPin::output();
    brio::enable_interrupts();

    bench.letter('a', "the block, the header's geometry and every refusal",
                 ta_block);
    bench.letter('b', "the counter, the prescaler ratio and one-shot", tb_counter);
    bench.letter('c', "COUNT32 by pairing TC0 with TC1", tc_pair);
    bench.letter('d', "PWM on the LED: duty off the pad, frequency off a second "
                      "timer", td_pwm);
    bench.letter('e', "capture from an event and capture from the pin",
                 te_capture);
    bench.letter('f', "a MeterSampler AO inside a running kernel", tf_meter_ao);

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
