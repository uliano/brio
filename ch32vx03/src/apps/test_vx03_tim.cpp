// test_vx03_tim - the reference bench suite for the TIMERS of the
// CH32V203 and the CH32V303: ch32vx03/tim.hpp over RM ch. 14 (the
// advanced-control blocks), ch. 15 (the general-purpose ones) and ch. 16
// (the basic ones).
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// WHAT IT MEASURES WITH, AND WHAT A WIRE ADDS. Every letter runs with
// the board bare: a timer's channel drives its own pad through the
// output stage and another channel of the SAME timer captures it (the
// input path of a channel is live whatever CCyS says), the internal
// triggers link two timers with no pad at all, and the core's own STK
// counter is the ruler every number is weighed against. TWO OPTIONAL
// JUMPERS add what one timer cannot see of itself: PA6 to PA1 - TIM3's
// channel 1 to TIM2's channel 2 - lets a SECOND timer measure the first
// one's waveform, and carries the CH32V303's dual-edge capture; PC6 to
// PB8 - TIM8's channel 1 to TIM4's channel 3 - carries the CH32V303's
// extra advanced timer to a capture. Each letter that needs one tests
// for it first (a plain output on one pad read on the other, both
// levels) and says "no jumper" and passes when it is not there.
//
// THE CH32V303's TIMERS (letters k..s, compiled where the part has
// them): TIM8, TIM9 and TIM10 with TIM1's shape and four vectors each,
// TIM5 sixteen bits wide on that class, the two basic timers TIM6 and
// TIM7, the dual-edge capture register, TIM3's external trigger on PD2
// and the internal-trigger links the class adds.
//
// THE PADS. TIM3's column 0 is PA6/PA7/PB0/PB1, TIM2's is PA0..PA3,
// TIM1's is PA8 with PB13 for its complementary output and PB12 for its
// break input, TIM4's is PB6..PB9. Of those this suite drives PA1,
// PA6, PA7, PA8, PB6..PB9, PB12 and PB13; on the CH32V303 it adds PC6
// and PA7 (TIM8's channel 1 and its complement, PA6 its break input),
// PA4 (TIM9's channel 3), PC3 (TIM10's) and PD2 (TIM3's external
// trigger). A pad strapped to another pad of the board is driven only
// while the other end is an input, and a pad under a pull resistor is
// judged against the level the resistor gives it (letter f).
// NEVER TOUCHED: PA9/PA10 (the console), PA13/PA14 (the debug port),
// PA11/PA12 (the USB pads), PC14/PC15 and the 8 MHz crystal's pads (the
// crystals), PB2 (the LED, toggled per command as every suite of this
// target does) and PA0, which carries the KEY button and is TIM2's
// channel 1 - so this suite uses TIM2's channel 2 and leaves its first
// channel alone.
//
// What is exercised, letter by letter:
//   a  THE TIME BASE: the gate and the reset state, the prescaler and
//      the auto-reload counted against the STK over a tenth of a
//      second, and the two SHADOW registers - a period written with
//      ARPE set taken at the update and not before
//   b  PWM: frequency and duty read back through the registers, the
//      high time MEASURED by a second channel of the same timer
//      through the indirect input mapping, and by TIM2 over the jumper
//      when it is there
//   c  THE COMPLEMENTARY PAIR on TIM1 with its dead time: the DTG
//      ladder read back in all four of its ranges, the two outputs
//      sampled together (never both high), each one's high time, and
//      the dead band TIMED as the gap it is - from one output's fall
//      to the other's rise
//   d  THE METERS: TimIntervalMeter between edges a channel's own
//      output stage makes on its pad, TimPeriodMeter over the TI1 XOR
//      with the CPU making the waveform, both feeding a
//      util/meter_sampler.hpp MeterLatch - and a stale source silent
//   e  THE COUNTERS: TimEventCounter counting another timer's updates
//      over an internal trigger, TimGatedCounter measuring a duty
//      cycle from the master's OC1REF - both with no pad at all
//   f  THE TASKS: TimPeriodicTick against the STK, TimOnePulse whose
//      width is timed on the pad (the preloaded compare proven
//      LOADED), and TimEncoder - whose tracks the PORT drives, the
//      forced output stage being cut off in an encoder mode (the
//      letter measures both halves of that)
//   g  THE BREAK: the software break clearing MOE, the automatic
//      output enable bringing it back at the next update, the idle
//      levels and the off-state bits read back, and the BKIN pad
//      driven by the port itself
//   h  THE TRIGGER CHAINS: TRGO in its three shapes (update, enable,
//      compare) reaching a slave over ITRx - reset mode, gated mode
//      and trigger mode, each judged by what the slave's counter did
//   i  THE VECTORS: the advanced timer's four lines each answering for
//      its own flags, a general-purpose timer's one line answering for
//      everything, and a flag whose interrupt is not enabled left
//      standing for a poller
//   j  THE JUMPER: whether PA6 and PA1 are strapped together, and
//      whether a pad in plain OUTPUT mode reaches a timer's capture
//      input at all
//   k  (the CH32V303) TIM8's PWM on PC6 CAPTURED BY TIM4 over the
//      PC6-PB8 jumper: period and high time at five duties and four
//      frequencies, rising and falling edges on two channels of one input
//   l  (the CH32V303) TIM8's COMPLEMENTARY PAIR, PC6 and PA7: the dead
//      time's ladder read back and the band timed as the gap it is
//   m  (the CH32V303) TIM8's FOUR VECTORS: update and compare counted
//      on two lines, the trigger and a commutation on the third, and the
//      break raised by BKIN - PA6 driven by the port - on the fourth
//   n  (the CH32V303) TIM9 AND TIM10 with no wire: the time base against
//      the core's counter, a PWM captured by its own timer, and the two
//      vectors of each that the letter arms
//   o  (the CH32V303) TIM5's WIDTH: a 32-bit value written raw into the
//      counter and the auto-reload, and the counter run across 0xFFFF
//   p  (the CH32V303) THE BASIC TIMERS: TIM6 and TIM7 against the core's
//      counter, their vectors, and their TRGO counted by TIM9 over the
//      internal triggers table 14-2 gives them
//   q  (the CH32V303) THE DUAL-EDGE CAPTURE over the PA6-PA1 jumper:
//      TIM2's channel 2 holding a pulse's width in one register, beside
//      the two-channel measurement of the same wave
//   r  (a package with PD2) TIM3'S EXTERNAL TRIGGER on PD2, the pad
//      driven by the port: external clock mode 2 counting its edges,
//      inverted and prescaled
//   s  (the CH32V303) THE INTERNAL TRIGGERS the class adds: every link
//      into TIM8, TIM9 and TIM10, TIM8 and TIM5 as masters of the others,
//      and TIM2's ITR1 with its AFIO field both ways
//
// build: boards = v203c6,v203c8,v303vc
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "ch32vx03/afio.hpp"
#include "ch32vx03/clock.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/ticker.hpp"
#include "ch32vx03/tim.hpp"
#include "ch32vx03/usart.hpp"
#include "util/meter_sampler.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using P = Ch32vx03Platform<>;
using Serial = Uart<1, P>;
constexpr Serial serial;
using Led = Pin<'B', 2>;

using SysClock = Clock<ClockSource::pll, 144'000'000>;
constexpr SysClock clock;

TestBench<Serial> bench;

// The four timers, by the job they do here.
using Wave = Tim<3>;      ///< the waveform generator, column 0: PA6, PA7, PB0, PB1
using Meas = Tim<2>;      ///< the measurer over the jumper, column 0: PA0..PA3
using Adv = Tim<1>;       ///< the advanced timer, column 0: PA8 / PB13 / PB12
using Quad = Tim<4>;      ///< the encoder and the meters, column 0: PB6..PB9

/// TIMxCLK is HCLK on both buses at every rate this stratum makes
/// (RM 3.3.1 with PB1 capped at half the ceiling), so one constant
/// serves every arithmetic here.
constexpr uint32_t tim_hz = Wave::clock_hz(clock);
static_assert(tim_hz == SysClock::hz);
static_assert(Adv::clock_hz(clock) == tim_hz);

/// A microsecond of the timers' clock, and of the core's counter -
/// the same number here, which is why the STK is a fair ruler.
constexpr uint32_t ticks_per_us = tim_hz / 1'000'000u;

// The pads, once. Every one of them comes from afio.hpp's own column
// table through tim.hpp, so a remap code and a pad cannot disagree.
constexpr Pad wave_ch1 = tim_channel_pad(3, 0, 0);     ///< PA6
constexpr Pad meas_ch2 = tim_channel_pad(2, 0, 1);     ///< PA1
constexpr Pad adv_ch1 = tim_channel_pad(1, 0, 0);      ///< PA8
constexpr Pad adv_ch1n = tim_complementary_pad(1, 0, 0);  ///< PB13
constexpr Pad adv_bkin = tim_break_pad(1, 0);             ///< PB12
constexpr Pad quad_a = tim_channel_pad(4, 0, 0);       ///< PB6
constexpr Pad quad_b = tim_channel_pad(4, 0, 1);       ///< PB7
constexpr Pad quad_c = tim_channel_pad(4, 0, 2);       ///< PB8

static_assert(wave_ch1 == Pad{'A', 6} && meas_ch2 == Pad{'A', 1});
static_assert(adv_ch1 == Pad{'A', 8} && adv_ch1n == Pad{'B', 13});
static_assert(adv_bkin == Pad{'B', 12});
static_assert(quad_a == Pad{'B', 6} && quad_c == Pad{'B', 8});

using WavePad = TimPad<wave_ch1>;
using MeasPad = TimPad<meas_ch2>;
using AdvPad = TimPad<adv_ch1>;
using AdvPadN = TimPad<adv_ch1n>;
using BreakPad = TimPad<adv_bkin>;
using QuadA = TimPad<quad_a>;
using QuadB = TimPad<quad_b>;
using QuadC = TimPad<quad_c>;

/// Whether the one optional wire is on the board. Letter j measures it;
/// every other letter reads it and says so.
bool jumper = false;
bool jumper_known = false;

/// What the vectors counted (letter i).
volatile uint32_t up_calls = 0;
volatile uint32_t cc_calls = 0;
volatile uint32_t brk_calls = 0;
volatile uint32_t trg_calls = 0;
volatile uint32_t wave_calls = 0;
volatile uint16_t brk_mask = 0;
volatile uint16_t cc_mask = 0;

/// The capture bridge of letter d: a MeterLatch per meter, filled in
/// the capture body and emptied in the loop (design/meters.md).
using PeriodLatch = MeterLatch<uint32_t, P, 0>;
using WidthLatch = MeterLatch<uint32_t, P, 1>;
using IntervalLatch = MeterLatch<uint32_t, P, 2>;

using Interval = TimIntervalMeter<Wave, 1>;   ///< TIM3 channel 2, indirect on TI1
using Period = TimPeriodMeter<Quad>;          ///< TIM4 channels 1 and 2, over the TI1 XOR

volatile bool interval_arm = false;
volatile bool period_arm = false;

/**
 * THE RULER OF THIS SUITE: the core's own STK counter, read as a
 * stopwatch. It counts up to its reload - one tick period, a
 * millisecond here - and starts again, so a span is accumulated poll by
 * poll with one period folded in across each wrap, exactly as
 * ch32vx03/delay.hpp's own wait does. That is also why this file does
 * not use delay_us(): it refuses a tick period and above, and several
 * letters here measure milliseconds.
 */
class Stopwatch {
public:
    Stopwatch() { start(); }

    void start() {
        last_ = stk()->CNTL;
        acc_ = 0;
    }

    uint32_t cycles() {
        const uint32_t period = stk()->CMPLR + 1u;
        const uint32_t now = stk()->CNTL;
        acc_ += (now >= last_) ? (now - last_) : (now + period - last_);
        last_ = now;
        return acc_;
    }

    uint32_t us() { return cycles() / ticks_per_us; }

private:
    uint32_t last_ = 0;
    uint32_t acc_ = 0;
};

/// Spin for that many microseconds on the same ruler, with no ceiling.
void wait_us(uint32_t us) {
    Stopwatch w;
    while (w.us() < us) {
    }
}

/// Every timer and pad this suite touches, back to reset. Called at the
/// top of each letter, so no letter inherits another's state.
void all_off() {
    Wave::release();
    Meas::release();
    Adv::release();
    Quad::release();
    for (const Irq line : {Irq::tim1_brk, Irq::tim1_up, Irq::tim1_trg_com, Irq::tim1_cc,
                           Irq::tim2, Irq::tim3, Irq::tim4}) {
        Pfic::disable(line);
        Pfic::clear_pending(line);
    }
    WavePad::release();
    MeasPad::release();
    AdvPad::release();
    AdvPadN::release();
    BreakPad::release();
    QuadA::release();
    QuadB::release();
    QuadC::release();
    up_calls = 0;
    cc_calls = 0;
    brk_calls = 0;
    trg_calls = 0;
    wave_calls = 0;
    brk_mask = 0;
    cc_mask = 0;
    interval_arm = false;
    period_arm = false;
    PeriodLatch::clear();
    WidthLatch::clear();
    IntervalLatch::clear();
}

/// Is the jumper there? Drive PA6 as a plain output and read PA1, both
/// ways round: a strap shows both levels, an open pad holds whatever
/// its pull says.
bool measure_jumper() {
    all_off();
    using Src = Pin<wave_ch1.port, wave_ch1.pin>;
    using Dst = Pin<meas_ch2.port, meas_ch2.pin>;
    Src::output(false);
    Dst::input(PinPull::up);
    wait_us(20);
    const bool low_seen = !Dst::read();
    Src::set();
    wait_us(20);
    const bool high_seen = Dst::read();
    Dst::input(PinPull::down);
    Src::clear();
    wait_us(20);
    const bool low_again = !Dst::read();
    Src::release();
    Dst::release();
    return low_seen && high_seen && low_again;
}

void need_jumper() {
    if (!jumper_known) {
        jumper = measure_jumper();
        jumper_known = true;
    }
}

// ===========================================================================
// a - the time base: the gate, the reset state, the counting, the shadows
// ===========================================================================
void ta_time_base() {
    all_off();

    // The gate: closed by hand, then init() opens it and the reset
    // pulse puts every register back to the chapter's own values.
    Rcc::disable(Bus::pb1, rcc_pb1_tim3);
    const bool closed = !Wave::bus_clock();
    Wave::init();
    const bool opened = Wave::bus_clock();
    const bool reset_state = Wave::regs().CTLR1 == 0u && Wave::regs().CCER == 0u &&
                             Wave::period() == 0xFFFFu && Wave::prescaler() == 0u &&
                             Wave::count() == 0u;
    print(serial, "  TIM3 after init(): CTLR1=", hex(Wave::regs().CTLR1),
          " ATRLR=", Wave::period(), " CNT=", Wave::count(), crlf);
    bench.verdict("the gate is closed until init() opens it, and the reset pulse leaves the "
                  "chapter's own register values (ATRLR 0xFFFF, everything else zero)",
                  closed && opened && reset_state);

    // The counter, against the core's own ruler: 1 MHz counting ticks,
    // a tenth of a second measured on the STK.
    (void)Wave::configure({.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                           .period = 0xFFFF});
    Wave::enable(true);
    Stopwatch hundred;
    const uint32_t c0 = Wave::count();
    while (hundred.us() < 100u * 1000u) {
    }
    const uint32_t c1 = Wave::count();
    const uint32_t counted = (c1 - c0) & 0xFFFFu;
    Wave::enable(false);
    // 100 ms of a 1 MHz counter is 100000 ticks, which wraps a 16-bit
    // counter once and a half: what is compared is the modulus.
    const uint32_t expected = (100u * 1000u) & 0xFFFFu;
    print(serial, "  100 ms at 1 MHz: ", counted, " counts of ", expected, " expected (",
          tim_hz / 1'000'000u, " MHz timer clock, prescaler ", Wave::prescaler(), ")", crlf);
    bench.verdict("the prescaler and the counter agree with the core's counter to better "
                  "than a per cent over a tenth of a second",
                  counted + 1000u > expected && counted < expected + 1000u);

    // The update event, counted through its flag.
    (void)Wave::configure({.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                           .period = 999});
    Wave::clear_flags(Wave::all_flags);
    Wave::enable(true);
    uint32_t updates = 0;
    Stopwatch fifty;
    while (fifty.us() < 50u * 1000u) {
        if (Wave::flag(Wave::update_flag)) {
            Wave::clear_flags(Wave::update_flag);
            ++updates;
        }
    }
    Wave::enable(false);
    print(serial, "  50 ms of a 1 kHz update: ", updates, " events", crlf);
    bench.verdict("the update event arrives once per (period + 1) counter ticks - fifty in "
                  "fifty milliseconds",
                  updates >= 49u && updates <= 51u);

    // THE SHADOW REGISTERS. With ARPE set a new period is taken at the
    // next update; without it, at once. The counter is the witness: a
    // period shortened below the current count wraps immediately when
    // the write lands at once, and only at the next update otherwise.
    (void)Wave::configure({.prescaler = 0, .period = 0xFFFF, .auto_reload_preload = true});
    Wave::enable(true);
    wait_us(10);
    (void)Wave::set_period(100);
    const uint32_t reads_back = Wave::period();
    wait_us(2);
    const uint32_t count_after = Wave::count();
    Wave::enable(false);
    print(serial, "  ATRLR written 100 with ARPE set: reads back ", reads_back,
          ", counter at ", count_after, " two microseconds later", crlf);
    bench.verdict("a preloaded auto-reload reads back at once and does NOT take effect "
                  "until the update: the counter is still past the new period",
                  reads_back == 100u && count_after > 100u);

    // And the prescaler, which is shadowed whatever ARPE says (14.2.3).
    (void)Wave::configure({.prescaler = 0, .period = 0xFFFF});
    Wave::enable(true);
    Wave::set_prescaler(static_cast<uint16_t>(ticks_per_us - 1u));
    const uint32_t before = Wave::count();
    wait_us(5);
    const uint32_t after = Wave::count();
    const uint32_t moved = (after - before) & 0xFFFFu;
    Wave::update();
    Wave::clear_flags(Wave::update_flag);
    const uint32_t b2 = Wave::count();
    wait_us(5);
    const uint32_t moved_after = (Wave::count() - b2) & 0xFFFFu;
    Wave::enable(false);
    print(serial, "  PSC written while running: ", moved, " counts in 5 us before the update, ",
          moved_after, " after it", crlf);
    bench.verdict("the prescaler is shadowed too - the counter keeps its old rate until an "
                  "update event loads it, and the new one after",
                  moved > 100u && moved_after <= 12u);
    all_off();
}

// ===========================================================================
// b - PWM, read back and measured
// ===========================================================================

/// Set TIM3 up as a PWM on channel 1 and a capture of its OWN output on
/// channel 2, through the INDIRECT mapping (IC2 on TI1): the input path
/// of a channel is live whatever the other channel's CCyS says, so the
/// timer measures the wave it is making.
bool wave_self_capture(uint16_t period, uint16_t compare) {
    if (!Wave::configure({.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                          .period = period,
                          .auto_reload_preload = true})) {
        return false;
    }
    if (!Wave::output_channel(0, {.mode = TimOutputMode::pwm1, .compare = compare})) {
        return false;
    }
    if (!Wave::capture_channel(1, {.select = TimChannelSelect::indirect,
                                   .polarity = TimCapturePolarity::falling})) {
        return false;
    }
    WavePad::claim();
    // THE COMPARE IS PRELOADED: a value written into CHxCVR reaches the
    // shadow register at the next update event and not before, so
    // without this the first period of every new duty would be made
    // with the PREVIOUS one - measured, and the reason the one-pulse
    // task does the same thing (tim.hpp's fact 3).
    Wave::update();
    Wave::clear_flags(Wave::all_flags);
    Wave::enable(true);
    return true;
}

/// The falling-edge capture of the wave above: the high time in
/// microseconds, nothing if no edge arrived.
std::optional<uint32_t> wave_high_time_us() {
    Stopwatch w;
    while (w.us() < 50u * 1000u) {
        if (Wave::flag(Wave::compare_flag(1))) {
            return Wave::compare(1);
        }
    }
    return std::nullopt;
}

void tb_pwm() {
    all_off();
    Wave::init();
    (void)Wave::remap(0);

    // The registers first: the duty a program writes is the compare
    // register, and the frequency is the timer's own.
    const uint16_t period = 999;   // 1 kHz at a 1 MHz counter
    struct Case {
        uint16_t compare;
    };
    constexpr Case cases[] = {{100}, {250}, {500}, {750}};
    uint8_t exact = 0;
    for (const Case& c : cases) {
        if (!wave_self_capture(period, c.compare)) {
            continue;
        }
        const std::optional<uint32_t> high = wave_high_time_us();
        const uint32_t read_back = Wave::compare(0);
        const bool ok = high.has_value() && read_back == c.compare &&
                        *high + 2u >= c.compare && *high <= c.compare + 2u;
        if (ok) {
            ++exact;
        }
        print(serial, "  compare ", c.compare, " us: register ", read_back, ", captured ",
              high.has_value() ? *high : 0u, " us", high.has_value() ? "" : " (no edge)", crlf);
        Wave::enable(false);
    }
    bench.verdict("a channel captures the wave the SAME timer is making, through the "
                  "indirect input mapping, to the microsecond at four duties",
                  exact == 4u);

    // The frequency, through the period register: three rates, each
    // timed against the STK over their own update events.
    constexpr uint16_t periods[] = {199, 999, 4999};
    uint8_t rates_ok = 0;
    for (const uint16_t p : periods) {
        (void)Wave::configure({.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                               .period = p});
        (void)Wave::output_channel(0, {.mode = TimOutputMode::pwm1,
                                       .compare = static_cast<uint16_t>((p + 1u) / 2u)});
        WavePad::claim();
        Wave::clear_flags(Wave::all_flags);
        Wave::enable(true);
        // Twenty periods, measured on the core's counter.
        Stopwatch w;
        uint32_t seen = 0;
        while (seen < 20u && w.us() < 200u * 1000u) {
            if (Wave::flag(Wave::update_flag)) {
                Wave::clear_flags(Wave::update_flag);
                ++seen;
            }
        }
        const uint32_t span = w.us();
        Wave::enable(false);
        const uint32_t want = 20u * (static_cast<uint32_t>(p) + 1u);
        const bool ok = seen == 20u && span + want / 100u > want && span < want + want / 100u;
        if (ok) {
            ++rates_ok;
        }
        print(serial, "  period ", p + 1, " us: twenty of them in ", span, " us (", want,
              " asked)", crlf);
    }
    bench.verdict("the PWM frequency is the auto-reload's own, within a per cent of the "
                  "core's counter at three rates",
                  rates_ok == 3u);

    // The jumper, if it is there: a SECOND timer measures the first's
    // wave. TIM2's channel 2 is the pad the strap lands on, so the PWM
    // input arrangement is the mirrored one - the direct capture on TI2
    // with the slave resetting the counter on it, and the indirect one
    // (IC1 on TI2) taking the falling edge.
    need_jumper();
    if (!jumper) {
        print(serial, "  no jumper (PA6 to PA1): the second timer's measurement is skipped",
              crlf);
        bench.verdict("the jumper's letters are skipped and say so", true);
    } else {
        // The jumper's test released every timer of this suite, TIM3's
        // gate with it: the wave is brought up again before it is made.
        Wave::init();
        (void)Wave::remap(0);
        (void)wave_self_capture(999, 300);
        Meas::init();
        (void)Meas::remap(0);
        (void)Meas::configure({.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                               .period = 0xFFFF});
        (void)Meas::capture_channel(1, {.select = TimChannelSelect::direct,
                                        .polarity = TimCapturePolarity::rising});
        (void)Meas::capture_channel(0, {.select = TimChannelSelect::indirect,
                                        .polarity = TimCapturePolarity::falling});
        (void)Meas::slave({.mode = TimSlaveMode::reset, .trigger = TimTrigger::ti2});
        MeasPad::claim_input();
        Meas::clear_flags(Meas::all_flags);
        Meas::enable(true);
        wait_us(5000);
        const uint32_t measured_period = Meas::compare(1);
        const uint32_t measured_width = Meas::compare(0);
        print(serial, "  over the jumper: period ", measured_period, " us, high ",
              measured_width, " us (1000 and 300 asked)", crlf);
        bench.verdict("a second timer measures the first one's wave over the jumper - the "
                      "period and the high time, both to a microsecond",
                      measured_period >= 999u && measured_period <= 1001u &&
                          measured_width >= 299u && measured_width <= 301u);
    }
    all_off();
}

// ===========================================================================
// c - the complementary pair and its dead band
// ===========================================================================
void tc_pair() {
    all_off();
    Adv::init();
    (void)Adv::remap(0);

    // The DTG ladder, in all four of its ranges, read back off the
    // silicon rather than out of the constexpr table.
    constexpr uint8_t codes[] = {0x20, 0x8F, 0xCF, 0xFF};
    uint8_t ladder = 0;
    for (const uint8_t code : codes) {
        (void)Adv::break_dead_time({.dead_time = code});
        if (Adv::dead_time_ticks() == tim_dead_time_ticks(code)) {
            ++ladder;
        }
    }
    print(serial, "  DTG 0x20/0x8F/0xCF/0xFF -> ", tim_dead_time_ticks(0x20), "/",
          tim_dead_time_ticks(0x8F), "/", tim_dead_time_ticks(0xCF), "/",
          tim_dead_time_ticks(0xFF), " tDTS ticks", crlf);
    bench.verdict("the dead-time register reads back through all four ranges of 14.4.18's "
                  "encoding", ladder == 4u);

    // The pair itself, with a dead time big enough to SAMPLE: tDTS is
    // the timer clock divided by CKD, and the largest code is 1008 of
    // them - about 28 us with CKD at four.
    constexpr uint8_t dtg = 0xFF;
    constexpr uint16_t period = 999;    // 1 ms at a 1 MHz counter
    (void)Adv::configure({.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                          .period = period,
                          .clock_division = TimClockDivision::div4,
                          .auto_reload_preload = true});
    (void)Adv::output_channel(0, {.mode = TimOutputMode::pwm1,
                                  .compare = 500,
                                  .complementary_enable = true});
    (void)Adv::break_dead_time({.dead_time = dtg, .main_output_enable = true});
    AdvPad::claim();
    AdvPadN::claim();
    Adv::update();                 // the preloaded compare into its shadow
    Adv::enable(true);

    // THE DEAD BAND IS THE GAP ITSELF, and the core's counter is fast
    // enough to time it: the two pads are polled until the first falls,
    // then until the other rises. What that measures is one edge of the
    // band; the high times of the two outputs are measured the same
    // way, and together they say what the pair is doing.
    const auto wait_for = [](auto read, bool level, uint32_t limit_us) -> uint32_t {
        Stopwatch w;
        while (read() != level) {
            if (w.us() > limit_us) {
                return 0;
            }
        }
        return w.cycles();
    };
    // Start from a falling edge of the main output, so the band that
    // follows is the one being timed.
    (void)wait_for([] { return AdvPad::read(); }, true, 4000);
    (void)wait_for([] { return AdvPad::read(); }, false, 4000);
    Stopwatch band;
    while (!AdvPadN::read() && band.us() < 4000u) {
    }
    const uint32_t gap_ns = band.cycles() * 1000u / ticks_per_us;
    // And the two high times: a pad is timed from its RISING edge, so
    // each measurement waits the pad low, then high, then times the
    // fall.
    const auto high_time_us = [&wait_for](auto read) -> uint32_t {
        (void)wait_for(read, false, 4000);
        (void)wait_for(read, true, 4000);
        return wait_for(read, false, 4000) / ticks_per_us;
    };
    const uint32_t a_high = high_time_us([] { return AdvPad::read(); });
    const uint32_t n_high = high_time_us([] { return AdvPadN::read(); });

    // And a census of the levels, which is what says they are never
    // both high: the two pads are on two ports, so a sample is two
    // reads a few tens of nanoseconds apart.
    uint32_t both_low = 0;
    uint32_t both_high = 0;
    uint32_t high_a = 0;
    uint32_t high_n = 0;
    constexpr uint32_t samples = 20000;
    for (uint32_t i = 0; i < samples; ++i) {
        const bool a = AdvPad::read();
        const bool nn = AdvPadN::read();
        if (a && nn) {
            ++both_high;
        } else if (!a && !nn) {
            ++both_low;
        } else if (a) {
            ++high_a;
        } else {
            ++high_n;
        }
    }
    Adv::enable(false);
    (void)Adv::main_output(false);
    const uint32_t dead_ns = tim_dead_time_ticks(dtg) * 4u * 1000u / ticks_per_us;  // CKD = /4
    print(serial, "  ", samples, " samples of PA8 and PB13: ", high_a, " / ", high_n,
          " one high, ", both_low, " both low, ", both_high, " both high", crlf);
    print(serial, "  the gap between the outputs: ", gap_ns, " ns measured, ", dead_ns,
          " ns asked (DTG ", hex(dtg), ", tDTS = the timer clock over four)", crlf);
    print(serial, "  their high times: PA8 ", a_high, " us, PB13 ", n_high, " us of a ",
          period + 1, " us period at half duty", crlf);
    bench.verdict("the complementary outputs are never both high, both are driven, and each "
                  "is high for its half of the period less the dead band",
                  both_high == 0u && high_a > samples / 8u && high_n > samples / 8u &&
                      a_high + 40u > 500u - dead_ns / 1000u && a_high < 500u &&
                      n_high + 40u > 500u - dead_ns / 1000u && n_high < 500u);
    bench.verdict("and the dead band between them is the one the DTG code asks for, to a "
                  "tenth of it - a gap measured with its own measurer",
                  gap_ns * 10u > dead_ns * 9u && gap_ns * 9u < dead_ns * 10u);

    // The idle levels and the off-state bits, read back: what the pads
    // do when the outputs are disabled is BDTR's and CTLR2's business.
    (void)Adv::output_channel(0, {.mode = TimOutputMode::pwm1,
                                  .compare = 500,
                                  .complementary_enable = true,
                                  .idle_high = true,
                                  .complementary_idle_high = false});
    (void)Adv::break_dead_time({.dead_time = dtg,
                                .main_output_enable = false,
                                .off_state_run = true,
                                .off_state_idle = true});
    const uint16_t ctlr2 = Adv::regs().CTLR2;
    const uint16_t bdtr = Adv::regs().BDTR;
    print(serial, "  CTLR2=", hex(ctlr2), " BDTR=", hex(bdtr), crlf);
    bench.verdict("the idle level of the output and of its complement, OSSI and OSSR all "
                  "read back as written, with MOE clear",
                  (ctlr2 & (1u << 8)) != 0u && (ctlr2 & (1u << 9)) == 0u &&
                      (bdtr & tim_ossi) != 0u && (bdtr & tim_ossr) != 0u &&
                      (bdtr & tim_moe) == 0u);
    all_off();
}

// ===========================================================================
// d - the meters, behind a MeterLatch
// ===========================================================================
void td_meters() {
    all_off();
    Wave::init();
    (void)Wave::remap(0);

    // The INTERVAL meter: channel 2 captures the rising edges of TI1
    // and the capture body hands each interval to a MeterLatch. THE
    // COUNTER MUST BE FREE-RUNNING and longer than what it measures -
    // a counter whose own period is the wave's captures every edge at
    // the same count and reads an interval of zero, measured - so the
    // wave is the CPU's here, written through channel 1's output stage
    // while the counter runs to its full sixteen bits.
    (void)Wave::configure({.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                           .period = 0xFFFF});
    (void)Wave::output_channel(0, {.mode = TimOutputMode::force_inactive, .preload = false});
    (void)Wave::capture_channel(1, {.select = TimChannelSelect::indirect,
                                    .polarity = TimCapturePolarity::rising});
    WavePad::claim();
    Interval::restart();
    Wave::clear_flags(Wave::all_flags);
    interval_arm = true;
    Wave::interrupts(Wave::compare_interrupt(1), true);
    Pfic::enable(Irq::tim3);
    Wave::enable(true);
    for (uint8_t i = 0; i < 6u; ++i) {
        (void)Wave::output_mode(0, TimOutputMode::force_active);
        wait_us(200);
        (void)Wave::output_mode(0, TimOutputMode::force_inactive);
        wait_us(800);
    }
    Wave::interrupts(Wave::compare_interrupt(1), false);
    Pfic::disable(Irq::tim3);
    Wave::enable(false);
    const std::optional<uint32_t> interval = IntervalLatch::take();
    const uint16_t missed = IntervalLatch::missed();
    print(serial, "  the interval meter over six edges 1000 us apart: last interval ",
          interval.has_value() ? *interval : 0u, " us, ", missed,
          " readings overwritten before the loop took them", crlf);
    bench.verdict("consecutive rising edges are one interval apart to within a per cent, "
                  "and the latch counted every reading the loop did not take",
                  interval.has_value() && *interval >= 990u && *interval <= 1010u &&
                      missed >= 3u);

    // A STALE SOURCE IS SILENT (design/meters.md): the latch was
    // emptied by that take(), and with the timer stopped the next one
    // has nothing to say.
    const bool silent = !IntervalLatch::take().has_value();
    bench.verdict("a second take() with no new capture answers nothing - the meter's own "
                  "doctrine, a stale source publishes nothing", silent);

    // The PERIOD meter: TimPeriodMeter watches TI1 with two channels,
    // and the waveform is made by the CPU through channel 3's output
    // stage, with CTLR2.TI1S turning TI1 into the XOR of the first
    // three channel inputs - so the driver and the two captures live on
    // one timer with no wire at all.
    all_off();
    Quad::init();
    (void)Quad::remap(0);
    QuadA::claim_input(PinPull::down);
    QuadB::claim_input(PinPull::down);
    QuadC::claim();
    if (!Period::setup(static_cast<uint16_t>(ticks_per_us - 1u))) {
        bench.verdict("the period meter was configured", false);
        all_off();
        return;
    }
    (void)Quad::ti1_xor(true);
    (void)Quad::output_channel(2, {.mode = TimOutputMode::force_inactive, .preload = false});
    Quad::clear_flags(Quad::all_flags);
    period_arm = true;
    Quad::interrupts(Quad::compare_interrupt(0) | Quad::compare_interrupt(1), true);
    Pfic::enable(Irq::tim4);
    // Five square waves of 1000 us, 300 of them high, written by hand.
    for (uint8_t i = 0; i < 5u; ++i) {
        (void)Quad::output_mode(2, TimOutputMode::force_active);
        wait_us(300);
        (void)Quad::output_mode(2, TimOutputMode::force_inactive);
        wait_us(700);
    }
    Quad::interrupts(Quad::compare_interrupt(0) | Quad::compare_interrupt(1), false);
    Pfic::disable(Irq::tim4);
    const std::optional<uint32_t> per = PeriodLatch::take();
    const std::optional<uint32_t> wid = WidthLatch::take();
    print(serial, "  the period meter on the TI1 XOR, the CPU making the wave: period ",
          per.has_value() ? *per : 0u, " us, high ", wid.has_value() ? *wid : 0u,
          " us (1000 and 300 asked)", crlf);
    bench.verdict("PWM input mode measures the period and the high time of a wave the "
                  "timer's own output stage puts on the pad, within two per cent",
                  per.has_value() && wid.has_value() && *per >= 980u && *per <= 1030u &&
                      *wid >= 295u && *wid <= 320u);
    all_off();
}

// ===========================================================================
// e - the counters, with no pad at all
// ===========================================================================
void te_counters() {
    all_off();
    Wave::init();
    Meas::init();

    // TIM2's ITR2 is TIM3 (table 15-2), which the driver's own fold
    // says - a caller names the MASTER and gets the index.
    constexpr uint8_t itr = tim_trigger_index_for(2, 3);
    static_assert(itr == 2, "TIM3 reaches TIM2 on ITR2");
    print(serial, "  TIM3 reaches TIM2 on ITR", itr, ", and TIM2 reaches TIM3 on ITR",
          tim_trigger_index_for(3, 2), crlf);

    // THE EVENT COUNTER: the slave counts the master's UPDATE events.
    // The slave is configured and started FIRST - a slave whose clock
    // is not running does not see the trigger that arrives while it is
    // not.
    (void)TimEventCounter<Meas>::setup(static_cast<TimTrigger>(itr));
    TimEventCounter<Meas>::restart();
    (void)Wave::configure({.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                           .period = 99});   // 10 kHz of updates
    (void)Wave::master(TimMasterMode::update);
    Wave::enable(true);
    wait_us(20000);
    Wave::enable(false);
    const uint32_t counted = TimEventCounter<Meas>::count();
    print(serial, "  20 ms of a 10 kHz update on TRGO: the slave counted ", counted, crlf);
    bench.verdict("external clock mode 1 over an internal trigger counts the master's "
                  "update events exactly - two hundred of them, no pad in the path",
                  counted >= 199u && counted <= 201u);

    // THE GATED COUNTER: the master publishes its channel-1 waveform
    // on TRGO and the slave counts ITS OWN clock while that is high -
    // which makes the count a DUTY CYCLE, measured internally.
    all_off();
    Wave::init();
    Meas::init();
    (void)Wave::configure({.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                           .period = 999,
                           .auto_reload_preload = true});
    (void)Wave::output_channel(0, {.mode = TimOutputMode::pwm1, .compare = 250});
    // The compare is preloaded here too: without this the master's
    // FIRST period carries the shadow register's zero and the gate
    // measures one period short of the truth (measured).
    Wave::update();
    (void)Wave::master(TimMasterMode::oc1ref);
    (void)TimGatedCounter<Meas>::setup(static_cast<TimTrigger>(itr),
                                       static_cast<uint16_t>(ticks_per_us - 1u));
    TimGatedCounter<Meas>::restart();
    Wave::clear_flags(Wave::all_flags);
    Wave::enable(true);
    // EXACTLY twenty periods, counted on the master's own update flag:
    // a window measured in microseconds would cut the last high phase
    // in half and read a whole period short, measured.
    uint32_t periods = 0;
    Stopwatch window;
    while (periods < 20u && window.us() < 100u * 1000u) {
        if (Wave::flag(Wave::update_flag)) {
            Wave::clear_flags(Wave::update_flag);
            ++periods;
        }
    }
    Wave::enable(false);
    const uint32_t high_us = TimGatedCounter<Meas>::count();
    print(serial, "  twenty periods of a 25 % duty: the gated counter ran for ", high_us,
          " us of them (5000 expected)", crlf);
    bench.verdict("gated mode measures a duty cycle from the master's own waveform, within "
                  "a per cent and with nothing on a pad",
                  periods == 20u && high_us >= 4975u && high_us <= 5060u);
    all_off();
}

// ===========================================================================
// f - the periodic tick, the one pulse, the encoder
// ===========================================================================
void tf_tasks() {
    all_off();
    Wave::init();

    // The periodic tick, against the STK.
    (void)TimPeriodicTick<Wave>::setup(static_cast<uint16_t>(ticks_per_us - 1u), 499, false);
    Wave::clear_flags(Wave::all_flags);
    uint32_t ticks = 0;
    Stopwatch w;
    while (w.us() < 50u * 1000u) {
        if (Wave::flag(Wave::update_flag)) {
            Wave::clear_flags(Wave::update_flag);
            ++ticks;
        }
    }
    TimPeriodicTick<Wave>::stop();
    print(serial, "  a 500 us periodic tick over 50 ms: ", ticks, " of them", crlf);
    bench.verdict("the periodic tick is the period the caller asked for, counted against "
                  "the core's own counter", ticks >= 99u && ticks <= 101u);

    // THE ONE PULSE, whose width is the thing to prove: the compare
    // register is PRELOADED, and a one-pulse timer sees no update
    // between its configuration and its trigger - so a setup that did
    // not load the shadow would make a pulse the whole period wide
    // instead of the width asked. The pad is timed by the core's
    // counter, which is the only ruler that is not the timer itself.
    all_off();
    Wave::init();
    (void)Wave::remap(0);
    constexpr uint32_t delay_us_asked = 200;
    constexpr uint32_t width_asked = 500;
    (void)TimOnePulse<Wave, 0>::setup(static_cast<uint16_t>(ticks_per_us - 1u),
                                      delay_us_asked, width_asked);
    WavePad::claim();
    Stopwatch pulse;
    TimOnePulse<Wave, 0>::fire();
    while (!WavePad::read() && pulse.us() < 5000u) {
    }
    const uint32_t rose = pulse.cycles();
    while (WavePad::read() && pulse.us() < 5000u) {
    }
    const uint32_t fell = pulse.cycles();
    const uint32_t start_us = rose / ticks_per_us;
    const uint32_t width_us = (fell - rose) / ticks_per_us;
    print(serial, "  the pulse rose ", start_us, " us after the trigger and lasted ",
          width_us, " us (", delay_us_asked, " and ", width_asked, " asked)", crlf);
    bench.verdict("one pulse of the width asked, after the delay asked - the preloaded "
                  "compare register LOADED by the setup, not left at its old value",
                  width_us + 5u >= width_asked && width_us <= width_asked + 5u &&
                      start_us + 5u >= delay_us_asked && start_us <= delay_us_asked + 5u);
    const bool stopped = !Wave::enabled();
    bench.verdict("and the counter stopped itself at the update that ended the pulse",
                  stopped);

    // THE ENCODER, on pads the timer drives itself: both channels stay
    // inputs for the quadrature interface, and the CPU writes their
    // output stages through the forced modes.
    all_off();
    Quad::init();
    (void)Quad::remap(0);
    QuadA::claim();
    QuadB::claim();
    // WHERE THE PAD RESTS on its own: an input with its own pull-down
    // reads high only if something outside the chip holds it - a board
    // that puts a pull-up on PB6 (an I2C strap) - and the question below
    // is then asked the other way round: whether the stage can pull the
    // pad OFF its resting level.
    QuadA::claim_input(PinPull::down);
    wait_us(20);
    const bool rests_high = QuadA::read();
    QuadA::claim();
    const TimOutputMode away = rests_high ? TimOutputMode::force_inactive
                                          : TimOutputMode::force_active;
    const TimOutputMode home = rests_high ? TimOutputMode::force_active
                                          : TimOutputMode::force_inactive;
    // First, the stimulus on its own: a channel in forced output mode
    // with the timer in its ORDINARY mode, so that what the encoder
    // arrangement changes is the only thing left to explain.
    (void)Quad::configure({.prescaler = 0, .period = 0xFFFF});
    (void)Quad::output_channel(0, {.mode = TimOutputMode::force_inactive,
                                   .preload = false});
    Quad::enable(true);
    (void)Quad::output_mode(0, TimOutputMode::force_active);
    wait_us(2);
    const bool plain_high = QuadA::read();
    (void)Quad::output_mode(0, TimOutputMode::force_inactive);
    wait_us(2);
    const bool plain_low = !QuadA::read();
    print(serial, "  PB6 rests ", rests_high ? "HIGH (a pull-up outside the chip)" : "low",
          "; outside encoder mode, the forced output stage drives it: ",
          plain_high ? "high" : "LOW", " and ", plain_low ? "low" : "HIGH", crlf);
    bench.verdict("a channel's forced output mode puts a level on its pad - the stimulus "
                  "every wireless capture in this suite rests on",
                  plain_high && plain_low);
    (void)Quad::output_mode(0, home);
    Quad::enable(false);
    if (!TimEncoder<Quad>::setup({.mode = TimSlaveMode::encoder3}, 0xFFFF)) {
        bench.verdict("the encoder was configured", false);
        all_off();
        return;
    }
    // AND NOW THE SAME STIMULUS INSIDE THE ENCODER ARRANGEMENT, which
    // is where this family parts company with its relatives: with the
    // slave controller in an encoder mode the channel's output stage
    // no longer reaches the pad, whatever CCER and CHCTLRx say. The
    // two measurements differ in SMS and in nothing else, and the stage
    // is asked to take the pad AWAY from where it rests.
    (void)Quad::output_channel(0, {.mode = home, .preload = false});
    (void)Quad::output_channel(1, {.mode = TimOutputMode::force_inactive,
                                   .preload = false});
    wait_us(2);
    (void)Quad::output_mode(0, away);
    wait_us(2);
    const bool moved = QuadA::read() != rests_high;
    (void)Quad::output_mode(0, home);
    print(serial, "  in encoder mode the same forced output leaves PB6 ",
          moved ? "MOVED" : "where it rests", " (SMCFGR=", hex(Quad::regs().SMCFGR),
          " CHCTLR1=", hex(Quad::regs().CHCTLR1), " CCER=", hex(Quad::regs().CCER), ")", crlf);
    bench.verdict("with the slave controller in an encoder mode a channel's output stage "
                  "does NOT reach its pad - the same registers that drove it a moment ago",
                  !moved);

    // So the quadrature is written by the PORT, which this family's
    // input path does see (letter j): the pads go back to plain
    // outputs and the encoder interface reads them through TI1 and TI2.
    QuadA::release();
    QuadB::release();
    using PadA = Pin<quad_a.port, quad_a.pin>;
    using PadB = Pin<quad_b.port, quad_b.pin>;
    PadA::output(false);
    PadB::output(false);
    wait_us(2);
    Quad::set_count(0x8000);
    // EACH EDGE IS HELD: two stores in a row are one bus cycle (the
    // trap this stratum's README names), so the quadrature is written
    // at a rate an encoder could actually turn at.
    const auto edge = [](auto pad, bool level) {
        pad(level);
        wait_us(2);
    };
    const auto set_a = [](bool level) {
        if (level) { PadA::set(); } else { PadA::clear(); }
    };
    const auto set_b = [](bool level) {
        if (level) { PadB::set(); } else { PadB::clear(); }
    };
    const auto quadrature = [&](bool forward) {
        for (uint8_t i = 0; i < 10u; ++i) {
            if (forward) {
                edge(set_a, true);
                edge(set_b, true);
                edge(set_a, false);
                edge(set_b, false);
            } else {
                edge(set_b, true);
                edge(set_a, true);
                edge(set_b, false);
                edge(set_a, false);
            }
        }
    };
    quadrature(true);
    const uint32_t forward = Quad::count();
    const bool up = !TimEncoder<Quad>::reversing();
    quadrature(false);
    const uint32_t back = Quad::count();
    const bool down = TimEncoder<Quad>::reversing();
    PadA::release();
    PadB::release();
    print(serial, "  ten quadrature cycles forward: ", forward - 0x8000u,
          " counts, then ten back: ", static_cast<int32_t>(back - forward), crlf);
    bench.verdict("encoder mode 3 counts four times per quadrature cycle, both ways, and "
                  "CTLR1.DIR follows the shaft",
                  forward == 0x8000u + 40u && back == 0x8000u && up && down);
    all_off();
}

// ===========================================================================
// g - the break input
// ===========================================================================
void tg_break() {
    all_off();
    Adv::init();
    (void)Adv::remap(0);
    (void)Adv::configure({.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                          .period = 999,
                          .auto_reload_preload = true});
    (void)Adv::output_channel(0, {.mode = TimOutputMode::pwm1, .compare = 500});
    (void)Adv::break_dead_time({.main_output_enable = true});
    AdvPad::claim();
    Adv::enable(true);
    wait_us(2000);
    const bool driving = Adv::main_output();

    // The software break: SWEVGR.BG is the one a board with no wire can
    // raise, and what it does is exactly what a pad would.
    Adv::clear_flags(Adv::all_flags);
    (void)Adv::break_event();
    wait_us(10);
    const bool moe_cleared = !Adv::main_output();
    const bool bif = Adv::flag(Adv::break_flag);
    print(serial, "  the software break: MOE ", driving ? "was set" : "was NOT set",
          ", now ", moe_cleared ? "clear" : "STILL SET", ", BIF ", bif ? "raised" : "absent",
          crlf);
    bench.verdict("a break clears the master output enable asynchronously and raises its "
                  "own flag", driving && moe_cleared && bif);

    // AUTOMATIC OUTPUT ENABLE: with AOE set the outputs come back at
    // the next update event, and only then.
    (void)Adv::break_dead_time({.main_output_enable = true, .automatic_output_enable = true});
    Adv::clear_flags(Adv::all_flags);
    (void)Adv::break_event();
    const bool cleared_again = !Adv::main_output();
    wait_us(3000);   // three update periods
    const bool came_back = Adv::main_output();
    print(serial, "  with AOE: MOE ", cleared_again ? "cleared" : "NOT cleared",
          " by the break and ", came_back ? "back" : "STILL clear", " three periods later",
          crlf);
    bench.verdict("the automatic output enable brings MOE back at the next update, which is "
                  "what makes a break a cycle and not a latch",
                  cleared_again && came_back);

    // The BKIN PAD, driven by the port itself. Whether an output pad
    // reaches a peripheral's input is the question letter j asks of a
    // capture; here it is asked of the break input, which is the same
    // multiplexer.
    //
    // FIRST, WHETHER THE PAD IS THIS BOARD'S TO DRIVE: the break input
    // is PB12, which is also a select line the moment a second board is
    // strapped to it, and a peer holding it high would break the outputs
    // before this test wrote a bit. An input with its own pull-down that
    // still reads high is a pad something else owns, and the measurement
    // is declined rather than claimed.
    using BkinProbe = Pin<adv_bkin.port, adv_bkin.pin>;
    BkinProbe::input(PinPull::down);
    wait_us(20);
    const bool bkin_held = BkinProbe::read();
    BkinProbe::release();
    if (bkin_held) {
        print(serial, "  SKIPPED, no verdict claimed: the break pad PB12 reads HIGH against "
                      "its own pull-down, so something outside this board holds it - a "
                      "link's select line, today. The software break above is what this "
                      "board can measure alone.",
              crlf);
        all_off();
        return;
    }
    (void)Adv::break_dead_time({.main_output_enable = true,
                                .break_enable = true,
                                .break_active_high = true});
    // BKE and BKP need one bus period before they read back (14.4.18).
    wait_us(2);
    const uint16_t bdtr_before = Adv::regs().BDTR;
    using BkinPin = Pin<adv_bkin.port, adv_bkin.pin>;
    BkinPin::output(false);
    wait_us(10);
    const bool armed = Adv::main_output();
    BkinPin::set();
    wait_us(50);
    const bool pad_broke = !Adv::main_output();
    BkinPin::clear();
    BkinPin::release();
    print(serial, "  BDTR with BKE and BKP: ", hex(bdtr_before), "; the pad driven high ",
          pad_broke ? "DID" : "did NOT", " break the outputs", crlf);
    bench.verdict("the break enable and its polarity read back after one bus period",
                  (bdtr_before & tim_bke) != 0u && (bdtr_before & tim_bkp) != 0u && armed);
    bench.verdict("and the break input reached the timer from a pad the PORT was driving - "
                  "this family's alternate-function input is the pad's own input buffer",
                  pad_broke);
    all_off();
}

// ===========================================================================
// h - the trigger chains
// ===========================================================================
void th_chains() {
    all_off();
    Wave::init();
    Meas::init();
    constexpr uint8_t itr = tim_trigger_index_for(2, 3);

    // RESET MODE: every trigger reinitializes the slave's counter, so
    // its count is never far from zero and its update event never
    // arrives.
    (void)Meas::configure({.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                           .period = 0xFFFF});
    (void)Meas::slave({.mode = TimSlaveMode::reset, .trigger = static_cast<TimTrigger>(itr)});
    Meas::clear_flags(Meas::all_flags);
    Meas::enable(true);
    (void)Wave::configure({.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                           .period = 199});
    (void)Wave::master(TimMasterMode::update);
    Wave::enable(true);
    wait_us(5000);
    const uint32_t held = Meas::count();
    const bool triggered = Meas::flag(Meas::trigger_flag);
    Wave::enable(false);
    Meas::enable(false);
    print(serial, "  reset mode: the slave's counter sat at ", held,
          " after 5 ms of 200 us triggers, TIF ", triggered ? "raised" : "absent", crlf);
    bench.verdict("a trigger in reset mode reinitializes the slave's counter, which "
                  "therefore never passes one trigger period",
                  held <= 205u && triggered);

    // TRIGGER MODE: the slave starts on the first trigger and runs on
    // by itself - CEN set by hardware, which is what the readback says.
    all_off();
    Wave::init();
    Meas::init();
    (void)Meas::configure({.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                           .period = 0xFFFF});
    (void)Meas::slave({.mode = TimSlaveMode::trigger, .trigger = static_cast<TimTrigger>(itr)});
    const bool idle_before = !Meas::enabled() && Meas::count() == 0u;
    (void)Wave::configure({.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                           .period = 999});
    (void)Wave::master(TimMasterMode::update);
    Wave::enable(true);
    wait_us(3000);
    Wave::enable(false);
    const bool running = Meas::enabled();
    const uint32_t ran = Meas::count();
    print(serial, "  trigger mode: the slave was ", idle_before ? "idle" : "NOT idle",
          " before the master and had counted ", ran, " us after 3 ms of it", crlf);
    bench.verdict("a trigger STARTS the slave (CEN set by hardware) and does not reset it, "
                  "so it counts on from the first trigger",
                  idle_before && running && ran >= 1900u && ran <= 3100u);

    // GATED MODE on the master's ENABLE signal: the slave runs exactly
    // while the master does.
    all_off();
    Wave::init();
    Meas::init();
    (void)Meas::configure({.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                           .period = 0xFFFF});
    (void)Meas::slave({.mode = TimSlaveMode::gated, .trigger = static_cast<TimTrigger>(itr)});
    Meas::enable(true);
    (void)Wave::configure({.prescaler = 0, .period = 0xFFFF});
    (void)Wave::master(TimMasterMode::enable);
    const uint32_t before = Meas::count();
    Wave::enable(true);
    wait_us(2000);
    Wave::enable(false);
    const uint32_t after = Wave::enabled() ? 0u : Meas::count();
    wait_us(2000);
    const uint32_t later = Meas::count();
    print(serial, "  gated on the master's enable: the slave counted ", after - before,
          " us while the master ran and ", later - after, " us after it stopped", crlf);
    bench.verdict("the slave counts its own clock only while the master's enable signal is "
                  "high - a window opened and closed with no pad",
                  after - before >= 1900u && after - before <= 2100u && later == after);
    all_off();
}

// ===========================================================================
// i - the vectors
// ===========================================================================
void ti_vectors() {
    all_off();
    Adv::init();

    // The advanced timer's four lines, each with its own body. The
    // update and the compare are raised by the counter; the trigger and
    // the break by software events.
    (void)Adv::configure({.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                          .period = 99});
    (void)Adv::output_channel(0, {.mode = TimOutputMode::pwm1, .compare = 50, .enable = false});
    (void)Adv::slave({.mode = TimSlaveMode::disabled, .trigger = TimTrigger::itr1});
    Adv::clear_flags(Adv::all_flags);
    Adv::interrupts(Adv::update_interrupt | Adv::compare_interrupt(0) |
                        Adv::trigger_interrupt | Adv::break_interrupt,
                    true);
    Pfic::enable(Irq::tim1_up);
    Pfic::enable(Irq::tim1_cc);
    Pfic::enable(Irq::tim1_brk);
    Pfic::enable(Irq::tim1_trg_com);
    Adv::enable(true);
    wait_us(2000);            // twenty periods of 100 us
    Adv::enable(false);
    (void)Adv::trigger_event();
    wait_us(10);
    (void)Adv::break_event();
    wait_us(10);
    Adv::interrupts(Adv::update_interrupt | Adv::compare_interrupt(0) |
                        Adv::trigger_interrupt | Adv::break_interrupt,
                    false);
    Pfic::disable(Irq::tim1_up);
    Pfic::disable(Irq::tim1_cc);
    Pfic::disable(Irq::tim1_brk);
    Pfic::disable(Irq::tim1_trg_com);
    print(serial, "  TIM1's four lines: ", up_calls, " update, ", cc_calls, " compare, ",
          trg_calls, " trigger, ", brk_calls, " break (masks ", hex(cc_mask), " / ",
          hex(brk_mask), ")", crlf);
    bench.verdict("each of the advanced timer's four vectors ran for its own events - "
                  "twenty updates and twenty compares on two different lines, one trigger "
                  "and one break",
                  up_calls >= 19u && up_calls <= 21u && cc_calls >= 19u && cc_calls <= 21u &&
                      trg_calls == 1u && brk_calls == 1u);
    bench.verdict("and each body saw only the flags ITS vector answers for",
                  cc_mask == Adv::compare_flag(0) && brk_mask == Adv::break_flag);

    // A general-purpose timer has ONE line for everything.
    all_off();
    Wave::init();
    (void)Wave::configure({.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                           .period = 99});
    (void)Wave::output_channel(0, {.mode = TimOutputMode::pwm1, .compare = 50,
                                   .enable = false});
    Wave::clear_flags(Wave::all_flags);
    Wave::interrupts(Wave::update_interrupt | Wave::compare_interrupt(0), true);
    Pfic::enable(Irq::tim3);
    Wave::enable(true);
    wait_us(1000);
    Wave::enable(false);
    Wave::interrupts(Wave::update_interrupt | Wave::compare_interrupt(0), false);
    Pfic::disable(Irq::tim3);
    print(serial, "  TIM3's one line ran ", wave_calls, " times for ten periods of two "
                  "events each", crlf);
    bench.verdict("a general-purpose timer's single vector answers for every one of its "
                  "events", wave_calls >= 15u && wave_calls <= 25u);

    // A FLAG WHOSE INTERRUPT IS NOT ENABLED is left standing: isr()
    // returns nothing and clears nothing, which is what makes every
    // polled measurement in this suite possible.
    Wave::clear_flags(Wave::all_flags);
    Wave::enable(true);
    wait_us(300);
    Wave::enable(false);
    const bool standing = Wave::flag(Wave::update_flag);
    const uint16_t served = Wave::isr();
    const bool still_standing = Wave::flag(Wave::update_flag);
    bench.verdict("a flag whose interrupt is disabled stands after isr(), which serves only "
                  "what is enabled", standing && served == 0u && still_standing);
    all_off();
}

// ===========================================================================
// j - the jumper, and what a plain output pad reaches
// ===========================================================================
void tj_jumper() {
    jumper = measure_jumper();
    jumper_known = true;
    print(serial, "  PA6 driven, PA1 read: the jumper is ", jumper ? "THERE" : "absent", crlf);
    bench.verdict("the strap between TIM3's channel 1 and TIM2's channel 2 was tested for "
                  "(present or absent, both legal)", true);

    // Does a pad in PLAIN OUTPUT mode reach a timer's capture input?
    // On the STM32F4 it does not - the alternate-function input
    // multiplexer is opened by MODER there. Here the question is the
    // same and the answer is this family's.
    all_off();
    Wave::init();
    (void)Wave::remap(0);
    (void)Wave::configure({.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                           .period = 0xFFFF});
    (void)Wave::capture_channel(0, {.select = TimChannelSelect::direct,
                                    .polarity = TimCapturePolarity::rising});
    using SrcPin = Pin<wave_ch1.port, wave_ch1.pin>;
    SrcPin::output(false);
    Wave::clear_flags(Wave::all_flags);
    Wave::enable(true);
    uint32_t captures = 0;
    for (uint8_t i = 0; i < 16u; ++i) {
        SrcPin::set();
        wait_us(5);
        SrcPin::clear();
        wait_us(5);
        if (Wave::flag(Wave::compare_flag(0))) {
            ++captures;
            Wave::clear_flags(Wave::compare_flag(0));
        }
    }
    Wave::enable(false);
    SrcPin::release();
    print(serial, "  sixteen edges written on PA6 in plain OUTPUT mode: ", captures,
          " captures", crlf);
    bench.verdict("a pad the PORT drives is seen by the timer's capture input on this "
                  "family - the alternate-function input is the pad's own input buffer, "
                  "not a multiplexer a mode register opens",
                  captures == 16u);
    all_off();
}

// ===========================================================================
// The CH32V303's timers. Every letter below is a TEMPLATE on the timer's
// number, so that a part without the timer never forms the type: an
// `if constexpr` outside a template still instantiates both branches.
// ===========================================================================

/// A strap between two pads, tested the way letter j tests PA6-PA1: the
/// first driven as a plain output, the second read with its own pull set
/// against the level, both ways round.
template <Pad from, Pad to>
bool strap_present() {
    using Src = Pin<from.port, from.pin>;
    using Dst = Pin<to.port, to.pin>;
    Src::output(false);
    Dst::input(PinPull::up);
    wait_us(20);
    const bool low_seen = !Dst::read();
    Src::set();
    Dst::input(PinPull::down);
    wait_us(20);
    const bool high_seen = Dst::read();
    Src::release();
    Dst::release();
    return low_seen && high_seen;
}

/// Wait for a flag with a bound, and say whether it came.
template <class T>
bool wait_flag(uint16_t flag, uint32_t limit_us) {
    Stopwatch w;
    while (!T::flag(flag)) {
        if (w.us() > limit_us) {
            return false;
        }
    }
    return true;
}

/// One timer back to reset, where the part has it - a template on the
/// number, so a part without the timer never forms its type.
template <uint8_t N>
void release_if() {
    if constexpr (tim_present(N)) {
        Tim<N>::release();
    }
}

/// One pad back to a floating input, where the package bonds it.
template <Pad pad>
void release_pad() {
    if constexpr (pad_bonded(pad)) {
        TimPad<pad>::release();
    }
}

/// Everything the CH32V303's letters touch, back to reset.
template <uint8_t A = 8>
void v303_off() {
    if constexpr (tim_present(A)) {
        release_if<A>();
        release_if<9>();
        release_if<10>();
        release_if<5>();
        release_if<6>();
        release_if<7>();
        for (const Irq line : {Irq::tim8_brk, Irq::tim8_up, Irq::tim8_trg_com, Irq::tim8_cc,
                               Irq::tim9_up, Irq::tim9_cc, Irq::tim10_up, Irq::tim10_cc,
                               Irq::tim6, Irq::tim7}) {
            Pfic::disable(line);
            Pfic::clear_pending(line);
        }
        release_pad<tim_channel_pad(A, 0, 0)>();
        release_pad<tim_complementary_pad(A, 0, 0)>();
        release_pad<tim_break_pad(A, 0)>();
        release_pad<tim_channel_pad(9, 0, 2)>();
        release_pad<tim_channel_pad(10, 0, 2)>();
    }
}

/// What the CH32V303's vectors counted.
volatile uint32_t v8_up = 0;
volatile uint32_t v8_cc = 0;
volatile uint32_t v8_trg = 0;
volatile uint32_t v8_brk = 0;
volatile uint16_t v8_trg_mask = 0;
volatile uint16_t v8_brk_mask = 0;
/// Letter m's pad break: the break body masks its own interrupt, because
/// a break input held active keeps BIF set - it cannot be cleared then -
/// and the vector would otherwise re-enter until the program starves.
volatile bool v8_brk_self_mask = false;
volatile uint16_t v8_brk_flags_seen = 0;
volatile uint32_t v9_up = 0;
volatile uint32_t v9_cc = 0;
volatile uint32_t v10_up = 0;
volatile uint32_t v10_cc = 0;
volatile uint32_t v6_up = 0;
volatile uint32_t v7_up = 0;

void clear_v303_counts() {
    v8_up = 0;
    v8_cc = 0;
    v8_trg = 0;
    v8_brk = 0;
    v8_trg_mask = 0;
    v8_brk_mask = 0;
    v9_up = 0;
    v9_cc = 0;
    v10_up = 0;
    v10_cc = 0;
    v6_up = 0;
    v7_up = 0;
}

// ---------------------------------------------------------------------------
// k - TIM8's PWM captured by TIM4 over the PC6-PB8 jumper
// ---------------------------------------------------------------------------

struct CaptureCase {
    uint16_t prescaler;
    uint16_t period;     ///< ATRLR: the period is period + 1 counts
    uint16_t compare;    ///< the high time, in counts
};

template <uint8_t A = 8>
void tk_tim8_capture() {
    if constexpr (tim_present(A)) {
        using T8 = Tim<A>;
        constexpr Pad out_pad = tim_channel_pad(A, 0, 0);   // PC6
        constexpr Pad in_pad = tim_channel_pad(4, 0, 2);    // PB8, TIM4's channel 3
        using Out = TimPad<out_pad>;
        using In = TimPad<in_pad>;
        all_off();
        v303_off<A>();
        const bool wired = strap_present<out_pad, in_pad>();
        print(serial, "  PC6 driven, PB8 read: the jumper is ", wired ? "THERE" : "absent", crlf);
        if (!wired) {
            bench.verdict("the PC6-PB8 jumper was tested for (no jumper: TIM8's wave is not "
                          "captured)", true);
            return;
        }
        // Five duties at 1 kHz on a 1 MHz count, then three more rates -
        // the last two on the undivided timer clock, 10 kHz and 100 kHz.
        constexpr CaptureCase cases[] = {
            {143, 999, 100},  {143, 999, 250},    {143, 999, 500}, {143, 999, 750},
            {143, 999, 900},  {143, 499, 125},    {143, 199, 50},  {0, 14399, 3600},
            {0, 1439, 720},
        };
        uint8_t exact = 0;
        for (const CaptureCase& c : cases) {
            T8::init();
            (void)T8::remap(0);
            (void)T8::configure({.prescaler = c.prescaler, .period = c.period,
                                 .auto_reload_preload = true});
            (void)T8::output_channel(0, {.mode = TimOutputMode::pwm1, .compare = c.compare});
            (void)T8::main_output(true);
            T8::update();
            T8::clear_flags(T8::all_flags);
            Out::claim();

            Quad::init();
            (void)Quad::remap(0);
            (void)Quad::configure({.prescaler = c.prescaler, .period = 0xFFFF});
            (void)Quad::capture_channel(2, {.select = TimChannelSelect::direct,
                                            .polarity = TimCapturePolarity::rising});
            (void)Quad::capture_channel(3, {.select = TimChannelSelect::indirect,
                                            .polarity = TimCapturePolarity::falling});
            In::claim_input();
            Quad::clear_flags(Quad::all_flags);
            Quad::enable(true);
            T8::enable(true);
            wait_us(3000);

            // A rising edge, the falling edge after it, the next rising.
            Quad::clear_flags(Quad::all_flags);
            bool ok = wait_flag<Quad>(Quad::compare_flag(2), 20000);
            const uint32_t r1 = Quad::compare(2);
            Quad::clear_flags(Quad::compare_flag(3));
            ok = ok && wait_flag<Quad>(Quad::compare_flag(3), 20000);
            const uint32_t f = Quad::compare(3);
            ok = ok && wait_flag<Quad>(Quad::compare_flag(2), 20000);
            const uint32_t r2 = Quad::compare(2);
            T8::enable(false);
            Quad::enable(false);
            const uint32_t per = (r2 - r1) & 0xFFFFu;
            const uint32_t high = (f - r1) & 0xFFFFu;
            const uint32_t want_per = static_cast<uint32_t>(c.period) + 1u;
            const bool good = ok && per + 1u >= want_per && per <= want_per + 1u &&
                              high + 1u >= c.compare && high <= c.compare + 1u;
            if (good) {
                ++exact;
            }
            print(serial, "  TIM8 at ", SysClock::hz / (static_cast<uint32_t>(c.prescaler) + 1u) /
                                         want_per,
                  " Hz, high ", c.compare, " of ", want_per, " counts: TIM4 read a period of ",
                  per, " and a high time of ", high, ok ? "" : " (an edge did not come)", crlf);
            Out::release();
            In::release();
        }
        bench.verdict("TIM4's capture on PB8 reads TIM8's period and high time to a count, at "
                      "five duties and four frequencies up to 100 kHz",
                      exact == sizeof(cases) / sizeof(cases[0]));
        Quad::release();
        v303_off<A>();
    }
}

// ---------------------------------------------------------------------------
// l - TIM8's complementary pair on PC6 and PA7
// ---------------------------------------------------------------------------

template <uint8_t A = 8>
void tl_tim8_pair() {
    if constexpr (tim_present(A)) {
        using T8 = Tim<A>;
        using Out = TimPad<tim_channel_pad(A, 0, 0)>;          // PC6
        using OutN = TimPad<tim_complementary_pad(A, 0, 0)>;   // PA7
        all_off();
        v303_off<A>();
        T8::init();
        (void)T8::remap(0);

        constexpr uint8_t codes[] = {0x20, 0x8F, 0xCF, 0xFF};
        uint8_t ladder = 0;
        for (const uint8_t code : codes) {
            (void)T8::break_dead_time({.dead_time = code});
            if (T8::dead_time_ticks() == tim_dead_time_ticks(code)) {
                ++ladder;
            }
        }
        bench.verdict("TIM8's dead-time register reads back through all four ranges of "
                      "14.4.18's encoding", ladder == 4u);

        constexpr uint8_t dtg = 0xFF;
        (void)T8::configure({.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                             .period = 999,
                             .clock_division = TimClockDivision::div4,
                             .auto_reload_preload = true});
        (void)T8::output_channel(0, {.mode = TimOutputMode::pwm1, .compare = 500,
                                     .complementary_enable = true});
        (void)T8::break_dead_time({.dead_time = dtg, .main_output_enable = true});
        Out::claim();
        OutN::claim();
        T8::update();
        T8::enable(true);
        const auto wait_for = [](auto read, bool level, uint32_t limit_us) -> uint32_t {
            Stopwatch w;
            while (read() != level) {
                if (w.us() > limit_us) {
                    return 0;
                }
            }
            return w.cycles();
        };
        (void)wait_for([] { return Out::read(); }, true, 4000);
        (void)wait_for([] { return Out::read(); }, false, 4000);
        Stopwatch band;
        while (!OutN::read() && band.us() < 4000u) {
        }
        const uint32_t gap_ns = band.cycles() * 1000u / ticks_per_us;
        uint32_t both_high = 0;
        uint32_t high_a = 0;
        uint32_t high_n = 0;
        constexpr uint32_t samples = 20000;
        for (uint32_t i = 0; i < samples; ++i) {
            const bool a = Out::read();
            const bool nn = OutN::read();
            if (a && nn) {
                ++both_high;
            } else if (a) {
                ++high_a;
            } else if (nn) {
                ++high_n;
            }
        }
        T8::enable(false);
        (void)T8::main_output(false);
        const uint32_t dead_ns = tim_dead_time_ticks(dtg) * 4u * 1000u / ticks_per_us;
        print(serial, "  ", samples, " samples of PC6 and PA7: ", high_a, " / ", high_n,
              " one high, ", both_high, " both high; the gap ", gap_ns, " ns measured, ", dead_ns,
              " ns asked (DTG ", hex(dtg), ", tDTS = the timer clock over four)", crlf);
        bench.verdict("TIM8's outputs are never both high, both are driven, and the band between "
                      "them is the one the DTG code asks for, to a tenth of it",
                      both_high == 0u && high_a > samples / 8u && high_n > samples / 8u &&
                          gap_ns * 10u > dead_ns * 9u && gap_ns * 9u < dead_ns * 10u);
        v303_off<A>();
    }
}

// ---------------------------------------------------------------------------
// m - TIM8's four vectors, the break through BKIN driven by the port
// ---------------------------------------------------------------------------

template <uint8_t A = 8>
void tm_tim8_vectors() {
    if constexpr (tim_present(A)) {
        using T8 = Tim<A>;
        constexpr Pad bkin = tim_break_pad(A, 0);   // PA6
        using Bkin = Pin<bkin.port, bkin.pin>;
        all_off();
        v303_off<A>();
        clear_v303_counts();
        T8::init();
        (void)T8::configure({.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                             .period = 99});
        (void)T8::output_channel(0, {.mode = TimOutputMode::pwm1, .compare = 50,
                                     .enable = false});
        T8::clear_flags(T8::all_flags);
        const uint16_t all_four = static_cast<uint16_t>(
            T8::update_interrupt | T8::compare_interrupt(0) | T8::trigger_interrupt |
            T8::commutation_interrupt | T8::break_interrupt);
        T8::interrupts(all_four, true);
        for (const Irq line : {Irq::tim8_up, Irq::tim8_cc, Irq::tim8_brk, Irq::tim8_trg_com}) {
            Pfic::clear_pending(line);
            Pfic::enable(line);
        }
        T8::enable(true);
        wait_us(2000);            // twenty periods of 100 us
        T8::enable(false);
        // The third line: a trigger event, then a COMMUTATION - the
        // preloaded channel configuration moved by SWEVGR.COMG.
        (void)T8::trigger_event();
        wait_us(10);
        (void)T8::preload_channels(true, false);
        (void)T8::commutation_event();
        wait_us(10);
        const uint32_t trg_calls = v8_trg;
        // The fourth: a break by software first - one flag, one call -
        // then the BREAK INPUT, PA6 in column 0, driven by the PORT for a
        // few microseconds (BIF cannot be cleared while the input stands
        // active, so the line may run more than once while the pad is
        // high). Declined if something outside holds the pad high.
        (void)T8::break_dead_time({.main_output_enable = true});
        (void)T8::break_event();
        wait_us(10);
        const uint32_t brk_soft = v8_brk;
        const uint16_t soft_mask = v8_brk_mask;
        Bkin::input(PinPull::down);
        wait_us(20);
        const bool held = Bkin::read();
        bool pad_broke = false;
        bool armed = false;
        bool brk_flag_left = true;
        if (!held) {
            (void)T8::break_dead_time({.main_output_enable = true, .break_enable = true,
                                       .break_active_high = true});
            wait_us(2);
            Bkin::output(false);
            wait_us(10);
            armed = T8::main_output();
            // THE LEVEL IS THE BREAK: while BKIN stands active BIF cannot
            // be cleared, so a vector that only acknowledges would run
            // again the moment it returned and the loop below would never
            // lower the pad (measured, exactly that way). The body masks
            // its own interrupt for this pulse.
            v8_brk_self_mask = true;
            Bkin::set();
            wait_us(10);
            pad_broke = !T8::main_output();
            Bkin::clear();
            wait_us(10);
            v8_brk_self_mask = false;
            // BIF could not be cleared while the pad was high; with the
            // pad low it can.
            T8::clear_flags(T8::break_flag);
            brk_flag_left = T8::flag(T8::break_flag);
            // The break input OFF while the pad is still driven low: a
            // released pad floats, and a break input that reads it active
            // holds BIF for ever.
            (void)T8::break_dead_time({});
            wait_us(2);
            Bkin::release();
        }
        const uint32_t brk_after_pad = v8_brk;
        T8::interrupts(all_four, false);
        for (const Irq line : {Irq::tim8_up, Irq::tim8_cc, Irq::tim8_brk, Irq::tim8_trg_com}) {
            Pfic::disable(line);
        }
        print(serial, "  TIM8's four lines: ", v8_up, " update, ", v8_cc, " compare, ", trg_calls,
              " trigger/commutation (last mask ", hex(v8_trg_mask), "), ", brk_soft,
              " break by software (mask ", hex(soft_mask), ") and ", brk_after_pad - brk_soft,
              " for a 2 us pulse on BKIN (", held ? "HELD HIGH outside" : "driven by the port",
              ", MOE ", armed ? "set" : "CLEAR", " before it and ",
              pad_broke ? "cleared" : "STILL SET", " after; BIF ", hex(v8_brk_flags_seen & tim_bif),
              " in the body, cleared once the pad fell: ", !brk_flag_left, ")", crlf);
        bench.verdict("TIM8's update and compare vectors ran for their own events - twenty "
                      "each on two lines",
                      v8_up >= 19u && v8_up <= 21u && v8_cc >= 19u && v8_cc <= 21u);
        bench.verdict("its trigger/commutation vector ran once for a trigger and once for a "
                      "commutation, its mask the pair of flags that line answers for",
                      trg_calls == 2u && (v8_trg_mask & static_cast<uint16_t>(~(tim_tif | tim_comif))) == 0u);
        bench.verdict("and the break vector ran for a software break alone, then for BKIN raised "
                      "by the port - MOE set before the pulse and cleared by it",
                      brk_soft == 1u && soft_mask == tim_bif && !held && armed && pad_broke &&
                          brk_after_pad == brk_soft + 1u && !brk_flag_left);
        v303_off<A>();
    }
}

// ---------------------------------------------------------------------------
// n - TIM9 and TIM10 with no wire
// ---------------------------------------------------------------------------

/// One advanced timer's wireless round: the time base against the core's
/// counter, a PWM on channel 3 captured by channel 4 through the indirect
/// mapping, and the update and compare vectors.
template <uint8_t N>
bool wireless_round(volatile uint32_t& up_calls, volatile uint32_t& cc_calls) {
    using T = Tim<N>;
    using Wave3 = TimPad<tim_channel_pad(N, 0, 2)>;
    T::init();
    (void)T::remap(0);
    (void)T::configure({.prescaler = static_cast<uint16_t>(ticks_per_us - 1u), .period = 999});
    T::clear_flags(T::all_flags);
    T::enable(true);
    uint32_t updates = 0;
    Stopwatch fifty;
    while (fifty.us() < 50u * 1000u) {
        if (T::flag(T::update_flag)) {
            T::clear_flags(T::update_flag);
            ++updates;
        }
    }
    T::enable(false);

    (void)T::configure({.prescaler = static_cast<uint16_t>(ticks_per_us - 1u), .period = 999,
                        .auto_reload_preload = true});
    (void)T::output_channel(2, {.mode = TimOutputMode::pwm1, .compare = 300});
    (void)T::capture_channel(3, {.select = TimChannelSelect::indirect,
                                 .polarity = TimCapturePolarity::falling});
    (void)T::main_output(true);
    Wave3::claim();
    T::update();
    T::clear_flags(T::all_flags);
    T::enable(true);
    const bool came = wait_flag<T>(T::compare_flag(3), 50000);
    T::clear_flags(T::compare_flag(3));
    const bool came2 = wait_flag<T>(T::compare_flag(3), 50000);
    const uint32_t high = T::compare(3);
    const uint16_t chctlr2 = T::regs().CHCTLR2;
    const uint16_t ccer = T::regs().CCER;
    T::enable(false);

    // The two vectors this letter arms: the update and the compare.
    up_calls = 0;
    cc_calls = 0;
    (void)T::configure({.prescaler = static_cast<uint16_t>(ticks_per_us - 1u), .period = 99});
    (void)T::output_channel(0, {.mode = TimOutputMode::pwm1, .compare = 50, .enable = false});
    T::clear_flags(T::all_flags);
    T::interrupts(T::update_interrupt | T::compare_interrupt(0), true);
    Pfic::clear_pending(T::irq());
    Pfic::clear_pending(T::cc_irq());
    Pfic::enable(T::irq());
    Pfic::enable(T::cc_irq());
    T::enable(true);
    wait_us(2000);
    T::enable(false);
    T::interrupts(T::update_interrupt | T::compare_interrupt(0), false);
    Pfic::disable(T::irq());
    Pfic::disable(T::cc_irq());
    Wave3::release();
    print(serial, "  TIM", N, ": ", updates, " updates of a 1 kHz period in 50 ms; channel 3's "
          "PWM (compare 300) captured by channel 4 at ", high, " us (CHCTLR2=", hex(chctlr2),
          " CCER=", hex(ccer), "); its vectors ran ", up_calls, " update and ", cc_calls,
          " compare in 2 ms of a 100 us period", crlf);
    return updates >= 49u && updates <= 51u && came && came2 && high >= 299u && high <= 301u &&
           up_calls >= 19u && up_calls <= 21u && cc_calls >= 19u && cc_calls <= 21u;
}

template <uint8_t A = 8>
void tn_tim9_tim10() {
    if constexpr (tim_present(A)) {
        all_off();
        v303_off<A>();
        const bool nine = wireless_round<9>(v9_up, v9_cc);
        bench.verdict("TIM9's time base keeps the core's time, its channel 3 makes a PWM its "
                      "channel 4 captures to the microsecond, and its update and compare "
                      "vectors are its own",
                      nine);
        const bool ten = wireless_round<10>(v10_up, v10_cc);
        bench.verdict("and TIM10's the same", ten);
        v303_off<A>();
    }
}

// ---------------------------------------------------------------------------
// o - TIM5's width
// ---------------------------------------------------------------------------

template <uint8_t N5 = 5>
void to_tim5_width() {
    if constexpr (tim_present(N5)) {
        using T5 = Tim<N5>;
        all_off();
        T5::init();
        (void)T5::configure({.prescaler = 0, .period = T5::max_period});
        // Thirty-two bits written raw, around the driver's own width
        // check: what the register keeps is the width.
        T5::regs().ATRLR = 0x00012345u;
        const uint32_t arr = T5::regs().ATRLR;
        T5::regs().CNT = 0x0001FFF0u;
        const uint32_t cnt = T5::regs().CNT;
        T5::regs().CHCVR[0] = 0x00054321u;
        const uint32_t ccr = T5::regs().CHCVR[0];
        // The counter run from just below 0xFFFF with the auto-reload
        // at its widest: past 0xFFFF a sixteen-bit counter wraps.
        T5::regs().ATRLR = 0xFFFFFFFFu;
        T5::regs().CNT = 0x0000FFF0u;
        T5::enable(true);
        wait_us(10);
        T5::enable(false);
        const uint32_t after = T5::regs().CNT;
        const bool wrapped = after < 0xFFF0u;
        const uint8_t measured = (arr == 0x2345u && cnt == 0xFFF0u && wrapped) ? 16u
                                 : (arr == 0x12345u && cnt == 0x1FFF0u && !wrapped) ? 32u
                                                                                     : 0u;
        print(serial, "  TIM5 written raw: ATRLR 0x12345 reads ", hex(arr), ", CNT 0x1FFF0 reads ",
              hex(cnt), ", CH1CVR 0x54321 reads ", hex(ccr), "; run from 0xFFF0 for 10 us: ",
              hex(after), " - ", measured, " bits (the driver says ", T5::counter_bits, ")", crlf);
        bench.verdict("TIM5's counter, auto-reload and compare are as wide as the driver says - "
                      "chapter 15's class note, answered by the silicon",
                      measured == T5::counter_bits);
        T5::release();
    }
}

// ---------------------------------------------------------------------------
// p - the basic timers
// ---------------------------------------------------------------------------

/// One basic timer: its update against the core's counter, its vector,
/// and its TRGO counted by TIM9 over ITRx (S, a parameter so the type
/// stays one of the template's own).
template <uint8_t B, uint8_t S = 9>
void basic_round(volatile uint32_t& calls) {
    using T = Tim<B>;
    using Slave = Tim<S>;
    T::init();
    (void)T::configure({.prescaler = static_cast<uint16_t>(ticks_per_us - 1u), .period = 99});
    T::clear_flags(T::all_flags);
    T::enable(true);
    uint32_t seen = 0;
    Stopwatch w;
    while (seen < 20u && w.us() < 100u * 1000u) {
        if (T::flag(T::update_flag)) {
            T::clear_flags(T::update_flag);
            ++seen;
        }
    }
    const uint32_t span = w.us();
    T::enable(false);

    calls = 0;
    T::clear_flags(T::all_flags);
    T::interrupts(T::update_interrupt, true);
    Pfic::clear_pending(T::irq());
    Pfic::enable(T::irq());
    T::enable(true);
    wait_us(5000);
    T::enable(false);
    T::interrupts(T::update_interrupt, false);
    Pfic::disable(T::irq());
    const uint32_t vector_calls = calls;

    constexpr uint8_t itr = tim_trigger_index_for(S, B);
    Slave::init();
    (void)TimEventCounter<Slave>::setup(static_cast<TimTrigger>(itr));
    TimEventCounter<Slave>::restart();
    (void)T::configure({.prescaler = static_cast<uint16_t>(ticks_per_us - 1u), .period = 99});
    (void)T::master(TimMasterMode::update);
    T::enable(true);
    wait_us(20000);
    T::enable(false);
    const uint32_t counted = TimEventCounter<Slave>::count();
    Slave::release();
    const bool refused = !T::master(TimMasterMode::oc1ref);
    print(serial, "  TIM", B, ": twenty update periods of 100 us in ", span, " us, ", vector_calls,
          " calls of its vector in 5 ms, and TIM9 on ITR", itr, " counted ", counted,
          " of its TRGO updates in 20 ms (a channel's TRGO code refused: ", refused, ")", crlf);
    bench.verdict(B == 6 ? "TIM6 keeps the core's time, its vector is its own, and its TRGO "
                           "reaches TIM9's ITR2 - every update"
                         : "TIM7 the same, over TIM9's ITR3",
                  span + 20u >= 2000u && span <= 2020u && vector_calls >= 49u &&
                      vector_calls <= 51u && counted >= 199u && counted <= 201u && refused);
    T::release();
}

template <uint8_t B6 = 6>
void tp_basic() {
    if constexpr (tim_present(B6)) {
        all_off();
        v303_off<8>();
        basic_round<B6>(v6_up);
        basic_round<7>(v7_up);
    }
}

// ---------------------------------------------------------------------------
// q - the dual-edge capture over the PA6-PA1 jumper
// ---------------------------------------------------------------------------

template <bool aux = Tim<2>::has_dual_edge_capture>
void tq_dual_edge() {
    if constexpr (aux) {
        all_off();
        need_jumper();
        if (!jumper) {
            print(serial, "  no jumper (PA6 to PA1): the dual-edge capture is not measured", crlf);
            bench.verdict("the jumper was tested for (no jumper: nothing captured)", true);
            return;
        }
        // FIRST, WHETHER THIS DIE HAS THE REGISTER: the class has it only
        // "for lot numbers where the penultimate sixth bit is not zero",
        // and the verb asks the die by reading its bit back.
        Meas::init();
        const uint16_t chctlr_before = Meas::regs().CHCTLR1;
        const bool present = Meas::dual_edge_capture(1);
        const uint16_t aux_read = Meas::regs().AUX;
        const uint16_t chctlr_after = Meas::regs().CHCTLR1;
        Meas::release();
        if (!present) {
            print(serial, "  TIM2's AUX reads ", hex(aux_read), " after CAP_ED_CH2 was written: this die's "
                  "lot has no dual-edge capture; CHCTLR1 ", hex(chctlr_before), " before and ",
                  hex(chctlr_after), " after", crlf);
            bench.verdict("the dual-edge verb answers false on a die whose TIMx_AUX does not keep "
                          "the bit, and writes nothing else",
                          aux_read == 0u && chctlr_after == chctlr_before);
            all_off();
            return;
        }
        constexpr uint16_t duties[] = {100, 250, 500, 750};
        uint8_t high_hits = 0;
        uint8_t low_hits = 0;
        uint8_t agree = 0;
        for (const uint16_t d : duties) {
            Wave::init();
            (void)Wave::remap(0);
            (void)wave_self_capture(999, d);
            uint32_t by_pol[2] = {0, 0};
            for (uint8_t pol = 0; pol < 2u; ++pol) {
                Meas::init();
                (void)Meas::remap(0);
                (void)Meas::configure({.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                                       .period = 0xFFFF});
                (void)Meas::dual_edge_capture(1, pol == 0u ? TimCapturePolarity::rising
                                                           : TimCapturePolarity::falling);
                MeasPad::claim_input();
                Meas::clear_flags(Meas::all_flags);
                Meas::enable(true);
                (void)wait_flag<Meas>(Meas::compare_flag(1), 20000);
                (void)Meas::compare(1);
                Meas::clear_flags(Meas::compare_flag(1));
                const bool came = wait_flag<Meas>(Meas::compare_flag(1), 20000);
                by_pol[pol] = came ? Meas::compare(1) : 0xFFFFFFFFu;
                Meas::enable(false);
            }
            const bool on = Meas::dual_edge(1);
            // The two-channel method on the same wave, for comparison.
            Meas::init();
            (void)Meas::remap(0);
            (void)Meas::configure({.prescaler = static_cast<uint16_t>(ticks_per_us - 1u),
                                   .period = 0xFFFF});
            (void)Meas::capture_channel(1, {.select = TimChannelSelect::direct,
                                            .polarity = TimCapturePolarity::rising});
            (void)Meas::capture_channel(0, {.select = TimChannelSelect::indirect,
                                            .polarity = TimCapturePolarity::falling});
            (void)Meas::slave({.mode = TimSlaveMode::reset, .trigger = TimTrigger::ti2});
            MeasPad::claim_input();
            Meas::clear_flags(Meas::all_flags);
            Meas::enable(true);
            wait_us(5000);
            const uint32_t two_ch = Meas::compare(0);
            Meas::enable(false);
            Wave::enable(false);
            const uint32_t low = 1000u - d;
            const auto near = [](uint32_t v, uint32_t want) { return v + 2u >= want && v <= want + 2u; };
            if (near(by_pol[0], d)) { ++high_hits; }
            if (near(by_pol[1], low)) { ++low_hits; }
            if (near(two_ch, d)) { ++agree; }
            print(serial, "  high ", d, " us of 1000: the dual-edge register read ", by_pol[0],
                  " (CC2P clear) and ", by_pol[1], " (CC2P set); AUX's bit ", on,
                  "; the two-channel method ", two_ch, " us", crlf);
        }
        bench.verdict("the two-channel method on the same wave reads its high time (the "
                      "reference the dual-edge register is weighed against)",
                      agree == 4u);
        bench.verdict("the dual-edge capture holds the width of the HIGH pulse with CC2P clear "
                      "and of the LOW pulse with it set, to two microseconds, in one register",
                      high_hits == 4u && low_hits == 4u);
        Meas::release();
        all_off();
    }
}

// ---------------------------------------------------------------------------
// r - TIM3's external trigger on PD2, driven by the port
// ---------------------------------------------------------------------------

template <Pad etr = tim_etr_pad(3, 0)>
void tr_etr() {
    if constexpr (pad_bonded(etr)) {
        using Etr = Pin<etr.port, etr.pin>;
        all_off();
        Wave::init();
        (void)Wave::configure({.prescaler = 0, .period = 0xFFFF});
        const auto burst = [](uint8_t rising) {
            for (uint8_t i = 0; i < rising; ++i) {
                Etr::set();
                wait_us(2);
                Etr::clear();
                wait_us(2);
            }
        };
        struct Case {
            const char* name;
            TimEtrConfig cfg;
            uint32_t want;
        };
        const Case cases[] = {
            {"rising edges", {.clock_mode2 = true}, 40},
            {"inverted (falling edges)", {.inverted = true, .clock_mode2 = true}, 40},
            {"prescaled by two", {.prescaler = 1, .clock_mode2 = true}, 20},
            {"prescaled by eight", {.prescaler = 3, .clock_mode2 = true}, 5},
        };
        uint8_t right = 0;
        for (const Case& c : cases) {
            Etr::output(false);
            wait_us(5);
            (void)Wave::external_trigger(c.cfg);
            Wave::set_count(0);
            Wave::enable(true);
            wait_us(5);
            const uint32_t before = Wave::count();
            burst(40);
            wait_us(5);
            const uint32_t counted = Wave::count() - before;
            Wave::enable(false);
            if (counted == c.want) {
                ++right;
            }
            print(serial, "  forty pulses on PD2, ", c.name, ": TIM3 counted ", counted, " (",
                  c.want, " expected)", crlf);
        }
        // External clock mode 1 on ETRF: the same edges through the slave
        // controller instead of ECE.
        (void)Wave::external_trigger({});
        (void)Wave::slave({.mode = TimSlaveMode::external_clock1, .trigger = TimTrigger::etr});
        Wave::set_count(0);
        Wave::enable(true);
        wait_us(5);
        burst(40);
        wait_us(5);
        const uint32_t mode1 = Wave::count();
        Wave::enable(false);
        Etr::release();
        print(serial, "  the same forty through external clock mode 1 on ETRF: ", mode1, crlf);
        bench.verdict("TIM3's external trigger on PD2 counts the pad's edges as the port writes "
                      "them - both polarities, the prescaler, and both external clock modes",
                      right == 4u && mode1 == 40u);
        all_off();
    }
}

// ---------------------------------------------------------------------------
// s - the internal triggers the CH32V303 adds
// ---------------------------------------------------------------------------

/// Slave S counts master M's update events over ITRx: the slave first,
/// the master at 100 kHz for two milliseconds - two hundred events.
template <uint8_t S, uint8_t M>
uint32_t link_count() {
    using Sl = Tim<S>;
    using Ma = Tim<M>;
    constexpr uint8_t itr = tim_trigger_index_for(S, M);
    static_assert(itr != 0xFFu, "no such link in tables 14-2 and 15-2");
    Sl::init();
    Ma::init();
    (void)TimEventCounter<Sl>::setup(static_cast<TimTrigger>(itr));
    TimEventCounter<Sl>::restart();
    (void)Ma::configure({.prescaler = 0, .period = 1439});
    (void)Ma::master(TimMasterMode::update);
    Stopwatch w;
    Ma::enable(true);
    while (w.us() < 2000u) {
    }
    Ma::enable(false);
    const uint32_t counted = TimEventCounter<Sl>::count();
    Sl::release();
    Ma::release();
    return counted;
}

template <uint8_t S, uint8_t M>
bool link_line(uint8_t& good) {
    const uint32_t c = link_count<S, M>();
    const bool ok = c >= 199u && c <= 201u;
    print(serial, "  TIM", S, " ITR", tim_trigger_index_for(S, M), " <- TIM", M, ": ", c,
          ok ? "" : "  <- NOT the 200 the master made", crlf);
    if (ok) {
        ++good;
    }
    return ok;
}

template <uint8_t A = 8>
void ts_links() {
    if constexpr (tim_present(A)) {
        all_off();
        v303_off<A>();
        uint8_t good = 0;
        // Into the three extra advanced timers: table 14-2, whole.
        (void)link_line<A, 1>(good);
        (void)link_line<A, 2>(good);
        (void)link_line<A, 4>(good);
        (void)link_line<A, 5>(good);
        (void)link_line<9, 10>(good);
        (void)link_line<9, 5>(good);
        (void)link_line<9, 6>(good);
        (void)link_line<9, 7>(good);
        (void)link_line<10, 9>(good);
        (void)link_line<10, 2>(good);
        (void)link_line<10, 4>(good);
        (void)link_line<10, 5>(good);
        // TIM8 and TIM5 as masters of the others (tables 14-2, 15-2).
        (void)link_line<4, A>(good);
        (void)link_line<5, A>(good);
        (void)link_line<1, 5>(good);
        (void)link_line<3, 5>(good);
        (void)link_line<5, 2>(good);
        (void)link_line<5, 3>(good);
        (void)link_line<5, 4>(good);
        bench.verdict("every link into TIM8, TIM9 and TIM10 and every link TIM8 and TIM5 master "
                      "counts the master's two hundred updates - tables 14-2 and 15-2 as the "
                      "driver folds them",
                      good == 19u);
        // TIM2's ITR1: "TIM8/USB/ETH" in table 15-2, and an AFIO field
        // that chooses between the last two. Both settings of the field,
        // with TIM8 as the master.
        const uint8_t kept = Afio::remap_code(Remap::tim2_itr1);
        (void)Afio::remap(Remap::tim2_itr1, 0);
        const uint32_t at0 = link_count<2, A>();
        const bool can1 = Afio::remap(Remap::tim2_itr1, 1);
        const uint32_t at1 = can1 ? link_count<2, A>() : 0u;
        (void)Afio::remap(Remap::tim2_itr1, kept);
        print(serial, "  TIM2 ITR1 <- TIM8: ", at0, " with TIM2ITR1_RM at 0, ", at1,
              " with it at 1", can1 ? "" : " (not writable)", crlf);
        bench.verdict("TIM2's ITR1 carries TIM8's TRGO with its AFIO field at the reset value, "
                      "which is the link the driver's table names",
                      at0 >= 199u && at0 <= 201u);
        v303_off<A>();
    }
}

/// The CH32V303's letters, registered where the part has the timers - a
/// template for the same reason as the letters.
template <uint8_t A = 8>
void register_v303_letters() {
    if constexpr (tim_present(A)) {
        bench.letter('k', "TIM8's PWM captured by TIM4 over the PC6-PB8 jumper",
                     tk_tim8_capture<A>);
        bench.letter('l', "TIM8's complementary pair on PC6/PA7 and its dead band",
                     tl_tim8_pair<A>);
        bench.letter('m', "TIM8's four vectors, the break from BKIN driven by the port",
                     tm_tim8_vectors<A>);
        bench.letter('n', "TIM9 and TIM10 with no wire: time base, PWM, vectors",
                     tn_tim9_tim10<A>);
        bench.letter('o', "TIM5's width, written raw", to_tim5_width<>);
        bench.letter('p', "the basic timers TIM6 and TIM7, their TRGO on TIM9", tp_basic<>);
        bench.letter('s', "the internal triggers the class adds", ts_links<A>);
    }
    if constexpr (Tim<2>::has_dual_edge_capture) {
        bench.letter('q', "the dual-edge capture over the PA6-PA1 jumper", tq_dual_edge<>);
    }
    if constexpr (pad_bonded(tim_etr_pad(3, 0))) {
        bench.letter('r', "TIM3's external trigger on PD2, driven by the port", tr_etr<>);
    }
}

/// The CH32V303's vector bodies: templates, so that a part without the
/// timer compiles an empty function that no table entry reaches.
template <uint8_t A = 8>
[[gnu::always_inline]] inline void tim8_body(Irq line) {
    if constexpr (tim_present(A)) {
        const uint16_t hit = Tim<A>::isr(Tim<A>::vector_flags(line));
        if (hit == 0u) {
            return;
        }
        if (line == Irq::tim8_up) {
            v8_up = v8_up + 1u;
        } else if (line == Irq::tim8_cc) {
            v8_cc = v8_cc + 1u;
        } else if (line == Irq::tim8_trg_com) {
            v8_trg = v8_trg + 1u;
            v8_trg_mask = hit;
        } else {
            v8_brk = v8_brk + 1u;
            v8_brk_mask = hit;
            if (v8_brk_self_mask) {
                v8_brk_flags_seen = Tim<A>::flags();
                Tim<A>::interrupts(Tim<A>::break_interrupt, false);
            }
        }
    }
}

template <uint8_t N>
[[gnu::always_inline]] inline void advanced_body(Irq line, volatile uint32_t& up,
                                                 volatile uint32_t& cc) {
    if constexpr (tim_present(N)) {
        const uint16_t hit = Tim<N>::isr(Tim<N>::vector_flags(line));
        if (hit != 0u) {
            if (line == Tim<N>::irq()) {
                up = up + 1u;
            } else {
                cc = cc + 1u;
            }
        }
    }
}

template <uint8_t B>
[[gnu::always_inline]] inline void basic_body(volatile uint32_t& calls) {
    if constexpr (tim_present(B)) {
        if (Tim<B>::isr() != 0u) {
            calls = calls + 1u;
        }
    }
}

void banner() {
    print(serial, crlf, "test_vx03_tim on ", device::part_name,
          " - the timers (RM ch. 14 and 15) at ", tim_hz / 1'000'000u, " MHz", crlf,
          "  nothing to wire: a timer drives its own pad and captures it, and the internal",
          crlf, "  triggers link two timers with no pad at all. ONE OPTIONAL JUMPER, PA6 to",
          crlf, "  PA1, lets TIM2 measure TIM3's wave - letter j says whether it is there",
          crlf, crlf);
    bench.menu();
    print(serial, crlf);
}

}  // namespace

// The vectors this program owns. Each body reads AND CLEARS the flags of
// its own vector first, which on the advanced timer is the mask of the
// line it was bound to.
extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }

extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }

extern "C" BRIO_CH32_INTERRUPT void tim1_up_handler() {
    if (Adv::isr(Adv::vector_flags(brio::Irq::tim1_up)) != 0u) {
        up_calls = up_calls + 1u;
    }
}

extern "C" BRIO_CH32_INTERRUPT void tim1_cc_handler() {
    const uint16_t hit = Adv::isr(Adv::vector_flags(brio::Irq::tim1_cc));
    if (hit != 0u) {
        cc_calls = cc_calls + 1u;
        cc_mask = hit;
    }
}

extern "C" BRIO_CH32_INTERRUPT void tim1_brk_handler() {
    const uint16_t hit = Adv::isr(Adv::vector_flags(brio::Irq::tim1_brk));
    if (hit != 0u) {
        brk_calls = brk_calls + 1u;
        brk_mask = hit;
    }
}

extern "C" BRIO_CH32_INTERRUPT void tim1_trg_com_handler() {
    if (Adv::isr(Adv::vector_flags(brio::Irq::tim1_trg_com)) != 0u) {
        trg_calls = trg_calls + 1u;
    }
}

extern "C" BRIO_CH32_INTERRUPT void tim3_handler() {
    const uint16_t hit = Wave::isr();
    if (hit != 0u) {
        wave_calls = wave_calls + 1u;
    }
    if (interval_arm && (hit & Interval::capture_flag) != 0u) {
        if (const std::optional<uint32_t> d = Interval::interval()) {
            IntervalLatch::store(*d);
        }
    }
}

extern "C" BRIO_CH32_INTERRUPT void tim8_up_handler() { tim8_body<>(brio::Irq::tim8_up); }
extern "C" BRIO_CH32_INTERRUPT void tim8_cc_handler() { tim8_body<>(brio::Irq::tim8_cc); }
extern "C" BRIO_CH32_INTERRUPT void tim8_brk_handler() { tim8_body<>(brio::Irq::tim8_brk); }
extern "C" BRIO_CH32_INTERRUPT void tim8_trg_com_handler() {
    tim8_body<>(brio::Irq::tim8_trg_com);
}
extern "C" BRIO_CH32_INTERRUPT void tim9_up_handler() {
    advanced_body<9>(brio::Irq::tim9_up, v9_up, v9_cc);
}
extern "C" BRIO_CH32_INTERRUPT void tim9_cc_handler() {
    advanced_body<9>(brio::Irq::tim9_cc, v9_up, v9_cc);
}
extern "C" BRIO_CH32_INTERRUPT void tim10_up_handler() {
    advanced_body<10>(brio::Irq::tim10_up, v10_up, v10_cc);
}
extern "C" BRIO_CH32_INTERRUPT void tim10_cc_handler() {
    advanced_body<10>(brio::Irq::tim10_cc, v10_up, v10_cc);
}
extern "C" BRIO_CH32_INTERRUPT void tim6_handler() { basic_body<6>(v6_up); }
extern "C" BRIO_CH32_INTERRUPT void tim7_handler() { basic_body<7>(v7_up); }

extern "C" BRIO_CH32_INTERRUPT void tim4_handler() {
    const uint16_t hit = Quad::isr();
    if (period_arm && (hit & Period::period_flag) != 0u) {
        PeriodLatch::store(Period::period_ticks());
    }
    if (period_arm && (hit & Period::width_flag) != 0u) {
        WidthLatch::store(Period::width_ticks());
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "the time base: the gate, the counting, the two shadow registers",
                 ta_time_base);
    bench.letter('b', "PWM read back, captured by its own timer and over the jumper", tb_pwm);
    bench.letter('c', "the complementary pair on TIM1 and its dead band", tc_pair);
    bench.letter('d', "the interval and period meters behind a MeterLatch", td_meters);
    bench.letter('e', "the event and gated counters over an internal trigger", te_counters);
    bench.letter('f', "the periodic tick, the one pulse, the encoder", tf_tasks);
    bench.letter('g', "the break input, the software break and the automatic re-enable",
                 tg_break);
    bench.letter('h', "the trigger chains: reset, trigger and gated modes", th_chains);
    bench.letter('i', "the vectors: four lines on TIM1, one on TIM3", ti_vectors);
    bench.letter('j', "the jumper, and what a plain output pad reaches", tj_jumper);
    register_v303_letters<>();

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL144" : "FAILED",
                    " tick=", tick_ok ? "STK" : "FAILED", brio::crlf);
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
        brio::print(serial, "  stack: ", brio::stack_untouched(), " B never touched",
                    brio::crlf);
        bench.prompt();
    }
}
