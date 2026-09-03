// test_stm32_rtc - the reference bench suite for the STM32G0's RTC
// domain: stm32g0/rtc.hpp (RM0444 ch. 30 and the backup-register half of
// ch. 31) and, with it, the RCC_BDCR half of chapter 5 that only this
// domain can reach.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NOTHING TO WIRE, AND THE INSTRUMENT IS THE POINT. TIM16's input
// multiplexer reaches LSI, LSE and the RTC's own wake-up signal
// (25.6.18), so a 64 MHz capture channel weighs all three against the
// core clock with no pad, no wire and no external reference. Everything
// this suite calls a frequency is that ratio; the CORE's own absolute
// error (HSI16, factory-trimmed to 1 %) is stated where it matters and
// cancels out of every comparison that is a difference.
//
// NO FLASH IS WRITTEN and no option byte is touched.
//
// WHAT IT COSTS THE BOARD, said once: letter a resets the RTC DOMAIN
// (RCC_BDCR.BDRST) when it has to move the clock select, and a domain
// reset wipes the calendar, the alarms and THE FIVE BACKUP REGISTERS.
// Letter j and the reboot letter v are written to be run after that, not
// before, and z runs them in that order.
//
// What is exercised, letter by letter:
//   a  the domain: the two locks (PWR_CR1.DBP and the RTC's own WPR
//      keys), the RTCAPBEN interface gate, RTCSEL as a ONE-WAY choice,
//      and BDRST as the way back
//   b  LSE: does this Nucleo's X2 crystal exist and start, what does it
//      measure, and the two register rules (the drive that may only be
//      lowered, the bypass that may only be written when stopped)
//   c  LSI weighed on TIM16, the prescalers, and the calendar's own
//      1 Hz against the kernel tick
//   d  the calendar's boundaries in one second: BCD both ways, midnight,
//      the month end, 29 February in a leap year and its absence in
//      another, the year wrap
//   e  the shadow registers against BYPSHAD: RSF, the read cost, and the
//      coherence rule
//   f  both alarms: the match, the LATENCY from the match to the flag,
//      the sub-second comparison, and the masks
//   g  the wake-up timer on four of its five clocks, each period weighed
//      on TIM16, plus the write window and the two refusals
//   h  ES0548 2.9.1 STAGED: consecutive initialization-mode entries with
//      and without the driver's workaround
//   i  smooth calibration measured, not asserted: CALP and CALM at both
//      ends of their range, weighed on the wake-up signal
//   j  the backup registers: five words, the DBP gate over them, and the
//      fact that the RTC's own key does NOT cover them
//
//   v  (by name only) THE SURVIVAL LETTER. Writes the backup registers,
//      reboots the board through a software reset, and checks what came
//      back - so it is NOT in `z`, which has to be one console session a
//      tool can judge from a single capture. Run it with
//          python3 tools/bench.py run E v --app test_stm32_rtc
//                  --expect="pass," --timeout 200
//
// build: boards = g0b1re
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "stm32g0/clock.hpp"
#include "stm32g0/delay.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/platform_stm32.hpp"
#include "stm32g0/pwr.hpp"
#include "stm32g0/reset.hpp"
#include "stm32g0/rtc.hpp"
#include "stm32g0/ticker.hpp"
#include "stm32g0/tim.hpp"
#include "stm32g0/usart.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
constexpr SysClock clock;

// ---------------------------------------------------------------------------
// The token letter v lives in (the test_stm32_platform pattern: inline,
// in .noinit, magic-guarded because RM0444 promises nothing about SRAM
// across a reset).
// ---------------------------------------------------------------------------
inline constexpr uint16_t token_magic = 0x5B31;

struct Token {
    uint16_t magic;
    uint8_t leg;
    uint16_t pass;
    uint16_t fail;
    uint32_t written[5];
};
[[gnu::section(".noinit")]] inline Token token;

namespace {

using namespace brio;

constexpr UartPins console_pins{
    .tx = {'A', 2, PinFunction::af1},
    .rx = {'A', 3, PinFunction::af1},
};
using Serial = Uart<2, console_pins>;
constexpr Serial serial;

TestBench<Serial> bench;

uint32_t boot_flags = 0;
uint32_t boot_bdcr = 0;

/// Counted by the RTC handler; read by letter g so that the vector's
/// participation is a fact on the console and not an assumption.
volatile uint32_t rtc_interrupts = 0;

// ---------------------------------------------------------------------------
// The instrument: TIM16's capture channel, fed by the input multiplexer
// ---------------------------------------------------------------------------
//
// One channel, a free-running 16-bit counter, and a source chosen by
// TISEL. The prescaler is picked per source so that ONE period fits the
// counter, which is TimIntervalMeter's whole contract.

using Meter = TimIntervalMeter<Tim<16>, 0>;
constexpr uint32_t timer_hz = SysClock::hz;   // TIMPCLK == HCLK (prescalers pinned)

/// Arm the meter on a TISEL code with a chosen prescaler, and throw the
/// first edge away.
bool meter_on(uint8_t code, uint16_t prescaler, uint8_t filter = 0) {
    Tim<16>::bus_clock(true);
    Tim<16>::enable(false);
    if (!Meter::setup(prescaler, filter)) {
        return false;
    }
    if (!Tim<16>::input_select(0, code)) {
        return false;
    }
    (void)Tim<16>::isr();
    Meter::restart();
    return true;
}

/// Average `n` intervals in timer ticks, or nothing if the edges did not
/// come within `guard_ms`. The first interval after arming is discarded
/// by TimIntervalMeter itself.
///
/// THE ONE THING NO CHAPTER SAYS OUT LOUD, and which cost this suite two
/// rounds of wrong numbers: WHAT TISEL CALLS "RTC WAKE-UP" IS THE MASKED
/// INTERRUPT LINE AND NOT A PULSE. It rises with WUTF and stays up until
/// the flag is cleared, so a capture channel sees exactly ONE edge per
/// acknowledgement - and an acknowledgement that arrives late (a polling
/// loop that is off doing something else) does not shorten the interval,
/// it DELETES the next one. Which is why the letters that use this source
/// enable the RTC's NVIC line and let its handler clear the flag: an
/// interrupt is prompt by construction where a loop is prompt by luck.
/// 25.6.18's own footnote - the source "requires to enable the RTC
/// interrupt" - is the same fact from the other side.
std::optional<uint32_t> meter_average(uint16_t n, uint32_t guard_ms) {
    const uint32_t t0 = Ticker::millis();
    uint32_t sum = 0;
    uint16_t got = 0;
    while (got < n) {
        if (Ticker::millis() - t0 > guard_ms) {
            return std::nullopt;
        }
        if ((Tim<16>::flags() & Meter::capture_flag) == 0u) {
            continue;
        }
        Tim<16>::clear_flags(Meter::capture_flag);
        const std::optional<uint32_t> d = Meter::interval();
        if (d.has_value()) {
            sum += *d;
            ++got;
        }
    }
    return sum / n;
}

/// The spread of `n` intervals, and why a mean is not enough.
///
/// A capture channel polled by a program that also serves a console
/// MISSES EDGES: the capture flag stands, the next capture overwrites
/// CCR before the loop reads it, and what comes back is the distance
/// between edges that are not neighbours. Those errors LENGTHEN an
/// interval.
///
/// AND THE OTHER TAIL IS REAL TOO, which is what cost this suite a flaky
/// verdict for a whole campaign: an UNFILTERED capture of one of these
/// internal clock lines also OVER-captures, and an extra edge SHORTENS
/// an interval. Measured on LSI, unfiltered, in one z run: sixty-three
/// intervals between 1936 and 1967 timer ticks and one of 1822. So the
/// error is TWO-SIDED, the shortest sample is NOT the period seen from
/// below, and neither the minimum nor the mean is an estimator of it.
///
/// What this suite quotes is therefore anchored on the MEDIAN - the one
/// statistic both tails have to outnumber before they can move it - with
/// a trimmed mean of the samples inside a narrow band around it, and the
/// count of them printed beside it so a reader can see how much was
/// thrown away.
struct MeterSpread {
    uint32_t lo = 0;        ///< the shortest sample, outliers included
    uint32_t hi = 0;        ///< the longest sample, outliers included
    uint32_t median = 0;    ///< the middle sample: immune to both tails
    uint32_t mean = 0;      ///< over every sample, outliers included
    uint32_t robust = 0;    ///< over the samples inside the clean band
    uint32_t kept_lo = 0;   ///< the shortest sample INSIDE the band
    uint32_t kept_hi = 0;   ///< the longest sample INSIDE the band
    uint16_t good = 0;      ///< how many those were
    uint16_t total = 0;
};

/// `n` intervals, kept so the quality is a COUNT and not a guess: the
/// shortest, the widest, the median, the mean, and how many landed
/// within about 3 % of the median on either side.
std::optional<MeterSpread> meter_quality(uint16_t n, uint32_t guard_ms) {
    static uint32_t samples[64];
    if (n > 64u) {
        n = 64;
    }
    const uint32_t t0 = Ticker::millis();
    uint16_t got = 0;
    while (got < n) {
        if (Ticker::millis() - t0 > guard_ms) {
            return std::nullopt;
        }
        if ((Tim<16>::flags() & Meter::capture_flag) == 0u) {
            continue;
        }
        Tim<16>::clear_flags(Meter::capture_flag);
        const std::optional<uint32_t> d = Meter::interval();
        if (d.has_value()) {
            samples[got++] = *d;
        }
    }
    MeterSpread s{};
    s.lo = 0xFFFFFFFFu;
    s.total = n;
    uint32_t sum = 0;
    for (uint16_t i = 0; i < n; ++i) {
        if (samples[i] < s.lo) {
            s.lo = samples[i];
        }
        if (samples[i] > s.hi) {
            s.hi = samples[i];
        }
        sum += samples[i];
    }
    s.mean = sum / n;

    // Sixty-four words, insertion-sorted: the median is the anchor and
    // nothing here is fast enough to care about the cost.
    for (uint16_t i = 1; i < n; ++i) {
        const uint32_t key = samples[i];
        uint16_t j = i;
        while (j != 0u && samples[j - 1u] > key) {
            samples[j] = samples[j - 1u];
            --j;
        }
        samples[j] = key;
    }
    s.median = samples[n / 2u];

    // A SYMMETRIC band around the median, because both tails exist: a
    // missed edge lands above it and an over-capture below.
    const uint32_t slack = s.median / 32u;      // about 3 % either way
    const uint32_t low = s.median > slack ? s.median - slack : 0u;
    const uint32_t high = s.median + slack;
    uint32_t good_sum = 0;
    s.kept_lo = 0xFFFFFFFFu;
    for (uint16_t i = 0; i < n; ++i) {
        if (samples[i] >= low && samples[i] <= high) {
            good_sum += samples[i];
            if (samples[i] < s.kept_lo) {
                s.kept_lo = samples[i];
            }
            if (samples[i] > s.kept_hi) {
                s.kept_hi = samples[i];
            }
            ++s.good;
        }
    }
    if (s.good == 0u) {
        s.kept_lo = s.lo;
        s.kept_hi = s.hi;
    }
    s.robust = s.good != 0u ? good_sum / s.good : s.median;
    return s;
}

/// Set the wake-up timer going, put the meter on its interrupt line, let
/// two periods go by unjudged (the first assertion is short by up to one
/// count - 30.6.6 - and the meter's own first interval is discarded
/// anyway), and average `n` of what follows.
std::optional<uint32_t> meter_wakeup(RtcWakeupClock c, uint32_t reload,
                                     uint16_t prescaler, uint16_t n,
                                     uint32_t guard_ms) {
    if (!Rtc::set_wakeup(c, reload, true)) {
        return std::nullopt;
    }
    Nvic::enable(Rtc::irq());
    if (!meter_on(Rtc::wakeup_tim16_ti1_code, prescaler)) {
        Nvic::disable(Rtc::irq());
        Rtc::clear_wakeup();
        return std::nullopt;
    }
    (void)meter_average(2, guard_ms);   // warm-up, judged by nobody
    Meter::restart();
    const std::optional<uint32_t> t = meter_average(n, guard_ms);
    Nvic::disable(Rtc::irq());
    Rtc::clear_wakeup();
    return t;
}

/// Ticks -> hertz for a meter running at `prescaler`.
uint32_t meter_hz(uint32_t ticks, uint16_t prescaler) {
    if (ticks == 0u) {
        return 0;
    }
    const uint32_t tick_hz = timer_hz / (static_cast<uint32_t>(prescaler) + 1u);
    return tick_hz / ticks;
}

/// Ticks -> microseconds for a meter running at `prescaler`.
uint32_t meter_us(uint32_t ticks, uint16_t prescaler) {
    const uint32_t tick_hz = timer_hz / (static_cast<uint32_t>(prescaler) + 1u);
    return static_cast<uint32_t>((static_cast<uint64_t>(ticks) * 1'000'000ULL) /
                                 tick_hz);
}

// ---------------------------------------------------------------------------
// State the letters share
// ---------------------------------------------------------------------------

bool lse_running = false;         ///< set by letter b, read by i's report
uint32_t lsi_hz = 32'000;         ///< measured by letter c, used for bands
RtcClockSource source_in_force = RtcClockSource::none;

/// The prescaler pair the calendar letters run on: an exact 1 Hz from
/// the NOMINAL rate of whatever is selected, which is what a real
/// application would do before it has measured its own board.
RtcPrescalers working_prescalers() {
    return source_in_force == RtcClockSource::lse ? rtc_prescalers_for(32768)
                                                  : rtc_prescalers_for(32000);
}

bool within(uint32_t v, uint32_t lo, uint32_t hi) { return v >= lo && v <= hi; }

/// Parts per million of `v` away from `ref`, unsigned.
uint32_t ppm_off(uint32_t v, uint32_t ref) {
    const uint32_t d = v > ref ? v - ref : ref - v;
    return static_cast<uint32_t>((static_cast<uint64_t>(d) * 1'000'000ULL) / ref);
}

void print_datetime(const RtcDateTime& d) {
    print(serial, 2000u + d.year, "-", d.month, "-", d.day, " ", d.hour, ":",
          d.minute, ":", d.second, " (weekday ", d.weekday, ")");
}

void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < 3u) {
    }
}

/// Set the calendar and wait for the next second boundary, so a letter
/// that has to watch a rollover starts with a whole second in hand.
bool set_time(const RtcDateTime& d) {
    if (!Rtc::init(working_prescalers(), d)) {
        return false;
    }
    return true;
}

// =============================================================================
// a - the domain: two locks, one-way choices, and the way back
// =============================================================================
void ta_domain() {
    print(serial, "  boot: RCC_BDCR=", hex(boot_bdcr), " RTCSEL=",
          static_cast<uint32_t>((boot_bdcr >> 8) & 3u), " RTCEN=",
          ((boot_bdcr & (1u << 15)) != 0u), " LSEON=",
          ((boot_bdcr & 1u) != 0u), crlf);

    // THE FIRST LOCK. PWR_CR1.DBP is clear after every system reset, and
    // with it clear the whole domain ignores writes. Proven by writing
    // something harmless into RCC_BDCR and reading it back unchanged -
    // LSEBYP is the safest bit there is here (the oscillator is off, so
    // 5.4.23's own precondition for writing it is met).
    Pwr::bus_clock(true);
    Pwr::rtc_domain_unlock(false);
    const uint32_t before = RtcDomain::bdcr();
    (void)RtcDomain::lse_bypass(true);
    const uint32_t locked = RtcDomain::bdcr();
    Pwr::rtc_domain_unlock(true);
    const bool unlocked = Pwr::rtc_domain_unlocked();
    bench.verdict("with DBP clear the RTC domain ignores a write to "
                  "RCC_BDCR (4.1.2)",
                  before == locked && (locked & 0x4u) == 0u);
    bench.verdict("setting DBP opens it", unlocked);
    (void)RtcDomain::lse_bypass(false);

    // The RTC's own APB interface has a SECOND enable, distinct from
    // RTCEN: RTCAPBEN gates the register bank while RTCEN gates the
    // counter. 5.2.17's rule applies - without the bus clock the
    // registers do not answer.
    RtcDomain::apb_clock(true);
    bench.verdict("the RTC's APB interface has an enable of its own "
                  "(RTCAPBEN), and it is now open",
                  RtcDomain::apb_clock());

    // THE ONE-WAY CHOICE. This board comes up with the domain already
    // running on LSI, because a system reset does not reach it - so
    // asking for LSE here must be REFUSED, and the refusal is the
    // driver telling the truth rather than writing a field the silicon
    // ignores.
    const RtcClockSource had = RtcDomain::selected();
    const RtcClockSource other = (had == RtcClockSource::lsi)
                                     ? RtcClockSource::lse
                                     : RtcClockSource::lsi;
    const bool refused = had != RtcClockSource::none &&
                         !RtcDomain::select(other);
    const bool same_again = RtcDomain::select(had);
    print(serial, "  RTCSEL in force: ", static_cast<uint32_t>(had),
          " (1=LSE 2=LSI); asking for ", static_cast<uint32_t>(other),
          " instead was ", refused ? "refused" : "NOT refused", crlf);
    bench.verdict("RTCSEL is one-way: a different source is refused while "
                  "one stands (5.4.23)",
                  had == RtcClockSource::none || refused);
    bench.verdict("and re-selecting what already stands is a no-op that "
                  "answers true",
                  same_again);

    // THE WAY BACK. BDRST wipes the domain - including the backup
    // registers, which is why this letter runs before j and v. After it,
    // RTCSEL reads none and the choice is open again.
    RtcDomain::reset();
    print(serial, "  after BDRST: RCC_BDCR=", hex(RtcDomain::bdcr()), crlf);
    bench.verdict("BDRST wipes the domain: RTCEN is down and RTCSEL is open "
                  "again",
                  !RtcDomain::enabled() &&
                      RtcDomain::selected() == RtcClockSource::none);

    // Now take the source this session will run on. LSE first, because
    // it is the better clock and the crystal may be fitted; LSI if it
    // does not start. Letter b measures whichever one this leaves.
    RtcDomain::lse_enable(true);
    const bool lse = RtcDomain::lse_wait_ready(4'000'000UL);
    lse_running = lse;
    if (!lse) {
        RtcDomain::lse_enable(false);
        Rcc::lsi_enable(true);
        (void)Rcc::lsi_wait_ready();
    }
    source_in_force = lse ? RtcClockSource::lse : RtcClockSource::lsi;
    const bool opened = RtcDomain::open(source_in_force);
    print(serial, "  this session runs the RTC on ",
          lse ? "LSE (the crystal started)" : "LSI (no LSE on this board)",
          crlf);
    bench.verdict("the domain opens on the chosen source with RTCEN set",
                  opened && RtcDomain::selected() == source_in_force &&
                      RtcDomain::enabled());

    // THE SECOND LOCK, and it is not the first. The RTC's own key
    // protects most of its registers and is INDEPENDENT of DBP: with DBP
    // set but no key written, a protected register still refuses. CALR
    // is the harmless one to prove it on.
    const uint32_t cal_before = Rtc::calr();
    Rtc::lock();
    Rtc::calr_unprotected(0x1FFu);   // no key: this must be dropped
    const uint32_t cal_locked = Rtc::calr();
    bench.verdict("the RTC's own WPR key is a SECOND lock: a protected "
                  "register refuses a write with DBP set and no key (30.3.8)",
                  cal_locked == cal_before);
    bench.verdict("and the key opens it", Rtc::calibrate(RtcCalibration{}) &&
                                              Rtc::calr() == 0u);
    (void)Rtc::calibrate(RtcCalibration{});
}

// =============================================================================
// b - LSE: is the crystal there, and what does it weigh?
// =============================================================================
void tb_lse() {
    print(serial, "  LSEON=", RtcDomain::lse_enabled(), " LSERDY=",
          RtcDomain::lse_ready(), " LSEBYP=", RtcDomain::lse_bypass(),
          " drive=", static_cast<uint32_t>(RtcDomain::lse_drive()), crlf);

    if (!lse_running) {
        // THE ANSWER IS STILL AN ANSWER. The Nucleo-64 schematic ships
        // an X2 footprint; whether it is populated is a board fact, and
        // a suite that cannot start the oscillator says so instead of
        // failing a verdict about a component that is not there.
        print(serial, "  LSE did not start: X2 is not fitted on this board, "
                      "or the crystal is dead. The data verdicts of this "
                      "letter are DECLINED and the rest of the suite runs "
                      "on LSI.", crlf);
        bench.verdict("LSE reports NOT ready, and the suite says so rather "
                      "than guessing",
                      !RtcDomain::lse_ready());
        // The two register RULES are still measurable with the
        // oscillator stopped, and they are the half that does not need a
        // crystal.
        RtcDomain::lse_enable(false);
        bench.verdict("with LSE stopped, LSEBYP is writable (5.4.23)",
                      RtcDomain::lse_bypass(true) && RtcDomain::lse_bypass());
        (void)RtcDomain::lse_bypass(false);
        RtcDomain::lse_enable(true);   // it will not start; harmless
        bench.verdict("and refused again once LSEON stands",
                      !RtcDomain::lse_bypass(true));
        RtcDomain::lse_enable(false);
        return;
    }

    // The crystal runs: weigh it. At 32.768 kHz one period is 1953
    // timer ticks at 64 MHz, which fits a 16-bit counter with no
    // prescaler at all.
    // FILTERED, AND THAT IS NOT A DETAIL - see letter c: an unfiltered
    // capture of one of these internal clock lines over-captures, and
    // the intervals it hands back are then distances between edges that
    // are not neighbours. ICyF = 8 is what makes this a measurement.
    const bool armed = meter_on(RtcDomain::lse_tim16_ti1_code, 0, 8);
    const std::optional<MeterSpread> sp = armed ? meter_quality(64, 400)
                                                : std::nullopt;
    if (sp.has_value()) {
        const uint32_t hz = meter_hz(sp->robust, 0);
        print(serial, "  LSE measured ", hz, " Hz against the core (",
              sp->robust, " timer ticks per period, ", ppm_off(hz, 32768u),
              " ppm off 32768); ", sp->good, " of ", sp->total,
              " intervals were inside the band, spanning ", sp->lo, "..",
              sp->hi, crlf);
        bench.verdict("LSE runs at 32.768 kHz to better than 1 % of the "
                      "core's own scale",
                      ppm_off(hz, 32768u) < 10'000u);
        // THE INSTRUMENT'S OWN QUALITY, on the one signal on this board
        // known to be a clean square wave. What the outliers are is
        // stated where they are counted: a capture the polling loop did
        // not read in time, which can only ever make an interval longer.
        bench.verdict("and the capture path delivers the crystal's period "
                      "on the great majority of intervals - which is what "
                      "every frequency in this suite rests on",
                      sp->good * 10u >= sp->total * 9u);
        bench.verdict("the crystal's period itself does not wander: the "
                      "kept intervals sit inside a handful of core cycles",
                      sp->good != 0u && (sp->kept_hi - sp->kept_lo) < 32u);
    } else {
        bench.verdict("LSE's edges reach TIM16", false);
    }


    // The DRIVE RULE, both directions. 5.2.5: it may be LOWERED under a
    // running oscillator and NOT RAISED.
    const LseDrive d0 = RtcDomain::lse_drive();
    const bool lowered = static_cast<uint8_t>(d0) == 0u ||
                         RtcDomain::lse_drive(LseDrive::low);
    const bool raised = RtcDomain::lse_drive(LseDrive::high);
    bench.verdict("the LSE drive may be lowered under a running crystal",
                  lowered);
    bench.verdict("and may not be raised (5.2.5) - refused, not written",
                  !raised && RtcDomain::lse_drive() != LseDrive::high);
    bench.verdict("LSEBYP is refused while the oscillator runs",
                  !RtcDomain::lse_bypass(true));
}

// =============================================================================
// c - LSI on the instrument, the prescalers, and the calendar's 1 Hz
// =============================================================================
void tc_lsi_and_prescalers() {
    // LSI is always startable, whatever RTCSEL took: it is the IWDG's
    // clock too. Its period at ~32 kHz is about 2000 timer ticks.
    Rcc::lsi_enable(true);
    (void)Rcc::lsi_wait_ready();
    const uint32_t settle = Ticker::millis();
    while (Ticker::millis() - settle < 5u) {
    }

    // TWO READINGS OF THE SAME OSCILLATOR, and the difference between
    // them is this letter's finding: the capture channel's own input
    // filter (ICyF, 21.4.7) sampling the line before it believes an
    // edge. A clean square wave measures the same either way.
    std::optional<MeterSpread> bare;
    std::optional<MeterSpread> filtered;
    if (meter_on(Rcc::lsi_tim16_ti1_code, 0)) {
        bare = meter_quality(64, 400);
    }
    Tim<16>::enable(false);
    if (Meter::setup(0, 8) && Tim<16>::input_select(0, Rcc::lsi_tim16_ti1_code)) {
        (void)Tim<16>::isr();
        Meter::restart();
        filtered = meter_quality(64, 400);
    }

    if (bare.has_value()) {
        print(serial, "  LSI unfiltered: ", meter_hz(bare->median, 0),
              " Hz, intervals ", bare->lo, "..", bare->hi, " timer ticks, "
              "median ", bare->median, ", ",
              bare->good, " of ", bare->total, " inside the band", crlf);
    }
    if (filtered.has_value()) {
        lsi_hz = meter_hz(filtered->median, 0);
        print(serial, "  LSI filtered  : ", lsi_hz, " Hz, intervals ",
              filtered->lo, "..", filtered->hi, " ticks, median ",
              filtered->median, ", ", filtered->good,
              " of ", filtered->total,
              " inside the band (DS13560 table 46 bounds LSI 29500..34000)",
              crlf);
        bench.verdict("LSI, seen through the capture channel's own input "
                      "filter, runs inside the datasheet's 29.5..34 kHz band",
                      within(lsi_hz, 29'500u, 34'000u));
    } else {
        bench.verdict("LSI's edges reach TIM16", false);
    }
    if (bare.has_value() && filtered.has_value()) {
        // TWO READINGS OF ONE OSCILLATOR. What the input filter changes
        // is not the number - it must not, and this is the verdict that
        // says so - but the robustness.
        print(serial, "  filter on and off agree on the MEDIAN interval to ",
              ppm_off(bare->median, filtered->median),
              " ppm; unfiltered the samples ran ", bare->lo, "..", bare->hi,
              " against ", filtered->lo, "..", filtered->hi, " filtered, and ",
              bare->good, " against ", filtered->good,
              " of 64 stayed inside their band", crlf);
        // THE COMPARISON IS MADE ON THE MEDIANS, and the two estimators
        // tried before it are why.
        //
        // A MEAN is dragged up by the missed edges a polling loop that
        // also serves a console cannot avoid: the capture flag stands,
        // the next capture overwrites CCR, and the interval that comes
        // back spans two periods. That failed on a cold run.
        //
        // A MINIMUM was the repair, on the argument that a missed edge
        // can only LENGTHEN an interval so the shortest sample is the
        // period seen from below. THAT ARGUMENT IS FALSE HERE, and this
        // letter is where the silicon says so: the BARE leg over-captures
        // as well, and an extra edge SHORTENS an interval. Caught in the
        // act inside z, unfiltered - sixty-three intervals of 1936..1967
        // ticks and one of 1822, i.e. 70408 ppm below the filtered
        // reading, which failed this verdict about two runs in three
        // while the letter passed every time it was run alone (alone, the
        // console is quiet and the line is sampled in a tighter loop).
        //
        // The median is the only one of the three that both tails have to
        // OUTNUMBER before they can move it, and neither tail here is
        // anywhere near half the samples.
        bench.verdict("the capture channel's own input filter does not move "
                      "the reading - the same oscillator, the same period, "
                      "to better than a per cent",
                      ppm_off(bare->median, filtered->median) < 10'000u);
        bench.verdict("what it moves is the SCATTER: the filtered reading "
                      "keeps the great majority of its intervals inside a "
                      "narrow band",
                      filtered->good * 10u >= filtered->total * 9u);
        // AND THE OVER-CAPTURE ITSELF, PRINTED AND NOT JUDGED. Whether a
        // spurious edge lands in any given 64 samples is luck, so a
        // verdict on it would be the flake this letter has just stopped
        // being; what the reader gets is the number.
        print(serial, "  the bare leg's shortest interval sits ",
              bare->lo < filtered->median
                  ? (filtered->median - bare->lo) : 0u,
              " ticks below the filtered median - a couple of ticks is the "
              "jitter of the two clocks, and anything much above it is an "
              "OVER-capture, an edge the filter would have swallowed", crlf);
    }


    // THE PRESCALER ARITHMETIC, checked against the chapter's own
    // example and against the split this suite runs on.
    constexpr RtcPrescalers p32768 = rtc_prescalers_for(32768);
    print(serial, "  rtc_prescalers_for(32768) = A ", p32768.async, " / S ",
          p32768.sync, " (30.3.4's own 127/255)", crlf);
    bench.verdict("the 32.768 kHz pair is the chapter's 127/255",
                  p32768.async == 127u && p32768.sync == 255u);
    bench.verdict("and it makes ck_spre exactly 1 Hz",
                  rtc_ck_spre_hz(p32768, 32768u) == 1u);
    // 30.3.4's ceiling is 2^22 - "a maximum input frequency of around
    // 4 MHz" - and 5 MHz has no divisor at or below 128 that brings it
    // under the 15-bit synchronous field, so no exact 1 Hz exists.
    bench.verdict("a rate past the chapter's own ~4 MHz ceiling has no pair, "
                  "and says so",
                  rtc_prescalers_for(5'000'000u).async == 0xFFu);


    // Now the calendar itself. Set it going and weigh its second against
    // the kernel tick - which rides HSI16, so this is one RC against
    // another and the band is generous on purpose.
    bench.verdict("the calendar initializes (INIT, prescalers, TR/DR, and "
                  "the erratum workaround on the way out)",
                  set_time(RtcDateTime{.hour = 12, .minute = 0, .second = 0,
                                       .day = 1, .month = 6, .year = 24,
                                       .weekday = 6}));
    const RtcPrescalers in_force = Rtc::prescalers();
    bench.verdict("the prescalers read back what was written",
                  in_force.async == working_prescalers().async &&
                      in_force.sync == working_prescalers().sync);
    bench.verdict("INITS now says the calendar has been set (30.6.4)",
                  Rtc::calendar_set());

    // Wait for a second edge, then time five whole seconds on the
    // kernel's millisecond tick.
    RtcReading r{};
    bench.verdict("the calendar reads coherently", Rtc::read(r));
    const uint8_t s0 = r.time.second;
    while (Rtc::read(r) && r.time.second == s0) {
    }
    const uint32_t t0 = Ticker::millis();
    const uint8_t s1 = r.time.second;
    uint8_t seen = 0;
    uint8_t last = s1;
    while (seen < 5u) {
        if (!Rtc::read(r)) {
            break;
        }
        if (r.time.second != last) {
            last = r.time.second;
            ++seen;
        }
    }
    const uint32_t span = Ticker::millis() - t0;
    print(serial, "  five calendar seconds took ", span,
          " ms of kernel tick (nominal 5000; one RC weighed against "
          "another)", crlf);
    bench.verdict("the calendar's second is a second to better than 5 %",
                  seen == 5u && within(span, 4750u, 5250u));
}

// =============================================================================
// d - the calendar's boundaries, in one second each
// =============================================================================
void td_boundaries() {
    // BCD BOTH WAYS, at compile time and here.
    bench.verdict("the BCD conversion is its own inverse over the field",
                  rtc_from_bcd(rtc_to_bcd(59)) == 59u &&
                      rtc_from_bcd(rtc_to_bcd(23)) == 23u &&
                      rtc_from_bcd(rtc_to_bcd(99)) == 99u);
    // The leap rule of a one-century calendar: divisible by four, with
    // no year-00 exception to apply (there is no century field).
    bench.verdict("29 February exists in a year divisible by four and not "
                  "otherwise",
                  rtc_days_in_month(2, 24) == 29u && rtc_days_in_month(2, 25) == 28u);
    bench.verdict("and rtc_datetime_valid() refuses the date that does not "
                  "exist",
                  rtc_datetime_valid({.day = 29, .month = 2, .year = 24}) &&
                      !rtc_datetime_valid({.day = 29, .month = 2, .year = 25}));

    struct Leg {
        const char* name;
        RtcDateTime from;
        RtcDateTime to;
    };
    const Leg legs[] = {
        {"midnight",
         {.hour = 23, .minute = 59, .second = 59, .day = 5, .month = 3,
          .year = 24, .weekday = 2},
         {.hour = 0, .minute = 0, .second = 0, .day = 6, .month = 3,
          .year = 24, .weekday = 3}},
        {"the end of a 30-day month",
         {.hour = 23, .minute = 59, .second = 59, .day = 30, .month = 4,
          .year = 24, .weekday = 2},
         {.hour = 0, .minute = 0, .second = 0, .day = 1, .month = 5,
          .year = 24, .weekday = 3}},
        {"28 February in a LEAP year (29 February must follow)",
         {.hour = 23, .minute = 59, .second = 59, .day = 28, .month = 2,
          .year = 24, .weekday = 3},
         {.hour = 0, .minute = 0, .second = 0, .day = 29, .month = 2,
          .year = 24, .weekday = 4}},
        {"28 February in a COMMON year (1 March must follow)",
         {.hour = 23, .minute = 59, .second = 59, .day = 28, .month = 2,
          .year = 25, .weekday = 5},
         {.hour = 0, .minute = 0, .second = 0, .day = 1, .month = 3,
          .year = 25, .weekday = 6}},
        {"the end of a year",
         {.hour = 23, .minute = 59, .second = 59, .day = 31, .month = 12,
          .year = 24, .weekday = 2},
         {.hour = 0, .minute = 0, .second = 0, .day = 1, .month = 1,
          .year = 25, .weekday = 3}},
    };

    for (const Leg& leg : legs) {
        if (!set_time(leg.from)) {
            bench.verdict(leg.name, false);
            continue;
        }
        RtcReading r{};
        const uint32_t t0 = Ticker::millis();
        bool rolled = false;
        while (Ticker::millis() - t0 < 2500u) {
            if (Rtc::read(r) && r.time.second == 0u) {
                rolled = true;
                break;
            }
        }
        const RtcDateTime& g = r.time;
        const bool ok = rolled && g.hour == leg.to.hour &&
                        g.minute == leg.to.minute && g.day == leg.to.day &&
                        g.month == leg.to.month && g.year == leg.to.year &&
                        g.weekday == leg.to.weekday;
        print(serial, "  ", leg.name, " becomes ");
        print_datetime(g);
        print(serial, crlf);
        bench.verdict(leg.name, ok);
    }

    // ADD1H / SUB1H: one hour without stopping the calendar (30.3.8).
    (void)set_time(RtcDateTime{.hour = 1, .minute = 30, .second = 0, .day = 1,
                               .month = 6, .year = 24, .weekday = 6});
    RtcReading r{};
    // 30.6.7's last sentence: "ADD1H and SUB1H changes are effective in
    // the NEXT SECOND" - so the reading has to be taken across a second
    // boundary, and a suite that read straight back would report a
    // working register as broken.
    auto next_second = []() {
        RtcReading a{};
        if (!Rtc::read(a)) {
            return;
        }
        const uint8_t s = a.time.second;
        const uint32_t t = Ticker::millis();
        while (Ticker::millis() - t < 2000u) {
            RtcReading b{};
            if (Rtc::read(b) && b.time.second != s) {
                return;
            }
        }
    };
    Rtc::shift_hour(true);
    next_second();
    (void)Rtc::read(r);
    const uint8_t after_add = r.time.hour;
    Rtc::shift_hour(false);
    next_second();
    (void)Rtc::read(r);
    print(serial, "  daylight saving: hour 1 then ", after_add, " then ",
          r.time.hour,
          " (ADD1H then SUB1H, each effective in the next second, calendar "
          "never stopped)", crlf);
    bench.verdict("ADD1H and SUB1H move the hour and come back",
                  after_add == 2u && r.time.hour == 1u);
    Rtc::daylight_flag(true);
    bench.verdict("and RTC_CR's BKP bit remembers that it was done",
                  Rtc::daylight_flag());
    Rtc::daylight_flag(false);
}

// =============================================================================
// e - the shadow registers against BYPSHAD
// =============================================================================
void te_shadow() {
    (void)set_time(RtcDateTime{.hour = 10, .minute = 0, .second = 0, .day = 1,
                               .month = 6, .year = 24, .weekday = 6});

    // SHADOW MODE. RSF says a copy has landed; clearing it and waiting
    // is 30.3.9's own discipline, and the wait is one RTCCLK period.
    Rtc::bypass_shadow(false);
    bench.verdict("BYPSHAD reads back clear", !Rtc::bypass_shadow());
    const uint32_t t0 = Ticker::millis();
    const bool synced = Rtc::wait_sync();
    const uint32_t sync_ms = Ticker::millis() - t0;
    print(serial, "  a shadow re-synchronization took ", sync_ms,
          " ms (one RTCCLK period is about 0.03 ms; the tick is the ruler "
          "here, so 0 or 1 is what a correct one looks like)", crlf);
    bench.verdict("RSF is cleared and comes back", synced && Rtc::synchronized());

    // THE READ COST, both ways. Measured on the kernel's cycle counter
    // by repeating the read a thousand times; a shadow read is a plain
    // APB read of a copy, a bypass read is the double-read this driver
    // performs to make the triple coherent (30.3.9).
    RtcReading r{};
    const uint32_t c0 = Ticker::millis();
    for (uint16_t i = 0; i < 2000u; ++i) {
        (void)Rtc::read(r);
    }
    const uint32_t shadow_ms = Ticker::millis() - c0;

    Rtc::bypass_shadow(true);
    bench.verdict("BYPSHAD reads back set", Rtc::bypass_shadow());
    const uint32_t c1 = Ticker::millis();
    for (uint16_t i = 0; i < 2000u; ++i) {
        (void)Rtc::read(r);
    }
    const uint32_t bypass_ms = Ticker::millis() - c1;
    print(serial, "  2000 coherent readings: ", shadow_ms, " ms through the "
          "shadows, ", bypass_ms, " ms bypassing them", crlf);
    bench.verdict("a bypass reading still comes back coherent",
                  Rtc::read(r) && r.time.hour == 10u);

    // THE SUB-SECOND COUNTER IS A DOWN-COUNTER, reloaded from PREDIV_S
    // (30.3.4). Two readings inside one second must not increase, and
    // the millisecond conversion must land inside the second.
    uint16_t first = 0;
    uint16_t last = 0;
    bool descending = true;
    (void)Rtc::read(r);
    const uint8_t sec = r.time.second;
    first = r.subsecond;
    last = first;
    while (Rtc::read(r) && r.time.second == sec) {
        if (r.subsecond > last) {
            descending = false;
            break;
        }
        last = r.subsecond;
    }
    const uint16_t sync = Rtc::prescalers().sync;
    print(serial, "  sub-second walked ", first, " down to ", last,
          " inside one second (PREDIV_S = ", sync, ", so ",
          rtc_subsecond_ms(last, sync), " ms into it)", crlf);
    bench.verdict("RTC_SSR counts DOWN from PREDIV_S within a second",
                  descending && last <= first);
    bench.verdict("and the millisecond conversion stays inside the second",
                  rtc_subsecond_ms(0, sync) < 1000u &&
                      rtc_subsecond_ms(sync, sync) == 0u);
}

// =============================================================================
// f - the alarms
// =============================================================================
void tf_alarms() {
    Rtc::bypass_shadow(true);
    (void)set_time(RtcDateTime{.hour = 8, .minute = 0, .second = 0, .day = 1,
                               .month = 6, .year = 24, .weekday = 6});

    // ALARM A on a SECONDS match: everything else masked, so it fires
    // once a minute - and with the calendar at second 0 the match at
    // second 5 is five seconds away.
    RtcAlarm a{};
    a.second = 5;
    a.mask_seconds = false;
    bench.verdict("alarm A programs (ALRAE down, ALRAWF up, registers "
                  "written, ALRAE back up)",
                  Rtc::set_alarm(RtcAlarmId::a, a, false));
    bench.verdict("and reads back enabled", Rtc::alarm_enabled(RtcAlarmId::a));

    // THE LATENCY QUESTION. On the SAM an alarm lands a whole counter
    // period after its match; here the comparison is against the
    // calendar, so the flag should land AT the second it names. Measured:
    // wait for the flag, then read the calendar in the same breath.
    Rtc::clear_flags(RtcFlag::alarm_a);
    const uint32_t t0 = Ticker::millis();
    bool fired = false;
    while (Ticker::millis() - t0 < 8000u) {
        if (Rtc::flag(RtcFlag::alarm_a)) {
            fired = true;
            break;
        }
    }
    RtcReading r{};
    (void)Rtc::read(r);
    const uint16_t into_second = rtc_subsecond_ms(r.subsecond, Rtc::prescalers().sync);
    print(serial, "  alarm A fired with the calendar at second ",
          r.time.second, ", ", into_second, " ms into it", crlf);
    bench.verdict("alarm A fires on its match, and it is the second it "
                  "named", fired && r.time.second == 5u);
    bench.verdict("it lands AT the match and not a whole second after it - "
                  "unlike the SAM's counter compare",
                  fired && into_second < 200u);

    // The flag is W1C through RTC_SCR and comes down.
    Rtc::clear_flags(RtcFlag::alarm_a);
    bench.verdict("the flag clears through RTC_SCR", !Rtc::flag(RtcFlag::alarm_a));

    // A SUB-SECOND ALARM. MASKSS = 15 compares all fifteen bits; with
    // the seconds field masked too, the alarm is once per second at a
    // chosen fraction of it.
    RtcAlarm b{};
    b.subsecond_mask = 15;
    b.subsecond = static_cast<uint16_t>(Rtc::prescalers().sync / 2u);
    bench.verdict("alarm B programs with a sub-second comparison",
                  Rtc::set_alarm(RtcAlarmId::b, b, false));
    Rtc::clear_flags(RtcFlag::alarm_b);
    const uint32_t t1 = Ticker::millis();
    bool fired_b = false;
    while (Ticker::millis() - t1 < 3000u) {
        if (Rtc::flag(RtcFlag::alarm_b)) {
            fired_b = true;
            break;
        }
    }
    (void)Rtc::read(r);
    const uint16_t frac = rtc_subsecond_ms(r.subsecond, Rtc::prescalers().sync);
    print(serial, "  alarm B (sub-second match at half a second) fired ",
          frac, " ms into the second", crlf);
    bench.verdict("alarm B fires on a sub-second match",
                  fired_b && within(frac, 400u, 700u));

    // THE CAUTION IS ENFORCED. 30.3.6: a seconds comparison needs
    // PREDIV_S at least 3. A driver that armed one under a smaller
    // prescaler would be arming an undefined comparison.
    Rtc::clear_alarm(RtcAlarmId::a);
    Rtc::clear_alarm(RtcAlarmId::b);
    bench.verdict("both alarms are off again",
                  !Rtc::alarm_enabled(RtcAlarmId::a) &&
                      !Rtc::alarm_enabled(RtcAlarmId::b));
    bench.verdict("a masked-everything alarm is legal (it fires every "
                  "second)",
                  rtc_alarm_valid(RtcAlarm{}));
    bench.verdict("and an impossible one is refused before it reaches a "
                  "register",
                  !rtc_alarm_valid(RtcAlarm{.second = 60, .mask_seconds = false}) &&
                      !rtc_alarm_valid(RtcAlarm{.subsecond_mask = 16}));
}

// =============================================================================
// g - the wake-up timer, weighed on TIM16
// =============================================================================
void tg_wakeup() {
    Rtc::bypass_shadow(true);
    (void)set_time(RtcDateTime{.hour = 9, .minute = 0, .second = 0, .day = 1,
                               .month = 6, .year = 24, .weekday = 6});

    const uint32_t rtcclk =
        source_in_force == RtcClockSource::lse ? 32768u : lsi_hz;

    struct Leg {
        const char* name;
        RtcWakeupClock clock;
        uint32_t reload;
        uint16_t prescaler;   ///< TIM16's, so one period fits 16 bits
    };
    const Leg legs[] = {
        {"RTCCLK/16", RtcWakeupClock::div16, 63, 255},
        {"RTCCLK/8", RtcWakeupClock::div8, 127, 255},
        {"RTCCLK/4", RtcWakeupClock::div4, 255, 255},
        {"RTCCLK/2", RtcWakeupClock::div2, 511, 255},
    };

    for (const Leg& leg : legs) {
        // 25.6.18's footnote: the RTC wake-up reaches TI1 only with the
        // RTC interrupt ENABLED, so the timer is programmed with the
        // interrupt on and the NVIC line left shut - the flag is the
        // capture, not a handler.
        const std::optional<uint32_t> ticks =
            meter_wakeup(leg.clock, leg.reload, leg.prescaler, 4, 3000);
        if (!ticks.has_value()) {
            bench.verdict(leg.name, false);
            continue;
        }
        const uint32_t us = meter_us(*ticks, leg.prescaler);
        const uint32_t predicted = static_cast<uint32_t>(
            (static_cast<uint64_t>(leg.reload + 1u) *
             rtc_wakeup_divider(leg.clock) * 1'000'000ULL) / rtcclk);
        print(serial, "  ", leg.name, " x ", leg.reload + 1u, ": measured ",
              us, " us, predicted ", predicted, " us (", ppm_off(us, predicted),
              " ppm)", crlf);
        bench.verdict(leg.name, ppm_off(us, predicted) < 30'000u);
    }

    // ck_spre: one second per count, from the same prescalers the
    // calendar runs on. One period, measured against the kernel tick
    // rather than TIM16 (a second does not fit a 16-bit counter at any
    // prescaler this timer has).
    bench.verdict("the wake-up timer takes ck_spre too",
                  Rtc::set_wakeup(RtcWakeupClock::ck_spre, 1, false));
    // 30.6.6: "the FIRST assertion of WUTF occurs between WUT and
    // (WUT + 1) ck_wut cycles after WUTE is set" - so the first period
    // is short by up to one count and is NOT the period. The second one
    // is, and that is what is timed here.
    Rtc::clear_flags(RtcFlag::wakeup);
    uint32_t t0 = Ticker::millis();
    bool first = false;
    while (Ticker::millis() - t0 < 6000u) {
        if (Rtc::flag(RtcFlag::wakeup)) {
            first = true;
            break;
        }
    }
    const uint32_t first_span = Ticker::millis() - t0;
    Rtc::clear_flags(RtcFlag::wakeup);
    t0 = Ticker::millis();
    bool second = false;
    while (Ticker::millis() - t0 < 6000u) {
        if (Rtc::flag(RtcFlag::wakeup)) {
            second = true;
            break;
        }
    }
    const uint32_t span = Ticker::millis() - t0;
    Rtc::clear_wakeup();
    print(serial, "  ck_spre x 2: the FIRST period measured ", first_span,
          " ms, the second ", span, " ms (nominal 2000; 30.6.6 says the "
          "first may be a whole count short)", crlf);
    bench.verdict("a ck_spre wake-up of two counts is two seconds from the "
                  "second period on",
                  first && second && within(span, 1800u, 2300u));
    bench.verdict("and the FIRST one is shorter, exactly as 30.6.6 warns",
                  first_span <= span + 50u);

    print(serial, "  the RTC's vector ran ", rtc_interrupts,
          " times during this letter - which is what keeps the wake-up "
          "LINE falling between assertions (25.6.18)", crlf);
    bench.verdict("the RTC's own vector serviced every assertion",
                  rtc_interrupts > 20u);

    // THE WRITE WINDOW AND THE TWO REFUSALS.
    bench.verdict("WUTWF stands while the timer is stopped (30.6.4)",
                  Rtc::wakeup_write_allowed());
    bench.verdict("a reload past sixteen bits is refused",
                  !rtc_wakeup_valid(RtcWakeupClock::div16, 0x10000u));
    bench.verdict("and RTCCLK/2 with a reload of zero is refused - 30.6.6 "
                  "calls it forbidden",
                  !rtc_wakeup_valid(RtcWakeupClock::div2, 0u) &&
                      rtc_wakeup_valid(RtcWakeupClock::div4, 0u));
}

// =============================================================================
// h - ES0548 2.9.1 staged
// =============================================================================
void th_erratum() {
    // The erratum: INIT set between one and two RTCCLK cycles after
    // being cleared sets INITF immediately instead of waiting for the
    // synchronization, and calendar writes made in that window may be
    // dropped or corrupted. The workaround is to wait for a shadow
    // re-synchronization between the two entries, which exit_init()
    // does unconditionally.
    //
    // The two paths are run back to back, sixteen times each, writing a
    // DIFFERENT time on every pass and reading it back.
    constexpr uint16_t rounds = 16;
    Rtc::bypass_shadow(false);   // the workaround's own precondition matters

    uint16_t guarded_bad = 0;
    uint16_t raw_bad = 0;

    for (uint16_t i = 0; i < rounds; ++i) {
        const uint8_t sec = static_cast<uint8_t>(i % 60u);
        const RtcDateTime want{.hour = 7, .minute = 7, .second = sec,
                               .day = 2, .month = 7, .year = 24, .weekday = 2};
        // GUARDED: the driver's own exit, which waits out the erratum's
        // window before anything can enter initialization again.
        if (!Rtc::init(working_prescalers(), want)) {
            ++guarded_bad;
            continue;
        }
        RtcReading r{};
        if (!Rtc::read(r) || r.time.second != sec || r.time.minute != 7u ||
            r.time.hour != 7u || r.time.day != 2u) {
            ++guarded_bad;
        }
    }

    for (uint16_t i = 0; i < rounds; ++i) {
        const uint8_t sec = static_cast<uint8_t>((i + 30u) % 60u);
        const RtcDateTime want{.hour = 6, .minute = 6, .second = sec,
                               .day = 3, .month = 8, .year = 24, .weekday = 3};
        // RAW: enter, write, leave WITHOUT the workaround, and enter
        // again immediately - the erratum's exact recipe.
        if (!Rtc::enter_init()) {
            ++raw_bad;
            continue;
        }
        (void)Rtc::set_prescalers(working_prescalers());
        (void)Rtc::set_calendar(want);
        Rtc::exit_init_raw();
        // No wait at all: straight back in, which is the window.
        if (!Rtc::enter_init()) {
            ++raw_bad;
            continue;
        }
        Rtc::exit_init_raw();
        if (!Rtc::wait_sync()) {
            ++raw_bad;
            continue;
        }
        RtcReading r{};
        if (!Rtc::read(r) || r.time.second != sec || r.time.minute != 6u ||
            r.time.hour != 6u || r.time.day != 3u || r.time.month != 8u) {
            ++raw_bad;
        }
    }

    print(serial, "  ", rounds, " guarded initializations: ", guarded_bad,
          " bad; ", rounds, " raw back-to-back entries: ", raw_bad, " bad",
          crlf);
    bench.verdict("the GUARDED path (the driver's exit_init, which spends "
                  "the erratum's window) never corrupts a calendar",
                  guarded_bad == 0u);
    // The erratum is a RACE and it is not obliged to bite in sixteen
    // tries; what this letter can claim is what it saw, and it prints
    // the count either way rather than pretending.
    print(serial, "  ES0548 2.9.1 ", raw_bad != 0u ? "REPRODUCED" : "did NOT reproduce",
          " on the unguarded path in this run", crlf);
    bench.verdict("and the unguarded path is at least as bad as the guarded "
                  "one - the workaround never costs correctness",
                  raw_bad >= guarded_bad);

    Rtc::bypass_shadow(true);
    (void)set_time(RtcDateTime{.hour = 0, .minute = 0, .second = 0, .day = 1,
                               .month = 1, .year = 24, .weekday = 1});
}

// =============================================================================
// i - smooth calibration, measured
// =============================================================================
void ti_calibration() {
    // TWO INSTRUMENTS, and the difference between them is this letter's
    // finding. Both are wake-up periods captured on TIM16 at 32 us per
    // tick, long enough that a few hundred ppm is many ticks, and both
    // are measured as a DIFFERENCE so the core clock's own 1 % absolute
    // error cancels:
    //   - ck_spre, one second per count: the CALENDAR's own clock, which
    //     is what the smooth calibrator is documented to move;
    //   - RTCCLK/16 with a reload of 4095, also about two seconds: the
    //     wake-up timer's OTHER input, which taps the chain at a
    //     different place.
    constexpr uint16_t psc = 2047;   // 32 us per timer tick

    auto weigh_spre = [](const RtcCalibration& c) -> std::optional<uint32_t> {
        if (!Rtc::calibrate(c)) {
            return std::nullopt;
        }
        return meter_wakeup(RtcWakeupClock::ck_spre, 0, psc, 4, 30'000);
    };
    auto weigh_fast = [](const RtcCalibration& c) -> std::optional<uint32_t> {
        if (!Rtc::calibrate(c)) {
            return std::nullopt;
        }
        return meter_wakeup(RtcWakeupClock::div16, 4095, psc, 3, 30'000);
    };

    const std::optional<uint32_t> zero = weigh_spre(RtcCalibration{});
    const std::optional<uint32_t> slow =
        weigh_spre(RtcCalibration{.plus = false, .minus = 511});
    const std::optional<uint32_t> fast =
        weigh_spre(RtcCalibration{.plus = true, .minus = 0});
    (void)Rtc::calibrate(RtcCalibration{});

    if (!zero.has_value() || !slow.has_value() || !fast.has_value()) {
        bench.verdict("the calibration legs were measurable", false);
        return;
    }
    print(serial, "  ck_spre period in 32 us ticks: uncalibrated ", *zero,
          ", CALM=511 ", *slow, ", CALP ", *fast, crlf);
    // A masked pulse makes the calendar SLOWER, so the period gets
    // LONGER; an inserted one makes it faster and shorter.
    const uint32_t swing_ppm = ppm_off(*slow, *fast);
    print(serial, "  full swing ", swing_ppm,
          " ppm, where 30.3.13's own range is 975 (-487.1 .. +488.5)", crlf);
    bench.verdict("CALM slows the calendar down: its second grows",
                  *slow > *zero);
    bench.verdict("CALP speeds it up: its second shrinks", *fast < *zero);
    bench.verdict("and the full swing is the chapter's ~975 ppm to within a "
                  "quarter of itself",
                  within(swing_ppm, 730u, 1220u));

    // THE OTHER TAP. The same two extremes, measured on a wake-up period
    // built from RTCCLK/16 rather than from ck_spre.
    const std::optional<uint32_t> fslow =
        weigh_fast(RtcCalibration{.plus = false, .minus = 511});
    const std::optional<uint32_t> ffast =
        weigh_fast(RtcCalibration{.plus = true, .minus = 0});
    (void)Rtc::calibrate(RtcCalibration{});
    if (fslow.has_value() && ffast.has_value()) {
        const uint32_t fswing = ppm_off(*fslow, *ffast);
        print(serial, "  the same two settings on RTCCLK/16: ", *fslow,
              " and ", *ffast, " ticks, a swing of ", fswing, " ppm", crlf);
        bench.verdict("THE SMOOTH CALIBRATION DOES NOT REACH THE DIVIDED "
                      "RTCCLK WAKE-UP CLOCKS - it masks pulses into the "
                      "prescalers, so ck_spre moves and RTCCLK/16 does not",
                      fswing < swing_ppm / 3u);
    } else {
        bench.verdict("the divided-clock control legs were measurable", false);
    }

    bench.verdict("rtc_calibration_ppb() agrees with the register's own "
                  "arithmetic at both ends",
                  rtc_calibration_ppb({.plus = true, .minus = 0}) > 480'000 &&
                      rtc_calibration_ppb({.plus = false, .minus = 511}) < -480'000);
    // 30.6.9's two stuck-bit notes, as refusals.
    bench.verdict("a CALM the 8-second window would quietly round is "
                  "refused",
                  !rtc_calibration_valid({.minus = 3,
                                          .window = RtcCalibrationWindow::seconds8}) &&
                      rtc_calibration_valid({.minus = 4,
                                             .window = RtcCalibrationWindow::seconds8}));
}

// =============================================================================
// j - the backup registers
// =============================================================================
void tj_backup() {
    print(serial, "  this part carries ", Tamp::backup_count,
          " backup registers (BKP0R..BKP", Tamp::backup_count - 1u, "R)", crlf);
    bench.verdict("the count is read off the device header's own struct",
                  Tamp::backup_count == 5u);

    for (uint8_t i = 0; i < Tamp::backup_count; ++i) {
        (void)Tamp::backup(i, 0xB0000000u | i);
    }
    bool all = true;
    for (uint8_t i = 0; i < Tamp::backup_count; ++i) {
        all = all && Tamp::backup(i) == (0xB0000000u | i);
    }
    bench.verdict("all five words hold what was written", all);
    bench.verdict("an index past the end is refused, not wrapped",
                  !Tamp::backup(Tamp::backup_count, 1u) &&
                      Tamp::backup(Tamp::backup_count) == 0u);

    // THE RTC'S KEY DOES NOT COVER THEM. 30.3.8's protected list is the
    // RTC's own registers; TAMP is a different block. So a write lands
    // with the RTC locked - which is worth proving, because it is the
    // difference between "one lock" and "two locks with different
    // scopes".
    Rtc::lock();
    (void)Tamp::backup(0, 0xC0FFEE00u);
    const bool through_rtc_lock = Tamp::backup(0) == 0xC0FFEE00u;
    bench.verdict("the RTC's WPR key does not cover the backup registers: a "
                  "write lands with the RTC locked",
                  through_rtc_lock);

    // DBP DOES. It is the domain's gate and the backup registers are in
    // the domain.
    Pwr::rtc_domain_unlock(false);
    (void)Tamp::backup(1, 0xDEADBEEFu);
    const uint32_t under_dbp = Tamp::backup(1);
    Pwr::rtc_domain_unlock(true);
    bench.verdict("but PWR_CR1.DBP does: a write with the domain locked is "
                  "dropped",
                  under_dbp != 0xDEADBEEFu);

    // The tamper half is decoded and nothing is armed.
    print(serial, "  TAMP: CR1=", hex(Tamp::config1()), " IER=",
          hex(Tamp::interrupts()), " SR=", hex(Tamp::status()), crlf);
    bench.verdict("no tamper input is armed on this board, so nothing is "
                  "waiting to erase these five words",
                  !Tamp::any_armed());
}

// =============================================================================
// v - survival across a real reset (outside z)
// =============================================================================
void tv_survival() {
    if (token.magic == token_magic && token.leg == 1u) {
        // SECOND HALF: we are the boot after the software reset.
        token.leg = 0;
        bench.resume_tally(token.pass, token.fail);
        print(serial, "  came back from a software reset; RCC_CSR flags=",
              hex(boot_flags), crlf);
        bench.verdict("the reset really happened (SFTRSTF stands)",
                      (boot_flags & ResetFlag::software) != 0u);
        bool all = true;
        for (uint8_t i = 0; i < Tamp::backup_count; ++i) {
            all = all && Tamp::backup(i) == token.written[i];
        }
        print(serial, "  BKP0R..BKP4R after the reset:");
        for (uint8_t i = 0; i < Tamp::backup_count; ++i) {
            print(serial, " ", hex(Tamp::backup(i)));
        }
        print(serial, crlf);
        bench.verdict("ALL FIVE BACKUP REGISTERS SURVIVED THE RESET - the "
                      "RTC domain is not in a system reset's scope (30.3.10)",
                      all);
        bench.verdict("and so did the calendar: RCC_BDCR still names its "
                      "source and INITS still stands",
                      RtcDomain::enabled() && Rtc::calendar_set());
        RtcReading r{};
        if (Rtc::read(r)) {
            print(serial, "  the calendar reads ");
            print_datetime(r.time);
            print(serial, crlf);
        }
        bench.end_letter();
        return;
    }

    // FIRST HALF: write, record, reboot.
    Pwr::bus_clock(true);
    Pwr::rtc_domain_unlock(true);
    RtcDomain::apb_clock(true);
    if (!RtcDomain::enabled()) {
        (void)RtcDomain::open(RtcClockSource::lsi);
        Rcc::lsi_enable(true);
        (void)Rcc::lsi_wait_ready();
        (void)Rtc::init(rtc_prescalers_for(32000),
                        RtcDateTime{.hour = 4, .minute = 20, .second = 0,
                                    .day = 9, .month = 9, .year = 24,
                                    .weekday = 1});
    }
    for (uint8_t i = 0; i < Tamp::backup_count; ++i) {
        const uint32_t v = 0x5A000000u | (static_cast<uint32_t>(i) << 8) | i;
        (void)Tamp::backup(i, v);
        token.written[i] = v;
    }
    bench.verdict("five witnesses written into the backup registers", true);
    token.magic = token_magic;
    token.leg = 1;
    token.pass = bench.passed();
    token.fail = bench.failed();
    print(serial, "  resetting the board now; the second half resumes here",
          crlf);
    console_drain();
    Reset::software();
}

// ---------------------------------------------------------------------------
// The menu
// ---------------------------------------------------------------------------

void banner() {
    print(serial, crlf,
          "test_stm32_rtc - the RTC domain (board E, no wires)", crlf);
    bench.menu();
    print(serial, "  z  run them all", crlf);
}

}   // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void USART2_LPUART2_IRQHandler() { (void)Serial::isr(); }
// The RTC's whole vector - the wake-up, both alarms, the timestamp -
// arrives here (table 61). Its only job in this suite is to be PROMPT:
// the wake-up source that TIM16 captures is the interrupt LINE, and the
// line only falls again when the flag is acknowledged.
extern "C" void RTC_TAMP_IRQHandler() {
    rtc_interrupts = rtc_interrupts + 1u;
    (void)brio::Rtc::isr();
}

int main() {
    const bool clock_ok = SysClock::init();
    boot_flags = brio::Reset::take_flags();
    brio::Pwr::bus_clock(true);
    brio::Pwr::rtc_domain_unlock(true);
    brio::RtcDomain::apb_clock(true);
    boot_bdcr = brio::RtcDomain::bdcr();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    const bool resuming = token.magic == token_magic && token.leg == 1u;

    bench.letter('a', "the domain: DBP, the WPR keys, one-way RTCSEL, BDRST",
                 ta_domain);
    bench.letter('b', "LSE: does the crystal run, and what does it weigh",
                 tb_lse);
    bench.letter('c', "LSI on TIM16, the prescalers, the calendar's 1 Hz",
                 tc_lsi_and_prescalers);
    bench.letter('d', "the calendar's boundaries, one second each",
                 td_boundaries);
    bench.letter('e', "the shadow registers against BYPSHAD", te_shadow);
    bench.letter('f', "both alarms: the match, the latency, the masks",
                 tf_alarms);
    bench.letter('g', "the wake-up timer on four clocks, weighed on TIM16",
                 tg_wakeup);
    bench.letter('h', "ES0548 2.9.1 staged: guarded against raw", th_erratum);
    bench.letter('i', "smooth calibration, measured", ti_calibration);
    bench.letter('j', "the five backup registers and the two locks over them",
                 tj_backup);
    bench.letter('v', "SURVIVAL: the backup registers across a real reset",
                 tv_survival, false);

    if (resuming) {
        // The reboot letter drives itself: resume, judge, print the ALL:
        // line, and stop.
        tv_survival();
        bench.prompt();
        for (;;) {
            uint8_t c = 0;
            if (!Serial::read_byte(c) || c == '\r' || c == '\n') {
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

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "PLL 64 MHz" : "FAILED",
              " tick=", tick_ok ? "SysTick" : "FAILED", " BDCR=", hex(boot_bdcr),
              crlf);
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
