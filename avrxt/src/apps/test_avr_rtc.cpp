// test_avr_rtc - the RTC/PIT test SUITE for the AVR DA/DB target: the
// real-time counter and the periodic interrupt timer measured against
// the 24 MHz main clock - the DB's crystal, the DA's external quartz
// oscillator on EXTCLK, crystal-referenced either way ("crystal ticks"
// below) - with no wires at all. A TCB cascade (TCB1+TCB2,
// 32 bits at CLK_PER) is the stopwatch; the RTC's own OVF and CMP
// events latch it through an event channel, so every interval is
// timestamped by hardware and the CPU only reads the latches.
// Reference test of avrxt/rtc.hpp and avrxt/ticker.hpp
// (docs/avrxt/rtc.md): keep it passing.
//
// Bench diagnostic, NOT a kernel app (sequential, blocking). Console on
// USART2 ALT1 (PF4/PF5) at 460800. No pin is driven, nothing to wire.
// Event channels: 3 = the PIT divider level (odd channel: PIT_DIV64),
// 4 = the cascade's carry, 5 = the cascade's snapshot, re-sourced per
// test (software pulses, RTC OVF, RTC CMP).
//
// Commands: ? | 1 resource verbs | 2 counter period | 3 compare phase
// | 4 crystal error correction | 5 busy flags | 6 first tick
// | 7 PIT and counter together | 8 OSC1K | 9 XOSC32K (the boards with
// the crystal) | p PIT event slots (the AVR EA) | a all
//
// Tests:
//   1  the verbs: clock select and readback, init/enable, CNT advancing
//      at the expected rate, PER/CMP/CNT write-read, prescaler, the
//      OVF/CMP interrupts on their shared vector, the calibrate()
//      refusals (negative correction at DIV1, +-128 ppm), the PIT verbs;
//   2  the counter's period in crystal ticks at DIV1/DIV2/DIV32 with PER
//      chosen so all three mean 32768 CLK_RTC cycles: each within the
//      OSC32K's 10 % (a wrong division is a factor of two), their
//      agreement and the OSC32K's real rate in ppm printed;
//   3  the compare phase: OVF stamps give the period, then the same
//      channel re-sourced to CMP gives the phase - CMP + 1 RTC ticks
//      after the overflow, for CMP = 8191 and CMP = 99, printed and
//      judged within the 22 % two windows of the oscillator can differ by;
//   4  crystal error correction: quarter-second counter periods measured
//      with the trim in force against periods without it, alternating in
//      swapped order, at +127 and -127 ppm (prescaler DIV2 - a negative
//      correction needs it), judged against the pairs' own standard
//      error; readback of the trim in force;
//   5  the synchronization busy flags: CNT/PER/CMP/CTRLA and the PIT's
//      CTRLBUSY, each timed in crystal ticks from the write to the flag
//      clearing (expected: two CLK_RTC periods and the write's phase);
//   6  the first tick after an enable: 32 enable cycles timed from the
//      PITEN write to the first PI flag, with the prescaler stopped
//      (both functions off) and free-running (the counter left on) -
//      every one inside a full period, and the spread is the point;
//   7  no interference: the counter's period with the PIT running and
//      with it stopped (printed), the Ticker's ticks over one counter period
//      (exactly 1024), and PIT_DIV64 measured while the counter's
//      prescaler changes - the PIT does not see PRESCALER;
//   8  OSC1K as CLK_RTC: the same 1 s from a 1.024 kHz clock (PER =
//      1023), and the Ticker's rate divided by 32.
//   9  XOSC32K as CLK_RTC (the board with a 32.768 kHz crystal: the
//      AVR128DA48 board; the AVR128DB48 board fits none and its build
//      has no letter 9; the AVR64EA48 board's is fitted but does not run,
//      board.hpp's board::xosc32k_runs, and the letter declines there with
//      that reason): preferred() names the crystal once it runs, the
//      counter's period against the 24 MHz external clock (printed: no
//      tolerance on record for either) - forced on, then held by the RTC's
//      request alone - and the Ticker's default init taking the crystal,
//      1024 ticks to its second.
//   p  the PIT's event slots (the AVR EA, RTC.PITEVGENCTRLA): every
//      divider 4..32768 through each slot, its period against the OSC32K
//      second just measured; both slots at once; the divider chain still
//      while neither RTCEN nor PITEN is set, running with PITEN (with
//      RTCEN alone printed); a slot turned off carries nothing.
// Not testable here: the RTC's external clock (no 32 kHz clock on either
// board's XTAL32K1; RtcSource::extclk is the EXTCLK pin, whose 24 MHz on
// the DA board is far past the CLK_PER >= 4 x CLK_RTC rule of 26.3),
// RUNSTDBY and the PIT as a power-down wake-up (no sleep in this suite),
// DBGRUN (needs a halted CPU).

// build: boards = db48,da48,ea48
// build: monitor_speed = 460800

#include <avr/interrupt.h>
#include <stdint.h>

#include "avrxt/clock.hpp"
#include "avrxt/delay.hpp"
#include "avrxt/evsys.hpp"
#include "avrxt/rtc.hpp"
#include "avrxt/tcb.hpp"
#include "avrxt/ticker.hpp"
#include "avrxt/usart.hpp"
#include "avrxt/userrow.hpp"
#include "util/print.hpp"
#include "util/test_wait.hpp"

#include "board.hpp"

using SysClock = board::Clock;   // the board's HF source (board.hpp)
constexpr SysClock clock;

namespace {

using namespace brio;

using Serial = Uart<2, Route::alt1>;
constexpr Serial serial;

// ---- the instrument ----------------------------------------------------------
using T0 = Tcb<0>;                   // the PIT-divider meter
using T1 = Tcb<1>;                   // the stopwatch, low half
using T2 = Tcb<2>;                   // the stopwatch, high half
using Wide = CascadedCounter<T1, T2>;

using ChPit = EventChannel<3>;       // PIT_DIV64 (odd channel)
using ChCarry = EventChannel<4>;
using ChSnap = EventChannel<5>;      // software / RTC OVF / RTC CMP

constexpr uint32_t crystal_hz = SysClock::hz;
// The board's facts (board::clkout, board::xosc32k, board::pa7_free) are
// the board file's (board.hpp).
constexpr uint32_t nominal_hz = 32768;           // CLK_RTC from OSC32K
// Crystal ticks in one nominal CLK_RTC cycle (24e6 / 32768 = 732.42).
constexpr uint32_t ticks_per_rtc_x100 = (crystal_hz * 100ul) / nominal_hz;
/// The letters' text was first written at 24 MHz and keeps it there; at
/// another board rate the same verdicts are derived from the rate.
constexpr bool at24 = crystal_hz == 24'000'000u;
/// The OSC32K's band, the one figure the data sheets give for it: within
/// 10 % of 32768 Hz over the full operating range (DS40002247B table 39-9,
/// 29.491..36.045 kHz; DS40002443A table 35-10, "<10 %"). Two windows of
/// the oscillator a few seconds apart also differ by its WANDER, which no
/// data sheet states: a comparison that only the wander could bound is
/// printed, never judged. Two measurements each within the band are within
/// a ratio of 0.9 / 1.1 of each other: 2/9 (22 %) of the first, the bound
/// of every same-oscillator comparison judged below - wide, and enough to
/// tell a divider from its neighbour (a factor of two).
constexpr uint32_t osc32k_band(uint32_t nominal) { return nominal / 10u; }
constexpr uint32_t osc32k_ratio_band(uint32_t first) { return first / 9u * 2u; }

volatile uint16_t rtc_ovf_irqs = 0;
volatile uint16_t rtc_cmp_irqs = 0;
volatile uint16_t captures = 0;
volatile uint16_t cap_a = 0;
volatile bool meter_on = false;
#if defined(RTC_EVGEN0SEL_gm)
// Letter p's averaging: the sum, the shortest and the longest of the
// periods captured since the last clear.
volatile uint32_t cap_sum = 0;
volatile uint16_t cap_min = 0xFFFF, cap_max = 0;
#endif

// ---- tiny test harness --------------------------------------------------------
uint8_t passed = 0, failed = 0, skipped = 0;

void verdict(const char* name, bool ok) {
    if (ok) ++passed; else ++failed;
    print(serial, "  ", ok ? "PASS" : "FAIL", "  ", name, crlf);
}
/// A check this board cannot run, counted apart from passes and failures
/// in TestBench's grammar (util/testbench.hpp, "SKIP  <reason>").
void skip(const char* reason) {
    if (skipped != 0xFFu) ++skipped;
    print(serial, "  SKIP  ", reason, crlf);
}
/// A tally line: "N pass, M fail", then ", K skipped" when K is not zero.
void tally(const char* head, uint16_t p, uint16_t f, uint16_t s) {
    print(serial, head, p, " pass, ", f, " fail");
    if (s != 0u) print(serial, ", ", s, " skipped");
    print(serial, crlf);
}
bool near(int32_t a, int32_t b, int32_t tol) {
    const int32_t d = a > b ? a - b : b - a;
    return d <= tol;
}
// Holds are counted in CRYSTAL time (delay_us, a cycle loop), never by
// the Ticker: the RTC is the device under test, and quiesce() leaves the
// tick paused. For the same reason no wait here is on the kernel tick:
// the ones the cascade can time go on Ruler below, the ones it cannot
// (its snapshot channel carrying an RTC event) on this delay verb.
void hold_ms(uint32_t ms) {
    while (ms--) delay_us(clock, 1000);
}

// ---- the stopwatch ------------------------------------------------------------
// The cascade free-runs at CLK_PER for the whole suite (32 bits =
// 178 s before it wraps; every interval below is a difference, so a
// wrap inside one is harmless). Stamps come two ways: a software pulse
// on the snapshot channel (now()), or the RTC event the channel is
// sourced with (stamp(), latched by hardware - no interrupt latency).
void stopwatch_init() {
    Wide::init(TcbClock::div1, ChCarry{}, ChSnap{});
    Wide::reset();
}
uint32_t now() {
    ChSnap::off();
    return Wide::read();
}
/// The cascade as a CycleRuler (util/test_wait.hpp): now()'s software
/// stamp at the board rate. It takes the snapshot channel off whatever
/// RTC event it carried, so it times only the waits of letters that do
/// not stamp through the channel (5, 6 and 9's oscillator waits).
struct Ruler {
    static uint32_t now() {
        ChSnap::off();
        return Wide::read();
    }
    static constexpr uint32_t hz() { return crystal_hz; }
};
/// A stamp latched by the RTC event the channel carries, waited for at
/// most `timeout_ms` milliseconds. The bound is the delay verb's, a
/// millisecond a turn on the crystal: the cascade cannot time a wait for
/// its own event stamp (a software snapshot would be taken for one) and
/// the tick is paused (the file's hold_ms()).
bool stamp(uint32_t& out, uint32_t timeout_ms) {
    T1::clear_capt();
    T2::clear_capt();
    while (timeout_ms--) {
        if (T1::capt_flag() && T2::capt_flag()) {
            out = Wide::captured();
            return true;
        }
        delay_us(clock, 1000);
    }
    return false;
}
// n consecutive event-latched intervals, total crystal ticks.
bool measure(uint8_t n, uint32_t& total, uint32_t timeout_ms) {
    uint32_t first = 0, last = 0;
    if (!stamp(first, timeout_ms)) return false;
    for (uint8_t i = 0; i < n; ++i) {
        if (!stamp(last, timeout_ms)) return false;
    }
    total = last - first;
    return true;
}
// A measured interval against its nominal length, in ppm (positive =
// the RTC is SLOW, i.e. the interval took longer than nominal).
int32_t ppm_of(uint32_t measured, uint32_t nominal) {
    return static_cast<int32_t>(
        (static_cast<int64_t>(measured) - nominal) * 1'000'000 / nominal);
}

void quiesce() {
    Rtc::enable_ovf_interrupt(false);
    Rtc::enable_cmp_interrupt(false);
    Rtc::disable();
    Rtc::calibrate(0);
    T0::disable();
    T0::enable_capt_interrupt(false);
    meter_on = false;
    ChPit::off();
    ChSnap::off();
    RtcClock::select(RtcSource::osc32k);
    Ticker::init();                    // PIT back at 1024 Hz ...
    Ticker::pause();                   // ... with its interrupt masked
    rtc_cmp_irqs = 0;
    rtc_ovf_irqs = rtc_cmp_irqs;
    captures = 0;
    stopwatch_init();
}

// ---- 1 the verbs ---------------------------------------------------------------
void t1_verbs() {
    print(serial, "1 RtcClock / Rtc / Pit verbs", crlf);
    quiesce();

    RtcClock::select(RtcSource::osc32k);
    verdict("CLKSEL reads back", RtcClock::selected() == RtcSource::osc32k);
    verdict("source rate 32768 Hz", RtcClock::hz() == 32768u);

    verdict("init accepts the config",
            Rtc::init({.prescaler = RtcPrescaler::div1, .period = 32767, .compare = 1000}));
    verdict("enabled", Rtc::enabled());
    verdict("prescaler reads back", Rtc::prescaler() == RtcPrescaler::div1);
    verdict("PER reads back", Rtc::period() == 32767);
    verdict("CMP reads back", Rtc::compare() == 1000);
    verdict("tick_hz = 32768", Rtc::tick_hz() == 32768u);

    const uint16_t c0 = Rtc::count();
    hold_ms(100);
    const uint16_t c1 = Rtc::count();
    print(serial, "  CNT after 100 ms: ", c0, " -> ", c1, " (expect +3277)", crlf);
    // 3277 +- 328 (the OSC32K's 10 %), + 3 for hold_ms()'s loop overhead
    // (under 0.1 % of the hold) and 1 for each read's phase: +-333.
    verdict("CNT advances at CLK_RTC (+-10 %)", near(static_cast<int32_t>(c1) - c0, 3277, 333));
    Rtc::count(0);
    // The write takes effect two CLK_RTC cycles after it (the CNT register
    // description) plus up to one of phase; CNTBUSY reads clear once it has,
    // and the read after the poll sees at most one count more: 0..4.
    verdict("CNT written to 0", Rtc::sync() && Rtc::count() <= 4);

    // the shared vector: OVF and CMP both raise it
    Rtc::init({.prescaler = RtcPrescaler::div32, .period = 1023, .compare = 511});
    Rtc::enable_ovf_interrupt(true);
    Rtc::enable_cmp_interrupt(true);
    rtc_cmp_irqs = 0;
    rtc_ovf_irqs = rtc_cmp_irqs;
    hold_ms(3100);                        // 3 periods of 1 s, plus slack
    print(serial, "  in 3.1 s: ", rtc_ovf_irqs, " OVF, ", rtc_cmp_irqs, " CMP interrupts", crlf);
    // init() zeroes CNT: the overflows land at k x T and the compares at
    // (k - 1/2) x T, T = 1 s within 10 % (0.909..1.111 s). In 3.1 s that is
    // 2 or 3 overflows (3.33 s is out at the slow end) and exactly 3
    // compares (0.45..2.78 s in, 3.18 s out at the fast end).
    verdict("OVF interrupts (2 or 3)", rtc_ovf_irqs == 2 || rtc_ovf_irqs == 3);
    verdict("CMP interrupts (3)", rtc_cmp_irqs == 3);
    Rtc::enable_ovf_interrupt(false);
    Rtc::enable_cmp_interrupt(false);

    // crystal error correction: the legality rules
    Rtc::prescaler(RtcPrescaler::div1);
    verdict("positive trim accepted at DIV1", Rtc::calibrate(100));
    verdict("trim reads back", Rtc::calibration_ppm() == 100 && Rtc::correcting());
    verdict("negative trim REFUSED at DIV1", !Rtc::calibrate(-100));
    verdict("the refused call changed nothing", Rtc::calibration_ppm() == 100);
    Rtc::prescaler(RtcPrescaler::div2);
    verdict("negative trim accepted at DIV2", Rtc::calibrate(-100));
    verdict("negative trim reads back", Rtc::calibration_ppm() == -100);
    verdict("-128 ppm refused (7-bit magnitude)", !Rtc::calibrate(-128));
    verdict("0 disables the correction", Rtc::calibrate(0) && !Rtc::correcting());
    verdict("config form refuses the illegal pair",
            !Rtc::init({.prescaler = RtcPrescaler::div1, .correction_ppm = -1}));

    // run-standby / debug-run bits survive their verbs
    Rtc::run_standby(true);
    verdict("RUNSTDBY set", Rtc::run_standby());
    Rtc::run_standby(false);
    verdict("RUNSTDBY cleared", !Rtc::run_standby());
    Rtc::debug_run(true);
    verdict("DBGRUN set", Rtc::debug_run());
    Rtc::debug_run(false);

    // the PIT half
    Pit::init(PitPeriod::cyc64, false);
    verdict("PIT enabled", Pit::enabled());
    verdict("PIT period reads back", Pit::period() == PitPeriod::cyc64);
    verdict("PIT rate = 32768/64 = 512 Hz", Pit::tick_hz() == 512u);
    verdict("PIT interrupt off as asked", !Pit::interrupt_enabled());
    Pit::clear_flag();
    hold_ms(5);
    verdict("PI flag rises without the interrupt", Pit::flag());
    verdict("take_flag reports and clears", Pit::take_flag() && !Pit::flag());
    Pit::debug_run(true);
    verdict("PIT DBGRUN set", Pit::debug_run());
    Pit::debug_run(false);
    Pit::disable();
    verdict("PIT disabled", !Pit::enabled());
    quiesce();
}

// ---- 2 the counter's period ------------------------------------------------------
void t2_period() {
    print(serial, "2 counter period against the crystal (32768 CLK_RTC cycles three ways)", crlf);
    quiesce();
    ChSnap::source(Rtc::OvfEvent{});

    struct Case { RtcPrescaler p; uint16_t per; const char* name; };
    const Case cases[] = {
        {RtcPrescaler::div1, 32767, "DIV1  PER=32767"},
        {RtcPrescaler::div2, 16383, "DIV2  PER=16383"},
        {RtcPrescaler::div32, 1023, "DIV32 PER=1023 "},
    };
    uint32_t seen[3] = {0, 0, 0};
    uint8_t k = 0;
    for (const Case& c : cases) {
        Rtc::init({.prescaler = c.p, .period = c.per});
        uint32_t total = 0;
        const bool got = measure(1, total, 1500);
        seen[k++] = total;
        print(serial, "  ", c.name, ": ", total, " crystal ticks (nominal ", crystal_hz,
              ") = ", ppm_of(total, crystal_hz), " ppm", crlf);
        verdict("period measured", got);
        // A wrong division (a neighbouring prescaler) is a factor of two:
        // the band tells it; the band itself plus a tick of each stamp.
        verdict("within the OSC32K's +-10 % spec",
                near(static_cast<int32_t>(total), static_cast<int32_t>(crystal_hz),
                     static_cast<int32_t>(osc32k_band(crystal_hz) + 2u)));
    }
    // How closely the three agree is the oscillator's wander between the
    // windows, which no data sheet bounds: printed.
    const int32_t d12 = static_cast<int32_t>(seen[0]) - static_cast<int32_t>(seen[1]);
    const int32_t d13 = static_cast<int32_t>(seen[0]) - static_cast<int32_t>(seen[2]);
    print(serial, "  prescaler agreement (the OSC32K's wander): DIV1-DIV2 = ", d12,
          ", DIV1-DIV32 = ", d13, " ticks", crlf);

    // eight periods in one measurement: the same rate, less quantization
    Rtc::init({.prescaler = RtcPrescaler::div1, .period = 32767});
    uint32_t total8 = 0;
    verdict("eight periods measured", measure(8, total8, 1500));
    print(serial, "  8 s = ", total8, " crystal ticks -> OSC32K is ",
          -ppm_of(total8, 8u * crystal_hz), " ppm FAST; their mean is ",
          static_cast<int32_t>(total8 / 8) - static_cast<int32_t>(seen[0]),
          " ticks from the one-period figure (the wander)", crlf);
    verdict("eight periods are eight seconds within the OSC32K's +-10 % spec",
            near(static_cast<int32_t>(total8 / 8), static_cast<int32_t>(crystal_hz),
                 static_cast<int32_t>(osc32k_band(crystal_hz) + 1u)));
    quiesce();
}

// ---- 3 the compare phase ----------------------------------------------------------
void t3_compare() {
    print(serial, "3 compare phase: CMP + 1 RTC ticks after the overflow", crlf);
    quiesce();

    const uint16_t cmps[] = {8191, 99};
    for (uint16_t cmp : cmps) {
        Rtc::init({.prescaler = RtcPrescaler::div1, .period = 32767, .compare = cmp});
        ChSnap::source(Rtc::OvfEvent{});
        uint32_t a = 0, b = 0, c = 0;
        const bool ok_a = stamp(a, 1500);
        const bool ok_b = stamp(b, 1500);
        const uint32_t period = b - a;
        ChSnap::source(Rtc::CmpEvent{});
        const bool ok_c = stamp(c, 1500);
        const uint32_t phase = (c - b) % period;
        // crystal ticks -> RTC ticks with the period just measured (the
        // OSC32K's real rate, not its nominal one), which still moves
        // between the period's window and the phase's.
        const uint32_t in_rtc_ticks = static_cast<uint32_t>(
            (static_cast<uint64_t>(phase) * 32768u + period / 2) / period);
        // The phase is printed, not judged: one tick in CMP + 1 is the
        // oscillator's wander between the period's window and the phase's
        // over 8192 ticks (122 ppm), and no data sheet bounds the wander.
        print(serial, "  CMP=", cmp, ": period=", period, " phase=", phase,
              " = ", in_rtc_ticks, " RTC ticks (expect ", cmp + 1u, ")", crlf);
        verdict("stamps arrive", ok_a && ok_b && ok_c);
        // What the band does tell: the compare fires inside the period, after
        // the overflow, at CMP + 1 ticks within the 22 % two windows of one
        // oscillator can differ by (osc32k_ratio_band) and a tick of rounding.
        verdict("the compare fires near CMP + 1 ticks after the overflow (+-22 %)",
                near(static_cast<int32_t>(in_rtc_ticks), static_cast<int32_t>(cmp) + 1,
                     static_cast<int32_t>(osc32k_ratio_band(cmp + 1u) + 1u)));
    }
    quiesce();
}

// ---- 4 crystal error correction ----------------------------------------------------
// Two things make a single measurement useless here. The OSC32K wanders
// by a couple of hundred ppm over tens of seconds - more than the
// 127 ppm being measured - so two blocks of periods compared far apart
// measure the drift, not the trim. And the correction itself is
// GRANULAR: it adds or removes whole CLK_RTC cycles, one every
// 1e6 / ERROR cycles (127 ppm = one cycle every 7874, i.e. one every
// 240 ms), so a quarter-second period is one or two corrections long,
// never 1.04 of one.
// What works is FAST ALTERNATION plus averaging: one short period with
// the trim in force against one without, the two taken in swapped order
// every other pair (a drift linear in time cancels between pairs), the
// mean over many pairs averaging both the wander and the granularity
// down. The period straddling each switch is discarded.
// The noise left in the mean is the oscillator's own, which no data sheet
// states: the bound is computed from the pairs themselves - four standard
// errors of their mean (a false failure about once in 16000 for a normal
// spread) - plus the granularity's bias (below).
struct TrimResult {
    int32_t mean;        ///< mean (trimmed - untrimmed) period, crystal ticks
    int32_t lo;          ///< the extremes of the individual pairs: the noise
    int32_t hi;
    uint32_t se;         ///< the standard error of the mean, crystal ticks
    bool ok;
};
/// The integer square root of v, rounded down.
uint32_t isqrt(uint64_t v) {
    uint64_t r = 0;
    for (uint64_t bit = 1ULL << 62; bit != 0; bit >>= 2) {
        if (v >= r + bit) {
            v -= r + bit;
            r = (r >> 1) + bit;
        } else {
            r >>= 1;
        }
    }
    return static_cast<uint32_t>(r);
}
TrimResult trim_effect(int8_t ppm, uint8_t pairs) {
    TrimResult r{0, 0x7FFFFFFF, -0x7FFFFFFF, 0, true};
    int32_t sum = 0;
    uint64_t sum_sq = 0;
    uint32_t prev = 0;
    if (!stamp(prev, 1500)) { r.ok = false; return r; }
    for (uint8_t i = 0; i < pairs; ++i) {
        const bool trim_first = (i & 1) == 0;
        uint32_t half[2] = {0, 0};
        for (uint8_t h = 0; h < 2; ++h) {
            const bool trimmed = (h == 0) == trim_first;
            (void)Rtc::calibrate(trimmed ? ppm : static_cast<int8_t>(0));
            uint32_t t = 0;
            if (!stamp(t, 1500)) { r.ok = false; return r; }   // straddles the switch
            prev = t;
            if (!stamp(t, 1500)) { r.ok = false; return r; }
            half[h] = t - prev;
            prev = t;
        }
        const int32_t diff = trim_first
            ? static_cast<int32_t>(half[0]) - static_cast<int32_t>(half[1])
            : static_cast<int32_t>(half[1]) - static_cast<int32_t>(half[0]);
        sum += diff;
        sum_sq += static_cast<uint64_t>(static_cast<int64_t>(diff) * diff);
        if (diff < r.lo) r.lo = diff;
        if (diff > r.hi) r.hi = diff;
    }
    r.mean = sum / pairs;
    // The sample variance (n - 1), then the standard error sqrt(var / n).
    const int64_t centred = static_cast<int64_t>(sum_sq) -
                            static_cast<int64_t>(sum) * sum / static_cast<int64_t>(pairs);
    const uint64_t var = centred > 0 ? static_cast<uint64_t>(centred) / (pairs - 1u) : 0u;
    r.se = isqrt(var / pairs);
    return r;
}

void t4_correction() {
    print(serial, "4 crystal error correction, trimmed and untrimmed periods alternating", crlf);
    quiesce();
    ChSnap::source(Rtc::OvfEvent{});
    // DIV2 throughout: a negative correction requires it, and the same
    // prescaler for both signs keeps the comparison honest.
    // A QUARTER-SECOND period (8192 CLK_RTC cycles at DIV2): the shorter
    // the period, the closer in time the trimmed and untrimmed samples
    // of a pair are, and the less of the oscillator's wander sits
    // between them. The stamps stay exact - they are hardware latches.
    Rtc::init({.prescaler = RtcPrescaler::div2, .period = 4095});
    const uint32_t nominal_period = crystal_hz / 4;

    constexpr uint8_t pairs = 40;
    constexpr int8_t trim = 127;
    // The granularity's bias: a trimmed period holds 1 or 2 of the
    // corrections spread one every 7874 cycles (8192 cycles a period), so
    // the mean of forty lies between one correction (122 ppm) and the
    // long-run 127 ppm, give or take a fortieth of one (3 ppm): 8 ppm.
    constexpr int32_t granularity_ppm = 8;
    int32_t measured[2] = {0, 0};
    uint32_t se_ppm[2] = {0, 0};
    uint8_t k = 0;
    for (int8_t sign = 1; sign >= -1; sign -= 2) {
        const int8_t ppm = static_cast<int8_t>(sign * trim);
        const TrimResult t = trim_effect(ppm, pairs);
        verdict("series measured", t.ok);
        if (!t.ok) { ++k; continue; }
        const int32_t expect = static_cast<int32_t>(
            (static_cast<int64_t>(nominal_period) * ppm) / 1'000'000);
        const int32_t measured_ppm = static_cast<int32_t>(
            (static_cast<int64_t>(t.mean) * 1'000'000) / nominal_period);
        // One standard error in ppm, rounded up (+1 for the rounding).
        const uint32_t se = static_cast<uint32_t>(
            (static_cast<uint64_t>(t.se) * 1'000'000u) / nominal_period) + 1u;
        se_ppm[k] = se;
        measured[k++] = measured_ppm;
        print(serial, "  trim ", static_cast<int32_t>(ppm), " ppm: mean effect over ", pairs,
              " pairs = ", t.mean, " ticks = ", measured_ppm, " ppm (asked ", expect,
              " ticks), standard error ", se, " ppm; pairs spread ", t.lo, " .. ", t.hi,
              " ticks (one correction cycle = ", ticks_per_rtc_x100 / 100, " ticks)", crlf);
        const int32_t tol = static_cast<int32_t>(4u * se) + granularity_ppm;
        verdict("its size is the ppm asked for (within 4 standard errors + 8 ppm)",
                near(measured_ppm, ppm, tol));
        if (tol < trim) {
            verdict("the trim moves the period its own way",
                    sign > 0 ? measured_ppm > 0 : measured_ppm < 0);
        } else {
            skip("the trim's sign: the pairs are too noisy to tell it (4 standard errors "
                 "+ 8 ppm reach the trim)");
        }
    }
    print(serial, "  the two signs span ", measured[0] - measured[1], " ppm (expect 254)", crlf);
    // The span's standard error is the root of the two squared, at most
    // their sum; the bias is twice one sign's.
    verdict("the two signs span twice the trim (within 4 standard errors + 16 ppm)",
            near(measured[0] - measured[1], 2 * trim,
                 static_cast<int32_t>(4u * (se_ppm[0] + se_ppm[1])) + 2 * granularity_ppm));
    verdict("readback of the trim in force", Rtc::calibrate(-127) && Rtc::calibration_ppm() == -127);
    verdict("cleared", Rtc::calibrate(0) && !Rtc::correcting() && Rtc::calibration_ppm() == 0);
    quiesce();
}

// ---- 5 the busy flags -------------------------------------------------------------
void t5_busy() {
    print(serial, "5 synchronization busy flags, timed in crystal ticks", crlf);
    quiesce();
    Rtc::init({.prescaler = RtcPrescaler::div1, .period = 32767, .compare = 1000});
    ChSnap::off();

    const uint32_t one_rtc = ticks_per_rtc_x100 / 100;

    // Each flag waited for on the cascade, at most 1 ms - sixteen times
    // the two CLK_RTC periods (61 us) it should take; a flag still set
    // then reads as the budget, which every verdict below refuses. The
    // reading is late by one turn of the poll: a flag load and a stamp.
    constexpr uint32_t busy_budget_us = 1000u;
    const auto busy_ticks = [](const std::optional<uint32_t>& c) {
        return c ? *c : us_cycles<Ruler>(busy_budget_us);
    };
    Rtc::count(0);
    const bool cnt_set = Rtc::count_busy();
    const uint32_t cnt_ticks =
        busy_ticks(cycles_until<Ruler>([] { return !Rtc::count_busy(); }, busy_budget_us));

    Rtc::period(32767);
    const bool per_set = Rtc::period_busy();
    const uint32_t per_ticks =
        busy_ticks(cycles_until<Ruler>([] { return !Rtc::period_busy(); }, busy_budget_us));

    Rtc::compare(1000);
    const bool cmp_set = Rtc::compare_busy();
    const uint32_t cmp_ticks =
        busy_ticks(cycles_until<Ruler>([] { return !Rtc::compare_busy(); }, busy_budget_us));

    Rtc::prescaler(RtcPrescaler::div1);
    const bool ctrla_set = Rtc::ctrla_busy();
    const uint32_t ctrla_ticks =
        busy_ticks(cycles_until<Ruler>([] { return !Rtc::ctrla_busy(); }, busy_budget_us));

    Pit::period(PitPeriod::cyc32);
    const bool pit_set = Pit::ctrl_busy();
    const uint32_t pit_ticks =
        busy_ticks(cycles_until<Ruler>([] { return !Pit::ctrl_busy(); }, busy_budget_us));

    print(serial, "  one CLK_RTC period = ", one_rtc, " crystal ticks", crlf);
    print(serial, "  CNTBUSY=", cnt_ticks, " PERBUSY=", per_ticks, " CMPBUSY=", cmp_ticks,
          " CTRLABUSY=", ctrla_ticks, " PIT CTRLBUSY=", pit_ticks, crlf);
    verdict("a write raises its busy flag",
            cnt_set && per_set && cmp_set && ctrla_set && pit_set);
    // The bounds, in nominal CLK_RTC periods (one_rtc): a write takes effect
    // two CLK_RTC cycles after it (the CNT, PER, CMP and CTRLA register
    // descriptions) plus up to one of phase, and the flag's way back into
    // CLK_PER is a few CLK_PER cycles, as is the poll's turn: 1..3 real
    // cycles. At the band's slow end a real cycle is 1/0.9 nominal (3.34 for
    // three), at its fast end 1/1.1 (0.91 for one): within 4, at least half.
    verdict("CNTBUSY clears within 4 CLK_RTC", cnt_ticks < 4 * one_rtc);
    verdict("PERBUSY clears within 4 CLK_RTC", per_ticks < 4 * one_rtc);
    verdict("CMPBUSY clears within 4 CLK_RTC", cmp_ticks < 4 * one_rtc);
    verdict("CTRLABUSY clears within 4 CLK_RTC", ctrla_ticks < 4 * one_rtc);
    verdict("PIT CTRLBUSY clears within 4 CLK_RTC", pit_ticks < 4 * one_rtc);
    verdict("all of them take at least half a CLK_RTC",
            cnt_ticks > one_rtc / 2 && per_ticks > one_rtc / 2 && cmp_ticks > one_rtc / 2 &&
            ctrla_ticks > one_rtc / 2 && pit_ticks > one_rtc / 2);

    // the sync() sweep leaves nothing in flight
    Rtc::count(0);
    Rtc::period(32767);
    verdict("sync() waits them all out", Rtc::sync() &&
            !Rtc::count_busy() && !Rtc::period_busy() && !Rtc::ctrla_busy());
    quiesce();
}

// ---- 6 the first tick -------------------------------------------------------------
// The prescaler's counting phase at the moment of the enable is not
// observable, so the first PIT interrupt lands anywhere inside one full
// period (26.5.2.2). Both cases are timed: with the prescaler stopped
// (RTC counter off too) and with it free-running (counter on).
/// The first PI flag waited for at most 10 ms on the cascade, ten times
/// the 977 us period (CYC32 at 32768 Hz); a flag that never came reads as
/// that budget, which the period verdicts refuse.
void first_tick_stats(bool counter_running, uint32_t& lo, uint32_t& hi, uint32_t& mean) {
    constexpr uint8_t n = 32;
    uint32_t sum = 0;
    lo = 0xFFFFFFFFu;
    hi = 0;
    for (uint8_t i = 0; i < n; ++i) {
        Pit::disable();
        if (counter_running) Rtc::enable(); else Rtc::disable();
        Pit::clear_flag();
        const uint32_t t0 = now();
        Pit::init(PitPeriod::cyc32, false);
        const bool came = wait_until<Ruler>([] { return Pit::flag(); }, 10'000u);
        const uint32_t d = came ? now() - t0 : us_cycles<Ruler>(10'000u);
        sum += d;
        if (d < lo) lo = d;
        if (d > hi) hi = d;
    }
    mean = sum / n;
}

void t6_first_tick() {
    print(serial, "6 the first PIT interrupt after an enable (32 cycles each)", crlf);
    quiesce();
    Rtc::init({.prescaler = RtcPrescaler::div1, .period = 32767});
    Rtc::disable();

    const uint32_t period_ticks = 32u * (ticks_per_rtc_x100 / 100);   // CYC32
    // A period at the band's slow end is 1/0.9 of nominal (+11.1 %); the
    // poll's turn (a flag load and a stamp, tens of cycles) is far inside
    // the further 1.4 % of 1/8.
    const uint32_t period_max = period_ticks + period_ticks / 8u;
    uint32_t lo = 0, hi = 0, mean = 0;
    first_tick_stats(false, lo, hi, mean);
    print(serial, "  prescaler stopped (both functions off): min=", lo, " max=", hi,
          " mean=", mean, " of a ", period_ticks, "-tick period", crlf);
    verdict("every first tick inside one period (stopped)", hi <= period_max);
    const uint32_t spread_stopped = hi - lo;

    first_tick_stats(true, lo, hi, mean);
    print(serial, "  prescaler free-running (counter enabled): min=", lo, " max=", hi,
          " mean=", mean, crlf);
    verdict("every first tick inside one period (running)", hi <= period_max);
    verdict("the phase is genuinely unknown (spread > 10 % of a period)",
            (hi - lo) > period_ticks / 10 || spread_stopped > period_ticks / 10);
    quiesce();
}

// ---- 7 PIT and counter together ----------------------------------------------------
void t7_together() {
    print(serial, "7 PIT and counter together, and PIT_DIV64 against PRESCALER", crlf);
    quiesce();

    // the counter's period with the PIT stopped, then with it running
    Pit::disable();
    Rtc::init({.prescaler = RtcPrescaler::div1, .period = 32767});
    ChSnap::source(Rtc::OvfEvent{});
    uint32_t alone = 0, shared = 0;
    verdict("period measured with the PIT stopped", measure(2, alone, 1500));
    Ticker::init();                      // PIT 1024 Hz, interrupt enabled
    verdict("period measured with the PIT running", measure(2, shared, 1500));
    print(serial, "  2 s alone=", alone, " with the PIT (and its ISR)=", shared,
          " difference=", static_cast<int32_t>(shared) - static_cast<int32_t>(alone),
          " ticks = ", ppm_of(shared, alone), " ppm (the OSC32K's own wander between the "
          "two windows, which no data sheet bounds: printed; the exact proof is the tick "
          "ratio below)", crlf);

    // the two halves of the same prescaler chain: 1024 PIT ticks per
    // 32768 CLK_RTC counter period
    uint32_t dummy = 0;
    stamp(dummy, 1500);                  // align on an overflow
    const uint32_t k0 = Ticker::ticks();
    stamp(dummy, 1500);
    const uint32_t k1 = Ticker::ticks();
    print(serial, "  Ticker ticks in one counter period: ", k1 - k0, " (expect 1024)", crlf);
    verdict("1024 PIT ticks per counter period", near(static_cast<int32_t>(k1 - k0), 1024, 1));
    Ticker::pause();

    // PIT_DIV64 as a level event, measured by TCB0: 64 CLK_RTC cycles
    ChPit::source(EvPitDiv<64>{});
    meter_on = true;
    FrequencyMeter<T0>::init(clock, ChPit{});
    captures = 0;
    hold_ms(50);
    const uint16_t div1_ticks = cap_a;
    Rtc::prescaler(RtcPrescaler::div32);
    hold_ms(50);
    const uint16_t div32_ticks = cap_a;
    Rtc::prescaler(RtcPrescaler::div1);
    print(serial, "  PIT_DIV64 period: ", div1_ticks, " ticks with PRESCALER=DIV1, ",
          div32_ticks, " with DIV32 (nominal ", 64u * (ticks_per_rtc_x100 / 100), ")", crlf);
    // Both judged against the band (+-10 % and a tick of the meter): the
    // neighbouring dividers are a factor of two away, and a PIT that saw
    // the PRESCALER would run 32 times slower.
    const uint32_t div64_nominal = 64u * (ticks_per_rtc_x100 / 100);
    const int32_t div64_tol = static_cast<int32_t>(osc32k_band(div64_nominal) + 1u);
    verdict("PIT_DIV64 = 64 CLK_RTC cycles (+-10 %)",
            near(div1_ticks, static_cast<int32_t>(div64_nominal), div64_tol));
    verdict("the PIT does not see the counter's PRESCALER (still 64 cycles at DIV32)",
            near(div32_ticks, static_cast<int32_t>(div64_nominal), div64_tol));
    meter_on = false;
    T0::disable();
    quiesce();
}

// ---- 8 OSC1K --------------------------------------------------------------------
void t8_osc1k() {
    print(serial, "8 OSC1K as CLK_RTC (the same OSC32K divided by 32)", crlf);
    quiesce();

    // reference: one second from the 32.768 kHz source
    Rtc::init({.prescaler = RtcPrescaler::div1, .period = 32767});
    ChSnap::source(Rtc::OvfEvent{});
    uint32_t at32k = 0;
    verdict("32768 Hz second measured", measure(1, at32k, 1500));

    // the same second from the 1.024 kHz source
    Rtc::disable();
    Pit::disable();
    RtcClock::select(RtcSource::osc1k);
    verdict("CLKSEL reads back OSC1K", RtcClock::selected() == RtcSource::osc1k);
    verdict("source rate 1024 Hz", RtcClock::hz() == 1024u);
    Rtc::init({.prescaler = RtcPrescaler::div1, .period = 1023});
    verdict("tick_hz = 1024", Rtc::tick_hz() == 1024u);
    uint32_t at1k = 0;
    verdict("1024 Hz second measured", measure(1, at1k, 1500));
    print(serial, "  one second: ", at32k, " ticks from OSC32K, ", at1k, " from OSC1K, difference ",
          static_cast<int32_t>(at1k) - static_cast<int32_t>(at32k),
          " (the OSC32K's wander between the windows: printed)", crlf);
    // A divider other than 32 is a factor of two or more: the band tells
    // it (two windows of one oscillator, 22 %, and a tick of each stamp).
    // The exact proof of the division is the Ticker's count below.
    verdict("OSC1K's second is OSC32K's (+-22 %): the divider is 32",
            near(static_cast<int32_t>(at1k), static_cast<int32_t>(at32k),
                 static_cast<int32_t>(osc32k_ratio_band(at32k) + 2u)));

    // the Ticker's rate follows the source: 1024/32 = 32 ticks/s
    Ticker::init(RtcSource::osc1k);
    uint32_t dummy = 0;
    stamp(dummy, 1500);
    const uint32_t k0 = Ticker::ticks();
    stamp(dummy, 1500);
    const uint32_t k1 = Ticker::ticks();
    print(serial, "  Ticker ticks in one second on OSC1K: ", k1 - k0, " (expect 32)", crlf);
    verdict("the Ticker's rate divides by 32", near(static_cast<int32_t>(k1 - k0), 32, 1));
    Ticker::pause();
    quiesce();
}


// ---- 9 XOSC32K as CLK_RTC (the board with the crystal) --------------------------
// The rates are printed, not judged: a 32.768 kHz tuning fork against the
// board's 24 MHz oscillator, neither part number on record, so no
// tolerance bounds their difference.

void t9_xosc32k() {
    print(serial, "9 XOSC32K as CLK_RTC: the 32.768 kHz crystal against the 24 MHz external clock",
          crlf);
    if constexpr (!board::xosc32k_runs) {
        // The crystal is fitted but the board says it does not oscillate:
        // every verdict below is about the crystal running.
        skip("the 32.768 kHz crystal as CLK_RTC: the board's crystal is fitted but does not "
             "run (board.hpp's board::xosc32k_runs)");
        return;
    }
    quiesce();
    Xosc32k::start_crystal(Xosc32kStartup::cycles1k, false, true);   // forced on: nothing requests it yet
    const bool up = wait_until<Ruler>([] { return Xosc32k::stable(); }, 1'500'000u);
    verdict("XOSC32K runs (started forced on, stable within 1.5 s)", up);
    verdict("RtcClock::preferred() names the crystal once it runs",
            RtcClock::preferred() == RtcSource::xosc32k);
    if (!up) {
        Xosc32k::stop();
        quiesce();
        return;
    }

    Rtc::disable();
    Pit::disable();
    RtcClock::select(RtcSource::xosc32k);
    verdict("CLKSEL reads back XOSC32K", RtcClock::selected() == RtcSource::xosc32k);
    verdict("source rate 32768 Hz", RtcClock::hz() == 32768u);
    Rtc::init({.prescaler = RtcPrescaler::div1, .period = 32767});
    ChSnap::source(Rtc::OvfEvent{});
    uint32_t forced = 0;
    const bool got_forced = measure(4, forced, 1500);
    // The force dropped: the RTC is a requester (CLKCTRL 11.3.6), so the
    // crystal must keep running for it alone.
    Xosc32k::force_on(false);
    uint32_t requested = 0;
    const bool got_requested = measure(4, requested, 1500);
    const int32_t p_forced = ppm_of(forced, 4u * crystal_hz);
    const int32_t p_requested = ppm_of(requested, 4u * crystal_hz);
    print(serial, "  4 s of XOSC32K = ", forced, " ticks forced on (the crystal ", -p_forced,
          " ppm fast against the external clock); ", requested, " (", -p_requested,
          " ppm fast) held by the RTC's request alone", crlf);
    verdict("four periods measured, forced on and requested", got_forced && got_requested);

    // The Ticker's default init takes the crystal (preferred()) and counts
    // 1024 ticks to its second: the PIT and the counter share CLK_RTC.
    Ticker::init();
    verdict("Ticker::init() selected XOSC32K by itself", RtcClock::selected() == RtcSource::xosc32k);
    Rtc::init({.prescaler = RtcPrescaler::div1, .period = 32767});
    ChSnap::source(Rtc::OvfEvent{});
    uint32_t dummy = 0;
    const bool s0 = stamp(dummy, 1500);
    const uint32_t k0 = Ticker::ticks();
    const bool s1 = stamp(dummy, 1500);
    const uint32_t k1 = Ticker::ticks();
    print(serial, "  Ticker ticks in one crystal second: ", k1 - k0, " (expect 1024)", crlf);
    verdict("the Ticker counts 1024 ticks to the crystal's second",
            s0 && s1 && near(static_cast<int32_t>(k1 - k0), 1024, 1));
    Ticker::pause();
    // Stopped, the crystal's XOSC32KS lingers (a Ticker re-initialised in
    // that window froze test_avr_clock letter 8 before preferred() asked
    // enabled() too): preferred() is asked at once, while the status may
    // still read 1, and must already name the internal oscillator.
    Rtc::disable();
    Pit::disable();
    RtcClock::select(RtcSource::osc32k);
    Xosc32k::stop();
    const bool status_after_stop = Xosc32k::stable();
    const RtcSource preferred_after_stop = RtcClock::preferred();
    const std::optional<uint32_t> linger =
        cycles_until<Ruler>([] { return !Xosc32k::stable(); }, 2'000'000u);
    const uint32_t lingered = linger ? *linger / (crystal_hz / 1000u) : 2000u;
    print(serial, "  XOSC32KS read ", status_after_stop ? 1 : 0, " when preferred() was asked, 1 for ",
          lingered, " ms after the stop", crlf);
    verdict("right after the stop preferred() names OSC32K",
            preferred_after_stop == RtcSource::osc32k);
    verdict("the stopped crystal's status drops within 2 s", !Xosc32k::stable());
    quiesce();
}

#if defined(RTC_EVGEN0SEL_gm)
// ---- p the PIT's event slots (the AVR EA) ------------------------------------
// RTC.PITEVGENCTRLA (DS40002443A 24.13.17) gives the event system two
// SLOTS, each carrying CLK_RTC divided by 4..32768 - where the chapter's
// overview (24.2) still names "64..8192". Each divider is selected from
// the RTC's side (Pit::event_divider<slot>) under a route made once
// (EvPitDiv<4, slot>: the channel names the slot, not the divider) and
// its period measured in crystal ticks: by TCB0 as a frequency meter up
// to DIV64 (39062 ticks at 20 MHz, inside the 16-bit capture), by the
// cascade's hardware stamps from DIV128 (3.9 ms, past the stamp loop's
// 1 ms poll) - against the SAME oscillator's second, measured first
// through the counter's OVF.
//
// The bound of a period against d / 32768 of that second: two windows of
// one oscillator, each within the data sheet's 10 %, differ by at most 22 %
// (osc32k_ratio_band; their wander no data sheet bounds) plus the
// instrument's quantization, +-1 tick for the meter and +-1 for each of the
// two stamps: expected x 2/9 + 2 ticks. A neighbouring divider is a factor
// of two away; the printed figures show how close each lands.
/// The mean of sixteen consecutive periods (the first capture after the
/// change discarded: it may straddle it), and their spread: a single
/// DIV4 period - four cycles of an RC oscillator - read 0.24 % short
/// once in ten runs, the oscillator's cycle-to-cycle jitter; the mean of
/// consecutive periods telescopes it to its two ends.
uint16_t slot_spread = 0;
uint16_t slot_meter_period(uint8_t slot, PitEventDiv d) {
    if (slot == 0) Pit::event_divider<0>(d); else Pit::event_divider<1>(d);
    delay_us(clock, 100);
    cli(); captures = 0; sei();
    for (uint16_t ms = 0; ms < 50 && captures < 1; ++ms) delay_us(clock, 1000);
    cli(); captures = 0; cap_sum = 0; cap_min = 0xFFFF; cap_max = 0; sei();
    // Sixteen periods of DIV64 are 32 ms: up to 200 ms.
    for (uint16_t ms = 0; ms < 2000 && captures < 16; ++ms) delay_us(clock, 100);
    cli();
    const uint16_t n = captures;
    const uint32_t sum = cap_sum;
    slot_spread = static_cast<uint16_t>(cap_max - cap_min);
    sei();
    return n >= 16 ? static_cast<uint16_t>((sum + n / 2) / n) : 0;
}
uint32_t slot_stamp_period(uint8_t slot, PitEventDiv d) {
    if (slot == 0) Pit::event_divider<0>(d); else Pit::event_divider<1>(d);
    uint32_t a = 0, b = 0;
    // A stamp after the change, then the period between the next two.
    if (!stamp(a, 1500) || !stamp(a, 1500) || !stamp(b, 1500)) return 0;
    return b - a;
}
bool slot_period_ok(uint32_t measured, uint32_t second, uint16_t cycles) {
    const uint32_t expect = static_cast<uint32_t>(
        (static_cast<uint64_t>(second) * cycles + 16384u) / 32768u);
    return measured != 0 && near(static_cast<int32_t>(measured), static_cast<int32_t>(expect),
                                 static_cast<int32_t>(osc32k_ratio_band(expect) + 2u));
}

void tp_pit_slots() {
    print(serial, "p the PIT event slots: every divider through each slot, both at once, "
                  "the chain's enables", crlf);
    quiesce();
    // The reference: one OSC32K second through the counter's OVF.
    Rtc::init({.prescaler = RtcPrescaler::div1, .period = 32767});
    ChSnap::source(Rtc::OvfEvent{});
    uint32_t second = 0;
    verdict("the OSC32K second measured (the reference)", measure(1, second, 1500));
    print(serial, "  one OSC32K second = ", second, " crystal ticks", crlf);

    // 1. Every divider, each slot in turn.
    meter_on = true;
    FrequencyMeter<T0>::init(clock, ChPit{});
    for (uint8_t slot = 0; slot < 2; ++slot) {
        if (slot == 0) {
            ChPit::source(EvPitDiv<4, 0>{});
            ChSnap::source(EvPitDiv<4, 0>{});
        } else {
            ChPit::source(EvPitDiv<4, 1>{});
            ChSnap::source(EvPitDiv<4, 1>{});
        }
        bool all = true, readback = true;
        for (uint8_t code = 1; code <= 14; ++code) {
            const PitEventDiv d = static_cast<PitEventDiv>(code);
            const uint16_t cycles = pit_event_cycles(d);
            const uint32_t t = code <= 5 ? slot_meter_period(slot, d) : slot_stamp_period(slot, d);
            const PitEventDiv back = slot == 0 ? Pit::event_divider<0>() : Pit::event_divider<1>();
            const bool ok = slot_period_ok(t, second, cycles);
            print(serial, "  slot ", slot, " DIV", cycles, ": ", t, " ticks (expect ",
                  static_cast<uint32_t>((static_cast<uint64_t>(second) * cycles + 16384u) / 32768u),
                  ")");
            if (code <= 5) print(serial, ", 16 periods spread ", slot_spread);
            print(serial, ok ? "" : " - OFF", crlf);
            all = all && ok;
            readback = readback && back == d;
        }
        verdict(slot == 0 ? "slot 0 carries every divider 4..32768 (period within 22 % + 2)"
                          : "slot 1 carries every divider 4..32768 (period within 22 % + 2)", all);
        verdict(slot == 0 ? "slot 0's divider reads back" : "slot 1's divider reads back",
                readback);
        if (slot == 0) EvPitDiv<4, 0>::release(); else EvPitDiv<4, 1>::release();
    }

    // 2. Both slots at once: slot 0 DIV64 on the meter, slot 1 DIV4096 on
    //    the stamps, measured together; slot 1's selection leaves slot 0's
    //    nibble alone.
    ChPit::source(EvPitDiv<64, 0>{});
    ChSnap::source(EvPitDiv<4096, 1>{});
    const PitEventDiv kept0 = Pit::event_divider<0>();
    cli(); captures = 0; sei();
    uint32_t a = 0, b = 0;
    const bool st = stamp(a, 1500) && stamp(b, 1500);
    const uint16_t m64 = cap_a;
    const uint16_t n_caps = captures;
    print(serial, "  together: slot 0 DIV64 ", m64, " ticks (", n_caps, " captures meanwhile), "
          "slot 1 DIV4096 ", b - a, " ticks", crlf);
    verdict("selecting slot 1 leaves slot 0's divider", kept0 == PitEventDiv::div64);
    verdict("both slots run at once, each at its own divider",
            st && n_caps > 2 && slot_period_ok(m64, second, 64) && slot_period_ok(b - a, second, 4096));
    EvPitDiv<4096, 1>::release();
    ChSnap::off();

    // 3. The prescaler runs while RTCEN or PITEN is set (24.5.2.2): slot 0
    //    at DIV4 (8 kHz) on the meter, both functions off - nothing in
    //    20 ms; the PIT alone - the slot runs; the counter alone - the
    //    counter counts (its CNT read across the window), and what the
    //    slot does then is printed: the chapter does not say whether the
    //    slots follow PITEN or the prescaler's run condition.
    Ticker::pause();
    Pit::disable();
    Rtc::disable();
    rtc_wait(RTC_CTRLABUSY_bm);
    rtc_pit_wait();
    Pit::event_divider<0>(PitEventDiv::div4);
    auto caps_20ms = []() -> uint16_t {
        delay_us(clock, 200);
        cli(); captures = 0; sei();
        for (uint8_t ms = 0; ms < 20; ++ms) delay_us(clock, 1000);
        cli(); const uint16_t n = captures; sei();
        return n;
    };
    const uint16_t off_caps = caps_20ms();
    Pit::init(PitPeriod::cyc1024, false);
    const uint16_t pit_caps = caps_20ms();
    Pit::disable();
    Rtc::init({.prescaler = RtcPrescaler::div1, .period = 32767});
    rtc_wait(RTC_CTRLABUSY_bm);
    const uint16_t cnt0 = Rtc::count();
    const uint16_t rtc_caps = caps_20ms();
    const uint16_t cnt_moved = static_cast<uint16_t>(Rtc::count() - cnt0);
    // 4. The slot turned off: a constant low, nothing to capture.
    Pit::event_divider<0>(PitEventDiv::off);
    const uint16_t slot_off_caps = caps_20ms();
    print(serial, "  DIV4 captures in 20 ms: both off ", off_caps, ", PIT on ", pit_caps,
          ", counter on ", rtc_caps, " (its CNT moved ", cnt_moved, "); the slot off ",
          slot_off_caps, crlf);
    // 20 ms of 8192 Hz is 164 periods: "running" is anything past 100;
    // the counter at CLK_RTC counts some 655 in the window (32768 Hz,
    // OSC32K within 10 %): "counting" is anything past 500.
    verdict("neither RTCEN nor PITEN: the slot's divider is still", off_caps == 0);
    verdict("PITEN alone runs the slot", pit_caps > 100);
    verdict("RTCEN alone: the counter counts", cnt_moved > 500);
    print(serial, "  RTCEN alone: the slot ", rtc_caps == 0 ? "stays still (it follows PITEN)"
                                                            : "runs (it follows the prescaler)",
          crlf);
    verdict("a slot set OFF carries nothing", slot_off_caps == 0);
    meter_on = false;
    T0::disable();
    ChPit::off();
    quiesce();
}
#endif

using TestFn = void (*)();
struct Test { char key; TestFn fn; };
constexpr Test tests[] = {
    {'1', t1_verbs}, {'2', t2_period}, {'3', t3_compare}, {'4', t4_correction},
    {'5', t5_busy}, {'6', t6_first_tick}, {'7', t7_together}, {'8', t8_osc1k},
};

void run(TestFn fn) {
    passed = failed = 0;
    skipped = 0;
    fn();
    tally("  -> ", passed, failed, skipped);
    print(serial, crlf);
}

void help() {
#if defined(RTC_EVGEN0SEL_gm)
    print(serial, "test_avr_rtc: 1 resource verbs | 2 counter period | 3 compare phase | "
                  "4 crystal error correction | 5 busy flags | 6 first tick | "
                  "7 PIT and counter together | 8 OSC1K | 9 XOSC32K | p PIT event slots | a all",
          crlf);
    return;
#endif
    if constexpr (board::xosc32k) {
        print(serial, "test_avr_rtc: 1 resource verbs | 2 counter period | 3 compare phase | "
                      "4 crystal error correction | 5 busy flags | 6 first tick | "
                      "7 PIT and counter together | 8 OSC1K | 9 XOSC32K | a all", crlf);
    } else {
        print(serial, "test_avr_rtc: 1 resource verbs | 2 counter period | 3 compare phase | "
                      "4 crystal error correction | 5 busy flags | 6 first tick | "
                      "7 PIT and counter together | 8 OSC1K | a all", crlf);
    }
}

} // namespace

ISR(USART2_RXC_vect) { (void)Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }
ISR(RTC_PIT_vect)    { Ticker::pit(); }
ISR(RTC_CNT_vect) {
    const auto f = brio::Rtc::take_flags();
    if (f.ovf) rtc_ovf_irqs = rtc_ovf_irqs + 1;
    if (f.cmp) rtc_cmp_irqs = rtc_cmp_irqs + 1;
}
ISR(TCB0_INT_vect) {
#if defined(RTC_EVGEN0SEL_gm)
    if (meter_on) {
        const uint16_t t = brio::FrequencyMeter<T0>::period_ticks();
        cap_a = t;
        captures = captures + 1;
        cap_sum = cap_sum + t;
        if (t < cap_min) cap_min = t;
        if (t > cap_max) cap_max = t;
    }
#else
    if (meter_on) { cap_a = brio::FrequencyMeter<T0>::period_ticks(); captures = captures + 1; }
#endif
    else (void)T0::take_flags();
}

int main() {
    const bool xtal = SysClock::init();
    Serial::init(clock, 460800);
    Ticker::init();
    Ticker::pause();                      // the RTC is the device under test
    stopwatch_init();
    sei();
    auto board = board_id();
    if (board.empty()) board = "?";
    if constexpr (at24) {
        print(serial, crlf, "test_avr_rtc - RTC/PIT test suite (board ", board,
              ", clk=", xtal ? (has_xoschf ? "XTAL" : "EXTCLK") : "OSCHF",
              " 24 MHz, silicon rev ", hex(SYSCFG.REVID), ")", crlf);
    } else {
        print(serial, crlf, "test_avr_rtc - RTC/PIT test suite (board ", board,
              ", clk=", xtal ? (has_xoschf ? "XTAL" : "EXTCLK") : "OSCHF", " ",
              crystal_hz / 1'000'000u, " MHz, silicon rev ", hex(SYSCFG.REVID), ")", crlf);
    }
    help();
    print(serial, "> ");
    for (;;) {
        uint8_t c;
        if (!Serial::read_byte(c)) continue;
        if (c == '\r' || c == '\n') continue;
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') { help(); }
        else if (c == 'a' || c == 'A') {
            uint16_t tp = 0, tf = 0, ts = 0;
            for (const Test& t : tests) { run(t.fn); tp += passed; tf += failed; ts += skipped; }
            if constexpr (board::xosc32k) {
                run(t9_xosc32k); tp += passed; tf += failed; ts += skipped;
            }
#if defined(RTC_EVGEN0SEL_gm)
            run(tp_pit_slots); tp += passed; tf += failed; ts += skipped;
#endif
            tally("ALL: ", tp, tf, ts);
        } else {
            bool found = false;
            for (const Test& t : tests) if (t.key == c) { run(t.fn); found = true; }
            if constexpr (board::xosc32k) {
                if (c == '9') { run(t9_xosc32k); found = true; }
            }
#if defined(RTC_EVGEN0SEL_gm)
            if (c == 'p') { run(tp_pit_slots); found = true; }
#endif
            if (!found) print(serial, "? for help", crlf);
        }
        print(serial, "> ");
    }
}
