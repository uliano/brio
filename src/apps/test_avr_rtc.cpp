// test_avr_rtc - the RTC/PIT test SUITE for the AVR DA/DB target: the
// real-time counter and the periodic interrupt timer measured against
// the 24 MHz crystal, with no wires at all. A TCB cascade (TCB1+TCB2,
// 32 bits at CLK_PER) is the stopwatch; the RTC's own OVF and CMP
// events latch it through an event channel, so every interval is
// timestamped by hardware and the CPU only reads the latches.
// Reference test of avrdx/rtc.hpp and avrdx/ticker.hpp
// (docs/avrdx/rtc.md): keep it passing.
//
// Bench diagnostic, NOT a kernel app (sequential, blocking). Console on
// USART2 ALT1 (PF4/PF5) at 460800. No pin is driven, nothing to wire.
// Event channels: 3 = the PIT divider level (odd channel: PIT_DIV64),
// 4 = the cascade's carry, 5 = the cascade's snapshot, re-sourced per
// test (software pulses, RTC OVF, RTC CMP).
//
// Commands: ? | 1 resource verbs | 2 counter period | 3 compare phase
// | 4 crystal error correction | 5 busy flags | 6 first tick
// | 7 PIT and counter together | 8 OSC1K | a all
//
// Tests:
//   1  the verbs: clock select and readback, init/enable, CNT advancing
//      at the expected rate, PER/CMP/CNT write-read, prescaler, the
//      OVF/CMP interrupts on their shared vector, the calibrate()
//      refusals (negative correction at DIV1, +-128 ppm), the PIT verbs;
//   2  the counter's period in crystal ticks at DIV1/DIV2/DIV32 with PER
//      chosen so all three mean 32768 CLK_RTC cycles: the three must
//      agree (the prescaler divides exactly) and the common value gives
//      the OSC32K's real rate in ppm;
//   3  the compare phase: OVF stamps give the period, then the same
//      channel re-sourced to CMP gives the phase - CMP + 1 RTC ticks
//      after the overflow, for CMP = 8191 and CMP = 99;
//   4  crystal error correction: quarter-second counter periods measured
//      with the trim in force against periods without it, alternating in
//      swapped order, at +127 and -127 ppm (prescaler DIV2 - a negative
//      correction needs it); readback of the trim in force;
//   5  the synchronization busy flags: CNT/PER/CMP/CTRLA and the PIT's
//      CTRLBUSY, each timed in crystal ticks from the write to the flag
//      clearing (expected: two CLK_RTC periods);
//   6  the first tick after an enable: 32 enable cycles timed from the
//      PITEN write to the first PI flag, with the prescaler stopped
//      (both functions off) and free-running (the counter left on) -
//      every one inside a full period, and the spread is the point;
//   7  no interference: the counter's period with the PIT running and
//      with it stopped, the Ticker's ticks over one counter period
//      (exactly 1024), and PIT_DIV64 measured while the counter's
//      prescaler changes - the PIT does not see PRESCALER;
//   8  OSC1K as CLK_RTC: the same 1 s from a 1.024 kHz clock (PER =
//      1023), and the Ticker's rate divided by 32.
// Not testable here: XOSC32K and the external clock (no 32 kHz crystal
// and no external clock on the bench board), RUNSTDBY and the PIT as a
// power-down wake-up (no sleep in this suite), DBGRUN (needs a halted
// CPU).

// pio: monitor_speed = 460800

#include <avr/interrupt.h>
#include <stdint.h>

#include "avrdx/clock.hpp"
#include "avrdx/delay.hpp"
#include "avrdx/evsys.hpp"
#include "avrdx/rtc.hpp"
#include "avrdx/tcb.hpp"
#include "avrdx/ticker.hpp"
#include "avrdx/uart.hpp"
#include "util/print.hpp"

using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
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
constexpr uint32_t nominal_hz = 32768;           // CLK_RTC from OSC32K
// Crystal ticks in one nominal CLK_RTC cycle (24e6 / 32768 = 732.42).
constexpr uint32_t ticks_per_rtc_x100 = (crystal_hz * 100ul) / nominal_hz;

volatile uint16_t rtc_ovf_irqs = 0;
volatile uint16_t rtc_cmp_irqs = 0;
volatile uint16_t captures = 0;
volatile uint16_t cap_a = 0;
volatile bool meter_on = false;

// ---- tiny test harness --------------------------------------------------------
uint8_t passed = 0, failed = 0;

void verdict(const char* name, bool ok) {
    if (ok) ++passed; else ++failed;
    print(serial, "  ", ok ? "PASS" : "FAIL", "  ", name, crlf);
}
bool near(int32_t a, int32_t b, int32_t tol) {
    const int32_t d = a > b ? a - b : b - a;
    return d <= tol;
}
// Holds are counted in CRYSTAL time (delay_us, a cycle loop), never by
// the Ticker: the RTC is the device under test.
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
    rtc_ovf_irqs = rtc_cmp_irqs = 0;
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
    verdict("CNT advances at CLK_RTC", near(static_cast<int32_t>(c1) - c0, 3277, 200));
    Rtc::count(0);
    verdict("CNT written to 0", Rtc::count() < 200);

    // the shared vector: OVF and CMP both raise it
    Rtc::init({.prescaler = RtcPrescaler::div32, .period = 1023, .compare = 511});
    Rtc::enable_ovf_interrupt(true);
    Rtc::enable_cmp_interrupt(true);
    rtc_ovf_irqs = rtc_cmp_irqs = 0;
    hold_ms(3100);                        // 3 periods of 1 s, plus slack
    print(serial, "  in 3.1 s: ", rtc_ovf_irqs, " OVF, ", rtc_cmp_irqs, " CMP interrupts", crlf);
    verdict("OVF interrupts (3 +-1)", near(rtc_ovf_irqs, 3, 1));
    verdict("CMP interrupts (3 +-1)", near(rtc_cmp_irqs, 3, 1));
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
        verdict("within the OSC32K's +-10 % spec", near(static_cast<int32_t>(total), static_cast<int32_t>(crystal_hz), crystal_hz / 10));
    }
    const int32_t d12 = static_cast<int32_t>(seen[0]) - static_cast<int32_t>(seen[1]);
    const int32_t d13 = static_cast<int32_t>(seen[0]) - static_cast<int32_t>(seen[2]);
    print(serial, "  prescaler agreement: DIV1-DIV2 = ", d12, ", DIV1-DIV32 = ", d13, " ticks", crlf);
    verdict("the three prescalers divide identically (0.05 %)",
            near(d12, 0, 12'000) && near(d13, 0, 12'000));

    // eight periods in one measurement: the same rate, less quantization
    Rtc::init({.prescaler = RtcPrescaler::div1, .period = 32767});
    uint32_t total8 = 0;
    verdict("eight periods measured", measure(8, total8, 1500));
    print(serial, "  8 s = ", total8, " crystal ticks -> OSC32K is ",
          -ppm_of(total8, 8u * crystal_hz), " ppm FAST", crlf);
    verdict("eight periods agree with one (within the oscillator's wander)",
            near(static_cast<int32_t>(total8 / 8), static_cast<int32_t>(seen[0]), 8000));
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
        // OSC32K's real rate, not its nominal one). The conversion is
        // worth a tick or so at the far end of a period: the oscillator
        // wanders by ~100 ppm between the period and the phase, and
        // 100 ppm of 8192 ticks is one tick.
        const uint32_t in_rtc_ticks = static_cast<uint32_t>(
            (static_cast<uint64_t>(phase) * 32768u + period / 2) / period);
        print(serial, "  CMP=", cmp, ": period=", period, " phase=", phase,
              " = ", in_rtc_ticks, " RTC ticks (expect ", cmp + 1u, ")", crlf);
        verdict("stamps arrive", ok_a && ok_b && ok_c);
        verdict("phase = CMP + 1 ticks", near(static_cast<int32_t>(in_rtc_ticks), static_cast<int32_t>(cmp) + 1, 2));
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
struct TrimResult {
    int32_t mean;        ///< mean (trimmed - untrimmed) period, crystal ticks
    int32_t lo;          ///< the extremes of the individual pairs: the noise
    int32_t hi;
    bool ok;
};
TrimResult trim_effect(int8_t ppm, uint8_t pairs) {
    TrimResult r{0, 0x7FFFFFFF, -0x7FFFFFFF, true};
    int32_t sum = 0;
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
        if (diff < r.lo) r.lo = diff;
        if (diff > r.hi) r.hi = diff;
    }
    r.mean = sum / pairs;
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
    int32_t measured[2] = {0, 0};
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
        measured[k++] = measured_ppm;
        print(serial, "  trim ", static_cast<int32_t>(ppm), " ppm: mean effect over ", pairs,
              " pairs = ", t.mean, " ticks = ", measured_ppm, " ppm (asked ", expect,
              " ticks); pairs spread ", t.lo, " .. ", t.hi, " ticks (one correction cycle = ",
              ticks_per_rtc_x100 / 100, " ticks)", crlf);
        verdict("the trim moves the period its own way", sign > 0 ? measured_ppm > 0 : measured_ppm < 0);
        verdict("its size is the ppm asked for (+-110, the noise floor)",
                near(measured_ppm, ppm, 110));
    }
    print(serial, "  the two signs span ", measured[0] - measured[1], " ppm (expect 254)", crlf);
    verdict("the two signs span twice the trim", near(measured[0] - measured[1], 254, 150));
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

    Rtc::count(0);
    const bool cnt_set = Rtc::count_busy();
    uint32_t t0 = now();
    while (Rtc::count_busy()) {}
    uint32_t cnt_ticks = now() - t0;

    Rtc::period(32767);
    const bool per_set = Rtc::period_busy();
    t0 = now();
    while (Rtc::period_busy()) {}
    const uint32_t per_ticks = now() - t0;

    Rtc::compare(1000);
    const bool cmp_set = Rtc::compare_busy();
    t0 = now();
    while (Rtc::compare_busy()) {}
    const uint32_t cmp_ticks = now() - t0;

    Rtc::prescaler(RtcPrescaler::div1);
    const bool ctrla_set = Rtc::ctrla_busy();
    t0 = now();
    while (Rtc::ctrla_busy()) {}
    const uint32_t ctrla_ticks = now() - t0;

    Pit::period(PitPeriod::cyc32);
    const bool pit_set = Pit::ctrl_busy();
    t0 = now();
    while (Pit::ctrl_busy()) {}
    const uint32_t pit_ticks = now() - t0;

    print(serial, "  one CLK_RTC period = ", one_rtc, " crystal ticks", crlf);
    print(serial, "  CNTBUSY=", cnt_ticks, " PERBUSY=", per_ticks, " CMPBUSY=", cmp_ticks,
          " CTRLABUSY=", ctrla_ticks, " PIT CTRLBUSY=", pit_ticks, crlf);
    verdict("a write raises its busy flag",
            cnt_set && per_set && cmp_set && ctrla_set && pit_set);
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
        while (!Pit::flag()) {}
        const uint32_t d = now() - t0;
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
    uint32_t lo = 0, hi = 0, mean = 0;
    first_tick_stats(false, lo, hi, mean);
    print(serial, "  prescaler stopped (both functions off): min=", lo, " max=", hi,
          " mean=", mean, " of a ", period_ticks, "-tick period", crlf);
    verdict("every first tick inside one period (stopped)", hi <= period_ticks + period_ticks / 10);
    const uint32_t spread_stopped = hi - lo;

    first_tick_stats(true, lo, hi, mean);
    print(serial, "  prescaler free-running (counter enabled): min=", lo, " max=", hi,
          " mean=", mean, crlf);
    verdict("every first tick inside one period (running)", hi <= period_ticks + period_ticks / 10);
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
          "two windows; the exact proof is the tick ratio below)", crlf);
    verdict("the counter's period is unchanged within the oscillator's wander",
            near(static_cast<int32_t>(shared) - static_cast<int32_t>(alone), 0, static_cast<int32_t>(alone / 2000)));

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
    verdict("PIT_DIV64 = 64 CLK_RTC cycles",
            near(div1_ticks, static_cast<int32_t>(64u * (ticks_per_rtc_x100 / 100)), 2000));
    verdict("the PIT does not see the counter's PRESCALER",
            near(div1_ticks, div32_ticks, 100));
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
          static_cast<int32_t>(at1k) - static_cast<int32_t>(at32k), crlf);
    verdict("the divider is exact (0.05 %)",
            near(static_cast<int32_t>(at1k) - static_cast<int32_t>(at32k), 0, 12'000));

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

using TestFn = void (*)();
struct Test { char key; TestFn fn; };
constexpr Test tests[] = {
    {'1', t1_verbs}, {'2', t2_period}, {'3', t3_compare}, {'4', t4_correction},
    {'5', t5_busy}, {'6', t6_first_tick}, {'7', t7_together}, {'8', t8_osc1k},
};

void run(TestFn fn) {
    passed = failed = 0;
    fn();
    print(serial, "  -> ", passed, " pass, ", failed, " fail", crlf, crlf);
}

void help() {
    print(serial, "test_avr_rtc: 1 resource verbs | 2 counter period | 3 compare phase | "
                  "4 crystal error correction | 5 busy flags | 6 first tick | "
                  "7 PIT and counter together | 8 OSC1K | a all", crlf);
}

} // namespace

ISR(USART2_RXC_vect) { (void)Serial::rxc(); }
ISR(USART2_DRE_vect) { Serial::dre(); }
ISR(RTC_PIT_vect)    { Ticker::pit(); }
ISR(RTC_CNT_vect) {
    const auto f = brio::Rtc::take_flags();
    if (f.ovf) ++rtc_ovf_irqs;
    if (f.cmp) ++rtc_cmp_irqs;
}
ISR(TCB0_INT_vect) {
    if (meter_on) { cap_a = brio::FrequencyMeter<T0>::period_ticks(); ++captures; }
    else (void)T0::take_flags();
}

int main() {
    const bool xtal = SysClock::init();
    Serial::init(clock, 460800);
    Ticker::init();
    Ticker::pause();                      // the RTC is the device under test
    stopwatch_init();
    sei();
    print(serial, crlf, "test_avr_rtc - RTC/PIT test suite (clk=", xtal ? "XTAL" : "OSCHF",
          " 24 MHz, silicon rev ", hex(SYSCFG.REVID), ")", crlf);
    help();
    print(serial, "> ");
    for (;;) {
        uint8_t c;
        if (!Serial::read_byte(c)) continue;
        if (c == '\r' || c == '\n') continue;
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') { help(); }
        else if (c == 'a' || c == 'A') {
            uint8_t tp = 0, tf = 0;
            for (const Test& t : tests) { run(t.fn); tp += passed; tf += failed; }
            print(serial, "ALL: ", tp, " pass, ", tf, " fail", crlf);
        } else {
            bool found = false;
            for (const Test& t : tests) if (t.key == c) { run(t.fn); found = true; }
            if (!found) print(serial, "? for help", crlf);
        }
        print(serial, "> ");
    }
}
