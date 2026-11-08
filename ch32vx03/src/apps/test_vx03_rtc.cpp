// test_vx03_rtc - the reference bench suite for the REAL-TIME CLOCK of the
// CH32V203 and the CH32V303, and the backup domain around it:
// ch32vx03/rtc.hpp over RM ch. 6 (the counter), ch. 4 (the backup
// registers and the tamper input), 3.3.3 and 3.4.9 (the low-speed
// crystal and the clock select) and 2.4.1 (the one bit that unlocks all
// of it).
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// THE CORE'S COUNTER IS THE RULER, SO THE TREE IS THE CRYSTAL'S. What
// this suite measures is a 32.768 kHz oscillator, and a ruler half a per
// cent off would swamp the answer: the board's 8 MHz crystal is what the
// system clock is rooted in here, which puts the STK inside a twentieth
// of a per mille (the clock chapter's own measurement) and leaves the
// low-speed crystal the only unknown in the room.
//
// WHAT COSTS THE BOARD SOMETHING. Nothing here wears the flash, but two
// things are destructive in another sense: a BACKUP DOMAIN RESET wipes
// the counter, the alarm and the data registers, and a TAMPER event
// wipes the data registers too. Both are letters by name. `z` leaves
// the domain as it found it - it starts the crystal, selects it if
// nothing else already stands, and configures the counter, all of which
// a program would do at boot anyway.
//
// NOTHING IS WIRED. The one pad this chapter has is PC13, the
// TAMPER/RTC output, and the tamper letter drives it from the port at
// one end and watches the block at the other - which is either a
// measurement or a gap, and the letter says which.
//
// What is exercised, letter by letter:
//   a  THE THREE GATES: the PWR bus clock, PWR_CTLR's DBP, and what a
//      write into the domain does with each of them shut
//   b  THE LOW-SPEED CRYSTAL: started, its time to ready MEASURED, the
//      bypass refused while it runs
//   c  THE CLOCK SELECT: RTCSEL taken, RTCEN behind it, the ONE-WAY
//      refusal, and what RTCCLK would be on each of the three sources
//   d  THE COUNTER: the write window and the read synchronization, the
//      prescaler's exact-or-nothing arithmetic, the count set and read
//      back, the live divider as the fraction of a tick, and the CARRY
//      between the counter's two halves hammered across the low one's
//      roll-over
//   e  THE SECOND TICK measured against the core's counter over ten of
//      them - which is the crystal's own rate
//   f  THE ALARM on both its paths: the RTC's own vector and EXTI line
//      17, with the count in the handler and the two arrivals timed
//   g  THE OVERFLOW, staged by setting the counter two ticks short of
//      the top
//   h  THE BACKUP DATA REGISTERS: all of this device class's, the one
//      past the count refused, the calibration field, the block's own
//      reset line against the domain's, and the two pad outputs that
//      are exclusive with the tamper input
//   k  (on a class with forty-two) THE SECOND BLOCK of data registers,
//      BKP_DATAR11..42 above the tamper registers: every one written and
//      read back under two patterns, and neither block reaching into the
//      other
//   w  (by name, WIPES THE DOMAIN) the domain reset, the HSE division
//      MEASURED - 512 or 128, which 3.4.9 keys on the lot number - and
//      the LSI as RTCCLK, its rate timed against the core's counter
//   v  (by name, reboots) the data registers across a system reset
//   t  (by name, WIPES THE DATA REGISTERS) the tamper input, driven
//      from PC13's own port
//
// build: boards = v203c6,v203c8,v303cb,v303vc
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/exti.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "ch32vx03/platform.hpp"
#include "ch32vx03/reset.hpp"
#include "ch32vx03/rtc.hpp"
#include "ch32vx03/ticker.hpp"
#include "ch32vx03/usart.hpp"
#include "util/print.hpp"
#include "util/test_wait.hpp"
#include "util/testbench.hpp"

using P = brio::Ch32vx03Platform<>;

/// The board's high-speed crystal: the ruler this suite weighs a
/// 32.768 kHz oscillator against, and the root of the third RTCCLK.
inline constexpr uint32_t xtal_hz = 8'000'000;
using SysClock = brio::Clock<brio::ClockSource::pll, 96'000'000, xtal_hz>;
constexpr SysClock clock;

// ---------------------------------------------------------------------------
// The token letter v lives in.
// ---------------------------------------------------------------------------
inline constexpr uint16_t token_magic = 0x5D72;

struct Token {
    uint16_t magic;
    char letter;
    uint8_t leg;
    uint16_t pass;
    uint16_t fail;
    uint16_t wrote;   ///< what letter v put in the data registers
};
[[gnu::section(".noinit")]] inline Token token;

namespace {

using namespace brio;

using Serial = Uart<1, P>;
constexpr Serial serial;
using TamperPad = Pin<'C', 13>;

TestBench<Serial> bench;

uint32_t boot_flags = 0;

/// The three gates as THIS BOOT found them, read before anything in
/// this program opens one: what a reset leaves is a fact only the first
/// instructions can state.
bool boot_pwr_clock = false;
bool boot_bkp_clock = false;
bool boot_dbp = false;

constexpr uint32_t ticks_per_us = SysClock::hz / 1'000'000u;
constexpr uint32_t lse_hz = 32'768;

/// What the two handlers of letter f leave behind.
volatile uint32_t rtc_irqs = 0;
volatile uint16_t rtc_irq_flags = 0;
volatile uint32_t rtc_irq_count = 0;
volatile uint32_t alarm_irqs = 0;
volatile uint32_t alarm_irq_count = 0;
volatile uint32_t tamper_irqs = 0;

/// The suite's one ruler: HCLK cycles, the kernel tick and STK's
/// position composed (ch32vx03/ticker.hpp) - right over any span under
/// 2^32 cycles (44 s at 96 MHz) while interrupts are enabled, which they
/// are wherever this suite times something.
struct Ruler {
    static uint32_t now() { return Ticker::cycles(); }
    static constexpr uint32_t hz() { return SysClock::hz; }
};

/// A span of the ruler in microseconds.
uint32_t us_of(uint32_t cycles) { return cycles / ticks_per_us; }

/// The domain as every letter but the destructive ones wants it: the
/// crystal running and RTCCLK on it. False when the board has no
/// crystal or the domain already stands on something else, which the
/// letters report rather than work around.
bool domain_ready = false;

bool open_domain() {
    if (!RtcDomain::unlock(true)) {
        return false;
    }
    RtcDomain::lse_enable(true);
    if (!RtcDomain::lse_wait_ready()) {
        return false;
    }
    if (!RtcDomain::open(RtcClockSource::lse)) {
        return false;
    }
    return Rtc::synchronize();
}

/// The prescaler for a tick of `hz` out of the crystal, set inside one
/// window with the count where the caller wants it.
bool set_tick(uint32_t hz, uint32_t count) {
    const uint32_t reload = rtc_prescaler_for(lse_hz, hz);
    if (reload == rtc_prescaler_none) {
        return false;
    }
    return Rtc::configure(RtcConfig{.prescaler = reload, .count = count, .set_count = true});
}

/// Wait for the next tick, with a bound: the second flag cleared, then
/// watched. The microseconds it took, or zero if it never came.
uint32_t wait_tick(uint32_t bound_us) {
    Rtc::clear(rtc_secf);
    const std::optional<uint32_t> took =
        brio::cycles_until<Ruler>([] { return Rtc::second(); }, bound_us);
    return took ? us_of(*took) : 0u;
}

// ===========================================================================
// a - the three gates
// ===========================================================================
void ta_gates() {
    // What THIS BOOT found, before main() opened anything: both bus
    // clocks shut and DBP clear, which is what makes the door three
    // gates and not one.
    print(serial, "  at this boot: PWR bus clock ", boot_pwr_clock ? "OPEN" : "shut",
          ", BKP bus clock ", boot_bkp_clock ? "OPEN" : "shut", ", DBP ",
          boot_dbp ? "SET" : "clear", crlf);
    bench.verdict("every reset leaves the two bus clocks shut and DBP clear - the backup "
                  "domain's CONTENTS survive a reset (letter v) while the door to them "
                  "does not, so a program opens it at every boot",
                  !boot_pwr_clock && !boot_bkp_clock && !boot_dbp);

    // The PWR block's bus clock is clear at reset, and with it shut
    // PWR_CTLR does not take a write at all.
    RtcDomain::pwr_bus_clock(false);
    const bool bus_shut = !RtcDomain::pwr_bus_clock();
    const bool dbp_with_bus_shut = RtcDomain::unlocked();
    // unlock() opens the bus clock on the way, which is the whole point
    // of its being a verb and not a bit.
    const bool unlocked = RtcDomain::unlock(true);
    print(serial, "  PWR bus clock shut: DBP reads ", dbp_with_bus_shut ? 1 : 0,
          "; unlock(true) -> DBP=", RtcDomain::unlocked() ? 1 : 0, ", bus clock=",
          RtcDomain::pwr_bus_clock() ? 1 : 0, crlf);
    bench.verdict("the PWR block's bus clock is shut at reset and unlock() opens it on the "
                  "way to DBP - a program that wrote the bit alone would write nothing",
                  bus_shut && unlocked && RtcDomain::pwr_bus_clock());

    // With DBP clear the whole domain ignores a write: the backup data
    // registers, and RCC_BDCTLR's own four bits.
    Bkp::clock(true);
    (void)Bkp::data(1, 0x1234u);
    const auto before = Bkp::data(1);
    const bool locked = RtcDomain::unlock(false);
    (void)Bkp::data(1, 0xABCDu);
    const auto while_locked = Bkp::data(1);
    const uint32_t bd_before = RtcDomain::bdctlr();
    RtcDomain::lse_enable(!RtcDomain::lse_enabled());
    const uint32_t bd_after = RtcDomain::bdctlr();
    const bool reopened = RtcDomain::unlock(true);
    print(serial, "  with DBP CLEAR: BKP_DATAR1 stayed ", hex(while_locked.value_or(0)),
          " (was ", hex(before.value_or(0)), "), RCC_BDCTLR stayed ", hex(bd_after),
          " (was ", hex(bd_before), ")", crlf);
    bench.verdict("with DBP clear every write into the backup domain is DROPPED - the data "
                  "register keeps its value and RCC_BDCTLR's own bits do not move",
                  locked && while_locked.has_value() &&
                      while_locked.value() == before.value_or(0xFFFFu) &&
                      bd_after == bd_before && reopened);

    // And the same write lands once DBP is back.
    (void)Bkp::data(1, 0xABCDu);
    const auto after = Bkp::data(1);
    bench.verdict("and the same store lands the moment DBP is set again",
                  after.has_value() && after.value() == 0xABCDu);
    (void)Bkp::data(1, before.value_or(0));
}

// ===========================================================================
// b - the low-speed crystal
// ===========================================================================
void tb_crystal() {
    (void)RtcDomain::unlock(true);
    const bool was_running = RtcDomain::lse_ready();

    uint32_t start_us = 0;
    if (!was_running) {
        RtcDomain::lse_enable(true);
        // The start is up to seconds and the driver's wait is not polled
        // from here: the ruler, unlike a fold of STK's position per poll,
        // holds a span of any length under 44 s.
        brio::Stopwatch<Ruler> w;
        w.start();
        const bool ready = RtcDomain::lse_wait_ready();
        start_us = us_of(w.elapsed());
        print(serial, "  the crystal was stopped: LSERDY ", ready ? "after " : "NEVER, ",
              start_us, " us", crlf);
        bench.verdict("the 32.768 kHz crystal on this board starts: LSEON set and LSERDY "
                      "raised by the hardware, at the start-up time printed above",
                      ready);
    } else {
        print(serial, "  the crystal is already running (the domain survived the reset, "
              "which is what the domain is for)", crlf);
        bench.verdict("the crystal is running - and it did not have to be started at this "
                      "boot, because LSEON lives in the backup domain and a system reset "
                      "does not clear it",
                      RtcDomain::lse_ready());
    }

    // The bypass takes a write only with LSEON clear (3.4.9), which is
    // the one refusal this half of the register has.
    const bool refused = !RtcDomain::lse_bypass(true);
    print(serial, "  LSEBYP while the oscillator runs: ", refused ? "REFUSED" : "taken",
          ", the bit reads ", RtcDomain::lse_bypass() ? 1 : 0, crlf);
    bench.verdict("LSEBYP is refused while LSEON stands - an external square wave is a "
                  "decision taken with the oscillator off, and the driver says so instead "
                  "of writing a bit the hardware ignores",
                  refused && !RtcDomain::lse_bypass());

    static_assert(RtcDomain::has_lse_pins,
                  "this suite's board types bond the two 32 kHz oscillator pads (PC14/PC15)");
}

// ===========================================================================
// c - the clock select
// ===========================================================================
void tc_select() {
    (void)RtcDomain::unlock(true);
    const bool opened = RtcDomain::open(RtcClockSource::lse);
    const RtcClockSource now = RtcDomain::selected();
    print(serial, "  RTCSEL=", static_cast<uint32_t>(now), " RTCEN=",
          RtcDomain::enabled() ? 1 : 0, crlf);
    bench.verdict("RTCSEL takes the crystal and RTCEN stands behind it - the clock the "
                  "counter runs on is chosen in RCC and not in the RTC's own registers",
                  opened && now == RtcClockSource::lse && RtcDomain::enabled());

    // ONE-WAY: a different source is refused with nothing written, and
    // re-selecting what already stands costs nothing.
    const bool refused = !RtcDomain::select(RtcClockSource::lsi);
    const bool again = RtcDomain::select(RtcClockSource::lse);
    print(serial, "  select(LSI) with the crystal standing: ", refused ? "REFUSED" : "TAKEN",
          "; re-selecting the crystal: ", again ? "free" : "refused", crlf);
    bench.verdict("RTCSEL is ONE-WAY (3.4.9): a different source is refused while one "
                  "stands, and only a backup domain reset is the way out - which is why "
                  "select() answers a bool",
                  refused && again && RtcDomain::selected() == RtcClockSource::lse);

    // What the three sources would be worth. The third is the one the
    // manual states twice: 512 or 128 by lot number.
    print(serial, "  RTCCLK: the crystal ", lse_hz, " Hz, the LSI ", device::lsi_typ_hz,
          " Hz nominal, the HSE over ", device::rtc_hse_div[0], " = ",
          rtc_hse_clock_hz(xtal_hz, 0), " Hz or over ", device::rtc_hse_div[1], " = ",
          rtc_hse_clock_hz(xtal_hz, 1), " Hz", crlf);
    // The part table's facts, checked by the compiler: one number on a
    // part 3.4.9 names among those that divide by 128 with no lot rule
    // (every CH32V303), the pair 512 or 128 BY LOT NUMBER elsewhere - and
    // there a program that needs the rate measures it (letter w).
    static_assert(rtc_hse_divider_known
                      ? device::rtc_hse_div[0] == 128u && device::rtc_hse_div[1] == 128u
                      : device::rtc_hse_div[0] == 512u && device::rtc_hse_div[1] == 128u,
                  "the HSE division: 128 where it is known, 512 or 128 where the lot keys it");
}

// ===========================================================================
// d - the counter, its window and its synchronization
// ===========================================================================
void td_counter() {
    if (!domain_ready) {
        bench.verdict("the domain is open on the crystal", false);
        return;
    }

    // The read discipline: RSF cleared and waited for. It costs up to
    // one RTCCLK period, which on the crystal is thirty microseconds.
    brio::Stopwatch<Ruler> sync_watch;
    sync_watch.start();
    const bool synced = Rtc::synchronize();
    const uint32_t sync_us = us_of(sync_watch.elapsed());
    print(serial, "  synchronize(): ", synced ? "RSF back" : "TIMED OUT", " in ", sync_us,
          " us (one RTCCLK period is 30 us)", crlf);
    bench.verdict("the read synchronization costs at most one period of the RTC's own "
                  "clock, which is what 6.2.3 promises and what a program pays ONCE after "
                  "a reset",
                  synced && sync_us <= 200u);

    // The prescaler's arithmetic: exact or nothing.
    bench.verdict("the prescaler is exact-or-nothing: 32768 Hz makes a one-second tick "
                  "with the chapter's own 0x7FFF, a rate that does not divide whole "
                  "answers rtc_prescaler_none, and so does one past the twenty-bit field",
                  rtc_prescaler_for(lse_hz, 1) == 0x7FFFu &&
                      rtc_prescaler_for(lse_hz, 3) == rtc_prescaler_none &&
                      rtc_prescaler_for(1'500'000u, 1) == rtc_prescaler_none &&
                      rtc_tick_hz(lse_hz, 0x7FFFu) == 1u);

    // The write window: one crossing for the prescaler and the counter
    // together.
    const bool configured = set_tick(64, 0x12345678u);
    const bool closed = !Rtc::in_config() && Rtc::write_finished();
    const uint32_t read_back = Rtc::count();
    print(serial, "  configure(prescaler for 64 Hz, count 0x12345678): CNT reads ",
          hex(read_back), ", CNF ", Rtc::in_config() ? "STILL SET" : "clear", ", RTOFF ",
          Rtc::write_finished() ? "set" : "LOW", crlf);
    bench.verdict("one window sets the prescaler and the counter together, closes itself "
                  "and waits for the write to land - and the thirty-two bits read back "
                  "through two sixteen-bit registers as they were written",
                  configured && closed &&
                      (read_back == 0x12345678u || read_back == 0x12345679u));

    // The live divider: the fraction of the current tick, counting DOWN
    // towards the next one.
    const uint32_t d1 = Rtc::divider();
    brio::spin_us<Ruler>(3'000u);
    const uint32_t d2 = Rtc::divider();
    print(serial, "  RTC_DIV ", d1, " then ", d2, " three milliseconds later (the reload "
          "is ", rtc_prescaler_for(lse_hz, 64), ")", crlf);
    bench.verdict("RTC_DIV is the prescaler's live DOWN-counter and the finest time this "
                  "block offers - it moved, and it stayed inside the reload",
                  d1 != d2 && d1 <= rtc_prescaler_for(lse_hz, 64) &&
                      d2 <= rtc_prescaler_for(lse_hz, 64));

    // THE CARRY BETWEEN THE TWO HALVES, which the chapter says nothing
    // about. With the prescaler at zero the counter takes an RTCCLK
    // edge - 32768 a second - so the low half rolls over twice in the
    // four seconds below, and the loop hammers BOTH the driver's
    // count() and the raw three-read sequence it is built out of.
    (void)Rtc::configure(RtcConfig{.prescaler = 0, .count = 0x0000FF00u, .set_count = true});
    uint32_t disagreements = 0;
    uint32_t reads = 0;
    uint32_t backwards = 0;
    uint32_t worst_back = 0;
    uint32_t worst_forward = 0;
    uint32_t last = Rtc::count();
    brio::Stopwatch<Ruler> fast;
    fast.start();
    while (fast.elapsed() < brio::us_cycles<Ruler>(4'000'000u)) {
        const uint16_t high = Rtc::regs().CNTH;
        (void)Rtc::regs().CNTL;
        if (Rtc::regs().CNTH != high) {
            ++disagreements;
        }
        const uint32_t now = Rtc::count();
        if (now < last) {
            ++backwards;
            if (last - now > worst_back) {
                worst_back = last - now;
            }
        } else if (now - last > worst_forward) {
            worst_forward = now - last;
        }
        last = now;
        ++reads;
    }
    const uint32_t span = last - 0x0000FF00u;
    print(serial, "  four seconds at 32768 ticks a second: the count advanced ", span,
          " through ", span / 65536u, " roll-over(s) of its low half, over ", reads,
          " reads", crlf);
    print(serial, "  the raw three-read sequence disagreed ", disagreements,
          " time(s); count() stepped BACK ", backwards, " time(s), the worst by ",
          worst_back, ", and forward at most ", worst_forward, crlf);
    bench.verdict("NO READ WAS EVER TORN across the low half's roll-over - the worst step "
                  "in either direction is a tick or two and never the 65536 a half-read "
                  "counter would give - but THE COUNT DOES STEP BACK by a tick now and "
                  "then, which is the bus reading a register of another clock domain and "
                  "seeing a stale copy: a program that reads this counter faster than it "
                  "ticks cannot assume the number only grows",
                  span > 100'000u && worst_back <= 16u && worst_forward <= 16u);

    // The counter moves on its own, which is the one thing a counter
    // has to do.
    (void)set_tick(64, 0x12345678u);
    const uint32_t c1 = Rtc::count();
    brio::spin_us<Ruler>(60'000u);
    const uint32_t c2 = Rtc::count();
    print(serial, "  CNT ", hex(c1), " then ", hex(c2), " some sixty milliseconds later at "
          "64 ticks a second", crlf);
    bench.verdict("the counter counts: at sixty-four ticks a second it advanced by two to "
                  "five over sixty milliseconds",
                  c2 > c1 && (c2 - c1) <= 8u);
}

// ===========================================================================
// e - the second tick, and the crystal's rate
// ===========================================================================
void te_rate() {
    if (!domain_ready) {
        bench.verdict("the domain is open on the crystal", false);
        return;
    }

    const bool configured = set_tick(1, 0);
    bench.verdict("the counter is set to one tick a second, the chapter's own example",
                  configured);

    // Ten ticks against the core's counter. The first is thrown away,
    // because the loop joined the tick wherever it happened to be.
    (void)wait_tick(2'000'000u);
    Rtc::clear(rtc_secf);
    brio::Stopwatch<Ruler> w;
    w.start();
    uint32_t ticks = 0;
    while (ticks < 10u) {
        if (Rtc::second()) {
            Rtc::clear(rtc_secf);
            ++ticks;
        }
        if (w.elapsed() > brio::us_cycles<Ruler>(15'000'000u)) {
            break;
        }
    }
    const uint32_t span_us = us_of(w.elapsed());
    const uint32_t period_us = span_us / (ticks == 0u ? 1u : ticks);
    // RTCCLK = (reload + 1) / period. In hundredths of a hertz, so the
    // number has two digits after the point without floating point.
    const uint32_t rtcclk_centi =
        period_us == 0u ? 0u : static_cast<uint32_t>((32768ULL * 100'000'000ULL) / period_us);
    print(serial, "  ten seconds took ", span_us, " us, one tick ", period_us,
          " us - which puts the crystal at ", rtcclk_centi / 100u, ".",
          (rtcclk_centi / 10u) % 10u, (rtcclk_centi % 10u), " Hz against a nominal ",
          lse_hz, crlf);
    const int32_t ppm =
        static_cast<int32_t>((static_cast<int64_t>(period_us) - 1'000'000) * 1000 / 1000);
    print(serial, "  that is ", ppm >= 0 ? "+" : "-",
          static_cast<uint32_t>(ppm < 0 ? -ppm : ppm), " us a second, ",
          ppm >= 0 ? "slow" : "fast", crlf);
    bench.verdict("ten ticks of the second event against the core's crystal-rooted counter "
                  "put the low-speed crystal within a hundred parts per million of 32.768 "
                  "kHz - which is what a watch crystal is sold as",
                  ticks == 10u && period_us > 999'000u && period_us < 1'001'000u);
}

// ===========================================================================
// f - the alarm, on both its paths
// ===========================================================================
void tf_alarm() {
    if (!domain_ready) {
        bench.verdict("the domain is open on the crystal", false);
        return;
    }

    // Sixty-four ticks a second, so an alarm two ticks away is thirty
    // milliseconds and not two seconds.
    (void)set_tick(64, 0);

    // ---- the RTC's own vector -------------------------------------------
    rtc_irqs = 0;
    rtc_irq_count = 0;
    Rtc::clear(rtc_secf | rtc_alrf | rtc_owf);
    const uint32_t target = Rtc::count() + 4u;
    const bool armed = Rtc::alarm(target);
    const bool ints = Rtc::interrupts(rtc_alrie);
    const bool ints_read = Rtc::interrupts() == rtc_alrie;
    const bool outside_window = !Rtc::in_config();
    Pfic::enable(Irq::rtc);
    const std::optional<uint32_t> own =
        brio::cycles_until<Ruler>([] { return rtc_irqs != 0u; }, 500'000u);
    const uint32_t own_us = own ? us_of(*own) : 500'000u;
    Pfic::disable(Irq::rtc);
    (void)Rtc::interrupts(0);
    print(serial, "  the RTC's own vector: alarm armed at ", hex(target), ", the handler "
          "ran ", rtc_irqs, " time(s) after ", own_us, " us with CNT=", hex(rtc_irq_count),
          " and flags ", hex(rtc_irq_flags), crlf);
    // Five ticks after the count was set to zero, on every die measured:
    // the event is raised as the counter LEAVES the armed value. Which
    // count the handler then reads - the armed value, or the one past it
    // - is a question 6.2.3 leaves open (the count's copy on PB1 is "not
    // necessarily real-time"), nothing in brio depends on (the sleep
    // site resynchronizes before it reads), and the dies answer
    // differently: the old count on the CH32V303VCT6, the new one on the
    // CH32V203C8T6 and the CH32V303CBT6. It is printed, and the verdict
    // asks only that the read lands on one of the two.
    const bool read_armed = rtc_irq_count == target;
    const bool read_next = rtc_irq_count == target + 1u;
    print(serial, "  FINDING: the handler read ", read_armed ? "the ARMED value itself" : read_next ?
                  "the armed value PLUS ONE" : "neither the armed value nor the one past it", crlf);
    bench.verdict("the alarm reaches the RTC's own vector, and the counter the handler reads is "
                  "the armed value or the one past it: the event is raised as the counter LEAVES "
                  "the number it was armed at (five ticks after a count of zero); which of the two "
                  "the bus reads by then is the die's, printed above",
                  armed && ints && rtc_irqs >= 1u && (read_armed || read_next));
    bench.verdict("and RTC_CTLRH takes its write OUTSIDE the configuration window - "
                  "`interrupts()` waits for RTOFF and stores, with no CNF around it - "
                  "which is 6.2.2's list of the four backup-domain registers read for "
                  "what it does NOT contain: the interrupt enables are not one of them",
                  ints_read && outside_window);
    bench.verdict("and it arrived four to five ticks of a sixty-four hertz counter later, "
                  "which is the time the program asked for",
                  own_us > 40'000u && own_us < 100'000u);

    // ---- EXTI line 17, the path that survives a low-power mode -----------
    // First WITHOUT the alarm's own interrupt enable, which is the
    // question neither chapter answers: does the event reach the line
    // on its own?
    alarm_irqs = 0;
    Rtc::clear(rtc_secf | rtc_alrf | rtc_owf);
    (void)Exti::clear(Rtc::wake_line);
    const bool line_armed = Rtc::arm_wake(true);
    Pfic::enable(Irq::rtc_alarm);
    const uint32_t t2 = Rtc::count() + 4u;
    (void)Rtc::alarm(t2);
    const std::optional<uint32_t> line =
        brio::cycles_until<Ruler>([] { return alarm_irqs != 0u; }, 500'000u);
    const uint32_t without_ie = alarm_irqs;
    const uint32_t line_us = line ? us_of(*line) : 500'000u;

    // Then WITH it, if the line stayed quiet.
    uint32_t with_ie = 0;
    uint32_t with_ie_us = 0;
    if (without_ie == 0u) {
        alarm_irqs = 0;
        Rtc::clear(rtc_secf | rtc_alrf | rtc_owf);
        (void)Exti::clear(Rtc::wake_line);
        (void)Rtc::interrupts(rtc_alrie);
        const uint32_t t3 = Rtc::count() + 4u;
        (void)Rtc::alarm(t3);
        const std::optional<uint32_t> took =
            brio::cycles_until<Ruler>([] { return alarm_irqs != 0u; }, 500'000u);
        with_ie = alarm_irqs;
        with_ie_us = took ? us_of(*took) : 500'000u;
    }
    Pfic::disable(Irq::rtc_alarm);
    (void)Rtc::arm_wake(false);
    (void)Rtc::interrupts(0);

    print(serial, "  EXTI line 17 with ALRIE CLEAR: ", without_ie, " handler call(s) in ",
          line_us, " us; with ALRIE set: ", with_ie, " in ", with_ie_us, " us", crlf);
    bench.verdict("the alarm reaches its SECOND vector through EXTI line 17 WITH THE "
                  "BLOCK'S OWN INTERRUPT ENABLE CLEAR: the event leaves the peripheral by "
                  "itself, where an EXTI line INTO a peripheral of this family needs that "
                  "peripheral's event enable - so the wake a low-power mode will use costs "
                  "no vector of the RTC's own",
                  line_armed && without_ie >= 1u);

    // The alarm register cannot be read back at all, which is a fact a
    // program has to hold rather than discover - stated, not measured.
    print(serial, "  RTC_ALRM is write-only (6.3.9, 6.3.10): a program that wants to know what "
          "it armed keeps the number itself, and this driver invents no shadow for it", crlf);
}

// ===========================================================================
// g - the overflow
// ===========================================================================
void tg_overflow() {
    if (!domain_ready) {
        bench.verdict("the domain is open on the crystal", false);
        return;
    }

    // Two ticks short of the top at sixty-four a second: thirty
    // milliseconds to the wrap.
    (void)set_tick(64, 0xFFFFFFFEu);
    Rtc::clear(rtc_secf | rtc_alrf | rtc_owf);
    const std::optional<uint32_t> took =
        brio::cycles_until<Ruler>([] { return Rtc::overflowed(); }, 500'000u);
    const uint32_t us = took ? us_of(*took) : 500'000u;
    const uint32_t at_flag = Rtc::count();
    brio::spin_us<Ruler>(2'000u);
    const uint32_t after = Rtc::count();
    print(serial, "  set two ticks short of 0xFFFFFFFF: OWF after ", us,
          " us, CNT read at the flag ", hex(at_flag), " and two milliseconds later ",
          hex(after), crlf);
    bench.verdict("the counter wraps and raises the overflow event - the only warning a "
                  "thirty-two-bit number gives, which at one tick a second is a hundred "
                  "and thirty-six years away and is why a program that cares keeps the "
                  "epoch itself",
                  Rtc::overflowed() && us > 20'000u && us < 200'000u);
    bench.verdict("AND THE FLAG ARRIVES BEFORE THE COUNT THE BUS READS: at the moment OWF "
                  "stands the counter still reads its last value, and the wrapped one "
                  "appears afterwards - the two registers cross a clock domain and the "
                  "flag is the faster of the two, so a handler that wants the count reads "
                  "it and not the event",
                  at_flag == 0xFFFFFFFFu && after < 0x10u);
    Rtc::clear(rtc_owf);
    bench.verdict("and the flag is rw0: a store of ones everywhere but its own bit is what "
                  "clears it, and it stays down",
                  !Rtc::overflowed());
}

// ===========================================================================
// h - the backup data registers and the block around them
// ===========================================================================
void th_backup() {
    const bool opened = Bkp::open();
    print(serial, "  this device class carries ", Bkp::count, " backup data registers, and "
          "the tamper pad is ", Bkp::has_tamper_pad ? "bonded" : "not bonded", crlf);
    static_assert(Bkp::count == (device::device_class == DeviceClass::v20x_d6 ? 10u : 42u),
                  "the data register count is the device class's: ten on the CH32V20x_D6, "
                  "forty-two on the other two");
    bench.verdict("the block opens with both bus clocks and DBP", opened);

    // Every register of this class, written and read back.
    bool all = true;
    for (uint8_t n = 1; n <= Bkp::count; ++n) {
        if (!Bkp::data(n, static_cast<uint16_t>(0xB000u + n))) {
            all = false;
        }
    }
    for (uint8_t n = 1; n <= Bkp::count; ++n) {
        const auto v = Bkp::data(n);
        if (!v.has_value() || v.value() != static_cast<uint16_t>(0xB000u + n)) {
            all = false;
        }
    }
    bench.verdict("every one of them holds sixteen bits, written and read back through the "
                  "chapter's own numbering, which starts at one",
                  all);

    // The one past the class's count is refused at run time by an empty
    // optional, and at compile time by a static_assert where the index
    // is a constant (test/family_ch32vx03/neg).
    const bool past = !Bkp::data(static_cast<uint8_t>(Bkp::count + 1u)).has_value();
    const bool zero = !Bkp::data(0).has_value();
    const bool past_write = !Bkp::data(static_cast<uint8_t>(Bkp::count + 1u), 0x1234u);
    bench.verdict("the register past the class's count and the zeroth are refused with an "
                  "empty optional and nothing written",
                  past && zero && past_write);

    // The calibration field: seven bits that only SLOW the clock.
    const bool cal = Bkp::calibration(0x40u);
    const bool cal_read = Bkp::calibration() == 0x40u;
    const bool cal_refused = !Bkp::calibration(0x80u);
    (void)Bkp::calibration(0);
    print(serial, "  CAL: 0x40 ", cal ? "written" : "REFUSED", ", reads back ",
          cal_read ? "0x40" : "wrong", ", 0x80 ", cal_refused ? "refused" : "TAKEN", crlf);
    bench.verdict("CAL is seven bits of pulses SKIPPED every 2^20 - it can slow the clock "
                  "by up to 121 ppm and can never speed it up - and a value past the field "
                  "is refused",
                  cal && cal_read && cal_refused && Bkp::calibration() == 0u);

    // The pad's two other outputs, and their exclusion with the tamper
    // input: the block has one pad and three things want it.
    const bool cco_on = Bkp::clock_output(true);
    const bool cco_reads = Bkp::clock_output();
    (void)Bkp::clock_output(false);
    const bool pulse_on = Bkp::pulse_output(true, BkpPulse::second);
    const bool pulse_reads = Bkp::pulse_output() && Bkp::pulse_source() == BkpPulse::second;
    const bool alarm_source = Bkp::pulse_output(true, BkpPulse::alarm) &&
                              Bkp::pulse_source() == BkpPulse::alarm;
    (void)Bkp::pulse_output(false);
    bench.verdict("the TAMPER pad carries RTCCLK over 64 or a pulse on every second or "
                  "alarm event, and each of the three is a bit of BKP_OCTLR read back as "
                  "written",
                  cco_on && cco_reads && pulse_on && pulse_reads && alarm_source &&
                      !Bkp::pulse_output());

    // THE BLOCK'S OWN RESET LINE IS NOT THE DOMAIN'S. 4.2.4 says BKPRST
    // does not touch the data registers; what it does reach is the rest
    // of the block, which is what makes the two resets worth telling
    // apart.
    (void)Bkp::calibration(0x2A);
    (void)Bkp::data(3, 0xFACEu);
    Bkp::reset_block();
    const auto kept = Bkp::data(3);
    const uint8_t cal_after = Bkp::calibration();
    print(serial, "  after the block's own RCC reset line: BKP_DATAR3 reads ",
          hex(kept.value_or(0)), " and CAL ", hex(cal_after), crlf);
    bench.verdict("the block's own reset line moves NOTHING in the register file - not the "
                  "data registers (4.2.4 says so) and not the calibration either: BKPRST "
                  "resets the backup INTERFACE and what lies behind it is the domain's, so "
                  "a program that wants any of it cleared asks for a domain reset",
                  kept.value_or(0) == 0xFACEu && cal_after == 0x2Au);
    (void)Bkp::calibration(0);

    // And with the tamper input holding the pad, both are refused - the
    // exclusion 4.3.2's note 1 states.
    (void)Bkp::tamper(true, true);
    const bool cco_refused = !Bkp::clock_output(true);
    const bool pulse_refused = !Bkp::pulse_output(true);
    (void)Bkp::tamper(false);
    bench.verdict("and both are REFUSED while the tamper input holds that pad, which is "
                  "the exclusion the chapter states and the driver enforces rather than "
                  "letting two functions fight over one pin",
                  cco_refused && pulse_refused && !Bkp::tamper_enabled());
}

// ===========================================================================
// k - the second block of data registers (a class with forty-two)
// ===========================================================================
/// BKP_DATAR11..42 sit in a second block above the tamper registers
/// (RM 4.3, table 4-1), so a register numbered past ten is not simply
/// the next word: every one is written and read back under two patterns,
/// each block checked after the other was written, which catches a
/// numbering that folds one block onto the other.
template <uint8_t count = Bkp::count>
void tk_second_block() {
    if constexpr (count == 42u) {
        (void)Bkp::open();
        constexpr uint16_t patterns[2] = {0x5A00u, 0xA500u};
        bool every = true;
        bool first_block_kept = true;
        for (const uint16_t base : patterns) {
            for (uint8_t n = 1; n <= 10u; ++n) {
                (void)Bkp::data(n, static_cast<uint16_t>(base | n));
            }
            // The second block, written with the OTHER pattern's bits.
            for (uint8_t n = 11; n <= 42u; ++n) {
                (void)Bkp::data(n, static_cast<uint16_t>((base ^ 0xFF00u) | n));
            }
            for (uint8_t n = 1; n <= 10u; ++n) {
                if (Bkp::data(n).value_or(0) != static_cast<uint16_t>(base | n)) {
                    first_block_kept = false;
                }
            }
            for (uint8_t n = 11; n <= 42u; ++n) {
                if (Bkp::data(n).value_or(0) != static_cast<uint16_t>((base ^ 0xFF00u) | n)) {
                    every = false;
                }
            }
        }
        const auto eleventh = Bkp::data(11);
        const auto last = Bkp::data(42);
        const bool past = !Bkp::data(43).has_value();
        print(serial, "  BKP_DATAR11 reads ", hex(eleventh.value_or(0)), ", BKP_DATAR42 ",
              hex(last.value_or(0)), "; the forty-third ", past ? "refused" : "ANSWERED", crlf);
        bench.verdict("the thirty-two registers of the second block hold sixteen bits each "
                      "under two patterns, read back through their own numbers",
                      every);
        bench.verdict("and writing them leaves the first block's ten as they were - the two "
                      "blocks are two sets of registers and not one numbered twice",
                      first_block_kept);
        bench.verdict("the forty-third is refused with nothing written", past);
    }
}

// ===========================================================================
// w - the domain reset, and the HSE division measured (WIPES THE DOMAIN)
// ===========================================================================
void tw_domain_reset() {
    (void)RtcDomain::unlock(true);
    Bkp::clock(true);

    // Something to lose, so that the wipe is observed and not assumed.
    (void)Bkp::data(1, 0xDEADu);
    (void)Bkp::data(10, 0xBEEFu);
    const auto before = Bkp::data(1);

    RtcDomain::reset();
    const auto after = Bkp::data(1);
    const RtcClockSource source = RtcDomain::selected();
    print(serial, "  BDRST: BKP_DATAR1 went from ", hex(before.value_or(0)), " to ",
          hex(after.value_or(0)), ", RTCSEL back to ", static_cast<uint32_t>(source),
          ", LSEON ", RtcDomain::lse_enabled() ? 1 : 0, crlf);
    bench.verdict("a backup domain reset wipes the data registers, puts RTCSEL back to "
                  "none and stops the crystal - which is the way back from every one-way "
                  "bit in this register, and the only one",
                  before.value_or(0) == 0xDEADu && after.value_or(0xFFFFu) == 0u &&
                      source == RtcClockSource::none && !RtcDomain::lse_enabled());

    // THE HSE DIVISION, MEASURED. 3.4.9 gives 512 or 128 by lot number,
    // so with the domain empty the third source is chosen and its tick
    // is timed: 8 MHz over 512 is 15625 Hz, over 128 it is 62500.
    const bool selected = RtcDomain::open(RtcClockSource::hse_divided);
    (void)Rtc::synchronize();
    const uint32_t reload = rtc_prescaler_for(rtc_hse_clock_hz(xtal_hz, 0), 1);
    const bool configured =
        Rtc::configure(RtcConfig{.prescaler = reload, .count = 0, .set_count = true});
    (void)wait_tick(3'000'000u);
    const uint32_t period_us = wait_tick(3'000'000u);
    const uint32_t implied = period_us == 0u ? 0u
                                             : static_cast<uint32_t>((static_cast<uint64_t>(
                                                   reload + 1u) * 1'000'000ULL) / period_us);
    const uint32_t div = implied == 0u ? 0u : xtal_hz / implied;
    print(serial, "  RTCSEL on the HSE: a tick asked for at one hertz through a reload of ",
          reload, " took ", period_us, " us - RTCCLK is ", implied, " Hz, the crystal over ",
          div, crlf);
    bench.verdict("the HSE division on THIS die is the number above: 3.4.9 states 512 or "
                  "128 and keys the choice on the lot number, so the part table carries "
                  "both and a program that clocks the RTC from the crystal measures which "
                  "it has",
                  selected && configured && period_us != 0u &&
                      (div == device::rtc_hse_div[0] || div == device::rtc_hse_div[1]));

    // THE LSI AS RTCCLK: the third source, which costs one more domain
    // reset (RTCSEL is one-way). The RC is started by the program here -
    // no watchdog forces it - the tick asked for at one hertz at the
    // part's typical rate, and five of them timed against the core's
    // crystal-rooted counter: (reload + 1) / period is the RC's rate.
    RtcDomain::reset();
    const bool lsi_up = Rcc::lsi_start();
    const bool lsi_selected = RtcDomain::open(RtcClockSource::lsi);
    (void)Rtc::synchronize();
    const uint32_t lsi_reload = rtc_prescaler_for(device::lsi_typ_hz, 1);
    const bool lsi_configured =
        Rtc::configure(RtcConfig{.prescaler = lsi_reload, .count = 0, .set_count = true});
    (void)wait_tick(3'000'000u);
    Rtc::clear(rtc_secf);
    brio::Stopwatch<Ruler> lw;
    lw.start();
    uint32_t lsi_ticks = 0;
    while (lsi_ticks < 5u && lw.elapsed() < brio::us_cycles<Ruler>(10'000'000u)) {
        if (Rtc::second()) {
            Rtc::clear(rtc_secf);
            ++lsi_ticks;
        }
    }
    const uint32_t lsi_span = us_of(lw.elapsed());
    const uint32_t lsi_count = Rtc::count();
    const uint32_t lsi_rate =
        lsi_ticks == 0u ? 0u
                        : static_cast<uint32_t>((lsi_reload + 1ULL) * lsi_ticks * 1'000'000ULL /
                                                lsi_span);
    print(serial, "  RTCSEL on the LSI: five ticks asked for at one hertz through a reload of ",
          lsi_reload, " took ", lsi_span, " us, the counter at ", lsi_count,
          " - RTCCLK, the LSI, is ", lsi_rate, " Hz", crlf);
    bench.verdict("the LSI clocks the counter: RTCSEL takes it, the second tick runs and the "
                  "counter counts it, and the rate it implies lies in the part's rated 25 to "
                  "60 kHz",
                  lsi_up && lsi_selected && lsi_configured && lsi_ticks == 5u &&
                      lsi_count >= 5u && lsi_rate >= device::lsi_min_hz &&
                      lsi_rate <= device::lsi_max_hz);
    Rcc::lsi_stop();

    // And back to the crystal, which needs the domain reset again
    // because RTCSEL is one-way.
    RtcDomain::reset();
    domain_ready = open_domain();
    const bool back = domain_ready && RtcDomain::selected() == RtcClockSource::lse;
    (void)set_tick(1, 0);
    print(serial, "  the domain reset again and re-opened on the crystal: ",
          back ? "ready" : "FAILED", crlf);
    bench.verdict("and a second domain reset is what lets the crystal be chosen again - "
                  "the counter, the prescaler, the alarm and the data registers being "
                  "the price of every such change",
                  back);
    bench.end_letter();
}

// ===========================================================================
// v - the data registers across a system reset (reboots)
// ===========================================================================
void tv_across_reset() {
    bench.reset_tally();
    (void)Bkp::open();
    token.magic = token_magic;
    token.letter = 'v';
    token.leg = 1;
    token.pass = 0;
    token.fail = 0;
    token.wrote = 0xC3A5u;
    for (uint8_t n = 1; n <= Bkp::count; ++n) {
        (void)Bkp::data(n, static_cast<uint16_t>(token.wrote + n));
    }
    const uint32_t count = Rtc::count();
    print(serial, "  ", Bkp::count, " registers written from ", hex(token.wrote),
          ", the counter at ", hex(count), ", rebooting...", crlf);
    Reset::software();
}

void tv_resume() {
    print(serial, crlf, "-> back from the software reset", crlf);
    // The gates this boot found, captured before main() opened them.
    const bool shut = !boot_bkp_clock && !boot_pwr_clock && !boot_dbp;
    const bool opened = Bkp::open();
    bool all = true;
    for (uint8_t n = 1; n <= Bkp::count; ++n) {
        const auto v = Bkp::data(n);
        if (!v.has_value() || v.value() != static_cast<uint16_t>(token.wrote + n)) {
            all = false;
        }
    }
    print(serial, "  the block came up with its clocks ", shut ? "SHUT and DBP clear"
          : "OPEN", "; re-opened, the data registers read ",
          all ? "exactly what was written" : "WRONG", crlf);
    bench.verdict("the backup data registers survive a system reset - the domain is not "
                  "reset with the rest of the chip, which is what makes them the one place "
                  "a program can leave something across a reboot with no flash wear",
                  shut && opened && all);

    const bool running = RtcDomain::lse_ready() &&
                         RtcDomain::selected() == RtcClockSource::lse && RtcDomain::enabled();
    const uint32_t count = Rtc::count();
    print(serial, "  and the clock kept running: RTCSEL and RTCEN still set, CNT=",
          hex(count), crlf);
    bench.verdict("the crystal, the clock select and the counter came through the reset "
                  "too: a program that reboots finds the time it left, and only its "
                  "INTERRUPT enables gone",
                  running && Rtc::interrupts() == 0u);
    bench.end_letter();
}

// ===========================================================================
// t - the tamper input (WIPES THE DATA REGISTERS)
// ===========================================================================
/// One attempt at raising the tamper event from the pad's own side:
/// the level put on PC13 the way `how` describes, the input armed on
/// that level, and the flag watched. The microseconds it took, or zero.
uint32_t tamper_attempt(const char* how, bool active_low, bool use_pull) {
    (void)Bkp::tamper(false);
    Bkp::clear_event();
    Bkp::clear_interrupt();
    tamper_irqs = 0;

    // The pad at the INACTIVE level first, so that arming cannot fire
    // on the remembered edge 4.2.2 describes.
    if (use_pull) {
        TamperPad::input(active_low ? PinPull::up : PinPull::down);
    } else {
        TamperPad::output(active_low);
    }
    const bool armed = Bkp::tamper(true, active_low);
    const bool quiet = !Bkp::tamper_event();

    // And then the level it is watching for.
    if (use_pull) {
        TamperPad::input(active_low ? PinPull::down : PinPull::up);
    } else {
        TamperPad::output(!active_low);
    }
    const std::optional<uint32_t> took =
        brio::cycles_until<Ruler>([] { return Bkp::tamper_event(); }, 50'000u);
    const uint32_t us = took ? us_of(*took) : 0u;
    print(serial, "  ", how, ": armed=", armed ? 1 : 0, " quiet=", quiet ? 1 : 0,
          ", the pad reads ", TamperPad::read() ? 1 : 0, ", TEF ",
          us != 0u ? "after " : "NEVER, ", us, " us, the handler ran ", tamper_irqs,
          " time(s)", crlf);
    return us;
}

void tt_tamper() {
    if (!Bkp::has_tamper_pad) {
        bench.verdict("this package does not bond PC13, so the tamper input cannot be "
                      "driven here",
                      false);
        bench.end_letter();
        return;
    }
    (void)Bkp::open();
    (void)Bkp::tamper(false);
    Bkp::clear_event();

    // Something to lose.
    for (uint8_t n = 1; n <= Bkp::count; ++n) {
        (void)Bkp::data(n, static_cast<uint16_t>(0x7000u + n));
    }
    const auto before = Bkp::data(1);

    // Two ways to put a level on a pad with no wire, because the first
    // is the one the block may take away: an OUTPUT driver, which TPE
    // may disconnect when it claims the pad, and a PULL, which lives in
    // the port's own registers and may survive that claim.
    Bkp::tamper_interrupt(true);
    Pfic::enable(Irq::tamper);
    const uint32_t driven = tamper_attempt("the pad driven by its own output stage", false,
                                           false);
    const uint32_t pulled = driven != 0u
                                ? 0u
                                : tamper_attempt("the pad pulled by its own port", false,
                                                 true);
    const uint32_t irqs = tamper_irqs;
    const auto kept = Bkp::data(1);

    if (driven != 0u || pulled != 0u) {
        // A write while TEF stands is dropped, which is the one thing
        // that makes this flag worth reading outside a handler. The
        // vector's body writes CTE, so TEF may already be down here: it
        // is read right before the write, and the drop is judged only
        // when it stood.
        const bool tef_stood = Bkp::tamper_event();
        (void)Bkp::data(1, 0x1234u);
        const auto still = Bkp::data(1);
        print(serial, "  a write into a data register with TEF ", tef_stood ? "standing" : "DOWN",
              ": reads ", hex(still.value_or(0)), crlf);
        bench.verdict("the tamper event is raised from the pad's own side, the data "
                      "registers are wiped by the hardware and the vector runs",
                      kept.value_or(0xFFFFu) == 0u && irqs >= 1u);
        if (tef_stood) {
            bench.verdict("and while TEF stands every write into a data register is DROPPED "
                          "(4.3.4) - the flag is a state and not only a report",
                          still.value_or(0xFFFFu) == 0u);
        } else {
            bench.skip("TEF was already down at the write (the vector's CTE): the drop "
                       "while it stands is not judged");
        }
    } else {
        print(serial, "  the registers still read ", hex(kept.value_or(0)), " (they held ",
              hex(before.value_or(0)), ")", crlf);
        bench.verdict("THE TAMPER INPUT CANNOT BE RAISED FROM THIS BOARD'S OWN SIDE: "
                      "neither the pad's output stage - which TPE takes away when it "
                      "claims the pin - nor the port's pull reaches the detector, so the "
                      "data registers are untouched and what would measure this input is a "
                      "WIRE from another pad",
                      kept.value_or(0) == before.value_or(0xFFFFu) && irqs == 0u);
    }

    // And whatever happened, the block is left as it was found.
    (void)Bkp::tamper(false);
    Bkp::clear_event();
    Bkp::clear_interrupt();
    Pfic::disable(Irq::tamper);
    Bkp::tamper_interrupt(false);
    TamperPad::input();
    const bool cleared = !Bkp::tamper_event();
    const bool writes_again = Bkp::data(1, 0x55AAu) && Bkp::data(1).value_or(0) == 0x55AAu;
    bench.verdict("CTE puts the flag down and the registers take a write again, so a "
                  "program that survived a tamper can start keeping things once more",
                  cleared && writes_again);
    bench.end_letter();
}

void banner() {
    print(serial, crlf, "test_vx03_rtc on ", device::part_name,
          " - the RTC (RM ch. 6) and the backup domain (ch. 4)", crlf,
          "  z leaves the domain as it found it", crlf,
          "  w WIPES THE DOMAIN, t wipes the data registers, v reboots - by name", crlf,
          "  the domain: ", domain_ready ? "open on the crystal" : "NOT READY", crlf, crlf);
    bench.menu();
    print(serial, crlf);
}

}  // namespace

BRIO_CH32_LEAF_VECTOR(systick_handler) { brio::Ticker::tick(); }

BRIO_CH32_LEAF_VECTOR(usart1_handler) { (void)Serial::isr(); }

/// The RTC's own vector: the second, the alarm and the overflow all
/// arrive here.
BRIO_CH32_VECTOR(rtc_handler) {
    rtc_irq_count = brio::Rtc::count();
    rtc_irq_flags = brio::Rtc::isr();
    rtc_irqs = rtc_irqs + 1u;
}

/// The ALARM's second vector, behind EXTI line 17 - the path that
/// survives a low-power mode.
BRIO_CH32_VECTOR(rtc_alarm_handler) {
    alarm_irq_count = brio::Rtc::count();
    (void)brio::Rtc::alarm_isr();
    alarm_irqs = alarm_irqs + 1u;
}

/// The tamper vector: the event stood, both flags cleared.
BRIO_CH32_VECTOR(tamper_handler) {
    (void)brio::Bkp::isr();
    tamper_irqs = tamper_irqs + 1u;
}

int main() {
    const bool clock_ok = SysClock::init();
    boot_flags = brio::Reset::take_flags();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    // BEFORE the domain is opened: what the reset left. DBP reads as
    // zero with PWR's bus clock shut, so the clock is opened for the one
    // read - after the RCC read-back that orders the enable before it -
    // and shut again as the reset left it.
    boot_pwr_clock = brio::RtcDomain::pwr_bus_clock();
    boot_bkp_clock = brio::Bkp::clock();
    if (!boot_pwr_clock) {
        brio::RtcDomain::pwr_bus_clock(true);
        (void)brio::RtcDomain::pwr_bus_clock();
    }
    boot_dbp = brio::RtcDomain::unlocked();
    if (!boot_pwr_clock) {
        brio::RtcDomain::pwr_bus_clock(false);
    }

    domain_ready = open_domain();

    bench.letter('a', "the three gates: the bus clock, DBP, and a write with each shut",
                 ta_gates);
    bench.letter('b', "the low-speed crystal: its start-up and its bypass", tb_crystal);
    bench.letter('c', "the clock select, one-way, and the three sources", tc_select);
    bench.letter('d', "the counter: the window, the synchronization, the divider",
                 td_counter);
    bench.letter('e', "the second tick against the core's counter: the crystal's rate",
                 te_rate);
    bench.letter('f', "the alarm on both its paths", tf_alarm);
    bench.letter('g', "the overflow, staged two ticks short of the top", tg_overflow);
    bench.letter('h', "the backup registers, the calibration and the pad's three uses",
                 th_backup);
    if constexpr (brio::Bkp::count == 42u) {
        bench.letter('k', "the second block of data registers, 11 to 42", tk_second_block<>);
    }
    bench.letter('w', "THE DOMAIN RESET and the HSE division measured (wipes the domain)",
                 tw_domain_reset, false);
    bench.letter('v', "THE DATA REGISTERS ACROSS A SYSTEM RESET (reboots)", tv_across_reset,
                 false);
    bench.letter('t', "THE TAMPER INPUT from PC13's own port (wipes the registers)",
                 tt_tamper, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL96 on the crystal" : "FAILED",
                    " tick=", tick_ok ? "STK" : "FAILED", brio::crlf);
        if (token.magic == token_magic && token.letter == 'v') {
            token.letter = '\0';
            bench.reset_tally();
            bench.resume_tally(token.pass, token.fail);
            tv_resume();
        } else {
            banner();
        }
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
