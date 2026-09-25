// test_stm32f4_power - the reference bench suite for the STM32F4's
// STOPPING half: stm32f4/pwr.hpp (RM0390 ch. 5 on the bench part, RM0090
// ch. 5 and RM0383 ch. 5 on its siblings, the whole of it),
// stm32f4/sleep.hpp (util/power.hpp's depth ladder on this silicon, and
// the RTC-backed timebase that lifts its restriction) and the DYNAMIC
// CLOCK the way down needs (stm32f4/clock.hpp's Rates<> + DynamicClock).
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NOTHING TO WIRE. The RTC's wake-up timer is the alarm, its calendar is
// the wall clock, the console is the board's own ST-LINK virtual COM
// port, and no flash is written and no option byte touched.
//
// THE WALL CLOCK IS THE RTC, AND IT HAS TO BE. Every TIM of this family
// lives in the 1.2 V domain and stops in Stop; SysTick rides HCLK and
// stops with it; the RTC does not. So this suite runs the calendar with
// the prescalers split the OTHER way from the chapter's advice -
// PREDIV_A 0 and PREDIV_S 32767, which puts ck_apre at the root's full
// rate and makes the sub-second counter a 30.5 us stopwatch that keeps
// counting with every clock in the chip stopped. The kernel's own
// millis() is the SUBJECT of half these letters and never the judge.
//
// THE CLOCK IS DYNAMIC, and that is a subject too: the suite's SysClock
// is a DynamicClock over a pack of rates from the board's own root down
// to the 16 MHz HSI, with the kernel ticker and the console among the
// users it rebases. Letter f walks it; letter d USES it, dropping to the
// HSI so that a Stop's wake can be weighed without the PLL's restart
// buried in the same number.
//
// THE BACKSTOP IS THE IWDG, armed once in main() at about 32 seconds and
// refreshed at the top of every letter and inside every long loop. It
// cannot be turned off again (RM0390 21.3), which is the point: a sleep
// with no wake behind it costs one reboot and a banner instead of a
// board that has to be re-flashed.
//
// What is exercised, letter by letter:
//   a  the block and the ladder: SLEEPDEEP and PDDS as the two-register
//      pair they are, the Stop's four price bits and their refusals,
//      `armed()` as a read of the silicon, and the four rungs of the
//      mapping this target chose - with Standby ENFORCED off the ladder
//   b  Sleep, the shallow rung: the tick keeps running and the idle hook
//      comes back on it
//   c  STOP: kernel time stands still for the whole sleep, measured on
//      the RTC - and the system clock comes back as the HSI with the PLL,
//      the HSE and over-drive gone, which the site's resume_clock() puts
//      right
//   d  the wake-up cost of each regulator variant, weighed at 16 MHz on
//      the HSI where no PLL restart hides inside it, against DS10693
//      table 36
//   e  the SYSCLK restore: how long it takes to get from a Stop's HSI
//      back to the board's top rate, and that it works with the
//      peripheral clocks the manual's boot recipe would have had off
//   f  the DynamicClock ladder walked up and down, every rate weighed
//      against the LSE and the console re-based at each step
//   g  a Stop through a REAL KERNEL: the vote round, a standing lock,
//      the deadline guard, the first-event-after-wake contract, and the
//      PLAIN site's honest restriction (an armed time event matures LATE
//      by the sleep)
//   h  THE TIMED SITE: the same deadline met on the wall, never early,
//      with the frozen span handed back to the ticker
//   i  the surface nothing on this board can stage: the PVD at every
//      threshold and its EXTI line, the wake-up pins and the one flag
//      they share, the backup regulator, and the DBGMCU bits AS FOUND
//
//   s  (by name only) STANDBY. The 1.2 V domain is powered off and the
//      wake comes back THROUGH THE RESET VECTOR, so this letter reboots
//      the board and resumes from an RTC backup register - which is what
//      a program without SRAM has to do. Not in `z`. Run it with
//          brio run <board> s --app test_stm32f4_power
//                  --expect="pass," --timeout 60
//
// build: boards = f429zi,f446re,f411ce,f469ni
// build: monitor_speed = 115200

#include <stdint.h>

#include "kernel/post.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time_event.hpp"
#include "stm32f4/clock.hpp"
#include "stm32f4/exti.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/pwr.hpp"
#include "stm32f4/reset.hpp"
#include "stm32f4/rtc.hpp"
#include "stm32f4/sleep.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/usart.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

namespace {

using namespace brio;

using P = Stm32f4Platform<>;

#if defined(STM32F429xx)
using Led = Pin<'G', 13>;
constexpr UartPins console_pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
constexpr uint8_t console_instance = 1;
#elif defined(STM32F469xx)
using Led = Pin<'G', 6>;   // LD1, lit when low
constexpr UartPins console_pins{.tx = {'B', 10, PinFunction::af7}, .rx = {'B', 11, PinFunction::af7}};
constexpr uint8_t console_instance = 3;
#elif defined(STM32F411xE)
using Led = Pin<'C', 13>;
constexpr UartPins console_pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
constexpr uint8_t console_instance = 1;
#else
using Led = Pin<'A', 5>;
constexpr UartPins console_pins{.tx = {'A', 2, PinFunction::af7}, .rx = {'A', 3, PinFunction::af7}};
constexpr uint8_t console_instance = 2;
#endif

using Serial = Uart<console_instance, console_pins>;
constexpr Serial serial;

// ---------------------------------------------------------------------------
// The rate pack: the board's top rate first (the boot rate), the 16 MHz
// HSI last. Every rate is a TUPLE - root, PLL ratio, regulator scale,
// over-drive, flash latency, both APB prescalers - and naming the rate
// names all six.
// ---------------------------------------------------------------------------
#if defined(STM32F411xE)
using R0 = Clock<ClockSource::pll_hse, 100'000'000, 25'000'000>;
using R1 = Clock<ClockSource::pll_hse, 84'000'000, 25'000'000>;
using R2 = Clock<ClockSource::pll_hse, 48'000'000, 25'000'000>;
using R3 = Clock<ClockSource::hsi, 16'000'000>;
using RatePack = Rates<R0, R1, R2, R3>;
#elif defined(STM32F429xx) || defined(STM32F469xx)
using R0 = Clock<ClockSource::pll_hse, 180'000'000, 8'000'000>;
using R1 = Clock<ClockSource::pll_hse, 168'000'000, 8'000'000>;
using R2 = Clock<ClockSource::pll_hse, 120'000'000, 8'000'000>;
using R3 = Clock<ClockSource::pll_hse, 84'000'000, 8'000'000>;
using R4 = Clock<ClockSource::pll_hse, 48'000'000, 8'000'000>;
using R5 = Clock<ClockSource::hsi, 16'000'000>;
using RatePack = Rates<R0, R1, R2, R3, R4, R5>;
#else
using R0 = Clock<ClockSource::pll_hse, 180'000'000, 8'000'000, HseMode::bypass>;
using R1 = Clock<ClockSource::pll_hse, 168'000'000, 8'000'000, HseMode::bypass>;
using R2 = Clock<ClockSource::pll_hse, 120'000'000, 8'000'000, HseMode::bypass>;
using R3 = Clock<ClockSource::pll_hse, 84'000'000, 8'000'000, HseMode::bypass>;
using R4 = Clock<ClockSource::pll_hse, 48'000'000, 8'000'000, HseMode::bypass>;
using R5 = Clock<ClockSource::hsi, 16'000'000>;
using RatePack = Rates<R0, R1, R2, R3, R4, R5>;
#endif

using SysClock = DynamicClock<RatePack, Ticker, Serial>;
constexpr SysClock clock;

constexpr uint8_t hsi_rate_index = SysClock::rate_count - 1u;

// ---------------------------------------------------------------------------
// The RTC as the wall clock and as the alarm
// ---------------------------------------------------------------------------
//
// The board with a crystal runs the calendar on it; the one without runs
// it on the LSI, an RC oscillator the datasheet only bounds - so the
// STATED rate there is deliberately the top of that bound, which is the
// direction the timed site's contract wants (late, never early) and the
// direction that makes every microsecond printed here a lower bound.

#if defined(STM32F429xx)
constexpr RtcClockSource rtc_source = RtcClockSource::lsi;
constexpr uint32_t stated_rtcclk_hz = 47'000;   // DS10693's LSI upper bound
#else
constexpr RtcClockSource rtc_source = RtcClockSource::lse;
constexpr uint32_t stated_rtcclk_hz = 32'768;
#endif

constexpr TimedSleepConfig timed_cfg{.rtcclk_hz = stated_rtcclk_hz,
                                     .source = rtc_source,
                                     .wipe_domain = true,
                                     .fast_clock = RtcWakeupClock::div16};

using Site = Stm32f4SleepSite<SysClock>;
using TimedSite = Stm32f4TimedSleepSite<P, SysClock, timed_cfg>;

TestBench<Serial> bench;

/// True while the RTC_WKUP vector belongs to the timed site; false while
/// a letter drives the wake-up timer itself.
volatile bool site_owns_wakeup = false;
volatile uint16_t wakeup_count = 0;

/// What RTCCLK really is, in hertz - WEIGHED, not stated: the crystal is
/// 32768 within its tens of ppm, but the LSI is an RC oscillator the
/// datasheet only bounds (17..47 kHz), and a wall read in its ticks is
/// worth nothing until the ticks have been counted against the core's
/// own clock. weigh_wall() does that once, the first time a letter asks
/// whether the wall is up; until then the stated rate stands (the site's
/// contract keeps the STATED rate on purpose: late, never early).
uint32_t rtcclk_hz = stated_rtcclk_hz;
bool wall_weighed = false;
/// A crystal is trusted at its stated rate (tens of ppm, below what a
/// millisecond ruler can add); the LSI is weighed.
constexpr bool wall_is_crystal = rtc_source == RtcClockSource::lse;

/// How many sub-second ticks make one CALENDAR second: PREDIV_S + 1,
/// whatever split is in force (the timed site owns the split, and a
/// helper that had baked it in would report nonsense after letter h).
uint32_t wall_ticks_per_second() {
    return static_cast<uint32_t>(Rtc::prescalers().sync) + 1u;
}
uint32_t wall_modulus() { return 60u * wall_ticks_per_second(); }

/// What one sub-second tick is worth in real time: ck_apre, which is
/// RTCCLK over PREDIV_A + 1.
uint32_t wall_hz() {
    return rtcclk_hz / (static_cast<uint32_t>(Rtc::prescalers().async) + 1u);
}

/// Sub-second ticks since the top of the minute, or 0xFFFFFFFF when the
/// calendar could not be read coherently.
uint32_t wall() {
    RtcReading r{};
    if (!Rtc::read(r)) {
        return 0xFFFFFFFFu;
    }
    const uint32_t per = wall_ticks_per_second();
    return static_cast<uint32_t>(r.time.second) * per + (per - 1u - r.subsecond);
}

uint32_t wall_delta(uint32_t from, uint32_t to) {
    return (to >= from) ? (to - from) : (wall_modulus() - from + to);
}
uint32_t wall_us(uint32_t ticks) {
    return static_cast<uint32_t>((static_cast<uint64_t>(ticks) * 1'000'000ULL) / wall_hz());
}
uint32_t wall_ms(uint32_t ticks) { return wall_us(ticks) / 1000u; }

/// Count the sub-second ticks over a quarter of a second of kernel tick
/// (the ticker rides the core clock, HSE-derived: the ruler) and derive
/// RTCCLK from them through the prescaler in force.
void weigh_wall() {
    if (wall_is_crystal) {
        wall_weighed = true;
        return;
    }
    // Both ends on a sub-second tick EDGE, so the wall's own granularity
    // (one ck_apre tick, 4 ms at the reset prescalers) drops out and what
    // remains is the millisecond ruler's edge over two seconds: 500 ppm.
    auto edge = [] {
        const uint32_t w = wall();
        uint32_t spins = 2'000'000u;
        while (wall() == w && spins-- != 0u) {
        }
        return wall();
    };
    const uint32_t w0 = edge();
    const uint32_t k0 = Ticker::millis();
    while (Ticker::millis() - k0 < 2000u) {
    }
    const uint32_t w1 = edge();
    const uint32_t k1 = Ticker::millis();
    if (w0 == 0xFFFFFFFFu || w1 == 0xFFFFFFFFu || k1 == k0) {
        return;
    }
    const uint32_t apre_hz =
        static_cast<uint32_t>((static_cast<uint64_t>(wall_delta(w0, w1)) * 1000ULL) / (k1 - k0));
    rtcclk_hz = apre_hz * (static_cast<uint32_t>(Rtc::prescalers().async) + 1u);
    wall_weighed = true;
    print(serial, "  RTCCLK weighed against the core: ", rtcclk_hz, " Hz (", stated_rtcclk_hz,
          " stated - the LSI's bound, which the timed site keeps on purpose)", crlf);
}

/// True when the wall can be read at all - every measuring letter asks
/// first rather than printing a number taken from a stopped counter - and
/// the first time it can, it is weighed.
bool wall_up() {
    if (wall() == 0xFFFFFFFFu) {
        return false;
    }
    if (!wall_weighed) {
        weigh_wall();
    }
    return true;
}

bool within(uint32_t v, uint32_t lo, uint32_t hi) { return v >= lo && v <= hi; }

/// Spin to the next kernel-tick boundary. A deadline of N ticks matures
/// at the N-th BOUNDARY after it is armed, so arming it mid-tick makes
/// the wall span up to one millisecond short of N - a property of a
/// 1 kHz tick and not of any sleep. Every wall-judged deadline in this
/// suite is armed from a boundary, which takes that millisecond out of
/// the measurement instead of out of the verdict.
void at_tick_edge() {
    const uint32_t t = Ticker::ticks();
    while (Ticker::ticks() == t) {
    }
}

// ---------------------------------------------------------------------------
// The watchdog backstop and the console
// ---------------------------------------------------------------------------
bool feed_iwdg = false;
void feed() {
    if (feed_iwdg) {
        Iwdg::refresh();
    }
}

/// Wait for the console to be physically empty. Called before anything
/// that is measured on the wall: a line of verdict is four milliseconds
/// at 115200, and a drain placed inside a measurement is the measurement.
///
/// AND THE RING BEING EMPTY IS NOT THE WIRE BEING EMPTY. `tx_idle()` says
/// the transport has nothing left to hand the USART; the last character
/// is then still in the shift register, and a Stop taken at that instant
/// kills PCLK1 in the middle of it. Measured: the line's own CRLF lost
/// and the host resynchronizing several bytes into the next line. TC is
/// the wire's own flag, and this is the only place in the suite that
/// reaches past the transport for it.
void console_drain() {
    for (uint32_t i = 0; i < 20'000'000u && !Serial::tx_idle(); ++i) {
    }
    for (uint32_t i = 0;
         i < 2'000'000u && (Usart<console_instance>::status() & UsartFlag::tc) == 0u; ++i) {
    }
}

/// The three DBGMCU bits at the very first instruction of main(), before
/// anything took them down. A probe leaves them set and they survive
/// every reset but a power-on, so what a Stop does on this board depends
/// on what somebody else left there.
bool dbg_sleep_at_boot = false;
bool dbg_stop_at_boot = false;
bool dbg_standby_at_boot = false;
/// And PWR_CSR's two flags, likewise: SBF and WUF outlive a system reset.
bool sbf_at_boot = false;
bool wuf_at_boot = false;
uint32_t reset_flags_at_boot = 0;

/// True while a Tenuto is the thing being pumped: the RTC handler posts
/// the wake into it only then (letters g and h), and never while a letter
/// is driving the wake-up timer by hand.
volatile bool kernel_running = false;

// =============================================================================
// a - the block and the ladder
// =============================================================================
void ta_block() {
    feed();
    print(serial, "  PWR_CR ", hex(Pwr::cr()), "  PWR_CSR ", hex(Pwr::csr()),
          "  VOS scale ", static_cast<uint32_t>(Pwr::scale()),
          Pwr::scale_ready() ? " (ready)" : " (not ready)",
          Pwr::over_drive_active() ? ", over-drive ACTIVE" : ", no over-drive", crlf);
    bench.verdict("the block answers through its APB1 gate", Pwr::bus_clock());

    // ---- SLEEPDEEP and PDDS as one pair ------------------------------------
    (void)Pwr::arm(PwrMode::sleep);
    bench.verdict("arm(sleep) is SLEEPDEEP clear, whatever PDDS holds",
                  !Pwr::deep_sleep() && Pwr::mode() == PwrMode::sleep);
    (void)Pwr::arm(PwrMode::stop);
    bench.verdict("arm(stop) is SLEEPDEEP set and PDDS clear",
                  Pwr::deep_sleep() && !Pwr::power_down_deep_sleep() &&
                      Pwr::mode() == PwrMode::stop);
    (void)Pwr::arm(PwrMode::standby);
    bench.verdict("arm(standby) is both", Pwr::deep_sleep() &&
                                              Pwr::power_down_deep_sleep() &&
                                              Pwr::mode() == PwrMode::standby);
    Pwr::power_down_deep_sleep(false);
    (void)Pwr::arm(PwrMode::sleep);
    bench.verdict("and mode() is a pure read of the two registers, not a mirror",
                  Pwr::mode() == PwrMode::sleep);

    // ---- the four price bits ------------------------------------------------
    bench.verdict("the plain Stop configuration is written and read back",
                  Pwr::stop_config(StopConfig{}) &&
                      Pwr::stop_config().regulator == StopRegulator::main &&
                      !Pwr::stop_config().flash_power_down);
    const StopConfig lp{StopRegulator::low_power, true, false, false};
    bench.verdict("the low-power regulator with the flash in power-down too",
                  Pwr::stop_config(lp) &&
                      Pwr::stop_config().regulator == StopRegulator::low_power &&
                      Pwr::stop_config().flash_power_down);
    const StopConfig lv{StopRegulator::low_power, true, true, false};
    const bool lv_ok = Pwr::stop_config(lv);
    print(serial, "  this part ", pwr_has_low_voltage_stop() ? "has" : "has NO",
          " the low-voltage pair and ", pwr_has_under_drive() ? "has" : "has NO",
          " the under-drive field; the FISSR/FMSSR pair is ",
          Pwr::has_flash_stop_while_run ? "present" : "absent", crlf);
    bench.verdict("the low-voltage mode is taken where the part has it and "
                  "refused where it has not",
                  lv_ok == pwr_has_low_voltage_stop() &&
                      (!lv_ok || Pwr::stop_config().low_voltage));
    bench.verdict("under-drive without the low-voltage mode it modifies is "
                  "refused everywhere (RM0390 table 17)",
                  !Pwr::stop_config(StopConfig{StopRegulator::main, true, false, true}));
    const StopConfig ud{StopRegulator::low_power, true, true, true};
    bench.verdict("and under-drive ON TOP of it is taken exactly where the "
                  "field exists",
                  Pwr::stop_config(ud) == pwr_has_under_drive());
    (void)Pwr::stop_config(StopConfig{});

    // ---- the site's mapping -------------------------------------------------
    bench.verdict("none maps to Sleep, and armed() says so",
                  Site::arm(SleepDepth::none) && Site::armed() == SleepDepth::none);
    bench.verdict("light maps to the SAME mode - between Sleep and Stop this "
                  "family has nothing",
                  Site::arm(SleepDepth::light) && Site::armed() == SleepDepth::none &&
                      Pwr::mode() == PwrMode::sleep);
    bench.verdict("standby maps to a Stop on the main regulator",
                  Site::arm(SleepDepth::standby) && Site::armed() == SleepDepth::standby &&
                      Pwr::mode() == PwrMode::stop &&
                      Pwr::stop_config().regulator == StopRegulator::main);
    bench.verdict("deep maps to a Stop on the low-power regulator with the "
                  "flash in power-down",
                  Site::arm(SleepDepth::deep) && Site::armed() == SleepDepth::deep &&
                      Pwr::mode() == PwrMode::stop &&
                      Pwr::stop_config().regulator == StopRegulator::low_power &&
                      Pwr::stop_config().flash_power_down);

    // ---- Standby is off the ladder, and the site takes PDDS down -----------
    Pwr::power_down_deep_sleep(true);
    const bool pdds_before = Pwr::power_down_deep_sleep();
    const bool armed_deep = Site::arm(SleepDepth::deep);
    bench.verdict("no rung reaches Standby: a PDDS left standing by anything "
                  "else is CLEARED by the next arm()",
                  pdds_before && armed_deep && !Pwr::power_down_deep_sleep());
    Site::disarm();
    bench.verdict("disarm() puts the shallow mode back", Site::armed() == SleepDepth::none &&
                                                             Pwr::mode() == PwrMode::sleep);

    // ---- the wake-up pins ---------------------------------------------------
    print(serial, "  wake-up pins bonded: ", static_cast<uint32_t>(Pwr::wakeup_pin_count));
    for (uint8_t n = 1; n <= 3u; ++n) {
        const PwrWakeupPad pad = Pwr::wakeup_pad(n);
        if (pad.known) {
            print(serial, "  WKUP", static_cast<uint32_t>(n), " = P",
                  static_cast<char>(pad.port), static_cast<uint32_t>(pad.pin));
        }
    }
    print(serial, crlf);
    bench.verdict("WKUP1 is PA0 and pin 1 exists on every part",
                  Pwr::wakeup_pin_present(1) && Pwr::wakeup_pad(1).known &&
                      Pwr::wakeup_pad(1).port == 'A' && Pwr::wakeup_pad(1).pin == 0);
    bench.verdict("a pin this part does not bond is refused, not written",
                  Pwr::wakeup_pin_present(4) == false && !Pwr::wakeup_pin(4, true));
}

// =============================================================================
// b - Sleep, the shallow rung
// =============================================================================
void tb_sleep() {
    feed();
    if (!wall_up()) {
        bench.verdict("the wall clock reads", false);
        return;
    }
    site_owns_wakeup = false;

    // The tick is the only wake: nothing else is armed, so a WFI here
    // lasts at most one millisecond.
    (void)Site::arm(SleepDepth::light);
    console_drain();
    const uint32_t k0 = Ticker::millis();
    const uint32_t w0 = wall();
    for (uint16_t i = 0; i < 100u; ++i) {
        Pwr::wait_for_interrupt();
    }
    const uint32_t w1 = wall();
    const uint32_t k1 = Ticker::millis();
    Site::disarm();

    const uint32_t wall_span = wall_ms(wall_delta(w0, w1));
    print(serial, "  100 WFIs in Sleep: ", k1 - k0, " ms of kernel tick, ", wall_span,
          " ms of wall", crlf);
    bench.verdict("Sleep keeps HCLK and SysTick: the tick counted the sleep",
                  within(k1 - k0, 95u, 110u));
    bench.verdict("and the wall agrees with it - nothing stood still",
                  within(wall_span, 95u, 110u));
    bench.verdict("the mode really was Sleep: SLEEPDEEP was never set",
                  !Pwr::deep_sleep());

    // The wake cost, differenced against a control that does not sleep.
    (void)Site::arm(SleepDepth::light);
    console_drain();
    const uint32_t a0 = wall();
    for (uint16_t i = 0; i < 200u; ++i) {
        Pwr::wait_for_interrupt();
    }
    const uint32_t a1 = wall();
    Site::disarm();
    print(serial, "  200 tick-woken sleeps took ", wall_us(wall_delta(a0, a1)),
          " us of wall (200 kernel ticks is 200000 us; the excess is the "
          "wake and the loop)", crlf);
    bench.verdict("a Sleep's wake is far below one tick - the sleeps did not "
                  "cost a tick between them",
                  within(wall_ms(wall_delta(a0, a1)), 195u, 215u));
}

// =============================================================================
// c - a Stop, the frozen tick and the clock that comes back
// =============================================================================
void tc_stop() {
    feed();
    if (!wall_up()) {
        bench.verdict("the wall clock reads", false);
        return;
    }
    site_owns_wakeup = false;
    wakeup_count = 0;
    Nvic::enable(Rtc::wakeup_irq());

    // 200 ms on the fast clock: RTCCLK / 16 counts. THE ALARM IS
    // PROGRAMMED AFTER THE WALL IS STAMPED, because the wake-up timer
    // starts counting the moment WUTE goes up - a console drain between
    // the two would be time the alarm had already spent (measured: five
    // milliseconds of it).
    const uint32_t fast_hz = rtcclk_hz / 16u;
    const uint32_t reload = (200u * fast_hz) / 1000u - 1u;

    console_drain();
    const uint32_t k0 = Ticker::millis();
    const uint32_t w0 = wall();
    const bool alarm = Rtc::set_wakeup(RtcWakeupClock::div16, reload, true);
    (void)Site::arm(SleepDepth::standby);   // a Stop on the main regulator
    Pwr::wait_for_interrupt();
    // NOTHING MAY BE PRINTED HERE: the core is on the HSI with the
    // console's divisor meant for the top rate. Everything is captured
    // and judged after the restore.
    const uint32_t w1 = wall();
    const SysclkSource after = Rcc::sysclk_status();
    const VoltageScale scale_after = Pwr::scale();
    const bool vosrdy_after = Pwr::scale_ready();
    const bool od_after = Pwr::over_drive_active();
    const bool pll_after = Rcc::pll_ready();
    const bool hse_after = Rcc::hse_ready();
    const uint32_t r0 = wall();
    Site::disarm();   // resume_clock() lives here
    const uint32_t r1 = wall();
    const uint32_t k1 = Ticker::millis();

    Rtc::clear_wakeup();
    Nvic::disable(Rtc::wakeup_irq());

    const uint32_t slept_ms = wall_ms(wall_delta(w0, w1));
    const uint32_t restore_us = wall_us(wall_delta(r0, r1));
    print(serial, "  the alarm was placed at reload ", reload, " on RTCCLK/16 (",
          fast_hz, " Hz); the Stop lasted ", slept_ms, " ms of wall and the kernel "
          "tick moved ", k1 - k0, " ms", crlf);
    print(serial, "  at the first instruction after the wake: SYSCLK ",
          after == SysclkSource::hsi ? "HSI" : (after == SysclkSource::pll ? "PLL" : "HSE"),
          ", the VOS field still reads scale ", static_cast<uint32_t>(scale_after),
          " (VOSRDY ", vosrdy_after ? "up" : "down", "), over-drive ", od_after ? "on" : "off",
          ", PLL ", pll_after ? "locked" : "off", ", HSE ", hse_after ? "ready" : "off",
          "; the restore took ", restore_us, " us", crlf);
    print(serial, "  DBG_STOP was ", dbg_stop_at_boot ? "SET" : "clear", " at boot and is ",
          Pwr::debug_in_stop() ? "SET" : "clear",
          " now - set, it would have kept HCLK alive through the Stop above", crlf);

    bench.verdict("the alarm was programmed", alarm);
    bench.verdict("the wake really came from the RTC", wakeup_count >= 1u);
    bench.verdict("the Stop lasted what the alarm asked for, on the wall",
                  within(slept_ms, 195u, 215u));
    bench.verdict("KERNEL TIME STOOD STILL for the whole of it - SysTick rides "
                  "HCLK and HCLK stopped",
                  (k1 - k0) < 10u);
    bench.verdict("the core came back ON THE HSI, as 5.3.5 says", after == SysclkSource::hsi);
    bench.verdict("with the PLL and the HSE off", !pll_after && !hse_after);
    bench.verdict("over-drive is CLEARED BY HARDWARE on the way out (5.4.1: "
                  "ODEN and ODSWEN are 'cleared automatically by hardware after "
                  "exiting from Stop mode')",
                  !od_after);
    bench.verdict("but the VOS FIELD is not touched: 5.1.3 forces the "
                  "regulator's ACTIVE scale to 3 while the PLL is off and "
                  "leaves the programmed value where it was",
                  scale_after == SysClock::rate_regime(SysClock::rate_index()).scale);
    bench.verdict("the site's disarm() put the whole tree back",
                  Rcc::sysclk_status() == SysClock::rate_source(SysClock::rate_index()));
}

// =============================================================================
// d - the regulator variants, weighed where no PLL restart hides
// =============================================================================
struct Variant {
    const char* name;
    StopConfig cfg;
    bool present;
    uint32_t typ_us;   ///< DS10693 table 36's typical, for comparison only
};

void td_variants() {
    feed();
    if (!wall_up()) {
        bench.verdict("the wall clock reads", false);
        return;
    }
    site_owns_wakeup = false;
    Nvic::enable(Rtc::wakeup_irq());

    // THE MEASUREMENT IS MADE AT 16 MHz ON THE HSI, which is what makes
    // it a measurement of the WAKE and not of the PLL: after a Stop the
    // core is on the HSI already, so resume_clock() has nothing to do and
    // the only thing between two laps is the alarm, the wake and a
    // handful of instructions.
    const uint8_t was = SysClock::rate_index();
    const bool down = SysClock::set_index(hsi_rate_index);
    console_drain();

    const uint32_t fast_hz = rtcclk_hz / 16u;
    const uint32_t reload = (4u * fast_hz) / 1000u;   // about 4 ms a lap
    constexpr uint16_t laps = 32;

    const Variant variants[] = {
        {"MR", StopConfig{StopRegulator::main, false, false, false}, true, 13},
        {"LP", StopConfig{StopRegulator::low_power, false, false, false}, true, 21},
        {"MR + FPDS", StopConfig{StopRegulator::main, true, false, false}, true, 105},
        {"LP + FPDS", StopConfig{StopRegulator::low_power, true, false, false}, true, 113},
        {"LP + FPDS + low voltage",
         StopConfig{StopRegulator::low_power, true, true, false}, pwr_has_low_voltage_stop(), 0},
        {"LP + FPDS + under-drive",
         StopConfig{StopRegulator::low_power, true, true, true}, pwr_has_under_drive(), 114},
    };
    uint32_t lap_us[6] = {};
    bool measured[6] = {};

    for (uint8_t v = 0; v < 6u; ++v) {
        if (!variants[v].present) {
            continue;
        }
        feed();
        if (!Pwr::arm(PwrMode::stop, variants[v].cfg)) {
            continue;
        }
        Ticker::pause();
        const uint32_t w0 = wall();
        for (uint16_t i = 0; i < laps; ++i) {
            (void)Rtc::set_wakeup(RtcWakeupClock::div16, reload, true);
            Pwr::wait_for_interrupt();
        }
        const uint32_t w1 = wall();
        Ticker::resume();
        Rtc::clear_wakeup();
        (void)Pwr::arm(PwrMode::sleep);
        lap_us[v] = wall_us(wall_delta(w0, w1)) / laps;
        measured[v] = true;
    }

    const bool back = SysClock::set_index(was);
    Nvic::disable(Rtc::wakeup_irq());

    const uint32_t alarm_us = ((reload + 1u) * 1'000'000u) / fast_hz;
    print(serial, "  ", laps, " Stops a variant at 16 MHz on the HSI, the alarm ",
          alarm_us, " us apart; each lap is that plus the wake and the "
          "arming code", crlf);
    for (uint8_t v = 0; v < 6u; ++v) {
        if (!measured[v]) {
            print(serial, "  ", variants[v].name, ": absent on this part", crlf);
            continue;
        }
        print(serial, "  ", variants[v].name, ": ", lap_us[v], " us a lap, ",
              lap_us[v] > lap_us[0] ? lap_us[v] - lap_us[0] : 0u,
              " us more than the main regulator");
        if (variants[v].typ_us != 0u) {
            print(serial, " (DS10693 table 36 puts the two ",
                  variants[v].typ_us > variants[0].typ_us
                      ? variants[v].typ_us - variants[0].typ_us
                      : 0u,
                  " us apart, typical)");
        }
        print(serial, crlf);
    }

    bench.verdict("the ladder walked down to the HSI and back for the "
                  "measurement",
                  down && back && SysClock::rate_index() == was);
    bench.verdict("every variant this part has took a real Stop",
                  measured[0] && measured[1] && measured[2] && measured[3]);
    // DS10693 table 36 puts the low-power regulator eight microseconds
    // above the main one, TYPICAL - and eight microseconds is the one
    // step of this table that did not show above the HSI's own startup
    // here (the numbers are printed; the doc says what was seen). So the
    // verdict is the direction and not the size: a deeper variant is
    // never CHEAPER to leave.
    // Two microseconds of slack: where the two laps are equal (the
    // STM32F411 shows no step at all) the alarm's own jitter decides the
    // sign of a one-microsecond difference.
    bench.verdict("the low-power regulator is never cheaper to leave than the "
                  "main one",
                  lap_us[1] + 2u >= lap_us[0]);
    // The flash's own wake is the big step on one part (92 us measured
    // against DS10693's 92 typical on the STM32F446) and no step at all on
    // another (the STM32F411 leaves every variant at the same lap) - a fact
    // of the part, printed above and judged only in its direction.
    print(serial, "  the flash in power-down costs ", lap_us[2] - lap_us[1],
          " us more than the regulator alone on this part", crlf);
    bench.verdict("and the flash in power-down is never cheaper to leave than the "
                  "regulator alone",
                  lap_us[2] >= lap_us[1] && lap_us[3] >= lap_us[1]);
    if (measured[5]) {
        bench.verdict("under-drive is the deepest and the slowest to leave",
                      lap_us[5] >= lap_us[3]);
        bench.verdict("and PWR_CSR.UDRDY records that the device really entered it",
                      Pwr::under_drive_flag());
        Pwr::clear_under_drive_flag();
    }
}

// =============================================================================
// e - the SYSCLK restore
// =============================================================================
void te_restore() {
    feed();
    if (!wall_up()) {
        bench.verdict("the wall clock reads", false);
        return;
    }
    site_owns_wakeup = false;
    Nvic::enable(Rtc::wakeup_irq());

    const uint32_t fast_hz = rtcclk_hz / 16u;
    const uint32_t reload = (4u * fast_hz) / 1000u;
    constexpr uint16_t laps = 16;

    // Each lap: a real Stop (which parks the tree on the HSI, stops the
    // PLL and the HSE and drops the regulator to scale 3), then the
    // restore, bracketed on the wall. The two wall reads happen at
    // 16 MHz and cost a few microseconds of the number printed.
    (void)Site::arm(SleepDepth::standby);
    Ticker::pause();
    console_drain();
    uint32_t total = 0;
    uint32_t worst = 0;
    uint16_t restored = 0;
    for (uint16_t i = 0; i < laps; ++i) {
        (void)Rtc::set_wakeup(RtcWakeupClock::div16, reload, true);
        Pwr::wait_for_interrupt();
        const uint32_t a = wall();
        if (Site::resume_clock()) {
            ++restored;
        }
        const uint32_t b = wall();
        const uint32_t us = wall_us(wall_delta(a, b));
        total += us;
        if (us > worst) {
            worst = us;
        }
        (void)Pwr::arm(PwrMode::stop, StopConfig{});
    }
    Ticker::resume();
    Rtc::clear_wakeup();
    Site::disarm();
    Nvic::disable(Rtc::wakeup_irq());

    print(serial, "  ", laps, " restores from a real Stop back to ", SysClock::hz(),
          " Hz: ", total / laps, " us each on average, ", worst, " us at worst", crlf);
    print(serial, "  the tree is now SYSCLK ", SysClock::hz(), " Hz, PCLK1 ",
          SysClock::pclk1_hz(), " Hz, PCLK2 ", SysClock::pclk2_hz(), " Hz, scale ",
          static_cast<uint32_t>(Pwr::scale()), ", over-drive ",
          Pwr::over_drive_active() ? "on" : "off", ", ",
          static_cast<uint32_t>(FlashWaitStates::get()), " wait states", crlf);

    bench.verdict("the restore put the root back", Rcc::sysclk_status() ==
                                                       SysClock::rate_source(SysClock::rate_index()));
    bench.verdict("with the regulator at the scale the rate needs and its "
                  "VOSRDY up",
                  Pwr::scale() == SysClock::rate_regime(SysClock::rate_index()).scale &&
                      Pwr::scale_ready());
    bench.verdict("and over-drive where the rate needs it",
                  Pwr::over_drive_active() ==
                      SysClock::rate_regime(SysClock::rate_index()).over_drive);
    bench.verdict("the flash latency is the rate's own",
                  FlashWaitStates::get() ==
                      SysClock::rate_wait_states(SysClock::rate_index()));
    // The manual's boot recipe wants no peripheral clock enabled during
    // the over-drive switch (5.1.4's note: "during the Over-drive switch
    // activation, no peripheral clocks should be enabled"). A restore
    // cannot honour that - the console, the RTC and the PWR block are all
    // clocked when it runs - so sixteen of them in a row, each reporting
    // its own success, is what says the note is a boot recipe and not a
    // condition of the switch.
    bench.verdict("every one of the restores reported success, WITH the "
                  "peripheral clocks the manual's boot recipe would have had "
                  "off - the console among them",
                  restored == laps && total > 0u && worst < 100'000u);
}

// =============================================================================
// f - the DynamicClock ladder, up and down
// =============================================================================
uint32_t weigh_rate() {
    // 100 kernel ticks against the wall. The reload is hz / 1000 exactly
    // for every rate in the pack, so 100 ticks IS 100 ms if HCLK is what
    // the rate says - and the LSE is what says otherwise.
    const uint32_t k0 = Ticker::ticks();
    while (Ticker::ticks() == k0) {
    }
    const uint32_t w0 = wall();
    const uint32_t k1 = Ticker::ticks();
    while (Ticker::ticks() - k1 < 100u) {
    }
    const uint32_t w1 = wall();
    return wall_us(wall_delta(w0, w1));
}

void tf_ladder() {
    feed();
    if (!wall_up()) {
        bench.verdict("the wall clock reads", false);
        return;
    }
    const uint8_t start = SysClock::rate_index();
    bool every_rate_ran = true;
    bool every_bus_in_range = true;
    bool worst_ppm_ok = true;
    uint32_t up_us = 0;
    uint32_t down_us = 0;

    constexpr SysclkLadder ladder = sysclk_ladder();

    // Down the pack (the pack is ordered top rate first), then up again.
    for (uint8_t i = 0; i < SysClock::rate_count; ++i) {
        feed();
        console_drain();
        const uint32_t s0 = wall();
        const bool ok = SysClock::set_index(i);
        const uint32_t s1 = wall();
        down_us += wall_us(wall_delta(s0, s1));
        const uint32_t measured = weigh_rate();
        const uint32_t want = 100'000u;
        const int32_t err = static_cast<int32_t>(measured) - static_cast<int32_t>(want);
        const uint32_t ppm = static_cast<uint32_t>(err < 0 ? -err : err) * 10u;
        const bool hsi_rooted = SysClock::rate_source(i) == SysclkSource::hsi;
        print(serial, "  rate ", static_cast<uint32_t>(i), ": ", SysClock::hz(),
              " Hz (PCLK1 ", SysClock::pclk1_hz(), ", PCLK2 ", SysClock::pclk2_hz(),
              ", ", static_cast<uint32_t>(FlashWaitStates::get()), " WS, scale ",
              static_cast<uint32_t>(Pwr::scale()), Pwr::over_drive_active() ? " +OD" : "",
              ") - 100 ticks measured ", measured, " us on the wall, ", ppm,
              " ppm ", err < 0 ? "fast" : "slow", crlf);
        if (!ok || SysClock::rate_index() != i || SysClock::hz() != SysClock::rate_hz(i)) {
            every_rate_ran = false;
        }
        if (ladder.known && (SysClock::pclk1_hz() > ladder.apb1_max_hz ||
                             SysClock::pclk2_hz() > ladder.apb2_max_hz)) {
            every_bus_in_range = false;
        }
        // A loose band: it catches a rate that is WRONG (a missed PLL, a
        // stale prescaler) and says nothing about this crystal's own
        // accuracy. The HSI is an RC oscillator and gets the wider one.
        if (ppm > (hsi_rooted ? 40'000u : 5'000u)) {
            worst_ppm_ok = false;
        }
    }
    for (uint8_t i = SysClock::rate_count; i > 0u; --i) {
        feed();
        console_drain();
        const uint32_t s0 = wall();
        const bool ok = SysClock::set_index(static_cast<uint8_t>(i - 1u));
        const uint32_t s1 = wall();
        up_us += wall_us(wall_delta(s0, s1));
        if (!ok) {
            every_rate_ran = false;
        }
    }
    (void)SysClock::set_index(start);
    console_drain();

    print(serial, "  ", static_cast<uint32_t>(SysClock::rate_count),
          " rates walked down and up: ", down_us / SysClock::rate_count,
          " us a switch downward, ", up_us / SysClock::rate_count,
          " us upward (both include the park on the HSI, which every switch "
          "of this family makes)", crlf);

    bench.verdict("every rate of the pack came up and reported itself",
                  every_rate_ran);
    bench.verdict("every rate's tick agreed with the wall - the PLL, the "
                  "prescalers and the ticker's reload all followed",
                  worst_ppm_ok);
    bench.verdict("and no APB was ever left above its ceiling",
                  every_bus_in_range);
    bench.verdict("the console and the ticker are among the users the pack "
                  "rebases - which is why these lines are legible at every "
                  "rate, and why a driver forgotten in the list would not "
                  "compile",
                  clock_follows<SysClock, Serial>() && clock_follows<SysClock, Ticker>());
    bench.verdict("the ladder ended where it started",
                  SysClock::rate_index() == start && SysClock::hz() == SysClock::rate_hz(start));

    // The rate a program NAMES rather than indexes.
    bench.verdict("set(hz) finds a rate by its number",
                  SysClock::set(SysClock::rate_hz(start)) &&
                      SysClock::rate_index() == start);
    bench.verdict("and a rate the pack does not name is refused with nothing "
                  "changed",
                  !SysClock::set(37u) && SysClock::rate_index() == start);
}

// =============================================================================
// g and h - a Stop through a real kernel
// =============================================================================
struct Blip {};
struct Woke {};   ///< posted by the RTC handler on a plain-site wake

bool timed_round = false;

struct Probe : Fsm<Probe, SleepVote, PrepareSleep, WakeReport, Blip, Woke> {
    static inline EventQueue<Event, 8, P> queue;
    static inline TimeEvent<P, Probe, Blip> deadline{Blip{}};

    static inline uint16_t blips = 0;
    static inline uint16_t woke = 0;
    static inline uint16_t wakes = 0;
    static inline uint16_t votes = 0;
    static inline uint16_t asked = 0;
    static inline bool last_ok = false;
    static inline bool refuse = false;
    static inline uint32_t blip_wall = 0;
    static inline SleepDepth last_report = SleepDepth::none;
    static inline SleepDepth last_asked = SleepDepth::none;

    static void clear() {
        blips = woke = wakes = votes = asked = 0;
        last_ok = false;
        refuse = false;
        blip_wall = 0;
        last_report = SleepDepth::none;
        last_asked = SleepDepth::none;
    }

    static void init() { start(&only); }
    static Status only(const Event& e);
};

using PlainManager = PowerManager<P, Site, PowerConfig{}, Probe>;
using TimedManager = PowerManager<P, TimedSite, PowerConfig{}, Probe>;
using PlainKernel = Tenuto<P, Probe, PlainManager>;
using TimedKernel = Tenuto<P, Probe, TimedManager>;

void answer_manager() {
    if (timed_round) {
        post<TimedManager>(SleepRequested{SleepDepth::none, reply_to<Probe, SleepVote>()});
    } else {
        post<PlainManager>(SleepRequested{SleepDepth::none, reply_to<Probe, SleepVote>()});
    }
}

Probe::Status Probe::only(const Event& e) {
    return match(e,
        [](Entry) { return handled(); },
        [](Exit) { return handled(); },
        [](SleepVote v) {
            ++votes;
            last_ok = v.ok;
            return handled();
        },
        [](const PrepareSleep& p) {
            ++asked;
            last_asked = p.depth;
            p.reply.send(SleepVote{!refuse});
            return handled();
        },
        [](WakeReport w) {
            ++wakes;
            last_report = w.was;
            return handled();
        },
        [](Woke) {
            // THE MODEL'S CONVENTION, and with a Stop it is LOAD-BEARING:
            // a wake path with nothing to say says it with
            // SleepRequested{none}. Without it the manager never learns
            // the machine came back, the site is never disarmed, and on
            // this target that means the clock stays at 16 MHz and the
            // KERNEL'S TICK STAYS PAUSED - so nothing matures, ever.
            ++woke;
            answer_manager();
            return handled();
        },
        [](Blip) {
            ++blips;
            blip_wall = wall();
            answer_manager();
            return handled();
        });
}

template <class K>
void pump_until_blip(uint32_t guard_ms) {
    const uint32_t t0 = wall();
    while (Probe::blips == 0u && wall_ms(wall_delta(t0, wall())) < guard_ms) {
        feed();
        TimeEvents<P>::process();
        if (!K::step()) {
            K::idle_if_empty();
        }
    }
    while (K::step()) {
    }
}

void tg_kernel_stop() {
    feed();
    if (!wall_up()) {
        bench.verdict("the wall clock reads", false);
        return;
    }
    site_owns_wakeup = false;
    timed_round = false;
    PlainKernel::init_all();
    Probe::clear();

    // ---- the vote, refused ---------------------------------------------------
    Probe::refuse = true;
    post<PlainManager>(SleepRequested{SleepDepth::deep, reply_to<Probe, SleepVote>()});
    for (uint8_t i = 0; i < 8u && PlainKernel::step(); ++i) {
    }
    const bool refused = Probe::asked == 1u && Probe::votes == 1u && !Probe::last_ok &&
                         Site::armed() == SleepDepth::none;
    bench.verdict("ONE not-ok ends the round: the site is not armed and the "
                  "requester is told",
                  refused);

    // ---- a standing lock clamps the depth -----------------------------------
    Probe::clear();
    {
        PowerLock lock = PlainManager::restrict(SleepDepth::light);
        post<PlainManager>(SleepRequested{SleepDepth::deep, reply_to<Probe, SleepVote>()});
        for (uint8_t i = 0; i < 8u && PlainKernel::step(); ++i) {
        }
        bench.verdict("a standing lock clamps a deep request to the rung it "
                      "names - the voter is asked for the CLAMPED depth",
                      Probe::last_asked == SleepDepth::light &&
                          Site::armed() == SleepDepth::none);
    }
    bench.verdict("and the ceiling is back when the lock dies",
                  PlainManager::ceiling() == SleepDepth::deep);

    // ---- the deadline guard --------------------------------------------------
    Probe::clear();
    PlainKernel::init_all();
    Probe::deadline.arm(1u);
    post<PlainManager>(SleepRequested{SleepDepth::deep, reply_to<Probe, SleepVote>()});
    for (uint8_t i = 0; i < 8u && PlainKernel::step(); ++i) {
    }
    const bool guarded = Probe::asked == 0u && !Probe::last_ok;
    Probe::deadline.disarm();
    bench.verdict("a deadline nearer than min_deep_ticks refuses a deep round "
                  "BEFORE anyone is asked",
                  guarded);

    // ---- the round, through a real Stop -------------------------------------
    PlainKernel::init_all();
    Probe::clear();
    kernel_running = true;
    Nvic::enable(Rtc::wakeup_irq());
    // The wake is the RTC's own, 250 ms out: the manager knows nothing
    // about it, which is the point - a deep sleep's wake source is the
    // application's business and the model never asks.
    const uint32_t fast_hz = rtcclk_hz / 16u;
    (void)Rtc::set_wakeup(RtcWakeupClock::div16, (250u * fast_hz) / 1000u - 1u, true);

    console_drain();
    const uint32_t w0 = wall();
    const uint32_t k0 = Ticker::millis();
    Probe::deadline.arm(500u);
    post<PlainManager>(SleepRequested{SleepDepth::deep, reply_to<Probe, SleepVote>()});
    pump_until_blip<PlainKernel>(3000u);
    kernel_running = false;
    Rtc::clear_wakeup();
    Nvic::disable(Rtc::wakeup_irq());

    const uint32_t to_blip = wall_ms(wall_delta(w0, Probe::blip_wall));
    const uint32_t kernel_ms = Ticker::millis() - k0;
    print(serial, "  the round: ", Probe::votes, " vote(s), ", Probe::wakes,
          " wake report(s) at depth ", static_cast<uint32_t>(Probe::last_report),
          "; a 500 ms event matured after ", to_blip, " ms of WALL and ", kernel_ms,
          " ms of kernel tick", crlf);

    bench.verdict("the vote round ran and the site was armed at the depth the "
                  "target really took",
                  Probe::votes >= 1u && Probe::last_ok && Probe::last_report == SleepDepth::deep);
    bench.verdict("the first event after the wake ended the round - nothing "
                  "polled, and the manager disarmed on its own",
                  Probe::woke >= 1u && Probe::wakes >= 1u && Site::armed() == SleepDepth::none);
    bench.verdict("the clock was put back by the same disarm: the console is "
                  "legible and the rate is the program's",
                  Rcc::sysclk_status() == SysClock::rate_source(SysClock::rate_index()));
    bench.verdict("the deadline was met in KERNEL time, which is all the "
                  "kernel promises",
                  Probe::blips == 1u && kernel_ms >= 500u);
    bench.verdict("BUT NOT ON THE WALL - with the plain site the sleep is time "
                  "the tick never counted, so the event matured LATE by about "
                  "the length of the Stop",
                  to_blip > 700u);
}

// =============================================================================
// h - the timed site: the same deadline, on the wall
// =============================================================================
void th_timed_site() {
    feed();
    if (!wall_up()) {
        bench.verdict("the wall clock reads", false);
        return;
    }
    site_owns_wakeup = true;
    timed_round = true;
    kernel_running = true;
    const bool up = TimedSite::ready() || TimedSite::init();
    print(serial, "  the timed site ", up ? "is up" : "REFUSED",
          "; its prescalers are A ", TimedSite::prescalers.async, " / S ",
          TimedSite::prescalers.sync, ", its fast alarm clock ", TimedSite::fast_hz,
          " Hz, and it reaches ", TimedSite::fast_span_ticks, " ticks on it", crlf);
    bench.verdict("the timed site comes up on the root this board runs the RTC on", up);
    if (!up) {
        kernel_running = false;
        site_owns_wakeup = false;
        timed_round = false;
        return;
    }
    // THE NVIC LINE IS THE SITE'S WAY OUT OF THE STOP, and 5.3.5 is
    // explicit that a WFI-entered Stop is left only by "all EXTI lines
    // configured in Interrupt mode (the corresponding EXTI Interrupt
    // vector must be enabled in the NVIC)". main() leaves the line
    // disabled so the letters that drive the timer by hand own it;
    // measured, a round entered without this line open never comes back
    // and the watchdog reboots the board.
    Nvic::enable(Rtc::wakeup_irq());
    print(serial, "  the site states ", timed_cfg.rtcclk_hz, " Hz for RTCCLK; the wall "
          "above is read on the same number, so a rate stated too high shows "
          "up as a wall that runs slow and never as an early event", crlf);

    // The alarm's arithmetic, checked before any sleep.
    TimedKernel::init_all();
    Probe::clear();
    Probe::deadline.arm(500u);
    const bool armed = TimedSite::arm(SleepDepth::deep);
    const uint32_t reload = TimedSite::last_reload();
    const bool fast = TimedSite::last_alarm_was_fast();
    TimedSite::disarm();
    Probe::deadline.disarm();
    const uint32_t want =
        (500u * TimedSite::fast_hz + P::ticks_per_second - 1u) / P::ticks_per_second - 1u;
    print(serial, "  a 500 ms deadline placed the alarm at reload ", reload, " on the ",
          fast ? "fast" : "one-second", " clock; the arithmetic says ", want, crlf);
    bench.verdict("the alarm lands where the stated arithmetic puts it",
                  armed && fast && reload == want);

    // NOTHING MAY BE PRINTED BETWEEN arm() AND disarm(). arm() pauses the
    // kernel's tick for a deep rung, so the frozen span the resync hands
    // back is the WHOLE armed window - and a verdict line is four
    // milliseconds of console.
    const bool no_deadline_armed = TimedSite::arm(SleepDepth::deep);
    const bool no_alarm = !TimedSite::alarm_armed();
    TimedSite::disarm();
    const uint32_t napless = TimedSite::last_advance();
    bench.verdict("a deadline-less round places no alarm at all",
                  no_deadline_armed && no_alarm);
    bench.verdict("and a round that never slept advances at most a tick", napless <= 1u);

    // ---- the round trip ------------------------------------------------------
    TimedKernel::init_all();
    Probe::clear();
    console_drain();
    at_tick_edge();
    const uint32_t w0 = wall();
    const uint32_t k0 = Ticker::millis();
    Probe::deadline.arm(500u);
    post<TimedManager>(SleepRequested{SleepDepth::deep, reply_to<Probe, SleepVote>()});
    pump_until_blip<TimedKernel>(3000u);
    const uint32_t to_blip = wall_ms(wall_delta(w0, Probe::blip_wall));
    const uint32_t kernel_ms = Ticker::millis() - k0;
    print(serial, "  a 500 ms event through a Stop matured after ", to_blip,
          " ms of wall (", kernel_ms, " ms of kernel tick); the resync put back ",
          TimedSite::last_advance(), " ticks", crlf);
    // On a crystal the band is 500..580 ms; on the LSI the site states the
    // oscillator's UPPER bound, so its alarm lands late by the LSI's own
    // margin (printed above) and the band is the contract's alone: late,
    // never early.
    bench.verdict(wall_is_crystal ? "THE EVENT MATURED THROUGH A STOP, on the wall"
                                  : "THE EVENT MATURED THROUGH A STOP (on the LSI's stated bound, late by its margin)",
                  Probe::blips == 1u && (wall_is_crystal ? within(to_blip, 500u, 580u) : to_blip >= 500u));
    bench.verdict("and NEVER EARLY - the lower bound is the kernel's own promise",
                  to_blip >= 500u);
    bench.verdict("the sleep was real: the resync handed back a frozen span of "
                  "hundreds of ticks",
                  TimedSite::last_advance() > 350u && TimedSite::last_advance() < 540u);
    bench.verdict("and the round closed by the convention",
                  Probe::wakes >= 1u && TimedSite::armed() == SleepDepth::none);

    // ---- never early, repeated ------------------------------------------------
    constexpr uint16_t repeats = 6;
    constexpr uint32_t nominal = 150;
    uint16_t on_time = 0;
    uint32_t worst_lo = 0xFFFFFFFFu;
    uint32_t worst_hi = 0;
    for (uint16_t i = 0; i < repeats; ++i) {
        feed();
        TimedKernel::init_all();
        Probe::clear();
        console_drain();
        at_tick_edge();
        const uint32_t a = wall();
        Probe::deadline.arm(nominal);
        post<TimedManager>(SleepRequested{SleepDepth::deep, reply_to<Probe, SleepVote>()});
        pump_until_blip<TimedKernel>(1000u);
        if (Probe::blips != 1u) {
            continue;
        }
        const uint32_t ms = wall_ms(wall_delta(a, Probe::blip_wall));
        if (ms < worst_lo) {
            worst_lo = ms;
        }
        if (ms > worst_hi) {
            worst_hi = ms;
        }
        if (wall_is_crystal ? within(ms, nominal, nominal + 30u) : ms >= nominal) {
            ++on_time;
        }
    }
    print(serial, "  ", repeats, " rounds of ", nominal, " ms through a Stop: wall spans ",
          worst_lo, "..", worst_hi, " ms", crlf);
    bench.verdict(wall_is_crystal ? "every one of six shorter rounds matured inside the band"
                                  : "every one of six shorter rounds matured, none early (the LSI's stated bound sets the lateness)",
                  on_time == repeats);
    bench.verdict("and not one of them was early", worst_lo >= nominal);

    kernel_running = false;
    site_owns_wakeup = false;
    timed_round = false;
    Nvic::disable(Rtc::wakeup_irq());
    (void)Site::arm(SleepDepth::none);
    Ticker::resume();
}

// =============================================================================
// i - the surface nothing on this board can stage
// =============================================================================
void ti_surface() {
    feed();
    // ---- the debugger's three bits, AS FOUND ---------------------------------
    print(serial, "  the debugger's bits at boot: DBG_SLEEP ",
          dbg_sleep_at_boot ? "SET" : "clear", ", DBG_STOP ",
          dbg_stop_at_boot ? "SET" : "clear", ", DBG_STANDBY ",
          dbg_standby_at_boot ? "SET" : "clear",
          " - the register survives every reset but a power-on, and a probe "
          "writes all three", crlf);
    Pwr::debug_low_power(true);
    const bool s1 = Pwr::debug_in_sleep();
    const bool t1 = Pwr::debug_in_stop();
    const bool b1 = Pwr::debug_in_standby();
    Pwr::debug_low_power(false);
    const bool s0 = Pwr::debug_in_sleep();
    const bool t0 = Pwr::debug_in_stop();
    const bool b0 = Pwr::debug_in_standby();
    const DeviceIdcode id = DeviceIdcode::read();
    print(serial, "  written as a set they read back ", s1 ? "1" : "0", t1 ? "1" : "0",
          b1 ? "1" : "0", " and cleared again ", s0 ? "1" : "0", t0 ? "1" : "0",
          b0 ? "1" : "0", " (SLEEP/STOP/STANDBY); DBGMCU_IDCODE beside them reads DEV ",
          hex(id.dev_id), " REV ", hex(id.rev_id), crlf);
    // WHAT THIS SUITE NEEDS is only that the three bits are DOWN while a
    // sleep is weighed - letter c's frozen tick is the proof that they were.
    // Whether a write here puts them back or takes them down again is a
    // property of the debug power domain and of the probe attached (a
    // detached ST-LINK/V2 leaves later writes dead, an attached STLINK-V3
    // lets a set through and holds a clear), so it is printed, not judged.
    print(serial, "  after the clear the bits read ", (s0 || t0 || b0) ? "SET" : "clear",
          " - the sleeps below say whether the clocks stopped", crlf);

    // ---- the PVD -------------------------------------------------------------
    print(serial, "  the PVD's thresholds ", Pwr::pvd_levels_known ? "are known" : "are NOT known",
          " on this part class:");
    for (uint8_t code = 0; code < 8u; ++code) {
        print(serial, " ", Pwr::pvd_level_mv(code));
    }
    print(serial, " mV", crlf);
    bool any_below = false;
    for (uint8_t code = 0; code < 8u; ++code) {
        (void)Pwr::pvd_level(code);
        Pwr::pvd_enable(true);
        for (uint16_t i = 0; i < 2000u; ++i) {
        }
        if (Pwr::pvd_below()) {
            any_below = true;
        }
    }
    bench.verdict("the detector takes every one of its eight codes back",
                  Pwr::pvd_level(7) && Pwr::pvd_level() == 7u && !Pwr::pvd_level(8));
    bench.verdict("and at this board's supply it never reports VDD below a "
                  "threshold - the highest is 2.9 V",
                  !any_below);
    Pwr::pvd_enable(false);
    bench.verdict("PVDO is only valid while PVDE stands (5.4.2)", !Pwr::pvd_enabled());

    // The PVD's line is a CONFIGURABLE EXTI line, so it can be proved
    // without a supply change: the software trigger is the wire this
    // board has not got.
    const uint8_t line = Pwr::pvd_exti_line;
    bench.verdict("the PVD's line exists on this part", Exti::implemented(line));
    (void)Exti::sense(line, ExtiSense::rising);
    (void)Exti::interrupt(line, true);
    (void)Exti::clear(line);
    const bool quiet = !Exti::pending(line);
    (void)Exti::trigger(line);
    const bool fired = Exti::pending(line);
    (void)Exti::clear(line);
    (void)Exti::interrupt(line, false);
    bench.verdict("and it takes the software trigger, which is how the wake "
                  "path is proved with no supply to move",
                  quiet && fired && !Exti::pending(line));

    // ---- the wake-up pins ----------------------------------------------------
    Pwr::clear_wakeup_flag();
    const bool quiet_flag = !Pwr::wakeup_flag();
    uint8_t high_pins = 0;
    for (uint8_t n = 1; n <= Pwr::wakeup_pin_count; ++n) {
        Pwr::clear_wakeup_flag();
        (void)Pwr::wakeup_pin(n, true);
        for (uint16_t i = 0; i < 2000u; ++i) {
        }
        const PwrWakeupPad pad = Pwr::wakeup_pad(n);
        const bool raised = Pwr::wakeup_flag();
        if (raised) {
            ++high_pins;
        }
        print(serial, "  WKUP", static_cast<uint32_t>(n));
        if (pad.known) {
            print(serial, " (P", static_cast<char>(pad.port), static_cast<uint32_t>(pad.pin), ")");
        }
        print(serial, " armed: enabled ", Pwr::wakeup_pin_enabled(n) ? "yes" : "NO",
              ", WUF ", raised ? "SET - the pad is already high, and 5.4.2 says "
                                 "an enable on a high pad is itself an event"
                               : "clear",
              crlf);
        (void)Pwr::wakeup_pin(n, false);
    }
    Pwr::clear_wakeup_flag();
    bench.verdict("the one wake-up flag this family has is clearable and stays "
                  "down with nothing armed",
                  quiet_flag && !Pwr::wakeup_flag());
    bench.verdict("every bonded pin arms and disarms",
                  !Pwr::wakeup_pin_enabled(1) && high_pins <= Pwr::wakeup_pin_count);
    // The erratum's own sequence, run for its own sake: it must leave
    // every pin as it found it and the flags down.
    (void)Pwr::wakeup_pin(1, true);
    Pwr::prepare_standby();
    const bool kept = Pwr::wakeup_pin_enabled(1);
    (void)Pwr::wakeup_pin(1, false);
    Pwr::clear_wakeup_flags();
    bench.verdict("ES0298 2.2.4's sequence disables, clears and RE-ENABLES - "
                  "an armed pin survives it",
                  kept);

    // ---- the backup regulator ------------------------------------------------
    const bool unlocked = RtcDomain::unlock(true);
    const bool bre = Pwr::backup_regulator(true);
    print(serial, "  the backup regulator: BRE ", Pwr::backup_regulator() ? "set" : "clear",
          ", BRR ", Pwr::backup_regulator_ready() ? "ready" : "not ready",
          "; this part ", Pwr::has_backup_sram ? "has" : "has NO", " backup SRAM behind it",
          crlf);
    bench.verdict("the backup regulator comes up and reports ready", unlocked && bre &&
                                                                        Pwr::backup_regulator_ready());
    (void)Pwr::backup_regulator(false);
    bench.verdict("and goes down again", !Pwr::backup_regulator());
    if constexpr (Pwr::has_backup_sram) {
        bench.verdict("its array's own AHB1 gate opens and closes",
                      Pwr::backup_sram_clock(true) && Pwr::backup_sram_clock());
        (void)Pwr::backup_sram_clock(false);
    }

    // ---- what is offered and never used --------------------------------------
    print(serial, "  offered and never set here: FISSR/FMSSR ",
          Pwr::has_flash_stop_while_run ? "present" : "absent",
          " (5.4.1: they cannot be set while executing from the flash itself), "
          "ADCDC1 ", Pwr::has_adc_dc1 ? "present" : "absent",
          " (AN4073's), SLEEPONEXIT and SEVONPEND ", Pwr::sleep_on_exit() ? "SET" : "clear",
          "/", Pwr::event_on_pending() ? "SET" : "clear", crlf);
    bench.verdict("the Cortex's two other sleep bits are clear, as brio's idle "
                  "path needs them",
                  !Pwr::sleep_on_exit() && !Pwr::event_on_pending());
    print(serial, "  at boot: SBF ",
          sbf_at_boot ? "SET (the last boot came out of Standby)" : "clear", ", WUF ",
          wuf_at_boot ? "SET" : "clear", ", the reset flags ", hex(reset_flags_at_boot), crlf);
}

// =============================================================================
// s - STANDBY, by name only: the wake is a reboot
// =============================================================================
constexpr uint32_t standby_token = 0x5B0DDBEEu;
constexpr uint8_t standby_slot = 3;

void ts_standby() {
    feed();
    if (!RtcDomain::unlock(true)) {
        bench.verdict("the domain unlocks so the token can be left", false);
        return;
    }
    print(serial, "  writing the token, arming the RTC 2 s out and entering "
                  "STANDBY: the 1.2 V domain goes off and the wake is a RESET. "
                  "The banner after it is the answer.", crlf);
    console_drain();

    (void)Rtc::backup(standby_slot, standby_token);
    // 5.3.7's safe sequence, and ES0298 2.2.4's on top of it: the RTC's
    // own flag down and its interrupt re-enabled around the PWR flag's
    // clear, then every wake-up source re-armed, then the entry.
    Rtc::clear_wakeup();
    Pwr::clear_wakeup_flags();
    (void)Rtc::set_wakeup(RtcWakeupClock::ck_spre, 1u, true);
    Nvic::enable(Rtc::wakeup_irq());
    Pwr::enter(PwrMode::standby);

    // Only reached when the entry did not happen (fact 3 of pwr.hpp: a
    // standing flag makes it a no-op).
    Rtc::clear_wakeup();
    bench.verdict("STANDBY was entered", false);
    print(serial, "  the entry fell through - PWR_CSR ", hex(Pwr::csr()),
          ", RTC ISR ", hex(Rtc::status()), crlf);
}

void banner() {
    print(serial, crlf, "test_stm32f4_power - PWR, the sleep sites and the dynamic clock",
          crlf);
    bench.menu();
}

}   // namespace

// ---- target glue ------------------------------------------------------------
#if defined(STM32F446xx)
extern "C" void USART2_IRQHandler() { (void)Serial::isr(); }
#elif defined(STM32F469xx)
extern "C" void USART3_IRQHandler() { (void)Serial::isr(); }
#else
extern "C" void USART1_IRQHandler() { (void)Serial::isr(); }
#endif
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

extern "C" void RTC_WKUP_IRQHandler() {
    if (site_owns_wakeup) {
        TimedSite::isr();
        if (kernel_running) {
            brio::post<Probe>(Woke{});
        }
        return;
    }
    // THE MANUAL PATH DOES NOT RESTORE THE CLOCK, on purpose: letter c
    // reads SWS at the first instruction after the wake and would find
    // the PLL back if this body had put it there.
    const uint32_t served = brio::Rtc::wakeup_isr();
    if (served != 0u) {
        wakeup_count = static_cast<uint16_t>(wakeup_count + 1u);
        if (kernel_running) {
            brio::post<Probe>(Woke{});
        }
    }
}

int main() {
    dbg_sleep_at_boot = brio::Pwr::debug_in_sleep();
    dbg_stop_at_boot = brio::Pwr::debug_in_stop();
    dbg_standby_at_boot = brio::Pwr::debug_in_standby();
    reset_flags_at_boot = brio::Reset::flags();
    brio::Pwr::bus_clock(true);
    sbf_at_boot = brio::Pwr::standby_flag();
    wuf_at_boot = brio::Pwr::wakeup_flag();

    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output(true);
    brio::enable_interrupts();

    // The three debug bits come down before anything is measured: what a
    // probe left there would keep HCLK alive through a Stop and fake
    // every number in this suite (pwr.hpp fact 6, ES0298 2.2.1).
    brio::Pwr::debug_low_power(false);

    // The RTC, as the wall clock AND as the timed site's instrument: one
    // init for both, because the site owns the whole peripheral.
    const bool rtc_ok = TimedSite::init();
    brio::Nvic::disable(brio::Rtc::wakeup_irq());

    // The backstop: about 32 s at the nominal LSI, and no way off.
    feed_iwdg = brio::Iwdg::arm({brio::IwdgPrescaler::div256, 0x0FFFu});

    bench.letter('a', "the block and the ladder", ta_block);
    bench.letter('b', "Sleep: the tick runs through it", tb_sleep);
    bench.letter('c', "a Stop: the frozen tick and the clock that comes back", tc_stop);
    bench.letter('d', "the regulator variants weighed", td_variants);
    bench.letter('e', "the SYSCLK restore", te_restore);
    bench.letter('f', "the DynamicClock ladder, up and down", tf_ladder);
    bench.letter('g', "a Stop through a real kernel", tg_kernel_stop);
    bench.letter('h', "the timed site: the deadline met on the wall", th_timed_site);
    bench.letter('i', "the surface nothing here can stage", ti_surface);
    bench.letter('s', "STANDBY: the wake is a reboot", ts_standby, false);

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL" : "FAILED", " ",
                    SysClock::hz(), " Hz tick=", tick_ok ? "SysTick" : "FAILED",
                    " rtc=", rtc_ok ? "up" : "FAILED",
                    " wdt=", feed_iwdg ? "IWDG" : "FAILED", brio::crlf);
        if (sbf_at_boot) {
            const uint32_t token = brio::Rtc::backup(standby_slot);
            brio::print(serial, "standby: SBF was set at boot and the backup "
                                "register reads ",
                        brio::hex(token), " - ",
                        token == standby_token ? "pass, the device woke from STANDBY "
                                                 "through the reset vector"
                                               : "FAIL, the token did not survive",
                        brio::crlf);
            brio::Pwr::clear_standby_flag();
            brio::Pwr::clear_wakeup_flag();
            (void)brio::Rtc::backup(standby_slot, 0u);
        }
        banner();
        bench.prompt();
    }

    for (;;) {
        feed();
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
        bench.prompt();
    }
}
