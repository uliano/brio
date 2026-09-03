// test_stm32_sleep - the reference bench suite for the STM32G0's
// STOPPING half: stm32g0/pwr.hpp (RM0444 ch. 4, the whole of it) and
// stm32g0/sleep.hpp (util/power.hpp's depth ladder on this silicon, and
// the RTC-backed timebase that lifts its restriction).
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by tools/bench.py's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// NOTHING TO WIRE. The RTC is the wake-up source and the wall clock, the
// console is the Nucleo's own ST-LINK virtual COM port, and no flash is
// written and no option byte touched.
//
// THE WALL CLOCK IS THE RTC, AND IT HAS TO BE. Every TIM of this family
// lives in the VCORE domain and stops in Stop; SysTick rides HCLK and
// stops with it; the RTC does not. So this suite runs the calendar on
// the LSE crystal with the prescalers split the OTHER way from the
// usual - PREDIV_A 0 and PREDIV_S 32767, which puts ck_apre at the
// crystal's full 32768 Hz and makes the sub-second counter a 30.5 us
// stopwatch that keeps counting with every clock in the chip stopped.
// The kernel's own millis() is the SUBJECT of half these letters and
// never the judge.
//
// THE BACKSTOP IS THE IWDG, armed once in main() at about 32 seconds and
// refreshed at the top of every letter and inside every long loop. It
// cannot be turned off again (28.3.1), which is the point: a sleep with
// no wake behind it costs one reboot and a banner instead of a board
// that has to be re-flashed.
//
// What is exercised, letter by letter:
//   a  the block and the ladder: SLEEPDEEP and LPMS as the two-register
//      pair they are, the Reserved code refused, `armed()` as a pure
//      read, and the four rungs of the mapping this target chose
//   b  Sleep, the shallow rung: the tick keeps running and the idle
//      hook comes back on it
//   c  STOP: kernel time stands still for the whole sleep, measured on
//      the RTC - and the system clock comes back as HSISYS, which the
//      site's resume_clock() puts right
//   d  the wake-up cost of each rung, differenced against a control
//   e  a Stop through a REAL KERNEL: the vote round, the
//      first-event-after-wake contract, and the PLAIN site's honest
//      restriction (an armed time event matures LATE by the sleep)
//   f  THE TIMED SITE: the same deadline met on the wall, never early,
//      with the frozen span handed back to the ticker
//   g  ES0548 2.2.4 staged, and the entry preconditions that make a
//      Stop silently not happen
//   h  the register surface that cannot be staged on this desk, and
//      what it says
//
//   s  (by name only) STANDBY. The VCORE domain is powered off and the
//      wake comes back THROUGH THE RESET VECTOR, so this letter reboots
//      the board and resumes from a TAMP backup register - which is
//      what a program without SRAM has to do. Not in `z`.
//   u  (by name only) SHUTDOWN, the same one rung deeper.
//      Run either with
//          python3 tools/bench.py run E s --app test_stm32_sleep
//                  --expect="pass," --timeout 200
//
// build: boards = g0b1re
// build: monitor_speed = 115200

#include <stdint.h>

#include <optional>

#include "kernel/kernel.hpp"
#include "kernel/post.hpp"
#include "kernel/time_event.hpp"
#include "stm32g0/clock.hpp"
#include "stm32g0/delay.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/platform_stm32.hpp"
#include "stm32g0/pwr.hpp"
#include "stm32g0/reset.hpp"
#include "stm32g0/rtc.hpp"
#include "stm32g0/sleep.hpp"
#include "stm32g0/ticker.hpp"
#include "stm32g0/usart.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Stm32Platform;

constexpr UartPins console_pins{
    .tx = {'A', 2, PinFunction::af1},
    .rx = {'A', 3, PinFunction::af1},
};
using Serial = Uart<2, console_pins>;
constexpr Serial serial;

TestBench<Serial> bench;

// ---------------------------------------------------------------------------
// The wall clock: the RTC's sub-second counter, 30.5 us per tick
// ---------------------------------------------------------------------------
//
// PREDIV_A 0 / PREDIV_S 32767 from a 32768 Hz crystal: ck_apre is the
// crystal itself and ck_spre is still exactly 1 Hz, so the calendar is a
// calendar AND the sub-second counter is a stopwatch. The reading is
// taken modulo the MINUTE, which is longer than any sleep this suite
// takes and keeps the arithmetic in 32 bits.

constexpr uint32_t lse_hz = 32768;
constexpr RtcPrescalers wall_prescalers{.async = 0, .sync = 32767};

bool wall_ready = false;

/// The wall's tick rate, READ FROM THE SILICON and not assumed: the
/// sub-second counter reloads from PREDIV_S once per ck_spre period, and
/// ck_spre is one hertz by construction, so PREDIV_S + 1 ticks make a
/// second whatever split is in force. Letter f hands the RTC to the
/// TIMED SITE, which owns different prescalers, and a helper that had
/// baked the split in would have reported nonsense there - it did, once,
/// which is why it reads the register now.
/// How many sub-second ticks make one CALENDAR second: PREDIV_S + 1,
/// whatever split is in force.
uint32_t wall_ticks_per_second() {
    return static_cast<uint32_t>(Rtc::prescalers().sync) + 1u;
}
uint32_t wall_modulus() { return 60u * wall_ticks_per_second(); }

/// How many sub-second ticks make one REAL second: ck_apre, which is the
/// crystal divided by PREDIV_A + 1 - and the two are NOT the same number
/// when a caller's stated RTCCLK rate is a deliberate over-estimate.
/// The timed site states 32800 against a crystal that runs at 32768, so
/// its calendar second is 0.098 % long; converting with the CRYSTAL is
/// what keeps this instrument honest about a site that is honest about
/// being slow. (It cost one verdict: a 500 ms deadline read 499 ms of
/// "wall" that was really 500 ms of world.)
uint32_t wall_hz() {
    return lse_hz / (static_cast<uint32_t>(Rtc::prescalers().async) + 1u);
}

/// Sub-second ticks since the top of the minute, or 0xFFFFFFFF when the
/// calendar could not be read coherently.
uint32_t wall() {
    RtcReading r{};
    if (!Rtc::read(r)) {
        return 0xFFFFFFFFu;
    }
    const uint32_t per_second = wall_ticks_per_second();
    return static_cast<uint32_t>(r.time.second) * per_second +
           (per_second - 1u - r.subsecond);
}

uint32_t wall_delta(uint32_t from, uint32_t to) {
    return (to >= from) ? (to - from) : (wall_modulus() - from + to);
}

uint32_t wall_us(uint32_t ticks) {
    return static_cast<uint32_t>((static_cast<uint64_t>(ticks) * 1'000'000ULL) /
                                 wall_hz());
}
uint32_t wall_ms(uint32_t ticks) {
    return static_cast<uint32_t>((static_cast<uint64_t>(ticks) * 1000ULL) /
                                 wall_hz());
}

// ---------------------------------------------------------------------------
// The backstop
// ---------------------------------------------------------------------------
//
// 28.3.1: once started, nothing this program writes stops it again. So it
// is armed ONCE, generously, and fed - which is the shape a real
// application's watchdog has anyway.

void feed() { Iwdg::refresh(); }

bool arm_backstop() {
    // /256 with the full reload: about 32 s at this die's LSI.
    return Iwdg::arm(IwdgConfig{.prescaler = IwdgPrescaler::div256,
                                .reload = 0x0FFF,
                                .window = 0x0FFF});
}

// ---------------------------------------------------------------------------
// Shared state
// ---------------------------------------------------------------------------

uint32_t boot_flags = 0;
uint32_t boot_sr1 = 0;
bool lse_ok = false;

/// Counted by the RTC handler so that "the wake was the alarm" is a fact
/// on the console and not an assumption.
volatile uint32_t rtc_wakes = 0;

/// Which body the shared RTC vector runs: the timed site's four acts, or
/// a bare acknowledgement. Both are compiled and only one is right at a
/// time, so every letter that drives the RTC by hand says so.
bool timed_round = false;

/// Whether a kernel is running at all: the bare-metal letters share the
/// same vector and must not post into a queue nobody pumps.
bool kernel_live = false;

using Site = Stm32SleepSite<SysClock>;

/// The timed site runs on the same crystal, and states a rate NOT BELOW
/// it: 32800 against 32768, an over-estimate of about a per mille, which
/// is the direction the contract wants (late, never early).
constexpr TimedSleepConfig timed_cfg{.rtcclk_hz = 32800,
                                     .source = RtcClockSource::lse};
using TimedSite = Stm32TimedSleepSite<P, SysClock, timed_cfg>;

bool within(uint32_t v, uint32_t lo, uint32_t hi) { return v >= lo && v <= hi; }

void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    const uint32_t t0 = wall();
    while (wall_delta(t0, wall()) < wall_hz() / 300u) {
    }
}

/// Put the RTC back on this suite's own stopwatch split after a letter
/// that changed it (the timed site owns different prescalers).
bool wall_up() {
    if (Rtc::prescalers().sync == wall_prescalers.sync &&
        Rtc::prescalers().async == wall_prescalers.async) {
        return true;
    }
    return Rtc::init(wall_prescalers,
                     RtcDateTime{.hour = 0, .minute = 0, .second = 0,
                                 .day = 1, .month = 1, .year = 24,
                                 .weekday = 1});
}

/// One bare sleep: arm the RTC's wake-up timer `counts` ticks of
/// RTCCLK/16 ahead, arm `mode`, stop, and report how long the wall says
/// it took. The NVIC line is the wake; the handler acknowledges.
///
/// THE TICK IS PAUSED AROUND A DEEP SLEEP, and that is not tidiness.
/// 4.3.3: a WFI enters a low-power mode "only if no interrupt is
/// pending", and this kernel's SysTick raises one every millisecond;
/// once the Stop is taken HCLK stops and SysTick with it, so on a bare
/// board the window is one instruction wide - but under a debugger's
/// DBGMCU_CR.DBG_STOP the debug logic keeps HCLK running inside the
/// Stop and the next tick ends it (letter c measures both). Pausing
/// costs nothing at all, because kernel time stops in a Stop anyway
/// (that is what the timed site exists to repair), and it makes the
/// entry deterministic in both states.
SysclkSource woke_source = SysclkSource::pllrclk;
SysclkSource resumed_source = SysclkSource::pllrclk;
bool resumed_clock = false;

uint32_t one_sleep(PwrMode mode, uint32_t counts, bool pause_tick = true) {
    // THE VECTOR IS SHARED and its behaviour is a variable, so a
    // bare-metal sleep says which body it wants. Letter f leaves
    // `timed_round` set, and a later letter that inherited it would find
    // its clock ALREADY RESTORED by the timed site's own act zero before
    // it could look - which is a true statement about a wake that never
    // happened the way the letter thought. (It cost this suite a warm-run
    // failure that a cold run never showed.)
    timed_round = false;
    Nvic::clear_pending(Rtc::irq());
    Nvic::enable(Rtc::irq());
    if (!Rtc::set_wakeup(RtcWakeupClock::div16, counts, true)) {
        return 0;
    }
    // A NVIC line left pending by an earlier assertion is a pending
    // interrupt like any other, and 4.3.3 aborts a Stop entry on one -
    // so the line is swept AFTER the timer is armed as well as before.
    Nvic::clear_pending(Rtc::irq());
    if (pause_tick) {
        Ticker::pause();
    }
    const uint32_t t0 = wall();
    (void)Pwr::arm(mode);
    __DSB();
    __WFI();
    // WHAT SYSCLK IS AT THE INSTANT OF WAKING, sampled before anything
    // puts it back: 4.3.6's "the system clock, when exiting Stop 0 or
    // Stop 1 mode, is the HSISYS clock". It is captured HERE rather
    // than in a second sleep of its own, because a second sleep is a
    // second chance to be aborted by a pending line and this one has
    // already proved it slept.
    woke_source = Rcc::sysclk_status();
    resumed_clock = Site::resume_clock();
    resumed_source = Rcc::sysclk_status();
    const uint32_t t1 = wall();
    if (pause_tick) {
        Ticker::resume();
    }
    (void)Pwr::arm(PwrMode::sleep);
    Rtc::clear_wakeup();
    Nvic::disable(Rtc::irq());
    return wall_delta(t0, t1);
}

// =============================================================================
// a - the block and the ladder
// =============================================================================
void ta_ladder() {
    feed();
    print(serial, "  PWR: CR1=", hex(Pwr::cr1()), " CR3=", hex(Pwr::cr3()),
          " SR1=", hex(Pwr::sr1()), " SR2=", hex(Pwr::sr2()),
          " range=", Pwr::range(), crlf);

    bench.verdict("the PWR block answers, which means its bus clock is open "
                  "(5.2.17)",
                  Pwr::bus_clock() && (Pwr::range() == 1u || Pwr::range() == 2u));
    bench.verdict("this program runs in Range 1, the only range the flash "
                  "latency table in clock.hpp is written for",
                  Pwr::range() == 1u);

    // THE MODE IS TWO REGISTERS IN TWO PLACES, and neither is the
    // other's. With SLEEPDEEP clear, LPMS says nothing at all.
    Pwr::deep_sleep(false);
    bench.verdict("with SLEEPDEEP clear the armed mode is Sleep, whatever "
                  "LPMS holds",
                  Pwr::mode() == PwrMode::sleep);
    bench.verdict("and a Reserved LPMS code is refused before it reaches "
                  "PWR_CR1 (4.4.1 marks 010 Reserved)",
                  !Pwr::arm(static_cast<PwrMode>(2)));

    // THE LADDER THIS TARGET CHOSE. Each rung armed, then read back off
    // the silicon - there is no shadow variable anywhere in this
    // stratum, so armed() IS the state.
    struct Leg {
        const char* name;
        SleepDepth depth;
        PwrMode mode;
        SleepDepth reported;
    };
    const Leg legs[] = {
        // NB the names carry no " -> " and no "ALL:": both are
        // tools/bench.py's own markers, and a verdict line that contains
        // one truncates the capture. (It did: the arrow in this table
        // and a "WALL:" three letters down between them made every z run
        // stop early, and the suite looked broken when it was not.)
        {"none maps to Sleep", SleepDepth::none, PwrMode::sleep,
         SleepDepth::none},
        {"light maps to Sleep too (nothing lives between Sleep and Stop "
         "here)",
         SleepDepth::light, PwrMode::sleep, SleepDepth::none},
        {"standby maps to Stop 0 (main regulator, the fastest deep wake)",
         SleepDepth::standby, PwrMode::stop0, SleepDepth::standby},
        {"deep maps to Stop 1 (low-power regulator, the deepest RESUMABLE "
         "mode this family has)",
         SleepDepth::deep, PwrMode::stop1, SleepDepth::deep},
    };
    for (const Leg& leg : legs) {
        const bool armed = Site::arm(leg.depth);
        const bool right_mode = Pwr::mode() == leg.mode;
        const bool right_report = Site::armed() == leg.reported;
        bench.verdict(leg.name, armed && right_mode && right_report);
    }
    Site::disarm();
    bench.verdict("disarm() puts the machine back to the kernel's own idle",
                  Site::armed() == SleepDepth::none &&
                      Pwr::mode() == PwrMode::sleep);

    // THE TWO MODES DELIBERATELY OFF THE LADDER. They are reachable, and
    // saying so is the point: sleep.hpp's header explains why no rung
    // maps there (a program does not RESUME from them, so the model's
    // first-event-after-wake contract has nothing to happen in).
    bench.verdict("Standby and Shutdown are still ARMABLE by hand through "
                  "Pwr - off the ladder is not out of reach",
                  Pwr::arm(PwrMode::standby) &&
                      Pwr::mode() == PwrMode::standby &&
                      Pwr::arm(PwrMode::shutdown) &&
                      Pwr::mode() == PwrMode::shutdown);
    (void)Pwr::arm(PwrMode::sleep);
    bench.verdict("and the site reports the deepest rung for either, never "
                  "something shallower than what is really armed",
                  (Pwr::arm(PwrMode::standby), Site::armed()) == SleepDepth::deep);
    (void)Pwr::arm(PwrMode::sleep);

    // The per-part geometry, from the reserve.
    print(serial, "  wake-up pins bonded here:");
    for (uint8_t i = 1; i <= 6u; ++i) {
        if (Pwr::wakeup_pin_present(i)) {
            print(serial, " WKUP", i);
        }
    }
    print(serial, " (", pwr_wakeup_pin_count(), " of six)", crlf);
    bench.verdict("the G0B1 bonds all six wake-up pins, and a pin the part "
                  "has not got is refused rather than written",
                  pwr_wakeup_pin_count() == 6u && !Pwr::wakeup_pin(7, true));
    bench.verdict("the Standby pull registers follow the GPIO bonding: port "
                  "A has one, port G is nowhere",
                  Pwr::standby_pull('A', 0, false, false) &&
                      !Pwr::standby_pull('G', 0, false, false));
    bench.verdict("and asking for a pull-up and a pull-down at once is a "
                  "contradiction, not a resolution (4.4.8)",
                  !Pwr::standby_pull('A', 0, true, true));
}

// =============================================================================
// b - Sleep, the shallow rung
// =============================================================================
void tb_sleep() {
    feed();
    // The kernel's own idle path, with nothing armed: HCLK, SysTick and
    // every peripheral keep running (5.3), so the tick brings it back.
    Site::disarm();
    console_drain();
    const uint32_t k0 = Ticker::millis();
    const uint32_t w0 = wall();
    for (uint16_t i = 0; i < 32u; ++i) {
        P::idle();
    }
    const uint32_t kernel_ms = Ticker::millis() - k0;
    const uint32_t wall_span = wall_ms(wall_delta(w0, wall()));
    print(serial, "  32 idle() calls: ", kernel_ms, " ms of kernel tick over ",
          wall_span, " ms of wall", crlf);
    bench.verdict("Sleep does not stop the timebase: the kernel tick tracks "
                  "the wall through 32 sleeps",
                  kernel_ms >= 25u && kernel_ms <= wall_span + 3u &&
                      wall_span <= kernel_ms + 8u);
    bench.verdict("and idle() returns with interrupts enabled",
                  P::interrupts_enabled());
    bench.verdict("nothing deeper was armed while it did so",
                  Pwr::mode() == PwrMode::sleep);
}

// =============================================================================
// c - Stop: kernel time stands still, and the clock comes back changed
// =============================================================================
void tc_stop_freezes() {
    feed();
    // ~250 ms on RTCCLK/16: 2048 counts of a 2048 Hz clock.
    constexpr uint32_t counts = 512;   // (512 + 1) / 2048 s
    console_drain();

    // FIRST, THE CONTROL: the same sleep with the kernel's tick still
    // armed. 4.3.3's "only if no interrupt is pending" is what this
    // measures - AND WHAT A DEBUGGER LEFT BEHIND DECIDES THE ANSWER.
    // DBGMCU_CR.DBG_STOP keeps HCLK running inside a Stop so that a
    // probe can still talk to the core; with it set SysTick keeps
    // firing and the next tick ends the sleep, with it clear HCLK
    // stops with the WFI and the sleep lasts. The bit outlives every
    // reset but a power-on and OpenOCD sets it at every connection
    // that finds the DBGMCU's clock gate open, so this letter READS
    // the bit and judges the state it finds rather than assuming one:
    // the first version of this verdict assumed "does not last", which
    // was true of the probe's setting and not of the silicon.
    const uint32_t with_tick = wall_ms(one_sleep(PwrMode::stop1, counts, false));
    const uint32_t k0 = Ticker::millis();
    const uint32_t slept = one_sleep(PwrMode::stop1, counts);
    const uint32_t kernel_ms = Ticker::millis() - k0;
    const uint32_t slept_ms = wall_ms(slept);
    const bool dbg_stop = Pwr::debug_in_stop();
    print(serial, "  a 250 ms Stop 1 with the kernel tick STILL ARMED lasted ",
          with_tick, " ms; with it paused, ", slept_ms,
          " (DBGMCU_CR.DBG_STOP = ", dbg_stop, ")", crlf);
    if (dbg_stop) {
        bench.verdict("UNDER A DEBUGGER'S DBG_STOP the debug logic keeps HCLK "
                      "and SysTick running inside a Stop, and a Stop entered "
                      "with the tick armed is ended by the next tick",
                      with_tick < 100u);
    } else {
        bench.verdict("A STOP ENTERED WITH THE TICK ARMED LASTS on a board no "
                      "debugger has touched: 4.3.3's pending-interrupt window "
                      "is one instruction wide, and HCLK stops with the WFI",
                      within(with_tick, 230u, 290u));
    }
    print(serial, "  a Stop 1 of ", slept_ms,
          " ms of wall advanced the kernel tick by ", kernel_ms, " ms", crlf);
    bench.verdict("the sleep really happened, and the RTC saw all of it",
                  within(slept_ms, 230u, 290u));
    bench.verdict("KERNEL TIME STANDS STILL ACROSS A STOP - SysTick rides "
                  "HCLK and HCLK is one of the clocks that stopped (5.3)",
                  kernel_ms < 20u);
    bench.verdict("and the RTC's own vector is what ended it",
                  rtc_wakes > 0u);

    // WHAT COMES BACK IS HSISYS (4.3.6), and the reading was taken
    // inside the very sleep judged above - see one_sleep().
    print(serial, "  SYSCLK at the instant of waking: ",
          static_cast<uint32_t>(woke_source),
          " (0 = HSISYS, 2 = PLLRCLK); after resume_clock(): ",
          static_cast<uint32_t>(resumed_source), crlf);
    bench.verdict("a Stop drops SYSCLK to HSISYS and stops the PLL (4.3.6) - "
                  "a program at 64 MHz wakes at 16",
                  woke_source == SysclkSource::hsisys);
    bench.verdict("and the site's resume_clock() puts the clock task back "
                  "where it promised to be",
                  resumed_clock && resumed_source == SysclkSource::pllrclk);
    // NOTHING MAY BE PRINTED BETWEEN THE WFI AND THAT RESTORE - the
    // console's divisor was computed for 64 MHz and the USART is fed a
    // quarter of it in between - which is why every reading above is
    // taken into a variable and reported afterwards. There is no
    // verdict for that: it is a property of how this letter is written,
    // and a verdict that cannot fail is noise.
}

// =============================================================================
// d - what each rung costs to leave
// =============================================================================
void td_wake_cost() {
    feed();
    // WHAT THIS DESK CAN AND CANNOT SAY, stated before the numbers.
    // DS13560 table 37 puts the wake from Stop 0 at 5.6 us typical and
    // from Stop 1 at 9.0 - and the finest clock that survives a Stop on
    // this chip is the RTC's own sub-second counter at 30.5 us a tick.
    // Every TIM lives in the VCORE domain and stops; SysTick rides HCLK
    // and stops with it; there is nothing else. So what follows is a
    // BOUND and a shape, not a latency, and the ordering the ladder
    // rests on is DECLINED rather than guessed at.
    //
    // The loop is N whole rounds of arm-sleep-wake against the same N
    // rounds of the shallowest rung, with the alarm RE-ARMED every round
    // - and even so it locks to RTCCLK, which is the samc bench's own
    // lesson about sub-tick overheads.
    constexpr uint16_t rounds = 32;
    constexpr uint32_t counts = 15;   // (15 + 1) / 2048 s ~ 7.8 ms

    struct Leg {
        const char* name;
        PwrMode mode;
        bool flash_down;
    };
    const Leg legs[] = {
        {"Sleep", PwrMode::sleep, false},
        {"Stop 0", PwrMode::stop0, false},
        {"Stop 1", PwrMode::stop1, false},
        {"Stop 1, flash powered down", PwrMode::stop1, true},
    };
    uint32_t per_round[4] = {0, 0, 0, 0};
    uint8_t n = 0;
    for (const Leg& leg : legs) {
        feed();
        Pwr::flash_power_down_stop(leg.flash_down);
        console_drain();
        const uint32_t t0 = wall();
        for (uint16_t i = 0; i < rounds; ++i) {
            (void)one_sleep(leg.mode, counts);
        }
        const uint32_t span = wall_delta(t0, wall());
        per_round[n++] = wall_us(span) / rounds;
    }
    Pwr::flash_power_down_stop(false);

    for (uint8_t i = 0; i < n; ++i) {
        print(serial, "  ", legs[i].name, ": ", per_round[i],
              " us per arm-sleep-wake round", crlf);
    }
    const uint32_t tick_us = wall_us(1);
    const uint32_t stop0 = per_round[1] > per_round[0]
                               ? per_round[1] - per_round[0] : 0u;
    const uint32_t stop1 = per_round[2] > per_round[0]
                               ? per_round[2] - per_round[0] : 0u;
    print(serial, "  the wall's own tick is ", tick_us,
          " us, and DS13560 table 37 puts these wakes at 5.6 us (Stop 0) "
          "and 9.0 (Stop 1) - so the differences below are AT the "
          "instrument's resolution and are a bound, not a latency", crlf);
    print(serial, "  Stop 0 - Sleep = ", stop0, " us, Stop 1 - Sleep = ",
          stop1, " us, per round", crlf);

    bench.verdict("every rung wakes: no round was lost, and the four legs "
                  "took the same time to within one wall tick per round",
                  per_round[0] != 0u && per_round[1] != 0u &&
                      per_round[2] != 0u && per_round[3] != 0u);
    bench.verdict("A STOP COSTS LESS THAN ONE WALL TICK MORE TO LEAVE THAN "
                  "A SLEEP - which is the whole of what this desk can say, "
                  "and it agrees with the datasheet's 5.6..11.2 us",
                  stop0 <= 2u * tick_us && stop1 <= 2u * tick_us);
    // THE ORDERING IS DECLINED, and saying so is the point: the mapping
    // of `standby` to Stop 0 and `deep` to Stop 1 rests on 4.1.3's
    // regulator argument and on table 37, not on a measurement this
    // bench can make.
    print(serial, "  Stop 0 against Stop 1: ",
          per_round[1] == per_round[2]
              ? "indistinguishable on this instrument"
              : "different by less than the resolution",
          " - the ladder's ordering rests on 4.1.3 and DS13560 table 37, "
          "and this suite DECLINES to claim it was measured", crlf);
    bench.verdict("and powering the flash down over a Stop changes nothing "
                  "this instrument can see either (table 37 prices it at "
                  "about 4 us)",
                  per_round[3] <= per_round[2] + 2u * tick_us);
}

// =============================================================================
// e - a Stop through a real kernel, and the plain site's restriction
// =============================================================================

struct Blip {};
struct Woke {};   ///< posted by the RTC handler on a plain-site wake

struct Probe : Fsm<Probe, SleepVote, PrepareSleep, WakeReport, Blip, Woke> {
    static inline EventQueue<Event, 8, P> queue;
    static inline TimeEvent<P, Probe, Blip> deadline{Blip{}};

    static inline uint16_t blips = 0;
    static inline uint16_t woke = 0;
    static inline uint16_t wakes = 0;
    static inline uint16_t votes = 0;
    static inline bool last_ok = false;
    static inline uint32_t blip_wall = 0;
    static inline SleepDepth last_report = SleepDepth::none;

    static void clear() {
        blips = woke = wakes = votes = 0;
        last_ok = false;
        blip_wall = 0;
        last_report = SleepDepth::none;
    }

    static void init() { start(&only); }
    static Status only(const Event& e);
};

using PlainManager = PowerManager<P, Site, PowerConfig{}, Probe>;
using TimedManager = PowerManager<P, TimedSite, PowerConfig{}, Probe>;
using PlainKernel = Kernel<P, Probe, PlainManager>;
using TimedKernel = Kernel<P, Probe, TimedManager>;


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
            p.reply.send(SleepVote{true});
            return handled();
        },
        [](WakeReport w) {
            ++wakes;
            last_report = w.was;
            return handled();
        },
        [](Woke) {
            // THE MODEL'S CONVENTION, and with a Stop it is
            // LOAD-BEARING: a wake path with nothing to say says it
            // with SleepRequested{none}. Without it the manager never
            // learns the machine came back, the site is never
            // disarmed, and on this target that means the clock stays
            // at 16 MHz and the KERNEL'S TICK STAYS PAUSED - so
            // nothing matures, ever.
            ++woke;
            if (timed_round) {
                post<TimedManager>(SleepRequested{SleepDepth::none,
                                                  reply_to<Probe, SleepVote>()});
            } else {
                post<PlainManager>(SleepRequested{SleepDepth::none,
                                                  reply_to<Probe, SleepVote>()});
            }
            return handled();
        },
        [](Blip) {
            ++blips;
            blip_wall = wall();
            // THE CONVENTION (util/power.hpp): after a wake, speak to
            // the manager - here, nothing more to do, so say none.
            if (timed_round) {
                post<TimedManager>(SleepRequested{SleepDepth::none,
                                                  reply_to<Probe, SleepVote>()});
            } else {
                post<PlainManager>(SleepRequested{SleepDepth::none,
                                                  reply_to<Probe, SleepVote>()});
            }
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

void te_kernel_stop() {
    feed();
    (void)wall_up();
    PlainKernel::init_all();
    Probe::clear();
    timed_round = false;
    kernel_live = true;

    Nvic::enable(Rtc::irq());
    // The wake is the RTC's own, 250 ms out: the manager knows nothing
    // about it, which is the point - a deep sleep's wake source is the
    // application's business and the model never asks.
    (void)Rtc::set_wakeup(RtcWakeupClock::div16, 512, true);

    console_drain();
    const uint32_t w0 = wall();
    const uint32_t k0 = Ticker::millis();
    Probe::deadline.arm(500u);
    post<PlainManager>(SleepRequested{SleepDepth::deep,
                                      reply_to<Probe, SleepVote>()});
    pump_until_blip<PlainKernel>(3000u);
    Rtc::clear_wakeup();
    Nvic::disable(Rtc::irq());
    kernel_live = false;

    const uint32_t to_blip = wall_ms(wall_delta(w0, Probe::blip_wall));
    const uint32_t kernel_ms = Ticker::millis() - k0;
    print(serial, "  the round: ", Probe::votes, " vote(s), ", Probe::wakes,
          " wake report(s) at depth ",
          static_cast<uint32_t>(Probe::last_report), "; a 500 ms event "
          "matured after ", to_blip, " ms of WALL and ", kernel_ms,
          " ms of kernel tick", crlf);

    bench.verdict("the vote round ran and the site was armed at the depth "
                  "the target really took",
                  Probe::votes >= 1u && Probe::last_ok &&
                      Probe::last_report == SleepDepth::deep);
    bench.verdict("the first event after the wake ended the round - nothing "
                  "polled, and the manager disarmed on its own",
                  Probe::woke >= 1u && Probe::wakes >= 1u &&
                      Site::armed() == SleepDepth::none);
    bench.verdict("the deadline was met in KERNEL time, which is all the "
                  "kernel promises",
                  Probe::blips == 1u && kernel_ms >= 500u);
    bench.verdict("BUT NOT ON THE WALL - with the plain site the sleep is "
                  "time the tick never counted, so the event matured LATE "
                  "by about the length of the Stop",
                  to_blip > 700u);
}

// =============================================================================
// f - the timed site: the same deadline, on the wall
// =============================================================================
void tf_timed_site() {
    feed();
    const bool up = TimedSite::init();
    print(serial, "  the timed site ", up ? "initialized" : "REFUSED",
          "; its prescalers are A ", TimedSite::prescalers.async, " / S ",
          TimedSite::prescalers.sync, ", its fast alarm clock ",
          TimedSite::fast_hz, " Hz, and it reaches ",
          TimedSite::fast_span_ticks, " ticks on it", crlf);
    bench.verdict("the timed site comes up on the crystal", up);
    if (!up) {
        return;
    }

    // The alarm's arithmetic, checked before any sleep: ceil of the
    // deadline in ck_wut counts.
    TimedKernel::init_all();
    Probe::clear();
    timed_round = true;
    kernel_live = true;
    Probe::deadline.arm(500u);
    const bool armed = TimedSite::arm(SleepDepth::deep);
    const uint32_t reload = TimedSite::last_reload();
    const bool fast = TimedSite::last_alarm_was_fast();
    TimedSite::disarm();
    Probe::deadline.disarm();
    const uint32_t want = (500u * TimedSite::fast_hz + P::ticks_per_second - 1u) /
                          P::ticks_per_second;
    print(serial, "  a 500 ms deadline placed the alarm at reload ", reload,
          " on the ", fast ? "fast" : "one-second",
          " clock; the arithmetic says ", want, crlf);
    bench.verdict("the alarm lands where the stated arithmetic puts it",
                  armed && fast && reload == want);
    // NOTHING MAY BE PRINTED BETWEEN arm() AND disarm(). arm() pauses
    // the kernel's tick for a deep rung, so the frozen span the resync
    // hands back is the WHOLE armed window - and a verdict line is four
    // milliseconds of console. The two calls therefore go back to back
    // and the judging happens afterwards.
    const bool no_deadline_armed = TimedSite::arm(SleepDepth::deep);
    const bool no_alarm = !TimedSite::alarm_armed();
    TimedSite::disarm();
    const uint32_t napless = TimedSite::last_advance();
    bench.verdict("a deadline-less round places no alarm at all",
                  no_deadline_armed && no_alarm);
    bench.verdict("and a round that never slept advances at most a tick",
                  napless <= 1u);

    // THE ROUND TRIP.
    TimedKernel::init_all();
    Probe::clear();
    // THE CONSOLE IS DRAINED BEFORE THE CLOCK IS STARTED, not after.
    // The lines printed above take tens of milliseconds to leave at
    // 115200, and a drain placed between arming the deadline and
    // stamping the wall would put all of them INSIDE the measurement -
    // which is the samc bench's own lesson about a print in a
    // measurement window, in a new dress. It cost this letter one round
    // of "the event matured early", which is the one verdict that must
    // never be wrong here.
    console_drain();
    const uint32_t w0 = wall();
    const uint32_t k0 = Ticker::millis();
    Probe::deadline.arm(500u);
    post<TimedManager>(SleepRequested{SleepDepth::deep,
                                      reply_to<Probe, SleepVote>()});
    pump_until_blip<TimedKernel>(3000u);
    const uint32_t to_blip = wall_ms(wall_delta(w0, Probe::blip_wall));
    const uint32_t kernel_ms = Ticker::millis() - k0;
    print(serial, "  a 500 ms event through a Stop matured after ", to_blip,
          " ms of wall (", kernel_ms, " ms of kernel tick); the resync put "
          "back ", TimedSite::last_advance(), " ticks", crlf);
    bench.verdict("THE EVENT MATURED THROUGH A STOP, on the wall",
                  Probe::blips == 1u && within(to_blip, 500u, 560u));
    bench.verdict("and NEVER EARLY - the lower bound is the kernel's own "
                  "promise",
                  to_blip >= 500u);
    bench.verdict("the sleep was real: the resync handed back a frozen span "
                  "of hundreds of ticks",
                  TimedSite::last_advance() > 350u &&
                      TimedSite::last_advance() < 520u);
    bench.verdict("and the round closed by the convention",
                  Probe::wakes >= 1u && TimedSite::armed() == SleepDepth::none);

    // NEVER EARLY, REPEATED - six shorter rounds, each judged on the
    // wall, because one round proves an arrangement and six prove a
    // rule.
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
        const uint32_t a = wall();
        Probe::deadline.arm(nominal);
        post<TimedManager>(SleepRequested{SleepDepth::deep,
                                          reply_to<Probe, SleepVote>()});
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
        if (within(ms, nominal, nominal + 20u)) {
            ++on_time;
        }
    }
    kernel_live = false;
    timed_round = false;
    print(serial, "  ", repeats, " rounds of ", nominal,
          " ms through a Stop: wall spans ", worst_lo, "..", worst_hi,
          " ms", crlf);
    bench.verdict("every one of six shorter rounds matured inside the band",
                  on_time == repeats);
    bench.verdict("and not one of them was early", worst_lo >= nominal);

    (void)wall_up();
}

// =============================================================================
// g - the entry preconditions, and ES0548 2.2.4
// =============================================================================
void tg_entry() {
    feed();
    timed_round = false;
    (void)wall_up();

    // A STOP THAT DOES NOT HAPPEN IS NOT AN ERROR ANYWHERE IN THE
    // SILICON. Table 31: with a wake-up flag standing, "the Stop mode
    // entry procedure is ignored and program execution continues" - and
    // 4.3.3 makes a WFI a no-op with any interrupt pending. So the only
    // honest way to judge a sleep is by TIME ELAPSED, which is what
    // every letter here does, and this is the demonstration.
    //
    // STAGING A PENDING INTERRUPT TAKES A MASK. Setting an enabled NVIC
    // line pending with interrupts on does not leave it pending - the
    // handler runs at once and clears it - so the pending bit is raised
    // INSIDE a critical section, where the core can see it and the
    // handler cannot run. On this core WFI still refuses to sleep on a
    // pending-but-masked interrupt, which is exactly the fact under
    // test.
    Nvic::clear_pending(Rtc::irq());
    Nvic::enable(Rtc::irq());
    (void)Rtc::set_wakeup(RtcWakeupClock::div16, 512, true);
    Nvic::clear_pending(Rtc::irq());
    console_drain();
    Ticker::pause();
    uint32_t fell_through = 0;
    {
        P::CriticalSection cs;
        Nvic::set_pending(Rtc::irq());
        const uint32_t t0 = wall();
        (void)Pwr::arm(PwrMode::stop1);
        __DSB();
        __WFI();
        fell_through = wall_delta(t0, wall());
    }
    (void)Site::resume_clock();
    Ticker::resume();
    (void)Pwr::arm(PwrMode::sleep);
    Rtc::clear_wakeup();
    Nvic::disable(Rtc::irq());
    Nvic::clear_pending(Rtc::irq());
    print(serial, "  a WFI with the wake already pending spent ",
          wall_us(fell_through), " us where 250000 was armed - the Stop did "
          "not happen at all", crlf);
    bench.verdict("a pending interrupt makes a Stop FALL THROUGH in "
                  "microseconds, silently and by design (4.3.3, table 31)",
                  wall_us(fell_through) < 20'000u);

    // ES0548 2.2.4, STAGED. "With HSIDIV[2:0] set to a value different
    // from 000, peripherals with clock request capability fail to wake
    // the device up from Stop modes." The RTC is NOT one of them - it
    // wakes through the internal wake-up line and EXTI 19, and asks for
    // no HSI16 - so the prediction is that a divided clock changes
    // nothing here, and the erratum's reach is the USARTs and the I2Cs.
    const bool quiet_at_zero = !Pwr::stop_hsidiv_hazard();

    // NOTHING MAY BE PRINTED WHILE THE CLOCK IS DIVIDED: the console's
    // divisor was computed for 64 MHz and a USART fed a quarter of that
    // sends a quarter of the baud rate. The ring is emptied first, every
    // reading is kept in a variable, and the whole leg reports itself
    // once the clock is back.
    console_drain();
    Rcc::sysclk_select(SysclkSource::hsisys);
    (void)Rcc::sysclk_wait(SysclkSource::hsisys);
    Rcc::hsi_div(2);   // HSISYS = 4 MHz
    const bool noisy_when_divided = Pwr::stop_hsidiv_hazard();
    Nvic::clear_pending(Rtc::irq());
    Nvic::enable(Rtc::irq());
    const bool armed = Rtc::set_wakeup(RtcWakeupClock::div16, 512, true);
    Nvic::clear_pending(Rtc::irq());
    Ticker::pause();
    const uint32_t t1 = wall();
    (void)Pwr::arm(PwrMode::stop1);
    __DSB();
    __WFI();
    const uint32_t divided = wall_delta(t1, wall());
    Ticker::resume();
    (void)Pwr::arm(PwrMode::sleep);
    Rtc::clear_wakeup();
    Nvic::disable(Rtc::irq());
    Rcc::hsi_div(0);
    (void)SysClock::init();
    (void)Serial::init(clock, 115200);

    print(serial, "  with HSIDIV = /4 (SYSCLK 4 MHz), a Stop woken by the "
                  "RTC lasted ", wall_ms(divided), " ms - 250 was asked for",
          crlf);
    bench.verdict("the hazard predicate is quiet with HSIDIV at zero and "
                  "says so as soon as the divider moves",
                  quiet_at_zero && noisy_when_divided);
    bench.verdict("ES0548 2.2.4 DOES NOT REACH AN RTC WAKE: the erratum is "
                  "about peripherals that REQUEST HSI16 while stopped, and "
                  "the RTC asks for nothing - it wakes through the internal "
                  "line and EXTI 19",
                  armed && within(wall_ms(divided), 230u, 320u));
}

// =============================================================================
// h - the surface this desk cannot stage, and what it can still say
// =============================================================================
void th_declared() {
    feed();
    // THE PVD IS NOT STAGEABLE HERE and saying so is the honest half of
    // covering the chapter: VDD on this board is the ST-LINK's 3.3 V
    // regulator, there is no way to move it from inside the chip, and
    // every threshold this detector has sits BELOW it. What can be
    // measured is that the detector agrees with that.
    Pwr::pvd_enable(false);
    bench.verdict("the PVD's thresholds are refused when the hysteresis is "
                  "the wrong way round (4.2.2)",
                  !Pwr::pvd_config({.rising = PvdRising::v2_1,
                                    .falling = PvdFalling::v2_9}) &&
                      Pwr::pvd_config(PvdConfig{}));
    Pwr::pvd_enable(true);
    (void)delay_us(clock, 200);
    const bool below = Pwr::pvd_below();
    print(serial, "  PVD at 2.9/2.8 V: VDD reads ", below ? "BELOW" : "above",
          " the threshold (this board runs at 3.3 V from the ST-LINK, so "
          "'above' is the only answer the desk can produce)", crlf);
    bench.verdict("the detector says VDD is above its highest fixed "
                  "threshold, which is the only reading this supply can "
                  "give - the crossing itself is NOT stageable here",
                  !below);
    Pwr::pvd_enable(false);
    bench.verdict("and its EXTI line is a CONFIGURABLE one, so a sense has "
                  "to be chosen before anything is pending (table 65)",
                  Exti::configurable(Pwr::pvd_exti_line));

    // THE WAKE-UP PINS NEED A WIRE. What is reachable with none is the
    // register discipline: enabling one and the erratum's own
    // workaround, which this driver applies so a caller cannot meet it.
    Pwr::clear_wakeup_flags();
    const bool armed = Pwr::wakeup_pin(1, true, false);
    const bool flag_clean = !Pwr::wakeup_flag(1);
    (void)Pwr::wakeup_pin(1, false);
    print(serial, "  WKUP1 armed and disarmed; its flag after the "
                  "configuration was ", flag_clean ? "clear" : "SET", crlf);
    bench.verdict("a wake-up pin arms, and ES0548 2.2.2's spurious flag "
                  "cannot reach a caller: wakeup_pin() clears WUFx as part "
                  "of the configuration",
                  armed && flag_clean);
    bench.verdict("the internal wake-up line is enabled out of reset, which "
                  "is why an RTC alarm out of Standby needs nothing set "
                  "here (PWR_CR3 resets to 0x8000)",
                  Pwr::internal_wakeup());

    // The regulator's own bits, read but not driven: Low-power run is a
    // whole-program decision (4.3.2 wants the system clock below 2 MHz
    // first), and this suite runs at 64.
    print(serial, "  regulator: REGLPF=", Pwr::on_low_power_regulator(),
          " REGLPS=", Pwr::low_power_regulator_ready(), " LPR=",
          Pwr::low_power_run(), " FLASH_RDY=", Pwr::flash_ready(), crlf);
    bench.verdict("the core is supplied from the MAIN regulator, as a "
                  "64 MHz program must be",
                  !Pwr::on_low_power_regulator() && !Pwr::low_power_run());
    bench.verdict("and the flash is awake", Pwr::flash_ready());
}

// =============================================================================
// s / u - Standby and Shutdown (outside z: they come back through reset)
// =============================================================================
//
// THE RESUME CANNOT USE .noinit. Standby powers the VCORE domain off and
// SRAM with it (unless RRS is set, and Shutdown loses it either way), so
// the token has to live where the RTC does: a TAMP backup register,
// which is the only storage on this chip that survives both.

constexpr uint32_t token_magic = 0x51EE0000u;
constexpr uint8_t token_reg = 4;      ///< BKP4R: leg + tally
constexpr uint8_t token_reg2 = 3;     ///< BKP3R: the wall stamp at entry

void deep_leg(PwrMode mode, const char* name, uint8_t leg_id) {
    feed();
    timed_round = false;
    const uint32_t token = Tamp::backup(token_reg);
    if ((token & 0xFFFF0000u) == token_magic &&
        static_cast<uint8_t>(token >> 8) == leg_id) {
        // SECOND HALF: this is the boot that followed the sleep.
        (void)Tamp::backup(token_reg, 0);
        bench.resume_tally(static_cast<uint16_t>(token & 0xFFu), 0);
        print(serial, "  came back from ", name, "; RCC_CSR flags=",
              hex(boot_flags), " PWR_SR1 at boot=", hex(boot_sr1), crlf);
        // 4.3.8: "program execution restarts in the same way as after a
        // reset". So the evidence is not a return address - it is the
        // flags, and the calendar that never stopped.
        const bool sbf = (boot_sr1 & PWR_SR1_SBF) != 0u;
        bench.verdict("the machine came back THROUGH THE RESET VECTOR - "
                      "which is exactly why no rung of the ladder maps here",
                      true);
        // WHAT THE TWO MODES LOOK LIKE FROM THE OTHER SIDE, and the
        // answer is the same for both - which is not what chapter 4
        // invites a reader to expect.
        //
        // 4.4.5 makes PWR_SR1.SBF "set by hardware when the device
        // enters Standby mode", and 4.3.9 says a POWER-ON RESET occurs
        // on leaving Shutdown with "all registers ... reset". Measured
        // on this silicon: SBF STANDS AFTER BOTH, and RCC_CSR carries
        // NO RESET FLAG AT ALL after either - not PWRRSTF, not even the
        // catch-all PINRSTF that every ordinary reset of this board
        // raises. So neither register tells the two apart, and a
        // program that has to know must leave itself a note (this
        // letter's own backup register is that note).
        bench.verdict("PWR_SR1.SBF stands after the wake: the boot can tell "
                      "a DEEP-mode wake from a plain reset, which is what "
                      "4.4.5 promises",
                      sbf);
        bench.verdict("and RCC_CSR names NO reset source for it - not "
                      "PWRRSTF, not the catch-all PINRSTF - so a deep wake "
                      "is invisible to the reset chapter's own register",
                      (boot_flags & ResetFlag::all) == 0u);
        const uint32_t stamp = Tamp::backup(token_reg2);
        const uint32_t now = wall();
        const uint32_t slept = wall_delta(stamp, now);
        print(serial, "  the RTC never stopped: ", wall_ms(slept),
              " ms of wall between the sleep and this line", crlf);
        bench.verdict("THE RTC DOMAIN OUTLIVED THE SLEEP - the calendar "
                      "kept counting with the VCORE domain powered off",
                      Rtc::calendar_set() && RtcDomain::enabled() &&
                          within(wall_ms(slept), 200u, 4000u));
        bench.verdict("and the backup registers carried the resume across, "
                      "which is what a program without SRAM has to lean on",
                      true);
        Pwr::clear_wakeup_flags();
        bench.end_letter();
        return;
    }

    // FIRST HALF.
    if (!wall_up()) {
        bench.verdict("the wall clock is up", false);
        return;
    }
    bench.verdict("the RTC wake-up timer is set for the return trip",
                  Rtc::set_wakeup(RtcWakeupClock::ck_spre, 1, true));
    Nvic::enable(Rtc::irq());
    (void)Tamp::backup(token_reg2, wall());
    (void)Tamp::backup(token_reg,
                       token_magic | (static_cast<uint32_t>(leg_id) << 8) |
                           (bench.passed() & 0xFFu));
    print(serial, "  entering ", name,
          " now; the RTC wakes it, and the board reboots into the second "
          "half", crlf);
    console_drain();
    // The chapter's own preconditions (tables 33 and 34): the wake-up
    // flags clear, and the RTC flag that will do the waking clear too.
    Rtc::clear_flags(RtcFlag::wakeup);
    Pwr::enter(mode);
    // Falling through means the entry conditions were not met - which is
    // silent in the silicon and must not be silent here.
    print(serial, "  the entry did NOT happen (a flag was standing)", crlf);
    bench.verdict("the machine entered the mode", false);
    (void)Tamp::backup(token_reg, 0);
}

void ts_standby() { deep_leg(PwrMode::standby, "Standby", 1); }
void tu_shutdown() { deep_leg(PwrMode::shutdown, "Shutdown", 2); }

// ---------------------------------------------------------------------------
// The menu
// ---------------------------------------------------------------------------

void banner() {
    print(serial, crlf,
          "test_stm32_sleep - PWR and the two sleep sites (board E, no wires)",
          crlf);
    bench.menu();
    print(serial, "  z  run them all", crlf);
}

}   // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void USART2_LPUART2_IRQHandler() { (void)Serial::isr(); }
extern "C" void RTC_TAMP_IRQHandler() {
    rtc_wakes = rtc_wakes + 1u;
    if (timed_round) {
        // The four-act body: restore the clock, acknowledge, resync,
        // and hand the machine back to a ticking sleep.
        TimedSite::isr();
    } else {
        (void)brio::Rtc::isr();
        if (kernel_live) {
            brio::post<Probe>(Woke{});
        }
    }
}

int main() {
    const bool clock_ok = SysClock::init();
    boot_flags = brio::Reset::take_flags();
    brio::Pwr::bus_clock(true);
    boot_sr1 = brio::Pwr::sr1();
    brio::Pwr::rtc_domain_unlock(true);
    brio::RtcDomain::apb_clock(true);

    // THE RTC DOMAIN, ONCE, BEFORE ANY LETTER: this suite's wall clock
    // is the crystal, and RTCSEL is one-way, so a domain that came up on
    // something else has to be wiped first. That costs the backup
    // registers, which is why it happens here and not inside a letter
    // that has just written one.
    if (brio::RtcDomain::selected() != brio::RtcClockSource::lse) {
        brio::RtcDomain::reset();
    }
    brio::RtcDomain::lse_enable(true);
    lse_ok = brio::RtcDomain::lse_wait_ready(4'000'000UL);
    if (lse_ok) {
        (void)brio::RtcDomain::open(brio::RtcClockSource::lse);
        brio::Rtc::bypass_shadow(true);
        wall_ready = wall_up();
    }

    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    const bool wd = arm_backstop();
    brio::enable_interrupts();

    bench.letter('a', "the block and the ladder this target chose", ta_ladder);
    bench.letter('b', "Sleep: the timebase keeps running", tb_sleep);
    bench.letter('c', "STOP: kernel time stands still, and SYSCLK changes",
                 tc_stop_freezes);
    bench.letter('d', "what each rung costs to leave", td_wake_cost);
    bench.letter('e', "a Stop through a real kernel, and the v1 restriction",
                 te_kernel_stop);
    bench.letter('f', "THE TIMED SITE: the deadline met on the wall",
                 tf_timed_site);
    bench.letter('g', "the entry preconditions, and ES0548 2.2.4",
                 tg_entry);
    bench.letter('h', "what this desk cannot stage, and what it still says",
                 th_declared);
    bench.letter('s', "STANDBY: off the ladder, back through the reset vector",
                 ts_standby, false);
    bench.letter('u', "SHUTDOWN: the same, one rung deeper", tu_shutdown,
                 false);

    // A boot that follows one of the two deep legs finishes it and stops.
    const uint32_t token = brio::Tamp::backup(token_reg);
    const bool resuming = (token & 0xFFFF0000u) == token_magic;
    if (resuming && serial_ok) {
        print(serial, crlf, "boot: resuming a deep-sleep letter", crlf);
        const uint8_t leg = static_cast<uint8_t>(token >> 8);
        if (leg == 1u) {
            ts_standby();
        } else {
            tu_shutdown();
        }
        bench.prompt();
    } else if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "PLL 64 MHz" : "FAILED",
              " tick=", tick_ok ? "SysTick" : "FAILED",
              " wall=", wall_ready ? "RTC on LSE" : "NO CRYSTAL",
              " backstop=", wd ? "IWDG 32 s" : "FAILED", crlf);
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
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        bench.prompt();
    }
}
