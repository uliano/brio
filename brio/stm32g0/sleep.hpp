/*
 * sleep.hpp
 *
 * util/power.hpp's depth ladder on STM32G0 silicon: `Stm32SleepSite`,
 * which arms one of this family's modes, and `Stm32TimedSleepSite`,
 * which additionally keeps KERNEL TIME HONEST across a Stop by putting
 * the RTC's wake-up timer where the deadline is and handing the frozen
 * span back to the ticker.
 *
 * The MECHANISM - which modes exist, what they gate, what wakes them -
 * is stm32g0/pwr.hpp's. The POLICY - when a program may stop - is
 * util/power.hpp's. What is HERE is the mapping between them, and the
 * mapping is this target's own decision.
 *
 * ## The ladder, and why it is not the identity
 *
 *   none    -> Sleep      SLEEPDEEP = 0: the CPU clock stops, HCLK,
 *                         SysTick and every peripheral keep running
 *   light   -> Sleep      THE SAME MODE (see below)
 *   standby -> Stop 0     SLEEPDEEP = 1, LPMS = 000: every VCORE clock
 *                         stops, SRAM and registers are retained, the
 *                         main regulator stays on for the fastest wake
 *   deep    -> Stop 1     SLEEPDEEP = 1, LPMS = 001: the same, on the
 *                         low-power regulator - deeper and slower to
 *                         leave
 *
 * THREE THINGS TO READ BEFORE USING IT.
 *
 * 1. `light` AND `none` ARE ONE CODE, and that is the honest answer
 *    rather than a shortcut. util/power.hpp's rule is that a target
 *    "maps what it does not have to the nearest SHALLOWER mode - never
 *    deeper than asked". Between Sleep and Stop 0 this family has
 *    exactly one thing, Low-power SLEEP, and it is not a rung a sleep
 *    site may take: 4.3.5 reaches it only from Low-power RUN, which
 *    means the regulator in low-power mode and the system clock at or
 *    below 2 MHz - a whole-program decision that an application makes,
 *    not something to do behind its back for the duration of one idle.
 *    So `light` maps to Sleep, and `armed()` - which is a PURE READ of
 *    the silicon, the samc position kept - answers `none` for it,
 *    because that is what the machine will really do. The manager's
 *    WakeReport then carries `none`, which is the truth.
 *
 * 2. STANDBY AND SHUTDOWN ARE OFF THE LADDER ON PURPOSE, and this is
 *    the first target where a mode the silicon has is deliberately not
 *    a rung. util/power.hpp's model is built on the program RESUMING:
 *    "the manager's next dispatch - of ANY event - first disarms the
 *    site and publishes a WakeReport". After this family's Standby or
 *    Shutdown there is no next dispatch - 4.3.8: "program execution
 *    restarts in the same way as after a reset (boot pin sampling,
 *    option bytes loading, reset vector is fetched)". A site that armed
 *    one would leave a manager waiting for a wake that arrives as a
 *    reboot, so no rung maps there. The two modes are reachable, as
 *    they should be, through `Pwr::enter(PwrMode::standby)` - a
 *    deliberate one-shot, whose resumption is the application's boot
 *    path reading PWR_SR1.SBF and the TAMP backup registers.
 *
 * 3. A STOP ENTERED WITH THE KERNEL'S TICK ARMED DOES NOT LAST, and
 *    this is the fact that most changes what a site has to do here.
 *    4.3.3: entering a low-power mode through WFI "is executed only if
 *    no interrupt is pending", and a 1 kHz SysTick is a pending
 *    interrupt every millisecond. MEASURED, and reproducibly: a 250 ms
 *    Stop 1 asked for with the tick armed lasts 0..3 ms; the same one
 *    with SysTick's interrupt paused lasts 250 ms to the RTC's own tick,
 *    three runs out of three (docs/stm32g0/pwr.md). So `arm()` pauses
 *    the ticker for the deep rungs and `disarm()` resumes it - which
 *    costs NOTHING, because a Stop stops SysTick anyway and kernel time
 *    was going to stand still for the whole sleep either way. It is the
 *    difference between a sleep that happens and a sleep that is a coin
 *    toss.
 *
 * 4. WHAT COMES BACK FROM A STOP IS NOT WHAT WENT IN. 4.3.6 and 5.3:
 *    the system clock on exit is HSISYS and the PLL is off, so a
 *    program running at 64 MHz resumes at 16 MHz with a SysTick reload
 *    and a USART divisor meant for four times that. The site is
 *    therefore TEMPLATED ON THE CLOCK TASK and restores it - and the
 *    place it does so is the model's own hook, not a new one: arm() is
 *    called before the machine stops and disarm() on the FIRST EVENT
 *    AFTER THE WAKE, which is exactly "put the clock back". A program
 *    on `ClockSource::internal` pays nothing at all: HSIDIV survives a
 *    Stop, so SWS already reads what the task asked for and
 *    `resume_clock()` finds nothing to do.
 *
 * ## The timed site
 *
 * The plain site keeps the v1 HONEST RESTRICTION the samc stated: with
 * kernel time frozen for the whole Stop, a program with armed time
 * events must not take one. `Stm32TimedSleepSite` LIFTS it, by the same
 * two-verb trick that worked on the SAM and with the RTC in both roles:
 *
 *  - the ALARM is the periodic wake-up timer (30.3.7), placed on
 *    TimeEvents<P>::ticks_to_next() rounded UP. 4.3.10 calls this the
 *    device's auto-wake-up and it is the one counter on this family
 *    that runs with every VCORE clock stopped;
 *  - the WITNESS is the calendar plus the sub-second counter, read
 *    through `Rtc::time_of_hour_ms()`; the difference between the
 *    reading at arm() and the reading at disarm() is how long the world
 *    moved, and subtracting what SysTick itself counted leaves the
 *    FROZEN span, which goes to `Ticker::advance()`.
 *
 * THE RATE RULE IS DIRECTIONAL, and on this target BOTH halves want the
 * same direction: state an RTCCLK rate NOT BELOW the true one.
 * Over-estimating it makes the prescalers divide too hard, so ck_spre
 * runs slow, so the witness UNDER-reports the span and the resync
 * UNDER-advances; and it makes the alarm arithmetic ask for more
 * counts than needed, so the wake lands LATE. Both errors land on the
 * side the kernel's time contract allows: at least, never early. The
 * default over-estimates LSI, which this family specifies at 32 kHz
 * nominal and DS13560 bounds at 29.5..34 kHz; a board that has measured
 * its own says so in the config, and gets the slack back.
 *
 * THE ISR HAS FOUR ACTS, and every one of them is load-bearing. The
 * first three are the samc's, learned at that bench; the fourth is this
 * family's own:
 *   0. RESTORE THE CLOCK - fact 4 above. First, so that everything
 *      after it (and every handler that runs later) is at full speed
 *      and the console's divisor is right again.
 *   1. ACKNOWLEDGE - the wake-up flag, or the interrupt re-enters.
 *   2. RESYNC - hand the frozen span to the ticker.
 *   3. HAND THE MACHINE BACK TO A TICKING SLEEP. The never-early bias
 *      GUARANTEES that kernel time is still a shade short of the
 *      deadline when the alarm lands, and an RTC wake posts nothing to
 *      any queue, so a loop that idled again into the still-armed Stop
 *      with the alarm now spent would never come back. Downgrading to
 *      Sleep here is the escape: the residual ticks mature on SysTick
 *      within milliseconds and TimeEvents::process() posts the deadline
 *      on its own clock.
 *
 * A FOREIGN wake - any other interrupt - does not run this body. Its
 * progress is its own event, the resync happens in disarm() instead,
 * and the Stop stays armed WITH the alarm still standing. That is why
 * util/power.hpp's convention (a wake path with nothing to say sends
 * SleepRequested{none}) is LOAD-BEARING with this site, exactly as it
 * is with the SAM's.
 *
 * ## What the site owns
 *
 * THE RTC, WHOLE: the domain gate, the clock select, the prescalers,
 * the calendar and the wake-up timer. An application using this site
 * must not drive stm32g0/rtc.hpp elsewhere, and must bind the vector:
 *
 *     extern "C" void RTC_TAMP_IRQHandler() { Site::isr(); }
 *
 * ## Errata
 *
 * ES0548 2.2.4 (revision Z, no workaround): with RCC_CR.HSIDIV nonzero,
 * peripherals with clock-REQUEST capability cannot wake the device from
 * Stop. The RTC is not one of them - it wakes through the internal
 * wake-up line and EXTI 19, neither of which asks for HSI16 - so this
 * site is not refused on a divided clock; `Pwr::stop_hsidiv_hazard()`
 * is the predicate for an application whose wake is a USART or an I2C,
 * and docs/stm32g0/pwr.md says which is which.
 */

#pragma once

#include <stdint.h>

#include <optional>

#include "kernel/platform.hpp"
#include "kernel/time_event.hpp"
#include "stm32g0/clock.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pwr.hpp"
#include "stm32g0/rtc.hpp"
#include "stm32g0/ticker.hpp"
#include "util/power.hpp"

namespace brio {

/// The SysclkSource a Clock task's root ends up in RCC_CFGR.SWS as -
/// what `resume_clock()` compares against to decide whether a Stop
/// really happened.
template <class C>
constexpr SysclkSource sysclk_source_of() {
    return C::source == ClockSource::pll ? SysclkSource::pllrclk
                                         : SysclkSource::hsisys;
}

/**
 * The plain site: arm a mode, say what is armed, put it back.
 *
 * `C` is the application's Clock task (stm32g0/clock.hpp) - not for the
 * rate, but because a Stop drops SYSCLK to HSISYS and something has to
 * put it back (fact 4 in this file's header).
 */
template <class C>
struct Stm32SleepSite {
    Stm32SleepSite() = delete;

    static constexpr SysclkSource expected_source = sysclk_source_of<C>();

    /**
     * Re-establish the clock the task promised, if a Stop took it away.
     *
     * The test is the silicon's own: RCC_CFGR.SWS says what SYSCLK is
     * NOW, and a Stop leaves it on HSISYS. For a PLL-based clock that
     * is a mismatch and the task is re-run; for an HSISYS-based one
     * there is nothing to detect and nothing to do, because HSIDIV
     * survives a Stop - so a program on `internal` pays zero for this
     * call, every time.
     *
     * Returns true when the clock is the one `C` promises.
     */
    static bool resume_clock() {
        if (Rcc::sysclk_status() == expected_source) {
            return true;
        }
        return C::init();
    }

    /**
     * Arm a rung. For the two deep ones this ALSO pauses the kernel's
     * tick, which is fact 3 in this file's header and not an
     * optimization: a Stop entered with a millisecond interrupt armed
     * does not last. The pause costs nothing - a Stop stops SysTick
     * anyway - and `disarm()` puts it back.
     */
    static bool arm(SleepDepth d) {
        switch (d) {
            case SleepDepth::none:
            case SleepDepth::light:
                Ticker::resume();
                return Pwr::arm(PwrMode::sleep);
            case SleepDepth::standby:
                if (!Pwr::arm(PwrMode::stop0)) {
                    return false;
                }
                Ticker::pause();
                return true;
            case SleepDepth::deep:
                if (!Pwr::arm(PwrMode::stop1)) {
                    return false;
                }
                Ticker::pause();
                return true;
        }
        return false;
    }

    /// Back to the kernel's own idle behaviour: the tick running again,
    /// the shallow mode armed, and the clock the program was promised -
    /// because the first thing that reaches the manager after a Stop is
    /// the first thing that can put any of the three back.
    static void disarm() {
        (void)Pwr::arm(PwrMode::sleep);
        (void)resume_clock();
        Ticker::resume();
    }

    /// A pure read of SLEEPDEEP and LPMS. `light` is never reported: it
    /// maps to Sleep, and Sleep is `none` (fact 1).
    static SleepDepth armed() {
        switch (Pwr::mode()) {
            case PwrMode::sleep:
                return SleepDepth::none;
            case PwrMode::stop0:
                return SleepDepth::standby;
            case PwrMode::stop1:
                return SleepDepth::deep;
            case PwrMode::standby:
            case PwrMode::shutdown:
                // Not this site's doing - somebody armed one by hand
                // through Pwr. Report the deepest rung: whatever else it
                // is, it is not shallower than `deep`.
                return SleepDepth::deep;
        }
        return SleepDepth::none;
    }
};

// ---- the timed site ---------------------------------------------------------

/**
 * Stm32TimedSleepSite's knobs.
 *
 * `rtcclk_hz` is the rate of the clock the RTC counts AND THE RULE IS
 * DIRECTIONAL: give a value NOT BELOW the true rate (see the file
 * header). The default over-estimates LSI on purpose. `source` is what
 * RTCSEL is asked for; `wipe_domain` says whether init() may reset the
 * RTC domain to get there - which it must on a board whose domain came
 * up on a different source, and which COSTS THE BACKUP REGISTERS.
 */
struct TimedSleepConfig {
    uint32_t rtcclk_hz = 33'000;
    RtcClockSource source = RtcClockSource::lsi;
    bool wipe_domain = false;
    /// The wake-up timer's fast clock. div16 gives the widest span at
    /// this family's 32 kHz-ish rates (about 31 s) with half-millisecond
    /// resolution; a deadline past that is placed on ck_spre instead,
    /// automatically.
    RtcWakeupClock fast_clock = RtcWakeupClock::div16;
};

constexpr bool timed_sleep_config_valid(const TimedSleepConfig& c) {
    // Below 1 kHz the resync granularity is coarser than a kernel tick.
    if (c.rtcclk_hz < 1024u) {
        return false;
    }
    // The prescaler pair has to exist, or ck_spre is not 1 Hz and every
    // conversion in this file is wrong...
    const RtcPrescalers p = rtc_prescalers_for_resolution(c.rtcclk_hz);
    if (p.async == 0xFFu) {
        return false;
    }
    // ...and it has to divide a second at least a thousand ways, or the
    // resync's own granularity is coarser than the tick it repairs and
    // an event can mature early.
    if (static_cast<uint32_t>(p.sync) + 1u < 1000u) {
        return false;
    }
    // ck_spre is the long alarm's clock; the short one must be a real
    // divider.
    return rtc_wakeup_divider(c.fast_clock) != 0u;
}

/**
 * The sleep site that LIFTS the Stop restriction: kernel time no longer
 * stands still on the wall across a Stop, and a program with ARMED TIME
 * EVENTS may stop its clocks and still meet them.
 *
 * The power MODEL is untouched - no new concept member, no new manager
 * hook, no change to util/power.hpp - because everything the lift needs
 * fits inside the two verbs the SleepSite concept always had. That is
 * the second target on which that has held; docs/design/power.md
 * records it.
 */
template <Platform P, class C, TimedSleepConfig cfg = TimedSleepConfig{}>
struct Stm32TimedSleepSite {
    static_assert(timed_sleep_config_valid(cfg),
                  "brio Stm32TimedSleepSite: the RTC rate must be at least "
                  "1024 Hz; it must admit an exact prescaler pair for a 1 Hz "
                  "ck_spre whose SYNCHRONOUS factor divides the second at "
                  "least a thousand ways (the resync's granularity must be "
                  "finer than the kernel tick it repairs); and the fast "
                  "wake-up clock must be one of the divided-RTCCLK codes");

    Stm32TimedSleepSite() = delete;

    using Plain = Stm32SleepSite<C>;

    /// THE OTHER PRESCALER SPLIT, and the choice is the site's whole
    /// resolution. 30.3.4's advice - a high asynchronous factor, to save
    /// current - is right for a calendar and wrong here: PREDIV_S is how
    /// finely the sub-second counter divides a second, and that counter
    /// is this site's ONLY way of measuring a span the kernel's tick did
    /// not count. Taking the chapter's default at 32.8 kHz would give
    /// 328 steps a second - THREE MILLISECONDS a step, three times the
    /// kernel tick - and a resync quantized that coarsely can advance a
    /// tick too many and mature an event EARLY, which is the one thing
    /// brio's time contract forbids. (Measured, and it did: a 150 ms
    /// deadline came back at 149 before this line was what it is.)
    static constexpr RtcPrescalers prescalers =
        rtc_prescalers_for_resolution(cfg.rtcclk_hz);

    /// ck_wut for the fast code, rounded UP - the direction that makes a
    /// placed alarm land late rather than early.
    static constexpr uint32_t fast_hz =
        (cfg.rtcclk_hz + rtc_wakeup_divider(cfg.fast_clock) - 1u) /
        rtc_wakeup_divider(cfg.fast_clock);

    /// The longest deadline the fast clock can hold, in kernel ticks.
    static constexpr uint32_t fast_span_ticks =
        static_cast<uint32_t>((static_cast<uint64_t>(0xFFFFu) *
                               P::ticks_per_second) /
                              fast_hz);

    /**
     * Route the RTC, start the counter, set the calendar going, bypass
     * the shadow registers and enable the NVIC line. Call once, after
     * the clock init, before the manager's first round.
     *
     * False = the domain or the RTC refused (a clock select already
     * taken by something else and `wipe_domain` not given, an oscillator
     * that never reported ready, a synchronization that never settled).
     * The site still works as a plain Stm32SleepSite in that case, minus
     * every timed property - which is why the caller must look at the
     * answer.
     *
     * WHY BYPSHAD: after a Stop the shadow registers are not updated and
     * RSF has to be cleared and awaited before a shadow read means
     * anything (30.3.5). Reading the counters directly deletes that step
     * from the wake path, at the price of the double-read this driver's
     * read() already performs.
     */
    static bool init() {
        Pwr::bus_clock(true);
        if (!RtcDomain::open(cfg.source, cfg.wipe_domain)) {
            return false;
        }
        if (cfg.source == RtcClockSource::lse) {
            RtcDomain::lse_enable(true);
            if (!RtcDomain::lse_wait_ready()) {
                return false;
            }
        } else if (cfg.source == RtcClockSource::lsi) {
            Rcc::lsi_enable(true);
            if (!Rcc::lsi_wait_ready()) {
                return false;
            }
        }
        Rtc::bypass_shadow(true);
        // A calendar that already runs is left alone: its epoch is
        // nobody's business here (only DIFFERENCES are read), and
        // stopping it would cost the very thing a surviving RTC is for.
        if (!Rtc::calendar_set() || Rtc::prescalers().sync != prescalers.sync ||
            Rtc::prescalers().async != prescalers.async) {
            if (!Rtc::init(prescalers, RtcDateTime{.hour = 0,
                                                   .minute = 0,
                                                   .second = 0,
                                                   .day = 1,
                                                   .month = 1,
                                                   .year = 1,
                                                   .weekday = 1})) {
                return false;
            }
        }
        Rtc::clear_wakeup();
        (void)Rtc::wake_line_open();
        Nvic::enable(Rtc::irq());
        ready_ = true;
        return true;
    }

    static bool ready() { return ready_; }
    static bool alarm_armed() { return alarm_armed_; }
    /// Ticks the last resync handed to Ticker::advance() (0 when the
    /// round never slept, or slept less than it stayed awake).
    static uint32_t last_advance() { return last_advance_; }
    /// The reload the last placed alarm was given, and on which clock -
    /// diagnostics, and what a suite checks the arithmetic against.
    static uint32_t last_reload() { return last_reload_; }
    static bool last_alarm_was_fast() { return last_fast_; }

    /**
     * Place the alarm for a deadline `ticks` kernel ticks away. Public
     * because the arithmetic is worth being able to check without
     * arming a sleep; arm() calls it.
     *
     * The FAST clock is used while the deadline fits its 16-bit reload,
     * and ck_spre (one second per count) beyond that. Both conversions
     * round UP. False = the RTC refused the programming.
     */
    static bool place_alarm(uint32_t ticks) {
        if (ticks <= fast_span_ticks) {
            const uint32_t counts = static_cast<uint32_t>(
                (static_cast<uint64_t>(ticks) * fast_hz + P::ticks_per_second -
                 1u) /
                P::ticks_per_second);
            last_reload_ = counts;
            last_fast_ = true;
            return Rtc::set_wakeup(cfg.fast_clock, counts);
        }
        const uint32_t seconds =
            (ticks + P::ticks_per_second - 1u) / P::ticks_per_second;
        last_reload_ = seconds > 0xFFFFu ? 0xFFFFu : seconds;
        last_fast_ = false;
        return Rtc::set_wakeup(RtcWakeupClock::ck_spre, last_reload_);
    }

    static bool arm(SleepDepth d) {
        if (!Plain::arm(d)) {
            return false;
        }
        if (!ready_ || !is_deep_mode(Plain::armed())) {
            return true;   // Sleep keeps SysTick: nothing to compensate
        }
        rtc_at_arm_ = Rtc::time_of_hour_ms();
        tick_at_arm_ = Ticker::ticks();
        resync_armed_ = rtc_at_arm_ != 0xFFFFFFFFu;
        const std::optional<uint32_t> next = TimeEvents<P>::ticks_to_next();
        if (next.has_value() && place_alarm(*next)) {
            alarm_armed_ = true;
        }
        return true;
    }

    static void disarm() {
        Plain::disarm();
        if (alarm_armed_) {
            Rtc::clear_wakeup();
            alarm_armed_ = false;
        }
        resync();
    }

    static SleepDepth armed() { return Plain::armed(); }

    /**
     * Catch kernel time up by the FROZEN span: the RTC's own elapsed
     * milliseconds minus what SysTick itself counted since arm(). The
     * subtraction is what makes several naps inside one armed round, and
     * rounds that never slept at all, come out right with no special
     * case. The baseline is consumed EXACTLY ONCE, under the platform's
     * critical section, whichever of isr() and disarm() gets there first.
     */
    static void resync() {
        typename P::CriticalSection cs;
        if (!resync_armed_) {
            return;
        }
        resync_armed_ = false;
        const uint32_t now = Rtc::time_of_hour_ms();
        if (now == 0xFFFFFFFFu) {
            last_advance_ = 0;
            return;
        }
        const uint32_t wall_ms = Rtc::elapsed_ms(rtc_at_arm_, now);
        const uint32_t span = static_cast<uint32_t>(
            (static_cast<uint64_t>(wall_ms) * P::ticks_per_second) / 1000u);
        const uint32_t awake = Ticker::ticks() - tick_at_arm_;   // wrap-safe
        last_advance_ = span > awake ? span - awake : 0u;
        if (last_advance_ != 0u) {
            Ticker::advance(last_advance_);
        }
    }

    /// The four-act ISR body an app's RTC_TAMP_IRQHandler binds. See the
    /// file header for why each act is there.
    [[gnu::always_inline]] static void isr() {
        (void)Plain::resume_clock();   // act 0
        (void)Rtc::isr();              // act 1
        Rtc::clear_wakeup();
        alarm_armed_ = false;
        resync();                      // act 2
        (void)Pwr::arm(PwrMode::sleep);  // act 3
        Ticker::resume();              // ...and the tick fact 3 paused
    }

private:
    static inline bool ready_ = false;
    static inline bool alarm_armed_ = false;
    static inline bool resync_armed_ = false;
    static inline bool last_fast_ = true;
    static inline uint32_t rtc_at_arm_ = 0;
    static inline uint32_t tick_at_arm_ = 0;
    static inline uint32_t last_advance_ = 0;
    static inline uint32_t last_reload_ = 0;
};

} // namespace brio
