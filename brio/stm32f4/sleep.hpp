/*
 * sleep.hpp
 *
 * util/power.hpp's depth ladder on STM32F4 silicon: `Stm32f4SleepSite`,
 * which arms one of this family's modes and puts the clock tree back
 * afterwards, and `Stm32f4TimedSleepSite`, which additionally keeps
 * KERNEL TIME HONEST across a Stop by putting the RTC's wake-up timer
 * where the deadline is and handing the frozen span back to the SysTick
 * ticker.
 *
 * The MECHANISM - which modes exist, what they gate, what wakes them - is
 * stm32f4/pwr.hpp's. The POLICY - when a program may stop - is
 * util/power.hpp's. What is HERE is the mapping between them, and the
 * mapping is this target's own decision.
 *
 * ## The ladder, and why it is not the identity
 *
 *   none    -> Sleep      SLEEPDEEP = 0: the CPU clock stops, HCLK,
 *                         SysTick and every peripheral keep running
 *   light   -> Sleep      THE SAME MODE (see below)
 *   standby -> Stop       SLEEPDEEP = 1, PDDS = 0, the MAIN regulator:
 *                         every 1.2 V clock stops, SRAM and registers are
 *                         retained, and the wake is the fastest this
 *                         silicon has
 *   deep    -> Stop       the same mode with the LOW-POWER regulator and
 *                         the flash in power-down: cheaper to hold, and
 *                         the datasheet prices the difference in the wake
 *
 * FIVE THINGS TO READ BEFORE USING IT.
 *
 * 1. `light` AND `none` ARE ONE CODE, and that is the honest answer rather
 *    than a shortcut. util/power.hpp's rule is that a target "maps what it
 *    does not have to the nearest SHALLOWER mode - never deeper than
 *    asked". Between Sleep and Stop this family has NOTHING: 5.3 lists
 *    three modes and no low-power run. So `light` maps to Sleep, and
 *    `armed()` - which is a PURE READ of the silicon - answers `none` for
 *    it, because that is what the machine will really do. The manager's
 *    WakeReport then carries `none`, which is the truth.
 *
 * 2. THE TWO DEEP RUNGS ARE ONE MODE WITH TWO PRICES. This family's Stop
 *    is a single mode (SLEEPDEEP with PDDS clear) whose cost is set by
 *    LPDS, FPDS and the low-voltage pair - so the ladder's two deep rungs
 *    are two `StopConfig`s and not two modes, and `armed()` cannot tell
 *    them apart from the registers alone. It reports the depth THIS SITE
 *    last armed for a Stop, and `deep` for a Stop somebody else armed
 *    through `Pwr` (whatever else such a Stop is, it is not shallower).
 *    The two configurations are the site's template argument, so a program
 *    that wants under-drive on the parts that have it, or a plain
 *    main-regulator Stop for both rungs, says so where it names the site.
 *
 * 3. STANDBY IS OFF THE LADDER ON PURPOSE, AND THE SITE ENFORCES IT: every
 *    `arm()` here goes through `Pwr::arm(PwrMode::stop, ...)`, which
 *    CLEARS PDDS - so no rung can reach Standby and a PDDS left standing
 *    by anything else is taken down at the next round.
 *    util/power.hpp's model is built on the program RESUMING: "the
 *    manager's next dispatch - of ANY event - first disarms the site and
 *    publishes a WakeReport". After this family's Standby there is no next
 *    dispatch - 5.3.6: "SRAM and register contents are lost except for
 *    registers in the backup domain ... and Standby circuitry", and the
 *    wake is a reset. A site that armed one would leave a manager waiting
 *    for a wake that arrives as a reboot. Standby stays reachable, as it
 *    should be, through `Pwr::enter(PwrMode::standby)` - a deliberate
 *    one-shot whose resumption is the application's boot path reading
 *    PWR_CSR.SBF and the RTC's backup registers.
 *
 * 4. A STOP ENTERED WITH THE KERNEL'S TICK ARMED LASTS - ON A BOARD NO
 *    DEBUGGER HAS TOUCHED. 5.3.3: a low-power mode entered through WFI is
 *    entered only if no interrupt is pending, and a 1 kHz SysTick raises
 *    one every millisecond; but once the WFI is taken HCLK stops and
 *    SysTick with it, so that window is one instruction wide and not a
 *    coin toss. UNLESS DBGMCU_CR.DBG_STOP IS SET, in which case the debug
 *    logic feeds FCLK and HCLK from the internal RC through the Stop and
 *    ES0298 2.2.1 is live: "if the SysTick timer interrupt is enabled
 *    during the Stop mode debug ..., it wakes up the system from Stop
 *    mode". That bit survives every reset but a power-on and OpenOCD's own
 *    stm32f4x.cfg writes it at every connection. `arm()` therefore PAUSES
 *    THE TICKER for the deep rungs and `disarm()` resumes it: it costs
 *    NOTHING (a Stop stops SysTick anyway and kernel time was going to
 *    stand still for the whole sleep either way), it closes 5.3.3's
 *    pending-tick window by construction, and it is ES0298 2.2.1's
 *    workaround - "to debug the Stop mode, disable the SysTick timer
 *    interrupt" - applied whether or not a probe is attached.
 *    `Pwr::debug_in_stop()` reads the bit for a program that wants to say
 *    what it found.
 *
 * 5. WHAT COMES BACK FROM A STOP IS NOT WHAT WENT IN. 5.3.5: SYSCLK on
 *    exit is the HSI, the PLL and the HSE are off, the regulator is back
 *    at scale 3 and over-drive is disabled - so a program running at
 *    180 MHz resumes at 16 MHz with a SysTick reload and a USART divisor
 *    meant for eleven times that. The site is therefore TEMPLATED ON THE
 *    CLOCK TASK and restores it - and the place it does so is the model's
 *    own hook, not a new one: arm() is called before the machine stops and
 *    disarm() on the FIRST EVENT AFTER THE WAKE, which is exactly "put the
 *    clock back". A program on `ClockSource::hsi` pays nothing at all: SWS
 *    already reads what the task asked for and `resume_clock()` finds
 *    nothing to do.
 *
 * ## The timed site
 *
 * The plain site keeps an HONEST RESTRICTION: with kernel time frozen for
 * the whole Stop, a program with armed time events must not take one.
 * `Stm32f4TimedSleepSite` LIFTS it, inside arm() and disarm() alone, with
 * the RTC in both roles:
 *
 *  - the ALARM is the periodic wake-up timer (17.3.5), placed on
 *    TimeEvents<P>::ticks_to_next() rounded UP. 5.3.7 calls this the
 *    device's auto-wake-up and it is the one counter on this family that
 *    runs with every 1.2 V clock stopped;
 *  - the WITNESS is the calendar plus the sub-second counter, read through
 *    this file's `time_of_hour_ms()`; the difference between the reading
 *    at arm() and the reading at disarm() is how long the world moved, and
 *    subtracting what SysTick itself counted leaves the FROZEN span, which
 *    goes to `Ticker::advance()`.
 *
 * THE RATE RULE IS DIRECTIONAL, and both halves want the same direction:
 * state an RTCCLK rate NOT BELOW the true one. Over-estimating it makes
 * the prescalers divide too hard, so ck_spre runs slow, so the witness
 * UNDER-reports the span and the resync UNDER-advances; and it makes the
 * alarm arithmetic ask for more counts than needed, so the wake lands
 * LATE. Both errors land on the side the kernel's time contract allows: at
 * least, never early. The default is LSE's exact 32768; a board on the LSI
 * says so in the config and states a rate above DS10693's upper bound.
 *
 * THE OTHER PRESCALER SPLIT. 17.3.1 advises a HIGH asynchronous factor to
 * save current; that is right for a calendar and wrong here, because
 * PREDIV_S is how finely the sub-second counter divides a second and that
 * counter is this site's ONLY way of measuring a span the kernel's tick
 * did not count. `rtc_prescalers_for_resolution()` is the deliberate
 * opposite choice - 0/32767 at 32768 Hz, 30.5 us a step - and the config
 * predicate refuses a split that divides the second fewer than a thousand
 * ways, because a resync quantized more coarsely than the kernel tick it
 * repairs can advance a tick too many and mature an event EARLY.
 *
 * THE ISR HAS FOUR ACTS, and every one of them is load-bearing:
 *   0. RESTORE THE CLOCK - fact 5 above. First, so that everything after
 *      it (and every handler that runs later) is at full speed and the
 *      console's divisor is right again.
 *   1. ACKNOWLEDGE - the EXTI line's pending bit and the RTC's own flag,
 *      or the vector re-enters for ever (`Rtc::wakeup_isr()` does both,
 *      with ES0287 2.8.3's loop inside it).
 *   2. RESYNC - hand the frozen span to the ticker.
 *   3. HAND THE MACHINE BACK TO A TICKING SLEEP. The never-early bias
 *      GUARANTEES that kernel time is still a shade short of the deadline
 *      when the alarm lands, and an RTC wake posts nothing to any queue,
 *      so a loop that idled again into the still-armed Stop with the alarm
 *      now spent would never come back. Downgrading to Sleep here is the
 *      escape: the residual ticks mature on SysTick within milliseconds
 *      and TimeEvents::process() posts the deadline on its own clock.
 *
 * A FOREIGN wake - any other interrupt - does not run this body. Its
 * progress is its own event, the resync happens in disarm() instead, and
 * the Stop stays armed WITH the alarm still standing. That is why
 * util/power.hpp's convention (a wake path with nothing to say sends
 * SleepRequested{none}) is LOAD-BEARING with this site.
 *
 * ## What the site owns
 *
 * THE RTC, WHOLE: the domain gate, the clock select, the prescalers, the
 * calendar and the wake-up timer. An application using this site must not
 * drive stm32f4/rtc.hpp elsewhere, and must bind the vector:
 *
 *     extern "C" void RTC_WKUP_IRQHandler() { Site::isr(); }
 *
 * ## Errata
 *
 * ES0298 2.2.1 (ES0206 2.2.1, ES0287 2.2.1) is answered by the ticker
 * pause - fact 4. ES0298 2.2.4's wake-up-source dance belongs to Standby,
 * which no rung reaches; `Pwr::prepare_standby()` is where it lives. The
 * RTC's own errata - the shadow lock, the shared-line loss, the
 * consecutive initialization entries - are stm32f4/rtc.hpp's and are
 * already coded there; this file inherits them through its verbs.
 */

#pragma once

#include <stdint.h>

#include <optional>

#include "kernel/platform.hpp"
#include "kernel/time_event.hpp"
#include "stm32f4/clock.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/pwr.hpp"
#include "stm32f4/rtc.hpp"
#include "stm32f4/ticker.hpp"
#include "util/power.hpp"

namespace brio {

/**
 * The two Stop flavours the ladder's deep rungs take (fact 2).
 *
 * The defaults are the two ends of the datasheet's table 36: `standby` is
 * the main regulator with the flash awake - the fastest wake this silicon
 * has - and `deep` is the low-power regulator with the flash in
 * power-down, the cheapest to hold of the two that need no extra bits.
 * A program on a part with the under-drive field can name it here; the
 * validity predicate refuses it where the part has not got it, so the
 * refusal is a compile error at the site and not a silent no-op.
 */
struct SleepSiteConfig {
    StopConfig standby{};
    StopConfig deep{StopRegulator::low_power, true, false, false};
};

constexpr bool sleep_site_config_valid(const SleepSiteConfig& c) {
    return stop_config_valid(c.standby) && stop_config_valid(c.deep);
}

/**
 * The plain site: arm a mode, say what is armed, put the clock back.
 *
 * `C` is the application's Clock task or DynamicClock (stm32f4/clock.hpp) -
 * not for the rate, but because a Stop drops SYSCLK to the HSI and
 * something has to put it back (fact 5). `TB` is the program's kernel
 * timebase - the platform's template argument, `Ticker` by default - and
 * the one thing the site asks of it is whether it has a periodic interrupt
 * to pause across a Stop (fact 4); an `if constexpr` on the verbs'
 * presence is what would let one site serve a timebase without one, the
 * day this family has a tickless timebase to offer.
 */
template <class C, class TB = Ticker, SleepSiteConfig cfg = SleepSiteConfig{}>
struct Stm32f4SleepSite {
    static_assert(sleep_site_config_valid(cfg),
                  "brio Stm32f4SleepSite: a Stop configuration asks for the low-voltage "
                  "regulator mode or the under-drive field on a part that has neither, or "
                  "for under-drive without the low-voltage mode it modifies (RM0390 table "
                  "17)");

    Stm32f4SleepSite() = delete;

    /// True when the timebase runs a periodic interrupt the deep rungs
    /// pause - what `arm()` and `disarm()` decide on.
    static constexpr bool pauses_tick = requires {
        TB::pause();
        TB::resume();
    };

    /// The two Stop flavours, for a caller that wants to print them.
    static constexpr SleepSiteConfig config = cfg;

    /**
     * Re-establish the clock the task promised, if a Stop took it away.
     *
     * The test is the silicon's own: RCC_CFGR.SWS says what SYSCLK is NOW,
     * and a Stop leaves it on the HSI. For a PLL-based clock that is a
     * mismatch and the task is re-run - which re-enables the HSE, sets the
     * regulator scale, reconfigures and starts the PLL, re-enters
     * over-drive where the rate needs it, and switches. For an HSI-based
     * one there is nothing to detect and nothing to do, so a program on
     * `hsi` pays one register read. Under a DynamicClock the same test and
     * the same re-run are the clock's own restore(), for the rate IN FORCE
     * - the one the users were rebased to - and never the boot one.
     *
     * Returns true when the clock is the one `C` promises.
     */
    static bool resume_clock() {
        if constexpr (C::is_static) {
            if (Rcc::sysclk_status() == C::sysclk_source) {
                return true;
            }
            return C::init();
        } else {
            return C::restore();
        }
    }

    /**
     * Arm a rung. For the two deep ones this ALSO pauses the kernel's tick
     * where there is one - fact 4: it closes 5.3.3's pending-interrupt
     * window by construction and makes the Stop last even under a
     * debugger's DBG_STOP, and it costs nothing because a Stop stops
     * SysTick anyway. `disarm()` puts it back.
     */
    static bool arm(SleepDepth d) {
        switch (d) {
            case SleepDepth::none:
            case SleepDepth::light:
                armed_ = SleepDepth::none;
                if constexpr (pauses_tick) {
                    TB::resume();
                }
                return Pwr::arm(PwrMode::sleep);
            case SleepDepth::standby:
            case SleepDepth::deep: {
                const StopConfig& sc = d == SleepDepth::deep ? cfg.deep : cfg.standby;
                if (!Pwr::arm(PwrMode::stop, sc)) {
                    return false;
                }
                armed_ = d;
                if constexpr (pauses_tick) {
                    TB::pause();
                }
                return true;
            }
        }
        return false;
    }

    /// Back to the kernel's own idle behaviour: the tick running again
    /// (where there is one), the shallow mode armed, and the clock the
    /// program was promised - because the first thing that reaches the
    /// manager after a Stop is the first thing that can put any of the
    /// three back.
    static void disarm() {
        (void)Pwr::arm(PwrMode::sleep);
        armed_ = SleepDepth::none;
        (void)resume_clock();
        if constexpr (pauses_tick) {
            TB::resume();
        }
    }

    /**
     * What a following idle path would take. SLEEPDEEP and PDDS are read
     * from the silicon; which of the two DEEP RUNGS a Stop is cannot be
     * (fact 2), so the mirror this site wrote at arm() answers that half -
     * and a Stop this site did not arm reports `deep`, the rung that
     * promises least.
     */
    static SleepDepth armed() {
        switch (Pwr::mode()) {
            case PwrMode::sleep:
                return SleepDepth::none;
            case PwrMode::stop:
                return is_deep_mode(armed_) ? armed_ : SleepDepth::deep;
            case PwrMode::standby:
                // Not this site's doing - somebody armed one by hand
                // through Pwr, and the next arm() will take PDDS down.
                return SleepDepth::deep;
        }
        return SleepDepth::none;
    }

private:
    static inline SleepDepth armed_ = SleepDepth::none;
};

// ---- the timed site ---------------------------------------------------------

/**
 * Stm32f4TimedSleepSite's knobs.
 *
 * `rtcclk_hz` is the rate of the clock the RTC counts AND THE RULE IS
 * DIRECTIONAL: give a value NOT BELOW the true rate (see the file header).
 * `source` is what RTCSEL is asked for; `wipe_domain` says whether init()
 * may reset the RTC domain to get there - which it must on a board whose
 * domain came up on a different source, RTCSEL being one-way, and which
 * COSTS THE TWENTY BACKUP REGISTERS.
 */
struct TimedSleepConfig {
    uint32_t rtcclk_hz = 32'768;
    RtcClockSource source = RtcClockSource::lse;
    bool wipe_domain = false;
    /// The wake-up timer's fast clock. div16 gives the widest span at this
    /// family's 32 kHz-ish rates (about 32 s) with half-millisecond
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
    // resync's own granularity is coarser than the tick it repairs and an
    // event can mature early.
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
 * fits inside the two verbs the SleepSite concept has;
 * docs/design/power.md records that.
 */
template <Platform P, class C, TimedSleepConfig cfg = TimedSleepConfig{},
          SleepSiteConfig site_cfg = SleepSiteConfig{}>
struct Stm32f4TimedSleepSite {
    static_assert(timed_sleep_config_valid(cfg),
                  "brio Stm32f4TimedSleepSite: the RTC rate must be at least 1024 Hz; it "
                  "must admit an exact prescaler pair for a 1 Hz ck_spre whose SYNCHRONOUS "
                  "factor divides the second at least a thousand ways (the resync's "
                  "granularity must be finer than the kernel tick it repairs); and the "
                  "fast wake-up clock must be one of the divided-RTCCLK codes");

    static_assert(requires {
                      P::Timebase::advance(0u);
                      P::Timebase::pause();
                      P::Timebase::resume();
                  },
                  "brio Stm32f4TimedSleepSite: a timed site repairs a timebase that STOPS "
                  "in Stop (the SysTick ticker, paused, resynchronized through advance()); "
                  "this platform's timebase has no such verbs");

    Stm32f4TimedSleepSite() = delete;

    using Plain = Stm32f4SleepSite<C, typename P::Timebase, site_cfg>;

    /// THE OTHER PRESCALER SPLIT - the file header says why the chapter's
    /// own advice is the wrong one here.
    static constexpr RtcPrescalers prescalers = rtc_prescalers_for_resolution(cfg.rtcclk_hz);

    /// ck_wut for the fast code, rounded UP - the direction that makes a
    /// placed alarm land late rather than early.
    static constexpr uint32_t fast_hz =
        (cfg.rtcclk_hz + rtc_wakeup_divider(cfg.fast_clock) - 1u) /
        rtc_wakeup_divider(cfg.fast_clock);

    /// The longest deadline the fast clock can hold, in kernel ticks.
    static constexpr uint32_t fast_span_ticks = static_cast<uint32_t>(
        (static_cast<uint64_t>(0xFFFFu) * P::ticks_per_second) / fast_hz);

    /**
     * Route the RTC, start the counter, set the calendar going, bypass the
     * shadow registers and enable the NVIC line. Call once, after the
     * clock init, before the manager's first round.
     *
     * False = the domain or the RTC refused (a clock select already taken
     * by something else and `wipe_domain` not given, an oscillator that
     * never reported ready, a synchronization that never settled). The
     * site still works as a plain Stm32f4SleepSite in that case, minus
     * every timed property - which is why the caller must look at the
     * answer.
     *
     * WHY BYPSHAD: after a Stop the shadow registers are not updated and
     * RSF has to be cleared and awaited before a shadow read means
     * anything (17.3.2, 17.3.6). Reading the counters directly deletes
     * that step from the wake path, at the price of the double-read this
     * driver's read() already performs in that mode.
     */
    static bool init() {
        Pwr::bus_clock(true);
        if (!RtcDomain::open(cfg.source, cfg.wipe_domain)) {
            return false;
        }
        // open() routes the clock and does not START it: which oscillator
        // runs is the application's, and here the config says which.
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
        // A calendar that already runs on the split this site needs is
        // left alone: its epoch is nobody's business here (only
        // DIFFERENCES are read), and stopping it would cost the very thing
        // a surviving RTC is for.
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
        Nvic::enable(Rtc::wakeup_irq());
        ready_ = true;
        return true;
    }

    static bool ready() { return ready_; }
    static bool alarm_armed() { return alarm_armed_; }
    /// Ticks the last resync handed to Timebase::advance() (0 when the
    /// round never slept, or slept less than it stayed awake).
    static uint32_t last_advance() { return last_advance_; }
    /// The reload the last placed alarm was given, and on which clock -
    /// diagnostics, and what a suite checks the arithmetic against.
    static uint32_t last_reload() { return last_reload_; }
    static bool last_alarm_was_fast() { return last_fast_; }

    /// Milliseconds since the top of the hour, on the RTC's own calendar
    /// and sub-second counter - the WITNESS. 0xFFFFFFFF when the reading
    /// could not be made coherent (a stopped RTCCLK).
    static uint32_t time_of_hour_ms() {
        RtcReading r{};
        if (!Rtc::read(r)) {
            return 0xFFFFFFFFu;
        }
        const uint16_t sync = Rtc::prescalers().sync;
        return (static_cast<uint32_t>(r.time.minute) * 60u +
                static_cast<uint32_t>(r.time.second)) *
                   1000u +
               rtc_subsecond_ms(r.subsecond, sync);
    }

    /// The difference of two such readings, over the hour's wrap.
    static uint32_t elapsed_ms(uint32_t from, uint32_t to) {
        constexpr uint32_t hour_ms = 3'600'000u;
        return to >= from ? to - from : hour_ms - from + to;
    }

    /**
     * Place the alarm for a deadline `ticks` kernel ticks away. Public
     * because the arithmetic is worth being able to check without arming a
     * sleep; arm() calls it.
     *
     * The FAST clock is used while the deadline fits its 16-bit reload,
     * and ck_spre (one second per count) beyond that. Both conversions
     * round UP. False = the RTC refused the programming.
     *
     * 17.6.6: the flag comes every reload + 1 counts, so the reload asked
     * for is one less than the counts wanted - and never below zero, which
     * on the div2 code the driver would refuse outright.
     */
    static bool place_alarm(uint32_t ticks) {
        if (ticks <= fast_span_ticks) {
            uint32_t counts = static_cast<uint32_t>(
                (static_cast<uint64_t>(ticks) * fast_hz + P::ticks_per_second - 1u) /
                P::ticks_per_second);
            if (counts == 0u) {
                counts = 1u;
            }
            last_reload_ = counts - 1u;
            last_fast_ = true;
            return Rtc::set_wakeup(cfg.fast_clock, last_reload_);
        }
        uint32_t seconds = (ticks + P::ticks_per_second - 1u) / P::ticks_per_second;
        if (seconds > 0x10000u) {
            seconds = 0x10000u;
        }
        last_reload_ = seconds - 1u;
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
        rtc_at_arm_ = time_of_hour_ms();
        tick_at_arm_ = P::Timebase::ticks();
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
     * rounds that never slept at all, come out right with no special case.
     * The baseline is consumed EXACTLY ONCE, under the platform's critical
     * section, whichever of isr() and disarm() gets there first.
     */
    static void resync() {
        typename P::CriticalSection cs;
        if (!resync_armed_) {
            return;
        }
        resync_armed_ = false;
        const uint32_t now = time_of_hour_ms();
        if (now == 0xFFFFFFFFu) {
            last_advance_ = 0;
            return;
        }
        const uint32_t wall_ms = elapsed_ms(rtc_at_arm_, now);
        const uint32_t span = static_cast<uint32_t>(
            (static_cast<uint64_t>(wall_ms) * P::ticks_per_second) / 1000u);
        const uint32_t awake = P::Timebase::ticks() - tick_at_arm_;   // wrap-safe
        last_advance_ = span > awake ? span - awake : 0u;
        if (last_advance_ != 0u) {
            P::Timebase::advance(last_advance_);
        }
    }

    /// The four-act ISR body an app's RTC_WKUP_IRQHandler binds. See the
    /// file header for why each act is there.
    [[gnu::always_inline]] static void isr() {
        (void)Plain::resume_clock();   // act 0
        (void)Rtc::wakeup_isr();       // act 1: the EXTI line and the flag
        Rtc::clear_wakeup();
        alarm_armed_ = false;
        resync();                        // act 2
        (void)Pwr::arm(PwrMode::sleep);  // act 3
        P::Timebase::resume();           // ...and the tick fact 4 paused
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
