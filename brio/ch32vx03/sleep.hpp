/*
 * sleep.hpp
 *
 * util/power.hpp's depth ladder on CH32V203 silicon: `Ch32vx03SleepSite`,
 * which arms one of this family's modes and puts the clock tree back
 * afterwards, and `Ch32vx03TimedSleepSite`, which additionally keeps
 * KERNEL TIME HONEST across a Stop by placing the RTC's alarm where the
 * deadline is and handing the frozen span back to the STK ticker.
 *
 * The MECHANISM - which modes exist, what they gate, what wakes them -
 * is ch32vx03/pwr.hpp's. The POLICY - when a program may stop - is
 * util/power.hpp's. What is HERE is the mapping between them, and the
 * mapping is this target's own decision.
 *
 * ## The ladder
 *
 *   none    -> Sleep      SLEEPDEEP = 0: the core clock stops and
 *                         nothing else does
 *   light   -> Sleep      THE SAME MODE (fact 1)
 *   standby -> Stop       SLEEPDEEP = 1, PDDS = 0, the MAIN regulator:
 *                         HSE, HSI, PLL and every peripheral clock
 *                         stop, SRAM and registers are kept, and the
 *                         wake is an EXTI line
 *   deep    -> Stop       the same mode with the LOW-POWER regulator:
 *                         cheaper to hold, and the price is in the wake
 *
 * ## Six things to read before using it
 *
 * 1. A SLEEP OF ANY DEPTH IS LEGAL HERE ONLY WHILE THE CORE OWNS THE
 *    BUS, and `arm()` REFUSES every rung but `none` while it does not.
 *    On this family no bus master but the core gets a cycle in a sleep
 *    (measured: ch32vx03/bus_activity.hpp), so a DMA channel or the USB
 *    controller that is working is not slowed by a sleep, it is broken
 *    by it. The refusal is what the vote round sees - a vote already
 *    lost, without a voter having to be written - and the kernel's idle
 *    path makes the same test on its own (ch32vx03/platform.hpp), so a
 *    program that never runs a manager is covered by the same fact.
 *
 * 2. `light` AND `none` ARE ONE CODE, and that is the honest answer
 *    rather than a shortcut. util/power.hpp's rule is that a target maps
 *    what it has not got to the nearest SHALLOWER mode. Between Sleep
 *    and Stop this family has nothing - 2.3 lists three modes and no
 *    low-power run - so `light` maps to Sleep, and `armed()`, which is a
 *    pure read of the silicon, answers `none` for it, because that is
 *    what the machine will really do.
 *
 * 3. THE TWO DEEP RUNGS ARE ONE MODE WITH TWO PRICES. This family's
 *    Stop is a single mode (SLEEPDEEP with PDDS clear) whose cost is set
 *    by LPDS and RAMLV - so the ladder's two deep rungs are two
 *    `StopConfig`s and not two modes, and `armed()` cannot tell them
 *    apart from the registers alone. It reports the depth THIS SITE last
 *    armed for a Stop, and `deep` for a Stop somebody else armed through
 *    `Pwr`. The two configurations are the site's template argument.
 *
 * 4. STANDBY IS OFF THE LADDER ON PURPOSE, AND THE SITE ENFORCES IT:
 *    every `arm()` here goes through `Pwr::arm()`, which CLEARS PDDS
 *    whichever rung is asked for, so no rung can reach Standby and a
 *    PDDS left standing by anything else is taken down at the next
 *    round.
 *    util/power.hpp's model is built on the program RESUMING - "the
 *    manager's next dispatch, of ANY event, first disarms the site and
 *    publishes a WakeReport" - and Standby's documented exits are a
 *    POWER RESET (2.3.4): a site that armed one would leave a manager
 *    waiting for a wake that arrives as a reboot. Standby stays
 *    reachable, as it should be, through `enter_standby()` - a
 *    deliberate one-shot whose resumption is the application's boot path
 *    reading PWR_CSR.SBF and whatever it left in the backup registers.
 *
 * 5. THE KERNEL'S TICK IS PAUSED ACROSS A STOP, AND THE PLATFORM DOES
 *    IT. The STK counts HCLK, which a Stop stops, so kernel time stands
 *    still for the whole sleep whatever anyone does; and with the WFE
 *    idiom this core's idle path uses, a tick merely PENDING would end
 *    the sleep before it began. Both are answered in one place - the
 *    idle hook pauses the timebase, clears the counter's flag and the
 *    line's pending bit immediately before the instruction and resumes
 *    after it (ch32vx03/platform.hpp) - because that is the only place
 *    where no tick can slip in between. The site therefore does NOT
 *    touch the ticker; it only says how far kernel time fell behind
 *    (the timed site below).
 *
 * 6. WHAT COMES BACK FROM A STOP IS NOT WHAT WENT IN. 2.3.3: the HSI is
 *    the system clock after the wake, with the PLL and the HSE off - so
 *    a program running at 144 MHz resumes at 8 with a USART divisor
 *    meant for eighteen times that. The site is TEMPLATED ON THE CLOCK
 *    TASK and puts it back, and the place it does so is the model's own
 *    hook and not a new one: arm() is called before the machine stops
 *    and disarm() on the FIRST EVENT AFTER THE WAKE, which is exactly
 *    "put the clock back". A program on `ClockSource::internal` pays one
 *    register read, because SWS already reads what the task asked for.
 *
 * ## The timed site
 *
 * The plain site keeps an HONEST RESTRICTION: with kernel time frozen
 * for the whole Stop, a program with armed time events must not take
 * one (the manager's deadline guard is what enforces it).
 * `Ch32vx03TimedSleepSite` LIFTS it, inside arm() and disarm() alone,
 * with the RTC in both roles:
 *
 *  - the ALARM is the RTC's own, reached through EXTI line 17 - the
 *    path that survives a Stop because an EXTI line is asynchronous and
 *    needs no clock (2.3.5, and ch32vx03/rtc.hpp's `arm_wake()`). It is
 *    placed at the current count plus the deadline, rounded UP;
 *  - the WITNESS is the same counter: the difference between the
 *    reading at arm() and the reading at the wake is how far the world
 *    moved, and subtracting what the STK itself counted leaves the
 *    FROZEN span, which goes to `Ticker::advance()`.
 *
 * THE RATE RULE IS DIRECTIONAL, and both halves want the same
 * direction: state a TR_CLK rate NOT BELOW the true one. Over-stating
 * it makes the alarm arithmetic ask for more counts than it needs, so
 * the wake lands LATE; and it makes the witness under-state the span,
 * so the resync UNDER-advances. Both errors land on the side the
 * kernel's time contract allows: at least, never early. With the LSE
 * the rate is exact; on the LSI a program states the datasheet's upper
 * bound (device::lsi_max_hz) and pays for it in lateness.
 *
 * THE TICK THE COUNTER COUNTS IS THE RULER, and it must not be coarser
 * than the kernel's own: the config asks for a TR_CLK and the site
 * refuses one below `P::ticks_per_second`, because a deadline placed on
 * a coarser ruler lands up to one of ITS periods late while the model's
 * deadline guard is counted in kernel ticks. The default is 1024 Hz
 * from the 32.768 kHz crystal - a prescaler of 32, one count every
 * 976 us, and a counter that wraps in 48 days.
 *
 * THE PRESCALER CANNOT BE READ BACK (6.3.3: RTC_PSCR is write-only), so
 * init() always writes it. The cost is the phase of the tick in
 * progress, once, at the boot, before anything has been measured.
 *
 * A COUNT READ AFTER A WAKE IS SYNCHRONIZED FIRST. 6.2.3: after a wake
 * from Stop or Standby the values the bus reads are stale until an
 * RTCCLK edge has passed, so the resync clears RSF and waits for it -
 * up to one RTCCLK period, thirty microseconds on the crystal - before
 * it believes a count. A resync that cannot synchronize advances
 * NOTHING, because an unknown span is not a span to guess at.
 *
 * THE ISR HAS FOUR ACTS, and every one of them is load-bearing:
 *   0. RESTORE THE CLOCK - fact 6. First, so that everything after it
 *      (and every handler that runs later) is at full speed.
 *   1. ACKNOWLEDGE - the EXTI line's flag and the RTC's own ALRF, or
 *      the vector re-enters for ever (`Rtc::alarm_isr()` does both).
 *   2. RESYNC - hand the frozen span to the ticker.
 *   3. HAND THE MACHINE BACK TO A TICKING SLEEP. The never-early bias
 *      GUARANTEES that kernel time is still a shade short of the
 *      deadline when the alarm lands, and an RTC wake posts nothing to
 *      any queue, so a loop that idled again into the still-armed Stop
 *      with the alarm now spent would never come back. Disarming here
 *      is the escape: the residual ticks mature on the STK within
 *      milliseconds and TimeEvents::process() posts the deadline on its
 *      own clock.
 *
 * A FOREIGN wake - any other interrupt - does not run this body. Its
 * progress is its own event, the resync happens in disarm() instead,
 * and the Stop stays armed WITH the alarm still standing. That is why
 * util/power.hpp's convention (a wake path with nothing to say sends
 * SleepRequested{none}) is LOAD-BEARING with this site.
 *
 * ## What the site owns
 *
 * THE RTC, WHOLE: the domain gate, the clock select, the prescaler and
 * the alarm. An application using the timed site must not drive
 * ch32vx03/rtc.hpp's counter elsewhere, and must bind the vector:
 *
 *     extern "C" BRIO_CH32_INTERRUPT void rtc_alarm_handler() { Site::isr(); }
 *
 * WHAT THE SILICON SAID OF ALL THIS is in docs/ch32vx03/sleep.md, with
 * the numbers: a Stop woken by the alarm inside a few tens of
 * microseconds of it, the clock tree back at the program's rate two
 * milliseconds later, and a five-hundred-millisecond deadline met on
 * the WALL to two milliseconds through the timed site.
 */

#pragma once

#include <stdint.h>

#include <optional>

#include "ch32vx03/bus_activity.hpp"
#include "ch32vx03/clock.hpp"
#include "ch32vx03/device.hpp"
#include "ch32vx03/exti.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pwr.hpp"
#include "ch32vx03/rtc.hpp"
#include "ch32vx03/ticker.hpp"
#include "kernel/platform.hpp"
#include "kernel/time_event.hpp"
#include "util/power.hpp"

namespace brio {

/**
 * The two Stop flavours the ladder's deep rungs take (fact 3).
 *
 * The defaults are the two the chapter prices without a footnote:
 * `standby` is the main regulator - the fastest wake this silicon has -
 * and `deep` is the low-power one. RAMLV is NOT on in either, although
 * 2.3.3 offers it as "the lowest power consumption": nothing in the
 * chapter says what the RAM's low-voltage mode does to the retention a
 * Stop is chosen for, and no meter on this desk has priced it, so a
 * program that wants it says so where it names the site.
 */
struct SleepSiteConfig {
    StopConfig standby{};
    StopConfig deep{StopRegulator::low_power, false};
};

constexpr bool sleep_site_config_valid(const SleepSiteConfig& c) {
    return stop_config_valid(c.standby) && stop_config_valid(c.deep);
}

/**
 * The plain site: arm a rung, say what is armed, put the clock back.
 *
 *   using Site = brio::Ch32vx03SleepSite<Clock>;
 *   brio::PowerManager<P, Site, ...> power;
 *
 * `C` is the application's Clock task or DynamicClock (ch32vx03/
 * clock.hpp) - not for the rate, but because a Stop drops SYSCLK to the
 * HSI and something has to put it back (fact 6).
 */
template <class C, SleepSiteConfig cfg = SleepSiteConfig{}>
struct Ch32vx03SleepSite {
    static_assert(sleep_site_config_valid(cfg),
                  "brio Ch32vx03SleepSite: RAMLV is valid only with the low-power regulator "
                  "(RM 2.4.1's own note on the bit), so a Stop configuration that asks for "
                  "the RAM's low-voltage mode on the main regulator is not a mode this "
                  "silicon has");

    Ch32vx03SleepSite() = delete;

    /// The two Stop flavours, for a caller that wants to print them.
    static constexpr SleepSiteConfig config = cfg;

    /**
     * Re-establish the clock the task promised, if a Stop took it away.
     *
     * The test is the silicon's own: RCC_CFGR0.SWS says what SYSCLK is
     * NOW, and a Stop leaves it on the HSI. For a PLL-based clock that
     * is a mismatch and the task is re-run - which parks on the HSI,
     * restarts the HSE where the rate needs it, reconfigures and starts
     * the PLL, and switches. Under a DynamicClock the same test and the
     * same re-run are the clock's own restore(), for the rate IN FORCE
     * and never the boot one.
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
     * Arm a rung. FALSE while any bus master but the core is working
     * (fact 1) - which is a refusal of the silicon's and not of the
     * model's, and the one thing this site says no to. `none` always
     * takes: disarming must never be refused, because the manager
     * disarms on the first event after a wake and a transfer started
     * meanwhile must not leave a deep mode standing.
     */
    static bool arm(SleepDepth d) {
        if (d == SleepDepth::none) {
            disarm();
            return true;
        }
        if (BusActivity::active() != 0u) {
            return false;
        }
        switch (d) {
            case SleepDepth::light:
                armed_ = SleepDepth::none;
                return Pwr::arm(PwrMode::sleep);
            case SleepDepth::standby:
            case SleepDepth::deep: {
                const StopConfig& sc = d == SleepDepth::deep ? cfg.deep : cfg.standby;
                if (!Pwr::arm(PwrMode::stop, sc)) {
                    return false;
                }
                armed_ = d;
                return true;
            }
            default:
                return false;
        }
    }

    /// Back to the kernel's own idle behaviour: the shallow mode armed
    /// and the clock the program was promised - because the first thing
    /// that reaches the manager after a Stop is the first thing that can
    /// put either back.
    static void disarm() {
        (void)Pwr::arm(PwrMode::sleep);
        armed_ = SleepDepth::none;
        (void)resume_clock();
    }

    /**
     * What a following idle path would take. SLEEPDEEP and PDDS are read
     * from the silicon; which of the two DEEP RUNGS a Stop is cannot be
     * (fact 3), so the mirror this site wrote at arm() answers that half
     * - and a Stop this site did not arm reports `deep`, the rung that
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

    /**
     * THE DOOR THE LADDER HAS NOT (fact 4): Standby, entered here and
     * now. The documented exits - the WKUP pad's rising edge, the RTC
     * alarm, the NRST pad, an IWDG reset - all come back through a power
     * reset, so what follows this call is the program's boot path
     * reading `Pwr::standby_flag()`; but table 2-1's own note says an
     * EXTI event exits without the reset, so this is not [[noreturn]]
     * and a caller that means to end here spins after it.
     *
     * The caller arms its own wake first (the pad through
     * `Pwr::wakeup_pin(true)`, the alarm through ch32vx03/rtc.hpp) and
     * leaves whatever it wants to survive in the backup registers: SRAM
     * is kept only as far as the retention bits say (`Pwr::retain_ram`).
     */
    static void enter_standby() {
        armed_ = SleepDepth::none;
        Pwr::enter(PwrMode::standby);
    }

private:
    static inline SleepDepth armed_ = SleepDepth::none;
};

static_assert(SleepSite<Ch32vx03SleepSite<Clock<ClockSource::internal, 8'000'000>>>);

// ---- the timed site ---------------------------------------------------------

/**
 * Ch32vx03TimedSleepSite's knobs.
 *
 * `rtcclk_hz` is the rate of the clock the RTC counts AND THE RULE IS
 * DIRECTIONAL: give a value NOT BELOW the true rate (the file header).
 * `tick_hz` is the TR_CLK wanted out of it - the ruler the alarm and the
 * witness both speak, and the one the prescaler is computed for.
 * `source` is what RTCSEL is asked for; `wipe_domain` says whether
 * init() may reset the backup domain to get there - which it must on a
 * board whose domain came up on a different source, RTCSEL being
 * one-way, AND WHICH COSTS THE BACKUP REGISTERS.
 */
struct TimedSleepConfig {
    uint32_t rtcclk_hz = 32'768;
    uint32_t tick_hz = 1024;
    RtcClockSource source = RtcClockSource::lse;
    bool wipe_domain = false;
};

/// The prescaler divider a config asks for: floor, so that the tick it
/// makes is never SLOWER than the one asked for.
constexpr uint32_t timed_sleep_divider(const TimedSleepConfig& c) {
    return (c.rtcclk_hz == 0u || c.tick_hz == 0u || c.rtcclk_hz < c.tick_hz)
               ? 0u
               : c.rtcclk_hz / c.tick_hz;
}

/// The TR_CLK that divider makes, ROUNDED UP - an upper bound on the
/// true rate, which is the direction both halves of the site want.
constexpr uint32_t timed_sleep_tr_hz(const TimedSleepConfig& c) {
    const uint32_t div = timed_sleep_divider(c);
    return div == 0u ? 0u : (c.rtcclk_hz + div - 1u) / div;
}

constexpr bool timed_sleep_config_valid(const TimedSleepConfig& c) {
    const uint32_t div = timed_sleep_divider(c);
    return div != 0u && c.source != RtcClockSource::none && rtc_prescaler_valid(div - 1u);
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
struct Ch32vx03TimedSleepSite {
    static_assert(timed_sleep_config_valid(cfg),
                  "brio Ch32vx03TimedSleepSite: the RTC's prescaler is twenty bits and its "
                  "source must be named (RM 6.3.4, 3.4.9) - rtcclk_hz / tick_hz must be a "
                  "divider this block can hold, and RtcClockSource::none is not a clock");

    static_assert(timed_sleep_tr_hz(cfg) >= P::ticks_per_second,
                  "brio Ch32vx03TimedSleepSite: the RTC tick is the ruler this site places "
                  "deadlines on and measures spans with, and a ruler coarser than the "
                  "kernel's own tick cannot do either - ask for a tick_hz at least the "
                  "platform's ticks_per_second (1024 Hz out of the crystal is the default)");

    static_assert(requires {
                      P::Timebase::advance(0u);
                  },
                  "brio Ch32vx03TimedSleepSite: a timed site repairs a timebase that STOPS "
                  "in a Stop (the STK ticker, resynchronized through advance()); this "
                  "platform's timebase has no such verb");

    Ch32vx03TimedSleepSite() = delete;

    using Plain = Ch32vx03SleepSite<C, site_cfg>;

    /// The prescaler reload this config asks for, and the TR_CLK it
    /// makes - an upper bound on the true rate (the file header).
    static constexpr uint32_t prescaler = timed_sleep_divider(cfg) - 1u;
    static constexpr uint32_t tr_hz = timed_sleep_tr_hz(cfg);

    /**
     * THE FLOOR ON A PLACED ALARM, in counts. The alarm register is
     * written through the chapter's configuration window, which crosses
     * into RTCCLK and takes a few of ITS cycles to land; an alarm placed
     * one count ahead could be passed by the counter while the write is
     * still in flight, and an alarm equal to a count that has gone by
     * never fires at all. Four counts of RTCCLK's own division is a
     * margin of an order of magnitude over the window's cost, and
     * anything nearer than that is a deadline the manager's guard should
     * have refused.
     */
    static constexpr uint32_t alarm_floor_counts = 4;

    /**
     * Route the RTC, start its oscillator, set the prescaler and arm the
     * alarm's EXTI line. Call once, after the clock init, before the
     * manager's first round.
     *
     * False = the domain or the RTC refused: a clock select already
     * taken by something else with `wipe_domain` not given, an
     * oscillator that never reported ready, a synchronization that never
     * settled. The site still works as a plain Ch32vx03SleepSite in that
     * case, minus every timed property - which is why the caller must
     * look at the answer.
     */
    static bool init() {
        Pwr::bus_clock(true);
        if (!RtcDomain::open(cfg.source, cfg.wipe_domain)) {
            return false;
        }
        // open() routes the clock and does not START it: which
        // oscillator runs is the application's, and here the config says
        // which. The HSE is the clock task's and is already running if
        // the program is on it.
        if (cfg.source == RtcClockSource::lse) {
            RtcDomain::lse_enable(true);
            if (!RtcDomain::lse_wait_ready()) {
                return false;
            }
        } else if (cfg.source == RtcClockSource::lsi) {
            if (!Rcc::lsi_start()) {
                return false;
            }
        }
        if (!Rtc::synchronize()) {
            return false;
        }
        if (!Rtc::configure(RtcConfig{.prescaler = prescaler})) {
            return false;
        }
        // The RTC's own vector carries the second and the overflow as
        // well as the alarm, and this site wants none of them: the wake
        // is the alarm ALONE, on the line that survives a Stop.
        (void)Rtc::interrupts(0);
        if (!Rtc::arm_wake(true) || !Rtc::arm_wake_event(true)) {
            return false;
        }
        Rtc::clear(rtc_alrf);
        (void)Exti::clear(Rtc::wake_line);
        Pfic::clear_pending(Irq::rtc_alarm);
        Pfic::enable(Irq::rtc_alarm);
        ready_ = true;
        return true;
    }

    static bool ready() { return ready_; }
    static bool alarm_armed() { return alarm_armed_; }
    /// Ticks the last resync handed to Timebase::advance() (0 when the
    /// round never slept, or slept less than it stayed awake).
    static uint32_t last_advance() { return last_advance_; }
    /// The counts the last placed alarm was given - diagnostics, and
    /// what a suite checks the arithmetic against.
    static uint32_t last_counts() { return last_counts_; }

    /// Counts for a deadline `ticks` kernel ticks away, rounded UP and
    /// floored: the conversion, public because it is worth being able to
    /// check without arming a sleep.
    static constexpr uint32_t counts_for(uint32_t ticks) {
        const uint32_t n = static_cast<uint32_t>(
            (static_cast<uint64_t>(ticks) * tr_hz + P::ticks_per_second - 1u) /
            P::ticks_per_second);
        return n < alarm_floor_counts ? alarm_floor_counts : n;
    }

    /// The kernel ticks a span of `counts` is worth, rounded DOWN: the
    /// witness's half, and the direction that never advances too far.
    static constexpr uint32_t ticks_for(uint32_t counts) {
        return static_cast<uint32_t>((static_cast<uint64_t>(counts) * P::ticks_per_second) /
                                     tr_hz);
    }

    /**
     * Place the alarm for a deadline `ticks` kernel ticks away. False =
     * the RTC refused the write, or the counter had already passed the
     * value by the time the write landed - which is checked, because an
     * alarm in the past is a wake that never comes.
     */
    static bool place_alarm(uint32_t ticks) {
        const uint32_t counts = counts_for(ticks);
        const uint32_t at = Rtc::count() + counts;
        if (!Rtc::alarm(at)) {
            return false;
        }
        last_counts_ = counts;
        // Wrap-safe: the difference is what matters, not the order.
        return static_cast<int32_t>(at - Rtc::count()) > 0;
    }

    /**
     * Arm a rung, and with it the alarm and the witness. A deep rung
     * with a deadline whose alarm could not be placed is DOWNGRADED to
     * the plain Sleep rather than taken blind: kernel time keeps running
     * there, so a missed alarm costs current and never a deadline.
     */
    static bool arm(SleepDepth d) {
        if (!Plain::arm(d)) {
            return false;
        }
        if (!ready_ || !is_deep_mode(Plain::armed())) {
            return true;   // Sleep keeps the STK: nothing to compensate
        }
        count_at_arm_ = Rtc::count();
        tick_at_arm_ = P::Timebase::ticks();
        resync_armed_ = true;
        const std::optional<uint32_t> next = TimeEvents<P>::ticks_to_next();
        if (!next.has_value()) {
            return true;   // nothing waiting: the sleep may last
        }
        if (!place_alarm(*next)) {
            // Shallower rather than blind, and through disarm() rather
            // than through arm(light): the plain site's arm() may refuse
            // (a master that started meanwhile), and a refusal here
            // would leave SLEEPDEEP standing over a deadline with no
            // alarm behind it. disarm() cannot refuse.
            resync_armed_ = false;
            Plain::disarm();
            return true;
        }
        alarm_armed_ = true;
        return true;
    }

    static void disarm() {
        Plain::disarm();
        if (alarm_armed_) {
            Rtc::clear(rtc_alrf);
            (void)Exti::clear(Rtc::wake_line);
            alarm_armed_ = false;
        }
        resync();
    }

    static SleepDepth armed() { return Plain::armed(); }

    /**
     * Catch kernel time up by the FROZEN span: the RTC's own elapsed
     * counts minus what the STK itself counted since arm(). The
     * subtraction is what makes several naps inside one armed round, and
     * rounds that never slept at all, come out right with no special
     * case. The baseline is consumed EXACTLY ONCE, under the platform's
     * critical section, whichever of isr() and disarm() gets there
     * first.
     */
    static void resync() {
        {
            // The BASELINE is what the two paths race for, and it is
            // consumed here and nowhere else. Only this short decision
            // is guarded: the synchronization that follows costs up to
            // an RTCCLK period, which is not a critical section anyone
            // should pay for.
            typename P::CriticalSection cs;
            if (!resync_armed_) {
                return;
            }
            resync_armed_ = false;
        }
        if (!Rtc::synchronize()) {
            last_advance_ = 0;
            return;   // an unknown span is not a span to guess at
        }
        const uint32_t counts = Rtc::count() - count_at_arm_;          // wrap-safe
        const uint32_t span = ticks_for(counts);
        const uint32_t awake = P::Timebase::ticks() - tick_at_arm_;    // wrap-safe
        last_advance_ = span > awake ? span - awake : 0u;
        if (last_advance_ != 0u) {
            P::Timebase::advance(last_advance_);
        }
    }

    /// The four-act ISR body an app's RTC alarm handler binds. See the
    /// file header for why each act is there.
    [[gnu::always_inline]] static void isr() {
        (void)Plain::resume_clock();   // act 0
        (void)Rtc::alarm_isr();        // act 1: the EXTI line's flag and ALRF
        alarm_armed_ = false;
        resync();                      // act 2
        Plain::disarm();               // act 3: SLEEPDEEP down again
    }

private:
    static inline bool ready_ = false;
    static inline bool alarm_armed_ = false;
    static inline bool resync_armed_ = false;
    static inline uint32_t count_at_arm_ = 0;
    static inline uint32_t tick_at_arm_ = 0;
    static inline uint32_t last_advance_ = 0;
    static inline uint32_t last_counts_ = 0;
};

} // namespace brio
