/*
 * sleep.hpp
 *
 * The RP2040's two ways of stopping (datasheet 2.11), and the sleep
 * sites of util/power.hpp over them.
 *
 * WHAT THE SILICON HAS. A core's WFI stops that core and nothing else:
 * every peripheral, the other core and the DMA run on. When BOTH cores
 * are asleep with SLEEPDEEP set and the DMA has no transfer in flight
 * the chip is in its SLEEP STATE (2.11.2), and the top-level clock
 * gates switch from the WAKE_ENx masks to the SLEEP_ENx masks: what a
 * program leaves out of SLEEP_ENx is unclocked until an interrupt of a
 * block still clocked wakes a core. The oscillators and the PLLs run
 * through it. DORMANT (2.11.3) is the other thing: the oscillator the
 * program runs on is stopped by a keyword written into its DORMANT
 * register and restarts on a GPIO event the IO bank's dormant-wake
 * logic detects, or on the RTC's interrupt; every clock derived from
 * that oscillator stands still meanwhile, the core included, and
 * execution resumes at the next instruction. The PLLs are not stopped
 * by the silicon: the program stops them first and brings them back.
 *
 * THE LADDER on this chip:
 *
 *   light    a plain WFI (SLEEPDEEP clear): the core stops, the chip
 *            does not. The kernel's own idle, named.
 *   standby  the SLEEP state: SLEEPDEEP set, the SLEEP_ENx masks
 *            applied (`Clocks::sleep_enables`, the sets in clock.hpp).
 *            What the masks prune is the program's decision; the
 *            default prunes nothing, so a standby armed by a program
 *            that never set them is a light sleep with SLEEPDEEP. The
 *            timer keeps counting (its gates are forced open by the
 *            timed site), so it is both the alarm and the witness.
 *   deep     DORMANT on the oscillator `source` names: the clocks
 *            moved onto that oscillator, the PLL stopped, the keyword
 *            written; the tree restored on the way back by the
 *            program's own Clock::init(). NOT A WFI - the platform's
 *            idle path takes the site's `sleep_hook` instead
 *            (rp2040/platform.hpp). REFUSED when no wake is
 *            configured: with nothing to restart the oscillator the
 *            keyword is the last instruction the chip executes.
 *
 * THE TWO OSCILLATORS AND THE WAKE. A GPIO dormant-wake event restarts
 * either oscillator. The RTC's alarm restarts either too, but the RTC
 * counts clk_rtc, which brio derives from the crystal: with the CRYSTAL
 * dormant the calendar stops and its alarm never comes, so the RTC is
 * a wake only when the RING OSCILLATOR is the one stopped and the
 * crystal keeps clk_rtc alive (`DormantSource::rosc`) - a dormant that
 * saves the digital core's whole dynamic power and leaves the crystal
 * running. The site's `dormant_wake_ready()` says whether a way back
 * exists, and arm(deep) refuses without one.
 *
 * WHAT A STANDBY ASKS OF THE APPLICATION: the other core asleep with
 * SLEEPDEEP set too (a core 1 still parked in the bootrom sits in a
 * WFE without it, and the suite measures whether the state is reached
 * regardless), the DMA idle, and a SLEEP_ENx set that keeps the
 * fabric, the memories and the wake sources clocked
 * (`sleep_clocks_core` plus the sources). A byte transport whose clocks
 * are pruned loses the byte in flight.
 *
 * THE TIMED SITE lifts nothing here that the plain site forbids -
 * SysTick's behaviour in the SLEEP state is measured by the suite and
 * stated in docs/rp2040/sleep.md - and does what the other strata's
 * timed sites do: places the nearest armed deadline on a TIMER alarm
 * (microseconds, rounded up: late, never early), and after the wake
 * hands the ticker the span the timer counted less the ticks SysTick
 * counted itself (rounded down: never early). For a dormant with a
 * deadline it places the RTC's alarm on the calendar's h:m:s (whole
 * seconds up), the calendar being the witness in whole seconds down;
 * it refuses a dormant with a deadline when the calendar is not
 * running or the crystal is the oscillator stopped.
 */

#pragma once

#include <stdint.h>

#include <optional>

#include "kernel/time_event.hpp"
#include "rp2040/clock.hpp"
#include "rp2040/device.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/rtc.hpp"
#include "rp2040/ticker.hpp"
#include "rp2040/timer.hpp"
#include "util/power.hpp"

namespace brio {

/// The oscillator a dormant stops.
enum class DormantSource : uint8_t {
    xosc,   ///< the crystal: everything stops, a GPIO event is the only way back (a millisecond's restart)
    rosc,   ///< the ring oscillator: clk_rtc on the crystal survives, the RTC's alarm is a wake too (a microsecond's restart)
};

/// The events the IO bank's dormant-wake logic detects on a pin
/// (2.19.6.3), one bit each, combinable.
enum class DormantWakeEvent : uint8_t {
    level_low = IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_LEVEL_LOW_BITS,
    level_high = IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_LEVEL_HIGH_BITS,
    edge_low = IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_EDGE_LOW_BITS,
    edge_high = IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_EDGE_HIGH_BITS,
};

constexpr uint8_t dormant_wake_bits(DormantWakeEvent e) { return static_cast<uint8_t>(e); }
constexpr uint8_t operator|(DormantWakeEvent a, DormantWakeEvent b) {
    return static_cast<uint8_t>(dormant_wake_bits(a) | dormant_wake_bits(b));
}

/// The IO bank's dormant-wake interrupt: four bits a pin in four
/// registers, the enable that names the events, the raw flags shared
/// with the cores' interrupts (INTR: a level flag follows the pad, an
/// edge flag is cleared by writing one), the masked status.
struct DormantWake {
    DormantWake() = delete;

    static constexpr uint8_t pin_count = 30;

    static uint32_t shift_of(uint8_t pin) { return 4u * (pin % 8u); }
    static uint32_t word_of(uint8_t pin) { return 4u * (pin / 8u); }

    static volatile uint32_t& inte(uint8_t pin) { return reg_at(IO_BANK0_BASE, IO_BANK0_DORMANT_WAKE_INTE0_OFFSET + word_of(pin)); }
    static volatile uint32_t& intf(uint8_t pin) { return reg_at(IO_BANK0_BASE, IO_BANK0_DORMANT_WAKE_INTF0_OFFSET + word_of(pin)); }
    static volatile uint32_t& ints(uint8_t pin) { return reg_at(IO_BANK0_BASE, IO_BANK0_DORMANT_WAKE_INTS0_OFFSET + word_of(pin)); }
    static volatile uint32_t& intr(uint8_t pin) { return reg_at(IO_BANK0_BASE, IO_BANK0_INTR0_OFFSET + word_of(pin)); }

    /// The events of `pin` that wake a dormant chip, added or removed.
    static bool enable(uint8_t pin, uint8_t events, bool on) {
        if (pin >= pin_count || (events & ~0x0Fu) != 0u) {
            return false;
        }
        const uint32_t bits = static_cast<uint32_t>(events) << shift_of(pin);
        if (on) { hw_set(inte(pin), bits); } else { hw_clear(inte(pin), bits); }
        return true;
    }
    static uint8_t enabled(uint8_t pin) {
        return pin < pin_count ? static_cast<uint8_t>((inte(pin) >> shift_of(pin)) & 0x0Fu) : 0u;
    }
    /// The raw flags of `pin` (the events seen, enabled or not).
    static uint8_t raised(uint8_t pin) {
        return pin < pin_count ? static_cast<uint8_t>((intr(pin) >> shift_of(pin)) & 0x0Fu) : 0u;
    }
    /// The enabled events of `pin` that stand.
    static uint8_t pending(uint8_t pin) {
        return pin < pin_count ? static_cast<uint8_t>((ints(pin) >> shift_of(pin)) & 0x0Fu) : 0u;
    }
    /// The edge flags of `pin` cleared (a level flag follows the pad).
    static void acknowledge(uint8_t pin, uint8_t events) {
        if (pin < pin_count) {
            intr(pin) = (static_cast<uint32_t>(events) & 0x0Fu) << shift_of(pin);
        }
    }
    /// Is any event of any pin enabled: a way back from a dormant?
    static bool any_enabled() {
        for (uint8_t w = 0; w < 4u; ++w) {
            if (reg_at(IO_BANK0_BASE, IO_BANK0_DORMANT_WAKE_INTE0_OFFSET + 4u * w) != 0u) {
                return true;
            }
        }
        return false;
    }
    static void disable_all() {
        for (uint8_t w = 0; w < 4u; ++w) {
            reg_at(IO_BANK0_BASE, IO_BANK0_DORMANT_WAKE_INTE0_OFFSET + 4u * w) = 0u;
        }
    }
};

/**
 * The plain site: the ladder onto a WFI, the SLEEP state and DORMANT.
 * `Clock` is the program's static clock type, whose init() restores the
 * tree after a dormant; `source` the oscillator a dormant stops.
 * Core 0's: a dormant is chip-wide, and the hook it installs is core
 * 0's platform's.
 */
template <typename Clock, DormantSource source = DormantSource::xosc>
struct Rp2040SleepSite {
    Rp2040SleepSite() = delete;

    static_assert(Clock::is_static, "brio Rp2040SleepSite: the tree is restored after a dormant by the "
                                    "program's static Clock");

    using Platform0 = Rp2040Platform<0>;

    static bool arm(SleepDepth d) {
        switch (d) {
        case SleepDepth::none:
            disarm();
            return true;
        case SleepDepth::light:
            SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
            Platform0::sleep_hook = nullptr;
            armed_ = SleepDepth::light;
            return true;
        case SleepDepth::standby:
            Platform0::sleep_hook = nullptr;
            SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
            armed_ = SleepDepth::standby;
            return true;
        case SleepDepth::deep:
            if (!dormant_wake_ready()) {
                return false;
            }
            SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
            Platform0::sleep_hook = &go_dormant;
            armed_ = SleepDepth::deep;
            return true;
        }
        return false;
    }

    static void disarm() {
        SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
        Platform0::sleep_hook = nullptr;
        armed_ = SleepDepth::none;
    }

    static SleepDepth armed() { return armed_; }

    /// Is there a way back from a dormant: a GPIO dormant-wake event
    /// enabled, or - the ring oscillator the one stopped, the crystal
    /// keeping clk_rtc - the calendar running with its alarm armed and
    /// its interrupt on.
    static bool dormant_wake_ready() {
        if (DormantWake::any_enabled()) {
            return true;
        }
        if constexpr (source == DormantSource::rosc) {
            return Rtc::running() && Rtc::alarm_armed() && Rtc::interrupt_enabled();
        }
        return false;
    }

    /// How many dormants this site has run and returned from.
    static uint32_t dormants() { return dormants_; }

    /**
     * The idle path's substitute for the WFI when deep is armed: called
     * with interrupts masked. clk_sys onto clk_ref and clk_ref onto the
     * oscillator that stays (the crystal already is clk_ref; the ring
     * oscillator is started and selected), the system PLL stopped, the
     * keyword; then, once a wake has restarted the oscillator, the
     * program's tree put back exactly as its Clock states it - clk_sys
     * on the PLL, clk_peri where it was - before the caller unmasks and
     * the wake's handler runs at the rate every driver was told.
     */
    static void go_dormant() {
        (void)Clocks::sys_from_ref();
        Clocks::sys_divider(1);
        if constexpr (source == DormantSource::rosc) {
            (void)Rosc::start();
            (void)Clocks::ref_select(RefSource::rosc);
        } else {
            (void)Clocks::ref_select(RefSource::xosc);
        }
        PllSys::stop();
        if constexpr (source == DormantSource::rosc) {
            Rosc::dormant();
        } else {
            Xosc::dormant();
        }
        (void)Clock::init();
        ++dormants_;
    }

private:
    static inline SleepDepth armed_ = SleepDepth::none;
    static inline uint32_t dormants_ = 0;
};

/**
 * The timed site: the plain site plus the system timer as alarm and
 * witness for a standby, and the calendar for a dormant with a
 * deadline (the file header). `alarm_n` is the timer alarm it owns
 * (0..3); the app binds that alarm's vector to isr() and, for the
 * dormant's wake, isr_rtc to rtc_isr().
 */
template <Platform P, typename Clock, DormantSource source = DormantSource::xosc, uint8_t alarm_n = 3>
struct Rp2040TimedSleepSite {
    Rp2040TimedSleepSite() = delete;

    static_assert(alarm_n < Timer::alarm_count, "brio Rp2040TimedSleepSite: the system timer has four alarms, 0..3");
    static_assert(P::core == 0u, "brio Rp2040TimedSleepSite: core 0's site - a dormant is chip-wide and the "
                                 "tree is core 0's to restore");

    using Plain = Rp2040SleepSite<Clock, source>;

    /// The gates a standby must keep for the timer to count and alarm:
    /// the timer and the watchdog whose tick it counts.
    static constexpr SleepClocks timer_gates{0u, CLOCKS_SLEEP_EN1_CLK_SYS_TIMER_BITS | CLOCKS_SLEEP_EN1_CLK_SYS_WATCHDOG_BITS};

    /// The alarm's line enabled at the NVIC; the timer itself is the
    /// application's (Timer::init). Call once, before the manager's
    /// first round. False when the timer's tick is not running.
    static bool init() {
        if (!WatchdogTick::running()) {
            return false;
        }
        Timer::interrupt(alarm_n, false);
        Timer::clear(alarm_n);
        Nvic::enable(Timer::irq(alarm_n));
        ready_ = true;
        return true;
    }

    static bool arm(SleepDepth d) {
        if (!Plain::arm(d)) {
            return false;
        }
        if (!ready_ || !is_deep_mode(Plain::armed())) {
            return true;   // a light sleep ticks on its own
        }
        const std::optional<uint32_t> next = TimeEvents<P>::ticks_to_next();
        tick_at_arm_ = Ticker::ticks();
        if (Plain::armed() == SleepDepth::standby) {
            Clocks::sleep_enables(Clocks::sleep_enables() | timer_gates);
            us_at_arm_ = Timer::now_low();
            witness_ = Witness::timer;
            if (next.has_value()) {
                // ticks -> microseconds, rounded UP: a shade late is "at
                // least", a shade early is a wasted vote round.
                const uint64_t us = (static_cast<uint64_t>(*next) * 1'000'000u + P::ticks_per_second - 1u) /
                                    P::ticks_per_second;
                Timer::clear(alarm_n);
                Timer::alarm_in(alarm_n, us > 0xFFFF'FFFFu ? 0xFFFF'FFFFu : static_cast<uint32_t>(us));
                Timer::interrupt(alarm_n, true);
                alarm_armed_ = true;
            }
            return true;
        }
        // Dormant. Without a deadline the application's wake is the
        // whole story; with one the calendar is alarm and witness, and
        // only while the crystal keeps it counting.
        if (!next.has_value()) {
            witness_ = Witness::none;
            return true;
        }
        if constexpr (source == DormantSource::xosc) {
            Plain::disarm();
            return false;
        } else {
            const std::optional<RtcDateTime> now = Rtc::read();
            if (!now.has_value()) {
                Plain::disarm();
                return false;
            }
            const uint32_t seconds = (*next + P::ticks_per_second - 1u) / P::ticks_per_second;
            if (seconds >= 86'400u) {
                Plain::disarm();
                return false;
            }
            const uint32_t at = (seconds_of_day(*now) + seconds) % 86'400u;
            RtcAlarm a{};
            a.at.hour = static_cast<uint8_t>(at / 3600u);
            a.at.minute = static_cast<uint8_t>((at / 60u) % 60u);
            a.at.second = static_cast<uint8_t>(at % 60u);
            a.match_hour = a.match_minute = a.match_second = true;
            rtc_at_arm_ = seconds_of_day(*now);
            witness_ = Witness::rtc;
            if (!Rtc::alarm(a)) {
                Plain::disarm();
                return false;
            }
            Rtc::interrupt(true);
            Nvic::enable(Rtc::irq());
            rtc_alarm_armed_ = true;
            // The wake now exists: the plain site's refusal is re-asked.
            if (!Plain::dormant_wake_ready()) {
                Plain::disarm();
                return false;
            }
            return true;
        }
    }

    static void disarm() {
        if (alarm_armed_) {
            Timer::interrupt(alarm_n, false);
            Timer::disarm(alarm_n);
            Timer::clear(alarm_n);
            alarm_armed_ = false;
        }
        if (rtc_alarm_armed_) {
            Rtc::interrupt(false);
            Rtc::disarm_alarm();
            rtc_alarm_armed_ = false;
        }
        resync();
        Plain::disarm();
    }

    static SleepDepth armed() { return Plain::armed(); }

    /// Catch kernel time up by the FROZEN span: the witness's elapsed
    /// time converted DOWN to ticks, less what SysTick itself counted
    /// since arm(). Consumed once - isr() and disarm() both call it, and
    /// a round that never slept advances by nothing because the two
    /// rulers agree. The critical section makes the once-ness hold
    /// against an alarm landing inside a disarm().
    static void resync() {
        typename P::CriticalSection cs;
        if (witness_ == Witness::none) {
            return;
        }
        uint32_t span = 0;
        if (witness_ == Witness::timer) {
            const uint32_t us = Timer::now_low() - us_at_arm_;   // wrap-safe
            span = static_cast<uint32_t>(static_cast<uint64_t>(us) * P::ticks_per_second / 1'000'000u);
        } else {
            const std::optional<RtcDateTime> now = Rtc::read();
            const uint32_t s = now ? (seconds_of_day(*now) + 86'400u - rtc_at_arm_) % 86'400u : 0u;
            span = s * P::ticks_per_second;
        }
        witness_ = Witness::none;
        const uint32_t awake = Ticker::ticks() - tick_at_arm_;   // wrap-safe
        if (span > awake) {
            last_advance_ = span - awake;
            Ticker::advance(last_advance_);
        } else {
            last_advance_ = 0;
        }
    }

    /**
     * The ISR body the app's timer alarm vector binds: acknowledge,
     * RESYNC, and HAND THE MACHINE BACK TO A TICKING SLEEP. The third
     * act is the SAM's lesson: the never-early rounding can leave
     * kernel time a fraction of a tick short of the deadline at the
     * alarm, and an alarm posts nothing to any queue, so with the
     * standby still armed and the alarm spent the loop would re-enter a
     * sleep nothing ends. Disarming the plain site here makes the next
     * idle a plain WFI, which the tick ends.
     */
    [[gnu::always_inline]] static void isr() {
        if (!Timer::pending(alarm_n)) {
            return;
        }
        Timer::clear(alarm_n);
        Timer::interrupt(alarm_n, false);
        alarm_armed_ = false;
        resync();
        Plain::disarm();
    }

    /// The same for the calendar's wake of a dormant (isr_rtc): the
    /// RTC's own body first (the line masked, the match disarmed).
    [[gnu::always_inline]] static bool rtc_isr() {
        if (!Rtc::isr()) {
            return false;
        }
        rtc_alarm_armed_ = false;
        resync();
        Plain::disarm();
        return true;
    }

    // ---- readbacks the suites and a status console want --------------------
    static bool ready() { return ready_; }
    static bool alarm_armed() { return alarm_armed_; }
    static bool rtc_alarm_armed() { return rtc_alarm_armed_; }
    /// Ticks the last resync handed to Ticker::advance() (0 when the
    /// round never slept, or slept less than it stayed awake).
    static uint32_t last_advance() { return last_advance_; }

private:
    enum class Witness : uint8_t { none, timer, rtc };

    static uint32_t seconds_of_day(const RtcDateTime& d) {
        return static_cast<uint32_t>(d.hour) * 3600u + static_cast<uint32_t>(d.minute) * 60u + d.second;
    }

    static inline bool ready_ = false;
    static inline bool alarm_armed_ = false;
    static inline bool rtc_alarm_armed_ = false;
    static inline Witness witness_ = Witness::none;
    static inline uint32_t us_at_arm_ = 0;
    static inline uint32_t rtc_at_arm_ = 0;
    static inline uint32_t tick_at_arm_ = 0;
    static inline uint32_t last_advance_ = 0;
};

} // namespace brio
