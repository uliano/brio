/*
 * ticker.hpp
 *
 * The kernel timebase of this target, on the Cortex-M0+ SysTick.
 *
 * WHY SYSTICK AND NOT A TC. SysTick is core-private: no application can
 * use it for PWM, capture or anything else, so claiming it costs the app
 * nothing - every TC, TCC and the RTC stay free. That is the same rule
 * the AVR side follows by taking the RTC's PIT (a timer nothing else
 * wants), applied to what this core offers.
 *
 * Monostate, exactly like avrdx/ticker.hpp: every member is a static
 * inline, there is one timebase per program because there is one SysTick,
 * and the ISR body reaches its counters with no pointer indirection.
 * State lives in .bss, zeroed before main().
 *
 * ## Time representations
 *  - ticks():  raw 32-bit tick counter (wraps: 49.7 days @ 1000 Hz).
 *  - millis(): milliseconds - EXACT here, see below.
 *  - secs():   exact seconds (wraps ~136 years).
 *  - now():    TimeStamp = whole seconds + millisecond fraction.
 *
 * ## No millisecond correction, and why the rate is constrained
 * AVR's 1024 Hz tick does not divide decimal milliseconds, so its ISR
 * skips three increments per 128 ticks to keep millis() honest. Nothing
 * of the sort is needed here: SysTick counts CPU cycles, so the rate is
 * ours to choose, and every rate this class accepts divides 1000 exactly
 * (1000, 500, 250, 200, 125, 100 ... Hz). millis() is then ticks times a
 * compile-time constant - exact, with no drift and no jitter. A rate that
 * does not divide 1000 is refused rather than silently approximated.
 *
 * ## Concurrency
 * tick() runs in the SysTick handler. The counters are 32-bit and this
 * core loads an aligned word in one uninterruptible access
 * (atomic_width 4), so no getter masks interrupts for a SINGLE counter.
 * But atomicity is not visibility: the getters read through a VOLATILE
 * access (read_shared below) so every call performs a real load - in a
 * header-only build a polling loop over an inlined getter would
 * otherwise fold to one hoisted read that never sees the handler's
 * store. AVR's getters get both properties from ATOMIC_BLOCK at once
 * (cli for atomicity, its barriers for the reload); here each has its
 * own explicit source, exactly as util/ring.hpp spells it for its
 * shared indexes. now() additionally masks: it reads TWO counters that
 * must belong to the same instant.
 *
 * ## The caveat that outlives this file
 * SysTick is clocked from the CPU clock, so its reload is a function of
 * Clock::hz. There is no DynamicClock on this target yet; when one
 * arrives, this ticker must either become a ClockUser (rebase the
 * reload) or move to the RTC - unlike AVR, where the 32 kHz PIT tick
 * never moves when CLK_PER does. init()'s clock_follows assertion below
 * is what will refuse to compile on that day, which is the point.
 *
 * ## The same caveat, in its second half: STANDBY FREEZES THIS TIMEBASE
 * Being clocked from the CPU clock has a consequence beyond rebasing:
 * in the PM's STANDBY sleep mode the CPU clock stops, so SysTick stops
 * and KERNEL TIME STANDS STILL for exactly as long as the sleep lasts.
 * Time events do not fire late by a little - they fire late by the
 * whole slept duration, and `millis()` under-reports the wall clock by
 * the same amount. This does NOT travel from the AVR, where the tick is
 * the RTC's PIT on a 32 kHz oscillator and runs through every sleep
 * mode the part has. IDLE is unaffected: MCLK and GCLK0 keep running
 * there.
 *
 * The two answers are stated in samc/sleep.hpp and enforced nowhere in
 * this file: `SamSleepSite` keeps the v1 restriction (standby only with
 * no armed time event), and `SamTimedSleepSite` lifts it by
 * resynchronizing this counter from the RTC after every wake - the
 * `advance()` verb below is that resync's landing point.
 *
 * ## Usage
 * ```cpp
 * #include "samc/ticker.hpp"
 *
 * extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
 *
 * int main() {
 *     SysClock::init();                     // brio::Clock<...>, samc/clock.hpp
 *     brio::Ticker::init(clock);            // reload from the clock's rate
 *     brio::enable_interrupts();
 * }
 * ```
 */

#pragma once

#include <stdint.h>

#include "sam.h"

#include "samc/nvic.hpp"
#include "util/clock.hpp"
#include "util/timestamp.hpp"

namespace brio {

/**
 * @class BasicTicker
 * @brief Static (monostate) time tracker driven by the SysTick interrupt
 * @tparam tps Tick frequency in Hz; must divide 1000 exactly
 */
template <uint16_t tps = 1000>
class BasicTicker {
    static_assert(tps > 0, "ticks per second must be positive");
    static_assert(1000u % tps == 0u,
                  "brio BasicTicker: the tick rate must divide 1000 exactly, so that "
                  "millis() is a whole number of milliseconds per tick - no skip "
                  "correction, no drift");

private:
    /// Milliseconds per tick: exact by the static_assert above.
    static constexpr uint16_t millis_per_tick = static_cast<uint16_t>(1000u / tps);

    // Counters shared with the handler (inline statics, in .bss).
    static inline uint32_t m_ticks = 0;
    static inline uint32_t m_secs = 0;
    static inline uint16_t m_frac = 0;   ///< ticks elapsed into the current second

    /// Read a counter the handler writes: the volatile access forces a
    /// real load on EVERY call (see the file header's Concurrency
    /// section for why atomicity alone is not enough).
    template <typename T>
    static T read_shared(const T& counter) {
        return *const_cast<const volatile T*>(&counter);
    }

public:
    BasicTicker() = delete;  // static-only class: no instances

    /// Tick frequency in Hz (exposed for period computations)
    static constexpr uint16_t ticks_per_second = tps;

    /**
     * @brief Program SysTick for `tps` and start its interrupt
     *
     * The reload is clock_hz(clock) / tps - 1 (SysTick counts down to
     * zero, so N + 1 cycles per period). Returns false and starts
     * nothing when that value does not fit the 24-bit RELOAD field or
     * comes out zero - a CPU too fast (or too slow) for the requested
     * rate is a fact the caller must see, not a silently wrong timebase.
     *
     * Call once after the clock init and before interrupts are enabled.
     * The SysTick_Handler vector must call tick().
     */
    template <typename C>
    static bool init(C clock) {
        static_assert(clock_follows<C, BasicTicker>(),
                      "brio BasicTicker: SysTick is clocked from the CPU clock, so a "
                      "dynamic clock must list the ticker among the users it rebases "
                      "- see the caveat in this file's header");

        const uint32_t reload = clock_hz(clock) / tps;
        if (reload == 0u || reload > SysTick_LOAD_RELOAD_Msk + 1u) {
            return false;
        }

        m_ticks = 0;
        m_secs = 0;
        m_frac = 0;

        SysTick->CTRL = 0;                  // stop before reprogramming
        SysTick->LOAD = reload - 1u;
        SysTick->VAL = 0;                   // any write clears the counter
        // CLKSOURCE = 1: the CPU clock. The alternative reference clock
        // is not implemented on this device (CALIB.NOREF), and CALIB is
        // never read - the reload comes from Clock::hz, which is also
        // why errata item 1.8.6 (wrong CALIB on rev B) cannot bite.
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk |
                        SysTick_CTRL_ENABLE_Msk;
        return true;
    }

    /**
     * @brief SysTick interrupt body - call from SysTick_Handler()
     *
     * No flag to clear: reading CTRL.COUNTFLAG would clear it, and the
     * exception is cleared by entry. Advances the tick counter, then the
     * exact second boundary.
     */
    // always_inline: an ISR body has exactly one call site (the vector
    // binding in the app), so inlining costs no flash and lets the
    // compiler save only the registers it actually uses.
    [[gnu::always_inline]] static void tick() {
        ++m_ticks;
        if (++m_frac >= tps) {
            m_frac = 0;
            ++m_secs;
        }
    }

    /// Current timestamp: whole seconds + millisecond fraction. The two
    /// counters are read under a guard so they belong to one instant.
    static void now(TimeStamp& out) {
        uint16_t frac;
        {
            InterruptGuard guard;
            out.seconds = m_secs;
            frac = m_frac;
        }
        out.millis = static_cast<uint16_t>(frac * millis_per_tick);
    }

    /// Raw 32-bit tick count since init() (wraps: 49.7 days @ 1000 Hz)
    static uint32_t ticks() { return read_shared(m_ticks); }

    /// Milliseconds since init(): exact, no drift (wraps ~49.7 days)
    static uint32_t millis() { return read_shared(m_ticks) * millis_per_tick; }

    /// Exact seconds since init() (wraps after ~136 years)
    static uint32_t secs() { return read_shared(m_secs); }

    /**
     * Advance kernel time by `n` ticks IN ONE STEP - the resync verb the
     * standby caveat above calls designed work, and the timed sleep site
     * (samc/sleep.hpp, SamTimedSleepSite) is its one intended caller: it
     * measures on the RTC how long the counters stood still and hands
     * the frozen span back here, so millis() stays honest against the
     * wall clock and time events matured during the standby are seen as
     * due on the very next loop pass (TimeEvents compares wrap-safe
     * differences, so a jump needs no kernel cooperation).
     *
     * Thread context only, under the guard so the handler's own tick
     * cannot interleave a half-updated second boundary. The caller is
     * responsible for `n` being the FROZEN span, not the whole slept
     * span - SysTick keeps counting whenever the CPU is awake, and
     * advancing time the handler already counted would mature events
     * EARLY, which brio's time contract (kernel/time.hpp: at least,
     * never early) forbids.
     */
    static void advance(uint32_t n) {
        InterruptGuard guard;
        m_ticks += n;
        const uint32_t f = static_cast<uint32_t>(m_frac) + n;
        m_secs += f / tps;
        m_frac = static_cast<uint16_t>(f % tps);
    }

    /// Stop the periodic interrupt without losing the counters. Time
    /// stands still while paused; resume() picks up where it was. The
    /// hardware counter keeps running underneath (SysTick has no pause),
    /// so the first tick after resume() can be a short one - accepted
    /// deliberately: this is the escape hatch for a CPU momentarily too
    /// slow to serve the tick, not a metrology verb.
    static void pause() {
        SysTick->CTRL = SysTick->CTRL & ~SysTick_CTRL_TICKINT_Msk;
    }
    static void resume() {
        SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_TICKINT_Msk;
    }
};

/**
 * Hold the SysTick INTERRUPT off for a scope, and put it back exactly as
 * it was found.
 *
 * This exists for one reason: ERRATUM 1.8.13 (DS80000740S, live on every
 * E/G/J revision including this one). With the standby back-bias option
 * set - STDBYCFG.BBIASHS, whose RESET value is 1 - a SysTick interrupt
 * that coincides exactly with a standby entry can raise a hard fault,
 * and the workaround is to disable that interrupt before entering
 * standby and re-enable it after. Both preconditions are the default
 * state of a brio program, since the SysTick interrupt IS the kernel
 * tick.
 *
 * It lives here rather than in samc/sleep.hpp because this file owns the
 * SysTick register; `SamPlatform::idle()` and `Pm::sleep()` are its two
 * users. It costs nothing in ticks: the counter is frozen across a
 * standby whether or not its interrupt is enabled (see the caveat in
 * this file's header).
 *
 * Restoring rather than unconditionally setting is what makes it safe to
 * nest inside a `pause()`d ticker.
 */
struct SysTickInterruptGuard {
    SysTickInterruptGuard() : saved_(SysTick->CTRL & SysTick_CTRL_TICKINT_Msk) {
        SysTick->CTRL = SysTick->CTRL & ~SysTick_CTRL_TICKINT_Msk;
    }
    ~SysTickInterruptGuard() { SysTick->CTRL = SysTick->CTRL | saved_; }

    SysTickInterruptGuard(const SysTickInterruptGuard&) = delete;
    SysTickInterruptGuard& operator=(const SysTickInterruptGuard&) = delete;

private:
    uint32_t saved_;
};

/// The project-wide time base on this target: 1000 ticks/s (1 ms).
/// Change the rate here; everything (time events, apps) follows the alias.
using Ticker = BasicTicker<1000>;

} // namespace brio
