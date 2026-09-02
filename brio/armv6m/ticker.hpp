/*
 * ticker.hpp - the CORE stratum: the kernel timebase on SysTick.
 *
 * SysTick is ARM's, not the vendor's: a 24-bit down-counter on the
 * processor clock with one interrupt, present on every Cortex-M0+ - and
 * this monostate ticker over it is the same on the SAM C21 and the
 * STM32G0 to the instruction, which is why it lives here. What differs
 * per family sits in the family's own ticker.hpp: the alias `Ticker`
 * (the project-wide rate), any guard the family's errata demand
 * (samc/ticker.hpp's SysTickInterruptGuard), and the caveats a family's
 * sleep modes attach to a core-clocked timebase.
 *
 * WHY SYSTICK AND NOT A VENDOR TIMER. SysTick is core-private: no
 * application can use it for PWM, capture or anything else, so claiming
 * it costs the app nothing - every TC/TCC/TIM and the RTC stay free.
 * That is the same rule the AVR side follows by taking the RTC's PIT (a
 * timer nothing else wants), applied to what this core offers.
 *
 * Include-order contract: the family's device header first (it brings
 * the CMSIS core header with `SysTick`); the family's ticker.hpp does.
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
 * store (gcc -Os deleted exactly such a loop on the samc bench). now()
 * additionally masks: it reads TWO counters that must belong to the
 * same instant.
 *
 * ## The caveat that outlives this file
 * SysTick is clocked from the CPU clock, so its reload is a function of
 * Clock::hz. Neither family has a DynamicClock yet; when one arrives,
 * this ticker must either become a ClockUser (rebase the reload) or
 * move to a timer that does not follow the core - init()'s
 * clock_follows assertion is what will refuse to compile on that day.
 * And a sleep mode that stops the CPU clock stops THIS TIMEBASE: kernel
 * time stands still for the whole sleep (the SAM's standby, the
 * STM32's Stop); `advance()` is the landing point of the resync a timed
 * sleep site performs from an RTC, samc/sleep.hpp's SamTimedSleepSite
 * being the built precedent.
 */

#pragma once

#include <stdint.h>

#if !defined(__CM0PLUS_REV) && !defined(__CM0_REV)
#error "armv6m/ticker.hpp: include the family's device header first (samc/ticker.hpp and stm32g0/ticker.hpp do)"
#endif

#include "armv6m/nvic.hpp"
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
        // CLKSOURCE = 1: the processor clock. The alternative reference
        // (not implemented on the SAM C21 - CALIB.NOREF; HCLK/8 on the
        // STM32G0) is never used and CALIB is never read - the reload
        // comes from Clock::hz, which is also why a wrong CALIB (SAM
        // erratum 1.8.6, rev B) cannot bite.
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
     * Advance kernel time by `n` ticks IN ONE STEP - the resync verb a
     * timed sleep site calls after a sleep that froze the core clock:
     * it measures on an RTC how long the counters stood still and hands
     * the frozen span back here, so millis() stays honest against the
     * wall clock and time events matured during the sleep are seen as
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

} // namespace brio
