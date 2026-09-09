/*
 * ticker.hpp - the CORE stratum: the kernel timebase on SysTick.
 *
 * SysTick is ARM's, not the vendor's: a 24-bit down-counter on the
 * processor clock with one interrupt, present on every Cortex-M0+ - and
 * this monostate ticker over it is the same on the SAM C21 and the
 * STM32G0 to the instruction, which is why it lives here. What differs
 * per family sits in the family's own ticker.hpp: the alias `Ticker`
 * (the project-wide rate), any guard the family's errata demand
 * (samc21/ticker.hpp's SysTickInterruptGuard), and the caveats a family's
 * sleep modes attach to a core-clocked timebase.
 *
 * WHY SYSTICK AND NOT A VENDOR TIMER. SysTick is core-private: no
 * application can use it for PWM, capture or anything else, so claiming
 * it costs the app nothing - every TC/TCC/TIM and the RTC stay free.
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
 * A timebase whose rate does not divide decimal milliseconds needs its
 * ISR to skip increments to keep millis() honest. Nothing
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
 * store (gcc -Os deletes exactly such a loop). now()
 * additionally masks: it reads TWO counters that must belong to the
 * same instant.
 *
 * ## The caveat that outlives this file
 * SysTick is clocked from the CPU clock, so its reload is a function of
 * the clock's rate. Under a DynamicClock (the STM32G0 has one,
 * stm32g0/clock.hpp; the SAM C21 has none) this ticker is a
 * ClockUser: `rebase(hz)` reprograms the reload and RESTARTS the period,
 * so the tick in progress at a switch is lost - kernel time runs up to
 * one tick LATE per switch, never early (the kernel's own direction) -
 * and a rescaling program that minds that runs on a timebase off the
 * core clock instead (the STM32G0's tickless one, below). init()'s
 * clock_follows assertion is what refuses a dynamic clock that forgot
 * to list the ticker.
 * And a sleep mode that stops the CPU clock stops THIS TIMEBASE: kernel
 * time stands still for the whole sleep (the SAM's standby, the
 * STM32's Stop); `advance()` is the landing point of the resync a timed
 * sleep site performs from an RTC (samc21/sleep.hpp's
 * SamTimedSleepSite). The other answer exists on the STM32G0: a
 * TICKLESS timebase on a low-power timer that counts through the Stop
 * (stm32g0/lptim_ticker.hpp, taken by stm32g0/platform.hpp as its
 * template argument), which gives up one LPTIM for the "costs the app
 * nothing" above and keeps SysTick running as delay_us's cycle counter
 * through SysTickCounter at the end of this file.
 */

#pragma once

#include <stdint.h>

#if !defined(__CM0PLUS_REV) && !defined(__CM0_REV)
#error "armv6m/ticker.hpp: include the family's device header first (samc21/ticker.hpp and stm32g0/ticker.hpp do)"
#endif

#include "armv6m/nvic.hpp"
#include "util/clock.hpp"
#include "util/timestamp.hpp"

namespace brio {

/// Static (monostate) time tracker driven by the SysTick interrupt.
/// `tps` is the tick frequency in Hz and must divide 1000 exactly.
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
     * Program SysTick for `tps` and start its interrupt.
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
     * SysTick interrupt body - call from SysTick_Handler().
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

    /**
     * The ClockUser verb: a new reload for a new CPU rate, the counters
     * and the interrupt kept. The period is RESTARTED (a VAL write clears
     * the counter and the next cycle reloads it), because a countdown
     * begun at the old rate would otherwise finish at the new one and a
     * fall from 64 to 2 MHz would stretch that one tick to 32 - so the
     * cost of a switch is the phase of the tick in progress, under one
     * tick, and it lands LATE. A rate this counter cannot serve (the same
     * 24-bit rule as init()) writes nothing and the tick then runs at the
     * old reload: there is no return value in the contract to say so,
     * and a program's rates are known at build time - a pack is checked
     * against the ticker where it is written, not here.
     */
    static void rebase(uint32_t hz) {
        const uint32_t reload = hz / tps;
        if (reload == 0u || reload > SysTick_LOAD_RELOAD_Msk + 1u) {
            return;
        }
        SysTick->LOAD = reload - 1u;
        SysTick->VAL = 0;
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
 * SysTickCounter: SysTick as a bare cycle counter, no interrupt, for a
 * program whose KERNEL TIMEBASE is elsewhere (stm32g0/lptim_ticker.hpp
 * counts kernel time on a low-power timer that runs through a Stop).
 * armv6m/delay.hpp reads SysTick's VAL against LOAD and tests ENABLE
 * only - it never needs the interrupt - so this is what keeps delay_us
 * working when BasicTicker is not the program's ticker. Same reload
 * rule and the same 24-bit refusal as BasicTicker::init(), so the
 * one-millisecond period delay_us derives from LOAD is the same on
 * both; the same clock_follows assertion, for the same reason (a
 * reload is a function of the CPU rate). No handler, no vector: a
 * program on this counter binds nothing to SysTick_Handler.
 *
 * SysTick keeps belonging to ONE writer per program: BasicTicker when
 * it is the timebase, this counter when it is not - never both.
 */
struct SysTickCounter {
    SysTickCounter() = delete;

    template <typename C>
    static bool start(C clock) {
        static_assert(clock_follows<C, SysTickCounter>(),
                      "brio SysTickCounter: SysTick is clocked from the CPU clock, so a "
                      "dynamic clock must list the counter among the users it rebases");
        return program(clock_hz(clock));
    }

    /// The ClockUser verb: a new reload for the new rate, the period
    /// restarted, CTRL untouched - the BasicTicker's own shape, so that
    /// the two writers agree on what a rebase IS (a program that hands
    /// SysTick from one to the other keeps whichever interrupt setting
    /// stands). No phase to keep here (nothing counts ticks on this
    /// counter); a rate it cannot serve leaves it STOPPED, so that
    /// delay_us refuses instead of waiting on a wrong period.
    static void rebase(uint32_t hz) {
        const uint32_t reload = hz / 1000u;
        if (reload == 0u || reload > SysTick_LOAD_RELOAD_Msk + 1u) {
            stop();
            return;
        }
        SysTick->LOAD = reload - 1u;
        SysTick->VAL = 0;
    }

    /// The counter off: delay_us then refuses (its ENABLE test).
    static void stop() { SysTick->CTRL = 0; }

    static bool running() { return (SysTick->CTRL & SysTick_CTRL_ENABLE_Msk) != 0u; }

private:
    [[gnu::always_inline]] static bool program(uint32_t hz) {
        const uint32_t reload = hz / 1000u;
        if (reload == 0u || reload > SysTick_LOAD_RELOAD_Msk + 1u) {
            return false;
        }
        SysTick->CTRL = 0;                  // stop before reprogramming
        SysTick->LOAD = reload - 1u;
        SysTick->VAL = 0;                   // any write clears the counter
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
        return true;
    }

};

} // namespace brio
