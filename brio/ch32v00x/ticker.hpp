/*
 * ticker.hpp
 *
 * The kernel timebase of this target, on the QingKe core's own system
 * counter (STK, RM 6.5.4).
 *
 * WHY STK AND NOT A TIM. The same reason as on the two Cortex-M0+
 * families: the system counter is core-private, so claiming it costs
 * the application nothing - TIM1, TIM2 and every other block stay
 * free. It is not the ARM SysTick though, and this file is not
 * armv6m/ticker.hpp with different names: the counter here counts UP to
 * a compare value, reloads to zero by itself when STRE is set, and
 * raises a flag in a status register that the handler must CLEAR. A
 * core stratum shared with a second QingKe family would start from
 * this file; there is no second family yet, so it stays here (the
 * stratum-naming rule: a name widens when a real part proves it).
 *
 * Monostate: one counter, one timebase per program, and the ISR body
 * reaches its counters with no indirection. State lives in .bss.
 *
 * ## Time representations
 *  - ticks():  raw 32-bit tick counter (wraps: 49.7 days @ 1000 Hz)
 *  - millis(): milliseconds - exact, see below
 *  - secs():   exact seconds
 *  - now():    TimeStamp = whole seconds + millisecond fraction
 *
 * ## The rate is constrained so millis() is exact
 * STK counts HCLK cycles, so the rate is ours to choose, and every rate
 * this class accepts divides 1000 exactly. millis() is then a
 * multiplication by a compile-time constant, with no drift and no
 * remainder. A rate that does not divide 1000 is refused rather than
 * silently approximated.
 *
 * ## Concurrency
 * tick() runs in the STK handler. The counters are 32-bit and this core
 * loads an aligned word in one uninterruptible access (atomic_width 4),
 * so a single-counter getter needs no mask - but it does need a
 * VOLATILE read, or a polling loop over an inlined getter folds to one
 * hoisted load that never sees the handler's store. now() reads two
 * counters that must belong to the same instant, so it masks.
 *
 * ## The caveat
 * STK is clocked from HCLK, so its compare value is a function of the
 * clock's rate: a program that changes SYSCLK at run time must rebase
 * the ticker, and that is what the ClockUser contract (util/clock.hpp)
 * is for. This family has no dynamic clock yet; when it gets one, the
 * assertion in init() is what will refuse a clock that forgot the
 * ticker. What the low-power modes do to this counter is not stated
 * here because no sleep site of this stratum exists yet.
 *
 * ## Usage
 * ```cpp
 * #include "ch32v00x/ticker.hpp"
 *
 * extern "C" BRIO_CH32_INTERRUPT void systick_handler() { brio::Ticker::tick(); }
 *
 * int main() {
 *     SysClock::init();
 *     brio::Ticker::init(clock);
 *     brio::enable_interrupts();
 * }
 * ```
 */

#pragma once

#include <stdint.h>

#include "ch32v00x/device.hpp"
#include "ch32v00x/pfic.hpp"
#include "util/clock.hpp"
#include "util/timestamp.hpp"

namespace brio {

template <uint16_t tps>
class BasicTicker {
    static_assert(tps > 0, "ticks per second must be positive");
    static_assert(1000u % tps == 0u,
                  "brio BasicTicker: the tick rate must divide 1000 exactly, so "
                  "millis() stays exact (1000, 500, 250, 200, 125, 100 ... Hz)");

    static constexpr uint16_t millis_per_tick = static_cast<uint16_t>(1000u / tps);

    static inline uint32_t m_ticks = 0;
    static inline uint32_t m_secs = 0;
    static inline uint16_t m_frac = 0;   ///< ticks elapsed into the current second

    /// Read a counter the handler writes: volatile so every call is a
    /// real load (see Concurrency above).
    template <typename T>
    static T read_shared(const T& counter) {
        return *const_cast<const volatile T*>(&counter);
    }

public:
    BasicTicker() = delete;

    static constexpr uint16_t ticks_per_second = tps;

    /**
     * Program STK for `tps` and start its interrupt.
     *
     * The counter counts up from zero to CMP and, with STRE set,
     * restarts from zero there - so the compare is hclk/tps - 1 cycles.
     * Returns false and starts nothing when that comes out zero: a CPU
     * too slow for the requested rate is a fact the caller must see,
     * not a silently wrong timebase. The 32-bit compare cannot overflow
     * at any rate this silicon reaches, so there is no upper check to
     * make.
     *
     * Call once after the clock init and before interrupts are enabled;
     * the vector at index Irq::systick must call tick().
     */
    template <typename C>
    static bool init(C clock) {
        static_assert(clock_follows<C, BasicTicker>(),
                      "brio BasicTicker: STK is clocked from HCLK, so a dynamic clock "
                      "must list the ticker among the users it rebases");

        const uint32_t period = clock_hz(clock) / tps;
        if (period == 0u) {
            return false;
        }

        m_ticks = 0;
        m_secs = 0;
        m_frac = 0;

        stk()->CTLR = 0;                 // stop before reprogramming
        stk()->SR = 0;                   // a stale CNTIF would fire at once
        stk()->CNT = 0;
        stk()->CMP = period - 1u;
        // STCLK = 1: HCLK undivided, the rate Clock::hz states. The
        // HCLK/8 alternative buys nothing here - the compare is 32 bits
        // wide, so no rate needs the prescaler to be reachable.
        stk()->CTLR = stk_ste | stk_stie | stk_stclk | stk_stre;

        Pfic::enable(Irq::systick);
        return true;
    }

    /**
     * STK interrupt body - call it from the bound vector.
     *
     * The flag MUST be cleared here (write 0 to CNTIF, RM 6.5.4.2):
     * unlike an ARM SysTick exception, entry does not clear it, and a
     * handler that returns with it standing is re-entered for ever.
     */
    // always_inline: one call site (the app's vector binding), so
    // inlining costs no flash and the compiler saves only what it uses.
    [[gnu::always_inline]] static void tick() {
        stk()->SR = 0;
        ++m_ticks;
        if (++m_frac >= tps) {
            m_frac = 0;
            ++m_secs;
        }
    }

    /// Current timestamp: whole seconds + millisecond fraction, both
    /// read under a guard so they belong to one instant.
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
     * timed sleep site calls after a Standby that stopped this counter
     * (sleep.hpp): the frozen span, and only that, handed back so time
     * events matured during the sleep are seen as due on the next loop
     * pass. Under the guard, so the handler's own tick cannot interleave
     * a half-updated second boundary.
     */
    static void advance(uint32_t n) {
        InterruptGuard guard;
        m_ticks += n;
        const uint32_t f = static_cast<uint32_t>(m_frac) + n;
        m_secs += f / tps;
        m_frac = static_cast<uint16_t>(f % tps);
    }

    /// Stop the periodic interrupt without losing the counters; time
    /// stands still while paused, resume() picks up where it was.
    static void pause() { stk()->CTLR = stk()->CTLR & ~stk_stie; }
    static void resume() { stk()->CTLR = stk()->CTLR | stk_stie; }

    /// Follow a clock that changed rate: reprogram the compare, keeping
    /// the counters. The tick in progress loses its phase - late, never
    /// early, the same promise kernel/time.hpp makes.
    static void rebase(uint32_t hz) {
        const uint32_t period = hz / tps;
        if (period == 0u) {
            return;
        }
        stk()->CNT = 0;
        stk()->CMP = period - 1u;
    }
};

/// The project-wide timebase on this target: 1000 ticks/s (1 ms).
using Ticker = BasicTicker<1000>;

} // namespace brio
