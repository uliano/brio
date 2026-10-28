/*
 * ticker.hpp - the CORE stratum: the kernel timebase on a QingKe core's
 * system counter.
 *
 * The 1000 Hz tick the kernel's time events and a program's millis() run
 * on, counted on the QingKe core's own system counter (the STK,
 * qingke/stk.hpp), written once for the CH32V00x, the CH32V203 and the
 * CH32V303, and the CH32X035. A program never includes this file: it
 * includes its family's ticker.hpp (ch32x035/ticker.hpp and its
 * siblings), or platform.hpp, which includes that and takes this
 * timebase by default. A program starts it after the clock and binds its
 * vector:
 *
 *     BRIO_CH32_LEAF_VECTOR(systick_handler) { brio::Ticker::tick(); }
 *
 *     int main() {
 *         SysClock::init();
 *         brio::Ticker::init(clock);
 *         brio::enable_interrupts();
 *     }
 *
 * `SysClock` is the program's clock type and `clock` its tag, an object
 * of that type (the family's clock.hpp); init() reads the rate from the
 * tag's type. The leaf binding is the cheapest of the three that compile
 * (qingke/pfic.hpp's three macros): tick() is always_inline, so the body
 * calls nothing and touches no f-register, and on the CH32V303's images
 * (the F extension with the hardware prologue) BRIO_CH32_LEAF_VECTOR
 * binds it as the attributed handler with no trampoline - the tick's
 * edge to its handler's first statement 22 cycles against 24 under
 * BRIO_CH32_VECTOR's trampoline, measured on the CH32V303VCT6 (letter w).
 * On every other image, and with the hardware prologue off, the three
 * macros are one; BRIO_CH32_VECTOR(systick_handler) compiles too, and on
 * the CH32V303's images the vector guard reports it as a candidate for
 * the leaf form. The vector must be bound by the program, under the crt's
 * name for it: the kernel binds no vector, and in a program that forgets
 * it the vector stays on the crt's spin for unbound vectors - init()
 * enables the line, so the first tick after interrupts are enabled enters
 * the spin and the program stops there.
 *
 * The clock task first. init() computes the compare from the rate the
 * clock type claims, not from the hardware, and writes it once, so a
 * ticker started while the chip still runs on its 8 MHz HSI reset rate
 * ticks at the ratio of the two rates until the clock task runs, and
 * nothing refuses it: six times too slowly under the CH32V00x's and the
 * CH32X035's 48 MHz clock types, eighteen times under the CH32V203's
 * 144 MHz, and too fast under a CH32V203 type whose HCLK is below the
 * HSI's (HPRE divides SYSCLK down that far: eight times too fast at
 * 1 MHz) - derived from the compare arithmetic, not measured.
 *
 * Who calls what: init() is a setup verb, main() after the clock task
 * and before interrupts are enabled; tick() is the STK vector's body,
 * called by the program's binding and by nothing else; ticks(), millis()
 * and secs() are single loads, from any context without a mask; cycles()
 * is from any context too, and repeats its five reads until they belong
 * to one instant (util/cycle_count.hpp); now() masks for the two loads it
 * joins; advance() masks and is a timed sleep site's (ch32v00x/sleep.hpp,
 * ch32vx03/sleep.hpp; the CH32X035 has none); pause() and resume() are
 * the platform's deep idle path's (qingke/platform.hpp); cycles_to_tick()
 * is the CH32V003's idle()'s, read masked; rebase() is called by a
 * dynamic clock's set(), in main context, first and at the old rate,
 * before the new rate's sequence runs (the family's clock.hpp). Nothing
 * here waits, except cycles(), which retries until its reads are
 * consistent.
 *
 * Why STK and not a TIM: the system counter is core-private, so claiming
 * it costs the application nothing - every TIM of the chip stays free.
 * It is not the ARM SysTick though, and this file is not
 * cortexm/ticker.hpp with different names: the counter here counts up to
 * a compare value, reloads to zero by itself when STRE is set, and
 * raises a flag in a status register that the handler must clear. The
 * V2's counter and compare are 32 bits wide, the V4's 64 in two register
 * pairs, which qingke/stk.hpp reads from the family's register struct;
 * the logic below is the same on both. The V4's control register carries
 * two bits the V2's has not - MODE, the direction, and INIT, which loads
 * the counter - and this file uses neither: MODE stays clear (count up)
 * and so does INIT, the counter being written by hand. The CH32X035's
 * manual shows STK_CTLR without INIT (its bits 30:5 are reserved), which
 * is then a difference this file never meets.
 *
 * Monostate (a type with only static members, never instantiated): one
 * counter, one timebase per program, and the ISR body reaches its
 * counters with no indirection. State lives in .bss.
 *
 * ## Time representations
 *  - ticks():  raw 32-bit tick counter (wraps: 49.7 days @ 1000 Hz)
 *  - millis(): milliseconds - exact, see below
 *  - secs():   exact seconds
 *  - now():    TimeStamp = whole seconds + millisecond fraction
 *  - cycles(): HCLK cycles, the tick count and STK's position in its period
 *              composed by util/cycle_count.hpp - right in the window
 *              between the restart at the compare and this handler
 *
 * ## The rate is constrained so millis() is exact
 * STK counts HCLK cycles, so the rate is ours to choose, and every rate
 * this class accepts divides 1000 exactly. millis() is then a
 * multiplication by a compile-time constant, with no drift and no
 * remainder. A rate that does not divide 1000 is refused rather than
 * silently approximated.
 *
 * ## Concurrency
 * tick() runs in the STK handler. The counters are 32-bit and these cores
 * load an aligned word in one uninterruptible access (atomic_width 4), so
 * a single-counter getter needs no mask - but it does need a volatile
 * read, or a polling loop over an inlined getter folds to one hoisted
 * load that never sees the handler's store. now() reads two counters that
 * must belong to the same instant, so it masks.
 *
 * ## The clock and the low-power modes
 * STK is clocked from HCLK, so its compare value is a function of the
 * clock's rate: a program that changes HCLK at run time rebases the
 * ticker, and that is what the ClockUser contract (util/clock.hpp) is
 * for - a family's DynamicClock calls rebase() on every user it lists,
 * and the assertion in init() refuses a dynamic clock that does not list
 * the ticker. In the plain Sleep the kernel's idle hook enters, the
 * counter keeps counting and its interrupt is the usual wake. A deeper
 * sleep stops it: the CH32V00x's Standby stops it with the core, and a
 * CH32V203's or CH32V303's Stop stops HCLK and the counter with it; of
 * the CH32X035 the QingKe V4 manual says that it may (6.1: in Sleep the
 * SysTick keeps its clock, in a deep sleep every clock may stop). The
 * platform's deep idle path (qingke/platform.hpp's
 * idle_deep()) pauses this ticker, clears its flag and its pending line
 * before such a sleep and resumes it after, so kernel time stands still
 * across it, and a timed sleep site hands the frozen span back through
 * advance(); the CH32X035 has no sleep site to arm one.
 *
 * Include-order contract: the family's device header and its pfic.hpp
 * first, which the family's ticker.hpp includes before this file.
 */

#pragma once

#include <stdint.h>

#include <type_traits>

#include "qingke/pfic.hpp"
#include "qingke/stk.hpp"
#include "util/clock.hpp"
#include "util/cycle_count.hpp"
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

    /// The family's STK register struct, as a type that depends on the
    /// template parameter: each verb below spells the register names of
    /// both widths, and only the branch qingke/stk.hpp's Stk64 selects is
    /// checked against the struct. The verbs read and write the registers
    /// in place rather than through a helper that returns the word
    /// (qingke/stk.hpp says why).
    using Stk = std::conditional_t<(tps != 0u), StkRegs, void>;
    static Stk* regs() { return stk(); }

public:
    BasicTicker() = delete;

    static constexpr uint16_t ticks_per_second = tps;

    /**
     * Program STK for `tps` and start its interrupt (the PFIC line
     * enabled here). `clock` is the clock task's tag: the compare is
     * computed from the rate its type claims, so a ticker started before
     * the clock task ran ticks off by the ratio of the two rates, and
     * nothing refuses it (the file header). Returns true when the
     * timebase runs.
     *
     * The counter counts up from zero to the compare and, with STRE set,
     * restarts from zero there - so the compare is hclk/tps - 1 cycles.
     * Returns false and starts nothing when that comes out zero: a CPU
     * too slow for the requested rate is a fact the caller must see, not
     * a silently wrong timebase. The period always fits 32 bits at any
     * rate this silicon reaches, so there is no upper check to make, and
     * on the V4 the compare's high half is zeroed and never touched again
     * (qingke/stk.hpp).
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
        if constexpr (Stk64<Stk>) {
            regs()->CNTL = 0;            // the high halves zero: the period fits the low ones
            regs()->CNTH = 0;
            regs()->CMPLR = period - 1u;
            regs()->CMPHR = 0;
        } else {
            regs()->CNT = 0;
            regs()->CMP = period - 1u;
        }
        // STCLK = 1: HCLK undivided, the rate Clock::hz states. On the V4
        // MODE stays clear (count up) and so does INIT: the counter was
        // just written by hand. The HCLK/8 alternative buys nothing here -
        // the compare is wide enough for any period these cores can need.
        stk()->CTLR = stk_ste | stk_stie | stk_stclk | stk_stre;

        Pfic::enable(Irq::systick);
        return true;
    }

    /**
     * STK interrupt body - call it from the bound vector (the file
     * header), from nowhere else.
     *
     * The flag must be cleared here (write 0 to CNTIF: the CH32V00X
     * reference manual 6.5.4.2, the QingKe V4 manual table 5-3, the
     * CH32X035 reference manual 7.5.5.2): unlike an ARM SysTick
     * exception, entry does not clear it, and a handler that returns
     * with it standing is re-entered for ever.
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

    /// HCLK cycles since init(), from any context: the tick count and
    /// STK's position in its period, with CNTIF saying whether a restart
    /// the handler has not counted yet is in (util/cycle_count.hpp). Wraps
    /// at 2^32 - a difference of two reads is exact under 2^32 cycles -
    /// and means cycles of the standing compare: a rebase restarts the
    /// period, and a paused ticker counts nothing.
    static uint32_t cycles() {
        // A braced list is evaluated in its written order, so these are
        // the five reads in util/cycle_count.hpp's order; the two widths
        // differ in the names of the position and the period alone.
        if constexpr (Stk64<Stk>) {
            const uint32_t period = regs()->CMPLR + 1u;
            for (;;) {
                const TickerSample s{
                    .t0 = read_shared(m_ticks),
                    .p0 = (stk()->SR & stk_cntif) != 0u,
                    .position = regs()->CNTL,
                    .p1 = (stk()->SR & stk_cntif) != 0u,
                    .t1 = read_shared(m_ticks),
                };
                if (ticker_consistent(s)) {
                    return ticker_compose(s, period);
                }
            }
        } else {
            const uint32_t period = regs()->CMP + 1u;
            for (;;) {
                const TickerSample s{
                    .t0 = read_shared(m_ticks),
                    .p0 = (stk()->SR & stk_cntif) != 0u,
                    .position = regs()->CNT,
                    .p1 = (stk()->SR & stk_cntif) != 0u,
                    .t1 = read_shared(m_ticks),
                };
                if (ticker_consistent(s)) {
                    return ticker_compose(s, period);
                }
            }
        }
    }

    /// Milliseconds since init(): exact, no drift (wraps ~49.7 days)
    static uint32_t millis() { return read_shared(m_ticks) * millis_per_tick; }

    /// Exact seconds since init() (wraps after ~136 years)
    static uint32_t secs() { return read_shared(m_secs); }

    /**
     * Advance kernel time by `n` ticks in one step - the resync verb a
     * timed sleep site calls after a sleep that stopped this counter: the
     * frozen span, and only that, handed back so time events matured
     * during the sleep are seen as due on the next loop pass. Under the
     * guard, so the handler's own tick cannot interleave a half-updated
     * second boundary.
     */
    static void advance(uint32_t n) {
        InterruptGuard guard;
        m_ticks += n;
        const uint32_t f = static_cast<uint32_t>(m_frac) + n;
        m_secs += f / tps;
        m_frac = static_cast<uint16_t>(f % tps);
    }

    /// STK counts until its next match, read now: the compare less the
    /// counter (the counter runs up to the compare and restarts from 0).
    /// In HCLK cycles - init() runs the STK on HCLK. Read masked by the
    /// one caller, the CH32V003's idle(), which will not go to sleep when
    /// the tick's edge is this close (qingke/platform.hpp says why).
    [[gnu::always_inline]] static uint32_t cycles_to_tick() {
        if constexpr (Stk64<Stk>) {
            return regs()->CMPLR - regs()->CNTL;
        } else {
            return regs()->CMP - regs()->CNT;
        }
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
        if constexpr (Stk64<Stk>) {
            regs()->CNTL = 0;
            regs()->CNTH = 0;
            regs()->CMPLR = period - 1u;
            regs()->CMPHR = 0;
        } else {
            regs()->CNT = 0;
            regs()->CMP = period - 1u;
        }
    }
};

} // namespace brio
