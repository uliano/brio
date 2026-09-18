/*
 * ticker.hpp
 *
 * The kernel timebase of this target - ONE SURFACE OVER TWO CLOCKS,
 * because the two processor architectures of this chip count time in
 * different hardware.
 *
 *  - On the Cortex-M33 half the timebase is SysTick, core-private,
 *    counting clk_sys cycles: `BasicTicker` from cortexm/ticker.hpp,
 *    the same monostate the SAM C21, the STM32G0, the RP2040 and the
 *    STM32F4 use, with this target's rate in the alias below.
 *  - On the Hazard3 half it is THE RISC-V PLATFORM TIMER in the SIO
 *    (datasheet 3.1.8): a 64-bit counter shared by both cores with a
 *    64-bit comparator per core, counting the microsecond TICK the tick
 *    generator of 8.5 divides out of clk_ref. `MtimeTicker` below is
 *    that timebase, written to BasicTicker's surface to the verb.
 *
 * Both offer ticks() / millis() / secs() / now() / tick() / advance() /
 * pause() / resume() / rebase() / ticks_per_second, so everything above
 * - the kernel's TimeEvents, the power model, an app - is written once.
 * And so is the VECTOR BINDING: on both halves the timebase's interrupt
 * is the name `isr_systick` (rp2350/src/glue/ has the two crts and the
 * reason), so an app writes
 *
 *   extern "C" void isr_systick() { brio::Ticker::tick(); }
 *
 * and is bound whichever processor the image was built for.
 *
 * THE ONLY PREPROCESSOR QUESTION ABOVE core.hpp IS ASKED IN THIS FILE,
 * twice, and both times about BRIO_RP2350_CORE_M33 - a macro this
 * stratum defines in core.hpp, never about the compiler's `__riscv`. It
 * is asked at all because an #include cannot be selected by a constant:
 * the two timebases live in different files, one of which is ARM's.
 *
 * TWO CORES, TWO TICKERS. The counters are statics keyed by the template
 * arguments, so `CoreTicker<0>` and `CoreTicker<1>` are two timebases
 * with equal rates and different phases, one per core - on the Arm half
 * two SysTicks, on the RISC-V half two comparators against one shared
 * counter. `Ticker` is core 0's, the one every single-core program
 * names.
 *
 * WHAT THE TWO DO NOT SHARE, and what a document must say: SysTick
 * counts CPU CYCLES, so its reload is a function of clk_sys and a rate
 * change is a rebase (the caveat cortexm/ticker.hpp states at length);
 * the platform timer counts a TICK off clk_ref, so a change of clk_sys
 * does not touch it at all and `rebase()` there has nothing to do. The
 * first is why a dynamic clock must list the ticker among its users; the
 * second is why the RISC-V half will not have to.
 */

#pragma once

#include <stdint.h>

#include "rp2350/core.hpp"

#include "rp2350/clock.hpp"
#include "rp2350/mtime.hpp"
#include "util/clock.hpp"
#include "util/timestamp.hpp"

#if defined(BRIO_RP2350_CORE_M33)
#include "cortexm/ticker.hpp"
#endif

namespace brio {

/// The tag that keeps the two cores' tickers apart.
template <uint8_t core>
struct CoreTag {
    static_assert(core < 2u, "the RP2350 has two cores per architecture, 0 and 1");
    static constexpr uint8_t index = core;
};

#if !defined(BRIO_RP2350_CORE_M33)

/**
 * The RISC-V platform timer as a brio timebase (3.1.8).
 *
 * ONE COUNTER, TWO COMPARATORS (rp2350/mtime.hpp holds the block: the
 * counter, the sequences its halves are read and written by, and the
 * microsecond tick it counts). MTIME is 64 bits and shared between the
 * cores; MTIMECMP is per core at one address, and the core's timer
 * interrupt stands while MTIME >= MTIMECMP. So the tick is not a
 * hardware period at all: the handler moves the comparator on by one
 * period, which makes the timebase DRIFT-FREE by construction - the
 * error of a late handler is not carried into the next deadline.
 *
 * THE UNIT IS THE TICK GENERATOR'S, not a clock's: the counter reads
 * microseconds whatever clk_sys is doing, so a rate change moves
 * nothing here.
 */
template <uint16_t tps, uint8_t core>
class MtimeTicker {
    static_assert(tps > 0, "ticks per second must be positive");
    static_assert(1000u % tps == 0u,
                  "brio MtimeTicker: the tick rate must divide 1000 exactly, so that "
                  "millis() is a whole number of milliseconds per tick - no skip "
                  "correction, no drift");
    static_assert(core < 2u, "the RP2350 has two cores per architecture, 0 and 1");

private:
    /// Milliseconds per tick: exact by the static_assert above.
    static constexpr uint16_t millis_per_tick = static_cast<uint16_t>(1000u / tps);
    /// Microseconds per tick: the comparator's step, the timer counting
    /// the microsecond tick of 8.5.
    static constexpr uint32_t period_us = 1'000'000UL / tps;

    static inline uint32_t m_ticks = 0;
    static inline uint32_t m_secs = 0;
    static inline uint16_t m_frac = 0;   ///< ticks elapsed into the current second

    template <typename T>
    static T read_shared(const T& counter) {
        return *const_cast<const volatile T*>(&counter);
    }

public:
    MtimeTicker() = delete;

    static constexpr uint16_t ticks_per_second = tps;

    /**
     * Start the timebase: the counter on its microsecond tick
     * (rp2350/mtime.hpp), the first deadline one period out, the machine
     * timer interrupt armed.
     *
     * False, and nothing started, when the counter could not be started
     * - which on this chip means clk_ref is not a whole number of
     * megahertz, so no cycle count divides it to a microsecond.
     */
    template <typename C>
    static bool init(C clock) {
        static_assert(clock_follows<C, MtimeTicker>(),
                      "brio MtimeTicker: a dynamic clock must list the ticker among the "
                      "users it rebases - even where, as here, the rebase has nothing "
                      "to do, because the pack is what proves it was considered");
        if (!Mtime::start(clock)) {
            return false;
        }
        m_ticks = 0;
        m_secs = 0;
        m_frac = 0;
        Mtime::set_compare(Mtime::now() + period_us);
        machine_timer_interrupt(true);
        return true;
    }

    /**
     * The machine timer trap's body - call it from `isr_systick()`.
     *
     * The comparator is moved on FIRST, which both clears the interrupt
     * (it stands only while the counter has passed the comparator) and
     * sets the next deadline from the LAST one, so nothing drifts.
     */
    [[gnu::always_inline]] static void tick() {
        Mtime::set_compare(Mtime::compare() + period_us);
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

    /// The microsecond counter itself, for a measurement finer than a
    /// tick: the same 64-bit ruler, in its own units.
    static uint64_t micros() { return Mtime::now(); }

    /// Advance kernel time by `n` ticks in one step - the resync verb a
    /// timed sleep site calls after a sleep that froze the timebase.
    /// Thread context only, under the guard so the handler's own tick
    /// cannot interleave a half-updated second boundary.
    static void advance(uint32_t n) {
        InterruptGuard guard;
        m_ticks += n;
        const uint32_t f = static_cast<uint32_t>(m_frac) + n;
        m_secs += f / tps;
        m_frac = static_cast<uint16_t>(f % tps);
    }

    /// The ClockUser verb, and here it has NOTHING TO DO: this timebase
    /// counts a tick divided out of clk_ref, so a change of clk_sys does
    /// not move it. The verb exists so that the two halves of this
    /// target present one surface.
    static void rebase(uint32_t) {}

    /// Stop the periodic interrupt without losing the counters. Time
    /// stands still while paused; resume() picks up from the count as it
    /// is, with the next deadline one period out - so the first tick
    /// after a resume is a full one, where the Arm half's is a short one.
    static void pause() { machine_timer_interrupt(false); }
    static void resume() {
        Mtime::set_compare(Mtime::now() + period_us);
        machine_timer_interrupt(true);
    }
};

#endif

/// One core's time base, 1000 ticks a second: its own SysTick on the Arm
/// half, its own comparator against the shared microsecond counter on
/// the RISC-V one. The named helper is what makes `CoreTicker<2>` a
/// COMPILE ERROR - an alias to a template whose tag is never completed
/// would take any number - and it costs nothing: the type it yields is
/// the timebase itself.
template <uint8_t core>
struct CoreTickerOf {
    static_assert(core < 2u, "the RP2350 has two cores per architecture, 0 and 1");
#if defined(BRIO_RP2350_CORE_M33)
    using type = BasicTicker<1000, CoreTag<core>>;
#else
    using type = MtimeTicker<1000, core>;
#endif
};

template <uint8_t core>
using CoreTicker = typename CoreTickerOf<core>::type;

/// The project-wide time base on this target, core 0's. Change the rate
/// here; everything (time events, apps) follows the alias.
using Ticker = CoreTicker<0>;

} // namespace brio
