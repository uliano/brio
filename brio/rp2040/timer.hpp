/*
 * timer.hpp
 *
 * The system timer (datasheet 4.6): ONE 64-bit counter of microseconds,
 * monotonic for thousands of years, read through a latching register
 * pair or two raw halves, and four alarms that match the low 32 bits and
 * raise one interrupt line each (TIMER_IRQ_0..3). It counts the
 * watchdog's tick (4.7.2, rp2040/watchdog.hpp), which is clk_ref
 * divided down to 1 us - so with clk_ref on the crystal this is THE
 * CHIP'S RULER: a microsecond wall clock independent of clk_sys, what
 * every timing measurement of the bench suites is judged against, and
 * what SysTick (on clk_sys) is compared to. It shares the crystal with
 * everything else, so it judges ratios and arithmetic, never the
 * crystal itself.
 *
 * READING 64 BITS ON A 32-BIT BUS. TIMELR latches TIMEHR until TIMEHR is
 * read - correct for one reader, wrong the moment a second context
 * (the other core, a handler over the loop) reads TIMELR in between.
 * `now()` therefore reads the RAW pair the SDK's way: high, low, high
 * again, repeat if the high half moved. `now_low()` is the low half
 * alone, enough for any span under 71 minutes and what a bracket wants.
 *
 * ALARMS match the LOW 32 bits: at most 2^32 us ahead, and an alarm
 * set in the past by more than that fires at once... no: it fires when
 * the low half next equals the value, up to 71 minutes later. Writing
 * ALARMn arms it (ARMED bit set); the match clears ARMED and raises
 * INTR's bit, a level cleared by writing 1 to it; ARMED is written with
 * 1 to disarm. The interrupt line is INTR gated by INTE (INTF forces,
 * INTS is the masked status).
 *
 * The block is one of the reset controller's; `init` releases it and
 * starts the tick. PAUSE stops the count (a debugging verb). DBGPAUSE's
 * two bits, SET AT RESET, stop it while either core is halted by a
 * debugger - and this chip has two cores: with the bits set, a
 * breakpoint on one core, or a BKPT taken by a core a probe left
 * debug-enabled, FREEZES THE RULER OF THE OTHER CORE, whose every
 * timed wait then never ends (measured: a panic on core 1 under a
 * probe's leftover C_DEBUGEN stopped core 0 dead in a 10 ms wait). So
 * `init` CLEARS them: the timer counts through a halt, a measurement
 * across an OpenOCD halt is a measurement of the halt, and a program
 * that wants the datasheet's default asks for it with debug_pause().
 */

#pragma once

#include <stdint.h>

#include "rp2040/device.hpp"

#include "rp2040/resets.hpp"
#include "rp2040/watchdog.hpp"

namespace brio {

struct Timer {
    Timer() = delete;

    static constexpr uint8_t alarm_count = 4;

    /// The block out of reset and the tick started at 1 us from
    /// clk_ref, which rp2040/clock.hpp's Clock puts on the crystal
    /// undivided: the divider is the crystal's megahertz. False when
    /// the block did not come ready or the divider does not fit.
    template <typename Clock>
    static bool init(Clock) {
        static_assert(Clock::xtal_hz % 1'000'000u == 0u,
                      "brio Timer: the 1 us tick divides clk_ref by a whole number of "
                      "cycles, so the crystal must be a whole number of megahertz");
        if (!Resets::release(ResetBlock::timer)) {
            return false;
        }
        debug_pause(false, false);
        return WatchdogTick::start(static_cast<uint16_t>(Clock::xtal_hz / 1'000'000u));
    }

    /// Microseconds since the tick started, 64 bits, safe from any
    /// context (the raw-pair discipline in the file header).
    static uint64_t now() {
        uint32_t hi = TIMER->TIMERAWH;
        for (;;) {
            const uint32_t lo = TIMER->TIMERAWL;
            const uint32_t hi2 = TIMER->TIMERAWH;
            if (hi == hi2) {
                return (static_cast<uint64_t>(hi) << 32) | lo;
            }
            hi = hi2;
        }
    }
    /// The low 32 bits alone: 71 minutes of unsigned span.
    static uint32_t now_low() { return TIMER->TIMERAWL; }

    /// The latching read, ONE CONTEXT ONLY (the file header): TIMELR
    /// then TIMEHR.
    static uint64_t now_latched() {
        const uint32_t lo = TIMER->TIMELR;
        const uint32_t hi = TIMER->TIMEHR;
        return (static_cast<uint64_t>(hi) << 32) | lo;
    }

    // ---- alarms -------------------------------------------------------------

    static volatile uint32_t& alarm_reg(uint8_t n) {
        return reg_at(TIMER_BASE, TIMER_ALARM0_OFFSET + 4u * n);
    }
    /// Arm alarm `n` to match the low 32 bits at `at`.
    static void alarm(uint8_t n, uint32_t at) { alarm_reg(n) = at; }
    /// Arm alarm `n` `us` microseconds from now.
    static void alarm_in(uint8_t n, uint32_t us) { alarm_reg(n) = now_low() + us; }
    static bool armed(uint8_t n) { return (TIMER->ARMED & (1u << n)) != 0u; }
    static void disarm(uint8_t n) { TIMER->ARMED = 1u << n; }

    /// The alarm's interrupt enable (INTE), through the aliases.
    static void interrupt(uint8_t n, bool on) {
        if (on) { hw_set(TIMER->INTE, 1u << n); } else { hw_clear(TIMER->INTE, 1u << n); }
    }
    /// Raised (INTR), whether enabled or not; masked status (INTS).
    static bool raised(uint8_t n) { return (TIMER->INTR & (1u << n)) != 0u; }
    static bool pending(uint8_t n) { return (TIMER->INTS & (1u << n)) != 0u; }
    /// Clear the raised flag: the handler's first act.
    static void clear(uint8_t n) { TIMER->INTR = 1u << n; }
    /// Raise it in software (INTF), for a test of the path.
    static void force(uint8_t n, bool on) {
        if (on) { hw_set(TIMER->INTF, 1u << n); } else { hw_clear(TIMER->INTF, 1u << n); }
    }
    static constexpr IRQn_Type irq(uint8_t n) {
        return static_cast<IRQn_Type>(static_cast<int>(TIMER_IRQ_0_IRQn) + n);
    }

    // ---- pause ---------------------------------------------------------------

    static void pause(bool on) { TIMER->PAUSE = on ? TIMER_PAUSE_BITS : 0u; }
    static bool paused() { return (TIMER->PAUSE & TIMER_PAUSE_BITS) != 0u; }
    /// Whether a halted core (0, 1) stops the count: both set at reset,
    /// both cleared by init() (the file header).
    static void debug_pause(bool core0, bool core1) {
        TIMER->DBGPAUSE = (core0 ? TIMER_DBGPAUSE_DBG0_BITS : 0u) |
                          (core1 ? TIMER_DBGPAUSE_DBG1_BITS : 0u);
    }
};

} // namespace brio
