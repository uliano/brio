/*
 * ticker.hpp
 *
 * The kernel timebase of this target, on the Cortex-M0+ SysTick.
 *
 * THE TICKER ITSELF IS THE CORE STRATUM'S: `BasicTicker` lives in
 * armv6m/ticker.hpp since the STM32G0 arrived with the identical class
 * (the second Cortex-M0+ family is what the naming rule factors the
 * core out at). What is SAM about this file is what stays in it - the
 * `Ticker` alias that fixes the project-wide rate, the erratum guard
 * below, and this comment's account of what standby does to a
 * core-clocked timebase. The class's own contract (rates that divide
 * 1000, the volatile reads, advance/pause/resume) is documented where
 * the class is.
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
#include "armv6m/ticker.hpp"

namespace brio {

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
