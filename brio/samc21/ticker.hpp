/*
 * ticker.hpp
 *
 * The kernel timebase of this target, on the Cortex-M0+ SysTick: the
 * `Ticker` a program's SamPlatform reads its time from (samc21/
 * platform.hpp), counting 1000 ticks a second, and the guard that keeps
 * the SysTick interrupt off across a standby entry. The smallest use is
 * the Usage block at the end of this comment: the program binds
 * SysTick_Handler to Ticker::tick(), then calls Ticker::init(clock) in
 * main() after the clock task's init() and before interrupts are
 * enabled.
 *
 * Who calls what. init() is a setup call in main(), and it answers
 * false, starting nothing, when the reload the clock's rate gives does
 * not fit SysTick's 24 bits; it computes that reload from the rate the
 * clock type claims, so an init() before the clock task's runs the tick
 * at the wrong rate and nothing refuses it. tick() is the interrupt
 * body: the program's SysTick_Handler calls it and nothing else does.
 * The getters may be read from any context (the Concurrency note
 * below); now() masks interrupts for the two reads it makes. advance()
 * is the timed sleep site's (samc21/sleep.hpp), pause() and resume()
 * the escape hatch the class's own comment describes.
 * SysTickInterruptGuard is the platform's idle() and Pm::sleep()'s.
 *
 * The ticker itself is the core stratum's: `BasicTicker` lives in
 * cortexm/ticker.hpp. What is SAM about this file is what stays in it -
 * the `Ticker` alias that fixes the project-wide rate, the erratum guard
 * below, and this comment's account of what standby does to a
 * core-clocked timebase. The class's own contract (rates that divide
 * 1000, the volatile reads, advance/pause/resume) is documented where the
 * class is.
 *
 * Why SysTick and not a TC. SysTick is core-private: no application can
 * use it for PWM, capture or anything else, so claiming it costs the app
 * nothing - every TC, TCC and the RTC stay free.
 *
 * Monostate (a class whose state and functions are all static, with no
 * instance to create): every member is a static inline, there is one
 * timebase per program because there is one SysTick, and the ISR body
 * reaches its counters with no pointer indirection. State lives in
 * .bss, zeroed before main().
 *
 * ## Time representations
 *  - ticks():  raw 32-bit tick counter (wraps: 49.7 days @ 1000 Hz).
 *  - millis(): milliseconds - EXACT here, see below.
 *  - secs():   exact seconds (wraps ~136 years).
 *  - now():    TimeStamp = whole seconds + millisecond fraction.
 *
 * ## No millisecond correction, and why the rate is constrained
 * A timebase whose rate does not divide decimal milliseconds needs its
 * ISR to skip increments to keep millis() honest. Nothing of the sort
 * is needed here: SysTick counts CPU cycles, so the rate is
 * ours to choose, and every rate this class accepts divides 1000 exactly
 * (1000, 500, 250, 200, 125, 100 ... Hz). millis() is then ticks times a
 * compile-time constant - exact, with no drift and no jitter. A rate that
 * does not divide 1000 is refused rather than silently approximated.
 *
 * ## Concurrency
 * tick() runs in the SysTick handler. The counters are 32-bit and this
 * core loads an aligned word in one uninterruptible access
 * (atomic_width 4), so no getter masks interrupts for a single counter.
 * But atomicity is not visibility: the getters read through a volatile
 * access (read_shared below) so every call performs a real load - in a
 * header-only build a polling loop over an inlined getter would
 * otherwise fold to one hoisted read that never sees the handler's
 * store. Atomicity and visibility have separate explicit sources here,
 * exactly as util/ring.hpp spells it for its
 * shared indexes. now() additionally masks: it reads TWO counters that
 * must belong to the same instant.
 *
 * ## The caveat that outlives this file
 * SysTick is clocked from the CPU clock, so its reload is a function of
 * Clock::hz. There is no DynamicClock on this target; if one arrives,
 * this ticker must either become a ClockUser (rebase the reload) or
 * move to the RTC. init()'s clock_follows assertion below is what will
 * refuse to compile on that day, which is the point.
 *
 * ## The same caveat, in its second half: standby stops kernel time
 * Being clocked from the CPU clock has a consequence beyond rebasing:
 * in the PM's STANDBY sleep mode kernel time stands still for exactly as
 * long as the sleep lasts - the tick's interrupt held off across it by
 * the erratum guard below, whatever the counter does (on the bench, with
 * the debug probe attached, it was measured running through a standby:
 * docs/samc21/platform.md).
 * Time events do not fire late by a little - they fire late by the
 * whole slept duration, and `millis()` under-reports the wall clock by
 * the same amount. IDLE is unaffected: MCLK and GCLK0 keep running
 * there.
 *
 * The two answers are stated in samc21/sleep.hpp and enforced nowhere in
 * this file: `SamSleepSite` keeps the restriction (standby only with
 * no armed time event), and `SamTimedSleepSite` lifts it by
 * resynchronizing this counter from the RTC after every wake -
 * BasicTicker's `advance()` (cortexm/ticker.hpp) is that resync's
 * landing point.
 *
 * ## Usage
 * ```cpp
 * #include "samc21/ticker.hpp"
 *
 * extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
 *
 * int main() {
 *     SysClock::init();                     // brio::Clock<...>, samc21/clock.hpp
 *     brio::Ticker::init(clock);            // reload from the clock's rate
 *     brio::enable_interrupts();
 * }
 * ```
 */

#pragma once

#include <stdint.h>

#include "sam.h"

#include "samc21/nvic.hpp"
#include "cortexm/ticker.hpp"

namespace brio {

/**
 * Hold the SysTick interrupt off for a scope, and put it back exactly as
 * it was found.
 *
 * This exists for one reason: erratum 1.8.13 (DS80000740S, live on every
 * E/G/J revision including this one). With the standby back-bias option
 * set - STDBYCFG.BBIASHS, whose reset value is 1 - a SysTick interrupt
 * that coincides exactly with a standby entry can raise a hard fault,
 * and the workaround is to disable that interrupt before entering
 * standby and re-enable it after. Both preconditions are the default
 * state of a brio program, since the SysTick interrupt is the kernel
 * tick.
 *
 * It lives here rather than in samc21/sleep.hpp because this file owns the
 * SysTick register; `SamPlatform::idle()` and `Pm::sleep()` are its two
 * users. It costs nothing in ticks: kernel time stands still across a
 * standby, and this guard is what makes it so whatever the counter does
 * in there - which also keeps the tick from ending the standby every
 * period (see the caveat in this file's header).
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
