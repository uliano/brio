/*
 * ticker.hpp
 *
 * The kernel timebase of this target, on the Cortex-M0+ SysTick: the
 * clock the kernel's time events are armed against and the default
 * timebase of Stm32g0Platform. `Ticker` is `BasicTicker<1000>`, a
 * 1000 Hz interrupt counting milliseconds; a program binds the SysTick
 * vector to its tick(), starts it from the clock's rate after the clock
 * task, and enables interrupts:
 *
 * ```cpp
 * #include "stm32g0/ticker.hpp"
 *
 * extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
 *
 * int main() {
 *     SysClock::init();                     // brio::Clock<...>, stm32g0/clock.hpp
 *     brio::Ticker::init(clock);            // reload from the clock's rate
 *     brio::enable_interrupts();
 * }
 * ```
 *
 * Who calls what: init() and rebase() are the main context's (setup, and
 * the dynamic clock's fan-out); tick() is the SysTick handler's and
 * nobody else's; the readings are callable from any context, and now()
 * masks interrupts for the two loads it must take together. The
 * ticker is a monostate - a class with static state and no instances:
 * there is one timebase per program because there is one SysTick, and
 * the ISR body reaches its counters with no pointer indirection. State
 * lives in .bss, zeroed before main().
 *
 * The ticker itself is the core stratum's: `BasicTicker` lives in
 * cortexm/ticker.hpp, since it is the Cortex-M's and not ST. What is
 * STM32G0 about this file is what stays in it: the `Ticker` alias that
 * fixes the project-wide rate, and this comment's account of what the
 * Stop modes do to a core-clocked timebase. The class's own contract is
 * documented where the class is.
 *
 * Why SysTick and not a TIM. SysTick is core-private: no application can
 * use it for PWM, capture or anything else, so claiming it costs the app
 * nothing - every TIM, the LPTIMs and the RTC stay free.
 *
 * ## Time representations
 *  - ticks():  raw 32-bit tick counter (wraps: 49.7 days @ 1000 Hz).
 *  - millis(): milliseconds - exact here, see below.
 *  - secs():   exact seconds (wraps ~136 years).
 *  - now():    TimeStamp = whole seconds + millisecond fraction.
 *
 * ## No millisecond correction, and why the rate is constrained
 * SysTick counts CPU cycles, so the rate is ours to choose, and every
 * rate this class accepts divides 1000 exactly (1000, 500, 250, 200,
 * 125, 100 ... Hz). millis() is then ticks times a compile-time
 * constant - exact, with no drift and no jitter. A rate that does not
 * divide 1000 is refused rather than silently approximated.
 *
 * ## Concurrency
 * tick() runs in the SysTick handler. The counters are 32-bit and this
 * core loads an aligned word in one uninterruptible access
 * (atomic_width 4), so no getter masks interrupts for a single counter.
 * But atomicity is not visibility: the getters read through a volatile
 * access (read_shared in cortexm/ticker.hpp) so every call performs a
 * real load - in a header-only build a polling loop over an inlined
 * getter would otherwise fold to one hoisted read that never sees the
 * handler's store (gcc -Os has been seen deleting exactly such a loop).
 * now() additionally masks: it reads two counters that must belong to
 * the same instant.
 *
 * ## The caveat that outlives this file
 * SysTick is clocked from HCLK, so its reload is a function of the
 * clock's rate. Under this target's DynamicClock (stm32g0/clock.hpp)
 * the ticker is a ClockUser - rebase(hz) reprograms the reload and
 * restarts the period, losing the phase of the tick in progress, late
 * and never early (cortexm/ticker.hpp) - and a program that minds that
 * runs tickless on the LPTIM instead, off SYSCLK altogether. init()'s
 * clock_follows assertion refuses a dynamic clock that forgot to list
 * the ticker.
 *
 * ## The same caveat, in its second half: Stop freezes this timebase
 * RM0444 5.3: the Stop modes stop every clock in the VCORE domain, so
 * SysTick stops and kernel time stands still for exactly as long as the
 * sleep lasts. stm32g0/sleep.hpp holds the two answers to that: a site
 * that restricts the depth while a deadline is near, and timed sites
 * that place an alarm on a clock which does keep running and hand the
 * frozen span back through `advance()` (cortexm/ticker.hpp). Sleep mode
 * proper (WFI with SLEEPDEEP clear, what Stm32g0Platform::idle() does)
 * keeps HCLK and SysTick running.
 */

#pragma once

#include <stdint.h>

#include "stm32g0xx.h"

#include "stm32g0/nvic.hpp"
#include "cortexm/ticker.hpp"

namespace brio {

/// The project-wide time base on this target: 1000 ticks/s (1 ms).
/// Change the rate here; everything (time events, apps) follows the alias.
using Ticker = BasicTicker<1000>;

} // namespace brio
