/*
 * ticker.hpp
 *
 * The kernel timebase of this target, on the Cortex-M0+ SysTick.
 *
 * WHY SYSTICK AND NOT A TIM. SysTick is core-private: no application can
 * use it for PWM, capture or anything else, so claiming it costs the app
 * nothing - every TIM, the LPTIMs and the RTC stay free. That is the same
 * rule the AVR side follows by taking the RTC's PIT (a timer nothing
 * else wants), and the samc side by taking this same SysTick.
 *
 * THE TICKER ITSELF IS THE CORE STRATUM'S: `BasicTicker` lives in
 * armv6m/ticker.hpp - this family's arrival is what factored it out of
 * the SAM's file, the two having been twins line for line. What is
 * STM32G0 about this file is what stays in it: the `Ticker` alias that
 * fixes the project-wide rate, and this comment's account of what the
 * Stop modes do to a core-clocked timebase. The class's own contract is
 * documented where the class is.
 *
 * Monostate, exactly like the other two: every member is a static
 * inline, there is one timebase per program because there is one
 * SysTick, and the ISR body reaches its counters with no pointer
 * indirection. State lives in .bss, zeroed before main().
 *
 * ## Time representations
 *  - ticks():  raw 32-bit tick counter (wraps: 49.7 days @ 1000 Hz).
 *  - millis(): milliseconds - EXACT here, see below.
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
 * SysTick is clocked from HCLK, so its reload is a function of
 * Clock::hz. There is no DynamicClock on this target; when one arrives
 * this ticker must either become a ClockUser (rebase the reload) or
 * move to a timer that does not follow SYSCLK. init()'s clock_follows
 * assertion is what will refuse to compile on that day.
 *
 * ## The same caveat, in its second half: STOP FREEZES THIS TIMEBASE
 * RM0444 5.3: the Stop modes stop every clock in the VCORE domain, so
 * SysTick stops and KERNEL TIME STANDS STILL for exactly as long as the
 * sleep lasts - the samc standby situation, and the same two answers
 * apply when the PWR pass arrives (a restriction site, or a timed site
 * resynchronizing from the RTC through `advance()` below). Sleep mode
 * proper (WFI with SLEEPDEEP clear, what Stm32Platform::idle() does)
 * keeps HCLK and SysTick running.
 *
 * ## Usage
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
 */

#pragma once

#include <stdint.h>

#include "stm32g0xx.h"

#include "stm32g0/nvic.hpp"
#include "armv6m/ticker.hpp"

namespace brio {

/// The project-wide time base on this target: 1000 ticks/s (1 ms).
/// Change the rate here; everything (time events, apps) follows the alias.
using Ticker = BasicTicker<1000>;

} // namespace brio
