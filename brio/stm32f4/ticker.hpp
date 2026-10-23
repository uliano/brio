/*
 * ticker.hpp
 *
 * The kernel timebase of this target, on the Cortex-M4 SysTick: the
 * 1000 Hz tick every time event of a program is counted in, and the
 * default timebase of Stm32f4Platform (stm32f4/platform.hpp). A program
 * starts it once in main(), after the clock task and before interrupts
 * are enabled, and binds SysTick_Handler to its tick() - the Usage block
 * below is the whole of it. Who calls what: init() is main()'s, after
 * SysClock::init(), because the reload is computed from the rate the
 * clock type claims and not read from the hardware (init() refuses only
 * a reload that does not fit 24 bits); tick() is the interrupt body and
 * runs in the SysTick handler and nowhere else; ticks(), millis() and
 * secs() are single loads, legal from any context, and now() masks
 * interrupts for the two loads that must belong to one instant. The
 * class's own contract is documented where the class is
 * (cortexm/ticker.hpp).
 *
 * Why SysTick and not a TIM. SysTick is core-private: no application can
 * use it for PWM, capture or anything else, so claiming it costs the app
 * nothing - every TIM and the RTC stay free.
 *
 * The ticker itself is the core stratum's: `BasicTicker` lives in
 * cortexm/ticker.hpp, the SysTick of ARMv7-M being the SysTick of
 * ARMv6-M register for register (PM0214 4.5). What is STM32F4 about
 * this file is what stays in it: the `Ticker` alias that fixes the
 * project-wide rate, and this comment's account of what the low-power
 * modes do to a core-clocked timebase.
 *
 * The reload fits. SysTick's RELOAD is 24 bits; at 1000 Hz the reload is
 * HCLK / 1000, which at this family's 180 MHz ceiling is 180000 - well
 * inside 16.7 million. init() refuses a reload that does not fit anyway.
 *
 * The caveat that outlives this file: Stop freezes this timebase. RM0090
 * 5.3.5: the Stop mode stops every clock in the 1.2 V domain, so SysTick
 * stops and kernel time stands still for exactly as long as the sleep
 * lasts; Sleep mode proper (WFI with SLEEPDEEP clear, what
 * Stm32f4Platform::idle() does) keeps HCLK and SysTick running. The
 * sleep sites, and the timed site that repairs a frozen span through
 * advance(), are stm32f4/sleep.hpp's, not this file's.
 *
 * ## Usage
 * ```cpp
 * #include "stm32f4/ticker.hpp"
 *
 * extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
 *
 * int main() {
 *     SysClock::init();                     // brio::Clock<...>, stm32f4/clock.hpp
 *     brio::Ticker::init(clock);            // reload from the clock's rate
 *     brio::enable_interrupts();
 * }
 * ```
 */
#pragma once

#include <stdint.h>

#include "stm32f4xx.h"

#include "stm32f4/nvic.hpp"
#include "cortexm/ticker.hpp"

namespace brio {

/// The project-wide time base on this target: 1000 ticks/s (1 ms).
/// Change the rate here; everything (time events, apps) follows the alias.
using Ticker = BasicTicker<1000>;

} // namespace brio
