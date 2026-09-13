/*
 * ticker.hpp
 *
 * The kernel timebase of this target, on the Cortex-M4 SysTick.
 *
 * WHY SYSTICK AND NOT A TIM. SysTick is core-private: no application can
 * use it for PWM, capture or anything else, so claiming it costs the app
 * nothing - every TIM and the RTC stay free.
 *
 * THE TICKER ITSELF IS THE CORE STRATUM'S: `BasicTicker` lives in
 * armv6m/ticker.hpp, the SysTick of ARMv7-M being the SysTick of
 * ARMv6-M register for register (PM0214 4.5). What is STM32F4 about
 * this file is what stays in it: the `Ticker` alias that fixes the
 * project-wide rate, and this comment's account of what the low-power
 * modes do to a core-clocked timebase. The class's own contract is
 * documented where the class is.
 *
 * THE RELOAD FITS. SysTick's RELOAD is 24 bits; at 1000 Hz the reload is
 * HCLK / 1000, which at this family's 180 MHz ceiling is 180000 - well
 * inside 16.7 million. init() refuses a reload that does not fit anyway.
 *
 * THE CAVEAT THAT OUTLIVES THIS FILE: STOP FREEZES THIS TIMEBASE. RM0090
 * 5.3.5: the Stop mode stops every clock in the 1.2 V domain, so SysTick
 * stops and KERNEL TIME STANDS STILL for exactly as long as the sleep
 * lasts; Sleep mode proper (WFI with SLEEPDEEP clear, what
 * Stm32f4Platform::idle() does) keeps HCLK and SysTick running. The
 * sleep sites and the timed sites that repair a frozen span are this
 * family's power chapter, not this file's.
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
#include "armv6m/ticker.hpp"

namespace brio {

/// The project-wide time base on this target: 1000 ticks/s (1 ms).
/// Change the rate here; everything (time events, apps) follows the alias.
using Ticker = BasicTicker<1000>;

} // namespace brio
