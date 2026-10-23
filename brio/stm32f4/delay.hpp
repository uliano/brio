/*
 * delay.hpp
 *
 * Microsecond busy-waits for the STM32F4 stratum: a short wait inside a
 * dispatch or a driver's setup, `brio::delay_us(clock, 20);`, for at
 * least that many microseconds. It spins on SysTick's VAL: it needs the
 * counter running (Ticker::init() or SysTickCounter::start()), never its
 * interrupt, so it is correct inside a masked window too; it waits, by
 * design, and is capped below one SysTick period - a millisecond or
 * more is a TimeEvent and is refused with false. The code and the
 * reasoning are cortexm/delay.hpp's `delay_us` / `delay_rate` /
 * `DelayRate`, the core-stratum file (docs/cortexm/README.md).
 *
 * What is the STM32F4's here is what holds on it: VAL is 5.6 ns of
 * resolution at 180 MHz; the core has a divide instruction (ARMv7-M's
 * UDIV), so the no-division-at-wait-time rule costs this family nothing
 * and buys it nothing - it is kept for the one code path; a counted loop
 * would be poorly deterministic behind the ART accelerator's prefetch
 * and up to five flash wait states (stm32f4/flash.hpp); and nothing here
 * survives a Stop, where SysTick stops with HCLK. The at-least contract
 * is the core file's; its numbers on this silicon are the platform
 * suite's to measure (docs/stm32f4/platform.md).
 */

#pragma once

#include "stm32f4xx.h"

#include "cortexm/delay.hpp"
