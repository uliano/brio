/*
 * delay.hpp
 *
 * Microsecond busy-waits for the STM32G0 stratum: `brio::delay_us(clock,
 * us)` spins on the SysTick counter for at least `us` microseconds -
 * a chip-select setup, an analog settle, a protocol gap - and answers
 * true when it served the time; it refuses (false, no time spent) a
 * wait of one millisecond or more, which belongs to a time event, and a
 * program whose SysTick is not running. `delay_rate` and `DelayRate`
 * are the same wait with the clock's rate converted once, for a driver
 * that waits at run-time rates. Callable from any context that may
 * spend the time, interrupts masked or not; an interrupt that runs
 * mid-wait lengthens it. The code and the reasoning are
 * cortexm/delay.hpp's, the core-stratum file this one includes
 * (docs/cortexm/README.md).
 *
 * What is the STM32G0's here is what was measured on it: VAL is 15.6 ns
 * of resolution at 64 MHz; a counted loop would be poorly deterministic,
 * since at 64 MHz the flash runs at two wait states
 * (stm32g0/flash.hpp); and nothing here survives the Stop modes, where
 * SysTick stops with HCLK.
 */

#pragma once

#include "stm32g0xx.h"

#include "cortexm/delay.hpp"
