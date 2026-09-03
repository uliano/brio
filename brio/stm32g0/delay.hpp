/*
 * delay.hpp
 *
 * Microsecond busy-waits for the STM32G0 stratum: armv6m/delay.hpp's
 * `delay_us` / `delay_rate` / `DelayRate` - the core file this stratum's
 * copy became when it proved identical to the samc's to the byte
 * (docs/armv6m/README.md). What is the STM32G0's here is what was
 * measured on it: VAL is 15.6 ns of resolution at 64 MHz; a counted
 * loop would be even less deterministic than on the samc, since at
 * 64 MHz the flash runs at TWO wait states (stm32g0/flash.hpp); nothing
 * here survives the Stop modes, where SysTick stops with HCLK; and
 * test_stm32_platform letter d is the suite that measures it.
 */

#pragma once

#include "stm32g0xx.h"

#include "armv6m/delay.hpp"
