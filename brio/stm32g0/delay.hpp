/*
 * delay.hpp
 *
 * Microsecond busy-waits for the STM32G0 stratum: armv6m/delay.hpp's
 * `delay_us` / `delay_rate` / `DelayRate`, the core-stratum file that
 * carries the code and the reasoning (docs/armv6m/README.md). What is
 * the STM32G0's here is what was measured on it: VAL is 15.6 ns of
 * resolution at 64 MHz; a counted loop would be poorly deterministic,
 * since at 64 MHz the flash runs at TWO wait states
 * (stm32g0/flash.hpp); and nothing here survives the Stop modes, where
 * SysTick stops with HCLK.
 */

#pragma once

#include "stm32g0xx.h"

#include "armv6m/delay.hpp"
