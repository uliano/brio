/*
 * delay.hpp
 *
 * Microsecond busy-waits for the RP2040 stratum: armv6m/delay.hpp's
 * `delay_us` / `delay_rate` / `DelayRate`, the core-stratum file that
 * carries the code and the reasoning (docs/armv6m/README.md). What is
 * the RP2040's here is what shapes the measurement: SysTick counts
 * clk_sys, 8 ns of resolution at 125 MHz; the code runs out of a
 * quad-SPI flash through a 16 KB cache shared by both cores, so a wait
 * whose loop misses the cache pays a flash round trip - the busy-wait
 * is "at least" as everywhere, and the bench measures how much.
 */

#pragma once

#include "rp2040/device.hpp"

#include "armv6m/delay.hpp"
