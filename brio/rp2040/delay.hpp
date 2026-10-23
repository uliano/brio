/*
 * delay.hpp
 *
 * Microsecond busy-waits for the RP2040 stratum: `brio::delay_us(clock,
 * us)` spins the calling core for at least `us` microseconds, for a
 * pad's settle or a device's setup time. It counts the calling core's
 * SysTick, so that SysTick must be running (false otherwise), and it
 * refuses a wait of one SysTick period or more (false, no time spent: a
 * millisecond is a time event's business); it masks nothing, works with
 * interrupts masked too, and an interrupt taken inside it lengthens it.
 * The code and
 * the reasoning are cortexm/delay.hpp's (`delay_us` / `delay_rate` /
 * `DelayRate`, docs/cortexm/README.md). What is the RP2040's here is
 * what shapes the measurement: SysTick counts clk_sys, 8 ns of
 * resolution at 125 MHz; the code runs out of a quad-SPI flash through
 * a 16 KB cache shared by both cores, so a wait whose loop misses the
 * cache pays a flash round trip - the busy-wait is "at least" as
 * everywhere, and the bench measures how much (docs/rp2040/platform.md:
 * a 5 us delay took about 17 us the first time, cold, and 5 us after).
 */

#pragma once

#include "rp2040/device.hpp"

#include "cortexm/delay.hpp"
