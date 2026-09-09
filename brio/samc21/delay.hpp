/*
 * delay.hpp
 *
 * Microsecond busy-waits for the SAM C21 stratum: armv6m/delay.hpp's
 * `delay_us` / `delay_rate` / `DelayRate` (docs/armv6m/README.md).
 * What is the SAM C21's here is what was measured on it: the division
 * this file refuses to run at wait time costs about 4 us a call on this
 * core; VAL is 20.8 ns of resolution at 48 MHz; the wait is correct
 * inside samc21/ticker.hpp's SysTickInterruptGuard windows because it
 * never consults the tick count; and it holds to its contract on the
 * bench (measured: 5..900 us at-least with about 1 us of call overhead,
 * 200 x 50 us not one early, the cap and the no-Ticker refusals at
 * bracket cost).
 */

#pragma once

#include "sam.h"

#include "armv6m/delay.hpp"
#include "samc21/clock.hpp"
