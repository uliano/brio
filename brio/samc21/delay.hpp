/*
 * delay.hpp
 *
 * Microsecond busy-waits for the SAM C21 stratum: cortexm/delay.hpp's
 * `delay_us` / `delay_rate` / `DelayRate` (docs/cortexm/README.md). The
 * smallest use is `brio::delay_us(clock, us)` with the clock task's tag:
 * it spins the caller for at least `us` microseconds, timed on SysTick's
 * counter, and answers false, spending no time, for a wait of a kernel
 * tick or more or when SysTick is not running - a wait of a tick is a time
 * event's (docs/samc21/platform.md, "Types and verbs"). It holds the
 * caller, whatever its context, for the whole wait, and masks nothing.
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

#include "cortexm/delay.hpp"
#include "samc21/clock.hpp"
