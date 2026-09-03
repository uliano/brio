/*
 * delay.hpp
 *
 * Microsecond busy-waits for the SAM C21 stratum: armv6m/delay.hpp's
 * `delay_us` / `delay_rate` / `DelayRate` - the core file this stratum's
 * copy became when the STM32G0's twin proved identical to the byte
 * (docs/armv6m/README.md). What is the SAM C21's here is what was
 * measured on it: the division this file refuses to run at wait time
 * costs about 4 us a call on this core (the DIVAS campaign's arithmetic,
 * and what the first version of the wait paid on every entry); VAL is
 * 20.8 ns of resolution at 48 MHz; the wait is correct inside
 * samc/ticker.hpp's SysTickInterruptGuard windows because it never
 * consults the tick count; and test_samc_platform letter d is the suite
 * that measures it (5..900 us at-least with about 1 us of call overhead,
 * 200 x 50 us not one early, the cap and the no-Ticker refusals at
 * bracket cost).
 */

#pragma once

#include "sam.h"

#include "armv6m/delay.hpp"
#include "samc/clock.hpp"
