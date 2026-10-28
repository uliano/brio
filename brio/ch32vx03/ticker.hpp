/*
 * ticker.hpp
 *
 * The kernel timebase of this target: the 1000 Hz tick the kernel's time
 * events and a program's millis() run on, counted on the QingKe core's
 * own system counter, on the CH32V203 and the CH32V303 alike.
 * `BasicTicker` is the QingKe core stratum's (qingke/ticker.hpp, whose
 * account it is), over this family's device header, and the
 * project-wide `Ticker` its 1000 Hz instance. A program starts it after
 * the clock and binds its vector, as docs/ch32vx03/platform.md's "How to
 * use it" writes it:
 *
 *     BRIO_CH32_LEAF_VECTOR(systick_handler) { brio::Ticker::tick(); }
 *
 *     int main() {
 *         SysClock::init();
 *         brio::Ticker::init(clock);
 *         brio::enable_interrupts();
 *     }
 *
 * This family's facts: the QingKe V4's STK is 64 bits wide, CNTL/CNTH
 * and CMPLR/CMPHR (QingKe V4 manual ch. 5); in the plain Sleep the
 * counter keeps counting and its interrupt is the usual wake, and a Stop
 * stops HCLK and the counter with it, across which the platform's deep
 * idle path pauses this ticker and a timed sleep site
 * (ch32vx03/sleep.hpp) hands the frozen span back through advance().
 */

#pragma once

#include <stdint.h>

#include "ch32vx03/device.hpp"
#include "ch32vx03/pfic.hpp"
#include "qingke/ticker.hpp"

namespace brio {

/// The project-wide timebase on this target: 1000 ticks/s (1 ms).
using Ticker = BasicTicker<1000>;

} // namespace brio
