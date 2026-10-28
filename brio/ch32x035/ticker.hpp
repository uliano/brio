/*
 * ticker.hpp
 *
 * The kernel timebase of this target: the 1000 Hz tick the kernel's time
 * events and a program's millis() run on, counted on the QingKe core's
 * own system counter (STK, RM 7.5.5 and QingKe V4 manual ch. 5).
 * `BasicTicker` is the QingKe core stratum's (qingke/ticker.hpp, whose
 * account it is), over this family's device header, and the
 * project-wide `Ticker` its 1000 Hz instance. A program starts it after
 * the clock and binds its vector:
 *
 *     BRIO_CH32_VECTOR(systick_handler) { brio::Ticker::tick(); }
 *
 *     int main() {
 *         SysClock::init();
 *         brio::Ticker::init(clock);
 *         brio::enable_interrupts();
 *     }
 *
 * This family's facts: the counter is the CH32V203's, sixty-four bits in
 * two register pairs, and the one difference this series' manual shows
 * is a bit the ticker never uses - STK_CTLR has no INIT on the CH32X035
 * (its bits 30:5 are reserved); no sleep site of this stratum exists to
 * pause it across a deep sleep.
 */

#pragma once

#include <stdint.h>

#include "ch32x035/device.hpp"
#include "ch32x035/pfic.hpp"
#include "qingke/ticker.hpp"

namespace brio {

/// The project-wide timebase on this target: 1000 ticks/s (1 ms).
using Ticker = BasicTicker<1000>;

} // namespace brio
