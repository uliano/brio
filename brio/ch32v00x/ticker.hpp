/*
 * ticker.hpp
 *
 * The kernel timebase of this target, on the QingKe core's own system
 * counter: `BasicTicker` is the QingKe core stratum's (qingke/ticker.hpp,
 * whose account it is), over this family's device header, and the
 * project-wide `Ticker` its 1000 Hz instance. The smallest use
 * (docs/ch32v00x/platform.md, "How to use it"):
 *
 *     BRIO_CH32_VECTOR(systick_handler) { brio::Ticker::tick(); }
 *
 *     int main() {
 *         SysClock::init();
 *         brio::Ticker::init(clock);
 *         brio::enable_interrupts();
 *     }
 *
 * This family's facts: the QingKe V2's STK is 32 bits wide, CNT and CMP
 * (the CH32V00X reference manual 6.5.4); its Standby stops the counter
 * with the core, so the platform's idle() pauses the interrupt across
 * one and the timed sleep site hands the slept span back through
 * advance() (ch32v00x/sleep.hpp); cycles_to_tick() is the CH32V003's
 * idle()'s. ch32v00x/platform.hpp uses this timebase by default.
 */

#pragma once

#include <stdint.h>

#include "ch32v00x/device.hpp"
#include "ch32v00x/pfic.hpp"
#include "qingke/ticker.hpp"

namespace brio {

/// The project-wide timebase on this target: 1000 ticks/s (1 ms).
using Ticker = BasicTicker<1000>;

} // namespace brio
