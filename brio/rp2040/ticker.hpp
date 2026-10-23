/*
 * ticker.hpp
 *
 * The kernel timebase of this target, on the Cortex-M0+ SysTick: the
 * 1000 Hz tick the kernel's time events are counted in. A program binds
 * the SysTick vector to the ticker's tick() and starts it after the
 * clock task (rp2040/clock.hpp; init() takes the clock type's empty tag
 * object, `constexpr SysClock clock;`):
 *
 *     extern "C" void isr_systick() { brio::Ticker::tick(); }
 *
 *     int main() {
 *         SysClock::init();                     // brio::Clock<...>, first
 *         brio::Ticker::init(clock);            // reload from the clock's rate
 *         brio::enable_interrupts();
 *         ...
 *     }
 *
 * Who calls what: init() in main(), on the core whose SysTick it is,
 * after the clock task (the reload is computed from the rate the clock
 * type claims, so a ticker started before the clock runs at the wrong
 * rate until the tree reaches that rate, and nothing refuses it); tick()
 * from the program's isr_systick and nowhere else; ticks() from any
 * context. The vector is the program's to bind: the crt aliases
 * isr_systick, as it does every handler name but isr_hardfault (a weak
 * spin of its own), weakly to Default_Handler, a spin
 * (rp2040/src/glue/startup_rp2040.cpp), so a program that forgets it
 * compiles, runs, and stops in that spin at its first tick.
 *
 * The ticker itself is the core stratum's: `BasicTicker` lives in
 * cortexm/ticker.hpp, since it is the Cortex-M's and not Raspberry
 * Pi's. What is the RP2040's here is the `Ticker` alias that fixes the
 * project-wide rate, the per-core tag, and the two facts below.
 *
 * SysTick rides clk_sys (CTRL.CLKSOURCE = 1, what BasicTicker::init
 * writes), so its reload is a function of the clock's rate - and
 * init()'s clock_follows assertion refuses a dynamic clock that forgot
 * to list the ticker. This stratum has no dynamic clock; a rate switch
 * through a second Clock::init() owes the ticker a rebase(hz)
 * (docs/rp2040/clock.md). The reference clock the core would otherwise
 * count (CLKSOURCE = 0) is the chip's 1 us tick from the watchdog's
 * tick generator (4.7.2), never used here: the reload comes from
 * Clock::hz. Nothing of this chip's sleep stops clk_sys short of the
 * DORMANT state, so kernel time does not stand still across a WFI.
 *
 * Two SysTicks. SysTick is core-private and this chip has two cores,
 * so a program running a kernel on each has two timebases with equal
 * rates and different phases. BasicTicker's counters are statics keyed
 * by its template arguments, and the second one is a tag: `CoreTicker<0>`
 * and `CoreTicker<1>` are two tickers over two SysTicks, each started
 * by its own core (init() writes the SysTick of the core that calls
 * it - the registers are core-private at one address). `Ticker` is
 * core 0's, the one every single-core program names. A two-kernel
 * program has one isr_systick for both cores and ticks the ticker of
 * the core that took it (docs/rp2040/multicore.md):
 *
 *     extern "C" void isr_systick() {
 *         if (P0::core_id() == 0) brio::CoreTicker<0>::tick(); else brio::CoreTicker<1>::tick();
 *     }
 */

#pragma once

#include <stdint.h>

#include "rp2040/device.hpp"

#include "rp2040/nvic.hpp"
#include "cortexm/ticker.hpp"

namespace brio {

/// The tag that keeps the two cores' tickers apart (cortexm/ticker.hpp).
template <uint8_t core>
struct CoreTag {
    static_assert(core < 2u, "the RP2040 has two cores, 0 and 1");
    static constexpr uint8_t index = core;
};

/// One core's time base: 1000 ticks/s (1 ms) on its own SysTick.
template <uint8_t core>
using CoreTicker = BasicTicker<1000, CoreTag<core>>;

/// The project-wide time base on this target, core 0's. Change the
/// rate here; everything (time events, apps) follows the alias.
using Ticker = CoreTicker<0>;

} // namespace brio
