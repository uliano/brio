/*
 * ticker.hpp
 *
 * The kernel timebase of this target, on the Cortex-M0+ SysTick.
 *
 * THE TICKER ITSELF IS THE CORE STRATUM'S: `BasicTicker` lives in
 * armv6m/ticker.hpp, since it is ARMv6-M and not Raspberry Pi's. What
 * is the RP2040's here is the `Ticker` alias that fixes the
 * project-wide rate, and the two facts below.
 *
 * SysTick rides clk_sys (CTRL.CLKSOURCE = 1, what BasicTicker::init
 * writes), so its reload is a function of the clock's rate - and
 * init()'s clock_follows assertion refuses a dynamic clock that forgot
 * to list the ticker. The reference clock the core would otherwise
 * count (CLKSOURCE = 0) is the chip's 1 us tick from the watchdog's
 * tick generator (4.7.2), never used here: the reload comes from
 * Clock::hz. Nothing of this chip's sleep stops clk_sys short of the
 * DORMANT state, so kernel time does not stand still across a WFI.
 *
 * TWO SYSTICKS. SysTick is core-private and this chip has two cores,
 * so a program running a kernel on each has two timebases with equal
 * rates and different phases. BasicTicker's counters are statics keyed
 * by its template arguments, and the second one is a TAG: `CoreTicker<0>`
 * and `CoreTicker<1>` are two tickers over two SysTicks, each started
 * by its own core (init() writes the SysTick of the core that calls
 * it - the registers are core-private at one address). `Ticker` is
 * core 0's, the one every single-core program names.
 *
 * ## Usage
 * ```cpp
 * #include "rp2040/ticker.hpp"
 *
 * extern "C" void isr_systick() { brio::Ticker::tick(); }
 *
 * int main() {
 *     SysClock::init();                     // brio::Clock<...>, rp2040/clock.hpp
 *     brio::Ticker::init(clock);            // reload from the clock's rate
 *     brio::enable_interrupts();
 * }
 * ```
 */

#pragma once

#include <stdint.h>

#include "rp2040/device.hpp"

#include "rp2040/nvic.hpp"
#include "armv6m/ticker.hpp"

namespace brio {

/// The tag that keeps the two cores' tickers apart (armv6m/ticker.hpp).
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
