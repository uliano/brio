/*
 * delay.hpp
 *
 * Microsecond busy-waits on the STK counter, for a wait shorter than a
 * tick: a pad's settling time, a peripheral's start-up. The smallest
 * use, after the clock task and the ticker are initialized:
 *
 *     if (!brio::delay_us(clock, 100)) { ... }   // refused: no time spent
 *
 * The contract, cortexm/delay.hpp's on this core: "at least", never
 * early, capped below one tick period (a millisecond at the 1000 Hz
 * Ticker): a wait of a tick period or more is
 * a TimeEvent's job and is refused, false and no time spent, rather
 * than served as a latency bug. It is refused too when the STK is not
 * running (Ticker::init() not yet called), for a request of 65536 us or
 * more, and when the rate gives zero cycles a microsecond. No division
 * at wait time. The answer is [[nodiscard]]: a caller that ignores a
 * refusal waited for nothing.
 *
 * Who calls it: any context - main, an active object's handler, an
 * interrupt handler - with interrupts masked or not; it reads the STK's
 * registers and nothing else. It spins for the whole wait, so every
 * other active object waits with it; an interrupt taken during it counts
 * toward the wait, which may then end later than asked, never earlier.
 *
 * The counter here counts up from zero to CMP and reloads (STRE), so
 * each poll's delta is (now - last) with one period folded in across
 * the reload - the mirror image of SysTick's down-count. It runs at
 * HCLK (STCLK = 1, ticker.hpp) so CNT is the current tick's phase in
 * CPU cycles: 20.8 ns of resolution at 48 MHz. Reading CNT
 * has no side effect, and the wait never consults the tick count, so
 * it is correct with interrupts masked as anywhere else.
 *
 * No division at wait time: this core has no divide instruction
 * (RV32EmC - the multiply-only M on the CH32V006, no multiply at all
 * on the CH32V003's V2A), so gcc would call __udivsi3 even
 * for a constant divisor. The one division lives in delay_rate(),
 * folded to a constant for a compile-time Clock; the wait runs in 32
 * bits, and the 65536 us gate is what keeps the product from wrapping.
 *
 * Under this target's DynamicClock the wait dispatches by rate index
 * into a table expanded at compile time (cortexm/delay.hpp's shape).
 *
 * Measured on the CH32V006K8U6 at 48 MHz: delay_us(100) spends 4871
 * cycles for 4800 asked, and twenty delay_us(500) take exactly ten
 * ticks (test_ch32_platform letter d; a suite's letter is one command
 * key of its console and the group of verdicts it runs).
 */

#pragma once

#include <stdint.h>

#include <array>

#include "ch32v00x/device.hpp"
#include "util/clock.hpp"

namespace brio {

/// The cycles-per-microsecond factor, precomputed.
struct DelayRate {
    uint32_t cycles_per_us = 0;
};

/// Ceil: late, never early.
constexpr DelayRate delay_rate(uint32_t hz) {
    return {(hz + 999'999UL) / 1'000'000UL};
}

/// Busy-wait at least `us` microseconds on the STK counter.
///
/// True when the time was served; false - and no time is spent - when
/// STK is not running, when the request is 65536 us or more, or when it
/// is one tick period or more. Any context; masks nothing.
[[nodiscard]] inline bool delay_us(DelayRate rate, uint32_t us) {
    if ((stk()->CTLR & stk_ste) == 0u) {
        return false;   // the counter is off: nothing here can count time
    }
    const uint32_t period = stk()->CMP + 1u;   // one tick period, in CPU cycles

    if (rate.cycles_per_us == 0u || us >= 65'536u) {
        return false;
    }
    const uint32_t cycles = us * rate.cycles_per_us;
    if (cycles >= period) {
        return false;
    }

    uint32_t last = stk()->CNT;
    uint32_t elapsed = 0;
    while (elapsed < cycles) {
        const uint32_t now = stk()->CNT;
        elapsed += (now >= last) ? (now - last) : (now + period - last);
        last = now;
    }
    return true;
}

/// The per-rate factors of a dynamic clock, expanded at compile time
/// over its discrete-rate surface (rate_count, rate_hz(i)): one table
/// in flash, indexed by rate_index() at wait time - which is why no
/// division ever runs here.
template <typename Clock>
inline constexpr auto delay_rates = [] {
    std::array<DelayRate, Clock::rate_count> table{};
    for (uint8_t i = 0; i < Clock::rate_count; ++i) {
        table[i] = delay_rate(Clock::rate_hz(i));
    }
    return table;
}();

/// The form a program calls: the clock task's tag picks the factor, folded
/// at compile time for a static Clock, read from delay_rates by
/// rate_index() under a DynamicClock. The same answer as above.
template <typename Clock>
[[nodiscard]] bool delay_us(Clock clock, uint32_t us) {
    if constexpr (Clock::is_static) {
        return delay_us(delay_rate(clock_hz(clock)), us);
    } else {
        (void)clock;
        return delay_us(delay_rates<Clock>[Clock::rate_index()], us);
    }
}

} // namespace brio
