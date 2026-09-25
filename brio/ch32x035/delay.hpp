/*
 * delay.hpp
 *
 * Microsecond busy-waits on the STK counter: cortexm/delay.hpp's
 * contract on this core - "at least", never early, CAPPED BELOW ONE TICK
 * PERIOD (a wait of a millisecond or more is TimeEvent territory and is
 * REFUSED, false and no time spent, rather than served as a latency
 * bug), no division at wait time. The CH32V203's file
 * (brio/ch32vx03/delay.hpp), on the same counter.
 *
 * The counter counts UP from zero to the compare and reloads (STRE), so
 * each poll's delta is (now - last) with one period folded in across the
 * reload - the mirror image of SysTick's down-count. It runs at HCLK
 * (STCLK = 1, ticker.hpp) so the counter's low half is the current tick's
 * phase in CPU cycles: 20.8 ns of resolution at 48 MHz. Reading it has no
 * side effect, and the wait never consults the tick count, so it is
 * correct with interrupts masked as anywhere else. Only CNTL is read: the
 * period is far below 2^32 cycles, so the high half never moves inside
 * one tick.
 *
 * NO DIVISION AT WAIT TIME. This core has the M extension and could
 * divide, but a division per wait is work bought for nothing: the one
 * division lives in delay_rate(), folded to a constant for a
 * compile-time Clock, and the wait runs in 32 bits with the 65536 us gate
 * keeping the product from wrapping.
 *
 * Under a DynamicClock the wait dispatches by rate index into a table
 * expanded at compile time (cortexm/delay.hpp's shape). Below 1 MHz of
 * HCLK - the ladder's three lowest rungs - the factor rounds up to one
 * cycle per microsecond, so the wait is longer than asked, never shorter.
 */

#pragma once

#include <stdint.h>

#include <array>

#include "ch32x035/device.hpp"
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

/// Busy-wait AT LEAST `us` microseconds on the STK counter.
///
/// True when the time was served; false - AND NO TIME IS SPENT - when
/// STK is not running or when the request is one tick period or more.
[[nodiscard]] inline bool delay_us(DelayRate rate, uint32_t us) {
    if ((stk()->CTLR & stk_ste) == 0u) {
        return false;   // the counter is off: nothing here can count time
    }
    const uint32_t period = stk()->CMPLR + 1u;   // one tick period, in CPU cycles

    if (rate.cycles_per_us == 0u || us >= 65'536u) {
        return false;
    }
    const uint32_t cycles = us * rate.cycles_per_us;
    if (cycles >= period) {
        return false;
    }

    uint32_t last = stk()->CNTL;
    uint32_t elapsed = 0;
    while (elapsed < cycles) {
        const uint32_t now = stk()->CNTL;
        elapsed += (now >= last) ? (now - last) : (now + period - last);
        last = now;
    }
    return true;
}

/// The per-rate factors of a DYNAMIC clock, expanded at compile time
/// over its discrete-rate surface (rate_count, rate_hz(i)): one table in
/// flash, indexed by rate_index() at wait time - which is why no
/// division ever runs here.
template <typename Clock>
inline constexpr auto delay_rates = [] {
    std::array<DelayRate, Clock::rate_count> table{};
    for (uint8_t i = 0; i < Clock::rate_count; ++i) {
        table[i] = delay_rate(Clock::rate_hz(i));
    }
    return table;
}();

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
