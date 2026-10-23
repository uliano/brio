/*
 * delay.hpp
 *
 * A wait shorter than a tick, for a driver's settling time or a bus's
 * setup time: a microsecond busy-wait on the STK counter of the CH32V203
 * and the CH32V303, which spins the calling context for the time asked
 * and returns. A call, half a millisecond:
 *
 *     (void)delay_us(clock, 500u);
 *
 * `clock` is the clock task's tag (ch32vx03/clock.hpp), and the factor
 * comes from the rate its type claims, so a wait timed before the clock
 * task ran is off by the ratio of the two rates: longer than asked
 * under a type faster than the 8 MHz reset clock, shorter under a
 * slower one (HPRE divides SYSCLK down to an HCLK below the HSI's
 * rate, and Clock<ClockSource::internal, 1'000'000> is such a type) -
 * derived from the factor arithmetic, not measured. The answer is
 * [[nodiscard]]: true when the time was served, false - no time spent -
 * for a request of one tick period or more, at or above the 65536 us
 * gate, or with the counter stopped, and a program that ignores a
 * refusal says so with the (void). Legal from any context, with
 * interrupts masked or not; it masks nothing itself, and it needs the
 * ticker running (ch32vx03/ticker.hpp), refusing otherwise. The spin is
 * the time asked plus the polling granularity - some 40 cycles late on
 * both bench parts and never early (docs/ch32vx03/platform.md, "Bench
 * findings") - and an interrupt taken during it can only make it end
 * later.
 *
 * It is cortexm/delay.hpp's contract on this core - "at least", never early,
 * capped below one tick period (a wait of a millisecond or more is
 * TimeEvent territory and is refused, false and no time spent, rather
 * than served as a latency bug), no division at wait time.
 *
 * The counter counts up from zero to the compare and reloads (STRE), so
 * each poll's delta is (now - last) with one period folded in across the
 * reload - the mirror image of SysTick's down-count. It runs at HCLK
 * (STCLK = 1, ticker.hpp) so the counter's low half is the current
 * tick's phase in CPU cycles: 6.9 ns of resolution at 144 MHz. Reading
 * it has no side effect, and the wait never consults the tick count, so
 * it is correct with interrupts masked as anywhere else. Only CNTL is
 * read: the period is far below 2^32 cycles, so the high half never
 * moves inside one tick.
 *
 * No division at wait time. This core has the M extension and could
 * divide, but a division per wait is work bought for nothing: the one
 * division lives in delay_rate(), folded to a constant for a
 * compile-time Clock, and the wait runs in 32 bits with the 65536 us
 * gate keeping the product from wrapping.
 *
 * Under a DynamicClock the wait dispatches by rate index into a table
 * expanded at compile time (cortexm/delay.hpp's shape).
 */

#pragma once

#include <stdint.h>

#include <array>

#include "ch32vx03/device.hpp"
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

/// Busy-wait at least `us` microseconds on the STK counter. Any
/// context, masked or not; it masks nothing.
///
/// True when the time was served; false - and no time is spent - when
/// STK is not running, when the factor is zero, when `us` is 65536 or
/// more (the gate that keeps the product in 32 bits), or when the
/// request is one tick period or more.
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

/// The per-rate factors of a dynamic clock, expanded at compile time
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

/// The form a program calls, with the clock task's tag: a static clock's
/// factor folded at compile time, a dynamic one's read from the table
/// above at its current rate index. The answer is the one above.
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
