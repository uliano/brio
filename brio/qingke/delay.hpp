/*
 * delay.hpp - the CORE stratum: microsecond busy-waits on a QingKe core's
 * system counter.
 *
 * A wait shorter than a tick, for a driver's settling time or a bus's
 * setup time - a pad's settling, a peripheral's start-up: a microsecond
 * busy-wait on the STK counter (qingke/stk.hpp), which spins the calling
 * context for the time asked and returns, written once for the CH32V00x,
 * the CH32V203 and the CH32V303, and the CH32X035. A program never
 * includes this file: it includes its family's delay.hpp. A call, half a
 * millisecond, after the clock task and the ticker are initialized:
 *
 *     (void)brio::delay_us(clock, 500u);
 *
 * `clock` is the clock task's tag (the family's clock.hpp), and the
 * factor comes from the rate its type claims, so a wait timed before the
 * clock task ran is off by the ratio of the two rates: longer than asked
 * under a type faster than the 8 MHz reset clock, shorter under a slower
 * one (on the CH32V203, HPRE divides SYSCLK down to an HCLK below the
 * HSI's rate, and Clock<ClockSource::internal, 1'000'000> is such a type)
 * - derived from the factor arithmetic, not measured.
 *
 * It is cortexm/delay.hpp's contract on these cores: "at least", never
 * early, capped below one tick period (a millisecond at the 1000 Hz
 * Ticker: a wait of a tick period or more is TimeEvent territory and is
 * refused, false and no time spent, rather than served as a latency
 * bug), no division at wait time. It is refused too when the STK is not
 * running (Ticker::init() not yet called), for a request of 65536 us or
 * more, and when the rate gives zero cycles a microsecond. The answer is
 * [[nodiscard]]: a caller that ignores a refusal waited for nothing, and
 * a program that means to ignore one says so with the (void).
 *
 * Who calls it: any context - main, an active object's handler, an
 * interrupt handler - with interrupts masked or not; it reads the STK's
 * registers and nothing else, and masks nothing itself. It spins for the
 * whole wait, so every other active object waits with it; an interrupt
 * taken during it counts toward the wait, which may then end later than
 * asked, never earlier. The spin is the time asked plus the polling
 * granularity: measured, delay_us(100) spends 4871 cycles for 4800 asked
 * on the CH32V006K8U6 at 48 MHz (twenty delay_us(500) take exactly ten
 * ticks; test_ch32_platform letter d), some 40 cycles late on both
 * CH32V203 and CH32V303 bench parts at 144 MHz, and 108 to 140 cycles
 * late on the CH32X035F8U6 at 48 MHz (docs/ch32vx03/platform.md and
 * docs/ch32x035/platform.md, "Bench findings").
 *
 * The counter counts up from zero to the compare and reloads (STRE), so
 * each poll's delta is (now - last) with one period folded in across the
 * reload - the mirror image of SysTick's down-count. It runs at HCLK
 * (STCLK = 1, qingke/ticker.hpp) so the counter's low word is the
 * current tick's phase in CPU cycles: 20.8 ns of resolution at the
 * CH32V00x's and the CH32X035's 48 MHz, 6.9 ns at the CH32V203's
 * 144 MHz. Reading it has no side effect, and the wait never consults
 * the tick count, so it is correct with interrupts masked as anywhere
 * else. On the V4 only the low half is read: the period is far below
 * 2^32 cycles, so the high half never moves inside one tick.
 *
 * No division at wait time. The QingKe V2 has no divide instruction
 * (RV32EmC - the multiply-only M on the CH32V006, no multiply at all on
 * the CH32V003's V2A), so gcc would call __udivsi3 even for a constant
 * divisor there; the V4 cores have the M extension and could divide, but
 * a division per wait is work bought for nothing. The one division lives
 * in delay_rate(), folded to a constant for a compile-time Clock, and the
 * wait runs in 32 bits with the 65536 us gate keeping the product from
 * wrapping.
 *
 * Under a DynamicClock the wait dispatches by rate index into a table
 * expanded at compile time (cortexm/delay.hpp's shape). Below 1 MHz of
 * HCLK - the CH32X035 ladder's three lowest rungs - the factor rounds up
 * to one cycle per microsecond, so the wait is longer than asked, never
 * shorter.
 *
 * Include-order contract: the family's device header first (it defines
 * `stk()` and `StkRegs`), which the family's delay.hpp includes before
 * this file.
 */

#pragma once

#include <stdint.h>

#include <array>

#include "qingke/stk.hpp"
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

/// The wait itself, over the family's STK register struct: each width's
/// register names spelled in place - the counter's low word CNT or CNTL
/// and the compare's CMP or CMPLR (qingke/stk.hpp) - and only the branch
/// the struct selects checked against it. Forced inline into delay_us()
/// below, the one caller.
template <class Stk>
[[gnu::always_inline]] inline bool delay_spin(uint32_t cycles_per_us, uint32_t us) {
    if ((stk()->CTLR & stk_ste) == 0u) {
        return false;   // the counter is off: nothing here can count time
    }
    Stk* const regs = stk();
    if constexpr (Stk64<Stk>) {
        const uint32_t period = regs->CMPLR + 1u;   // one tick period, in CPU cycles

        if (cycles_per_us == 0u || us >= 65'536u) {
            return false;
        }
        const uint32_t cycles = us * cycles_per_us;
        if (cycles >= period) {
            return false;
        }

        uint32_t last = regs->CNTL;
        uint32_t elapsed = 0;
        while (elapsed < cycles) {
            const uint32_t now = regs->CNTL;
            elapsed += (now >= last) ? (now - last) : (now + period - last);
            last = now;
        }
    } else {
        const uint32_t period = regs->CMP + 1u;   // one tick period, in CPU cycles

        if (cycles_per_us == 0u || us >= 65'536u) {
            return false;
        }
        const uint32_t cycles = us * cycles_per_us;
        if (cycles >= period) {
            return false;
        }

        uint32_t last = regs->CNT;
        uint32_t elapsed = 0;
        while (elapsed < cycles) {
            const uint32_t now = regs->CNT;
            elapsed += (now >= last) ? (now - last) : (now + period - last);
            last = now;
        }
    }
    return true;
}

/// Busy-wait at least `us` microseconds on the STK counter. Any
/// context, masked or not; it masks nothing.
///
/// True when the time was served; false - and no time is spent - when
/// STK is not running, when the factor is zero, when `us` is 65536 or
/// more (the gate that keeps the product in 32 bits), or when the
/// request is one tick period or more.
[[nodiscard]] inline bool delay_us(DelayRate rate, uint32_t us) {
    return delay_spin<StkRegs>(rate.cycles_per_us, us);
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
