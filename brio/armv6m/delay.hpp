/*
 * delay.hpp - the CORE stratum: microsecond busy-waits on SysTick.
 *
 * "At least", never early, and CAPPED BELOW ONE KERNEL TICK by contract.
 * The SAM C21 and the STM32G0 carried this file as twins (the samc one
 * drew the boundary at birth, the stm32g0 one kept it to the verb and
 * the return value) until the code was found identical to the byte and
 * the two families' comments differed only in their numbers - so it
 * lives here, exactly as armv6m/ticker.hpp does, and each family's
 * delay.hpp is the device include, this file, and its own measured
 * facts. Nothing below reads anything but CMSIS-Core's SysTick.
 *
 * WHY THE CAP IS THE DESIGN AND NOT A LIMITATION. In a cooperative
 * kernel a dispatch that busy-waits for milliseconds starves every
 * other active object; a wait of a tick or more is TimeEvent territory
 * (kernel/time_event.hpp) and this file REFUSES it rather than serving
 * it - the misuse fails visibly (false, and no time spent) instead of
 * becoming a latency bug. What remains is exactly what a busy-wait is
 * for: hardware timing at the microsecond scale - a chip-select setup,
 * an analog settle, a protocol gap. (avrdx/delay.hpp has no such cap
 * for historical reasons; both Cortex-M0+ strata got the boundary right
 * from birth.)
 *
 * WHY SYSTICK AND NOT A COUNTED LOOP. The Cortex-M0+ has no cycle
 * counter (DWT arrives with the M3), and a counted loop is not
 * cycle-deterministic through flash wait states, prefetch and code
 * placement - the calibrated app-local spins the bench suites carried
 * before this file existed were the honest workaround, not a
 * foundation. SysTick IS a cycle counter in all but name:
 * armv6m/ticker.hpp clocks it from the processor clock (CLKSOURCE = 1)
 * and reloads it every 1/tps second, so VAL is the current tick's phase
 * in CPU cycles - 20.8 ns of resolution at the SAM C21's 48 MHz, 15.6 ns
 * at the STM32G0's 64.
 *
 * THE OWNERSHIP LINE: SysTick belongs to the Ticker IN WRITING - a
 * store to VAL clears the counter and would skew the tick - but reading
 * VAL has no side effect at all. The one CTRL read below tests ENABLE;
 * a CTRL read clears COUNTFLAG, which is harmless because the ticker
 * does not use that flag. The wait accumulates VAL deltas with the wrap
 * folded in, so it is correct across reload boundaries; it never
 * consults the tick COUNT, so it is equally correct with interrupts
 * masked - inside the samc's SysTickInterruptGuard windows, where the
 * counter runs under a masked interrupt, as anywhere else.
 *
 * WHAT THE NUMBERS MEAN: the wait is in CPU cycles, converted from
 * microseconds through clock_hz(clock) - the one truth about the rate,
 * as everywhere in brio - with the cycles-per-microsecond factor
 * rounded UP so every conversion error lands LATE (the kernel's own
 * "at least"). NO DIVISION RUNS AT WAIT TIME, EVER: the M0+ has no
 * divide instruction, so gcc calls __aeabi_uidiv even for a CONSTANT
 * divisor - about 4 us a call, measured on the SAM C21 (the DIVAS
 * arithmetic) when the first version of this file paid it on every
 * entry. The ONE division lives in delay_rate() - folded to a constant
 * with a compile-time Clock, paid once per clock change by a caller
 * that only has a runtime rate (the SpiHost's rebase shape) - and
 * everything else runs in 32 bits, because this core taxes WIDTH too:
 * a 64-bit product is another libcall of the same size (measured when
 * the second version tried one).
 *
 * THE 32-BIT PRODUCT CANNOT WRAP, and the guard that makes that true is
 * SysTick's own geometry rather than any promise of the Ticker's: the
 * request is refused at 65536 us, and above that the product would
 * still have to clear `period` - which is at most 2^24 cycles, RELOAD
 * being a 24-bit field - so the largest product this file can compute
 * is 65535 x cycles_per_us, and cycles_per_us is 64 at the faster
 * family's ceiling.
 *
 * WHAT THIS FILE DOES NOT SERVE, stated: a program with no running
 * Ticker (SysTick disabled) gets false, not a fallback loop - a counted
 * loop could only promise "at least" by overshooting wildly, and a
 * caller that wants one can write one; and nothing here survives a
 * sleep that stops the CPU clock (the SAM's standby, the STM32's Stop
 * modes), where SysTick stops with it - a busy-wait is awake by
 * definition, so that is a non-question until someone makes it one.
 *
 * Include-order contract: the family's device header first (it brings
 * the CMSIS core header with `SysTick`); the family's delay.hpp does.
 */

#pragma once

#include <stdint.h>

#if !defined(__CM0PLUS_REV) && !defined(__CM0_REV)
#error "armv6m/delay.hpp: include the family's device header first (samc/delay.hpp and stm32g0/delay.hpp do)"
#endif

#include "util/clock.hpp"

namespace brio {

/// The cycles-per-microsecond factor, precomputed. delay_us(clock, us)
/// folds it at compile time; a caller with only a RUNTIME rate (a
/// driver rebased by a DynamicClock) stores one of these at each clock
/// change and never divides at wait time.
struct DelayRate {
    uint32_t cycles_per_us = 0;
};

/// Ceil: a 48.000001 MHz claim pays 49 cycles per microsecond and a
/// 64.000001 MHz one 65 - late, never early, the at-least direction.
constexpr DelayRate delay_rate(uint32_t hz) {
    return {(hz + 999'999UL) / 1'000'000UL};
}

/**
 * @brief Busy-wait AT LEAST `us` microseconds on the SysTick counter.
 *
 * @return true when the time was served; false - AND NO TIME IS SPENT -
 * when SysTick is not running (no Ticker in this program) or when the
 * request is one tick period or more (the cap: waits of a tick or more
 * belong to TimeEvents, and a refused misuse beats a served one).
 *
 * Callable with interrupts masked (pure VAL reads, wrap folded in) and
 * from any context that is allowed to spend the time. The elapsed time
 * includes the interruptions a busy-wait suffers - an ISR that fires
 * mid-wait lengthens the wait, which is the only honest reading of
 * "at least" on a machine with interrupts.
 */
[[nodiscard]] inline bool delay_us(DelayRate rate, uint32_t us) {
    if ((SysTick->CTRL & SysTick_CTRL_ENABLE_Msk) == 0u) {
        return false;   // no Ticker: nothing here can count time
    }
    const uint32_t period = SysTick->LOAD + 1u;   // one tick, in CPU cycles

    // The cap, in 32 bits and nothing wider (the header says why width
    // costs here). A zero rate has nothing to count with; past the
    // 65536 us gate the product cannot wrap; and then a whole tick or
    // more is TimeEvent territory, refused.
    if (rate.cycles_per_us == 0u || us >= 65'536u) {
        return false;
    }
    const uint32_t cycles = us * rate.cycles_per_us;
    if (cycles >= period) {
        return false;
    }

    // Accumulated VAL deltas: the counter counts DOWN and reloads at
    // zero, so each delta is (last - now) with one period folded in
    // across a wrap. Consecutive polls are a few cycles apart, so a
    // delta can never span a whole period and the sum is exact.
    uint32_t last = SysTick->VAL;
    uint32_t elapsed = 0;
    while (elapsed < cycles) {
        const uint32_t now = SysTick->VAL;
        elapsed += (last >= now) ? (last - now) : (last + period - now);
        last = now;
    }
    return true;
}

template <typename Clock>
[[nodiscard]] bool delay_us(Clock clock, uint32_t us) {
    return delay_us(delay_rate(clock_hz(clock)), us);
}

} // namespace brio
