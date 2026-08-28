/*
 * delay.hpp
 *
 * delay_us(clock, us): busy-wait for AT LEAST `us` microseconds, the
 * AVR implementation of the short-wait role. Reads the rate from the
 * Clock type (avrdx/clock.hpp), never from F_CPU.
 *
 * What it is for: hardware setup times inside drivers and bit-banged
 * bench code (a CS setup, a reset pulse, a bus turnaround) - single or
 * double-digit microseconds. What it is NOT for: waiting in an active
 * object. Anything long enough to be measured in milliseconds is a time
 * event (kernel/time_event.hpp): the loop keeps serving other AOs and
 * the CPU may sleep. Only pre-kernel init code (before Kernel::run) may
 * legitimately wait milliseconds here.
 *
 * NO DIVISION EVER RUNS AT WAIT TIME. The rate is compile-time
 * knowledge in every regime: a static Clock carries it in the type, and
 * a DynamicClock can only run at source_hz over one of the twelve main
 * prescalers, so its whole rate SET is in the type too and the one
 * runtime fact is WHICH rate is current - an index byte. delay_us
 * dispatches on that index into branches expanded per rate at compile
 * time (clock_divs / rate_index in avrdx/clock.hpp):
 *
 *  - `us` known at compile time: the branch folds the exact cycle count
 *    and __builtin_avr_delay_cycles emits the tightest loop - the only
 *    cost of the dynamic regime is the index dispatch;
 *  - `us` a runtime value: the branch selects a per-rate FIXED-POINT
 *    factor (ceil(rate / 4 MHz in Q4.12), a compile-time constant) and
 *    one shared tail does a 16x16 multiply, a shift and the 4-cycle
 *    loop. The factor is rounded UP at compile time and the product is
 *    rounded up again, so the wait can only land on or past the mark:
 *    "at least" holds by construction, not by a measured constant.
 *
 * The fixed-point path also serves a static clock with a runtime `us`,
 * which makes the microsecond arithmetic EXACT at every rate the
 * divisors can produce - including the sub-MHz ones, where the old
 * whole-cycles-per-us rounding overshot by up to 30x. What no
 * implementation can fix at those rates is granularity and overhead in
 * TIME: one loop turn is 4 cycles, so at 32.768 kHz it is 122 us and
 * even a handful of dispatch cycles is a whole tick - below roughly
 * 1 MHz a microsecond-denominated busy-wait is out of its domain, and
 * the honest tools are the folded path (exact in cycles) and
 * delay_cycles. The measured overhead constants of every path live in
 * docs/avrdx/platform.md; they are facts of the current compiler, not
 * part of the contract.
 *
 * Why a cycle loop here and not elsewhere: the AVR core executes one
 * instruction per known number of cycles from flash with no prefetch,
 * cache or wait states, so counting cycles IS timing. On a Cortex-M or
 * a RISC-V core the same role reads a hardware counter (DWT CYCCNT,
 * SysTick, mcycle) - same name, same "at least" semantics, its own
 * file in its own target stratum. There is no generic delay algorithm.
 */

#pragma once

#include <stdint.h>
#include <util/delay_basic.h>

#include <concepts>

#include "avrdx/clock.hpp"

namespace brio {

/// Cycles per microsecond of a rate, rounded up (24 MHz -> 24, 32768 Hz
/// -> 1). What a driver stores when it must delay by a runtime amount
/// without holding the Clock type (e.g. a per-request setup time). The
/// rounding is whole-cycle, so below 1 MHz it overstates the rate (and
/// the wait) grossly - see the header comment; a driver that lives at
/// such rates should store a delay_mult instead.
constexpr uint8_t cycles_per_us(uint32_t hz) {
    return static_cast<uint8_t>((hz + 999'999u) / 1'000'000u);
}

/// Spin a number of 4-cycle _delay_loop_2 turns, in 16-bit chunks.
inline void delay_loops(uint32_t loops) {
    while (loops > 0xFFFFu) {
        _delay_loop_2(0xFFFF);
        loops -= 0xFFFFu;
    }
    if (loops > 0) {
        _delay_loop_2(static_cast<uint16_t>(loops));
    }
}

/// Loop turns per microsecond of a rate, in Q4.12: ceil(rate_hz / 4e6 *
/// 2^12). The compile-time half of the fixed-point path; rounded UP so
/// a wait computed with it can only be long, never short.
inline constexpr uint8_t delay_fixed_shift = 12;
constexpr uint16_t delay_mult(uint32_t rate_hz) {
    const uint64_t num = static_cast<uint64_t>(rate_hz) << delay_fixed_shift;
    const uint64_t m = (num + 3'999'999u) / 4'000'000u;
    return static_cast<uint16_t>(m);
}

/// The fixed-point wait itself: one 16x16 -> 32 multiply and a shift
/// per <= 65535 us slice, no division. A single multiply site so the
/// compiler emits the 16x16 helper, not the 32x32 one; the rounding is
/// one extra loop turn per slice instead of an exact ceil - at most 4
/// cycles long, never short, cheaper than the wide add the ceil needs.
/// always_inline on purpose: with a COMPILE-TIME mult (a static clock's
/// rate) the multiply strength-reduces and the prologue disappears.
[[gnu::always_inline]] inline void delay_us_fixed_body(uint16_t mult, uint32_t us) {
    while (us != 0) {
        const uint16_t chunk =
            us > 0xFFFFu ? static_cast<uint16_t>(0xFFFFu) : static_cast<uint16_t>(us);
        const uint32_t prod = static_cast<uint32_t>(chunk) * mult;
        delay_loops((prod >> delay_fixed_shift) + 1u);
        us -= chunk;
    }
}

/// The same wait as ONE shared out-of-line function: what the dynamic
/// dispatch branches and the stored-byte path call, so twelve branches
/// cost twelve short calls and not twelve copies of the math.
[[gnu::noinline]] inline void delay_us_fixed(uint16_t mult, uint32_t us) {
    delay_us_fixed_body(mult, us);
}

/// Runtime busy-wait: at least `us` microseconds at `cpu` cycles/us -
/// the stored-byte driver pattern. Same contract and same tail as the
/// fixed-point path (cpu cycles/us = cpu/4 loops/us = cpu << 10 in
/// Q4.12), kept for callers that hold a cpu byte across a rebase. A cpu
/// of 64 or more would overflow the Q4.12 factor; it saturates instead
/// (a LONGER wait - the safe side), and no AVR Dx clock gets there.
inline void delay_us_runtime(uint8_t cpu, uint32_t us) {
    const uint16_t mult =
        cpu >= 64 ? 0xFFFFu : static_cast<uint16_t>(static_cast<uint16_t>(cpu) << 10);
    delay_us_fixed(mult, us);
}

/// Busy-wait a raw number of CPU cycles (rounded up to 4): for the
/// cases where microseconds make no sense because the clock is below
/// 1 MHz (a 32 kHz main clock).
inline void delay_cycles(uint32_t cycles) {
    delay_loops((cycles + 3u) / 4u);
}

/// Busy-wait at least `us` microseconds at a COMPILE-TIME rate: the
/// per-branch worker of delay_us. Folds completely when `us` is a
/// constant; otherwise the rate's Q4.12 factor is the only thing the
/// branch contributes - into the SHARED out-of-line tail when the call
/// comes from the dynamic dispatch (twelve branches must not carry
/// twelve copies of the math), inlined and strength-reduced when the
/// rate is a static clock's only one.
template <uint32_t rate_hz, bool from_dispatch = false>
[[gnu::always_inline]] inline void delay_us_at(uint32_t us) {
    static_assert(rate_hz > 0 && rate_hz < 64'000'000u,
                  "delay_us_at: rate out of the Q4.12 factor's range");
    if (__builtin_constant_p(us)) {
        // Folded after inlining: the builtin demands a constant, and
        // the constant_p guard keeps the runtime path for anything
        // else (and for -O0, where nothing folds).
        const uint32_t cycles = static_cast<uint32_t>(
            (static_cast<uint64_t>(us) * rate_hz + 999'999u) / 1'000'000u);
#if __has_builtin(__builtin_avr_delay_cycles)
        __builtin_avr_delay_cycles(cycles);
#else
        // A frontend without the gcc-only builtin (clang, parsing this
        // stratum for the editor): same "at least" contract through the
        // loop pair, only the sub-4-cycle exactness is lost.
        delay_cycles(cycles);
#endif
    } else if constexpr (from_dispatch) {
        delay_us_fixed(delay_mult(rate_hz), us);
    } else {
        // A static clock's one rate: the mult is a compile-time
        // constant here, so the inlined body strength-reduces.
        delay_us_fixed_body(delay_mult(rate_hz), us);
    }
}

/// The index dispatch: a compare-and-return chain over the clock's
/// discrete rate set, each branch a delay_us_at of ITS compile-time
/// rate. Recursion instead of a fold so every branch is a plain nested
/// always_inline call (the shape __builtin_constant_p folds through -
/// proven by the static path) and a hit RETURNS: the current rate's
/// index costs its position in clock_divs, and index 0 (div1, the boot
/// rate) costs one compare.
template <typename Clock, uint8_t I = 0>
[[gnu::always_inline]] inline void delay_us_dispatch(uint8_t idx, uint32_t us) {
    if constexpr (I < Clock::rate_count) {
        if (idx == I) {
            delay_us_at<Clock::rate_hz(I), true>(us);
            return;
        }
        delay_us_dispatch<Clock, I + 1>(idx, us);
    }
}

/// Busy-wait at least `us` microseconds at Clock's rate. Static clock:
/// straight to its one rate. Dynamic clock: dispatch on the current
/// rate INDEX into per-rate branches expanded from the clock's discrete
/// set (rate_count / rate_hz / rate_index) - a compare chain on a byte,
/// never an arithmetic derivation of the rate. A dynamic clock type
/// without the discrete surface falls back to the stored-byte path.
template <typename Clock>
[[gnu::always_inline]] inline void delay_us(Clock clock, uint32_t us) {
    if constexpr (Clock::is_static) {
        delay_us_at<Clock::hz>(us);
    } else if constexpr (requires {
                             { Clock::rate_count } -> std::convertible_to<uint8_t>;
                             { Clock::rate_index() } -> std::convertible_to<uint8_t>;
                             Clock::rate_hz(uint8_t{0});
                         }) {
        delay_us_dispatch<Clock>(Clock::rate_index(), us);
    } else {
        delay_us_runtime(cycles_per_us(clock_hz(clock)), us);
    }
}

} // namespace brio
