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
 * Two paths, one call (static clocks; a DynamicClock always takes the
 * runtime path from its current rate):
 *  - `us` known at compile time (the common case): the cycle count is
 *    folded and __builtin_avr_delay_cycles emits the tightest loop -
 *    exactly what avr-libc's _delay_us does, minus F_CPU;
 *  - `us` a runtime value: an integer loop of 4-cycle iterations from
 *    the constexpr cycles-per-microsecond of the clock, in 16-bit
 *    chunks. Coarse (a few cycles of call overhead are not counted, the
 *    ceil covers them for anything above a handful of us) but honest.
 * Both round UP: a setup time is "no less than".
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

#include "avrdx/clock.hpp"

namespace brio {

/// Cycles per microsecond of a rate, rounded up (24 MHz -> 24, 32768 Hz
/// -> 1). What a driver stores when it must delay by a runtime amount
/// without holding the Clock type (e.g. a per-request setup time).
constexpr uint8_t cycles_per_us(uint32_t hz) {
    return static_cast<uint8_t>((hz + 999'999u) / 1'000'000u);
}

/// Runtime busy-wait: at least `us` microseconds at `cpu` cycles/us.
/// One _delay_loop_2 iteration is 4 cycles, its count 16 bits.
inline void delay_us_runtime(uint8_t cpu, uint32_t us) {
    uint32_t loops = (us * cpu + 3u) / 4u;
    while (loops > 0xFFFFu) {
        _delay_loop_2(0xFFFF);
        loops -= 0xFFFFu;
    }
    if (loops > 0) {
        _delay_loop_2(static_cast<uint16_t>(loops));
    }
}

/// Busy-wait a raw number of CPU cycles (rounded up to 4): for the
/// cases where microseconds make no sense because the clock is below
/// 1 MHz (a 32 kHz main clock).
inline void delay_cycles(uint32_t cycles) {
    uint32_t loops = (cycles + 3u) / 4u;
    while (loops > 0xFFFFu) {
        _delay_loop_2(0xFFFF);
        loops -= 0xFFFFu;
    }
    if (loops > 0) {
        _delay_loop_2(static_cast<uint16_t>(loops));
    }
}

/// Busy-wait at least `us` microseconds at Clock's rate.
template <typename Clock>
[[gnu::always_inline]] inline void delay_us(Clock clock, uint32_t us) {
    if constexpr (Clock::is_static) {
        if (__builtin_constant_p(us)) {
            // Folded after inlining: the builtin demands a constant, and
            // the constant_p guard keeps the runtime path for anything
            // else (and for -O0, where nothing folds).
            const uint32_t cycles = static_cast<uint32_t>(
                (static_cast<uint64_t>(us) * Clock::hz + 999'999u) / 1'000'000u);
            __builtin_avr_delay_cycles(cycles);
        } else {
            delay_us_runtime(cycles_per_us(Clock::hz), us);
        }
    } else {
        // Dynamic clock: the rate is a value, so is the loop count.
        delay_us_runtime(cycles_per_us(clock_hz(clock)), us);
    }
}

} // namespace brio
