/*
 * time.hpp
 *
 * Kernel time units and conversions. The kernel's unit is the platform
 * TICK - an opaque quantum whose rate is Platform::ticks_per_second, a
 * compile-time constant of the target (1024 on AVR Dx, typically 1000 on
 * SysTick-based targets). Nothing here assumes a power of two or a
 * millisecond tick.
 *
 * Conversion semantics is CEIL - "at least this long": a timeout of 5 ms
 * must never fire early just because the tick rate does not divide
 * milliseconds. With ticks_per_second == 1000 the conversions fold to the
 * identity at compile time.
 *
 * These functions are constexpr and meant for compile-time constants
 * (deadlines, periods declared next to their AO). They work at runtime
 * too, but the 64-bit intermediate is expensive on 8-bit targets - prefer
 * precomputing.
 */

#pragma once

#include <stdint.h>

#include "kernel/platform.hpp"

namespace brio {

/// Ticks covering AT LEAST the given milliseconds (ceil, never early).
template <Platform P>
constexpr uint32_t ticks_from_ms(uint32_t ms) {
    return static_cast<uint32_t>(
        (static_cast<uint64_t>(ms) * P::ticks_per_second + 999u) / 1000u);
}

/// Ticks covering exactly the given whole seconds.
template <Platform P>
constexpr uint32_t ticks_from_secs(uint32_t secs) {
    return secs * P::ticks_per_second;
}

} // namespace brio
