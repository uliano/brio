/*
 * time.hpp
 *
 * Converting milliseconds and seconds to kernel ticks.
 *
 * The kernel counts time in ticks. A tick's rate is a constant of the
 * platform, Platform::ticks_per_second (1000 on the Cortex-M families,
 * 1024 on the AVR); nothing in the kernel assumes a power of two or a
 * millisecond tick. An application writes its delays and periods in
 * milliseconds or seconds and converts them with the two functions
 * below:
 *
 *     heartbeat.arm_every(brio::ticks_from_ms<P>(500));
 *
 * The conversion rounds up: the result is the smallest whole number of
 * ticks that covers at least the requested time, so a count is never
 * shortened by truncation when the tick rate does not divide
 * milliseconds. With ticks_per_second == 1000 the conversion folds to
 * the identity.
 *
 * What a count of ticks is worth in physical time is the timebase's
 * business, not this header's. A timer armed with N ticks matures when
 * the tick counter has advanced by N from the count read at arming, and
 * the arming instant can fall anywhere inside a tick interval, so the
 * physical wait before maturity lies between N - 1 and N tick intervals,
 * plus whatever the kernel loop adds before it notices (the "When a
 * timer matures" paragraph of kernel/time_event.hpp). A delay of one
 * tick can mature almost at once.
 *
 * Both functions are constexpr. With a constant argument the compiler
 * computes the result and the 64-bit intermediate costs nothing at run
 * time. They work with a run-time argument too, where the 64-bit
 * arithmetic is real and expensive on an 8-bit core: precompute where a
 * value is known.
 */

#pragma once

#include <stdint.h>

#include "kernel/platform.hpp"

namespace brio {

/// The smallest whole number of ticks covering at least the given
/// milliseconds (the conversion rounds up; what a tick is worth in
/// physical time is the file header's).
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
