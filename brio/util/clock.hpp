/*
 * clock.hpp (util)
 *
 * The target-independent side of brio's clock model: the contracts and
 * helpers every target's clock type and every clocked driver share.
 * Nothing here knows an oscillator or a prescaler - that is the target's
 * clock.hpp (avrdx/clock.hpp today). See docs/design/clock.md.
 *
 * A clock type C is either
 *  - static:  `C::is_static == true`, `C::hz` a compile-time constant;
 *  - dynamic: `C::is_static == false`, `C::hz()` a value, `C::set<hz>()`
 *    / `C::set(hz)` to change it, and a compile-time list of ClockUsers
 *    it rebases synchronously BEFORE switching.
 * Drivers read the rate with clock_hz(clock) - folding for the static
 * kind - and, in their init(clock), assert clock_follows<C, Driver>():
 * a driver fed by a dynamic clock must be among the users that clock
 * rebases, or it would silently keep the old rate.
 */

#pragma once

#include <stdint.h>
#include <concepts>

namespace brio {

/// What a driver must offer to be listed among a dynamic clock's users:
/// a static rebase(hz) that makes it follow the new rate (recomputing
/// divisors, first draining what it has in flight at the old rate if it
/// must). Applied where the users list is written.
template <typename U>
concept ClockUser = requires(uint32_t hz) {
    { U::rebase(hz) } -> std::same_as<void>;
};

/// The rate of any clock type: a constant for a static clock, a value
/// for a dynamic one. Drivers write `clock_hz(clock)` and get folding
/// for free when it can fold.
template <typename C>
constexpr uint32_t clock_hz(C) {
    if constexpr (C::is_static) {
        return C::hz;
    } else {
        return C::hz();
    }
}

/// For a clocked driver's init(clock): true when the clock is static
/// (nothing to follow) or when the dynamic clock lists Driver among the
/// users it rebases. static_assert(clock_follows<Clock, Driver>()).
template <typename C, typename Driver>
constexpr bool clock_follows() {
    if constexpr (C::is_static) {
        return true;
    } else {
        return C::template rebases<Driver>;
    }
}

} // namespace brio
