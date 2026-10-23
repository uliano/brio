/*
 * clock.hpp (util)
 *
 * The target-independent side of brio's clock model: the one concept
 * and the two helpers through which a clocked driver asks the program's
 * clock type for the rate, whatever the target. A program never calls
 * them itself; it names its family's clock type, initializes it first
 * in main() and passes the tag object (an empty value of the clock
 * type, standing for it in a call) to every driver that needs the rate.
 * The AVR spelling of the smallest use, as docs/design/clock.md gives
 * it:
 *
 *     using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
 *     constexpr SysClock clock;
 *     SysClock::init();                  // first in main()
 *     Serial::init(clock, 460800);       // the driver reads clock_hz(clock)
 *
 * Nothing here knows an oscillator or a prescaler: that is each
 * target's own clock.hpp. Everything here is constexpr, pure, and
 * callable from any context; nothing waits or masks. The model, its
 * realizations per family and what a caller owes at a rate change are
 * docs/design/clock.md.
 *
 * A clock type C is one of two kinds (docs/design/clock.md, "Two
 * regimes, one shape"):
 *  - static:  `C::is_static == true`, `C::hz` a compile-time constant;
 *  - dynamic: `C::is_static == false`, `C::hz()` a value, `C::set<hz>()`
 *    / `C::set(hz)` to change it, and a compile-time list of ClockUsers
 *    it rebases synchronously, in list order, before the hardware
 *    switches (docs/design/clock.md, "A rate change is a synchronous
 *    fan-out, not an event").
 * Drivers read the rate with clock_hz(clock), which folds to a constant
 * for the static kind, and, in their init(clock), assert
 * clock_follows<C, Driver>(): a driver fed by a dynamic clock must be
 * among the users that clock rebases, or it would keep the old rate
 * with nothing to say so. The assertion makes that a compile error.
 */

#pragma once

#include <stdint.h>
#include <concepts>

namespace brio {

/// What a driver must offer to be listed among a dynamic clock's users:
/// a static rebase(hz) that makes it follow the new rate (recomputing
/// divisors, first draining what it has in flight at the old rate if it
/// must). The clock's set() calls it in main context, before the
/// hardware switches; it may wait for its own hardware to go idle. The
/// concept is applied where the users list is written: a type without
/// rebase(hz) is refused there, at compile time.
template <typename U>
concept ClockUser = requires(uint32_t hz) {
    { U::rebase(hz) } -> std::same_as<void>;
};

/// The rate of any clock type, in hertz: a constant for a static clock,
/// the current value for a dynamic one. Drivers write the same
/// `clock_hz(clock)` either way, and for a static clock the divisors
/// computed from it fold at compile time.
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
/// users it rebases. The driver writes
/// static_assert(clock_follows<Clock, Driver>()), so a forgotten user
/// stops the build with the driver's message.
template <typename C, typename Driver>
constexpr bool clock_follows() {
    if constexpr (C::is_static) {
        return true;
    } else {
        return C::template rebases<Driver>;
    }
}

} // namespace brio
