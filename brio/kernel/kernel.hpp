/*
 * kernel.hpp
 *
 * The brio cooperative kernel ("QV-style"): active objects known at
 * compile time, priority = position in the pack (first = highest), one
 * event per iteration always rescanning from the top, run-to-completion
 * dispatch, IDLE sleep when nothing is pending.
 *
 * The loop, in words: process matured time events; pop ONE event from
 * the highest-priority non-empty queue and dispatch it; if every queue
 * was empty, re-check under the critical section and go idle - the
 * platform's idle() re-enables interrupts immediately followed by the
 * sleep instruction, so no wakeup can slip between the check and the
 * sleep (the lost-wakeup race is closed by the silicon).
 *
 * Starvation of low-priority AOs under fixed priority is by definition
 * a sizing/design error; the per-queue overflow counters make it
 * visible. All AOs run on the single main stack, one at a time.
 *
 * What an AO must provide to sit in the pack is the ActiveObject
 * contract (kernel/active_object.hpp); what the machine must provide is
 * the Platform contract (kernel/platform.hpp). Because priority IS the
 * pack order, the pack also carries the one ordering fact the payload
 * rule needs: an AO lending Lease::dispatch payloads (kernel/
 * borrowed.hpp) declares `using LendsTo = Subscribers<...>` and Kernel
 * static_asserts that every borrower precedes it.
 *
 * Host tests drive init_all()/step() directly (run() never returns).
 */

#pragma once

#include <stddef.h>

#include "kernel/active_object.hpp"
#include "kernel/platform.hpp"
#include "kernel/post.hpp"
#include "kernel/time_event.hpp"

namespace brio {

/// The pack as a type, so ordering questions can be asked of it.
template <typename... Aos>
struct Pack {
    /// Position of Ao in the pack (= its priority, 0 = highest);
    /// sizeof...(Aos) when absent.
    template <typename Ao>
    static constexpr size_t index() {
        size_t i = 0;
        size_t found = sizeof...(Aos);
        (void)((std::same_as<Ao, Aos> ? (found = i, true) : (++i, false)) || ...);
        return found;
    }

    /// True when every listed borrower precedes Lender in the pack - the
    /// condition that makes a Lease::dispatch loan correct by construction.
    template <typename Lender, typename... Borrowers>
    static constexpr bool borrowers_precede(Subscribers<Borrowers...>) {
        return ((index<Borrowers>() < index<Lender>()) && ...);
    }

    /// An AO lending Lease::dispatch payloads names its borrowers with
    /// `using LendsTo = Subscribers<B1, B2, ...>` (kernel/borrowed.hpp);
    /// AOs without LendsTo trivially pass.
    template <typename Ao>
    static constexpr bool lends_ok() {
        if constexpr (requires { typename Ao::LendsTo; }) {
            return borrowers_precede<Ao>(typename Ao::LendsTo{});
        } else {
            return true;
        }
    }
};

template <Platform P, ActiveObject... Aos>
    requires (sizeof...(Aos) > 0)
class Kernel {
    static_assert((Pack<Aos...>::template lends_ok<Aos>() && ...),
                  "a Lease::dispatch borrower must precede its lender in "
                  "the Kernel pack (see kernel/borrowed.hpp)");

public:
    Kernel() = delete;

    /// Start every AO (in pack order) before the first event is served.
    static void init_all() {
        (Aos::init(), ...);
    }

    /// Serve at most ONE event: highest-priority non-empty queue wins.
    /// Returns false when every queue was empty at the moment it looked.
    static bool step() {
        return (try_one<Aos>() || ...);
    }

    /// Sleep if - re-checked with interrupts masked - nothing is pending.
    static void idle_if_empty() {
        typename P::CriticalSection cs;
        if ((Aos::queue.empty() && ...)) {
            P::idle();   // re-enables interrupts, then sleeps: race-free
        }
    }

    [[noreturn]] static void run() {
        init_all();
        for (;;) {
            TimeEvents<P>::process();
            if (!step()) {
                idle_if_empty();
            }
        }
    }

private:
    template <typename Ao>
    static bool try_one() {
        auto e = Ao::queue.pop();      // atomic in itself
        if (!e.has_value()) {
            return false;
        }
        Ao::dispatch(*e);              // run-to-completion, interrupts free
        return true;
    }
};

} // namespace brio
