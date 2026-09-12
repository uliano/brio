/*
 * active_object.hpp
 *
 * The ActiveObject concept: the contract between the kernel and the
 * things it schedules. This is the FORMAL half of the contract - what
 * the compiler can check; the informal half (run-to-completion, who may
 * push, scheduling order) is written below and in docs/design/kernel.md.
 *
 * An active object (AO) is a monostate class - no instances, everything
 * static, exactly like the brio drivers - that owns:
 *  - `Event`: its own event type, a std::variant of small trivially
 *    copyable structs (usually built by Fsm<Derived, Alts...>, which
 *    prepends the reserved Entry/Exit alternatives);
 *  - `queue`: a static EventQueue<Event, depth, P> - the ONE place where
 *    events for this AO wait. Depth is the AO's own sizing decision,
 *    which is why the base class cannot declare it for you;
 *  - `init()`: called once by Kernel::init_all() in pack order, before
 *    the first event is served. An Fsm-based AO calls start(&initial)
 *    here, so its first state is entered before anything can be posted;
 *  - `dispatch(const Event&)`: run ONE event to completion. Called from
 *    the kernel loop only, interrupts enabled, never re-entered: while a
 *    dispatch runs no other AO runs (single stack, cooperative), so the
 *    AO's static data needs no locking against other AOs - only ISRs
 *    are concurrent, and they touch nothing but the queue (through
 *    post(), inside a critical section).
 *
 * The concept checks only what the compiler can see: the names, the
 * signatures, that pop() yields optional<Event> and empty() a bool. It
 * does NOT check that dispatch is run-to-completion, that init() calls
 * start(), or that the queue is really an EventQueue: those are the
 * rules of the model, and Kernel/Fsm/post are written assuming them.
 *
 * Fsm is one way to satisfy the contract (Event + dispatch for free),
 * not the contract itself: an AO with a plain switch and its own Event
 * variant is a legal citizen too. This header therefore knows nothing
 * of fsm.hpp.
 */

#pragma once

#include <concepts>
#include <optional>
#include <type_traits>

namespace brio {

/// What Kernel<P, Aos...> requires of every AO in its pack.
template <typename A>
concept ActiveObject = requires(const typename A::Event& e) {
    A::init();
    A::dispatch(e);
    { A::queue.pop() } -> std::same_as<std::optional<typename A::Event>>;
    { A::queue.empty() } -> std::same_as<bool>;
};

/// The platform an AO's queue is guarded by, when the queue names it
/// (EventQueue does): on a chip with more than one core this is the
/// AO's core. Kernel and TimeEvent check it against their own P, so an
/// AO sits in the pack of its own core's kernel and a timer posts to an
/// AO of its own core - or does not compile. A queue that names no
/// platform (a hand-rolled one in a test) is trusted.
template <typename Ao, typename P>
constexpr bool queue_on() {
    using Q = std::remove_cvref_t<decltype(Ao::queue)>;
    if constexpr (requires { typename Q::Platform; }) {
        return std::same_as<typename Q::Platform, P>;
    } else {
        return true;
    }
}

} // namespace brio
