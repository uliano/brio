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
 *  - `init()`: called once by Tenuto::init_all() in pack order, before
 *    the first event is served. An Fsm-based AO calls start(&initial)
 *    here, so its first state is entered before anything can be posted;
 *  - `dispatch(const Event&)`: run ONE event to completion. Called from
 *    the kernel loop only, interrupts enabled, never re-entered: while a
 *    dispatch runs no other AO runs (single stack, cooperative), so the
 *    AO's static data needs no locking against other AOs - only ISRs
 *    are concurrent, and they touch nothing but the queue (through
 *    post(), inside a critical section). The Event it receives is the
 *    one in the queue's slot, and the slot is the AO's until it is
 *    RELEASED: by the kernel right after the dispatch, or - when the AO
 *    called the queue's hold() inside it - by the AO itself, with the
 *    Held handle hold() returned, in a later dispatch of its own
 *    (Lease::hold, kernel/borrowed.hpp). Never being re-entered is also
 *    what keeps a slot from being taken twice, and the AO being the
 *    queue's one consumer is what lets it release without a mask.
 *
 * The concept checks only what the compiler can see: the names, the
 * signatures - that take() hands over a slot's number, at() the Event
 * in it, take_hold() whether the AO held it, release() takes the number
 * back, empty() a bool, and the queue names its `none`. It does NOT
 * check that dispatch is run-to-completion, that init() calls start(),
 * that a held slot is released once and by its owner, or that the queue
 * is really an EventQueue: those are the rules of the model, and
 * Tenuto/Fsm/post are written assuming them.
 *
 * Fsm is one way to satisfy the contract (Event + dispatch for free),
 * not the contract itself: an AO with a plain switch and its own Event
 * variant is a legal citizen too. This header therefore knows nothing
 * of fsm.hpp.
 */

#pragma once

#include <stdint.h>
#include <concepts>
#include <type_traits>

namespace brio {

/// What Tenuto<P, Aos...> requires of every AO in its pack.
template <typename A>
concept ActiveObject = requires(const typename A::Event& e, uint8_t h) {
    A::init();
    A::dispatch(e);
    { A::queue.take() } -> std::same_as<uint8_t>;
    { A::queue.at(h) } -> std::same_as<const typename A::Event&>;
    { A::queue.take_hold() } -> std::same_as<bool>;
    A::queue.release(h);
    { A::queue.empty() } -> std::same_as<bool>;
    { std::remove_cvref_t<decltype(A::queue)>::none } -> std::convertible_to<uint8_t>;
};

/// Serve at most ONE event of one AO: take the oldest waiting slot,
/// dispatch the event IN it, release the slot unless the AO held it.
/// False when nothing waits. The kernel loop's step for each AO
/// (kernel/tenuto.hpp), and the verb a program uses to pump an AO by
/// hand outside a kernel - the one that keeps a hold meaningful, where
/// a dispatch of the copy pop() returns has no slot to hold.
template <ActiveObject Ao>
bool serve_one() {
    const uint8_t h = Ao::queue.take();      // no copy, no critical section
    if (h == std::remove_cvref_t<decltype(Ao::queue)>::none) {
        return false;
    }
    Ao::dispatch(Ao::queue.at(h));          // in its slot, run-to-completion, interrupts free
    if (!Ao::queue.take_hold()) {
        Ao::queue.release(h);
    }
    return true;
}

/// The platform an AO's queue is guarded by, when the queue names it
/// (EventQueue does): on a chip with more than one core this is the
/// AO's core. Tenuto and TimeEvent check it against their own P, so an
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
