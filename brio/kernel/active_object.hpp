/*
 * active_object.hpp
 *
 * What the kernel requires of an active object: the ActiveObject
 * concept, and serve_one(), the kernel's step for one object.
 *
 * An active object is a class with no instances and only static members
 * (the type is the object), that provides:
 *  - `Event`: its own event type, a std::variant of small trivially
 *    copyable structs. Fsm<Derived, Alts...> (kernel/fsm.hpp) builds it
 *    for an object that derives from it, with the reserved Entry and
 *    Exit alternatives in front;
 *  - `queue`: a static EventQueue<Event, depth, P>, the one place where
 *    events for this object wait. The depth is the object's own sizing
 *    decision, which is why no base class declares the queue for it;
 *  - `init()`: called once by the kernel's init_all() (Tenuto's or
 *    Rubato's), in list order, before
 *    the first event is served. An Fsm-based object calls start(&initial)
 *    here, so its first state is entered before anything is posted;
 *  - `dispatch(const Event&)`: handle one event to completion. Fsm
 *    provides it; an object with a plain switch over its own variant
 *    writes it by hand and is a legal citizen too, which is why this
 *    header knows nothing of fsm.hpp.
 *
 * The rules the compiler cannot see, and that the kernel is written
 * assuming:
 *  - dispatch() is called by the kernel only - from its loop or, under
 *    Rubato (kernel/rubato.hpp), from inside a lower-priority object's
 *    dispatch -, with interrupts enabled, and is never re-entered:
 *    under Tenuto no other active object runs while it runs, under
 *    Rubato a higher one may run inside it, and in both the object's
 *    static data needs no lock against other objects, which touch
 *    nothing of it but its queue, through post(), as interrupt
 *    handlers do;
 *  - the event a dispatch receives lives in the queue's slot, and the
 *    slot is the object's until it is released: by the kernel right
 *    after the dispatch, or, when the object called the queue's hold()
 *    inside it, by the object itself in a later dispatch of its own, with
 *    the Held handle hold() returned (Lease::hold, kernel/borrowed.hpp).
 *    A held slot is released once, by its holder;
 *  - the object is its queue's one consumer, which is what lets the
 *    consumer side run without masking interrupts.
 *
 * Which object is being dispatched is a fact the kernel keeps, in
 * `dispatching<P>` below: the list index of the object whose dispatch is
 * running on platform P, `no_dispatch` when none is. The kernels write
 * it around every dispatch (the indexed serve_one() below), so a service
 * that is called from inside a dispatch - a port's writer sorting the
 * pieces of a line by who sent them - learns its caller's identity
 * without the caller naming itself. An interrupt handler reads the index
 * of the object it interrupted: a service that may be called from a
 * handler asks the platform first (`P::in_handler()`, the optional
 * member of kernel/platform.hpp).
 *
 * The concept below checks the names and signatures alone: that take()
 * hands over a slot's number, at() the Event in it, take_hold() whether
 * the object held it, release() takes the number back, empty() answers a
 * bool, and the queue names its `none`. It does not check that
 * dispatch() runs to completion, that init() calls start(), that a held
 * slot is released once, or that the queue is an EventQueue. The full
 * contract, both halves, is docs/design/kernel.md's.
 */

#pragma once

#include <stdint.h>
#include <concepts>
#include <type_traits>

namespace brio {

/// What a kernel (Tenuto<P, Aos...>, Rubato<P, Aos...>) requires of
/// every AO in its pack.
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

/// The value of dispatching<P> when no dispatch is running on P: before
/// the kernel's loop starts, between two dispatches, inside init_all().
inline constexpr uint8_t no_dispatch = 255;

/// The list index (the position in the kernel's pack, 0 the highest
/// priority) of the active object whose dispatch is running on platform
/// P, or no_dispatch. Written by the indexed serve_one() below around the
/// dispatch, the previous value put back after it, so a dispatch nested
/// in another under Rubato (kernel/rubato.hpp) shows its own index while
/// it runs and the outer one's again when it returns. Read from any
/// context; a handler sees the index of the dispatch it interrupted.
template <typename P>
inline uint8_t dispatching = no_dispatch;

/// Serve at most one event of one active object: take the oldest waiting
/// slot, dispatch the event in it, release the slot unless the object
/// held it. False when nothing waits. This is the step a program calls to
/// pump an object by hand outside a kernel: the one that keeps a hold
/// meaningful, where dispatching the copy pop() returns has no slot to
/// hold. It leaves dispatching<P> alone: a service pumped this way from
/// inside another object's dispatch runs as that object. Main-loop
/// context only.
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

/// The kernels' step for one object (kernel/tenuto.hpp, kernel/rubato.hpp):
/// serve_one() with dispatching<P> set to `index` for the dispatch and
/// put back after it. The two stores are plain: the variable is written
/// here alone and read by code that runs inside the dispatch or in a
/// handler, which the dispatch's call orders either way.
template <ActiveObject Ao, typename P>
bool serve_one(uint8_t index) {
    const uint8_t outer = dispatching<P>;
    dispatching<P> = index;
    const bool served = serve_one<Ao>();
    dispatching<P> = outer;
    return served;
}

/// The platform an AO's queue is guarded by, when the queue names it
/// (EventQueue does): on a chip with more than one core this is the
/// AO's core. The kernels and TimeEvent check it against their own P, so an
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
