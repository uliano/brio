/*
 * post.hpp
 *
 * Sending events: post() to one named active object, publish() to a
 * list of subscribers, ReplyTo to return a result to whoever asked.
 *
 *     brio::post<Blinker>(SetPeriod{250});
 *
 * post() may be called from an interrupt handler, from a handler of
 * another active object, or from a timer; it is the same function in
 * every context. It copies the event into the receiver's queue (built
 * in the queue slot, with interrupts masked for the few instructions of
 * the copy) and returns at once. It never waits: on a full queue the
 * event is dropped and the queue's saturating overflow counter counts
 * it. Nothing of the receiver runs inside post(); the receiver handles
 * the event in its own dispatch, on a later turn of the kernel loop.
 *
 * The compiler checks that the receiver accepts the event: an event type
 * that is not among the receiver's alternatives is an error at the call.
 * The reserved events Entry and Exit are refused too: the state machine
 * delivers them directly and they never travel through a queue.
 *
 * One copy per receiver, value semantics: no pools, no reference
 * counting. A payload that must not be copied travels as a pointer with
 * its lifetime rule in its type (kernel/borrowed.hpp).
 *
 * The three forms, in one line each: commands are addressed (post),
 * facts are published (publish), replies return to sender (ReplyTo).
 * What a receiver must look like is the ActiveObject contract
 * (kernel/active_object.hpp); post() itself requires only the two facts
 * it uses, that Ao::Event exists and that Ao::queue accepts the event.
 */

#pragma once

#include <concepts>

#include "kernel/fsm.hpp"

namespace brio {

/// Copy one event (any alternative of Ao::Event) into Ao's queue. Any
/// context; never waits; a full queue drops and counts.
template <typename Ao, typename Ev>
    requires std::constructible_from<typename Ao::Event, Ev>
void post(const Ev& e) {
    static_assert(!std::same_as<Ev, Entry> && !std::same_as<Ev, Exit>,
                  "Entry/Exit are reserved for the transition machinery "
                  "and cannot be posted");
    Ao::queue.push(e);
}

/// A compile-time subscriber list for publish().
template <typename... Aos>
struct Subscribers {};

/// Post one notification to every listed subscriber: one copy per
/// subscriber, in list order, each a post() with post()'s rules. Each
/// subscriber's Event variant must accept the notification type; a
/// subscription that cannot be received does not compile.
template <typename... Aos, typename Ev>
void publish(Subscribers<Aos...>, const Ev& e) {
    (post<Aos>(e), ...);
}

/**
 * ReplyTo<Payload>: the return channel of a request. A requester builds
 * it with reply_to<RequesterAo, Payload>() and puts it inside the
 * request event; the service that completes the request calls
 * `reply.send(result)`, which posts the result to the requester without
 * the service knowing who asked.
 *
 * Inside, a ReplyTo is one function pointer. The function it points at
 * is generated per requester and does one thing, post<Requester>(payload);
 * a function that exists only to forward a call to a destination fixed
 * when it was generated is called a thunk, and that is the word the code
 * below uses. So the capsule is one pointer wide and trivially copyable,
 * and travels inside the event like any other field. A default-
 * constructed capsule points at no thunk: send() does nothing, which is
 * how a fire-and-forget request is written.
 *
 *   struct SpiDone { SpiStatus status; };
 *   struct SpiRequest { ...; brio::ReplyTo<SpiDone> reply; };
 *   ...
 *   post<SpiBus>(SpiRequest{..., brio::reply_to<MyAo, SpiDone>()});
 *   ...                       // in SpiBus, transfer finished:
 *   req.reply.send(SpiDone{status});
 *
 * send() has post()'s rules: any context, never waits, a full queue
 * drops and counts. A requester whose Event variant cannot hold Payload
 * fails to compile at the reply_to<>() call. A static function generated
 * per requester is the same device TimeEvent and the state machine use
 * where another design would use a virtual call.
 */
template <typename Payload>
class ReplyTo {
    using Thunk = void (*)(const Payload&);

public:
    constexpr ReplyTo() = default;                 // null: fire and forget

    void send(const Payload& p) const {
        if (thunk_ != nullptr) {
            thunk_(p);
        }
    }

    constexpr explicit operator bool() const { return thunk_ != nullptr; }

    template <typename Ao>
    static constexpr ReplyTo to() {
        return ReplyTo{&thunk_for<Ao>};
    }

    /// A capsule around a forwarding function of the caller's own, for a
    /// reply that does not travel by post<Ao>(): util/inbox.hpp's
    /// send_reply_to builds the capsule that crosses to the requester's
    /// core.
    template <void (*fn)(const Payload&)>
    static constexpr ReplyTo through() {
        return ReplyTo{fn};
    }

private:
    constexpr explicit ReplyTo(Thunk t) : thunk_(t) {}

    template <typename Ao>
    static void thunk_for(const Payload& p) {
        post<Ao>(p);
    }

    Thunk thunk_ = nullptr;
};

/// Build the return channel: replies to Ao as a posted Payload event.
template <typename Ao, typename Payload>
constexpr ReplyTo<Payload> reply_to() {
    return ReplyTo<Payload>::template to<Ao>();
}

} // namespace brio
