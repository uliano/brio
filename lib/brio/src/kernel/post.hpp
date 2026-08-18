/*
 * post.hpp
 *
 * Event delivery primitives: post() addresses one AO, publish() fans a
 * notification out to a compile-time subscriber list, ReplyTo returns a
 * result to whoever asked ("commands are addressed, facts are
 * published, replies return to sender"). One copy per receiver - value
 * semantics, no pools, no reference counting.
 *
 * The reserved FSM events (Entry/Exit) are excluded at compile time:
 * they are delivered synchronously by the transition machinery and must
 * never travel through a queue.
 *
 * post() is safe from ISRs and from any main-loop code alike (the queue
 * push runs in a critical section) and never blocks: a full queue drops
 * the event and increments the queue's saturating overflow counter.
 *
 * What a receiver must look like is the ActiveObject contract
 * (kernel/active_object.hpp); post() itself needs only the two facts it
 * uses - Ao::Event exists and Ao::queue accepts it - so it requires
 * exactly those and nothing more.
 */

#pragma once

#include <concepts>

#include "kernel/fsm.hpp"

namespace brio {

/// Post one event (any alternative of Ao::Event) to one active object.
template <typename Ao, typename Ev>
    requires std::constructible_from<typename Ao::Event, Ev>
void post(const Ev& e) {
    static_assert(!std::same_as<Ev, Entry> && !std::same_as<Ev, Exit>,
                  "Entry/Exit are reserved for the transition machinery "
                  "and cannot be posted");
    Ao::queue.push(typename Ao::Event{e});
}

/// A compile-time subscriber list for publish().
template <typename... Aos>
struct Subscribers {};

/// Publish a notification to every subscriber: one post per AO. Each
/// subscriber's Event variant must accept the notification type - a
/// subscription that cannot be received does not compile.
template <typename... Aos, typename Ev>
void publish(Subscribers<Aos...>, const Ev& e) {
    (post<Aos>(e), ...);
}

/**
 * ReplyTo<Payload>: a "post back to whoever asked" capsule - the return
 * channel of a request/reply service (SPI/I2C transactions, ...). One
 * function pointer wide, trivially copyable: it travels INSIDE the
 * request event. The service neither knows nor cares who the requester
 * is; the capsule was built at compile time from <RequesterAo, Payload>
 * (same thunk technique as TimeEvent firing and FSM dispatch - no
 * virtuals). A default-constructed capsule is null: send() is a no-op,
 * which makes fire-and-forget requests free.
 *
 *   struct SpiDone { SpiStatus status; };
 *   struct SpiRequest { ...; brio::ReplyTo<SpiDone> reply; };
 *   ...
 *   post<SpiBus>(SpiRequest{..., brio::reply_to<MyAo, SpiDone>()});
 *   ...                       // in SpiBus, transfer finished:
 *   req.reply.send(SpiDone{status});
 *
 * A requester whose Event variant cannot hold Payload fails to compile
 * at the reply_to<>() site.
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
