/*
 * post.hpp
 *
 * Event delivery primitives: post() addresses one AO, publish() fans a
 * notification out to a compile-time subscriber list ("commands are
 * addressed, facts are published"). One copy per receiver - value
 * semantics, no pools, no reference counting.
 *
 * The reserved FSM events (Entry/Exit) are excluded at compile time:
 * they are delivered synchronously by the transition machinery and must
 * never travel through a queue.
 *
 * post() is safe from ISRs and from any main-loop code alike (the queue
 * push runs in a critical section) and never blocks: a full queue drops
 * the event and increments the queue's saturating overflow counter.
 */

#pragma once

#include <concepts>
#include <optional>

#include "kernel/fsm.hpp"

namespace brio {

/// What the kernel requires of an active object (see kernel.hpp for the
/// full lifecycle): its event type, a queue, init() and dispatch().
template <typename A>
concept ActiveObject = requires(const typename A::Event& e) {
    A::init();
    A::dispatch(e);
    { A::queue.pop() } -> std::same_as<std::optional<typename A::Event>>;
    { A::queue.empty() } -> std::same_as<bool>;
};

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

} // namespace brio
