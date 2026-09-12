/*
 * inbox.hpp
 *
 * The bridge between two kernels on two cores: how an event reaches an
 * AO that lives on the OTHER core. The kernel's own `post<Ao>` is one
 * core's primitive - the queue's critical section masks the interrupts
 * of the core that takes it and nothing else - so an event that crosses
 * takes another road, and this file is that road:
 *
 *   send<Ao>(ev)          the crossing verb, beside post<Ao>: the event
 *                         copied into Ao's INBOX and the receiving
 *                         core's doorbell rung
 *   Inbox<Ao>             one ring per AO that receives from the other
 *                         core, of Ao::Event exactly, sized by the AO
 *                         (`inbox_depth`, 8 by default)
 *   Inboxes<Aos...>       the receiving core's drain: the ISR body its
 *                         doorbell vector calls - the bells popped
 *                         FIRST, then every inbox emptied into ordinary
 *                         local post<Ao>() calls
 *   send_reply_to<Ao, P>  the ReplyTo capsule for a request that crosses:
 *                         the reply comes back through send, not post
 *
 * WHY A SEPARATE ROAD, AND NOT A LOCK IN POST. brio's guarantees are
 * all per core: run-to-completion, no nesting, pack-order priority, the
 * PRIMASK critical section, TimeEvents<P> - every one is a statement
 * about one core's loop, and every existing bridge (EventQueue, Ring,
 * MeterLatch, Trace) is built for the ISR-over-main boundary of one
 * core. A second core adds ONE boundary, core against core, and the
 * model closes it the same way it closed the first: a fixed set of
 * bridges, one primitive per boundary, and no AO ever sees either. The
 * bridge is this inbox; `post` stays what it is, byte for byte, on every
 * single-core target - the alternative, a spinlock in push, would tax
 * every post on every target for a crossing that is the exception.
 *
 * THE DISCIPLINE, one writer per word and a fence pair:
 *  - the slots and the head index are written by the SENDING core
 *    only, under that core's critical section (so the sender's loop
 *    and ISRs serialize among themselves, as they do around post); the
 *    tail index is written by the RECEIVING core only. One slot is
 *    sacrificed so full and empty differ with no shared counter -
 *    util/ring.hpp's rule, without its compiler-only fence;
 *  - the sender writes the slot, then a RELEASE fence, then the head;
 *    the receiver reads the head, then an ACQUIRE fence, then the slot
 *    - std::atomic_thread_fence, which the compiler turns into the
 *    core's data memory barrier (a DMB on ARMv6-M, whose memory model
 *    is weakly ordered between masters even without caches) and into
 *    nothing where nothing is needed (the host). ARMv6-M has no
 *    load-linked/store-conditional and this needs none: no word is
 *    read-modify-written by both sides;
 *  - a full inbox DROPS the event and counts it (`overflows()`), the
 *    same economy as post: nobody waits, so nothing deadlocks. What a
 *    dropped crossing costs is the requester's timeout, as a dropped
 *    reply already does.
 *
 * THE DOORBELL, and why the bells are popped BEFORE the drain. A send
 * rings the receiving core's doorbell (P::Doorbell::ring(): on the
 * RP2040 one word into the inter-core FIFO if it has room; if it has
 * not, a bell is already pending and the ring is skipped). The
 * receiving core's vector runs Inboxes<...>::isr(): pop EVERY pending
 * bell, an acquire fence, then drain every inbox into local posts. A
 * send that lands during the drain finds the FIFO empty, rings, the
 * interrupt re-pends at exit and a second pass runs; with the order
 * reversed, a send between the drain and the pop would see a bell
 * pending, skip its own, and its event would wait for the next
 * unrelated bell - the classic lost wakeup, closed by order exactly as
 * an interrupt flag is cleared before its data is read.
 *
 * WHAT DOES NOT CROSS. A Lease::dispatch loan never crosses: its
 * correctness IS pack order, and the kernel's lends_ok already refuses
 * a borrower outside the pack. A Lease::reply loan crosses inside a
 * request and comes back inside the reply, both through the inbox,
 * both covered by its fences. A TimeEvent posts to an AO of its own
 * core (kernel/time_event.hpp's static_assert). publish() stays local:
 * a remote subscriber is one send per subscriber, written where the
 * publisher knows it crosses.
 *
 * ORDER, stated so nobody relies on more: first-in first-out per inbox.
 * Two events to one AO, one sent and one posted, may be dispatched in
 * either order - exactly today's ISR-versus-loop pushes.
 *
 * STARTUP. The inbox is a static and holds what is sent before the
 * receiving kernel runs (its depth budgets that burst); the doorbell
 * interrupt is enabled when that core's program is ready
 * (Inboxes<...>::enable()), and a bell already pending fires it at once.
 *
 * TIME ACROSS THE BRIDGE: none. Each core's ticker is its own (equal
 * rate, a different phase), so a tick count is meaningless on the other
 * side; a timestamp that crosses is the chip's shared timer's, where the
 * chip has one.
 */

#pragma once

#include <stdint.h>
#include <atomic>
#include <concepts>
#include <tuple>
#include <type_traits>

#include "kernel/active_object.hpp"
#include "kernel/post.hpp"

namespace brio {

/// The platform of an AO's queue: its core.
template <typename Ao>
using queue_platform_t = typename std::remove_cvref_t<decltype(Ao::queue)>::Platform;

/// How many crossing events an AO's inbox holds: `Ao::inbox_depth` when
/// the AO says, 8 otherwise.
template <typename Ao>
constexpr uint8_t inbox_depth_of() {
    if constexpr (requires { { Ao::inbox_depth } -> std::convertible_to<uint8_t>; }) {
        return Ao::inbox_depth;
    } else {
        return 8;
    }
}

/// The ring of events sent to `Ao` from the other core (the file header).
template <typename Ao>
class Inbox {
    using E = typename Ao::Event;
    using P = queue_platform_t<Ao>;
    static_assert(requires { typename P::Doorbell; },
                  "brio Inbox: the AO's platform names no Doorbell - it is not one core of "
                  "several, and an event for it is post<Ao>()'s, not send<Ao>()'s");
    using Bell = typename P::Doorbell;
    static_assert(std::is_trivially_copyable_v<E>,
                  "events cross cores as bytes: they must be trivially copyable");

    static constexpr uint8_t depth = inbox_depth_of<Ao>();
    static_assert(depth >= 1u && depth < 255u, "brio Inbox: 1..254 events");
    static constexpr uint8_t size = depth + 1u;   // one slot sacrificed

public:
    Inbox() = delete;

    static constexpr uint8_t capacity() { return depth; }

    /// From the SENDING core, any context: copy `e` in and ring. False,
    /// the event dropped and counted, when the inbox is full.
    template <typename Ev>
        requires std::constructible_from<E, Ev>
    static bool send(const Ev& ev) {
        typename P::CriticalSection cs;   // the calling core's mask (the header)
        const uint8_t head = head_;
        const uint8_t next = static_cast<uint8_t>(head + 1u == size ? 0u : head + 1u);
        if (next == read_shared(tail_)) {
            if (overflows_ != UINT16_MAX) {
                ++overflows_;
            }
            return false;
        }
        slots_[head] = E{ev};
        std::atomic_thread_fence(std::memory_order_release);
        write_shared(head_, next);
        Bell::ring();
        return true;
    }

    /// On the RECEIVING core, from the doorbell's ISR body (Inboxes) or
    /// wherever that core drains: every event out, each posted locally.
    /// Returns how many.
    static uint8_t drain() {
        uint8_t n = 0;
        for (;;) {
            const uint8_t tail = tail_;
            if (tail == read_shared(head_)) {
                return n;
            }
            std::atomic_thread_fence(std::memory_order_acquire);
            const E e = slots_[tail];
            // The slot is free for the sender only once it is copied:
            // the release orders the copy before the tail store.
            std::atomic_thread_fence(std::memory_order_release);
            write_shared(tail_, static_cast<uint8_t>(tail + 1u == size ? 0u : tail + 1u));
            post<Ao>(e);
            ++n;
        }
    }

    /// Anything waiting (a read of both indices, no fence: a hint).
    static bool pending() { return read_shared(head_) != read_shared(tail_); }

    /// Events dropped for want of room, saturating; the sender's word.
    static uint16_t overflows() { return read_shared(overflows_); }

    /// The test's and the boot's broom: both indices to zero, the count
    /// too. Legal only with nobody sending.
    static void clear() {
        head_ = 0;
        tail_ = 0;
        overflows_ = 0;
    }

private:
    template <typename T>
    static T read_shared(const T& v) { return *const_cast<const volatile T*>(&v); }
    template <typename T>
    static void write_shared(T& v, T value) { *const_cast<volatile T*>(&v) = value; }

    static inline E slots_[size]{};
    static inline uint8_t head_ = 0;         // written by the sending core
    static inline uint8_t tail_ = 0;         // written by the receiving core
    static inline uint16_t overflows_ = 0;   // written by the sending core
};

/// The crossing verb: `ev` to `Ao` on the other core. Drop + count on a
/// full inbox, like post on a full queue; never blocks.
template <typename Ao, typename Ev>
    requires std::constructible_from<typename Ao::Event, Ev>
void send(const Ev& ev) {
    (void)Inbox<Ao>::send(ev);
}

/// The receiving core's side, over every AO of that core that has an
/// inbox: the ISR body its doorbell vector calls, and the enable.
template <typename... Aos>
    requires (sizeof...(Aos) > 0)
struct Inboxes {
    Inboxes() = delete;

    using P = queue_platform_t<std::tuple_element_t<0, std::tuple<Aos...>>>;
    static_assert((std::same_as<queue_platform_t<Aos>, P> && ...),
                  "brio Inboxes: every AO of one drain lives on one core");
    using Bell = typename P::Doorbell;

    /// The doorbell's ISR body: the bells first, then the inboxes.
    [[gnu::always_inline]] static void isr() {
        Bell::pop_all();
        std::atomic_thread_fence(std::memory_order_acquire);
        (Inbox<Aos>::drain(), ...);
    }

    /// The doorbell interrupt on, on this core; a bell already pending
    /// fires it at once.
    static void enable() { Bell::enable(); }

    /// Every inbox of this core empty.
    static bool idle() { return (!Inbox<Aos>::pending() && ...); }
};

/// The thunk a crossing reply takes.
template <typename Ao, typename Payload>
void send_thunk(const Payload& p) {
    send<Ao>(p);
}

/// The return channel of a request that CROSSES: the reply comes back to
/// `Ao` on the requester's core through its inbox. Built where the
/// request is sent, by the side that knows it crosses; the service
/// calls reply.send() as for any request.
template <typename Ao, typename Payload>
constexpr ReplyTo<Payload> send_reply_to() {
    return ReplyTo<Payload>::template through<&send_thunk<Ao, Payload>>();
}

} // namespace brio
