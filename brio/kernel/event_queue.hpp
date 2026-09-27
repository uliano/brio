/*
 * event_queue.hpp
 *
 * The per-AO event queue of the brio kernel: multi-producer (any ISR and
 * any main-loop code may push), single-consumer (the scheduler takes).
 *
 * THE PRODUCERS. push() runs inside Platform::CriticalSection. Under the
 * cooperative kernel the only real concurrency is ISR vs main loop, and
 * a small core may offer no CAS at all: the brief interrupts-off section
 * IS the honest primitive. The event is BUILT IN ITS SLOT - a variant's
 * alternative is emplaced there, never first assembled on the caller's
 * stack - so the one copy a post makes is the one the mask covers.
 * There are deliberately NO *_from_isr variants: one API, always safe -
 * the saved cycles would not pay for the doubled surface and the risk of
 * calling the wrong one. Revisit only with measurements.
 *
 * THE CONSUMER. take() hands the oldest waiting event over IN ITS SLOT:
 * no copy out and no critical section. Each side writes only its own
 * counter and reads the other's - the producers `pushed_`, under their
 * mask; the consumer `served_`, bare - and a counter is one byte, which
 * every platform loads and stores as one access (kernel/platform.hpp's
 * atomic_width), so the consumer never masks an interrupt and an empty
 * queue costs it two loads and a compare. The ordering between a slot's
 * contents and the counter that publishes or releases it is
 * std::atomic_signal_fence, util/ring.hpp's idiom: a compiler-only fence,
 * correct because the queue belongs to one core (below).
 *
 * DEPTH IS THE NUMBER OF EVENTS THAT CAN WAIT. The event being served is
 * not one of them: the slot take() handed over stays the consumer's
 * until its NEXT take(), so the queue holds depth + 1 slots and a
 * producer finds depth free places whether an event is being served or
 * not - a post from inside a dispatch counts against the same depth as a
 * post from anywhere else, and a full queue overflows at the same moment
 * in every state. The slot beyond depth is the storage the served event
 * needs while its dispatch reads it; it lives here instead of on the
 * consumer's stack. The pointer take() returns is valid until the
 * consumer's next take() or pop(): the consumer is one context, and a
 * dispatch is never re-entered (kernel/active_object.hpp).
 *
 * Overflow: push() never blocks and never fails from the caller's point
 * of view. On a full queue the event is dropped and a saturating
 * overflow counter is incremented: a full queue is a SIZING mistake, and
 * the counter names the culprit (from gdb on target, from asserts in
 * host tests). The count-vs-panic reaction knob belongs to the kernel
 * layer above, not here.
 *
 * Depth is arbitrary (no power-of-two rule): event slots are a few bytes
 * to a few words, so rounding a depth of 5 up to 8 would waste real RAM
 * to speed up a wrap that is already two instructions.
 *
 * THE QUEUE BELONGS TO ONE CORE: the platform's. Its critical section
 * masks the interrupts of the core that takes it and nothing else, and
 * its fences order nothing between cores, so a push from another core
 * would race the owner's pushes and takes - which is why events cross
 * cores through util/inbox.hpp and never through push(). A platform that
 * can tell which core is running offers `on_own_core()`
 * (kernel/platform.hpp), and push() then REFUSES a copy from the wrong
 * core before it touches anything: the event is dropped and a saturating
 * counter, `misposts()`, names the mistake - the same economy as
 * overflow. A single-core platform has no such member and compiles no
 * such check.
 */

#pragma once

#include <stdint.h>
#include <atomic>
#include <concepts>
#include <optional>
#include <type_traits>

#include "kernel/platform.hpp"

namespace brio {

/// Whether a platform can tell which core is running (the optional
/// member of kernel/platform.hpp).
template <typename Plat>
concept CoreAware = requires {
    { Plat::on_own_core() } -> std::same_as<bool>;
};

/// The mispost counter: a member of the queues of a core-aware platform
/// and NOTHING - an empty base, no byte - of every other queue, so a
/// single-core target's queues keep their size.
struct MispostCounter {
    uint16_t misposts_{0};
};
struct NoMispostCounter {};

template <typename E, uint8_t depth, Platform Plat>
class EventQueue
    : private std::conditional_t<CoreAware<Plat>, MispostCounter, NoMispostCounter> {
    static_assert(depth >= 1, "queue depth must be at least 1");
    static_assert(std::is_trivially_copyable_v<E>,
                  "events are copied byte-wise into the queue, possibly "
                  "from an ISR: they must be trivially copyable");
    static_assert(Plat::atomic_width >= sizeof(uint8_t),
                  "the two counters are bytes the producers and the consumer "
                  "share without a lock");

public:
    /// The platform whose critical section guards this queue: the core
    /// the owning AO lives on, on a chip with more than one.
    using Platform = Plat;

    /// Build an event in the queue from any type the queue's event can be
    /// made of: a variant's alternative is emplaced in the slot, an event
    /// of the queue's own type is copied into it. Full queue (depth events
    /// waiting): drop + count, never block. A push from a core that is not
    /// the queue's (a platform that can tell): dropped and counted as a
    /// mispost, the queue untouched.
    template <typename Ev>
        requires std::constructible_from<E, const Ev&>
    void push(const Ev& ev) {
        if constexpr (CoreAware<Plat>) {
            if (!Plat::on_own_core()) {
                if (this->misposts_ != UINT16_MAX) {
                    ++this->misposts_;
                }
                return;
            }
        }
        typename Plat::CriticalSection cs;
        if (static_cast<uint8_t>(pushed_ - load_shared(served_)) == depth) {
            if (overflows_ != UINT16_MAX) {  // saturate: never lie by wrapping
                ++overflows_;
            }
            return;
        }
        build(slots_[head_], ev);
        head_ = next(head_);
        store_shared(pushed_, static_cast<uint8_t>(pushed_ + 1u));
    }

    /// The oldest waiting event, IN ITS SLOT, or null when none waits. The
    /// slot is the consumer's until its next take(): no producer writes
    /// it meanwhile. Consumer context only; no critical section.
    const E* take() {
        if (load_shared(pushed_) == served_) {
            return nullptr;
        }
        const E* e = &slots_[tail_];
        tail_ = next(tail_);
        store_shared(served_, static_cast<uint8_t>(served_ + 1u));
        return e;
    }

    /// The oldest waiting event as a copy; nullopt when none waits. take()
    /// with a copy out, for the code that drains a queue by hand - a test,
    /// or an app pumping an AO outside a kernel. Consumer context only.
    std::optional<E> pop() {
        if (const E* e = take()) {
            return *e;
        }
        return std::nullopt;
    }

    /// No event waits (the one being served, if any, is not waiting).
    bool empty() const {
        return load_shared(pushed_) == load_shared(served_);
    }

    /// The events waiting, the one being served excluded.
    uint8_t size() const {
        return static_cast<uint8_t>(load_shared(pushed_) - load_shared(served_));
    }

    uint16_t overflows() const {
        typename Plat::CriticalSection cs;
        return overflows_;
    }
    /// Pushes refused because they came from another core (zero, and no
    /// code, on a platform that cannot tell). Written outside the
    /// critical section by construction - the wrong core's guard would
    /// not guard - so a lost count under a race is the counter's own
    /// caveat: it is a programming error's witness, not a statistic.
    uint16_t misposts() const {
        if constexpr (CoreAware<Plat>) {
            return this->misposts_;
        } else {
            return 0;
        }
    }

    /// The events that can wait: depth. The queue holds one slot more, for
    /// the event being served.
    static constexpr uint8_t capacity() { return depth; }

private:
    static constexpr uint16_t slot_count = depth + 1u;

    static uint8_t next(uint8_t i) {
        return static_cast<uint8_t>(i + 1u == slot_count ? 0u : i + 1u);
    }

    template <typename Ev>
    static void build(E& slot, const Ev& ev) {
        if constexpr (std::is_same_v<Ev, E>) {
            slot = ev;
        } else if constexpr (requires { slot.template emplace<Ev>(ev); }) {
            slot.template emplace<Ev>(ev);   // the alternative, built in place
        } else {
            slot = E(ev);
        }
    }

    /// Read a counter the other side writes: fresh, and ordered before
    /// the slot access it guards.
    static uint8_t load_shared(const uint8_t& counter) {
        const uint8_t v = *const_cast<const volatile uint8_t*>(&counter);
        std::atomic_signal_fence(std::memory_order_acquire);
        return v;
    }

    /// Publish a counter after the slot access it covers is complete.
    static void store_shared(uint8_t& counter, uint8_t value) {
        std::atomic_signal_fence(std::memory_order_release);
        *const_cast<volatile uint8_t*>(&counter) = value;
    }

    E slots_[slot_count]{};
    uint8_t head_{0};    // the producers', under their critical section
    uint8_t pushed_{0};  // the producers', under their critical section; the consumer reads it
    uint8_t tail_{0};    // the consumer's
    uint8_t served_{0};  // the consumer's; the producers read it
    uint16_t overflows_{0};
};

} // namespace brio
