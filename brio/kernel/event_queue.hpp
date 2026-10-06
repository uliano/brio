/*
 * event_queue.hpp
 *
 * The per-AO event queue of the brio kernel: multi-producer (any ISR and
 * any main-loop code may push), single-consumer (the AO's dispatch
 * context: the kernel loop, and the AO itself when it holds).
 *
 * THE STRUCTURE - an INDEX CAROUSEL. Two things are kept apart:
 *  - the SLOTS, depth + 1 of them: an event is BUILT in a slot by push()
 *    and stays there, read in place, until the slot is RELEASED - right
 *    after its dispatch, or later when the AO holds it;
 *  - the RING, depth + 2 positions of one byte each, holding slot
 *    NUMBERS and nothing else. No event is ever in the ring and nothing
 *    is ever copied between positions.
 * Three cursors cut the ring into three runs, in ring order:
 *
 *     [take_, alloc_)         PENDING      numbers of built events, oldest first
 *     [alloc_, free end)      FREE         numbers a producer may build in
 *     [free end, take_)       OUTSTANDING  positions of slots taken and not yet
 *                                          released; what they contain is
 *                                          stale, read by nobody
 *
 * A producer builds its event in the slot whose number sits at alloc_
 * and publishes it by advancing alloc_ - the advance moves that number
 * from the free run into the pending run, so allocation order IS push
 * order and the free run doubles as the order of the pending one. The
 * consumer takes the number at take_ (the event is dispatched in its
 * slot); a release writes the slot's number at the free run's end and
 * advances it - whatever order the slots come back in, each costs one
 * byte written. The ring has one position more than there are slots, so
 * the outstanding run is never empty while anything is outstanding (one
 * position is always spare there): an empty pending run is take_ ==
 * alloc_, an empty free run is alloc_ meeting the free end, and no count
 * is shared.
 *
 * THE PRODUCERS. push() runs inside Platform::CriticalSection. Under the
 * cooperative kernel the only real concurrency is ISR vs main loop, and
 * a small core may offer no CAS at all: the brief interrupts-off section
 * IS the honest primitive. The event is BUILT IN ITS SLOT - a variant's
 * alternative is emplaced there, never first assembled on the caller's
 * stack - so the one copy a post makes is the one the mask covers. The
 * producers write one cursor, alloc_, and read the free end. There are
 * deliberately NO *_from_isr variants: one API, always safe - the saved
 * cycles would not pay for the doubled surface and the risk of calling
 * the wrong one. Revisit only with measurements.
 *
 * THE CONSUMER NEVER MASKS. It writes take_ and the free end and reads
 * alloc_; every cursor is one byte with ONE writer, and a byte is one
 * access on every platform (kernel/platform.hpp's atomic_width). A
 * producer reads only the free run, which the consumer filled before it
 * moved the free end; the consumer reads only the pending run, whose
 * positions nobody touched since a producer read them; the consumer
 * writes only the outstanding run, which no producer reads. The ordering
 * between a slot or a ring position and the cursor that publishes it is
 * std::atomic_signal_fence, util/ring.hpp's idiom: a compiler-only fence,
 * correct because the queue belongs to one core (below).
 *
 * THE HANDLE. take() returns the slot's NUMBER (a Handle, 1 to depth +
 * 1; `none`, zero, when nothing waits) and at() the event in that slot.
 * The number is fixed from push() to release(); positions are the
 * queue's own bookkeeping and nobody outside keeps one. The kernel
 * releases the slot after the dispatch unless the AO called hold()
 * inside it: hold() returns a Held<E> (kernel/borrowed.hpp,
 * Lease::hold), and the slot stays the AO's - its event intact, read
 * with at() - until the AO's own release() in a later dispatch of its
 * own. Release order is free: a hold costs exactly its own slot for as
 * long as it lasts, never the slots behind it.
 *
 * DEPTH IS THE NUMBER OF EVENTS THAT CAN BE WAITING OR HELD AT ONCE,
 * the event being dispatched aside: the queue holds depth + 1 slots, so
 * a producer finds depth places while a dispatch is in progress and the
 * AO holds nothing. The overflow moment, in one rule: a push is refused
 * when EVERY slot is occupied, waiting or outstanding (taken and not
 * released: the one being dispatched and the held ones). With nothing
 * being dispatched the spare slot takes a waiting event too, so an idle
 * queue that holds nothing accepts depth + 1. Each hold takes one of the
 * depth for as long as it lasts; an AO sizes its depth as a COUNT - the
 * events it holds at once plus the waiting peak.
 *
 * A STATIC QUEUE IS ALL ZERO and lands in .bss, like every kernel
 * static - whenever the event's value-initialised state is all zero, as
 * every Fsm event's is (Entry, its first alternative, has no field); an
 * event type with non-zero defaults makes the queue a .data image - through
 * three choices that cost the masked window one XOR:
 *  - the free end is stored as the FENCE, the position after it (zero at
 *    the start, when the free run is every slot): push() computes
 *    next(alloc_) anyway to publish, and full is that value meeting the
 *    fence;
 *  - each position is stored ONE PLACE AHEAD in the array (position p in
 *    ring_[next(p)]), so the place a release writes is the fence itself
 *    and the place a take reads is the next(take_) it computes anyway,
 *    with no step back anywhere;
 *  - each number is stored XOR its place in the array, and the slots are
 *    numbered from 1 - so the first state, slot i at place i, reads zero,
 *    and the spare position decodes to 0, which is `none`. The slot's
 *    address takes the one off: folded into the base or an offset where
 *    the core's addressing allows (the Cortex-M4's), one subtraction
 *    where it does not (the AVR's, the Cortex-M0+'s).
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
 * to speed up a wrap that is already two instructions. It is at most
 * 254: the slot numbers 1 to depth + 1 are bytes and 0 is `none`.
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
 *
 * A host platform that explores interleavings offers `interleave_point()`
 * (kernel/platform.hpp): the consumer's verbs call it at every boundary
 * between two of their shared accesses, where an interrupt could land,
 * and the host suite pushes from there. No target has it, and nothing is
 * compiled for it.
 */

#pragma once

#include <stdint.h>
#include <atomic>
#include <concepts>
#include <optional>
#include <type_traits>

#include "kernel/borrowed.hpp"
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
    static_assert(depth <= 254, "slot numbers are 1 to depth + 1 in a byte: at most 255 slots");
    static_assert(std::is_trivially_copyable_v<E>,
                  "events are copied byte-wise into the queue, possibly "
                  "from an ISR: they must be trivially copyable");
    static_assert(Plat::atomic_width >= sizeof(uint8_t),
                  "the cursors are bytes the producers and the consumer "
                  "share without a lock");

public:
    /// The platform whose critical section guards this queue: the core
    /// the owning AO lives on, on a chip with more than one.
    using Platform = Plat;

    /// A slot's number, 1 to depth + 1: what take() returns and release()
    /// takes back.
    using Handle = uint8_t;
    /// The handle take() returns when nothing waits.
    static constexpr Handle none = 0;

    /// Build an event in the queue from any type the queue's event can be
    /// made of: a variant's alternative is emplaced in the slot, an event
    /// of the queue's own type is copied into it. Full queue (every slot
    /// waiting or outstanding): drop + count, never block. A push from a
    /// core that is not the queue's (a platform that can tell): dropped
    /// and counted as a mispost, the queue untouched.
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
        const uint8_t a = alloc_;
        const uint8_t n = next(a);
        if (n == load_shared(fence_)) {       // the free run is empty
            if (overflows_ != UINT16_MAX) {   // saturate: never lie by wrapping
                ++overflows_;
            }
            return;
        }
        build(slot(static_cast<uint8_t>(ring_[n] ^ n)), ev);   // position a lives at n
        store_shared(alloc_, n);              // the advance IS the publish
    }

    /// The oldest waiting event's slot number, or `none` when nothing
    /// waits. The slot is OUTSTANDING from here until release(): no
    /// producer writes it, and at() reads the event in it. Consumer
    /// context only; no critical section.
    Handle take() {
        const uint8_t t = take_;
        point();
        if (t == load_shared(alloc_)) {
            return none;
        }
        point();
        const uint8_t n = next(t);
        const uint8_t h = static_cast<uint8_t>(ring_[n] ^ n);   // position t lives at n
        // A pending position holds a slot's number, never `none`: said so
        // the caller's test for `none` folds into the emptiness test.
        [[assume(h != none && h <= slot_count)]];
        point();
        take_ = n;
        point();
        return h;
    }

    /// The event in a slot take() handed over and nobody has released.
    const E& at(Handle h) const { return slots_[h - 1]; }
    const E& at(Held<E> h) const { return slots_[h.handle() - 1]; }

    /// Inside a dispatch, and only there: keep the slot being dispatched
    /// after the dispatch ends. The slot is then the AO's until its own
    /// release() of the returned handle, in a later dispatch of its own;
    /// a second hold() in the same dispatch returns the same slot. The
    /// slot's number is read back from the position the take just left
    /// (stored at take_ itself) - the last of the outstanding run, which
    /// no release reaches while the slot in it is still outstanding - so
    /// a take records nothing for it. An AO that holds is served through
    /// take() (the kernel, or serve_one()), never by dispatching the copy
    /// pop() returns: that copy has no slot, and a hold() there would
    /// hand out the number of the slot pop() already released. A hold()
    /// outside a dispatch also leaves the flag set, so the NEXT dispatch's
    /// slot is never released - kept for ever, one slot the less.
    Held<E> hold() {
        held_ = 1;
        const uint8_t t = take_;              // position prev(take_) lives at take_
        return Held<E>{static_cast<uint8_t>(ring_[t] ^ t)};
    }

    /// The kernel's, once after each dispatch: whether the AO held the
    /// slot it was dispatched from. Clears the flag.
    bool take_hold() {
        if (held_ == 0) {
            return false;
        }
        held_ = 0;
        return true;
    }

    /// Give a slot back: its number is written at the free run's end, in
    /// any order. The kernel releases what was not held; a held slot is
    /// released by its owner. Consumer context only; no critical section.
    /// A release of a slot that is not outstanding - twice, or one never
    /// taken - is not checked and corrupts the queue for good: it writes
    /// a number the free run already has, so the queue reads FULL for
    /// ever (every push refused, size() 0, overflows() climbing) or hands
    /// one slot to two producers. One holder, one release.
    void release(Handle h) {
        const uint8_t fence = fence_;         // the free run's end lives here
        ring_[fence] = static_cast<uint8_t>(h ^ fence);
        point();
        store_shared(fence_, next(fence));    // the slot is free from here
        point();
    }
    void release(Held<E> h) { release(h.handle()); }

    /// The oldest waiting event as a copy; nullopt when none waits. take()
    /// with a copy out and the slot released at once, for the code that
    /// drains a queue by hand - a test, or a program pumping an AO that
    /// does not hold outside a kernel. Consumer context only.
    std::optional<E> pop() {
        const uint8_t t = take_;
        if (t == load_shared(alloc_)) {
            return std::nullopt;
        }
        const uint8_t n = next(t);
        const uint8_t h = static_cast<uint8_t>(ring_[n] ^ n);
        take_ = n;
        std::optional<E> e{slot(h)};
        release(h);
        return e;
    }

    /// No event waits (the outstanding ones are not waiting).
    bool empty() const {
        return load_shared(take_) == load_shared(alloc_);
    }

    /// The events waiting, the outstanding ones excluded.
    uint8_t size() const {
        const uint8_t a = load_shared(alloc_);
        const uint8_t t = load_shared(take_);
        return static_cast<uint8_t>(a >= t ? a - t : a + positions - t);
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

    /// The events that can be waiting or held at once while a dispatch is
    /// in progress: depth. The queue holds one slot more.
    static constexpr uint8_t capacity() { return depth; }

private:
    static constexpr uint16_t slot_count = depth + 1u;
    static constexpr uint16_t positions = slot_count + 1u;

    static uint8_t next(uint8_t i) {
        return static_cast<uint8_t>(i + 1u == positions ? 0u : i + 1u);
    }

    E& slot(uint8_t h) { return slots_[h - 1]; }

    /// Where an interrupt could land between two of the consumer's
    /// shared accesses: nothing on a target, the explorer on a host
    /// platform that offers it.
    static void point() {
        if constexpr (requires { Plat::interleave_point(); }) {
            Plat::interleave_point();
        }
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

    /// Read a cursor the other side writes: fresh, and ordered before
    /// the slot or ring access it guards.
    static uint8_t load_shared(const uint8_t& cursor) {
        const uint8_t v = *const_cast<const volatile uint8_t*>(&cursor);
        std::atomic_signal_fence(std::memory_order_acquire);
        return v;
    }

    /// Publish a cursor after the slot or ring access it covers is
    /// complete.
    static void store_shared(uint8_t& cursor, uint8_t value) {
        std::atomic_signal_fence(std::memory_order_release);
        *const_cast<volatile uint8_t*>(&cursor) = value;
    }

    E slots_[slot_count]{};      // slot number h is slots_[h - 1]
    uint8_t ring_[positions]{};  // position p at ring_[next(p)], its number XOR next(p)
    uint8_t alloc_{0};    // the producers', under their critical section; the consumer reads it
    uint8_t take_{0};     // the consumer's
    uint8_t fence_{0};    // the consumer's: the position after the free run's end; the producers read it
    uint8_t held_{0};     // the consumer's: the AO held the slot being dispatched
    uint16_t overflows_{0};
};

} // namespace brio
