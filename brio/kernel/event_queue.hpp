/*
 * event_queue.hpp
 *
 * The event queue of one active object: where its events wait between
 * being posted and being handled.
 *
 * An active object declares one, sized by itself:
 *
 *     static inline brio::EventQueue<Event, 4, P> queue;
 *
 * E is the object's Event type, `depth` how many events may be waiting
 * (or held: kept by the object past the dispatch of the event, hold()
 * below) at once while one is being dispatched, P the platform whose
 * critical section guards the producers' side. An application touches
 * the queue through post() (kernel/post.hpp) and, rarely, through
 * overflows(); the kernel loop drives the consumer side.
 *
 * Who may call what:
 *  - push() (what post() calls): any context, an interrupt handler or
 *    main-loop code alike, at any time. It copies the event into a free
 *    slot with interrupts masked for the few instructions of the copy,
 *    and never waits: on a full queue the event is dropped and a
 *    saturating counter, overflows(), is incremented. A full queue is a
 *    sizing mistake, and the counter names the queue;
 *  - take(), at(), hold(), take_hold(), release(), pop(): the consumer's,
 *    which is the kernel loop dispatching this object (and the object
 *    itself, inside a dispatch of its own, for hold() and for the
 *    release() of a slot it held). Main-loop context only, never from
 *    an interrupt handler, never from another object; they take no
 *    critical section;
 *  - empty(), size(): readable from anywhere, as a snapshot;
 *    overflows(): readable from anywhere, under the critical section,
 *    because the counter is sixteen bits and two accesses on an 8-bit
 *    core.
 *
 * Depth, in one rule: a push is refused when every slot is occupied,
 * waiting or outstanding (the one being dispatched, and any the object
 * holds). The queue has depth + 1 slots, so a producer finds `depth`
 * places while a dispatch is in progress and nothing is held; an idle
 * queue accepts depth + 1. Each hold takes one of the depth for as long
 * as it lasts, so an object sizes its depth as a count: the events it
 * holds at once plus the waiting peak. Depth is any number from 1 to
 * 254; there is no power-of-two rule, because slots are a few bytes to
 * a few words and rounding a depth of 5 up to 8 would spend real RAM to
 * speed up a wrap that is already two instructions.
 *
 * The queue belongs to one core, the platform's: its critical section
 * masks that core's interrupts and nothing else. On a chip with two
 * cores, events cross between them through util/inbox.hpp, never
 * through push() from the other core; where the platform can tell
 * which core is running (`on_own_core()`, kernel/platform.hpp), push()
 * refuses and counts a copy from the wrong core (misposts()). A
 * single-core platform compiles no such check.
 *
 * The rest of this comment is the implementation, for whoever reads or
 * changes it. Three words are used throughout. A SLOT is the storage
 * for one event; the queue has depth + 1 slots, numbered from 1. A
 * POSITION is one byte of a ring that holds a slot's number; the ring
 * has depth + 2 positions. A CURSOR is an index into the ring that one
 * side of the queue advances; there are three. docs/design/event-queue.svg
 * draws one state of all of this.
 *
 * Slots, and the ring of slot numbers
 *
 * An event is built in a slot by push() and stays there, read in place,
 * until the slot is released: right after its dispatch, or later when
 * the object holds it. No event is ever moved. What moves is slot
 * numbers, in the ring: the ring records which slots are free, which
 * hold a waiting event, and in what order. The three cursors cut the
 * ring into three runs, in ring order:
 *
 *     [take_, alloc_)         PENDING      the numbers of built events, oldest first
 *     [alloc_, free end)      FREE         the numbers a producer may build in
 *     [free end, take_)       OUTSTANDING  positions whose slots are taken and not
 *                                          yet released; stale, except the last
 *                                          one, which hold() reads back (below)
 *
 * The free end is the third cursor, fence_, stored as the position after
 * the run: the array place the next released number is written at ("Why
 * a static queue is all zero" says why).
 *
 * A producer builds its event in the slot whose number sits at alloc_
 * and publishes it by advancing alloc_: the advance moves that number
 * from the free run into the pending run, so allocation order is push
 * order and the free run doubles as the order of the pending one. The
 * consumer takes the number at take_ and dispatches the event in its
 * slot. A release writes the slot's number at the free run's end and
 * advances that end; slots may come back in any order, and each costs
 * one byte written. The ring has one position more than there are
 * slots, so the outstanding run is never empty while anything is
 * outstanding (one position is always spare there). An empty pending
 * run is take_ == alloc_, an empty free run is alloc_ meeting the free
 * end, and no count is shared between the two sides.
 *
 * The producers' side
 *
 * push() runs inside Platform::CriticalSection, and the critical section
 * is there for the producers against each other: a post from the main
 * loop and a post from an interrupt handler both write alloc_, and the
 * second can land in the middle of the first. Under the cooperative
 * kernel that is the only concurrency there is, and a small core may
 * offer no compare-and-swap instruction at all, so the brief
 * interrupts-off window is the primitive. The event is built in its
 * slot (for a variant, the alternative is constructed there directly,
 * never first assembled on the caller's stack), so the one copy a post
 * makes is the one the mask covers. The producers write one cursor,
 * alloc_, and read the free end. There is no separate variant of push() for
 * interrupt context: one function, safe from any context; the cycles a
 * split would save would not pay for the doubled surface and the risk
 * of calling the wrong one.
 *
 * The consumer's side, which never masks
 *
 * The consumer writes take_ and the free end and reads alloc_. Every
 * cursor is one byte written by one side only, and a byte is one access
 * on every platform (kernel/platform.hpp's atomic_width), so neither
 * side can see a cursor half written. The runs keep the two sides apart: a
 * producer reads only the free run, which the consumer filled before it
 * moved the free end; the consumer reads only the pending run, whose
 * positions nobody touched since a producer wrote them; the consumer
 * writes only the outstanding run, which no producer reads. What
 * remains is ordering: a slot or a ring position must be complete
 * before the cursor that publishes it moves. That ordering is a
 * compiler-only fence, std::atomic_signal_fence (util/ring.hpp uses the
 * same idiom): it stops the compiler from moving the cursor's store
 * ahead of the stores it publishes, and nothing more is needed because
 * the other side runs on the same core, as an interrupt handler of the
 * code it interrupts or as the code an interrupt handler returns to,
 * and a core sees its own stores in program order. A second core would
 * not ("One core", below).
 *
 * The handle, and holding a slot
 *
 * take() returns the slot's number as a Handle (1 to depth + 1; `none`,
 * zero, when nothing waits), and at() the event in that slot. The
 * number is fixed from push() to release(); positions are the queue's
 * own bookkeeping and nobody outside keeps one. The kernel releases the
 * slot after the dispatch unless the object called hold() inside it:
 * hold() returns a Held<E> (kernel/borrowed.hpp, Lease::hold) and the
 * slot stays the object's, its event intact and read with at(), until
 * the object's own release() in a later dispatch of its own. Release
 * order is free, so a hold costs exactly its own slot for as long as it
 * lasts, never the slots behind it.
 *
 * Why a static queue is all zero
 *
 * A static EventQueue lands in .bss, like every kernel static, whenever
 * the event type's value-initialised state is all zero, as every Fsm
 * event's is (Entry, the first alternative, has no field); an event type
 * with non-zero defaults makes the queue a .data image instead. Three
 * encoding choices make the initial bookkeeping zero, and together they
 * cost the masked window one XOR:
 *  - the free run's end is stored as the position after it, the fence
 *    (fence_), which is zero at the start, when the free run is every
 *    slot. push() computes next(alloc_) anyway to publish, and the
 *    queue is full when that value meets the fence;
 *  - each position is stored one place ahead in the array (position p
 *    in ring_[next(p)]), so the place a release writes is the fence
 *    itself and the place a take reads is the next(take_) it computes
 *    anyway, with no step back anywhere;
 *  - each number is stored XOR its place in the array, and the slots
 *    are numbered from 1, so the first state, slot i at place i, reads
 *    zero, and the spare position decodes to 0, which is `none`. The
 *    slot's address takes the one off: folded into the base or an
 *    offset where the core's addressing allows (the Cortex-M4's), one
 *    subtraction where it does not (the AVR's, the Cortex-M0+'s).
 *
 * Overflow
 *
 * push() never blocks and never fails from the caller's point of view;
 * the overflow counter is read by gdb on a target and by asserts in the
 * host tests. Whether a program counts or panics on an overflow is the
 * layer above's decision, not this header's. The depth limit of 254 is
 * the handle's: slot numbers 1 to depth + 1 are bytes and 0 is `none`.
 *
 * One core
 *
 * The fences above order nothing between cores and the critical section
 * masks one core's interrupts, so a push from another core would race
 * the owner's pushes and takes. The mispost check runs before push()
 * touches anything, and its counter saturates like overflow's.
 *
 * A preemptive platform's queue
 *
 * On a platform that is Preemptive (kernel/platform.hpp), the queue
 * carries one byte more, its owner's scheduling priority (QueuePriority
 * below), which post() compares with the platform's ceiling after the
 * push. Nothing else of the queue changes: a post that activates a
 * dispatch does so after push() has returned, outside its critical
 * section, and the consumer is still one, the owner, because an object
 * is never dispatched while a dispatch of its own is suspended.
 *
 * A host platform that explores interleavings offers `interleave_point()`
 * (kernel/platform.hpp): take() and release() call it at every boundary
 * between two of their shared accesses, where an interrupt could land,
 * and the host suite pushes from there (pop() has no such points of its
 * own, its release() does; hold() touches the consumer's own data only
 * and has none). No target has it, and nothing is compiled for it.
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

/// The scheduling priority of a queue on a preemptive platform
/// (kernel/platform.hpp's Preemptive): the pack index of the active
/// object that owns it, 0 the highest, written by Rubato's init_all()
/// (kernel/rubato.hpp) and compared by post() with the platform's
/// ceiling. 255 means none - an object in no Rubato pack, never activated
/// by a post - and is the value before init_all(). It is stored XOR 255,
/// so that this default is the all-zero state and a static queue stays
/// in .bss ("Why a static queue is all zero" above). Every other queue
/// carries NOTHING of it: an empty base, no byte.
struct QueuePriority {
    uint8_t priority() const { return static_cast<uint8_t>(inverted_ ^ 0xFFu); }
    void set_priority(uint8_t index) { inverted_ = static_cast<uint8_t>(index ^ 0xFFu); }

private:
    uint8_t inverted_{0};   // the priority XOR 255: zero is none
};
struct NoQueuePriority {};

template <typename E, uint8_t depth, Platform Plat>
class EventQueue
    : private std::conditional_t<CoreAware<Plat>, MispostCounter, NoMispostCounter>,
      public std::conditional_t<Preemptive<Plat>, QueuePriority, NoQueuePriority> {
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
    /// hand out the number of the slot pop() already released. (serve_one()
    /// is kernel/active_object.hpp's step for one object.) A hold()
    /// outside a dispatch also leaves the flag set, so the NEXT dispatch's
    /// slot is never released - kept for ever, one slot the less - and
    /// the handle it returns names the previous dispatch's slot, already
    /// released and possibly reused: releasing that handle is the double
    /// release release() calls fatal.
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

    /// No event waits (the outstanding ones are not waiting). Forced
    /// inline: the preemptive kernel's scan asks it of every queue on
    /// every pass, the cooperative one of every queue on a pass that
    /// served nothing, and at -Os GCC may keep it out of line where it
    /// is called from many sites, the call costing more than its two
    /// loads and a compare.
    [[gnu::always_inline]] bool empty() const {
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
