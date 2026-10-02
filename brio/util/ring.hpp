/*
 * ring.hpp
 *
 * Single-producer / single-consumer FIFO for ISR <-> main-loop traffic,
 * and for anything else that needs a bounded FIFO between exactly two
 * parties: driver byte rings (UART, I2C, SPI), sample buffers, logs.
 *
 * Concurrency model (SPSC): the producer only ever writes head_, the
 * consumer only ever writes tail_, each reads the other's index. When an
 * index is a single naturally-atomic access for the target (sizeof
 * (index_t) <= P::atomic_width - one byte on an 8-bit core, a word on a
 * 32-bit one),
 * every operation is LOCK-FREE: no interrupt masking, no added interrupt
 * latency, both sides may call the same functions from ISR or main
 * context. A stale read of the OTHER side's index only errs on the safe
 * side (the producer underestimates room, the consumer underestimates
 * data). A wider index - a capacity above 256 where the index is a
 * byte - is torn by an interrupt, so
 * every operation is wrapped in P::CriticalSection instead - selected
 * with if constexpr, invisible to the caller. Ordering between the slot
 * copy and the index publish is enforced with std::atomic_signal_fence
 * (a compiler-only fence: correct on single-core targets, free).
 *
 * The one API is therefore always safe: there are no *_from_isr twins
 * (docs/design/overview.md's style rule: no API doubling). Only clear()
 * is NOT concurrent: it rewrites both
 * indices and is legal only while the other party is quiescent (init,
 * or after masking its interrupt).
 *
 * TWO GRANULARITIES, ONE CONTRACT. push()/pop() move one element and
 * suit an ISR handed one byte at a time; read_span()/consume() and
 * write_span()/publish() hand a party the CONTIGUOUS RUN it already owns
 * so it can move the whole thing at once - a DMA block, a memcpy, a
 * bulk write. The spans change nothing about the concurrency model:
 * each side still writes only its own index and reads only the other's,
 * and the run a side is given is exactly the memory the SPSC invariant
 * already made private to it. A span never wraps (it stops at the end of
 * the buffer and the rest comes on the next call), and it stays valid
 * until its own side's next operation.
 *
 * Capacity is (size - 1): one slot is sacrificed to distinguish full from
 * empty without a shared counter (a counter would be written by both
 * sides and break the SPSC rule). No overwrite-oldest push either: the
 * producer would have to move tail_, again breaking the rule; a full
 * ring reports false and the caller counts or blocks (its policy).
 *
 * The lock-free path assumes an index the platform reads/writes
 * atomically (docs/design/overview.md, "Authority of util/"); the
 * strata Ring runs on, and the path each takes, are
 * docs/design/ring.md's realizations table.
 *
 * A SECOND FLAVOUR, for a producer that stores no index at all.
 * HardwareRing (below Ring) is the consumer half of a ring whose
 * producer is the hardware - a DMA channel writing the storage lap after
 * lap in circular mode - so its producer index IS the channel's counter,
 * read through a function and never stored by anybody. Same consumer
 * verbs under the same names; what it adds is the one thing a producer
 * that cannot be stopped forces on the consumer: the accounting of the
 * laps it did not keep up with. Its contract carries its own
 * assumptions, stated where it is defined.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <atomic>
#include <bit>
#include <concepts>
#include <optional>
#include <span>
#include <type_traits>

#include "kernel/platform.hpp"

namespace brio {

template <typename T, uint32_t size, Platform P>
class Ring {
    static_assert(size > 1, "ring size must be at least 2");
    static_assert(std::has_single_bit(size),
                  "ring size must be a power of 2 (fast bit-mask wrap)");
    static_assert(std::is_trivially_copyable_v<T>,
                  "slots are copied byte-wise, possibly from an ISR");

public:
    /// Smallest unsigned type that can index this ring. No upper bound
    /// on size here: what fits in RAM is the target's business, and the
    /// lock-free/guarded choice below follows the index width anyway.
    using index_t = std::conditional_t<(size <= 256), uint8_t,
                    std::conditional_t<(size <= 65536), uint16_t, uint32_t>>;

    /// True when head_/tail_ are shared bare (see the header comment).
    static constexpr bool lock_free = sizeof(index_t) <= P::atomic_width;

    static constexpr index_t capacity() {
        return static_cast<index_t>(size - 1);
    }

    /// Append one element; false (nothing written) when the ring is full.
    [[gnu::always_inline]] bool push(const T& value) {
        if constexpr (lock_free) {
            return push_body(value);
        } else {
            typename P::CriticalSection cs;
            return push_body(value);
        }
    }

    /// Remove and return the oldest element; nullopt when empty.
    [[gnu::always_inline]] std::optional<T> pop() {
        if constexpr (lock_free) {
            return pop_body();
        } else {
            typename P::CriticalSection cs;
            return pop_body();
        }
    }

    // ---- bulk access: the contiguous run each side owns right now ---------
    //
    // push() and pop() move one element per call, which is what an ISR
    // that is handed one byte at a time wants. A party that can move
    // MANY at once - a DMA engine given a block, a memcpy, a write()
    // that takes a buffer - wants instead to be told where its own run
    // of the buffer is and how long it is, do the work itself, and then
    // say how much of it it used.
    //
    // THE SPSC CONTRACT IS UNCHANGED, and that is the whole design:
    //  - the CONSUMER's run is the slots from tail_ up to head_, which
    //    only the consumer may read and only the consumer's consume()
    //    releases;
    //  - the PRODUCER's run is the free slots from head_ up to one
    //    before tail_, which only the producer may write and only the
    //    producer's publish() hands over.
    // Each side still writes only its own index and reads only the
    // other's, so the lock-free path stays exactly as correct as it was
    // for push/pop, and the guarded path wraps the same computation.
    //
    // A SPAN NEVER WRAPS. It stops at the end of the buffer, so a ring
    // whose data straddles the wrap reports the first part and offers
    // the rest on the next call. That is deliberate: a caller handing
    // the run to a DMA block or a memcpy needs ONE contiguous region,
    // and two calls are cheaper than the alternative of pretending.
    //
    // A span is valid until its OWN side's next operation on the ring;
    // the other side cannot invalidate it (it can only make it more
    // conservative than it needs to be, which is safe).

    /// The contiguous run of elements ready to be read, starting at the
    /// tail. Empty when the ring is empty. Consumer side only.
    std::span<const T> read_span() const {
        if constexpr (lock_free) {
            return read_span_body();
        } else {
            typename P::CriticalSection cs;
            return read_span_body();
        }
    }

    /// Release `n` elements the consumer has finished with, oldest
    /// first. Clamped to what is actually queued, so an over-long
    /// release cannot walk the tail past the head. Consumer side only.
    void consume(index_t n) {
        if constexpr (lock_free) {
            consume_body(n);
        } else {
            typename P::CriticalSection cs;
            consume_body(n);
        }
    }

    /// The contiguous run of free slots the producer may fill, starting
    /// at the head. Empty when the ring is full. Producer side only.
    std::span<T> write_span() {
        if constexpr (lock_free) {
            return write_span_body();
        } else {
            typename P::CriticalSection cs;
            return write_span_body();
        }
    }

    /// Hand `n` freshly written elements to the consumer. Clamped to the
    /// free room, so an over-long publish cannot walk the head into the
    /// tail and make a full ring read as empty. Producer side only.
    void publish(index_t n) {
        if constexpr (lock_free) {
            publish_body(n);
        } else {
            typename P::CriticalSection cs;
            publish_body(n);
        }
    }

    /// Elements currently queued (a snapshot; exact for the calling side's
    /// own view, conservative for the other).
    [[gnu::always_inline]] index_t count() const {
        if constexpr (lock_free) {
            return count_body();
        } else {
            typename P::CriticalSection cs;
            return count_body();
        }
    }

    [[gnu::always_inline]] bool empty() const { return count() == 0; }
    [[gnu::always_inline]] bool full() const { return count() == capacity(); }

    /// Reset to empty. NOT concurrent: both parties must be quiescent.
    void clear() {
        head_ = 0;
        tail_ = 0;
    }

private:
    static constexpr index_t mask = static_cast<index_t>(size - 1);

    T slots_[size]{};
    index_t head_{0};  // written by the producer only
    index_t tail_{0};  // written by the consumer only

    // ---- the bodies: one per verb, run bare or under the guard ------------
    //
    // Each verb above picks its path with if constexpr and runs its body
    // directly, so the lock-free verb IS its body and the guarded one is
    // the same body inside P::CriticalSection. Always inline, and no
    // lambda handed to a guard helper: at -Os gcc keeps such a lambda as
    // a function of its own on the QingKe cores, its closure built on the
    // stack at every call site - a call per byte on a print and in a
    // receive interrupt for one computation of a few instructions.
    //
    // The ELEMENT verbs - push(), pop(), count() and the two predicates
    // over it - are always inline themselves, because they sit on the
    // per-byte paths: a transport's interrupt body and its blocking
    // write. Left to -Os, gcc keeps pop() out of line as soon as it has
    // two call sites (a transmit vector and a read_byte(), on the QingKe
    // cores), and a call in an interrupt body makes it a non-leaf
    // function, which saves every caller-saved register the callee may
    // clobber - twenty f-registers more under ilp32f. Inline, each verb
    // is a few loads, a compare and a store at its call site, about the
    // size of the call it replaces. The span verbs are left to the
    // compiler: they run once per run, and a call per run is the run's
    // own price.

    [[gnu::always_inline]] bool push_body(const T& value) {
        const index_t head = head_;
        const index_t next = static_cast<index_t>((head + 1) & mask);
        if (next == load_other(tail_)) {
            return false;
        }
        slots_[head] = value;
        store_index(head_, next);
        return true;
    }

    [[gnu::always_inline]] std::optional<T> pop_body() {
        const index_t tail = tail_;
        if (tail == load_other(head_)) {
            return std::nullopt;
        }
        const T value = slots_[tail];
        store_index(tail_, static_cast<index_t>((tail + 1) & mask));
        return value;
    }

    [[gnu::always_inline]] std::span<const T> read_span_body() const {
        const index_t tail = tail_;
        const index_t head = load_other(head_);
        if (tail == head) {
            return {};
        }
        // Stop at head when the data does not wrap, at the end of the
        // buffer when it does. THE WIDTH IS NAMED: `size` is one more
        // than the largest index_t value at the two boundary sizes (256,
        // 65536), so casting it down would turn the whole-buffer run
        // into a zero-length one.
        const uint32_t end = (head > tail) ? static_cast<uint32_t>(head) : size;
        return {&slots_[tail], static_cast<size_t>(end - tail)};
    }

    [[gnu::always_inline]] void consume_body(index_t n) {
        const index_t tail = tail_;
        const index_t available =
            static_cast<index_t>((load_other(head_) - tail) & mask);
        const index_t take = (n < available) ? n : available;
        store_index(tail_, static_cast<index_t>((tail + take) & mask));
    }

    [[gnu::always_inline]] std::span<T> write_span_body() {
        const index_t head = head_;
        const index_t tail = load_other(tail_);
        // 32-bit throughout, for the same reason read_span_body() names
        // its width: at size 65536 the whole-buffer run does not fit in
        // index_t.
        uint32_t room;
        if (tail > head) {
            // The free run ends one slot short of the tail: that spare
            // slot is what tells full from empty.
            room = static_cast<uint32_t>(tail) - head - 1u;
        } else {
            // Up to the end of the buffer - and one short of it when the
            // tail sits at zero, for the same reason.
            room = size - head - (tail == 0u ? 1u : 0u);
        }
        if (room == 0u) {
            return {};
        }
        return {&slots_[head], static_cast<size_t>(room)};
    }

    [[gnu::always_inline]] void publish_body(index_t n) {
        const index_t head = head_;
        const index_t free_room =
            static_cast<index_t>((load_other(tail_) - head - 1u) & mask);
        const index_t give = (n < free_room) ? n : free_room;
        store_index(head_, static_cast<index_t>((head + give) & mask));
    }

    [[gnu::always_inline]] index_t count_body() const {
        return static_cast<index_t>((load_other(head_) - load_other(tail_)) & mask);
    }

    /// Read the index owned by the other side: fresh (never hoisted or
    /// cached across calls) and ordered before the slot access it guards.
    static index_t load_other(const index_t& idx) {
        const index_t v = *const_cast<const volatile index_t*>(&idx);
        std::atomic_signal_fence(std::memory_order_acquire);
        return v;
    }

    /// Publish our own index after the slot access it covers is complete.
    static void store_index(index_t& idx, index_t value) {
        std::atomic_signal_fence(std::memory_order_release);
        *const_cast<volatile index_t*>(&idx) = value;
    }
};

// =============================================================================
// HardwareRing - the ring whose producer is the hardware
// =============================================================================

/**
 * What a HardwareRing reads its producer through: two static functions,
 * each ONE READ of the producer's state.
 *
 *  - remaining(): the elements still to be written in the current lap,
 *    in [0, size] - a circular channel's own count register, which the
 *    hardware reloads to the storage's length at every wrap. It counts
 *    ELEMENTS (the channel's beats), never bytes, and it counts an
 *    element as written only once a read of its slot returns it.
 *  - laps(): the laps completed since the producer started at the
 *    storage's first element - the count the channel's completion
 *    interrupt increments at every wrap. It may LAG the counter (the
 *    handler runs after the wrap it counts) and must never LEAD it.
 *
 * Each is called from the consumer's context; a read that would be torn
 * there (a 32-bit count on a byte-atomic core) is the producer's to make
 * whole, because the producer is the one that knows its platform.
 */
template <typename C>
concept RingCounter = requires {
    { C::remaining() } -> std::convertible_to<uint32_t>;
    { C::laps() } -> std::convertible_to<uint32_t>;
};

/**
 * HardwareRing<storage, Counter> - the CONSUMER HALF of a ring whose
 * producer is the hardware. A DMA channel in circular mode writes
 * `storage` (a caller-owned array: the caller places and aligns it where
 * the channel can reach) lap after lap, and the producer index is not a
 * variable anybody stores: it is the channel's own count, read through
 * Counter, head = size - remaining().
 *
 * A MONOSTATE, like the engines and transports it sits between: the
 * array named in the type is its identity, so two views of one storage
 * are one view, and with a byte element it is util/stream.hpp's
 * SpanSource itself. The verbs are Ring's consumer half under Ring's
 * names - read_span()/consume(), pop(), count(), empty(), capacity(),
 * clear() - so a transport moving its receive side onto a circular
 * channel changes a type and not its code.
 *
 * POSITIONS, NOT INDICES. The view keeps two 32-bit positions counted
 * from the producer's start (modulo 2^32): the consumer's tail, and the
 * producer's head as last looked at, head = laps() * size + (size -
 * remaining()) mod size. Laps are read FIRST and the count second, so a
 * wrap landing between the two reads, or a completion handler not yet
 * run, makes the head a lap TOO FEW and never a lap too many - and a lap
 * too few shows as the head going backwards from the last look, which is
 * impossible, so it is added back. That inference is exact while the
 * consumer looks more often than once a lap.
 *
 * THE OVERRUN. The producer cannot be stopped and is never told where
 * the tail is, so a consumer that falls a lap behind is overwritten, and
 * the lap count is the only witness: head - tail >= size means the oldest
 * unread element is the producer's next write or already written over.
 * Every look that finds it counts one overrun and SKIPS - the tail jumps
 * to the head, everything unread discarded - because what is left within
 * a lap of the producer is racing it: skip rather than tear, the block
 * streams' doctrine (util/block_stream.hpp). The capacity is therefore
 * size - 1, as Ring's: a full lap unread is already an overrun.
 *
 * THE RUN IS JUDGED AT ITS RELEASE. A run handed out can be written over
 * while the consumer still holds it - the consumer is a lap behind and
 * slow, or interrupted. consume() looks again, AFTER the run's reads (a
 * fence keeps them before it): if the slot at the tail was written over
 * (head - tail > size), the run was torn, the overrun is counted, the
 * view skips, and consume() answers false - the one place a consumer can
 * learn that what it just read was overwritten under it. A run released
 * with true was intact when it was read.
 *
 * NO CRITICAL SECTION AND NO PLATFORM. The view's state is the consumer's
 * alone (one reader); the producer's two numbers arrive through Counter
 * in one read each. The fences are Ring's: compiler-only, enough on the
 * in-order cores with no data cache between the channel and the core
 * that every family with DMA has today - the assumption this contract
 * carries (docs/design/overview.md, "Authority of util/"). Validated on
 * the host against a scripted counter (test/test_ring), and on the
 * silicon by the circular receive engines of the STM32F4, the STM32G0
 * and the CH32V203, whose transports read their receive ring through
 * it (each family's dma.md).
 *
 * WHAT IT CANNOT KNOW. An element landing between the count read and the
 * run's use is not in the run: it is the next call's. A lap missed while
 * the completion that counts it is still pending, by a consumer that has
 * not looked for a whole lap, is counted at the first look after the
 * handler has run - the run handed in between holds the stream's real
 * elements from after the gap. A completion handler held off for a whole
 * lap loses that lap from the count (one flag latches one wrap), and the
 * view cannot see a lap the count never had. And a consumer that does not
 * look for 2^31 elements outruns the positions' arithmetic.
 */
template <auto& storage, RingCounter Counter>
class HardwareRing {
    using array_type = std::remove_reference_t<decltype(storage)>;
    static_assert(std::is_bounded_array_v<array_type>,
                  "the storage is an array of known length");

public:
    using element = std::remove_extent_t<array_type>;

    /// The storage's length in elements: what the producer's count is
    /// loaded with, and the period of every position.
    static constexpr uint32_t size = static_cast<uint32_t>(std::extent_v<array_type>);

    static_assert(size > 1u, "ring size must be at least 2");
    static_assert(std::has_single_bit(size),
                  "ring size must be a power of 2 (the head is a mask and a shift)");
    static_assert(size <= 0x8000'0000u,
                  "a lap must fit the positions' signed difference");
    static_assert(!std::is_const_v<element> && !std::is_volatile_v<element>,
                  "plain storage: the fences, not volatile, order its reads");
    static_assert(std::is_trivially_copyable_v<element>,
                  "slots are written by a channel and copied out byte-wise");

    /// The most a consumer can hold unread: a full lap is an overrun.
    static constexpr uint32_t capacity() { return size - 1u; }

    /// The contiguous run of elements ready to be read, starting at the
    /// tail; empty when there is none. It never wraps - it stops at the
    /// end of the storage and the rest comes on the next call. A lap the
    /// consumer did not keep up with is counted here and skipped, and
    /// the run is then empty.
    static std::span<const element> read_span() {
        const uint32_t unread = look_unread();
        if (unread == 0u) {
            return {};
        }
        const uint32_t tail = tail_ & mask;
        const uint32_t room = size - tail;
        return {&storage[tail], static_cast<size_t>(unread < room ? unread : room)};
    }

    /// Release `n` elements the consumer has finished with, oldest first,
    /// clamped to what is queued. False when the run was written over
    /// while it was held: the overrun is counted, the view has skipped to
    /// the head, and what the consumer read from it is not the stream.
    static bool consume(uint32_t n) { return release(n); }

    /// Remove and return the oldest element; nullopt when there is none,
    /// or when it was written over while it was being read (counted).
    static std::optional<element> pop() {
        if (look_unread() == 0u) {
            return std::nullopt;
        }
        const element value = storage[tail_ & mask];
        if (!release(1u)) {
            return std::nullopt;
        }
        return value;
    }

    /// Elements unread right now - a look at the producer, which counts
    /// and skips a lap missed as read_span() does. Always inline, as
    /// Ring's count(): an edge test in an interrupt body calls it.
    [[gnu::always_inline]] static uint32_t count() { return look_unread(); }
    [[gnu::always_inline]] static bool empty() { return count() == 0u; }

    /// Laps the consumer did not keep up with, and runs written over
    /// while held, since the last clear_overruns(): each one a skip.
    static uint32_t overruns() { return overruns_; }
    static void clear_overruns() { overruns_ = 0u; }

    /// Both positions back to zero, for a producer (re)started at the
    /// storage's first element with its lap count at zero. NOT
    /// concurrent: the consumer is quiescent while it runs.
    static void clear() {
        tail_ = 0u;
        head_ = 0u;
    }

private:
    static constexpr uint32_t mask = size - 1u;
    static constexpr uint32_t shift = static_cast<uint32_t>(std::countr_zero(size));

    static inline uint32_t tail_ = 0u;  // the consumer's position
    static inline uint32_t head_ = 0u;  // the producer's, as last looked at
    // Counted by whichever context consumes and read by any other, which
    // may poll it - so the load is a volatile one.
    static inline volatile uint32_t overruns_ = 0u;

    // ---- the bodies: always inline, so each public verb is one function --

    /// Look at the producer: laps first, then the count, then the lap a
    /// pending completion has not counted yet added back. Records the
    /// head; the fence behind it keeps the slot reads that follow after
    /// the look.
    [[gnu::always_inline]] static uint32_t look() {
        const uint32_t laps = static_cast<uint32_t>(Counter::laps());
        // Whatever the counter's functions compile to, the lap count is
        // read before the count: the order that can only err low.
        std::atomic_signal_fence(std::memory_order_seq_cst);
        const uint32_t remaining = static_cast<uint32_t>(Counter::remaining());
        uint32_t head = (laps << shift) + ((size - remaining) & mask);
        if (static_cast<int32_t>(head - head_) < 0) {
            head += size;  // a wrap whose completion has not been counted yet
        }
        head_ = head;
        std::atomic_signal_fence(std::memory_order_acquire);
        return head;
    }

    /// A look, judged: the unread count, or a lap missed counted and
    /// skipped (the oldest unread element is the producer's next write,
    /// or written over already).
    [[gnu::always_inline]] static uint32_t look_unread() {
        const uint32_t head = look();
        const uint32_t unread = head - tail_;
        if (unread >= size) {
            overruns_ = overruns_ + 1u;
            tail_ = head;
            return 0u;
        }
        return unread;
    }

    /// The release: a second look AFTER the run's reads, which judges
    /// them - the slot at the tail written over means the run was torn.
    [[gnu::always_inline]] static bool release(uint32_t n) {
        std::atomic_signal_fence(std::memory_order_seq_cst);
        const uint32_t tail = tail_;
        const uint32_t head = look();
        const uint32_t queued = head - tail;
        if (queued > size) {
            overruns_ = overruns_ + 1u;
            tail_ = head;
            return false;
        }
        tail_ = tail + (n < queued ? n : queued);
        return true;
    }
};

} // namespace brio
