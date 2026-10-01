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
 * atomically; a target with DMA producers needs a producer index
 * that IS the hardware counter (docs/design/overview.md, "Authority
 * of util/").
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <atomic>
#include <bit>
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

} // namespace brio
