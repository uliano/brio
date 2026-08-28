/*
 * ring.hpp
 *
 * Single-producer / single-consumer FIFO for ISR <-> main-loop traffic,
 * and for anything else that needs a bounded FIFO between exactly two
 * parties: driver byte rings (UART, I2C, SPI), sample buffers, logs.
 * Descends from the AVR-Multislope ring (2026-07: brio namespace, index
 * type derived from the size); rewritten 2026-08-16 as a pure util
 * service templated on the Platform.
 *
 * Concurrency model (SPSC): the producer only ever writes head_, the
 * consumer only ever writes tail_, each reads the other's index. When an
 * index is a single naturally-atomic access for the target (sizeof
 * (index_t) <= P::atomic_width - a byte on AVR, a word on 32-bit cores),
 * every operation is LOCK-FREE: no interrupt masking, no added interrupt
 * latency, both sides may call the same functions from ISR or main
 * context. A stale read of the OTHER side's index only errs on the safe
 * side (the producer underestimates room, the consumer underestimates
 * data). Wider indices (size > 256 on AVR) are torn by an interrupt, so
 * every operation is wrapped in P::CriticalSection instead - selected
 * with if constexpr, invisible to the caller. Ordering between the slot
 * copy and the index publish is enforced with std::atomic_signal_fence
 * (a compiler-only fence: correct on single-core targets, free).
 *
 * The one API is therefore always safe: there are no *_from_isr twins
 * (kernel style ruling). Only clear() is NOT concurrent: it rewrites both
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
 * Validated on: AVR DA/DB (atomic_width 1) and the host (4). The
 * lock-free path assumes an index the platform reads/writes
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
    bool push(const T& value) {
        return guarded([&] {
            const index_t head = head_;
            const index_t next = static_cast<index_t>((head + 1) & mask);
            if (next == load_other(tail_)) {
                return false;
            }
            slots_[head] = value;
            store_index(head_, next);
            return true;
        });
    }

    /// Remove and return the oldest element; nullopt when empty.
    std::optional<T> pop() {
        return guarded([&]() -> std::optional<T> {
            const index_t tail = tail_;
            if (tail == load_other(head_)) {
                return std::nullopt;
            }
            const T value = slots_[tail];
            store_index(tail_, static_cast<index_t>((tail + 1) & mask));
            return value;
        });
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
        return guarded([&]() -> std::span<const T> {
            const index_t tail = tail_;
            const index_t head = load_other(head_);
            if (tail == head) {
                return {};
            }
            // Stop at head when the data does not wrap, at the end of
            // the buffer when it does. THE WIDTH IS NAMED: `size` is one
            // more than the largest index_t value at the two boundary
            // sizes (256, 65536), so casting it down would turn the
            // whole-buffer run into a zero-length one.
            const uint32_t end = (head > tail) ? static_cast<uint32_t>(head) : size;
            return {&slots_[tail], static_cast<size_t>(end - tail)};
        });
    }

    /// Release `n` elements the consumer has finished with, oldest
    /// first. Clamped to what is actually queued, so an over-long
    /// release cannot walk the tail past the head. Consumer side only.
    void consume(index_t n) {
        guarded([&] {
            const index_t tail = tail_;
            const index_t available =
                static_cast<index_t>((load_other(head_) - tail) & mask);
            const index_t take = (n < available) ? n : available;
            store_index(tail_, static_cast<index_t>((tail + take) & mask));
        });
    }

    /// The contiguous run of free slots the producer may fill, starting
    /// at the head. Empty when the ring is full. Producer side only.
    std::span<T> write_span() {
        return guarded([&]() -> std::span<T> {
            const index_t head = head_;
            const index_t tail = load_other(tail_);
            // 32-bit throughout, for the same reason read_span() names
            // its width: at size 65536 the whole-buffer run does not fit
            // in index_t.
            uint32_t room;
            if (tail > head) {
                // The free run ends one slot short of the tail: that
                // spare slot is what tells full from empty.
                room = static_cast<uint32_t>(tail) - head - 1u;
            } else {
                // Up to the end of the buffer - and one short of it when
                // the tail sits at zero, for the same reason.
                room = size - head - (tail == 0u ? 1u : 0u);
            }
            if (room == 0u) {
                return {};
            }
            return {&slots_[head], static_cast<size_t>(room)};
        });
    }

    /// Hand `n` freshly written elements to the consumer. Clamped to the
    /// free room, so an over-long publish cannot walk the head into the
    /// tail and make a full ring read as empty. Producer side only.
    void publish(index_t n) {
        guarded([&] {
            const index_t head = head_;
            const index_t free_room =
                static_cast<index_t>((load_other(tail_) - head - 1u) & mask);
            const index_t give = (n < free_room) ? n : free_room;
            store_index(head_, static_cast<index_t>((head + give) & mask));
        });
    }

    /// Elements currently queued (a snapshot; exact for the calling side's
    /// own view, conservative for the other).
    index_t count() const {
        return guarded([&] {
            return static_cast<index_t>((load_other(head_) - load_other(tail_)) & mask);
        });
    }

    bool empty() const { return count() == 0; }
    bool full() const { return count() == capacity(); }

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

    /// Run op lock-free or inside a critical section, per lock_free.
    template <typename Op>
    static decltype(auto) guarded(Op&& op) {
        if constexpr (lock_free) {
            return op();
        } else {
            typename P::CriticalSection cs;
            return op();
        }
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
