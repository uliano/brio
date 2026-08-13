/*
 * ring.hpp
 *
 * Single-producer / single-consumer circular buffer for ISR <-> main-loop
 * traffic. Ported from uliano/AVR-Multislope; modernized 07/2026:
 * brio namespace, index type derived from the size (one template parameter
 * less), std::has_single_bit instead of the hand-rolled check.
 *
 * Concurrency model: one side runs in an ISR (use the *_from_isr methods,
 * no locking - AVR ISRs are not preempted), the other side runs in the
 * main loop (use the plain methods, which wrap the operation in
 * ATOMIC_BLOCK). Members are not volatile: the cli/sei memory barriers of
 * ATOMIC_BLOCK force the compiler to re-read memory.
 *
 * Capacity is (size - 1): one slot is sacrificed to distinguish full from
 * empty without a separate counter.
 */

#pragma once

#include <stdint.h>
#include <bit>
#include <type_traits>
#include <util/atomic.h>

namespace brio {

template <typename T, int size>
class Ring {
    static_assert(size > 1, "ring size must be at least 2");
    static_assert(std::has_single_bit(static_cast<unsigned>(size)),
                  "ring size must be a power of 2 (fast bit-mask wrap)");
    static_assert(size <= 65536, "ring size must fit a 16-bit index");

public:
    /// Smallest unsigned type that can index this ring.
    using index_t = std::conditional_t<(size <= 256), uint8_t, uint16_t>;

private:
    T m_data[size]{};
    index_t m_head{0};
    index_t m_tail{0};

    static constexpr index_t mask = static_cast<index_t>(size - 1);

    inline void advance_no_atomic(index_t &value) {
        value = static_cast<index_t>((value + 1) & mask);
    }

    inline index_t size_no_atomic() const {
        return static_cast<index_t>((m_head - m_tail) & mask);
    }

    inline bool empty_no_atomic() const {
        return m_head == m_tail;
    }

    inline bool full_no_atomic() const {
        return size_no_atomic() == capacity();
    }

    inline bool get_no_atomic(T &out_value) {
        if (empty_no_atomic()) {
            return false;
        }
        out_value = m_data[m_tail];
        advance_no_atomic(m_tail);
        return true;
    }

    inline bool try_put_no_atomic(const T &c) {
        if (full_no_atomic()) {
            return false;
        }
        m_data[m_head] = c;
        advance_no_atomic(m_head);
        return true;
    }

    inline void put_overwrite_no_atomic(const T &c) {
        m_data[m_head] = c;
        advance_no_atomic(m_head);
        if (m_head == m_tail) {  // was full: discard the oldest element
            advance_no_atomic(m_tail);
        }
    }

public:
    static constexpr index_t capacity() {
        return static_cast<index_t>(size - 1);
    }

    inline index_t count() const {
        index_t result;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            result = size_no_atomic();
        }
        return result;
    }

    inline index_t count_from_isr() const {
        return size_no_atomic();
    }

    inline bool empty() const {
        bool result;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            result = empty_no_atomic();
        }
        return result;
    }

    inline bool empty_from_isr() const {
        return empty_no_atomic();
    }

    inline bool full() const {
        bool result;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            result = full_no_atomic();
        }
        return result;
    }

    inline bool full_from_isr() const {
        return full_no_atomic();
    }

    /// Add an element, overwriting the oldest one when full (always succeeds).
    void put(const T &c) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            put_overwrite_no_atomic(c);
        }
    }

    inline void put_from_isr(const T &c) {
        put_overwrite_no_atomic(c);
    }

    /// Add an element only if there is room; false when full.
    inline bool try_put(const T &c) {
        bool inserted;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            inserted = try_put_no_atomic(c);
        }
        return inserted;
    }

    inline bool try_put_from_isr(const T &c) {
        return try_put_no_atomic(c);
    }

    /// Retrieve and remove the oldest element; false when empty.
    bool get(T &out_value) {
        bool has_data;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            has_data = get_no_atomic(out_value);
        }
        return has_data;
    }

    inline bool get_from_isr(T &out_value) {
        return get_no_atomic(out_value);
    }

    void clear() {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            m_head = 0;
            m_tail = 0;
        }
    }

    inline void clear_from_isr() {
        m_head = 0;
        m_tail = 0;
    }
};

} // namespace brio
