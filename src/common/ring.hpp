// Ported from uliano/AVR-Multislope (lib/core/src/ring.hpp).
/*
 * ring.hpp
 *
 * Thread-safe circular buffer with automatic overwrite on full condition.
 *
 * Created: 11/3/2022 5:00:04 PM
 *  Author: uliano
 * Revised: 01/9/2026 - Bug fixes (overflow detection) and documentation
 */

 #pragma once

 #include "utils.hpp"
 #include <stdint.h>
 #include <util/atomic.h>

 /**
  * @brief Thread-safe circular buffer (ring buffer) with overwrite behavior
  *
  * @tparam T Element type stored in the buffer
  * @tparam S Index type (uint8_t for buffers <= 256, uint16_t for larger)
  * @tparam ring_size Buffer size - MUST be a power of 2 (e.g., 8, 16, 32, 64, 128, 256)
  *
  * Features:
  * - Automatic overwrite: When full, new data overwrites the oldest element
  * - Thread-safe: All operations use ATOMIC_BLOCK for ISR safety
  * - Fast indexing: Uses bit-mask (value & (size-1)) instead of modulo
  * - Single producer/consumer: Designed for one writer (ISR) and one reader (main loop)
  *
  * Usage example:
  *   Ring<uint8_t, uint8_t, 256> rx_buffer;  // 256-byte buffer
  *
  *   // In ISR:
  *   rx_buffer.put_from_isr(USART.RXDATA);
  *
  *   // In main loop:
  *   uint8_t byte;
  *   if (rx_buffer.get(byte)) {
  *       process(byte);
  *   }
  *
  * Important notes:
  * - Buffer can hold (ring_size - 1) elements when using is_full()
  * - Overwrite mode keeps the same effective capacity (ring_size - 1)
  * - power-of-2 size is REQUIRED for fast bit-mask wrapping
  */
 template <typename T, typename S, int ring_size>
 class Ring {
     static_assert((S)-1 > 0, "ring index type S must be unsigned");
     static_assert(is_powerof2(ring_size), "ring buffer size should be power of 2");
     static_assert(uint32_t(ring_size) <= (uint32_t(1) << (8U * sizeof(S))),
                   "ring buffer size should be within representable range of index type S");

 private:
     T data[ring_size]{};   // Array to store buffer elements
     S m_head{0};           // Index for the head of the buffer
     S m_tail{0};           // Index for the tail of the buffer

     /**
      * @brief Advances the given index with wraparound using bit-mask
      *
      * Uses bit-mask instead of modulo for speed. Only works when ring_size is power of 2.
      * Example: (value + 1) & 7 wraps 0-7 in a size-8 buffer
      */
     inline void advance_no_atomic(S &value) {
         value = (value + 1) & (ring_size - 1);
     }

     // Returns the current size of the buffer without atomic operations
     inline S size_no_atomic() const {
         return ((m_head - m_tail) & (ring_size - 1));
     }

     // Checks if the buffer is empty without atomic operations
     inline bool empty_no_atomic() const {
         return m_head == m_tail;
     }

     // Checks if the buffer is full without atomic operations
     inline bool is_full_no_atomic() const {
         return size_no_atomic() == ring_size - 1;
     }

     inline bool get_no_atomic(T &out_value) {
         if (!size_no_atomic()) {
             return false;
         }
         out_value = data[m_tail];
         advance_no_atomic(m_tail);
         return true;
     }

     inline bool try_put_no_atomic(const T &c) {
         if (is_full_no_atomic()) {
             return false;
         }
         data[m_head] = c;
         advance_no_atomic(m_head);
         return true;
     }

     inline void put_overwrite_no_atomic(const T &c) {
         data[m_head] = c;
         advance_no_atomic(m_head);
         if (m_head == m_tail) {  // Buffer full: advance tail to overwrite oldest
             advance_no_atomic(m_tail);
         }
     }

     inline void clear_no_atomic() {
         m_head = 0;
         m_tail = 0;
     }

 public:
     static constexpr S capacity() {
         return static_cast<S>(ring_size - 1);
     }

     // Returns the current size of the buffer using atomic operations
     inline S size() const {
         S result;
         ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
             result = size_no_atomic();
         }
         return result;
     }

     inline S size_from_isr() const {
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

     // Checks if the buffer is full using atomic operations
     inline bool is_full() const {
         bool result;
         ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
             result = is_full_no_atomic();
         }
         return result;
     }

     inline bool is_full_from_isr() const {
         return is_full_no_atomic();
     }

     /**
      * @brief Adds an element to the buffer with automatic overwrite
      *
      * Always succeeds - if buffer is full, it overwrites the oldest element.
      * Thread-safe: Uses ATOMIC_BLOCK for ISR protection.
      *
      * Algorithm:
      * 1. Write data at head position
      * 2. Advance head
      * 3. If head caught up with tail (buffer was full), advance tail to discard oldest
      *
      * Note: The check "m_head == m_tail" AFTER advancing head is critical.
      *       Checking BEFORE would incorrectly identify empty buffer as full.
      *
      * @param c Element to add
      */
     void put(T c) {
         ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
             put_overwrite_no_atomic(c);
         }
     }

     inline void put_from_isr(const T &c) {
         put_overwrite_no_atomic(c);
     }

     inline bool try_put(const T &c) {
         bool inserted = false;
         ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
             inserted = try_put_no_atomic(c);
         }
         return inserted;
     }

     inline bool try_put_from_isr(const T &c) {
         return try_put_no_atomic(c);
     }

     /**
      * @brief Retrieves and removes the oldest element from the buffer
      *
      * Thread-safe: Uses ATOMIC_BLOCK for ISR protection.
      *
      * @param out_value Reference to store the retrieved element
      * @return true if element was retrieved, false if buffer was empty
      */
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

     /**
      * @brief Clears the buffer by resetting head and tail pointers
      *
      * Thread-safe: Uses ATOMIC_BLOCK for ISR protection.
      * Note: Does not overwrite data, only resets pointers.
      */
     void clear() {
         ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
             clear_no_atomic();
         }
     }

     inline void clear_from_isr() {
         clear_no_atomic();
     }
 };
