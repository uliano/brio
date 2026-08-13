/*
 * event_queue.hpp
 *
 * The per-AO event queue of the brio kernel: multi-producer (any ISR and
 * any main-loop code may push), single-consumer (the scheduler pops).
 *
 * Concurrency: every operation runs inside Platform::CriticalSection.
 * Under the cooperative kernel the only real concurrency is ISR vs main
 * loop, and the AVR has no CAS: the brief interrupts-off section IS the
 * honest primitive (copying an 8-byte event costs about 1 us at 24 MHz).
 * There are deliberately NO *_from_isr variants: one API, always safe -
 * the saved cycles would not pay for the doubled surface and the risk of
 * calling the wrong one. Revisit only with measurements.
 *
 * Overflow: push() never blocks and never fails from the caller's point
 * of view. On a full queue the event is dropped and a saturating
 * overflow counter is incremented: a full queue is a SIZING mistake, and
 * the counter names the culprit (from gdb on target, from asserts in
 * host tests). The count-vs-panic reaction knob belongs to the kernel
 * layer above, not here.
 *
 * Depth is arbitrary (no power-of-two rule, no sacrificed slot): event
 * slots are 4-8 bytes each, so rounding a depth of 5 up to 8 would waste
 * real RAM to speed up a wrap that is already two instructions.
 */

#pragma once

#include <stdint.h>
#include <optional>
#include <type_traits>

#include "platform.hpp"

namespace brio {

template <typename E, uint8_t depth, Platform Plat>
class EventQueue {
    static_assert(depth >= 1, "queue depth must be at least 1");
    static_assert(std::is_trivially_copyable_v<E>,
                  "events are copied byte-wise into the queue, possibly "
                  "from an ISR: they must be trivially copyable");

public:
    /// Copy an event into the queue. Full queue: drop + count, never block.
    void push(const E& e) {
        typename Plat::CriticalSection cs;
        if (count_ == depth) {
            if (overflows_ != UINT16_MAX) {  // saturate: never lie by wrapping
                ++overflows_;
            }
            return;
        }
        slots_[head_] = e;
        if (++head_ == depth) {
            head_ = 0;
        }
        ++count_;
    }

    /// Remove and return the oldest event; nullopt when empty.
    std::optional<E> pop() {
        typename Plat::CriticalSection cs;
        if (count_ == 0) {
            return std::nullopt;
        }
        E e = slots_[tail_];
        if (++tail_ == depth) {
            tail_ = 0;
        }
        --count_;
        return e;
    }

    bool empty() const {
        typename Plat::CriticalSection cs;
        return count_ == 0;
    }

    uint8_t size() const {
        typename Plat::CriticalSection cs;
        return count_;
    }

    uint16_t overflows() const {
        typename Plat::CriticalSection cs;
        return overflows_;
    }

    static constexpr uint8_t capacity() { return depth; }

private:
    E slots_[depth]{};
    uint8_t head_{0};
    uint8_t tail_{0};
    uint8_t count_{0};
    uint16_t overflows_{0};
};

} // namespace brio
