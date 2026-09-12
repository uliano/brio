/*
 * event_queue.hpp
 *
 * The per-AO event queue of the brio kernel: multi-producer (any ISR and
 * any main-loop code may push), single-consumer (the scheduler pops).
 *
 * Concurrency: every operation runs inside Platform::CriticalSection.
 * Under the cooperative kernel the only real concurrency is ISR vs main
 * loop, and a small core may offer no CAS at all: the brief
 * interrupts-off section IS the honest primitive (copying an 8-byte
 * event costs about 1 us at 24 MHz).
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
 *
 * THE QUEUE BELONGS TO ONE CORE: the platform's. Its critical section
 * masks the interrupts of the core that takes it and nothing else, so
 * a push from another core would race the owner's pushes and pops on
 * `count_` - which is why events cross cores through util/inbox.hpp
 * and never through push(). A platform that can tell which core is
 * running offers `on_own_core()` (kernel/platform.hpp), and push() then
 * REFUSES a copy from the wrong core before it touches anything: the
 * event is dropped and a saturating counter, `misposts()`, names the
 * mistake - the same economy as overflow. A single-core platform has no
 * such member and compiles no such check.
 */

#pragma once

#include <stdint.h>
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

public:
    /// The platform whose critical section guards this queue: the core
    /// the owning AO lives on, on a chip with more than one.
    using Platform = Plat;

    /// Copy an event into the queue. Full queue: drop + count, never block.
    /// A push from a core that is not the queue's (a platform that can
    /// tell): dropped and counted as a mispost, the queue untouched.
    void push(const E& e) {
        if constexpr (CoreAware<Plat>) {
            if (!Plat::on_own_core()) {
                if (this->misposts_ != UINT16_MAX) {
                    ++this->misposts_;
                }
                return;
            }
        }
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

    static constexpr uint8_t capacity() { return depth; }

private:
    E slots_[depth]{};
    uint8_t head_{0};
    uint8_t tail_{0};
    uint8_t count_{0};
    uint16_t overflows_{0};
};

} // namespace brio
