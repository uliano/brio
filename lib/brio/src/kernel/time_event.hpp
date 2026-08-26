/*
 * time_event.hpp
 *
 * Time events: a timer never runs user code - it POSTS an event to its
 * owner AO, so the logic stays serialized in the AO's dispatch. This is
 * the kernel-side replacement of the legacy callback Timer.
 *
 * Mechanics (the "T2" decision): the tick ISR only advances the counter
 * and, by firing, wakes the CPU from idle; expiry runs in the KERNEL
 * LOOP - Kernel::run() calls TimeEvents<P>::process() once per turn,
 * which compares P::now() against the armed deadlines and posts the
 * matured events in main context. Wrap-safe comparison via signed
 * difference: (int32_t)(now - deadline) >= 0 works across the 32-bit
 * counter wrap (~49 days at 1024 Hz).
 *
 * Periodic re-arm is DRIFT-FREE: next = previous deadline + period,
 * never now + period. If processing lags more than one period behind,
 * process() fires at most once per turn per event and the deadline
 * catches up over the following turns - the long-run tick count is
 * preserved.
 *
 * TimeEvent objects are static, owned by their AO, declared next to it;
 * arming links them into an intrusive list - no allocation, RAM = only
 * what is declared. MAIN-LOOP CONTEXT ONLY: arm/disarm/process all run
 * in the loop (an ISR that wants a delay posts an event to an AO, which
 * arms). This keeps the list free of critical sections by construction.
 */

#pragma once

#include <stdint.h>
#include <optional>

#include "kernel/platform.hpp"
#include "kernel/post.hpp"

namespace brio {

/// The armed-list manager for platform P (monostate).
template <Platform P>
class TimeEvents {
public:
    TimeEvents() = delete;

    /// Intrusive list node + firing glue; use TimeEvent<>, not this.
    class Base {
        friend class TimeEvents;

    protected:
        constexpr explicit Base(void (*fire)(Base&)) : fire_(fire) {}

    private:
        Base* next_ = nullptr;
        uint32_t deadline_ = 0;
        uint32_t period_ = 0;   // 0 = one-shot
        bool armed_ = false;
        void (*fire_)(Base&);
    };

    /// Arm: first firing after delay_ticks, then every period_ticks
    /// (0 = one-shot). Re-arming an armed event restarts it.
    static void arm(Base& te, uint32_t delay_ticks, uint32_t period_ticks) {
        disarm(te);
        te.deadline_ = P::now() + delay_ticks;
        te.period_ = period_ticks;
        te.armed_ = true;
        te.next_ = head_;
        head_ = &te;
    }

    static void disarm(Base& te) {
        for (Base** link = &head_; *link != nullptr; link = &(*link)->next_) {
            if (*link == &te) {
                *link = te.next_;
                break;
            }
        }
        te.next_ = nullptr;
        te.armed_ = false;
    }

    static bool armed(const Base& te) { return te.armed_; }

    /// Called by the kernel loop once per turn: post every matured event.
    static void process() {
        const uint32_t now = P::now();
        Base** link = &head_;
        while (*link != nullptr) {
            Base& te = **link;
            if (static_cast<int32_t>(now - te.deadline_) >= 0) {
                if (te.period_ != 0) {
                    te.deadline_ += te.period_;   // drift-free
                    link = &te.next_;
                } else {
                    *link = te.next_;             // one-shot: unlink first,
                    te.next_ = nullptr;           // so fire may re-arm freely
                    te.armed_ = false;
                }
                te.fire_(te);
            } else {
                link = &te.next_;
            }
        }
    }

    /**
     * Ticks from now to the NEAREST armed deadline: 0 when one is
     * already mature, empty when nothing is armed.
     *
     * The same wrap-safe signed difference process() uses, read the
     * other way round - `(int32_t)(deadline - now)`, so a deadline
     * across the 32-bit wrap answers with its true distance and an
     * overdue one answers with a non-positive value that clamps to 0.
     *
     * Why it exists: a power manager must know how long the program is
     * ALLOWED to stop before it decides how deeply to stop (a deep mode
     * that costs more to leave than the wait it saves is a bad trade -
     * util/power.hpp). It is a QUESTION, not a scheduling decision: the
     * loop still fires the events, and calling this changes nothing.
     * Main-loop context only, like the rest of this class.
     */
    static std::optional<uint32_t> ticks_to_next() {
        const uint32_t now = P::now();
        std::optional<uint32_t> best;
        for (const Base* te = head_; te != nullptr; te = te->next_) {
            const int32_t left = static_cast<int32_t>(te->deadline_ - now);
            const uint32_t ticks = left > 0 ? static_cast<uint32_t>(left) : 0;
            if (!best.has_value() || ticks < *best) {
                best = ticks;
            }
        }
        return best;
    }

    /// Tests / diagnostics: drop every armed event.
    static void clear_all() {
        while (head_ != nullptr) {
            Base* te = head_;
            head_ = te->next_;
            te->next_ = nullptr;
            te->armed_ = false;
        }
    }

private:
    static inline Base* head_ = nullptr;
};

/**
 * A time event owned by AO Ao: when it matures, `payload` is posted to
 * Ao's queue. Declare it static next to the AO:
 *
 *   static inline brio::TimeEvent<P, BlinkerAo, Toggle> tick{Toggle{}};
 *   ...
 *   tick.arm_every(brio::ticks_from_ms<P>(500));
 */
template <Platform P, typename Ao, typename Ev>
class TimeEvent : public TimeEvents<P>::Base {
    using Base = typename TimeEvents<P>::Base;

public:
    constexpr explicit TimeEvent(const Ev& payload)
        : Base(&do_fire), payload_(payload) {}

    /// RAII: a dying TimeEvent unlinks itself - an armed event must never
    /// outlive its storage (intrusive list). Free on the target, where
    /// time events are static and never die.
    ~TimeEvent() { disarm(); }

    TimeEvent(const TimeEvent&) = delete;             // list node: no copies
    TimeEvent& operator=(const TimeEvent&) = delete;

    /// One-shot: fire once, delay_ticks from now.
    void arm(uint32_t delay_ticks) {
        TimeEvents<P>::arm(*this, delay_ticks, 0);
    }

    /// Periodic: first firing one period from now, then every period.
    void arm_every(uint32_t period_ticks) {
        TimeEvents<P>::arm(*this, period_ticks, period_ticks);
    }

    void disarm() { TimeEvents<P>::disarm(*this); }
    bool armed() const { return TimeEvents<P>::armed(*this); }

private:
    static void do_fire(Base& b) {
        post<Ao>(static_cast<TimeEvent&>(b).payload_);
    }

    Ev payload_;
};

} // namespace brio
