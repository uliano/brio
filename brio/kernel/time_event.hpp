/*
 * time_event.hpp
 *
 * Time events: timers that post an event to their owner when they
 * expire. The replacement for a delay loop inside a handler.
 *
 * An active object declares a TimeEvent next to itself, with the event
 * it will receive as the payload, and arms it once (arm, a delay) or
 * periodically (arm_every):
 *
 *     static inline brio::TimeEvent<P, Blinker, Toggle> heartbeat{Toggle{}};
 *     ...
 *     heartbeat.arm_every(brio::ticks_from_ms<P>(500));
 *
 * When the timer expires, Toggle{} is posted to Blinker's queue and
 * handled in Blinker's dispatch like any other event. No user code runs
 * when the tick interrupt fires: the interrupt only advances the tick
 * counter (and wakes the core from idle). Expiry is detected in the
 * kernel loop, by TimeEvents<P>::process(), which Tenuto calls once per
 * turn: it compares P::now() with every armed deadline and posts the
 * matured payloads, in main context.
 *
 * Context: arm(), arm_every(), disarm() and process() run in the kernel
 * loop only, never in an interrupt handler. An interrupt handler that
 * wants a delay posts an event to an active object, whose handler arms
 * the timer. This is what keeps the armed list free of critical
 * sections.
 *
 * A periodic timer re-arms from its previous deadline, not from the
 * moment the loop noticed it, so the period does not drift. If the loop
 * falls more than a period behind, process() posts at most one event per
 * turn per timer and the deadline catches up over the following turns;
 * the long-run count of events is preserved. Re-arming a running timer
 * restarts it: the next expiry is one delay from the re-arm.
 *
 * Deadline comparison is wrap-safe: (int32_t)(now - deadline) >= 0 holds
 * across the 32-bit counter's wrap (about 49 days at 1000 Hz).
 *
 * Storage: a TimeEvent is a static object owned by its active object;
 * arming links it into an intrusive list, so there is no allocation and
 * the RAM is what is declared. A TimeEvent must not be destroyed while
 * armed (the destructor disarms it; on a target they are static and
 * never die).
 */

#pragma once

#include <stdint.h>
#include <optional>

#include "kernel/active_object.hpp"
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

    /**
     * The NEAREST armed deadline itself - the absolute tick, not the
     * distance - or empty when nothing is armed. An overdue deadline is
     * still the nearest (its signed distance is the smallest).
     *
     * ticks_to_next()'s sibling, for the other consumer: the loop hands
     * this to a platform that can sleep TO an instant rather than FOR a
     * span (tenuto.hpp's idle_if_empty, kernel/platform.hpp's optional
     * idle_until). The distance would drift by up to one tick between
     * this read of now() and the platform's own; the deadline does not.
     * A question, not a decision; main-loop context only.
     */
    static std::optional<uint32_t> next_deadline() {
        const uint32_t now = P::now();
        std::optional<uint32_t> best;
        int32_t best_left = 0;
        for (const Base* te = head_; te != nullptr; te = te->next_) {
            const int32_t left = static_cast<int32_t>(te->deadline_ - now);
            if (!best.has_value() || left < best_left) {
                best = te->deadline_;
                best_left = left;
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
 * A time event owned by the active object Ao: when it matures, `payload`
 * is posted to Ao's queue (with post()'s rules: a full queue drops and
 * counts). Declare it static next to the object:
 *
 *   static inline brio::TimeEvent<P, Blinker, Toggle> tick{Toggle{}};
 *   ...
 *   tick.arm_every(brio::ticks_from_ms<P>(500));
 *
 * Every member runs in the kernel loop only (the file header).
 */
template <Platform P, typename Ao, typename Ev>
class TimeEvent : public TimeEvents<P>::Base {
    using Base = typename TimeEvents<P>::Base;
    static_assert(queue_on<Ao, P>(),
                  "a TimeEvent posts to an AO of its own platform: on a chip with two "
                  "cores a timer of one core cannot feed an AO of the other "
                  "(kernel/active_object.hpp)");

public:
    constexpr explicit TimeEvent(const Ev& payload)
        : Base(&do_fire), payload_(payload) {}

    /// RAII: a dying TimeEvent unlinks itself - an armed event must never
    /// outlive its storage (intrusive list). Free on the target, where
    /// time events are static and never die.
    ~TimeEvent() { disarm(); }

    TimeEvent(const TimeEvent&) = delete;             // list node: no copies
    TimeEvent& operator=(const TimeEvent&) = delete;

    /// One-shot: post the payload once, delay_ticks from now. Re-arming a
    /// running event restarts it.
    void arm(uint32_t delay_ticks) {
        TimeEvents<P>::arm(*this, delay_ticks, 0);
    }

    /// Periodic: the first post one period from now, then one every
    /// period, drift-free. Re-arming a running event restarts it.
    void arm_every(uint32_t period_ticks) {
        TimeEvents<P>::arm(*this, period_ticks, period_ticks);
    }

    /// Cancel: a disarmed event posts nothing. Harmless when not armed.
    void disarm() { TimeEvents<P>::disarm(*this); }
    /// Whether the event is armed (a one-shot disarms itself when it fires).
    bool armed() const { return TimeEvents<P>::armed(*this); }

private:
    static void do_fire(Base& b) {
        post<Ao>(static_cast<TimeEvent&>(b).payload_);
    }

    Ev payload_;
};

} // namespace brio
