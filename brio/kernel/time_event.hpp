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
 * kernel loop, by TimeEvents<P>::process(), which Tenuto (the kernel,
 * kernel/tenuto.hpp) calls once per turn of its loop: it compares
 * P::now() with every armed deadline and posts the matured payloads, in
 * main context. The conversion from milliseconds in the example is
 * kernel/time.hpp's.
 *
 * Context: arm(), arm_every(), disarm() and process() run in the kernel
 * loop only, never in an interrupt handler. An interrupt handler that
 * wants a delay posts an event to an active object, whose handler arms
 * the timer. This is what keeps the armed list free of critical
 * sections.
 *
 * On a preemptive platform (kernel/platform.hpp, the Rubato kernel's) a
 * dispatch can start inside another, so "the kernel loop" is several
 * dispatches on one stack, and any of them may arm or disarm while
 * another is walking the list. There arm(), disarm(), clear_all(),
 * process() and the two questions take P::CriticalSection around their
 * list operations, the pointer reads and writes: arm() exactly one,
 * disarm() one, a question one. process() walks the list under the
 * guard and fires each matured payload with the guard dropped, because
 * a fire is a post and a post can run a dispatch to its end, which must
 * not run masked; after the fire it takes the guard again and reads
 * the walk's place afresh, because the dispatch may have disarmed or
 * re-armed anything. The walk's place is a pointer to the link it stands
 * on, and an unlink of the node that owns that link moves the place to
 * the node's predecessor, so the walk resumes at the node that follows:
 * no node is skipped, none is visited twice, a disarmed one is never
 * fired, and one armed during the walk is first looked at on the next
 * turn. The interrupt handler's rule above does not change.
 *
 * When a timer matures. arm() stores the deadline as P::now() plus the
 * delay, and P::now() is a whole number of ticks: the fraction of the
 * current tick already elapsed at the arming instant is not counted.
 * process() posts the payload on the first turn at which now() has
 * reached the deadline. So a delay of N ticks matures between N - 1 and
 * N tick intervals of physical time after arming (a one-tick delay
 * armed just before the next tick matures almost at once), and the
 * event is handled when the loop reaches process() after that and then
 * serves the queue, which can be later still. kernel/time.hpp's
 * conversion rounds up to whole ticks, which protects the count from
 * truncation and nothing more; a program that needs a minimum physical
 * wait arms one tick more.
 *
 * A periodic timer re-arms from its previous deadline, not from the
 * moment the loop noticed it, so the period does not drift. If the loop
 * falls more than a period behind, process() posts at most one event per
 * turn per timer and the deadline advances one period per turn, so each
 * missed period is posted on a later turn and the sequence of deadlines
 * is kept. What is delivered is the owner's queue's business: every post
 * has post()'s rules, and a queue that fills during the catch-up drops
 * and counts. Re-arming a running timer restarts it: the next expiry is
 * one delay from the re-arm.
 *
 * Limits of the deadline arithmetic. process(), ticks_to_next() and
 * next_deadline() all measure a deadline's distance from now() as a
 * signed 32-bit difference, which survives the counter's wrap (about
 * 49 days at 1000 Hz) as long as every distance stays strictly within
 * half the range: a delay or a period must be less than 2^31 ticks
 * (about 24 days at 1000 Hz), and a deadline that the loop leaves
 * unserved for 2^31 ticks or more reads as future again. Exactly 2^31
 * is not a supported distance: process() does not post it yet, while
 * ticks_to_next() already reports it due and next_deadline() prefers it
 * to a nearer deadline, because the two queries take the difference the
 * other way round. A delay above 2^31 is not refused: it reads as
 * overdue and is posted at once.
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
#include <type_traits>

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
    /// (0 = one-shot); both less than 2^31 (the file header's limits).
    /// Re-arming an armed event restarts it.
    static void arm(Base& te, uint32_t delay_ticks, uint32_t period_ticks) {
        if constexpr (Preemptive<P>) {
            typename P::CriticalSection cs;   // the one guard: unlink and link in it
            unlink(te);
            te.deadline_ = P::now() + delay_ticks;
            te.period_ = period_ticks;
            te.armed_ = true;
            te.next_ = head_;
            head_ = &te;
            if (walk_ == &head_) {
                walk_ = &te.next_;            // behind the walk: looked at next turn
            }
        } else {
            disarm(te);
            te.deadline_ = P::now() + delay_ticks;
            te.period_ = period_ticks;
            te.armed_ = true;
            te.next_ = head_;
            head_ = &te;
        }
    }

    static void disarm(Base& te) {
        if constexpr (Preemptive<P>) {
            typename P::CriticalSection cs;
            unlink(te);
        } else {
            for (Base** link = &head_; *link != nullptr; link = &(*link)->next_) {
                if (*link == &te) {
                    *link = te.next_;
                    break;
                }
            }
            te.next_ = nullptr;
            te.armed_ = false;
        }
    }

    static bool armed(const Base& te) { return te.armed_; }

    /// Called by the kernel loop once per turn: post every matured event.
    static void process() {
        if constexpr (Preemptive<P>) {
            process_guarded();
        } else {
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
        [[maybe_unused]] const Guard guard{};
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
        [[maybe_unused]] const Guard guard{};
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
        [[maybe_unused]] const Guard guard{};
        while (head_ != nullptr) {
            Base* te = head_;
            head_ = te->next_;
            te->next_ = nullptr;
            te->armed_ = false;
        }
    }

private:
    static inline Base* head_ = nullptr;

    // ---- the preemptive platform's list (the file header) ------------------

    /// The guard a question and clear_all() take: the platform's critical
    /// section on a preemptive platform, nothing - an empty object - on
    /// every other.
    struct NoGuard {};
    using Guard = std::conditional_t<Preemptive<P>, typename P::CriticalSection, NoGuard>;

    /// Where process() stands: the link it reads next (&head_, or the
    /// next_ of the node it last passed); null when no walk is under way.
    /// Written under the guard only, by process(), and by an unlink or an
    /// arm that moves the node owning it.
    static inline Base** walk_ = nullptr;

    /// Take te out of the list, under the caller's guard. When te owns the
    /// link the walk stands on, the walk moves to the link that pointed at
    /// te, which now points at te's successor: nothing skipped.
    static void unlink(Base& te) {
        for (Base** link = &head_; *link != nullptr; link = &(*link)->next_) {
            if (*link == &te) {
                *link = te.next_;
                if (walk_ == &te.next_) {
                    walk_ = link;
                }
                break;
            }
        }
        te.next_ = nullptr;
        te.armed_ = false;
    }

    /// From the walk's place, the next matured node, with its list update
    /// made (a periodic one's deadline advanced and the walk moved past
    /// it, a one-shot unlinked and disarmed); null at the end of the list,
    /// the walk over. Under the caller's guard.
    static Base* next_due(uint32_t now) {
        while (*walk_ != nullptr) {
            Base& te = **walk_;
            if (static_cast<int32_t>(now - te.deadline_) >= 0) {
                if (te.period_ != 0) {
                    te.deadline_ += te.period_;   // drift-free
                    walk_ = &te.next_;
                } else {
                    *walk_ = te.next_;            // one-shot: unlinked before the fire
                    te.next_ = nullptr;
                    te.armed_ = false;
                }
                return &te;
            }
            walk_ = &te.next_;
        }
        walk_ = nullptr;
        return nullptr;
    }

    /// process() on a preemptive platform: the list read and written under
    /// the guard, each fire with the guard dropped, the walk's place read
    /// again under the guard after it. One guard per process() and one
    /// more per fire.
    static void process_guarded() {
        const uint32_t now = P::now();
        Base* due;
        {
            typename P::CriticalSection cs;
            walk_ = &head_;
            due = next_due(now);
        }
        while (due != nullptr) {
            due->fire_(*due);                     // unmasked: may run dispatches to their end
            if constexpr (requires { P::preempt_point(); }) {
                P::preempt_point();               // a host's: where an interrupt could land
            }
            typename P::CriticalSection cs;
            due = next_due(now);
        }
    }
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

    /// One-shot: post the payload once, delay_ticks from now (between
    /// delay_ticks - 1 and delay_ticks tick intervals of physical time,
    /// the file header's "When a timer matures"). Re-arming a running
    /// event restarts it.
    void arm(uint32_t delay_ticks) {
        TimeEvents<P>::arm(*this, delay_ticks, 0);
    }

    /// Periodic: the first post one period from now, then one every
    /// period, drift-free; a period of zero is a one-shot. Re-arming a
    /// running event restarts it.
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
