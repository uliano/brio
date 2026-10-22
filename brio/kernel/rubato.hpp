/*
 * rubato.hpp
 *
 * Rubato, brio's preemptive kernel: the loop of Tenuto (kernel/tenuto.hpp)
 * with one thing added - the dispatch of an active object of higher
 * priority may start while a dispatch of lower priority is suspended,
 * on the same stack, and the suspended one resumes when the higher one
 * returns.
 *
 * An application starts it the way it starts Tenuto, once, at the end of
 * main(), on a platform that is preemptive (kernel/platform.hpp's
 * Preemptive), with its active objects from the highest priority to the
 * lowest:
 *
 *     brio::Rubato<P, Sensor, Logger>::run();   // never returns
 *
 * The same active objects, queues, state machines, post(), publish() and
 * ReplyTo, time events, loans and panic path; the list is the priority
 * order, the first the highest, and the same compile-time checks refuse
 * a Lease::dispatch borrower that follows its lender and an object whose
 * queue lives on another platform.
 *
 * The ceiling. The platform keeps one byte, P::ceiling: the list index
 * of the object being dispatched, 255 at the loop's base. Only an
 * object whose index is below the ceiling may start a dispatch; every
 * post() compares the receiver's index with it after the copy and, when
 * it is below, asks for the scheduler (P::pend_schedule()). The
 * scheduler is activate(): it serves the highest non-empty queue below
 * the ceiling, raising the ceiling to that object's index for the length
 * of the dispatch and restoring it after, and repeats from the top of
 * the list until no queue below the ceiling holds anything.
 *
 * Two roads lead to a dispatch that preempts another:
 *  1. an interrupt handler posts to an object above the ceiling: the
 *     request waits for the handler to end, and at its exit the platform
 *     runs activate(), so the higher object's dispatches run to their
 *     end over the code the interrupt suspended, and that code resumes
 *     after them. An interrupt handler is still never interrupted by a
 *     dispatch, and still runs to completion;
 *  2. a dispatch posts to an object above it: the platform runs
 *     activate() at once, inside post(), which returns after the
 *     receiver's dispatches have run to their end.
 * A post to an object at or below the ceiling, the poster itself
 * included, waits as under Tenuto: until no dispatch of its receiver's
 * priority or higher is running or suspended, and it is then served in
 * list order.
 *
 * What never changes. A handler is never re-entered, and an object is
 * never dispatched while one of its own dispatches is suspended, because
 * its index is never below a ceiling raised to it; so an object's own
 * statics need no lock, and its queue keeps one consumer. A dispatch
 * still runs to its end before any object of its own or lower priority
 * runs. Nothing blocks, nothing has a stack of its own, nothing is
 * sliced in time. Before every object's init() has run nothing is
 * dispatched: init_all() holds the ceiling at 0 while the inits run, so
 * an event posted from an init() waits for the loop, as under Tenuto;
 * and run() holds it at 0 while the matured time events are posted, so
 * the base turn serves them in priority order, as Tenuto does (run()'s
 * comment).
 *
 * The loans under preemption (kernel/borrowed.hpp). A Lease::dispatch
 * borrower precedes its lender in the list, so a loan posted from the
 * lender's dispatch starts the borrower's dispatch inside the post: the
 * borrower reads the storage, and may write it, before post() returns,
 * and its lease is over when post() returns. The lender may reuse the
 * storage from then on, within the same dispatch. What the lender may
 * not do is read the lent storage after the post expecting what it put
 * there: it reads what the borrower left, which under Tenuto it would
 * have read only on its next dispatch. Lease::reply and Lease::hold
 * are unchanged: a reply is an event, and a held slot is its holder's.
 *
 * Active objects share nothing but events (docs/design/kernel.md,
 * "Payloads"): under this kernel that is a rule of correctness, since
 * two objects of different priority run one inside the other. Two
 * structures any object may call directly guard themselves on a
 * preemptive platform: the armed time events (kernel/time_event.hpp)
 * and each producer verb of a Ring (util/ring.hpp).
 *
 * The accepted limits are Tenuto's, with preemption in place of "the
 * running dispatch finishes first": the kernel promises that an object
 * of higher priority is served before a lower one resumes, and nothing
 * more - no response time, no fairness, no ageing; a long dispatch
 * delays every object of its own and lower priority by its length.
 *
 * What the platform provides, beyond the Platform concept: `preemptive`
 * (true), `ceiling` (a uint8_t at 255) and `pend_schedule()`, which runs
 * activate() when no interrupt handler is running - at once when called
 * from a dispatch, at the handler's exit when called from one. The
 * kernel never names the vector a target uses for it.
 *
 * Host tests drive init_all() and activate() directly, since run() never
 * returns (test/test_rubato, over host/platform.hpp's RubatoHostPlatform).
 */

#pragma once

#include <stdint.h>

#include <optional>

#include "kernel/active_object.hpp"
#include "kernel/platform.hpp"
#include "kernel/post.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time_event.hpp"

namespace brio {

template <Platform P, ActiveObject... Aos>
    requires (sizeof...(Aos) > 0) && Preemptive<P>
class Rubato {
    static_assert(sizeof...(Aos) < 255,
                  "a list index is a byte and 255 is the ceiling at the loop's base");
    static_assert((Pack<Aos...>::template lends_ok<Aos>() && ...),
                  "a Lease::dispatch borrower must precede its lender in "
                  "the Rubato pack (see kernel/borrowed.hpp)");
    static_assert((queue_on<Aos, P>() && ...),
                  "every AO in a Rubato's pack lives on the kernel's platform: its "
                  "queue is guarded by that platform's critical section, and on a "
                  "chip with two cores the platform IS the core (kernel/active_object.hpp)");

public:
    Rubato() = delete;

    /// Give every queue its object's list index, then call every active
    /// object's init(), in list order, with the ceiling held at 0 so that
    /// nothing is dispatched before the last init() has run. run() does
    /// this; a host test calls it by hand.
    static void init_all() {
        (Aos::queue.set_priority(index_of<Aos>()), ...);
        const uint8_t ceiling = P::ceiling;
        P::ceiling = 0;
        (Aos::init(), ...);
        P::ceiling = ceiling;
    }

    /// The scheduler: serve the highest non-empty queue whose index is
    /// below the ceiling, one event, with the ceiling raised to that index
    /// for the dispatch and restored after it, and repeat from the top of
    /// the list until every queue below the ceiling is empty. Re-entrant:
    /// a call from a post() inside a dispatch, or from an interrupt's exit,
    /// sees the raised ceiling and serves only objects above it. Returns
    /// whether anything was served.
    static bool activate() {
        bool served = false;
        while (serve_highest()) {
            served = true;
        }
        return served;
    }

    /// Sleep if, re-checked with interrupts masked, nothing is pending:
    /// Tenuto's idle_if_empty(). Called at the loop's base only, where the
    /// ceiling is 255 and nothing is suspended.
    static void idle_if_empty() {
        typename P::CriticalSection cs;
        if ((Aos::queue.empty() && ...)) {
            if constexpr (requires {
                              { P::idle_until(std::optional<uint32_t>{}) } -> std::same_as<void>;
                          }) {
                P::idle_until(TimeEvents<P>::next_deadline());
            } else {
                P::idle();   // re-enables interrupts, then sleeps: race-free
            }
        }
    }

    /// Initialize every active object, then loop forever: time events,
    /// the scheduler, sleep when idle. Call once, with interrupts enabled
    /// and the clock, the timebase and the platform's scheduling request
    /// set up. The time events are posted with the ceiling held at 0,
    /// as init_all() holds it: a fire is a post, and a post that could
    /// activate would serve the matured timers in the armed list's order
    /// instead of the list's; with the ceiling at 0 every fire only
    /// queues, nothing is dispatched during the walk (a request an
    /// interrupt handler makes meanwhile waits for the walk's few
    /// cycles), and activate() then serves in priority order - the base
    /// turn is Tenuto's.
    [[noreturn]] static void run() {
        init_all();
        for (;;) {
            P::ceiling = 0;
            TimeEvents<P>::process();
            P::ceiling = 255;
            if (!activate()) {
                idle_if_empty();
            }
        }
    }

private:
    template <typename Ao>
    static constexpr uint8_t index_of() {
        return static_cast<uint8_t>(Pack<Aos...>::template index<Ao>());
    }

    /// One pass over the list from the top: the first object at or below
    /// the ceiling ends it, as does the first one served. True when one
    /// event was served.
    static bool serve_highest() {
        const uint8_t ceiling = P::ceiling;
        bool served = false;
        (void)(visit<Aos>(ceiling, served) || ...);
        return served;
    }

    /// One object of the pass: true ends the pass (the object is at or
    /// below the ceiling, so is every one after it; or it was served).
    template <typename Ao>
    static bool visit(uint8_t ceiling, bool& served) {
        constexpr uint8_t index = index_of<Ao>();
        if (index >= ceiling) {
            return true;
        }
        if (Ao::queue.empty()) {
            return false;
        }
        P::ceiling = index;
        // False only when an interrupt's exit, between the look above and
        // the raise of the ceiling, served this queue to empty: the pass
        // goes on.
        served = serve_one<Ao>();
        P::ceiling = ceiling;
        return served;
    }
};

} // namespace brio
