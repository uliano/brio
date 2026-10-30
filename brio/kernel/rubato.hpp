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
 * The ceiling. The platform keeps one byte, P::ceiling, 255 at the
 * loop's base. Only an object whose index is below the ceiling may start
 * a dispatch; every post() compares the receiver's index with it after
 * the copy and, when it is below, asks for the scheduler
 * (P::pend_schedule()), and every other post adds one to the platform's
 * count of quiet posts (quiet_posts<P>, kernel/platform.hpp). The
 * scheduler is activate(base), `base` being the ceiling the platform
 * found when it took the request and replaced with 0. It CHOOSES - a
 * scan from the top of the list for the first non-empty queue below
 * `base`, with the ceiling at 0, so that a post landing during the scan
 * asks for nothing and only moves the count - then raises the ceiling to
 * the chosen object's index, serves one event of it, and chooses again;
 * between two dispatches the ceiling stays at the index just served. A
 * choice during which the count moved looks once more, above the chosen
 * object and with the ceiling already raised to it: a post the scan had
 * passed is served first, and a post landing after the raise asks for
 * the scheduler when it is above the chosen object and cannot beat it
 * otherwise. When a choice finds nothing, activate() returns with the
 * ceiling still at 0 and the count it read, and the platform puts `base`
 * back and compares the count in one step no interrupt handler can
 * enter, starting the scheduler again when a quiet post landed behind the
 * last scan (kernel/platform.hpp, the entry and the exit).
 *
 * The stack. A run of the scheduler that preempts a dispatch starts at
 * that dispatch's index, strictly below the ceiling of the run it
 * preempts: every run in progress but the topmost is suspended in a
 * dispatch, those dispatches have strictly decreasing indices, and when
 * one of them is index 0's nothing can stand above it, since no post
 * asks at a ceiling of 0. So a pack of n objects holds at most n runs,
 * whatever the interrupts' timing (test/test_rubato derives and checks
 * it). One run starts otherwise, on a core where an interrupt handler can
 * run between the request being taken and the entry's store of 0 (the
 * Cortex-M and the QingKe V4: cortexm/schedule.hpp, qingke/schedule.hpp):
 * a request that handler made is taken right after the store, as a run
 * at a ceiling of 0 that serves nothing and returns. It stands over the
 * entry of the run it delays, which has not begun choosing, so the count
 * of runs stays within n; what it adds to the stack is one entry's frame
 * and an empty choice, in place of the delayed run's dispatch.
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
 * the scheduler when no interrupt handler is running - at once when
 * called from a dispatch, at the handler's exit when called from one -
 * through an entry and an exit that keep kernel/platform.hpp's two
 * steps. The kernel never names the vector a target uses for it.
 *
 * Host tests drive init_all() and the host platform's entry directly,
 * since run() never returns (test/test_rubato, over host/platform.hpp's
 * RubatoHostPlatform).
 */

#pragma once

#include <stdint.h>

#include <atomic>
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

    /// The platform the kernel runs on: what a platform's scheduler body,
    /// written against the kernel type it serves, reads the ceiling of
    /// (kernel/platform.hpp, the entry and the exit).
    using Platform = P;

    /// Give every queue its object's list index, then call every active
    /// object's init(), in list order, with the ceiling held at 0 so that
    /// nothing is dispatched before the last init() has run. run() does
    /// this; a host test calls it by hand.
    static void init_all() {
        (Aos::queue.set_priority(index_of<Aos>()), ...);
        const uint8_t ceiling = P::ceiling;
        P::ceiling = 0;
        std::atomic_signal_fence(std::memory_order_seq_cst);   // the hold before the inits (look() says why)
        (Aos::init(), ...);
        std::atomic_signal_fence(std::memory_order_seq_cst);
        P::ceiling = ceiling;
    }

    /// The scheduler, called by the platform's entry (kernel/platform.hpp)
    /// with the ceiling at 0, `base` being the ceiling the entry replaced:
    /// choose the highest non-empty queue whose index is below `base`,
    /// raise the ceiling to it, serve one event, and choose again, until a
    /// choice finds nothing (the file header). Returns with the ceiling at
    /// 0, handing back the quiet-post count its last choice read and
    /// whether it served; the caller puts `base` back and compares the
    /// count. Re-entrant: a run started from a post() inside a dispatch, or
    /// at an interrupt's exit, starts at that dispatch's index and serves
    /// only objects above it. Masks nothing and never waits.
    static Activation activate(uint8_t base) {
        bool served = false;
        for (;;) {
            // The ceiling is 0 here - the entry's store, or the store that
            // ends the loop below. The count is read after that store and
            // before the first look, and the fences keep both orders: a
            // post the scan could miss lands after the read, and moves it.
            std::atomic_signal_fence(std::memory_order_seq_cst);
            const uint8_t count = load_count();
            std::atomic_signal_fence(std::memory_order_seq_cst);
            // One choice: a first scan at the ceiling 0, which serves the
            // object it finds unless the count moved by the time the
            // ceiling is raised to it; then a second scan above that object,
            // the ceiling held there, which serves what it finds; and when
            // the second finds nothing, a third that walks down to the held
            // object itself and serves it.
            uint8_t above = base;
            uint8_t held = no_hold;
            for (;;) {
                Found found = Found::nothing;
                (void)(look<Aos>(above, held, count, found) || ...);
                if (found == Found::moved) {
                    held = above;          // the second scan: above the held object
                    continue;
                }
                if (found != Found::nothing) {
                    served = served || found == Found::served;
                    break;                 // a dispatch ran (or its queue was found emptied)
                }
                if (held == no_hold) {
                    point(SchedulerPoint::exit, base);
                    return Activation{count, served};
                }
                if (above > held) {
                    break;                 // the third scan found nothing: choose again
                }
                above = static_cast<uint8_t>(held + 1u);   // the third scan
            }
            std::atomic_signal_fence(std::memory_order_seq_cst);
            point(SchedulerPoint::gap, P::ceiling);
            P::ceiling = 0;
        }
    }

    /// Put the base's ceiling, 255, back and sleep if, re-checked with
    /// interrupts masked, nothing is pending: Tenuto's idle_if_empty() with
    /// the base's exit in it. Called at the loop's base only, nothing
    /// suspended, after a run that served nothing: the ceiling is then 0,
    /// and the store of 255 and the check share the mask, so a post that
    /// landed after the run's last scan is found in its queue and the loop
    /// turns again instead of sleeping.
    static void idle_if_empty() {
        typename P::CriticalSection cs;
        P::ceiling = 255;
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
    /// set up. The time events are posted with the ceiling held at 0, as
    /// init_all() holds it: a fire is a post, and a post that could
    /// activate would serve the matured timers in the armed list's order
    /// instead of the list's; with the ceiling at 0 every fire only
    /// queues, nothing is dispatched during the walk (a post an interrupt
    /// handler makes meanwhile asks for nothing either, and waits for the
    /// walk's few cycles), and activate() then serves in priority order -
    /// the base turn is Tenuto's. The ceiling stays at 0 from the walk into the run, which
    /// is the entry activate() asks for, and a run that served turns the
    /// loop again with it still there; one that served nothing ends in
    /// idle_if_empty(), the base's exit.
    [[noreturn]] static void run() {
        init_all();
        for (;;) {
            P::ceiling = 0;
            std::atomic_signal_fence(std::memory_order_seq_cst);   // the hold before the walk (look() says why)
            TimeEvents<P>::process();
            std::atomic_signal_fence(std::memory_order_seq_cst);
            if (!activate(255).served) {
                idle_if_empty();
            }
        }
    }

private:
    template <typename Ao>
    static constexpr uint8_t index_of() {
        return static_cast<uint8_t>(Pack<Aos...>::template index<Ao>());
    }

    /// The count of quiet posts, read fresh: an interrupt handler moves it.
    [[gnu::always_inline]] static uint8_t load_count() {
        return *const_cast<const volatile uint8_t*>(&quiet_posts<P>);
    }

    /// The host's landing places (kernel/platform.hpp's scan_point());
    /// nothing on a target.
    [[gnu::always_inline]] static void point([[maybe_unused]] SchedulerPoint where,
                                             [[maybe_unused]] uint8_t index) {
        if constexpr (requires { P::scan_point(where, index); }) {
            P::scan_point(where, index);
        }
    }

    /// What a scan of one choice found (activate()).
    enum class Found : uint8_t {
        nothing,   ///< no object above the scan's limit holds an event
        moved,     ///< the first scan's object, raised to, with the count moved: scan again above it
        served,    ///< an event served
        emptied,   ///< the object raised to had been served to empty first (the second scan's case)
    };

    /// `held` when the ceiling is 0 and the count still to be checked.
    static constexpr uint8_t no_hold = 255;

    /// One object of a scan: true ends the scan - the object is at or
    /// below `above`, and so is every one after it; or its queue holds an
    /// event. Then the ceiling is raised to it, and in a first scan (`held`
    /// is no_hold) the count read again: when it moved, a post may have
    /// landed above this object after its look, and the scan ends with
    /// `above` lowered to this object for a second scan. Otherwise its
    /// event is served here, inside the look, so the scan, the raise and
    /// the dispatch are one path with no choice by index between them.
    /// Forced inline: the choice holds its indices in registers.
    template <typename Ao>
    [[gnu::always_inline]] static bool look(uint8_t& above, uint8_t held, uint8_t count,
                                            Found& found) {
        constexpr uint8_t index = index_of<Ao>();
        if (index >= above) {
            return true;
        }
        point(SchedulerPoint::look, index);
        if (Ao::queue.empty()) {
            point(SchedulerPoint::empty, index);
            return false;
        }
        point(SchedulerPoint::found, index);
        P::ceiling = index;
        // The raise must reach memory before the count is read again and
        // the queue's cursors are read, and the next store of the ceiling
        // must follow the slot's release: nothing in the language orders a
        // plain store against those loads and stores, and GCC 16 at -Os
        // moved take()'s two loads ahead of the raise (measured on the
        // STM32F446RE: an interrupt's request landing in that gap served
        // the event in a nested run, the outer one took it again from its
        // stale cursor and released the slot twice, which wedges the queue
        // for good). The fences are compiler barriers alone - one core
        // sees its own stores in order - and cost no instruction.
        std::atomic_signal_fence(std::memory_order_seq_cst);
        point(SchedulerPoint::raised, index);
        if (held == no_hold && load_count() != count) {
            // A post landed during the scan, maybe above this object after
            // its look. Look again above it with the ceiling held here: a
            // post to an object above it now asks for a run of its own,
            // which starts below this one, and a post to one at or below it
            // cannot beat this choice - so the second scan needs no count.
            above = index;
            found = Found::moved;
            return true;
        }
        // False only in a second or third scan: a run an interrupt's
        // request started between this look and the raise served the queue
        // to empty. A first scan's object is served by nobody else first -
        // the ceiling is 0 until its raise, and its own index after - so it
        // always holds the event the look saw.
        found = serve_one<Ao, P>(index) ? Found::served : Found::emptied;
        return true;
    }
};

} // namespace brio
