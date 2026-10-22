/*
 * tenuto.hpp
 *
 * Tenuto, the brio kernel: the loop that serves the active objects of a
 * program, one event at a time, in priority order.
 *
 * An application starts it once, at the end of main(), with the platform
 * and its active objects listed from the highest priority to the lowest:
 *
 *     brio::Tenuto<P, Blinker, Supervisor>::run();   // never returns
 *
 * The list is the priority order; there is no separate priority table.
 * The compiler generates the scan over the list, so an object not in it
 * never runs, and an object that does not meet the ActiveObject contract
 * (kernel/active_object.hpp) is refused at this line with the name of
 * the requirement.
 *
 * One turn of the loop:
 *  1. post the time events that have matured (kernel/time_event.hpp);
 *  2. take ONE event from the highest-priority non-empty queue and
 *     dispatch it, to completion, on the main stack; then release its
 *     queue slot unless the object held it (serve_one() in
 *     kernel/active_object.hpp);
 *  3. if every queue was empty, check again with interrupts masked and,
 *     if still empty, sleep until the next interrupt.
 *
 * The rule the whole model rests on: a dispatch runs to its end before
 * any other active object runs. Priority decides what is served next,
 * never whether the running dispatch is interrupted: on the next turn
 * the loop rescans the list from the top and takes the oldest event of
 * the first non-empty queue. An event posted during a dispatch therefore
 * waits at least until that dispatch returns, and longer when an older
 * event is ahead of it in the same queue or a queue of higher priority
 * is not empty. Interrupt handlers do interrupt a dispatch; what they
 * may do with the kernel is post() an event. The name is the musical
 * marking for a note held for its full value, which is run-to-completion
 * said in one word.
 *
 * The scan order is a selection precedence, not a share of the
 * processor. The kernel promises no response time beyond "the running
 * dispatch finishes first" and no fairness: a long dispatch delays every
 * other object by its length, and an object whose higher-priority peers
 * always have an event waiting is never served. The kernel does not age
 * priorities; such a design shows in the per-queue overflow counters,
 * and keeping every dispatch short is the application's side of the
 * model.
 *
 * Sleeping without losing a wake-up: the platform's idle() is entered
 * with interrupts masked and must sleep in a way that an interrupt
 * arriving between the empty check and the sleep instruction still wakes
 * the core (kernel/platform.hpp says how each core keeps that promise).
 * A platform whose timebase keeps counting while the core sleeps may
 * offer idle_until(deadline) instead: the loop then hands it the absolute
 * tick of the nearest armed time event (TimeEvents::next_deadline(),
 * empty when nothing is armed), so the platform can place its wake there
 * and sleep through the whole wait with no periodic tick. The hook is
 * detected with a requires expression; a platform without it compiles
 * the plain idle() path alone.
 *
 * The list also carries one ordering fact the payload rules need
 * (kernel/borrowed.hpp): an object that lends Lease::dispatch payloads
 * declares its borrowers with `using LendsTo = Subscribers<...>`, and
 * this header checks at compile time that every borrower precedes the
 * lender in the list.
 *
 * Host tests drive init_all() and step() directly, since run() never
 * returns.
 */

#pragma once

#include <stddef.h>

#include <optional>

#include "kernel/active_object.hpp"
#include "kernel/platform.hpp"
#include "kernel/post.hpp"
#include "kernel/time_event.hpp"

namespace brio {

/// The pack as a type, so ordering questions can be asked of it.
template <typename... Aos>
struct Pack {
    /// Position of Ao in the pack (= its priority, 0 = highest);
    /// sizeof...(Aos) when absent.
    template <typename Ao>
    static constexpr size_t index() {
        size_t i = 0;
        size_t found = sizeof...(Aos);
        (void)((std::same_as<Ao, Aos> ? (found = i, true) : (++i, false)) || ...);
        return found;
    }

    /// True when every listed borrower precedes Lender in the pack: the
    /// ordering a Lease::dispatch loan relies on (kernel/borrowed.hpp).
    template <typename Lender, typename... Borrowers>
    static constexpr bool borrowers_precede(Subscribers<Borrowers...>) {
        return ((index<Borrowers>() < index<Lender>()) && ...);
    }

    /// An AO lending Lease::dispatch payloads names its borrowers with
    /// `using LendsTo = Subscribers<B1, B2, ...>` (kernel/borrowed.hpp);
    /// AOs without LendsTo trivially pass.
    template <typename Ao>
    static constexpr bool lends_ok() {
        if constexpr (requires { typename Ao::LendsTo; }) {
            return borrowers_precede<Ao>(typename Ao::LendsTo{});
        } else {
            return true;
        }
    }
};

template <Platform P, ActiveObject... Aos>
    requires (sizeof...(Aos) > 0)
class Tenuto {
    static_assert((Pack<Aos...>::template lends_ok<Aos>() && ...),
                  "a Lease::dispatch borrower must precede its lender in "
                  "the Tenuto pack (see kernel/borrowed.hpp)");
    static_assert((queue_on<Aos, P>() && ...),
                  "every AO in a Tenuto's pack lives on the kernel's platform: its "
                  "queue is guarded by that platform's critical section, and on a "
                  "chip with two cores the platform IS the core (kernel/active_object.hpp)");

public:
    Tenuto() = delete;

    /// Call every active object's init(), in list order, before the first
    /// event is served. run() does this; a host test calls it by hand.
    static void init_all() {
        (Aos::init(), ...);
    }

    /// Serve at most one event: the oldest in the highest-priority
    /// non-empty queue. Returns false when every queue was empty at the
    /// moment it looked. Main-loop context; a host test's turn of the crank.
    static bool step() {
        return (try_one<Aos>() || ...);
    }

    /// Sleep if, re-checked with interrupts masked, nothing is pending.
    /// Returns after the next interrupt (the platform's idle() unmasks
    /// and sleeps without a lost-wakeup window). A platform with
    /// idle_until() is told how far it may sleep: the nearest armed
    /// deadline, read here under the same mask (the armed list is
    /// main-loop state, and a masked read of now() is the timebase's
    /// contract). Without it, plain idle(): the other branch is not
    /// compiled at all.
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
    /// one dispatch, sleep when idle. Call once, with interrupts enabled
    /// and the clock and timebase set up.
    [[noreturn]] static void run() {
        init_all();
        for (;;) {
            TimeEvents<P>::process();
            if (!step()) {
                idle_if_empty();
            }
        }
    }

private:
    template <typename Ao>
    static bool try_one() {
        return serve_one<Ao>();
    }
};

} // namespace brio
