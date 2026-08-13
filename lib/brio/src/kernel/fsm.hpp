/*
 * fsm.hpp
 *
 * The state-machine contract of brio active objects. HSM-READY, FLAT
 * IMPLEMENTATION: the contract is fixed so that hierarchy can arrive
 * later as an additive extension (unhandled IS the future
 * bubble-to-parent hook), but today a state is one flat handler.
 *
 * Model:
 *  - a state is a function `Status handler(const Event&)`; the current
 *    state of the machine IS a pointer to it (2 bytes on AVR);
 *  - Event is the AO's own std::variant, with the kernel's reserved
 *    alternatives Entry and Exit prepended by this template: the handler
 *    receives them through the same visit as ordinary events. They are
 *    empty structs, so they do not enlarge the queue slots; they are
 *    delivered synchronously by the transition machinery and are never
 *    meant to be posted (the kernel post layer excludes them);
 *  - a handler returns handled(), unhandled() (today: ignore; tomorrow:
 *    ask the parent - the HSM hook), or transition(&next_state);
 *  - a transition delivers Exit to the old state, switches, delivers
 *    Entry to the new one. Entry may itself return transition() - the
 *    machinery follows the chain, giving pass-through states for free.
 *    The result of Exit is deliberately ignored: exit is an action, not
 *    a decision point. An entry chain that never settles is a
 *    programming error (it spins forever - keep entries simple);
 *  - start(initial) arms the machine and delivers the first Entry, so
 *    no state ever runs half-initialized. The kernel calls it for every
 *    AO before the event loop (the "kernel-delivered init" decision).
 *
 * Monostate like everything in brio: the current-state pointer is a
 * static inline per (Derived, alternatives...) instantiation - Derived
 * exists exactly so two AOs with the same event alternatives get
 * distinct machines. Trivial AOs may legally ignore transition() and
 * keep a plain switch inside one everlasting state.
 *
 * Pure kernel code: includes nothing of brio, no hardware, host-testable
 * (see test/test_fsm).
 */

#pragma once

#include <stdint.h>
#include <variant>

namespace brio {

struct Entry {};  ///< reserved event: delivered when a state is entered
struct Exit {};   ///< reserved event: delivered when a state is left

template <typename Derived, typename... Alts>
class Fsm {
public:
    Fsm() = delete;  // monostate: no instances

    /// The AO's event type: kernel reserved alternatives first.
    using Event = std::variant<Entry, Exit, Alts...>;

    struct Status;
    using Handler = Status (*)(const Event&);

    /// Outcome of a state handler; build via handled()/unhandled()/transition().
    struct Status {
        enum class Kind : uint8_t { handled, unhandled, transition };
        Kind kind;
        Handler target;
    };

    static constexpr Status handled() {
        return {Status::Kind::handled, nullptr};
    }
    static constexpr Status unhandled() {
        return {Status::Kind::unhandled, nullptr};
    }
    static constexpr Status transition(Handler next) {
        return {Status::Kind::transition, next};
    }

    /// Arm the machine on its initial state and deliver the first Entry.
    static void start(Handler initial) {
        state_ = initial;
        follow(state_(Event{Entry{}}));
    }

    /// Deliver one event to the current state, run-to-completion.
    static void dispatch(const Event& e) {
        follow(state_(e));
    }

    /// Current state (for tests and diagnostics).
    static Handler current() { return state_; }

private:
    static void follow(Status s) {
        while (s.kind == Status::Kind::transition) {
            (void)state_(Event{Exit{}});     // exit is an action, not a decision
            state_ = s.target;
            s = state_(Event{Entry{}});      // entry may chain (pass-through)
        }
        // handled: done. unhandled: ignored today, HSM bubble hook tomorrow.
    }

    static inline Handler state_ = nullptr;
};

} // namespace brio
