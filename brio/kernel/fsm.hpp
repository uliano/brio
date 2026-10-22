/*
 * fsm.hpp
 *
 * The state machine of a brio active object, and `match`, the function
 * a handler dispatches an event with.
 *
 * An active object derives from Fsm<Derived, Alts...>, naming itself and
 * the events it accepts:
 *
 *     struct Blinker : brio::Fsm<Blinker, Toggle, SetPeriod> { ... };
 *
 * From that the template defines the object's `Event` type (a
 * std::variant of the listed events, with the two reserved events Entry
 * and Exit in front), its `dispatch()`, and the vocabulary of a state
 * handler: handled(), unhandled(), transition(&next), start(&initial).
 * Why Derived names itself: the current-state pointer is a static member
 * of this template, so two objects accepting the same events get two
 * distinct state machines only because their types differ. The base
 * never calls anything on Derived.
 *
 * The model, flat:
 *  - a state is a function `Status handler(const Event&)`, and the
 *    current state of the machine is a pointer to it (one function
 *    pointer of RAM, two bytes on an 8-bit core);
 *  - a handler returns handled() (done), unhandled() (not for this
 *    state; ignored today) or transition(&next_state);
 *  - a transition delivers Exit to the old state, switches the pointer,
 *    and delivers Entry to the new one. Entry may itself return
 *    transition(): the machinery follows the chain, so a pass-through
 *    state costs nothing. Exit's return value is ignored: exit is an
 *    action, not a decision. An entry chain that never settles spins
 *    forever - keep entries simple;
 *  - start(&initial) arms the machine and delivers the first Entry, so
 *    no state ever runs half-initialized. An active object calls it from
 *    its init(), which the kernel runs before the first event is served.
 *  - Entry and Exit are empty structs, so they do not enlarge the queue
 *    slots. They are delivered by the transition machinery, never
 *    through a queue: post() refuses them at compile time.
 *
 * Context: start(), dispatch() and every handler run in the kernel loop,
 * never in an interrupt handler. A dispatch is never re-entered. An
 * object whose behaviour does not depend on history keeps one state for
 * its whole life, and may use a plain switch inside it.
 *
 * The contract is written so that a hierarchical extension (parent
 * states, unhandled() bubbling to the parent) can be added without
 * changing it; none is built, and nothing below depends on one.
 *
 * Handlers dispatch with match(e, lambdas...), defined below: the event
 * first, then one lambda per alternative, and a [](auto) catch-all for
 * the rest. It is a four-line wrapper over std::visit; `Overloaded` is
 * the standard building block underneath, public for whoever prefers
 * the bare std::visit idiom.
 *
 * Pure kernel code: includes nothing of brio, no hardware, host-testable
 * (test/test_fsm).
 */

#pragma once

#include <stdint.h>
#include <utility>
#include <variant>

namespace brio {

/// The classic visitor-from-lambdas helper: inherits every lambda and
/// pulls all their operator() into one overload set, so std::visit picks
/// the one whose parameter matches the alternative currently held.
template <class... Ts>
struct Overloaded : Ts... {
    using Ts::operator()...;
};

/// match(variant, lambdas...): call the lambda whose parameter type is
/// the alternative the variant holds now. Every alternative must be
/// covered or the call does not compile; a trailing [](auto) is the
/// catch-all for the ones not named. All lambdas return the same type.
/// Compiles to the same switch as the bare std::visit(Overloaded{...}, v).
template <class Variant, class... Handlers>
constexpr decltype(auto) match(Variant&& v, Handlers&&... hs) {
    return std::visit(Overloaded{std::forward<Handlers>(hs)...},
                      std::forward<Variant>(v));
}

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

    /// Outcome of a state handler; build it with handled(), unhandled()
    /// or transition(&next). Stored in one handler pointer: null means
    /// handled, the address of a private marker means unhandled, any other
    /// address names the next state. Keeping Status pointer-sized lets
    /// every handler return it in a register; a kind stored beside the
    /// pointer would make it two words, which the Arm calling convention
    /// returns through memory (measured on the Cortex-M0+ as a memset at
    /// every return). A consequence: transition(nullptr) reads as handled.
    struct Status {
        enum class Kind : uint8_t { handled, unhandled, transition };
        Handler target;

        constexpr Kind kind() const {
            if (target == nullptr) {
                return Kind::handled;
            }
            if (target == &Fsm::unhandled_mark) {
                return Kind::unhandled;
            }
            return Kind::transition;
        }
    };

    static_assert(sizeof(Status) == sizeof(Handler), "a Status is one handler pointer wide");

    static constexpr Status handled() {
        return {nullptr};
    }
    static constexpr Status unhandled() {
        return {&unhandled_mark};
    }
    static constexpr Status transition(Handler next) {
        return {next};
    }

    /// Make `initial` the current state and deliver Entry to it (following
    /// any transition chain Entry returns). Called from the object's init(),
    /// in the kernel loop before the first event is served.
    static void start(Handler initial) {
        state_ = initial;
        follow(state_(entry_event));
    }

    /// Deliver one event to the current state and follow the transition it
    /// returns, if any. Runs to completion in the kernel loop; the kernel
    /// calls it, an application does not.
    static void dispatch(const Event& e) {
        follow(state_(e));
    }

    /// Current state (for tests and diagnostics).
    static Handler current() { return state_; }

private:
    /// The two reserved events, built once as constants: a handler takes
    /// its event by const reference, so one object serves every delivery,
    /// where a temporary built per delivery costs the compiler a clear of
    /// the whole variant.
    static constexpr Event entry_event{std::in_place_type<Entry>};
    static constexpr Event exit_event{std::in_place_type<Exit>};

    /// Never called: its address is unhandled()'s mark, distinct from every
    /// state's because the language gives distinct functions distinct
    /// addresses (an identical-code folding pass keeps them apart too).
    static Status unhandled_mark(const Event&) {
        return {nullptr};
    }

    static void follow(Status s) {
        while (s.kind() == Status::Kind::transition) {
            (void)state_(exit_event);        // exit is an action, not a decision
            state_ = s.target;
            s = state_(entry_event);         // entry may chain (pass-through)
        }
        // handled: done. unhandled: ignored today, HSM bubble hook tomorrow.
    }

    static inline Handler state_ = nullptr;
};

} // namespace brio
