// Host tests for kernel/fsm.hpp: the HSM-ready contract on the flat
// implementation - entry/exit ordering, transitions, pass-through chains.
// Run with: pio test -e native

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>
#include <string>
#include <variant>
#include <vector>

#include "kernel/fsm.hpp"

namespace {


struct Go {};            // triggers A -> B
struct Bounce {};        // triggers B -> C via pass-through P
struct Ping { uint8_t n; };  // payload event, handled in place
struct Nope {};          // nobody handles this

// A toy AO exercising the whole contract. States record a trace.
struct Toy : brio::Fsm<Toy, Go, Bounce, Ping, Nope> {
    static inline std::vector<std::string> trace;
    static inline uint8_t last_ping = 0;

    static Status state_a(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { trace.push_back("A:entry"); return handled(); },
            [](brio::Exit)  { trace.push_back("A:exit");  return handled(); },
            [](Go)          { trace.push_back("A:go");    return transition(&state_b); },
            [](Ping p)      { trace.push_back("A:ping");  last_ping = p.n; return handled(); },
            [](auto)        { return unhandled(); }
        );
    }

    static Status state_b(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { trace.push_back("B:entry"); return handled(); },
            [](brio::Exit)  { trace.push_back("B:exit");  return handled(); },
            [](Bounce)      { trace.push_back("B:bounce"); return transition(&state_pass); },
            [](auto)        { return unhandled(); }
        );
    }

    // Pass-through: its Entry immediately chains a transition to C.
    static Status state_pass(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { trace.push_back("P:entry"); return transition(&state_c); },
            [](brio::Exit)  { trace.push_back("P:exit");  return handled(); },
            [](auto)        { return unhandled(); }
        );
    }

    static Status state_c(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { trace.push_back("C:entry"); return handled(); },
            [](brio::Exit)  { trace.push_back("C:exit");  return handled(); },
            [](auto)        { return unhandled(); }
        );
    }

    static void reset() { trace.clear(); last_ping = 0; }
};

using Trace = std::vector<std::string>;

} // namespace

TEST_CASE("start arms the initial state and delivers exactly one Entry") {
    Toy::reset();
    Toy::start(&Toy::state_a);

    CHECK(Toy::trace == Trace{"A:entry"});
    CHECK(Toy::current() == &Toy::state_a);
}

TEST_CASE("a handled event causes no entry/exit and no state change") {
    Toy::reset();
    Toy::start(&Toy::state_a);
    Toy::trace.clear();

    Toy::dispatch(Toy::Event{Ping{42}});

    CHECK(Toy::trace == Trace{"A:ping"});
    CHECK(Toy::last_ping == 42);
    CHECK(Toy::current() == &Toy::state_a);
}

TEST_CASE("a transition runs old-exit then new-entry, in that order") {
    Toy::reset();
    Toy::start(&Toy::state_a);
    Toy::trace.clear();

    Toy::dispatch(Toy::Event{Go{}});

    CHECK(Toy::trace == Trace{"A:go", "A:exit", "B:entry"});
    CHECK(Toy::current() == &Toy::state_b);
}

TEST_CASE("an entry may chain a transition: pass-through states") {
    Toy::reset();
    Toy::start(&Toy::state_a);
    Toy::dispatch(Toy::Event{Go{}});          // A -> B
    Toy::trace.clear();

    Toy::dispatch(Toy::Event{Bounce{}});      // B -> P -(entry chains)-> C

    CHECK(Toy::trace == Trace{"B:bounce", "B:exit", "P:entry", "P:exit", "C:entry"});
    CHECK(Toy::current() == &Toy::state_c);
}

TEST_CASE("an unhandled event is ignored and changes nothing") {
    Toy::reset();
    Toy::start(&Toy::state_a);
    Toy::trace.clear();

    Toy::dispatch(Toy::Event{Nope{}});

    CHECK(Toy::trace.empty());
    CHECK(Toy::current() == &Toy::state_a);
}

TEST_CASE("two AOs with the same alternatives have independent machines") {
    struct Other : brio::Fsm<Other, Go, Bounce, Ping, Nope> {
        static Status only(const Event&) { return handled(); }
    };

    Toy::reset();
    Toy::start(&Toy::state_a);
    Other::start(&Other::only);

    CHECK(Toy::current() == &Toy::state_a);       // untouched by Other
    CHECK(Other::current() == &Other::only);
}
