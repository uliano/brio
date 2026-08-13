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

template <class... Ts> struct overloaded : Ts... { using Ts::operator()...; };

struct Go {};            // triggers A -> B
struct Bounce {};        // triggers B -> C via pass-through P
struct Ping { uint8_t n; };  // payload event, handled in place
struct Nope {};          // nobody handles this

// A toy AO exercising the whole contract. States record a trace.
struct ToyAo : brio::Fsm<ToyAo, Go, Bounce, Ping, Nope> {
    static inline std::vector<std::string> trace;
    static inline uint8_t last_ping = 0;

    static Status state_a(const Event& e) {
        return std::visit(overloaded{
            [](brio::Entry) { trace.push_back("A:entry"); return handled(); },
            [](brio::Exit)  { trace.push_back("A:exit");  return handled(); },
            [](Go)          { trace.push_back("A:go");    return transition(&state_b); },
            [](Ping p)      { trace.push_back("A:ping");  last_ping = p.n; return handled(); },
            [](auto)        { return unhandled(); },
        }, e);
    }

    static Status state_b(const Event& e) {
        return std::visit(overloaded{
            [](brio::Entry) { trace.push_back("B:entry"); return handled(); },
            [](brio::Exit)  { trace.push_back("B:exit");  return handled(); },
            [](Bounce)      { trace.push_back("B:bounce"); return transition(&state_pass); },
            [](auto)        { return unhandled(); },
        }, e);
    }

    // Pass-through: its Entry immediately chains a transition to C.
    static Status state_pass(const Event& e) {
        return std::visit(overloaded{
            [](brio::Entry) { trace.push_back("P:entry"); return transition(&state_c); },
            [](brio::Exit)  { trace.push_back("P:exit");  return handled(); },
            [](auto)        { return unhandled(); },
        }, e);
    }

    static Status state_c(const Event& e) {
        return std::visit(overloaded{
            [](brio::Entry) { trace.push_back("C:entry"); return handled(); },
            [](brio::Exit)  { trace.push_back("C:exit");  return handled(); },
            [](auto)        { return unhandled(); },
        }, e);
    }

    static void reset() { trace.clear(); last_ping = 0; }
};

using Trace = std::vector<std::string>;

} // namespace

TEST_CASE("start arms the initial state and delivers exactly one Entry") {
    ToyAo::reset();
    ToyAo::start(&ToyAo::state_a);

    CHECK(ToyAo::trace == Trace{"A:entry"});
    CHECK(ToyAo::current() == &ToyAo::state_a);
}

TEST_CASE("a handled event causes no entry/exit and no state change") {
    ToyAo::reset();
    ToyAo::start(&ToyAo::state_a);
    ToyAo::trace.clear();

    ToyAo::dispatch(ToyAo::Event{Ping{42}});

    CHECK(ToyAo::trace == Trace{"A:ping"});
    CHECK(ToyAo::last_ping == 42);
    CHECK(ToyAo::current() == &ToyAo::state_a);
}

TEST_CASE("a transition runs old-exit then new-entry, in that order") {
    ToyAo::reset();
    ToyAo::start(&ToyAo::state_a);
    ToyAo::trace.clear();

    ToyAo::dispatch(ToyAo::Event{Go{}});

    CHECK(ToyAo::trace == Trace{"A:go", "A:exit", "B:entry"});
    CHECK(ToyAo::current() == &ToyAo::state_b);
}

TEST_CASE("an entry may chain a transition: pass-through states") {
    ToyAo::reset();
    ToyAo::start(&ToyAo::state_a);
    ToyAo::dispatch(ToyAo::Event{Go{}});          // A -> B
    ToyAo::trace.clear();

    ToyAo::dispatch(ToyAo::Event{Bounce{}});      // B -> P -(entry chains)-> C

    CHECK(ToyAo::trace == Trace{"B:bounce", "B:exit", "P:entry", "P:exit", "C:entry"});
    CHECK(ToyAo::current() == &ToyAo::state_c);
}

TEST_CASE("an unhandled event is ignored and changes nothing") {
    ToyAo::reset();
    ToyAo::start(&ToyAo::state_a);
    ToyAo::trace.clear();

    ToyAo::dispatch(ToyAo::Event{Nope{}});

    CHECK(ToyAo::trace.empty());
    CHECK(ToyAo::current() == &ToyAo::state_a);
}

TEST_CASE("two AOs with the same alternatives have independent machines") {
    struct OtherAo : brio::Fsm<OtherAo, Go, Bounce, Ping, Nope> {
        static Status only(const Event&) { return handled(); }
    };

    ToyAo::reset();
    ToyAo::start(&ToyAo::state_a);
    OtherAo::start(&OtherAo::only);

    CHECK(ToyAo::current() == &ToyAo::state_a);       // untouched by OtherAo
    CHECK(OtherAo::current() == &OtherAo::only);
}
