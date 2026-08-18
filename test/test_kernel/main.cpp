// Host tests for kernel/kernel.hpp (+ active_object.hpp, post.hpp): priority order,
// one-event-per-step, init ordering, idle gating, post/publish.
// Run with: pio test -e native

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>
#include <string>
#include <variant>
#include <vector>

#include "host/platform_host.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/kernel.hpp"

namespace {

using brio::HostPlatform;

std::vector<std::string> trace;

struct Hit { uint8_t n; };
struct Note { uint8_t n; };   // the published notification

struct High : brio::Fsm<High, Hit, Note> {
    static inline brio::EventQueue<Event, 4, HostPlatform> queue;
    static void init() { trace.push_back("hi:init"); start(&only); }
    static Status only(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { trace.push_back("hi:entry"); return handled(); },
            [](Hit h)  { trace.push_back("hi:hit" + std::to_string(h.n)); return handled(); },
            [](Note n) { trace.push_back("hi:note" + std::to_string(n.n)); return handled(); },
            [](auto)   { return unhandled(); }
        );
    }
};

struct Low : brio::Fsm<Low, Hit, Note> {
    static inline brio::EventQueue<Event, 4, HostPlatform> queue;
    static void init() { trace.push_back("lo:init"); start(&only); }
    static Status only(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { trace.push_back("lo:entry"); return handled(); },
            [](Hit h)  { trace.push_back("lo:hit" + std::to_string(h.n)); return handled(); },
            [](Note n) { trace.push_back("lo:note" + std::to_string(n.n)); return handled(); },
            [](auto)   { return unhandled(); }
        );
    }
};

using K = brio::Kernel<HostPlatform, High, Low>;
using Trace = std::vector<std::string>;

struct PingPong : brio::Fsm<PingPong, Hit> {
    static inline brio::EventQueue<Event, 4, HostPlatform> queue;
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { return handled(); },
            [](Hit h) {
                trace.push_back("pp:hit" + std::to_string(h.n));
                if (h.n > 0) {
                    brio::post<PingPong>(Hit{static_cast<uint8_t>(h.n - 1)});
                }
                return handled();
            },
            [](auto) { return unhandled(); }
        );
    }
};

void reset() {
    trace.clear();
    HostPlatform::reset();
    while (High::queue.pop().has_value()) {}
    while (Low::queue.pop().has_value()) {}
}

} // namespace

TEST_CASE("init_all starts every AO in pack order") {
    reset();
    K::init_all();
    CHECK(trace == Trace{"hi:init", "hi:entry", "lo:init", "lo:entry"});
}

TEST_CASE("step serves ONE event, highest priority first, rescan from top") {
    reset();
    K::init_all();
    trace.clear();

    brio::post<Low>(Hit{1});
    brio::post<High>(Hit{2});
    brio::post<High>(Hit{3});

    CHECK(K::step());                 // hi:2 (priority beats FIFO arrival)
    CHECK(trace == Trace{"hi:hit2"});
    CHECK(K::step());                 // hi:3 (high queue drained first)
    CHECK(K::step());                 // lo:1 only now
    CHECK(trace == Trace{"hi:hit2", "hi:hit3", "lo:hit1"});
    CHECK_FALSE(K::step());           // nothing left
}

TEST_CASE("an event posted DURING a dispatch is served on the next step") {
    using K2 = brio::Kernel<HostPlatform, PingPong>;

    reset();
    while (PingPong::queue.pop().has_value()) {}
    K2::init_all();
    brio::post<PingPong>(Hit{2});

    CHECK(K2::step());
    CHECK(K2::step());
    CHECK(K2::step());
    CHECK_FALSE(K2::step());
    CHECK(trace == Trace{"pp:hit2", "pp:hit1", "pp:hit0"});
}

TEST_CASE("idle_if_empty sleeps only when every queue is empty") {
    reset();
    K::init_all();

    K::idle_if_empty();
    CHECK(HostPlatform::idle_calls == 1);

    brio::post<Low>(Hit{9});
    K::idle_if_empty();                       // something pending: no sleep
    CHECK(HostPlatform::idle_calls == 1);

    CHECK(K::step());
    K::idle_if_empty();
    CHECK(HostPlatform::idle_calls == 2);
    CHECK(HostPlatform::CriticalSection::depth == 0);
}

TEST_CASE("publish delivers one copy to every subscriber, in list order") {
    reset();
    K::init_all();
    trace.clear();

    brio::publish(brio::Subscribers<High, Low>{}, Note{5});

    CHECK(High::queue.size() == 1);
    CHECK(Low::queue.size() == 1);
    while (K::step()) {}
    CHECK(trace == Trace{"hi:note5", "lo:note5"});
}
