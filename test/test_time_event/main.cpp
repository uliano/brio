// Host tests for kernel/time_event.hpp: one-shot exactness, drift-free
// periodics, catch-up after lag, disarm, re-arm from fire, counter wrap.
// Run with: pio test -e native

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>
#include <variant>
#include <vector>

#include "host/platform_host.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/fsm.hpp"
#include "kernel/time_event.hpp"

namespace {

template <class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
using brio::HostPlatform;
using TE = brio::TimeEvents<HostPlatform>;

struct Beep { uint8_t id; };

std::vector<uint32_t> fired_at;   // HostPlatform::ticks at each Beep

struct EarAo : brio::Fsm<EarAo, Beep> {
    static inline brio::EventQueue<Event, 8, HostPlatform> queue;
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return std::visit(overloaded{
            [](brio::Entry) { return handled(); },
            [](Beep) { fired_at.push_back(HostPlatform::ticks); return handled(); },
            [](auto) { return unhandled(); },
        }, e);
    }
};

// advance the virtual clock one tick at a time, processing + draining
// like the real loop would
void run_ticks(uint32_t n) {
    for (uint32_t i = 0; i < n; ++i) {
        ++HostPlatform::ticks;
        TE::process();
        while (auto e = EarAo::queue.pop()) {
            EarAo::dispatch(*e);
        }
    }
}

void reset() {
    HostPlatform::reset();
    TE::clear_all();
    fired_at.clear();
    while (EarAo::queue.pop().has_value()) {}
    EarAo::init();
}

struct SelfAo : brio::Fsm<SelfAo, Beep> {
    static inline brio::EventQueue<Event, 4, HostPlatform> queue;
    static inline brio::TimeEvent<HostPlatform, SelfAo, Beep> te{Beep{4}};
    static inline uint8_t count = 0;
    static void init() { start(&only); }
    static Status only(const Event& e) {
        return std::visit(overloaded{
            [](brio::Entry) { return handled(); },
            [](Beep) {
                if (++count < 3) { te.arm(4); }
                return handled();
            },
            [](auto) { return unhandled(); },
        }, e);
    }
};

} // namespace

TEST_CASE("a one-shot fires exactly once, at its tick, never before") {
    reset();
    brio::TimeEvent<HostPlatform, EarAo, Beep> te{Beep{1}};

    te.arm(10);
    CHECK(te.armed());
    run_ticks(9);
    CHECK(fired_at.empty());          // not early
    run_ticks(1);
    CHECK(fired_at == std::vector<uint32_t>{10});
    CHECK_FALSE(te.armed());          // consumed
    run_ticks(50);
    CHECK(fired_at.size() == 1);      // once means once
}

TEST_CASE("a periodic fires with drift-free cadence") {
    reset();
    brio::TimeEvent<HostPlatform, EarAo, Beep> te{Beep{2}};

    te.arm_every(7);
    run_ticks(70);
    CHECK(fired_at == std::vector<uint32_t>{7, 14, 21, 28, 35, 42, 49, 56, 63, 70});
    CHECK(te.armed());                // still going
    te.disarm();
    run_ticks(30);
    CHECK(fired_at.size() == 10);     // disarm stops it
}

TEST_CASE("processing lag: at most one firing per turn, cadence catches up") {
    reset();
    brio::TimeEvent<HostPlatform, EarAo, Beep> te{Beep{3}};

    te.arm_every(5);
    // the loop stalls for 23 ticks (e.g. a long dispatch), then resumes
    HostPlatform::ticks += 23;
    TE::process();                    // one firing per process() call...
    while (auto e = EarAo::queue.pop()) { EarAo::dispatch(*e); }
    CHECK(fired_at.size() == 1);
    TE::process();                    // ...but the backlog drains turn by turn
    while (auto e = EarAo::queue.pop()) { EarAo::dispatch(*e); }
    TE::process();
    while (auto e = EarAo::queue.pop()) { EarAo::dispatch(*e); }
    TE::process();
    while (auto e = EarAo::queue.pop()) { EarAo::dispatch(*e); }
    CHECK(fired_at.size() == 4);      // deadlines 5,10,15,20 all served
    // long-run count is preserved: after 50 total ticks, 10 firings
    run_ticks(27);                    // now at tick 50
    CHECK(fired_at.size() == 10);
}

TEST_CASE("a one-shot may re-arm itself from its own firing") {
    HostPlatform::reset();
    TE::clear_all();
    SelfAo::count = 0;
    SelfAo::init();
    SelfAo::te.arm(4);
    for (uint32_t i = 0; i < 20; ++i) {
        ++HostPlatform::ticks;
        TE::process();
        while (auto e = SelfAo::queue.pop()) { SelfAo::dispatch(*e); }
    }
    CHECK(SelfAo::count == 3);        // 3 chained one-shots, then quiet
    CHECK_FALSE(SelfAo::te.armed());
}

TEST_CASE("deadlines survive the 32-bit counter wrap") {
    reset();
    brio::TimeEvent<HostPlatform, EarAo, Beep> te{Beep{5}};

    HostPlatform::ticks = UINT32_MAX - 3;   // 4 ticks to the wrap
    te.arm(10);                             // deadline wraps past zero
    run_ticks(9);
    CHECK(fired_at.empty());
    run_ticks(1);
    CHECK(fired_at.size() == 1);
    CHECK(HostPlatform::ticks == 6);        // we are past the wrap indeed
}
