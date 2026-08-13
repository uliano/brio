// Host tests for brio::EventQueue (MPSC per-AO queue) on HostPlatform.
// Run with: pio test -e native

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>
#include <stdint.h>
#include <variant>

#include "kernel/event_queue.hpp"
#include "host/platform_host.hpp"

using brio::EventQueue;
using brio::HostPlatform;

namespace {

struct ButtonDown { uint8_t id; };
struct TempReading { int16_t centi; };
struct Tick {};
using Event = std::variant<Tick, ButtonDown, TempReading>;

} // namespace

TEST_CASE("a fresh queue is empty") {
    HostPlatform::reset();
    EventQueue<uint8_t, 4, HostPlatform> q;

    CHECK(q.empty());
    CHECK(q.size() == 0);
    CHECK(q.capacity() == 4);
    CHECK(q.overflows() == 0);
    CHECK_FALSE(q.pop().has_value());
}

TEST_CASE("push/pop preserves FIFO order and full capacity is usable") {
    HostPlatform::reset();
    EventQueue<uint8_t, 3, HostPlatform> q;

    q.push(10);
    q.push(20);
    q.push(30);           // depth slots, ALL usable (no sacrificed slot)
    CHECK(q.size() == 3);
    CHECK(q.overflows() == 0);

    CHECK(q.pop().value() == 10);
    CHECK(q.pop().value() == 20);
    CHECK(q.pop().value() == 30);
    CHECK(q.empty());
}

TEST_CASE("overflow drops the new event, counts, and leaves content intact") {
    HostPlatform::reset();
    EventQueue<uint8_t, 2, HostPlatform> q;

    q.push(1);
    q.push(2);
    q.push(3);            // full: dropped
    q.push(4);            // full: dropped

    CHECK(q.size() == 2);
    CHECK(q.overflows() == 2);
    CHECK(q.pop().value() == 1);   // survivors are the OLD events, in order
    CHECK(q.pop().value() == 2);
    CHECK_FALSE(q.pop().has_value());

    q.push(5);            // queue works normally after the overflow
    CHECK(q.pop().value() == 5);
    CHECK(q.overflows() == 2);     // the counter is history, not state
}

TEST_CASE("indices wrap correctly over many cycles at arbitrary depth") {
    HostPlatform::reset();
    EventQueue<uint16_t, 5, HostPlatform> q;   // 5: deliberately not a power of 2

    // keep 2 events in flight while cycling far past every index wrap
    q.push(0);
    q.push(1);
    for (uint16_t i = 2; i < 1000; ++i) {
        q.push(i);
        CHECK(q.pop().value() == i - 2);
    }
    CHECK(q.size() == 2);
    CHECK(q.overflows() == 0);
}

TEST_CASE("the overflow counter saturates instead of wrapping") {
    HostPlatform::reset();
    EventQueue<uint8_t, 1, HostPlatform> q;

    q.push(42);
    for (uint32_t i = 0; i < 70000; ++i) {  // > UINT16_MAX overflows
        q.push(0);
    }
    CHECK(q.overflows() == UINT16_MAX);
    CHECK(q.pop().value() == 42);
}

TEST_CASE("a variant event survives the queue with alternative and payload") {
    HostPlatform::reset();
    EventQueue<Event, 4, HostPlatform> q;
    static_assert(sizeof(Event) <= 8, "event size budget");

    q.push(Tick{});
    q.push(ButtonDown{7});
    q.push(TempReading{-1250});

    CHECK(std::holds_alternative<Tick>(q.pop().value()));

    Event e = q.pop().value();
    const auto* button = std::get_if<ButtonDown>(&e);
    CHECK(button != nullptr);
    if (button != nullptr) {
        CHECK(button->id == 7);
    }

    e = q.pop().value();
    const auto* temp = std::get_if<TempReading>(&e);
    CHECK(temp != nullptr);
    if (temp != nullptr) {
        CHECK(temp->centi == -1250);
    }
}

TEST_CASE("every operation balances its critical section") {
    HostPlatform::reset();
    EventQueue<uint8_t, 2, HostPlatform> q;

    q.push(1);
    q.push(2);
    q.push(3);            // overflow path too
    (void)q.pop();
    (void)q.empty();
    (void)q.size();
    (void)q.overflows();

    CHECK(HostPlatform::CriticalSection::depth == 0);
}
