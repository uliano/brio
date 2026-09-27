// Host tests for brio::EventQueue (MPSC per-AO queue) on HostPlatform.
// Run with: ctest --preset host (or ctest --preset host -R <suite name>)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>
#include <stdint.h>
#include <variant>

#include "kernel/event_queue.hpp"
#include "host/platform.hpp"

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
    q.push(30);           // depth events, ALL usable (no sacrificed slot)
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

// ---------------------------------------------------------------------------
// take(): the consumer's verb - the oldest event IN ITS SLOT, the slot the
// consumer's until its next take, depth events waiting in every state.
// ---------------------------------------------------------------------------

TEST_CASE("take() on an empty queue is null and takes no critical section") {
    HostPlatform::reset();
    EventQueue<Event, 3, HostPlatform> q;

    CHECK(q.take() == nullptr);
    CHECK(q.empty());
    CHECK(q.size() == 0);
    CHECK(HostPlatform::CriticalSection::entries == 0);
}

TEST_CASE("take() hands the oldest event in its slot, with no critical section") {
    HostPlatform::reset();
    EventQueue<Event, 3, HostPlatform> q;
    q.push(ButtonDown{1});
    q.push(ButtonDown{2});
    const uint32_t before = HostPlatform::CriticalSection::entries;

    const Event* e = q.take();
    REQUIRE(e != nullptr);
    CHECK(HostPlatform::CriticalSection::entries == before);
    CHECK(std::get<ButtonDown>(*e).id == 1);
    CHECK(q.size() == 1);            // the served event is not waiting
    CHECK(std::get<ButtonDown>(*q.take()).id == 2);
    CHECK(q.take() == nullptr);
}

TEST_CASE("while an event is served, depth more can wait and its slot is never written") {
    HostPlatform::reset();
    EventQueue<Event, 3, HostPlatform> q;
    q.push(ButtonDown{7});

    const Event* served = q.take();
    REQUIRE(served != nullptr);
    q.push(TempReading{101});        // posts arriving during the dispatch -
    q.push(TempReading{102});        // from an ISR or from the dispatch itself
    q.push(TempReading{103});
    CHECK(q.size() == 3);            // depth of them, all accepted
    CHECK(q.overflows() == 0);
    q.push(TempReading{104});        // the (depth + 1)-th: overflow, as today
    CHECK(q.overflows() == 1);

    // The served event is intact in its slot through all of it.
    REQUIRE(std::holds_alternative<ButtonDown>(*served));
    CHECK(std::get<ButtonDown>(*served).id == 7);

    CHECK(std::get<TempReading>(*q.take()).centi == 101);
    CHECK(std::get<TempReading>(*q.take()).centi == 102);
    CHECK(std::get<TempReading>(*q.take()).centi == 103);
    CHECK(q.take() == nullptr);
}

TEST_CASE("an idle queue also takes exactly depth events: the capacity is one number") {
    HostPlatform::reset();
    EventQueue<uint8_t, 2, HostPlatform> q;
    q.push(1);
    q.push(2);
    q.push(3);                       // nothing served, depth waiting: overflow
    CHECK(q.size() == 2);
    CHECK(q.overflows() == 1);
    CHECK(q.capacity() == 2);
}

TEST_CASE("the next take() releases the previous slot, and the ring reuses it in order") {
    HostPlatform::reset();
    EventQueue<uint16_t, 2, HostPlatform> q;
    // Keep the queue full around every served event across many wraps of
    // the three slots: each value comes out once and in order.
    q.push(0);
    q.push(1);
    uint16_t expected = 0;
    for (uint16_t i = 2; i < 700; ++i) {
        const uint16_t* e = q.take();
        REQUIRE(e != nullptr);
        CHECK(*e == expected);
        q.push(i);                   // lands behind, never in the served slot
        CHECK(*e == expected);       // still intact after the push
        ++expected;
    }
    CHECK(q.overflows() == 0);
}

TEST_CASE("the counters wrap past 255 events at the largest depth") {
    HostPlatform::reset();
    EventQueue<uint16_t, 255, HostPlatform> q;   // 256 slots: the index's own width
    for (uint16_t i = 0; i < 255; ++i) {
        q.push(i);
    }
    CHECK(q.size() == 255);
    q.push(999);
    CHECK(q.overflows() == 1);
    for (uint16_t round = 0; round < 3; ++round) {
        for (uint16_t i = 0; i < 255; ++i) {
            const uint16_t* e = q.take();
            REQUIRE(e != nullptr);
            CHECK(*e == static_cast<uint16_t>(round * 255u + i));
            q.push(static_cast<uint16_t>((round + 1u) * 255u + i));
        }
    }
    CHECK(q.size() == 255);
    CHECK(q.overflows() == 1);
}

TEST_CASE("push builds each alternative in its slot, and a whole Event too") {
    HostPlatform::reset();
    EventQueue<Event, 4, HostPlatform> q;
    q.push(Tick{});
    q.push(ButtonDown{9});
    q.push(TempReading{-3});
    q.push(Event{ButtonDown{11}});
    CHECK(std::holds_alternative<Tick>(*q.take()));
    CHECK(std::get<ButtonDown>(*q.take()).id == 9);
    CHECK(std::get<TempReading>(*q.take()).centi == -3);
    CHECK(std::get<ButtonDown>(*q.take()).id == 11);
}

TEST_CASE("pop() is take() with a copy out, and empty() and size() take no lock") {
    HostPlatform::reset();
    EventQueue<uint8_t, 3, HostPlatform> q;
    q.push(4);
    q.push(5);
    const uint32_t before = HostPlatform::CriticalSection::entries;
    CHECK_FALSE(q.empty());
    CHECK(q.size() == 2);
    CHECK(q.pop().value() == 4);
    CHECK(q.pop().value() == 5);
    CHECK_FALSE(q.pop().has_value());
    CHECK(HostPlatform::CriticalSection::entries == before);
}
