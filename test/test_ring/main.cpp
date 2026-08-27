// Host tests for brio::Ring (SPSC FIFO, util/ring.hpp).
// Run with: ctest --preset host (or ctest --preset host -R <suite name>)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <stdint.h>

#include "util/ring.hpp"
#include "host/platform_host.hpp"

using brio::HostPlatform;
using brio::Ring;

namespace {

// A platform whose only atomic access is one byte (the AVR truth): rings
// with 16-bit indices must then take the critical section. Its guard
// counts entries so the test can see the path that was chosen.
struct NarrowPlatform {
    class CriticalSection {
    public:
        CriticalSection() { ++entries; ++depth; }
        ~CriticalSection() { --depth; }
        CriticalSection(const CriticalSection&) = delete;
        CriticalSection& operator=(const CriticalSection&) = delete;
        static inline uint32_t entries = 0;
        static inline uint8_t depth = 0;
    };
    static void idle() {}
    static void break_here() {}
    static uint32_t now() { return 0; }
    static constexpr uint32_t ticks_per_second = 1000;
    static constexpr unsigned atomic_width = 1;
    static brio::PanicRecord& panic_record() {
        static brio::PanicRecord rec{};
        return rec;
    }
};
static_assert(brio::Platform<NarrowPlatform>);

} // namespace

TEST_CASE("index type and path follow the size and the platform width") {
    static_assert(sizeof(Ring<uint8_t, 2, HostPlatform>::index_t) == 1);
    static_assert(sizeof(Ring<uint8_t, 256, HostPlatform>::index_t) == 1);
    static_assert(sizeof(Ring<uint8_t, 512, HostPlatform>::index_t) == 2);
    static_assert(sizeof(Ring<uint8_t, 65536, HostPlatform>::index_t) == 2);
    static_assert(sizeof(Ring<uint8_t, 131072, HostPlatform>::index_t) == 4);

    // 32-bit-class host: everything lock-free
    static_assert(Ring<uint8_t, 64, HostPlatform>::lock_free);
    static_assert(Ring<uint8_t, 4096, HostPlatform>::lock_free);
    // byte-atomic target: lock-free up to 256 slots, guarded above
    static_assert(Ring<uint8_t, 256, NarrowPlatform>::lock_free);
    static_assert(!Ring<uint8_t, 512, NarrowPlatform>::lock_free);
    // 32-bit index on a 32-bit host: still lock-free; on a 16-bit-word
    // target it would take the guard - same source, no #ifdef
    static_assert(Ring<uint8_t, 131072, HostPlatform>::lock_free);
}

TEST_CASE("a fresh ring is empty and capacity is size - 1") {
    Ring<uint8_t, 8, HostPlatform> r;
    CHECK(r.empty());
    CHECK_FALSE(r.full());
    CHECK(r.count() == 0);
    CHECK(r.capacity() == 7);
    CHECK_FALSE(r.pop().has_value());
}

TEST_CASE("push/pop is FIFO, full rejects, then drains to empty") {
    Ring<uint16_t, 4, HostPlatform> r;
    CHECK(r.push(10));
    CHECK(r.push(20));
    CHECK(r.push(30));
    CHECK(r.full());
    CHECK(r.count() == 3);
    CHECK_FALSE(r.push(40));          // full: nothing written
    CHECK(r.count() == 3);

    CHECK(r.pop().value() == 10);
    CHECK(r.pop().value() == 20);
    CHECK_FALSE(r.full());
    CHECK(r.push(40));                // room again
    CHECK(r.pop().value() == 30);
    CHECK(r.pop().value() == 40);
    CHECK(r.empty());
    CHECK_FALSE(r.pop().has_value());
}

TEST_CASE("indices wrap correctly across many laps") {
    Ring<uint8_t, 4, HostPlatform> r;
    uint8_t next_in = 0, next_out = 0;
    for (int lap = 0; lap < 1000; ++lap) {
        // interleave: 3 in, 3 out - the ring is briefly full every lap
        // and the 2-bit indices wrap on almost every lap
        CHECK(r.push(next_in++));
        CHECK(r.push(next_in++));
        CHECK(r.push(next_in++));
        CHECK(r.full());
        CHECK(r.pop().value() == next_out++);
        CHECK(r.pop().value() == next_out++);
        CHECK(r.pop().value() == next_out++);
        CHECK(r.empty());
    }
    while (auto v = r.pop()) {
        CHECK(*v == next_out++);
    }
    CHECK(next_in == next_out);
}

TEST_CASE("clear resets to empty") {
    Ring<uint8_t, 8, HostPlatform> r;
    r.push(1);
    r.push(2);
    r.clear();
    CHECK(r.empty());
    CHECK(r.count() == 0);
    CHECK(r.push(3));
    CHECK(r.pop().value() == 3);
}

TEST_CASE("the full 65536-slot ring uses every one of its 65535 slots") {
    static Ring<uint8_t, 65536, HostPlatform> r;   // 64 KB: keep it off the stack
    r.clear();
    for (uint32_t i = 0; i < 65535; ++i) {
        REQUIRE(r.push(static_cast<uint8_t>(i)));
    }
    CHECK(r.full());
    CHECK_FALSE(r.push(0));
    for (uint32_t i = 0; i < 65535; ++i) {
        REQUIRE(r.pop().value() == static_cast<uint8_t>(i));
    }
    CHECK(r.empty());
}

TEST_CASE("lock-free path never touches the critical section") {
    NarrowPlatform::CriticalSection::entries = 0;
    Ring<uint8_t, 16, NarrowPlatform> r;
    r.push(1);
    (void)r.pop();
    (void)r.count();
    (void)r.empty();
    (void)r.full();
    CHECK(NarrowPlatform::CriticalSection::entries == 0);
}

TEST_CASE("guarded path wraps every operation and leaves the guard released") {
    NarrowPlatform::CriticalSection::entries = 0;
    Ring<uint8_t, 512, NarrowPlatform> r;
    CHECK(r.push(1));
    CHECK(NarrowPlatform::CriticalSection::entries == 1);
    CHECK(r.pop().value() == 1);
    CHECK(NarrowPlatform::CriticalSection::entries == 2);
    (void)r.count();
    CHECK(NarrowPlatform::CriticalSection::entries == 3);
    (void)r.empty();     // via count()
    (void)r.full();
    CHECK(NarrowPlatform::CriticalSection::entries == 5);
    CHECK(NarrowPlatform::CriticalSection::depth == 0);

    // and it still behaves as a FIFO
    for (int i = 0; i < 511; ++i) REQUIRE(r.push(static_cast<uint8_t>(i)));
    CHECK(r.full());
    CHECK_FALSE(r.push(0));
    for (int i = 0; i < 511; ++i) REQUIRE(r.pop().value() == static_cast<uint8_t>(i));
    CHECK(r.empty());
    CHECK(NarrowPlatform::CriticalSection::depth == 0);
}

TEST_CASE("simulated producer/consumer interleaving keeps order and count") {
    // A producer that pushes in bursts and a consumer that drains in
    // different bursts, over a ring much smaller than the traffic: models
    // an ISR filling and a loop draining. Every byte must come out once,
    // in order, and the ring must never report more than its capacity.
    Ring<uint8_t, 32, HostPlatform> r;
    uint32_t produced = 0, consumed = 0, rejected = 0;
    for (int round = 0; round < 5000; ++round) {
        const int burst_in = (round * 7) % 13;
        for (int i = 0; i < burst_in; ++i) {
            if (r.push(static_cast<uint8_t>(produced))) ++produced;
            else ++rejected;
        }
        CHECK(r.count() <= r.capacity());
        const int burst_out = (round * 5) % 11;
        for (int i = 0; i < burst_out; ++i) {
            if (auto v = r.pop()) {
                REQUIRE(*v == static_cast<uint8_t>(consumed));
                ++consumed;
            }
        }
    }
    while (auto v = r.pop()) {
        REQUIRE(*v == static_cast<uint8_t>(consumed));
        ++consumed;
    }
    CHECK(produced == consumed);
    CHECK(rejected > 0);   // the ring really was pushed past full
}
