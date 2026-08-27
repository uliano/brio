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

// =============================================================================
// The bulk (span) half of the API
//
// read_span()/consume() and write_span()/publish() hand each side the
// CONTIGUOUS run it already owns under the SPSC invariant. The cases
// below pin the two properties that are easy to get wrong: a span never
// wraps (so a wrapped ring is served in two calls), and the spare slot
// that tells full from empty is never handed out.
// =============================================================================

TEST_CASE("spans on an empty ring: nothing to read, the whole buffer to write") {
    Ring<uint8_t, 8, HostPlatform> r;
    CHECK(r.read_span().empty());
    // The producer starts at index 0 with the tail also at 0, so the run
    // stops one short of the end: that slot is the full/empty marker.
    const auto w = r.write_span();
    CHECK(w.size() == 7);
    CHECK(w.data() != nullptr);
    CHECK(r.capacity() == 7);
}

TEST_CASE("write_span/publish and read_span/consume move a whole run") {
    Ring<uint8_t, 8, HostPlatform> r;
    auto w = r.write_span();
    REQUIRE(w.size() == 7);
    for (uint8_t i = 0; i < 4; ++i) {
        w[i] = static_cast<uint8_t>(0xA0 + i);
    }
    CHECK(r.empty());          // nothing is visible until it is published
    r.publish(4);
    CHECK(r.count() == 4);

    auto rd = r.read_span();
    REQUIRE(rd.size() == 4);
    for (uint8_t i = 0; i < 4; ++i) {
        CHECK(rd[i] == static_cast<uint8_t>(0xA0 + i));
    }
    r.consume(4);
    CHECK(r.empty());
    CHECK(r.read_span().empty());
}

TEST_CASE("a partially consumed run is offered again from where it stopped") {
    Ring<uint8_t, 8, HostPlatform> r;
    auto w = r.write_span();
    for (uint8_t i = 0; i < 6; ++i) {
        w[i] = i;
    }
    r.publish(6);
    auto rd = r.read_span();
    REQUIRE(rd.size() == 6);
    r.consume(2);
    rd = r.read_span();
    REQUIRE(rd.size() == 4);
    CHECK(rd[0] == 2);
    CHECK(rd[3] == 5);
    CHECK(r.count() == 4);
}

TEST_CASE("a span NEVER wraps: a straddling ring is served in two calls") {
    Ring<uint8_t, 8, HostPlatform> r;
    // Push the head near the end of the buffer, then free the front.
    for (uint8_t i = 0; i < 6; ++i) {
        REQUIRE(r.push(i));
    }
    for (uint8_t i = 0; i < 5; ++i) {
        REQUIRE(r.pop().value() == i);
    }
    // head = 6, tail = 5: one element queued, and the free run must stop
    // at the end of the buffer rather than wrapping round to index 0.
    CHECK(r.count() == 1);
    auto w = r.write_span();
    REQUIRE(w.size() == 2);           // indices 6 and 7
    w[0] = 0x10;
    w[1] = 0x11;
    r.publish(2);
    CHECK(r.count() == 3);

    // Now the second half of the free room, from index 0.
    auto w2 = r.write_span();
    REQUIRE(w2.size() == 4);          // 0..3, stopping one short of tail = 5
    w2[0] = 0x20;
    r.publish(1);
    CHECK(r.count() == 4);

    // The readable side straddles the wrap the same way: 5,6,7 first...
    auto rd = r.read_span();
    REQUIRE(rd.size() == 3);
    CHECK(rd[0] == 5);
    CHECK(rd[1] == 0x10);
    CHECK(rd[2] == 0x11);
    r.consume(3);
    // ...then the part that had wrapped.
    auto rd2 = r.read_span();
    REQUIRE(rd2.size() == 1);
    CHECK(rd2[0] == 0x20);
    r.consume(1);
    CHECK(r.empty());
}

TEST_CASE("write_span never offers the spare slot, and is empty when full") {
    Ring<uint8_t, 8, HostPlatform> r;
    auto w = r.write_span();
    r.publish(static_cast<uint8_t>(w.size()));   // fill the whole first run
    CHECK(r.count() == 7);
    CHECK(r.full());
    CHECK(r.write_span().empty());
    CHECK_FALSE(r.push(0));            // and push agrees
    // One slot freed: exactly one slot offered back.
    (void)r.pop();
    CHECK(r.write_span().size() == 1);
}

TEST_CASE("publish and consume are clamped to what is really there") {
    Ring<uint8_t, 8, HostPlatform> r;
    // Publishing more than the free room must not walk the head into the
    // tail and make a full ring read as empty.
    r.publish(200);
    CHECK(r.count() == 7);
    CHECK(r.full());
    CHECK_FALSE(r.empty());
    // Consuming more than is queued must not walk the tail past the head.
    r.consume(200);
    CHECK(r.empty());
    CHECK(r.count() == 0);
    CHECK(r.read_span().empty());
}

TEST_CASE("byte and span operations interleave without losing order") {
    Ring<uint8_t, 16, HostPlatform> r;
    uint32_t produced = 0, consumed = 0;
    for (int round = 0; round < 2000; ++round) {
        // Produce: alternate a span burst with single pushes.
        if ((round & 1) == 0) {
            auto w = r.write_span();
            const size_t take = w.size() < 5u ? w.size() : 5u;
            for (size_t i = 0; i < take; ++i) {
                w[i] = static_cast<uint8_t>(produced + i);
            }
            r.publish(static_cast<uint8_t>(take));
            produced += static_cast<uint32_t>(take);
        } else {
            for (int i = 0; i < 3; ++i) {
                if (r.push(static_cast<uint8_t>(produced))) {
                    ++produced;
                }
            }
        }
        REQUIRE(r.count() <= r.capacity());

        // Consume: the mirror image.
        if ((round % 3) == 0) {
            auto rd = r.read_span();
            const size_t take = rd.size() < 4u ? rd.size() : 4u;
            for (size_t i = 0; i < take; ++i) {
                REQUIRE(rd[i] == static_cast<uint8_t>(consumed + i));
            }
            r.consume(static_cast<uint8_t>(take));
            consumed += static_cast<uint32_t>(take);
        } else {
            for (int i = 0; i < 2; ++i) {
                if (auto v = r.pop()) {
                    REQUIRE(*v == static_cast<uint8_t>(consumed));
                    ++consumed;
                }
            }
        }
    }
    while (auto v = r.pop()) {
        REQUIRE(*v == static_cast<uint8_t>(consumed));
        ++consumed;
    }
    CHECK(produced == consumed);
    CHECK(produced > 5000);   // the ring really was worked
}

TEST_CASE("the guarded path guards the span operations too") {
    NarrowPlatform::CriticalSection::entries = 0;
    Ring<uint8_t, 512, NarrowPlatform> r;
    (void)r.write_span();
    CHECK(NarrowPlatform::CriticalSection::entries == 1);
    r.publish(3);
    CHECK(NarrowPlatform::CriticalSection::entries == 2);
    (void)r.read_span();
    CHECK(NarrowPlatform::CriticalSection::entries == 3);
    r.consume(3);
    CHECK(NarrowPlatform::CriticalSection::entries == 4);
    CHECK(NarrowPlatform::CriticalSection::depth == 0);
    CHECK(r.empty());
}

TEST_CASE("spans are correct at the 65536-slot boundary, where size > index_t") {
    // index_t is uint16_t here and `size` is 65536: an end-of-buffer run
    // computed in index_t would come out as ZERO. This is the case that
    // says the arithmetic names its own width.
    static Ring<uint8_t, 65536, HostPlatform> r;
    r.clear();
    const auto w = r.write_span();
    CHECK(w.size() == 65535);
    r.publish(65535);
    CHECK(r.full());
    const auto rd = r.read_span();
    CHECK(rd.size() == 65535);
    r.consume(65535);
    CHECK(r.empty());
}
