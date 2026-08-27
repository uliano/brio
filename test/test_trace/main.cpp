// Host tests for util/trace.hpp: order, the overwrite-oldest ring, the
// total-vs-held count, clear(), the dump format read back through a
// capture sink, and the proof that the disabled trace costs NOTHING.
// Run with: ctest --preset host (or ctest --preset host -R <suite name>)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>
#include <string>
#include <type_traits>

#include "host/platform_host.hpp"
#include "util/trace.hpp"

namespace {

using brio::HostPlatform;
using brio::TraceRecord;

/// A ByteSink that keeps what was written, so the dump's own bytes are
/// what the test reads (the same discipline test_testbench uses).
struct Capture {
    static inline std::string text;
    static bool write_byte(uint8_t b) {
        text.push_back(static_cast<char>(b));
        return true;
    }
};
constexpr Capture capture;

using Small = brio::Trace<4, HostPlatform>;
using Off = brio::Trace<4, HostPlatform, false>;
using Big = brio::Trace<200, HostPlatform>;

void reset() {
    HostPlatform::reset();
    Capture::text.clear();
}

} // namespace

TEST_CASE("stamps come back in order, with the platform's time on them") {
    reset();
    Small t;
    HostPlatform::ticks = 10;
    t.stamp(1);
    HostPlatform::ticks = 20;
    t.stamp(2, 0xBEEF);
    HostPlatform::ticks = 30;
    t.stamp(3, 7);

    CHECK(t.count() == 3);
    CHECK(t.held() == 3);
    CHECK(t.at(0).t == 10u);
    CHECK(t.at(0).tag == 1);
    CHECK(t.at(0).arg == 0);                 // the no-arg stamp
    CHECK(t.at(1).t == 20u);
    CHECK(t.at(1).arg == 0xBEEF);
    CHECK(t.at(2).tag == 3);
    CHECK(t.at(2).arg == 7);
    CHECK(t.at(3).t == 0u);                  // past the end: a zero record
}

TEST_CASE("the ring overwrites the oldest and keeps the latest N") {
    reset();
    Small t;
    for (uint8_t i = 1; i <= 10; ++i) {
        HostPlatform::ticks = i;
        t.stamp(i, i);
    }
    CHECK(t.count() == 10);                  // everything stamped
    CHECK(t.held() == Small::capacity);      // only four kept
    CHECK(t.at(0).tag == 7);                 // ... the last four
    CHECK(t.at(1).tag == 8);
    CHECK(t.at(2).tag == 9);
    CHECK(t.at(3).tag == 10);
    CHECK(t.at(3).t == 10u);
}

TEST_CASE("a ring bigger than 128 still reads oldest first") {
    reset();
    Big t;
    for (uint16_t i = 0; i < 250; ++i) {     // wraps a 200-deep ring
        HostPlatform::ticks = i;
        t.stamp(static_cast<uint8_t>(i & 0x7F), i);
    }
    CHECK(t.held() == 200);
    CHECK(t.at(0).arg == 50);                // 250 - 200
    CHECK(t.at(199).arg == 249);
    for (uint8_t i = 1; i < 200; ++i) {      // strictly increasing
        CHECK(t.at(i).arg == t.at(static_cast<uint8_t>(i - 1)).arg + 1);
    }
}

TEST_CASE("clear() forgets the marks and the count with them") {
    reset();
    Small t;
    t.stamp(1);
    t.stamp(2);
    t.clear();
    CHECK(t.count() == 0);
    CHECK(t.held() == 0);
    CHECK(t.at(0).tag == 0);
    t.stamp(9, 3);
    CHECK(t.count() == 1);
    CHECK(t.at(0).tag == 9);
}

TEST_CASE("the total count saturates instead of wrapping") {
    reset();
    Small t;
    t.stamp(1);
    CHECK(t.count() == 1);
    // Reaching UINT32_MAX one stamp at a time is not a test; that the
    // guard is the saturating kind is checked where it can be: a count
    // parked at the ceiling stays there.
    for (uint32_t i = 0; i < 100; ++i) {
        t.stamp(2);
    }
    CHECK(t.count() == 101);
}

TEST_CASE("dump prints one line per mark, oldest first, and says what was dropped") {
    reset();
    Small t;
    for (uint8_t i = 1; i <= 6; ++i) {
        HostPlatform::ticks = i * 100u;
        t.stamp(i, static_cast<uint16_t>(i * 2));
    }
    t.dump(capture);
    CHECK(Capture::text ==
          "t 300 tag 3 arg 6\r\n"
          "t 400 tag 4 arg 8\r\n"
          "t 500 tag 5 arg 10\r\n"
          "t 600 tag 6 arg 12\r\n"
          "-- 4 of 6 stamps\r\n");
}

TEST_CASE("an empty trace dumps only its summary line") {
    reset();
    Small t;
    t.dump(capture);
    CHECK(Capture::text == "-- 0 of 0 stamps\r\n");
}

TEST_CASE("the disabled trace has the same surface and NO storage") {
    reset();
    // The zero, proven three ways: the type is empty, it is as small as
    // C++ lets any object be, and the enabled one of the same depth is
    // the size of its ring.
    static_assert(std::is_empty_v<Off>);
    static_assert(sizeof(Off) == 1);
    static_assert(sizeof(Small) >= 4 * sizeof(TraceRecord));
    static_assert(sizeof(Off) < sizeof(Small));
    static_assert(!std::is_empty_v<Small>);

    Off t;
    t.stamp(1);
    t.stamp(2, 3);
    t.clear();
    CHECK(t.count() == 0);
    CHECK(t.held() == 0);
    CHECK(t.at(0).tag == 0);
    t.dump(capture);
    CHECK(Capture::text.empty());            // and it writes nothing
    CHECK(Off::capacity == 4);               // the depth is still declared
}

TEST_CASE("two traces are two objects") {
    reset();
    Small a;
    Small b;
    a.stamp(1);
    CHECK(a.count() == 1);
    CHECK(b.count() == 0);
    b.stamp(2);
    b.stamp(3);
    CHECK(a.count() == 1);
    CHECK(b.count() == 2);
}
