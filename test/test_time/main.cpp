// Host tests for kernel/time.hpp conversions: ceil semantics ("at least
// this long") across tick rates. Run with: pio test -e native

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>

#include "kernel/time.hpp"
#include "host/platform_host.hpp"

using brio::ticks_from_ms;
using brio::ticks_from_secs;
using brio::HostPlatform;

namespace {

// A platform with the AVR Dx rate: 1024 ticks/s, milliseconds do NOT divide.
struct Fake1024 {
    struct CriticalSection {};
    static void idle() {}
    static void break_here() {}
    static uint32_t now() { return 0; }
    static constexpr uint32_t ticks_per_second = 1024;
    static brio::PanicRecord& panic_record() {
        static brio::PanicRecord rec{};
        return rec;
    }
};
static_assert(brio::Platform<Fake1024>);

} // namespace

TEST_CASE("1000 Hz platform: ms conversions are the identity") {
    static_assert(ticks_from_ms<HostPlatform>(0) == 0);
    static_assert(ticks_from_ms<HostPlatform>(1) == 1);
    static_assert(ticks_from_ms<HostPlatform>(500) == 500);
    static_assert(ticks_from_ms<HostPlatform>(86'400'000) == 86'400'000);
    CHECK(ticks_from_ms<HostPlatform>(123) == 123);
}

TEST_CASE("1024 Hz platform: ceil, a timeout never fires early") {
    static_assert(ticks_from_ms<Fake1024>(0) == 0);
    // 1 ms = 1.024 ticks -> 2 ticks: waiting 2 ticks >= 1 ms, 1 tick is not
    static_assert(ticks_from_ms<Fake1024>(1) == 2);
    // 500 ms = 512 ticks exactly
    static_assert(ticks_from_ms<Fake1024>(500) == 512);
    // 999 ms = 1022.976 ticks -> 1023
    static_assert(ticks_from_ms<Fake1024>(999) == 1023);
    // 1000 ms = exactly 1024 ticks
    static_assert(ticks_from_ms<Fake1024>(1000) == 1024);
    CHECK(ticks_from_ms<Fake1024>(1) == 2);

    // ceil property over a broad range: converted duration is always >= the
    // requested one, and never more than one tick longer than exact
    for (uint32_t ms = 0; ms < 5000; ++ms) {
        const uint64_t exact_num = static_cast<uint64_t>(ms) * 1024;
        const uint64_t got = static_cast<uint64_t>(ticks_from_ms<Fake1024>(ms)) * 1000;
        CHECK(got >= exact_num);              // never early
        CHECK(got - exact_num < 1000 * 1);    // less than 1 tick of excess
    }
}

TEST_CASE("seconds conversion is exact on both rates") {
    static_assert(ticks_from_secs<HostPlatform>(60) == 60'000);
    static_assert(ticks_from_secs<Fake1024>(60) == 61'440);
    CHECK(ticks_from_secs<Fake1024>(1) == 1024);
}

TEST_CASE("no 32-bit overflow in the intermediate at large arguments") {
    // ms * tps would overflow u32 already at ~48 days worth of ms with
    // tps=1024: the u64 intermediate must keep the result exact.
    const uint32_t ms = 4'000'000'000u;                       // ~46 days
    const uint64_t exact = (static_cast<uint64_t>(ms) * 1024 + 999) / 1000;
    CHECK(ticks_from_ms<Fake1024>(ms) == static_cast<uint32_t>(exact));
}
