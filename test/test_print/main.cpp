// Host tests for util/print.hpp's integers: every width through print(),
// and the 64-bit path's own two converters against the standard
// library's text, at every power of ten and its neighbours, the 2^32
// edge the path splits on, both extremes and a pseudo-random sweep. The
// host's `long` is 64 bits, so here every 64-bit type - long included -
// takes the explicit-width path the 32-bit targets and the AVR take for
// int64_t. Run with: ctest --preset host (or ctest --preset host -R
// test_print)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>
#include <limits>
#include <string>

#include "util/print.hpp"

namespace {

struct Capture {
    static inline std::string text;
    static bool write_byte(uint8_t b) {
        text.push_back(static_cast<char>(b));
        return true;
    }
};
static_assert(brio::ByteSink<Capture>);

template <typename... Args>
std::string printed(const Args&... args) {
    Capture::text.clear();
    brio::print(Capture{}, args...);
    return Capture::text;
}

std::string u64_text(uint64_t v) {
    char buffer[21];
    return brio::u64toa(v, buffer);
}

std::string i64_text(int64_t v) {
    char buffer[21];
    return brio::i64toa(v, buffer);
}

/// The 64-bit generator the bench suites use for patterns (xorshift64).
uint64_t next(uint64_t& x) {
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    return x;
}

} // namespace

TEST_CASE("narrow integers print as they always did") {
    CHECK(printed(uint8_t{0}) == "0");
    CHECK(printed(uint8_t{255}) == "255");
    CHECK(printed(int8_t{-128}) == "-128");
    CHECK(printed(uint16_t{65535}) == "65535");
    CHECK(printed(int16_t{-32768}) == "-32768");
    CHECK(printed(uint32_t{4294967295u}) == "4294967295");
    CHECK(printed(int32_t{std::numeric_limits<int32_t>::min()}) == "-2147483648");
    CHECK(printed(true) == "1");
    CHECK(printed(false) == "0");
}

TEST_CASE("64-bit integers print whole, not cut to a long") {
    CHECK(printed(uint64_t{0}) == "0");
    CHECK(printed(std::numeric_limits<uint64_t>::max()) == "18446744073709551615");
    CHECK(printed(std::numeric_limits<int64_t>::max()) == "9223372036854775807");
    CHECK(printed(std::numeric_limits<int64_t>::min()) == "-9223372036854775808");
    CHECK(printed(int64_t{-1}) == "-1");
    CHECK(printed(uint64_t{4294967296ULL}) == "4294967296");
    CHECK(printed(uint64_t{1'000'000'000ULL}) == "1000000000");
    CHECK(printed(uint64_t{10'000'000'000'000'000'000ULL}) == "10000000000000000000");
    // The low nine digits keep their zeros under a non-zero top.
    CHECK(printed(uint64_t{5'000'000'000ULL}) == "5000000000");
    CHECK(printed(uint64_t{5'000'000'007ULL}) == "5000000007");
    CHECK(printed(int64_t{-5'000'000'007LL}) == "-5000000007");
    CHECK(printed("t=", uint64_t{123456789012ULL}, " us") == "t=123456789012 us");
    // Every 64-bit spelling the language has goes the same way.
    CHECK(printed(static_cast<long long>(-12345678901LL)) == "-12345678901");
    CHECK(printed(static_cast<unsigned long long>(12345678901ULL)) == "12345678901");
}

TEST_CASE("u64toa and i64toa match the standard library at every power of ten") {
    uint64_t p = 1;
    for (int k = 0; k <= 19; ++k) {
        // 10^19 is the last power that fits: past it, only p - 1, p and
        // p + 1 are numbers at all.
        const uint64_t top = k < 19 ? 9 * p + (p - 1) : p + 1;
        for (const uint64_t v : {p - 1, p, p + 1, k < 19 ? 2 * p - 1 : p, k < 19 ? 9 * p : p, top}) {
            CHECK(u64_text(v) == std::to_string(v));
            if (v <= static_cast<uint64_t>(std::numeric_limits<int64_t>::max())) {
                const int64_t s = static_cast<int64_t>(v);
                CHECK(i64_text(s) == std::to_string(s));
                CHECK(i64_text(-s) == std::to_string(-s));
            }
        }
        if (k < 19) {
            p *= 10;
        }
    }
}

TEST_CASE("the 2^32 edge the path splits on") {
    for (uint64_t v = 0xFFFFFF00ULL; v <= 0x1000000FFULL; ++v) {
        REQUIRE(u64_text(v) == std::to_string(v));
    }
}

TEST_CASE("a pseudo-random sweep over the whole range") {
    uint64_t x = 0x9E3779B97F4A7C15ULL;
    for (int i = 0; i < 200'000; ++i) {
        const uint64_t v = next(x);
        // Every magnitude, not only the top: shift a random amount down.
        const uint64_t w = v >> (v & 63u);
        REQUIRE(u64_text(v) == std::to_string(v));
        REQUIRE(u64_text(w) == std::to_string(w));
        const int64_t s = static_cast<int64_t>(w);
        REQUIRE(i64_text(s) == std::to_string(s));
        REQUIRE(i64_text(static_cast<int64_t>(v)) == std::to_string(static_cast<int64_t>(v)));
    }
}
