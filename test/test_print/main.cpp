// Host tests for util/print.hpp: its integers - every width through
// print(), the two bases ultoa() converts with a constant divisor at
// their edges, and the 64-bit path's own two converters against the
// standard library's text, at every power of ten and its neighbours, the
// 2^32 edge the path splits on, both extremes and a pseudo-random sweep -
// and its DELIVERY: a sink with the byte verb alone is fed a byte at a
// time, a BulkSink a run at a time, through refusals and short takes,
// with the same text either way. The host's `long` is 64 bits, so here
// every 64-bit type - long included - takes the explicit-width path the
// 32-bit targets and the AVR take for int64_t. Run with: ctest --preset
// host (or ctest --preset host -R test_print)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>
#include <cstdio>
#include <limits>
#include <span>
#include <string>
#include <string_view>

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
static_assert(!brio::BulkSink<Capture>);

/// A sink that takes RUNS the way a transport's ring does: at most
/// `room` bytes a call, and every `refuse_every`-th call nothing at all -
/// the full ring a print spins on. Its byte verb counts, so a test sees
/// which path print took.
struct BulkCapture {
    static inline std::string text;
    static inline uint32_t bytes = 0;      // write_byte() calls
    static inline uint32_t runs = 0;       // write_bulk() calls that took something
    static inline uint32_t refusals = 0;   // write_bulk() calls that took nothing
    static inline uint32_t room = 1000;
    static inline uint32_t refuse_every = 0;
    static inline uint32_t calls = 0;
    static inline size_t longest = 0;      // the longest run ever offered
    static void reset(uint32_t r, uint32_t every) {
        text.clear();
        bytes = runs = refusals = calls = 0;
        longest = 0;
        room = r;
        refuse_every = every;
    }
    static bool write_byte(uint8_t b) {
        ++bytes;
        text.push_back(static_cast<char>(b));
        return true;
    }
    static uint32_t write_bulk(std::span<const uint8_t> run) {
        ++calls;
        longest = run.size() > longest ? run.size() : longest;
        if (refuse_every != 0u && calls % refuse_every == 0u) {
            ++refusals;
            return 0;
        }
        const uint32_t take = run.size() < room ? static_cast<uint32_t>(run.size()) : room;
        text.append(reinterpret_cast<const char*>(run.data()), take);
        ++runs;
        return take;
    }
};
static_assert(brio::BulkSink<BulkCapture>);

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

TEST_CASE("ultoa converts both bases with a constant divisor, at their edges") {
    char buffer[24];
    CHECK(std::string(brio::ultoa(0ul, buffer, 10)) == "0");
    CHECK(std::string(brio::ultoa(9ul, buffer, 10)) == "9");
    CHECK(std::string(brio::ultoa(10ul, buffer, 10)) == "10");
    CHECK(std::string(brio::ultoa(99ul, buffer, 10)) == "99");
    CHECK(std::string(brio::ultoa(100ul, buffer, 10)) == "100");
    CHECK(std::string(brio::ultoa(0xFFFFFFFFul, buffer, 10)) == "4294967295");
    CHECK(std::string(brio::ultoa(0ul, buffer, 16)) == "0");
    CHECK(std::string(brio::ultoa(9ul, buffer, 16)) == "9");
    CHECK(std::string(brio::ultoa(10ul, buffer, 16)) == "A");
    CHECK(std::string(brio::ultoa(15ul, buffer, 16)) == "F");
    CHECK(std::string(brio::ultoa(16ul, buffer, 16)) == "10");
    CHECK(std::string(brio::ultoa(0xFFFFFFFFul, buffer, 16)) == "FFFFFFFF");
    CHECK(std::string(brio::ultoa(0xBEEFul, buffer, 16)) == "BEEF");
    // Every value of a sweep against the standard library, both bases.
    uint64_t x = 0x2545F4914F6CDD1DULL;
    for (int i = 0; i < 100'000; ++i) {
        const uint64_t v = next(x);
        const unsigned long w = static_cast<unsigned long>((v >> (v & 31u)) & 0xFFFFFFFFu);
        REQUIRE(std::string(brio::ultoa(w, buffer, 10)) == std::to_string(w));
        char expect[24];
        (void)snprintf(expect, sizeof expect, "%lX", w);
        REQUIRE(std::string(brio::ultoa(w, buffer, 16)) == expect);
    }
    CHECK(std::string(brio::ltoa(-1l, buffer, 10)) == "-1");
    CHECK(std::string(brio::ltoa(-2147483648l, buffer, 10)) == "-2147483648");
    CHECK(printed(brio::hex(0)) == "0x0");
    CHECK(printed(brio::hex(9)) == "0x9");
    CHECK(printed(brio::hex(10)) == "0xA");
    CHECK(printed(brio::hex(0xFFFFFFFFu)) == "0xFFFFFFFF");
}

TEST_CASE("the 64-bit path's low nine digits go through ultoa, zero-padded") {
    // Above 2^32 every value's last nine digits are ultoa()'s: zeros kept,
    // each digit position from 0 to 9 at the edge of its range.
    CHECK(u64_text(4'294'967'296ULL) == "4294967296");
    CHECK(u64_text(4'000'000'000'000'000'009ULL) == "4000000000000000009");
    CHECK(u64_text(4'999'999'999'999'999'999ULL) == "4999999999999999999");
    CHECK(u64_text(10'000'000'010ULL) == "10000000010");
    CHECK(u64_text(10'999'999'990ULL) == "10999999990");
    CHECK(u64_text(12'000'000'000ULL) == "12000000000");
    for (uint64_t low : {0ULL, 9ULL, 10ULL, 99ULL, 100'000'000ULL, 999'999'999ULL}) {
        const uint64_t v = 7'000'000'000ULL + low;
        CHECK(u64_text(v) == std::to_string(v));
    }
}

TEST_CASE("a sink with the byte verb alone is fed a byte at a time") {
    CHECK(printed("abc", std::string_view("de"), 'f', 123, brio::crlf) == "abcdef123\r\n");
}

TEST_CASE("a C string reaches a BulkSink in stretches, never measured whole before its first byte") {
    std::string long_text(1000, 'x');
    BulkCapture::reset(1000, 0);
    brio::print(BulkCapture{}, long_text.c_str());
    CHECK(BulkCapture::text == long_text);
    CHECK(BulkCapture::longest == brio::print_scan_run);
    CHECK(BulkCapture::runs == 16u);   // fifteen stretches of 64 and the last 40
    // A string exactly a stretch long ends on its NUL with no empty run.
    BulkCapture::reset(1000, 0);
    brio::print(BulkCapture{}, std::string(64, 'y').c_str());
    CHECK(BulkCapture::calls == 1u);
    CHECK(BulkCapture::text == std::string(64, 'y'));
}

TEST_CASE("a BulkSink is handed every string and every number as one run") {
    BulkCapture::reset(1000, 0);
    brio::print(BulkCapture{}, "hello, ", std::string_view("world"), ' ', 4294967295u, " ",
                brio::hex(0xBEEFu), " ", int64_t{-12345678901LL}, brio::crlf);
    CHECK(BulkCapture::text == "hello, world 4294967295 0xBEEF -12345678901\r\n");
    CHECK(BulkCapture::bytes == 1u);   // the char alone takes the byte verb
    CHECK(BulkCapture::runs == 8u);    // every other argument one run
    CHECK(BulkCapture::refusals == 0u);
}

TEST_CASE("print spins on a BulkSink through refusals and short takes and loses nothing") {
    std::string long_text;
    for (int i = 0; i < 1000; ++i) {
        long_text.push_back(static_cast<char>('a' + i % 26));
    }
    // Seven bytes a call, every third call refused: the full ring a
    // print waits on, and a ring whose room is never the whole run.
    BulkCapture::reset(7, 3);
    brio::print(BulkCapture{}, long_text.c_str(), std::string_view(long_text), brio::crlf);
    CHECK(BulkCapture::text == long_text + long_text + "\r\n");
    CHECK(BulkCapture::bytes == 0u);
    CHECK(BulkCapture::refusals > 0u);
    // The C string in stretches of 64 (fifteen, then 40), each taken in
    // sevens: 10 runs a stretch and 6 for the last; the string_view whole,
    // 143 runs; the line end one.
    static_assert(brio::print_scan_run == 64u);
    CHECK(BulkCapture::runs == 15u * 10u + 6u + 143u + 1u);
    // An empty string is no call at all.
    BulkCapture::reset(7, 0);
    brio::print(BulkCapture{}, "", std::string_view());
    CHECK(BulkCapture::calls == 0u);
    CHECK(BulkCapture::text.empty());
}
