// Host tests for util/print.hpp: its integers - every width through
// print(), the divide by ten a decimal digit is made of against `/` and
// `%` over seventeen million values chosen for where its estimate can
// err, the two bases ultoa() converts with a constant divisor at
// their edges, and the 64-bit path's own two converters against the
// standard library's text, at every power of ten and its neighbours, the
// 2^32 edge the path splits on, both extremes and a pseudo-random sweep;
// hex() at both widths; its FLOATS - dtostrf() and dtostre(), the
// conversions every target but the AVR takes from this file, held
// against the host C library's snprintf ("%*.*f" and "%.*e" under each
// flag) over every binade of the float and of the double from 2^-204 to
// 2^128, random widths and precisions, the exact halves that round to
// even and the carries that add a digit, and the letters avr-libc prints
// for a NaN, an infinity and a value past the range -
// and its DELIVERY: a sink with the byte verb alone is fed a byte at a
// time, a BulkSink a run at a time - a C string's first byte through the
// byte verb before anything is measured - through refusals and short
// takes, with the same text either way. The host's `long` is 64 bits, so here
// every 64-bit type - long included - takes the explicit-width path the
// 32-bit targets and the AVR take for int64_t. Run with: ctest --preset
// host (or ctest --preset host -R test_print)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>

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

// ---- the float conversions against the host's C library ----------------

std::string fixed_text(double v, int width, int precision) {
    char buffer[400];
    return brio::dtostrf(v, static_cast<signed char>(width), static_cast<unsigned char>(precision),
                         buffer);
}

std::string libc_fixed(double v, int width, int precision) {
    char buffer[400];
    (void)snprintf(buffer, sizeof buffer, "%*.*f", width, precision, v);
    return buffer;
}

std::string sci_text(double v, int precision, unsigned flags) {
    char buffer[32];
    return brio::dtostre(v, buffer, static_cast<unsigned char>(precision),
                         static_cast<unsigned char>(flags));
}

/// snprintf's conversion under the flag dtostre() is given: '+' for
/// DTOSTR_PLUS_SIGN, ' ' for DTOSTR_ALWAYS_SIGN alone, 'E' for
/// DTOSTR_UPPERCASE - at the precision dtostre() keeps, 7 at most.
std::string libc_sci(double v, int precision, unsigned flags) {
    std::string format = "%";
    if ((flags & brio::DTOSTR_PLUS_SIGN) != 0u) {
        format += '+';
    } else if ((flags & brio::DTOSTR_ALWAYS_SIGN) != 0u) {
        format += ' ';
    }
    format += (flags & brio::DTOSTR_UPPERCASE) != 0u ? ".*E" : ".*e";
    char buffer[64];
    (void)snprintf(buffer, sizeof buffer, format.c_str(), precision > 7 ? 7 : precision, v);
    return buffer;
}

/// Mismatches counted over a sweep - one assertion for a million cases -
/// with the first one kept for the failure message.
struct Tally {
    uint64_t checked = 0;
    uint64_t wrong = 0;
    std::string first;
    void check(const std::string& ours, const std::string& libc, const std::string& what) {
        ++checked;
        if (ours != libc) {
            if (wrong++ == 0u) {
                first = what + ": ours [" + ours + "] libc [" + libc + "]";
            }
        }
    }
};

std::string describe(double v, int width, int precision) {
    char buffer[96];
    (void)snprintf(buffer, sizeof buffer, "%a width %d precision %d", v, width, precision);
    return buffer;
}

void check_fixed(Tally& t, double v, int width, int precision) {
    t.check(fixed_text(v, width, precision), libc_fixed(v, width, precision),
            describe(v, width, precision));
}

void check_sci(Tally& t, double v, int precision, unsigned flags) {
    t.check(sci_text(v, precision, flags), libc_sci(v, precision, flags),
            describe(v, static_cast<int>(flags), precision));
}

/// A float through BOTH entries: dtostrf() of it widened by the compiler,
/// and ftostrf() of its own bits - each against snprintf of the double the
/// float is.
void check_fixed(Tally& t, float f, int width, int precision) {
    check_fixed(t, static_cast<double>(f), width, precision);
    char buffer[400];
    t.check(brio::ftostrf(f, static_cast<signed char>(width), static_cast<unsigned char>(precision), buffer),
            libc_fixed(f, width, precision), "ftostrf " + describe(f, width, precision));
}

void check_sci(Tally& t, float f, int precision, unsigned flags) {
    check_sci(t, static_cast<double>(f), precision, flags);
    char buffer[32];
    t.check(brio::ftostre(f, buffer, static_cast<unsigned char>(precision), static_cast<unsigned char>(flags)),
            libc_sci(f, precision, flags), "ftostre " + describe(f, static_cast<int>(flags), precision));
}

/// A double with the given unbiased exponent and random mantissa and sign.
double random_double(uint64_t& x, int exponent) {
    const uint64_t bits = (next(x) & 0x800F'FFFF'FFFF'FFFFull) |
                          (static_cast<uint64_t>(exponent + 1023) << 52);
    return std::bit_cast<double>(bits);
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

// divmod10() at compile time: both ends of the range and the boundary of
// its one correction.
static_assert(brio::divmod10(0u).quot == 0u && brio::divmod10(0u).rem == 0u);
static_assert(brio::divmod10(9u).quot == 0u && brio::divmod10(9u).rem == 9u);
static_assert(brio::divmod10(10u).quot == 1u && brio::divmod10(10u).rem == 0u);
static_assert(brio::divmod10(0xFFFF'FFFFu).quot == 429'496'729u &&
              brio::divmod10(0xFFFF'FFFFu).rem == 5u);

TEST_CASE("divmod10 is exact: its estimate is never above the quotient nor short by more than one") {
    // One correction is enough only if the shift-and-add estimate lands on
    // floor(v / 10) or one below it (util/print.hpp states the bound).
    // An estimate short by two would leave a remainder of 20 or more and
    // the result one too low after the correction; one above would leave
    // a remainder that wraps and is "corrected" further off. Either shows
    // here as a quotient or a remainder unlike the compiler's own `/` and
    // `%`, so the sweep counts mismatches - one assertion for millions of
    // values. What it sweeps, and what each part reaches:
    //  - EVERY value below 2^20: every combination of the bits the first
    //    shifts drop (the low two of the value, the low four and eight of
    //    the running estimate) and every carry between them;
    //  - 10k - 1, 10k and 10k + 1 for every k below 2^20 and for k on a
    //    stride of 431 up to the top: the remainder's two ends, 9 (a short
    //    estimate leaves 19, the most the one correction must take back)
    //    and 0 (a short estimate leaves exactly 10, the comparison's edge);
    //  - every power of two and its neighbours: 2^j - 1 is a run of ones,
    //    where every truncation loses all it can at once;
    //  - a stride of 429 - coprime with ten and with two - over the whole
    //    range: ten million values, every residue, the high bits the
    //    sixteen-bit shift drops.
    // The whole 2^32 at the suite's -O0 would take over a minute.
    uint64_t checked = 0;
    uint64_t wrong = 0;
    const auto check = [&](uint32_t v) {
        const brio::DivMod d = brio::divmod10(v);
        if (d.quot != v / 10u || d.rem != v % 10u) {
            ++wrong;
        }
        ++checked;
    };
    for (uint32_t v = 0; v < (1u << 20); ++v) {
        check(v);
    }
    const auto tens = [&](uint32_t k) {
        const uint32_t t = k * 10u;
        check(t - 1u);   // k = 0 wraps to 0xFFFFFFFF: the top of the range
        check(t);
        check(t + 1u);
    };
    for (uint32_t k = 0; k < (1u << 20); ++k) {
        tens(k);
    }
    for (uint32_t k = 1u << 20; k <= 429'496'729u; k += 431u) {
        tens(k);
    }
    for (uint32_t j = 0; j < 32u; ++j) {
        const uint32_t p = 1u << j;
        check(p - 1u);
        check(p);
        check(p + 1u);
    }
    for (uint64_t v = 0; v <= 0xFFFF'FFFFull; v += 429u) {
        check(static_cast<uint32_t>(v));
    }
    CHECK(checked > 17'000'000u);
    CHECK(wrong == 0u);
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
    // This host's unsigned long is 64 bits: the digits above 32 bits are
    // peeled off before divmod10() takes the rest.
    if constexpr (sizeof(unsigned long) > sizeof(uint32_t)) {
        CHECK(std::string(brio::ultoa(4'294'967'296ul, buffer, 10)) == "4294967296");
        CHECK(std::string(brio::ultoa(10'000'000'009ul, buffer, 10)) == "10000000009");
        CHECK(std::string(brio::ultoa(std::numeric_limits<unsigned long>::max(), buffer, 10)) ==
              std::to_string(std::numeric_limits<unsigned long>::max()));
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

TEST_CASE("a C string reaches a BulkSink as its first byte, then in stretches, never measured whole") {
    std::string long_text(1000, 'x');
    BulkCapture::reset(1000, 0);
    brio::print(BulkCapture{}, long_text.c_str());
    CHECK(BulkCapture::text == long_text);
    CHECK(BulkCapture::bytes == 1u);   // the first byte, before anything is measured
    CHECK(BulkCapture::longest == brio::print_scan_run);
    CHECK(BulkCapture::runs == 16u);   // then fifteen stretches of 64 and the last 39
    // A string a byte and a stretch long ends on its NUL with no empty run.
    BulkCapture::reset(1000, 0);
    brio::print(BulkCapture{}, std::string(65, 'y').c_str());
    CHECK(BulkCapture::bytes == 1u);
    CHECK(BulkCapture::calls == 1u);
    CHECK(BulkCapture::text == std::string(65, 'y'));
    // A one-byte string is the byte verb alone.
    BulkCapture::reset(1000, 0);
    brio::print(BulkCapture{}, "z");
    CHECK(BulkCapture::bytes == 1u);
    CHECK(BulkCapture::calls == 0u);
    CHECK(BulkCapture::text == "z");
}

TEST_CASE("a BulkSink is handed a string_view as one run, a C string or a number as a byte and a run") {
    BulkCapture::reset(1000, 0);
    brio::print(BulkCapture{}, "hello, ", std::string_view("world"), ' ', 4294967295u, " ",
                brio::hex(0xBEEFu), " ", int64_t{-12345678901LL}, brio::crlf);
    CHECK(BulkCapture::text == "hello, world 4294967295 0xBEEF -12345678901\r\n");
    // The byte verb: the char, and the first byte of each of the six C
    // strings and numbers.
    CHECK(BulkCapture::bytes == 7u);
    // The runs: the five of those longer than a byte, after their first,
    // the string_view and the line end; the one-byte " " is a byte alone.
    CHECK(BulkCapture::runs == 6u);
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
    CHECK(BulkCapture::bytes == 1u);   // the C string's first
    CHECK(BulkCapture::refusals > 0u);
    // The rest of the C string in stretches of 64 (fifteen, then 39), each
    // taken in sevens: 10 runs a stretch and 6 for the last; the
    // string_view whole, 143 runs; the line end one.
    static_assert(brio::print_scan_run == 64u);
    CHECK(BulkCapture::runs == 15u * 10u + 6u + 143u + 1u);
    // An empty string is no call at all.
    BulkCapture::reset(7, 0);
    brio::print(BulkCapture{}, "", std::string_view());
    CHECK(BulkCapture::calls == 0u);
    CHECK(BulkCapture::text.empty());
}

// ---- hex() at 64 bits --------------------------------------------------------

// A 64-bit argument finds the 64-bit form; every narrower one, an int
// literal among them, still finds hex(uint32_t) and nothing else.
static_assert(std::is_same_v<decltype(brio::hex(0)), brio::Hex>);
static_assert(std::is_same_v<decltype(brio::hex(uint8_t{1})), brio::Hex>);
static_assert(std::is_same_v<decltype(brio::hex(uint32_t{1})), brio::Hex>);
static_assert(std::is_same_v<decltype(brio::hex(uint64_t{1})), brio::Hex64>);
static_assert(std::is_same_v<decltype(brio::hex(int64_t{-1})), brio::Hex64>);
static_assert(std::is_same_v<decltype(brio::hex(1ull)), brio::Hex64>);

TEST_CASE("hex() prints a 64-bit value whole: the high word, then the low one in all eight digits") {
    CHECK(printed(brio::hex(uint64_t{0})) == "0x0");
    CHECK(printed(brio::hex(uint64_t{0xBEEF})) == "0xBEEF");
    CHECK(printed(brio::hex(uint64_t{0xFFFF'FFFFu})) == "0xFFFFFFFF");
    CHECK(printed(brio::hex(uint64_t{1} << 32)) == "0x100000000");
    CHECK(printed(brio::hex(uint64_t{1} << 40)) == "0x10000000000");
    CHECK(printed(brio::hex(0x1000'0000'0000'000Full)) == "0x100000000000000F");
    CHECK(printed(brio::hex(0x1234'5678'9ABC'DEF0ull)) == "0x123456789ABCDEF0");
    CHECK(printed(brio::hex(std::numeric_limits<uint64_t>::max())) == "0xFFFFFFFFFFFFFFFF");
    CHECK(printed(brio::hex(int64_t{-1})) == "0xFFFFFFFFFFFFFFFF");
    // Against the standard library over every magnitude.
    uint64_t x = 0xD1B54A32D192ED03ULL;
    for (int i = 0; i < 100'000; ++i) {
        const uint64_t v = next(x);
        const uint64_t w = v >> (v & 63u);
        char expect[24];
        (void)snprintf(expect, sizeof expect, "0x%llX", static_cast<unsigned long long>(w));
        REQUIRE(printed(brio::hex(w)) == expect);
    }
}

// ---- the float conversions ---------------------------------------------------

TEST_CASE("dtostrf and ftostrf are snprintf's %*.*f for every float: each binade's edges, then random ones") {
    Tally t;
    // Every power of two a float can hold, from the smallest subnormal to
    // the largest binade, with its neighbours below and above, signed both
    // ways, at precisions 0 to 18 and a field each way.
    for (int k = -149; k <= 127; ++k) {
        const float p = std::ldexp(1.0f, k);
        for (const float v : {p, std::nextafter(p, 0.0f), std::nextafter(p, 1e38f)}) {
            for (int precision = 0; precision <= 18; ++precision) {
                check_fixed(t, v, 0, precision);
                check_fixed(t, -v, 12, precision);
                check_fixed(t, v, -12, precision);
            }
        }
    }
    check_fixed(t, std::numeric_limits<float>::max(), 0, 18);
    check_fixed(t, -std::numeric_limits<float>::max(), 50, 3);
    // Random floats - every exponent, every mantissa - at random fields
    // and precisions.
    uint64_t x = 0x9E3779B97F4A7C15ULL;
    for (int i = 0; i < 100'000; ++i) {
        const float v = std::bit_cast<float>(static_cast<uint32_t>(next(x)));
        if (!std::isfinite(v)) {
            continue;
        }
        const uint64_t r = next(x);
        check_fixed(t, v, static_cast<int>(r % 61u) - 30, static_cast<int>((r >> 8) % 19u));
    }
    CHECK(t.checked > 100'000u);
    INFO(t.first);
    CHECK(t.wrong == 0u);
}

TEST_CASE("dtostrf is snprintf's %*.*f for every double below 2^128, at any precision") {
    Tally t;
    uint64_t x = 0x2545F4914F6CDD1DULL;
    // From 2^-204 up every bit of the value is held: any precision, any
    // field, and the text grows past the field when it must.
    for (int i = 0; i < 60'000; ++i) {
        const int exponent = static_cast<int>(next(x) % 332u) - 204;
        const double v = random_double(x, exponent);
        const uint64_t r = next(x);
        const int precision = (r & 1u) != 0u ? static_cast<int>((r >> 1) % 19u) : static_cast<int>((r >> 1) % 256u);
        check_fixed(t, v, static_cast<int>((r >> 16) % 256u) - 128, precision);
    }
    // Below 2^-204 the bits under 2^-256 are cut, and no precision up to
    // 61 can see them: the text is still exact.
    for (int i = 0; i < 20'000; ++i) {
        const int exponent = -1022 + static_cast<int>(next(x) % 818u);
        check_fixed(t, random_double(x, exponent), 0, static_cast<int>(next(x) % 62u));
    }
    for (const double v : {std::numeric_limits<double>::denorm_min(), std::numeric_limits<double>::min(),
                           std::ldexp(1.0, 128) - std::ldexp(1.0, 75), 18446744073709551615.0,
                           1.0 / 3.0, 2.0 / 3.0, 0.1, 0.2, 0.3, 3.14159265358979, -273.15}) {
        for (int precision = 0; precision <= 61; ++precision) {
            check_fixed(t, v, 8, precision);
        }
    }
    CHECK(t.checked > 80'000u);
    INFO(t.first);
    CHECK(t.wrong == 0u);
}

TEST_CASE("an exact half rounds to even, and a carry adds a digit, as snprintf does") {
    // a / 2^j has exactly j decimals, so at j - 1 of them it is cut at an
    // exact half: the last digit kept decides.
    Tally t;
    uint64_t x = 0x6A09E667F3BCC909ULL;
    for (int j = 1; j <= 40; ++j) {
        for (int i = 0; i < 500; ++i) {
            const uint64_t a = (next(x) % (uint64_t{1} << 40)) | 1u;
            check_fixed(t, std::ldexp(static_cast<double>(a), -j), 0, j - 1);
        }
    }
    INFO(t.first);
    CHECK(t.wrong == 0u);
    CHECK(fixed_text(0.125, 0, 2) == "0.12");
    CHECK(fixed_text(0.375, 0, 2) == "0.38");
    CHECK(fixed_text(2.5, 0, 0) == "2");
    CHECK(fixed_text(3.5, 0, 0) == "4");
    CHECK(fixed_text(-0.5, 0, 0) == "-0");
    CHECK(fixed_text(9.5, 0, 0) == "10");
    CHECK(fixed_text(99.5, 4, 0) == " 100");
    CHECK(fixed_text(-999999.5, 0, 0) == "-1000000");
    CHECK(fixed_text(0.99951171875, 0, 3) == "1.000");   // 2047/2048, above the half
    CHECK(fixed_text(-0.0, 7, 3) == " -0.000");
    CHECK(fixed_text(0.0, -7, 0) == "0      ");
    CHECK(fixed_text(3.14159, 0, 0) == "3");
    CHECK(fixed_text(-1.0e-4, 0, 3) == "-0.000");   // the sign of a value that rounds to zero
}

TEST_CASE("dtostre and ftostre are snprintf's %.*e under each flag, from 2^-204 to 2^128") {
    Tally t;
    for (int k = -204; k <= 127; ++k) {
        const double p = std::ldexp(1.0, k);
        for (const double v : {p, std::nextafter(p, 0.0), -p}) {
            for (int precision = 0; precision <= 9; ++precision) {
                check_sci(t, v, precision, brio::DTOSTR_PLUS_SIGN);
                check_sci(t, v, precision, 0u);
            }
        }
    }
    for (int k = -149; k <= 127; ++k) {
        const float p = std::ldexp(1.0f, k);
        for (const float v : {p, std::nextafter(p, 0.0f), -std::nextafter(p, 1e38f)}) {
            for (int precision = 0; precision <= 8; ++precision) {
                check_sci(t, v, precision, brio::DTOSTR_PLUS_SIGN);
            }
        }
    }
    for (int k = -61; k <= 38; ++k) {
        const double v = std::pow(10.0, k);
        for (int precision = 0; precision <= 7; ++precision) {
            check_sci(t, v, precision, brio::DTOSTR_PLUS_SIGN);
            check_sci(t, -v, precision, brio::DTOSTR_ALWAYS_SIGN | brio::DTOSTR_UPPERCASE);
        }
    }
    uint64_t x = 0xBB67AE8584CAA73BULL;
    // Every float, and doubles over the whole range: every flag and the
    // precisions dtostre() keeps, one past them taken as 7.
    for (int i = 0; i < 60'000; ++i) {
        const float f = std::bit_cast<float>(static_cast<uint32_t>(next(x)));
        const uint64_t r = next(x);
        if (std::isfinite(f)) {
            check_sci(t, f, static_cast<int>(r % 9u), static_cast<unsigned>((r >> 8) % 8u));
        }
        const int exponent = static_cast<int>((r >> 16) % 332u) - 204;
        check_sci(t, random_double(x, exponent), static_cast<int>((r >> 32) % 9u),
                  static_cast<unsigned>((r >> 40) % 8u));
    }
    // Exact halves in the significant digits: integers ending in 5, cut
    // at the digit before it, and the carries of 9.5 and its kind.
    for (int i = 0; i < 20'000; ++i) {
        const uint64_t a = (next(x) % 100'000'000'000'000'000ULL) / 10u * 10u + 5u;
        const int digits = static_cast<int>(std::to_string(a).size());
        if (digits >= 2 && digits <= 9) {
            check_sci(t, static_cast<double>(a), digits - 2, brio::DTOSTR_PLUS_SIGN);
        }
        check_sci(t, static_cast<double>(a), static_cast<int>(a % 8u), 0u);
    }
    CHECK(t.checked > 150'000u);
    INFO(t.first);
    CHECK(t.wrong == 0u);
    CHECK(sci_text(0.0, 3, 0u) == "0.000e+00");
    CHECK(sci_text(-0.0, 3, brio::DTOSTR_PLUS_SIGN) == "-0.000e+00");
    CHECK(sci_text(0.0, 3, brio::DTOSTR_ALWAYS_SIGN) == " 0.000e+00");
    CHECK(sci_text(9.5, 0, 0u) == "1e+01");
    CHECK(sci_text(0.0012345, 0, 0u) == "1e-03");
    CHECK(sci_text(99999.5, 3, brio::DTOSTR_UPPERCASE) == "1.000E+05");
    CHECK(sci_text(1.0, 12, 0u) == "1.0000000e+00");   // precision taken as 7
}

TEST_CASE("NaN, the infinities and the values past 2^128 print the letters avr-libc prints") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    // dtostrf: upper case, in the field; a NaN carries no sign.
    CHECK(fixed_text(nan, 6, 2) == "   NAN");
    CHECK(fixed_text(-nan, 6, 2) == "   NAN");
    CHECK(fixed_text(nan, -6, 2) == "NAN   ");
    CHECK(fixed_text(inf, 0, 3) == "INF");
    CHECK(fixed_text(-inf, 6, 3) == "  -INF");
    CHECK(fixed_text(-inf, -6, 3) == "-INF  ");
    CHECK(fixed_text(std::ldexp(1.0, 128), 5, 1) == "  INF");
    CHECK(fixed_text(-1.0e300, 0, 1) == "-INF");
    CHECK(fixed_text(std::nextafter(std::ldexp(1.0, 128), 0.0), 0, 0) ==
          "340282366920938425684442744474606501888");   // the last value in range
    // dtostre: lower case unless DTOSTR_UPPERCASE; a NaN takes a sign only
    // from the flags, an infinity its own.
    CHECK(sci_text(nan, 3, 0u) == "nan");
    CHECK(sci_text(-nan, 3, 0u) == "nan");
    CHECK(sci_text(-nan, 3, brio::DTOSTR_PLUS_SIGN) == "+nan");
    CHECK(sci_text(nan, 3, brio::DTOSTR_ALWAYS_SIGN | brio::DTOSTR_UPPERCASE) == " NAN");
    CHECK(sci_text(inf, 3, 0u) == "inf");
    CHECK(sci_text(inf, 3, brio::DTOSTR_PLUS_SIGN) == "+inf");
    CHECK(sci_text(-inf, 3, brio::DTOSTR_PLUS_SIGN | brio::DTOSTR_UPPERCASE) == "-INF");
    CHECK(sci_text(1.0e39, 3, brio::DTOSTR_PLUS_SIGN) == "+inf");
    CHECK(sci_text(-std::numeric_limits<double>::max(), 3, 0u) == "-inf");
    // The float entries print the same letters.
    char buffer[32];
    CHECK(std::string(brio::ftostrf(std::numeric_limits<float>::quiet_NaN(), 5, 1, buffer)) == "  NAN");
    CHECK(std::string(brio::ftostrf(-std::numeric_limits<float>::infinity(), -5, 1, buffer)) == "-INF ");
    CHECK(std::string(brio::ftostre(std::numeric_limits<float>::infinity(), buffer, 2, brio::DTOSTR_PLUS_SIGN)) == "+inf");
    CHECK(std::string(brio::ftostre(-std::numeric_limits<float>::quiet_NaN(), buffer, 2, 0u)) == "nan");
    // Below 2^-256 a double reads as zero.
    CHECK(sci_text(std::ldexp(1.0, -300), 3, brio::DTOSTR_PLUS_SIGN) == "+0.000e+00");
    CHECK(sci_text(std::numeric_limits<double>::denorm_min(), 1, 0u) == "0.0e+00");
}

TEST_CASE("print routes fixed(), sci() and a bare float through the two conversions") {
    CHECK(printed(brio::fixed(3.14159f, 8, 3)) == "   3.142");
    // 3.1415f is 3.14149999618530273...: the float's own digits, rounded
    // once.
    CHECK(printed(brio::fixed(3.1415f, 8, 3)) == "   3.141");
    CHECK(printed(brio::fixed(-2.5f, -6, 1)) == "-2.5  ");
    CHECK(printed(brio::sci(0.00123f, 3)) == "+1.230e-03");
    CHECK(printed(brio::sci(-12345.0, 2)) == "-1.23e+04");
    CHECK(printed(1.5) == "+1.500e+00");
    CHECK(printed(2.5f) == "+2.500e+00");
    CHECK(printed("v=", brio::fixed(0.5, 0, 0), " w=", brio::fixed(1.5, 0, 0)) == "v=0 w=2");
    // The value travels as a double: a double keeps its digits, a float
    // prints the float it is.
    CHECK(printed(brio::fixed(0.1, 0, 17)) == "0.10000000000000001");
    CHECK(printed(brio::fixed(0.1f, 0, 17)) == "0.10000000149011612");
    // The precision is kept to print_fixed_precision_max and the field to
    // the buffer, either way.
    CHECK(printed(brio::fixed(1.0, 0, 30)) == "1." + std::string(brio::print_fixed_precision_max, '0'));
    CHECK(printed(brio::fixed(1.0, 127, 0)) == std::string(brio::print_fixed_text - 2u, ' ') + "1");
    CHECK(printed(brio::fixed(1.0, -128, 0)) == "1" + std::string(brio::print_fixed_text - 2u, ' '));
    CHECK(printed(brio::fixed(-std::numeric_limits<float>::max(), 0, 18)).size() ==
          brio::print_fixed_text - 1u);   // the widest text there is fills the buffer
    CHECK(printed(brio::sci(1.0, 12)) == "+1.0000000e+00");
}
