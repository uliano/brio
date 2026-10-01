// Host tests for rt/rt.cpp, the runtime every 32-bit image compiles: its
// memcpy, memmove, memset and memcmp against a byte-at-a-time reference,
// EXHAUSTIVELY over what decides their path - the misalignment of each
// pointer (0..7, twice the word, so every residue meets every other),
// every length from nothing up to several word blocks plus a tail, every
// overlap in both directions for memmove - with guard bytes around every
// destination, so a write one byte outside the range is caught.
// The file under test is INCLUDED, compiled exactly as the targets compile
// it; BRIO_RT_HOST_TEST leaves out its C-linkage definitions, which would
// otherwise replace this host's own C library.
// Run with: ctest --preset host (or ctest --preset host -R test_rt)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#define BRIO_RT_HOST_TEST
#include "rt/rt.cpp"
#include "rt/selftest.hpp"

#include <cstring>

#include <array>
#include <cstddef>
#include <cstdint>

namespace {

constexpr size_t max_offset = 8;    // twice the word: every residue pair
constexpr size_t max_length = 130;  // several word blocks, every tail
constexpr size_t guard = 16;        // bytes checked on each side
constexpr size_t buffer_bytes = guard + max_offset + max_length + guard;
constexpr uint8_t guard_byte = 0xA5u;

// Word-aligned storage, so that an offset IS the misalignment.
struct alignas(8) Buffer {
    std::array<uint8_t, buffer_bytes> b{};
};

// A pattern that no two positions share within a copy, so a byte taken
// from the wrong place cannot pass for the right one.
uint8_t pattern(size_t i) {
    return static_cast<uint8_t>((i * 7u + 13u) ^ (i >> 3));
}

void fill_guard(Buffer& buf) {
    for (auto& x : buf.b) {
        x = guard_byte;
    }
}

// Every byte outside [first, first + n) still holds the guard.
bool guards_intact(const Buffer& buf, size_t first, size_t n) {
    for (size_t i = 0; i < buffer_bytes; ++i) {
        if ((i < first || i >= first + n) && buf.b[i] != guard_byte) {
            return false;
        }
    }
    return true;
}

}  // namespace

TEST_CASE("memcpy: every misalignment of both pointers, every length") {
    Buffer src;
    Buffer dst;
    for (size_t i = 0; i < buffer_bytes; ++i) {
        src.b[i] = pattern(i);
    }
    size_t cases = 0;
    for (size_t so = 0; so < max_offset; ++so) {
        for (size_t dof = 0; dof < max_offset; ++dof) {
            for (size_t n = 0; n <= max_length; ++n) {
                fill_guard(dst);
                const size_t d0 = guard + dof;
                const size_t s0 = guard + so;
                void* r = rt_memcpy(&dst.b[d0], &src.b[s0], n);
                REQUIRE(r == &dst.b[d0]);
                for (size_t i = 0; i < n; ++i) {
                    REQUIRE(dst.b[d0 + i] == src.b[s0 + i]);
                }
                REQUIRE(guards_intact(dst, d0, n));
                ++cases;
            }
        }
    }
    CHECK(cases == max_offset * max_offset * (max_length + 1));
}

TEST_CASE("memmove: every overlap in both directions, every length") {
    // One buffer; source and destination both inside it, from disjoint to
    // coincident, with the destination below, equal to and above the
    // source. The expected result is computed through a separate copy.
    constexpr size_t span = 24;  // distances between the two starts
    constexpr size_t len_max = 70;
    constexpr size_t bytes = guard + span + len_max + span + guard;
    struct alignas(8) Big {
        std::array<uint8_t, bytes> b{};
    };
    size_t cases = 0;
    for (size_t so = 0; so <= 2 * span; so += 1) {
        for (size_t dof = 0; dof <= 2 * span; dof += 1) {
            for (size_t n = 0; n <= len_max; n += (n < 20 ? 1 : 7)) {
                if (so + n > 2 * span + len_max || dof + n > 2 * span + len_max) {
                    continue;
                }
                Big buf;
                for (size_t i = 0; i < bytes; ++i) {
                    buf.b[i] = pattern(i);
                }
                Big expect = buf;
                std::array<uint8_t, len_max> tmp{};
                for (size_t i = 0; i < n; ++i) {
                    tmp[i] = expect.b[guard + so + i];
                }
                for (size_t i = 0; i < n; ++i) {
                    expect.b[guard + dof + i] = tmp[i];
                }
                void* r = rt_memmove(&buf.b[guard + dof], &buf.b[guard + so], n);
                REQUIRE(r == &buf.b[guard + dof]);
                REQUIRE(buf.b == expect.b);
                ++cases;
            }
        }
    }
    CHECK(cases > 30000);
}

TEST_CASE("memset: every misalignment, every length, every kind of value") {
    Buffer dst;
    for (int c : {0x00, 0xFF, 0x80, 0x5A, 0x1FF, -1}) {
        const auto v = static_cast<uint8_t>(c);  // memset stores (unsigned char)c
        for (size_t off = 0; off < max_offset; ++off) {
            for (size_t n = 0; n <= max_length; ++n) {
                fill_guard(dst);
                const size_t d0 = guard + off;
                void* r = rt_memset(&dst.b[d0], c, n);
                REQUIRE(r == &dst.b[d0]);
                for (size_t i = 0; i < n; ++i) {
                    REQUIRE(dst.b[d0 + i] == v);
                }
                REQUIRE(guards_intact(dst, d0, n));
            }
        }
    }
}

TEST_CASE("memcmp: zero on equal ranges, the sign of the first difference as unsigned") {
    Buffer a;
    Buffer b;
    for (size_t i = 0; i < buffer_bytes; ++i) {
        a.b[i] = pattern(i);
        b.b[i] = pattern(i);
    }
    for (size_t off = 0; off < max_offset; ++off) {
        for (size_t n = 0; n <= 40; ++n) {
            REQUIRE(rt_memcmp(&a.b[guard + off], &b.b[guard + off], n) == 0);
        }
    }
    // A difference at every position, both orders, across the sign bit:
    // 0x01 against 0xFF is "less" read as unsigned char, "more" as signed.
    for (size_t k = 0; k < 40; ++k) {
        Buffer x = a;
        Buffer y = a;
        x.b[guard + k] = 0x01u;
        y.b[guard + k] = 0xFFu;
        CHECK(rt_memcmp(&x.b[guard], &y.b[guard], 40) < 0);
        CHECK(rt_memcmp(&y.b[guard], &x.b[guard], 40) > 0);
        // A difference past the compared range is not seen.
        CHECK(rt_memcmp(&x.b[guard], &y.b[guard], k) == 0);
    }
}

TEST_CASE("strlen: every length at every misalignment") {
    Buffer s;
    for (size_t off = 0; off < max_offset; ++off) {
        for (size_t n = 0; n <= max_length; ++n) {
            fill_guard(s);
            char* p = reinterpret_cast<char*>(&s.b[guard + off]);
            for (size_t i = 0; i < n; ++i) {
                p[i] = static_cast<char>(0x80u | (pattern(i) & 0x7Fu));  // high bytes too
            }
            p[n] = '\0';
            REQUIRE(rt_strlen(p) == n);
        }
    }
}

TEST_CASE("memchr: every position, absent values, unsigned char comparison, the bound") {
    Buffer s;
    for (size_t i = 0; i < buffer_bytes; ++i) {
        s.b[i] = static_cast<uint8_t>(i & 0x3Fu);  // values 0..63, repeating
    }
    // One byte above 0x7F, found as (unsigned char)c - placed past every
    // 64-byte window below, which must hold all of 0..63.
    s.b[guard + 100] = 0xC3u;
    for (size_t off = 0; off < max_offset; ++off) {
        uint8_t* base = &s.b[guard + off];
        // Every value 0..63 is found at its first occurrence.
        for (uint8_t v = 0; v < 64u; ++v) {
            void* r = rt_memchr(base, v, 64);
            REQUIRE(r != nullptr);
            REQUIRE(*static_cast<uint8_t*>(r) == v);
            for (uint8_t* q = base; q < static_cast<uint8_t*>(r); ++q) {
                REQUIRE(*q != v);
            }
        }
        // A value outside the range is not found, and the bound is honoured.
        REQUIRE(rt_memchr(base, 0x40, max_length) == nullptr);
        REQUIRE(rt_memchr(base, base[10], 10) == (base[10] < 10u ? static_cast<void*>(base + base[10]) : nullptr));
        REQUIRE(rt_memchr(base, 0x00, 0) == nullptr);
    }
    // c is converted to unsigned char: -61 and 0x1C3 both name 0xC3.
    REQUIRE(rt_memchr(&s.b[guard], -61, 110) == &s.b[guard + 100]);
    REQUIRE(rt_memchr(&s.b[guard], 0x1C3, 110) == &s.b[guard + 100]);
    REQUIRE(rt_memchr(&s.b[guard], -61, 100) == nullptr);  // one short of it
}

// The cases every 32-bit platform suite runs on its chip (rt/selftest.hpp),
// run here twice: against rt.cpp's implementations, and against this host's
// own C library - a library known to be right, so a failure there would be
// the cases' fault and not the runtime's.
namespace {

void check_all(const brio::RtFunctions& fns) {
    using namespace brio;
    const RtSelftestResult rs[] = {
        rt_selftest_memcpy(fns), rt_selftest_memmove(fns), rt_selftest_memset(fns),
        rt_selftest_memcmp(fns), rt_selftest_strlen(fns), rt_selftest_memchr(fns),
    };
    for (const auto& r : rs) {
        INFO("first failure: " << (r.first_what ? r.first_what : "-") << " " << r.first_a << " " << r.first_b);
        CHECK(r.cases > 0u);
        CHECK(r.failures == 0u);
    }
}

}  // namespace

TEST_CASE("rt/selftest.hpp: the chip's cases pass on rt.cpp's implementations") {
    check_all(brio::RtFunctions{&rt_memcpy, &rt_memmove, &rt_memset, &rt_memcmp,
                                &rt_strlen, &rt_memchr});
}

TEST_CASE("rt/selftest.hpp: the chip's cases pass on the host's own C library") {
    check_all(brio::RtFunctions{
        static_cast<void* (*)(void*, const void*, size_t)>(&::memcpy),
        static_cast<void* (*)(void*, const void*, size_t)>(&::memmove),
        static_cast<void* (*)(void*, int, size_t)>(&::memset),
        static_cast<int (*)(const void*, const void*, size_t)>(&::memcmp),
        static_cast<size_t (*)(const char*)>(&::strlen),
        &brio::rt_memchr});
}

TEST_CASE("rt/selftest.hpp: a broken copy is caught") {
    // A memcpy that drops the last byte of every copy longer than four:
    // the cases must see it - a check that cannot fail proves nothing.
    auto broken = [](void* d, const void* s, size_t n) -> void* {
        return rt_memcpy(d, s, n > 4u ? n - 1u : n);
    };
    brio::RtFunctions fns{+broken, &rt_memmove, &rt_memset, &rt_memcmp, &rt_strlen, &rt_memchr};
    const auto r = brio::rt_selftest_memcpy(fns);
    CHECK(r.failures > 0u);
}
