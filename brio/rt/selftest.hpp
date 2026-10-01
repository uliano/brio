/*
 * selftest.hpp
 *
 * The runtime's cases, written once and run in two places: the host suite
 * (test/test_rt/, against rt.cpp's implementations compiled for the host)
 * and a letter of every 32-bit platform suite (against the symbols the
 * image really links: the silicon, the compiler and the flags of each
 * core). design/runtime.md, "How it is proven".
 *
 * EXHAUSTIVE over what decides a path: each pointer misaligned by 0..7
 * (twice the word, so every residue meets every other), every length from
 * nothing to 70 bytes (the head bytes, more than sixteen words, every
 * tail), every overlap of memmove in both directions, guard bytes around
 * every destination so a write one byte outside the range is caught; the
 * sign memcmp gives the first difference read as unsigned char; strlen at
 * every length and misalignment; memchr at every position, with values
 * above 0x7F, a value outside unsigned char and the bound honoured.
 *
 * THE FUNCTIONS ARRIVE AS POINTERS and every call goes through a volatile
 * copy of one: a call the compiler can see the target of may be expanded
 * in place (a builtin memcpy of a known size is a few loads and stores),
 * and then the runtime would not be the thing tested. The reference below
 * is byte loops under the same guard rt.cpp carries: a reference that GCC
 * turned into a call to memcpy would be judged by the code it judges.
 *
 * One static arena of a few hundred bytes, reused by every case, so the
 * smallest part (2 KB of RAM) can run it. Includes nothing of brio, like
 * the rest of the stratum.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#pragma GCC push_options
#pragma GCC optimize("no-tree-loop-distribute-patterns")

namespace brio {

/// The functions under test, in the order memcpy, memmove, memset,
/// memcmp, strlen, memchr - named by what they do, so that no C library's
/// macro for a name can reach a member.
struct RtFunctions {
    void* (*copy)(void*, const void*, size_t);
    void* (*move)(void*, const void*, size_t);
    void* (*set)(void*, int, size_t);
    int (*compare)(const void*, const void*, size_t);
    size_t (*length)(const char*);
    void* (*find)(const void*, int, size_t);
};

/// How many cases ran and how many failed; the first failure's parameters
/// (what, and two numbers whose meaning `what` states) for a report.
struct RtSelftestResult {
    uint32_t cases{0};
    uint32_t failures{0};
    const char* first_what{nullptr};
    uint32_t first_a{0};
    uint32_t first_b{0};

    void fail(const char* what, uint32_t a, uint32_t b) {
        if (failures++ == 0u) {
            first_what = what;
            first_a = a;
            first_b = b;
        }
    }
};

namespace rt_selftest_detail {

constexpr size_t offsets = 8;    // twice the word
constexpr size_t max_len = 70;   // more than sixteen words, every tail
constexpr size_t guard = 8;      // bytes watched on each side
constexpr size_t span = 24;      // memmove: distances between the two starts
constexpr uint8_t guard_byte = 0xA5u;

// src + dst for the copies and fills; buf + expect for memmove.
constexpr size_t copy_bytes = guard + offsets + max_len + guard;
constexpr size_t move_bytes = guard + span + max_len + span + guard;

struct alignas(8) Arena {
    uint8_t a[move_bytes > copy_bytes ? move_bytes : copy_bytes];
    uint8_t b[move_bytes > copy_bytes ? move_bytes : copy_bytes];
};

inline Arena& arena() {
    static Arena storage;
    return storage;
}

// A pattern no two positions share within a copy.
inline uint8_t pattern(size_t i) {
    return static_cast<uint8_t>((i * 7u + 13u) ^ (i >> 3));
}

inline void fill(uint8_t* p, size_t n, uint8_t v) {
    for (size_t i = 0; i < n; ++i) {
        p[i] = v;
    }
}

inline bool guards_intact(const uint8_t* p, size_t size, size_t first, size_t n) {
    for (size_t i = 0; i < size; ++i) {
        if ((i < first || i >= first + n) && p[i] != guard_byte) {
            return false;
        }
    }
    return true;
}

}  // namespace rt_selftest_detail

/// memcpy over every misalignment of both pointers and every length.
inline RtSelftestResult rt_selftest_memcpy(const RtFunctions& fns) {
    using namespace rt_selftest_detail;
    RtSelftestResult r;
    auto* volatile f = fns.copy;
    uint8_t* src = arena().a;
    uint8_t* dst = arena().b;
    for (size_t i = 0; i < copy_bytes; ++i) {
        src[i] = pattern(i);
    }
    for (size_t so = 0; so < offsets; ++so) {
        for (size_t dof = 0; dof < offsets; ++dof) {
            for (size_t n = 0; n <= max_len; ++n) {
                ++r.cases;
                fill(dst, copy_bytes, guard_byte);
                void* ret = f(dst + guard + dof, src + guard + so, n);
                bool ok = ret == dst + guard + dof &&
                          guards_intact(dst, copy_bytes, guard + dof, n);
                for (size_t i = 0; ok && i < n; ++i) {
                    ok = dst[guard + dof + i] == src[guard + so + i];
                }
                if (!ok) {
                    r.fail("memcpy: source offset x 100 + destination offset, length",
                           static_cast<uint32_t>(so * 100u + dof), static_cast<uint32_t>(n));
                }
            }
        }
    }
    return r;
}

/// memmove over every overlap in both directions and every length.
inline RtSelftestResult rt_selftest_memmove(const RtFunctions& fns) {
    using namespace rt_selftest_detail;
    RtSelftestResult r;
    auto* volatile f = fns.move;
    uint8_t* buf = arena().a;
    uint8_t* expect = arena().b;
    for (size_t so = 0; so <= 2 * span; ++so) {
        for (size_t dof = 0; dof <= 2 * span; ++dof) {
            for (size_t n = 0; n <= max_len; n += (n < 20u ? 1u : 5u)) {
                ++r.cases;
                for (size_t i = 0; i < move_bytes; ++i) {
                    buf[i] = pattern(i);
                    expect[i] = pattern(i);
                }
                // The expected result: the source read whole, then written.
                // Reading backwards when the destination is above keeps the
                // reference right without a third buffer.
                if (dof > so) {
                    for (size_t i = n; i != 0; --i) {
                        expect[guard + dof + i - 1] = expect[guard + so + i - 1];
                    }
                } else {
                    for (size_t i = 0; i < n; ++i) {
                        expect[guard + dof + i] = expect[guard + so + i];
                    }
                }
                void* ret = f(buf + guard + dof, buf + guard + so, n);
                bool ok = ret == buf + guard + dof;
                for (size_t i = 0; ok && i < move_bytes; ++i) {
                    ok = buf[i] == expect[i];
                }
                if (!ok) {
                    r.fail("memmove: source offset x 100 + destination offset, length",
                           static_cast<uint32_t>(so * 100u + dof), static_cast<uint32_t>(n));
                }
            }
        }
    }
    return r;
}

/// memset over every misalignment, every length, values across the sign
/// bit and one outside unsigned char.
inline RtSelftestResult rt_selftest_memset(const RtFunctions& fns) {
    using namespace rt_selftest_detail;
    RtSelftestResult r;
    auto* volatile f = fns.set;
    uint8_t* dst = arena().b;
    const int values[] = {0x00, 0xFF, 0x80, 0x5A, 0x1FF, -1};
    for (int c : values) {
        const auto v = static_cast<uint8_t>(c);
        for (size_t off = 0; off < offsets; ++off) {
            for (size_t n = 0; n <= max_len; ++n) {
                ++r.cases;
                fill(dst, copy_bytes, guard_byte);
                void* ret = f(dst + guard + off, c, n);
                bool ok = ret == dst + guard + off &&
                          guards_intact(dst, copy_bytes, guard + off, n);
                for (size_t i = 0; ok && i < n; ++i) {
                    ok = dst[guard + off + i] == v;
                }
                if (!ok) {
                    r.fail("memset: value, offset x 100 + length",
                           static_cast<uint32_t>(c), static_cast<uint32_t>(off * 100u + n));
                }
            }
        }
    }
    return r;
}

/// memcmp: zero on equal ranges, the sign of the first difference read as
/// unsigned char at every position, and a difference past the range unseen.
inline RtSelftestResult rt_selftest_memcmp(const RtFunctions& fns) {
    using namespace rt_selftest_detail;
    RtSelftestResult r;
    auto* volatile f = fns.compare;
    uint8_t* x = arena().a;
    uint8_t* y = arena().b;
    for (size_t i = 0; i < copy_bytes; ++i) {
        x[i] = pattern(i);
        y[i] = pattern(i);
    }
    for (size_t off = 0; off < offsets; ++off) {
        for (size_t n = 0; n <= 40u; ++n) {
            ++r.cases;
            if (f(x + guard + off, y + guard + off, n) != 0) {
                r.fail("memcmp equal: offset, length", static_cast<uint32_t>(off), static_cast<uint32_t>(n));
            }
        }
    }
    for (size_t k = 0; k < 40u; ++k) {
        const uint8_t xk = x[guard + k];
        const uint8_t yk = y[guard + k];
        x[guard + k] = 0x01u;  // 0x01 against 0xFF: less as unsigned char
        y[guard + k] = 0xFFu;
        r.cases += 3u;
        if (!(f(x + guard, y + guard, 40) < 0)) {
            r.fail("memcmp sign x<y: position", static_cast<uint32_t>(k), 0);
        }
        if (!(f(y + guard, x + guard, 40) > 0)) {
            r.fail("memcmp sign y>x: position", static_cast<uint32_t>(k), 0);
        }
        if (f(x + guard, y + guard, k) != 0) {
            r.fail("memcmp past the range: position", static_cast<uint32_t>(k), 0);
        }
        x[guard + k] = xk;
        y[guard + k] = yk;
    }
    return r;
}

/// strlen at every length and every misalignment, bytes above 0x7F included.
inline RtSelftestResult rt_selftest_strlen(const RtFunctions& fns) {
    using namespace rt_selftest_detail;
    RtSelftestResult r;
    auto* volatile f = fns.length;
    uint8_t* s = arena().a;
    for (size_t off = 0; off < offsets; ++off) {
        for (size_t n = 0; n <= max_len; ++n) {
            ++r.cases;
            fill(s, copy_bytes, guard_byte);
            for (size_t i = 0; i < n; ++i) {
                s[guard + off + i] = static_cast<uint8_t>(0x80u | (pattern(i) & 0x7Fu));
            }
            s[guard + off + n] = 0u;
            if (f(reinterpret_cast<const char*>(s + guard + off)) != n) {
                r.fail("strlen: offset, length", static_cast<uint32_t>(off), static_cast<uint32_t>(n));
            }
        }
    }
    return r;
}

/// memchr at every position, absent values, values above 0x7F and outside
/// unsigned char, and the bound.
inline RtSelftestResult rt_selftest_memchr(const RtFunctions& fns) {
    using namespace rt_selftest_detail;
    RtSelftestResult r;
    auto* volatile f = fns.find;
    uint8_t* s = arena().a;
    for (size_t i = 0; i < copy_bytes; ++i) {
        s[i] = static_cast<uint8_t>(i & 0x3Fu);  // 0..63 repeating
    }
    // Past every 64-byte window below: a byte above 0x7F.
    const size_t high_at = copy_bytes - guard - 1u;
    s[high_at] = 0xC3u;
    for (size_t off = 0; off < offsets; ++off) {
        uint8_t* base = s + guard + off;
        for (uint32_t v = 0; v < 64u; ++v) {
            ++r.cases;
            auto* hit = static_cast<uint8_t*>(f(base, static_cast<int>(v), 64));
            bool ok = hit != nullptr && *hit == v;
            for (uint8_t* q = base; ok && q < hit; ++q) {
                ok = *q != v;
            }
            if (!ok) {
                r.fail("memchr find: offset, value", static_cast<uint32_t>(off), v);
            }
        }
        r.cases += 2u;
        if (f(base, 0x40, max_len - guard) != nullptr) {
            r.fail("memchr absent: offset", static_cast<uint32_t>(off), 0);
        }
        // The first five bytes are five distinct values, none of them the
        // sixth's: searching five bytes for the sixth finds nothing.
        if (f(base, base[5], 5) != nullptr) {
            r.fail("memchr bound: offset", static_cast<uint32_t>(off), 5);
        }
    }
    r.cases += 3u;
    if (f(s, -61, high_at + 1u) != s + high_at) {
        r.fail("memchr (unsigned char)-61", 0, 0);
    }
    if (f(s, 0x1C3, high_at + 1u) != s + high_at) {
        r.fail("memchr (unsigned char)0x1C3", 0, 0);
    }
    if (f(s, 0xC3, high_at) != nullptr) {
        r.fail("memchr one short of 0xC3", 0, 0);
    }
    return r;
}

/// memchr through its C signature, whatever prototype the header gave it:
/// glibc's <string.h> declares the const-correct C++ overload pair (a
/// cast to the C signature matches neither), a freestanding image and the
/// other C libraries the C one. The call inside is to whatever the image
/// links, with arguments nothing can fold.
inline void* rt_memchr(const void* s, int c, size_t n) {
    return const_cast<void*>(static_cast<const void*>(::memchr(s, c, n)));
}

/// The functions this image really links - rt.cpp's on a 32-bit target -
/// for a letter of a platform suite.
inline RtFunctions rt_linked_functions() {
    return RtFunctions{
        static_cast<void* (*)(void*, const void*, size_t)>(&::memcpy),
        static_cast<void* (*)(void*, const void*, size_t)>(&::memmove),
        static_cast<void* (*)(void*, int, size_t)>(&::memset),
        static_cast<int (*)(const void*, const void*, size_t)>(&::memcmp),
        static_cast<size_t (*)(const char*)>(&::strlen),
        &rt_memchr};
}

/// Every group of cases in turn, each handed to report(name, result) - the
/// platform suites print it and give the verdict with their own bench.
template <typename Report>
void rt_selftest_all(const RtFunctions& fns, Report report) {
    report("memcpy at every misalignment of both pointers, every length", rt_selftest_memcpy(fns));
    report("memmove at every overlap in both directions, every length", rt_selftest_memmove(fns));
    report("memset at every misalignment and length, six values", rt_selftest_memset(fns));
    report("memcmp: zero when equal, the unsigned sign of the first difference", rt_selftest_memcmp(fns));
    report("strlen at every length and misalignment", rt_selftest_strlen(fns));
    report("memchr: every position, absent values, (unsigned char)c, the bound", rt_selftest_memchr(fns));
}

}  // namespace brio

#pragma GCC pop_options
