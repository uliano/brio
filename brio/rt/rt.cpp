/*
 * rt.cpp
 *
 * The runtime: the seven functions a freestanding C++ image cannot avoid,
 * and the one source file of the framework (design/runtime.md).
 *
 *   memcpy, memmove, memset, memcmp  what GCC calls on its own - for a
 *                                    copy or a value-initialization of
 *                                    an object above a small size, for a
 *                                    library algorithm on trivially
 *                                    copyable data, for a loop it
 *                                    recognizes as a copy or a fill;
 *   strlen, memchr                   what the freestanding libstdc++
 *                                    brio builds on calls besides:
 *                                    std::char_traits<char>'s length and
 *                                    find, so a std::string_view made
 *                                    from a pointer, or searched;
 *   abort                            what libstdc++ calls on a throw
 *                                    path under -fno-exceptions, which
 *                                    -Og leaves alive.
 *
 * COMPILED INTO EVERY 32-BIT IMAGE, beside its crt, and linked with
 * -nostartfiles -nodefaultlibs and libgcc: no C library and no syscall
 * layer reach the link, so an accidental printf or malloc fails there
 * with its own name. The AVR does not compile this file: avr-libc's mem*
 * are hand-written assembly, small and fast at once.
 *
 * THE WORD PATH. A copy moves words when source and destination share
 * their alignment modulo the word (bytes up to a word boundary, then
 * words, then the tail), and bytes otherwise. No function here ever
 * performs a word access at an address that is not a multiple of the
 * word: ARMv6-M faults on one and Hazard3 traps, so the alignment test is
 * part of the contract, not an optimization. The events the kernel copies
 * are word-aligned by their own members whenever they are large enough to
 * reach memcpy at all (design/runtime.md, the census).
 *
 * THE SELF-CALL TRAP. GCC recognizes a byte loop that copies or fills and
 * replaces it with a call to memcpy or memset - here, a call to the very
 * function being defined. Measured without the guard: at -O2 memset
 * becomes a call to memset, at -O3 memcpy a call to memcpy; at -Os and -Og
 * it does not happen today, which is a property of the optimizer and not
 * a promise. The pragma below turns that recognition off for this file
 * alone, so no build project has to remember a flag; the family check
 * verifies on every compiler that none of these functions calls one of
 * the seven.
 *
 * On the host, test/test_rt/ includes this file with BRIO_RT_HOST_TEST
 * defined: the implementations are compiled and tested exactly as the
 * targets compile them, and the C-linkage definitions - which would
 * replace the host C library's own - are left out.
 */

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#pragma GCC push_options
#pragma GCC optimize("no-tree-loop-distribute-patterns")

namespace {

// A word as memory seen through a pointer that may alias anything: the
// copies move the bytes of objects of every type, so the word accesses
// must not be subject to the type-based aliasing rules.
using RtWord = uint32_t __attribute__((__may_alias__));

constexpr size_t rt_word_bytes = sizeof(RtWord);
constexpr uintptr_t rt_word_mask = rt_word_bytes - 1u;

uintptr_t rt_address(const void* p) {
    return reinterpret_cast<uintptr_t>(p);
}

// Forward copy: correct for disjoint ranges and for a destination below
// an overlapping source.
void rt_copy_forward(unsigned char* d, const unsigned char* s, size_t n) {
    if (((rt_address(d) ^ rt_address(s)) & rt_word_mask) == 0u) {
        while (n != 0u && (rt_address(d) & rt_word_mask) != 0u) {
            *d++ = *s++;
            --n;
        }
        while (n >= rt_word_bytes) {
            *reinterpret_cast<RtWord*>(d) = *reinterpret_cast<const RtWord*>(s);
            d += rt_word_bytes;
            s += rt_word_bytes;
            n -= rt_word_bytes;
        }
    }
    while (n != 0u) {
        *d++ = *s++;
        --n;
    }
}

// Backward copy, from the end: correct for a destination above an
// overlapping source. The same word path, mirrored.
void rt_copy_backward(unsigned char* d, const unsigned char* s, size_t n) {
    d += n;
    s += n;
    if (((rt_address(d) ^ rt_address(s)) & rt_word_mask) == 0u) {
        while (n != 0u && (rt_address(d) & rt_word_mask) != 0u) {
            *--d = *--s;
            --n;
        }
        while (n >= rt_word_bytes) {
            d -= rt_word_bytes;
            s -= rt_word_bytes;
            n -= rt_word_bytes;
            *reinterpret_cast<RtWord*>(d) = *reinterpret_cast<const RtWord*>(s);
        }
    }
    while (n != 0u) {
        *--d = *--s;
        --n;
    }
}

void* rt_memcpy(void* dst, const void* src, size_t n) {
    rt_copy_forward(static_cast<unsigned char*>(dst), static_cast<const unsigned char*>(src), n);
    return dst;
}

void* rt_memmove(void* dst, const void* src, size_t n) {
    auto* d = static_cast<unsigned char*>(dst);
    const auto* s = static_cast<const unsigned char*>(src);
    // Compared as integers: the pointers may belong to different objects,
    // where a pointer comparison has no meaning in the language.
    if (rt_address(d) <= rt_address(s) || rt_address(d) >= rt_address(s) + n) {
        rt_copy_forward(d, s, n);
    } else {
        rt_copy_backward(d, s, n);
    }
    return dst;
}

void* rt_memset(void* dst, int c, size_t n) {
    auto* d = static_cast<unsigned char*>(dst);
    const auto v = static_cast<unsigned char>(c);
    while (n != 0u && (rt_address(d) & rt_word_mask) != 0u) {
        *d++ = v;
        --n;
    }
    const RtWord w = static_cast<RtWord>(v) * static_cast<RtWord>(0x01010101u);
    while (n >= rt_word_bytes) {
        *reinterpret_cast<RtWord*>(d) = w;
        d += rt_word_bytes;
        n -= rt_word_bytes;
    }
    while (n != 0u) {
        *d++ = v;
        --n;
    }
    return dst;
}

// The sign of the first differing byte, both read as unsigned char.
int rt_memcmp(const void* a, const void* b, size_t n) {
    const auto* x = static_cast<const unsigned char*>(a);
    const auto* y = static_cast<const unsigned char*>(b);
    for (; n != 0u; --n, ++x, ++y) {
        if (*x != *y) {
            return static_cast<int>(*x) - static_cast<int>(*y);
        }
    }
    return 0;
}

// The length of a zero-terminated string: a byte scan. The strings brio
// measures at run time are short, so the word-at-a-time trick would buy
// nothing worth its code.
size_t rt_strlen(const char* s) {
    const char* p = s;
    while (*p != '\0') {
        ++p;
    }
    return static_cast<size_t>(p - s);
}

// The first byte equal to (unsigned char)c within n bytes, or nothing.
void* rt_memchr(const void* s, int c, size_t n) {
    const auto* p = static_cast<const unsigned char*>(s);
    const auto v = static_cast<unsigned char>(c);
    for (; n != 0u; --n, ++p) {
        if (*p == v) {
            return const_cast<unsigned char*>(p);
        }
    }
    return nullptr;
}

}  // namespace

#ifndef BRIO_RT_HOST_TEST

extern "C" {

void* memcpy(void* __restrict dst, const void* __restrict src, size_t n) {
    return rt_memcpy(dst, src, n);
}

void* memmove(void* dst, const void* src, size_t n) {
    return rt_memmove(dst, src, n);
}

void* memset(void* dst, int c, size_t n) {
    return rt_memset(dst, c, n);
}

int memcmp(const void* a, const void* b, size_t n) {
    return rt_memcmp(a, b, n);
}

size_t strlen(const char* s) {
    return rt_strlen(s);
}

void* memchr(const void* s, int c, size_t n) {
    return rt_memchr(s, c, n);
}

// A spin, not a breakpoint: on ARMv6-M a breakpoint with no debugger
// attached escalates to HardFault and the frame that got here is lost,
// while a spin leaves the whole call stack for a debugger to walk. The
// empty asm is a side effect, so the loop is not one the language lets
// the compiler assume away. No panic breadcrumb: the runtime knows no
// platform (design/runtime.md).
[[noreturn]] void abort() {
    for (;;) {
        __asm__ volatile("");
    }
}

}  // extern "C"

#endif  // BRIO_RT_HOST_TEST

#pragma GCC pop_options
