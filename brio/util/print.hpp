/*
 * print.hpp
 *
 * Text formatting as free variadic functions over any brio::ByteSink: a
 * transport moves bytes, formatting lives here.
 *
 * Usage (sink passed as a zero-cost tag instance):
 *
 *   constexpr brio::Uart<2> serial;
 *   brio::print(serial, "[", ts, "] count = ", count, brio::crlf);
 *   brio::print(serial, "mask = ", brio::hex(0xBEEF), " v = ", brio::fixed(v, 8, 3));
 *
 * Supported argument types: char, C strings, all integer types up to 64
 * bits (decimal; brio::hex() for hexadecimal, 32 bits), bool (as 1/0),
 * float/double (scientific by default, brio::fixed()/brio::sci() to
 * control), brio::TimeStamp, brio::crlf.
 *
 * Extension point: print() dispatches each argument to an unqualified
 * print_one(sink, value) call, so a new type becomes printable by providing
 * a print_one overload for it (in namespace brio or in the type's own
 * namespace, found via ADL).
 *
 * Delivery policy: print BLOCKS until the sink has accepted every byte,
 * and never truncates. Where the sink is a BulkSink (util/stream.hpp) a
 * string_view goes to it as ONE RUN, and a C string - every formatted
 * number among them, each formatted into a buffer and printed as a
 * string - as its first byte through write_byte(), so an idle
 * transmitter starts before anything is measured, then as runs of up to
 * print_scan_run bytes, measured as they go: write_bulk() is handed the
 * run and asked again for what is left, spinning while nothing fits -
 * one copy into the transport's ring and at most two nudges of its
 * transmitter a run, where a byte at a time pays both per byte. A sink
 * with the byte verb alone is spun on per byte, through write_byte().
 * Either way, with the interrupt-driven Uart this means "wait for the TX
 * ring to drain" when the text is longer than its room.
 * Consequence: only print after the sink is initialized and interrupts
 * are enabled, or the spin never ends.
 *
 * Number-to-text conversion uses avr-libc (ltoa/ultoa/dtostrf/dtostre):
 * <charconv> is not part of this freestanding libstdc++. Those four are
 * an AVR-libc extension, not standard C, so a HOSTED build (the native
 * test target, which drives print through a capture sink) gets snprintf
 * equivalents below - this is util/, and a service here must parse
 * wherever the framework is compiled. Same names, same call shapes, and
 * on AVR nothing but avr-libc is called.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>  // ltoa, ultoa, dtostrf, dtostre (AVR-libc)
#include <concepts>
#include <span>
#include <string_view>
#include "util/stream.hpp"
#include "util/timestamp.hpp"

#if !defined(__AVR__)
#include <stdio.h>
#endif

namespace brio {

/// A quotient and its remainder, C's div_t by its field names.
struct DivMod {
    uint32_t quot;
    uint32_t rem;
};

/// value / 10 and value % 10 by shifts and adds alone - no division, no
/// multiply, no library call, the same dozen and a half instructions on
/// every core, which is what a decimal digit costs where the core has no
/// divider (gcc at -Os keeps `/ 10` a libgcc call there, a shift and
/// subtract of some hundred and fifty instructions) and no multiplier
/// either (where a reciprocal multiply would be a call of its own).
///
/// THE ESTIMATE. value / 10 = value * 4/5 / 8, and 4/5 = 3/4 * 16/15
/// with 16/15 = (1 + 2^-4)(1 + 2^-8)(1 + 2^-16) / (1 - 2^-32): the first
/// line forms 3/4 of the value, the next three multiply it by the three
/// factors, the shift divides by 8. Every right shift truncates, so the
/// estimate is NEVER ABOVE value / 10. What the truncations lose, carried
/// through the factors that follow them (a product under 16/15), stays
/// under 5.2 before the last shift - 4/3 from the first line's 5/4, under
/// one from each of the three lines after it, under 0.8 from the 2^-32
/// the product leaves out - and the last shift divides that by 8 and
/// truncates up to 7/8 more: the estimate is short of value / 10 by under
/// 5.2/8 + 7/8 < 1.53. An integer at most value / 10 and short of it by
/// under 1.53 is floor(value / 10) or one less, so the remainder it
/// leaves is under 20 and ONE correction - subtract 10 once when it is 10
/// or more - makes both exact. Nothing overflows: every intermediate is
/// at most 0.8 * value and the product taken back, ten times the
/// estimate, at most value. test_print checks it against `/` and `%`.
constexpr DivMod divmod10(uint32_t value) {
    uint32_t q = (value >> 1) + (value >> 2);
    q += q >> 4;
    q += q >> 8;
    q += q >> 16;
    q >>= 3;
    uint32_t r = value - ((q << 3) + (q << 1));
    if (r >= 10u) {
        q += 1u;
        r -= 10u;
    }
    return {q, r};
}

#if defined(__AVR__)
using ::dtostre;
using ::dtostrf;
using ::ltoa;
using ::ultoa;
#else
/// The AVR-libc conversions the hosted C library does not ship. Two
/// bases are ever asked for here, 10 and 16, and each loop has its
/// divisor as a CONSTANT: base 16 by a mask and a shift, base 10 by
/// divmod10() above, the same shifts and adds on every core whether or
/// not it divides. Any base but 16 converts as 10. Where `unsigned long`
/// is wider than 32 bits - an LP64 host - the digits above 32 bits are
/// peeled off first by the compiler's own division, a loop that does not
/// exist where it is 32 bits; brio itself never hands this more than 32.
/// A longer buffer than the call sites give is impossible for the widths
/// brio prints.
inline char* ultoa(unsigned long value, char* buffer, int base) {
    char digits[24];
    uint8_t n = 0;
    if (base == 16) {
        do {
            const uint8_t d = static_cast<uint8_t>(value & 0xFu);
            digits[n++] = static_cast<char>(d < 10u ? '0' + d : 'A' + (d - 10u));
            value >>= 4;
        } while (value != 0u);
    } else {
        if constexpr (sizeof(unsigned long) > sizeof(uint32_t)) {
            while (value > 0xFFFF'FFFFul) {
                const unsigned long quotient = value / 10u;
                digits[n++] = static_cast<char>('0' + static_cast<uint8_t>(value - quotient * 10u));
                value = quotient;
            }
        }
        uint32_t v = static_cast<uint32_t>(value);
        do {
            const DivMod d = divmod10(v);
            digits[n++] = static_cast<char>('0' + d.rem);
            v = d.quot;
        } while (v != 0u);
    }
    for (uint8_t i = 0; i < n; ++i) {
        buffer[i] = digits[n - 1 - i];
    }
    buffer[n] = '\0';
    return buffer;
}

inline char* ltoa(long value, char* buffer, int base) {
    if (value < 0 && base == 10) {
        buffer[0] = '-';
        // Negate through the unsigned type so LONG_MIN is not UB.
        (void)ultoa(0UL - static_cast<unsigned long>(value), buffer + 1, base);
        return buffer;
    }
    return ultoa(static_cast<unsigned long>(value), buffer, base);
}

inline char* dtostrf(double value, signed char width, unsigned char precision,
                     char* buffer) {
    (void)snprintf(buffer, 20, "%*.*f", static_cast<int>(width),
                   static_cast<int>(precision), value);
    return buffer;
}

inline char* dtostre(double value, char* buffer, unsigned char precision,
                     unsigned char /*flags*/) {
    (void)snprintf(buffer, 16, "%+.*e", static_cast<int>(precision), value);
    return buffer;
}

inline constexpr unsigned char DTOSTR_ALWAYS_SIGN = 0x01;
#endif

// ---- 64-bit integers ----------------------------------------------------------

/// The decimal text of a 64-bit value - at most 20 digits and a NUL -
/// with NO 64-bit division: libgcc's divides one bit at a time, which on
/// the AVR is thousands of cycles a digit. Below 2^32 the value is
/// ultoa()'s, the path every narrower integer takes. Above it the digits
/// from 10^19 down to 10^9 are counted out by subtracting their power
/// (at most nine subtractions a digit), and what is left, under 10^9 and
/// so 32 bits, is ultoa()'s again, zero-padded to its nine digits.
inline char* u64toa(uint64_t value, char* buffer) {
    if ((value >> 32) == 0u) {
        return ultoa(static_cast<unsigned long>(value), buffer, 10);
    }
    static constexpr uint64_t upper_powers[] = {
        10'000'000'000'000'000'000ULL, 1'000'000'000'000'000'000ULL,
        100'000'000'000'000'000ULL,    10'000'000'000'000'000ULL,
        1'000'000'000'000'000ULL,      100'000'000'000'000ULL,
        10'000'000'000'000ULL,         1'000'000'000'000ULL,
        100'000'000'000ULL,            10'000'000'000ULL,
        1'000'000'000ULL,
    };
    char* p = buffer;
    for (const uint64_t power : upper_powers) {
        char digit = '0';
        while (value >= power) {
            value -= power;
            ++digit;
        }
        // 2^32 > 10^9, so a non-zero digit comes before the last power:
        // only LEADING zeros are skipped.
        if (digit != '0' || p != buffer) {
            *p++ = digit;
        }
    }
    char low[10];
    (void)ultoa(static_cast<unsigned long>(value), low, 10);
    uint8_t n = 0;
    while (low[n] != '\0') {
        ++n;
    }
    for (uint8_t i = n; i < 9u; ++i) {
        *p++ = '0';
    }
    for (uint8_t i = 0; i < n; ++i) {
        *p++ = low[i];
    }
    *p = '\0';
    return buffer;
}

/// The signed twin: a minus sign and the magnitude, negated through the
/// unsigned type so INT64_MIN is not UB. At most 20 characters and a NUL.
inline char* i64toa(int64_t value, char* buffer) {
    if (value < 0) {
        buffer[0] = '-';
        (void)u64toa(0ULL - static_cast<uint64_t>(value), buffer + 1);
        return buffer;
    }
    return u64toa(static_cast<uint64_t>(value), buffer);
}

// ---- tokens and format wrappers ---------------------------------------------

struct crlf_t {};
inline constexpr crlf_t crlf{};  ///< print(serial, ..., crlf) -> "\r\n"

struct Hex { uint32_t value; };
/// Hexadecimal integer with 0x prefix: print(s, hex(0xBEEF)) -> "0xBEEF"
inline constexpr Hex hex(uint32_t value) { return {value}; }

// Floating point is handled in float on purpose. There is NO C-varargs
// promotion here (a template pack deduces exact types), and on avr-gcc
// double IS float (32-bit) by default, so the dtostrf/dtostre `double`
// signatures cost nothing. The assert below fires if the toolchain is ever
// rebuilt with -mdouble=64: at that point passing floats into those calls
// would start dragging in 64-bit soft-float, and this file must be
// revisited consciously (float-only printing vs full double support).
// (checked via the predefined macros, not sizeof: the IDE language server
// parses with a host-like data model where sizeof(double) is 8 and would
// flag a sizeof-based assert as failed, while macros are queried from the
// real avr-g++ and evaluate correctly in both worlds.)
#if defined(__AVR__)
static_assert(__SIZEOF_DOUBLE__ == __SIZEOF_FLOAT__,
              "toolchain built with -mdouble=64: revisit print.hpp float handling");
#endif

struct Fixed { float value; int8_t width; uint8_t precision; };
/// Fixed-point float: print(s, fixed(3.1415f, 8, 3)) -> "   3.142"
inline constexpr Fixed fixed(float value, int8_t width, uint8_t precision) {
    return {value, width, precision};
}

struct Sci { float value; uint8_t precision; };
/// Scientific float: print(s, sci(0.00123f, 3)) -> "+1.230e-03"
inline constexpr Sci sci(float value, uint8_t precision = 3) {
    return {value, precision};
}

// ---- delivery: the run and the byte -----------------------------------------

/// Spin until the sink accepts the byte (see delivery policy in the header).
template <ByteSink S>
inline void write_blocking(S, uint8_t b) {
    while (!S::write_byte(b)) {}
}

/// Spin until the sink has taken the whole run. A BulkSink is handed the
/// run and asked again for the rest whenever it took less, spinning while
/// nothing fits exactly as the byte path spins on a refused byte; any
/// other sink takes it a byte at a time.
template <ByteSink S>
inline void write_blocking([[maybe_unused]] S s, std::span<const uint8_t> run) {
    if constexpr (BulkSink<S>) {
        while (!run.empty()) {
            run = run.subspan(S::write_bulk(run));
        }
    } else {
        for (const uint8_t b : run) {
            write_blocking(s, b);
        }
    }
}

// ---- single-value writers (the ADL extension point) -------------------------

template <ByteSink S>
inline void print_one(S s, char c) {
    write_blocking(s, static_cast<uint8_t>(c));
}

template <ByteSink S>
inline void print_one(S s, std::string_view text) {
    write_blocking(s, std::span<const uint8_t>(reinterpret_cast<const uint8_t*>(text.data()), text.size()));
}

/// The longest stretch of a C string print measures before handing it to
/// a BulkSink (below).
inline constexpr size_t print_scan_run = 64;

/// A BulkSink takes a C string's FIRST BYTE through the byte verb, before
/// anything is measured, and the rest a STRETCH at a time: up to
/// print_scan_run bytes measured and handed over as one run, then the
/// next. An idle transmitter starts on that byte while the first stretch
/// is scanned and copied behind it - the scan alone, ahead of it, put
/// some six hundred cycles in front of a long print's first frame on the
/// Cortex-M0+ parts - and the length is found as the string goes, never
/// all of it first, so every later scan runs while the transmitter
/// drains the stretch before it (a whole-string strlen() first delays a
/// 4096-byte print by its scan, measured at 260 us on the CH32V203).
/// Any other sink is fed byte by byte up to the NUL, with no length to
/// find.
template <ByteSink S>
inline void print_one(S s, const char *text) {
    if constexpr (BulkSink<S>) {
        if (*text == '\0') {
            return;
        }
        write_blocking(s, static_cast<uint8_t>(*text));
        ++text;
        for (;;) {
            size_t n = 0;
            while (n < print_scan_run && text[n] != '\0') {
                ++n;
            }
            write_blocking(s, std::span<const uint8_t>(reinterpret_cast<const uint8_t*>(text), n));
            if (n < print_scan_run) {
                return;   // the stretch ended at the NUL
            }
            text += n;
        }
    } else {
        while (*text) {
            write_blocking(s, static_cast<uint8_t>(*text));
            ++text;
        }
    }
}

template <ByteSink S>
inline void print_one(S s, crlf_t) {
    print_one(s, std::string_view("\r\n", 2));
}

/// Integers in decimal. Up to 32 bits through ltoa/ultoa; a 64-bit one
/// through its own explicit-width path, never through `long`, which is 32
/// bits on every target but the host.
template <ByteSink S, std::integral I>
inline void print_one(S s, I value) {
    if constexpr (sizeof(I) > 4u) {
        static_assert(sizeof(I) == 8u, "print: no integer wider than 64 bits");
        char buffer[21];
        if constexpr (std::is_signed_v<I>) {
            (void)i64toa(static_cast<int64_t>(value), buffer);
        } else {
            (void)u64toa(static_cast<uint64_t>(value), buffer);
        }
        print_one(s, static_cast<const char *>(buffer));
    } else {
        char buffer[12];
        if constexpr (std::is_signed_v<I>) {
            ltoa(static_cast<long>(value), buffer, 10);
        } else {
            ultoa(static_cast<unsigned long>(value), buffer, 10);
        }
        print_one(s, static_cast<const char *>(buffer));
    }
}

template <ByteSink S>
inline void print_one(S s, Hex h) {
    char buffer[11];
    buffer[0] = '0';
    buffer[1] = 'x';
    ultoa(h.value, &buffer[2], 16);
    print_one(s, static_cast<const char *>(buffer));
}

template <ByteSink S>
inline void print_one(S s, Fixed f) {
    char buffer[20];
    const int8_t width = (f.width > 18) ? int8_t{18} : f.width;  // keep in buffer
    dtostrf(f.value, width, f.precision, buffer);
    print_one(s, static_cast<const char *>(buffer));
}

template <ByteSink S>
inline void print_one(S s, Sci e) {
    char buffer[16];
    const uint8_t precision = (e.precision > 7) ? uint8_t{7} : e.precision;
    dtostre(e.value, buffer, precision, DTOSTR_ALWAYS_SIGN);
    print_one(s, static_cast<const char *>(buffer));
}

template <ByteSink S, std::floating_point F>
inline void print_one(S s, F value) {
    print_one(s, Sci{static_cast<float>(value), 3});
}

/// TimeStamp as "<seconds>.<millis>s", millis zero-padded to 3 digits
/// (12.045s) - unambiguous on every target, where a tick-based fraction
/// would change unit with the silicon.
template <ByteSink S>
inline void print_one(S s, const TimeStamp &t) {
    print_one(s, t.seconds);
    print_one(s, '.');
    if (t.millis < 100) {
        print_one(s, '0');
    }
    if (t.millis < 10) {
        print_one(s, '0');
    }
    print_one(s, t.millis);
    print_one(s, 's');
}

// ---- the variadic front end -------------------------------------------------

/// Print every argument in order onto the sink (see print_one overloads).
template <ByteSink S, typename... Args>
inline void print(S s, const Args &...args) {
    (print_one(s, args), ...);
}

} // namespace brio
