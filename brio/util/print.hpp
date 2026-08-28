/*
 * print.hpp
 *
 * Text formatting as free variadic functions over any brio::ByteSink - the
 * replacement for the print() methods that used to live inside the Uart
 * driver (which is byte transport only now).
 *
 * Usage (sink passed as a zero-cost tag instance):
 *
 *   constexpr brio::Uart<2, brio::Route::alt1> serial;
 *   brio::print(serial, "[", ts, "] count = ", count, brio::crlf);
 *   brio::print(serial, "mask = ", brio::hex(0xBEEF), " v = ", brio::fixed(v, 8, 3));
 *
 * Supported argument types: char, C strings, all integer types (decimal;
 * brio::hex() for hexadecimal), bool (as 1/0), float/double (scientific by
 * default, brio::fixed()/brio::sci() to control), brio::TimeStamp, brio::crlf.
 *
 * Extension point: print() dispatches each argument to an unqualified
 * print_one(sink, value) call, so a new type becomes printable by providing
 * a print_one overload for it (in namespace brio or in the type's own
 * namespace, found via ADL).
 *
 * Delivery policy: print BLOCKS until the sink accepts each byte (spinning
 * on write_byte). With the interrupt-driven Uart this means "wait for the
 * TX ring to drain", so printed text is never silently truncated - the old
 * driver dropped bytes on a full ring. Consequence: only print after the
 * sink is initialized and interrupts are enabled, or the spin never ends.
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

#include <stdint.h>
#include <stdlib.h>  // ltoa, ultoa, dtostrf, dtostre (AVR-libc)
#include <concepts>
#include <string_view>
#include "util/stream.hpp"
#include "util/timestamp.hpp"

#if !defined(__AVR__)
#include <stdio.h>
#endif

namespace brio {

#if defined(__AVR__)
using ::dtostre;
using ::dtostrf;
using ::ltoa;
using ::ultoa;
#else
/// The AVR-libc conversions the hosted C library does not ship. Only
/// base 10 and 16 are ever asked for here; a longer buffer than the
/// call sites give is impossible for the widths brio prints.
inline char* ultoa(unsigned long value, char* buffer, int base) {
    char digits[24];
    uint8_t n = 0;
    do {
        const unsigned long d = value % static_cast<unsigned long>(base);
        digits[n++] = static_cast<char>(d < 10 ? '0' + d : 'A' + (d - 10));
        value /= static_cast<unsigned long>(base);
    } while (value != 0);
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

// ---- single-value writers (the ADL extension point) -------------------------

/// Spin until the sink accepts the byte (see delivery policy in the header).
template <ByteSink S>
inline void write_blocking(S, uint8_t b) {
    while (!S::write_byte(b)) {}
}

template <ByteSink S>
inline void print_one(S s, char c) {
    write_blocking(s, static_cast<uint8_t>(c));
}

template <ByteSink S>
inline void print_one(S s, const char *text) {
    while (*text) {
        write_blocking(s, static_cast<uint8_t>(*text));
        ++text;
    }
}

template <ByteSink S>
inline void print_one(S s, std::string_view text) {
    for (const char c : text) write_blocking(s, static_cast<uint8_t>(c));
}

template <ByteSink S>
inline void print_one(S s, crlf_t) {
    print_one(s, '\r');
    print_one(s, '\n');
}

template <ByteSink S, std::integral I>
inline void print_one(S s, I value) {
    char buffer[12];
    if constexpr (std::is_signed_v<I>) {
        ltoa(static_cast<long>(value), buffer, 10);
    } else {
        ultoa(static_cast<unsigned long>(value), buffer, 10);
    }
    print_one(s, static_cast<const char *>(buffer));
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
/// (12.045s) - unambiguous on every target, unlike the old tick-based
/// "12s.46t" whose unit changed with the silicon.
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
