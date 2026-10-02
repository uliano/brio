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
 * bits (decimal; brio::hex() for hexadecimal, 32 or 64 bits), bool (as
 * 1/0), float/double (scientific by default, brio::fixed()/brio::sci() to
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
 * Number-to-text conversion goes through functions under avr-libc's
 * names and call shapes - ltoa, ultoa, dtostrf, dtostre and the float
 * twins ftostrf, ftostre (<charconv> is not part of this freestanding
 * libstdc++). On the AVR they ARE avr-libc's. Everywhere else they are
 * this file's own, written in C++ over no library at all, because no
 * 32-bit image links a C library (docs/design/runtime.md): the integers
 * by divmod10() below, the floats by DoubleSplit, an exact binary
 * fixed-point split of the value that needs no floating-point operation
 * either - its range and its rounding are stated where it is defined.
 * The host test (test/test_print) holds them against the host's own C
 * library.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>  // ltoa, ultoa, dtostrf, dtostre on the AVR (avr-libc)
#include <bit>
#include <concepts>
#include <span>
#include <string_view>
#include <type_traits>
#include "util/stream.hpp"
#include "util/timestamp.hpp"

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
using ::ftostre;
using ::ftostrf;
using ::ltoa;
using ::ultoa;
#else
/// avr-libc's integer conversions, for every target but the AVR. Two
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

// ---- the float conversions ----------------------------------------------------

/// dtostre()'s flags, under avr-libc's names and with its values.
inline constexpr unsigned char DTOSTR_ALWAYS_SIGN = 0x01;  ///< a space before a positive value
inline constexpr unsigned char DTOSTR_PLUS_SIGN = 0x02;    ///< '+' instead of that space
inline constexpr unsigned char DTOSTR_UPPERCASE = 0x04;    ///< "E", "NAN" and "INF"

/// THE INTEGER-AND-FRACTION SPLIT both float conversions stand on: the
/// magnitude of a double as an EXACT binary fixed-point number, 128 bits
/// of integer over 256 bits of fraction in twelve 32-bit words. No
/// floating-point operation is involved: the value's bits are taken apart
/// and placed, and every decimal digit is a multiply or a divide by ten
/// in shifts and adds - divmod10() above - on any core, with or without a
/// multiplier.
///
/// THE RANGE. The integer words hold every finite float and every double
/// below 2^128. A double of 2^128 or more is OUT OF RANGE and reads as an
/// infinity of its sign, so it prints the letters an infinite value
/// prints. The fraction words keep the value's bits from 2^-256 up:
/// every double from 2^-204 up - whose 53 bits all lie at or above
/// 2^-256 - is held exactly, and so is every float, the smallest of which
/// is 2^-149. A double below 2^-204 keeps its bits from 2^-256 up, and
/// one below 2^-256 reads as zero.
///
/// The digits come out of it one at a time: the integer part's least
/// significant first, each divided away, and the fraction's most
/// significant first, each multiplied out, the words left behind being
/// what is still to come - so the remainder after any digit is known
/// exactly, which is what rounds the last digit.
class DoubleSplit {
public:
    enum class Kind : uint8_t { finite, infinite, nan };

    explicit DoubleSplit(double value) {
        const uint64_t bits = std::bit_cast<uint64_t>(value);
        negative_ = (bits >> 63) != 0u;
        const uint32_t biased = static_cast<uint32_t>(bits >> 52) & 0x7FFu;
        const uint32_t high = static_cast<uint32_t>(bits >> 32) & 0x000F'FFFFu;
        const uint32_t low = static_cast<uint32_t>(bits);
        if (biased == 0x7FFu) {
            kind_ = (high | low) != 0u ? Kind::nan : Kind::infinite;
            return;
        }
        if (biased >= 1023u + 128u) {
            kind_ = Kind::infinite;   // 2^128 and up: past the integer words
            return;
        }
        // value = m * 2^(e - 1075): m the 52 stored bits under the hidden
        // one, e the biased exponent - a subnormal has no hidden bit and
        // the exponent of e = 1. Counted from bit 0 of the words (2^-256),
        // m's bit 0 lands at bit e - 1075 + 256 = e - 819; a word below
        // the first is a bit cut, a word past the last cannot be reached
        // in range (m's top bit lies at most at bit 383).
        const uint32_t m_high = biased != 0u ? (high | 0x0010'0000u) : high;
        place_mantissa(m_high, low, static_cast<int32_t>(biased != 0u ? biased : 1u) - 819);
    }

    /// A float split from its OWN bits: every float is a double, so this is
    /// the split of the same value, with no widening to double in between -
    /// a libgcc call on a core without a double-precision unit, which no
    /// 32-bit core brio has (140 bytes on the Cortex-M0+, 456 with the
    /// count-leading-zeros table behind it on the CH32V003). Every finite
    /// float is in range.
    explicit DoubleSplit(float value) {
        const uint32_t bits = std::bit_cast<uint32_t>(value);
        negative_ = (bits >> 31) != 0u;
        const uint32_t biased = (bits >> 23) & 0xFFu;
        const uint32_t mantissa = bits & 0x007F'FFFFu;
        if (biased == 0xFFu) {
            kind_ = mantissa != 0u ? Kind::nan : Kind::infinite;
            return;
        }
        // value = m * 2^(e - 150), m the 23 stored bits under the hidden
        // one: m's bit 0 lands at bit e - 150 + 256 = e + 106.
        const uint32_t m = biased != 0u ? (mantissa | 0x0080'0000u) : mantissa;
        place_mantissa(0u, m, static_cast<int32_t>(biased != 0u ? biased : 1u) + 106);
    }

    Kind kind() const { return kind_; }
    /// The sign bit: a negative zero and a NaN with it set are negative.
    bool negative() const { return negative_; }
    bool integer_zero() const { return integer_top_ == fraction_words; }
    bool fraction_zero() const { return fraction_low_ == fraction_words; }

    /// The integer part's lowest decimal digit, divided away. Each word is
    /// divided in two 16-bit steps, so every divmod10() holds a value
    /// under 10 * 2^16.
    uint8_t next_integer_digit() {
        uint32_t remainder = 0u;
        for (uint8_t i = integer_top_; i-- > fraction_words;) {
            const DivMod upper = divmod10((remainder << 16) | (words_[i] >> 16));
            const DivMod lower = divmod10((upper.rem << 16) | (words_[i] & 0xFFFFu));
            words_[i] = (upper.quot << 16) | lower.quot;
            remainder = lower.rem;
        }
        while (integer_top_ > fraction_words && words_[integer_top_ - 1u] == 0u) {
            --integer_top_;
        }
        return static_cast<uint8_t>(remainder);
    }

    /// The fraction's next decimal digit, multiplied out: ten times the
    /// fraction, the part that crosses the binary point the digit and the
    /// rest kept. Each word is 8w + 2w + the carry in 32-bit halves, the
    /// carries out of the two sums counted by comparison - written so,
    /// because on the Cortex-M0+ gcc turns 64-bit shifts and adds of a word
    /// back into a call to libgcc's 64-bit multiply, one a word a digit.
    /// Words below the lowest non-zero one stay zero and are skipped.
    uint8_t next_fraction_digit() {
        uint32_t carry = 0u;
        for (uint8_t i = fraction_low_; i < fraction_words; ++i) {
            const uint32_t word = words_[i];
            const uint32_t eight = word << 3;
            const uint32_t ten = eight + (word << 1);
            const uint32_t sum = ten + carry;
            carry = (word >> 29) + (word >> 31) + (ten < eight ? 1u : 0u) + (sum < ten ? 1u : 0u);
            words_[i] = sum;
        }
        while (fraction_low_ < fraction_words && words_[fraction_low_] == 0u) {
            ++fraction_low_;
        }
        return static_cast<uint8_t>(carry);
    }

private:
    static constexpr uint8_t fraction_words = 8;   // 2^-256 .. 2^-1
    static constexpr uint8_t word_count = 12;      // and 2^0 .. 2^127 above them

    void place(int32_t index, uint32_t bits) {
        if (index >= 0 && index < word_count) {
            words_[index] |= bits;
        }
    }

    /// The integer mantissa (up to 53 bits, as two words) with its bit 0
    /// at bit `position` of the words, then the two ends found.
    void place_mantissa(uint32_t m_high, uint32_t m_low, int32_t position) {
        const int32_t index = position >> 5;   // floor, below zero as well
        const uint32_t shift = static_cast<uint32_t>(position) & 31u;
        place(index, m_low << shift);
        place(index + 1, (shift != 0u ? m_low >> (32u - shift) : 0u) | (m_high << shift));
        place(index + 2, shift != 0u ? m_high >> (32u - shift) : 0u);
        while (fraction_low_ < fraction_words && words_[fraction_low_] == 0u) {
            ++fraction_low_;
        }
        while (integer_top_ > fraction_words && words_[integer_top_ - 1u] == 0u) {
            --integer_top_;
        }
    }

    uint32_t words_[word_count]{};
    uint8_t fraction_low_ = 0;           // the lowest non-zero fraction word, or 8
    uint8_t integer_top_ = word_count;   // one past the highest non-zero word, or 8
    Kind kind_ = Kind::finite;
    bool negative_ = false;
};

/// The rounding of a digit string cut short: up when what is cut is more
/// than half a unit of the last digit kept, down when less, and to EVEN on
/// an exact half - `next` the first digit cut, `sticky` whether anything
/// after it is non-zero, `last` the last digit kept (a digit or its
/// character: '0' is even). What the host's C library does for "%f" and
/// "%e", which the host test holds both conversions against.
constexpr bool rounds_up(uint8_t next, bool sticky, uint8_t last) {
    return next > 5u || (next == 5u && (sticky || (last & 1u) != 0u));
}

/// The body of dtostrf() and ftostrf(), over the value's split.
inline char* dtostrf_split(DoubleSplit& split, signed char width, unsigned char precision,
                           char* buffer) {
    const bool left = width < 0;
    const uint16_t field = static_cast<uint16_t>(left ? -static_cast<int16_t>(width) : width);
    // The text is built from buffer + 1: buffer[0] is kept for the digit a
    // carry out of the first one adds (9.96 to one decimal is 10.0).
    uint16_t start = 1;
    uint16_t length = 0;
    bool minus = false;
    if (split.kind() != DoubleSplit::Kind::finite) {
        const char* const letters = split.kind() == DoubleSplit::Kind::nan ? "NAN"
                                    : split.negative()                     ? "-INF"
                                                                           : "INF";
        for (; letters[length] != '\0'; ++length) {
            buffer[start + length] = letters[length];
        }
    } else {
        minus = split.negative();
        char* const text = buffer + start;
        do {
            text[length++] = static_cast<char>('0' + split.next_integer_digit());
        } while (!split.integer_zero());
        for (uint16_t i = 0, j = static_cast<uint16_t>(length - 1u); i < j; ++i, --j) {
            const char c = text[i];
            text[i] = text[j];
            text[j] = c;
        }
        if (precision != 0u) {
            text[length++] = '.';
            for (uint8_t i = 0; i < precision; ++i) {
                text[length++] = static_cast<char>('0' + split.next_fraction_digit());
            }
        }
        const uint8_t next = split.next_fraction_digit();
        if (rounds_up(next, !split.fraction_zero(), static_cast<uint8_t>(text[length - 1u]))) {
            uint16_t i = length;
            for (;;) {
                if (i == 0u) {
                    buffer[0] = '1';   // every digit was a nine
                    start = 0;
                    ++length;
                    break;
                }
                --i;
                if (text[i] == '.') {
                    continue;
                }
                if (text[i] == '9') {
                    text[i] = '0';
                    continue;
                }
                ++text[i];
                break;
            }
        }
    }
    // The field: the text moved to its place, the sign in front of it, the
    // padding before (right-aligned) or after (left-aligned).
    const uint16_t total = static_cast<uint16_t>(length + (minus ? 1u : 0u));
    const uint16_t pad = field > total ? static_cast<uint16_t>(field - total) : 0u;
    const uint16_t lead = left ? 0u : pad;
    const uint16_t to = static_cast<uint16_t>(lead + (minus ? 1u : 0u));
    if (to > start) {
        for (uint16_t i = length; i-- > 0u;) {
            buffer[to + i] = buffer[start + i];
        }
    } else if (to < start) {
        for (uint16_t i = 0; i < length; ++i) {
            buffer[to + i] = buffer[start + i];
        }
    }
    for (uint16_t i = 0; i < lead; ++i) {
        buffer[i] = ' ';
    }
    if (minus) {
        buffer[lead] = '-';
    }
    uint16_t end = static_cast<uint16_t>(to + length);
    if (left) {
        for (uint16_t i = 0; i < pad; ++i) {
            buffer[end++] = ' ';
        }
    }
    buffer[end] = '\0';
    return buffer;
}

/// The body of dtostre() and ftostre(), over the value's split.
inline char* dtostre_split(DoubleSplit& split, char* buffer, unsigned char precision,
                           unsigned char flags) {
    if (precision > 7u) {
        precision = 7u;
    }
    char* out = buffer;
    if (split.negative() && split.kind() != DoubleSplit::Kind::nan) {
        *out++ = '-';
    } else if ((flags & DTOSTR_PLUS_SIGN) != 0u) {
        *out++ = '+';
    } else if ((flags & DTOSTR_ALWAYS_SIGN) != 0u) {
        *out++ = ' ';
    }
    const bool upper = (flags & DTOSTR_UPPERCASE) != 0u;
    if (split.kind() != DoubleSplit::Kind::finite) {
        const char* const letters = split.kind() == DoubleSplit::Kind::nan ? (upper ? "NAN" : "nan")
                                                                           : (upper ? "INF" : "inf");
        for (uint8_t i = 0; i < 3u; ++i) {
            *out++ = letters[i];
        }
        *out = '\0';
        return buffer;
    }
    // The significant digits, as values: the first non-zero digit and the
    // precision after it, then the first digit cut and whether anything
    // after that is non-zero.
    const uint8_t count = static_cast<uint8_t>(precision + 1u);
    uint8_t digits[8]{};
    int8_t exponent = 0;
    uint8_t next = 0;
    bool sticky = false;
    if (!split.integer_zero()) {
        uint8_t integer[39];   // least significant first
        uint8_t n = 0;
        do {
            integer[n++] = split.next_integer_digit();
        } while (!split.integer_zero());
        exponent = static_cast<int8_t>(n - 1u);
        uint8_t k = 0;
        for (; k < count && k < n; ++k) {
            digits[k] = integer[n - 1u - k];
        }
        if (n <= count) {
            for (; k < count; ++k) {
                digits[k] = split.next_fraction_digit();
            }
            next = split.next_fraction_digit();
            sticky = !split.fraction_zero();
        } else {
            next = integer[n - 1u - count];
            sticky = !split.fraction_zero();
            for (uint8_t i = 0; i + 1u + count < n; ++i) {
                sticky = sticky || integer[i] != 0u;
            }
        }
    } else if (!split.fraction_zero()) {
        // Below one: the leading zeros multiplied out, each a step of the
        // exponent - a non-zero bit at 2^-256 or above makes a digit by the
        // 78th.
        exponent = -1;
        uint8_t d = split.next_fraction_digit();
        while (d == 0u) {
            --exponent;
            d = split.next_fraction_digit();
        }
        digits[0] = d;
        for (uint8_t k = 1; k < count; ++k) {
            digits[k] = split.next_fraction_digit();
        }
        next = split.next_fraction_digit();
        sticky = !split.fraction_zero();
    }
    if (rounds_up(next, sticky, digits[count - 1u])) {
        uint8_t i = count;
        for (;;) {
            if (i == 0u) {
                digits[0] = 1u;   // 9.99 became 10.0: the zeros stay, one decade up
                ++exponent;
                break;
            }
            --i;
            if (digits[i] == 9u) {
                digits[i] = 0u;
                continue;
            }
            ++digits[i];
            break;
        }
    }
    *out++ = static_cast<char>('0' + digits[0]);
    if (precision != 0u) {
        *out++ = '.';
        for (uint8_t k = 1; k < count; ++k) {
            *out++ = static_cast<char>('0' + digits[k]);
        }
    }
    *out++ = upper ? 'E' : 'e';
    if (exponent < 0) {
        *out++ = '-';
        exponent = static_cast<int8_t>(-exponent);
    } else {
        *out++ = '+';
    }
    // In range the exponent lies in -78 .. 38: two digits.
    const DivMod e = divmod10(static_cast<uint32_t>(exponent));
    *out++ = static_cast<char>('0' + e.quot);
    *out++ = static_cast<char>('0' + e.rem);
    *out = '\0';
    return buffer;
}

/// dtostrf(), avr-libc's contract (its stdlib.h, "Conversion functions
/// for double arguments"): the value in the format "[-]d.ddd" with
/// `precision` digits after the point - no point at precision zero -,
/// right-aligned in a field of `width` characters, or left-aligned in
/// -width when width is negative; a text longer than the field is not
/// cut. The digits are the value's own, rounded to the last decimal kept
/// as above: the text the host's snprintf("%*.*f") prints, for every
/// double below 2^128 at every precision while the value is 2^-204 or
/// more and at every precision up to 61 below it (2^-204 * 10^61 is under
/// half a unit, so the cut bits cannot reach a kept digit or its
/// rounding). A NaN prints "NAN" and an infinity "INF" or "-INF", in the
/// same field - avr-libc's letters, avr-libc's NaN carrying no sign. The
/// caller provides room for the text and its NUL: one character for the
/// sign, up to 39 integer digits, the point and the precision, or the
/// field when that is wider.
inline char* dtostrf(double value, signed char width, unsigned char precision,
                     char* buffer) {
    DoubleSplit split(value);
    return dtostrf_split(split, width, precision, buffer);
}

/// ftostrf(), avr-libc's float twin of dtostrf() under its name and
/// contract: the float split from its own bits, so a float prints with no
/// widening to double - every finite float in range.
inline char* ftostrf(float value, signed char width, unsigned char precision, char* buffer) {
    DoubleSplit split(value);
    return dtostrf_split(split, width, precision, buffer);
}

/// dtostre(), avr-libc's contract (its stdlib.h, "Conversion functions
/// for double arguments"): the value in the format "[-]d.ddde+dd", one
/// digit before the point and `precision` after it - no point at
/// precision zero, and a precision above 7 taken as 7, as avr-libc takes
/// it -, the exponent signed and in two digits, "+00" for a zero. A
/// negative value takes a '-'; a positive one takes '+' under
/// DTOSTR_PLUS_SIGN, else a space under DTOSTR_ALWAYS_SIGN, else nothing;
/// DTOSTR_UPPERCASE writes 'E'. The digits are the value's own, rounded
/// to the last one kept as above: the text the host's snprintf("%.*e")
/// prints with the matching flag, for zero and every double from 2^-204
/// up to 2^128 - every finite float among them. A NaN prints "nan" (a
/// sign only by the flags: avr-libc gives a NaN no '-') and an infinity
/// "inf" with its sign, "NAN" and "INF" under DTOSTR_UPPERCASE. At most
/// fifteen characters and the NUL.
inline char* dtostre(double value, char* buffer, unsigned char precision,
                     unsigned char flags) {
    DoubleSplit split(value);
    return dtostre_split(split, buffer, precision, flags);
}

/// ftostre(), avr-libc's float twin of dtostre(), as ftostrf() is
/// dtostrf()'s.
inline char* ftostre(float value, char* buffer, unsigned char precision, unsigned char flags) {
    DoubleSplit split(value);
    return dtostre_split(split, buffer, precision, flags);
}
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
/// Hexadecimal integer with 0x prefix and no leading zeros:
/// print(s, hex(0xBEEF)) -> "0xBEEF"
inline constexpr Hex hex(uint32_t value) { return {value}; }

struct Hex64 { uint64_t value; };
/// The same for a 64-bit integer - up to sixteen digits, so it is never
/// cut to its low 32 bits: print(s, hex(uint64_t{1} << 40)) ->
/// "0x10000000000". A template taking 64-bit types alone, so a narrower
/// argument (an int literal among them) still finds hex(uint32_t) and
/// nothing else.
template <std::integral I>
    requires(sizeof(I) == 8u)
inline constexpr Hex64 hex(I value) {
    return {static_cast<uint64_t>(value)};
}

// A value travels AS WHAT IT IS: a float as a float, a double as a
// double. There is no C-varargs promotion here (a template pack deduces
// exact types), so a float reaches ftostrf()/ftostre(), split from its
// own bits with no widening call, and a double reaches dtostrf()/
// dtostre() with every digit it has. On avr-gcc double IS float (32-bit)
// by default, so on the AVR both names are the one avr-libc function. The
// assert below fires if that toolchain is ever rebuilt with -mdouble=64:
// avr-libc's double conversions are then 64-bit, and this file must be
// revisited consciously. (Checked via the predefined macros, not sizeof:
// the IDE language server parses with a host-like data model where
// sizeof(double) is 8 and would flag a sizeof-based assert as failed,
// while macros are queried from the real avr-g++ and evaluate correctly in
// both worlds.)
#if defined(__AVR__)
static_assert(__SIZEOF_DOUBLE__ == __SIZEOF_FLOAT__,
              "toolchain built with -mdouble=64: revisit print.hpp float handling");
#endif

/// The type a value is printed as: a float stays a float, and any other
/// arithmetic type - a double, a long double, an integer handed to
/// fixed() - is printed as a double.
template <typename T>
using PrintFloat = std::conditional_t<std::is_same_v<T, float>, float, double>;

template <typename T>
struct Fixed { T value; int8_t width; uint8_t precision; };
/// Fixed-point: print(s, fixed(3.14159f, 8, 3)) -> "   3.142" (dtostrf()).
/// A negative width aligns left; the precision is kept to
/// print_fixed_precision_max and the width to the text's buffer.
template <typename T>
    requires std::is_arithmetic_v<T>
inline constexpr Fixed<PrintFloat<T>> fixed(T value, int8_t width, uint8_t precision) {
    return {static_cast<PrintFloat<T>>(value), width, precision};
}

template <typename T>
struct Sci { T value; uint8_t precision; };
/// Scientific, signed: print(s, sci(0.00123f, 3)) -> "+1.230e-03"
/// (dtostre() under DTOSTR_PLUS_SIGN; the precision kept to 7).
template <typename T>
    requires std::is_arithmetic_v<T>
inline constexpr Sci<PrintFloat<T>> sci(T value, uint8_t precision = 3) {
    return {static_cast<PrintFloat<T>>(value), precision};
}

/// The most decimals fixed() prints, and the buffer its text is built in:
/// a sign, the 39 integer digits of the largest float (and of a double
/// just below 2^128), the point, the decimals and the NUL.
inline constexpr uint8_t print_fixed_precision_max = 18;
inline constexpr uint8_t print_fixed_text = 1 + 39 + 1 + print_fixed_precision_max + 1;

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

/// The high word without its leading zeros, then the low word in all
/// eight of its digits - both through ultoa(), so the letters are the
/// target's as hex(uint32_t)'s are; under 2^32 it prints as hex(uint32_t).
template <ByteSink S>
inline void print_one(S s, Hex64 h) {
    const uint32_t high = static_cast<uint32_t>(h.value >> 32);
    const uint32_t low = static_cast<uint32_t>(h.value);
    if (high == 0u) {
        print_one(s, Hex{low});
        return;
    }
    char buffer[19];   // "0x", sixteen digits, the NUL
    buffer[0] = '0';
    buffer[1] = 'x';
    ultoa(high, &buffer[2], 16);
    uint8_t n = 2;
    while (buffer[n] != '\0') {
        ++n;
    }
    char low_text[9];
    ultoa(low, low_text, 16);
    uint8_t m = 0;
    while (low_text[m] != '\0') {
        ++m;
    }
    for (uint8_t i = m; i < 8u; ++i) {
        buffer[n++] = '0';
    }
    for (uint8_t i = 0; i < m; ++i) {
        buffer[n++] = low_text[i];
    }
    buffer[n] = '\0';
    print_one(s, static_cast<const char *>(buffer));
}

template <ByteSink S, typename T>
inline void print_one(S s, Fixed<T> f) {
    constexpr int8_t widest = static_cast<int8_t>(print_fixed_text - 1);
    char buffer[print_fixed_text];
    const int8_t width = f.width > widest ? widest : (f.width < -widest ? static_cast<int8_t>(-widest) : f.width);
    const uint8_t precision = f.precision > print_fixed_precision_max ? print_fixed_precision_max : f.precision;
    if constexpr (std::is_same_v<T, float>) {
        ftostrf(f.value, width, precision, buffer);
    } else {
        dtostrf(f.value, width, precision, buffer);
    }
    print_one(s, static_cast<const char *>(buffer));
}

template <ByteSink S, typename T>
inline void print_one(S s, Sci<T> e) {
    char buffer[16];
    const uint8_t precision = (e.precision > 7) ? uint8_t{7} : e.precision;
    if constexpr (std::is_same_v<T, float>) {
        ftostre(e.value, buffer, precision, DTOSTR_PLUS_SIGN);
    } else {
        dtostre(e.value, buffer, precision, DTOSTR_PLUS_SIGN);
    }
    print_one(s, static_cast<const char *>(buffer));
}

template <ByteSink S, std::floating_point F>
inline void print_one(S s, F value) {
    print_one(s, Sci<PrintFloat<F>>{static_cast<PrintFloat<F>>(value), 3});
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
