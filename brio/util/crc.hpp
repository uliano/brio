/*
 * crc.hpp
 *
 * Two checksums, both bitwise and both constexpr:
 *
 *   CRC-16/CCITT-FALSE  polynomial 0x1021, initial value 0xFFFF, no
 *                       reflection, no final XOR - what a stored record
 *                       carries so that half of it can be refused;
 *   CRC-32/MPEG-2       polynomial 0x04C11DB7 (the CRC-32 of Ethernet),
 *                       initial value 0xFFFFFFFF, most significant bit
 *                       first, no reflection, no final inversion.
 *
 * Bitwise on purpose. A table would cost 512 bytes of flash to save a
 * few microseconds on payloads that are tens of bytes long and are
 * written to nonvolatile memory at 70 us PER BYTE - the checksum is
 * never the slow part of anything that uses it. The loop is constexpr,
 * so a compile-time constant payload costs nothing at all.
 *
 * WHY THE CRC-32 IS HERE AND NOT IN A DRIVER. More than one of brio's
 * targets carries a hardware CRC block with the Ethernet polynomial
 * WIRED IN and no knob beside it: no polynomial register, no initial
 * value, no reversal, no final XOR. What such a block computes is
 * therefore exactly one function - the one catalogued as CRC-32/MPEG-2 -
 * and one function has one home. The software twin below is what each of
 * those blocks' bench suites judges the silicon against, what a program
 * uses for a length the hardware's word grain cannot take, and what
 * answers at compile time for a constant.
 *
 * The one property the CRC-16 users depend on: a torn write (power lost
 * halfway through a multi-byte record) shows up as a mismatch, so the
 * reader can refuse the record instead of returning half of it.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace brio {

// ---- CRC-16/CCITT-FALSE -----------------------------------------------------

/// Feed one byte into a running CRC-16/CCITT-FALSE.
constexpr uint16_t crc16_byte(uint16_t crc, uint8_t byte) {
    crc = static_cast<uint16_t>(crc ^ (static_cast<uint16_t>(byte) << 8));
    for (uint8_t bit = 0; bit < 8; ++bit) {
        crc = (crc & 0x8000u) != 0
                  ? static_cast<uint16_t>(static_cast<uint16_t>(crc << 1) ^ 0x1021u)
                  : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

/// CRC-16/CCITT-FALSE over a byte range. The empty range is 0xFFFF.
constexpr uint16_t crc16(const uint8_t* data, uint16_t len,
                         uint16_t seed = 0xFFFFu) {
    uint16_t crc = seed;
    for (uint16_t i = 0; i < len; ++i) {
        crc = crc16_byte(crc, data[i]);
    }
    return crc;
}

// ---- CRC-32/MPEG-2 ----------------------------------------------------------

/// The generator of the Ethernet CRC-32: x^32 + x^26 + x^23 + x^22 +
/// x^16 + x^12 + x^11 + x^10 + x^8 + x^7 + x^5 + x^4 + x^2 + x + 1.
inline constexpr uint32_t crc32_ethernet_poly = 0x04C11DB7u;

/// The value every computation starts from - and what the data register
/// of a hardware block of this kind holds after its reset.
inline constexpr uint32_t crc32_ethernet_init = 0xFFFFFFFFu;

/**
 * One byte into a running CRC-32, most significant bit first, no
 * reflection - the primitive a word-wise hardware operation is built out
 * of. Bitwise and table-free for the reason the CRC-16 above is: eight
 * shifts a byte, and a 1 KB table would cost more than it saves in a
 * program that has a hardware unit for the bulk of it.
 */
constexpr uint32_t crc32_ethernet_byte(uint32_t crc, uint8_t byte) {
    crc ^= static_cast<uint32_t>(byte) << 24;
    for (uint8_t i = 0; i < 8; ++i) {
        crc = (crc & 0x80000000u) != 0u ? ((crc << 1) ^ crc32_ethernet_poly) : (crc << 1);
    }
    return crc;
}

/// One 32-bit word into a running CRC, most significant BYTE first: the
/// order in which a hardware block of this kind consumes a word written
/// to its data register, so feeding `w` here and storing `w` there are
/// the same operation.
constexpr uint32_t crc32_ethernet_word(uint32_t crc, uint32_t word) {
    crc = crc32_ethernet_byte(crc, static_cast<uint8_t>(word >> 24));
    crc = crc32_ethernet_byte(crc, static_cast<uint8_t>(word >> 16));
    crc = crc32_ethernet_byte(crc, static_cast<uint8_t>(word >> 8));
    return crc32_ethernet_byte(crc, static_cast<uint8_t>(word));
}

/// A run of words from the initial value - the whole checksum in
/// software, for a constant known at compile time, for a program whose
/// block is busy with something else, and for a bench suite to judge the
/// silicon against.
constexpr uint32_t crc32_ethernet(const uint32_t* words, size_t count) {
    uint32_t crc = crc32_ethernet_init;
    for (size_t i = 0; i < count; ++i) {
        crc = crc32_ethernet_word(crc, words[i]);
    }
    return crc;
}

/// A run of BYTES in software, for the length a word-wise unit cannot
/// take (anything that is not a whole number of words) and for the
/// published check value an implementation is pinned to.
constexpr uint32_t crc32_ethernet_bytes(const uint8_t* bytes, size_t count) {
    uint32_t crc = crc32_ethernet_init;
    for (size_t i = 0; i < count; ++i) {
        crc = crc32_ethernet_byte(crc, bytes[i]);
    }
    return crc;
}

} // namespace brio
