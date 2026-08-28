/*
 * crc.hpp
 *
 * One checksum, bitwise: CRC-16/CCITT-FALSE (polynomial 0x1021,
 * initial value 0xFFFF, no reflection, no final XOR).
 *
 * Bitwise on purpose. A table would cost 512 bytes of flash to save a
 * few microseconds on payloads that are tens of bytes long and are
 * written to nonvolatile memory at 70 us PER BYTE - the checksum is
 * never the slow part of anything that uses it. The loop is constexpr,
 * so a compile-time constant payload costs nothing at all.
 *
 * The one property the users here depend on: a torn write (power lost
 * halfway through a multi-byte record) shows up as a mismatch, so the
 * reader can refuse the record instead of returning half of it.
 */

#pragma once

#include <stdint.h>

namespace brio {

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

} // namespace brio
