/*
 * wire.hpp
 *
 * Wire byte-order helpers: a device driver's conversions between the
 * multibyte words a bus device speaks and the bytes a transaction
 * carries. SPI (and most serial buses) ship multibyte words MSB-first -
 * big-endian on the wire - while the targets brio runs on are
 * little-endian. Every device driver that speaks 16/24/32-bit words
 * (register-based controllers, DACs, delta-sigma ADCs with 24-bit
 * channel groups) needs the same handful of conversions, so they live
 * here once, as pure constexpr functions over raw bytes. The smallest
 * use packs a 16-bit DAC code into the bytes after a command byte, and
 * reads a 24-bit ADC word out of a reply:
 *
 *     brio::store_be16(&i2c_tx[1], dac_code);
 *     const uint32_t raw = brio::load_be24(spi_rx);
 *
 * Each function reads or writes the caller's bytes during the call and
 * keeps nothing; nothing waits or masks, so any context may call them.
 * The buffer rule is the bus's, not this header's: a transaction's
 * buffers are lent to the bus from the post to its completion event
 * (docs/design/spi-bus.md, "The transaction descriptor
 * (`SpiHost<n>::Request`)"), so a client fills the transmit bytes
 * before it posts and reads the reply's after the completion arrives.
 *
 * This is not an engine feature: the bus engines move bytes and do not
 * format (docs/design/overview.md, "Design pillars": drivers move bytes;
 * they do not format). A word-based transaction is a byte span on the
 * wire; the client converts it at the edges with these helpers. 24-bit
 * ADC words come signed in two's complement: load_be24_signed()
 * sign-extends into int32_t (C++20 guarantees arithmetic right shift on
 * signed values).
 *
 * The pointer points at the word's first (most significant) wire byte;
 * callers index their rx/tx buffers by word stride themselves - a
 * frame of four 24-bit groups is p, p+3, p+6, p+9.
 */

#pragma once

#include <stdint.h>

namespace brio {

/// The unsigned 16-, 24- and 32-bit word whose most significant byte is
/// at p, read from 2, 3 or 4 bytes.
constexpr uint16_t load_be16(const uint8_t* p) {
    return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) << 8 | p[1]);
}

constexpr uint32_t load_be24(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) << 16 |
           static_cast<uint32_t>(p[1]) << 8 | p[2];
}

constexpr uint32_t load_be32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) << 24 |
           static_cast<uint32_t>(p[1]) << 16 |
           static_cast<uint32_t>(p[2]) << 8 | p[3];
}

/// 24-bit two's-complement word (delta-sigma ADC channel data),
/// sign-extended to int32_t.
constexpr int32_t load_be24_signed(const uint8_t* p) {
    return static_cast<int32_t>(load_be24(p) << 8) >> 8;
}

/// v written at p most significant byte first, in 2, 3 or 4 bytes
/// (store_be24 writes the low 24 bits of v).
constexpr void store_be16(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v >> 8);
    p[1] = static_cast<uint8_t>(v);
}

constexpr void store_be24(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v >> 16);
    p[1] = static_cast<uint8_t>(v >> 8);
    p[2] = static_cast<uint8_t>(v);
}

constexpr void store_be32(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v >> 24);
    p[1] = static_cast<uint8_t>(v >> 16);
    p[2] = static_cast<uint8_t>(v >> 8);
    p[3] = static_cast<uint8_t>(v);
}

} // namespace brio
