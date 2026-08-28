/*
 * wire.hpp
 *
 * Wire byte-order helpers: SPI (and most serial buses) ship multibyte
 * words MSB-first - big-endian on the wire - while the targets brio
 * runs on are little-endian. Every device driver that speaks 16/24/32
 * bit words (register-based controllers, DACs, delta-sigma ADCs with
 * 24-bit channel groups) needs the same handful of conversions, so
 * they live here once, as pure constexpr functions over raw bytes.
 *
 * This is deliberately NOT an engine feature: the bus engines move
 * bytes and do not format ("drivers move bytes" pillar). A word-based
 * transaction is a byte span on the wire; the client massages it at
 * the edges with these helpers. 24-bit ADC words come signed in two's
 * complement: load_be24_signed() sign-extends into int32_t (C++20
 * guarantees arithmetic right shift on signed values).
 *
 * The pointer points at the word's first (most significant) wire byte;
 * callers index their rx/tx buffers by word stride themselves - a
 * frame of four 24-bit groups is p, p+3, p+6, p+9.
 */

#pragma once

#include <stdint.h>

namespace brio {

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
