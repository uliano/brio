/*
 * nv_record.hpp
 *
 * A typed value that survives a power cycle: the byte-granular
 * nonvolatile store as a CONCEPT, and one record layout over it.
 *
 * Why a concept and not a driver call: the store is the only
 * target-specific thing here (on AVR DA/DB it is the EEPROM behind
 * avrdx/nvm.hpp; on the host it is an array), while the layout, the
 * validity rule and the write policy are pure logic and are tested on
 * the host. util/ never includes a target.
 *
 * THE WRITE POLICY IS THE POINT. A nonvolatile byte has a finite number
 * of erase/write cycles (100k on this silicon's EEPROM), so store()
 * READS FIRST and writes only the bytes that actually differ. Storing
 * the same value twice costs zero cycles of endurance; changing one
 * field of a struct costs the bytes of that field plus the two checksum
 * bytes. A record that is rewritten every second lives for days if it
 * is written blindly and for years if it is written this way.
 *
 * VALIDITY. Four header bytes precede the payload: a magic byte, a
 * version byte, and the CRC-16 of the payload. load() returns the value
 * only when all three agree - an erased store (0xFF everywhere), a
 * record written by an older layout, and a write torn by a power loss
 * are all indistinguishable from "no value", which is exactly what the
 * caller can act on. The magic is deliberately NOT rewritten around
 * each store: the checksum already catches a tear, and rewriting the
 * magic twice per store would wear one byte at twice the rate of the
 * payload it guards.
 *
 * NOT BUILT: wear levelling, journaling, more than one copy. Those are
 * policies that need a user with real numbers behind them; the
 * mechanism here is the floor they would be built on.
 */

#pragma once

#include <stdint.h>

#include <concepts>
#include <optional>
#include <type_traits>

#include "util/crc.hpp"

namespace brio {

/**
 * A byte-granular nonvolatile store.
 *
 *  - size()          bytes addressable, offsets are 0 .. size()-1;
 *  - read(a)         the byte at offset a (never fails: a read of a
 *                    never-written store returns the erased pattern);
 *  - write(a, b)     START writing one byte; false = refused (bad
 *                    offset, or the store was still busy). The write
 *                    may still be in flight when this returns;
 *  - ready()         no write/erase is in flight;
 *  - wait_ready()    bounded wait for that; false = it never came;
 *  - finish()        release whatever the store armed for the writes
 *                    (on this silicon: clear the NVMCTRL command).
 *
 * Every verb is static: a store is a piece of hardware, not an object.
 */
template <typename S>
concept NvStore = requires(uint16_t addr, uint8_t byte) {
    { S::size() } -> std::convertible_to<uint16_t>;
    { S::read(addr) } -> std::convertible_to<uint8_t>;
    { S::write(addr, byte) } -> std::convertible_to<bool>;
    { S::ready() } -> std::convertible_to<bool>;
    { S::wait_ready() } -> std::convertible_to<bool>;
    { S::finish() };
};

/**
 * A store that can also announce readiness with an interrupt, which is
 * what lets a writer run without ever blocking (util/nv_writer.hpp).
 *
 *  - arm_ready_interrupt(on)  enable/disable the ready interrupt;
 *  - ready_flag()             the interrupt flag itself;
 *  - clear_ready_flag()       clear it.
 */
template <typename S>
concept NvPacedStore = NvStore<S> && requires(bool on) {
    { S::arm_ready_interrupt(on) };
    { S::ready_flag() } -> std::convertible_to<bool>;
    { S::clear_ready_flag() };
};

/// The magic byte at the head of every record. Any value would do; this
/// one is not 0x00 and not the 0xFF an erased store reads back.
inline constexpr uint8_t nv_record_magic = 0xB2;

/// Bytes a record occupies in the store: the four-byte header plus the
/// payload.
template <typename T>
inline constexpr uint16_t nv_record_size =
    static_cast<uint16_t>(4u + sizeof(T));

/**
 * One typed record at a fixed offset of a store.
 *
 * T must be trivially copyable: the record IS its object representation,
 * byte for byte, and nothing is serialized. `version` travels in the
 * header, so changing T's layout and bumping the version makes every
 * older record read back as "no value" instead of as garbage.
 */
template <typename T, NvStore S, uint16_t offset = 0, uint8_t version = 1>
struct NvRecord {
    static_assert(std::is_trivially_copyable_v<T>,
                  "a record is stored as its object representation");
    static_assert(nv_record_size<T> >= 4, "an empty payload has no record");

    NvRecord() = delete;

    static constexpr uint16_t begin = offset;
    static constexpr uint16_t end =
        static_cast<uint16_t>(offset + nv_record_size<T>);

    /// The stored value, or nothing when the store holds no valid record
    /// of this version at this offset (erased, older version, or a write
    /// torn by a power loss).
    static std::optional<T> load() {
        if (!fits()) {
            return std::nullopt;
        }
        if (S::read(offset) != nv_record_magic ||
            S::read(static_cast<uint16_t>(offset + 1)) != version) {
            return std::nullopt;
        }
        T value{};
        uint8_t* const bytes = reinterpret_cast<uint8_t*>(&value);
        uint16_t crc = 0xFFFFu;
        for (uint16_t i = 0; i < sizeof(T); ++i) {
            const uint8_t b = S::read(static_cast<uint16_t>(offset + 4 + i));
            bytes[i] = b;
            crc = crc16_byte(crc, b);
        }
        const uint16_t stored = static_cast<uint16_t>(
            S::read(static_cast<uint16_t>(offset + 2)) |
            (static_cast<uint16_t>(S::read(static_cast<uint16_t>(offset + 3))) << 8));
        if (stored != crc) {
            return std::nullopt;
        }
        return value;
    }

    /// True when a valid record of this version is present.
    static bool valid() { return load().has_value(); }

    /// Write the value, touching ONLY the bytes that differ from what is
    /// already stored. Returns the number of bytes actually written -
    /// zero when the store already held exactly this value, which is the
    /// endurance the whole design is for. A store that refuses a byte or
    /// never becomes ready ends the run: the count returned is what got
    /// through, and load() will report the record invalid.
    static uint16_t store(const T& value) {
        if (!fits()) {
            return 0;
        }
        uint8_t image[nv_record_size<T>];
        build(value, image);
        uint16_t written = 0;
        for (uint16_t i = 0; i < nv_record_size<T>; ++i) {
            const uint16_t addr = static_cast<uint16_t>(offset + i);
            if (S::read(addr) == image[i]) {
                continue;
            }
            if (!S::wait_ready() || !S::write(addr, image[i])) {
                break;
            }
            ++written;
        }
        (void)S::wait_ready();
        S::finish();
        return written;
    }

    /// Invalidate the record with one byte write: the magic goes to the
    /// erased pattern and load() reports nothing from here on. The
    /// payload is left as it is - this is a "consume it" verb, not a
    /// scrub.
    static bool clear() {
        if (!fits()) {
            return false;
        }
        if (S::read(offset) == 0xFFu) {
            return true;
        }
        const bool ok = S::wait_ready() && S::write(offset, 0xFFu);
        const bool settled = S::wait_ready();
        S::finish();
        return ok && settled;
    }

    /// The record's bytes as they would be stored. Exposed so a caller
    /// that must write from a context where reads are unsafe (a panic
    /// reporter) can hand the image to the store itself.
    static void build(const T& value, uint8_t (&image)[nv_record_size<T>]) {
        const uint8_t* const bytes = reinterpret_cast<const uint8_t*>(&value);
        uint16_t crc = 0xFFFFu;
        for (uint16_t i = 0; i < sizeof(T); ++i) {
            image[4 + i] = bytes[i];
            crc = crc16_byte(crc, bytes[i]);
        }
        image[0] = nv_record_magic;
        image[1] = version;
        image[2] = static_cast<uint8_t>(crc & 0xFFu);
        image[3] = static_cast<uint8_t>(crc >> 8);
    }

private:
    /// The store may be smaller than the record's offset asks for. Not
    /// constexpr: a store's size() is a fact of the silicon it stands
    /// for, and the concept does not demand a compile-time one.
    static bool fits() {
        return static_cast<uint32_t>(offset) + nv_record_size<T> <=
               static_cast<uint32_t>(S::size());
    }
};

} // namespace brio
