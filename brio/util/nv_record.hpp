/*
 * nv_record.hpp
 *
 * A typed value that survives a power cycle: the byte-granular
 * nonvolatile store as a concept, and one record layout over it. A
 * record is one typed value at a fixed offset, checked by its version
 * byte and its CRC when loaded, updated in place with no second
 * committed copy, each store touching only the bytes that changed. An
 * update cut short (a refusal, a time-out, a power loss) that reached
 * the payload or the checksum leaves a record load() reports as "no
 * value", as an erased store and one written by an older layout are; one
 * that reached nothing leaves the previous record standing. Two tears
 * revive what should be gone: after clear(), which erases the magic byte
 * alone, a store cut short right after that byte makes the PREVIOUS
 * value valid again; and a store of a new version over an old record of
 * the same size, cut short right after the version byte, leaves the old
 * payload under the new version with the old checksum still matching,
 * so load() returns old-layout bytes as the new type - store()'s comment
 * has both sequences.
 *
 * The smallest use names the value's type, a store and an offset, then
 * loads and stores it:
 *
 *     struct Settings { uint16_t rate; uint8_t mode; uint8_t flags; };
 *     using SettingsRecord = brio::NvRecord<Settings, brio::EepromStore, 16>;
 *
 *     if (const std::optional<Settings> s = SettingsRecord::load()) { ... }
 *     SettingsRecord::store(Settings{9600, 1, 0});   // 0 bytes if unchanged
 *
 * EepromStore is the AVR's EEPROM as a store (avrxt/nvm.hpp); the host
 * suite runs the same record over an array (test/test_nv_record).
 *
 * Who calls what, and from where. NvRecord is a set of synchronous
 * static functions with no state of its own and no critical section.
 * load() and valid() only read. store() and clear() wait: before each
 * operation and after the last one they poll the store's bounded
 * wait_ready(), so the call returns once the last operation is over, the
 * store refused one, or a bounded wait gave up. The wait is one
 * operation per differing byte on a byte store and one per write unit
 * touched on a store that takes a run (below); on the AVR EA the
 * eight-byte panic record is one operation of 2.73 ms on the run path
 * where the byte path takes eight, 21.4 ms (docs/design/nv-journal.md,
 * "The EEPROM spelling's store: a byte, and a run where the silicon
 * writes one"). A caller that must not wait that long uses the writer
 * active object instead (util/nv_writer.hpp), which starts one operation
 * per dispatch and waits for nothing. The value passed to store() is
 * copied into a local image before the first write; the caller's object
 * is not kept.
 *
 * Why a concept and not a driver call: the store is the only
 * target-specific thing here (on silicon with EEPROM it is the EEPROM
 * behind that target's nvm driver; on the host it is an array), while
 * the layout, the validity rule and the write policy are pure logic and
 * are tested on the host. util/ never includes a target. (A concept is
 * a compiler-checked list of what a type must provide; NvStore below is
 * that list for a store.)
 *
 * The write policy is the point. A nonvolatile byte has a finite number
 * of erase/write cycles (100k on a typical EEPROM), so store() reads
 * first and writes only the bytes that actually differ. Storing the
 * same value twice costs zero cycles of endurance; changing one field
 * of a struct costs the bytes of that field plus the two checksum
 * bytes. What that buys depends on the workload: a value stored again
 * unchanged costs nothing, a changing field costs its own bytes and the
 * checksum's two every time, so the checksum bytes wear at the rate the
 * value changes, and a lifetime figure is the store's rated endurance
 * against that rate - not a property of the record.
 *
 * Validity. Four header bytes precede the payload: a magic byte, a
 * version byte, and the CRC-16 of the payload. load() returns the value
 * only when all three agree - an erased store (0xFF everywhere), a
 * record written by an older layout, and a write torn by a power loss
 * are all indistinguishable from "no value", which is exactly what the
 * caller can act on - with the two exceptions above: a store over a
 * cleared record writes the magic first, so a tear right after it
 * revives the old payload and its old checksum, which still agree, and
 * a store of a new version over a same-sized old record writes the
 * version before the checksum, so a tear between them leaves the old
 * payload readable as the new type. The
 * magic is deliberately not rewritten around each store: the checksum
 * already catches a tear inside the payload, and rewriting the magic
 * twice per store would wear one byte at twice the rate of the payload
 * it guards.
 *
 * The run. A run is a stretch of neighbouring bytes. A store whose
 * silicon writes several of them in one operation (a page buffer) says
 * so with two more verbs - the NvRunStore refinement below - and
 * store() then writes a run per operation instead of a byte: the same
 * bytes end up stored and the same count is returned, in as many
 * operations as the record touches write units. A store without them
 * is served a byte at a time. The choice is made at compile time from
 * the store's type; nothing is asked at run time.
 *
 * Not built: wear levelling, journaling, more than one copy. Those are
 * policies that need a user with real numbers behind them; the
 * mechanism here is the floor they would be built on (the flash
 * journal, util/nv_journal.hpp, is a different spelling for a part with
 * no EEPROM; docs/design/nv-journal.md, "The two spellings, and why
 * they stay apart").
 */

#pragma once

#include <stdint.h>

#include <concepts>
#include <optional>
#include <type_traits>

#include "util/crc.hpp"

namespace brio {

/**
 * A byte-granular nonvolatile store: what NvRecord, NvWriter and
 * PersistentPanic stand on.
 *
 *  - size()          bytes addressable, offsets are 0 .. size()-1;
 *  - read(a)         the byte at offset a (never fails: a read of a
 *                    never-written store returns the erased pattern);
 *  - write(a, b)     start writing one byte; false = refused (bad
 *                    offset, or the store was still busy). The write
 *                    may still be in flight when this returns;
 *  - ready()         no write/erase is in flight;
 *  - wait_ready()    bounded wait for that; false = it never came;
 *  - finish()        release whatever the store armed for the writes
 *                    (typically: clear the flash controller's command).
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

/**
 * A store that writes a run in one operation: the contract's optional
 * verb, offered by a store whose silicon has a page buffer and taken by
 * every writer here when it is offered (a contract's unit is the run;
 * the byte is its degenerate case).
 *
 *  - run_room(a)          the bytes from offset a to the end of the
 *                         store's write unit that holds it: the longest
 *                         run one operation writes from a. At least 1
 *                         for every a < size(), never past size();
 *  - write_run(a, src, n) start one operation that leaves src[0..n) in
 *                         [a, a + n), 1 <= n <= run_room(a). Bytes that
 *                         already hold their value may be left alone (a
 *                         store whose silicon marks bytes does). False =
 *                         refused, nothing started: a run past its room
 *                         or the store, a protected store, or one that
 *                         never became idle. The operation may still be
 *                         in flight when this returns, as for write().
 *
 * The byte verbs keep their meaning: write(a, b) is a run of one, and a
 * caller may mix the two. A run never crosses the room the store
 * reports - the writers below cut every run at it.
 */
template <typename S>
concept NvRunStore = NvStore<S> &&
    requires(uint16_t addr, const uint8_t* src, uint16_t len) {
        { S::run_room(addr) } -> std::convertible_to<uint16_t>;
        { S::write_run(addr, src, len) } -> std::convertible_to<bool>;
    };

/// What a writer scanning an image for its next run found: the run
/// [from, from + len) of the image (len 0 = nothing left differs), the
/// bytes in it that differ from the store, and where the scan resumes.
struct NvRunScan {
    uint16_t from;
    uint16_t len;
    uint16_t changed;
    uint16_t resume;
};

/**
 * The next run of image[i..n) to write at store offset `base`: the first
 * byte that differs from the store, and the last one that differs
 * within that byte's room - so the run carries no equal byte at either
 * end, and never more than one operation's worth. It reads the store:
 * call it with the store idle (a read during an operation can be a bus
 * wait that holds the CPU and its interrupts to the operation's end).
 */
template <NvRunStore S>
NvRunScan nv_scan_run(uint16_t base, const uint8_t* image, uint16_t i, uint16_t n) {
    while (i < n && S::read(static_cast<uint16_t>(base + i)) == image[i]) {
        ++i;
    }
    if (i == n) {
        return NvRunScan{n, 0, 0, n};
    }
    const uint16_t room = static_cast<uint16_t>(S::run_room(static_cast<uint16_t>(base + i)));
    const uint16_t window = room < static_cast<uint16_t>(n - i)
                                ? room
                                : static_cast<uint16_t>(n - i);
    uint16_t last = 0;
    uint16_t changed = 1;   // byte i itself
    for (uint16_t k = 1; k < window; ++k) {
        const uint16_t j = static_cast<uint16_t>(i + k);
        if (S::read(static_cast<uint16_t>(base + j)) != image[j]) {
            last = k;
            ++changed;
        }
    }
    return NvRunScan{i, static_cast<uint16_t>(last + 1u), changed,
                     static_cast<uint16_t>(i + window)};
}

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
 * T must be trivially copyable: the record is its object representation,
 * byte for byte, and nothing is serialized. `version` travels in the
 * header, so changing T's layout and bumping the version makes every
 * older record read back as "no value" instead of as garbage - but for
 * a store of the new version over it cut short right after its version
 * byte, which leaves the old payload readable as the new type (store()'s
 * comment). A monostate:
 * every verb is static and nothing is constructed (one record is one
 * place in one store, named by its type). `offset` is where its
 * nv_record_size<T> bytes live; nothing checks that two records or a
 * PersistentPanic do not overlap.
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
    /// torn by a power loss) - or, after one of the two torn-write
    /// sequences store()'s comment records, a previous value as if it
    /// were current. Reads only; a store too small for the record's
    /// offset answers nothing as well.
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

    /// Write the value, touching only the bytes that differ from what is
    /// already stored, and return when the last write is over. Returns
    /// the number of bytes actually written - zero when the store
    /// already held exactly this value, which is the endurance the whole
    /// design is for. A store that refuses a byte or never becomes ready
    /// ends the call: the count returned is what got through. When
    /// nothing got through the previous record stands. The bytes go
    /// down in address order, the magic first; over a valid record a
    /// call ended part-way (a refusal, a time-out, a power loss) leaves
    /// a checksum that no longer matches, and load() reports no value.
    /// Over a record clear() invalidated, whose payload and checksum
    /// clear() left in place, a call ended right after the magic (on a
    /// byte store, or a run store whose first run is the magic alone)
    /// makes the OLD record valid again: PersistentPanic's take() then a
    /// report() cut there brings back the failure take() consumed. Over
    /// a valid record of an OLDER version with the same size, a call
    /// ended right after the version byte (the checksum comes later in
    /// address order) leaves the old payload and its still-matching
    /// checksum under the new version, and load() returns those bytes as
    /// the new type.
    /// Zero, writing nothing, when the store is too small for the
    /// record's offset.
    ///
    /// On an NvRunStore the differing bytes go a run per operation, and
    /// every read waits for the store to be idle first; on a byte store
    /// each differing byte is an operation of its own.
    static uint16_t store(const T& value) {
        if (!fits()) {
            return 0;
        }
        uint8_t image[nv_record_size<T>];
        build(value, image);
        uint16_t written = 0;
        if constexpr (NvRunStore<S>) {
            uint16_t i = 0;
            while (i < nv_record_size<T>) {
                if (!S::wait_ready()) {
                    break;
                }
                const NvRunScan r = nv_scan_run<S>(offset, image, i, nv_record_size<T>);
                if (r.len == 0) {
                    break;
                }
                if (!S::write_run(static_cast<uint16_t>(offset + r.from),
                                  image + r.from, r.len)) {
                    break;
                }
                written = static_cast<uint16_t>(written + r.changed);
                i = r.resume;
            }
            (void)S::wait_ready();
            S::finish();
            return written;
        }
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
    /// payload and its checksum are left as they are - this is a
    /// "consume it" verb, not a scrub, and that is its limit: the next
    /// store() writes the magic first, so a store cut short right after
    /// that byte makes this record valid again (store()'s comment). A
    /// consume that must hold through any later tear needs a scrub of
    /// the checksum too, which nothing here does. Waits for the write
    /// like store(); true when the magic is
    /// erased (or already was), false when the store refused or never
    /// became ready.
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
